#!/usr/bin/env python
# Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import sys, os, time, signal, threading
import numpy # nice to be able to explicitly call some functions
from numpy import *
from optparse import OptionParser
from openravepy import *
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.databases import grasping, visibilitymodel

try:
    from Tkinter import *
    import tkFileDialog
    import Image, ImageDraw, ImageTk
except ImportError:
    pass

class CameraViewerGUI(threading.Thread):
    class Container:
        pass
    def __init__(self,sensor,title='Camera Viewer'):
        threading.Thread.__init__(self)
        self.sensor = sensor
        self.title = title
        self.laststamp = None
        self.imagelck = threading.Lock()

    def updateimage(self):
        data = self.sensor.GetSensorData()
        if data is not None and not self.laststamp == data.stamp:
            width = data.imagedata.shape[1]
            height = data.imagedata.shape[0]
            self.imagelck.acquire()
            self.image = Image.frombuffer('RGB',[width,height], data.imagedata.tostring(), 'raw','RGB',0,1)
            self.imagelck.release()

            photo = ImageTk.PhotoImage(self.image)
            if self.container is None:
                self.container = self.Container()
                self.container.width = width
                self.container.height = height
                self.container.main = self.main
                self.container.canvas = Canvas(self.main, width=width, height=height)
                self.container.canvas.pack(expand=1, fill=BOTH)#side=TOP,fill=X)#
                self.container.obr = None

            self.container.canvas.create_image(self.container.width/2, self.container.height/2, image=photo)
            self.container.obr = photo
            self.laststamp = data.stamp
        self.main.after(100,self.updateimage)

    def saveimage(self,filename):
        self.imagelck.acquire()
        self.image.save(filename)
        self.imagelck.release()

    def run(self):
        self.main = Tk()
        self.main.title(self.title)      # window title
        self.main.resizable(width=True, height=True)
        self.container = None
        self.main.after(0,self.updateimage)
        self.main.mainloop()

class VisibilityGrasping(metaclass.AutoReloader):
    """Calls on the openrave grasp planners to get a robot to pick up objects while guaranteeing visibility with its cameras"""
    def __init__(self):
        self.orenvreal = Environment()
        self.orenvreal.SetViewer('qtcoin')
        self.trajectorylog = []
        self.graspoffset = 0

    def loadscene(self,scenefilename,sensorname,robotname=None,showsensors=True,usecameraview=True):
        self.target = None
        self.robot = None
        self.robotreal = None
        self.orenvreal.Reset()
        if not self.orenvreal.Load(scenefilename):
            raise ValueError('failed to open %s openrave file'%scenefilename)
        if robotname is None:
            self.robotreal = self.orenvreal.GetRobots()[0]
        else:
            self.robotreal = [r for r in self.orenvreal.GetRobots() if r.GetName()==robotname][0]

        with self.orenvreal:
            self.basemanip = BaseManipulation(self.robotreal)
            self.homevalues = self.robotreal.GetDOFValues()
            # create a camera viewer for every camera sensor
            try:
                self.viewers = []
                if usecameraview:
                    Tk # check if Tk exists
                    if showsensors:
                        for attachedsensor in self.robotreal.GetAttachedSensors():
                            if attachedsensor.GetSensor() is not None:
                                sensordata = attachedsensor.GetSensor().GetSensorData()
                                if sensordata is not None and sensordata.type == Sensor.Type.Camera:
                                    attachedsensor.GetSensor().SendCommand('power 1')
                                    title = attachedsensor.GetName()
                                    if len(title) == 0:
                                        title = attachedsensor.GetSensor().GetName()
                                        if len(title) == 0:
                                            title = 'Camera Sensor'
                                    self.viewers.append(CameraViewerGUI(sensor=attachedsensor.GetSensor(),title=title))
                                    break # can only support one camera
                        print 'found %d camera sensors on robot %s'%(len(self.viewers),self.robotreal.GetName())
                        for viewer in self.viewers:
                            viewer.start()
            except NameError,e:
                print 'failed to create camera gui: ',e
                self.viewers = []

            self.sensor = [s for s in self.robotreal.GetAttachedSensors() if s.GetName()==sensorname][0]
            # find a manipulator whose end effector is the camera
            self.manip = [m for m in self.robotreal.GetManipulators() if m.GetEndEffector() == self.sensor.GetAttachingLink()][0]
            self.robotreal.SetActiveManipulator(self.manip)
    def computevisibilitymodel(self,target):
        vmodel = visibilitymodel.VisibilityModel(robot=self.robot,target=target,sensorname=self.sensor.GetName())
        if not vmodel.load():
            vmodel.autogenerate()
        return vmodel

    def waitrobot(self,robot=None):
        if robot is None:
            robot = self.robotreal
        while not robot.GetController().IsDone():
            time.sleep(0.01)

    def robotgohome(self,homevalues=None):
        if homevalues is None:
            homevalues = self.homevalues
        print 'moving arm'
        self.robotreal.SetActiveManipulator(self.manip)
        trajdata = self.basemanip.MoveManipulator(goal=homevalues[self.manip.GetArmJoints()],execute=False,outputtraj=True)
        self.starttrajectory(trajdata)
        
        print 'moving hand'
        values = self.robotreal.GetDOFValues()
        values[self.manip.GetGripperJoints()] = homevalues[self.manip.GetGripperJoints()]
        self.robotreal.GetController().SetDesired(values)
        self.waitrobot()
        if not self.robot is None:
            self.robot.GetController().SetDesired(self.robotreal.GetDOFValues())
    
    def movegripper(self,grippervalues,robot=None):
        if robot is None:
            robot = self.robotreal
        gripperjoints = self.manip.GetGripperJoints()
        assert len(gripperjoints) == len(grippervalues)

        with robot:
            robot.SetActiveDOFs(self.manip.GetArmJoints())
            trajdata = self.basemanip.MoveUnsyncJoints(jointvalues=grippervalues,jointinds=gripperjoints,outputtraj=True)
            self.trajectorylog.append(trajdata)
        self.waitrobot()
        # move the hand to the preshape
        with self.robot:
            self.robot.SetActiveDOFs(indices)
            self.basemanip.MoveActiveJoints(goal=values)
        self.waitrobot()

        values = robot.GetDOFValues()
        values[gripperjoints] = self.graspsetdata[0][12:]
        robot.GetController().SetDesired(values)
        self.waitrobot(robot)
        # set the real values for simulation
        v = self.robotreal.GetDOFValues()
        v[gripperjoints] = grippervalues
        self.robot.GetController().SetDesired(v)
    def syncrobot(self,robot):
        with self.robotreal:
            values = self.robotreal.GetDOFValues()
            T = self.robotreal.GetTransform()
        robot.SetTransform(T)
        robot.GetController().SetDesired(values)

    def gettarget(self,orenv):
        with orenv:
            bodies = orenv.GetBodies()
            targets = [b for b in bodies if b.GetName().find('frootloops') >=0 ]
            if len(targets) > 0:
                return targets[0]
        return None

    def starttrajectory(self,trajdata):
        if trajdata is not None and len(trajdata) > 0:
            self.trajectorylog.append(trajdata)
            self.basemanip.TrajFromData(trajdata)
            self.waitrobot()
            if self.robot is not None:
                self.robot.GetController().SetDesired(self.robotreal.GetDOFValues())
            time.sleep(0.2) # give some time for GUI to update

    def start(self,dopause=False,usevision=False):
        self.robotreal.ReleaseAllGrabbed()
        self.robotgohome()
        self.robotreal.GetController().SetDesired(self.homevalues)
        self.waitrobot()
        
        while True:
            self.robotreal.ReleaseAllGrabbed()
            self.orenv = self.orenvreal.CloneSelf(CloningOptions.Bodies)
            self.robot = self.orenv.GetRobot(self.robotreal.GetName())
            for sensor in self.robot.GetAttachedSensors():
                if sensor.GetSensor() is not None:
                    sensor.GetSensor().SendCommand('power 0')
            
            if self.orenv.CheckCollision(self.robot):
                print 'robot in collision, trying again...'
                time.sleep(0.5)
                continue

            self.robot.SetActiveManipulator(self.manip.GetName())
            self.robotreal.SetActiveManipulator(self.manip.GetName())
            self.target = self.gettarget(self.orenv)
            vmodel = self.computevisibilitymodel(self.target)
            vmodelreal = vmodel.clone(self.orenvreal)
            vmodelreal.moveToPreshape()
            basemanip = BaseManipulation(self.robot)
            self.robot.GetController().SetDesired(self.robotreal.GetDOFValues()) # update the robot
            try:
                trajdata = vmodel.visualprob.MoveToObserveTarget(sampleprob=0.001,maxiter=4000,execute=False,outputtraj=True)
                self.starttrajectory(trajdata)
            except planning_error:
                print 'failed to find visual feedback grasp'
                continue

            if not vmodel.visualprob.ComputeVisibility():
                print 'visibility has not been achieved!'
                continue
            T = self.target.GetTransform()
            targetfilename = self.target.GetXMLFilename()
            if usevision:
                self.orenvreal.RemoveKinBody(self.orenvreal.GetKinBody(self.target.GetName()))
                print 'waiting for object to be detected'
            self.orenv.RemoveKinBody(self.target)
            self.target = None
            while True:
                target = self.gettarget(self.orenvreal)
                if target is not None:
                    self.target = self.orenv.ReadKinBodyXMLFile(targetfilename)
                    self.orenv.AddKinBody(self.target)
                    self.target.SetTransform(target.GetTransform())
                    break
                time.sleep(0.1)

            # start visual servoing step
            gmodel = grasping.GraspingModel(robot=self.robot,target=self.target)
            if not gmodel.load():
                gmodel.autogenerate()
            taskmanip = TaskManipulation(self.robot,graspername=gmodel.grasper.plannername)
            trajdata = None
            with self.robot:
                validgrasps,validindices = gmodel.computeValidGrasps()
                if len(validgrasps) == 0:
                    continue
                for iter in range(4):
                    # set the real values for simulation
                    gmodel.setPreshape(validgrasps[0])
                    #trajdata = visualprob.SendCommand('VisualFeedbackGrasping target ' + self.target.GetName() + ' sensorindex 0 convexdata ' + str(self.convexdata.shape[0]) + ' ' + ' '.join(str(f) for f in self.convexdata.flat) + ' graspsetdata ' + str(self.graspsetdata.shape[0]) + ' ' + ' '.join(str(f) for f in self.graspsetdata[:,0:12].flat) + ' maxiter 100 visgraspthresh 0.5 gradientsamples 5 ' + cmdstr)
                    try:
                        trajdata = basemanip.MoveToHandPosition(matrices=[gmodel.getGlobalGraspTransform(g,collisionfree=True) for g in validgrasps],execute=False,outputtraj=True)
                        break
                    except planning_error:
                        print 'trying visual feedback grasp again'
            if trajdata is None:
                continue
            self.starttrajectory(trajdata)

            try:
                final,trajdata = taskmanip.CloseFingers(offset=self.graspoffset*ones(len(self.manip.GetGripperJoints())),execute=False,outputtraj=True)
                self.starttrajectory(trajdata)
            except planning_error:
                raise ValueError('failed to find visual feedback grasp')
            
            self.robot.Grab(self.target)
            self.robotreal.Grab(self.orenvreal.GetKinBody(self.target.GetName()))
            Trelative = dot(linalg.inv(self.target.GetTransform()),self.manip.GetEndEffectorTransform())
            Tnewgoals = [dot(Tgoal,Trelative) for Tgoal in self.Tgoals]
            if len(Tnewgoals) > 0:
                try:
                    print 'moving up'
                    trajdata = basemanip.MoveHandStraight(direction=[0,0,1],stepsize=0.001,maxsteps=100,execute=False,outputtraj=True)
                    self.starttrajectory(trajdata)
                except planning_error:
                    print 'failed to find trajectory'

                success = True
                try:
                    print 'moving to destination'
                    trajdata = basemanip.MoveToHandPosition(matrices=Tnewgoals, maxiter=1000,maxtries=1, seedik= 4,execute=False,outputtraj=True)
                    self.starttrajectory(trajdata)
                except planning_error:
                    print 'failed to find trajectory'
                    success = False

            try:
                final,trajdata = taskmanip.ReleaseFingers(target=self.target,execute=False,outputtraj=True)
                self.starttrajectory(trajdata)
            except planning_error:
                print 'failed to release'
                success = False

            if not success:
                continue

    def quitviewers(self):
        for viewer in self.viewers:
            viewer.main.quit()
    def __del__(self):
        self.quitviewers()
        self.orenv.Destroy()

class PA10GraspExample(VisibilityGrasping):
    """Specific class to setup an PA10 scene for visibility grasping"""
        
    def loadscene(self,randomize=True,**kwargs):
        VisibilityGrasping.loadscene(self,**kwargs)
        self.Tgoals = []

        if not randomize:
            return

        # randomize robot position
        Trobot = self.robotreal.GetTransform()
        while True:
            Tnew = array(Trobot)
            Tnew[0:2,3] += array([-0.1,-0.5])+random.rand(2)*array([0.3,1])
            self.robotreal.SetTransform(Tnew)
            if not self.robotreal.GetEnv().CheckCollision(self.robotreal):
                break

        createprob = 0.15
        obstacles = ['data/box0.kinbody.xml','data/box1.kinbody.xml','data/box2.kinbody.xml','data/box3.kinbody.xml']
        maxcreate = 6

        with self.orenvreal:
            self.target = self.gettarget(self.orenvreal)
            
            table = [b for b in self.orenvreal.GetBodies() if b.GetName() == 'table'][0]
            avoidbodies = [self.robotreal, self.target]
            
            Tidentity = eye(4)
            Ttable = table.GetTransform()
            table.SetTransform(eye(4))
            ab = table.ComputeAABB()
            table.SetTransform(Ttable)
            
            # table up is assumed to be +z, sample the +y axis of the table
            Nx = int(2*ab.extents()[0]/0.1)
            Ny = int(2*ab.extents()[1]/0.1)
            X = []
            Y = []
            for x in range(Nx):
                X = r_[X,0.5*random.rand(Ny)/(Nx+1) + float(x+1)/(Nx+1)]
                Y = r_[Y,0.5*random.rand(Ny)/(Ny+1) + (arange(Ny)+0.5)/(Ny+1)]

            offset = ab.pos() + array([-1,-1,1])*ab.extents()
            trans = c_[offset[0]+2*ab.extents()[0]*X, offset[1]+2*ab.extents()[1]*Y, tile(offset[2],len(X))]
            
            # for every destination try inserting a box
            if maxcreate > 0:
                numcreated = 0
                for t in trans:
                    if random.rand() < createprob:
                        iobs = random.randint(len(obstacles))
                        body = self.orenvreal.ReadKinBodyXMLFile(obstacles[iobs])
                        if body is None:
                            print 'invalid body %s'%obstacles[iobs]
                            continue
                        
                        body.SetName('obstacle%d'%numcreated)
                        self.orenvreal.AddKinBody(body)
                        
                        angs = arange(0,pi,pi/3)
                        angs = angs[random.permutation(len(angs))]
                        success = False
                        for roll in angs:
                            T = eye(4)
                            T[0:3,0:3] = rotationMatrixFromAxisAngle(array([0,0,1]),roll)
                            T[0:3,3] = t
                            T = dot(Ttable,T)
                            body.SetTransform(T)
                            if all([not self.orenvreal.CheckCollision(body,avoidbody) for avoidbody in avoidbodies]):
                                numcreated = numcreated+1
                                body = None
                                break
                        if body is not None:
                            self.orenvreal.RemoveKinBody(body)
                        if numcreated >= maxcreate:
                            break

            # find all destinations not in collision
            with self.target:
                Torig = self.target.GetTransform()
                for t in trans:
                    for roll in arange(0,pi,pi/4):
                        T = eye(4)
                        T[0:3,0:3] = rotationMatrixFromAxisAngle(array([0,0,1]),roll)
                        T[0:2,3] = t[0:2]
                        T = dot(Ttable,T)
                        T[2,3] = Torig[2,3] # preserve Z
                        self.target.SetTransform(T)
                        if not self.target.GetEnv().CheckCollision(self.target):
                            self.Tgoals.append(T)

def run(args=None):
    """Executes the visibilityplanning example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Visibility Planning Module.')
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/pa10grasp.env.xml',
                      help='openrave scene to load')
    parser.add_option('--nocameraview',action="store_false",dest='usecameraview',default=True,
                      help='If set, will not open any camera views')
    (options, leftargs) = parser.parse_args(args=args)
    scene = PA10GraspExample()
    scene.loadscene(scenefilename=options.scene,sensorname='wristcam',usecameraview=options.usecameraview)
    scene.start()

if __name__=='__main__':
    run()
