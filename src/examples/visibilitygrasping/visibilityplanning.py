#!/usr/bin/env python
#
# Copyright (C) 2009 Rosen Diankov
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
import sys, os, time, signal, threading, pickle
import numpy,scipy # nice to be able to explicitly call some functions
from numpy import *
from optparse import OptionParser
from openravepy import *

from Tkinter import *
import tkFileDialog
import Image, ImageDraw, ImageTk

#sys.path.append('..')
#import metaclass

class CameraViewerGUI(threading.Thread):
    class Container:
        pass
    def __init__(self,sensor,title='Camera Viewer'):
        threading.Thread.__init__(self)
        self.sensor = sensor
        self.title = title
        self.lastid = -1
        self.imagelck = threading.Lock()

    def updateimage(self):
        data = self.sensor.GetSensorData()
        if data is not None and not self.lastid == data.id:
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
            self.lastid = data.id
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

class VisibilityGrasping():
    """Calls on the openrave grasp planners to get a robot to pick up objects while guaranteeing visibility with its cameras"""
    def __init__(self):
        self.orenvreal = Environment()
        self.orenvreal.SetViewer('qtcoin')
        self.trajectorylog = []
        self.graspoffset = 0

    def reloadcontroller(self):
        self.orenvreal.LockPhysics(True)

        try:
            self.robotreal.GetController().Reset(0)
        finally:
            self.orenvreal.LockPhysics(False)

    def setjointvalue(self,value,index):
        v = self.robotreal.GetJointValues()
        v[index] = value
        self.robotreal.GetController().SetDesired(v)

    def preprocessdata(self, maskfile, convexfile, robotfile, graspsetfile, targetfile, graspingppfile, visibilityfile, robotjointinds = [], robotjoints = []):
        jointstring = ' --robotjointinds="' + ' '.join(str(f) for f in robotjointinds) + '" --robotjoints="' + ' '.join(str(f) for f in robotjoints) + '"'
        ret = os.system('python visibilityprocessing.py --func=mask --rayoffset=0.025 --robotfile=' + robotfile + ' --graspsetfile=' + graspsetfile +  ' --savefile=' + maskfile + jointstring)
        if ret != 0:
            raise ValueError('failed to compute mask')
        ret = os.system("""octave --eval "GetLargestFreeConvexPolygon(load('%s'),'%s');" """%(maskfile,convexfile))
        if ret != 0:
            raise ValueError('failed to compute convex hull')
        ret = os.system('python visibilityprocessing.py --func=visibility --robotfile=' + robotfile + ' --kinbodyfile=' + targetfile + ' --graspsetfile=' + graspsetfile + ' --savepp=' + graspingppfile + ' --convexfile=' + convexfile + ' --visibilityfile=' + visibilityfile + ' --savefile=visibilitytrans.mat ' + jointstring)
        if ret != 0:
            raise ValueError('failed to process visibility information')

    def loadscene(self,scenefilename,robotname,sensorname,graspingppfile,handfilename=None,showsensors=True):
        self.target = None
        self.robot = None
        self.robotreal = None
        self.orenvreal.Reset()
        time.sleep(2)

        if not self.orenvreal.Load(scenefilename):
            raise ValueError('failed to open %s openrave file'%scenefilename)
        if len(self.orenvreal.GetRobots()) == 0:
            raise ValueError('no robots found in scene %s'%scenefilename)

        if robotname is None:
            self.robotreal = self.orenvreal.GetRobots()[0]
        else:
            self.robotreal = [r for r in self.orenvreal.GetRobots() if r.GetName()==robotname][0]
                    
        self.taskprob = self.orenvreal.CreateProblem('TaskManipulation')
        self.manipprob = self.orenvreal.CreateProblem('BaseManipulation')
        self.visualprob = self.orenvreal.CreateProblem('VisualFeedback')
        self.orenvreal.LoadProblem(self.taskprob,self.robotreal.GetName())
        self.orenvreal.LoadProblem(self.manipprob,self.robotreal.GetName())
        self.orenvreal.LoadProblem(self.visualprob,self.robotreal.GetName())
        
        self.homevalues = self.robotreal.GetJointValues()

        # create a camera viewer for every camera sensor
        self.viewers = []
        if showsensors:
            for attachedsensor in self.robotreal.GetSensors():
                if attachedsensor.GetSensor() is not None:
                    sensordata = attachedsensor.GetSensor().GetSensorData()
                    if sensordata is not None and sensordata.type == Sensor.SensorType.Camera:
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

        self.sensor = [s for s in self.robotreal.GetSensors() if s.GetName()==sensorname][0]
        # find a manipulator whose end effector is the camera
        self.manipindex,self.manip = [(i,m) for i,m in enumerate(self.robotreal.GetManipulators()) if m.GetEndEffector().GetIndex() == self.sensor.GetAttachingLink().GetIndex()][0]
        self.robotreal.SetActiveManipulator(self.manipindex)

        self.handfilename = handfilename
        if self.handfilename is not None:
            self.hand = self.orenvreal.ReadRobotXMLFile(self.handfilename)
            self.orenvreal.AddRobot(self.hand)
            T = eye(4)
            T[2,3] = 100
            self.hand.SetTransform(T)
        else:
            self.hand = None

        if graspingppfile is not None:
            self.convexdata,self.visibilitydata,self.graspsetdata = pickle.load(open(graspingppfile,'r'))
            if self.hand is not None:
                self.hand.SetJointValues(self.graspsetdata[0][12:])

    def waitrobot(self):
        while not self.robotreal.GetController().IsDone():
            time.sleep(0.01)

    def robotgohome(self,homevalues=None):
        if homevalues is None:
            homevalues = self.homevalues
        print 'moving arm'
        trajdata = self.manipprob.SendCommand('MoveManipulator execute 0 outputtraj armvals ' + ' '.join(str(homevalues[i]) for i in self.manip.GetArmJoints()))
        if len(trajdata) == 0:
            raise ValueError('failed to start trajectory')
        self.starttrajectory(trajdata)
        
        print 'moving hand'
        values = array(self.robotreal.GetJointValues(),'float')
        values[self.manip.GetGripperJoints()] = homevalues[self.manip.GetGripperJoints()]
        self.robotreal.GetController().SetDesired(values)
        self.waitrobot()
        if self.robot is not None:
            self.robot.GetController().SetDesired(self.robotreal.GetJointValues())
    
    def movegripper(self,grippervalues,robot=None):
        if robot is None:
            robot = self.robotreal
        gripperjoints = self.manip.GetGripperJoints()
        if len(gripperjoints) != len(grippervalues):
            raise ValueError('dof not equal')
        robot.SetActiveDOFs(self.manip.GetArmJoints())
        trajdata = self.manipprob.SendCommand('MoveUnsyncJoints outputtraj handjoints ' + str(len(gripperjoints)) + ' ' + ' '.join(str(f) for f in grippervalues) + ' ' + ' '.join(str(ind) for ind in gripperjoints))
        self.trajectorylog.append(trajdata)
        self.waitrobot()
        values = array(robot.GetJointValues(),'float')
        values[gripperjoints] = self.graspsetdata[0][12:]
        robot.GetController().SetDesired(values)
        self.waitrobot()
        # set the real values for simulation
        v = array(self.robotreal.GetJointValues(),'float')
        v[gripperjoints] = grippervalues
        self.robot.GetController().SetDesired(v)

    def testhand(self,T=None,pose=None,target=None):
        if pose is not None:
            T = matrixFromPose(pose)
        if target is not None:
            T = dot(target.GetTransform(),T)
        self.hand.SetTransform(dot(T,linalg.inv(self.manip.GetGraspTransform())))

    def getGraspTransform(self,i=0):
        T = eye(4)
        T[0:3,0:4] = transpose(reshape(self.graspsetdata[i][0:12],(4,3)))
        return T

    def groundrobot(self):
        self.orenvreal.LockPhysics(True)
        try:
            #self.robot.GetController().Reset(0)
            ab = self.robotreal.ComputeAABB()
            T = self.robotreal.GetTransform()
            T[2,3] += 0.001-(ab.pos()[2]-ab.extents()[2])
            self.robotreal.SetTransform(T)
        finally:
            self.orenvreal.LockPhysics(False)

    def gettarget(self,orenv):
        orenv.LockPhysics(True)
        try:
            bodies = orenv.GetBodies()
            targets = [b for b in bodies if b.GetName().find('frootloops') >=0 ]
            if len(targets) > 0:
                return targets[0]
        finally:
            orenv.LockPhysics(False)
        return None

    def starttrajectory(self,trajdata):
        if trajdata is not None and len(trajdata) > 0:
            self.trajectorylog.append(trajdata)
            self.manipprob.SendCommand('traj stream ' + trajdata)
            self.waitrobot()
            if self.robot is not None:
                self.robot.GetController().SetDesired(self.robotreal.GetJointValues())

    def start(self,dopause=False,usevision=False):
        self.robotreal.ReleaseAllGrabbed()
        self.robotgohome()
        self.robotreal.GetController().SetDesired(self.homevalues)
        self.waitrobot()
        self.groundrobot()
        
        while True:
            self.robotreal.ReleaseAllGrabbed()
            self.orenv = self.orenvreal.CloneSelf(CloningOptions.Bodies)
            self.robot = self.orenv.GetRobot(self.robotreal.GetName())
            for sensor in self.robot.GetSensors():
                if sensor.GetSensor() is not None:
                    sensor.GetSensor().SendCommand('power 0')
            
            taskprob = self.orenv.CreateProblem('TaskManipulation')
            manipprob = self.orenv.CreateProblem('BaseManipulation')
            visualprob = self.orenv.CreateProblem('VisualFeedback')
            self.orenv.LoadProblem(taskprob,self.robot.GetName())
            self.orenv.LoadProblem(manipprob,self.robot.GetName())
            self.orenv.LoadProblem(visualprob,self.robot.GetName())
            cmdstr = ' execute 0 outputtraj '
            
            self.target = self.gettarget(self.orenv)
            self.movegripper(self.graspsetdata[0][12:])

            trajdata = visualprob.SendCommand('MoveToObserveTarget target ' + self.target.GetName() + ' sampleprob 0.001 sensorindex 0 maxiter 4000 convexdata ' + str(self.convexdata.shape[0]) + ' ' + ' '.join(str(f) for f in self.convexdata.flat) + ' visibilitydata ' + str(self.visibilitydata.shape[0]) + ' ' + ' '.join(str(f) for f in self.visibilitydata.flat) + cmdstr)
            if trajdata is None:
                raise ValueError('failed to find visual feedback grasp')
            self.starttrajectory(trajdata)
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
            while True:
                # set the real values for simulation
                v = array(self.robotreal.GetJointValues(),'float')
                v[self.manip.GetGripperJoints()] = self.graspsetdata[0][12:]
                self.robot.GetController().SetDesired(v)

                trajdata = visualprob.SendCommand('VisualFeedbackGrasping target ' + self.target.GetName() + ' sensorindex 0 convexdata ' + str(self.convexdata.shape[0]) + ' ' + ' '.join(str(f) for f in self.convexdata.flat) + ' graspsetdata ' + str(self.graspsetdata.shape[0]) + ' ' + ' '.join(str(f) for f in self.graspsetdata[:,0:12].flat) + ' maxiter 100 visgraspthresh 0.5 gradientsamples 5 ' + cmdstr)
                if trajdata is not None:
                    break
                print 'trying visual feedback grasp again'
            self.starttrajectory(trajdata)

            trajdata = manipprob.SendCommand('closefingers offset %f '%self.graspoffset + cmdstr)
            if trajdata is None:
                raise ValueError('failed to find visual feedback grasp')
            self.starttrajectory(trajdata)
            self.robot.Grab(self.target)
            self.robotreal.Grab(self.orenvreal.GetKinBody(self.target.GetName()))
            Trelative = dot(linalg.inv(self.target.GetTransform()),self.manip.GetEndEffectorTransform())
            Tnewgoals = [dot(Tgoal,Trelative) for Tgoal in self.Tgoals]

            trajdata = manipprob.SendCommand('movehandstraight direction 0 0 1 stepsize 0.001 maxsteps 100' + cmdstr)
            if trajdata is None:
                print 'failed to find trajectory'
            self.starttrajectory(trajdata)
            trajdata = manipprob.SendCommand('movetohandposition maxiter 1000 maxtries 1 seedik 4 matrices ' + str(len(Tnewgoals)) + ' '.join(matrixSerialization(T) for T in Tnewgoals) + cmdstr)
            if trajdata is None:
                print 'failed to find trajectory'
                trajdata = manipprob.SendCommand('releasefingers target ' + self.target.GetName() + cmdstr)
                self.starttrajectory(trajdata)
                continue
            self.starttrajectory(trajdata)

            self.robot.SetActiveDOFs(self.manip.GetGripperJoints())
            trajdata = manipprob.SendCommand('releasefingers target ' + self.target.GetName() + cmdstr)
            if trajdata is None:
                print 'failed to find trajectory'
                continue
            self.starttrajectory(trajdata)

    def quitviewers(self):
        for viewer in self.viewers:
            viewer.main.quit()
    def __del__(self):
        self.quitviewers()
        self.orenv.Destroy()

class HRP2GraspExample(VisibilityGrasping):
    """Specific class to setup an hrp2 scene for visibility grasping"""

    def preprocessdata(self):
        VisibilityGrasping.preprocessdata(self, maskfile = 'hrp2gripper_mask.mat', convexfile = 'hrp2gripper_convex.mat', robotfile = 'robots/hrp2jskreal.robot.xml', graspsetfile = 'simple_grasp_hrp2_cereal2.mat', targetfile = 'data/box_frootloops.kinbody.xml', graspingppfile = 'hrp2_frootloops_visibility.pp', visibilityfile = 'cereal_visibility.mat', robotjointinds = [21], robotjoints = [-1.7])

    def loadscene(self):
        VisibilityGrasping.loadscene(scenefilename='scenes/r602cerealmanip.env.xml',robotname='HRP2JSK',sensorname='wristcam',graspingppfile='hrp2_frootloops_visibility.pp', handfilename='robots/hrp2rhandjsk.robot.xml')
        
        self.homevalues = array([ -2.86345789e-03,   4.97418863e-04,  -4.52529013e-01, 8.72043371e-01,  -4.17762041e-01,   8.16814136e-04, 0.00000000e+00,   3.43606574e-03,  -4.97418863e-04, -4.52249706e-01,   8.72011900e-01,  -4.17901635e-01, 7.53982284e-04,   0.00000000e+00,   8.47067393e-04, 2.84897187e-03,   1.59534463e-03,   2.57708761e-03, 7.69527555e-02,  -1.91759229e-01,   2.73909390e-01, -2.02681810e-01,  -2.30580971e-01,   9.90253594e-03, -2.71063328e-01,   1.55539140e-01,   2.52415299e-01, -3.49661469e-01,  -4.49272335e-01,   3.51060212e-01, 7.22945556e-02,  -1.47477672e-01,  -2.63864666e-01, 2.64033407e-01])

        self.Tgoals = [array([[  5.96046377e-08,  -1.00000000e+00,   1.42291853e-07, 8.61060917e-01],
                              [  1.00000000e+00,   5.96046448e-08,   2.74064149e-08, -6.78065157e+00],
                              [ -2.74064291e-08,   1.42291853e-07,   1.00000000e+00, 8.79451871e-01],
                              [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])]
        self.graspoffset = 0.3

class PA10GraspExample(VisibilityGrasping):
    """Specific class to setup an PA10 scene for visibility grasping"""
    def preprocessdata(self):
        VisibilityGrasping.preprocessdata(self, maskfile = 'pa10gripper_mask.mat', convexfile = 'pa10gripper_convex.mat', robotfile = 'robots/pa10schunk.robot.xml', graspsetfile = 'simple_cereal_grasps_pa10.mat', targetfile = 'data/box_frootloops.kinbody.xml', graspingppfile = 'pa10_frootloops_visibility.pp', visibilityfile = 'cereal_visibility.mat', robotjointinds = [], robotjoints = [])
        
    def loadscene(self):
        VisibilityGrasping.loadscene(self,scenefilename='data/pa10grasp.env.xml',robotname='PA10',sensorname='wristcam',graspingppfile='pa10_frootloops_visibility.pp', handfilename='robots/schunk_manip.robot.xml')

        createprob = 0.15
        obstacles = ['data/box0.kinbody.xml','data/box1.kinbody.xml','data/box2.kinbody.xml','data/box3.kinbody.xml']
        maxcreate = 4

        # randomize robot position
        Trobot = self.robotreal.GetTransform()
        while True:
            Tnew = array(Trobot)
            Tnew[0:2,3] += array([-0.1,-0.5])+random.rand(2)*array([0.3,1])
            self.robotreal.SetTransform(Tnew)
            if not self.orenvreal.CheckCollision(self.robotreal):
                break

        self.orenvreal.LockPhysics(True)
        try:
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
                X = r_[X,0.5*random.rand(Ny)/(Nx+1) + (x+1)/(Nx+1)]
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
            self.Tgoals = []
            Torig = self.target.GetTransform()
            for t in trans:
                for roll in arange(0,pi,pi/4):
                    T = eye(4)
                    T[0:3,0:3] = rotationMatrixFromAxisAngle(array([0,0,1]),roll)
                    T[0:2,3] = t[0:2]
                    T = dot(Ttable,T)
                    T[2,3] = Torig[2,3] # preserve Z
                    self.target.SetTransform(T)
                    if not self.orenvreal.CheckCollision(self.target):
                        self.Tgoals.append(T)
            self.target.SetTransform(Torig)
        finally:
            self.orenvreal.LockPhysics(False)

if __name__=='__main__':
    parser = OptionParser(description='Visibility Planning Module.')
    parser.add_option('--examplenum',
                      action="store",type='int',dest='examplenum',default=0,
                      help='The example number to load: 0 - PA10')
    (options, args) = parser.parse_args()

    scene = PA10GraspExample()
    scene.loadscene()
    scene.start()

def test():
    import visibilityplanning, time
    self = visibilityplanning.HRP2GraspingScene()
    time.sleep(5)
    self.loadscene(scenefilename='scenes/r602real.env.xml')
    self.testsim()

