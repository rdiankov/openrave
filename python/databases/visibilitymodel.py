#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
"""Samples visible locations of a target object and a sensor.

.. image:: ../../images/databases/visibilitymodel.jpg
  :width: 640

`[source] <../_modules/openravepy/databases/visibilitymodel.html>`_

**Running the Generator**

.. code-block:: bash

  openrave.py --database visibilitymodel --robot=robots/pa10schunk.robot.xml

**Showing Visible Locations**

.. code-block:: bash

  openrave.py --database visibilitymodel --robot=robots/pa10schunk.robot.xml --show

Usage
-----

Dynamically generate/load the visibility sampler for a manipulator/sensor/target combination:

.. code-block:: python

  robot.SetActiveManipulator(...)
  ikmodel = openravepy.databases.visibilitymodel.VisibilityModel(robot,target,sensorname)
  if not vmodel.load():
      vmodel.autogenerate()


Description
-----------

As long as a sensor is attached to a robot arm, can be applied to any robot to get immediate visibiliy configuration sampling:

.. image:: ../../images/databases/visibilitymodel_extents.jpg
  :height: 250

The visibility database generator uses the :ref:`module-visualfeedback` for the underlying visibility
computation. The higher level functions it provides are sampling configurations, computing all valid
configurations with the manipulator, and display.

Command-line
------------

.. shell-block:: openrave.py --database visibilitymodel --help

Class Definitions
-----------------

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import time
import os.path

if not __openravepy_build_doc__:
    from ..openravepy_int import *
    from ..openravepy_ext import *
    from numpy import *
else:
    from numpy import array

from . import DatabaseGenerator
import inversekinematics, kinematicreachability
from .. import interfaces

class VisibilityModel(DatabaseGenerator):
    class GripperVisibility:
        """Used to hide links not beloning to gripper.

        When 'entered' will hide all the non-gripper links in order to facilitate visiblity of the gripper
        """
        def __init__(self,manip):

            self.manip = manip
            self.robot = self.manip.GetRobot()
            self.hiddengeoms = []
        def __enter__(self):
            self.hiddengeoms = []
            with self.robot.GetEnv():
                # stop rendering the non-gripper links
                childlinkids = [link.GetIndex() for link in self.manip.GetChildLinks()]
                for link in self.robot.GetLinks():
                    if link.GetIndex() not in childlinkids:
                        for geom in link.GetGeometries():
                            self.hiddengeoms.append((geom,geom.IsDraw()))
                            geom.SetDraw(False)
        def __exit__(self,type,value,traceback):
            with self.robot.GetEnv():
                for geom,isdraw in self.hiddengeoms:
                    geom.SetDraw(isdraw)

    def __init__(self,robot,target,sensorrobot=None,sensorname=None,maxvelmult=None):
        """Starts a visibility model using a robot, a sensor, and a target

        The minimum needed to be specified is the robot and a sensorname. Supports sensors that do
        not belong to the current robot in the case that a robot is holding the target with its
        manipulator. Providing the target allows visibility information to be computed.
        """
        DatabaseGenerator.__init__(self,robot=robot)
        self.sensorrobot = sensorrobot if sensorrobot is not None else robot
        self.target = target
        self.visualprob = interfaces.VisualFeedback(self.robot,maxvelmult=maxvelmult)
        self.basemanip = interfaces.BaseManipulation(self.robot,maxvelmult=maxvelmult)
        self.convexhull = None
        self.sensorname = sensorname
        if self.sensorname is None:
            possiblesensors = [s.GetName() for s in self.sensorrobot.GetAttachedSensors() if s.GetSensor() is not None and s.GetSensor().Supports(Sensor.Type.Camera)]
            if len(possiblesensors) > 0:
                self.sensorname = possiblesensors[0]
        self.manip = robot.GetActiveManipulator()
        self.manipname = None if self.manip is None else self.manip.GetName()
        self.visibilitytransforms = None
        self.rmodel = self.ikmodel = None
        self.preshapes = None
        self.preprocess()
    def clone(self,envother):
        clone = DatabaseGenerator.clone(self,envother)
        clone.rmodel = self.rmodel.clone(envother) if not self.rmodel is None else None
        clone.preshapes = array(self.preshapes) if not self.preshapes is None else None
        clone.ikmodel = self.ikmodel.clone(envother) if not self.ikmodel is None else None
        clone.visualprob = self.visualprob.clone(envother)
        clone.basemanip = self.basemanip.clone(envother)
        clone.preprocess()
        return clone
    def has(self):
        return self.visibilitytransforms is not None and len(self.visibilitytransforms) > 0
    def getversion(self):
        return 2
    def getfilename(self,read=False):
        return RaveFindDatabaseFile(os.path.join('robot.'+self.robot.GetKinematicsGeometryHash(), 'visibility.' + self.manip.GetStructureHash() + '.' + self.attachedsensor.GetStructureHash() + '.' + self.target.GetKinematicsGeometryHash()+'.pp'),read)
    def load(self):
        try:
            params = DatabaseGenerator.load(self)
            if params is None:
                return False
            self.visibilitytransforms,self.convexhull,self.KK,self.dims,self.preshapes = params
            self.preprocess()
            return self.has()
        except e:
            return False
    def save(self):
        DatabaseGenerator.save(self,(self.visibilitytransforms,self.convexhull,self.KK,self.dims,self.preshapes))

    def preprocess(self):
        with self.env:
            manipname = self.visualprob.SetCameraAndTarget(sensorname=self.sensorname,sensorrobot=self.sensorrobot,manipname=self.manipname,target=self.target)
            assert(self.manipname is None or self.manipname==manipname)
            self.manip = self.robot.SetActiveManipulator(manipname)
            self.attachedsensor = [s for s in self.sensorrobot.GetAttachedSensors() if s.GetName() == self.sensorname][0]
            self.ikmodel = inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()
            if self.visibilitytransforms is not None:
                self.visualprob.SetCameraTransforms(transforms=self.visibilitytransforms)
    
    def autogenerate(self,options=None,gmodel=None):
        preshapes = None
        sphere =None
        conedirangles = None
        if options is not None:
            if options.preshapes is not None:
                preshapes = zeros((0,len(self.manip.GetGripperIndices())))
                for preshape in options.preshapes:
                    preshapes = r_[preshapes,[array([float(s) for s in preshape.split()])]]
            if options.sphere is not None:
                sphere = [float(s) for s in options.sphere.split()]
            if options.conedirangles is not None:
                conedirangles = []
                for conediranglestring in options.conedirangles:
                    conedirangles.append([float(s) for s in conediranglestring.split()])
        if not gmodel is None:
            preshapes = array([gmodel.grasps[0][gmodel.graspindices['igrasppreshape']]])
        if preshapes is None:
            with self.target:
                self.target.Enable(False)
                taskmanip = interfaces.TaskManipulation(self.robot)
                final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
            preshapes = array([final])
        self.generate(preshapes=preshapes,sphere=sphere,conedirangles=conedirangles)
        self.save()
    def generate(self,preshapes,sphere=None,conedirangles=None):
        self.preshapes=preshapes
        self.preprocess()
        self.sensorname = self.attachedsensor.GetName()
        self.manipname = self.manip.GetName()
        bodies = [(b,b.IsEnabled()) for b in self.env.GetBodies() if b != self.robot and b != self.target]
        for b in bodies:
            b[0].Enable(False)
        try:
            with self.env:
                sensor = self.attachedsensor.GetSensor()
                if sensor is not None: # set power to 0?
                    sensordata = sensor.GetSensorGeometry(Sensor.Type.Camera)
                    self.KK = sensordata.KK
                    self.dims = sensordata.imagedata.shape
                        
                with RobotStateSaver(self.robot):
                    # find better way of handling multiple grasps
                    if len(self.preshapes) > 0:
                            self.robot.SetDOFValues(self.preshapes[0],self.manip.GetGripperIndices())
                    extentsfile = os.path.join(RaveGetHomeDirectory(),'kinbody.'+self.target.GetKinematicsGeometryHash(),'visibility.txt')
                    if sphere is None and os.path.isfile(extentsfile):
                        self.visibilitytransforms = self.visualprob.ProcessVisibilityExtents(extents=loadtxt(extentsfile,float),conedirangles=conedirangles)
                    else:
                        if sphere is None:
                            sphere = [3,0.1,0.15,0.2,0.25,0.3]
                        self.visibilitytransforms = self.visualprob.ProcessVisibilityExtents(sphere=sphere,conedirangles=conedirangles)
                print 'total transforms: ',len(self.visibilitytransforms)
                self.visualprob.SetCameraTransforms(transforms=self.visibilitytransforms)
        finally:
            for b,enable in bodies:
                b.Enable(enable)

    def SetCameraTransforms(self,transforms):
        """Sets the camera transforms to the visual feedback problem"""
        self.visualprob.SetCameraTransforms(transforms=transforms)
    def showtransforms(self,options=None):
        pts = array([dot(self.target.GetTransform(),matrixFromPose(pose))[0:3,3] for pose in self.visibilitytransforms])
        h=self.env.plot3(pts,5,colors=array([0.5,0.5,1,0.2]))
        with RobotStateSaver(self.robot):
            # disable all non-child links
            for link in self.robot.GetLinks():
                link.Enable(link in self.manip.GetChildLinks())
            with self.GripperVisibility(self.manip):
                for i,pose in enumerate(self.visibilitytransforms):
                    with self.env:
                        if len(self.preshapes) > 0:
                            self.robot.SetDOFValues(self.preshapes[0],self.manip.GetGripperIndices())
                        Trelative = dot(linalg.inv(self.attachedsensor.GetTransform()),self.manip.GetEndEffectorTransform())
                        Tcamera = dot(self.target.GetTransform(),matrixFromPose(pose))
                        Tgrasp = dot(Tcamera,Trelative)
                        Tdelta = dot(Tgrasp,linalg.inv(self.manip.GetEndEffectorTransform()))
                        for link in self.manip.GetChildLinks():
                            link.SetTransform(dot(Tdelta,link.GetTransform()))
                        visibility = self.visualprob.ComputeVisibility()
                        self.env.UpdatePublishedBodies()
                    msg='%d/%d visibility=%d, press any key to continue: '%(i,len(self.visibilitytransforms),visibility)
                    if options is not None and options.showimage:
                        pilutil=__import__('scipy.misc',fromlist=['pilutil'])
                        I=self.getCameraImage()
                        print(msg)
                        pilutil.imshow(I)
                    else:
                        raw_input(msg)
    def show(self,options=None):
        self.env.SetViewer('qtcoin')
        self.attachedsensor.GetSensor().Configure(Sensor.ConfigureCommand.PowerOn)
        self.attachedsensor.GetSensor().Configure(Sensor.ConfigureCommand.RenderDataOn)
        return self.showtransforms(options)
    def moveToPreshape(self):
        """uses a planner to safely move the hand to the preshape and returns the trajectory"""
        if len(self.preshapes) > 0:
            preshape=self.preshapes[0]
            with self.robot:
                self.robot.SetActiveDOFs(self.manip.GetArmIndices())
                self.basemanip.MoveUnsyncJoints(jointvalues=preshape,jointinds=self.manip.GetGripperIndices())
            while not self.robot.GetController().IsDone(): # busy wait
                time.sleep(0.01)        
            with self.robot:
                self.robot.SetActiveDOFs(self.manip.GetGripperIndices())
                self.basemanip.MoveActiveJoints(goal=preshape)
            while not self.robot.GetController().IsDone(): # busy wait
                time.sleep(0.01)

    def computeValidTransform(self,returnall=False,checkcollision=True,computevisibility=True,randomize=False):
        with self.robot:
            if self.manip.CheckIndependentCollision():
                raise planning_error('robot independent links are initiallly in collision')
            validjoints = []
            if randomize:
                order = random.permutation(len(self.visibilitytransforms))
            else:
                order = xrange(len(self.visibilitytransforms))
            for i in order:
                pose = self.visibilitytransforms[i]
                Trelative = dot(linalg.inv(self.attachedsensor.GetTransform()),self.manip.GetEndEffectorTransform())
                Tcamera = dot(self.target.GetTransform(),matrixFromPose(pose))
                Tgrasp = dot(Tcamera,Trelative)
                s = self.manip.FindIKSolution(Tgrasp,checkcollision)
                if s is not None:
                    self.robot.SetDOFValues(s,self.manip.GetArmIndices())
                    if computevisibility and not self.visualprob.ComputeVisibility():
                        continue
                    validjoints.append((s,i))
                    if not returnall:
                        return validjoints
                    print 'found',len(validjoints)
            return validjoints

    def pruneTransformations(self,thresh=0.04,numminneighs=10,maxdist=None,translationonly=True):
        if self.rmodel is None:
            self.rmodel = kinematicreachability.ReachabilityModel(robot=self.robot)
            if not self.rmodel.load():
                # do not autogenerate since that would force this model to depend on the reachability
                self.rmodel = None
                return array(visibilitytransforms)
        kdtree=self.rmodel.ComputeNN(translationonly)
        if maxdist is not None:
            visibilitytransforms = self.visibilitytransforms[invertPoses(self.visibilitytransforms)[:,6]<maxdist]
        else:
            visibilitytransforms = self.visibilitytransforms
        newtrans = poseMultArrayT(poseFromMatrix(dot(linalg.inv(self.manip.GetBase().GetTransform()),self.target.GetTransform())),visibilitytransforms)
        if translationonly:
            transdensity = kdtree.kFRSearchArray(newtrans[:,4:7],thresh**2,0,thresh*0.01)[2]
            I=flatnonzero(transdensity>numminneighs)
            return visibilitytransforms[I[argsort(-transdensity[I])]]
        raise ValueError('not supported')
#         Imask = GetCameraRobotMask(orenv,options.robotfile,sensorindex=options.sensorindex,gripperjoints=gripperjoints,robotjoints=robotjoints,robotjointinds=robotjointinds,rayoffset=options.rayoffset)
#         # save as a ascii matfile
#         numpy.savetxt(options.savefile,Imask,'%d')
#         print 'mask saved to ' + options.savefile
#         try:
#             scipy.misc.pilutil.imshow(array(Imask*255,'uint8'))
#         except:
#             pass

#     def GetCameraRobotMask(self,rayoffset=0):
#         with self.env:
#             inds = array(range(self.width*self.height))
#             imagepoints = array((mod(inds,self.width),floor(inds/self.width)))
#             camerapoints = dot(linalg.inv(self.KK), r_[imagepoints,ones((1,imagepoints.shape[1]))])
#             Tcamera = self.attached.GetSensor().GetTransform()
#             raydirs = dot(Tcamera[0:3,0:3], camerapoints / tile(sqrt(sum(camerapoints**2,0)),(3,1)))
#             rays = r_[tile(Tcamera[0:3,3:4],(1,raydirs.shape[1]))+rayoffset*raydirs,100.0*raydirs]
#             hitindices,hitpositions = self.prob.GetEnv().CheckCollisionRays(rays,self.robot,False)
#             # gather all the rays that hit and form an image
#             return reshape(array(hitindices,'float'),(height,width))

    def getCameraImage(self,delay=1.0):
        sensor=self.attachedsensor.GetSensor()
        sensor.Configure(Sensor.ConfigureCommand.PowerOn)
        try:
            time.sleep(delay)
            return sensor.GetSensorData().imagedata
        finally:
            sensor.Configure(Sensor.ConfigureCommand.PowerOff)

    @staticmethod
    def CreateOptionParser():
        parser = DatabaseGenerator.CreateOptionParser()
        parser.description='Computes and manages the visibility transforms for a manipulator/target.'
        parser.add_option('--target',action="store",type='string',dest='target',
                          help='OpenRAVE kinbody target filename')
        parser.add_option('--sensorname',action="store",type='string',dest='sensorname',default=None,
                          help='Name of the sensor to build visibilty model for (has to be camera). If none, takes first possible sensor.')
        parser.add_option('--preshape', action='append', type='string',dest='preshapes',default=None,
                          help='Add a preshape for the manipulator gripper joints')
        parser.add_option('--sphere', action='store', type='string',dest='sphere',default=None,
                          help='Force detectability extents to be distributed around a sphere. Parameter is a string with the first value being density (3 is default) and the rest being distances')
        parser.add_option('--conedirangle', action='append', type='string',dest='conedirangles',default=None,
                          help='The direction of the cone multiplied with the half-angle (radian) that the detectability extents are constrained to. Multiple cones can be provided.')
        parser.add_option('--rayoffset',action="store",type='float',dest='rayoffset',default=0.03,
                          help='The offset to move the ray origin (prevents meaningless collisions), default is 0.03')
        parser.add_option('--showimage',action="store_true",dest='showimage',default=False,
                          help='If set, will show the camera image when showing the models')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None,args=None,**kwargs):
        if parser is None:
            parser = VisibilityModel.CreateOptionParser()
        (options, leftargs) = parser.parse_args(args=args)
        env = Environment()
        try:
            target = None
            with env:
                target = env.ReadKinBodyXMLFile(options.target)
                target.SetTransform(eye(4))
                env.AddKinBody(target)
            if Model is None:
                Model = lambda robot: VisibilityModel(robot=robot,target=target,sensorname=options.sensorname)
            DatabaseGenerator.RunFromParser(env=env,Model=Model,parser=parser,args=args,**kwargs)
        finally:
            env.Destroy()
            RaveDestroy()

def run(*args,**kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    VisibilityModel.RunFromParser(*args,**kwargs)
