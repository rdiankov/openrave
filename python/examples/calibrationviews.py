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
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'
__docformat__ = "restructuredtext en"

import sys, os, time, signal, threading
import numpy # nice to be able to explicitly call some functions
from numpy import *
from optparse import OptionParser
from openravepy import *
from openravepy.interfaces import BaseManipulation
from openravepy.databases import inversekinematics,visibilitymodel

class CalibrationViews(metaclass.AutoReloader):
    def __init__(self,robot,sensorname=None,sensorrobot=None,target=None,maxvelmult=None,randomize=False):
        """Starts a calibration sequencer using a robot and a sensor.

        The *minimum needed* to be **specified** is the `robot` and a ``sensorname``. Supports camera sensors that do not belong to the current robot, in this case the IK is done assuming the target is grabbed by the active manipulator of the robot
        Can use the visibility information of the target.

        :param sensorrobot: If specified, used to determine what robot the sensor lies on.
        """
        self.env = robot.GetEnv()
        self.robot = robot
        self.basemanip = BaseManipulation(self.robot,maxvelmult=maxvelmult)
        if target is None:
            target = self.env.GetKinBody('calibration')
        if randomize and target is not None:
            pose = poseFromMatrix(target.GetTransform())
            target.SetTransform(pose)
        self.vmodel = visibilitymodel.VisibilityModel(robot=robot,sensorrobot=sensorrobot,target=target,sensorname=sensorname)
        self.vmodel.load()
        self.Tpatternrobot = None
        if self.vmodel.robot != self.vmodel.sensorrobot and target is not None:
            print 'Assuming target \'%s\' is attached to %s'%(target.GetName(),self.vmodel.manip)
            self.Tpatternrobot = dot(linalg.inv(self.vmodel.target.GetTransform()),self.vmodel.manip.GetEndEffectorTransform())

    def computevisibilityposes(self,dists=arange(0.05,1.0,0.15),orientationdensity=1,num=inf):
        """Computes robot poses using visibility information from the target.

        Sample the transformations of the camera. the camera x and y axes should always be aligned with the 
        xy axes of the calibration pattern.
        """
        with self.vmodel.target:
            if not self.vmodel.has():
                # nothing is loaded
                self.vmodel.visibilitytransforms = self.vmodel.visualprob.ProcessVisibilityExtents(numrolls=1,sphere=[orientationdensity]+dists.tolist())
                self.vmodel.preshapes=array([self.robot.GetDOFValues(self.vmodel.manip.GetGripperJoints())])
                self.vmodel.preprocess()
            if self.Tpatternrobot is not None:
                self.vmodel.target.SetTransform(dot(self.vmodel.manip.GetEndEffectorTransform(),linalg.inv(self.Tpatternrobot)))
            with RobotStateSaver(self.robot,KinBody.SaveParameters.GrabbedBodies):
                with KinBodyStateSaver(self.vmodel.target,KinBody.SaveParameters.LinkTransformation):
                    self.vmodel.target.SetTransform(eye(4))
                    ab=self.vmodel.target.ComputeAABB()
                centers = dot(array(((0,0,0),(0.5,0.5,0),(-0.5,0.5,0),(0.5,-0.5,0),(-0.5,-0.5,0))),diag(ab.extents()))
                if self.Tpatternrobot is not None:
                    self.robot.Grab(self.vmodel.target,self.vmodel.manip.GetEndEffector())
                    Tbase = self.vmodel.attachedsensor.GetTransform()
                    visibilitytransforms = invertPoses(self.vmodel.visibilitytransforms)
                else:
                    Tbase = self.vmodel.target.GetTransform()
                    visibilitytransforms = self.vmodel.visibilitytransforms
                posebase = poseFromMatrix(Tbase)
                poses = []
                configs = []
                for relativepose in visibilitytransforms:
                    for center in centers:
                        if self.Tpatternrobot is not None:
                            pose = array(posebase)
                            pose[4:7] += quatRotate(pose[0:4],center)
                            pose = poseMult(pose,relativepose)
                        else:
                            pose = poseMult(posebase,relativepose)
                            pose[4:7] += quatRotate(pose[0:4],center)
                        try:
                            q=self.vmodel.visualprob.ComputeVisibleConfiguration(pose=pose)
                            poses.append(pose)
                            configs.append(q)
                            if len(poses) > num:
                                return array(poses), array(configs)
                        except planning_error:
                            pass
                return array(poses), array(configs)

    def computelocalposes(self,maxconeangle = 0.5,maxconedist = 0.15,averagedist=0.03,angledelta=0.2,**kwargs):
        """Computes robot poses using a cone pointing to the negative z-axis of the camera

        """
        with self.env:
            localpositions = SpaceSampler().sampleR3(averagedist=averagedist,boxdims=[2*maxconedist,2*maxconedist,maxconedist])
            localpositions -= maxconedist
            angles = arctan2(sqrt(localpositions[:,0]**2+localpositions[:,1]**2),-localpositions[:,2])
            localpositions = localpositions[angles<maxconeangle]
            Tsensor = self.vmodel.attachedsensor.GetTransform()
            manip = self.vmodel.manip
            self.robot.SetActiveManipulator(manip)
            positions = transformPoints(Tsensor, localpositions)
            Tcameratogripper = dot(linalg.inv(Tsensor),manip.GetEndEffectorTransform())
            configs = [self.robot.GetDOFValues(manip.GetArmJoints())]
            poses = [poseFromMatrix(manip.GetEndEffectorTransform())]
            Trotations = [eye(4),matrixFromAxisAngle([angledelta,0,0]),matrixFromAxisAngle([-angledelta,0,0]),matrixFromAxisAngle([0,angledelta,0]),matrixFromAxisAngle([0,-angledelta,0])]
            for position in positions:
                Tsensor[0:3,3] = position
                for Trotation in Trotations:
                    T=dot(dot(Tsensor,Trotation),Tcameratogripper)
                    config=manip.FindIKSolution(T,True)
                    if config is not None:
                        configs.append(config)
                        poses.append(poseFromMatrix(dot(Tsensor,Trotation)))
        return array(poses), array(configs)

    def computeAndMoveToObservations(self,waitcond=None,maxobservations=inf,posedist=0.05,usevisibility=True,**kwargs):
        """Computes several configuration for the robot to move. If usevisibility is True, will use the visibility model of the pattern to gather data.
        Otherwise, given that the pattern is currently detected in the camera, move the robot around the local neighborhood. This does not rely on the visibiliy information of the pattern and does not create a pattern
        """
        if usevisibility:
            poses,configs = self.computevisibilityposes(**kwargs)
        else:
            poses,configs = self.computelocalposes(**kwargs)                
        graphs = [self.env.drawlinelist(array([pose[4:7],pose[4:7]+0.05*rotationMatrixFromQuat(pose[0:4])[0:3,2]]),1) for pose in poses]
        try:
            return self.moveToObservations(poses,configs,waitcond=waitcond,maxobservations=maxobservations,posedist=posedist)
        finally:
            graphs = None

    def moveToObservations(self,poses,configs,waitcond=None,maxobservations=inf,posedist=0.05):
        """
        """
        # order the poses with respect to distance
        assert len(poses) == len(configs)
        poseorder=arange(len(poses))
        observations=[]
        with RobotStateSaver(self.robot,KinBody.SaveParameters.GrabbedBodies):
            if self.Tpatternrobot is not None:
                with self.env:
                    self.robot.Grab(self.vmodel.target,self.vmodel.manip.GetEndEffector())
            while len(poseorder) > 0:
                with self.robot:
                    curconfig=self.robot.GetDOFValues(self.vmodel.manip.GetArmJoints())
                index=argmin(sum((configs[poseorder]-tile(curconfig,(len(poseorder),1)))**2,1))
                config=configs[poseorder[index]]
                try:
                    data=self.moveToConfiguration(config,waitcond=waitcond)
                    if data is not None:
                        with self.robot:
                            data['jointvalues'] = self.robot.GetDOFValues(self.vmodel.manip.GetArmJoints())
                            data['Tlink'] = self.vmodel.attachedsensor.GetAttachingLink().GetTransform()
                        observations.append(data)
                        if len(observations) >= maxobservations:
                            break
                        # prune the nearby observations
                        allposes = poses[poseorder]
                        quatdist = quatArrayTDist(allposes[index,0:4],allposes[:,0:4])
                        transdist= sqrt(sum((allposes[:,4:7]-tile(allposes[index,4:7],(len(allposes),1)))**2,1))
                        poseorder = poseorder[0.2*quatdist+transdist > posedist]
                    else:
                        poseorder = delete(poseorder,index) # just prune this one since the real pattern might be a little offset
                except planning_error:
                    pass
        return observations

    def moveToConfiguration(self,config,waitcond=None):
        """moves the robot to a configuration"""
        with self.env:
            self.robot.SetActiveDOFs(self.vmodel.manip.GetArmJoints())
            self.basemanip.MoveActiveJoints(config)
        while not self.robot.GetController().IsDone():
            time.sleep(0.01)
        if waitcond:
            return waitcond()
    def viewVisibleConfigurations(self,**kwargs):
        poses,configs = self.createvisibility(**kwargs)
        graphs = [self.env.drawlinelist(array([pose[4:7],pose[4:7]+0.03*rotationMatrixFromQuat(pose[0:4])[0:3,2]]),1) for pose in poses]
        try:
            with self.robot:
                for i,config in enumerate(configs):
                    self.robot.SetJointValues(config,self.vmodel.manip.GetArmJoints())
                    self.env.UpdatePublishedBodies()
                    raw_input('%d: press any key'%i)
        finally:
            graphs = None

    @staticmethod
    def gatherCalibrationData(robot,sensorname,waitcond,target=None,**kwargs):
        """function to gather calibration data, relies on an outside waitcond function to return information about the calibration pattern"""
        env=robot.GetEnv()
        data=waitcond()
        if data is not None and 'T' in data:
            T=data['T']
            type = data.get('type',None)
            if target is None and type is not None:
                if type[0] == '<': # test for XML
                    target = env.ReadKinBodyXMLData(type)
                else:
                    target = env.ReadKinBodyXMLFile(type)
                if target is not None:
                    env.AddKinBody(target,True)
                    env.UpdatePublishedBodies()
        self = CalibrationViews(robot=robot,sensorname=sensorname,target=target)
        if target:
            target.SetTransform(dot(self.vmodel.attachedsensor.GetTransform(),T))
        return self.computeAndMoveToObservations(waitcond=waitcond,**kwargs), self.vmodel.target

def run(args=None):
    """Executes the calibrationviews example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Views a calibration pattern from multiple locations.')
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/pa10calib.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--sensorname',action="store",type='string',dest='sensorname',default=None,
                      help='Name of the sensor whose views to generate (default is first sensor on robot)')
    parser.add_option('--sensorrobot',action="store",type='string',dest='sensorrobot',default=None,
                      help='Name of the robot the sensor is attached to (default=%default)')
    parser.add_option('--norandomize', action='store_false',dest='randomize',default=True,
                      help='If set, will not randomize the bodies and robot position in the scene.')
    parser.add_option('--novisibility', action='store_false',dest='usevisibility',default=True,
                      help='If set, will not perform any visibility searching.')
    parser.add_option('--posedist',action="store",type='float',dest='posedist',default=0.05,
                      help='An average distance between gathered poses. The smaller the value, the more poses robot will gather close to each other')
    (options, args) = parser.parse_args(args=args)

    env = Environment()
    try:
        env.SetViewer('qtcoin')
        env.Load(options.scene)
        robot = env.GetRobots()[0]
        sensorrobot = None if options.sensorrobot is None else env.GetRobot(options.sensorrobot)
        env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        self = CalibrationViews(robot,sensorname=options.sensorname,sensorrobot=sensorrobot,randomize=options.randomize)
        self.computeAndMoveToObservations(usevisibility=options.usevisibility,posedist=options.posedist)
        raw_input('press any key to exit... ')
    finally:
        env.Destroy()

if __name__ == "__main__":
    run()

