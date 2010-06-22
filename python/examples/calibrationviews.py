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
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import sys, os, time, signal, threading
import numpy # nice to be able to explicitly call some functions
from numpy import *
from optparse import OptionParser
from openravepy import *
from openravepy.interfaces import BaseManipulation
from openravepy.databases import inversekinematics,visibilitymodel

class CalibrationViews(metaclass.AutoReloader):
    def __init__(self,robot,sensorname,target=None,maxvelmult=None,randomize=False):
        self.env = robot.GetEnv()
        self.robot = robot
        self.basemanip = BaseManipulation(self.robot,maxvelmult=maxvelmult)
        if target is None:
            target = self.env.GetKinBody('calibration')
        if randomize and target is not None:
            pose = poseFromMatrix(target.GetTransform())
            target.SetTransform(pose)
        self.vmodel = visibilitymodel.VisibilityModel(robot=robot,target=target,sensorname=sensorname)

    def computevisibilityposes(self,anglerange=pi/3,dists=arange(0.05,1.0,0.15),angledensity=1,num=inf):
        """sample the transformations of the camera. the camera x and y axes should always be aligned with the 
        xy axes of the calibration pattern.
        """
        with self.env:
            values=self.robot.GetJointValues()
            self.vmodel.preshapes=array([values[self.vmodel.manip.GetGripperJoints()]])
            self.vmodel.preprocess()
            dirs,indices = ComputeGeodesicSphereMesh(level=angledensity)
            targetright = self.vmodel.target.GetTransform()[0:3,0]
            targetdir = self.vmodel.target.GetTransform()[0:3,2]
            dirs = dirs[dot(dirs,targetdir)>=cos(anglerange)]
            with self.vmodel.target:
                Ttarget = self.vmodel.target.GetTransform()
                self.vmodel.target.SetTransform(eye(4))
                ab=self.vmodel.target.ComputeAABB()
            centers = transformPoints(Ttarget,dot(array(((0,0,0),(0.5,0.5,0),(-0.5,0.5,0),(0.5,-0.5,0),(-0.5,-0.5,0))),diag(ab.extents())))
            Rs = []
            for dir in dirs:
                right=targetright-dir*dot(targetright,dir)
                right/=sqrt(sum(right**2))
                Rs.append(c_[right,cross(dir,right),dir])
            poses = []
            configs = []
            for R in Rs:
                quat=quatFromRotationMatrix(R)
                for dist in dists:
                    for center in centers:
                        pose = r_[quat,center-dist*R[0:3,2]]
                        try:
                            q=self.vmodel.visualprob.ComputeVisibleConfiguration(target=self.vmodel.target,pose=pose)
                            poses.append(pose)
                            configs.append(q)
                            if len(poses) > num:
                                return array(poses), array(configs)
                        except planning_error:
                            pass
            return array(poses), array(configs)

    def computelocalposes(self,maxangle = 0.5,maxdist = 0.15,averagedist=0.03,angledelta=0.2,**kwargs):
        with self.env:
            # sample a cone around -z
            localpositions = SpaceSampler().sampleR3(averagedist=averagedist,boxdims=[2*maxdist,2*maxdist,maxdist])
            localpositions -= maxdist
            angles = arctan2(sqrt(localpositions[:,0]**2+localpositions[:,1]**2),-localpositions[:,2])
            localpositions = localpositions[angles<maxangle]
            Tsensor = self.vmodel.attachedsensor.GetTransform()
            manip = self.vmodel.manip
            self.robot.SetActiveManipulator(manip)
            positions = transformPoints(Tsensor, localpositions)
            Tcameratogripper = dot(linalg.inv(Tsensor),manip.GetEndEffectorTransform())
            configs = [self.robot.GetJointValues()[manip.GetArmJoints()]]
            poses = [poseFromMatrix(manip.GetEndEffectorTransform())]
            Trotations = [eye(4),matrixFromAxisAngle([angledelta,0,0]),matrixFromAxisAngle([-angledelta,0,0]),matrixFromAxisAngle([0,angledelta,0]),matrixFromAxisAngle([0,-angledelta,0])]
            for position in positions:
                Tsensor[0:3,3] = position
                for Trotation in Trotations:
                    T=dot(dot(Tsensor,Tcameratogripper),Trotation)
                    config=manip.FindIKSolution(T,True)
                    if config is not None:
                        configs.append(config)
                        poses.append(poseFromMatrix(T))
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
        # order the poses with respect to distance
        poseorder=arange(len(poses))
        observations=[]
        while len(poseorder) > 0:
            with self.robot:
                curconfig=self.robot.GetJointValues()[self.vmodel.manip.GetArmJoints()]
                index=argmin(sum((configs[poseorder]-tile(curconfig,(len(poseorder),1)))**2,1))
            config=configs[poseorder[index]]
            data=self.moveToConfiguration(config,waitcond=waitcond)
            if data is not None:
                with self.robot:
                    data['jointvalues'] = self.robot.GetJointValues()[self.vmodel.manip.GetArmJoints()]
                    data['Tlink'] = self.vmodel.attachedsensor.GetAttachingLink().GetTransform()
                observations.append(data)
                if len(observations) >= maxobservations:
                    break
                # prune the nearby observations
                allposes = poses[poseorder]
                quatdist = quatArrayTDist(allposes[index,0:4],allposes[:,0:4])
                transdist= sqrt(sum((allposes[:,4:7]-tile(allposes[index,4:7],(len(allposes),1)))**2,1))
                poseorder = poseorder[0.3*quatdist+transdist > posedist]
            else:
                poseorder = delete(poseorder,index) # just prune this one since the real pattern might be a little offset
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
            if target is None and type:
                target = env.ReadKinBodyXMLFile(type)
                if target:
                    env.AddKinBody(target,True)
                    env.UpdatePublishedBodies()
        self = CalibrationViews(robot=robot,sensorname=sensorname,target=target)
        if target:
            target.SetTransform(dot(self.vmodel.attachedsensor.GetTransform(),T))
        return self.computeAndMoveToObservations(waitcond=waitcond,**kwargs), self.vmodel.target

def run():
    parser = OptionParser(description='Views a calibration pattern from multiple locations.')
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/pa10calib.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--sensorname',action="store",type='string',dest='sensorname',default='wristcam',
                      help='Name of the sensor whose views to generate (default=%default)')
    parser.add_option('--norandomize', action='store_false',dest='randomize',default=True,
                      help='If set, will not randomize the bodies and robot position in the scene.')
    parser.add_option('--novisibility', action='store_false',dest='usevisibility',default=True,
                      help='If set, will not perform any visibility searching.')
    parser.add_option('--posedist',action="store",type='float',dest='posedist',default=0.05,
                      help='An average distance between gathered poses. The smaller the value, the more poses robot will gather close to each other')
    (options, args) = parser.parse_args()

    env = Environment()
    try:
        env.SetViewer('qtcoin')
        env.Load(options.scene)
        robot = env.GetRobots()[0]
        env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        self = CalibrationViews(robot,sensorname=options.sensorname,randomize=options.randomize)
        self.computeAndMoveToObservations(usevisibility=options.usevisibility,posedist=options.posedist)
        raw_input('press any key to exit... ')
    finally:
        env.Destroy()

if __name__ == "__main__":
    run()
