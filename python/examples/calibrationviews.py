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
from openravepy.databases import visibilitymodel

class CalibrationViews(metaclass.AutoReloader):
    def __init__(self,robot,sensorname,target=None,maxvelmult=None,randomize=False):
        self.env = robot.GetEnv()
        self.robot = robot
        self.basemanip = BaseManipulation(self.robot,maxvelmult=maxvelmult)
        if target is None:
            target = self.env.GetKinBody('calibration')
        if randomize:
            pose = poseFromMatrix(target.GetTransform())
            target.SetTransform(pose)
        self.vmodel = visibilitymodel.VisibilityModel(robot=robot,target=target,sensorname=sensorname)
    def createvisibility(self,anglerange=pi/3,maxdist=1.0,angledensity=1,num=inf):
        """
        sample the transformations of the camera. the camera x and y axes should always be aligned with the 
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
            dists = arange(0.05,maxdist,0.05)
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
    def moveToObservations(self,waitcond=None,maxobservations=inf,posedist=0.2,**kwargs):
        """moves robot to all visible configurations"""
        poses,configs = self.createvisibility(**kwargs)

        graphs = [self.env.drawlinelist(array([pose[4:7],pose[4:7]+0.05*rotationMatrixFromQuat(pose[0:4])[0:3,2]]),1) for pose in poses]
        try:
            # order the poses with respect to distance
            targetcenter = self.vmodel.target.ComputeAABB().pos()
            poseorder=argsort(-sum((poses[:,4:7]-tile(targetcenter,(len(poses),1)))**2,1))
            observations=[]
            while len(poseorder) > 0:
                config=configs[poseorder[0]]
                data=self.moveToConfiguration(config,waitcond=waitcond)
                if data is not None:
                    data['jointvalues'] = self.robot.GetJointValues()[self.vmodel.manip.GetArmJoints()]
                    data['Tlink'] = self.vmodel.attachedsensor.GetAttachingLink().GetTransform()
                    observations.append(data)
                    if len(observations) >= maxobservations:
                        break
                # prune the observations
                allposes = poses[poseorder]
                quatdist = quatArrayTDist(allposes[0,0:4],allposes[:,0:4])
                transdist= sqrt(sum((allposes[:,4:7]-tile(allposes[0,4:7],(len(allposes),1)))**2,1))
                poseorder = poseorder[0.3*quatdist+transdist > posedist]
            return observations
        finally:
            graphs = None
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
    def gatherCalibrationData(robot,sensorname,waitcond,**kwargs):
        """function to gather calibration data, relies on an outside waitcond function to return information about the calibration pattern"""
        env=robot.GetEnv()
        data=waitcond()
        T=data['T']
        type = data.get('type',None)
        if type:
            target = env.ReadKinBodyXMLFile(type)
            if target:
                env.AddKinBody(target,True)
                env.UpdatePublishedBodies()
        self = CalibrationViews(robot=robot,sensorname=sensorname,target=target)
        if target:
            target.SetTransform(dot(self.vmodel.attachedsensor.GetTransform(),T))
        return self.moveToObservations(waitcond=waitcond,**kwargs), self.vmodel.target

def run():
    parser = OptionParser(description='Views a calibration pattern from multiple locations.')
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/pa10calib.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--sensorname',action="store",type='string',dest='sensorname',default='wristcam',
                      help='Name of the sensor whose views to generate (default=%default)')
    parser.add_option('--norandomize', action='store_false',dest='randomize',default=True,
                      help='If set, will not randomize the bodies and robot position in the scene.')
    (options, args) = parser.parse_args()

    env = Environment()
    try:
        env.SetViewer('qtcoin')
        env.Load(options.scene)
        robot = env.GetRobots()[0]
        env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        self = CalibrationViews(robot,sensorname=options.sensorname,randomize=options.randomize)
        self.moveToObservations()
        raw_input('press any key to exit... ')
    finally:
        env.Destroy()

if __name__ == "__main__":
    run()
