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
    def createvisibility(self,anglerange=pi/3,maxdist=1.0,angledelta=0.5,num=inf):
        """
        sample the transformations of the camera. the camera x and y axes should always be aligned with the 
        xy axes of the calibration pattern.
        """
        with self.env:
            values=robot.GetJointValues()
            self.vmodel.preshapes=array([values[self.vmodel.manip.GetGripperJoints()]])
            self.vmodel.preprocess()
            dirs = SpaceSampler().sampleS2(angledelta=angledelta)
            dirs = dirs[dirs[:,2]>=cos(anglerange)]
            targetright = self.vmodel.target.GetTransform()[0:3,0]
            with self.vmodel.target:
                Ttarget = self.vmodel.target.GetTransform()
                self.vmodel.target.SetTransform(eye(4))
                ab=self.vmodel.target.ComputeAABB()
            centers = transformPoints(Ttarget,array(((0,0,0),(0.5,0.5,0),(-0.5,0.5),(0.5,-0.5,0),(-0.5,-0.5,0))))
            Rs = []
            for dir in dirs:
                right=targetright-dir*dot(targetright,dir)
                right/=sqrt(sum(right**2))
                Rs.append(c_[right,cross(dir,right),dir])
            dists = arange(0.0,maxdist,0.05)
            poses = []
            configs = []
            for R in Rs:
                q=quatFromRotationMatrix(R)
                for dist in dists:
                    for center in centers:
                        pose = r_[q,center-dist*R[0:3,2]]
                        try:
                            q=ComputeVisibleConfiguration(target=self.target,pose=pose)
                            poses.append(pose)
                            configs.append(q)
                            if len(poses) > num:
                                return array(poses), array(configs)
                        except planning_error:
                            pass
            return array(poses), array(configs)
    def moveToObservations(self,waitcond=None,posedist=**kwargs):
        poses,configs = self.createvisibility(**kwargs)
        # order the poses with respect to distance
        targetcenter = self.vmodel.target.ComputeAABB().pos()
        poseorder=argsort(-sum((poses[:,4:7]-tile(targetcenter,(len(poses),1)))**2))
        while len(poseorder) > 0:
            pass
    def moveToConfiguration(self,config,waitcond=None):
        with self.env:
            self.robot.SetActiveDOFs(self.vmodel.manip.GetArmJoints())
            self.basemanip.MoveActiveJoints(config)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
        if waitcond:
            waitcond()
        
def run():
    parser = OptionParser(description='Views a calibration pattern from multiple locations.')
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/pa10calib.env.xml',
                      help='Scene file to load (default=%default)')
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
        self = CalibrationViews(robot,randomize=options.randomize)
        self.performGraspPlanning()
    finally:
        env.Destroy()

if __name__ == "__main__":
    run()
