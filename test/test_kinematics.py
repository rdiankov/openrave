#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
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
from openravepy import *
from numpy import *
from itertools import izip, combinations
from optparse import OptionParser
import nose

_multiprocess_can_split_ = True

def setup_module(module):
    RaveInitialize(load_all_plugins=True)
    
def teardown_module(module):
    RaveDestroy()

class TestKinematics():
    def setup(self):
        self.env=Environment()
    def teardown(self):
        self.env.Destroy()
        self.env=None
        
    def test_transformations(self):
        N = 100
        epsilon = 1e-7
        for iter in range(N):
            axisangle0 = (random.rand(3)-0.5)*1.99*pi/sqrt(3) # cannot have mag more than pi
            trans = random.rand(3)-0.5
            R0 = rotationMatrixFromAxisAngle(axisangle0)
            quat0 = quatFromAxisAngle(axisangle0)
            axisangle1 = axisAngleFromQuat(quat0)
            axisangle2 = axisAngleFromRotationMatrix(R0)
            R1 = rotationMatrixFromQuat(quat0)
            quat1 = quatFromRotationMatrix(R0)
            T0 = matrixFromAxisAngle(axisangle1)
            T0[0:3,3] = trans
            pose0 = poseFromMatrix(T0)
            T1 = matrixFromPose(pose0)
            poses = poseFromMatrices([T0,T1])
            T2,T3 = matrixFromPoses(poses)
            assert(sum(abs(R0-R1)) < epsilon)
            assert(sum(abs(T0-T1)) < epsilon and sum(abs(T0-T2)) < epsilon and sum(abs(T0-T3)) < epsilon)
            assert(abs(abs(dot(quat0,quat1))-1) < epsilon)
            assert(sum(abs(axisangle0-axisangle1)) < epsilon and sum(abs(axisangle0-axisangle2)) < epsilon)
            
    def test_transform1(self):
        self.env.Load('data/lab1.env.xml')
        T = eye(4)
        for body in self.env.GetBodies():
            Told = body.GetTransform()
            T = matrixFromAxisAngle(random.rand(3)*6-3)
            T[0:3,3] = random.rand(3)-0.5
            body.SetTransform(T)
        print 'transform1',len(self.env.GetBodies())
        
    def test_transform2(self):
        robot=self.env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
        self.env.AddRobot(robot)
        print 'transform2',len(self.env.GetBodies())
        
