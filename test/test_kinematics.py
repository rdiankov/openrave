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
_epsilon = 1e-7

def setup_module(module):
    RaveInitialize(load_all_plugins=True)
    
def teardown_module(module):
    RaveDestroy()

def transdist(list0,list1):
    return sum([sum(abs(item0-item1)) for item0, item1 in izip(list0,list1)])

def randtrans():
    T = matrixFromAxisAngle(random.rand(3)*6-3)
    T[0:3,3] = random.rand(3)-0.5            
    return T

def randlimits(lower,upper):
    return lower+random.rand(len(lower))*(upper-lower)

_envfiles = ['data/lab1.env.xml','data/pr2wam_test1.env.xml','data/hanoi_complex.env.xml']
_robotfiles = ['robots/pr2-beta-static.zae','robots/barrettsegway.robot.xml','robots/neuronics-katana.zae']

class TestKinematics():
    def setup(self):
        self.env=Environment()
    def teardown(self):
        self.env.Destroy()
        self.env=None

    def test_bodybasic(self):
        """checks if the joint/link set/get space conversion commands are consistent"""
        for envfile in _envfiles:
            self.env.Reset()
            self.env.Load(envfile)
            for i in range(10):
                T = eye(4)
                for body in self.env.GetBodies():
                    Told = body.GetTransform()
                    Tallold = body.GetBodyTransformations()
                    dofvaluesold = body.GetDOFValues()
                    T = randtrans()
                    body.SetTransform(T)
                    assert( transdist(T,body.GetTransform()) <= _epsilon )
                    body.SetBodyTransformations(Tallold)
                    assert( transdist(Tallold,body.GetBodyTransformations()) <= _epsilon )
                    Tallnew = [randtrans() for j in range(len(Tallold))]
                    body.SetBodyTransformations(Tallnew)
                    assert( transdist(Tallnew,body.GetBodyTransformations()) <= _epsilon )
                    for link, T in izip(body.GetLinks(),Tallold):
                        link.SetTransform(T)
                    assert( transdist(Tallold,body.GetBodyTransformations()) <= _epsilon )
                    # dof
                    assert( transdist(dofvaluesold,body.GetDOFValues()) <= _epsilon )
                    dofvaluesnew = randlimits(*body.GetDOFLimits())
                    body.SetDOFValues(dofvaluesnew)
                    assert( transdist(dofvaluesnew,body.GetDOFValues()) <= _epsilon )
                    Tallnew = body.GetBodyTransformations()
                    body.SetTransformWithDOFValues(body.GetTransform(),dofvaluesnew)
                    assert( transdist(Tallnew,body.GetBodyTransformations()) <= _epsilon )
                    assert( transdist(dofvaluesnew,body.GetDOFValues()) <= _epsilon )
                    for joint in body.GetJoints():
                        assert( transdist(joint.GetValues(), dofvaluesnew[joint.GetDOFIndex():(joint.GetDOFIndex()+joint.GetDOF())]) <= _epsilon )
                    body.SetBodyTransformations(Tallold)
                    # velocities
                    oldlinkvels = body.GetLinkVelocities()
                    dofvelnew = randlimits(*body.GetDOFVelocityLimits())
                    link0vel = [random.rand(3)-0.5,random.rand(3)-0.5]
                    body.SetVelocity(*link0vel)
                    assert( sum(abs(body.GetDOFVelocities())) <= _epsilon )
                    body.SetDOFVelocities(dofvelnew,*link0vel,checklimits=True)
                    linkvels = body.GetLinkVelocities()
                    assert( transdist(body.GetDOFVelocities(),dofvelnew) <= _epsilon )
                    body.SetDOFVelocities(dofvelnew,linear=[0,0,0],angular=[0,0,0],checklimits=True)
                    newlinkvels = random.rand(*linkvels.shape)-0.5
                    for link,vel in izip(body.GetLinks(),newlinkvels):
                        link.SetVelocity(vel[0:3],vel[3:6])
                    assert( transdist(body.GetLinkVelocities(),newlinkvels) <= _epsilon )
                    body.SetDOFVelocities(body.GetDOFVelocities(),newlinkvels[0][0:3],newlinkvels[0][3:6],checklimits=False)
                    print body.GetLinkVelocities()
                    print newlinkvels
                    assert( transdist(body.GetLinkVelocities(),newlinkvels) <= _epsilon )
                    for joint in body.GetJoints():
                        assert( transdist(joint.GetVelocities(), dofvelnew[joint.GetDOFIndex():(joint.GetDOFIndex()+joint.GetDOF())]) <= _epsilon )
                    for link,vel in izip(body.GetLinks(),newlinkvels):
                        assert( transdist(link.GetVelocity(),[vel[0:3],vel[3:6]]) <= _epsilon )
    def test_jacobian(self):
        for envfile in _envfiles:
            #self.env.Reset()
            #self.env.Load(envfile)
            for i in range(10):
                pass


def test_transformations():
    for i in range(100):
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
        assert(sum(abs(R0-R1)) <= _epsilon)
        assert(abs(sum(quat0**2)-1) <= _epsilon)
        assert(sum(abs(linalg.inv(R0)-R0.transpose())))
        assert(sum(abs(linalg.inv(T0[0:3,0:3])-R0[0:3,0:3])))
        assert(sum(abs(T0-T1)) <= _epsilon and sum(abs(T0-T2)) <= _epsilon and sum(abs(T0-T3)) <= _epsilon)
        assert(abs(abs(dot(quat0,quat1))-1) <= _epsilon)
        assert(sum(abs(axisangle0-axisangle1)) <= _epsilon and sum(abs(axisangle0-axisangle2)) <= _epsilon)
        # test multiplication
        X = random.rand(10,3)-0.5
        Xnew = quatRotateArrayT(quat0,X)
        assert( sum(abs(Xnew-dot(X,R0.transpose()))) <= _epsilon )
        assert( sum(abs(transformPoints(T0,X)-Xnew-tile(trans,(len(X),1)))) <= _epsilon )
        assert( sum(abs(rotationMatrixFromQuat(quatMult(quat0,quat0)) - dot(R0,R0))) <= _epsilon )
