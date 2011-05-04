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

def randquat(N=1):
    q = random.rand(N,4)-0.5
    return q/tile(sqrt(sum(q**2,1)),(4,1)).transpose()

def randpose(N=1):
    poses = random.rand(N,7)-0.5
    poses[:,0:4] /= tile(sqrt(sum(poses[:,0:4]**2,1)),(4,1)).transpose()
    return poses

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
    """tests basic math transformations"""
    for i in range(20):
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
        assert( sum(abs(quatRotate(quat0,X[0]) - Xnew[0])) <= _epsilon )
        assert( sum(abs(transformPoints(T0,X)-Xnew-tile(trans,(len(X),1)))) <= _epsilon )
        assert( sum(abs(rotationMatrixFromQuat(quatMult(quat0,quat0)) - dot(R0,R0))) <= _epsilon )
        assert( sum(abs(dot(T0,T0)-matrixFromPose(poseMult(pose0,pose0)))) <= _epsilon )
        qarray0 = randquat(5)
        qarray1 = quatArrayTMult(qarray0,quat0)
        assert( sum(abs(quatArrayTRotate(qarray0,Xnew[0])-quatArrayTRotate(qarray1,X[0]))) <= _epsilon )
        assert( sum(abs(quatArrayRotate(qarray0.transpose(),Xnew[0])-quatArrayRotate(qarray1.transpose(),X[0]))) <= _epsilon )
        qarray2 = quatMultArrayT(quat0,qarray0)
        assert( sum(abs(quatRotateArrayT(quat0,quatArrayTRotate(qarray0,X[0]))-quatArrayTRotate(qarray2,X[0]))) <= _epsilon )
        dists = quatArrayTDist(qarray0[0],qarray0)
        assert( all(dists>=0) and sum(dists)>0 and dists[0] <= _epsilon )                    
        posearray0 = randpose(5)
        posearray1 = poseMultArrayT(pose0,posearray0)
        assert( sum(abs(poseMult(pose0,posearray0[0])-posearray1[0])) <= _epsilon )
        for j in range(len(posearray0)):
            poseTransformPoints(pose0,poseTransformPoints(posearray0[j],X))
            poseTransformPoints(posearray1[j],X)
            assert( sum(abs(poseTransformPoints(pose0,poseTransformPoints(posearray0[j],X)) - poseTransformPoints(posearray1[j],X))) <= _epsilon )
        # inverses
        matrices = matrixFromPoses(posearray0)
        posearrayinv0 = invertPoses(posearray0)
        for j in range(len(posearray0)):
            assert( sum(abs(transformInversePoints(matrices[j],X) - poseTransformPoints(posearrayinv0[j],X))) <= _epsilon )
        
def test_fitcircle():
    perturbation = 0.001
    for i in range(10):
        T = randtrans()
        radius = random.rand()*5+0.1
        angles = random.rand(5)*2*pi
        points = c_[radius*cos(angles)+perturbation*(random.rand(len(angles))-0.5),radius*sin(angles)+perturbation*(random.rand(len(angles))-0.5),perturbation*(random.rand(len(angles))-0.5)]
        newpoints = transformPoints(T,points)
        newcenter, newradius = fitCircle(newpoints)
        assert( sum(abs(T[0:3,3]-newcenter)) <= perturbation*2 )
        assert( sum(abs(radius-newradius)) <= perturbation*2 )
        newpoints2d = points[:,0:2] + T[0:2,3]
        newcenter2d, newradius2d = fitCircle(newpoints2d)
        assert( sum(abs(T[0:2,3]-newcenter2d)) <= perturbation*2 )
        assert( sum(abs(radius-newradius2d)) <= perturbation*2 )
