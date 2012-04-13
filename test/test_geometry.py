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
from common_test_openrave import *

def test_transformations():
    log.info('tests basic math transformations')
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
        assert(sum(abs(R0-R1)) <= g_epsilon)
        assert(abs(sum(quat0**2)-1) <= g_epsilon)
        assert(sum(abs(linalg.inv(R0)-R0.transpose())))
        assert(sum(abs(linalg.inv(T0[0:3,0:3])-R0[0:3,0:3])))
        assert(sum(abs(T0-T1)) <= g_epsilon and sum(abs(T0-T2)) <= g_epsilon and sum(abs(T0-T3)) <= g_epsilon)
        assert(abs(abs(dot(quat0,quat1))-1) <= g_epsilon)
        assert(sum(abs(axisangle0-axisangle1)) <= g_epsilon and sum(abs(axisangle0-axisangle2)) <= g_epsilon)
        # test multiplication
        X = random.rand(10,3)-0.5
        Xnew = quatRotateArrayT(quat0,X)
        assert( sum(abs(Xnew-dot(X,R0.transpose()))) <= g_epsilon )
        assert( sum(abs(quatRotate(quat0,X[0]) - Xnew[0])) <= g_epsilon )
        assert( sum(abs(transformPoints(T0,X)-Xnew-tile(trans,(len(X),1)))) <= g_epsilon )
        assert( sum(abs(rotationMatrixFromQuat(quatMult(quat0,quat0)) - dot(R0,R0))) <= g_epsilon )
        assert( sum(abs(dot(T0,T0)-matrixFromPose(poseMult(pose0,pose0)))) <= g_epsilon )
        qarray0 = randquat(5)
        qarray1 = quatArrayTMult(qarray0,quat0)
        assert( sum(abs(quatArrayTRotate(qarray0,Xnew[0])-quatArrayTRotate(qarray1,X[0]))) <= g_epsilon )
        assert( sum(abs(quatArrayRotate(qarray0.transpose(),Xnew[0])-quatArrayRotate(qarray1.transpose(),X[0]))) <= g_epsilon )
        qarray2 = quatMultArrayT(quat0,qarray0)
        assert( sum(abs(quatRotateArrayT(quat0,quatArrayTRotate(qarray0,X[0]))-quatArrayTRotate(qarray2,X[0]))) <= g_epsilon )
        dists = quatArrayTDist(qarray0[0],qarray0)
        assert( all(dists>=0) and sum(dists)>0 and dists[0] <= g_epsilon )
        posearray0 = randpose(5)
        posearray1 = poseMultArrayT(pose0,posearray0)
        assert( sum(abs(poseMult(pose0,posearray0[0])-posearray1[0])) <= g_epsilon )
        for j in range(len(posearray0)):
            poseTransformPoints(pose0,poseTransformPoints(posearray0[j],X))
            poseTransformPoints(posearray1[j],X)
            assert( sum(abs(poseTransformPoints(pose0,poseTransformPoints(posearray0[j],X)) - poseTransformPoints(posearray1[j],X))) <= g_epsilon )
        # inverses
        matrices = matrixFromPoses(posearray0)
        posearrayinv0 = invertPoses(posearray0)
        for j in range(len(posearray0)):
            assert( sum(abs(transformInversePoints(matrices[j],X) - poseTransformPoints(posearrayinv0[j],X))) <= g_epsilon )

        # slightly unnormalized pose
        T = matrixFromPose([ 0.00422863, 0.00522595, 0.707, 0.707182, 0.204229, 0.628939, 1.40061])
        assert(abs(linalg.det(T[0:3,0:3])-1) <= g_epsilon )

def test_quatRotateDirection():
    pairs = [ [[1,0,0], [0,1,0]], [[1,0,0], [1,0,0]], [[1,1,0], [0,1,1]] ]
    for sourcedir, targetdir in pairs:
        T = matrixFromQuat(quatRotateDirection(sourcedir,targetdir))
        assert( transdist(dot(T[0:3,0:3],sourcedir),targetdir) <= g_epsilon )
        
