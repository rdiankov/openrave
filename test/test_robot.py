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
import nose
from common_test_openrave import *

_multiprocess_can_split_ = True

class TestRobot(EnvironmentSetup):
    def test_dualarm_grabbing(self):
        with self.env:
            robot = self.env.ReadRobotXMLFile('robots/schunk-lwa3-dual.robot.xml')
            self.env.AddRobot(robot)
            body = self.env.ReadKinBodyXMLFile('data/box3.kinbody.xml')
            self.env.AddKinBody(body)
            T = eye(4)
            T[1,3] = -1.18
            T[2,3] = 0.712
            body.SetTransform(T)
            robot.SetActiveManipulator('leftarm')
            assert(self.env.CheckCollision(robot))
            robot.Grab(body)
            assert(not self.env.CheckCollision(robot))
            robot.SetDOFValues(array([  0.00000000e+00,  -1.43329144e+00,  -3.99190831e-15, -1.86732388e+00,   5.77239752e-01,  -3.37631690e-07, 6.67713991e-08,   0.00000000e+00,  -1.70089030e+00, -6.42544150e-01,  -1.25030589e+00,  -3.33493233e-08, -5.58212676e-08,   1.60115015e-08]))
            assert(robot.CheckSelfCollision())


# def test_ikgeneration():
#     import inversekinematics
#     env = Environment()
#     env.SetDebugLevel(DebugLevel.Debug)
#     #robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
#     robot = env.ReadRobotXMLFile('robots/barrettwam4.robot.xml')
#     robot.SetActiveManipulator('arm')
#     env.AddRobot(robot)
#     self = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)
# 
#     freejoints=None
#     usedummyjoints=False
#     accuracy = None
#     precision = None
#     iktype=inversekinematics.InverseKinematicsModel.Type_Direction3D
#     self.generate(freejoints=freejoints,usedummyjoints=usedummyjoints,iktype=iktype)
# 
#     baselink=self.manip.GetBase().GetIndex()
#     eelink = self.manip.GetEndEffector().GetIndex()
#     solvejoints=solvejoints
#     freeparams=freejoints
#     usedummyjoints=usedummyjoints
#     solvefn=solvefn
# 
# def test_handstraight_jacobian():
#     env = Environment()
#     robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
#     env.AddRobot(robot)
#     # use jacobians for validation
#     with env:
#         deltastep = 0.01
#         thresh=1e-5
#         lower,upper = robot.GetDOFLimits()
#         manip = robot.GetActiveManipulator()
#         ilink = manip.GetEndEffector().GetIndex()
#         localtrans = [0.1,0.2,0.3]
#         localquat = [1.0,0.0,0.0,0.0]
#         stats = []
#         robot.SetActiveDOFs(manip.GetArmIndices())
#         while True:
#             robot.SetDOFValues(random.rand()*(upper-lower)+lower)
#             if not robot.CheckSelfCollision() and not env.CheckCollision(robot):
#                 break
#         deltatrans = deltastep*(random.rand(3)-0.5)
#         while True:
#             values = robot.GetDOFValues(manip.GetArmIndices())
#             T = manip.GetEndEffectorTransform()
#             Tnew = array(T)
#             Tnew[0:3,3] += deltatrans
#             sols = manip.FindIKSolutions(Tnew,IkFilterOptions.CheckEnvCollisions)
#             if len(sols) == 0:
#                 break
#             dists = sum( (array(sols)-tile(values,(len(sols),1)))**2, 1)
#             sol = sols[argmin(dists)]            
#             J = robot.CalculateActiveJacobian(ilink,manip.GetEndEffectorTransform()[0:3,3])
#             Jrot = robot.CalculateActiveAngularVelocityJacobian(ilink)
#             Jtrans = r_[J,Jrot]
#             JJt = dot(Jtrans,transpose(Jtrans))
#             deltavalues = dot(transpose(Jtrans),dot(linalg.inv(JJt),r_[deltatrans,0,0,0]))
#             #dtt = dot(r_[deltatrans,0,0,0],JJt)
#             #alpha = dot(r_[deltatrans,0,0,0], dtt)/dot(dtt,dtt)
#             #deltavalues = alpha*dot(transpose(Jtrans),r_[deltatrans,0,0,0])
#             realvalues = sol-values
#             realtransdelta = dot(J,realvalues)
#             #err = sum(abs(sign(deltatrans)-sign(realtransdelta)))
#             err = dot(deltatrans,realtransdelta)/(linalg.norm(deltatrans)*linalg.norm(realtransdelta))
#             d = sqrt(sum(realvalues**2)/sum(deltavalues**2))
#             if err < 0.95 or d > 10:
#                 print realvalues
#                 print deltavalues
#             stats.append((err,d))
#             print stats[-1]
#             robot.SetDOFValues(sol,manip.GetArmIndices())
# 
#     import matplotlib.pyplot as plt
#     fig = plt.figure()
#     ax = fig.add_subplot(111)
#     ax.hist(stats,100)
#     fig.show()
# 
# def test_ik():
#     import inversekinematics
#     env = Environment()
#     env.SetDebugLevel(DebugLevel.Debug)
#     robot = env.ReadRobotXMLFile('/home/rdiankov/ros/honda/binpicking/robots/tx90.robot.xml')
#     env.AddRobot(robot)
#     manip=robot.GetActiveManipulator()
#     #manip=robot.SetActiveManipulator('leftarm_torso')
#     self = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
#     self.load()
#     self.perftiming(10)
#     robot.SetDOFValues([-2.62361, 1.5708, -0.17691, -3.2652, 0, -3.33643],manip.GetArmJoints())
#     T=manip.GetEndEffectorTransform()
#     print robot.CheckSelfCollision()
#     #[j.SetJointLimits([-pi],[pi]) for j in robot.GetJoints()]
#     robot.SetDOFValues(zeros(robot.GetDOF()))
#     values=manip.FindIKSolution(T,False)
#     Tlocal = dot(dot(linalg.inv(manip.GetBase().GetTransform()),T),linalg.inv(manip.GetGraspTransform()))
#     print ' '.join(str(f) for f in Tlocal[0:3,0:4].flatten())
#     robot.SetDOFValues (values,manip.GetArmJoints())
#     print manip.GetEndEffectorTransform()
#     
#     sols=manip.FindIKSolutions(T,False)
#     for i,sol in enumerate(sols):
#         robot.SetDOFValues(sol)
#         Tnew = manip.GetEndEffectorTransform()
#         if sum((Tnew-T)**2) > 0.0001:
#             print i
#             break
#         
# def debug_ik():
#     env = Environment()
#     env.Load('data/katanatable.env.xml')
#     env.StopSimulation()
#     robot = env.GetRobots()[0]
# 
#     print robot.GetTransform()[0:3,3]
#     target=array([-0.34087322,  0.64355438,  1.01439696])
#     ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Translation3D)
#     if not ikmodel.load():
#         ikmodel.autogenerate()
#     sol = ikmodel.manip.FindIKSolution(IkParameterization(target,IkParameterization.Type.Translation3D),IkFilterOptions.CheckEnvCollisions)
#     print sol
#     robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
#     print linalg.norm(target - ikmodel.manip.GetEndEffectorTransform()[0:3,3])
