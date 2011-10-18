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

class TestRobot(EnvironmentSetup):
    def test_dualarm_grabbing(self):
        with self.env:
            robot = self.env.ReadRobotURI('robots/schunk-lwa3-dual.robot.xml')
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

    def test_basic(self):
        with self.env:
            for robotfile in g_robotfiles:
                self.env.Reset()
                robot = self.env.ReadRobotURI(robotfile)
                self.env.AddRobot(robot)
                assert(robot.GetDOF() == robot.GetActiveDOF())

    def test_collisionmaprobot(self):
        env=self.env
        xml = """<environment>
<robot file="robots/collisionmap.robot.xml">
</robot>
</environment>
"""
        env.LoadData(xml)
        with env:
            robot=env.GetRobots()[0]
            assert(robot.GetXMLId().lower()=='collisionmaprobot')
            robot.SetDOFValues([9/180.0*pi,1/180.0*pi],[1,2])
            assert(robot.CheckSelfCollision())
            robot.SetDOFValues([0/180.0*pi,1/180.0*pi],[1,2])
            assert(not robot.CheckSelfCollision())
            env.Reset()
            robot=env.ReadRobotURI('robots/collisionmap.robot.xml')
            env.AddRobot(robot)
            assert(robot.GetXMLId().lower()=='collisionmaprobot')

    def test_grabcollision(self):
        env=self.env
        env.Load('robots/man1.zae') # load a simple scene
        with env:
            robot = env.GetRobots()[0] # get the first robot
            leftarm = robot.GetManipulator('leftarm')
            rightarm = robot.GetManipulator('rightarm')

            env.Load('data/mug1.kinbody.xml'); 
            leftmug = env.GetKinBody('mug')
            env.Load('data/mug2.kinbody.xml')
            rightmug = env.GetKinBody('mug2')

            env.StopSimulation()
            leftMugGrabPose = array([[ 0.99516672, -0.0976999 ,  0.00989374,  0.14321238],
                                     [ 0.09786028,  0.99505007, -0.01728364,  0.94120538],
                                     [-0.00815616,  0.01816831,  0.9998017 ,  0.38686624],
                                     [ 0.        ,  0.        ,  0.        ,  1.        ]])
            leftmug.SetTransform(leftMugGrabPose)
            rightMugGrabPose = array([[  9.99964535e-01,  -1.53668225e-08,   8.41848925e-03, -1.92047462e-01],
                                      [ -8.40134174e-03,  -6.37951940e-02,   9.97927606e-01, 9.22815084e-01],
                                      [  5.37044369e-04,  -9.97963011e-01,  -6.37929291e-02, 4.16847348e-01],
                                      [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])
            rightmug.SetTransform(rightMugGrabPose);
            assert(not env.CheckCollision(leftmug,rightmug))
            
            grabJointAngles = array([ -3.57627869e-07,   0.00000000e+00,  -1.46997878e-15, -1.65528119e+00,  -1.23030146e-08,  -8.41909389e-11, 0.00000000e+00], dtype=float32)

            robot.SetDOFValues(grabJointAngles,rightarm.GetArmIndices())
            robot.SetDOFValues(grabJointAngles,leftarm.GetArmIndices())
            
            robot.SetActiveManipulator(rightarm)
            robot.Grab(rightmug)
            robot.SetActiveManipulator(leftarm)
            robot.Grab(leftmug)

            assert(not robot.CheckSelfCollision())
            assert(not env.CheckCollision(robot))

            print 'Now changing arm joint angles so that the two mugs collide. The checkSelfCollision returns:'
            collisionJointAngles = array([ -2.38418579e-07,   0.00000000e+00,  -2.96873480e-01, -1.65527940e+00,  -3.82479293e-08,  -1.23165381e-10, 1.35525272e-20]);
            robot.SetDOFValues(collisionJointAngles,rightarm.GetArmIndices())
            robot.SetDOFValues(collisionJointAngles,leftarm.GetArmIndices())
            assert(robot.CheckSelfCollision())
            assert(not env.CheckCollision(robot))

            robot.ReleaseAllGrabbed()
            assert(env.CheckCollision(leftmug,rightmug))

    def test_basic(self):
        robot = self.env.ReadRobotURI('robots/schunk-lwa3-dual.robot.xml')
        self.env.AddRobot(robot)
        assert(robot.GetLinks()[0].GetParent().GetActiveDOF() == robot.GetActiveDOF())

    def test_ikcollision(self):
        print 'test if can solve IK during collisions'
        env=self.env
        with env:
            robot = env.ReadRobotURI('robots/pr2-beta-static.zae')
            env.AddRobot(robot)
            target = env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.AddKinBody(target,True)
            T=target.GetTransform()
            T[0:3,3] = [-0.342,0,0.8]
            target.SetTransform(T)
            floor = RaveCreateKinBody(env,'')
            floor.InitFromBoxes(array([[0,0,0,2,2,0.01]]),True)
            floor.SetName('floor')
            env.AddKinBody(floor,True)
            
            assert(env.CheckCollision(robot))
            manip=robot.SetActiveManipulator('leftarm')
            manip2 = robot.GetManipulator('rightarm')
            robot.SetActiveDOFs(manip.GetArmIndices())
            assert(not manip.CheckEndEffectorCollision(manip.GetTransform()))
            assert(not manip2.CheckEndEffectorCollision(manip2.GetTransform()))
            
            robot.SetActiveDOFValues([  0.678,   0.00000000e+00,   1.75604762e+00, -1.74228108e+00,   3.23831570e-16,   0.00000000e+00, 0.00000000e+00])
            assert(not robot.CheckSelfCollision())
            Tmanip = manip.GetTransform()
            robot.SetActiveDOFValues(zeros(robot.GetActiveDOF()))
            assert(manip.FindIKSolution(Tmanip,IkFilterOptions.CheckEnvCollisions) is not None)

            basemanip = interfaces.BaseManipulation(robot)
            out=basemanip.MoveToHandPosition(matrices=[Tmanip],execute=False)
            assert(out is not None)
            
            # self colliding
            robot.SetActiveDOFValues([  2.20622614e-01,   0.00000000e+00,   1.75604762e+00, -1.74228108e+00,   0.00000000e+00,  -9.56775092e-16, 0.00000000e+00])
            assert(robot.CheckSelfCollision())
            Tmanip = manip.GetTransform()
            robot.SetActiveDOFValues(zeros(robot.GetActiveDOF()))
            assert(manip.FindIKSolution(Tmanip,IkFilterOptions.CheckEnvCollisions) is None)
            assert(manip.FindIKSolution(Tmanip,IkFilterOptions.CheckEnvCollisions|IkFilterOptions.IgnoreEndEffectorCollisions) is not None)
            assert(not manip.CheckEndEffectorCollision(Tmanip))

            box = RaveCreateKinBody(env,'')
            box.InitFromBoxes(array([[0,0,0,0.05,0.05,0.2]]),True)
            box.SetName('box')
            env.AddKinBody(box,True)
            box.SetTransform(manip.GetTransform())
            robot.Grab(box)
            robot.SetActiveDOFValues([  0.5,   0.00000000e+00,   1.57, -1.74228108e+00,   3.23831570e-16,   0.00000000e+00, 0.00000000e+00])
            assert(robot.CheckSelfCollision())
            Tmanip = manip.GetTransform()
            robot.SetActiveDOFValues(zeros(robot.GetActiveDOF()))
            assert(manip.FindIKSolution(Tmanip,IkFilterOptions.CheckEnvCollisions|IkFilterOptions.IgnoreEndEffectorCollisions) is not None)
            assert(not manip.CheckEndEffectorCollision(Tmanip))

            robot.SetActiveDOFValues([ 0.00000000e+00,   0.858,   2.95911693e+00, -0.1,   0.00000000e+00,  -3.14018492e-16, 0.00000000e+00])
            Tmanip = manip.GetTransform()
            assert(manip.FindIKSolution(Tmanip,IkFilterOptions.CheckEnvCollisions|IkFilterOptions.IgnoreEndEffectorCollisions) is not None)

            # test if initial colliding attachments are handled correctly
            robot.SetActiveDOFValues(zeros(robot.GetActiveDOF()))
            T = manip.GetTransform()
            T[0,3] += 0.2
            target.SetTransform(T)
            assert(not robot.CheckSelfCollision())
            assert(env.CheckCollision(box,target))
            assert(manip.CheckEndEffectorCollision(manip.GetTransform()))
            assert(not manip2.CheckEndEffectorCollision(manip2.GetTransform()))
            robot.Grab(target)
            assert(robot.IsGrabbing(target))
            assert(not robot.CheckSelfCollision())
            robot.RegrabAll()
            assert(not robot.CheckSelfCollision())

            robot.Release(target)
            assert(not robot.IsGrabbing(target))
            
            box2 = RaveCreateKinBody(env,'')
            box2.InitFromBoxes(array([[0,0,0,0.05,0.05,0.2]]),True)
            box2.SetName('box2')
            env.AddKinBody(box2,True)
            box2.SetTransform(manip2.GetTransform())
            robot.Grab(box2,grablink=manip2.GetEndEffector())
            assert(not manip2.CheckEndEffectorCollision(manip2.GetTransform()))

            robot.Grab(target)
            Tmanip = manip.GetTransform()
            assert(not manip.CheckEndEffectorCollision(Tmanip))
            robot.SetActiveDOFValues([ 0.00000000e+00,   0.858,   2.95911693e+00, -1.57009246e-16,   0.00000000e+00,  -3.14018492e-16, 0.00000000e+00])
            assert(not manip.CheckEndEffectorCollision(Tmanip))

# def test_ikgeneration():
#     import inversekinematics
#     env = Environment()
#     env.SetDebugLevel(DebugLevel.Debug)
#     #robot = env.ReadRobotURI('robots/barrettsegway.robot.xml')
#     robot = env.ReadRobotURI('robots/barrettwam4.robot.xml')
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
#     robot = env.ReadRobotURI('robots/barrettwam.robot.xml')
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
#     robot = env.ReadRobotURI('/home/rdiankov/ros/honda/binpicking/robots/tx90.robot.xml')
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
