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

class TestMoving(EnvironmentSetup):
    def test_constraintpr2(self):
        env = self.env
        robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
        env.AddRobot(robot)
        with env:
            manip=robot.SetActiveManipulator('leftarm_torso')
            basemanip = interfaces.BaseManipulation(robot)
            robot.SetDOFValues([.31],[robot.GetJoint('torso_lift_joint').GetDOFIndex()])
            T=array([[0,0,1,.6], [0,1,0,.1], [-1,0,0,.73], [0,0,0,1]])
            robot.SetDOFValues(manip.FindIKSolution(T,IkFilterOptions.CheckEnvCollisions),manip.GetArmIndices())
            Tgoal=array([[0,0,1,.6], [0,1,0,.3], [-1,0,0,.73], [0,0,0,1]])
            constraintfreedoms=array([1,1,0,1,0,0]) # can rotate along z, translate along y
            constraintmatrix=array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
            for constrainterrorthresh in [0.002,0.01]:
                ret = basemanip.MoveToHandPosition(matrices=[Tgoal],maxiter=6000,maxtries=2,seedik=16, constraintfreedoms=constraintfreedoms, constraintmatrix=constraintmatrix, constrainterrorthresh=constrainterrorthresh,execute=False,outputtraj=True,steplength=0.001)
                assert(ret is not None)

    def test_movehandstraight(self):
        env = self.env
        env.Load('data/lab1.env.xml')
        robot = env.GetRobots()[0]
        basemanip = interfaces.BaseManipulation(robot)

        testvalues = [[array([ -2.83686683e-01,   1.40828054e+00,   0.00000000e+00, 5.26754682e-01,  -3.14159265e+00,  -1.20655743e+00, -1.85448301e+00,   1.66533454e-16,   1.66533454e-16,         1.66533454e-16,   0.00000000e+00]), array([ -9.22429319e-16,  -3.90560499e-15,  -1.00000000e+00])],
                      [array([-0.26085414,  1.37967815,  0.        ,  0.60871186, -3.14159265,       -1.15320264, -0.26085414,  0.        ,  0.        ,  0.        ,  0.        ]), array([ -7.21644966e-16,  -3.28903571e-15,  -1.00000000e+00])],
                      [array([ -5.90848599e-02,   9.54294051e-01,   0.00000000e+00,         2.22628339e+00,   9.99200722e-15,  -3.89847865e-02,         1.51171147e+00,   1.66533454e-16,   1.66533454e-16,         1.66533454e-16,   0.00000000e+00]), array([ -1.03626102e-14,   7.85046229e-16,  -1.00000000e+00])],
                      [array([-0.53374407,  0.9960226 ,  0.        ,  1.91409838, -3.14159265,       -0.23147168, -2.10454039,  0.        ,  0.        ,  0.        ,  0.        ]), array([  6.86915451e-16,  -1.35420475e-15,  -1.00000000e+00])]]
        
        with env:
            for dofvalues, direction in testvalues:
                robot.SetDOFValues(dofvalues)
                assert( not env.CheckCollision(robot) )
                ret = basemanip.MoveHandStraight(direction=direction, ignorefirstcollision=False,stepsize=0.001,minsteps=19,maxsteps=20, execute=False)
                assert(ret is not None)
                

    def test_navigationmanip(self):
        env=self.env
        env.StartSimulation(0.1,False)
        env.Load('data/pr2test2.env.xml')
        robot = env.GetRobots()[0]

        manip = robot.SetActiveManipulator('leftarm_torso')
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        nonadjlinks = array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))
        basemanip = interfaces.BaseManipulation(robot)
        taskprob = interfaces.TaskManipulation(robot)
        target=env.GetKinBody('TibitsBox1')
        with env:
            targetcollision = env.CheckCollision(target)
            jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
            jointvalues = [1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]
            robot.SetDOFValues(jointvalues,[robot.GetJoint(name).GetDOFIndex() for name in jointnames])
            robot.SetActiveDOFs([],Robot.DOFAffine.X|Robot.DOFAffine.Y|Robot.DOFAffine.RotationAxis,[0,0,1])
            basemanip.MoveActiveJoints(goal=[2.8,-1.3,0],maxiter=5000,steplength=0.15,maxtries=2)
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
        robot.WaitForController(100)

        taskprob.ReleaseFingers()
        with env:
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
        robot.WaitForController(100)

        Tgoal = array([[0,-1,0,3.5],[-1,0,0,-1.3],[0,0,-1,0.842],[0,0,0,1]])
        res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)
        with env:
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
        robot.WaitForController(100)

        taskprob.CloseFingers()
        with env:
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
        robot.WaitForController(100)

        with env:
            robot.Grab(target)
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
            assert( not targetcollision or env.CheckCollision(robot) )
            basemanip.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
        robot.WaitForController(100)

    def test_planwithcollision(self):
        env=self.env
        env.Load('data/pr2test1.env.xml')
        robot=env.GetRobots()[0]
        with env:
            defaultvalues = robot.GetDOFValues()
            robot.SetDOFValues([0.187],[robot.GetJoint('l_shoulder_lift_joint').GetDOFIndex()])
            
            print 'case: environment collision'
            manip = robot.SetActiveManipulator('rightarm')
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()

            Tdelta = eye(4)
            Tdelta[0,3] = -0.2
            Tdelta[2,3] = -0.2
            Tnew = dot(manip.GetEndEffectorTransform(),Tdelta)

            basemanip = interfaces.BaseManipulation(robot,execute=False)
            ret = basemanip.MoveToHandPosition([Tnew])
            assert(ret is not None)

            print 'case: self collision'
#             robot.SetDOFValues(defaultvalues)
#             ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
#             if not ikmodel.load():
#                 ikmodel.autogenerate()

            #array([ 1.34046301,  0.94535038,  3.03934583, -1.30743665,  0.        , 0.        ,  0.        ])
