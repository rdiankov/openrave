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

class TestMoving(EnvironmentSetup):
    def test_constraint(self):
        env = self.env
        robot = env.ReadRobotXMLFile('robots/pr2-beta-static.zae')
        env.AddRobot(robot)
        with env:
            manip=robot.SetActiveManipulator('leftarm_torso')
            basemanip = interfaces.BaseManipulation(robot,validatetrajectory=True)
            T=array([[0,0,1,.6], [0,1,0,.1], [-1,0,0,.73], [0,0,0,1]]);
            robot.SetDOFValues([.31],[robot.GetJoint('torso_lift_joint').GetDOFIndex()])
            robot.SetDOFValues(manip.FindIKSolution(T,IkFilterOptions.CheckEnvCollisions),manip.GetArmIndices())
            Tgoal=array([[0,0,1,.6], [0,1,0,.2], [-1,0,0,.73], [0,0,0,1]])
            constraintfreedoms=array([1,1,0,0,0,1]) # can rotate along z, translate along y
            constraintmatrix=array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
            for constrainterrorthresh in [.005,0.01]:
                success = basemanip.MoveToHandPosition(matrices=[Tgoal],maxiter=4000,maxtries=3,seedik=16, constraintfreedoms=constraintfreedoms, constraintmatrix=constraintmatrix, constrainterrorthresh=constrainterrorthresh,execute=False)
                assert(success)

    def test_movehandstraight(self):
        env = self.env
        env.Load('data/lab1.env.xml')
        robot = env.GetRobots()[0]
        basemanip = interfaces.BaseManipulation(robot,validatetrajectory=True)

        testvalues = [[array([ -2.83686683e-01,   1.40828054e+00,   0.00000000e+00, 5.26754682e-01,  -3.14159265e+00,  -1.20655743e+00, -1.85448301e+00,   1.66533454e-16,   1.66533454e-16,         1.66533454e-16,   0.00000000e+00]), array([ -9.22429319e-16,  -3.90560499e-15,  -1.00000000e+00])],
                      [array([-0.26085414,  1.37967815,  0.        ,  0.60871186, -3.14159265,       -1.15320264, -0.26085414,  0.        ,  0.        ,  0.        ,  0.        ]), array([ -7.21644966e-16,  -3.28903571e-15,  -1.00000000e+00])],
                      [array([ -5.90848599e-02,   9.54294051e-01,   0.00000000e+00,         2.22628339e+00,   9.99200722e-15,  -3.89847865e-02,         1.51171147e+00,   1.66533454e-16,   1.66533454e-16,         1.66533454e-16,   0.00000000e+00]), array([ -1.03626102e-14,   7.85046229e-16,  -1.00000000e+00])],
                      [array([-0.53374407,  0.9960226 ,  0.        ,  1.91409838, -3.14159265,       -0.23147168, -2.10454039,  0.        ,  0.        ,  0.        ,  0.        ]), array([  6.86915451e-16,  -1.35420475e-15,  -1.00000000e+00])]]
        
        with env:
            for dofvalues, direction in testvalues:
                robot.SetDOFValues(dofvalues)
                assert( not env.CheckCollision(robot) )
                trajdata = basemanip.MoveHandStraight(direction=direction, ignorefirstcollision=False,stepsize=0.001,minsteps=19,maxsteps=20, execute=False, outputtraj=True)
                assert( trajdata is not None )
                assert( basemanip.ValidateTrajectory(data=trajdata) is not None )
                
