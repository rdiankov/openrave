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
            robot.Grab(body)
            robot.SetDOFValues(array([  0.00000000e+00,  -1.43329144e+00,  -3.99190831e-15, -1.86732388e+00,   5.77239752e-01,  -3.37631690e-07, 6.67713991e-08,   0.00000000e+00,  -1.70089030e+00, -6.42544150e-01,  -1.25030589e+00,  -3.33493233e-08, -5.58212676e-08,   1.60115015e-08]))
            print body.GetTransform()
            assert(robot.CheckSelfCollision())
