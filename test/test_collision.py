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
_multiprocess_can_split_ = True

class TestCollision(EnvironmentSetup):
    def test_selfcollision(self):
        with self.env:
            self.env.Load('data/lab1.env.xml')
            target1 = self.env.GetKinBody('mug1')
            target2 = self.env.GetKinBody('mug2')
            target2.SetTransform(target1.GetTransform())
            assert(self.env.CheckCollision(target1))
            robot = self.env.GetRobots()[0]
            report = CollisionReport()
            assert(not robot.CheckSelfCollision(report))
            robot.Grab(target1)
            assert(not robot.CheckSelfCollision(report))
            assert(not target1.CheckSelfCollision())
            assert(self.env.CheckCollision(target1,report))
