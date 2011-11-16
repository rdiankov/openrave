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

class TestTrajectory(EnvironmentSetup):
    def test_merging(self):
        env = self.env
        env.Load('robots/pr2-beta-static.zae')
        robot=env.GetRobots()[0]
        basemanip=interfaces.BaseManipulation(robot)
        manip1=robot.SetActiveManipulator('leftarm')
        Tgoal1 = manip1.GetTransform()
        Tgoal1[0,3] -= 0.3
        Tgoal1[2,3] += 0.4
        trajdata=basemanip.MoveToHandPosition(matrices=[Tgoal1],execute=False,outputtraj=True)
        traj1=RaveCreateTrajectory(env,'').deserialize(trajdata)

        manip2=robot.SetActiveManipulator('rightarm')
        Tgoal2 = manip2.GetTransform()
        Tgoal2[0,3] -= 0.5
        Tgoal2[1,3] -= 0.5
        Tgoal2[2,3] += 0.2
        trajdata=basemanip.MoveToHandPosition(matrices=[Tgoal2],execute=False,outputtraj=True)
        traj2=RaveCreateTrajectory(env,'').deserialize(trajdata)

        traj3=planningutils.MergeTrajectories([traj1,traj2])

        with robot:
            dofvalues=traj3.GetConfigurationSpecification().ExtractJointValues(traj3.GetWaypoint(-1),robot,range(robot.GetDOF()),0)
            robot.SetDOFValues(dofvalues)
            assert( transdist(manip1.GetTransform(),Tgoal1) <= g_epsilon)
            assert( transdist(manip2.GetTransform(),Tgoal2) <= g_epsilon)
            assert( abs(traj3.GetDuration() - max(traj1.GetDuration(),traj2.GetDuration())) <= g_epsilon )
