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

class RunController(EnvironmentSetup):
    def __init__(self,controllername):
        self.controllername = controllername
    def setup(self):
        EnvironmentSetup.setup(self)

    def _PreprocessRobot(self,robot):
        robot.SetController(RaveCreateController(self.env,self.controllername),range(robot.GetDOF()), 1)
        return super(RunController,self)._PreprocessRobot(robot)
        
    def test_multitraj(self):
        self.log.debug('runs two separate trajectories')
        robot1=self.LoadRobot('robots/schunk-lwa3.zae')
        robot1.SetName('_R1_')
        robot2=self.LoadRobot('robots/schunk-lwa3.zae')
        robot2.SetName('_R2_')
        env=self.env
        #from IPython.Shell import IPShellEmbed; IPShellEmbed(argv='')(local_ns=locals())
        with env:
            # make sure name is present
            assert(repr(robot1.GetConfigurationSpecification()).find(robot1.GetName()) >= 0)
            assert(repr(robot2.GetConfigurationSpecification()).find(robot2.GetName()) >= 0)
            T=eye(4)
            T[0,3] = 0.5
            robot1.SetTransform(T)
            initvalues1 = robot1.GetActiveDOFValues()
            T=eye(4)
            T[0,3] = -0.5
            robot2.SetTransform(T)
            initvalues2 = robot2.GetActiveDOFValues()
            
            waypoint=zeros(robot1.GetActiveDOF())
            waypoint[0] = 0.5
            waypoint[1] = 0.5
            traj=RaveCreateTrajectory(env, '')
            traj.Init(robot1.GetActiveConfigurationSpecification('quadratic'))
            traj.Insert(0,r_[initvalues1,waypoint])
            ret=planningutils.RetimeActiveDOFTrajectory(traj,robot1,False)
            assert(ret==PlannerStatus.HasSolution)
            assert(traj.GetDuration()>0)
            # shouldn't move
            self.RunTrajectory(robot2,traj)
            assert(transdist(robot2.GetActiveDOFValues(),initvalues2) <= g_epsilon)
            # should move
            self.RunTrajectory(robot1,traj)
            assert(transdist(robot1.GetActiveDOFValues(),waypoint) <= g_epsilon)
        
#generate_classes(RunController, globals(), [('ode','ode'),('bullet','bullet')])

class test_ideal(RunController):
    def __init__(self):
        RunController.__init__(self, 'IdealController')

# class test_bullet(RunController):
#     def __init__(self):
#         RunController.__init__(self, 'bullet')
# 
