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

class TestPhysics(EnvironmentSetup):
    def test_static(self):
        env=self.env
        scene = 'data/hanoi.env.xml'
        env.Load(scene)
        with env:
            robot=env.GetRobots()[0]
            Trobot = robot.GetTransform()
            physics = RaveCreatePhysicsEngine(env,'ode')
            env.SetPhysicsEngine(physics)
            env.GetPhysicsEngine().SetGravity([0,0,-9.81])

            assert(not robot.GetLinks()[0].IsStatic())
            robot.GetLinks()[0].SetStatic(True)

            bodynames = ['data/lego2.kinbody.xml', 'data/lego4.kinbody.xml', 'data/mug1.kinbody.xml']
            numbodies = 0
            env.StopSimulation()
            env.StartSimulation(timestep=0.01)

        while numbodies < 5:
            with env:
                body = env.ReadKinBodyXMLFile(bodynames[random.randint(len(bodynames))])
                body.SetName('body%d'%numbodies)
                numbodies += 1
                env.AddKinBody(body)
                T = eye(4)
                T[0:3,3] = array((-0.5,-0.5,2))+0.4*random.rand(3)
                body.SetTransform(T)

            time.sleep(0.4)
        with env:
            assert( transdist(robot.GetTransform(),Trobot) <= g_epsilon )
