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
                body = env.ReadKinBodyURI(bodynames[random.randint(len(bodynames))])
                body.SetName('body%d'%numbodies)
                numbodies += 1
                env.AddKinBody(body)
                T = eye(4)
                T[0:3,3] = array((-0.5,-0.5,2))+0.4*random.rand(3)
                body.SetTransform(T)

            time.sleep(0.4)
        with env:
            assert( transdist(robot.GetTransform(),Trobot) <= g_epsilon )

    def test_simtime(self):
        env=self.env
        physics = RaveCreatePhysicsEngine(env,'ode')
        env.SetPhysicsEngine(physics)
        env.GetPhysicsEngine().SetGravity([0,0,-9.81])

        with env:
            body = env.ReadKinBodyURI('data/lego2.kinbody.xml')
            body.SetName('body')
            env.AddKinBody(body)
            Tinit = eye(4)
            Tinit[2,3] = 3
            body.SetTransform(Tinit)
        
        env.StartSimulation(0.01,realtime=True)
        starttime = 1e-6*env.GetSimulationTime()
        realtime0 = time.time()
        time.sleep(1)
        env.StopSimulation()
        realtime1 = time.time()
        simtime0 = 1e-6*env.GetSimulationTime()
        assert( abs(simtime0-starttime-1) < 0.05 )
        with env:
            T = body.GetTransform()
            assert(abs(T[2,3]-Tinit[2,3]) > 0.2)
        time.sleep(2)
        with env:
            T2 = body.GetTransform()
            assert(abs(T[2,3]-T2[2,3]) < g_epsilon )
        env.StartSimulation(timestep=0.01,realtime=True)
        realtime2 = time.time()
        time.sleep(1)
        env.StopSimulation()
        realtime3 = time.time()
        assert( abs(1e-6*env.GetSimulationTime()-starttime-(realtime3-realtime2)-(realtime1-realtime0)) < 0.05 )
        with env:
            T2 = body.GetTransform()
            assert(abs(T[2,3]-T2[2,3])>0.2)
            body.SetVelocity([0,0,0],[0,0,0])
        
        simtime1 = 1e-6*env.GetSimulationTime()
        for i in range(int((simtime0-starttime)*100)):
            env.StepSimulation(0.01)
        with env:
            T3 = body.GetTransform()
            print (T[2,3]-Tinit[2,3]),(T3[2,3]-T2[2,3])
            assert( abs((T[2,3]-Tinit[2,3]) - (T3[2,3]-T2[2,3])) < 0.001)
        assert(abs(1e-6*env.GetSimulationTime()-simtime1-(simtime0-starttime)) < g_epsilon)

        env.StartSimulation(timestep=0.01,realtime=False)
        simtime2 = 1e-6*env.GetSimulationTime()
        while True:
            if 1e-6*env.GetSimulationTime() > simtime2+1:
                break
        env.StopSimulation()
