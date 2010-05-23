#!/usr/bin/env python
# Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
import time

def run():
    env = Environment()
    env.Load('data/hanoi.env.xml')
    env.SetViewer('qtcoin')
    physics = env.CreatePhysicsEngine('ode')
    env.SetPhysicsEngine(physics)
    physics.SetGravity(array((0,-9.8,0)))
    bodynames = ['data/lego2.kinbody.xml', 'data/lego4.kinbody.xml', 'data/mug1.kinbody.xml']
    numbodies = 0
    env.StopSimulation()
    env.StartSimulation(timestep=0.01)
    starttime = time.time()
    while True:
        if numbodies < 40:
            with env:
                body = env.ReadKinBodyXMLFile(bodynames[random.randint(len(bodynames))])
                body.SetName('body%d'%numbodies)
                numbodies += 1
                env.AddKinBody(body)
                T = eye(4)
                T[0:3,3] = array((-0.5,2,-0.5))+0.4*random.rand(3)
                body.SetTransform(T)
                #env.GetRobots()[0].GetLinks()[6].GetGeometries()[0].SetCollisionMesh(KinBody.Link.TriMesh(*ComputeBoxMesh([1,0.2,0.3])))
                        
        time.sleep(0.4)
        simtime = env.GetSimulationTime()*1e-6
        realtime = time.time()-starttime
        print 'sim time: %fs, real time: %fs, diff = %fs'%(simtime,realtime,simtime-realtime)

if __name__=='__main__':
    run()
