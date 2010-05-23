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

    with env:
        robot = env.GetRobots()[0]
        robot.SetController(env.CreateController('odevelocity'))
        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    starttime = time.time()
    while True:
        velocities = 0.5*(random.rand(robot.GetDOF())-0.5)
        robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
        time.sleep(2.0)

if __name__=='__main__':
    run()
