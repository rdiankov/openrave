#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
"""Shows velocity control with ode controller.

.. examplepre-block:: testphysics_controller

.. examplepost-block:: testphysics_controller
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    if options._physics is None:
        # no physics engine set, so set one
        physics = RaveCreatePhysicsEngine(env,'ode')
        env.SetPhysicsEngine(physics)
    physics.SetGravity(array((0,0,-9.8)))

    with env:
        robot = env.GetRobots()[0]
        robot.GetLinks()[0].SetStatic(True)
        robot.SetController(RaveCreateController(env,'odevelocity'),range(robot.GetDOF()),0)
        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    starttime = time.time()
    while True:
        velocities = 0.5*(random.rand(robot.GetDOF())-0.5)
        robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
        time.sleep(2.0)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description="test physics")
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/hanoi.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__=='__main__':
    run()
