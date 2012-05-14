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
"""Manually call the simulation loop to update the bodies while environment is locked.

.. examplepre-block:: testupdatingbodies

.. examplepost-block:: testupdatingbodies
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def main(env,options):
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    manipprob = interfaces.BaseManipulation(robot)
    
    Tcamera = array(((0.84028,  -0.14715,   0.52179,0.930986),
                     (0.52639,   0.45182,  -0.72026,-1.233453),
                     (-0.12976,   0.87989,   0.45711,2.412977)))
    env.GetViewer().SetCamera(Tcamera)
    env.GetViewer().EnvironmentSync()
    
    print 'Stopping the environment loop from updating the simulation'
    env.StopSimulation()
    print 'Locking environment and starting to plan'
    with env:
        res = manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19])
        print 'Calling the simulation loop internally to python'
        while not robot.GetController().IsDone():
            env.StepSimulation(0.01)
            env.UpdatePublishedBodies() # used to publish body information while environment is locked
            time.sleep(0.1)    
    raw_input('press any key to exit: ')

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description="test physics")
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
