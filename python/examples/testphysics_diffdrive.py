#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
"""Shows how to enable differential drive with physics

.. image:: ../../images/examples/testphysics_diffdrive.jpg
  :width: 400

**Running the Example**::

  openrave.py --example testphysics_diffdrive

Command-line
------------

.. shell-block:: openrave.py --example testphysics_diffdrive --help

Class Definitions
-----------------
"""

from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
import time
from optparse import OptionParser

@with_destroy
def run(args=None):
    """Executes the testphysics_diffdrive example.
    
    :type args: arguments for script to parse, if not specified will use sys.argv
    
    **Help**
    
    .. shell-block:: openrave.py --example testphysics_diffdrive --help
    """
    parser = OptionParser(description="test physics diff drive controller")
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/diffdrive_sample.env.xml',
                      help='Scene file to load (default=%default)')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load(options.scene)
    if options._physics is None:
        # no physics engine set, so set one
        physics = RaveCreatePhysicsEngine(env,'ode')
        env.SetPhysicsEngine(physics)
        physics.SetGravity(array((0,0,-9.8)))

    with env:
        robot = env.GetRobots()[0]
        robot.SetController(RaveCreateController(env,'odevelocity'),range(robot.GetDOF()),0)
        env.StopSimulation()
        env.StartSimulation(timestep=0.001)

    starttime = time.time()
    while True:
        velocities = 2*(random.rand(robot.GetDOF())-0.5)
        print 'velocities: ',velocities
        robot.GetController().SendCommand('setvelocity '+' '.join(str(f) for f in velocities))
        time.sleep(2.0)

if __name__=='__main__':
    run()
