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
"""Examples of laser sensors attached to a robot.

.. examplepre-block:: testlasersensor

Description
-----------

The :ref:`sensor-baselaser2d` interface has a simple implementation of ray-casting laser sensors. The following OpenRAVE XML attaches a simple 2D laser to the **wam1** link of the robot:

.. code-block:: xml

  <Robot>
    <AttachedSensor name="mylaser">
      <link>wam1</link>
      <translation>0 0.2 0.4</translation>
      <rotationaxis>0 0 1 90</rotationaxis>
      <sensor type="BaseLaser2D" args="">
        <minangle>-135</minangle>
        <maxangle>135</maxangle>
        <resolution>0.35</resolution>
        <maxrange>5</maxrange>
        <scantime>0.1</scantime>
      </sensor>
    </AttachedSensor>
  </Robot>

To OpenRAVE XML to attach a flash LIDAR sensor is:

.. code-block:: xml

  <Robot>
    <AttachedSensor name="myflashlaser">
      <link>wam2</link>
      <translation>-0.2 -0.2 0</translation>
      <rotationaxis>0 1 0 -90</rotationaxis>
      <sensor type="BaseFlashLidar3D">
        <maxrange>5</maxrange>
        <scantime>0.2</scantime>
        <KK>32 24 32 24</KK>
        <width>64</width>
        <height>48</height>
        <color>1 1 0</color>
      </sensor>
    </AttachedSensor>
  </Robot>

See `Sensor Concepts`_ for more infromation on sensors.

.. examplepost-block:: testlasersensor
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time, threading
from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from numpy import *

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    if options.robotname is not None:
        robot = env.GetRobot(options.robotname)
    else:
        robot = env.GetRobots()[0]
    ienablesensor = 0
    sensors = [sensor for sensor in robot.GetAttachedSensors() if sensor.GetSensor() is not None]
    while True:
        for i,sensor in enumerate(sensors):
            if i==ienablesensor:
                sensor.GetSensor().Configure(Sensor.ConfigureCommand.PowerOn)
                sensor.GetSensor().Configure(Sensor.ConfigureCommand.RenderDataOn)
            else:
                sensor.GetSensor().Configure(Sensor.ConfigureCommand.PowerOff)
                sensor.GetSensor().Configure(Sensor.ConfigureCommand.RenderDataOff)
        print 'showing sensor %s, try moving obstacles'%sensors[ienablesensor].GetName()
        time.sleep(5)
        ienablesensor = (ienablesensor+1)%len(sensors)

from optparse import OptionParser
from openravepy import OpenRAVEGlobalArguments, with_destroy

@with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Examples of laser sensors attached to a robot.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/testwamlaser.env.xml',
                      help='OpenRAVE scene to load')
    parser.add_option('--robotname',
                      action="store",type='string',dest='robotname',default=None,
                      help='Specific robot sensors to display (otherwise first robot found will be displayed)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__=='__main__':
    run()
