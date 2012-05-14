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
"""Opens a GUI window showing the sensor data of a scene.

.. examplepre-block:: showsensors
  :image-width: 200

Description
-----------

See `Sensor Concepts`_ for detailed infromation on sensors. When requesting data from the sensor right after it is powered on, the sensor might not return valid data. Therefore have to make sure the data is good by checking that the time stamps changed.

Camera
~~~~~~


The :ref:`sensor-basecamera` interface has a simple implementation of a pinhole camera. This example shows a robot
with a camera attached to its wrist. The example opens ``data/testwamcamera.env.xml`` and
queries image data from the sensor as fast as possible. The image will change in real-time as the
robot is moved around the scene. The wireframe frustum rendered next to the robot shows the camera's
field of view.

.. image:: ../../images/examples/showsensors_camera.jpg
  :width: 640

The OpenRAVE XML required to attach a camera to the robot similar to the example above is:

.. code-block:: xml

  <Robot>
    <AttachedSensor>
      <link>wam4</link>
      <translation>0 -0.2 0</translation>
      <rotationaxis>0 1 0 -90</rotationaxis>
      <sensor type="BaseCamera" args="">
        <KK>640 480 320 240</KK>
        <width>640</width>
        <height>480</height>
        <framerate>5</framerate>
        <color>0.5 0.5 1</color>
      </sensor>
    </AttachedSensor>
  </Robot>

Lasers
~~~~~~

The :ref:`sensor-baselaser2d` interface has a simple implementation of ray-casting laser sensors. 

.. image:: ../../images/examples/showsensors_laser.jpg
  :width: 640

The following OpenRAVE XML attaches a simple 2D laser to the **wam1** link of the robot:

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

.. examplepost-block:: showsensors
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time, threading
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    ienablesensor = 0
    while True:
        sensors = env.GetSensors()
        for i,sensor in enumerate(sensors):
            if i==ienablesensor:
                sensor.Configure(Sensor.ConfigureCommand.PowerOn)
                sensor.Configure(Sensor.ConfigureCommand.RenderDataOn)
            else:
                sensor.Configure(Sensor.ConfigureCommand.PowerOff)
                sensor.Configure(Sensor.ConfigureCommand.RenderDataOff)
        print 'showing sensor %s, try moving obstacles'%(sensors[ienablesensor].GetName())
        if sensors[ienablesensor].Supports(Sensor.Type.Laser):
            # if laser, wait for the sensor data to be updated and then print it
            olddata = sensors[ienablesensor].GetSensorData(Sensor.Type.Laser)
            while True:
                data = sensors[ienablesensor].GetSensorData(Sensor.Type.Laser)
                if data.stamp != olddata.stamp:
                    break
                time.sleep(0.1)
            print 'sensor data: ',data.ranges                        
        time.sleep(5)
        ienablesensor = (ienablesensor+1)%len(sensors)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Displays all images of all camera sensors attached to a robot.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/testwamcamera.env.xml',
                      help='OpenRAVE scene to load')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__=='__main__':
    run()
