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
"""Computes the visibilty extents of a camera and an object.

.. examplepre-block:: checkvisibility

Description
-----------

Uses the :mod:`.visibiltymodel` generator and :ref:`module-visualfeedback` interface.

.. examplepost-block:: checkvisibility

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import time, threading
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    # initialiation
    with env:
        robot = env.GetRobots()[0]
        # move a cup in front of another to show power of visibility
        body = env.GetKinBody('mug6')
        if body is not None:
            T = body.GetTransform()
            T[0:3,3] = [-0.14,0.146,0.938]
            body.SetTransform(T)

        vmodels = []
        print('creating visibility structures, please wait...')
        sensors = []
        for sensor in robot.GetAttachedSensors():
            # if sensor is a camera
            if sensor.GetSensor() is not None and sensor.GetSensor().Supports(Sensor.Type.Camera):
                sensor.GetSensor().Configure(Sensor.ConfigureCommand.PowerOn)
                sensor.GetSensor().Configure(Sensor.ConfigureCommand.RenderDataOn)
                # go through all objects
                for target in env.GetBodies():
                    # load the visibility model
                    vmodel = databases.visibilitymodel.VisibilityModel(robot,target=target,sensorname=sensor.GetName())
                    if not vmodel.load():
                        vmodel.autogenerate()
                    # set internal discretization parameters 
                    vmodel.visualprob.SetParameter(raydensity=0.002,allowableocclusion=0.0)
                    vmodels.append(vmodel)
                sensors.append(sensor.GetSensor())

    print('try moving objects in and out of the sensor view. green is inside')
    while True:
        print('-----')
        with env:
            handles = []
            for vmodel in vmodels:
                if vmodel.visualprob.ComputeVisibility():
                    # draw points around the object
                    ab = vmodel.target.ComputeAABB()
                    corners = array([[1,1,1],[1,1,-1],[1,-1,1],[1,-1,-1],[-1,1,1],[-1,1,-1],[-1,-1,1],[-1,-1,-1]],float64)
                    handles.append(env.plot3(tile(ab.pos(),(8,1))+corners*tile(ab.extents(),(8,1)),10,[0,1,0]))
                    print('%s is visible in sensor %s'%(vmodel.target.GetName(),vmodel.sensorname))
            oldhandles = handles # replace
        time.sleep(0.2)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Computes if an object is visibile inside the robot cameras.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/testwamcamera.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
