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
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from optparse import OptionParser
import time

class CheckVisibility(metaclass.AutoReloader):
    def __init__(self,robot):
        self.robot = robot
        self.vmodels = []
        # go through all objects
        for target in self.robot.GetEnv().GetBodies():
            # through all sensors
            for sensor in self.robot.GetAttachedSensors():
                # if sensor is a camera
                if sensor.GetSensor() is not None and sensor.GetData().type == Sensor.Type.Camera:
                    # load the visibility model
                    vmodel = databases.visibilitymodel.VisibilityModel(robot,target=target,sensorname=sensor.GetName())
                    if not vmodel.load():
                        vmodel.autogenerate()
                    # set internal discretization parameters 
                    vmodel.visualprob.SetParameter(raydensity=0.002,allowableocclusion=0.0)
                    self.vmodels.append(vmodel)

    def computeVisibleObjects(self):
        print '-----'
        for vmodel in self.vmodels:
            if vmodel.visualprob.ComputeVisibility():
                print '%s is visible in sensor %s'%(vmodel.target.GetName(),vmodel.sensorname)

    def viewSensors(self):
        pilutil=__import__('scipy.misc',fromlist=['pilutil'])
        shownsensors = []
        for vmodel in self.vmodels:
            if not vmodel.sensorname in shownsensors:
                print vmodel.sensorname
                pilutil.imshow(vmodel.getCameraImage())
                shownsensors.append(vmodel.sensorname)
def run(args=None):
    """Executes the checkvisibility example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Computes if an object is visibile inside the robot cameras.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/testwamcamera.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    try:
        env.Load(options.scene)
        robot = env.GetRobots()[0]
        self = CheckVisibility(robot)
        while True:
            self.computeVisibleObjects()
            time.sleep(0.01)
    finally:
        env.Destroy()

if __name__ == "__main__":
    run()
