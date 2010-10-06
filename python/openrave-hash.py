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
from optparse import OptionParser

if __name__ == "__main__":
    parser = OptionParser(usage='usage: %prog [options] openrave-filename', description='Returns the hashes of openrave robots and bodies (%s)'%openravepy.__version__)
    parser.add_option('--manipname', action="store",type='string',dest='manipname',default=None,
                      help='if manipulator name is specified will return the manipulator hash of the robot')
    parser.add_option('--sensorname', action="store",type='string',dest='sensorname',default=None,
                      help='if manipulator name is specified will return the manipulator hash of the robot')
    parser.add_option('--robothash',action='store_true',dest='robothash',default=False,
                      help='if set, will output the robot hash of the loaded body')
    parser.add_option('--kinematics',action='store_true',dest='kinematics',default=False,
                      help='If set, will only return kinematics information')
    (options, args) = parser.parse_args()
    body = None
    RaveInitialize(False, DebugLevel.Fatal)
    env=Environment()
    # ideally, the hashes should *not* be overloaded by other users, so we should expect the same values regardless of what plugins are loaded
    try:
        env.Load(args[0])
        if options.manipname:
            if options.kinematics:
                print env.GetRobots()[0].GetManipulator(options.manipname).GetKinematicsStructureHash()
            else:
                print env.GetRobots()[0].GetManipulator(options.manipname).GetStructureHash()
        elif options.sensorname:
            print env.GetRobots()[0].GetSensor(options.sensorname).GetStructureHash()
        elif options.robothash:
            assert(not options.kinematics)
            print env.GetRobots()[0].GetRobotStructureHash()
        else:
            assert(not options.kinematics)
            print env.GetBodies()[0].GetKinematicsGeometryHash()
    finally:
        env.Destroy()
