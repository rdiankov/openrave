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
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.databases import convexdecomposition,grasping,inversekinematics
from numpy import *
import numpy,time,traceback
from optparse import OptionParser
from itertools import izip

class FastGrasping(metaclass.AutoReloader):
    """Computes a valid grasp for a given object as fast as possible without relying on a pre-computed grasp set
    """
    class GraspingException(Exception):
        pass

    def __init__(self,robot,target):
        self.robot = robot
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.gmodel = databases.grasping.GraspingModel(robot,target)

    def computeGrasp(self):
        self.gmodel.autogenerate()

def run(args=None):
    """Executes the fastgrasping example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Example showing how to compute a valid grasp as fast as possible without computing a grasp set, this is used when the target objects change frequently.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    try:
        env.Load(options.scene)
        robot = env.GetRobots()[0]
        self = GraspPlanning(robot,randomize=options.randomize,nodestinations=options.nodestinations)
        self.performGraspPlanning()
    finally:
        env.Destroy()

# if __name__ == "__main__":
#     run()
