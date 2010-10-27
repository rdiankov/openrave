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
"""
Shows how to use different IK solutions for arms with few joints.



"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
else:
    from openravepy import OpenRAVEModel, OpenRAVEGlobalArguments
from numpy import random, array, linspace
from optparse import OptionParser

def run(args=None):
    """Executes tutorial_iktypes

    :type args: arguments for script to parse, if not specified will use sys.argv

    **Help**
    
    .. shell-block:: openrave.py --example tutorial_iktypes --help
    """

    parser = OptionParser(description='Shows how to use different IK solutions for arms with few joints.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/katanatable.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load(options.scene)
    while True:
        robot = env.GetRobots()[0]
        robot.SetActiveManipulator('arm')
        lower,upper = robot.GetDOFLimits(robot.GetActiveManipulator().GetArmIndices())

        # generate the ik solver
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Translation3D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        with env:
            # move the robot in a random collision-free position and call the IK
            ikmodel.manip.GetEndEffectorTransform()[0:3,3]
            for i in range(4):
                
                if not robot.CheckSelfCollision():
                    solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetEndEffectorTransform(),True)
                    if solutions is not None and len(solutions) > 0: # if found, then break
                        break

if __name__ == "__main__":
    run()
