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
.. image:: ../../images/example_tutorials/ik5d.jpg
  :height: 200

Shows how to use 5D translation+direction inverse kinematics for an arm with >= 5 joints.
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
from numpy import random, array, linspace, linalg
from optparse import OptionParser
import time

def run(args=None):
    """Executes tutorial_ik5d

    :type args: arguments for script to parse, if not specified will use sys.argv

    **Help**
    
    .. shell-block:: openrave.py --example tutorial_ik5d --help
    """

    parser = OptionParser(description='Shows how to use the 5DOF IK solution for arms with >= 5 joints.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/katanatable.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',action="store",type='string',dest='manipname',default='arm',
                      help='name of manipulator to use (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    robot.SetActiveManipulator(options.manipname)

    # generate the ik solver
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.TranslationDirection5D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    while True:
        with env:
            while True:
                target=ikmodel.manip.GetEndEffectorTransform()[0:3,3]+(random.rand(3)-0.5)
                direction = random.rand(3)-0.5
                direction /= linalg.norm(direction)
                solutions = ikmodel.manip.FindIKSolutions(IkParameterization(Ray(target,direction),IkParameterization.Type.TranslationDirection5D),IkFilterOptions.CheckEnvCollisions)
                if solutions is not None and len(solutions) > 0: # if found, then break
                    break
        h=env.drawlinestrip(array([target,target+0.1*direction]),10)
        for i in random.permutation(len(solutions))[0:min(80,len(solutions))]:
            with env:
                robot.SetDOFValues(solutions[i],ikmodel.manip.GetArmIndices())
                env.UpdatePublishedBodies()
            time.sleep(0.2)
        h=None

if __name__ == "__main__":
    run()
