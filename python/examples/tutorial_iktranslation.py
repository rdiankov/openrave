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
"""Shows how to use translational inverse kinematics for an arm with few joints.

.. examplepre-block:: tutorial_iktranslation
  :image-width: 400

.. examplepost-block:: tutorial_iktranslation
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from numpy import *

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    robot.SetActiveManipulator(options.manipname)

    # generate the ik solver
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Translation3D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    while True:
        with env:
            while True:
                target=ikmodel.manip.GetEndEffectorTransform()[0:3,3]+(random.rand(3)-0.5)
                solutions = ikmodel.manip.FindIKSolutions(IkParameterization(target,IkParameterization.Type.Translation3D),IkFilterOptions.CheckEnvCollisions)
                if solutions is not None and len(solutions) > 0: # if found, then break
                    break
        h=env.plot3(array([target]),10.0)
        for i in random.permutation(len(solutions))[0:min(80,len(solutions))]:
            with env:
                robot.SetDOFValues(solutions[i],ikmodel.manip.GetArmIndices())
                env.UpdatePublishedBodies()
            time.sleep(0.05)
        h=None

from optparse import OptionParser
from openravepy import OpenRAVEGlobalArguments, with_destroy

@with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to use different IK solutions for arms with few joints.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/katanatable.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',action="store",type='string',dest='manipname',default='arm',
                      help='name of manipulator to use (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__ == "__main__":
    run()
