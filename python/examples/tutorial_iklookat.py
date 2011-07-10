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
"""Shows how to use lookat inverse kinematics to maintain line of sight with a moving object.

.. examplepre-block:: tutorial_iklookat

.. examplepost-block:: tutorial_iklookat
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    robot.SetActiveManipulator(options.manipname)

    # generate the ik solver
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Lookat3D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    while True:
        with env:
            # move the robot in a random collision-free position and call the IK
            while True:
                target=ikmodel.manip.GetEndEffectorTransform()[0:3,3]+(random.rand(3)-0.5)
                solutions = ikmodel.manip.FindIKSolutions(IkParameterization(target,IkParameterization.Type.Lookat3D),IkFilterOptions.CheckEnvCollisions)
                if len(solutions) > 0:
                    break
        h=env.plot3(array([target]),20.0)
        for i in random.permutation(len(solutions))[0:min(100,len(solutions))]:
            with env:
                robot.SetDOFValues(solutions[i],ikmodel.manip.GetArmIndices())
                T = ikmodel.manip.GetEndEffectorTransform()
                globaldir = numpy.dot(T[0:3,0:3],ikmodel.manip.GetDirection())
                dist = linalg.norm(T[0:3,3]-target)+0.4
                hray = env.drawlinelist(array([T[0:3,3], T[0:3,3]+dist*globaldir]),5,colors=[0.1,0.1,1])
                env.UpdatePublishedBodies()
            time.sleep(0.1)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to use different IK solutions for arms with few joints.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/pr2test1.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',action="store",type='string',dest='manipname',default='head_torso',
                      help='name of manipulator to use (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__ == "__main__":
    run()
