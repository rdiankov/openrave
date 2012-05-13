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
"""Shows how to use the grasping.GraspingModel to compute valid grasps for manipulation.

.. examplepre-block:: simplegrasping

Description
-----------

This type of example is suited for object geometries that are dynamically created from sensor data.

.. examplepost-block:: simplegrasping
"""

from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

from itertools import izip
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    if options.manipname is not None:
        robot.SetActiveManipulator(options.manipname)

    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 0.2]
    target = bodies[random.randint(len(bodies))]
    print 'choosing target %s'%target

    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    gmodel = databases.grasping.GraspingModel(robot,target)
    if not gmodel.load():
        print 'generating grasping model (one time computation)'
        gmodel.init(friction=0.4,avoidlinks=[])
        gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.04,normalanglerange=0))
        gmodel.save()

    returnnum = 5

    print 'computing first %d valid grasps'%returnnum
    validgrasps,validindices = gmodel.computeValidGrasps(returnnum=returnnum)
    for validgrasp in validgrasps:
        gmodel.showgrasp(validgrasp)

    print 'choosing a random grasp and move to its preshape'
    basemanip = openravepy.interfaces.BaseManipulation(robot)
    with env:
        initialvalues = robot.GetDOFValues(gmodel.manip.GetArmIndices())

    for validgrasp in random.permutation(validgrasps):
        try:
            gmodel.moveToPreshape(validgrasp)
            print 'move robot arm to grasp'
            Tgrasp = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
            basemanip.MoveToHandPosition(matrices=[Tgrasp])
            break
        except planning_error,e:
            print 'try again: ',e

    robot.WaitForController(10)
    taskmanip = openravepy.interfaces.TaskManipulation(robot)
    taskmanip.CloseFingers()
    robot.WaitForController(10)
    with env:
        robot.Grab(target)
    raw_input('press any key to release')
    taskmanip.ReleaseFingers(target=target)
    robot.WaitForController(10)
    print 'initial values'
    basemanip.MoveManipulator(initialvalues)
    robot.WaitForController(10)

    print 'using the grasp iterator'
    with env:
        for validgrasp,validindex in gmodel.validGraspIterator():
            gmodel.showgrasp(validgrasp,useik=True,collisionfree=True,delay=0.4)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to use the grasping.GraspingModel to compute valid grasps for manipulation.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene', action="store",type='string',dest='scene',default='data/wamtest1.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname', action="store",type='string',dest='manipname',default=None,
                      help='Choose the manipulator to perform the grasping for')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
