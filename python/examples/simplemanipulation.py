#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 Alan Tan
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
"""Explicitly specify goals to get a simple navigation and manipulation demo.

.. examplepre-block:: simplemanipulation

Description
-----------

This example shows how to string in a navigation and manipulation planner to achieve a simple goto -> grab -> move task.

.. examplepost-block:: simplemanipulation

"""
from __future__ import with_statement # for python 2.5
__author__= 'Alan Tan'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def main(env,options):
    "Main example code."
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    manip = robot.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm + torso
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    # create the interface for basic manipulation programs
    basemanip = interfaces.BaseManipulation(robot,plannername=options.planner)
    taskprob = interfaces.TaskManipulation(robot,plannername=options.planner)

    target=env.GetKinBody('TibitsBox1')
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        basemanip.MoveActiveJoints(goal=[1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])
    waitrobot(robot)

    print('move robot base to target')
    with env:
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        basemanip.MoveActiveJoints(goal=[2.8,-1.3,0],maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot)

    taskprob.ReleaseFingers()
    waitrobot(robot)

    print('move the arm to the target')
    Tgoal = array([[0,-1,0,3.5],[-1,0,0,-1.3],[0,0,-1,0.842],[0,0,0,1]])
    res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)
    waitrobot(robot)

    print('close fingers until collision')
    taskprob.CloseFingers()
    waitrobot(robot)

    print('move the arm with the target back to the initial position')
    with env:
        robot.Grab(target)
        basemanip.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
    waitrobot(robot)

    print('move the robot to another location')
    with env:
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        localgoal = [0,2.4,0]
        T = robot.GetTransform()
        goal = dot(T[0:3,0:3],localgoal) + T[0:3,3]
        with robot:
            robot.SetActiveDOFValues(goal)
            incollision = env.CheckCollision(robot)
            if incollision:
                print('goal in collision!!')

    basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot)

    print('move the arm to the designated position on another table to place the target down')
    Tgoal = array([[0,-1,0,3.5],[-1,0,0,1.5],[0,0,-1,0.855],[0,0,0,1]])
    res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)
    waitrobot(robot)

    taskprob.ReleaseFingers(target=target)
    waitrobot(robot)

    print('move manipulator to initial position')
    basemanip.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
    waitrobot(robot)

    print('close fingers until collision')
    taskprob.CloseFingers()
    waitrobot(robot)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Explicitly specify goals to get a simple navigation and manipulation demo.', usage='openrave.py --example simplemanipulation [options]')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)


if __name__ == "__main__":
    run()
