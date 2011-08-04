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
"""Tests moving the end effctor of the manipulator in straight paths using just the translation.

.. examplepre-block:: movehandstraight2

Description
-----------

Shows how to use the MoveHandStraight basemanipulation command. The example picks a random trajectory of the end effector and tests if this trajectory is feasible to achieve in the robot.

.. examplepost-block:: movehandstraight2
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
    if options.manipname is not None:
        robot.SetActiveManipulator(options.manipname)
    with env:
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        basemanip = interfaces.BaseManipulation(robot)
        taskmanip = interfaces.TaskManipulation(robot)

    armlength = 0
    armjoints = [j for j in robot.GetDependencyOrderedJoints() if j.GetJointIndex() in ikmodel.manip.GetArmIndices()]
    eetrans = ikmodel.manip.GetEndEffectorTransform()[0:3,3]
    for j in armjoints[::-1]:
        armlength += sqrt(sum((eetrans-j.GetAnchor())**2))
        eetrans = j.GetAnchor()
    stepsize=0.01
    failedattempt = 0
    while True:
        with env:
            #Tee = dot(ikmodel.manip.GetEndEffectorTransform(),matrixFromAxisAngle(random.rand(3)-0.5,0.2*random.rand()))
            Tee = matrixFromAxisAngle(random.rand(3)-0.5,pi*random.rand())
            direction = random.rand(3)
            direction /= linalg.norm(direction)
            x = random.rand(3)-0.5
            length = 0.7*random.rand()*armlength
            Tee[0:3,3] = eetrans + x/linalg.norm(x)*(armlength-length)
            maxsteps=int(length/stepsize)
            minsteps = maxsteps/2
            h = env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),1)
        try:
            # setting ignorefirstcollision to 0.01 of initial collision before moving straight, this is very useful when lifting objects in collision with a table
            success = basemanip.MoveHandStraight(direction=direction,starteematrix=Tee,stepsize=stepsize,minsteps=minsteps,maxsteps=maxsteps,ignorefirstcollision=0.0)
            params = (direction,Tee)
            print '%d failed attemps before found'%failedattempt
            failedattempt = 0
            h = env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),4,[0,0,0.5])
            while not robot.GetController().IsDone():
                with env:
                    hend = env.plot3(ikmodel.manip.GetEndEffectorTransform()[0:3,3],10,[0.2,0.2,1])
                time.sleep(0.01)
            #robot.WaitForController(0)
        except planning_error,e:
            failedattempt += 1

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy 
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how choose IK solutions so that move hand straight can move without discontinuities.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/katanatable.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',
                      action="store",type='string',dest='manipname',default=None,
                      help='Choose the manipulator to perform movement for')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__ == "__main__":
    run()
