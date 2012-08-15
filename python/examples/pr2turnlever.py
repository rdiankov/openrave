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
"""Shows how to set a workspace trajectory for the hand and have a robot plan it.

.. examplepre-block:: pr2turnlever

Description
-----------

Shows how to instantiate a planner in python and pass in a workspace trajectory.

.. examplepost-block:: pr2turnlever

"""
from __future__ import with_statement # for python 2.5
__author__= 'Rosen Diankov'

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
    env.Load(options.scene)
    target = env.GetKinBody('lever')
    if target is None:
        target = RaveCreateKinBody(env,'')
        target.InitFromBoxes(array([[0,0.1,0,0.01,0.1,0.01]]),True)
        target.SetName('lever')
        env.Add(target)
        T = eye(4)
        T[0:3,3] = [-0.2,-0.2,1]
        target.SetTransform(T)
        
    robot = env.GetRobots()[0]

    robot.SetActiveManipulator('rightarm')
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    with env:
        robot.SetActiveDOFs(ikmodel.manip.GetArmIndices())
        basemanip=interfaces.BaseManipulation(robot)
        taskmanip=interfaces.TaskManipulation(robot)
        taskmanip.ReleaseFingers()
    robot.WaitForController(0)

    with env:
        Toffset = eye(4)
        Toffset[0:3,3] = array([0,0.2,0])
        Ttarget0 = target.GetTransform()
        Ttarget1 = dot(Ttarget0,matrixFromAxisAngle([pi/2,0,0]))

        # with target.CreateKinBodyStateSaver():
        #     target.SetTransform(Ttarget1)
        #     raw_input('as')
        Tgrasp0 = dot(matrixFromAxisAngle([pi/2,0,0]),matrixFromAxisAngle([0,pi/2,0]))
        Tgrasp0[0:3,3] = dot(Ttarget0,Toffset)[0:3,3]
        Tgraspoffset = dot(linalg.inv(Ttarget0),Tgrasp0)
        Tgrasp1 = dot(Ttarget1,Tgraspoffset)

        # check if ik solutions exist
        sol0=ikmodel.manip.FindIKSolution(Tgrasp0,IkFilterOptions.CheckEnvCollisions)
        assert(sol0 is not None)
        sol1=ikmodel.manip.FindIKSolution(Tgrasp1,IkFilterOptions.CheckEnvCollisions)
        assert(sol1 is not None)
        traj = RaveCreateTrajectory(env,'')
        spec = IkParameterization.GetConfigurationSpecificationFromType(IkParameterizationType.Transform6D,'linear')
        traj.Init(spec)
        for angle in arange(0,pi/2,0.05):
            Ttarget = dot(Ttarget0,matrixFromAxisAngle([angle,0,0]))
            Tgrasp = dot(Ttarget,Tgraspoffset)
            traj.Insert(traj.GetNumWaypoints(),poseFromMatrix(Tgrasp))

        planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(7),maxaccelerations=5*ones(7))

    h=misc.DrawAxes(env,Ttarget0)
    basemanip.MoveToHandPosition(matrices=[Tgrasp0])
    robot.WaitForController(0)
    taskmanip.CloseFingers()
    robot.WaitForController(0)

    with env:
        robot.Grab(target)
        print 'planning for turning lever'
        planner = RaveCreatePlanner(env,'workspacetrajectorytracker')
        params = Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetExtraParameters('<workspacetrajectory>%s</workspacetrajectory>'%traj.serialize(0))
        planner.InitPlan(robot,params)
        
        outputtraj = RaveCreateTrajectory(env,'')
        success=planner.PlanPath(outputtraj)
        assert(success)

    # also create reverse the trajectory and run infinitely
    trajectories = [outputtraj,planningutils.ReverseTrajectory(outputtraj)]
    while True:
        for traj in trajectories:
            robot.GetController().SetPath(traj)
            robot.WaitForController(0)
        if options.testmode:
            break
    
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to set a workspace trajectory for the hand and have a robot plan it.', usage='openrave.py --example pr2turnlever [options]')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/pr2test1.env.xml',
                      help='scene to load')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)


if __name__ == "__main__":
    run()
