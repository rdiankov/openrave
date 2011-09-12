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
"""Finds the first feasible grasp for an object as fast as possible without generating a grasp database.

.. examplepre-block:: fastgrasping

Description
-----------

This type of example is suited for object geometries that are dynamically created from sensor data.

.. examplepost-block:: fastgrasping
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

from itertools import izip
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class FastGrasping:
    class GraspingException(Exception):
        def __init__(self,args):
            self.args=args

    def __init__(self,robot,target):
        self.robot = robot
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.gmodel = databases.grasping.GraspingModel(robot,target)
        self.gmodel.init(friction=0.4,avoidlinks=[])

    def checkgraspfn(self, contacts,finalconfig,grasp,info):
        # check if grasp can be reached by robot
        Tglobalgrasp = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
        # have to set the preshape since the current robot is at the final grasp!
        self.gmodel.setPreshape(grasp)
        sol = self.gmodel.manip.FindIKSolution(Tglobalgrasp,True)
        if sol is not None:
            jointvalues = array(finalconfig[0])
            jointvalues[self.gmodel.manip.GetArmIndices()] = sol
            raise self.GraspingException([grasp,jointvalues])
        return True

    def computeGrasp(self):
        approachrays = self.gmodel.computeBoxApproachRays(delta=0.02,normalanglerange=0.5) # rays to approach object
        standoffs = [0]
        # roll discretization
        rolls = arange(0,2*pi,0.5*pi)
        # initial preshape for robot is the released fingers
        with self.gmodel.target:
            self.gmodel.target.Enable(False)
            taskmanip = interfaces.TaskManipulation(self.robot)
            final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
            preshapes = array([final])
        try:
            self.gmodel.disableallbodies=False
            self.gmodel.generate(preshapes=preshapes,standoffs=standoffs,rolls=rolls,approachrays=approachrays,checkgraspfn=self.checkgraspfn,graspingnoise=0.01)
            return None,None # did not find anything
        except self.GraspingException, e:
            return e.args

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    if options.manipname is not None:
        robot.SetActiveManipulator(options.manipname)
    # find an appropriate target
    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 0.2]
    for body in bodies:
        self = FastGrasping(robot,target=body)
        grasp,jointvalues = self.computeGrasp()
        if grasp is not None:
            print 'grasp is found!'
            self.gmodel.showgrasp(grasp)
            self.robot.SetDOFValues(jointvalues)
            raw_input('press any key')

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Example showing how to compute a valid grasp as fast as possible without computing a grasp set, this is used when the target objects change frequently.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene', action="store",type='string',dest='scene',default='data/wamtest1.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname', action="store",type='string',dest='manipname',default=None,
                      help='Choose the manipulator to perform the grasping for')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__ == "__main__":
    run()
