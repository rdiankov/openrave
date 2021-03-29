#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Achint Aggarwal
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
#
#Created: 5 January 2010 
#Modified: 20 March 2010
"""Dual arm manipulation example.

.. examplepre-block:: dualarmdemo_schunk

.. examplepost-block:: dualarmdemo_schunk
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Achint Aggarwal'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from openravepy.misc import MultiManipIKSolver

class Schunkplanner:
    probsmanip = None
    def __init__(self,env):
        self.env = env
        self.robot = self.env.GetRobots()[0]
        self.probsmanip = RaveCreateModule(self.env,'dualmanipulation')
        args = self.robot.GetName()
        #args += ' planner birrt' 
        self.env.Add(self.probsmanip,True,args)
        self.leftArm=self.robot.GetManipulator('leftarm')
        self.rightArm=self.robot.GetManipulator('rightarm')
        self.dualsolver = MultiManipIKSolver([self.leftArm,self.rightArm])
        for manip in [self.leftArm,self.rightArm]:
            self.robot.SetActiveManipulator(manip)
            ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
               ikmodel.autogenerate()
        self.robot.SetActiveManipulator(self.leftArm)

    def WaitForController(self):
        while not self.robot.GetController().IsDone():
            time.sleep(0.001)

    def Serialize(self, T):
        return 'goal %s'%(' '.join(str(f) for f in T))

    def MoveArmsToJointPosition(self, T):
        """Moves the two arms to the given joint position T"""
        success = self.probsmanip.SendCommand('movealljoints '+self.Serialize(T))
        return success is not None
    def MoveObjectToPosition(self, T):
        """Constrained movement of the arms to new position T while holding the object"""
        success = self.probsmanip.SendCommand('movealljoints '+self.Serialize(T)+' constrainterrorthresh 1')
        return success is not None

    def planDualPath(self,obj):
        """this plans the trajectory for both the manipulators"""
        Tbody=obj.GetTransform()
        ab = obj.ComputeAABB().extents()
        halfwidth= ab[1] #this is y
        TRightGrasp= dot(Tbody,array([[0, 0, -1, 0],[1, 0, 0, (halfwidth+.1)],[0, -1, 0, 0],[0, 0, 0, 1]]))
        TLeftGrasp= dot(Tbody,array([[0, 0, -1, 0],[-1, 0, 0,-(halfwidth+.1)],[0, 1, 0, 0],[0, 0, 0, 1]])) #to determine the grasp for the eef given the transform of the object

        solutions = self.dualsolver.findMultiIKSolution(Tgrasps=[TLeftGrasp,TRightGrasp],filteroptions=IkFilterOptions.CheckEnvCollisions)
        if solutions is None or len(solutions) == 0:
            raise planning_error('failed to find solution')
        if not self.MoveArmsToJointPosition(r_[solutions[0],solutions[1]]):
            print('failed to move to position next to object')

    def moveObject(self,obj,delta):
        """this plans the trajectory for both the manipulators manipulating the object 'name' """
        Tbody=obj.GetTransform()
        ab = obj.ComputeAABB().extents()
        halfwidth= ab[1] #this is y
        Tbody[0:3,3]+=delta
        
        TRightGrasp= dot(Tbody,array([[0, 0, -1, 0],[1, 0, 0, (halfwidth+.04)],[0, -1, 0, 0 ],[0, 0, 0, 1]])) #.04 is just half the thickness of the EEF
        TLeftGrasp= dot(Tbody,array([[0, 0, -1, 0],[-1, 0, 0, -(halfwidth+.04)],[0, 1, 0, 0],[0, 0, 0, 1]])) #to determine the grasp for the eef given the transform of the object
        solutions = self.dualsolver.findMultiIKSolution(Tgrasps=[TLeftGrasp,TRightGrasp],filteroptions=IkFilterOptions.CheckEnvCollisions)
        if solutions is None or len(solutions) == 0:
            raise planning_error('failed to find solution')
        self.MoveObjectToPosition(r_[solutions[0],solutions[1]])

    def graspObject(self):
        ThandR=self.robot.GetManipulators()[0].GetEndEffectorTransform()
        ThandL=self.robot.GetManipulators()[1].GetEndEffectorTransform()
        self.probsmanip.SendCommand('movebothhandsstraight direction1 %lf ' %(ThandR[0,3]-ThandL[0,3]) +'%lf '%(ThandR[1,3]-ThandL[1,3]) +'%lf'%(ThandR[2,3]-ThandL[2,3]) +' direction0 %lf ' %(ThandL[0,3]-ThandR[0,3]) +'%lf '%(ThandL[1,3]-ThandR[1,3]) +'%lf'%(ThandL[2,3]-ThandR[2,3]))

    def releaseObject(self):
        ThandR=self.robot.GetManipulators()[0].GetEndEffectorTransform()
        ThandL=self.robot.GetManipulators()[1].GetEndEffectorTransform()
        self.probsmanip.SendCommand('movebothhandsstraight direction1 %lf ' %(ThandL[0,3]-ThandR[0,3]) +'%lf '%(ThandL[1,3]-ThandR[1,3]) +'%lf'%(ThandL[2,3]-ThandR[2,3]) +' direction0 %lf ' %(ThandR[0,3]-ThandL[0,3]) +'%lf '%(ThandR[1,3]-ThandL[1,3]) +'%lf'%(ThandR[2,3]-ThandL[2,3]) +' maxsteps 100')

    def graspAndMoveObject(self,jointvalues,obj):
        print ('Moving to Grasping position for object: %s'%(obj))
        self.planDualPath(obj)
        self.WaitForController()

        print ('Grasping body %s'%(obj))
        self.graspObject()
        self.WaitForController()
        
        print ('Grabbing body %s'%(obj))
        with self.env:
            self.robot.Grab(obj,self.rightArm.GetEndEffector())

        print ('Moving body %s out of the shelf'%(obj))
        self.moveObject(obj, delta=array([.2,0.0,0.0]))
        self.WaitForController()

        print ('Moving body %s to final position'%(obj))
        #change delta to give a new position
        self.moveObject(obj,delta=array([-.20,-0.0,0]))
        self.WaitForController()

        with self.env:
            print ('Releasing body %s'%(obj))
            self.robot.ReleaseAllGrabbed()
        self.releaseObject()
        self.WaitForController()

        print ('Returning to Starting position')
        self.MoveArmsToJointPosition(jointvalues)
        self.WaitForController()

        print ('Body %s successfully manipulated'%(obj))

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    schunk = Schunkplanner(env)
    time.sleep(1)    
    while True:
        # initialize
        with env:
            jointvalues=schunk.robot.GetDOFValues()
            schunk.robot.SetActiveManipulator(schunk.rightArm)
            obj=env.GetKinBody('Object1')
            while True:
                Toriginal=obj.GetTransform()
                with obj:
                    T=array(Toriginal)
                    T[0:3,3] += 0.4*(random.rand(3)-0.5) # yz only
                    obj.SetTransform(T)
                    if not env.CheckCollision(obj):
                        break
            obj.SetTransform(T)
        try:
            schunk.graspAndMoveObject(jointvalues,obj)
            schunk.WaitForController()
            print("Path Planning complete....")
        except planning_error:
            pass

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description="Schunk Manipulation planning example\nFor a dual arm robot with Schunk LWA3 arms, plan trajectories for grasping an object and manipulating it on a shelf.")
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/dualarmmanipulation.env.xml',
                      help='Scene file to load')   
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
