#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2012 Rosen Diankov (rosen.diankov@gmail.com)
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
"""Shows how to use simple gradient-based jacobians to constrain the motion of the robot while planning.

.. examplepre-block:: constraintplanning

Description
-----------

A good introduction to these methods can be found in [1]_.

A GripperJacobianConstrains class is defined in the rmanipulation plugin. It holds a RetractionConstraint function that takes in a robot configuration, and constrains the manipulator to lie in a certain manifold specified by a target frame and the degrees of freedom to constraint (translation and rotation about axes). If the projection succeeded, it returns true along with the new configuration. Such functions can be set to any planner at any time by filling the PlannerBase::PlannerParameters::_constraintfn field. In the example above, the constraint function is set inside basemanipulation.h in the following way: 

.. code-block:: cpp

  PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
  
  // ...
  // other params initialization like distance metrics (_distmetricfn)
  // ...
  
  // constrained params initialization
  Transform tConstraintTargetWorldFrame; // target frame in world coordinates
  RobotBase::ManipulatorPtr manip = robot->GetActiveManipulator(); // manipulator
  boost::array<double,6> vconstraintfreedoms = {{1,1,0,0,0,0}}; // rotx, roty, rotz, transx, transy, transz
  double constrainterrorthresh = 0.02; // threshold
  // create the class
  boost::shared_ptr<CM::GripperJacobianConstrains<double> > pconstraints(new CM::GripperJacobianConstrains<double>(manip,tConstraintTargetWorldFrame,vconstraintfreedoms,constrainterrorthresh));
  
  // set the distance metric used from the one already defined in params
  pconstraints->_distmetricfn = params->_distmetricfn;
  
  // set the constraint function
  params->_constraintfn = boost::bind(&CM::GripperJacobianConstrains<double>::RetractionConstraint,pconstraints,_1,_2,_3);

.. examplepost-block:: constraintplanning

.. [1] Mike Stilman. Task constrained motion planning in robot joint space. In: Proceedings of the IEEE International Conference on Intelligent Robots and Systems (IROS), 2007. 

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class ConstraintPlanning:
    def __init__(self,robot,randomize=False,dests=None,switchpatterns=None,plannername=None):
        self.envreal = robot.GetEnv()
        self.robot = robot
        self.manip = self.robot.GetActiveManipulator()
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.gmodel = databases.grasping.GraspingModel(robot=self.robot,target=self.envreal.GetKinBody('mug1'))
        if not self.gmodel.load():
            self.gmodel.autogenerate()
        self.basemanip = interfaces.BaseManipulation(self.robot,plannername=plannername)
        self.taskmanip = interfaces.TaskManipulation(self.robot,graspername=self.gmodel.grasper.plannername,plannername=plannername)

    def graspAndMove(self,showgoalcup=True):
        target = self.gmodel.target
        print('grasping %s'%target.GetName())
        # only use one grasp since preshape can change
        validgrasps,validindices = self.gmodel.computeValidGrasps(returnnum=10)
        validgrasp=validgrasps[random.randint(len(validgrasps))]
        with self.robot:
            self.gmodel.setPreshape(validgrasp)
            jointvalues = self.robot.GetDOFValues()
        self.robot.GetController().SetDesired(jointvalues)
        self.robot.WaitForController(0)
        matrices = [self.gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)]
        self.basemanip.MoveToHandPosition(matrices=matrices,maxiter=1000,maxtries=1,seedik=10)
        self.robot.WaitForController(0)
        self.taskmanip.CloseFingers()
        self.robot.WaitForController(0)
        self.robot.Grab(target)
        showtarget = None
        if showgoalcup:
            # create a dummy cup to show destinations
            with self.envreal:
                showtarget = RaveCreateKinBody(self.envreal,'')
                showtarget.Clone(target,0)
                self.envreal.Add(showtarget,True)
                showtarget.Enable(False)
                for geom in showtarget.GetLinks()[0].GetGeometries():
                    geom.SetTransparency(0.7)

        try:
            print('moving mug without global XY rotation')
            while True:
                # find the z rotation axis of the cup's frame
                localrotaxis = dot(linalg.inv(target.GetTransform()[0:3,0:3]),[0,0,1])
                xyzconstraints = random.permutation(3)[0:2]
                constraintfreedoms = ones(6) # rotation xyz, translation xyz
                constraintfreedoms[3+xyzconstraints] = 0
                index = argmax(abs(localrotaxis))
                constraintfreedoms[index] = 0
                localrotaxis = zeros(3)
                localrotaxis[index] = 1
                print(localrotaxis)
                print('planning with freedoms: %s, local rot axis: %s '%(constraintfreedoms,localrotaxis))
                
                constrainterrorthresh = 0.005
                for iter in range(3):
                    with self.robot:
                        vcur = self.robot.GetDOFValues()
                        Tee = self.manip.GetTransform()
                        while True:
                            Ttarget = target.GetTransform()
                            Tlocaltarget = matrixFromAxisAngle(localrotaxis*2*(random.rand()-0.5))
                            Tlocaltarget[0:3,3] = 0.5*(random.rand(3)-0.5)*(1.0-array(constraintfreedoms[3:]))
                            Tnewtarget = dot(Ttarget,Tlocaltarget)
                            T = dot(Tnewtarget, dot(linalg.inv(target.GetTransform()), Tee))
                            if self.manip.FindIKSolution(T,IkFilterOptions.CheckEnvCollisions) is not None:
                                break
                    if showtarget is not None:
                        showtarget.SetTransform(Tnewtarget)
                        self.envreal.UpdatePublishedBodies()
                        Tplane = array(Ttarget)
                        Tplane[0:3,0:2] = Tplane[0:3,xyzconstraints]
                        Tplane[0:3,2] = cross(Tplane[0:3,0],Tplane[0:3,1])
                        hplane = self.envreal.drawplane(transform=Tplane,extents=[1.0,1.0],texture=reshape([1,1,0.5,0.5],(1,1,4)))

                    try:
                        constrainttaskmatrix=dot(linalg.inv(Tee),target.GetTransform())
                        constraintmatrix = linalg.inv(target.GetTransform())
                        self.basemanip.MoveToHandPosition(matrices=[T],maxiter=3000,maxtries=1,seedik=40,constraintfreedoms=constraintfreedoms,constraintmatrix=constraintmatrix, constrainttaskmatrix=constrainttaskmatrix,constrainterrorthresh=constrainterrorthresh,steplength=0.002)
                    except planning_error as e:
                        print(e)
                    self.robot.WaitForController(0)
        finally:
            if showtarget is not None:
                self.envreal.Remove(showtarget)

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = ConstraintPlanning(robot)
    self.graspAndMove()

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='RRT motion planning with constraints on the robot end effector.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
