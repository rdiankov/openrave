#!/usr/bin/env python
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
"""Shows how to use simple gradient-based jacobians to constrain the motion of the robot while planning.

.. image:: ../../images/examples/constraintplanning.jpg
  :width: 640

**Running the Example**::

  openrave.py --example constraintplanning

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

Command-line
------------

.. shell-block:: openrave.py --example constraintplanning --help

.. [1] Mike Stilman. Task constrained motion planning in robot joint space. In: Proceedings of the IEEE International Conference on Intelligent Robots and Systems (IROS), 2007. 

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.databases import grasping,inversekinematics
from numpy import *
import numpy,time
from optparse import OptionParser

class ConstraintPlanning(metaclass.AutoReloader):
    def __init__(self,robot,randomize=False,dests=None,switchpatterns=None):
        self.envreal = robot.GetEnv()
        self.robot = robot
        self.manip = self.robot.GetActiveManipulator()
        self.ikmodel = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.gmodel = grasping.GraspingModel(robot=self.robot,target=self.envreal.GetKinBody('mug1'))
        if not self.gmodel.load():
            self.gmodel.autogenerate()
        self.basemanip = BaseManipulation(self.robot)
        self.taskmanip = TaskManipulation(self.robot,graspername=self.gmodel.grasper.plannername)

    def graspAndMove(self,showgoalcup=True):
        target = self.gmodel.target
        print 'grasping %s'%target.GetName()
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
                self.envreal.AddKinBody(showtarget,True)
                showtarget.Enable(False)
                for geom in showtarget.GetLinks()[0].GetGeometries():
                    geom.SetTransparency(0.7)

        try:
            print 'moving mug without XY rotation'
            while True:
                xyzconstraints = random.permutation(3)[0:2]
                constraintfreedoms = array([1,1,0,1,1,1]) # rotation xyz, translation xyz
                constraintfreedoms[3+xyzconstraints] = 0
                print 'planning with freedoms: ',constraintfreedoms
                Tplane = eye(4)
                Tplane[0:3,0:2] = Tplane[0:3,xyzconstraints]
                Tplane[0:3,2] = cross(Tplane[0:3,0],Tplane[0:3,1])
                Tplane[0:3,3] = self.manip.GetEndEffectorTransform()[0:3,3]
                hplane = self.envreal.drawplane(transform=Tplane,extents=[1.0,1.0],texture=reshape([1,1,0.5,0.5],(1,1,4)))

                constraintmatrix = eye(4)
                constrainterrorthresh = 0.005
                for iter in range(5):
                    with self.robot:
                        vcur = self.robot.GetDOFValues()
                        Tee = self.manip.GetEndEffectorTransform()
                        while True:
                            T = array(Tee)
                            T[0:3,3] += 0.5*(random.rand(3)-0.5)*(1.0-array(constraintfreedoms[3:]))
                            if self.manip.FindIKSolution(T,True) is not None:
                                break
                    if showtarget is not None:
                        showtarget.SetTransform(dot(T,dot(linalg.inv(self.manip.GetEndEffectorTransform()),target.GetTransform())))
                        self.envreal.UpdatePublishedBodies()
                    try:
                        self.basemanip.MoveToHandPosition(matrices=[T],maxiter=10000,maxtries=1,seedik=8,constraintfreedoms=constraintfreedoms,constraintmatrix=constraintmatrix,constrainterrorthresh=constrainterrorthresh)
                    except planning_error,e:
                        print e
                    self.robot.WaitForController(0)
        finally:
            if showtarget is not None:
                self.envreal.Remove(showtarget)

@with_destroy
def run(args=None):
    """Executes the constraintplanning example
    
    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='RRT motion planning with constraints on the robot end effector.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = ConstraintPlanning(robot)
    self.graspAndMove()

if __name__ == "__main__":
    run()
