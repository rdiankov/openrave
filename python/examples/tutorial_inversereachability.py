#!/usr/bin/env python
"""
======================================================
Move Hand to Target: Use Inverse Reachability Database
======================================================

This tutorial shows how to generate and use the inverse-reachability database in OpenRAVE for PR2.

.. image:: ../../images/example_tutorials/ir_grasps.png
   :width: 200px

.. image:: ../../images/example_tutorials/ir_reachability_front.png
   :width: 200 px
   
.. image:: ../../images/example_tutorials/ir_10_solutions_top.png
   :width: 200 px

.. contents::

Prerequisite
------------

OpenRAVE version
~~~~~~~~~~~~~~~~

Make sure OpenRAVE is built with r1764 or higher, to check::

  roscd openrave
  cat Makefile | grep SVN_REVISION
  
IK solvers
~~~~~~~~~~

Get the IK solvers from::

  http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/kinematics.30b0c746bc8e2c01a8b0478ad7acb287/
  http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/kinematics.cf3ad23d0c0394969de000d2727ed5bc/

and store both folders under::

  ~/.openrave/

You can also generate the IK solvers for both arms. It takes about 10 minutes for each arm::

  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.robot.xml --manipname=rightarm --freejoint=r_shoulder_pan_joint --freeinc=0.01
  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.robot.xml --manipname=leftarm --freejoint=l_shoulder_pan_joint --freeinc=0.01

[optional] Download inverse-reachability database
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Get the inverse-reachability and reachability databases from::
 
  http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/robot.e66273b7c010736b42f24e2e30240dcb/

and store them under::

  ~/.openrave/robot.e66273b7c010736b42f24e2e30240dcb

You can also `Generate database`_.

Run this tutorial
-----------------

.. code-block:: bash

   openrave.py --example tutorial_inversereachability

Generate database
-----------------

Command::

  openrave.py --database inversereachability --robot=robots/pr2-beta-static.robot.xml
  
This process took ~10 hours on the PR2 base station (Intel Core2 Duo E8400 @ 3GHz with 4G of memory).

There are a few other parameters that can be set:

.. code-block:: python

   heightthresh = .05 # The max radius of the arm to perform the computation (default=0.05)
   quatthresh = .15 # The max radius of the arm to perform the computation (default=0.15)
   Nminimum = 10 # Minimum number of equivalence classes for each base distribution
   id = None # Special id differentiating inversereachability models
   jointvalues = None # String of joint values that connect the robot base link to the manipulator base link

You can also `[optional] Download inverse-reachability database`_.

Visualize database
------------------

Visualize inverse-reachability database
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This tutorial visualizes the database for one grasp, there will be another tutorial that will show how to visualize the base distribution for all possible grasps for an object. 

.. Figure:: ../../images/example_tutorials/ir_grasps.png
   :width: 640px
    
   The blue cloud indicates possible base positions for grasping the object.


Visualize reachability database
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Command::

   openrave.py --database kinematicreachability --robot=robots/pr2-beta-static.robot.xml --show

.. figure:: ../../images/example_tutorials/ir_reachability_front.png
   :width: 640 px
    
   The color indicates reachability.

.. figure:: ../../images/example_tutorials/ir_reachability_back.png
   :width: 640 px
    
   The color indicates reachability.

Use database: Move the robot where it can perform a specific grasp
------------------------------------------------------------------

.. figure:: ../../images/example_tutorials/ir_goal.png
   :width: 640 px

   Robot in a place where it can grasp the cup.
   
Set up environment
~~~~~~~~~~~~~~~~~~

.. code-block:: python

    # set up planning environment
    parser = OptionParser(description='Move base where the robot can perform target grasp using inversereachability database.')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args) # use default options
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    try:
        robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml')
        env.AddRobot(robot)
        target = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
        env.AddKinBody(target)
        # initialize target pose, for visualization and collision checking purpose only
        O_T_Target = array([[1,0,0,1],[0,1,0,0],[0,0,1,.9],[0,0,0,1]])
        target.SetTransform(array(O_T_Target))

        ...
        
    finally:
        # destroy planning environment and clean up
        env.Destroy()
        RaveDestroy()
        
Set up goal
~~~~~~~~~~~

.. code-block:: python

    # set up goal grasp transform
    # goal grasp transform specified in global frame, this equals manip.GetEndEffectorTransform() in the goal state    
    O_T_grasp = array([[ -9.88017917e-01,  -1.54339954e-01 ,  0.00000000e+00 ,  1.06494129e+00],
                       [  1.54339954e-01,  -9.88017917e-01 ,  0.00000000e+00 ,  5.51449812e-05],
                       [  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00 ,  9.55221763e-01],
                       [  0.00000000e+00 ,  0.00000000e+00,   0.00000000e+00  , 1.00000000e+00]]) 

.. figure:: ../../images/example_tutorials/ir_goal_grasp.png
   :width: 640 px
   
   Goal grasp
   
Set up robot
~~~~~~~~~~~~

.. code-block:: python

        # initialize robot pose
        v = self.robot.GetActiveDOFValues()
        v[35] = 3.14/2 # l shoulder pan
        v[56] = -3.14/2 # r shoulder pan
        # note here the torso height must be set to 0, because the database was generated for torso height=0
        v[14] = 0# torso  
        v[47] = .54 # l gripper
        self.robot.SetActiveDOFValues(v)

.. figure:: ../../images/example_tutorials/ir_before.png
   :width: 640 px

   Robot starting state
   
Load database
~~~~~~~~~~~~~

.. code-block:: python

        # load inverserechability database
        self.irmodel = inversereachability.InverseReachabilityModel(robot=self.robot)
        starttime = time.time()
        print 'loading irmodel'
        if not self.irmodel.load():
            print 'do you want to generate irmodel for your robot? it might take several hours'
            print 'or you can go to http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/ to get the database for PR2 for openrave r1764'
            input = raw_input('[Y/n]')
            if input == 'y' or input == 'Y' or input == '\\n' or input == '':
                class IrmodelOption:
                    def __init__(self,robot,heightthresh=.05,quatthresh=.15,Nminimum=10,id=None,jointvalues=None):
                        self.robot = robot
                        self.heightthresh = heightthresh
                        self.quatthresh = quatthresh
                        self.Nminimum = Nminimum
                        self.id = id
                        self.jointvalues = jointvalues
                option = IrmodelOption(self.robot)
                self.irmodel.autogenerate(option)
                self.irmodel.load()
            else:
                self.env.Destroy()
                RaveDestroy()
                sys.exit()
        print 'time to load inverse-reachability model: %fs'%(time.time()-starttime)

Get robot base distribution
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

        densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp)

- Input for computeBaseDistribution()::

   Tgrasp: 4x4 numpy.array, row major matrix, the grasp transform in global frame
           equals manip.GetEndEffectorTransform() in the goal state

- Output for computeBaseDistribution()::

   densityfn: gaussian kernel density function taking poses of openrave quaternion type, returns probabilities
   samplerfn: gaussian kernel sampler function taking number of sample and weight, returns robot base poses and joint states
   bounds: 2x3 array, bounds of samples, [[min rotation, min x, min y],[max rotation, max x, max y]]

Find valid poses from the distribution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Code fragment from `examples.mobilemanipulation`

.. code-block:: python

        # initialize sampling parameters
        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = inf
        with self.robot:
            while len(goals) < N:
                if time.time()-starttime > timeout:
                    break
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetJointValues(*jointstate)
                    # validate that base is not in collision
                    if not self.manip.CheckIndependentCollision(CollisionReport()):
                        q = self.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.manip.GetArmIndices()] = q
                            goals.append((Tgrasp,pose,values))
                        elif self.manip.FindIKSolution(Tgrasp,0) is None:
                            numfailures += 1

Move robot to valid poses
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

        print 'showing %d results'%N
        for ind,goal in enumerate(goals):
            raw_input('press ENTER to show goal %d'%ind)
            Tgrasp,pose,values = goal
            self.robot.SetTransform(pose)
            self.robot.SetJointValues(values)

.. figure:: ../../images/example_tutorials/ir_after.png
   :width: 640 px
   
   One sample from the goal distribution.
   
[optional] Overlay results
~~~~~~~~~~~~~~~~~~~~~~~~~~

Code fragment from `databases.inversereachability`

.. code-block:: python

        transparency = .8
        with self.env:
            self.env.Remove(self.robot)
            newrobots = []
            for goal in goals:
                Tgrasp,T,values = goal
                newrobot = self.env.ReadRobotXMLFile(self.robot.GetXMLFilename())
                newrobot.SetName(self.robot.GetName())
                for link in newrobot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)
                self.env.AddRobot(newrobot,True)
                with self.env:
                    newrobot.SetTransform(T)
                    newrobot.SetJointValues(values)
                newrobots.append(newrobot)


.. figure:: ../../images/example_tutorials/ir_10_solutions_top.png
   :width: 640 px
   
   Top view of ten samples.

.. figure:: ../../images/example_tutorials/ir_10_solutions_side.png
   :width: 640 px
   
   Side view of ten samples.
   
Related classes
---------------

- `databases.inversereachability` - Code that generates the inverse-reachability database
- `databases.kinematicreachability` - Code that generates the reachability database
- `examples.mobilemanipulation` - Example of mobile manipulation
   
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Huan Liu'
__copyright__ = 'Copyright (C) 2010 Huan Liu (liuhuanjim013@gmail.com)'
__license__ = 'Apache License, Version 2.0'
from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from numpy import *
else:
    from openravepy import OpenRAVEModel
    from numpy import inf, array

import time,sys
from optparse import OptionParser
from openravepy.databases import inversereachability,grasping

class InverseReachabilityDemo:
    def __init__(self,env):
        self.env = env
        self.robot = self.env.GetRobot('pr2')
        self.manip = self.robot.GetActiveManipulator()

        # initialize robot pose
        v = self.robot.GetActiveDOFValues()
        v[35] = 3.14/2 # l shoulder pan
        v[56] = -3.14/2 # r shoulder pan
        """note here the torso height must be set to 0, because the database was generated for torso height=0"""
        v[14] = 0# torso  
        v[47] = .54 # l gripper
        self.robot.SetActiveDOFValues(v)
    
        # load inverserechability database
        self.irmodel = inversereachability.InverseReachabilityModel(robot=self.robot)
        starttime = time.time()
        print 'loading irmodel'
        if not self.irmodel.load():
            print 'do you want to generate irmodel for your robot? it might take several hours'
            print 'or you can go to http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/ to get the database for PR2 for openrave r1764'
            input = raw_input('[Y/n]')
            if input == 'y' or input == 'Y' or input == '\n' or input == '':
                class IrmodelOption:
                    def __init__(self,robot,heightthresh=.05,quatthresh=.15,Nminimum=10,id=None,jointvalues=None):
                        self.robot = robot
                        self.heightthresh = heightthresh
                        self.quatthresh = quatthresh
                        self.Nminimum = Nminimum
                        self.id = id
                        self.jointvalues = jointvalues
                option = IrmodelOption(self.robot)
                self.irmodel.autogenerate(option)
                self.irmodel.load()
            else:
                self.env.Destroy()
                RaveDestroy()
                sys.exit()
        print 'time to load inverse-reachability model: %fs'%(time.time()-starttime)
        # make sure the robot and manipulator match the database
        assert self.irmodel.robot == self.robot and self.irmodel.manip == self.robot.GetActiveManipulator()   
        
    def showPossibleBasePoses(self,Tgrasp, gripper_angle=.548,N=1):
        """visualizes possible base poses for a grasp specified by Tgrasp and gripper_angle
        
        :param Tgrasp: 4x4 numpy.array, row major matrix, the grasp transform in global frame. equals manip.GetEndEffectorTransform() in the goal state
        :param gripper_angle: float, the gripper angle
        :param N: int, the number of sample poses we want to get 
        """
        # setting the gripper angle
        v = self.robot.GetActiveDOFValues()
        v[47] = gripper_angle # l gripper
        self.robot.SetActiveDOFValues(v)

        print 'showing the goal grasp'
        self.showGrasp(Tgrasp)

        # find the robot base distribution for the grasp specified by Tgrasp
        # Input for computeBaseDistribution():
        #      Tgrasp: 4x4 numpy.array, row major matrix, the grasp transform in global frame
        #              equals manip.GetEndEffectorTransform() in the goal state
        # Output for computeBaseDistribution():
        #      densityfn: gaussian kernel density function taking poses of openrave quaternion type, returns probabilities
        #      samplerfn: gaussian kernel sampler function taking number of sample and weight, returns robot base poses and joint states
        #      bounds: 2x3 array, bounds of samples, [[min rotation, min x, min y],[max rotation, max x, max y]]
        densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp)
        if densityfn == None:
            print 'the specified grasp is not reachable!'
            return
        
        # Code fragment from `examples.mobilemanipulation`
        # initialize sampling parameters
        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = inf
        with self.robot:
            while len(goals) < N:
                if time.time()-starttime > timeout:
                    break
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetJointValues(*jointstate)
                    # validate that base is not in collision
                    if not self.manip.CheckIndependentCollision(CollisionReport()):
                        q = self.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.manip.GetArmIndices()] = q
                            goals.append((Tgrasp,pose,values))
                        elif self.manip.FindIKSolution(Tgrasp,0) is None:
                            numfailures += 1
        print 'showing %d results'%N
        for ind,goal in enumerate(goals):
            raw_input('press ENTER to show goal %d'%ind)
            Tgrasp,pose,values = goal
            self.robot.SetTransform(pose)
            self.robot.SetJointValues(values)

        raw_input('press ENTER to show all results simultaneously')
        # Code fragment from `databases.inversereachability`
        transparency = .8
        with self.env: # save the environment state
            self.env.Remove(self.robot)
            newrobots = []
            for goal in goals:
                Tgrasp,T,values = goal
                newrobot = self.env.ReadRobotXMLFile(self.robot.GetXMLFilename())
                newrobot.SetName(self.robot.GetName())
                for link in newrobot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)
                self.env.AddRobot(newrobot,True)
                newrobot.SetTransform(T)
                newrobot.SetJointValues(values)
                newrobots.append(newrobot)
        print 'overlaying all results, wait for a few seconds for the viewer to update'
        time.sleep(10)
        pause()
    
    def showGrasp(self,Tgrasp,angle=.548):
        """visualizes a grasp transform
        
        :param Tgrasp: a 4x4 row-major matrix in numpy.array format in global frame
        :param angle: between 0 and .548
        """
        probot = self.robot
        pmanip = probot.GetActiveManipulator()
        v = probot.GetActiveDOFValues()
        v[47] = angle
        probot.SetActiveDOFValues(v)
        with grasping.GraspingModel.GripperVisibility(pmanip): # show only the gripper
            O_T_R = probot.GetTransform() # robot transform R in global frame O 
            O_T_G = pmanip.GetEndEffectorTransform() # grasping frame G in global frame O
            G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
            G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
            O_T_G_goal = Tgrasp # final grasping frame G_goal in global frame O 
            O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O
                            
            probot.SetTransform(O_T_R_goal)
            pause()
            probot.SetTransform(O_T_R)

def pause():
    raw_input('press ENTER to continue...')

def run(args=None):
    """
    
    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    # set up planning environment
    parser = OptionParser(description='Move base where the robot can perform target grasp using inversereachability database.')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args) # use default options 
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    try:
        robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml')
        env.AddRobot(robot)
        target = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
        env.AddKinBody(target)
        # initialize target pose, for visualization and collision checking purpose only
        O_T_Target = mat([[1,0,0,1],[0,1,0,0],[0,0,1,.9],[0,0,0,1]])
        target.SetTransform(array(O_T_Target))
        
        # set up goal grasp transform
        # goal grasp transform specified in global frame, this equals manip.GetEndEffectorTransform() in the goal state    
        O_T_grasp = array([[ -9.88017917e-01,  -1.54339954e-01 ,  0.00000000e+00 ,  1.06494129e+00],
                           [  1.54339954e-01,  -9.88017917e-01 ,  0.00000000e+00 ,  5.51449812e-05],
                           [  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00 ,  9.55221763e-01],
                           [  0.00000000e+00 ,  0.00000000e+00,   0.00000000e+00  , 1.00000000e+00]])
       
        gripper_angle = .1
    
        # use inversereachability dabase to find the possible robot base poses for the grasp  
        gr = InverseReachabilityDemo(env)
        gr.showPossibleBasePoses(O_T_grasp,gripper_angle,10)
    
    finally:
        # destroy planning environment and clean up
        env.Destroy()
        RaveDestroy()


if __name__=='__main__':
    run()
    