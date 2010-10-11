#!/usr/bin/env python
"""
Move Hand To Target: Find Transforms
====================================

This tutorial shows how to find the transform that moves the hand to the target.

.. contents::

Run this tutorial
-----------------

.. code-block:: bash

   openrave.py --example tutorial_grasp_transform

Initial setup
-------------
.. figure:: ../../images/example_tutorials/grasp_transform_tutorial_initial.png
   :width: 642 px
   
   Initial robot configuration

Initialize the environment
~~~~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: python

    # set up planning environment
    env = Environment()
    env.SetViewer('qtcoin')

Load the robot and object
~~~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: python

    robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml')
    env.AddRobot(robot)
    target = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
    env.AddKinBody(target)

Move the object
~~~~~~~~~~~~~~~
.. code-block:: python

    # init target pose
    O_T_Target = array([[1,0,0,1],
                        [0,1,0,1],
                        [0,0,1,1],
                        [0,0,0,1]])
    target.SetTransform(O_T_Target)
    
Open arms, raise torso, open gripper
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: python

    # init robot pose
    v = robot.GetActiveDOFValues()
    v[35] = 3.14/2 # l shoulder pan
    v[56] = -3.14/2 # r shoulder pan
    v[14] = .31 # torso
    v[47] = .54 # l gripper
    robot.SetActiveDOFValues(v)

Doing transform
---------------
Given:

.. code-block:: python

    O_T_R = self.robot.GetTransform() # robot transform R in global frame O 
    O_T_G = self.robot.GetActiveManipulator().GetEndEffectorTransform() # grasping frame G in global frame O
    
Want:

.. code-block:: python
   
    O_T_R_goal # final robot transform R_goal in global frame O

Solution:

.. code-block:: python

    G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
    G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
    O_T_G_goal = self.robot.GetTransform() # final grasping frame G_goal in global frame O 
    O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O

.. figure:: ../../images/example_tutorials/grasp_transform_tutorial_O_T_R.png
   :width: 642 px
   
   robot frame R in global frame O

.. figure:: ../../images/example_tutorials/grasp_transform_tutorial_O_T_Target.png
   :width: 642 px
   
   target frame Target in global frame O
   
.. figure:: ../../images/example_tutorials/grasp_transform_tutorial_O_T_G.png
   :width: 642 px
   
   grasping frame G in global frame O
   
.. figure:: ../../images/example_tutorials/grasp_transform_tutorial_final.png
   :width: 642 px
   
   Robot goal configuration
   
Related Functions
-----------------
 `Environment.ReadRobotXMLFile` , `Environment.AddRobot` , `Environment.ReadKinBodyXMLFile` , `Environment.AddKinBody` , 
 `KinBody.GetTransform` , `KinBody.SetTransform` , 
 `Robot.GetActiveDOFValues` , `Robot.SetActiveDOFValues` , `Robot.GetTransform` , `Robot.SetTransform` , `Robot.GetActiveManipulator`,
 `Robot.Manipulator.GetEndEffectorTransform`
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

from optparse import OptionParser

class GraspTransform:
    def __init__(self,env,target):
        self.env = env
        self.robot = env.GetRobots()[0]
        self.target= target

    def drawTransform(self,T,length=0.1):
        """draws a set of arrows around a coordinate system
        """
        return [self.env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=0.01,color=[1.0,0.0,0.0]),
                self.env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=0.01,color=[0.0,1.0,0.0]),
                self.env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=0.01,color=[0.0,0.0,1.0])]
            
    def showGrasp(self,Tgrasp):
        """visualizes the robot configuration when robot.GetActiveManipulator().GetEndEffectorTransform()==Tgrasp
        
        :param Tgrasp: a row-major 4x4 matrix in numpy.array format
        """
        O_T_R = self.robot.GetTransform() # robot transform R in global frame O 
        O_T_G = self.robot.GetActiveManipulator().GetEndEffectorTransform() # grasping frame G in global frame O
        G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
        G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
        O_T_G_goal = Tgrasp # final grasping frame G_goal in global frame O 
        O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O                
        self.robot.SetTransform(O_T_R_goal)

def run(args=None):
    """
    
    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    # set up planning environment
    parser = OptionParser(description='Find the transform that moves the hand to target')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args) # use default options 
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True) # the special setup for openrave tutorial
    try:
        robot = env.ReadRobotXMLFile('robots/pr2-beta-static.robot.xml')
        env.AddRobot(robot)
        target = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
        env.AddKinBody(target)
    
        # init target pose
        O_T_Target = array([[1,0,0,1],
                            [0,1,0,1],
                            [0,0,1,1],
                            [0,0,0,1]])
        target.SetTransform(O_T_Target)
    
        # init robot pose: l_shoulder_pan, r_shoulder_pan, torso, l_gripper
        robot.SetJointValues([pi/2,-pi/2,0.31,0.54],[35,56,14,47])
        gt = GraspTransform(env,target)
        handles = []
        raw_input('This demo shows how to find the transform that moves the hand to the target.\npress ENTER to continue...')
        print 'showing robot transform in global frame O_T_R'
        handles = gt.drawTransform(gt.robot.GetTransform())
        raw_input('press ENTER to continue...')
        print 'showing target transform in global frame O_T_Target'
        handles = gt.drawTransform(gt.target.GetTransform())
        raw_input('press ENTER to continue...')
        print 'showing grasping frame in global frame O_T_G'
        handles = gt.drawTransform(gt.robot.GetActiveManipulator().GetEndEffectorTransform())
        raw_input('press ENTER to continue...')
        raw_input('Guess what the robot will look like when the hand is on the target?\npress ENTER to continue...')
        gt.showGrasp(target.GetTransform())
        raw_input('press ENTER to exit')
    except openrave_exception, e:
        print e
    finally:
        # destroy planning environment and clean up
        env.Destroy()
        RaveDestroy()

if __name__ == '__main__':
    run()
