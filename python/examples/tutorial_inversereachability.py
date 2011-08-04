#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2010-2011 Huan Liu (liuhuanjim013@gmail.com)
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
"""Generate and use the inverse-reachability database in OpenRAVE for the PR2 robot.

.. examplepre-block:: tutorial_inversereachability
   :image-width: 400

Prerequisite
------------

IK solvers
~~~~~~~~~~

Generate the IK solver for leftarm_torso. It takes several minutes:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.zae --manipname=leftarm_torso

[optional] Download inverse-reachability database
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Get the inverse-reachability and reachability databases for OpenRAVE r1974 from::
 
  http://people.csail.mit.edu/liuhuan/pr2/openrave/openrave_database/robot.1fd7b38c8ca370ea2f8d4ab79bbb074b.tgz

and decompress the file under:

.. code-block:: bash

  ~/.openrave/

You can also `Generate database`_.

Generate database
-----------------

**Generate the databases with single core**

.. code-block:: bash

  openrave.py --database inversereachability --robot=robots/pr2-beta-static.zae --manipname=leftarm_torso --ignorefreespace 
  
This process will generate both reachability and inverse-rechability databases. It will take more than 10 hours (Intel Core2 Duo E8400 @ 3GHz with 4G of memory).

**Generate the databases with multiple cores**

If you have the openrave_planning ROS stack installed, you can try a parallelized version of the reachability database generation:

First:

.. code-block:: bash

  rosmake openrave_data
  
Then:

.. code-block:: bash

  roscore
  
Finally, in a separate terminal:

.. code-block:: bash

  rosrun openrave_database kinematicreachability_ros.py --robot=robots/pr2-beta-static.zae --manipname=leftarm_torso --ignorefreespace --launchservice='8*localhost'
  
To add another computer with 8 cores add:

.. code-block:: bash
  
  --launchservice='8*newhost'

Once the reachability database is generated, generate the inversereachability database (which will take much less time): 

.. code-block:: bash

  openrave.py --database inversereachability --robot=robots/pr2-beta-static.zae --manipname=leftarm_torso --ignorefreespace 


There are a few other parameters that can be set:

.. code-block:: python

   heightthresh = .05 # The max radius of the arm to perform the computation (default=0.05)
   quatthresh = .15 # The max radius of the arm to perform the computation (default=0.15)
   show_maxnumber = 10 # Number of robots to show simultaneously (default=20)
   id = None # Special id differentiating inversereachability models
   jointvalues = None # String of joint values that connect the robot base link to the manipulator base link
   show_transparency = .8 # Transparency of the robots to show (default=.8)

You can also `[optional] Download inverse-reachability database`_.

Visualize database
------------------

Visualize inverse-reachability database
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This tutorial visualizes the database for one grasp, there will be another tutorial that will show how to visualize the base distribution for all possible grasps for an object. 

.. Figure:: ../../images/examples/tutorial_inversereachability_ir_grasps.png
   :width: 640
    
   The blue cloud indicates possible base positions for grasping the object.


Visualize reachability database
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Command::

   openrave.py --database kinematicreachability --robot=robots/pr2-beta-static.zae --show

.. figure:: ../../images/examples/tutorial_inversereachability_front.png
   :width: 640
    
   The color indicates reachability.

.. figure:: ../../images/examples/tutorial_inversereachability_back.png
   :width: 640 px
    
   The color indicates reachability.

Use database: Move the robot where it can perform a specific grasp
------------------------------------------------------------------

.. figure:: ../../images/examples/tutorial_inversereachability_goal.png
   :width: 640 px

   Robot in a place where it can grasp the cup.
   
Set up environment
~~~~~~~~~~~~~~~~~~

.. code-block:: python

    # set up planning environment
    env = Environment()
    env.SetViewer('qtcoin')
    
Set up goal
~~~~~~~~~~~

.. code-block:: python

    # set up goal grasp transform
    # goal grasp transform specified in global frame, this equals manip.GetEndEffectorTransform() in the goal state    
    O_T_grasp = array([[ -9.88017917e-01,  -1.54339954e-01 ,  0.00000000e+00 ,  1.06494129e+00],
                       [  1.54339954e-01,  -9.88017917e-01 ,  0.00000000e+00 ,  5.51449812e-05],
                       [  0.00000000e+00 ,  0.00000000e+00 ,  1.00000000e+00 ,  9.55221763e-01],
                       [  0.00000000e+00 ,  0.00000000e+00,   0.00000000e+00  , 1.00000000e+00]]) 

.. figure:: ../../images/examples/tutorial_inversereachability_goal_grasp.png
   :width: 640 px
   
   Goal grasp
   
Set up robot
~~~~~~~~~~~~

.. code-block:: python

        # initialize robot pose
        v = self.robot.GetActiveDOFValues()
        v[self.robot.GetJoint('l_shoulder_pan_joint').GetDOFIndex()]= 3.14/2
        v[self.robot.GetJoint('r_shoulder_pan_joint').GetDOFIndex()] = -3.14/2
        v[self.robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = .54
        self.robot.SetActiveDOFValues(v)

.. figure:: ../../images/examples/tutorial_inversereachability_before.png
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
            print 'or you can go to http://people.csail.mit.edu/liuhuan/pr2/openrave/openrave_database/ to get the database for PR2'
            input = raw_input('[Y/n]')
            if input == 'y' or input == 'Y' or input == '\\n' or input == '':
                class IrmodelOption:
                self.irmodel.autogenerate()
                self.irmodel.load()
            else:
                raise ValueError('')
                
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

Code fragment from sanbox/mobilemanipulation.py

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
                    self.robot.SetDOFValues(*jointstate)
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
            self.robot.SetDOFValues(values)

.. figure:: ../../images/examples/tutorial_inversereachability_after.png
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
                newrobot = RaveCreateRobot(self.env,self.robot.GetXMLId())
                newrobot.Clone(self.robot,0)
                newrobot.SetName(self.robot.GetName())
                for link in newrobot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)
                self.env.AddRobot(newrobot,True)
                with self.env:
                    newrobot.SetTransform(T)
                    newrobot.SetDOFValues(values)
                newrobots.append(newrobot)


.. figure:: ../../images/examples/tutorial_inversereachability.jpg
   :width: 640 px
   
   Top view of ten samples.

.. figure:: ../../images/examples/tutorial_inversereachability_10_solutions_side.png
   :width: 640 px
   
   Side view of ten samples.
   
Related classes
---------------

- `databases.inversereachability` - Code that generates the inverse-reachability database
- `databases.kinematicreachability` - Code that generates the reachability database

.. examplepost-block:: tutorial_inversereachability
   
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Huan Liu'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
else:
    from numpy import inf

class InverseReachabilityDemo:
    def __init__(self,robot):
        self.robot = robot
        self.env = self.robot.GetEnv()
        self.robot.SetActiveManipulator('leftarm_torso')
        self.manip = self.robot.GetActiveManipulator()

        # initialize robot pose
        v = self.robot.GetActiveDOFValues()
        v[self.robot.GetJoint('l_shoulder_pan_joint').GetDOFIndex()]= 3.14/2
        v[self.robot.GetJoint('r_shoulder_pan_joint').GetDOFIndex()] = -3.14/2
        v[self.robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = .54
        self.robot.SetActiveDOFValues(v)
    
        # load inverserechability database
        self.irmodel = databases.inversereachability.InverseReachabilityModel(robot=self.robot)
        starttime = time.time()
        print 'loading irmodel'
        if not self.irmodel.load():            
            print 'do you want to generate irmodel for your robot? it might take several hours'
            print 'or you can go to http://people.csail.mit.edu/liuhuan/pr2/openrave/openrave_database/ to get the database for PR2'
            input = raw_input('[Y/n]')
            if input == 'y' or input == 'Y' or input == '\n' or input == '':
                self.irmodel.autogenerate()
                self.irmodel.load()
            else:
                raise ValueError('')
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
        v[self.robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = gripper_angle # l gripper
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
        densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp,logllthresh=1.8)
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
                    self.robot.SetDOFValues(*jointstate)
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
            self.robot.SetDOFValues(values)

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
                newrobot.SetDOFValues(values)
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
        v[self.robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = angle
        probot.SetActiveDOFValues(v)
        with databases.grasping.GraspingModel.GripperVisibility(pmanip): # show only the gripper
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

def main(env,options):
    "Main example code."
    robot = env.ReadRobotXMLFile(options.robot)
    env.AddRobot(robot)
    if options.manipname is not None:
        robot.SetActiveManipulator(options.manipname)
    else:
        robot.SetActiveManipulator('leftarm')
    target = env.ReadKinBodyXMLFile(options.target)
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
    gr = InverseReachabilityDemo(robot)
    gr.showPossibleBasePoses(O_T_grasp,gripper_angle,10)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Move base where the robot can perform target grasp using inversereachability database.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--robot',action="store",type='string',dest='robot',default='robots/pr2-beta-static.zae',
                      help='Robot filename to use (default=%default)')
    parser.add_option('--manipname',action="store",type='string',dest='manipname',default=None,
                      help='name of manipulator to use (default=%default)')
    parser.add_option('--target',action="store",type='string',dest='target',default='data/mug2.kinbody.xml',
                      help='filename of the target to use (default=%default)')
    (options, leftargs) = parser.parse_args(args=args) # use default options 
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True) # the special setup for openrave tutorial
    main(env,options)

if __name__=='__main__':
    run()
    
