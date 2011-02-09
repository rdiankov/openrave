#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
"""

.. image:: ../../images/example_tutorials/iksolutions.jpg
  :height: 200

Shows how to call an IK solver and render all the solutions.

First the inversekinematics database generator is called querying a **Transform6D** IK.

.. code-block:: python
  
  ikmodel = database.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
      ikmodel.autogenerate()

.. lang-block:: ja

  衝突していない状態に動かす

.. lang-block:: en

  Then a collision-free random configuration is set on the robot:

.. 
.. code-block:: python

  lower,upper = [v[ikmodel.manip.GetArmIndices()] for v in ikmodel.robot.GetDOFLimits()]
  robot.SetDOFValues(random.rand()*(upper-lower)+lower,ikmodel.manip.GetArmIndices()) # set random values
  if not robot.CheckSelfCollision():
      ...

Finally all the IK solutions are computed

.. code-block:: python

  solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetEndEffectorTransform(),True)


In order to render the ik solutions, create a new robot for every solution and make it trasparent

.. code-block:: python

  newrobot = RaveCreateRobot(env,robot.GetXMLId())
  newrobot.Clone(robot,0)
  for link in newrobot.GetLinks():
      for geom in link.GetGeometries():
          geom.SetTransparency(transparency)
  env.AddRobot(newrobot,True)
  newrobot.SetTransform(robot.GetTransform())
  newrobot.SetDOFValues(solution,ikmodel.manip.GetArmIndices())


"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
else:
    from openravepy import OpenRAVEModel, OpenRAVEGlobalArguments
from numpy import random, array, linspace
from optparse import OptionParser
import time

def run(args=None):
    """Executes tutorial_iksolutions.

    :type args: arguments for script to parse, if not specified will use sys.argv

    **Help**
    
    .. shell-block:: openrave.py --example tutorial_solutions --help
    """

    parser = OptionParser(description='Shows how to generate a 6D inverse kinematics solver and use it for getting all solutions.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--transparency',action="store",type='float',dest='transparency',default=0.8,
                      help='Transparency for every robot (default=%default)')
    parser.add_option('--maxnumber',action="store",type='int',dest='maxnumber',default=10,
                      help='Max number of robots to render (default=%default)')
    parser.add_option('--manipname',action="store",type='string',dest='manipname',default=None,
                      help='name of manipulator to use (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    if options.manipname is not None:
        robot.SetActiveManipulator(options.manipname)
    newrobots = []
    for ind in range(options.maxnumber):
        newrobot = RaveCreateRobot(env,robot.GetXMLId())
        newrobot.Clone(robot,0)
        for link in newrobot.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(options.transparency)
        newrobots.append(newrobot)
    for link in robot.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(options.transparency)

    while True:
        # generate the ik solver
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        with env:
            # move the robot in a random collision-free position and call the IK
            while True:
                lower,upper = [v[ikmodel.manip.GetArmIndices()] for v in ikmodel.robot.GetDOFLimits()]
                robot.SetDOFValues(random.rand()*(upper-lower)+lower,ikmodel.manip.GetArmIndices()) # set random values
                if not robot.CheckSelfCollision():
                    solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetEndEffectorTransform(),IkFilterOptions.CheckEnvCollisions)
                    if solutions is not None and len(solutions) > 0: # if found, then break
                        break
            
            print 'found %d solutions, rendering solutions:'%len(solutions)
            if len(solutions) < options.maxnumber:
                inds = range(len(solutions))
            else:
                inds = array(linspace(0,len(solutions)-1,options.maxnumber),int)
            for i,ind in enumerate(inds):
                print ind
                newrobot = newrobots[i]
                env.AddRobot(newrobot,True)
                newrobot.SetTransform(robot.GetTransform())
                newrobot.SetDOFValues(solutions[ind],ikmodel.manip.GetArmIndices())

        env.UpdatePublishedBodies()
        print('waiting...')
        time.sleep(20)
        # remove the robots
        for newrobot in newrobots:
            env.Remove(newrobot)
    del newrobots

if __name__ == "__main__":
    run()
