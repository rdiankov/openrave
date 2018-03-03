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
"""Shows how to call an IK solver and render all the solutions.

.. examplepre-block:: tutorial_iksolutions

Description
-----------

First the inversekinematics database generator is called querying a **Transform6D** IK.

.. code-block:: python
  
  ikmodel = database.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
      ikmodel.autogenerate()

Then a collision-free random configuration is set on the robot:
 
.. code-block:: python

  lower,upper = [v[ikmodel.manip.GetArmIndices()] for v in ikmodel.robot.GetDOFLimits()]
  robot.SetDOFValues(random.rand()*(upper-lower)+lower,ikmodel.manip.GetArmIndices()) # set random values
  if not robot.CheckSelfCollision():
      ...

Finally all the IK solutions are computed

.. code-block:: python

  solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetTransform(),True)


In order to render the ik solutions, create a new robot for every solution and make it trasparent

.. code-block:: python

  newrobot = RaveCreateRobot(env,robot.GetXMLId())
  newrobot.Clone(robot,0)
  for link in newrobot.GetLinks():
      for geom in link.GetGeometries():
          geom.SetTransparency(transparency)
  env.Add(newrobot,True)
  newrobot.SetTransform(robot.GetTransform())
  newrobot.SetDOFValues(solution,ikmodel.manip.GetArmIndices())

.. examplepost-block:: tutorial_iksolutions
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
                    solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetTransform(),IkFilterOptions.CheckEnvCollisions)
                    if solutions is not None and len(solutions) > 0: # if found, then break
                        break
            
            print('found %d solutions, rendering solutions:'%len(solutions))
            if len(solutions) < options.maxnumber:
                inds = range(len(solutions))
            else:
                inds = array(linspace(0,len(solutions)-1,options.maxnumber),int)
            for i,ind in enumerate(inds):
                print(ind)
                newrobot = newrobots[i]
                env.Add(newrobot,True)
                newrobot.SetTransform(robot.GetTransform())
                newrobot.SetDOFValues(solutions[ind],ikmodel.manip.GetArmIndices())

        env.UpdatePublishedBodies()
        print('waiting...')
        time.sleep(20)
        # remove the robots
        for newrobot in newrobots:
            env.Remove(newrobot)
    del newrobots

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
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
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
