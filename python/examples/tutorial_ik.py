#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

.. image:: ../../images/examples_tutorial_ik.jpg
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

  lower,upper = [v[ikmodel.manip.GetArmIndices()] for v in ikmodel.robot.GetJointLimits()]
  robot.SetDOFValues(random.rand()*(upper-lower)+lower,ikmodel.manip.GetArmIndices()) # set random values
  if not robot.CheckSelfCollision():
      ...

Finally all the IK solutions are computed

.. code-block:: python

  solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetEndEffectorTransform(),True)


In order to render the ik solutions, create a new robot for every solution and make it trasparent

.. code-block:: python

  newrobot = env.ReadRobotXMLFile(robot.GetXMLFilename())
  for link in newrobot.GetLinks():
      for geom in link.GetGeometries():
          geom.SetTransparency(transparency)
  env.AddRobot(newrobot,True)
  newrobot.SetTransform(robot.GetTransform())
  newrobot.SetDOFValues(solution,ikmodel.manip.GetArmIndices())


"""
from __future__ import with_statement # for python 2.5
from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
else:
    from openravepy import OpenRAVEModel, OpenRAVEGlobalArguments
from numpy import random, array, linspace
from optparse import OptionParser

def run(args=None):
    """Executes tutorial_ik.

    :type args: arguments for script to parse, if not specified will use sys.argv
    """

    parser = OptionParser(description='tutorial_ik.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--transparency',action="store",type='float',dest='transparency',default=0.8,
                      help='Transparency for every robot (default=%default)')
    parser.add_option('--maxnumber',action="store",type='int',dest='maxnumber',default=10,
                      help='Max number of robots to render (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load(options.scene)
    while True:
        robot = env.GetRobots()[0]
        # generate the ik solver
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        with env:
            # move the robot in a random collision-free position and call the IK
            while True:
                lower,upper = [v[ikmodel.manip.GetArmIndices()] for v in ikmodel.robot.GetJointLimits()]
                robot.SetDOFValues(random.rand()*(upper-lower)+lower,ikmodel.manip.GetArmIndices()) # set random values
                if not robot.CheckSelfCollision():
                    solutions = ikmodel.manip.FindIKSolutions(ikmodel.manip.GetEndEffectorTransform(),True)
                    if solutions is not None and len(solutions) > 0: # if found, then break
                        break
            
            print 'found %d solutions, rendering solutions:'%len(solutions)
            newrobots = []
            if len(solutions) < options.maxnumber:
                inds = range(len(solutions))
            else:
                inds = array(linspace(0,len(solutions)-1,options.maxnumber),int)
            for ind in inds:
                print ind
                newrobot = env.ReadRobotXMLFile(robot.GetXMLFilename())
                for link in newrobot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(options.transparency)
                env.AddRobot(newrobot,True)
                newrobot.SetTransform(robot.GetTransform())
                newrobot.SetDOFValues(solutions[ind],ikmodel.manip.GetArmIndices())
                newrobots.append(newrobot)
            for link in robot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(options.transparency)

        raw_input('press any key for another configuration')
        # remove the robots
        for newrobot in newrobots:
            env.Remove(newrobot)
        del newrobots

if __name__ == "__main__":
    run()
