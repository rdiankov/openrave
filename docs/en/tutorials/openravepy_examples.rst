Quick examples in openravepy
============================

Short snippets of code to help get the feel for OpenRAVE functionality. Code can be directly executed inside the python interpreter.


Simple Environment Loading
--------------------------

Loads up an environment, attaches a viewer, loads a scene, and requests information about the robot.

.. code-block:: python

  from openravepy import *
  env = Environment() # create openrave environment
  env.SetViewer('qtcoin') # attach viewer (optional)
  env.Load('data/lab1.env.xml') # load a simple scene
  robot = env.GetRobots()[0] # get the first robot
  with env: # lock the environment since robot will be used
      print "Robot ",robot.GetName()," has ",robot.GetDOF()," joints with values:\\n",robot.GetJointValues()
      robot.SetDOFValues([0.5],[0]) # set joint 0 to value 0.5
      T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
      print "The transformation of link 1 is:\n",T

Using a BiRRT Planner
---------------------

Use a planner to get a collision free path to a configuration space goal.

.. code-block:: python

  from openravepy import *
  env = Environment() # create the environment
  env.SetViewer('qtcoin') # start the viewer
  env.Load('data/lab1.env.xml') # load a scene
  robot = env.GetRobots()[0] # get the first robot
  manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
  res = manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19]) # call motion planner with goal joint angles
  robot.WaitForController(0) # wait

Move End Effector with IK
-------------------------

Use a planner to get a collision free path to a workspace goal of the end effector.

.. code-block:: python

  from openravepy import *
  import numpy
  env = Environment() # create the environment
  env.SetViewer('qtcoin') # start the viewer
  env.Load('data/pr2test1.env.xml') # load a scene
  robot = env.GetRobots()[0] # get the first robot
  robot.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm + torso
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
      ikmodel.autogenerate()
  manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
  Tgoal = numpy.array([[0,-1,0,-0.21],[-1,0,0,0.04],[0,0,-1,0.92],[0,0,0,1]])
  res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
  robot.WaitForController(0) # wait

Logging
-------

Save the current scene using the '''logging''' plugin:

.. code-block:: python

  from openravepy import *
  env = Environment() # create openrave environment
  env.Load('data/lab1.env.xml') # load a simple scene
  logger = env.CreateProblem('logging')
  logger.SendCommand('savescene filename myscene.env.xml')
