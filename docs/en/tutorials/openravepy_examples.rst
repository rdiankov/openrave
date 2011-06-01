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

Rotating a Body
---------------

Rotates all bodies along world z-direction by 45 degrees:

.. code-block:: python

  from openravepy import *
  import numpy
  env = Environment() # create openrave environment
  env.SetViewer('qtcoin') # attach viewer (optional)
  env.Load('data/lab1.env.xml') # load a simple scene
  
  Tz = matrixFromAxisAngle([0,0,numpy.pi/4])
  with env:
      for body in env.GetBodies():
         body.SetTransform(numpy.dot(Tz,body.GetTransform()))

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

Inverse Kinematics: Transform6D
-------------------------------

Shows how to get all 6D IK solutions

.. code-block:: python

  from openravepy import *
  import numpy
  env = Environment() # create the environment
  env.SetViewer('qtcoin') # start the viewer
  env.Load('data/pr2test1.env.xml') # load a scene
  robot = env.GetRobots()[0] # get the first robot
  
  manip = robot.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
      ikmodel.autogenerate()
  
  with env: # lock environment
      Tgoal = numpy.array([[0,-1,0,-0.21],[-1,0,0,0.04],[0,0,-1,0.92],[0,0,0,1]])
      sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
      with robot: # save robot state
          robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
          Tee = manip.GetEndEffectorTransform()
          env.UpdatePublishedBodies() # allow viewer to update new robot
          raw_input('press any key')
      
      print Tee

Inverse Kinematics: Translation3D
---------------------------------

Moves a robot in a random position, gets the end effector transform, and calls IK on it.

.. code-block:: python

  from openravepy import *
  env = Environment() # create the environment
  env.SetViewer('qtcoin') # start the viewer
  env.Load('data/katanatable.env.xml') # load a scene
  robot = env.GetRobots()[0] # get the first robot
  
  manip = robot.GetActiveManipulator()
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
  if not ikmodel.load():
      ikmodel.autogenerate()
  
  with robot: # lock environment and save robot state
      robot.SetDOFValues([2.58, 0.547, 1.5, -0.7],[0,1,2,3]) # set the first 4 dof values
      Tee = manip.GetEndEffectorTransform() # get end effector
      ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
      sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions

  h = env.plot3(Tee[0:3,3],10) # plot one point
  with robot: # save robot state
      for sol in sols[::10]: # go through every 10th solution
          robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
          env.UpdatePublishedBodies() # allow viewer to update new robot
          raw_input('press any key')

  print robot.GetDOFValues() # robot state is restored to original
    

Plan to End Effector Position
-----------------------------

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

Grabbing Object with Planner
----------------------------

Shows how to use a planner to close and open a gripper using planning.

.. code-block:: python

  from openravepy import *
  import numpy
  env = Environment() # create openrave environment
  env.SetViewer('qtcoin') # attach viewer (optional)
  env.Load('data/lab1.env.xml') # load a simple scene
  
  robot=env.GetRobots()[0]
  manip = robot.GetActiveManipulator()
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
      ikmodel.autogenerate()

  manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
  Tgoal = numpy.array([[0,-1,0,-0.23],[-1,0,0,-0.1446],[0,0,-1,0.85],[0,0,0,1]])
  res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
  robot.WaitForController(0) # wait

  taskprob = interfaces.TaskManipulation(robot) # create the interface for task manipulation programs
  taskprob.CloseFingers() # close fingers until collision
  robot.WaitForController(0) # wait
  with env:
      robot.Grab(env.GetKinBody('mug4'))

  manipprob.MoveManipulator(numpy.zeros(len(manip.GetArmIndices()))) # move manipulator to all zeros

Logging
-------

Save the current scene using the :ref:`probleminstance-logging` plugin:

.. code-block:: python

  from openravepy import *
  env = Environment() # create openrave environment
  env.Load('data/lab1.env.xml') # load a simple scene
  
  logger = env.CreateProblem('logging')
  logger.SendCommand('savescene filename myscene.env.xml')
