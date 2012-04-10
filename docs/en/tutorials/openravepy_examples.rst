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
      raveLogInfo("Robot "+robot.GetName()+" has "+repr(robot.GetDOF())+" joints with values:\\n"+repr(robot.GetDOFValues()))
      robot.SetDOFValues([0.5],[0]) # set joint 0 to value 0.5
      T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
      raveLogInfo("The transformation of link 1 is:\n"+repr(T))

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

Creating/Updating a Body
------------------------

Creates a box and then update the box's geometry dynamically.

.. code-block:: python

  from openravepy import *
  import numpy, time
  env = Environment() # create openrave environment
  env.SetViewer('qtcoin') # attach viewer (optional)
  with env:
      body = RaveCreateKinBody(env,'')
      body.SetName('testbody')
      body.InitFromBoxes(numpy.array([[0,0,0,0.1,0.2,0.3]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
      env.AddKinBody(body)

  time.sleep(4) # sleep 4 seconds
  with env:
      env.Remove(body)
      body.InitFromBoxes(numpy.array([[-0.4,0,0,0.1,0.2,0.3],[0.4,0,0,0.1,0.2,0.9]]),True) # set geometry as two boxes
      env.AddKinBody(body)


Using a BiRRT Planner
---------------------

Use a planner to get a collision free path to a configuration space goal.

.. code-block:: python

  from openravepy import *
  env = Environment() # create the environment
  env.SetViewer('qtcoin') # start the viewer
  env.Load('data/lab1.env.xml') # load a scene
  robot = env.GetRobots()[0] # get the first robot
  RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug
  
  manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
  manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19]) # call motion planner with goal joint angles
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

      raveLogInfo('Tee is: '+repr(Tee))

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

  raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original


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

Custom IK Filters
-----------------

Set a custom IK filter to abort computation after 100ms.

.. code-block:: python

  from openravepy import *
  import numpy, time
  env=Environment()
  env.Load('data/pr2test1.env.xml')

  robot=env.GetRobots()[0]
  manip = robot.SetActiveManipulator('leftarm_torso')
  lower,upper = robot.GetDOFLimits(manip.GetArmIndices()) # get the limits of just the arm indices
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
      ikmodel.autogenerate()

  maxtime = 0.1 # 100ms
  while True:
      with env:
          robot.SetDOFValues(lower+numpy.random.rand(len(lower))*(upper-lower),manip.GetArmIndices()) # set a random values to just the arm
          incollision = not env.CheckCollision(robot) and not robot.CheckSelfCollision()
          starttime = time.time()
          def timeoutfilter(values, manip, ikparam):
              return IkFilterReturn.Quit if time.time()-starttime > maxtime else IkFilterReturn.Success

          handle=manip.GetIkSolver().RegisterCustomFilter(0,timeoutfilter)
          success = manip.FindIKSolution(manip.GetIkParameterization(IkParameterization.Type.Transform6D),IkFilterOptions.CheckEnvCollisions)
          raveLogInfo('in collision: %d, real success: %d, time passed: %f'%(incollision,success is not None,time.time()-starttime))

Sending Torques to a Physics Engine
-----------------------------------

Shows how to set a physics engine and send torque commands to the robot

.. code-block:: python

  from openravepy import *
  import numpy, time
  env=Environment()
  env.Load('data/lab1.env.xml')
  env.SetViewer('qtcoin')
  with env:
      # set a physics engine
      physics = RaveCreatePhysicsEngine(env,'ode')
      env.SetPhysicsEngine(physics)
      physics.SetGravity(numpy.array((0,0,-9.8)))

      robot = env.GetRobots()[0]
      robot.GetLinks()[0].SetStatic(True)
      env.StopSimulation()
      env.StartSimulation(timestep=0.001)

  while True:
      torques = 100*(numpy.random.rand(robot.GetDOF())-0.5)
      for i in range(100):
          robot.SetJointTorques(torques,True)
          time.sleep(0.01)

Testing a Grasp
---------------

Loads the grasping model and moves the robot to the first grasp found

.. code-block:: python

  from openravepy import *
  import numpy, time
  env=Environment()
  env.Load('data/lab1.env.xml')
  env.SetViewer('qtcoin')
  robot = env.GetRobots()[0]
  target = env.GetKinBody('mug1')
  gmodel = databases.grasping.GraspingModel(robot,target)
  if not gmodel.load():
      gmodel.autogenerate()
  
  validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
  gmodel.moveToPreshape(validgrasps[0])
  Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
  basemanip = interfaces.BaseManipulation(robot)
  basemanip.MoveToHandPosition(matrices=[Tgoal])
  robot.WaitForController(0)
  taskmanip = interfaces.TaskManipulation(robot)
  taskmanip.CloseFingers()
  robot.WaitForController(0)

Returning a Trajectory
----------------------

Get a trajectory to a grasp before executing it.

.. code-block:: python

  from openravepy import *
  import numpy, time
  env=Environment()
  env.Load('data/lab1.env.xml')
  env.SetViewer('qtcoin')
  robot = env.GetRobots()[0]
  target = env.GetKinBody('mug1')
  gmodel = databases.grasping.GraspingModel(robot,target)
  if not gmodel.load():
      gmodel.autogenerate()
  
  validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
  basemanip = interfaces.BaseManipulation(robot)
  with robot:
      grasp = validgrasps[0]
      gmodel.setPreshape(grasp)
      T = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
      traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)

  raveLogInfo('traj has %d waypoints, last waypoint is: %s'%(traj.GetNumWaypoints(),repr(traj.GetWaypoint(-1))))
  robot.GetController().SetPath(traj)
  robot.WaitForController(0)

Using Link Statistics
---------------------

Speed-up planning and increase precision by using link statistics

.. code-block:: python

  from openravepy import *
  import time
  env = Environment() # create the environment
  env.SetViewer('qtcoin') # start the viewer
  env.Load('data/pr2test1.env.xml') # load a scene
  robot = env.GetRobots()[0] # get the first robot
  robot.SetActiveManipulator('leftarm_torso')
  goal = [ 0.24865706, 0.09362862, 0, 2.21558089, -1.00901245, -1.18879056, -0.74486442, 0]
  # normal planning
  manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
  starttime = time.time()
  manipprob.MoveManipulator(goal=goal,execute=False)
  raveLogInfo('non-linkstatistics planning time: %fs'%(time.time()-starttime))
  # using link statistics
  lmodel=databases.linkstatistics.LinkStatisticsModel(robot)
  if not lmodel.load():
      lmodel.autogenerate()
  lmodel.setRobotResolutions(0.01) # set resolution given smallest object is 0.01m
  lmodel.setRobotWeights() # set the weights for planning
  starttime = time.time()
  traj=manipprob.MoveManipulator(goal=goal,execute=False,outputtrajobj=True)
  raveLogInfo('linkstatistics planning time: %fs'%(time.time()-starttime))
  robot.GetController().SetPath(traj)
  robot.WaitForController(0)

Saving Viewer Image
-------------------

Save a 640x480 image from the viewer.

.. code-block:: python

  from openravepy import *
  import scipy
  import time
  env = Environment() # create openrave environment
  env.SetViewer('qtcoin')
  env.Load('data/lab1.env.xml') # load a simple scene
  time.sleep(1) # wait for viewer to initialize
  
  env.GetViewer().SendCommand('SetFiguresInCamera 1') # also shows the figures in the image
  I = env.GetViewer().GetCameraImage(640,480,  env.GetViewer().GetCameraTransform(),[640,640,320,240])
  scipy.misc.imsave('openrave.jpg',I)

Recording Videos
----------------

Start and stop recording videos using the Python API.

.. code-block:: python

  from openravepy import *
  import scipy
  import time
  env = Environment() # create openrave environment
  env.SetViewer('qtcoin')
  env.Load('data/lab1.env.xml') # load a simple scene
  
  recorder = RaveCreateModule(env,'viewerrecorder')
  env.AddModule(recorder,'')
  filename = 'openrave.mpg'
  codec = 13 # mpeg2
  recorder.SendCommand('Start 640 480 30 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,env.GetViewer().GetName()))

Parabolic Retiming 
------------------

Do parabolic segment retiming to a goal position without checking collisions.

.. code-block:: python

  from openravepy import *
  env = Environment() # create openrave environment
  env.SetViewer('qtcoin')
  env.Load('robots/barrettwam.robot.xml') # load a simple scene
  robot=env.GetRobots()[0]
  with env:  
      lower,upper = robot.GetActiveDOFLimits()
      goalvalues = random.rand(len(lower))*(upper-lower)+lower
      traj = RaveCreateTrajectory(env,'')
      traj.Init(robot.GetActiveConfigurationSpecification())
      traj.Insert(0,robot.GetActiveDOFValues())
      traj.Insert(1,goalvalues)
      planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1,plannername='ParabolicTrajectoryRetimer')
      print 'duration',traj.GetDuration()
  robot.GetController().SetPath(traj)
  robot.WaitForController(0)

Logging
-------

Save the current scene using the :ref:`probleminstance-logging` plugin:

.. code-block:: python

  from openravepy import *
  env = Environment() # create openrave environment
  env.Load('data/lab1.env.xml') # load a simple scene
  
  logger = RaveCreateModule(env,'logging')
  logger.SendCommand('savescene filename myscene.env.xml')
