Advanced openravepy
===================

Exceptions
----------

OpenRAVE C++ exceptions in the form of the openrave_exception class are automatically translated to a '''openrave_exception''' class in python. An openrave exception can be caught by:

.. code-block:: python

  try:
      env = Environment()
      env.Load('robots/barrettwam.robot.xml')
      env.GetRobots()[0].SetDOFValues([])
  except openrave_exception, e:
      print e

Locking/Thread-safety Mechanisms
--------------------------------

When doing heavy operations, the environment should always be locked to keep other users from changing it. This is possible with '''Environment.LockPhysics(dolock)'''. Scoped locking can be implemented using a try/finally block or the python '''with''' statement:

.. code-block:: python

  env = Environment()
  # initialization code
  with env:
      # environment is now locked
      env.CheckCollision(...)

In the same spirit, the '''with''' statement used on boides/robots simultaneously locks the environment and preserves their state:

.. code-block:: python

  with robot:
      robot.SetTransform(newtrans)
      robot.SetActiveDOFs(...)
      # do work
  
  # robot now has its previous state restored

For those who want to optimize the locking of the environment every time, they can use the new `KinBodyStateSaver` and `RobotStateSaver` classes:

.. code-block:: python

  # enviroment should be locked at this point..
  with KinBodyStateSaver(body):
      # body state now preserved
  with RobotStateSaver(robot):
      # robot state now preserved
