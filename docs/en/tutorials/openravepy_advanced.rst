Advanced openravepy
===================

Exceptions
----------

OpenRAVE C++ exceptions in the form of the openrave_exception class are automatically translated to a :class:`openrave_exception` class in python. An openrave exception can be caught by:

.. code-block:: python

  try:
      env = Environment()
      env.Load('robots/barrettwam.robot.xml')
      env.GetRobots()[0].SetDOFValues([])
  except openrave_exception, e:
      print e

Locking/Thread-safety Mechanisms
--------------------------------

When doing heavy operations, the environment should always be locked to keep other users from changing it. This is possible with **Environment.LockPhysics(dolock)**. Scoped locking can be implemented using a try/finally block or the python **with** statement:

.. code-block:: python

  env = Environment()
  # initialization code
  with env:
      # environment is now locked
      env.CheckCollision(...)

In the same spirit, the **with** statement used on boides/robots simultaneously locks the environment and preserves their state:

.. code-block:: python

  with robot:
      robot.SetTransform(newtrans)
      robot.SetActiveDOFs(...)
      # do work
  
  # robot now has its previous state restored

For those who want to optimize the locking of the environment every time, they can use the new `KinBodyStateSaver` and `RobotStateSaver` classes:

.. code-block:: python

  # enviroment should be locked at this point
  with KinBodyStateSaver(body):
      # body state now preserved
  with RobotStateSaver(robot):
      # robot state now preserved

Initialization
--------------

`RaveInitialize` initializes the OpenRAVE runtime and provides many options for configuring it. Options include what plugins to load at startup. If the runtime has not been initialized when creating :class:`Environment`, `RaveInitialize` is automatically called.

The following example shows how to start the runtime and load only one plugin:

.. code-block:: python

  try:
      RaveInitialize(load_all_plugins=False)
      success = RaveLoadPlugin('libbasemanipulation')
      # do work
  finally:
      RaveDestroy() # destroy the runtime

Destruction
-----------

Due to circular dependencies with the internal OpenRAVE resources, :class:`Environment` instances must be excplicitly destroyed using `Environment.Destroy`. In order to guarantee it is always called, users are recommended to use **try/finally**:

.. code-block:: python

  try:
      env=Environment()
      # do work
  finally:
      env.Destroy()

In addition, the OpenRAVE runtime managing plugin resources and environments has to be explicitly destroyed using `RaveDestroy` when users are shutting down the program; it destroys all environments and unloads all plugins:

.. code-block:: python

  try:
      env1=Environment()
      env2=Environment()
      RaveLoadPlugin('myplugin')
      # do work
  finally:
      RaveDestroy() # destroys all environments and loaded plugins
