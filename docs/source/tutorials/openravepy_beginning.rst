.. _openravepy_beginning:

Beginning with openravepy
=========================

Getting Started
---------------

:ref:`package-openravepy` allows Python to use the C++ API seamlessly. The bindings are developed using the
`Boost.Python <http://www.boost.org/doc/libs/release/libs/python/doc>`_ library. Because
openravepy directly links with OpenRAVE instead of going through the network, it allows much
faster execution times in a much more natural setting. In fact, most of the python bindings match
the exactly C++ header files exactly.

The main components are:

* :mod:`openravepy_int` - provides C++ internal bindinings, is generated using Boost Python.
* :mod:`openravepy_ext` - provides useful functions/classes to be used by rest of classes

There are 3 major subcomponents:

* :ref:`package-databases` - database generators
* :ref:`package-examples` - runnable examples
* :ref:`package-interfaces` - bindings to interfaces offered through OpenRAVE plugins.

**openravepy** can be found in ``C:\\Program Files\\openrave\\share\\openrave`` on Windows. For unix-based systems, the follow command can be used to retrieve the path:

.. code-block:: bash

  openrave-config --python-dir

When directly importing openravepy, this path needs to be added into the **PYTHONPATH** environment variable. For unix-based systems, it would look like:

.. code-block:: bash

  export PYTHONPATH=$PYTHONPATH:`openrave-config --python-dir`

All the examples are stored in the ``openravepy/examples`` directory. For example, the simplest
planning example hanoi can be found in ``openravepy/examples/hanoi.py`` and executed by:

.. code-block:: bash

  openrave.py --example hanoi

OpenRAVE should automatically pop up and the puma arm will attempt to grasp the pegs on the table.


The docstrings for each function and class are automatically compiled from the C++ documentation. In the python interpreter, simply type:

.. code-block:: python

  help env.CloneSelf # env is an instance of Environment()
  help KinBoby.GetChain # KinBody is a class
  help Robot.Manipulator.FindIKSolution # Robot.Manipulator is a sub-class

Exceptions
----------

OpenRAVE C++ exceptions in the form of the openrave_exception class are automatically translated to a :class:`.openrave_exception` class in python. An openrave exception can be caught by:

.. code-block:: python

  try:
      env = Environment()
      env.Load('robots/barrettwam.robot.xml')
      env.GetRobots()[0].SetDOFValues([])
  except openrave_exception, e:
      print e

Locking/Thread-safety Mechanisms
--------------------------------

When doing heavy operations, the environment should **always be locked** to keep other users from changing it. All environment methods are **multi-thread safe**, but any other method to kinbodies, robots, controllers, planners, etc are **not thread safe**! If the documentation does not say multi-thread safe, **don't use the method without locking the environment**!


Locking is done with **Environment.Lock(dolock)**. Scoped locking can be implemented using a
try/finally block or the python **with** statement:

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

For those who want to reduce the number of environment locks, they can use the new :class:`.KinBodyStateSaver` and :class:`.RobotStateSaver` classes:

.. code-block:: python

  with env:
      # enviroment locked at this point
      with KinBodyStateSaver(body):
          # body state now preserved
      with RobotStateSaver(robot):
          # robot state now preserved

Initialization
--------------

:func:`.RaveInitialize` initializes the OpenRAVE runtime and provides many options for configuring it. Options include what plugins to load at startup. If the runtime has not been initialized when creating :class:`.Environment`, :func:`.RaveInitialize` is automatically called.

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

Due to circular dependencies with the internal OpenRAVE resources, :class:`.Environment` instances must be excplicitly destroyed using `Environment.Destroy`. In order to guarantee it is always called, users are recommended to use **try/finally**:

.. code-block:: python

  try:
      env=Environment()
      # do work
  finally:
      env.Destroy()

In addition, the OpenRAVE runtime managing plugin resources and environments has to be explicitly destroyed using :func:`.RaveDestroy` when users are shutting down the program; it destroys all environments and unloads all plugins:

.. code-block:: python

  try:
      env1=Environment()
      env2=Environment()
      RaveLoadPlugin('myplugin')
      # do work
  finally:
      RaveDestroy() # destroys all environments and loaded plugins

Loading Different Versions
--------------------------

If multiple openrave versions are installed, then it is possible to select which version of
openravepy by setting the **__openravepy_version__** variable to the desired version before
importing anything. For example:

.. code-block:: python

  __builtins__.__openravepy_version__ = '0.4'
  import openravepy

Logging
-------

It is possible to set the logging levels of internal OpenRAVE using the DebugLevel enum:

.. code-block:: python
  
  RaveSetDebugLevel(DebugLevel.Verbose)

A lot of the OpenRAVE Python bindings use the python logging module directly. In order to initialize it with the correct output handles and have it sync with the internal OpenRAVE logging level, use the following command:

.. code-block:: python

  from openravepy.misc import InitOpenRAVELogging
  InitOpenRAVELogging()
