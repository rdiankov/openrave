# -*- coding: utf-8 -*-
"""
================
openravepy usage
================

.. contents::

Introduction
------------

.. lang-block:: ja

  はじめに

.. lang-block:: en

  OpenRAVE_ can be used with Python seamlessly through the `openravepy` module. The bindings with the
  C++ API are developed using the Boost.Python library. Because `openravepy` directly links with
  OpenRAVE_ instead of going through the network, it allows much faster execution times in a much more
  natural setting. In fact, most of the python bindings match the exactly C++ header files exactly.

.. lang-block:: ja

  OpenRAVE_ の機能はopenravepyモジュールによってPythonで使える．バインディングはネットワーク通信やシリアライズのではなく，メモリのやりとりで行われているため，非常に早いである．このおかげで，PythonのバインディングはC＋＋のAPIに一致するようになっている．

Setup
=====

`openravepy` can be found in ``${OPENRAVE_INSTALL_DIR}/share/openrave`` on unix-based systems and ``C:\\Program Files\\openrave\\share\\openrave`` on Windows. For unix-based systems, the follow command can be used to retrieve the path::

  openrave-config --python-dir

This path needs to be added into the **PYTHONPATH** environment variable. For unix-based systems, it would look like::

  export PYTHONPATH=$PYTHONPATH:`openrave-config --python-dir`

All the examples are stored in the ``openravepy/examples`` directory. For example, the simplest
planning example hanoi can be found in ``openravepy/examples/hanoi.py`` and executed by::

  python hanoi.py

OpenRAVE should automatically pop up and the puma arm will attempt to grasp the pegs on the table.

Quick Examples
--------------

Simple Environment Loading
==========================

Here is a small example that loads up an environment, attaches a viewer, loads a scene, and requests
various information about the robot

.. code-block:: python

  from openravepy import *
  env = Environment() # create openrave environment
  env.SetViewer('qtcoin') # attach viewer (optional)
  env.Load('data/lab1.env.xml') # load a simple scene
  robot = env.GetRobots()[0] # get the first robot
  print "Robot ",robot.GetName()," has ",robot.GetDOF()," joints with values:\\n",robot.GetJointValues()
  robot.SetJointValues([0.5],[0]) # set joint 0 to value 0.5
  T = robot.GetLinks()[1].GetTransform() # get the transform of link 1
  print "The transformation of link 1 is:\\n",T
  env.Destroy() # explicitly destroy the environment once done with it


Using a BiRRT Planner
=====================

.. code-block:: python

  from openravepy import *
  env = Environment() # create the environment
  env.Load('data/lab1.env.xml') # load a scene
  env.SetViewer('qtcoin') # start the viewer
  robot = env.GetRobots()[0] # get the first robot
  manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
  res = manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19]) # call motion planner with goal joint angles
  robot.WaitForController(0) # wait
  env.Destroy()

Logging
=======

Here is an example on how to save the current scene using the '''logging''' plugin:

.. code-block:: python

  from openravepy import *
  env = Environment() # create openrave environment
  env.Load('data/lab1.env.xml') # load a simple scene
  logger = env.CreateProblem('logging')
  logger.SendCommand('savescene filename myscene.env.xml')
  env.Destroy() # explicitly destroy the environment once done with it

Usage
-----

Exceptions
==========

OpenRAVE C++ exceptions in the form of the openrave_exception class are automatically translated to a '''openrave_exception''' class in python. An openrave exception can be caught by:

.. code-block:: python

  try:
      env = Environment()
      env.Load('robots/barrettwam.robot.xml')
      env.GetRobots()[0].SetJointValues([])
  except openrave_exception, e:
      print e

Locking/Thread-safety Mechanisms
================================

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

openrave.py
-----------

The openrave.py script is an attempt to make the command line parameters for openrave much simpler to use. It is functionally equivalent to the original openrave program, except it is consistent and supports many other interesting features:

- Opening files with the **-i** option will now drop into the ipython interpreter after loading the particular files specified.  For example::

    openrave.py -i data/lab1.env.xml

  **Output**::

    [openravepy_int.cpp:2679] viewer qtcoin successfully attached
    OpenRAVE Dropping into IPython
    In [1]:

  The first robot in the scene is automatically loaded into the 'robot' variable, so it can be immediately used for scripting operations:

  .. code-block:: python

    In [1]: robot.GetJoints()
    Out[1]:
    [<env.GetKinBody('BarrettWAM').GetJoint('Shoulder_Yaw')>,
     <env.GetKinBody('BarrettWAM').GetJoint('Shoulder_Pitch')>, ...]

- Can start a database generation process::

    openrave.py --database="inversekinematics --robot=robots/pa10.robot.xml"

- Can execute an example::

    openrave.py --example="graspplanning"

- Can query all executable databases::

    openrave.py --listdatabases

  **Output**::

     convexdecomposition
     grasping
     inversekinematics
     inversereachability
     kinematicreachability
     linkstatistics
     visibilitymodel

- Can set custom collision, physics, and viewer::

    openrave.py --collision=pqp --viewer=qtcoin --physics=ode data/lab1.env.xml

- Can query all the hashes openrave uses to manage robot descriptions::

    openrave.py --bodyhash=data/mug1.kinbody.xml
    openrave.py --robothash=robots/barrettsegway.robot.xml

- Can set debug mode::

    openrave.py --debug=verbose data/lab1.env.xml

Given that environment xml files can now contain tags for any interface, it is possible to setup all the used interfaces in the XML, open it with **openrave.py -i**, and immediately start introspecting on the state.

.. _OpenRAVE: http://openrave.programmingvision.com

"""
try:
    __build_doc__ = __openravepy_build_doc__
    if __build_doc__:
        print 'openravepy imported in documentation mode'
except NameError:
    __build_doc__ = None

from openravepy_int import *
from openravepy_int import _openrave_exception_
from openravepy_int import __version__
from openravepy_int import __author__
from openravepy_int import __copyright__
__license__ = 'core: Lesser GPL, examples: Apache License, Version 2.0'
__docformat__ = 'restructuredtext ja'

from openravepy_ext import *

import metaclass
try:
    import ikfast
except ImportError, e:
    print 'openravepy: Failed to import ikfast: ',e
import pyANN
import convexdecompositionpy
import interfaces
import databases
import examples

pyANN._pyann_exception_.py_err_class = pyann_exception
_openrave_exception_.py_err_class = openrave_exception

# would "from openravepy import *" be slower if this is enabled?
#__all__ = ["examples", "ikfast", "interfaces", "metaclass", "pyANN"]
