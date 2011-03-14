.. _command_line_tools:

Command Line Tools
==================

openrave.py
-----------

The openrave.py script is an attempt to make the command line parameters for openrave much simpler
to use. It is a superset of the functions provided by the original *openrave* program, except it
supports many other interesting features and provides a window to all openravepy functions. It can
also automatically add openravepy to the PYTHONPATH making it simpler for users. Here are some
features it supports:

* Opening files with the **-i** option will now drop into the ipython interpreter after loading the particular files specified.  For example::

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

* Can start a database generation process::

    openrave.py --database inversekinematics --robot=robots/pa10.robot.xml

* Can execute an example::

    openrave.py --example graspplanning

* Can query all executable databases::

    openrave.py --listdatabases

  **Output**:

.. shell-block:: openrave.py --listdatabases

* Can set custom collision, physics, and viewer::

    openrave.py --collision=pqp --viewer=qtcoin --physics=ode data/lab1.env.xml

* Can set debug mode::

    openrave.py --level=verbose data/lab1.env.xml

* Can execute arbitrary python code::

   openrave.py -p "print 'robot manipulators: ',robot.GetManipulators()" robots/pr2-beta-sim.robot.xml

* Can execute arbitrary python code and step into the ipython interpreter::

   openrave.py -p "manip=robot.GetActiveManipulator()" -i robots/pr2-beta-sim.robot.xml

* Can execute arbitrary python code and exit::

   openrave.py -p "print('links: '+str(robot.GetLinks())); sys.exit(0)" robots/pr2-beta-sim.robot.xml


Given that environment xml files can now contain tags for any interface, it is possible to setup all the used interfaces in the XML, open it with **openrave.py -i**, and immediately start introspecting on the state.

Command-line
~~~~~~~~~~~~

.. shell-block:: openrave.py --help

openrave-hash.py
----------------

Can query all the hashes openrave uses to manage robot descriptions::

  openrave-hash.py data/mug1.kinbody.xml
  openrave.py --robothash robots/barrettsegway.robot.xml

Command-line
~~~~~~~~~~~~

.. shell-block:: openrave-hash.py --help

openrave
--------

Simple executable written in C++ that can start an OpenRAVE environment and load problems. It provides simple configuration of parameters for easy testing. 

It is possible to save robots into 

Command-line
~~~~~~~~~~~~

.. shell-block:: openrave --help
