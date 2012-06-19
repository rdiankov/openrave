.. _command_line_tools:

Command Line Tools
==================

.. _tool-openravepy:

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

.. _tool-openrave-robot:

openrave-robot.py
-----------------

Queries information about OpenRAVE-loadable robots. Allows to query simple information for robot links, joints, manipulators, sensor as fast as possible.

For example, getting info of all the manipulators is as simple as:

.. code-block:: bash

  openrave-robot.py robots/pr2-beta-static.zae --info manipulators

or can just get a list of the manipulator names:

.. code-block:: bash

  openrave-robot.py robots/pr2-beta-static.zae --list manipulators

Each robot can hold several different types of hashes depending on the information being queried. Hashes are retrieved with the **--hash** option:

.. code-block:: bash

  openrave-robot.py data/mug1.kinbody.xml --hash body
  openrave-robot.py robots/barrettsegway.robot.xml --hash robot
  openrave-robot.py robots/barrettsegway.robot.xml --manipname=arm --hash kinematics

Command-line
~~~~~~~~~~~~

.. shell-block:: openrave-robot.py --help

.. _tool-openrave-createplugin:

openrave-createplugin.py
------------------------

Sets up a project directory and initial files for creating OpenRAVE plugins and executables.

This command line will create a plugin that offers a **MyNewModule** :class:`.ModuleBase`:

.. code-block:: bash

  openrave-createplugin.py myplugin --module MyNewModule

Command-line
~~~~~~~~~~~~

.. shell-block:: openrave-createplugin.py --help

.. _tool-openrave:

openrave
--------

Simple executable written in C++ that can start an OpenRAVE environment and load modules. It provides simple configuration of parameters for easy testing. 

It is possible to save robots into 

Command-line
~~~~~~~~~~~~

.. shell-block:: openrave --help

.. _tool-openrave-config:

openrave-config
---------------

Used to find the openrave installation directories, used libraries, headers, and shared files.

Command-line
~~~~~~~~~~~~

.. shell-block:: openrave-config --help
