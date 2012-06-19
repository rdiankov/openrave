.. _overview:

Overview
========

OpenRAVE includes a seamless integration of simulation, visualization, planning, scripting and
control. The plugin architecture allows users to easily write custom controllers or extend
functionality. Using OpenRAVE plugins, any planning algorithm, robot control, or sensing-based
subsystem can be distributed and dynamically loaded at run-time; this distributed nature frees
developers from struggling with monolithic code-bases. Users of OpenRAVE can concentrate on the
development of planning and scripting aspects of a problem without having to explicitly manage the
details of robot kinematics and dynamics, collision detection, world updates, and robot
control. OpenRAVE provides a powerful :ref:`Python API <package-openravepy>` for scripting demos, which makes it simple to
control and monitor the demo and environment state. There are also interfaces for Octave and Matlab.

OpenRAVE's major design goals and features are:

* Have a plugin-based architecture that allows users to expand its functionality without having to recompile the base code. Most functionality should be offered as plugins, thus keeping the core as simple as possible.

* Offer many motion planning algorithm implementations that can be easily extended to new tasks.

* Make it easy to debug components during run-time without having to recompile or restart the entire system in order to prevent flushing of the in-memory environment state.

* Allow the OpenRAVE core to be used as a simulation environment, as a high-level scripting environment, as a kinematics and dynamics backend module for robot con- trollers, or as a manipulation planning black box in a distributed robotics environment.

* Allow simple planning knowledgebases to be generated, stored, and retried.

* Support a multi-threaded environment and allow easy parallelization of planners and other functions with minimal synchronization required on the user side.

One of OpenRAVE’s strongest points when compared with other planning packages is the idea of being
able to apply algorithms to any scenario with very little modification when robots or target objects
change.  Users of OpenRAVE can concentrate on the development of planning and scripting aspects of a
problem without having to explicitly manage the details of robot kinematics, dynamics, collision
detection, world updates, sensor modeling, and robot control.

OpenRAVE has been used for planning on many real robotics systems. It's architcture makes it
possible for planning-enabled robots to work consistently in a continuously changing and
unpredictable environment. Many new layer of functionality have been developed that go beyond the
basic kinematics, collision detection, and graphics interface requirements of classic robotics
libraries. It provides a set of interfaces that let users modify existing functions and expand
OpenRAVE-enabled modules without having to recompile OpenRAVE or deal with messy monolithic
code-bases.

* :ref:`package-databases`

 * :ref:`ikfast_compiler`

.. toctree::
   :maxdepth: 1

   architecture/index
   robots_overview
   geometric_conventions

History
-------

OpenRAVE was founded by `Rosen Diankov <http://www.programmingvision.com>`_ at the `Quality of Life
Technology Center <http://www.cmu.edu/qolt>`_ in the `Carnegie Mellon University Robotics Institute
<http://www.ri.cmu.edu/>`_. It was inspired from the RAVE simulator `James Kuffner
<http://www.kuffner.org/james/>`_ had started developing in 1995 and used for his experiments ever
since. The OpenRAVE project was started in 2006 and is a complete rewrite of RAVE. It is actively
being maintained at the `JSK Lab at University of Tokyo <http://www.jsk.t.u-tokyo.ac.jp/>`_.

Developers/Contributors
-----------------------

.. include:: ../../AUTHORS
