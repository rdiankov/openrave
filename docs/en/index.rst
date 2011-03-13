Welcome to Open Robotics Automation Virtual Environment
=======================================================

.. htmlonly::

   :Release: |version|
   :Date: |today|

  OpenRAVE provides an environment for testing, developing, and deploying motion planning algorithms in real-world robotics applications. The main focus is on simulation and analysis of kinematic and geometric information related to motion planning. OpenRAVE's stand-alone nature allows is to be easily integrated into existing robotics systems. An important target application is industrial robotics automation. 

Getting Started
---------------

* `Install from sources <http://openrave.programmingvision.com/ordocs/en/html/installation.html>`_

 * See `ChangeLog`_ for what's new.

 * Latest source code from Subversion
::

  svn co https://openrave.svn.sourceforge.net/svnroot/openrave/trunk openrave

* :ref:`getting_started`

* :ref:`overview`

Resources
---------

This document is automatically generated for every OpenRAVE release and binds together the following versioned resources:

* `Core C++ API`_
* :ref:`Python API <package-openravepy>`
* :ref:`plugin_interfaces`
* `Database Generators <databases.html>`_
* `Examples <examples.html>`_
* :ref:`robots`

The `official wiki`_ describes latest news, projects using OpenRAVE, and how to link OpenRAVE with other systems.

Licenses
--------

The core C++ API is licenced under the `Lesser GPL <http://www.gnu.org/licenses/lgpl.html>`_, which makes it possible for commercial use but allows the OpenRAVE developers to guarantee a consistent API. Most of the examples and scripts outside the core are licensed under `Apache License, Version 2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>`_, which is much less restrictive (similar to BSD). Plugins can be released in any license the plugin authors choose to.

Please support OpenRAVE development by referencing it in your works/publications/projects with::

  @phdthesis{diankov_thesis,
   author = "Rosen Diankov",
   title = "Automated Construction of Robotic Manipulation Programs",
   school = "Carnegie Mellon University, Robotics Institute",
   month = "August",
   year = "2010",
   number= "CMU-RI-TR-10-29",
   url={http://www.programmingvision.com/rosen_diankov_thesis.pdf},
  }
