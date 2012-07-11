.. _robots_overview:

Robots Overview
===============

* :ref:`geometric_conventions` - Standards for the coordinate system and scale of the robot.

.. _robots_repositories:

Robot Repositories
------------------

* Subversion: https://openrave.svn.sourceforge.net/svnroot/openrave/trunk/src/robots

* Subversion: https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots

File Formats
------------

:ref:`collada_robot_extensions` - (\*.dae, \*.zae)

OpenRAVE XML
~~~~~~~~~~~~

`OpenRAVE XML <http://openrave.programmingvision.com/wiki/index.php/Format:XML>`_ (\*.xml)

Because COLLADA can be a little difficult to edit by hand, OpenRAVE also defines its own format to help users quickly get robots into the environment. It is possible to convert these custom robots into COLLADA using the following command:

.. code-block:: bash

   openrave -save myrobot.zae myrobot.xml
