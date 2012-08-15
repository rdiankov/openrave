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

OpenRAVE supports the COLLADA file format for specifying robots and adds its own set of robot-specific extensions. The `COLLADA <https://collada.org/mediawiki/index.php/COLLADA_-_Digital_Asset_and_FX_Exchange_Schema>`_ format can be used to specify all robot and scene related information.

:ref:`collada_robot_extensions` - (\*.dae, \*.zae)

COLLADA files saved as **dae** store the raw XML, files stored as **zae** stored the compressed XML. In order to preserve space, most robots in OpenRAVE are stored as **zae**.

OpenRAVE XML
~~~~~~~~~~~~

`OpenRAVE XML <http://openrave.programmingvision.com/wiki/index.php/Format:XML>`_ (\*.xml)

Because COLLADA can be a little difficult to edit by hand, OpenRAVE also defines its own format to help users quickly get robots into the environment. It is possible to convert these custom robots into COLLADA using the following command:

.. code-block:: bash

   openrave -save myrobot.zae myrobot.xml
