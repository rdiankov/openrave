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

COLLADA
~~~~~~~

OpenRAVE fully supports the COLLADA file format for specifying robots and adds its own set of robot-specific extensions. The robot extensions include:

    * manipulators
    * sensors
    * planning-specific parameters

`COLLADA Robot Extensions`_ (*.dae, *.zae)

Robots are standardized with the COLLADA 1.5 file format. OpenRAVE defines these for storing information like manipulators, sensors, and collision data. COLLADA files saved as **dae** store the raw XML, files stored as **zae** stored the compressed XML. In order to preserve space, all robots in OpenRAVE are stored as **zae**.

OpenRAVE XML
~~~~~~~~~~~~

`OpenRAVE XML`_ (\*.xml)

Because COLLADA can be a little difficult to edit by hand, OpenRAVE also defines its own format to help users quickly get robots into the environment. It is possible to convert these custom robots into COLLADA using the following command:

.. code-block:: bash

   openrave -save myrobot.zae myrobot.xml
