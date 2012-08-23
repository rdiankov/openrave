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

OpenRAVE supports the COLLADA file format for specifying robots and adds its own set of robot-specific extensions. The `COLLADA <https://collada.org/mediawiki/index.php/COLLADA_-_Digital_Asset_and_FX_Exchange_Schema>`_ format can be used to specify all robot and scene related information.

:ref:`collada_robot_extensions` - (\*.dae, \*.zae)

COLLADA files saved as **dae** store the raw XML, files stored as **zae** stored the compressed XML. In order to preserve space, most robots in OpenRAVE are stored as **zae**.

The following attributes can be passed to the :class:`.Environment` Load/Read methods:

* **skipgeometry="true"/"false"** - whether to skip geometry or not
* **scalegeometry="10 10 10"** - scale all geometry by this amount
* **prefix="newname_"** - add prefix to all links/joints/sensors/etc
* **openravescheme="x1 x2"** - external references relative to $OPENRAVE_DATA paths are only specified with **x1:/** or **x2:/**. If there is an authority, use **x1://authority**. The multiple schemes are all alias for the OpenRAVE database.
* **uripassword="URI password"** - adds an entry for a URI/password key-value pair to be used if the archive is encrypted

The following attributes can be passed to the :class:`.Environment` Save/Write methods:

* **externalref="bodyname1 bodyname2"** - if writing collada, specify the names that should be exported via external references. If **\***, then export all possible values via external references.

* **openravescheme="customscheme"** - if specified, writer will attempt to convert a local system URI (**file:/**) to a a relative path with respect to $OPENRAVE_DATA paths and use **customscheme** as the scheme.

* **unit="1.0"**  - how many real-world meters in one distance unit. For example unit="0.001" represents millimeters.

* **password="????"**
Any attributes are also set through the collada-dom DAE::getIOPlugin::setOption.

OpenRAVE XML
~~~~~~~~~~~~

`OpenRAVE XML <http://openrave.programmingvision.com/wiki/index.php/Format:XML>`_ (\*.xml)

Because COLLADA can be a little difficult to edit by hand, OpenRAVE also defines its own format to help users quickly get robots into the environment. It is possible to convert these custom robots into COLLADA using the following command:

.. code-block:: bash

   openrave -save myrobot.zae myrobot.xml
