.. _robots_overview:

Robots Overview
===============

File Formats
------------

Robots are standardized with the COLLADA 1.5 file format. OpenRAVE defines `robot extensions <http://openrave.programmingvision.com/index.php/Started:COLLADA>`_ for storing information like manipulators, sensors, and collision data. COLLADA files saved as **dae** store the raw XML, files stored as **zae** stored the compressed XML. In order to preserve space, all robots in OpenRAVE are stored as **zae**.

Because COLLADA can be a little difficult to edit by hand, OpenRAVE also defines a `Custom XML Format <http://openrave.programmingvision.com/index.php/Started:Formats>`_ to help users quickly get robots into the environment. It is possible to convert these custom robots into COLLADA using the following command:

.. code-block:: bash

   openrave -save myrobot.zae myrobot.xml

Standards for the coordinate system and scale of the robot: :ref:`geometric_conventions`.
