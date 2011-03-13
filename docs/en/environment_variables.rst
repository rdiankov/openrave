Environment Variables
=====================

.. envvar:: OPENRAVE_DATA

   Search paths/URLs for loading robot/environment/model files. The default data directory ``$OPENRAVE_INSTALL/share/openrave`` is always included, so there's no need to specify it again. Use ':' to separate each directory (';' for Windows).

.. envvar:: OPENRAVE_DATABASE

  Use ':' to separate each directory (';' for Windows). Search paths for loading database files created by the openrave database system. Databases are used to store useful information/statistics about the robot, target object, and sensors that take a long time to pre-compute or rely on real-world data. When writing, the first valid directory is used. If environment variable is not set, then $OPENRAVE_HOME is used.

.. envvar:: OPENRAVE_HOME

  Used to set the directory of the openrave local cache and log files. The default directory is <b>$HOME/.openrave</b>.

.. envvar:: OPENRAVE_PLUGINS

  Use ':' to separate each directory (';' for Windows). At startup, OpenRAVE searches for every shared object/dll plugin in these directories and loads them. The default plugins in ``$OPENRAVE_INSTALL/share/openrave/plugins`` are always loaded, so there is no need to include them.
