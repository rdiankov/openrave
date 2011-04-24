.. _install:

Installing OpenRAVE
===================

:ref:`release_organization` - detailed information about releases, versioning, and updates.

:ref:`changelog`

Platforms
---------

~~~~

.. image:: ../images/linux_ubuntu_logo.png
  :height: 96

`Official Release PPA <https://launchpad.net/~openrave/+archive/release>`_. Execute the following to add the OpenRAVE repository:

.. code-block:: bash
   
  sudo add-apt-repository ppa:openrave/releasae
  sudo apt-get update
  sudo apt-get install openrave

~~~~

.. image:: ../images/windows_logo.png
  :height: 96

`Windows Installers <http://sourceforge.net/projects/openrave/files/latest_stable>`_ are compiled for every Visual Studio version.

~~~~

Compile From Sources
--------------------

Get latest stable source code (constantly updated):

.. code-block:: bash

  svn co https://openrave.svn.sourceforge.net/svnroot/openrave/tags/latest_stable openrave

`Compile/Install From Sources`_
