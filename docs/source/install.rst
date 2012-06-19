.. _install:

Installing OpenRAVE
===================

:ref:`changelog`

`Roadmap <https://sourceforge.net/apps/trac/openrave/roadmap>`_ - timeline on new features and major changes.

:ref:`release_organization` - infrastructure on releases, versioning, and updates.

Platforms
---------

~~~~

.. image:: ../images/linux_ubuntu_logo.png
  :height: 96

`Official Release PPA <https://launchpad.net/~openrave/+archive/release>`_. Execute the following to add the OpenRAVE repository:

.. code-block:: bash
   
  sudo add-apt-repository ppa:openrave/release
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
