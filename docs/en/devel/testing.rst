.. _testing_framework:

OpenRAVE Testing Framework
==========================

Using `python nose <http://somethingaboutorange.com/mrl/projects/nose>`_ for the testing framework.

`OpenRAVE testing server <http://www.openrave.org/testing>`_ uses Jenkins and runs these tests, records statistics, and generates this documentation. The tests are run on **trunk**, and if deemed stable, the **stable** subversion tag is updated.

Configuring
-----------

Because it is not necessary to cater to the lowest common denominator, use easy_install to get latest test packages::

  sudo apt-get install graphviz python-svn gnu-pgagent
  sudo easy_install nose coverage

Jenkins
~~~~~~~

The following Jenkins plugins should be installed:
	
* CMake plugin
* Measurement Plots
* Performance Publisher plugin
* Post build task
* Publish Over SSH
* Python Plugin
* SSH Slaves plugin
* Subversion Plug-in
* Subversion Release Manager plugin
* Subversion Tagging Plugin
* Edgewall Trac plugin
* Jenkins xUnit plugin
* Downstream buildview plugin
* Global Build Stats Plugin
* Dependency Graph View Plugin
* Join Plugin
* Release Plugin
* Parameterized Trigger Plugin

The tests are designed to output to Jenkins-friendly format using the python nose and `Jenkins xUnit <http://wiki.jenkins-ci.org/display/JENKINS/xUnit+Plugin>`_ plugins.

Measurements and their history are recorded with the `Jenkins Measurement Plots Plugins <http://wiki.hudson-ci.org/display/HUDSON/Measurement+Plots+Plugin>`_. This is done by outputting the following xml on stdout:

.. code-block:: xml

  <measurement><name>myvaluename</name><value>14.669</value></measurement>

chroot
~~~~~~

Can build entire ubuntu environments using `debootstrap <https://wiki.ubuntu.com/DebootstrapChroot>`_.

Tests
-----

Running 'make test' in the root directory should setup the OpenRAVE environment from local **build** folder and run all tests. If the PARALLEL_JOBS environment variable is set to "-jX", then will distribute the computation onto X processes.

Basic Tests
~~~~~~~~~~~

All these tests must pass for a release candidate:

* single precision, double precision
* compile wihtout python bindings, without octave bindings, without plugins, without collada
* boost versions: 1.35, 1.39, 1.44
* cmake versions: 2.6, 2.8
* 32bit vs 64bit
* Visual Studio 2005 (sp1), 2008, and 2010.
* All exapmles/databases run without throwing exceptions

IKFast
~~~~~~

Once building is successful, IKFast tests are run for every robot in its database using the **test/test_ikfast.py** script. The `statistics  <http://www.openrave.org/testing/job/openrave/>`_ are used to generate reStructuredText for the :ref:`robots` page. :ref:`ikfast-testing` goes into details about what is tested.
