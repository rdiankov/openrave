.. _writing_plugins:

Writing OpenRAVE Programs
=========================

Plugins
-------

Creating OpenRAVE plugins allows others to use your work by just instantiating the interfaces through the **RaveCreateX** methods. Although it is highly recommended to start working with Python/Octave, eventually users should create plugins to offer their functionality through them. 

The simplest way to create a plugin is through the :ref:`tool-openrave-createplugin` program:

For example, the following command will create a plugin that offers **MyNewModule** :class:`.Module`:

.. code-block:: bash

  openrave-createplugin.py myplugin --module MyNewModule

This creates a **myplugin** directory where all the files are written in. In order to compile and test it do:

.. code-block:: bash

  cd myplugin
  make
  python testplugin.py

By default, the new plugin/executable is stored in the **build** folder. The following files are created to help users get started:

* **CMakeLists.txt** - used to create Makefiles and Visual Studio solutions

* **Makefile** - for Unix users, type 'make' to build the CMake project.

* **myplugin.cpp** - main C++ file

* **testplugin.py** - will load the plugin inside OpenRAVE and call its SendCommand method.

* **scenes/robots** - directories holding scene and robot files

For more details, please see

`Writing Plugins and Interfaces in C++ <../../coreapihtml/writing_plugins.html>`_

Programs
--------

It is also possible to create and use OpenRAVE inside a program by linking with the **openrave-core** library. The simplest way to create an example program is:

.. code-block:: bash

  openrave-createplugin.py myprogram --usecore


This will create a simple program that creates the OpenRAVE Environment and Loads a scene. The environment is created without attaching any viewer.
