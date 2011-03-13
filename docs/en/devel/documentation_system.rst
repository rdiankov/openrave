Documentation System
====================

Describes the OpenRAVE documentation system and all necessary function calls for generating and maintaining the documents. This is mean for developers.

Installation
------------

.. code-block:: bash

  apt-get install python-pygments python-setuptools python-lxml python-matplotlib dvipng
  easy_install --upgrade docutils
  easy_install sphinx

`doxygen <http://www.stack.nl/~dimitri/doxygen/download.html#latestsrc>`_ version 1.7.1 or later is required.

`breath <http://github.com/michaeljones/breathe>`_ is used for converting doxygen to python comments so that python users can be internal help for the C++ bindings. The breathe sources are in **docs/breathe**.

If using latex, install::

 dot2tex, texlive-base, texlive-latex-base, texlive-pictures, texlive-fonts-recommended

For japanese fonts install::

  latex-cjk-japanese

Getting Started
---------------

OpenRAVE uses:

* doxygen for the core system and c++ documentation
* reStructuredText for all other documentation.

The following script generates all documentation for English and Japanese:

.. code-block:: bash

  cd docs
  ./build.sh

Upload the generated documentation to the server with:

.. code-block:: bash

  cd docs
  ./sendtoserver.sh

Most build files are stored in the **docs/build** directory.
All images are saved into **docs/images**, including automatically generated ones.

Because running **build.sh** can take a long time, developers should execute the individual commands pertaining to the part of the system they are maintaining. The commands are given in the following sections.

Python (reStructuredText)
-------------------------

Compiling the python documentation is divided into several steps.

# Take in the doxygen comments from the C++ API and creates ``python/bindings/docstrings.cpp``, which is then compiled in the **openravepy_int** module.

# Generate rst files from all the openravepy modules using ``sphinx-autopackage-script/generate_modules.py``.

# Generate rst files from the interfaces documentation and testing results.

# Compile with ``sphinx-build``

The entire script is:

.. literalinclude:: ../../build_sphinx.sh
  :language: bash

Because the documentation for openravepy is built from the install directory, whenever a change to the openravepy documentation made openrave has to be reinstalled with ``make install``.

C++ Core Documentation (doxygen)
--------------------------------

Compiling HTML:

.. code-block:: bash

  cd docs
  doxygen Doxygen.html.en
  firefox en/html/index.html

Compiling Latex:

.. code-block:: bash

  cd docs
  doxygen Doxygen.latex.en
  ./build_latex.py en/latex
  evince en/latex/refman.pdf

The mainpage and bulk of the documentation is in **docs/mainpage.dox**. Installation instructions are in **docs/install.dox**.

Use the **\en**, **\ja**, **\~** tags to switch between language modes.

To reference image in **docs/images/tutorial0_myimage.png**, write::

  \image html tutorial0_myimage.png
  \image latex tutorial0_myimage.png "My Caption" width=15cm


Interfaces
----------

To build the webpage of interface descriptions, run

.. code-block:: bash

  python build_interfaces.py

This outputs a set of reStructuredText files, which can be used by sphinx to build up the page.

Robots
------

Generate a set of webpages for each robot using the statistics file output from **test/test_ikfast.py**.

An image of all the robots in openrave can be extracted using the **build_robots.py** script.
