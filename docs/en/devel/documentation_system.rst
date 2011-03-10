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
  ./makedocs.sh

Upload the generated documentation to the server with:

.. code-block:: bash

  cd docs
  ./sendtoserver.sh

Most build files are stored in the **docs/build** directory.
All images are saved into **docs/images**, including automatically generated ones.

Because running **makedocs.sh** can take a long time, developers should execute the individual commands pertaining to the part of the system they are maintaining. The commands are given in the following sections.

C++ Core Documentation (doxygen)
--------------------------------

Compiling HTML:

.. code-block::bash

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
 
Python (reStructuredText)
-------------------------

Compiling the python documentation:

.. code-block:: bash

  cd build/python
  make install
  cd ../../docs
  python build_doc.py build_doc --outdir="en/openravepy-html" --languagecode=en
  firefox en/openravepy-html/index.html

Adding a Tutorial
-----------------

Adding a tutorial0 to the python documentation:

# Add tutorial0.py to python/examples/ directory
# Add **import tutorial0** to python/exmaples/__init__.py
# Add to svn: using::

 svn add tutorial0.py
# Once installed, make sure it is executable through::

  openrave.py --example tutorial0

Images
~~~~~~

To add image to **docs/images/tutorial0_myimage.png**. Reference image by::

  .. image:: ../../images/tutorial0_myimage.png
    :height: 200

Custom Commands
~~~~~~~~~~~~~~~

Several special comments have been added to restructured text:

* **.. code-block:: [python, cpp, ...]** - used to get syntax highlighting
* **.. lang-block:: [en, ja, ...]** - used to restrict a block of text to a particular language
* **.. interface-command:: interface, command** - used to get the help text of a particular plugin interface
* **.. shell-block:: [program arguments]** - executes program and inserts the text literally
* **.. doxygenfunction:: [function]** - get commands for a doxygen function.

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

Generating Documentation for a Python File
------------------------------------------

In order to generate html documentation for an openrave demo documented in reStructuredText, first create a **epydoc.config** file with the following contents::

  [epydoc]
  modules: mydemo
  output: html
  inheritance: included
  frames: no
  private: no
  imports: no
  src_code_tab_width: 4
  redundant_details: no
  exclude: numpy, sympy, numeric
  docformat: restructuredtext

Where the code is stored in **mydemo.py**. In order generate html, run:

.. code-block:: bash

  cp `rospack find openrave`/openrave_svn/docs/epydoc.css .
  python `rospack find openrave`/openrave_svn/docs/build_doc.py build_doc --outdir="mydemofiles" --languagecode=en

Output is in **mydemofiles/index.html**.
