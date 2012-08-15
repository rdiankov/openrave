Documentation System
====================

.. image:: ../../images/openrave_documentation.png
  :width: 640

Describes the OpenRAVE documentation system and all necessary function calls for generating and maintaining the documents. Meant for developers.

Because documentation is uploaded to `http://www.openrave.org/docs/`_ , its HTML/CSS is managed by :ref:`openrave.org` system.

Installation
------------

.. code-block:: bash

  apt-get remove python-sphinx
  apt-get install python-pygments python-setuptools python-lxml python-matplotlib dvipng dia-common python-svn
  pip install --upgrade docutils sphinx sphinx-gettext-helper

`doxygen <http://www.stack.nl/~dimitri/doxygen/download.html#latestsrc>`_ version 1.8.2 or later is required.

`breathe <http://github.com/michaeljones/breathe>`_ is used for converting doxygen to python comments so that python users can be internal help for the C++ bindings. The breathe sources are in **docs/breathe**.

If using latex, install::

  apt-get install  dot2tex texlive-base texlive-latex-base texlive-pictures texlive-fonts-recommended

For japanese fonts install::

  apt-get install latex-cjk-japanese

Getting Started
---------------

OpenRAVE uses:

* doxygen for the core system and c++ documentation
* reStructuredText for all other documentation.

Go into **docs** directory and execute following script to generate all documentation for all the English language:

.. code-block:: bash

  LANG=en_US.UTF-8 make html_en

If c++ header files documentation changed, then have to call with a rebuild to openravepy docs:

.. code-block:: bash

  LANG=en_US.UTF-8 make openravepy_changed build/en/coreapixml/index.xml
  LANG=ja_JP.UTF-8 make openravepy_changed build/ja/coreapixml/index.xml
  make openravepy_internal_doc

Create the packages to be sent to the doc server:

.. code-block:: bash

   cd docs
   LANG=en_US.UTF-8 make openravejsonzip

Most build files are stored in the **docs/build** directory.
All images are saved into **docs/images**, including automatically generated ones.

Because generation can take a long time, developers should execute the individual commands pertaining to the part of the system they are maintaining. The commands are given in the following sections.

Python (reStructuredText)
-------------------------

Compiling the python documentation is divided into several steps.

# Take in the doxygen comments from the C++ API and creates ``python/bindings/docstrings.cpp``, which is then compiled into the **openravepy_int** module.

# Generate rst files from all the openravepy modules using ``sphinx-autopackage-script/generate_modules.py``.

# Generate rst files from the interfaces documentation and testing results.

# Compile with ``sphinx-build``

In order to generate files managed by openrave.org, it has to be called with:

.. code-block:: bash

    LANG=en_US.UTF-8 make json_en

Because the documentation for openravepy is built from the install directory, whenever a change to the openravepy documentation made openrave has to be reinstalled with ``make install``.

C++ Core Documentation (doxygen)
--------------------------------

Compiling HTML and Latex:

.. code-block:: bash

  cd docs
  LANG=en_US.UTF-8 make doxygenhtml_installed_en
  firefox build/en/coreapihtml/index.html
  evince build/en/coreapilatex/refman.pdf

The script internally makes these calls:

.. code-block:: bash

  cd docs
  doxygen build/Doxyfile.html.en
  doxygen build/Doxyfile.latex.en

The **build/Doxyfile.html.en** file is generated from **Doxyfile.html** and **Doxyfile.en**. The separation is necessary in order to provide better localization support for multiple languages.

The mainpage and bulk of the documentation is in **docs/mainpage.dox**. Installation instructions are in **docs/install.dox**.

Use the **\en**, **\ja**, **\~** tags to switch between language modes.

To reference image in **docs/images/tutorial0_myimage.png**, write::

  \image html tutorial0_myimage.png
  \image latex tutorial0_myimage.png "My Caption" width=15cm

Interfaces
----------

To build the webpage of interface descriptions, run

.. code-block:: bash

  LANG=en_US.UTF-8 make source/interface_types

This outputs a set of reStructuredText files, which can be used by sphinx to build up the page.

Robots Database
---------------

Generate a set of webpages for each robot using the statistics file output from **test/test_ikfast.py**.

An image of all the robots in openrave can be extracted using the **build_ikdatabase.py** script:

.. code-block:: bash

  cd docs
  LANG=en_US.UTF-8 make ikfaststats=ikfaststats.pp ikfast

Internationalization
--------------------

Use gettext internationalization to store separate translation files for each language `Sphinx Internationalization <http://sphinx.pocoo.org/latest/intl.html>`_, `Japanese Tutorial <http://d.hatena.ne.jp/tk0miya/20111203>`_.

The translation PO files are stored in `https://openrave.svn.sourceforge.net/svnroot/openrave/trunk/docs/locale`_. Anyone is welcome to send diff files of translations.

In order to test how the translation looks for the Japanese language, execute

.. code-block:: bash

  LANG=ja_JP.UTF-8 make html_ja

Note that both **LANG** and **language** have to be set.
