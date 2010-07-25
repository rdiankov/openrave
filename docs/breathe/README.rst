Breathe
=======

This is an extension to restructured text and Sphinx to be able to read and
render the Doxygen xml output.

Status
------

This has passed proof of concept stage and has proved itself to be a useful
tool. It is usable, but will generate a couple of warnings from time to time.

For example usage, please see the `documentation
<http://michaeljones.github.com/breathe>`_.

There are no unittests.

Running Testsuite
-----------------

This process assumes the ``sphinx-build`` script is on your path. If it is not,
then either edit ``testsuite/Makefile`` to point at it or create a link to it in
the ``testsuite`` directory.

First run ``make`` for each folder in the examples directory, then::

   cd testsuite
   make html

Then view the resulting html with your browser of choice.


Credits
-------

Thank you to:

- `nijel <http://github.com/nijel>`_
- `sebastianschaetz <http://github.com/sebastianschaetz>`_

For their contributions; improving the code and the documentation. And thanks to:

- Dimitri van Heesch for `Doxygen <http://www.stack.nl/~dimitri/doxygen/>`_.
- Georg Brandl for `Sphinx <http://sphinx.pocoo.org>`_.
- David Goodger for `Docutils <http://docutils.sourceforge.net/>`_ and Restructured Text. 


