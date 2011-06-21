Adding a Python Example
=======================

This document is for developers that want to add an official openravepy example to the subversion repository.
Adding an **example0** to the openravepy.examples module requires:

* Add example0.py to ``python/examples/`` directory

* Add **import example0** to ``python/exmaples/__init__.py``

* Add to svn: using

.. code-block:: bash

  svn add example0.py

* Once installed, make sure it is executable through

.. code-block:: bash

  openrave.py --example example0

Documentation
-------------

Every example must follow this format when documenting it::

  """One sentence introduction.

  .. examplepre-block:: example0

  [user-specific text]

  .. examplepost-block:: example0
  """

**examplepre-block** and **examplepost-block** are special sphinx extensions that add the example images, code, and execution instructions.

Images
------

All images should be put in the ``docs/images/examples`` directory and prefixed with the example name. The image that represents the example has to be in JPEG format and have the same name as the example:

* ``docs/images/examples/example0.jpg``

This same image will be used as a thumb image on the examples gallery. It can be replaced with a different image by adding:

* ``docs/images/examples/example0_thumb.jpg``

Other images part of the example can be:

* ``docs/images/examples/example0_myimage.png``
* ``docs/images/examples/example0_anotherimage.jpg``

To reference an image use::

  .. image:: ../../images/examples/example0_myimage.png
    :width: 640
