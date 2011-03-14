Adding a Python Example
=======================

Adding an **example0** to the openravepy.examples module requires:

# Add example0.py to python/examples/ directory
# Add **import examples0** to python/exmaples/__init__.py
# Add to svn: using::

  svn add tutorial0.py

# Once installed, make sure it is executable through::

  openrave.py --example tutorial0

Images
------

All images should be prefixed with the example name. Like:

* example0_myimage.png
* example0_anotherimage.jpg

They should be put in **docs/images/examples/**. To reference an image use::

  .. image:: ../../images/example0/myimage.png
    :height: 200
