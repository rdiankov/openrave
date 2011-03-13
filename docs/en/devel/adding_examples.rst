Adding a Python Example
=======================

Adding an example0 to the openravepy.examples module requires:

# Add example0.py to python/examples/ directory
# Add **import examples0** to python/exmaples/__init__.py
# Add to svn: using::

  svn add tutorial0.py

# Once installed, make sure it is executable through::

  openrave.py --example tutorial0

Images
------

To add image to **docs/images/example0/myimage.png**. Reference image by::

  .. image:: ../../images/example0/myimage.png
    :height: 200
