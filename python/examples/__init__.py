#!/usr/bin/env python
"""Examples showing common planning and usage scenarios.

Unlike the tutorials, python examples are installed with openravepy and their list can be retrieved with:

.. code-block:: bash

  openrave.py --listexamples

Execute example ``X`` using:

.. code-block:: bash

  openrave.py --example X

Each sub-module contains a 'run' function that can be called directly with options to configure the example.
"""

# tutorials showing a single functionality (simple)
from . import tutorial_grasptransform
from . import tutorial_ik5d
from . import tutorial_iklookat
from . import tutorial_iklookat_multiple
from . import tutorial_iksolutions
from . import tutorial_iktranslation
from . import tutorial_iktranslation2d
from . import tutorial_inversereachability
from . import tutorial_plotting

# examples showing complex demos
from . import calibrationviews
from . import checkconvexdecomposition
from . import checkvisibility
from . import collision
from . import collision2
from . import constraintplanning
from . import cubeassembly
from . import dualarmdemo_schunk
from . import fastgrasping
from . import fastgraspingthreaded
from . import graspplanning
from . import hanoi
from . import inversekinematicspick
from . import movehandstraight
from . import pr2turnlever

try:
    from PyQt4 import QtGui, QtCore
    from . import qtexampleselector
    from . import qtserverprocess
except ImportError:
    pass

from . import showsensors
from . import simplegrasping
from . import simplemanipulation
from . import simplenavigation
from . import testphysics
from . import testphysics_controller
from . import testphysics_diffdrive
from . import testupdatingbodies
from . import testviewercallback
from . import visibilityplanning
