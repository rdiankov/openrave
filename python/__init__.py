# unfortunately it is necessary to add openravepy to the path in order to get sympy working since it uses constructs like "from sympy.core import *"
import sys,pkgutil
sys.path += pkgutil.extend_path(__path__, __name__)

from openravepy_int import *
from openravepy_int import __version__
from openravepy_int import __author__
from openravepy_int import __copyright__
__license = 'core: Lesser GPL, examples: Apache License, Version 2.0'

from openravepy_ext import *
import metaclass
try:
    import sympy
    import ikfast
except ImportError, e:
    print 'openravepy: Failed to import sympy and ikfast: ',e
import examples
import interfaces
import pyANN
