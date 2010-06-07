from openravepy_int import *
from openravepy_int import _openrave_exception_
from openravepy_int import __version__
from openravepy_int import __author__
from openravepy_int import __copyright__
__license = 'core: Lesser GPL, examples: Apache License, Version 2.0'

from openravepy_ext import *

import metaclass
try:
    import ikfast
except ImportError, e:
    print 'openravepy: Failed to import ikfast: ',e
import pyANN
import convexdecompositionpy
import interfaces
import databases
import examples

pyANN._pyann_exception_.py_err_class = pyann_exception
_openrave_exception_.py_err_class = openrave_exception

# would "from openravepy import *" be slower if this is enabled?
#__all__ = ["examples", "ikfast", "interfaces", "metaclass", "pyANN"]
