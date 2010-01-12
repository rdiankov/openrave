# when openravepy becomes a more official package, see http://peak.telecommunity.com/DevCenter/setuptools#namespace-packages
# try: 
#     __import__('pkg_resources').declare_namespace(__name__) 
# except ImportError: 
#     from pkgutil import extend_path 
#     __path__ = extend_path(__path__, __name__) 

from openravepy_int import *
from openravepy_ext import *
import metaclass
import ikfast
import examples
import interfaces
import pyANN
