# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov <rosen.diankov@gmail.com>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 
"""Library allows OpenRAVE to be used through Python.

Beginners should first check out :ref:`openravepy_beginning`.

Internals
---------

There is a __build_doc__ external variable that is set to True only when building docs. This allows package to clean their symbols and documentation.
"""
try:
    if __openravepy_build_doc__:
        print('openravepy imported in documentation mode')
except NameError:
    __builtins__['__openravepy_build_doc__'] = False

from .openravepy_int import *
from .openravepy_int import __version__
from .openravepy_int import __author__
from .openravepy_int import __copyright__
__license__ = 'core: Lesser GPL, examples: Apache License, Version 2.0'
__docformat__ = 'restructuredtext'

"""
When building with Boost.Python, this wraps up the C++ class openravepy::openrave_exception.
Available methods for an exception e are
- e.GetCode()
- e.message()
"""
if openravepy_int.__pythonbinding__ == 'pybind11':
    from .openravepy_int import _OpenRAVEException as OpenRAVEException
else:
    from .openravepy_int import _OpenRAVEException
    
    class openrave_exception_helper(Exception):
        # wrap up the C++ openrave_exception
        def __init__( self, app_error ):
            Exception.__init__( self )
            self._pimpl = app_error
        def __str__( self ):
            return str(self._pimpl)
        def __unicode__( self ):
            return unicode(self._pimpl)
        def __getattribute__(self, attr):
            my_pimpl = super(openrave_exception_helper, self).__getattribute__("_pimpl")
            try:
                return getattr(my_pimpl, attr)
            except AttributeError:
                return super(openrave_exception_helper,self).__getattribute__(attr)
    
    class std_exception(Exception):
        """wrap up the C++ std_exception"""
        def __init__( self, app_error ):
            Exception.__init__( self )
            self._pimpl = app_error
        def __str__( self ):
            return self._pimpl.message()
        def __getattribute__(self, attr):
            my_pimpl = super(std_exception, self).__getattribute__("_pimpl")
            try:
                return getattr(my_pimpl, attr)
            except AttributeError:
                return super(std_exception,self).__getattribute__(attr)
    
    class runtime_error(Exception):
        """wrap up the C++ runtime_error"""
        def __init__( self, app_error ):
            Exception.__init__( self )
            self._pimpl = app_error
        def __str__( self ):
            return self._pimpl.message()
        def __getattribute__(self, attr):
            my_pimpl = super(runtime_error, self).__getattribute__("_pimpl")
            try:
                return getattr(my_pimpl, attr)
            except AttributeError:
                return super(runtime_error,self).__getattribute__(attr)
    
    OpenRAVEException = openrave_exception_helper
    _OpenRAVEException.py_err_class = openrave_exception_helper

openrave_exception = OpenRAVEException # for back compat

class PlanningError(Exception):
    def __init__(self,parameter=u'', recoverySuggestions=None):
        """:param recoverySuggestions: list of unicode suggestions to fix or recover from the error
        """
        self.parameter = unicode(parameter)
        if recoverySuggestions is None:
            self.recoverySuggestions = []
        else:
            self.recoverySuggestions = [unicode(s) for s in recoverySuggestions]
            
    def __unicode__(self):
        s = u'Planning Error\n%s'%self.parameter
        if len(self.recoverySuggestions) > 0:
            s += u'\nRecovery Suggestions:\n'
            for suggestion in self.recoverySuggestions:
                s += u'- %s\n'%unicode(suggestion)
            s += u'\n'
        return s
        
    def __str__(self):
        return unicode(self).encode('utf-8')
    
    def __repr__(self):
        return '<openravepy.PlanningError(%r,%r)>'%(self.parameter,self.recoverySuggestions)
    
    def __eq__(self, r):
        return self.parameter == r.parameter and self.recoverySuggestions == r.recoverySuggestions
    
    def __ne__(self, r):
        return self.parameter != r.parameter or self.recoverySuggestions != r.recoverySuggestions
    
# deprecated
planning_error = PlanningError

from .openravepy_ext import *

from . import metaclass
from . import interfaces
from . import databases

OpenRAVEModel = databases.DatabaseGenerator # for backwards compatibility

# would "from openravepy import *" be slower if this is enabled?
#__all__ = ["interfaces", "databases", "metaclass", "openravepy_int"]
