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
        print 'openravepy imported in documentation mode'
except NameError:
    __builtins__['__openravepy_build_doc__'] = False

from openravepy_int import *
from openravepy_int import _openrave_exception_
from openravepy_int import __version__
from openravepy_int import __author__
from openravepy_int import __copyright__
__license__ = 'core: Lesser GPL, examples: Apache License, Version 2.0'
__docformat__ = 'restructuredtext'

from openravepy_ext import *
import metaclass
import interfaces
import databases

OpenRAVEModel = databases.DatabaseGenerator # for backwards compatibility
_openrave_exception_.py_err_class = openrave_exception

# deprecated
Problem = Module

# would "from openravepy import *" be slower if this is enabled?
#__all__ = ["interfaces", "databases", "metaclass", "openravepy_int"]
