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
"""Imports ANN extentions
"""
from .pyANN_int import *
from .pyANN_int import _pyann_exception_

class pyann_exception(Exception):
    """wrap up the C++ pyann_exception"""
    def __init__( self, app_error ):
        Exception.__init__( self )
        self._pimpl = app_error
    def __str__( self ):
        return self._pimpl.message()
    def __getattribute__(self, attr):
        my_pimpl = super(pyann_exception, self).__getattribute__("_pimpl")
        try:
            return getattr(my_pimpl, attr)
        except AttributeError:
            return super(pyann_exception,self).__getattribute__(attr)

_pyann_exception_.py_err_class = pyann_exception
