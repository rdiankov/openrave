# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
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
from openravepy import *
from numpy import *
from itertools import izip, combinations
import nose
from common_test_openrave import *

_multiprocess_can_split_ = True

def run_example(name):
    __doc__='testing example '+name
    example = getattr(examples,name)
    example.run(args=['--testmode',"--viewer="])

def test_examples():
    """test if all the examples run"""
    yield run_example, 'hanoi'
    yield run_example, 'calibrationviews'
    yield run_example, 'graspplanning'
#     for name in dir(examples):
#         if not name.startswith('__'):
#             try:
#                 m=__import__('openravepy.examples.'+name)
#                 if type(m) is ModuleType:
#                     yield test_example, name
#             except ImportError:
#                 pass




def test_databases():
    """test if all the databases run on default parameters"""
    pass
