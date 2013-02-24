# -*- coding: utf-8 -*-
# Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
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
from common_test_openrave import *

import os

def _VmB(VmKey):
    '''Private.
    '''
    # get pseudo file  /proc/<pid>/status
    _proc_status = '/proc/%d/status' % os.getpid()
    _scale = {'kB': 1024.0, 'mB': 1024.0*1024.0, 'KB': 1024.0, 'MB': 1024.0*1024.0}
    try:
        t = open(_proc_status)
        v = t.read()
        t.close()
    except Exception,e:
        # non-Linux?
        print e
        return 0.0
        
    # get VmKey line e.g. 'VmRSS:  9999  kB\n ...'
    i = v.index(VmKey)
    # whitespace
    v = v[i:].split(None, 3) 
    if len(v) < 3:
        # invalid format?
        return 0.0
        
    # convert Vm value to bytes
    return float(v[1]) * _scale[v[2]]

def memory(since=0.0):
    '''Return memory usage in bytes.
    '''
    return _VmB('VmSize:') - since


def resident(since=0.0):
    '''Return resident memory usage in bytes.
    '''
    return _VmB('VmRSS:') - since


def stacksize(since=0.0):
    '''Return stack size in bytes.
    '''
    return _VmB('VmStk:') - since

class TestViewer(EnvironmentSetup):
    def setup(self):
        EnvironmentSetup.setup(self)
        # select collision engine here
        #self.env.SetViewer('qtcoin',False)

#     def test_memory(self):
#         env=self.env
#         env.StartSimulation(0.1,False)
#         self.LoadEnv('data/pr2test2.env.xml')
#         memusage = []
#         for i in range(10):
#             memusage.append(memory())
#             time.sleep(1)
#         env.Destroy()
#         # check if memory is growing
#         assert(memusage[-1] < memusage[1]*2)
#         

    def test_crash(self):
        env=self.env
        filename = 'data/mug1.dae'
        env.Load(filename)
        env.SetViewer('qtcoin')
        #env.Reset()
        print 'env2 '
        env.Load(filename)
        print 'done'
        time.sleep(4)
        print 'quitting'
        
