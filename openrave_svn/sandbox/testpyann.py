#!/usr/bin/env python
# Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
import numpy

def run():
    kdtree = pyANN.KDTree(random.rand(1000,5))
    neighs,dists = kdtree.kSearch(random.rand(5),10,0.001)
    print neighs,dists
    print 'radius search, single point'
    neighs,dists,kball = kdtree.kFRSearch(random.rand(5),0.2**2,10,0.001)
    print neighs,dists,kball
    print 'radius search, array'
    neighs,dists,kball = kdtree.kFRSearchArray(random.rand(10,5),0.2**2,10,0.001)
    for n,d in zip(neighs,dists):
        print n[n>=0],d[n>=0]

if __name__=='__main__':
    run()
