#!/usr/bin/env python
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

def run():
    orenv = Environment()
    orenv.SetViewer('qtcoin')
    h1 = orenv.plot3(points=array((-1.5,-1,0)),
                pointsize=5.0,
                colors=array((1,0,0)))
    h2 = orenv.plot3(points=array(((-1.5,-0.5,0),(-1.5,0.5,0))),
                pointsize=15.0,
                colors=array(((0,1,0),(0,0,0))))
    h3 = orenv.drawlinestrip(points=array(((-1.25,-0.5,0),(-1.25,0.5,0),(-1.5,1,0))),
                        linewidth=3.0,
                        colors=array(((0,1,0),(0,0,1),(1,0,0))))
    h4 = orenv.plot3(points=array(((-0.5,-0.5,0),(-0.5,0.5,0))),
                pointsize=0.05,
                colors=array(((0,1,0),(1,1,0))),
                drawstyle=1)
    h5 = orenv.drawtrimesh(points=array(((0,0,0),(0.5,0,0),(0,0.5,0))),
                      indices=None,
                      colors=array(((0,1,0),(0,0,1),(1,0,0))))
    h6 = orenv.drawtrimesh(points=array(((0,0,0.5),(0.5,0,0.5),(0,0.5,0.5),(0.5,0.5,0.5))),
                      indices=array(((0,1,2),(2,1,3)),int64),
                      colors=array((1,0,0,0.5)))
    raw_input('Enter any key to quit. ')
    h1 = None
    h2 = None
    h3 = None
    h4 = None
    h5 = None
    h6 = None
    orenv.Destroy()
    
if __name__=='__main__':
    run()
