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
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
import numpy

def run(args=None):
    """Executes the testplotting example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    orenv = Environment()
    orenv.SetViewer('qtcoin')
    handles = []
    handles.append(orenv.plot3(points=array(((-1.5,-0.5,0),(-1.5,0.5,0))),
                               pointsize=15.0,
                               colors=array(((0,1,0),(0,0,0)))))
    handles.append(orenv.plot3(points=array((-1.5,-1,0)),
                               pointsize=25.0,
                               colors=array(((0,0,1,0.2)))))
    handles.append(orenv.drawlinestrip(points=array(((-1.25,-0.5,0),(-1.25,0.5,0),(-1.5,1,0))),
                                       linewidth=3.0,
                                       colors=array(((0,1,0),(0,0,1),(1,0,0)))))
    handles.append(orenv.plot3(points=array(((-0.5,-0.5,0),(-0.5,0.5,0))),
                               pointsize=0.05,
                               colors=array(((0,1,0),(1,1,0))),
                               drawstyle=1))
    handles.append(orenv.drawtrimesh(points=array(((0,0,0),(0.5,0,0),(0,0.5,0))),
                                     indices=None,
                                     colors=array(((0,1,0),(0,0,1),(1,0,0)))))
    handles.append(orenv.drawtrimesh(points=array(((0,0,0.5),(0.5,0,0.5),(-0.5,0.5,0.5),(1,0.5,0.5))),
                                     indices=array(((0,1,2),(2,1,3)),int64),
                                     colors=array((1,0,0,0.5))))
    handles.append(orenv.plot3(points=array(((0.5,0,1.0),(-0.5,0,1.0))),
                               pointsize=45.0,
                               colors=array(((0,0,0,0.1),(0,0,0,0.8)))))
    # draw a random texture with alpha channel
    X,Y = meshgrid(arange(0,1,0.005),arange(0,1,0.01))
    Ic = zeros((X.shape[0],X.shape[1],4))
    Ic[:,:,1] = 0.5*sin(20*Y)+0.5
    Ic[:,:,2] = 1
    Ic[:,:,3] = 0.5*sin(20*Y)+0.5
    handles.append(orenv.drawplane(transform=matrixFromAxisAngle(array((1,0,0)),pi/4),extents=[1.0,0.5],texture=Ic))
    raw_input('Enter any key to quit. ')
    handles = None
    orenv.Destroy()
    
if __name__=='__main__':
    run()
