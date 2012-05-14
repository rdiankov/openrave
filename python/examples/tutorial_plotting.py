#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
"""Example plotting calls.

.. examplepre-block:: tutorial_plotting

.. examplepost-block:: tutorial_plotting
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time, threading
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class PlotSpinner(threading.Thread):
    def __init__(self,handle):
        threading.Thread.__init__(self)
        self.starttime = time.time()
        self.handle=handle
        self.ok = True
    def run(self):
        while self.ok:
            self.handle.SetTransform(matrixFromAxisAngle([0,mod(time.time()-self.starttime,2*pi),0]))
            self.handle.SetShow(bool(mod(time.time()-self.starttime,2.0) < 1.0))
            time.sleep(0.01)

def main(env,options):
    "Main example code."
    spinner = None
    try:
        handles = []
        handles.append(env.plot3(points=array(((-1.5,-0.5,0),(-1.5,0.5,0))),
                                   pointsize=15.0,
                                   colors=array(((0,1,0),(0,0,0)))))
        handles.append(env.plot3(points=array((-1.5,-1,0)),
                                   pointsize=25.0,
                                   colors=array(((0,0,1,0.2)))))
        handles.append(env.drawlinestrip(points=array(((-1.25,-0.5,0),(-1.25,0.5,0),(-1.5,1,0))),
                                           linewidth=3.0,
                                           colors=array(((0,1,0),(0,0,1),(1,0,0)))))
        handles.append(env.plot3(points=array(((-0.5,-0.5,0),(-0.5,0.5,0))),
                                   pointsize=0.05,
                                   colors=array(((0,1,0),(1,1,0))),
                                   drawstyle=1))
        handles.append(env.drawtrimesh(points=array(((0,0,0),(0.5,0,0),(0,0.5,0))),
                                         indices=None,
                                         colors=array(((0,1,0),(0,0,1),(1,0,0)))))
        handles.append(env.drawtrimesh(points=array(((0,0,0.5),(0.5,0,0.5),(-0.5,0.5,0.5),(1,0.5,0.5))),
                                         indices=array(((0,1,2),(2,1,3)),int64),
                                         colors=array((1,0,0,0.5))))
        handles.append(env.plot3(points=array(((0.5,0,1.0),(-0.5,0,1.0))),
                                   pointsize=45.0,
                                   colors=array(((0,0,0,0.1),(0,0,0,0.8)))))
        # draw a random texture with alpha channel
        X,Y = meshgrid(arange(0,1,0.005),arange(0,1,0.01))
        Ic = zeros((X.shape[0],X.shape[1],4))
        Ic[:,:,1] = 0.5*sin(20*Y)+0.5
        Ic[:,:,2] = 1
        Ic[:,:,3] = 0.5*sin(20*Y)+0.5
        handles.append(env.drawplane(transform=matrixFromAxisAngle(array((1,0,0)),pi/4),extents=[1.0,0.5],texture=Ic))
        # spin one of the plots in another thread
        spinner = PlotSpinner(handles[-1])
        spinner.start()
        Tcamera = eye(4)
        Tcamera[0:3,3] = [-0.37, 0.26, 3.3]
        env.GetViewer().SetCamera(Tcamera)
        raw_input('Enter any key to quit. ')
        handles = None
    finally:
        if spinner is not None:
            spinner.ok = False

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='tutorial_plotting')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)
    
if __name__=='__main__':
    run()
