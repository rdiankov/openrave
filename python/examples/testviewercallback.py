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
"""Shows how to setup a callback for mouse clicks on the viewer.

.. examplepre-block:: testviewercallback
  :image-width: 400

.. examplepost-block:: testviewercallback
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

ghandle = None
def itemselectioncb(link,pos,org,env):
    global ghandle
    print 'in python: body ',link.GetParent().GetName(),':',link.GetName(),'at',reshape(pos,(3))
    ghandle = env.plot3(points=pos,pointsize=25.0,colors=array((1,0,0)))
    return 0

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    handle = env.GetViewer().RegisterItemSelectionCallback(lambda link,pos,org: itemselectioncb(link,pos,org,env))
    if handle is None:
        print 'failed to register handle'
        sys.exit(1)
    while True:
        cmd = raw_input('In selection mode (ESC), click anywhere on the viewer. Enter command (q-quit): ')
        if cmd == 'q':
            break

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to attach a callback to a viewer to perform functions.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='OpenRAVE scene to load')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__=='__main__':
    run()
