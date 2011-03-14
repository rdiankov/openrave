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
"""Shows how to setup a callback for mouse clicks on the viewer.

.. image:: ../../images/examples/testviewercallback.jpg
  :height: 256

**Running the Example**::

  openrave.py --example testviewercallback

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
from optparse import OptionParser

ghandle = None
def itemselectioncb(link,pos,org,env):
    global ghandle
    print 'in python: body ',link.GetParent().GetName(),':',link.GetName(),'at',reshape(pos,(3))
    ghandle = env.plot3(points=pos,pointsize=25.0,colors=array((1,0,0)))
    return 0

@with_destroy
def run(args=None):
    """Executes the testviewercallback example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to attach a callback to a viewer to perform functions.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='OpenRAVE scene to load')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load(options.scene)
    handle = env.GetViewer().RegisterCallback(Viewer.Events.ItemSelection,lambda link,pos,org: itemselectioncb(link,pos,org,env))
    if handle is None:
        print 'failed to register handle'
        sys.exit(1)
    while True:
        cmd = raw_input('In selection mode, click anywhere on the viewer. Enter command (q-quit): ')
        if cmd == 'q':
            break

if __name__=='__main__':
    run()
