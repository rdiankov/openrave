#!/usr/bin/env python
# Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
from openravepy import *
from numpy import *
from optparse import OptionParser

ghandle = None
def itemselectioncb(link,pos,org,env):
    global ghandle
    print 'in python: body ',link.GetParent().GetName(),':',link.GetName(),'at',reshape(pos,(3))
    ghandle = env.plot3(points=pos,pointsize=25.0,colors=array((1,0,0)))
    return 0

if __name__=='__main__':
    global env
    parser = OptionParser(description='Shows how to attach a callback to a viewer to perform functions.')
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='OpenRAVE scene to load')
    parser.add_option('--viewer',
                      action="store",type='string',dest='viewer',default='qtcoin',
                      help='Viewer to load')
    (options, args) = parser.parse_args()

    env = Environment()
    env.Load(options.scene)
    env.SetViewer(options.viewer)
    handle = env.GetViewer().RegisterCallback(Viewer.ViewerEvents.ItemSelection,lambda link,pos,org: itemselectioncb(link,pos,org,env))
    if handle is None:
        print 'failed to register handle'
        sys.exit(1)

    while(True):
        cmd = raw_input('In selection mode, click anywhere on the viewer. Enter command (q-quit): ')
        if cmd == 'q':
            break
    env.Destroy() # done with the environment
