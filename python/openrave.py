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
from optparse import OptionParser
import sys,time

if __name__ == "__main__":
    parser = OptionParser(description='Queries body/robot/interface hashes')
    parser.add_option('--listplugins', action="store_true",dest='listplugins',default=False,
                      help='List all plugins and the interfaces they provide.')
    parser.add_option('--loadplugin', action="append",type='string',dest='loadplugins',default=[],
                      help='List all plugins and the interfaces they provide.')
    parser.add_option('--collision', action="store",type='string',dest='collision',default=None,
                      help='Default collision checker to use')
    parser.add_option('--physics', action="store",type='string',dest='physics',default=None,
                      help='Default physics engine to use')
    parser.add_option('--viewer', action="store",type='string',dest='viewer',default='qtcoin',
                      help='Default viewer to use.')
    parser.add_option('--debug','-d', action="store",type='string',dest='debug',default=None,
                      help='Debug level')
    parser.add_option('--robothash', action="store",type='string',dest='robothash',default=None,
                      help='if specified will query the robot structure hash and return')
    parser.add_option('--bodyhash', action="store",type='string',dest='bodyhash',default=None,
                      help='if specified will query the kinbody hash and return')
    parser.add_option('--ipython', '-i',action="store_true",dest='ipython',default=False,
                      help='if true will drop into the ipython interpreter rather than spin')
    (options, args) = parser.parse_args()

    if options.debug is not None:
        for debuglevel in [DebugLevel.Fatal,DebugLevel.Error,DebugLevel.Warn,DebugLevel.Info,DebugLevel.Debug,DebugLevel.Verbose]:
            if (not options.debug.isdigit() and options.debug.lower() == debuglevel.name.lower()) or (options.debug.isdigit() and int(options.debug) == int(debuglevel)):
                RaveSetDebugLevel(debuglevel)
                break
    else: # test if special functions are requested whose output shouldn't be cluttered with debug statements
        if options.robothash is not None or options.bodyhash is not None or options.listplugins:
            RaveSetDebugLevel(DebugLevel.Fatal)
    env = Environment()
    try:
        if options.robothash is not None:
            env.Reset()
            env.Load(options.robothash)
            print env.GetRobots()[0].GetRobotStructureHash()
            sys.exit(0)
        if options.bodyhash is not None:
            env.Reset()
            env.Load(options.bodyhash)
            print env.GetBodies()[0].GetKinematicsGeometryHash()
            sys.exit(0)
        if options.listplugins:
            plugins = env.GetPluginInfo()
            interfacenames = dict()
            for name,type in PluginType.names.iteritems():
                interfacenames[type] = []
            for pluginname,info in plugins:
                for type,names in info.interfacenames:
                    interfacenames[type] += [(n,pluginname) for n in names]
            print 'Number of plugins: %d'%len(plugins)
            for type,names in interfacenames.iteritems():
                print '%s: %d'%(str(type),len(names))
                names.sort()
                for interfacename,pluginname in names:
                    print '  %s - %s'%(interfacename,pluginname)
            sys.exit(0)
        if options.collision:
            cc = env.CreateCollisionChecker(options.collision)
            if cc is not None:
                env.SetCollisionChecker(cc)
        for arg in args:
            if arg.endswith('.xml') or arg.endswith('.dae'):
                env.Load(arg)
        if options.viewer:
            env.SetViewer(options.viewer)
        if options.ipython:
            with env:
                robots=env.GetRobots()
                robot=None if len(robots) == 0 else robots[0]
            from IPython.Shell import IPShellEmbed
            ipshell = IPShellEmbed(argv='',banner = 'OpenRAVE Dropping into IPython',exit_msg = 'Leaving Interpreter and closing program.')
            ipshell(local_ns=locals())
            sys.exit(0)
        while True:
            time.sleep(0.01)
    finally:
        env.Destroy()
