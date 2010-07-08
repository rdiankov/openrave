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
from optparse import OptionParser
import sys,time

if __name__ == "__main__":
    parser = OptionParser(description='OpenRAVE %s'%openravepy.__version__)
    parser.add_option('--listplugins', action="store_true",dest='listplugins',default=False,
                      help='List all plugins and the interfaces they provide.')
    parser.add_option('--loadplugin', action="append",type='string',dest='loadplugins',default=[],
                      help='List all plugins and the interfaces they provide.')
    parser.add_option('--collision', action="store",type='string',dest='collision',default=None,
                      help='Default collision checker to use')
    parser.add_option('--physics', action="store",type='string',dest='physics',default=None,
                      help='physics engine to use (default=%default)')
    parser.add_option('--viewer', action="store",type='string',dest='viewer',default='qtcoin',
                      help='viewer to use (default=%default)' )
    parser.add_option('--server', action="store",type='string',dest='server',default=None,
                      help='server to use (default=%default).')
    parser.add_option('--serverport', action="store",type='int',dest='serverport',default=4765,
                      help='port to load server on (default=%default).')
    parser.add_option('--debug','-d', action="store",type='string',dest='debug',default=None,
                      help='Debug level')
    parser.add_option('--database', action="store",type='string',dest='database',default=None,
                      help='If set, will call a database generator from the openravepy.databases module. The database string contains both the database name and the arguments that will be passed to it; any arguments specified in %prog will be ignored.')
    parser.add_option('--example', action="store",type='string',dest='example',default=None,
                      help='Will call an example from the openravepy.examples module. The example string contains both the example name and the arguments that will be passed to it; any arguments specified in %prog will be ignored.')
    parser.add_option('--ipython', '-i',action="store_true",dest='ipython',default=False,
                      help='if true will drop into the ipython interpreter rather than spin')
    parser.add_option('--version',action='store_true',dest='version',default=False,
                      help='Return the openrave version MAJOR.MINOR.PATCH')
    parser.add_option('--listexamples',action='store_true',dest='listexamples',default=False,
                      help='Lists the available core examples')
    parser.add_option('--listdatabases',action='store_true',dest='listdatabases',default=False,
                      help='Lists the available core database generators')
    (options, args) = parser.parse_args()

    if options.version:
        print openravepy.__version__
        sys.exit(0)
    if options.listdatabases:
        for name in dir(databases):
            if not name.startswith('__'):
                print name
        sys.exit(0)
    if options.listexamples:
        for name in dir(examples):
            if not name.startswith('__'):
                print name
        sys.exit(0)
    if options.database is not None:
        args=options.database.split()
        try:
            database=getattr(databases,args[0])
        except (AttributeError,IndexError):
            print 'bad database generator, current list of executable generators are:'
            for name in dir(databases):
                if not name.startswith('__'):
                    print ' ' + name
            sys.exit(1)
        database.run(args=args)
        sys.exit(0)
    if options.example is not None:
        args=options.example.split()
        try:
            example = getattr(examples,args[0])
        except (AttributeError,IndexError):
            print 'bad example, current list of executable examples are:'
            for name in dir(examples):
                if not name.startswith('__'):
                    print ' ' + name
            sys.exit(1)
        example.run(args=args)
        sys.exit(0)
    if options.debug is not None:
        for debuglevel in [DebugLevel.Fatal,DebugLevel.Error,DebugLevel.Warn,DebugLevel.Info,DebugLevel.Debug,DebugLevel.Verbose]:
            if (not options.debug.isdigit() and options.debug.lower() == debuglevel.name.lower()) or (options.debug.isdigit() and int(options.debug) == int(debuglevel)):
                RaveSetDebugLevel(debuglevel)
                break
    env = Environment()
    try:
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
        if options.physics:
            ph = env.CreatePhysicsEngine(options.physics)
            if ph is not None:
                env.SetPhysicsEngine(ph)
        if options.server:
            sr = env.CreateProblem(options.server)
            if sr is not None:
                env.LoadProblem(sr,'%d'%options.serverport)
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
