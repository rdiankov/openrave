#!/usr/bin/env python
"""
Kinematic, quasi-static, dynamic, and geometric analyses are precomputed and used as databases during the run-time. All database generators rely on a specific robot and provide many helpful methods to use the information after it has been generated.

The base abstract class is `databases.DatabaseGenerator` and always takes a robot as its
parameter. Sometimes database generators are dependent on a robot's manipulator, sensor, or target
object. These extra parameters are always required at the constructor level. The hashes of all
objects the generator relies on are used to produce a unique ID to index the database with
$OPENRAVE_DATABASE. For example, the grasping database will combine the robot manipulator hash and
the target object hash.
"""
from __future__ import with_statement # for python 2.5

try:
    import cPickle as pickle
except:
    import pickle

from .. import openravepy_int
from .. import metaclass
from ..misc import mkdir_recursive
from ..misc import OpenRAVEGlobalArguments
import os.path
from os import getenv
import time

class DatabaseGenerator(metaclass.AutoReloader):
    """The base class defining the structure of the openrave database generators.
    """
    def __init__(self,robot):
        self.robot = robot
        self.env = self.robot.GetEnv()
        try:
            self.manip = self.robot.GetActiveManipulator()
        except:
            self.manip = None
    def clone(self,envother):
        """clones a database onto a different environment"""
        import copy
        clone = copy.copy(self)
        clone.env = envother
        clone.robot = clone.env.GetRobot(self.robot.GetName())
        clone.manip = clone.robot.GetManipulators(self.manip.GetName())[0] if not self.manip is None else None
        return clone
    def has(self):
        raise NotImplementedError()
    def getfilename(self,read=False):
        return NotImplementedError()
    def load(self):
        filename = self.getfilename(True)
        if len(filename) == 0:
            return None
        try:
            modelversion,params = pickle.load(open(filename, 'r'))
            if modelversion == self.getversion():
                return params
            else:
                print 'version is wrong ',modelversion,'!=',self.getversion()
        except MemoryError,e:
            print '%s failed: '%filename,e
        except:
            pass
        return None
    def getversion(self):
        return 0
    def save(self,params):
        filename=self.getfilename(False)
        print 'saving model to %s'%filename
        mkdir_recursive(os.path.split(filename)[0])
        pickle.dump((self.getversion(),params), open(filename, 'w'))
    def generate(self):
        raise NotImplementedError()
    def show(self,options=None):
        raise NotImplementedError()

    def autogenerateparams(self,options=None):
        """Caches parameters for most commonly used robots/objects and starts the generation process for them"""
        raise NotImplementedError()

    def autogenerate(self,options=None):
        self.generate(*self.autogenerateparams(options))
        self.save()

    def generatepcg(self):
        """Generate producer, consumer, and gatherer functions allowing parallelization
        """
        return NotImplementedError()
    def generate(self,*args,**kwargs):
        starttime = time.time()
        producer,consumer,gatherer,numjobs = self.generatepcg(*args,**kwargs)
        print 'database %s has %d items'%(__name__,num)
        for work in producer():
            results = consumer(*work)
            if len(results) > 0:
                gatherer(*results)
        gatherer() # gather results
        print 'database %s finished in %fs'%(__name__,time.time()-starttime)

    @staticmethod
    def CreateOptionParser(useManipulator=True):
        """set basic option parsing options for using databasers through the command line
        """
        from optparse import OptionParser, OptionGroup
        parser = OptionParser(description='OpenRAVE Database Generator.')
        OpenRAVEGlobalArguments.addOptions(parser)
        dbgroup = OptionGroup(parser,"OpenRAVE Database Generator General Options")
        dbgroup.add_option('--show',action='store_true',dest='show',default=False,
                           help='Graphically shows the built model')
        dbgroup.add_option('--getfilename',action="store_true",dest='getfilename',default=False,
                           help='If set, will return the final database filename where all data is stored')
        dbgroup.add_option('--gethas',action="store_true",dest='gethas',default=False,
                           help='If set, will exit with 0 if datafile is generated and up to date, otherwise will return a 1. This will require loading the model and checking versions, so might be a little slow.')
        dbgroup.add_option('--robot',action='store',type='string',dest='robot',default=getenv('OPENRAVE_ROBOT',default='robots/barrettsegway.robot.xml'),
                           help='OpenRAVE robot to load (default=%default)')
        dbgroup.add_option('--numthreads',action='store',type='int',dest='numthreads',default=1,
                           help='number of threads to compute the database with (default=%default)')
        if useManipulator:
            dbgroup.add_option('--manipname',action='store',type='string',dest='manipname',default=None,
                               help='The name of the manipulator on the robot to use')
        parser.add_option_group(dbgroup)
        return parser

    @staticmethod
    def InitializeFromParser(Model,parser=None,env=None,args=None,robotatts=dict(),defaultviewer=False,allowkinbody=False):
        """run the database generator from the command line using
        """
        from numpy import eye
        import sys
        if parser is None:
            parser = DatabaseGenerator.CreateOptionParser()
        (options, args) = parser.parse_args(args=args)
        loadplugins=True
        level=openravepy_int.DebugLevel.Info
        if options.getfilename:
            loadplugins = False
            level = openravepy_int.DebugLevel.Fatal
        if options.gethas:
            level = openravepy_int.DebugLevel.Fatal
        openravepy_int.RaveInitialize(loadplugins,level)
        OpenRAVEGlobalArguments.parseGlobal(options)
        destroyenv = False
        if env is None:
            env = openravepy_int.Environment()
            destroyenv = True
        try:
            options.viewername=OpenRAVEGlobalArguments.parseEnvironment(options,env,defaultviewer=defaultviewer,returnviewer=True)
            with env:
                env.Load(options.robot,robotatts)
                # TODO: if exception is raised after this point, program exits with glibc double-link list corruption. most likely something in Load?
                if len(env.GetRobots()) > 0:
                    robot = env.GetRobots()[0]
                elif allowkinbody:
                    robot = env.GetBodies()[0]
                assert(robot is not None)
                robot.SetTransform(eye(4))
                if hasattr(options,'manipname') and robot.IsRobot():
                    if options.manipname is None:
                        # prioritize manipulators with ik solvers
                        indices = [i for i,m in enumerate(robot.GetManipulators()) if m.GetIkSolver() is not None]
                        if len(indices) > 0:
                            robot.SetActiveManipulator(indices[0])
                    else:
                        robot.SetActiveManipulator([i for i,m in enumerate(robot.GetManipulators()) if m.GetName()==options.manipname][0])
            model = Model(robot=robot)
            destroyenv = False
            return options,model
        
        finally:
            if destroyenv:
                robot = None
                model = None
                env.Destroy()

    @staticmethod
    def RunFromParser(Model=None,env=None,parser=None,args=None,robotatts=dict(),defaultviewer=False,allowkinbody=False,**kwargs):
        """run the database generator from the command line using
        """
        import sys
        orgenv=env
        destroyenv = orgenv is None
        env = None
        try:
            options,model = DatabaseGenerator.InitializeFromParser(Model,parser,orgenv,args,robotatts,defaultviewer,allowkinbody)
            env=model.env
            if options.getfilename:
                # prioritize first available
                filename=model.getfilename(True)
                if len(filename) == 0:
                    filename=model.getfilename(False)
                print filename
                openravepy_int.RaveDestroy()
                sys.exit(0)
            if options.gethas:
                hasmodel=model.load()
                print int(hasmodel)
                openravepy_int.RaveDestroy()
                sys.exit(not hasmodel)
            if options.viewername is not None:
                env.SetViewer(options.viewername)
            if options.show:
                if not model.load():
                    raise ValueError('failed to find cached model %s:%s'%(model.getfilename(True),model.getfilename(False)))
                model.show(options=options)
                return model
            model.autogenerate(options=options)
            return model
        finally:
            if destroyenv and env is not None:
                env.Destroy()

import convexdecomposition
import linkstatistics
import grasping
import inversekinematics
import kinematicreachability
import inversereachability
import visibilitymodel
