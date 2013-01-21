# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
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

from openravepy import *
from openravepy import misc
import numpy
from numpy import *

from itertools import izip, combinations
import nose
from nose.tools import assert_raises
import fnmatch
import time
import os
import cPickle as pickle
import logging

_multiprocess_can_split_ = True

g_epsilon = 1e-7
g_jacobianstep = 0.01
g_envfiles = ['data/lab1.env.xml','data/pr2wam_test1.env.xml','data/hanoi_complex.env.xml']
g_robotfiles = ['robots/pr2-beta-static.zae','robots/barrettsegway.robot.xml','robots/neuronics-katana.zae','robots/pa10schunk.robot.xml','robots/barrettwam-dual.robot.xml']

log=logging.getLogger('openravepytest')
    
def setup_module(module):
    dbdir = os.path.join(os.getcwd(),'.openravetest')
    os.environ['OPENRAVE_DATABASE'] = dbdir
    os.environ['OPENRAVE_HOME'] = dbdir
    if hasattr(os,'putenv'):
        os.putenv('OPENRAVE_DATABASE',dbdir)
        os.putenv('OPENRAVE_HOME',dbdir)
    RaveInitialize(load_all_plugins=True, level=DebugLevel.Info|DebugLevel.VerifyPlans)
    if hasattr(os.path,'samefile'):
        assert(os.path.samefile(RaveGetHomeDirectory(),dbdir))
    else:
        assert(RaveGetHomeDirectory()==dbdir)
    try:
        colorize=__import__('logutils.colorize',fromlist=['colorize'])
        handler = colorize.ColorizingStreamHandler()
        handler.level_map[logging.DEBUG] =(None, 'green', False)
        handler.level_map[logging.INFO] = (None, None, False)
        handler.level_map[logging.WARNING] = (None, 'yellow', False)
        handler.level_map[logging.ERROR] = (None, 'red', False)
        handler.level_map[logging.CRITICAL] = ('white', 'magenta', True)
    except ImportError:
        handler = logging.StreamHandler()
        raveLogVerbose('python logutils not present so cannot colorize python output.')

    handler.setFormatter(logging.Formatter('%(name)s.%(funcName)s (pid='+str(os.getpid())+'): %(message)s'))
    log.setLevel(logging.INFO)
    log.addHandler(handler)
    logging.getLogger('openravepy').setLevel(logging.INFO)
    logging.getLogger('openravepy').addHandler(handler)
    
def teardown_module(module):
    RaveDestroy()

def transdist(list0,list1):
    assert(len(list0)==len(list1))
    return sum([sum(abs(item0-item1)) for item0, item1 in izip(list0,list1)])

def axisangledist(axis0,axis1):
    return 2*arccos(numpy.minimum(1.0,dot(quatFromAxisAngle(axis0),quatFromAxisAngle(axis1))))

def ComputePoseDistance(pose0,pose1):
    return sqrt(0.3*2*arccos(numpy.minimum(1.0,dot(pose0[0:4],pose1[0:4])))**2 + sum((pose0[4:7]-pose1[4:7])**2))

def randtrans():
    T = matrixFromAxisAngle(random.rand(3)*6-3)
    T[0:3,3] = random.rand(3)-0.5            
    return T

def randquat(N=1):
    L = 0
    while any(L == 0):
        q = random.rand(N,4)-0.5
        L = sqrt(sum(q**2,1))
    return q/tile(L,(4,1)).transpose()

def randpose(N=1):
    poses = random.rand(N,7)-0.5
    poses[:,0:4] /= tile(sqrt(sum(poses[:,0:4]**2,1)),(4,1)).transpose()
    return poses

def randlimits(lower,upper):
    return lower+random.rand(len(lower))*(upper-lower)

def bodymaxjointdist(link,localtrans):
    body = link.GetParent()
    joints = body.GetChain(0,link.GetIndex(),returnjoints=True)
    eetrans = transformPoints(link.GetTransform(),[localtrans])[0]
    armlength = 0
    for j in joints[::-1]:
        if not j.IsStatic():
            if j.IsRevolute(0):
                d = dot(j.GetAxis(0), eetrans-j.GetAnchor())
                orthogonaldir = eetrans-j.GetAnchor()-d*j.GetAxis(0)
                armlength += linalg.norm(orthogonaldir)
                eetrans -= orthogonaldir
    return armlength

def locate(pattern, root=os.curdir):
    """Locate all files matching supplied filename pattern in and below supplied root directory.
    """
    for path, dirs, files in os.walk(os.path.abspath(root)):
        for filename in fnmatch.filter(files, pattern):
            yield os.path.join(path, filename)

class EnvironmentSetup(object):
    __name__='openrave_common_test'
    def setup(self):
        self.env=Environment()
        self.env.StopSimulation()
        self.log = logging.getLogger('openravepytest.'+self.__class__.__name__)
        
    def teardown(self):
        self.env.Destroy()
        self.env=None
    def LoadDataEnv(self,*args,**kwargs):
        self.log.info('LoadDataEnv')
        assert(self.env.LoadData(*args,**kwargs))
        self._PreprocessEnv()
    
    def LoadEnv(self,*args,**kwargs):
        self.log.info('%r, %r',args,kwargs)
        assert(self.env.Load(*args,**kwargs))
        self._PreprocessEnv()

    def LoadRobot(self,*args,**kwargs):
        self.log.info('%r, %r',args,kwargs)
        robot=self.env.ReadRobotURI(*args,**kwargs)
        self.env.Add(robot,True)
        self._PreprocessRobot(robot)
        return robot

    def LoadRobotData(self,*args,**kwargs):
        robot=self.env.ReadRobotData(*args,**kwargs)
        self.env.Add(robot,True)
        self._PreprocessRobot(robot)
        return robot

    def RunTrajectory(self,robot,traj):
        assert(traj is not None)
        # set the first point of the trajectory manually
        robot.SetConfigurationValues(traj.GetWaypoint(0,robot.GetConfigurationSpecification()))
        robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            self.env.StepSimulation(0.01)
        
    def _PreprocessEnv(self):
        for robot in self.env.GetRobots():
            self._PreprocessRobot(robot)

    def _PreprocessRobot(self,robot):
        if robot.GetController() is not None and robot.GetController().GetXMLId().lower() == 'idealcontroller':
            # need to throw exceptions so test fails
            robot.GetController().SendCommand('SetThrowExceptions 1')
                    
def generate_classes(BaseClass, namespace, data):
    """Used to generate test classes inside a namespace since nose generators do not support classes

    Create the test classes using something like this:

generate_classes(Base, globals(), [
    ("Test1", expr1),
    ("Test2", expr2),
    ("Test3", expr3),
]) 
    """
    for args in data:
        test_name = args[0]
        args = args[1:]
        class DummyClass(BaseClass):
            def __init__(self, methodName=None):
                BaseClass.__init__(self, *args)

        # Set the name for this class, and place it into the namespace
        class_name = "test_" + test_name
        DummyClass.__name__ = class_name
        #from sys import modules
        #setattr(modules[BaseClass.__module__],class_name,DummyClass)
        if class_name in namespace:
            raise ValueError("namespace already contains a '%s' object" %class_name)
        namespace[class_name] = DummyClass
        
