# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov (rosen.diankov@gmail.com)
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
"""
ikfast testing is being synchronized with the ikfast version, so don't try to regenerate if ikfast version hasn't changed. However, the module will use the cached IK data if possible.


"""
import openravepy
from openravepy import databases, ikfast, IkParameterization, RaveInitialize, RaveDestroy, RaveCreateProblem, Environment, RaveLoadPlugin, RaveSetDebugLevel, RaveGetDebugLevel, DebugLevel, RaveFindDatabaseFile
import numpy
from itertools import izip, combinations

import time, unittest, platform, os, sys
from distutils import ccompiler
import nose
import logging
log = logging.getLogger(__name__)

import multiprocessing
_multiprocess_can_split_ = True

# global parameters
maxfreejoints = 2 # max free joints to allow, 3 or more will take too long to evaluate, and most likely will never be used in real life
perftiming = 10000 # perf timing, only computed when wrong solutions == 0
numiktests = [5000,10,100] # number of iktests indexed by the number of free joints
#iktypes = [iktype for value,iktype in IkParameterization.Type.values.iteritems()]
iktypes = [IkParameterization.Type.Transform6D] # IK types to test for
freeinc = 0.04 # increment of the free joints
globalstats = multiprocessing.Queue() # used for gathering statistics

env=None
ikfastproblem=None
def setup_robotstats():
    global env,ikfastproblem
    # just load the plugin we'll be using
    RaveInitialize(load_all_plugins=False)
    success = RaveLoadPlugin('libikfastsolvers')
    assert(success)
    RaveSetDebugLevel(DebugLevel.Error) # set to error in order to avoid expected plugin loading errors
    format = logging.Formatter('%(name)s: %(message)s')
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(format)
    databases.inversekinematics.log.addHandler(handler)
    databases.inversekinematics.log.setLevel(logging.ERROR)
    ikfast.log.addHandler(handler)
    ikfast.log.setLevel(logging.ERROR)
    env=Environment()
    env.StopSimulation()
    ikfastproblem = RaveCreateProblem(env,'ikfast')
    env.LoadProblem(ikfastproblem,'')

def teardown_robotstats():
    global env,ikfastproblem
    env.Remove(ikfastproblem)
    env.Destroy()
    RaveDestroy()

def test_robots():
    robotfilenames = ['robots/unimation-pumaarm.zae','robots/barrettwam.robot.xml']
    RaveInitialize(load_all_plugins=False)
    RaveSetDebugLevel(DebugLevel.Error) # set to error in order to avoid expected plugin loading errosr
    envlocal=Environment()
    envlocal.StopSimulation()
    try:
        for robotfilename in robotfilenames:
            envlocal.Reset()
            robot = envlocal.ReadRobotXMLFile(robotfilename,{'skipgeometry':'1'})
            envlocal.AddRobot(robot)
            for iktype in iktypes:
                expecteddof = IkParameterization.GetDOF(iktype)
                for manip in robot.GetManipulators():
                    armdof = len(manip.GetArmIndices())
                    if armdof >= expecteddof:
                        for freeindices in combinations(manip.GetArmIndices(),armdof-expecteddof):
                            yield robotstats, robotfilename, manip.GetName(), str(iktype),freeindices
    finally:
        envlocal.Destroy()
        # for some reason the plugindatabase _threadPluginLoader thread is on a different process
        # than the main threading waiting for it to finish, so it is necessary to call RaveDestroy
        RaveDestroy()

class InverseKinematicsModelTest(databases.inversekinematics.InverseKinematicsModel):
    """keeps the written shared object files separate from the main stream files and adds the used indices.
    """
    def __init__(self,robot,iktype,freeindices):
        databases.inversekinematics.InverseKinematicsModel.__init__(self,robot=robot,iktype=iktype)
        if freeindices is None:
            self.freeindices = self.getDefaultFreeIndices()
        else:
            self.freeindices = freeindices

    def getfilename(self,read=False):
        if self.iktype is None:
            raise ValueError('ik type is not set')
        
        solveindices = [i for i in self.manip.GetArmIndices() if not i in self.freeindices]
        basename = 'ikfast%s.%s.%s_'%(self.getversion(),self.iktype,platform.machine()) + '_'.join(str(ind) for ind in solveindices)
        if len(self.freeindices)>0:
            basename += '_f'+'_'.join(str(ind) for ind in self.freeindices)
        return RaveFindDatabaseFile(os.path.join('kinematics.'+self.manip.GetKinematicsStructureHash(),ccompiler.new_compiler().shared_object_filename(basename=basename)),read)

@nose.with_setup(setup_robotstats, teardown_robotstats)
def robotstats(robotfilename,manipname,iktypestr,freeindices):
    global env, ikfastproblem, globalstats
    iktype = None
    for value,type in IkParameterization.Type.values.iteritems():
        if type.name.lower() == iktypestr.lower():
            iktype = type
            break
    with env:
        robot = env.ReadRobotXMLFile(robotfilename,{'skipgeometry':'1'})
        env.AddRobot(robot)
        manip = robot.SetActiveManipulator(manipname)
        # set base to identity to avoid complications when reporting errors, testing that IK works under transformations is a different test
        robot.SetTransform(numpy.dot(numpy.linalg.inv(manip.GetBase().GetTransform()),robot.GetTransform()))
        manip=robot.SetActiveManipulator(manipname)
        ikmodel = InverseKinematicsModelTest(robot,iktype=iktype,freeindices=freeindices)
        try:
            # remove any default ik solver for the manipulator, it can get in the way loading
            ikmodel.manip.SetIKSolver(None)
            if not ikmodel.load():
                ikmodel.generate(iktype=iktype,freeindices=freeindices,forceikbuild=True)
                ikmodel.setrobot(freeinc)

            cmd = 'DebugIK robot %s '%robot.GetName()
            cmd += 'numtests %d '%int(numiktests[len(freeindices)])
            res = ikfastproblem.SendCommand(cmd).split()
            numtested = int(res[0])
            numsuccessful = int(res[1])
            solutionresults = []
            index = 2
            numvalues=1+IkParameterization.GetNumberOfValues(iktype)+manip.GetIkSolver().GetNumFreeParameters()
            for iresults in range(3):
                num = int(res[index])
                index += 1
                samples = numpy.reshape(numpy.array([numpy.float64(s) for s in res[index:(index+num*numvalues)]]),(num,numvalues))
                solutionresults.append(samples)
                index += num*numvalues
            print 'success rate: ',float(numsuccessful)/numtested
            print 'wrong solutions: %f', len(solutionresults[0])/numtested
            print 'no solutions: %f', len(solutionresults[1])/numtested
            print 'missing solution: %f',len(solutionresults[2])/numtested
            globalstats.put([numtested,numsuccessful,solutionresults])
            #raise IKStatisticsException(s)
        except ikfast.IKFastSolver.IKFeasibilityError:
            # this is expected, and is normal operation, have to notify
            pass
            #raise IKStatisticsException('not solvable!')


from noseplugins import multiprocess, xunitmultiprocess, capture
from nose.plugins.cover import Coverage

if __name__ == "__main__":
    nose.plugins.capture.log.setLevel(logging.DEBUG)
    import test_ikfast
    numprocesses = 1
    for arg in sys.argv[1:]:
        if arg.startswith('-j'):
            numprocesses = int(arg[2:])

    prog=nose.core.TestProgram(argv=['nosetests','-v','--with-xunitmp','--xunit-file=ikfastresults.xml','--processes=4','--process-timeout=1200','test_ikfast.py'],plugins=[capture.Capture(),multiprocess.MultiProcess(),xunitmultiprocess.Xunitmp()],exit=False)
    # save the queue to file
    f = open('stats.xml','w')
    while not test_ikfast.globalstats.empty():
        f.write(test_ikfast.globalstats.get())
