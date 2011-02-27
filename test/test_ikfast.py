
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
from optparse import OptionParser

import time, unittest, platform, os, sys
from distutils import ccompiler
import nose
import logging
from functools import partial
import multiprocessing

#global variables
_multiprocess_can_split_ = True
#globalstats = multiprocessing.Queue() # used for gathering statistics
options = None
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

def measurement(name,value):
    return '<measurement><name>%s</name><value>%s</value></measurement>'%(name,value)

@nose.with_setup(setup_robotstats, teardown_robotstats)
def robotstats(description,robotfilename,manipname, iktypestr,freeindices):
    global env, ikfastproblem#, globalstats
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
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=iktype,freeindices=freeindices)
        freeindicesstr = ', '.join(robot.GetJointFromDOFIndex(dof).GetName()+'('+str(dof)+')' for dof in freeindices)
        description.append('%s::%s.%s free:[%s]'%(os.path.split(robotfilename)[1].split('.')[0], manipname, iktypestr,freeindicesstr))
        try:
            # remove any default ik solver for the manipulator, it can get in the way loading
            ikmodel.manip.SetIKSolver(None)
            ikfasttime = None
            if not ikmodel.load():
                ikmodel.generate(iktype=iktype,freeindices=freeindices,forceikbuild=True)
                ikmodel.save()
                ikmodel.setrobot()

            if ikmodel.ikfeasibility is not None:
                # nothing more to do than print the text
                print ikmodel.ikfeasibility # will repeat text if just generated
                return
            
            ikmodel.freeinc = ikmodel.getDefaultFreeIncrements(options.freeincrot,options.freeinctrans)
            solutionresults = []                
            cmd = 'DebugIK robot %s '%robot.GetName()
            cmd += 'numtests %d '%int(options.numiktests[len(freeindices)])
            res = ikfastproblem.SendCommand(cmd).split()
            numtested = int(res[0])
            numsuccessful = int(res[1])
            index = 2
            ikdof = 1+IkParameterization.GetNumberOfValues(iktype)
            assert(manip.GetIkSolver().GetNumFreeParameters() == len(freeindices))
            for iresults in range(3):
                num = int(res[index])
                index += 1
                samples = []
                for i in range(num):
                    ikparam = IkParameterization(' '.join(res[index:(index+ikdof)]))
                    index += ikdof
                    samples.append([ikparam,res[index:(index+len(freeindices))]])
                    index += len(freeindices)
                solutionresults.append(samples)
            successrate = float(numsuccessful)/numtested
            nosolutions = float(len(solutionresults[1]))/numtested
            resultsstr = ikfastproblem.SendCommand('PerfTiming num %d %s'%(options.perftests,ikmodel.getfilename(True)))
            results = [numpy.double(s)*1e-6 for s in resultsstr.split()]
            jointnames = ', '.join(robot.GetJointFromDOFIndex(dof).GetName() for dof in ikmodel.manip.GetArmIndices())
            print 'ikfast version %s'%ikfast.__version__
            print 'free joint increment: %s'%ikmodel.freeinc
            print 'manipulator: %s %s [%s]'%(ikmodel.manip.GetBase().GetName(),ikmodel.manip.GetEndEffector().GetName(),jointnames)
            lower,upper = robot.GetDOFLimits(ikmodel.manip.GetArmIndices())
            print 'lower limits: '+' '.join(str(f) for f in lower)
            print 'upper limits: '+' '.join(str(f) for f in upper)
            if len(solutionresults[0])>0 or len(solutionresults[1])>0:
                print '\n\nThe following IK parameterizations are when link %s is at the origin, the last %d values are the normalized free variables [%s].\n'%(ikmodel.manip.GetBase().GetName(),len(freeindices),str(freeindicesstr))
            for isol in range(2):
                if len(solutionresults[isol]) == 0:
                    continue
                if isol == 0:
                    print 'Examples of Wrong Solutions:\n\n',
                else:
                    print 'Examples of No Solutions:\n\n'
                rows = []
                numprint = min(10,len(solutionresults[isol]))
                for index in numpy.random.permutation(len(solutionresults[isol]))[0:numprint]:
                    ikparam,freevalues = solutionresults[isol][index]
                    ikparamvalues = [str(f) for f in ikparam.GetTransform6D()[0:3,0:4].flatten()]
                    rows.append(ikparamvalues+freevalues)
                colwidths = [max([len(row[i]) for row in rows]) for i in range(len(rows[0]))]
                for i,row in enumerate(rows):
                    print ' '.join([row[j].ljust(colwidths[j]) for j in range(len(colwidths))])
            # jenkins plot measurement data
            print measurement('compile-time (s)', '%.3f'%ikmodel.statistics.get('generationtime',-1))
            print measurement('test success (%d/%d)'%(numsuccessful, numtested), '%.4f'%successrate)
            print measurement('test wrong solutions (%d/%d)'%(len(solutionresults[0]),numtested),'%.4f'%(float(len(solutionresults[0]))/numtested))
            print measurement('test no solutions (%d/%d)'%(len(solutionresults[1]),numtested), '%.4f'%nosolutions)
            print measurement('test missing solutions (%d/%d)'%(len(solutionresults[2]),numtested), '%.4f'%(float(len(solutionresults[2]))/numtested))
            print measurement('run-time mean (s)','%.6f'%numpy.mean(results))
            print measurement('run-time median (s)','%.6f'%numpy.median(results))
            print measurement('run-time min (s)','%.6f'%numpy.min(results))
            print measurement('run-time max (s)','%.6f'%numpy.max(results))
            # jenkins perfpublisher data
            print '<targets><target>%s</target></targets>'%iktypestr
            compiletime = ikmodel.statistics.get('generationtime',None)
            if compiletime is not None:
                print '<compiletime unit="s" mesure="%s" isRelevant="true" />'%compiletime
            #print '<performance unit="GFLOPs" mesure="%s" isRelevant="true" />'
            print '<executiontime unit="s" mesure="%s" isRelevant="true" />'%numpy.mean(results)
            #globalstats.put([numtested,numsuccessful,solutionresults])
            assert(len(solutionresults[0])==0)
            assert(successrate > options.minimumsuccess)
            assert(nosolutions < options.maximumnosolutions)
        except ikfast.IKFastSolver.IKFeasibilityError,e:
            # this is expected, and is normal operation, have to notify
            print e

class RunRobotStats:
    __name__='RunRobotStats'
    def __call__(self,*args):
        try:
            description = []
            setup_robotstats()
            robotstats(description,*args)
        finally:
            if len(description) > 0:
                self.description = description[0]
            teardown_robotstats()

def test_robots():
    RaveInitialize(load_all_plugins=False)
    RaveSetDebugLevel(DebugLevel.Error) # set to error in order to avoid expected plugin loading errosr
    envlocal=Environment()
    envlocal.StopSimulation()
    try:
        index = 0
        for robotfilename in options.robotfilenames:
            envlocal.Reset()
            robot = envlocal.ReadRobotXMLFile(robotfilename,{'skipgeometry':'1'})
            envlocal.AddRobot(robot)
            for iktype in options.iktypes:
                expecteddof = IkParameterization.GetDOF(iktype)
                for manip in robot.GetManipulators():
                    armdof = len(manip.GetArmIndices())
                    if armdof >= expecteddof and armdof <= expecteddof+options.maxfreejoints:
                        for freeindices in combinations(manip.GetArmIndices(),armdof-expecteddof):
                            yield RunRobotStats(),robotfilename, manip.GetName(), str(iktype),freeindices
    finally:
        envlocal.Destroy()
        # for some reason the plugindatabase _threadPluginLoader thread is on a different process
        # than the main threading waiting for it to finish, so it is necessary to call RaveDestroy
        RaveDestroy()

from noseplugins import multiprocess, xunitmultiprocess, capture, callableclass, jenkinsperfpublisher
from nose.plugins.cover import Coverage

if __name__ == "__main__":
    import test_ikfast
    parser = OptionParser(description='ikfast unit tests')
    parser.add_option('--robots', action='store', type='string', dest='robots',default='basic',
                      help='Robot groups to test, these are predetermined. type * for all robots. (default=%default)')
    parser.add_option('-j', action='store', type='int', dest='numprocesses',default='4',
                      help='Number of processors to run this in (default=%default).')
    parser.add_option('--timeout','-t', action='store', type='float', dest='timeout',default='600',
                      help='Timeout for each ikfast run, this includes time for generation and performance measurement. (default=%default)')
    parser.add_option('--perftests', action='store', type='int', dest='perftests',default='7000',
                      help='Number of tests to determine performance of generated IK. Performance is only computed if there are no wrong solutions. (default=%default)')
    parser.add_option('--numiktests', action='store', type='string', dest='numiktests',default='10000,7000,200',
                      help='Number of tests for testing the generated IK for correctness. Because test times increase exponentially with number of free joints, the iktests is an array of values indexec by the number of free joints. (default=%default)')
    parser.add_option('--debug','-d', action='store', type='int',dest='debug',default=logging.INFO,
                      help='Debug level for python nose (smaller values allow more text).')
    parser.add_option('--maxfreejoints',action='store',type='int',dest='maxfreejoints',default=2,
                      help='max free joints to allow, 3 or more will take too long to evaluate, and most likely will never be used in real life (default=%default)')
    parser.add_option('--iktypes',action='store',type='string',dest='iktypes',default='Transform6D',
                      help='IK types to test for. Can be a comma separated list of the specific names or * for all (default=%default)')
    parser.add_option('--freeincrot',action='store',type='float',dest='freeincrot',default='0.1',
                      help='increment of revolute free joints (default=%default)')
    parser.add_option('--freeinctrans',action='store',type='float',dest='freeinctrans',default='0.01',
                      help='percentage increment of the free joints, this should be scaled with robot size (default=%default)')
    parser.add_option('--minimumsuccess',action='store',type='float',dest='minimumsuccess',default='0.5',
                      help='Minimum success rate required to count test as passing (default=%default)')
    parser.add_option('--maximumnosolutions',action='store',type='float',dest='maximumnosolutions',default='0.5',
                      help='Maximum no-solutions rate allowed before test is decalred as a failure. In other words, if IK never finds anything, it is useless. (default=%default)')
    (options, args) = parser.parse_args()
    
    if options.iktypes == '*':
        options.iktypes = [iktype for value,iktype in IkParameterization.Type.values.iteritems()]
    else:
        iktypes = []
        for iktype in options.iktypes.split(','):
            for value,type in IkParameterization.Type.values.iteritems():
                if type.name.lower() == iktype.lower():
                    iktypes.append(type)
                    break
        options.iktypes = iktypes
    if options.robots == 'basic':
        options.robotfilenames = ['robots/unimation-pumaarm.zae','robots/barrettwam.robot.xml']
    elif options.robots == 'pr2':
        options.robotfilenames = ['robots/pr2-beta-static.zae']
    elif options.robots == '*':
        options.robotfilenames = ['robots/unimation-pumaarm.zae','robots/barrettwam.robot.xml','robots/pr2-beta-static.zae','robots/neuronics-katana.zae','robots/mitsubishi-pa10.zae','robots/schunk-lwa3.zae','robots/darpa-arm.zae','robots/exactdynamics-manusarmleft.zae','robots/kuka-kr5-r650.zae','robots/kuka-kr5-r850.zae']
    elif options.robots == 'random':
        options.robotfilenames = None
    else:
        options.robotfilenames = options.robots.split(',')
    options.numiktests = [int(s) for s in options.numiktests.split(',')]
    test_ikfast.options = options

    format = logging.Formatter('%(name)s: %(levelname)s: %(message)s')
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(format)
    multiprocess.log.addHandler(handler)
    multiprocess.log.setLevel(options.debug)

    header = '"name=\"%s robots\" package=\"%s\""'%(options.robots,ikfast.__name__)
    jenkins_header = '"name=\"%s\" categ=\"%s\"'%(options.robots,ikfast.__name__)
    prog=nose.core.TestProgram(argv=['nosetests','-v','--with-xunitmp','--xunit-file=test_ikfast.xml','--xunit-header=%s'%header,'--with-jenkinsperf','--jenkinsperf-file=jenkins_ikfast.xml', '--jenkinsperf-header=%s'%jenkins_header,'--processes=%d'%options.numprocesses,'--process-timeout=%f'%options.timeout,'--process-restartworker','--with-callableclass','test_ikfast.py'],plugins=[capture.Capture(),multiprocess.MultiProcess(),xunitmultiprocess.Xunitmp(),callableclass.CallableClass(), jenkinsperfpublisher.JenkinsPerfPublisher()],exit=False)
    # save the queue to file
#     f = open('stats.xml','w')
#     while not test_ikfast.globalstats.empty():
#         f.write(test_ikfast.globalstats.get())
