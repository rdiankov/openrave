#!/usr/bin/env python
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
"""
See :ref:`ikfast-testing`_

test_ikfast.py is being synchronized with the ikfast version, so don't try to regenerate if ikfast version hasn't changed. However, the module will use the cached IK data if possible.

"""
from common_test_openrave import *
from openravepy import ikfast
from optparse import OptionParser

import time, sys, logging, multiprocessing
#from nose.plugins import multiprocess
from noseplugins import multiprocess,xunitmultiprocess, capture, callableclass

import cPickle as pickle

_multiprocess_can_split_ = True

#global variables
globalmanager = multiprocessing.Manager()
globalstats = globalmanager.list() # used for gathering statistics
options = None
env=None
ikfastproblem=None

def setup_robotstats():
    global env,ikfastproblem
    # just load the plugin we'll be using
    RaveInitialize(load_all_plugins=False)
    success = RaveLoadPlugin('basesamplers')
    success = RaveLoadPlugin('ikfastsolvers')
    assert(success)
    RaveSetDebugLevel(DebugLevel.Error) # set to error in order to avoid expected plugin loading errors
    #format = logging.Formatter('%(name)s: %(message)s')
    #handler = logging.StreamHandler(sys.stderr)
    #handler.setFormatter(format)
    #databases.inversekinematics.log.addHandler(handler)
    #databases.inversekinematics.log.setLevel(logging.ERROR)
    #ikfast.log.addHandler(handler)
    #ikfast.log.setLevel(logging.ERROR)
    env=Environment()
    env.StopSimulation()
    ikfastproblem = RaveCreateProblem(env,'ikfast')
    assert(ikfastproblem is not None)
    env.LoadProblem(ikfastproblem,'')

def teardown_robotstats():
    global env,ikfastproblem
    if env is not None:
        env.Remove(ikfastproblem)
        env.Destroy()
    RaveDestroy()

def measurement(name,value):
    return '<measurement><name>%s</name><value>%s</value></measurement>'%(name,value)

@nose.with_setup(setup_robotstats, teardown_robotstats)
def robotstats(description,robotfilename,manipname, iktypestr,freeindices):
    global env, ikfastproblem, globalstats
    iktype = None
    for value,type in IkParameterization.Type.values.iteritems():
        if type.name == iktypestr:
            iktype = type
            break
    with env:
        robot = env.ReadRobotURI(robotfilename,{'skipgeometry':'1'})
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

            compiletime = ikmodel.statistics.get('generationtime',0)
            if ikmodel.ikfeasibility is not None:
                # nothing more to do than print the text
                print ikmodel.ikfeasibility # will repeat text if just generated
                globalstats.append([robotfilename,manip.GetName(),iktypestr,freeindices,description,None,None,None,None,None])
                return

            ikmodel.freeinc = ikmodel.getDefaultFreeIncrements(options.freeincrot,options.freeinctrans)
            solutionresults = []                
            cmd = 'DebugIK robot %s '%robot.GetName()
            cmd += 'numtests %d '%int(options.numiktests[len(freeindices)])
            cmd += 'threshold %f '%options.errorthreshold
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
            wrongrate = float(len(solutionresults[0]))/numtested
            resultsstr = ikfastproblem.SendCommand('PerfTiming num 10000 maxtime %f %s'%(options.perftime,ikmodel.getfilename(True)))
            results = [numpy.double(s)*1e-9 for s in resultsstr.split()]
            jointnames = ', '.join(robot.GetJointFromDOFIndex(dof).GetName() for dof in ikmodel.manip.GetArmIndices())
        except ikfast.IKFastSolver.IKFeasibilityError,e:
            # this is expected, and is normal operation, have to notify
            globalstats.append([robotfilename,manip.GetName(),iktypestr,freeindices,description,None,None,None,None,None])
            print e
            return

        print 'ikfast version: %s'%ikfast.__version__
        #print 'SECTION Robot Information'
        print 'robot: %s, manipulator: '%(robotfilename)
        print 'free joint increment: %s'%ikmodel.freeinc
        print 'manipulator %s: %s %s [%s]'%(manipname, ikmodel.manip.GetBase().GetName(),ikmodel.manip.GetEndEffector().GetName(),jointnames)
        lower,upper = robot.GetDOFLimits(ikmodel.manip.GetArmIndices())
        print 'lower limits: '+' '.join(str(f) for f in lower)
        print 'upper limits: '+' '.join(str(f) for f in upper)
        if len(solutionresults[0])>0 or len(solutionresults[1])>0:
            #print '\nSECTION Problematic IK'
            print '\n\nThe following IK parameterizations are when link %s is at the origin, the last %d values are the normalized free variables [%s].\n'%(ikmodel.manip.GetBase().GetName(),len(freeindices),str(freeindicesstr))
        for isol in range(2):
            if len(solutionresults[isol]) == 0:
                continue
            prefix = ''
            if isol == 0:
                #prefix = 'ERROR '
                print '\n\nExamples of Wrong Solutions:\n',
            else:
                #prefix = 'WARN '
                print '\n\nExamples of No Solutions:\n'
            rows = []
            numprint = min(10,len(solutionresults[isol]))
            for index in numpy.random.permutation(len(solutionresults[isol]))[0:numprint]:
                ikparam,freevalues = solutionresults[isol][index]
                ikparamvalues = [str(f) for f in ikparam.GetTransform6D()[0:3,0:4].flatten()]
                rows.append(ikparamvalues+freevalues)
            colwidths = [max([len(row[i]) for row in rows]) for i in range(len(rows[0]))]
            for i,row in enumerate(rows):
                print prefix + ' '.join([row[j].ljust(colwidths[j]) for j in range(len(colwidths))])
        # jenkins plot measurement data
        print measurement('compile-time (s)', '%.3f'%compiletime)
        print measurement('success', '%.4f'%successrate)
        print measurement('wrong solutions','%.4f'%wrongrate)
        print measurement('no solutions', '%.4f'%nosolutions)
        print measurement('missing solutions', '%.4f'%(float(len(solutionresults[2]))/numtested))
        print measurement('number tests',str(numtested))
        print measurement('run-time mean (s)','%.6f'%numpy.mean(results))
        print measurement('run-time max (s)','%.6f'%numpy.max(results))
        globalstats.append([robotfilename,manip.GetName(),iktypestr,freeindices,description,ikmodel.getsourcefilename(read=True),numpy.mean(results),numpy.max(results),successrate,wrongrate])
        assert(len(solutionresults[0])==0)
        assert(successrate > options.minimumsuccess)
        assert(nosolutions < options.maximumnosolutions)


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

def parseoptions(args=None):
    parser = OptionParser(description='ikfast unit tests')
    parser.add_option('--robots', action='store', type='string', dest='robots',default='basic',
                      help='Robot groups to test, these are predetermined. type * for all default robots. The list can be comma separated to specify multiple groups/filenames. (default=%default)')
    parser.add_option('-j', action='store', type='int', dest='numprocesses',default='4',
                      help='Number of processors to run this in (default=%default).')
    parser.add_option('--timeout','-t', action='store', type='float', dest='timeout',default='600',
                      help='Timeout for each ikfast run, this includes time for generation and performance measurement. (default=%default)')
    parser.add_option('--perftime', action='store', type='float', dest='perftime',default='20',
                      help='Time (s) to run performance tests of generated IK. Performance is only computed if there are no wrong solutions. (default=%default)')
    parser.add_option('--numiktests', action='store', type='string', dest='numiktests',default='1000,1000,100',
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
    parser.add_option('--errorthreshold',action='store',type='float',dest='errorthreshold',default='0.001',
                      help='The error threshold between ik parameterizations defining boundary of wrong solutions (default=%default)')
    parser.add_option('--minimumsuccess',action='store',type='float',dest='minimumsuccess',default='0.4',
                      help='Minimum success rate required to count test as passing (default=%default)')
    parser.add_option('--maximumnosolutions',action='store',type='float',dest='maximumnosolutions',default='0.6',
                      help='Maximum no-solutions rate allowed before test is decalred as a failure. In other words, if IK never finds anything, it is useless. (default=%default)')
    parser.add_option('--outputdir',action='store',type='string',dest='outputdir',default='rst',
                      help='Directory to output the RST files used to show off the ikfast results. The root file in this directory is index.rst. (default=%default)')
    parser.add_option('--jenkinsbuild',action='store',type='string',dest='jenkinsbuild_url',default='http://www.openrave.org/testing/job/openrave/lastSuccessfulBuild/',
                      help='URL for the test results published by jenkins. This URL will be used to create links from the robot pages to the test page. (default=%default)')
    (options, parseargs) = parser.parse_args(args=args)
    
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
    robots = options.robots.split(',')
    options.robotfilenames = []
    for robot in robots:
        if robot == 'basic':
            # only robots that are in defualt openrave repository
            options.robotfilenames += ['robots/pumaarm.zae','robots/barrettwam.robot.xml','robots/kawada-hironx.zae','ikfastrobots/fail1.robot.xml','robots/pr2-beta-static.zae','robots/kuka-youbot.zae']
        elif robot == 'pr2':
            options.robotfilenames += ['robots/pr2-beta-static.zae']
        elif robot == '*':
            options.robotfilenames += ['robots/unimation-pumaarm.zae','robots/barrett-wam.zae','robots/pr2-beta-static.zae','robots/neuronics-katana.zae','robots/mitsubishi-pa10.zae','robots/schunk-lwa3.zae','robots/darpa-arm.zae','robots/exactdynamics-manusarmleft.zae','robots/kuka-kr5-r650.zae','robots/kuka-kr5-r850.zae','robots/kuka-kr30l16.zae','robots/tridof.robot.xml','robots/barrett-wam4.zae','robots/kawada-hironx.zae','ikfastrobots/fail1.robot.xml','robots/kuka-youbot.zae']
        elif options.robots == 'random':
            options.robotfilenames.append('random')
        else:
            options.robotfilenames.append(robot)
    options.numiktests = [int(s) for s in options.numiktests.split(',')]
    return options

def test_robot_ikfast():
    global options
    if options is None:
        return
        #options = parseoptions([])
    RaveInitialize(load_all_plugins=False)
    RaveSetDebugLevel(DebugLevel.Error) # set to error in order to avoid expected plugin loading errosr
    envlocal=Environment()
    envlocal.StopSimulation()
    try:
        workitems = []
        index = 0
        for robotfilename in options.robotfilenames:
            envlocal.Reset()
            robot = envlocal.ReadRobotURI(robotfilename,{'skipgeometry':'1'})
            envlocal.AddRobot(robot)
            for iktype in options.iktypes:
                expecteddof = IkParameterization.GetDOF(iktype)
                for manip in robot.GetManipulators():
                    armdof = len(manip.GetArmIndices())
                    if armdof >= expecteddof and armdof <= expecteddof+options.maxfreejoints:
                        for freeindices in combinations(manip.GetArmIndices(),armdof-expecteddof):
                            workitems.append((RunRobotStats(), robotfilename, manip.GetName(), str(iktype),freeindices))
    finally:
        envlocal.Destroy()
        # for some reason the plugindatabase _threadPluginLoader thread is on a different process
        # than the main threading waiting for it to finish, so it is necessary to call RaveDestroy
        RaveDestroy()

    for work in workitems:
        yield work
    
if __name__ == "__main__":
    import test_ikfast
    options = parseoptions()
    test_ikfast.options = options

    format = logging.Formatter('%(name)s: %(levelname)s %(message)s')
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(format)
    multiprocess.log.addHandler(handler)
    multiprocess.log.setLevel(options.debug)
    #ikfast.log.addHandler(handler)
    #ikfast.log.setLevel(options.debug)
    
    multiprocess._instantiate_plugins = [capture.Capture, xunitmultiprocess.Xunitmp, callableclass.CallableClass]

    header = 'name=\"%s robots\" package=\"%s\"'%(options.robots,ikfast.__name__)
    argv=['nosetests','-v','--with-xunitmp','--xunit-file=test_ikfast.xml','--xunit-header=%s'%header,'--processes=%d'%options.numprocesses,'--process-timeout=%f'%options.timeout,'--process-restartworker','--with-callableclass','test_ikfast.py']
    plugins=[capture.Capture(),multiprocess.MultiProcess(),xunitmultiprocess.Xunitmp(),callableclass.CallableClass()]
    prog=nose.core.TestProgram(argv=argv,plugins=plugins,exit=False)
    print 'processing the global stats'
    stats = []
    while len(test_ikfast.globalstats) > 0:
        stat = test_ikfast.globalstats.pop(0)
        if not stat[5] is None:
            stat.append(open(stat[5],'r').read())
        else:
            stat.append(None)
        stats.append(stat)
    pickle.dump([stats,options],open('ikfaststats.pp','w'))
