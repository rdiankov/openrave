#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Masaho Ishida, Rosen Diankov <rosen.diankov@gmail.com>
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
"""updates the cached ikfast files in the plugins/ikfastsolvers directory

To update openrave cached ik files run:

.. code-block:: bash
 
  python updateikfiles.py --forcegenerate --destdir=../plugins/ikfastsolvers --testnum=100
  
"""

from numpy import *
from itertools import *
import time,platform,os,sys
import multiprocessing
from optparse import OptionParser
import logging
import pickle
from openravepy import *
from openravepy import ikfast

databases.inversekinematics.log.setLevel(logging.ERROR)

def updateik(robotfilename,manipname,iktype,destfilename=None,freeindices=None,results=None, do_test=True, forcegenerate=False, testnum=5000, delta='0.01'):
    print robotfilename, manipname, iktype, destfilename
    RaveInitialize()
    env=Environment()
    try:
        with env:
            robot = env.ReadRobotXMLFile(robotfilename,{'skipgeometry':'1'})
            env.AddRobot(robot)
            if manipname is not None:
                manip = robot.SetActiveManipulator(manipname)
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=iktype,freeindices=freeindices)
            ikmodel.manip.SetIKSolver(None)
            ikmodel.ikfast.log.setLevel(logging.ERROR)
            if forcegenerate or not ikmodel.load():
                ikmodel.autogenerate()
            if freeindices is not None:
                ikmodel.setrobot([float(delta)]*len(freeindices))
            if do_test and testnum > 0:
                successrate, wrongrate = ikmodel.testik(str(testnum))
                if results is not None:
                    results[0].value = successrate
                    results[1].value = wrongrate
                    results[2].value = mean(ikmodel.perftiming(100000))
            if destfilename is not None:
                robotid = os.path.split(destfilename)[1][:-4]
                code = """#define IKFAST_NAMESPACE %s
#include "plugindefs.h"

"""%robotid
                sourcefilename = ikmodel.getsourcefilename(True)
                if len(sourcefilename) == 0:
                    raise ValueError(u'robot %s manip %s cannot generate ik %s'%(robot.GetName(),manip.GetName(),iktype))
                
                code += open(sourcefilename,'r').read()
                code += """
#include "plugindefs.h" 
namespace IKFAST_NAMESPACE {
IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr penv, std::istream& sinput, const std::vector<dReal>& vfreeinc) {
    boost::shared_ptr<ikfast::IkFastFunctions<IkReal> > ikfunctions(new ikfast::IkFastFunctions<IkReal>());
    ikfunctions->_ComputeIk = IKFAST_NAMESPACE::ComputeIk;
    ikfunctions->_ComputeFk = IKFAST_NAMESPACE::ComputeFk;
    ikfunctions->_GetNumFreeParameters = IKFAST_NAMESPACE::GetNumFreeParameters;
    ikfunctions->_GetFreeParameters = IKFAST_NAMESPACE::GetFreeParameters;
    ikfunctions->_GetNumJoints = IKFAST_NAMESPACE::GetNumJoints;
    ikfunctions->_GetIkRealSize = IKFAST_NAMESPACE::GetIkRealSize;
    ikfunctions->_GetIkFastVersion = IKFAST_NAMESPACE::GetIkFastVersion;
    ikfunctions->_GetIkType = IKFAST_NAMESPACE::GetIkType;
    ikfunctions->_GetKinematicsHash = IKFAST_NAMESPACE::GetKinematicsHash;
    return CreateIkFastSolver(penv,sinput,ikfunctions,vfreeinc);
}
} // end namespace
"""
                print 'writing %s'%destfilename
                open(destfilename,'w').write(code)
    finally:
        print "destroying environment"
        env.Destroy()


def get_freeindies_combinations(robot_file, manip_name):
    print robot_file, manip_name
    if manip_name == None:
        return [None]
    RaveInitialize()
    env=Environment()
    env.Load(robot_file)
    robot=env.GetRobots()[0]
    manip=robot.SetActiveManipulator(manip_name)
    joints =  manip.GetArmIndices()
    if len(joints) <= 6:
        freeindies_combination=[None]
    else:
        freeindies_combination = list(combinations(joints, len(joints)-6))
    RaveDestroy()
    return freeindies_combination


if __name__ == "__main__":
    parser = OptionParser()
    usage = "usage: %prog [options] <arg>"
    parser = OptionParser(usage)
    parser.add_option("-n", "--numthreads", dest="numthreads", default=multiprocessing.cpu_count(), help='the number of using core')
    parser.add_option("-l", "--timelimit", dest="time_limit", default='800', help='time to stop test ik.')
    parser.add_option("-d", "--destdir", dest="destdir", default=None,
                      help='destination directory to save ik file results ')
    parser.add_option("-r", "--robot", dest="robot", default=None, help='Robot file path')
    parser.add_option("-m", "--manip", dest="manip", default=None, help='Manipulator')
    parser.add_option("-t", "--type", dest="type", default=IkParameterization.Type.Transform6D, help='Ik type')
    parser.add_option("--testnum", dest="testnum", type='int', default=5000, help='the number of ik test sets')
    parser.add_option('--forcegenerate', dest='forcegenerate',action='store_true',default=False,help='if true will always force generation of ik')
    parser.add_option("-e", "--delta", dest="delta", default='0.01', help='the step of free indies angle')
    (options, args) = parser.parse_args()

    numthreads=int(options.numthreads)
    time_limit=int(options.time_limit) #seconds
    robot_manip_type_fouts = None
    if (not options.robot is None and options.manip is None) or (options.robot is None and not options.manip is None):
        print 'set robot and manip name'
        sys.exit (0)
    elif not options.robot is None and not options.manip is None:
        fout = os.path.splitext(os.path.basename(options.robot))[0]+'.cpp'
        robot_manip_type_fouts = [[options.robot, options.manip, options.type, True, fout]]

    # default files
    if robot_manip_type_fouts is None:
        robot_manip_type_fouts=[['robots/puma.robot.xml', None, IkParameterization.Type.Transform6D, False, 'ik_puma.cpp'],
                                ['robots/barrettwam.robot.xml', None, IkParameterization.Type.Transform6D, False, 'ik_barrettwam.cpp'],
                                ['robots/pa10schunk.robot.xml','arm',IkParameterization.Type.Transform6D, False, 'ik_pa10.cpp'],
                                ['robots/pr2-beta-static.zae','head',IkParameterization.Type.Lookat3D, False, 'ik_pr2_head.cpp'],
                                ['robots/pr2-beta-static.zae','head_torso',IkParameterization.Type.Lookat3D,False,'ik_pr2_head_torso.cpp'],
                                ['robots/pr2-beta-static.zae','leftarm',IkParameterization.Type.Transform6D,False,'ik_pr2_leftarm.cpp'],
                                ['robots/pr2-beta-static.zae','rightarm',IkParameterization.Type.Transform6D,False,'ik_pr2_rightarm.cpp'],
                                ['robots/pr2-beta-static.zae','leftarm_torso',IkParameterization.Type.Transform6D,False,'ik_pr2_leftarm_torso.cpp'],
                                ['robots/pr2-beta-static.zae','rightarm_torso',IkParameterization.Type.Transform6D,False,'ik_pr2_rightarm_torso.cpp'],
                                ['robots/schunk-lwa3.zae',None,IkParameterization.Type.Transform6D,False,'ik_schunk_lwa3.cpp'],
                                ['robots/neuronics-katana.zae','arm',IkParameterization.Type.TranslationDirection5D,False,'ik_katana5d.cpp'],
                                ['robots/neuronics-katana.zae','armgrasp',IkParameterization.Type.Translation3D,False,'ik_katana5d_trans.cpp']]


    # create all jobs/args
    args = []
    robotmanip_offsets = []
    offset = 0
    for robotfilename,manipname,iktype,testallindices,destfilename in robot_manip_type_fouts:
        if testallindices:
            freeindices_combs = get_freeindies_combinations(robotfilename,manipname)
        else:
            freeindices_combs = [None] # take best one
        robotmanip_offsets.append([offset,len(freeindices_combs)])
        offset += len(freeindices_combs)
        for freeindices in freeindices_combs:
            a={'robotfilename':robotfilename, 'manipname':manipname, 'iktype':iktype, 'freeindices':freeindices, 'testnum':options.testnum, 'delta':options.delta, 'forcegenerate':options.forcegenerate}
            if destfilename is not None and options.destdir is not None:
                a['destfilename'] = os.path.join(options.destdir, destfilename)
            args.append(a)

    finalresults=[None]*len(args)
    timer=0
    processes = []
    try:
        starttime=time.time()
        for i in range (len(args)):
            results = [multiprocessing.Value('f'),multiprocessing.Value('f'),multiprocessing.Value('f')]
            results[0].value = 0
            results[1].value = 0
            results[2].value = 0
            kwargs = dict(args[i])
            kwargs['results'] = results
            p = multiprocessing.Process(target=updateik, kwargs=kwargs)
            print 'start process ('+str(i)+'/'+str(args[i])+')'
            p.start()
            p.endtime = (time.time()-starttime)+time_limit
            p.index = i
            p.results = results
            p.kwargs = kwargs
            processes.append(p)

            # wait until thread finishes, or it times out
            waituntilend = len(args)==i+1
            while len(processes) >= numthreads or (waituntilend and len(processes)>0):
                terminateprocesses = []
                for p in processes:
                    if not p.is_alive():
                        finalresults[p.index] = [p.results[0].value,p.results[1].value,p.results[2].value]
                        print 'finished %s with %s rate'%(p.kwargs,finalresults[p.index])
                        processes.remove(p)
                    elif p.endtime < (time.time()-starttime):
                        terminateprocesses.append(p)

                for p in terminateprocesses:
                    p.terminate()
                    p.join()
                    processes.remove(p)

                if len(processes) >= numthreads or (waituntilend and len(processes)>0):
                    time.sleep(1)

        #if None or 0.0, failed. 
        print finalresults

    finally:
        for p in processes:
            p.terminate()
            p.join()

    saveresults = [[args[i], finalresults[i]] for i in range(len(finalresults)) if finalresults[i] is not None]
    pickle.dump(saveresults,open(os.path.join(options.destdir, 'results.pp'),'w'))
    print 'results: ',saveresults

    # select max success rate one in all free indies combinations.
    findices=[]
    for offset,numjobs in robotmanip_offsets:
        sorted_results=[[finalresults[offset+j],offset+j] for j in range(numjobs) if finalresults[offset+j] is not None]
        sorted_results.sort(key=lambda k: k[0][2])
        sorted_results.sort(key=lambda k: k[0][0],reverse=True)
        findex = None
        for [successrate,wrongrate,perftime],offset in sorted_results:
            if wrongrate <= 0.0 or wrongrate == None:
                findex = offset
                break
        if findex is None:
            raise ValueError('ik has failures: %r'%args[offset])
        
        findices.append(findex)

    for i in findices:
        try:
            args[i]['forcegenerate'] = False
            updateik(do_test=False,**args[i])
        except Exception,e:
            print e
            print 'error occured in writing file %s.'%args[i]

