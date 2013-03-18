# -*- coding: utf-8 -*-
# Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
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
from common_test_openrave import *

class TestIkFast(EnvironmentSetup):
    freeincrot=0.1
    """increment of revolute free joints"""
    
    freeinctrans = 0.01
    """percentage increment of the free joints, this should be scaled with robot size"""

    numiktests = [200,100,100]
    """Number of tests for testing the generated IK for correctness. Because test times increase exponentially with number of free joints, the iktests is an array of values indexec by the number of free joints."""
    
    errorthreshold=1e-6
    """The error threshold between ik parameterizations defining boundary of wrong solutions"""
    
    perftime = 10
    """Time (s) to run performance tests of generated IK. Performance is only computed if there are no wrong solutions."""
    
    minimumsuccess = 0.9
    """Minimum success rate required to count test as passing"""
    
    maximumnosolutions = 0.6
    """Maximum no-solutions rate allowed before test is decalred as a failure. In other words, if IK never finds anything, it is useless."""
    
    def setup(self):
        EnvironmentSetup.setup(self)
        self.ikfastproblem = RaveCreateProblem(self.env,'ikfast')
        assert(self.ikfastproblem is not None)
        self.env.Add(self.ikfastproblem)
    def teardown(self):
        self.env.Remove(self.ikfastproblem)
        EnvironmentSetup.teardown(self)

    def PrintMeasurement(self, name,value):
        self.log.info('<measurement><name>%s</name><value>%s</value></measurement>'%(name,value))

    def RunIkFast(self, robotfilename,manipname, iktype, freeindices=None, expectedruntime=None, ikfeasibility=True, minimumsuccess=None):
        """
        :param ikfeasibility: if True, then ik is feasible
        """
        env=self.env
        if minimumsuccess is None:
            minimumsuccess = self.minimumsuccess
        with env:
            robot = env.ReadRobotURI(robotfilename,{'skipgeometry':'1'})
            env.Add(robot)
            manip = robot.SetActiveManipulator(manipname)
            # set base to identity to avoid complications when reporting errors, testing that IK works under transformations is a different test
            robot.SetTransform(numpy.dot(numpy.linalg.inv(manip.GetBase().GetTransform()),robot.GetTransform()))
            manip=robot.SetActiveManipulator(manipname)
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=iktype,freeindices=freeindices)
            if ikmodel.freeindices is not None:
                freeindicesstr = ', '.join(robot.GetJointFromDOFIndex(dof).GetName()+'('+str(dof)+')' for dof in ikmodel.freeindices)
            else:
                freeindicesstr = ''
            
            # remove any default ik solver for the manipulator, it can get in the way loading
            ikmodel.manip.SetIKSolver(None)
            ikfasttime = None
            if not ikmodel.load():
                ikmodel.generate(iktype=iktype,forceikbuild=True)
                ikmodel.save()
                ikmodel.setrobot()
                
            compiletime = ikmodel.statistics.get('generationtime',0)
            if ikfeasibility and ikmodel.ikfeasibility is not None:
                raise ValueError('ik should be feasible: %s'%ikmodel.ikfeasibility)
            
            if not ikfeasibility and ikmodel.ikfeasibility is None:
                raise ValueError('ik should not be feasible')
            
            ikmodel.freeinc = ikmodel.getDefaultFreeIncrements(self.freeincrot,self.freeinctrans)
            numfree = manip.GetIkSolver().GetNumFreeParameters()
            if ikmodel.freeindices is not None:
                assert(numfree == len(ikmodel.freeindices))
            solutionresults = [[],[],[]]
            
            numtested = 0
            numsuccessful = 0
            numiktests = int(self.numiktests[numfree])
            chunksize = 100 # have to split into chunks because of timeouts
            cmd = 'DebugIK robot %s threshold %f '%(robot.GetName(), self.errorthreshold)
            for ichunk in range((numiktests+chunksize-1)/chunksize):
                res = self.ikfastproblem.SendCommand(cmd+'numtests %d '%chunksize).split()
                numtested += int(res[0])
                numsuccessful += int(res[1])
                index = 2
                ikdof = 1+IkParameterization.GetNumberOfValuesFromType(iktype)
                for iresults in range(3):
                    num = int(res[index])
                    index += 1
                    for i in range(num):
                        ikparam = IkParameterization(' '.join(res[index:(index+ikdof)]))
                        index += ikdof
                        solutionresults[iresults].append([ikparam,res[index:(index+numfree)]])
                        index += numfree
            successrate = float(numsuccessful)/numtested
            nosolutions = float(len(solutionresults[1]))/numtested
            wrongrate = float(len(solutionresults[0]))/numtested
            resultsstr = self.ikfastproblem.SendCommand('PerfTiming num 5000 maxtime %f %s'%(self.perftime,ikmodel.getfilename(True)))
            results = [numpy.double(s)*1e-9 for s in resultsstr.split()]
            jointnames = ', '.join(robot.GetJointFromDOFIndex(dof).GetName() for dof in ikmodel.manip.GetArmIndices())

            self.log.info('ikfast version: %s, file: %s', ikmodel.ikfast.__version__, ikmodel.getsourcefilename(read=True))
            #print 'SECTION Robot Information'
            self.log.info('robot: %s, manipulator: '%(robotfilename))
            self.log.info('free joint increment: %s'%ikmodel.freeinc)
            self.log.info('manipulator %s: %s %s [%s]'%(manipname, ikmodel.manip.GetBase().GetName(),ikmodel.manip.GetEndEffector().GetName(),jointnames))
            lower,upper = robot.GetDOFLimits(ikmodel.manip.GetArmIndices())
            self.log.info('lower limits: '+' '.join(str(f) for f in lower))
            self.log.info('upper limits: '+' '.join(str(f) for f in upper))
            if len(solutionresults[0])>0 or len(solutionresults[1])>0:
                #print '\nSECTION Problematic IK'
                self.log.info('\n\nThe following IK parameterizations are when link %s is at the origin, the last %d values are the normalized free variables [%s].\n'%(ikmodel.manip.GetBase().GetName(),numfree,str(freeindicesstr)))
            for isol in range(2):
                if len(solutionresults[isol]) == 0:
                    continue
                prefix = ''
                if isol == 0:
                    #prefix = 'ERROR '
                    self.log.info('\n\nExamples of Wrong Solutions:\n')
                else:
                    #prefix = 'WARN '
                    self.log.info('\n\nExamples of No Solutions:\n')
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
            self.PrintMeasurement('compile-time (s)', '%.3f'%compiletime)
            self.PrintMeasurement('success', '%.4f'%successrate)
            self.PrintMeasurement('wrong solutions','%.4f'%wrongrate)
            self.PrintMeasurement('no solutions', '%.4f'%nosolutions)
            self.PrintMeasurement('missing solutions', '%.4f'%(float(len(solutionresults[2]))/numtested))
            self.PrintMeasurement('number tests',str(numtested))
            meanresults = numpy.mean(results)
            maxresults = numpy.max(results)
            self.PrintMeasurement('run-time mean (s)','%.6f'%meanresults)
            self.PrintMeasurement('run-time max (s)','%.6f'%maxresults)
            assert(len(solutionresults[0])==0)
            assert(successrate >= minimumsuccess)
            assert(nosolutions <= self.maximumnosolutions)
            assert(wrongrate == 0 )
            if expectedruntime is not None:
                assert(meanresults<=expectedruntime)

    def test_testik0(self):
        self.RunIkFast('ikfastrobots/testik0.zae','arm', IkParameterizationType.Transform6D, expectedruntime=45e-6,minimumsuccess=1)
    def test_fail1(self):
        self.RunIkFast('ikfastrobots/fail1.dae','arm', IkParameterizationType.Transform6D, expectedruntime=450e-6,minimumsuccess=0.98)
    def test_fail2(self):
        self.RunIkFast('ikfastrobots/fail2.robot.xml','arm', IkParameterizationType.Transform6D, minimumsuccess=0.99)
    def test_fail3(self):
        self.RunIkFast('ikfastrobots/fail3.robot.xml','FLLeg', IkParameterizationType.Transform6D, minimumsuccess=0.97)
#     def test_fail4(self):
#         # should finish, but takes too long to complete...
#         self.RunIkFast('ikfastrobots/fail4.dae','head', IkParameterizationType.Transform6D, minimumsuccess=0.99)
    def test_fail5_4d_Flange(self):
        self.RunIkFast('ikfastrobots/fail5_4d.zae','Flange', IkParameterizationType.TranslationXAxisAngleZNorm4D, minimumsuccess=1)
    def test_fail5_4d_0(self):
        self.RunIkFast('ikfastrobots/fail5_4d.zae','0', IkParameterizationType.TranslationXAxisAngleZNorm4D, minimumsuccess=1)
    def test_kawada_hironx_left(self):
        self.RunIkFast('robots/kawada-hironx.zae','leftarm', IkParameterizationType.Transform6D, expectedruntime=0.0016, minimumsuccess=0.989)
    def test_kawada_hironx_lefttorso(self):
        self.RunIkFast('robots/kawada-hironx.zae','leftarm_torso', IkParameterizationType.Transform6D, [0], expectedruntime=0.0016, minimumsuccess=0.989)
