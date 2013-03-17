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
import cPickle as pickle

class TestIkSolver(EnvironmentSetup):
    def test_customfilter(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        with env:
            robot.SetDOFValues(ones(robot.GetDOF()),range(robot.GetDOF()),checklimits=True)
            T = ikmodel.manip.GetTransform()
            sols = ikmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
            index = 2
            assert(len(sols)>0 and any([sol[index] > 0.2 for sol in sols]) and any([sol[index] < -0.2 for sol in sols]) and any([sol[index] > -0.2 and sol[index] < 0.2 for sol in sols]))

            def filter1(sol,manip,ikparam):
                return IkReturnAction.Success if sol[2] > -0.2 else IkReturnAction.Reject

            def filter1test(sol,manip,ikparam):
                assert(sol[2] < 0.2)
                return IkReturnAction.Success if sol[2] > -0.2 else IkReturnAction.Reject

            def filter2(sol,manip,ikparam):
                return IkReturnAction.Success if sol[2] < 0.2 else IkReturnAction.Reject

            def filter2test(sol,manip,ikparam):
                assert(sol[2] > -0.2)
                return IkReturnAction.Success if sol[2] < 0.2 else IkReturnAction.Reject

            handle1 = ikmodel.manip.GetIkSolver().RegisterCustomFilter(0,filter1)
            sols = ikmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
            assert(len(sols)>0 and all([sol[index] > -0.2 for sol in sols]))

            handle2 = ikmodel.manip.GetIkSolver().RegisterCustomFilter(-1,filter2test)
            sols = ikmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
            assert(len(sols) > 0 and all([sol[index] > -0.2 and sol[index] < 0.2 for sol in sols]))

            handle1.close()
            handle2.close()

            handle1 = ikmodel.manip.GetIkSolver().RegisterCustomFilter(-1,filter1test)
            handle2 = ikmodel.manip.GetIkSolver().RegisterCustomFilter(0,filter2)
            sols = ikmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
            assert(len(sols) > 0 and all([sol[index] > -0.2 and sol[index] < 0.2 for sol in sols]))
            
            handle1.close()
            handle2.close()
            sols = ikmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
            assert(len(sols)>0 and any([sol[index] > 0.2 for sol in sols]) and any([sol[index] < -0.2 for sol in sols]) and any([sol[index] > -0.2 and sol[index] < 0.2 for sol in sols]))

    def test_iksolutionjitter(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        with env:
            # set robot in collision
            robot.SetDOFValues([ -8.44575603e-02,   1.48528347e+00,  -5.09108824e-08, 6.48108822e-01,  -4.57571203e-09,  -1.04008750e-08, 7.26855048e-10,   5.50807826e-08,   5.50807826e-08, -1.90689327e-08,   0.00000000e+00])
            manip = robot.GetActiveManipulator()
            ikparam=manip.GetIkParameterization(IkParameterizationType.Transform6D)
            ikparamcopy = IkParameterization(ikparam)
            assert(ikparam.GetDOF()==6)
            assert(ikparam.GetDOF(IkParameterizationType.Transform6D)==6)
            assert(IkParameterization.GetDOFFromType(IkParameterizationType.Transform6D)==6)
            assert(IkParameterization.GetDOFFromType(ikparam.GetType())==6)
            assert(ikparam.GetNumberOfValues()==7)
            assert(ikparam.GetNumberOfValues(IkParameterizationType.Transform6D)==7)
            assert(IkParameterization.GetNumberOfValuesFromType(IkParameterizationType.Transform6D)==7)
            assert(IkParameterization.GetNumberOfValuesFromType(ikparam.GetType())==7)
            assert(manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions) is None)
            assert(manip.FindIKSolution(ikparam,0) is not None)
            sampler=planningutils.ManipulatorIKGoalSampler(robot.GetActiveManipulator(),[ikparam],nummaxsamples=20,nummaxtries=10,jitter=0)
            assert(sampler.Sample() is None)
            sampler=planningutils.ManipulatorIKGoalSampler(robot.GetActiveManipulator(),[ikparam],nummaxsamples=20,nummaxtries=10,jitter=0.03)
            assert(sampler.Sample() is not None)

    def test_jointlimitsfilter(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        def filtertest(sol,manip,ikparam):
            return IkReturnAction.Success

        handle1 = ikmodel.manip.GetIkSolver().RegisterCustomFilter(0,filtertest)

        orgvalues = robot.GetDOFValues()
        joint=robot.GetJointFromDOFIndex(ikmodel.manip.GetArmIndices()[-1])
        joint.SetLimits([-pi+1e-5],[pi-1e-5])
        robot.SetDOFValues([pi/4,pi/4,pi],[1,5,joint.GetDOFIndex()],False)
        assert(not env.CheckCollision(robot))
        T = ikmodel.manip.GetTransform()
        robot.SetDOFValues(orgvalues)
        sols=ikmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions|IkFilterOptions.IgnoreJointLimits)

        joint.SetLimits([-pi],[pi])
        sols=ikmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions|IkFilterOptions.IgnoreJointLimits)

    def test_returnactions(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        solver=ikmodel.manip.GetIkSolver()
        with env:
            robot.SetDOFValues([0.4,0.4],[1,3])
            ikparam = ikmodel.manip.GetIkParameterization(IkParameterizationType.Transform6D)
            RaveSetDebugLevel(DebugLevel.Verbose)
            solexpected = ikmodel.manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions)
            ikreturn = ikmodel.manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions,ikreturn=True)
            assert( transdist(solexpected, ikreturn.GetSolution()) <= g_epsilon )
            
            sols = ikmodel.manip.FindIKSolutions(ikparam,IkFilterOptions.CheckEnvCollisions)
            ikreturns = ikmodel.manip.FindIKSolutions(ikparam,IkFilterOptions.CheckEnvCollisions,ikreturn=True)
            assert(len(sols) == len(ikreturns))
            
            Tbaseinv = linalg.inv(ikmodel.manip.GetBase().GetTransform())
            ikparam2 = ikparam.Transform(Tbaseinv)
            ikreturn = solver.Solve(ikparam2,robot.GetDOFValues(ikmodel.manip.GetArmIndices()), IkFilterOptions.CheckEnvCollisions)
            assert( transdist(solexpected, ikreturn.GetSolution()) <= g_epsilon )
            assert(ikreturn.GetAction() == IkReturnAction.Success)
            
            ikreturn = solver.Solve(ikparam2,None, IkFilterOptions.CheckEnvCollisions)
            assert(ikreturn.GetAction() == IkReturnAction.Success)
            
            T = eye(4)
            T[2,3] = 0.5
            ikreturn = solver.Solve(ikparam2.Transform(T),None, IkFilterOptions.CheckEnvCollisions)
            assert(ikreturn.GetAction() == IkReturnAction.RejectKinematics)
            
            robot.SetDOFValues([pi/2,1],[1,3])
            ikparam = ikmodel.manip.GetIkParameterization(IkParameterizationType.Transform6D)
            solexpected = ikmodel.manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions)
            assert(solexpected is None)
            expectedaction = int(IkReturnAction.QuitEndEffectorCollision)|int(IkReturnAction.RejectJointLimits)
            ikreturn = ikmodel.manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions,ikreturn=True)
            assert(ikreturn.GetAction()==expectedaction)
            
            ikparam2 = ikparam.Transform(Tbaseinv)
            ikreturn = solver.Solve(ikparam2,None, IkFilterOptions.CheckEnvCollisions)
            assert( ikreturn.GetAction() == expectedaction)
        
    def test_customikvalues(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        with env:
            robot.SetDOFValues([pi/2,pi/4],[3,5])
            manip = robot.GetActiveManipulator()
            ikparam=manip.GetIkParameterization(IkParameterizationType.Transform6D)
            d = [1,0,0]
            p = [1,2,3]
            ikparam.SetCustomValues('myparam0_transform=direction_',d)
            ikparam.SetCustomValues('myparam1_transform=point_',p)
            ikparam.SetCustomValues('myparam2',[5,4])
            ikparam.SetCustomValues('myparam3_transform=ikparam_',r_[float(ikparam.GetType()),ikparam.GetValues()])
            assert(transdist(ikparam.GetCustomValues('myparam0_transform=direction_'),d) <= g_epsilon)
            assert(transdist(ikparam.GetCustomValues('myparam1_transform=point_'),p) <= g_epsilon)
            assert(transdist(ikparam.GetCustomValues('myparam2'),[5,4]) <= g_epsilon)
            assert(transdist(ikparam.GetCustomValues('myparam3_transform=ikparam_'),r_[float(ikparam.GetType()),ikparam.GetValues()]) <= g_epsilon)
            data=str(ikparam)
            ikparam2 = IkParameterization(data)
            assert(transdist(ikparam2.GetCustomValues('myparam0_transform=direction_'),d) <= g_epsilon)
            assert(transdist(ikparam2.GetCustomValues('myparam1_transform=point_'),p) <= g_epsilon)
            assert(transdist(ikparam2.GetCustomValues('myparam2'),[5,4]) <= g_epsilon)
            ikparam2.ClearCustomValues('myparam2')
            assert(ikparam2.GetCustomValues('myparam2') is None)
            assert(transdist(ikparam2.GetCustomValues('myparam3_transform=ikparam_'),r_[float(ikparam2.GetType()),ikparam2.GetValues()]) <= g_epsilon)
            ikparam2.ClearCustomValues()
            assert(ikparam2.GetCustomValues('myparam3_transform=ikparam_') is None)

            T = matrixFromAxisAngle([0,1,0])
            T[0:3,3] = [0.5,0.2,0.8]
            ikparam3=ikparam.Transform(T)
            assert(transdist(ikparam3.GetCustomValues('myparam0_transform=direction_'),dot(T[0:3,0:3],d)) <= g_epsilon)
            assert(transdist(ikparam3.GetCustomValues('myparam1_transform=point_'),dot(T[0:3,0:3],p)+T[0:3,3]) <= g_epsilon)
            assert(transdist(ikparam3.GetCustomValues('myparam2'),[5,4]) <= g_epsilon)
            assert(transdist(ikparam3.GetCustomValues('myparam3_transform=ikparam_'),r_[float(ikparam3.GetType()),ikparam3.GetValues()]) <= g_epsilon)

            T = linalg.inv(manip.GetBase().GetTransform())
            ikparam4 = ikparam.Transform(T)
            ikparam5 = manip.GetIkParameterization(ikparam4,False)
            assert(transdist(ikparam5.GetTransform6D(),ikparam4.GetTransform6D()) <= g_epsilon)
            assert(transdist(ikparam5.GetCustomValues('myparam0_transform=direction_'),dot(T[0:3,0:3],d)) <= g_epsilon)
            assert(transdist(ikparam5.GetCustomValues('myparam1_transform=point_'),dot(T[0:3,0:3],p)+T[0:3,3]) <= g_epsilon)
            assert(transdist(ikparam5.GetCustomValues('myparam2'),[5,4]) <= g_epsilon)
            assert(transdist(ikparam5.GetCustomValues('myparam3_transform=ikparam_'),r_[float(ikparam5.GetType()),ikparam5.GetValues()]) <= g_epsilon)
            
            def filtertest(sol,manip,ikparamint):
                assert(transdist(ikparamint.GetCustomValues('myparam0_transform=direction_'),dot(T[0:3,0:3],d)) <= g_epsilon)
                assert(transdist(ikparamint.GetCustomValues('myparam1_transform=point_'),dot(T[0:3,0:3],p)+T[0:3,3]) <= g_epsilon)
                assert(transdist(ikparamint.GetCustomValues('myparam2'),[5,4]) != g_epsilon)
                assert(transdist(ikparamint.GetCustomValues('myparam3_transform=ikparam_'),r_[float(ikparamint.GetType()),ikparamint.GetValues()]) != g_epsilon)
                return IkReturnAction.Success
            
            handle = ikmodel.manip.GetIkSolver().RegisterCustomFilter(0,filtertest)
            manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions)

            # test with velocities
            T = matrixFromAxisAngle([0,1,0])
            T[0:3,3] = [0.5,0.2,0.8]
            
            angularvelocity = array([1,0.5,-0.4])
            quatvelocity = 0.5*quatMultiply(r_[0,angularvelocity],ikparam.GetValues()[0:4])
            transvelocity = array([0.1,0.2,0.3])
            ikparamvel = IkParameterization()
            ikparamvel.SetValues(r_[quatvelocity,transvelocity],IkParameterizationType.Transform6DVelocity)

            ikparam2 = ikparam.Transform(T)
            ikparamvel2 = ikparamvel.Transform(T)
            newquatvelocity = ikparamvel2.GetValues()[0:4]
            newtransvelocity = ikparamvel2.GetValues()[4:7]
            newangularvelocity = 2*quatMultiply(newquatvelocity, quatInverse(ikparam2.GetValues()[0:4]))
            assert(transdist(newangularvelocity[1:4], dot(T[0:3,0:3],angularvelocity)) <= g_epsilon)
            assert(transdist(newtransvelocity, dot(T[0:3,0:3],transvelocity)) <= g_epsilon)

            # test pickling
            ikparam3pickled = pickle.loads(pickle.dumps(ikparam3))
            assert(str(ikparam3pickled) == str(ikparam3))

    def test_ikfastrobotsolutions(self):
        env=self.env
        testrobotfiles = [('ikfastrobots/testik0.zae','arm',[(zeros(6), 175)])]
        for robotfilename, manipname, testsolutions in testrobotfiles:
            env.Reset()
            robot=self.LoadRobot(robotfilename)
            manip=robot.GetManipulator(manipname)
            ikmodel=databases.inversekinematics.InverseKinematicsModel(manip=manip,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            for values, numexpected in testsolutions:
                robot.SetDOFValues(values,manip.GetArmIndices())
                solutions=manip.FindIKSolutions(manip.GetIkParameterization(ikmodel.iktype),0)
                numsolutions = len(solutions)
                if numsolutions != numexpected:
                    raise ValueError('%s!=%s, robot=%s, manip=%s, values=%r'%(numsolutions,numexpected,robotfilename,manipname, values))
                
                assert(numsolutions==numexpected)
