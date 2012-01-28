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
from common_test_openrave import *

class TestIkSolver(EnvironmentSetup):
    def test_customfilter(self):
        env=self.env
        env.Load('data/lab1.env.xml')
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
                return IkFilterReturn.Success if sol[2] > -0.2 else IkFilterReturn.Reject

            def filter1test(sol,manip,ikparam):
                assert(sol[2] < 0.2)
                return IkFilterReturn.Success if sol[2] > -0.2 else IkFilterReturn.Reject

            def filter2(sol,manip,ikparam):
                return IkFilterReturn.Success if sol[2] < 0.2 else IkFilterReturn.Reject

            def filter2test(sol,manip,ikparam):
                assert(sol[2] > -0.2)
                return IkFilterReturn.Success if sol[2] < 0.2 else IkFilterReturn.Reject

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
        env.Load('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        with env:
            # set robot in collision
            robot.SetDOFValues([ -8.44575603e-02,   1.48528347e+00,  -5.09108824e-08, 6.48108822e-01,  -4.57571203e-09,  -1.04008750e-08, 7.26855048e-10,   5.50807826e-08,   5.50807826e-08, -1.90689327e-08,   0.00000000e+00])
            manip = robot.GetActiveManipulator()
            ikparam=manip.GetIkParameterization(IkParameterizationType.Transform6D)
            assert(manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions) is None)
            assert(manip.FindIKSolution(ikparam,0) is not None)
            sampler=planningutils.ManipulatorIKGoalSampler(robot.GetActiveManipulator(),[ikparam],nummaxsamples=20,nummaxtries=10,jitter=0)
            assert(sampler.Sample() is None)
            sampler=planningutils.ManipulatorIKGoalSampler(robot.GetActiveManipulator(),[ikparam],nummaxsamples=20,nummaxtries=10,jitter=0.03)
            assert(sampler.Sample() is not None)


