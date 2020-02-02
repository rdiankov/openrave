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
            expectedactions = [int(IkReturnAction.QuitEndEffectorCollision)|int(IkReturnAction.RejectJointLimits), int(IkReturnAction.QuitEndEffectorCollision)|int(IkReturnAction.Reject)]
            ikreturn = ikmodel.manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions,ikreturn=True)
            assert(ikreturn.GetAction() in expectedactions)
            
            ikparam2 = ikparam.Transform(Tbaseinv)
            ikreturn = solver.Solve(ikparam2,None, IkFilterOptions.CheckEnvCollisions)
            assert( ikreturn.GetAction() in expectedactions)
        
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
            ikparam.SetCustomValues('myparam3_transform=ikparam_',r_[float(int(ikparam.GetType())),ikparam.GetValues()])
            assert(transdist(ikparam.GetCustomValues('myparam0_transform=direction_'),d) <= g_epsilon)
            assert(transdist(ikparam.GetCustomValues('myparam1_transform=point_'),p) <= g_epsilon)
            assert(transdist(ikparam.GetCustomValues('myparam2'),[5,4]) <= g_epsilon)
            assert(transdist(ikparam.GetCustomValues('myparam3_transform=ikparam_'),r_[float(int(ikparam.GetType())),ikparam.GetValues()]) <= g_epsilon)
            data=str(ikparam)
            ikparam2 = IkParameterization(data)
            assert(transdist(ikparam2.GetCustomValues('myparam0_transform=direction_'),d) <= g_epsilon)
            assert(transdist(ikparam2.GetCustomValues('myparam1_transform=point_'),p) <= g_epsilon)
            assert(transdist(ikparam2.GetCustomValues('myparam2'),[5,4]) <= g_epsilon)
            ikparam2.ClearCustomValues('myparam2')
            assert(ikparam2.GetCustomValues('myparam2') is None)
            assert(transdist(ikparam2.GetCustomValues('myparam3_transform=ikparam_'),r_[float(int(ikparam2.GetType())),ikparam2.GetValues()]) <= g_epsilon)
            ikparam2.ClearCustomValues()
            assert(ikparam2.GetCustomValues('myparam3_transform=ikparam_') is None)

            T = matrixFromAxisAngle([0,1,0])
            T[0:3,3] = [0.5,0.2,0.8]
            ikparam3=ikparam.Transform(T)
            assert(transdist(ikparam3.GetCustomValues('myparam0_transform=direction_'),dot(T[0:3,0:3],d)) <= g_epsilon)
            assert(transdist(ikparam3.GetCustomValues('myparam1_transform=point_'),dot(T[0:3,0:3],p)+T[0:3,3]) <= g_epsilon)
            assert(transdist(ikparam3.GetCustomValues('myparam2'),[5,4]) <= g_epsilon)
            assert(transdist(ikparam3.GetCustomValues('myparam3_transform=ikparam_'),r_[float(int(ikparam3.GetType())),ikparam3.GetValues()]) <= g_epsilon)

            T = linalg.inv(manip.GetBase().GetTransform())
            ikparam4 = ikparam.Transform(T)
            ikparam5 = manip.GetIkParameterization(ikparam4,False)
            assert(transdist(ikparam5.GetTransform6D(),ikparam4.GetTransform6D()) <= g_epsilon)
            assert(transdist(ikparam5.GetCustomValues('myparam0_transform=direction_'),dot(T[0:3,0:3],d)) <= g_epsilon)
            assert(transdist(ikparam5.GetCustomValues('myparam1_transform=point_'),dot(T[0:3,0:3],p)+T[0:3,3]) <= g_epsilon)
            assert(transdist(ikparam5.GetCustomValues('myparam2'),[5,4]) <= g_epsilon)
            assert(transdist(ikparam5.GetCustomValues('myparam3_transform=ikparam_'),r_[float(int(ikparam5.GetType())),ikparam5.GetValues()]) <= g_epsilon)
            
            def filtertest(sol,manip,ikparamint):
                assert(transdist(ikparamint.GetCustomValues('myparam0_transform=direction_'),dot(T[0:3,0:3],d)) <= g_epsilon)
                assert(transdist(ikparamint.GetCustomValues('myparam1_transform=point_'),dot(T[0:3,0:3],p)+T[0:3,3]) <= g_epsilon)
                assert(transdist(ikparamint.GetCustomValues('myparam2'),[5,4]) != g_epsilon)
                assert(transdist(ikparamint.GetCustomValues('myparam3_transform=ikparam_'),r_[float(int(ikparamint.GetType())),ikparamint.GetValues()]) != g_epsilon)
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
        testrobotfiles = [('ikfastrobots/testik0.zae','arm',[(zeros(6), 100)])]
        for robotfilename, manipname, testsolutions in testrobotfiles:
            env.Reset()
            robot=self.LoadRobot(robotfilename)
            manip=robot.GetManipulator(manipname)
            ikmodel=databases.inversekinematics.InverseKinematicsModel(manip=manip,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            for values, numexpected in testsolutions:
                robot.SetDOFValues(values,manip.GetArmIndices())
                realpose = manip.GetTransformPose()
                solutions=manip.FindIKSolutions(manip.GetIkParameterization(ikmodel.iktype),0)
                numsolutions = len(solutions)
                if numsolutions < numexpected:
                    raise ValueError('%s!=%s, robot=%s, manip=%s, values=%r'%(numsolutions,numexpected,robotfilename,manipname, values))
                
                # test every solution
                counts = 0
                f = 0
                usedindices = []
                for isolution, testsolution in enumerate(solutions):
                    robot.SetDOFValues(testsolution)
                    testpose = manip.GetTransformPose()
                    distquat = arccos(min(1, abs(dot(testpose[:4],realpose[:4]))))
                    disttrans2 = sum((testpose[4:]-realpose[4:])**2)
                    assert(distquat+sqrt(disttrans2) <= 1e-6)
                    
                    # make sure this solution is unique
                    numCloseSolutions = sum(sum((solutions - tile(testsolution, (len(solutions),1)))**2, axis=1) <= 1e-10)
                    f += 1.0/numCloseSolutions
                    assert(numCloseSolutions==1)
    
    def test_circularfree(self):
        # test when free joint is circular and IK doesn't succeed (thanks to Chris Dellin)
        robotxmldata = '''<Robot name="BarrettWAM">
<kinbody>
    <Body name="wam0" type="dynamic">
      <Translation>-0.22 -0.14 -0.346</Translation>
      <Geom type="trimesh">
        <Data>models/WAM/wam0.iv 1.0</Data>
        <Render>models/WAM/wam0.iv  1.0</Render>
      </Geom>
      <mass type="custom">
        <total>9.97059584</total>
        <com>0.19982328999999999 0.14 0.079952939999999972</com>
        <inertia>0.10916849 0.00640270  0.02557874 0.00640270 0.18294303 0.00161433 0.02557874 0.00161433 0.11760385</inertia>
      </mass>
    </Body>

    <Body name="wam1" type="dynamic">
      <offsetfrom>wam0</offsetfrom>
      <Translation>0.22 0.14 0.346</Translation>
      <rotationaxis>1 0 0 -90</rotationaxis>
      <Geom type="trimesh">
        <rotationaxis>1 0 0 90</rotationaxis>
        <Data>models/WAM/wam1.iv 1.0</Data>
        <Render>models/WAM/wam1.iv  1.0</Render>
      </Geom>
      <mass type="custom">
        <total>8.3936</total>
        <com> -0.00443422 0.12189039 -0.00066489</com>
        <inertia>0.13488033 -0.00213041 -0.00012485 -0.00213041 0.11328369 0.00068555 -0.00012485 0.00068555 0.09046330</inertia>
      </mass>
    </Body>

    <Joint name="Shoulder_Yaw" type="hinge">
      <Body>wam0</Body>
      <Body>wam1</Body>
      <offsetfrom>wam0</offsetfrom>
      <axis>0 0 1</axis>
      <anchor>0.22 0.14 0.346</anchor>
      <limitsdeg>-150 150</limitsdeg>
      <maxvel>1.5708</maxvel>
      <weight>1.92154</weight>
      <resolution>0.25</resolution>
      <rotorinertia>0.0 0.201 0.0</rotorinertia>
      <maxtorque>77.3</maxtorque>
    </Joint>
    <Body name="wam2" type="dynamic">
      <offsetfrom>wam1</offsetfrom>
      <rotationaxis>1 0 0 90</rotationaxis>
      <Translation>0  0  0</Translation>
      <Geom type="trimesh">
        <rotationaxis>1 0 0 -90</rotationaxis>
        <Data>models/WAM/wam2.iv  1.0</Data>
        <Render>models/WAM/wam2.iv  1.0</Render>
      </Geom>
      <mass type="custom">
        <total>3.87493756</total>
        <com> -0.00236983 0.03105614 0.01542114</com>
        <inertia>0.02140958 0.00027172 0.00002461 0.00027172 0.01377875  -0.00181920 0.00002461 -0.00181920 0.01558906</inertia>
      </mass>
    </Body>
    <Joint name="Shoulder_Pitch" type="hinge">
      <Body>wam1</Body>
      <Body>wam2</Body>
      <offsetfrom>wam1</offsetfrom>
      <axis>0 0 1</axis>
      <limitsdeg>-113 113</limitsdeg>
      <weight>0.91739941</weight>
      <maxvel>1.0472</maxvel>
      <resolution>0.5</resolution>
      <rotorinertia>0 0.182 0</rotorinertia>
      <maxtorque>160.6</maxtorque>
    </Joint>
    <Body name="wam3" type="dynamic">
      <offsetfrom>wam2</offsetfrom>
      <rotationaxis>1 0 0 -90</rotationaxis>
      <translation>0.045 0 0.55</translation>
      <Geom type="trimesh">
        <rotationaxis>1 0 0 90</rotationaxis>
        <translation>-0.045 0.55 0</translation>
        <Data>models/WAM/wam3.iv  1.0</Data>
        <Render>models/WAM/wam3.iv  1.0</Render>
      </Geom>
      <mass type="custom">
        <total>1.80228141</total>
        <com>-0.03825858 0.20750770 0.00003309</com>
        <inertia>0.05911077 -0.00249612 0.00000738 -0.00249612 0.00324550 -0.00001767 0.00000738 -0.00001767 0.05927043</inertia>
      </mass>
    </Body>
    <Joint name="Shoulder_Roll" type="hinge" circular="true">
      <Body>wam2</Body>
      <Body>wam3</Body>
      <offsetfrom>wam2</offsetfrom>
      <axis>0 0 1</axis>
      <!--<limitsdeg>-157 157</limitsdeg>-->
      <weight>0.882397</weight>
      <maxvel>2.0944</maxvel>
      <resolution>0.5</resolution>
      <rotorinertia>0 0.067 0</rotorinertia>
      <maxtorque>95.6</maxtorque>
    </Joint>
    <Body name="wam4" type="dynamic">
      <offsetfrom>wam3</offsetfrom>
      <rotationaxis>1 0 0 90</rotationaxis>
      <translation>-0.045 0 0</translation>
      <Geom type="trimesh">
        <rotationaxis>1 0 0 -90</rotationaxis>
        <translation>0.045 0 0</translation>
        <Data>models/WAM/wam4.iv  1.0</Data>
        <Render>models/WAM/wam4.iv  1.0</Render>
      </Geom>
      <mass type="custom">
        <total>2.40016804 </total>
        <com>0.00498512 -0.00022942 0.13271662</com>
        <inertia>0.01491672 0.00001741 -0.00150604 0.00001741 0.01482922 -0.00002109 -0.00150604 -0.00002109 0.00294463</inertia>
      </mass>
    </Body>
    <Joint name="Elbow" type="hinge">
      <Body>wam3</Body>
      <Body>wam4</Body>
      <offsetfrom>wam3</offsetfrom>
      <axis>0 0 1</axis>
      <limitsdeg>-50 180</limitsdeg>
      <weight>0.45504</weight>
      <maxvel>2.0944</maxvel>
      <resolution>1</resolution>
      <rotorinertia>0 0.034 0</rotorinertia>
      <maxtorque>29.4</maxtorque>
    </Joint>
    <Body name="wam5" type="dynamic">
      <offsetfrom>wam4</offsetfrom>
      <translation>0 0 0.3</translation>
      <rotationaxis>1 0 0 -90</rotationaxis>
      <Geom type="trimesh">
        <translation>0 0.3 0</translation>
        <rotationaxis>1 0 0 90</rotationaxis>
        <Data>models/WAM/wam5.iv  1.0</Data>
        <Render>models/WAM/wam5.iv  1.0</Render>
      </Geom>
      <mass type="custom">
        <total>0.12376019</total>
        <com>0.00008921 0.00511217 0.00435824</com>
        <inertia>0.00005029 0.00000020 -0.00000005 0.00000020 0.00007582 -0.00000359 -0.00000005 -0.00000359 0.00006270</inertia>
      </mass>
    </Body>
    <Joint name="Wrist_Yaw" type="hinge">
      <Body>wam4</Body>
      <Body>wam5</Body>
      <offsetfrom>wam4</offsetfrom>
      <axis>0 0 1</axis>
      <limitsdeg>-275 75</limitsdeg>
      <weight>0.40141</weight>
      <maxvel>4.1888</maxvel>
      <resolution>1.11</resolution>
      <rotorinertia>0 0.0033224 0</rotorinertia>
      <maxtorque>11.6</maxtorque>
    </Joint>

    <Body name="wam6" type="dynamic">
      <offsetfrom>wam5</offsetfrom>
      <rotationaxis>1 0 0 90</rotationaxis>
      <Geom type="trimesh">
        <rotationaxis>1 0 0 -90</rotationaxis>
        <Data>models/WAM/wam6.iv  1.0</Data>
        <Render>models/WAM/wam6.iv  1.0</Render>
      </Geom>
      <mass type="custom">
        <total>0.41797364</total>
        <com>-0.00012262 -0.01703194 0.02468336</com>
        <inertia>0.00055516 0.00000061 -0.00000074 0.00000061 0.00024367 -0.00004590 -0.00000074 -0.00004590 0.00045358</inertia>
      </mass>
    </Body>

    <Joint name="Wrist_Pitch" type="hinge">
      <Body>wam5</Body>
      <Body>wam6</Body>
      <offsetfrom>wam5</offsetfrom>
      <axis>0 0 1</axis>
      <limitsdeg>-90 90</limitsdeg>
      <weight>0.22841245</weight>
      <maxvel>4.1888</maxvel>
      <resolution>1.62</resolution>
      <rotorinertia>0 0.0033224 0</rotorinertia>
      <maxtorque>11.6</maxtorque>
    </Joint>
    <Body name="wam7" type="dynamic">
      <offsetfrom>wam6</offsetfrom>
      <Translation>0.0  0.0  0.06</Translation>
      <Geom type="trimesh">
        <Translation>0.0  0.0  -0.06</Translation>
        <data>models/WAM/wam7_nohand.iv 1.0</data>
        <Render>models/WAM/wam7_nohand.iv 1.0</Render>
      </Geom>
      <mass type="custom">
        <total>0.06864753</total>
        <com>-0.00007974 0.00016313 -0.00323552</com>
        <inertia>0.00003773 -0.00000019 0.00000000 -0.00000019 0.00003806 0.00000000 0.00000000 0.00000000 0.00007408</inertia>
      </mass>
    </Body>

    <Joint name="Wrist_Roll" type="hinge">
      <Body>wam6</Body>
      <Body>wam7</Body>
      <offsetfrom>wam6</offsetfrom>
      <axis>0 0 1</axis>
      <limitsdeg>-172 172</limitsdeg>
      <weight>0.20178</weight>
      <maxvel>1.0472</maxvel>
      <resolution>1.62</resolution>
      <rotorinertia>0 0 0.000466939</rotorinertia>
      <maxtorque>2.7</maxtorque>
    </Joint>

    <adjacent>wam1 wam3</adjacent>
    <adjacent>wam4 wam6</adjacent>
    <adjacent>wam4 wam7</adjacent>
  </kinbody>
  <KinBody file="robots/barretthand.kinbody.xml"/>

 <kinbody>
   <body name="handbase">
     <offsetfrom>wam7</offsetfrom>
   </body>
   <joint name="dummyhand" type="hinge" enable="false">
     <body>wam7</body>
     <body>handbase</body>
     <limits>0 0</limits>
   </joint>
 </kinbody>

  <Manipulator name="arm">
    <base>wam0</base>
    <effector>wam7</effector>
    <Translation>0 0 0.16</Translation>
    <closingdirection>1 1 1 0</closingdirection>
    <direction>0 0 1</direction>
  </Manipulator>
</Robot>
'''
        env=self.env
        r=self.LoadRobotData(robotxmldata)
        # all of this transform's solutions are self-colliding
        Tee = numpy.array( # wam
            [[ -1.1857439e-02,  -3.7203997e-16,  -9.9992969e-01,  -1.4459252e-02 ],
             [ -3.6269429e-14,   1.0000000e+00,   5.8026667e-17,  -5.9604621e-10 ],
             [  9.9992969e-01,   3.6267567e-14,  -1.1857439e-02,  -2.7371536e-01 ],
             [  0.0000000e+00,   0.0000000e+00,   0.0000000e+00,   1.0000000e+00 ]])
        
        r.SetActiveManipulator('arm')
        r.SetActiveDOFs(r.GetActiveManipulator().GetArmIndices())
        ikmodel = databases.inversekinematics.InverseKinematicsModel(r, iktype=IkParameterizationType.Transform6D,freeindices=[2],forceikfast=True)
        if not ikmodel.load(checkforloaded=False):
            ikmodel.autogenerate()
            
        sol = r.GetActiveManipulator().FindIKSolution(Tee, IkFilterOptions.IgnoreSelfCollisions)
        assert(sol is not None)
        r.SetActiveDOFValues(sol)
        
        sol = r.GetActiveManipulator().FindIKSolution(Tee, 0)
        assert( sol is None)
