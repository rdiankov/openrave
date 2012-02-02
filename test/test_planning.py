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

class TestMoving(EnvironmentSetup):
    def test_basicplanning(self):
        env = self.env
        with env:
            self.LoadEnv('data/hironxtable.env.xml')
            robot = env.GetRobots()[0]
            manip = robot.SetActiveManipulator('leftarm_torso')
            basemanip = interfaces.BaseManipulation(robot)
            robot.SetActiveDOFs(manip.GetArmIndices())
            goal = robot.GetActiveDOFValues()
            goal[0] = -0.556
            goal[3] = -1.86
            traj = basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.01,maxtries=2,execute=False,outputtrajobj=True)
            with robot:
                parameters = Planner.PlannerParameters()
                parameters.SetRobotActiveJoints(robot)
                planningutils.VerifyTrajectory(parameters,traj,samplingstep=0.002)
            self.RunTrajectory(robot,traj)
            
    def test_ikplanning(self):
        env = self.env
        self.LoadEnv('data/lab1.env.xml')
        robot = env.GetRobots()[0]
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        with env:
            Tee = ikmodel.manip.GetEndEffectorTransform()
            Tee[0:3,3] -= 0.4
            solutions = ikmodel.manip.FindIKSolutions(Tee,IkFilterOptions.CheckEnvCollisions)
            assert(len(solutions)>1)
            basemanip = interfaces.BaseManipulation(robot)
            with robot:
                for sol in solutions:
                    robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
                    assert(transdist(Tee,ikmodel.manip.GetEndEffectorTransform()) <= g_epsilon)
            traj=basemanip.MoveManipulator(goals=solutions,execute=True,outputtrajobj=True)
            # check that last point is accurate
            lastvalues = traj.GetConfigurationSpecification().ExtractJointValues(traj.GetWaypoint(-1), robot, ikmodel.manip.GetArmIndices(), 0)
            with robot:
                robot.SetDOFValues(lastvalues,ikmodel.manip.GetArmIndices())
                assert(min([sum(abs(s-lastvalues)) for s in solutions]) <= g_epsilon)
                assert(transdist(Tee,ikmodel.manip.GetEndEffectorTransform()) <= g_epsilon)
            
        env.StartSimulation(0.01,False)
        robot.WaitForController(0)
        with env:
            print 'test_ikplanning:',transdist(Tee,ikmodel.manip.GetEndEffectorTransform())
            assert(transdist(Tee,ikmodel.manip.GetEndEffectorTransform()) <= g_epsilon)
            
    def test_constraintpr2(self):
        env = self.env
        robot = self.LoadRobot('robots/pr2-beta-static.zae')
        with env:
            manip=robot.SetActiveManipulator('leftarm_torso')
            basemanip = interfaces.BaseManipulation(robot)
            robot.SetDOFValues([.31],[robot.GetJoint('torso_lift_joint').GetDOFIndex()])
            T=array([[0,0,1,.6], [0,1,0,.1], [-1,0,0,.73], [0,0,0,1]])
            robot.SetDOFValues(manip.FindIKSolution(T,IkFilterOptions.CheckEnvCollisions),manip.GetArmIndices())
            Tgoal=array([[0,0,1,.6], [0,1,0,.3], [-1,0,0,.73], [0,0,0,1]])
            constraintfreedoms=array([1,1,0,1,0,0]) # can rotate along z, translate along y
            constraintmatrix=array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
            for constrainterrorthresh in [0.002,0.01]:
                traj = basemanip.MoveToHandPosition(matrices=[Tgoal],maxiter=6000,maxtries=2,seedik=16, constraintfreedoms=constraintfreedoms, constraintmatrix=constraintmatrix, constrainterrorthresh=constrainterrorthresh,execute=False,outputtrajobj=True,steplength=0.001)
                self.RunTrajectory(robot,traj)

    def test_constraintwam(self):
        env = self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]

        print 'test_constraintwam: ikmodel'
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        print 'test_constraintwam: gmodel'
        gmodel = databases.grasping.GraspingModel(robot=robot,target=env.GetKinBody('mug1'))
        if not gmodel.load():
            # don't do multithreaded yet since ode on some ubuntu distors does not support it
            #gmodel.numthreads = 2 # at least two threads
            gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.04))
            gmodel.save()
        print 'test_constraintwam: planning'
        with env:
            basemanip = interfaces.BaseManipulation(robot)
            robot.SetActiveDOFs(ikmodel.manip.GetArmIndices())
            validgrasps,validindices = gmodel.computeValidGrasps(returnnum=1)
            validgrasp=validgrasps[0]
            gmodel.setPreshape(validgrasp)
            T = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
            sol = gmodel.manip.FindIKSolution(T,IkFilterOptions.CheckEnvCollisions)
            robot.SetActiveDOFValues(sol)
            robot.Grab(gmodel.target)            
            xyzconstraints = array([1,2],int)
            constraintfreedoms = array([1,0,1,1,0,0]) # rotation xyz, translation xyz
            localrotaxis = array([0,1,0])
            constrainterrorthresh = 0.005
            constrainttaskmatrix=dot(linalg.inv(T),gmodel.target.GetTransform())
            constraintmatrix = linalg.inv(gmodel.target.GetTransform())
            Tgoal = ikmodel.manip.GetTransform()
            Tgoal[0:3,3] += [0,0.1,0.2]
            solgoal = ikmodel.manip.FindIKSolution(Tgoal,IkFilterOptions.CheckEnvCollisions)
            assert(solgoal is not None)
            traj = basemanip.MoveToHandPosition(matrices=[Tgoal],maxiter=3000,maxtries=1,seedik=40,constraintfreedoms=constraintfreedoms,constraintmatrix=constraintmatrix,constrainttaskmatrix=constrainttaskmatrix,constrainterrorthresh=constrainterrorthresh,steplength=0.002,outputtrajobj=True,execute=False,jitter=0.05)
            soltraj = traj.Sample(0,robot.GetActiveConfigurationSpecification())
            print 'test_constraintwam: make sure it starts at the initial configuration'
            assert(transdist(soltraj,sol) <= g_epsilon)
            self.RunTrajectory(robot,traj)
                
            # try another goal
            Tlocal = matrixFromAxisAngle([0,1,0])
            Tlocal[0:3,3] = [0,0.1,-0.2]
            Tnewtarget = dot(gmodel.target.GetTransform(),Tlocal)
            Tgoal = dot(Tnewtarget, dot(linalg.inv(gmodel.target.GetTransform()), ikmodel.manip.GetTransform()))
            traj = basemanip.MoveToHandPosition(matrices=[Tgoal],maxiter=3000,maxtries=1,seedik=40,constraintfreedoms=constraintfreedoms,constraintmatrix=constraintmatrix,constrainttaskmatrix=constrainttaskmatrix,constrainterrorthresh=constrainterrorthresh,steplength=0.002,outputtrajobj=True,execute=False,jitter=0.05)
            self.RunTrajectory(robot,traj)

    def test_wamgraspfromcollision(self):
        env = self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]

        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        gmodel = databases.grasping.GraspingModel(robot=robot,target=env.GetKinBody('mug1'))
        if not gmodel.load():
            # don't do multithreaded yet since ode on some ubuntu distors does not support it
            #gmodel.numthreads = 2 # at least two threads
            gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.04))
            gmodel.save()
            
        with env:
            basemanip = interfaces.BaseManipulation(robot)
            robot.SetActiveDOFs(ikmodel.manip.GetArmIndices())
            orgvalues = robot.GetActiveDOFValues()
            validgrasps,validindices = gmodel.computeValidGrasps(returnnum=1)
            validgrasp=validgrasps[0]
            gmodel.setPreshape(validgrasp)
            T = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True)
            traj = basemanip.MoveToHandPosition(matrices=[T],outputtrajobj=True,execute=False)
            self.RunTrajectory(robot,traj)
            robot.Grab(gmodel.target)
            sol = robot.GetActiveDOFValues()
            traj = basemanip.MoveManipulator(orgvalues,outputtrajobj=True,execute=False,jitter=0.05)
            soltraj = traj.Sample(0,robot.GetActiveConfigurationSpecification())
            print 'test_wamgraspfromcollision: make sure it starts at the initial configuration'
            assert(transdist(soltraj,sol) <= g_epsilon)
            self.RunTrajectory(robot,traj)
                
    def test_movehandstraight(self):
        env = self.env
        self.LoadEnv('data/lab1.env.xml')
        robot = env.GetRobots()[0]
        basemanip = interfaces.BaseManipulation(robot)

        testvalues = [[array([ -2.83686683e-01,   1.40828054e+00,   0.00000000e+00, 5.26754682e-01,  -3.14159265e+00,  -1.20655743e+00, -1.85448301e+00,   1.66533454e-16,   1.66533454e-16,         1.66533454e-16,   0.00000000e+00]), array([ -9.22429319e-16,  -3.90560499e-15,  -1.00000000e+00])],
                      [array([-0.26085414,  1.37967815,  0.        ,  0.60871186, -3.14159265,       -1.15320264, -0.26085414,  0.        ,  0.        ,  0.        ,  0.        ]), array([ -7.21644966e-16,  -3.28903571e-15,  -1.00000000e+00])],
                      [array([ -5.90848599e-02,   9.54294051e-01,   0.00000000e+00,         2.22628339e+00,   9.99200722e-15,  -3.89847865e-02,         1.51171147e+00,   1.66533454e-16,   1.66533454e-16,         1.66533454e-16,   0.00000000e+00]), array([ -1.03626102e-14,   7.85046229e-16,  -1.00000000e+00])],
                      [array([-0.53374407,  0.9960226 ,  0.        ,  1.91409838, -3.14159265,       -0.23147168, -2.10454039,  0.        ,  0.        ,  0.        ,  0.        ]), array([  6.86915451e-16,  -1.35420475e-15,  -1.00000000e+00])]]
        
        with env:
            for dofvalues, direction in testvalues:
                robot.SetDOFValues(dofvalues)
                assert( not env.CheckCollision(robot) )
                ret = basemanip.MoveHandStraight(direction=direction, ignorefirstcollision=False,stepsize=0.001,minsteps=19,maxsteps=20, execute=False)
                assert(ret is not None)
                

    def test_movetohandpositiongrab(self):
        env=self.env
        self.LoadEnv('data/hanoi_complex2.env.xml')
        robot = env.GetRobots()[0]
        basemanip = interfaces.BaseManipulation(robot)
        with env:
            resolutions = [ 0.00292825,  0.00303916,  0.01520142,  0.0163279,   0.03591959,  0.03591959,  0.08129367]
            weights = [ 1.61903856,  1.11858069,  0.20061367,  0.15267405,  0.05951496,  0.04199751,  0.01950391]
            for j in robot.GetJoints():
                j.SetWeights(weights[j.GetDOFIndex():(j.GetDOFIndex()+j.GetDOF())])
                j.SetResolution(resolutions[j.GetDOFIndex()])
            Tdisk2 = array([[-0.9152146 ,  0.40084098, -0.04133701, -0.21687754],
                            [-0.39826911, -0.88415551,  0.24423505, -0.19242871],
                            [ 0.06135107,  0.23999073,  0.96883461,  1.12189841],
                            [ 0.        ,  0.        ,  0.        ,  1.        ]])
            env.GetKinBody('disk2').SetTransform(Tdisk2)
            disk1 = env.GetKinBody('disk1')
            robot.SetDOFValues([ 0.4620151 , -2.67934022,  0.29334635, -1.45878047, -1.2220377 , -1.83725485, -0.25900999])
            robot.Grab(disk1)
            Tgoal = array([[-0.7912808 ,  0.25088882,  0.55761053, -0.1646556 ],
                           [-0.3734526 ,  0.52379002, -0.76562208,  0.02972558],
                           [-0.48415685, -0.81406315, -0.32076991,  1.38703197],
                           [ 0.        ,  0.        ,  0.        ,  1.        ]])
            for i in range(4):
                # have to execute several times to bring out the bug
                out = basemanip.MoveToHandPosition(matrices=[Tgoal],execute=False)

    def test_navigationmanip(self):
        env=self.env
        env.StartSimulation(0.1,False)
        self.LoadEnv('data/pr2test2.env.xml')
        robot = env.GetRobots()[0]

        manip = robot.SetActiveManipulator('leftarm_torso')
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        nonadjlinks = array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))
        basemanip = interfaces.BaseManipulation(robot)
        taskmanip = interfaces.TaskManipulation(robot)
        target=env.GetKinBody('TibitsBox1')
        with env:
            targetcollision = env.CheckCollision(target)
            jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
            armindices = [robot.GetJoint(name).GetDOFIndex() for name in jointnames]
            armgoal = [1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]
            robot.SetActiveDOFs(armindices)
            basemanip.MoveActiveJoints(goal=armgoal)
        robot.WaitForController(100)
        
        with env:
            robot.SetActiveDOFs([],Robot.DOFAffine.X|Robot.DOFAffine.Y|Robot.DOFAffine.RotationAxis,[0,0,1])
            basemanip.MoveActiveJoints(goal=[2.8,-1.3,0],maxiter=5000,steplength=0.15,maxtries=2)
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
        robot.WaitForController(100)

        taskmanip.ReleaseFingers()
        with env:
            assert( transdist(robot.GetDOFValues(armindices),armgoal) <= g_epsilon )
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
        robot.WaitForController(100)

        Tgoal = array([[0,-1,0,3.5],[-1,0,0,-1.3],[0,0,-1,0.842],[0,0,0,1]])
        res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)
        with env:
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
        robot.WaitForController(100)

        taskmanip.CloseFingers()
        with env:
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
        robot.WaitForController(100)

        with env:
            robot.Grab(target)
            assert( transdist(nonadjlinks,array(robot.GetNonAdjacentLinks(KinBody.AdjacentOptions.Enabled))) == 0 )
            assert( not targetcollision or env.CheckCollision(robot) )
            basemanip.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
        robot.WaitForController(100)

    def test_planwithcollision(self):
        env=self.env
        self.LoadEnv('data/pr2test1.env.xml')
        robot=env.GetRobots()[0]
        with env:
            defaultvalues = robot.GetDOFValues()
            manip = robot.SetActiveManipulator('rightarm')
            basemanip = interfaces.BaseManipulation(robot)

            robot.SetDOFValues([0.187],[robot.GetJoint('l_shoulder_lift_joint').GetDOFIndex()])
            assert(env.CheckCollision(robot))

            print 'test_planwithcollision: environment collision'
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()

            Tdelta = eye(4)
            Tdelta[0,3] = -0.2
            Tdelta[2,3] = -0.2
            Tnew = dot(manip.GetEndEffectorTransform(),Tdelta)

            ret = basemanip.MoveToHandPosition([Tnew],execute=False)
            assert(ret is not None)

            print 'test_planwithcollision: self collision'
            robot.SetDOFValues(defaultvalues)
            robot.SetDOFValues([ 1.34046301,  0.94535038,  3.03934583, -1.30743665, 0 , 0 ,  0], robot.GetManipulator('leftarm').GetArmIndices())
            assert(robot.CheckSelfCollision())
            ret = basemanip.MoveToHandPosition([Tnew],execute=False)
            assert(ret is not None)

            manip = robot.SetActiveManipulator('rightarm_torso')
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            try:
                ret = basemanip.MoveToHandPosition([Tnew],execute=False)
            except planning_error:
                ret = None
            assert(ret==None)

            robot.SetDOFValues([ 1.34046301, -0.52360053,  0.03541482, -2.32130534,  0, 0,  0], robot.GetManipulator('leftarm').GetArmIndices())
            assert(robot.CheckSelfCollision())
            ret = basemanip.MoveToHandPosition([Tnew],execute=False)
            assert(ret is not None)

    def test_movebase(self):
        env=self.env
        xml = """
<robot name="diffdrive_caster">
  <kinbody>
    <body name="base" type="static">
      <mass type="box">
        <total>50</total>
        <extents>1 1 1</extents>
      </mass>
      <geom type="box">
        <extents>0.1 0.2 0.25</extents>
        <translation>0 0 0.35</translation>
      </geom>
    </body>
    <body name="wheel_left">
      <geom type="cylinder">
        <radius>0.2</radius>
        <height>0.05</height>
        <translation>0 0.26 0.2</translation>
      </geom>
    </body>
    <body name="wheel_right">s
      <geom type="cylinder">
        <radius>0.2</radius>
        <height>0.05</height>
        <translation>0 -0.26 0.2</translation>
      </geom>
    </body>
  </kinbody>
</robot>
"""
        robot=self.LoadRobotData(xml)
        robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y |DOFAffine.RotationAxis, [0,0,1])
        basemanip = interfaces.BaseManipulation(robot)
        traj=basemanip.MoveActiveJoints([1,1,1],outputtrajobj=True)
        self.RunTrajectory(robot,traj)

    def test_wamtaskplanwithgoal(self):
        env = self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]

        def ComputeDestinations(target, table,transdelta=0.1,zoffset=0.01):
            with table:
                Ttable = table.GetTransform()
                table.SetTransform(eye(4))
                ab = table.ComputeAABB()
                table.SetTransform(Ttable)
                p = ab.pos()
                e = ab.extents()
                Nx = floor(2*e[0]/transdelta)
                Ny = floor(2*e[1]/transdelta)
                X = []
                Y = []
                for x in arange(Nx):
                    X = r_[X, tile((x+1)/(Nx+1),Ny)]
                    Y = r_[Y, arange(0.5,Ny,1.0)/(Ny+1)]
                translations = c_[p[0]-e[0]+2*e[0]*X,p[1]-e[1]+2*e[1]*Y,tile(p[2]+e[2]+zoffset,len(X))]
                Trolls = [matrixFromAxisAngle(array((0,0,1)),roll) for roll in arange(0,2*pi,pi/2)] + [matrixFromAxisAngle(array((1,0,0)),roll) for roll in [pi/2,pi,1.5*pi]]
                dests = []
                Torg = eye(4)
                with KinBodyStateSaver(target):
                    dests = []
                    for translation in translations:
                        for Troll in Trolls:
                            Troll = array(Troll)
                            Troll[0:3,3] = translation
                            target.SetTransform(dot(Ttable, dot(Troll, Torg)))
                            if not table.GetEnv().CheckCollision(target):
                                dests.append(target.GetTransform())
                return dests

        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        gmodel = databases.grasping.GraspingModel(robot=robot,target=env.GetKinBody('mug1'))
        if not gmodel.load():
            # don't do multithreaded yet since ode on some ubuntu distors does not support it
            #gmodel.numthreads = 2 # at least two threads
            gmodel.generate(approachrays=gmodel.computeBoxApproachRays(delta=0.04))
            gmodel.save()
            
        with env:
            basemanip = interfaces.BaseManipulation(robot)
            taskmanip = interfaces.TaskManipulation(robot,graspername=gmodel.grasper.plannername)            
            target = env.GetKinBody('mug1')
            approachoffset = 0.02
            dests = ComputeDestinations(target,env.GetKinBody('table'))
            goals,graspindex,searchtime,traj = taskmanip.GraspPlanning(graspindices=gmodel.graspindices,grasps=gmodel.grasps, target=target,approachoffset=approachoffset,destposes=dests, seedgrasps = 3,seeddests=8,seedik=1,maxiter=1000, randomgrasps=False,randomdests=False,grasptranslationstepmult=gmodel.translationstepmult,graspfinestep=gmodel.finestep,execute=False,outputtrajobj=True)
            self.RunTrajectory(robot,traj)

            stepsize=0.001
            expectedsteps = floor(approachoffset/stepsize)
            traj = basemanip.MoveHandStraight(direction=dot(gmodel.manip.GetTransform()[0:3,0:3],gmodel.manip.GetDirection()), ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps,maxsteps=expectedsteps,execute=False,outputtrajobj=True)
            self.RunTrajectory(robot,traj)

            print 'test_wamtaskplanwithgoal: this check is wrong'
            Tglobalgrasp = gmodel.getGlobalGraspTransform(gmodel.grasps[graspindex],collisionfree=False)
            assert(transdist(Tglobalgrasp,gmodel.manip.GetTransform()) <= 2*stepsize)
        
    def test_releasefingers(self):
        env=self.env
        self.LoadEnv('data/katanatable.env.xml')
        with env:
            robot=env.GetRobots()[0]
            m=robot.SetActiveManipulator('arm')
            body=env.GetKinBody('mug2')
            T = eye(4)
            T[0:3,3] = [-0.053,0.39,1.58643]
            body.SetTransform(T)
            initialvalues = tile(-0.43,len(m.GetGripperIndices()))
            robot.SetDOFValues(initialvalues,m.GetGripperIndices())
        taskmanip=interfaces.TaskManipulation(robot)
        traj=taskmanip.ReleaseFingers(execute=False,outputtrajobj=True)[1]
        assert(traj.GetDuration()>0)
        newvalues=traj.GetConfigurationSpecification().ExtractJointValues(traj.GetWaypoint(-1),robot,m.GetGripperIndices(),0)
        assert(transdist(initialvalues,newvalues) > 0.1 )
        self.RunTrajectory(robot,traj)

    def test_releasefingerscollision(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        with env:
            robot=env.GetRobots()[0]
            # move the robot until in collision
            Trobot=robot.GetTransform()
            Trobot[2,3] -= 1.5
            robot.SetTransform(Trobot)
            assert(env.CheckCollision(robot))
            m=robot.SetActiveManipulator('arm')
            Tmanip = m.GetTransform()
            assert( not m.CheckEndEffectorCollision(Tmanip) )
            
            taskmanip=interfaces.TaskManipulation(robot)
            traj=taskmanip.CloseFingers(execute=False,outputtrajobj=True)[1]
            self.RunTrajectory(robot,traj)
            traj=taskmanip.ReleaseFingers(execute=False,outputtrajobj=True)[1]
            self.RunTrajectory(robot,traj)
            
            body=env.GetKinBody('mug2')
            body.SetTransform(Tmanip)
            initialvalues = tile(-0.43,len(m.GetGripperIndices()))
            robot.SetDOFValues(initialvalues,m.GetGripperIndices())
            
            taskmanip=interfaces.TaskManipulation(robot)
            traj=taskmanip.CloseFingers(execute=False,outputtrajobj=True)[1]
            self.RunTrajectory(robot,traj)
            assert( m.CheckEndEffectorCollision(Tmanip) )
            traj=taskmanip.ReleaseFingers(execute=False,outputtrajobj=True)[1]
            self.RunTrajectory(robot,traj)
            
    def test_handgoal_collision(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        robot.SetDOFValues(array([ -8.44575603e-02,   1.48528347e+00,  -5.09108824e-08, 6.48108822e-01,  -4.57571203e-09,  -1.04008750e-08, 7.26855048e-10,   5.50807826e-08,   5.50807826e-08, -1.90689327e-08,   0.00000000e+00]))
        assert(env.CheckCollision(robot))
        
