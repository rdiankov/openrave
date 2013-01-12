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

class RunCollision(EnvironmentSetup):
    def __init__(self,collisioncheckername):
        self.collisioncheckername = collisioncheckername
    def setup(self):
        EnvironmentSetup.setup(self)
        self.env.SetCollisionChecker(RaveCreateCollisionChecker(self.env,self.collisioncheckername))

    def test_basic(self):
        env=self.env
        with env:
            self.LoadEnv('data/hironxtable.env.xml')
            robot=env.GetRobots()[0]
            env.CheckCollision(robot)
            newobject=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.Add(newobject,True)
            box=RaveCreateKinBody(env,'')
            env.CheckCollision(box)
            box.InitFromBoxes(array([[0,0,0,1,1,1]]),True)
            box.SetName('box')
            env.Add(box,True)
            
    def test_collisioncaching(self):
        filenames = ['robots/barrettwam.robot.xml']
        env=self.env
        for filename in filenames:
            env.Reset()
            robot=env.ReadRobotURI(filename)
            for i in range(10):
                env.Add(robot)
                lower,upper = robot.GetDOFLimits()
                v = random.rand()*(upper-lower)+lower
                robot.SetDOFValues(v)
                check=robot.CheckSelfCollision()
                robot.SetDOFValues(v)
                assert(check==robot.CheckSelfCollision())
                env.Remove(robot)

    def test_selfcollision(self):
        with self.env:
            self.LoadEnv('data/lab1.env.xml')
            target1 = self.env.GetKinBody('mug1')
            target2 = self.env.GetKinBody('mug2')
            target2.SetTransform(target1.GetTransform())
            assert(self.env.CheckCollision(target1))
            robot = self.env.GetRobots()[0]
            report = CollisionReport()
            assert(not robot.CheckSelfCollision(report))
            robot.Grab(target1)
            assert(not robot.CheckSelfCollision(report))
            assert(not target1.CheckSelfCollision())
            assert(self.env.CheckCollision(target1,report))

    def test_selfcollision_joinxml(self):
        testrobot_xml="""<Robot>
  <KinBody>
    <!-- add a segway model to the base link -->
    <Body name="segway">
      <translation>0 0 0.305</translation>
      <Geom type="box">
        <diffusecolor>1.0 0 0</diffusecolor>
        <extents>0.33 0.255 0.305</extents>
      </Geom>
      <Geom type="cylinder">
        <translation>0.18 0.32 -0.195</translation>
        <diffusecolor>0.3 0.3 0.3</diffusecolor>
        <radius>0.11</radius>
	<height>0.13</height>
      </Geom>
      <Geom type="cylinder">
        <diffusecolor>0.3 0.3 0.3</diffusecolor>
        <translation>-0.18 0.32 -0.195</translation>
        <radius>0.11</radius>
	<height>0.13</height>
      </Geom>
      <Geom type="cylinder">
        <diffusecolor>0.3 0.3 0.3</diffusecolor>
        <translation>0.18 -0.32 -0.195</translation>
        <radius>0.11</radius>
	<height>0.13</height>
      </Geom>
      <Geom type="cylinder">
        <diffusecolor>0.3 0.3 0.3</diffusecolor>
        <translation>-0.18 -0.32 -0.195</translation>
        <radius>0.11</radius>
	<height>0.13</height>
      </Geom>
      <Geom type="box">
	<diffusecolor>0 0 1.0</diffusecolor>
	<translation>-0.31 0 0.635</translation>
        <extents>0.05 0.255 0.33</extents>
      </Geom>
      <Geom type="box">
	<diffusecolor>0 0 1.0</diffusecolor>
	<translation>-0.31 0.205 1.085</translation>
        <extents>0.05 0.05 0.17</extents>
      </Geom>
      <Geom type="box">
	<diffusecolor>0 0 1.0</diffusecolor>
	<translation>-0.31 -0.205 1.085</translation>
        <extents>0.05 0.05 0.17</extents>
      </Geom>
      <Geom type="box">
	<diffusecolor>0 0 1.0</diffusecolor>
	<translation>-0.31 0 1.305</translation>
        <extents>0.05 0.255 0.05</extents>
      </Geom>
      <Geom type="box">
	<diffusecolor>0 0 0</diffusecolor>
	<translation>-0.615 0 0.635</translation>
        <extents>0.255 0.255 0.01</extents>
      </Geom>
      <Geom type="box">
	<diffusecolor>0 0 0</diffusecolor>
	<translation>-0.361 0 1.085</translation>
        <extents>0.001 0.155 0.17</extents>
      </Geom>
      <mass type="custom">
        <total>40</total>
      </mass>
    </Body>
  </KinBody>
  <Robot file="robots/barrettwam.robot.xml"></Robot>
  <KinBody>
    <body name="wam0">
      <!-- shift wam0 to align correctly with segway base -->
      <translation>0.22 0.14 0.346</translation>
      <translation>-0.099 -0.14 0.61</translation>
    </body>
    <joint name="joint4" type="hinge" enable="false">
      <body>segway</body>
      <body>wam0</body>
      <limits>0 0</limits>
    </joint>
  </KinBody>
</Robot>
"""
        with self.env:
            robot=self.LoadRobotData(testrobot_xml)
            robot.SetDOFValues([-0.91,2.05],[1,3])
            assert(robot.CheckSelfCollision())

    def test_known_collisions(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        robot.SetDOFValues([ -8.44575603e-02,   1.48528347e+00,  -5.09108824e-08, 6.48108822e-01,  -4.57571203e-09,  -1.04008750e-08, 7.26855048e-10,   5.50807826e-08,   5.50807826e-08, -1.90689327e-08,   0.00000000e+00])
        assert(env.CheckCollision(robot))
        
    def test_collisioncallbacks(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        robot=env.GetRobots()[0]
        
        reports = []
        def collisioncallback(report,fromphysics):
            assert(not fromphysics)
            reports.append(report)
            return CollisionAction.DefaultAction
        
        handle = env.RegisterCollisionCallback(collisioncallback)
        assert(not env.CheckCollision(robot))
        assert(len(reports)==0)

        assert(env.CheckCollision(env.GetKinBody('mug1')))
        assert(len(reports)==1)

        def collisioncallback2(report,fromphysics):
            return CollisionAction.Ignore

        handle2 = env.RegisterCollisionCallback(collisioncallback2)
        assert(not env.CheckCollision(env.GetKinBody('mug1')))

        del reports[:]
        handle2.Close()
        assert(env.CheckCollision(env.GetKinBody('mug1')))
        assert(len(reports)==1)

        handle.Close()
        assert(env.CheckCollision(env.GetKinBody('mug1')))
        assert(len(reports)==1)

    def test_activedofdistance(self):
        self.log.debug('test distance computation with active dofs')
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        with env:
            pqp = RaveCreateCollisionChecker(env,'pqp')
            pqp.InitEnvironment()
            robot=env.GetRobots()[0]
            manip=robot.GetActiveManipulator()
            report = CollisionReport()
            pqp.SetCollisionOptions(CollisionOptions.Contacts|CollisionOptions.Distance)
            pqp.CheckCollision(manip.GetEndEffector(),report=report)
            assert(abs(report.minDistance-0.38737) < 0.01 )
            assert(report.plink1 == manip.GetEndEffector())
            assert(report.plink2 == env.GetKinBody('pole').GetLinks()[0])
            
            pqp.CheckCollision(robot,report=report)
            assert(abs(report.minDistance-0.0027169) < 0.01 )
            assert(report.plink1 == robot.GetLink('segway'))
            assert(report.plink2 == env.GetKinBody('floorwalls').GetLinks()[0])
            
            pqp.SetCollisionOptions(CollisionOptions.Contacts|CollisionOptions.Distance|CollisionOptions.ActiveDOFs)
            robot.SetActiveDOFs(manip.GetArmIndices())
            pqp.CheckCollision(robot,report=report)
            assert(abs(report.minDistance-0.29193971893003506) < 0.01 )
            assert(report.plink1 == robot.GetLink('wam1'))
            assert(report.plink2 == env.GetKinBody('pole').GetLinks()[0])
        
#generate_classes(RunCollision, globals(), [('ode','ode'),('bullet','bullet')])

class test_ode(RunCollision):
    def __init__(self):
        RunCollision.__init__(self, 'ode')

# class test_bullet(RunCollision):
#     def __init__(self):
#         RunCollision.__init__(self, 'bullet')
# 
