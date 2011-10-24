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

class TestCollision(EnvironmentSetup):
    def setup(self):
        EnvironmentSetup.setup(self)
        # select collision engine here
        self.env.SetCollisionChecker(RaveCreateCollisionChecker(self.env,'bullet'))

    def test_basic(self):
        env=self.env
        with env:
            env.Load('data/hironxtable.env.xml')
            robot=env.GetRobots()[0]
            env.CheckCollision(robot)
            newobject=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.AddKinBody(newobject,True)
            box=RaveCreateKinBody(env,'')
            env.CheckCollision(box)
            box.InitFromBoxes(array([[0,0,0,1,1,1]]),True)
            box.SetName('box')
            env.AddKinBody(box,True)
            
    def test_collisioncaching(self):
        filenames = ['robots/barrettwam.robot.xml']
        env=self.env
        for filename in filenames:
            env.Reset()
            robot=env.ReadRobotURI(filename)
            for i in range(10):
                env.AddRobot(robot)
                lower,upper = robot.GetDOFLimits()
                v = random.rand()*(upper-lower)+lower
                robot.SetDOFValues(v)
                check=robot.CheckSelfCollision()
                robot.SetDOFValues(v)
                assert(check==robot.CheckSelfCollision())
                env.Remove(robot)

    def test_selfcollision(self):
        with self.env:
            self.env.Load('data/lab1.env.xml')
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
            robot=self.env.ReadRobotXMLData(testrobot_xml)
            self.env.AddRobot(robot)
            robot.SetDOFValues([-0.91,2.05],[1,3])
            assert(robot.CheckSelfCollision())
