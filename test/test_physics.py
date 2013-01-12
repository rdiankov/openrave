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

class RunPhysics(EnvironmentSetup):
    def __init__(self,physicsenginename):
        self.physicsenginename = physicsenginename
    def setup(self):
        EnvironmentSetup.setup(self)
        self.env.SetPhysicsEngine(RaveCreatePhysicsEngine(self.env,self.physicsenginename))

    def test_basic(self):
        scene = 'data/hanoi.env.xml'
        self.LoadEnv(scene)
        self.env.SetPhysicsEngine(RaveCreatePhysicsEngine(self.env,self.physicsenginename))
        self.env.SetPhysicsEngine(RaveCreatePhysicsEngine(self.env,self.physicsenginename))
        self.env.GetPhysicsEngine().SetGravity([0,0,-9.81])
        
    def test_static(self):
        env=self.env
        scene = 'data/hanoi.env.xml'
        self.LoadEnv(scene)
        with env:
            robot=env.GetRobots()[0]
            Trobot = robot.GetTransform()
            env.GetPhysicsEngine().SetGravity([0,0,-9.81])

            assert(not robot.GetLinks()[0].IsStatic())
            robot.GetLinks()[0].SetStatic(True)

            bodynames = ['data/lego2.kinbody.xml', 'data/lego4.kinbody.xml', 'data/mug1.kinbody.xml']
            numbodies = 0
            env.StopSimulation()
            env.StartSimulation(timestep=0.01)

        while numbodies < 5:
            with env:
                body = env.ReadKinBodyURI(bodynames[random.randint(len(bodynames))])
                body.SetName('body%d'%numbodies)
                numbodies += 1
                env.Add(body)
                T = eye(4)
                T[0:3,3] = array((-0.5,-0.5,2))+0.4*random.rand(3)
                body.SetTransform(T)

            time.sleep(0.4)
        with env:
            assert( transdist(robot.GetTransform(),Trobot) <= g_epsilon )

    def test_simtime(self):
        env=self.env
        env.GetPhysicsEngine().SetGravity([0,0,-9.81])

        with env:
            body = env.ReadKinBodyURI('data/lego2.kinbody.xml')
            body.SetName('body')
            env.Add(body)
            Tinit = eye(4)
            Tinit[2,3] = 3
            body.SetTransform(Tinit)
        
        env.StartSimulation(0.01,realtime=True)
        starttime = 1e-6*env.GetSimulationTime()
        realtime0 = time.time()
        time.sleep(1)
        env.StopSimulation()
        realtime1 = time.time()
        simtime0 = 1e-6*env.GetSimulationTime()
        assert( abs(simtime0-starttime-1) < 0.05 )
        with env:
            T = body.GetTransform()
            assert(abs(T[2,3]-Tinit[2,3]) > 0.2)
        time.sleep(2)
        with env:
            T2 = body.GetTransform()
            assert(abs(T[2,3]-T2[2,3]) < g_epsilon )
        env.StartSimulation(timestep=0.01,realtime=True)
        realtime2 = time.time()
        time.sleep(1)
        env.StopSimulation()
        realtime3 = time.time()
        assert( abs(1e-6*env.GetSimulationTime()-starttime-(realtime3-realtime2)-(realtime1-realtime0)) < 0.05 )
        with env:
            T2 = body.GetTransform()
            assert(abs(T[2,3]-T2[2,3])>0.2)
            body.SetVelocity([0,0,0],[0,0,0])
        
        simtime1 = 1e-6*env.GetSimulationTime()
        for i in range(int((simtime0-starttime)*100)):
            env.StepSimulation(0.01)
        with env:
            T3 = body.GetTransform()
            self.log.info('differences: %f, %f', (T[2,3]-Tinit[2,3]),(T3[2,3]-T2[2,3]))
            assert( abs((T[2,3]-Tinit[2,3]) - (T3[2,3]-T2[2,3])) < 0.01)
        assert(abs(1e-6*env.GetSimulationTime()-simtime1-(simtime0-starttime)) < g_epsilon)

        env.StartSimulation(timestep=0.01,realtime=False)
        simtime2 = 1e-6*env.GetSimulationTime()
        while True:
            if 1e-6*env.GetSimulationTime() > simtime2+1:
                break
        env.StopSimulation()

    def test_kinematics(self):
        log.info("test that physics kinematics are consistent")
        env=self.env
        with env:
            for robotfilename in g_robotfiles+['testdata/bobcat.robot.xml']:
                env.Reset()
                self.LoadEnv(robotfilename,{'skipgeometry':'1'})
                robot=self.env.GetRobots()[0]
                robot.GetLinks()[0].SetStatic(True)
                for i in range(1000):
                    env.StepSimulation(0.001)
                curvalues = robot.GetDOFValues()
                curlinks = robot.GetLinkTransformations()
                robot.SetDOFValues(curvalues)
                newlinks = robot.GetLinkTransformations()
                assert(transdist(curlinks,newlinks) <= 10*g_epsilon*len(curlinks))
                
    def test_rotationaxis(self):
        if self.physicsenginename != 'ode':
            return
        
        env=self.env
        if self.physicsenginename == 'ode':
            properties = """<odeproperties>
              <friction>3.5</friction>
              <selfcollision>1</selfcollision>
              <erp>0.6</erp>
              <cfm>0.00000001</cfm>
            </odeproperties>"""
        else:
            properties = ''
            
        xmldata = """<environment>
          <Robot file="robots/diffdrive_caster.robot.xml">
            <Translation>4 6 0.1</Translation>
            <RotationAxis>0 0 1 270</RotationAxis>
          </Robot>

          <KinBody name="floor">
            <Body type="static">
              <Translation>0 0 -0.015</Translation>
              <Geom type="box">
                <extents>10 10 0.005</extents>
                <diffuseColor>.3 1 .3</diffuseColor>
                <ambientColor>0.3 1 0.3</ambientColor>
              </Geom>
            </Body>
          </KinBody>


          <physicsengine type="%s">
            %s
          </physicsengine>
        </environment>
        """%(self.physicsenginename,properties)
        self.LoadDataEnv(xmldata)
        robot = env.GetRobots()[0]
                    
        with env:
            env.GetPhysicsEngine().SetGravity([0,0,-9.8])
            robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis,[0,0,1])
            orgvalues = robot.GetActiveDOFValues()
            for i in range(100):
                env.StepSimulation(0.01)
                values = robot.GetActiveDOFValues()
                assert(sum(abs(values-orgvalues))<1)

    def test_odenoslip(self):
        log.info("test that an ode option can prevent a body from slipping")
        xmldata = '''<Environment>
  <camtrans>-0.032706 -22.004591 -0.003697</camtrans>
  <camrotationaxis>-0.999781 -0.008689 -0.019015 89.161379</camrotationaxis>
  <camfocal>22.008177</camfocal>
  <Robot name="fallbot">
    <Translation>0 0 .2</Translation>
    <RotationAxis>0 1 0 20</RotationAxis>
    <KinBody name="fallbox" makejoinedlinksadjacent="true">
      <Body name="box" type="dynamic">
        <Geom type="box">
          <extents>.5 .5 .1</extents>
          <diffuseColor>.9 .9 .4</diffuseColor>
          <ambientColor>0.7 0.7 0.4</ambientColor>
        </Geom>
        <mass type="box">
          <extents>.5 .5 .1</extents>
          <density>1</density>
        </mass>
      </Body>
      <Body name="box2" type="dynamic">
        <offsetfrom>box</offsetfrom>
        <Translation>0 0 .2</Translation>
        <Geom type="box">
          <extents>.5 .5 .1</extents>
          <diffuseColor>.9 .9 .4</diffuseColor>
          <ambientColor>0.7 0.7 0.4</ambientColor>
        </Geom>
        <mass type="box">
          <extents>.5 .5 .1</extents>
          <density>1</density>
        </mass>
      </Body>
      <Joint name="boxjoint" type="hinge" enabled="false">
        <Body>box</Body>
        <Body>box2</Body>
        <limits>0 0</limits>
      </Joint>
    </KinBody>
  </Robot>
  <KinBody name="floor">
    <RotationAxis>0 1 0 20</RotationAxis>
    <Body type="static">
      <Translation>0 0 -.1</Translation>
      <Geom type="box">
        <extents>10 10 .1</extents>
        <diffuseColor>.41 .4 .4</diffuseColor>
        <ambientColor>0.4 0.5 0.6</ambientColor>
      </Geom>
    </Body>
  </KinBody>
  <physicsengine type="ode">
    <odeproperties>
      <friction>.37</friction>
      <gravity>0 0 -9.8</gravity>
      <selfcollision>1</selfcollision>
      <erp>.5</erp>
      <cfm>.000001</cfm>
      <dcontactapprox>1</dcontactapprox>
    </odeproperties>
  </physicsengine>
</Environment>
'''
        with self.env:
            self.env.LoadData(xmldata) 
            nonmovingbody = self.env.GetKinBody('fallbot')
            T0 = nonmovingbody.GetTransform()
            for i in range(1000):
                self.env.StepSimulation(0.01)
            T1 = nonmovingbody.GetTransform()
            assert(transdist(T0,T1)<=0.5)

    def test_applytorque(self):
        log.info('test if torque can be applied')
        self.LoadEnv('data/lab1.env.xml')
        env=self.env
        with env:
            env.GetPhysicsEngine().SetGravity(numpy.array((0,0,-9.8)))
            robot = env.GetRobots()[0]
            robot.GetLinks()[0].SetStatic(True)
            # test the torque call
            robot.SetDOFTorques(numpy.ones(robot.GetDOF()),True)
            for i in range(10):
                env.StepSimulation(0.01)

#generate_classes(RunPhysics, globals(), [('ode','ode'),('bullet','bullet')])

class test_ode(RunPhysics):
    def __init__(self):
        RunPhysics.__init__(self, 'ode')

# class test_bullet(RunPhysics):
#     def __init__(self):
#         RunPhysics.__init__(self, 'bullet')
#         
