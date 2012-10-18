# -*- coding: utf-8 -*-
# Copyright (C) 2011-2012 Rosen Diankov <rosen.diankov@gmail.com>
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

class TestCOLLADA(EnvironmentSetup):
    def test_collada_loading(self):
        self.log.info('test that collada import/export works for robots')
        env=self.env
        testdesc='asdfa{}<>ff\nffas\nff<f>'
        with env:
            for robotfile in g_robotfiles:
                env.Reset()
                robot0=self.LoadRobot(robotfile)
                # add a transform to test that offsets are handled correctly
                Trobot = matrixFromAxisAngle([pi/4,0,0])
                Trobot[0:3,3] = [1.0,2.0,3.0]
                robot0.SetTransform(Trobot)
                robot0.SetDescription(testdesc)
                env.Save('test.zae')
                robot1=self.LoadRobot('test.zae')
                misc.CompareBodies(robot0,robot1,epsilon=g_epsilon)

            # test if collada can store current joint values
            env.Reset()
            robot0=self.LoadRobot(g_robotfiles[0])
            robot0.SetTransform(eye(4))
            lower,upper = robot0.GetDOFLimits()
            # try to limit circular joints since they throw off precision
            values = lower+random.rand(robot0.GetDOF())*(upper-lower)
            for j in robot0.GetJoints():
                if j.IsCircular(0):
                    values[j.GetDOFIndex()] = (random.rand()-pi)*2*pi
            robot0.SetDOFValues(values)
            
            env.Save('test.dae')
            oldname = robot0.GetName()
            robot0.SetName('__dummy__')
            self.LoadEnv('test.dae')
            robot1=env.GetRobot(oldname)
            # for now have to use this precision until collada-dom can store doubles
            assert(transdist(robot0.GetDOFValues(),robot1.GetDOFValues()) <= robot0.GetDOF()*1e-4 )

    def test_colladascene_simple(self):
        self.log.info('test that collada simple import/export')
        env=self.env
        xmldata = """<kinbody name="a" scalegeometry="10">
  <translation>1 1 1</translation>
  <body name="b">
    <mass type="mimicgeom">
      <density>1000</density>
    </mass>
    <geom type="box">
      <extents>0.2 0.4 0.5</extents>
      <translation>1 0.01 0.02</translation>
    </geom>
    <geom type="box">
      <extents>0.1 0.2 0.3</extents>
      <translation>1.3 0.21 0.02</translation>
    </geom>
  </body>
</kinbody>
"""
        body = env.ReadKinBodyData(xmldata)
        env.Add(body)
        env.Save('test_colladascenes.dae')
        
        env2 = Environment()
        env2.Load('test_colladascenes.dae')
        assert(len(env2.GetBodies())==len(env.GetBodies()))
        body2=env2.GetBodies()[0]
        assert(body.GetName() == body2.GetName())
        assert(len(body2.GetLinks())==len(body.GetLinks()))
        link=body.GetLinks()[0]
        link2=body2.GetLinks()[0]
        assert(len(link2.GetGeometries())==len(link.GetGeometries()))
        
        assert(transdist(link.ComputeAABB().pos(), link2.ComputeAABB().pos()) <= g_epsilon)
        assert(transdist(link.ComputeAABB().extents(), link2.ComputeAABB().extents()) <= g_epsilon)
        for ig,g in enumerate(link.GetGeometries()):
            g2 = link2.GetGeometries()[ig]
            ab = g.ComputeAABB(eye(4))
            ab2 = g2.ComputeAABB(eye(4))
            assert(transdist(ab.pos(), ab2.pos()) <= g_epsilon)
            assert(transdist(ab.extents(), ab2.extents()) <= g_epsilon)

    def test_colladascenes(self):
        self.log.info('test that collada import/export works for scenes with multiple objects')
        env=self.env
        env2 = Environment()
        xmldata = """<environment>
  <kinbody name="b1">
    <body name="base">
      <geom type="box">
        <extents>1 1 1</extents>
      </geom>
    </body>
  </kinbody>
  <kinbody name="b2">
    <translation> 0 0 2.1</translation>
    <body name="base">
      <geom type="box">
        <extents>1 1 1</extents>
      </geom>
    </body>
  </kinbody>
</environment>
"""
        open('test_colladascenes.env.xml','w').write(xmldata)
        with env:
            with env2:
                for name in ['test_colladascenes.env.xml','data/lab1.env.xml']:
                    env.Reset()
                    env.Load(name)
                    env.Save('test_colladascenes_new.dae',Environment.SelectionOptions.Everything)
                    env2.Reset()
                    env2.Load('test_colladascenes_new.dae')
                    assert(len(env.GetBodies())==len(env2.GetBodies()))
                    for body in env.GetBodies():
                        body2 = env2.GetKinBody(body.GetName())
                        misc.CompareBodies(body,body2,epsilon=g_epsilon)

                    # try saving with external refs
                    env.Save('test_colladascenes_new.dae',Environment.SelectionOptions.Everything,{'externalref':'*'})
                    env2.Reset()
                    env2.Load('test_colladascenes_new.dae')
                    assert(len(env.GetBodies())==len(env2.GetBodies()))
                    for body in env.GetBodies():
                        body2 = env2.GetKinBody(body.GetName())
                        misc.CompareBodies(body,body2,epsilon=g_epsilon)
                
    def test_collada_dummyjoints(self):
        env=self.env
        xmldata="""<kinbody name="a">
  <body name="b1">
  </body>
  <body name="b2">
  </body>
  <body name="b3">
  </body>
  <joint name="j1" type="hinge">
    <body>b1</body>
    <body>b2</body>
  </joint>
  <joint name="j2" type="hinge" enable="false">
    <body>b2</body>
    <body>b3</body>
  </joint>
</kinbody>
        """
        body = env.ReadKinBodyData(xmldata)
        env.Add(body)
        env.Save('test_dummyjoints.dae')
        assert(len(body.GetJoints())==1)
        assert(len(body.GetPassiveJoints())==1)
        
        env2 = Environment()
        env2.Load('test_dummyjoints.dae')
        assert(len(env2.GetBodies())==len(env.GetBodies()))
        body2=env2.GetBodies()[0]
        assert(not body2.IsRobot())
        assert(body.GetName() == body2.GetName())
        assert(len(body2.GetJoints())==len(body.GetJoints()))
        assert(len(body2.GetPassiveJoints())==len(body.GetPassiveJoints()))

    def test_colladagrabbing(self):
        self.log.info('test if robot grabbing information can be exported')
        env=self.env
        with env:
            robot = self.LoadRobot('robots/pr2-beta-static.zae')
            target1 = env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.Add(target1,True)
            T1 = eye(4)
            T1[0:3,3] = [0.88,0.18,0.8]
            target1.SetTransform(T1)
            robot.SetActiveManipulator('leftarm')
            robot.Grab(target1)
            
            target2 = env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.Add(target2,True)
            T2 = matrixFromAxisAngle([pi/2,0,0])
            T2[0:3,3] = [0.88,-0.18,0.8]
            target2.SetTransform(T2)
            robot.SetActiveManipulator('rightarm')
            robot.Grab(target2)
            
            env.Save('test_colladagrabbing.dae')
            
            env2=Environment()
            env2.Load('test_colladagrabbing.dae')
            misc.CompareBodies(robot,env2.GetRobot(robot.GetName()))

    def test_collada_readexternal(self):
        self.log.info('test collada loading with external references')
        env=self.env
        env.LoadURI('openrave:/robots/schunk-lwa3.zae',{'colladaurischeme':'openrave'})
        assert(len(env.GetRobots())==1)
        
    def test_saving(self):
        self.log.info('test collada saving options')
        env=self.env
        self.LoadEnv('data/pr2test1.env.xml')

        robot=env.GetRobots()[0]
        robot.SetDOFValues(ones(robot.GetDOF()),range(robot.GetDOF()),checklimits=KinBody.CheckLimitsAction.CheckLimits)
        Trobot = matrixFromAxisAngle([pi/4,0,0])
        Trobot[0:3,3] = [1.0,2.0,3.0]
        robot.SetTransform(Trobot)
        env.Save('test_collada_savingoptions.dae',Environment.SelectionOptions.Body,{'target':'pr2'})
        
        env2=Environment()
        env2.Load('test_collada_savingoptions.dae')
        assert(len(env2.GetBodies())==1)
        misc.CompareBodies(robot,env2.GetRobot(robot.GetName()))
            
        env.Save('test_collada_savingoptions.dae',Environment.SelectionOptions.Body,[('target','pr2'),('target','mug1')])

        env2.Reset()
        env2.Load('test_collada_savingoptions.dae')
        assert(len(env2.GetBodies())==2)
        misc.CompareBodies(env.GetKinBody('pr2'),env2.GetKinBody('pr2'))
        misc.CompareBodies(env.GetKinBody('mug1'),env2.GetKinBody('mug1'))

    def test_externalref_joints(self):
        self.log.info('test basic collada saving/loading with external references')
        env=self.env
        reffile = 'openrave:/robots/schunk-lwa3.zae'
        env2=Environment()
        assert(env.LoadURI(reffile))
        robot=env.GetRobots()[0]
        robot.SetDOFValues(ones(robot.GetDOF()))
        env.Save('test_externalref_joints.dae',Environment.SelectionOptions.Everything,{'externalref':'*'})
        filedata=open('test_externalref_joints.dae','r').read()
        assert(filedata.find(reffile)>=0)
        assert(len(filedata)<7000) # should be small
        assert(env2.Load('test_externalref_joints.dae'))
        misc.CompareBodies(robot,env2.GetRobots()[0])
        assert(len(env.GetBodies())==len(env2.GetBodies()))

        env.Reset()
        env2.Reset()
        
        assert(env.Load('robots/schunk-lwa3.zae'))
        robot=env.GetRobots()[0]
        env.Save('test_externalref_joints.dae',Environment.SelectionOptions.Everything,{'externalref':'*', 'openravescheme':'testscheme'})
        filedata=open('test_externalref_joints.dae','r').read()
        assert(filedata.find('testscheme:/')>=0)
        assert(env2.Load('test_externalref_joints.dae',{'openravescheme':'testscheme'}))
        misc.CompareBodies(env.GetRobots()[0],env2.GetRobots()[0])
        assert(len(env.GetBodies())==len(env2.GetBodies()))

        env.Reset()
        env.Load('robots/barrett-hand.zae')
        robot=env.GetRobots()[0]
        env.Save('test_externalref_joints.dae',Environment.SelectionOptions.Everything,{'externalref':'*'})
        env2.Reset()
        assert(env2.Load('test_externalref_joints.dae'))
        misc.CompareBodies(robot,env2.GetRobots()[0])
        assert(len(env.GetBodies())==len(env2.GetBodies()))

    def test_externalref_scene(self):
        xmldata = '''<Environment>
  <Robot file="robots/barrett-wamhand.dae" name="BarrettWAM">
    <translation>-0.8 0.14 1</translation>
  </Robot>

  <KinBody name="floorwalls">
    <Body name="basefloor" type="static">
      <Translation>0 0 0</Translation>
      <Geom type="box">
        <extents>2.5 2.5 0.005</extents>
        <translation>0 0 -0.005</translation>
        <diffuseColor>.6 .6 .6</diffuseColor>
      	<ambientColor>0.6 0.6 0.6</ambientColor>
      </Geom>
      <Geom type="box">
        <extents>2.5 0.01 0.2</extents>
        <translation>0 -2.5 0.2</translation>
        <diffuseColor>.6 .6 .6</diffuseColor>
      	<ambientColor>0.6 0.6 0.6</ambientColor>
      </Geom>
      <Geom type="box">
        <extents>2.5 0.01 0.2</extents>
        <translation>0 2.5 0.2</translation>
        <diffuseColor>.6 .6 .6</diffuseColor>
      	<ambientColor>0.6 0.6 0.6</ambientColor>
      </Geom>
      <Geom type="box">
        <extents>0.01 2.5 0.2</extents>
        <translation>2.5 0 0.2</translation>
        <diffuseColor>.6 .6 .6</diffuseColor>
      	<ambientColor>0.6 0.6 0.6</ambientColor>
      </Geom>
      <Geom type="box">
        <extents>0.01 2.5 0.2</extents>
        <translation>-2.5 0 0.2</translation>
        <diffuseColor>.6 .6 .6</diffuseColor>
      	<ambientColor>0.6 0.6 0.6</ambientColor>
      </Geom>
    </Body>
  </KinBody>
  <KinBody name="pole">
    <translation>-0.312 0.416 1</translation>
    <Body name="basepole" type="static">
      <Geom type="box">
        <extents>0.05 0.05 1</extents>
        <diffuseColor>1 .2 .2</diffuseColor>
      </Geom>
    </Body>
  </KinBody>
  <KinBody name="pole2">
    <translation>1.3 0.6 1</translation>
    <Body name="basepole" type="static">
      <Geom type="box">
        <extents>0.05 0.05 1</extents>
        <diffuseColor>1 .2 .2</diffuseColor>
      </Geom>
    </Body>
  </KinBody>
  <KinBody name="pole3">
    <translation>0.8 -0.9 1</translation>
    <Body name="basepole" type="static">
      <Geom type="box">
        <extents>0.05 0.05 1</extents>
        <diffuseColor>1 .2 .2</diffuseColor>
      </Geom>
    </Body>
  </KinBody>

  <KinBody name="wall1">
    <Translation>-0.173 -0.247 0</Translation>
    <Body name="basewall" type="static">
      <Geom type="box">
        <Translation>-0.735 -0.80 0.675</Translation>
        <extents>0.07 0.71 0.73</extents>
        <diffuseColor>.64 .64 .64</diffuseColor>
      </Geom>
      <Geom type="box">
        <Translation>-1.235 -0.46 0.7</Translation>
        <extents>0.54 0.35 0.06</extents>
        <diffuseColor>.64 .64 .64</diffuseColor>
      </Geom>
      <Geom type="box">
        <Translation>-1.235 -0.76 0.675</Translation>
        <extents>0.54 0.06 0.71</extents>
        <diffuseColor>.64 .64 .64</diffuseColor>
      </Geom>
    </Body>
  </KinBody>
  
  <KinBody name="mug1" file="data/mug1.dae">
    <Translation> -0.0568 -0.2406 0.7550</Translation>
  </KinBody>
  <KinBody name="mug2" file="data/mug1.dae">
    <Translation>-0.02   0.15    0.7550</Translation>
  </KinBody>
  <KinBody name="mug3" file="data/mug1.dae">
    <Translation>0.0854  -0.0769    0.7550</Translation>
  </KinBody>
  <KinBody name="mug4" file="data/mug1.dae">
    <Translation>-0.23   -0.1446    0.7550</Translation>
  </KinBody>
  <KinBody name="mug5" file="data/mug1.dae">
    <Translation>-0.1   -0.0151    0.7550</Translation>
  </KinBody>
  <KinBody name="mug6" file="data/mug1.dae">
    <Translation>-0.2   0.11    0.7550</Translation>
  </KinBody>

  <KinBody name="table">
    <Translation>-0.4966 1.0164 0.9</Translation>
    <body name="basetable">
      <geom type="box">
        <translation>0 0.2 0</translation>
        <extents>0.2 0.01 0.01</extents>
      </geom>
      <geom type="box">
        <translation>0 -0.2 0</translation>
        <extents>0.2 0.01 0.01</extents>
      </geom>
    </body>
  </KinBody>
</Environment>
'''
        env=self.env
        self.LoadDataEnv(xmldata)
        env.Save('test_externalref_scene.dae',Environment.SelectionOptions.Everything,{'externalref':'*'})
        env2 = Environment()
        env2.Load('test_externalref_scene.dae')
        assert(len(env.GetBodies())==len(env2.GetBodies()))
        for body in env.GetBodies():
            self.log.info(body.GetName())
            body2 = env2.GetKinBody(body.GetName())
            misc.CompareBodies(body,body2,epsilon=g_epsilon)

    def test_writekinematicsonly(self):
        self.log.info('test writing kinematics only')
        env=self.env
        robot = self.LoadRobot('robots/pr2-beta-static.zae')
        env.Save('test_writekinematicsonly.dae',Environment.SelectionOptions.Body,{'target':robot.GetName(), 'skipwrite':'visual readable sensors physics'})
        filedata=open('test_writekinematicsonly.dae','r').read()
        assert(len(filedata)<400000) # should be ~331kb
        env2=Environment()
        env2.Load('test_writekinematicsonly.dae')
        robot2=env2.GetRobots()[0]
        misc.CompareBodies(robot,robot2,comparegeometries=False,comparesensors=False,comparemanipulators=True,comparegrabbed=False,comparephysics=False,epsilon=1e-10)

        env.Save('test_writekinematicsonly.dae',Environment.SelectionOptions.Body,{'target':robot.GetName(), 'skipwrite':'geometry readable sensors physics'})
        filedata=open('test_writekinematicsonly.dae','r').read()
        assert(len(filedata)<400000) # should be ~331kb
        env2.Reset()
        env2.Load('test_writekinematicsonly.dae')
        robot2=env2.GetRobots()[0]

        # check kinematics hashes since IK relies on it
        for m in robot.GetManipulators():
            m2 = robot2.GetManipulator(m.GetName())
            assert(m.GetKinematicsStructureHash()==m2.GetKinematicsStructureHash())

    def test_colladamerge(self):
        self.log.info('test that loading collada with prefixes')
        xmldata="""<robot>
 <robot file="robots/schunk-lwa3.zae"></robot>
 <robot prefix="hand_" file="robots/pumagripper.zae"></robot>
 <kinbody>
   <body name="hand_Puma6">
     <offsetfrom>link7</offsetfrom>
     <translation>0.03 0 0</translation>
     <rotationaxis>0 1 0 90</rotationaxis>
   </body>
   <joint type="hinge" enable="false" name="dummy">
     <body>link7</body>
     <body>hand_Puma6</body>
     <limits>0 0</limits>
   </joint>
 </kinbody>
 <manipulator name="myarm">
   <base>base</base>
   <effector>hand_Puma6</effector>
   <direction>1 0 0</direction>
 </manipulator>
</robot>
"""
        robot=self.LoadRobotData(xmldata)
        assert(robot.GetActiveDOF()==8)
        
    def test_external_extrainfo(self):
        self.log.info('test saving extra info along with external references')
        env=self.env
        robot=self.LoadRobot('robots/schunk-lwa3.zae')
        enabled = [False]*len(robot.GetLinks())
        for ilink,link in enumerate(robot.GetLinks()):
            enabled[ilink] = ilink%2
            link.Enable(enabled[ilink])
        env.Save('test_external_extrainfo.dae',Environment.SelectionOptions.Everything,{'externalref':'*', 'forcewrite':'link_collision_state'})

        env2=Environment()
        env2.Load('test_external_extrainfo.dae')
        robot2=env2.GetRobots()[0]
        misc.CompareBodies(robot,robot2)

        with env:
            # change limits, add manipulators, etc
            minfo=Robot.ManipulatorInfo()
            minfo._name = 'mynewmanip'
            minfo._sBaseLinkName = robot.GetLinks()[0].GetName()
            minfo._sEffectorLinkName = robot.GetLinks()[4].GetName()
            minfo._tLocalTool = matrixFromAxisAngle([1,1,1])
            minfo._tLocalTool[0:3,3] = [0.1,0.2,0.3]
            minfo._vdirection = [-1,0,0]
            minfo._vGripperJointNames = ['j6']
            minfo._vClosingDirection = [-1.0]
            robot.AddManipulator(minfo)
            #robot.SetDOFLimits(-linspace(0.4,0.8,robot.GetDOF()),linspace(1.4,1.8,robot.GetDOF()))
            #robot.SetDOFVelocityLimits(linspace(1,10,robot.GetDOF()))
            #robot.SetDOFAccelerationLimits(linspace(10,100,robot.GetDOF()))
            #robot.SetDOFWeights(linspace(0.1,0.5,robot.GetDOF()))
            env.Save('test_external_extrainfo2.dae',Environment.SelectionOptions.Everything,{'externalref':'*', 'forcewrite':'*'})
        
        env2=Environment()
        env2.Load('test_external_extrainfo2.dae')
        robot2=env2.GetRobots()[0]
        misc.CompareBodies(robot,robot2)
