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
from subprocess import Popen, PIPE
import shutil
import threading

class TestEnvironment(EnvironmentSetup):
    def test_load(self):
        env=self.env
        self.LoadEnv('../src/models/WAM/wam0.iv')
        
        for fullfilename in locate('*.xml','../src/data'):
            self.log.info('loading: %s',fullfilename)
            env.Reset()
            self.LoadEnv(fullfilename)
            
    def test_loadnogeom(self):
        env=self.env
        self.LoadEnv('robots/pr2-beta-static.zae',{'skipgeometry':'1'})
        assert(len(env.GetBodies())==1)
        robot=env.GetRobots()[0]
        trimesh=env.Triangulate(robot)
        assert(len(trimesh.vertices)==0)
        
    def test_misc(self):
        env=self.env
        assert(env.plot3([0,0,0],10)==None) # no viewer attached
        self.LoadEnv('data/lab1.env.xml')
        bodies = dict([(b,b.GetEnvironmentId()) for b in env.GetBodies()])
        assert(env.GetBodies()[0] in bodies)
        assert(bodies[env.GetBodies()[0]] == env.GetBodies()[0].GetEnvironmentId())
        s = 'this is a test string'
        env.SetUserData(s)
        assert(env.GetUserData()==s)
            
    def test_uri(self):
        env=self.env
        xml="""<environment>
  <kinbody file="data/jsk-plate.zae"/>
</environment>"""
        self.LoadDataEnv(xml)
        assert(env.GetBodies()[0].GetURI().find('data/jsk-plate.zae') >= 0)

    def test_scalegeometry(self):
        env=self.env
        with env:
            scalefactor = array([2.0,3.0,4.0])
            body1=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.Add(body1,True)
            renderfilename = body1.GetLinks()[0].GetGeometries()[0].GetRenderFilename()
            body1.SetTransform(eye(4))
            body2=env.ReadKinBodyURI('data/mug1.kinbody.xml',{'scalegeometry':'%f %f %f'%tuple(scalefactor)})
            env.Add(body2,True)
            body2.SetTransform(eye(4))
            ab1=body1.ComputeAABB()
            ab2=body2.ComputeAABB()
            assert( transdist(ab1.pos()*scalefactor,ab2.pos()) <= g_epsilon )
            assert( transdist(ab1.extents()*scalefactor,ab2.extents()) <= g_epsilon )
            for link in body2.GetLinks():
                for geom in link.GetGeometries():
                    assert( transdist(geom.GetRenderScale(),scalefactor) <= g_epsilon )
            if len(body1.GetLinks()) == 1:
                body3 = env.ReadKinBodyURI(renderfilename,{'scalegeometry':'%f %f %f'%tuple(scalefactor)})
                env.Add(body3,True)
                body3.SetTransform(eye(4))
                ab3=body3.ComputeAABB()
                assert( transdist(ab3.pos(),ab2.pos()) <= g_epsilon )
                assert( transdist(ab3.extents(),ab2.extents()) <= g_epsilon )
                for link in body3.GetLinks():
                    for geom in link.GetGeometries():
                        assert( transdist(geom.GetRenderScale(),scalefactor) <= g_epsilon )
            
    def test_collada(self):
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
                    env.Save('test_colladascenes_new.dae',Environment.SelectionOptions.Everything,'')
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

    def test_unicode(self):
        env=self.env
        name = 'テスト名前'.decode('utf-8')
        body=RaveCreateKinBody(env,'')
        body.InitFromBoxes(array([[0,0,0,1,1,1]]),True)
        body.SetName(name)
        env.Add(body)
        assert(body.GetName()==name)
        assert(unicode(body.GetName())==name)

        openrave_config = Popen(['openrave-config','--share-dir'],stdout=PIPE)
        share_dir = openrave_config.communicate()[0].strip()
        srcfile = os.path.join(share_dir,'models','WAM','wam0.iv')
        destfile = '新しいファイル.iv'.decode('utf-8')
        robotname = 'ロボット名'.decode('utf-8')
        linkname = 'テスト'.decode('utf-8')
        shutil.copyfile(srcfile,destfile)
        urobotxml = u"""<?xml version="1.0" encoding="utf-8"?>
<robot name="%s">
  <kinbody>
    <body name="%s">
      <geom type="trimesh">
        <data>%s</data>
      </geom>
    </body>
  </kinbody>
</robot>
"""%(robotname,linkname,destfile)
        robotxml = urobotxml.encode('utf-8')
        robot=self.LoadRobotData(robotxml)
        assert(robot.GetName() == robotname)
        assert(robot.GetLinks()[0].GetName() == linkname)
        assert(unicode(robot.GetName()).encode('euc-jp') == robotname.encode('euc-jp'))
        env.Remove(robot)

        robot=self.LoadRobotData(robotxml)
        assert(robot.GetName() == robotname)
        assert(unicode(robot.GetName()).encode('euc-jp') == robotname.encode('euc-jp'))
        assert(robot.GetLinks()[0].GetName() == linkname)
                    
    def test_cloneplan(self):
        env=self.env
        self.LoadEnv('data/lab1.env.xml')
        cloningoptions = [CloningOptions.Bodies | CloningOptions.RealControllers | CloningOptions.Simulation, CloningOptions.Bodies | CloningOptions.RealControllers]
        for options in cloningoptions:
            env2 = env.CloneSelf(options)
            misc.CompareEnvironments(env,env2,epsilon=g_epsilon)
            robot = env2.GetRobots()[0]
            basemanip = interfaces.BaseManipulation(robot)
            base_angle = zeros(len(robot.GetActiveManipulator().GetArmIndices()))
            base_angle[0] = 1
            basemanip.MoveManipulator(base_angle)
            while not robot.GetController().IsDone():
                env2.StepSimulation(0.01)
            env2.Destroy()

    def test_clone_basic(self):
        env=self.env
        self.LoadEnv('data/pr2test1.env.xml')
        with env:
            robot=env.GetRobots()[0]
            robot.SetActiveManipulator('leftarm')
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load(checkforloaded=False):
                ikmodel.autogenerate()

            clonedenv = Environment()
            starttime=time.time()
            clonedenv.Clone(env, CloningOptions.Bodies)
            endtime=time.time()-starttime
            self.log.info('clone time: %fs',endtime)
            print endtime
            misc.CompareEnvironments(env,clonedenv,epsilon=g_epsilon)
            clonedrobot = clonedenv.GetRobot(robot.GetName())
            assert(clonedrobot.GetActiveManipulator().GetName() == robot.GetActiveManipulator().GetName())
            assert(clonedrobot.GetActiveManipulator().GetIkSolver() is not None)

            # change robot and check if cloning is quick
            Trobot=robot.GetTransform()
            Trobot[0,3] += 1.5
            robot.SetTransform(Trobot)
            starttime=time.time()
            clonedenv.Clone(env, CloningOptions.Bodies)
            endtime=time.time()-starttime
            self.log.info('new clone time: %fs',endtime)
            assert(endtime <= 0.05)
            misc.CompareEnvironments(env,clonedenv,epsilon=g_epsilon)

            env.Remove(env.GetKinBody('mug1'))
            mug2body = env.ReadKinBodyURI('data/mug2.kinbody.xml')
            env.Add(mug2body,True)
            starttime=time.time()
            clonedenv.Clone(env, CloningOptions.Bodies)
            endtime=time.time()-starttime
            self.log.info('new clone time: %fs',endtime)
            assert(endtime <= 0.1)
            misc.CompareEnvironments(env,clonedenv,epsilon=g_epsilon)

            robot.Grab(mug2body)
            starttime=time.time()
            clonedenv.Clone(env, CloningOptions.Bodies)
            endtime=time.time()-starttime
            self.log.info('new clone time: %fs',endtime)
            assert(endtime <= 0.05)
            misc.CompareEnvironments(env,clonedenv,epsilon=g_epsilon)
            
    def test_multithread(self):
        self.log.info('test multiple threads accessing same resource')
        def mythread(env,threadid):
            for counter in range(3):
                with env:
                    self.log.info('%s: %s',threadid,counter)
                    env.Reset()
                    env.Load('data/lab1.env.xml')
                time.sleep(0.1)

        threads = []
        for ithread in range(4):
            t = threading.Thread(target=mythread,args=(self.env,'t%d'%ithread))
            t.start()
            threads.append(t)
        for t in threads:
            t.join()

    def test_dataccess(self):
        RaveDestroy()
        os.environ['OPENRAVE_DATA'] = os.path.join(os.getcwd(),'testdata')
        env2=Environment() # should reread the OPENRAVE_DATA
        try:
            RaveSetDataAccess(0)
            assert(env2.Load('bobcat.robot.xml'))
            assert(env2.Load('../ikfastrobots/fail1.robot.xml'))
            env2.Reset()
            assert(env2.Load('../ikfastrobots/fail1.dae'))
            RaveSetDataAccess(1)
            env2.Reset()
            assert(env2.Load('bobcat.robot.xml'))
            assert(not env2.Load('../ikfastrobots/fail1.robot.xml'))
            env2.Reset()
            assert(not env2.Load('../ikfastrobots/fail1.dae'))
        finally:
            RaveSetDataAccess(0)
            env2.Destroy()
            
    def test_load_cwd(self):
        env=self.env
        oldcwd = os.getcwd()
        try:
            assert(env.Load('testdata/box0.dae'))
            assert(not env.Load('box0.dae'))
            os.chdir('testdata')
            assert(env.Load('box0.dae'))
        finally:
            os.chdir(oldcwd)
    
    def test_collada_savingoptions(self):
        self.log.info('test collada saving options')
        env=self.env
        self.LoadEnv('data/pr2test1.env.xml')
        env.Save('test_collada_savingoptions.dae',Environment.SelectionOptions.Body,[('target','pr2'),('target','mug1')])

        env2=Environment()
        assert(env2.Load('test_collada_savingoptions.dae'))
        assert(len(env2.GetBodies())==2)
        misc.CompareBodies(env.GetKinBody('pr2'),env2.GetKinBody('pr2'))
        misc.CompareBodies(env.GetKinBody('mug1'),env2.GetKinBody('mug1'))

        env.Save('test_collada_savingoptions.dae',Environment.SelectionOptions.Body,{'target':'mug1'})
        xmldata = '''
  <COLLADA xmlns="http://www.collada.org/2008/03/COLLADASchema" version="1.5.0">
    <asset>
      <created/>
      <modified/>
    </asset>
    <scene>
      <instance_physics_scene url="./test_collada_savingoptions.dae#pscene" sid="pscene_inst"/>
      <instance_visual_scene url="./test_collada_savingoptions.dae#vscene" sid="vscene_inst"/>
      <instance_kinematics_scene url="./test_collada_savingoptions.dae#kscene" sid="kscene_inst">
        <bind_kinematics_model node="visual1/node0">
          <param>kscene_kmodel1_inst</param>
        </bind_kinematics_model>
        <bind_joint_axis target="visual1/node_joint0_axis0">
          <axis><param>kscene_kmodel1_inst_robot1_kinematics_kmodel1_inst_joint0.axis0</param></axis>
          <value><float>1.5</float></value>
        </bind_joint_axis>
      </instance_kinematics_scene>
    </scene>
  </COLLADA>
'''
        env2.Reset()
        assert(env2.LoadData(xmldata))
