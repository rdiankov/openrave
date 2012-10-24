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
        robot=env.GetRobots()[0]
        testdict = {robot.GetManipulators()[0]:1}
        assert(testdict[robot.GetManipulators()[0]] == 1)
        
    def test_uri(self):
        env=self.env
        xml="""<environment>
  <kinbody file="data/mug1.dae"/>
</environment>"""
        self.LoadDataEnv(xml)
        assert(env.GetBodies()[0].GetURI().find('data/mug1.dae') >= 0)

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
        OPENRAVE_DATA = os.environ.get('OPENRAVE_DATA','')
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
            # have to restore everything
            RaveSetDataAccess(0)
            os.environ['OPENRAVE_DATA'] = OPENRAVE_DATA
            env2.Destroy()
            RaveDestroy()
            
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
    
