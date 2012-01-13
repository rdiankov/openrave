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

class TestEnvironment(EnvironmentSetup):
    def test_load(self):
        env=self.env
        env.Load('../src/models/WAM/wam0.iv')
        
        for fullfilename in locate('*.xml','../src/data'):
            print 'loading: ',fullfilename
            env.Reset()
            assert(env.Load(fullfilename))
            
    def test_loadnogeom(self):
        env=self.env
        assert(env.Load('robots/pr2-beta-static.zae',{'skipgeometry':'1'}))
        assert(len(env.GetBodies())==1)
        robot=env.GetRobots()[0]
        trimesh=env.Triangulate(robot)
        assert(len(trimesh.vertices)==0)
        
    def test_misc(self):
        assert(self.env.plot3([0,0,0],10)==None) # no viewer attached

    def test_uri(self):
        env=self.env
        xml="""<environment>
  <kinbody file="data/jsk-plate.zae"/>
</environment>"""
        env.LoadData(xml)
        assert(env.GetBodies()[0].GetURI().find('data/jsk-plate.zae') >= 0)

    def test_scalegeometry(self):
        env=self.env
        with env:
            scalefactor = array([2.0,3.0,4.0])
            body1=env.ReadKinBodyURI('data/mug1.kinbody.xml')
            env.AddKinBody(body1,True)
            renderfilename = body1.GetLinks()[0].GetGeometries()[0].GetRenderFilename()
            body1.SetTransform(eye(4))
            body2=env.ReadKinBodyURI('data/mug1.kinbody.xml',{'scalegeometry':'%f %f %f'%tuple(scalefactor)})
            env.AddKinBody(body2,True)
            body2.SetTransform(eye(4))
            ab1=body1.ComputeAABB()
            ab2=body2.ComputeAABB()
            assert( transdist(ab1.pos()*scalefactor,ab2.pos()) <= g_epsilon )
            assert( transdist(ab1.extents()*scalefactor,ab2.extents()) <= g_epsilon )

            if len(body1.GetLinks()) == 1:
                body3 = env.ReadKinBodyURI(renderfilename,{'scalegeometry':'%f %f %f'%tuple(scalefactor)})
                env.AddKinBody(body3,True)
                body3.SetTransform(eye(4))
                ab3=body2.ComputeAABB()
                assert( transdist(ab3.pos(),ab2.pos()) <= g_epsilon )
                assert( transdist(ab3.extents(),ab2.extents()) <= g_epsilon )

    def test_collada(self):
        print "test that collada import/export works"
        epsilon = 400*g_epsilon # because exporting, expect to lose precision, should fix this
        env=self.env
        with env:
            for robotfile in g_robotfiles:
                env.Reset()
                robot0=env.ReadRobotURI(robotfile)
                env.AddRobot(robot0,True)
                robot0.SetTransform(eye(4))
                env.Save('test.zae')
                robot1=env.ReadRobotURI('test.zae')
                env.AddRobot(robot1,True)
                robot1.SetTransform(eye(4))
                assert(len(robot0.GetJoints())==len(robot1.GetJoints()))
                assert(len(robot0.GetPassiveJoints()) == len(robot1.GetPassiveJoints()))
                assert(robot0.GetDOF()==robot1.GetDOF())
                robot1.SetDOFValues(robot0.GetDOFValues()) # in case
                joints0 = robot0.GetJoints()+robot0.GetPassiveJoints()
                joints1 = robot1.GetJoints()+robot1.GetPassiveJoints()
                for j0 in joints0:
                    assert( len(j0.GetName()) > 0 )
                    if j0.GetJointIndex() >= 0:
                        # if not passive, indices should match
                        j1 = joints1[j0.GetJointIndex()]
                        assert(j1.GetJointIndex()==j0.GetJointIndex() and j1.GetDOFIndex() == j0.GetDOFIndex())
                    else:
                        j1s = [j1 for j1 in joints1 if j0.GetName() == j1.GetName()]
                        assert( len(j1s) == 1 )
                        j1 = j1s[0]
                    assert( transdist(j0.GetAnchor(),j1.GetAnchor()) <= epsilon )
                    assert( j0.GetDOF() == j1.GetDOF() and j0.GetType() == j1.GetType() )
                    # todo, once physics is complete, uncomment
                    #assert( j0.GetHierarchyParentLink().GetName() == j1.GetHierarchyParentLink().GetName() )
                    #assert( j0.GetHierarchyChildLink().GetName() == j1.GetHierarchyChildLink().GetName() )
                    assert( transdist(j0.GetInternalHierarchyLeftTransform(),j1.GetInternalHierarchyLeftTransform()) <= epsilon )
                    assert( transdist(j0.GetInternalHierarchyRightTransform(),j1.GetInternalHierarchyRightTransform()) <= epsilon )
                    assert( j0.IsStatic() == j1.IsStatic() )
                    assert( transdist(j0.GetLimits(),j1.GetLimits()) <= epsilon )
                    for idof in range(j0.GetDOF()):
                        assert( abs(j0.GetMaxVel(idof)-j1.GetMaxVel(idof)) <= epsilon )
                        assert( abs(j0.GetMaxAccel(idof)-j1.GetMaxAccel(idof)) <= epsilon )
                        assert( j0.IsCircular(idof) == j1.IsCircular(idof) )
                        assert( j0.IsRevolute(idof) == j1.IsRevolute(idof) )
                        assert( j0.IsPrismatic(idof) == j1.IsPrismatic(idof) )
                        assert( transdist(j0.GetInternalHierarchyAxis(idof),j1.GetInternalHierarchyAxis(idof)) <= epsilon )
                        assert( j0.IsMimic(idof) == j1.IsMimic(idof) )
                        if j0.IsMimic(idof):
                            mimicjoints0 = [robot0.GetJointFromDOFIndex(index).GetName() for index in j0.GetMimicDOFIndices(idof)]
                            mimicjoints1 = [robot1.GetJointFromDOFIndex(index).GetName() for index in j1.GetMimicDOFIndices(idof)]
                            assert( mimicjoints0 == mimicjoints1 )
                            # is it possible to compare equations?
                            # assert( j0.GetMimicEquation(idof) == j1.GetMimicEquation(idof) )
                        #todo: GetResolution, GetWeight
                        #todo: ignore links
                assert(len(robot0.GetLinks())==len(robot1.GetLinks()))
                indexmap = []
                for link0 in robot0.GetLinks():
                    link1s = [link1 for link1 in robot1.GetLinks() if link0.GetName() == link1.GetName()]
                    assert( len(link1s) == 1 )
                    link1 = link1s[0]
                    indexmap.append(link1.GetIndex())
                    assert( transdist(link0.GetTransform(),link1.GetTransform()) <= epsilon )
                    #assert( link0.IsStatic() == link1.IsStatic() )
                    assert( len(link0.GetParentLinks()) == len(link1.GetParentLinks()) )
                    assert( all([lp0.GetName()==lp1.GetName() for lp0, lp1 in izip(link0.GetParentLinks(),link1.GetParentLinks())]) )
                    assert( len(link0.GetGeometries()) == len(link1.GetGeometries()) )
                    ab0=link0.ComputeAABB()
                    ab1=link1.ComputeAABB()
                    assert(transdist(ab0.pos(),ab1.pos()) <= epsilon)
                    assert(transdist(ab0.extents(),ab1.extents()) <= epsilon)
                    # todo: compare geometry, collada still does not support writing boxes
#                     for ig in range(len(link0.GetGeometries())):
#                         g0=link0.GetGeometries()[ig]
#                         g1=link1.GetGeometries()[ig]
#                         assert(g0.GetType()==g1.GetType())


                adjacentlinks = set([tuple(sorted((indexmap[index0],indexmap[index1]))) for index0,index1 in robot0.GetAdjacentLinks()])
                assert(adjacentlinks == set(robot1.GetAdjacentLinks()))

            # test if collada can store current joint values
            env.Reset()
            robot0=env.ReadRobotURI(g_robotfiles[0])
            env.AddRobot(robot0,True)
            robot0.SetTransform(eye(4))
            lower,upper = robot0.GetDOFLimits()
            robot0.SetDOFValues(lower+random.rand(robot0.GetDOF())*(upper-lower))
            env.Save('test.dae')
            oldname = robot0.GetName()
            robot0.SetName('__dummy__')
            env.Load('test.dae')
            robot1=env.GetRobot(oldname)
            assert(transdist(robot0.GetDOFValues(),robot1.GetDOFValues()) <= 1e-4 ) # for now have to use this precision until collada-dom can store doubles

    def test_unicode(self):
        env=self.env
        name = 'テスト名前'.decode('utf-8')
        body=RaveCreateKinBody(env,'')
        body.InitFromBoxes(array([[0,0,0,1,1,1]]),True)
        body.SetName(name)
        env.AddKinBody(body)
        assert(body.GetName().decode('utf-8')==name)

        openrave_config = Popen(['openrave-config','--share-dir'],stdout=PIPE)
        share_dir = openrave_config.communicate()[0].strip()
        srcfile = os.path.join(share_dir,'models','WAM','wam0.iv')
        destfile = '新しいファイル.iv'
        shutil.copyfile(srcfile,destfile)
        robotxml = """<?xml version="1.0" encoding="utf-8"?>
<robot name="ロボット名">
  <kinbody>
    <body name="テスト">
      <geom type="trimesh">
        <data>%s</data>
      </geom>
    </body>
  </kinbody>
</robot>
"""%destfile
        urobotxml = robotxml.decode('utf-8')
        robot=env.ReadRobotXMLData(urobotxml)
        env.AddRobot(robot)
        assert(robot.GetName() == 'ロボット名')
        assert(robot.GetLinks()[0].GetName() == 'テスト')
        env.Remove(robot)

        robot=env.ReadRobotXMLData(robotxml)
        env.AddRobot(robot)
        assert(robot.GetName() == 'ロボット名')
        assert(robot.GetLinks()[0].GetName() == 'テスト')
