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
