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
