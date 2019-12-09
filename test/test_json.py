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
try:
    import ultrajson as json
except ImportError as e:
    import json

from common_test_openrave import *

class TestJSONSeralization(EnvironmentSetup):
    def test_manipInfo(self):
        self.log.info('test serialize/deserialize manipualtor info')
        env = self.env
        with env:
            for robotfile in g_robotfiles:
                env.Reset()
                robot0 = self.LoadRobot(robotfile)
                manips = robot0.GetManipulators()
                for manip in manips:
                    manipInfo = manip.GetInfo().SeiralizeJSON()
                    manip.DeserializeJSON(json.dumps(manipInfo))
