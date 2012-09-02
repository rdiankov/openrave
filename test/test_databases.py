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

class TestDatabases(EnvironmentSetup):
    def test_ikmodulegeneration(self):
        env=self.env
        for robotname,manipname,iktype in [('robots/neuronics-katana.zae','arm',IkParameterizationType.TranslationDirection5D),('robots/pr2-beta-static.zae','rightarm',IkParameterizationType.Transform6D)]:
            env.Reset()
            self.LoadEnv(robotname)
            robot=env.GetRobots()[0]
            manip=robot.SetActiveManipulator(manipname)
            manip.SetIkSolver(None)

            env.Save('test_imodulegeneration.dae',Environment.SelectionOptions.Body, {'target':robot.GetName(), 'skipwrite':'visual readable sensors physics'})
            env2=Environment()
            robot2=env2.ReadRobotURI('test_imodulegeneration.dae')
            env2.Add(robot2)
            # check that the hashes match
            for m in robot.GetManipulators():
                m2 = robot2.GetManipulator(m.GetName())
                assert(m.GetKinematicsStructureHash()==m2.GetKinematicsStructureHash())

            ikmodule = RaveCreateModule(env,'ikfast')
            env.Add(ikmodule)
            out=ikmodule.SendCommand('LoadIKFastSolver %s %d 1'%(robot.GetName(),iktype))
            assert(out is not None)
            assert(manip.GetIkSolver() is not None)
            
#     def test_database_paths(self):
#         pass
