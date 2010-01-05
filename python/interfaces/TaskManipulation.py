# Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
from openravepy import *
from numpy import *

class TaskManipulation:
    def __init__(self,env,robot,plannername=None):
        self.prob = env.CreateProblem('TaskManipulation')
        self.robot = robot
        args = self.robot.GetName()
        if plannername is not None:
            args += ' planner ' + plannername
        if env.LoadProblem(self.prob,args) != 0:
            raise ValueError('problem failed to initialize')

    def TestAllGrasps(self,grasps,maxiter=None,randomgrasps=None,randomdests=None, execute=None,outputtraj=None):
        pass
