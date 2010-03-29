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
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'
from openravepy import *
from numpy import *

class TaskManipulation:
    def __init__(self,robot,plannername=None):
        env = robot.GetEnv()
        self.prob = env.CreateProblem('TaskManipulation')
        self.robot = robot
        args = self.robot.GetName()
        if plannername is not None:
            args += ' planner ' + plannername
        if env.LoadProblem(self.prob,args) != 0:
            raise ValueError('problem failed to initialize')
    def  __del__(self):
        self.prob.GetEnv().RemoveProblem(self.prob)

    def GraspPlanning(self,graspindices,grasps,target,approachoffset=0,destposes=None,seedgrasps=None,seeddests=None,seedik=None,switchpatterns=None,maxiter=None,randomgrasps=None,randomdests=None, execute=None,outputtraj=None):
        cmd = 'graspplanning target %s approachoffset %f grasps %d %d '%(target.GetName(),approachoffset, grasps.shape[0],grasps.shape[1])
        for f in grasps.flat:
            cmd += str(f) + ' '
        for name,valuerange in graspindices.iteritems():
            if name[0] == 'i':
                cmd += name + ' ' + str(valuerange[0]) + ' '
        if destposes is not None:
            if len(destposes[0]) == 7: # pose
                cmd += 'posedests %d '%len(destposes)
                for pose in destposes:
                    cmd += poseSerialization(pose) + ' '
            else:
                cmd += 'matdests %d '%len(destposes)
                for mat in destposes:
                    cmd += matrixSerialization(mat) + ' '
        if seedgrasps is not None:
            cmd += 'seedgrasps %d '%seedgrasps
        if seeddests is not None:
            cmd += 'seeddests %d '%seeddests
        if seedik is not None:
            cmd += 'seedik %d '%seedik
        if switchpatterns is not None:
            for pattern in switchpatterns:
                cmd += 'switch %s %s '%(pattern[0],pattern[1])
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if randomgrasps is not None:
            cmd += 'randomgrasps %d '%randomgrasps
        if randomdests is not None:
            cmd += 'randomdests %d '%randomdests
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None:
            cmd += 'outputtraj '
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error()
        resvalues = res.split()
        numgoals = int(resvalues.pop(0))
        goals = []
        for i in range(numgoals):
            T = eye(4)
            for j in range(4):
                for k in range(3):
                    T[k][j] = float(resvalues.pop(0))
            goals.append(T)
        graspindex = int(resvalues.pop(0))
        searchtime = double(resvalues.pop(0))
        trajdata = None
        if outputtraj is not None and outputtraj:
            trajdata = ' '.join(resvalues)
        return goals,graspindex,searchtime,trajdata
    def EvaluateConstraints(self,freedoms,configs,targetframematrix=None,targetframepose=None,errorthresh=None):
        cmd = 'EvaluateConstraints constraintfreedoms %s '%(' '.join(str(f) for f in freedoms))
        if targetframematrix is not None:
            cmd += 'constraintmatrix %s '%matrixSerialization(targetframematrix)
        if targetframepose is not None:
            cmd += 'pose %s '%poseSerialization(targetframepose)
        if errorthresh is not None:
            cmd += 'constrainterrorthresh %f '%errorthresh
        for config in configs:
            cmd += 'config %s '%(' '.join(str(f) for f in config))
        res = self.prob.SendCommand(cmd)
        resvalues = res.split()
        iters = array([int(s) for s in resvalues[0:len(configs)]])
        newconfigs = reshape(array([float(s) for s in resvalues[len(configs):]]),(len(configs),self.robot.GetActiveDOF()))
        return iters,newconfigs
