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
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'
from openravepy import *
from numpy import *

class BaseManipulation:
    def __init__(self,env,robot,plannername=None):
        self.prob = env.CreateProblem('BaseManipulation')
        self.robot = robot
        args = self.robot.GetName()
        if plannername is not None:
            args += ' planner ' + plannername
        if env.LoadProblem(self.prob,args) != 0:
            raise ValueError('problem failed to initialize')
    def  __del__(self):
        self.prob.GetEnv().RemoveProblem(self.prob)
    def TrajFromFile(self,filename):
        return self.prob.SendCommand('traj sep ; %s;'%filename)
    def TrajFromData(self,data):
        return self.prob.SendCommand('traj stream ' + data)
    def MoveHandStraight(self,direction,minsteps=None,maxsteps=None,stepsize=None,ignorefirstcollision=None,execute=None,outputtraj=None):
        cmd = 'MoveHandStraight direction %f %f %f '%(direction[0],direction[1],direction[2])
        if minsteps is not None:
            cmd += 'minsteps %d '%minsteps
        if maxsteps is not None:
            cmd += 'maxsteps %d '%maxsteps
        if stepsize is not None:
            cmd += 'stepsize %f '%stepsize
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None:
            cmd += 'outputtraj %d '%outputtraj
        if ignorefirstcollision is not None:
            cmd += 'ignorefirstcollision %d '%ignorefirstcollision
        return self.prob.SendCommand(cmd)
    def MoveManipulator(self,goal,maxiter=None,execute=None,outputtraj=None):
        assert(len(goal) == len(self.robot.GetActiveManipulator().GetArmJoints()) and len(goal) > 0)
        cmd = 'MoveManipulator goal ' + ' '.join(str(f) for f in goal)
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None:
            cmd += 'outputtraj %d '%outputtraj
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        return self.prob.SendCommand(cmd)
    def MoveActiveJoints(self,goal,maxiter=None,execute=None,outputtraj=None):
        assert(len(goal) == len(self.robot.GetActiveDOF()) and len(goal) > 0)
        cmd = 'MoveActiveJoints goal ' + ' '.join(str(f) for f in goal)
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None:
            cmd += 'outputtraj %d '%outputtraj
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        return self.prob.SendCommand(cmd)
    def MoveToHandPosition(self,matrices,affinedofs=None,maxiter=None,maxtries=None,seedik=None,execute=None,outputtraj=None):
        cmd = 'MoveToHandPosition matrices %d '%len(matrices)
        for m in matrices:
            cmd += matrixSerialization(m) + ' '
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if maxtries is not None:
            cmd += 'maxtries %d '%maxtries
        if seedik is not None:
            cmd += 'seedik %d '%seedik
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None:
            cmd += 'outputtraj %d '%outputtraj
        return self.prob.SendCommand(cmd)
    def MoveUnsyncJoints(self,jointvalues,jointinds,planner=None,execute=None,outputtraj=None):
        assert(len(jointinds)==len(jointvalues) and len(jointinds)>0)
        cmd = 'MoveUnsyncJoints handjoints %d %s %s '%(len(jointinds),' '.join(str(f) for f in jointvalues), ' '.join(str(f) for f in jointinds))
        if planner is not None:
            cmd += 'planner %s '%planner
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None:
            cmd += 'outputtraj %d '%outputtraj
        return self.prob.SendCommand(cmd)
    def CloseFingers(self,offset=None,execute=None,outputtraj=None):
        cmd = 'CloseFingers '
        if offset is not None:
            assert(len(offset) == len(self.robot.GetActiveManipulator().GetGripperJoints()))
            cmd += ' '.join(str(f) for f in offset) + ' '
        return self.prob.SendCommand(cmd)
    def ReleaseFingers(self,target=None,movingdir=None,execute=None,outputtraj=None):
        cmd = 'ReleaseFingers '
        if target is not None:
            cmd += 'target %s '%target.GetName()
        if movingdir is not None:
            assert(len(movingdir) == self.robot.GetActiveDOF())
            cmd += 'movingdir %s '%(' '.join(str(f) for f in movingdir))
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None:
            cmd += 'outputtraj %d '%outputtraj
        return self.prob.SendCommand(cmd)
