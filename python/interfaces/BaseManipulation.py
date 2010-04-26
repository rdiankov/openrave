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
from copy import copy as shallowcopy
class BaseManipulation:
    def __init__(self,robot,plannername=None,maxvelmult=None):
        env = robot.GetEnv()
        self.prob = env.CreateProblem('BaseManipulation')
        self.robot = robot
        self.args = self.robot.GetName()
        if plannername is not None:
            self.args += ' planner ' + plannername
        if maxvelmult is not None:
            self.args += ' maxvelmult %f '%maxvelmult
        if env.LoadProblem(self.prob,self.args) != 0:
            raise ValueError('problem failed to initialize')
    def  __del__(self):
        self.prob.GetEnv().RemoveProblem(self.prob)
    def clone(self,envother):
        clone = shallowcopy(self)
        clone.prob = envother.CreateProblem('BaseManipulation')
        clone.robot = envother.GetRobot(self.robot.GetName())
        if envother.LoadProblem(clone.prob,clone.args) != 0:
            raise ValueError('problem failed to initialize')
        return clone
    def TrajFromData(self,data):
        return self.prob.SendCommand('traj stream ' + data)
    def MoveHandStraight(self,direction,minsteps=None,maxsteps=None,stepsize=None,ignorefirstcollision=None,jacobian=None,searchall=None,execute=None,outputtraj=None):
        cmd = 'MoveHandStraight direction %f %f %f '%(direction[0],direction[1],direction[2])
        if minsteps is not None:
            cmd += 'minsteps %d '%minsteps
        if maxsteps is not None:
            cmd += 'maxsteps %d '%maxsteps
        if stepsize is not None:
            cmd += 'stepsize %f '%stepsize
        if execute is not None:
            cmd += 'execute %d '%execute
        if jacobian is not None:
            cmd += 'jacobian %f '%jacobian
        if searchall is not None:
            cmd += 'searchall %d '%searchall
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if ignorefirstcollision is not None:
            cmd += 'ignorefirstcollision %d '%ignorefirstcollision
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('MoveHandStraight')
        return res
    def MoveManipulator(self,goal,maxiter=None,execute=None,outputtraj=None):
        assert(len(goal) == len(self.robot.GetActiveManipulator().GetArmJoints()) and len(goal) > 0)
        cmd = 'MoveManipulator goal ' + ' '.join(str(f) for f in goal) + ' '
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('MoveManipulator')
        return res
    def MoveActiveJoints(self,goal,steplength=None,maxiter=None,maxtries=None,execute=None,outputtraj=None):
        assert(len(goal) == self.robot.GetActiveDOF() and len(goal) > 0)
        cmd = 'MoveActiveJoints goal ' + ' '.join(str(f) for f in goal)+' '
        if steplength is not None:
            cmd += 'steplength %f '%steplength
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if maxtries is not None:
            cmd += 'maxtries %d '%maxtries
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('MoveActiveJoints')
        return res
    def MoveToHandPosition(self,matrices,affinedofs=None,maxiter=None,maxtries=None,seedik=None,constraintfreedoms=None,constraintmatrix=None,constrainterrorthresh=None,execute=None,outputtraj=None):
        cmd = 'MoveToHandPosition matrices %d '%len(matrices)
        for m in matrices:
            cmd += matrixSerialization(m) + ' '
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if maxtries is not None:
            cmd += 'maxtries %d '%maxtries
        if seedik is not None:
            cmd += 'seedik %d '%seedik
        if constraintfreedoms is not None:
            cmd += 'constraintfreedoms %s '%(' '.join(str(constraintfreedoms[i]) for i in range(6)))
        if constraintmatrix is not None:
            cmd += 'constraintmatrix %s '%matrixSerialization(constraintmatrix)
        if constrainterrorthresh is not None:
            cmd += 'constrainterrorthresh %s '%constrainterrorthresh
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('MoveToHandPosition')
        return res
    def MoveUnsyncJoints(self,jointvalues,jointinds,maxtries=None,planner=None,execute=None,outputtraj=None):
        assert(len(jointinds)==len(jointvalues) and len(jointinds)>0)
        cmd = 'MoveUnsyncJoints handjoints %d %s %s '%(len(jointinds),' '.join(str(f) for f in jointvalues), ' '.join(str(f) for f in jointinds))
        if planner is not None:
            cmd += 'planner %s '%planner
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if maxtries is not None:
            cmd += 'maxtries %d '%maxtries
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('MoveUnsyncJoints')
        return res
    def CloseFingers(self,offset=None,movingdir=None,execute=None,outputtraj=None,outputfinal=None):
        cmd = 'CloseFingers '
        dof=len(self.robot.GetActiveManipulator().GetGripperJoints())
        if offset is not None:
            assert(len(offset) == dof)
            cmd += 'offset ' + ' '.join(str(f) for f in offset) + ' '
        if movingdir is not None:
            assert(len(movingdir) == dof)
            cmd += 'movingdir %s '%(' '.join(str(f) for f in movingdir))
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if outputfinal:
            cmd += 'outputfinal'
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('CloseFingers')
        resvalues = res.split()
        if outputfinal:
            final = array([float(resvalues[i]) for i in range(dof)])
            resvalues=resvalues[dof:]
        else:
            final=None
        if outputtraj is not None and outputtraj:
            traj = ' '.join(resvalues)
        else:
            traj = None
        return final,traj
    def ReleaseFingers(self,target=None,movingdir=None,execute=None,outputtraj=None,outputfinal=None):
        cmd = 'ReleaseFingers '
        dof=len(self.robot.GetActiveManipulator().GetGripperJoints())
        if target is not None:
            cmd += 'target %s '%target.GetName()
        if movingdir is not None:
            assert(len(movingdir) == dof)
            cmd += 'movingdir %s '%(' '.join(str(f) for f in movingdir))
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if outputfinal:
            cmd += 'outputfinal'
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('ReleaseFingers')
        resvalues = res.split()
        if outputfinal:
            final = array([float(resvalues[i]) for i in range(dof)])
            resvalues=resvalues[len(final):]
        else:
            final=None
        if outputtraj is not None and outputtraj:
            traj = ' '.join(resvalues)
        else:
            traj = None
        return final,traj
    def ReleaseActive(self,movingdir=None,execute=None,outputtraj=None,outputfinal=None):
        cmd = 'ReleaseActive '
        if movingdir is not None:
            assert(len(movingdir) == self.robot.GetActiveDOF())
            cmd += 'movingdir %s '%(' '.join(str(f) for f in movingdir))
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if outputfinal:
            cmd += 'outputfinal'
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('ReleaseActive')
        resvalues = res.split()
        if outputfinal:
            final = array([float(resvalues[i]) for i in range(self.robot.GetActiveDOF())])
            resvalues=resvalues[len(final):]
        else:
            final=None
        if outputtraj is not None and outputtraj:
            traj = ' '.join(resvalues)
        else:
            traj = None
        return final,traj
    def JitterActive(self,maxiter=None,jitter=None,execute=None,outputtraj=None,outputfinal=None):
        cmd = 'JitterActive '
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if jitter is not None:
            cmd += 'jitter %f '%jitter
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if outputfinal:
            cmd += 'outputfinal'
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('JitterActive')
        resvalues = res.split()
        if outputfinal:
            final = array([float(resvalues[i]) for i in range(self.robot.GetActiveDOF())])
            resvalues=resvalues[len(final):]
        else:
            final=None
        if outputtraj is not None and outputtraj:
            traj = ' '.join(resvalues)
        else:
            traj = None
        return final,traj
    def DebugIK(self,numiters,rotonly=False):
        cmd = 'DebugIK numtests %d '%numiters
        if rotonly:
            cmd += 'rotonly '
        return float(self.prob.SendCommand(cmd)) # success rate
