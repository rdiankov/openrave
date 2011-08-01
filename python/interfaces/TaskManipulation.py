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
__copyright__ = 'Copyright (C) 2009-2011 Rosen Diankov <rosen.diankov@gmail.com>'
__license__ = 'Apache License, Version 2.0'
from ..openravepy_ext import *
from ..openravepy_int import *
from numpy import *
from copy import copy as shallowcopy
class TaskManipulation:
    """Interface wrapper for :ref:`module-taskmanipulation`
    """
    def __init__(self,robot,plannername=None,maxvelmult=None,graspername=None):
        env = robot.GetEnv()
        self.prob = RaveCreateModule(env,'TaskManipulation')
        self.robot = robot
        self.args = self.robot.GetName()
        if plannername is not None and len(plannername) > 0:
            self.args += ' planner ' + plannername
        if maxvelmult is not None:
            self.args += ' maxvelmult %.15e '%maxvelmult
        if graspername is not None and len(graspername)>0:
            self.args += ' graspername %s '%graspername
        if env.AddModule(self.prob,self.args) != 0:
            raise ValueError('module failed to initialize')
    def  __del__(self):
        self.prob.GetEnv().Remove(self.prob)
    def clone(self,envother):
        """Clones the interface into another environment
        """
        clone = shallowcopy(self)
        clone.prob = RaveCreateModule(envother,'TaskManipulation')
        clone.robot = envother.GetRobot(self.robot.GetName())
        if envother.AddModule(clone.prob,clone.args) != 0:
            raise ValueError('module failed to initialize')
        return clone
    def GraspPlanning(self,graspindices,grasps,target,approachoffset=0,destposes=None,seedgrasps=None,seeddests=None,seedik=None,maxiter=None,randomgrasps=None,randomdests=None, execute=None,outputtraj=None):
        """See :ref:`module-taskmanipulation-graspplanning`
        """
        cmd = 'graspplanning target %s approachoffset %.15e grasps %d %d '%(target.GetName(),approachoffset, grasps.shape[0],grasps.shape[1])
        for f in grasps.flat:
            cmd += str(f) + ' '
        for name,valuerange in graspindices.iteritems():
            if name[0] == 'i' and len(valuerange) > 0 or name == 'grasptrans_nocol':
                cmd += name + ' ' + str(valuerange[0]) + ' '
        if destposes is not None and len(destposes) > 0:
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
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if randomgrasps is not None:
            cmd += 'randomgrasps %d '%randomgrasps
        if randomdests is not None:
            cmd += 'randomdests %d '%randomdests
        if execute is not None:
            cmd += 'execute %d '%execute
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error()
        resvalues = res.split()
        numgoals = int(resvalues.pop(0))
        goals = []
        for i in range(numgoals):
            # get the number of values
            numvalues = 1+IkParameterization.GetNumberOfValues(IkParameterization.Type(int(resvalues[0])))
            goals.append(IkParameterization(' '.join(resvalues[0:numvalues])))
            resvalues = resvalues[numvalues:]
        graspindex = int(resvalues.pop(0))
        searchtime = double(resvalues.pop(0))
        trajdata = None
        if outputtraj is not None and outputtraj:
            trajdata = ' '.join(resvalues)
        return goals,graspindex,searchtime,trajdata
    def EvaluateConstraints(self,freedoms,configs,targetframematrix=None,targetframepose=None,errorthresh=None):
        """See :ref:`module-taskmanipulation-evaluateconstraints`
        """
        cmd = 'EvaluateConstraints constraintfreedoms %s '%(' '.join(str(f) for f in freedoms))
        if targetframematrix is not None:
            cmd += 'constraintmatrix %s '%matrixSerialization(targetframematrix)
        if targetframepose is not None:
            cmd += 'pose %s '%poseSerialization(targetframepose)
        if errorthresh is not None:
            cmd += 'constrainterrorthresh %.15e '%errorthresh
        for config in configs:
            cmd += 'config %s '%(' '.join(str(f) for f in config))
        res = self.prob.SendCommand(cmd)
        resvalues = res.split()
        iters = array([int(s) for s in resvalues[0:len(configs)]])
        newconfigs = reshape(array([float64(s) for s in resvalues[len(configs):]]),(len(configs),self.robot.GetActiveDOF()))
        return iters,newconfigs
    def CloseFingers(self,offset=None,movingdir=None,execute=None,outputtraj=None,outputfinal=None,coarsestep=None):
        """See :ref:`module-taskmanipulation-closefingers`
        """
        cmd = 'CloseFingers '
        dof=len(self.robot.GetActiveManipulator().GetGripperIndices())
        if offset is not None:
            assert(len(offset) == dof)
            cmd += 'offset ' + ' '.join(str(f) for f in offset) + ' '
        if movingdir is not None:
            assert(len(movingdir) == dof)
            cmd += 'movingdir %s '%(' '.join(str(f) for f in movingdir))
        if execute is not None:
            cmd += 'execute %d '%execute
        if coarsestep is not None:
            cmd += 'coarsestep %.15e '%coarsestep
        if outputtraj is not None and outputtraj:
            cmd += 'outputtraj '
        if outputfinal:
            cmd += 'outputfinal'
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('CloseFingers')
        resvalues = res.split()
        if outputfinal:
            final = array([float64(resvalues[i]) for i in range(dof)])
            resvalues=resvalues[dof:]
        else:
            final=None
        if outputtraj is not None and outputtraj:
            traj = ' '.join(resvalues)
        else:
            traj = None
        return final,traj
    def ReleaseFingers(self,target=None,movingdir=None,execute=None,outputtraj=None,outputfinal=None,coarsestep=None):
        """See :ref:`module-taskmanipulation-releasefingers`
        """
        cmd = 'ReleaseFingers '
        dof=len(self.robot.GetActiveManipulator().GetGripperIndices())
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
        if coarsestep is not None:
            cmd += 'coarsestep %.15e '%coarsestep
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('ReleaseFingers')
        resvalues = res.split()
        if outputfinal:
            final = array([float64(resvalues[i]) for i in range(dof)])
            resvalues=resvalues[len(final):]
        else:
            final=None
        if outputtraj is not None and outputtraj:
            traj = ' '.join(resvalues)
        else:
            traj = None
        return final,traj
    def ReleaseActive(self,movingdir=None,execute=None,outputtraj=None,outputfinal=None):
        """See :ref:`module-taskmanipulation-releaseactive`
        """
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
            final = array([float64(resvalues[i]) for i in range(self.robot.GetActiveDOF())])
            resvalues=resvalues[len(final):]
        else:
            final=None
        if outputtraj is not None and outputtraj:
            traj = ' '.join(resvalues)
        else:
            traj = None
        return final,traj
    def SwitchModels(self,switchpatterns=None,unregister=None,switchtofat=None,clearpatterns=None,clearmodels=None,update=None):
        """See :ref:`module-taskmanipulation-switchmodels`
        """
        cmd = 'switchmodels '
        if switchpatterns is not None:
            for pattern,filename in switchpatterns:
                cmd += 'register %s %s '%(pattern,filename)
        if unregister is not None:
            for pattern in unregister:
                cmd += 'unregister %s '%pattern
        if switchtofat is not None:
            cmd += 'switchtofat %d '%(switchtofat)
        if clearpatterns is not None and clearpatterns:
            cmd += 'clearpatterns '
        if clearmodels is not None and clearmodels:
            cmd += 'clearmodels '
        if update is not None:
            cmd += 'update %d '%update
        return self.prob.SendCommand(cmd)
    class SwitchState:
        def __init__(self,taskmanip,update=True):
            self.taskmanip = taskmanip
            self.update=update
        def __enter__(self):
            self.taskmanip.SwitchModels(switchtofat=True,update=self.update)
        def __exit__(self, type, value, traceback):
            self.taskmanip.SwitchModels(switchtofat=False)
