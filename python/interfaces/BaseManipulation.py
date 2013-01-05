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
# python 2.5 raises 'import *' not allowed with 'from .'
from ..openravepy_int import RaveCreateModule, RaveCreateTrajectory, matrixSerialization, IkParameterization
from ..openravepy_ext import planning_error
    
import numpy
from copy import copy as shallowcopy

class BaseManipulation:
    """Interface wrapper for :ref:`module-basemanipulation`
    """
    def __init__(self,robot,plannername=None,maxvelmult=None):
        env = robot.GetEnv()
        self.prob = RaveCreateModule(env,'BaseManipulation')
        self.robot = robot
        self.args = self.robot.GetName()
        if plannername is not None:
            self.args += u' planner ' + plannername
        if maxvelmult is not None:
            self.args += u' maxvelmult %.15e '%maxvelmult
        env.Add(self.prob,True,self.args)
    def  __del__(self):
        self.prob.GetEnv().Remove(self.prob)

    def clone(self,envother):
        return self.Clone(envother)
    
    def Clone(self,envother):
        """Clones the interface into another environment
        """
        clone = shallowcopy(self)
        clone.prob = RaveCreateModule(envother,'BaseManipulation')
        clone.robot = envother.GetRobot(self.robot.GetName())
        envother.Add(clone.prob,True,clone.args)
        return clone

    def SetRobot(self,robot):
        """See :ref:`module-basemanipulation-setrobot`
        """
        success = self.prob.SendCommand(u'setrobot '+robot.GetName())
        if success is not None:
            self.robot = robot
            return True
        
        return False
    
    def TrajFromData(self,data,resettrans=False,resettiming=False):
        """See :ref:`module-basemanipulation-traj`
        """
        return self.prob.SendCommand('traj stream ' + data + ' %d %d '%(resettrans,resettiming))
    def VerifyTrajectory(self,data,resettrans=False,resettiming=False,samplingstep=None):
        """See :ref:`module-basemanipulation-verifytrajectory`
        """
        cmd = 'VerifyTrajectory stream ' + data + ' resettrans %d resettiming %d '%(resettrans,resettiming)
        if samplingstep is not None:
            cmd += 'samplingstep %.15e '%samplingstep
        print cmd
        return self.prob.SendCommand(cmd)

    def MoveHandStraight(self,direction,minsteps=None,maxsteps=None,stepsize=None,ignorefirstcollision=None,starteematrix=None,greedysearch=None,execute=None,outputtraj=None,maxdeviationangle=None,steplength=None,planner=None,outputtrajobj=None):
        """See :ref:`module-basemanipulation-movehandstraight`
        """
        cmd = 'MoveHandStraight direction %.15e %.15e %.15e '%(direction[0],direction[1],direction[2])
        if minsteps is not None:
            cmd += 'minsteps %d '%minsteps
        if maxsteps is not None:
            cmd += 'maxsteps %d '%maxsteps
        if stepsize is not None:
            cmd += 'steplength %.15e '%stepsize
        if steplength is not None:
            cmd += 'steplength %.15e '%steplength
        if planner is not None:
            cmd += 'planner %s '%planner
        if execute is not None:
            cmd += 'execute %d '%execute
        if starteematrix is not None:
            cmd += 'starteematrix ' + matrixSerialization(starteematrix) + ' '
        if greedysearch is not None:
            cmd += 'greedysearch %d '%greedysearch
        if (outputtraj is not None and outputtraj) or (outputtrajobj is not None and outputtrajobj):
            cmd += 'outputtraj '
        if ignorefirstcollision is not None:
            cmd += 'ignorefirstcollision %.15e '%ignorefirstcollision
        if maxdeviationangle is not None:
            cmd += 'maxdeviationangle %.15e '%maxdeviationangle
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('MoveHandStraight')
        if outputtrajobj is not None and outputtrajobj:
            return RaveCreateTrajectory(self.prob.GetEnv(),'').deserialize(res)
        return res
    def MoveManipulator(self,goal=None,maxiter=None,execute=None,outputtraj=None,maxtries=None,goals=None,steplength=None,outputtrajobj=None,jitter=None,releasegil=False):
        """See :ref:`module-basemanipulation-movemanipulator`
        """
        if goal is not None:
            assert(len(goal) == len(self.robot.GetActiveManipulator().GetArmIndices()))
        return self._MoveJoints('MoveManipulator',goal=goal,steplength=steplength,maxiter=maxiter,maxtries=maxtries,execute=execute,outputtraj=outputtraj,goals=goals,outputtrajobj=outputtrajobj,jitter=jitter,releasegil=releasegil)
    
    def MoveActiveJoints(self,goal=None,steplength=None,maxiter=None,maxtries=None,execute=None,outputtraj=None,goals=None,outputtrajobj=None,jitter=None,releasegil=False,postprocessingplanner=None,postprocessingparameters=None):
        """See :ref:`module-basemanipulation-moveactivejoints`
        """
        if goal is not None:
            assert(len(goal) == self.robot.GetActiveDOF() and len(goal) > 0)
        return self._MoveJoints('MoveActiveJoints',goal=goal,steplength=steplength,maxiter=maxiter,maxtries=maxtries,execute=execute,outputtraj=outputtraj,goals=goals,outputtrajobj=outputtrajobj,jitter=jitter,releasegil=releasegil,postprocessingplanner=postprocessingplanner,postprocessingparameters=postprocessingparameters)

    def _MoveJoints(self,cmd,goal=None,steplength=None,maxiter=None,maxtries=None,execute=None,outputtraj=None,goals=None,outputtrajobj=None,jitter=None,releasegil=False,postprocessingplanner=None,postprocessingparameters=None):
        """See :ref:`module-basemanipulation-moveactivejoints`
        """
        cmd += ' '
        if goal is not None:
            cmd += 'goal ' + ' '.join('%.15e'%f for f in goal) + ' '
        if goals is not None:
            cmd += 'goals %d '%len(goals)
            for g in goals:
                for f in g:
                    cmd += '%.15e '%f
        if steplength is not None:
            cmd += 'steplength %.15e '%steplength
        if execute is not None:
            cmd += 'execute %d '%execute
        if (outputtraj is not None and outputtraj) or (outputtrajobj is not None and outputtrajobj):
            cmd += 'outputtraj '
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if maxtries is not None:
            cmd += 'maxtries %d '%maxtries
        if jitter is not None:
            cmd += 'jitter %f '%jitter
        if postprocessingplanner is not None:
            cmd += 'postprocessingplanner %s\n'%postprocessingplanner
        if postprocessingparameters is not None:
            cmd += 'postprocessingparameters %s\n'%postprocessingparameters
        res = self.prob.SendCommand(cmd,releasegil=releasegil)
        if res is None:
            raise planning_error('MoveActiveJoints')
        if outputtrajobj is not None and outputtrajobj:
            return RaveCreateTrajectory(self.prob.GetEnv(),'').deserialize(res)
        return res

    def MoveToHandPosition(self,matrices=None,affinedofs=None,maxiter=None,maxtries=None,translation=None,rotation=None,seedik=None,constraintfreedoms=None,constraintmatrix=None,constrainterrorthresh=None,execute=None,outputtraj=None,steplength=None,goalsamples=None,ikparam=None,ikparams=None,jitter=None,minimumgoalpaths=None,outputtrajobj=None,postprocessing=None,jittergoal=None, constrainttaskmatrix=None, constrainttaskpose=None,goalsampleprob=None,goalmaxsamples=None,goalmaxtries=None):
        """See :ref:`module-basemanipulation-movetohandposition`

        postprocessing is two parameters: (plannername,parmaeters)
        """
        cmd = 'MoveToHandPosition '
        if matrices is not None:
            cmd += 'matrices %d '%len(matrices)
            for m in matrices:
                cmd += matrixSerialization(m) + ' '
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if maxtries is not None:
            cmd += 'maxtries %d '%maxtries
        if translation is not None:
            cmd += 'translation %.15e %.15e %.15e '%(translation[0],translation[1],translation[2])
        if rotation is not None:
            cmd += 'rotation %.15e %.15e %.15e %.15e '%(rotation[0],rotation[1],rotation[2],rotation[3])
        if seedik is not None:
            cmd += 'seedik %d '%seedik
        if goalsamples is not None:
            cmd += 'goalsamples %d '%goalsamples
        if postprocessing is not None:
            cmd += 'postprocessingplanner %s\n postprocessingparameters %s\n '%(postprocessing[0],postprocessing[1])
        if constraintfreedoms is not None:
            cmd += 'constraintfreedoms %s '%(' '.join(str(constraintfreedoms[i]) for i in range(6)))
        if constraintmatrix is not None:
            cmd += 'constraintmatrix %s '%matrixSerialization(constraintmatrix)
        if constrainttaskmatrix is not None:
            cmd += 'constrainttaskmatrix %s '%matrixSerialization(constrainttaskmatrix)
        if constrainterrorthresh is not None:
            cmd += 'constrainterrorthresh %s '%constrainterrorthresh
        if jitter is not None:
            cmd += 'jitter %.15e '%jitter
        if steplength is not None:
            cmd += 'steplength %.15e '%steplength
        if jittergoal is not None:
            cmd += 'jittergoal %.15e '%jittergoal
        if ikparam is not None:
            cmd += 'ikparam ' + str(ikparam) + ' '
        if ikparams is not None:
            cmd += 'ikparams %d '%len(ikparams)
            for ikp in ikparams:
                cmd += str(ikp) + ' '
        if execute is not None:
            cmd += 'execute %d '%execute
        if (outputtraj is not None and outputtraj) or (outputtrajobj is not None and outputtrajobj):
            cmd += 'outputtraj '
        if minimumgoalpaths is not None:
            cmd += 'minimumgoalpaths %d '%minimumgoalpaths
        if goalsampleprob is not None:
            cmd += 'goalsampleprob %.15e '%goalsampleprob
        if goalmaxtries is not None:
            cmd += 'goalmaxtries %d '%goalmaxtries
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('MoveToHandPosition')
        if outputtrajobj is not None and outputtrajobj:
            return RaveCreateTrajectory(self.prob.GetEnv(),'').deserialize(res)
        return res
    def MoveUnsyncJoints(self,jointvalues,jointinds,maxtries=None,planner=None,maxdivision=None,execute=None,outputtraj=None,outputtrajobj=None):
        """See :ref:`module-basemanipulation-moveunsyncjoints`
        """
        assert(len(jointinds)==len(jointvalues) and len(jointinds)>0)
        cmd = 'MoveUnsyncJoints handjoints %d %s %s '%(len(jointinds),' '.join('%.15e'%f for f in jointvalues), ' '.join(str(f) for f in jointinds))
        if planner is not None:
            cmd += 'planner %s '%planner
        if execute is not None:
            cmd += 'execute %d '%execute
        if (outputtraj is not None and outputtraj) or (outputtrajobj is not None and outputtrajobj):
            cmd += 'outputtraj '
        if maxtries is not None:
            cmd += 'maxtries %d '%maxtries
        if maxdivision is not None:
            cmd += 'maxdivision %d '%maxdivision
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('MoveUnsyncJoints')
        if outputtrajobj is not None and outputtrajobj:
            return RaveCreateTrajectory(self.prob.GetEnv(),'').deserialize(res)
        return res
    def JitterActive(self,maxiter=None,jitter=None,execute=None,outputtraj=None,outputfinal=None,outputtrajobj=None):
        """See :ref:`module-basemanipulation-jitteractive`
        """
        cmd = 'JitterActive '
        if maxiter is not None:
            cmd += 'maxiter %d '%maxiter
        if jitter is not None:
            cmd += 'jitter %.15e '%jitter
        if execute is not None:
            cmd += 'execute %d '%execute
        if (outputtraj is not None and outputtraj) or (outputtrajobj is not None and outputtrajobj):
            cmd += 'outputtraj '
        if outputfinal:
            cmd += 'outputfinal'
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('JitterActive')
        resvalues = res.split()
        if outputfinal:
            final = numpy.array([numpy.float64(resvalues[i]) for i in range(self.robot.GetActiveDOF())])
            resvalues=resvalues[len(final):]
        else:
            final=None
        if (outputtraj is not None and outputtraj) or (outputtrajobj is not None and outputtrajobj):
            traj = ' '.join(resvalues)
        else:
            traj = None
        if traj is not None and outputtrajobj is not None and outputtrajobj:
            traj = RaveCreateTrajectory(self.prob.GetEnv(),'').deserialize(traj)
        return final,traj
    def FindIKWithFilters(self,ikparam,cone=None,solveall=None,filteroptions=None):
        """See :ref:`module-basemanipulation-findikwithfilters`
        """
        cmd = 'FindIKWithFilters ikparam %s '%str(ikparam)
        if cone is not None:
            cmd += 'cone %s '%(' '.join('%.15e'%f for f in cone))
        if solveall is not None and solveall:
            cmd += 'solveall '
        if filteroptions is not None:
            cmd += 'filteroptions %d '%filteroptions
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('FindIKWithFilters')
        resvalues = res.split()
        num = int(resvalues[0])
        dim = (len(resvalues)-1)/num
        solutions = numpy.reshape([numpy.float64(s) for s in resvalues[1:]],(num,dim))
        return solutions
