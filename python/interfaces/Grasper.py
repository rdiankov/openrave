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

class Grasper:
    def __init__(self,robot,friction=0.3,avoidlinks=None,plannername=None):
        env = robot.GetEnv()
        self.prob = env.CreateProblem('Grasper')
        self.robot = robot
        self.friction = friction
        self.avoidlinks = avoidlinks
        self.plannername=plannername
        args = self.robot.GetName()
        if plannername is not None:
            args += ' plannername %s '%plannername
        if env.LoadProblem(self.prob,args) != 0:
            raise ValueError('problem failed to initialize')
    def  __del__(self):
        self.prob.GetEnv().RemoveProblem(self.prob)

    def Grasp(self,direction=None,roll=None,position=None,standoff=None,target=None,stablecontacts=False,forceclosure=False,transformrobot=True,onlycontacttarget=True,tightgrasp=False,graspingnoise=None,execute=None,outputfinal=False):
        cmd = 'Grasp '
        if direction is not None:
            cmd += 'direction %f %f %f '%(direction[0],direction[1],direction[2])
        if transformrobot:
            cmd += 'roll %f position %f %f %f standoff %f '%(roll,position[0],position[1],position[2],standoff)
        if target is not None:
            cmd += 'target %s '%target.GetName()
        cmd += 'stablecontacts %d forceclosure %d transformrobot %d onlycontacttarget %d tightgrasp %d outputfinal %d '%(stablecontacts,forceclosure,transformrobot,onlycontacttarget,tightgrasp,outputfinal)
        if self.friction is not None:
            cmd += 'friction %f '%self.friction
        if self.avoidlinks is not None:
            for link in self.avoidlinks:
                cmd += 'avoidlink %s '%link.GetName()
        if graspingnoise is not None:
            cmd += '%f %f '%(graspingnoise[0],graspingnoise[1])
        if execute is not None:
            cmd += 'execute %d '%execute
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('Grasp failed')
        resvalues = res.split()
        mindist = None
        volume = None
        contacts = None
        finalconfig = None
        if forceclosure:
            volume = float(resvalues.pop())
            mindist = float(resvalues.pop())
        if outputfinal:
            jointvalues = []
            for i in range(self.robot.GetDOF()):
                jointvalues.insert(0,float(resvalues.pop()))
            pose = []
            for i in range(7):
                pose.insert(0,float(resvalues.pop()))
            finalconfig = (jointvalues,matrixFromPose(pose))
        contacts = reshape(array([float(s) for s in resvalues],float),(len(resvalues)/6,6))
        return contacts,finalconfig,mindist,volume
