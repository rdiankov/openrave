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

class Grasper:
    """Interface wrapper for :ref:`probleminstance-grasper`
    """
    def __init__(self,robot,friction=0.3,avoidlinks=None,plannername=None):
        env = robot.GetEnv()
        self.prob = RaveCreateProblem(env,'Grasper')
        self.robot = robot
        self.friction = friction
        self.avoidlinks = avoidlinks
        self.plannername=plannername
        self.args = self.robot.GetName()
        if plannername is not None and len(plannername)>0:
            self.args += ' planner %s '%plannername
        if env.LoadProblem(self.prob,self.args) != 0:
            raise ValueError('problem failed to initialize')
    def  __del__(self):
        self.prob.GetEnv().Remove(self.prob)
    def clone(self,envother):
        clone = shallowcopy(self)
        clone.prob = RaveCreateProblem(envother,'Grasper')
        clone.robot = envother.GetRobot(self.robot.GetName())
        clone.avoidlinks = [clone.robot.GetLink(link.GetName()) for link in self.avoidlinks]
        if envother.LoadProblem(clone.prob,clone.args) != 0:
            raise ValueError('problem failed to initialize')
        return clone
    def Grasp(self,direction=None,roll=None,position=None,standoff=None,target=None,stablecontacts=False,forceclosure=False,transformrobot=True,onlycontacttarget=True,tightgrasp=False,graspingnoise=None,execute=None,translationstepmult=None,outputfinal=False):
        """See :ref:`probleminstance-grasper-grasp`
        """
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
            cmd += 'graspingnoise %f '%graspingnoise
        if translationstepmult is not None:
            cmd += 'translationstepmult %f '%translationstepmult
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
            volume = float64(resvalues.pop())
            mindist = float64(resvalues.pop())
        if outputfinal:
            jointvalues = []
            for i in range(self.robot.GetDOF()):
                jointvalues.insert(0,float64(resvalues.pop()))
            pose = []
            for i in range(7):
                pose.insert(0,float64(resvalues.pop()))
            finalconfig = (jointvalues,matrixFromPose(pose))
        contacts = reshape(array([float64(s) for s in resvalues],float64),(len(resvalues)/6,6))
        return contacts,finalconfig,mindist,volume
    def ConvexHull(self,points,returnplanes=True,returnfaces=True,returntriangles=True):
        """See :ref:`probleminstance-grasper-convexhull`
        """
        dim = len(points[0])
        cmd = 'ConvexHull points %d %d '%(len(points),dim) + ' '.join(str(f) for f in points.flat) + ' '
        if returnplanes is not None:
            cmd += 'returnplanes %d '%returnplanes
        if returnfaces is not None:
            cmd += 'returnfaces %d '%returnfaces
        if returntriangles is not None:
            cmd += 'returntriangles %d '%returntriangles
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('ConvexHull')
        resvalues = res.split()
        planes = None
        faces = None
        triangles = None
        if returnplanes:
            numplanes = int(resvalues.pop(0))
            planes = reshape(array([float64(resvalues.pop(0)) for i in range((dim+1)*numplanes)],float64),(numplanes,dim+1))
        if returnfaces:
            numfaces = int(resvalues.pop(0))
            faces = []
            for i in range(numfaces):
                numvertices = int(resvalues.pop(0))
                faces.append(array([int(resvalues.pop(0)) for i in range(numvertices)],int))
        if returntriangles:
            numtriangles = int(resvalues.pop(0))
            triangles = reshape(array([int(resvalues.pop(0)) for i in range(3*numtriangles)],int),(numtriangles,3))
        return planes,faces,triangles
