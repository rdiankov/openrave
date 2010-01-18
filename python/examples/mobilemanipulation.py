#!/usr/bin/env python
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

import time,traceback
from openravepy import *
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.examples import inversereachability,graspplanning,grasping
from numpy import *
import numpy
from itertools import izip

class GraspReachability(metaclass.AutoReloader):
    def __init__(self,robot,target=None,gmodel=None,irmodel=None):
        self.robot = robot
        self.env = robot.GetEnv()
        self.manip = robot.GetActiveManipulator()
        if gmodel is not None and gmodel.has():
            self.gmodel = gmodel
        else:
            self.gmodel = grasping.GraspingModel(robot=robot,target=target)
            if not self.gmodel.load():
                self.gmodel.autogenerate()
        self.target = self.gmodel.target
        if irmodel is not None and irmodel.has():
            self.irmodel = irmodel
        else:
            self.irmodel = inversereachability.InverseReachabilityModel(robot=robot)
            if not self.irmodel.load():
                self.irmodel.autogenerate()

    def computeGraspDistribution(self,logllthresh=2000.0):
        validgrasps = self.gmodel.computeValidGrasps()
        def graspiter():
            for grasp in validgrasps:
                yield self.gmodel.getGlobalGraspTransform(grasp)
        densityfn,samplerfn,bounds = self.irmodel.computeAggregateBaseDistribution(graspiter(),logllthresh=logllthresh)
        return densityfn,samplerfn,bounds,validgrasps

    def sampleGoals(self,samplerfn,validgrasps,N=1):
        """samples a base placement and attemps to find an IK solution there.
        samplerfn should return a robot position and an index into validgrasps"""
        goals = []
        numfailures = 0
        with self.robot:
            while len(goals) < N:
                poses,indices = samplerfn(N-len(goals))
                for pose,index in izip(poses,indices):
                    self.robot.SetTransform(pose)
                    # before recording failure, validate that base is not in collision
                    if not self.manip.CheckIndependentCollision(CollisionReport()):
                        grasp = validgrasps[index]
                        self.gmodel.setPreshape(grasp)
                        q = self.manip.FindIKSolution(self.gmodel.getGlobalGraspTransform(grasp),envcheck=True)
                        if q is not None:
                            goals.append((grasp,pose,q))
                        elif not self.manip.FindIKSolution(self.gmodel.getGlobalGraspTransform(grasp),envcheck=False)
                            numfailures += 1
        return goals,numfailures

class MobileManipulationPlanning(graspplanning.GraspPlanning):
    def __init__(self,robot,randomize=False,irmodel=None,**kwargs):
        graspplanning.GraspPlanning.__init__(self, robot=robot,randomize=randomize,**kwargs)
        if irmodel is not None and irmodel.has():
            self.irmodel = irmodel
        else:
            self.irmodel = inversereachability.InverseReachabilityModel(robot=robot)
            if not self.irmodel.load():
                self.irmodel.autogenerate()

def run():
    env = Environment()
    try:
        env.SetViewer('qtcoin')
        env.Load('data/lab1.env.xml')
        robot = env.GetRobots()[0]
        env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        self = MobileManipulationPlanning(robot,randomize=True)
        self.performGraspPlanning()
    finally:
        env.Destroy()

if __name__ == "__main__":
    run()
