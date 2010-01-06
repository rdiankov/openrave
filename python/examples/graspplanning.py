#!/usr/bin/env python
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
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.examples.grasping import Grasping
from numpy import *
import numpy

class GraspPlanning(metaclass.AutoReloader):
    def __init__(self,env,robot,randomize=False,dests=None):
        self.env = env
        self.robot = robot
        self.basemanip = BaseManipulation(env,robot)
        self.taskmanip = TaskManipulation(env,robot)
        
        # find all the bodies to manipulate
        print 'searching for graspable objects...'
        self.graspable = []
        for target in self.env.GetBodies():
            if not target.IsRobot():
                grasping = Grasping(env,robot,target)
                if grasping.loadGrasps():
                    print '%s is graspable'%target.GetName()
                    if randomize:
                        Tbody = target.GetTransform()
                        for iter in range(5):
                            Tnew = array(Tbody)
                            Tnew[0,3] += -0.1 + 0.2 * random.rand()
                            Tnew[1,3] += -0.1 + 0.2 * random.rand()
                            target.SetTransform(Tnew)
                            if not self.env.CheckCollision(target):
                                Tbody = Tnew
                                break
                        target.SetTransform(Tbody)
                    self.graspable.append(grasping)
        
        if randomize: # randomize the robot
            Trobot = self.robot.GetTransform()
            for iter in range(5):
                Tnew = array(Trobot)
                Tnew[0,3] += -0.1 + 0.2 * random.rand()
                Tnew[1,3] += -0.1 + 0.2 * random.rand()
                self.robot.SetTransform(Tnew)
                if not self.env.CheckCollision(self.robot):
                    Trobot = Tnew
                    break
            self.robot.SetTransform(Trobot)
        
        self.dests = dests
        if self.dests is None:
            tablename = 'table'
            print 'searching for destinations on %s...'%tablename
            table = self.env.GetKinBody(tablename)
            if table is None:
                print 'could not find %s'%tablename
            else:
                Ttable = table.GetTransform()
                table.SetTransform(eye(4))
                ab = table.ComputeAABB()
                table.SetTransform(Ttable)
                p = ab.pos()
                e = numpy.minimum(ab.extents(),array((0.2,0.2,1)))
                Nx = floor(2*e[0]/0.1)
                Ny = floor(2*e[1]/0.1)
                X = []
                Y = []
#                 for x in arange(Nx):
#                     X = r_[X, random.rand(Ny)*0.5/(Nx+1) + (x+1)/(Nx+1)]
#                     Y = r_[Y, random.rand(Ny)*0.5/(Ny+1) + arange(0.5,Ny-0.5,1.0)/(Ny+1)]

def run():
    env = Environment()
    try:
        env.Load('data/lab1.env.xml')
        env.SetViewer('qtcoin')
        robot = env.GetRobots()[0]
        self = GraspPlanning(env,robot)
    finally:
        env.Destroy()

if __name__ == "__main__":
    run()
