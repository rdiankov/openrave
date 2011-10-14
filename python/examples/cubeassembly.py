#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
"""Simple cube assembly task using grasp sets

.. examplepre-block:: cubeassembly

Description
-----------

.. examplepost-block:: cubeassembly

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

import multiprocessing

from openravepy.examples import graspplanning

class CubeAssembly(metaclass.AutoReloader):
    def __init__(self,robot):
        self.env=robot.GetEnv()
        self.robot=robot
        manip=self.robot.GetActiveManipulator()
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D,freeindices=manip.GetArmIndices()[0:-6])
        if not self.ikmodel.load():
            self.ikmodel.autogenerate() # autogenerate if one doesn't exist

        self.lmodel = databases.linkstatistics.LinkStatisticsModel(self.robot)
        if self.lmodel.load():
            self.lmodel.setRobotWeights()
            self.lmodel.setRobotResolutions(xyzdelta=0.005)
            print 'robot resolutions: ',robot.GetDOFResolutions()
            print 'robot weights: ',robot.GetDOFWeights()

        self.basemanip = interfaces.BaseManipulation(self.robot)
        self.taskmanip = interfaces.TaskManipulation(self.robot)
        self.Tgoal = None
        self.gmodels = []

    def CreateBlocks(self,side=0.015,T=eye(4)):
        with self.env:
            for gmodel in self.gmodels:
                self.env.Remove(gmodel.target)
            self.gmodels = []
            volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
            blocks = [[[0,0,0],[1,0,0],[2,0,0],[2,1,0]],
                      [[0,1,0],[1,1,0],[1,1,1],[2,1,1]],
                      [[0,2,0],[1,2,0],[2,2,0],[1,2,1],[0,2,1]],
                      [[2,0,1],[1,0,1],[0,0,1],[0,0,2],[2,0,2]],
                      [[0,1,1],[0,1,2],[1,1,2],[1,0,2]],
                      [[2,1,2],[2,2,2],[1,2,2],[0,2,2],[2,2,1]]
                      ]
            for iblock,block in enumerate(blocks):
                print 'creating block %d/%d'%(iblock,len(blocks))
                color = volumecolors[mod(iblock,len(volumecolors))]
                boxes = []
                for geom in block:
                    boxes.append([(2*geom[0]+1)*side,(2*geom[1]+1)*side,(2*geom[2]+1)*side, 0.99*side,0.99*side,0.99*side])

                body=RaveCreateKinBody(self.env,'')
                body.InitFromBoxes(array(boxes),True)
                for g in body.GetLinks()[0].GetGeometries():
                    g.SetDiffuseColor(color)
                body.SetName('block%d'%iblock)
                self.env.AddKinBody(body,True)
                body.SetTransform(T)

                gmodel = databases.grasping.GraspingModel(robot=self.robot,target=body)
                if not gmodel.load():
                    approachrays = gmodel.computeBoxApproachRays(delta=0.015,normalanglerange=0,directiondelta=0)
                    gmodel.numthreads = multiprocessing.cpu_count()
                    gmodel.generate(standoffs=array([0,0.04,0.08]),approachrays=approachrays, friction=0.1)
                    gmodel.save()
                self.gmodels.append(gmodel)

    def SetGoal(self,Tgoal,randomize=True):
        with self.env:
            minextents = array([2.56,-2.9,0.24])
            maxextents = array([3.05,-3.153,0.24])
            for gmodel in self.gmodels:
                target=gmodel.target
                T = eye(4)
                target.SetTransform()
                ab = target.ComputeAABB()
                gmodel = databases.grasping.GraspingModel(robot=robot,target=target)
                if not gmodel.load():
                    approachrays = gmodel.computeBoxApproachRays(delta=0.01,normalanglerange=0,directiondelta=0)
                    gmodel.generate(standoffs=[0,0.04,0.08],approachrays=approachrays, friction=0.1)
                    gmodel.save()
                gmodels.append(gmodel)
                while True:
                    T[0:3,3] = (maxextents-minextents)*random.rand(3)+minextents
                    T[2,3] += 0.01-ab.pos()[2]
                    if linalg.norm(Tgoal[0:3,3]-T[0:3,3]) < 0.1:
                        continue
                    target.SetTransform(T)
                    if not self.env.CheckCollision(target):
                        validgrasps,validindices=gmodel.computeValidGrasps(returnnum=1)
                        if len(validgrasps) > 0:
                            break
            self.Tgoal = Tgoal

#             with gmodel.target:
#                 gmodel.target.SetTransform(Tgoal)
#                 self.env.UpdatePublishedBodies()
#                 raw_input('yo')
# 
#             try:
#                 with env:
#                     savers = []
#                     for gmodel in gmodels:
#                         savers.append(gmodel.target.CreateKinBodyStateSaver())
#                         gmodel.target.SetTransform(Tgoal)
#                 raw_input('press any key')
#             finally:
#                 del savers

    def Plan(self):
        if self.Tgoal is None:
            raise ValueError('need to set goal with SetGoal()')
        planner=graspplanning.GraspPlanning(robot,randomize=False,dests=None,nodestinations=True)
        for gmodel in gmodels:
            success=-1
            while success < 0:
                success = planner.graspAndPlaceObject(gmodel,dests=[Tgoal],movehanddown=False)

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    if len(options.manipname) > 0:
        robot.SetActiveManipulator(options.manipname)
    time.sleep(0.1) # give time for environment to update
    self = CubeAssembly(robot)
    self.CreateBlocks()
    Tgoal = eye(4)
    Tgoal[0,3] = -0.2
    self.SetGoal(Tgoal)
    self.Plan()

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Simple cube assembly task using grasp sets.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='robots/kawada-hironx.zae',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',
                      action="store",type='string',dest='manipname',default='leftarm_torso',
                      help='The manipulator to use')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__ == "__main__":
    run()

def test():
    import cubeassembly
    env.Load('data/hironxtable.env.xml')
    robot=env.GetRobots()[0]
    robot.SetActiveManipulator('leftarm_torso')
    self = cubeassembly.CubeAssembly(robot)
    self.CreateBlocks()
    Tgoal = eye(4)
    Tgoal[0,3] = -0.2
    self.SetGoal(Tgoal)
    self.Plan()
