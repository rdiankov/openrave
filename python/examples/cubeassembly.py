#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
"""Assembly of cube from randomly scattered blocks using grasp sets.

.. examplepre-block:: cubeassembly

There's an ODE bug that can cause the demo to crash, if that happens try with a different collision checker like:

.. code-block:: bash

  openrave.py --example cubeassembly --collision=bullet

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
else:
    from numpy import eye

try:
    from multiprocessing import cpu_count
except:
    def cpu_count(): return 1

from openravepy.examples import graspplanning

class CubeAssembly(object):
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

    def CreateBlocks(self,side=0.015,T=eye(4),generategrasps=True):
        with self.env:
            for gmodel in self.gmodels:
                self.env.Remove(gmodel.target)
            self.gmodels = []
            volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
            blocks = [[[-1,-1,0],[0,-1,0],[-1,0,0],[-1,-1,1]],
                      [[-1,1,0],[-1,1,1],[-1,0,1]],
                      [[0,0,0],[0,1,0],[0,0,1],[0,-1,1]],
                      [[1,1,0],[1,0,0],[1,1,1],[0,1,1]],
                      [[1,-1,0],[1,-1,1],[1,0,1],[1,-1,2]],
                      [[0,0,2],[1,0,2],[0,1,2],[1,1,2]],
                      [[-1,-1,2],[0,-1,2],[-1,0,2],[-1,1,2]]
                      ]
            for iblock,block in enumerate(blocks):
                print 'creating block %d/%d, grasp set generation could take a couple of minutes...'%(iblock,len(blocks))
                color = volumecolors[mod(iblock,len(volumecolors))]
                boxes = []
                for geom in block:
                    boxes.append([(2*geom[0]+1)*side,(2*geom[1]+1)*side,(2*geom[2]+1)*side, 0.99*side,0.99*side,0.99*side])

                body=RaveCreateKinBody(self.env,'')
                body.InitFromBoxes(array(boxes),True)
                for g in body.GetLinks()[0].GetGeometries():
                    g.SetDiffuseColor(color)
                body.SetName('block%d'%iblock)
                self.env.Add(body,True)
                body.SetTransform(T)

                gmodel = databases.grasping.GraspingModel(robot=self.robot,target=body)
                if generategrasps:
                    if not gmodel.load():
                        approachrays = gmodel.computeBoxApproachRays(delta=0.01,normalanglerange=0,directiondelta=0)
                        gmodel.numthreads = cpu_count()
                        gmodel.generate(standoffs=array([0,0.04,0.08]),approachrays=approachrays, friction=0.1)
                        gmodel.save()
                self.gmodels.append(gmodel)

    def ShowGoal(self,Tgoal):
        savers = []
        try:
            with self.env:
                for gmodel in self.gmodels:
                    savers.append(KinBody.KinBodyStateSaver(gmodel.target))
                    gmodel.target.SetTransform(Tgoal)
            raw_input('press any key')
        finally:
            for saver in savers:
                saver.Restore()

    def SetGoal(self,Tgoal,randomize=True):
        """sets the goal of all the target bodies and randomizes the obstacles across the plane
        """
        print 'randomizing blocks, might take a couple of seconds...'
        if not randomize:
            self.Tgoal = Tgoal
            return
        
        with self.env:
            self.Tgoal = None
            while self.Tgoal is None:
                for gmodel in self.gmodels:
                    gmodel.target.SetTransform(Tgoal)
                minextents = array([-0.1,-0.2,0])
                maxextents = array([0.2,0.2,0])
                invalidgrasp = False
                for igmodel in range(len(self.gmodels)-1,-1,-1):
                    gmodel = self.gmodels[igmodel]
                    target=gmodel.target
                    T = eye(4)
                    target.SetTransform(T)
                    ab = target.ComputeAABB()
                    while True:
                        T = array(Tgoal)
                        target.SetTransform(T)
                        validgrasps,validindices=gmodel.computeValidGrasps(returnnum=1)
                        if len(validgrasps) == 0:
                            print 'no valid goal grasp for target %s'%gmodel.target
                            invalidgrasp = True
                            break

                        T[0:3,3] += (maxextents-minextents)*random.rand(3)+minextents
                        T[2,3] += 0.001-(ab.pos()[2]-ab.extents()[2])
                        if linalg.norm(Tgoal[0:3,3]-T[0:3,3]) < 0.1:
                            continue
                        target.SetTransform(T)
                        if not self.env.CheckCollision(target):
                            # have to check all previously moved targets still maintain their grasps
                            success = True
                            for igmodel2 in range(igmodel,len(self.gmodels)):
                                validgrasps,validindices=self.gmodels[igmodel2].computeValidGrasps(returnnum=1)
                                if len(validgrasps) == 0:
                                    success = False
                                    break
                            if success:
                                break
                            
                if not invalidgrasp:
                    self.Tgoal = Tgoal

    def Plan(self):
        if self.Tgoal is None:
            raise ValueError('need to set goal with SetGoal()')
        print 'planning...'
        planner=graspplanning.GraspPlanning(self.robot,randomize=False,dests=None,nodestinations=True)
        for gmodel in self.gmodels:
            print 'grasping %s'%gmodel.target
            success=-1
            while success < 0:
                success = planner.graspAndPlaceObject(gmodel,dests=[self.Tgoal],movehanddown=False)
                
def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    if len(options.manipname) > 0:
        robot.SetActiveManipulator(options.manipname)
    time.sleep(0.1) # give time for environment to update
    self = CubeAssembly(robot)
    self.CreateBlocks()
    while True:
        Tgoal = eye(4)
        Tgoal[0,3] = -0.2
        Tgoal[2,3] = 0.001
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
                      action="store",type='string',dest='scene',default='data/hironxtable.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',
                      action="store",type='string',dest='manipname',default='leftarm_torso',
                      help='The manipulator to use')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()

def test():
    import cubeassembly
    from openravepy.examples import graspplanning
    env.Load('data/hironxtable.env.xml')
    robot=env.GetRobots()[0]
    robot.SetActiveManipulator('leftarm_torso')
    self = cubeassembly.CubeAssembly(robot)
    self.CreateBlocks()
    Tgoal = eye(4)
    Tgoal[0,3] = -0.2
    Tgoal[2,3] = 0.001
    self.SetGoal(Tgoal)
    self.Plan()
