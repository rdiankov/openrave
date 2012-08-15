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
"""Combine the power of grasp sets and randomized planners to get any robot arm picking up objects from a table and putting them in a dish rack. 

.. examplepre-block:: graspplanning

Description
-----------

The example uses the powerful TaskManipulation problem interface, which takes advantage of many OpenRAVE features. It performs:

* Pick grasps and validate them with the grasper planner
* Move to the appropriate grasp preshape while avoiding obstacles
* Use an RRT and Jacobian-based gradient descent methods to safely move close to an obstacle
* Use CloseFingers to grasp the object while checking for collisions with other unwanted objects
* Use body grabbing to grasp the object and move it to its destination
* Lower the object until collision and then release and move away from it. 

The scene is randomized every run in order to show the powerful of the planners.


.. figure:: ../../images/examples/graspplanning_gallery.jpg
  :width: 640
  
  Gallery of runs.

Destinations
============

By default, the grasp planner will choose a grasp that is also valid at a destination point. If
running on custom environments, sometimes it is interesting to see if the robot can just grasp the
object, without moving it to the destination. To test planning without destinations use:

.. code-block:: bash

  openrave.py --example graspplanning --nodestinations

5D IK Grasp Planning
====================

It is possible to perform grasp planning with 5D IK.

Neuronics Katana
~~~~~~~~~~~~~~~~

First create the grasp set with:

.. code-block:: bash

  openrave.py --database grasping --robot=robots/neuronics-katana.zae --manipname=arm --target=data/box_frootloops.kinbody.xml --manipulatordirection="0 1 0"

Then execute:

.. code-block:: bash

  openrave.py --example graspplanning --scene=data/katanatable.env.xml

.. figure:: ../../images/examples/graspplanning_katana.jpg
  :width: 640

Kuka Youbot
~~~~~~~~~~~

Grasp set generation:

.. code-block:: bash

  openrave.py --database grasping --robot=robots/kuka-youbot.zae --manipulatordirection="0 1 0" --target=thinbox.kinbody.xml

Then execute:

.. code-block:: bash

  openrave.py --example graspplanning --scene=data/youbot1.env.xml

.. examplepost-block:: graspplanning

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
from itertools import izip
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

try:
    from multiprocessing import cpu_count
except:
    def cpu_count(): return 1


class GraspPlanning:
    def __init__(self,robot,randomize=True,dests=None,nodestinations=False,switchpatterns=None,plannername=None,minimumgoalpaths=1):
        self.envreal = robot.GetEnv()
        self.robot = robot
        self.plannername=plannername
        self.nodestinations = nodestinations
        self.minimumgoalpaths=minimumgoalpaths
        try:
            self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()
        except ValueError:
            print '6D IK failed, trying 5D IK'
            self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.TranslationDirection5D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()

        self.lmodel = databases.linkstatistics.LinkStatisticsModel(self.robot)
        if not self.lmodel.load():
            self.lmodel.autogenerate()
        self.lmodel.setRobotWeights()
        self.lmodel.setRobotResolutions(xyzdelta=0.005)
        print 'robot resolutions: ',robot.GetDOFResolutions()
        print 'robot weights: ',robot.GetDOFWeights()

        # could possibly affect generated grasp sets?
#         self.cdmodel = databases.convexdecomposition.ConvexDecompositionModel(self.robot)
#         if not self.cdmodel.load():
#             self.cdmodel.autogenerate()
        self.switchpatterns = switchpatterns
        with self.envreal:
            self.basemanip = interfaces.BaseManipulation(self.robot,plannername=plannername)
            self.basemanip.prob.SendCommand('SetMinimumGoalPaths %d'%self.minimumgoalpaths)
            self.taskmanip = None
            self.updir = array((0,0,1))
            
            # find all the bodies to manipulate
            self.graspables = self.getGraspables(dests=dests)
            if len(self.graspables) == 0:
                print 'attempting to auto-generate a grasp table'
                targets=[t for t in self.envreal.GetBodies() if t.GetName().find('mug')>=0 or t.GetName().find('target')>=0]
                if len(targets) > 0:
                    gmodel = databases.grasping.GraspingModel(robot=self.robot,target=targets[0])
                    if not gmodel.load():
                        gmodel.numthreads = cpu_count()
                        gmodel.autogenerate()
                        self.graspables = self.getGraspables(dests=dests)

            self.randomize=randomize
            if self.randomize:
                self.randomizeObjects()

            if dests is None and not self.nodestinations:
                tablename = 'table'
                table = self.envreal.GetKinBody(tablename)
                if table is not None:
                    alltargets = [graspable[0].target for graspable in self.graspables]
                    for target in alltargets:
                        target.Enable(False)
                    try:
                        needdests_graspables = [graspable for graspable in self.graspables if graspable[1] is None]
                        curdests = [graspable[0].target.GetTransform() for graspable in needdests_graspables]
                        alldests = self.setRandomDestinations([graspable[0].target for graspable in needdests_graspables],table)
                        for graspable,dests in izip(needdests_graspables,alldests):
                            graspable[1] = dests+curdests
                    finally:
                        for target in alltargets:
                            target.Enable(True)
                else:
                    print 'could not find %s'%tablename

    def getGraspables(self,dests=None):
        graspables = []
        print 'searching for graspable objects (robot=%s)...'%(self.robot.GetRobotStructureHash())
        for target in self.envreal.GetBodies():
            if not target.IsRobot():
                gmodel = databases.grasping.GraspingModel(robot=self.robot,target=target)
                if gmodel.load():
                    print '%s is graspable'%target.GetName()
                    graspables.append([gmodel,dests])
        return graspables

    def randomizeObjects(self):
        for graspable in self.graspables:
            target = graspable[0].target
            Tbody = target.GetTransform()
            for iter in range(5):
                Tnew = array(Tbody)
                Tnew[0,3] += -0.1 + 0.2 * random.rand()
                Tnew[1,3] += -0.1 + 0.2 * random.rand()
                target.SetTransform(Tnew)
                if not self.envreal.CheckCollision(target):
                    Tbody = Tnew
                    break
            target.SetTransform(Tbody)

        # randomize the robot
        Trobot = self.robot.GetTransform()
        for iter in range(5):
            Tnew = array(Trobot)
            Tnew[0,3] += -0.1 + 0.2 * random.rand()
            Tnew[1,3] += -0.1 + 0.2 * random.rand()
            self.robot.SetTransform(Tnew)
            if not self.envreal.CheckCollision(self.robot):
                Trobot = Tnew
                break
        self.robot.SetTransform(Trobot)

    @staticmethod
    def setRandomDestinations(targets, table,transdelta=0.1,zoffset=0.01,Trolls=None,randomize=False,preserverotation=True):
        with table.GetEnv():
            print 'searching for destinations on %s...'%table.GetName()
            Ttable = table.GetTransform()
            table.SetTransform(eye(4))
            ab = table.ComputeAABB()
            table.SetTransform(Ttable)
            p = ab.pos()
            e = ab.extents()
            Nx = floor(2*e[0]/transdelta)
            Ny = floor(2*e[1]/transdelta)
            X = []
            Y = []
            if randomize:
                for x in arange(Nx):
                    X = r_[X, random.rand(Ny)*0.5/(Nx+1) + (x+1)/(Nx+1)]
                    Y = r_[Y, random.rand(Ny)*0.5/(Ny+1) + arange(0.5,Ny,1.0)/(Ny+1)]
            else:
                for x in arange(Nx):
                    X = r_[X, tile((x+1)/(Nx+1),Ny)]
                    Y = r_[Y, arange(0.5,Ny,1.0)/(Ny+1)]
            translations = c_[p[0]-e[0]+2*e[0]*X,p[1]-e[1]+2*e[1]*Y,tile(p[2]+e[2]+zoffset,len(X))]
            if Trolls is None:
                Trolls = [matrixFromAxisAngle(array((0,0,1)),roll) for roll in arange(0,2*pi,pi/2)] + [matrixFromAxisAngle(array((1,0,0)),roll) for roll in [pi/2,pi,1.5*pi]]
            for target in targets:
                target.Enable(False)
            try:
                alldests = []
                for target in targets:
                    Torg = eye(4)
                    if preserverotation:
                        Torg[0:3,0:3] = target.GetTransform()[0:3,0:3]
                    with target.CreateKinBodyStateSaver():
                        target.Enable(True)
                        dests = []
                        for translation in translations:
                            for Troll in Trolls:
                                Troll = array(Troll)
                                Troll[0:3,3] = translation
                                target.SetTransform(dot(Ttable, dot(Troll, Torg)))
                                if not table.GetEnv().CheckCollision(target):
                                    dests.append(target.GetTransform())
                        alldests.append(dests)
                return alldests
            finally:
                for target in targets:
                    target.Enable(True)

    def viewDestinations(self,gmodel,Tdests,delay=0.5):
        with gmodel.target:
            for i,T in enumerate(Tdests):
                print 'target %s dest %d/%d'%(gmodel.target.GetName(),i,len(Tdests))
                gmodel.target.SetTransform(T)
                validgrasps, indices = gmodel.computeValidGrasps(returnnum=1)
                gmodel.target.GetEnv().UpdatePublishedBodies()
                gmodel.showgrasp(validgrasps[0],useik=True,collisionfree=True,delay=delay)

    def waitrobot(self,robot=None):
        """busy wait for robot completion"""
        if robot is None:
            robot = self.robot
        while not robot.GetController().IsDone():
            time.sleep(0.01)
            
    def graspAndPlaceObject(self,gmodel,dests,waitforkey=False,movehanddown=True,**kwargs):
        """grasps an object and places it in one of the destinations. If no destination is specified, will just grasp it"""
        env = self.envreal#.CloneSelf(CloningOptions.Bodies)
        robot = self.robot
        with env:
            self.taskmanip = interfaces.TaskManipulation(self.robot,graspername=gmodel.grasper.plannername,plannername=self.plannername)
            self.taskmanip.prob.SendCommand('SetMinimumGoalPaths %d'%self.minimumgoalpaths)
            if self.switchpatterns is not None:
                self.taskmanip.SwitchModels(switchpatterns=self.switchpatterns)
            robot.SetActiveManipulator(gmodel.manip)
            robot.SetActiveDOFs(gmodel.manip.GetArmIndices())
        istartgrasp = 0
        approachoffset = 0.02 if self.ikmodel.iktype == IkParameterization.Type.Transform6D else 0.0
        target = gmodel.target
        stepsize = 0.001
        while istartgrasp < len(gmodel.grasps):
            goals,graspindex,searchtime,trajdata = self.taskmanip.GraspPlanning(gmodel=gmodel,grasps=gmodel.grasps[istartgrasp:], approachoffset=approachoffset,destposes=dests, seedgrasps = 3,seeddests=8,seedik=1,maxiter=1000, randomgrasps=self.randomize,randomdests=self.randomize)
            istartgrasp = graspindex+1
            grasp = gmodel.grasps[graspindex]
            Tglobalgrasp = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
            self.waitrobot(robot)
            print 'grasp %d initial planning time: %f'%(graspindex,searchtime)
            
            if approachoffset != 0:
                print 'moving hand'
                expectedsteps = floor(approachoffset/stepsize)
                try:
                    # should not allow any error since destination goal depends on accurate relative placement
                    # of the gripper with respect to the object
                    with gmodel.target:
                        print 'current robot', repr(robot.GetDOFValues())
                        print 'global direction',repr(dot(gmodel.manip.GetTransform()[0:3,0:3],gmodel.manip.GetDirection())), gmodel.getGlobalApproachDir(grasp)
                        print 'local direction',grasp[gmodel.graspindices.get('igraspdir')]
                        gmodel.target.Enable(False)
                        res = self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp), ignorefirstcollision=0,stepsize=stepsize,minsteps=expectedsteps,maxsteps=expectedsteps)
                except planning_error:
                    print 'use a planner to move the rest of the way'
                    try:
                        self.basemanip.MoveToHandPosition(matrices=[Tglobalgrasp],maxiter=1000,maxtries=1,seedik=4)
                    except planning_error,e:
                        print 'failed to reach grasp',e
                        continue
                self.waitrobot(robot)

            self.taskmanip.CloseFingers(translationstepmult=gmodel.translationstepmult,finestep=gmodel.finestep)
            self.waitrobot(robot)            

            with env:
                robot.Grab(target)
            if waitforkey:
                raw_input('press any key to continue grasp')

            success = graspindex
            if movehanddown:
                try:
                    print 'move hand up'
                    self.basemanip.MoveHandStraight(direction=self.updir,stepsize=0.003,minsteps=1,maxsteps=60)
                except:
                    print 'failed to move hand up'
                self.waitrobot(robot)

            if len(goals) > 0:
                print 'planning to destination'
                try:
                    self.basemanip.MoveToHandPosition(ikparams=goals,maxiter=2000,maxtries=2,seedik=8)
                    self.waitrobot(robot)
                except planning_error,e:
                    print 'failed to reach a goal, trying to move goal a little up',e
                    if goals[0].GetType() == IkParameterizationType.Transform6D:
                        Tgoal = goals[0].GetTransform6D()
                        Tgoal[0:3,3] += self.updir*0.015
                        try:
                            self.basemanip.MoveToHandPosition(matrices=[Tgoal],maxiter=3000,maxtries=2,seedik=8)
                            self.waitrobot(robot)
                            self.basemanip.MoveToHandPosition(ikparams=goals,maxiter=2000,maxtries=2,seedik=8)
                            self.waitrobot(robot)
                        except planning_error,e:
                            print e
                            success = -1
            if movehanddown:
                print 'moving hand down'
                try:
                    res = self.basemanip.MoveHandStraight(direction=-self.updir,stepsize=0.003,minsteps=1,maxsteps=100)
                except:
                    print 'failed to move hand down'
                self.waitrobot(robot)

            try:
                res = self.taskmanip.ReleaseFingers(target=target,translationstepmult=gmodel.translationstepmult,finestep=gmodel.finestep)
            except planning_error:
                res = None
            if res is None:
                print 'problems releasing, releasing target first'
                with env:
                    robot.ReleaseAllGrabbed()
                    try:
                        res = self.taskmanip.ReleaseFingers(target=target,translationstepmult=gmodel.translationstepmult,finestep=gmodel.finestep)
                    except planning_error:
                        res = None
                if res is None:
                    print 'forcing fingers'
                    with env:
                        robot.SetDOFValues(gmodel.grasps[graspindex][gmodel.graspindices['igrasppreshape']],gmodel.manip.GetGripperIndices())
            self.waitrobot(robot)
            with env:
                robot.ReleaseAllGrabbed()
            if env.CheckCollision(robot):
                print 'robot in collision, moving back a little'
                try:
                    self.basemanip.MoveHandStraight(direction=-dot(gmodel.manip.GetTransform()[0:3,0:3],gmodel.manip.GetDirection()), stepsize=stepsize,minsteps=1,maxsteps=10)
                    self.waitrobot(robot)
                except planning_error,e:
                    pass
                if env.CheckCollision(robot):
                    try:
                        self.taskmanip.ReleaseFingers(target=target,translationstepmult=gmodel.translationstepmult,finestep=gmodel.finestep)
                    except planning_error:
                        res = None
                    #raise ValueError('robot still in collision?')

            if success >= 0:
                return success # return successful grasp index
            
        # exhausted all grasps
        return -1

    def performGraspPlanning(self,withreplacement=True,**kwargs):
        print 'starting to pick and place random objects'
        graspables = self.graspables[:]
        failures = 0
        while True:
            if len(graspables) == 0 or failures > len(graspables)+1:
                if withreplacement:
                    time.sleep(4)
                    self.randomizeObjects()
                    graspables = self.graspables[:]
                else:
                    break

            if self.randomize:
                i=random.randint(len(graspables))
            else:
                i = 0
                
            try:
                print 'grasping object %s'%graspables[i][0].target.GetName()
                with self.envreal:
                    self.robot.ReleaseAllGrabbed()
                success = self.graspAndPlaceObject(graspables[i][0],graspables[i][1],**kwargs)
                print 'success: ',success
                graspables.pop(i)
                failures = 0
            except planning_error, e:
                print 'failed to grasp object %s'%graspables[i][0].target.GetName()
                failures += 1
                graspables.append(graspables.pop(0)) # push front to back
                print e

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = GraspPlanning(robot,randomize=options.randomize,nodestinations=options.nodestinations,plannername=options.planner)
    self.performGraspPlanning(withreplacement=not options.testmode)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Autonomous grasp and manipulation planning example.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--nodestinations', action='store_true',dest='nodestinations',default=False,
                      help='If set, will plan without destinations.')
    parser.add_option('--norandomize', action='store_false',dest='randomize',default=True,
                      help='If set, will not randomize the bodies and robot position in the scene.')
    parser.add_option('--planner',action="store",type='string',dest='planner',default=None,
                      help='the planner to use')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()

def test():
    import graspplanning
    self=graspplanning.GraspPlanning(robot,randomize=False,nodestinations=False)
    success = self.graspAndPlaceObject(self.graspables[2][0],self.graspables[2][1])
