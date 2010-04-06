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
    def __init__(self,robot,target=None,irgmodels=None):
        self.robot = robot
        self.env = robot.GetEnv()
        if irgmodels is None:
            gmodel = grasping.GraspingModel(robot=robot,target=target)
            if not gmodel.load():
                gmodel.autogenerate()
            irmodel = inversereachability.InverseReachabilityModel(robot=robot)
            if not irmodel.load():
                irmodel.autogenerate()
            self.irgmodels = [[irmodel,gmodel]]
        else:
            # check for consistency
            for irmodel,gmodel in irgmodels:
                assert irmodel.robot == self.robot and irmodel.robot == gmodel.robot and irmodel.manip == gmodel.manip
            self.irgmodels = irgmodels

    def computeGraspDistribution(self,randomgrasps=False,**kwargs):
        """computes distribution of all grasps"""
        densityfns = []
        samplerfns = []
        totalbounds = None
        for irmodel,gmodel in self.irgmodels:
            #validgrasps,validindices = gmodel.computeValidGrasps(checkik=False,backupdist=0.01)
            def graspiter():
                for grasp,graspindex in izip(validgrasps,validindices):
                    yield gmodel.getGlobalGraspTransform(grasp,collisionfree=True),(gmodel,graspindex)
            densityfn,samplerfn,bounds = irmodel.computeAggregateBaseDistribution(graspiter(),**kwargs)
            densityfns.append(densityfn)
            samplerfns.append(samplerfns)
            if totalbounds is None:
                totalbounds = bounds
            else:
                bounds = array((numpy.minimum(bounds[0,:],totalbounds[0,:]),numpy.maximum(bounds[1,:],totalbounds[1,:])))
        def totaldensityfn(**kwargs):
            res = None
            for densityfn in densityfns:
                s = densityfn(**kwargs)
                res = res+s if res is not None else s
            return res
        def totalsamplerfn(**kwargs):
            return samplerfns[random.randint(len(samplerfns))](**kwargs)
        return totaldensityfn,totalsamplerfn,totalbounds
    def sampleGoals(self,samplerfn,N=1,updateenv=False,timeout=inf):
        """samples a base placement and attemps to find an IK solution there.
        samplerfn should return a robot position and an index into gmodel.grasps"""
        goals = []
        numfailures = 0
        starttime = time.time()
        with self.robot:
            while len(goals) < N:
                if time.time()-starttime > timeout:
                    return goals,numfailures
                poses,graspindices,jointstate = samplerfn(N-len(goals))
                for pose,graspindex in izip(poses,graspindices):
                    gmodel = graspindex[0]
                    self.robot.SetTransform(pose)
                    self.robot.SetJointValues(*jointstate)
                    if updateenv:
                        self.env.UpdatePublishedBodies()
                    # before recording failure, validate that base is not in collision
                    if not gmodel.manip.CheckIndependentCollision(CollisionReport()):
                        grasp = gmodel.grasps[graspindex[1]]
                        gmodel.setPreshape(grasp)
                        q = gmodel.manip.FindIKSolution(gmodel.getGlobalGraspTransform(grasp,collisionfree=True),envcheck=True)
                        if q is not None:
                            values = self.robot.GetJointValues()
                            values[gmodel.manip.GetArmJoints()] = q
                            goals.append((grasp,pose,values))
                        elif gmodel.manip.FindIKSolution(gmodel.getGlobalGraspTransform(grasp,collisionfree=True),envcheck=False) is None:
                            numfailures += 1
        return goals,numfailures
    def sampleValidPlacementIterator(self,giveuptime=inf,randomgrasps=False,updateenv=False,randomplacement=False,**kwargs):
        """continues to sample valid goal placements. Returns the robot base position, configuration, and the target grasp from gmodel. Environment should be locked, robot state is saved"""
        starttime = time.time()
        origjointvalues = self.robot.GetJointValues()
        statesaver = self.robot.CreateRobotStateSaver()
        try:
            baseIterators = []
            for irmodel,gmodel in self.irgmodels:
                def graspiter():
                    for grasp,graspindex in gmodel.validGraspIterator(checkik=False,randomgrasps=randomgrasps,backupdist=0.01):
                        yield gmodel.getGlobalGraspTransform(grasp,collisionfree=True),(gmodel,graspindex)
                baseIterator = irmodel.randomBaseDistributionIterator if randomplacement else irmodel.sampleBaseDistributionIterator
                baseIterators.append((baseIterator(Tgrasps=graspiter(),**kwargs),irmodel,gmodel))
            while True:
                iterindex = random.randint(len(baseIterators))
                baseIterator,irmodel,gmodel = baseIterators[iterindex]
                try:                    
                    pose,graspindex,jointstate = baseIterator.next()
                except planning_error:
                    baseIterators.pop(iterindex)
                    continue
                self.robot.SetTransform(pose)
                self.robot.SetJointValues(origjointvalues)
                if not self.env.CheckCollision(self.robot,CollisionReport()):
                    self.robot.SetJointValues(*jointstate)
                    if updateenv:
                        self.env.UpdatePublishedBodies()
                    #irmodel.manip.CheckIndependentCollision(CollisionReport()):
                    gmodel = graspindex[0]
                    grasp = gmodel.grasps[graspindex[1]]
                    gmodel.setPreshape(grasp)
                    q = irmodel.manip.FindIKSolution(gmodel.getGlobalGraspTransform(grasp,collisionfree=True),envcheck=True)
                    if q is not None:
                        values = self.robot.GetJointValues()
                        values[irmodel.manip.GetArmJoints()] = q
                        statesaver.close()
                        yield pose,values,grasp,graspindex
                        statesaver = self.robot.CreateRobotStateSaver()
                        starttime = time.time()
                    else:
                        if not isinf(giveuptime) and time.time()-starttime > giveuptime:
                            raise planning_error('timed out')
        finally:
            statesaver.close()
    def showBaseDistribution(self,thresh=1.0,zoffset=0.5,**kwargs):
        starttime = time.time()
        densityfn,samplerfn,bounds = self.computeGraspDistribution(**kwargs)
        print 'time to build distribution: %fs'%(time.time()-starttime)
        return inversereachability.InverseReachabilityModel.showBaseDistribution(self.env,densityfn,bounds,zoffset=zoffset,thresh=thresh)
    def testSampling(self,**kwargs):
        """random sample goals on the current environment"""
        with self.env:
            configsampler = self.sampleValidPlacementIterator(**kwargs)
            basemanip = BaseManipulation(self.robot)
            jointvalues = self.robot.GetJointValues()
        with RobotStateSaver(self.robot):
            while True:
                try:
                    with self.env:
                        self.robot.SetJointValues(jointvalues) # reset to original
                        pose,values,grasp,graspindex = configsampler.next()
                        gmodel = graspindex[0]
                        print 'found grasp',gmodel.target.GetName(),graspindex[1]
                        self.robot.SetActiveManipulator(gmodel.manip)
                        self.robot.SetTransform(pose)
                        gmodel.setPreshape(grasp)
                        self.robot.SetJointValues(values)
                        final,traj = basemanip.CloseFingers(execute=False,outputfinal=True)
                        self.robot.SetJointValues(final,gmodel.manip.GetGripperJoints())
                        self.env.UpdatePublishedBodies()
                except planning_error, e:
                    traceback.print_exc(e)
                    continue

class MobileManipulationPlanning(metaclass.AutoReloader):
    def __init__(self,robot,grmodel,switchpatterns=None):
        self.envreal = robot.GetEnv()
        self.env=self.envreal
        self.robot = robot
        self.grmodel = grmodel
        self.switchpatterns = switchpatterns
        with self.envreal:
            self.basemanip = BaseManipulation(self.robot)
            self.taskmanip = TaskManipulation(self.robot)
            self.updir = array((0,0,1))

    def waitrobot(self):
        """busy wait for robot completion"""
        while not self.robot.GetController().IsDone():
            time.sleep(0.01)

    def graspObject(self,gmodel):
        print 'graspplanning grasp and place object script'
        self.robot.SetActiveManipulator(gmodel.manip)
        gp = graspplanning.GraspPlanning(self.robot,randomize=False,nodestinations=True,switchpatterns=self.switchpatterns)
        return gp.graspAndPlaceObject(gmodel,None)

    def graspAndPlaceObjectMobileSearch(self,targetdests):
        assert(len(targetdests)>0)
        weight = 1.5
        logllthresh = 0.5
        origjointvalues = self.robot.GetJointValues()
        configsampler = self.grmodel.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=False,updateenv=False)
        with self.env:
            starttime=time.time()
            while True:
                pose,values,grasp,graspindex = configsampler.next()
                gmodel = graspindex[0]
                dests = [dests for target,dests in targetdests if target == gmodel.target]
                if len(dests) == 0:
                    continue
                print 'found grasp in %fs'%(time.time()-starttime),gmodel.target.GetName(),graspindex[1]
                # check all destinations
                self.robot.SetActiveManipulator(gmodel.manip)
                Tgrasp = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
                Trelative = dot(linalg.inv(gmodel.target.GetTransform()),Tgrasp)
                Ttarget = None
                try:
                    gmodel.target.Enable(False)
                    with KinBodyStateSaver(self.robot):
                        gmodel.setPreshape(grasp)
                        for T in dests[0]:
                            Tnewgrasp = dot(T,Trelative)
                            if gmodel.manip.FindIKSolution(Tnewgrasp,True) is not None:
                                Ttarget = T
                                break
                    if Ttarget is not None:
                        break
                finally:
                    gmodel.target.Enable(True)
        self.graspObjectMobile(pose,values,grasp,graspindex)
        # if everything finished correctly, put object in destination        
        self.basemanip.MoveToHandPosition([Tnewgrasp])
    
    def graspObjectMobileSearch(self):
        weight = 1.5
        logllthresh = 0.5
        origjointvalues = self.robot.GetJointValues()
        configsampler = self.grmodel.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=False,updateenv=False)
        with self.env:
            starttime=time.time()
            sample = configsampler.next()
            print 'found in %fs'%(time.time()-starttime)
        return self.graspObjectMobile(*sample)
    def graspObjectMobile(self,pose,values,grasp,graspindex):
        approachoffset = 0.02
        stepsize = 0.001
        gmodel = graspindex[0]
        print 'found grasp',gmodel.manip.GetName(),gmodel.target.GetName(),graspindex[1]
        self.robot.SetActiveManipulator(gmodel.manip)
        self.moveBase(pose)
        with self.env:
            dofindices = inversereachability.InverseReachabilityModel.getdofindices(gmodel.manip)
            if len(dofindices) > 0:
                self.robot.SetActiveDOFs(dofindices)
                self.basemanip.MoveActiveJoints(goal=values[dofindices])
        self.waitrobot()
        gmodel.moveToPreshape(grasp)
        self.robot.SetActiveManipulator(gmodel.manip)
        with self.env:
            armjoints=gmodel.manip.GetArmJoints()
            finalarmsolution = values[armjoints]
            # although values holds the final configuration, robot should approach from a distance of approachoffset
            Tgrasp = gmodel.getGlobalGraspTransform(grasp)
            Tfirstgrasp = array(Tgrasp)
            Tfirstgrasp[0:3,3] -= approachoffset*gmodel.getGlobalApproachDir(grasp)
            solutions = gmodel.manip.FindIKSolutions(Tfirstgrasp,True)
            if len(solutions) == 0:
                return self.graspObject(gmodel)
            # find the closest solution
            weights = self.robot.GetJointWeights()[armjoints]
            dists = [numpy.max(abs(finalarmsolution-s)*weights) for s in solutions]
            index = argmin(dists)            
            if sqrt(dists[index]) > 5.0*approachoffset:
                print 'closest solution is too far',sqrt(dists[index])
                return self.graspObject(gmodel)
            self.robot.SetActiveDOFs(armjoints)
            self.basemanip.MoveActiveJoints(goal=solutions[index])
        self.waitrobot()

        try:
            print 'moving hand'
            expectedsteps = floor(approachoffset/stepsize)
            self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp),ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps-2,maxsteps=expectedsteps+1)
        except:
            print 'failed to move straight, using planning to move rest of the way'
            try:
                self.basemanip.MoveActiveJoints(goal=finalarmsolution)
            except:
                return self.graspObject(gmodel)
        self.waitrobot()

        self.basemanip.CloseFingers()
        self.waitrobot()
        self.robot.Grab(gmodel.target)
        try:
            self.basemanip.MoveHandStraight(direction=self.updir,jacobian=0.02,stepsize=0.002,minsteps=1,maxsteps=100)
        except:
            print 'failed to move up'
        self.waitrobot()

        # move to original position
        try:
            self.basemanip.MoveActiveJoints(goal=origjointvalues[armjoints])
        except:
            print 'failed to move cup closer to robot'
        return gmodel
    
    def moveBase(self,pose):
        with self.robot:
            self.robot.SetTransform(pose)
            if self.env.CheckCollision(self.robot):
                raise planning_error('goal position in self collision')
        with self.env: # find the boundaries of the environment
            envmin = []
            envmax = []
            for b in self.env.GetBodies():
                ab = b.ComputeAABB()
                envmin.append(ab.pos()-ab.extents())
                envmax.append(ab.pos()+ab.extents())
            abrobot = self.robot.ComputeAABB()
            envmin = numpy.min(array(envmin),0)+abrobot.extents()
            envmax = numpy.max(array(envmax),0)-abrobot.extents()
            bounds = array(((envmin[0],envmin[1],-pi),(envmax[0],envmax[1],pi)))
            self.robot.SetAffineTranslationLimits(envmin,envmax)
            self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
            self.robot.SetAffineRotationAxisMaxVels(ones(4))
            self.robot.SetActiveDOFs([],Robot.DOFAffine.X|Robot.DOFAffine.Y|Robot.DOFAffine.RotationAxis,[0,0,1])
            
            goal2d = [pose[4],pose[5],2.0*numpy.arctan2(pose[3],pose[0])]
            print 'planning to: ',goal2d
            center = r_[goal2d[0:2],0.2]
            xaxis = 0.5*array((cos(goal2d[2]),sin(goal2d[2]),0))
            yaxis = 0.25*array((-sin(goal2d[2]),cos(goal2d[2]),0))
            self.hgoal = self.env.drawlinelist(transpose(c_[center-xaxis,center+xaxis,center-yaxis,center+yaxis]),linewidth=5.0,colors=array((0,1,0)))
            if self.basemanip.MoveActiveJoints(goal=goal2d,maxiter=3000,steplength=0.05) is None:
                raise planning_error('failed to plan goal position')
        print 'waiting for controller'
        self.robot.WaitForController(0)
            
    def performGraspPlanning(self):
        print 'starting to pick and place random objects'
        while True:
            i = random.randint(len(self.graspables))
            try:
                print 'grasping object %s'%self.graspables[i][0].target.GetName()
#                 if len(self.graspables[i]) == 2:
#                     self.graspables[i].append(GraspReachability(robot=self.robot,gmodel=self.graspables[i][0],irmodel=self.irmodel))
# 
#                 with self.envreal:
#                     self.robot.ReleaseAllGrabbed()
#                 success = self.graspAndPlaceObjectMobile(grmodel=self.graspables[i][2],dests=self.graspables[i][1])
#                 print 'success: ',success
            except e:
                print 'failed to grasp object %s'%self.graspables[i][0].target.GetName()
                print e
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
    #run()
    print 'not implemented yet'
