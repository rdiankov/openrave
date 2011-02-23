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
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import time,traceback
from copy import copy as shallowcopy
from openravepy import *
from openravepy.interfaces import BaseManipulation, TaskManipulation, VisualFeedback
from openravepy.databases import inversekinematics,inversereachability,grasping,visibilitymodel
from openravepy.examples import graspplanning
from numpy import *
import numpy
from itertools import izip
# try:
#     import scipy # used for showing images
# except ImportError:
#     pass

class GraspReachability(metaclass.AutoReloader):
    def __init__(self,robot,target=None,irmodels=None,irgmodels=None,maxvelmult=None):
        self.robot = robot
        self.env = robot.GetEnv()
        self.irmodels=irmodels
        self.maxvelmult=maxvelmult
        self.irgmodels = []
        if irgmodels is None:
            if target is not None:
                gmodel = grasping.GraspingModel(robot=robot,target=target,maxvelmult=maxvelmult)
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
    def clone(self,envother):
        clone = shallowcopy(self)
        clone.env = envother
        clone.robot = clone.env.GetRobot(self.robot.GetName())
        clone.irmodels = []
        for irmodel in self.irmodels:
            try:
                clone.irmodels.append(irmodel.clone(envother))
            except openrave_exception,e:
                print e
        clone.irgmodels = []
        for irmodel,gmodel in self.irgmodels:
            try:
                clone.irgmodels.append([irmodel.clone(envother),gmodel.clone(envother)])
            except openrave_exception,e:
                print e
        return clone
    def computeGraspDistribution(self,randomgrasps=False,**kwargs):
        """computes distribution of all grasps"""
        densityfns = []
        samplerfns = []
        totalbounds = None
        for irmodel,gmodel in self.irgmodels:
            validgrasps,validindices = gmodel.computeValidGrasps(checkik=False,backupdist=0.01)
            def graspiter():
                for grasp,graspindex in izip(validgrasps,validindices):
                    yield gmodel.getGlobalGraspTransform(grasp,collisionfree=True),(gmodel,graspindex)
            densityfn,samplerfn,bounds = irmodel.computeAggregateBaseDistribution(graspiter(),**kwargs)
            if densityfn is not None:
                densityfns.append(densityfn)
                samplerfns.append(samplerfn)
                if totalbounds is None:
                    totalbounds = bounds
                else:
                    bounds = array((numpy.minimum(bounds[0,:],totalbounds[0,:]),numpy.maximum(bounds[1,:],totalbounds[1,:])))
        def totaldensityfn(*args,**kwargs):
            res = None
            for densityfn in densityfns:
                s = densityfn(*args,**kwargs)
                res = res+s if res is not None else s
            return res
        def totalsamplerfn(*args,**kwargs):
            return samplerfns[random.randint(len(samplerfns))](*args,**kwargs)
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
                    self.robot.SetDOFValues(*jointstate)
                    if updateenv:
                        self.env.UpdatePublishedBodies()
                    # before recording failure, validate that base is not in collision
                    if not gmodel.manip.CheckIndependentCollision(CollisionReport()):
                        grasp = gmodel.grasps[graspindex[1]]
                        gmodel.setPreshape(grasp)
                        q = gmodel.manip.FindIKSolution(gmodel.getGlobalGraspTransform(grasp,collisionfree=True),filteroptions=IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[gmodel.manip.GetArmIndices()] = q
                            goals.append((grasp,pose,values))
                        elif gmodel.manip.FindIKSolution(gmodel.getGlobalGraspTransform(grasp,collisionfree=True),0) is None:
                            numfailures += 1
        return goals,numfailures
    def sampleValidPlacementIterator(self,giveuptime=inf,randomgrasps=False,updateenv=False,randomplacement=False,**kwargs):
        """continues to sample valid goal placements. Returns the robot base position, configuration, and the target grasp from gmodel. Environment should be locked, robot state is saved"""
        starttime = time.time()
        origjointvalues = self.robot.GetDOFValues()
        statesaver = self.robot.CreateRobotStateSaver()
        try:
            baseIterators = []
            for irmodel,gmodel in self.irgmodels:
                def graspiter():
                    for grasp,graspindex in gmodel.validGraspIterator(checkik=False,randomgrasps=randomgrasps,backupdist=0.01):
                        yield gmodel.getGlobalGraspTransform(grasp,collisionfree=True),(gmodel,graspindex)
                baseIterator = irmodel.randomBaseDistributionIterator if randomplacement else irmodel.sampleBaseDistributionIterator
                baseIterators.append((baseIterator(Tgrasps=graspiter(),**kwargs),irmodel,gmodel))
            if len(baseIterators)==0:
                raise planning_error('could not find any valid iterators...')
            while True:
                iterindex = random.randint(len(baseIterators))
                baseIterator,irmodel,gmodel = baseIterators[iterindex]
                try:                    
                    pose,graspindex,jointstate = baseIterator.next()
                except planning_error:
                    baseIterators.pop(iterindex)
                    continue
                self.robot.SetTransform(pose)
                self.robot.SetDOFValues(origjointvalues)
                if not self.env.CheckCollision(self.robot,CollisionReport()):
                    self.robot.SetDOFValues(*jointstate)
                    if updateenv:
                        self.env.UpdatePublishedBodies()
                    #irmodel.manip.CheckIndependentCollision(CollisionReport()):
                    gmodel = graspindex[0]
                    grasp = gmodel.grasps[graspindex[1]]
                    gmodel.setPreshape(grasp)
                    q = irmodel.manip.FindIKSolution(gmodel.getGlobalGraspTransform(grasp,collisionfree=True),filteroptions=IkFilterOptions.CheckEnvCollisions)
                    if q is not None:
                        values = self.robot.GetDOFValues()
                        values[irmodel.manip.GetArmIndices()] = q
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
    def testSampling(self,delay=0,**kwargs):
        """random sample goals on the current environment"""
        with self.env:
            configsampler = self.sampleValidPlacementIterator(**kwargs)
            basemanip = BaseManipulation(self.robot,maxvelmult=self.maxvelmult)
            taskmanip = TaskManipulation(self.robot,maxvelmult=self.maxvelmult)
            jointvalues = self.robot.GetDOFValues()
        with RobotStateSaver(self.robot):
            while True:
                try:
                    with self.env:
                        self.robot.SetDOFValues(jointvalues) # reset to original
                        pose,values,grasp,graspindex = configsampler.next()
                        gmodel = graspindex[0]
                        print 'found grasp',gmodel.target.GetName(),graspindex[1]
                        self.robot.SetActiveManipulator(gmodel.manip)
                        self.robot.SetTransform(pose)
                        gmodel.setPreshape(grasp)
                        self.robot.SetDOFValues(values)
                        final,traj = taskmanip.CloseFingers(execute=False,outputfinal=True)
                        self.robot.SetDOFValues(final,gmodel.manip.GetGripperIndices())
                        self.env.UpdatePublishedBodies()
                    if delay > 0:
                        time.sleep(delay)
                except planning_error, e:
                    traceback.print_exc(e)
                    continue
    def getGraspables(self,manip=None,loadik=True):
        print 'searching for graspable objects (robot=%s)...'%(self.robot.GetRobotStructureHash())
        graspables = []
        with self.env:
            manips = self.robot.GetManipulators() if manip is None else [manip]
            for m in manips:
                self.robot.SetActiveManipulator(m)
                for target in self.env.GetBodies():
                    if not target.IsRobot():
                        gmodel = grasping.GraspingModel(robot=self.robot,target=target,maxvelmult=self.maxvelmult)
                        if gmodel.load():
                            print '%s is graspable by %s'%(target.GetName(),m.GetName())
                            ikmodel = inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
                            if not ikmodel.load():
                                ikmodel.autogenerate()
                            graspables.append(gmodel)
        return graspables
    def setGraspReachability(self,possiblemanips=None):
        targets = []
        self.irgmodels = []
        with self.env:
            for gmodel in self.getGraspables():
                if possiblemanips is not None:
                    if not any([possiblemanip==gmodel.manip for possiblemanip in possiblemanips]):
                        continue
                for irmodel in self.irmodels:
                    if irmodel.manip == gmodel.manip:
                        self.irgmodels.append([irmodel,gmodel])
                        if not gmodel.target in targets:
                            targets.append(gmodel.target)
        return targets

class MobileManipulationPlanning(metaclass.AutoReloader):
    def __init__(self,robot,grmodel=None,switchpatterns=None,maxvelmult=None):
        self.env=robot.GetEnv()
        self.envreal = None
        self.robot = robot
        self.grmodel = grmodel
        self.maxvelmult=maxvelmult
        self.basemanip = BaseManipulation(self.robot,maxvelmult=maxvelmult)
        self.taskmanip = TaskManipulation(self.robot,maxvelmult=maxvelmult)
        self.switchpatterns=switchpatterns
        if switchpatterns is not None:
            self.taskmanip.SwitchModels(switchpatterns=switchpatterns)
        self.updir = array((0,0,1))
    def clone(self,envother):
        clone = shallowcopy(self)
        clone.env = envother
        clone.envreal = self.env
        clone.robot = clone.env.GetRobot(self.robot.GetName())
        clone.grmodel = self.grmodel.clone(envother) if self.grmodel is not None else None
        clone.basemanip = self.basemanip.clone(envother)
        clone.taskmanip = self.taskmanip.clone(envother)
        if clone.switchpatterns is not None:
            clone.taskmanip.SwitchModels(switchpatterns=clone.switchpatterns)
        return clone
    def waitrobot(self):
        """busy wait for robot completion"""
        while not self.robot.GetController().IsDone():
            time.sleep(0.01)
        time.sleep(0.1)
    def graspObjectWithModels(self,allgmodels,usevisibilitycamera=None,neutraljointvalues=None):
        viewmanip = None
        if usevisibilitycamera:
            # filter gmodel to be the same as the camera
            gmodels = None
            target = None
            for testgmodel in allgmodels:
                try:
                    target,viewmanip = self.viewTarget(usevisibilitycamera,testgmodel.target)
                    gmodels = [gmodel for gmodel in allgmodels if gmodel.target==target]
                    order = argsort([gmodel.manip.GetEndEffector()!=viewmanip.GetEndEffector() for gmodel in gmodels])
                    gmodels = [gmodels[i] for i in order]
                    break
                except (RuntimeError,planning_error):
                    print 'failed to view ',testgmodel.target.GetName()
        else:
            gmodels = allgmodels

        if gmodels is None:
            raise planning_error('graspObjectWithModels')
        approachoffset = 0.03
        stepsize = 0.001
        graspiterators = [(gmodel,gmodel.validGraspIterator(checkik=True,randomgrasps=False,backupdist=approachoffset)) for gmodel in gmodels]
        while(len(graspiterators)>0):
            index=random.randint(len(graspiterators))
            try:
                with self.env:
                    grasp,graspindex=graspiterators[index][1].next()
            except StopIteration:
                graspiterators.pop(index)
                continue
            gmodel = graspiterators[index][0]
            if viewmanip is not None and neutraljointvalues is not None and gmodel.manip.GetEndEffector()!=viewmanip.GetEndEffector():
                try:
                    self.moveToNeutral(neutraljointvalues=neutraljointvalues,bounds=array(((-0.1,-0.1,-0.1),(0.1,0.1,0.1))))
                    neutraljointvalues=None
                except planning_error,e:
                    print 'failed to move to neutral values:',e
            print 'selected %s grasp %d '%(gmodel.manip,graspindex)
            gmodel.moveToPreshape(grasp)
            time.sleep(1.5)
            self.waitrobot()
            with self.env:
                self.robot.SetActiveManipulator(gmodel.manip)
                Tgrasp=gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
                finalarmsolution = gmodel.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                if finalarmsolution is None:
                    print 'final grasp has no valid IK'
                    continue
                armjoints=gmodel.manip.GetArmIndices()
                Tfirstgrasp = array(Tgrasp)
                Tfirstgrasp[0:3,3] -= approachoffset*gmodel.getGlobalApproachDir(grasp)
                solutions = gmodel.manip.FindIKSolutions(Tfirstgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                if solutions is None or len(solutions) == 0:
                    continue
                # find the closest solution
                weights = self.robot.GetDOFWeights()[armjoints]
                dists = [numpy.max(abs(finalarmsolution-s)) for s in solutions]
                index = argmin(dists)
                usejacobian = False
                if dists[index] < 15.0*approachoffset:
                    usejacobian = True
                    self.robot.SetActiveDOFs(armjoints)
                    print 'moving to initial ',solutions[index]
                    try:
                        with TaskManipulation.SwitchState(self.taskmanip):
                            self.basemanip.MoveActiveJoints(goal=solutions[index],maxiter=6000,maxtries=2)
                    except planning_error,e:
                        usejacobian=False
                else:
                    print 'closest solution is too far',dists[index]
            self.waitrobot()
            if usejacobian:
                try:
                    print 'moving hand'
                    expectedsteps = floor(approachoffset/stepsize)
                    self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp),ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps-2,maxsteps=expectedsteps+3,searchall=False)
                except planning_error,e:
#                     try:
#                         self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp),ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps-2,maxsteps=expectedsteps+1,searchall=True)
#                     except planning_error,e:
                    print 'failed to move straight: ',e,' Using planning to move rest of the way.'
                    usejacobian = False
            if not usejacobian:
                print 'moving to final ',solutions[index]
                with self.env:
                    self.robot.SetActiveDOFs(armjoints)
                    self.basemanip.MoveActiveJoints(goal=finalarmsolution,maxiter=5000,maxtries=2)
            self.waitrobot()
            success = False
            try:
                self.closefingers(target=gmodel.target)
                success = True
            except planning_error,e:
                print e
            try:
                self.basemanip.MoveHandStraight(direction=self.updir,stepsize=0.002,minsteps=1,maxsteps=100)
            except:
                print 'failed to move up'
            self.waitrobot()
            if not success:
                continue
            return gmodel
        raise planning_error('failed to grasp')
    def graspObject(self,allgmodels,usevisibilitycamera=None,neutraljointvalues=None):
        viewmanip = None
        if usevisibilitycamera:
            # filter gmodel to be the same as the camera
            gmodels = None
            target = None
            for testgmodel in allgmodels:
                try:
                    target,viewmanip = self.viewTarget(usevisibilitycamera,testgmodel.target)
                    gmodels = [gmodel for gmodel in allgmodels if gmodel.target==target]
                    order = argsort([gmodel.manip.GetEndEffector()!=viewmanip.GetEndEffector() for gmodel in gmodels])
                    gmodels = [gmodels[i] for i in order]
                    break
                except (RuntimeError,planning_error):
                    print 'failed to view ',testgmodel.target.GetName()
        else:
            gmodels = allgmodels
        print 'graspplanning grasp and place object'
        for gmodel in gmodels:
            try:
                if viewmanip is not None and neutraljointvalues is not None and gmodel.manip.GetEndEffector()!=viewmanip.GetEndEffector():
                    try:
                        self.moveToNeutral(neutraljointvalues=neutraljointvalues,bounds=array(((-0.1,-0.1,-0.1),(0.1,0.1,0.1))))
                        neutraljointvalues=None
                    except planning_error,e:
                        print 'failed to move to neutral values:',e
                self.robot.SetActiveManipulator(gmodel.manip)
                approachoffset = 0.03
                stepsize = 0.001
                goals,graspindex,searchtime,trajdata = self.taskmanip.GraspPlanning(graspindices=gmodel.graspindices,grasps=gmodel.grasps,
                                                                                    target=gmodel.target,approachoffset=approachoffset,destposes=None,
                                                                                    seedgrasps = 3,seedik=3,maxiter=2000,randomgrasps=False)
            except planning_error,e:
                print 'failed to grasp with %s'%(gmodel.manip),e
                continue
            self.waitrobot()
            usejacobian = False
            armjoints = gmodel.manip.GetArmIndices()
            self.robot.SetActiveDOFs(armjoints)
            grasp=gmodel.grasps[graspindex]
            with self.env:
                # although values holds the final configuration, robot should approach from a distance of approachoffset
                self.robot.SetActiveManipulator(gmodel.manip)
                Tgrasp = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
                finalarmsolution = gmodel.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                if finalarmsolution is None:
                    print 'final grasp has no valid IK'
                    continue
                # find the closest solution
                curjointvalues = self.robot.GetDOFValues(armjoints)
                weights = self.robot.GetDOFWeights()[armjoints]
                dist = numpy.max(abs(finalarmsolution-curjointvalues))
                if dist < 15.0*approachoffset:
                    usejacobian = True

            if usejacobian:
                try:
                    print 'moving hand'
                    expectedsteps = floor(approachoffset/stepsize)
                    self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp),ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps-2,maxsteps=expectedsteps+1,searchall=False)
                except planning_error,e:
                    try:
                        self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp),ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps-2,maxsteps=expectedsteps+1,searchall=True)
                    except planning_error,e:
                        print 'failed to move straight: ',e,' Using planning to move rest of the way.'
                        usejacobian = False
            self.waitrobot()
            if not usejacobian:
                try:
                    with TaskManipulation.SwitchState(self.taskmanip):
                        print 'moving active joints'
                        self.basemanip.MoveActiveJoints(goal=finalarmsolution)
                except planning_error,e:
                    print e
                    continue
            self.waitrobot()
            self.closefingers(target=gmodel.target)
            try:
                self.basemanip.MoveHandStraight(direction=self.updir,jacobian=0.02,stepsize=0.002,minsteps=1,maxsteps=100)
            except:
                print 'failed to move up'
            self.waitrobot()
            return gmodel
        raise planning_error('failed to grasp target')

    def moveToNeutral(self,neutraljointvalues,bounds=None,manipnames=None,ikcollisionbody=None):
        """moves the robot to a neutral position defined by several manipulators and neutraljointvalues. Can also specify a special collision body to constraint choosing the goals."""
        if manipnames is None:
            manipnames = ['leftarm','rightarm']
        manips = [self.robot.GetManipulator(name) for name in manipnames]
        ikmodels = []
        for manip in manips:
            self.robot.SetActiveManipulator(manip)
            ikmodel = inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            ikmodels.append(ikmodel)
        goaljointvalues = None
        alltrajdata = None
        with self.robot:
            origjointvalues=self.robot.GetDOFValues()
            self.robot.SetDOFValues(neutraljointvalues)
            if ikcollisionbody is not None:
                ikcollisionbody.Enable(True)
            nocollision = not self.env.CheckCollision(self.robot) and not self.robot.CheckSelfCollision()
            if ikcollisionbody is not None:
                ikcollisionbody.Enable(False)
            if nocollision:
                goaljointvalues = neutraljointvalues
                try:
                    alltrajdata = []
                    self.robot.SetDOFValues(origjointvalues)
                    for manip in manips:
                        self.robot.SetActiveDOFs(manip.GetArmIndices())
                        alltrajdata.append(self.basemanip.MoveActiveJoints(goal=goaljointvalues[manip.GetArmIndices()],execute=False,outputtraj=True))
                        self.robot.SetDOFValues(goaljointvalues[manip.GetArmIndices()],manip.GetArmIndices())
                except planning_error:
                    alltrajdata = None
            if alltrajdata is None:
                self.robot.SetDOFValues(neutraljointvalues)
                Tmanips = [dot(linalg.inv(manip.GetBase().GetTransform()),manip.GetEndEffectorTransform()) for manip in manips]
                for niter in range(500):
                    self.robot.SetDOFValues(origjointvalues)
                    if bounds is None:
                        r = random.rand(3)*0.4-0.2
                    else:
                        r = bounds[0,:]+random.rand(3)*(bounds[1,:]-bounds[0,:])
                    success = True
                    startjointvalues=[]
                    for manip,Tmanip in izip(manips,Tmanips):
                        T = dot(manip.GetBase().GetTransform(),Tmanip)
                        if niter > 0:
                            T[0:3,3] += r
                            if niter > 200:
                                T[0:3,0:3] = dot(T[0:3,0:3],rotationMatrixFromAxisAngle(random.rand(3)*0.5))
                        s=None
                        try:
                            if ikcollisionbody is not None:
                                ikcollisionbody.Enable(True)
                                for link in manip.GetIndependentLinks():
                                    link.Enable(False)
                            s = manip.FindIKSolution(T,filteroptions=IkFilterOptions.CheckEnvCollisions)
                        finally:
                            if ikcollisionbody is not None:
                                ikcollisionbody.Enable(False)
                                for link in manip.GetIndependentLinks():
                                    link.Enable(True)
                        if s is None:
                            success = False
                            break
                        else:
                            if ikcollisionbody is not None:
                                # have to check self collision
                                with self.robot:
                                    self.robot.SetDOFValues(s,manip.GetArmIndices())
                                    if self.robot.CheckSelfCollision():
                                        success = False
                                        break
                            startjointvalues.append(self.robot.GetDOFValues())
                            self.robot.SetDOFValues(s,manip.GetArmIndices())
                    if not success:
                        continue
                    try:
                        print 'attempting to plan...'
                        goaljointvalues = self.robot.GetDOFValues()
                        alltrajdata = []
                        for jointvalues,manip in izip(startjointvalues,manips):
                            self.robot.SetDOFValues(jointvalues)
                            self.robot.SetActiveDOFs(manip.GetArmIndices())
                            alltrajdata.append(self.basemanip.MoveActiveJoints(goal=goaljointvalues[manip.GetArmIndices()],execute=False,outputtraj=True))
                        break
                    except planning_error:
                        alltrajdata = None
                if alltrajdata is None:
                    raise planning_error('failed to find collision free position')
        for trajdata in alltrajdata:
            self.basemanip.TrajFromData(trajdata)
            self.waitrobot()

    def searchrealenv(self,target=None,updateenv=False,waitfortarget=0):
        """sync with the real environment and find the target"""
        if self.envreal is None:
            return target,self.env
        if updateenv:
            print 'updating entire environment not supported yet'
        
        starttime = time.time()
        while True:
            with self.envreal:
                # find target of the same name/type
                try:
                    b = self.envreal.GetKinBody(target.GetName())
                    if b:
                        target.SetBodyTransformations(b.GetBodyTransformations())
                        return target,self.env
                except:
                    pass
                bodies = [b for b in self.envreal.GetBodies() if b.GetXMLFilename()==target.GetXMLFilename()]
                if len(bodies) > 0:
                    # find the closest
                    dists = [sum((b.GetTransform()[0:3,3]-target.GetTransform()[0:3,3])**2) for b in bodies]
                    index = argmin(dists)
                    if dists[index] < 1:
                        print 'distance to original: ',dists[index]
                        target.SetBodyTransformations(bodies[index].GetBodyTransformations())
                        return target, self.env
            if time.time()-starttime > waitfortarget:
                break
            time.sleep(0.05)
        raise planning_error('failed to recognize target')

    def graspAndPlaceObjectMobileSearch(self,targetdests):
        assert(len(targetdests)>0)
        weight = 2.0
        logllthresh = 0.5
        origjointvalues = self.robot.GetDOFValues()
        configsampler = self.grmodel.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=False,randomplacement=False,updateenv=False)
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
                        self.robot.SetTransform(pose)
                        self.robot.SetDOFValues(values)
                        gmodel.setPreshape(grasp)
                        for T in dests[0]:
                            Tnewgrasp = dot(T,Trelative)
                            if gmodel.manip.FindIKSolution(Tnewgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions) is not None:
                                Ttarget = T
                                break
                    if Ttarget is not None:
                        break
                finally:
                    gmodel.target.Enable(True)
        self.graspObjectMobile(pose,values,grasp,graspindex)
        # if everything finished correctly, put object in destination        
        self.basemanip.MoveToHandPosition([Tnewgrasp],seedik=10,maxiter=5000)
        self.waitrobot()
        self.basemanip.ReleaseFingers(target=gmodel.target)
        self.waitrobot()
#         with self.env:
#             self.robot.SetActiveDOFs(gmodel.manip.GetArmIndices())
#             try:
#                 self.basemanip.MoveActiveJoints(goal=origjointvalues[gmodel.manip.GetArmIndices()],maxiter=3000)
#             except:
#                 print 'failed to move arm closer'

        self.waitrobot()
    def graspObjectMobileSearch(self,**kwargs):
        weight = 2.0
        logllthresh = 0.5
        origjointvalues = self.robot.GetDOFValues()
        configsampler = self.grmodel.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=False,randomplacement=False,updateenv=False)
        with self.env:
            starttime=time.time()
            pose,values,grasp,graspindex = configsampler.next()
            print 'found in %fs'%(time.time()-starttime)
        gmodel = self.graspObjectMobile(pose,values,grasp,graspindex,**kwargs)
#         # move to original position
#         with self.env:
#             self.robot.SetActiveDOFs(gmodel.manip.GetArmIndices())
#             try:
#                 self.basemanip.MoveActiveJoints(goal=origjointvalues[gmodel.manip.GetArmIndices()],maxiter=3000)
#             except:
#                 print 'failed to move arm closer'
#         self.waitrobot()
        return gmodel
    def closefingers(self,manip=None,target=None):
        """close fingers and grab body"""
        with self.env:
            if manip:
                self.robot.SetActiveManipulator(manip)
            self.taskmanip.CloseFingers()
        self.waitrobot()
        with self.env:
            expectedvalues = self.robot.GetDOFValues(self.robot.GetActiveManipulator().GetGripperIndices())
        if target is not None:
            with TaskManipulation.SwitchState(self.taskmanip): # grab the fat object!
                with self.env:
                    self.robot.Grab(target)
            if self.envreal is not None: # try to grab with the real environment
                try:
                    with self.envreal:
                        robotreal = self.envreal.GetRobot(self.robot.GetName())
                        if robotreal is not None:
                            # always create a dummy body
                            targetreal = self.envreal.ReadKinBodyXMLFile(target.GetXMLFilename())
                            if target is not None:
                                targetreal.SetName(target.GetName()+str(100+random.randint(10000)))
                                self.envreal.AddKinBody(targetreal)
                                targetreal.SetBodyTransformations(target.GetBodyTransformations())
                                robotreal.SetActiveManipulator(self.robot.GetActiveManipulator().GetName())
                                robotreal.Grab(targetreal)
                            else:
                                print 'failed to create real target with filename ',target.GetXMLFilename()
                except openrave_exception,e:
                    print 'closefingers:',e
        return expectedvalues
    def releasefingers(self,manip=None):
        """open fingers and release body"""
        with self.env:
            if manip:
                self.robot.SetActiveManipulator(manip)
            # find the grabbing body
            grabbed = [grabbedbody for grabbedbody in self.robot.GetGrabbed() if self.robot.GetActiveManipulator().IsGrabbing(grabbedbody)]
            self.basemanip.ReleaseFingers(target=grabbed[0] if len(grabbed)>0 else None)
        self.waitrobot()
        if self.envreal is not None:
            try:
                with self.envreal:
                    robotreal = self.envreal.GetRobot(self.robot.GetName())
                    if robotreal is not None:
                        targets = robotreal.GetGrabbed()
                        robotreal.ReleaseAllGrabbed()
#                         for target in targets:
#                             self.envreal.Remove(target)
            except openrave_exception,e:
                print 'releasefingers:',e

    def moveWithFreeArms(self,jointvalues,jointinds,maxtries=3):
        with self.env:
            self.robot.SetActiveDOFs(jointinds)
            self.basemanip.MoveActiveJoints(goal=jointvalues,maxtries=maxtries,maxiter=5000)
        self.waitrobot()

    def graspObjectMobile(self,pose,values,grasp,graspindex,usevisibilitycamera=None):
        approachoffset = 0.03
        stepsize = 0.001
        gmodel = graspindex[0]
        print 'found grasp',gmodel.manip.GetName(),gmodel.target.GetName(),graspindex[1]
        self.robot.SetActiveManipulator(gmodel.manip)
        self.moveBase(pose)
        with self.env:
            armjoints=gmodel.manip.GetArmIndices()
            #finalarmsolution = values[armjoints]
            dofindices = inversereachability.InverseReachabilityModel.getdofindices(gmodel.manip)
        if len(dofindices) > 0:
            print 'moving for reachability: ',values[dofindices],dofindices
            self.moveWithFreeArms(values[dofindices],dofindices,maxtries=2)
        self.robot.SetActiveManipulator(gmodel.manip)

        if usevisibilitycamera:
            newtarget,viewmanip = self.viewTarget(usevisibilitycamera,gmodel.target)

        gmodel.moveToPreshape(grasp)
        time.sleep(1.5)
        self.waitrobot()

        usejacobian = False
        with self.env:
            # although values holds the final configuration, robot should approach from a distance of approachoffset
            Tgrasp = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
            finalarmsolution = gmodel.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
            if finalarmsolution is None:
                raise planning_error('final grasp has no valid IK')
            Tfirstgrasp = array(Tgrasp)
            Tfirstgrasp[0:3,3] -= approachoffset*gmodel.getGlobalApproachDir(grasp)
            solutions = gmodel.manip.FindIKSolutions(Tfirstgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
            if solutions is None or len(solutions) == 0:
                return self.graspObject([gmodel])
            # find the closest solution
            weights = self.robot.GetDOFWeights()[armjoints]
            dists = [numpy.max(abs(finalarmsolution-s)) for s in solutions]
            index = argmin(dists)
            if dists[index] < 15.0*approachoffset:
                usejacobian = True
                with TaskManipulation.SwitchState(self.taskmanip):
                    self.robot.SetActiveDOFs(armjoints)
                    self.basemanip.MoveActiveJoints(goal=solutions[index],maxiter=5000,maxtries=2)
            else:
                print 'closest solution is too far',dists[index],15.0*approachoffset
        self.waitrobot()

        if usejacobian:
            try:
                print 'moving hand'
                expectedsteps = floor(approachoffset/stepsize)
                self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp),ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps-2,maxsteps=expectedsteps+1,searchall=False)
            except:
                print 'failed to move straight, using planning to move rest of the way'
                usejacobian = False

        if not usejacobian:
            with self.env:
                self.robot.SetActiveDOFs(armjoints)
                self.basemanip.MoveActiveJoints(goal=finalarmsolution,maxiter=5000,maxtries=2)
        self.waitrobot()
        success = False
        try:
            self.closefingers(target=gmodel.target)
            success = True
        except planning_error,e:
            print e
        try:
            self.basemanip.MoveHandStraight(direction=self.updir,stepsize=0.002,minsteps=1,maxsteps=100)
        except:
            print 'failed to move up'
        self.waitrobot()
        if not success:
            raise planning_error('failed to close fingers')
        return gmodel

    def placeObjectMobileSearch(self,gmodel,dests,goalbounds,numtries=10,nummaxdests=10):
        assert(len(dests)>0)
        trajdata=None
        pose = None
        self.robot.SetActiveManipulator(gmodel.manip)
        with self.robot:
            dbounds = goalbounds[1,:]-goalbounds[0,:]
            Trobot = self.robot.GetTransform()
            Trelative = dot(linalg.inv(gmodel.target.GetTransform()),gmodel.manip.GetEndEffectorTransform())
            for iter in range(numtries):
                r = random.rand(3)
                if iter > 0:
                    angle = 0.5*(goalbounds[0,0]+r[0]*dbounds[0])
                    pose = r_[cos(angle),0,0,sin(angle),goalbounds[0,1:3]+r[1:3]*dbounds[1:3],Trobot[2,3]]
                else:
                    pose = poseFromMatrix(Trobot)
                self.robot.SetTransform(pose)
                if self.env.CheckCollision(self.robot):
                    continue
                try:
                    I=random.permutation(range(len(dests)))[0:min(100,len(dests))]
                    trajdata = self.basemanip.MoveToHandPosition(matrices=[dot(dests[ind],Trelative) for ind in I],maxiter=4000,seedik=4,execute=False,outputtraj=True)
                    if trajdata is not None:
                        break
                except planning_error:
                    pass
        if pose is None or trajdata is None:
            raise planning_error()
        self.moveBase(pose)
        self.basemanip.TrajFromData(trajdata)
        self.waitrobot()
        # move hand down
        try: 
            self.basemanip.MoveHandStraight(direction=-self.updir,jacobian=0.02,stepsize=0.002,minsteps=0,maxsteps=100)
            self.waitrobot()
            self.basemanip.MoveHandStraight(direction=-self.updir,jacobian=0.02,ignorefirstcollision=True,stepsize=0.002,minsteps=0,maxsteps=5)
            self.waitrobot()
        except planning_error,e:
            print 'failed to move hand down'
            traceback.print_exc(e)
        self.releasefingers()

    def placeObject(self,target,dests):
        assert(len(dests)>0)
        with self.robot:
            Trelative = dot(linalg.inv(target.GetTransform()),self.robot.GetActiveManipulator().GetEndEffectorTransform())
            I=random.permutation(range(len(dests)))
            num = len(I)/50
            success = False
            for i in range(num):
                try:
                    self.basemanip.MoveToHandPosition(matrices=[dot(dests[ind],Trelative) for ind in I[i::num]],maxiter=4000,seedik=4)
                    success = True
                    break
                except planning_error:
                    pass
            if not success:
                raise planning_error('failed to place object in one of dests')
        self.waitrobot()
        # move hand down
        try: 
            print self.robot.GetActiveManipulator()
            print self.robot.GetDOFValues()
            self.basemanip.MoveHandStraight(direction=-self.updir,jacobian=0.02,stepsize=0.002,minsteps=0,maxsteps=100)
            self.waitrobot()
            self.basemanip.MoveHandStraight(direction=-self.updir,jacobian=0.02,ignorefirstcollision=True,stepsize=0.002,minsteps=0,maxsteps=5)
            self.waitrobot()
        except planning_error,e:
            print 'failed to move hand down'
            traceback.print_exc(e)
        self.releasefingers()
    
    def moveBase(self,pose):
        with self.robot:
            self.robot.SetTransform(pose)
            if self.env.CheckCollision(self.robot):
                raise planning_error('goal position in self collision')
        with self.robot: # find the boundaries of the environment
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
            if self.basemanip.MoveActiveJoints(goal=goal2d,maxiter=3000,steplength=0.1) is None:
                raise planning_error('failed to plan goal position')
        self.waitrobot()
            
    def viewTarget(self,usevisibilitycamera,target):
        print 'attempting visibility to ',target.GetName(),' planning with ',usevisibilitycamera['sensorname']
        vmodel = visibilitymodel.VisibilityModel(robot=self.robot,target=target,sensorname=usevisibilitycamera['sensorname'],maxvelmult=self.maxvelmult)
        if not vmodel.load():
            raise planning_error('failed to load visibility model')
        #pts = array([dot(vmodel.target.GetTransform(),matrixFromPose(pose))[0:3,3] for pose in vmodel.visibilitytransforms])
        #h=vmodel.env.plot3(pts,10,colors=array([1,0.7,0,0.05]))
        reachabletransforms = vmodel.pruneTransformations(thresh=0.04,numminneighs=40,maxdist=usevisibilitycamera.get('maxdist',0.25))
        vmodel.SetCameraTransforms(reachabletransforms)
        for iter in range(20):
            try:
                vmodel.visualprob.MoveToObserveTarget(sampleprob=0.001,maxiter=4000)
            except RuntimeError,e:
                print e
                raise planning_error('cannot find target')
            except planning_error,e:
                print e
                continue
            #s=vmodel.visualprob.SampleVisibilityGoal(target)
            self.waitrobot()
            if usevisibilitycamera.get('dosync',False):
                if usevisibilitycamera.get('ask',False):
                    cmd=raw_input('press n key to capture new environment: ')
                    if cmd == 'n':
                        continue
                with self.env:
                    time.sleep(usevisibilitycamera.get('syncdelay',1.0))
                    try:
                        newtarget,newenv = self.searchrealenv(target,waitfortarget=4.0)
                        if newtarget is not None:
                            break
                    except planning_error, e:
                        print e
                        continue
            else:
                newtarget = target
                break
        if newtarget is None:
            raise planning_error('cannot find target')
        if usevisibilitycamera.get('storeimage',False):
            usevisibilitycamera['image']= vmodel.getCameraImage()
        print repr(self.robot.GetDOFValues())
        print repr(self.robot.GetTransform())
        return target,vmodel.manip
def run():
    """Executes the mobilemanipulation example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description="mobile manipulation")
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    try:
        env.Load('data/lab1.env.xml')
        robot = env.GetRobots()[0]
        env.UpdatePublishedBodies()
        time.sleep(0.1) # give time for environment to update
        self = MobileManipulationPlanning(robot,randomize=True)
        self.performGraspPlanning()
    finally:
        env.Destroy()
        RaveDestroy()

if __name__ == "__main__":
    #run()
    print 'not implemented yet'
