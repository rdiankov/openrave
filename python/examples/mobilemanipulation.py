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
from copy import copy as shallowcopy
from openravepy import *
from openravepy.interfaces import BaseManipulation, TaskManipulation, VisualFeedback
from openravepy.examples import inversekinematics,inversereachability,graspplanning,grasping,visibilitymodel
from numpy import *
import numpy
from itertools import izip
# try:
#     import scipy # used for showing images
# except ImportError:
#     pass

class GraspReachability(metaclass.AutoReloader):
    def __init__(self,robot,target=None,irmodels=None,irgmodels=None):
        self.robot = robot
        self.env = robot.GetEnv()
        self.irmodels=irmodels
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
    def clone(self,envother):
        clone = shallowcopy(self)
        clone.env = envother
        clone.robot = clone.env.GetRobot(self.robot.GetName())
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
    def testSampling(self,delay=0,**kwargs):
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
                    if delay > 0:
                        time.sleep(delay)
                except planning_error, e:
                    traceback.print_exc(e)
                    continue

class MobileManipulationPlanning(metaclass.AutoReloader):
    def __init__(self,robot,grmodel=None,switchpatterns=None,maxvelmult=None):
        self.env=robot.GetEnv()
        self.envreal = None
        self.robot = robot
        self.grmodel = grmodel
        self.switchpatterns = switchpatterns
        self.basemanip = BaseManipulation(self.robot,maxvelmult=maxvelmult)
        self.taskmanip = TaskManipulation(self.robot,maxvelmult=maxvelmult)
        self.updir = array((0,0,1))
    def clone(self,envother):
        clone = shallowcopy(self)
        clone.env = envother
        clone.envreal = self.env
        clone.robot = clone.env.GetRobot(self.robot.GetName())
        clone.grmodel = self.grmodel.clone(envother)
        clone.basemanip = self.basemanip.clone(envother)
        clone.taskmanip = self.taskmanip.clone(envother)
        return clone
    def waitrobot(self):
        """busy wait for robot completion"""
        while not self.robot.GetController().IsDone():
            time.sleep(0.01)
    def getGraspables(self,manip=None):
        print 'searching for graspable objects (robot=%s)...'%(self.robot.GetRobotStructureHash())
        graspables = []
        with self.env:
            manips = self.robot.GetManipulators() if manip is None else [manip]
            for m in manips:
                self.robot.SetActiveManipulator(m)
                for target in self.env.GetBodies():
                    if not target.IsRobot():
                        gmodel = grasping.GraspingModel(robot=self.robot,target=target)
                        if gmodel.load():
                            print '%s is graspable'%target.GetName()
                            graspables.append(gmodel)
        return graspables
    def graspObject(self,allgmodels,usevisibilitycamera=None):
        if usevisibilitycamera:
            # filter gmodel to be the same as the camera
            gmodels = None
            target = None
            for testgmodel in allgmodels:
                try:
                    target,viewmanip = self.viewTarget(usevisibilitycamera,testgmodel.target)
                    gmodels = [gmodel for gmodel in allgmodels if gmodel.target==testgmodel.target and gmodel.manip.GetEndEffector()==viewmanip.GetEndEffector()]
                    break
                except planning_error:
                    pass
        else:
            target = allgmodels[0].target
            gmodels = [gmodel for gmodel in allgmodels if gmodel.target==target]
        if target is None:
            raise planning_error('no graspable target')
        print 'graspplanning grasp and place object',target.GetName()
        for gmodel in gmodels:
            try:
                self.robot.SetActiveManipulator(gmodel.manip)
                approachoffset = 0.02
                stepsize = 0.001
                goals,graspindex,searchtime,trajdata = self.taskmanip.GraspPlanning(graspindices=gmodel.graspindices,grasps=gmodel.grasps,
                                                                                    target=target,approachoffset=approachoffset,destposes=None,
                                                                                    seedgrasps = 3,seedik=3,maxiter=2000,
                                                                                    randomgrasps=True,switchpatterns=self.switchpatterns)
            except planning_error:
                continue
            self.waitrobot()
            usejacobian = False
            armjoints = gmodel.manip.GetArmJoints()
            self.robot.SetActiveDOFs(armjoints)
            grasp=gmodel.grasps[graspindex]
            with self.env:
                # although values holds the final configuration, robot should approach from a distance of approachoffset
                Tgrasp = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
                finalarmsolution = gmodel.manip.FindIKSolution(Tgrasp,True)
                if finalarmsolution is None:
                    print 'final grasp has no valid IK'
                    continue
                # find the closest solution
                curjointvalues = self.robot.GetJointValues()[armjoints]
                weights = self.robot.GetJointWeights()[armjoints]
                dist = numpy.max(abs(finalarmsolution-curjointvalues))
                if dist < 15.0*approachoffset:
                    usejacobian = True

            if usejacobian:
                try:
                    print 'moving hand'
                    expectedsteps = floor(approachoffset/stepsize)
                    self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp),ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps-2,maxsteps=expectedsteps+1,searchall=True)
                except planning_error,e:
                    print 'failed to move straight: ',e,' Using planning to move rest of the way.'
                    usejacobian = False

            if not usejacobian:
                try:
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

    def moveToNeutral(self,neutraljointvalues,bounds=None,manipnames=None):
        """moves the robot to a neutral position defined by several manipulators and neutraljointvalues"""
        if manipnames is None:
            manipnames = ['leftarm','rightarm']
        manips = [self.robot.GetManipulators(name)[0] for name in manipnames]
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
            origjointvalues=self.robot.GetJointValues()
            self.robot.SetJointValues(neutraljointvalues)
            if not self.env.CheckCollision(self.robot) and not self.robot.CheckSelfCollision():
                goaljointvalues = neutraljointvalues
                try:
                    alltrajdata = []
                    self.robot.SetJointValues(origjointvalues)
                    for manip in manips:
                        self.robot.SetActiveDOFs(manip.GetArmJoints())
                        alltrajdata.append(self.basemanip.MoveActiveJoints(goal=goaljointvalues[manip.GetArmJoints()],execute=False,outputtraj=True))
                        self.robot.SetJointValues(goaljointvalues[manip.GetArmJoints()],manip.GetArmJoints())
                except planning_error:
                    alltrajdata = None
            if alltrajdata is None:
                self.robot.SetJointValues(origjointvalues)
                Tmanips = [manip.GetEndEffectorTransform() for manip in manips]
                for iter in range(500):
                    self.robot.SetJointValues(origjointvalues)
                    if bounds is None:
                        r = random.rand(3)*0.4-0.2
                    else:
                        r = bounds[0,:]+random.rand(3)*(bounds[1,:]-bounds[0,:])
                    success = True
                    startjointvalues=[]
                    for manip,Tmanip in izip(manips,Tmanips):
                        T = array(Tmanip)
                        T[0:3,3] += r
                        s = manip.FindIKSolution(T,True)
                        if s is None:
                            success = False
                            break
                        else:
                            startjointvalues.append(self.robot.GetJointValues())
                            self.robot.SetJointValues(s,manip.GetArmJoints())
                    if not success:
                        continue
                    try:
                        print 'attempting to plan...'
                        goaljointvalues = self.robot.GetJointValues()
                        alltrajdata = []
                        for jointvalues,manip in izip(startjointvalues,manips):
                            self.robot.SetJointValues(jointvalues)
                            self.robot.SetActiveDOFs(manip.GetArmJoints())
                            alltrajdata.append(self.basemanip.MoveActiveJoints(goal=goaljointvalues[manip.GetArmJoints()],execute=False,outputtraj=True))
                        break
                    except planning_error:
                        alltrajdata = None
                if alltrajdata is None:
                    raise planning_error('failed to find collision free position')
        for trajdata in alltrajdata:
            self.basemanip.TrajFromData(trajdata)
            self.waitrobot()

    def searchrealenv(self,target=None,updateenv=False,waitfortarget=True):
        """sync with the real environment and find the target"""
        if self.envreal is None:
            return target,self.env
        if updateenv:
            print 'updating entire environment not supported yet'
        
        while True:
            with self.envreal:
                # find target of the same name/type
                b = self.envreal.GetKinBody(target.GetName())
                if b:
                    target.SetBodyTransformations(b.GetBodyTransformations())
                    return target,self.env
                bodies = [b for b in self.envreal.GetBodies() if b.GetXMLFilename()==target.GetXMLFilename()]
                if len(bodies) > 0:
                    # find the closest
                    dists = [sum((b.GetTransform()[0:3,3]-target.GetTransform()[0:3,3])**2) for b in bodies]
                    index = argmin(dists)
                    print 'distance to original: ',dists[index]
                    target.SetBodyTransformations(bodies[index].GetBodyTransformations())
                    return target, self.env
            if not waitfortarget:
                break
            time.sleep(1)
        raise planning_error('failed to recognize target')

    def graspAndPlaceObjectMobileSearch(self,targetdests):
        assert(len(targetdests)>0)
        weight = 2.0
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
                        self.robot.SetTransform(pose)
                        self.robot.SetJointValues(values)
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
        self.basemanip.MoveToHandPosition([Tnewgrasp],seedik=10,maxiter=5000)
        self.waitrobot()
        self.basemanip.ReleaseFingers(target=gmodel.target)
        self.waitrobot()
#         with self.env:
#             self.robot.SetActiveDOFs(gmodel.manip.GetArmJoints())
#             try:
#                 self.basemanip.MoveActiveJoints(goal=origjointvalues[gmodel.manip.GetArmJoints()],maxiter=3000)
#             except:
#                 print 'failed to move arm closer'

        self.waitrobot()
    def graspObjectMobileSearch(self,**kwargs):
        weight = 2.0
        logllthresh = 0.5
        origjointvalues = self.robot.GetJointValues()
        configsampler = self.grmodel.sampleValidPlacementIterator(weight=weight,logllthresh=logllthresh,randomgrasps=True,randomplacement=False,updateenv=False)
        with self.env:
            starttime=time.time()
            pose,values,grasp,graspindex = configsampler.next()
            print 'found in %fs'%(time.time()-starttime)
        gmodel = self.graspObjectMobile(pose,values,grasp,graspindex,**kwargs)
#         # move to original position
#         with self.env:
#             self.robot.SetActiveDOFs(gmodel.manip.GetArmJoints())
#             try:
#                 self.basemanip.MoveActiveJoints(goal=origjointvalues[gmodel.manip.GetArmJoints()],maxiter=3000)
#             except:
#                 print 'failed to move arm closer'
#         self.waitrobot()
        return gmodel
    def closefingers(self,manip=None,target=None):
        """close fingers and grab body"""
        with self.env:
            if manip:
                self.robot.SetActiveManipulator(manip)
            self.basemanip.CloseFingers()
        self.waitrobot()
        if target is not None:
            with self.env:
                self.robot.Grab(target)
            if self.envreal is not None: # try to grab with the real environment
                with self.envreal:
                    robotreal = self.envreal.GetRobot(self.robot.GetName())
                    targetreal = self.envreal.GetKinBody(target.GetName())
                    if not robotreal is None and not targetreal is None:
                        robotreal.SetActiveManipulator(self.robot.GetActiveManipulator().GetName())
                        robotreal.Grab(targetreal)
    def releasefingers(self,manip=None):
        """open fingers and release body"""
        with self.env:
            if manip:
                self.robot.SetActiveManipulator(manip)
            # find the grabbing body
            grabbed = [g.grabbedbody for g in self.robot.GetGrabbed() if self.robot.GetActiveManipulator().IsGrabbing(g.grabbedbody)]
            self.basemanip.ReleaseFingers(target=grabbed[0] if len(grabbed)>0 else None)
        self.waitrobot()
        if self.envreal is not None:
            with self.envreal:
                robotreal = self.envreal.GetRobot(self.robot.GetName())
                if robotreal is not None:
                    robotreal.ReleaseAllGrabbed()

    def graspObjectMobile(self,pose,values,grasp,graspindex,usevisibilitycamera=None):
        approachoffset = 0.02
        stepsize = 0.001
        gmodel = graspindex[0]
        print 'found grasp',gmodel.manip.GetName(),gmodel.target.GetName(),graspindex[1]
        self.robot.SetActiveManipulator(gmodel.manip)
        self.moveBase(pose)
        with self.env:
            armjoints=gmodel.manip.GetArmJoints()
            #finalarmsolution = values[armjoints]
            dofindices = inversereachability.InverseReachabilityModel.getdofindices(gmodel.manip)
            if len(dofindices) > 0:
                self.robot.SetActiveDOFs(dofindices)
                self.basemanip.MoveActiveJoints(goal=values[dofindices])
        self.waitrobot()
        gmodel.moveToPreshape(grasp)
        self.robot.SetActiveManipulator(gmodel.manip)

        if usevisibilitycamera:
            newtarget,viewmanip = self.viewTarget(usevisibilitycamera,gmodel.target)

        usejacobian = False
        with self.env:
            # although values holds the final configuration, robot should approach from a distance of approachoffset
            Tgrasp = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
            finalarmsolution = gmodel.manip.FindIKSolution(Tgrasp,True)
            if finalarmsolution is None:
                raise planning_error('final grasp has no valid IK')
            Tfirstgrasp = array(Tgrasp)
            Tfirstgrasp[0:3,3] -= approachoffset*gmodel.getGlobalApproachDir(grasp)
            solutions = gmodel.manip.FindIKSolutions(Tfirstgrasp,True)
            if solutions is None or len(solutions) == 0:
                return self.graspObject(gmodel.target)
            # find the closest solution
            weights = self.robot.GetJointWeights()[armjoints]
            dists = [numpy.max(abs(finalarmsolution-s)) for s in solutions]
            index = argmin(dists)
            if dists[index] < 15.0*approachoffset:
                usejacobian = True
                self.robot.SetActiveDOFs(armjoints)
                self.basemanip.MoveActiveJoints(goal=solutions[index],maxiter=5000)
            else:
                print 'closest solution is too far',dists[index]
        self.waitrobot()

        if usejacobian:
            try:
                print 'moving hand'
                expectedsteps = floor(approachoffset/stepsize)
                self.basemanip.MoveHandStraight(direction=gmodel.getGlobalApproachDir(grasp),ignorefirstcollision=False,stepsize=stepsize,minsteps=expectedsteps-2,maxsteps=expectedsteps+1,searchall=True)
            except:
                print 'failed to move straight, using planning to move rest of the way'
                usejacobian = False

        if not usejacobian:
            try:
                self.basemanip.MoveActiveJoints(goal=finalarmsolution)
            except:
                return self.graspObject(gmodel.target)
        self.waitrobot()
        self.closefingers(target=gmodel.target)
        try:
            self.basemanip.MoveHandStraight(direction=self.updir,jacobian=0.02,stepsize=0.002,minsteps=1,maxsteps=100)
        except:
            print 'failed to move up'
        self.waitrobot()
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
        vmodel = visibilitymodel.VisibilityModel(robot=self.robot,target=target,sensorname=usevisibilitycamera['sensorname'])
        if not vmodel.load():
            raise planning_error('failed to load visibility model')
        #pts = array([dot(vmodel.target.GetTransform(),matrixFromPose(pose))[0:3,3] for pose in vmodel.visibilitytransforms])
        #h=vmodel.env.plot3(pts,10,colors=array([1,0.7,0,0.05]))
        reachabletransforms = vmodel.pruneTransformations(thresh=0.04,numminneighs=40)
        vmodel.SetCameraTransforms(reachabletransforms)
        for iter in range(5):
            vmodel.visualprob.MoveToObserveTarget(target=vmodel.target,sampleprob=0.001,maxiter=4000)
            #s=vmodel.visualprob.SampleVisibilityGoal(target)
            self.waitrobot()
            if usevisibilitycamera.get('dosync',False):
                if usevisibilitycamera.get('ask',False):
                    raw_input('press any key to capture new environment')
                time.sleep(1)
                with self.env:
                    newtarget,newenv = self.searchrealenv(target)
                    if newtarget is not None:
                        break
            else:
                newtarget = target
                break
        if newtarget is None:
            raise planning_error('cannot find target')
        if usevisibilitycamera.get('storeimage',False):
            usevisibilitycamera['image']= vmodel.getCameraImage()
        print repr(self.robot.GetJointValues())
        print repr(self.robot.GetTransform())
        return target,vmodel.manip
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
