# -*- coding: utf-8 -*-
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
"""Inverse reachability space of manipulators.

.. image:: ../../images/databases/inversereachability.jpg
  :width: 640

`[source] <../_modules/openravepy/databases/inversereachability.html>`_

**Running the Generator**

.. code-block:: bash

  openrave.py --database inversereachability --robot=robots/barrettsegway.robot.xml

Description
-----------

  Clusters the reachability space for a base-placement sampling distribution that can be used to find out where the robot should stand in order to perform a manipulation task.

.. image:: ../../images/databases/inversereachability_wam2.jpg
  :height: 200

.. image:: ../../images/databases/inversereachability_wam3.jpg
  :height: 200

Command-line
------------

.. shell-block:: openrave.py --database inversereachability --help

Class Definitions
-----------------

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import time,bisect

if not __openravepy_build_doc__:
    from numpy import *
else:
    from numpy import array

from ..openravepy_int import RaveFindDatabaseFile, RaveCreateRobot, IkParameterization, rotationMatrixFromAxisAngle, poseFromMatrix, matrixFromPose, matrixFromQuat, matrixFromAxisAngle, poseMult, quatFromAxisAngle, IkFilterOptions
from ..openravepy_ext import quatArrayTMult, quatArrayTDist, poseMultArrayT, normalizeZRotation
from . import DatabaseGenerator
from .. import pyANN
from . import kinematicreachability, linkstatistics, inversekinematics

import numpy
import os.path
from optparse import OptionParser

import logging
log = logging.getLogger('openravepy.'+__name__.split('.',2)[-1])

try:
    from scipy.optimize import leastsq
except ImportError:
    print 'could not import scipy.optimize.leastsq'

class InverseReachabilityModel(DatabaseGenerator):
    """Inverts the reachability and computes probability distributions of the robot's base given an end effector position"""
    def __init__(self,robot,id=None):
        DatabaseGenerator.__init__(self,robot=robot)
        self.rmodel = kinematicreachability.ReachabilityModel(robot=robot)
        self.ikmodel = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        self.equivalenceclasses = None
        self.rotweight = 0.2 # in-plane rotation weight with respect to xy offset
        self.id=id
        with self.robot:
            self.jointvalues = self.robot.GetDOFValues(self.getdofindices(self.manip))
    def clone(self,envother):
        clone = DatabaseGenerator.clone(self,envother)
        return clone        
    def has(self):
        return self.equivalenceclasses is not None and len(self.equivalenceclasses) > 0
    def getversion(self):
        return 3
    def load(self):
        try:
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()
            params = DatabaseGenerator.load(self)
            if params is None:
                return False
            self.equivalenceclasses,self.rotweight,self.xyzdelta,self.quatdelta,self.jointvalues = params
            self.preprocess()
            return self.has()
        except e:
            return False
    @staticmethod
    def classnormalizationconst(classstd):
        """normalization const for the equation exp(dot(-0.5/bandwidth**2,r_[arccos(x[0])**2,x[1:]**2]))"""
        gaussconst = -0.5*(len(classstd)-1)*numpy.log(pi)-0.5*numpy.log(prod(classstd[1:]))
        # normalization for the weights so that integrated volume is 1. this is necessary when comparing across different distributions?
        quatconst = numpy.log(1.0/classstd[0]**2+0.3334)
        return quatconst+gaussconst
    
    def preprocess(self):
        self.equivalencemeans = array([e[0] for e in self.equivalenceclasses])
        samplingbandwidth = array([self.quatdelta*0.1,self.xyzdelta*0.1])
        self.equivalenceweights = array([-0.5/(e[1]+samplingbandwidth)**2 for e in self.equivalenceclasses])
        self.equivalenceoffset = array([self.classnormalizationconst(e[1]+samplingbandwidth) for e in self.equivalenceclasses])

    def save(self):
        DatabaseGenerator.save(self,(self.equivalenceclasses,self.rotweight,self.xyzdelta,self.quatdelta,self.jointvalues))

    def getfilename(self,read=False):
        if self.id is None:
            basename='invreachability.' + self.manip.GetStructureHash() + '.pp'
        else:
            basename='invreachability.' + self.manip.GetStructureHash() + '.' + str(self.id) + '.pp'
        return RaveFindDatabaseFile(os.path.join('robot.'+self.robot.GetKinematicsGeometryHash(), basename),read)
    @staticmethod
    def getdofindices(manip):
        joints = manip.GetRobot().GetChain(0,manip.GetBase().GetIndex())
        dofindices = [arange(joint.GetDOFIndex(),joint.GetDOFIndex()+joint.GetDOF()) for joint in joints if joint.GetDOFIndex()>=0 and not joint.IsStatic()]
        return hstack(dofindices) if len(dofindices) > 0 else array([],int)
    @staticmethod
    def getManipulatorLinks(manip):
        links = manip.GetChildLinks()
        robot=manip.GetRobot()
        joints = robot.GetJoints()
        for jindex in r_[manip.GetArmIndices(),InverseReachabilityModel.getdofindices(manip)]:
            joint = joints[jindex]
            if joint.GetFirstAttached() and not joint.GetFirstAttached() in links:
                links.append(joint.GetFirstAttached())
            if joint.GetSecondAttached() and not joint.GetSecondAttached() in links:
                links.append(joint.GetSecondAttached())
        # don't forget the rigidly attached links
        for link in links[:]:
            for newlink in link.GetRigidlyAttachedLinks():
                if not newlink in links:
                    links.append(newlink)
        return links
    def necessaryjointstate(self):
        return self.jointvalues,self.getdofindices(self.manip)
    def autogenerate(self,options=None):
        heightthresh=None
        quatthresh=None
        Nminimum=None
        if options is not None:
            if options.heightthresh is not None:
                heightthresh=options.heightthresh
            if options.quatthresh is not None:
                quatthresh=options.quatthresh
            if options.id is not None:
                self.id=options.id
            if options.jointvalues is not None:
                self.jointvalues = array([float(s) for s in options.jointvalues.split()])
                assert(len(self.jointvalues)==len(self.getdofindices(self.manip)))
        if self.robot.GetKinematicsGeometryHash() == 'e829feb384e6417bbf5bd015f1c6b49a' or self.robot.GetKinematicsGeometryHash() == '22548f4f2ecf83e88ae7e2f3b2a0bd08': # wam 7dof
            if heightthresh is None:
                heightthresh=0.05
            if quatthresh is None:
                quatthresh=0.15            
        self.generate(heightthresh=heightthresh,quatthresh=quatthresh,Nminimum=Nminimum)
        self.save()
    def generate(self,heightthresh=None,quatthresh=None,Nminimum=None):
        """First transform all end effectors to the identity and get the robot positions,
        then cluster the robot position modulo in-plane rotation (z-axis) and position (xy),
        then compute statistics for each cluster."""
        # disable every body but the target and robot
        bodies = [(b,b.IsEnabled()) for b in self.env.GetBodies() if b != self.robot]
        for b in bodies:
            b[0].Enable(False)
        statesaver = self.robot.CreateRobotStateSaver()
        maniplinks = self.getManipulatorLinks(self.manip)
        try:
            for link in self.robot.GetLinks():
                link.Enable(link in maniplinks)
            if heightthresh is None:
                heightthresh=0.05
            if quatthresh is None:
                quatthresh=0.15
            if Nminimum is None:
                Nminimum=10
            if not self.rmodel.load():
                self.rmodel.autogenerate()
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()
            log.info("Generating Inverse Reachability, heightthresh=%f, quatthresh=%f",heightthresh,quatthresh)
            self.robot.SetDOFValues(*self.necessaryjointstate())
            # get base of link manipulator with respect to base link
            Tbase = dot(linalg.inv(self.robot.GetTransform()),self.manip.GetBase().GetTransform())
            # convert the quatthresh to a loose euclidean distance
            self.xyzdelta = self.rmodel.xyzdelta
            self.quatdelta = self.rmodel.quatdelta
            self.rotweight = heightthresh/quatthresh
            quateucdist2 = (1-cos(quatthresh))**2+sin(quatthresh)**2
            # find the density
            basetrans = array(self._GetValue(self.rmodel.reachabilitystats))
            assert len(basetrans) > 0
            basetrans[:,0:7] = poseMultArrayT(poseFromMatrix(Tbase),basetrans[:,0:7])
            # find the density of the points
            searchtrans = c_[basetrans[:,0:4],basetrans[:,6:7]]
            kdtree = kinematicreachability.ReachabilityModel.QuaternionKDTree(searchtrans,1.0/self.rotweight)
            transdensity = kdtree.kFRSearchArray(searchtrans,0.25*quateucdist2,0,quatthresh*0.2)[2]
            basetrans = basetrans[argsort(-transdensity),:]
            Nminimum = max(Nminimum,4)
            # find all equivalence classes
            quatrolls = array([quatFromAxisAngle(array((0,0,1)),roll) for roll in arange(0,2*pi,quatthresh*0.5)])
            self.equivalenceclasses = []
            while len(basetrans) > 0:
                searchtrans = c_[basetrans[:,0:4],basetrans[:,6:7]]
                kdtree = kinematicreachability.ReachabilityModel.QuaternionKDTree(searchtrans,1.0/self.rotweight)
                querypoints = c_[quatArrayTMult(quatrolls, searchtrans[0][0:4]),tile(searchtrans[0][4:],(len(quatrolls),1))]
                foundindices = zeros(len(searchtrans),bool)
                for querypoint in querypoints:
                    k = min(len(searchtrans),1000)
                    neighs,dists,kball = kdtree.kFRSearchArray(reshape(querypoint,(1,5)),quateucdist2,k,quatthresh*0.01)
                    if k < kball:
                        neighs,dists,kball = kdtree.kFRSearchArray(reshape(querypoint,(1,5)),quateucdist2,kball,quatthresh*0.01)
                    foundindices[neighs] = True
                equivalenttrans = basetrans[flatnonzero(foundindices),:]
                normalizedqarray,zangles = normalizeZRotation(equivalenttrans[:,0:4])
                # get the 'mean' of the normalized quaternions best describing the distribution
                # for initialization, make sure all quaternions are on the same hemisphere
                identityquat = tile(array((1.0,0,0,0)),(normalizedqarray.shape[0],1))
                normalizedqarray[flatnonzero(sum((normalizedqarray+identityquat)**2,1) < sum((normalizedqarray-identityquat)**2, 1)),0:4] *= -1
                q0 = sum(normalizedqarray,axis=0)
                q0 /= sqrt(sum(q0**2))
                if len(normalizedqarray) >= Nminimum:
                    qmean,success = leastsq(lambda q: quatArrayTDist(q/sqrt(sum(q**2)),normalizedqarray), normalizedqarray[0],maxfev=10000)
                    qmean /= sqrt(sum(qmean**2))
                else:
                    qmean = q0
                qstd = sqrt(sum(quatArrayTDist(qmean,normalizedqarray)**2)/len(normalizedqarray))
                # compute statistics, store the angle, xy offset, and remaining unprocessed data
                czangles = cos(zangles)
                szangles = sin(zangles)
                equivalenttransinv = -c_[czangles*equivalenttrans[:,4]+szangles*equivalenttrans[:,5],-szangles*equivalenttrans[:,4]+czangles*equivalenttrans[:,5]]
                equivalenceclass = (r_[qmean,mean(equivalenttrans[:,6])],
                                    r_[qstd,std(equivalenttrans[:,6])],
                                    c_[-zangles,equivalenttransinv,equivalenttrans[:,7:]])
                self.equivalenceclasses.append(equivalenceclass)
                basetrans = basetrans[flatnonzero(foundindices==False),:]
                log.info('new equivalence class outliers: %d/%d, left over trans: %d',self.testEquivalenceClass(equivalenceclass)*len(zangles),len(zangles),len(basetrans))
        finally:
            statesaver.Release()
            for b,enable in bodies:
                b.Enable(enable)
        self.preprocess()
        
    def getEquivalenceClass(self,Tgrasp):
        with self.env:
            Tbase = self.manip.GetBase().GetTransform()
        posebase = poseFromMatrix(Tbase)
        qbaserobotnorm,zbaseangle = normalizeZRotation(reshape(posebase[0:4],(1,4)))
        if quatArrayTDist([1.0,0,0,0],qbaserobotnorm) > 0.05:
            raise planning_error('out of plane rotations for base are not supported')
        posetarget = poseFromMatrix(dot(linalg.inv(Tbase),Tgrasp))
        qnormalized,znormangle = normalizeZRotation(reshape(posetarget[0:4],(1,4)))
        # find the closest cluster
        logll = quatArrayTDist(qnormalized[0],self.equivalencemeans[:,0:4])**2*self.equivalenceweights[:,0] + (posetarget[6]-self.equivalencemeans[:,4])**2*self.equivalenceweights[:,1] + self.equivalenceoffset
        bestindex = argmax(logll)
        return self.equivalenceclasses[bestindex],logll[bestindex]

    def computeBaseDistribution(self,Tgrasp,logllthresh=2.0,zaxis=None):
        """Return a function of the distribution of possible positions of the robot such that Tgrasp is reachable. Also returns a sampler function"""
        if zaxis is not None:
            raise NotImplementedError('cannot specify a custom zaxis yet')
        with self.env:
            Tbase = self.manip.GetBase().GetTransform()
            poserobot = poseFromMatrix(dot(self.robot.GetTransform(),linalg.inv(Tbase)))
        
        rotweight = self.rotweight
        irotweight = 1.0/rotweight
        posebase = poseFromMatrix(Tbase)
        qbaserobotnorm,zbaseangle = normalizeZRotation(reshape(posebase[0:4],(1,4)))
        if quatArrayTDist([1.0,0,0,0],qbaserobotnorm) > 0.05:
            raise planning_error('out of plane rotations for base are not supported')
        
        bandwidth = array((rotweight*self.quatdelta ,self.xyzdelta,self.xyzdelta))
        ibandwidth=-0.5/bandwidth**2
        normalizationconst = (1.0/sqrt(pi**3*prod(bandwidth)))
        searchradius=9.0*sum(bandwidth**2)
        searcheps=bandwidth[0]*0.1
        
        posetarget = poseFromMatrix(dot(linalg.inv(Tbase),Tgrasp))
        qnormalized,znormangle = normalizeZRotation(reshape(posetarget[0:4],(1,4)))
        # find the closest cluster
        logll = quatArrayTDist(qnormalized[0],self.equivalencemeans[:,0:4])**2*self.equivalenceweights[:,0] + (posetarget[6]-self.equivalencemeans[:,4])**2*self.equivalenceweights[:,1] + self.equivalenceoffset
        bestindex = argmax(logll)
        if logll[bestindex] < logllthresh:
            log.info('inversereachability: could not find base distribution: index=%d',logll[bestindex])
            return None,None,None

        # transform the equivalence class to the global coord system and create a kdtree for faster retrieval
        equivalenceclass = self.equivalenceclasses[bestindex]
        points = array(equivalenceclass[2][:,0:3])
        # transform points by the base and grasp pose
        Tbaserot = c_[rotationMatrixFromAxisAngle([0,0,1],zbaseangle)[0:2,0:2],posebase[4:6]]
        Ttargetrot = c_[rotationMatrixFromAxisAngle([0,0,1],znormangle)[0:2,0:2],posetarget[4:6]]
        Trot = dot(Tbaserot, r_[Ttargetrot,[[0,0,1]]])
        points = c_[equivalenceclass[2][:,0]+znormangle+zbaseangle,dot(equivalenceclass[2][:,1:3],transpose(Trot[0:2,0:2]))+ tile(Trot[0:2,2], (len(equivalenceclass[2]),1))]
        bounds = array((numpy.min(points,0)-bandwidth,numpy.max(points,0)+bandwidth))
        if bounds[1,0]-bounds[0,0] > 2*pi:
           # already covering entire circle, so limit to 2*pi
           bounds[0,0] = -pi
           bounds[1,0] = pi

        points[:,0] *= rotweight
        kdtree = pyANN.KDTree(points)
        searchradius=9.0*sum(bandwidth**2)
        searcheps=bandwidth[0]*0.2
        weights=equivalenceclass[2][:,3]*normalizationconst
        cumweights = cumsum(weights)
        cumweights = cumweights[1:]/cumweights[-1]

        def gaussiankerneldensity(poses):
            """returns the density"""
            qposes,zposeangles = normalizeZRotation(poses[:,0:4])
            p = c_[zposeangles*rotweight,poses[:,4:6]]
            neighs,dists,kball = kdtree.kFRSearchArray(p,searchradius,16,searcheps)
            probs = zeros(p.shape[0])
            for i in range(p.shape[0]):
                inds = neighs[i,neighs[i,:]>=0]
                if len(inds) > 0:
                    probs[i] = dot(weights[inds],numpy.exp(dot((points[inds,:]-tile(p[i,:],(len(inds),1)))**2,ibandwidth)))
            return probs
        def gaussiankernelsampler(N=1,weight=1.0):
            """samples the distribution and returns a transform as a pose"""
            samples = random.normal(array([points[bisect.bisect(cumweights,random.rand()),:]  for i in range(N)]),bandwidth*weight)
            samples[:,0] *= 0.5*irotweight
            return poseMultArrayT(poserobot,c_[cos(samples[:,0]),zeros((N,2)),sin(samples[:,0]),samples[:,1:3],tile(Tbase[2,3],N)]),self.necessaryjointstate()
        return gaussiankerneldensity,gaussiankernelsampler,bounds

    def computeAggregateBaseDistribution(self,Tgrasps,logllthresh=2.0,zaxis=None):
        """Return a function of the distribution of possible positions of the robot such that any grasp from Tgrasps is reachable.
        Also computes a sampler function that returns a random position of the robot along with the index into Tgrasps"""
        if zaxis is not None:
            raise NotImplementedError('cannot specify a custom zaxis yet')
        with self.env:
            Tbase = self.manip.GetBase().GetTransform()
            poserobot = poseFromMatrix(dot(self.robot.GetTransform(),linalg.inv(Tbase)))
        
        rotweight = self.rotweight
        irotweight = 1.0/rotweight
        posebase = poseFromMatrix(Tbase)
        qbaserobotnorm,zbaseangle = normalizeZRotation(reshape(posebase[0:4],(1,4)))
        if quatArrayTDist([1.0,0,0,0],qbaserobotnorm) > 0.05:
            raise planning_error('out of plane rotations for base are not supported')
        
        bandwidth = array((rotweight*self.quatdelta ,self.xyzdelta,self.xyzdelta))
        ibandwidth=-0.5/bandwidth**2
        # normalization for the weights so that integrated volume is 1. this is necessary when comparing across different distributions?
        normalizationconst = (1.0/sqrt(pi**3*sum(bandwidth**2)))
        searchradius=9.0*sum(bandwidth**2)
        searcheps=bandwidth[0]*0.1
        
        points = zeros((0,3))
        weights = array(())
        graspindices = []
        graspindexoffsets = []
        highestlogll = -inf
        for Tgrasp,graspindex in Tgrasps:
            posetarget = poseFromMatrix(dot(linalg.inv(Tbase),Tgrasp))
            qnormalized,znormangle = normalizeZRotation(reshape(posetarget[0:4],(1,4)))
            # find the closest cluster
            logll = quatArrayTDist(qnormalized[0],self.equivalencemeans[:,0:4])**2*self.equivalenceweights[:,0] + (posetarget[6]-self.equivalencemeans[:,4])**2*self.equivalenceweights[:,1] + self.equivalenceoffset
            bestindex = argmax(logll)
            highestlogll = max(highestlogll,logll[bestindex])
            if logll[bestindex] <logllthresh:
                continue
            graspindices.append(graspindex)
            graspindexoffsets.append(len(points))
            # transform the equivalence class to the global coord system and create a kdtree for faster retrieval
            equivalenceclass = self.equivalenceclasses[bestindex]
            # transform points by the grasp pose
            newpoints = c_[equivalenceclass[2][:,0]+znormangle,dot(equivalenceclass[2][:,1:3],transpose(rotationMatrixFromAxisAngle([0,0,1],znormangle)[0:2,0:2])) + tile(posetarget[4:6], (len(equivalenceclass[2]),1))]
            points = r_[points,newpoints]
            weights = r_[weights,equivalenceclass[2][:,3]*normalizationconst]

        if len(points) == 0:
            log.info('inversereachability: could not find base distribution, logllthresh too high? logll=%f', highestlogll)
            return None,None,None
        
        # transform points by the base pose
        points[:,0] += zbaseangle
        points[:,1:3] = dot(points[:,1:3],transpose(rotationMatrixFromAxisAngle([0,0,1],zbaseangle)[0:2,0:2])) + tile(posebase[4:6], (len(points),1))
        
        bounds = array((numpy.min(points,0)-bandwidth,numpy.max(points,0)+bandwidth))
        if bounds[1,0]-bounds[0,0] > 2*pi:
            # already covering entire circle, so limit to 2*pi
            bounds[0,0] = -pi
            bounds[1,0] = pi
        points[:,0] *= rotweight
        kdtree = pyANN.KDTree(points)
        cumweights = cumsum(weights)
        cumweights = cumweights[1:]/cumweights[-1]
        
        def gaussiankerneldensity(poses):
            """returns the density"""
            qposes,zposeangles = normalizeZRotation(poses[:,0:4])
            p = c_[zposeangles*rotweight,poses[:,4:6]]
            neighs,dists,kball = kdtree.kFRSearchArray(p,searchradius,16,searcheps)
            probs = zeros(p.shape[0])
            for i in range(p.shape[0]):
                inds = neighs[i,neighs[i,:]>=0]
                if len(inds) > 0:
                    probs[i] = dot(weights[inds],numpy.exp(dot((points[inds,:]-tile(p[i,:],(len(inds),1)))**2,ibandwidth)))
            return probs
        def gaussiankernelsampler(N=1,weight=1.0):
            """samples the distribution and returns a transform as a pose"""
            sampledgraspindices = []
            sampledpoints = zeros((N,3))
            for i in range(N):
                pointindex = bisect.bisect(cumweights,random.rand())
                sampledgraspindices.append(graspindices[bisect.bisect(graspindexoffsets,pointindex)-1])
                sampledpoints[i,:] = points[pointindex,:]
            samples = random.normal(sampledpoints,bandwidth*weight)
            samples[:,0] *= 0.5*irotweight
            return poseMultArrayT(poserobot,c_[cos(samples[:,0]),zeros((N,2)),sin(samples[:,0]),samples[:,1:3],tile(Tbase[2,3],N)]),sampledgraspindices,self.necessaryjointstate()
        return gaussiankerneldensity,gaussiankernelsampler,bounds

    def sampleBaseDistributionIterator(self,Tgrasps,logllthresh=2.0,weight=1.0,Nprematuresamples=1,zaxis=None):
        """infinitely samples valid base placements from Tgrasps. Assumes environment is locked. If Nprematuresamples > 0, will sample from the clusters as soon as they are found"""
        Tbase = self.manip.GetBase().GetTransform()
        poserobot = poseFromMatrix(dot(self.robot.GetTransform(),linalg.inv(Tbase)))
        rotweight = self.rotweight
        irotweight = 1.0/rotweight
        posebase = poseFromMatrix(Tbase)
        qbaserobotnorm,zbaseangle = normalizeZRotation(reshape(posebase[0:4],(1,4)))
        if quatArrayTDist([1.0,0,0,0],qbaserobotnorm) > 0.05:
            raise planning_error('out of plane rotations for base are not supported')
        
        bandwidth = array((rotweight*self.quatdelta ,self.xyzdelta,self.xyzdelta))
        ibandwidth=-0.5/bandwidth**2
        normalizationconst = (1.0/sqrt(pi**3*prod(bandwidth)))
        searchradius=9.0*sum(bandwidth**2)
        searcheps=bandwidth[0]*0.1
        
        points = zeros((0,3))
        weights = array(())
        graspindices = []
        graspindexoffsets = []
        Tbaserot = c_[rotationMatrixFromAxisAngle([0,0,1],zbaseangle)[0:2,0:2],posebase[4:6]]
        for Tgrasp,graspindex in Tgrasps:
            posetarget = poseFromMatrix(dot(linalg.inv(Tbase),Tgrasp))
            qnormalized,znormangle = normalizeZRotation(reshape(posetarget[0:4],(1,4)))
            # find the closest cluster
            logll = quatArrayTDist(qnormalized[0],self.equivalencemeans[:,0:4])**2*self.equivalenceweights[:,0] + (posetarget[6]-self.equivalencemeans[:,4])**2*self.equivalenceweights[:,1] + self.equivalenceoffset
            bestindex = argmax(logll)
            if logll[bestindex] < logllthresh:
                continue
            # transform the equivalence class to the global coord system and create a kdtree for faster retrieval
            equivalenceclass = self.equivalenceclasses[bestindex]
            # transform points by the grasp pose
            Ttargetrot = c_[rotationMatrixFromAxisAngle([0,0,1],znormangle)[0:2,0:2],posetarget[4:6]]
            Trot = dot(Tbaserot, r_[Ttargetrot,[[0,0,1]]])
            newpoints = c_[equivalenceclass[2][:,0]+znormangle+zbaseangle,dot(equivalenceclass[2][:,1:3],transpose(Trot[0:2,0:2]))+ tile(Trot[0:2,2], (len(equivalenceclass[2]),1))]
            newweights = equivalenceclass[2][:,3]*normalizationconst
            if Nprematuresamples > 0:
                newpoints[:,0] *= rotweight
                kdtree = pyANN.KDTree(newpoints)
                cumweights = cumsum(newweights)
                cumweights = cumweights[1:]/cumweights[-1]
                for i in range(Nprematuresamples):
                    sample = random.normal(newpoints[bisect.bisect(cumweights,random.rand()),:],bandwidth*weight)
                    sample[0] *= 0.5*irotweight
                    yield poseMult(poserobot,r_[cos(sample[0]),0,0,sin(sample[0]),sample[1:3],Tbase[2,3]]),graspindex,self.necessaryjointstate()
            graspindices.append(graspindex)
            graspindexoffsets.append(len(points))
            points = r_[points,newpoints]
            weights = r_[weights,newweights]

        if len(points) == 0:
            raise planning_error('could not find base distribution')
        
        kdtree = pyANN.KDTree(points)
        cumweights = cumsum(weights)
        cumweights = cumweights[1:]/cumweights[-1]
        while True:
            pointindex = bisect.bisect(cumweights,random.rand())
            sampledgraspindex = graspindices[bisect.bisect(graspindexoffsets,pointindex)-1]
            sample = random.normal(points[pointindex,:],bandwidth*weight)
            sample[0] *= 0.5*irotweight
            yield poseMult(poserobot,r_[cos(sample[0]),0,0,sin(sample[0]),sample[1:3],Tbase[2,3]]),sampledgraspindex,self.necessaryjointstate()

    def randomBaseDistributionIterator(self,Tgrasps,Nprematuresamples=1,bounds=None,**kwargs):
        """randomly sample base positions given the grasps. This is mostly used for comparison"""
        if bounds is None:
            bounds = array(((0,-1.0,-1.0),(2*pi,1.0,1.0)))
        dbounds = bounds[1,:]-bounds[0,:]
        Trobot = self.robot.GetTransform()
        grasps = []
        for Tgrasp,graspindex in Tgrasps:
            for i in range(Nprematuresamples):
                r = random.rand(3)
                angle = 0.5*(bounds[0,0]+r[0]*dbounds[0])
                yield r_[cos(angle),0,0,sin(angle),Tgrasp[0:2,3] + (bounds[0,1:3]+r[1:3]*dbounds[1:3]),Trobot[2,3]],graspindex,self.necessaryjointstate()
            grasps.append((Tgrasp,graspindex))
        while True:
            Tgrasp,graspindex = grasps[random.randint(len(grasps))]
            r = random.rand(3)
            angle = 0.5*(bounds[0,0]+r[0]*dbounds[0])
            yield r_[cos(angle),0,0,sin(angle),Tgrasp[0:2,3] + (bounds[0,1:3]+r[1:3]*dbounds[1:3]),Trobot[2,3]],graspindex,self.necessaryjointstate()

    def testSampling(self, heights=None,N=100,weight=1.0,**kwargs):
        if heights is None:
            heights = arange(0,0.5,-0.3,-0.7)
        with self.robot:
            allfailures = []
            for height in heights:
                T = eye(4)
                T[2,3] = height
                self.robot.SetTransform(T)
                densityfn,samplerfn,bounds = self.computeBaseDistribution(eye(4),**kwargs)
                if densityfn is not None:
                    poses,jointstate = samplerfn(N,weight)
                    failures = 0
                    for pose in poses:
                        self.robot.SetTransform(matrixFromPose(pose))
                        self.robot.SetDOFValues(*jointstate)
                        if self.manip.FindIKSolution(eye(4),0) is None:
                            #print 'pose failed: ',pose
                            failures += 1
                    log.info('height %f, failures: %d',height,failures)
                    allfailures.append(failures)
                else:
                    allfailures.append(inf)
        return allfailures

    def testEquivalenceClass(self,equivalenceclass):
        """tests that configurations in the cluster has IK solutions"""
        with self.robot:
            self.robot.SetDOFValues(*self.necessaryjointstate())
            Tgrasp = matrixFromQuat(equivalenceclass[0][0:4])
            Tgrasp[2,3] = equivalenceclass[0][4]
            failed = 0
            for sample in equivalenceclass[2]:
                Tmanip = matrixFromAxisAngle([0,0,1],sample[0])
                Tmanip[0:2,3] = sample[1:3]
                self.robot.SetTransform(Tmanip)
                solution = self.manip.FindIKSolution(Tgrasp,0)
                if solution is None:
                    failed += 1
            return float(failed)/len(equivalenceclass[2])
    def getCloseIK(self,Tgrasp):
        with self.env:
            Torggrasp = array(Tgrasp)
            while True:
                solution = self.manip.FindIKSolution(Tgrasp,0)
                if solution is not None:
                    break
                Tgrasp[0:3,3] = Torggrasp[0:3,3] + (random.rand(3)-0.5)*0.01
                Tgrasp[0:3,0:3] = dot(Torggrasp[0:3,0:3],rotationMatrixFromAxisAngle(random.rand(3)*0.05))
            return solution
    @staticmethod
    def showBaseDistribution(env,densityfn,bounds,zoffset=0,thresh=1.0,maxprob=None,marginalizeangle=True):
        discretization = [0.1,0.04,0.04]
        A,Y,X = mgrid[bounds[0,0]:bounds[1,0]:discretization[0], bounds[0,2]:bounds[1,2]:discretization[2], bounds[0,1]:bounds[1,1]:discretization[1]]
        N = prod(A.shape)
        poses = c_[cos(ravel(A)*0.5),zeros((N,2)),sin(ravel(A)*0.5),X.flat,Y.flat,zeros((N,1))]
        # split it into chunks to avoid memory overflow
        probs = zeros(len(poses))
        for i in range(0,len(poses),500000):
            log.info('%d/%d',i,len(poses))
            probs[i:(i+500000)] = densityfn(poses[i:(i+500000),:]);
        if marginalizeangle:
            probsxy = mean(reshape(probs,(A.shape[0],A.shape[1]*A.shape[2])),axis=0)
            inds = flatnonzero(probsxy>thresh)
            if maxprob is None:
                maxprob = max(probsxy)
            normalizedprobs = numpy.minimum(1,probsxy/maxprob)
            Ic = zeros((X.shape[1],X.shape[2],4))
            Ic[:,:,2] = reshape(normalizedprobs,Ic.shape[0:2])
            Ic[:,:,3] = reshape(normalizedprobs*array(probsxy>thresh,'float'),Ic.shape[0:2])
            Tplane = eye(4)
            Tplane[0:2,3] = mean(bounds[:,1:3],axis=0)
            Tplane[2,3] = zoffset
            return env.drawplane(transform=Tplane,extents=(bounds[1,1:3]-bounds[0,1:3])/2,texture=Ic)

        else:
            inds = flatnonzero(probs>thresh)
            if maxprob is None:
                maxprob = max(probs)
            normalizedprobs = numpy.minimum(1,probs[inds]/maxprob)
            colors = c_[0*normalizedprobs,normalizedprobs,normalizedprobs,normalizedprobs]
            points = c_[poses[inds,4:6],A.flatten()[inds]+zoffset]
            return env.plot3(points=array(points),colors=array(colors),pointsize=10)
    def showEquivalenceClass(self,equivalenceclass,transparency = 0.8,neighthresh=0.1,onlymaniplinks=True):
        """Overlays several robots of the same equivalence class"""
        inds = linkstatistics.LinkStatisticsModel.prunePointsKDTree(equivalenceclass[2][:,0:3],neighthresh,1)
        robotlocs = []
        with self.robot:
            if onlymaniplinks:
                maniplinks = self.getManipulatorLinks(self.manip)
                for link in self.robot.GetLinks():
                    link.Enable(link in maniplinks)
            Tgrasp = matrixFromQuat(equivalenceclass[0][0:4])
            Tgrasp[2,3] = equivalenceclass[0][4]
            Tgrasp = dot(dot(linalg.inv(self.robot.GetTransform()),self.manip.GetBase().GetTransform()),Tgrasp)
            #posebase = poseFromMatrix(Tbase)
            #qbaserobotnorm,zbaseangle = normalizeZRotation(reshape(posebase[0:4],(1,4)))
            #qbaserobotnorm,zbaseangle = normalizeZRotation(
            for sample in equivalenceclass[2][inds,:]:
                Tmanip = matrixFromAxisAngle([0,0,1],sample[0])
                Tmanip[0:2,3] = sample[1:3]
                self.robot.SetTransform(Tmanip)
                self.robot.SetDOFValues(*self.necessaryjointstate())
                solution = self.manip.FindIKSolution(Tgrasp,0)
                if solution is not None:
                    self.robot.SetDOFValues(solution,self.manip.GetArmIndices())
                    robotlocs.append((self.robot.GetTransform(),self.robot.GetDOFValues()))
        try:
            log.info('number of locations %d',len(robotlocs))
            with self.env:
                self.env.Remove(self.robot)
                newrobots = []
                for T,values in robotlocs:
                    newrobot = RaveCreateRobot(self.robot.GetXMLId())
                    newrobot.Clone(self.robot,0)
                    for link in newrobot.GetLinks():
                        for geom in link.GetGeometries():
                            geom.SetTransparency(transparency)
                    self.env.Add(newrobot,True)
                    newrobot.SetTransform(T)
                    newrobot.SetDOFValues(values)
                    newrobots.append(newrobot)
                    #time.sleep(0.1)
            raw_input('press any key to continue')
        finally:
            for newrobot in newrobots:
                self.env.Remove(newrobot)
            self.env.Add(self.robot)

    def show(self,options=None):
        if self.env.GetViewer() is None:
            self.env.SetViewer('qtcoin')
            time.sleep(0.4) # give time for viewer to initialize
        maxnumber=20 if options is None else options.show_maxnumber
        transparency=0.8 if options is None else options.show_transparency
        with self.env:
            newrobots = []
            for ind in range(maxnumber):
                newrobot = RaveCreateRobot(self.env,self.robot.GetXMLId())
                newrobot.Clone(self.robot,0)
                for link in newrobot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)
                newrobots.append(newrobot)
            lower,upper = [v[self.manip.GetArmIndices()] for v in self.robot.GetDOFLimits()]
        while True:
            # find a random position
            with self.robot:
                while True:
                    self.robot.SetDOFValues(random.rand()*(upper-lower)+lower,self.manip.GetArmIndices()) # set random values
                    if not self.robot.CheckSelfCollision():
                        break
                Tgrasp = self.manip.GetEndEffectorTransform()
            with self.env:
                densityfn,samplerfn,bounds = self.computeBaseDistribution(Tgrasp,logllthresh=1.0)
                if densityfn is None:
                    log.warn('no distribution exists')
                    continue
                goals = []
                while len(goals) < maxnumber:
                    poses,jointstate = samplerfn(1,1.0)
                    for pose in poses:
                        self.robot.SetTransform(matrixFromPose(pose))
                        self.robot.SetDOFValues(*jointstate)
                        q = self.manip.FindIKSolution(Tgrasp,filteroptions=IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            values = self.robot.GetDOFValues()
                            values[self.manip.GetArmIndices()] = q
                            goals.append((pose,values))
                            if len(goals) >= maxnumber:
                                break
                        else:
                            log.warn('failed to sample')
            try:
                h=self.env.plot3(Tgrasp[0:3,3],20.0)
                with self.env:
                    self.env.Remove(self.robot)
                    for i in range(len(goals)):
                        newrobot = newrobots[i]
                        self.env.Add(newrobot,True)
                        newrobot.SetTransform(goals[i][0])
                        newrobot.SetDOFValues(goals[i][1])
                raw_input('press any key to continue')
            finally:
                h=None
                for newrobot in newrobots:
                    self.env.Remove(newrobot)
                self.env.Add(self.robot)


    @staticmethod
    def CreateOptionParser():
        parser = DatabaseGenerator.CreateOptionParser()
        parser.description = 'Generates model storing the inverse reachability space and its clusters.'
        parser.usage='openrave.py --database inversereachability [options]'
        parser.add_option('--heightthresh',action='store',type='float',dest='heightthresh',default=None,
                          help='The max radius of the arm to perform the computation (default=0.05)')
        parser.add_option('--quatthresh',action='store',type='float',dest='quatthresh',default=None,
                          help='The max radius of the arm to perform the computation (default=0.15)')
        parser.add_option('--id',action='store',type='string',dest='id',default=None,
                          help='Special id differentiating inversereachability models')
        parser.add_option('--jointvalues',action='store',type='string',dest='jointvalues',default=None,
                          help='String of joint values that connect the robot base link to the manipulator base link')
        parser.add_option('--show_maxnumber',action='store',type='int',dest='show_maxnumber',default=20,
                          help='Number of robots to show simultaneously (default=%default)')
        parser.add_option('--show_transparency',action='store',type='float',dest='show_transparency',default=0.8,
                          help='Transparency of the robots to show (default=%default)')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None,args=None,**kwargs):
        if parser is None:
           parser = InverseReachabilityModel.CreateOptionParser()
        (options, leftargs) = parser.parse_args(args=args)
        Model = lambda robot: InverseReachabilityModel(robot=robot)
        DatabaseGenerator.RunFromParser(Model=Model,parser=parser,args=args,**kwargs)

def run(*args,**kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    InverseReachabilityModel.RunFromParser(*args,**kwargs)
