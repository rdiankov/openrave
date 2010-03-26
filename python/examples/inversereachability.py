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

import time,bisect,itertools
from openravepy import *
from openravepy import pyANN
from openravepy.examples import kinematicreachability, linkstatistics
import numpy
from numpy import *
from optparse import OptionParser
try:
    from scipy.optimize import leastsq
except ImportError:
    pass
#from IPython.Debugger import Tracer; debug_here = Tracer()

class InverseReachabilityModel(OpenRAVEModel):
    """Inverts the reachability and computes probability distributions of the robot's base given an end effector position"""

    class QuaternionKDTree(metaclass.AutoReloader):
        """Artificially add more weight to the X,Y,Z translation dimensions"""
        def __init__(self, poses,transmult):
            self.numposes = len(poses)
            self.transmult = transmult
            self.itransmult = 1/transmult
            searchposes = array(poses)
            searchposes[:,4:] *= self.transmult # take translation errors more seriously
            allposes = r_[searchposes,searchposes]
            allposes[self.numposes:,0:4] *= -1
            self.nnposes = pyANN.KDTree(allposes)
        def kSearch(self,poses,k,eps):
            """returns distance squared"""
            poses[:,4:] *= self.transmult
            neighs,dists = self.nnposes.kSearch(poses,k,eps)
            neighs[neighs>=self.numposes] -= self.numposes
            poses[:,4:] *= self.itransmult
            return neighs,dists
        def kFRSearch(self,pose,radiussq,k,eps):
            """returns distance squared"""
            pose[4:] *= self.transmult
            neighs,dists,kball = self.nnposes.kFRSearch(pose,radiussq,k,eps)
            neighs[neighs>=self.numposes] -= self.numposes
            pose[4:] *= self.itransmult
            return neighs,dists,kball
        def kFRSearchArray(self,poses,radiussq,k,eps):
            """returns distance squared"""
            poses[:,4:] *= self.transmult
            neighs,dists,kball = self.nnposes.kFRSearchArray(poses,radiussq,k,eps)
            neighs[neighs>=self.numposes] -= self.numposes
            poses[:,4:] *= self.itransmult
            return neighs,dists,kball

    def __init__(self,robot):
        OpenRAVEModel.__init__(self,robot=robot)
        self.rmodel = kinematicreachability.ReachabilityModel(robot=robot)
        self.rmodel.load()
        self.equivalenceclasses = None
        self.rotweight = 0.2 # in-plane rotation weight with respect to xy offset
        
    def has(self):
        return self.equivalenceclasses is not None and len(self.equivalenceclasses) > 0
    def getversion(self):
        return 1
    def load(self):
        try:
            params = OpenRAVEModel.load(self)
            if params is None:
                return False
            self.equivalenceclasses,self.rotweight,self.xyzdelta,self.quatdelta = params
            self.preprocess()
            return self.has()
        except e:
            return False
    @staticmethod
    def classnormalizationconst(classstd):
        """normalization const for the equation exp(dot(-0.5/bandwidth**2,r_[arccos(x[0])**2,x[1:]**2]))"""
        gaussconst = -0.5*(len(classstd)-1)*log(pi)-0.5*log(prod(classstd[1:]))
        # normalization for the weights so that integrated volume is 1. this is necessary when comparing across different distributions?
        quatconst = log(1.0/classstd[0]**2+0.3334)
        return quatconst+gaussconst

    def preprocess(self):
        self.equivalencemeans = array([e[0] for e in self.equivalenceclasses])
        samplingbandwidth = array([self.quatdelta*0.1,self.xyzdelta*0.1])
        self.equivalenceweights = array([-0.5/(e[1]+samplingbandwidth)**2 for e in self.equivalenceclasses])
        self.equivalenceoffset = array([self.classnormalizationconst(e[1]+samplingbandwidth) for e in self.equivalenceclasses])

    def save(self):
        OpenRAVEModel.save(self,(self.equivalenceclasses,self.rotweight,self.xyzdelta,self.quatdelta))

    def getfilename(self):
       return os.path.join(OpenRAVEModel.getfilename(self),'invreachability.' + self.manip.GetName() + '.pp')

    def autogenerate(self,options=None):
        # disable every body but the target and robot
        bodies = [b for b in self.env.GetBodies() if b.GetNetworkId() != self.robot.GetNetworkId()]
        for b in bodies:
            b.Enable(False)
        try:
            heightthresh=None
            quatthresh=None
            Nminimum=None
            if options is not None:
                if options.heightthresh is not None:
                    heightthresh=options.heightthresh
                if options.quatthresh is not None:
                    quatthresh=options.quatthresh
            if self.robot.GetRobotStructureHash() == '7b789782446d86b95c6fb16de7f204c7' and self.manip.GetName() == 'arm':
                if heightthresh is None:
                    heightthresh=0.05
                if quatthresh is None:
                    quatthresh=0.15
            self.generate(heightthresh=heightthresh,quatthresh=quatthresh,Nminimum=Nminimum)
            self.save()
        finally:
            for b in bodies:
                b.Enable(True)

    def generate(self,heightthresh=None,quatthresh=None,Nminimum=None):
        """First transform all end effectors to the identity and get the robot positions,
        then cluster the robot position modulo in-plane rotation (z-axis) and position (xy),
        then compute statistics for each cluster."""
        if heightthresh is None:
            heightthresh=0.05
        if quatthresh is None:
            quatthresh=0.15
        if Nminimum is None:
            Nminimum=10
        if not self.rmodel.load():
            self.rmodel.autogenerate()
        print "Generating Inverse Reachability",heightthresh,quatthresh

        # convert the quatthresh to a loose euclidean distance
        self.xyzdelta = self.rmodel.xyzdelta
        self.quatdelta = self.rmodel.quatdelta
        self.rotweight = heightthresh/quatthresh
        quateucdist2 = (1-cos(quatthresh))**2+sin(quatthresh)**2
        # find the density
        basetrans = array(self.rmodel.reachabilitystats)
        if basetrans.shape[1] < 8:
            basetrans = c_[basetrans,ones(basetrans.shape[0])]
        with self.robot:
            Tbase = poseFromMatrix(self.manip.GetBase().GetTransform())
            basetrans[:,0:7] = poseMultArrayT(Tbase,basetrans[:,0:7])
        # find the density of the points
        searchtrans = c_[basetrans[:,0:4],basetrans[:,6:7]]
        kdtree = self.QuaternionKDTree(searchtrans,1.0/self.rotweight)
        transdensity = kdtree.kFRSearchArray(searchtrans,0.25*quateucdist2,0,quatthresh*0.2)[2]
        basetrans = basetrans[argsort(-transdensity),:]
        Nminimum = max(Nminimum,4)
        
        # find all equivalence classes
        quatrolls = array([quatFromAxisAngle(array((0,0,1)),roll) for roll in arange(0,2*pi,quatthresh*0.5)])
        self.equivalenceclasses = []
        while len(basetrans) > 0:
            searchtrans = c_[basetrans[:,0:4],basetrans[:,6:7]]
            kdtree = self.QuaternionKDTree(searchtrans,1.0/self.rotweight)
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
            print 'new equivalence class outliers: %d/%d, left over trans: %d'%(self.testEquivalenceClass(equivalenceclass)*len(zangles),len(zangles),len(basetrans))
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
        logll = quatDist(qnormalized,self.equivalencemeans[:,0:4],self.equivalenceweights[:,0:4]) + (posetarget[6]-self.equivalencemeans[:,4])*self.equivalenceweights[:,4] + self.equivalenceoffset
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
            print 'could not find base distribution: ',logll[bestindex]
            return None,None,None

        # transform the equivalence class to the global coord system and create a kdtree for faster retrieval
        equivalenceclass = self.equivalenceclasses[bestindex]
        points = equivalenceclass[2][:,0:3]
        # transform points by the base and grasp pose
        points[:,0] += zbaseangle+znormangle
        points[:,1:3] = dot(points[:,1:3],transpose(rotationMatrixFromAxisAngle([0,0,1],zbaseangle+znormangle)[0:2,0:2])) + tile(posetarget[4:6]+posebase[4:6], (len(points),1))
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
            return poseMultArrayT(poserobot,c_[cos(samples[:,0]),zeros((N,2)),sin(samples[:,0]),samples[:,1:3],tile(Tbase[2,3],N)])
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
        for Tgrasp,graspindex in Tgrasps:
            posetarget = poseFromMatrix(dot(linalg.inv(Tbase),Tgrasp))
            qnormalized,znormangle = normalizeZRotation(reshape(posetarget[0:4],(1,4)))
            # find the closest cluster
            logll = quatArrayTDist(qnormalized[0],self.equivalencemeans[:,0:4])**2*self.equivalenceweights[:,0] + (posetarget[6]-self.equivalencemeans[:,4])**2*self.equivalenceweights[:,1] + self.equivalenceoffset
            bestindex = argmax(logll)
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
            print 'could not find base distribution'
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
            sampledgraspindices = zeros(N,int)
            sampledpoints = zeros((N,3))
            for i in range(N):
                pointindex = bisect.bisect(cumweights,random.rand())
                sampledgraspindices[i] = graspindices[bisect.bisect(graspindexoffsets,pointindex)-1]
                sampledpoints[i,:] = points[pointindex,:]
            samples = random.normal(sampledpoints,bandwidth*weight)
            samples[:,0] *= 0.5*irotweight
            return poseMultArrayT(poserobot,c_[cos(samples[:,0]),zeros((N,2)),sin(samples[:,0]),samples[:,1:3],tile(Tbase[2,3],N)]),sampledgraspindices
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
            newpoints = c_[equivalenceclass[2][:,0]+znormangle+zbaseangle,dot(equivalenceclass[2][:,1:3],transpose(rotationMatrixFromAxisAngle([0,0,1],znormangle+zbaseangle)[0:2,0:2])) + tile(posetarget[4:6]+posebase[4:6], (len(equivalenceclass[2]),1))]
            newweights = equivalenceclass[2][:,3]*normalizationconst
            if Nprematuresamples > 0:
                newpoints[:,0] *= rotweight
                kdtree = pyANN.KDTree(newpoints)
                cumweights = cumsum(newweights)
                cumweights = cumweights[1:]/cumweights[-1]
                for i in range(Nprematuresamples):
                    sample = random.normal(newpoints[bisect.bisect(cumweights,random.rand()),:],bandwidth*weight)
                    sample[0] *= 0.5*irotweight
                    yield poseMult(poserobot,r_[cos(sample[0]),0,0,sin(sample[0]),sample[1:3],Tbase[2,3]]),graspindex
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
            yield poseMult(poserobot,r_[cos(sample[0]),0,0,sin(sample[0]),sample[1:3],Tbase[2,3]]),sampledgraspindex

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
                yield r_[cos(angle),0,0,sin(angle),Tgrasp[0:2,3] + (bounds[0,1:3]+r[1:3]*dbounds[1:3]),Trobot[2,3]],graspindex
            grasps.append((Tgrasp,graspindex))
        while True:
            Tgrasp,graspindex = grasps[random.randint(len(grasps))]
            r = random.rand(3)
            angle = 0.5*(bounds[0,0]+r[0]*dbounds[0])
            yield r_[cos(angle),0,0,sin(angle),Tgrasp[0:2,3] + (bounds[0,1:3]+r[1:3]*dbounds[1:3]),Trobot[2,3]],graspindex

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
                    poses = samplerfn(N,weight)
                    failures = 0
                    for pose in poses:
                        self.robot.SetTransform(matrixFromPose(pose))
                        if self.manip.FindIKSolution(eye(4),False) is None:
                            #print 'pose failed: ',pose
                            failures += 1
                    print 'height %f, failures: %d'%(height,failures)
                    allfailures.append(failures)
                else:
                    allfailures.append(inf)
        return allfailures

    def testEquivalenceClass(self,equivalenceclass):
        """tests that configurations in the cluster has IK solutions"""
        with self.robot:
            Tgrasp = matrixFromQuat(equivalenceclass[0][0:4])
            Tgrasp[2,3] = equivalenceclass[0][4]
            Tmanipframe = self.robot.GetTransform()
            failed = 0
            for sample in equivalenceclass[2]:
                Tmanip = matrixFromAxisAngle([0,0,1],sample[0])
                Tmanip[0:2,3] = sample[1:3]
                self.robot.SetTransform(dot(Tmanipframe,Tmanip))
                solution = self.manip.FindIKSolution(Tgrasp,False)
                if solution is None:
                    failed += 1
            return float(failed)/len(equivalenceclass[2])
    def getCloseIK(self,Tgrasp):
        with self.env:
            Torggrasp = array(Tgrasp)
            while True:
                solution = self.manip.FindIKSolution(Tgrasp,False)
                if solution is not None:
                    break
                Tgrasp[0:3,3] = Torggrasp[0:3,3] + (random.rand(3)-0.5)*0.01
                Tgrasp[0:3,0:3] = dot(Torggrasp[0:3,0:3],rotationMatrixFromAxisAngle(random.rand(3)*0.05))
            return solution
    def showBaseDistribution(self,densityfn,bounds,zoffset=0,thresh=1.0,maxprob=None,marginalizeangle=True):
        discretization = [0.1,0.04,0.04]
        A,Y,X = mgrid[bounds[0,0]:bounds[1,0]:discretization[0], bounds[0,2]:bounds[1,2]:discretization[2], bounds[0,1]:bounds[1,1]:discretization[1]]
        N = prod(A.shape)
        poses = c_[cos(ravel(A)*0.5),zeros((N,2)),sin(ravel(A)*0.5),X.flat,Y.flat,zeros((N,1))]
        # split it into chunks to avoid memory overflow
        probs = zeros(len(poses))
        for i in range(0,len(poses),500000):
            print '%d/%d'%(i,len(poses))
            probs[i:(i+500000)] = densityfn(poses[i:(i+500000),:]);
        if marginalizeangle:
            probsxy = mean(reshape(probs,(A.shape[0],A.shape[1]*A.shape[2])),axis=0)
            inds = flatnonzero(probsxy>thresh)
            if maxprob is None:
                maxprob = max(probsxy)
            normalizedprobs = numpy.minimum(1,probsxy/maxprob)
            Ic = zeros((X.shape[1],X.shape[2],4))
            Ic[:,:,0] = reshape(normalizedprobs,Ic.shape[0:2])
            Ic[:,:,3] = reshape(normalizedprobs*array(probsxy>thresh,'float'),Ic.shape[0:2])
            Tplane = eye(4)
            Tplane[0:2,3] = mean(bounds[:,1:3],axis=0)
            Tplane[2,3] = zoffset
            return self.env.drawplane(transform=Tplane,extents=(bounds[1,1:3]-bounds[0,1:3])/2,texture=Ic)

        else:
            inds = flatnonzero(probs>thresh)
            if maxprob is None:
                maxprob = max(probs)
            normalizedprobs = numpy.minimum(1,probs[inds]/maxprob)
            colors = c_[0*normalizedprobs,normalizedprobs,normalizedprobs,normalizedprobs]
            points = c_[poses[inds,4:6],A.flatten()[inds]+zoffset]
            return self.env.plot3(points=array(points),colors=array(colors),pointsize=10)
    def showEquivalenceClass(self,equivalenceclass,transparency = 0.8,neighthresh=0.1):
        """Overlays several robots of the same equivalence class"""
        inds = linkstatistics.LinkStatisticsModel.prunePointsKDTree(equivalenceclass[2][:,0:3],neighthresh,1)
        robotlocs = []
        with self.robot:
            Tgrasp = matrixFromQuat(equivalenceclass[0][0:4])
            Tgrasp[2,3] = equivalenceclass[0][4]
            Tmanipframe = dot(self.robot.GetTransform(), linalg.inv(self.manip.GetBase().GetTransform()))
            for sample in equivalenceclass[2][inds,:]:
                Tmanip = matrixFromAxisAngle([0,0,1],sample[0])
                Tmanip[0:2,3] = sample[1:3]
                self.robot.SetTransform(dot(Tmanipframe,Tmanip))
                solution = self.manip.FindIKSolution(Tgrasp,False)
                if solution is not None:
                    self.robot.SetJointValues(solution,self.manip.GetArmJoints())
                    robotlocs.append((self.robot.GetTransform(),self.robot.GetJointValues()))

        self.env.RemoveKinBody(self.robot)
        newrobots = []
        for T,values in robotlocs:
            newrobot = self.env.ReadRobotXMLFile(self.robot.GetXMLFilename())
            newrobot.SetName('robot%d'%random.randint(10000))
            for link in newrobot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
            self.env.AddRobot(newrobot)
            with self.env:
                newrobot.SetTransform(T)
                newrobot.SetJointValues(values)
            newrobots.append(newrobot)
        raw_input('press any key to continue')
        for newrobot in newrobots:
            self.env.RemoveKinBody(newrobot)
        self.env.AddRobot(self.robot)

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.add_option('--heightthresh',action='store',type='float',dest='heightthresh',default=None,
                          help='The max radius of the arm to perform the computation (default=0.05)')
        parser.add_option('--quatthresh',action='store',type='float',dest='quatthresh',default=None,
                          help='The max radius of the arm to perform the computation (default=0.15)')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
           parser = InverseReachabilityModel.CreateOptionParser()
        (options, args) = parser.parse_args()
        Model = lambda robot: InverseReachabilityModel(robot=robot)
        OpenRAVEModel.RunFromParser(Model=Model,parser=parser)

if __name__ == "__main__":
    InverseReachabilityModel.RunFromParser()
