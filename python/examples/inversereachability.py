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

import time
try:
   import cPickle as pickle
except:
   import pickle
from openravepy import *
from openravepy.examples import kinematicreachability
import numpy
from numpy import *
from optparse import OptionParser
#from IPython.Debugger import Tracer; debug_here = Tracer()

class InverseReachabilityModel(OpenRAVEModel):
    def __init__(self,robot):
        OpenRAVEModel.__init__(self,robot=robot)
        self.rmodel = kinematicreachability.ReachabilityModel(robot=robot)
        if not self.rmodel.load():
            self.rmodel.autogenerate()
        self.equivalenceclasses = None
        self.rotweight = 0.2 # in-plane rotation weight with respect to xy offset

    def has(self):
        return self.equivalenceclasses is not None and len(self.equivalenceclasses) > 0

    def load(self):
        params = OpenRAVEModel.load(self)
        if params is None:
            return False
        self.equivalenceclasses,self.rotweight = params
        self.preprocess()
        return self.has()

    def preprocess(self):
        self.equivalencemeans = array([e[0] for e in self.equivalenceclasses])
        self.equivalenceweights = array([-0.5/e[1]**2 for e in self.equivalenceclasses])
        self.equivalenceoffset = array([-0.5*pi*len(e[1])-0.5*log(prod(e[1])) for e in self.equivalenceclasses])

    def save(self):
        OpenRAVEModel.save(self,(self.equivalenceclasses,self.rotweight))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'invreachability.' + self.manip.GetName() + '.pp')

    def generateFromOptions(self,options):
        self.generate(xyzthresh=options.xyzthresh,rotthresh=options.rotthresh)

    def autogenerate(self,forcegenerate=True):
        # disable every body but the target and robot
        bodies = [b for b in self.env.GetBodies() if b.GetNetworkId() != self.robot.GetNetworkId()]
        for b in bodies:
            b.Enable(False)
        try:
            if self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531' and self.manip.GetName() == 'arm':
                self.generate(heightthresh=0.05,rotthresh=0.25)
            else:
                if not forcegenerate:
                    raise ValueError('failed to find auto-generation parameters')
                self.generate()
            self.save()
        finally:
            for b in bodies:
                b.Enable(True)

    def generate(self,heightthresh=0.05,rotthresh=0.25):
        # find the density
        basetrans = c_[invertPoses(self.rmodel.reachabilitystats[:,0:7]),self.rmodel.reachabilitystats[:,7:]]
        if basetrans.shape[1] < 8:
            basetrans = c_[basetrans,ones(basetrans.shape[0])]
        # find the density of the points
        self.rotweight = heightthresh/rotthresh
        searchtrans = c_[basetrans[:,0:4],basetrans[:,6:7]/self.rotweight]
        kdtree = pyANN.KDTree(searchtrans)
        transdensity = kdtree.kFRSearchArray(searchtrans,0.25*rotthresh**2,0,rotthresh*0.2)[2]
        basetrans = basetrans[argsort(-transdensity),:]
        
        # find all equivalence classes
        quatrolls = array([quatFromAxisAngle(array((0,0,1)),roll) for roll in arange(0,2*pi,pi/32)])
        self.equivalenceclasses = []
        while len(basetrans) > 0:
            print len(basetrans)
            searchtrans = c_[basetrans[:,0:4],basetrans[:,6:7]/self.rotweight]
            kdtree = pyANN.KDTree(searchtrans)
            foundindices = zeros(len(searchtrans),bool)
            querypoints = c_[self.__quatMultArrayT(searchtrans[0][0:4],quatrolls),tile(searchtrans[0][4:],(len(quatrolls),1))]
            for querypoint in querypoints:
                k = min(len(searchtrans),1000)
                neighs,dists,kball = kdtree.kFRSearch(querypoint,rotthresh**2,k,rotthresh*0.01)
                if k < kball:
                    neighs,dists,kball = kdtree.kFRSearch(querypoint,rotthresh**2,kball,rotthresh*0.01)
                foundindices[neighs] = True
            equivalenttrans = basetrans[flatnonzero(foundindices),:]
            basetrans = basetrans[flatnonzero(foundindices==False),:]
            normalizedquat,zangles = self.normalizeZRotation(equivalenttrans[:,0:4])
            # make sure all quaternions are on the same hemisphere
            identityquat = tile(array((1.0,0,0,0)),(normalizedquat.shape[0],1))
            dists1 = sum( (normalizedquat-identityquat)**2, 1)
            dists2 = sum( (normalizedquat+identityquat)**2,1)
            normalizedquat[flatnonzero(dists2<dists1),0:4] *= -1
            # get statistics and store the angle, xy offset, and remaining unprocessed data
            meanquat = sum(normalizedquat,axis=0)
            meanquat /= sqrt(sum(meanquat**2))
            self.equivalenceclasses.append((r_[meanquat,mean(equivalenttrans[:,6])],
                                            r_[std(normalizedquat,axis=0),std(equivalenttrans[:,6])],
                                            c_[zangles,equivalenttrans[:,4:6],equivalenttrans[:,7:]]))
        self.preprocess()

    def getBaseDistribution(self,Tee,logllthresh=1e6,zaxis=None):
        """Return a function of the distribution of possible positions of the robot that can put their end effector at Tee"""
        if zaxis is not None:
            raise NotImplementedError('cannot specify a custom zaxis yet')
        with self.env:
            posetarget = poseFromMatrix(dot(linalg.inv(Tee),self.robot.GetTransform()))
            qnormalized,zangles = self.normalizeZRotation(reshape(posetarget[0:4],(1,4)))
            # find the closest cluster
            logll = self.quatDist(qnormalized,self.equivalencemeans[:,0:4],self.equivalenceweights[:,0:4]) + (posetarget[6]-self.equivalencemeans[:,4])*self.equivalenceweights[:,4] + self.equivalenceoffset
            bestindex = argmax(logll)
            if logll[bestindex] < logllthresh:
                print 'could not find base distribution: ',logll[bestindex]
                return None
            # transform the equivalence class to the global coord system and create a kdtree for faster retrieval
            equivalenceclass = self.equivalenceclasses[bestindex]
            points = equivalenceclass[2][:,0:3]-tile(r_[zangles,posetarget[4:6]],(equivalenceclass[2].shape[0],1))
            bounds = array((numpy.min(points,0),numpy.max(points,0)))
            points[:,0] *= self.rotweight
            kdtree = pyANN.KDTree(points)
            angdelta = math.acos(1-0.5*self.rmodel.quatdelta) # convert quat dist to angle distance
            bandwidth = array((angdelta*self.rotweight,self.rmodel.xyzdelta,self.rmodel.xyzdelta))
            searchradius=9.0*sum(bandwidth**2)
            searcheps=bandwidth[0]*0.2
            ibandwidth=-0.5/bandwidth**2
            #normalizationconst = (1.0/sqrt(pi**3*sum(bandwidth**2)))
            weights=equivalenceclass[2][:,3]

            def gaussiankernel(Trobot):
                qrobot = quatFromRotationMatrix(Trobot[0:3,0:3])
                qrobotnorm,zangle = self.normalizeZRotation(reshape(qrobot,(1,4)))
                p = array((self.rotweight*zangle[0],Trobot[0,3],Trobot[1,3]))
                neighs,dists,kball = kdtree.kFRSearch(p,searchradius,16,searcheps)
                if len(neighs) == 0:
                    return 0
                return dot(weights[neighs],numpy.exp(dot((kdtree.points[neighs,:]-p)**2,ibandwidth)))
            return gaussiankernel,bounds

    def showBaseDistribution(self,basedistfn):
        basedistfn()

    @staticmethod
    def normalizeZRotation(qarray):
        """for all quaternions, find the rotation about z that minimizes the distance between the identify (1,0,0,0), and transform the quaternions"""
        zangles = arctan2(-qarray[:,3],qarray[:,0])
        sinangles = sin(zangles)
        cosangles = cos(zangles)
        return c_[cosangles*qarray[:,0]-sinangles*qarray[:,3], cosangles*qarray[:,1]-sinangles*qarray[:,2], cosangles*qarray[:,2]+sinangles*qarray[:,1], cosangles*qarray[:,3]+sinangles*qarray[:,0]],-zangles

    @staticmethod
    def __quatMultArrayT(q,qarray):
        """ multiplies a quaternion q with each quaternion in the Nx4 array qarray"""
        return c_[(q[0]*qarray[:,0] - q[1]*qarray[:,1] - q[2]*qarray[:,2] - q[3]*qarray[:,3],
                   q[0]*qarray[:,1] + q[1]*qarray[:,0] + q[2]*qarray[:,3] - q[3]*qarray[:,2],
                   q[0]*qarray[:,2] + q[2]*qarray[:,0] + q[3]*qarray[:,1] - q[1]*qarray[:,3],
                   q[0]*qarray[:,3] + q[3]*qarray[:,0] + q[1]*qarray[:,2] - q[2]*qarray[:,1])]
    @staticmethod
    def quatMult(q1,q2):
        return array((q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
                      q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
                      q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3],
                      q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1]))

    @staticmethod
    def quatDist(q,qarray,weightssq=None):
        """find the squared distance between q and all quaternions in the Nx4 qarray. Optional squared weights can be specified"""
        qtile = tile(q,(qarray.shape[0],1))
        if weightssq is None:
            return minimum(sum( (qtile-qarray)**2, 1), sum( (qtile+qarray)**2,1))
        else:
            q1 = (qtile-qarray)**2
            q2 = (qtile+qarray)**2
            indices = array(sum(q1,axis=1)<sum(q2,axis=1))
            return sum(q1*weightssq,axis=1)*indices+sum(q2*weightssq,axis=1)*(1-indices)
        
    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.add_option('--xyzthresh',action='store',type='float',dest='xyzdelta',default=0.02,
                          help='The max radius of the arm to perform the computation')
        parser.add_option('--rotthresh',action='store',type='float',dest='rotthresh',default=pi/16.0,
                          help='The max radius of the arm to perform the computation')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = kinematicreachability.ReachabilityModel.CreateOptionParser()
        (options, args) = parser.parse_args()
        Model = lambda robot: InverseReachabilityModel(robot=robot)
        OpenRAVEModel.RunFromParser(Model=Model,parser=parser)

if __name__ == "__main__":
    InverseReachabilityModel.RunFromParser()
