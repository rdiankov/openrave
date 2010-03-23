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

import openravepy
from openravepy import *
from openravepy.examples import convexdecomposition,inversekinematics
from numpy import *
import time
import heapq # for nth smallest element
from optparse import OptionParser

class ReachabilityModel(OpenRAVEModel):
    """Computes the robot manipulator's reachability space (stores it in 6D) and
    offers several functions to use it effectively in planning."""
    def __init__(self,robot):
        OpenRAVEModel.__init__(self,robot=robot)
        self.ikmodel = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.cdmodel = convexdecomposition.ConvexDecompositionModel(self.robot)
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        self.reachabilitystats = None
        self.reachability3d = None
        self.reachabilitydensity3d = None
        self.pointscale = None
        self.xyzdelta = None
        self.quatdelta = None

    def has(self):
        return len(self.reachabilitydensity3d) > 0 and len(self.reachability3d) > 0
    def getversion(self):
        return 2
    def load(self):
        try:
            params = OpenRAVEModel.load(self)
            if params is None:
                return False
            self.reachabilitystats,self.reachabilitydensity3d,self.reachability3d,self.pointscale,self.xyzdelta,self.quatdelta = params
            return self.has()
        except e:
            return False
    def save(self):
        OpenRAVEModel.save(self,(self.reachabilitystats,self.reachabilitydensity3d,self.reachability3d, self.pointscale,self.xyzdelta,self.quatdelta))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'reachability.' + self.manip.GetName() + '.pp')

    def generateFromOptions(self,options):
        self.generate(maxradius=options.maxradius,xyzdelta=options.xyzdelta,quatdelta=options.quatdelta)

    def getOrderedArmJoints(self):
        return [j for j in self.robot.GetDependencyOrderedJoints() if j.GetJointIndex() in self.manip.GetArmJoints()]

    def generate(self,maxradius=None,translationonly=False,xyzdelta=0.04,quatdelta=0.5):
        starttime = time.time()
        with self.robot:
            self.robot.SetTransform(eye(4))
            
            # the axes' anchors are the best way to find the max radius
            # the best estimate of arm length is to sum up the distances of the anchors of all the points in between the chain
            armjoints = self.getOrderedArmJoints()
            baseanchor = armjoints[0].GetAnchor()
            eetrans = self.manip.GetEndEffectorTransform()[0:3,3]
            armlength = 0
            for j in armjoints[::-1]:
                armlength += sqrt(sum((eetrans-j.GetAnchor())**2))
                eetrans = j.GetAnchor()    
            if maxradius is None:
                maxradius = armlength+xyzdelta

            allpoints,insideinds,shape,self.pointscale = self.UniformlySampleSpace(maxradius,delta=xyzdelta)
            # select the best sphere level matching quatdelta;
            # level=0, quatdist = 0.5160220
            # level=1: quatdist = 0.2523583
            # level=2: quatdist = 0.120735
            qarray = SpaceSampler().sampleSO3(level=max(0,int(-0.5-log2(quatdelta))))
            rotations = [eye(3)] if translationonly else rotationMatrixFromQArray(qarray)
            self.xyzdelta = xyzdelta
            self.quatdelta = 0
            if not translationonly:
                # for rotations, get the average distance to the nearest rotation
                neighdists = []
                for q in qarray:
                    neighdists.append(heapq.nsmallest(2,quatArrayTDist(q,qarray))[1])
                self.quatdelta = mean(neighdists)
            print 'radius: %f, xyzsamples: %d, quatdelta: %f, rot samples: %d'%(maxradius,len(insideinds),self.quatdelta,len(rotations))
            
            T = eye(4)
            reachabilitydensity3d = zeros(prod(shape))
            reachability3d = zeros(prod(shape))
            self.reachabilitystats = []
            with self.env:
                for i,ind in enumerate(insideinds):
                    numvalid = 0
                    numrotvalid = 0
                    T[0:3,3] = allpoints[ind]+baseanchor
                    for rotation in rotations:
                        T[0:3,0:3] = rotation
                        solutions = self.manip.FindIKSolutions(T,False) # do not want to include the environment
                        if solutions is not None:
                            self.reachabilitystats.append(r_[poseFromMatrix(T),len(solutions)])
                            numvalid += len(solutions)
                            numrotvalid += 1
                    if mod(i,1000)==0:
                        print '%d/%d'%(i,len(insideinds))
                    reachabilitydensity3d[ind] = numvalid/float(len(rotations))
                    reachability3d[ind] = numrotvalid/float(len(rotations))
            self.reachability3d = reshape(reachability3d,shape)
            self.reachabilitydensity3d = reshape(reachabilitydensity3d,shape)
            self.reachabilitystats = array(self.reachabilitystats)
            print 'reachability finished in %fs'%(time.time()-starttime)

    def show(self,showrobot=True,contours=[0.01,0.1,0.2,0.5,0.9,0.99],opacity=None,figureid=1, xrange=None,options=None):
        mlab.figure(figureid,fgcolor=(0,0,0), bgcolor=(1,1,1),size=(1024,768))
        mlab.clf()
        print 'max reachability: ',numpy.max(self.reachability3d)
        if options is not None:
            reachability3d = minimum(self.reachability3d*options.showscale,1.0)
        else:
            reachability3d = minimum(self.reachability3d,1.0)
        reachability3d[0,0,0] = 1 # have at least one point be at the maximum
        if xrange is None:
            offset = array((0,0,0))
            src = mlab.pipeline.scalar_field(reachability3d)
        else:
            offset = array((xrange[0]-1,0,0))
            src = mlab.pipeline.scalar_field(r_[zeros((1,)+reachability3d.shape[1:]),reachability3d[xrange,:,:],zeros((1,)+reachability3d.shape[1:])])
            
        for i,c in enumerate(contours):
            mlab.pipeline.iso_surface(src,contours=[c],opacity=min(1,0.7*c if opacity is None else opacity[i]))
        #mlab.pipeline.volume(mlab.pipeline.scalar_field(reachability3d*100))
        if showrobot:
            baseanchor = armjoints = self.getOrderedArmJoints()[0].GetAnchor()
            with self.robot:
                self.robot.SetTransform(eye(4))
                trimesh = self.env.Triangulate(self.robot)
            v = self.pointscale[0]*(trimesh.vertices-tile(baseanchor,(len(trimesh.vertices),1)))+self.pointscale[1]
            mlab.triangular_mesh(v[:,0]-offset[0],v[:,1]-offset[1],v[:,2]-offset[2],trimesh.indices,color=(0.5,0.5,0.5))
        mlab.show()

    def autogenerate(self,forcegenerate=True):
        # disable every body but the target and robot
        bodies = [b for b in self.env.GetBodies() if b.GetNetworkId() != self.robot.GetNetworkId()]
        for b in bodies:
            b.Enable(False)
        try:
            if self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531' and self.manip.GetName() == 'arm':
                self.generate(maxradius=1.1)
            else:
                if not forcegenerate:
                    raise ValueError('failed to find auto-generation parameters')
                self.generate()
            self.save()
        finally:
            for b in bodies:
                b.Enable(True)

    def UniformlySampleSpace(self,maxradius,delta):
        nsteps = floor(maxradius/delta)
        X,Y,Z = mgrid[-nsteps:nsteps,-nsteps:nsteps,-nsteps:nsteps]
        allpoints = c_[X.flat,Y.flat,Z.flat]*delta
        insideinds = flatnonzero(sum(allpoints**2,1)<maxradius**2)
        return allpoints,insideinds,X.shape,array((1.0/delta,nsteps))

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.description='Computes the reachability region of a robot manipulator and python pickles it into a file.'
        parser.add_option('--maxradius',action='store',type='float',dest='maxradius',default=None,
                          help='The max radius of the arm to perform the computation')
        parser.add_option('--xyzdelta',action='store',type='float',dest='xyzdelta',default=0.04,
                          help='The max radius of the arm to perform the computation (default=%default)')
        parser.add_option('--quatdelta',action='store',type='float',dest='quatdelta',default=0.5,
                          help='The max radius of the arm to perform the computation (default=%default)')
        parser.add_option('--showscale',action='store',type='float',dest='showscale',default=1.0,
                          help='Scales the reachability by this much in order to show colors better (default=%default)')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = ReachabilityModel.CreateOptionParser()
        (options, args) = parser.parse_args()
        env = Environment()
        try:
            if Model is None:
                Model = lambda robot: ReachabilityModel(robot=robot)
            OpenRAVEModel.RunFromParser(env=env,Model=Model,parser=parser)
        finally:
            env.Destroy()

if __name__=='__main__':
    parser = ReachabilityModel.CreateOptionParser()
    (options, args) = parser.parse_args()
    if options.show: # only load mayavi if showing
        try:
            from enthought.mayavi import mlab
        except ImportError:
            pass
    ReachabilityModel.RunFromParser()
