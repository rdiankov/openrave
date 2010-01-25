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

from openravepy import *
from openravepy import pyANN
from openravepy.examples import convexdecomposition
from numpy import *
import time
from optparse import OptionParser
from itertools import izip

try:
    from enthought.tvtk.api import tvtk
except ImportError:
    pass

class LinkStatisticsModel(OpenRAVEModel):
    """Computes the convex decomposition of all of the robot's links"""
    def __init__(self,robot):
        OpenRAVEModel.__init__(self,robot=robot)
        self.cdmodel = convexdecomposition.ConvexDecompositionModel(self.robot)
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        self.linkstats = None
        self.samplingdelta = 0.005

    def has(self):
        return self.linkstats is not None and len(self.linkstats)==len(self.robot.GetLinks())

    def load(self):
        try:
            params = OpenRAVEModel.load(self)
            if params is None:
                return False
            self.linkstats,self.samplingdelta = params
            return self.has()
        except e:
            return False
    def save(self):
        OpenRAVEModel.save(self,(self.linkstats,self.samplingdelta))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'linkstatistics.pp')

    def generateFromOptions(self,options):
        args = {}
        self.generate(**args)
    def autogenerate(self,forcegenerate=True):
        if self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531':
            self.generate()
        else:
            if not forcegenerate:
                raise ValueError('failed to find auto-generation parameters')
            self.generate()
            raise ValueError('could not auto-generate reachability for %s:%s'%(self.robot.GetName()))
        self.save()
    def generate(self,**kwargs):
        with self.robot:
            self.robot.SetTransform(eye(4))
            # compute the convex hulls for every link
            print 'Generating link volume points...'
            self.linkstats = []
            links = self.robot.GetLinks()
            for ilink,link,linkcd in izip(range(len(links)),links,self.cdmodel.linkgeometry):
                print 'link %d/%d'%(ilink,len(links))
                hulls = []
                for ig,geom in enumerate(link.GetGeometries()):
                    cdhulls = [cdhull for ilink,cdhull in linkcd if ilink==ig]
                    if len(cdhulls) > 0:
                        hulls += [self.transformHull(geom.GetTransform(),hull) for hull in cdhulls[0]]
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Box:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeBoxMesh(geom.GetBoxExtents())))
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Sphere:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeGeodesicSphereMesh(geom.GetSphereRadius(),level=1)))
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Cylinder:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeCylinderYMesh(radius=geom.GetCylinderRadius(),height=geom.GetCylinderHeight())))
                    else:
                        raise ValueError('unknown geometry type %s'%str(geom.GetType()))
                self.linkstats.append(self.computeGeometryStatistics(hulls))

            print 'Generating link/joint swept volumes...'
            for linkstat in self.linkstats:
                linkstat['sweptvolumes'] = []            
            for ijoint,joint in enumerate(self.robot.GetJoints()):
                print 'joint %d/%d'%(ijoint,len(self.robot.GetJoints()))
                lower,upper = joint.GetLimits()
                if joint.GetDOF() > 1:
                    print 'do not support joints with > 1 DOF'
                for ilink,linkstat in enumerate(self.linkstats):
                    if self.robot.DoesAffect(ijoint,ilink):
                        sweptpoints,sweptindices = self.computeSweptVolume(volumepoints=linkstat['volumepoints'],axis=joint.GetAxis(0),minangle=lower[0],maxangle=upper[0])
                        self.linkstats[ilink]['sweptvolumes'].append((ijoint,sweptpoints,sweptindices))

    def computeSweptVolume(self,volumepoints,axis,minangle,maxangle):
        """Compute the swept volume and mesh of volumepoints around rotated around an axis"""
        maxradius = sqrt(numpy.max(sum(cross(volumepoints,axis)**2,1)))
        anglerange = maxangle-minangle
        angledelta = self.samplingdelta/maxradius
        angles = r_[arange(0,anglerange,angledelta),anglerange]
        volumepoints_pow = [volumepoints]
        maxbit = int(log2(len(angles)))
        for i in range(maxbit+1):
            kdtree = pyANN.KDTree(volumepoints_pow[-1])
            R = rotationMatrixFromAxisAngle(axis,angles[2**i])
            newpoints = dot(volumepoints_pow[-1],transpose(R))
            # only choose points that do not have neighbors
            neighs,dists,kball = kdtree.kFRSearchArray(newpoints,self.samplingdelta**2,0,self.samplingdelta*0.01)
            volumepoints_pow.append(r_[volumepoints_pow[-1],newpoints[kball==0]])
        # compute all points inside the swept volume
        sweptvolume = None
        curangle = 0
        for i in range(maxbit+1):
            if len(angles)&(1<<i):
                R = rotationMatrixFromAxisAngle(axis,curangle)
                newpoints = dot(volumepoints_pow[i+1],transpose(R))
                if sweptvolume is None:
                    sweptvolume = newpoints
                else:
                    kdtree = pyANN.KDTree(sweptvolume)
                    neighs,dists,kball = kdtree.kFRSearchArray(newpoints,self.samplingdelta**2,0,self.samplingdelta*0.01)
                    sweptvolume = r_[sweptvolume,newpoints[kball==0]]
                curangle += angles[1<<i]
        # compute the isosurface
        minpoint = numpy.min(sweptvolume,0)-2.0*self.samplingdelta
        maxpoint = numpy.max(sweptvolume,0)+2.0*self.samplingdelta
        N = ceil(maxpoint-minpoint)/self.samplingdelta
        X,Y,Z = mgrid[minpoint[0]:maxpoint[0]:self.samplingdelta,minpoint[1]:maxpoint[1]:self.samplingdelta,minpoint[2]:maxpoint[2]:self.samplingdelta]
        kdtree = pyANN.KDTree(sweptvolume)
        neighthresh = self.samplingdelta
        neighs,dists,kball = kdtree.kFRSearchArray(c_[X.flat,Y.flat,Z.flat],neighthresh**2,0,self.samplingdelta*0.01)
        sweptdata = reshape(array(kball>0,'float'),X.shape)
        id = tvtk.ImageData(origin=minpoint,spacing=array((self.samplingdelta,self.samplingdelta,self.samplingdelta)),dimensions=sweptdata.shape[::-1])
        id.point_data.scalars = sweptdata.ravel()
        m = tvtk.MarchingCubes()
        m.set_input(id)
        m.set_value(0,0.5)
        m.update()
        o = m.get_output()
        sweptpoints = array(o.points)
        sweptindices = reshape(array(o.polys.data,'int'),(len(o.polys.data)/4,4))[:,1:4] # first column is usually 3 (for 3 points per tri)
        #h = self.env.plot3(points=sweptpoints,pointsize=2.0,colors=array((1.0,0,0)))
        #h2 = self.env.drawtrimesh (points=sweptpoints,indices=sweptindices,colors=array((0,0,1,0.5)))
        return sweptpoints,sweptindices

    def computeGeometryStatistics(self,hulls):
        minpoint = numpy.min([numpy.min(vertices,axis=0) for vertices,indices in hulls],axis=0)
        maxpoint = numpy.max([numpy.max(vertices,axis=0) for vertices,indices in hulls],axis=0)
        hullplanes = LinkStatisticsModel.computeHullPlanes(hulls)
        X,Y,Z = mgrid[minpoint[0]:maxpoint[0]:self.samplingdelta,minpoint[1]:maxpoint[1]:self.samplingdelta,minpoint[2]:maxpoint[2]:self.samplingdelta]
        volumepoints = SpaceSampler().sampleR3(self.samplingdelta,maxpoint-minpoint)
        volumepoints[:,0] += minpoint[0]
        volumepoints[:,1] += minpoint[1]
        volumepoints[:,2] += minpoint[2]
        insidepoints = zeros(len(volumepoints),bool)
        for i,point in enumerate(volumepoints):
            if mod(i,10000) == 0:
                print '%d/%d'%(i,len(volumepoints))
            for planes in hullplanes:
                if all(dot(planes[:,0:3],point)+planes[:,3] <= 0):
                    insidepoints[i] = True
                    break
        volumepoints = volumepoints[insidepoints,:]
        volume = len(volumepoints)*self.samplingdelta**3
        com = mean(volumepoints,0)
        inertia = cov(volumepoints,rowvar=0,bias=1)*(len(volumepoints)*self.samplingdelta**3)
        return {'com':com,'inertia':inertia,'volume':volume,'volumepoints':volumepoints}

    @staticmethod
    def transformHull(T,hull):
        return dot(hull[0],transpose(T[0:3,0:3]))+tile(T[0:3,3],(len(hull[0]),1)),hull[1]
    @staticmethod
    def computeHullPlanes(hulls):
        hullplanes = [] # planes point outward
        for vertices,indices in hulls:
            vm = mean(vertices,0)
            v0 = vertices[indices[:,0],:]
            v1 = vertices[indices[:,1],:]
            v2 = vertices[indices[:,2],:]
            normals = cross(v1-v0,v2-v0,1)
            planes = c_[normals,-sum(normals*v0,1)]
            planes *= transpose(tile(-sign(dot(planes,r_[vm,1])),(4,1)))
            normalizedplanes = planes/transpose(tile(sqrt(sum(planes**2,1)),(4,1)))
            # prune similar planes
            uniqueplanes = ones(len(planes),bool)
            for i in range(len(normalizedplanes)-1):
                uniqueplanes[i+1:] &= dot(normalizedplanes[i+1:,:],normalizedplanes[i])<0.999
            hullplanes.append(planes[uniqueplanes])
        return hullplanes
    def show(self,options=None):
        self.env.SetViewer('qtcoin')
        self.env.UpdatePublishedBodies()
        T = self.env.Triangulate(self.robot)
        print 'vertices: %d, triangles: %d'%(len(T.vertices),len(T.indices)/3)
        raw_input('Go to View->Geometry->Render/Collision to see render and collision models: ')

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser(useManipulator=False)
        parser.description='Computes statistics about the link geometry'
#         parser.add_option('--skinWidth',action='store',type='float',dest='skinWidth',default=None,
#                           help='Skin width on the convex hulls generated')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = LinkStatisticsModel.CreateOptionParser()
        env = Environment()
        try:
            if Model is None:
                Model = lambda robot: LinkStatisticsModel(robot=robot)
            OpenRAVEModel.RunFromParser(env=env,Model=Model,parser=parser)
        finally:
            env.Destroy()

if __name__=='__main__':
    LinkStatisticsModel.RunFromParser()
