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
from openravepy.examples import convexdecomposition
from numpy import *
import time
from optparse import OptionParser
from itertools import izip

class LinkStatisticsModel(OpenRAVEModel):
    """Computes the convex decomposition of all of the robot's links"""
    def __init__(self,robot):
        OpenRAVEModel.__init__(self,robot=robot)
        self.cdmodel = convexdecomposition.ConvexDecompositionModel(self.robot)
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        self.linkstats = None

    def has(self):
        return self.linkstats is not None and len(self.linkstats)==len(self.robot.GetLinks())

    def load(self):
        params = OpenRAVEModel.load(self)
        if params is None:
            return False
        self.linkstats = params
        return self.has()

    def save(self):
        OpenRAVEModel.save(self,(self.linkstats))

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
        # gather the convex hulls for every link
        self.linkstats = []
        links = self.robot.GetLinks()
        for i,link,linkcd in izip(range(len(links)),links,self.cdmodel.linkgeometry):
            print 'link %d/%d'%(i,len(links))
            hulls = []
            for ig,geom in enumerate(link.GetGeometries()):
                cdhulls = [cdhull for i,cdhull in linkcd if i==ig]
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
    @staticmethod
    def transformHull(T,hull):
        return dot(hull[0],transpose(T[0:3,0:3]))+tile(T[0:3,3],(len(hull[0]),1)),hull[1]
    @staticmethod
    def computeGeometryStatistics(hulls,delta=0.005):
        minpoint = numpy.min([numpy.min(vertices,axis=0) for vertices,indices in hulls],axis=0)
        maxpoint = numpy.max([numpy.max(vertices,axis=0) for vertices,indices in hulls],axis=0)
        hullplanes = LinkStatisticsModel.computeHullPlanes(hulls)
        X,Y,Z = mgrid[minpoint[0]:maxpoint[0]:delta,minpoint[1]:maxpoint[1]:delta,minpoint[2]:maxpoint[2]:delta]
        allpoints = c_[X.flat,Y.flat,Z.flat]
        insidepoints = zeros(len(allpoints),bool)
        for i,point in enumerate(allpoints):
            if mod(i,10000) == 0:
                print '%d/%d'%(i,len(allpoints))
            for planes in hullplanes:
                if all(dot(planes[:,0:3],point)+planes[:,3] <= 0):
                    insidepoints[i] = True
                    break
        allpoints = allpoints[insidepoints,:]
        volume = len(allpoints)*delta**3
        com = mean(allpoints,0)
        inertia = cov(allpoints,rowvar=0,bias=1)*(len(allpoints)*delta**3)
        return {'com':com,'inertia':inertia,'volume':volume,'points':allpoints}
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
