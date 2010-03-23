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
from openravepy import convexdecompositionpy
from numpy import *
import time
from optparse import OptionParser
from itertools import izip

class ConvexDecompositionModel(OpenRAVEModel):
    """Computes the convex decomposition of all of the robot's links"""
    def __init__(self,robot):
        OpenRAVEModel.__init__(self,robot=robot)
        self.linkgeometry = None
        self.convexparams = None

    def has(self):
        return self.linkgeometry is not None and len(self.linkgeometry)==len(self.robot.GetLinks())

    def load(self):
        try:
            params = OpenRAVEModel.load(self)
            if params is None:
                return False
            self.linkgeometry,self.convexparams = params
            if not self.has():
                return False
            with self.env:
                for link,linkcd in izip(self.robot.GetLinks(),self.linkgeometry):
                    for ig,hulls in linkcd:
                        if link.GetGeometries()[ig].IsModifiable():
                            link.GetGeometries()[ig].SetCollisionMesh(self.generateTrimeshFromHulls(hulls))
            return True
        except e:
            return False

    def save(self):
        OpenRAVEModel.save(self,(self.linkgeometry,self.convexparams))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'convexdecomposition.pp')

    def generateFromOptions(self,options):
        args = {'skinWidth':options.skinWidth, 'decompositionDepth':options.decompositionDepth, 'maxHullVertices':options.maxHullVertices, 'concavityThresholdPercent':options.concavityThresholdPercent, 'mergeThresholdPercent':options.mergeThresholdPercent, 'volumeSplitThresholdPercent':options.volumeSplitThresholdPercent, 'useInitialIslandGeneration':options.useInitialIslandGeneration, 'useIslandGeneration':options.useIslandGeneration}
        self.generate(**args)
    def autogenerate(self,forcegenerate=True):
        if False and self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531':
            self.generate()
        else:
            if not forcegenerate:
                raise ValueError('failed to find auto-generation parameters')
            self.generate()
        self.save()
    def generate(self,**kwargs):
        self.convexparams = kwargs
        print 'Generating Convex Decomposition: ',self.convexparams
        starttime = time.time()
        self.linkgeometry = []
        with self.env:
            links = self.robot.GetLinks()
            for il,link in enumerate(links):
                print 'link %d/%d'%(il,len(links))
                geoms = []
                for ig,geom in enumerate(link.GetGeometries()):
                    if geom.GetType() == KinBody.Link.GeomProperties.Type.Trimesh:
                        trimesh = geom.GetCollisionMesh()
                        hulls = convexdecompositionpy.computeConvexDecomposition(trimesh.vertices,trimesh.indices,**self.convexparams)
                        if len(hulls) > 0:
                            geoms.append((ig,hulls))
                self.linkgeometry.append(geoms)
        print 'all convex decomposition finished in %fs'%(time.time()-starttime)
    @staticmethod
    def generateTrimeshFromHulls(hulls):
        allvertices = zeros((0,3),float)
        allindices = zeros((0,3),int)
        for vertices,indices in hulls:
            allindices = r_[allindices,indices+len(allvertices)]
            allvertices = r_[allvertices,vertices]
        return KinBody.Link.TriMesh(allvertices,allindices)
    @staticmethod
    def transformHull(T,hull):
        return dot(hull[0],transpose(T[0:3,0:3]))+tile(T[0:3,3],(len(hull[0]),1)),hull[1]
    def show(self,options=None):
        self.env.SetViewer('qtcoin')
        self.env.UpdatePublishedBodies()
        T = self.env.Triangulate(self.robot)
        print 'total vertices: %d, total triangles: %d'%(len(T.vertices),len(T.indices)/3)
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        handles = []
        jointvalues = tile(inf,self.robot.GetDOF())
        while True:
            newvalues = self.robot.GetJointValues()
            if all(abs(jointvalues-newvalues)<0.01):
                time.sleep(0.5)
                continue
            jointvalues = newvalues
            handles = []
            colorindex = 0
            for ilink,link in enumerate(self.robot.GetLinks()):
                hulls = []
                for ig,geom in enumerate(link.GetGeometries()):
                    cdhulls = [cdhull for ig2,cdhull in self.linkgeometry[ilink] if ig2==ig]
                    if len(cdhulls) > 0:
                        hulls += [self.transformHull(geom.GetTransform(),hull) for hull in cdhulls[0]]
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Box:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeBoxMesh(geom.GetBoxExtents())))
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Sphere:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeGeodesicSphereMesh(geom.GetSphereRadius(),level=1)))
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Cylinder:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeCylinderYMesh(radius=geom.GetCylinderRadius(),height=geom.GetCylinderHeight())))
                handles += [self.env.drawtrimesh(points=transformPoints(link.GetTransform(),hull[0]),indices=hull[1],colors=volumecolors[mod(colorindex+i,len(volumecolors))]) for i,hull in enumerate(hulls)]
                colorindex+=len(hulls)
        raw_input('Press any key to exit: ')

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser(useManipulator=False)
        parser.description='Computes the set of convex hulls for each triangle mesh geometry.using convexdecomposition'
        parser.add_option('--skinWidth',action='store',type='float',dest='skinWidth',default=0.0,
                          help='Skin width on the convex hulls generated (default=%default)')
        parser.add_option('--decompositionDepth',action='store',type='int',dest='decompositionDepth',default=8,
                          help='recursion depth for convex decomposition (default=%default)')
        parser.add_option('--maxHullVertices',action='store',type='int',dest='maxHullVertices',default=64,
                          help='maximum number of vertices in output convex hulls (default=%default)')
        parser.add_option('--concavityThresholdPercent',action='store',type='float',dest='concavityThresholdPercent',default=5.0,
                          help='The percentage of concavity allowed without causing a split to occur (default=%default).')
        parser.add_option('--mergeThresholdPercent',action='store',type='float',dest='mergeThresholdPercent',default=30.0,
                          help='The percentage of volume difference allowed to merge two convex hulls (default=%default).')
        parser.add_option('--volumeSplitThresholdPercent',action='store',type='float',dest='volumeSplitThresholdPercent',default=5.0,
                          help='The percentage of the total volume of the object above which splits will still occur (default=%default).')
        parser.add_option('--useInitialIslandGeneration',action='store',type='int',dest='useInitialIslandGeneration',default=1,
                          help='whether or not to perform initial island generation on the input mesh (default=%default).')
        parser.add_option('--useIslandGeneration',action='store',type='int',dest='useIslandGeneration',default=0,
                          help='Whether or not to perform island generation at each split.  Currently disabled due to bug in RemoveTjunctions (default=%default).')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = ConvexDecompositionModel.CreateOptionParser()
        env = Environment()
        try:
            if Model is None:
                Model = lambda robot: ConvexDecompositionModel(robot=robot)
            OpenRAVEModel.RunFromParser(env=env,Model=Model,parser=parser)
        finally:
            env.Destroy()

if __name__=='__main__':
    ConvexDecompositionModel.RunFromParser()
