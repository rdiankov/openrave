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
        params = OpenRAVEModel.load(self)
        if params is None:
            return False
        self.linkgeometry,self.convexparams = params
        if not self.has():
            return False
        with self.env:
            for link,linkgeom in izip(self.robot.GetLinks(),self.linkgeometry):
                for ig,hulls in linkgeom:
                    link.GetGeometries()[ig].SetCollisionMesh(self.generateTrimeshFromHulls(hulls))
        return True

    def save(self):
        OpenRAVEModel.save(self,(self.linkgeometry,self.convexparams))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'convexdecomposition.pp')

    def generateFromOptions(self,options):
        args = {'skinWidth':options.skinWidth, 'decompositionDepth':options.decompositionDepth, 'maxHullVertices':maxHullVertices, 'concavityThresholdPercent':concavityThresholdPercent, 'mergeThresholdPercent':mergeThresholdPercent, 'volumeSplitThresholdPercent':volumeSplitThresholdPercent, 'useInitialIslandGeneration':useInitialIslandGeneration, 'useIslandGeneration':useIslandGeneration}
        self.generate(**args)
    def autogenerate(self,forcegenerate=True):
        if self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531':
            self.generate(maxradius=1.1)
        else:
            if not forcegenerate:
                raise ValueError('failed to find auto-generation parameters')
            self.generate()
            raise ValueError('could not auto-generate reachability for %s:%s'%(self.robot.GetName()))
        self.save()
    def generate(self,**kwargs):
        starttime = time.time()
        self.convexparams = kwargs
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
    def show(self):
        self.env.SetViewer('qtcoin')
        self.env.UpdatePublishedBodies()
        T = self.env.Triangulate(self.robot)
        print 'vertices: %d, triangles: %d'%(len(T.vertices),len(T.indices)/3)
        raw_input('Go to View->Geometry->Render/Collision to see render and collision models: ')

    @staticmethod
    def generateTrimeshFromHulls(hulls):
        allvertices = zeros((0,3),float)
        allindices = zeros((0,3),int)
        for vertices,indices in hulls:
            allindices = r_[allindices,indices+len(allvertices)]
            allvertices = r_[allvertices,vertices]
        return KinBody.Link.TriMesh(allvertices,allindices)

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser(useManipulator=False)
        parser.description='Computes the set of convex hulls for each triangle mesh geometry.using convexdecomposition'
        parser.add_option('--skinWidth',action='store',type='float',dest='skinWidth',default=None,
                          help='Skin width on the convex hulls generated')
        parser.add_option('--decompositionDepth',action='store',type='int',dest='decompositionDepth',default=8,
                          help='recursion depth for convex decomposition')
        parser.add_option('--maxHullVertices',action='store',type='int',dest='maxHullVertices',default=64,
                          help='maximum number of vertices in output convex hulls')
        parser.add_option('--concavityThresholdPercent',action='store',type='float',dest='concavityThresholdPercent',default=0.1,
                          help='The percentage of concavity allowed without causing a split to occur.')
        parser.add_option('--mergeThresholdPercent',action='store',type='float',dest='mergeThresholdPercent',default=30.0,
                          help='The percentage of volume difference allowed to merge two convex hulls.')
        parser.add_option('--volumeSplitThresholdPercent',action='store',type='float',dest='volumeSplitThresholdPercent',default=0.1,
                          help='The percentage of the total volume of the object above which splits will still occur.')
        parser.add_option('--useInitialIslandGeneration',action='store',type='int',dest='useInitialIslandGeneration',default=1,
                          help='whether or not to perform initial island generation on the input mesh.')
        parser.add_option('--useIslandGeneration',action='store',type='int',dest='useIslandGeneration',default=0,
                          help='Whether or not to perform island generation at each split.  Currently disabled due to bug in RemoveTjunctions')
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
