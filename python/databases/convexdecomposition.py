#!/usr/bin/env python
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
"""
.. lang-block:: en

  Convex decomposition of the link geometry of the robot.

.. lang-block:: ja

  剛体を複数の凸形状での近似

.. image:: ../../images/databases_convexdecomposition_wam.jpg
  :height: 200

.. image:: ../../images/databases_convexdecomposition_sda10.jpg
  :height: 200

.. image:: ../../images/databases_convexdecomposition_hrp2.jpg
  :height: 200

**Running the Example**

.. code-block:: bash

  openrave.py --database convexdecomposition --robot=robots/barrettsegway.robot.xml


**Showing the Decomposition**

.. code-block:: bash

  openrave.py --database convexdecomposition --robot=robots/barrettsegway.robot.xml --show

Usage
-----

Dynamically load the convex hulls for a robot:

.. code-block:: python

  cdmodel = openravepy.databases.convexdecomposition.ConvexDecompositionModel(robot)
  if not cdmodel.load():
      cdmodel.autogenerate()

.. lang-block:: ja

  **解説**

  凸形状の一つの利点は体積が計算出来る．


Description
-----------

Approximates each of the links with a set of convex hulls using John Ratcliff's `convexdecomposition library`_. 

.. _`convexdecomposition library`: http://codesuppository.blogspot.com/2009/11/convex-decomposition-library-now.html

.. image:: ../../images/databases_convexdecomposition_wamenv.jpg
  :height: 250

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from numpy import *
else:
    from openravepy import OpenRAVEModel
    from numpy import array

from openravepy import convexdecompositionpy
import time, os
from optparse import OptionParser
from itertools import izip

class ConvexDecompositionModel(OpenRAVEModel):
    """Computes the convex decomposition of all of the robot's links"""
    def __init__(self,robot):
        OpenRAVEModel.__init__(self,robot=robot)
        self.linkgeometry = None
        self.convexparams = None
    def clone(self,envother):
        clone = OpenRAVEModel.clone(self,envother)
        #TODO need to set convex decomposition?
        return clone
    def has(self):
        return self.linkgeometry is not None and len(self.linkgeometry)==len(self.robot.GetLinks())
    def getversion(self):
        return 1
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
    def autogenerate(self,options=None):
        if options is not None:
            self.generate(padding=options.padding,skinWidth=options.skinWidth, decompositionDepth=options.decompositionDepth, maxHullVertices=options.maxHullVertices,concavityThresholdPercent=options.concavityThresholdPercent, mergeThresholdPercent=options.mergeThresholdPercent, volumeSplitThresholdPercent=options.volumeSplitThresholdPercent, useInitialIslandGeneration=options.useInitialIslandGeneration, useIslandGeneration=options.useIslandGeneration)
        else:
            self.generate()
        self.save()
    def generate(self,padding=None,**kwargs):
        self.convexparams = kwargs
        if padding is None:
            padding = 0.0
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
                        #hulls = convexdecompositionpy.computeConvexDecomposition(*self.padMesh(trimesh.vertices,trimesh.indices,padding),**self.convexparams)
                        hulls = convexdecompositionpy.computeConvexDecomposition(trimesh.vertices,trimesh.indices,**self.convexparams)
                        if len(hulls) > 0:
                            # add in the padding
                            if padding != 0:
                                hulls = [self.padMesh(hull[0],hull[1],padding) for hull in hulls]
                            geoms.append((ig,hulls))
                self.linkgeometry.append(geoms)
        print 'all convex decomposition finished in %fs'%(time.time()-starttime)
    @staticmethod
    def padMesh(vertices,indices,padding):
        M = mean(vertices,0)
        facenormals = array([cross(vertices[i1]-vertices[i0],vertices[i2]-vertices[i0]) for i0,i1,i2 in indices])
        facenormals *= transpose(tile(1.0/sqrt(sum(facenormals**2,1)),(3,1)))
        # make sure normals are facing outward
        newvertices = zeros((0,3))
        newindices = zeros((0,3))
        originaledges = []
        for i in range(len(facenormals)):
            if dot(vertices[indices[i,0]]-M,facenormals[i]) < 0:
                facenormals[i] *= -1
            offset = len(newvertices)
            newvertices = r_[newvertices,vertices[indices[i,:]]+tile(facenormals[i]*padding,(3,1))]
            newindices = r_[newindices,[[offset,offset+1,offset+2]]]
            for j0,j1 in [[0,1],[0,2],[1,2]]:
                if indices[i,j0] < indices[i,j1]:
                    originaledges.append([indices[i,j0],indices[i,j1],offset+j0,offset+j1])
                else:
                    originaledges.append([indices[i,j1],indices[i,j0],offset+j1,offset+j0])
        # find the connecting edges across the new faces
        originaledges = array(originaledges)
        offset = len(newvertices)
        for i,edge in enumerate(originaledges):
            inds = flatnonzero(logical_and(edge[0]==originaledges[i+1:,0],edge[1]==originaledges[i+1:,1]))
            if len(inds) > 0:
                # add 2 triangles for the edge, and 2 for each vertex
                cedge = originaledges[i+1+inds[0]]
                newindices = r_[newindices,[[edge[2],edge[3],cedge[3]],[edge[2],cedge[3],cedge[2]],[edge[2],cedge[2],offset+edge[0]],[edge[3],cedge[3],offset+edge[1]]]]
        # for every vertex, add a point representing the mean of surrounding extruded vertices
        for i in range(len(vertices)):
            vertexindices = flatnonzero(indices==i)
            newvertices = r_[newvertices,[mean(newvertices[vertexindices],0)]]
        # make sure all faces are facing outward
        for inds in newindices:
            if dot(cross(newvertices[inds[1]]-newvertices[inds[0]],newvertices[inds[2]]-newvertices[inds[0]]),newvertices[inds[0]]-M) < 0:
                inds[1],inds[2] = inds[2],inds[1]
        return newvertices,newindices

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
            newvalues = self.robot.GetDOFValues()
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
                          help='Skin width on the convex hulls generated, convex decomposition side (default=%default)')
        parser.add_option('--padding',action='store',type='float',dest='padding',default=0.005,
                          help='The distance to move the hull planes along their respective normals (default=%default)')
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
    def RunFromParser(Model=None,parser=None,**kwargs):
        """Executes the ConvexDecompositionModel database generation
        """
        if parser is None:
            parser = ConvexDecompositionModel.CreateOptionParser()
        env = Environment()
        try:
            if Model is None:
                Model = lambda robot: ConvexDecompositionModel(robot=robot)
            OpenRAVEModel.RunFromParser(env=env,Model=Model,parser=parser,**kwargs)
        finally:
            env.Destroy()

def run(*args,**kwargs):
    """Executes the convexdecomposition database generation,  ``args`` specifies a list of the arguments to the script.
    
    **Help**
    
    .. shell-block:: openrave.py --database convexdecomposition --help
    """
    ConvexDecompositionModel.RunFromParser(*args,**kwargs)

if __name__=='__main__':
    run()
