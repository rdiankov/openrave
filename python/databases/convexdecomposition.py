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
"""Convex decomposition of the link geometry of the robot.

.. image:: ../../images/databases/convexdecomposition.jpg
  :width: 640

`[source] <../_modules/openravepy/databases/convexdecomposition.html>`_

**Running the Generator**

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

Description
-----------

Approximates each of the links with a set of convex hulls using John Ratcliff's `convexdecomposition library`_. 

.. _`convexdecomposition library`: http://codesuppository.blogspot.com/2009/11/convex-decomposition-library-now.html

.. image:: ../../images/databases/convexdecomposition_wamenv.jpg
  :height: 250

Command-line
------------

.. shell-block:: openrave.py --database convexdecomposition --help

Class Definitions
-----------------

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2012 Rosen Diankov <rosen.diankov@gmail.com>'
__license__ = 'Apache License, Version 2.0'

if not __openravepy_build_doc__:
    from numpy import *
else:
    from numpy import array

from ..misc import ComputeGeodesicSphereMesh, ComputeBoxMesh, ComputeCylinderYMesh
from ..openravepy_int import KinBody, RaveFindDatabaseFile, RaveDestroy, Environment, TriMesh
from ..openravepy_ext import transformPoints, transformInversePoints
from . import DatabaseGenerator
from .. import convexdecompositionpy

import time
import os.path
from optparse import OptionParser
from itertools import izip

import logging
log = logging.getLogger('openravepy.'+__name__.split('.',2)[-1])

class ConvexDecompositionModel(DatabaseGenerator):
    """Computes the convex decomposition of all of the robot's links"""
    def __init__(self,robot):
        DatabaseGenerator.__init__(self,robot=robot)
        self.linkgeometry = None
        self.convexparams = None
    def clone(self,envother):
        clone = DatabaseGenerator.clone(self,envother)
        #TODO need to set convex decomposition?
        return clone
    def has(self):
        return self.linkgeometry is not None and len(self.linkgeometry)==len(self.robot.GetLinks())
    def getversion(self):
        return 2
    def load(self):
        try:
            params = DatabaseGenerator.load(self)
            if params is None:
                return False
            self.linkgeometry,self.convexparams = params
            if not self.has():
                return False
            return True
        except e:
            return False

    def setrobot(self):
        with self.env:
            for link,linkcd in izip(self.robot.GetLinks(),self.linkgeometry):
                for ig,hulls in linkcd:
                    if link.GetGeometries()[ig].IsModifiable():
                        link.GetGeometries()[ig].SetCollisionMesh(self.generateTrimeshFromHulls(hulls))

    def save(self):
        DatabaseGenerator.save(self,(self.linkgeometry,self.convexparams))

    def getfilename(self,read=False):
        return RaveFindDatabaseFile(os.path.join('robot.'+self.robot.GetKinematicsGeometryHash(), 'convexdecomposition.pp'),read)

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
        log.info('Generating Convex Decomposition: %r',self.convexparams)
        starttime = time.time()
        self.linkgeometry = []
        with self.env:
            links = self.robot.GetLinks()
            for il,link in enumerate(links):
                log.info('link %d/%d',il,len(links))
                geoms = []
                for ig,geom in enumerate(link.GetGeometries()):
                    if geom.GetType() == KinBody.Link.GeomType.Trimesh:
                        trimesh = geom.GetCollisionMesh()
                        if len(trimesh.indices) > 0:
                            orghulls = convexdecompositionpy.computeConvexDecomposition(trimesh.vertices,trimesh.indices,**self.convexparams)
                        else:
                            orghulls = []
                        if len(orghulls) > 0:
                            # add in the padding
                            if padding != 0:
                                orghulls = [self.PadMesh(hull[0],hull[1],padding) for hull in orghulls]
                            geoms.append((ig,[(hull[0],hull[1],self.computeHullPlanes(hull)) for hull in orghulls]))
                self.linkgeometry.append(geoms)
        log.info('all convex decomposition finished in %fs',time.time()-starttime)
    @staticmethod
    def PadMesh(vertices,indices,padding):
        M = mean(vertices,0)
        facenormals = array([cross(vertices[i1]-vertices[i0],vertices[i2]-vertices[i0]) for i0,i1,i2 in indices])
        facenormals *= transpose(tile(1.0/sqrt(sum(facenormals**2,1)),(3,1)))
        # make sure normals are facing outward
        newvertices = zeros((0,3),float64)
        newindices = zeros((0,3),int)
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
    def computeHullPlanes(hull,thresh=0.99999):
        """The computed planes point outside of the mesh. Therefore a point is inside only if the distance to all planes is negative.
        """
        vm = mean(hull[0],0)
        v0 = hull[0][hull[1][:,0],:]
        v1 = hull[0][hull[1][:,1],:]-v0
        v2 = hull[0][hull[1][:,2],:]-v0
        normals = cross(v1,v2,1)/sqrt(sum(v1**2)*sum(v2**2)) # need to multiply by lengths because of floating-point precision problems
        planes = c_[normals,-sum(normals*v0,1)]
        meandist = dot(planes[:,0:3],vm)+planes[:,3]
        planes = r_[planes[flatnonzero(meandist<-1e-7)],-planes[flatnonzero(meandist>1e-7)]]
        if len(planes) == 0:
            return planes
        normalizedplanes = planes/transpose(tile(sqrt(sum(planes**2,1)),(4,1)))
        # prune similar planes
        uniqueplanes = ones(len(planes),bool)
        for i in range(len(normalizedplanes)-1):
            uniqueplanes[i+1:] &= dot(normalizedplanes[i+1:,:],normalizedplanes[i])<thresh
        return planes[uniqueplanes]

    def testPointsInside(self,points):
        """tests if a point is inside the convex mesh of the robot.

        Returns an array the same length as points that specifies whether the point is in or not. This method is not meant to be optimized (use C++ for that).
        """
        with self.env:
            inside = zeros(len(points),bool)
            leftinds = arange(len(points))
            leftpoints = points[:]
            for ilink,link in enumerate(self.robot.GetLinks()):
                if len(leftinds) == 0:
                    break
                for ig,geom in enumerate(link.GetGeometries()):
                    T = dot(link.GetTransform(),geom.GetTransform())
                    localpoints = transformInversePoints(T,leftpoints)
                    cdhulls = [cdhull for ig2,cdhull in self.linkgeometry[ilink] if ig2==ig]
                    if len(cdhulls) > 0:
                        for ihull,hull in enumerate(cdhulls[0]):
                            insideinds = numpy.all(dot(localpoints,transpose(hull[2][:,0:3]))+tile(hull[2][:,3],(len(localpoints),1))<=0,1)
                            inside[leftinds[flatnonzero(insideinds)]] = True
                            outsideinds = flatnonzero(insideinds==0)
                            if len(outsideinds) > 0:
                                leftpoints = leftpoints[outsideinds]
                                leftinds = leftinds[outsideinds]
                                localpoints = localpoints[outsideinds]
                            if len(leftinds) == 0:
                                break
                        continue
                    elif geom.GetType() == KinBody.Link.GeomType.Box:
                        insideinds = numpy.all(numpy.less_equal(abs(localpoints), geom.GetBoxExtents()) ,1)
                    elif geom.GetType() == KinBody.Link.GeomType.Sphere:
                        insideinds = numpy.less_equal(sum(localpoints**2,1), geom.GetSphereRadius()**2)
                    elif geom.GetType() == KinBody.Link.GeomType.Cylinder:
                        insideinds = numpy.less_equal(abs(localpoints[:,1]), 0.5*geom.GetCylinderHeight()) and numpy.less_equal(localpoint[:,0]**2+localpoint[:2]**2, geom.GetCylinderRadius()**2)
                    else:
                        continue
                    inside[leftinds[flatnonzero(insideinds)]] = True
                    outsideinds = flatnonzero(insideinds==0)
                    if len(outsideinds) > 0:
                        leftpoints = leftpoints[outsideinds]
                        leftinds = leftinds[outsideinds]
                    if len(leftinds) == 0:
                        break
        return inside

    @staticmethod
    def generateTrimeshFromHulls(hulls):
        allvertices = zeros((0,3),float64)
        allindices = zeros((0,3),int)
        for hull in hulls:
            allindices = r_[allindices,hull[1]+len(allvertices)]
            allvertices = r_[allvertices,hull[0]]
        return TriMesh(allvertices,allindices)
    
    @staticmethod
    def transformHull(T,hull):
        """hull can be (vertices,indices) or (vertices,indices,planes)
        """
        vertices = dot(hull[0],transpose(T[0:3,0:3]))+tile(T[0:3,3],(len(hull[0]),1))
        if len(hull) == 2:
            return vertices,hull[1]
        return vertices,hull[1],dot(hull[2],linalg.inv(T))
    
    def show(self,options=None):
        if self.env.GetViewer() is None:
            self.env.SetViewer('qtcoin')
            time.sleep(0.4) # give time for viewer to initialize
        self.env.UpdatePublishedBodies()
        T = self.env.Triangulate(self.robot)
        log.info('total vertices: %d, total triangles: %d',len(T.vertices),len(T.indices)/3)
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        handles = []
        jointvalues = None
        while True:
            newvalues = self.robot.GetDOFValues()
            if jointvalues is not None and all(abs(jointvalues-newvalues)<0.01):
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
                    elif geom.GetType() == KinBody.Link.GeomType.Box:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeBoxMesh(geom.GetBoxExtents())))
                    elif geom.GetType() == KinBody.Link.GeomType.Sphere:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeGeodesicSphereMesh(geom.GetSphereRadius(),level=1)))
                    elif geom.GetType() == KinBody.Link.GeomType.Cylinder:
                        hulls.append(self.transformHull(geom.GetTransform(),ComputeCylinderYMesh(radius=geom.GetCylinderRadius(),height=geom.GetCylinderHeight())))
                handles += [self.env.drawtrimesh(points=transformPoints(link.GetTransform(),hull[0]),indices=hull[1],colors=volumecolors[mod(colorindex+i,len(volumecolors))]) for i,hull in enumerate(hulls)]
                colorindex+=len(hulls)
        raw_input('Press any key to exit: ')

    def ShowLink(self,ilink,progressive=False):
        if self.env.GetViewer() is None:
            self.env.SetViewer('qtcoin')
            time.sleep(0.4) # give time for viewer to initialize
        self.env.UpdatePublishedBodies()
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        link = self.robot.GetLinks()[ilink]
        hulls = []
        for ig,geom in enumerate(link.GetGeometries()):
            cdhulls = [cdhull for ig2,cdhull in self.linkgeometry[ilink] if ig2==ig]
            if len(cdhulls) > 0:
                hulls += [self.transformHull(geom.GetTransform(),hull) for hull in cdhulls[0]]
            elif geom.GetType() == KinBody.Link.GeomType.Box:
                hulls.append(self.transformHull(geom.GetTransform(),ComputeBoxMesh(geom.GetBoxExtents())))
            elif geom.GetType() == KinBody.Link.GeomType.Sphere:
                hulls.append(self.transformHull(geom.GetTransform(),ComputeGeodesicSphereMesh(geom.GetSphereRadius(),level=1)))
            elif geom.GetType() == KinBody.Link.GeomType.Cylinder:
                hulls.append(self.transformHull(geom.GetTransform(),ComputeCylinderYMesh(radius=geom.GetCylinderRadius(),height=geom.GetCylinderHeight())))
        try:
            if not progressive:
                handles = [self.env.drawtrimesh(points=transformPoints(link.GetTransform(),hull[0]),indices=hull[1],colors=volumecolors[mod(i,len(volumecolors))]) for i,hull in enumerate(hulls)]
                raw_input('Press any key to exit: ')
            else:
                ihull = 0
                while ihull < len(hulls):
                    hull = hulls[ihull]
                    handles = [self.env.drawtrimesh(points=transformPoints(link.GetTransform(),hull[0]),indices=hull[1],colors=volumecolors[mod(ihull,len(volumecolors))])]
                    cmd = raw_input(str(ihull))
                    if cmd == 'p':
                        ihull -= 1
                    else:
                        ihull += 1
        finally:
            handles = None
            
    @staticmethod
    def CreateOptionParser():
        parser = DatabaseGenerator.CreateOptionParser(useManipulator=False)
        parser.description='Computes the set of convex hulls for each triangle mesh geometry.using convexdecomposition'
        parser.usage='openrave.py --database convexdecomposition [options]'
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
            DatabaseGenerator.RunFromParser(env=env,Model=Model,parser=parser,allowkinbody=True,**kwargs)
        finally:
            env.Destroy()
            RaveDestroy()

def run(*args,**kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    ConvexDecompositionModel.RunFromParser(*args,**kwargs)
