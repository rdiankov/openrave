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

from numpy import reshape, array, float64, int32, zeros, isnan, newaxis, empty, arange, repeat, where, isclose
from numpy.linalg import norm
from numpy.core.umath_tests import inner1d

from ..misc import ComputeGeodesicSphereMesh, ComputeBoxMesh, ComputeCylinderYMesh
from ..openravepy_int import KinBody, RaveFindDatabaseFile, RaveDestroy, Environment, TriMesh, RaveCreateModule, GeometryType, RaveGetDefaultViewerType
from ..openravepy_ext import transformPoints, transformInversePoints
from . import DatabaseGenerator

import numpy
import time
import os.path
from os import makedirs
from optparse import OptionParser
from itertools import izip

try:
    from cStringIO import StringIO
except:
    from StringIO import StringIO
    
import logging
log = logging.getLogger('openravepy.'+__name__.split('.',2)[-1])

try:
    from .. import convexdecompositionpy
except Exception, e:
    print 'failed to import convexdecompositionpy', e

class ConvexDecompositionError(Exception):
    def __init__(self,msg=u''):
        self.msg = msg
    def __unicode__(self):
        return u'Convex Decomposition Error: %s'%self.msg
    def __str__(self):
        return unicode(self).encode('utf-8')

class ConvexDecompositionModel(DatabaseGenerator):
    """Computes the convex decomposition of all of the robot's links"""
    def __init__(self,robot,padding=0.0):
        """
        :param padding: the desired padding
        """
        DatabaseGenerator.__init__(self,robot=robot)
        self.linkgeometry = None
        self.convexparams = None
        self._padding = padding
        self._graspermodule = None # for convex hulls
        
    def clone(self,envother):
        clone = DatabaseGenerator.clone(self,envother)
        #TODO need to set convex decomposition?
        return clone
    
    def has(self):
        return self.linkgeometry is not None and len(self.linkgeometry)==len(self.robot.GetLinks())
    
    def getversion(self):
        return 2
    
    def save(self):
        try:
            self.SaveHDF5()
        except ImportError:
            log.warn('python h5py library not found, will not be able to speedup database access')
            self.SavePickle()

    def load(self):
        try:
            try:
                return self.LoadHDF5()
            except ImportError:
                log.warn('python h5py library not found, will not be able to speedup database access')
                return self.LoadPickle()
        except Exception, e:
            log.warn(e)
            return False

    def SavePickle(self):
        DatabaseGenerator.save(self,(self.linkgeometry,self.convexparams))

    def SaveHDF5(self):
        import h5py
        filename=self.getfilename(False)
        log.info(u'saving model to %s',filename)
        try:
            makedirs(os.path.split(filename)[0])
        except OSError:
            pass
        
        f=h5py.File(filename,'w')
        try:
            f['version'] = self.getversion()
            gparams = f.create_group('params')
            for name,value in self.convexparams.iteritems():
                gparams[name] = value
            f['padding'] = self._padding
            glinkgeometry = f.create_group('linkgeometry')
            for ilink, linkgeometry in enumerate(self.linkgeometry):
                glink = glinkgeometry.create_group(str(ilink))
                for ig, geometryhulls in linkgeometry:
                    glinkhulls = glink.create_group(str(ig))
                    glinkhulls['igeometry'] = ig
                    ghulls = glinkhulls.create_group('hulls')
                    for j, hull in enumerate(geometryhulls):
                        ghull = ghulls.create_group(str(j))
                        for name,values in izip(['vertices','indices','planes'],hull):
                            if len(values) == 0:
                                ghull.create_dataset(name,[],dtype=values.dtype)
                            else:
                                ghull[name] = values
        finally:
            f.close()

    def LoadPickle(self):
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

    def LoadHDF5(self):
        import h5py
        filename = self.getfilename(True)
        if len(filename) == 0:
            return False
        
        self._CloseDatabase()
        f = None
        try:
            f=h5py.File(filename,'r')
            if f['version'].value != self.getversion():
                log.error(u'version is wrong %s!=%s ',f['version'],self.getversion())
                return False
            
            self.convexparams = {}
            gparams = f['params']
            for name,value in gparams.iteritems():
                self.convexparams[name] = value.value
            self._padding = f['padding'].value
            glinkgeometry = f['linkgeometry']
            self.linkgeometry = []
            for ilink, glink in glinkgeometry.iteritems():
                linkgeometry = []
                for ig, glinkhulls in glink.iteritems():
                    ghulls = glinkhulls['hulls']
                    geometryhulls = []
                    for j, ghull in ghulls.iteritems():
                        if 'vertices' in ghull and len(ghull['vertices'].shape) == 2 and 'indices' in ghull and len(ghull['indices'].shape\
) == 2 and 'planes' in ghull and len(ghull['planes'].shape) == 2:
			    hull = [ghull['vertices'].value, ghull['indices'].value, ghull['planes'].value]
                            geometryhulls.append(hull)
                        else:
                            log.warn('could not open link %s geometry %s hull %s: %r', ilink, ig, j, ghull)
                    linkgeometry.append((int(ig),geometryhulls))
                while len(self.linkgeometry) <= int(ilink):
                    self.linkgeometry.append(None)
                self.linkgeometry[int(ilink)] = linkgeometry
            self._databasefile = f
            f = None
            return self.has()
        
        except Exception,e:
            log.debug(u'LoadHDF5 for %s: ',filename,e)
            return False
        finally:
            if f is not None:
                f.close()

    def setrobot(self):
        with self.env:
            for link,linkcd in izip(self.robot.GetLinks(),self.linkgeometry):
                for ig,hulls in linkcd:
                    if link.GetGeometries()[ig].IsModifiable():
                        link.GetGeometries()[ig].SetCollisionMesh(self.GenerateTrimeshFromHulls(hulls))

    def getfilename(self,read=False):
        filename = 'convexdecomposition_%.3f.pp'%self._padding
        return RaveFindDatabaseFile(os.path.join('robot.'+self.robot.GetKinematicsGeometryHash(), 'convexdecomposition_%.3f.pp'%self._padding),read)
    
    def autogenerate(self,options=None):
        if options is not None:
            self.generate(padding=options.padding,skinWidth=options.skinWidth, decompositionDepth=options.decompositionDepth, maxHullVertices=options.maxHullVertices,concavityThresholdPercent=options.concavityThresholdPercent, mergeThresholdPercent=options.mergeThresholdPercent, volumeSplitThresholdPercent=options.volumeSplitThresholdPercent, useInitialIslandGeneration=options.useInitialIslandGeneration, useIslandGeneration=options.useIslandGeneration,convexHullLinks=options.convexHullLinks.split(','))
        else:
            self.generate()
        self.save()
    def generate(self,padding=None,minTriangleConvexHullThresh=None,convexHullLinks=None,**kwargs):
        """
        :param padding: the padding in meters
        :param minTriangleConvexHullThresh: If not None, then describes the minimum number of triangles needed to use convex hull rather than convex decomposition. Although this might seem counter intuitive, the current convex decomposition module cannot handle really complex meshes and it takes a long time if it does handle them.
        :param convexHullLinks: a list of link names to compute convex hulls instead of decomposition
        """
        self.convexparams = kwargs
        if padding is None:
            padding = self._padding
            if padding is None:
                padding = 0.0
        if convexHullLinks is None:
            convexHullLinks = []
        log.info(u'Generating Convex Decomposition: %r',self.convexparams)
        starttime = time.time()
        self.linkgeometry = []
        with self.env:
            links = self.robot.GetLinks()
            for il,link in enumerate(links):
                geomhulls = []
                geometries = link.GetGeometries()
                for ig,geom in enumerate(geometries):
                    if geom.GetType() == KinBody.Link.GeomType.Trimesh or padding > 0:
                        trimesh = geom.GetCollisionMesh()
                        if len(trimesh.indices) == 0:
                            geom.InitCollisionMesh()
                            trimesh = geom.GetCollisionMesh()
                        if link.GetName() in convexHullLinks or (minTriangleConvexHullThresh is not None and len(trimesh.indices) > minTriangleConvexHullThresh):
                            log.info(u'computing hull for link %d/%d geom %d/%d: vertices=%d, indices=%d',il,len(links), ig, len(geometries), len(trimesh.vertices), len(trimesh.indices))
                            orghulls = [self.ComputePaddedConvexHullFromTriMesh(trimesh,padding)]
                        else:
                            log.info(u'computing decomposition for link %d/%d geom %d/%d type %s',il,len(links), ig, len(geometries), geom.GetType())
                            orghulls = self.ComputePaddedConvexDecompositionFromTriMesh(trimesh,padding)
                        cdhulls = []
                        for hull in orghulls:
                            if any(isnan(hull[0])):
                                raise ConvexDecompositionError(u'geom link %s has NaNs', link.GetName())
                            cdhulls.append((hull[0],hull[1],self.ComputeHullPlanes(hull)))
                        geomhulls.append((ig,cdhulls))
                self.linkgeometry.append(geomhulls)
        self._padding = padding
        log.info(u'all convex decomposition finished in %fs',time.time()-starttime)

    def ComputePaddedConvexDecompositionFromTriMesh(self, trimesh, padding=0.0):
        if len(trimesh.indices) > 0:
            orghulls = convexdecompositionpy.computeConvexDecomposition(trimesh.vertices,trimesh.indices,**self.convexparams)
        else:
            orghulls = []
        if len(orghulls) > 0:
            # add in the padding
            if padding != 0:
                orghulls = [self.PadMesh(hull[0],hull[1],padding) for hull in orghulls]
        return orghulls
    
    def ComputePaddedConvexHullFromTriMesh(self, trimesh, padding=0.0):
        """computes a padded convex hull from all the links and returns it as a list of trimeshes
        """
        if len(trimesh.indices) > 0:
            if self._graspermodule is None:
                self._graspermodule = RaveCreateModule(self.env,'grasper')
                self.env.AddModule(self._graspermodule,self.robot.GetName())
            cmd = StringIO()
            cmd.write('ConvexHull returnplanes 0 returnfaces 0 returntriangles 1 points %d %d '%(len(trimesh.vertices),3))
            for v in trimesh.vertices:
                cmd.write('%.15e %.15e %.15e '%(v[0],v[1],v[2]))
            minpos = numpy.min(trimesh.vertices,axis=0)
            maxpos = numpy.max(trimesh.vertices,axis=0)
            if all(abs(maxpos-minpos) <= 1e-7):
                log.warn('trimesh of %d vertices is very small!', len(trimesh.vertices))
                return zeros((0,3), float), zeros((0,3),int)# trimesh.vertices, trimesh.indices
            res = self._graspermodule.SendCommand(cmd.getvalue()).split()
            if res is None:
                raise ConvexDecompositionError(u'failed to compute convex hull')
            
            offset = 0
            #numplanes = int(res[offset]); offset += 1
            #planes = reshape(array(res[offset:(offset+4*numplanes)], float64), (numplanes,4))
            #offset += 4*numplanes
            numtriangles = int(res[offset]); offset += 1
            return self.PadMesh(trimesh.vertices,reshape(array(res[offset:(offset+3*numtriangles)],int32),(numtriangles,3)), padding)
        
    @staticmethod
    def PadMesh(vertices, indices, padding, mergeDuplicated=True, rtol=1e-05, atol=1e-08, setNormalsAwayFromCenter=False):
        """pads a mesh by increasing towards the normal
        :param mergeDuplicated: This function will return a bad mesh when there are duplicated vertices and mergeDuplicated=False
        :param rtol: The relative tolerance parameter to np.isclose when finding duplicated vertices
        :param atol: The absolute tolerance parameter to np.isclose when finding duplicated vertices
        :param setNormalsAwayFromCenter: if True, then will normalize all normals so that they point away from center-of-geometry.
        """
        M = mean(vertices,0)
        
        # Merge duplicated vertices (+- epsilon)
        indices_raveled = indices.ravel()
        vertices_map = arange(len(vertices), dtype=indices.dtype)
        for a in range(len(vertices) - 1):
            if vertices_map[a] == a:
                combine = a + 1 + flatnonzero(isclose(vertices[a], vertices[a+1:], rtol, atol).all(axis=1))
                vertices_map[combine] = a
        indices = vertices_map[indices.ravel()].reshape(-1, 3)
        
        vertices_0 = vertices[indices[:, 0]]
        facenormals = cross(vertices[indices[:, 1]] - vertices_0, vertices[indices[:, 2]] - vertices_0)
        degenerate = isclose(facenormals, 0.0, rtol, atol).all(axis=1)
        facenormals /= norm(facenormals, axis=1)[:, newaxis]
        # Take care of degenerate triangles. Otherwise their normals would be none
        facenormals[degenerate] = 0.0
        
        # make sure normals are facing outward
        newindices = arange(3 * len(facenormals), dtype=int32).reshape(-1, 3)
        originaledges = empty((3 * len(facenormals), 5), dtype=int32)
        
        if setNormalsAwayFromCenter:
            flip = inner1d(vertices[indices[:,0]] - M, facenormals) < 0
            facenormals[flip] *= -1 # dangerous for hollow objects!
        newvertices = vertices[indices.ravel()] + repeat(facenormals*padding, 3, axis=0)
        
        offsets = arange(0, 3 * len(facenormals), 3)
        indices_j = [indices[:, j] for j in range(3)]
        for j0, j1, n in [[0,1,0],[1,2,1],[2,0,2]]:
            swap = indices_j[j0] < indices_j[j1]
            originaledges_n = originaledges[n::3]
            originaledges_n[:, 0] = where(swap, indices_j[j0], indices_j[j1])
            originaledges_n[:, 1] = where(swap, indices_j[j1], indices_j[j0])
            originaledges_n[:, 2] = offsets + j0#where(swap, j0, j1)
            originaledges_n[:, 3] = offsets + j1#where(swap, j1, j0)
            originaledges_n[:, 4] = swap
        
        # find the connecting edges across the new faces
        offset = 0
        verticesofinterest = {}
        for i,edge in enumerate(originaledges):
            inds = flatnonzero(logical_and(edge[0]==originaledges[i+1:,0],edge[1]==originaledges[i+1:,1]))
            if len(inds) > 0:
                # add 2 triangles for the edge, and 2 for each vertex
                cedge = originaledges[i+1+inds[0]]
                if not edge[0] in verticesofinterest:
                    verticesofinterest[edge[0]] = len(newvertices)+offset
                    offset += 1
                if not edge[1] in verticesofinterest:
                    verticesofinterest[edge[1]] = len(newvertices)+offset
                    offset += 1
                newindices = r_[newindices,[[edge[2],cedge[3],edge[3]],[edge[3],cedge[3],cedge[2]]]]
                if edge[4] == 0:
                    newindices = r_[newindices, [[edge[3],cedge[2],verticesofinterest[edge[0]]],[cedge[3],edge[2],verticesofinterest[edge[1]]]]]
                else:
                    newindices = r_[newindices, [[cedge[3],edge[2],verticesofinterest[edge[0]]],[edge[3],cedge[2],verticesofinterest[edge[1]]]]]
        if offset > 0:
            newvertices = r_[newvertices,zeros((offset,3))]
        # for every vertex, add a point representing the mean of surrounding extruded vertices
        for originalvertex, newvertex in verticesofinterest.iteritems():
            vertexindices = flatnonzero(indices==originalvertex)
            assert(len(vertexindices) > 0)
            newvertices[newvertex,:] = mean(newvertices[vertexindices],0)
        assert(not any(isnan(newvertices)))
        
        # make sure all faces are facing outward
        if setNormalsAwayFromCenter:
            newvertices_0 = newvertices[newindices[:, 0]]
            flip = inner1d(cross(newvertices[newindices[:, 1]] - newvertices_0,
                                 newvertices[newindices[:, 2]] - newvertices_0), newvertices_0 - M) < 0
            for n, inds in enumerate(newindices):
                if flip[n]:
                    inds[1],inds[2] = inds[2],inds[1]
        
        return newvertices,newindices
    
    @staticmethod
    def ComputeHullPlanes(hull,thresh=0.99999):
        """computes the planes of a hull
        
        The computed planes point outside of the mesh. Therefore a point is inside only if the distance to all planes is negative.
        """
        if len(hull[0]) == 0:
            return zeros((0,4),float)
        
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

    def GetGeometryInfosFromLink(self,ilink,preservetransform=False,color=None):
        """gets a list of geometries for the link
        :param preservetransform: if True, will set the same geometry transform as the original geometry. Otherwise will pre-multiply with the new trimesh.
        """
        if color is None:
            color = [0,0,0.4]
        with self.env:
            geometryinfos = []
            geometries = self.robot.GetLinks()[ilink].GetGeometries()
            for ig,hulls in self.linkgeometry[ilink]:
                ginfo = KinBody.GeometryInfo()
                ginfo._vDiffuseColor = color
                ginfo._type = GeometryType.Trimesh
                ginfo._meshcollision = TriMesh()
                if preservetransform:
                    ginfo._t = geometries[ig].GetTransform()
                else:
                    ginfo._t = eye(4)
                numvertices = 0
                numindices = 0
                for hull in hulls:
                    numvertices += len(hull[0])
                    numindices += len(hull[1])
                ginfo._meshcollision.vertices = zeros((numvertices,3),float64)
                ginfo._meshcollision.indices = zeros((numindices,3),int)
                voffset = 0
                ioffset = 0
                for hull in hulls:
                    if preservetransform:
                        ginfo._meshcollision.vertices[voffset:(voffset+len(hull[0])),:] = hull[0]
                    else:
                        ginfo._meshcollision.vertices[voffset:(voffset+len(hull[0])),:] = transformPoints(geometries[ig].GetTransform(),hull[0])
                    ginfo._meshcollision.indices[ioffset:(ioffset+len(hull[1])),:] = hull[1]+voffset
                    voffset += len(hull[0])
                    ioffset += len(hull[1])
                geometryinfos.append(ginfo)
            return geometryinfos
    
    @staticmethod
    def GenerateTrimeshFromHulls(hulls):
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
            self.env.SetViewer(RaveGetDefaultViewerType())
            time.sleep(0.4) # give time for viewer to initialize
        self.env.UpdatePublishedBodies()
        T = self.env.Triangulate(self.robot)
        log.info('total vertices: %d, total triangles: %d',len(T.vertices),len(T.indices)/3)
        volumecolors = array(((1,0,0,0.5),(0,1,0,0.5),(0,0,1,0.5),(0,1,1,0.5),(1,0,1,0.5),(1,1,0,0.5),(0.5,1,0,0.5),(0.5,0,1,0.5),(0,0.5,1,0.5),(1,0.5,0,0.5),(0,1,0.5,0.5),(1,0,0.5,0.5)))
        handles = []
        try:
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
        finally:
            # close all graphs
            handles = None
    def ShowLink(self,ilink,progressive=False):
        if self.env.GetViewer() is None:
            self.env.SetViewer(RaveGetDefaultViewerType())
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
        parser.add_option('--convexHullLinks',action='store',type='str',dest='convexHullLinks',default='',
                          help='comma separated list of link names to compute convex hull for instead')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None,args=None,**kwargs):
        """Executes the ConvexDecompositionModel database generation
        """
        if parser is None:
            parser = ConvexDecompositionModel.CreateOptionParser()
        (options, leftargs) = parser.parse_args(args=args)
        env = Environment()
        try:
            if Model is None:
                Model = lambda robot: ConvexDecompositionModel(robot=robot,padding=options.padding)
            DatabaseGenerator.RunFromParser(env=env,Model=Model,parser=parser,allowkinbody=True,args=args,**kwargs)
        finally:
            env.Destroy()
            RaveDestroy()

def run(*args,**kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    ConvexDecompositionModel.RunFromParser(*args,**kwargs)
