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
"""Computes statistics on body links like swept volumes.

.. image:: ../../images/databases/linkstatistics.jpg
  :width: 640

`[source] <../_modules/openravepy/databases/linkstatistics.html>`_

**Running the Generator**

.. code-block:: bash

  openrave.py --database linkstatistics --robot=robots/barrettsegway.robot.xml

**Showing the Swept Volumes**

.. code-block:: bash

  openrave.py --database linkstatistics --robot=robots/barrettsegway.robot.xml --show

Command-line
------------

.. shell-block:: openrave.py --database linkstatistics --help

Description
-----------

When using link statics, it is possible to set the joints weights and resolutions so that planning is fastest. The **xyzdelta** parameter specifies the smallest object that can be found in the environment, this becomes the new discretization factor when checking collision. Higher values mean faster planning.


.. code-block:: python

   lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
   if not lmodel.load():
       lmodel.autogenerate()
   lmodel.setRobotWeights()
   lmodel.setRobotResolutions(xyzdelta=0.01)
   print 'robot resolutions: ',repr(robot.GetDOFResolutions())
   print 'robot weights: ',repr(robot.GetDOFWeights())

Class Definitions
-----------------

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

if not __openravepy_build_doc__:
    from numpy import *
else:
    from numpy import array

import numpy
from ..openravepy_ext import transformPoints, openrave_exception
from ..openravepy_int import RaveFindDatabaseFile, RaveDestroy, Environment, KinBody, rotationMatrixFromQuat, quatRotateDirection, rotationMatrixFromAxisAngle
from . import DatabaseGenerator
from .. import pyANN
import convexdecomposition
from ..misc import ComputeGeodesicSphereMesh, ComputeBoxMesh, ComputeCylinderYMesh, SpaceSamplerExtra
import time
import os.path
from optparse import OptionParser
from itertools import izip
from os import makedirs

import logging
log = logging.getLogger('openravepy.'+__name__.split('.',2)[-1])

class LinkStatisticsModel(DatabaseGenerator):
    """Computes the convex decomposition of all of the robot's links"""
    def __init__(self,robot):
        DatabaseGenerator.__init__(self,robot=robot)
        self.cdmodel = convexdecomposition.ConvexDecompositionModel(self.robot)
        self.linkstats = None
        self.jointvolumes = None
        self.affinevolumes = None # affine volumes for x,y,z translation and rotation around x-,y-,z-axes
        self.samplingdelta = None

    def has(self):
        return self.linkstats is not None and len(self.linkstats)==len(self.robot.GetLinks())

    @staticmethod
    def _GetValue(value):
        if hasattr(value,'value'):
            return value.value
        else:
            return value

    def getversion(self):
        return 4
    
    def save(self):
        try:
            self.SaveHDF5()
        except ImportError:
            log.warn('python h5py library not found, will not be able to speedup database access')
            self.SavePickle()

    def load(self):
        try:
            if not self.cdmodel.load():
                self.cdmodel.autogenerate()

            try:
                return self.LoadHDF5()
            except ImportError:
                log.warn('python h5py library not found, will not be able to speedup database access')
                return self.LoadPickle()
        except Exception, e:
            log.warn(e)
            return False
            
    def SavePickle(self):
        DatabaseGenerator.save(self,(self.linkstats,self.jointvolumes,self.affinevolumes,self.samplingdelta))
        
    def LoadPickle(self):
        params = DatabaseGenerator.load(self)
        if params is None:
            return False
        self.linkstats,self.jointvolumes,self.affinevolumes,self.samplingdelta = params
        return self.has()

    def SaveHDF5(self):
        import h5py
        filename=self.getfilename(False)
        log.info('saving model to %s',filename)
        try:
            makedirs(os.path.split(filename)[0])
        except OSError:
            pass

        f=h5py.File(filename,'w')
        try:
            f['version'] = self.getversion()
            glinkstats = f.create_group('linkstats')
            for i, linkstat in enumerate(self.linkstats):
                glinkstat = glinkstats.create_group(str(i))
                for name,values in linkstat.iteritems():
                    # limitation of h5py
                    if isinstance(values, numpy.ndarray) and len(values) == 0:
                        glinkstat.create_dataset(name,[],dtype=values.dtype)
                    else:
                        glinkstat[name] = values
            gjointvolumes = f.create_group('jointvolumes')
            for i, jointvolume in enumerate(self.jointvolumes):
                gjointvolume = gjointvolumes.create_group(str(i))
                for name,values in jointvolume.iteritems():
                    if isinstance(values, numpy.ndarray) and len(values) == 0:
                        gjointvolume.create_dataset(name,[],dtype=values.dtype)
                    else:
                        gjointvolume[name] = values
            gaffinevolumes = f.create_group('affinevolumes')
            for i, affinevolume in enumerate(self.affinevolumes):
                if affinevolume is not None:
                    gaffinevolume = gaffinevolumes.create_group(str(i))
                    for name,values in affinevolume.iteritems():
                        if isinstance(values, numpy.ndarray) and len(values) == 0:
                            gaffinevolume.create_dataset(name,[],dtype=values.dtype)
                        else:
                            gaffinevolume[name] = values
            f['samplingdelta'] = self.samplingdelta
        finally:
            f.close()

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
                log.error('version is wrong %s!=%s ',f['version'],self.getversion())
                return False

            self.samplingdelta = f['samplingdelta'].value
            glinkstats = f['linkstats']
            self.linkstats = []
            for name,linkstat in glinkstats.iteritems():
                index = int(name)
                while len(self.linkstats) <= index:
                    self.linkstats.append(None)
                self.linkstats[index] = dict(list(linkstat.items()))
            gjointvolumes = f['jointvolumes']
            self.jointvolumes = []
            for name,jointvolume in gjointvolumes.iteritems():
                index = int(name)
                while len(self.jointvolumes) <= index:
                    self.jointvolumes.append(None)
                self.jointvolumes[index] = dict(list(jointvolume.iteritems()))
            gaffinevolumes = f['affinevolumes']
            self.affinevolumes = []
            for name,affinevolume in gaffinevolumes.iteritems():
                index = int(name)
                while len(self.affinevolumes) <= index:
                    self.affinevolumes.append(None)
                self.affinevolumes[index] = dict(list(affinevolume.iteritems()))

            self._databasefile = f
            f = None
            return self.has()
        
        except Exception,e:
            log.debug('LoadHDF5 for %s: ',filename,e)
            return False
        finally:
            if f is not None:
                f.close()
            
    def getfilename(self,read=False):
        return RaveFindDatabaseFile(os.path.join('robot.'+self.robot.GetKinematicsGeometryHash(), 'linkstatistics.pp'),read)

    def setRobotResolutions(self,xyzdelta=0.005):
        """sets the robot resolution
        xyzdelta is the maxdistance allowed to be swept."""
        with self.env:
            for index,joint in enumerate(self.robot.GetJoints()):
                if joint.GetType() == KinBody.Joint.Type.Prismatic:
                    joint.SetResolution(xyzdelta)
                elif not joint.IsStatic():
                    crossarea = self._GetValue(self.jointvolumes[index]['crossarea'])
                    # some cross areas are 0.... bad sampling? opening with hdf5 sometimes has crossarea as a float
                    if isinstance(crossarea,numpy.ndarray) and len(crossarea) > 0:
                        joint.SetResolution(xyzdelta/numpy.max(crossarea[:,0]))
                    else:
                        pass
            self.robot.SetAffineTranslationResolution(tile(xyzdelta,3))
            crossarea = self._GetValue(self.affinevolumes[3+2]['crossarea'])
            self.robot.SetAffineRotationAxisResolution(tile(xyzdelta/numpy.max(crossarea[:,0]),4))

    def setRobotWeights(self,weightexp=0.3333,type=0,weightmult=10.0):
        """sets the weights for the robot.
        weightexp is the exponent for the final weights to help reduce the max:min (default is 1/3 which results in 50:1)
        Weights should be proportional so that equal distances displace the same volume on average.
        """
        with self.env:
            if type == 0:
                linkvolumes = array([self._GetValue(linkstat['volume']) for linkstat in self.linkstats])
                def getweight(ijoint,volumeinfo):
                    if ijoint < 0:
                        accumvolume = sum(linkvolumes)
                    else:
                        accumvolume = sum(array([volume for ilink,volume in enumerate(linkvolumes) if self.robot.DoesAffect(ijoint,ilink)]))
                    weight=(self._GetValue(volumeinfo['volumedelta'])*accumvolume)**weightexp
                    if weight <= 0:
                        log.warn('joint %d has weight=%e, setting to 1e-3'%(ijoint,weight))
                        weight = 1e-3
                    return weight
                
                jweights = array([getweight(ijoint,jv) for ijoint,jv in enumerate(self.jointvolumes)])
                # this is a weird factor.... but at least it is consistent when composing robots
                jweights *= weightmult
                dofweights = []
                for w,j in izip(jweights,self.robot.GetJoints()):
                    dofweights += [w]*j.GetDOF()
                self.robot.SetDOFWeights(dofweights)
                self.robot.SetAffineTranslationWeights([getweight(-1,self.affinevolumes[i]) for i in range(3)])
                self.robot.SetAffineRotationAxisWeights(tile(getweight(-1,self.affinevolumes[3+2]),4)) # only z axis
            elif type == 1:
                # set everything to 1
                for j in self.robot.GetJoints():
                    j.SetWeights(ones(j.GetDOF()))
                self.robot.SetAffineTranslationWeights(ones(3))
                self.robot.SetAffineRotationAxisWeights(ones(4))
            else:
                raise ValueError('no such type')

    def autogenerate(self,options=None):
        samplingdelta=None
        if options is not None:
            if options.samplingdelta is not None:
                samplingdelta=options.samplingdelta
        # compare hashes here
        if self.robot.GetKinematicsGeometryHash() == 'ba2ac00ac66812b08d5c61678d306dcc'or self.robot.GetKinematicsGeometryHash() == '22548f4f2ecf83e88ae7e2f3b2a0bd08': # wam 7dof
            if samplingdelta is None:
                samplingdelta=0.03
        self.generate(samplingdelta=samplingdelta)
        self.save()
    def generate(self,samplingdelta=None,**kwargs):
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        #self.cdmodel.setrobot()
        self.samplingdelta=samplingdelta if samplingdelta is not None else 0.02
        with self.robot:
            self.robot.SetTransform(eye(4))
            # compute the convex hulls for every link
            log.info('Generating link volume points, sampling delta = %f',self.samplingdelta)
            links = self.robot.GetLinks()
            self.linkstats = [None]*len(links)
            for ilink,link,linkcd in izip(range(len(links)),links,self.cdmodel.linkgeometry):
                log.info('link %d/%d',ilink,len(links))
                hulls = []
                for ig,geom in enumerate(link.GetGeometries()):
                    cdhulls = [cdhull for ig2,cdhull in linkcd if ig2==ig]
                    if len(cdhulls) > 0:
                        hulls += [self.cdmodel.transformHull(geom.GetTransform(),hull) for hull in cdhulls[0]]
                    elif geom.GetType() == KinBody.Link.GeomType.Box:
                        hull= self.cdmodel.transformHull(geom.GetTransform(),ComputeBoxMesh(geom.GetBoxExtents()))
                        hulls.append([hull[0],hull[1],self.cdmodel.computeHullPlanes(hull,0.999)])
                    elif geom.GetType() == KinBody.Link.GeomType.Sphere:
                        hull = self.cdmodel.transformHull(geom.GetTransform(),ComputeGeodesicSphereMesh(geom.GetSphereRadius(),level=1))
                        hulls.append([hull[0],hull[1],self.cdmodel.computeHullPlanes(hull,0.999)])
                    elif geom.GetType() == KinBody.Link.GeomType.Cylinder:
                        hull = self.cdmodel.transformHull(geom.GetTransform(),ComputeCylinderYMesh(radius=geom.GetCylinderRadius(),height=geom.GetCylinderHeight()))
                        hulls.append([hull[0],hull[1],self.cdmodel.computeHullPlanes(hull,0.999)])
                self.linkstats[ilink] = self.ComputeGeometryStatistics(hulls)
                
            log.info('Generating swept volumes...')
            self.jointvolumes = [None]*len(self.robot.GetJoints())
            jointvolumes_points = [None]*len(self.robot.GetJoints())
            density = 0.2*self.samplingdelta
            for joint in self.robot.GetDependencyOrderedJoints()[::-1]: # go through all the joints in reverse hierarchical order
                log.info('joint %d',joint.GetJointIndex())
                if joint.GetDOF() > 1:
                    log.info('do not support joints with > 1 DOF')
                lower,upper = joint.GetLimits()
                # final all the directly connected links
                connectedjoints = [joint]+[j for j in self.robot.GetJoints()+self.robot.GetPassiveJoints() if j.IsMimic(0) and joint.GetDOFIndex() in j.GetMimicDOFIndices(0)]
                connectedlinkindices = []
                for j in connectedjoints:
                    if self.robot.DoesAffect(joint.GetJointIndex(),j.GetFirstAttached().GetIndex()):
                        connectedlinkindices.append(j.GetFirstAttached().GetIndex())
                    if self.robot.DoesAffect(joint.GetJointIndex(),j.GetSecondAttached().GetIndex()):
                        connectedlinkindices.append(j.GetSecondAttached().GetIndex())
                connectedlinkindices = unique(connectedlinkindices)
                jointvolume = zeros((0,3))
                for ilink in connectedlinkindices:
                    Tlinkjoint = self.robot.GetLinks()[ilink].GetTransform()
                    Tlinkjoint[0:3,3] -= joint.GetAnchor() # joint anchor should be at center
                    jointvolume = r_[jointvolume, transformPoints(Tlinkjoint,self.linkstats[ilink]['volumepoints'])]
                # gather the swept volumes of all child joints
                for childjoint in self.robot.GetJoints():
                    if childjoint.GetJointIndex() != joint.GetJointIndex() and (childjoint.GetFirstAttached().GetIndex() in connectedlinkindices or childjoint.GetSecondAttached().GetIndex() in connectedlinkindices):
                        jointvolume = r_[jointvolume, self.TransformJointPoints(childjoint,jointvolumes_points[childjoint.GetJointIndex()],translation=-joint.GetAnchor())]
                        jointvolumes_points[childjoint.GetJointIndex()] = None # release since won't be needing it anymore
                sweptvolume = self.ComputeSweptVolume(volumepoints=jointvolume,axis=-joint.GetAxis(0),minangle=lower[0],maxangle=upper[0],samplingdelta=self.samplingdelta)
                # rotate jointvolume so that -joint.GetAxis(0) matches with the z-axis
                R = rotationMatrixFromQuat(quatRotateDirection(-joint.GetAxis(0),[0,0,1]))
                sweptvolume = dot(sweptvolume,transpose(R))
                jointvolume = dot(jointvolume,transpose(R))
                # compute simple statistics and compress the joint volume
                volumecom = mean(sweptvolume,0)
                volume = len(sweptvolume)*self.samplingdelta**3
                log.debug('volume points: %r',sweptvolume.shape)
                if sweptvolume.size > 1:
                    volumeinertia = cov(sweptvolume,rowvar=0,bias=1)*volume
                    # get the cross sections and a dV/dAngle measure
                    crossarea = c_[sqrt(sum(jointvolume[:,0:2]**2,1)),jointvolume[:,2:]]
                else:
                    volumeinertia = zeros((3,3))
                    crossarea = zeros((0,2))
                if len(crossarea) > 0:
                    crossarea = crossarea[self.PrunePointsKDTree(crossarea, density**2, 1,k=50),:]
                    volumedelta = sum(crossarea[:,0])*density**2
                else:
                    volumedelta = 0
                # don't save sweptvolume until we can do it more efficiently
                self.jointvolumes[joint.GetJointIndex()] = {'sweptvolume':sweptvolume, 'crossarea':crossarea,'volumedelta':volumedelta,'volumecom':volumecom,'volumeinertia':volumeinertia,'volume':volume}
                jointvolumes_points[joint.GetJointIndex()] = sweptvolume
                del sweptvolume

            log.info('Computing statistics for the entire robot volume...')
            Trobot = self.robot.GetTransform()
            robotvolume = None
            for link,linkstat in izip(self.robot.GetLinks(),self.linkstats):
                points = transformPoints(dot(linalg.inv(Trobot), link.GetTransform()),linkstat['volumepoints'])
                if robotvolume is None or len(robotvolume) == 0:
                    robotvolume = points
                elif len(points) > 0:
                    kdtree = pyANN.KDTree(robotvolume)
                    neighs,dists,kball = kdtree.kFRSearchArray(points,self.samplingdelta**2,0,self.samplingdelta*0.01)
                    robotvolume = r_[robotvolume, points[kball==0]]
                    del kdtree
            # since jointvolumes were removed as soon as they were used, only consider ones that are still initialized
            for joint,jointvolume in izip(self.robot.GetJoints(),jointvolumes_points):
                if jointvolume is not None:
                    points = self.TransformJointPoints(joint,jointvolume)
                    if len(points) > 1:
                        kdtree = pyANN.KDTree(robotvolume)
                        neighs,dists,kball = kdtree.kFRSearchArray(points,self.samplingdelta**2,0,self.samplingdelta*0.01)
                        robotvolume = r_[robotvolume, points[kball==0]]
                        del kdtree
            del jointvolumes_points # not used anymore, so free memory
            self.affinevolumes = [None]*6
            # compute for rotation around axes
            for i in [2]:
                log.info('rotation %s',('x','y','z')[i])
                axis = array((0,0,0))
                axis[i] = 1.0
                R = rotationMatrixFromQuat(quatRotateDirection(axis,[0,0,1]))
                volume = dot(robotvolume,transpose(R))
                # get the cross sections and a dV/dAngle measure
                crossarea = c_[sqrt(sum(volume[:,0:2]**2,1)),volume[:,2:]]
                crossarea = crossarea[self.PrunePointsKDTree(crossarea, density**2, 1,k=50),:]
                # compute simple statistics and compress the joint volume
                volumedelta = sum(crossarea[:,0])*density**2
                volumecom = r_[tile(mean(crossarea[:,0]),2),mean(crossarea[:,1])]
                volume = sum(crossarea[:,0]**2*pi)*density**2
                self.affinevolumes[3+i] = {'crossarea':crossarea,'volumedelta':volumedelta,'volumecom':volumecom,'volume':volume}
            for i in range(3):
                log.info('translation %s',('x','y','z')[i])
                indices = range(3)
                indices.remove(i)
                crossarea = robotvolume[:,indices]
                crossarea = crossarea[self.PrunePointsKDTree(crossarea, density**2, 1,k=50),:]
                volumedelta = len(crossarea)*density**2
                volumecom = mean(robotvolume,0)
                volume = len(robotvolume)*self.samplingdelta**3
                volumeinertia = cov(robotvolume,rowvar=0,bias=1)*volume
                self.affinevolumes[i] = {'crossarea':crossarea,'volumedelta':volumedelta,'volumecom':volumecom,'volumeinertia':volumeinertia,'volume':volume}

    @staticmethod
    def ComputeSweptVolume(volumepoints,axis,minangle,maxangle,samplingdelta):
        """Compute the swept volume and mesh of volumepoints around rotated around an axis"""
        if len(volumepoints) == 0:
            return zeros((0,3))
        maxradius = sqrt(numpy.max(sum(cross(volumepoints,axis)**2,1)))
        anglerange = maxangle-minangle
        angledelta = samplingdelta/maxradius
        angles = r_[arange(0,anglerange,angledelta),anglerange]
        numangles = len(angles)-1
        volumepoints_pow = [volumepoints]
        sweptvolume = None
        if numangles > 0:
            # compute all points inside the swept volume
            maxbit = int(log2(numangles))
            for i in range(maxbit):
                kdtree = pyANN.KDTree(volumepoints_pow[-1])
                R = rotationMatrixFromAxisAngle(axis,angles[2**i])
                newpoints = dot(volumepoints_pow[-1],transpose(R))
                # only choose points that do not have neighbors
                neighs,dists,kball = kdtree.kFRSearchArray(newpoints,samplingdelta**2,0,samplingdelta*0.01)
                volumepoints_pow.append(r_[volumepoints_pow[-1],newpoints[kball==0]])
            curangle = 0
            for i in range(maxbit+1):
                if numangles&(1<<i):
                    R = rotationMatrixFromAxisAngle(axis,curangle)
                    newpoints = dot(volumepoints_pow[i],transpose(R))
                    if sweptvolume is None:
                        sweptvolume = newpoints
                    else:
                        kdtree = pyANN.KDTree(sweptvolume)
                        neighs,dists,kball = kdtree.kFRSearchArray(newpoints,samplingdelta**2,0,samplingdelta*0.01)
                        sweptvolume = r_[sweptvolume,newpoints[kball==0]]
                    curangle += angles[2**i]
                volumepoints_pow[i] = None # free precious memory
        else:
            sweptvolume = volumepoints_pow[0]
        del volumepoints_pow
        # transform points by minangle since everything was computed ignoring it
        sweptvolume = dot(sweptvolume,transpose(rotationMatrixFromAxisAngle(axis,minangle)))
        return sweptvolume

    @staticmethod
    def TransformJointPoints(joint,points,translation=array((0.0,0.0,0.0))):
        if len(points) > 0:
            Rinv = rotationMatrixFromQuat(quatRotateDirection([0,0,1],-joint.GetAxis(0)))
            return dot(points,transpose(Rinv)) + tile(joint.GetAnchor()+translation,(len(points),1))
        
        return points

    def show(self,options=None):
        if self.env.GetViewer() is None:
            self.env.SetViewer('qtcoin')
            time.sleep(0.4) # give time for viewer to initialize
        for joint in self.robot.GetJoints():
            log.info('joint %d',joint.GetJointIndex())
            haxis = self.env.drawlinestrip(points=vstack((joint.GetAnchor()-2.0*joint.GetAxis(0),joint.GetAnchor()+2.0*joint.GetAxis(0))),linewidth=3.0,colors=array((0.1,0.1,0,1)))
            jv = self.jointvolumes[joint.GetJointIndex()]
            if 'sweptvolume' in jv:
                hvol = self.env.plot3(self.TransformJointPoints(joint,self._GetValue(jv['sweptvolume'])),2,colors=array((0,0,1,0.2)))
            crossarea = self._GetValue(jv['crossarea'])
            harea = self.env.plot3(points=self.TransformJointPoints(joint,c_[crossarea[:,0],zeros(len(crossarea)),crossarea[:,1]]),pointsize=5.0,colors=array((1,0,0,0.3)))
            raw_input('press any key to go to next: ')

    def ComputeGeometryStatistics(self,hulls):
        if len(hulls) == 0:
            return {'com':zeros(3),'inertia':zeros((3,3)),'volume':0,'volumepoints':zeros((0,3))}
        minpoint = numpy.min([numpy.min(hull[0],axis=0) for hull in hulls],axis=0)
        maxpoint = numpy.max([numpy.max(hull[0],axis=0) for hull in hulls],axis=0)
        X,Y,Z = mgrid[minpoint[0]:maxpoint[0]:self.samplingdelta,minpoint[1]:maxpoint[1]:self.samplingdelta,minpoint[2]:maxpoint[2]:self.samplingdelta]
        volumepoints = SpaceSamplerExtra().sampleR3(self.samplingdelta,boxdims=maxpoint-minpoint)
        volumepoints[:,0] += minpoint[0]
        volumepoints[:,1] += minpoint[1]
        volumepoints[:,2] += minpoint[2]
        insidepoints = zeros(len(volumepoints),bool)
        for i,point in enumerate(volumepoints):
            if mod(i,10000) == 0:
                log.debug('%d/%d'%(i,len(volumepoints)))
            for hull in hulls:
                if all(dot(hull[2][:,0:3],point)+hull[2][:,3] <= 0):
                    insidepoints[i] = True
                    break
        volumepoints = volumepoints[insidepoints,:]
        volume = len(volumepoints)*self.samplingdelta**3
        com = mean(volumepoints,0)
        inertia = cov(volumepoints,rowvar=0,bias=1)*(len(volumepoints)*self.samplingdelta**3)
        return {'com':com,'inertia':inertia,'volume':volume,'volumepoints':volumepoints}

    @staticmethod
    def PrunePointsKDTree(points, thresh2, neighsize,k=20):
        """Prunes the poses so that every pose has at most neighsize neighbors within sqrt(thresh2) distance. In order to successfully compute the nearest neighbors, each pose's quaternion is also negated.
        Input:
        thresh2 - squared threshold
        """
        N = points.shape[0]
        k = min(k,N)
        if N <= 1:
            return range(N)
        kdtree = pyANN.KDTree(points)
        while True:
            try:
                allneighs,alldists,kball = kdtree.kFRSearchArray(points,thresh2,k,sqrt(thresh2)*0.01)
                break
            except pyANN.pyann_exception:
                log.error('PrunePointsKDTree: ann memory exceeded. Retrying with less neighbors')
                k = (k+1)/2
            except MemoryError:
                log.error('PrunePointsKDTree: memory error. Retrying with less neighbors')
                k = (k+1)/2
        inds = []
        for i in xrange(N):
            n = neighsize
            for j in xrange(k):
                if allneighs[i,j] < i:
                    if allneighs[i,j] >= 0:
                        n -= 1
                        if n > 0:
                            continue
                    break
            if n > 0:
                inds.append(i)
        dorepeat = any(allneighs[:,-1]>=0)
        del kdtree, allneighs, alldists
        if dorepeat:
            log.debug('repeating pruning... %d/%d',len(inds),points.shape[0])
            newinds = LinkStatisticsModel.PrunePointsKDTree(points[inds,:], thresh2, neighsize,k)
            inds = [inds[i] for i in newinds]
        return inds

    @staticmethod
    def CreateOptionParser():
        parser = DatabaseGenerator.CreateOptionParser(useManipulator=False)
        parser.description='Computes statistics about the link geometry'
        parser.usage='openrave.py --database linkstatistics [options]'
        parser.add_option('--samplingdelta',action='store',type='float',dest='samplingdelta',default=None,
                          help='Skin width on the convex hulls generated (default=0.02)')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None,**kwargs):
        if parser is None:
            parser = LinkStatisticsModel.CreateOptionParser()
        env = Environment()
        try:
            if Model is None:
                Model = lambda robot: LinkStatisticsModel(robot=robot)
            DatabaseGenerator.RunFromParser(env=env,Model=Model,parser=parser,**kwargs)
        finally:
            env.Destroy()
            RaveDestroy()

def run(*args,**kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    LinkStatisticsModel.RunFromParser(*args,**kwargs)
