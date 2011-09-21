
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

**Showing the Reachability** (uses mayavi2)

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
    from ..openravepy_int import *
    from ..openravepy_ext import *
    from numpy import *
else:
    from numpy import array

from . import DatabaseGenerator
from .. import pyANN
import convexdecomposition
from ..misc import ComputeGeodesicSphereMesh, ComputeBoxMesh, ComputeCylinderYMesh, SpaceSamplerExtra
import time
import os.path
from optparse import OptionParser
from itertools import izip

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
    def load(self):
        try:
            if not self.cdmodel.load():
                self.cdmodel.autogenerate()
            params = DatabaseGenerator.load(self)
            if params is None:
                return False

            self.linkstats,self.jointvolumes,self.affinevolumes,self.samplingdelta = params
            return self.has()
        except e:
            return False

    def save(self):
        DatabaseGenerator.save(self,(self.linkstats,self.jointvolumes,self.affinevolumes,self.samplingdelta))

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
                    crossarea = self.jointvolumes[index]['crossarea']
                    # some cross areas are 0.... bad sampling?
                    if len(crossarea) > 0:
                        joint.SetResolution(xyzdelta/numpy.max(crossarea[:,0]))
            self.robot.SetAffineTranslationResolution(tile(xyzdelta,3))
            self.robot.SetAffineRotationAxisResolution(tile(xyzdelta/numpy.max(self.affinevolumes[3+2]['crossarea'][:,0]),4))

    def setRobotWeights(self,weightexp=0.3333,type=0,weightmult=10.0):
        """sets the weights for the robot.
        weightexp is the exponent for the final weights to help reduce the max:min (default is 1/3 which results in 50:1)
        Weights should be proportional so that equal distances displace the same volume on average.
        """
        with self.env:
            if type == 0:
                linkvolumes = array([linkstat['volume'] for linkstat in self.linkstats])
                def getweight(ijoint,volumeinfo):
                    if ijoint < 0:
                        accumvolume = sum(linkvolumes)
                    else:
                        accumvolume = sum(array([volume for ilink,volume in enumerate(linkvolumes) if self.robot.DoesAffect(ijoint,ilink)]))
                    return (volumeinfo['volumedelta']*accumvolume)**weightexp
                jweights = array([getweight(ijoint,jv) for ijoint,jv in enumerate(self.jointvolumes)])
                # this is a weird factor.... but at least it is consistent when composing robots
                jweights *= weightmult
                for w,j in izip(jweights,self.robot.GetJoints()):
                    j.SetWeights(tile(w,j.GetDOF()))
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
            print 'Generating link volume points, sampling delta = %f'%self.samplingdelta
            links = self.robot.GetLinks()
            self.linkstats = [None]*len(links)
            for ilink,link,linkcd in izip(range(len(links)),links,self.cdmodel.linkgeometry):
                print 'link %d/%d'%(ilink,len(links))
                hulls = []
                for ig,geom in enumerate(link.GetGeometries()):
                    cdhulls = [cdhull for ig2,cdhull in linkcd if ig2==ig]
                    if len(cdhulls) > 0:
                        hulls += [self.cdmodel.transformHull(geom.GetTransform(),hull) for hull in cdhulls[0]]
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Box:
                        hull= self.cdmodel.transformHull(geom.GetTransform(),ComputeBoxMesh(geom.GetBoxExtents()))
                        hulls.append([hull[0],hull[1],self.cdmodel.computeHullPlanes(hull,0.999)])
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Sphere:
                        hull = self.cdmodel.transformHull(geom.GetTransform(),ComputeGeodesicSphereMesh(geom.GetSphereRadius(),level=1))
                        hulls.append([hull[0],hull[1],self.cdmodel.computeHullPlanes(hull,0.999)])
                    elif geom.GetType() == KinBody.Link.GeomProperties.Type.Cylinder:
                        hull = self.cdmodel.transformHull(geom.GetTransform(),ComputeCylinderYMesh(radius=geom.GetCylinderRadius(),height=geom.GetCylinderHeight()))
                        hulls.append([hull[0],hull[1],self.cdmodel.computeHullPlanes(hull,0.999)])
                self.linkstats[ilink] = self.computeGeometryStatistics(hulls)
                
            print 'Generating swept volumes...'
            self.jointvolumes = [None]*len(self.robot.GetJoints())
            jointvolumes_points = [None]*len(self.robot.GetJoints())
            density = 0.2*self.samplingdelta
            for joint in self.robot.GetDependencyOrderedJoints()[::-1]: # go through all the joints in reverse hierarchical order
                print 'joint %d'%joint.GetJointIndex()
                if joint.GetDOF() > 1:
                    print 'do not support joints with > 1 DOF'
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
                        jointvolume = r_[jointvolume, self.transformJointPoints(childjoint,jointvolumes_points[childjoint.GetJointIndex()],translation=-joint.GetAnchor())]
                        jointvolumes_points[childjoint.GetJointIndex()] = None # release since won't be needing it anymore
                sweptpoints,sweptindices,sweptvolume = self.computeSweptVolume(volumepoints=jointvolume,axis=-joint.GetAxis(0),minangle=lower[0],maxangle=upper[0],samplingdelta=self.samplingdelta)
                # rotate jointvolume so that -joint.GetAxis(0) matches with the z-axis
                R = rotationMatrixFromQuat(quatRotateDirection(-joint.GetAxis(0),[0,0,1]))
                sweptvolume = dot(sweptvolume,transpose(R))
                sweptpoints = dot(sweptpoints,transpose(R))
                # compute simple statistics and compress the joint volume
                volumecom = mean(sweptvolume,0)
                volume = len(sweptvolume)*self.samplingdelta**3
                volumeinertia = cov(sweptvolume,rowvar=0,bias=1)*volume
                # get the cross sections and a dV/dAngle measure
                crossarea = c_[sqrt(sum(jointvolume[:,0:2]**2,1)),jointvolume[:,2:]]
                if len(crossarea) > 0:
                    crossarea = crossarea[self.prunePointsKDTree(crossarea, density**2, 1,k=50),:]
                    volumedelta = sum(crossarea[:,0])*density**2
                else:
                    volumedelta = 0
                self.jointvolumes[joint.GetJointIndex()] = {'sweptpoints':sweptpoints,'sweptindices':sweptindices,'crossarea':crossarea,'volumedelta':volumedelta,'volumecom':volumecom,'volumeinertia':volumeinertia,'volume':volume}
                jointvolumes_points[joint.GetJointIndex()] = sweptvolume
                del sweptvolume

            print 'Computing statistics for the entire robot volume...'
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
                    points = self.transformJointPoints(joint,jointvolume)
                    if len(points) > 1:
                        kdtree = pyANN.KDTree(robotvolume)
                        neighs,dists,kball = kdtree.kFRSearchArray(points,self.samplingdelta**2,0,self.samplingdelta*0.01)
                        robotvolume = r_[robotvolume, points[kball==0]]
                        del kdtree
            del jointvolumes_points # not used anymore, so free memory
            self.affinevolumes = [None]*6
            # compute for rotation around axes
            for i in [2]:
                print 'rotation %s'%(('x','y','z')[i])
                axis = array((0,0,0))
                axis[i] = 1.0
                # get the cross sections and a dV/dAngle measure
                crossarea = c_[sqrt(sum(robotvolume[:,0:2]**2,1)),robotvolume[:,2:]]
                crossarea = crossarea[self.prunePointsKDTree(crossarea, density**2, 1,k=50),:]
                # compute simple statistics and compress the joint volume
                volumedelta = sum(crossarea[:,0])*density**2
                volumecom = r_[tile(mean(crossarea[:,0]),2),mean(crossarea[:,1])]
                volume = sum(crossarea[:,0]**2*pi)*density**2
                self.affinevolumes[3+i] = {'crossarea':crossarea,'volumedelta':volumedelta,'volumecom':volumecom,'volume':volume}
            for i in range(3):
                print 'translation %s'%(('x','y','z')[i])
                indices = range(3)
                indices.remove(i)
                crossarea = robotvolume[:,indices]
                crossarea = crossarea[self.prunePointsKDTree(crossarea, density**2, 1,k=50),:]
                volumedelta = len(crossarea)*density**2
                volumecom = mean(robotvolume,0)
                volume = len(robotvolume)*self.samplingdelta**3
                volumeinertia = cov(robotvolume,rowvar=0,bias=1)*volume
                self.affinevolumes[i] = {'crossarea':crossarea,'volumedelta':volumedelta,'volumecom':volumecom,'volumeinertia':volumeinertia,'volume':volume}

    @staticmethod
    def computeSweptVolume(volumepoints,axis,minangle,maxangle,samplingdelta):
        """Compute the swept volume and mesh of volumepoints around rotated around an axis"""
        if len(volumepoints) == 0:
            return zeros((0,3)),zeros(0,int),zeros((0,3))
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
        sweptpoints,sweptindices = LinkStatisticsModel.computeIsosurface(sweptvolume,samplingdelta*2.0,0.5)
        #h1 = self.env.plot3(points=sweptpoints,pointsize=2.0,colors=array((1.0,0,0)))
        #h2 = self.env.drawtrimesh (points=sweptpoints,indices=sweptindices,colors=array((0,0,1,0.5)))
        return sweptpoints,sweptindices,sweptvolume

    @staticmethod
    def computeIsosurface(sweptvolume,samplingdelta,thresh=0.1):
        """computes the isosurface"""
        api = __import__('enthought.tvtk.api',fromlist=['tvtk']) # for isosurface computation, don't put this at the top of the py file
        minpoint = numpy.min(sweptvolume,0)-2.0*samplingdelta
        maxpoint = numpy.max(sweptvolume,0)+2.0*samplingdelta
        volumeshape = array(ceil((maxpoint-minpoint)/samplingdelta),'int')
        indices = array((sweptvolume-tile(minpoint,(len(sweptvolume),1)))*(1.0/samplingdelta)+0.5,int)
        sweptdata = zeros(prod(volumeshape))
        sweptdata[indices[:,0]+volumeshape[0]*(indices[:,1]+volumeshape[1]*indices[:,2])] = 1
        id = api.tvtk.ImageData(origin=minpoint,spacing=array((samplingdelta,samplingdelta,samplingdelta)),dimensions=volumeshape)
        id.point_data.scalars = sweptdata.ravel()
        m = api.tvtk.MarchingCubes()
        m.set_input(id)
        m.set_value(0,thresh)
        m.update()
        o = m.get_output()
        sweptpoints = array(o.points)
        sweptindices = reshape(array(o.polys.data,'int'),(len(o.polys.data)/4,4))[:,1:4] # first column is usually 3 (for 3 points per tri)
        return sweptpoints,sweptindices

    @staticmethod
    def transformJointPoints(joint,points,translation=array((0.0,0.0,0.0))):
        Rinv = rotationMatrixFromQuat(quatRotateDirection([0,0,1],-joint.GetAxis(0)))
        return dot(points,transpose(Rinv)) + tile(joint.GetAnchor()+translation,(len(points),1))

    def show(self,options=None):
        self.env.SetViewer('qtcoin')
        for joint in self.robot.GetJoints():
            print joint.GetJointIndex()
            haxis = self.env.drawlinestrip(points=vstack((joint.GetAnchor()-2.0*joint.GetAxis(0),joint.GetAnchor()+2.0*joint.GetAxis(0))),linewidth=3.0,colors=array((0.1,0.1,0,1)))
            jv = self.jointvolumes[joint.GetJointIndex()]
            hvol = self.env.drawtrimesh(points=self.transformJointPoints(joint,jv['sweptpoints']),indices=jv['sweptindices'],colors=array((0,0,1,0.2)))
            crossarea = jv['crossarea']
            harea = self.env.plot3(points=self.transformJointPoints(joint,c_[crossarea[:,0],zeros(len(crossarea)),crossarea[:,1]]),pointsize=5.0,colors=array((1,0,0,0.3)))
            raw_input('press any key to go to next: ')

    def computeGeometryStatistics(self,hulls):
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
                print '%d/%d'%(i,len(volumepoints))
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
    def prunePointsKDTree(points, thresh2, neighsize,k=20):
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
            except pyann_exception:
                print 'prunePointsKDTree: ann memory exceeded. Retrying with less neighbors'
                k = (k+1)/2
            except MemoryError:
                print 'prunePointsKDTree: memory error. Retrying with less neighbors'
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
            #print 'repeating pruning... %d/%d'%(len(inds),points.shape[0])
            newinds = LinkStatisticsModel.prunePointsKDTree(points[inds,:], thresh2, neighsize,k)
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
