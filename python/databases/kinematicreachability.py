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
"""6D kinematic reachability space of a robot's manipulators.

.. image:: ../../images/databases/kinematicreachability.jpg
  :width: 640

.. image:: ../../images/databases/kinematicreachability_side.jpg
  :width: 640

`[source] <../_modules/openravepy/databases/kinematicreachability.html>`_

**Running the Generator**

.. code-block:: bash

  openrave.py --database kinematicreachability --robot=robots/barrettsegway.robot.xml

**Showing the Reachability** (uses mayavi2)

.. code-block:: bash

  openrave.py --database kinematicreachability --robot=robots/barrettsegway.robot.xml --show

Description
-----------

This is the reachability when counting the total number of configurations possible at each pose. 

Command-line
------------

.. shell-block:: openrave.py --database kinematicreachability --help

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

from .. import metaclass, pyANN
from ..misc import SpaceSamplerExtra
from . import DatabaseGenerator
import convexdecomposition,inversekinematics
import time
import os.path
from heapq import nsmallest # for nth smallest element
from optparse import OptionParser

class ReachabilityModel(DatabaseGenerator):
    """Computes the robot manipulator's reachability space (stores it in 6D) and
    offers several functions to use it effectively in planning."""

    class QuaternionKDTree(metaclass.AutoReloader):
        """Artificially add more weight to the X,Y,Z translation dimensions"""
        def __init__(self, poses,transmult):
            self.numposes = len(poses)
            self.transmult = transmult
            self.itransmult = 1/transmult
            searchposes = array(poses)
            searchposes[:,4:] *= self.transmult # take translation errors more seriously
            allposes = r_[searchposes,searchposes]
            allposes[self.numposes:,0:4] *= -1
            self.nnposes = pyANN.KDTree(allposes)
        def kSearch(self,poses,k,eps):
            """returns distance squared"""
            poses[:,4:] *= self.transmult
            neighs,dists = self.nnposes.kSearch(poses,k,eps)
            neighs[neighs>=self.numposes] -= self.numposes
            poses[:,4:] *= self.itransmult
            return neighs,dists
        def kFRSearch(self,pose,radiussq,k,eps):
            """returns distance squared"""
            pose[4:] *= self.transmult
            neighs,dists,kball = self.nnposes.kFRSearch(pose,radiussq,k,eps)
            neighs[neighs>=self.numposes] -= self.numposes
            pose[4:] *= self.itransmult
            return neighs,dists,kball
        def kFRSearchArray(self,poses,radiussq,k,eps):
            """returns distance squared"""
            poses[:,4:] *= self.transmult
            neighs,dists,kball = self.nnposes.kFRSearchArray(poses,radiussq,k,eps)
            neighs[neighs>=self.numposes] -= self.numposes
            poses[:,4:] *= self.itransmult
            return neighs,dists,kball

    def __init__(self,robot):
        DatabaseGenerator.__init__(self,robot=robot)
        self.ikmodel = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        self.reachabilitystats = None
        self.reachability3d = None
        self.reachabilitydensity3d = None
        self.pointscale = None
        self.xyzdelta = None
        self.quatdelta = None
        self.kdtree6d = None
        self.kdtree3d = None
    def clone(self,envother):
        clone = DatabaseGenerator.clone(self,envother)
        return clone
    def has(self):
        return len(self.reachabilitydensity3d) > 0 and len(self.reachability3d) > 0
    def getversion(self):
        return 4
    def load(self):
        try:
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()
            params = DatabaseGenerator.load(self)
            if params is None:
                return False
            self.reachabilitystats,self.reachabilitydensity3d,self.reachability3d,self.pointscale,self.xyzdelta,self.quatdelta = params
            return self.has()
        except e:
            return False
    def save(self):
        DatabaseGenerator.save(self,(self.reachabilitystats,self.reachabilitydensity3d,self.reachability3d, self.pointscale,self.xyzdelta,self.quatdelta))

    def getfilename(self,read=False):
        return RaveFindDatabaseFile(os.path.join('robot.'+self.robot.GetKinematicsGeometryHash(), 'reachability.' + self.manip.GetStructureHash() + '.pp'),read)

    def autogenerateparams(self,options=None):
        maxradius=None
        translationonly=False
        xyzdelta=None
        quatdelta=None
        usefreespace=False
        if options is not None:
            if options.maxradius is not None:
                maxradius = options.maxradius
            if options.xyzdelta is not None:
                xyzdelta=options.xyzdelta
            if options.quatdelta is not None:
                quatdelta=options.quatdelta
            usefreespace=options.usefreespace
        if self.robot.GetKinematicsGeometryHash() == 'e829feb384e6417bbf5bd015f1c6b49a' or self.robot.GetKinematicsGeometryHash() == '22548f4f2ecf83e88ae7e2f3b2a0bd08': # wam 7dof
            if maxradius is None:
                maxradius = 1.1
        elif self.robot.GetKinematicsGeometryHash() == 'e3b4168a72a78fa2c37dc414cabb933a': # pr2
            if xyzdelta is None:
                xyzdelta = 0.03
            if quatdelta is None:
                quatdelta = 0.2
        return maxradius,translationonly,xyzdelta,quatdelta,usefreespace

    def getOrderedArmJoints(self):
        return [j for j in self.robot.GetDependencyOrderedJoints() if j.GetJointIndex() in self.manip.GetArmIndices()]
    @staticmethod
    def getManipulatorLinks(manip):
        links = manip.GetChildLinks()
        # add the links connecting to the base link.... although this reduces the freespace of the arm, it is better to have than not (ie waist on humanoid)
        tobasejoints = manip.GetRobot().GetChain(0,manip.GetBase().GetIndex())
        dofindices = [arange(joint.GetDOFIndex(),joint.GetDOFIndex()+joint.GetDOF()) for joint in tobasejoints if joint.GetDOFIndex() >= 0 and not joint.IsStatic()]
        tobasedofs = hstack(dofindices) if len(dofindices) > 0 else array([],int)
        robot = manip.GetRobot()
        joints = robot.GetJoints()
        for jindex in r_[manip.GetArmIndices(),tobasedofs]:
            joint = joints[jindex]
            if joint.GetFirstAttached() and not joint.GetFirstAttached() in links:
                links.append(joint.GetFirstAttached())
            if joint.GetSecondAttached() and not joint.GetSecondAttached() in links:
                links.append(joint.GetSecondAttached())
        # don't forget the rigidly attached links
        for link in links[:]:
            for newlink in link.GetRigidlyAttachedLinks():
                if not newlink in links:
                    links.append(newlink)
        return links

    def generatepcg(self,maxradius=None,translationonly=False,xyzdelta=None,quatdelta=None,usefreespace=False):
        """Generate producer, consumer, and gatherer functions allowing parallelization
        """
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        # disable every body but the target and robot\
        if xyzdelta is None:
            xyzdelta=0.04
        if quatdelta is None:
            quatdelta=0.5
        self.kdtree3d = self.kdtree6d = None
        with self.robot:
            self.robot.SetTransform(eye(4))
            Tbase = self.manip.GetBase().GetTransform()
            Tbaseinv = linalg.inv(Tbase)
            maniplinks = self.getManipulatorLinks(self.manip)
            for link in self.robot.GetLinks():
                link.Enable(link in maniplinks)
            # the axes' anchors are the best way to find the max radius
            # the best estimate of arm length is to sum up the distances of the anchors of all the points in between the chain
            armjoints = self.getOrderedArmJoints()
            baseanchor = transformPoints(Tbaseinv,[armjoints[0].GetAnchor()])
            eetrans = self.manip.GetEndEffectorTransform()[0:3,3]
            armlength = 0
            for j in armjoints[::-1]:
                armlength += sqrt(sum((eetrans-j.GetAnchor())**2))
                eetrans = j.GetAnchor()    
            if maxradius is None:
                maxradius = armlength+xyzdelta*sqrt(3.0)

            allpoints,insideinds,shape,self.pointscale = self.UniformlySampleSpace(maxradius,delta=xyzdelta)
            qarray = SpaceSamplerExtra().sampleSO3(quatdelta=quatdelta)
            rotations = [eye(3)] if translationonly else rotationMatrixFromQArray(qarray)
            self.xyzdelta = xyzdelta
            self.quatdelta = 0
            if not translationonly:
                # for rotations, get the average distance to the nearest rotation
                neighdists = []
                for q in qarray:
                    neighdists.append(nsmallest(2,quatArrayTDist(q,qarray))[1])
                self.quatdelta = mean(neighdists)
            print 'radius: %f, xyzsamples: %d, quatdelta: %f, rot samples: %d, freespace: %d'%(maxradius,len(insideinds),self.quatdelta,len(rotations),usefreespace)
            
        self.reachabilitydensity3d = zeros(prod(shape))
        self.reachability3d = zeros(prod(shape))
        self.reachabilitystats = []

        def producer():
            T = eye(4)
            for i,ind in enumerate(insideinds):
                T[0:3,3] = allpoints[ind]+baseanchor
                if mod(i,1000)==0:
                    raveLogInfo('%s/%d'%(i,len(insideinds)))
                yield ind,T
        def consumer(ind,T):
            with self.env:
                reachabilitystats = []
                numvalid = 0
                numrotvalid = 0
                T = array(T)
                for rotation in rotations:
                    T[0:3,0:3] = rotation
                    if usefreespace:
                        solutions = self.manip.FindIKSolutions(dot(Tbase,T),0)
                        if solutions is not None:
                            reachabilitystats.append(r_[poseFromMatrix(T),len(solutions)])
                            numvalid += len(solutions)
                            numrotvalid += 1
                    else:
                        solution = self.manip.FindIKSolution(dot(Tbase,T),0)
                        if solution is not None:
                            reachabilitystats.append(r_[poseFromMatrix(T),1])
                            numvalid += 1
                            numrotvalid += 1
                return ind,reachabilitystats, numvalid, numrotvalid

        def gatherer(ind=None,reachabilitystats=None,numvalid=None,numrotvalid=None):
            if ind is not None:
                self.reachabilitystats += reachabilitystats
                self.reachabilitydensity3d[ind] = numvalid/float(len(rotations))
                self.reachability3d[ind] = numrotvalid/float(len(rotations))
            else:
                self.reachability3d = reshape(self.reachability3d,shape)
                self.reachabilitydensity3d = reshape(self.reachabilitydensity3d,shape)
                self.reachabilitystats = array(self.reachabilitystats)

        return producer, consumer, gatherer, len(insideinds)


    def show(self,showrobot=True,contours=[0.01,0.1,0.2,0.5,0.8,0.9,0.99],opacity=None,figureid=1, xrange=None,options=None):
        mlab = __import__('enthought.mayavi.mlab',fromlist=['mlab'])
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
            with self.robot:
                Tbase = self.manip.GetBase().GetTransform()
                Tbaseinv = linalg.inv(Tbase)
                self.robot.SetTransform(dot(Tbaseinv,self.robot.GetTransform()))
                baseanchor = self.getOrderedArmJoints()[0].GetAnchor()
                trimesh = self.env.Triangulate(self.robot)
            v = self.pointscale[0]*(trimesh.vertices-tile(baseanchor,(len(trimesh.vertices),1)))+self.pointscale[1]
            mlab.triangular_mesh(v[:,0]-offset[0],v[:,1]-offset[1],v[:,2]-offset[2],trimesh.indices,color=(0.5,0.5,0.5))
        mlab.show()

    def UniformlySampleSpace(self,maxradius,delta):
        nsteps = floor(maxradius/delta)
        X,Y,Z = mgrid[-nsteps:nsteps,-nsteps:nsteps,-nsteps:nsteps]
        allpoints = c_[X.flat,Y.flat,Z.flat]*delta
        insideinds = flatnonzero(sum(allpoints**2,1)<maxradius**2)
        return allpoints,insideinds,X.shape,array((1.0/delta,nsteps))

    def ComputeNN(self,translationonly=False):
        if translationonly:
            if self.kdtree3d is None:
                self.kdtree3d = pyANN.KDTree(self.reachabilitystats[:,4:7])
            return self.kdtree3d
        else:
            if self.kdtree6d is None:
                self.kdtree6d = self.QuaternionKDTree(self.reachabilitystats,5.0)
            return self.kdtree6d
    @staticmethod
    def CreateOptionParser():
        parser = DatabaseGenerator.CreateOptionParser()
        parser.description='Computes the reachability region of a robot manipulator and python pickles it into a file.'
        parser.usage='openrave.py --database kinematicreachability [options]'
        parser.add_option('--maxradius',action='store',type='float',dest='maxradius',default=None,
                          help='The max radius of the arm to perform the computation')
        parser.add_option('--xyzdelta',action='store',type='float',dest='xyzdelta',default=None,
                          help='The max radius of the arm to perform the computation (default=0.04)')
        parser.add_option('--quatdelta',action='store',type='float',dest='quatdelta',default=None,
                          help='The max radius of the arm to perform the computation (default=0.5)')
        parser.add_option('--usefreespace',action='store_true',dest='usefreespace',default=False,
                          help='If set, will record the number of IK solutions that exist for every transform rather than just finding one. More useful map, but much slower to produce')
        parser.add_option('--showscale',action='store',type='float',dest='showscale',default=1.0,
                          help='Scales the reachability by this much in order to show colors better (default=%default)')
        return parser
    @staticmethod
    def InitializeFromParser(Model=None,parser=None,*args,**kwargs):
        if Model is None:
            Model = lambda robot: ReachabilityModel(robot=robot)
        if parser is None:
            parser = ReachabilityModel.CreateOptionParser()
        return DatabaseGenerator.InitializeFromParser(Model,parser,*args,**kwargs)

def run(*args,**kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    ReachabilityModel.RunFromParser(Model = lambda robot: ReachabilityModel(robot=robot), parser = ReachabilityModel.CreateOptionParser(), *args,**kwargs)
