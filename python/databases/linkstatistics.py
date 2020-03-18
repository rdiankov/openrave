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
from ..openravepy_ext import transformPoints
from ..openravepy_int import RaveFindDatabaseFile, RaveDestroy, Environment, KinBody, rotationMatrixFromQuat, quatRotateDirection, rotationMatrixFromAxisAngle, RaveGetDefaultViewerType
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
    
    grabbedjointspheres = None # a list of (grabbedinfo, dict) that stores swept spheres of each joint. key is joint index. 
    def __init__(self,robot):
        DatabaseGenerator.__init__(self,robot=robot)
    
    def has(self):
        return self.grabbedjointspheres is not None and len(self.grabbedjointspheres) > 0
    
    @staticmethod
    def _GetValue(value):
        if hasattr(value,'value'):
            return value.value
        else:
            return value

    def getversion(self):
        return 6
    
    def save(self):
        self.SavePickle()

    def load(self):
        return self.LoadPickle()
    
    def SavePickle(self):
        DatabaseGenerator.save(self,self.grabbedjointspheres)
    
    def LoadPickle(self):
        try:
            params = DatabaseGenerator.load(self)
        except Exception, e:
            log.warn(u'failed to load linkstatistics: %s', e)
            return False
        
        if params is None:
            return False
        self.grabbedjointspheres = params
        return self.has()
    
    def getfilename(self,read=False):
        return RaveFindDatabaseFile(os.path.join('robot.'+self.robot.GetKinematicsGeometryHash(), 'linkstatistics.pp'),read)
        
    def setRobotResolutions(self,xyzdelta=None):
        """sets the robot resolution
        xyzdelta is the maxdistance allowed to be 
        """
        with self.env:
            if xyzdelta is None:
                xyzdelta = 0.005/self.env.GetUnit()[1]

            jointspheres = self._GetJointSpheresFromGrabbed(self.robot.GetGrabbedInfo())
            resolutions = xyzdelta*ones(self.robot.GetDOF())
            for ijoint in range(len(self.robot.GetJoints())):
                if ijoint in jointspheres:
                    dofindex = self.robot.GetJoints()[ijoint].GetDOFIndex()
                    if abs(jointspheres[ijoint][1]) > 1e-7:
                        # sometimes there are no geometries attached for prototype robots...
                        resolutions[dofindex] = xyzdelta/jointspheres[ijoint][1]
            self.robot.SetDOFResolutions(resolutions)
            self.robot.SetAffineTranslationResolution(tile(xyzdelta,3))
            self.robot.SetAffineRotationAxisResolution(tile(resolutions[0],4))
        
    def setRobotWeights(self,type=0,weightmult=1.0):
        """sets the weights for the robot.
        """
        with self.env:
            jointspheres = self._GetJointSpheresFromGrabbed(self.robot.GetGrabbedInfo())
            if type == 0:
                weights = ones(self.robot.GetDOF())
                totalweight = 0.0
                numweights = 0
                for ijoint in range(len(self.robot.GetJoints())):
                    if ijoint in jointspheres:
                        dofindex = self.robot.GetJoints()[ijoint].GetDOFIndex()
                        weights[dofindex] = jointspheres[ijoint][1]
                        totalweight += jointspheres[ijoint][1]
                        numweights += 1

                # avoid division by zero, and let small weights be handled below
                if totalweight > 1e-7:
                    for ijoint in range(len(self.robot.GetJoints())):
                        if ijoint in jointspheres:
                            dofindex = self.robot.GetJoints()[ijoint].GetDOFIndex()
                            weights[dofindex] *= weightmult*numweights/totalweight
                else:
                    log.debug('total weight (%s) for robot %s is too small, so do not normalize', totalweight, self.robot.GetName())
                    
                # shouldn't have small weights...
                for idof in range(self.robot.GetDOF()):
                    if weights[idof] <= 1e-7:
                        weights[idof] = 0.02
                
                self.robot.SetDOFWeights(weights)
                self.robot.SetAffineTranslationWeights([weights[0],weights[0],weights[0]])
                self.robot.SetAffineRotationAxisWeights(tile(weights[0],4)) # only z axis
            elif type == 1:
                # set everything to 1
                for j in self.robot.GetJoints():
                    j.SetWeights(ones(j.GetDOF()))
                self.robot.SetAffineTranslationWeights(ones(3))
                self.robot.SetAffineRotationAxisWeights(ones(4))
            else:
                raise ValueError('no such type')
    
    def autogenerate(self,options=None):
        self.generate()
        self.save()
    
    def generate(self,**kwargs):
        """
        :param computeaffinevolumes: if True will compute affine volumes
        """
        with self.robot:
            self.robot.SetTransform(eye(4))
            self.robot.SetDOFValues(zeros(self.robot.GetDOF()))
            self.grabbedjointspheres = [(self.robot.GetGrabbedInfo(), self._ComputeJointSpheres())]
    
    def _GetJointSpheresFromGrabbed(self, grabbedinfo):
        for testgrabbedinfo, testjointspheres in self.grabbedjointspheres:
            if len(testgrabbedinfo) == len(grabbedinfo):
                if all([(grabbedinfo[i]._grabbedname == testgrabbedinfo[i]._grabbedname and grabbedinfo[i]._robotlinkname == testgrabbedinfo[i]._robotlinkname and sum(abs(grabbedinfo[i]._trelative-testgrabbedinfo[i]._trelative)) <= 1e-7 and grabbedinfo[i]._setRobotLinksToIgnore == testgrabbedinfo[i]._setRobotLinksToIgnore) for i in range(len(grabbedinfo))]):
                    return testjointspheres
        
        log.debug('adding new linkstatistic for grabbed bodies: %r', [g._grabbedname for g in grabbedinfo])
        jointspheres = self._ComputeJointSpheres()
        self.grabbedjointspheres.append((grabbedinfo, jointspheres)) # tuple copies so that it doesn't change...
        return jointspheres
    
    def _ComputeJointSpheres(self):
        jointspheres = {}
        for j in self.robot.GetDependencyOrderedJoints()[::-1]:
            if not j.IsRevolute(0):
                continue
            
            childlinks = j.GetHierarchyChildLink().GetRigidlyAttachedLinks()
            sphereradius = 0.0
            for childlink in childlinks:
                Tlink = childlink.GetTransform()
                localaabb = childlink.ComputeLocalAABB()
                linkpos = dot(Tlink[:3,:3], localaabb.pos()) + Tlink[:3,3]
                extensiondist = linalg.norm(j.GetAnchor() - linkpos)
                linkradius = linalg.norm(localaabb.extents())
                sphereradius = max(sphereradius, linkradius+extensiondist)

            spherepos = j.GetAnchor()
            # process any child joints
            minpos = spherepos - sphereradius*ones([1,1,1])
            maxpos = spherepos + sphereradius*ones([1,1,1])

            childjoints = [testj for testj in self.robot.GetJoints() if testj.GetHierarchyParentLink() in childlinks]
            for childjoint in childjoints:
                if childjoint.GetJointIndex() in jointspheres:
                    childpos, childradius = jointspheres[childjoint.GetJointIndex()]
                    minpos = numpy.minimum(minpos, childpos - sphereradius*ones([1,1,1]))
                    maxpos = numpy.maximum(maxpos, childpos + sphereradius*ones([1,1,1]))
            
            newspherepos = 0.5*(minpos + maxpos)
            newsphereradius = linalg.norm(newspherepos - spherepos) + sphereradius
            for childjoint in childjoints:
                if childjoint.GetJointIndex() in jointspheres:
                    childpos, childradius = jointspheres[childjoint.GetJointIndex()]
                    newsphereradius = max(newsphereradius, linalg.norm(newspherepos - childpos) + childradius)

            # go through all the spheres and get the max radius
            jointspheres[j.GetJointIndex()] = (numpy.around(newspherepos, 8), numpy.around(newsphereradius, 8))
        return jointspheres
    
    def show(self,options=None):
        pass
    
    @staticmethod
    def CreateOptionParser():
        parser = DatabaseGenerator.CreateOptionParser(useManipulator=False)
        parser.description='Computes statistics about the link geometry'
        parser.usage='openrave.py --database linkstatistics [options]'
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
