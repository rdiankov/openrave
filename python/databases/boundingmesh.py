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
"""Bounding mesh of the link geometry of the robot

Usage
-----


Description
-----------

Command-line
------------


Class Definitions
-----------------

"""

from __future__ import with_statement
__author__ = ''
__copyright__= 'Copyright (C) 2009-2012 Rosen Diankov <rosen.diankov@gmail.com>'
__license__ = 'Apache License, Version 2.0'

from numpy import array, vectorize

from ..openravepy_int import KinBody, RaveFindDatabaseFile, RaveDestroy, Environment, TriMesh, RaveCreateModule, GeometryType, RaveGetDefaultViewerType
from ..openravepy_ext import transformPoints, transformInversePoints
from . import DatabaseGenerator

import time
import os.path
from os import makedirs
from optparse import OptionParser
from itertools import izip

import logging
log = logging.getLogger('openravepy.' + __name__.split('.',2)[-1])

try:
    from .. import boundingmeshpy
except Exception, e:
    print 'failed to import boundingmeshpy (boundingmesh might not be installed)', e

class BoundingMeshError(Exception):
    def __init__(self, msg=u''):
        self.msg = msg
    def __unicode__(self):
        return u'Convex Decomposition Error : %s' % self.msg
    def __str__(self):
        return unicode(self).encode('utf-8')

class BoundingMeshModel(DatabaseGenerator):
    """Computes the bounding mesh of all the robot's links"""

    def __init__(self, robot):
        DatabaseGenerator.__init__(self, robot=robot)
        self.linkgeometry = None
        self.boundingparams = None

    def clone(self, envother):
        clone = DatabaseGenerator.clone(self, envother)
        return clone

    def has(self):
        return self.linkgeometry is not None and len(self.linkgeometry) == len(self.robot.GetLinks())

    def getversion(self):
        return 0

    def save(self):
        try:
            self.SaveHDF5()
        except ImportError:
            log.warn('python h5py not found, will not be able to speedup database access')
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
        DatabaseGenerator.save(self, (self.linkgeometry, self.boundingparams))

    def SaveHDF5(self):
        import h5py
        filename = self.getfilename(False)
        log.info(u'saving model to %s', filename)
        try:
            makedirs(os.path.split(filename)[0])
        except OSError:
            pass

        f = h5py.File(filename, 'w')
        try:
            f['version'] = self.getversion()
            gparams = f.create_group('params')
            for name, value in self.boundingparams.iteritems():
                gparams[name] = value
            glinkgeometry = f.create_group('linkgeometry')
            for ilink, linkgeometry in enumerate(self.linkgeometry):
                glink = glinkgeometry.create_group(str(ilink))
                for ig, geometryboundingmesh in linkgeometry:
                    gboundingmesh = glink.create_group(str(ig))
                    gboundingmesh['igeometry'] = ig
                    for name in ['vertices', 'indices']:
                        values = getattr(geometryboundingmesh, name)
                        if len(values) == 0:
                            gboundingmesh.create_dataset(name, [], dtype=values.dtype)
                        else:
                            gboundingmesh[name] = values
        finally:
            f.close()

    def LoadPickle(self):
        try:
            params = DatabaseGenerator.load(self)
            if params is None:
                return False
            self.linkgeometry, self.boundingparams = params
            return self.has()
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
            f = h5py.File(filename, 'r')
            if f['version'].value != self.getversion():
                log.error(u'version is wrong %s != %s', f['version'], self.getversion())
                return False

            self.boundingparams = {}
            gparams = f['params']
            for name, value in gparams.iteritems():
                self.boundingparams[name] = value
            glinkgeometry = f['linkgeometry']
            self.linkgeometry = []
            for ilink, glink in glinkgeometry.iteritems():
                linkgeometry = []
                for ig, glinkboundingmesh in glink.iteritems():
                    mesh = TriMesh()
                    mesh.vertices = glinkboundingmesh['vertices'].value
                    mesh.indices = glinkboundingmesh['indices'].value
                    linkgeometry.append((int(ig), mesh))
                while len(self.linkgeometry) <= int(ilink):
                    self.linkgeometry.append(None)
                self.linkgeometry[int(ilink)] = linkgeometry
            self._databasefile = f
            f = None
            return self.has()
        except Exception, e:
            log.debug(u'LoadHDF5 for %s: ' , filename, e)
            return False
        finally:
            if f is not None:
                f.close()

    def setrobot(self):
        with self.env:
            for link, linkboundingmeshes in izip(self.robot.GetLinks(), self.linkgeometry):
                for ig, boundingmesh in linkboundingmeshes:
                    if link.GetGeometries()[ig].IsModifiable():
                        link.GetGeometries()[ig].SetCollisionMesh(boundingmesh)

    def getfilename(self, read = False):
        filename = 'boundingmesh.3f.pp'
        return RaveFindDatabaseFile(os.path.join('robot'+self.robot.GetKinematicsGeometryHash(), filename), read)

    def autogenerate(self, options = None):
        if options is not None:
            self.generate(skipLinks=options.skipLinks.split(','),
                          targetVerticesCount=options.targetVerticesCount,
                          maximumError=options.maximumError,
                          direction=self.GetOptional(boundingmeshpy.DecimationDirection, options.direction),
                          metric=self.GetOptional(boundingmeshpy.Metric, options.metric),
                          initialization=self.GetOptional(boundingmeshpy.Initialization,options.initialization))
        else:
            self.generate()
        self.save()

    @staticmethod
    def GetOptional(obj, attr):
        try:
            return getattr(obj, attr)
        except:
            return None

    def generate(self, skipLinks = None, **kwargs):
        """
        :param skipLinks : a list of link names which should keep their original mesh
        """
        self.boundingparams = kwargs
        if skipLinks == None:
            skipLinks = []
        log.info(u'Generating Bounding Meshes : %r', self.boundingparams)
        starttime = time.time()
        self.linkgeometry = []
        with self.env:
            links = self.robot.GetLinks()
            for il, link in enumerate(links):
                if link in skipLinks:
                    continue
                boundingmeshes = []
                geometries = link.GetGeometries()
                for ig, geom in enumerate(geometries):
                    trimesh = geom.GetCollisionMesh()
                    if len(trimesh.indices) == 0:
                        geom.InitCollisionMesh(100.0)
                        trimesh = geom.GetCollisionMesh()
                    log.info(u'Computing bounding mesh for link %d/%d geom %d/%d', il, len(links), ig, len(geometries))
                    resultmesh = self.ComputeBoundingMesh(trimesh)
                    if not self.CheckBoundingMesh(trimesh, resultmesh):
                        raise BoundingMeshError(u'Geom link %s could not be bounded correctly', link.GetName())
                    vsize = resultmesh.vertices.shape[0]
                    indsize = resultmesh.indices.shape[0]
                    log.info(u'Vertices left : %d (%f%%), Triangles left : %d (%f%%)', vsize, 100 * float(vsize )/float(trimesh.vertices.shape[0]), indsize, 100*float(indsize)/float(trimesh.indices.shape[0]))
                    boundingmeshes.append((ig, resultmesh))
                self.linkgeometry.append(boundingmeshes)
        log.info(u'All meshes bounded in %fs', time.time()-starttime)

    def ComputeBoundingMesh(self, trimesh):
        trimesh = self.ReduceTrimesh(trimesh)
        if len(trimesh.indices) > 0:
            return boundingmeshpy.ComputeBoundingMesh(trimesh, **self.boundingparams)
        return trimesh

    @staticmethod
    def ReduceTrimesh(trimesh):
        to3Tuple = lambda v : (v[0], v[1], v[2])
        v = list(set(map(to3Tuple, trimesh.vertices)))
        d = { v[i] : i for i in range(0, len(v)) }
        reducedMesh = TriMesh()
        reducedMesh.vertices = array(map(array, v))
        reducedMesh.indices = vectorize(lambda i : d[to3Tuple(trimesh.vertices[i])])(trimesh.indices)
        return reducedMesh

    def CheckBoundingMesh(self, originalmesh, boundingmesh):
        # TODO : do a real exhaustive check that the distance to the original mesh is bounded by the maximum error and that all points are outside of the mesh
        return len(boundingmesh.indices) > 0


    def show(self, options=None):
        if self.env.GetViewer() is None:
            self.env.SetViewer(RaveGetDefaultViewerType())
            time.sleep(0.4) # give time for viewer to initialize
        self.env.UpdatePublishedBodies()
        if not self.has():
            self.autogenerate(options=options)
        for link, linkboundingmeshes in izip(self.robot.GetLinks(), self.linkgeometry):
            for ig, boundingmesh in linkboundingmeshes:
                if link.GetGeometries()[ig].IsModifiable():
                    link.GetGeometries()[ig].SetCollisionMesh(boundingmesh)
        #from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
        raw_input('Press any key to exit : ')

    @staticmethod
    def CreateOptionParser():
        parser = DatabaseGenerator.CreateOptionParser(useManipulator=False)
        parser.description = 'Computes a bounding mesh for each geometry.'
        parser.usage = 'openrave.py --database boundingmesh [options]'
        parser.add_option('--targetVerticesCount',
                          action='store',
                          type='int',
                          dest='targetVerticesCount',
                          default=600,
                          help='Minimum number of vertices to be reached (default:%default)')
        parser.add_option('--maximumError',
                          action='store',
                          type='float',
                          dest='maximumError',
                          default=0.0001,
                          help='Maximum error tolerated when bounding a mesh (default:%default)')
        parser.add_option('--direction',
                          action='store',
                          type='str',
                          dest='direction',
                          default='Outward',
                          help='Direction in which the bounding mesh can extend, either Outward, Inward or Any (default:%default)')
        parser.add_option('--metric',
                          action='store',
                          type='str',
                          dest='metric',
                          default='ModifiedQEM',
                          help='Error metric used by the algorithm, either ClassicQEM, ModifiedQEM, MinimizedConstant, Diagonalization or Average (default:%default)')
        parser.add_option('--initialization',
                          action='store',
                          type='str',
                          dest='initialization',
                          default='DistancePrimitives',
                          help="How to initialize the algorithm, either DistancePrimitives or Midpoint (default:%default)")
        parser.add_option('--skipLinks',
                          action='store',
                          type='str',
                          dest='skipLinks',
                          default='',
                          help='Comma separated list of link names whose meshes must be left intact')
        return parser

    @staticmethod
    def RunFromParser(Model=None, parser=None, args=None, **kwargs):
        """Executes the BoundingMeshModel database generation
        """
        if parser is None:
            parser = BoundingMeshModel.CreateOptionParser()
            (options, leftargs) = parser.parse_args(args=args)
            env = Environment()
            try:
                if Model is None:
                    Model = lambda robot: BoundingMeshModel(robot=robot)
                DatabaseGenerator.RunFromParser(env=env, Model=Model, parser=parser, allowkinbody=True, args=args, **kwargs)
            finally:
                env.Destroy()
                RaveDestroy()

def run(*args, **kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    BoundingMeshModel.RunFromParser(*args, **kwargs)
