# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
from __future__ import with_statement # for python 2.5
import openravepy_int
import numpy
try:
    import cPickle as pickle
except:
    import pickle

import logging
log = logging.getLogger('openravepy')

def KinBodyStateSaver(body,options=None):
    log.warn('use body.CreateKinBodyStateSaver instead of KinBodyStateSaver')
    return body.CreateKinBodyStateSaver(options)
def RobotStateSaver(body,options=None):
    log.warn('use body.CreateRobotStateSaver instead of RobotStateSaver')
    return body.CreateRobotStateSaver(options)

class CollisionOptionsStateSaver(object):
    """Saves/restores the state of the collision checker options
    """
    def __init__(self,checker,options=None,required=True):
        self.checker=checker
        self.oldoptions = None
        self.newoptions=options
        self.required = required
    def __enter__(self):
        if self.newoptions is not None:
            self.oldoptions = self.checker.GetCollisionOptions()
            success = self.checker.SetCollisionOptions(self.newoptions)
            if not success and self.required:
                self.checker.SetCollisionOptions(self.oldoptions)
                raise openrave_exception('Failed to set options 0x%x on checker %s'%(self.newoptions,str(self.checker.GetXMLId())))
            
    def __exit__(self, type, value, traceback):
        if self.oldoptions is not None:
            self.checker.SetCollisionOptions(self.oldoptions)

class TransformQuaternionsSaver(object):
    """saves/restores the openravepy_int.options.returnTransformQuaternion state
    """
    def __enter__(self):
        self.laststate = openravepy_int.options.returnTransformQuaternion
    def __exit__(self, type, value, traceback):
        openravepy_int.options.returnTransformQuaternion = self.laststate
        
def with_destroy(fn):
    """a decorator that always calls openravepy_int.RaveDestroy at the function end"""
    def newfn(*args,**kwargs):
        try:
            return fn(*args,**kwargs)
        finally:
            openravepy_int.RaveDestroy()
    newfn.__doc__ = fn.__doc__
    newfn.__module__ = fn.__module__
    newfn.__name__ = fn.__name__
    return newfn

# this is necessary due to broken boost python pickle support for enums
def _tuple2enum(enum, value):
    enum = getattr(openravepy_int, enum)
    e = enum.values.get(value,None)
    if e is None:
        e = enum(value)
    return e

#def isEnumType(o):
#    return isinstance(o, type) and issubclass(o,int) and not (o is int)

def _registerEnumPicklers(): 
    from copy_reg import constructor, pickle
    def reduce_enum(e):
        enum = type(e).__name__.split('.')[-1]
        return ( _tuple2enum, ( enum, int(e) ) )
    constructor( _tuple2enum)
    pickle(openravepy_int.IkParameterizationType,reduce_enum)
    #for e in [ e for e in vars(openravepy).itervalues() if isEnumType(e) ]:
    #    pickle(e, reduce_enum)

_registerEnumPicklers()

import atexit
atexit.register(openravepy_int.RaveDestroy)

class openrave_exception(Exception):
    """wrap up the C++ openrave_exception"""
    def __init__( self, app_error ):
        Exception.__init__( self )
        self._pimpl = app_error
    def __str__( self ):
        return str(self._pimpl)
    def __unicode__( self ):
        return unicode(self._pimpl)
    def __getattribute__(self, attr):
        my_pimpl = super(openrave_exception, self).__getattribute__("_pimpl")
        try:
            return getattr(my_pimpl, attr)
        except AttributeError:
            return super(openrave_exception,self).__getattribute__(attr)

class runtime_error(Exception):
    """wrap up the C++ runtime_error"""
    def __init__( self, app_error ):
        Exception.__init__( self )
        self._pimpl = app_error
    def __str__( self ):
        return self._pimpl.message()
    def __getattribute__(self, attr):
        my_pimpl = super(runtime_error, self).__getattribute__("_pimpl")
        try:
            return getattr(my_pimpl, attr)
        except AttributeError:
            return super(runtime_error,self).__getattribute__(attr)

class planning_error(Exception):
    def __init__(self,parameter=''):
        self.parameter = parameter
    def __str__(self):
        return 'openrave planning_error: '+repr(self.parameter)

def normalizeZRotation(qarray):
    """for each quaternion, find the rotation about z that minimizes the distance between the identify (1,0,0,0).
    Return the transformed the quaternions along with the angle around the z-axis eliminated.
    qarray is a Nx4 array."""
    zangles = numpy.arctan2(-qarray[:,3],qarray[:,0])
    sinangles = numpy.sin(zangles)
    cosangles = numpy.cos(zangles)
    return numpy.c_[cosangles*qarray[:,0]-sinangles*qarray[:,3], cosangles*qarray[:,1]-sinangles*qarray[:,2], cosangles*qarray[:,2]+sinangles*qarray[:,1], cosangles*qarray[:,3]+sinangles*qarray[:,0]],-2.0*zangles

def quatArrayTMult(qarray,q):
    """ multiplies a Nx4 array of quaternions with a quaternion"""
    return numpy.c_[(qarray[:,0]*q[0] - qarray[:,1]*q[1] - qarray[:,2]*q[2] - qarray[:,3]*q[3],
                     qarray[:,0]*q[1] + qarray[:,1]*q[0] + qarray[:,2]*q[3] - qarray[:,3]*q[2],
                     qarray[:,0]*q[2] + qarray[:,2]*q[0] + qarray[:,3]*q[1] - qarray[:,1]*q[3],
                     qarray[:,0]*q[3] + qarray[:,3]*q[0] + qarray[:,1]*q[2] - qarray[:,2]*q[1])]

def quatMultArrayT(q,qarray):
    """ multiplies a quaternion q with each quaternion in the Nx4 array qarray"""
    return numpy.c_[(q[0]*qarray[:,0] - q[1]*qarray[:,1] - q[2]*qarray[:,2] - q[3]*qarray[:,3],
                     q[0]*qarray[:,1] + q[1]*qarray[:,0] + q[2]*qarray[:,3] - q[3]*qarray[:,2],
                     q[0]*qarray[:,2] + q[2]*qarray[:,0] + q[3]*qarray[:,1] - q[1]*qarray[:,3],
                     q[0]*qarray[:,3] + q[3]*qarray[:,0] + q[1]*qarray[:,2] - q[2]*qarray[:,1])]

def quatArrayRotate(qarray,trans):
    """rotates a point by an array of 4xN quaternions. Returns a 3xN vector"""
    xx = qarray[1,:] * qarray[1,:]
    xy = qarray[1,:] * qarray[2,:]
    xz = qarray[1,:] * qarray[3,:]
    xw = qarray[1,:] * qarray[0,:]
    yy = qarray[2,:] * qarray[2,:]
    yz = qarray[2,:] * qarray[3,:]
    yw = qarray[2,:] * qarray[0,:]
    zz = qarray[3,:] * qarray[3,:]
    zw = qarray[3,:] * qarray[0,:]
    return 2*numpy.vstack(((0.5-yy-zz)*trans[0]+(xy-zw)*trans[1]+(xz+yw)*trans[2], (xy+zw)*trans[0]+(0.5-xx-zz)*trans[1]+(yz-xw)*trans[2], (xz-yw)*trans[0]+(yz+xw)*trans[1]+(0.5-xx-yy)*trans[2]))

def quatArrayTRotate(qarray,trans):
    """rotates a point by an array of Nx4 quaternions. Returns a Nx3 vector"""
    xx = qarray[:,1] * qarray[:,1]
    xy = qarray[:,1] * qarray[:,2]
    xz = qarray[:,1] * qarray[:,3]
    xw = qarray[:,1] * qarray[:,0]
    yy = qarray[:,2] * qarray[:,2]
    yz = qarray[:,2] * qarray[:,3]
    yw = qarray[:,2] * qarray[:,0]
    zz = qarray[:,3] * qarray[:,3]
    zw = qarray[:,3] * qarray[:,0]
    return 2*numpy.c_[(0.5-yy-zz)*trans[0]+(xy-zw)*trans[1]+(xz+yw)*trans[2], (xy+zw)*trans[0]+(0.5-xx-zz)*trans[1]+(yz-xw)*trans[2], (xz-yw)*trans[0]+(yz+xw)*trans[1]+(0.5-xx-yy)*trans[2]]

def quatRotate(q,trans):
    """rotates a point by a 4-elt quaternion. Returns a 3 elt vector"""
    xx = q[1] * q[1]
    xy = q[1] * q[2]
    xz = q[1] * q[3]
    xw = q[1] * q[0]
    yy = q[2] * q[2]
    yz = q[2] * q[3]
    yw = q[2] * q[0]
    zz = q[3] * q[3]
    zw = q[3] * q[0]
    return 2*numpy.array(((0.5-yy-zz)*trans[0]+(xy-zw)*trans[1]+(xz+yw)*trans[2],
                          (xy+zw)*trans[0]+(0.5-xx-zz)*trans[1]+(yz-xw)*trans[2],
                          (xz-yw)*trans[0]+(yz+xw)*trans[1]+(0.5-xx-yy)*trans[2]))

def quatRotateArrayT(q,transarray):
    """rotates a set of points in Nx3 transarray by a quaternion. Returns a Nx3 vector"""
    xx = q[1] * q[1]
    xy = q[1] * q[2]
    xz = q[1] * q[3]
    xw = q[1] * q[0]
    yy = q[2] * q[2]
    yz = q[2] * q[3]
    yw = q[2] * q[0]
    zz = q[3] * q[3]
    zw = q[3] * q[0]
    return 2*numpy.c_[((0.5-yy-zz)*transarray[:,0]+(xy-zw)*transarray[:,1]+(xz+yw)*transarray[:,2],
                       (xy+zw)*transarray[:,0]+(0.5-xx-zz)*transarray[:,1]+(yz-xw)*transarray[:,2],
                       (xz-yw)*transarray[:,0]+(yz+xw)*transarray[:,1]+(0.5-xx-yy)*transarray[:,2])]

def poseMultArrayT(pose,posearray):
    """multiplies a pose with an array of poses (each pose is a quaterion + translation)"""
    return numpy.c_[quatMultArrayT(pose[0:4],posearray[:,0:4]),quatRotateArrayT(pose[0:4],posearray[:,4:7])+numpy.tile(pose[4:7],(len(posearray),1))]
    
def quatArrayTDist(q,qarray):
    """computes the natural distance (Haar measure) for quaternions, q is a 4-element array, qarray is Nx4"""
    return numpy.arccos(numpy.minimum(1.0,numpy.abs(numpy.dot(qarray,q))))

def transformPoints(T,points):
    """Transforms a Nxk array of points by an affine matrix"""
    kminus = T.shape[1]-1
    return numpy.dot(points,numpy.transpose(T[0:kminus,0:kminus]))+numpy.tile(T[0:kminus,kminus],(len(points),1))

def transformInversePoints(T,points):
    """Transforms a Nxk array of points by the inverse of an affine matrix"""
    kminus = T.shape[1]-1
    return numpy.dot(points-numpy.tile(T[0:kminus,kminus],(len(points),1)),T[0:kminus,0:kminus])
