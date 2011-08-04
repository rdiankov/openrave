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

try:
    from itertools import izip
except ImportError:
    pass

class KinBodyStateSaver:
    """Saves/restores the body state, use **with** statement.
    """
    def __init__(self,body,options=None):
        self.body = body
        self.options=options
    def __enter__(self):
        if self.options is None:
            self.handle = self.body.CreateKinBodyStateSaver()
        else:
            self.handle = self.body.CreateKinBodyStateSaver(self.options)
    def __exit__(self, type, value, traceback):
        self.handle.close()

class RobotStateSaver:
    """Saves/restores the robot state, use **with** statement.
    """
    def __init__(self,robot,options=None):
        self.robot = robot
        self.options = options
    def __enter__(self):
        if self.options is None:
            self.handle = self.robot.CreateRobotStateSaver()
        else:
            self.handle = self.robot.CreateRobotStateSaver(self.options)
    def __exit__(self, type, value, traceback):
        self.handle.close()

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


class openrave_exception(Exception):
    """wrap up the C++ openrave_exception"""
    def __init__( self, app_error ):
        Exception.__init__( self )
        self._pimpl = app_error
    def __str__( self ):
        return self._pimpl.message()
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

def fitCircle(points,geometric_refinement=True):
    """Very simple function to return the best fit circle.

    Used when fitting real data to joint trajectories. Currently this fits the algebraic distance, a further refinement step for geometric distance should be inserted.

    :return: center, radius, error
    """
    if points.shape[1] == 3:
        M = numpy.mean(points,0)
        points2=points-numpy.tile(M,[len(points),1])
        # use the svd to get the plane normal from the null space
        planenormal=numpy.linalg.svd(numpy.dot(points2.transpose(),points2))[2][-1,:]
        R=openravepy_int.rotationMatrixFromQuat(openravepy_int.quatRotateDirection([0,0,1],planenormal))
        points=numpy.dot(points2,R)
    x,error = numpy.linalg.lstsq(numpy.c_[points[:,0],points[:,1],numpy.ones(len(points))],-points[:,0]**2-points[:,1]**2)[0:2]
    if points.shape[1] == 3:
        # error should also include off-plane offsets!
        return M+numpy.array(numpy.dot(R,[-0.5*x[0],-0.5*x[1],0])), numpy.sqrt((x[0]**2+x[1]**2)/4-x[2]),error
    
    return numpy.array([-0.5*x[0],-0.5*x[1]]), numpy.sqrt((x[0]**2+x[1]**2)/4-x[2]),error

def fitSphere(points,geometric_refinement=True):
    """Very simple function to return the best fit sphere from 3D points.

    :return: [center, radius]
    """
    x,error = numpy.linalg.lstsq(numpy.c_[points,numpy.ones(len(points))],-numpy.sum(points**2,1))[0:2]
    return numpy.array([-0.5*x[0],-0.5*x[1], -0.5*x[2]]), numpy.sqrt((x[0]**2+x[1]**2+x[2]**2)/4-x[3]),error

def intersectSegments(L0,L1):
    """Computes the intersection point of two line segments
    """
    dL0 = L0[1]-L0[0]
    dL1 = L1[1]-L1[0]
    normal = numpy.cross(dL1,dL0)
    normal /= numpy.linalg.norm(normal)
    binormal = numpy.cross(normal,dL1)
    denom = numpy.dot(dL0,binormal)
    tL0 = numpy.dot(binormal,L1[0]-L0[0])/denom
    intersection0 = L0[0]*(1-tL0)+L0[1]*tL0
    dist = numpy.dot(intersection0-L1[0],normal)
    return intersection0-0.5*dist*normal,dist

