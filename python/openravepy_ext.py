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
from __future__ import with_statement # for python 2.5
import openravepy
import metaclass
import sys, os, numpy, copy
import optparse
try:
    import cPickle as pickle
except:
    import pickle

try:
    from itertools import izip
except ImportError:
    pass

def mkdir_recursive(newdir):
    """works the way a good mkdir should :)
        - already exists, silently complete
        - regular file in the way, raise an exception
        - parent directory(ies) does not exist, make them as well
    """
    if os.path.isdir(newdir):
        pass
    elif os.path.isfile(newdir):
        raise OSError("a file with the same name as the desired dir, '%s', already exists." % newdir)
    else:
        head, tail = os.path.split(newdir)
        if head and not os.path.isdir(head):
            mkdir_recursive(head)
        if tail:
            os.mkdir(newdir)

def ComputeGeodesicSphereMesh(radius=1.0,level=2):
    """Computes a geodesic sphere to a specified level. Returns the vertices and triangle indices"""
    GTS_M_ICOSAHEDRON_X = numpy.sqrt(numpy.sqrt(5)+1)/numpy.sqrt(2*numpy.sqrt(5))
    GTS_M_ICOSAHEDRON_Y = numpy.sqrt(2)/numpy.sqrt(5+numpy.sqrt(5))
    GTS_M_ICOSAHEDRON_Z = 0.0
    vertices = [numpy.array((+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y)),
                numpy.array((+GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                numpy.array((+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X)),
                numpy.array((+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X)),
                numpy.array((+GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                numpy.array((+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y)),
                numpy.array((-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X)),
                numpy.array((+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y)),
                numpy.array((-GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                numpy.array((-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X)),
                numpy.array((-GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                numpy.array((+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y))]
    triindices = [[0, 1, 2],[1, 3, 4],[3, 5, 6],[2, 4, 7],[6, 5, 8],[2, 7, 9],[5, 0, 8],[9, 7, 10],[1, 0, 5],[10, 7, 11],[3, 1, 5],[6, 10, 11],[3, 6, 11],[9, 10, 8],[4, 3, 11],[6, 8, 10],[7, 4, 11],[2, 1, 4],[8, 0, 9],[0, 2, 9]]
    while level > 0:
        level -= 1
        newindices = []
        mapnewinds = dict()
        for tri in triindices:
            # for ever tri, create 3 new vertices and 4 new triangles.
            v = [vertices[i] for i in tri]
            inds = []
            for j in range(3):
                key = (tri[j],tri[numpy.mod(j+1,3)])
                if key in mapnewinds:
                    inds.append(mapnewinds[key])
                else:
                    mapnewinds[key] = mapnewinds[key[::-1]] = len(vertices)
                    inds.append(len(vertices))
                    vnew = v[j]+v[numpy.mod(j+1,3)]
                    vertices.append(vnew/numpy.sqrt(sum(vnew**2)))
            newindices += [[tri[0],inds[0],inds[2]],[inds[0],tri[1],inds[1]],[inds[2],inds[0],inds[1]],[inds[2],inds[1],tri[2]]]
        triindices = newindices
    return radius*numpy.array(vertices),triindices

def ComputeBoxMesh(extents):
    """Computes a box mesh"""
    indices = numpy.reshape([0, 1, 2, 1, 2, 3, 4, 5, 6, 5, 6, 7, 0, 1, 4, 1, 4, 5, 2, 3, 6, 3, 6, 7, 0, 2, 4, 2, 4, 6, 1, 3, 5,3, 5, 7],(12,3))
    vertices = numpy.array(((extents[0],extents[1],extents[2]),
                            (extents[0],extents[1],-extents[2]),
                            (extents[0],-extents[1],extents[2]),
                            (extents[0],-extents[1],-extents[2]),
                            (-extents[0],extents[1],extents[2]),
                            (-extents[0],extents[1],-extents[2]),
                            (-extents[0],-extents[1],extents[2]),
                            (-extents[0],-extents[1],-extents[2])))
    return vertices,indices

def ComputeCylinderYMesh(radius,height,angledelta=0.1):
    """Computes a mesh of a cylinder oriented towards y-axis"""
    angles = numpy.arange(0,2*numpy.pi,angledelta)
    cangles = numpy.cos(angles)
    sangles = numpy.sin(angles)
    N = len(angles)
    vertices = numpy.c_[radius*numpy.tile(cangles,2),numpy.r_[numpy.tile(height*0.5,N),numpy.tile(-height*0.5,N)], radius*numpy.tile(sangles,2)]
    indices = []
    iprev = N-1
    for i in range(N):
        indices.append((iprev,i,iprev+N))
        indices.append((i,i+N,iprev+N))
        iprev = i
    return vertices,numpy.array(indices)

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
    return 2*numpy.vstack(((0.5-yy-zz)*trans[0]+(xy-zw)*trans[1]+(xz+yw)*trans[2],
                           (xy+zw)*trans[0]+(0.5-xx-zz)*trans[1]+(yz-xw)*trans[2],
                           (xz-yw)*trans[0]+(yz+xw)*trans[1]+(0.5-xx-yy)*trans[2]))

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

def sequence_cross_product(*sequences):
    # visualize an odometer, with "wheels" displaying "digits"...:
    wheels = map(iter, sequences)
    digits = [it.next( ) for it in wheels]
    while True:
        yield tuple(digits)
        for i in range(len(digits)-1, -1, -1):
            try:
                digits[i] = wheels[i].next( )
                break
            except StopIteration:
                wheels[i] = iter(sequences[i])
                digits[i] = wheels[i].next( )
        else:
            break

class MultiManipIKSolver(metaclass.AutoReloader):
    """Finds the simultaneous IK solutions of all disjoint manipulators (no manipulators share a joint)"""
    def __init__(self,manips):
        self.robot = manips[0].GetRobot()
        self.manips = manips
        indeplinksets=[set([l.GetName() for l in manip.GetIndependentLinks()]) for manip in self.manips]
        indeplinknames=indeplinksets[0].intersection(*indeplinksets[1:])
        alllinknames = set([l.GetName() for l in self.robot.GetLinks()])
        self.enablelinknames = [alllinknames.difference(indeplinksets[i]).union(indeplinknames) for i in range(len(self.manips))]
    
    def findMultiIKSolution(self,Tgrasps,envcheck=True):
        """return one ik solution"""
        assert(len(Tgrasps)==len(self.manips))
        with self.robot:
            alljointvalues = []
            for i,manip in enumerate(self.manips):
                # invalidate all manip links for now (since will be moving them later)
                for link in self.robot.GetLinks():
                    link.Enable(link.GetName() in self.enablelinknames[i])
                values=manip.FindIKSolutions(Tgrasps[i],envcheck)
                if values is not None and len(values) > 0:
                    alljointvalues.append(values)
                else:
                    return None
                
            # take all combinations of the solutions
            for sols in sequence_cross_product(*alljointvalues):
                for sol,manip in izip(sols,self.manips):
                    self.robot.SetJointValues(sol,manip.GetArmIndices()) 
                if not self.robot.CheckSelfCollision():
                    if not envcheck or not self.robot.GetEnv().CheckCollision(self.robot):
                        return sols
                    
            return None

class SpaceSampler(metaclass.AutoReloader):
    def __init__(self):
         self.faceindices = self.facenumr = self.facenump = None
    @staticmethod
    def computeSepration(qarray):
        """used to test separation of a set of quaternions"""
        qdists = numpy.zeros((qarray.shape[0],qarray.shape[0]))
        for i,q in enumerate(qarray):
            dists = numpy.abs(numpy.dot(qarray[(i+1):],q))
            qdists[i,(i+1):] = qdists[(i+1):,i] = dists
        qmaxdists = numpy.max(qdists,axis=0)
        return numpy.arccos(numpy.min(1.0,numpy.max(qmaxdists))), numpy.arccos(numpy.min(1.0,numpy.min(qmaxdists)))
    def computeFaceIndices(self,N):
        if self.faceindices is None or len(self.faceindices[0]) < N:
            indices = numpy.arange(N**2)
            # separate the odd and even bits into odd,even
            maxiter = int(numpy.log2(len(indices)))
            oddbits = numpy.zeros(N**2,int)
            evenbits = numpy.zeros(N**2,int)
            mult = 1
            for i in range(maxiter):
                oddbits += (indices&1)*mult
                evenbits += mult*((indices&2)/2)
                indices >>= 2
                mult *= 2
            self.faceindices = [oddbits+evenbits,oddbits-evenbits]
        if self.facenumr is None or len(self.facenumr) != N*12:
            self.facenumr = numpy.reshape(numpy.transpose(numpy.tile([2,2,2,2,3,3,3,3,4,4,4,4],(N,1))),N*12)
            self.facenump = numpy.reshape(numpy.transpose(numpy.tile([1,3,5,7,0,2,4,6,1,3,5,7],(N,1))),N*12)
    def sampleS2(self,level=0,angledelta=None):
        """uses healpix algorithm with ordering from Yershova et. al. 2009 journal paper"""
        if angledelta is not None:
            # select the best sphere level matching angledelta;
            # level,delta: 
            # [0, 1.0156751592381095]
            # [1, 0.5198842203445676]
            # [2, 0.25874144949351713]
            # [3, 0.13104214473149575]
            # [4, 0.085649339187184162]
            level=max(0,int(0.5-numpy.log2(angledelta)))
        Nside = 2**level
        Nside2 = Nside**2
        N = 12*Nside**2
        self.computeFaceIndices(Nside**2)        
        # compute sphere z coordinate
        jr = self.facenumr*Nside-numpy.tile(self.faceindices[0][0:Nside2],12)-1
        nr = numpy.tile(Nside,N)
        z = 2*(2*Nside-jr)/(3.0*Nside)
        kshift = numpy.mod(jr-Nside,2)
        # north pole test
        northpoleinds = numpy.flatnonzero(jr<Nside)
        nr[northpoleinds] = jr[northpoleinds]
        z[northpoleinds] = 1.0 - nr[northpoleinds]**2*(1.0/(3.0*Nside2))
        kshift[northpoleinds] = 0
        # south pole test
        southpoleinds = numpy.flatnonzero(jr>3*Nside)
        nr[southpoleinds] = 4*Nside - jr[southpoleinds]
        z[southpoleinds] = -1.0 + nr[southpoleinds]**2*(1.0/(3.0*Nside2))
        kshift[southpoleinds] = 0
        # compute pfi
        facenump = numpy.reshape(numpy.transpose(numpy.tile([1,3,5,7,0,2,4,6,1,3,5,7],(Nside2,1))),N)
        jp = (self.facenump*nr+numpy.tile(self.faceindices[1][0:Nside2],12)+1+kshift)/2
        jp[jp>4*Nside] -= 4*Nside
        jp[jp<1] += 4*Nside
        return numpy.arccos(z),(jp-(kshift+1)*0.5)*((0.5*numpy.pi)/nr)
    @staticmethod
    def hopf2quat(hopfarray):
        """convert hopf rotation coordinates to quaternion"""
        half0 = hopfarray[:,0]*0.5
        half2 = hopfarray[:,2]*0.5
        c0 = numpy.cos(half0)
        c2 = numpy.cos(half2)
        s0 = numpy.sin(half0)
        s2 = numpy.sin(half2)
        return numpy.c_[c0*c2,c0*s2,s0*numpy.cos(hopfarray[:,1]+half2),s0*numpy.sin(hopfarray[:,1]+half2)]
    def sampleSO3(self,level=0,quatdelta=None):
        """Uniformly Sample 3D Rotations.
        If quatdelta is specified, will compute the best level aiming for that average quaternion distance.
        Algorithm From
        A. Yershova, S. Jain, S. LaValle, J. Mitchell "Generating Uniform Incremental Grids on SO(3) Using the Hopf Fibration", International Journal of Robotics Research, Nov 13, 2009.
        """
        if quatdelta is not None:
            # level=0, quatdist = 0.5160220
            # level=1: quatdist = 0.2523583
            # level=2: quatdist = 0.120735
            level=max(0,int(-0.5-numpy.log2(quatdelta)))
        s1samples,step = numpy.linspace(0.0,2*numpy.pi,6*(2**level),endpoint=False,retstep=True)
        s1samples += step*0.5
        theta,pfi = self.sampleS2(level)
        band = numpy.zeros((len(s1samples),3))
        band[:,2] = s1samples
        qarray = numpy.zeros((0,4))
        for i in range(len(theta)):
            band[:,0] = theta[i]
            band[:,1] = pfi[i]
            qarray = numpy.r_[qarray,self.hopf2quat(band)]
        return qarray
    @staticmethod
    def sampleR3lattice(averagedist,boxdims):
        """low-discrepancy lattice sampling in using the roots of x^3-3x+1.
        The samples are evenly distributed with an average distance of averagedist inside the box with extents boxextents.
        Algorithim from "Geometric Discrepancy: An Illustrated Guide" by Jiri Matousek"""
        roots = numpy.array([2.8793852415718155,0.65270364466613917,-0.53208888623795614])
        bases = c_[numpy.ones(3),roots,roots**2]
        tbases = numpy.transpose(bases)
        boxextents = 0.5*numpy.array(boxdims)
        # determine the input bounds, which can be very large and inefficient...
        bounds = numpy.array(((boxextents[0],boxextents[1],boxextents[2]),
                              (boxextents[0],boxextents[1],-boxextents[2]),
                              (boxextents[0],-boxextents[1],boxextents[2]),
                              (boxextents[0],-boxextents[1],-boxextents[2]),
                              (-boxextents[0],boxextents[1],boxextents[2]),
                              (-boxextents[0],boxextents[1],-boxextents[2]),
                              (-boxextents[0],-boxextents[1],boxextents[2]),
                              (-boxextents[0],-boxextents[1],-boxextents[2])))
        inputbounds = numpy.max(dot(bounds,linalg.inv(tbases)),0)
        scale = averagedist/numpy.sqrt(3.0)
        X,Y,Z = numpy.mgrid[-inputbounds[0]:inputbounds[0]:scale,-inputbounds[1]:inputbounds[1]:scale,-inputbounds[2]:inputbounds[2]:scale]
        p = numpy.c_[X.flat,Y.flat,Z.flat]
        pts = numpy.dot(p,tbases)
        ptsabs = numpy.abs(pts)
        newpts = pts[numpy.logical_and(ptsabs[:,0]<=boxextents[0],numpy.logical_and(ptsabs[:,1]<=boxextents[1] ,ptsabs[:,2]<=boxextents[2]))]
        newpts[:,0] += boxextents[0]
        newpts[:,1] += boxextents[1]
        newpts[:,2] += boxextents[2]
        return newpts
    @staticmethod
    def sampleR3(averagedist,boxdims):
        """low-discrepancy sampling using primes.
        The samples are evenly distributed with an average distance of averagedist inside the box with dimensions boxdims.
        Algorithim from "Geometric Discrepancy: An Illustrated Guide" by Jiri Matousek"""
        minaxis = numpy.argmin(boxdims)
        maxaxis = numpy.argmax(boxdims)
        meddimdist = numpy.sort(boxdims)[1]
        # convert average distance to number of samples.... do simple 3rd degree polynomial fitting...
        x = meddimdist/averagedist
        if x < 25.6:
            N = int(numpy.polyval([ -3.50181522e-01,   2.70202333e+01,  -3.10449514e+02, 1.07887093e+03],x))
        elif x < 36.8:
            N = int(numpy.polyval([  4.39770585e-03,   1.10961031e+01,  -1.40066591e+02, 1.24563464e+03],x))
        else:
            N = int(numpy.polyval([5.60147111e-01,  -8.77459988e+01,   7.34286834e+03, -1.67779452e+05],x))
        pts = numpy.zeros((N,3))
        pts[:,0] = numpy.linspace(0.0,meddimdist,N)
        pts[:,1] = meddimdist*numpy.mod(0.5+0.5*numpy.sqrt(numpy.arange(0,5.0*N,5.0)),1.0)
        pts[:,2] = meddimdist*numpy.mod(0.5+numpy.sqrt(numpy.arange(0,13.0*N,13.0)),1.0)
        if boxdims[minaxis] < meddimdist:
            pts = pts[pts[:,minaxis]<=boxdims[minaxis],:]
        if boxdims[maxaxis] > meddimdist:
            # have to copy across the max dimension
            numfullcopies = numpy.floor(boxdims[maxaxis]/meddimdist)
            oldpts = pts
            pts = numpy.array(oldpts)
            for i in range(int(numfullcopies)-1):
                oldpts[:,maxaxis] += meddimdist
                pts = numpy.r_[pts,oldpts]
            if boxdims[maxaxis]/meddimdist > numfullcopies:
                oldpts[:,maxaxis] += meddimdist
                pts = numpy.r_[pts,oldpts[oldpts[:,maxaxis]<=boxdims[maxaxis],:]]
        return pts

class KinBodyStateSaver:
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

class OpenRAVEModel(metaclass.AutoReloader):
    """
    .. lang-block:: en

      The base class defining the structure of the openrave database generators.

    .. lang-block:: ja
    
      データベース生成の構造を定義した基本クラス

    """
    def __init__(self,robot):
        self.robot = robot
        self.env = self.robot.GetEnv()
        try:
            self.manip = self.robot.GetActiveManipulator()
        except:
            self.manip = None
    def clone(self,envother):
        clone = copy.copy(self)
        clone.env = envother
        clone.robot = clone.env.GetRobot(self.robot.GetName())
        clone.manip = clone.robot.GetManipulators(self.manip.GetName())[0] if not self.manip is None else None
        return clone
    def has(self):
        raise NotImplementedError()
    def getfilename(self):
        return os.path.join(self.env.GetHomeDirectory(),'robot.'+self.robot.GetKinematicsGeometryHash())
    def load(self):
        if not os.path.isfile(self.getfilename()):
            return None
        try:
            modelversion,params = pickle.load(open(self.getfilename(), 'r'))
            if modelversion == self.getversion():
                return params
            else:
                print 'version is wrong ',modelversion,'!=',self.getversion()
        except:
            pass
        return None
    def getversion(self):
        return 0
    def save(self,params):
        print 'saving model to %s'%self.getfilename()
        mkdir_recursive(os.path.split(self.getfilename())[0])
        pickle.dump((self.getversion(),params), open(self.getfilename(), 'w'))
    def generate(self):
        raise NotImplementedError()
    def show(self,options=None):
        raise NotImplementedError()
    def autogenerate(self,options=None):
        """Caches parameters for most commonly used robots/objects and starts the generation process for them"""
        raise NotImplementedError()
    @staticmethod
    def CreateOptionParser(useManipulator=True):
        parser = optparse.OptionParser(description='OpenRAVE Database Generator.')
        ogroup = optparse.OptionGroup(parser,"OpenRAVE Environment Options")
        ogroup.add_option('--collision', action="store",type='string',dest='collision',default=None,
                          help='Default collision checker to use')
        ogroup.add_option('--physics', action="store",type='string',dest='physics',default=None,
                          help='Default physics engine to use')
        ogroup.add_option('--debug','-d', action="store",type='string',dest='debug',default=None,
                          help='Debug level')
        parser.add_option_group(ogroup)
        dbgroup = optparse.OptionGroup(parser,"OpenRAVE Database Generator General Options")
        dbgroup.add_option('--show',action='store_true',dest='show',default=False,
                           help='Graphically shows the built model')
        dbgroup.add_option('--getfilename',action="store_true",dest='getfilename',default=False,
                           help='If set, will return the final database filename where all data is stored')
        dbgroup.add_option('--gethas',action="store_true",dest='gethas',default=False,
                           help='If set, will exit with 0 if datafile is generated and up to date, otherwise will return a 1. This will require loading the model and checking versions, so might be a little slow.')
        dbgroup.add_option('--robot',action='store',type='string',dest='robot',default=os.getenv('OPENRAVE_ROBOT',default='robots/barrettsegway.robot.xml'),
                           help='OpenRAVE robot to load (default=%default)')
        if useManipulator:
            dbgroup.add_option('--manipname',action='store',type='string',dest='manipname',default=None,
                               help='The name of the manipulator on the robot to use')
        parser.add_option_group(dbgroup)
        return parser
    @staticmethod
    def RunFromParser(Model,env=None,parser=None,args=None,**kwargs):
        if parser is None:
            parser = OpenRAVEModel.CreateOptionParser()
        (options, args) = parser.parse_args(args=args)
        destroyenv = False
        loadplugins=True
        if options.getfilename:
            # don't want unnecessary messages cluttering the console...
            openravepy.raveSetDebugLevel(openravepy.DebugLevel.Fatal)
            loadplugins = False
        if options.gethas:
            openravepy.raveSetDebugLevel(openravepy.DebugLevel.Fatal)
        if env is None:
            env = openravepy.Environment(loadplugins)
            destroyenv = True
        try:
            with env:
                if options.collision is not None:
                    collision = env.CreateCollisionChecker(options.collision)
                    if collision is not None:
                        env.SetCollisionChecker(collision)
                if options.physics is not None:
                    physics = env.CreatePhysicsEngine(options.physics)
                    if physics is not None:
                        env.SetPhysicsEngine(physics)
                if options.debug is not None and not options.getfilename:
                    for debuglevel in [openravepy.DebugLevel.Fatal,openravepy.DebugLevel.Error,openravepy.DebugLevel.Warn,openravepy.DebugLevel.Info,openravepy.DebugLevel.Debug,openravepy.DebugLevel.Verbose]:
                        if (not options.debug.isdigit() and options.debug.lower() == debuglevel.name.lower()) or (options.debug.isdigit() and int(options.debug) == int(debuglevel)):
                            openravepy.raveSetDebugLevel(debuglevel)
                            break
                robot = env.ReadRobotXMLFile(options.robot)
                env.AddRobot(robot)
                robot.SetTransform(numpy.eye(4))
                if hasattr(options,'manipname'):
                    if options.manipname is None:
                        # prioritize manipulators with ik solvers
                        indices = [i for i,m in enumerate(robot.GetManipulators()) if m.HasIKSolver()]
                        if len(indices) > 0:
                            robot.SetActiveManipulator(indices[0])
                    else:
                        robot.SetActiveManipulator([i for i,m in enumerate(robot.GetManipulators()) if m.GetName()==options.manipname][0])
            model = Model(robot=robot)
            if options.getfilename:
                print model.getfilename()
                sys.exit(0)
            if options.gethas:
                hasmodel=model.load()
                print int(hasmodel)
                env.Destroy()
                sys.exit(not hasmodel)
            if options.show:
                if not model.load():
                    raise ValueError('failed to find cached model %s'%model.getfilename())
                model.show(options=options)
                return
            model.autogenerate(options=options)
        finally:
            if destroyenv:
                env.Destroy()

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

class pyann_exception(Exception):
    """wrap up the C++ pyann_exception"""
    def __init__( self, app_error ):
        Exception.__init__( self )
        self._pimpl = app_error
    def __str__( self ):
        return self._pimpl.message()
    def __getattribute__(self, attr):
        my_pimpl = super(pyann_exception, self).__getattribute__("_pimpl")
        try:
            return getattr(my_pimpl, attr)
        except AttributeError:
            return super(pyann_exception,self).__getattribute__(attr)

class planning_error(Exception):
    def __init__(self,parameter=''):
        self.parameter = parameter
    def __str__(self):
        return 'openrave planning_error: '+repr(self.parameter)
