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
import openravepy
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

def with_destroy(fn):
    """a decorator that always calls openravepy.RaveDestroy at the function end"""
    def newfn(*args,**kwargs):
        try:
            return fn(*args,**kwargs)
        finally:
            openravepy.RaveDestroy()
    newfn.__doc__ = fn.__doc__
    newfn.__module__ = fn.__module__
    newfn.__name__ = fn.__name__
    return newfn

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
            try:
                os.mkdir(newdir)
            except OSError:
                # race conditions could still lead to such errors...
                pass

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
        R=openravepy.rotationMatrixFromQuat(openravepy.quatRotateDirection([0,0,1],planenormal))
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

def sequence_cross_product(*sequences):
    """iterates through the cross product of all items in the sequences"""
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

def TSP(solutions,distfn):
    """solution to travelling salesman problem. orders the set of solutions such that visiting them one after another is fast.
    """
    newsolutions = numpy.array(solutions)
    for i in range(newsolutions.shape[0]-2):
        n = newsolutions.shape[0]-i-1
        dists = [distfn(newsolutions[i,:],newsolutions[j,:]) for j in range(i+1,newsolutions.shape[0])]
        minind = numpy.argmin(dists)+i+1
        sol = numpy.array(newsolutions[i+1,:])
        newsolutions[i+1,:] = newsolutions[minind,:]
        newsolutions[minind,:] = sol
    return newsolutions

class MultiManipIKSolver:
    """Finds the simultaneous IK solutions of all disjoint manipulators (no manipulators share a joint).

    The class is extremely useful in dual-manipulation IK solutions. It also handled grabbed bodies correctly.
    """
    def __init__(self,manips):
        self.robot = manips[0].GetRobot()
        self.manips = manips
        indeplinksets=[set([l for l in manip.GetIndependentLinks()]) for manip in self.manips]
        indeplinknames=indeplinksets[0].intersection(*indeplinksets[1:])
        alllinknames = set([l for l in self.robot.GetLinks()])
        self.enablelinknames = [alllinknames.difference(indeplinksets[i]).union(indeplinknames) for i in range(len(self.manips))]
    
    def findMultiIKSolution(self,Tgrasps,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions):
        """Return one set collision-free ik solutions for all manipulators.

        Method always checks self-collisions.
        
        :param Tgrasps: a list of all the end effector transforms of each of the manipualtors
        :param filteroptions: a bitmask of `IkFilterOptions`
        """
        assert(len(Tgrasps)==len(self.manips))
        with self.robot:
            alljointvalues = []
            grabbed = self.robot.GetGrabbed()
            statesavers = [body.CreateKinBodyStateSaver() for body in grabbed]
            try:
                with RobotStateSaver(self.robot): # for storing enabled state
                    for i,manip in enumerate(self.manips):
                        # invalidate all links that are controlled by the other manipulators
                        for link in self.robot.GetLinks():
                            link.Enable(link in self.enablelinknames[i])
                        # enable only the grabbed bodies of this manipulator
                        for body in grabbed:
                            body.Enable(manip.IsGrabbing(body))
                        values=manip.FindIKSolutions(Tgrasps[i],filteroptions)
                        if values is not None and len(values) > 0:
                            alljointvalues.append(values)
                        else:
                            return None
                        
            finally:
                del statesavers # destroy them
            
            for sols in sequence_cross_product(*alljointvalues):
                for sol,manip in izip(sols,self.manips):
                    self.robot.SetDOFValues(sol,manip.GetArmIndices()) 
                if not self.robot.CheckSelfCollision():
                    if not (filteroptions&openravepy.IkFilterOptions.CheckEnvCollisions) or not self.robot.GetEnv().CheckCollision(self.robot):
                        return sols
            
            return None


class SpaceSampler:
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

class OpenRAVEGlobalArguments:
    """manages a global set of command-line options applicable to all openrave environments"""
    @staticmethod
    def addOptions(parser,testmode=True):
        ogroup = optparse.OptionGroup(parser,"OpenRAVE Environment Options")
        ogroup.add_option('--loadplugin', action="append",type='string',dest='_loadplugins',default=[],
                          help='List all plugins and the interfaces they provide.')
        ogroup.add_option('--collision', action="store",type='string',dest='_collision',default=None,
                          help='Default collision checker to use')
        ogroup.add_option('--physics', action="store",type='string',dest='_physics',default=None,
                          help='physics engine to use (default=%default)')
        ogroup.add_option('--viewer', action="store",type='string',dest='_viewer',default=None,
                          help='viewer to use (default=qtcoin)' )
        ogroup.add_option('--server', action="store",type='string',dest='_server',default=None,
                          help='server to use (default=None).')
        ogroup.add_option('--serverport', action="store",type='int',dest='_serverport',default=4765,
                          help='port to load server on (default=%default).')
        ogroup.add_option('--level','-l', action="store",type='string',dest='_level',default=None,
                          help='Debug level, one of (%s)'%(','.join(str(debugname).lower() for debuglevel,debugname in openravepy.DebugLevel.values.iteritems())))
        if testmode:
            ogroup.add_option('--testmode', action="store_true",dest='testmode',default=False,
                              help='if set, will run the program in a finite amount of time and spend computation time validating results. Used for testing')
        parser.add_option_group(ogroup)
    @staticmethod
    def parseGlobal(options,**kwargs):
        """Parses all global options independent of the environment"""
        if options._level is not None:
            for debuglevel,debugname in openravepy.DebugLevel.values.iteritems():
                if (not options._level.isdigit() and options._level.lower() == debugname.name.lower()) or (options._level.isdigit() and int(options._level) == int(debuglevel)):
                    openravepy.RaveSetDebugLevel(debugname)
                    break
    
    @staticmethod
    def parseEnvironment(options,env,defaultviewer=False,returnviewer=False,**kwargs):
        """Parses all options that affect the environment. If returnviewer is set, will return the viewer to set instead of setting it"""
        try:
            if options._collision:
                cc = openravepy.RaveCreateCollisionChecker(env,options._collision)
                if cc is not None:
                    env.SetCollisionChecker(cc)
        except openrave_exception, e:
            print e
        try:
            if options._physics:
                ph = openravepy.RaveCreatePhysicsEngine(env,options._physics)
                if ph is not None:
                    env.SetPhysicsEngine(ph)
        except openrave_exception, e:
            print e
        try:
            if options._server:
                sr = openravepy.RaveCreateModule(env,options._server)
                if sr is not None:
                    env.AddModule(sr,'%d'%options._serverport)
        except openrave_exception, e:
            print e
        try:
            viewer=None
            if options._viewer is not None:
                if len(options._viewer) > 0:
                    viewer=options._viewer
            elif defaultviewer:
                viewer='qtcoin'
            if returnviewer:
                return viewer
            elif viewer is not None:
                env.SetViewer(viewer)
        except openrave_exception, e:
            print e
    @staticmethod
    def parseAndCreate(options,createenv=openravepy.Environment,**kwargs):
        """Parse all options and create the global Environment. The left over arguments are passed to the parse functions"""
        openravepy.RaveInitialize(True)
        for plugin in options._loadplugins:
            openravepy.RaveLoadPlugin(plugin)
        OpenRAVEGlobalArguments.parseGlobal(options,**kwargs)
        if createenv is None:
            return None
        env = createenv()
        OpenRAVEGlobalArguments.parseEnvironment(options,env,**kwargs)
        return env

# this is necessary due to broken boost python pickle support for enums
def _tuple2enum(enum, value):
    enum = getattr(openravepy, enum)
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
    pickle(openravepy.IkParameterizationType,reduce_enum)
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

# deprecated
openravepy.Problem = openravepy.Module
