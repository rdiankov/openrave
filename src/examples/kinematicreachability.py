#!/usr/bin/env python
# Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
from openravepy import *
from numpy import *
import time,pickle
from optparse import OptionParser

class ReachabilityModel(object):
    def __init__(self,robotfile=None,manipname=None,showviewer=False):
        self.orenv = Environment()
        if showviewer:
            self.orenv.SetViewer('qtcoin')
        self.robotfile = None
        self.orrobot = None
        if robotfile is not None:
            self.loadrobot(robotfile,manipname)
                
        self.reachabilitystats = None
        self.reachabilitydensity3d = None

    def save(self,filename='reachability.pp'):
        f = open(filename,'w')
        pickle.dump((self.robotfile,self.manipname,self.reachabilitystats,self.reachabilitydensity3d,self.pointscale), f)
        f.close()
    def load(self,filename='reachability.pp'):
        f = open(filename, 'r')
        self.robotfile,self.manipname,self.reachabilitystats,self.reachabilitydensity3d,self.pointscale = pickle.load(f)
        self.loadrobot(self.robotfile,self.manipname)

    def loadrobot(self,robotfile,manipname):
        if self.orrobot is not None:
            self.orenv.RemoveKinBody(self.orrobot)
            self.orrobot = None
        self.orrobot = self.orenv.ReadRobotXML(robotfile)
        if self.orrobot is None:
            raise ValueError('failed to open %s openrave file'%robotfile)
        self.robotfile = robotfile
        self.orenv.AddRobot(self.orrobot)
        self.orrobot.SetTransform(eye(4))
        self.trimesh = self.orenv.Triangulate(self.orrobot)

        if manipname is None:
            manips = [m for m in self.orrobot.GetManipulators() if m.HasIKSolver()]
            if len(manips) == 0:
                raise ValueError('no valid manipulator')
            self.manip = manips[0]
        else:
            manips = [m for m in self.orrobot.GetManipulators() if m.GetName()==manipname and m.HasIKSolver()]
            if len(manips) == 0:
                raise ValueError('no valid manipulator')
            self.manip = manips[0]
        self.manipname = self.manip.GetName()

    def ComputeReachability(self,maxradius):
        starttime = time.time()
        Tgrasp = dot(linalg.inv(self.manip.GetBase().GetTransform()),self.manip.GetEndEffectorTransform())
        armlength = sqrt(sum(Tgrasp[0:3,3]**2))
        if maxradius is None:
            maxradius = 1.5*armlength
        else:
            maxradius = max(1.5*armlength,maxradius)
        print 'radius: ',maxradius

        allpoints,insideinds,shape,self.pointscale = self.UniformlySampleSpace(maxradius,delta=0.02)
        rotations = self.GetUniformRotations(spherelevel=0,rolldelta=pi/8.0)
        T = zeros((3,4))

        reachabilitydensity3d = zeros(prod(shape))
        self.reachabilitystats = []
        self.orenv.LockPhysics(True)
        try:
            for i,ind in enumerate(insideinds):
                numvalid = 0
                T[0:3,3] = allpoints[ind]
                for rotation in rotations:
                    T[0:3,0:3] = rotation
                    solution = self.manip.FindIKSolution(T,True)
                    if solution is not None:
                        self.reachabilitystats.append(T)
                        numvalid += 1
                if mod(i,1000)==0:
                    print '%d/%d'%(i,len(insideinds))
                reachabilitydensity3d[ind] = numvalid/float(len(rotations))
        finally:
            self.orenv.LockPhysics(False)
        self.reachabilitydensity3d = reshape(reachabilitydensity3d,shape)
        print 'reachability finished in %fs'%(time.time()-starttime)

    def showdensity(self,showrobot=True,contours=[0.1,0.5,0.9,0.99],figureid=1, xrange=None):
        from enthought.mayavi import mlab
        mlab.figure(figureid,fgcolor=(0,0,0), bgcolor=(1,1,1),size=(1024,768))
        mlab.clf()
        if xrange is None:
            offset = array((0,0,0))
            src = mlab.pipeline.scalar_field(self.reachabilitydensity3d)
        else:
            offset = array((xrange[0]-1,0,0))
            src = mlab.pipeline.scalar_field(r_[zeros((1,)+self.reachabilitydensity3d.shape[1:]),self.reachabilitydensity3d[xrange,:,:],zeros((1,)+self.reachabilitydensity3d.shape[1:])])
            
        for c in contours:
            mlab.pipeline.iso_surface(src,contours=[c],opacity=c*0.3)
        #mlab.pipeline.volume(mlab.pipeline.scalar_field(self.reachabilitydensity3d*100))
        if showrobot:
            v = self.pointscale[0]*self.trimesh.vertices+self.pointscale[1]
            mlab.triangular_mesh(v[:,0]-offset[0],v[:,1]-offset[1],v[:,2]-offset[2],self.trimesh.indices,color=(0.5,0.5,0.5))

    def UniformlySampleSpace(self,maxradius,delta=0.02):
        nsteps = floor(maxradius/delta)
        X,Y,Z = mgrid[-nsteps:nsteps,-nsteps:nsteps,-nsteps:nsteps]
        allpoints = c_[X.flat,Y.flat,Z.flat]*delta
        insideinds = flatnonzero(sum(allpoints**2,1)<maxradius**2)
        return allpoints,insideinds,X.shape,array((1.0/delta,nsteps))

    def GetUniformRotations(self,spherelevel=2,rolldelta=pi/8.0):
        """Generate a discreteized uniform sampling of rotations using geodesic spheres"""
        vertices,triindices = self.GetGeodesicSphere(spherelevel=spherelevel)
        rotations = []
        for d in vertices:
            up = array((0,1,0)) - d*d[1]
            lup = sum(up**2)
            if lup < 0.0001:
                up = array((0,0,1)) - d*d[2]
                lup = sum(up**2)
            up = up / sqrt(lup)
            right = cross(up,d)
            Rbase = c_[right,up,d]
            for roll in arange(0,2*pi,rolldelta):
                Rz = array(((cos(roll),-sin(roll),0),(sin(roll),cos(roll),0),(0,0,1)))
                rotations.append(dot(Rbase,Rz))
        return rotations

    def GetGeodesicSphere(self,spherelevel=2):
        GTS_M_ICOSAHEDRON_X = sqrt(sqrt(5)+1)/sqrt(2*sqrt(5))
        GTS_M_ICOSAHEDRON_Y = sqrt(2)/sqrt(5+sqrt(5))
        GTS_M_ICOSAHEDRON_Z = 0.0
        vertices = [array((+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y)),
                    array((+GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                    array((+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X)),
                    array((+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X)),
                    array((+GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                    array((+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y)),
                    array((-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X)),
                    array((+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y)),
                    array((-GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                    array((-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X)),
                    array((-GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                    array((+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y))]
        triindices = [[0, 1, 2],[1, 3, 4],[3, 5, 6],[2, 4, 7],[6, 5, 8],[2, 7, 9],[5, 0, 8],[9, 7, 10],[1, 0, 5],[10, 7, 11],[3, 1, 5],[6, 10, 11],[3, 6, 11],[9, 10, 8],[4, 3, 11],[6, 8, 10],[7, 4, 11],[2, 1, 4],[8, 0, 9],[0, 2, 9]]
        while spherelevel > 0:
            spherelevel -= 1
            newindices = []
            mapnewinds = dict()
            for tri in triindices:
                # for ever tri, create 3 new vertices and 4 new triangles.
                v = [vertices[i] for i in tri]
                inds = []
                for j in range(3):
                    key = (tri[j],tri[mod(j+1,3)])
                    if key in mapnewinds:
                        inds.append(mapnewinds[key])
                    else:
                        mapnewinds[key] = mapnewinds[key[::-1]] = len(vertices)
                        inds.append(len(vertices))
                        vnew = v[j]+v[mod(j+1,3)]
                        vertices.append(vnew/sqrt(sum(vnew**2)))
                newindices += [[tri[0],inds[0],inds[2]],[inds[0],tri[1],inds[1]],[inds[2],inds[0],inds[1]],[inds[2],inds[1],tri[2]]]
            triindices = newindices
        return array(vertices),triindices

if __name__=='__main__':
    parser = OptionParser(description='Computes the reachability region of a robot and python pickles it into a file.')
    parser.add_option('--robot',action='store',type='string',dest='robot',default='robots/barrettwam.robot.xml',
                      help='OpenRAVE robot to load')
    parser.add_option('--savereachability', action='store',type='string',dest='savereachability',default='reachability.pp',
                      help='Compute the statistics and save in this file')
    parser.add_option('--loadreachability', action='store',type='string',dest='loadreachability',default=None,
                      help='Load the reachability statistics')
    parser.add_option('--manipname',action='store',type='string',dest='manipname',default=None,
                      help='The name of the manipulator')
    parser.add_option('--maxradius',action='store',type='float',dest='maxradius',default=None,
                      help='The name of the manipulator')
    parser.add_option('--show',action='store_true',dest='show',default=False,
                      help='If set, uses mayavi (v3+) to display the reachability')
    (options, args) = parser.parse_args()
    
    model = ReachabilityModel(options.robot,manipname=options.manipname)
    if options.loadreachability:
        model.load(options.loadreachability)
    if options.savereachability:
        model.ComputeReachability(maxradius=options.maxradius)
        model.save(options.savereachability)
    if options.show:
        model.showdensity()
        input('press any key to exit')
