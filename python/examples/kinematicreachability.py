#!/usr/bin/env python
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

from openravepy import *
from numpy import *
import time,pickle
from optparse import OptionParser

class ReachabilityModel(metaclass.AutoReloader):
    def __init__(self,env,robot,target=None):
        self.env = env
        self.robot = robot
        self.manip = self.robot.GetActiveManipulator()
        self.trimesh = self.env.Triangulate(self.robot)
        self.reachabilitystats = None
        self.reachabilitydensity3d = None
        self.pointscale = None

    def has(self):
        return len(self.reachabilitydensity3d) > 0

    def getfilename(self):
        return os.path.join(self.env.GetHomeDirectory(),self.robot.GetRobotStructureHash(),self.manip.GetName() + '.reachability.pp')

    def load(self):
        if not os.path.isfile(self.getfilename()):
            return False
        self.reachabilitystats,self.reachabilitydensity3d,self.pointscale = pickle.load(open(self.getfilename(), 'r'))

    def save(self):
        print 'saving grasps to %s'%self.getfilename()
        mkdir_recursive(os.path.join(self.env.GetHomeDirectory(),self.robot.GetRobotStructureHash()))
        pickle.dump((self.reachabilitystats,self.reachabilitydensity3d,self.pointscale), open(self.getfilename(), 'w'))

    def generate(self,maxradius,translationonly=False,xyzdelta=0.02,rolldelta=pi/8.0):
        starttime = time.time()
        Tgrasp = dot(linalg.inv(self.manip.GetBase().GetTransform()),self.manip.GetEndEffectorTransform())
        armlength = sqrt(sum(Tgrasp[0:3,3]**2))
        if maxradius is None:
            maxradius = 1.5*armlength
        print 'radius: %f'%maxradius

        allpoints,insideinds,shape,self.pointscale = self.UniformlySampleSpace(maxradius,delta=xyzdelta)
        rotations = [eye(3)] if translationonly else self.GetUniformRotations(spherelevel=0,rolldelta=rolldelta)
        T = eye(4)

        reachabilitydensity3d = zeros(prod(shape))
        self.reachabilitystats = []
        with self.env:
            for i,ind in enumerate(insideinds):
                numvalid = 0
                T[0:3,3] = allpoints[ind]
                for rotation in rotations:
                    T[0:3,0:3] = rotation
                    solutions = self.manip.FindIKSolutions(T,True)
                    if solutions is not None:
                        self.reachabilitystats.append(allpoints[ind])
                        numvalid += len(solutions)
                if mod(i,1000)==0:
                    print '%d/%d'%(i,len(insideinds))
                reachabilitydensity3d[ind] = numvalid/(50.0*len(rotations))
        self.reachabilitydensity3d = reshape(reachabilitydensity3d,shape)
        print 'reachability finished in %fs'%(time.time()-starttime)

    def show(self,showrobot=True,contours=[0.1,0.5,0.9,0.99],opacity=None,figureid=1, xrange=None):
        mlab.figure(figureid,fgcolor=(0,0,0), bgcolor=(1,1,1),size=(1024,768))
        mlab.clf()
        reachabilitydensity3d = minimum(self.reachabilitydensity3d,1.0)
        reachabilitydensity3d[0,0,0] = 1 # have at least one point be at the maximum
        if xrange is None:
            offset = array((0,0,0))
            src = mlab.pipeline.scalar_field(self.reachabilitydensity3d)
        else:
            offset = array((xrange[0]-1,0,0))
            src = mlab.pipeline.scalar_field(r_[zeros((1,)+self.reachabilitydensity3d.shape[1:]),self.reachabilitydensity3d[xrange,:,:],zeros((1,)+self.reachabilitydensity3d.shape[1:])])
            
        for i,c in enumerate(contours):
            mlab.pipeline.iso_surface(src,contours=[c],opacity=min(1,0.5*c if opacity is None else opacity[i]))
        #mlab.pipeline.volume(mlab.pipeline.scalar_field(self.reachabilitydensity3d*100))
        if showrobot:
            v = self.pointscale[0]*self.trimesh.vertices+self.pointscale[1]
            mlab.triangular_mesh(v[:,0]-offset[0],v[:,1]-offset[1],v[:,2]-offset[2],self.trimesh.indices,color=(0.5,0.5,0.5))

    def autogenerate(self):
        """Caches parameters for most commonly used robots and starts the generation process for them"""
        # disable every body but the target and robot
        bodies = [b for b in self.env.GetBodies() if b.GetNetworkId() != self.robot.GetNetworkId() and b.GetNetworkId() != self.target.GetNetworkId()]
        for b in bodies:
            b.Enable(False)
        try:
            if self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531' and self.manip.GetName() == 'arm':
                self.generate(maxradius=1.0)
            else:
                raise ValueError('could not auto-generate grasp set for %s:%s'%(self.robot.GetName(),self.manip.GetName()))
            self.save()
        finally:
            for b in bodies:
                b.Enable(True)

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

    @staticmethod
    def GetGeodesicSphere(spherelevel=2):
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

def CreateOptionParser():
    parser = OptionParser(description='Computes the reachability region of a robot and python pickles it into a file.')
    parser.add_option('--robot',action='store',type='string',dest='robot',default='robots/barrettsegway.robot.xml',
                      help='OpenRAVE robot to load')
    parser.add_option('--manipname',action='store',type='string',dest='manipname',default=None,
                      help='The name of the manipulator to use')
    parser.add_option('--maxradius',action='store',type='float',dest='maxradius',default=None,
                      help='The max radius of the arm to perform the computation')
    parser.add_option('--show',action='store_true',dest='show',default=False,
                      help='If set, uses mayavi (v3+) to display the reachability')
    return parser

def run(Model=ReachabilityModel,parser=CreateOptionParser()):
    (options, args) = parser.parse_args()

    env = Environment()
    try:
        with env:
            robot = env.ReadRobotXMLFile(options.robot)
            env.AddRobot(robot)
            robot.SetTransform(eye(4))
            if options.manipname is None:
                robot.SetActiveManipulator([i for i,m in enumerate(robot.GetManipulators()) if m.HasIKSolver()][0])
            else:
                robot.SetActiveManipulator([i for i,m in self.robot.GetManipulators() if m.GetName()==options.manipname][0])

        model = Model(env=env,robot=robot)
        if options.show:
            from enthought.mayavi import mlab
            if not model.load():
                print 'failed to find cached grasp set %s'%self.getfilename()
                sys.exit(1)
            while True:
                model.show()
        
        try:
            model.autogenerate()
        except ValueError, e:
            print e
            print 'attempting preset values'
            model.generate(maxradius=options.maxradius)
            model.save()
    finally:
        env.Destroy()

if __name__=='__main__':
    run(ReachabilityModel)
