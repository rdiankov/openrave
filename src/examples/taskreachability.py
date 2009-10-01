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

import metaclass

class TaskReachability(metaclass.AutoReloader):
    def __init__(self,scenefile,targetname=None,robotname=None,manipname=None,showviewer=False):
        self.orenv = Environment()
        if showviewer:
            self.orenv.SetViewer('qtcoin')
        self.loadscene(scenefile,targetname,robotname,manipname)
                
        self.reachabilitystats = None
        self.reachabilitydensity3d = None

    def save(self,filename='taskreachability.pp'):
        f = open(filename,'w')
        pickle.dump((self.reachabilitystats,self.reachabilitydensity3d,self.workscale,self.Twork), f)
        f.close()

    def load(self,filename='taskreachability.pp'):
        f = open(filename, 'r')
        self.reachabilitystats,self.reachabilitydensity3d,self.workscale,self.Twork = pickle.load(f)

    def loadscene(self,scenefile,targetname,robotname,manipname):
        self.ortarget = None
        self.orrobot = None
        self.manip = None
        self.orenv.Reset()

        self.manipname = manipname
        self.scenefile = scenefile
        self.orenv.Load(scenefile)
        if robotname is not None:
            self.orrobot = [robot for robot in self.orenv.GetRobots() if robot.GetName() == robotname][0]
        else:
            self.orrobot = self.orenv.GetRobots()[0]
        self.robotname = self.orrobot.GetName()

        if manipname is None:
            self.manip = [m for m in self.orrobot.GetManipulators() if m.HasIKSolver()][0]
        else:
            self.manip = [m for m in self.orrobot.GetManipulators() if m.GetName()==manipname and m.HasIKSolver()][0]
        self.manipname = self.manip.GetName()

        # get the manipulator gripper links
        eeinverse = linalg.inv(self.manip.GetEndEffectorTransform())
        self.gripperlinks = [(link,dot(eeinverse,link.GetTransform())) for link in self.manip.GetChildLinks()]
        
        if targetname is None:
            targetname = 'target'
        self.ortarget = [body for body in self.orenv.GetBodies() if body.GetName() == targetname][0]
        self.targetname = self.ortarget.GetName()

        self.trimesh = self.orenv.TriangulateScene(Environment.TriangulateOptions.AllExceptBody,self.targetname)

    def CheckGripperCollision(self,Tgripper):
        for link,trelative in self.gripperlinks:
            link.SetTransform(dot(Tgripper,trelative))
            if self.orenv.CheckCollision(link):
                return True
        return False#self.orenv.CheckCollision(self.orrobot)

    def ComputeReachability(self,workobjectspace,workspace,workextents,grasptablefile):
        starttime = time.time()
        grasptable = [float(token) for token in open(grasptablefile, "r").read().split()]
        if len(grasptable) == 0:
            raise ValueError('grasp table is empty')

        self.orenv.LockPhysics(True)

        try:
            gripperindices = self.manip.GetChildDOFIndices()
            print 'gripper indices: ',gripperindices
            dim = 12 + len(gripperindices)
            grasptable = reshape(grasptable,(len(grasptable)/dim,dim))
            
            steps = array([0.02,0.02,0.2])
            self.workscale = 1.0/steps
            txrange = arange(-workextents[0],workextents[0],steps[0])
            tyrange = arange(-workextents[1],workextents[1],steps[1])
            trrange = arange(-workextents[2],workextents[2],steps[2])
            shape = (len(txrange),len(tyrange),len(trrange))
            reachabilitydensity3d = zeros(prod(shape))
            self.reachabilitystats = []

            Twork = eye(4)
            Twork[0,0] = workspace[0]; Twork[1,0] = workspace[1]; Twork[2,0] = workspace[2]
            Twork[0,1] = workspace[3]; Twork[1,1] = workspace[4]; Twork[2,1] = workspace[5]
            Twork[0,3] = workspace[6]; Twork[1,3] = workspace[7]; Twork[2,3] = workspace[8]
            Twork[0:3,2] = cross(Twork[0:3,0],Twork[0:3,1])
            orwork = self.orenv.GetKinBody(workobjectspace)
            if orwork is not None:
                Twork = dot(orwork.GetTransform(),Twork)
            self.Twork = Twork

            Tobj = self.ortarget.GetTransform()
            Tlocal = eye(4)
            Tgrasp = eye(4)
            index = 0
            for tx in txrange:
                print index,'/',len(reachabilitydensity3d),(time.time()-starttime)
                for ty in tyrange:
                    print index,'*',len(reachabilitydensity3d),(time.time()-starttime)
                    for tr in trrange:
                        Tlocal[0:3,0:3] = array(((cos(tr),-sin(tr),0),(sin(tr),cos(tr),0),(0,0,1)))
                        Tlocal[0,3] = tx
                        Tlocal[1,3] = ty
                        Tobject = dot(self.Twork,Tlocal)
                        self.ortarget.SetTransform(Tobject)
                        if not self.orenv.CheckCollision(self.ortarget):
                            # check if at least one grasp is satisfied
                            for grasp in grasptable:
                                self.orrobot.SetJointValues(grasp[12:],gripperindices)
                                Tgrasp[0:3,0:4] = transpose(reshape(grasp[0:12],(4,3)))
                                Tgrasp = dot(Tobject,Tgrasp)
                                if not self.CheckGripperCollision(Tgrasp) and self.manip.FindIKSolution(Tgrasp,True) is not None:
                                    self.reachabilitystats.append(array((tx,ty,tr)))
                                    reachabilitydensity3d[index] += 1.0
                                    break
                        index += 1
        finally:
            self.orenv.LockPhysics(False)
        self.reachabilitydensity3d = reshape(reachabilitydensity3d,shape)/len(trrange)
        print 'reachability finished in %fs'%(time.time()-starttime)

    def showdensity(self,showrobot=True,contours=[0.1,0.5,0.9,0.99],figureid=1):
        from enthought.mayavi import mlab
        mlab.figure(figureid,fgcolor=(0,0,0), bgcolor=(1,1,1),size=(1024,768))
        mlab.clf()
        offset = array((0,0,0))
        density2d = sum(self.reachabilitydensity3d,2)/float(self.reachabilitydensity3d.shape[2])
        print 'percent covered: ',sum(density2d)/float(density2d.size)
        density3d = zeros((self.reachabilitydensity3d.shape[0]+2,self.reachabilitydensity3d.shape[1]+2,3))
        density3d[1:-1,1:-1,1] = sign(density2d)
        src = mlab.pipeline.scalar_field(density3d)
        for c in contours:
            mlab.pipeline.iso_surface(src,contours=[c])
#         density3d = zeros((density2d.shape[0],density2d.shape[1],3))
#         density3d[:,:,1] = density2d
#         mlab.pipeline.volume(mlab.pipeline.scalar_field(density3d*200))
        if showrobot:
            Tscale = diag([self.workscale[0],self.workscale[1],0.5*(self.workscale[0]+self.workscale[1]),1])
            Tscale[0,3] += self.reachabilitydensity3d.shape[0]/2+1
            Tscale[1,3] += self.reachabilitydensity3d.shape[1]/2+1
            T = dot(Tscale,linalg.inv(self.Twork))
            v = dot(self.trimesh.vertices,transpose(T[0:-1,0:-1]))
            v += tile(transpose(T[0:-1,-1:]),(v.shape[0],1))
            mlab.triangular_mesh(v[:,0],v[:,1],v[:,2],self.trimesh.indices,color=(0.5,0.5,0.5))

if __name__=='__main__':
    parser = OptionParser(description='Given a robot, a sampling function for obstacles, a workspace for possible target object locations, computes the feasibilyt of the robot reaching those locations given its current position.')
    parser.add_option('--scenefile',
                      action="store",type='string',dest='scenefile',default='data/lab1.env.xml',
                      help='OpenRAVE scene to load')
    parser.add_option('--robotname',
                      action="store",type='string',dest='robotname',default=None,
                      help='Specific robot to use (otherwise first robot found will be displayed)')
    parser.add_option('--manipname',
                      action="store",type='string',dest='manipname',default=None,
                      help='Specific manipulator to use on robot (otherwise first manipulator with an IK solver found will be used)')
    parser.add_option('--targetname',
                      action="store",type='string',dest='targetname',default='target',
                      help='Name of target object to attempt to grasp')
    parser.add_option('--grasptablefile',
                      action="store",type='string',dest='grasptablefile',default='barrettgrasptables.txt',
                      help='Simple file specifying all possible ways to grasp the target (every line is 3x4 matrix and preshape)')
#     parser.add_option('--obstacles', action='append', type='string', dest='obstacles',default=[],
#                       help='The names of the obstacles to create and move around the environment')
    parser.add_option('--workobjectspace', action='store', type='string', dest='workobjectspace',default=None,
                      help='Object in which the workspace is centered in')
    parser.add_option('--workspace', action='store', type='string', dest='workspace',default=None,
                      help='[rightdir] [updir] [xyz position]')
    parser.add_option('--workextents', action='store', type='string', dest='workextents',default=None,
                      help='right/up/rotation extents')
    parser.add_option('--savereachability', action='store',type='string',dest='savereachability',default='reachability.pp',
                      help='Compute the statistics and pickle the data to this file')
    (options, args) = parser.parse_args()
    
    model = TaskReachability(options.scenefile,options.targetname,options.robotname,options.manipname)
    if options.savereachability:
        model.ComputeReachability(options.workobjectspace,[float(token) for token in options.workspace.split()],[float(token) for token in options.workextents.split()],options.grasptablefile)
        model.save(options.savereachability)

def test1():
    """
    WAM
    ./taskreachability.py --scenefile=data/tablewam.env.xml --robotname=BarrettWAM --targetname=frootloops --grasptablefile=cereal_grasps_barrett.mat --workobjectspace=table --workspace='1 0 0 0 1 0 0 0 0.015' --workextents='0.3 0.25 1.57' --savereachability=reach_tablewam.pp

    ./taskreachability.py --scenefile=data/reach_wam1.env.xml --robotname=BarrettWAM --targetname=target_cereal --grasptablefile=cereal_grasps_barrett.mat --workobjectspace=table --workspace='1 0 0 0 1 0 0 0 0.015' --workextents='0.3 0.25 1.57' --savereachability=reach_wam1_tablecereal.pp

    ./taskreachability.py --scenefile=data/reach_wam1.env.xml --robotname=BarrettWAM --targetname=target_cereal --grasptablefile=cereal_grasps_barrett.mat --workobjectspace=cabinetshelf --workspace='1 0 0 0 1 0 0 -0.1 0.005' --workextents='0.2 0.2 1.57' --savereachability=reach_wam1_shelfcereal.pp

    ./taskreachability.py --scenefile=data/reach_wam1.env.xml --robotname=BarrettWAM --targetname=target_cereal --grasptablefile=cereal_grasps_barrett.mat --workobjectspace=cabinetshelf --workspace='1 0 0 0 1 0 0 -0.1 0.165' --workextents='0.2 0.2 1.57' --savereachability=reach_wam1_uppercereal.pp

    PRISM
    ./taskreachability.py --scenefile=scenes/reach_prism1.env.xml --robotname=PRISM --targetname=target_cereal --grasptablefile=cereal_grasps_prism1.mat --workobjectspace=table --workspace='1 0 0 0 1 0 0 0 0.015' --workextents='0.3 0.25 1.57' --savereachability=reach_prism1_tablecereal.pp
    
    ./taskreachability.py --scenefile=scenes/reach_prism2.env.xml --robotname=PRISM --targetname=target_cereal --grasptablefile=cereal_grasps_prism1.mat --workobjectspace=cabinetshelf --workspace='1 0 0 0 1 0 0 0 0.015' --workextents='0.2 0.2 1.57' --savereachability=reach_prism1_shelfcereal.pp

    ./taskreachability.py --scenefile=scenes/reach_prism2.env.xml --robotname=PRISM --targetname=target_cereal --grasptablefile=cereal_grasps_prism1.mat --workobjectspace=cabinetshelf --workspace='1 0 0 0 1 0 0 0 0.165' --workextents='0.2 0.2 1.57' --savereachability=reach_prism1_uppercereal.pp
    """
    self = TaskReachability('data/tablewam.env.xml','frootloops','BarrettWAM',None)
    self.ComputeReachability('table',[1,0,0,0,1,0,0,0,0.015],[0.3,0.25,pi/2],'cereal_grasps_barrett.mat')
    
    self = TaskReachability('data/reach_wam1.env.xml','target_cereal','BarrettWAM',None)
    self = TaskReachability('scenes/reach_prism1.env.xml','target_cereal','PRISM',None)
