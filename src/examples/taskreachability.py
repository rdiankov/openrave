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

    def save(self,filename='taskreachability.pp'):
        f = open(filename,'w')
        pickle.dump((self.robotfile,self.manipname,self.reachabilitystats,self.reachabilitydensity3d,self.pointscale), f)
        f.close()
    def load(self,filename='taskreachability.pp'):
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

    def ComputeReachability(self,maxradius,grasptablefilename):
        starttime = time.time()
        grasptable = [float(token) for token in open(grasptablefilename, "r").read().split()]
        if len(grasptable) == 0:
            raise ValueError('grasp table is empty')

        gripperindices = manip.GetChildDOFIndices()
        print 'gripper indices: ',gripperindices
        dim = 12 + len(self.gripperindices)
        grasptable = reshape(grasptable,(len(grasptable)/dim,dim))

        txrange = arange(-self.workextents[0],self.workextents[0],0.01)
        tyrange = arange(-self.workextents[1],self.workextents[1],0.01)
        trrange = arange(-self.rotrange,self.rotrange,0.1)
        shape = (len(txrange),len(tyrange),len(trrange))
        reachabilitydensity3d = zeros(prod(shape))
        self.reachabilitystats = []
        self.orenv.LockPhysics(True)

        try:
            Tobj = self.target.GetTransform()
            Tlocal = eye(4)
            Tgrasp = eye(4)
            index = 0
            for tx in txrange:
                print index,'/',len(reachabilitydensity3d)
                for ty in tyrange:
                    for tr in trrange:
                        Tlocal[0:3,0:3] = array(((cos(tr),-sin(tr),0),(sin(tr),cos(tr),0),(0,0,1)))
                        Tlocal[0,3] = tx
                        Tlocal[1,4] = ty
                        Tobject = dot(self.Twork,Tlocal)
                        self.target.SetTransform(Tobject)
                        if not self.orenv.CheckCollision(self.target):
                            # check if at least one grasp is satisfied
                            for grasp in grasptable:
                                self.orrobot.SetJointValues(grasptable[12:],gripperindices)
                                Tgrasp[0:3,0:4] = transpose(reshape(grasptable[0:12],(4,3)))
                                if self.manip.FindIKSolution(dot(Tobject,Tgrasp),True) is not None:
                                    self.reachabilitystats.append(array((tx,ty,tr)))
                                    reachabilitydensity3d[index] += 1
                                    index += 1
                                    break
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

if __name__=='__main__':
    parser = OptionParser(description='Given a robot, a sampling function for obstacles, a workspace for possible target object locations, computes the feasibilyt of the robot reaching those locations given its current position.')
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
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
    parser.add_option('--grasptables',
                      action="store",type='string',dest='grasptables',default='barrettgrasptables.txt',
                      help='Simple file specifying all possible ways to grasp the target (every line is 3x4 matrix and preshape)')
    parser.add_option('--grasptablefilename',
                      action="store",type='string',dest='grasptablefilename',default='barrettgrasptables.txt',
                      help='Simple file specifying all possible ways to grasp the target (every line is 3x4 matrix and preshape)')
#     parser.add_option('--obstacles', action='append', type='string', dest='obstacles',default=[],
#                       help='The names of the obstacles to create and move around the environment')
    parser.add_option('--workobjectspace', action='store', type='string', dest='workobjectspace',default=None,
                      help='Object in which the workspace is centered in')
    parser.add_option('--workregion', action='store', type='string', dest='workregion',default=None,
                      help='[rightdir] [updir] [xyz position] [xy extents]')
    parser.add_option('--rotrange',
                      action="store",type='float',dest='rotrange',default=pi,
                      help='Defines min and max angle to rotate part by')
    parser.add_option('--savereachability', action='store',type='string',dest='savereachability',default='reachability.pp',
                      help='Compute the statistics and pickle the data to this file')
    (options, args) = parser.parse_args()
    
    model = ReachabilityModel(options.robot,manipname=options.manipname)
    if options.savereachability:
        model.ComputeReachability(maxradius=options.maxradius,grasptablefilename)
        model.save(options.savereachability)
