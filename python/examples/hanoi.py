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
from openravepy import *
from openravepy.interfaces import BaseManipulation
from numpy import *
from optparse import OptionParser

class HanoiPuzzle:
    basemanip = None
    def __init__(self,env):
        self.env = env
        self.robot = self.env.GetRobots()[0]
        self.basemanip = BaseManipulation(env,self.robot)
        
        disknames = ['disk0','disk1','disk2']
        self.heights = array([0.021,0.062,0.103])+0.01
        
        disks = []
        diskradius = []
        for name in disknames:
            disk = env.GetKinBody(name)
            ab = disk.ComputeAABB()
            disk.radius = ab.extents()[1]-0.02
            disks.append(disk)
        
        self.srcpeg = env.GetKinBody('srcpeg')
        self.destpeg = env.GetKinBody('destpeg')
        self.peg = env.GetKinBody('peg')
        self.srcpeg.disks = disks
        self.destpeg.disks = []
        self.peg.disks = []

    def WaitForController(self):
        self.robot.GetEnv().LockPhysics(False)
        try:
            while not self.robot.GetController().IsDone():
                self.robot.WaitForController(1)
        finally:
            self.robot.GetEnv().LockPhysics(True)

    def MoveToPosition(self, values,indices):
        self.WaitForController()
        allvalues = self.robot.GetJointValues()
        allvalues[indices] = values
        self.robot.GetController().SetDesired(allvalues)
        self.WaitForController()

    def putblock(self, disk, srcpeg, destpeg, height):
        srcpegbox = srcpeg.ComputeAABB()
        destpegbox = destpeg.ComputeAABB()
        
        # get all the transformations
        Thand = self.robot.GetActiveManipulator().GetEndEffector().GetTransform()
        Tdisk = disk.GetTransform()
        Tsrcpeg = srcpeg.GetTransform()
        Tpeg = destpeg.GetTransform()

        src_upvec = Tsrcpeg[0:3,1:2]
        dest_upvec = Tpeg[0:3,1:2]
        
        # the IK grasp system is [0 0.175 0] from the center of rotation
        Tgrasp = eye(4)
        Tgrasp[1,3] = 0.175
        Tdiff = dot(dot(linalg.inv(Tdisk), Thand), Tgrasp)
        
        # iterate across all possible orientations the destination peg can be in
        for ang in arange(-pi,pi,0.3):
            # find the dest position
            p = Tpeg[0:3,3:4] + height * Tpeg[0:3,1:2]
            R = dot(Tpeg[0:3,0:3], array(((cos(ang),0,sin(ang)),(0,1,0),(-sin(ang),0,cos(ang)))))
            T = dot(r_[c_[R,p], [[0,0,0,1]]], Tdiff)

            # check the IK of the destination
            if self.robot.GetActiveManipulator().FindIKSolution(T,True) is None:
                continue

            # add two intermediate positions, one right above the source peg
            # and one right above the destination peg
            Tnewhand = dot(Thand, Tgrasp)
            Tnewhand[0:3,3:4] = Tnewhand[0:3,3:4] + src_upvec*(max(srcpegbox.extents())*2.5-0.02)

            # check the IK of the destination
            if self.robot.GetActiveManipulator().FindIKSolution(Tnewhand,True) is None:
                print('Tnewhand invalid')
                continue

            Tnewhand2 = array(T)
            Tnewhand2[0:3,3:4] = Tnewhand2[0:3,3:4] + dest_upvec*(max(destpegbox.extents())*2.5-height)
            # check the IK of the destination
            if self.robot.GetActiveManipulator().FindIKSolution(Tnewhand2,True) is None:
                print('Tnewhand2 invalid')
                continue

            if self.basemanip.MoveToHandPosition(matrices=[Tnewhand]) is None:
                print('failed to move to position above source peg')
                continue
            self.WaitForController() # wait for robot to complete all trajectories

            if self.basemanip.MoveToHandPosition(matrices=[Tnewhand2]) is None:
                print('failed to move to position above dest peg')
                continue
            self.WaitForController() # wait for robot to complete all trajectories

            if self.basemanip.MoveToHandPosition(matrices=[T]) is None:
                print('failed to move to dest peg')
                continue
            self.WaitForController() # wait for robot to complete all trajectories

            return True

        print('failed to put block')
        return False

    def GetGrasp(self, Tdisk, radius, angles):
        """ returns the transform of the grasp given its orientation and the location/size of the disk"""
        ydir = -dot(Tdisk[0:3,0:3],vstack([cos(angles[0])*cos(angles[1]),-sin(angles[0]),cos(angles[0])*sin(angles[1])]))
        pos = Tdisk[0:3,3:4] + radius*dot(Tdisk[0:3,0:3],vstack([cos(angles[1]),0,sin(angles[1])]))
        xdir = cross(Tdisk[0:3,1:2],ydir,axis=0)
        xdir = xdir / linalg.norm(xdir)
        zdir = cross(xdir,ydir,axis=0)
        Tgrasp = r_[c_[xdir,ydir,zdir,pos],[[0,0,0,1]]]
        return [Tgrasp,dot(Tgrasp, array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]))]

    def hanoimove(self, disk, srcpeg, destpeg, height):
        """Moves the arm and manipulator to grasp a peg and place it on a different peg"""
        openhandfn = lambda: self.MoveToPosition([-0.7],self.robot.GetActiveManipulator().GetGripperJoints())
        openhandfn()
        Tdisk = disk.GetTransform()
        for ang2 in arange(-pi/2,1.5*pi,0.4):
            for ang1 in arange(-0.8,0,0.2):
                Tgrasps = self.GetGrasp(Tdisk, disk.radius, [ang1,ang2]) # get the grasp transform given the two angles
                for Tgrasp in Tgrasps: # for each of the grasps
                    if self.basemanip.MoveToHandPosition(matrices=[Tgrasp]) is not None: # move the hand to that location
                        # succeeded so grab the disk
                        self.WaitForController()
                        self.basemanip.CloseFingers()
                        self.WaitForController()
                        self.robot.Grab(disk)

                        # try to pub the disk in the destination peg
                        if self.putblock(disk, srcpeg, destpeg, height):
                            # succeeded so release the disk
                            self.WaitForController() # wait for robot to complete all trajectories
                            self.robot.ReleaseAllGrabbed()
                            openhandfn()
                            return True

                        # open hand and try a different grasp
                        self.WaitForController() # wait for robot to complete all trajectories
                        self.robot.ReleaseAllGrabbed()
                        openhandfn()
        return True

    def hanoisolve(self, n, pegfrom, pegto, pegby):
        if n == 1:
            # move the disk
            disk = pegfrom.disks[-1]
            print('hanoimove %s from %s to %s'%(disk.GetName(), pegfrom.GetName(), pegto.GetName()))
            if not self.hanoimove(disk, pegfrom, pegto, self.heights[len(pegto.disks)]):
                print('failed to solve hanoi')
                raise
            # add the disk onto the correct peg list
            pegto.disks.append(disk)
            pegfrom.disks.pop()
        else:
            self.hanoisolve(n-1, pegfrom, pegby, pegto)
            self.hanoisolve(1, pegfrom, pegto, pegby)
            self.hanoisolve(n-1, pegby, pegto, pegfrom)

def run():
    parser = OptionParser(description='Manipulation planning example solving the hanoi problem.')
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/hanoi_complex.env.xml',
                      help='Scene file to load')
    (options, args) = parser.parse_args()
    
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load(options.scene)
    hanoi = HanoiPuzzle(env)
    with env: # lock the environment
        hanoi.hanoisolve(3,hanoi.srcpeg,hanoi.destpeg,hanoi.peg)
    env.Destroy() # done with the environment

if __name__ == "__main__":
    run()
