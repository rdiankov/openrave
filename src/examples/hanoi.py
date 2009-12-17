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
from optparse import OptionParser

g_probsmanip = None

def MoveToHandPosition(T):
    global g_probsmanip
    print 'movetohandposition matrix '+ matrixSerialization(T)
    success = g_probsmanip.SendCommand('movetohandposition matrix '+matrixSerialization(T))
    return False if success is None or len(success) == 0 else True

def WaitForController(robot):
    while not robot.GetController().IsDone():
        robot.WaitForController(1)

def MoveToPosition(robot,values,indices):
    WaitForController(robot)
    allvalues = robot.GetJointValues()
    allvalues[indices] = values
    robot.GetController().SetDesired(allvalues)
    WaitForController(robot)

def putblock(robot, disk, srcpeg, destpeg, height):
    global g_probsmanip
    srcpegbox = srcpeg.ComputeAABB()
    destpegbox = destpeg.ComputeAABB()
    
    # get all the transformations
    Thand = robot.GetActiveManipulator().GetEndEffector().GetTransform()
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
        if robot.GetActiveManipulator().FindIKSolution(T,True) is None:
            continue
        
        # add two intermediate positions, one right above the source peg
        # and one right above the destination peg
        Tnewhand = dot(Thand, Tgrasp)
        Tnewhand[0:3,3:4] = Tnewhand[0:3,3:4] + src_upvec*(max(srcpegbox.extents())*2.5-0.02)
        
        # check the IK of the destination
        if robot.GetActiveManipulator().FindIKSolution(Tnewhand,True) is None:
            print('Tnewhand invalid')
            continue

        Tnewhand2 = array(T)
        Tnewhand2[0:3,3:4] = Tnewhand2[0:3,3:4] + dest_upvec*(max(destpegbox.extents())*2.5-height)
        # check the IK of the destination
        if robot.GetActiveManipulator().FindIKSolution(Tnewhand2,True) is None:
            print('Tnewhand2 invalid')
            continue
        
        if not MoveToHandPosition(Tnewhand):
            print('failed to move to position above source peg')
            continue
        WaitForController(robot) # wait for robot to complete all trajectories
        
        if not MoveToHandPosition(Tnewhand2):
            print('failed to move to position above dest peg')
            continue
        WaitForController(robot) # wait for robot to complete all trajectories
        
        if not MoveToHandPosition(T):
            print('failed to move to dest peg')
            continue
        WaitForController(robot) # wait for robot to complete all trajectories
        
        return True

    print('failed to put block')
    return False

def GetGrasp(Tdisk, radius, angles):
    """ returns the transform of the grasp given its orientation and the location/size of the disk"""
    ydir = -dot(Tdisk[0:3,0:3],vstack([cos(angles[0])*cos(angles[1]),-sin(angles[0]),cos(angles[0])*sin(angles[1])]))
    pos = Tdisk[0:3,3:4] + radius*dot(Tdisk[0:3,0:3],vstack([cos(angles[1]),0,sin(angles[1])]))
    xdir = cross(Tdisk[0:3,1:2],ydir,axis=0)
    xdir = xdir / linalg.norm(xdir)
    zdir = cross(xdir,ydir,axis=0)
    Tgrasp = r_[c_[xdir,ydir,zdir,pos],[[0,0,0,1]]]
    return [Tgrasp,dot(Tgrasp, array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]))]

def hanoimove(robot, disk, srcpeg, destpeg, height):
    """Moves the arm and manipulator to grasp a peg and place it on a different peg"""
    global g_probsmanip
    openhandfn = lambda: MoveToPosition(robot,[-0.7],robot.GetActiveManipulator().GetGripperJoints())
    openhandfn()
    Tdisk = disk.GetTransform()

    for ang2 in arange(-pi/2,1.5*pi,0.4):
        for ang1 in arange(-0.8,0,0.2):
            Tgrasps = GetGrasp(Tdisk, disk.radius, [ang1,ang2]) # get the grasp transform given the two angles
            for Tgrasp in Tgrasps: # for each of the grasps
                if MoveToHandPosition(Tgrasp): # move the hand to that location
                    # succeeded so grab the disk
                    WaitForController(robot)
                    g_probsmanip.SendCommand('closefingers')
                    WaitForController(robot)
                    robot.Grab(disk)
                    
                    # try to pub the disk in the destination peg
                    if putblock(robot, disk, srcpeg, destpeg, height):
                        # succeeded so release the disk
                        WaitForController(robot) # wait for robot to complete all trajectories
                        robot.ReleaseAllGrabbed()
                        openhandfn()
                        return True
                    
                    # open hand and try a different grasp
                    WaitForController(robot) # wait for robot to complete all trajectories
                    robot.ReleaseAllGrabbed()
                    openhandfn()

    return True

def hanoisolve(n, pegfrom, pegto, pegby, robot, heights):
    if n == 1:
        # move the disk
        disk = pegfrom.disks[-1]
        print('hanoimove %s from %s to %s'%(disk.GetName(), pegfrom.GetName(), pegto.GetName()))
        if not hanoimove(robot, disk, pegfrom, pegto, heights[len(pegto.disks)]):
            print('failed to solve hanoi')
            raise
        
        # add the disk onto the correct peg list
        pegto.disks.append(disk)
        pegfrom.disks.pop()
    else:
        hanoisolve(n-1, pegfrom, pegby, pegto, robot, heights)
        hanoisolve(1, pegfrom, pegto, pegby, robot, heights)
        hanoisolve(n-1, pegby, pegto, pegfrom, robot, heights)

if __name__ == "__main__":
    parser = OptionParser(description='Manipulation planning example solving the hanoi problem.')
    parser.add_option('--collision',
                      action="store",type='string',dest='collision',default=None,
                      help='Name of collision checker to use when solving')
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/hanoi_complex.env.xml',
                      help='Scene file to load')
    (options, args) = parser.parse_args()

    oldenv = Environment()

    if options.collision is not None:
        c = oldenv.CreateCollisionChecker(options.collision)
        if c is not None:
            print('setting %s collision checker'%options.collision)
            oldenv.SetCollisionChecker(c)

    # test cloning
    oldenv.Reset()
    oldenv.Load(options.scene)
    env = oldenv.CloneSelf(CloningOptions.Bodies+CloningOptions.RealControllers)
    env.SetViewer('qtcoin')
    oldenv.Destroy() # destroy the old environment

    robot = env.GetRobots()[0]

    g_probsmanip = env.CreateProblem('basemanipulation')
    env.LoadProblem(g_probsmanip,robot.GetName())

    disknames = ['disk0','disk1','disk2']
    heights = array([0.021,0.062,0.103])+0.01
    
    disks = []
    diskradius = []
    for name in disknames:
        disk = env.GetKinBody(name)
        ab = disk.ComputeAABB()
        disk.radius = ab.extents()[1]-0.02
        disks.append(disk)

    srcpeg = env.GetKinBody('srcpeg')
    destpeg = env.GetKinBody('destpeg')
    peg = env.GetKinBody('peg')

    srcpeg.disks = disks
    destpeg.disks = []
    peg.disks = []

    hanoisolve(3,srcpeg,destpeg,peg,robot,heights)
    env.Destroy() # done with the environment
