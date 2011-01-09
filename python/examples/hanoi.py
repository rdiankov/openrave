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
"""Solves the hanoi problem using simple arm planning.

.. image:: ../../images/examples_hanoi.jpg
  :height: 200

**Running the Example**::

  openrave.py --example hanoi

Description
-----------

This example solves the Hanoi Puzzle using the Puma arm. You can easily change the locations of the
pegs, disks, or add obstacles in the environment files **data/hanoi_complex.env.xml** and
**data/hanoi.env.xml** to make the problem harder. The default planner used is the rBiRRT, you can
easily change it to a different planner by changing the arguments to the BaseManipulation problem.

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import time
from openravepy import Environment, IkParameterization, planning_error, raveLogInfo, raveLogWarn, OpenRAVEGlobalArguments, RaveDestroy
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.databases import inversekinematics
from numpy import array, arange, linalg, pi, dot, vstack, cos, sin, cross, r_, c_
from optparse import OptionParser

class HanoiPuzzle:
    def __init__(self,env,robot):
        self.env = env
        self.robot = robot
        # load the IK solver
        self.ikmodel = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate() # autogenerate if one doesn't exist
        with self.env: # lock the environment
            self.basemanip = BaseManipulation(self.robot)
            self.taskmanip = TaskManipulation(self.robot)
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

    def waitrobot(self):
        """busy wait for robot completion"""
        while not self.robot.GetController().IsDone():
            time.sleep(0.01)
    def MoveToPosition(self, values,indices):
        """uses a planner to safely move the hand to the preshape and returns the trajectory"""
        # move the robot out of the way so it can complete a preshape
        with self.robot:
            self.robot.SetActiveDOFs(indices)
            self.basemanip.MoveUnsyncJoints(jointvalues=values,jointinds=indices)
        self.waitrobot()
        # move the hand to the preshape
        with self.robot:
            self.robot.SetActiveDOFs(indices)
            self.basemanip.MoveActiveJoints(goal=values)
        self.waitrobot()

    def putblock(self, disk, srcpeg, destpeg, height):
        with self.env:
            srcpegbox = srcpeg.ComputeAABB()
            destpegbox = destpeg.ComputeAABB()
            # get all the transformations
            Thand = self.robot.GetActiveManipulator().GetEndEffectorTransform()
            Tdisk = disk.GetTransform()
            Tsrcpeg = srcpeg.GetTransform()
            Tpeg = destpeg.GetTransform()
        src_upvec = Tsrcpeg[0:3,2:3]
        dest_upvec = Tpeg[0:3,2:3]
        Tdiff = dot(linalg.inv(Tdisk), Thand)
        
        # iterate across all possible orientations the destination peg can be in
        for ang in arange(-pi,pi,0.3):
            # find the dest position
            p = Tpeg[0:3,3:4] + height * dest_upvec
            R = dot(Tpeg[0:3,0:3], array(((cos(ang),-sin(ang),0),(sin(ang),cos(ang),0),(0,0,1))))
            T = dot(r_[c_[R,p], [[0,0,0,1]]], Tdiff)
            with self.env:
                # check the IK of the destination
                if self.robot.GetActiveManipulator().FindIKSolution(T,True) is None:
                    continue
                # add two intermediate positions, one right above the source peg
                # and one right above the destination peg
                Tnewhand = array(Thand)
                Tnewhand[0:3,3:4] += src_upvec*(max(srcpegbox.extents())*2.5-0.02)
                # check the IK of the destination
                if self.robot.GetActiveManipulator().FindIKSolution(Tnewhand,True) is None:
                    print('Tnewhand invalid')
                    continue
                Tnewhand2 = array(T)
                Tnewhand2[0:3,3:4] += dest_upvec*(max(destpegbox.extents())*2.5-height)
                # check the IK of the destination
                if self.robot.GetActiveManipulator().FindIKSolution(Tnewhand2,True) is None:
                    print('Tnewhand2 invalid')
                    continue
            try:
                self.basemanip.MoveToHandPosition(matrices=[Tnewhand])
                raveLogInfo('move to position above source peg')
                self.waitrobot() # wait for robot to complete all trajectories

                self.basemanip.MoveToHandPosition(matrices=[Tnewhand2])
                raveLogInfo('move to position above dest peg')
                self.waitrobot() # wait for robot to complete all trajectories

                self.basemanip.MoveToHandPosition(matrices=[T])
                raveLogInfo('move to dest peg')
                self.waitrobot() # wait for robot to complete all trajectories
                return True
            except planning_error, e:
                raveLogWarn(str(e))
        raise planning_error('failed to put block')

    def GetGrasp(self, Tdisk, radius, angles):
        """ returns the transform of the grasp given its orientation and the location/size of the disk"""
        zdir = -dot(Tdisk[0:3,0:3],vstack([cos(angles[0])*cos(angles[1]),-cos(angles[0])*sin(angles[1]),-sin(angles[0])]))
        pos = Tdisk[0:3,3:4] + radius*dot(Tdisk[0:3,0:3],vstack([cos(angles[1]),-sin(angles[1]),0]))
        xdir = cross(Tdisk[0:3,1:2],zdir,axis=0)
        xdir = xdir / linalg.norm(xdir)
        ydir = cross(zdir,xdir,axis=0)
        Tgrasp = r_[c_[xdir,ydir,zdir,pos],[[0,0,0,1]]]
        return [Tgrasp,dot(Tgrasp, array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]]))]

    def hanoimove(self, disk, srcpeg, destpeg, height):
        """Moves the arm and manipulator to grasp a peg and place it on a different peg"""
        openhandfn = lambda: self.MoveToPosition([-0.7],self.robot.GetActiveManipulator().GetGripperIndices())
        openhandfn()
        Tdisk = disk.GetTransform()
        for ang2 in arange(-pi/2,1.5*pi,0.4):
            for ang1 in arange(-0.8,0,0.2):
                Tgrasps = self.GetGrasp(Tdisk, disk.radius, [ang1,ang2]) # get the grasp transform given the two angles
                for Tgrasp in Tgrasps: # for each of the grasps
                    try:
                        self.basemanip.MoveToHandPosition(matrices=[Tgrasp])
                        raveLogInfo('moving hand to location')
                        self.waitrobot()
                        # succeeded so grab the disk
                        self.taskmanip.CloseFingers()
                        self.waitrobot()
                        with self.env:
                            self.robot.Grab(disk)

                        # try to pub the disk in the destination peg
                        self.putblock(disk, srcpeg, destpeg, height)
                        self.waitrobot() # wait for robot to complete all trajectories
                        with self.env:
                            self.robot.ReleaseAllGrabbed()
                        openhandfn()
                        return True
                    except planning_error,e:
                        raveLogWarn(str(e))
                        with self.env:
                            self.robot.ReleaseAllGrabbed()
                        openhandfn()
        return False

    def hanoisolve(self, n, pegfrom, pegto, pegby):
        if n == 1:
            # move the disk
            disk = pegfrom.disks[-1]
            print('hanoimove %s from %s to %s'%(disk.GetName(), pegfrom.GetName(), pegto.GetName()))
            if not self.hanoimove(disk, pegfrom, pegto, self.heights[len(pegto.disks)]):
                raise ValueError('failed to solve hanoi')
            # add the disk onto the correct peg list
            pegto.disks.append(disk)
            pegfrom.disks.pop()
        else:
            self.hanoisolve(n-1, pegfrom, pegby, pegto)
            self.hanoisolve(1, pegfrom, pegto, pegby)
            self.hanoisolve(n-1, pegby, pegto, pegfrom)

def run(args=None):
    """Executes the example, ``args`` specifies a list of the arguments to the script.
    
    **Help**
    
    .. shell-block:: openrave.py --example hanoi --help
    """
    parser = OptionParser(description='Manipulation planning example solving the hanoi problem.', usage='openrave.py --example hanoi [options]')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/hanoi_complex2.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    try:
        while True:
            env.Reset()
            env.Load(options.scene)
            hanoi = HanoiPuzzle(env,env.GetRobots()[0])
            hanoi.hanoisolve(3,hanoi.srcpeg,hanoi.destpeg,hanoi.peg)
    finally:
        env.Destroy() # done with the environment
        RaveDestroy()

if __name__ == "__main__":
    run()
