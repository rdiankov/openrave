#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
"""Finds the first feasible grasp for an object as fast as possible without generating a grasp database.

.. examplepre-block:: fastgraspingthreaded

Description
-----------

This type of example is suited for object geometries that are dynamically created from sensor data.

.. examplepost-block:: fastgraspingthreaded
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Atsushi Tsuda and Rosen Diankov'

from itertools import izip
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

import multiprocessing
import time

class FastGraspingThreaded:
    def __init__(self,robot,target):
        self.env = robot.GetEnv()
        self.robot = robot
        self.manip = robot.GetActiveManipulator()
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
        self.target=target
        self.prob = RaveCreateModule(self.env,'Grasper')
        self.env.AddModule(self.prob,self.robot.GetName())
        # used for maintaining compatible grasp structures
        self.gmodel = databases.grasping.GraspingModel(self.robot,self.target)

    def callGraspThreaded(self,approachrays,standoffs,preshapes,rolls,manipulatordirections=None,target=None,transformrobot=True,onlycontacttarget=True,tightgrasp=False,graspingnoise=None,ngraspingnoiseretries=None,forceclosurethreshold=None,avoidlinks=None,collisionchecker=None,translationstepmult=None,numthreads=None,startindex=None,maxgrasps=None,checkik=False,friction=None):
        """See :ref:`module-grasper-graspthreaded`
        """
        cmd = 'GraspThreaded '

        if target is not None:
            cmd += 'target %s '%target.GetName()
        cmd += 'forceclosure %d %g onlycontacttarget %d tightgrasp %d '%(forceclosurethreshold is not None,forceclosurethreshold,onlycontacttarget,tightgrasp)
        if friction is not None:
            cmd += 'friction %.15e '%friction
        if checkik is not None:
            cmd += 'checkik %d '%checkik
        if startindex is not None:
            cmd += 'startindex %d '%startindex
        if maxgrasps is not None:
            cmd += 'maxgrasps %d '%maxgrasps
        if avoidlinks is not None:
            for link in avoidlinks:
                cmd += 'avoidlink %s '%link.GetName()
        if graspingnoise is not None:
            cmd += 'graspingnoise %.15e %d '%(graspingnoise,ngraspingnoiseretries)
        if translationstepmult is not None:
            cmd += 'translationstepmult %.15e '%translationstepmult
        if numthreads is not None:
            cmd += 'numthreads %d '%numthreads
        cmd += 'approachrays %d '%len(approachrays)
        for f in approachrays.flat:
            cmd += str(f) + ' '
        cmd += 'rolls %d '%len(rolls)
        for f in rolls.flat:
            cmd += str(f) + ' '
        cmd += 'standoffs %d '%len(standoffs)
        for f in standoffs.flat:
            cmd += str(f) + ' '
        cmd += 'preshapes %d '%len(preshapes)
        for f in preshapes.flat:
            cmd += str(f) + ' '
        cmd += 'manipulatordirections %d '%len(manipulatordirections)
        for f in manipulatordirections.flat:
            cmd += str(f) + ' '
        res = self.prob.SendCommand(cmd)
        if res is None:
            raise planning_error('Grasp failed')
        resultgrasps = res.split()
        resvalues=[]
        nextid = int(resultgrasps.pop(0))
        preshapelen = len(self.robot.GetActiveManipulator().GetGripperIndices())
        for i in range(int(resultgrasps.pop(0))):
            position = array([float64(resultgrasps.pop(0)) for i in range(3)])
            direction = array([float64(resultgrasps.pop(0)) for i in range(3)])
            roll = float64(resultgrasps.pop(0))
            standoff = float64(resultgrasps.pop(0))
            manipulatordirection = array([float64(resultgrasps.pop(0)) for i in range(3)])
            mindist = float64(resultgrasps.pop(0))
            volume = float64(resultgrasps.pop(0))
            preshape = [float64(resultgrasps.pop(0)) for i in range(preshapelen)]
            Tfinal = matrixFromPose([float64(resultgrasps.pop(0)) for i in range(7)])
            finalshape = array([float64(resultgrasps.pop(0)) for i in range(self.robot.GetDOF())])
            contacts_num=int(resultgrasps.pop(0))
            contacts=[float64(resultgrasps.pop(0)) for i in range(contacts_num*6)]
            contacts = reshape(contacts,(contacts_num,6))
            resvalues.append([position, direction, roll, standoff, manipulatordirection, mindist, volume, preshape,Tfinal,finalshape,contacts])
        return nextid, resvalues

    def computeGrasp(self):
        with self.env:
            approachrays = databases.grasping.GraspingModel._computeBoxApproachRays(self.env,self.target,delta=0.02,normalanglerange=0.5) # rays to approach object
            N=approachrays.shape[0]

            Ttarget = self.target.GetTransform()
            gapproachrays = c_[dot(approachrays[:,0:3],transpose(Ttarget[0:3,0:3]))+tile(Ttarget[0:3,3],(N,1)),dot(approachrays[:,3:6],transpose(Ttarget[0:3,0:3]))]
            self.approachgraphs = [self.env.plot3(points=gapproachrays[:,0:3],pointsize=5,colors=array((1,0,0))),
                                   self.env.drawlinelist(points=reshape(c_[gapproachrays[:,0:3],gapproachrays[:,0:3]+0.005*gapproachrays[:,3:6]],(2*N,3)),linewidth=4,colors=array((1,0,0,1)))]

            standoffs = array([0])
            rolls = arange(0,2*pi,0.5*pi)
            manipulatordirections = array([self.manip.GetDirection()])
            target = self.target
            graspingnoise = 0.01
            ngraspingnoiseretries = 20
            forceclosurethreshold=1e-9
            avoidlinks = []
            friction = 0.4
            numthreads = multiprocessing.cpu_count()
            maxgrasps = 1
            checkik = True
            grasps = []
            jointvalues = []
            with self.robot:
                self.robot.SetActiveDOFs(self.manip.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z)
                approachrays[:,3:6] = -approachrays[:,3:6]

                # initial preshape for robot is the released fingers
                with self.target:
                    self.target.Enable(False)
                    taskmanip = interfaces.TaskManipulation(self.robot)
                    final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
                    preshapes = array([final])

                starttime = time.time()
                nextid, resultgrasps = self.callGraspThreaded(approachrays,standoffs,preshapes,rolls,manipulatordirections=manipulatordirections,target=target,graspingnoise=graspingnoise,ngraspingnoiseretries=ngraspingnoiseretries,forceclosurethreshold=forceclosurethreshold,avoidlinks=avoidlinks,numthreads=numthreads,maxgrasps=maxgrasps,checkik=checkik,friction=friction)
                totaltime = time.time()-starttime
                for resultgrasp in resultgrasps:
                    grasp = zeros(self.gmodel.totaldof)
                    grasp[self.gmodel.graspindices.get('igrasppos')] = resultgrasp[0]
                    grasp[self.gmodel.graspindices.get('igraspdir')] = resultgrasp[1]
                    grasp[self.gmodel.graspindices.get('igrasproll')] = resultgrasp[2]
                    grasp[self.gmodel.graspindices.get('igraspstandoff')] = resultgrasp[3]
                    grasp[self.gmodel.graspindices.get('imanipulatordirection')] = resultgrasp[4]
                    grasp[self.gmodel.graspindices.get('igrasppreshape')] = resultgrasp[7]
                    Tfinal = resultgrasp[8]
                    finaljointvalues = resultgrasp[9]
                    with self.robot:
                        Tlocalgrasp = eye(4)
                        self.robot.SetTransform(Tfinal)
                        Tgrasp = self.manip.GetEndEffectorTransform()
                        Tlocalgrasp = dot(linalg.inv(self.target.GetTransform()),Tgrasp)
                        grasp[self.gmodel.graspindices.get('igrasptrans')] = reshape(transpose(Tlocalgrasp[0:3,0:4]),12)
                    grasps.append(grasp)
                    jointvalues.append(finaljointvalues)

                grasps = array(grasps)
                jointvalues = array(jointvalues)
                print 'found %d grasps in %.3fs'%(len(grasps),totaltime)
                return grasps, jointvalues

    def showgrasps(self, grasps, jointvalues):
        for i,grasp in enumerate(grasps):
            with self.env:
                self.robot.SetDOFValues(jointvalues[i])
                self.env.UpdatePublishedBodies()
                raw_input('press any key')

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    if options.manipname is not None:
        robot.SetActiveManipulator(options.manipname)
    # find an appropriate target
    bodies = [b for b in env.GetBodies() if not b.IsRobot() and linalg.norm(b.ComputeAABB().extents()) < 0.2]
    for body in bodies:
        self = FastGraspingThreaded(robot,target=body)
        grasps, jointvalues = self.computeGrasp()
        self.showgrasps(grasps, jointvalues)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Example showing how to compute a valid grasp as fast as possible without computing a grasp set, this is used when the target objects change frequently.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene', action="store",type='string',dest='scene',default='data/wamtest1.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname', action="store",type='string',dest='manipname',default=None,
                      help='Choose the manipulator to perform the grasping for')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__ == "__main__":
    run()
