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
"""Tests moving the hand straight with several robots.

.. image:: ../../images/examples_movehandstraight.jpg
  :height: 200

**Running the Example**::

  openrave.py --example movehandstraight

Description
-----------

Shows how to use the MoveHandStraight basemanipulation command.

"""

from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from openravepy.interfaces import BaseManipulation, TaskManipulation
from openravepy.databases import convexdecomposition,grasping,inversekinematics
from numpy import *
import numpy,time,traceback
from optparse import OptionParser
from itertools import izip

        
def run(args=None):
    """Executes the movehandstraight example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how choose IK solutions so that move hand straight can move without discontinuities.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/puma_tabletop.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',
                      action="store",type='string',dest='manipname',default=None,
                      help='Choose the manipulator to perform movement for')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    try:
        env.Load(options.scene)
        robot = env.GetRobots()[0]
        if options.manipname is not None:
            robot.SetActiveManipulator(options.manipname)
        with env:
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                ikmodel.autogenerate()
            basemanip = interfaces.BaseManipulation(robot)
            taskmanip = interfaces.TaskManipulation(robot)
            robot.SetJointValues([-0.97],ikmodel.manip.GetGripperIndices())
            Tstart = array([[ -1,  0,  0,   2.00000000e-01], [  0,0,   1, 6.30000000e-01], [  0,   1  , 0,   5.50000000e-02], [  0,0,0,1]])
            sol = ikmodel.manip.FindIKSolution(Tstart,IkFilterOptions.CheckEnvCollisions)
            robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
        #basemanip.MoveToHandPosition([Tstart],maxiter=1000,maxtries=1,seedik=4)
        #robot.WaitForController(0)
        taskmanip.CloseFingers()
        robot.WaitForController(0)
        with env:
            target = env.GetKinBody('cylinder_green_3')
            robot.Grab(target)
            updir = array((0,0,1))

#         from IPython.Shell import IPShellEmbed
#         ipshell = IPShellEmbed(argv='',banner = 'OpenRAVE Dropping into IPython, variables: env, robot',exit_msg = 'Leaving Interpreter and closing program.')
#         ipshell(local_ns=locals())

        success = basemanip.MoveHandStraight(direction=updir,stepsize=0.01,minsteps=1,maxsteps=40)
        robot.WaitForController(0)
        success = basemanip.MoveHandStraight(direction=-updir,stepsize=0.01,minsteps=1,maxsteps=40)
        robot.WaitForController(0)
        
        # test verification with offset (should succeed)
        T = ikmodel.manip.GetEndEffectorTransform()
        T[1,3] += 0.1
        success = basemanip.MoveHandStraight(direction=updir,starteematrix=T,stepsize=0.01,minsteps=1,maxsteps=20)
        robot.WaitForController(0)
     
        print 'checking for existance of trajectories with random queries of moving in a straight line'
        armlength = 0
        armjoints = [j for j in robot.GetDependencyOrderedJoints() if j.GetJointIndex() in ikmodel.manip.GetArmIndices()]
        eetrans = ikmodel.manip.GetEndEffectorTransform()[0:3,3]
        for j in armjoints[::-1]:
            armlength += sqrt(sum((eetrans-j.GetAnchor())**2))
            eetrans = j.GetAnchor()
        stepsize=0.01
        failedattempt = 0
        while True:
            with env:
                #Tee = dot(ikmodel.manip.GetEndEffectorTransform(),matrixFromAxisAngle(random.rand(3)-0.5,0.2*random.rand()))
                Tee = matrixFromAxisAngle(random.rand(3)-0.5,pi*random.rand())
                direction = random.rand(3)
                direction /= linalg.norm(direction)
                x = random.rand(3)-0.5
                length = 0.2*random.rand()*armlength
                Tee[0:3,3] = eetrans + x/linalg.norm(x)*(armlength-length)
                maxsteps=int(length/stepsize)
                minsteps = maxsteps/3
                h = env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),1)
            try:
                success = basemanip.MoveHandStraight(direction=direction,starteematrix=Tee,stepsize=stepsize,minsteps=minsteps,maxsteps=maxsteps)
                params = (direction,Tee)
                print '%d failed attemps before found'%failedattempt
                failedattempt = 0
                h = env.drawlinelist(array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),4,[0,0,1])
                robot.WaitForController(0)
            except planning_error,e:
                failedattempt += 1
            
    finally:
        RaveDestroy()

if __name__ == "__main__":
    run()
