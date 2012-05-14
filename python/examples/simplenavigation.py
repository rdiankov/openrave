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
"""Shows how to use RRTs for navigation planning by setting affine degrees of freedom.

.. examplepre-block:: simplenavigation
  :image-width: 400

.. examplepost-block:: simplenavigation
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time,numpy
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class SimpleNavigationPlanning:
    def __init__(self,robot,randomize=False,dests=None,switchpatterns=None):
        self.env = robot.GetEnv()
        self.robot = robot
        self.cdmodel = databases.convexdecomposition.ConvexDecompositionModel(self.robot)
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        self.basemanip = interfaces.BaseManipulation(self.robot)
    def performNavigationPlanning(self):
        # find the boundaries of the environment
        with self.env:
            envmin = []
            envmax = []
            for b in self.env.GetBodies():
                ab = b.ComputeAABB()
                envmin.append(ab.pos()-ab.extents())
                envmax.append(ab.pos()+ab.extents())
            abrobot = self.robot.ComputeAABB()
            envmin = numpy.min(array(envmin),0)+abrobot.extents()
            envmax = numpy.max(array(envmax),0)-abrobot.extents()
        bounds = array(((envmin[0],envmin[1],-pi),(envmax[0],envmax[1],pi)))
        while True:
            with self.env:
                self.robot.SetAffineTranslationLimits(envmin,envmax)
                self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
                self.robot.SetAffineRotationAxisMaxVels(ones(4))
                self.robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
                # pick a random position
                with self.robot:
                    while True:
                        goal = bounds[0,:]+random.rand(3)*(bounds[1,:]-bounds[0,:])
                        self.robot.SetActiveDOFValues(goal)
                        if not self.env.CheckCollision(self.robot):
                            break
            print 'planning to: ',goal
            # draw the marker
            center = r_[goal[0:2],0.2]
            xaxis = 0.5*array((cos(goal[2]),sin(goal[2]),0))
            yaxis = 0.25*array((-sin(goal[2]),cos(goal[2]),0))
            h = self.env.drawlinelist(transpose(c_[center-xaxis,center+xaxis,center-yaxis,center+yaxis,center+xaxis,center+0.5*xaxis+0.5*yaxis,center+xaxis,center+0.5*xaxis-0.5*yaxis]),linewidth=5.0,colors=array((0,1,0)))
            if self.basemanip.MoveActiveJoints(goal=goal,maxiter=3000,steplength=0.1) is None:
                print 'retrying...'
                continue
            print 'waiting for controller'
            self.robot.WaitForController(0)

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = SimpleNavigationPlanning(robot)
    self.performNavigationPlanning()

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Simple navigation planning using RRTs.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
