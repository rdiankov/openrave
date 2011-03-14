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
"""Shows how to use RRTs for navigation planning by setting affine degrees of freedom.

.. image:: ../../images/examples/simplenavigation.jpg
  :width: 300

**Running the Example**::

  openrave.py --example simplenavigation


Command-line
------------

.. shell-block:: openrave.py --example simplenavigation --help

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from openravepy.interfaces import BaseManipulation
from openravepy.databases import convexdecomposition
from numpy import *
import numpy,time
from optparse import OptionParser

class SimpleNavigationPlanning(metaclass.AutoReloader):
    def __init__(self,robot,randomize=False,dests=None,switchpatterns=None):
        self.env = robot.GetEnv()
        self.robot = robot
        self.cdmodel = convexdecomposition.ConvexDecompositionModel(self.robot)
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        self.basemanip = BaseManipulation(self.robot)
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
                self.robot.SetActiveDOFs([],Robot.DOFAffine.X|Robot.DOFAffine.Y|Robot.DOFAffine.RotationAxis,[0,0,1])
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

@with_destroy
def run(args=None):
    """Executes the simplenavigation example

    :type args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Simple navigation planning using RRTs.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = SimpleNavigationPlanning(robot)
    self.performNavigationPlanning()

if __name__ == "__main__":
    run()
