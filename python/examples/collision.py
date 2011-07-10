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
"""Check collision calls, use collision reports, and do distance queries.

.. examplepre-block:: collision
  :image-width: 300

.. examplepost-block:: collision
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = '2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import time
import openravepy
if not __openravepy_build_doc__:
    from numpy import *
    from openravepy import *

def collisioncallback(report,fromphysics):
    """Whenever a collision or physics detects a collision, this function is called"""
    s = 'collision callback with '
    if report.plink1 is not None:
        s += '%s:%s '%(report.plink1.GetParent().GetName(),report.plink1.GetName())
    else:
        s += '(NONE)'
    s += ' x '
    if report.plink2 is not None:
        s += ' %s:%s'%(report.plink2.GetParent().GetName(),report.plink2.GetName())
    else:
        s += '(NONE)'
    print s
    print 'from physics: ',fromphysics
    return CollisionAction.DefaultAction

def main(env,options):
    "Main example code."
    env.Load('robots/barrettwam.robot.xml')
    # register an optional collision callback
    handle = env.RegisterCollisionCallback(collisioncallback)
    robot1 = env.GetRobots()[0]

    try:
        # when doing fast ray collision checking, can specify multiple rays where each column is one ray
        ray1 = array((0,0,-10,0,0,100)) # specify dir*range, pos
        ray2 = array((0,0,10,0,0,-100)) # specify dir*range, pos
        inliers,hitpoints = env.CheckCollisionRays(r_[[ray1],[ray2]],robot1)
        print 'rays hit:',inliers,'hit points:',hitpoints

        # can get more fine grained information
        report1 = CollisionReport()
        inlier1 = env.CheckCollision(Ray(ray1[0:3],ray1[3:6]))
        inlier1 = env.CheckCollision(Ray(ray1[0:3],ray1[3:6]),report1)
        print 'numcontacts: ',len(report1.contacts),' pos: ', report1.contacts[0].pos,' norm: ',report1.contacts[0].norm
    except openrave_exception,e:
        print e

    robot2 = env.ReadRobotXMLFile('robots/mitsubishi-pa10.zae')
    env.AddRobot(robot2)
    body1 = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
    env.AddKinBody(body1)

    env.CheckCollision(robot1,robot2)
    env.CheckCollision(robot1,body1)
    env.CheckCollision(robot2,body1)

    print 'checking self collision'
    robot1.SetDOFValues([2.98],[3])
    if not robot1.CheckSelfCollision():
        print 'no collision detected (bad)!!, ',env.CheckCollision(robot1.GetLinks()[1],robot1.GetLinks()[13])

    handle.close()
    # test distance queries
    T = eye(4)
    T[0,3] = 0.5
    robot1.SetTransform(T)
    T = eye(4)
    T[1,3] = 0.5
    body1.SetTransform(T)

    print 'move the robots to update the closest distance'
    if not env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts):
        print 'current checker does not support distance, switching to pqp...'
        collisionChecker = RaveCreateCollisionChecker(env,'pqp')
        collisionChecker.SetCollisionOptions(CollisionOptions.Distance|CollisionOptions.Contacts)
        env.SetCollisionChecker(collisionChecker)
    report = CollisionReport()
    while True:
        contacts = []
        with env:
            check = env.CheckCollision(robot1, report)
            print 'mindist: ',report.minDistance
            contacts += report.contacts
            check = env.CheckCollision(robot2, report)
            print 'mindist: ',report.minDistance
            contacts += report.contacts
            check = env.CheckCollision(body1, report)
            print 'mindist: ',report.minDistance
            contacts += report.contacts
        handles = [env.drawlinestrip(points=array((c.pos,c.pos-c.depth*c.norm)), linewidth=3.0, colors=array((1,0,0,1))) for c in contacts]
        time.sleep(0.1)

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Example shows how to query collision detection information using openravepy')
    OpenRAVEGlobalArguments.addOptions(parser)
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__ == "__main__":
    run()
