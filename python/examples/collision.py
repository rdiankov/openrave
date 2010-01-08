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
from __future__ import with_statement # for python 2.5

from openravepy import *
from numpy import *

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
    print 'form physics: ',fromphysics
    return CollisionAction.DefaultAction

if __name__ == "__main__":
    print 'Example shows how to query collision detection information using openravepy'
    env = Environment()
    env.Load('robots/barrettwam.robot.xml')
    
    # register an optional collision callback
    handle = env.RegisterCollisionCallback(collisioncallback)
    
    robot1 = env.GetRobots()[0]
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
    
    robot2 = env.ReadRobotXMLFile('robots/pa10.robot.xml')
    env.AddRobot(robot2)
    body1 = env.ReadRobotXMLFile('data/mug1.kinbody.xml')
    env.AddRobot(body1)
    
    env.CheckCollision(robot1,robot2)
    env.CheckCollision(robot1,body1)
    env.CheckCollision(robot2,body1)
    env.Destroy() # done with the environment
