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
    inliers,hitpoints = env.CheckCollisionRays(c_[ray1,ray2],robot1)
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
