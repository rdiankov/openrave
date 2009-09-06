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

if __name__ == "__main__":
    print 'Example shows how to query collision detection information using openravepy'
    env = Environment()
    env.Load('robots/barrettwam.robot.xml')
    orobj = env.GetRobots()[0]
    # when doing fast ray collision checking, can specify multiple rays where each column is one ray
    ray1 = array((0,0,-10,0,0,100))
    ray2 = array((0,0,10,0,0,-100))
    inliers,hitpoints = env.CheckCollisionRays(c_[ray1,ray2],orobj)
    print 'rays hit:',inliers,'hit points:',hitpoints

    # can get more fine grained information
    report1 = CollisionReport()
    inlier1 = env.CheckCollision(Ray(ray1[0:3],ray1[3:6]),report1)
    print 'numcontacts: ',len(report1.contacts),' pos: ', report1.contacts[0].pos[:,0],' norm: ',report1.contacts[0].norm[:,0]
    
