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
import numpy
import time, pickle, threading
from optparse import OptionParser

class ReachabilityModel(metaclass.AutoReloader):
    def __init__(self):
        pass
    def computetwolinkreachability(self,linklengths=array((1.0,0.5)),jointlimits=array(((-pi/4,pi/4),(-pi/4,pi/4)))):
        delta = 0.01
        maxradius = sum(linklengths)
        nsteps = floor(maxradius/delta)
        X,Y = mgrid[-nsteps:nsteps,-nsteps:nsteps]
        allpoints = c_[X.flat,Y.flat]*delta
        insideinds = flatnonzero(sum(allpoints**2,1)<maxradius**2)
        reachabilitydensity = zeros(prod(X.shape))
        for i in insideinds:
            pt = allpoints[i,:]
            cosang1 = 0.5*(sum(pt**2)-sum(linklengths**2))/(linklengths[0]*linklengths[1])
            if cosang1 < -1 or cosang1 > 1:
                continue
            for ang1 in [math.acos(cosang1),pi-math.acos(cosang1)]:
                if ang1 < jointlimits[1,0] or mod(ang1-jointlimits[1,0],2*pi) > jointlimits[1,1]-jointlimits[1,0]:
                    continue
                print ang1
                lx = linklengths[1]*cos(ang1)+linklengths[0]
                ly = linklengths[1]*sin(ang1)
                ang0 = math.atan2(lx*pt[1]-ly*pt[0],lx*pt[0]+ly*pt[1])
                if ang0 < jointlimits[0,0] or mod(ang0-jointlimits[0,0],2*pi) > jointlimits[0,1]-jointlimits[0,0]:
                    continue
                reachabilitydensity[i] += 1.0
        reachabilitydensity = reshape(reachabilitydensity,X.shape)
        return reachabilitydensity
