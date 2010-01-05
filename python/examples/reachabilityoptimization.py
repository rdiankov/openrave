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
