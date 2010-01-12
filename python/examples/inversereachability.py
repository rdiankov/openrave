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
import time,pickle
from optparse import OptionParser

class InverseReachabilityModel(OpenRAVEModel):
    def __init__(self,env,robot,target):
        OpenRAVEModel.__init__(self,env=env,robot=robot)
        
    def has(self):
        return len(self.reachabilitydensity3d) > 0

    def load(self):
        params = OpenRAVEModel.load(self)
        if params is None:
            return False
        self.reachabilitystats,self.reachabilitydensity3d,self.pointscale = params
        return self.has()

    def save(self):
        OpenRAVEModel.save(self,(self.reachabilitystats,self.reachabilitydensity3d,self.pointscale))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'reachability.' + self.manip.GetName() + '.pp')

    def generateFromOptions(self,options):
        self.generate(xyzthresh=options.xyzthresh,rotthresh=options.rotthresh)

    def generate(self,xyzthresh=0.02,rotthresh=pi/16.0):
        pass

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.add_option('--xyzthresh',action='store',type='float',dest='xyzdelta',default=0.02,
                          help='The max radius of the arm to perform the computation')
        parser.add_option('--rotthresh',action='store',type='float',dest='rotthresh',default=pi/16.0,
                          help='The max radius of the arm to perform the computation')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = ReachabilityModel.CreateOptionParser()
        (options, args) = parser.parse_args()
        Model = lambda env,robot: InverseReachabilityModel(env=env,robot=robot)
        OpenRAVEModel.RunFromParser(Model=Model,parser=parser)

if __name__ == "__main__":
    InverseReachabilityModel.RunFromParser()
