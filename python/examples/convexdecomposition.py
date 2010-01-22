#!/usr/bin/env python
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
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import openravepy
from openravepy import *
from openravepy import convexdecompositionpy
from numpy import *
import time
from optparse import OptionParser

class ConvexDecompositionModel(OpenRAVEModel):
    """Computes the robot manipulator's reachability space (stores it in 6D) and
    offers several functions to use it effectively in planning."""
    def __init__(self,robot):
        OpenRAVEModel.__init__(self,robot=robot)
        self.pointscale = None

    def has(self):
        return len(self.reachabilitydensity3d) > 0

    def load(self):
        params = OpenRAVEModel.load(self)
        if params is None:
            return False
        self.reachabilitystats,self.reachabilitydensity3d,self.pointscale,self.xyzdelta,self.quatdelta = params
        return self.has()

    def save(self):
        OpenRAVEModel.save(self,(self.reachabilitystats,self.reachabilitydensity3d,self.pointscale,self.xyzdelta,self.quatdelta))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'convexdecomposition.pp')

    def generateFromOptions(self,options):
        self.generate(maxradius=options.maxradius,xyzdelta=options.xyzdelta,quatdelta=options.quatdelta)

    def generate(self,maxradius=None,translationonly=False,xyzdelta=0.04,quatdelta=0.5):
        starttime = time.time()
        print 'all convex decomposition finished in %fs'%(time.time()-starttime)

    def autogenerate(self,forcegenerate=True):
        if self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531' and self.manip.GetName() == 'arm':
            self.generate(maxradius=1.1)
        else:
            if not forcegenerate:
                raise ValueError('failed to find auto-generation parameters')
            self.generate()
            raise ValueError('could not auto-generate reachability for %s:%s'%(self.robot.GetName(),self.manip.GetName()))
        self.save()

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.description='Computes the reachability region of a robot manipulator and python pickles it into a file.'
        parser.add_option('--maxradius',action='store',type='float',dest='maxradius',default=None,
                          help='The max radius of the arm to perform the computation')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = ConvexDecompositionModel.CreateOptionParser()
        env = Environment()
        try:
            if Model is None:
                Model = lambda robot: ConvexDecompositionModel(robot=robot)
            OpenRAVEModel.RunFromParser(env=env,Model=Model,parser=parser)
        finally:
            env.Destroy()

if __name__=='__main__':
    ConvexDecompositionModel.RunFromParser()
