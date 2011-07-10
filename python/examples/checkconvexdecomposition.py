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
"""Builds the convex decomposition of the robot and plots all points inside its volume.

.. examplepre-block:: checkconvexdecomposition
  :image-width: 400

Description
-----------

Uses :meth:`.ConvexDecompositionModel.testPointsInside` from :mod:`.convexdecomposition`.

.. examplepost-block:: checkconvexdecomposition

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import openravepy
from openravepy import databases
import numpy

def main(env,options):
    "Main example code."
    samplingdelta = options.samplingdelta
    env.Load(options.target)
    body = env.GetBodies()[0]
    cdmodel = databases.convexdecomposition.ConvexDecompositionModel(body)
    if not cdmodel.load():
        cdmodel.autogenerate()
    ab = body.ComputeAABB()
    if samplingdelta is None:
        samplingdelta = numpy.linalg.norm(ab.extents())/30.0
    boxmin = ab.pos()-ab.extents()
    boxmax = ab.pos()+ab.extents()
    X,Y,Z = numpy.mgrid[boxmin[0]:boxmax[0]:samplingdelta,boxmin[1]:boxmax[1]:samplingdelta,boxmin[2]:boxmax[2]:samplingdelta]
    points = numpy.c_[X.flat,Y.flat,Z.flat]
    print 'computing %d points...'%len(points)
    inside = cdmodel.testPointsInside(points)
    plottedpoints = points[numpy.flatnonzero(inside),:]
    plottedpoints[:,1] += ab.extents()[1]*2
    print '%d points are inside'%len(plottedpoints)
    h = env.plot3(plottedpoints,2)
    if not options.testmode:
        raw_input('press any key to exit')

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Builds the convex decomposition of the robot and plots all the points that are tested inside of it.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--target', action="store",type='string',dest='target',default='robots/barrettwam.robot.xml',
                      help='Target body to load (default=%default)')
    parser.add_option('--samplingdelta', action="store",type='float',dest='samplingdelta',default=None,
                      help='The sampling rate for the robot (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)
    main(env,options)

if __name__ == "__main__":
    run()
