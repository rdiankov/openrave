#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
from optparse import OptionParser
import os
import scipy

if __name__ == "__main__":
    parser = OptionParser(description='Preprocessing all *.robot.xml files and generates images for them')
    parser.add_option('--outdir', action="store",type='string',dest='outdir',default='images/robots',
                      help='Directory to output the images' )
    parser.add_option('--robotdir', action="store",type='string',dest='robotdir',default=None,
                      help='Directory to output the images' )
    parser.add_option('--robot', action="append",type='string',dest='robotfiles',default=[],
                      help='Robot file to store' )
    (options, args) = parser.parse_args()
    env=Environment()
    try:
        env.SetViewer('qtcoin')
        viewer=env.GetViewer()
        width=640
        height=480
        focal = 1000.0
        ext='.robot.xml'
        robotfiles = options.robotfiles
        if options.robotdir is not None:
            robotfiles += [os.path.join(options.robotdir,name) for name in os.listdir(options.robotdir) if name.endswith(ext)]
        mkdir_recursive(options.outdir)
        for robotfile in robotfiles:
            env.Reset()
            r=env.ReadRobotXMLFile(robotfile)
            if r is None:
                print 'failed ',robotfile
                continue
            robotname = os.path.split(robotfile)[1][:-len(ext)]
            print 'processing ',robotname
            env.AddRobot(r)
            # transform the robot so we don't render it at its axis aligned (white robots completely disappear)
            r.SetTransform(matrixFromQuat(quatRotateDirection([-1,0,0],[-1,1,0.5])))
            ab = r.ComputeAABB()
            K=[focal,focal,width/2,height/2]
            
            L = ab.extents()[0] + max(ab.extents()[1]*focal/(0.5*width),ab.extents()[2]*focal/(0.5*height))
            Tx=transformLookat(lookat=ab.pos(),camerapos=ab.pos()-array([-1.0,0,0])*L,cameraup=[0,0,-1])
            Ix = viewer.GetCameraImage(width=width,height=height,transform=Tx,K=K)
            scipy.misc.pilutil.imsave(os.path.join(options.outdir,'%s.x.jpg'%robotname),Ix)
            
            L = ab.extents()[1] + max(ab.extents()[0]*focal/(0.5*width),ab.extents()[2]*focal/(0.5*height))
            Ty=transformLookat(lookat=ab.pos(),camerapos=ab.pos()-array([0,1.0,0])*L,cameraup=[0,0,-1])
            Iy = viewer.GetCameraImage(width=width,height=height,transform=Ty,K=K)
            scipy.misc.pilutil.imsave(os.path.join(options.outdir,'%s.y.jpg'%robotname),Iy)
            r=None
        print 'finished'
    finally:
        env.Destroy()
