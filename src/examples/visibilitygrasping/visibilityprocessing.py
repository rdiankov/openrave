#!/usr/bin/env python
#
# Copyright (C) 2009 Rosen Diankov
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
import os, time, pickle
import numpy,scipy # nice to be able to explicitly call some functions
from numpy import *
from optparse import OptionParser
from openravepy import *

def GetCameraRobotMask(orenv,robotfile,sensorindex=0,robotjoints=None,robotjointinds=None,gripperjoints=None,rayoffset=0):
    orenv.Reset()
    # ode returns better ray casts
    if not orenv.GetCollisionChecker().GetXMLId() == 'ode':
        orcol = orenv.CreateCollisionChecker('ode')
        if orcol is not None:
            print 'setting ode checker'
            orenv.SetCollisionChecker(orcol)
    #orenv.SetViewer('qtcoin',True)
    orrobot = orenv.ReadRobotXMLFile(robotfile)
    orenv.AddRobot(orrobot)
    if robotjoints is not None:
        if robotjointinds is not None:
            orrobot.SetJointValues(robotjoints,robotjointinds)
        else:
            orrobot.SetJointValues(robotjoints)
    
    attached = orrobot.GetSensors()[sensorindex]
    attached.GetSensor().SendCommand('power 0')
    Tcamera = attached.GetSensor().GetTransform()
    sensordata = attached.GetSensor().GetSensorData()
    KK = sensordata.KK
    height = sensordata.imagedata.shape[0]
    width = sensordata.imagedata.shape[1]

    # find a manipulator whose end effector is the camera
    manip = [m for m in orrobot.GetManipulators() if m.GetEndEffector().GetIndex() == attached.GetAttachingLink().GetIndex()]
    if len(manip) > 0 and gripperjoints is not None:
        assert len(manip[0].GetGripperJoints()) == len(gripperjoints)
        orrobot.SetJointValues(gripperjoints,manip[0].GetGripperJoints())
    
    inds = array(range(width*height))
    imagepoints = array((mod(inds,width),floor(inds/width)))
    camerapoints = dot(linalg.inv(KK), r_[imagepoints,ones((1,imagepoints.shape[1]))])
    raydirs = dot(Tcamera[0:3,0:3], camerapoints / tile(sqrt(sum(camerapoints**2,0)),(3,1)))
    rays = r_[tile(Tcamera[0:3,3:4],(1,raydirs.shape[1]))+rayoffset*raydirs,100.0*raydirs]
    hitindices,hitpositions = orenv.CheckCollisionRays(rays,orrobot,False)

    attached = None
    manip = None
    orrobot = None
    manip = None
    # gather all the rays that hit and form an image
    return reshape(array(hitindices,'float'),(height,width))

def ProcessVisibilityExtents(orenv,robotfile,kinbodyfile,convexfilename,visibilityfilename,sensorindex=0,robotjoints=None,robotjointinds=None,gripperjoints=None):
    orenv.Reset()
    orenv.LockPhysics(True)
    # ode returns better ray casts
    if not orenv.GetCollisionChecker().GetXMLId() == 'ode':
        orcol = orenv.CreateCollisionChecker('ode')
        if orcol is not None:
            print 'setting ode checker'
            orenv.SetCollisionChecker(orcol)
    orrobot = orenv.ReadRobotXMLFile(robotfile)
    orenv.AddRobot(orrobot)
    orobj = orenv.ReadKinBodyXMLFile(kinbodyfile)
    orenv.AddKinBody(orobj)

    if robotjoints is not None:
        if robotjointinds is not None:
            orrobot.SetJointValues(robotjoints,robotjointinds)
        else:
            orrobot.SetJointValues(robotjoints)

    # find a manipulator whose end effector is the camera
    attached = orrobot.GetSensors()[sensorindex]
    manip = [m for m in orrobot.GetManipulators() if m.GetEndEffector().GetIndex() == attached.GetAttachingLink().GetIndex()]
    if len(manip) > 0 and gripperjoints is not None:
        assert len(manip[0].GetGripperJoints()) == len(gripperjoints)
        orrobot.SetJointValues(gripperjoints,manip[0].GetGripperJoints())

    # move the robot away from the object
    T = eye(4)
    T[0,3] = 4
    orrobot.SetTransform(T)
    visualprob = orenv.CreateProblem('VisualFeedback')
    orenv.LoadProblem(visualprob,orrobot.GetName())
    response = visualprob.SendCommand('ProcessVisibilityExtents target %s sensorindex %d convexfile %s extentsfile %s'%(orobj.GetName(),sensorindex,convexfilename,visibilityfilename))
    orenv.LockPhysics(False)
    if response is None:
        raise ValueError('ProcessVisibilityExtents failed')
    
    transforms = [float(s) for s in response.split()]
    if not mod(len(transforms),7) == 0:
        raise ValueError('corrupted transforms')

    del visualprob
    del manip
    del attached
    del orrobot
    del orobj
    return reshape(transforms,(len(transforms)/7,7))

if __name__=='__main__':
    parser = OptionParser(description='Helper functions for processing the gripper mask and visiblity extents')
    parser.add_option('--func',
                      action="store",type='string',dest='func',default='mask',
                      help='Function to perform, can be {mask,visibility}')   
    parser.add_option('--robotfile',
                      action="store",type='string',dest='robotfile',
                      help='OpenRAVE robot file to generate mask')
    parser.add_option('--kinbodyfile',
                      action="store",type='string',dest='kinbodyfile',
                      help='OpenRAVE kinbody target filename')
    parser.add_option('--gripperjoints',
                      action="store",type='string',dest='gripperjoints',default=None,
                      help='OpenRAVE robot file to generate mask')
    parser.add_option('--graspsetfile',
                      action="store",type='string',dest='graspsetfile',default=None,
                      help='OpenRAVE grasp set file to get gripper joints from')
    parser.add_option('--robotjoints',
                      action="store",type='string',dest='robotjoints',default=None,
                      help='Robot joints to preset robot with (used with --robotjointinds)')
    parser.add_option('--robotjointinds',
                      action="store",type='string',dest='robotjointinds',default=None,
                      help='Robot joint indices to use in conjunction with robotjoints')
    parser.add_option('--sensorindex',
                      action="store",type='int',dest='sensorindex',default=0,
                      help='Sensor index to use (default is 0)')
    parser.add_option('--savefile',
                      action="store",type='string',dest='savefile',default='mask.mat',
                      help='Filename to save matlab data into (default is mask.mat)')
    parser.add_option('--savepp',
                      action="store",type='string',dest='savepp',default=None,
                      help='Save all data into a pp file')
    parser.add_option('--convexfile',
                      action="store",type='string',dest='convexfile',
                      help='Filename specifying convex polygon of gripper mask')
    parser.add_option('--visibilityfile',
                      action="store",type='string',dest='visibilityfile',
                      help='Filename specifying visibility extents of target kinbody')
    parser.add_option('--rayoffset',
                      action="store",type='float',dest='rayoffset',default=0.03,
                      help='The offset to move the ray origin (prevents meaningless collisions), default is 0.03')
    (options, args) = parser.parse_args()

    if not options.robotfile:
        print 'failed to specify robot file'
        sys.exit(1)

    robotjoints = [float(s) for s in options.robotjoints.split()] if options.robotjoints else None
    robotjointinds = [int(s) for s in options.robotjointinds.split()] if options.robotjointinds else None
    if options.graspsetfile is not None:
        graspdata = numpy.loadtxt(options.graspsetfile)
        gripperjoints = graspdata[0][12:]
    if options.gripperjoints is not None:
        gripperjoints = [float(s) for s in options.gripperjoints.split()] if options.gripperjoints else None

    orenv = Environment()
    if options.func == 'mask':
        Imask = GetCameraRobotMask(orenv,options.robotfile,sensorindex=options.sensorindex,gripperjoints=gripperjoints,robotjoints=robotjoints,robotjointinds=robotjointinds,rayoffset=options.rayoffset)
        # save as a ascii matfile
        numpy.savetxt(options.savefile,Imask,'%d')
        print 'mask saved to ' + options.savefile
        try:
            scipy.misc.pilutil.imshow(array(Imask*255,'uint8'))
        except:
            pass
    elif options.func == 'visibility':
        if not options.convexfile:
            print 'need convexfile'
            sys.exit(1)
        if not options.visibilityfile:
            print 'need visibilityfile'
            sys.exit(1)
        if not options.kinbodyfile:
            print 'need kinbodyfile'
            sys.exit(1)
        
        visibilitydata = ProcessVisibilityExtents(orenv,options.robotfile,kinbodyfile=options.kinbodyfile,convexfilename=options.convexfile,visibilityfilename=options.visibilityfile,sensorindex=options.sensorindex,gripperjoints=gripperjoints,robotjoints=robotjoints,robotjointinds=robotjointinds)
        if options.savefile is not None:
            f = open(options.savefile,'w')
            for t in visibilitydata:
                f.write(' '.join(str(val) for val in t)+'\n')
            f.close()
            print '%d transforms saved to %s'%(visibilitydata.shape[0],options.savefile)
    if options.savepp is not None:
        orenv.Reset()
        target = orenv.ReadKinBodyXMLFile(options.kinbodyfile)
        orenv.AddKinBody(target)
        target.SetTransform(eye(4))

        orrobot = orenv.ReadRobotXMLFile(options.robotfile)
        orenv.AddRobot(orrobot)
        if options.robotjoints is not None:
            if options.robotjointinds is not None:
                orrobot.SetJointValues(options.robotjoints,options.robotjointinds)
            else:
                orrobot.SetJointValues(options.robotjoints)

        attached = orrobot.GetSensors()[options.sensorindex]
        # find a manipulator whose end effector is the camera
        manip = [m for m in orrobot.GetManipulators() if m.GetEndEffector().GetIndex() == attached.GetAttachingLink().GetIndex()][0]

        # transform into end effect gripper
        for g in graspdata:
            Tgrasp = r_[transpose(reshape(g[0:12],(4,3))),[[0,0,0,1]]]
            orrobot.SetJointValues(g[12:],manip.GetGripperJoints())
            T = Tgrasp
            while manip.CheckEndEffectorCollision(T,None):
                T = array(Tgrasp)
                T[0:3,3] += 0.004*(random.rand(3)-0.5)
            g[0:12] = reshape(transpose(T[0:3,:]),(12,))
        
        pickle.dump((numpy.loadtxt(options.convexfile),visibilitydata,graspdata),open(options.savepp,'w'))

    orenv.Destroy()
