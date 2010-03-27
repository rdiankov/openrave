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

import os,sys,itertools,traceback,time
from openravepy import *
from openravepy.interfaces import Grasper, BaseManipulation
from openravepy.examples import convexdecomposition
from numpy import *
from optparse import OptionParser

try:
    from itertools import productasdf as iterproduct
except:
    # have to define it
    def iterproduct(*args, **kwds):
        # product('ABCD', 'xy') --> Ax Ay Bx By Cx Cy Dx Dy
        # product(range(2), repeat=3) --> 000 001 010 011 100 101 110 111
        pools = map(tuple, args) * kwds.get('repeat', 1)
        result = [[]]
        for pool in pools:
            result = [x+[y] for x in result for y in pool]
        for prod in result:
            yield tuple(prod)

class GraspingModel(OpenRAVEModel):
    """Holds all functions/data related to a grasp between a robot hand and a target"""
    def __init__(self,robot,target):
        OpenRAVEModel.__init__(self,robot=robot)
        self.target = target
        self.grasps = []
        self.graspindices = dict()
        self.grasper = None
    def has(self):
        return len(self.grasps) > 0 and len(self.graspindices) > 0 and self.grasper is not None
    def getversion(self):
        return 3
    def init(self,friction,avoidlinks,plannername=None):
        self.grasper = Grasper(self.robot,friction,avoidlinks,plannername)
        self.grasps = []
        self.graspindices = dict()

    def load(self):
        try:
            params = OpenRAVEModel.load(self)
            if params is None:
                return False;
            self.grasps,self.graspindices,friction,linknames,plannername = params
            self.grasper = Grasper(self.robot,friction,avoidlinks = [self.robot.GetLink(name) for name in linknames],plannername=plannername)
            return self.has()
        except e:
            return False

    def save(self):
        OpenRAVEModel.save(self,(self.grasps,self.graspindices,self.grasper.friction,[link.GetName() for link in self.grasper.avoidlinks],self.grasper.plannername))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'graspset.' + self.manip.GetName() + '.' + self.target.GetKinematicsGeometryHash()+'.pp')

    class GripperVisibility:
        """When 'entered' will hide all the non-gripper links in order to facilitate visiblity of the gripper"""
        def __init__(self,manip):
            self.manip = manip
            self.robot = self.manip.GetRobot()
            self.hiddengeoms = []
        def __enter__(self):
            self.hiddengeoms = []
            with self.robot.GetEnv():
                # stop rendering the non-gripper links
                childlinkids = [link.GetIndex() for link in self.manip.GetChildLinks()]
                for link in self.robot.GetLinks():
                    if link.GetIndex() not in childlinkids:
                        for geom in link.GetGeometries():
                            self.hiddengeoms.append((geom,geom.IsDraw()))
                            geom.SetDraw(False)
        def __exit__(self,type,value,traceback):
            with self.robot.GetEnv():
                for geom,isdraw in self.hiddengeoms:
                    geom.SetDraw(isdraw)

    def generate(self,preshapes,standoffs,rolls,approachrays, graspingnoise=None,updateenv=True,forceclosurethreshold=1e-9):
        """all grasp parameters have to be in the bodies's coordinate system (ie: approachrays)"""
        print 'Generating Grasp Set for %s:%s:%s'%(self.robot.GetName(),self.manip.GetName(),self.target.GetName())
        time.sleep(0.1) # sleep or otherwise viewer might not load well
        N = approachrays.shape[0]
        Ttarget = self.target.GetTransform()
        # transform each ray into the global coordinate system in order to plot it
        gapproachrays = c_[dot(approachrays[:,0:3],transpose(Ttarget[0:3,0:3]))+tile(Ttarget[0:3,3],(N,1)),dot(approachrays[:,3:6],transpose(Ttarget[0:3,0:3]))]
        approachgraphs = [self.env.plot3(points=gapproachrays[:,0:3],pointsize=5,colors=array((1,0,0))),
                          self.env.drawlinelist(points=reshape(c_[gapproachrays[:,0:3],gapproachrays[:,0:3]+0.005*gapproachrays[:,3:6]],(2*N,3)),linewidth=4,colors=array((1,0,0,1)))]
        contactgraph = None
        statesaver = self.robot.CreateKinBodyStateSaver()
        try:
            with self.GripperVisibility(self.manip):
                totalgrasps = N*len(preshapes)*len(rolls)*len(standoffs)
                counter = 0
                self.grasps = []
                # only the indices used by the TaskManipulation plugin should start with an 'i'
                graspdof = {'igraspdir':3,'igrasppos':3,'igrasproll':1,'igraspstandoff':1,'igrasppreshape':preshapes.shape[1],'igrasptrans':12,'forceclosure':1}
                self.graspindices = dict()
                totaldof = 0
                for name,dof in graspdof.iteritems():
                    self.graspindices[name] = range(totaldof,totaldof+dof)
                    totaldof += dof
                if updateenv:
                    self.env.UpdatePublishedBodies()
                counter = 0
                for approachray,roll,preshape,standoff in iterproduct(approachrays,rolls,preshapes,standoffs):
                    print 'grasp %d/%d'%(counter,totalgrasps),'preshape:',preshape
                    counter += 1
                    grasp = zeros(totaldof)
                    grasp[self.graspindices.get('igrasppos')] = approachray[0:3]
                    grasp[self.graspindices.get('igraspdir')] = -approachray[3:6]
                    grasp[self.graspindices.get('igrasproll')] = roll
                    grasp[self.graspindices.get('igraspstandoff')] = standoff
                    grasp[self.graspindices.get('igrasppreshape')] = preshape
                    try:
                        contacts,finalconfig,mindist,volume = self.runGrasp(grasp,graspingnoise=graspingnoise,translate=True,forceclosure=True)
                    except planning_error, e:
                        print 'Grasp Failed: '
                        traceback.print_exc(e)
                        continue
                    Tgrasp = eye(4)
                    with self.env:
                        self.robot.SetJointValues(finalconfig[0])
                        self.robot.SetTransform(finalconfig[1])
                        Tgrasp = dot(linalg.inv(self.target.GetTransform()),self.manip.GetEndEffectorTransform())
                        if updateenv:
                            contactgraph = self.drawContacts(contacts) if len(contacts) > 0 else None
                            self.env.UpdatePublishedBodies()
                    grasp[self.graspindices.get('igrasptrans')] = reshape(transpose(Tgrasp[0:3,0:4]),12)
                    grasp[self.graspindices.get('forceclosure')] = mindist
                    if mindist > forceclosurethreshold:
                        print 'found good grasp'
                        self.grasps.append(grasp)
                self.grasps = array(self.grasps)
        finally:
            # force closing the handles (if an exception is thrown, python 2.6 does not close them without a finally)
            approachgraphs = None
            contactgraph = None
            statesaver = None

    def show(self,delay=0.5,options=None):
        with RobotStateSaver(self.robot):
            with self.GripperVisibility(self.manip):
                time.sleep(1.0) # let viewer update?
                if options is not None and options.graspindex is not None:
                    print 'showing grasp %d'%options.graspindex
                    grasps = [self.grasps[options.graspindex]]
                    delay=10000000
                else:
                    grasps = self.grasps
                for i,grasp in enumerate(grasps):
                    print 'grasp %d/%d'%(i,len(grasps))
                    try:
                        contacts,finalconfig,mindist,volume = self.runGrasp(grasp,translate=True)
                        contactgraph = self.drawContacts(contacts) if len(contacts) > 0 else None
                        self.robot.GetController().Reset(0)
                        self.robot.SetJointValues(finalconfig[0])
                        self.robot.SetTransform(finalconfig[1])
                        self.env.UpdatePublishedBodies()
                        time.sleep(delay)
                    except planning_error,e:
                        print e
    def showgrasp(self,grasp):
        with RobotStateSaver(self.robot):
            with self.GripperVisibility(self.manip):
                with self.env:
                    self.setPreshape(grasp)
                    Tgrasp = self.getGlobalGraspTransform(grasp)
                    Tdelta = dot(Tgrasp,linalg.inv(self.manip.GetEndEffectorTransform()))
                    for link in self.manip.GetChildLinks():
                        link.SetTransform(dot(Tdelta,link.GetTransform()))
                    self.env.UpdatePublishedBodies()
                raw_input('press any key to continue: ')
    def autogenerate(self,options=None):
        # disable every body but the target and robot
        bodies = [b for b in self.env.GetBodies() if b.GetNetworkId() != self.robot.GetNetworkId() and b.GetNetworkId() != self.target.GetNetworkId()]
        for b in bodies:
            b.Enable(False)
        try:
            friction = None
            preshapes = None
            approachrays = None
            standoffs = None
            rolls = None
            avoidlinks = None
            updateenv=False
            if options is not None:
                if len(options.preshapes) > 0:
                    preshapes = zeros((0,len(self.manip.GetGripperJoints())))
                    for preshape in options.preshapes:
                        preshapes = r_[preshapes,array([float(s) for s in preshape.split()])]
                if options.boxdelta is not None:
                    approachrays = self.computeBoxApproachRays(delta=options.boxdelta,normalanglerange=options.normalanglerange,directiondelta=options.directiondelta)
                elif options.spheredelta is not None:
                    approachrays = self.computeBoxApproachRays(delta=options.spheredelta,normalanglerange=options.normalanglerange,directiondelta=options.directiondelta)
                if len(options.standoffs)>0:
                    standoffs = array(options.standoffs)
                if len(options.rolls)>0:
                    rolls = array(options.rolls)
                if options.friction is not None:
                    friction = options.friction
                if options.avoidlinks is not None:
                    avoidlinks = [self.robot.GetLink(avoidlink) for avoidlink in options.avoidlinks]
                updateenv = options.useviewer
            # check for specific robots
            if self.robot.GetRobotStructureHash() == '7b789782446d86b95c6fb16de7f204c7' and self.manip.GetName() == 'arm' and self.target.GetKinematicsGeometryHash() == 'bbf03c6db8efc712a765f955a27b0d0f':
                if preshapes is None:
                    preshapes=array(((0.5,0.5,0.5,pi/3),(0.5,0.5,0.5,0),(0,0,0,pi/2)))

            if preshapes is None:
                manipprob = BaseManipulation(self.robot)
                with self.target:
                    self.target.Enable(False)
                    self.robot.SetActiveDOFs(self.manip.GetGripperJoints())
                    manipprob.ReleaseFingers(execute=True)
                self.robot.WaitForController(0)
                preshapes = array([self.robot.GetJointValues()[self.manip.GetGripperJoints()]])
            if approachrays is None:
                approachrays = self.computeBoxApproachRays(delta=0.02)
            if rolls is None:
                rolls = arange(0,2*pi,pi/2)
            if standoffs is None:
                standoffs = array([0,0.025])
            if friction is None:
                friction = 0.4
            if avoidlinks is None:
                avoidlinks = []
            self.init(friction=friction,avoidlinks=avoidlinks)
            self.generate(preshapes=preshapes,rolls=rolls,standoffs=standoffs,approachrays=approachrays,updateenv=updateenv)
            self.save()
        finally:
            for b in bodies:
                b.Enable(True)

    def runGrasp(self,grasp,graspingnoise=None,translate=True,forceclosure=False):
        with self.robot: # lock the environment and save the robot state
            self.robot.SetJointValues(grasp[self.graspindices.get('igrasppreshape')],self.manip.GetGripperJoints())
            self.robot.SetActiveDOFs(self.manip.GetGripperJoints(),Robot.DOFAffine.X+Robot.DOFAffine.Y+Robot.DOFAffine.Z if translate else 0)
            return self.grasper.Grasp(direction=grasp[self.graspindices.get('igraspdir')],
                                     roll=grasp[self.graspindices.get('igrasproll')],
                                     position=grasp[self.graspindices.get('igrasppos')],
                                     standoff=grasp[self.graspindices.get('igraspstandoff')],
                                     target=self.target,graspingnoise = graspingnoise,
                                     forceclosure=forceclosure, execute=False, outputfinal=True)

    def getGlobalGraspTransform(self,grasp):
        Tlocalgrasp = eye(4)
        Tlocalgrasp[0:3,0:4] = transpose(reshape(grasp[self.graspindices ['igrasptrans']],(4,3)))
        return dot(self.target.GetTransform(),Tlocalgrasp)
    def setPreshape(self,grasp):
        """sets the preshape on the robot, assumes environment is locked"""
        self.robot.SetJointValues(grasp[self.graspindices['igrasppreshape']],self.manip.GetGripperJoints())

    def computeValidGrasps(self,startindex=0,checkcollision=True,checkik=True,returnnum=inf):
        """Returns the set of grasps that satisfy certain conditions. If returnnum is set, will also return once that many number of grasps are found"""
        with self.robot:
            validgrasps = []
            validindices = []
            report = CollisionReport()
            for i in range(startindex,len(self.grasps)):
                grasp = self.grasps[i]
                self.setPreshape(grasp)
                Tglobalgrasp = self.getGlobalGraspTransform(grasp)
                if checkik:
                    if self.manip.FindIKSolution(Tglobalgrasp,True) is None:
                        continue
                elif checkcollision:
                    if self.manip.CheckEndEffectorCollision(Tglobalgrasp):
                        continue
                validgrasps.append(grasp)
                validindices.append(i)
                if len(validgrasps) == returnnum:
                    return validgrasps,validindices
            return validgrasps,validindices

    def validGraspIterator(self,startindex=0,checkcollision=True,checkik=True,randomgrasps=False):
        """Returns an iterator for valid grasps that satisfy certain conditions."""
        validgrasps = []
        validindices = []
        if randomgrasps:
            order = range(startindex,len(self.grasps))
        else:
            order = startindex+random.permutation(len(self.grasps)-startindex)
        for i in order:
            grasp = self.grasps[i]
            with KinBodyStateSaver(self.robot):
                self.setPreshape(grasp)
                Tglobalgrasp = self.getGlobalGraspTransform(grasp)
                if checkik:
                    if self.manip.FindIKSolution(Tglobalgrasp,True) is None:
                        continue
                elif checkcollision:
                    if self.manip.CheckEndEffectorCollision(Tglobalgrasp):
                        continue
            yield grasp,i

    def computeBoxApproachRays(self,delta=0.02,normalanglerange=0,directiondelta=0.4):
        with self.target:
            self.target.SetTransform(eye(4))
            ab = self.target.ComputeAABB()
            p = ab.pos()
            e = ab.extents()
            sides = array(((0,0,e[2],0,0,-1,e[0],0,0,0,e[1],0),
                           (0,0,-e[2],0,0,1,e[0],0,0,0,e[1],0),
                           (0,e[1],0,0,-1,0,e[0],0,0,0,0,e[2]),
                           (0,-e[1],0,0,1,0,e[0],0,0,0,0,e[2]),
                           (e[0],0,0,-1,0,0,0,e[1],0,0,0,e[2]),
                           (-e[0],0,0,1,0,0,0,e[1],0,0,0,e[2])))
            maxlen = 2*sqrt(sum(e**2))
            approachrays = zeros((0,6))
            for side in sides:
                ex = sqrt(sum(side[6:9]**2))
                ey = sqrt(sum(side[9:12]**2))
                XX,YY = meshgrid(r_[arange(-ex,-0.25*delta,delta),0,arange(delta,ex,delta)],
                                 r_[arange(-ey,-0.25*delta,delta),0,arange(delta,ey,delta)])
                localpos = outer(XX.flatten(),side[6:9]/ex)+outer(YY.flatten(),side[9:12]/ey)
                N = localpos.shape[0]
                rays = c_[tile(p+side[0:3],(N,1))+localpos,maxlen*tile(side[3:6],(N,1))]
                collision, info = self.env.CheckCollisionRays(rays,self.target)
                # make sure all normals are the correct sign: pointing outward from the object)
                newinfo = info[collision,:]
                if len(newinfo) > 0:
                    newinfo[sum(rays[collision,3:6]*newinfo[:,3:6],1)>0,3:6] *= -1
                    approachrays = r_[approachrays,newinfo]
            if normalanglerange > 0:
                theta,pfi = SpaceSampler().sampleS2(angledelta=directiondelta)
                dirs = c_[cos(theta),sin(theta)*cos(pfi),sin(theta)*sin(pfi)]
                dirs = array([dir for dir in dirs if arccos(dir[2])<=normalanglerange]) # find all dirs within normalanglerange
                if len(dirs) == 0:
                    dirs = array([[0,0,1]])
                newapproachrays = zeros((0,6))
                for approachray in approachrays:
                    R = rotationMatrixFromQuat(quatRotateDirection(array((0,0,1)),approachray[3:6]))
                    newapproachrays = r_[newapproachrays,c_[tile(approachray[0:3],(len(dirs),1)),dot(dirs,transpose(R))]]
                approachrays = newapproachrays
            return approachrays
    def computeSphereApproachRays(self,delta=0.1,normalanglerange=0,directiondelta=0.4):
        with self.target:
            self.target.SetTransform(eye(4))
            dirs = newinfo[:,0:3]-tile(p,(len(newinfo),1))
            L=sqrt(sum(dirs**2,1))
            I=flatnonzero(L>1e-8)
            approachrays = r_[approachrays,c_[newinfo[I,0:3],dirs[I,:]/transpose(tile(1.0/L[I],(3,1)))]]

    def drawContacts(self,contacts,conelength=0.03,transparency=0.5):
        angs = linspace(0,2*pi,10)
        conepoints = r_[[[0,0,0]],conelength*c_[self.grasper.friction*cos(angs),self.grasper.friction*sin(angs),ones(len(angs))]]
        triinds = array(c_[zeros(len(angs)),range(2,1+len(angs))+[1],range(1,1+len(angs))].flatten(),int)
        allpoints = zeros((0,3))
        for c in contacts:
            R = rotationMatrixFromQuat(quatRotateDirection(array((0,0,1)),c[3:6]))
            points = dot(conepoints,transpose(R)) + tile(c[0:3],(conepoints.shape[0],1))
            allpoints = r_[allpoints,points[triinds,:]]
        return self.env.drawtrimesh(points=allpoints,indices=None,colors=array((1,0.4,0.4,transparency)))
    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.description='Grasp set generation example for any robot/body pair.'
        parser.add_option('--target',action="store",type='string',dest='target',default='data/mug1.kinbody.xml',
                          help='The filename of the target body whose grasp set to be generated (default=%default)')
        parser.add_option('--noviewer', action='store_false', dest='useviewer',default=True,
                          help='If specified, will generate the tables without launching a viewer')
        parser.add_option('--boxdelta', action='store', type='float',dest='boxdelta',default=None,
                          help='Step size of of box surface sampling')
        parser.add_option('--spheredelta', action='store', type='float',dest='spheredelta',default=None,
                          help='Delta angle between directions on the sphere')
        parser.add_option('--normalanglerange', action='store', type='float',dest='normalanglerange',default=0.0,
                          help='The range of angles around the surface normal to approach from (default=%default)')
        parser.add_option('--directiondelta', action='store', type='float',dest='directiondelta',default=0.4,
                          help='The average distance of approach directions for each surface point in radians (default=%default)')
        parser.add_option('--standoff', action='append', type='float',dest='standoffs',default=[],
                          help='Add a standoff distance')
        parser.add_option('--roll', action='append', type='float',dest='rolls',default=[],
                          help='Add a roll angle')
        parser.add_option('--preshape', action='append', type='string',dest='preshapes',default=[],
                          help='Add a preshape for the manipulator gripper joints')
        parser.add_option('--avoidlink', action='append', type='string',dest='avoidlinks',default=[],
                          help='Add a link name to avoid at all costs (like sensor links)')
        parser.add_option('--friction', action='store', type='float',dest='friction',default=None,
                          help='Friction between robot and target object (default=0.4)')
        parser.add_option('--graspindex', action='store', type='int',dest='graspindex',default=None,
                          help='If set, then will only show this grasp index')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        (options, args) = GraspingModel.CreateOptionParser().parse_args()
        if parser is None:
            parser = GraspingModel.CreateOptionParser()
        env = Environment()
        try:
            target = None
            with env:
                target = env.ReadKinBodyXMLFile(options.target)
                target.SetTransform(eye(4))
                env.AddKinBody(target)
            if Model is None:
                Model = lambda robot: GraspingModel(robot=robot,target=target)
            if options.useviewer:
                env.SetViewer('qtcoin')
                env.UpdatePublishedBodies()
            OpenRAVEModel.RunFromParser(env=env,Model=Model,parser=parser)
        finally:
            env.Destroy()

if __name__ == "__main__":
    GraspingModel.RunFromParser()
