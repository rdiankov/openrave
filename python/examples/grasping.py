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

    def init(self,friction,avoidlinks,plannername=None):
        self.grasper = Grasper(self.env,self.robot,friction,avoidlinks,plannername)
        self.grasps = []
        self.graspindices = dict()

    def load(self):
        params = OpenRAVEModel.load(self)
        if params is None:
            return False;
        self.grasps,self.graspindices,friction,avoidlinks,plannername = params
        self.grasper = Grasper(self.env,self.robot,friction,avoidlinks,plannername)
        return self.has()

    def save(self):
        OpenRAVEModel.save(self,(self.grasps,self.graspindices,self.grasper.friction,self.grasper.avoidlinks,self.grasper.plannername))

    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'graspset.' + self.manip.GetName() + '.' + self.target.GetKinematicsGeometryHash()+'.pp')

    def generate(self,preshapes,standoffs,rolls,approachrays, graspingnoise=None,addSphereNorms=False,updateenv=True,forceclosurethreshold=1e-9):
        """all grasp parameters have to be in the bodies's coordinate system (ie: approachrays)"""
        N = approachrays.shape[0]
        Ttarget = self.target.GetTransform()
        # transform each ray into the global coordinate system in order to plot it
        gapproachrays = c_[dot(approachrays[:,0:3],transpose(Ttarget[0:3,0:3]))+tile(Ttarget[0:3,3],(N,1)),dot(approachrays[:,3:6],transpose(Ttarget[0:3,0:3]))]
        approachgraphs = [self.env.plot3(points=gapproachrays[:,0:3],pointsize=5,colors=array((1,0,0))),
                          self.env.drawlinelist(points=reshape(c_[gapproachrays[:,0:3],gapproachrays[:,0:3]+0.005*gapproachrays[:,3:6]],(2*N,3)),linewidth=4,colors=array((1,0,0,0)))]
        contactgraph = None
        statesaver = self.robot.CreateKinBodyStateSaver()
        try:
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
                print 'grasp %d/%d'%(counter,totalgrasps)
                counter += 1
                grasp = zeros(totaldof)
                grasp[self.graspindices.get('igrasppos')] = approachray[0:3]
                grasp[self.graspindices.get('igraspdir')] = -approachray[3:6]
                grasp[self.graspindices.get('igrasproll')] = roll
                grasp[self.graspindices.get('igraspstandoff')] = standoff
                grasp[self.graspindices.get('igrasppreshape')] = preshape

                try:
                    contacts,finalconfig,mindist,volume = self.runGrasp(grasp,graspingnoise=graspingnoise,translate=True,forceclosure=True)
                except ValueError, e:
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

    def show(self,delay=0.5):
        statesaver = self.robot.CreateRobotStateSaver()
        try:
            for i,grasp in enumerate(self.grasps):
                print 'grasp %d/%d'%(i,len(self.grasps))
                contacts,finalconfig,mindist,volume = self.runGrasp(grasp,translate=True)
                contactgraph = self.drawContacts(contacts) if len(contacts) > 0 else None
                self.robot.SetJointValues(finalconfig[0])
                self.robot.SetTransform(finalconfig[1])
                self.env.UpdatePublishedBodies()
                time.sleep(delay)
        finally:
            statesaver = None # force restoring

    def generateFromOptions(self,options):
        print 'attempting default grasp generation for %s:%s:%s'%(self.robot.GetName(),self.manip.GetName(),self.target.GetName())
        self.init(friction=0.4,avoidlinks=[])
        if self.robot.GetName() == 'BarrettHand' or self.robot.GetName() == 'BarrettWAM':
            preshapes = array(((0.5,0.5,0.5,pi/3),(0.5,0.5,0.5,0),(0,0,0,pi/2)))
        else:
            manipprob = BaseManipulation(self.env,self.robot)
            self.target.Enable(False)
            manipprop.ReleaseFingers(True)
            self.robot.WaitForController(0)
            self.target.Enable(True)
            preshapes = array([self.robot.GetJointValues()])
        self.generate(preshapes=preshapes, rolls = arange(0,2*pi,pi/2), standoffs = array([0,0.025]),
                      approachrays = self.computeBoxApproachRays(stepsize=0.02),
                      updateenv=options.useviewer, addSphereNorms=False)

    def autogenerate(self,forcegenerate=True):
        # disable every body but the target and robot
        bodies = [b for b in self.env.GetBodies() if b.GetNetworkId() != self.robot.GetNetworkId() and b.GetNetworkId() != self.target.GetNetworkId()]
        for b in bodies:
            b.Enable(False)
        try:
            if self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531' and self.manip.GetName() == 'arm' and self.target.GetKinematicsGeometryHash() == 'bbf03c6db8efc712a765f955a27b0d0f':
                self.init(friction=0.4,avoidlinks=[])
                self.generate(preshapes=array(((0.5,0.5,0.5,pi/3),(0.5,0.5,0.5,0),(0,0,0,pi/2))),
                                      rolls = arange(0,2*pi,pi/2), standoffs = array([0,0.025]),
                                      approachrays = self.computeBoxApproachRays(stepsize=0.02))
            else:
                if not forcegenerate:
                    raise ValueError('failed to find auto-generation parameters')
                print 'attempting default grasp generation for %s:%s:%s'%(self.robot.GetName(),self.manip.GetName(),self.target.GetName())
                self.init(friction=0.4,avoidlinks=[])
                if self.robot.GetName() == 'BarrettHand' or self.robot.GetName() == 'BarrettWAM':
                    preshapes = array(((0.5,0.5,0.5,pi/3),(0.5,0.5,0.5,0),(0,0,0,pi/2)))
                else:
                    manipprob = BaseManipulation(self.env,self.robot)
                    self.target.Enable(False)
                    manipprop.ReleaseFingers(True)
                    self.robot.WaitForController(0)
                    self.target.Enable(True)
                    preshapes = array([self.robot.GetJointValues()])
                self.generate(preshapes=preshapes, rolls = arange(0,2*pi,pi/2), standoffs = array([0,0.025]),
                              approachrays = self.computeBoxApproachRays(stepsize=0.02),
                              updateenv=options.useviewer, addSphereNorms=False)
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

    def computeValidGrasps(self,checkcollision=True,checkik=True,returnfirst=False):
        """Returns the set of grasps that satisfy certain conditions"""
        with self.robot:
            validgrasps = []
            for grasp in self.grasps:
                self.setPreshape(grasp)
                Tglobalgrasp = self.getGlobalGraspTransform(grasp)
                if checkik:
                    if self.manip.FindIKSolution(Tglobalgrasp,True) is None:
                        continue
                elif checkcollision:
                    if self.manip.CheckEndEffectorCollision(Tglobalgrasp):
                        continue
                validgrasps.append(grasp)
                if returnfirst:
                    return validgrasps
            return validgrasps

    def computeBoxApproachRays(self,stepsize=0.02):
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
                XX,YY = meshgrid(r_[arange(-ex,-0.25*stepsize,stepsize),0,arange(stepsize,ex,stepsize)],
                                 r_[arange(-ey,-0.25*stepsize,stepsize),0,arange(stepsize,ey,stepsize)])
                localpos = outer(XX.flatten(),side[6:9]/ex)+outer(YY.flatten(),side[9:12]/ey)
                N = localpos.shape[0]
                rays = c_[tile(p+side[0:3],(N,1))+localpos,maxlen*tile(side[3:6],(N,1))]
                collision, info = self.env.CheckCollisionRays(rays,self.target)
                # make sure all normals are the correct sign: pointing outward from the object)
                newinfo = info[collision,:]
                newinfo[sum(rays[collision,3:6]*newinfo[:,3:6],1)>0,3:6] *= -1
                approachrays = r_[approachrays,newinfo]
            return approachrays

    def drawContacts(self,contacts,conelength=0.03,transparency=0.5):
        angs = linspace(0,2*pi,10)
        conepoints = r_[[[0,0,0]],conelength*c_[self.grasper.friction*cos(angs),self.grasper.friction*sin(angs),ones(len(angs))]]
        triinds = array(c_[zeros(len(angs)),range(2,1+len(angs))+[1],range(1,1+len(angs))].flatten(),int)
        allpoints = zeros((0,3))
        for c in contacts:
            rotaxis = cross(array((0,0,1)),c[3:6])
            sinang = sqrt(sum(rotaxis**2))
            if sinang > 1e-4:
                R = rotationMatrixFromAxisAngle(rotaxis/sinang,math.atan2(sinang,c[5]))
            else:
                R = eye(3)
                R[1,1] = R[2,2] = sign(c[5])
            points = dot(conepoints,transpose(R)) + tile(c[0:3],(conepoints.shape[0],1))
            allpoints = r_[allpoints,points[triinds,:]]
        return self.env.drawtrimesh(points=allpoints,indices=None,colors=array((1,0.4,0.4,transparency)))
    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.description='Grasp set generation example for any robot/body pair.'
        parser.add_option('--target',action="store",type='string',dest='target',default='data/mug1.kinbody.xml',
                          help='The filename of the target body whose grasp set to be generated')
        parser.add_option('--noviewer', action='store_false', dest='useviewer',default=True,
                          help='If specified, will generate the tables without launching a viewer')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = GraspingModel.CreateOptionParser()
        (options, args) = parser.parse_args()
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
