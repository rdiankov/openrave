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
import os,sys,pickle,itertools,traceback
import openravepy
from openravepy import *
from numpy import *

def myproduct(*args, **kwds):
    # product('ABCD', 'xy') --> Ax Ay Bx By Cx Cy Dx Dy
    # product(range(2), repeat=3) --> 000 001 010 011 100 101 110 111
    pools = map(tuple, args) * kwds.get('repeat', 1)
    result = [[]]
    for pool in pools:
        result = [x+[y] for x in result for y in pool]
    for prod in result:
        yield tuple(prod)

class Grasping(metaclass.AutoReloader):
    """Holds all functions/data related to a grasp between a robot hand and a target"""
    def __init__(self,env,robot,target):
        self.env = env
        self.robot = robot
        self.manip = self.robot.GetActiveManipulator()
        self.target = target
        self.grasps = []
        self.graspindices = dict()
        self.grasper = None
        try:
            self.grasps,self.graspindices,friction,avoidlinks,plannername = pickle.load(open(self.getGraspFilename(), 'r'))
            self.grasper = openravepy.interfaces.Grasper(self.env,self.robot,friction,avoidlinks,plannername)
        except IOError:
            print 'failed to find cached grasp set %s'%self.getGraspFilename()

    def initGrasper(self,friction,avoidlinks,plannername=None):
        self.grasper = openravepy.interfaces.Grasper(self.env,self.robot,friction,avoidlinks,plannername)
        self.grasps = []
        self.graspindices = dict()

    def saveGrasps(self):
        print 'saving grasps to %s'%self.getGraspFilename()
        pickle.dump((self.grasps,self.graspindices,self.grasper.friction,self.grasper.avoidlinks,self.grasper.plannername),open(self.getGraspFilename(), 'w'))

    def getGraspFilename(self):
        self._mkdir(os.path.join(self.env.GetHomeDirectory(),self.robot.GetKinematicsGeometryHash()))
        return os.path.join(self.env.GetHomeDirectory(),self.robot.GetKinematicsGeometryHash(),self.target.GetKinematicsGeometryHash()+'.grasp.pp')

    def generateGraspSet(self,preshapes,standoffs,rolls,approachrays, graspingnoise=None,addSphereNorms=False,updateenv=True,forceclosurethreshold=1e-9):
        N = approachrays.shape[0]
        approachgraphs = [self.env.plot3(points=approachrays[:,0:3],pointsize=5,colors=array((1,0,0))),
                          self.env.drawlinelist(points=reshape(c_[approachrays[:,0:3],approachrays[:,0:3]+0.005*approachrays[:,3:6]],(2*N,3)),linewidth=4,colors=array((1,0,0,0)))]
        totalgrasps = N*len(preshapes)*len(rolls)*len(standoffs)
        counter = 0
        self.grasps = []
        graspdof = {'igraspdir':3,'igrasppos':3,'igrasproll':1,'igraspstandoff':1,'igrasppreshape':preshapes.shape[1],'igrasptrans':12,'iforceclosure':1}
        self.graspindices = dict()
        totaldof = 0
        for name,dof in graspdof.iteritems():
            self.graspindices[name] = range(totaldof,totaldof+dof)
            totaldof += dof
        counter = 0
        contactgraph = None
        for approachray,roll,preshape,standoff in myproduct(approachrays,rolls,preshapes,standoffs):
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

            if updateenv:
                self.env.LockPhysics(True)
                self.robot.SetJointValues(finalconfig[0])
                self.robot.SetTransform(finalconfig[1])
                contactgraph = self.drawContacts(contacts)
                self.env.UpdatePublishedBodies()
                self.env.LockPhysics(False)
            grasp[self.graspindices.get('igrasptrans')] = reshape(transpose(finalconfig[1][0:3,0:4]),12)
            grasp[self.graspindices.get('iforceclosure')] = mindist
            if mindist > forceclosurethreshold:
                print 'found good grasp'
                self.grasps.append(grasp)
        self.grasps = array(self.grasps)

    def runGrasp(self,grasp,graspingnoise=None,translate=True,forceclosure=False):
        self.env.LockPhysics(True)
        oldvalues = self.robot.GetJointValues()
        try:
            self.robot.SetJointValues(grasp[self.graspindices.get('igrasppreshape')],self.manip.GetGripperJoints())
            self.robot.SetActiveDOFs(self.manip.GetGripperJoints(),Robot.DOFAffine.X+Robot.DOFAffine.Y+Robot.DOFAffine.Z if translate else 0)
            return self.grasper.Grasp(direction=grasp[self.graspindices.get('igraspdir')],
                                     roll=grasp[self.graspindices.get('igrasproll')],
                                     position=grasp[self.graspindices.get('igrasppos')],
                                     standoff=grasp[self.graspindices.get('igraspstandoff')],
                                     target=self.target,graspingnoise = graspingnoise,
                                     forceclosure=forceclosure, execute=True, outputfinal=True)
        finally:
            self.robot.SetJointValues(oldvalues)
            self.env.LockPhysics(False)

    def GetBoxApproachRays(self,stepsize=0.02):
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

    def drawContacts(self,contacts,conelength=0.02,transparency=0.5):
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
                R[4] = R[8] = sign(c[5])
            points = dot(conepoints,transpose(R)) + tile(c[0:3],(conepoints.shape[0],1))
            allpoints = r_[allpoints,points[triinds,:]]
        return self.env.drawtrimesh(points=allpoints,indices=None,colors=array((1,0.4,0.4,transparency)))

    @staticmethod
    def _mkdir(newdir):
        """works the way a good mkdir should :)
            - already exists, silently complete
            - regular file in the way, raise an exception
            - parent directory(ies) does not exist, make them as well
        """
        if os.path.isdir(newdir):
            pass
        elif os.path.isfile(newdir):
            raise OSError("a file with the same name as the desired " \
                          "dir, '%s', already exists." % newdir)
        else:
            head, tail = os.path.split(newdir)
            if head and not os.path.isdir(head):
                Grasping._mkdir(head)
            if tail:
                os.mkdir(newdir)

def run():
    env = Environment()
    try:
        env.SetViewer('qtcoin')
        robot = env.ReadRobotXMLFile('robots/barretthand.robot.xml')
        env.AddRobot(robot)
        target = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
        target.SetTransform(eye(4))
        env.AddKinBody(target)
        grasping = Grasping(env,robot,target)
        grasping.initGrasper(friction=0.4,avoidlinks=[])
        grasping.generateGraspSet(preshapes=array(((0.5,0.5,0.5,pi/3),(0.5,0.5,0.5,0),(0,0,0,pi/2))),
                                  rolls = arange(0,2*pi,pi/2),
                                  standoffs = array([0,0.25]),
                                  approachrays = grasping.GetBoxApproachRays(stepsize=0.02),
                                  graspingnoise=None,
                                  addSphereNorms=False)
        grasping.saveGrasps()
    finally:
        env.Destroy()

if __name__ == "__main__":
    run()
