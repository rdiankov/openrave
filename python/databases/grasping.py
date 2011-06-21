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
"""Simulate grasping of objects and computing force closure metrics.

.. image:: ../../images/databases/grasping.jpg
  :width: 640

`[source] <../_modules/openravepy/databases/grasping.html>`_

**Running the Generator**

.. code-block:: bash

  openrave.py --database grasping

_databases_grasping_usage:

Usage
-----

Dynamically generate/load the grasping set for a robot manipulator and target object:

.. code-block:: python

  robot.SetActiveManipulator(...)
  gmodel = openravepy.databases.grasping.GraspingModel(robot,target)
  if not gmodel.load():
      gmodel.autogenerate()


To compute at most 10 valid grasps in the current environment do:

.. code-block:: python

  validgrasps,validindices = gmodel.computeValidGrasps(returnnum=10)
  validgrasp=validgrasps[0] # choose first grasp
  gmodel.showgrasp(validgrasp) # show the grasp
  gmodel.moveToPreshape(validgrasp) # move to the preshape of the first grasp
  Tgrasp = gmodel.getGlobalGraspTransform(validgrasp,collisionfree=True) # get the grasp transform
  basemanip = openravepy.interfaces.BaseManipulation(robot)
  basemanip.MoveToHandPosition(matrices=[Tgrasp]) # move the robot to the grasp

It is also possible to return an iterator to the valid grasps via:

.. code-block:: python

  for validgrasps,validindices in gmodel.validGraspIterator():
      gmodel.showgrasp(validgrasp) # show the grasp

Description
-----------

OpenRAVE can simulate grasps for any type of robotic hand, evaluate the quality of the grasps, and
use those grasps in a more complex grasp planning framework. This tutorial is meant to introduce you
to the ``grasper`` plugin and the scripts provided to manage the testing and simulation. At the
end, you should be able to create grasp tables and use them effectively in OpenRAVE.

A grasp is simulated by giving the end-effector an initial pose and initial joint angles
(preshape). Then the end effector moves along a direction (usually along the normal of the palm)
until it hits the target object. Once hit, the 'fingers' of the end-effector slowly close around the
object until they cannot close further. The contacts between the end-effector and target object are
extracted, and force closure is calculated. The :ref:`module-grasper` is responsible for this
simulation, the scripts just pass in the correct parameters to it.

Grasp set creation first tries to uniformly sample the surface of the object to determine where to
the approach directions should be. Sampling the actual geometric surface of the object can lead to
unwanted results due to possible concavities like the handle of a cup. A simpler approach is to take
the bounding box of the object and sample its surface uniformly (see
`GraspingModel.computeBoxApproachRays`).

.. image:: ../../images/databases/grasping_box_sampling.jpg
  :width: 250

Once the surface of the box is sampled, the intersection of the object and a ray originating from
each point going inward is taken. The normal of the object's surface from each of these intersection
points is taken to be the approaching direction of the end-effector. The red lines in the above
image indicate the rays along which the end-effector will approach the cup.

.. image:: ../../images/databases/grasping_surface_sampling.jpg
  :width: 200

Once the initial pose, preshape, and approach direction are chosen, the grasper planner is called,
which queries the contact points of the grasp and analyzes them for force closure.

.. image:: ../../images/databases/barrett_grasp_strategy.jpg
  :width: 640

Render the final configuration of the end-effector closing down on the target object along with the
friction cones at each contact point (red transparent cones).

.. image:: ../../images/databases/grasping_barrett_mug1.jpg
  :width: 300

.. image:: ../../images/databases/grasping_barrett_mug3.jpg
  :width: 300

Calling `GraspingModel.generate` generates tables for a ketchup bottle.

.. image:: ../../images/databases/grasping_barrett_ketchup1.jpg
  :width: 250

.. image:: ../../images/databases/grasping_barrett_ketchup2.jpg
  :width: 350

Features
========

Here's a short list of features of the grasper planner and problem interfaces:

- the grasper planner takes care of initializing the robot transform given the grasp parameters. the parameters structure for the now contains: stand off, target body, roll, direction, center, and several flags for controlling internal behavior (definition in in grasper/plugindefs.h)

- the grasper problem has two functions: grasp and computedistancemap. grasp just takes care of filling the parameters structure for the planner and returning contact points.

- All grasp parameters like roll, direction, and center offset now specified in target body space. The user *never* has to transform them correspondingly anymore (this was causing many headaches before).

- The grasp coordinate system is defined to be the manipulator's grasp coordinate system (ie, it isn't a link). This allows grasps to define a center of approach. The manipulator definition itself also supports specifying a 'palm direction', which the grasper planner now uses.

- Because the grasper planner reads the gripper links from the manipulator definition, it can now function correctly just by being passed the full robot. Inside the loop, the gripper is separated momentarily to complete the grasping process, the rest of the body is ignored. This allows users to test grasps on a real scene without having to introduce a floating hand into the scene.

Command-line
------------

.. shell-block:: openrave.py --database grasping --help

Class Definitions
-----------------

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov, Atsushi Tsuda'
__copyright__ = 'Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import os,sys,itertools,traceback,time
from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from openravepy.databases import DatabaseGenerator
    from numpy import *
else:
    from openravepy.databases import DatabaseGenerator
    from numpy import inf, array

from optparse import OptionParser

try:
    from itertools import product as iterproduct
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

class GraspingModel(DatabaseGenerator):
    """Holds all functions/data related to a grasp between a robot hand and a target"""

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
                for link in self.robot.GetLinks():
                    if link not in self.manip.GetChildLinks():
                        for geom in link.GetGeometries():
                            self.hiddengeoms.append((geom,geom.IsDraw()))
                            geom.SetDraw(False)
        def __exit__(self,type,value,traceback):
            with self.robot.GetEnv():
                for geom,isdraw in self.hiddengeoms:
                    geom.SetDraw(isdraw)

    def __init__(self,robot,target,maxvelmult=None):
        DatabaseGenerator.__init__(self,robot=robot)
        self.target = target
        self.grasps = []
        self.graspindices = dict()
        self.grasper = None
        self.basemanip = None
        self.maxvelmult=maxvelmult
        self.collision_escape_offset = 0.00009 # used for escaping collision/alignment errors during grasp generation
        self.preprocess()
        # only the indices used by the TaskManipulation plugin should start with an 'i'
        graspdof = {'igraspdir':3,'igrasppos':3,'igrasproll':1,'igraspstandoff':1,'igrasppreshape':len(self.manip.GetGripperIndices()),'igrasptrans':12,'forceclosure':1,'grasptrans_nocol':12}
        self.graspindices = dict()
        self.totaldof = 0
        for name,dof in graspdof.iteritems():
            self.graspindices[name] = range(self.totaldof,self.totaldof+dof)
            self.totaldof += dof
    def clone(self,envother):
        clone = DatabaseGenerator.clone(self,envother)
        clone.basemanip = self.basemanip.clone(envother)
        clone.grasper = self.grasper.clone(envother)
        clone.target = clone.env.GetKinBody(self.target.GetName())
        return clone
    def has(self):
        return len(self.grasps) > 0 and len(self.graspindices) > 0 and self.grasper is not None
    def getversion(self):
        return 4
    def init(self,friction,avoidlinks,plannername=None):
        self.basemanip = interfaces.BaseManipulation(self.robot,maxvelmult=self.maxvelmult)
        self.grasper = interfaces.Grasper(self.robot,friction=friction,avoidlinks=avoidlinks,plannername=plannername)
        self.grasps = []
    def load(self):
        try:
            params = DatabaseGenerator.load(self)
            if params is None:
                return False;
            self.grasps,self.graspindices,friction,linknames,plannername = params
            self.basemanip = interfaces.BaseManipulation(self.robot,maxvelmult=self.maxvelmult)
            self.grasper = interfaces.Grasper(self.robot,friction,avoidlinks = [self.robot.GetLink(name) for name in linknames],plannername=plannername)
            return self.has()
        except:
            return False
    def save(self):
        DatabaseGenerator.save(self,(self.grasps,self.graspindices,self.grasper.friction,[link.GetName() for link in self.grasper.avoidlinks],self.grasper.plannername))
    def getfilename(self,read=False):
        return RaveFindDatabaseFile(os.path.join('robot.'+self.robot.GetKinematicsGeometryHash(), 'graspset.' + self.manip.GetStructureHash() + '.' + self.target.GetKinematicsGeometryHash()+'.pp'),read)

        return DatabaseGenerator.getfilename(self,read,)
    def preprocess(self):
        with self.env:
            self.jointmaxlengths = zeros(len(self.robot.GetJoints()))
            for i,joint in enumerate(self.robot.GetJoints()):
                childlink = None
                if joint.GetFirstAttached() and self.robot.DoesAffect(joint.GetJointIndex(),joint.GetFirstAttached().GetIndex()):
                    childlink = joint.GetFirstAttached()
                elif joint.GetSecondAttached() and self.robot.DoesAffect(joint.GetJointIndex(),joint.GetSecondAttached().GetIndex()):
                    childlink = joint.GetSecondAttached()
                if childlink is not None:
                    # find how much an axis displaces the link
                    if joint.GetType() == KinBody.Joint.Type.Slider or joint.GetType() == KinBody.Joint.Type.Prismatic:
                        self.jointmaxlengths[i] = 1.0
                    else: # revolute
                        T = childlink.GetTransform()
                        T[0:3,3] -= joint.GetAnchor()
                        vertices = transformPoints(T,childlink.GetCollisionData().vertices)
                        # find all child joints and add their anchor to vertices
                        for childjoint in self.robot.GetJoints():
                            if childjoint.GetJointIndex() != joint.GetJointIndex():
                                if (childjoint.GetFirstAttached() and childjoint.GetFirstAttached().GetIndex() == childlink.GetIndex()) or (childjoint.GetSecondAttached() and childjoint.GetSecondAttached().GetIndex() == childlink.GetIndex()):
                                    vertices = r_[vertices,[childjoint.GetAnchor()-joint.GetAnchor()]]
                        self.jointmaxlengths[i] = sqrt(numpy.max(sum(vertices**2,1)-dot(vertices,joint.GetAxis(0))**2)) if len(vertices) > 0 else 0
    def autogenerate(self,options=None):
        # disable every body but the target and robot
        friction = None
        preshapes = None
        approachrays = None
        standoffs = None
        rolls = None
        avoidlinks = None
        graspingnoise = None
        plannername = None
        updateenv=False
        normalanglerange = 0
        directiondelta=0
        if options is not None:
            if options.preshapes is not None:
                preshapes = zeros((0,len(self.manip.GetGripperIndices())))
                for preshape in options.preshapes:
                    preshapes = r_[preshapes,[array([float(s) for s in preshape.split()])]]
            if options.boxdelta is not None:
                approachrays = self.computeBoxApproachRays(delta=options.boxdelta,normalanglerange=options.normalanglerange,directiondelta=options.directiondelta)
            elif options.spheredelta is not None:
                approachrays = self.computeBoxApproachRays(delta=options.spheredelta,normalanglerange=options.normalanglerange,directiondelta=options.directiondelta)
            if options.standoffs is not None:
                standoffs = array(options.standoffs)
            if options.rolls is not None:
                rolls = array(options.rolls)
            if options.friction is not None:
                friction = options.friction
            if options.avoidlinks is not None:
                avoidlinks = [self.robot.GetLink(avoidlink) for avoidlink in options.avoidlinks]
            if options.graspingnoise is not None:
                graspingnoise = options.graspingnoise
            if options.plannername is not None:
                plannername = options.plannername
            if options.normalanglerange is not None:
                normalanglerange = options.normalanglerange
            if options.directiondelta is not None:
                directiondelta = options.directiondelta
            updateenv = True#options.useviewer
        # check for specific robots
        if self.robot.GetRobotStructureHash() == 'dc6974d07d6adf9db8b790f49e304ffa' or self.robot.GetRobotStructureHash() == 'd51f2af3be3f82ae9a9393af36e72bcd': # wam+barretthand
            if preshapes is None:
                preshapes=array(((0.5,0.5,0.5,pi/3),(0.5,0.5,0.5,0),(0,0,0,pi/2)))
            if graspingnoise is None:
                graspingnoise = 0.01 # 0.01m of noise
        elif self.robot.GetRobotStructureHash() == 'a43d21a7b6d60d7443922d012bd3b43e': # pa10
            if graspingnoise is None:
                graspingnoise = 0.01 # 0.01m of noise
        if avoidlinks is None:
            avoidlinks = []
        if friction is None:
            friction = 0.4
        if approachrays is None:
            approachrays = self.computeBoxApproachRays(delta=0.02,normalanglerange=normalanglerange,directiondelta=directiondelta)
        self.init(friction=friction,avoidlinks=avoidlinks,plannername=plannername)
        self.generate(preshapes=preshapes,rolls=rolls,graspingnoise=graspingnoise,standoffs=standoffs,approachrays=approachrays,updateenv=updateenv)
        self.save()

    def generate(self,preshapes=None,standoffs=None,rolls=None,approachrays=None, graspingnoise=None,updateenv=True,forceclosure=True,forceclosurethreshold=1e-9,checkgraspfn=None,disableallbodies=True):
        """Generates a grasp set by searching space and evaluating contact points.

        All grasp parameters have to be in the bodies's coordinate system (ie: approachrays).
        @param checkgraspfn: If set, then will be used to validate the grasp. If its evaluation returns false, then grasp will not be added to set. Called by checkgraspfn(contacts,finalconfig,grasp,info)"""
        print 'Generating Grasp Set for %s:%s:%s'%(self.robot.GetName(),self.manip.GetName(),self.target.GetName())
        if approachrays is None:
            approachrays = self.computeBoxApproachRays(delta=0.02,normalanglerange=0)
        if preshapes is None:
            # should disable everything but the robot
            with self.target:
                self.target.Enable(False)
                # do not fill with plannername
                taskmanip = interfaces.TaskManipulation(self.robot)
                final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
            preshapes = array([final])
        if rolls is None:
            rolls = arange(0,2*pi,pi/2)
        if standoffs is None:
            standoffs = array([0,0.025])
        if graspingnoise is None:
            graspingnoise = 0.0
        time.sleep(0.1) # sleep or otherwise viewer might not load well
        N = approachrays.shape[0]
        with self.env:
            Ttarget = self.target.GetTransform()
            Trobotorig = self.robot.GetTransform()
        # transform each ray into the global coordinate system in order to plot it
        gapproachrays = c_[dot(approachrays[:,0:3],transpose(Ttarget[0:3,0:3]))+tile(Ttarget[0:3,3],(N,1)),dot(approachrays[:,3:6],transpose(Ttarget[0:3,0:3]))]
        approachgraphs = [self.env.plot3(points=gapproachrays[:,0:3],pointsize=5,colors=array((1,0,0))),
                          self.env.drawlinelist(points=reshape(c_[gapproachrays[:,0:3],gapproachrays[:,0:3]+0.005*gapproachrays[:,3:6]],(2*N,3)),linewidth=4,colors=array((1,0,0,1)))]
        contactgraph = None
        totalgrasps = N*len(preshapes)*len(rolls)*len(standoffs)
        counter = 0
        self.grasps = []
        statesaver = self.robot.CreateRobotStateSaver()
        bodies = [(b,b.IsEnabled()) for b in self.env.GetBodies() if b != self.robot and b != self.target]
        if disableallbodies:
            for b in bodies:
                b[0].Enable(False)
        try:
            with self.GripperVisibility(self.manip):
                if updateenv:
                    self.env.UpdatePublishedBodies()
                for approachray in approachrays:
                    for roll,preshape,standoff in iterproduct(rolls,preshapes,standoffs):
                        print 'grasp %d/%d'%(counter,totalgrasps),'preshape:',preshape
                        counter += 1
                        grasp = zeros(self.totaldof)
                        grasp[self.graspindices.get('igrasppos')] = approachray[0:3]
                        grasp[self.graspindices.get('igraspdir')] = -approachray[3:6]
                        grasp[self.graspindices.get('igrasproll')] = roll
                        grasp[self.graspindices.get('igraspstandoff')] = standoff
                        grasp[self.graspindices.get('igrasppreshape')] = preshape
                        try:
                            contacts,finalconfig,mindist,volume = self.testGrasp(grasp=grasp,graspingnoise=graspingnoise,translate=True,forceclosure=forceclosure,forceclosurethreshold=forceclosurethreshold)
                        except planning_error, e:
                            print 'Grasp Failed: '
                            traceback.print_exc(e)
                            continue
                        Tlocalgrasp = eye(4)
                        with self.robot:
                            self.robot.SetTransform(finalconfig[1])
                            Tgrasp = self.manip.GetEndEffectorTransform()
                            Tlocalgrasp = dot(linalg.inv(self.target.GetTransform()),Tgrasp)
                            # find a non-colliding transform
                            self.setPreshape(grasp)
                            direction = self.getGlobalApproachDir(grasp)
                            Tgrasp_nocol = array(Tgrasp)
                            while self.manip.CheckEndEffectorCollision(Tgrasp_nocol):
                                Tgrasp_nocol[0:3,3] -= direction*self.collision_escape_offset
                            Tlocalgrasp_nocol = dot(linalg.inv(self.target.GetTransform()),Tgrasp_nocol)
                            self.robot.SetDOFValues(finalconfig[0])
                            if updateenv:
                                contactgraph = self.drawContacts(contacts) if len(contacts) > 0 else None
                                self.env.UpdatePublishedBodies()
                            grasp[self.graspindices.get('igrasptrans')] = reshape(transpose(Tlocalgrasp[0:3,0:4]),12)
                            grasp[self.graspindices.get('grasptrans_nocol')] = reshape(transpose(Tlocalgrasp_nocol[0:3,0:4]),12)
                            grasp[self.graspindices.get('forceclosure')] = mindist if mindist is not None else 0
                            self.robot.SetTransform(Trobotorig) # transform back to original position for checkgraspfn
                            if not forceclosure or mindist >= forceclosurethreshold:
                                if checkgraspfn is None or checkgraspfn(contacts,finalconfig,grasp,{'mindist':mindist,'volume':volume}):
                                    print 'found good grasp',len(self.grasps),'config: ',array(finalconfig[0])[self.manip.GetGripperIndices()]
                                    self.grasps.append(grasp)
                self.grasps = array(self.grasps)
                print 'ordering grasps'
                self.orderGrasps()
        finally:
            for b,enable in bodies:
                b.Enable(enable)
            # force closing the handles (if an exception is thrown, python 2.6 does not close them without a finally)
            approachgraphs = None
            contactgraph = None
            statesaver = None

    def generateThreaded(self,preshapes=None,standoffs=None,rolls=None,approachrays=None,graspingnoise=None,forceclosurethreshold=1e-9,numthreads=None,checkgraspfn=None,disableallbodies=True,translate=True):
        """Generates a grasp set by searching space and evaluating contact points.

        All grasp parameters have to be in the bodies's coordinate system (ie: approachrays).
        @param checkgraspfn: If set, then will be used to validate the grasp. If its evaluation returns false, then grasp will not be added to set. Called by checkgraspfn(contacts,finalconfig,grasp,info)"""
        print 'Generating Grasp Set for %s:%s:%s'%(self.robot.GetName(),self.manip.GetName(),self.target.GetName())
        if approachrays is None:
            approachrays = self.computeBoxApproachRays(delta=0.02,normalanglerange=0)
        if preshapes is None:
            # should disable everything but the robot
            with self.target:
                self.target.Enable(False)
                # do not fill with plannername
                taskmanip = interfaces.TaskManipulation(self.robot)
                final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
            preshapes = array([final])
        if rolls is None:
            rolls = arange(0,2*pi,pi/2)
        if standoffs is None:
            standoffs = array([0,0.025])
        if numthreads is None:
            numthreads = 2

        try:
            with self.robot: # lock the environment and save the robot state
                bodies = [(b,b.IsEnabled()) for b in self.env.GetBodies() if b != self.robot and b != self.target]
                if disableallbodies:
                    for b in bodies:
                        b[0].Enable(False)

                Ttarget = self.target.GetTransform()
                Trobotorig = self.robot.GetTransform()
                self.robot.SetActiveManipulator(self.manip)
                self.robot.SetTransform(eye(4)) # have to reset transform in order to remove randomness
                self.robot.SetActiveDOFs(self.manip.GetGripperIndices(),Robot.DOFAffine.X+Robot.DOFAffine.Y+Robot.DOFAffine.Z if translate else 0)
                approachrays[:,3:6] = -approachrays[:,3:6]
                self.nextid, self.resultgrasps = self.grasper.GraspThreaded(approachrays=approachrays, rolls=rolls, standoffs=standoffs, preshapes=preshapes, target=self.target, graspingnoise=graspingnoise, forceclosurethreshold=forceclosurethreshold,numthreads=numthreads)
                print 'graspthreaded done, processing grasps'

                for resultgrasp in self.resultgrasps:
                    grasp = zeros(self.totaldof)
                    grasp[self.graspindices.get('igrasppos')] = resultgrasp[0]
                    grasp[self.graspindices.get('igraspdir')] = resultgrasp[1]
                    grasp[self.graspindices.get('igrasproll')] = resultgrasp[2]
                    grasp[self.graspindices.get('igraspstandoff')] = resultgrasp[3]
                    mindist = resultgrasp[4]
                    volume = resultgrasp[5]
                    grasp[self.graspindices.get('igrasppreshape')] = resultgrasp[6]
                    Tfinal = resultgrasp[7]
                    finalshape = resultgrasp[8]
                    contacts = resultgrasp[9]

                    with self.robot:
                        Tlocalgrasp = eye(4)
                        self.robot.SetTransform(Tfinal)
                        Tgrasp = self.manip.GetEndEffectorTransform()
                        Tlocalgrasp = dot(linalg.inv(self.target.GetTransform()),Tgrasp)
                        # find a non-colliding transform
                        direction = self.getGlobalApproachDir(grasp)
                        Tgrasp_nocol = array(Tgrasp)
                        while self.manip.CheckEndEffectorCollision(Tgrasp_nocol):
                            Tgrasp_nocol[0:3,3] -= direction*self.collision_escape_offset
                        Tlocalgrasp_nocol = dot(linalg.inv(self.target.GetTransform()),Tgrasp_nocol)
                        self.robot.SetDOFValues(finalshape)

                        grasp[self.graspindices.get('igrasptrans')] = reshape(transpose(Tlocalgrasp[0:3,0:4]),12)
                        grasp[self.graspindices.get('grasptrans_nocol')] = reshape(transpose(Tlocalgrasp_nocol[0:3,0:4]),12)
                        grasp[self.graspindices.get('forceclosure')] = mindist if mindist is not None else 0
                        if not forceclosurethreshold or mindist >= forceclosurethreshold:
                            if checkgraspfn is None or checkgraspfn(contacts,[Tfinal,finalshape],grasp,{'mindist':mindist,'volume':volume}):
                                self.grasps.append(grasp)

                self.grasps = array(self.grasps)
                print 'ordering grasps'
                self.orderGrasps()
        finally:
            for b,enable in bodies:
                b.Enable(enable)

    def show(self,delay=0.1,options=None,forceclosure=True):
        with RobotStateSaver(self.robot):
            with self.GripperVisibility(self.manip):
                time.sleep(1.0) # let viewer update?
                graspingnoise=None
                if options is not None:
                    if options.graspindex is not None:
                        print 'showing grasp %d'%options.graspindex
                        grasps = [self.grasps[options.graspindex]]
                        delay=10000000
                    else:
                        #self.orderGrasps()
                        #self.save()
                        grasps = self.grasps
                    graspingnoise=options.graspingnoise
                else:
                    grasps = self.grasps
                for i,grasp in enumerate(grasps):
                    print 'grasp %d/%d'%(i,len(grasps))
                    try:
                        with self.env:
                            #self.env.SetDebugLevel(DebugLevel.Verbose)
                            contacts,finalconfig,mindist,volume = self.testGrasp(grasp=grasp,translate=True,forceclosure=forceclosure,graspingnoise=graspingnoise)
                            #contacts,finalconfig,mindist,volume = self.runGrasp(grasp=grasp,translate=True,forceclosure=True)
                            if mindist == 0:
                                print 'grasp is not in force closure!'
                            contactgraph = self.drawContacts(contacts) if len(contacts) > 0 else None
                            self.robot.GetController().Reset(0)
                            self.robot.SetDOFValues(finalconfig[0])
                            self.robot.SetTransform(finalconfig[1])
                            self.env.UpdatePublishedBodies()
                        if delay is None:
                            raw_input('press any key to continue: ')
                        elif delay > 0:
                            time.sleep(delay)
                    except planning_error,e:
                        print 'bad grasp!',e
    def showgrasp(self,grasp,collisionfree=False,useik=False,delay=None):
        with RobotStateSaver(self.robot):
            with self.GripperVisibility(self.manip):
                with self.env:
                    self.setPreshape(grasp)
                    Tgrasp = self.getGlobalGraspTransform(grasp,collisionfree=collisionfree)
                    if useik and collisionfree:
                        sol = self.manip.FindIKSolution(Tgrasp,IkFilterOptions.CheckEnvCollisions)
                        self.robot.SetDOFValues(sol,self.manip.GetArmIndices())
                    else:
                        Tdelta = dot(Tgrasp,linalg.inv(self.manip.GetEndEffectorTransform()))
                        for link in self.manip.GetChildLinks():
                            link.SetTransform(dot(Tdelta,link.GetTransform()))
                    self.env.UpdatePublishedBodies()
                    # wait while environment is locked?
                    if delay is None:
                        raw_input('press any key to continue: ')
                    elif delay > 0:
                        time.sleep(delay)
    def testGrasp(self,graspingnoise=None,Ngraspingtries = 20,forceclosurethreshold=1e-9,**kwargs):
        contacts,finalconfig,mindist,volume = self.runGrasp(graspingnoise=0,**kwargs)
        if mindist >= forceclosurethreshold and graspingnoise > 0:
            print 'testing with noise',graspingnoise
            # try several times and make sure that grasp succeeds all the time
            allfinaltrans = [finalconfig[1]]
            allfinaljoints = [finalconfig[0]]
            for iter in range(Ngraspingtries):
                contacts2,finalconfig2,mindist2,volume2 = self.runGrasp(graspingnoise=graspingnoise,**kwargs)
                # don't check mindist since slight variations can really affect force closure
#                 with self.robot:
#                     self.robot.SetTransform(finalconfig2[1])
#                     self.robot.SetDOFValues(finalconfig2[0])
#                     self.env.UpdatePublishedBodies()
#                     time.sleep(1)
                allfinaltrans.append(finalconfig2[1])
                allfinaljoints.append(finalconfig2[0])
            if mindist > 0:
                # deinty grasps will usually change the configuration greatly, so detect such changes
                # get the std deviation of all transformations and joints and make sure it is small
                translationstd = mean(std([T[0:3,3] for T in allfinaltrans],0))
                jointvaluesstd = self.jointmaxlengths*std(array(allfinaljoints),0)
                # compute the max distance of each link
                maxjointvaluestd = max([sum([jointvaluesstd[i] for i in range(self.robot.GetDOF()) if self.robot.DoesAffect(i,link.GetIndex())]) for link in self.robot.GetLinks()])
                print 'grasp:',translationstd,maxjointvaluestd
                if translationstd+maxjointvaluestd > 0.7*graspingnoise:
                    print 'deinty grasp:',translationstd,maxjointvaluestd
                    mindist = 0
        return contacts,finalconfig,mindist,volume
    def runGrasp(self,grasp,graspingnoise=None,translate=True,forceclosure=False):
        with self.robot: # lock the environment and save the robot state
            self.robot.SetActiveManipulator(self.manip)
            self.robot.SetTransform(eye(4)) # have to reset transform in order to remove randomness
            self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')],self.manip.GetGripperIndices())
            self.robot.SetActiveDOFs(self.manip.GetGripperIndices(),Robot.DOFAffine.X+Robot.DOFAffine.Y+Robot.DOFAffine.Z if translate else 0)
            return self.grasper.Grasp(direction=grasp[self.graspindices.get('igraspdir')],
                                      roll=grasp[self.graspindices.get('igrasproll')],
                                      position=grasp[self.graspindices.get('igrasppos')],
                                      standoff=grasp[self.graspindices.get('igraspstandoff')],
                                      target=self.target,graspingnoise = graspingnoise,
                                      forceclosure=forceclosure, execute=False, outputfinal=True)
    def runGraspFromTrans(self,grasp):
        """squeeze the fingers to test whether the completed grasp only collides with the target, throws an exception if it fails. Otherwise returns the Grasp parameters. Uses the grasp transformation directly."""
        with self.robot:
            self.robot.SetActiveManipulator(self.manip)
            self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')],self.manip.GetGripperIndices())
            self.robot.SetTransform(dot(self.getGlobalGraspTransform(grasp),dot(linalg.inv(self.manip.GetEndEffectorTransform()),self.robot.GetTransform())))
            self.robot.SetActiveDOFs(self.manip.GetGripperIndices())
            return self.grasper.Grasp(transformrobot=False,target=self.target,onlycontacttarget=True, forceclosure=False, execute=False, outputfinal=True)

    def getGlobalGraspTransform(self,grasp,collisionfree=False):
        """returns the final grasp transform before fingers start closing. If collisionfree is set to True, then will return a grasp that is guaranteed to be not in collision with the target object when at its preshape. This is achieved by by moving the hand back along igraspdir."""
        Tlocalgrasp = eye(4)
        Tlocalgrasp[0:3,0:4] = transpose(reshape(grasp[self.graspindices ['grasptrans_nocol' if collisionfree else 'igrasptrans']],(4,3)))
        return dot(self.target.GetTransform(),Tlocalgrasp)
    def getGlobalApproachDir(self,grasp):
        """returns the global approach direction"""
        return dot(self.target.GetTransform()[0:3,0:3],grasp[self.graspindices.get('igraspdir')])
    def setPreshape(self,grasp):
        """sets the preshape on the robot, assumes environment is locked"""
        self.robot.SetDOFValues(grasp[self.graspindices['igrasppreshape']],self.manip.GetGripperIndices())
    def getPreshape(self,grasp):
        """returns the preshape joint values"""
        return grasp[self.graspindices['igrasppreshape']]
    def moveToPreshape(self,grasp,execute=True,outputtraj=False):
        """uses a planner to safely move the hand to the preshape and returns the trajectory"""
        trajdata = []
        with self.robot:
            self.robot.SetActiveDOFs(self.manip.GetArmIndices())
            trajdata.append(self.basemanip.MoveUnsyncJoints(jointvalues=grasp[self.graspindices['igrasppreshape']],jointinds=self.manip.GetGripperIndices(),execute=execute,outputtraj=outputtraj))
        while execute and not self.robot.GetController().IsDone(): # busy wait
            time.sleep(0.01)

        with self.robot:
            if not execute and outputtraj:
                # set the final point!
                s = trajdata[-1].split()
                numpoints, numjoints, options = int(s[0]),int(s[1]),int(s[2])
                if numpoints > 0:
                    dof = numjoints
                    lastpointindex = 3
                    if options & 4:
                        dof += 1
                        lastpointindex += 1
                    if options & 8:
                        dof += 7
                    if options & 16:
                        dof += numjoints
                    if options & 32:
                        dof += numjoints
                    lastpointindex+=(numpoints-1)*dof
                    self.robot.SetDOFValues(array([float(f) for f in s[lastpointindex:(lastpointindex+numjoints)]]))
            self.robot.SetActiveDOFs(self.manip.GetGripperIndices())
            trajdata.append(self.basemanip.MoveActiveJoints(goal=grasp[self.graspindices['igrasppreshape']],execute=execute,outputtraj=outputtraj))
        while execute and not self.robot.GetController().IsDone(): # busy wait
            time.sleep(0.01)
        return trajdata
    def computeValidGrasps(self,startindex=0,checkcollision=True,checkik=True,checkgrasper=True,backupdist=0.0,returnnum=inf):
        """Returns the set of grasps that satisfy conditions like collision-free and reachable.

        :param returnnum: If set, will also return once that many number of grasps are found.
        :param backupdist: If > 0, then will move the hand along negative approach direction and check for validity.
        :param checkgrasper: If True, will execute the grasp and check if gripper only contacts target.
        :param startindex: The index to start searching for grasps
        :param checkik: If True will check that the grasp is reachable by the arm.
        :param checkcollision: If true will return only collision-free grasps. If checkik is also True, will return grasps that have collision-free arm solutions.
        """
        with self.robot:
            validgrasps = []
            validindices = []
            self.robot.SetActiveManipulator(self.manip)
            report = CollisionReport()
            for i in range(startindex,len(self.grasps)):
                grasp = self.grasps[i]
                self.setPreshape(grasp)
                Tglobalgrasp = self.getGlobalGraspTransform(grasp,collisionfree=True)
                if checkik:
                    if self.manip.FindIKSolution(Tglobalgrasp,checkcollision) is None:
                        continue
                elif checkcollision:
                    if self.manip.CheckEndEffectorCollision(Tglobalgrasp):
                        continue
                if backupdist > 0:
                    Tnewgrasp = array(Tglobalgrasp)
                    Tnewgrasp[0:3,3] -= backupdist * self.getGlobalApproachDir(grasp)
                    if checkik:
                        if self.manip.FindIKSolution(Tnewgrasp,checkcollision) is None:
                            continue
                    elif checkcollision:
                        if self.manip.CheckEndEffectorCollision(Tnewgrasp):
                            continue
                if checkcollision and checkgrasper:
                    try:
                        self.runGraspFromTrans(grasp)
                    except planning_error, e:
                        continue
                validgrasps.append(grasp)
                validindices.append(i)
                if len(validgrasps) == returnnum:
                    return validgrasps,validindices
            return validgrasps,validindices

    def validGraspIterator(self,startindex=0,checkcollision=True,checkik=True,checkgrasper=True,backupdist=0.0,randomgrasps=False):
        """Returns an iterator for valid grasps that satisfy certain conditions.

        See :meth:`computeValidGrasps` for description of parameters.
        """
        if randomgrasps:
            order = startindex+random.permutation(len(self.grasps)-startindex)
        else:
            order = range(startindex,len(self.grasps))
        for i in order:
            grasp = self.grasps[i]
            with KinBodyStateSaver(self.robot):
                self.setPreshape(grasp)
                Tglobalgrasp = self.getGlobalGraspTransform(grasp,collisionfree=True)
                if checkik:
                    if self.manip.FindIKSolution(Tglobalgrasp,checkcollision) is None:
                        continue
                elif checkcollision:
                    if self.manip.CheckEndEffectorCollision(Tglobalgrasp):
                        continue
                if backupdist > 0:
                    Tnewgrasp = array(Tglobalgrasp)
                    Tnewgrasp[0:3,3] -= backupdist * self.getGlobalApproachDir(grasp)
                    if checkik:
                        if self.manip.FindIKSolution(Tnewgrasp,checkcollision) is None:
                            continue
                    elif checkcollision:
                        if self.manip.CheckEndEffectorCollision(Tnewgrasp):
                            continue
                if checkcollision and checkgrasper:
                    try:
                        self.runGraspFromTrans(grasp)
                    except planning_error, e:
                        continue
            yield grasp,i
    def orderGrasps(self):
        """order the grasps by the closest contact to the center of object."""
        with self.target:
            contactdists = []
            self.target.SetTransform(eye(4))
            ab=self.target.ComputeAABB()
            for grasp in self.grasps:
                try:
                    contacts,finalconfig,mindist,volume = self.runGrasp(grasp=grasp,translate=True,forceclosure=False)
                except planning_error, e:
                    print 'grasp failed: ',e
                    contacts = []
                # find closest contact to center of object
                if len(contacts) > 0: # sometimes we get no contacts?!
                    contactdists.append(numpy.min(sum((contacts[:,0:3]-tile(ab.pos(),(len(contacts),1)))**2,1)))
                else:
                    contactdists.append(inf)
            order = argsort(array(contactdists))
            self.grasps = self.grasps[order]

    def computePlaneApproachRays(self,center,sidex,sidey,delta=0.02,normalanglerange=0,directiondelta=0.4):
        # ode gives the most accurate rays
        cc = RaveCreateCollisionChecker(self.env,'ode')
        if cc is not None:
            ccold = self.env.GetCollisionChecker()
            self.env.SetCollisionChecker(cc)
            cc = ccold
        try:
            with self.env:
                ex = sqrt(sum(sidex**2))
                ey = sqrt(sum(sidey**2))
                normal=cross(sidex,sidey)
                normal /= sqrt(sum(normal**2))
                XX,YY = meshgrid(r_[arange(-ex,-0.25*delta,delta),0,arange(delta,ex,delta)],
                                 r_[arange(-ey,-0.25*delta,delta),0,arange(delta,ey,delta)])
                localpos = outer(XX.flatten(),sidex/ex)+outer(YY.flatten(),sidey/ey)
                N = localpos.shape[0]
                rays = c_[tile(center,(N,1))+localpos,100.0*tile(normal,(N,1))]
                collision, info = self.env.CheckCollisionRays(rays,self.target)
                # make sure all normals are the correct sign: pointing outward from the object)
                newinfo = info[collision,:]
                approachrays = zeros((0,6))
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
                    approachrays=newapproachrays
            return approachrays
        finally:
            # restore the collision checker
            if cc is not None:
                self.env.SetCollisionChecker(cc)

    def computeBoxApproachRays(self,delta=0.02,normalanglerange=0,directiondelta=0.4):
        # ode gives the most accurate rays
        cc = RaveCreateCollisionChecker(self.env,'ode')
        if cc is not None:
            ccold = self.env.GetCollisionChecker()
            self.env.SetCollisionChecker(cc)
            cc = ccold
        try:
            with self.target:
                self.target.SetTransform(eye(4))
                ab = self.target.ComputeAABB()
                p = ab.pos()
                e = ab.extents()+0.01 # increase since origin of ray should be outside of object
                sides = array(((0,0,e[2],0,0,-1,e[0],0,0,0,e[1],0),
                               (0,0,-e[2],0,0,1,e[0],0,0,0,e[1],0),
                               (0,e[1],0,0,-1,0,e[0],0,0,0,0,e[2]),
                               (0,-e[1],0,0,1,0,e[0],0,0,0,0,e[2]),
                               (e[0],0,0,-1,0,0,0,e[1],0,0,0,e[2]),
                               (-e[0],0,0,1,0,0,0,e[1],0,0,0,e[2])))
                maxlen = 2*sqrt(sum(e**2))+0.03
                approachrays = zeros((0,6))
                for side in sides:
                    ex = sqrt(sum(side[6:9]**2))
                    ey = sqrt(sum(side[9:12]**2))
                    if ex/delta > 1000:
                        raise ValueError('object is way too big for its discretization! %f > 1000'%(ex/delta))
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
        finally:
            # restore the collision checker
            if cc is not None:
                self.env.SetCollisionChecker(cc)
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
        parser = DatabaseGenerator.CreateOptionParser()
        parser.description='Grasp set generation example for any robot/body pair.'
        parser.usage='openrave.py --database grasping [options]'
        parser.add_option('--plannername',action="store",type='string',dest='plannername',default=None,
                          help='The grasper planner to use for this model (default=%default)')
        parser.add_option('--target',action="store",type='string',dest='target',default='data/mug1.kinbody.xml',
                          help='The filename of the target body whose grasp set to be generated (default=%default)')
#         parser.add_option('--noviewer', action='store_false', dest='useviewer',default=True,
#                           help='If specified, will generate the tables without launching a viewer')
        parser.add_option('--boxdelta', action='store', type='float',dest='boxdelta',default=None,
                          help='Step size of of box surface sampling')
        parser.add_option('--spheredelta', action='store', type='float',dest='spheredelta',default=None,
                          help='Delta angle between directions on the sphere')
        parser.add_option('--normalanglerange', action='store', type='float',dest='normalanglerange',default=0.0,
                          help='The range of angles around the surface normal to approach from (default=%default)')
        parser.add_option('--directiondelta', action='store', type='float',dest='directiondelta',default=0.4,
                          help='The average distance of approach directions for each surface point in radians (default=%default)')
        parser.add_option('--standoff', action='append', type='float',dest='standoffs',default=None,
                          help='Add a standoff distance')
        parser.add_option('--roll', action='append', type='float',dest='rolls',default=None,
                          help='Add a roll angle')
        parser.add_option('--preshape', action='append', type='string',dest='preshapes',default=None,
                          help='Add a preshape for the manipulator gripper joints')
        parser.add_option('--avoidlink', action='append', type='string',dest='avoidlinks',default=None,
                          help='Add a link name to avoid at all costs (like sensor links)')
        parser.add_option('--friction', action='store', type='float',dest='friction',default=None,
                          help='Friction between robot and target object (default=0.4)')
        parser.add_option('--graspingnoise', action='store', type='float',dest='graspingnoise',default=None,
                          help='Random undeterministic noise to add to the target object, represents the max possible displacement of any point on the object. Noise is added after global direction and start have been determined (default=0)')
        parser.add_option('--graspindex', action='store', type='int',dest='graspindex',default=None,
                          help='If set, then will only show this grasp index')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None,args=None,**kwargs):
        if parser is None:
            parser = GraspingModel.CreateOptionParser()
        (options, leftargs) = parser.parse_args(args=args)
        env = Environment()
        try:
            target = None
            with env:
                target = env.ReadKinBodyXMLFile(options.target)
                target.SetTransform(eye(4))
                env.AddKinBody(target)
            if Model is None:
                Model = lambda robot: GraspingModel(robot=robot,target=target)
#             if options.useviewer:
#                 env.SetViewer('qtcoin')
#                 env.UpdatePublishedBodies()
            DatabaseGenerator.RunFromParser(env=env,Model=Model,parser=parser,args=args,defaultviewer=True,**kwargs)
        finally:
            env.Destroy()
            RaveDestroy()

def run(*args,**kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    GraspingModel.RunFromParser(*args,**kwargs)

if __name__ == "__main__":
    run()
