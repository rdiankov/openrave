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
# random code that helps with debugging/testing the python interfaces and examples
# this is not meant to be run by normal users
from __future__ import with_statement # for python 2.5
__copyright__ = 'Copyright (C) 2009-2010'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
from itertools import izip

def RetractionConstraint(self,prev,cur,thresh=1e-4):
    """jacobian gradient descent"""
    new = array(cur)
    robot.SetActiveDOFValues(prev)
    distprev = sum((prev-cur)**2)
    distcur = 0
    lasterror = 0
    for iter in range(10):
        Jrotation = robot.CalculateActiveAngularVelocityJacobian(manip.GetEndEffector().GetIndex())
        Jtranslation = robot.CalculateActiveJacobian(manip.GetEndEffector().GetIndex(),manip.GetEndEffectorTransform()[0:3,3])
        J = r_[Jrotation,Jtranslation]
        JJt = dot(J,transpose(J))+eye(6)*1e-8
        invJ = dot(transpose(J),linalg.inv(JJt))
        Terror = dot(dot(targetframematrix,linalg.inv(Tee)),manip.GetEndEffectorTransform())
        poseerror = poseFromMatrix(Terror)
        error = r_[Terror[0:3,3],[2.0*arctan2(poseerror[i],poseerror[0]) for i in range(1,4)]]
        error *= array([1,0,0,1,1,1])
        print 'error: ', sum(error**2)
        if sum(error**2) < thresh**2:
            return True
        if sum(error**2) > lasterror and distcur > distprev:
            return False;
        qdelta = dot(invJ,error)
        vnew -= qdelta
        robot.SetActiveDOFValues(vnew)
        distcur = sum((vnew-vcur)**2)
        print manip.GetEndEffectorTransform()

def test_jacobianconstraints():
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
    env.AddRobot(robot)
    manip = robot.GetActiveManipulator()
    taskmanip = TaskManipulation(robot)
    robot.SetActiveDOFs(manip.GetArmJoints())
    vorg = robot.GetActiveDOFValues()
    
    freedoms = [1,1,1,1,1,1]
    v = array(vorg)
    v[1] += 0.2
    configs = [v]
    targetframematrix = eye(4)
    env.SetDebugLevel(DebugLevel.Debug)

    robot.SetActiveDOFValues(vorg)
    Tee = manip.GetEndEffectorTransform()
    print manip.GetEndEffectorTransform()
    errorthresh = 1e-3
    iters,newconfigs = taskmanip.EvaluateConstraints(freedoms=freedoms,targetframematrix=targetframematrix,configs=configs,errorthresh=errorthresh)
    print 'iters: ',iters
    robot.SetActiveDOFValues(configs[0])
    print 'old: ', manip.GetEndEffectorTransform()
    robot.SetActiveDOFValues(newconfigs[0])
    print 'new: ', manip.GetEndEffectorTransform()

def test_jacobian():
    """tests if jacobians work"""
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')#barretthand.robot.xml')
    env.AddRobot(robot)
    fingertip_global = array([-0.15,0,0.2])

    with env:
        # get a link name
        link = robot.GetLink('Finger2-2')
        fingertip_local = dot(linalg.inv(link.GetTransform()),r_[fingertip_global,1])[0:3]
        J = robot.CalculateJacobian(link.GetIndex(),fingertip_global)
        # move each joint a little
        curvalues = robot.GetDOFValues()
        for iter in range(10000):
            robot.SetJointValues(curvalues + (random.randint(0,2,len(curvalues))-0.5)*0.03)
            fingertip_realdir = dot(link.GetTransform(),r_[fingertip_local,1])[0:3] - fingertip_global
            deltavalues = robot.GetDOFValues()-curvalues
            fingertip_estimatedir = dot(J,deltavalues)
            dreal = fingertip_realdir/sqrt(sum(fingertip_realdir**2))
            destimate = fingertip_estimatedir/sqrt(sum(fingertip_estimatedir**2))
            if dot(dreal,destimate) < 0.98:
                print deltavalues
                raise ValueError('bad jacobian: %f'%dot(dreal,destimate))

def test_hash():
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    s = robot.serialize(SerializationOptions.Kinematics)
    hash = robot.GetKinematicsGeometryHash()
    print hash

def test_convexdecomp():
    env = Environment()
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    link = robot.GetLinks()[0]
    geom = link.GetGeometries()[0]
    trimesh = geom.GetCollisionMesh()
    hulls = convexdecompositionpy.computeConvexDecomposition(trimesh.vertices,trimesh.indices)

def generate_box(pos,extents):
    indices = reshape([0, 1, 2, 1, 2, 3, 4, 5, 6, 5, 6, 7, 0, 1, 4, 1, 4, 5, 2, 3, 6, 3, 6, 7, 0, 2, 4, 2, 4, 6, 1, 3, 5,3, 5, 7],(12,3))
    vertices = array(((extents[0],extents[1],extents[2]),
                      (extents[0],extents[1],-extents[2]),
                      (extents[0],-extents[1],extents[2]),
                      (extents[0],-extents[1],-extents[2]),
                      (-extents[0],extents[1],extents[2]),
                      (-extents[0],extents[1],-extents[2]),
                      (-extents[0],-extents[1],extents[2]),
                      (-extents[0],-extents[1],-extents[2])))
    return vertices+tile(pos,(8,1)),indices

def test_geometrychange():
    """changed geometry and tests if changed are updated"""
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    link = robot.GetLinks()[-1]
    geom = link.GetGeometries()[0]    
    geom.SetCollisionMesh(KinBody.Link.TriMesh(*generate_box([0,0,0.1],[1,1,0.1])))
    env.Destroy()

def test_selfcollision():
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('data/lab1.env.xml')
    target1 = env.GetKinBody('mug1')
    target2 = env.GetKinBody('mug2')
    env.SetDebugLevel(DebugLevel.Verbose)
    with env:
        target2.SetTransform(target1.GetTransform())
        env.CheckCollision(target1)
        robot = env.GetRobots()[0]
        robot.Grab(target1)
        report = CollisionReport()
        robot.CheckSelfCollision(report)
        target1.CheckSelfCollision()
        env.CheckCollision(target1,report)

def test_fkconsistency():
    # tests if fk is consistent
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/schunk-lwa3-dual.robot.xml')
    env.AddRobot(robot)
    lower,upper = robot.GetJointLimits()
    with robot:
        T = eye(4)
        T[0:3,0:3] = rotationMatrixFromAxisAngle(random.rand(3))
        T[0:3,3] = random.rand(3)-0.5
        robot.SetTransform(T)
        while True:
            values = lower*random.rand(len(lower))+(upper-lower)
            robot.SetJointValues(values)
            Tlinks1 = robot.GetBodyTransformations()[:]
            for iter2 in range(100):
                robot.SetJointValues(lower+random.rand(len(lower))*(upper-lower))
            robot.SetJointValues(values)
            Tlinks2 = robot.GetBodyTransformations()[:]
            # Tlinks1 and Tlinks2 have to be similar
            for T1,T2 in izip(Tlinks1,Tlinks2):
                if not all(T1-T2==0):
                    print 'error ',values

def test_kinematicsstructure():
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/schunk-lwa3-dual.robot.xml')
    env.AddRobot(robot)
    manips = robot.GetManipulators()
    for manip in manips:
        joints = robot.GetChain(manip.GetBase().GetIndex(),manip.GetEndEffector().GetIndex())
        print [joint.GetJointIndex() for joint in joints]
        joints = robot.GetChain(manip.GetEndEffector().GetIndex(),manip.GetBase().GetIndex())
        print [joint.GetJointIndex() for joint in joints]

def test_chain():
    hand = env.CreateKinBody();
    hand.SetName('object')
    hand.InitFromFile('robots/barretthand.kinbody.xml')
    env.AddKinBody(hand)
    palm = hand.GetLink('wam7')
    tip = hand.GetLink('Finger0-2')
    print palm, tip # first print
    print hand.GetChain( tip.GetIndex(), palm.GetIndex() ) # second print
    print hand.GetChain( palm.GetIndex(), tip.GetIndex() ) # third print

    
def test_dualarm_grabbing():
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/schunk-lwa3-dual.robot.xml')
    env.AddRobot(robot)
    manips = robot.GetManipulators()
    body = env.ReadKinBodyXMLFile('data/box3.kinbody.xml')
    env.AddKinBody(body)
    T = eye(4)
    T[1,3] = -1.18
    T[2,3] = 0.712
    body.SetTransform(T)
    robot.SetActiveManipulator(manips[0])
    robot.Grab(body)
    robot.SetJointValues(array([  0.00000000e+00,  -1.43329144e+00,  -3.99190831e-15, -1.86732388e+00,   5.77239752e-01,  -3.37631690e-07, 6.67713991e-08,   0.00000000e+00,  -1.70089030e+00, -6.42544150e-01,  -1.25030589e+00,  -3.33493233e-08, -5.58212676e-08,   1.60115015e-08]))
    assert robot.CheckSelfCollision()
def test_boxes():
    env = Environment()
    k = env.CreateKinBody()
    boxes = array(((0,0.5,0,0.1,0.2,0.3),(0.5,0,0,0.2,0.2,0.2)))
    k.InitFromBoxes(boxes,True)
    k.SetName('temp')
    env.AddKinBody(k)
    env.SetViewer('qtcoin')

def test_save():
    env=Environment()
    env.SetViewer('qtcoin')
    robot=env.ReadRobotXMLFile('robots/barretthand.robot.xml')
    env.AddRobot(robot)
    env.Save('test.dae')
    env.Reset()
    env.Load('test.dae')

def test_drawjoints():
    """draws the joint axes of the robot
    """
    env = robot.GetEnv()
    while True:
        h = None
        h = [env.drawlinelist(array([j.GetAnchor()-j.GetAxis(0),j.GetAnchor()+j.GetAxis(0)]),5,array([0,0,float(j.GetDOFIndex())/robot.GetDOF()]))  for j in robot.GetJoints()]
        time.sleep(0.1)

    while True:
        h = None
        joints = [robot.GetJoints()[i] for i in robot.GetManipulator('arm').GetArmJoints()]
        h = [env.drawlinelist(array([j.GetAnchor()-j.GetAxis(0),j.GetAnchor()+j.GetAxis(0)]),5,array([0,0,i/8.0]))  for i,j in enumerate(joints)]
        time.sleep(0.1)

def test_trimesh():
    env = Environment()
    body = env.CreateKinBody()
    vertices = array()
    indices = array()
    body.InitFromTrimesh(KinBody.Link.TriMesh(vertices,indices),True)
    env.AddKinBody(body)

def test_kinematics():
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
    env.AddRobot(robot)
    # use jacobians for validation
    with env:
        thresh=1e-5
        lower,upper = robot.GetDOFLimits()
        vlower,vupper = robot.GetDOFVelocityLimits()
        for j in range(robot.GetDOF()):
            valuestotest = []
            velocities = linspace(0.1,1.0,robot.GetDOF())
            if j%2:
                velocities[0::2] *= -1
            else:
                velocities[0::2] *= -1
            velocities = numpy.minimum(vupper,numpy.maximum(vlower,velocities))
            robot.SetDOFVelocities(velocities,True)
            assert linalg.norm(robot.GetDOFVelocities()-velocities) < thresh
            
            if 0 >= lower[j] and 0 <= upper[j]:
                valuestotest.append(0)
            if lower[j] != upper[j]:
                valuestotest.append(0.2*upper[j]+0.8*lower[j])
                valuestotest.append(0.8*upper[j]+0.2*lower[j])
            for value in valuestotest:
                robot.SetDOFValues([value],[j])
                values = robot.GetDOFValues()
                assert linalg.norm(robot.GetDOFValues()-values) < thresh
                Tlinks = robot.GetBodyTransformations()
                robot.SetBodyTransformations(Tlinks)
                assert linalg.norm(robot.GetDOFValues()-values) < thresh                
                # test velocities
                robot.SetDOFVelocities(velocities,True)
                linkvelocities = robot.GetLinkVelocities()
                assert linalg.norm(robot.GetDOFVelocities()-velocities) < thresh
                
                with robot:
                    # move each joint a little
                    for ilink in range(len(robot.GetLinks())):
                        localtrans = [0.1,0.2,0.3]
                        worldtrans = transformPoints(Tlinks[ilink],[localtrans])[0]
                        localquat = [1.0,0.0,0.0,0.0]
                        worldquat = quatFromRotationMatrix(Tlinks[ilink][0:3,0:3])
                        Jtrans = robot.CalculateJacobian(ilink,worldtrans)
                        Jquat = robot.CalculateRotationJacobian(ilink,worldquat)
                        Jangvel = robot.CalculateAngularVelocityJacobian(ilink)
                        robot.SetJointValues(values+(upper-lower)*0.001*sign(velocities))
                        deltavalues = robot.GetJointValues()-values
                        T=robot.GetLinks()[ilink].GetTransform()
                        deltatrans = worldtrans - transformPoints(T,[localtrans])[0]
                        deltarot = dot(linalg.inv(Tlinks[ilink][0:3,0:3]),T[0:3,0:3])
                        deltaquat = quatFromRotationMatrix(deltarot)
                        deltaangvel = axisAngleFromRotationMatrix(deltarot)
                        print dot(Jtrans,deltavalues) - deltatrans
                        print dot(Jquat,deltavalues) - deltaquat
                        print dot(Jangvel,deltavalues) - deltaangvel


def derive_normalizeAxisRotation():
    """Find the rotation theta around axis v such that rot(v,theta) * q is closest to the identity"""
    from sympy import *
    vx,vy,vz = Symbol('vx'),Symbol('vy'),Symbol('vz')
    v = Matrix(3,1,[vx,vy,vz])
    theta = Symbol('theta')
    q0 = Matrix(4,1,[cos(theta/2),sin(theta/2)*v[0],sin(theta/2)*v[1],sin(theta/2)*v[2]])
    q0dtheta = Matrix(4,1,[-sin(theta/2)/2,cos(theta/2)*v[0]/2,cos(theta/2)*v[1]/2,cos(theta/2)*v[2]/2])
    qx,qy,qz,qw = Symbol('qx'),Symbol('qy'),Symbol('qz'),Symbol('qw')
    q1 = Matrix(4,1,[qx,qy,qz,qw])
    qidentity = Matrix(4,1,[S.One,S.Zero,S.Zero,S.Zero])
    qfinal = Matrix(4,1,[q0[0]*q1[0] - q0[1]*q1[1] - q0[2]*q1[2] - q0[3]*q1[3],
                         q0[0]*q1[1] + q0[1]*q1[0] + q0[2]*q1[3] - q0[3]*q1[2],
                         q0[0]*q1[2] + q0[2]*q1[0] + q0[3]*q1[1] - q0[1]*q1[3],
                         q0[0]*q1[3] + q0[3]*q1[0] + q0[1]*q1[2] - q0[2]*q1[1]])
    qfinaldtheta = Matrix(4,1,[q0dtheta[0]*q1[0] - q0dtheta[1]*q1[1] - q0dtheta[2]*q1[2] - q0dtheta[3]*q1[3],
                         q0dtheta[0]*q1[1] + q0dtheta[1]*q1[0] + q0dtheta[2]*q1[3] - q0dtheta[3]*q1[2],
                         q0dtheta[0]*q1[2] + q0dtheta[2]*q1[0] + q0dtheta[3]*q1[1] - q0dtheta[1]*q1[3],
                         q0dtheta[0]*q1[3] + q0dtheta[3]*q1[0] + q0dtheta[1]*q1[2] - q0dtheta[2]*q1[1]])
    solveeq = qfinaldtheta.dot(qidentity-qfinal).expand()

    sthetad2 = Symbol('sthetad2') # sin(theta/2)
    cthetad2 = Symbol('cthetad2') # cos(theta/2)
    finaleq = Poly(solveeq.subs([(sin(theta/2),sthetad2),(cos(theta/2),cthetad2)]),sthetad2,cthetad2)
    # should be:
    # Poly((qw**2/2 + qx**2/2 + qy**2/2 + qz**2/2 - qw**2*vx**2/2 - qw**2*vy**2/2 - qw**2*vz**2/2 - qx**2*vx**2/2 - qx**2*vy**2/2 - qx**2*vz**2/2 - qy**2*vx**2/2 - qy**2*vy**2/2 - qy**2*vz**2/2 - qz**2*vx**2/2 - qz**2*vy**2/2 - qz**2*vz**2/2)*sthetad2*cthetad2 - qx/2*sthetad2 + (-qw*vz/2 - qy*vx/2 - qz*vy/2)*cthetad2, sthetad2, cthetad2)
    # sthetad2*cthetad2 coefficient reduces to 0
    # Poly(- qx/2*sthetad2 + (-qw*vz/2 - qy*vx/2 - qz*vy/2)*cthetad2, sthetad2, cthetad2)
    # theta = 2*atan2(-qw*vz-qz*vy-qy*vx,qx)
    
