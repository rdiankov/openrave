# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
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
from common_test_openrave import *

_multiprocess_can_split_ = True

def test_transformations():
    print "tests basic math transformations"
    for i in range(20):
        axisangle0 = (random.rand(3)-0.5)*1.99*pi/sqrt(3) # cannot have mag more than pi
        trans = random.rand(3)-0.5
        R0 = rotationMatrixFromAxisAngle(axisangle0)
        quat0 = quatFromAxisAngle(axisangle0)
        axisangle1 = axisAngleFromQuat(quat0)
        axisangle2 = axisAngleFromRotationMatrix(R0)
        R1 = rotationMatrixFromQuat(quat0)
        quat1 = quatFromRotationMatrix(R0)
        T0 = matrixFromAxisAngle(axisangle1)
        T0[0:3,3] = trans
        pose0 = poseFromMatrix(T0)
        T1 = matrixFromPose(pose0)
        poses = poseFromMatrices([T0,T1])
        T2,T3 = matrixFromPoses(poses)
        assert(sum(abs(R0-R1)) <= g_epsilon)
        assert(abs(sum(quat0**2)-1) <= g_epsilon)
        assert(sum(abs(linalg.inv(R0)-R0.transpose())))
        assert(sum(abs(linalg.inv(T0[0:3,0:3])-R0[0:3,0:3])))
        assert(sum(abs(T0-T1)) <= g_epsilon and sum(abs(T0-T2)) <= g_epsilon and sum(abs(T0-T3)) <= g_epsilon)
        assert(abs(abs(dot(quat0,quat1))-1) <= g_epsilon)
        assert(sum(abs(axisangle0-axisangle1)) <= g_epsilon and sum(abs(axisangle0-axisangle2)) <= g_epsilon)
        # test multiplication
        X = random.rand(10,3)-0.5
        Xnew = quatRotateArrayT(quat0,X)
        assert( sum(abs(Xnew-dot(X,R0.transpose()))) <= g_epsilon )
        assert( sum(abs(quatRotate(quat0,X[0]) - Xnew[0])) <= g_epsilon )
        assert( sum(abs(transformPoints(T0,X)-Xnew-tile(trans,(len(X),1)))) <= g_epsilon )
        assert( sum(abs(rotationMatrixFromQuat(quatMult(quat0,quat0)) - dot(R0,R0))) <= g_epsilon )
        assert( sum(abs(dot(T0,T0)-matrixFromPose(poseMult(pose0,pose0)))) <= g_epsilon )
        qarray0 = randquat(5)
        qarray1 = quatArrayTMult(qarray0,quat0)
        assert( sum(abs(quatArrayTRotate(qarray0,Xnew[0])-quatArrayTRotate(qarray1,X[0]))) <= g_epsilon )
        assert( sum(abs(quatArrayRotate(qarray0.transpose(),Xnew[0])-quatArrayRotate(qarray1.transpose(),X[0]))) <= g_epsilon )
        qarray2 = quatMultArrayT(quat0,qarray0)
        assert( sum(abs(quatRotateArrayT(quat0,quatArrayTRotate(qarray0,X[0]))-quatArrayTRotate(qarray2,X[0]))) <= g_epsilon )
        dists = quatArrayTDist(qarray0[0],qarray0)
        assert( all(dists>=0) and sum(dists)>0 and dists[0] <= g_epsilon )                    
        posearray0 = randpose(5)
        posearray1 = poseMultArrayT(pose0,posearray0)
        assert( sum(abs(poseMult(pose0,posearray0[0])-posearray1[0])) <= g_epsilon )
        for j in range(len(posearray0)):
            poseTransformPoints(pose0,poseTransformPoints(posearray0[j],X))
            poseTransformPoints(posearray1[j],X)
            assert( sum(abs(poseTransformPoints(pose0,poseTransformPoints(posearray0[j],X)) - poseTransformPoints(posearray1[j],X))) <= g_epsilon )
        # inverses
        matrices = matrixFromPoses(posearray0)
        posearrayinv0 = invertPoses(posearray0)
        for j in range(len(posearray0)):
            assert( sum(abs(transformInversePoints(matrices[j],X) - poseTransformPoints(posearrayinv0[j],X))) <= g_epsilon )
        
def test_fitcircle():
    print "fits 2d and 3d circles to a set of points"
    perturbation = 0.001
    for i in range(1000):
        T = randtrans()
        radius = random.rand()*5+0.3
        angles = random.rand(5)*2*pi
        points = c_[radius*cos(angles)+perturbation*(random.rand(len(angles))-0.5),radius*sin(angles)+perturbation*(random.rand(len(angles))-0.5),perturbation*(random.rand(len(angles))-0.5)]
        M = mean(points,0)
        Z=dot(transpose(points-M),points-M)
        eigvals = sort(linalg.eigvals(Z))
        if min(sqrt(eigvals[1]),sqrt(eigvals[2])) < 0.5*radius:
            # badly condition points, try again
            continue

        newpoints = transformPoints(T,points)
        newcenter, newradius, error = fitCircle(newpoints)
        assert( sum(abs(T[0:3,3]-newcenter)) <= perturbation*4 )
        assert( sum(abs(radius-newradius)) <= perturbation*4 )
        assert( error <= perturbation*4 )
        newpoints2d = points[:,0:2] + T[0:2,3]
        newcenter2d, newradius2d, error = fitCircle(newpoints2d)
        assert( sum(abs(T[0:2,3]-newcenter2d)) <= perturbation*4 )
        assert( sum(abs(radius-newradius2d)) <= perturbation*4 )
        assert( error <= perturbation*4 )

class TestKinematics(EnvironmentSetup):
    def test_bodybasic(self):
        print "check if the joint-link set-get functions are consistent along with jacobians"
        with self.env:
            for envfile in g_envfiles:
                self.env.Reset()
                self.env.Load(envfile)
                for i in range(20):
                    T = eye(4)
                    for body in self.env.GetBodies():
                        print body.GetXMLFilename()
                        # change the staticness of the first link (shouldn't affect anything)
                        body.GetLinks()[0].SetStatic((i%2)>0)

                        Told = body.GetTransform()
                        Tallold = body.GetLinkTransformations()
                        dofvaluesold = body.GetDOFValues()
                        T = randtrans()
                        body.SetTransform(T)
                        assert( transdist(T,body.GetTransform()) <= g_epsilon )
                        body.SetLinkTransformations(Tallold)
                        assert( transdist(Tallold,body.GetLinkTransformations()) <= g_epsilon )
                        Tallnew = [randtrans() for j in range(len(Tallold))]
                        body.SetLinkTransformations(Tallnew)
                        assert( transdist(Tallnew,body.GetLinkTransformations()) <= g_epsilon )
                        for link, T in izip(body.GetLinks(),Tallold):
                            link.SetTransform(T)
                        assert( transdist(Tallold,body.GetLinkTransformations()) <= g_epsilon )
                        # dof
                        assert( transdist(dofvaluesold,body.GetDOFValues()) <= g_epsilon )
                        dofvaluesnew = randlimits(*body.GetDOFLimits())
                        body.SetDOFValues(dofvaluesnew)
                        assert( transdist(dofvaluesnew,body.GetDOFValues()) <= g_epsilon )
                        Tallnew = body.GetLinkTransformations()
                        body.SetTransformWithDOFValues(body.GetTransform(),dofvaluesnew)
                        assert( transdist(Tallnew,body.GetLinkTransformations()) <= g_epsilon )
                        assert( transdist(dofvaluesnew,body.GetDOFValues()) <= g_epsilon )
                        for joint in body.GetJoints():
                            assert( transdist(joint.GetValues(), dofvaluesnew[joint.GetDOFIndex():(joint.GetDOFIndex()+joint.GetDOF())]) <= g_epsilon )
                        Tallnew2 = [randtrans() for link in body.GetLinks()]
                        for link,T in izip(body.GetLinks(),Tallnew2):
                            link.SetTransform(T)
                        assert( transdist(body.GetLinkTransformations(),Tallnew2) <= g_epsilon )
                        body.SetLinkTransformations(Tallnew)
                        for idir in range(20):
                            deltavalues0 = array([g_jacobianstep*(random.randint(3)-1) for j in range(body.GetDOF())])
                            localtrans = randtrans()
                            for ilink,link in enumerate(body.GetLinks()):
                                jointchain = body.GetChain(0,link.GetIndex())
                                if len(jointchain) == 0:
                                    continue                            
                                body.SetDOFValues(dofvaluesnew)
                                Tlink = dot(link.GetTransform(),localtrans)
                                worldtrans = Tlink[0:3,3]
                                worldquat = quatFromRotationMatrix(Tlink[0:3,0:3]) # should use localquat
                                worldaxisangle = axisAngleFromRotationMatrix(Tlink[0:3,0:3])
                                Jtrans = body.CalculateJacobian(ilink,worldtrans)
                                Jquat = body.CalculateRotationJacobian(ilink,worldquat)
                                Jangvel = body.CalculateAngularVelocityJacobian(ilink)
                                body.SetDOFValues(dofvaluesnew+deltavalues0)
                                deltavalues = body.GetDOFValues()-dofvaluesnew
                                armlength = bodymaxjointdist(link,localtrans[0:3,3])
                                thresh = armlength*sum(abs(deltavalues))*1.1
                                if armlength < 0.0001:
                                    continue
                                Tlinknew=dot(link.GetTransform(),localtrans)
                                newaxisangle = axisAngleFromRotationMatrix(Tlinknew[0:3,0:3])
                                newquat = quatFromRotationMatrix(Tlinknew[0:3,0:3])
                                if dot(worldquat,newquat) < 0:
                                    newquat = -newquat
                                deltatrans = Tlinknew[0:3,3] - worldtrans
                                assert( transdist(dot(Jtrans,deltavalues),deltatrans) <= thresh )
                                assert( transdist(dot(Jquat,deltavalues)+worldquat,newquat) <= 2*thresh )
                                raveLogDebug(repr(dofvaluesnew))
                                raveLogDebug(repr(deltavalues))
                                print 'angle dist: ',axisangledist(dot(Jangvel,deltavalues)+worldaxisangle,newaxisangle)
                                assert( axisangledist(dot(Jangvel,deltavalues)+worldaxisangle,newaxisangle) <= 2*thresh )

    def test_bodyvelocities(self):
        print "check physics/dynamics properties"
        with self.env:
            for envfile in g_envfiles:
                self.env.Reset()
                self.env.Load(envfile)
                # try all loadable physics engines
                #self.env.SetPhysicsEngine()
                for i in range(10):
                    T = eye(4)
                    for body in self.env.GetBodies():
                        for ijoint,joint in enumerate(body.GetJoints()):
                            assert(joint.GetJointIndex()==ijoint and body.GetJointIndex(joint.GetName()) == ijoint )
                            assert(joint == body.GetJoint(joint.GetName()) )
                            assert(joint == body.GetJointFromDOFIndex(joint.GetDOFIndex()) )
                        # velocities, has to do with physics engine
                        oldlinkvels = body.GetLinkVelocities()
                        dofvelnew = randlimits(*body.GetDOFVelocityLimits())
                        link0vel = [random.rand(3)-0.5,random.rand(3)-0.5]
                        body.SetVelocity(*link0vel)
                        assert( sum(abs(body.GetDOFVelocities())) <= g_epsilon )
                        body.SetDOFVelocities(dofvelnew,*link0vel,checklimits=True)
                        assert( transdist(body.GetDOFVelocities(),dofvelnew) <= g_epsilon )
                        linkvels = body.GetLinkVelocities()
                        newlinkvels = random.rand(len(body.GetLinks()),6)-0.5
                        for link,vel in izip(body.GetLinks(),newlinkvels):
                            link.SetVelocity(vel[0:3],vel[3:6])
                            assert( transdist(link.GetVelocity(),[vel[0:3],vel[3:6]]) <= g_epsilon )
                        assert( transdist(body.GetLinkVelocities(),newlinkvels) <= g_epsilon )
                        body.SetDOFVelocities(dofvelnew,linear=[0,0,0],angular=[0,0,0],checklimits=True)
                        assert( transdist(body.GetDOFVelocities(),dofvelnew) <= g_epsilon )
                        for link,vel in izip(body.GetLinks(),linkvels):
                            link.SetVelocity(vel[0:3],vel[3:6])
                        assert( transdist(body.GetLinkVelocities(),linkvels) <= g_epsilon and transdist(body.GetDOFVelocities(),dofvelnew) <= g_epsilon )
                        for joint in body.GetJoints():
                            assert( transdist(joint.GetVelocities(), dofvelnew[joint.GetDOFIndex():(joint.GetDOFIndex()+joint.GetDOF())]) <= g_epsilon )
                        for link,vel in izip(body.GetLinks(),linkvels):
                            assert( transdist(link.GetVelocity(),[vel[0:3],vel[3:6]]) <= g_epsilon )
                        # test consistency with kinematics

    def test_hierarchy(self):
        print "tests the kinematics hierarchy"
        with self.env:
            for robotfile in g_robotfiles:
                self.env.Reset()
                self.env.Load(robotfile)
                body = self.env.GetBodies()[0]
                lowerlimit,upperlimit = body.GetDOFLimits()
                for i in range(len(lowerlimit)):
                    if upperlimit[i] - lowerlimit[i] > pi:
                        upperlimit[i] = lowerlimit[i]+pi
                body.SetDOFValues(lowerlimit)
                for ijoint,joint in enumerate(body.GetJoints()):
                    for ilink, link in enumerate(body.GetLinks()):
                        affect = body.DoesAffect(ijoint,ilink)
                        body.SetDOFValues([lowerlimit[joint.GetDOFIndex()]],[joint.GetDOFIndex()])
                        Tlink = link.GetTransform()
                        body.SetDOFValues([upperlimit[joint.GetDOFIndex()]],[joint.GetDOFIndex()])
                        Tlinknew = link.GetTransform()
                        if affect != 0 and not joint.IsStatic():
                            assert( lowerlimit[joint.GetDOFIndex()] < upperlimit[joint.GetDOFIndex()] )
                            assert( transdist(Tlink,Tlinknew) >= (upperlimit[joint.GetDOFIndex()]-lowerlimit[joint.GetDOFIndex()])*0.1 )
                        else:
                            assert( transdist(Tlink,Tlinknew) <= g_epsilon )
                body.SetDOFValues(lowerlimit)
                jointaxes0 = [joint.GetAxis(0) for joint in body.GetDependencyOrderedJoints()]
                jointanchors0 = [joint.GetAnchor() for joint in body.GetDependencyOrderedJoints()]
                assert( len(body.GetDependencyOrderedJoints()) == len(body.GetJoints()) )
                for ijoint in range(len(body.GetDependencyOrderedJoints())-1,0,-1):
                    joint = body.GetDependencyOrderedJoints()[ijoint]
                    body.SetDOFValues([upperlimit[joint.GetDOFIndex()]],[joint.GetDOFIndex()])
                    for ijoint2,joint2 in enumerate(body.GetDependencyOrderedJoints()[:ijoint]):
                        assert( transdist(joint2.GetAxis(0), jointaxes0[ijoint2]) <= g_epsilon )
                        assert( transdist(joint2.GetAnchor(), jointanchors0[ijoint2]) <= g_epsilon )
                for ilink0,ilink1 in combinations(range(len(body.GetLinks())),2):
                    link0 = body.GetLinks()[ilink0]
                    link1 = body.GetLinks()[ilink1]
                    jointchain = body.GetChain( ilink0, ilink1, returnjoints = True )
                    linkchain = body.GetChain( ilink0, ilink1, returnjoints = False )
                    assert( len(jointchain)+1 == len(linkchain) )
                    assert( jointchain[::-1] == body.GetChain( ilink1, ilink0, returnjoints = True ))
                    assert( linkchain[::-1] == body.GetChain( ilink1, ilink0, returnjoints = False ))
                    for ilink in range(len(linkchain)-1):
                        j = jointchain[ilink]
                        if j.GetDOFIndex() >= 0:
                            assert( body.IsDOFInChain(linkchain[ilink].GetIndex(),linkchain[ilink+1].GetIndex(),j.GetDOFIndex()) )
                        assert( linkchain[ilink].IsParentLink(linkchain[ilink+1]) or linkchain[ilink+1].IsParentLink(linkchain[ilink]) )
                        assert( linkchain[ilink] == j.GetHierarchyChildLink() or linkchain[ilink] == j.GetHierarchyParentLink())
                        assert( linkchain[ilink+1] == j.GetHierarchyChildLink() or linkchain[ilink+1] == j.GetHierarchyParentLink())
                        
                #loops = body.GetClosedLoops()
                #for loop in loops:
                #    lknotindex = [i for i,(link,joint) in enumerate(loop) if link.GetIndex() == knot]
                #    lknownindex = [i for i,(link,joint) in enumerate(loop) if link == knownlink]
                    
    def test_collada(self):
        print "test that collada import/export works"
        epsilon = 400*g_epsilon # because exporting, expect to lose precision, should fix this
        for robotfile in g_robotfiles[3:4]:
            self.env.Reset()
            robot0=self.env.ReadRobotXMLFile(robotfile)
            self.env.AddRobot(robot0,True)
            robot0.SetTransform(eye(4))
            self.env.Save('test.zae')
            robot1=self.env.ReadRobotXMLFile('test.zae')
            self.env.AddRobot(robot1,True)
            robot1.SetTransform(eye(4))
            assert(len(robot0.GetJoints())==len(robot1.GetJoints()))
            assert(len(robot0.GetPassiveJoints()) == len(robot1.GetPassiveJoints()))
            joints0 = robot0.GetJoints()+robot0.GetPassiveJoints()
            joints1 = robot1.GetJoints()+robot1.GetPassiveJoints()
            for j0 in joints0:
                assert( len(j0.GetName()) > 0 )
                j1s = [j1 for j1 in joints1 if j0.GetName() == j1.GetName()]
                assert( len(j1s) == 1 )
                j1 = j1s[0]
                assert( transdist(j0.GetAnchor(),j1.GetAnchor()) <= epsilon )
                assert( j0.GetDOF() == j1.GetDOF() and j0.GetType() == j1.GetType() )
                # todo, once physics is complete, uncomment
                #assert( j0.GetHierarchyParentLink().GetName() == j1.GetHierarchyParentLink().GetName() )
                #assert( j0.GetHierarchyChildLink().GetName() == j1.GetHierarchyChildLink().GetName() )
                assert( transdist(j0.GetInternalHierarchyLeftTransform(),j1.GetInternalHierarchyLeftTransform()) <= epsilon )
                assert( transdist(j0.GetInternalHierarchyRightTransform(),j1.GetInternalHierarchyRightTransform()) <= epsilon )
                assert( j0.IsStatic() == j1.IsStatic() )
                assert( transdist(j0.GetLimits(),j1.GetLimits()) <= epsilon )
                for idof in range(j0.GetDOF()):
                    assert( abs(j0.GetMaxVel(idof)-j1.GetMaxVel(idof)) <= epsilon )
                    assert( abs(j0.GetMaxAccel(idof)-j1.GetMaxAccel(idof)) <= epsilon )
                    assert( j0.IsCircular(idof) == j1.IsCircular(idof) )
                    assert( j0.IsRevolute(idof) == j1.IsRevolute(idof) )
                    assert( j0.IsPrismatic(idof) == j1.IsPrismatic(idof) )
                    assert( transdist(j0.GetInternalHierarchyAxis(idof),j1.GetInternalHierarchyAxis(idof)) <= epsilon )
                    assert( j0.IsMimic(idof) == j1.IsMimic(idof) )
                    if j0.IsMimic(idof):
                        mimicjoints0 = [robot0.GetJointFromDOFIndex(index).GetName() for index in j0.GetMimicDOFIndices(idof)]
                        mimicjoints1 = [robot1.GetJointFromDOFIndex(index).GetName() for index in j1.GetMimicDOFIndices(idof)]
                        assert( mimicjoints0 == mimicjoints1 )
                        # is it possible to compare equations?
                        # assert( j0.GetMimicEquation(idof) == j1.GetMimicEquation(idof) )
                    #todo: GetResolution, GetWeight
                    #todo: ignore links
            assert(len(robot0.GetLinks())==len(robot1.GetLinks()))
            for link0 in robot0.GetLinks():
                link1s = [link1 for link1 in robot1.GetLinks() if link0.GetName() == link1.GetName()]
                assert( len(link1s) == 1 )
                link1 = link1s[0]
                assert( transdist(link0.GetTransform(),link1.GetTransform()) <= epsilon )
                #assert( link0.IsStatic() == link1.IsStatic() )
                assert( len(link0.GetParentLinks()) == len(link1.GetParentLinks()) )
                assert( all([lp0.GetName()==lp1.GetName() for lp0, lp1 in izip(link0.GetParentLinks(),link1.GetParentLinks())]) )
                assert( len(link0.GetGeometries()) == len(link1.GetGeometries()) )
                # todo: compare geometry

    def test_initkinbody(self):
        print "tests initializing a kinematics body"
        with self.env:
            k = self.env.CreateKinBody()
            boxes = array(((0,0.5,0,0.1,0.2,0.3),(0.5,0,0,0.2,0.2,0.2)))
            k.InitFromBoxes(boxes,True)
            k.SetName('temp')
            self.env.AddKinBody(k)
            assert( len(k.GetLinks()) == 1 and len(k.GetLinks()[0].GetGeometries()) == 2 )
            assert( k.GetLinks()[0].GetGeometries()[0].GetType() == KinBody.Link.GeomProperties.Type.Box )
            assert( k.GetLinks()[0].GetGeometries()[1].GetType() == KinBody.Link.GeomProperties.Type.Box )
            k2 = self.env.CreateKinBody()
            k2.InitFromTrimesh(TriMesh(*ComputeBoxMesh([0.1,0.2,0.3])),True)
            k2.SetName('temp')
            self.env.AddKinBody(k2,True)
            assert( transdist(k2.ComputeAABB().extents(),[0.1,0.2,0.3]) <= g_epsilon )

    def test_geometrychange(self):
        print "change geometry and test if changes are updated"
        env=self.env
        with env:
            env.Load(g_envfiles[0])
            for body in env.GetBodies():
                for link in body.GetLinks():
                    geom = link.GetGeometries()[0]
                    if geom.IsModifiable():
                        extents = [0.6,0.5,0.1]
                        geom.SetCollisionMesh(TriMesh(*ComputeBoxMesh(extents)))
                        vmin = numpy.min(geom.GetCollisionMesh().vertices,0)
                        vmax = numpy.max(geom.GetCollisionMesh().vertices,0)
                        assert( transdist(0.5*(vmax-vmin),extents) <= g_epsilon )

    def test_hashes(self):
        robot = self.env.ReadRobotXMLFile(g_robotfiles[0])
        self.env.AddRobot(robot)
        s = robot.serialize(SerializationOptions.Kinematics)
        hash0 = robot.GetKinematicsGeometryHash()
        robot.SetLinkTransformations([randtrans() for link in robot.GetLinks()])
        hash1 = robot.GetKinematicsGeometryHash()
        assert( hash0 == hash1 )
        
    def test_staticlinks(self):
        env=self.env
        robot=env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
        env.AddRobot(robot)
        with env:
            robot.SetDOFValues([0.5],[0])
            values = robot.GetDOFValues()
            anchors = [j.GetAnchor() for j in robot.GetJoints()]
            axes = [j.GetAxis(0) for j in robot.GetJoints()]
            Tlinks = robot.GetLinkTransformations()
            assert(not robot.GetLinks()[0].IsStatic())
            robot.SetDOFValues(zeros(robot.GetDOF()))
            robot.GetLinks()[0].SetStatic(True)
            assert(robot.GetLinks()[0].IsStatic())
            robot.SetDOFValues([0.5],[0])
            assert( transdist(values,robot.GetDOFValues()) <= g_epsilon )
            assert( transdist(Tlinks,robot.GetLinkTransformations()) <= g_epsilon )
            assert( transdist(anchors, [j.GetAnchor() for j in robot.GetJoints()]) <= g_epsilon )
            assert( transdist(axes, [j.GetAxis(0) for j in robot.GetJoints()]) <= g_epsilon )

    def test_jointoffset(self):
        env=self.env
        with env:
            for robotfile in g_robotfiles:
                env.Reset()
                env.Load(robotfile)
                body = env.GetBodies()[0]
                
                zerovalues = zeros(body.GetDOF())
                body.SetDOFValues(zerovalues)
                Tlinks = body.GetLinkTransformations()
                limits = body.GetDOFLimits()
                limits = [numpy.maximum(-3*ones(body.GetDOF()),limits[0]), numpy.minimum(3*ones(body.GetDOF()),limits[1])]
                for myiter in range(10):
                    # todo set the first link to static
                    body.SetLinkTransformations(Tlinks)
                    body.SetZeroConfiguration()
                    assert( transdist(Tlinks,body.GetLinkTransformations()) <= g_epsilon )
                    body.GetLinks()[0].SetStatic(myiter%2)
                    offsets = randlimits(*limits)
                    raveLogDebug(repr(offsets))
                    body.SetDOFValues(offsets)
                    Tlinksnew = body.GetLinkTransformations()
                    body.SetZeroConfiguration()
                    assert( transdist(zerovalues,body.GetDOFValues()) <= g_epsilon )
                    assert( transdist(Tlinksnew,body.GetLinkTransformations()) <= g_epsilon )
                    #body.SetDOFValues(-offsets,range(body.GetDOF()),checklimits=False)
                    #assert( transdist(Tlinksnew,body.GetLinkTransformations()) <= g_epsilon )
                    offsets = randlimits(*limits)
                    body.SetDOFValues(offsets)
                    assert( transdist(offsets,body.GetDOFValues()) <= g_epsilon )

    def test_wrapoffset(self):
        env=self.env
        with env:
            for robotfile in g_robotfiles:
                env.Reset()
                env.Load(robotfile)
                body = env.GetBodies()[0]
                
                zerovalues = zeros(robot.GetDOF())
                for i,offset in enumerate(zerovalues):
                    joint=robot.GetJointFromDOFIndex(i)
                    joint.SetWrapOffset(offset,i-joint.GetDOFIndex())
                robot.SetDOFValues(zerovalues)
                limits = robot.GetDOFLimits()
                limits = [numpy.maximum(-3*ones(robot.GetDOF()),limits[0]), numpy.minimum(3*ones(robot.GetDOF()),limits[1])]
                for myiter in range(10):
                    robot.GetLinks()[0].SetStatic(myiter%2)
                    robot.SetDOFValues(zerovalues)
                    Tlinks = robot.GetLinkTransformations()
                    offsets = randlimits(*limits)
                    raveLogDebug(repr(offsets))
                    for i,offset in enumerate(offsets):
                        joint=robot.GetJointFromDOFIndex(i)
                        joint.SetWrapOffset(offset,i-joint.GetDOFIndex())

                    assert( transdist(zerovalues,robot.GetDOFValues()) <= g_epsilon )
                    assert( transdist(Tlinks,robot.GetLinkTransformations()) <= g_epsilon )
                    robot.SetDOFValues(offsets)
                    assert( transdist(offsets,robot.GetDOFValues()) <= g_epsilon )
