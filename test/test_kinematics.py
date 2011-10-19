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

class TestKinematics(EnvironmentSetup):
    def test_bodybasic(self):
        print "check if the joint-link set-get functions are consistent along with jacobians"
        with self.env:
            for envfile in g_envfiles:
                self.env.Reset()
                self.env.Load(envfile,{'skipgeometry':'1'})
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
                        # do it again
                        body.SetDOFValues(dofvaluesnew)
                        assert( transdist(Tallnew,body.GetLinkTransformations()) <= g_epsilon )
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
                                if armlength < 0.0001 or thresh < 1e-12:
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
                                #print 'angle dist: ',axisangledist(dot(Jangvel,deltavalues)+worldaxisangle,newaxisangle), 2*thresh, armlength
                                assert( axisangledist(dot(Jangvel,deltavalues)+worldaxisangle,newaxisangle) <= 2*thresh )

    def test_bodyvelocities(self):
        print "check physics/dynamics properties"
        with self.env:
            for envfile in g_envfiles:
                self.env.Reset()
                self.env.Load(envfile,{'skipgeometry':'1'})
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
                        vellimits = body.GetDOFVelocityLimits()
                        dofvelnew = randlimits(-vellimits,vellimits)
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
                self.env.Load(robotfile,{'skipgeometry':'1'})
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
        for robotfile in g_robotfiles:
            self.env.Reset()
            robot0=self.env.ReadRobotURI(robotfile)
            self.env.AddRobot(robot0,True)
            robot0.SetTransform(eye(4))
            self.env.Save('test.zae')
            robot1=self.env.ReadRobotURI('test.zae')
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
            k = RaveCreateKinBody(self.env,'')
            boxes = array(((0,0.5,0,0.1,0.2,0.3),(0.5,0,0,0.2,0.2,0.2)))
            k.InitFromBoxes(boxes,True)
            k.SetName('temp')
            self.env.AddKinBody(k)
            assert( len(k.GetLinks()) == 1 and len(k.GetLinks()[0].GetGeometries()) == 2 )
            assert( k.GetLinks()[0].GetGeometries()[0].GetType() == KinBody.Link.GeomProperties.Type.Box )
            assert( k.GetLinks()[0].GetGeometries()[1].GetType() == KinBody.Link.GeomProperties.Type.Box )
            k2 = RaveCreateKinBody(self.env,'')
            k2.InitFromTrimesh(TriMesh(*misc.ComputeBoxMesh([0.1,0.2,0.3])),True)
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
                        geom.SetCollisionMesh(TriMesh(*misc.ComputeBoxMesh(extents)))
                        vmin = numpy.min(geom.GetCollisionMesh().vertices,0)
                        vmax = numpy.max(geom.GetCollisionMesh().vertices,0)
                        assert( transdist(0.5*(vmax-vmin),extents) <= g_epsilon )

    def test_hashes(self):
        robot = self.env.ReadRobotURI(g_robotfiles[0])
        self.env.AddRobot(robot)
        s = robot.serialize(SerializationOptions.Kinematics)
        hash0 = robot.GetKinematicsGeometryHash()
        robot.SetLinkTransformations([randtrans() for link in robot.GetLinks()])
        hash1 = robot.GetKinematicsGeometryHash()
        assert( hash0 == hash1 )

    def test_staticlinks(self):
        env=self.env
        robot=env.ReadRobotURI('robots/barrettwam.robot.xml')
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
                env.Load(robotfile,{'skipgeometry':'1'})
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
                env.Load(robotfile,{'skipgeometry':'1'})
                body = env.GetBodies()[0]

                zerovalues = zeros(body.GetDOF())
                for i,offset in enumerate(zerovalues):
                    joint=body.GetJointFromDOFIndex(i)
                    joint.SetWrapOffset(offset,i-joint.GetDOFIndex())
                body.SetDOFValues(zerovalues)
                limits = body.GetDOFLimits()
                limits = [numpy.maximum(-3*ones(body.GetDOF()),limits[0]), numpy.minimum(3*ones(body.GetDOF()),limits[1])]
                for myiter in range(10):
                    body.GetLinks()[0].SetStatic(myiter%2)
                    body.SetDOFValues(zerovalues)
                    Tlinks = body.GetLinkTransformations()
                    offsets = randlimits(*limits)
                    raveLogDebug(repr(offsets))
                    for i,offset in enumerate(offsets):
                        joint=body.GetJointFromDOFIndex(i)
                        joint.SetWrapOffset(offset,i-joint.GetDOFIndex())

                    assert( transdist(zerovalues,body.GetDOFValues()) <= g_epsilon )
                    assert( transdist(Tlinks,body.GetLinkTransformations()) <= g_epsilon )
                    body.SetDOFValues(offsets)
                    assert( transdist(offsets,body.GetDOFValues()) <= g_epsilon )

    def test_joints(self):
        env=self.env
        xml = """
<kinbody name="universal">
  <body name="L0">
    <geom type="box">
      <extents>0.05 0.05 0.1</extents>
    </geom>
  </body>
  <body name="L1">
    <translation>0 0 0.3</translation>
    <geom type="box">
      <extents>0.05 0.05 0.1</extents>
    </geom>
  </body>
  <joint type="%s" name="J0">
    <body>L0</body>
    <body>L1</body>
    <axis1>1 0 0</axis1>
    <axis2>0 1 0</axis2>
    <anchor>0 0 0.2</anchor>
  </joint>
</kinbody>
"""
        with env:
#             body = env.ReadKinBodyData(xml%'universal')
#             env.AddKinBody(body)
#             assert(body.GetDOF()==2)
#             assert(len(body.GetJoints())==1)
#             assert(env.GetJoints()[0].GetType()==Joint.Type.Universal)
            env.Reset()
            body = env.ReadKinBodyData(xml%'hinge2')
            env.AddKinBody(body)
            assert(body.GetDOF()==2)
            assert(len(body.GetJoints())==1)
            assert(body.GetJoints()[0].GetType()==KinBody.Joint.Type.Hinge2)

    def test_mimicjoints(self):
        env=self.env
        xml="""
<kinbody name="a">
  <body name="L0">
  </body>
  <body name="L1">
  </body>
  <body name="L2">
  </body>
  <body name="L3">
  </body>
  <joint name="J0" type="hinge">
    <body>L0</body>
    <body>L1</body>
    <axis>1 0 0</axis>
  </joint>
  <joint name="J0a" type="hinge" mimic_pos="2*J0" mimic_vel="|J0 2" mimic_accel="|J0 0">
    <body>L1</body>
    <body>L2</body>
    <axis>1 0 0</axis>
  </joint>
  <joint name="J0b" type="hinge" mimic_pos="2*J0a+J0" mimic_vel="|J0a 2 |J0 1" mimic_accel="|J0a 0 |J0 0">
    <body>L2</body>
    <body>L3</body>
    <axis>1 0 0</axis>
  </joint>
</kinbody>
"""
        body = env.ReadKinBodyData(xml)
        env.AddKinBody(body)
        assert(len(body.GetJoints())==1)
        assert(len(body.GetPassiveJoints())==2)
        value = 0.5
        body.SetDOFValues([value])
        J0a = body.GetJoint('J0a')
        J0b = body.GetJoint('J0b')
        assert(abs(J0a.GetValues()[0]-2*value) <= g_epsilon )
        assert(abs(J0b.GetValues()[0]-5*value) <= g_epsilon )
        assert(J0a.GetMimicDOFIndices() == [0])
        assert(J0b.GetMimicDOFIndices() == [0])
