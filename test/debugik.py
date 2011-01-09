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

# random code that helps with debugging/testing the python interfaces and examples
# this is not meant to be run by normal users
from openravepy import *
import openravepy.examples
from openravepy.interfaces import *
from numpy import *
import numpy,time

def test_ikgeneration():
    import inversekinematics
    env = Environment()
    env.SetDebugLevel(DebugLevel.Debug)
    #robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    robot = env.ReadRobotXMLFile('robots/barrettwam4.robot.xml')
    robot.SetActiveManipulator('arm')
    env.AddRobot(robot)
    self = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)

    freejoints=None
    usedummyjoints=False
    accuracy = None
    precision = None
    iktype=inversekinematics.InverseKinematicsModel.Type_Direction3D
    self.generate(freejoints=freejoints,usedummyjoints=usedummyjoints,iktype=iktype)

    baselink=self.manip.GetBase().GetIndex()
    eelink = self.manip.GetEndEffector().GetIndex()
    solvejoints=solvejoints
    freeparams=freejoints
    usedummyjoints=usedummyjoints
    solvefn=solvefn

def test_handstraight_jacobian():
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
    env.AddRobot(robot)
    # use jacobians for validation
    with env:
        deltastep = 0.01
        thresh=1e-5
        lower,upper = robot.GetDOFLimits()
        manip = robot.GetActiveManipulator()
        ilink = manip.GetEndEffector().GetIndex()
        localtrans = [0.1,0.2,0.3]
        localquat = [1.0,0.0,0.0,0.0]
        stats = []
        robot.SetActiveDOFs(manip.GetArmIndices())
        while True:
            robot.SetDOFValues(random.rand()*(upper-lower)+lower)
            if not robot.CheckSelfCollision() and not env.CheckCollision(robot):
                break
        deltatrans = deltastep*(random.rand(3)-0.5)
        while True:
            values = robot.GetDOFValues(manip.GetArmIndices())
            T = manip.GetEndEffectorTransform()
            Tnew = array(T)
            Tnew[0:3,3] += deltatrans
            sols = manip.FindIKSolutions(Tnew,IkFilterOptions.CheckEnvCollisions)
            if len(sols) == 0:
                break
            dists = sum( (array(sols)-tile(values,(len(sols),1)))**2, 1)
            sol = sols[argmin(dists)]            
            J = robot.CalculateActiveJacobian(ilink,manip.GetEndEffectorTransform()[0:3,3])
            Jrot = robot.CalculateActiveAngularVelocityJacobian(ilink)
            Jtrans = r_[J,Jrot]
            JJt = dot(Jtrans,transpose(Jtrans))
            deltavalues = dot(transpose(Jtrans),dot(linalg.inv(JJt),r_[deltatrans,0,0,0]))
            #dtt = dot(r_[deltatrans,0,0,0],JJt)
            #alpha = dot(r_[deltatrans,0,0,0], dtt)/dot(dtt,dtt)
            #deltavalues = alpha*dot(transpose(Jtrans),r_[deltatrans,0,0,0])
            realvalues = sol-values
            realtransdelta = dot(J,realvalues)
            #err = sum(abs(sign(deltatrans)-sign(realtransdelta)))
            err = dot(deltatrans,realtransdelta)/(linalg.norm(deltatrans)*linalg.norm(realtransdelta))
            d = sqrt(sum(realvalues**2)/sum(deltavalues**2))
            if err < 0.95 or d > 10:
                print realvalues
                print deltavalues
            stats.append((err,d))
            print stats[-1]
            robot.SetDOFValues(sol,manip.GetArmIndices())

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.hist(stats,100)
    fig.show()

def test_ik():
    import inversekinematics
    env = Environment()
    env.SetDebugLevel(DebugLevel.Debug)
    robot = env.ReadRobotXMLFile('/home/rdiankov/ros/honda/binpicking/robots/tx90.robot.xml')
    env.AddRobot(robot)
    manip=robot.GetActiveManipulator()
    #manip=robot.SetActiveManipulator('leftarm_torso')
    self = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
    self.load()
    self.perftiming(10)
    robot.SetDOFValues([-2.62361, 1.5708, -0.17691, -3.2652, 0, -3.33643],manip.GetArmJoints())
    T=manip.GetEndEffectorTransform()
    print robot.CheckSelfCollision()
    #[j.SetJointLimits([-pi],[pi]) for j in robot.GetJoints()]
    robot.SetDOFValues(zeros(robot.GetDOF()))
    values=manip.FindIKSolution(T,False)
    Tlocal = dot(dot(linalg.inv(manip.GetBase().GetTransform()),T),linalg.inv(manip.GetGraspTransform()))
    print ' '.join(str(f) for f in Tlocal[0:3,0:4].flatten())
    robot.SetDOFValues (values,manip.GetArmJoints())
    print manip.GetEndEffectorTransform()
    
    sols=manip.FindIKSolutions(T,False)
    for i,sol in enumerate(sols):
        robot.SetDOFValues(sol)
        Tnew = manip.GetEndEffectorTransform()
        if sum((Tnew-T)**2) > 0.0001:
            print i
            break
        
def debug_ik():
    env = Environment()
    env.Load('data/katanatable.env.xml')
    env.StopSimulation()
    robot = env.GetRobots()[0]

    print robot.GetTransform()[0:3,3]
    target=array([-0.34087322,  0.64355438,  1.01439696])
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Translation3D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    sol = ikmodel.manip.FindIKSolution(IkParameterization(target,IkParameterization.Type.Translation3D),IkFilterOptions.CheckEnvCollisions)
    print sol
    robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
    print linalg.norm(target - ikmodel.manip.GetEndEffectorTransform()[0:3,3])

def test_ik():
    from sympy import *
    import __builtin__
    from openravepy.ikfast import SolverStoreSolution, SolverSolution, combinations, SolverSequence, fmod, SolverRotation, SolverIKChainTransform6D, SolverBranchConds
    ikmodel=self
    self = solver
    alljoints = self.getJointsInChain(baselink, eelink)
    chain = []
    for joint in alljoints:
        issolvejoint = any([i == joint.jointindex for i in solvejoints])
        if usedummyjoints and not issolvejoint and not any([i == joint.jointindex for i in freejointinds]):
            joint.isdummy = True
        joint.isfreejoint = not issolvejoint and not joint.isdummy
        chain.append(joint)
    Tee = eye(4)
    for i in range(0,3):
        for j in range(0,3):
            Tee[i,j] = Symbol("r%d%d"%(i,j))
    Tee[0,3] = Symbol("px")
    Tee[1,3] = Symbol("py")
    Tee[2,3] = Symbol("pz")

    chaintree = solvefn(self,chain,Tee)
    code=ikfast_generator_cpp.CodeGenerator().generate(chaintree)
    
