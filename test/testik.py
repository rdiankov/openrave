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
    robot.SetJointValues([-2.62361, 1.5708, -0.17691, -3.2652, 0, -3.33643],manip.GetArmJoints())
    T=manip.GetEndEffectorTransform()
    print robot.CheckSelfCollision()
    #[j.SetJointLimits([-pi],[pi]) for j in robot.GetJoints()]
    robot.SetJointValues(zeros(robot.GetDOF()))
    values=manip.FindIKSolution(T,False)
    Tlocal = dot(dot(linalg.inv(manip.GetBase().GetTransform()),T),linalg.inv(manip.GetGraspTransform()))
    print ' '.join(str(f) for f in Tlocal[0:3,0:4].flatten())
    robot.SetJointValues (values,manip.GetArmJoints())
    print manip.GetEndEffectorTransform()
    
    sols=manip.FindIKSolutions(T,False)
    for i,sol in enumerate(sols):
        robot.SetJointValues(sol)
        Tnew = manip.GetEndEffectorTransform()
        if sum((Tnew-T)**2) > 0.0001:
            print i
            break
        
def debug_ik():
    env = Environment()
    env.Reset()
    robot = env.ReadRobotXMLFile('robots/man1.robot.xml')
    env.AddRobot(robot)
    manip=robot.SetActiveManipulator('rightarm')
    prob=interfaces.BaseManipulation(robot)
    prob.DebugIK(10)

def test_drillray():
    python inversekinematics.py --robot=/home/rdiankov/downloads/drilling/newdrill.robot.xml --ray4donly --accuracy=1e-5
    from openravepy import *
    import numpy,time
    from openravepy.examples import inversekinematics
    import ikfast
    from ikfast import SolverStoreSolution, SolverSequence
    from sympy import *
    env = Environment()
    env.Reset()
    robot = env.ReadRobotXMLFile('drill.robot.xml')
    env.AddRobot(robot)
    manip = robot.SetActiveManipulator('vision')
    ikmodel = inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)
    #self.generate()
    rawbasedir = manip.GetDirection()
    rawbasepos = manip.GetGraspTransform()[0:3,3]
    def solveFullIK_Ray4D(*args,**kwargs):
        kwargs['rawbasedir'] = rawbasedir
        kwargs['rawbasepos'] = rawbasepos
        return ikfast.IKFastSolver.solveFullIK_Ray4D(*args,**kwargs)

    solvefn=solveFullIK_Ray4D
    solvejoints = list(manip.GetArmJoints())
    freejoints = []
    sourcefilename = 'temp.cpp'
    self = ikfast.IKFastSolver(kinbody=robot,accuracy=1e-5,precision=None)
    #code = self.generateIkSolver(manip.GetBase().GetIndex(),manip.GetEndEffector().GetIndex(),solvejoints=solvejoints,freeparams=freejoints,usedummyjoints=False,solvefn=solvefn)
    baselink=manip.GetBase().GetIndex()
    eelink=manip.GetEndEffector().GetIndex()
    alljoints = self.getJointsInChain(baselink, eelink)
    usedummyjoints=False
    chain = []
    for joint in alljoints:
        issolvejoint = any([i == joint.jointindex for i in solvejoints])
        joint.isdummy = usedummyjoints and not issolvejoint and not any([i == joint.jointindex for i in freeparams])
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
    code=ikfast_generator_vb.CodeGeneratorVB6().generate(chaintree)
    code=ikfast_generator_cpp.CodeGenerator().generate(chaintree)

def drillray_visionsol():
    solutions=[]
    solutions.append(ikfast.SolverSolution(Symbol('j3'),jointevalsin=[-Symbol('r00')],IsHinge=True))
    j0eq = simplify(eqns[5]*cos(Symbol('j3'))*2-Symbol('r02')*eqns[3]-Symbol('r01')*eqns[4])
    solutions.append(self.solveSinCosEquation(j0eq.subs(varsubsinv),Symbol('j0'))[1])
    solvedvars = self.solveIKTranslation(Positions,Positionsee,rawvars = [Symbol('j1'),Symbol('j2')],otherunsolvedvars=[])
    solutions.append(solvedvars[0][1])
    solvedvars = self.solveIKTranslation(Positions,Positionsee,rawvars = [Symbol('j1')],otherunsolvedvars=[])
    solutions.append(solvedvars[0][1])
    solutions.append(ikfast.SolverStoreSolution (jointvars))

    chaintree=ikfast.SolverIKChainRay4D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], TfirstleftInv[0:3,0:3] * Pee + TfirstleftInv[0:3,3], TfirstleftInv[0:3,0:3] * Dee, solutions,Dfk=Tfirstleft[0:3,0:3]*LinksAccumRightAll[0][0:3,0:3]*Matrix(3,1,basedir.tolist()),Pfk=Tfirstleft*(LinksAccumRightAll[0]*Matrix(4,1,basepos.tolist()+[1.0])))
    code = ikfast.CppGenerator().generate(chaintree)

    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Ray4D)
    freejointinds = []
    output_filename = ikmodel.getfilename()
    sourcefilename = ikmodel.getsourcefilename()
    sourcefilename += '_' + '_'.join(str(ind) for ind in solvejoints)
    if len(freejointinds)>0:
        sourcefilename += '_f'+'_'.join(str(ind) for ind in freejointinds)
    sourcefilename += '.cpp'
    open(sourcefilename,'w').write(code)
def test_6dik():
    #python inversekinematics.py --robot=/home/rdiankov/downloads/SDA10-OpenRave/robots/SDA10-dual.robot.xml
    from openravepy import *
    import numpy,time
    from openravepy.examples import inversekinematics
    import ikfast
    from ikfast import SolverStoreSolution, SolverSequence
    from sympy import *
    env = Environment()
    env.Reset()
    robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
    env.AddRobot(robot)
    manip = robot.SetActiveManipulator('arm')
    ikmodel = inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)

    solvefn=ikfast.IKFastSolver.solveFullIK_6D
    solvejoints = list(manip.GetArmJoints())
    solvejoints.remove(2)
    freeparams=[2]
    sourcefilename = 'temp.cpp'
    self = ikfast.IKFastSolver(kinbody=robot,accuracy=None,precision=None)
    #code = self.generateIkSolver(manip.GetBase().GetIndex(),manip.GetEndEffector().GetIndex(),solvejoints=solvejoints,freeparams=freejoints,usedummyjoints=False,solvefn=solvefn)
    rawbasedir=numpy.dot(manip.GetGraspTransform()[0:3,0:3],manip.GetDirection())
    rawbasepos=manip.GetGraspTransform()[0:3,3]
    baselink=manip.GetBase().GetIndex()
    eelink=manip.GetEndEffector().GetIndex()
    alljoints = self.getJointsInChain(baselink, eelink)
    usedummyjoints=True
    chain = []
    for joint in alljoints:
        issolvejoint = any([i == joint.jointindex for i in solvejoints])
        joint.isdummy = usedummyjoints and not issolvejoint and not any([i == joint.jointindex for i in freeparams])
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
