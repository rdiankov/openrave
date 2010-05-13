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
    robot = env.ReadRobotXMLFile('/home/rdiankov/downloads/SDA10-OpenRave/robots/SDA10-dual.robot.xml')
    env.AddRobot(robot)
    manip=robot.SetActiveManipulator('rightarm')
    self = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
    self.load()
    #T=manip.GetEndEffectorTransform()
    Tlocal=matrixFromPose(array([ 0.124762, -0.377634, -0.917424, -0.0126727, -0.0104459, 0.63172, -0.395333]))
    T = dot(manip.GetBase().GetTransform(),Tlocal)
    robot.SetJointValues(zeros(robot.GetDOF()))
    [j.SetJointLimits([-pi],[pi]) for j in robot.GetJoints()]
    values=manip.FindIKSolution(T,False)
    print ' '.join(str(f) for f in Tlocal[0:3,0:4].flatten())
    robot.SetJointValues (values,manip.GetArmJoints())
    print manip.GetEndEffectorTransform()


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
    robot = env.ReadRobotXMLFile('/home/rdiankov/downloads/drilling/newdrill.robot.xml')
    env.AddRobot(robot)
    manip = robot.GetActiveManipulator()
    ikmodel = inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)
    #self.generate()
    basedir = manip.GetDirection()
    basepos = manip.GetGraspTransform()[0:3,3]
    def solveFullIK_Ray4D(*args,**kwargs):
        kwargs['basedir'] = basedir
        kwargs['basepos'] = basepos
        return ikfast.IKFastSolver.solveFullIK_Direction3D(*args,**kwargs)

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

def test_6dik():
    python inversekinematics.py --robot=/home/rdiankov/downloads/SDA10-OpenRave/robots/SDA10-dual.robot.xml
    from openravepy import *
    import numpy,time
    from openravepy.examples import inversekinematics
    import ikfast
    from ikfast import SolverStoreSolution, SolverSequence
    from sympy import *
    env = Environment()
    env.Reset()
    robot = env.ReadRobotXMLFile('examples/test.robot.xml')#robots/pr2static.robot.xml')#/home/rdiankov/downloads/SDA10-OpenRave/robots/SDA10-dual.robot.xml')
    env.AddRobot(robot)
    manip = robot.SetActiveManipulator('arm')
    ikmodel = inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Transform6D)

    solvefn=ikfast.IKFastSolver.solveFullIK_6D
    solvejoints = list(manip.GetArmJoints())
    #solvejoints.remove(35)
    #freejoints = [35]
    #freeparams=[35]
    sourcefilename = 'temp.cpp'
    self = ikfast.IKFastSolver(kinbody=robot,accuracy=None,precision=None)
    #code = self.generateIkSolver(manip.GetBase().GetIndex(),manip.GetEndEffector().GetIndex(),solvejoints=solvejoints,freeparams=freejoints,usedummyjoints=False,solvefn=solvefn)
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
