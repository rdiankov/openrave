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
from __future__ import with_statement # for python 2.5

from openravepy import *
from openravepy.ikfast import IKFastSolver
from numpy import *
import time,pickle,platform
import distutils
from distutils import ccompiler
from optparse import OptionParser

class InverseKinematicsModel(OpenRAVEModel):
    Type_6D=0
    Type_Rotation3D=1
    Type_Direction3D=2
    Type_Translation3D=3
    def __init__(self,robot,type=Type_6D):
        OpenRAVEModel.__init__(self,robot=robot)
        self.type = type
        if self.type == self.Type_Rotation3D:
            self.dofexpected = 3
        elif self.type == self.Type_Direction3D:
            self.dofexpected = 2
        elif self.type == self.Type_Translation3D:
            self.dofexpected = 3
        elif self.type == type == self.Type_6D:
            self.dofexpected = 6
        else:
            raise ValueError('bad type')
        self.iksolver = None

    def has(self):
        return self.iksolver is not None and self.manip.HasIKSolver()

    def load(self):
        self.iksolver = None
        if self.manip.HasIKSolver():
            self.iksolver = self.env.CreateIkSolver(self.manip.GetIKSolverName()) if self.manip.HasIKSolver() else None
        if self.iksolver is None:
            with self.env:
                ikfastproblem = [p for p in self.env.GetLoadedProblems() if p.GetXMLId() == 'IKFast'][0]
                ikname = 'ikfast.%s.%s'%(self.robot.GetRobotStructureHash(),self.manip.GetName())
                if ikfastproblem.SendCommand('AddIkLibrary %s %s'%(ikname,self.getfilename())) is None:
                    return False
                self.iksolver = self.env.CreateIkSolver(ikname)
                if self.iksolver is not None:
                    self.manip.SetIKSolver(self.iksolver)
                    if not self.manip.InitIKSolver():
                        return False
        return self.has()

    def save(self):
        pass # already saved as a lib

    def getfilename(self):
        basename = 'ikfast.' + self.manip.GetName()
        if self.type == self.Type_Rotation3D:
            sourcefilename += 'r3d'
        elif self.type == self.Type_Direction3D:
            sourcefilename += 'd2d'
        elif self.type == self.Type_Translation3D:
            basename += 't3d'
        elif self.type == self.Type_6D:
            basename += '6d'
        else:
            raise ValueError('bad type')
        return ccompiler.new_compiler().shared_object_filename(basename=basename,output_dir=OpenRAVEModel.getfilename(self))

    def generateFromOptions(self,options):
        type = self.Type_6D
        if options.rotation3donly:
            type = self.Type_Rotation3D
        if options.rotation2donly:
            type = self.Type_Direction3D
        if options.translation3donly:
            type = self.Type_Translation3D
        return self.generate(freejoints=options.freejoints,usedummyjoints=options.usedummyjoints,type=type)

    def generate(self,freejoints=None,usedummyjoints=False,type=None):
        if type is not None:
            self.type = type
        output_filename = self.getfilename()
        sourcefilename = os.path.splitext(output_filename)[0]
        if self.type == self.Type_Rotation3D:
            solvefn=IKFastSolver.solveFullIK_Rotation3D
        elif self.type == self.Type_Direction3D:
            solvefn=IKFastSolver.solveFullIK_Direction3D
        elif self.type == self.Type_Translation3D:
            solvefn=IKFastSolver.solveFullIK_Translation3D
        elif self.type == self.Type_6D:
            solvefn=IKFastSolver.solveFullIK_6D

        solvejoints = self.manip.GetArmJoints()
        if freejoints is not None:
            for jointname in freejoints:
                solvejoints.remove(jointname)
        elif len(solvejoints) > self.dofexpected:
            freejoints = []
            for i in range(len(solvejoints) - self.dofexpected):
                if self.dofexpected == 6:
                    freejoints.append(solvejoints.pop(2))
                else:
                    freejoints.append(solvejoints.pop(0))
        else:
            freejoints = []
        if not len(solvejoints) == self.dofexpected:
            raise ValueError('Need %d solve joints, got: %d'%(self.dofexpected, len(solvejoints)))

        sourcefilename += '_' + '_'.join(str(ind) for ind in solvejoints)
        if len(freejoints)>0:
            sourcefilename += '_f'+'_'.join(str(ind) for ind in freejoints)
        sourcefilename += '.cpp'
        if not os.path.isfile(sourcefilename):
            print 'generating inverse kinematics file %s'%sourcefilename
            mkdir_recursive(OpenRAVEModel.getfilename(self))
            solver = IKFastSolver(kinbody=self.robot)
            code = solver.generateIkSolver(self.manip.GetBase().GetIndex(),self.manip.GetEndEffector().GetIndex(),solvejoints=solvejoints,freeparams=freejoints,usedummyjoints=usedummyjoints,solvefn=solvefn)
            if len(code) == 0:
                raise ValueError('failed to generate ik solver for robot %s:%s'%(self.roobt.GetName(),self.manip.GetName()))
            open(sourcefilename,'w').write(code)

        # compile the code and create the shared object
        compiler,optimization_options = self.getcompiler()
        objectfiles = compiler.compile(sources=[sourcefilename],macros=[('IKFAST_CLIBRARY',1)],extra_postargs=optimization_options,output_dir=os.path.relpath('/',os.getcwd()))
        compiler.link_shared_object(objectfiles,output_filename=output_filename)
        if not self.load():
            return ValueError('failed to generate ik solver')
    def autogenerate(self):
        if self.robot.GetRobotStructureHash() == '409764e862c254605cafb9de013eb531' and self.manip.GetName() == 'arm':
            self.generate(freejoints=[self.robot.GetJoint('Shoulder_Roll').GetJointIndex()])
        else:
            self.generate()

    @staticmethod
    def getcompiler():
        compiler = ccompiler.new_compiler()
        optimization_options = []
        if compiler.compiler_type == 'msvc':
            optimization_options.append('/Ox')
            try:
                # make sure it is correct version!
                cname,cver = openravepyCompilerVersion().split()
                if cname == 'msvc':
                    majorVersion = int(cver)/100-6
                    minorVersion = mod(int(cver),100)/10.0
                    if abs(compiler._MSVCCompiler__version - majorVersion+minorVersion) > 0.001:
                        # not the same version, look for a different compiler
                        distutils.msvc9compiler.VERSION = majorVersion + minorVersion
                        newcompiler = ccompiler.new_compiler()
                        if newcompiler is not None:
                            compiler = newcompiler
            except:
                pass
        else:
            compiler.add_library('stdc++')
            if compiler.compiler_type == 'unix':
                optimization_options.append('-O3')
        return compiler,optimization_options

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.description='Computes the closed-form inverse kinematics equations of a robot manipulator, generates a C++ file, and compiles this file into a shared object which can then be loaded by OpenRAVE'
        parser.add_option('--freejoint', action='append', type='int', dest='freejoints',default=[],
                          help='Optional joint index specifying a free parameter of the manipulator. If not specified, assumes all joints not solving for are free parameters. Can be specified multiple times for multiple free parameters.')
        parser.add_option('--rotation3donly', action='store_true', dest='rotation3donly',default=False,
                          help='If true, need to specify only 3 solve joints and will solve for a target rotation')
        parser.add_option('--rotation2donly', action='store_true', dest='rotation2donly',default=False,
                          help='If true, need to specify only 2 solve joints and will solve for a target direction')
        parser.add_option('--translation3donly', action='store_true', dest='translation3donly',default=False,
                          help='If true, need to specify only 3 solve joints and will solve for a target translation')
        parser.add_option('--usedummyjoints', action='store_true',dest='usedummyjoints',default=False,
                          help='Treat the unspecified joints in the kinematic chain as dummy and set them to 0. If not specified, treats all unspecified joints as free parameters.')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = InverseKinematicsModel.CreateOptionParser()
        (options, args) = parser.parse_args()
        Model = lambda robot: InverseKinematicsModel(robot=robot)
        OpenRAVEModel.RunFromParser(Model=Model,parser=parser)

if __name__ == "__main__":
    InverseKinematicsModel.RunFromParser()
