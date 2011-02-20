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
"""
.. lang-block:: en

  Automatically generate and load inverse kinematics equations for robots.

.. lang-block:: ja

  逆運動学解析解の自動作成

.. image:: ../../images/databases_inversekinematics_wam.png
  :height: 250

**Running the Generator**

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/barrettsegway.robot.xml

**Testing the Solvers**

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/barrettsegway.robot.xml --iktests=100

Usage
-----

Dynamically generate/load the inverse kinematics for a robot's manipulator:

.. code-block:: python

  robot.SetActiveManipulator(...)
  ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
      ikmodel.autogenerate()

Description
-----------

This process allows users to generate OpenRAVE inverse kinematics solvers for any robot
manipulator. The manipulator's arm joints are used for obtaining the joints to solve for. The user
can specify the IK type (Rotation, Translation, Full 6D, Ray 4D, etc), the free joints of the
kinematics, and the precision. For example, generating the right arm 6D IK for the PR2 robot where
the free joint is the first joint and the free increment is 0.01 radians is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.zae --manipname=rightarm  --freejoint=r_shoulder_pan_joint --freeinc=0.01

Generating the 3d rotation IK for the stage below is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/rotation_stage.robot.xml --iktype=rotation3d


.. image:: ../../images/databases_inversekinematics_rotation_stage.jpg
  :height: 200

Generating the ray inverse kinematics for the 4 degrees of freedom barrett wam is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/barrettwam4.robot.xml --iktype=ray4d
  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.zae --iktype=ray4d --manipname=rightarm_camera

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/neuronics-katana.zae --iktype=translationdirection5d --manipname=arm

Testing
-------

Every IK solver should be tested with the robot using ``--iktests=XXX``. However, calling
``inversekinematics`` will always re-generate the IK, even if one already exists. In order to just
run tests, it is possible to specify the ``--usecached`` option to prevent re-generation and
specifically test:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/barrettwam.robot.xml --usecached --iktests=100

This will give the success rate along with information whether the IK gives a wrong results or fails
to find a solution.

If there are a lot of free joints in the IK solver, then their discretization can greatly affect
whether solutions are found or not. In this case, it is advisable to reduce the discretization
threshold by using the ``--freeinc`` option.

Loading from C++
----------------

It is possible to use the auto-generation process through c++ by loading the IKFast problem and
calling LoadIKFastSolver command. Check out the `ikfastloader.cpp`_ example program.

.. _`ikfastloader.cpp`: http://openrave.programmingvision.com/ordocs/en/html/ikfastloader_8cpp-example.html

"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import __build_doc__
if not __build_doc__:
    from openravepy import *
    from numpy import *
else:
    from openravepy import OpenRAVEModel
    from numpy import array

from openravepy import ikfast
import time,platform,shutil,os
import distutils
from distutils import ccompiler
from optparse import OptionParser

class InverseKinematicsModel(OpenRAVEModel):
    """Generates analytical inverse-kinematics solutions, compiles them into a shared object/DLL, and sets the robot's iksolver. Only generates the models for the robot's active manipulator. To generate IK models for each manipulator in the robot, mulitple InverseKinematicsModel classes have to be created."""
    def __init__(self,robot,iktype=None,forceikfast=False):
        """
        :param forceikfast: if set will always force the ikfast solver
        """
        OpenRAVEModel.__init__(self,robot=robot)
        self.iktype = iktype
        self.iksolver = None
        self.freeinc = None
        self.forceikfast = forceikfast
        self.ikfastproblem = RaveCreateProblem(self.env,'ikfast')
        if self.ikfastproblem is not None:
            self.env.LoadProblem(self.ikfastproblem,'')
    def  __del__(self):
        if self.ikfastproblem is not None:
            try:
                self.env.Remove(self.ikfastproblem)
            except openrave_exception, e:
                print '__del__',e
    def clone(self,envother):
        clone = OpenRAVEModel.clone(self,envother)
        clone.ikfastproblem = RaveCreateProblem(envother,'ikfast')
        if clone.ikfastproblem is not None:
            envother.LoadProblem(clone.ikfastproblem,'')
        clone.setrobot(self.freeinc)
        return clone
    def has(self):
        return self.iksolver is not None and self.manip.GetIkSolver() is not None and self.manip.GetIkSolver().Supports(self.iktype)
    def load(self,*args,**kwargs):
        return self.setrobot(*args,**kwargs)
    def getversion(self):
        return int(ikfast.__version__)
    def setrobot(self,freeinc=None):
        self.iksolver = None
        self.freeinc=freeinc
        if freeinc is not None:
            iksuffix = ' %f'%freeinc
        else:
            iksuffix = ''
#         if self.manip.GetIkSolver() is not None:
#             self.iksolver = RaveCreateIkSolver(self.env,self.manip.GetIKSolverName()+iksuffix)
        if self.iksolver is None:
            with self.env:
                ikname = 'ikfast.%s.%s'%(self.manip.GetKinematicsStructureHash(),self.manip.GetName())
                iktype = self.ikfastproblem.SendCommand('AddIkLibrary %s %s'%(ikname,self.getfilename(True)))
                if iktype is None:
                    if self.forceikfast:
                        return False
                    self.iksolver = RaveCreateIkSolver(self.env,self.manip.GetIkSolver().GetXMLId()+iksuffix) if self.manip.GetIkSolver() is not None else None
                else:
                    if int(self.iktype) != int(iktype):
                        raise ValueError('ik does not match types %s!=%s'%(self.iktype,iktype))
                    ikname = 'ikfast ' + ikname
                    self.iksolver = RaveCreateIkSolver(self.env,ikname+iksuffix)
        if self.iksolver is not None and self.iksolver.Supports(self.iktype):
            return self.manip.SetIKSolver(self.iksolver)
        return self.has()
    
    def save(self):
        # already saved as a lib
        print 'inversekinematics generation is done, compiled shared object: %s'%self.getfilename(False)
    
    def getfilename(self,read=False):
        if self.iktype is None:
            raise ValueError('ik type is not set')
        basename = 'ikfast' + str(self.getversion()) + '.' + str(self.iktype) + '.' + platform.machine()
        return RaveFindDatabaseFile(os.path.join('kinematics.'+self.manip.GetKinematicsStructureHash(),ccompiler.new_compiler().shared_object_filename(basename=basename)),read)
    def getsourcefilename(self,read=False):
        if self.iktype is None:
            raise ValueError('ik type is not set')
        return RaveFindDatabaseFile(os.path.join('kinematics.'+self.manip.GetKinematicsStructureHash(),'ikfast' + str(self.getversion()) + '.' + str(self.iktype)),read)
    def autogenerate(self,options=None):
        freejoints = None
        iktype = self.iktype
        usedummyjoints = None
        precision = None
        forceikbuild = True
        outputlang = None
        ipython = None
        if options is not None:
            forceikbuild=options.force
            precision=options.precision
            usedummyjoints=options.usedummyjoints
            if options.freejoints is not None:
                freejoints=options.freejoints
            outputlang=options.outputlang
            ipython=options.ipython
        if self.manip.GetKinematicsStructureHash() == 'f17f58ee53cc9d185c2634e721af7cd3': # wam 4dof
            if iktype is None:
                iktype=IkParameterization.Type.Translation3D
            if iktype == IkParameterization.Type.Translation3D and freejoints is None:
                freejoints = ['Shoulder_Roll']
        elif self.manip.GetKinematicsStructureHash() == 'bfc61bd497e9993b85f1ab511ee7bdbc': # stage
            if iktype is None:
                iktype=IkParameterization.Type.Rotation3D
        elif self.manip.GetKinematicsStructureHash() == 'c363859a2d7a151a22dc1e251d6d8669' or self.manip.GetKinematicsStructureHash() == '12ceb0aaa06143fe305efa6e48faae0b': # pr2
            if iktype == None:
                iktype=IkParameterization.Type.Transform6D
            if freejoints is None:
                # take the torso and roll joint
                freejoints=[self.robot.GetJoints()[self.manip.GetArmIndices()[ind]].GetName() for ind in [0,3]]
        elif self.manip.GetKinematicsStructureHash()=='a1e9aea0dc0fda631ca376c03d500927' or self.manip.GetKinematicsStructureHash()=='ceb6be51bd14f345e22997cc0bca9f2f': # pr2 cameras
            if iktype is None:
                iktype=IkParameterization.Type.Ray4D
            if freejoints is None:
                # take the torso joint
                freejoints=[self.robot.GetJoints()[self.manip.GetArmIndices()[0]].GetName()]
        elif self.manip.GetKinematicsStructureHash()=='ab9d03903279e44bc692e896791bcd05': # katana
            if iktype==IkParameterization.Type.Translation3D or (iktype==None and self.iktype==IkParameterization.Type.Translation3D):
                freejoints = [self.robot.GetJoints()[ind].GetName() for ind in self.manip.GetArmIndices()[3:]]
        self.generate(iktype=iktype,freejoints=freejoints,usedummyjoints=usedummyjoints,precision=precision,forceikbuild=forceikbuild,outputlang=outputlang,ipython=ipython)
        self.save()

    def generate(self,iktype=None,freejoints=None,usedummyjoints=False,precision=None,forceikbuild=True,outputlang=None,ipython=False):
        if iktype is not None:
            self.iktype = iktype
        if self.iktype is None:
            self.iktype = iktype = IkParameterization.Type.Transform6D
        output_filename = self.getfilename(False)
        sourcefilename = self.getsourcefilename(False)
        if self.iktype == IkParameterization.Type.Rotation3D:
            Rbaseraw=self.manip.GetGraspTransform()[0:3,0:3]
            def solveFullIK_Rotation3D(*args,**kwargs):
                kwargs['Rbaseraw'] = Rbaseraw
                return ikfast.IKFastSolver.solveFullIK_Rotation3D(*args,**kwargs)
            solvefn=solveFullIK_Rotation3D
        elif self.iktype == IkParameterization.Type.Direction3D:
            rawbasedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            def solveFullIK_Direction3D(*args,**kwargs):
                kwargs['rawbasedir'] = rawbasedir
                return ikfast.IKFastSolver.solveFullIK_Direction3D(*args,**kwargs)
            solvefn=solveFullIK_Direction3D
        elif self.iktype == IkParameterization.Type.Ray4D:
            rawbasedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            rawbasepos=self.manip.GetGraspTransform()[0:3,3]
            def solveFullIK_Ray4D(*args,**kwargs):
                kwargs['rawbasedir'] = rawbasedir
                kwargs['rawbasepos'] = rawbasepos
                return ikfast.IKFastSolver.solveFullIK_Ray4D(*args,**kwargs)
            solvefn=solveFullIK_Ray4D
        elif self.iktype == IkParameterization.Type.TranslationDirection5D:
            rawbasedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            rawbasepos=self.manip.GetGraspTransform()[0:3,3]
            def solveFullIK_TranslationDirection5D(*args,**kwargs):
                kwargs['rawbasedir'] = rawbasedir
                kwargs['rawbasepos'] = rawbasepos
                return ikfast.IKFastSolver.solveFullIK_TranslationDirection5D(*args,**kwargs)
            solvefn=solveFullIK_TranslationDirection5D
        elif self.iktype == IkParameterization.Type.Translation3D:
            rawbasepos=self.manip.GetGraspTransform()[0:3,3]
            def solveFullIK_Translation3D(*args,**kwargs):
                kwargs['rawbasepos'] = rawbasepos
                return ikfast.IKFastSolver.solveFullIK_Translation3D(*args,**kwargs)
            solvefn=solveFullIK_Translation3D
        elif self.iktype == IkParameterization.Type.Transform6D:
            Tgripperraw=self.manip.GetGraspTransform()
            def solveFullIK_6D(*args,**kwargs):
                kwargs['Tgripperraw'] = Tgripperraw
                return ikfast.IKFastSolver.solveFullIK_6D(*args,**kwargs)
            solvefn=solveFullIK_6D
        elif self.iktype == IkParameterization.Type.Lookat3D:
            rawbasedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            rawbasepos=self.manip.GetGraspTransform()[0:3,3]
            def solveFullIK_Lookat3D(*args,**kwargs):
                kwargs['rawbasedir'] = rawbasedir
                kwargs['rawbasepos'] = rawbasepos
                return ikfast.IKFastSolver.solveFullIK_Lookat3D(*args,**kwargs)
            solvefn=solveFullIK_Lookat3D
        else:
            raise ValueError('bad type')
        solvejoints = list(self.manip.GetArmIndices())
        freejointinds = []
        if freejoints is not None:
            for jointname in freejoints:
                if type(jointname) == int:
                    freejointinds.append(jointname)
                    solvejoints.remove(jointname)
                else:
                    # find the correct joint index
                    jointindices = [joint.GetJointIndex() for joint in self.robot.GetJoints() if joint.GetName()==jointname]
                    if len(jointindices) == 0:
                        raise LookupError("cannot find '%s' joint in %s robot"%(jointname,self.robot.GetName()))
                    if not jointindices[0] in solvejoints:
                        raise LookupError("cannot find joint '%s(%d)' in solve joints: %s"%(jointname,jointindices[0],str(solvejoints)))
                    freejointinds.append(jointindices[0])
                    solvejoints.remove(jointindices[0])
        print 'Generating inverse kinematics for manip',self.manip.GetName(),':',self.iktype,solvejoints,'(this might take ~5 min)'
        dofexpected = IkParameterization.GetDOF(self.iktype)
        if len(solvejoints) > dofexpected:
            for i in range(len(solvejoints) - dofexpected):
                if self.iktype == IkParameterization.Type.Transform6D or self.iktype == IkParameterization.Type.Translation3D:
                    freejointinds.append(solvejoints.pop(2))
                elif self.iktype == IkParameterization.Type.Lookat3D:
                    # usually head (rotation joints) are at the end
                    freejointinds.append(solvejoints.pop(0))
                elif self.iktype == IkParameterization.Type.TranslationDirection5D:
                    # usually on arms, so remove furthest joints
                    freejointinds.append(solvejoints.pop(-1))
                else:
                    # if not 6D, then don't need to worry about intersecting joints
                    # so remove the least important joints
                    freejointinds.append(solvejoints.pop(-1))
        
        if not len(solvejoints) == dofexpected:
            raise ValueError('Need %d solve joints, got: %d'%(dofexpected, len(solvejoints)))
        
        sourcefilename += '_' + '_'.join(str(ind) for ind in solvejoints)
        if len(freejointinds)>0:
            sourcefilename += '_f'+'_'.join(str(ind) for ind in freejointinds)
        if outputlang is None:
            outputlang = 'cpp'
        sourcefilename += '.' + outputlang

        if forceikbuild or not os.path.isfile(sourcefilename):
            print 'creating ik file %s'%sourcefilename
            mkdir_recursive(os.path.split(sourcefilename)[0])
            solver = ikfast.IKFastSolver(kinbody=self.robot,kinematicshash=self.manip.GetKinematicsStructureHash(),precision=precision)
            baselink=self.manip.GetBase().GetIndex()
            eelink=self.manip.GetEndEffector().GetIndex()
            if ipython:
                IPython = __import__('IPython')
                ipshell = IPython.Shell.IPShellEmbed(argv='',banner = 'inversekinematics dropping into ipython',exit_msg = 'Leaving Interpreter and continuing solver.')
                ipshell(local_ns=locals())
                reload(ikfast) # in case changes occurred
            chaintree = solver.generateIkSolver(baselink=baselink,eelink=eelink,freejointinds=freejointinds,solvefn=solvefn)
            code = solver.writeIkSolver(chaintree,lang=outputlang)
            if len(code) == 0:
                raise ValueError('failed to generate ik solver for robot %s:%s'%(self.robot.GetName(),self.manip.GetName()))
            open(sourcefilename,'w').write(code)
            if outputlang != 'cpp':
                print 'cannot continue further if outputlang is not cpp'
                sys.exit(0)

        # compile the code and create the shared object
        compiler,compile_flags = self.getcompiler()
        try:
           output_dir = os.path.relpath('/',os.getcwd())
        except AttributeError: # python 2.5 does not have os.path.relpath
           output_dir = self.myrelpath('/',os.getcwd())

        platformsourcefilename = os.path.splitext(output_filename)[0]+'.cpp' # needed in order to prevent interference with machines with different architectures 
        shutil.copyfile(sourcefilename, platformsourcefilename)
        try:
            objectfiles = compiler.compile(sources=[platformsourcefilename],macros=[('IKFAST_CLIBRARY',1),('IKFAST_NO_MAIN',1)],extra_postargs=compile_flags,output_dir=output_dir)
            # because some parts of ikfast require lapack, always try to link with it
            try:
                compiler.link_shared_object(objectfiles,output_filename=output_filename,libraries=['lapack'])
            except distutils.errors.LinkError:
                print 'linking without lapack...'
                compiler.link_shared_object(objectfiles,output_filename=output_filename)
            if not self.load():
                return ValueError('failed to generate ik solver')
        finally:
            # cleanup intermediate files
            os.remove(platformsourcefilename)
            for objectfile in objectfiles:
                try:
                    os.remove(objectfile)
                except:
                    pass

    def perftiming(self,num):
        with self.env:
            results = self.ikfastproblem.SendCommand('PerfTiming num %d %s'%(num,self.getfilename(True)))
            return [double(s)*1e-6 for s in results.split()]
    def testik(self,iktests):
        """Tests the iksolver.
        """
        with self.robot:
            # set base to identity to avoid complications when reporting errors
            self.robot.SetTransform(dot(linalg.inv(self.manip.GetBase().GetTransform()),self.robot.GetTransform()))
            cmd = 'DebugIK robot %s '%self.robot.GetName()
            if iktests.isdigit():
                cmd += 'numtests %d '%int(iktests)
            else:
                cmd += 'readfile %s '%iktests
            res = self.ikfastproblem.SendCommand(cmd).split()
            numtested = float(res[0])
            successrate = float(res[1])/numtested
            solutionresults = []
            index = 2
            numvalues=1+IkParameterization.GetNumberOfValues(self.iktype)+self.manip.GetIkSolver().GetNumFreeParameters()
            for iresults in range(3):
                num = int(res[index])
                index += 1
                samples = reshape(array([float64(s) for s in res[index:(index+num*numvalues)]]),(num,numvalues))
                solutionresults.append(samples)
                index += num*numvalues
            print 'success rate: %f, wrong solutions: %f, no solutions: %f, missing solution: %f'%(float(res[1])/numtested,len(solutionresults[0])/numtested,len(solutionresults[1])/numtested,len(solutionresults[2])/numtested)
        return successrate

    @staticmethod
    def getcompiler():
        compiler = ccompiler.new_compiler()
        compile_flags = []
        if compiler.compiler_type == 'msvc':
            compile_flags.append('/Ox')
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
                compile_flags.append('-O3')
                compile_flags.append('-fPIC')
        return compiler,compile_flags

    @staticmethod
    def myrelpath(path, start=os.path.curdir):
        """Return a relative version of a path"""
        if not path:
            raise ValueError("no path specified")

        start_list = os.path.abspath(start).split(os.path.sep)
        path_list = os.path.abspath(path).split(os.path.sep)

        # Work out how much of the filepath is shared by start and path.
        i = len(os.path.commonprefix([start_list, path_list]))

        rel_list = [os.path.pardir] * (len(start_list)-i) + path_list[i:]
        if not rel_list:
            return os.path.curdir
        return os.path.join(*rel_list)

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.description='Computes the closed-form inverse kinematics equations of a robot manipulator, generates a C++ file, and compiles this file into a shared object which can then be loaded by OpenRAVE'
        parser.usage='openrave.py --database inversekinematics [options]'
        parser.add_option('--freejoint', action='append', type='string', dest='freejoints',default=None,
                          help='Optional joint name specifying a free parameter of the manipulator. If nothing specified, assumes all joints not solving for are free parameters. Can be specified multiple times for multiple free parameters.')
        parser.add_option('--precision', action='store', type='int', dest='precision',default=8,
                          help='The precision to compute the inverse kinematics in, (default=%default).')
        parser.add_option('--usecached', action='store_false', dest='force',default=True,
                          help='If set, will always try to use the cached ik c++ file, instead of generating a new one.')
        parser.add_option('--usedummyjoints', action='store_true',dest='usedummyjoints',default=False,
                          help='Treat the unspecified joints in the kinematic chain as dummy and set them to 0. If not specified, treats all unspecified joints as free parameters.')
        parser.add_option('--freeinc', action='store', type='float', dest='freeinc',default=None,
                          help='The discretization value of freejoints.')
        parser.add_option('--numiktests','--iktests',action='store',type='string',dest='iktests',default=None,
                          help='Will test the ik solver and return the success rate. IKTESTS can be an integer to specify number of random tests, it can also be a filename to specify the joint values of the manipulator to test. The formst of the filename is #numiktests [dof values]*')
        parser.add_option('--perftiming', action='store',type='int',dest='perftiming',default=None,
                          help='Number of IK calls for measuring the internal ikfast solver.')
        parser.add_option('--outputlang', action='store',type='string',dest='outputlang',default=None,
                          help='If specified, will output the generated code in that language (ie --outputlang=cpp).')
        parser.add_option('--ipython', '-i',action="store_true",dest='ipython',default=False,
                          help='if true will drop into the ipython interpreter right before ikfast is called')
        parser.add_option('--iktype', action='store',type='string',dest='iktype',default=None,
                          help='The ik type to build the solver current types are: %s'%(', '.join(iktype.name for iktype in IkParameterization.Type.values.values())))
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None,args=None,**kwargs):
        if parser is None:
            parser = InverseKinematicsModel.CreateOptionParser()
        (options, leftargs) = parser.parse_args(args=args)
        if options.iktype is not None:
            # cannot use .names due to python 2.5 (or is it boost version?)
            for value,type in IkParameterization.Type.values.iteritems():
                if type.name.lower() == options.iktype.lower():
                    iktype = type
                    break
        else:
            iktype = IkParameterization.Type.Transform6D
        Model = lambda robot: InverseKinematicsModel(robot=robot,iktype=iktype,forceikfast=True)
        OpenRAVEModel.RunFromParser(Model=Model,parser=parser,args=args,**kwargs)
        if options.iktests is not None or options.perftiming is not None:
            print 'testing the success rate of robot ',options.robot
            env = Environment()
            try:
                robot = env.ReadRobotXMLFile(options.robot)
                env.AddRobot(robot)
                if options.manipname is not None:
                    robot.SetActiveManipulator(options.manipname)
                ikmodel = InverseKinematicsModel(robot,iktype=iktype,forceikfast=True)
                if not ikmodel.setrobot(freeinc=options.freeinc):
                    raise ValueError('failed to load ik')
                if options.iktests is not None:
                    successrate = ikmodel.testik(iktests=options.iktests)
                    print 'success rate is: ',successrate
                elif options.perftiming:
                    results = array(ikmodel.perftiming(num=options.perftiming))
                    print 'mean: %fs, median: %fs, min: %fs, max: %fs'%(mean(results),median(results),min(results),max(results))
            finally:
                env.Destroy()
                RaveDestroy()

def run(*args,**kwargs):
    """Executes the inversekinematics database generation,  ``args`` specifies a list of the arguments to the script.
    
    **Help**
    
    .. shell-block:: openrave.py --database inversekinematics --help
    """
    InverseKinematicsModel.RunFromParser(*args,**kwargs)

if __name__ == "__main__":
    run()
