#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
kinematics, and the accuracy. For example, generating the right arm 6D IK for the PR2 robot where
the free joint is the first joint and the free increment is 0.01 radians is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.robot.xml --manipname=rightarm  --freejoint=r_shoulder_pan_joint --freeinc=0.01

Generating the 3d rotation IK for the stage below is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/rotation_stage.robot.xml --rotation3donly


.. image:: ../../images/databases_inversekinematics_rotation_stage.jpg
  :height: 200

Generating the ray inverse kinematics for the 4 degrees of freedom barrett wam is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=drill.robot.xml --ray4donly 

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
            self.env.Remove(self.ikfastproblem)
    def clone(self,envother):
        clone = OpenRAVEModel.clone(self,envother)
        clone.setrobot(self.freeinc)
        return clone
    def has(self):
        return self.iksolver is not None and self.manip.GetIkSolver() is not None
    def load(self,*args,**kwargs):
        return self.setrobot(*args,**kwargs)
    def getversion(self):
        return 15
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
                iktype = self.ikfastproblem.SendCommand('AddIkLibrary %s %s'%(ikname,self.getfilename()))
                if iktype is None:
                    if self.forceikfast:
                        return False
                    self.iksolver = RaveCreateIkSolver(self.env,self.manip.GetIkSolver().GetXMLId()+iksuffix) if self.manip.GetIkSolver() is not None else None
                else:
                    if int(self.iktype) != int(iktype):
                        raise ValueError('ik does not match types %s!=%s'%(self.iktype,iktype))
                    ikname = 'ikfast ' + ikname
                    self.iksolver = RaveCreateIkSolver(self.env,ikname+iksuffix)
        if self.iksolver is not None:
            return self.manip.SetIKSolver(self.iksolver)
        return self.has()
    
    def save(self):
        # already saved as a lib
        print 'inversekinematics generation is done, compiled shared object: %s'%self.getfilename()
    
    def getdir(self):
        return os.path.join(RaveGetHomeDirectory(),'kinematics.'+self.manip.GetKinematicsStructureHash())
    def getfilename(self):
        if self.iktype is None:
            raise ValueError('ik type is not set')
        basename = 'ikfast' + str(self.getversion()) + '.' + str(self.iktype) + '.' + platform.machine()
        return ccompiler.new_compiler().shared_object_filename(basename=basename,output_dir=self.getdir())
    def getsourcefilename(self):
        if self.iktype is None:
            raise ValueError('ik type is not set')
        return os.path.join(self.getdir(),'ikfast' + str(self.getversion()) + '.' + str(self.iktype))
    def autogenerate(self,options=None):
        freejoints = None
        iktype = None
        usedummyjoints = None
        accuracy = None
        precision = None
        forceikbuild = True
        outputlang = None
        if options is not None:
            if options.rotation3donly:
                iktype = IkParameterization.Type.Rotation3D
            if options.direction3donly:
                iktype = IkParameterization.Type.Direction3D
            if options.translation3donly:
                iktype = IkParameterization.Type.Translation3D
            if options.ray4donly:
                iktype = IkParameterization.Type.Ray4D
            forceikbuild=options.force
            precision=options.precision
            accuracy=options.accuracy
            usedummyjoints=options.usedummyjoints
            if options.freejoints is not None:
                freejoints=options.freejoints
            outputlang=options.outputlang
        if self.manip.GetKinematicsStructureHash() == 'b4ff7e5e04780df16a6bda565e714741': # wam 7dof
            if freejoints is None:
                freejoints = ['Shoulder_Roll']
        elif self.manip.GetKinematicsStructureHash() == 'f17f58ee53cc9d185c2634e721af7cd3': # wam 4dof
            if iktype is None:
                iktype=IkParameterization.Type.Translation3D
            if iktype == IkParameterization.Type.Translation3D and freejoints is None:
                freejoints = ['Shoulder_Roll']
        elif self.manip.GetKinematicsStructureHash() == 'bfc61bd497e9993b85f1ab511ee7bdbc': # stage
            if iktype is None:
                iktype=IkParameterization.Type.Rotation3D
        elif self.manip.GetKinematicsStructureHash() == '3d237bf9cd0926ca3151dfca1d0d8936' or self.manip.GetKinematicsStructureHash() == '63aa5661bbae9c2637e1f44660108a4f': # pr2
            iktype=IkParameterization.Type.Transform6D
            if freejoints is None:
                # take the torso and roll joint
                freejoints=[self.robot.GetJoints()[self.manip.GetArmIndices()[ind]].GetName() for ind in [0,3]]
        elif self.manip.GetKinematicsStructureHash()=='0c314a70ee8eadd2c4cb0ff7770893f8' or self.manip.GetKinematicsStructureHash()=='f202b5bed247928f6a730bc6105058b9': # pr2 cameras
            iktype=IkParameterization.Type.Ray4D
        elif self.manip.GetKinematicsStructureHash()=='cc9744737db7b45be2451aedff296c08': # katana
            if iktype==IkParameterization.Type.Translation3D or (iktype==None and self.iktype==IkParameterization.Type.Translation3D):
                freejoints = [self.robot.GetJoints()[ind].GetName() for ind in self.manip.GetArmIndices()[3:]]
        self.generate(iktype=iktype,freejoints=freejoints,usedummyjoints=usedummyjoints,accuracy=accuracy,precision=precision,forceikbuild=forceikbuild,outputlang=outputlang)
        self.save()

    def generate(self,iktype=None,freejoints=None,usedummyjoints=False,accuracy=None,precision=None,forceikbuild=True,outputlang=None):
        if iktype is not None:
            self.iktype = iktype
        if self.iktype is None:
            self.iktype = iktype = IkParameterization.Type.Transform6D
        output_filename = self.getfilename()
        sourcefilename = self.getsourcefilename()
        if self.iktype == IkParameterization.Type.Rotation3D:
            Rbaseraw=self.manip.GetGraspTransform()[0:3,0:3]
            def solveFullIK_Rotation3D(*args,**kwargs):
                kwargs['Rbaseraw'] = Rbaseraw
                return ikfast.IKFastSolver.solveFullIK_Rotation3D(*args,**kwargs)
            solvefn=solveFullIK_Rotation3D
        elif self.iktype == IkParameterization.Type.Direction3D:
            basedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            def solveFullIK_Direction3D(*args,**kwargs):
                kwargs['basedir'] = basedir
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
                    freejointinds.append([joint.GetJointIndex() for joint in self.robot.GetJoints() if joint.GetName()==jointname][0])
                    solvejoints.remove(freejointinds[-1])
        print 'Generating inverse kinematics for manip',self.manip.GetName(),':',self.iktype,solvejoints,'(this might take ~10-30 min)'
        if self.iktype == IkParameterization.Type.Rotation3D:
            self.dofexpected = 3
        elif self.iktype == IkParameterization.Type.Direction3D:
            self.dofexpected = 2
        elif self.iktype == IkParameterization.Type.Translation3D:
            self.dofexpected = 3
        elif self.iktype == IkParameterization.Type.Transform6D:
            self.dofexpected = 6
        elif self.iktype == IkParameterization.Type.Ray4D:
            self.dofexpected = 4
        elif self.iktype == IkParameterization.Type.Lookat3D:
            self.dofexpected = 2
        else:
            raise ValueError('bad type')

        if len(solvejoints) > self.dofexpected:
            for i in range(len(solvejoints) - self.dofexpected):
                if self.iktype == IkParameterization.Type.Transform6D or self.iktype == IkParameterization.Type.Translation3D:
                    freejointinds.append(solvejoints.pop(2))
                elif self.iktype == IkParameterization.Type.Lookat3D:
                    # usually head (rotation joints) are at the end
                    freejointinds.append(solvejoints.pop(0))
                else:
                    # if not 6D, then don't need to worry about intersecting joints
                    # so remove the least important joints
                    freejointinds.append(solvejoints.pop(-1))
        
        if not len(solvejoints) == self.dofexpected:
            raise ValueError('Need %d solve joints, got: %d'%(self.dofexpected, len(solvejoints)))
        
        sourcefilename += '_' + '_'.join(str(ind) for ind in solvejoints)
        if len(freejointinds)>0:
            sourcefilename += '_f'+'_'.join(str(ind) for ind in freejointinds)
        if outputlang is None:
            outputlang = 'cpp'
        sourcefilename += '.' + outputlang
        if forceikbuild or not os.path.isfile(sourcefilename):
            print 'generating inverse kinematics file %s'%sourcefilename
            mkdir_recursive(self.getdir())
            solver = ikfast.IKFastSolver(kinbody=self.robot,accuracy=accuracy,precision=precision)
            code = solver.generateIkSolver(self.manip.GetBase().GetIndex(),self.manip.GetEndEffector().GetIndex(),solvejoints=solvejoints,freeparams=freejointinds,usedummyjoints=usedummyjoints,solvefn=solvefn,lang=outputlang)
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
            compiler.link_shared_object(objectfiles,output_filename=output_filename)
            if not self.load():
                return ValueError('failed to generate ik solver')
        finally:
            # cleanup intermediate files
            os.remove(platformsourcefilename)
            for objectfile in objectfiles:
                os.remove(objectfile)

    def perftiming(self,num):
        with self.env:
            results = self.ikfastproblem.SendCommand('PerfTiming num %d %s'%(num,self.getfilename()))
            return [double(s)*1e-6 for s in results.split()]
    def testik(self,iktests):
        """Tests the iksolver.
        """
        with self.robot:
            self.robot.Enable(False) # removes self-collisions from being considered
            # set base to identity to avoid complications when reporting errors
            self.robot.SetTransform(dot(linalg.inv(self.manip.GetBase().GetTransform()),self.robot.GetTransform()))
            lower,upper = [v[self.manip.GetArmIndices()] for v in self.robot.GetJointLimits()]

            # get the values to test
            success = 0.0
            iktestvalues = []
            if iktests.isdigit():
                numiktests = int(iktests)
                for i in range(numiktests):
                    while True:
                        self.robot.SetJointValues(random.rand()*(upper-lower)+lower,self.manip.GetArmIndices()) # set random values
                        if not self.robot.CheckSelfCollision():
                            break
                    iktestvalues.append(self.robot.GetDOFValues(self.manip.GetArmIndices()))
            else:
                tokens = open(iktests).read().split()
                numiktests = int(tokens[0])
                iktestvalues = [float(tokens[i+1]) for i in range(len(self.manip.GetArmIndices())*numiktests)]
                iktestvalues = reshape(iktestvalues,(numiktests,len(self.manip.GetArmIndices())))
            success = 0.0

            if self.iktype == IkParameterization.Type.Direction3D:
                for orgvalues in iktestvalues:
                    self.robot.SetDOFValues(orgvalues,self.manip.GetArmIndices())
                    targetdir = dot(self.manip.GetEndEffectorTransform()[0:3,0:3],self.manip.GetDirection())
                    self.robot.SetJointValues(random.rand()*(upper-lower)+lower,self.manip.GetArmIndices()) # set random values
                    sol = self.manip.FindIKSolution(IkParameterization(targetdir,self.iktype),0)
                    if sol is not None:
                        self.robot.SetJointValues(sol,self.manip.GetArmIndices())
                        realdir = dot(self.manip.GetEndEffectorTransform()[0:3,0:3],self.manip.GetDirection())
                        if sum((targetdir-realdir)**2) < 1e-7:
                            success += 1
                        else:
                            print 'wrong solution to: ',targetpos, 'returned is: ',realpos
                    else:
                        print 'failed to find: ',targetpos,'solution is: ',orgvalues
                return success/numiktests
            elif self.iktype == IkParameterization.Type.Translation3D:
                for orgvalues in iktestvalues:
                    self.robot.SetDOFValues(orgvalues,self.manip.GetArmIndices())
                    targetpos = self.manip.GetEndEffectorTransform()[0:3,3]
                    #self.robot.SetJointValues(random.rand()*(upper-lower)+lower,self.manip.GetArmIndices()) # set random values
                    sol = self.manip.FindIKSolution(IkParameterization(targetpos,self.iktype),0)
                    if sol is not None:
                        self.robot.SetJointValues(sol,self.manip.GetArmIndices())
                        realpos = self.manip.GetEndEffectorTransform()[0:3,3]
                        if sum((targetpos-realpos)**2) < 1e-7:
                            success += 1
                        else:
                            print 'wrong solution to: ',targetpos, 'returned is: ',realpos
                    else:
                        print 'failed to find: ',targetpos,'solution is: ',orgvalues
                return success/numiktests
            elif self.iktype == IkParameterization.Type.Rotation3D:
                for orgvalues in iktestvalues:
                    self.robot.SetDOFValues(orgvalues,self.manip.GetArmIndices())
                    targetquat = quatFromRotationMatrix(self.manip.GetEndEffectorTransform()[0:3,0:3])
                    self.robot.SetJointValues(random.rand()*(upper-lower)+lower,self.manip.GetArmIndices()) # set random values
                    sol = self.manip.FindIKSolution(IkParameterization(targetquat,self.iktype),0)
                    if sol is not None:
                        self.robot.SetJointValues(sol,self.manip.GetArmIndices())
                        realquat = quatFromRotationMatrix(self.manip.GetEndEffectorTransform()[0:3,0:3])
                        if quatArrayTDist(targetquat,realquat) < 1e-3:
                            success += 1
                        else:
                            print 'wrong solution to: ',targetquat, 'returned is: ',realquat, 'original solution is ',orgvalues,'returned solution is',sol,'error is: ',quatArrayTDist(targetquat,realquat)
                    else:
                        print 'failed to find: ',targetquat,'solution is: ',orgvalues
                return success/numiktests
            elif self.iktype == IkParameterization.Type.Ray4D:
                for orgvalues in iktestvalues:
                    self.robot.SetDOFValues(orgvalues,self.manip.GetArmIndices())
                    targetdir = dot(self.manip.GetEndEffectorTransform()[0:3,0:3],self.manip.GetDirection())
                    targetpos = self.manip.GetEndEffectorTransform()[0:3,3]
                    targetpos += (random.rand()-0.5)*sqrt(sum(targetpos**2))*targetdir
                    #print targetdir[0],targetdir[1],targetdir[2],targetpos[0],0,0,0,targetpos[1],0,0,0,targetpos[2]
                    targetprojpos = targetpos - targetdir*dot(targetdir,targetpos)
                    self.robot.SetJointValues(random.rand()*(upper-lower)+lower,self.manip.GetArmIndices()) # set random values
                    sol = self.manip.FindIKSolution(IkParameterization(Ray(targetpos,targetdir),self.iktype),0)
                    if sol is not None:
                        self.robot.SetJointValues(sol,self.manip.GetArmIndices())
                        realdir = dot(self.manip.GetEndEffectorTransform()[0:3,0:3],self.manip.GetDirection())
                        realpos = self.manip.GetEndEffectorTransform()[0:3,3]
                        realprojpos = realpos - realdir*dot(realdir,realpos)
                        if sum((targetdir-realdir)**2) < 2e-5 and sum((targetprojpos-realprojpos)**2) < 1e-6:
                            success += 1
                        else:
                            print 'wrong solution to: ',targetpos,targetdir, 'returned is: ',realpos,realdir,'wrong sol is:',sol,', org values are:',orgvalues
                            print 'errors direction: ', sum((targetdir-realdir)**2), ' on position: ',sum((targetprojpos-realprojpos)**2)
                    else:
                        print 'failed to find: ',targetpos,targetdir,'solution is: ',orgvalues
                return success/numiktests
            elif self.iktype == IkParameterization.Type.Lookat3D:
                for orgvalues in iktestvalues:
                    self.robot.SetDOFValues(orgvalues,self.manip.GetArmIndices())
                    targetdir = dot(self.manip.GetEndEffectorTransform()[0:3,0:3],self.manip.GetDirection())
                    targetpos = self.manip.GetEndEffectorTransform()[0:3,3] + 10.0*random.rand()*targetdir
                    self.robot.SetJointValues(random.rand()*(upper-lower)+lower,self.manip.GetArmIndices()) # set random values
                    sol = self.manip.FindIKSolution(IkParameterization(targetpos,self.iktype),0)
                    if sol is not None:
                        self.robot.SetJointValues(sol,self.manip.GetArmIndices())
                        realdir = dot(self.manip.GetEndEffectorTransform()[0:3,0:3],self.manip.GetDirection())
                        realpos = self.manip.GetEndEffectorTransform()[0:3,3]
                        if linalg.norm(cross(realdir,realpos-targetpos)) < 7e-5: # relax error a little...
                            success += 1
                        else:
                            print 'wrong solution to: ',repr(targetpos), 'returned is: ',repr(realpos),repr(realdir),'wrong sol is:',repr(sol),', org values are:',repr(orgvalues)
                            print 'errors on position: ',linalg.norm(cross(realdir,realpos-targetpos))
                    else:
                        print 'failed to find: ',targetpos,'solution is: ',orgvalues
                return success/numiktests
            else:
                cmd = 'DebugIK robot %s '%self.robot.GetName()
                if iktests.isdigit():
                    cmd += 'numtests %d '%int(iktests)
                else:
                    cmd += 'readfile %s '%iktests
                successrate = float(self.ikfastproblem.SendCommand(cmd))
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
        parser.add_option('--precision', action='store', type='int', dest='precision',default=15,
                          help='The precision to compute the inverse kinematics in, (default=%default).')
        parser.add_option('--accuracy', action='store', type='float', dest='accuracy',default=1e-12,
                          help='The small number that will be recognized as a zero used to eliminate floating point errors (default=%default).')
        parser.add_option('--usecached', action='store_false', dest='force',default=True,
                          help='If set, will always try to use the cached ik c++ file, instead of generating a new one.')
        parser.add_option('--rotation3donly', action='store_true', dest='rotation3donly',default=False,
                          help='[deprecated] If true, need to specify only 3 solve joints and will solve for a target rotation')
        parser.add_option('--direction3donly', action='store_true', dest='direction3donly',default=False,
                          help='[deprecated] If true, need to specify only 2 solve joints and will solve for a target direction')
        parser.add_option('--translation3donly', action='store_true', dest='translation3donly',default=False,
                          help='[deprecated] If true, need to specify only 3 solve joints and will solve for a target translation')
        parser.add_option('--ray4donly', action='store_true', dest='ray4donly',default=False,
                          help='[deprecated] If true, need to specify only 4 solve joints and will solve for a target ray')
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
            if options.rotation3donly:
                iktype = IkParameterization.Type.Rotation3D
            if options.direction3donly:
                iktype = IkParameterization.Type.Direction3D
            if options.translation3donly:
                iktype = IkParameterization.Type.Translation3D
            if options.ray4donly:
                iktype = IkParameterization.Type.Ray4D
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

def run(*args,**kwargs):
    """Executes the inversekinematics database generation,  ``args`` specifies a list of the arguments to the script.
    
    **Help**
    
    .. shell-block:: openrave.py --database inversekinematics --help
    """
    InverseKinematicsModel.RunFromParser(*args,**kwargs)

if __name__ == "__main__":
    run()
