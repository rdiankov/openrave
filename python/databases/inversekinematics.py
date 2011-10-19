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
"""Manages compiled inverse kinematics files for robots using ikfast.

.. image:: ../../images/databases/inversekinematics.jpg
  :width: 640

`[source] <../_modules/openravepy/databases/inversekinematics.html>`_

**Running:**

.. code-block:: bash

  openrave.py --database inversekinematics

Usage
-----

First set the active manipulator, and then instantiate the InverseKinematicsModel class specifying the iktype and free indices.

.. code-block:: python

  robot.SetActiveManipulator(...)
  ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
      ikmodel.autogenerate()

The supported types are defined by `IkParameterization.Type` and are propagated throughout the entire OpenRAVE framework. All solve methods take in a `IkParameterization` structure, which handles each IK type's serialization, distances metrics, derivatives, and transformation.

To show the manipulator and IK results do:

.. image:: ../../images/databases/inversekinematics_pr2show.jpg
  :height: 200

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.zae --manipname=leftarm --show

Description
-----------

This database generator uses :ref:`ikfast_compiler` to generate optimized and stable analytic
inverse kinematics solvers for any robot manipulator. The manipulator's arm joints are used for
obtaining the joints to solve for. The user can specify the IK type (Rotation, Translation, Full 6D,
Ray 4D, etc), the free joints of the kinematics, and the precision. For example, generating the
right arm 6D IK for the PR2 robot where the free joint is the first joint and the free increment is
0.01 radians is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.zae --manipname=rightarm  --freejoint=r_shoulder_pan_joint --freeinc=0.01

Generating the 3d rotation IK for the stage below is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/rotation_stage.robot.xml --iktype=rotation3d


.. image:: ../../images/databases/inversekinematics_rotation_stage.jpg
  :height: 200

Generating the ray inverse kinematics for the 4 degrees of freedom barrett wam is:

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/barrettwam4.robot.xml --iktype=ray4d
  openrave.py --database inversekinematics --robot=robots/pr2-beta-static.zae --iktype=ray4d --manipname=rightarm_camera

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/neuronics-katana.zae --iktype=translationdirection5d --manipname=arm

The filename that the code is saved in can be retrieved by 

.. code-block:: bash

  openrave.py --database inversekinematics --robot=robots/neuronics-katana.zae --iktype=translationdirection5d --manipname=arm --getfilename

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
calling LoadIKFastSolver command.

`ikfastloader.cpp`_ - example for loading IK in C++.

Reference
---------

* :ref:`ikfast-database` - statistics and performance results of ikfast for many robots

* :ref:`ikfast_compiler` - details on the technology behind IKFast

.. _`ikfastloader.cpp`: ../../coreapihtml/ikfastloader_8cpp-example.html

Command-line
------------

.. shell-block:: openrave.py --database inversekinematics --help

Class Definitions
-----------------
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

if not __openravepy_build_doc__:
    from ..openravepy_int import *
    from ..openravepy_ext import *
    from numpy import *
else:
    from numpy import array

from . import DatabaseGenerator
from ..misc import mkdir_recursive, myrelpath, TSP
import time,platform,shutil,sys
import os.path
from os import getcwd, remove
import distutils
import logging
from distutils import ccompiler
from optparse import OptionParser

log = logging.getLogger(__name__)
format = logging.Formatter('%(name)s: %(message)s')
handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(format)
log.addHandler(handler)
log.setLevel(logging.INFO)

class InverseKinematicsModel(DatabaseGenerator):
    """Generates analytical inverse-kinematics solutions, compiles them into a shared object/DLL, and sets the robot's iksolver. Only generates the models for the robot's active manipulator. To generate IK models for each manipulator in the robot, mulitple InverseKinematicsModel classes have to be created.
    """

    class ArmVisibility:
        """When 'entered' will hide all the non-arm links in order to facilitate visiblity of the gripper"""
        def __init__(self,manip,transparency=1):
            self.manip = manip
            self.robot = self.manip.GetRobot()
            self.hiddengeoms = []
            self.transparency = transparency
        def __enter__(self):
            self.hiddengeoms = []
            with self.robot.GetEnv():
                childlinks = self.robot.GetChain(self.manip.GetBase().GetIndex(),self.manip.GetEndEffector().GetIndex(),returnjoints=False)
                for link in self.robot.GetLinks():
                    if link not in childlinks:
                        for geom in link.GetGeometries():
                            self.hiddengeoms.append((geom,geom.IsDraw(),geom.GetTransparency()))
                            if self.transparency >= 1:
                                geom.SetDraw(False)
                            else:
                                geom.SetDraw(True)
                                geom.SetTransparency(self.transparency)
        def __exit__(self,type,value,traceback):
            with self.robot.GetEnv():
                for geom,isdraw,tr in self.hiddengeoms:
                    geom.SetDraw(isdraw)
                    geom.SetTransparency(tr)

    def __init__(self,robot,iktype=None,forceikfast=False,freeindices=None,freejoints=None):
        """
        :param forceikfast: if set will always force the ikfast solver
        """
        self.ikfastproblem = None
        DatabaseGenerator.__init__(self,robot=robot)
        # check if robot manipulator has no static links (except the base)
        for link in self.robot.GetChain(self.manip.GetBase().GetIndex(),self.manip.GetEndEffector().GetIndex(),returnjoints=False)[1:]:
            for rigidlyattached in link.GetRigidlyAttachedLinks():
                if rigidlyattached.IsStatic():
                    raise ValueError('link %s part of IK chain cannot be declared static'%str(link))
        self.ikfast = __import__('openravepy.ikfast',fromlist=['openravepy'])
        self.ikfast.log.addHandler(handler)
        self.ikfast.log.setLevel(logging.INFO)
        self.ikfastproblem = RaveCreateModule(self.env,'ikfast')
        if self.ikfastproblem is not None:
            self.env.AddModule(self.ikfastproblem,'')
        self.iktype = iktype
        self.iksolver = None
        self.freeinc = None
        if freeindices is not None:
            self.freeindices = freeindices
        elif freejoints is not None:
            self.freeindices = self.getIndicesFromJointNames(freejoints)
        else:
            self.freeindices = None
        if self.freeindices is None:
            self.solveindices = None
        else:
            if not all([ifree in self.manip.GetArmIndices() for ifree in self.freeindices]):
                raise ValueError('not all free indices %s are part of the manipulator indices %s'%(self.freeindices,self.manip.GetArmIndices()))
            
            self.solveindices = [i for i in self.manip.GetArmIndices() if not i in self.freeindices]
        self.forceikfast = forceikfast
        self.ikfeasibility = None # if not None, ik is NOT feasibile and contains the error message
        self.statistics = dict()

    def  __del__(self):
        if self.ikfastproblem is not None:
            try:
                self.env.Remove(self.ikfastproblem)
            except openrave_exception, e:
                log.debug('__del__ %s',e)
    def clone(self,envother):
        clone = DatabaseGenerator.clone(self,envother)
        clone.ikfastproblem = RaveCreateModule(envother,'ikfast')
        if clone.ikfastproblem is not None:
            envother.AddModule(clone.ikfastproblem,'')
        if self.has():
            clone.setrobot(self.freeinc)
        return clone
    def has(self):
        return self.iksolver is not None and self.manip.GetIkSolver() is not None and self.manip.GetIkSolver().Supports(self.iktype)
    def save(self):
        statsfilename=self.getstatsfilename(False)
        mkdir_recursive(os.path.split(statsfilename)[0])
        pickle.dump((self.getversion(),self.statistics,self.ikfeasibility,self.solveindices,self.freeindices,self.freeinc), open(statsfilename, 'w'))
        log.info('inversekinematics generation is done, compiled shared object: %s',self.getfilename(False))

    def load(self,freeinc=None,checkforloaded=True,*args,**kwargs):
        try:
            filename = self.getstatsfilename(True)
            if len(filename) == 0:
                return checkforloaded and self.manip.GetIkSolver() is not None and self.manip.GetIkSolver().Supports(self.iktype) # might have ik already loaded
            
            modelversion,self.statistics,self.ikfeasibility,self.solveindices,self.freeindices,self.freeinc = pickle.load(open(filename, 'r'))
            if modelversion != self.getversion():
                log.warn('version is wrong %s!=%s',modelversion,self.getversion())
                return checkforloaded and self.manip.GetIkSolver() is not None  and self.manip.GetIkSolver().Supports(self.iktype) # might have ik already loaded
                
        except Exception,e:
            print e
            return checkforloaded and self.manip.GetIkSolver() is not None and self.manip.GetIkSolver().Supports(self.iktype) # might have ik already loaded
            
        if self.ikfeasibility is not None:
            # ik is infeasible, but load successfully completed, so return success
            return True
        return self.setrobot(freeinc,*args,**kwargs)
    def getversion(self):
        return int(self.ikfast.__version__)
    def getikname(self):
        return 'ikfast ikfast.%s.%s.%s'%(self.manip.GetKinematicsStructureHash(),str(self.iktype),self.manip.GetName())

    def setrobot(self,freeinc=None):
        """Sets the ik solver on the robot.
        
        freeinc is a list of the delta increments of the freejoint values that can override the default values.
        """
        self.iksolver = None
        if freeinc is not None:
            self.freeinc=freeinc
        if self.freeinc is not None:
            try:
                iksuffix = ' ' + ' '.join(str(f) for f in self.freeinc)
            except TypeError:
                # possibly just a float
                iksuffix = ' %f'%self.freeinc
        else:
            iksuffix = ' ' + ' '.join(str(f) for f in self.getDefaultFreeIncrements(0.1, 0.01))
#         if self.manip.GetIkSolver() is not None:
#             self.iksolver = RaveCreateIkSolver(self.env,self.manip.GetIKSolverName()+iksuffix)
        if self.iksolver is None:
            with self.env:
                ikname = self.getikname()
                iktype = self.ikfastproblem.SendCommand('AddIkLibrary %s %s'%(ikname.split()[1],self.getfilename(True)))
                if iktype is None:
                    if self.forceikfast:
                        return False
                    
                    self.iksolver = RaveCreateIkSolver(self.env,self.manip.GetIkSolver().GetXMLId()+iksuffix) if self.manip.GetIkSolver() is not None else None
                else:
                    if int(self.iktype) != int(iktype):
                        raise ValueError('ik does not match types %s!=%s'%(self.iktype,iktype))
                    
                    self.iksolver = RaveCreateIkSolver(self.env,ikname+iksuffix)
        if self.iksolver is not None and self.iksolver.Supports(self.iktype):
            return self.manip.SetIKSolver(self.iksolver)
        return self.has()
    
    def getDefaultFreeIncrements(self,freeincrot, freeinctrans):
        """Returns a list of delta increments appropriate for each free index
        """
        with self.env:
            values = []
            eetrans = self.manip.GetEndEffectorTransform()[0:3,3]
            armlength = 0
            orderedarmindices = [j for j in self.robot.GetDependencyOrderedJoints() if j.GetJointIndex() in self.manip.GetArmIndices()]
            for j in orderedarmindices[::-1]:
                armlength += sqrt(sum((eetrans-j.GetAnchor())**2))
                eetrans = j.GetAnchor()
            freeinc = []
            for index in self.freeindices:
                joint = self.robot.GetJointFromDOFIndex(index)
                if joint.IsRevolute(index-joint.GetDOFIndex()):
                    freeinc.append(freeincrot)
                elif joint.IsPrismatic(index-joint.GetDOFIndex()):
                    freeinc.append(freeinctrans*armlength)
                else:
                    log.warn('cannot set increment for joint type %s'%joint.GetType())
            return freeinc

    def getDefaultIndices(self):
        """Returns a default set of free indices if the robot has more joints than required by the IK.
        In the futrue, this function will contain heuristics in order to select the best indices candidates.
        """
        if self.iktype is None:
            raise ValueError('ik type is not set')
        
        freeindices = []
        dofexpected = IkParameterization.GetDOF(self.iktype)
        remainingindices = list(self.manip.GetArmIndices())
        if len(remainingindices) > dofexpected:
            for i in range(len(remainingindices) - dofexpected):
                if self.iktype == IkParameterization.Type.Transform6D or self.iktype == IkParameterization.Type.Translation3D or self.iktype == IkParameterization.Type.TranslationLocalGlobal6D:
                    freeindices.append(remainingindices.pop(2))
                elif self.iktype == IkParameterization.Type.Lookat3D:
                    # usually head (rotation joints) are at the end
                    freeindices.append(remainingindices.pop(0))
                elif self.iktype == IkParameterization.Type.TranslationDirection5D:
                    # usually on arms, so remove furthest joints
                    freeindices.append(remainingindices.pop(-1))
                else:
                    # if not 6D, then don't need to worry about intersecting joints
                    # so remove the least important joints
                    freeindices.append(remainingindices.pop(-1))
        solveindices = [i for i in self.manip.GetArmIndices() if not i in freeindices]
        return solveindices,freeindices

    def getfilename(self,read=False):
        if self.iktype is None:
            raise ValueError('ik type is not set')

        if self.solveindices is None or self.freeindices is None:
            solveindices, freeindices = self.getDefaultIndices()
        else:
            solveindices, freeindices = self.solveindices, self.freeindices

        index = -1
        allfreeindices = None
        while True:
            basename = 'ikfast%s.%s.%s.'%(self.getversion(),self.iktype,platform.machine()) + '_'.join(str(ind) for ind in solveindices)
            if len(freeindices)>0:
                basename += '_f'+'_'.join(str(ind) for ind in freeindices)
            filename = RaveFindDatabaseFile(os.path.join('kinematics.'+self.manip.GetKinematicsStructureHash(),ccompiler.new_compiler().shared_object_filename(basename=basename)),read)
            if not read or len(filename) > 0 or self.freeindices is not None:
                break
            # user did not specify a set of freeindices, so the expected behavior is to search for the next loadable one
            index += 1
            dofexpected = IkParameterization.GetDOF(self.iktype)
            if allfreeindices is None:
                allfreeindices = [f for f in self.ikfast.permutations(self.manip.GetArmIndices(),len(self.manip.GetArmIndices())-dofexpected)]
            if index >= len(allfreeindices):
                break
            freeindices = allfreeindices[index]
            solveindices = [i for i in self.manip.GetArmIndices() if not i in freeindices]
        return filename

    def getsourcefilename(self,read=False,outputlang='cpp'):
        if self.iktype is None:
            raise ValueError('ik type is not set')
        
        if self.solveindices is None or self.freeindices is None:
            solveindices, freeindices = self.getDefaultIndices()
        else:
            solveindices, freeindices = self.solveindices, self.freeindices
        basename = 'ikfast%s.%s.'%(self.getversion(),self.iktype)
        basename += '_'.join(str(ind) for ind in solveindices)
        if len(freeindices)>0:
            basename += '_f'+'_'.join(str(ind) for ind in freeindices)
        basename += '.' + outputlang
        return RaveFindDatabaseFile(os.path.join('kinematics.'+self.manip.GetKinematicsStructureHash(),basename),read)

    def getstatsfilename(self,read=False):
        if self.iktype is None:
            raise ValueError('ik type is not set')
        
        if self.solveindices is None or self.freeindices is None:
            solveindices, freeindices = self.getDefaultIndices()
        else:
            solveindices, freeindices = self.solveindices, self.freeindices
            
        index = -1
        while True:
            freeindicesstrings = []
            if len(freeindices)>0:
                for _freeindices in self.ikfast.permutations(freeindices):
                    freeindicesstrings.append(['_f'+'_'.join(str(ind) for ind in _freeindices),_freeindices])
            else:
                freeindicesstrings.append(['',[]])

            for freeindicesstring, fi in freeindicesstrings:
                basename = 'ikfast%s.%s.'%(self.getversion(),self.iktype)
                basename += '_'.join(str(ind) for ind in solveindices)
                basename += freeindicesstring
                basename += '.pp'
                filename = RaveFindDatabaseFile(os.path.join('kinematics.'+self.manip.GetKinematicsStructureHash(),basename),read)
                if not read or len(filename) > 0 or self.freeindices is not None:
                    self.freeindices = fi
                    return filename

            # user did not specify a set of freeindices, so the expected behavior is to search for the next loadable one
            index += 1
            dofexpected = IkParameterization.GetDOF(self.iktype)
            allfreeindices = [f for f in self.ikfast.combinations(self.manip.GetArmIndices(),len(self.manip.GetArmIndices())-dofexpected)]
            if index >= len(allfreeindices):
                break
            freeindices = allfreeindices[index]
            solveindices = [i for i in self.manip.GetArmIndices() if not i in freeindices]
        return filename

    def autogenerate(self,options=None):
        freejoints = None
        iktype = self.iktype
        precision = None
        forceikbuild = True
        outputlang = None
        ipython = None
        freeinc = None
        if options is not None:
            forceikbuild=options.force
            precision=options.precision
            if options.freejoints is not None:
                freejoints=options.freejoints
            outputlang=options.outputlang
            ipython=options.ipython
            if options.freeinc is not None:
                freeinc = [float64(s) for s in options.freeinc]
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
        elif self.manip.GetKinematicsStructureHash()=='ab9d03903279e44bc692e896791bcd05' or self.manip.GetKinematicsStructureHash()=='afe50514bf09aff5f2a84beb078bafbd': # katana
            if iktype==IkParameterization.Type.Translation3D or (iktype==None and self.iktype==IkParameterization.Type.Translation3D):
                freejoints = [self.robot.GetJoints()[ind].GetName() for ind in self.manip.GetArmIndices()[3:]]
            if iktype==None:
                iktype == IkParameterization.Type.TranslationDirection5D
        self.generate(iktype=iktype,freejoints=freejoints,precision=precision,forceikbuild=forceikbuild,outputlang=outputlang,ipython=ipython)
        self.save()

    def getIndicesFromJointNames(self,freejoints):
        freeindices = []
        for jointname in freejoints:
            if type(jointname) == int:
                freeindices.append(jointname)
            else:
                # find the correct joint index
                dofindices = [joint.GetDOFIndex() for joint in self.robot.GetJoints() if joint.GetName()==jointname]
                if len(dofindices) == 0:
                    raise LookupError("cannot find '%s' joint in %s robot"%(jointname,self.robot.GetName()))
                if not dofindices[0] in self.manip.GetArmIndices():
                    raise LookupError("cannot find joint '%s(%d)' in solve joints: %s"%(jointname,dofindices[0],self.manip.GetArmIndices()))
                freeindices.append(dofindices[0])
        return freeindices

    def generate(self,iktype=None,freejoints=None,freeinc=None,freeindices=None,precision=None,forceikbuild=True,outputlang=None,ipython=False):
        if iktype is not None:
            self.iktype = iktype
        if self.iktype is None:
            self.iktype = iktype = IkParameterization.Type.Transform6D
        if self.iktype == IkParameterization.Type.Rotation3D:
            Rbaseraw=self.manip.GetGraspTransform()[0:3,0:3]
            def solveFullIK_Rotation3D(*args,**kwargs):
                kwargs['Rbaseraw'] = Rbaseraw
                return self.ikfast.IKFastSolver.solveFullIK_Rotation3D(*args,**kwargs)
            solvefn=solveFullIK_Rotation3D
        elif self.iktype == IkParameterization.Type.Direction3D:
            rawbasedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            def solveFullIK_Direction3D(*args,**kwargs):
                kwargs['rawbasedir'] = rawbasedir
                return self.ikfast.IKFastSolver.solveFullIK_Direction3D(*args,**kwargs)
            solvefn=solveFullIK_Direction3D
        elif self.iktype == IkParameterization.Type.Ray4D:
            rawbasedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            rawbasepos=self.manip.GetGraspTransform()[0:3,3]
            def solveFullIK_Ray4D(*args,**kwargs):
                kwargs['rawbasedir'] = rawbasedir
                kwargs['rawbasepos'] = rawbasepos
                return self.ikfast.IKFastSolver.solveFullIK_Ray4D(*args,**kwargs)
            solvefn=solveFullIK_Ray4D
        elif self.iktype == IkParameterization.Type.TranslationDirection5D:
            rawbasedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            rawbasepos=self.manip.GetGraspTransform()[0:3,3]
            def solveFullIK_TranslationDirection5D(*args,**kwargs):
                kwargs['rawbasedir'] = rawbasedir
                kwargs['rawbasepos'] = rawbasepos
                return self.ikfast.IKFastSolver.solveFullIK_TranslationDirection5D(*args,**kwargs)
            solvefn=solveFullIK_TranslationDirection5D
        elif self.iktype == IkParameterization.Type.Translation3D:
            rawbasepos=self.manip.GetGraspTransform()[0:3,3]
            def solveFullIK_Translation3D(*args,**kwargs):
                kwargs['rawbasepos'] = rawbasepos
                return self.ikfast.IKFastSolver.solveFullIK_Translation3D(*args,**kwargs)
            solvefn=solveFullIK_Translation3D
        elif self.iktype == IkParameterization.Type.TranslationXY2D:
            rawbasepos=self.manip.GetGraspTransform()[0:2,3]
            def solveFullIK_TranslationXY2D(*args,**kwargs):
                kwargs['rawbasepos'] = rawbasepos
                return self.ikfast.IKFastSolver.solveFullIK_TranslationXY2D(*args,**kwargs)
            solvefn=solveFullIK_TranslationXY2D
        elif self.iktype == IkParameterization.Type.TranslationXYOrientation3D:
            rawbasepos=self.manip.GetGraspTransform()[0:2,3]
            rawangle=normalizeAxisRotation([0,0,1],-self.manip.GetGraspTransform()[0:3,0:3])[0]
            def solveFullIK_TranslationXYOrientation3D(*args,**kwargs):
                kwargs['rawbasepos'] = rawbasepos
                kwargs['rawangle'] = rawangle
                return self.ikfast.IKFastSolver.solveFullIK_TranslationXYOrientation3D(*args,**kwargs)
            solvefn=solveFullIK_TranslationXYOrientation3D
        elif self.iktype == IkParameterization.Type.Transform6D:
            Tgripperraw=self.manip.GetGraspTransform()
            def solveFullIK_6D(*args,**kwargs):
                kwargs['Tgripperraw'] = Tgripperraw
                return self.ikfast.IKFastSolver.solveFullIK_6D(*args,**kwargs)
            solvefn=solveFullIK_6D
        elif self.iktype == IkParameterization.Type.Lookat3D:
            rawbasedir=dot(self.manip.GetGraspTransform()[0:3,0:3],self.manip.GetDirection())
            rawbasepos=self.manip.GetGraspTransform()[0:3,3]
            def solveFullIK_Lookat3D(*args,**kwargs):
                kwargs['rawbasedir'] = rawbasedir
                kwargs['rawbasepos'] = rawbasepos
                return self.ikfast.IKFastSolver.solveFullIK_Lookat3D(*args,**kwargs)
            solvefn=solveFullIK_Lookat3D
        elif self.iktype == IkParameterization.Type.TranslationLocalGlobal6D:
            Tgripperraw=self.manip.GetGraspTransform()
            def solveFullIK_TranslationLocalGlobal6D(*args,**kwargs):
                kwargs['Tgripperraw'] = Tgripperraw
                return self.ikfast.IKFastSolver.solveFullIK_TranslationLocalGlobal6D(*args,**kwargs)
            solvefn=solveFullIK_TranslationLocalGlobal6D
        else:
            raise ValueError('bad type')

        dofexpected = IkParameterization.GetDOF(self.iktype)
        if freeindices is not None:
            self.freeindices = freeindices
        if self.freeindices is None:
            if freejoints is not None:
                self.freeindices = self.getIndicesFromJointNames(freejoints)
            else:
                self.solveindices,self.freeindices = self.getDefaultIndices()
        self.solveindices = [i for i in self.manip.GetArmIndices() if not i in self.freeindices]
        if len(self.solveindices) != dofexpected:
            raise ValueError('number of joints to solve for is not equal to required joints %d!=%d'%(len(self.solveindices),dofexpected))

        if freeinc is not None:
            self.freeinc = freeinc
        if self.freeinc is None:
            self.freeinc = self.getDefaultFreeIncrements(0.1,0.01)
        
        log.info('Generating inverse kinematics for manip %s: %s %s (this might take up to 10 min)',self.manip.GetName(),self.iktype,self.solveindices)
        if outputlang is None:
            outputlang = 'cpp'
        sourcefilename = self.getsourcefilename(False,outputlang)
        statsfilename = self.getstatsfilename(False)
        output_filename = self.getfilename(False)

        if forceikbuild or not os.path.isfile(sourcefilename):
            log.info('creating ik file %s',sourcefilename)
            mkdir_recursive(os.path.split(sourcefilename)[0])
            solver = self.ikfast.IKFastSolver(kinbody=self.robot,kinematicshash=self.manip.GetKinematicsStructureHash(),precision=precision)
            baselink=self.manip.GetBase().GetIndex()
            eelink=self.manip.GetEndEffector().GetIndex()
            if ipython:
                IPython = __import__('IPython')
                ipshell = IPython.Shell.IPShellEmbed(argv='',banner = 'inversekinematics dropping into ipython',exit_msg = 'Leaving Interpreter and continuing solver.')
                ipshell(local_ns=locals())
                reload(self.ikfast) # in case changes occurred
                
            try:
                generationstart = time.time()
                chaintree = solver.generateIkSolver(baselink=baselink,eelink=eelink,freeindices=self.freeindices,solvefn=solvefn)
                self.ikfeasibility = None
                code = solver.writeIkSolver(chaintree,lang=outputlang)
                if len(code) == 0:
                    raise ValueError('failed to generate ik solver for robot %s:%s'%(self.robot.GetName(),self.manip.GetName()))
                
                self.statistics['generationtime'] = time.time()-generationstart
                self.statistics['usinglapack'] = solver.usinglapack
                open(sourcefilename,'w').write(code)
            except self.ikfast.IKFastSolver.IKFeasibilityError, e:
                self.ikfeasibility = str(e)
                print e

        if self.ikfeasibility is None:
            if outputlang == 'cpp':
                # compile the code and create the shared object
                compiler,compile_flags = self.getcompiler()
                try:
                   output_dir = os.path.relpath('/',getcwd())
                except AttributeError: # python 2.5 does not have os.path.relpath
                   output_dir = myrelpath('/',getcwd())

                platformsourcefilename = os.path.splitext(output_filename)[0]+'.cpp' # needed in order to prevent interference with machines with different architectures 
                shutil.copyfile(sourcefilename, platformsourcefilename)
                try:
                    objectfiles = compiler.compile(sources=[platformsourcefilename],macros=[('IKFAST_CLIBRARY',1),('IKFAST_NO_MAIN',1)],extra_postargs=compile_flags,output_dir=output_dir)
                    # because some parts of ikfast require lapack, always try to link with it
                    try:
                        libraries = []
                        if self.statistics.get('usinglapack',True):
                            libraries.append('lapack')
                        compiler.link_shared_object(objectfiles,output_filename=output_filename,libraries=libraries)
                    except distutils.errors.LinkError,e:
                        print e
                        log.info('linking again... (MSVC bug?)')
                        compiler.link_shared_object(objectfiles,output_filename=output_filename,libraries=libraries)
                        
                    if not self.setrobot():
                        return ValueError('failed to generate ik solver')
                finally:
                    # cleanup intermediate files
                    if os.path.isfile(platformsourcefilename):
                        remove(platformsourcefilename)
                    for objectfile in objectfiles:
                        try:
                            remove(objectfile)
                        except:
                            pass
            else:
                log.warn('cannot continue further if outputlang %s is not cpp',outputlang)
        
    def perftiming(self,num):
        with self.env:
            results = self.ikfastproblem.SendCommand('PerfTiming num %d %s'%(num,self.getfilename(True)))
            return [double(s)*1e-9 for s in results.split()]
    def testik(self,iktests):
        """Tests the iksolver.
        """
        if self.ikfeasibility is not None:
            raise ValueError('ik is infeasible')
        
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
            wrongrate = len(solutionresults[0])/numtested
            log.info('success rate: %f, wrong solutions: %f, no solutions: %f, missing solution: %f',float(res[1])/numtested,wrongrate,len(solutionresults[1])/numtested,len(solutionresults[2])/numtested)
        return successrate, wrongrate

    def show(self,delay=0.1,options=None,forceclosure=True):
        if self.env.GetViewer() is None:
            self.env.SetViewer('qtcoin')
        with RobotStateSaver(self.robot):
            with self.ArmVisibility(self.manip,0.95):
                time.sleep(3) # let viewer load
                self.setrobot(0.05)
                while True:
                    with self.env:
                        lower,upper = self.robot.GetDOFLimits(self.manip.GetArmIndices())
                        self.robot.SetDOFValues(lower+random.rand(len(lower))*(upper-lower),self.manip.GetArmIndices())
                        ikparam = self.manip.GetIkParameterization(self.iktype)
                        sols = self.manip.FindIKSolutions(ikparam,IkFilterOptions.CheckEnvCollisions)
                        weights = self.robot.GetDOFWeights(self.manip.GetArmIndices())
                        print 'found %d solutions'%len(sols)
                        sols = TSP(sols,lambda x,y: sum(weights*(x-y)**2))
                        # find shortest route
                        for sol in sols:
                            self.robot.SetDOFValues(sol,self.manip.GetArmIndices())
                            self.env.UpdatePublishedBodies()
                            time.sleep(delay)

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
    def CreateOptionParser():
        parser = DatabaseGenerator.CreateOptionParser()
        parser.description='Uses ikfast to compute the closed-form inverse kinematics equations of a robot manipulator, generates a C++ file, and compiles this file into a shared object which can then be loaded by OpenRAVE.'
        parser.usage='openrave.py --database inversekinematics [options]'
        parser.add_option('--freejoint', action='append', type='string', dest='freejoints',default=None,
                          help='Optional joint name specifying a free parameter of the manipulator. The value of a free joint is known at runtime, but not known at IK generation time. If nothing specified, assumes all joints not solving for are free parameters. Can be specified multiple times for multiple free parameters.')
        parser.add_option('--precision', action='store', type='int', dest='precision',default=8,
                          help='The precision to compute the inverse kinematics in, (default=%default).')
        parser.add_option('--usecached', action='store_false', dest='force',default=True,
                          help='If set, will always try to use the cached ik c++ file, instead of generating a new one.')
        parser.add_option('--freeinc', action='append', type='float', dest='freeinc',default=None,
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
        robotatts={}
        if not options.show:
            robotatts = {'skipgeometry':'1'}
        model = DatabaseGenerator.RunFromParser(Model=Model,parser=parser,robotatts=robotatts,args=args,**kwargs)
        if options.iktests is not None or options.perftiming is not None:
            log.info('testing the success rate of robot %s ',options.robot)
            env = Environment()
            try:
                robot = env.ReadRobotXMLFile(options.robot,{'skipgeometry':'1'})
                env.AddRobot(robot)
                if options.manipname is not None:
                    robot.SetActiveManipulator(options.manipname)
                ikmodel = InverseKinematicsModel(robot,iktype=model.iktype,forceikfast=True,freeindices=model.freeindices)
                if not ikmodel.load():
                    raise ValueError('failed to load ik')
                if options.iktests is not None:
                    successrate = ikmodel.testik(iktests=options.iktests)
                elif options.perftiming:
                    results = array(ikmodel.perftiming(num=options.perftiming))
                    log.info('mean: %fs, median: %fs, min: %fs, max: %fs', mean(results),median(results),min(results),max(results))
            finally:
                env.Destroy()
                RaveDestroy()

def run(*args,**kwargs):
    """Command-line execution of the example. ``args`` specifies a list of the arguments to the script.
    """
    InverseKinematicsModel.RunFromParser(*args,**kwargs)
