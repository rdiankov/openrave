#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (Lesser GPL)
#
# Copyright (C) 2009-2010 Rosen Diankov
#
# ikfast is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# ikfast is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
from __future__ import with_statement # for python 2.5

import sys, copy, time, math, datetime
import __builtin__
from optparse import OptionParser
try:
    from openravepy.metaclass import AutoReloader
except:
    class AutoReloader:
        pass

from sympy import *

try:
    import re # for latex cleanup
except ImportError:
    pass

from itertools import izip
try:
    from itertools import combinations
except ImportError:
    def combinations(items,n):
        if n == 0: yield[]
        else:
            for  i in xrange(len(items)):
                for cc in combinations(items[i+1:],n-1):
                    yield [items[i]]+cc

CodeGenerators = {}
try:
    import ikfast_generator_vb
    CodeGenerators['vb'] = ikfast_generator_vb.CodeGenerator
    CodeGenerators['vb6'] = ikfast_generator_vb.CodeGeneratorVB6
    CodeGenerators['vb6special'] = ikfast_generator_vb.CodeGeneratorVB6Special
except ImportError:
    pass
try:
    import ikfast_generator_cpp
    CodeGenerators['cpp'] = ikfast_generator_cpp.CodeGenerator
except ImportError:
    pass

def customcse(exprs,symbols=None):
    return cse(exprs,symbols=symbols)
#     replacements,reduced_exprs = cse(exprs,symbols=symbols)
#     newreplacements = []
#     # look for opany expressions of the order of (x**(1/a))**b, usually computer wants x^(b/a)
#     for r in replacements:
#         if r[1].is_Pow and r[1].exp.is_real and r[1].base.is_Symbol:
#             baseexpr = r[1].base.subs(replacements)
#             if baseexpr.is_Pow and baseexpr.exp.is_real:
#                 newreplacements.append((r[0],baseexpr.base**(r[1].exp*baseexpr.exp)))
#                 continue
#         newreplacements.append(r)
#     return newreplacements,reduced_exprs

class SolverSolution:
    jointname = None
    jointeval = None
    jointevalcos = None
    jointevalsin = None
    AddPiIfNegativeEq = None
    IsHinge = True
    checkforzeros = None

    """Meaning of FeasibleIsZeros:
    If set to false, then solution is feasible only if all of these equations evalute to non-zero.
    If set to true, solution is feasible only if all these equations evaluate to zero.
    """
    FeasibleIsZeros = False
    score = None
    def __init__(self, jointname, jointeval=None,jointevalcos=None,jointevalsin=None,AddPiIfNegativeEq=None,IsHinge=True):
        self.jointname = jointname
        self.jointeval = jointeval
        self.jointevalcos = jointevalcos
        self.jointevalsin = jointevalsin
        self.AddPiIfNegativeEq = AddPiIfNegativeEq
        self.IsHinge=IsHinge
        assert self.checkValidSolution()
    def subs(self,solsubs):
        if self.jointeval is not None:
            self.jointeval = [e.subs(solsubs) for e in self.jointeval]
        if self.jointevalcos is not None:
            self.jointevalcos = [e.subs(solsubs) for e in self.jointevalcos]
        if self.jointevalsin is not None:
            self.jointevalsin = [e.subs(solsubs) for e in self.jointevalsin]
        if self.checkforzeros is not None:
            self.checkforzeros = [e.subs(solsubs) for e in self.checkforzeros]
        if not self.checkValidSolution():
            raise IKFastSolver.CannotSolveError('substitution produced invalid results')
        return self
    def generate(self, generator):
        assert self.checkValidSolution()
        return generator.generateSolution(self)
    def end(self, generator):
        return generator.endSolution(self)
    def checkValidSolution(self):
        valid=True
        if self.jointeval is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointeval])
        if self.jointevalsin is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointevalsin])
        if self.jointevalcos is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointevalcos])
        return valid

class SolverPolynomialRoots:
    """find all roots of the polynomial and plug it into jointeval. poly should be polys.polynomial.Poly
    """
    jointname = None
    poly = None
    jointeval = None
    jointevalcos = None # not used
    jointevalsin = None # not used
    checkforzeros = None
    postcheckforzeros = None # fail if zero
    postcheckfornonzeros = None # fail if nonzero
    postcheckforrange = None # checks that value is within [-1,1]
    thresh = 1e-6
    IsHinge = True
    FeasibleIsZeros = False
    score = None    
    def __init__(self, jointname, poly=None, jointeval=None,IsHinge=True):
        self.poly = poly
        self.jointname=jointname
        self.jointeval = jointeval
        self.IsHinge = IsHinge
    def subs(self,solsubs):
        if self.jointeval is not None:
            self.jointeval = [e.subs(solsubs) for e in self.jointeval]
        if self.checkforzeros is not None:
            self.checkforzeros = [e.subs(solsubs) for e in self.checkforzeros]
        if self.postcheckforzeros is not None:
            self.postcheckforzeros = [e.subs(solsubs) for e in self.postcheckforzeros]
        if self.postcheckfornonzeros is not None:
            self.postcheckfornonzeros = [e.subs(solsubs) for e in self.postcheckfornonzeros]
        if self.postcheckforrange is not None:
            self.postcheckforrange = [e.subs(solsubs) for e in self.postcheckforrange]
        self.poly = self.poly.subs(solsubs)
        assert self.checkValidSolution()
        return self
    def generate(self, generator):
        return generator.generatePolynomialRoots(self)        
    def end(self, generator):
        return generator.endPolynomialRoots(self)
    def checkValidSolution(self):
        valid = IKFastSolver.isValidSolution(self.poly.as_basic())
        if self.jointeval is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointeval])
        return valid

class SolverConicRoots:
    """find all roots of the polynomial and plug it into jointeval. poly should be polys.polynomial.Poly
    """
    jointname = None
    jointeval = None # only one polynomial in cos(jointname) sin(jointname)
    IsHinge = True
    score = None
    checkforzeros = None
    def __init__(self, jointname, jointeval=None,IsHinge=True):
        self.jointname=jointname
        self.jointeval = jointeval
        self.IsHinge = IsHinge
    def subs(self,solsubs):
        if self.jointeval is not None:
            self.jointeval = [e.subs(solsubs) for e in self.jointeval]
        if self.checkforzeros is not None:
            self.checkforzeros = [e.subs(solsubs) for e in self.checkforzeros]
        assert self.checkValidSolution()
        return self
    def generate(self, generator):
        return generator.generateConicRoots(self)
    def end(self, generator):
        return generator.endConicRoots(self)
    def checkValidSolution(self):
        return all([IKFastSolver.isValidSolution(e) for e in self.jointeval])

class SolverConditionedSolution:
    solversolutions = None
    thresh=0.00001
    def __init__(self, solversolutions):
        self.solversolutions = solversolutions
    def subs(self,solsubs):
        for s in self.solversolutions:
            s.subs(solsubs)
        return self
    def generate(self, generator):
        return generator.generateConditionedSolution(self)
    def end(self, generator):
        return generator.endConditionedSolution(self)
    
class SolverBranch:
    jointname = None
    jointeval = None # only used for evaluation, do use these for real solutions
    # list of tuples, first gives expected value of joint, then the code that follows.
    # Last element in list is executed if nothing else is
    jointbranches = None
    def __init__(self, jointname, jointeval, jointbranches):
        self.jointname = jointname
        self.jointeval = jointeval
        self.jointbranches = jointbranches
        assert(jointeval is not None)
    def generate(self, generator):
        return generator.generateBranch(self)
    def end(self, generator):
        return generator.endBranch(self)

class SolverBranchConds:
    jointbranches = None
    thresh = 0.00001
    def __init__(self, jointbranches):
        self.jointbranches = jointbranches
    def generate(self, generator):
        return generator.generateBranchConds(self)
    def end(self, generator):
        return generator.endBranchConds(self)

class SolverCheckZeros:
    jointname = None
    jointcheckeqs = None # only used for evaluation
    zerobranch = None
    nonzerobranch = None
    anycondition=None
    def __init__(self, jointname, jointcheckeqs, zerobranch, nonzerobranch,thresh=0.00001,anycondition=True):
        self.jointname = jointname
        self.jointcheckeqs = jointcheckeqs
        self.zerobranch = zerobranch
        self.nonzerobranch = nonzerobranch
        self.thresh = thresh
        self.anycondition = anycondition
    def generate(self, generator):
        return generator.generateCheckZeros(self)
    def end(self, generator):
        return generator.endCheckZeros(self)

class SolverFreeParameter:
    jointname = None
    jointtree = None
    def __init__(self, jointname, jointtree):
        self.jointname = jointname
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateFreeParameter(self)
    def end(self, generator):
        return generator.endFreeParameter(self)

class SolverIKChainTransform6D:
    solvejointvars = None
    freejointvars = None
    Tee = None
    jointtree = None
    Tfk = None
    def __init__(self, solvejointvars, freejointvars, Tee, jointtree,Tfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Tee = Tee
        self.jointtree = jointtree
        self.Tfk = Tfk
    def generate(self, generator):
        return generator.generateChain(self)
    def end(self, generator):
        return generator.endChain(self)

class SolverIKChainRotation3D:
    solvejointvars = None
    freejointvars = None
    Ree = None
    Rfk = None
    jointtree = None
    def __init__(self, solvejointvars, freejointvars, Ree, jointtree,Rfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Ree = Ree
        self.Rfk=Rfk
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateIKChainRotation3D(self)
    def end(self, generator):
        return generator.endIKChainRotation3D(self)

class SolverIKChainTranslation3D:
    solvejointvars = None
    freejointvars = None
    Pee = None
    jointtree = None
    Pfk = None
    def __init__(self, solvejointvars, freejointvars, Pee, jointtree,Pfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Pee = Pee
        self.jointtree = jointtree
        self.Pfk=Pfk
    def generate(self, generator):
        return generator.generateIKChainTranslation3D(self)
    def end(self, generator):
        return generator.endIKChainTranslation3D(self)

class SolverIKChainDirection3D:
    solvejointvars = None
    freejointvars = None
    Dee = None
    jointtree = None
    Dfk = None
    def __init__(self, solvejointvars, freejointvars, Dee, jointtree,Dfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Dee = Dee
        self.jointtree = jointtree
        self.Dfk=Dfk
    def generate(self, generator):
        return generator.generateIKChainDirection3D(self)
    def end(self, generator):
        return generator.endIKChainDirection3D(self)

class SolverIKChainRay4D:
    solvejointvars = None
    freejointvars = None
    Pee = None
    Dee = None
    jointtree = None
    Pfk = None
    Dfk = None
    def __init__(self, solvejointvars, freejointvars, Pee, Dee, jointtree,Pfk=None,Dfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Pee = Pee
        self.Dee = Dee
        self.jointtree = jointtree
        self.Pfk = Pfk
        self.Dfk = Dfk
    def generate(self, generator):
        return generator.generateIKChainRay4D(self)
    def end(self, generator):
        return generator.endIKChainRay4D(self)

class SolverIKChainLookat3D:
    solvejointvars = None
    freejointvars = None
    Pee = None
    jointtree = None
    Pfk = None
    Dfk = None
    def __init__(self, solvejointvars, freejointvars, Pee, jointtree,Pfk=None,Dfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Pee = Pee
        self.jointtree = jointtree
        self.Pfk=Pfk
        self.Dfk=Dfk
    def generate(self, generator):
        return generator.generateIKChainLookat3D(self)
    def end(self, generator):
        return generator.endIKChainLookat3D(self)

class SolverRotation:
    T = None
    jointtree = None
    functionid=0
    def __init__(self, T, jointtree):
        self.T = T
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateRotation(self)
    def end(self, generator):
        return generator.endRotation(self)

class SolverDirection:
    D = None
    jointtree = None
    def __init__(self, D, jointtree):
        self.D = D
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateDirection(self)
    def end(self, generator):
        return generator.endDirection(self)

class SolverStoreSolution:
    alljointvars = None
    def __init__(self, alljointvars):
        self.alljointvars = alljointvars
    def generate(self, generator):
        return generator.generateStoreSolution(self)
    def end(self, generator):
        return generator.endStoreSolution(self)

class SolverSequence:
    jointtrees = None
    def __init__(self, jointtrees):
        self.jointtrees = jointtrees
    def generate(self, generator):
        return generator.generateSequence(self)
    def end(self, generator):
        return generator.endSequence(self)

class SolverSetJoint:
    jointname = None
    jointvalue = None
    def __init__(self, jointname,jointvalue):
        self.jointname = jointname
        self.jointvalue = jointvalue
    def generate(self, generator):
        return generator.generateSetJoint(self)
    def end(self, generator):
        return generator.endSetJoint(self)

class SolverBreak:
    def generate(self,generator):
        return generator.generateBreak(self)
    def end(self,generator):
        return generator.endBreak(self)

class fmod(core.function.Function):
    nargs = 2
    is_real = True
    is_Function = True

class IKFastSolver(AutoReloader):
    """Solves the analytical inverse kinematics equations.    
    """

    class CannotSolveError(Exception):
        def __init__(self,value):
            self.value=value
        def __str__(self):
            return repr(self.value)

    class Joint:
        __slots__ = ['jointtype','axis','Tleft','Tright','jcoeff','linkcur','linkbase','jointindex','isfreejoint','isdummy']

    class Variable:
        __slots__ = ['var','svar','cvar','tvar']
        def __init__(self, var):
            self.var = var
            self.svar = Symbol("s%s"%var.name)
            self.cvar = Symbol("c%s"%var.name)
            self.tvar = Symbol("t%s"%var.name)

    class DegenerateCases:
        def __init__(self):
            self.handleddegeneratecases = []
        def clone(self):
            clone=IKFastSolver.DegenerateCases()
            clone.handleddegeneratecases = self.handleddegeneratecases[:]
            return clone
        def addcasesconds(self,newconds,currentcases):
            for case in newconds:
                newcases = set(currentcases)
                newcases.add(case)
                assert not self.hascases(newcases)
                self.handleddegeneratecases.append(newcases)
        def addcases(self,currentcases):
            assert not self.hascases(currentcases)
            self.handleddegeneratecases.append(currentcases)
        def gethandledconds(self,currentcases):
            handledconds = []
            for handledcases in self.handleddegeneratecases:
                if len(currentcases)+1==len(handledcases) and currentcases < handledcases:
                    handledconds.append((handledcases - currentcases).pop())
            return handledconds
        def hascases(self,currentcases):
            for handledcases in self.handleddegeneratecases:
                if handledcases == currentcases:
                    return True
            return False

    def __init__(self, robotfile=None,robotfiledata=None,kinbody=None,kinematicshash='',accuracy=None,precision=None):
        self.joints = []
        self.freevarsubs = []
        self.degeneratecases = None
        self.kinematicshash = kinematicshash
        if accuracy is None:
            self.accuracy=1e-12
        else:
            self.accuracy=accuracy
        if precision is None:
            self.precision=15
        else:
            self.precision=precision
        alljoints = []
        if kinbody is not None:
            # this actually requires openravepy to run, but it isn't a good idea to make ikfast dependent on openravepy
            with kinbody.GetEnv():
                for bodyjoint in kinbody.GetJoints()+kinbody.GetPassiveJoints():
                    joint = IKFastSolver.Joint()
                    joint.type = bodyjoint.GetType().name.lower()
                    joint.jointindex = bodyjoint.GetJointIndex()
                    joint.jcoeff = [round(float(x),5) for x in bodyjoint.GetMimicCoeffs()]
                    joint.isfreejoint = False
                    joint.isdummy = bodyjoint.IsStatic()
                    joint.Tright = self.normalizeRotation(Matrix(4,4,[Real(round(float(x),5),30) for x in bodyjoint.GetInternalHierarchyRightTransform().flat]))
                    joint.Tleft = self.normalizeRotation(Matrix(4,4,[Real(round(float(x),5),30) for x in bodyjoint.GetInternalHierarchyLeftTransform().flat]))
                    joint.axis = Matrix(3,1,[Real(round(float(x),4),30) for x in bodyjoint.GetInternalHierarchyAxis(0)])
                    joint.linkcur = bodyjoint.GetSecondAttached().GetIndex()
                    joint.linkbase = bodyjoint.GetFirstAttached().GetIndex()
                    if bodyjoint.GetSecondAttached() and (bodyjoint.GetSecondAttached() == bodyjoint.GetFirstAttached().GetParentLink()):
                        # bodies[0] is the child
                        joint.linkcur,joint.linkbase = joint.linkbase,joint.linkcur
                    alljoints.append(joint)
        else:
            if robotfiledata is not None:
                tokens = robotfiledata.split()
            elif robotfile is not None:
                f=open(robotfile, "r")
                tokens = f.read().split()
            else:
                raise ValueError('no valid way to initialize ikfast solver')
            numdof = int(tokens[0])
            offset = 1
            for i in range(numdof):
                joint = IKFastSolver.Joint()
                joint.type = tokens[offset+0]
                joint.linkcur = int(tokens[offset+1])
                joint.linkbase = int(tokens[offset+2])
                joint.jointindex = int(tokens[offset+3])
                joint.axis = Matrix(3,1,[Real(round(float(x),4),30) for x in tokens[offset+4:offset+7]])
                joint.axis /= sqrt(joint.axis.dot(joint.axis))
                joint.jcoeff = [round(float(x),5) for x in tokens[offset+7:offset+9]]
                joint.Tleft = eye(4)
                joint.Tleft[0:3,0:4] = self.normalizeRotation(Matrix(3,4,[Real(round(float(x),5),30) for x in tokens[offset+9:offset+21]]))
                joint.Tright = eye(4)
                joint.Tright[0:3,0:4] = self.normalizeRotation(Matrix(3,4,[Real(round(float(x),5),30) for x in tokens[offset+21:offset+33]]))
                joint.isfreejoint = False
                joint.isdummy = False
                alljoints.append(joint)
                offset = offset + 33

        # order with respect to joint index
        numjoints = max([joint.jointindex for joint in alljoints])+1
        self.joints = [[] for i in range(numjoints)]
        for joint in alljoints:
            self.joints[joint.jointindex].append(joint)

    def normalizeRotation(self,M):
        right = M[0,0:3]/sqrt(M[0,0:3].dot(M[0,0:3]))
        up = M[1,0:3] - right*right.dot(M[1,0:3])
        up = up/sqrt(up.dot(up))
        M[0,0:3] = right
        M[1,0:3] = up
        M[2,0:3] = right.cross(up)
        return M

    def getJointsInChain(self, baselink, eelink):
        # find a path of joints between baselink and eelink using BFS
        linkqueue = [[baselink,[]]]
        alljoints = []
        while len(linkqueue)>0:
            link = linkqueue.pop(0)
            if link[0] == eelink:
                alljoints = link[1]
                break
            attachedjoints = []
            for jointgroup in self.joints:
                attachedjoints += [joint for joint in jointgroup if joint.linkbase == link[0]]
            for joint in attachedjoints:
                path = link[1]+[joint]
                if len(path) > 1 and path[0] == path[-1]:
                    print "discovered circular in joints"
                    return False
                linkqueue.append([joint.linkcur,path])
        return alljoints

    @staticmethod
    def rodrigues(axis, angle):
        skewsymmetric = Matrix(3, 3, [S.Zero,-axis[2],axis[1],axis[2],S.Zero,-axis[0],-axis[1],axis[0],S.Zero])
        return eye(3) + sin(angle) * skewsymmetric + (S.One-cos(angle))*skewsymmetric*skewsymmetric

    # The first and last matrices returned are always numerical
    def forwardKinematics(self, baselink, eelink):
        return self.forwardKinematicsChain(self.getJointsInChain(baselink, eelink))

#     def eye(self,n):
#         tmp = Matrix(n,n,[Real(0,30)]*n*n)
#         for i in range(tmp.lines):
#             tmp[i,i] = Real(1,30)
#         return tmp

    def IsHinge(self,jointname):
        assert jointname[0]=='j'
        jointindex=self.jointindexmap[int(jointname[1:])]
        return self.joints[jointindex][0].type=='hinge' or self.joints[jointindex][0].type=='revolute'
    # The first and last matrices returned are always numerical
    def forwardKinematicsChain(self, chain):
        Links = []
        Tright = eye(4)
        jointvars = []
        isolvejointvars = []
        ifreejointvars = []
        jointinds = []
        self.jointindexmap = dict()
        jointindexmap_inv = dict()
        for i,joint in enumerate(chain):
            if not joint.isdummy:
                if not joint.jointindex in self.jointindexmap:
                    self.jointindexmap[len(jointvars)] = joint.jointindex
                    jointindexmap_inv[joint.jointindex] = len(jointvars)
                    var = Symbol("j%d"%len(jointvars))
                else:
                    var = Symbol("j%d"%jointindexmap_inv[joint.jointindex])
                Tjoint = eye(4)
                if joint.type == 'hinge' or joint.type == 'revolute':
                    Tjoint[0:3,0:3] = self.rodrigues(joint.axis,joint.jcoeff[0]*var+joint.jcoeff[1])
                elif joint.type == 'slider' or joint.type == 'prismatic':
                    Tjoint[0:3,3] = joint.axis*(joint.jcoeff[0]*var+joint.jcoeff[1])
                else:
                    raise ValueError('failed to process joint type %s'%joint.type)
                
                if i > 0 and chain[i].jointindex==chain[i-1].jointindex:
                    # the joint is the same as the last joint
                    Links[-1] = self.affineSimplify(Links[-1] * Tright * joint.Tleft * Tjoint)
                    Tright = joint.Tright
                else:
                    # the joints are different, so add regularly
                    if joint.isfreejoint:
                        ifreejointvars.append(len(jointvars))
                    else:
                        isolvejointvars.append(len(jointvars))
                    jointvars.append(var)
                    Links.append(Tright * joint.Tleft)
                    jointinds.append(len(Links))
                    Links.append(Tjoint)
                    Tright = joint.Tright
            else:
                Tright = self.affineSimplify(Tright * joint.Tleft * joint.Tright)
        Links.append(Tright)
        
        # before returning the final links, try to push as much translation components
        # outwards to both ends. Sometimes these components can get in the way of detecting
        # intersecting axes

        if len(jointinds) > 0:
            iright = jointinds[-1]
            Ttrans = eye(4)
            Ttrans[0:3,3] = Links[iright-1][0:3,0:3].transpose() * Links[iright-1][0:3,3]
            Trot_with_trans = Ttrans * Links[iright]
            separated_trans = Trot_with_trans[0:3,0:3].transpose() * Trot_with_trans[0:3,3]
            for j in range(0,3):
                if separated_trans[j].has_any_symbols(jointvars[-1]):
                    Ttrans[j,3] = Real(0,30)
                else:
                    Ttrans[j,3] = separated_trans[j]
            Links[iright+1] = Ttrans * Links[iright+1]
            Links[iright-1] = Links[iright-1] * self.affineInverse(Ttrans)
            print "moved translation ",Ttrans[0:3,3].transpose(),"to right end"
        
        if len(jointinds) > 1:
            ileft = jointinds[0]
            separated_trans = Links[ileft][0:3,0:3] * Links[ileft+1][0:3,3]
            Ttrans = eye(4)
            for j in range(0,3):
                if not separated_trans[j].has_any_symbols(jointvars[0]):
                    Ttrans[j,3] = separated_trans[j]
            Links[ileft-1] = Links[ileft-1] * Ttrans
            Links[ileft+1] = self.affineInverse(Ttrans) * Links[ileft+1]
            print "moved translation ",Ttrans[0:3,3].transpose(),"to left end"

        if len(jointinds) > 3: # last 3 axes always have to be intersecting, move the translation of the first axis to the left
            ileft = jointinds[-3]
            separated_trans = Links[ileft][0:3,0:3] * Links[ileft+1][0:3,3]
            Ttrans = eye(4)
            for j in range(0,3):
                if not separated_trans[j].has_any_symbols(jointvars[-3]):
                    Ttrans[j,3] = separated_trans[j]
            Links[ileft-1] = Links[ileft-1] * Ttrans
            Links[ileft+1] = self.affineInverse(Ttrans) * Links[ileft+1]
            print "moved translation on intersecting axis ",Ttrans[0:3,3].transpose(),"to left"

        return Links, jointvars, isolvejointvars, ifreejointvars
        
    def generateIkSolver(self, baselink, eelink, solvejoints, freejointinds, usedummyjoints,solvefn=None,lang=None):
        if solvefn is None:
            solvefn = IKFastSolver.solveFullIK_6D
        alljoints = self.getJointsInChain(baselink, eelink)

        # mark the free joints and form the chain
        chain = []
        for joint in alljoints:
            issolvejoint = any([i == joint.jointindex for i in solvejoints])
            if usedummyjoints and not issolvejoint and not any([i == joint.jointindex for i in freejointinds]):
                joint.isdummy = True
            joint.isfreejoint = not issolvejoint and not joint.isdummy
            chain.append(joint)
        
        return self.generateIkSolverChain(chain,solvefn,lang=lang)
        
    def generateIkSolverChain(self, chain, solvefn,lang=None):
        Tee = eye(4)
        for i in range(0,3):
            for j in range(0,3):
                Tee[i,j] = Symbol("r%d%d"%(i,j))
        Tee[0,3] = Symbol("px")
        Tee[1,3] = Symbol("py")
        Tee[2,3] = Symbol("pz")
        self.degeneratecases = None

        chaintree = solvefn(self,chain, Tee)
        if chaintree is None:
            print "failed to genreate ik solution"
            return ""

        # parse the solver tree
        if lang is None:
            # prioritize c++
            generator = CodeGenerators.get('cpp',CodeGenerators.values()[0])
        return CodeGenerators[lang]().generate(chaintree,kinematicshash=self.kinematicshash)

    @staticmethod
    def affineInverse(affinematrix):
        T = eye(4)
        T[0:3,0:3] = affinematrix[0:3,0:3].transpose()
        T[0:3,3] = -affinematrix[0:3,0:3].transpose() * affinematrix[0:3,3]
        return T

    # deep chopping of tiny numbers due to floating point precision errors
    def chop(self,expr,precision=None,accuracy=None):
        # go through all arguments and chop them
        if precision is None:
            precision = self.precision
        if accuracy is None:
            accuracy = self.accuracy
        if expr.is_Function:
            return expr.func(*[self.chop(arg, precision,accuracy) for arg in expr.args])
        elif expr.is_Mul:
            ret = S.One
            for x in expr.args:
                ret *= self.chop(x, precision, accuracy)
            return ret
        elif expr.is_Pow:
            return Pow(self.chop(expr.base, precision, accuracy), expr.exp)
        elif expr.is_Add:
            # Scan for the terms we need
            ret = S.Zero
            for term in expr.args:
                term = self.chop(term, precision, accuracy)
                ret += term
            return ret
        e = expr.evalf(precision,chop=True)
        if e.is_number:
            try:
                e = Real(e,30) # re-add the precision for future operations
            except TypeError:
                pass # not a number? (like I)
        return e if abs(e) > accuracy else S.Zero

    def countVariables(self,expr,var):
        """Counts number of terms variable appears in"""
        if not expr.is_Add:
            if expr.has_any_symbols(var):
                return 1
            return 0
        
        num = 0
        for term in expr.args:
            if term.has_any_symbols(var):
                num += 1

        return num

    def codeComplexity(self,expr):
        complexity = 1
        if expr.is_Add:
            for term in expr.args:
                complexity += self.codeComplexity(term)
        elif expr.is_Mul:
            for term in expr.args:
                complexity += self.codeComplexity(term)
        elif expr.is_Pow:
            complexity += self.codeComplexity(expr.base)+self.codeComplexity(expr.exp)
        elif expr.is_Function:
            complexity += 1
            for term in expr.args:
                complexity += self.codeComplexity(term)
        return complexity
    def sortComplexity(self,exprs):
        exprs.sort(lambda x, y: self.codeComplexity(x)-self.codeComplexity(y))

#     def checkDivideByZero(self,expr):
#         checkforzero = []                
#         if expr.is_Function or expr.is_Add or expr.is_Mul:
#             for arg in expr.args:
#                 checkforzero += self.checkDivideByZero(arg)
#         elif expr.is_Pow and expr.exp.is_real and expr.exp < 0:
#             checkforzero.append(expr.base)
#         return checkforzero

    def checkForDivideByZero(self,eq):
        try:
            checkforzeros = []
            if eq.is_Function:
                for arg in eq.args:
                    checkforzeros += self.checkForDivideByZero(arg)
                return checkforzeros
            subexprs,reduced_exprs = customcse(eq)
            def checkpow(expr,sexprs):
                if expr.is_Pow:
                    sexprs.append(expr.base)
                    if expr.exp.is_real and expr.exp < 0:
                        base = self.subsExpressions(expr.base,subexprs)
                        if not base.is_number:
                            eq=self.subsExpressions(expr.base,subexprs)
                            if self.isExpressionUnique(checkforzeros,-eq) and self.isExpressionUnique(checkforzeros,eq):
                                checkforzeros.append(eq)

            sexprs = [subexpr[1] for subexpr in subexprs]+reduced_exprs
            while len(sexprs) > 0:
                sexpr = sexprs.pop(0)
                if sexpr.is_Add:
                    for arg in sexpr.args:
                        if arg.is_Mul:
                            for arg2 in arg.args:
                                checkpow(arg2,sexprs)
                        else:
                            checkpow(arg,sexprs)
                elif sexpr.is_Mul:
                    for arg in sexpr.args:
                        checkpow(arg,sexprs)
                else:
                    checkpow(sexpr,sexprs)
            return checkforzeros
        except AssertionError,e:
            print e
            return []

    def solutionComplexity(self,sol,solvedvars,unsolvedvars):
        # for all solutions, check if there is a divide by zero
        sol.checkforzeros = []
        sol.score = 0
        try:
            # multiby by 400 in order to prioritize equations with less solutions
            if sol.jointeval is not None:
                sol.score = 400*len(sol.jointeval)
                for s in sol.jointeval:
                    sol.score += self.codeComplexity(s)
                subexprs = sol.jointeval
            elif sol.jointevalsin is not None:
                sol.score = 400*len(sol.jointevalsin)
                for s in sol.jointevalsin:
                    sol.score += self.codeComplexity(s)
                subexprs = sol.jointevalsin
                #sol.score += 500
            elif sol.jointevalcos is not None:
                sol.score = 400*len(sol.jointevalcos)
                for s in sol.jointevalcos:
                    sol.score += self.codeComplexity(s)
                subexprs = sol.jointevalcos
                #sol.score += 500
            else:
                assert False
            
            def checkpow(expr,sexprs):
                score = 0
                if expr.is_Pow:
                    sexprs.append(expr.base)
                    if expr.base.is_finite is not None and not expr.baseis_finite:
                        return oo # infinity
                    if expr.exp.is_real and expr.exp < 0:
                        #exprbase = self.subsExpressions(expr.base,subexprs)
                        # check if exprbase contains any variables that have already been solved
                        containsjointvar = expr.base.has_any_symbols(*solvedvars)
                        cancheckexpr = not expr.base.has_any_symbols(*unsolvedvars)
                        score += 10000
                        if not cancheckexpr:
                            score += 100000
                        else:
                            if self.isExpressionUnique(sol.checkforzeros,-expr.base) and self.isExpressionUnique(sol.checkforzeros,expr.base):
                                sol.checkforzeros.append(expr.base)
                elif not self.isValidSolution(expr):
                    return oo # infinity
                return score
            
            sexprs = subexprs[:]
            while len(sexprs) > 0:
                sexpr = sexprs.pop(0)
                if sexpr.is_Add:
                    for arg in sexpr.args:
                        if arg.is_Mul:
                            for arg2 in arg.args:
                                sol.score += checkpow(arg2,sexprs)
                        else:
                            sol.score += checkpow(arg,sexprs)
                elif sexpr.is_Mul:
                    for arg in sexpr.args:
                        sol.score += checkpow(arg,sexprs)
                elif sexpr.is_Function:
                    sexprs += sexpr.args
                elif not self.isValidSolution(sexpr):
                    print 'not valid: ',sexpr
                    sol.score = oo # infinity
                else:
                    sol.score += checkpow(sexpr,sexprs)
        except AssertionError, e:
            print e
            sol.score=1e10
        return sol.score

    def affineSimplify(self, T):
        # yes, it is necessary to call self.trigsimp so many times since it gives up too easily
        values = [trigsimp(x.expand()) for x in T]
        for i in range(12):
            if (i%4)<3: # rotation should have bigger accuracy threshold
                values[i] = self.chop(values[i],accuracy=self.accuracy*10.0)
            else: # translation
                values[i] = self.chop(values[i])
        return Matrix(4,4,values)

    def fk(self, chain, joints):
        Tlast = eye(4)
        Tlinks = []
        for i,joint in enumerate(chain):
            R = eye(4)
            value = joints[joint.jointindex]
            if joint.type == 'hinge' or joint.type == 'revolute':
                R[0:3,0:3] = self.rodrigues(joint.axis,joint.jcoeff[0]*value+joint.jcoeff[1])
            elif joint.type == 'slider' or joint.type == 'prismatic':
                R[0:3,3] = joint.axis*(joint.jcoeff[0]*value+joint.jcoeff[1])
            else:
                raise ValueError('undefined joint type %s'%joint.type)
            Tlast = (Tlast * joint.Tleft * R * joint.Tright).evalf()
            if not joint.isdummy:
                Tlinks.append(Tlast)
        return Tlinks

    @staticmethod
    def rotateDirection(sourcedir,targetdir):
        sourcedir /= sqrt(sourcedir.dot(sourcedir))
        targetdir /= sqrt(targetdir.dot(targetdir))
        rottodirection = sourcedir.cross(targetdir)
        fsin = sqrt(rottodirection.dot(rottodirection))
        fcos = sourcedir.dot(targetdir)
        M = eye(4)
        if fsin > 1e-6:
            M[0:3,0:3] = IKFastSolver.rodrigues(rottodirection*(1/fsin),atan2(fsin,fcos))
        elif fcos < 0: # hand is flipped 180, rotate around x axis
            rottodirection = Matrix(3,1,[S.One,S.Zero,S.Zero])
            rottodirection -= sourcedir * sourcedir.dot(rottodirection)
            M[0:3,0:3] = IKFastSolver.rodrigues(rottodirection.normalized(), atan2(fsin, fcos))
        return M

    @staticmethod
    def buildPositionEquations(Links, LinksInv, P, Pee):
        Positions = [Matrix(3,1,[IKFastSolver.customtrigsimp(x) for x in T[0:3,0:3]*P+T[0:3,3]]) for T in Links]
        Positionsee = [Matrix(3,1,[IKFastSolver.customtrigsimp(x) for x in T[0:3,0:3]*Pee+T[0:3,3]]) for T in LinksInv]

        # try to shift all the constants of each Position expression to one side
        for i in range(len(Positions)):
            for j in range(3):
                p = Positions[i][j]
                pee = Positionsee[i][j]
                pconstterm = None
                peeconstterm = None
                if p.is_Add:
                    pconstterm = [term for term in p.args if term.is_number]
                elif p.is_number:
                    pconstterm = [p]
                else:
                    continue
                if pee.is_Add:
                    peeconstterm = [term for term in pee.args if term.is_number]
                elif pee.is_number:
                    peeconstterm = [pee]
                else:
                    continue
                if len(pconstterm) > 0 and len(peeconstterm) > 0:
                    # shift it to the one that has the least terms
                    for term in peeconstterm if len(p.args) < len(pee.args) else pconstterm:
                        Positions[i][j] -= term
                        Positionsee[i][j] -= term

        # remove similar equations
        i = 1
        while i < len(Positions):
            if all([Positions[i][j]-Positions[i-1][j] == S.Zero and Positionsee[i][j]-Positionsee[i-1][j] == S.Zero for j in range(3)]):
                Positions.pop(i)
                Positionsee.pop(i)
            else:
                i += 1
        return Positions,Positionsee

    def buildEquationsFromPositions(self,Positions,Positionsee,uselength=True):
        AllEquations = []
        for i in range(len(Positions)):
            for j in range(3):
                e = Positions[i][j] - Positionsee[i][j]
                if self.codeComplexity(e) < 1500:
                    e = self.customtrigsimp(e)

                if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                    AllEquations.append(e)
            if uselength:
                p2 = (Positions[i][0]**2+Positions[i][1]**2+Positions[i][2]**2).expand()
                pe2 = (Positionsee[i][0]**2+Positionsee[i][1]**2+Positionsee[i][2]**2).expand()
                if self.codeComplexity(p2) < 1500 and self.codeComplexity(pe2) < 1500:
                    # sympy's trigsimp/customtrigsimp give up too easily
                    e = self.customtrigsimp(self.customtrigsimp(self.customtrigsimp(self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp(p2)).expand())) - self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp(pe2)).expand())))))
                    if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                        AllEquations.append(e)
                else:
                    print 'length equations too big, skipping...',self.codeComplexity(p2),self.codeComplexity(pe2)
        self.sortComplexity(AllEquations)
        return AllEquations

    def solveFullIK_Direction3D(self,chain,Tee,rawbasedir=Matrix(3,1,[S.Zero,S.Zero,S.One])):
        """basedir needs to be filled with a 3elemtn vector of the initial direction to control"""
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        LinksInv = [self.affineInverse(link) for link in Links]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]
        
        if not len(solvejointvars) == 2:
            raise ValueError('solve joints needs to be 2')

        basedir = Matrix(3,1,rawbasedir.tolist())
        basedir /= sqrt(basedir[0]*basedir[0]+basedir[1]*basedir[1]+basedir[2]*basedir[2]).evalf()

        # when solving equations, convert all free variables to constants
        self.freevarsubs = []
        for freevar in freejointvars:
            var = self.Variable(freevar)
            self.freevarsubs += [(cos(var.var), var.cvar), (sin(var.var), var.svar)]
        
        Dee = Matrix(3,1,[x for x in Tee[0,0:3]])
        # Dee = Tfirstleft.inv() * Dee_in
        # LinksAccumLeftInv[x] * Dee = LinksAccumRight[x]
        # LinksAccumLeftInv[x] = InvLinks[x-1] * ... * InvLinks[0]
        # LinksAccumRight[x] = Links[x]*Links[x+1]...*Links[-1]
        LinksAccumLeftAll = [eye(4)]
        LinksAccumLeftInvAll = [eye(4)]
        LinksAccumRightAll = [eye(4)]
        for i in range(len(Links)):
            LinksAccumLeftAll.append(LinksAccumLeftAll[-1]*Links[i])
            LinksAccumLeftInvAll.append(LinksInv[i]*LinksAccumLeftInvAll[-1])
            LinksAccumRightAll.append(Links[len(Links)-i-1]*LinksAccumRightAll[-1])
        LinksAccumRightAll.reverse()
        
        LinksAccumLeftAll = [self.affineSimplify(T) for T in LinksAccumLeftAll]
        LinksAccumLeftInvAll = [self.affineSimplify(T) for T in LinksAccumLeftInvAll]
        LinksAccumRightAll = [self.affineSimplify(T) for T in LinksAccumRightAll]
        
        solverbranches = []
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        for freevar in freejointvars+[None]:
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None and self.IsHinge(freevar.name) else [None]
            for freevalue in freevalues:
                print 'attempting ',freevar,' = ',freevalue
                if freevar is not None and freevalue is not None:
                    var = self.Variable(freevar)
                    valuesubs = [(var.var,freevalue),(var.svar,sin(freevalue)),(var.cvar,cos(freevalue))] if freevalue is not None else []
                    freevarcond = [freevar-freevalue]
                else:
                    valuesubs = []
                    freevarcond = None

                storesolutiontree = [SolverStoreSolution (jointvars)]
                solvedvarsubs = valuesubs+self.freevarsubs
                rotsubs = [(Symbol('r%d%d'%(0,i)),Symbol('new_r%d%d'%(0,i))) for i in range(3)]
                rotvars = [var for var in jointvars if any([var==svar for svar in solvejointvars])]
                D = Matrix(3,1, [x.subs(self.freevarsubs) for x in LinksAccumRightAll[0][0:3,0:3]*basedir])
                rottree = self.solveIKRotation(R=D,Ree = Dee.subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
                solverbranches.append((freevarcond,[SolverDirection(LinksAccumLeftInvAll[0].subs(solvedvarsubs)[0:3,0:3]*Dee, rottree)]))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for kinematics'
            return None
        return SolverIKChainDirection3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Tfirstleft.inv()[0:3,0:3] * Dee, [SolverBranchConds(solverbranches)])

    def solveFullIK_Rotation3D(self,chain,Tee,Rbaseraw=eye(3)):
        Rbase = eye(4)
        for i in range(3):
            for j in range(3):
                Rbase[i,j] = Rbaseraw[i,j]
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        Tfirstright = Links.pop()*Rbase
        LinksInv = [self.affineInverse(link) for link in Links]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]

        if not len(solvejointvars) == 3:
            raise ValueError('solve joints needs to be 3')

        # when solving equations, convert all free variables to constants
        self.freevarsubs = []
        for freevar in freejointvars:
            var = self.Variable(freevar)
            self.freevarsubs += [(cos(var.var), var.cvar), (sin(var.var), var.svar)]

        Ree = Tee[0:3,0:3]
        #Ree = Tfirstleft.inv() * Ree_in * Tfirstright.inv()
        # LinksAccumLeftInv[x] * Ree = LinksAccumRight[x]
        # LinksAccumLeftInv[x] = InvLinks[x-1] * ... * InvLinks[0]
        # LinksAccumRight[x] = Links[x]*Links[x+1]...*Links[-1]
        LinksAccumLeftAll = [eye(4)]
        LinksAccumLeftInvAll = [eye(4)]
        LinksAccumRightAll = [eye(4)]
        for i in range(len(Links)):
            LinksAccumLeftAll.append(LinksAccumLeftAll[-1]*Links[i])
            LinksAccumLeftInvAll.append(LinksInv[i]*LinksAccumLeftInvAll[-1])
            LinksAccumRightAll.append(Links[len(Links)-i-1]*LinksAccumRightAll[-1])
        LinksAccumRightAll.reverse()
        
        LinksAccumLeftAll = [self.affineSimplify(T) for T in LinksAccumLeftAll]
        LinksAccumLeftInvAll = [self.affineSimplify(T) for T in LinksAccumLeftInvAll]
        LinksAccumRightAll = [self.affineSimplify(T) for T in LinksAccumRightAll]

        solverbranches = []
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        for freevar in freejointvars+[None]:
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None and self.IsHinge(freevar.name) else [None]
            for freevalue in freevalues:
                print 'attempting ',freevar,' = ',freevalue
                if freevar is not None and freevalue is not None:
                    var = self.Variable(freevar)
                    valuesubs = [(var.var,freevalue),(var.svar,sin(freevalue)),(var.cvar,cos(freevalue))] if freevalue is not None else []
                    freevarcond = [freevar-freevalue]
                else:
                    valuesubs = []
                    freevarcond = None

                storesolutiontree = [SolverStoreSolution (jointvars)]
                solvedvarsubs = valuesubs+self.freevarsubs
                rotsubs = [(Symbol('r%d%d'%(i,j)),Symbol('new_r%d%d'%(i,j))) for i in range(3) for j in range(3)]
                rotvars = [var for var in jointvars if any([var==svar for svar in solvejointvars])]
                R = Matrix(3,3, [x.subs(solvedvarsubs) for x in LinksAccumRightAll[0][0:3,0:3]])
                rottree = self.solveIKRotation(R=R,Ree = Ree.subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
                solverbranches.append((freevarcond,[SolverRotation(LinksAccumLeftInvAll[0].subs(solvedvarsubs)*Tee, rottree)]))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for kinematics'
            return None
        return SolverIKChainRotation3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Tfirstleft.inv()[0:3,0:3] * Ree * Tfirstright.inv()[0:3,0:3], [SolverBranchConds(solverbranches)],Rfk = Tfirstleft[0:3,0:3] * LinksAccumRightAll[0][0:3,0:3])

    def solveFullIK_Translation3D(self,chain,Tee,rawbasepos=Matrix(3,1,[S.Zero,S.Zero,S.Zero])):
        basepos = Matrix(3,1,rawbasepos.tolist())
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        TfirstleftInv = Tfirstleft.inv()
        LinksInv = [self.affineInverse(link) for link in Links]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]

        if not len(solvejointvars) == 3:
            raise ValueError('solve joints needs to be 3')

        # when solving equations, convert all free variables to constants
        self.freevarsubs = []
        for freevar in freejointvars:
            var = self.Variable(freevar)
            self.freevarsubs += [(cos(var.var), var.cvar), (sin(var.var), var.svar)]

        Pee = Matrix(3,1,[x for x in Tee[0:3,3]])
        #Pee = Tfirstleft.inv() * Pee_in
        # LinksAccumLeftInv[x] * Pee = LinksAccumRight[x]
        # LinksAccumLeftInv[x] = InvLinks[x-1] * ... * InvLinks[0]
        # LinksAccumRight[x] = Links[x]*Links[x+1]...*Links[-1]
        LinksAccumLeftAll = [eye(4)]
        LinksAccumLeftInvAll = [eye(4)]
        LinksAccumRightAll = [eye(4)]
        for i in range(len(Links)):
            LinksAccumLeftAll.append(LinksAccumLeftAll[-1]*Links[i])
            LinksAccumLeftInvAll.append(LinksInv[i]*LinksAccumLeftInvAll[-1])
            LinksAccumRightAll.append(Links[len(Links)-i-1]*LinksAccumRightAll[-1])
        LinksAccumRightAll.reverse()
        
        LinksAccumLeftAll = [self.affineSimplify(T) for T in LinksAccumLeftAll]
        LinksAccumLeftInvAll = [self.affineSimplify(T) for T in LinksAccumLeftInvAll]
        LinksAccumRightAll = [self.affineSimplify(T) for T in LinksAccumRightAll]
        
        solverbranches = []
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        valuesubs = []
        freevarcond = None
                
        Positions, Positionsee = self.buildPositionEquations(LinksAccumRightAll, LinksAccumLeftInvAll,basepos,Pee)
        AllEquations = self.buildEquationsFromPositions(Positions,Positionsee)
        solsubs = self.freevarsubs[:]
        endbranchtree = [SolverStoreSolution (jointvars)]                
        curtransvars = solvejointvars[:]
        transtree = self.solveAllEquations(AllEquations,curvars=curtransvars,othersolvedvars=freejointvars,solsubs = solsubs,endbranchtree=endbranchtree)
        solverbranches.append((None,transtree))
        return SolverIKChainTranslation3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], TfirstleftInv[0:3,0:3] * Pee + TfirstleftInv[0:3,3], [SolverBranchConds(solverbranches)],Pfk = Tfirstleft * (LinksAccumRightAll[0]*Matrix(4,1,[basepos[0],basepos[1],basepos[2],1.0])))

    def solveFullIK_Ray4D(self,chain,Tee,rawbasedir=Matrix(3,1,[S.Zero,S.Zero,S.One]),rawbasepos=Matrix(3,1,[S.Zero,S.Zero,S.Zero])):
        """basedir,basepos needs to be filled with a direction and position of the ray to control"""
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        TfirstleftInv = Tfirstleft.inv()
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]
        
        basedir = Matrix(3,1,rawbasedir.tolist())
        basepos = Matrix(3,1,rawbasepos.tolist())
        basedir /= sqrt(basedir[0]*basedir[0]+basedir[1]*basedir[1]+basedir[2]*basedir[2]).evalf()
        basepos = basepos-basedir*basedir.dot(basepos)
        
        if not len(solvejointvars) == 4:
            raise ValueError('solve joints needs to be 4')

        # rotate all links so that basedir becomes the z-axis
        #Trightnormaliation = self.rotateDirection(sourcedir=Matrix(3,1,[Real(round(float(x),4),30) for x in basedir]), targetdir = Matrix(3,1,[S.Zero,S.Zero,S.One]))
        #Links[-1] *= self.affineInverse(Trightnormaliation)
        LinksInv = [self.affineInverse(link) for link in Links]

        # when solving equations, convert all free variables to constants
        self.freevarsubs = []
        for freevar in freejointvars:
            var = self.Variable(freevar)
            self.freevarsubs += [(cos(var.var), var.cvar), (sin(var.var), var.svar)]
        
        Dee = Matrix(3,1,[x for x in Tee[0,0:3]])
        Pee = Matrix(3,1,[x for x in Tee[0:3,3]])
        # Dee = Tfirstleft.inv() * Dee_in
        # LinksAccumLeftInv[x] * Dee = LinksAccumRight[x]
        # LinksAccumLeftInv[x] = InvLinks[x-1] * ... * InvLinks[0]
        # LinksAccumRight[x] = Links[x]*Links[x+1]...*Links[-1]
        LinksAccumLeftAll = [eye(4)]
        LinksAccumLeftInvAll = [eye(4)]
        LinksAccumRightAll = [eye(4)]
        for i in range(len(Links)):
            LinksAccumLeftAll.append(LinksAccumLeftAll[-1]*Links[i])
            LinksAccumLeftInvAll.append(LinksInv[i]*LinksAccumLeftInvAll[-1])
            LinksAccumRightAll.append(Links[len(Links)-i-1]*LinksAccumRightAll[-1])
        LinksAccumRightAll.reverse()
        
        LinksAccumLeftAll = [self.affineSimplify(T) for T in LinksAccumLeftAll]
        LinksAccumLeftInvAll = [self.affineSimplify(T) for T in LinksAccumLeftInvAll]
        LinksAccumRightAll = [self.affineSimplify(T) for T in LinksAccumRightAll]

        # create LinksAccumX indexed by joint indices
        assert( len(LinksAccumLeftAll)%2 == 1 )
        LinksAccumLeft = []
        LinksAccumLeftInv = []
        LinksAccumRight = []
        for i in range(0,len(LinksAccumLeftAll),2):
            LinksAccumLeft.append(LinksAccumLeftAll[i])
            LinksAccumLeftInv.append(LinksAccumLeftInvAll[i])
            LinksAccumRight.append(LinksAccumRightAll[i])
        assert( len(LinksAccumLeft) == len(jointvars)+1 )
        
        solverbranches = []
        valuesubs = []
        freevarcond = None

        Ds = [Matrix(3,1, [self.chop(x,accuracy=self.accuracy*10.0) for x in T[0:3,0:3]*basedir]) for T in LinksAccumRightAll[0:6]]
        Dsee = [Matrix(3,1, [self.chop(x,accuracy=self.accuracy*10.0) for x in T[0:3,0:3]*Dee]) for T in LinksAccumLeftInvAll[0:6]]
        Positions, Positionsee = self.buildPositionEquations(LinksAccumRightAll[0:5], LinksAccumLeftInvAll[0:5],basepos,Pee)
        Positionsnew = []
        Positionseenew = []
#         for i in range(len(Positions)):
#             p = Positions[i].cross(Dsee[i]).expand()
#             if all([self.codeComplexity(x)<2000 for x in p]):
#                 p = Matrix(3,1,[self.customtrigsimp(self.customtrigsimp(x)) for x in p])
#             pee = Positionsee[i].cross(Dsee[i]).expand()
#             if all([self.codeComplexity(x)<2000 for x in pee]):
#                 pee = Matrix(3,1,[self.customtrigsimp(self.customtrigsimp(x)) for x in pee])
#             Positionsnew.append(p)
#             Positionseenew.append(pee)
#         for i in range(len(Positions)):
#             p = Positions[i].cross(Ds[i]).expand()
#             if all([self.codeComplexity(x)<2000 for x in p]):
#                 p = Matrix(3,1,[self.customtrigsimp(self.customtrigsimp(x)) for x in p])
#             pee = Positionsee[i].cross(Ds[i]).expand()
#             if all([self.codeComplexity(x)<2000 for x in pee]):
#                 pee = Matrix(3,1,[self.customtrigsimp(self.customtrigsimp(x)) for x in pee])
#             Positionsnew.append(p)
#             Positionseenew.append(pee)
        for i in range(len(Positions)):
            p = Positions[i].cross(Ds[i]).expand()
            if all([self.codeComplexity(x)<2000 for x in p]):
                p = Matrix(3,1,[self.customtrigsimp(self.customtrigsimp(x)) for x in p])
            pee = Positionsee[i].cross(Dsee[i]).expand()
            if all([self.codeComplexity(x)<2000 for x in pee]):
                pee = Matrix(3,1,[self.customtrigsimp(self.customtrigsimp(x)) for x in pee])
            Positionsnew.append(p)
            Positionseenew.append(pee)

        # try to shift all the constants of each Position expression to one side
        for i in range(len(Positionsnew)):
            for j in range(3):
                p = Positionsnew[i][j]
                pee = Positionseenew[i][j]
                pconstterm = None
                peeconstterm = None
                if p.is_Add:
                    pconstterm = [term for term in p.args if term.is_number]
                elif p.is_number:
                    pconstterm = [p]
                else:
                    continue
                if pee.is_Add:
                    peeconstterm = [term for term in pee.args if term.is_number]
                elif pee.is_number:
                    peeconstterm = [pee]
                else:
                    continue
                if len(pconstterm) > 0 and len(peeconstterm) > 0:
                    # shift it to the one that has the least terms
                    for term in peeconstterm if len(p.args) < len(pee.args) else pconstterm:
                        Positionsnew[i][j] -= term
                        Positionseenew[i][j] -= term

        orgsolsubs = self.freevarsubs[:]
        rottree = []
        endbranchtree = [SolverSequence([rottree])]
        AllEquations = self.buildEquationsFromPositions(Positionsnew,Positionseenew,uselength=False)
        for i in range(len(Ds)):
            for j in range(3):
                e = (Ds[i][j] - Dsee[i][j]).expand()
                if self.codeComplexity(e) <= 1000:
                    e = self.customtrigsimp(e)
                if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                    AllEquations.append(e)
        self.sortComplexity(AllEquations)
        endbranchtree = [SolverStoreSolution (jointvars)]
        fulltree = self.solveAllEquations(AllEquations,curvars=solvejointvars,othersolvedvars = freejointvars[:],solsubs = orgsolsubs,endbranchtree=endbranchtree)
        solverbranches.append((freevarcond,fulltree))
        return SolverIKChainRay4D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], TfirstleftInv[0:3,0:3] * Pee + TfirstleftInv[0:3,3], TfirstleftInv[0:3,0:3] * Dee, [SolverBranchConds(solverbranches)],Dfk=Tfirstleft[0:3,0:3]*LinksAccumRightAll[0][0:3,0:3]*basedir,Pfk=Tfirstleft*(LinksAccumRightAll[0]*Matrix(4,1,[rawbasepos[0],rawbasepos[1],rawbasepos[2],1.0])))

    def solveFullIK_Lookat3D(self,chain,Tee,rawbasedir=Matrix(3,1,[S.Zero,S.Zero,S.One]),rawbasepos=Matrix(3,1,[S.Zero,S.Zero,S.Zero])):
        """basedir,basepos needs to be filled with a direction and position of the ray to control the lookat
        """
        basedir = Matrix(3,1,rawbasedir.tolist())
        basepos = Matrix(3,1,rawbasepos.tolist())
        basedir /= sqrt(basedir[0]*basedir[0]+basedir[1]*basedir[1]+basedir[2]*basedir[2]).evalf()
        basepos = basepos-basedir*basedir.dot(basepos)
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        TfirstleftInv = Tfirstleft.inv()
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]        
        if not len(solvejointvars) == 2:
            raise ValueError('solve joints needs to be 2')

        LinksInv = [self.affineInverse(link) for link in Links]

        # when solving equations, convert all free variables to constants
        self.freevarsubs = []
        for freevar in freejointvars:
            var = self.Variable(freevar)
            self.freevarsubs += [(cos(var.var), var.cvar), (sin(var.var), var.svar)]
        
        Pee = Matrix(3,1,[x for x in Tee[0:3,3]])
        # Pee = Tfirstleft.inv() * Pee_in
        # LinksAccumLeftInv[x] * Pee = LinksAccumRight[x]
        # LinksAccumLeftInv[x] = InvLinks[x-1] * ... * InvLinks[0]
        # LinksAccumRight[x] = Links[x]*Links[x+1]...*Links[-1]
        LinksAccumLeftAll = [eye(4)]
        LinksAccumLeftInvAll = [eye(4)]
        LinksAccumRightAll = [eye(4)]
        for i in range(len(Links)):
            LinksAccumLeftAll.append(LinksAccumLeftAll[-1]*Links[i])
            LinksAccumLeftInvAll.append(LinksInv[i]*LinksAccumLeftInvAll[-1])
            LinksAccumRightAll.append(Links[len(Links)-i-1]*LinksAccumRightAll[-1])
        LinksAccumRightAll.reverse()
        
        LinksAccumLeftAll = [self.affineSimplify(T) for T in LinksAccumLeftAll]
        LinksAccumLeftInvAll = [self.affineSimplify(T) for T in LinksAccumLeftInvAll]
        LinksAccumRightAll = [self.affineSimplify(T) for T in LinksAccumRightAll]

        # create LinksAccumX indexed by joint indices
        assert( len(LinksAccumLeftAll)%2 == 1 )
        LinksAccumLeft = []
        LinksAccumLeftInv = []
        LinksAccumRight = []
        for i in range(0,len(LinksAccumLeftAll),2):
            LinksAccumLeft.append(LinksAccumLeftAll[i])
            LinksAccumLeftInv.append(LinksAccumLeftInvAll[i])
            LinksAccumRight.append(LinksAccumRightAll[i])
        assert( len(LinksAccumLeft) == len(jointvars)+1 )
        
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        valuesubs = []
        freevarcond = None

        Ds = [Matrix(3,1, [self.chop(x,accuracy=self.accuracy*10.0) for x in T[0:3,0:3]*basedir]) for T in LinksAccumRightAll[0:5]]
        Positions, Positionsee = self.buildPositionEquations(LinksAccumRightAll[0:5], LinksAccumLeftInvAll[0:5],basepos,Pee)

        Positionsnew = []
        Positionseenew = []
        for i in range(len(Positions)):
            p = Positions[i].cross(Ds[i]).expand()
            if all([self.codeComplexity(x)<2000 for x in p]):
                p = Matrix(3,1,[self.customtrigsimp(self.customtrigsimp(x)) for x in p])
            pee = Positionsee[i].cross(Ds[i]).expand()
            if all([self.codeComplexity(x)<2000 for x in pee]):
                pee = Matrix(3,1,[self.customtrigsimp(self.customtrigsimp(x)) for x in pee])
            Positionsnew.append(p)
            Positionseenew.append(pee)
        Positions = Positionsnew
        Positionsee = Positionseenew

        # try to shift all the constants of each Position expression to one side
        for i in range(len(Positions)):
            for j in range(3):
                p = Positions[i][j]
                pee = Positionsee[i][j]
                pconstterm = None
                peeconstterm = None
                if p.is_Add:
                    pconstterm = [term for term in p.args if term.is_number]
                elif p.is_number:
                    pconstterm = [p]
                else:
                    continue
                if pee.is_Add:
                    peeconstterm = [term for term in pee.args if term.is_number]
                elif pee.is_number:
                    peeconstterm = [pee]
                else:
                    continue
                if len(pconstterm) > 0 and len(peeconstterm) > 0:
                    # shift it to the one that has the least terms
                    for term in peeconstterm if len(p.args) < len(pee.args) else pconstterm:
                        Positions[i][j] -= term
                        Positionsee[i][j] -= term

        orgsolsubs = self.freevarsubs[:]
        AllEquations = self.buildEquationsFromPositions(Positions,Positionsee)
        endbranchtree = [SolverStoreSolution (jointvars)]
        fulltree = self.solveAllEquations(AllEquations,curvars=solvejointvars,othersolvedvars = freejointvars[:],solsubs = orgsolsubs,endbranchtree=endbranchtree)
        solverbranches = [(freevarcond,fulltree)]
        return SolverIKChainLookat3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Pee=TfirstleftInv[0:3,0:3] * Pee + TfirstleftInv[0:3,3], jointtree=[SolverBranchConds(solverbranches)],Dfk=Tfirstleft[0:3,0:3]*LinksAccumRightAll[0][0:3,0:3]*basedir,Pfk=Tfirstleft*(LinksAccumRightAll[0]*Matrix(4,1,[rawbasepos[0],rawbasepos[1],rawbasepos[2],1.0])))
        
    def solveFullIK_6D(self, chain, Tee, Tgripperraw=eye(4)):
        Tgripper = eye(4)
        for i in range(4):
            for j in range(4):
                Tgripper[i,j] = Tgripperraw[i,j]
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        Tfirstright = Links.pop()*Tgripper
        LinksInv = [self.affineInverse(link) for link in Links]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]

        #valuesubs = [(jointvars[i],jointvalues[i]) for i in range(7)]+[(Tee[i],Teevalue[i]) for i in range(12)]+[(Symbol('cj%d'%i),cos(jointvalues[i])) for i in range(7)]+[(Symbol('sj%d'%i),sin(jointvalues[i])) for i in range(7)]
        
        # when solving equations, convert all free variables to constants
        self.freevarsubs = []
        self.freevars = []
        for freevar in freejointvars:
            var = self.Variable(freevar)
            self.freevarsubs += [(cos(var.var), var.cvar), (sin(var.var), var.svar)]
            self.freevars += [var.cvar,var.svar]

        inverted = False
        while True:
            #Tee = Tfirstleft.inv() * Tee_in * Tfirstright.inv()
            # LinksAccumLeftInv[x] * Tee = LinksAccumRight[x]
            # LinksAccumLeftInv[x] = InvLinks[x-1] * ... * InvLinks[0]
            # LinksAccumRight[x] = Links[x]*Links[x+1]...*Links[-1]
            LinksAccumLeftAll = [eye(4)]
            LinksAccumLeftInvAll = [eye(4)]
            LinksAccumRightAll = [eye(4)]
            for i in range(len(Links)):
                LinksAccumLeftAll.append(LinksAccumLeftAll[-1]*Links[i])
                LinksAccumLeftInvAll.append(LinksInv[i]*LinksAccumLeftInvAll[-1])
                LinksAccumRightAll.append(Links[len(Links)-i-1]*LinksAccumRightAll[-1])
            LinksAccumRightAll.reverse()

            LinksAccumLeftAll = [self.affineSimplify(T) for T in LinksAccumLeftAll]
            LinksAccumLeftInvAll = [self.affineSimplify(T) for T in LinksAccumLeftInvAll]
            LinksAccumRightAll = [self.affineSimplify(T) for T in LinksAccumRightAll]

            # create LinksAccumX indexed by joint indices
            assert( len(LinksAccumLeftAll)%2 == 0 )
            LinksAccumLeft = []
            LinksAccumLeftInv = []
            LinksAccumRight = []
            for i in range(0,len(LinksAccumLeftAll),2)+[len(LinksAccumLeftAll)-1]:
                LinksAccumLeft.append(LinksAccumLeftAll[i])
                LinksAccumLeftInv.append(LinksAccumLeftInvAll[i])
                LinksAccumRight.append(LinksAccumRightAll[i])
            assert( len(LinksAccumLeft) == len(jointvars)+1 )

            # find last point that separates translation and rotation
            lastsepindex = -1
            for i in isolvejointvars:
                testjoints = [j for j in jointvars[i:] if not any([j==jfree for jfree in freejointvars])]
                if not any([LinksAccumRight[i][j,3].has_any_symbols(*testjoints) for j in range(0,3)]):
                    lastsepindex = i
                    break

            if lastsepindex < 0:
                if inverted:
                    print 'failed to find joint index to separate translation and rotation'
                    return None
                print "Taking the inverse of the mechanism (sometimes intersecting joints are on the base)"
                Links = [self.affineInverse(Links[i]) for i in range(len(Links)-1,-1,-1)]
                LinksInv = [self.affineInverse(link) for link in Links]
                inverted=True
                continue

            # find a set of joints starting from the last that can solve for a full 3D rotation
            rotindex = min(len(jointvars)-3,lastsepindex)
            while rotindex>=0:
                # check if any entries are constant
                if all([not LinksAccumRight[rotindex][i,j].is_zero for i in range(3) for j in range(3)]):
                    break
                rotindex = rotindex-1

            if rotindex < 0:
                print 'Current joints cannot solve for a full 3D rotation'

            # add all but first 3 vars to free parameters
            rotvars = []
            transvars = []
            for svar in solvejointvars:
                if any([LinksAccumRight[rotindex][i,j].has_any_symbols(svar) for i in range(3) for j in range(3)]):
                    rotvars.append(svar)
                else:
                    transvars.append(svar)
            #transvars = solvejointvars[0:min(3,lastsepindex)]
            if len(rotvars) != 3 or len(transvars) != 3:
                if inverted:
                    print 'rotvars: ',rotvars,' transvars: ',transvars,' not 3 dims'
                    return None
                print "Taking the inverse of the mechanism (sometimes intersecting joints are on the base)"
                Links = [self.affineInverse(Links[i]) for i in range(len(Links)-1,-1,-1)]
                LinksInv = [self.affineInverse(link) for link in Links]
                inverted=True
                continue
            
            # check if the translation variables affect rotation or the rotation variables affect translation.
            # If translation variables do not affect rotation, solve for rotation first.
            solveRotationFirst = None
            if not any([LinksAccumRight[0][i,j].has_any_symbols(*transvars) for i in range(3) for j in range(3)]):
                solveRotationFirst = True
            # If rotation variables do not affect translation, solve for translation first
            elif not any([LinksAccumRight[0][i,3].has_any_symbols(*rotvars) for i in range(3)]):
                solveRotationFirst = False
            else:
                # If both affect each other, fail
                if inverted:
                    print 'could not divide translation/rotation'
                    return None
                print "Taking the inverse of the mechanism (sometimes intersecting joints are on the base)"
                Links = [self.affineInverse(Links[i]) for i in range(len(Links)-1,-1,-1)]
                LinksInv = [self.affineInverse(link) for link in Links]
                inverted=True
                continue
            break

        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        valuesubs = []
        freevarcond = None
                
        Positions, Positionsee = self.buildPositionEquations(LinksAccumRightAll[0:(1+lastsepindex*2)], LinksAccumLeftInvAll[0:(1+lastsepindex*2)],zeros((3,1)),Tee[0:3,3])
        rottree = []
        solsubs = self.freevarsubs[:]
        if solveRotationFirst:
            for rvar in rotvars:
                solsubs += [(cos(rvar),self.Variable(rvar).cvar),(sin(rvar),self.Variable(rvar).svar)]
            endbranchtree = [SolverStoreSolution (jointvars)]
        else:
            endbranchtree = [SolverSequence([rottree])]

        curtransvars = transvars[:]
        AllEquations = self.buildEquationsFromPositions(Positions,Positionsee)
        if not solveRotationFirst:
            AllEquations = [eq for eq in AllEquations if not eq.has_any_symbols(*rotvars)]
        transtree = self.solveAllEquations(AllEquations,curvars=curtransvars,othersolvedvars = rotvars+freejointvars if solveRotationFirst else freejointvars,solsubs = solsubs,endbranchtree=endbranchtree)

        solvertree = []
        solvedvarsubs = valuesubs+self.freevarsubs
        if solveRotationFirst:
            storesolutiontree = transtree
        else:
            solvertree += transtree
            storesolutiontree = [SolverStoreSolution (jointvars)]
            for tvar in transvars:
                solvedvarsubs += [(cos(tvar),self.Variable(tvar).cvar),(sin(tvar),self.Variable(tvar).svar)]

        rotsubs = [(Symbol('r%d%d'%(i,j)),Symbol('new_r%d%d'%(i,j))) for i in range(3) for j in range(3)]
        R = Matrix(3,3, [x.subs(solvedvarsubs) for x in LinksAccumRight[rotindex][0:3,0:3]])
        rottree += self.solveIKRotation(R=R,Ree = Tee[0:3,0:3].subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
        if len(rottree) == 0:
            raise self.CannotSolveError('could not solve for all rotation variables: %s:%s'%(str(freevar),str(freevalue)))
        if solveRotationFirst:
            solvertree.append(SolverRotation(LinksAccumLeftInv[rotindex].subs(solvedvarsubs)*Tee, rottree))
        else:
            rottree[:] = [SolverRotation(LinksAccumLeftInv[rotindex].subs(solvedvarsubs)*Tee, rottree[:])]
        solverbranches = [(freevarcond,solvertree)]

        Tgoal = Tfirstleft.inv() * Tee * Tfirstright.inv()
        if inverted:
            Tgoal = self.affineInverse(Tgoal)
        return SolverIKChainTransform6D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Tgoal, [SolverBranchConds(solverbranches)],Tfk=Tfirstleft * LinksAccumRightAll[0] * Tfirstright)

    def isExpressionUnique(self, exprs, expr):
        for exprtest in exprs:
            e = expr-exprtest
            if self.chop(e,self.accuracy*0.01) == S.Zero:
                return False
        return True

    def solveAllEquations(self,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree,currentcases=None):
        if len(curvars) == 0:
            return endbranchtree
        print curvars,othersolvedvars
        solsubs = solsubs[:]
        freevarinvsubs = [(f[1],f[0]) for f in self.freevarsubs]
        solinvsubs = [(f[1],f[0]) for f in solsubs]
        # single variable solutions
        solutions = []
        for curvar in curvars:
            othervars = [var for var in curvars if var != curvar]
            raweqns = []
            for e in AllEquations:
                if (len(othervars) == 0 or not e.has_any_symbols(*othervars)) and e.has_any_symbols(curvar):
                    eq = e.subs(self.freevarsubs+solsubs)
                    if self.isExpressionUnique(raweqns,eq) and self.isExpressionUnique(raweqns,-eq):
                        raweqns.append(eq)
            if len(raweqns) > 0:
                try:
                    rawsolutions=self.solveSingleVariable(raweqns,curvar)
                    for solution in rawsolutions:
                        #solution.subs(freevarinvsubs)
                        self.solutionComplexity(solution,othersolvedvars,curvars)
                        solutions.append((solution,curvar))
                except self.CannotSolveError:
                    pass

        if len(solutions) > 0:
            return self.addSolution(solutions,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree,currentcases=currentcases)

        curvarsubssol = []
        for var0,var1 in combinations(curvars,2):
            othervars = [var for var in curvars if var != var0 and var != var1]
            raweqns = []
            complexity = 0
            for e in AllEquations:
                if (len(othervars) == 0 or not e.has_any_symbols(*othervars)) and e.has_any_symbols(var0,var1):
                    eq = e.subs(self.freevarsubs+solsubs)
                    if self.isExpressionUnique(raweqns,eq) and self.isExpressionUnique(raweqns,-eq):
                        raweqns.append(eq)
                        complexity += self.codeComplexity(eq)
            if len(raweqns) > 1:
                curvarsubssol.append((var0,var1,raweqns,complexity))
        curvarsubssol.sort(lambda x, y: x[3]-y[3])
        for var0,var1,raweqns,complexity in curvarsubssol:
            try:
                rawsolutions=self.solvePairVariables(raweqns,var0,var1)
                for solution in rawsolutions:
                    #solution.subs(freevarinvsubs)
                    self.solutionComplexity(solution,othersolvedvars,curvars)
                    solutions.append((solution,Symbol(solution.jointname)))
                if len(rawsolutions) > 0: # solving a pair is rare, so any solution will do
                    break
            except self.CannotSolveError:
                pass

        # take the least complex solution and go on
        if len(solutions) > 0:
            return self.addSolution(solutions,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree,currentcases=currentcases)

        # try to solve one variable for the others and substitute
        # really bad since substitued variable does not constraing cos**2+sin**2=1
#         curvarsubssol = []
#         for curvar in curvars:
#             vv = self.Variable(curvar)
#             varsubs = [(sin(curvar),vv.svar),(cos(curvar),vv.cvar)]
#             othervars = [var for var in curvars if var != curvar]
#             raweqns = []
#             for e in AllEquations:
#                 p = Poly(e.subs(varsubs),vv.cvar,vv.svar)
#                 if p.degree == 1 and not p.coeff(1,0).has_any_symbols(*othervars) and not p.coeff(0,1).has_any_symbols(*othervars):
#                     eq = e.subs(self.freevarsubs+solsubs)
#                     if self.isExpressionUnique(raweqns,eq) and self.isExpressionUnique(raweqns,-eq):
#                         raweqns.append(eq)
#             if len(raweqns) > 1:
#                 eqns = [eq.subs(varsubs) for eq in raweqns]
#                 for eqns2 in combinations(eqns,2):
#                     s = solve(eqns2,vv.cvar,vv.svar)
#                     if s is not None and s.has_key(vv.svar) and s.has_key(vv.cvar):
#                         cvarsol = s[vv.cvar].subs(freevarinvsubs+solinvsubs)
#                         svarsol = s[vv.svar].subs(freevarinvsubs+solinvsubs)
#                         curvarsubssol.append((curvar,cvarsol,svarsol,self.codeComplexity(cvarsol)+self.codeComplexity(svarsol),raweqns))
#                         break
#         if len(curvarsubssol) > 0:
#             try:
#                 curvar, cvarsol, svarsol, rank, raweqns = min(curvarsubssol,key=lambda f: f[3])
#                 vv = self.Variable(curvar)
#                 varsubs = [(sin(curvar),vv.svar),(cos(curvar),vv.cvar)]
#                 othervars = [var for var in curvars if var != curvar]
#                 print 'attempting to cancel variable',curvar
#                 commonmult0 = fraction(cvarsol)[1]
#                 commonmult1 = fraction(svarsol)[1]
#                 commonmultsubs = [(commonmult0,Symbol('commonmult0')),(commonmult1,Symbol('commonmult1'))]
#                 commonmultsubsinv = [(Symbol('commonmult0'),commonmult0),(Symbol('commonmult1'),commonmult1)]
#                 tempsubs = [(sin(curvar),svarsol.subs(commonmultsubs)),(cos(curvar),cvarsol.subs(commonmultsubs))]
#                 NewEquations = []
#                 for eq in AllEquations:
#                     if eq.has_any_symbols(curvar):
#                         p = Poly(eq.subs([(sin(curvar),vv.svar),(cos(curvar),vv.cvar)]),vv.cvar,vv.svar)
#                         maxdegree0 = max([m[0] for m in p.monoms])
#                         maxdegree1 = max([m[1] for m in p.monoms])
#                         neweq = self.customtrigsimp(self.removecommonexprs(eq.subs(tempsubs)*(Symbol('commonmult0')**maxdegree0)*(Symbol('commonmult1')**maxdegree1)))
#                         if self.codeComplexity(neweq) < 300:
#                             neweq = self.chop(simplify(neweq.subs(commonmultsubsinv)),accuracy=self.accuracy*0.1)
#                             if neweq != S.Zero and self.codeComplexity(neweq) < 300:
#                                 NewEquations.append(neweq)
#                     else:
#                         NewEquations.append(eq)
#                 NewEquations.sort(lambda x, y: self.codeComplexity(x)-self.codeComplexity(y))
#                 newvars = curvars[:]
#                 newvars.remove(curvar)
#                 newsolsubs = solsubs[:]
#                 for var in newvars:
#                     newsolsubs += [(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)]
#                 newendbranchtree = self.solveAllEquations(AllEquations,[curvar],othersolvedvars+newvars,newsolsubs,endbranchtree)
#                 return self.solveAllEquations(NewEquations,newvars,othersolvedvars,solsubs,newendbranchtree)
#             
#             except self.CannotSolveError:
#                 pass

        raise self.CannotSolveError('failed to find a variable to solve')

    def addSolution(self,solutions,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree,currentcases=None):
        """Take the least complex solution of a set of solutions and resume solving
        """
        solutions = [s for s in solutions if s[0].score < oo and s[0].checkValidSolution()] # remove infinite scores
        solutions.sort(lambda x, y: x[0].score-y[0].score)
        for solution in solutions:
            checkforzeros = solution[0].checkforzeros
            if len(checkforzeros) == 0:
                # did find a good solution, so take it. Make sure to check any zero branches
                var = solution[1]
                newvars=curvars[:]
                newvars.remove(var)
                return [solution[0].subs(solsubs)]+self.solveAllEquations(AllEquations,curvars=newvars,othersolvedvars=othersolvedvars+[var],solsubs=solsubs+[(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)],endbranchtree=endbranchtree,currentcases=currentcases)

        # all solutions have check for zero equations
        # choose the variable with the shortest solution and compute (this is a conservative approach)
        usedsolutions = []
        # remove any solutions with similar checkforzero constraints (because they are essentially the same)
        for solution,var in solutions:
            solution.subs(solsubs)
            if len(usedsolutions) == 0:
                usedsolutions.append((solution,var))
            else:
                match = False
                for usedsolution,usedvar in usedsolutions:
                    if len(solution.checkforzeros) == len(usedsolution.checkforzeros):
                        if not any([self.isExpressionUnique(usedsolution.checkforzeros,-eq) or self.isExpressionUnique(usedsolution.checkforzeros,eq) for eq in solution.checkforzeros]):
                            match = True
                            break
                if not match:
                    usedsolutions.append((solution,var))
                    if len(usedsolutions) >= 3:
                        break # don't need more than three alternatives (used to be two, but then lookat barrettwam4 proved that wrong)
        nextsolutions = dict()
        allvars = curvars+[Symbol('s%s'%v.name) for v in curvars]+[Symbol('c%s'%v.name) for v in curvars]
        lastbranch = []
        prevbranch=lastbranch
        if currentcases is None:
            currentcases = set()
        if self.degeneratecases is None:
            self.degeneratecases = self.DegenerateCases()
        handledconds = self.degeneratecases.gethandledconds(currentcases)
        eqs = []
        hascheckzeros = False
        for solution,var in usedsolutions[::-1]: # iterate in reverse order
            # there are divide by zeros, so check if they can be explicitly solved for joint variables
            checkforzeros = []
            for checkzero in solution.checkforzeros:
                if checkzero.has_any_symbols(*allvars):
                    print 'ignoring special check for zero'
                    continue
                checkforzeros.append(checkzero)
                for othervar in othersolvedvars:
                    sothervar = Symbol('s%s'%othervar.name)
                    cothervar = Symbol('c%s'%othervar.name)
                    if checkzero.has_any_symbols(othervar,sothervar,cothervar):
                        # the easiest thing to check first is if the equation evaluates to zero on boundaries 0,pi/2,pi,-pi/2
                        s = SolverSolution(othervar.name,jointeval=[],IsHinge=self.IsHinge(othervar.name))
                        for value in [S.Zero,pi/2,pi,-pi/2]:
                            try:
                                checkzerosub=checkzero.subs([(othervar,value),(sothervar,sin(value).evalf()),(cothervar,cos(value).evalf())])
                                if self.isValidSolution(checkzerosub) and checkzerosub.evalf() == S.Zero:
                                    s.jointeval.append(S.One*value)
                            except AssertionError,e:
                                print 'othervar %s=%f'%(str(othervar),value),e
                        if len(s.jointeval) > 0:
                            ss = [s]
                        else:
                            ss = []
                        try:
                            ss += self.solveSingleVariable([checkzero.subs([(sothervar,sin(othervar)),(cothervar,cos(othervar))])],othervar)
                        except self.CannotSolveError:
                            # this is actually a little tricky, sometimes really good solutions can have a divide that looks like:
                            # ((0.405 + 0.331*cj2)**2 + 0.109561*sj2**2 (manusarm_left.robot.xml)
                            # This will never be 0, but the solution cannot be solved. Instead of rejecting, add a condition to check if checkzero itself is 0 or not
                            pass
                        
                        for s in ss:
                            # can actually simplify Positions and possibly get a new solution!                            
                            if s.jointeval is not None:
                                for eq in s.jointeval:
                                    if eq.is_real:
                                        cond=othervar-eq.evalf()
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.IsHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(othervar,eq),(sothervar,sin(eq).evalf()),(sin(othervar),sin(eq).evalf()),(cothervar,cos(eq).evalf()),(cos(othervar),cos(eq).evalf())]])
                            elif s.jointevalsin is not None:
                                for eq in s.jointevalsin:
                                    if eq.is_real:
                                        cond=othervar-asin(eq).evalf()
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.IsHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(othervar,asin(eq).evalf()),(sothervar,eq),(sin(othervar),eq),(cothervar,sqrt(1-eq*eq).evalf()),(cos(othervar),sqrt(1-eq*eq).evalf())]])
                                        cond=othervar-(pi-asin(eq).evalf())
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.IsHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(othervar,(pi-asin(eq)).evalf()),(sothervar,eq),(sin(othervar),eq),(cothervar,-sqrt(1-eq*eq).evalf()),(cos(othervar),-sqrt(1-eq*eq).evalf())]])
                            elif s.jointevalcos is not None:
                                for eq in s.jointevalcos:
                                    if eq.is_real:
                                        cond=othervar-acos(eq).evalf()
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.IsHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(othervar,acos(eq).evalf()),(sothervar,sqrt(1-eq*eq).evalf()),(sin(othervar),sqrt(1-eq*eq).evalf()),(cothervar,eq),(cos(othervar),eq)]])
                                        cond=othervar+acos(eq).evalf()
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.IsHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(othervar,-acos(eq).evalf()),(sothervar,-sqrt(1-eq*eq).evalf()),(sin(othervar),-sqrt(1-eq*eq).evalf()),(cothervar,eq),(cos(othervar),eq)]])
                
            if not var in nextsolutions:
                newvars=curvars[:]
                newvars.remove(var)
                olddegeneratecases = self.degeneratecases
                self.degeneratecases = olddegeneratecases.clone()
                nextsolutions[var] = self.solveAllEquations(AllEquations,curvars=newvars,othersolvedvars=othersolvedvars+[var],solsubs=solsubs+[(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)],endbranchtree=endbranchtree,currentcases=currentcases)
                self.degeneratecases = olddegeneratecases
            if len(checkforzeros) > 0:
                hascheckzeros = True
                prevbranch=[SolverCheckZeros(jointname=var.name,jointcheckeqs=checkforzeros,nonzerobranch=[solution]+nextsolutions[var],zerobranch=prevbranch,thresh = 0.000001,anycondition=True)]
            else:
                prevbranch = [solution]+nextsolutions[var]

        if len(prevbranch) == 0:
            raise self.CannotSolveError('failed to add solution!')

        if len(currentcases) >= 4:
            print '4 levels deep in checking degenerate cases, skipping...'
            lastbranch.append(SolverBreak())
            return prevbranch

        # fill the last branch with all the zero conditions
        if hascheckzeros and len(eqs) == 0:
            # if not equations found, try setting two variables at once
            # also try setting px, py, or pz to 0 (barrettwam4 lookat)
            for solution,var in usedsolutions:
                for othervar in othersolvedvars:
                    if not self.IsHinge(othervar.name):
                        continue
                    sothervar = Symbol('s%s'%othervar.name)
                    cothervar = Symbol('c%s'%othervar.name)
                    if checkzero.has_any_symbols(othervar,sothervar,cothervar):
                        for preal in [Symbol('px'),Symbol('py'),Symbol('pz')]:
                            if checkzero.has_any_symbols(preal):
                                for value in [S.Zero,pi/2,pi,-pi/2]:
                                    eq = checkzero.subs([(othervar,value),(sothervar,sin(value).evalf()),(cothervar,cos(value).evalf()),(preal,S.Zero)]).evalf()
                                    if eq == S.Zero:
                                        cond = abs(othervar-value)+abs(preal)
                                        evalcond = abs(fmod(othervar-value+pi,2*pi)-pi)+abs(preal)
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            eqs.append([cond,evalcond,[(othervar,value),(sothervar,sin(value).evalf()),(sin(othervar),sin(value).evalf()),(cothervar,cos(value).evalf()),(cos(othervar),cos(value).evalf()),(preal,S.Zero)]])
                                            print '%s=%s,%s=0 in %s'%(str(othervar),str(value),str(preal),str(checkzero))

        # test the solutions
        zerobranches = []
        for cond,evalcond,othervarsubs in eqs:
            NewEquations = copy.copy(AllEquations)
            for i in range(len(NewEquations)):
                NewEquations[i] = NewEquations[i].subs(othervarsubs).evalf()
            try:
                # forcing a value, so have to check if all equations in NewEquations that do not contain
                # unknown variables are really 0
                extrazerochecks=[]
                for i in range(len(NewEquations)):
                    expr = NewEquations[i]
                    if not self.isValidSolution(expr):
                        print 'not valid',expr
                        extrazerochecks=None
                        break
                    if not expr.has_any_symbols(*allvars) and (self.isExpressionUnique(extrazerochecks,expr) or self.isExpressionUnique(extrazerochecks,-expr)):
                        extrazerochecks.append(expr.subs(solsubs).evalf())
                if extrazerochecks is not None:
                    newcases = set(currentcases)
                    newcases.add(cond)
                    zerobranches.append(([evalcond]+extrazerochecks,self.solveAllEquations(NewEquations,curvars,othersolvedvars,solsubs,endbranchtree,currentcases=newcases)))
                    self.degeneratecases.addcases(newcases)
            except self.CannotSolveError:
                continue

        if len(zerobranches) > 0:
            lastbranch.append(SolverBranchConds(zerobranches+[(None,[SolverBreak()])]))
        else:
            lastbranch.append(SolverBreak())

        return prevbranch

    def solveSingleVariable(self,raweqns,var):
        svar = Symbol('s%s'%var.name)
        cvar = Symbol('c%s'%var.name)
        varsubs = [(sin(var),svar),(cos(var),cvar)]
        eqns = [eq.expand() for eq in raweqns if eq.has_any_symbols(var)]
        if len(eqns) == 0:
            raise self.CannotSolveError('not enough equations')
        if len(eqns) > 1:
            neweqns = []
            listsymbols = []
            symbolgen = cse_main.numbered_symbols('const')
            for e in eqns:
                enew, symbols = self.removeConstants(e.subs(varsubs),[cvar,svar,var], symbolgen)
                # remove coupled equations
                if any([(m[0]>0)+(m[1]>0)+(m[2]>0)>1 for m in Poly(enew,cvar,svar,var).monoms]):
                    continue
                # ignore any equations with degree 3 or more
                if Poly(enew,svar).degree >= 3 or Poly(enew,cvar).degree >= 3:
                    print 'ignoring equation: ',enew
                    continue
                enew2,symbols2 = self.factorLinearTerms(enew,[svar,cvar,var],symbolgen)
                symbols += [(s[0],s[1].subs(symbols)) for s in symbols2]
                rank = self.codeComplexity(enew2)
                for s in symbols:
                    rank += self.codeComplexity(s[1])
                neweqns.append((rank,enew2))
                listsymbols += symbols
            # since we're solving for two variables, we only want to use two equations, so
            # start trying all the equations starting from the least complicated ones to the most until a solution is found
            eqcombinations = []
            for eqs in combinations(neweqns,2):
                eqcombinations.append((eqs[0][0]+eqs[1][0],[Eq(e[1],0) for e in eqs]))
            eqcombinations.sort(lambda x, y: x[0]-y[0])
            solutions = []
            for icomb,comb in enumerate(eqcombinations):
                # skip if too complex
                if len(solutions) > 0 and comb[0] > 200:
                    break
                # try to solve for both sin and cos terms
                try:
                    s = solve(comb[1],[svar,cvar])
                except PolynomialError, e:
                    print 'solveSingleVariable: ',e
                    continue
                if s is not None:
                    sollist = None
                    if hasattr(s,'has_key'):
                        if s.has_key(svar) and s.has_key(cvar):
                            sollist = [(s[svar],s[cvar])]
                        else:
                            sollist = []
                    else:
                        sollist = s
                    solversolution = SolverSolution(var.name,jointeval=[],IsHinge=self.IsHinge(var.name))
                    goodsolution = 0
                    for svarsol,cvarsol in sollist:
                        # solutions cannot be trivial
                        if self.chop((svarsol-cvarsol).subs(listsymbols)) == 0 or self.chop(svarsol.subs(listsymbols)) == 0 or self.chop(cvarsol.subs(listsymbols)) == 0:
                            break
                        # check the numerator and denominator if solutions are the same or for possible divide by zeros
                        svarfrac=fraction(svarsol)
                        svarfrac = [svarfrac[0].subs(listsymbols), svarfrac[1].subs(listsymbols)]
                        cvarfrac=fraction(cvarsol)
                        cvarfrac = [cvarfrac[0].subs(listsymbols), cvarfrac[1].subs(listsymbols)]
                        if self.chop(svarfrac[0]-cvarfrac[0]) == 0 and self.chop(svarfrac[1]-cvarfrac[1]) == 0:
                            break
                        if not self.isValidSolution(svarfrac[0]) or not self.isValidSolution(svarfrac[1]) or not self.isValidSolution(cvarfrac[0]) or not self.isValidSolution(cvarfrac[1]):
                            continue
                        scomplexity = self.codeComplexity(svarfrac[0])+self.codeComplexity(svarfrac[1])
                        ccomplexity = self.codeComplexity(cvarfrac[0])+self.codeComplexity(cvarfrac[1])
                        if scomplexity > 1200 or ccomplexity > 1200:
                            print 'equation too complex for single variable solution (%d,%d).... (probably wrong?)'%(scomplexity,ccomplexity)
                            break
                        svarfrac[1] = simplify(svarfrac[1])
                        if self.chop(svarfrac[1])== 0:
                            break
                        cvarfrac[1] = simplify(cvarfrac[1])
                        if self.chop(cvarfrac[1])== 0:
                            break
                        # sometimes the returned simplest solution makes really gross approximations
                        svarsol = svarfrac[0]/svarfrac[1]
                        cvarsol = cvarfrac[0]/cvarfrac[1]
                        expandedsol = atan2(svarsol,cvarsol)
                        if scomplexity < 700 and ccomplexity < 700:
                            # sometimes customtrigsimp freezes, so use trigsimp?
                            simpsol = atan2(trigsimp(svarsol),trigsimp(cvarsol))
                        else:
                            simpsol = expandedsol
                        if self.codeComplexity(expandedsol) < self.codeComplexity(simpsol):
                            solversolution.jointeval.append(expandedsol)
                            if len(self.checkForDivideByZero(expandedsol)) == 0:
                                goodsolution += 1
                        else:
                            solversolution.jointeval.append(simpsol)
                            if len(self.checkForDivideByZero(simpsol)) == 0:
                                goodsolution += 1
                    if len(solversolution.jointeval) == len(sollist) and len(sollist) > 0:
                        solutions.append(solversolution)
                        if len(sollist) == goodsolution and goodsolution == 1:
                            break
                        if len(solutions) >= 4:
                            # probably more than enough already?
                            break

            if len(solutions) > 0:
                return solutions

        # solve one equation
        solutions = []
        for eq in eqns:
            symbolgen = cse_main.numbered_symbols('const')
            eqnew, symbols = self.removeConstants(eq.subs([(sin(var),svar),(cos(var),cvar)]), [cvar,svar,var], symbolgen)
            try:
                # ignore any equations with degree 3 or more 
                ps = Poly(eqnew,svar)
                pc = Poly(eqnew,cvar)
                if ps.degree >= 3 or pc.degree >= 3:
                    print 'cannot solve equation with high degree'
                    continue
                if ps.coeff(0) == S.Zero and len(ps.monoms) > 0:
                    #print 'equation %s has trivial solution, ignoring...'%(str(ps))
                    continue
                if pc.coeff(0) == S.Zero and len(pc.monoms) > 0:
                    #print 'equation %s has trivial solution, ignoring...'%(str(pc))
                    continue
            except polys.polynomial.PolynomialError:
                # might not be a polynomial, so ignore
                continue
            
            eqnew2,symbols2 = self.factorLinearTerms(eqnew,[cvar,svar,var], symbolgen)
            symbols += [(s[0],s[1].subs(symbols)) for s in symbols2]
            numcvar = self.countVariables(eqnew2,cvar)
            numsvar = self.countVariables(eqnew2,svar)
            if numcvar == 1 and numsvar == 1:
                a = Wild('a',exclude=[svar,cvar])
                b = Wild('b',exclude=[svar,cvar])
                c = Wild('c',exclude=[svar,cvar])
                m = eqnew2.match(a*cvar+b*svar+c)
                if m is not None:
                    symbols += [(svar,sin(var)),(cvar,cos(var))]
                    asinsol = trigsimp(asin(-m[c]/abs(sqrt(m[a]*m[a]+m[b]*m[b]))).subs(symbols),deep=True)
                    constsol = -atan2(m[a],m[b]).subs(symbols).evalf()
                    jointsolutions = [constsol+asinsol,constsol+pi.evalf()-asinsol]
                    if all([self.isValidSolution(s) for s in jointsolutions]):
                        solutions.append(SolverSolution(var.name,jointeval=jointsolutions,IsHinge=self.IsHinge(var.name)))
                    continue
            if numcvar > 0:
                try:
                    # substitute cos
                    if self.countVariables(eqnew2,svar) <= 1 or (self.countVariables(eqnew2,cvar) <= 2 and self.countVariables(eqnew2,svar) == 0): # anything more than 1 implies quartic equation
                        tempsolutions = self.customtsolve(eqnew2.subs(svar,sqrt(1-cvar**2)),cvar)
                        jointsolutions = [self.customtrigsimp(s.subs(symbols+[(cvar,cos(var))])) for s in tempsolutions]
                        if all([self.isValidSolution(s) for s in jointsolutions]):
                            solutions.append(SolverSolution(var.name,jointevalcos=jointsolutions,IsHinge=self.IsHinge(var.name)))
                        continue
                except self.CannotSolveError:
                    pass
            if numsvar > 0:
                # substitute sin
                try:
                    if self.countVariables(eqnew2,svar) <= 1 or (self.countVariables(eqnew2,svar) <= 2 and self.countVariables(eqnew2,cvar) == 0): # anything more than 1 implies quartic equation
                        tempsolutions = self.customtsolve(eqnew2.subs(cvar,sqrt(1-svar**2)),svar)
                        jointsolutions = [self.customtrigsimp(s.subs(symbols+[(svar,sin(var))])) for s in tempsolutions]
                        if all([self.isValidSolution(s) for s in jointsolutions]):
                            solutions.append(SolverSolution(var.name,jointevalsin=jointsolutions,IsHinge=self.IsHinge(var.name)))
                        continue
                except self.CannotSolveError:
                    pass
            if numcvar == 0 and numsvar == 0:
                tempsolutions = self.customtsolve(eqnew2,var)
                jointsolutions = [self.customtrigsimp(s.subs(symbols)) for s in tempsolutions]
                if all([self.isValidSolution(s) for s in jointsolutions]):
                    solutions.append(SolverSolution(var.name,jointeval=jointsolutions,IsHinge=self.IsHinge(var.name)))
                continue
        if len(solutions) > 0:
            return solutions
        raise self.CannotSolveError('cannot solve equations %s'%str(raweqns))

    def solvePairVariables(self,raweqns,var0,var1):
        cvar0 = Symbol('c%s'%var0.name)
        svar0 = Symbol('s%s'%var0.name)
        cvar1 = Symbol('c%s'%var1.name)
        svar1 = Symbol('s%s'%var1.name)
        varsubs=[(cos(var0),cvar0),(sin(var0),svar0),(cos(var1),cvar1),(sin(var1),svar1)]
        varsubsinv = [(f[1],f[0]) for f in varsubs]
        unknownvars=[v[1] for v in varsubs]
        eqns = [self.removecommonexprs(eq.subs(varsubs)) for eq in raweqns if eq.has_any_symbols(var0,var1)]
        if len(eqns) <= 1:
            raise self.CannotSolveError('not enough equations')

        # group equations with single variables
        symbolgen = cse_main.numbered_symbols('const')
        orgeqns = []
        allsymbols = []
        for eq in eqns:
            eqnew, symbols = self.removeConstants(eq, unknownvars, symbolgen)
            eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
            allsymbols += symbols + [(s[0],s[1].subs(symbols)) for s in symbols2]
            p = Poly(eqnew2,*unknownvars)
            # make sure there are no monomials that have more than 2 diferent terms
            if all([__builtin__.sum([m[j]>0 for j in range(len(unknownvars))])<=2 for m in p.monoms]):
                orgeqns.append([self.codeComplexity(eq),Poly(eqnew2,*unknownvars)])
        orgeqns.sort(lambda x, y: x[0]-y[0])
        neweqns = orgeqns[:]

        pairwisesubs = [(svar0*cvar1,Symbol('s0c1')),(svar0*svar1,Symbol('s0s1')),(cvar0*cvar1,Symbol('c0c1')),(cvar0*svar1,Symbol('c0s1'))]
        pairwiseinvsubs = [(f[1],f[0]) for f in pairwisesubs]
        pairwisevars = [f[1] for f in pairwisesubs]
        reduceeqns = [Poly(eq.as_basic().subs(pairwisesubs),*pairwisevars) for rank,eq in orgeqns]
        for i,eq in enumerate(reduceeqns):
            if eq.TC != S.Zero and not eq.TC.is_Symbol:
                n=symbolgen.next()
                allsymbols.append((n,eq.TC.subs(allsymbols)))
                reduceeqns[i] += n-eq.TC

        # try to at least subtract as much paired variables out
        eqcombs = [c for c in combinations(reduceeqns,2)]
        while len(eqcombs) > 0:
            eq0,eq1 = eqcombs.pop()
            for i in range(4):
                monom = [0,0,0,0]
                monom[i] = 1
                if eq0.coeff(*monom) != 0 and eq1.coeff(*monom) != 0:
                    eq = simplify((eq0.as_basic()*eq1.coeff(*monom)-eq0.coeff(*monom)*eq1.as_basic()).subs(allsymbols+pairwiseinvsubs))
                    #eq = self.chop(self.removecommonexprs(eq),accuracy=self.accuracy**2)
                    if not self.isExpressionUnique(eqns,eq) or not self.isExpressionUnique(eqns,-eq):
                        continue
                    if self.codeComplexity(eq) > 50:
                        # don't need such complex equations
                        #print self.codeComplexity(eq)#, eq
                        continue
                    if eq.has_any_symbols(*unknownvars):
                        eqns.append(eq)
                        eqnew, symbols = self.removeConstants(eq, unknownvars, symbolgen)
                        eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
                        allsymbols += symbols + [(s[0],s[1].subs(symbols)) for s in symbols2]
                        neweqns.append([self.codeComplexity(eq),Poly(eqnew2,*unknownvars)])
#                    reducedeq = Poly(eqnew2.subs(pairwisesubs),*pairwisevars)
#                     for e in reduceeqns:
#                         if e != eq0 and e != eq1:
#                             eqcombs.append((reducedeq,e))

        # try single variable solution
        try:
            return self.solveSingleVariable([e.subs(varsubsinv) for e in eqns if not e.has_any_symbols(cvar1,svar1,var1)],var0)
        except self.CannotSolveError:
            pass

        try:
            return self.solveSingleVariable([e.subs(varsubsinv) for e in eqns if not e.has_any_symbols(cvar0,svar0,var0)],var1)
        except self.CannotSolveError:
            pass

        orgeqns = neweqns[:]
        # reduce the equations so that we get X+a*(sin|cos)+b=0
        singleeqs = None
        # try to solve for all pairwise variables
        for eqs in combinations(reduceeqns,4):
            # count all non-zero coeffs to determine complexity of equations
            nummonoms = __builtin__.sum([len(eq.monoms) for eq in eqs]), eqs
            if nummonoms > 16:
                continue # too much, solve will never finish.. i think, at least it doesn't when nummonoms=20
            solution=solve(eqs,pairwisevars)
            if solution is not None:
                if not hasattr(solution,'has_key'):
                    sollist = solution[0]
                    solution = {}
                    for i in range(len(sollist)):
                        solution[pairwisevars[i]] = sollist[i]
                if all([not s.has_any_symbols(*pairwisevars) for s in solution.itervalues()]):
                    singleeqs = []
                    for v,s in solution.iteritems():
                        snum,sdenom = fraction(s.subs(allsymbols))
                        eq = simplify(snum) - simplify(sdenom)*v.subs(pairwiseinvsubs)
                        eqnew, symbols = self.removeConstants(eq, unknownvars, symbolgen)
                        eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
                        allsymbols += symbols + [(s2[0],s2[1].subs(symbols)) for s2 in symbols2]
                        singleeqs.append([self.codeComplexity(eq),Poly(eqnew2,*unknownvars)])
                    break
        if singleeqs is not None:
            neweqns += singleeqs
            neweqns.sort(lambda x, y: x[0]-y[0])
#             for (rank0,eq0),(rank1,eq1) in combinations(singleeqs,2):
#                 seq=(eq0.sub_term(*eq0.lead_term)**2+eq1.sub_term(*eq1.lead_term)**2).subs(allsymbols)
#                 print self.customtrigsimp(seq).monoms

        groups=[]
        for i,unknownvar in enumerate(unknownvars):
            listeqs = []
            for rank,eq in neweqns:
                # if variable ever appears, it should be alone
                if all([m[i] == 0 or (__builtin__.sum(m) == m[i] and m[i]>0) for m in eq.iter_monoms()]) and any([m[i] > 0 for m in eq.iter_monoms()]):
                    # make sure there's only one monom that includes other variables
                    othervars = [__builtin__.sum(m) - m[i] > 0 for m in eq.iter_monoms()]
                    if __builtin__.sum(othervars) <= 1:
                        listeqs.append(eq)
            groups.append(listeqs)
        # find a group that has two or more equations:
        useconic=False
        goodgroup = [(i,g) for i,g in enumerate(groups) if len(g) >= 2]
        if len(goodgroup) == 0:
            # might have a set of equations that can be solved with conics
            # look for equations where the variable and its complement are alone
            groups=[]
            for i in [0,2]:
                unknownvar = unknownvars[i]
                complementvar = unknownvars[i+1]
                listeqs = []
                for rank,eq in neweqns:
                    # if variable ever appears, it should be alone
                    if all([(m[i] == 0 and m[i+1]==0) or __builtin__.sum(m) == m[i]+m[i+1] for m in eq.iter_monoms()]):
                        # make sure there's only one monom that includes other variables
                        othervars = [__builtin__.sum(m) - m[i]-m[i+1] > 0 for m in eq.iter_monoms()]
                        if __builtin__.sum(othervars) <= 1:
                            listeqs.append(eq)
                groups.append(listeqs)
                groups.append([]) # necessary to get indices correct
            goodgroup = [(i,g) for i,g in enumerate(groups) if len(g) >= 2]
            useconic=True
            if len(goodgroup) == 0:
                print 'attempting to isolate a variable'
                finalsolutions = []
                for i in range(4): # for every variable
                    # if variable ever appears, it should be alone
                    complementvar = unknownvars[[1,0,3,2][i]]
                    print 'var:',unknownvars[i]
                    varsol = None
                    for rank,eq in orgeqns:
                        if eq.has_any_symbols(unknownvars[i]):
                            solutions = solve(eq.as_basic(),unknownvars[i])
                            for solution in solutions:
                                tempsol = solution.subs(allsymbols)
                                if not fraction(tempsol)[1].has_any_symbols(complementvar):
                                    varsol = tempsol
                                    break
                        if varsol is not None:
                            break
                    if varsol is not None:
                        #eq=simplify(fraction(varsol)[0]**2 + fraction(varsol)[1]**2*(complementvar**2 - 1))
                        varsolvalid=fraction(varsol)[1]
                        valideqs = []
                        for rank,eq in orgeqns:
                            # find the max degree tha unknownvars[i] appears
                            maxdegree = max([m[i] for m in eq.iter_monoms()])
                            if maxdegree <= 1:
                                # simplify with just the symbol
                                eqsub = self.customtrigsimp(Symbol('tempsym')**maxdegree*eq.as_basic().subs(allsymbols+[(unknownvars[i],varsol)]).subs(fraction(varsol)[1],Symbol('tempsym')))
                                eqnew = simplify(eqsub.subs(Symbol('tempsym'),fraction(varsol)[1]))
                                if eqnew != S.Zero and self.isExpressionUnique(valideqs,eqnew):
                                    valideqs.append(eqnew)
                        valideqs2 = []
                        for eq in valideqs:
                            eqnew, symbols = self.removeConstants(eq, unknownvars, symbolgen)
                            eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
                            # only accept simple equations
                            if self.codeComplexity(eqnew2) < 100:
                                allsymbols += symbols + [(s2[0],s2[1].subs(symbols)) for s2 in symbols2]
                                valideqs2.append(eqnew2)
                        self.sortComplexity(valideqs2)
                        complementvarsols = []
                        othervarpoly = None                        
                        othervars = unknownvars[0:2] if i >= 2 else unknownvars[2:4]
                        postcheckforzeros = []
                        postcheckforrange = []
                        postcheckfornonzeros = []
                        for eq in valideqs2:
                            if eq.has_any_symbols(complementvar):
                                solutions = solve(eq,complementvar)
                                if len(solutions) > 0 and solutions[0].subs(allsymbols) != S.Zero:
                                    complementvarsols.append(solutions[0].subs(allsymbols))
                                    if len(complementvarsols) >= 2:
                                        # test new solution with previous ones
                                        eq0num,eq0denom = fraction(complementvarsols[-1].subs(allsymbols))
                                        for eq1 in complementvarsols[:-1]:
                                            eq1num,eq1denom = fraction(eq1.subs(allsymbols))
                                            # although not apparent, this is actually a dangerous transformation that allows
                                            # wrong solutions to pass through since complementvar is actually constrained, but the constraint
                                            # is ignored. Therefore, this requires us to explicitly check denominator for zero and
                                            # that each solution is within the [-1,1] range.
                                            neweq = eq0num*eq1denom-eq1num*eq0denom
                                            if self.codeComplexity(neweq.expand()) < 700:
                                                neweq = simplify(neweq)
                                            neweq = self.chop(neweq,accuracy=self.accuracy*0.01)
                                            if neweq != S.Zero:
                                                othervarpoly = Poly(neweq,*othervars).subs(othervars[0]**2,1-othervars[1]**2).subs(othervars[1]**2,1-othervars[0]**2)
                                                if othervarpoly != S.Zero:
                                                    postcheckforzeros = [varsolvalid, eq0denom, eq1denom]
                                                    postcheckfornonzeros = [eq1**2+varsol.subs(complementvar,eq1)**2-1]
                                                    break
                                                else:
                                                    othervarpoly = None
                                        if othervarpoly is not None:
                                            break
                        if othervarpoly is not None:
                            # now we have one polynomial with only one variable (sin and cos)!, try to remove one
                            othervarpoly0 = Poly(othervarpoly,othervars[0])
                            othervarpoly_simp = None
                            solvevar = None
                            if othervarpoly0.degree == 1:
                                othervarpoly_simp = othervarpoly0
                                solvevar = othervars[1]
                            else:
                                othervarpoly1 = Poly(othervarpoly,othervars[1])
                                if othervarpoly1.degree == 1:
                                    othervarpoly_simp = othervarpoly1
                                    solvevar = othervars[0]
                            if solvevar is not None:
                                # note that squaring on both sides introduces bad solutions!
                                # chop with a very small threshold
                                finaleq_expand = self.chop(((solvevar**2-S.One)*othervarpoly_simp.coeff(1)**2 + othervarpoly_simp.coeff(0)**2).expand(),accuracy=self.accuracy**2)
                                if self.codeComplexity(finaleq_expand) < 4000:
                                    print 'simplifying final equation',self.codeComplexity(finaleq_expand)
                                    finaleq_expand = simplify(finaleq_expand)
                                finaleq = Poly(finaleq_expand,solvevar)
                                # finaleq can be pretty big degree!
                                print 'deg: ',finaleq.degree
                                complementvarsol = -othervarpoly_simp.coeff(0)/othervarpoly_simp.coeff(1)
                                if solvevar.name[0] == 'c':
                                    jointsol = atan2(complementvarsol,solvevar)
                                else:
                                    assert solvevar.name[0] == 's'
                                    jointsol = atan2(solvevar,complementvarsol)
                                var=var1 if i < 2 else var0
                                solution = SolverPolynomialRoots(jointname=var.name,poly=finaleq,jointeval=[jointsol],IsHinge=self.IsHinge(var.name))
                                solution.postcheckforzeros = postcheckforzeros
                                solution.postcheckfornonzeros = postcheckfornonzeros
                                solution.postcheckforrange = postcheckforrange
                                finalsolutions.append(solution)
                                if finaleq.degree <= 2:
                                    # found a really good solution, so choose it
                                    break
                            else:
                                print 'othervarpoly too complex: ',othervarpoly
                if len(finalsolutions) > 0:
                    # find the equation with the minimal degree, and the least code complexity
                    return [min(finalsolutions, key=lambda f: f.poly.degree*1e6 + self.codeComplexity(f.poly.as_basic()))]
                else:
                    raise self.CannotSolveError('cannot cleanly separate pair equations')


        varindex=goodgroup[0][0]
        var = var0 if varindex < 2 else var1
        unknownvar=unknownvars[goodgroup[0][0]]
        rawsolutions = None
        # try single variable solution
        try:
            rawsolutions=self.solveSingleVariable([e.as_basic().subs(allsymbols+varsubsinv) for e in goodgroup[0][1] if not e.has_any_symbols(cvar1,svar1,var1)],var0)
        except self.CannotSolveError:
            pass

        try:
            rawsolutions = self.solveSingleVariable([e.as_basic().subs(allsymbols+varsubsinv) for e in goodgroup[0][1] if not e.has_any_symbols(cvar0,svar0,var0)],var1)
        except self.CannotSolveError:
            pass

        if rawsolutions is not None:
            solutions = []
            for rawsolution in rawsolutions:
                try:
                    solutions.append(rawsolution.subs(allsymbols))
                except self.CannotSolveError:
                    pass
                
            return solutions

        eqs = goodgroup[0][1][0:2]
        simpleterms = []
        complexterms = []
        for i in range(2):
            if useconic:
                terms=[(c,m) for c,m in eqs[i].iter_terms() if __builtin__.sum(m) - m[varindex] - m[varindex+1] > 0]
            else:
                terms=[(c,m) for c,m in eqs[i].iter_terms() if __builtin__.sum(m) - m[varindex] > 0]
            simpleterms.append(eqs[i].sub_term(*terms[0]).as_basic()/terms[0][0]) # divide by the coeff
            complexterms.append(Poly(0,*unknownvars).add_term(S.One,terms[0][1]).as_basic())
        # here is the magic transformation:
        finaleq = self.customtrigsimp(self.customtrigsimp(expand(((complexterms[0]**2+complexterms[1]**2) - simpleterms[0]**2 - simpleterms[1]**2).subs(varsubsinv)))).subs(varsubs)
        finaleq = simplify(finaleq*(fraction(simpleterms[0])[1]**2) * (fraction(simpleterms[1])[1]**2) * (fraction(complexterms[0])[1]**2) * (fraction(complexterms[1])[1]**2))
        complementvarindex = varindex-(varindex%2)+((varindex+1)%2)
        complementvar = unknownvars[complementvarindex]
        finaleq = expand(simplify(simplify(finaleq.subs(complementvar**2,1-unknownvar**2)).subs(allsymbols)))
        if not self.isValidSolution(finaleq):
            raise self.CannotSolveError('failed to solve pairwise equation: %s'%str(finaleq))
        if useconic:
            if not self.IsHinge(var.name):
                print 'got conic equation from a non-hinge joint?: ',finaleq
            return [SolverConicRoots(var.name,[finaleq],IsHinge=self.IsHinge(var.name))]
        newunknownvars = unknownvars[:]
        newunknownvars.remove(unknownvar)
        if finaleq.has_any_symbols(*newunknownvars):
            raise self.CannotSolveError('ray ik bad equation %s'%str(finaleq))
        # now that everything is with respect to one variable, simplify and solve the equation
        eqnew, symbols = self.removeConstants(finaleq, unknownvars, symbolgen)
        eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
        allsymbols += symbols + [(s[0],s[1].subs(symbols)) for s in symbols2]
        solutions=solve(eqnew2,unknownvar)
        print 'pair solution: ',eqnew2,',',solutions
        if solutions:
            
            solversolution=SolverSolution(var.name, IsHinge=self.IsHinge(var.name))
            if (varindex%2)==0:
                solversolution.jointevalcos=[self.customtrigsimp(s.subs(allsymbols+varsubsinv)).subs(varsubs) for s in solutions]
            else:
                solversolution.jointevalsin=[self.customtrigsimp(s.subs(allsymbols+varsubsinv)).subs(varsubs) for s in solutions]
            return [solversolution]
        raise self.CannotSolveError('cannot solve pair equation')
        
    def solveIKRotation(self, R, Ree, rawvars,endbranchtree=None,solvedvarsubs=[],ignorezerochecks=[]):
        """Solve for the rotation component"""
        vars = [self.Variable(rawvar) for rawvar in rawvars]
        subreal = [(cos(var.var),var.cvar) for var in vars]+[(sin(var.var),var.svar) for var in vars]
        subrealinv = [(var.cvar,cos(var.var)) for var in vars]+[(var.svar,sin(var.var)) for var in vars]
        Rsolve = R.subs(subreal)
        roottree = []
        curtree = roottree
        invsolvedvarsubs = [(s[1],s[0]) for s in solvedvarsubs]

        while len(vars)>0:
            solvedvars, checkzerovars = self.checkStandaloneVariables(R, Rsolve, Ree, vars)
            solvedvars2 = self.checkQuotientVariables(R, Rsolve, Ree, vars)

            # for every solved variable in solvedvars, check that there isn't a cleaner solution in solvedvars2
            for var,solutions in solvedvars.iteritems():
                if solvedvars2.has_key(var) and not solvedvars2[var][1]:
                    if self.codeComplexity(solutions[0]) > self.codeComplexity(solvedvars2[var][0][0]):
                        solvedvars[var] = solvedvars2[var][0] # replace

            # check if there is a divide by zero
            for var,solutions in solvedvars.iteritems():
                jointbranches = []
                for solution in solutions:
                    checkforzero = self.checkForDivideByZero(self.customtrigsimp(solution.subs(subrealinv+invsolvedvarsubs),deep=True))
                    if len(checkforzero) > 0:
                        for sraw in checkforzero:
                            s = sraw.subs(solvedvarsubs)
                            if self.isExpressionUnique(ignorezerochecks+[b[0][0] for b in jointbranches],s):
                                Rsubs = Matrix(R.shape[0],R.shape[1],[x.subs(s,0) for x in R])
                                if not all([r==0 for r in R-Rsubs]):
                                    addignore = []
                                    if s.is_Function:
                                        if s.func == cos:
                                            addignore.append(sin(*s.args))
                                        elif s.func == sin:
                                            addignore.append(cos(*s.args))
                                    jointbranches.append(([s],self.solveIKRotation(Rsubs,Ree,[v.var for v in vars],endbranchtree,solvedvarsubs,ignorezerochecks+addignore)))
                if len(jointbranches) > 0:
                    newrawvars = [v.var for v in vars if v.var != var]
                    curtree.append(SolverBranchConds(jointbranches+[(None,[SolverSolution(var.name, jointeval=solutions)]+self.solveIKRotation(R,Ree,newrawvars,endbranchtree,solvedvarsubs,ignorezerochecks))]))
                    return roottree
                else:
                    curtree.append(SolverSolution(var.name, jointeval=solutions))
                vars = [v for v in vars if v.var!=var]

            if len(vars) == 0:
                break

            solvedvars2 = self.checkQuotientVariables(R, Rsolve, Ree, vars)
            if len(solvedvars2) == 0:
                newrawvars = [var.var for var in vars]
                #print 'picking free parameter for potential degenerate case: ',newrawvars[0]
                #print 'R = '
                #pprint(R)
                tree = self.solveIKRotation(R,Ree,newrawvars[1:],endbranchtree,solvedvarsubs,ignorezerochecks)
                curtree.append(SolverFreeParameter(newrawvars[0].name,tree))
                return roottree

            nextvars = vars[:]
            nextsol = None
            for var,solutions in solvedvars2.iteritems():
                if not solutions[1]:
                    nextsol = (var,solutions[0],None)
                    break
            if not nextsol:
                for var,solutions in solvedvars2.iteritems():
                    if solutions[1]:
                        nextsol = (var, solutions[0], solutions[2])
                        break

            # for each solution in nexttree, check for divides by 0
            nexttree = [SolverSolution(nextsol[0].name, jointeval=nextsol[1],AddPiIfNegativeEq=nextsol[2])]
            nextvars = filter(lambda x: not x.var == nextsol[0], nextvars)
            jointbranches = []
            for solution in nextsol[1]:
                checkforzero = self.checkForDivideByZero(self.customtrigsimp(solution.subs(subrealinv+invsolvedvarsubs),deep=True))
                if len(checkforzero) > 0:
                    for sraw in checkforzero:
                        s = sraw.subs(solvedvarsubs)
                        if self.isExpressionUnique(ignorezerochecks+[b[0][0] for b in jointbranches],s):
                            Rsubs = Matrix(R.shape[0],R.shape[1],[x.subs(s,0) for x in R])
                            if not all([r==0 for r in R-Rsubs]):
                                addignore = []
                                if s.is_Function:
                                    if s.func == cos:
                                        addignore.append(sin(*s.args))
                                    elif s.func == sin:
                                        addignore.append(cos(*s.args))
                                jointbranches.append(([s],self.solveIKRotation(Rsubs,Ree,[v.var for v in vars],endbranchtree,solvedvarsubs,ignorezerochecks+addignore)))
            if len(jointbranches) > 0:
                curtree.append(SolverBranchConds(jointbranches+[(None,nexttree)]))
                curtree = nexttree
                nexttree = []

            # if checkzerovars1 are 0 or pi/2, substitute with new R2 and resolve
            if len(checkzerovars) > 0:
                checkvar = checkzerovars[0][0]
                checkeq = checkzerovars[0][1]
                if checkeq.is_Function and checkeq.func == asin:
                    checkeq = checkeq.args[0]
                    valuestotest = [(-1,[-pi/2]),(1,[pi/2])]
                elif checkeq.is_Function and checkeq.func == acos:
                    checkeq = checkeq.args[0]
                    valuestotest = [(-1,[pi]),(1,[0])]
                elif checkeq.is_Mul and len(checkeq.args)>1 and checkeq.args[0].is_number:
                    if checkeq.args[1].is_Function and checkeq.args[1].func == asin:
                        c = checkeq.args[0].evalf()
                        checkeq = checkeq.args[1].args[0]
                        valuestotest = [(-c,[-pi/2]),(c,[pi/2])]
                    elif checkeq.args[1].is_Function and checkeq.args[1].func == acos:
                        c = checkeq.args[0].evalf()
                        checkeq = checkeq.args[1].args[0]
                        valuestotest = [(-c,[pi]),(c,[0])]
                else:
                    print 'unknown function when checking for 0s',checkeq
                    valuestotest = [(0,[checkeq.subs(checkvar,0)])]
                
                newrawvars = [var.var for var in vars if not var.var == checkvar]
                subtrees = []
                for checkvalue in valuestotest:
                    subtree = []
                    for value in checkvalue[1]:
                        subtree += [[SolverSetJoint(checkvar,value)]+self.solveIKRotation(R.subs(checkvar,value), Ree, newrawvars,endbranchtree,solvedvarsubs,ignorezerochecks)]
                    subtrees.append((checkvalue[0],[SolverSequence(subtree)]))
                branchtree = SolverBranch(checkvar, checkeq, subtrees+[(None,nexttree)])
                curtree.append(branchtree)
                curtree = nexttree
            else:
                curtree += nexttree
            vars = nextvars

        if endbranchtree is not None:
            curtree += endbranchtree
        return roottree
            
    # look for individual elements that have only one symbol
    def checkStandaloneVariables(self, R, Rsolve, Ree, vars):
        solvedvars = dict()
        checkzerovars = []
        
        for var in vars:
            othervars = [v.var for v in vars if not v == var]
            inds = [i for i in range(R.shape[0]*R.shape[1]) if R[i].has_any_symbols(var.var) and (len(othervars) == 0 or not R[i].has_any_symbols(*othervars))]
            if len(inds) == 0:
                continue
            
            if len(inds) > 2:
                inds = inds[0:2]

            if len(inds) == 1 and Rsolve[inds[0]].has_any_symbols(var.svar) and Rsolve[inds[0]].has_any_symbols(var.cvar):
                # have to solve with atan2
                a = Wild('a',exclude=[var.svar,var.cvar])
                b = Wild('b',exclude=[var.svar,var.cvar])
                c = Wild('c',exclude=[var.svar,var.cvar])
                m = (Rsolve[inds[0]]-Ree[inds[0]]).match(a*var.cvar+b*var.svar+c)
                if m is not None:
                    symbols = [(var.svar,sin(var.var)),(var.cvar,cos(var.var))]
                    asinsol = asin(-m[c]/abs(sqrt(m[a]*m[a]+m[b]*m[b]))).subs(symbols)
                    constsol = -atan2(m[a],m[b]).subs(symbols).evalf()
                    solvedvars[var.var] = [constsol+asinsol,constsol+pi.evalf()-asinsol]
                    continue
            solution = solve([Eq(Rsolve[i],Ree[i]) for i in inds],[var.svar,var.cvar])
            if solution is None:
                continue
            if solution.has_key(var.svar) and solution.has_key(var.cvar):
                solvedvars[var.var] = [self.customtrigsimp(atan2(self.customtrigsimp(solution[var.svar]), self.customtrigsimp(solution[var.cvar])), deep=True)]
            else:
                # although variable cannot be solved completely,
                # can check for multiples of pi/2 and eliminate possible 0s
                if solution.has_key(var.svar):
                    checkzerovars.append((var.var, asin(solution[var.svar])))
                elif solution.has_key(var.cvar):
                    checkzerovars.append((var.var, acos(solution[var.cvar])))
        
        return solvedvars, checkzerovars

    def checkQuotientVariables(self, R, Rsolve, Ree, vars):
        # look for quotient of two elements that has only one element
        solvedvars = dict()

        for var in vars:
            othervars = [v.var for v in vars if not v == var]+[v.cvar for v in vars if not v == var]+[v.svar for v in vars if not v == var]

            listsymbols = []
            newR = []
            symbolgen = cse_main.numbered_symbols('const')
            for i in range(R.shape[0]*R.shape[1]):
                enew, symbols = self.removeConstants(Rsolve[i],[var.cvar,var.svar,var.var], symbolgen)
                enew2,symbols2 = self.factorLinearTerms(enew,[var.svar,var.cvar,var.var],symbolgen)
                symbols += [(s[0],s[1].subs(symbols)) for s in symbols2]
                #rank = self.codeComplexity(enew2)+reduce(lambda x,y: x+self.codeComplexity(y[1]),symbols,0)
                newR.append(enew2)
                listsymbols += symbols

            havegoodsol = False
            for i in range(R.shape[0]*R.shape[1]):
                if not R[i].has_any_symbols(var.var):
                    continue
                for i2 in [i]:#range(R.shape[0]*R.shape[1]):
                    for isign in [1.0]:#[-1.0,1.0]:
                        for j in range(i+1,R.shape[0]*R.shape[1]):
                            if not R[j].has_any_symbols(var.var):
                                continue
                            for j2 in [j]:#range(i+1,R.shape[0]*R.shape[1]):
                                for jsign in [1.0]:#[-1.0,1.0]:
                                    solution = solve([Eq(newR[i]+isign*newR[i2],Ree[i]+isign*Ree[i2]), Eq(newR[j]+jsign*newR[j2],Ree[j]+jsign*Ree[j2])],[var.svar,var.cvar])
                                    if solution is not None and solution.has_key(var.svar) and solution.has_key(var.cvar):
                                        divisor = self.customtrigsimp(solution[var.cvar].subs(listsymbols))
                                        numerator = self.customtrigsimp(solution[var.svar].subs(listsymbols))
                                        simplified = self.customtrigsimp(numerator/divisor,deep=True)
                                        if len(othervars) == 0 or not simplified.has_any_symbols(*othervars):
                                            varsolution = self.customtrigsimp(atan2(numerator, divisor), deep=True)
                                            if len(othervars) > 0 and varsolution.has_any_symbols(*othervars):
                                                if not havegoodsol: 
                                                    # have to use the simplified solution, but this might remove negative terms ie:
                                                    # atan2(-_r01/sj5, -_r21/sj5)
                                                    # therefore set all the unknown variables to 1 (ie something positive so that atan2 doesn't flip)
                                                    othervarsubs = [(v,S.One) for v in othervars]
                                                    #solvedvars[var.var] = [[atan2(*fraction(simplified))],True,divisor]
                                                    if var.var in solvedvars:
                                                        # have to check code complexity
                                                        if self.codeComplexity(varsolution) > self.codeComplexity(solvedvars[var.var][0][0]):
                                                            continue
                                                    solvedvars[var.var] = [[varsolution.subs(othervarsubs)],True,divisor]
                                            else:
                                                if var.var in solvedvars:
                                                    # have to check code complexity
                                                    if self.codeComplexity(varsolution) > self.codeComplexity(solvedvars[var.var][0][0]):
                                                        continue
                                                solvedvars[var.var] = [[varsolution],False,divisor]
                                                havegoodsol = True
        
        return solvedvars

    ## SymPy helper routines

    @staticmethod
    def isValidSolution(expr):
        """return true if solution does not contain any nan or inf terms"""
        if expr.is_number:
            e=expr.evalf()
            if e.has(I) or math.isinf(e) or math.isnan(e):
                return False
        for arg in expr.args:
            if not IKFastSolver.isValidSolution(arg):
                return False
        return True

    @staticmethod
    def factorLinearTerms(expr,vars,symbolgen = None):
        """factors linear terms together
        """
        if symbolgen is None:
            symbolgen = cse_main.numbered_symbols('const')

        # for better factoring, try to decompose into polynomial
        try:
            pexpr = Poly(expr,*vars)
            newexpr = Poly(S.Zero,*vars)
            symbols = []
            for coeff, monom in pexpr.iter_terms():
                if coeff.is_Symbol:
                    newexpr = newexpr.add_term(coeff,monom)
                else:
                    c = symbolgen.next()
                    newexpr = newexpr.add_term(c,monom)
                    symbols.append((c,coeff))
            return newexpr.as_basic(),symbols
        except PolynomialError, e:
            pass

        if not expr.is_Add:
            return expr,[]
        
        cexprs = dict()
        newexpr = S.Zero
        symbols = []
        for term in expr.args:
            if term.is_Mul:
                termconst = []
                termlinear = []
                termquad = []
                termother = []
                for x in term.args:
                    haslinear = any([x - var == 0 for var in vars])
                    hasquad = any([x - var*var == 0 for var in vars])
                    if haslinear and not hasquad:
                        termquad.append(x)
                    elif not haslinear and hasquad:
                        termquad.append(x)
                    elif x.has_any_symbols(*vars):
                        termother.append(x)
                        break
                    else:
                        termconst.append(x)
                
                if len(termother) == 0 and len(termlinear) == 1 and len(termquad) == 0:
                    if cexprs.has_key(termlinear[0]):
                        cexprs[termlinear[0]] += term/termlinear[0]
                    else:
                        cexprs[termlinear[0]] = term/termlinear[0]
                elif len(termother) == 0 and len(termlinear) == 0 and len(termquad) == 1:
                    if cexprs.has_key(termquad[0]):
                        cexprs[termquad[0]] += term/termquad[0]
                    else:
                        cexprs[termquad[0]] = term/termquad[0]
                else:
                    newexpr += term
            elif any([term - var == 0 for var in vars]) or any([term - var*var == 0 for var in vars]):
                if cexprs.has_key(term):
                    cexprs[term] += S.One
                else:
                    cexprs[term] = S.One
            else:
                newexpr += term
        
        for var,cexpr in cexprs.iteritems():
            c = symbolgen.next()
            newexpr += c*var
            symbols.append((c,cexpr))
        return newexpr,symbols

    @staticmethod
    def removeConstants(expr,vars,symbolgen = None):
        """Separates all terms that do have var in them"""
        if symbolgen is None:
            symbolgen = cse_main.numbered_symbols('const')
        if expr.is_Add:
            newexpr = S.Zero
            cexpr = S.Zero
            symbols = []
            for term in expr.args:
                if term.has_any_symbols(*vars):
                    expr2, symbols2 = IKFastSolver.removeConstants(term,vars,symbolgen)
                    newexpr += expr2
                    symbols += symbols2
                else:
                    cexpr += term

            if not cexpr == 0:
                c = symbolgen.next()
                newexpr += c
                symbols.append((c,cexpr))
            return newexpr,symbols
        elif expr.is_Mul:
            newexpr = S.One
            cexpr = S.One
            symbols = []
            for term in expr.args:
                if term.has_any_symbols(*vars):
                    expr2, symbols2 = IKFastSolver.removeConstants(term,vars,symbolgen)
                    newexpr *= expr2
                    symbols += symbols2
                else:
                    cexpr *= term

            if not cexpr == 0:
                c = symbolgen.next()
                newexpr *= c
                symbols.append((c,cexpr))
            return newexpr,symbols
        return expr,[]

    def subsExpressions(self, expr, subexprs):
        changed = True
        while changed:
            newexpr = expr.subs(subexprs)
            if simplify(expr-newexpr) == 0:
                break
            expr = newexpr
        return expr

#     @staticmethod
#     def sub_in(expression, pattern, replacement, match=True):
#         """ Searches through the expression looking for the pattern and will
#         replace the pattern with replacement if it finds it. Use wilds to write
#         your pattern and replacement. Ex:
#           x,y = symbols('xy'); j = Wild('j'); k = Wild('k')
#           sub_in(exp(x*y), exp(j*k), cos(j-k))
#         """
#         # Might take out match optionality because will always need it.
#         if match:
#             check = lambda expression, pattern: expression.match(pattern)
#         else:
#             check = lambda expression, pattern: isinstance(expression,pattern)
#         new = IKFastSolver._walk_it(expression, pattern, replacement, check)
#         if new != None: return new
#         else: return None
# 
#     @staticmethod
#     def _walk_it(expression, pattern, replacement, check):
#         """ Helper method for sub_in """
#         # Grab the operation of the expression
#         op = expression.__class__
# 
#         """ If the length of the expression and pattern are the same, then
#         just scan through to make sure everything in the pattern is present in the
#         expression. If so, then replace the wilds in the replacement with the
#         variables they correspond to in the expression"""
#         if len(expression.args) == len(pattern.args):
#             expr_terms = [sub for sub in expression.args]
#             pat_terms = [sub for sub in pattern.args]
#             new = [expression]; v = []
#             allThere = True
#             for i in range(len(expr_terms)):
#                 if check(expr_terms[i],pat_terms[i]) == None: allThere = False; break
#             if allThere == True:
#                 ch = list(check(expression,pattern).iteritems())
#                 for i in ch: v.append(i)
#                 new.insert(new.index(expression),replacement.subs(v))
#                 new.remove(expression)
#                 return Mul(*new)
# 
#     #    """ If the expression is just a function (generally just like exp(blah blah))
#     #    then check if the whole expression matches the pattern. If so, replace the
#     #    wilds in the replacement with the variables they correspond to in the
#     #    expression"""
#         if isinstance(type(expression),FunctionClass):
#             new = [expression]; v = []
#             if check(expression,pattern) != None:
#                 ch = list(check(expression,pattern).iteritems())
#                 for i in ch: v.append(i)
#                 new.insert(new.index(expression),replacement.subs(v))
#                 new.remove(expression)
#                 return Mul(*new)
#             elif expression.args:
#                 new = [subexpression for subexpression in expression.args]; v = []
#                 for sub in new:
#                     if check(sub,pattern) != None:
#                         ch = list(check(sub,pattern).iteritems())
#                         for i in ch: v.append(i)
#                         new.insert(new.index(sub),replacement.subs(v))
#                         new.remove(sub)
#                     else:
#                         new.insert(new.index(sub),IKFastSolver._walk_it(sub, pattern, replacement, check))
#                         new.remove(sub)
#                 return op(*new)
#             #return Mul(*new)
# 
#         #""" Else if the expression has multiple arguments, scan through each. """
#         elif expression.args:
#             new = [subexpression for subexpression in expression.args]; v = []
#             for sub in new:
#                 if check(sub,pattern) != None:
#                     ch = list(check(sub,pattern).iteritems())
#                     for i in ch: v.append(i)
#                     new.insert(new.index(sub),replacement.subs(v))
#                     new.remove(sub)
#                 else:
#                     new.insert(new.index(sub),IKFastSolver._walk_it(sub, pattern, replacement, check))
#                     new.remove(sub)
#             return op(*new)
#         else: return expression

    @staticmethod
    def removecommonexprs(eq):
        """removes common expressions from a sum. For example:
        a*c_0 + a*c_1 + a*c_2 = 0
        will return in
        c_0 + c_1 + c_2 = 0
        """
        if eq.is_Add and len(eq.args) > 1:
            exprs = eq.args
            for i in range(len(exprs)):
                denom = fraction(exprs[i])[1]
                if denom != S.One:
                    exprs = [expr*denom for expr in exprs]
            # there are no fractions, so can start simplifying
            common = exprs[0]/fraction(cancel(exprs[0]/exprs[1]))[0]
            for i in range(2,len(exprs)):
                common = common/fraction(cancel(common/exprs[i]))[0]
                if common.is_number:
                    common=S.One
                    break
            if common.is_Mul:
                # remove all numbers since dividing will introduce precision problems
                args = common.args
                common = S.One
                for arg in args:
                    if not arg.is_number:
                        common *= arg
            # find biggest number
            smallestnumber = None
            for expr in exprs:
                if expr.is_number:
                    if smallestnumber is None or smallestnumber < abs(expr):
                        smallestnumber = abs(expr)
                elif expr.is_Mul:
                    n = S.One
                    for arg in expr.args:
                        if arg.is_number:
                            n *= arg
                    if smallestnumber is None or smallestnumber < abs(n):
                        smallestnumber = abs(n)
            if smallestnumber is not None:
                common *= smallestnumber
            eq = S.Zero
            for expr in exprs:
                eq += expr/common
        return eq

    @staticmethod
    def customtrigsimp(expr, deep=False):
        """
        Usage
        =====
        trig(expr) -> reduces expression by using known trig identities

        Notes
        =====


        Examples
        ========
        >>> from sympy import *
        >>> x = Symbol('x')
        >>> y = Symbol('y')
        >>> e = 2*sin(x)**2 + 2*cos(x)**2
        >>> trigsimp(e)
        2
        >>> trigsimp(log(e))
        log(2*cos(x)**2 + 2*sin(x)**2)
        >>> trigsimp(log(e), deep=True)
        log(2)
        """
        from sympy.core.basic import S
        sin, cos, tan, cot, atan2 = C.sin, C.cos, C.tan, C.cot, C.atan2
        
        #XXX this stopped working:
        if expr == 1/cos(Symbol('x'))**2 - 1:
            return tan(Symbol('x'))**2

        a,b,c = map(Wild, 'abc')
        artifacts = [
            (a*cos(2.0*b), a*cos(b)**2-a*sin(b)**2, []),
            (a*sin(2.0*b), 2.0*a*cos(b)*sin(b), [])
            ]
        for pattern, result, ex in artifacts:
            # Substitute a new wild that excludes some function(s)
            # to help influence a better match. This is because
            # sometimes, for example, 'a' would match sec(x)**2
            a_t = Wild('a', exclude=ex)
            pattern = pattern.subs(a, a_t)
            result = result.subs(a, a_t)

            if expr.is_number:
                return expr

            try:
                m = expr.match(pattern)
            except TypeError:
                break

            if m is not None:
                if m[a_t] == 0 or not m[b].is_Symbol:
                    break
                # make sure none of the coefficients have divides
                if m[a_t].is_Mul and any([fraction(e)[1]!=S.One for e in m[a_t].args]):
                    break
                expr = result.subs(m)

        if expr.is_Function:
            if deep:
                newargs = [IKFastSolver.customtrigsimp(a, deep) for a in expr.args]

                if expr.func == atan2:
                    # check special simplification
                    a,b,c = map(Wild, 'abc')
                    patterns = [
                        (a*sin(b)+c*cos(b),c*sin(b)-a*cos(b),-b-atan2(c,a)-pi),
                        (a*sin(b)-c*cos(b),c*sin(b)+a*cos(b),b-atan2(c,a)),
                        (a*sin(b)+c*cos(b),-c*sin(b)+a*cos(b),b+atan2(c,a)),
                        (a*sin(b)-c*cos(b),-c*sin(b)-a*cos(b),-b+atan2(c,a)-pi),
                        (-a*sin(b)-c*cos(b),c*sin(b)-a*cos(b),b+atan2(c,a)+pi),
                        (-a*sin(b)+c*cos(b),c*sin(b)+a*cos(b),-b+atan2(c,a)),
                        (-a*sin(b)-c*cos(b),a*cos(b)-c*sin(b),-b-atan2(c,a)),
                        (-a*sin(b)+c*cos(b),-c*sin(b)-a*cos(b),b-atan2(c,a)+pi),
                        ]
                    for pattern in patterns:
                        m0 = newargs[0].match(pattern[0])
                        m1 = newargs[1].match(pattern[1])
                        if m0 is not None and m1 is not None and len(m0) == len(m1) and all([m1.has_key(key) for key in m0.keys()]):
                            if (not m0.has_key(a) or m0[a]-m1[a]==0) and (not m0.has_key(b) or m0[b]-m1[b]==0) and (not m0.has_key(c) or m0[c]-m1[c]==0):
                                return pattern[2].subs(m0)

                newexpr = expr.func(*newargs)
                return newexpr
        elif expr.is_Mul:
            ret = S.One
            for x in expr.args:
                ret *= IKFastSolver.customtrigsimp(x, deep)
            return ret.expand()
        elif expr.is_Pow:
            return Pow(IKFastSolver.customtrigsimp(expr.base, deep), IKFastSolver.customtrigsimp(expr.exp, deep))
        elif expr.is_Add:
            # TODO this needs to be faster
            # The types of trig functions we are looking for
            matchers = (
                (a*sin(b)**2, a - a*cos(b)**2),
                (a*tan(b)**2, a*(1/cos(b))**2 - a),
                (a*cot(b)**2, a*(1/sin(b))**2 - a),
                )
            
            # Scan for the terms we need
            ret = S.Zero
            for term in expr.args:
                term = IKFastSolver.customtrigsimp(term, deep)
                res = None
                for pattern, result in matchers:
                    res = term.match(pattern)
                    if res is not None:
                        ret += result.subs(res)
                        break
                if res is None:
                    ret += term
            
            # Reduce any lingering artifacts, such as sin(x)**2 changing
            # to 1-cos(x)**2 when sin(x)**2 was "simpler"
            artifacts = [
                (a - a*cos(b)**2 + c, a*sin(b)**2 + c, []),
                (a*cos(b)**2 - a*cos(b)**4 + c, a*cos(b)**2*sin(b)**2 + c, []),
                (a*sin(b)**2 - a*sin(b)**4 + c, a*cos(b)**2*sin(b)**2 + c, []),
                (a*sin(b)**2 + c*sin(b)**2, sin(b)**2*(a+c), []),
                (a*sin(b)**2 + a*cos(b)**2 + c, a + c, [])
                #(a*cos(b)*cos(2.0*b)+a*sin(b)*sin(2.0*b)+a*cos(2.0*b)*sin(b) - a*cos(b)*sin(2.0*b) + c, cos(b)-sin(b) + c,[])
                # tangets should go after all identities with cos/sin (otherwise infinite looping)!!
                #(a - a*(1/cos(b))**2 + c, -a*tan(b)**2 + c, [cos]),
                #(a - a*(1/sin(b))**2 + c, -a*cot(b)**2 + c, [sin]),
                #(a*cos(b)**2*tan(b)**2 + c, a*sin(b)**2+c, [sin,cos,tan]),
                #(a*sin(b)**2/tan(b)**2 + c, a*cos(b)**2+c, [sin,cos,tan]),
                ]

            expr = ret
            Changed = True
            while Changed:
                Changed = False
                prevexpr = expr
                
                for pattern, result, ex in artifacts:
                    # Substitute a new wild that excludes some function(s)
                    # to help influence a better match. This is because
                    # sometimes, for example, 'a' would match sec(x)**2
                    a_t = Wild('a', exclude=ex)
                    pattern = pattern.subs(a, a_t)
                    result = result.subs(a, a_t)
                    
                    if expr.is_number:
                        return expr
                    
                    try:
                        m = expr.match(pattern)
                    except TypeError:
                        break
                    
                    while m is not None:
                        if not m.has_key(c):
                            break
                        if m[a_t] == 0 or -m[a_t] in m[c].args or m[a_t] + m[c] == 0:
                            break
                        if m[a_t].is_Mul and any([fraction(e)[1]!=S.One for e in m[a_t].args]):
                            break
                        exprnew = result.subs(m)
                        if len(exprnew.args) > len(expr.args):
                            break
                        expr = exprnew
                        Changed = True
                        if not expr.is_Add:
                            break # exhausted everything
                        if len(exprnew.args) == len(expr.args):
                            break
                        
                        try:
                            m = expr.match(pattern)
                        except TypeError:
                            break
                    
                    if not expr.is_Add:
                        break # exhausted everything
                
                if not expr.is_Add:
                    break # exhausted everything
                if expr.is_Add and len(expr.args) >= len(prevexpr.args):
                    break # new expression hasn't helped, so stop
            
            return expr
        return expr

    @staticmethod
    def customtsolve(eq, sym):
        """
        Solves a transcendental equation with respect to the given
        symbol. Various equations containing mixed linear terms, powers,
        and logarithms, can be solved.

        Only a single solution is returned. This solution is generally
        not unique. In some cases, a complex solution may be returned
        even though a real solution exists.

            >>> from sympy import *
            >>> x = Symbol('x')

            >>> tsolve(3**(2*x+5)-4, x)
            (-5*log(3) + log(4))/(2*log(3))

            >>> tsolve(log(x) + 2*x, x)
            1/2*LambertW(2)

        """
        if solvers.patterns is None:
            solvers._generate_patterns()
        eq = sympify(eq)
        if isinstance(eq, Equality):
            eq = eq.lhs - eq.rhs
        sym = sympify(sym)
        
        eq2 = eq.subs(sym, solvers.x)
        # First see if the equation has a linear factor
        # In that case, the other factor can contain x in any way (as long as it
        # is finite), and we have a direct solution
        r = Wild('r')
        m = eq2.match((solvers.a*solvers.x+solvers.b)*r)
        if m and m[solvers.a]:
            return [(-solvers.b/solvers.a).subs(m).subs(solvers.x, sym)]
        for p, sol in solvers.patterns:
            m = eq2.match(p)
            if m:
                return [sol.subs(m).subs(solvers.x, sym)]
        
        # let's also try to inverse the equation
        lhs = eq
        rhs = S.Zero
        while True:
            indep, dep = lhs.as_independent(sym)

            # dep + indep == rhs
            if lhs.is_Add:
                # this indicates we have done it all
                if indep is S.Zero:
                    break

                lhs = dep
                rhs-= indep
        
            # dep * indep == rhs
            else:
                # this indicates we have done it all
                if indep is S.One:
                    break

                lhs = dep
                rhs/= indep
        
        #                    -1
        # f(x) = g  ->  x = f  (g)
        if lhs.is_Function and lhs.nargs==1 and hasattr(lhs, 'inverse'):
            rhs = lhs.inverse() (rhs)
            lhs = lhs.args[0]
            
            sol = solve(lhs-rhs, sym)
            return sol
        elif lhs.is_Pow:
            rhs = pow(rhs,1/lhs.exp)
            lhs = lhs.base

            sol = IKFastSolver.customtsolve(lhs-rhs, sym)
            return sol
        elif lhs.is_Add:
            # just a simple case - we do variable substitution for first function,
            # and if it removes all functions - let's call solve.
            #      x    -x                   -1
            # UC: e  + e   = y      ->  t + t   = y
            t = Symbol('t', dummy=True)
            terms = lhs.args
            
            # find first term which is Function
            IsPow = False
            for f1 in lhs.args:
                if f1.is_Function:
                    break
                elif f1.is_Pow and f1.exp.evalf() < 1:
                    IsPow = True
                    break
                elif f1.is_Mul:
                    if len(filter(lambda f2: f2.is_Pow and f2.exp.evalf() < 1, f1.args)) == 1:
                        IsPow = True
                        break
                    elif len(filter(lambda f2: f2.is_Function, f1.args)) == 1:
                        break
            else:
                return solve(lhs-rhs,sym)

            # perform the substitution
            lhs_ = lhs.subs(f1, t)
            
            # if no Functions left, we can proceed with usual solve
            if not (lhs_.is_Function or
                    any(term.is_Function for term in lhs_.args)):
                cv_sols = solve(lhs_ - rhs, t)
                if IsPow:
                    cv_inv = IKFastSolver.customtsolve( t - f1, sym )[0]
                else:
                    cv_inv = solve( t - f1, sym )[0]
                
                sols = list()
                if cv_inv.is_Pow:
                    for sol in cv_sols:
                        neweq = (pow(sym,1/cv_inv.exp)-cv_inv.base.subs(t, sol)).expand()
                        symbolgen = cse_main.numbered_symbols('tsolveconst')
                        neweq2,symbols = IKFastSolver.factorLinearTerms(neweq,[sym],symbolgen)
                        neweq3,symbols2 = IKFastSolver.removeConstants(neweq2,[sym],symbolgen)
                        symbols += [(s[0],s[1].subs(symbols)) for s in symbols2]
                        newsols = solve(neweq3,sym)
                        sols = sols + [simplify(sol.subs(symbols)) for sol in newsols]
                else:
                    for sol in cv_sols:
                        newsol = cv_inv.subs(t, sol)
                        if newsol.has_any_symbols(sym):
                            sols = sols + solve(newsol,sym)
                        else:
                            sols.append(newsol)
                return sols        
        raise IKFastSolver.CannotSolveError('unable to solve the equation')

    @staticmethod
    def tolatex(e):
        s = printing.latex(e)
        s1 = re.sub('\\\\operatorname\{(sin|cos)\}\\\\left\(j_\{(\d)\}\\\\right\)','\g<1>_\g<2>',s)
        s2 = re.sub('1\.(0*)([^0-9])','1\g<2>',s1)
        s3 = re.sub('1 \\\\(sin|cos)','\g<1>',s2)
        s4 = re.sub('(\d*)\.([0-9]*[1-9])(0*)([^0-9])','\g<1>.\g<2>\g<4>',s3)
        s5 = re.sub('sj_','s_',s4)
        s5 = re.sub('cj_','c_',s5)
        s5 = re.sub('sin','s',s5)
        s5 = re.sub('cos','c',s5)
        replacements = [('px','p_x'),('py','p_y'),('pz','p_z'),('r00','r_{00}'),('r01','r_{01}'),('r02','r_{02}'),('r10','r_{10}'),('r11','r_{11}'),('r12','r_{12}'),('r20','r_{20}'),('r21','r_{21}'),('r022','r_{22}')]
        for old,new in replacements:
            s5 = re.sub(old,new,s5)
        return s5

if __name__ == '__main__':
    parser = OptionParser(usage='usage: %prog [options] [solve joint indices]',
                          description="""
Software License Agreement (Lesser GPL v3)
Copyright (C) 2009 Rosen Diankov
ikfast is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

ikfast generates an analytical inverse kinematics solver in C++ for a set of 2, 3, or 6 joint values.
It takes as input an OpenRAVE robot file, the base link to specify all end effector transformations
again, the end effector link, and 6 joints in the chain from the base to the end effector to solve
the analytical inverse kinematics for. Extra joints between the base-end effector chain that are
not part of the 6 chosen joints need to be given inside the inverse kinematics call. For now, only
supports hinge joints.

Example usage for 7 DOF Barrett WAM where 1st joint is a free parameter:

ikfast.py --fkfile=fk_WAM7.txt --baselink=0 --eelink=7 --savefile=ik.cpp 1 2 3 4 5 6

""")
    parser.add_option('--fkfile', action='store', type='string', dest='fkfile',
                      help='forward kinematics file usually a text based representation outputed by OpenRAVE on load of the robot')
    parser.add_option('--savefile', action='store', type='string', dest='savefile',default='ik.cpp',
                      help='filename where to store the generated c++ code')
    parser.add_option('--baselink', action='store', type='int', dest='baselink',
                      help='base link index to start extraction of ik chain')
    parser.add_option('--eelink', action='store', type='int', dest='eelink',
                      help='end effector link index to end extraction of ik chain')
    parser.add_option('--freejointind','--freeparam', action='append', type='int', dest='freejointinds',default=[],
                      help='Optional joint index specifying a free parameter of the manipulator. If not specified, assumes all joints not solving for are free parameters. Can be specified multiple times for multiple free parameters.')
    parser.add_option('--rotation3donly', action='store_true', dest='rotation3donly',default=False,
                      help='If true, need to specify only 3 solve joints and will solve for a target rotation')
    parser.add_option('--rotation2donly', action='store_true', dest='rotation2donly',default=False,
                      help='If true, need to specify only 2 solve joints and will solve for a target direction')
    parser.add_option('--translation3donly', action='store_true', dest='translation3donly',default=False,
                      help='If true, need to specify only 3 solve joints and will solve for a target translation')
    parser.add_option('--usedummyjoints', action='store_true',dest='usedummyjoints',default=True,
                      help='Treat the unspecified joints in the kinematic chain as dummy and set them to 0. If not specified, treats all unspecified joints as free parameters (default=%default).')
    parser.add_option('--lang', action='store',type='string',dest='lang',default='cpp',
                      help='The language to generate the code in (default=%default), available=('+','.join(name for name,value in CodeGenerators.iteritems())+')')

    (options, args) = parser.parse_args()
    if options.fkfile is None or options.baselink is None or options.eelink is None:
        print('Error: Not all arguments specified')
        sys.exit(1)

    solvejoints = [int(joint) for joint in args]
    solvefn=IKFastSolver.solveFullIK_6D
    numexpected = 6
    if options.rotation3donly:
        numexpected = 3
        solvefn = IKFastSolver.solveFullIK_Rotation3D
    elif options.rotation2donly:
        numexpected = 2
        solvefn = IKFastSolver.solveFullIK_Direction3D
    elif options.translation3donly:
        numexpected = 3
        solvefn = IKFastSolver.solveFullIK_Translation3D

    if not len(solvejoints) == numexpected:
        print 'Need ',numexpected, 'solve joints, got: ', solvejoints
        sys.exit(1)

    tstart = time.time()
    kinematics = IKFastSolver(options.fkfile)
    code = kinematics.generateIkSolver(options.baselink,options.eelink,solvejoints,options.freejointinds,options.usedummyjoints,solvefn=solvefn,lang=options.lang)

    success = True if len(code) > 0 else False

    print 'total time for ik generation of %s is %fs'%(options.savefile,time.time()-tstart)
    if success:
        open(options.savefile,'w').write(code)

    sys.exit(0 if success else 1)
