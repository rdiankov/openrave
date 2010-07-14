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

import sys, copy, time, datetime
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

try:
    from itertools import izip, combinations
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
    replacements,reduced_exprs = cse(exprs,symbols=symbols)
    newreplacements = []
    # look for opany expressions of the order of (x**(1/a))**b, usually computer wants x^(b/a)
    for r in replacements:
        if r[1].is_Pow and r[1].exp.is_real and r[1].base.is_Symbol:
            baseexpr = r[1].base.subs(replacements)
            if baseexpr.is_Pow and baseexpr.exp.is_real:
                newreplacements.append((r[0],baseexpr.base**(r[1].exp*baseexpr.exp)))
                continue
        newreplacements.append(r)
    return newreplacements,reduced_exprs

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
    def subs(self,solsubs):
        if self.jointeval is not None:
            self.jointeval = [e.subs(solsubs) for e in self.jointeval]
        if self.jointevalcos is not None:
            self.jointevalcos = [e.subs(solsubs) for e in self.jointevalcos]
        if self.jointevalsin is not None:
            self.jointevalsin = [e.subs(solsubs) for e in self.jointevalsin]
        return self
    def generate(self, generator):
        return generator.generateSolution(self)
    def end(self, generator):
        return generator.endSolution(self)

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
    
class SolverBranch(AutoReloader):
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

class SolverBranchConds(AutoReloader):
    jointbranches = None
    def __init__(self, jointbranches):
        self.jointbranches = jointbranches
    def generate(self, generator):
        return generator.generateBranchConds(self)
    def end(self, generator):
        return generator.endBranchConds(self)

class SolverCheckZeros(AutoReloader):
    jointname = None
    jointcheckeqs = None # only used for evaluation
    zerobranch = None
    nonzerobranch = None
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

class SolverFreeParameter(AutoReloader):
    jointname = None
    jointtree = None
    def __init__(self, jointname, jointtree):
        self.jointname = jointname
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateFreeParameter(self)
    def end(self, generator):
        return generator.endFreeParameter(self)

class SolverIKChainTransform6D(AutoReloader):
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

class SolverIKChainRotation3D(AutoReloader):
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

class SolverIKChainTranslation3D(AutoReloader):
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

class SolverIKChainDirection3D(AutoReloader):
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

class SolverIKChainRay4D(AutoReloader):
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

class SolverRotation(AutoReloader):
    T = None
    jointtree = None
    def __init__(self, T, jointtree):
        self.T = T
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateRotation(self)
    def end(self, generator):
        return generator.endRotation(self)

class SolverDirection(AutoReloader):
    D = None
    jointtree = None
    def __init__(self, D, jointtree):
        self.D = D
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateDirection(self)
    def end(self, generator):
        return generator.endDirection(self)

class SolverStoreSolution(AutoReloader):
    alljointvars = None
    def __init__(self, alljointvars):
        self.alljointvars = alljointvars
    def generate(self, generator):
        return generator.generateStoreSolution(self)
    def end(self, generator):
        return generator.endStoreSolution(self)

class SolverSequence(AutoReloader):
    jointtrees = None
    def __init__(self, jointtrees):
        self.jointtrees = jointtrees
    def generate(self, generator):
        return generator.generateSequence(self)
    def end(self, generator):
        return generator.endSequence(self)

class SolverSetJoint(AutoReloader):
    jointname = None
    jointvalue = None
    def __init__(self, jointname,jointvalue):
        self.jointname = jointname
        self.jointvalue = jointvalue
    def generate(self, generator):
        return generator.generateSetJoint(self)
    def end(self, generator):
        return generator.endSetJoint(self)

class SolverBreak(AutoReloader):
    def generate(self,generator):
        return generator.generateBreak(self)
    def end(self,generator):
        return generator.endBreak(self)

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

    def __init__(self, robotfile=None,robotfiledata=None,kinbody=None,accuracy=None,precision=None):
        self.joints = []
        self.freevarsubs = []
        if accuracy is None:
            self.accuracy=1e-7
        else:
            self.accuracy=accuracy
        if precision is None:
            self.precision=10
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
            Ttrans = eye(4); Ttrans[0:3,3] = Links[iright-1][0:3,0:3].transpose() * Links[iright-1][0:3,3]
            Trot_with_trans = Ttrans * Links[iright]
            separated_trans = Trot_with_trans[0:3,0:3].transpose() * Trot_with_trans[0:3,3]
            if not any([separated_trans[j].has_any_symbols(jointvars[-1]) for j in range(0,3)]):
                Ttrans[0:3,3] = separated_trans
                Links[iright+1] = Ttrans * Links[iright+1]
                Links[iright-1][0:3,3] = Matrix(3,1,[Real(0,30)]*3)
                print "moved translation ",separated_trans.transpose(),"to right end"
        
        if len(jointinds) > 1:
            ileft = jointinds[0]
            separated_trans = Links[ileft][0:3,0:3] * Links[ileft+1][0:3,3]
            if not any([separated_trans[j].has_any_symbols(jointvars[0]) for j in range(0,3)]):
                Ttrans = eye(4); Ttrans[0:3,3] = separated_trans
                Links[ileft-1] = Links[ileft-1] * Ttrans
                Links[ileft+1][0:3,3] = Matrix(3,1,[Real(0,30)]*3)
                print "moved translation ",separated_trans.transpose(),"to left end"

        if len(jointinds) > 3: # last 3 axes always have to be intersecting, move the translation of the first axis to the left
            ileft = jointinds[-3]
            separated_trans = Links[ileft][0:3,0:3] * Links[ileft+1][0:3,3]
            if not any([separated_trans[j].has_any_symbols(jointvars[-3]) for j in range(0,3)]):
                Ttrans = eye(4); Ttrans[0:3,3] = separated_trans
                Links[ileft-1] = Links[ileft-1] * Ttrans
                Links[ileft+1][0:3,3] = Matrix(3,1,[Real(0,30)]*3)
                print "moved translation on intersecting axis ",separated_trans.transpose(),"to left"

        return Links, jointvars, isolvejointvars, ifreejointvars
        
    def generateIkSolver(self, baselink, eelink, solvejoints, freeparams, usedummyjoints,solvefn=None,lang=None):
        if solvefn is None:
            solvefn = IKFastSolver.solveFullIK_6D
        alljoints = self.getJointsInChain(baselink, eelink)
        
        # mark the free joints and form the chain
        chain = []
        for joint in alljoints:
            issolvejoint = any([i == joint.jointindex for i in solvejoints])
            if usedummyjoints and not issolvejoint and not any([i == joint.jointindex for i in freeparams]):
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
        
        chaintree = solvefn(self,chain, Tee)
        if chaintree is None:
            print "failed to genreate ik solution"
            return ""

        # parse the solver tree
        if lang is None:
            # prioritize c++
            generator = CodeGenerators.get('cpp',CodeGenerators.values()[0])
        return CodeGenerators[lang]().generate(chaintree)

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
            return expr.func( self.chop(expr.args[0], precision,accuracy) )
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

    def checkForDivideByZero(self,eq):
        try:
            checkforzeros = []
            subexprs,reduced_exprs = customcse(eq)
            def checkpow(expr,sexprs):
                if expr.is_Pow:
                    sexprs.append(expr.base)
                    if expr.exp.is_real and expr.exp < 0:
                        base = self.subsExpressions(expr.base,subexprs)
                        if not base.is_number:
                            checkforzeros.append(self.subsExpressions(expr.base,subexprs))

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
                subexprs,reduced_exprs = customcse(sol.jointeval)
            elif sol.jointevalsin is not None:
                sol.score = 400*len(sol.jointevalsin)
                for s in sol.jointevalsin:
                    sol.score += self.codeComplexity(s)
                subexprs,reduced_exprs = customcse(sol.jointevalsin)
                #sol.score += 500
            elif sol.jointevalcos is not None:
                sol.score = 400*len(sol.jointevalcos)
                for s in sol.jointevalcos:
                    sol.score += self.codeComplexity(s)
                subexprs,reduced_exprs = customcse(sol.jointevalcos)
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
                        exprbase = self.subsExpressions(expr.base,subexprs)
                        # check if exprbase contains any variables that have already been solved
                        containsjointvar = exprbase.has_any_symbols(*solvedvars)
                        cancheckexpr = not exprbase.has_any_symbols(*unsolvedvars)
                        score += 10000
                        if not cancheckexpr:
                            score += 100000
                        else:
                            sol.checkforzeros.append(exprbase)
                elif expr.is_finite is not None and not expr.is_finite:
                    return oo # infinity
                return score
            
            sexprs = [subexpr[1] for subexpr in subexprs]+reduced_exprs
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
                elif sexpr.is_finite is not None and not sexpr.is_finite:
                    so.score = oo # infinity
                else:
                    sol.score += checkpow(sexpr,sexprs)
        except AssertionError, e:
            print e
            sol.score=1e10
        return sol.score

    def affineSimplify(self, T):
        # yes, it is necessary to call self.trigsimp so many times since it gives up too easily
        values = map(lambda x: self.chop(trigsimp(trigsimp(self.chop(trigsimp(x))))), T)
        # rotation should have bigger accuracy threshold
        for i in [0,1,2,4,5,6,8,9,10]:
            values[i] = self.chop(values[i],accuracy=self.accuracy*10.0)
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

    def solveFullIK_Direction3D(self,chain,Tee,basedir):
        """basedir needs to be filled with a 3elemtn vector of the initial direction to control"""
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]
        
        if not len(solvejointvars) == 2:
            raise ValueError('solve joints needs to be 2')

        # rotate all links so that basedir becomes the z-axis
        Trightnormaliation = self.rotateDirection(sourcedir=Matrix(3,1,[Real(round(float(x),4),30) for x in basedir]), targetdir = Matrix(3,1,[S.Zero,S.Zero,S.One]))
        Links[-1] *= self.affineInverse(Trightnormaliation)
        LinksInv = [self.affineInverse(link) for link in Links]

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
        
        LinksAccumLeftAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftAll)
        LinksAccumLeftInvAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftInvAll)
        LinksAccumRightAll = map(lambda T: self.affineSimplify(T), LinksAccumRightAll)
        
        solverbranches = []
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        for freevar in freejointvars+[None]:
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None and self.IsHinge(freevar.name) else [None]
            for freevalue in freevalues:
                print 'attempting ',freevar,' = ',freevalue
                if freevar is not None and freevalue is not None:
                    var = self.Variable(freevar)
                    valuesubs = [(var.var,freevalue),(var.svar,sin(freevalue)),(var.cvar,cos(freevalue))] if freevalue is not None else []
                    freevarcond = freevar-freevalue
                else:
                    valuesubs = []
                    freevarcond = None

                storesolutiontree = [SolverStoreSolution (jointvars)]
                solvedvarsubs = valuesubs+self.freevarsubs
                rotsubs = [(Symbol('r%d%d'%(0,i)),Symbol('new_r%d%d'%(0,i))) for i in range(3)]
                rotvars = [var for var in jointvars if any([var==svar for svar in solvejointvars])]
                D = Matrix(3,1, map(lambda x: x.subs(self.freevarsubs), LinksAccumRightAll[0][0:3,2]))
                rottree = self.solveIKRotation(R=D,Ree = Dee.subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
                solverbranches.append((freevarcond,[SolverDirection(LinksAccumLeftInvAll[0].subs(solvedvarsubs)[0:3,0:3]*Dee, rottree)]))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for kinematics'
            return None
        return SolverIKChainDirection3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Tfirstleft.inv()[0:3,0:3] * Dee, [SolverBranchConds(solverbranches)])

    def solveFullIK_Rotation3D(self,chain,Tee):
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        Tfirstright = Links.pop()
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
        
        LinksAccumLeftAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftAll)
        LinksAccumLeftInvAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftInvAll)
        LinksAccumRightAll = map(lambda T: self.affineSimplify(T), LinksAccumRightAll)

        solverbranches = []
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        for freevar in freejointvars+[None]:
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None and self.IsHinge(freevar.name) else [None]
            for freevalue in freevalues:
                print 'attempting ',freevar,' = ',freevalue
                if freevar is not None and freevalue is not None:
                    var = self.Variable(freevar)
                    valuesubs = [(var.var,freevalue),(var.svar,sin(freevalue)),(var.cvar,cos(freevalue))] if freevalue is not None else []
                    freevarcond = freevar-freevalue
                else:
                    valuesubs = []
                    freevarcond = None

                storesolutiontree = [SolverStoreSolution (jointvars)]
                solvedvarsubs = valuesubs+self.freevarsubs
                rotsubs = [(Symbol('r%d%d'%(i,j)),Symbol('new_r%d%d'%(i,j))) for i in range(3) for j in range(3)]
                rotvars = [var for var in jointvars if any([var==svar for svar in solvejointvars])]
                R = Matrix(3,3, map(lambda x: x.subs(solvedvarsubs), LinksAccumRightAll[0][0:3,0:3]))
                rottree = self.solveIKRotation(R=R,Ree = Ree.subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
                solverbranches.append((freevarcond,[SolverRotation(LinksAccumLeftInvAll[0].subs(solvedvarsubs)*Tee, rottree)]))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for kinematics'
            return None
        return SolverIKChainRotation3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Tfirstleft.inv()[0:3,0:3] * Ree * Tfirstright.inv()[0:3,0:3], [SolverBranchConds(solverbranches)],Rfk = Tfirstleft[0:3,0:3] * LinksAccumRightAll[0][0:3,0:3])

    def solveFullIK_Translation3D(self,chain,Tee):
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
        
        LinksAccumLeftAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftAll)
        LinksAccumLeftInvAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftInvAll)
        LinksAccumRightAll = map(lambda T: self.affineSimplify(T), LinksAccumRightAll)
        
        solverbranches = []
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        for freevar in freejointvars+[None]:
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None and self.IsHinge(freevar.name) else [None]
            for freevalue in freevalues:
                print 'attempting ',freevar,' = ',freevalue
                if freevar is not None and freevalue is not None:
                    var = self.Variable(freevar)
                    valuesubs = [(var.var,freevalue),(var.svar,sin(freevalue)),(var.cvar,cos(freevalue))] if freevalue is not None else []
                    freevarcond = freevar-freevalue
                else:
                    valuesubs = []
                    freevarcond = None
                
                Positions = [Matrix(3,1,map(lambda x: self.customtrigsimp(x), LinksAccumRightAll[i][0:3,3].subs(valuesubs))) for i in range(len(LinksAccumRightAll))]
                Positionsee = [Matrix(3,1,map(lambda x: self.customtrigsimp(x), (LinksAccumLeftInvAll[i][0:3,0:3]*Pee+LinksAccumLeftInvAll[i][0:3,3]).subs(valuesubs))) for i in range(len(LinksAccumLeftInvAll))]
                
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
                
                solsubs = self.freevarsubs[:]
                endbranchtree = [SolverStoreSolution (jointvars)]                
                curtransvars = solvejointvars[:]
                transtree = self.solveIKTranslationAll(Positions,Positionsee,curtransvars,
                                                       otherunsolvedvars = [],
                                                       othersolvedvars = freejointvars,
                                                       endbranchtree=endbranchtree,
                                                       solsubs = solsubs)
                
                if len(curtransvars) > 0:
                    print 'error, cannot solve translation for ',freevar,freevalue
                    continue
                solverbranches.append((freevarcond,transtree))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for direction3d kinematics'
            return None
        return SolverIKChainTranslation3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], TfirstleftInv[0:3,0:3] * Pee + TfirstleftInv[0:3,3] , [SolverBranchConds(solverbranches)],Pfk = Tfirstleft * LinksAccumRightAll[0][0:4,3])

    def solveFullIK_Ray4D(self,chain,Tee,rawbasedir,rawbasepos):
        """basedir,basepos needs to be filled with a direction and position of the ray to control"""
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        TfirstleftInv = Tfirstleft.inv()
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]
        
        basedir = Matrix(3,1,rawbasedir.tolist())
        basepos = Matrix(3,1,rawbasepos.tolist())
        basedir /= sqrt(basedir[0]*basedir[0]+basedir[1]*basedir[1]+basedir[2]*basedir[2]).evalf()
        basepos = orgbasepos-basedir*basedir.dot(orgbasepos)
        
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
        
        LinksAccumLeftAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftAll)
        LinksAccumLeftInvAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftInvAll)
        LinksAccumRightAll = map(lambda T: self.affineSimplify(T), LinksAccumRightAll)

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
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        for freevar in freejointvars+[None]:
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None and self.IsHinge(freevar.name) else [None]
            for freevalue in freevalues:
                print 'attempting ',freevar,' = ',freevalue
                if freevar is not None and freevalue is not None:
                    var = self.Variable(freevar)
                    valuesubs = [(var.var,freevalue),(var.svar,sin(freevalue)),(var.cvar,cos(freevalue))] if freevalue is not None else []
                    freevarcond = freevar-freevalue
                else:
                    valuesubs = []
                    freevarcond = None

                Ds = [Matrix(3,1, map(lambda x: self.chop(x,accuracy=self.accuracy*10.0), LinksAccumRightAll[i][0:3,0:3]*basedir)) for i in range(5)]
                Dsee = [Matrix(3,1, map(lambda x: self.chop(x,accuracy=self.accuracy*10.0), LinksAccumLeftInvAll[i][0:3,0:3]*Dee)) for i in range(5)]

                Positions = [Matrix(3,1,map(lambda x: self.customtrigsimp(x), (LinksAccumRightAll[i][0:3,0:3]*basepos+LinksAccumRightAll[i][0:3,3]).subs(valuesubs))) for i in range(5)]
                Positionsee = [Matrix(3,1,map(lambda x: self.customtrigsimp(x), (LinksAccumLeftInvAll[i][0:3,0:3]*Pee+LinksAccumLeftInvAll[i][0:3,3]).subs(valuesubs))) for i in range(5)]
                for i in range(len(Positions)):
                    #Positions[i] -= Dsee[i]*(Dsee[i][0]*Positions[i][0]+Dsee[i][1]*Positions[i][1]+Dsee[i][2]*Positions[i][2])
                    Positions[i] = Positions[i].cross(Dsee[i])
                    Positions[i] = Matrix(3,1,map(lambda x: self.customtrigsimp(self.customtrigsimp(x)),Positions[i]))
                    #Positionsee[i] -= Dsee[i]*(Dsee[i][0]*Positionsee[i][0]+Dsee[i][1]*Positionsee[i][1]+Dsee[i][2]*Positionsee[i][2])
                    Positionsee[i] = Positionsee[i].cross(Dsee[i])
                    Positionsee[i] = Matrix(3,1,map(lambda x: self.customtrigsimp(self.customtrigsimp(x)),Positionsee[i]))

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

                uselength=False
                orgsolsubs = self.freevarsubs[:]
                rottree = []
                endbranchtree = [SolverSequence([rottree])]
                try:
                    # attempt the full solution
                    print 'attempting full solutions'
                    AllEquations = []
                    for i in range(len(Positions)):
                        for j in range(3):
                            e = Positions[i][j] - Positionsee[i][j]
                            if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                                AllEquations.append(e)
                            e = Ds[i][j] - Dsee[i][j]
                            if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                                AllEquations.append(e)
                        if uselength:
                            e = self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((Positions[i][0]**2+Positions[i][1]**2+Positions[i][2]**2).expand())).expand())) - self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((Positionsee[i][0]**2+Positionsee[i][1]**2+Positionsee[i][2]**2).expand())).expand()))
                            if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                                AllEquations.append(e)
                    AllEquations.sort(lambda x, y: self.codeComplexity(x)-self.codeComplexity(y))
                    endbranchtree = [SolverStoreSolution (jointvars)]
                    fulltree = self.solveAllEquations(AllEquations,curvars=solvejointvars,othersolvedvars = freejointvars[:],solsubs = orgsolsubs,endbranchtree=endbranchtree)
                    solverbranches.append((freevarcond,fulltree))
                    continue
                except self.CannotSolveError:
                    transtree = self.solveIKTranslationAll(Positions,Positionsee,curtransvars = solvejointvars[0:2],
                                                           otherunsolvedvars = solvejointvars[2:4],
                                                           othersolvedvars = freejointvars[:],
                                                           endbranchtree=endbranchtree,
                                                           solsubs = orgsolsubs,uselength=uselength)

                storesolutiontree = [SolverStoreSolution (jointvars)]
                solvedvarsubs = valuesubs+self.freevarsubs
                for tvar in solvejointvars[0:2]:
                    solvedvarsubs += [(cos(tvar),self.Variable(tvar).cvar),(sin(tvar),self.Variable(tvar).svar)]
                rotsubs = [(Symbol('r%d%d'%(0,i)),Symbol('new_r%d%d'%(0,i))) for i in range(3)]
                rotvars = [var for var in jointvars if any([var==svar for svar in solvejointvars[2:4]])]
                Dsub = Matrix(3,1, map(lambda x: x.subs(solvedvarsubs), LinksAccumRight[2][0:3,0:3]*basedir))
                dirtree = self.solveIKRotation(R=Dsub,Ree = Dee.subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
                Tlinksub = LinksAccumLeftInv[2].subs(solvedvarsubs)
                rottree += [SolverDirection(Tlinksub[0:3,0:3]*Dee, dirtree)]
                solverbranches.append((freevarcond,transtree))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for ray4d kinematics'
            return None
        return SolverIKChainRay4D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], TfirstleftInv[0:3,0:3] * Pee + TfirstleftInv[0:3,3], TfirstleftInv[0:3,0:3] * Dee, [SolverBranchConds(solverbranches)],Dfk=Tfirstleft[0:3,0:3]*LinksAccumRightAll[0][0:3,0:3]*basedir,Pfk=Tfirstleft*(LinksAccumRightAll[0]*Matrix(4,1,[orgbasepos[0],orgbasepos[1],orgbasepos[2],1.0])))
        
    def solveFullIK_6D(self, chain, Tee):
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        Tfirstright = Links.pop()
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
        
        LinksAccumLeftAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftAll)
        LinksAccumLeftInvAll = map(lambda T: self.affineSimplify(T), LinksAccumLeftInvAll)
        LinksAccumRightAll = map(lambda T: self.affineSimplify(T), LinksAccumRightAll)
        
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
            print 'failed to find joint index to separate translation and rotation'
            return None
        
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
            print 'rotvars: ',rotvars,' transvars: ',transvars,' not 3 dims'
            return None

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
            print 'could not divide translation/rotation'
            return None

        solverbranches = []
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        for freevar in freejointvars+[None]:
            if freevar is not None:
                if not self.IsHinge(freevar.name):
                    continue
                freevalues = [0,pi/2,pi,-pi/2]
            else:
                freevalues = [None]
            for freevalue in freevalues:
                print 'attempting ',freevar,' = ',freevalue
                if freevar is not None and freevalue is not None:
                    var = self.Variable(freevar)
                    valuesubs = [(var.var,freevalue),(var.svar,sin(freevalue)),(var.cvar,cos(freevalue))] if freevalue is not None else []
                    freevarcond = freevar-freevalue
                else:
                    valuesubs = []
                    freevarcond = None
                
                Positions = [Matrix(3,1,map(lambda x: self.customtrigsimp(x), LinksAccumRightAll[i][0:3,3].subs(valuesubs))) for i in range(0,1+lastsepindex*2)]
                Positionsee = [Matrix(3,1,map(lambda x: self.customtrigsimp(x), (LinksAccumLeftInvAll[i]*Tee)[0:3,3].subs(valuesubs))) for i in range(0,1+lastsepindex*2)]
                
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
                
                rottree = []
                solsubs = self.freevarsubs[:]
                if solveRotationFirst:
                    for rvar in rotvars:
                        solsubs += [(cos(rvar),self.Variable(rvar).cvar),(sin(rvar),self.Variable(rvar).svar)]
                    endbranchtree = [SolverStoreSolution (jointvars)]
                else:
                    endbranchtree = [SolverSequence([rottree])]
                
                curtransvars = transvars[:]
                uselength=True
#                 try:
#                     AllEquations = []
#                     for i in range(len(Positions)):
#                         for j in range(3):
#                             e = Positions[i][j] - Positionsee[i][j]
#                             if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
#                                 AllEquations.append(e)
#                         if uselength:
#                             e = self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((Positions[i][0]**2+Positions[i][1]**2+Positions[i][2]**2).expand())).expand())) - self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((Positionsee[i][0]**2+Positionsee[i][1]**2+Positionsee[i][2]**2).expand())).expand()))
#                             if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
#                                 AllEquations.append(e)
#                     AllEquations.sort(lambda x, y: self.codeComplexity(x)-self.codeComplexity(y))
#                     if not solveRotationFirst:
#                         AllEquations = [eq for eq in AllEquations if not eq.has_any_symbols(*rotvars)]
#                     transtree = self.solveAllEquations(AllEquations,curvars=curtransvars,othersolvedvars = rotvars+freejointvars if solveRotationFirst else freejointvars,solsubs = solsubs,endbranchtree=endbranchtree)
#                 except self.CannotSolveError:
                #print 'failed full solution, resolve to old method'
                transtree = self.solveIKTranslationAll(Positions,Positionsee,curtransvars=curtransvars,
                                                       otherunsolvedvars = [] if solveRotationFirst else rotvars,
                                                       othersolvedvars = rotvars+freejointvars if solveRotationFirst else freejointvars,
                                                       endbranchtree=endbranchtree,
                                                       solsubs = solsubs, uselength=uselength)
                    
                if len(curtransvars) > 0:
                    print 'error, cannot solve translation for ',freevar,freevalue
                    continue
                
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
                R = Matrix(3,3, map(lambda x: x.subs(solvedvarsubs), LinksAccumRight[rotindex][0:3,0:3]))
                rottree += self.solveIKRotation(R=R,Ree = Tee[0:3,0:3].subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
                
                if len(rottree) == 0:
                    print 'could not solve for all rotation variables',freevar,freevalue
                    continue
                if solveRotationFirst:
                    solvertree.append(SolverRotation(LinksAccumLeftInv[rotindex].subs(solvedvarsubs)*Tee, rottree))
                else:
                    rottree[:] = [SolverRotation(LinksAccumLeftInv[rotindex].subs(solvedvarsubs)*Tee, rottree[:])]
                solverbranches.append((freevarcond,solvertree))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for kinematics'
            return None
        return SolverIKChainTransform6D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Tfirstleft.inv() * Tee * Tfirstright.inv(), [SolverBranchConds(solverbranches)],Tfk=Tfirstleft * LinksAccumRightAll[0] * Tfirstright)

    def isExpressionUnique(self, exprs, expr):
        for exprtest in exprs:
            e = expr-exprtest
            if e.is_number and abs(e) < 1e-10:
                return False
        return True

    def solveIKTranslationAll(self,Positions,Positionsee,curtransvars,otherunsolvedvars,othersolvedvars,endbranchtree=None,solsubs=[],uselength=True):
        allsolvedvars = othersolvedvars[:]
        transtree = []
        solsubs = solsubs[:]
        while len(curtransvars)>0:
            unsolvedvariables = curtransvars + otherunsolvedvars
            solvedvars = self.solveIKTranslation(Positions,Positionsee,rawvars = curtransvars,otherunsolvedvars=otherunsolvedvars)
            # merge solvedvars and solvedvars2 (have to compute both since sometimes solvedvars can give really bogus solutions (see pr2-beta-static.robot)
            if uselength:
                solvedvars += self.solveIKTranslationLength(Positions, Positionsee, rawvars = curtransvars,otherunsolvedvars=otherunsolvedvars)
            if len(solvedvars) == 0:
                solvedvars = self.solveIKTranslationPair(Positions,Positionsee,rawvars=curtransvars,otherunsolvedvars=otherunsolvedvars,uselength=uselength)
                if solvedvars is None:
                    raise self.CannotSolveError('Could not solve variable from length of translation')

            # for all solutions, check if there is a divide by zero
            checkforzeros = []
            scores = []
            for solvedvar in solvedvars:
                self.solutionComplexity(solvedvar[1],allsolvedvars,unsolvedvariables)
                checkforzeros.append(solvedvar[1].checkforzeros)
                scores.append(solvedvar[1].score)

            # pick only one solution, and let that be the smallest one
            bestindex = min([(score,i) for i,score in enumerate(scores)])[1]
            var = solvedvars[bestindex][0]
            
            zerobranches = []
            if len(checkforzeros[bestindex]) > 0:
                for checkzero in checkforzeros[bestindex]:
                    if len(zerobranches)>0:
                        break
                    for svar in allsolvedvars:
                        if len(zerobranches)>0:
                            break
                        covered = False
                        if checkzero.has_any_symbols(svar):
                            try:
                                solutions = self.customtsolve(Eq(checkzero,0),svar)
                                covered = True
                            except:
                                solutions = []
                            for s in solutions:
                                if s.is_real:
                                    # can actually simplify Positions and possibly get a new solution!
                                    NewPositions = copy.copy(Positions)
                                    for i in range(len(NewPositions)):
                                        NewPositions[i] = NewPositions[i].subs(svar,s)
                                    NewPositionsee = copy.copy(Positionsee)
                                    for i in range(len(NewPositionsee)):
                                        NewPositionsee[i] = NewPositionsee[i].subs(svar,s)
                                    try:
                                        newbranch = self.solveIKTranslationAll(NewPositions,NewPositionsee,curtransvars[:],otherunsolvedvars[:],allsolvedvars,endbranchtree,solsubs,uselength=uselength)
                                    except self.CannotSolveError:
                                        newbranch = []
                                    if len(newbranch) > 0:
                                        zerobranches.append((svar-s,newbranch))
                                    else:
                                        covered = False
                                else:
                                    covered = False
                        if not covered and len(curtransvars) > 1:
                            # test several values for all variables
                            valuebranches = []
                            for targetvar in curtransvars:
                            # force the value of the variable. The variable should in fact be *free*, but things get complicated with checking conditions, therefore, pick a couple of values and manually set them
                                for value in [0]:
                                    newtransvars = curtransvars[:]
                                    newtransvars.remove(targetvar)
                                    NewPositions2 = copy.copy(Positions)
                                    for i in range(len(NewPositions2)):
                                        NewPositions2[i] = NewPositions2[i].subs(targetvar,value)
                                    NewPositionsee2 = copy.copy(Positionsee)
                                    for i in range(len(NewPositionsee2)):
                                        NewPositionsee2[i] = NewPositionsee2[i].subs(targetvar,value)
                                    # have to check if positions is still consistent
                                    checkconsistenteqs = []
                                    for i in range(len(NewPositions2)):
                                        for j in range(3):
                                            expr = NewPositions2[i][j]-NewPositionsee2[i][j]
                                            if not expr.has_any_symbols(*unsolvedvariables) and (self.isExpressionUnique(checkconsistenteqs,expr) or self.isExpressionUnique(checkconsistenteqs,-expr)):
                                                checkconsistenteqs.append(expr.subs(solsubs))
                                    try:
                                        newbranch = self.solveIKTranslationAll(NewPositions2,NewPositionsee2,newtransvars,otherunsolvedvars[:],allsolvedvars+[targetvar],endbranchtree,solsubs,uselength=uselength)
                                    except self.CannotSolveError,e:
                                        newbranch = []
                                    if len(newbranch) > 0:
                                        valuebranches.append(SolverCheckZeros(targetvar.name,checkconsistenteqs,[SolverSetJoint(targetvar,value)]+newbranch,[SolverBreak()],thresh=0.0001,anycondition=False))
                            if len(valuebranches) > 0:
                                zerobranches.append((checkzero,valuebranches))

            allsolvedvars.append(var)
            curtransvars.remove(var)
            solsubs += [(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)]

            if len(curtransvars) > 0 and len(checkforzeros[bestindex]) > 0:
                childbranch = self.solveIKTranslationAll(Positions,Positionsee,curtransvars,otherunsolvedvars,allsolvedvars,endbranchtree,solsubs,uselength=uselength)
                if len(zerobranches) > 0:
                    zerobranch = [SolverBranchConds(zerobranches+[(None,[solvedvars[bestindex][1].subs(solsubs)]+childbranch)])]
                else:
                    zerobranch = [SolverBreak()] # in this case, cannot handle zeros, so don't output solution

                #TODO: cannot handle free parameters yet since it requires subroutines for evaluating the nonlinearities.
                transtree.append(SolverCheckZeros(var.name,checkforzeros[bestindex],zerobranch,[solvedvars[bestindex][1].subs(solsubs)]+childbranch))
                return transtree
            else:
                if len(zerobranches) > 0:
                    childbranch = self.solveIKTranslationAll(Positions,Positionsee,curtransvars,otherunsolvedvars,allsolvedvars,endbranchtree,solsubs,uselength=uselength)
                    transtree.append(SolverBranchConds(zerobranches+[(None,[solvedvars[bestindex][1].subs(solsubs)]+childbranch)]))
                    return transtree
                else:
                    transtree.append(solvedvars[bestindex][1].subs(solsubs))

        if len(curtransvars) == 0 and endbranchtree is not None:
            transtree += endbranchtree
        return transtree

    # solve for just the translation component
    def solveIKTranslationLength(self, Positions, Positionsee, rawvars,otherunsolvedvars=None):
        vars = map(lambda rawvar: self.Variable(rawvar), rawvars)
        
        # try to get an equation from the lengths
        Lengths = map(lambda x: self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((x[0]**2+x[1]**2+x[2]**2).expand())).expand())), Positions)
        Lengthsee = map(lambda x: self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((x[0]**2+x[1]**2+x[2]**2).expand())).expand())), Positionsee)
        LengthEq = map(lambda i: Lengths[i]-Lengthsee[i], range(len(Lengths)))
        solvedvars = []
        
        for eq in LengthEq:
            if otherunsolvedvars and len(otherunsolvedvars) > 0 and eq.has_any_symbols(*otherunsolvedvars):
                continue
            for var in vars:
                othervars = [v.var for v in vars if not v == var]
                if eq.has_any_symbols(var.var) and (len(othervars) == 0 or not eq.has_any_symbols(*othervars)):
                    symbolgen = cse_main.numbered_symbols('const')
                    eqnew, symbols = self.removeConstants(eq.subs(self.freevarsubs+[(sin(var.var),var.svar),(cos(var.var),var.cvar)]),[var.cvar,var.svar,var.var], symbolgen)
                    eqnew2,symbols2 = self.factorLinearTerms(eqnew,[var.svar,var.cvar,var.var],symbolgen)
                    symbols += [(s[0],s[1].subs(symbols)) for s in symbols2]
                    
                    numcvar = self.countVariables(eqnew2,var.cvar)
                    numsvar = self.countVariables(eqnew2,var.svar)
                    if numcvar == 1 and numsvar == 1:
                        a = Wild('a',exclude=[var.svar,var.cvar])
                        b = Wild('b',exclude=[var.svar,var.cvar])
                        c = Wild('c',exclude=[var.svar,var.cvar])
                        m = eqnew2.match(a*var.cvar+b*var.svar+c)
                        if m is not None:
                            symbols += [(var.svar,sin(var.var)),(var.cvar,cos(var.var))]
                            asinsol = asin(-m[c]/sqrt(m[a]*m[a]+m[b]*m[b])).subs(symbols)
                            constsol = -atan2(m[a],m[b]).subs(symbols).evalf()
                            jointsolutions = [constsol+asinsol,constsol+pi.evalf()-asinsol]
                            solvedvars.append((var.var,SolverSolution(var.var.name,jointeval=jointsolutions,IsHinge=self.IsHinge(var.var.name)), [self.codeComplexity(s) for s in jointsolutions]))
                            continue
                    
                    if numsvar > 0:
                        try:
                            # substitute cos
                            totalcvar = self.countVariables(eq.subs(cos(var.var),var.cvar),var.cvar)
                            if totalcvar <= 1 and totalcvar+numsvar <= 2: # anything more than 1 implies quartic equation
                                eqnew = eq.subs(self.freevarsubs+[(cos(var.var),sqrt(Real(1,30)-var.svar**2)),(sin(var.var),var.svar)])
                                eqnew,symbols = self.factorLinearTerms(eqnew,[var.svar])
                                solutions = self.customtsolve(eqnew,var.svar)
                                #print eqnew
                                jointsolutions = [s.subs(symbols+[(var.svar,sin(var.var))]) for s in solutions]
                                solvedvars.append((var.var,SolverSolution(var.var.name, jointevalsin=jointsolutions,IsHinge=self.IsHinge(var.var.name)), [self.codeComplexity(s) for s in jointsolutions]))
                        except (CannotSolveError, AttributeError):
                            pass
                    
                    if numcvar > 0:
                        # substite sin
                        try:
                            totalsvar=self.countVariables(eq.subs(sin(var.var),var.svar),var.svar)
                            if totalsvar <= 1 and totalsvar+numcvar <= 2: # anything more than 1 implies quartic equation
                                eqnew = eq.subs(self.freevarsubs+[(sin(var.var),sqrt(Real(1,30)-var.cvar**2)),(cos(var.var),var.cvar)])
                                eqnew,symbols = self.factorLinearTerms(eqnew,[var.cvar])
                                solutions = self.customtsolve(eqnew,var.cvar)
                                jointsolutions = [s.subs(symbols+[(var.cvar,cos(var.var))]) for s in solutions]
                                solvedvars.append((var.var,SolverSolution(var.var.name, jointevalcos=jointsolutions,IsHinge=self.IsHinge(var.var.name)), [self.codeComplexity(s) for s in jointsolutions]))
                        except (CannotSolveError, AttributeError):
                            pass
                    
                    if numsvar == 0 and numcvar == 0:
                        eqnew = eq.subs(self.freevarsubs)
                        eqnew,symbols = self.factorLinearTerms(eqnew,[var.var])
                        solutions = self.customtsolve(eqnew,var.var)
                        jointsolutions = [s.subs(symbols) for s in solutions]
                        solvedvars.append((var.var,SolverSolution(var.var.name, jointeval=jointsolutions,IsHinge=self.IsHinge(var.var.name)), [self.codeComplexity(s) for s in jointsolutions]))
        return solvedvars

    # solve for just the translation component
    def solveIKTranslation(self, Positions, Positionsee, rawvars,otherunsolvedvars=None):
        freevarinvsubs = [(f[1],f[0]) for f in self.freevarsubs]
        vars = map(lambda rawvar: self.Variable(rawvar), rawvars)
        solvedvars = []
        P = map(lambda i: Positions[i] - Positionsee[i], range(len(Positions)))
        for var in vars:
            othervars = [v.var for v in vars if not v == var and not v.var==solvedvars]
            eqns = []
            for p in P:
                for j in range(3):
                    if p[j].has_any_symbols(var.var) and (len(othervars)==0 or not p[j].has_any_symbols(*othervars)):
                        if otherunsolvedvars and len(otherunsolvedvars) > 0 and p[j].has_any_symbols(*otherunsolvedvars):
                            continue
                        if self.isExpressionUnique(eqns,p[j]) and self.isExpressionUnique(eqns,-p[j]):
                            eqns.append(p[j].subs(self.freevarsubs))
            if len(eqns) == 0:
                continue
            try:
                rawsolutions= self.solveSingleVariable(eqns,var.var)
                for sol in rawsolutions:
                    sol.subs(freevarinvsubs)
                    solvedvars.append((var.var,sol))
            except self.CannotSolveError:
                pass
        return solvedvars

    def solveIKTranslationPair(self, Positions, Positionsee, rawvars,otherunsolvedvars=None,uselength=True):
        if len(rawvars) < 2:
            return None
        freevarinvsubs = [(f[1],f[0]) for f in self.freevarsubs]
        vars = map(lambda rawvar: self.Variable(rawvar), rawvars)
        AllEquations = []
        for i in range(len(Positions)):
            x = Positions[i] - Positionsee[i]
            for j in range(3):
                e = Positions[i][j] - Positionsee[i][j]
                if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                    AllEquations.append(e)
            if uselength:
                e = self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((Positions[i][0]**2+Positions[i][1]**2+Positions[i][2]**2).expand())).expand())) - self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((Positionsee[i][0]**2+Positionsee[i][1]**2+Positionsee[i][2]**2).expand())).expand()))
                if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                    AllEquations.append(e)
        AllEquations.sort(lambda x, y: self.codeComplexity(x)-self.codeComplexity(y))
        for var0,var1 in combinations(vars,2):
            othervars = [v.var for v in vars if v!=var0 and v!=var1]
            eqns = []
            for p in AllEquations:
                if (len(othervars)==0 or not p.has_any_symbols(*othervars)):
                    if otherunsolvedvars and len(otherunsolvedvars) > 0 and p.has_any_symbols(*otherunsolvedvars):
                        continue
                    eq = p.subs(self.freevarsubs)
                    if self.isExpressionUnique(eqns,eq) and self.isExpressionUnique(eqns,-eq):
                        eqns.append(eq)
            if len(eqns) == 0:
                continue
            try:
                rawsolutions = self.solvePairVariables(eqns,var0.var,var1.var)
                return [(var,sol.subs(freevarinvsubs)) for sol in rawsolutions]
            except self.CannotSolveError:
                pass
        return None

    def solveAllEquations(self,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree):
        if len(curvars) == 0:
            return endbranchtree
        solsubs = solsubs[:]
        freevarinvsubs = [(f[1],f[0]) for f in self.freevarsubs]
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
                        solution.subs(freevarinvsubs)
                        self.solutionComplexity(solution,othersolvedvars,curvars)
                        solutions.append((solution,curvar))
                except self.CannotSolveError:
                    pass

        if len(solutions) > 0:
            return self.addSolution(solutions,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree)

        for var0,var1 in combinations(curvars,2):
            othervars = [var for var in curvars if var != var0 and var != var1]
            raweqns = []
            for e in AllEquations:
                if (len(othervars) == 0 or not e.has_any_symbols(*othervars)) and e.has_any_symbols(var0,var1):
                    eq = e.subs(self.freevarsubs+solsubs)
                    if self.isExpressionUnique(raweqns,eq) and self.isExpressionUnique(raweqns,-eq):
                        raweqns.append(eq)
            if len(raweqns) > 0:
                try:
                    rawsolutions=self.solvePairVariables(raweqns,var0,var1)
                    for solution in rawsolutions:
                        solution.subs(freevarinvsubs)
                        self.solutionComplexity(solution,othersolvedvars,curvars)
                        solutions.append((solution,Symbol(solution.jointname)))
                except self.CannotSolveError:
                    pass

        # take the least complex solution and go on
        if len(solutions) > 0:
            return self.addSolution(solutions,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree)

        # try to solve one variable for the others and substitute
        for curvar in curvars:
            vv = self.Variable(curvar)
            varsubs = [(sin(curvar),vv.svar),(cos(curvar),vv.cvar)]
            othervars = [var for var in curvars if var != curvar]
            raweqns = []
            for e in AllEquations:
                p = Poly(e.subs(varsubs),vv.cvar,vv.svar)
                if p.degree == 1 and not p.coeff(1,0).has_any_symbols(*othervars) and not p.coeff(0,1).has_any_symbols(*othervars):
                    eq = e.subs(self.freevarsubs+solsubs)
                    if self.isExpressionUnique(raweqns,eq) and self.isExpressionUnique(raweqns,-eq):
                        raweqns.append(eq)
            if len(raweqns) > 1:
                try:
                    eqns = [eq.subs(varsubs) for eq in raweqns]
                    s = solve(eqns,vv.cvar,vv.svar)
                    if s is not None and s.has_key(vv.svar) and s.has_key(vv.cvar):
                        commonmult = fraction(s[vv.cvar])[1]
                        commonmult *= fraction(s[vv.svar]*commonmult)[1]
                        tempsubs = [(sin(curvar),simplify(s[vv.svar]*commonmult)),(cos(curvar),simplify(s[vv.cvar]*commonmult))]
                        NewEquations = [self.customtrigsimp(eq.subs(tempsubs)) for eq in AllEquations]
                        newvars = curvars[:]
                        newvars.remove(curvar)
                        solutiontree = self.solveAllEquations(NewEquations,newvars,othersolvedvars,solsubs,endbranchtree)
                        newsolsubs = solsubs[:]
                        for var in newvars:
                            newsolsubs += [(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)]
                        return solutiontree + self.solveAllEquations(AllEquations,[curvar],othersolvedvars+newvars,newsolsubs,endbranchtree)
                except self.CannotSolveError:
                    pass
        
        raise self.CannotSolveError('failed to find a variable to solve')

    def addSolution(self,solutions,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree):
        """Take the least complex solution of a set of solutions and resume solving
        """
        solutions = [s for s in solutions if s[0].score < oo] # remove infinite scores
        solutions.sort(lambda x, y: x[0].score-y[0].score)
        bestsolution = None
        for solution in solutions:
            var = solution[1]
            zerobranches = []
            checkforzeros = solution[0].checkforzeros
            if len(checkforzeros) == 0:
                bestsolution = solution
                break
            # there are divide by zeros, so check if they can be explicitly solved for joint variables
            covered = False
            for checkzero in checkforzeros:
                for svar in othersolvedvars:
                    if len(zerobranches)>0:
                        break
                    if checkzero.has_any_symbols(svar):
                        try:
                            ss = self.customtsolve(Eq(checkzero,0),svar)
                        except:
                            covered = False
                            break
                        for s in ss:
                            if s.is_real:
                                # can actually simplify Positions and possibly get a new solution!
                                NewEquations = copy.copy(AllEquations)
                                for i in range(len(NewEquations)):
                                    NewEquations[i] = NewEquations[i].subs(svar,s)
                                try:
                                    newbranch = self.solveAllEquations(NewEquations,curvars,othersolvedvars,solsubs,endbranchtree)
                                    zerobranches.append((svar-s,newbranch))
                                    covered = True
                                except self.CannotSolveError:
                                    covered = False
                                    break
                            else:
                                covered = False
                                break
                if not covered: # couldn't find a solution, so give up completely
                    break
            if covered:
                bestsolution = solution
                break
        if bestsolution is not None:
            # did find a good solution, so take it. Make sure to check any zero branches
            newvars=curvars[:]
            newvars.remove(var)
            originalbranch = [bestsolution[0].subs(solsubs)]+self.solveAllEquations(AllEquations,curvars=newvars,othersolvedvars=othersolvedvars+[var],solsubs=solsubs+[(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)],endbranchtree=endbranchtree)
            if len(zerobranches) > 0:
                return [SolverBranchConds(zerobranches+[(None,originalbranch)])]
            else:
                return originalbranch

        # neither solution is satisfactory, so choose the variable with the shortest solution and compute (this is a conservative approach)
        var = solutions[0][1]
        usedsolutions = []
        # remove any solutions with similar checkforzero constraints (because they are essentially the same)
        for solution,nvar in solutions:
            if var != nvar:
                continue
            solution.subs(solsubs)
            if len(usedsolutions) == 0:
                usedsolutions.append(solution)
            else:
                match = False
                for usedsolution in usedsolutions:
                    if len(solution.checkforzeros) == len(usedsolution.checkforzeros):
                        if not any([self.isExpressionUnique(usedsolution.checkforzeros,-eq) or self.isExpressionUnique(usedsolution.checkforzeros,eq) for eq in solution.checkforzeros]):
                            match = True
                            break
                if not match:
                    usedsolutions.append(solution)
        newvars=curvars[:]
        newvars.remove(var)

        # finally as a last resort, test the boundaries of the current variable [0,pi/2,pi,-pi/2]
        # by forcing the value of the variable to them.
        # The variable should in fact be *free*, but things get complicated with checking conditions, therefore, pick a couple of values and manually set them
        for value in [S.Zero,pi/2,pi,-pi/2]:
            NewEquations2 = copy.copy(AllEquations)
            for i in range(len(NewEquations2)):
                NewEquations2[i] = NewEquations2[i].subs(var,value)
            # have to check if positions is still consistent
            solution = SolverSolution(var.name,jointeval=[value],IsHinge=self.IsHinge(var.name))
            solution.FeasibleIsZeros = True
            solution.checkforzeros = []
            solution.thresh = 0.000001
            for i in range(len(NewEquations2)):
                expr = NewEquations2[i]
                if not expr.has_any_symbols(*curvars) and (self.isExpressionUnique(solution.checkforzeros,expr) or self.isExpressionUnique(solution.checkforzeros,-expr)):
                    solution.checkforzeros.append(expr.subs(solsubs))
            try:
                usedsolutions.append(solution)
            except self.CannotSolveError,e:
                pass
        return [SolverConditionedSolution(usedsolutions)]+self.solveAllEquations(AllEquations,curvars=newvars,othersolvedvars=othersolvedvars+[var],solsubs=solsubs+[(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)],endbranchtree=endbranchtree)

    def solveSingleVariable(self,raweqns,var):
        svar = Symbol('s%s'%var.name)
        cvar = Symbol('c%s'%var.name)
        varsubs = [(sin(var),svar),(cos(var),cvar)]
        eqns = [eq for eq in raweqns if eq.has_any_symbols(var)]
        if len(eqns) == 0:
            raise self.CannotSolveError('not enough equations')
        if len(eqns) > 1:
            neweqns = []
            listsymbols = []
            symbolgen = cse_main.numbered_symbols('const')
            for e in eqns:
                enew, symbols = self.removeConstants(e.subs(varsubs),[cvar,svar,var], symbolgen)
                # ignore any equations with degree 3 or more
                if Poly(enew,svar).degree >= 3 or Poly(enew,cvar).degree >= 3:
                    print 'ignoring equation: ',enew
                    continue
                enew2,symbols2 = self.factorLinearTerms(enew,[svar,cvar,var],symbolgen)
                symbols += [(s[0],s[1].subs(symbols)) for s in symbols2]
                rank = self.codeComplexity(enew2)+reduce(lambda x,y: x+self.codeComplexity(y[1]),symbols,0)
                neweqns.append((rank,enew2))
                listsymbols += symbols
            # since we're solving for two variables, we only want to use two equations, so
            # start trying all the equations starting from the least complicated ones to the most until a solution is found
            eqcombinations = []
            for eqs in combinations(neweqns,2):
                eqcombinations.append((eqs[0][0]+eqs[1][0],[Eq(e[1],0) for e in eqs]))
            eqcombinations.sort(lambda x, y: x[0]-y[0])
            solutions = []
            for comb in eqcombinations:
                # try to solve for both sin and cos terms
                s = solve(comb[1],[svar,cvar])
                try:
                    if s is not None and s.has_key(svar) and s.has_key(cvar):
                        if self.chop((s[svar]-s[cvar]).subs(listsymbols),6) == 0:
                            continue
                        if s[svar].is_fraction() and s[cvar].is_fraction():
                            # check the numerator and denominator
                            if self.chop((s[svar].args[0]-s[cvar].args[0]).subs(listsymbols),6) == 0 and self.chop((s[svar].args[1]-s[cvar].args[1]).subs(listsymbols),6) == 0:
                                continue
                        expandedsol = atan2(s[svar],s[cvar]).subs(listsymbols)
                        # sometimes the returned simplest solution makes really gross approximations
                        simpsol = self.customtrigsimp(expandedsol, deep=True)
                        if self.codeComplexity(expandedsol) < self.codeComplexity(simpsol):
                            solutions.append(SolverSolution(var.name,jointeval=[expandedsol],IsHinge=self.IsHinge(var.name)))
                            if len(self.checkForDivideByZero(expandedsol)) == 0:
                                break
                        else:
                            solutions.append(SolverSolution(var.name,jointeval=[simpsol],IsHinge=self.IsHinge(var.name)))
                            if len(self.checkForDivideByZero(simpsol)) == 0:
                                break
                        if len(solutions) >= 4: # probably more than enough already?
                            break
                except AttributeError,e:
                    print 'solve is returning bad solution:',s
            if len(solutions) > 0:
                return solutions

        # solve one equation
        solutions = []
        for eq in eqns:
            symbolgen = cse_main.numbered_symbols('const')
            eqnew, symbols = self.removeConstants(eq.subs([(sin(var),svar),(cos(var),cvar)]), [cvar,svar,var], symbolgen)
            # ignore any equations with degree 3 or more
            if Poly(eqnew,svar).degree >= 3 or Poly(eqnew,cvar).degree >= 3:
                raise self.CannotSolveError('cannot solve equation with high degree')

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
                    asinsol = trigsimp(asin(-m[c]/sqrt(m[a]*m[a]+m[b]*m[b])).subs(symbols),deep=True)
                    constsol = -atan2(m[a],m[b]).subs(symbols).evalf()
                    jointsolutions = [constsol+asinsol,constsol+pi.evalf()-asinsol]
                    solutions.append(SolverSolution(var.name,jointeval=jointsolutions,IsHinge=self.IsHinge(var.name)))
                    continue
            if numcvar > 0:
                try:
                    # substitute cos
                    if self.countVariables(eqnew2,svar) <= 1 or (self.countVariables(eqnew2,cvar) <= 2 and self.countVariables(eqnew2,svar) == 0): # anything more than 1 implies quartic equation
                        tempsolutions = self.customtsolve(eqnew2.subs(svar,sqrt(1-cvar**2)),cvar)
                        jointsolutions = [s.subs(symbols+[(cvar,cos(var))]) for s in tempsolutions]
                        solutions.append(SolverSolution(var.name,jointevalcos=jointsolutions,IsHinge=self.IsHinge(var.name)))
                        continue
                except (CannotSolveError):
                    pass
            if numsvar > 0:
                # substitute sin
                try:
                    if self.countVariables(eqnew2,svar) <= 1 or (self.countVariables(eqnew2,svar) <= 2 and self.countVariables(eqnew2,cvar) == 0): # anything more than 1 implies quartic equation
                        tempsolutions = self.customtsolve(eqnew2.subs(cvar,sqrt(1-svar**2)),svar)
                        jointsolutions = [trigsimp(s.subs(symbols+[(svar,sin(var))])) for s in tempsolutions]
                        solutions.append(SolverSolution(var.name,jointevalsin=jointsolutions,IsHinge=self.IsHinge(var.name)))
                        continue
                except (CannotSolveError):
                    pass
            if numcvar == 0 and numsvar == 0:
                tempsolutions = self.customtsolve(eqnew2,var)
                jointsolutions = [self.customtrigsimp(s.subs(symbols)) for s in tempsolutions]
                solutions.append(SolverSolution(var.name,jointeval=jointsolutions,IsHinge=self.IsHinge(var.name)))
                continue
        if len(solutions) > 0:
            return solutions
        raise self.CannotSolveError('cannot solve equation with high degree')

    def solvePairVariables(self,raweqns,var0,var1):
        cvar0 = Symbol('c%s'%var0.name)
        svar0 = Symbol('s%s'%var0.name)
        cvar1 = Symbol('c%s'%var1.name)
        svar1 = Symbol('s%s'%var1.name)
        varsubs=[(cos(var0),cvar0),(sin(var0),svar0),(cos(var1),cvar1),(sin(var1),svar1)]
        varsubsinv = [(f[1],f[0]) for f in varsubs]
        eqns = [eq.subs(varsubs) for eq in raweqns if eq.has_any_symbols(var0,var1)]
        if len(eqns) <= 1:
            raise self.CannotSolveError('not enough equations')

        unknownvars=[v[1] for v in varsubs]
        # group equations with single variables
        symbolgen = cse_main.numbered_symbols('const')
        neweqns = []
        allsymbols = []
        for eq in eqns:
            eqnew, symbols = self.removeConstants(eq, unknownvars, symbolgen)
            eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
            allsymbols += symbols + [(s[0],s[1].subs(symbols)) for s in symbols2]
            neweqns.append([self.codeComplexity(eq),Poly(eqnew2,*unknownvars)])
        neweqns.sort(lambda x, y: x[0]-y[0])

        pairwisesubs = [(svar0*cvar1,Symbol('s0c1')),(svar0*svar1,Symbol('s0s1')),(cvar0*cvar1,Symbol('c0c1')),(cvar0*svar1,Symbol('c0s1'))]
        pairwiseinvsubs = [(f[1],f[0]) for f in pairwisesubs]
        pairwisevars = [f[1] for f in pairwisesubs]
        reduceeqns = [Poly(eq.as_basic().subs(pairwisesubs),*pairwisevars) for rank,eq in neweqns]
        # try to at least subtract as much paired variables out, could be infinite loop?
        eqcombs = [c for c in combinations(reduceeqns,2)]
        while len(eqcombs) > 0:
            eq0,eq1 = eqcombs.pop()
            for i in range(4):
                monom = [0,0,0,0]
                monom[i] = 1
                if eq0.coeff(*monom) != 0 and eq1.coeff(*monom) != 0:
                    eq = simplify((eq0.as_basic()*eq1.coeff(*monom)-eq0.coeff(*monom)*eq1.as_basic()).subs(allsymbols+pairwiseinvsubs))
                    eqns.append(eq)
                    eqnew, symbols = self.removeConstants(eq, unknownvars, symbolgen)
                    eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
                    allsymbols += symbols + [(s[0],s[1].subs(symbols)) for s in symbols2]
                    neweqns.append([self.codeComplexity(eq),Poly(eqnew2,*unknownvars)])
                    reducedeq = Poly(eqnew2.subs(pairwisesubs),*pairwisevars)
                    for e in reduceeqns:
                        if e != eq0 and e != eq1:
                            eqcombs.append((reducedeq,e))
                            
        # try single variable solution
        try:
            return self.solveSingleVariable([e.subs(varsubsinv) for e in eqns if not e.has_any_symbols(cvar1,svar1,var1)],var0)
        except self.CannotSolveError:
            pass

        try:
            return self.solveSingleVariable([e.subs(varsubsinv) for e in eqns if not e.has_any_symbols(cvar0,svar0,var0)],var1)
        except self.CannotSolveError:
            pass

        # reduce the equations so that we get X+a*(sin|cos)+b=0
        singleeqs = None
        # try to solve for all pairwise variables
        for eqs in combinations(reduceeqns,4):
            solution=solve(eqs,pairwisevars)
            if solution:
                if all([not s.has_any_symbols(*pairwisevars) for s in solution.itervalues()]):
                    singleeqs = []
                    for v,s in solution.iteritems():
                        eq = simplify(s.subs(allsymbols)) - v.subs(pairwiseinvsubs)
                        eqnew, symbols = self.removeConstants(eq, unknownvars, symbolgen)
                        eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
                        allsymbols += symbols + [(s[0],s[1].subs(symbols)) for s in symbols2]
                        singleeqs.append([self.codeComplexity(eq),Poly(eqnew2,*unknownvars)])
                    break
        if singleeqs is not None:
            neweqns += singleeqs
            neweqns.sort(lambda x, y: x[0]-y[0])

        groups=[]
        for i,unknownvar in enumerate(unknownvars):
            listeqs = []
            for rank,eq in neweqns:
                # if variable ever appears, it should be alone
                if all([m[i] == 0 or __builtin__.sum(m) == m[i] for m in eq.iter_monoms()]):
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
            if len(goodgroup) == 0:
                raise self.CannotSolveError('cannot cleanly separate pair equations')
            useconic=True
        varindex=goodgroup[0][0]
        unknownvar=unknownvars[goodgroup[0][0]]
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
        complementvarindex = varindex-(varindex%2)+((varindex+1)%2)
        complementvar = unknownvars[complementvarindex]
        finaleq = expand(simplify(simplify(finaleq.subs(complementvar**2,1-unknownvar**2)).subs(allsymbols)))
        if useconic:
            # equation of the form a*cos^2 + b*cos*sin + c*cos + d*sin + e = 0
            # we also have cos*cos + sin*sin -1 = 0
            conicpoly = Poly(finaleq,unknownvar,complementvar)
            # the general method to solve an intersection of two conics C1 and C2 is to first
            # note that any solution to their intersection is also a solution of
            # x^T C x = 0, where C = t1*C1 + t2*C2
            # for t1, t2 in Reals. Without loss of generality, we set t2 = 1, and find t1 when
            # C becomes degenerate, ie det(C) = 0. This produces a cubic equation in t1,
            # which gives 4 solutions. Gathering all the equations produced by the degenerate
            # conic should give rise to two equations of lines. Intersect these lines with the simpler of the
            # two conics, the unit circle: c^2+s^2-1 = 0
            raise self.CannotSolveError('have conic section equation, but cannot solve yet!'%str(finalpoly))
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
            var = var0 if varindex < 2 else var1
            solversolution=SolverSolution(var.name, IsHinge=self.IsHinge(var.name))
            if (varindex%2)==0:
                solversolution.jointevalcos=[self.customtrigsimp(s.subs(allsymbols+varsubsinv)).subs(varsubs) for s in solutions]
            else:
                solversolution.jointevalsin=[self.customtrigsimp(s.subs(allsymbols+varsubsinv)).subs(varsubs) for s in solutions]
            return [solversolution]
        raise self.CannotSolveError('cannot solve pair equation')

    @staticmethod
    def solveConicEquationWithCircle(self,conicpoly):
        x,y = conicpoly.symbols
        # x^2+y^2-1 = 0
        C0 = Matrix(3,3,[conicpoly.coeff(2,0),0.5*conicpoly.coeff(1,1),0.5*conicpoly.coeff(1,0),0.5*conicpoly.coeff(1,1),conicpoly.coeff(0,2),0.5*conicpoly.coeff(0,1),0.5*conicpoly.coeff(1,0),0.5*conicpoly.coeff(0,1),conicpoly.coeff(0,0)])
        # call a function to solve this conic equation
        #C1 = eye(3); C1[2,2] = -1
        #t=Symbol('t')
        #tpoly=(C0+t*C1).det() # usually this freezes because equation is too big
        
    def solveIKTranslationTriple(self, Positions, Positionsee, rawvars,otherunsolvedvars=None,uselength=True):
        varsubs=[(cos(var0.var),var0.cvar),(sin(var0.var),var0.svar),(cos(var1.var),var1.cvar),(sin(var1.var),var1.svar)]

        # group equations with single variables
        symbolgen = cse_main.numbered_symbols('const')
        neweqns = []
        allsymbols = freevarinvsubs[:]
        for eq in eqns:
            eqnew, symbols = self.removeConstants(eq, unknownvars, symbolgen)
            eqnew2,symbols2 = self.factorLinearTerms(eqnew,unknownvars, symbolgen)
            allsymbols += symbols + [(s[0],s[1].subs(symbols)) for s in symbols2]
            neweqns.append([self.codeComplexity(eq),Poly(eqnew2,*unknownvars)])
        neweqns.sort(lambda x, y: x[0]-y[0])

        svar = var0.svar
        cvar = var0.cvar
        osvar = var1.svar
        ocvar = var1.cvar
        PE = [Poly(e,svar,cvar) for c,e in neweqns]
        for p0,p1 in combinations(PE,2):
            pcvar=Poly((p0.coeff(1,0)*p1.as_basic()-p0.as_basic()*p1.coeff(1,0)).subs(allsymbols),p0.symbols[1])
            psvar=Poly((p0.coeff(0,1)*p1.as_basic()-p0.as_basic()*p1.coeff(0,1)).subs(allsymbols),p0.symbols[1])
            print simplify(pcvar.coeff(0).subs(var0.svar**2,1-var0.cvar**2))
            print simplify(pcvar.coeff(1).subs(var0.svar**2,1-var0.cvar**2))
        csol = simplify(((-PE[0].coeff(1,0)-PE[0].coeff(0,0))/PE[0].coeff(0,1)).subs(allsymbols))
        ssol = simplify(((-PE[0].coeff(0,1)-PE[0].coeff(0,0))/PE[0].coeff(1,0)).subs(allsymbols))
        pp=simplify((PE[2].as_basic()*PE[1].coeff(1,1)-PE[1].as_basic()*PE[2].coeff(1,1)).subs(allsymbols))
        ppssol=Poly(pp.subs(cvar,csol)*fraction(csol)[1],svar)
        ppcsol=Poly(pp.subs(svar,ssol)*fraction(ssol)[1],cvar)
        #c=-ppcsol.coeff(0)/ppssol.coeff(1)
        #s=(-ppssol.coeff(2)*(c**2)-ppssol.coeff(1))/ppssol.coeff(0)
        final=c**2+s**2-1
        po=Poly(ppcsol.as_basic()**2+ppssol.as_basic()**2,osvar,ocvar)
        
    def solveIKRotation(self, R, Ree, rawvars,endbranchtree=None,solvedvarsubs=[],ignorezerochecks=[]):
        """Solve for the rotation component"""
        vars = map(lambda rawvar: self.Variable(rawvar), rawvars)
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
                    checkforzero = self.checkDivideByZero(self.customtrigsimp(solution.subs(subrealinv+invsolvedvarsubs),deep=True))
                    if len(checkforzero) > 0:
                        for sraw in checkforzero:
                            s = sraw.subs(solvedvarsubs)
                            if self.isExpressionUnique(ignorezerochecks+[b[0] for b in jointbranches],s):
                                Rsubs = Matrix(R.shape[0],R.shape[1],map(lambda x: x.subs(s,0), R))
                                if not all([r==0 for r in R-Rsubs]):
                                    addignore = []
                                    if s.is_Function:
                                        if s.func == cos:
                                            addignore.append(sin(*s.args))
                                        elif s.func == sin:
                                            addignore.append(cos(*s.args))
                                    jointbranches.append((s,self.solveIKRotation(Rsubs,Ree,[v.var for v in vars],endbranchtree,solvedvarsubs,ignorezerochecks+addignore)))
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
                checkforzero = self.checkDivideByZero(self.customtrigsimp(solution.subs(subrealinv+invsolvedvarsubs),deep=True))
                if len(checkforzero) > 0:
                    for sraw in checkforzero:
                        s = sraw.subs(solvedvarsubs)
                        if self.isExpressionUnique(ignorezerochecks+[b[0] for b in jointbranches],s):
                            Rsubs = Matrix(R.shape[0],R.shape[1],map(lambda x: x.subs(s,0), R))
                            if not all([r==0 for r in R-Rsubs]):
                                addignore = []
                                if s.is_Function:
                                    if s.func == cos:
                                        addignore.append(sin(*s.args))
                                    elif s.func == sin:
                                        addignore.append(cos(*s.args))
                                jointbranches.append((s,self.solveIKRotation(Rsubs,Ree,[v.var for v in vars],endbranchtree,solvedvarsubs,ignorezerochecks+addignore)))
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
                    asinsol = asin(-m[c]/sqrt(m[a]*m[a]+m[b]*m[b])).subs(symbols)
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

    # factors linear terms together
    @staticmethod
    def factorLinearTerms(expr,vars,symbolgen = None):
        if not expr.is_Add:
            return expr,[]
        
        if symbolgen is None:
            symbolgen = cse_main.numbered_symbols('const')
        
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

    def checkDivideByZero(self,expr):
        checkforzero = []                
        if expr.is_Function or expr.is_Add or expr.is_Mul:
            for arg in expr.args:
                checkforzero += self.checkDivideByZero(arg)
        elif expr.is_Pow and expr.exp.is_real and expr.exp < 0:
            checkforzero.append(expr.base)
        return checkforzero

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
                
    def customtrigsimp(self,expr, deep=False):
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
                if m[a_t].is_Mul and any([e.is_fraction() for e in m[a_t].args]):
                    break
                expr = result.subs(m)

        if expr.is_Function:
            if deep:
                newargs = [self.customtrigsimp(a, deep) for a in expr.args]

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
                        if m0 is not None and m1 is not None and m0[a]-m1[a]==0 and m0[b]-m1[b]==0 and m0[c]-m1[c]==0:
                            return pattern[2].subs(m0)

                newexpr = expr.func(*newargs)
                return newexpr
        elif expr.is_Mul:
            ret = S.One
            for x in expr.args:
                ret *= self.customtrigsimp(x, deep)
            return ret.expand()
        elif expr.is_Pow:
            return Pow(self.customtrigsimp(expr.base, deep), self.customtrigsimp(expr.exp, deep))
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
                term = self.customtrigsimp(term, deep)
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
                        if m[a_t].is_Mul and any([e.is_fraction() for e in m[a_t].args]):
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
        raise self.CannotSolveError('unable to solve the equation')

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
    parser.add_option('--freeparam', action='append', type='int', dest='freeparams',default=[],
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
    code = kinematics.generateIkSolver(options.baselink,options.eelink,solvejoints,options.freeparams,options.usedummyjoints,solvefn=solvefn,lang=lang)

    success = True if len(code) > 0 else False

    print 'total time for ik generation of %s is %fs'%(options.savefile,time.time()-tstart)
    if success:
        open(options.savefile,'w').write(code)

    sys.exit(0 if success else 1)
