#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (Lesser GPL)
#
# Copyright (C) 2009-2011 Rosen Diankov
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
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Lesser GPL, Version 3'
__version__ = '33'

import sys, copy, time, math, datetime
import __builtin__
from optparse import OptionParser
try:
    from openravepy.metaclass import AutoReloader
except:
    class AutoReloader:
        pass

import numpy # required for fast eigenvalue computation

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

class SolverSolution:
    jointname = None
    jointeval = None
    jointevalcos = None
    jointevalsin = None
    AddPiIfNegativeEq = None
    isHinge = True
    checkforzeros = None
    thresh = None
    """Meaning of FeasibleIsZeros:
    If set to false, then solution is feasible only if all of these equations evalute to non-zero.
    If set to true, solution is feasible only if all these equations evaluate to zero.
    """
    FeasibleIsZeros = False
    score = None
    def __init__(self, jointname, jointeval=None,jointevalcos=None,jointevalsin=None,AddPiIfNegativeEq=None,isHinge=True,thresh=0.00001):
        self.jointname = jointname
        self.jointeval = jointeval
        self.jointevalcos = jointevalcos
        self.jointevalsin = jointevalsin
        self.AddPiIfNegativeEq = AddPiIfNegativeEq
        self.isHinge=isHinge
        self.thresh = thresh
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
    def numsolutions(self):
        n=0
        if self.jointeval is not None:
            n += len(self.jointeval)
        if self.jointevalcos is not None:
            n += len(self.jointevalcos)
        if self.jointevalsin is not None:
            n += len(self.jointevalsin)
        return n
    def checkValidSolution(self):
        valid=True
        if self.jointeval is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointeval])
        if self.jointevalsin is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointevalsin])
        if self.jointevalcos is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointevalcos])
        return valid
    def getPresetCheckForZeros(self):
        return []

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
    thresh = 1e-7
    isHinge = True
    FeasibleIsZeros = False
    score = None
    def __init__(self, jointname, poly=None, jointeval=None,isHinge=True):
        self.poly = poly
        self.jointname=jointname
        self.jointeval = jointeval
        self.isHinge = isHinge
    def numsolutions(self):
        return self.poly.degree
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
        if self.poly is not None:
            self.poly = self.poly.subs(solsubs)
        assert self.checkValidSolution()
        return self
    def generate(self, generator):
        return generator.generatePolynomialRoots(self)
    def end(self, generator):
        return generator.endPolynomialRoots(self)
    def checkValidSolution(self):
        if self.poly is not None:
            valid = IKFastSolver.isValidSolution(self.poly.as_basic())
        if self.jointeval is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointeval])
        return valid
    def getPresetCheckForZeros(self):
        return [self.poly.coeffs[0]] # make sure highest coefficient is not 0!

class SolverCoeffFunction:
    """find all roots of the polynomial and plug it into jointeval. poly should be polys.polynomial.Poly
    """
    jointnames = None
    jointeval = None
    isHinges = True
    exportvar = None
    exportcoeffeqs = None
    rootmaxdim = None
    exportfnname = None
    jointevalcos = None # used for half angles
    jointevalsin = None # used for half angles
    checkforzeros = None
    FeasibleIsZeros = False
    score = None
    def __init__(self, jointnames, jointeval=None, exportvar=None, exportcoeffeqs=None,exportfnname=None,isHinges=None,rootmaxdim=16,jointevalcos=None,jointevalsin=None):
        self.jointnames=jointnames
        self.jointeval = jointeval
        self.isHinges = isHinges
        self.exportvar=exportvar
        self.exportcoeffeqs=exportcoeffeqs
        self.exportfnname=exportfnname
        self.rootmaxdim=rootmaxdim
        self.jointevalsin=jointevalsin
        self.jointevalcos=jointevalcos
    def numsolutions(self):
        return self.rootmaxdim
    def subs(self,solsubs):
        if self.jointeval is not None:
            self.jointeval = [e.subs(solsubs) for e in self.jointeval]
        if self.jointevalcos is not None:
            self.jointevalcos = [e.subs(solsubs) for e in self.jointevalcos]
        if self.jointevalsin is not None:
            self.jointevalsin = [e.subs(solsubs) for e in self.jointevalsin]
        if self.checkforzeros is not None:
            self.checkforzeros = [e.subs(solsubs) for e in self.checkforzeros]
        #if self.poly is not None:
        #    self.poly = self.poly.subs(solsubs)
        assert self.checkValidSolution()
        return self
    def generate(self, generator):
        return generator.generateCoeffFunction(self)
    def end(self, generator):
        return generator.endCoeffFunction(self)
    def checkValidSolution(self):
        #if self.poly is not None:
        #    valid = IKFastSolver.isValidSolution(self.poly.as_basic())
        if self.jointeval is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointeval])
        if self.jointevalcos is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointevalcos])
        if self.jointevalsin is not None:
            valid &= all([IKFastSolver.isValidSolution(e) for e in self.jointevalsin])
        return valid
    #def getPresetCheckForZeros(self):
    #    return [self.poly.coeffs[0]] # make sure highest coefficient is not 0!

class SolverMatrixInverse:
    """find all roots of the polynomial and plug it into jointeval. poly should be polys.polynomial.Poly
    """
    A = None
    Asymbols = None # has to be same size as B
    checkforzeros = None
    def __init__(self, A, Asymbols):
        self.A = A
        self.Asymbols = Asymbols
    def subs(self,solsubs):
        return self
    def generate(self, generator):
        return generator.generateMatrixInverse(self)
    def end(self, generator):
        return generator.endMatrixInverse(self)
    def checkValidSolution(self):
        return True
    def getsubs(self,psubs):
        Anew = self.A.subs(psubs).inv()
        subs = []
        for i in range(self.A.shape[0]):
            for j in range(self.A.shape[1]):
                subs.append((self.Asymbols[i][j],Anew[i,j]))
        return subs

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
    thresh=None # a threshold of 1e-6 breaks hiro ik
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

class SolverRotation:
    T = None
    jointtree = None
    functionid=0
    def __init__(self, T, jointtree):
        self.T = T
        self.jointtree = jointtree
        self.dictequations = []
    def generate(self, generator):
        return generator.generateRotation(self)
    def end(self, generator):
        return generator.endRotation(self)

class SolverStoreSolution:
    alljointvars = None
    checkgreaterzero = None # used for final sanity checks to ensure IK solution is consistent
    thresh = 0
    def __init__(self, alljointvars,checkgreaterzero=None):
        self.alljointvars = alljointvars
        self.checkgreaterzero = checkgreaterzero
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

class SolverBreak:
    def generate(self,generator):
        return generator.generateBreak(self)
    def end(self,generator):
        return generator.endBreak(self)

class SolverIKChainTransform6D:
    solvejointvars = None
    freejointvars = None
    jointtree = None
    Tfk = None
    Tee = None
    dictequations = None
    def __init__(self, solvejointvars, freejointvars, Tee, jointtree,Tfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Tee = Tee
        self.jointtree = jointtree
        self.Tfk = Tfk
        self.dictequations = []
    def generate(self, generator):
        return generator.generateChain(self)
    def end(self, generator):
        return generator.endChain(self)
    def leftmultiply(self,Tleft,Tleftinv):
        self.Tfk = Tleft*self.Tfk
        self.Tee = Tleftinv*self.Tee

class SolverIKChainRotation3D:
    solvejointvars = None
    freejointvars = None
    Rfk = None
    Ree = None
    jointtree = None
    dictequations = None
    def __init__(self, solvejointvars, freejointvars, Ree, jointtree,Rfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Ree = Ree
        self.Rfk=Rfk
        self.jointtree = jointtree
        self.dictequations = []
    def generate(self, generator):
        return generator.generateIKChainRotation3D(self)
    def end(self, generator):
        return generator.endIKChainRotation3D(self)
    def leftmultiply(self,Tleft,Tleftinv):
        self.Rfk = Tleft[0:3,0:3]*self.Rfk
        self.Ree = Tleftinv[0:3,0:3]*self.Ree

class SolverIKChainTranslation3D:
    solvejointvars = None
    freejointvars = None
    jointtree = None
    Pfk = None
    Pee = None
    dictequations = None
    def __init__(self, solvejointvars, freejointvars, Pee, jointtree,Pfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Pee = Pee
        self.jointtree = jointtree
        self.Pfk=Pfk
        self.dictequations = []
    def generate(self, generator):
        return generator.generateIKChainTranslation3D(self)
    def end(self, generator):
        return generator.endIKChainTranslation3D(self)
    def leftmultiply(self,Tleft,Tleftinv):
        self.Pfk = Tleft[0:3,0:3]*self.Pfk+Tleft[0:3,3]
        self.Pee = Tleftinv[0:3,0:3]*self.Pee+Tleftinv[0:3,3]

class SolverIKChainDirection3D:
    solvejointvars = None
    freejointvars = None
    jointtree = None
    Dfk = None
    Dee = None
    dictequations = None
    def __init__(self, solvejointvars, freejointvars, Dee, jointtree,Dfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Dee = Dee
        self.jointtree = jointtree
        self.Dfk=Dfk
        self.dictequations = []
    def generate(self, generator):
        return generator.generateIKChainDirection3D(self)
    def end(self, generator):
        return generator.endIKChainDirection3D(self)
    def leftmultiply(self,Tleft,Tleftinv):
        self.Dfk = Tleft[0:3,0:3]*self.Dfk
        self.Dee = Tleftinv[0:3,0:3]*self.Dee

class SolverIKChainRay:
    solvejointvars = None
    freejointvars = None
    jointtree = None
    Pfk = None
    Dfk = None
    Pee = None
    Dee = None
    dictequations = None
    is5dray = False # if True, then full 3D position becomes important and things shouldn't be normalized
    def __init__(self, solvejointvars, freejointvars, Pee, Dee, jointtree,Pfk=None,Dfk=None,is5dray=False):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Pee = Pee
        self.Dee = Dee
        self.jointtree = jointtree
        self.Pfk = Pfk
        self.Dfk = Dfk
        self.dictequations = []
        self.is5dray=is5dray
    def generate(self, generator):
        return generator.generateIKChainRay(self)
    def end(self, generator):
        return generator.endIKChainRay(self)
    def leftmultiply(self,Tleft,Tleftinv):
        self.Pfk = Tleft[0:3,0:3]*self.Pfk+Tleft[0:3,3]
        self.Dfk = Tleft[0:3,0:3]*self.Dfk
        self.Pee = Tleftinv[0:3,0:3]*self.Pee+Tleftinv[0:3,3]
        self.Dee = Tleftinv[0:3,0:3]*self.Dee

class SolverIKChainLookat3D:
    solvejointvars = None
    freejointvars = None
    jointtree = None
    Pfk = None
    Dfk = None
    Pee = None
    dictequations = None
    def __init__(self, solvejointvars, freejointvars, Pee, jointtree,Pfk=None,Dfk=None):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Pee = Pee
        self.jointtree = jointtree
        self.Pfk=Pfk
        self.Dfk=Dfk
        self.dictequations = []
    def generate(self, generator):
        return generator.generateIKChainLookat3D(self)
    def end(self, generator):
        return generator.endIKChainLookat3D(self)
    def leftmultiply(self,Tleft,Tleftinv):
        self.Pfk = Tleft[0:3,0:3]*self.Pfk+Tleft[0:3,3]
        self.Dfk = Tleft[0:3,0:3]*self.Dfk
        self.Pee = Tleftinv[0:3,0:3]*self.Pee+Tleftinv[0:3,3]

class fmod(core.function.Function):
    nargs = 2
    is_real = True
    is_Function = True

class IKFastSolver(AutoReloader):
    """Solves the analytical inverse kinematics equations. The symbol naming conventions are as follows:

    cjX - cos joint angle
    constX - temporary constant used to simplify computations    
    dummyX - dummy intermediate variables to solve for
    gconstX - global constant that is also used during ik generation phase
    htjX - half tan of joint angle
    jX - joint angle
    pX - end effector position information
    rX - end effector rotation information
    sjX - sin joint angle
    tconstX - second-level temporary constant
    tjX - tan of joint angle    
    """

    class CannotSolveError(Exception):
        def __init__(self,value):
            self.value=value
        def __str__(self):
            return repr(self.value)

    class JointAxis:
        __slots__ = ['joint','iaxis']

    class Variable:
        __slots__ = ['var','svar','cvar','tvar','htvar']
        def __init__(self, var):
            self.name = var.name
            self.var = var
            self.svar = Symbol("s%s"%var.name)
            self.cvar = Symbol("c%s"%var.name)
            self.tvar = Symbol("t%s"%var.name)
            self.htvar = Symbol("ht%s"%var.name)
            self.vars = [self.var,self.svar,self.cvar,self.tvar,self.htvar]
            self.subs = [(cos(self.var),self.cvar),(sin(self.var),self.svar),(tan(self.var),self.tvar),(tan(self.var/2),self.htvar)]
            self.subsinv = [(self.cvar,cos(self.var)),(self.svar, sin(self.var)),(self.tvar,tan(self.tvar))]

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

    def __init__(self, kinbody=None,kinematicshash='',precision=None):
        self.usinglapack = False
        self.freevarsubs = []
        self.degeneratecases = None
        self.kinematicshash = kinematicshash
        self.testconsistentvalues = None
        if precision is None:
            self.precision=8
        else:
            self.precision=precision
        self.kinbody = kinbody
        self.axismap = {}
        self.axismapinv = {}
        with self.kinbody:
            for idof in range(self.kinbody.GetDOF()):
                axis = IKFastSolver.JointAxis()
                axis.joint = self.kinbody.GetJointFromDOFIndex(idof)
                axis.iaxis = idof-axis.joint.GetDOFIndex()
                name = str('j%d')%idof
                self.axismap[name] = axis
                self.axismapinv[idof] = name

    def convertRealToRational(self, x,precision=None):
        if precision is None:
            precision=self.precision
        if abs(x) < 10**-precision:
            return S.Zero
        r0 = Rational(str(round(Real(float(x),30),precision)))
        if x == 0:
            return r0
        r1 = 1/Rational(str(round(Real(1/float(x),30),precision)))
        return r0 if len(str(r0)) < len(str(r1)) else r1

    def normalizeRotation(self,M):
        """error from openrave can be on the order of 1e-6 (especially if they are defined diagonal to some axis)
        """
        right = Matrix(3,1,[self.convertRealToRational(x,self.precision-3) for x in M[0,0:3]])
        right = right/right.norm()
        up = Matrix(3,1,[self.convertRealToRational(x,self.precision-3) for x in M[1,0:3]])
        up = up - right*right.dot(up)
        up = up/up.norm()
        d = right.cross(up)
        for i in range(3):
            # don't round the rotational part anymore since it could lead to unnormalized rotations!
            M[0,i] = right[i]
            M[1,i] = up[i]
            M[2,i] = d[i]
            M[i,3] = self.convertRealToRational(M[i,3])
            M[3,i] = S.Zero
        M[3,3] = S.One
        return M

    def numpyMatrixToSympy(self,T):
        return self.normalizeRotation(Matrix(4,4,[Real(x,30) for x in T.flat]))

    def numpyVectorToSympy(self,v,precision=None):
        return Matrix(len(v),1,[self.convertRealToRational(x,precision) for x in v])

    @staticmethod
    def rodrigues(axis, angle):
        return IKFastSolver.rodrigues2(axis,cos(angle),sin(angle))

    @staticmethod
    def rodrigues2(axis, cosangle, sinangle):
        skewsymmetric = Matrix(3, 3, [S.Zero,-axis[2],axis[1],axis[2],S.Zero,-axis[0],-axis[1],axis[0],S.Zero])
        return eye(3) + sinangle * skewsymmetric + (S.One-cosangle)*skewsymmetric*skewsymmetric

    @staticmethod
    def affineInverse(affinematrix):
        T = eye(4)
        T[0:3,0:3] = affinematrix[0:3,0:3].transpose()
        T[0:3,3] = -affinematrix[0:3,0:3].transpose() * affinematrix[0:3,3]
        return T

    @staticmethod
    def affineSimplify(T):
        return Matrix(T.shape[0],T.shape[1],[trigsimp(x.expand()) for x in T])

    @staticmethod
    def multiplyMatrix(Ts):
        Tfinal = eye(4)
        for T in Ts:
            Tfinal = Tfinal*T
        return Tfinal

    @staticmethod
    def equal(eq0,eq1):
        return (eq0-eq1).expand() == S.Zero

    def chop(self,expr,precision=None):
        return expr

    def isHinge(self,axisname):
        if axisname[0]!='j' or not axisname in self.axismap:
            if axisname.startswith('dummy') >= 0:
                return False
            print 'isHinge returning true for joint: %s'%axisname
            return True # dummy joint most likely for angles
        
        return self.axismap[axisname].joint.IsRevolute(self.axismap[axisname].iaxis)

    def forwardKinematicsChain(self, chainlinks, chainjoints):
        """The first and last matrices returned are always non-symbolic
        """
        with self.kinbody:
            assert(len(chainjoints)+1==len(chainlinks))
            Links = []
            Tright = eye(4)
            jointvars = []
            jointinds = []
            for i,joint in enumerate(chainjoints):
                if chainjoints[i].GetHierarchyParentLink() == chainlinks[i]:
                    TLeftjoint = self.numpyMatrixToSympy(joint.GetInternalHierarchyLeftTransform())
                    TRightjoint = self.numpyMatrixToSympy(joint.GetInternalHierarchyRightTransform())
                    axissign = S.One
                else:
                    TLeftjoint = self.affineInverse(self.numpyMatrixToSympy(joint.GetInternalHierarchyRightTransform()))
                    TRightjoint = self.affineInverse(self.numpyMatrixToSympy(joint.GetInternalHierarchyLeftTransform()))
                    axissign = -S.One
                if joint.IsStatic():
                    Tright = self.affineSimplify(Tright * TLeftjoint * TRightjoint)
                else:
                    Tjoints = []
                    for iaxis in range(joint.GetDOF()):
                        if joint.GetDOFIndex() >= 0:
                            var = Symbol(self.axismapinv[joint.GetDOFIndex()])
                            cosvar = cos(var)
                            sinvar = sin(var)
                            jointvars.append(var)
                        elif joint.IsMimic(iaxis):
                            # get the mimic equation
                            var = joint.GetMimicEquation(iaxis)
                            # this needs to be reduced!
                            cosvar = cos(var)
                            sinvar = sin(var)
                        else:
                            raise ValueError('cannot solve for mechanism when a non-mimic passive joint %s is in chain'%str(joint))
                        
                        Tj = eye(4)
                        jaxis = axissign*self.numpyVectorToSympy(joint.GetInternalHierarchyAxis(iaxis))
                        if joint.IsRevolute(iaxis):
                            Tj[0:3,0:3] = self.rodrigues2(jaxis,cosvar,sinvar)
                        elif joint.IsPrismatic(iaxis):
                            Tj[0:3,3] = jaxis*(var)
                        else:
                            raise ValueError('failed to process joint %s'%joint.GetName())
                        
                        Tjoints.append(Tj)
                    Links.append(Tright * TLeftjoint)
                    for Tj in Tjoints:
                        jointinds.append(len(Links))
                        Links.append(Tj)
                    Tright = TRightjoint
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
                if separated_trans[j].has_any_symbols(*jointvars):
                    Ttrans[j,3] = Rational(0)
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
                if not separated_trans[j].has_any_symbols(*jointvars):
                    Ttrans[j,3] = separated_trans[j]
            Links[ileft-1] = Links[ileft-1] * Ttrans
            Links[ileft+1] = self.affineInverse(Ttrans) * Links[ileft+1]
            print "moved translation ",Ttrans[0:3,3].transpose(),"to left end"

        if len(jointinds) > 3: # last 3 axes always have to be intersecting, move the translation of the first axis to the left
            ileft = jointinds[-3]
            separated_trans = Links[ileft][0:3,0:3] * Links[ileft+1][0:3,3]
            Ttrans = eye(4)
            for j in range(0,3):
                if not separated_trans[j].has_any_symbols(*jointvars):
                    Ttrans[j,3] = separated_trans[j]
            Links[ileft-1] = Links[ileft-1] * Ttrans
            Links[ileft+1] = self.affineInverse(Ttrans) * Links[ileft+1]
            print "moved translation on intersecting axis ",Ttrans[0:3,3].transpose(),"to left"

        return Links, jointvars
        
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

    @staticmethod
    def isValidPowers(expr):
        if expr.is_Pow:
            if not expr.exp.is_number or expr.exp < 0:
                return False
            return IKFastSolver.isValidPowers(expr.base)
        elif expr.is_Add or expr.is_Mul or expr.is_Function:
            return all([IKFastSolver.isValidPowers(arg) for arg in expr.args])
        else:
            return True

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
    def has_any_symbols(eqs,*sym):
        return any([eq.has_any_symbols(*sym) for eq in eqs])

    @staticmethod
    def trigsimp(eq,trigvars):
        trigsubs = [(sin(v)**2,1-cos(v)**2) for v in trigvars]
        eq=eq.expand()
        curcount = eq.count_ops()
        while True:
            eq=eq.subs(trigsubs).expand()
            newcount = eq.count_ops()
            if IKFastSolver.equal(curcount,newcount):
                break
            curcount=newcount
        return eq
        
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
                    if expr.exp.is_number and expr.exp < 0:
                        base = self.subsExpressions(expr.base,subexprs)
                        if not base.is_number:
                            eq=self.subsExpressions(expr.base,subexprs)
                            if self.isExpressionUnique(checkforzeros,-eq) and self.isExpressionUnique(checkforzeros,eq):
                                checkforzeros.append(self.removecommonexprs(eq,onlygcd=False,onlynumbers=True))

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
        sol.checkforzeros = sol.getPresetCheckForZeros()
        sol.score = 0
        try:
            # multiby by 400 in order to prioritize equations with less solutions
            if sol.jointeval is not None:
                sol.score = 20000*len(sol.jointeval)
                for s in sol.jointeval:
                    sol.score += self.codeComplexity(s)
                subexprs = sol.jointeval
            elif sol.jointevalsin is not None:
                sol.score = 20000*len(sol.jointevalsin)
                for s in sol.jointevalsin:
                    sol.score += self.codeComplexity(s)
                subexprs = sol.jointevalsin
            elif sol.jointevalcos is not None:
                sol.score = 20000*len(sol.jointevalcos)
                for s in sol.jointevalcos:
                    sol.score += self.codeComplexity(s)
                subexprs = sol.jointevalcos
            else:
                assert False
            
            def checkpow(expr,sexprs):
                score = 0
                if expr.is_Pow:
                    sexprs.append(expr.base)
                    if expr.base.is_finite is not None and not expr.base.is_finite:
                        return oo # infinity
                    if expr.exp.is_number and expr.exp < 0:
                        #exprbase = self.subsExpressions(expr.base,subexprs)
                        # check if exprbase contains any variables that have already been solved
                        containsjointvar = expr.base.has_any_symbols(*solvedvars)
                        cancheckexpr = not expr.base.has_any_symbols(*unsolvedvars)
                        score += 10000
                        if not cancheckexpr:
                            score += 100000
                        else:
                            if self.isExpressionUnique(sol.checkforzeros,-expr.base) and self.isExpressionUnique(sol.checkforzeros,expr.base):
                                sol.checkforzeros.append(self.removecommonexprs(expr.base,onlygcd=False,onlynumbers=True))
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

    def writeIkSolver(self,chaintree,lang=None):
        # parse the solver tree
        if lang is None:
            # prioritize c++
            generator = CodeGenerators.get('cpp',CodeGenerators.values()[0])
        return CodeGenerators[lang](kinematicshash=self.kinematicshash,version=__version__).generate(chaintree)

    def generateIkSolver(self, baselink, eelink, freejointinds=None,solvefn=None):
        if solvefn is None:
            solvefn = IKFastSolver.solveFullIK_6D
        chainlinks = self.kinbody.GetChain(baselink,eelink,returnjoints=False)
        chainjoints = self.kinbody.GetChain(baselink,eelink,returnjoints=True)
        LinksRaw, jointvars = self.forwardKinematicsChain(chainlinks,chainjoints)
        self.degeneratecases = None

        if freejointinds is None:
            # need to iterate through all combinations of free joints
            assert(0)
        isolvejointvars = []
        solvejointvars = []
        self.ifreejointvars = []
        self.freevarsubs = []
        self.freevars = []
        self.freejointvars = []
        self.invsubs = []
        for i,v in enumerate(jointvars):
            var = self.Variable(v)
            axis = self.axismap[v.name]
            dofindex = axis.joint.GetDOFIndex()+axis.iaxis
            if dofindex in freejointinds:
                # convert all free variables to constants
                self.ifreejointvars.append(i)
                self.freevarsubs += [(cos(var.var), var.cvar), (sin(var.var), var.svar)]
                self.freevars += [var.cvar,var.svar]
                self.freejointvars.append(var.var)
            else:
                solvejointvars.append(v)
                isolvejointvars.append(i)
                self.invsubs += [(var.cvar,cos(v)),(var.svar,sin(v))]

        # set up the destination symbols
        self.Tee = eye(4)
        for i in range(0,3):
            for j in range(0,3):
                self.Tee[i,j] = Symbol("r%d%d"%(i,j))
        self.Tee[0,3] = Symbol("px")
        self.Tee[1,3] = Symbol("py")
        self.Tee[2,3] = Symbol("pz")
        r00,r01,r02,px,r10,r11,r12,py,r20,r21,r22,pz = self.Tee[0:12]
        self.pp = Symbol('pp')
        self.ppsubs = [(self.pp,px**2+py**2+pz**2)]
        self.npxyz = [Symbol('npx'),Symbol('npy'),Symbol('npz')]
        self.npxyzsubs = [(self.npxyz[i],px*self.Tee[0,i]+py*self.Tee[1,i]+pz*self.Tee[2,i]) for i in range(3)]
        # cross products between columns of self.Tee
        self.rxp = []
        self.rxpsubs = []
        for i in range(3):
            self.rxp.append([Symbol('rxp%d_%d'%(i,j)) for j in range(3)])
            c = self.Tee[0:3,i].cross(self.Tee[0:3,3])    
            self.rxpsubs += [(self.rxp[-1][j],c[j]) for j in range(3)]
        self.pvars = self.Tee[0:12]+self.npxyz+[self.pp]+self.rxp[0]+self.rxp[1]+self.rxp[2]
        self.Teeinv = self.affineInverse(self.Tee)
        LinksLeft = []
        while not self.has_any_symbols(LinksRaw[0],*solvejointvars):
            LinksLeft.append(LinksRaw.pop(0))
        LinksLeftInv = [self.affineInverse(T) for T in LinksLeft]
        self.testconsistentvalues = None
        # before passing to the solver, set big numbers to constants, this will greatly reduce computation times
        self.gsymbolgen = cse_main.numbered_symbols('gconst')
        self.globalsymbols = []
#         for T in LinksRaw:
#             for i in range(12):
#                 if T[i].is_number and len(str(T[i])) > 30:
#                     sym = None
#                     for c,v in self.globalsymbols:
#                         if self.equal(v,T[i]):
#                             sym = c
#                             break
#                     if sym is None:
#                         sym = self.gsymbolgen.next()
#                         print 'adding global symbol %s=%s'%(sym,T[i])
#                         self.globalsymbols.append((sym,T[i]))
#                     T[i] = sym
        chaintree = solvefn(self, LinksRaw, jointvars, isolvejointvars)
        chaintree.leftmultiply(Tleft=self.multiplyMatrix(LinksLeft), Tleftinv=self.multiplyMatrix(LinksLeftInv[::-1]))
        chaintree.dictequations += self.globalsymbols
        return chaintree

    def computeConsistentValues(self,jointvars,T,numsolutions=1,subs=None):
        possibleangles = [S.Zero, pi.evalf()/2, asin(3.0/5).evalf(), asin(4.0/5).evalf(), asin(5.0/13).evalf(), asin(12.0/13).evalf()]
        testconsistentvalues = []
        for isol in range(numsolutions):
            jointvalues = [S.Zero]*len(jointvars)
            if isol < numsolutions-1:
                for j in range(len(jointvars)):
                    jointvalues[j] = possibleangles[(isol+j)%len(possibleangles)]
            valsubs = []
            for var,value in izip(jointvars,jointvalues):
                valsubs += [(var,value),(Symbol('c%s'%var.name),self.convertRealToRational(cos(value).evalf())),(Symbol('s%s'%var.name),self.convertRealToRational(sin(value).evalf())),(Symbol('t%s'%var.name),self.convertRealToRational(tan(value).evalf())),(Symbol('ht%s'%var.name),self.convertRealToRational(tan(value/2).evalf()))]
            psubs = []
            for i in range(12):
                psubs.append((self.pvars[i],self.convertRealToRational(T[i].subs(self.globalsymbols+valsubs).evalf())))
            for s,v in self.ppsubs+self.npxyzsubs+self.rxpsubs:
                psubs.append((s,v.subs(psubs)))
            allsubs = valsubs+psubs
            if subs is not None:
                allsubs += [(dvar,self.convertRealToRational(var.subs(valsubs).evalf())) for dvar,var in subs]
            testconsistentvalues.append(allsubs)
        return testconsistentvalues

    def solveFullIK_Direction3D(self,LinksRaw, jointvars, isolvejointvars, rawbasedir=Matrix(3,1,[S.Zero,S.Zero,S.One])):
        """basedir needs to be filled with a 3elemtn vector of the initial direction to control"""
        basedir = Matrix(3,1,[Real(x,30) for x in rawbasedir])
        basedir /= sqrt(basedir[0]*basedir[0]+basedir[1]*basedir[1]+basedir[2]*basedir[2])
        for i in range(3):
            basedir[i] = self.convertRealToRational(basedir[i])
        Links = LinksRaw[:]
        LinksInv = [self.affineInverse(link) for link in Links]
        T = self.multiplyMatrix(Links)
        Tfinal = zeros((4,4))
        Tfinal[0,0:3] = (T[0:3,0:3]*basedir).transpose()
        self.testconsistentvalues = self.computeConsistentValues(jointvars,Tfinal,numsolutions=4)
        endbranchtree = [SolverStoreSolution (jointvars)]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        if len(solvejointvars) != 2:
            raise self.CannotSolveError('need 2 joints')

        print 'ikfast direction3d: ',solvejointvars

        Daccum = self.Tee[0,0:3].transpose()
        numvarsdone = 2
        Ds = []
        Dsee = []
        for i in range(len(Links)-1):
            T = self.multiplyMatrix(Links[i:])
            D = T[0:3,0:3]*basedir
            hasvars = [self.has_any_symbols(D,v) for v in solvejointvars]
            if __builtin__.sum(hasvars) == numvarsdone:
                Ds.append(D)
                Dsee.append(Daccum)
                numvarsdone -= 1
            Tinv = self.affineInverse(Links[i])
            Daccum = Tinv[0:3,0:3]*Daccum
        AllEquations = self.buildEquationsFromTwoSides(Ds,Dsee,jointvars,uselength=False)
        tree = self.solveAllEquations(AllEquations,curvars=solvejointvars,othersolvedvars = self.freejointvars[:],solsubs = self.freevarsubs[:],endbranchtree=endbranchtree)
        tree = self.verifyAllEquations(AllEquations,solvejointvars,self.freevarsubs,tree)
        return SolverIKChainDirection3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(v,i) for v,i in izip(self.freejointvars,self.ifreejointvars)], Dee=self.Tee[0,0:3].transpose().subs(self.freevarsubs), jointtree=tree,Dfk=Tfinal[0,0:3].transpose())

    def solveFullIK_Lookat3D(self,LinksRaw, jointvars, isolvejointvars,rawbasedir=Matrix(3,1,[S.Zero,S.Zero,S.One]),rawbasepos=Matrix(3,1,[S.Zero,S.Zero,S.Zero])):
        """basedir,basepos needs to be filled with a direction and position of the ray to control the lookat
        """
        basedir = Matrix(3,1,[Real(x,30) for x in rawbasedir])
        basepos = Matrix(3,1,[self.convertRealToRational(x) for x in rawbasepos])
        basedir /= sqrt(basedir[0]*basedir[0]+basedir[1]*basedir[1]+basedir[2]*basedir[2])
        for i in range(3):
            basedir[i] = self.convertRealToRational(basedir[i])
        basepos = basepos-basedir*basedir.dot(basepos)
        Links = LinksRaw[:]
        LinksInv = [self.affineInverse(link) for link in Links]
        T = self.multiplyMatrix(Links)
        Tfinal = zeros((4,4))
        Tfinal[0,0:3] = (T[0:3,0:3]*basedir).transpose()
        Tfinal[0:3,3] = T[0:3,0:3]*basepos+T[0:3,3]
        self.testconsistentvalues = self.computeConsistentValues(jointvars,Tfinal,numsolutions=4)
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        if len(solvejointvars) != 2:
            raise self.CannotSolveError('need 2 joints')

        print 'ikfast lookat3d: ',solvejointvars
        
        Paccum = self.Tee[0:3,3]
        numvarsdone = 2
        Positions = []
        Positionsee = []
        for i in range(len(Links)-1):
            T = self.multiplyMatrix(Links[i:])
            P = T[0:3,0:3]*basepos+T[0:3,3]
            D = T[0:3,0:3]*basedir
            hasvars = [self.has_any_symbols(P,v) or self.has_any_symbols(D,v) for v in solvejointvars]
            if __builtin__.sum(hasvars) == numvarsdone:
                Positions.append(P.cross(D))
                Positionsee.append(Paccum.cross(D))
                numvarsdone -= 1
            Tinv = self.affineInverse(Links[i])
            Paccum = Tinv[0:3,0:3]*Paccum+Tinv[0:3,3]

        frontcond = (Links[-1][0:3,0:3]*basedir).dot(Paccum-(Links[-1][0:3,0:3]*basepos+Links[-1][0:3,3]))
        for v in jointvars:
            frontcond = frontcond.subs(self.Variable(v).subs)
        endbranchtree = [SolverStoreSolution (jointvars,checkgreaterzero=[frontcond])]
        AllEquations = self.buildEquationsFromTwoSides(Positions,Positionsee,jointvars,uselength=True)
        tree = self.solveAllEquations(AllEquations,curvars=solvejointvars,othersolvedvars = self.freejointvars[:],solsubs = self.freevarsubs[:],endbranchtree=endbranchtree)
        tree = self.verifyAllEquations(AllEquations,solvejointvars,self.freevarsubs,tree)
        chaintree = SolverIKChainLookat3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(v,i) for v,i in izip(self.freejointvars,self.ifreejointvars)], Pee=self.Tee[0:3,3].subs(self.freevarsubs), jointtree=tree,Dfk=Tfinal[0,0:3].transpose(),Pfk=Tfinal[0:3,3])
        chaintree.dictequations += self.ppsubs
        return chaintree

    def solveFullIK_Rotation3D(self,LinksRaw, jointvars, isolvejointvars, Rbaseraw=eye(3)):
        Rbase = eye(4)
        for i in range(3):
            for j in range(3):
                Rbase[i,j] = self.convertRealToRational(Rbaseraw[i,j])
        Tfirstright = LinksRaw[-1]*Rbase
        Links = LinksRaw[:-1]
        LinksInv = [self.affineInverse(link) for link in Links]
        Tfinal = self.multiplyMatrix(Links)
        self.testconsistentvalues = self.computeConsistentValues(jointvars,Tfinal,numsolutions=4)
        endbranchtree = [SolverStoreSolution (jointvars)]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        if len(solvejointvars) != 3:
            raise self.CannotSolveError('need 3 joints')
        
        print 'ikfast rotation3d: ',solvejointvars

        AllEquations = self.buildEquationsFromRotation(Links,self.Tee[0:3,0:3],solvejointvars,self.freejointvars)
        tree = self.solveAllEquations(AllEquations,curvars=solvejointvars[:],othersolvedvars=self.freejointvars,solsubs = self.freevarsubs[:],endbranchtree=endbranchtree)
        tree = self.verifyAllEquations(AllEquations,solvejointvars,self.freevarsubs,tree)
        return SolverIKChainRotation3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(v,i) for v,i in izip(self.freejointvars,self.ifreejointvars)], (self.Tee[0:3,0:3] * self.affineInverse(Tfirstright)[0:3,0:3]).subs(self.freevarsubs), tree, Rfk = Tfinal[0:3,0:3] * Tfirstright[0:3,0:3])

    def solveFullIK_Translation3D(self,LinksRaw, jointvars, isolvejointvars, rawbasepos=Matrix(3,1,[S.Zero,S.Zero,S.Zero])):
        basepos = Matrix(3,1,[self.convertRealToRational(x) for x in rawbasepos])
        Links = LinksRaw[:]
        LinksInv = [self.affineInverse(link) for link in Links]
        Tfinal = self.multiplyMatrix(Links)
        Tfinal[0:3,3] = Tfinal[0:3,0:3]*basepos+Tfinal[0:3,3]
        self.testconsistentvalues = self.computeConsistentValues(jointvars,Tfinal,numsolutions=4)
        endbranchtree = [SolverStoreSolution (jointvars)]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        if len(solvejointvars) != 3:
            raise self.CannotSolveError('need 3 joints')
        
        print 'ikfast translation3d: ',solvejointvars
        Tbaseposinv = eye(4)
        Tbaseposinv[0:3,3] = -basepos
        T1links = [Tbaseposinv]+LinksInv[::-1]+[self.Tee]
        T1linksinv = [self.affineInverse(Tbaseposinv)]+Links[::-1]+[self.Teeinv]
        AllEquations = self.buildEquationsFromPositions(T1links,T1linksinv,solvejointvars,self.freejointvars,uselength=True)
        transtree = self.solveAllEquations(AllEquations,curvars=solvejointvars[:],othersolvedvars=self.freejointvars,solsubs = self.freevarsubs[:],endbranchtree=endbranchtree)
        transtree = self.verifyAllEquations(AllEquations,solvejointvars,self.freevarsubs,transtree)
        chaintree = SolverIKChainTranslation3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(v,i) for v,i in izip(self.freejointvars,self.ifreejointvars)], Pee=self.Tee[0:3,3], jointtree=transtree, Pfk = Tfinal[0:3,3])
        chaintree.dictequations += self.ppsubs
        return chaintree

    def solveFullIK_Ray4D(self,LinksRaw, jointvars, isolvejointvars, rawbasedir=Matrix(3,1,[S.Zero,S.Zero,S.One]),rawbasepos=Matrix(3,1,[S.Zero,S.Zero,S.Zero])):
        """basedir,basepos needs to be filled with a direction and position of the ray to control"""
        basedir = Matrix(3,1,[Real(x,30) for x in rawbasedir])
        basepos = Matrix(3,1,[self.convertRealToRational(x) for x in rawbasepos])
        basedir /= sqrt(basedir[0]*basedir[0]+basedir[1]*basedir[1]+basedir[2]*basedir[2])
        for i in range(3):
            basedir[i] = self.convertRealToRational(basedir[i])
        basepos = basepos-basedir*basedir.dot(basepos)
        Links = LinksRaw[:]
        LinksInv = [self.affineInverse(link) for link in Links]
        T = self.multiplyMatrix(Links)
        Tfinal = zeros((4,4))
        Tfinal[0,0:3] = (T[0:3,0:3]*basedir).transpose()
        Tfinal[0:3,3] = T[0:3,0:3]*basepos+T[0:3,3]
        self.testconsistentvalues = self.computeConsistentValues(jointvars,Tfinal,numsolutions=4)
        endbranchtree = [SolverStoreSolution (jointvars)]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        if len(solvejointvars) != 4:
            raise self.CannotSolveError('need 4 joints')

        print 'ikfast ray4d: ',solvejointvars
        
        Pee = self.Tee[0:3,3]
        Dee = self.Tee[0,0:3].transpose()
        numvarsdone = 2
        Positions = []
        Positionsee = []
        for i in range(len(Links)-1):
            T = self.multiplyMatrix(Links[i:])
            P = T[0:3,0:3]*basepos+T[0:3,3]
            D = T[0:3,0:3]*basedir
            hasvars = [self.has_any_symbols(P,v) or self.has_any_symbols(D,v) for v in solvejointvars]
            if __builtin__.sum(hasvars) == numvarsdone:
                Positions.append(P.cross(D))
                Positionsee.append(Pee.cross(Dee))
                Positions.append(D)
                Positionsee.append(Dee)
                break
            Tinv = self.affineInverse(Links[i])
            Pee = Tinv[0:3,0:3]*Pee+Tinv[0:3,3]
            Dee = Tinv[0:3,0:3]*Dee
        AllEquations = self.buildEquationsFromTwoSides(Positions,Positionsee,jointvars,uselength=True)

        try:
            tree = self.solveAllEquations(AllEquations,curvars=solvejointvars[:],othersolvedvars = self.freejointvars[:],solsubs = self.freevarsubs[:],endbranchtree=endbranchtree)
        except self.CannotSolveError:
            # build the raghavan/roth equations and solve with higher power methods
            pass
        tree = self.verifyAllEquations(AllEquations,solvejointvars,self.freevarsubs,tree)
        chaintree = SolverIKChainRay([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(v,i) for v,i in izip(self.freejointvars,self.ifreejointvars)], Pee=self.Tee[0:3,3].subs(self.freevarsubs), Dee=self.Tee[0,0:3].transpose().subs(self.freevarsubs),jointtree=tree,Dfk=Tfinal[0,0:3].transpose(),Pfk=Tfinal[0:3,3])
        chaintree.dictequations += self.ppsubs
        return chaintree

    def solveFullIK_TranslationDirection5D(self, LinksRaw, jointvars, isolvejointvars, rawbasedir=Matrix(3,1,[S.Zero,S.Zero,S.One]),rawbasepos=Matrix(3,1,[S.Zero,S.Zero,S.Zero])):
        """Solves 3D translation + 3D direction
        """
        basepos = Matrix(3,1,[self.convertRealToRational(x) for x in rawbasepos])
        basedir = Matrix(3,1,[Real(x,30) for x in rawbasedir])
        basedir /= sqrt(basedir[0]*basedir[0]+basedir[1]*basedir[1]+basedir[2]*basedir[2])
        for i in range(3):
            basedir[i] = self.convertRealToRational(basedir[i])
        Links = LinksRaw[:]
        LinksInv = [self.affineInverse(link) for link in Links]
        T = self.multiplyMatrix(Links)
        Tfinal = zeros((4,4))
        Tfinal[0,0:3] = (T[0:3,0:3]*basedir).transpose()
        Tfinal[0:3,3] = T[0:3,0:3]*basepos+T[0:3,3]
        self.testconsistentvalues = self.computeConsistentValues(jointvars,Tfinal,numsolutions=4)
        endbranchtree = [SolverStoreSolution (jointvars)]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        if len(solvejointvars) != 5:
            raise self.CannotSolveError('need 5 joints')
        
        print 'ikfast translation direction 5d: ',solvejointvars

        # if last two axes are intersecting, can divide computing position and direction
        ilinks = [i for i,Tlink in enumerate(Links) if self.has_any_symbols(Tlink,*solvejointvars)]
        T = self.multiplyMatrix(Links[ilinks[-2]:])
        P = T[0:3,0:3]*basepos+T[0:3,3]
        D = T[0:3,0:3]*basedir
        tree = None
        if not self.has_any_symbols(P,*solvejointvars):
            Tposinv = eye(4)
            Tposinv[0:3,3] = -P
            T0links=[Tposinv]+Links[:ilinks[-2]]
            try:
                print 'last 2 axes are intersecting'
                tree = self.solve5DIntersectingAxes(T0links,basepos,D,endbranchtree)
            except self.CannotSolveError, e:
                print e

        if tree is None:
            rawpolyeqs2 = [None]*len(solvejointvars)
            coupledsolutions = None
            for solvemethod in [self.solveLiWoernleHiller, self.solveKohliOsvatic, self.solveManochaCanny]:
                if coupledsolutions is not None:
                    break
                for index in [2,3]:
                    T0links=LinksInv[:ilinks[index]][::-1]
                    T0 = self.multiplyMatrix(T0links)
                    T1links=Links[ilinks[index]:]
                    T1 = self.multiplyMatrix(T1links)
                    p0 = T0[0:3,0:3]*self.Tee[0:3,3]+T0[0:3,3]
                    p1 = T1[0:3,0:3]*basepos+T1[0:3,3]
                    l0 = T0[0:3,0:3]*self.Tee[0,0:3].transpose()
                    l1 = T1[0:3,0:3]*basedir
                    if rawpolyeqs2[index] is None:
                        rawpolyeqs2[index] = self.buildRaghavanRothEquations(p0,p1,l0,l1,solvejointvars)
                    try:
                        coupledsolutions,usedvars = solvemethod(rawpolyeqs2[index])
                        break
                    except self.CannotSolveError:
                        continue

            if coupledsolutions is None:
                raise self.CannotSolveError('raghavan roth equations too complex')

            print 'solving coupled variables: ',usedvars
            AllEquations = []
            for i in range(3):
                AllEquations.append(self.simplifyTransform(p0[i]-p1[i]))
                AllEquations.append(self.simplifyTransform(l0[i]-l1[i]))
            self.sortComplexity(AllEquations)
            curvars=solvejointvars[:]
            solsubs = self.freevarsubs[:]
            for var in usedvars:
                curvars.remove(var)
                solsubs += [(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)]
            tree = self.solveAllEquations(AllEquations,curvars=curvars,othersolvedvars = self.freejointvars+usedvars,solsubs = solsubs,endbranchtree=endbranchtree)
            tree = coupledsolutions+tree
        
        chaintree = SolverIKChainRay([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(v,i) for v,i in izip(self.freejointvars,self.ifreejointvars)], Pee=self.Tee[0:3,3].subs(self.freevarsubs), Dee=self.Tee[0,0:3].transpose().subs(self.freevarsubs),jointtree=tree,Dfk=Tfinal[0,0:3].transpose(),Pfk=Tfinal[0:3,3],is5dray=True)
        chaintree.dictequations += self.ppsubs
        return chaintree

    def solve5DIntersectingAxes(self, T0links, basepos, D, solvejointvars, endbranchtree):
        LinksInv = [self.affineInverse(T) for T in T0links]
        T0 = self.multiplyMatrix(T0links)
        Tbaseposinv = eye(4)
        Tbaseposinv[0:3,3] = -basepos
        T1links = [Tbaseposinv]+LinksInv[::-1]+[self.Tee]
        T1linksinv = [self.affineInverse(Tbaseposinv)]+T0links[::-1]+[self.Teeinv]
        AllEquations = self.buildEquationsFromPositions(T1links,T1linksinv,solvejointvars,self.freejointvars,uselength=True)
        transvars = [v for v in solvejointvars if self.has_any_symbols(T0,v)]
        transtree = self.solveAllEquations(AllEquations,curvars=transvars[:],othersolvedvars=self.freejointvars,solsubs = self.freevarsubs[:],endbranchtree=endbranchtree)
        transtree = self.verifyAllEquations(AllEquations,solvejointvars,self.freevarsubs,transtree)
        rotvars = [v for v in solvejointvars if self.has_any_symbols(D,v)]
        solsubs = self.freevarsubs[:]
        for v in transvars:
            solsubs += self.Variable(v).subs
        AllEquations = self.buildEquationsFromTwoSides([D],[T0[0:3,0:3]*self.Tee[0,0:3].transpose()],jointvars,uselength=False)
        dirtree = self.solveAllEquations(AllEquations,curvars=rotvars[:],othersolvedvars = self.freejointvars+transvars,solsubs=solsubs,endbranchtree=endbranchtree)
        dirtree = self.verifyAllEquations(AllEquations,rotvars,solsubs,dirtree)
        return transtree+dirtree

    def solveFullIK_6D(self, LinksRaw, jointvars, isolvejointvars,Tgripperraw=eye(4)):
        """Solves the full 6D translatio + rotation IK
        """
        Tgripper = eye(4)
        for i in range(4):
            for j in range(4):
                Tgripper[i,j] = self.convertRealToRational(Tgripperraw[i,j])
        Tfirstright = LinksRaw[-1]*Tgripper
        Links = LinksRaw[:-1]
        LinksInv = [self.affineInverse(link) for link in Links]
        Tfinal = self.multiplyMatrix(Links)
        self.testconsistentvalues = self.computeConsistentValues(jointvars,Tfinal,numsolutions=4)
        endbranchtree = [SolverStoreSolution (jointvars)]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        if len(solvejointvars) != 6:
            raise self.CannotSolveError('need 6 joints')
        
        print 'ikfast 6d: ',solvejointvars

        tree = None
        for T0links,T1links,transvars,rotvars,solveRotationFirst in self.iterateThreeIntersectingAxes(solvejointvars,Links, LinksInv):
            try:
                print 'found 3 consecutive intersecting axes: %s, translation=%s'%(str(transvars),str(rotvars))
                tree = self.solve6DIntersectingAxes(T0links,T1links,transvars,rotvars,solveRotationFirst=solveRotationFirst, endbranchtree=endbranchtree)
                break
            except self.CannotSolveError, e:
                print e
                        
        if tree is None:
            for T0links, T1links in self.iterateThreeNonIntersectingAxes(solvejointvars,Links, LinksInv):
                try:
                    print 'found 3 consecutive non-intersecting axes'
                    tree = self.solveFullIK_6DGeneral(T0links, T1links, solvejointvars, endbranchtree)
                    break
                except self.CannotSolveError, e:
                    print e

        if tree is None:
            raise self.CannotSolveError('cannot solve 6D mechanism!')

        chaintree = SolverIKChainTransform6D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(v,i) for v,i in izip(self.freejointvars,self.ifreejointvars)], (self.Tee * self.affineInverse(Tfirstright)).subs(self.freevarsubs), tree,Tfk=Tfinal*Tfirstright)
        chaintree.dictequations += self.ppsubs+self.npxyzsubs+self.rxpsubs
        return chaintree

    def iterateThreeIntersectingAxes(self, solvejointvars, Links, LinksInv):
        """Search for 3 consectuive intersecting axes. If a robot has this condition, it makes a lot of IK computations simpler.
        """
        ilinks = [i for i,Tlink in enumerate(Links) if self.has_any_symbols(Tlink,*solvejointvars)]
        for i in range(len(ilinks)-2):
            startindex = ilinks[i]
            endindex = ilinks[i+2]+1
            T0links = Links[startindex:endindex]
            T0 = self.multiplyMatrix(T0links)
            solveRotationFirst = None
            if not self.has_any_symbols(T0[:3,3],*solvejointvars):
                T1links = LinksInv[:startindex][::-1]
                T1links.append(self.Tee)
                T1links += LinksInv[endindex:][::-1]
                solveRotationFirst = False
            else:
                T0links = LinksInv[startindex:endindex][::-1]
                T0 = self.multiplyMatrix(T0links)
                if not self.has_any_symbols(T0[:3,3],*solvejointvars):
                    T1links = Links[endindex:]
                    T1links.append(self.Teeinv)
                    T1links += Links[:startindex]
                    solveRotationFirst = False
            if solveRotationFirst is not None:
                rotvars = []
                transvars = []
                for svar in solvejointvars:
                    if self.has_any_symbols(T0,svar):
                        rotvars.append(svar)
                    else:
                        transvars.append(svar)
                if len(rotvars) == 3 and len(transvars) == 3:
                    yield T0links,T1links,transvars,rotvars,solveRotationFirst

    def iterateThreeNonIntersectingAxes(self, solvejointvars, Links, LinksInv):
        """check for three consecutive non-intersecting axes.
        if several points exist, so have to choose one that is least complex?
        """
        ilinks = [i for i,Tlink in enumerate(Links) if self.has_any_symbols(Tlink,*solvejointvars)]
        for i in range(len(ilinks)-2):
            startindex = ilinks[i]
            endindex = ilinks[i+2]+1
            p0 = self.multiplyMatrix(Links[ilinks[i]:ilinks[i+1]])[0:3,3]
            p1 = self.multiplyMatrix(Links[ilinks[i+1]:ilinks[i+2]])[0:3,3]
            if self.has_any_symbols(p0,*solvejointvars) and self.has_any_symbols(p1,*solvejointvars):
                T0links = Links[startindex:endindex]
                T1links = LinksInv[:startindex][::-1]
                T1links.append(self.Tee)
                T1links += LinksInv[endindex:][::-1]
                yield T0links, T1links

    def solve6DIntersectingAxes(self, T0links, T1links, transvars,rotvars,solveRotationFirst,endbranchtree):
        """Solve 6D equations using fact that 3 axes are intersecting. The 3 intersecting axes are all part of T0links and will be used to compute the rotation of the robot. The other 3 axes are part of T1links and will be used to first compute the position.
        """
        assert(len(transvars)==3 and len(rotvars) == 3)
        T1 = self.multiplyMatrix(T1links)
        othersolvedvars = rotvars+self.freejointvars if solveRotationFirst else self.freejointvars
        T1linksinv = [self.affineInverse(T) for T in T1links]
        AllEquations = self.buildEquationsFromPositions(T1links,T1linksinv,transvars,othersolvedvars,uselength=True)
        rottree = []
        if solveRotationFirst:
            newendbranchtree = endbranchtree
        else:
            newendbranchtree = [SolverSequence([rottree])]
        curvars = transvars[:]
        solsubs=self.freevarsubs[:]
        transtree = self.solveAllEquations(AllEquations,curvars=curvars,othersolvedvars=othersolvedvars,solsubs=solsubs,endbranchtree=newendbranchtree)
        transtree = self.verifyAllEquations(AllEquations,rotvars if solveRotationFirst else transvars+rotvars,self.freevarsubs,transtree)
        solvertree= []
        solvedvarsubs = self.freevarsubs
        if solveRotationFirst:
            storesolutiontree = transtree
        else:
            solvertree += transtree
            storesolutiontree = endbranchtree
            for tvar in transvars:
                solvedvarsubs += self.Variable(tvar).subs

        oldglobalsymbols = self.globalsymbols[:]
        try:
            T1sub = T1.subs(solvedvarsubs)
            Ree = zeros((3,3))
            for i in range(3):
                for j in range(3):
                    Ree[i,j] = Symbol('new_r%d%d'%(i,j))
                    self.globalsymbols.append((Ree[i,j],T1sub[i,j]))
            othersolvedvars = self.freejointvars if solveRotationFirst else transvars+self.freejointvars
            AllEquations = self.buildEquationsFromRotation(T0links,Ree,rotvars,othersolvedvars)
            currotvars = rotvars[:]
            rottree += self.solveAllEquations(AllEquations,curvars=currotvars,othersolvedvars=othersolvedvars,solsubs=self.freevarsubs[:],endbranchtree=storesolutiontree)

            if len(rottree) == 0:
                raise self.CannotSolveError('could not solve for all rotation variables: %s:%s'%(str(freevar),str(freevalue)))

            if solveRotationFirst:
                solvertree.append(SolverRotation(T1sub, rottree))
            else:
                rottree[:] = [SolverRotation(T1sub, rottree[:])]
            return solvertree
        finally:
            self.globalsymbols = oldglobalsymbols

    def solveFullIK_6DGeneral(self, T0links, T1links, solvejointvars, endbranchtree):
        """Solve 6D equations of a general kinematics structure.
        This method only works if there exists 3 consecutive joints in that do not always intersect!
        """
        rawpolyeqs2 = [None,None]
        coupledsolutions = None
        for solvemethod in [self.solveLiWoernleHiller, self.solveKohliOsvatic, self.solveManochaCanny]:
            if coupledsolutions is not None:
                break
            for j in range(2):
                if rawpolyeqs2[j] is None:
                    if j == 0:
                        # invert, this seems to always give simpler solutions, so prioritize it
                        T0 = self.affineSimplify(self.multiplyMatrix([self.affineInverse(T) for T in T0links][::-1]))
                        T1 = self.affineSimplify(self.multiplyMatrix([self.affineInverse(T) for T in T1links][::-1]))
                    else:
                        T0 = self.affineSimplify(self.multiplyMatrix(T0links))
                        T1 = self.affineSimplify(self.multiplyMatrix(T1links))
                    rawpolyeqs,numminvars = self.buildRaghavanRothEquationsFromMatrix(T0,T1,solvejointvars)
                    if numminvars <= 5 or len(PolyEquations[0][1].symbols) <= 6:
                        rawpolyeqs2[j] = rawpolyeqs
                try:
                    if rawpolyeqs2[j] is not None:
                        coupledsolutions,usedvars = solvemethod(rawpolyeqs2[j])
                        break
                except self.CannotSolveError:
                    continue

        if coupledsolutions is None:
            raise self.CannotSolveError('raghavan roth equations too complex')

        print 'solving coupled variables: ',usedvars
        AllEquations = []
        for i in range(3):
            for j in range(4):
                    AllEquations.append(self.simplifyTransform(T0[i,j]-T1[i,j]))
        self.sortComplexity(AllEquations)
        curvars=solvejointvars[:]
        solsubs = self.freevarsubs[:]
        for var in usedvars:
            curvars.remove(var)
            solsubs += [(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)]
        tree = self.solveAllEquations(AllEquations,curvars=curvars,othersolvedvars = self.freejointvars+usedvars,solsubs = solsubs,endbranchtree=endbranchtree)
        return coupledsolutions+tree

    def buildEquationsFromTwoSides(self,leftside, rightside, usedvars, uselength=True):
        # try to shift all the constants of each Position expression to one side
        for i in range(len(leftside)):
            for j in range(3):
                p = leftside[i][j]
                pee = rightside[i][j]
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
                        leftside[i][j] -= term
                        rightside[i][j] -= term

        AllEquations = []
        for i in range(len(leftside)):
            for j in range(3):
                e = self.trigsimp(leftside[i][j] - rightside[i][j],usedvars)
                if self.codeComplexity(e) < 1500:
                    e = self.simplifyTransform(e)
                if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                    AllEquations.append(e)
            if uselength:
                p2 = leftside[i][0]**2+leftside[i][1]**2+leftside[i][2]**2
                pe2 = rightside[i][0]**2+rightside[i][1]**2+rightside[i][2]**2
                if self.codeComplexity(p2) < 1200 and self.codeComplexity(pe2) < 1200:
                    # sympy's trigsimp/customtrigsimp give up too easily
                    e = self.simplifyTransform(self.trigsimp(p2,usedvars)-self.trigsimp(pe2,usedvars))
                    if self.isExpressionUnique(AllEquations,e) and self.isExpressionUnique(AllEquations,-e):
                        AllEquations.append(e)
                else:
                    print 'length equations too big, skipping...',self.codeComplexity(p2),self.codeComplexity(pe2)
        self.sortComplexity(AllEquations)
        return AllEquations
        
    def buildEquationsFromPositions(self,T1links,T1linksinv,transvars,othersolvedvars,uselength=True):
        Taccum = eye(4)
        numvarsdone = 1
        Positions = []
        Positionsee = []
        for i in range(len(T1links)-1):
            Taccum = T1linksinv[i]*Taccum
            hasvars = [self.has_any_symbols(Taccum,v) for v in transvars]
            if __builtin__.sum(hasvars) == numvarsdone:
                Positions.append(Taccum[0:3,3])
                Positionsee.append(self.multiplyMatrix(T1links[(i+1):])[0:3,3])
                numvarsdone += 1
            if numvarsdone > 2:
                # more than 2 variables is almost always useless
                break
        if len(Positions) == 0:
            Positions.append(zeros((3,1)))
            Positionsee.append(self.multiplyMatrix(T1links)[0:3,3])
        return self.buildEquationsFromTwoSides(Positions,Positionsee,transvars+othersolvedvars,uselength=uselength)

    def buildEquationsFromRotation(self,T0links,Ree,rotvars,othersolvedvars):
        """Ree is a 3x3 matrix
        """
        Raccum = Ree
        numvarsdone = 1
        AllEquations = []
        for i in range(len(T0links)-1):
            Raccum = T0links[i][0:3,0:3].transpose()*Raccum # transpose is the inverse 
            hasvars = [self.has_any_symbols(Raccum,v) for v in rotvars]
            if len(AllEquations) > 0 and __builtin__.sum(hasvars) >= len(rotvars):
                break
            if __builtin__.sum(hasvars) == numvarsdone:
                R = self.multiplyMatrix(T0links[(i+1):])
                for i in range(3):
                    for j in range(3):
                        AllEquations.append(self.trigsimp(Raccum[i,j]-R[i,j],othersolvedvars+rotvars))
                numvarsdone += 1
        self.sortComplexity(AllEquations)
        return AllEquations

    def buildRaghavanRothEquationsFromMatrix(self,T0,T1,solvejointvars):
        """Builds the 14 equations using only 5 unknowns. Method explained by:

        M. Raghavan and B. Roth, "Inverse Kinematics of the General 6R Manipulator and related Linkages",  Journal of Mechanical Design, Volume 115, Issue 3, 1993.

        Basically take the position and one column/row so that the least number of variables are used.
        """
        p0 = T0[0:3,3]
        p1 = T1[0:3,3]
        p=p0-p1
        T = T0-T1
        numminvars = 100000
        for irow in range(3):
            hasvar = [self.has_any_symbols(T[0:3,irow],var) or self.has_any_symbols(p,var) for var in solvejointvars]
            numcurvars = __builtin__.sum(hasvar)
            if numminvars > numcurvars and numcurvars > 0:
                numminvars = numcurvars
                l0 = T0[0:3,irow]
                l1 = T1[0:3,irow]
            hasvar = [self.has_any_symbols(T[irow,0:3],var) or self.has_any_symbols(p,var) for var in solvejointvars]
            numcurvars = __builtin__.sum(hasvar)
            if numminvars > numcurvars and numcurvars > 0:
                numminvars = numcurvars
                l0 = T0[irow,0:3].transpose()
                l1 = T1[irow,0:3].transpose()
        return self.buildRaghavanRothEquations(p0,p1,l0,l1,solvejointvars),numminvars
    
    def buildRaghavanRothEquations(self,p0,p1,l0,l1,solvejointvars):
        eqs = []
        for i in range(3):
            eqs.append([l0[i],l1[i]])
        for i in range(3):
            eqs.append([p0[i],p1[i]])
        l0xp0 = l0.cross(p0)
        l1xp1 = l1.cross(p1)
        for i in range(3):
            eqs.append([l0xp0[i],l1xp1[i]])
        ppl0 = p0.dot(p0)*l0 - 2*l0.dot(p0)*p0
        ppl1 = p1.dot(p1)*l1 - 2*l1.dot(p1)*p1
        for i in range(3):
            eqs.append([ppl0[i],ppl1[i]])
        eqs.append([p0.dot(p0),p1.dot(p1)])
        eqs.append([l0.dot(p0),l1.dot(p1)])
        trigsubs = []
        polysubs = []
        polyvars = []
        for v in solvejointvars:
            polyvars.append(v)
            if self.isHinge(v.name):
                var = self.Variable(v)
                polysubs += [(cos(v),var.cvar),(sin(v),var.svar)]
                polyvars += [var.cvar,var.svar]
                trigsubs.append((var.svar**2,1-var.cvar**2))
        for v in self.freejointvars:
            if self.isHinge(v.name):
                trigsubs.append((sin(v)**2,1-cos(v)**2))
        polysubsinv = [(b,a) for a,b in polysubs]
        usedvars = []
        for j in range(2):
            usedvars.append([var for var in polyvars if any([eq[j].subs(polysubs).has_any_symbols(var) for eq in eqs])])
        polyeqs = []
        for i in range(len(eqs)):
            polyeqs.append([None,None])
        for j in range(2):
            for i in range(len(eqs)):
                poly0 = Poly(eqs[i][j].subs(polysubs),*usedvars[j]).subs(trigsubs)
                poly1 = Poly(poly0.as_basic().expand().subs(trigsubs),*usedvars[j])
                poly2 = Poly(S.Zero,*poly1.symbols)
                for c,m in poly1.iter_terms():
                    cnew = self.simplifyTransform(c)
                    if cnew != S.Zero:
                        poly2 = poly2.add_term(cnew,m)
                polyeqs[i][j] = poly2
        # remove all fractions? having big integers could blow things up...
        return polyeqs

    def reduceBothSides(self,polyeqs):
        """Reduces a set of equations in 5 unknowns to a set of equations with 3 unknowns by solving for one side with respect to another.
        The input is usually the output of buildRaghavanRothEquations.
        """
        reducedeqs = []
        tree = []
        usedvars = [polyeqs[0][0].symbols, polyeqs[0][1].symbols]
        for j in range(2):
            if len(usedvars[j]) <= 4:
                leftsideeqs = [polyeq[j] for polyeq in polyeqs if polyeq[j].degree > 0]
                rightsideeqs = [polyeq[1-j] for polyeq in polyeqs if polyeq[j].degree > 0]
                if all([eq.degree <= 2 for eq in leftsideeqs]):
                    try:
                        reducedeqs2 = self.reduceBothSidesSymbolically(leftsideeqs,rightsideeqs)
                        if len(reducedeqs2) == 0:
                            print 'forcing matrix inverse (might take some time)'
                            reducedeqs2,tree = self.reduceBothSidesInverseMatrix(leftsideeqs,rightsideeqs)
                        if len(reducedeqs2) > 0:
                            # success, add all the reduced equations
                            reducedeqs += [[Poly(eq[0],*usedvars[j]),Poly(eq[1],*usedvars[1-j])] for eq in reducedeqs2] + [[Poly(S.Zero,*polyeq[j].symbols),polyeq[1-j]-polyeq[j].as_basic()] for polyeq in polyeqs if polyeq[j].degree == 0]
                    except self.CannotSolveError:
                        continue
        if len(reducedeqs) > 0:
            # check if any substitutions are needed
#             for eq in reducedeqs:
#                 for j in range(2):
#                     eq[j] = Poly(eq[j].subs(trigsubs).as_basic().expand(),*eq[j].symbols)
            polyeqs = reducedeqs
        return [eq for eq in polyeqs if eq[0] != S.Zero or eq[1] != S.Zero],tree

    def reduceBothSidesInverseMatrix(self,leftsideeqs,rightsideeqs):
        """solve a linear system inside the program since the matrix cannot be reduced so easily
        """
        allmonomsleft = set()
        for peq in leftsideeqs:
            allmonomsleft = allmonomsleft.union(set(peq.monoms))
        allmonomsleft = list(allmonomsleft)
        allmonomsleft.sort()
        if __builtin__.sum(allmonomsleft[0]) == 0:
            allmonomsleft.pop(0)
        if len(leftsideeqs) < len(allmonomsleft):
            raise self.CannotSolveError('left side has too few equations for the number of variables %d<%d'%(len(leftsideeqs),len(allmonomsleft)))
        
        systemcoeffs = []
        for ileft,left in enumerate(leftsideeqs):
            coeffs = [S.Zero]*len(allmonomsleft)
            rank = 0
            for c,m in left.iter_terms():
                if __builtin__.sum(m) > 0:
                    if c != S.Zero:
                        rank += 1
                    coeffs[allmonomsleft.index(m)] = c
            systemcoeffs.append((rank,ileft,coeffs))
        # ideally we want to try all combinations of simple equations first until we arrive to linearly independent ones.
        # However, in practice most of the first equations are linearly dependent and it takes a lot of time to prune all of them,
        # so start at the most complex
        systemcoeffs.sort(lambda x,y: -x[0]+y[0])
        # sort left and right in the same way
        leftsideeqs = [leftsideeqs[ileft] for rank,ileft,coeffs in systemcoeffs]
        rightsideeqs = [rightsideeqs[ileft] for rank,ileft,coeffs in systemcoeffs]

        A = zeros((len(allmonomsleft),len(allmonomsleft)))
        Asymbols = []
        for i in range(A.shape[0]):
            Asymbols.append([Symbol('gconst%d_%d'%(i,j)) for j in range(A.shape[1])])
        solution = None
        for eqindices in combinations(range(len(leftsideeqs)),len(allmonomsleft)):
            for i,index in enumerate(eqindices):
                for k in range(len(allmonomsleft)):
                    A[i,k] = systemcoeffs[index][2][k]
            det = self.det_bareis(A,*self.pvars)
            if det == S.Zero:
                continue
            solution = SolverMatrixInverse(A=A,Asymbols=Asymbols)
            solution.checkforzeros = [self.removecommonexprs(det,onlygcd=False,onlynumbers=True)]
            Aadj=A.adjugate() # too big to be useful for now, but can be used to see if any symbols are always 0
            break
        if solution is None:
            return self.CannotSolveError('failed to find %d linearly independent equations'%len(allmonomsleft))
        reducedeqs = []
        for i in range(len(allmonomsleft)):
            var=S.One
            for k,kpower in enumerate(allmonomsleft[i]):
                if kpower != 0:
                    var *= leftsideeqs[0].symbols[k]**kpower
            pright = S.Zero
            for k in range(len(allmonomsleft)):
                if Aadj[i,k] != S.Zero:
                    pright += Asymbols[i][k] * (rightsideeqs[eqindices[k]].as_basic()-leftsideeqs[eqindices[k]].coeff())
            reducedeqs.append([var,pright.expand()])
        othereqindices = set(range(len(leftsideeqs))).difference(set(eqindices))
        for i in othereqindices:
            # have to multiply just the constant by the determinant
            neweq = rightsideeqs[i].as_basic()
            for c,m in leftsideeqs[i].iter_terms():
                if __builtin__.sum(m) > 0:
                    neweq -= c*reducedeqs[allmonomsleft.index(m)][1]
                else:
                    neweq -= c
            reducedeqs.append([S.Zero,neweq])
        return reducedeqs, [solution]

#                 Adj=M[:,:-1].adjugate()
#                 #D=M[:,:-1].det()
#                 D=M[:,:-1].det()
#                 sols=-Adj*M[:,-1]
#                 solsubs = []
#                 for i,v in enumerate(newunknowns):
#                     newsol=sols[i].subs(localsymbols)
#                     solsubs.append((v,newsol))
#                     reducedeqs.append([v.subs(localsymbols)*D,newsol])
#                 othereqindices = set(range(len(newleftsideeqs))).difference(set(eqindices))
#                 for i in othereqindices:
#                     # have to multiply just the constant by the determinant
#                     newpoly = S.Zero
#                     for c,m in newleftsideeqs[i].iter_terms():
#                         monomindices = [index for index in range(len(newunknowns)) if m[index]>0]
#                         if len(monomindices) == 0:
#                             newpoly += c.subs(localsymbols)*D
#                         else:
#                             assert(len(monomindices)==1)
#                             newpoly += c.subs(localsymbols)*solsubs[monomindices[0]][1]
#                     reducedeqs.append([S.Zero,newpoly])
#                 break

#                 # there are too many symbols, so have to resolve to a little more involved method
#                 P,L,DD,U= M[:,:-1].LUdecompositionFF(*self.pvars)
#                 finalnums = S.One
#                 finaldenoms = S.One
#                 for i in range(len(newunknowns)):
#                     print i
#                     n,d = self.recursiveFraction(L[i,i]*U[i,i]/DD[i,i])
#                     finalnums *= n
#                     finaldenoms *= d
#                     n,d = self.recursiveFraction(DD[i,i])
#                     q,r = div(n,d,*pvars)
#                     DD[i,i] = q
#                     assert(r==S.Zero)
#                 det,r = div(finalnums,finaldenoms,*pvars)
#                 assert(r==S.Zero)
#                 b = -P*M[:,-1]
#                 y = [[b[0],L[0,0]]]
#                 for i in range(1,L.shape[0]):
#                     commondenom=y[0][1]
#                     for j in range(1,i):
#                         commondenom=lcm(commondenom,y[j][1],*pvars)
#                     accum = S.Zero
#                     for j in range(i):
#                         accum += L[i,j]*y[j][0]*(commondenom/y[j][1])
#                     res = (commondenom*b[i]-accum)/(commondenom*L[i,i])
#                     y.append(self.recursiveFraction(res))
# 
#                 ynew = []
#                 for i in range(L.shape[0]):
#                     print i
#                     q,r=div(y[i][0]*DD[i,i],y[i][1],*pvars)
#                     print 'remainder: ',r
#                     ynew.append(q)
#                 
#                 x = [[ynew[-1],U[-1,-1]]]
#                 for i in range(U.shape[0]-2,-1,-1):
#                     commondenom=x[0][1]
#                     for j in range(i+1,U.shape[0]):
#                         commondenom=lcm(commondenom,x[j][1],*pvars)
#                     accum = S.Zero
#                     for j in range(i+1,U.shape[0]):
#                         accum += U[i,j]*x[j][0]*(commondenom/x[j][1])
#                     res = (commondenom*b[i]-accum)/(commondenom*U[i,i])
#                     x.append(self.recursiveFraction(res))
#                 
#                 print 'ignoring num symbols: ',numsymbols
#                 continue

    def reduceBothSidesSymbolically(self,leftsideeqs,rightsideeqs,maxsymbols=10,usesymbols=True):
        """the left and right side of the equations need to have different variables
        """
        assert(len(leftsideeqs)==len(rightsideeqs))
        # first count the number of different monomials, then try to solve for each of them
        symbolgen = cse_main.numbered_symbols('const')
        vargen = cse_main.numbered_symbols('tempvar')
        rightsidedummy = []
        localsymbols = []
        dividesymbols = []
        allmonoms = dict()
        for left,right in izip(leftsideeqs,rightsideeqs):
            if right != S.Zero:
                rightsidedummy.append(symbolgen.next())
                localsymbols.append((rightsidedummy[-1],right.as_basic().expand()))
            else:
                rightsidedummy.append(S.Zero)
            for m in left.iter_monoms():
                if __builtin__.sum(m) > 0 and not m in allmonoms:
                    newvar = vargen.next()
                    localsymbols.append((newvar,Poly(S.Zero,*left.symbols).add_term(S.One,m).as_basic()))
                    allmonoms[m] = newvar

        if len(leftsideeqs) < len(allmonoms):
            raise self.CannotSolveError('left side has too few equations for the number of variables %d<%d'%(len(leftsideeqs),len(allmonoms)))
        
        unknownvars = leftsideeqs[0].symbols
        newleftsideeqs = []
        numsymbolcoeffs = []
        for left,right in izip(leftsideeqs,rightsidedummy):
            left = left - right
            newleft = Poly(S.Zero,*allmonoms.values())
            leftcoeffs = [c for c,m in left.iter_terms() if __builtin__.sum(m) > 0]
            allnumbers = all([c.is_number for c in leftcoeffs])
            if usesymbols and not allnumbers:
                # check if all the equations are within a constant from each other
                # This is neceesary since the current linear system solver cannot handle too many symbols.
                reducedeq0,common0 = self.removecommonexprs(leftcoeffs[0],returncommon=True)
                commonmults = [S.One]
                for c in leftcoeffs[1:]:
                    reducedeq1,common1 = self.removecommonexprs(c,returncommon=True)
                    if self.equal(reducedeq1,reducedeq0):
                        commonmults.append(common1/common0)
                    elif self.equal(reducedeq1,-reducedeq0):
                        commonmults.append(-common1/common0)
                    else:
                        break
                if len(commonmults) == len(leftcoeffs):
                    # divide everything by reducedeq0
                    index = 0
                    for c,m in left.iter_terms():
                        if __builtin__.sum(m) > 0:
                            newleft = newleft + commonmults[index]*allmonoms.get(m)
                            index += 1
                        else:
                            # look in the dividesymbols for something similar
                            gmult = None
                            for gsym,geq in dividesymbols:
                                greducedeq,gcommon = self.removecommonexprs(S.One/geq,returncommon=True)
                                if self.equal(greducedeq,reducedeq0):
                                    gmult = gsym*(gcommon/common0)
                                    break
                                elif self.equal(greducedeq,-reducedeq0):
                                    gmult = gsym*(-gcommon/common0)
                                    break
                            if gmult is None:
                                gmult = symbolgen.next()
                                dividesymbols.append((gmult,S.One/leftcoeffs[0]))
                            newc = (c*gmult).subs(localsymbols).expand()
                            sym = symbolgen.next()
                            localsymbols.append((sym,newc))
                            newleft = newleft + sym
                    numsymbolcoeffs.append(0)
                    newleftsideeqs.append(newleft)
                    continue
            numsymbols = 0
            for c,m in left.iter_terms():
                polyvar = S.One
                if __builtin__.sum(m) > 0:
                    polyvar = allmonoms.get(m)
                    if not c.is_number:
                        numsymbols += 1
                newleft = newleft + c*polyvar
            numsymbolcoeffs.append(numsymbols)
            newleftsideeqs.append(newleft)
        reducedeqs = []
        # order the equations based on the number of terms
        newleftsideeqs.sort(lambda x,y: len(x.monoms) - len(y.monoms))
        newunknowns = newleftsideeqs[0].symbols
        print 'solving for all pairwise variables in',unknownvars,'number of symbol coeffs: ',__builtin__.sum(numsymbolcoeffs)
        systemcoeffs = []
        for eq in newleftsideeqs:
            coeffs = []
            for i,var in enumerate(newunknowns):
                monom = [0]*len(newunknowns)
                monom[i] = 1
                coeffs.append(eq.coeff(*monom))
            monom = [0]*len(newunknowns)
            coeffs.append(-eq.coeff(*monom))
            systemcoeffs.append(coeffs)

        detvars = [s for s,v in localsymbols] + self.pvars
        for eqindices in combinations(range(len(newleftsideeqs)),len(newunknowns)):
            # very quick rejection
            numsymbols = __builtin__.sum([numsymbolcoeffs[i] for i in eqindices])
            if numsymbols > maxsymbols:
                #print 'num symbols ',numsymbols,' is too much'
                continue
            M = Matrix([systemcoeffs[i] for i in eqindices])
            det = self.det_bareis(M[:,:-1], *detvars)
            if det == S.Zero:
                continue
            try:
                eqused = [newleftsideeqs[i] for i in eqindices]
                solution=solve(eqused,newunknowns)
            except IndexError:
                # not enough equations?
                continue
#             try:
#                 print 'nonzero system'
#                 solution=solve_linear_system(M, *newunknowns)
#             except ZeroDivisionError:
#                 continue
            
            if solution is not None and all([self.isValidSolution(value.subs(localsymbols)) for key,value in solution.iteritems()]):
                # substitute 
                solsubs = []
                allvalid = True
                for key,value in solution.iteritems():
                    valuesub = value.subs(localsymbols)
                    solsubs.append((key,valuesub))
                    reducedeqs.append([key.subs(localsymbols),valuesub])
                othereqindices = set(range(len(newleftsideeqs))).difference(set(eqindices))
                for i in othereqindices:
                    reducedeqs.append([S.Zero,(newleftsideeqs[i].subs(solsubs).subs(localsymbols)).as_basic().expand()])
                break
        # remove the dividesymbols from reducedeqs
        for sym,ivalue in dividesymbols:
            value=1/ivalue
            for i in range(len(reducedeqs)):
                eq = reducedeqs[i][1]
                if eq.has_any_symbols(sym):
                    neweq = S.Zero
                    peq = Poly(eq,sym)
                    for c,m in peq.iter_terms():
                        neweq += c*value**(peq.degree - m[0])
                    reducedeqs[i][1] = neweq.expand()
                    reducedeqs[i][0] = (reducedeqs[i][0]*value**peq.degree).expand()
        if len(reducedeqs) > 0:
            print 'finished with %d equations'%len(reducedeqs)
        return reducedeqs

    def solveManochaCanny(self,rawpolyeqs):
        """Solves the IK equations using eigenvalues/eigenvectors of a 12x12 quadratic eigenvalue problem.
        
        D. Manocha and J.F. Canny. "Efficient inverse kinematics for general 6R manipulators", IEEE Transactions on Robotics and Automation, Volume 10, Issue 5, Oct 1994.
        """
        print 'attempting manocha/canny general ik method'
        PolyEquations, raghavansolutiontree = self.reduceBothSides(rawpolyeqs)
        # find all equations with zeros on the left side
        RightEquations = []
        for ipeq,peq in enumerate(PolyEquations):
            if peq[0] == S.Zero:
                if len(raghavansolutiontree) > 0:
                    # give up on optimization
                    RightEquations.append(peq[1])
                else:
                    RightEquations.append(Poly(self.simplifyTransform(peq[1]),*peq[1].symbols))
        
        if len(RightEquations) < 6:
            raise self.CannotSolveError('number of equations %d less than 6'%(len(RightEquations)))

        # sort with respect to the number of monomials
        RightEquations.sort(lambda x, y: len(x.monoms)-len(y.monoms))

        # substitute with dummy=tan(half angle)
        symbols = RightEquations[0].symbols
        symbolsubs = [(symbols[i].subs(self.invsubs),symbols[i]) for i in range(6)]
        print 'solving simultaneously for symbols: ',symbols

        dummys = []
        dummysubs = []
        dummysubs2 = []
        dummyvars = []
        usedvars = []
        for i in range(0,len(symbols),2):
            dummy = Symbol('ht%s'%symbols[i][1:])
            # [0] - cos, [1] - sin
            dummys.append(dummy)
            dummysubs += [(symbols[i],(1-dummy**2)/(1+dummy**2)),(symbols[i+1],2*dummy/(1+dummy**2))]
            var = symbols[i].subs(self.invsubs).args[0]
            dummysubs2.append((var,2*atan(dummy)))
            dummyvars.append((dummy,tan(0.5*var)))
            if not var in usedvars:
                usedvars.append(var)

        newreducedeqs = []
        for peq in RightEquations:
            maxdenom = [0]*(len(symbols)/2)
            for monoms in peq.iter_monoms():
                for i in range(len(maxdenom)):
                    maxdenom[i] = max(maxdenom[i],monoms[2*i]+monoms[2*i+1])
            eqnew = S.Zero
            for c,monoms in peq.iter_terms():
                term = c
                for i in range(len(symbols)):
                    num,denom = fraction(dummysubs[i][1])
                    term *= num**monoms[i]
                # the denoms for 0,1 and 2,3 are the same
                for i in range(len(maxdenom)):
                    denom = fraction(dummysubs[2*i][1])[1]
                    term *= denom**(maxdenom[i]-monoms[2*i]-monoms[2*i+1])
                eqnew += term
            newreducedeqs.append(Poly(eqnew,*dummys))

        # choose which leftvar can determine the singularity of the following equations!
        ileftvar = 0
        leftvar = dummys[ileftvar]
        coupledvars = dummys[:]
        coupledvars.pop(ileftvar)
        getsubs = raghavansolutiontree[0].getsubs if len(raghavansolutiontree) > 0 else None
        exportcoeffeqs,exportmonoms = self.solveDialytically(newreducedeqs,ileftvar,getsubs)
        coupledsolution = SolverCoeffFunction(jointnames=[v.name for v in usedvars],jointeval=[v[1] for v in dummysubs2],jointevalcos=[dummysubs[2*i][1] for i in range(len(usedvars))],jointevalsin=[dummysubs[2*i+1][1] for i in range(len(usedvars))],isHinges=[self.isHinge(v.name) for v in usedvars],exportvar=[v.name for v in dummys],exportcoeffeqs=exportcoeffeqs,exportfnname='solvedialyticpoly12qep',rootmaxdim=16)
        self.usinglapack = True
        return [raghavansolutiontree,coupledsolution],usedvars

    def solveLiWoernleHiller(self,rawpolyeqs):
        """Li-Woernle-Hiller procedure covered in 
        Jorge Angeles, "Fundamentals of Robotics Mechanical Systems, Springer, 2007.
        """
        print 'attempting li/woernle/hiller general ik method'
        if len(rawpolyeqs[0][0].symbols) < len(rawpolyeqs[0][1].symbols):
            for peq in rawpolyeqs:
                peq[0],peq[1] = peq[1],peq[0]

        symbols = list(rawpolyeqs[0][0].symbols)
        othersymbols = list(rawpolyeqs[0][1].symbols)
        symbolsubs = [(symbols[i].subs(self.invsubs),symbols[i]) for i in range(len(symbols))]
        if len(symbols) != 6:
            raise CannotSolveError('Kohli/Osvatic method requires 3 unknown variables')
            
        # choose which leftvar can determine the singularity of the following equations!
        for i in range(0,6,2):
            eqs = [peq for peq in rawpolyeqs if peq[0].has_any_symbols(symbols[i],symbols[i+1])]
            if len(eqs) <= 8:
                break
        if len(eqs) > 8:
            raise self.CannotSolveError('need 8 or less equations of one variable')
        
        cvar = symbols[i]
        svar = symbols[i+1]
        varname = cvar.name[1:]
        tvar = Symbol('ht'+varname)
        symbols.remove(cvar)
        symbols.remove(svar)
        symbols.append(tvar)
        othersymbols.append(tvar)
        
        polyeqs = [[eq[0].as_basic(),eq[1]] for eq in eqs]
        neweqs=[]
        for i in range(0,8,2):
            p0 = Poly(polyeqs[i][0],cvar,svar)
            p1 = Poly(polyeqs[i+1][0],cvar,svar)
            r0 = polyeqs[i][1].as_basic()
            r1 = polyeqs[i+1][1].as_basic()
            if self.equal(p0.coeff(1,0),-p1.coeff(0,1)) and self.equal(p0.coeff(0,1),p1.coeff(1,0)):
                p0,p1 = p1,p0
                r0,r1 = r1,r0
            if self.equal(p0.coeff(1,0),p1.coeff(0,1)) and self.equal(p0.coeff(0,1),-p1.coeff(1,0)):
                # p0+tvar*p1, p1-tvar*p0
                # subs: tvar*svar + cvar = 1, svar-tvar*cvar=tvar
                neweqs.append([Poly(p0.coeff(1,0) + p0.coeff(0,1)*tvar + p0.coeff(0,0) + tvar*p1.coeff(0,0),*symbols), Poly(r0+tvar*r1,*othersymbols)])
                neweqs.append([Poly(p0.coeff(1,0)*tvar - p0.coeff(0,1) - p0.coeff(0,0)*tvar + p1.coeff(0,0),*symbols), Poly(r1-tvar*r0,*othersymbols)])
        if len(neweqs) != 8:
            raise self.CannotSolveError('coefficients of equations need to match! only got %d reduced equations'%len(neweqs))

        for peq in rawpolyeqs:
            if not peq[0].has_any_symbols(cvar,svar):
                neweqs.append([Poly(peq[0],*symbols),Poly(peq[1],*othersymbols)])
                neweqs.append([Poly(peq[0].as_basic()*tvar,*symbols),Poly(peq[1].as_basic()*tvar,*othersymbols)])

        # one side should have only numbers, this makes the following inverse operations trivial
        neweqs_full = []
        reducedeqs = []
        for peq in neweqs:
            peq[1] = peq[1] - tvar*peq[0].coeff(0,0,0,0,1)-peq[0].coeff()
            peq[0] = peq[0] - tvar*peq[0].coeff(0,0,0,0,1)-peq[0].coeff()
            if peq[0] != S.Zero:
                neweqs_full.append(peq)
            else:
                reducedeqs.append(peq[1].as_basic())
        allmonoms = set()
        for peq in neweqs_full:
            allmonoms = allmonoms.union(set(peq[0].monoms))
        allmonoms = list(allmonoms)
        allmonoms.sort()
        if len(allmonoms) > len(neweqs_full):
            raise self.CannotSolveError('new monoms is not %d!=16'%len(allmonoms))
        
        A = zeros((len(neweqs_full),len(allmonoms)))
        B = zeros((len(neweqs_full),1))
        for ipeq,peq in enumerate(neweqs_full):
            for c,m in peq[0].iter_terms():
                A[ipeq,allmonoms.index(m)] = c
            B[ipeq] = peq[1].as_basic()
        AU = zeros((len(allmonoms),len(allmonoms)))
        AL = zeros((A.shape[0]-len(allmonoms),len(allmonoms)))
        BU = zeros((len(allmonoms),1))
        BL = zeros((A.shape[0]-len(allmonoms),1))
        AUinv = None
        for rows in combinations(range(A.shape[0]),A.shape[1]):
            for i,row in enumerate(rows):
                AU[i,:] = A[row,:]
            if AU.det() != S.Zero:
                AUinv = AU.inv()
                break
        if AUinv is None:
            raise self.CannotSolveError('could not find the inverse matrix')
        
        otherrows = range(A.shape[0])
        for i,row in enumerate(rows):
            BU[i] = B[row]
            otherrows.remove(row)
        for i,row in enumerate(otherrows):
            BL[i] = B[row]
            AL[i,:] = A[row,:]
        C = AL*(AUinv*BU)-BL
        for c in C:
            reducedeqs.append(c)
        # is now a (len(neweqs)-len(allmonoms))x1 matrix, usually this is 4x1
        htvars = []
        htvarsubs = []
        htvarsubs2 = []
        usedvars = []
        for nextindex in [0,2]:
            name = othersymbols[nextindex].name[1:]
            htvar = Symbol('ht%s'%name)
            htvarsubs += [(othersymbols[nextindex],(1-htvar**2)/(1+htvar**2)),(othersymbols[nextindex+1],2*htvar/(1+htvar**2))]
            htvars.append(htvar)
            htvarsubs2.append((Symbol(name),2*atan(htvar)))
            usedvars.append(Symbol(name))
        htvarsubs += [(cvar,(1-tvar**2)/(1+tvar**2)),(svar,2*tvar/(1+tvar**2))]
        htvars.append(tvar)
        htvarsubs2.append((Symbol(varname),2*atan(tvar)))
        usedvars.append(Symbol(varname))
        newreducedeqs = []
        for eq in reducedeqs:
            peq = Poly(eq,*othersymbols)
            maxdenom = [0,0]
            for monoms in peq.iter_monoms():
                for i in range(len(maxdenom)):
                    maxdenom[i] = max(maxdenom[i],monoms[2*i]+monoms[2*i+1])
            eqnew = S.Zero
            for c,monoms in peq.iter_terms():
                term = c
                for i in range(4):
                    num,denom = fraction(htvarsubs[i][1])
                    term *= num**monoms[i]
                term *= tvar**monoms[4]
                # the denoms for 0,1 and 2,3 are the same
                for i in range(len(maxdenom)):
                    denom = fraction(htvarsubs[2*i][1])[1]
                    term *= denom**(maxdenom[i]-monoms[2*i]-monoms[2*i+1])
                eqnew += term
            newreducedeqs.append(Poly(eqnew,htvars[0],htvars[1],tvar))

        exportcoeffeqs,exportmonoms = self.solveDialytically(newreducedeqs,0)
        coupledsolution = SolverCoeffFunction(jointnames=[v.name for v in usedvars],jointeval=[v[1] for v in htvarsubs2],jointevalcos=[htvarsubs[2*i][1] for i in range(len(htvars))],jointevalsin=[htvarsubs[2*i+1][1] for i in range(len(htvars))],isHinges=[self.isHinge(v.name) for v in usedvars],exportvar=[v.name for v in htvars],exportcoeffeqs=exportcoeffeqs,exportfnname='solvedialyticpoly8qep',rootmaxdim=16)
        self.usinglapack = True
        return [coupledsolution],usedvars

    def solveKohliOsvatic(self,rawpolyeqs):
        """Find a 16x16 matrix where the entries are linear with respect to the tan half-angle of one of the variables.
        Takes in the 14 raghavan/roth equations.
        
        D. Kohli and M. Osvatic, "Inverse Kinematics of General 6R and 5R,P Serial Manipulators", Journal of Mechanical Design, Volume 115, Issue 4, Dec 1993.
        """
        print 'attempting kohli/osvatic general ik method'
        if len(rawpolyeqs[0][0].symbols) < len(rawpolyeqs[0][1].symbols):
            for peq in rawpolyeqs:
                peq[0],peq[1] = peq[1],peq[0]

        symbols = list(rawpolyeqs[0][0].symbols)
        othersymbols = list(rawpolyeqs[0][1].symbols)
        symbolsubs = [(symbols[i].subs(self.invsubs),symbols[i]) for i in range(len(symbols))]
        if len(symbols) != 6:
            raise CannotSolveError('Kohli/Osvatic method requires 3 unknown variables')
            
        # choose which leftvar can determine the singularity of the following equations!
        for i in range(0,6,2):
            eqs = [peq for peq in rawpolyeqs if peq[0].has_any_symbols(symbols[i],symbols[i+1])]
            if len(eqs) <= 8:
                break
        if len(eqs) > 8:
            raise self.CannotSolveError('need 8 or less equations of one variable')

        cvar = symbols[i]
        svar = symbols[i+1]
        tvar = Symbol('t'+cvar.name[1:])
        symbols.remove(cvar)
        symbols.remove(svar)
        othereqs = [peq for peq in rawpolyeqs if not peq[0].has_any_symbols(cvar,svar)]

        polyeqs = [[eq[0].as_basic(),eq[1]] for eq in eqs]
        neweqs=[]
        for i in range(0,8,2):
            p0 = Poly(polyeqs[i][0],cvar,svar)
            p1 = Poly(polyeqs[i+1][0],cvar,svar)
            r0 = polyeqs[i][1].as_basic()
            r1 = polyeqs[i+1][1].as_basic()
            if self.equal(p0.coeff(1,0),-p1.coeff(0,1)) and self.equal(p0.coeff(0,1),p1.coeff(1,0)):
                p0,p1 = p1,p0
                r0,r1 = r1,r0
            if self.equal(p0.coeff(1,0),p1.coeff(0,1)) and self.equal(p0.coeff(0,1),-p1.coeff(1,0)):
                # p0+tvar*p1, p1-tvar*p0
                # subs: tvar*svar + cvar = 1, svar-tvar*cvar=tvar
                neweqs.append([Poly(p0.coeff(1,0) + p0.coeff(0,1)*tvar + p0.coeff(0,0) + tvar*p1.coeff(0,0),*symbols), Poly(r0+tvar*r1,*othersymbols)])
                neweqs.append([Poly(p0.coeff(1,0)*tvar - p0.coeff(0,1) - p0.coeff(0,0)*tvar + p1.coeff(0,0),*symbols), Poly(r1-tvar*r0,*othersymbols)])
        if len(neweqs) != 8:
            raise self.CannotSolveError('coefficients of equations need to match! only got %d reduced equations'%len(neweqs))

        # solve the othereqs for symbols without the standalone symbols[2] and symbols[3]
        for jother in range(len(othersymbols)/2):
            if jother == 0:
                cosmonom = (1,0,0,0)
                sinmonom = (0,1,0,0)
            else:
                cosmonom = (0,0,1,0)
                sinmonom = (0,0,0,1)
            leftsideeqs = []
            rightsideeqs = []
            for eq0,eq1 in othereqs:
                leftsideeq = Poly(eq1,*othersymbols)
                rightsideeq = Poly(eq0,*(symbols+othersymbols[2*jother:(2*jother+2)]))
                coscoeff = leftsideeq.coeff(*cosmonom)
                if coscoeff != S.Zero:
                    rightsideeq = rightsideeq.sub_term(coscoeff,(0,0,0,0,1,0))
                    leftsideeq = leftsideeq.sub_term(coscoeff,cosmonom)
                sincoeff = leftsideeq.coeff(*sinmonom)
                if sincoeff != S.Zero:
                    rightsideeq = rightsideeq.sub_term(sincoeff,(0,0,0,0,0,1))
                    leftsideeq = leftsideeq.sub_term(sincoeff,sinmonom)
                const = leftsideeq.coeff(0,0,0,0)
                if const != S.Zero:
                    rightsideeq = rightsideeq.sub_term(const,(0,0,0,0,0,0))
                    leftsideeq = leftsideeq.sub_term(const,(0,0,0,0))
                rightsideeqs.append(rightsideeq)
                leftsideeqs.append(leftsideeq)
            # number of symbols for hiro robot is 16
            if len(othersymbols) > 2:
                reducedeqs = self.reduceBothSidesSymbolically(leftsideeqs,rightsideeqs,usesymbols=False,maxsymbols=18)
                for peq in reducedeqs:
                    peq[0] = Poly(peq[0],*othersymbols)
            else:
                reducedeqs = [[left,right] for left,right in izip(leftsideeqs,rightsideeqs)]
            if len(reducedeqs) > 0:
                break
            
        if len(reducedeqs) == 0:
            raise self.CannotSolveError('KohliOsvatic method: could not reduce the equations')

        finaleqs = []
        finaleqsymbols = symbols+othersymbols[2*jother:(2+2*jother)]
        print 'build final equations for symbols: ',finaleqsymbols
        for eq0,eq1 in neweqs:
            commondenom = Poly(S.One,*self.pvars)
            for c,m in eq1.iter_terms():
                foundreq = [req[1] for req in reducedeqs if req[0].monoms[0] == m]
                if len(foundreq) > 0:
                    n,d = fraction(foundreq[0])
                    commondenom = Poly(lcm(commondenom,d),*self.pvars)
            commondenom = self.removecommonexprs(commondenom.as_basic(),onlygcd=True,onlynumbers=True)
            finaleq = eq0.as_basic()*commondenom
            for c,m in eq1.iter_terms():
                foundreq = [req[1] for req in reducedeqs if req[0].monoms[0] == m]
                if len(foundreq) > 0:
                    finaleq = finaleq - c*simplify(foundreq[0]*commondenom)
                else:
                    finaleq = finaleq - Poly(S.Zero,*eq1.symbols).add_term(c*commondenom,m).as_basic()
            finaleqs.append(Poly(finaleq.expand(),*finaleqsymbols))

        # finally do the half angle substitution with symbols
        # set:
        # j=othersymbols[2]*(1+dummys[0]**2)*(1+dummys[1]**2)
        # k=othersymbols[3]*(1+dummys[0]**2)*(1+dummys[1]**2)
        dummys = []
        dummysubs = []
        dummysubs2 = []
        dummyvars = []
        usedvars = []

        dummys.append(tvar)
        dummyvars.append((tvar,tan(0.5*Symbol(tvar.name[1:]))))
        usedvars.append(Symbol(cvar.name[1:]))
        dummysubs2.append((usedvars[-1],2*atan(tvar)))
        dummysubs += [(cvar,(1-tvar**2)/(1+tvar**2)),(svar,2*tvar/(1+tvar**2))]

        for i in range(0,len(symbols),2):
            dummy = Symbol('ht%s'%symbols[i].name[1:])
            # [0] - cos, [1] - sin
            dummys.append(dummy)
            dummysubs += [(symbols[i],(1-dummy**2)/(1+dummy**2)),(symbols[i+1],2*dummy/(1+dummy**2))]
            var = symbols[i].subs(self.invsubs).args[0]
            dummyvars.append((dummy,tan(0.5*var)))
            dummysubs2.append((var,2*atan(dummy)))
            if not var in usedvars:
                usedvars.append(var)
        commonmult = (1+dummys[1]**2)*(1+dummys[2]**2)

        usedvars.append(Symbol(othersymbols[2*jother].name[1:]))
        dummyj = Symbol('dummyj')
        dummyk = Symbol('dummyk')
        dummyjk = Symbol('dummyjk')

        dummys.append(dummyj)
        dummyvars.append((dummyj,othersymbols[2*jother]*(1+dummyvars[1][1]**2)*(1+dummyvars[2][1]**2)))
        dummysubs.append((othersymbols[2*jother],cos(dummyjk)))        
        dummys.append(dummyk)
        dummyvars.append((dummyk,othersymbols[1+2*jother]*(1+dummyvars[1][1]**2)*(1+dummyvars[2][1]**2)))
        dummysubs.append((othersymbols[1+2*jother],sin(dummyjk)))
        dummysubs2.append((usedvars[-1],dummyjk))

        ileftvar=0
        newreducedeqs = []
        for peq in finaleqs:
            eqnew = S.Zero
            for c,monoms in peq.iter_terms():
                term = S.One
                for i in range(4):
                    term *= dummysubs[i+2][1]**monoms[i]
                if monoms[4] == 1:
                    eqnew += c * dummyj
                elif monoms[5] == 1:
                    eqnew += c * dummyk
                else:
                    eqnew += c*simplify(term*commonmult)
            newreducedeqs.append(Poly(eqnew,*dummys))

        exportcoeffeqs,exportmonoms = self.solveDialytically(newreducedeqs,ileftvar)
        coupledsolution = SolverCoeffFunction(jointnames=[v.name for v in usedvars],jointeval=[v[1] for v in dummysubs2],jointevalcos=[dummysubs[2*i][1] for i in range(len(usedvars))],jointevalsin=[dummysubs[2*i+1][1] for i in range(len(usedvars))],isHinges=[self.isHinge(v.name) for v in usedvars],exportvar=dummys[0:3]+[dummyjk],exportcoeffeqs=exportcoeffeqs,exportfnname='solvedialyticpoly16lep',rootmaxdim=16)
        self.usinglapack = True
        return [coupledsolution],usedvars

    def solveDialytically(self,newreducedeqs,ileftvar,getsubs=None):
        """ Return the coefficients to solve equations dialytically (Salmon 1885) leaving out variable index ileftvar.

        Extract the coefficients of 1, leftvar**1, leftvar**2, ... of every equation
        every len(newreducedeqs)*len(monoms) coefficients specify one degree of all the equations (order of monoms is specified in exportmonomorder
        there should be len(newreducedeqs)*len(monoms)*maxdegree coefficients

        Method also checks if the equations are linearly dependent
        """
        allmonoms = set()
        origmonoms = set()
        maxdegree = 0
        for peq in newreducedeqs:
            for m in peq.iter_monoms():
                mlist = list(m)
                maxdegree=max(maxdegree,mlist.pop(ileftvar))
                allmonoms.add(tuple(mlist))
                origmonoms.add(tuple(mlist))
                mlist[0] += 1
                allmonoms.add(tuple(mlist))
        allmonoms = list(allmonoms)
        allmonoms.sort()
        origmonoms = list(origmonoms)
        origmonoms.sort()
        if len(allmonoms)>2*len(newreducedeqs):
            raise self.CannotSolveError('solveDialytically: more unknowns than equations')
        
        Mall = [zeros((2*len(newreducedeqs),len(allmonoms))) for i in range(maxdegree+1)]
        exportcoeffeqs = [S.Zero]*(len(newreducedeqs)*len(origmonoms)*(maxdegree+1))
        for ipeq,peq in enumerate(newreducedeqs):
            for c,m in peq.iter_terms():
                mlist = list(m)
                degree=mlist.pop(ileftvar)
                exportindex = degree*len(origmonoms)*len(newreducedeqs) + len(origmonoms)*ipeq+origmonoms.index(tuple(mlist))
                exportcoeffeqs[exportindex] = c
                Mall[degree][len(newreducedeqs)+ipeq,allmonoms.index(tuple(mlist))] = c
                mlist[0] += 1
                Mall[degree][ipeq,allmonoms.index(tuple(mlist))] = c
        # have to check that the determinant is not zero for several values of ileftvar! It is very common that
        # some equations are linearly dependent and not solvable through this method.
        if self.testconsistentvalues is not None:
            linearlyindependent = False
            for subs in self.testconsistentvalues:
                if getsubs is not None:
                    # have to explicitly evaluate since testsubs can be very complex
                    subsvals = [(s,v.evalf()) for s,v in subs]
                    subs = subsvals+getsubs(subsvals)
                A = Mall[maxdegree].subs(subs).evalf()
                eigenvals = numpy.linalg.eigvals(numpy.array(numpy.array(A),numpy.float64))
                if all([abs(f) > (10**-(self.precision-3)) for f in eigenvals]):
                    linearlyindependent = True
                    break
            if not linearlyindependent:
                raise self.CannotSolveError('equations are not linearly independent')
            
        return exportcoeffeqs,origmonoms

    def simplifyTransform(self,eq,othervars=None):
        """Attemps to simplify an equation given that variables from a rotation matrix have been used. There are 12 constraints that are tested:
        - lengths of rows and colums are 1
        - dot products of combinations of rows/columns are 0
        - cross products of combinations of rows/columns yield the left over row/column
        """
        if othervars is not None:
            peq = Poly(eq,*othervars)
            peqnew = Poly(S.Zero,*othervars)
            for c,m in peq.iter_terms():
                cnew = self.simplifyTransform(c)
                if cnew:
                    peqnew = peqnew.add_term(cnew,m)
            return peqnew.as_basic()
        
        # first simplify just rotations (since they don't add any new variables)
        allsymbols = list(self.Tee[0:3,0:3])
        # check normals
        normgroups = []
        for i in range(3):
            normgroups.append([self.Tee[i,0],self.Tee[i,1],self.Tee[i,2],S.One])
            normgroups.append([self.Tee[0,i],self.Tee[1,i],self.Tee[2,i],S.One])
        def _simplifynorm(eq):
            neweq = None
            for group in normgroups:
                p = Poly(eq,group[0],group[1],group[2])
                for m0,m1 in combinations(p.monoms,2):
                    if self.equal(p.coeff(*m0),p.coeff(*m1)):
                        for i,j,k in [(0,1,2),(0,2,1),(1,2,0)]:
                            if ((m0[i] == 2 and m1[j] == 2) or (m0[j]==2 and m1[i]==2)) and m0[k]==m1[k]:
                                # there is a bug in sympy polynomial adding here! (0.6.7)
                                p = p + p.coeff(*m0)*(group[3]-p.symbols[0]**2-p.symbols[1]**2-p.symbols[2]**2)*p.symbols[k]**(m0[k])
                                neweq = p.as_basic()
                                eq = neweq
                                break
            return neweq

        # check for dot products between rows and columns
        dotgroups = []
        for i,j in combinations(range(3),2):
            # dot product of rotations is always 0
            dotgroups.append([[i,j],[i+3,j+3],[i+6,j+6],S.Zero])
            dotgroups.append([[3*i,3*j],[3*i+1,3*j+1],[3*i+2,3*j+2],S.Zero])
        def _simplifydot(eq):
            p = Poly(eq,*allsymbols)
            changed = False
            for dg in dotgroups:
                for i,j,k in [(0,1,2),(0,2,1),(1,2,0)]:
                    for comb in combinations(p.iter_terms(),2):
                        if self.equal(comb[0][0],comb[1][0]):
                            for (c0,m0),(c1,m1) in [comb,comb[::-1]]:
                                if m0[dg[i][0]] == 1 and m0[dg[i][1]] == 1 and m1[dg[j][0]] == 1 and m1[dg[j][1]] == 1:
                                    # make sure the left over terms are also the same
                                    m0l = list(m0); m0l[dg[i][0]] = 0; m0l[dg[i][1]] = 0
                                    m1l = list(m1); m1l[dg[j][0]] = 0; m1l[dg[j][1]] = 0
                                    if tuple(m0l) == tuple(m1l):
                                        m2 = list(m0l); m2[dg[k][0]] += 1; m2[dg[k][1]] += 1
                                        # there is a bug in sympy polynomial adding here! (0.6.7)
                                        p = p.sub_term(c0,m0).sub_term(c1,m1).sub_term(c0,tuple(m2))
                                        if dg[3] != S.Zero:
                                            p = p.add_term(c0*dg[3],tuple(m0l))
                                        changed = True
                                        break
                            if changed:
                                break
            return p.as_basic() if changed else None

        # add cross products
        crossgroups = []
        for i,j,k in [(0,1,2),(0,2,1),(1,2,0)]:
            # column
            crossgroups.append([[i+3,j+6],[i+6,j+3],k])
            crossgroups.append([[i+6,j],[i,j+6],k+3])
            crossgroups.append([[i,j+3],[i+3,j],k+6])
            # row
            crossgroups.append([[3*i+1,3*j+2],[3*i+2,3*j+1],3*k])
            crossgroups.append([[3*i+2,3*j],[3*i,3*j+2],3*k+1])
            crossgroups.append([[3*i,3*j+1],[3*i+1,3*j],3*k+2])
            # swap if sign is negative: if j!=1+i
            if j!=1+i:
                for crossgroup in crossgroups[-6:]:
                    crossgroup[0],crossgroup[1] = crossgroup[1],crossgroup[0]
        def _simplifycross(eq):
            # check cross products
            changed = False
            p = Poly(eq,*allsymbols)
            pzero = Poly(S.Zero,*allsymbols)
            for cg in crossgroups:
                for comb in combinations(p.iter_terms(),2):
                    if self.equal(comb[0][0],-comb[1][0]):
                        for (c0,m0),(c1,m1) in [comb,comb[::-1]]:
                            if m0[cg[0][0]] == 1 and m0[cg[0][1]] == 1 and m1[cg[1][0]] == 1 and m1[cg[1][1]] == 1:
                                # make sure the left over terms are also the same
                                m0l = list(m0); m0l[cg[0][0]] = 0; m0l[cg[0][1]] = 0
                                m1l = list(m1); m1l[cg[1][0]] = 0; m1l[cg[1][1]] = 0
                                if tuple(m0l) == tuple(m1l):
                                    m2 = m0l; m2[cg[2]] += 1
                                    # there is a bug in sympy polynomial caching here! (0.6.7)
                                    #p = p.sub_term(c0,m0).sub_term(c1,m1).add_term(c0,tuple(m2))
                                    p = Poly(p.as_basic() - pzero.add_term(c0,m0).as_basic() - pzero.add_term(c1,m1).as_basic() + pzero.add_term(c0,tuple(m2)).as_basic(),*allsymbols)
                                    changed = True
                                    break
                        if changed:
                            break
            return p.as_basic() if changed else None

        fns = [_simplifynorm,_simplifydot,_simplifycross]
        changed = True
        while changed and eq.has_any_symbols(*allsymbols):
            changed = False
            for fn in fns:
                neweq = fn(eq)
                if neweq is not None:
                    eq = neweq
                    changed = True

        # add positions
        ip = 9
        inp = 12
        ipp = 15
        irxp = 16
        allsymbols += list(self.Tee[0:3,3])+self.npxyz+[self.pp]+self.rxp[0]+self.rxp[1]+self.rxp[2]
        normgroups.append([self.Tee[0,3],self.Tee[1,3],self.Tee[2,3],self.pp])
        for i in range(3):
            dotgroups.append([[i,ip],[i+3,ip+1],[i+6,ip+2],self.npxyz[i]])
            dotgroups.append([[3*i+0,inp],[3*i+1,inp+1],[3*i+2,inp+2],self.Tee[i,3]])
            # column i cross position
            crossgroups.append([[i+3,ip+2],[i+6,ip+1],irxp+3*i+0])
            crossgroups.append([[i+6,ip+0],[i,ip+2],irxp+3*i+1])
            crossgroups.append([[i,ip+1],[i+3,ip+0],irxp+3*i+2])
        changed = True
        while changed and eq.has_any_symbols(*allsymbols):
            changed = False
            for fn in fns:
                neweq = fn(eq)
                if neweq is not None:
                    eq = neweq
                    changed = True

        return eq

    def isExpressionUnique(self, exprs, expr):
        for exprtest in exprs:
            if self.equal(expr,exprtest):
                return False
        return True

    def getCommonExpression(self, exprs, expr):
        for i,exprtest in enumerate(exprs):
            if self.equal(expr,exprtest):
                return i
        return None

    def verifyAllEquations(self,AllEquations,unsolvedvars, solsubs, tree=None):
        extrazerochecks=[]
        for i in range(len(AllEquations)):
            expr = AllEquations[i]
            if not self.isValidSolution(expr):
                raise self.CannotSolveError('verifyAllEquations: equation is not valid: %s'%(str(expr)))
            
            if not expr.has_any_symbols(*unsolvedvars) and (self.isExpressionUnique(extrazerochecks,expr) or self.isExpressionUnique(extrazerochecks,-expr)):
                extrazerochecks.append(self.removecommonexprs(expr.subs(solsubs).evalf(),onlygcd=False,onlynumbers=True))
        if len(extrazerochecks) > 0:
            return [SolverCheckZeros(None,extrazerochecks,tree,[SolverBreak()],anycondition=False)]
        return tree

    def solveAllEquations(self,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree,currentcases=None):
        if len(curvars) == 0:
            return endbranchtree
        print othersolvedvars,curvars
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
                    rawsolutions=self.solveSingleVariable(raweqns,curvar,othersolvedvars)
                    for solution in rawsolutions:
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
                rawsolutions=self.solvePairVariables(raweqns,var0,var1,othersolvedvars)
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
                
        # test with higher degrees, necessary?
        for curvar in curvars:
            othervars = [var for var in curvars if var != curvar]
            raweqns = []
            for e in AllEquations:
                if (len(othervars) == 0 or not e.has_any_symbols(*othervars)) and e.has_any_symbols(curvar):
                    eq = e.subs(self.freevarsubs+solsubs)
                    if self.isExpressionUnique(raweqns,eq) and self.isExpressionUnique(raweqns,-eq):
                        raweqns.append(eq)
            for raweqn in raweqns:
                try:
                    print 'testing with higher degrees'
                    solution=self.solveHighDegreeEquationsHalfAngle([raweqn],self.Variable(curvar))
                    self.solutionComplexity(solution,othersolvedvars,curvars)
                    solutions.append((solution,curvar))
                except self.CannotSolveError:
                    pass

        if len(solutions) > 0:
            return self.addSolution(solutions,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree,currentcases=currentcases)

        # solve with all 3 variables together
        
        raise self.CannotSolveError('failed to find a variable to solve')

    def addSolution(self,solutions,AllEquations,curvars,othersolvedvars,solsubs,endbranchtree,currentcases=None):
        """Take the least complex solution of a set of solutions and resume solving
        """
        solutions = [s for s in solutions if s[0].score < oo and s[0].checkValidSolution()] # remove infinite scores
        if len(solutions) == 0:
            raise self.CannotSolveError('no valid solutions')
        
        solutions.sort(lambda x, y: x[0].score-y[0].score)
        hasonesolution = False
        for solution in solutions:
            checkforzeros = solution[0].checkforzeros
            hasonesolution |= solution[0].numsolutions() == 1
            if len(checkforzeros) == 0 and solution[0].numsolutions() == 1:
                # did find a good solution, so take it. Make sure to check any zero branches
                var = solution[1]
                newvars=curvars[:]
                newvars.remove(var)
                return [solution[0].subs(solsubs)]+self.solveAllEquations(AllEquations,curvars=newvars,othersolvedvars=othersolvedvars+[var],solsubs=solsubs+self.Variable(var).subs,endbranchtree=endbranchtree,currentcases=currentcases)
        if not hasonesolution:
            # check again except without the number of solutions requirement
            for solution in solutions:
                checkforzeros = solution[0].checkforzeros
                if len(checkforzeros) == 0:
                    # did find a good solution, so take it. Make sure to check any zero branches
                    var = solution[1]
                    newvars=curvars[:]
                    newvars.remove(var)
                    return [solution[0].subs(solsubs)]+self.solveAllEquations(AllEquations,curvars=newvars,othersolvedvars=othersolvedvars+[var],solsubs=solsubs+self.Variable(var).subs,endbranchtree=endbranchtree,currentcases=currentcases)

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
                        # don't need more than three alternatives (used to be two, but then lookat barrettwam4 proved that wrong)
                        break
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
        # iterate in reverse order and put the most recently processed solution at the front.
        # There is a problem with this algorithm transferring the degenerate cases correctly.
        # Although the zeros of the first equation are checked, they are not added as conditions
        # to the later equations, so that the later equations will also use variables as unknowns (even though they are determined to be specific constants). This is most apparent in rotations.
        for solution,var in usedsolutions[::-1]:
            # there are divide by zeros, so check if they can be explicitly solved for joint variables
            checkforzeros = []
            for checkzero in solution.checkforzeros:
                if checkzero.has_any_symbols(*allvars):
                    print 'ignoring special check for zero'
                    continue
                # fractions could get big, so evaluate directly
                checkforzeros.append(self.removecommonexprs(checkzero.evalf(),onlygcd=False,onlynumbers=True))
                for othervar in othersolvedvars:
                    sothervar = self.Variable(othervar).svar
                    cothervar = self.Variable(othervar).cvar
                    if checkzero.has_any_symbols(othervar,sothervar,cothervar):
                        # the easiest thing to check first is if the equation evaluates to zero on boundaries 0,pi/2,pi,-pi/2
                        s = SolverSolution(othervar.name,jointeval=[],isHinge=self.isHinge(othervar.name))
                        for value in [S.Zero,pi/2,pi,-pi/2]:
                            try:
                                checkzerosub=checkzero.subs([(othervar,value),(sothervar,sin(value).evalf()),(cothervar,cos(value).evalf())])
                                if self.isValidSolution(checkzerosub) and checkzerosub.evalf() == S.Zero:
                                    if s.jointeval is None:
                                        s.jointeval = []
                                    s.jointeval.append(S.One*value)
                            except AssertionError,e:
                                print 'othervar %s=%f'%(str(othervar),value),e
                        if s.jointeval is not None and len(s.jointeval) > 0:
                            ss = [s]
                        else:
                            ss = []
                        try:
                            ss += self.solveSingleVariable([checkzero.subs([(sothervar,sin(othervar)),(cothervar,cos(othervar))])],othervar,othersolvedvars)
                        except self.CannotSolveError,e:
                            # this is actually a little tricky, sometimes really good solutions can have a divide that looks like:
                            # ((0.405 + 0.331*cj2)**2 + 0.109561*sj2**2 (manusarm_left)
                            # This will never be 0, but the solution cannot be solved. Instead of rejecting, add a condition to check if checkzero itself is 0 or not
                            pass
                        
                        for s in ss:
                            # can actually simplify Positions and possibly get a new solution!                            
                            if s.jointeval is not None:
                                for eq in s.jointeval:
                                    if eq.is_number:
                                        cond=othervar-eq.evalf()
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.isHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(sothervar,sin(eq).evalf()),(sin(othervar),sin(eq).evalf()),(cothervar,cos(eq).evalf()),(cos(othervar),cos(eq).evalf()),(othervar,eq)]])
                            elif s.jointevalsin is not None:
                                for eq in s.jointevalsin:
                                    if eq.is_number:
                                        cond=othervar-asin(eq).evalf()
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.isHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(sothervar,eq),(sin(othervar),eq),(cothervar,sqrt(1-eq*eq).evalf()),(cos(othervar),sqrt(1-eq*eq).evalf()),(othervar,asin(eq).evalf())]])
                                        cond=othervar-(pi-asin(eq).evalf())
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.isHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(sothervar,eq),(sin(othervar),eq),(cothervar,-sqrt(1-eq*eq).evalf()),(cos(othervar),-sqrt(1-eq*eq).evalf()),(othervar,(pi-asin(eq)).evalf())]])
                            elif s.jointevalcos is not None:
                                for eq in s.jointevalcos:
                                    if eq.is_number:
                                        cond=othervar-acos(eq).evalf()
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.isHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(sothervar,sqrt(1-eq*eq).evalf()),(sin(othervar),sqrt(1-eq*eq).evalf()),(cothervar,eq),(cos(othervar),eq),(othervar,acos(eq).evalf())]])
                                        cond=othervar+acos(eq).evalf()
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            if self.isHinge(othervar.name):
                                                evalcond=fmod(cond+pi,2*pi)-pi
                                            else:
                                                evalcond=cond
                                            eqs.append([cond,evalcond,[(sothervar,-sqrt(1-eq*eq).evalf()),(sin(othervar),-sqrt(1-eq*eq).evalf()),(cothervar,eq),(cos(othervar),eq),(othervar,-acos(eq).evalf())]])
                
            if not var in nextsolutions:
                newvars=curvars[:]
                newvars.remove(var)
                olddegeneratecases = self.degeneratecases
                self.degeneratecases = olddegeneratecases.clone()
                nextsolutions[var] = self.solveAllEquations(AllEquations,curvars=newvars,othersolvedvars=othersolvedvars+[var],solsubs=solsubs+self.Variable(var).subs,endbranchtree=endbranchtree,currentcases=currentcases)
                self.degeneratecases = olddegeneratecases
            if len(checkforzeros) > 0:
                hascheckzeros = True
                prevbranch=[SolverCheckZeros(jointname=var.name,jointcheckeqs=checkforzeros,nonzerobranch=[solution]+nextsolutions[var],zerobranch=prevbranch,anycondition=True,thresh=solution.thresh)]
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
                for preal in [Symbol('px'),Symbol('py'),Symbol('pz')]:
                    if checkzero.has_any_symbols(preal):
                        # first check if the position alone can yield a zero
                        eq = checkzero.subs([(preal,S.Zero)]).evalf()
                        if eq == S.Zero:
                            cond = abs(preal)
                            evalcond = abs(preal)
                            if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                eqs.append([cond,evalcond,[(preal,S.Zero)]])
                                print '%s=0 in %s'%(str(preal),str(checkzero))
                            continue
                        for othervar in othersolvedvars:
                            if not self.isHinge(othervar.name):
                                continue
                            sothervar = Symbol('s%s'%othervar.name)
                            cothervar = Symbol('c%s'%othervar.name)
                            if checkzero.has_any_symbols(othervar,sothervar,cothervar):
                                for value in [S.Zero,pi/2,pi,-pi/2]:
                                    eq = checkzero.subs([(othervar,value),(sothervar,sin(value).evalf()),(cothervar,cos(value).evalf()),(preal,S.Zero)]).evalf()
                                    if eq == S.Zero:
                                        cond = abs(othervar-value)+abs(preal)
                                        evalcond = abs(fmod(othervar-value+pi,2*pi)-pi)+abs(preal)
                                        if self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],-cond) and self.isExpressionUnique(handledconds+[tempeq[0] for tempeq in eqs],cond):
                                            eqs.append([cond,evalcond,[(sothervar,sin(value).evalf()),(sin(othervar),sin(value).evalf()),(cothervar,cos(value).evalf()),(cos(othervar),cos(value).evalf()),(preal,S.Zero),(othervar,value)]])
                                            print '%s=%s,%s=0 in %s'%(str(othervar),str(value),str(preal),str(checkzero))

        # test the solutions
        zerobranches = []
        for cond,evalcond,othervarsubs in eqs:
            # have to convert to fractions before substituting!
            if not all([self.isValidSolution(v) for s,v in othervarsubs]):
                continue
            othervarsubs = [(s,self.convertRealToRational(v)) for s,v in othervarsubs]
            NewEquations = [eq.subs(othervarsubs) for eq in AllEquations]
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

    def solvePairVariablesHalfAngle(self,raweqns,var0,var1,othersolvedvars,subs=None):
        """solves equations of two variables in sin and cos
        """
        varsym0 = self.Variable(var0)
        varsym1 = self.Variable(var1)
        varsyms = [varsym0,varsym1]
        unknownvars=[varsym0.cvar,varsym0.svar,varsym1.cvar,varsym1.svar]
        varsubs=varsym0.subs+varsym1.subs
        varsubsinv = varsym0.subsinv+varsym1.subsinv
        halftansubs = []
        for varsym in varsyms:
            halftansubs += [(varsym.cvar,(1-varsym.htvar**2)/(1+varsym.htvar**2)),(varsym.svar,2*varsym.htvar/(1+varsym.htvar**2))]
        dummyvars = []
        for othervar in othersolvedvars:
            v = self.Variable(othervar)
            dummyvars += [v.cvar,v.svar,v.var,v.htvar]

        polyeqs = []
        for eq in raweqns:
            peq = Poly(eq.subs(varsubs).subs(varsym0.svar**2,1-varsym0.cvar**2).expand().subs(varsym1.svar**2,1-varsym1.cvar**2),*unknownvars)
            if peq.has_any_symbols(varsym0.var) or peq.has_any_symbols(varsym1.var):
                raise self.CannotSolveError('expecting only sin and cos! %s'%peq)
            
            maxmonoms = [0,0,0,0]
            maxdenom = [0,0]
            for monoms in peq.iter_monoms():
                for i in range(4):
                    maxmonoms[i] = max(maxmonoms[i],monoms[i])
                maxdenom[0] = max(maxdenom[0],monoms[0]+monoms[1])
                maxdenom[1] = max(maxdenom[1],monoms[2]+monoms[3])
            eqnew = S.Zero
            for c,monoms in peq.iter_terms():
                term = c
                for i in range(4):
                    num,denom = fraction(halftansubs[i][1])
                    term *= num**monoms[i]
                # the denoms for 0,1 and 2,3 are the same
                for i in [0,2]:
                    denom = fraction(halftansubs[i][1])[1]
                    term *= denom**(maxdenom[i/2]-monoms[i]-monoms[i+1])
                eqnew += simplify(term)
            polyeqs.append(Poly(eqnew,varsym0.htvar,varsym1.htvar))

        solutions = [None,None]
        for ileftvar in range(2):
            leftvar = varsyms[ileftvar].htvar
            newpolyeqs = [Poly(eq,varsyms[1-ileftvar].htvar) for eq in polyeqs]
            for peq in newpolyeqs:
                if len(peq.monoms) == 1:
                    solutions[ileftvar] = self.checkFinalEquation(Poly(peq.coeffs[0],leftvar),subs)
                    if solutions[ileftvar] is not None:
                        break
            if solutions[ileftvar] is not None:
                break
            maxdegree = __builtin__.max([peq.degree for peq in newpolyeqs])
            for degree in range(1,maxdegree):
                newpolyeqs2 = [peq for peq in newpolyeqs if peq.degree <= degree]
                if degree+1 <= len(newpolyeqs2):
                    for eqs in combinations(newpolyeqs2,degree+1):
                        Mall = zeros((degree+1,degree+1))
                        for i,eq in enumerate(eqs):
                            for j in range(degree+1):
                                Mall[i,j] = eq.coeff(j)
                        #det=self.det_bareis(Mall,*(self.pvars+dummyvars+[leftvar]))
                        det=Mall.berkowitz_det()
                        if det.evalf() != S.Zero:
                            solutions[ileftvar] = self.checkFinalEquation(Poly(det,leftvar),subs)
                            if solutions[ileftvar] is not None:
                                break
                if solutions[ileftvar] is not None:
                    break

        # take the solution with the smallest degree
        pfinal = None
        ileftvar = None
        if solutions[0] is not None:
            if solutions[1] is not None and solutions[1].degree < solutions[0].degree:
                pfinal = solutions[1]
                ileftvar = 1
            else:
                pfinal = solutions[0]
                ileftvar = 0
        elif solutions[1] is not None:
            pfinal = solutions[1]
            ileftvar = 1

        if pfinal is None:
            raise self.CannotSolveError('solvePairVariablesHalfAngle: solve dialytically with %d equations'%(len(polyeqs)))

        jointsol = 2*atan(varsyms[ileftvar].htvar)
        solution = SolverPolynomialRoots(jointname=varsyms[ileftvar].name,poly=pfinal,jointeval=[jointsol],isHinge=self.isHinge(varsyms[ileftvar].name))
        solution.checkforzeros = []
        solution.postcheckforzeros = []
        solution.postcheckfornonzeros = []
        solution.postcheckforrange = []
        return solution

    def solveSingleVariableLinearly(self,raweqs,solvevar,othervars,maxnumeqs=2,douniquecheck=True):
        """tries to linearly solve for one variable treating everything else as constant
        """
        cvar = Symbol('c%s'%solvevar.name)
        svar = Symbol('s%s'%solvevar.name)
        varsubs = [(cos(solvevar),cvar),(sin(solvevar),svar)]
        othervarsubs = [(sin(v)**2,1-cos(v)**2) for v in othervars]
        eqpolys = [Poly(eq.subs(varsubs),cvar,svar) for eq in raweqs]
        eqpolys = [eq for eq in eqpolys if eq.degree == 1 and not eq.coeff(0,0).has_any_symbols(solvevar)]
        #eqpolys.sort(lambda x,y: iksolver.codeComplexity(x) - iksolver.codeComplexity(y))
        partialsolutions = []
        neweqs = []
        for p0,p1 in combinations(eqpolys,2):
            M = Matrix(2,3,[p0.coeff(1,0),p0.coeff(0,1),p0.coeff(0,0),p1.coeff(1,0),p1.coeff(0,1),p1.coeff(0,0)])
            M = M.subs(othervarsubs).expand()
            partialsolution = [-M[1,1]*M[0,2]+M[0,1]*M[1,2],M[1,0]*M[0,2]-M[0,0]*M[1,2],M[0,0]*M[1,1]-M[0,1]*M[1,0]]
            partialsolution = [eq.expand().subs(othervarsubs).expand() for eq in partialsolution]
            rank = [self.codeComplexity(eq) for eq in partialsolution]
            partialsolutions.append([rank,partialsolution])
            # cos(A)**2 + sin(A)**2 - 1 = 0, useful equation but the squares introduce wrong solutions
            #neweqs.append(partialsolution[0]**2+partialsolution[1]**2-partialsolution[2]**2)
        # try to cross
        partialsolutions.sort(lambda x, y: int(min(x[0])-min(y[0])))
        for (rank0,ps0),(rank1,ps1) in combinations(partialsolutions,2):
            if self.equal(ps0[0]*ps1[2]-ps1[0]*ps0[2],S.Zero):
                continue
            neweqs.append(ps0[0]*ps1[2]-ps1[0]*ps0[2])
            neweqs.append(ps0[1]*ps1[2]-ps1[1]*ps0[2])
            # probably a linear combination of the first two
            #neweqs.append(ps0[0]*ps1[1]-ps1[0]*ps0[1])
            # too long
            #neweqs.append(ps0[0]*ps1[0]+ps0[1]*ps1[1]-ps0[2]*ps1[2])
            if len(neweqs) >= maxnumeqs:
                break;
        neweqs2 = [eq.expand().subs(othervarsubs).expand() for eq in neweqs]
        if douniquecheck:
            reducedeqs = []
            i = 0
            while i < len(neweqs2):
                reducedeq = self.removecommonexprs(neweqs2[i])
                if neweqs2[i] != S.Zero and self.isExpressionUnique(reducedeqs,reducedeq) and self.isExpressionUnique(reducedeqs,-reducedeq):
                    reducedeqs.append(reducedeq)
                    i += 1
                else:
                    eq=neweqs2.pop(i)
        return neweqs2

    def solveHighDegreeEquationsHalfAngle(self,lineareqs,varsym,subs=None):
        """solve a set of equations in one variable with half-angle substitution
        """
        dummysubs = [(varsym.cvar,(1-varsym.htvar**2)/(1+varsym.htvar**2)),(varsym.svar,2*varsym.htvar/(1+varsym.htvar**2))]
        polyeqs = []
        for eq in lineareqs:
            peq = Poly(eq.subs(varsym.subs).subs(varsym.svar**2,1-varsym.cvar**2),varsym.cvar,varsym.svar)
            if peq.has_any_symbols(varsym.var):
                raise self.CannotSolveError('expecting only sin and cos! %s'%peq)
            
            # check if all terms are multiples of cos/sin
            maxmonoms = [0,0]
            maxdenom = 0
            for monoms in peq.iter_monoms():
                for i in range(2):
                    maxmonoms[i] = max(maxmonoms[i],monoms[i])
                maxdenom = max(maxdenom,monoms[0]+monoms[1])
            eqnew = S.Zero
            for c,monoms in peq.iter_terms():
                if c.evalf() != S.Zero: # big fractions might make this difficult to reduce to 0
                    term = c
                    for i in range(2):
                        num,denom = fraction(dummysubs[i][1])
                        term *= num**monoms[i]
                    # the denoms for 0,1 and 2,3 are the same
                    denom = fraction(dummysubs[0][1])[1]
                    term *= denom**(maxdenom-monoms[0]-monoms[1])
                    eqnew += simplify(term)
            polyeqs.append(Poly(eqnew,varsym.htvar))

        for peq in polyeqs:
            # do some type of resultants, for now just choose first polynomial
            finaleq = simplify(peq.as_basic()).expand()
            pfinal = Poly(self.removecommonexprs(finaleq,onlygcd=False,onlynumbers=True),varsym.htvar)
            pfinal = self.checkFinalEquation(pfinal,subs)
            if pfinal is not None:
                jointsol = 2*atan(varsym.htvar)
                solution = SolverPolynomialRoots(jointname=varsym.name,poly=pfinal,jointeval=[jointsol],isHinge=self.isHinge(varsym.name))
                solution.checkforzeros = []
                solution.postcheckforzeros = []
                solution.postcheckfornonzeros = []
                solution.postcheckforrange = []
                return solution

        raise self.CannotSolveError('half-angle substitution for joint %s failed, %d equations examined'%(varsym.var,len(polyeqs)))

    def checkFinalEquation(self,pfinal,subs=None):
        """check an equation in one variable for validity
        """
        assert(len(pfinal.symbols)==1)
        if subs is None:
            subs = []
        htvar = pfinal.symbols[0]
        # remove all trivial 0s
        while pfinal.degree > 0 and pfinal.coeff(0) == S.Zero:
            pfinalnew = Poly(S.Zero,htvar)
            for c,m in pfinal.iter_terms():
                if m[0] > 0:
                    pfinalnew += c*htvar**(m[0]-1)
            pfinal = pfinalnew
        # check to see that LC is non-zero for at least one solution
        if pfinal.LC.evalf() == S.Zero or all([pfinal.LC.subs(subs).subs(testconsistentvalue).evalf()==S.Zero for testconsistentvalue in self.testconsistentvalues]):
            return None

        # sanity check that polynomial can produce a solution and is not actually very small values
        found = False
        for testconsistentvalue in self.testconsistentvalues:
            coeffs = [pfinal.coeff(degree).subs(subs).subs(self.globalsymbols).subs(testconsistentvalue).evalf() for degree in range(pfinal.degree,-1,-1)]
            if coeffs[0] == S.Zero:
                continue
            
            if not all([c.is_number for c in coeffs]):
                # cannot evalute
                print 'cannot evalute',coeffs
                found = True
                return pfinal
            
            realsolution = pfinal.symbols[0].subs(subs).subs(self.globalsymbols).subs(testconsistentvalue).evalf()
            roots = mpmath.polyroots(coeffs)
            for root in roots:
                if abs(float(root.imag)) < 10.0**-self.precision and abs(float(root.real)-realsolution) < 10.0**-(self.precision-2):
                    found = True
                    break
            if found:
                break
        return pfinal if found else None

    def solveSingleVariable(self,raweqns,var,othersolvedvars,maxsolutions=4,maxdegree=2):
        varsym = self.Variable(var)
        cvar,svar = varsym.cvar,varsym.svar
        eqns = [eq.expand() for eq in raweqns if eq.has_any_symbols(var)]
        if len(eqns) == 0:
            raise self.CannotSolveError('not enough equations')
        
        # prioritize finding a solution when var is alone
        for eq in eqns:
            symbolgen = cse_main.numbered_symbols('const')
            eqnew, symbols = self.groupTerms(eq.subs(varsym.subs), [varsym.cvar,varsym.svar,var], symbolgen)
            try:
                ps = Poly(eqnew,varsym.svar)
                pc = Poly(eqnew,varsym.cvar)
                if ps.degree > 0 or pc.degree > 0 or ps.coeff(0) == S.Zero or pc.coeff(0) == S.Zero:
                    continue
            except polys.polynomial.PolynomialError:
                continue            
            numvar = self.countVariables(eqnew,var)
            if numvar >= 1 and numvar <= 2:
                tempsolutions = solve(eqnew,var)
                jointsolutions = [self.trigsimp(s.subs(symbols),othersolvedvars) for s in tempsolutions]
                if all([self.isValidSolution(s) and self.isValidSolution(s) for s in jointsolutions]):
                    return [SolverSolution(var.name,jointeval=jointsolutions,isHinge=self.isHinge(var.name))]

        solutions = []
        if len(eqns) > 1:
            neweqns = []
            listsymbols = []
            symbolgen = cse_main.numbered_symbols('const')
            for e in eqns:
                enew, symbols = self.groupTerms(e.subs(varsym.subs),[varsym.cvar,varsym.svar,var], symbolgen)
                # remove coupled equations
                if any([(m[0]>0)+(m[1]>0)+(m[2]>0)>1 for m in Poly(enew,varsym.cvar,varsym.svar,var).monoms]):
                    continue
                # ignore any equations with degree 3 or more
                if Poly(enew,varsym.svar).degree > maxdegree or Poly(enew,varsym.cvar).degree > maxdegree:
                    print 'ignoring equation: ',enew
                    continue
                rank = self.codeComplexity(enew)
                for s in symbols:
                    rank += self.codeComplexity(s[1])
                neweqns.append((rank,enew))
                listsymbols += symbols
            # since we're solving for two variables, we only want to use two equations, so
            # start trying all the equations starting from the least complicated ones to the most until a solution is found
            eqcombinations = []
            for eqs in combinations(neweqns,2):
                eqcombinations.append((eqs[0][0]+eqs[1][0],[Eq(e[1],0) for e in eqs]))
            eqcombinations.sort(lambda x, y: x[0]-y[0])
            hasgoodsolution = False
            for icomb,comb in enumerate(eqcombinations):
                # skip if too complex
                if len(solutions) > 0 and comb[0] > 200:
                    break
                # try to solve for both sin and cos terms
                try:
                    s = solve(comb[1],[varsym.svar,varsym.cvar])
                except PolynomialError, e:
                    print 'solveSingleVariable: ',e
                    continue
                if s is not None:
                    sollist = None
                    if hasattr(s,'has_key'):
                        if s.has_key(varsym.svar) and s.has_key(varsym.cvar):
                            sollist = [(s[varsym.svar],s[varsym.cvar])]
                        else:
                            sollist = []
                    else:
                        sollist = s
                    solversolution = SolverSolution(var.name,jointeval=[],isHinge=self.isHinge(var.name))
                    goodsolution = 0
                    for svarsol,cvarsol in sollist:
                        # solutions cannot be trivial
                        if self.chop((svarsol-cvarsol).subs(listsymbols)) == S.Zero:
                            break
                        if self.chop(svarsol.subs(listsymbols)) == S.Zero and self.chop(abs(cvarsol.subs(listsymbols)) - S.One) != S.Zero:
                            break
                        if self.chop(cvarsol.subs(listsymbols)) == S.Zero and self.chop(abs(svarsol.subs(listsymbols)) - S.One) != S.Zero:
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
                        simpsol = atan2(self.trigsimp(svarsol,othersolvedvars),self.trigsimp(cvarsol,othersolvedvars))
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
                        if goodsolution > 0:
                            hasgoodsolution = True
                        if len(sollist) == goodsolution and goodsolution == 1:
                            break
                        if len(solutions) >= maxsolutions:
                            # probably more than enough already?
                            break

            if len(solutions) > 0 or hasgoodsolution: # found a solution without any divides, necessary for pr2 head_torso lookat3d ik
                return solutions

        # solve one equation
        for eq in eqns:
            symbolgen = cse_main.numbered_symbols('const')
            eqnew, symbols = self.groupTerms(eq.subs(varsym.subs), [varsym.cvar,varsym.svar,varsym.var], symbolgen)
            try:
                # ignore any equations with degree 3 or more 
                ps = Poly(eqnew,varsym.svar)
                pc = Poly(eqnew,varsym.cvar)
                if ps.degree > maxdegree or pc.degree > maxdegree:
                    print 'cannot solve equation with high degree: %s'%str(eqnew)
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
            
            numcvar = self.countVariables(eqnew,varsym.cvar)
            numsvar = self.countVariables(eqnew,varsym.svar)
            if numcvar == 1 and numsvar == 1:
                a = Wild('a',exclude=[varsym.svar,varsym.cvar])
                b = Wild('b',exclude=[varsym.svar,varsym.cvar])
                c = Wild('c',exclude=[varsym.svar,varsym.cvar])
                m = eqnew.match(a*varsym.cvar+b*varsym.svar+c)
                if m is not None:
                    symbols += [(varsym.svar,sin(var)),(varsym.cvar,cos(var))]
                    asinsol = trigsimp(asin(-m[c]/abs(sqrt(m[a]*m[a]+m[b]*m[b]))).subs(symbols),deep=True)
                    constsol = -atan2(m[a],m[b]).subs(symbols).evalf()
                    jointsolutions = [constsol+asinsol,constsol+pi.evalf()-asinsol]
                    if all([self.isValidSolution(s) and self.isValidSolution(s) for s in jointsolutions]):
                        solutions.append(SolverSolution(var.name,jointeval=jointsolutions,isHinge=self.isHinge(var.name)))
                    continue
            if numcvar > 0:
                try:
                    # substitute cos
                    if self.countVariables(eqnew,varsym.svar) <= 1 or (self.countVariables(eqnew,varsym.cvar) <= 2 and self.countVariables(eqnew,varsym.svar) == 0): # anything more than 1 implies quartic equation
                        tempsolutions = solve(eqnew.subs(varsym.svar,sqrt(1-varsym.cvar**2)),varsym.cvar)
                        jointsolutions = [self.trigsimp(s.subs(symbols+varsym.subsinv),othersolvedvars) for s in tempsolutions]
                        if all([self.isValidSolution(s) and self.isValidSolution(s) for s in jointsolutions]):
                            solutions.append(SolverSolution(var.name,jointevalcos=jointsolutions,isHinge=self.isHinge(var.name)))
                        continue
                except self.CannotSolveError:
                    pass
                except NotImplementedError:
                    pass
            if numsvar > 0:
                # substitute sin
                try:
                    if self.countVariables(eqnew,varsym.svar) <= 1 or (self.countVariables(eqnew,varsym.svar) <= 2 and self.countVariables(eqnew,varsym.cvar) == 0): # anything more than 1 implies quartic equation
                        tempsolutions = solve(eqnew.subs(varsym.cvar,sqrt(1-varsym.svar**2)),varsym.svar)
                        jointsolutions = [self.trigsimp(s.subs(symbols+varsym.subsinv),othersolvedvars) for s in tempsolutions]
                        if all([self.isValidSolution(s) and self.isValidSolution(s) for s in jointsolutions]):
                            solutions.append(SolverSolution(var.name,jointevalsin=jointsolutions,isHinge=self.isHinge(var.name)))
                        continue
                except self.CannotSolveError:
                    pass
                except NotImplementedError:
                    pass
            if numcvar == 0 and numsvar == 0:
                tempsolutions = solve(eqnew,var)
                jointsolutions = [self.trigsimp(s.subs(symbols),othersolvedvars) for s in tempsolutions]
                if all([self.isValidSolution(s) and self.isValidSolution(s) for s in jointsolutions]):
                    solutions.append(SolverSolution(var.name,jointeval=jointsolutions,isHinge=self.isHinge(var.name)))
                continue
            try:
                solution = self.solveHighDegreeEquationsHalfAngle([eqnew],varsym,symbols)
                solutions.append(solution.subs(symbols))
            except self.CannotSolveError:
                pass
        if len(solutions) > 0:
            return solutions

        return [self.solveHighDegreeEquationsHalfAngle(eqns,varsym)]
        
    def solvePairVariables(self,raweqns,var0,var1,othersolvedvars,maxcomplexity=50):
        # make sure both variables are hinges
        if not self.isHinge(var0.name) or not self.isHinge(var1.name):
            raise self.CannotSolveError('pairwise variables only supports hinge joints')
        
        varsym0 = self.Variable(var0)
        varsym1 = self.Variable(var1)
        cvar0,svar0 = varsym0.cvar, varsym0.svar
        cvar1,svar1 = varsym1.cvar, varsym1.svar
        varsubs=varsym0.subs+varsym1.subs
        varsubsinv = varsym0.subsinv+varsym1.subsinv
        unknownvars=[cvar0,svar0,cvar1,svar1]
        reducesubs = [(svar0**2,1-cvar0**2),(svar1**2,1-cvar1**2)]
        eqns = [eq.subs(varsubs).subs(reducesubs).expand() for eq in raweqns if eq.has_any_symbols(var0,var1)]
        if len(eqns) <= 1:
            raise self.CannotSolveError('not enough equations')
        
        # group equations with single variables
        symbolgen = cse_main.numbered_symbols('const')
        orgeqns = []
        allsymbols = []
        for eq in eqns:
            eqnew, symbols = self.groupTerms(eq, unknownvars, symbolgen)
            allsymbols += symbols
            #p = Poly(eqnew,*unknownvars)
            # make sure there are no monomials that have more than 2 diferent terms
            #if all([__builtin__.sum([m[j]>0 for j in range(len(unknownvars))])<=2 for m in p.monoms]):
            orgeqns.append([self.codeComplexity(eq),Poly(eqnew,*unknownvars)])
        orgeqns.sort(lambda x, y: x[0]-y[0])
        neweqns = orgeqns[:]
        
        pairwisesubs = [(svar0*cvar1,Symbol('s0c1')),(svar0*svar1,Symbol('s0s1')),(cvar0*cvar1,Symbol('c0c1')),(cvar0*svar1,Symbol('c0s1')),(cvar0*svar0,Symbol('s0c0')),(cvar1*svar1,Symbol('c1s1'))]
        pairwiseinvsubs = [(f[1],f[0]) for f in pairwisesubs]
        pairwisevars = [f[1] for f in pairwisesubs]
        reduceeqns = [Poly(eq.as_basic().subs(pairwisesubs),*pairwisevars) for rank,eq in orgeqns if rank < 4*maxcomplexity]
        for i,eq in enumerate(reduceeqns):
            if eq.TC != S.Zero and not eq.TC.is_Symbol:
                n=symbolgen.next()
                allsymbols.append((n,eq.TC.subs(allsymbols)))
                reduceeqns[i] += n-eq.TC
        
        # try to at least subtract as much paired variables out
        eqcombs = [c for c in combinations(reduceeqns,2)]
        while len(eqcombs) > 0 and len(neweqns) < 20:
            eq0,eq1 = eqcombs.pop()
            for i in range(6):
                monom = [0,0,0,0,0,0]
                monom[i] = 1
                if eq0.coeff(*monom) != 0 and eq1.coeff(*monom) != 0:
                    tempeq = (eq0.as_basic()*eq1.coeff(*monom)-eq0.coeff(*monom)*eq1.as_basic()).subs(allsymbols+pairwiseinvsubs).expand()
                    if self.codeComplexity(tempeq) > 200:
                        continue
                    eq = simplify(tempeq)
                    if self.codeComplexity(eq) > maxcomplexity:
                        # don't need such complex equations
                        continue
                    if not self.isExpressionUnique(eqns,eq) or not self.isExpressionUnique(eqns,-eq):
                        continue
                    if eq.has_any_symbols(*unknownvars) and eq != S.Zero: # be a little strict about new candidates
                        eqns.append(eq)
                        eqnew, symbols = self.groupTerms(eq, unknownvars, symbolgen)
                        allsymbols += symbols
                        neweqns.append([self.codeComplexity(eq),Poly(eqnew,*unknownvars)])

        # try single variable solution
        try:
            return self.solveSingleVariable([e.subs(varsubsinv) for e in eqns if not e.has_any_symbols(cvar1,svar1,var1)],var0,othersolvedvars)
        except self.CannotSolveError:
            pass

        try:
            return self.solveSingleVariable([e.subs(varsubsinv) for e in eqns if not e.has_any_symbols(cvar0,svar0,var0)],var1,othersolvedvars)
        except self.CannotSolveError:
            pass

        orgeqns = neweqns[:]
        # try to solve for all pairwise variables
        systemofequations = []
        for i in range(len(reduceeqns)):
            if reduceeqns[i].has_any_symbols(pairwisevars[4],pairwisevars[5]):
                continue
            if not all([__builtin__.sum(m) <= 1 for m in reduceeqns[i].iter_monoms()]):
                continue
            arr = [S.Zero]*5
            for c,m in reduceeqns[i].iter_terms():
                if __builtin__.sum(m) == 1:
                    arr[m.index(1)] = c
                else:
                    arr[4] = c
            systemofequations.append(arr)

        singleeqs = None
        for eqs in combinations(systemofequations,4):
            M = zeros((4,4))
            B = zeros((4,1))
            for i,arr in enumerate(eqs):
                for j in range(4):
                    M[i,j] = arr[j]
                B[i] = -arr[4]
            det = self.det_bareis(M,*(self.pvars+unknownvars)).subs(allsymbols)
            if det.evalf() != S.Zero:
                X = M.adjugate()*B
                singleeqs = []
                for i in range(4):
                    eq = (pairwisesubs[i][0]*det - X[i]).subs(allsymbols)
                    eqnew, symbols = self.groupTerms(eq, unknownvars, symbolgen)
                    allsymbols += symbols
                    singleeqs.append([self.codeComplexity(eq),Poly(eqnew,*unknownvars)])
                break
        if singleeqs is not None:
            neweqns += singleeqs
            neweqns.sort(lambda x, y: x[0]-y[0])

        # check if any equations are at least degree 1 (if not, try to compute some)
        for ivar in range(2):
            polyunknown = []
            for rank,eq in orgeqns:
                p = Poly(eq,unknownvars[2*ivar],unknownvars[2*ivar+1])
                if p.degree == 1 and __builtin__.sum(p.lead_monom) == 1:
                    polyunknown.append((rank,p))
            if len(polyunknown) > 0:
                break
        if len(polyunknown) == 0:
            addedeqs = eqns[:]
            polyeqs = []
            for ivar in range(2):
                polyunknown = []
                for rank,eq in orgeqns:
                    p = Poly(eq,unknownvars[2*ivar],unknownvars[2*ivar+1])
                    polyunknown.append(p.subs(unknownvars[2*ivar+1]**2,1-unknownvars[2*ivar]**2))
                if len(polyunknown) >= 2:
                    monomtoremove = [[polyunknown,(2,0)],[polyunknown,(1,1)]]
                    for curiter in range(2):
                        # remove the square
                        polyunknown,monom = monomtoremove[curiter]
                        pbase = [p for p in polyunknown if p.coeff(*monom) != S.Zero]
                        if len(pbase) == 0:
                            continue
                        pbase = pbase[0]
                        for i in range(len(polyunknown)):
                            eq = (polyunknown[i]*pbase.coeff(*monom)-pbase*polyunknown[i].coeff(*monom)).as_basic().subs(allsymbols).expand()
                            if eq != S.Zero and self.isExpressionUnique(addedeqs,eq):
                                eqnew, symbols = self.groupTerms(eq, unknownvars, symbolgen)
                                allsymbols += symbols
                                p = Poly(eqnew,*pbase.symbols)
                                if p.coeff(1,1) != S.Zero and curiter == 0:
                                    monomtoremove[1][0].insert(0,p)
                                polyeqs.append([self.codeComplexity(eqnew),Poly(eqnew,*unknownvars)])
                                addedeqs.append(eq)
            orgeqns += polyeqs
        orgeqns.sort(lambda x,y: x[0]-y[0])
        
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
                    if all([__builtin__.sum(m) == m[i]+m[i+1] for m in eq.iter_monoms()]):
                        listeqs.append(eq)
                    else:
                        # make sure there's only one monom that includes other variables
                        othervars = 0
                        for m in eq.iter_monoms():
                            if __builtin__.sum(m) >  m[i]+m[i+1]:
                                if m[i] == 0 and m[i+1]==0:
                                    othervars += 1
                                else:
                                    othervars = 10000
                        if othervars <= 1:
                            listeqs.append(eq)
                groups.append(listeqs)
                groups.append([]) # necessary to get indices correct
            goodgroup = [(i,g) for i,g in enumerate(groups) if len(g) >= 2]
            useconic=True
            if len(goodgroup) == 0:
                try:
                    return [self.solvePairVariablesHalfAngle(raweqns,var0,var1,othersolvedvars)]
                except self.CannotSolveError,e:
                    print e

                # try a separate approach where the two variables are divided on both sides
                neweqs = []
                for rank,eq in orgeqns:
                    p = Poly(eq,unknownvars[0],unknownvars[1])
                    iscoupled = False
                    for m in p.iter_monoms():
                        if __builtin__.sum(m) > 0:
                            if p.coeff(*m).has_any_symbols(unknownvars[2],unknownvars[3]):
                                iscoupled = True
                                break
                    if not iscoupled:
                        neweqs.append([p-p.coeff(0,0),Poly(-p.coeff(0,0),unknownvars[2],unknownvars[3])])
                if len(neweqs) > 0:
                    for ivar in range(2):
                        lineareqs = [eq for eq in neweqs if __builtin__.sum(eq[ivar].lead_monom)==1]
                        for paireq0,paireq1 in combinations(lineareqs,2):
                            print 'solving separated equations with linear terms'
                            eq0 = paireq0[ivar]
                            eq1 = paireq1[ivar]
                            disc = (eq0.coeff(1,0)*eq1.coeff(0,1) - eq0.coeff(0,1)*eq1.coeff(1,0)).subs(allsymbols).expand()
                            if disc == S.Zero:
                                continue
                            othereq0 = paireq0[1-ivar].as_basic() - eq0.coeff(0,0)
                            othereq1 = paireq1[1-ivar].as_basic() - eq1.coeff(0,0)                        
                            csol = - eq1.coeff(0,1) * othereq0 + eq0.coeff(0,1) * othereq1
                            ssol = eq1.coeff(1,0) * othereq0 - eq0.coeff(1,0) * othereq1
                            polysymbols = paireq0[1-ivar].symbols
                            totaleq = (csol**2+ssol**2-disc**2).subs(allsymbols).expand()
                            if self.codeComplexity(totaleq) < 4000:
                                print 'simplifying final equation',self.codeComplexity(totaleq)
                                totaleq = simplify(totaleq)
                            ptotal_cos = Poly(totaleq,*polysymbols).subs(polysymbols[0]**2,1-polysymbols[1]**2).subs(polysymbols[1]**2,1-polysymbols[0]**2)
                            ptotal_sin = Poly(S.Zero,*polysymbols)
                            for c,m in ptotal_cos.iter_terms():
                                if m[1] > 0:
                                    assert m[1] == 1
                                    ptotal_sin = ptotal_sin.sub_term(c,(m[0],0))
                                    ptotal_cos = ptotal_cos.sub_term(c,m)
                            finaleq = (ptotal_cos.as_basic()**2 - (1-polysymbols[0]**2)*ptotal_sin.as_basic()**2).expand()
                            # sometimes denominators can accumulate
                            pfinal = Poly(self.removecommonexprs(finaleq,onlygcd=False,onlynumbers=True),polysymbols[0])
                            # check to see that LC is non-zero for at least one solution
                            if pfinal.LC.evalf() == S.Zero or all([pfinal.LC.subs(testconsistentvalue).evalf()==S.Zero for testconsistentvalue in self.testconsistentvalues]):
                                raise self.CannotSolveError('leading coefficient is always zero in %s'%(str(pfinal)))
                            
                            jointsol = atan2(ptotal_cos.as_basic()/ptotal_sin.as_basic(), polysymbols[0])
                            var = var1 if ivar == 0 else var0
                            solution = SolverPolynomialRoots(jointname=var.name,poly=pfinal,jointeval=[jointsol],isHinge=self.isHinge(var.name))
                            solution.postcheckforzeros = [ptotal_sin.as_basic()]
                            solution.postcheckfornonzeros = []
                            solution.postcheckforrange = []
                            return [solution]
                    
                # if maxnumeqs is any less, it will miss linearly independent equations
                lineareqs = self.solveSingleVariableLinearly(raweqns,var0,[var1],maxnumeqs=len(raweqns))
                if len(lineareqs) > 0:
                    try:
                        return [self.solveHighDegreeEquationsHalfAngle(lineareqs,varsym1)]
                    except self.CannotSolveError,e:
                        print e
                raise self.CannotSolveError('cannot cleanly separate pair equations')

        varindex=goodgroup[0][0]
        var = var0 if varindex < 2 else var1
        varsym = varsym0 if varindex < 2 else varsym1
        unknownvar=unknownvars[goodgroup[0][0]]
        rawsolutions = None
        # try single variable solution
        try:
            rawsolutions=self.solveSingleVariable([e.as_basic().subs(allsymbols+varsubsinv) for e in goodgroup[0][1] if not e.has_any_symbols(cvar1,svar1,var1)],var0,othersolvedvars)
        except self.CannotSolveError:
            pass

        try:
            rawsolutions = self.solveSingleVariable([e.as_basic().subs(allsymbols+varsubsinv) for e in goodgroup[0][1] if not e.has_any_symbols(cvar0,svar0,var0)],var1,othersolvedvars)
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
        domagicsquare = False
        for i in range(2):
            if useconic:
                terms=[(c,m) for c,m in eqs[i].iter_terms() if __builtin__.sum(m) - m[varindex] - m[varindex+1] > 0]
            else:
                terms=[(c,m) for c,m in eqs[i].iter_terms() if __builtin__.sum(m) - m[varindex] > 0]
            if len(terms) > 0:
                simpleterms.append(eqs[i].sub_term(*terms[0]).as_basic()/terms[0][0]) # divide by the coeff
                complexterms.append(Poly(0,*unknownvars).add_term(S.One,terms[0][1]).as_basic())
                domagicsquare = True
            else:
                simpleterms.append(eqs[i].as_basic())
                complexterms.append(S.Zero)
        finaleq = None
        checkforzeros = []
        if domagicsquare:
            # here is the magic transformation:
            finaleq = self.trigsimp(expand(((complexterms[0]**2+complexterms[1]**2) - simpleterms[0]**2 - simpleterms[1]**2).subs(varsubsinv)),othersolvedvars+[var0,var1]).subs(varsubs)
            denoms = [fraction(simpleterms[0])[1], fraction(simpleterms[1])[1], fraction(complexterms[0])[1], fraction(complexterms[1])[1]]
            lcmvars = self.pvars+unknownvars
            for othersolvedvar in othersolvedvars:
                lcmvars += self.Variable(othersolvedvar).vars
            denomlcm = Poly(S.One,*lcmvars)
            for denom in denoms:
                if denom != S.One:
                    checkforzeros.append(self.removecommonexprs(denom,onlygcd=False,onlynumbers=True))
                    denomlcm = Poly(lcm(denomlcm,denom),*lcmvars)
            finaleq = simplify(finaleq*denomlcm.as_basic()**2)
            complementvarindex = varindex-(varindex%2)+((varindex+1)%2)
            complementvar = unknownvars[complementvarindex]
            finaleq = simplify(finaleq.subs(complementvar**2,1-unknownvar**2)).subs(allsymbols).expand()
            if not self.isValidSolution(finaleq):
                raise self.CannotSolveError('failed to solve pairwise equation: %s'%str(finaleq))
            
        else:
            # try to reduce finaleq
            p0 = Poly(simpleterms[0],unknownvars[varindex],unknownvars[varindex+1])
            p1 = Poly(simpleterms[1],unknownvars[varindex],unknownvars[varindex+1])
            if p0.degree > 1 and p1.degree > 1 and p0.degree == p1.degree and p0.lead_term[1] == p1.lead_term[1]:
                finaleq = (p0*p1.lead_term[0]-p1*p0.lead_term[0]).as_basic()
                finaleq = expand(simplify(finaleq.subs(allsymbols)))
                if finaleq == S.Zero:
                    finaleq = expand(p0.as_basic().subs(allsymbols))
        if finaleq is None:
            raise self.CannotSolveError('solvePairVariables: did not compute a final variable. This is a weird condition...')
        
        if useconic:
            #if not self.isHinge(var.name):
            #    print 'got conic equation from a non-hinge joint?: ',finaleq
            #return [SolverConicRoots(var.name,[finaleq],isHinge=self.isHinge(var.name))]
            solution = self.solveHighDegreeEquationsHalfAngle([finaleq],varsym)
            solution.checkforzeros += checkforzeros
            return [solution]

        newunknownvars = unknownvars[:]
        newunknownvars.remove(unknownvar)
        if finaleq.has_any_symbols(*newunknownvars):
            raise self.CannotSolveError('ray ik bad equation %s'%str(finaleq))
        # now that everything is with respect to one variable, simplify and solve the equation
        eqnew, symbols = self.groupTerms(finaleq, unknownvars, symbolgen)
        allsymbols += symbols
        solutions=solve(eqnew,unknownvar)
        print 'pair solution: ',eqnew,',',solutions
        if solutions:
            
            solversolution=SolverSolution(var.name, isHinge=self.isHinge(var.name))
            if (varindex%2)==0:
                solversolution.jointevalcos=[self.trigsimp(s.subs(allsymbols+varsubsinv),othersolvedvars).subs(varsubs) for s in solutions]
            else:
                solversolution.jointevalsin=[self.trigsimp(s.subs(allsymbols+varsubsinv),othersolvedvars).subs(varsubs) for s in solutions]
            return [solversolution]
        raise self.CannotSolveError('cannot solve pair equation')
        
    ## SymPy helper routines

    @staticmethod
    def isValidSolution(expr):
        """return true if solution does not contain any nan or inf terms"""
        if expr.is_number:
            e=expr.evalf()
            if e.has(I) or math.isinf(e) or math.isnan(e):
                return False
            return True
        if expr.is_Mul:
            # first multiply all numbers
            number = S.One
            for arg in expr.args:
                if arg.is_number:
                    number *= arg
                elif not IKFastSolver.isValidSolution(arg):
                    return False
            # finally evalute the multiplied form
            return IKFastSolver.isValidSolution(number.evalf())
        for arg in expr.args:
            if not IKFastSolver.isValidSolution(arg):
                return False
        return True

    @staticmethod
    def recursiveFraction(expr):
        if expr.is_Add:
            allpoly = []
            finaldenom = S.One
            for arg in expr.args:
                n,d = IKFastSolver.recursiveFraction(arg)
                finaldenom = finaldenom*d
                allpoly.append([n,d])
            finalnum = S.Zero
            for n,d in allpoly:
                finalnum += n*(finaldenom/d)
            return finalnum,finaldenom
        elif expr.is_Mul:
            finalnum = S.One
            finaldenom = S.One
            for arg in expr.args:
                n,d = IKFastSolver.recursiveFraction(arg)
                finalnum = finalnum * n
                finaldenom = finaldenom * d
            return finalnum,finaldenom
        elif expr.is_Pow and expr.exp.is_number:
            n,d=IKFastSolver.recursiveFraction(expr.base)
            if expr.exp < 0:
                exponent = -expr.exp
                n,d = d,n
            else:
                exponent = expr.exp
            return n**exponent,d**exponent
        else:
            return fraction(expr)

    @staticmethod
    def groupTerms(expr,vars,symbolgen = None):
        """Separates all terms that do have var in them"""
        if symbolgen is None:
            symbolgen = cse_main.numbered_symbols('const')
        symbols = []
        p = Poly(expr,*vars)
        newexpr = S.Zero
        for c,m in p.iter_terms():
            if not c.is_number and not c.is_Symbol:
                # if it is a product of a symbol and a number, then ignore
                if not c.is_Mul or not all([e.is_number or e.is_Symbol for e in c.args]):
                    sym = symbolgen.next()
                    symbols.append((sym,c))
                    c = sym
            if __builtin__.sum(m) == 0:
                newexpr += c
            else:
                for i,degree in enumerate(m):
                    c = c*vars[i]**degree
                newexpr += c
        return newexpr,symbols

    @staticmethod
    def replaceNumbers(expr,symbolgen = None):
        """Replaces all numbers with symbols, this is to make gcd faster when fractions get too big"""
        if symbolgen is None:
            symbolgen = cse_main.numbered_symbols('const')
        symbols = []
        if expr.is_number:
            result = symbolgen.next()
            symbols.append((result,expr))
        elif expr.is_Mul:
            result = S.One
            for arg in expr.args:
                newresult, newsymbols = IKFastSolver.replaceNumbers(arg,symbolgen)
                result *= newresult
                symbols += newsymbols
        elif expr.is_Add:
            result = S.Zero
            for arg in expr.args:
                newresult, newsymbols = IKFastSolver.replaceNumbers(arg,symbolgen)
                result += newresult
                symbols += newsymbols
        elif expr.is_Pow:
            # don't replace the exponent
            newresult, newsymbols = IKFastSolver.replaceNumbers(expr.base,symbolgen)
            symbols += newsymbols
            result = newresult**expr.exp
        else:
            result = expr
        return result,symbols

    def subsExpressions(self, expr, subexprs):
        changed = True
        while changed:
            newexpr = expr.subs(subexprs)
            if simplify(expr-newexpr) == 0:
                break
            expr = newexpr
        return expr

    @staticmethod
    def frontnumbers(eq):
        if eq.is_Number:
            return [eq]
        if eq.is_Mul:
            n = []
            for arg in eq.args:
                n += IKFastSolver.frontnumbers(arg)
            return n
        return []

    @staticmethod
    def removecommonexprs(eq,returncommon=False,onlygcd=False,onlynumbers=True):
        """removes common expressions from a sum. Assumes all the coefficients are rationals. For example:
        a*c_0 + a*c_1 + a*c_2 = 0
        will return in
        c_0 + c_1 + c_2 = 0
        """
        eq = eq.expand() # doesn't work otherwise
        if eq.is_Add:
            exprs = eq.args
            totaldenom = S.One
            common = S.One
            if onlynumbers:
                for i in range(len(exprs)):
                    denom = S.One
                    for d in IKFastSolver.frontnumbers(fraction(exprs[i])[1]):
                        denom *= d
                    if denom != S.One:
                        exprs = [expr*denom for expr in exprs]
                        totaldenom *= denom
                if onlygcd:
                    common = None
                    for i in range(len(exprs)):
                        coeff = S.One
                        for n in IKFastSolver.frontnumbers(exprs[i]):
                            coeff *= n
                        if common == None:
                            common = coeff
                        else:
                            common = igcd(common,coeff)
                        if common == S.One:
                            break
            else:
                for i in range(len(exprs)):
                    denom = fraction(exprs[i])[1]
                    if denom != S.One:
                        exprs = [expr*denom for expr in exprs]
                        totaldenom *= denom
                # there are no fractions, so can start simplifying
                common = exprs[0]/fraction(cancel(exprs[0]/exprs[1]))[0]
                for i in range(2,len(exprs)):
                    common = common/fraction(cancel(common/exprs[i]))[0]
                    if common.is_number:
                        common=S.One
            # find the smallest number and divide by it
            if not onlygcd:
                smallestnumber = None
                for expr in exprs:
                    if expr.is_number:
                        if smallestnumber is None or smallestnumber > abs(expr):
                            smallestnumber = abs(expr)
                    elif expr.is_Mul:
                        n = S.One
                        for arg in expr.args:
                            if arg.is_number:
                                n *= arg
                        if smallestnumber is None or smallestnumber > abs(n):
                            smallestnumber = abs(n)
                if smallestnumber is not None:
                    common = common*smallestnumber
            eq = S.Zero
            for expr in exprs:
                eq += expr/common
            if returncommon:
                return eq,common/totaldenom
        elif eq.is_Mul:
            coeff = S.One
            for d in IKFastSolver.frontnumbers(eq):
                coeff *= d
            if returncommon:
                return eq/coeff,coeff
            return eq/coeff
        if returncommon:
            return eq,S.One
        return eq

    @staticmethod
    def det_bareis(M,*vars):
        """Compute matrix determinant using Bareis' fraction-free
           algorithm which is an extension of the well known Gaussian
           elimination method. This approach is best suited for dense
           symbolic matrices and will result in a determinant with
           minimal number of fractions. It means that less term
           rewriting is needed on resulting formulae.

           TODO: Implement algorithm for sparse matrices (SFF).

           Function from sympy/matrices/matrices.py
        """
        if not M.is_square:
            raise NonSquareMatrixException()
        n = M.rows
        M = M[:,:] # make a copy
        if n == 1:
            det = M[0, 0]
        elif n == 2:
            det = M[0, 0]*M[1, 1] - M[0, 1]*M[1, 0]
        else:
            sign = 1 # track current sign in case of column swap

            for k in range(n-1):
                # look for a pivot in the current column
                # and assume det == 0 if none is found
                if M[k, k] == 0:
                    for i in range(k+1, n):
                        if M[i, k] != 0:
                            M.row_swap(i, k)
                            sign *= -1
                            break
                    else:
                        return S.Zero

                # proceed with Bareis' fraction-free (FF)
                # form of Gaussian elimination algorithm
                for i in range(k+1, n):
                    for j in range(k+1, n):
                        D = M[k, k]*M[i, j] - M[i, k]*M[k, j]

                        if k > 0:
                            if len(vars) > 0:
                                #print i,M[k-1, k-1]
                                D,r = div(Poly(D,*vars),M[k-1, k-1])
                            else:
                                D /= M[k-1, k-1]

                        if D.is_Atom:
                            M[i, j] = D
                        else:
                            if len(vars) > 0:
                                M[i, j] = D
                            else:
                                M[i, j] = Poly.cancel(D)

            det = sign * M[n-1, n-1]
            
        return det.expand()

    @staticmethod
    def LUdecompositionFF(M,*vars):
        """
        Returns 4 matrices P, L, D, U such that PA = L D**-1 U.

        From the paper "fraction-free matrix factors..." by Zhou and Jeffrey
        """
        n, m = M.rows, M.cols
        U, L, P = M[:,:], eye(n), eye(n)
        DD = zeros(n) # store it smarter since it's just diagonal
        oldpivot = 1

        for k in range(n-1):
            if U[k,k] == S.Zero:
                for kpivot in range(k+1, n):
                    if U[kpivot, k] != S.Zero:
                        break
                else:
                    raise ValueError("Matrix is not full rank")
                U[k, k:], U[kpivot, k:] = U[kpivot, k:], U[k, k:]
                L[k, :k], L[kpivot, :k] = L[kpivot, :k], L[k, :k]
                P[k, :], P[kpivot, :] = P[kpivot, :], P[k, :]
            L[k,k] = Ukk = U[k,k]
            DD[k,k] = oldpivot * Ukk
            for i in range(k+1, n):
                L[i,k] = Uik = U[i,k]
                for j in range(k+1, m):
                    if len(vars) == 0:
                        U[i,j] = (Ukk * U[i,j] - U[k,j]*Uik) / oldpivot
                    else:
                        print i,j,oldpivot
                        q,r = div(Poly(Ukk * U[i,j] - U[k,j]*Uik,*vars),oldpivot)
                        assert(r==S.Zero)
                        U[i,j] = q
                U[i,k] = S.Zero
            oldpivot = Ukk
        DD[n-1,n-1] = oldpivot
        return P, L, DD, U

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
                      help='robot file (COLLADA or OpenRAVE XML)')
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
    parser.add_option('--lang', action='store',type='string',dest='lang',default='cpp',
                      help='The language to generate the code in (default=%default), available=('+','.join(name for name,value in CodeGenerators.iteritems())+')')

    (options, args) = parser.parse_args()
    if options.fkfile is None or options.baselink is None or options.eelink is None:
        print('Error: Not all arguments specified')
        sys.exit(1)

    solvefn=IKFastSolver.solveFullIK_6D
    if options.rotation3donly:
        solvefn = IKFastSolver.solveFullIK_Rotation3D
    elif options.rotation2donly:
        solvefn = IKFastSolver.solveFullIK_Direction3D
    elif options.translation3donly:
        solvefn = IKFastSolver.solveFullIK_Translation3D

    tstart = time.time()
    RaveInitialize()
    try:
        env=Environment()
        kinbody=env.ReadKinBodyXMLFile(options.fkfile)
        env.AddKinBody(kinbody)
        kinematics = IKFastSolver(kinbody,kinbody)
        code = kinematics.generateIkSolver(options.baselink,options.eelink,options.freejointinds,solvefn=solvefn,lang=options.lang)
        success = True if len(code) > 0 else False
        print 'total time for ik generation of %s is %fs'%(options.savefile,time.time()-tstart)
        if success:
            open(options.savefile,'w').write(code)
    finally:
        RaveDestroy()
