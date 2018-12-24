#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (Lesser GPL)
#
# Copyright (C) 2009-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
"""generates C++ code from the IKFastSolver AST.
"""
from __future__ import with_statement # for python 2.5

from sympy import __version__ as sympy_version
if sympy_version < '0.7.0':
    raise ImportError('ikfast needs sympy 0.7.x or greater')

import sys, copy, time, datetime
import cStringIO
try:
    from openravepy.metaclass import AutoReloader
except:
    class AutoReloader:
        pass

# import the correct iktypes from openravepy (if present)
try:
    from openravepy import IkParameterization
    IkType = IkParameterization.Type
except:
    class IkType:
        Transform6D=0x67000001
        Rotation3D=0x34000002
        Translation3D=0x33000003
        Direction3D=0x23000004
        Ray4D=0x46000005
        Lookat3D=0x23000006
        TranslationDirection5D=0x56000007
        TranslationXY2D=0x22000008
        TranslationXYOrientation3D=0x33000009
        TranslationLocalGlobal6D=0x3600000a
        TranslationXAxisAngle4D=0x4400000b
        TranslationYAxisAngle4D=0x4400000c
        TranslationZAxisAngle4D=0x4400000d
        TranslationXAxisAngleZNorm4D=0x4400000e
        TranslationYAxisAngleXNorm4D=0x4400000f
        TranslationZAxisAngleYNorm4D=0x44000010

from sympy import *

try:
    import re # for indenting
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

try:
    # not necessary, just used for testing
    import swiginac
    using_swiginac = True
except ImportError:
    using_swiginac = False

import logging
log = logging.getLogger('openravepy.ikfast')

from sympy.core import function # for sympy 0.7.1+
class fmod(function.Function):
    nargs = 2
    is_real = True
    is_Function = True

class atan2check(atan2):
    nargs = 2
    is_real = True
    is_Function = True

class RemoveAbsFn(function.Function):
    """defines a function that so that things don't get evaluated internally
    ie protects Pow(-1,0.5) from evaluating to I: RemoveAbsFn(Pow(base,expr.exp,evaluate=False))
    """
    nargs = 1
    is_real = True
    is_Function = True

def evalNumbers(expr):
    """Replaces all numbers with symbols, this is to make gcd faster when fractions get too big
    Also returns checks that need to be all >= 0
    """
    if expr.is_number:
        return expr.evalf()
    
    elif expr.is_Mul:
        result = S.One
        for arg in expr.args:
            newresult = evalNumbers(arg)
            result *= newresult
    elif expr.is_Add:
        # because the arguments can get to the thousands, do a tree for adding numbers
        evalexprs = []
        for arg in expr.args:
            newresult = evalNumbers(arg)
            evalexprs.append(newresult)
        N = len(evalexprs)
        while N > 1:
            for i in range(N/2):
                evalexprs[2*i]+=evalexprs[2*i+1]
                evalexprs[i] = evalexprs[2*i]
            if N & 1:
                evalexprs[N/2] = evalexprs[N-1]
                N += 1
            N /= 2
        return evalexprs[0]
    
    elif expr.is_Pow:
        # don't replace the exponent
        # (-x)**0.5 unfortunately evalutes to I*x**0.5, so have to take the absolute value no matter what...
        base = evalNumbers(expr.base)
        if abs(expr.exp) < S.One:
            return RemoveAbsFn(base)**expr.exp
        
        else:
            return base**expr.exp
        
    elif expr.is_Function:
        args = []
        for arg in expr.args:
            newresult = evalNumbers(arg)
            args.append(newresult)
        return expr.func(*args)
    
    else:
        result = expr
    return result

def customcse(rawexprs,symbols=None):
    if not hasattr(rawexprs,'__iter__') and not hasattr(rawexprs,'__array__'):
        rawexprs = [rawexprs]
    if symbols is None:
        symbols = cse_main.numbered_symbols('x')
    # fractions can get big, so evaluate as many decimals as possible
    reduced_exprs = []
    allexprs = []
    for iexpr,expr in enumerate(rawexprs):
        evalexpr = evalNumbers(expr)
        complexity = evalexpr.count_ops()
        # need to threshold complexity or otherwise cse will not terminate
        if complexity > 300:
            reduced_exprs.append(evalexpr)
        else:
            allexprs.append(evalexpr)
            reduced_exprs.append(None)
            
    newreplacements = []
    if len(allexprs)>0:
        try:
            replacements,reduced_exprs2 = cse(allexprs,symbols=symbols)
        except PolynomialError: # non-commutative expressions are not supported
            reduced_exprs2 = allexprs
            replacements = []
        # have to maintain the same order
        for expr in reduced_exprs2:
            for i in range(len(reduced_exprs)):
                if reduced_exprs[i] is None:
                    reduced_exprs[i] = expr
                    break
        assert(all([expr is not None for expr in reduced_exprs]))
        # look for any expressions of the order of (x**(1/a))**b, usually computer wants x^(b/a)
        for r in replacements:
            newr = r[1]
            if newr.is_Pow and newr.exp.is_number and newr.base.is_Symbol:
                baseexpr = newr.base.subs(replacements)
                if baseexpr.is_Pow and baseexpr.exp.is_number:
                    newreplacements.append((r[0],baseexpr.base**(newr.exp*baseexpr.exp)))
                    continue
            newreplacements.append((r[0],newr))
    return newreplacements,reduced_exprs

class CodeGenerator(AutoReloader):
    """Generates C++ code from an AST generated by IKFastSolver.
    """
    _checkpreemptfn = None
    def __init__(self,kinematicshash='',version='0',iktypestr='',checkpreemptfn=None):
        """
        :param checkpreemptfn: checkpreemptfn(msg, progress) called periodically at various points in ikfast. Takes in two arguments to notify user how far the process has completed.
        """
        self.symbolgen = cse_main.numbered_symbols('x')
        self.strprinter = printing.StrPrinter({'full_prec':False})
        self.freevars = None # list of free variables in the solution
        self.freevardependencies = None # list of variables depending on the free variables
        self.functions = dict()
        self.iktypestr=iktypestr
        self.kinematicshash=kinematicshash
        self.resetequations() # dictionary of symbols already written
        self._globalvariables = {} # a set of global variables already written
        self._solutioncounter = 0
        self.version=version
        self._checkpreemptfn = checkpreemptfn
    
    def resetequations(self):
        self.dictequations = [[],[]]
    def copyequations(self,dictequations=None):
        if dictequations is None:
            dictequations=self.dictequations
        return [copy.copy(dictequations[0]),copy.copy(dictequations[1])]
    
    def generate(self, solvertree):
        code = """/// autogenerated analytical inverse kinematics code from ikfast program part of OpenRAVE
/// \\author Rosen Diankov
///
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///     http://www.apache.org/licenses/LICENSE-2.0
/// 
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.
///
/// ikfast version %s generated on %s
/// Generated using solver %s
/// To compile with gcc:
///     gcc -lstdc++ ik.cpp
/// To compile without any main function as a shared object (might need -llapack):
///     gcc -fPIC -lstdc++ -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -shared -Wl,-soname,libik.so -o libik.so ik.cpp
#define IKFAST_HAS_LIBRARY
#include "ikfast.h" // found inside share/openrave-X.Y/python/ikfast.h
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==%s);

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

#ifndef IKFAST_ASSERT
#include <stdexcept>
#include <sstream>
#include <iostream>

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif

#define IKFAST_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }

#endif

#if defined(_MSC_VER)
#define IKFAST_ALIGNED16(x) __declspec(align(16)) x
#else
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))
#endif

#define IK2PI  ((IkReal)6.28318530717959)
#define IKPI  ((IkReal)3.14159265358979)
#define IKPI_2  ((IkReal)1.57079632679490)

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#ifndef isinf
#define isinf _isinf
#endif
//#ifndef isfinite
//#define isfinite _isfinite
//#endif
#endif // _MSC_VER

// lapack routines
extern "C" {
  void dgetrf_ (const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
  void zgetrf_ (const int* m, const int* n, std::complex<double>* a, const int* lda, int* ipiv, int* info);
  void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info);
  void dgesv_ (const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
  void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
  void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi,double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
}

using namespace std; // necessary to get std math routines

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKsqr(float f) { return f*f; }
inline double IKsqr(double f) { return f*f; }

inline float IKlog(float f) { return logf(f); }
inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)1e-7)
#endif

// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)1e-7)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

// there are checkpoints in ikfast that are evaluated to make sure they are 0. This threshold speicfies by how much they can deviate
#ifndef IKFAST_EVALCOND_THRESH
#define IKFAST_EVALCOND_THRESH ((IkReal)0.03) // 5D IK has some crazy degenerate cases, but can rely on jacobian refinment to make better, just need good starting point
#endif


inline float IKasin(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(-IKPI_2);
else if( f >= 1 ) return float(IKPI_2);
return asinf(f);
}
inline double IKasin(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0) {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline double IKfmod(double x, double y)
{
    while(x < 0) {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(IKPI);
else if( f >= 1 ) return float(0);
return acosf(f);
}
inline double IKacos(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKtan(float f) { return tanf(f); }
inline double IKtan(double f) { return tan(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2Simple(float fy, float fx) {
    return atan2f(fy,fx);
}
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return float(IKPI_2);
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2Simple(double fy, double fx) {
    return atan2(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2(fy,fx);
}

template <typename T>
struct CheckValue
{
    T value;
    bool valid;
};

template <typename T>
inline CheckValue<T> IKatan2WithCheck(T fy, T fx, T epsilon)
{
    CheckValue<T> ret;
    ret.valid = false;
    ret.value = 0;
    if( !isnan(fy) && !isnan(fx) ) {
        if( IKabs(fy) >= IKFAST_ATAN2_MAGTHRESH || IKabs(fx) > IKFAST_ATAN2_MAGTHRESH ) {
            ret.value = IKatan2Simple(fy,fx);
            ret.valid = true;
        }
    }
    return ret;
}

inline float IKsign(float f) {
    if( f > 0 ) {
        return float(1);
    }
    else if( f < 0 ) {
        return float(-1);
    }
    return 0;
}

inline double IKsign(double f) {
    if( f > 0 ) {
        return 1.0;
    }
    else if( f < 0 ) {
        return -1.0;
    }
    return 0;
}

template <typename T>
inline CheckValue<T> IKPowWithIntegerCheck(T f, int n)
{
    CheckValue<T> ret;
    ret.valid = true;
    if( n == 0 ) {
        ret.value = 1.0;
        return ret;
    }
    else if( n == 1 )
    {
        ret.value = f;
        return ret;
    }
    else if( n < 0 )
    {
        if( f == 0 )
        {
            ret.valid = false;
            ret.value = (T)1.0e30;
            return ret;
        }
        if( n == -1 ) {
            ret.value = T(1.0)/f;
            return ret;
        }
    }

    int num = n > 0 ? n : -n;
    if( num == 2 ) {
        ret.value = f*f;
    }
    else if( num == 3 ) {
        ret.value = f*f*f;
    }
    else {
        ret.value = 1.0;
        while(num>0) {
            if( num & 1 ) {
                ret.value *= f;
            }
            num >>= 1;
            f *= f;
        }
    }
    
    if( n < 0 ) {
        ret.value = T(1.0)/ret.value;
    }
    return ret;
}

template <typename T> struct ComplexLess
{
    bool operator()(const complex<T>& lhs, const complex<T>& rhs) const
    {
        if (real(lhs) < real(rhs)) {
            return true;
        }
        if (real(lhs) > real(rhs)) {
            return false;
        }
        return imag(lhs) < imag(rhs);
    }
};

"""%(self.version,str(datetime.datetime.now()),self.iktypestr,self.version)
        code += solvertree.generate(self)
        code += solvertree.end(self)
        
        code += """

/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API bool ComputeIk2(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions, void* pOpenRAVEManip) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API const char* GetKinematicsHash() { return "%s"; }

IKFAST_API const char* GetIkFastVersion() { return "%s"; }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif
"""%(self.kinematicshash, self.version)

        code += """
#ifndef IKFAST_NO_MAIN
#include <stdio.h>
#include <stdlib.h>
#ifdef IKFAST_NAMESPACE
using namespace IKFAST_NAMESPACE;
#endif
int main(int argc, char** argv)
{
    if( argc != 12+GetNumFreeParameters()+1 ) {
        printf("\\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\\n\\n"
               "Returns the ik solutions given the transformation of the end effector specified by\\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\\n"
               "There are %d free parameters that have to be specified.\\n\\n",GetNumFreeParameters());
        return 1;
    }

    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    IkReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\\n");
        return -1;
    }

    printf("Found %d ik solutions:\\n", (int)solutions.GetNumSolutions());
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j)
            printf("%.15f, ", solvalues[j]);
        printf("\\n");
    }
    return 0;
}

#endif
"""
        return code

    def getClassInit(self,node,iktype,userotation=7,usetranslation=7):
        code = "IKFAST_API int GetNumFreeParameters() { return %d; }\n"%len(node.freejointvars)
        if len(node.freejointvars) == 0:
            code += "IKFAST_API int* GetFreeParameters() { return NULL; }\n"
        else:
            code += "IKFAST_API int* GetFreeParameters() { static int freeparams[] = {"
            for i,freejointvar in enumerate(node.freejointvars):
                code += "%d"%(freejointvar[1])
                if i < len(node.freejointvars)-1:
                    code += ", "
            code += "}; return freeparams; }\n"
        code += "IKFAST_API int GetNumJoints() { return %d; }\n\n"%(len(node.freejointvars)+len(node.solvejointvars))
        code += "IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }\n\n"
        code += 'IKFAST_API int GetIkType() { return 0x%x; }\n\n'%iktype
        code += "class IKSolver {\npublic:\n"
        
        usedvars = []
        for var in node.solvejointvars:
            usedvars += [var[0].name,'c'+var[0].name,'s'+var[0].name,'ht'+var[0].name]
            # if the variable becomes free need the multiplier
            usedvars.append('%smul'%var[0].name)
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            usedvars += [name,'c'+name,'s'+name,'ht'+name]

        for i in range(3):
            if userotation & (1<<i):
                for j in range(3):
                    usedvars += ['new_r%d%d'%(i,j), 'r%d%d'%(i,j), 'rxp%d_%d'%(i,j)]
        if usetranslation & 1:
            usedvars += ['new_px', 'px', 'npx']
        if usetranslation & 2:
            usedvars += ['new_py', 'py', 'npy']
        if usetranslation & 4:
            usedvars += ['new_pz', 'pz', 'npz']
        if usetranslation ==7:
            usedvars.append('pp')
        # create any other global variables
        for var, value in node.dictequations:
            if var.is_Symbol and not var.name in usedvars:
                usedvars.append(var.name)
        code += 'IkReal ' + ','.join(usedvars) + ';\n'
        self._globalvariables = set(usedvars)
        self._solutioncounter = 0
        code += 'unsigned char ' + ','.join('_i%s[2], _n%s'%(var[0].name,var[0].name) for var in node.solvejointvars+node.freejointvars) + ';\n\n'
        # special variable
        code += 'IkReal j100, cj100, sj100;\n' # for dummy joints that is sum of real joints
        code += 'unsigned char _ij100[2], _nj100;\n'
        return code

    def GetIkFunctionPreamble(self, node):
        code = "bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {\n"
        for var in node.solvejointvars:
            code += '%s=numeric_limits<IkReal>::quiet_NaN(); _i%s[0] = -1; _i%s[1] = -1; _n%s = -1; '%(var[0].name,var[0].name,var[0].name,var[0].name)
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            code += ' _i%s[0] = -1; _i%s[1] = -1; _n%s = 0; '%(name,name,name)
        code += "\nfor(int dummyiter = 0; dummyiter < 1; ++dummyiter) {\n"
        code += "    solutions.Clear();\n"
        return code

    def getFKFunctionPreamble(self):
        code = "/// solves the forward kinematics equations.\n"
        code += "/// \\param pfree is an array specifying the free joints of the chain.\n"
        code += "IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot) {\n"
        return code
    
    def generateChain(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.resetequations()
        self.symbolgen = cse_main.numbered_symbols('x')

        code = ''
        if node.Tfk:
            code += self.getFKFunctionPreamble()
            allvars = node.solvejointvars + node.freejointvars
            subexprs,reduced_exprs = customcse(node.Tfk[0:3,0:4].subs([(v[0],Symbol('j[%d]'%v[1])) for v in allvars]),self.symbolgen)
            outputnames = ['eerot[0]','eerot[1]','eerot[2]','eetrans[0]','eerot[3]','eerot[4]','eerot[5]','eetrans[1]','eerot[6]','eerot[7]','eerot[8]','eetrans[2]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += fcode
            code += '}\n\n'
        code += self.getClassInit(node,IkType.Transform6D)
        code += self.GetIkFunctionPreamble(node)
        fcode = ''
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]), ht%s=tan(pfree[%d]*0.5);\n'%(name,i,name,i,name,i,name,i)
        for i in range(3):
            for j in range(3):
                fcode += "r%d%d = eerot[%d*3+%d];\n"%(i,j,i,j)
        fcode += "px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];\n\n"
        psymbols = ["new_px","new_py","new_pz"]
        for i in range(3):
            for j in range(3):
                fcode += self.writeEquations(lambda k: "new_r%d%d"%(i,j),node.Tee[4*i+j].evalf())
            fcode += self.writeEquations(lambda k: psymbols[i],node.Tee[4*i+3].evalf())
        for i in range(3):
            for j in range(3):
                fcode += "r%d%d = new_r%d%d; "%(i,j,i,j)
        fcode += "px = new_px; py = new_py; pz = new_pz;\n"
        if node.dictequations is not None:
            # be careful with dictequations since having an equation like atan2(px,py) is invalid and will force the IK to terminate.
            fcode += self.WriteDictEquations(node.dictequations).getvalue()
        fcode += self.generateTree(node.jointtree)
        code += fcode + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"

        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += functioncode
        code += "};\n"
        return code
    def endChain(self, node):
        return ""

    def generateIKChainRotation3D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.resetequations()
        self.symbolgen = cse_main.numbered_symbols('x')

        code = ''
        if node.Rfk:
            code += self.getFKFunctionPreamble()
            allvars = node.solvejointvars + node.freejointvars
            subexprs,reduced_exprs = customcse(node.Rfk[0:3,0:3].subs([(v[0],Symbol('j[%d]'%v[1])) for v in allvars]),self.symbolgen)
            outputnames = ['eerot[0]','eerot[1]','eerot[2]','eerot[3]','eerot[4]','eerot[5]','eerot[6]','eerot[7]','eerot[8]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += fcode
            code += '}\n\n'

        code += self.getClassInit(node,IkType.Rotation3D,usetranslation=0)
        code += self.GetIkFunctionPreamble(node)
        fcode = ''
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]);\n'%(name,i,name,i,name,i)
        for i in range(3):
            for j in range(3):
                fcode += "r%d%d = eerot[%d*3+%d];\n"%(i,j,i,j)
        
        for i in range(3):
            for j in range(3):
                fcode += self.writeEquations(lambda k: "new_r%d%d"%(i,j),node.Ree[i,j].evalf())
        for i in range(3):
            for j in range(3):
                fcode += "r%d%d = new_r%d%d; "%(i,j,i,j)
        fcode += '\n'
        if node.dictequations is not None:
            fcode += self.WriteDictEquations(node.dictequations).getvalue()
        fcode += self.generateTree(node.jointtree)
        code += fcode + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"
        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += functioncode
        code += "};\n"
        return code
    def endIKChainRotation3D(self, node):
        return ""

    def generateIKChainTranslation3D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.resetequations()
        self.symbolgen = cse_main.numbered_symbols('x')

        code = ''
        if node.Pfk:
            code += self.getFKFunctionPreamble()
            allvars = node.solvejointvars + node.freejointvars
            allsubs = [(v[0],Symbol('j[%d]'%v[1])) for v in allvars]
            eqs = []
            for eq in node.Pfk[0:3]:
                eqs.append(eq.subs(allsubs))
            subexprs,reduced_exprs = customcse(eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]','eetrans[2]']
            if node.uselocaltrans:
                fcode = """
// necessary for local/global translation3d
eerot[0] = eerot[4] = eerot[8] = 0;
IkReal r00 = 0, r11 = 0, r22 = 0;
"""
            else:
                fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode += 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += fcode
            code += '}\n\n'

        if node.uselocaltrans:
            code += self.getClassInit(node,IkType.TranslationLocalGlobal6D,userotation=7)
        else:
            code += self.getClassInit(node,IkType.Translation3D,userotation=0)
        code += self.GetIkFunctionPreamble(node)
        fcode = ''
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]);\n'%(name,i,name,i,name,i)
        if node.uselocaltrans:
            for i in range(3):
                fcode += "r%d%d = eerot[%d];\n"%(i,i,4*i)
        fcode += "px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];\n\n"

        psymbols = ["new_px","new_py","new_pz"]
        for i in range(3):
            fcode += self.writeEquations(lambda k: psymbols[i],node.Pee[i].evalf())
        fcode += "px = new_px; py = new_py; pz = new_pz;\n"
        if node.dictequations is not None:
            fcode += self.WriteDictEquations(node.dictequations).getvalue()
        fcode += self.generateTree(node.jointtree)
        code += fcode + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"
        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += functioncode
        code += "};\n"
        return code
    def endIKChainTranslation3D(self, node):
        return ""

    def generateIKChainTranslationXY2D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.resetequations()
        self.symbolgen = cse_main.numbered_symbols('x')

        code = ''
        if node.Pfk:
            code += self.getFKFunctionPreamble()
            allvars = node.solvejointvars + node.freejointvars
            allsubs = [(v[0],Symbol('j[%d]'%v[1])) for v in allvars]
            eqs = []
            for eq in node.Pfk[0:2]:
                eqs.append(eq.subs(allsubs))
            subexprs,reduced_exprs = customcse(eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += fcode
            code += '}\n\n'

        code += self.getClassInit(node,IkType.TranslationXY2D,userotation=0,usetranslation=3)
        code += self.GetIkFunctionPreamble(node)
        fcode = ''
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]);\n'%(name,i,name,i,name,i)
        fcode += "px = eetrans[0]; py = eetrans[1];\n\n"

        psymbols = ["new_px","new_py"]
        for i in range(2):
            fcode += self.writeEquations(lambda k: psymbols[i],node.Pee[i].evalf())
        fcode += "px = new_px; py = new_py;\n"
        if node.dictequations is not None:
            fcode += self.WriteDictEquations(node.dictequations).getvalue()
        fcode += self.generateTree(node.jointtree)
        code += fcode + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"
        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += functioncode
        code += "};\n"
        return code
    def endIKChainTranslationXY2D(self, node):
        return ""

    def generateIKChainDirection3D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.resetequations()
        self.symbolgen = cse_main.numbered_symbols('x')
        
        code = ''
        if node.Dfk:
            code += self.getFKFunctionPreamble()
            allvars = node.solvejointvars + node.freejointvars
            allsubs = [(v[0],Symbol('j[%d]'%v[1])) for v in allvars]
            eqs = []
            for eq in node.Dfk:
                eqs.append(eq.subs(allsubs))
            subexprs,reduced_exprs = customcse(eqs,self.symbolgen)
            outputnames = ['eerot[0]','eerot[1]','eerot[2]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += fcode
            code += '}\n\n'

        code += self.getClassInit(node,IkType.Direction3D,userotation=1,usetranslation=0)
        code += self.GetIkFunctionPreamble(node)
        fcode = ''
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]);\n'%(name,i,name,i,name,i)
        for i in range(3):
            fcode += "r0%d = eerot[%d];\n"%(i,i)

        for i in range(3):
            fcode += self.writeEquations(lambda k: "new_r%d%d"%(0,i),node.Dee[i].evalf())
        for i in range(3):
            fcode += "r0%d = new_r0%d; "%(i,i)
        if node.dictequations is not None:
            fcode += self.WriteDictEquations(node.dictequations).getvalue()
        fcode += self.generateTree(node.jointtree)
        code += fcode + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"

        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += functioncode
        code += "};\n"
        return code
    def endIKChainDirection3D(self, node):
        return ''

    def generateIKChainRay(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.resetequations()
        self.symbolgen = cse_main.numbered_symbols('x')
        
        code = ''
        if node.Dfk and node.Pfk:
            code += self.getFKFunctionPreamble()
            allvars = node.solvejointvars + node.freejointvars
            allsubs = [(v[0],Symbol('j[%d]'%v[1])) for v in allvars]
            eqs = []
            for eq in node.Pfk[0:3]:
                eqs.append(eq.subs(allsubs))
            for eq in node.Dfk[0:3]:
                eqs.append(eq.subs(allsubs))
            subexprs,reduced_exprs = customcse(eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]','eetrans[2]','eerot[0]','eerot[1]','eerot[2]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += fcode
            code += '}\n\n'

        code += self.getClassInit(node,IkType.TranslationDirection5D if node.is5dray else IkType.Ray4D,userotation=1)
        code += self.GetIkFunctionPreamble(node)
        fcode = "px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];\n\n"
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]);\n'%(name,i,name,i,name,i)
        for i in range(3):
            fcode += "r0%d = eerot[%d];\n"%(i,i)
        fcode += "px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];\n"

        psymbols = ["new_px","new_py","new_pz"]
        for i in range(3):
            fcode += self.writeEquations(lambda k: "new_r%d%d"%(0,i),node.Dee[i].evalf())
            fcode += self.writeEquations(lambda k: psymbols[i],node.Pee[i].evalf())
        for i in range(3):
            fcode += "r0%d = new_r0%d; "%(i,i)
        if node.is5dray:
            fcode += "px = new_px; py = new_py; pz = new_pz;\n\n"
        else:
            fcode += "\nIkReal new_pdotd = new_px*new_r00+new_py*new_r01+new_pz*new_r02;\n"
            fcode += "px = new_px-new_pdotd * new_r00; py = new_py- new_pdotd * new_r01; pz = new_pz - new_pdotd * new_r02;\n\n"
        if node.dictequations is not None:
            fcode += self.WriteDictEquations(node.dictequations).getvalue()
        fcode += self.generateTree(node.jointtree)
        code += fcode + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"

        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += functioncode
        code += "};\n"
        return code
    def endIKChainRay(self, node):
        return ''

    def generateIKChainLookat3D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.resetequations()
        self.symbolgen = cse_main.numbered_symbols('x')

        code = ''
        if node.Dfk and node.Pfk:
            code += self.getFKFunctionPreamble()
            allvars = node.solvejointvars + node.freejointvars
            allsubs = [(v[0],Symbol('j[%d]'%v[1])) for v in allvars]
            eqs = []
            for eq in node.Pfk[0:3]:
                eqs.append(eq.subs(allsubs))
            for eq in node.Dfk[0:3]:
                eqs.append(eq.subs(allsubs))
            subexprs,reduced_exprs = customcse(eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]','eetrans[2]','eerot[0]','eerot[1]','eerot[2]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += fcode
            code += '}\n\n'

        code += self.getClassInit(node,IkType.Lookat3D,userotation=0)
        code += self.GetIkFunctionPreamble(node)
        fcode = "px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];\n\n"
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]);\n'%(name,i,name,i,name,i)

        psymbols = ["new_px","new_py","new_pz"]
        for i in range(3):
            fcode += self.writeEquations(lambda k: psymbols[i],node.Pee[i].evalf())
        fcode += "px = new_px; py = new_py; pz = new_pz;\n"
        if node.dictequations is not None:
            fcode += self.WriteDictEquations(node.dictequations).getvalue()
        fcode += self.generateTree(node.jointtree)
        code += fcode + "}\nreturn solutions.GetNumSolutions()>0;\n}\n\n"
        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += functioncode
        code += "};\n"
        return code
    def endIKChainLookat3D(self, node):
        return ''

    def generateSolverIKChainAxisAngle(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.resetequations()
        self.symbolgen = cse_main.numbered_symbols('x')
        
        code = ''
        if node.anglefk and node.Pfk:
            code += self.getFKFunctionPreamble()
            code += "for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {\n"
            allvars = node.solvejointvars + node.freejointvars
            allsubs = [(v[0],Symbol('j[%d]'%v[1])) for v in allvars]
            eqs = []
            for eq in node.Pfk[0:3]:
                eqs.append(eq.subs(allsubs))
            eqs.append(node.anglefk.subs(allsubs))
            subexprs,reduced_exprs = customcse(eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]','eetrans[2]','eerot[0]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += fcode
            # if gets to the end of the for loop statement, return successfully. if exits for loop, assert(0)
            code += 'return;\n}\nIKFAST_ASSERT(0);\n}\n\n'

        code += self.getClassInit(node,node.iktype,userotation=1)
        code += self.GetIkFunctionPreamble(node)
        fcode = "px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];\n\n"
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]);\n'%(name,i,name,i,name,i)
        fcode += "r00 = eerot[0];\n"
        fcode += "px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];\n"

        psymbols = ["new_px","new_py","new_pz"]
        for i in range(3):
            fcode += self.writeEquations(lambda k: psymbols[i],node.Pee[i].evalf())
        fcode += self.writeEquations(lambda k: "new_r00",node.angleee.evalf())
        fcode += "r00 = new_r00; "
        fcode += "px = new_px; py = new_py; pz = new_pz;\n\n"
        if node.dictequations is not None:
            fcode += self.WriteDictEquations(node.dictequations).getvalue()
        fcode += self.generateTree(node.jointtree)
        code += fcode + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"

        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += functioncode
        code += "};\n"
        return code
    def endSolverIKChainAxisAngle(self, node):
        return ''

    def generateSolution(self, node,declarearray=True,acceptfreevars=True):
        """writes the solution of one variable
        :param declarearray: if False, will return the equations to be written without evaluating them. Used for conditioned solutions.
        """
        code = cStringIO.StringIO()
        numsolutions = 0
        eqcode = cStringIO.StringIO()
        name = node.jointname
        self._solutioncounter += 1
        log.info('c=%d var=%s', self._solutioncounter, name)
        node.HasFreeVar = False
        allnumsolutions = 0
        #log.info('generateSolution %s (%d)', name, len(node.dictequations))
        self.WriteDictEquations(node.dictequations, code=eqcode)
#         for var,value in node.dictequations:
#             eqcode.write('IkReal %s;\n'%var)
#             self.WriteEquations2(lambda k: var,value,code=eqcode)
            
        if node.jointeval is not None:
            numsolutions = len(node.jointeval)
            equations = []
            names = []
            for i,expr in enumerate(node.jointeval):
                if acceptfreevars and self.freevars is not None:
                    m = None
                    for freevar in self.freevars:
                        if expr.has(Symbol(freevar)):
                            # has free variables, so have to look for a*freevar+b form
                            a = Wild('a',exclude=[Symbol(freevar)])
                            b = Wild('b',exclude=[Symbol(freevar)])
                            m = expr.match(a*Symbol(freevar)+b)
                            if m is not None:
                                self.freevardependencies.append((freevar,name))
                                assert(len(node.jointeval)==1)
                                self.WriteEquations2(lambda i: '%smul'%name, m[a], code=code)
                                self.WriteEquations2(lambda i: name, m[b],code=code)
                                node.HasFreeVar = True
                                return code.getvalue()
                            else:
                                log.error('failed to extract free variable %s for %s from: eq=%s', freevar,node.jointname, expr)
    #                             m = dict()
    #                             m[a] = Float(-1,30)
    #                             m[b] = Float(0,30)

                equations.append(expr)
                names.append('%sarray[%d]'%(name,allnumsolutions+i))
                equations.append(sin(Symbol('%sarray[%d]'%(name,allnumsolutions+i))))
                names.append('s%sarray[%d]'%(name,allnumsolutions+i))
                equations.append(cos(Symbol('%sarray[%d]'%(name,allnumsolutions+i))))
                names.append('c%sarray[%d]'%(name,allnumsolutions+i))
            self.WriteEquations2(lambda i: names[i], equations,code=eqcode)
            if node.AddPiIfNegativeEq:
                for i in range(numsolutions):
                    eqcode.write('%sarray[%d] = %sarray[%d] > 0 ? %sarray[%d]-IKPI : %sarray[%d]+IKPI;\n'%(name,allnumsolutions+numsolutions+i,name,allnumsolutions+i,name,allnumsolutions+i,name,allnumsolutions+i))
                    eqcode.write('s%sarray[%d] = -s%sarray[%d];\n'%(name,allnumsolutions+numsolutions+i,name,allnumsolutions+i))
                    eqcode.write('c%sarray[%d] = -c%sarray[%d];\n'%(name,allnumsolutions+numsolutions+i,name,allnumsolutions+i))
                numsolutions *= 2
            for i in range(numsolutions):
                if node.isHinge:
                    eqcode.write('if( %sarray[%d] > IKPI )\n{\n    %sarray[%d]-=IK2PI;\n}\nelse if( %sarray[%d] < -IKPI )\n{    %sarray[%d]+=IK2PI;\n}\n'%(name,allnumsolutions+i,name,allnumsolutions+i,name,allnumsolutions+i,name,allnumsolutions+i))
                eqcode.write('%svalid[%d] = true;\n'%(name,allnumsolutions+i))
            allnumsolutions += numsolutions
        # might also have cos solutions ...
        if node.jointevalcos is not None:
            numsolutions = 2*len(node.jointevalcos)
            self.WriteEquations2(lambda i: 'c%sarray[%d]'%(name,allnumsolutions+2*i),node.jointevalcos,code=eqcode)
            for i in range(len(node.jointevalcos)):
                eqcode.write('if( c%sarray[%d] >= -1-IKFAST_SINCOS_THRESH && c%sarray[%d] <= 1+IKFAST_SINCOS_THRESH )\n{\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                eqcode.write('    %svalid[%d] = %svalid[%d] = true;\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i+1))
                eqcode.write('    %sarray[%d] = IKacos(c%sarray[%d]);\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                eqcode.write('    s%sarray[%d] = IKsin(%sarray[%d]);\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                # second solution
                eqcode.write('    c%sarray[%d] = c%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i))
                eqcode.write('    %sarray[%d] = -%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i))
                eqcode.write('    s%sarray[%d] = -s%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i))
                eqcode.write('}\n')
                eqcode.write('else if( isnan(c%sarray[%d]) )\n{\n'%(name,allnumsolutions+2*i))
                eqcode.write('    // probably any value will work\n')
                eqcode.write('    %svalid[%d] = true;\n'%(name,allnumsolutions+2*i))
                eqcode.write('    c%sarray[%d] = 1; s%sarray[%d] = 0; %sarray[%d] = 0;\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                eqcode.write('}\n')
            allnumsolutions += numsolutions

        if node.jointevalsin is not None:
            numsolutions = 2*len(node.jointevalsin)
            self.WriteEquations2(lambda i: 's%sarray[%d]'%(name,allnumsolutions+2*i),node.jointevalsin,code=eqcode)
            for i in range(len(node.jointevalsin)):
                eqcode.write('if( s%sarray[%d] >= -1-IKFAST_SINCOS_THRESH && s%sarray[%d] <= 1+IKFAST_SINCOS_THRESH )\n{\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                eqcode.write('    %svalid[%d] = %svalid[%d] = true;\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i+1))
                eqcode.write('    %sarray[%d] = IKasin(s%sarray[%d]);\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                eqcode.write('    c%sarray[%d] = IKcos(%sarray[%d]);\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                # second solution
                eqcode.write('    s%sarray[%d] = s%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i))
                eqcode.write('    %sarray[%d] = %sarray[%d] > 0 ? (IKPI-%sarray[%d]) : (-IKPI-%sarray[%d]);\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i,name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                eqcode.write('    c%sarray[%d] = -c%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i))
                eqcode.write('}\n')
                eqcode.write('else if( isnan(s%sarray[%d]) )\n{\n'%(name,allnumsolutions+2*i))
                eqcode.write('    // probably any value will work\n')
                eqcode.write('    %svalid[%d] = true;\n'%(name,allnumsolutions+2*i))
                eqcode.write('    c%sarray[%d] = 1; s%sarray[%d] = 0; %sarray[%d] = 0;\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i,name,allnumsolutions+2*i))
                eqcode.write('}\n')
            allnumsolutions += numsolutions

        if not declarearray:
            return eqcode.getvalue(),allnumsolutions

        code.write('{\nIkReal %sarray[%d], c%sarray[%d], s%sarray[%d];\n'%(name,allnumsolutions,name,allnumsolutions,name,allnumsolutions))
        code.write('bool %svalid[%d]={false};\n'%(name,allnumsolutions))
        code.write('_n%s = %d;\n'%(name,allnumsolutions))
        code.write(eqcode.getvalue())
        if allnumsolutions > 1:
            if allnumsolutions >= 256:
                log.error('num solutions is %d>=256, which exceeds unsigned char',allnumsolutions)
        code.write('for(int i%s = 0; i%s < %d; ++i%s)\n{\n'%(name,name,allnumsolutions,name))
        code.write('if( !%svalid[i%s] )\n{\n    continue;\n}\n'%(name,name))
        code.write('_i%s[0] = i%s; _i%s[1] = -1;\n'%(name,name,name))
        # check for a similar solution
        code.write('for(int ii%s = i%s+1; ii%s < %d; ++ii%s)\n{\n'%(name,name,name,allnumsolutions,name))
        code.write('if( %svalid[ii%s] && IKabs(c%sarray[i%s]-c%sarray[ii%s]) < IKFAST_SOLUTION_THRESH && IKabs(s%sarray[i%s]-s%sarray[ii%s]) < IKFAST_SOLUTION_THRESH )\n{\n    %svalid[ii%s]=false; _i%s[1] = ii%s; break; \n}\n'%(name,name,name,name,name,name,name,name,name,name,name,name,name,name))
        code.write('}\n')
        code.write('%s = %sarray[i%s]; c%s = c%sarray[i%s]; s%s = s%sarray[i%s];\n'%(name,name,name,name,name,name,name,name,name))
        if node.AddHalfTanValue:
            code.write('ht%s = IKtan(%s/2);\n'%(name,name))
        if node.getEquationsUsed() is not None and len(node.getEquationsUsed()) > 0:
            code.write('{\nIkReal evalcond[%d];\n'%len(node.getEquationsUsed()))
            self.WriteEquations2(lambda i: 'evalcond[%d]'%(i),node.getEquationsUsed(),code=code)
            code.write('if( ')
            for i in range(len(node.getEquationsUsed())):
                if i != 0:
                    code.write(' || ')
                # using smaller node.thresh increases the missing solution rate, really not sure whether the solutions themselves
                # are bad due to double precision arithmetic, or due to singularities
                code.write('IKabs(evalcond[%d]) > IKFAST_EVALCOND_THRESH '%(i))
            code.write(' )\n{\n')
            #code += 'cout <<'
            #code += '<<'.join(['evalcond[%d]'%i for i in range(len(node.getEquationsUsed()))])
            #code += '<< endl;'
            code.write('continue;\n}\n')
            code.write('}\n')
        code.write('\n')
        return code.getvalue()

    def endSolution(self, node):
        if node.HasFreeVar:
            self.freevardependencies.pop()
            return ''
        return '}\n}\n'

    def generateConditionedSolution(self, node):
        name=node.solversolutions[0].jointname
        assert all([name == s.jointname for s in node.solversolutions])
        origequations = self.copyequations()
        maxchecks = max([len(s.checkforzeros) for s in node.solversolutions])
        allnumsolutions = 0
        AddHalfTanValue = False
        checkcode = ''
        checkcode += self.WriteDictEquations(node.dictequations).getvalue()
        for solversolution in node.solversolutions:
            if len(solversolution.checkforzeros) > 0:
                if solversolution.AddHalfTanValue:
                    AddHalfTanValue = True
                self.dictequations = self.copyequations(origequations)
                checkcode += '{\n'
                checkcode += self.writeEquations(lambda i: 'evalcond[%d]'%(i),solversolution.checkforzeros)
                checkcode += 'if( '
                for i in range(len(solversolution.checkforzeros)):
                    if i != 0:
                        checkcode += ' && '
                    checkcode += 'IKabs(evalcond[%d]) %s %.16f '%(i,'<=' if solversolution.FeasibleIsZeros else '>',node.thresh)
                checkcode += ' )\n'
            checkcode += '{\n'
            scode, numsolutions = self.generateSolution(solversolution,declarearray=False,acceptfreevars=False)
            scode += 'numsolutions%s = %d;\n'%(name,numsolutions)
            allnumsolutions = max(allnumsolutions,numsolutions)
            checkcode += scode
            if len(solversolution.checkforzeros) == 0:
                # can never go to the other clauses anyway...
                checkcode += '\n}\n'
                break
            checkcode += '\n} else\n'
        checkcode += '{\n    continue;\n}\n'  # if got here, then current solution branch is not good, so skip
        checkcode += '}\n'*len(node.solversolutions)
        checkcode += 'if( numsolutions%s == 0 )\n{\n    continue;\n}\n'%name
        
        code = '{\nIkReal evalcond[%d]; int numsolutions%s = 0;\n'%(maxchecks,name)
        code += 'IkReal %sarray[%d], c%sarray[%d], s%sarray[%d];\n'%(name,allnumsolutions,name,allnumsolutions,name,allnumsolutions)
        code += 'bool %svalid[%d]={false};\n'%(name,allnumsolutions)
        code += '_n%s = %d;\n'%(name,allnumsolutions)
        code += checkcode
        if allnumsolutions > 1:
            if allnumsolutions >= 256:
                log.error('num solutions is %d>=256, which exceeds unsigned char',allnumsolutions)
        code += 'for(int i%s = 0; i%s < numsolutions%s; ++i%s)\n{\n'%(name,name,name,name)
        code += 'if( !%svalid[i%s] )\n{\n    continue;\n}\n'%(name,name)
        code += '_i%s[0] = i%s; _i%s[1] = -1;\n'%(name,name,name)
        # check for a similar solution
        code += 'for(int ii%s = i%s+1; ii%s < numsolutions%s; ++ii%s)\n{\n'%(name,name,name,name,name)
        code += 'if( %svalid[ii%s] && IKabs(c%sarray[i%s]-c%sarray[ii%s]) < IKFAST_SOLUTION_THRESH && IKabs(s%sarray[i%s]-s%sarray[ii%s]) < IKFAST_SOLUTION_THRESH )\n{\n    %svalid[ii%s]=false; _i%s[1] = ii%s; break; \n}\n'%(name,name,name,name,name,name,name,name,name,name,name,name,name,name)
        code += '}\n'
        code += '%s = %sarray[i%s]; c%s = c%sarray[i%s]; s%s = s%sarray[i%s];\n\n'%(name,name,name,name,name,name,name,name,name)
        if AddHalfTanValue:
            code += 'ht%s = IKtan(%s/2);\n'%(name,name)
        self.dictequations = origequations
        return code

    def endConditionedSolution(self, node):
        return '}\n}\n'

    def generatePolynomialRoots(self, node):
        D=node.poly.degree(0)
        if D == 0:
            log.warn('polynomial %s is of degree 0!', node.poly)
            return 'continue; // poly is 0\n'
        polyroots=self.using_polyroots(D)
        name = node.jointname
        polyvar = node.poly.gens[0].name
        code = 'IkReal op[%d+1], zeror[%d];\nint numroots;\n'%(D,D)
        numevals = 0
        if node.postcheckforzeros is not None:
            numevals = max(numevals,len(node.postcheckforzeros))
        if node.postcheckfornonzeros is not None:
            numevals = max(numevals,len(node.postcheckfornonzeros))
        if node.postcheckforrange is not None:
            numevals = max(numevals,len(node.postcheckforrange))
        if node.postcheckforNumDenom is not None:
            numevals = max(numevals,len(node.postcheckforNumDenom)*2)
        if numevals > 0:
            code += 'IkReal %sevalpoly[%d];\n'%(name,numevals)
        code += self.WriteDictEquations(node.dictequations).getvalue()
#         for var,value in node.dictequations:
#             code += 'IkReal %s;\n'%var
#             code += self.writeEquations(lambda k: var,value)
        polydict = node.poly.as_dict()
        code += self.writeEquations(lambda i: 'op[%d]'%(i),[polydict.get((i,),S.Zero) for i in range(D,-1,-1)])
        code += "%s(op,zeror,numroots);\n"%(polyroots)
        code += 'IkReal %sarray[%d], c%sarray[%d], s%sarray[%d], temp%sarray[%d];\n'%(name,len(node.jointeval)*D,name,len(node.jointeval)*D,name,len(node.jointeval)*D,name,len(node.jointeval))
        code += 'int numsolutions = 0;\n'
        code += 'for(int i%s = 0; i%s < numroots; ++i%s)\n{\n'%(name,name,name)
        fcode = 'IkReal %s = zeror[i%s];\n'%(polyvar,name)
        origequations = self.copyequations()
        fcode += self.writeEquations(lambda i: 'temp%sarray[%d]'%(name,i), node.jointeval)
        self.dictequations = origequations
        fcode += 'for(int k%s = 0; k%s < %d; ++k%s)\n{\n'%(name,name,len(node.jointeval),name)
        fcode += '%sarray[numsolutions] = temp%sarray[k%s];\n'%(name,name,name)
        if node.isHinge:
            fcode += 'if( %sarray[numsolutions] > IKPI )\n{\n    %sarray[numsolutions]-=IK2PI;\n}\nelse if( %sarray[numsolutions] < -IKPI )\n{\n    %sarray[numsolutions]+=IK2PI;\n}\n'%(name,name,name,name)
        fcode += 's%sarray[numsolutions] = IKsin(%sarray[numsolutions]);\n'%(name,name)
        fcode += 'c%sarray[numsolutions] = IKcos(%sarray[numsolutions]);\n'%(name,name)
        fcode += 'numsolutions++;\n'
        fcode += '}\n'
        code += fcode
        code += '}\n'

        allnumsolutions = D*len(node.jointeval)
        if allnumsolutions >= 256:
            log.error('num solutions is %d>=256, which exceeds unsigned char',allnumsolutions)
        code += 'bool %svalid[%d]={%s};\n'%(name,allnumsolutions,','.join(['true']*allnumsolutions))
        code += '_n%s = %d;\n'%(name,allnumsolutions)
        code += 'for(int i%s = 0; i%s < numsolutions; ++i%s)\n    {\n'%(name,name,name)
        code += 'if( !%svalid[i%s] )\n{\n    continue;\n}\n'%(name,name)
        code += '    %s = %sarray[i%s]; c%s = c%sarray[i%s]; s%s = s%sarray[i%s];\n'%(name,name,name,name,name,name,name,name,name)
        if node.AddHalfTanValue:
            code += 'ht%s = IKtan(%s/2);\n'%(name,name)
        code += '\n'
        if node.postcheckforzeros is not None and len(node.postcheckforzeros) > 0:
            fcode = self.writeEquations(lambda i: '%sevalpoly[%d]'%(name,i),node.postcheckforzeros)
            fcode += 'if( '
            for i in range(len(node.postcheckforzeros)):
                if i != 0:
                    fcode += ' || '
                fcode += 'IKabs(%sevalpoly[%d]) <= %.16f '%(name,i,node.postcheckforzerosThresh)
            fcode += ' )\n{\n    continue;\n}\n'
            code += fcode
        if node.postcheckfornonzeros is not None and len(node.postcheckfornonzeros) > 0:
            fcode = self.writeEquations(lambda i: '%sevalpoly[%d]'%(name,i),node.postcheckfornonzeros)
            fcode += 'if( '
            for i in range(len(node.postcheckfornonzeros)):
                if i != 0:
                    fcode += ' || '
                fcode += 'IKabs(%sevalpoly[%d]) > %.16f '%(name,i,node.postcheckfornonzerosThresh)
            fcode += ' )\n{\n    continue;\n}\n'
            code += fcode
        if node.postcheckforrange is not None and len(node.postcheckforrange) > 0:
            fcode = self.writeEquations(lambda i: '%sevalpoly[%d]'%(name,i),node.postcheckforrange)
            fcode += 'if( '
            for i in range(len(node.postcheckforrange)):
                if i != 0:
                    fcode += ' || '
                fcode += ' (%sevalpoly[%d] <= %.16f || %sevalpoly[%d] > %.16f) '%(name,i,-1.0-node.postcheckforrangeThresh,name,i,1.0+node.postcheckforrangeThresh)
            fcode += ' )\n{\n    continue;\n}\n'
            code += fcode
        if node.postcheckforNumDenom is not None and len(node.postcheckforNumDenom) > 0:
            allequations = []
            for A, B in node.postcheckforNumDenom:
                allequations.append(A)
                allequations.append(B)
            fcode = self.writeEquations(lambda i: '%sevalpoly[%d]'%(name,i),allequations)
            fcode += 'if( '
            for i in range(len(node.postcheckforNumDenom)):
                if i != 0:
                    fcode += ' || '
                fcode += ' (IKabs(%sevalpoly[%d]) <= %.16f && IKabs(%sevalpoly[%d]) > %.16f) '%(name,2*i,node.postcheckforNumDenomThresh,name,2*i+1,node.postcheckforNumDenomThresh)
            fcode += ' )\n{\n    continue;\n}\n'
            code += fcode

        # passed all tests
        code += '_i%s[0] = i%s; _i%s[1] = -1;\n'%(name,name,name)
        # check for a similar solution
        code += 'for(int ii%s = i%s+1; ii%s < numsolutions; ++ii%s)\n{\n'%(name,name,name,name)
        code += 'if( %svalid[ii%s] && IKabs(c%sarray[i%s]-c%sarray[ii%s]) < IKFAST_SOLUTION_THRESH && IKabs(s%sarray[i%s]-s%sarray[ii%s]) < IKFAST_SOLUTION_THRESH )\n{\n    %svalid[ii%s]=false; _i%s[1] = ii%s; break; \n}\n'%(name,name,name,name,name,name,name,name,name,name,name,name,name,name)
        code += '}\n'
        return code
    
    def endPolynomialRoots(self, node):
        return '    }\n'
    
    def generateCoeffFunction(self, node):
        assert(len(node.jointnames) == len(node.jointeval))
        firstname = node.jointnames[0]
        if node.exportfnname == 'solvedialyticpoly16lep':
            fnname=self.using_solvedialyticpoly16lep()
        elif node.exportfnname == 'solvedialyticpoly12qep':
            fnname=self.using_solvedialyticpoly12qep()
        elif node.exportfnname == 'solvedialyticpoly8qep':
            fnname=self.using_solvedialyticpoly8qep()
        else:
            fnname = 'unknownfn'
        code = cStringIO.StringIO()
        code.write('IkReal op[%d], zeror[%d];\nint numroots;\n'%(len(node.exportcoeffeqs),node.rootmaxdim*len(node.jointnames)))
#         for var,value in node.dictequations:
#             code.write('IkReal %s,'%var)
        code.seek(code.tell()-1) # backtrack the comma
        code.write(';\n')
        #code.write(self.writeEquations(lambda k: node.dictequations[k][0],[eq for name,eq in node.dictequations]))
        self.WriteDictEquations(node.dictequations, code)
        code.write(self.writeEquations(lambda i: 'op[%d]'%(i),node.exportcoeffeqs))
        code.write("%s(op,zeror,numroots);\n"%(fnname))
        code.write('IkReal ')
        for i,name in enumerate(node.jointnames):
            code.write('%sarray[%d], c%sarray[%d], s%sarray[%d]'%(name,node.rootmaxdim,name,node.rootmaxdim,name,node.rootmaxdim))
            if i+1 < len(node.jointnames):
                code.write(', ')
            else:
                code.write(';\n')
        code.write('int numsolutions = 0;\n')
        code.write('for(int i%s = 0; i%s < numroots; i%s += %d)\n{\n'%(firstname,firstname,firstname,len(node.jointnames)))
        fcode = 'IkReal '
        for i in range(len(node.exportvar)):
            fcode += '%s = zeror[i%s+%d]'%(node.exportvar[i],firstname,i)
            if i+1<len(node.exportvar):
                fcode += ', '
            else:
                fcode += ';\n'
        # if NaN, exit
        fcode += 'if(%s){\ncontinue;\n}\n'%('||'.join('isnan(%s)'%exportvar for exportvar in node.exportvar))
        origequations = self.copyequations()
        fcode += self.writeEquations(lambda i: '%sarray[numsolutions]'%(node.jointnames[i]), node.jointeval)
        # if Inf, use sin/cos rather than the pre-specified equations
        for ivar,exportvar in enumerate(node.exportvar):
            fcode += 'if(isinf(%(htvar)s)){\nc%(var)sarray[numsolutions] = IKcos(%(var)sarray[numsolutions]);\ns%(var)sarray[numsolutions] = IKsin(%(var)sarray[numsolutions]);\n}\nelse{\n'%{'htvar':exportvar, 'var':node.jointnames[ivar]}
            fcode += self.writeEquations(lambda i: 'c%sarray[numsolutions]'%(node.jointnames[ivar]), node.jointevalcos[ivar:(ivar+1)])
            fcode += self.writeEquations(lambda i: 's%sarray[numsolutions]'%(node.jointnames[ivar]), node.jointevalsin[ivar:(ivar+1)])
            fcode += '}\n'
        self.dictequations = origequations
        for i in range(len(node.jointnames)):
            if node.isHinges[i]:
                fcode += 'if( %sarray[numsolutions] > IKPI )\n{\n    %sarray[numsolutions]-=IK2PI;\n}\nelse if( %sarray[numsolutions] < -IKPI )\n{\n    %sarray[numsolutions]+=IK2PI;\n}\n'%(node.jointnames[i],node.jointnames[i],node.jointnames[i],node.jointnames[i])
        fcode += 'numsolutions++;\n'
#         fcode += 'bool valid = true;\n'
#         # test all the solutions up to now for validity
#         fcode += 'for( int k%s = 0; k%s < numsolutions; ++k%s)\n{\n'%(firstname,firstname,firstname)
#         fcode += '    if( '
#         for name in node.jointnames:
#             fcode += 'IKabs(c%sarray[k%s]-c%sarray[numsolutions]) < IKFAST_SOLUTION_THRESH && IKabs(s%sarray[k%s]-s%sarray[numsolutions]) < IKFAST_SOLUTION_THRESH &&'%(name,firstname,name,name, firstname,name)
#         fcode += ' 1 )\n    {\n        valid=false; break;\n    }\n'
#         fcode += '}\n'
#         fcode += 'if( valid ) { numsolutions++; }\n'
        fcode += '}\n'
        code.write(fcode)
        
        code.write('bool %svalid[%d]={%s};\n'%(firstname,node.rootmaxdim,','.join(['true']*node.rootmaxdim)))
        code.write('_n%s = %d;\n'%(firstname,node.rootmaxdim))
        for name in node.jointnames[1:]:
            code.write('_n%s = 1;\n'%name)
        if node.rootmaxdim >= 256:
            log.error('num solutions is %d>=256, which exceeds unsigned char',node.rootmaxdim)
        
        code.write('for(int i%s = 0; i%s < numsolutions; ++i%s)\n    {\n'%(firstname,firstname,firstname))
        code.write('if( !%svalid[i%s] )\n{\n    continue;\n}\n'%(firstname,firstname))
        code.write('_i%s[0] = i%s; _i%s[1] = -1;\n'%(firstname,firstname,firstname))
        for name in node.jointnames[1:]:
            code.write('_i%s[0] = 0; _i%s[1] = -1;\n'%(name,name))
            
        # check for a similar solution
        code.write('for(int ii%s = i%s+1; ii%s < numsolutions; ++ii%s)\n{\n'%(firstname,firstname,firstname,firstname))
        code.write('if( !%svalid[ii%s] ) { continue; }\n'%(firstname,firstname))
        code.write('if( ')
        for name in node.jointnames:
            code.write('IKabs(c%sarray[i%s]-c%sarray[ii%s]) < IKFAST_SOLUTION_THRESH && IKabs(s%sarray[i%s]-s%sarray[ii%s]) < IKFAST_SOLUTION_THRESH && '%(name,firstname,name,firstname,name,firstname,name,firstname))
        code.write(' 1 )\n{\n    %svalid[ii%s]=false; '%(firstname,firstname))
        code.write('_i%s[1] = ii%s; '%(firstname,firstname))
        for name in node.jointnames[1:]:
            code.write('_i%s[1] = 0; '%name)
        code.write(' break; \n}\n')
        code.write('}\n')           
        for name in node.jointnames:
            code.write('    %s = %sarray[i%s]; c%s = c%sarray[i%s]; s%s = s%sarray[i%s];\n\n'%(name,name,firstname,name,name,firstname,name,name,firstname))
        log.info('end generateCoeffFunction')
        return code.getvalue()
                   
    def endCoeffFunction(self, node):
        return '    }\n'
    
    def generateMatrixInverse(self, node):
        # lapack takes matrices in column order
        assert( node.A.shape[0] == node.A.shape[1] )
        matrixinverse=self.using_matrixinverse()
        code = ''
        # for some reason things work even if the determinant is 0....
#         if len(node.checkforzeros) > 0:
#             code = 'IkReal matrixcondition[%d];\n'%(len(node.checkforzeros))
#             code += self.writeEquations(lambda i: 'matrixcondition[%d]'%(i),node.checkforzeros)
#             code += 'if( '
#             for i in range(len(node.checkforzeros)):
#                 if i != 0:
#                     code += ' || '
#                 code += 'IKabs(matrixcondition[%d]) < 1e-14 '%(i)
#             code += ' )\n{  continue;\n}\n'
        code += 'IkReal IKFAST_ALIGNED16(matrixinvcoeffs[%d]);\n'%(node.A.shape[0]*node.A.shape[1])
        code += self.writeEquations(lambda i: 'matrixinvcoeffs[%d]'%(i),node.A.transpose()[:])
        code += 'if( !%s<%d>(matrixinvcoeffs) ) {\ncontinue;\n}\n'%(matrixinverse,node.A.shape[0]);
        # create the variables
        mcode = ''
        for i in range(len(node.Asymbols)):
            for j in range(len(node.Asymbols[i])):
                if node.Asymbols[i][j] is not None:
                    if len(mcode) > 0:
                        mcode += ', '
                    else:
                        mcode = 'IkReal '
                    mcode += '%s=matrixinvcoeffs[%d]'%(node.Asymbols[i][j],i+j*node.A.shape[0])
        if len(mcode)> 0:
            code += mcode + ';\n'
        return code
    def endMatrixInverse(self,node):
        return ''

    def generateBranchConds(self, node):
        #log.info('generateBranchConds(%d)', len(node.jointbranches))
        origequations = self.copyequations()
        code = cStringIO.StringIO()
        code.write('{\n')
        numevals = None
        for checkzeroequations, branch, dictequations in node.jointbranches:
            if checkzeroequations is not None:
                if numevals is None or numevals < len(checkzeroequations):
                    numevals=len(checkzeroequations)
        if numevals is not None:
            code.write('IkReal evalcond[%d];\n'%numevals)
        for checkzeroequations, branch, extradictequations in node.jointbranches:
            #log.info('generateBranchConds(%d) %s', len(node.jointbranches), checkzeroequations)
            self.dictequations = self.copyequations(origequations)
            
            # writing the equations for the branch could force the system to call "continue" if out-of-bounds computations are detected.
            # therefore surround each different branch in a do/while statement
            code.write('bool bgotonextstatement = true;\n')
            code.write('do\n{\n')
            self.WriteDictEquations(extradictequations, code)
#             for var,value in extradictequations:
#                 code.write('IkReal %s;\n'%var)
#                 self.WriteEquations2(lambda k: var, value, code)
                
            if checkzeroequations is None:
                code.write('if( 1 )\n')
            else:
                self.WriteEquations2(lambda x: 'evalcond[%d]'%x, checkzeroequations, code)
                code.write('if( ')
                for i in range(len(checkzeroequations)):
                    if i != 0:
                        code.write(' && ')
                    code.write('IKabs(evalcond[%d]) < %.16f '%(i,node.thresh))
                code.write(' )\n')
            code.write('{\n')
            code.write('bgotonextstatement=false;\n')
            for n in branch:
                code.write(n.generate(self))
            for n in reversed(branch):
                code.write(n.end(self))
            code.write('\n}\n')
            code.write('} while(0);\n')
            code.write('if( bgotonextstatement )\n{\n')
        code.write('}\n'*(len(node.jointbranches)+1))
        self.dictequations = origequations
        return code.getvalue()
    
    def endBranchConds(self, node):
        return ''
    
    def generateCheckZeros(self, node):
        origequations = self.copyequations()
        name = node.jointname if node.jointname is not None else 'dummy'
        code = cStringIO.StringIO()
        code.write('{\n')
        code.write('IkReal %seval[%d];\n'%(name,len(node.jointcheckeqs)))
#         for var,value in node.dictequations:
#             code.write('IkReal %s;\n'%var)
#             self.WriteEquations2(lambda k: var,value,code=code)
        self.WriteDictEquations(node.dictequations, code)
        self.WriteEquations2(lambda i: '%seval[%d]'%(name,i),node.jointcheckeqs, code=code)
        if len(node.jointcheckeqs) > 0:
            code.write('if( ')
            for i in range(len(node.jointcheckeqs)):
                if i != 0:
                    if node.anycondition:
                        code.write(' || ')
                    else:
                        code.write(' && ')
                code.write('IKabs(%seval[%d]) < %.16f '%(name,i,node.thresh))
            code.write(' )\n{\n')
            self.dictequations = self.copyequations(origequations)
            code.write(self.generateTree(node.zerobranch))
            code.write('\n} else\n')
        code.write('{\n')
        self.dictequations = self.copyequations(origequations)
        code.write(self.generateTree(node.nonzerobranch))
        code.write('\n}\n')
        self.dictequations = origequations
        code.write('\n}\n')
        return code.getvalue()
    
    def endCheckZeros(self, node):
        return ''
    def generateFreeParameter(self, node):
        #'free variable ',node.jointname,': ',self.freevars
        self.freevars.append(node.jointname)
        self.freevardependencies.append((node.jointname,node.jointname))
        code = 'IkReal %smul = 1;\n%s=0;\n'%(node.jointname,node.jointname)
        return code+self.generateTree(node.jointtree)
    def endFreeParameter(self, node):
        self.freevars.pop()
        self.freevardependencies.pop()
        return ''
    def generateBreak(self,node):
        return 'continue; // %s\n'%node.comment
    def endBreak(self,node):
        return ''

    def generateFunction(self, node):
        if not node.name in self.functions:
            code = 'inline void %s(IkSolutionListBase<IkReal>& solutions) {\n'%(node.name)
            code += 'for(int fniter = 0; fniter < 1; ++fniter) {\n'
            origequations = self.dictequations
            self.resetequations()
            code += self.generateTree(node.jointtree)
            code += '}\n}'
            self.dictequations = origequations
            self.functions[node.name] = code
        return '%s(solutions);\n'%(node.name)
    
    def endFunction(self, node):
        return ''

    def generateRotation(self, node):
        if not node.functionid in self.functions:
            code = 'inline void rotationfunction%d(IkSolutionListBase<IkReal>& solutions) {\n'%(node.functionid)
            code += 'for(int rotationiter = 0; rotationiter < 1; ++rotationiter) {\n'
            origequations = self.dictequations
            self.resetequations()
            listequations = []
            names = []
            for i in range(3):
                for j in range(3):
                    listequations.append(node.T[i,j])
                    names.append(Symbol('new_r%d%d'%(i,j)))
            code += self.writeEquations(lambda i: names[i],listequations)
            code += self.generateTree(node.jointtree)
            code += '}\n}'
            self.dictequations = origequations
            self.functions[node.functionid] = code
        return 'rotationfunction%d(solutions);\n'%(node.functionid)

    def endRotation(self, node):
        return ''
    def generateDirection(self, node):
        code = ''
        listequations = []
        names = []
        for i in range(3):
            listequations.append(node.D[i])
            names.append(Symbol('new_r%d%d'%(0,i)))
        code += self.writeEquations(lambda i: names[i],listequations)
        code += self.generateTree(node.jointtree)
        return code
    def endDirection(self, node):
        return ''
    def generateStoreSolution(self, node):
        self._solutioncounter += 1
        log.info('c=%d, store solution', self._solutioncounter)
        code = cStringIO.StringIO()
        if node.checkgreaterzero is not None and len(node.checkgreaterzero) > 0:
            origequations = self.copyequations()
            code.write('IkReal soleval[%d];\n'%(len(node.checkgreaterzero)))
            self.WriteEquations2(lambda i: 'soleval[%d]'%(i),node.checkgreaterzero,code=code)
            code.write('if( ')
            for i in range(len(node.checkgreaterzero)):
                if i != 0:
                    code.write(' && ')
                code.write('soleval[%d] > %.16f '%(i,node.thresh))
            code.write(' )\n')
            self.dictequations = origequations
        code.write('{\n')
        code.write('std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(%d);\n'%len(node.alljointvars))
        for i,var in enumerate(node.alljointvars):
            offsetvalue = '+%.15e'%node.offsetvalues[i] if node.offsetvalues is not None else ''
            code.write('vinfos[%d].jointtype = %d;\n'%(i,0x01 if node.isHinge[i] else 0x11))
            code.write('vinfos[%d].foffset = %s%s;\n'%(i,var,offsetvalue))
            vardeps = [vardep for vardep in self.freevardependencies if vardep[1]==var.name]
            if len(vardeps) > 0:
                freevarname = vardeps[0][0]
                ifreevar = [j for j in range(len(self.freevars)) if freevarname==self.freevars[j]]
                code.write('vinfos[%d].fmul = %smul;\n'%(i,var.name))
                code.write('vinfos[%d].freeind = %d;\n'%(i,ifreevar[0]))
                code.write('vinfos[%d].maxsolutions = 0;\n'%(i))
            else:
                code.write('vinfos[%d].indices[0] = _i%s[0];\n'%(i,var))
                code.write('vinfos[%d].indices[1] = _i%s[1];\n'%(i,var))
                code.write('vinfos[%d].maxsolutions = _n%s;\n'%(i,var))
        code.write('std::vector<int> vfree(%d);\n'%len(self.freevars))
        for i,varname in enumerate(self.freevars):
            ind = [j for j in range(len(node.alljointvars)) if varname==node.alljointvars[j].name]
            code.write('vfree[%d] = %d;\n'%(i,ind[0]))
        code.write('solutions.AddSolution(vinfos,vfree);\n')
        code.write('}\n')
        return code.getvalue()
    
    def endStoreSolution(self, node):
        return ''
    def generateSequence(self, node):
        code = ''
        for tree in node.jointtrees:
            code += self.generateTree(tree)
        return code
    def endSequence(self, node):
        return ''
    def generateTree(self,tree):
        code = ''
        for n in tree:
            code += n.generate(self)
        for n in reversed(tree):
            code += n.end(self)
        return code

    def WriteDictEquations(self, dictequations, code=None):
        """writes the dict equations (sym,var)
        If first parameter is not a symbol or in self._globalvariables, will skip the equation
        """
        if code is None:
            code = cStringIO.StringIO()
        if len(dictequations) == 0:
            return code
        
        # calling cse on many long expressions will freeze it, so try to divide the problem
        complexitythresh = 4000
        exprs = []
        curcomplexity = 0
        for i,varexpr in enumerate(dictequations):
            if varexpr[0].is_Symbol and not varexpr[0] in self._globalvariables: # dangerous to put the check?
                curcomplexity += varexpr[1].count_ops()
                exprs.append(varexpr)
            if curcomplexity > complexitythresh or i == len(dictequations)-1:
                self._WriteDictEquations(exprs, code)
                exprs = []
                curcomplexity = 0
        assert(len(exprs)==0)
        return code
    
    def _WriteDictEquations(self, dictequations, code=None):
        """assumes all the dictequations are writing symbols
        :param dictequations: list of (var,eq) pairs
        """
        if code is None:
            code = cStringIO.StringIO()
        exprs = [expr for var, expr in dictequations]
        replacements,reduced_exprs = customcse(exprs,symbols=self.symbolgen)
        N = len(self.dictequations[0])
        for rep in replacements:
            comparerep = rep[1].subs(self.dictequations[0]).expand()
            found = False
            complexity = rep[1].count_ops()
            maxcomplexity = 3 if N > 1000 else 2
            if complexity > maxcomplexity: # check only long expressions
                for i in range(N):
                    if self.dictequations[1][i] is not None and comparerep-self.dictequations[1][i]==S.Zero:
                        #self.dictequations.append((rep[0],self.dictequations[0][i][0],self.dictequations[1][i]))
                        code.write('IkReal %s=%s;\n'%(rep[0],self.dictequations[0][i][0]))
                        found = True
                        break
            else:
                comparerep = None
            if not found:
                self.dictequations[0].append(rep)
                self.dictequations[1].append(comparerep)
                code2,sepcodelist2 = self._WriteExprCode(rep[1])
                for sepcode in sepcodelist2:
                    code.write(sepcode)
                code.write('IkReal %s='%rep[0])
                code.write(code2.getvalue())
                code.write(';\n')
        for i,rexpr in enumerate(reduced_exprs):
            code2,sepcodelist2 = self._WriteExprCode(rexpr)
            for sepcode in sepcodelist2:
                code.write(sepcode)
            if not dictequations[i][0].name in self._globalvariables:
                code.write('IkReal ')
            code.write('%s='%dictequations[i][0])
            code.write(code2.getvalue())
            code.write(';\n')
        return code
    
    def writeEquations(self, varnamefn, allexprs):
        code = cStringIO.StringIO()
        self.WriteEquations2(varnamefn, allexprs, code)
        return code.getvalue()
    
    def WriteEquations2(self, varnamefn, allexprs, code=None):
        if code is None:
            code = cStringIO.StringIO()
            
        if not hasattr(allexprs,'__iter__') and not hasattr(allexprs,'__array__'):
            allexprs = [allexprs]
        if using_swiginac and isinstance(allexprs,list):
            if len(allexprs) == 0:
                return code
            
            if isinstance(allexprs[0],swiginac.basic):
                code.write(self._writeGinacEquations(varnamefn, allexprs))
                return code
            
        # calling cse on many long expressions will freeze it, so try to divide the problem
        complexity = [expr.count_ops() for expr in allexprs]
        complexitythresh = 4000
        exprs = []
        curcomplexity = 0
        for i,expr in enumerate(allexprs):
            curcomplexity += complexity[i]
            exprs.append(expr)
            if curcomplexity > complexitythresh or i == len(allexprs)-1:
                self._WriteEquations(varnamefn, exprs, i+1-len(exprs), code)
                exprs = []
                curcomplexity = 0
        assert(len(exprs)==0)
        return code
    
    def _writeGinacEquations(self, varnamefn, allexprs):
        allcode = cStringIO.StringIO()
        for i,expr in enumerate(allexprs):
            print '%d/%d'%(i,len(allexprs))
            code,sepcodelist = self._WriteGinacExprCode(expr)
            for sepcode in sepcodelist:
                allcode.write(sepcode)
            allcode.write(str(varnamefn(i)))
            allcode.write('=')
            allcode.write(code.getvalue())
            allcode.write(';\n')
        return allcode.getvalue()
    
    def _WriteEquations(self, varnamefn, exprs, ioffset, code=None):
        if code is None:
            code = cStringIO.StringIO()
        replacements,reduced_exprs = customcse(exprs,symbols=self.symbolgen)
        #for greaterzerocheck in greaterzerochecks:
        #    code.write('if((%s) < -0.00001)\ncontinue;\n'%exprbase)
        N = len(self.dictequations[0])
        for rep in replacements:
            comparerep = rep[1].subs(self.dictequations[0]).expand()
            found = False
            complexity = rep[1].count_ops()
            maxcomplexity = 3 if N > 1000 else 2
            if complexity > maxcomplexity: # check only long expressions
                for i in range(N):
                    if self.dictequations[1][i] is not None and comparerep-self.dictequations[1][i]==S.Zero:
                        #self.dictequations.append((rep[0],self.dictequations[0][i][0],self.dictequations[1][i]))
                        code.write('IkReal %s=%s;\n'%(rep[0],self.dictequations[0][i][0]))
                        found = True
                        break
            else:
                comparerep = None
            if not found:
                self.dictequations[0].append(rep)
                self.dictequations[1].append(comparerep)
                code2,sepcodelist2 = self._WriteExprCode(rep[1])
                for sepcode in sepcodelist2:
                    code.write(sepcode)
                code.write('IkReal %s='%rep[0])
                code.write(code2.getvalue())
                code.write(';\n')
        for i,rexpr in enumerate(reduced_exprs):
            code2,sepcodelist2 = self._WriteExprCode(rexpr)
            for sepcode in sepcodelist2:
                code.write(sepcode)
            code.write('%s='%varnamefn(i+ioffset))
            code.write(code2.getvalue())
            code.write(';\n')
        return code

    def _WriteExprCode(self, expr, code=None):
        # go through all arguments and chop them
        if code is None:
            code = cStringIO.StringIO()
        if expr.is_Function:
            if expr.func == Abs:
                code.write('IKabs(')
                code2,sepcodelist = self._WriteExprCode(expr.args[0],code)
            elif expr.func == conjugate:
                # because we're not dealing with imaginary, this is just the regular number
                code2,sepcodelist = self._WriteExprCode(expr.args[0],code)
                return code,sepcodelist
            elif expr.func == sign:
                code.write('IKsign(')
                code2,sepcodelist = self._WriteExprCode(expr.args[0],code)
            elif expr.func == acos:
                code.write('IKacos(')
                pos0 = code.tell()
                code2,sepcodelist = self._WriteExprCode(expr.args[0],code)
                pos1 = code.tell()
                code.seek(pos0)
                exprbase = code.read(pos1-pos0)
                code.seek(pos1)
                sepcodelist.append('if( (%s) < -1-IKFAST_SINCOS_THRESH || (%s) > 1+IKFAST_SINCOS_THRESH )\n    continue;\n'%(exprbase,exprbase))
            elif expr.func == asin:
                code.write('IKasin(')
                pos0 = code.tell()
                code2,sepcodelist = self._WriteExprCode(expr.args[0],code)
                pos1 = code.tell()
                code.seek(pos0)
                exprbase = code.read(pos1-pos0)
                code.seek(pos1)
                sepcodelist.append('if( (%s) < -1-IKFAST_SINCOS_THRESH || (%s) > 1+IKFAST_SINCOS_THRESH )\n    continue;\n'%(exprbase,exprbase))
            elif expr.func == atan2check:
                code.write('IKatan2(')
                # check for divides by 0 in arguments, this could give two possible solutions?!?
                # if common arguments is nan! solution is lost!
                pos0 = code.tell()
                code2,sepcodelist = self._WriteExprCode(expr.args[0],code)
                pos1 = code.tell()
                code.seek(pos0)
                exprbase0 = code.read(pos1-pos0)
                code.seek(pos1)
                code.write(', ')
                pos0 = code.tell()
                code3,sepcodelist2 = self._WriteExprCode(expr.args[1],code)
                sepcodelist += sepcodelist2
                pos1 = code.tell()
                code.seek(pos0)
                exprbase1 = code.read(pos1-pos0)
                code.seek(pos1)
                sepcodelist.append('if( IKabs(%s) < IKFAST_ATAN2_MAGTHRESH && IKabs(%s) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(%s)+IKsqr(%s)-1) <= IKFAST_SINCOS_THRESH )\n    continue;\n'%(exprbase0,exprbase1,exprbase0,exprbase1))
            elif expr.func == atan2:
                # check for divides by 0 in arguments, this could give two possible solutions?!?
                # if common arguments is nan! solution is lost!
                # use IKatan2WithCheck in order to make it robust against NaNs
                iktansymbol = self.symbolgen.next()
                
                code2 = cStringIO.StringIO()
                code2.write('CheckValue<IkReal> %s = IKatan2WithCheck(IkReal('%iktansymbol)
                code3,sepcodelist = self._WriteExprCode(expr.args[0], code2)
                code2.write('),IkReal(')
                code4,sepcodelist2 = self._WriteExprCode(expr.args[1], code2)
                code2.write('),IKFAST_ATAN2_MAGTHRESH);\nif(!%s.valid){\ncontinue;\n}\n'%iktansymbol)
                sepcodelist += sepcodelist2
                sepcodelist.append(code2.getvalue())
                
                code.write('%s.value'%iktansymbol)
                return code,sepcodelist
            
            elif expr.func == sin:
                code.write('IKsin(')
                code2,sepcodelist = self._WriteExprCode(expr.args[0], code)
            elif expr.func == cos:
                code.write('IKcos(')
                code2,sepcodelist = self._WriteExprCode(expr.args[0], code)
            elif expr.func == fmod:
                code.write('IKfmod(')
                code2,sepcodelist = self._WriteExprCode(expr.args[0], code)
                code.write(', ')
                code3,sepcodelist2 = self._WriteExprCode(expr.args[1], code)
                sepcodelist += sepcodelist2
            elif expr.func == RemoveAbsFn:
                return self._WriteExprCode(expr.args[0], code)
            
            else:
                code.write(expr.func.__name__)
                code.write('(')
                sepcodelist = []
                for arg in expr.args:
                    code2,sepcodelist2 = self._WriteExprCode(arg, code)
                    sepcodelist += sepcodelist2
                    if not arg == expr.args[-1]:
                        code.write(',')
            code.write(')')
            return code,sepcodelist
        
        elif expr.is_number:
            expreval = expr.evalf()
            assert(expreval.is_real)
            code.write(self.strprinter.doprint(expreval))
            return code, []
        
        elif expr.is_Mul:
            sepcodelist = []
            code.write('(')
            for arg in expr.args:
                if arg.is_Symbol:
                    code.write(str(arg))
                else:
                    code.write('(')
                    code2,sepcodelist2 = self._WriteExprCode(arg, code)
                    code.write(')')
                    sepcodelist += sepcodelist2
                if not arg == expr.args[-1]:
                    code.write('*')
            code.write(')')
            return code, sepcodelist
        
        elif expr.is_Pow:
            if expr.base.is_Function and expr.base.func == RemoveAbsFn:
                return self._WriteExprCode(Pow(expr.base.args[0], expr.exp, evaluate=False), code)
            
            if expr.exp.is_number:
                if expr.exp.is_integer and expr.exp > 0:
                    if expr.base.is_Symbol:
                        # simple, can use the symbol as is
                        exprbasestr = str(expr.base)
                        code.write(exprbasestr)
                        for i in range(1,int(expr.exp)):
                            code.write('*')
                            code.write(exprbasestr)
                        return code, []
                    else:
                        # need to create a new symbol
                        ikpowsymbol = self.symbolgen.next()
                        code2 = cStringIO.StringIO()
                        code2.write('IkReal ')
                        code2.write(str(ikpowsymbol))
                        code2.write('=')
                        code3,sepcodelist = self._WriteExprCode(expr.base, code2)
                        code2.write(';\n')
                        sepcodelist.append(code2.getvalue())
                        code.write(str(ikpowsymbol))
                        for i in range(1,int(expr.exp)):
                            code.write('*')
                            code.write(str(ikpowsymbol))
                        return code,sepcodelist
                elif expr.exp.is_integer:
                    # use IKPowWithIntegerCheck in order to make it robust
                    ikpowsymbol = self.symbolgen.next()
                    code2 = cStringIO.StringIO()
                    code2.write('CheckValue<IkReal> %s=IKPowWithIntegerCheck('%ikpowsymbol)
                    code3,sepcodelist = self._WriteExprCode(expr.base, code2)
                    code2.write(',')
                    code2.write(str(int(expr.exp)))
                    code2.write(');\nif(!%s.valid){\ncontinue;\n}\n'%ikpowsymbol)
                    sepcodelist.append(code2.getvalue())
                    code.write('%s.value'%ikpowsymbol)
                    return code,sepcodelist
                    
                elif expr.exp-0.5 == S.Zero:
                    code.write('IKsqrt(')
                    pos0 = code.tell()
                    code2,sepcodelist = self._WriteExprCode(expr.base,code)
                    pos1 = code.tell()
                    code.seek(pos0)
                    exprbase = code.read(pos1-pos0)
                    code.seek(pos1)
                    sepcodelist.append('if((%s) < -0.00001)\ncontinue;\n'%exprbase)
                    code.write(')')
                    return code, sepcodelist
                
                elif expr.exp < 0:
                    # use IKPowWithIntegerCheck in order to make it robust
                    # check if exprbase is 0
                    ikpowsymbol = self.symbolgen.next()
                    code2 = cStringIO.StringIO()
                    code2.write('IkReal %s = '%ikpowsymbol)
                    code3,sepcodelist = self._WriteExprCode(expr.base, code2)
                    code2.write(';\nif(IKabs(%s)==0){\ncontinue;\n}\n'%ikpowsymbol)
                    sepcodelist.append(code2.getvalue())                    
                    code.write('pow(%s,%s)'%(ikpowsymbol, self.strprinter.doprint(expr.exp.evalf())))
                    return code,sepcodelist
                
            # do the most general pow function
            code.write('pow(')
            code2,sepcodelist = self._WriteExprCode(expr.base, code)
            code.write(',')
            code3,sepcodelist2 = self._WriteExprCode(expr.exp, code)
            sepcodelist += sepcodelist2
            code.write(')')
            return code, sepcodelist
        
        elif expr.is_Add:
            code.write('(')
            sepcodelist = []
            for arg in expr.args:
                if arg.is_Symbol:
                    code.write(str(arg))
                else:
                    code.write('(')
                    code2,sepcodelist2 = self._WriteExprCode(arg, code)
                    code.write(')')
                    sepcodelist += sepcodelist2
                if not arg == expr.args[-1]:
                    code.write('+')
            code.write(')')
            return code, sepcodelist

        elif hasattr(expr, 'is_Sub') and expr.is_Sub: # for cse.Sub
            code.write('(')
            sepcodelist = []
            for arg in expr.args:
                if arg.is_Symbol:
                    code.write(str(arg))
                else:
                    code.write('(')
                    code2,sepcodelist2 = self._WriteExprCode(arg, code)
                    code.write(')')
                    sepcodelist += sepcodelist2
                if not arg == expr.args[-1]:
                    code.write('-')
            code.write(')')
            return code, sepcodelist
        
        code.write(self.strprinter.doprint(expr.evalf()))
        return code,[]

    def _WriteGinacExprCode(self, expr, code=None):
        """writes with ginac expression
        """
        # go through all arguments and chop them
        if code is None:
            code = cStringIO.StringIO()
        if isinstance(expr, swiginac.numeric):
            code.write('IkReal(')
            code.write(self.strprinter.doprint(expr.evalf()))
            code.write(')')
            return code,[]
        
        elif isinstance(expr, swiginac.mul):
            sepcodelist = []
            code.write('(')
            for iarg in range(expr.nops()):
                code.write('(')
                code2,sepcode2 = self._WriteGinacExprCode(expr.op(iarg),code)
                code.write(')')
                sepcodelist += sepcode2
                if iarg+1 < expr.nops():
                    code.write('*')
            code.write(')')
            return code,sepcodelist
        
        elif isinstance(expr, swiginac.power):
            exp = expr.op(1)
            if isinstance(exp, swiginac.numeric):
                if exp.is_integer() and exp.eval() > 0:
                    code.write('(')
                    pos0 = code.tell()
                    code2,sepcodelist = self._WriteGinacExprCode(expr.op(0),code)
                    pos1 = code.tell()
                    code.seek(pos0)
                    exprbase = code.read(pos1-pos0)
                    code.seek(pos1)
                    code.write(')')
                    for i in range(1,exp.eval()):
                        code.write('*(')
                        code.write(exprbase)
                        code.write(')')
                    return code,sepcodelist
                
                elif exp.evalf()-0.5 == 0:
                    code.write('IKsqrt(')
                    pos0 = code.tell()
                    code2,sepcodelist = self._WriteGinacExprCode(expr.op(0),code)
                    pos1 = code.tell()
                    code.seek(pos0)
                    exprbase = code.read(pos1-pos0)
                    code.seek(pos1)
                    sepcodelist.append('if( (%s) < (IkReal)-0.00001 )\n    continue;\n'%exprbase)
                    code.write(')')
                    return code, sepcodelist
                
                elif exp.evalf()+1 == 0:
                    # check if exprbase is 0
                    code.write('((IKabs(')
                    pos0 = code.tell()
                    code2,sepcodelist = self._WriteGinacExprCode(expr.op(0),code)
                    pos1 = code.tell()
                    code.seek(pos0)
                    exprbase = code.read(pos1-pos0)
                    code.seek(pos1)
                    code.write(') != 0)?((IkReal)1/(')
                    code.write(exprbase)
                    code.write(')):(IkReal)1.0e30)')
                    return code,sepcodelist
                
                elif exp.is_integer() and exp.eval() < 0:
                    code.write('((IKabs(')
                    pos0 = code.tell()
                    code2,sepcodelist = self._WriteGinacExprCode(expr.op(0),code)
                    pos1 = code.tell()
                    code.seek(pos0)
                    exprbase = code.read(pos1-pos0)
                    code.seek(pos1)
                    code.write(') != 0)?((IkReal)1/(')
                    # check if exprbase is 0
                    code.write('(')
                    code.write(exprbase)
                    code.write(')')
                    for i in range(1,-exp.eval()):
                        code.write('*(')
                        code.write(exprbase)
                        code.write(')')
                    code.write(')):(IkReal)1.0e30)')
                    return code,sepcodelist
                
                elif exp < 0:
                    # check if exprbase is 0
                    code.write('((IKabs(')
                    pos0 = code.tell()
                    code2,sepcodelist = self._WriteGinacExprCode(expr.op(0),code)
                    pos1 = code.tell()
                    code.seek(pos0)
                    exprbase = code.read(pos1-pos0)
                    code.seek(pos1)
                    code.write(') != 0)?(pow(')
                    code.write(exprbase)
                    code.write(',')
                    code.write(str(exp.evalf()))
                    code.write(')):(IkReal)1.0e30)')
                    return code,sepcodelist
                    
            code.write('pow(')
            code2,sepcodelist = self._WriteGinacExprCode(expr.op(0),code)
            code.write(',')
            code2,sepcodelist2 = self._WriteGinacExprCode(exp,code)
            sepcodelist += sepcodelist2
            code.write(')')
            return code,sepcodelist
        
        elif isinstance(expr, swiginac.add):
            sepcodelist = []
            code.write('(')
            for iarg in range(expr.nops()):
                code.write('(')
                code2,sepcode2 = self._WriteGinacExprCode(expr.op(iarg),code)
                sepcodelist += sepcode2
                code.write(')')
                if iarg+1 < expr.nops():
                    code.write('+')
            code.write(')')
            return code, sepcodelist
        
        code.write(self.strprinter.doprint(expr.evalf()))
        return code,[]
    
    def using_polyroots(self, deg):
        name = 'polyroots%d'%deg
        if not name in self.functions:
            if deg == 1:            
                fcode = """static inline void %s(IkReal rawcoeffs[1+1], IkReal rawroots[1], int& numroots) {
    numroots=0;
    if( rawcoeffs[0] != 0 ) {
        rawroots[0] = -rawcoeffs[1]/rawcoeffs[0];
        numroots=1;
    }
}
"""%name
            elif deg == 2:
                fcode = """static inline void %s(IkReal rawcoeffs[2+1], IkReal rawroots[2], int& numroots) {
    IkReal det = rawcoeffs[1]*rawcoeffs[1]-4*rawcoeffs[0]*rawcoeffs[2];
    if( det < 0 ) {
        numroots=0;
    }
    else if( det == 0 ) {
        rawroots[0] = -0.5*rawcoeffs[1]/rawcoeffs[0];
        numroots = 1;
    }
    else {
        det = IKsqrt(det);
        rawroots[0] = (-rawcoeffs[1]+det)/(2*rawcoeffs[0]);
        rawroots[1] = (-rawcoeffs[1]-det)/(2*rawcoeffs[0]);//rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
        numroots = 2;
    }
}
"""%name
            elif False:#deg == 3: # amazing, cubic formula is not as accurate as iterative method...
                fcode = """static inline void %s(IkReal rawcoeffs[%d+1], IkReal rawroots[%d], int& numroots)
{
    using std::complex;
    IKFAST_ASSERT(rawcoeffs[0] != 0);
    IkReal a0 = rawcoeffs[3]/rawcoeffs[0], a1 = rawcoeffs[2]/rawcoeffs[0], a2 = rawcoeffs[1]/rawcoeffs[0];
    IkReal a2_3 = a2/3.0;
    IkReal Q = (3*a1-a2*a2)/9, R = (9*a2*a1-27*a0-2*a2*a2*a2)/54;
    complex<IkReal> D(Q*Q*Q+R*R,0.0);
    complex<IkReal> Dsqrt = sqrt(D);
    complex<IkReal> S, T;
    if( imag(Dsqrt) != 0 ) {
        S = pow(complex<IkReal>(R,0)+Dsqrt,IkReal(1.0/3.0));
        T = pow(complex<IkReal>(R,0)-Dsqrt,IkReal(1.0/3.0));
    }
    else {
        IkReal temp = R+real(Dsqrt);
        S = pow(IKabs(temp),IkReal(1.0/3.0));
        if( temp < 0 ) {
            S = -S;
        }
        temp = R-real(Dsqrt);
        T = pow(IKabs(temp),IkReal(1.0/3.0));
        if( temp < 0 ) {
            T = -T;
        }
    }
    complex<IkReal> B = S+T, A = S-T;
    numroots = 0;
    if( IKabs(imag(B)) < 40*std::numeric_limits<IkReal>::epsilon() ) {
        rawroots[numroots++] = -a2_3+real(B);
    }
    complex<IkReal> Arot = complex<IkReal>(0,SQRT_3)*A;
    if( IKabs(imag(B-Arot)) < 40*std::numeric_limits<IkReal>::epsilon() ) {
        rawroots[numroots++] = -a2_3-0.5*real(B-Arot);
        rawroots[numroots++] = -a2_3-0.5*real(B+Arot);
    }
}
"""%name
            elif deg > 0:
                # Durand-Kerner polynomial root finding method
                # In case of multiple roots, see Pierre Fraigniaud's work on Quadratic-Like Convergence of the Mean
                # http://www.springerlink.com/content/t72g1635574u10q3/
                # the following poly has 2 double roots and requires 105-110 steps to get accurate roots (-2.48003364697210,  -0.266182031226875, -0.399192786087098, 5.23118700753098, 1.42932474708659, -3.56915690046190, 1.51649127129379, -0.100045756315578, -1.36239190484780)
                # on universalrobots-ur6-85-5-a.zae (deg 8), 110 steps is 87.5%, 150 steps is 88.8%, performance difference is 200ms
                fcode = """static inline void %(name)s(IkReal rawcoeffs[%(deg)d+1], IkReal rawroots[%(deg)d], int& numroots)
{
    using std::complex;
    if( rawcoeffs[0] == 0 ) {
        // solve with one reduced degree
        %(reducedpolyroots)s(&rawcoeffs[1], &rawroots[0], numroots);
        return;
    }
    IKFAST_ASSERT(rawcoeffs[0] != 0);
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
    complex<IkReal> coeffs[%(deg)d];
    const int maxsteps = 110;
    for(int i = 0; i < %(deg)d; ++i) {
        coeffs[i] = complex<IkReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IkReal> roots[%(deg)d];
    IkReal err[%(deg)d];
    roots[0] = complex<IkReal>(1,0);
    roots[1] = complex<IkReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < %(deg)d; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < %(deg)d; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IkReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < %(deg)d; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < %(deg)d; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    // sort roots hoping that it solution indices become more robust to slight change in coeffs
    std::sort(roots, roots+%(deg)d, ComplexLess<IkReal>());

    numroots = 0;
    bool visited[%(deg)d] = {false};
    for(int i = 0; i < %(deg)d; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IkReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < %(deg)d; ++j) {
                // care about error in real much more than imaginary
                if( abs(real(roots[i])-real(roots[j])) < tolsqrt && (abs(imag(roots[i])-imag(roots[j])) < 0.002 || abs(imag(roots[i])+imag(roots[j])) < 0.002) && abs(imag(roots[i])) < 0.002 ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
            if( IKabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}
"""%{'name':name, 'deg':deg, 'reducedpolyroots':self.using_polyroots(deg-1) }
            else:
                raise ValueError('poly degree is %d'%deg)
            
            self.functions[name] = fcode
        return name

    def using_checkconsistency12(self):
        name = 'checkconsistency12'
        if not name in self.functions:
            fcode = """
static inline bool %s(const IkReal* Breal)
{
    IkReal norm = 0.1;
    for(int i = 0; i < 11; ++i) {
        norm += IKabs(Breal[i]);
    }
    IkReal tol = 1e-6*norm; // have to increase the threshold since many computations are involved
    return IKabs(Breal[0]*Breal[0]-Breal[1]) < tol && IKabs(Breal[0]*Breal[2]-Breal[3]) < tol && IKabs(Breal[1]*Breal[2]-Breal[4]) < tol && IKabs(Breal[2]*Breal[2]-Breal[5]) < tol && IKabs(Breal[0]*Breal[5]-Breal[6]) < tol && IKabs(Breal[1]*Breal[5]-Breal[7]) < tol && IKabs(Breal[2]*Breal[5]-Breal[8]) < tol && IKabs(Breal[0]*Breal[8]-Breal[9]) < tol && IKabs(Breal[1]*Breal[8]-Breal[10]) < tol;
}
"""%name
            self.functions[name] = fcode
        return name

    def using_matrixinverse(self):
        name = 'matrixinverse'
        if not name in self.functions:
            fcode = """
template<int D>
static inline bool %s(IkReal* A)
{
    int n = D;
    int info;
    IkReal IKFAST_ALIGNED16(work[D*D*(D-1)]);
    int ipiv[D];
    dgetrf_(&n, &n, A, &n, &ipiv[0], &info);
    if( info != 0 ) {
        return false;
    }
    int worksize=D*D*(D-1);
    dgetri_(&n, A, &n, &ipiv[0], &work[0], &worksize, &info);
    return info==0;
}
"""%(name)
            self.functions[name] = fcode
        return name

    def using_solvedialyticpoly12qep(self):
        name = 'solvedialyticpoly12qep'
        checkconsistency12=self.using_checkconsistency12()
        if not name in self.functions:
            fcode = """
/// \\brief Solve the det Ax^2+Bx+C = 0 problem using the Manocha and Canny method (1994)
///
/// matcoeffs is of length 54*3, for 3 matrices
static inline void %s(const IkReal* matcoeffs, IkReal* rawroots, int& numroots)
{
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    IkReal IKFAST_ALIGNED16(M[24*24]) = {0};
    IkReal IKFAST_ALIGNED16(A[12*12]);
    IkReal IKFAST_ALIGNED16(work[24*24*23]);
    int ipiv[12];
    int info, coeffindex;
    const int worksize=24*24*23;
    const int matrixdim = 12;
    const int matrixdim2 = 24;
    numroots = 0;
    // first setup M = [0 I; -C -B] and A
    coeffindex = 0;
    for(int j = 0; j < 6; ++j) {
        for(int k = 0; k < 9; ++k) {
            M[matrixdim+(j+6)+2*matrixdim*k] = M[matrixdim+j+2*matrixdim*(k+3)] = -matcoeffs[coeffindex++];
        }
    }
    for(int j = 0; j < 6; ++j) {
        for(int k = 0; k < 9; ++k) {
            M[matrixdim+(j+6)+2*matrixdim*k+matrixdim*2*matrixdim] = M[matrixdim+j+2*matrixdim*(k+3)+matrixdim*2*matrixdim] = -matcoeffs[coeffindex++];
        }
    }
    for(int j = 0; j < 6; ++j) {
        for(int k = 0; k < 9; ++k) {
            A[(j+6)+matrixdim*k] = A[j+matrixdim*(k+3)] = matcoeffs[coeffindex++];
        }
        for(int k = 0; k < 3; ++k) {
            A[j+matrixdim*k] = A[(j+6)+matrixdim*(k+9)] = 0;
        }
    }
    const IkReal lfpossibilities[4][4] = {{1,-1,1,1},{1,0,-2,1},{1,1,2,0},{1,-1,4,1}};
    int lfindex = -1;
    bool bsingular = true;
    do {
        dgetrf_(&matrixdim,&matrixdim,A,&matrixdim,&ipiv[0],&info);
        if( info == 0 ) {
            bsingular = false;
            for(int j = 0; j < matrixdim; ++j) {
                if( IKabs(A[j*matrixdim+j]) < 100*tol ) {
                    bsingular = true;
                    break;
                }
            }
            if( !bsingular ) {
                break;
            }
        }
        if( lfindex == 3 ) {
            break;
        }
        // transform by the linear functional
        lfindex++;
        const IkReal* lf = lfpossibilities[lfindex];
        // have to reinitialize A
        coeffindex = 0;
        for(int j = 0; j < 6; ++j) {
            for(int k = 0; k < 9; ++k) {
                IkReal a = matcoeffs[coeffindex+108], b = matcoeffs[coeffindex+54], c = matcoeffs[coeffindex];
                A[(j+6)+matrixdim*k] = A[j+matrixdim*(k+3)] = lf[0]*lf[0]*a+lf[0]*lf[2]*b+lf[2]*lf[2]*c;
                M[matrixdim+(j+6)+2*matrixdim*k] = M[matrixdim+j+2*matrixdim*(k+3)] = -(lf[1]*lf[1]*a + lf[1]*lf[3]*b + lf[3]*lf[3]*c);
                M[matrixdim+(j+6)+2*matrixdim*k+matrixdim*2*matrixdim] = M[matrixdim+j+2*matrixdim*(k+3)+matrixdim*2*matrixdim] = -(2*lf[0]*lf[1]*a + (lf[0]*lf[3]+lf[1]*lf[2])*b + 2*lf[2]*lf[3]*c);
                coeffindex++;
            }
            for(int k = 0; k < 3; ++k) {
                A[j+matrixdim*k] = A[(j+6)+matrixdim*(k+9)] = 0;
            }
        }
    } while(lfindex<4);

    if( bsingular ) {
        return;
    }
    dgetrs_("No transpose", &matrixdim, &matrixdim2, A, &matrixdim, &ipiv[0], &M[matrixdim], &matrixdim2, &info);
    if( info != 0 ) {
        return;
    }

    // set identity in upper corner
    for(int j = 0; j < matrixdim; ++j) {
        M[matrixdim*2*matrixdim+j+matrixdim*2*j] = 1;
    }
    IkReal IKFAST_ALIGNED16(wr[24]);
    IkReal IKFAST_ALIGNED16(wi[24]);
    IkReal IKFAST_ALIGNED16(vr[24*24]);
    int one=1;
    dgeev_("N", "V", &matrixdim2, M, &matrixdim2, wr, wi,NULL, &one, vr, &matrixdim2, work, &worksize, &info);
    if( info != 0 ) {
        return;
    }
    IkReal Breal[matrixdim-1];
    for(int i = 0; i < matrixdim2; ++i) {
        if( IKabs(wi[i]) < tol*100 ) {
            IkReal* ev = vr+matrixdim2*i;
            if( IKabs(wr[i]) > 1 ) {
                ev += matrixdim;
            }
            // consistency has to be checked!!
            if( IKabs(ev[0]) < tol ) {
                continue;
            }
            IkReal iconst = 1/ev[0];
            for(int j = 1; j < matrixdim; ++j) {
                Breal[j-1] = ev[j]*iconst;
            }
            if( %s(Breal) ) {
                if( lfindex >= 0 ) {
                    const IkReal* lf = lfpossibilities[lfindex];
                    rawroots[numroots++] = (wr[i]*lf[0]+lf[1])/(wr[i]*lf[2]+lf[3]);
                }
                else {
                    rawroots[numroots++] = wr[i];
                }
                bool bsmall0=IKabs(ev[0]) > IKabs(ev[3]);
                bool bsmall1=IKabs(ev[0]) > IKabs(ev[1]);
                if( bsmall0 && bsmall1 ) {
                    rawroots[numroots++] = ev[3]/ev[0];
                    rawroots[numroots++] = ev[1]/ev[0];
                }
                else if( bsmall0 && !bsmall1 ) {
                    rawroots[numroots++] = ev[5]/ev[2];
                    rawroots[numroots++] = ev[2]/ev[1];
                }
                else if( !bsmall0 && bsmall1 ) {
                    rawroots[numroots++] = ev[9]/ev[6];
                    rawroots[numroots++] = ev[10]/ev[9];
                }
                else if( !bsmall0 && !bsmall1 ) {
                    rawroots[numroots++] = ev[11]/ev[8];
                    rawroots[numroots++] = ev[11]/ev[10];
                }
            }
        }
    }
}"""%(name,checkconsistency12)
            self.functions[name] = fcode
        return name

    def using_checkconsistency8(self):
        name = 'checkconsistency8'
        if not name in self.functions:
            fcode = """
// [(0, 0), (0, 1), (1, 0), (1, 1), (2, 0), (2, 1), (3, 0), (3, 1)] (original are [(0, 0), (0, 1), (1, 0), (1, 1), (2, 0), (2, 1)])
static inline bool %s(const IkReal* Breal)
{
    IkReal norm = 0.1;
    for(int i = 0; i < 7; ++i) {
        norm += IKabs(Breal[i]);
    }
    // HACK should be 1e-5*norm
    IkReal tol = 1e-2*norm; // have to increase the threshold since many computations are involved
    return IKabs(Breal[0]*Breal[1]-Breal[2]) < tol && IKabs(Breal[1]*Breal[1]-Breal[3]) < tol && IKabs(Breal[0]*Breal[3]-Breal[4]) < tol && IKabs(Breal[1]*Breal[3]-Breal[5]) < tol && IKabs(Breal[0]*Breal[5]-Breal[6]) < tol;
}"""%name
            self.functions[name] = fcode
        return name

    def using_solvedialyticpoly8qep(self):
        name = 'solvedialyticpoly8qep'
        checkconsistency8=self.using_checkconsistency8()
        if not name in self.functions:
            fcode = """
/// \\brief Solve the det Ax^2+Bx+C = 0 problem using the Manocha and Canny method (1994)
///
/// matcoeffs is of length 54*3, for 3 matrices
static inline void %s(const IkReal* matcoeffs, IkReal* rawroots, int& numroots)
{
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    IkReal IKFAST_ALIGNED16(M[16*16]) = {0};
    IkReal IKFAST_ALIGNED16(A[8*8]);
    IkReal IKFAST_ALIGNED16(work[16*16*15]);
    int ipiv[8];
    int info, coeffindex;
    const int worksize=16*16*15;
    const int matrixdim = 8;
    const int matrixdim2 = 16;
    numroots = 0;
    // first setup M = [0 I; -C -B] and A
    coeffindex = 0;
    for(int j = 0; j < 4; ++j) {
        for(int k = 0; k < 6; ++k) {
            M[matrixdim+(j+4)+2*matrixdim*k] = M[matrixdim+j+2*matrixdim*(k+2)] = -matcoeffs[coeffindex++];
        }
    }
    for(int j = 0; j < 4; ++j) {
        for(int k = 0; k < 6; ++k) {
            M[matrixdim+(j+4)+2*matrixdim*k+matrixdim*2*matrixdim] = M[matrixdim+j+2*matrixdim*(k+2)+matrixdim*2*matrixdim] = -matcoeffs[coeffindex++];
        }
    }
    for(int j = 0; j < 4; ++j) {
        for(int k = 0; k < 6; ++k) {
            A[(j+4)+matrixdim*k] = A[j+matrixdim*(k+2)] = matcoeffs[coeffindex++];
        }
        for(int k = 0; k < 2; ++k) {
            A[j+matrixdim*k] = A[(j+4)+matrixdim*(k+6)] = 0;
        }
    }
    const IkReal lfpossibilities[4][4] = {{1,-1,1,1},{1,0,-2,1},{1,1,2,0},{1,-1,4,1}};
    int lfindex = -1;
    bool bsingular = true;
    do {
        dgetrf_(&matrixdim,&matrixdim,A,&matrixdim,&ipiv[0],&info);
        if( info == 0 ) {
            bsingular = false;
            for(int j = 0; j < matrixdim; ++j) {
                if( IKabs(A[j*matrixdim+j]) < 100*tol ) {
                    bsingular = true;
                    break;
                }
            }
            if( !bsingular ) {
                break;
            }
        }
        if( lfindex == 3 ) {
            break;
        }
        // transform by the linear functional
        lfindex++;
        const IkReal* lf = lfpossibilities[lfindex];
        // have to reinitialize A
        coeffindex = 0;
        for(int j = 0; j < 4; ++j) {
            for(int k = 0; k < 6; ++k) {
                IkReal a = matcoeffs[coeffindex+48], b = matcoeffs[coeffindex+24], c = matcoeffs[coeffindex];
                A[(j+4)+matrixdim*k] = A[j+matrixdim*(k+2)] = lf[0]*lf[0]*a+lf[0]*lf[2]*b+lf[2]*lf[2]*c;
                M[matrixdim+(j+4)+2*matrixdim*k] = M[matrixdim+j+2*matrixdim*(k+2)] = -(lf[1]*lf[1]*a + lf[1]*lf[3]*b + lf[3]*lf[3]*c);
                M[matrixdim+(j+4)+2*matrixdim*k+matrixdim*2*matrixdim] = M[matrixdim+j+2*matrixdim*(k+2)+matrixdim*2*matrixdim] = -(2*lf[0]*lf[1]*a + (lf[0]*lf[3]+lf[1]*lf[2])*b + 2*lf[2]*lf[3]*c);
                coeffindex++;
            }
            for(int k = 0; k < 2; ++k) {
                A[j+matrixdim*k] = A[(j+4)+matrixdim*(k+6)] = 0;
            }
        }
    } while(lfindex<4);

    if( bsingular ) {
        return;
    }
    dgetrs_("No transpose", &matrixdim, &matrixdim2, A, &matrixdim, &ipiv[0], &M[matrixdim], &matrixdim2, &info);
    if( info != 0 ) {
        return;
    }

    // set identity in upper corner
    for(int j = 0; j < matrixdim; ++j) {
        M[matrixdim*2*matrixdim+j+matrixdim*2*j] = 1;
    }
    IkReal IKFAST_ALIGNED16(wr[16]);
    IkReal IKFAST_ALIGNED16(wi[16]);
    IkReal IKFAST_ALIGNED16(vr[16*16]);
    int one=1;
    dgeev_("N", "V", &matrixdim2, M, &matrixdim2, wr, wi,NULL, &one, vr, &matrixdim2, work, &worksize, &info);
    if( info != 0 ) {
        return;
    }
    IkReal Breal[matrixdim-1];
    for(int i = 0; i < matrixdim2; ++i) {
        // HACK should be tol*100
        if( IKabs(wi[i]) < 5e-5 ) {
            IkReal* ev = vr+matrixdim2*i;
            if( IKabs(wr[i]) > 1 ) {
                ev += matrixdim;
            }
            // consistency has to be checked!!
            if( IKabs(ev[0]) < tol ) {
                continue;
            }
            IkReal iconst = 1/ev[0];
            for(int j = 1; j < matrixdim; ++j) {
                Breal[j-1] = ev[j]*iconst;
            }
            if( %s(Breal) ) {
                if( lfindex >= 0 ) {
                    const IkReal* lf = lfpossibilities[lfindex];
                    rawroots[numroots++] = (wr[i]*lf[0]+lf[1])/(wr[i]*lf[2]+lf[3]);
                }
                else {
                    rawroots[numroots++] = wr[i];
                }
                bool bsmall0=IKabs(ev[0]) > IKabs(ev[2]);
                bool bsmall1=IKabs(ev[0]) > IKabs(ev[1]);
                if( bsmall0 && bsmall1 ) {
                    rawroots[numroots++] = ev[2]/ev[0];
                    rawroots[numroots++] = ev[1]/ev[0];
                }
                else if( bsmall0 && !bsmall1 ) {
                    rawroots[numroots++] = ev[3]/ev[1];
                    rawroots[numroots++] = ev[1]/ev[0];
                }
                else if( !bsmall0 && bsmall1 ) {
                    rawroots[numroots++] = ev[6]/ev[4];
                    rawroots[numroots++] = ev[7]/ev[6];
                }
                else if( !bsmall0 && !bsmall1 ) {
                    rawroots[numroots++] = ev[7]/ev[5];
                    rawroots[numroots++] = ev[7]/ev[6];
                }
            }
        }
    }
}"""%(name,checkconsistency8)
            self.functions[name] = fcode
        return name

    def using_checkconsistency16(self):
        name = 'checkconsistency16'
        if not name in self.functions:
            fcode = """
static inline bool %s(const IkReal* Breal)
{
    IkReal norm = 0.1;
    for(int i = 0; i < 15; ++i) {
        norm += IKabs(Breal[i]);
    }
    IkReal tol = 5e-5*norm; // have to increase the threshold since many computations are involved
    return IKabs(Breal[2]*Breal[2]-Breal[3]) < tol && IKabs(Breal[0]*Breal[4]-Breal[5]) < tol && IKabs(Breal[1]*Breal[4]-Breal[6]) < tol && IKabs(Breal[2]*Breal[4]-Breal[7]) < tol && IKabs(Breal[3]*Breal[4]-Breal[8]) < tol && IKabs(Breal[4]*Breal[4]-Breal[9]) < tol && IKabs(Breal[2]*Breal[9]-Breal[10]) < tol && IKabs(Breal[3]*Breal[9]-Breal[11]) < tol && IKabs(Breal[4]*Breal[9]-Breal[12]) < tol && IKabs(Breal[2]*Breal[12]-Breal[13]) < tol && IKabs(Breal[3]*Breal[12]-Breal[14]) < tol;
};
"""%name
            self.functions[name] = fcode
        return name
    def using_solvedialyticpoly16lep(self):
        name = 'solvedialyticpoly16lep'
        checkconsistency16=self.using_checkconsistency16()
        if not name in self.functions:
            fcode = """
/// \\brief Solve the det Bx-A = 0 problem
///
/// matcoeffs is of length 88*2, for 2 matrices
static inline void %s(const IkReal* matcoeffs, IkReal* rawroots, int& numroots)
{
    const IkReal tol = std::numeric_limits<IkReal>::epsilon();
    const int matrixdim = 16;
    IkReal IKFAST_ALIGNED16(M[2][matrixdim*matrixdim]);
    const int worksize=matrixdim*matrixdim*(matrixdim-1);
    IkReal IKFAST_ALIGNED16(work[worksize]);
    int ipiv[matrixdim];
    int info, coeffindex;
    const int secondrowindices[11] = {0,1,2,3,4,5,8,9,10,11,12};
    numroots = 0;
    coeffindex = 0;
    for(int degree = 0; degree < 2; ++degree) {
        for(int j = 0; j < 8; ++j) {
            for(int k = 0; k < 11; ++k) {
                M[degree][(j+8)+matrixdim*secondrowindices[k]] = M[degree][j+matrixdim*(k+5)] = matcoeffs[coeffindex++];
            }
            M[degree][j+matrixdim*0] = M[degree][j+matrixdim*1] = M[degree][j+matrixdim*2] = M[degree][j+matrixdim*3] = M[degree][j+matrixdim*4] = 0;
            M[degree][(j+8)+matrixdim*6] = M[degree][(j+8)+matrixdim*7] = M[degree][(j+8)+matrixdim*13] = M[degree][(j+8)+matrixdim*14] = M[degree][(j+8)+matrixdim*15] = 0;
        }
    }
    const IkReal lfpossibilities[2][4] = {{1,-1,1,1},{1,0,-2,1}};
    int lfindex = -1;
    IkReal polymultiplier = 0;
    bool bsingular = false;
    do {
        dgetrf_(&matrixdim,&matrixdim,M[1],&matrixdim,&ipiv[0],&info);
        if( info == 0 ) {
            bsingular = false;
            for(int j = 0; j < matrixdim; ++j) {
                if( IKabs(M[1][j*matrixdim+j]) < 100*tol ) {
                    bsingular = true;
                    break;
                }
            }
            if( !bsingular ) {
                break;
            }
        }
        else {
            bsingular = true;
        }
        if( lfindex == 1 ) {
            break;
        }
        // transform by the linear functional
        lfindex++;
        const IkReal* lf = lfpossibilities[lfindex];
        coeffindex = 0;
        for(int j = 0; j < 8; ++j) {
            for(int k = 0; k < 11; ++k) {
                IkReal b = matcoeffs[coeffindex+88], a = matcoeffs[coeffindex];
                M[0][(j+8)+matrixdim*secondrowindices[k]] = M[0][j+matrixdim*(k+5)] = lf[1]*b+lf[3]*a;
                M[1][(j+8)+matrixdim*secondrowindices[k]] = M[1][j+matrixdim*(k+5)] = lf[0]*b+lf[2]*a;
                ++coeffindex;
            }
            for(int degree = 0; degree < 2; ++degree ) {
                M[degree][j+matrixdim*0] = M[degree][j+matrixdim*1] = M[degree][j+matrixdim*2] = M[degree][j+matrixdim*3] = M[degree][j+matrixdim*4] = 0;
                M[degree][(j+8)+matrixdim*6] = M[degree][(j+8)+matrixdim*7] = M[degree][(j+8)+matrixdim*13] = M[degree][(j+8)+matrixdim*14] = M[degree][(j+8)+matrixdim*15] = 0;
            }
        }
    } while(lfindex<2);

    if( bsingular ) {
        // solve the generalized eigenvalue problem (qz algorithm)
        return;
    }
    dgetrs_("No transpose", &matrixdim, &matrixdim, M[1], &matrixdim, &ipiv[0], M[0], &matrixdim, &info);
    if( info != 0 ) {
        return;
    }

    IkReal IKFAST_ALIGNED16(wr[matrixdim]);
    IkReal IKFAST_ALIGNED16(wi[matrixdim]);
    IkReal IKFAST_ALIGNED16(vr[matrixdim*matrixdim]);
    int one=1;
    dgeev_("N", "V", &matrixdim, M[0], &matrixdim, wr, wi,NULL, &one, vr, &matrixdim, work, &worksize, &info);
    if( info != 0 ) {
        return;
    }
    // eigen values are negatives of the real solution
    IkReal Breal[matrixdim-1];
    for(int i = 0; i < matrixdim; ++i) {
        if( IKabs(wi[i]) < tol*10000 ) {
            IkReal* ev = vr+matrixdim*i;
            // consistency has to be checked!!
            if( IKabs(ev[0]) < tol ) {
                continue;
            }
            IkReal iconst = 1/ev[0];
            IkReal constsign = ev[0] > 0 ? IkReal(1) : IkReal(-1);
            for(int j = 1; j < matrixdim; ++j) {
                Breal[j-1] = ev[j]*iconst;
            }
            if( %s(Breal) ) {
                if( lfindex >= 0 ) {
                    const IkReal* lf = lfpossibilities[lfindex];
                    rawroots[numroots++] = (-wr[i]*lf[0]+lf[1])/(-wr[i]*lf[2]+lf[3]);
                }
                else {
                    rawroots[numroots++] = -wr[i];
                }
                bool bsmall0=IKabs(ev[0]) > IKabs(ev[5]);
                bool bsmall1=IKabs(ev[0]) > IKabs(ev[3]);
                if( bsmall0 && bsmall1 ) {
                    rawroots[numroots++] = ev[5]/ev[0];
                    rawroots[numroots++] = ev[3]/ev[0];
                    rawroots[numroots++] = IKatan2(constsign*ev[1],constsign*ev[2]);
                }
                else if( bsmall0 && !bsmall1 ) {
                    rawroots[numroots++] = ev[8]/ev[3];
                    rawroots[numroots++] = ev[4]/ev[3];
                    rawroots[numroots++] = IKatan2(constsign*ev[1],constsign*ev[2]);
                }
                else if( !bsmall0 && bsmall1 ) {
                    rawroots[numroots++] = ev[13]/ev[10];
                    rawroots[numroots++] = ev[14]/ev[13];
                    if( rawroots[numroots-2] < 0 ) {
                        constsign = -constsign;
                    }
                    rawroots[numroots++] = IKatan2(constsign*ev[6],constsign*ev[7]);
                }
                else if( !bsmall0 && !bsmall1 ) {
                    rawroots[numroots++] = ev[15]/ev[12];
                    rawroots[numroots++] = ev[15]/ev[14];
                    if( rawroots[numroots-2] < 0 ) {
                        constsign = -constsign;
                    }
                    rawroots[numroots++] = IKatan2(constsign*ev[6],constsign*ev[7]);
                }
            }
        }
    }
}
"""%(name,checkconsistency16)
            self.functions[name] = fcode
        return name
