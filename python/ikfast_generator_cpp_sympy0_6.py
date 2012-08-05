#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (Lesser GPL)
#
# Copyright (C) 2009-2012 Rosen Diankov
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
if sympy_version >= '0.7.0':
    raise ImportError('ikfast needs sympy 0.6.x')

import sys, copy, time, datetime
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

import logging
log = logging.getLogger('ikfast')

from sympy.core import function # for sympy 0.7.1+
class fmod(function.Function):
    nargs = 2
    is_real = True
    is_Function = True

class atan2check(atan2):
    nargs = 2
    is_real = True
    is_Function = True

def evalNumbers(expr):
    """Replaces all numbers with symbols, this is to make gcd faster when fractions get too big"""
    if expr.is_number:
        return expr.evalf()
    elif expr.is_Mul:
        result = S.One
        for arg in expr.args:
            result *= evalNumbers(arg)
    elif expr.is_Add:
        result = S.Zero
        for arg in expr.args:
            result += evalNumbers(arg)
    elif expr.is_Pow:
        # don't replace the exponent
        result = evalNumbers(expr.base)**expr.exp
    elif expr.is_Function:
        args = [evalNumbers(arg) for arg in expr.args]
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
    complexitysubs = [(Symbol('POW'),1),(Symbol('ADD'),1),(Symbol('MUL'),1)]
    reduced_exprs = []
    allexprs = []
    for expr in rawexprs:
        evalexpr = evalNumbers(expr)
        complexity = evalexpr.count_ops().subs(complexitysubs)
        # need to threshold complexity or otherwise cse will not terminate
        if complexity > 300:
            reduced_exprs.append(evalexpr)
        else:
            allexprs.append(evalexpr)
            reduced_exprs.append(None)

    newreplacements = []
    if len(allexprs)>0:
        replacements,reduced_exprs2 = cse(allexprs,symbols=symbols)
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
    def __init__(self,kinematicshash='',version=''):
        self.symbolgen = cse_main.numbered_symbols('x')
        self.strprinter = printing.StrPrinter()
        self.freevars = None # list of free variables in the solution
        self.freevardependencies = None # list of variables depending on the free variables
        self.functions = dict()
        self.kinematicshash=kinematicshash
        self.resetequations() # dictionary of symbols already written
        self.version=version

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
/// To compile with gcc:
///     gcc -lstdc++ ik.cpp
/// To compile without any main function as a shared object (might need -llapack):
///     gcc -fPIC -lstdc++ -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -shared -Wl,-soname,libik.so -o libik.so ik.cpp
#define IKFAST_HAS_LIBRARY
#include "ikfast.h"
using namespace ikfast;

// check if the included ikfast version matches what this file was compiled with
#define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
IKFAST_COMPILE_ASSERT(IKFAST_VERSION==%s);

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <complex>

#define IKFAST_STRINGIZE2(s) #s
#define IKFAST_STRINGIZE(s) IKFAST_STRINGIZE2(s)

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

// allows asin and acos to exceed 1
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)0.000001)
#endif

// used to check input to atan2 for degenerate cases
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)2e-6)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
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

"""%(self.version,str(datetime.datetime.now()),self.version)
        code += solvertree.generate(self)
        code += solvertree.end(self)

        code += """

/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

IKFAST_API const char* GetKinematicsHash() { return "%s"; }

IKFAST_API const char* GetIkFastVersion() { return IKFAST_STRINGIZE(IKFAST_VERSION); }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif
"""%(self.kinematicshash)

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
        code += 'IkReal ' + ','.join(usedvars) + ';\n'
        code += 'unsigned char ' + ','.join('_i%s[2], _n%s'%(var[0].name,var[0].name) for var in node.solvejointvars+node.freejointvars) + ';\n\n'
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
            subexprs,reduced_exprs=customcse (node.Tfk[0:3,0:4].subs([(v[0],Symbol('j[%d]'%v[1])) for v in allvars]),self.symbolgen)
            outputnames = ['eerot[0]','eerot[1]','eerot[2]','eetrans[0]','eerot[3]','eerot[4]','eerot[5]','eetrans[1]','eerot[6]','eerot[7]','eerot[8]','eetrans[2]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += self.indentCode(fcode,4)
            code += '}\n\n'
        code += self.getClassInit(node,IkType.Transform6D)
        code += self.GetIkFunctionPreamble(node)
        fcode = ''
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d]; c%s=cos(pfree[%d]); s%s=sin(pfree[%d]);\n'%(name,i,name,i,name,i)
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
            for var,value in node.dictequations:
                fcode += self.writeEquations(lambda k: var,value)
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"

        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += self.indentCode(functioncode,4)
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
            subexprs,reduced_exprs=customcse (node.Rfk[0:3,0:3].subs([(v[0],Symbol('j[%d]'%v[1])) for v in allvars]),self.symbolgen)
            outputnames = ['eerot[0]','eerot[1]','eerot[2]','eerot[3]','eerot[4]','eerot[5]','eerot[6]','eerot[7]','eerot[8]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += self.indentCode(fcode,4)
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
            for var,value in node.dictequations:
                fcode += self.writeEquations(lambda k: var,value)
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"
        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += self.indentCode(functioncode,4)
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
            subexprs,reduced_exprs=customcse (eqs,self.symbolgen)
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
            code += self.indentCode(fcode,4)
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
            for var,value in node.dictequations:
                fcode += self.writeEquations(lambda k: var,value)
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"
        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += self.indentCode(functioncode,4)
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
            subexprs,reduced_exprs=customcse (eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += self.indentCode(fcode,4)
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
            for var,value in node.dictequations:
                fcode += self.writeEquations(lambda k: var,value)
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"
        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += self.indentCode(functioncode,4)
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
            subexprs,reduced_exprs=customcse (eqs,self.symbolgen)
            outputnames = ['eerot[0]','eerot[1]','eerot[2]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += self.indentCode(fcode,4)
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
            for var,value in node.dictequations:
                fcode += self.writeEquations(lambda k: var,value)
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"

        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += self.indentCode(functioncode,4)
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
            subexprs,reduced_exprs=customcse (eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]','eetrans[2]','eerot[0]','eerot[1]','eerot[2]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += self.indentCode(fcode,4)
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
            for var,value in node.dictequations:
                fcode += self.writeEquations(lambda k: var,value)
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"

        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += self.indentCode(functioncode,4)
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
            subexprs,reduced_exprs=customcse (eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]','eetrans[2]','eerot[0]','eerot[1]','eerot[2]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += self.indentCode(fcode,4)
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
            for var,value in node.dictequations:
                fcode += self.writeEquations(lambda k: var,value)
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn solutions.GetNumSolutions()>0;\n}\n\n"
        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += self.indentCode(functioncode,4)
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
            subexprs,reduced_exprs=customcse (eqs,self.symbolgen)
            outputnames = ['eetrans[0]','eetrans[1]','eetrans[2]','eerot[0]']
            fcode = ''
            if len(subexprs) > 0:
                vars = [var for var,expr in subexprs]
                fcode = 'IkReal ' + ','.join(str(var) for var,expr in subexprs) + ';\n'
                for var,expr in subexprs:
                    fcode += self.writeEquations(lambda k: str(var),collect(expr,vars))
            for i in range(len(outputnames)):
                fcode += self.writeEquations(lambda k: outputnames[i],reduced_exprs[i])
            code += self.indentCode(fcode,4)
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
            for var,value in node.dictequations:
                fcode += self.writeEquations(lambda k: var,value)
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn solutions.GetNumSolutions()>0;\n}\n"

        # write other functions
        for name,functioncode in self.functions.iteritems():
            code += self.indentCode(functioncode,4)
        code += "};\n"
        return code
    def endSolverIKChainAxisAngle(self, node):
        return ''


    def generateSolution(self, node,declarearray=True,acceptfreevars=True):
        """writes the solution of one variable
        :param declarearray: if False, will return the equations to be written without evaluating them. Used for conditioned solutions.
        """
        code = ''
        numsolutions = 0
        eqcode = ''
        name = node.jointname
        node.HasFreeVar = False
        allnumsolutions = 0
        for var,value in node.dictequations:
            eqcode += 'IkReal %s;\n'%var
            eqcode += self.writeEquations(lambda k: var,value)

        if node.jointeval is not None:
            numsolutions = len(node.jointeval)
            equations = []
            names = []
            for i,expr in enumerate(node.jointeval):
                if acceptfreevars:
                    m = None
                    for freevar in self.freevars:
                        if expr.has_any_symbols(Symbol(freevar)):
                            # has free variables, so have to look for a*freevar+b form
                            a = Wild('a',exclude=[Symbol(freevar)])
                            b = Wild('b',exclude=[Symbol(freevar)])
                            m = expr.match(a*Symbol(freevar)+b)
                            if m is not None:
                                self.freevardependencies.append((freevar,name))
                                assert(len(node.jointeval)==1)
                                code += 'IkReal ' + self.writeEquations(lambda i: '%smul'%name, m[a])
                                code += self.writeEquations(lambda i: name, m[b])
                                node.HasFreeVar = True
                                return code
                            else:
                                log.error('failed to extract free variable %s for %s from: eq=%s', freevar,node.jointname, expr)
    #                             m = dict()
    #                             m[a] = Real(-1,30)
    #                             m[b] = Real(0,30)

                equations.append(expr)
                names.append('%sarray[%d]'%(name,allnumsolutions+i))
                equations.append(sin(Symbol('%sarray[%d]'%(name,allnumsolutions+i))))
                names.append('s%sarray[%d]'%(name,allnumsolutions+i))
                equations.append(cos(Symbol('%sarray[%d]'%(name,allnumsolutions+i))))
                names.append('c%sarray[%d]'%(name,allnumsolutions+i))
            eqcode += self.writeEquations(lambda i: names[i], equations)
            if node.AddPiIfNegativeEq:
                for i in range(numsolutions):
                    eqcode += '%sarray[%d] = %sarray[%d] > 0 ? %sarray[%d]-IKPI : %sarray[%d]+IKPI;\n'%(name,allnumsolutions+numsolutions+i,name,allnumsolutions+i,name,allnumsolutions+i,name,allnumsolutions+i)
                    eqcode += 's%sarray[%d] = -s%sarray[%d];\n'%(name,allnumsolutions+numsolutions+i,name,allnumsolutions+i)
                    eqcode += 'c%sarray[%d] = -c%sarray[%d];\n'%(name,allnumsolutions+numsolutions+i,name,allnumsolutions+i)
                numsolutions *= 2
            for i in range(numsolutions):
                if node.isHinge:
                    eqcode += 'if( %sarray[%d] > IKPI )\n{\n    %sarray[%d]-=IK2PI;\n}\nelse if( %sarray[%d] < -IKPI )\n{    %sarray[%d]+=IK2PI;\n}\n'%(name,allnumsolutions+i,name,allnumsolutions+i,name,allnumsolutions+i,name,allnumsolutions+i)
                eqcode += '%svalid[%d] = true;\n'%(name,allnumsolutions+i)
            allnumsolutions += numsolutions
        # might also have cos solutions ...
        if node.jointevalcos is not None:
            numsolutions = 2*len(node.jointevalcos)
            eqcode += self.writeEquations(lambda i: 'c%sarray[%d]'%(name,allnumsolutions+2*i),node.jointevalcos)
            for i in range(len(node.jointevalcos)):
                eqcode += 'if( c%sarray[%d] >= -1-IKFAST_SINCOS_THRESH && c%sarray[%d] <= 1+IKFAST_SINCOS_THRESH )\n{\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                eqcode += '    %svalid[%d] = %svalid[%d] = true;\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i+1)
                eqcode += '    %sarray[%d] = IKacos(c%sarray[%d]);\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                eqcode += '    s%sarray[%d] = IKsin(%sarray[%d]);\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                # second solution
                eqcode += '    c%sarray[%d] = c%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i)
                eqcode += '    %sarray[%d] = -%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i)
                eqcode += '    s%sarray[%d] = -s%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i)
                eqcode += '}\n'
                eqcode += 'else if( isnan(c%sarray[%d]) )\n{\n'%(name,allnumsolutions+2*i)
                eqcode += '    // probably any value will work\n'
                eqcode += '    %svalid[%d] = true;\n'%(name,allnumsolutions+2*i)
                eqcode += '    c%sarray[%d] = 1; s%sarray[%d] = 0; %sarray[%d] = 0;\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                eqcode += '}\n'
            allnumsolutions += numsolutions

        if node.jointevalsin is not None:
            numsolutions = 2*len(node.jointevalsin)
            eqcode += self.writeEquations(lambda i: 's%sarray[%d]'%(name,allnumsolutions+2*i),node.jointevalsin)
            for i in range(len(node.jointevalsin)):
                eqcode += 'if( s%sarray[%d] >= -1-IKFAST_SINCOS_THRESH && s%sarray[%d] <= 1+IKFAST_SINCOS_THRESH )\n{\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                eqcode += '    %svalid[%d] = %svalid[%d] = true;\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i+1)
                eqcode += '    %sarray[%d] = IKasin(s%sarray[%d]);\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                eqcode += '    c%sarray[%d] = IKcos(%sarray[%d]);\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                # second solution
                eqcode += '    s%sarray[%d] = s%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i)
                eqcode += '    %sarray[%d] = %sarray[%d] > 0 ? (IKPI-%sarray[%d]) : (-IKPI-%sarray[%d]);\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i,name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                eqcode += '    c%sarray[%d] = -c%sarray[%d];\n'%(name,allnumsolutions+2*i+1,name,allnumsolutions+2*i)
                eqcode += '}\n'
                eqcode += 'else if( isnan(s%sarray[%d]) )\n{\n'%(name,allnumsolutions+2*i)
                eqcode += '    // probably any value will work\n'
                eqcode += '    %svalid[%d] = true;\n'%(name,allnumsolutions+2*i)
                eqcode += '    c%sarray[%d] = 1; s%sarray[%d] = 0; %sarray[%d] = 0;\n'%(name,allnumsolutions+2*i,name,allnumsolutions+2*i,name,allnumsolutions+2*i)
                eqcode += '}\n'
            allnumsolutions += numsolutions

        if not declarearray:
            return eqcode,allnumsolutions

        code += '{\nIkReal %sarray[%d], c%sarray[%d], s%sarray[%d];\n'%(name,allnumsolutions,name,allnumsolutions,name,allnumsolutions)
        code += 'bool %svalid[%d]={false};\n'%(name,allnumsolutions)
        code += '_n%s = %d;\n'%(name,allnumsolutions)
        code += eqcode
        if allnumsolutions > 1:
            if allnumsolutions >= 256:
                log.error('num solutions is %d>=256, which exceeds unsigned char',allnumsolutions)
        code += 'for(int i%s = 0; i%s < %d; ++i%s)\n{\n'%(name,name,allnumsolutions,name)
        code += 'if( !%svalid[i%s] )\n{\n    continue;\n}\n'%(name,name)
        code += '_i%s[0] = i%s; _i%s[1] = -1;\n'%(name,name,name)
        # check for a similar solution
        code += 'for(int ii%s = i%s+1; ii%s < %d; ++ii%s)\n{\n'%(name,name,name,allnumsolutions,name)
        code += 'if( %svalid[ii%s] && IKabs(c%sarray[i%s]-c%sarray[ii%s]) < IKFAST_SOLUTION_THRESH && IKabs(s%sarray[i%s]-s%sarray[ii%s]) < IKFAST_SOLUTION_THRESH )\n{\n    %svalid[ii%s]=false; _i%s[1] = ii%s; break; \n}\n'%(name,name,name,name,name,name,name,name,name,name,name,name,name,name)
        code += '}\n'
        code += '%s = %sarray[i%s]; c%s = c%sarray[i%s]; s%s = s%sarray[i%s];\n'%(name,name,name,name,name,name,name,name,name)
        if node.AddHalfTanValue:
            code += 'ht%s = IKtan(%s/2);\n'%(name,name)
        if node.getEquationsUsed() is not None and len(node.getEquationsUsed()) > 0:
            code += '{\nIkReal evalcond[%d];\n'%len(node.getEquationsUsed())
            code += self.writeEquations(lambda i: 'evalcond[%d]'%(i),node.getEquationsUsed())
            code += 'if( '
            for i in range(len(node.getEquationsUsed())):
                if i != 0:
                    code += ' || '
                # using smaller node.thresh increases the missing solution rate, really not sure whether the solutions themselves
                # are bad due to double precision arithmetic, or due to singularities
                code += 'IKabs(evalcond[%d]) > 0.000001 '%(i)
            code += ' )\n{\n'
            #code += 'cout <<'
            #code += '<<'.join(['evalcond[%d]'%i for i in range(len(node.getEquationsUsed()))])
            #code += '<< endl;'
            code += 'continue;\n}\n'
            code += '}\n'
        code += '\n'
        return code

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
        for var,value in node.dictequations:
            checkcode += 'IkReal %s;\n'%var
            checkcode += self.writeEquations(lambda k: var,value)
        for solversolution in node.solversolutions:
            assert len(solversolution.checkforzeros) > 0
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
            checkcode += ' )\n{\n'
            scode,numsolutions = self.generateSolution(solversolution,declarearray=False,acceptfreevars=False)
            scode += 'numsolutions%s = %d;\n'%(name,numsolutions)
            allnumsolutions = max(allnumsolutions,numsolutions)
            checkcode += self.indentCode(scode,4)
            checkcode += '\n} else\n'
        checkcode += '{\n    continue;\n}\n'  # if got here, then current solution branch is not good, so skip
        checkcode += '}\n'*len(node.solversolutions)
        checkcode += 'if( numsolutions%s == 0 )\n{\n    continue;\n}\n'%name

        code = '{\nIkReal evalcond[%d]; int numsolutions%s = 0;\n'%(maxchecks,name)
        code += 'IkReal %sarray[%d], c%sarray[%d], s%sarray[%d];\n'%(name,allnumsolutions,name,allnumsolutions,name,allnumsolutions)
        code += 'bool %svalid[%d]={false};\n'%(name,allnumsolutions)
        code += '_n%s = %d;\n'%(name,allnumsolutions)
        code += self.indentCode(checkcode,4)
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
        D=node.poly.degree
        polyroots=self.using_polyroots(D)
        name = node.jointname
        polyvar = node.poly.symbols[0].name
        code = 'IkReal op[%d+1], zeror[%d];\nint numroots;\n'%(D,D)
        numevals = 0
        if node.postcheckforzeros is not None:
            numevals = max(numevals,len(node.postcheckforzeros))
        if node.postcheckfornonzeros is not None:
            numevals = max(numevals,len(node.postcheckfornonzeros))
        if node.postcheckforrange is not None:
            numevals = max(numevals,len(node.postcheckforrange))
        if numevals > 0:
            code += 'IkReal %sevalpoly[%d];\n'%(name,numevals)
        for var,value in node.dictequations:
            code += 'IkReal %s;\n'%var
            code += self.writeEquations(lambda k: var,value)
        code += self.writeEquations(lambda i: 'op[%d]'%(i),[node.poly.coeff(i) for i in range(D,-1,-1)])
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
        code += self.indentCode(fcode,4)
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
                fcode += 'IKabs(%sevalpoly[%d]) < %.16f '%(name,i,node.thresh)
            fcode += ' )\n{\n    continue;\n}\n'
            code += self.indentCode(fcode,4)
        if node.postcheckfornonzeros is not None and len(node.postcheckfornonzeros) > 0:
            fcode = self.writeEquations(lambda i: '%sevalpoly[%d]'%(name,i),node.postcheckfornonzeros)
            fcode += 'if( '
            for i in range(len(node.postcheckfornonzeros)):
                if i != 0:
                    fcode += ' || '
                fcode += 'IKabs(%sevalpoly[%d]) > %.16f '%(name,i,node.thresh)
            fcode += ' )\n{\n    continue;\n}\n'
            code += self.indentCode(fcode,4)
        if node.postcheckforrange is not None and len(node.postcheckforrange) > 0:
            fcode = self.writeEquations(lambda i: '%sevalpoly[%d]'%(name,i),node.postcheckforrange)
            fcode += 'if( '
            for i in range(len(node.postcheckforrange)):
                if i != 0:
                    fcode += ' || '
                fcode += ' (%sevalpoly[%d] < %.16f || %sevalpoly[%d] > %.16f) '%(name,i,-1.0-node.thresh,name,i,1.0+node.thresh)
            fcode += ' )\n{\n    continue;\n}\n'
            code += self.indentCode(fcode,4)

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
        code = 'IkReal op[%d], zeror[%d];\nint numroots;\n'%(len(node.exportcoeffeqs),node.rootmaxdim*len(node.jointnames))
        numevals = 0
        code += self.writeEquations(lambda i: 'op[%d]'%(i),node.exportcoeffeqs)
        code += "%s(op,zeror,numroots);\n"%(fnname)
        code += 'IkReal '
        for i,name in enumerate(node.jointnames):
            code += '%sarray[%d], c%sarray[%d], s%sarray[%d]'%(name,node.rootmaxdim,name,node.rootmaxdim,name,node.rootmaxdim)
            if i+1 < len(node.jointnames):
                code += ', '
            else:
                code += ';\n'
        code += 'int numsolutions = 0;\n'
        code += 'for(int i%s = 0; i%s < numroots; i%s += %d)\n{\n'%(firstname,firstname,firstname,len(node.jointnames))
        fcode = 'IkReal '
        for i in range(len(node.exportvar)):
            fcode += '%s = zeror[i%s+%d]'%(node.exportvar[i],firstname,i)
            if i+1<len(node.exportvar):
                fcode += ', '
            else:
                fcode += ';\n'
        origequations = self.copyequations()
        fcode += self.writeEquations(lambda i: '%sarray[numsolutions]'%(node.jointnames[i]), node.jointeval)
        fcode += self.writeEquations(lambda i: 'c%sarray[numsolutions]'%(node.jointnames[i]), node.jointevalcos)
        fcode += self.writeEquations(lambda i: 's%sarray[numsolutions]'%(node.jointnames[i]), node.jointevalsin)
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
        code += self.indentCode(fcode,4)

        code += 'bool %svalid[%d]={%s};\n'%(firstname,node.rootmaxdim,','.join(['true']*node.rootmaxdim))
        code += '_n%s = %d;\n'%(firstname,node.rootmaxdim)
        for name in node.jointnames[1:]:
            code += '_n%s = 1;\n'%name
        if node.rootmaxdim >= 256:
            log.error('num solutions is %d>=256, which exceeds unsigned char',node.rootmaxdim)
        
        code += 'for(int i%s = 0; i%s < numsolutions; ++i%s)\n    {\n'%(firstname,firstname,firstname)
        code += 'if( !%svalid[i%s] )\n{\n    continue;\n}\n'%(firstname,firstname)
        code += '_i%s[0] = i%s; _i%s[1] = -1;\n'%(firstname,firstname,firstname)
        for name in node.jointnames[1:]:
            code += '_i%s[0] = 0; _i%s[1] = -1;\n'%(name,name)
            
        # check for a similar solution
        code += 'for(int ii%s = i%s+1; ii%s < numsolutions; ++ii%s)\n{\n'%(firstname,firstname,firstname,firstname)
        code += 'if( !%svalid[ii%s] ) { continue; }\n'%(firstname,firstname)
        code += 'if( '
        for name in node.jointnames:
            code += 'IKabs(c%sarray[i%s]-c%sarray[ii%s]) < IKFAST_SOLUTION_THRESH && IKabs(s%sarray[i%s]-s%sarray[ii%s]) < IKFAST_SOLUTION_THRESH && '%(name,firstname,name,firstname,name,firstname,name,firstname)
        code += ' 1 )\n{\n    %svalid[ii%s]=false; '%(firstname,firstname)
        code += '_i%s[1] = ii%s; '%(firstname,firstname)
        for name in node.jointnames[1:]:
            code += '_i%s[1] = 0; '%name
        code += ' break; \n}\n'
        code += '}\n'
        
        for name in node.jointnames:
            code += '    %s = %sarray[i%s]; c%s = c%sarray[i%s]; s%s = s%sarray[i%s];\n\n'%(name,name,firstname,name,name,firstname,name,name,firstname)
        return code
    
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
        origequations = self.copyequations()
        code = '{\n'
        numevals = None
        for branch in node.jointbranches:
            if branch[0] is not None:
                if numevals is None or numevals < len(branch[0]):
                    numevals=len(branch[0])
        if numevals is not None:
            code += 'IkReal evalcond[%d];\n'%numevals
        for branch in node.jointbranches:
            self.dictequations = self.copyequations(origequations)
            if branch[0] is None:
                branchcode = 'if( 1 )\n{\n'
            else:
                branchcode = self.writeEquations(lambda x: 'evalcond[%d]'%x,branch[0])
                branchcode += 'if( '
                for i in range(len(branch[0])):
                    if i != 0:
                        branchcode += ' && '
                    branchcode += 'IKabs(evalcond[%d]) < %.16f '%(i,node.thresh)
                branchcode += ' )\n{\n'
            for n in branch[1]:
                branchcode += n.generate(self)
            for n in reversed(branch[1]):
                branchcode += n.end(self)
            code += self.indentCode(branchcode,4)+'\n} else\n{\n'
        code += '}\n'*(len(node.jointbranches)+1)
        self.dictequations = origequations
        return code
    def endBranchConds(self, node):
        return ''
    def generateCheckZeros(self, node):
        origequations = self.copyequations()
        name = node.jointname if node.jointname is None else 'dummy'
        code = 'IkReal %seval[%d];\n'%(name,len(node.jointcheckeqs))
        for var,value in node.dictequations:
            code += 'IkReal %s;\n'%var
            code += self.writeEquations(lambda k: var,value)
        code += self.writeEquations(lambda i: '%seval[%d]'%(name,i),node.jointcheckeqs)
        if len(node.jointcheckeqs) > 0:
            code += 'if( '
            for i in range(len(node.jointcheckeqs)):
                if i != 0:
                    if node.anycondition:
                        code += ' || '
                    else:
                        code += ' && '
                code += 'IKabs(%seval[%d]) < %.16f '%(name,i,node.thresh)
            code += ' )\n{\n'
            self.dictequations = self.copyequations(origequations)
            code += self.indentCode(self.generateTree(node.zerobranch),4)
            code += '\n} else\n'
        code += '{\n'
        self.dictequations = self.copyequations(origequations)
        code += self.indentCode(self.generateTree(node.nonzerobranch),4)
        code += '\n}\n'
        self.dictequations = origequations
        return '{\n' + self.indentCode(code,4) + '\n}\n'
    def endCheckZeros(self, node):
        return ''
    def generateFreeParameter(self, node):
        #print 'free variable ',node.jointname,': ',self.freevars
        self.freevars.append(node.jointname)
        self.freevardependencies.append((node.jointname,node.jointname))
        code = 'IkReal %smul = 1;\n%s=0;\n'%(node.jointname,node.jointname)
        return code+self.generateTree(node.jointtree)
    def endFreeParameter(self, node):
        self.freevars.pop()
        self.freevardependencies.pop()
        return ''
    def generateBreak(self,node):
        return 'continue;\n'
    def endBreak(self,node):
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
            code += self.indentCode(self.writeEquations(lambda i: names[i],listequations),4)
            code += self.indentCode(self.generateTree(node.jointtree),4)
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
        code = ''
        if node.checkgreaterzero is not None and len(node.checkgreaterzero) > 0:
            origequations = self.copyequations()
            code += 'IkReal soleval[%d];\n'%(len(node.checkgreaterzero))
            code += self.writeEquations(lambda i: 'soleval[%d]'%(i),node.checkgreaterzero)
            code += 'if( '
            for i in range(len(node.checkgreaterzero)):
                if i != 0:
                    code += ' && '
                code += 'soleval[%d] > %.16f '%(i,node.thresh)
            code += ' )\n'
            self.dictequations = origequations
        code += '{\n'
        code += 'std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(%d);\n'%len(node.alljointvars)
        for i,var in enumerate(node.alljointvars):
            offsetvalue = '+%.15e'%node.offsetvalues[i] if node.offsetvalues is not None else ''
            code += 'vinfos[%d].jointtype = %d;\n'%(i,0x01 if node.isHinge[i] else 0x11)
            code += 'vinfos[%d].foffset = %s%s;\n'%(i,var,offsetvalue)
            vardeps = [vardep for vardep in self.freevardependencies if vardep[1]==var.name]
            if len(vardeps) > 0:
                freevarname = vardeps[0][0]
                ifreevar = [j for j in range(len(self.freevars)) if freevarname==self.freevars[j]]
                code += 'vinfos[%d].fmul = %smul;\n'%(i,var.name)
                code += 'vinfos[%d].freeind = %d;\n'%(i,ifreevar[0])
                code += 'vinfos[%d].maxsolutions = 0;\n'%(i)
            else:
                code += 'vinfos[%d].indices[0] = _i%s[0];\n'%(i,var)
                code += 'vinfos[%d].indices[1] = _i%s[1];\n'%(i,var)
                code += 'vinfos[%d].maxsolutions = _n%s;\n'%(i,var)
        code += 'std::vector<int> vfree(%d);\n'%len(self.freevars)
        for i,varname in enumerate(self.freevars):
            ind = [j for j in range(len(node.alljointvars)) if varname==node.alljointvars[j].name]
            code += 'vfree[%d] = %d;\n'%(i,ind[0])
        code += 'solutions.AddSolution(vinfos,vfree);\n'
        code += '}\n'
        return code
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

    def writeEquations(self, varnamefn, allexprs):
        if not hasattr(allexprs,'__iter__') and not hasattr(allexprs,'__array__'):
            allexprs = [allexprs]
        code = ''
        # calling cse on many long expressions will freeze it, so try to divide the problem
        complexitysubs = [(Symbol('POW'),1),(Symbol('ADD'),1),(Symbol('MUL'),1)]
        complexity = [expr.count_ops().subs(complexitysubs) for expr in allexprs]
        complexitythresh = 4000
        exprs = []
        curcomplexity = 0
        for i,expr in enumerate(allexprs):
            curcomplexity += complexity[i]
            exprs.append(expr)
            if curcomplexity > complexitythresh or i == len(allexprs)-1:
                code += self._writeEquations(varnamefn,exprs,i+1-len(exprs))
                exprs = []
                curcomplexity = 0
        assert(len(exprs)==0)
        return code
    def _writeEquations(self, varnamefn, exprs,ioffset):
        code = ''
        replacements,reduced_exprs = customcse(exprs,symbols=self.symbolgen)
        N = len(self.dictequations[0])
        complexitysubs = [(Symbol('POW'),1),(Symbol('ADD'),1),(Symbol('MUL'),1)]
        for rep in replacements:
            comparerep = rep[1].subs(self.dictequations[0]).expand()
            found = False
            complexity = rep[1].count_ops().subs(complexitysubs)
            maxcomplexity = 3 if N > 1000 else 2
            if complexity > maxcomplexity: # check only long expressions
                for i in range(N):
                    if self.dictequations[1][i] is not None and comparerep-self.dictequations[1][i]==S.Zero:
                        #self.dictequations.append((rep[0],self.dictequations[0][i][0],self.dictequations[1][i]))
                        code += 'IkReal %s=%s;\n'%(rep[0],self.dictequations[0][i][0])
                        found = True
                        break
            else:
                comparerep = None
            if not found:
                self.dictequations[0].append(rep)
                self.dictequations[1].append(comparerep)
                code2,sepcode2 = self.writeExprCode(rep[1])
                code += sepcode2+'IkReal %s=%s;\n'%(rep[0],code2)
        for i,rexpr in enumerate(reduced_exprs):
            code2,sepcode2 = self.writeExprCode(rexpr)
            code += sepcode2+'%s=%s;\n'%(varnamefn(i+ioffset), code2)
        return code

    def writeExprCode(self, expr):
        # go through all arguments and chop them
        code = ''
        sepcode = ''
        if expr.is_Function:
            if expr.func == abs:
                code += 'IKabs('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2
            elif expr.func == sign:
                code += 'IKsign('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2
            elif expr.func == acos:
                code += 'IKacos('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2
                sepcode += 'if( (%s) < -1-IKFAST_SINCOS_THRESH || (%s) > 1+IKFAST_SINCOS_THRESH )\n    continue;\n'%(code2,code2)
            elif expr.func == asin:
                code += 'IKasin('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2
                sepcode += 'if( (%s) < -1-IKFAST_SINCOS_THRESH || (%s) > 1+IKFAST_SINCOS_THRESH )\n    continue;\n'%(code2,code2)
            elif expr.func == atan2check:
                code += 'IKatan2('
                # check for divides by 0 in arguments, this could give two possible solutions?!?
                # if common arguments is nan! solution is lost!
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2+', '
                code3,sepcode2 = self.writeExprCode(expr.args[1])
                code += code3
                sepcode += sepcode2
                sepcode += 'if( IKabs(%s) < IKFAST_ATAN2_MAGTHRESH && IKabs(%s) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(%s)+IKsqr(%s)-1) <= IKFAST_SINCOS_THRESH )\n    continue;\n'%(code2,code3,code2,code3)
            elif expr.func == atan2:
                code += 'IKatan2('
                # check for divides by 0 in arguments, this could give two possible solutions?!?
                # if common arguments is nan! solution is lost!
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2+', '
                code3,sepcode2 = self.writeExprCode(expr.args[1])
                code += code3
                sepcode += sepcode2
                sepcode += 'if( IKabs(%s) < IKFAST_ATAN2_MAGTHRESH && IKabs(%s) < IKFAST_ATAN2_MAGTHRESH )\n    continue;\n'%(code2,code3)
            elif expr.func == sin:
#                 if expr.args[0].is_Symbol and expr.args[0].name[0] == 'j':
#                     # probably already have initialized
#                     code += '(s%s'%expr.args[0].name
#                 else:
                code += 'IKsin('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2
            elif expr.func == cos:
#                 if expr.args[0].is_Symbol and expr.args[0].name[0] == 'j':
#                     # probably already have initialized
#                     code += '(c%s'%expr.args[0].name
#                 else:
                code += 'IKcos('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2
            elif expr.func == fmod:
                code += 'IKfmod('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2+', '
                code3,sepcode2 = self.writeExprCode(expr.args[1])
                code += code3
                sepcode += sepcode2
            else:
                code += expr.func.__name__ + '('
                for arg in expr.args:
                    code2,sepcode2 = self.writeExprCode(arg)
                    code += code2
                    sepcode += sepcode2
                    if not arg == expr.args[-1]:
                        code += ','
            return code + ')',sepcode
        elif expr.is_number:
            return self.strprinter.doprint(expr.evalf()),sepcode
        elif expr.is_Mul:
            code += '('
            for arg in expr.args:
                code2,sepcode2 = self.writeExprCode(arg)
                code += '('+code2+')'
                sepcode += sepcode2
                if not arg == expr.args[-1]:
                    code += '*'
            return code + ')',sepcode
        elif expr.is_Pow:
            exprbase,sepcode = self.writeExprCode(expr.base)
            if expr.exp.is_number:
                if expr.exp.is_integer and expr.exp.evalf() > 0:
                    code += '('+exprbase+')'
                    for i in range(1,expr.exp.evalf()):
                        code += '*('+exprbase+')'
                    return code,sepcode
                elif expr.exp-0.5 == S.Zero:
                    sepcode += 'if( (%s) < (IkReal)-0.00001 )\n    continue;\n'%exprbase
                    return 'IKsqrt('+exprbase+')',sepcode
                elif expr.exp+1 == S.Zero:
                    # check if exprbase is 0
                    return '((IKabs('+exprbase+') != 0)?((IkReal)1/('+exprbase+')):(IkReal)1.0e30)',sepcode
                elif expr.exp.is_integer and expr.exp.evalf() < 0:
                    # check if exprbase is 0
                    fcode = '('+exprbase+')'
                    for i in range(1,-expr.exp.evalf()):
                        fcode += '*('+exprbase+')'
                    return '((IKabs('+exprbase+') != 0)?((IkReal)1/('+fcode+')):(IkReal)1.0e30)',sepcode
                elif expr.exp < 0:
                    # check if exprbase is 0
                    return '((IKabs('+exprbase+') != 0)?(pow(' + exprbase + ',' + str(expr.exp.evalf()) + ')):(IkReal)1.0e30)',sepcode
                    
            exprexp,sepcode2 = self.writeExprCode(expr.exp)
            sepcode += sepcode2
            return 'pow(' + exprbase + ',' + exprexp + ')',sepcode
        elif expr.is_Add:
            code += '('
            for arg in expr.args:
                code2,sepcode2 = self.writeExprCode(arg)
                code += '('+code2+')'
                sepcode += sepcode2
                if not arg == expr.args[-1]:
                    code += '+'
            return code + ')',sepcode

        return self.strprinter.doprint(expr.evalf()),sepcode

    def indentCode(self, code, numspaces):
        # actually a lot of time can be wasted in this phase...
        if True:
            return code
        numspaces /= 4
        try:
            return re.sub('\n','\n'+' '*numspaces,s)
        except:
            lcode = list(code)
            locations = [i for i in range(len(lcode)-1) if lcode[i]=='\n']
            locations.reverse()
            insertcode = [' ' for i in range(numspaces)]
            for loc in locations:
                lcode[loc+1:0] = insertcode
            lcode[:0] = insertcode
            return ''.join(lcode)

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
            else:
                # Durand-Kerner polynomial root finding method
                # In case of multiple roots, see Pierre Fraigniaud's work on Quadratic-Like Convergence of the Mean
                # http://www.springerlink.com/content/t72g1635574u10q3/
                # the following poly has 2 double roots and requires 105-110 steps to get accurate roots (-2.48003364697210,  -0.266182031226875, -0.399192786087098, 5.23118700753098, 1.42932474708659, -3.56915690046190, 1.51649127129379, -0.100045756315578, -1.36239190484780)
                # on universalrobots-ur6-85-5-a.zae (deg 8), 110 steps is 87.5%, 150 steps is 88.8%, performance difference is 200ms
                fcode = """static inline void %s(IkReal rawcoeffs[%d+1], IkReal rawroots[%d], int& numroots)
{
    using std::complex;
    IKFAST_ASSERT(rawcoeffs[0] != 0);
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
    complex<IkReal> coeffs[%d];
    const int maxsteps = 110;
    for(int i = 0; i < %d; ++i) {
        coeffs[i] = complex<IkReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IkReal> roots[%d];
    IkReal err[%d];
    roots[0] = complex<IkReal>(1,0);
    roots[1] = complex<IkReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < %d; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < %d; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IkReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < %d; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < %d; ++j) {
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

    numroots = 0;
    bool visited[%d] = {false};
    for(int i = 0; i < %d; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IkReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < %d; ++j) {
                if( abs(roots[i]-roots[j]) < 8*tolsqrt ) {
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
"""%(name,deg,deg,deg,deg,deg,deg,deg,deg,deg,deg,deg,deg,deg)
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
static inline bool %s(const IkReal* Breal)
{
    IkReal norm = 0.1;
    for(int i = 0; i < 7; ++i) {
        norm += IKabs(Breal[i]);
    }
    IkReal tol = 1e-5*norm; // have to increase the threshold since many computations are involved
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
