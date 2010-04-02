#!/usr/bin/env python
# Software License Agreement (Lesser GPL)
#
# Copyright (C) 2009 Rosen Diankov
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

import sys, copy, time
from optparse import OptionParser
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
        Transform6D=1
        Rotation3D=2
        Translation3D=3
        Direction3D=4
        Ray4D=5

from sympy import *

def xcombinations(items,n):
    if n == 0: yield[]
    else:
        for  i in xrange(len(items)):
            for cc in xcombinations(items[i+1:],n-1):
                yield [items[i]]+cc

def customcse(exprs,symbols=None):
    replacements,reduced_exprs = cse(exprs,symbols=symbols)
    newreplacements = []
    # look for any expressions of the order of (x**(1/a))**b, usually computer wants x^(b/a)
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
    AddPi = False
    def __init__(self, jointname, jointeval=None,jointevalcos=None,jointevalsin=None,AddPi=False):
        self.jointname = jointname
        self.jointeval = jointeval
        self.jointevalcos = jointevalcos
        self.jointevalsin = jointevalsin
        self.AddPi = AddPi
        assert(jointeval is not None or jointevalcos is not None or jointevalsin is not None)
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
    def __init__(self, solvejointvars, freejointvars, Tee, jointtree):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Tee = Tee
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateChain(self)
    def end(self, generator):
        return generator.endChain(self)

class SolverIKChainRotation3D(AutoReloader):
    solvejointvars = None
    freejointvars = None
    Ree = None
    jointtree = None
    def __init__(self, solvejointvars, freejointvars, Ree, jointtree):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Ree = Ree
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
    def __init__(self, solvejointvars, freejointvars, Pee, jointtree):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Pee = Pee
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateIKChainTranslation3D(self)
    def end(self, generator):
        return generator.endIKChainTranslation3D(self)

class SolverIKChainDirection3D(AutoReloader):
    solvejointvars = None
    freejointvars = None
    Dee = None
    jointtree = None
    def __init__(self, solvejointvars, freejointvars, Dee, jointtree):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Dee = Dee
        self.jointtree = jointtree
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
    def __init__(self, solvejointvars, freejointvars, Pee, Dee, jointtree):
        self.solvejointvars = solvejointvars
        self.freejointvars = freejointvars
        self.Pee = Pee
        self.Dee = Dee
        self.jointtree = jointtree
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

class SolverRay(AutoReloader):
    P = None
    D = None
    jointtree = None
    def __init__(self, P, D, jointtree):
        self.P = P
        self.D = D
        self.jointtree = jointtree
    def generate(self, generator):
        return generator.generateRay(self)
    def end(self, generator):
        return generator.endRay(self)

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

class CppGenerator(AutoReloader):
    """Generates C++ code from an AST"""
    dictequations = [] # dictionary of symbols already written
    symbolgen = cse_main.numbered_symbols('x')
    strprinter = printing.StrPrinter()
    freevars = None # list of free variables in the solution
    freevardependencies = None # list of variables depending on the free variables

    def generate(self, solvertree):
        code = """/// autogenerated analytical inverse kinematics code from ikfast program
/// \\author Rosen Diankov
///
/// To compile with gcc use: gcc -lstdc++ 
/// To compile without any main function use: gcc -lstdc++ -DIKFAST_NO_MAIN
#include <math.h>
#include <assert.h>
#include <vector>
#include <float.h>

#define IK2PI  6.28318530717959
#define IKPI  3.14159265358979
#define IKPI_2  1.57079632679490

#ifdef _MSC_VER
#ifndef isnan
#define isnan _isnan
#endif
#endif // _MSC_VER

// defined when creating a shared object/dll
#ifdef IKFAST_CLIBRARY
#ifdef _MSC_VER
#define IKFAST_API extern "C" __declspec(dllexport)
#else
#define IKFAST_API extern "C"
#endif
#else
#define IKFAST_API
#endif

typedef double IKReal;
class IKSolution
{
public:
    /// Gets a solution given its free parameters
    /// \\param pfree The free parameters required, range is in [-pi,pi]
    void GetSolution(IKReal* psolution, const IKReal* pfree) const {
        for(size_t i = 0; i < basesol.size(); ++i) {
            if( basesol[i].freeind < 0 )
                psolution[i] = basesol[i].foffset;
            else {
                assert(pfree != NULL);
                psolution[i] = pfree[basesol[i].freeind]*basesol[i].fmul + basesol[i].foffset;
                if( psolution[i] > IKPI )
                    psolution[i] -= IK2PI;
                else if( psolution[i] < -IKPI )
                    psolution[i] += IK2PI;
            }
        }
    }

    /// Gets the free parameters the solution requires to be set before a full solution can be returned
    /// \\return vector of indices indicating the free parameters
    const std::vector<int>& GetFree() const { return vfree; }

    struct VARIABLE
    {
        VARIABLE() : freeind(-1), fmul(0), foffset(0) {}
        VARIABLE(int freeind, IKReal fmul, IKReal foffset) : freeind(freeind), fmul(fmul), foffset(foffset) {}
        int freeind;
        IKReal fmul, foffset; ///< joint value is fmul*sol[freeind]+foffset
    };

    std::vector<VARIABLE> basesol;       ///< solution and their offsets if joints are mimiced
    std::vector<int> vfree;
};

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKasin(float f)
{
assert( f > -1.001f && f < 1.001f ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asinf(f);
}
inline double IKasin(double f)
{
assert( f > -1.001 && f < 1.001 ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

inline float IKacos(float f)
{
assert( f > -1.001f && f < 1.001f ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0.0f;
return acosf(f);
}
inline double IKacos(double f)
{
assert( f > -1.001 && f < 1.001 ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0.0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        assert(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) )
        return 0;
    return atan2f(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        assert(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) )
        return 0;
    return atan2(fy,fx);
}

"""
        code += solvertree.generate(self)
        code += solvertree.end(self)

        code += """
#ifndef IKFAST_NO_MAIN
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv)
{
    if( argc != 12+getNumFreeParameters()+1 ) {
        printf("\\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\\n\\n"
               "Returns the ik solutions given the transformation of the end effector specified by\\n"
               "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\\n"
               "There are %d free parameters that have to be specified.\\n\\n",getNumFreeParameters());
        return 1;
    }

    std::vector<IKSolution> vsolutions;
    std::vector<IKReal> vfree(getNumFreeParameters());
    IKReal eerot[9],eetrans[3];
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
    for(size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = atof(argv[13+i]);
    bool bSuccess = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\\n");
        return -1;
    }

    printf("Found %d ik solutions:\\n", (int)vsolutions.size());
    std::vector<IKReal> sol(getNumJoints());
    for(size_t i = 0; i < vsolutions.size(); ++i) {
        printf("sol%d (free=%d): ", (int)i, (int)vsolutions[i].GetFree().size());
        std::vector<IKReal> vsolfree(vsolutions[i].GetFree().size());
        vsolutions[i].GetSolution(&sol[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( size_t j = 0; j < sol.size(); ++j)
            printf("%f, ", (float)sol[j]);
        printf("\\n");
    }
    return 0;
}

#endif
"""
        return code

    def generateChain(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.dictequations = []
        self.symbolgen = cse_main.numbered_symbols('x')
        
        code = "IKFAST_API int getNumFreeParameters() { return %d; }\n"%len(node.freejointvars)
        if len(node.freejointvars) == 0:
            code += "IKFAST_API int* getFreeParameters() { return NULL; }\n"
        else:
            code += "IKFAST_API int* getFreeParameters() { static int freeparams[] = {"
            for i,freejointvar in enumerate(node.freejointvars):
                code += "%d"%(freejointvar[1])
                if i < len(node.freejointvars)-1:
                    code += ", "
            code += "}; return freeparams; }\n"
        code += "IKFAST_API int getNumJoints() { return %d; }\n\n"%(len(node.freejointvars)+len(node.solvejointvars))
        code += "IKFAST_API int getIKRealSize() { return sizeof(IKReal); }\n\n"
        code += 'IKFAST_API int getIKType() { return %d; }\n\n'%IkType.Transform6D
        code += "/// solves the inverse kinematics equations.\n"
        code += "/// \\param pfree is an array specifying the free joints of the chain.\n"
        code += "IKFAST_API bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<IKSolution>& vsolutions) {\n"
        code += "for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {\n"
        fcode = "vsolutions.resize(0); vsolutions.reserve(8);\n"
        fcode += 'IKReal '
        
        for var in node.solvejointvars:
            fcode += '%s, c%s, s%s,\n'%(var[0].name,var[0].name,var[0].name)
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d], c%s=cos(pfree[%d]), s%s=sin(pfree[%d]),\n'%(name,i,name,i,name,i)

        for i in range(3):
            for j in range(3):
                fcode += "_r%d%d, r%d%d = eerot[%d*3+%d],\n"%(i,j,i,j,i,j)
        fcode += "_px, _py, _pz, px = eetrans[0], py = eetrans[1], pz = eetrans[2];\n\n"
        
        rotsubs = [(Symbol("r%d%d"%(i,j)),Symbol("_r%d%d"%(i,j))) for i in range(3) for j in range(3)]
        rotsubs += [(Symbol("px"),Symbol("_px")),(Symbol("py"),Symbol("_py")),(Symbol("pz"),Symbol("_pz"))]

        psymbols = ["_px","_py","_pz"]
        for i in range(3):
            for j in range(3):
                fcode += self.writeEquations(lambda k: "_r%d%d"%(i,j),node.Tee[4*i+j])
            fcode += self.writeEquations(lambda k: psymbols[i],node.Tee[4*i+3])
        for i in range(3):
            for j in range(3):
                fcode += "r%d%d = _r%d%d; "%(i,j,i,j)
        fcode += "px = _px; py = _py; pz = _pz;\n"

        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn vsolutions.size()>0;\n}\n"
        return code
    def endChain(self, node):
        return ""

    def generateIKChainRotation3D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.dictequations = []
        self.symbolgen = cse_main.numbered_symbols('x')
        
        code = "IKFAST_API int getNumFreeParameters() { return %d; }\n"%len(node.freejointvars)
        if len(node.freejointvars) == 0:
            code += "IKFAST_API int* getFreeParameters() { return NULL; }\n"
        else:
            code += "IKFAST_API int* getFreeParameters() { static int freeparams[] = {"
            for i,freejointvar in enumerate(node.freejointvars):
                code += "%d"%(freejointvar[1])
                if i < len(node.freejointvars)-1:
                    code += ", "
            code += "}; return freeparams; }\n"
        code += "IKFAST_API int getNumJoints() { return %d; }\n\n"%(len(node.freejointvars)+len(node.solvejointvars))
        code += "IKFAST_API int getIKRealSize() { return sizeof(IKReal); }\n\n"
        code += 'IKFAST_API int getIKType() { return %d; }\n\n'%IkType.Rotation3D
        code += "/// solves the inverse kinematics equations.\n"
        code += "/// \\param pfree is an array specifying the free joints of the chain.\n"
        code += "IKFAST_API bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<IKSolution>& vsolutions) {\n"
        code += "for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {\n"
        fcode = "vsolutions.resize(0); vsolutions.reserve(8);\n"
        fcode += 'IKReal '
        
        for var in node.solvejointvars:
            fcode += '%s, c%s, s%s,\n'%(var[0].name,var[0].name,var[0].name)
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d], c%s=cos(pfree[%d]), s%s=sin(pfree[%d]),\n'%(name,i,name,i,name,i)
        for i in range(3):
            for j in range(3):
                fcode += "_r%d%d, r%d%d = eerot[%d*3+%d]"%(i,j,i,j,i,j)
                if i == 2 and j == 2:
                    fcode += ';\n\n'
                else:
                    fcode += ',\n'
        
        rotsubs = [(Symbol("r%d%d"%(i,j)),Symbol("_r%d%d"%(i,j))) for i in range(3) for j in range(3)]
        for i in range(3):
            for j in range(3):
                fcode += self.writeEquations(lambda k: "_r%d%d"%(i,j),node.Ree[i,j])
        for i in range(3):
            for j in range(3):
                fcode += "r%d%d = _r%d%d; "%(i,j,i,j)
        fcode += '\n'
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn vsolutions.size()>0;\n}\n"
        return code
    def endIKChainRotation3D(self, node):
        return ""

    def generateIKChainTranslation3D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.dictequations = []
        self.symbolgen = cse_main.numbered_symbols('x')
        
        code = "IKFAST_API int getNumFreeParameters() { return %d; }\n"%len(node.freejointvars)
        if len(node.freejointvars) == 0:
            code += "IKFAST_API int* getFreeParameters() { return NULL; }\n"
        else:
            code += "IKFAST_API int* getFreeParameters() { static int freeparams[] = {"
            for i,freejointvar in enumerate(node.freejointvars):
                code += "%d"%(freejointvar[1])
                if i < len(node.freejointvars)-1:
                    code += ", "
            code += "}; return freeparams; }\n"
        code += "IKFAST_API int getNumJoints() { return %d; }\n\n"%(len(node.freejointvars)+len(node.solvejointvars))
        code += "IKFAST_API int getIKRealSize() { return sizeof(IKReal); }\n\n"
        code += 'IKFAST_API int getIKType() { return %d; }\n\n'%IkType.Translation3D
        code += "/// solves the inverse kinematics equations.\n"
        code += "/// \\param pfree is an array specifying the free joints of the chain.\n"
        code += "IKFAST_API bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<IKSolution>& vsolutions) {\n"
        code += "for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {\n"
        fcode = "vsolutions.resize(0); vsolutions.reserve(8);\n"
        fcode += 'IKReal '
        
        for var in node.solvejointvars:
            fcode += '%s, c%s, s%s,\n'%(var[0].name,var[0].name,var[0].name)
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d], c%s=cos(pfree[%d]), s%s=sin(pfree[%d]),\n'%(name,i,name,i,name,i)
        fcode += "_px, _py, _pz, px = eetrans[0], py = eetrans[1], pz = eetrans[2];\n\n"
        rotsubs = [(Symbol("px"),Symbol("_px")),(Symbol("py"),Symbol("_py")),(Symbol("pz"),Symbol("_pz"))]
        psymbols = ["_px","_py","_pz"]
        for i in range(3):
            fcode += self.writeEquations(lambda k: psymbols[i],node.Pee[i])
        fcode += "px = _px; py = _py; pz = _pz;\n"
        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn vsolutions.size()>0;\n}\n"
        return code
    def endIKChainTranslation3D(self, node):
        return ""

    def generateIKChainDirection3D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.dictequations = []
        self.symbolgen = cse_main.numbered_symbols('x')
        
        code = "IKFAST_API int getNumFreeParameters() { return %d; }\n"%len(node.freejointvars)
        if len(node.freejointvars) == 0:
            code += "IKFAST_API int* getFreeParameters() { return NULL; }\n"
        else:
            code += "IKFAST_API int* getFreeParameters() { static int freeparams[] = {"
            for i,freejointvar in enumerate(node.freejointvars):
                code += "%d"%(freejointvar[1])
                if i < len(node.freejointvars)-1:
                    code += ", "
            code += "}; return freeparams; }\n"
        code += "IKFAST_API int getNumJoints() { return %d; }\n\n"%(len(node.freejointvars)+len(node.solvejointvars))
        code += "IKFAST_API int getIKRealSize() { return sizeof(IKReal); }\n\n"
        code += 'IKFAST_API int getIKType() { return %d; }\n\n'%IkType.Direction3D
        code += "/// solves the inverse kinematics equations.\n"
        code += "/// \\param pfree is an array specifying the free joints of the chain.\n"
        code += "IKFAST_API bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<IKSolution>& vsolutions) {\n"
        code += "for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {\n"
        fcode = "vsolutions.resize(0); vsolutions.reserve(8);\n"
        fcode += 'IKReal '
        
        for var in node.solvejointvars:
            fcode += '%s, c%s, s%s,\n'%(var[0].name,var[0].name,var[0].name)
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d], c%s=cos(pfree[%d]), s%s=sin(pfree[%d]),\n'%(name,i,name,i,name,i)

        for i in range(3):
            fcode += "_r0%d, r0%d = eerot[%d]"%(i,i,i)
            if i == 2:
                fcode += ';\n\n'
            else:
                fcode += ',\n'
        rotsubs = [(Symbol("r%d%d"%(0,i)),Symbol("_r%d%d"%(0,i))) for i in range(3)]

        for i in range(3):
            fcode += self.writeEquations(lambda k: "_r%d%d"%(0,i),node.Dee[i])
        for i in range(3):
            fcode += "r0%d = _r0%d; "%(i,i)

        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn vsolutions.size()>0;\n}\n"
        return code
    def endIKChainDirection3D(self, node):
        return ''

    def generateIKChainRay4D(self, node):
        self.freevars = []
        self.freevardependencies = []
        self.dictequations = []
        self.symbolgen = cse_main.numbered_symbols('x')
        
        code = "IKFAST_API int getNumFreeParameters() { return %d; }\n"%len(node.freejointvars)
        if len(node.freejointvars) == 0:
            code += "IKFAST_API int* getFreeParameters() { return NULL; }\n"
        else:
            code += "IKFAST_API int* getFreeParameters() { static int freeparams[] = {"
            for i,freejointvar in enumerate(node.freejointvars):
                code += "%d"%(freejointvar[1])
                if i < len(node.freejointvars)-1:
                    code += ", "
            code += "}; return freeparams; }\n"
        code += "IKFAST_API int getNumJoints() { return %d; }\n\n"%(len(node.freejointvars)+len(node.solvejointvars))
        code += "IKFAST_API int getIKRealSize() { return sizeof(IKReal); }\n\n"
        code += 'IKFAST_API int getIKType() { return %d; }\n\n'%IkType.Ray4D
        code += "/// solves the inverse kinematics equations.\n"
        code += "/// \\param pfree is an array specifying the free joints of the chain.\n"
        code += "IKFAST_API bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<IKSolution>& vsolutions) {\n"
        code += "for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {\n"
        fcode = "vsolutions.resize(0); vsolutions.reserve(8);\n"
        fcode += 'IKReal '
        
        for var in node.solvejointvars:
            fcode += '%s, c%s, s%s,\n'%(var[0].name,var[0].name,var[0].name)
        for i in range(len(node.freejointvars)):
            name = node.freejointvars[i][0].name
            fcode += '%s=pfree[%d], c%s=cos(pfree[%d]), s%s=sin(pfree[%d]),\n'%(name,i,name,i,name,i)
        for i in range(3):
            fcode += "_r0%d, r0%d = eerot[%d],\n"%(i,i,i)
        fcode += "_px, _py, _pz, px = eetrans[0], py = eetrans[1], pz = eetrans[2];\n\n"

        rotsubs = [(Symbol("r%d%d"%(0,i)),Symbol("_r%d%d"%(0,i))) for i in range(3)]
        rotsubs += [(Symbol("px"),Symbol("_px")),(Symbol("py"),Symbol("_py")),(Symbol("pz"),Symbol("_pz"))]

        psymbols = ["_px","_py","_pz"]
        for i in range(3):
            fcode += self.writeEquations(lambda k: "_r%d%d"%(0,i),node.Dee[i])
            fcode += self.writeEquations(lambda k: psymbols[i],node.Pee[i])
        for i in range(3):
            fcode += "r0%d = _r0%d; "%(i,i)
        fcode += "px = _px; py = _py; pz = _pz;\n"

        fcode += self.generateTree(node.jointtree)
        code += self.indentCode(fcode,4) + "}\nreturn vsolutions.size()>0;\n}\n"
        return code
    def endIKChainRay4D(self, node):
        return ''

    def generateSolution(self, node):
        code = ''
        numsolutions = 0
        eqcode = ''
        name = node.jointname
        node.HasFreeVar = False

        if node.jointeval is not None:
            numsolutions = len(node.jointeval)
            equations = []
            names = []
            for i,expr in enumerate(node.jointeval):
                
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
                            code += 'IKReal ' + self.writeEquations(lambda i: '%smul'%name, m[a])
                            code += self.writeEquations(lambda i: name, m[b])
                            node.HasFreeVar = True
                            return code
                        else:
                            print 'failed to extract free variable %s for %s from'%(freevar,node.jointname), expr
#                             m = dict()
#                             m[a] = Real(-1,30)
#                             m[b] = Real(0,30)

                equations.append(expr)
                names.append('%sarray[%d]'%(name,i))
                equations.append(sin(Symbol('%sarray[%d]'%(name,i))))
                names.append('s%sarray[%d]'%(name,i))
                equations.append(cos(Symbol('%sarray[%d]'%(name,i))))
                names.append('c%sarray[%d]'%(name,i))
            eqcode += self.writeEquations(lambda i: names[i], equations)

            if node.AddPi:
                for i in range(numsolutions):
                    eqcode += '%sarray[%d] = %sarray[%d] > 0 ? %sarray[%d]-IKPI : %sarray[%d]+IKPI;\n'%(name,numsolutions+i,name,i,name,i,name,i)
                    eqcode += 's%sarray[%d] = -s%sarray[%d];\n'%(name,numsolutions+i,name,i)
                    eqcode += 'c%sarray[%d] = -c%sarray[%d];\n'%(name,numsolutions+i,name,i)
                numsolutions *= 2
            
            for i in range(numsolutions):
                eqcode += 'if( %sarray[%d] > IKPI )\n    %sarray[%d]-=IK2PI;\nelse if( %sarray[%d] < -IKPI )\n    %sarray[%d]+=IK2PI;\n'%(name,i,name,i,name,i,name,i)
                eqcode += '%svalid[%d] = true;\n'%(name,i)
        elif node.jointevalcos is not None:
            numsolutions = 2*len(node.jointevalcos)
            eqcode += self.writeEquations(lambda i: 'c%sarray[%d]'%(name,2*i),node.jointevalcos)
            for i in range(len(node.jointevalcos)):
                eqcode += 'if( c%sarray[%d] >= -1.0001 && c%sarray[%d] <= 1.0001 ) {\n'%(name,2*i,name,2*i)
                eqcode += '    %svalid[%d] = %svalid[%d] = true;\n'%(name,2*i,name,2*i+1)
                eqcode += '    %sarray[%d] = IKacos(c%sarray[%d]);\n'%(name,2*i,name,2*i)
                eqcode += '    s%sarray[%d] = IKsin(%sarray[%d]);\n'%(name,2*i,name,2*i)
                # second solution
                eqcode += '    c%sarray[%d] = c%sarray[%d];\n'%(name,2*i+1,name,2*i)
                eqcode += '    %sarray[%d] = -%sarray[%d];\n'%(name,2*i+1,name,2*i)
                eqcode += '    s%sarray[%d] = -s%sarray[%d];\n'%(name,2*i+1,name,2*i)
                eqcode += '}\n'
                eqcode += 'else if( isnan(c%sarray[%d]) ) {\n'%(name,2*i)
                eqcode += '    // probably any value will work\n'
                eqcode += '    %svalid[%d] = true;\n'%(name,2*i)
                eqcode += '    c%sarray[%d] = 1; s%sarray[%d] = 0; %sarray[%d] = 0;\n'%(name,2*i,name,2*i,name,2*i)
                eqcode += '}\n'
        elif node.jointevalsin is not None:
            numsolutions = 2*len(node.jointevalsin)
            eqcode += self.writeEquations(lambda i: 's%sarray[%d]'%(name,2*i),node.jointevalsin)
            for i in range(len(node.jointevalsin)):
                eqcode += 'if( s%sarray[%d] >= -1.0001 && s%sarray[%d] <= 1.0001 ) {\n'%(name,2*i,name,2*i)
                eqcode += '    %svalid[%d] = %svalid[%d] = true;\n'%(name,2*i,name,2*i+1)
                eqcode += '    %sarray[%d] = IKasin(s%sarray[%d]);\n'%(name,2*i,name,2*i)
                eqcode += '    c%sarray[%d] = IKcos(%sarray[%d]);\n'%(name,2*i,name,2*i)
                # second solution
                eqcode += '    s%sarray[%d] = s%sarray[%d];\n'%(name,2*i+1,name,2*i)
                eqcode += '    %sarray[%d] = %sarray[%d] > 0 ? (IKPI-%sarray[%d]) : (-IKPI-%sarray[%d]);\n'%(name,2*i+1,name,2*i,name,2*i,name,2*i)
                eqcode += '    c%sarray[%d] = -c%sarray[%d];\n'%(name,2*i+1,name,2*i)
                eqcode += '}\n'
                eqcode += 'else if( isnan(s%sarray[%d]) ) {\n'%(name,2*i)
                eqcode += '    // probably any value will work\n'
                eqcode += '    %svalid[%d] = true;\n'%(name,2*i)
                eqcode += '    c%sarray[%d] = 1; s%sarray[%d] = 0; %sarray[%d] = 0;\n'%(name,2*i,name,2*i,name,2*i)
                eqcode += '}\n'

        code += '{\nIKReal %sarray[%d], c%sarray[%d], s%sarray[%d];\n'%(name,numsolutions,name,numsolutions,name,numsolutions)

        code += 'bool %svalid[%d]={false};\n'%(name,numsolutions)
        code += eqcode
        for i,j in xcombinations(range(numsolutions),2):
            code += 'if( %svalid[%d] && %svalid[%d] && IKabs(c%sarray[%d]-c%sarray[%d]) < 0.0001 && IKabs(s%sarray[%d]-s%sarray[%d]) < 0.0001 )\n    %svalid[%d]=false;\n'%(name,i,name,j,name,i,name,j,name,i,name,j,name,j)
        if numsolutions > 1:
            code += 'for(int i%s = 0; i%s < %d; ++i%s) {\n'%(name,name,numsolutions,name)
        else:
            code += '{ int i%s = 0;\n'%(name)
        code += 'if( !%svalid[i%s] )\n    continue;\n'%(name,name)

        if numsolutions > 1:
            code += '%s = %sarray[i%s]; c%s = c%sarray[i%s]; s%s = s%sarray[i%s];\n\n'%(name,name,name,name,name,name,name,name,name)
        else:
            code += '%s = %sarray[0]; c%s = c%sarray[0]; s%s = s%sarray[0];\n\n'%(name,name,name,name,name,name)
        return code

    def endSolution(self, node):
        if node.HasFreeVar:
            self.freevardependencies.pop()
            return ''
        return '}\n}\n'

    def generateBranch(self, node):
        origequations = copy.copy(self.dictequations)
        name = node.jointname
        code = '{\nIKReal %seval;\n'%name
        code += self.writeEquations(lambda x: '%seval'%name,[node.jointeval])
        for branch in node.jointbranches:
            branchcode = ''
            self.dictequations = copy.copy(origequations)
            for n in branch[1]:
                branchcode += n.generate(self)
            for n in reversed(branch[1]):
                branchcode += n.end(self)
            branchcode = self.indentCode(branchcode,4)
            if branch[0] is None:
                code += '{\n' + branchcode + '}\n'
            else:
                code += 'if( %seval >= %f && %seval <= %f ) {\n'%(name,branch[0]-0.00001,name,branch[0]+0.00001)
                code += branchcode + '}\nelse '
        code += '}\n'
        self.dictequations = origequations
        return code
    def endBranch(self, node):
        return ''
    def generateBranchConds(self, node):
        origequations = copy.copy(self.dictequations)
        code = '{\n'
        if any([branch[0] for branch in node.jointbranches]):
            code += 'IKReal evalcond;\n'
        for branch in node.jointbranches:
            if branch[0] is None:
                branchcode = 'if( 1 ) {\n'
            else:
                branchcode = self.writeEquations(lambda x: 'evalcond',branch[0])
                branchcode += 'if( IKabs(evalcond) < 0.00001 ) {\n'
            self.dictequations = copy.copy(origequations)
            for n in branch[1]:
                branchcode += n.generate(self)
            for n in reversed(branch[1]):
                branchcode += n.end(self)
            code += self.indentCode(branchcode,4)+'} else {\n'
        code += '}\n'*(len(node.jointbranches)+1)
        self.dictequations = origequations
        return code
    def endBranchConds(self, node):
        return ''
    def generateCheckZeros(self, node):
        origequations = copy.copy(self.dictequations)
        name = node.jointname
        code = 'IKReal %seval[%d];\n'%(name,len(node.jointcheckeqs))
        code += self.writeEquations(lambda i: '%seval[%d]'%(name,i),node.jointcheckeqs)
        if len(node.jointcheckeqs) > 0:
            code += 'if( '
            for i in range(len(node.jointcheckeqs)):
                if i != 0:
                    if node.anycondition:
                        code += ' || '
                    else:
                        code += ' && '
                code += 'IKabs(%seval[%d]) < %f '%(name,i,node.thresh)
            code += ' ) {\n'
            self.dictequations = copy.copy(origequations)
            code += self.indentCode(self.generateTree(node.zerobranch),4)
            code += '\n} else\n'
        code += '{\n'
        self.dictequations = copy.copy(origequations)
        code += self.indentCode(self.generateTree(node.nonzerobranch),4)
        code += '\n}\n'
        self.dictequations = origequations
        return '{\n' + self.indentCode(code,4) + '}\n'
    def endCheckZeros(self, node):
        return ''
    def generateFreeParameter(self, node):
        #print 'free variable ',node.jointname,': ',self.freevars
        self.freevars.append(node.jointname)
        self.freevardependencies.append((node.jointname,node.jointname))
        code = 'IKReal %smul = 1;\n%s=0;\n'%(node.jointname,node.jointname)
        return code+self.generateTree(node.jointtree)
    def endFreeParameter(self, node):
        self.freevars.pop()
        self.freevardependencies.pop()
        return ''
    def generateSetJoint(self, node):
        code = '{\n%s = %f; s%s = %f; c%s = %f;\n'%(node.jointname,node.jointvalue,node.jointname,sin(node.jointvalue),node.jointname,cos(node.jointvalue))
        return code
    def endSetJoint(self, node):
        return '}\n'
    def generateBreak(self,node):
        return 'continue;\n'
    def endBreak(self,node):
        return ''
    def generateRotation(self, node):
        code = ''
        listequations = []
        names = []
        for i in range(3):
            for j in range(3):
                listequations.append(node.T[i,j])
                names.append(Symbol('_r%d%d'%(i,j)))
        code += self.writeEquations(lambda i: names[i],listequations)
        code += self.generateTree(node.jointtree)
        return code
    def endRotation(self, node):
        return ''
    def generateDirection(self, node):
        code = ''
        listequations = []
        names = []
        for i in range(3):
            listequations.append(node.D[i])
            names.append(Symbol('_r%d%d'%(0,i)))
        code += self.writeEquations(lambda i: names[i],listequations)
        code += self.generateTree(node.jointtree)
        return code
    def endDirection(self, node):
        return ''
    def generateRay(self, node):
        code = ''
        listequations = []
        names = []
        for i in range(3):
            listequations.append(node.D[i])
            names.append(Symbol('_r%d%d'%(0,i)))
        for i in range(3):
            listequations.append(node.P[i])
        names.append(Symbol('_px'))
        names.append(Symbol('_py'))
        names.append(Symbol('_pz'))
        code += self.writeEquations(lambda i: names[i],listequations)
        code += self.generateTree(node.jointtree)
        return code
    def endRay(self, node):
        return ''
    def generateStoreSolution(self, node):
        code = 'vsolutions.push_back(IKSolution()); IKSolution& solution = vsolutions.back();\n'
        code += 'solution.basesol.resize(%d);\n'%len(node.alljointvars)
        for i,var in enumerate(node.alljointvars):
            code += 'solution.basesol[%d].foffset = %s;\n'%(i,var)
            
            vardeps = [vardep for vardep in self.freevardependencies if vardep[1]==var.name]
            if len(vardeps) > 0:
                freevarname = vardeps[0][0]
                ifreevar = [j for j in range(len(self.freevars)) if freevarname==self.freevars[j]]
                code += 'solution.basesol[%d].fmul = %smul;\n'%(i,var.name)
                code += 'solution.basesol[%d].freeind = %d;\n'%(i,ifreevar[0])
        code += 'solution.vfree.resize(%d);\n'%len(self.freevars)
        for i,varname in enumerate(self.freevars):
            ind = [j for j in range(len(node.alljointvars)) if varname==node.alljointvars[j].name]
            code += 'solution.vfree[%d] = %d;\n'%(i,ind[0])
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
    def writeEquations(self, varnamefn, exprs):
        code = ''
        [replacements,reduced_exprs] = customcse(exprs,symbols=self.symbolgen)
        for rep in replacements:                
            eqns = filter(lambda x: rep[1]-x[1]==0, self.dictequations)
            if len(eqns) > 0:
                self.dictequations.append((rep[0],eqns[0][0]))
                code += 'IKReal %s=%s;\n'%(rep[0],eqns[0][0])
            else:
                self.dictequations.append(rep)
                code2,sepcode2 = self.writeExprCode(rep[1])
                code += sepcode2+'IKReal %s=%s;\n'%(rep[0],code2)

        for i,rexpr in enumerate(reduced_exprs):
            code2,sepcode2 = self.writeExprCode(rexpr)
            code += sepcode2+'%s=%s;\n'%(varnamefn(i), code2)
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
            elif expr.func == acos:
                code += 'IKacos('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2
                sepcode += 'if( (%s) < -1.0001 || (%s) > 1.0001 )\n    continue;\n'%(code2,code2)
            elif expr.func == asin:
                code += 'IKasin('
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2
                sepcode += 'if( (%s) < -1.0001 || (%s) > 1.0001 )\n    continue;\n'%(code2,code2)
            elif expr.func == atan2:
                code += 'IKatan2('
                # check for divides by 0 in arguments, this could give two possible solutions?!?
                # if common arguments is nan! solution is lost!
                code2,sepcode = self.writeExprCode(expr.args[0])
                code += code2+', '
                code3,sepcode2 = self.writeExprCode(expr.args[1])
                code += code3
                sepcode += sepcode2
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
            else:
                code += expr.func.__name__ + '(';
                for arg in expr.args:
                    code2,sepcode2 = self.writeExprCode(arg)
                    code += code2
                    sepcode += sepcode2
                    if not arg == expr.args[-1]:
                        code += ','
            return code + ')',sepcode
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
            if expr.exp.is_real:
                if expr.exp.is_integer and expr.exp.evalf() > 0:
                    code += '('+exprbase+')'
                    for i in range(1,expr.exp.evalf()):
                        code += '*('+exprbase+')'
                    return code,sepcode
                elif expr.exp-0.5 == 0:
                    sepcode += 'if( (%s) < (IKReal)-0.00001 )\n    continue;\n'%exprbase
                    return 'IKsqrt('+exprbase+')',sepcode
                elif expr.exp < 0:
                    # check if exprbase is 0
                    if expr.exp+1 == 0:
                        return '((IKabs('+exprbase+') != 0)?((IKReal)1/('+exprbase+')):(IKReal)1.0e30)',sepcode
                    return '((IKabs('+exprbase+') != 0)?(pow(' + exprbase + ',' + str(expr.exp.evalf()) + ')):(IKReal)1.0e30)',sepcode
                    
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
        lcode = list(code)
        locations = [i for i in range(len(lcode)) if lcode[i]=='\n']
        locations.reverse()
        insertcode = [' ' for i in range(numspaces)]
        for loc in locations:
            lcode[loc+1:0] = insertcode
        lcode[:0] = insertcode
        return ''.join(lcode)

class IKFastSolver(AutoReloader):
    """
    Parses the kinematics from an openrave fk file and generates C++ code for analytical inverse kinematics.
    author: Rosen Diankov

    Generated C++ code structure:

    typedef double IKReal;
    class IKSolution
    {
    public:
        void GetSolution(IKReal* psolution, const IKReal* pfree) const;
    };

    bool ik(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<SOLUTION>& vsolutions);
    bool fk(const IKReal* joints, IKReal* eetrans, IKReal* eerot);
    
    """
    
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
        self.rigidlyconnectedlinks = []
        if kinbody is not None:
            # this actually requires openravepy to run, but it isn't a good idea to make ikfast dependent on openravepy
            with kinbody.GetEnv():
                for bodyjoint in kinbody.GetJoints():
                    joint = IKFastSolver.Joint()
                    joint.type = bodyjoint.GetType().name.lower()
                    joint.jointindex = bodyjoint.GetJointIndex()
                    joint.jcoeff = [round(float(x),5) for x in bodyjoint.GetMimicCoeffs()]
                    joint.isfreejoint = False
                    joint.isdummy = False
                    joint.Tright = self.normalizeRotation(Matrix(4,4,[Real(round(float(x),5),30) for x in bodyjoint.GetInternalHierarchyRightTransform().flat]))
                    joint.Tleft = self.normalizeRotation(Matrix(4,4,[Real(round(float(x),5),30) for x in bodyjoint.GetInternalHierarchyLeftTransform().flat]))
                    joint.axis = Matrix(3,1,[Real(round(float(x),4),30) for x in bodyjoint.GetInternalHierarchyAxis(0)])
                    joint.linkcur = bodyjoint.GetSecondAttached().GetIndex()
                    joint.linkbase = bodyjoint.GetFirstAttached().GetIndex()
                    if kinbody.DoesAffect(bodyjoint.GetJointIndex(),bodyjoint.GetFirstAttached().GetIndex()):
                        # bodies[0] is the child
                        joint.linkcur,joint.linkbase = joint.linkbase,joint.linkcur
                    alljoints.append(joint)
                for passivejoint in kinbody.GetPassiveJoints():
                    lower,upper = passivejoint.GetLimits()
                    if all(lower==upper):
                        self.rigidlyconnectedlinks.append((passivejoint.GetFirstAttached().GetIndex(),passivejoint.GetSecondAttached().GetIndex()))
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
        rigidlyconnectedlinks = self.rigidlyconnectedlinks[:]
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
            # examine rigidly connected links
            for i,connectedlinks in enumerate(rigidlyconnectedlinks):
                if connectedlinks[0] == link[0]:
                    linkqueue.append([connectedlinks[1],path])
                    rigidlyconnectedlinks.pop(i)
                    break
                elif connectedlinks[0] == link[0]:
                    linkqueue.append([connectedlinks[0],path])
                    rigidlyconnectedlinks.pop(i)
                    break
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

    # The first and last matrices returned are always numerical
    def forwardKinematicsChain(self, chain):
        Links = []
        Tright = eye(4)
        jointvars = []
        isolvejointvars = []
        ifreejointvars = []
        jointinds = []
        jointindexmap = dict()
        for i,joint in enumerate(chain):
            if not joint.isdummy:
                if not joint.jointindex in jointindexmap:
                    jointindexmap[joint.jointindex] = len(jointvars)
                    var = Symbol("j%d"%len(jointvars))
                else:
                    var = Symbol("j%d"%jointindexmap[joint.jointindex])
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
        
    def generateIkSolver(self, baselink, eelink, solvejoints, freeparams, usedummyjoints,solvefn=None):
        if solvefn is None:
            solvefn = IKFastSolver.solveFullIK_6D
        alljoints = self.getJointsInChain(baselink, eelink)
        
        # mark the free joints and form the chain
        chain = []
        for joint in alljoints:
            issolvejoint = any([i == joint.jointindex for i in solvejoints])
            joint.isdummy = usedummyjoints and not issolvejoint and not any([i == joint.jointindex for i in freeparams])
            joint.isfreejoint = not issolvejoint and not joint.isdummy
            chain.append(joint)
        
        return self.generateIkSolverChain(chain,solvefn)
        
    def generateIkSolverChain(self, chain, solvefn):
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
        return CppGenerator().generate(chaintree)

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
        LinksInv = [self.affineInverse(link) for link in Links]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]
        
        if not len(solvejointvars) == 2:
            raise ValueError('solve joints needs to be 2')

        # rotate all links so that basedir becomes the z-axis
        Trightnormaliation = self.rotateDirection(sourcedir=Matrix(3,1,[Real(round(float(x),4),30) for x in basedir]), targetdir = Matrix(3,1,[S.Zero,S.Zero,S.One]))
        Links[-1] *= self.affineInverse(Trightnormaliation)

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
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None else [None]
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
                rotsubs = [(Symbol('r%d%d'%(0,i)),Symbol('_r%d%d'%(0,i))) for i in range(3)]
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
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None else [None]
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
                rotsubs = [(Symbol('r%d%d'%(i,j)),Symbol('_r%d%d'%(i,j))) for i in range(3) for j in range(3)]
                rotvars = [var for var in jointvars if any([var==svar for svar in solvejointvars])]
                R = Matrix(3,3, map(lambda x: x.subs(solvedvarsubs), LinksAccumRightAll[0][0:3,0:3]))
                rottree = self.solveIKRotation(R=R,Ree = Ree.subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
                solverbranches.append((freevarcond,[SolverRotation(LinksAccumLeftInvAll[0].subs(solvedvarsubs)*Tee, rottree)]))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for kinematics'
            return None
        return SolverIKChainRotation3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Tfirstleft.inv()[0:3,0:3] * Ree * Tfirstright.inv()[0:3,0:3], [SolverBranchConds(solverbranches)])

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
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None else [None]
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
        return SolverIKChainTranslation3D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], TfirstleftInv[0:3,0:3] * Pee + TfirstleftInv[0:3,3] , [SolverBranchConds(solverbranches)])

    def solveFullIK_Ray4D(self,chain,Tee,basedir,basepos):
        """basedir,basepos needs to be filled with a direction and position of the ray to control"""
        Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(chain)
        Tfirstleft = Links.pop(0)
        TfirstleftInv = Tfirstleft.inv()
        LinksInv = [self.affineInverse(link) for link in Links]
        solvejointvars = [jointvars[i] for i in isolvejointvars]
        freejointvars = [jointvars[i] for i in ifreejointvars]
        
        if not len(solvejointvars) == 4:
            raise ValueError('solve joints needs to be 4')

        # rotate all links so that basedir becomes the z-axis
        Trightnormaliation = self.rotateDirection(sourcedir=Matrix(3,1,[Real(round(float(x),4),30) for x in basedir]), targetdir = Matrix(3,1,[S.Zero,S.Zero,S.One]))
        Links[-1] *= self.affineInverse(Trightnormaliation)

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
        
        solverbranches = []
        # depending on the free variable values, sometimes the rotation matrix can have zeros where it does not expect zeros. Therefore, test all boundaries of all free variables
        for freevar in freejointvars+[None]:
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None else [None]
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
                rottree = []
                endbranchtree = [SolverSequence([rottree])]
                curtransvars = solvejointvars[0:2]
                transtree = self.solveIKTranslationAll(Positions,Positionsee,curtransvars,
                                                       otherunsolvedvars = [],
                                                       othersolvedvars = freejointvars,
                                                       endbranchtree=endbranchtree,
                                                       solsubs = solsubs)

                storesolutiontree = [SolverStoreSolution (jointvars)]
                solvedvarsubs = valuesubs+self.freevarsubs
                for tvar in solvejointvars[0:2]:
                    solvedvarsubs += [(cos(tvar),self.Variable(tvar).cvar),(sin(tvar),self.Variable(tvar).svar)]
                rotsubs = [(Symbol('r%d%d'%(0,i)),Symbol('_r%d%d'%(0,i))) for i in range(3)]
                rotvars = [var for var in jointvars if any([var==svar for svar in solvejointvars[2:4]])]
                D = Matrix(3,1, map(lambda x: x.subs(self.freevarsubs), LinksAccumRightAll[0][0:3,2]))
                rottree += self.solveIKRotation(R=D,Ree = Dee.subs(rotsubs),rawvars = rotvars,endbranchtree=storesolutiontree,solvedvarsubs=solvedvarsubs)
                Tlinksub = LinksAccumLeftInvAll[0].subs(solvedvarsubs)
                solverbranches.append((freevarcond,[SolverRay(Tlinksub[0:3,0:3]*Pee+Tlinksub[0:3,3],Tlinksub[0:3,0:3]*Dee, rottree)]))

        if len(solverbranches) == 0 or not solverbranches[-1][0] is None:
            print 'failed to solve for ray4d kinematics'
            return None
        return SolverIKChainRay4D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], TfirstleftInv[0:3,0:3] * Pee + TfirstleftInv[0:3,3], TfirstleftInv[0:3,0:3] * Dee, [SolverBranchConds(solverbranches)])
        
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
            freevalues = [0,pi/2,pi,-pi/2] if freevar is not None else [None]
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
                transtree = self.solveIKTranslationAll(Positions,Positionsee,curtransvars,
                                                       otherunsolvedvars = [] if solveRotationFirst else rotvars,
                                                       othersolvedvars = rotvars+freejointvars if solveRotationFirst else freejointvars,
                                                       endbranchtree=endbranchtree,
                                                       solsubs = solsubs)
                
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
                
                rotsubs = [(Symbol('r%d%d'%(i,j)),Symbol('_r%d%d'%(i,j))) for i in range(3) for j in range(3)]
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
        return SolverIKChainTransform6D([(jointvars[ijoint],ijoint) for ijoint in isolvejointvars], [(jointvars[ijoint],ijoint) for ijoint in ifreejointvars], Tfirstleft.inv() * Tee * Tfirstright.inv(), [SolverBranchConds(solverbranches)])

    def isExpressionUnique(self, exprs, expr):
        for exprtest in exprs:
            e = expr-exprtest
            if e.is_number and abs(e) < 1e-10:
                return False
        return True

    def solveIKTranslationAll(self,Positions,Positionsee,transvars,otherunsolvedvars,othersolvedvars,endbranchtree=None,solsubs=[]):
        allsolvedvars = othersolvedvars[:]
        transtree = []
        solsubs = solsubs[:]
        while len(transvars)>0:
            unsolvedvariables = transvars + otherunsolvedvars
            solvedvars = self.solveIKTranslation(Positions,Positionsee,rawvars = transvars)
            if len(solvedvars) == 0:
                solvedvars = self.solveIKTranslationLength(Positions, Positionsee, rawvars = transvars)
                if len(solvedvars) == 0:
                    raise ValueError('Could not solve variable from length of translation')

            # for all solutions, check if there is a divide by zero
            checkforzeros = []
            scores = []
            for solvedvar in solvedvars:
                score = 1000*len(solvedvar[2])+reduce(lambda x,y:x+y,solvedvar[2])
                checkforzero = []
                if solvedvar[1].jointeval is not None:
                    subexprs,reduced_exprs = customcse(solvedvar[1].jointeval)
                elif solvedvar[1].jointevalsin is not None:
                    subexprs,reduced_exprs = customcse(solvedvar[1].jointevalsin)
                elif solvedvar[1].jointevalcos is not None:
                    subexprs,reduced_exprs = customcse(solvedvar[1].jointevalcos)
                else:
                    continue
                
                def checkpow(expr):
                    score = 0
                    if expr.is_Pow and expr.exp.is_real and expr.exp < 0:
                        exprbase = self.subsExpressions(expr.base,subexprs)
                        # check if exprbase contains any variables that have already been solved
                        containsjointvar = exprbase.has_any_symbols(*allsolvedvars)
                        cancheckexpr = not exprbase.has_any_symbols(*unsolvedvariables)
                        score += 10000
                        if not cancheckexpr:
                            score += 100000
                        else:
                            checkforzero.append(exprbase)
                    return score
                
                for sexpr in [subexpr[1] for subexpr in subexprs]+reduced_exprs:
                    if sexpr.is_Add:
                        for arg in sexpr.args:
                            if arg.is_Mul:
                                for arg2 in arg.args:
                                    score += checkpow(arg2)
                            else:
                                score += checkpow(arg)
                    elif sexpr.is_Mul:
                        for arg in sexpr.args:
                            score += checkpow(arg)
                    else:
                        score += checkpow(sexpr)
                scores.append(score)
                checkforzeros.append(checkforzero)

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
                                        newbranch = self.solveIKTranslationAll(NewPositions,NewPositionsee,transvars[:],otherunsolvedvars[:],allsolvedvars,endbranchtree,solsubs)
                                    except ValueError:
                                        newbranch = []
                                    if len(newbranch) > 0:
                                        zerobranches.append((svar-s,newbranch))
                                    else:
                                        covered = False
                                else:
                                    covered = False
                        if not covered and len(transvars) > 1:
                            # test several values for all variables
                            valuebranches = []
                            for targetvar in transvars:
                            # force the value of the variable. The variable should in fact be *free*, but things get complicated with checking conditions, therefore, pick a couple of values and manually set them
                                for value in [0]:
                                    newtransvars = transvars[:]
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
                                        newbranch = self.solveIKTranslationAll(NewPositions2,NewPositionsee2,newtransvars,otherunsolvedvars[:],allsolvedvars+[targetvar],endbranchtree,solsubs)
                                    except:
                                        newbranch = []
                                    if len(newbranch) > 0:
                                        valuebranches.append(SolverCheckZeros(targetvar.name,checkconsistenteqs,[SolverSetJoint(targetvar,value)]+newbranch,[SolverBreak()],thresh=0.0001,anycondition=False))
                            if len(valuebranches) > 0:
                                zerobranches.append((checkzero,valuebranches))

            allsolvedvars.append(var)
            transvars.remove(var)
            solsubs += [(cos(var),self.Variable(var).cvar),(sin(var),self.Variable(var).svar)]

            if len(transvars) > 0 and len(checkforzeros[bestindex]) > 0:
                childbranch = self.solveIKTranslationAll(Positions,Positionsee,transvars,otherunsolvedvars,allsolvedvars,endbranchtree,solsubs)
                if len(zerobranches) > 0:
                    zerobranch = [SolverBranchConds(zerobranches+[(None,[solvedvars[bestindex][1].subs(solsubs)]+childbranch)])]
                else:
                    zerobranch = [SolverBreak()] # in this case, cannot handle zeros, so don't output solution

                #TODO: cannot handle free parameters yet since it requires subroutines for evaluating the nonlinearities.
                transtree.append(SolverCheckZeros(var.name,checkforzeros[bestindex],zerobranch,[solvedvars[bestindex][1].subs(solsubs)]+childbranch))
                return transtree
            else:
                if len(zerobranches) > 0:
                    childbranch = self.solveIKTranslationAll(Positions,Positionsee,transvars,otherunsolvedvars,allsolvedvars,endbranchtree,solsubs)
                    transtree.append(SolverBranchConds(zerobranches+[(None,[solvedvars[bestindex][1].subs(solsubs)]+childbranch)]))
                    return transtree
                else:
                    transtree.append(solvedvars[bestindex][1].subs(solsubs))

        if len(transvars) == 0 and endbranchtree is not None:
            transtree += endbranchtree
        return transtree

    # solve for just the translation component
    def solveIKTranslationLength(self, Positions, Positionsee, rawvars):
        vars = map(lambda rawvar: self.Variable(rawvar), rawvars)
        
        # try to get an equation from the lengths
        Lengths = map(lambda x: self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((x[0]**2+x[1]**2+x[2]**2).expand())).expand())), Positions)
        Lengthsee = map(lambda x: self.chop(self.customtrigsimp(self.customtrigsimp(self.customtrigsimp((x[0]**2+x[1]**2+x[2]**2).expand())).expand())), Positionsee)
        LengthEq = map(lambda i: Lengths[i]-Lengthsee[i], range(len(Lengths)))
        solvedvars = []
        
        for eq in LengthEq:
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
                            solvedvars.append((var.var,SolverSolution(var.var.name,jointeval=jointsolutions), [self.codeComplexity(s) for s in jointsolutions]))
                            continue
                    
                    if numsvar > 0:
                        try:
                            # substitute cos
                            if self.countVariables(eq.subs(cos(var.var),var.cvar),var.cvar) <= 1: # anything more than 1 implies quartic equation
                                eqnew = eq.subs(self.freevarsubs+[(cos(var.var),sqrt(Real(1,30)-var.svar**2)),(sin(var.var),var.svar)])
                                eqnew,symbols = self.factorLinearTerms(eqnew,[var.svar])
                                solutions = self.customtsolve(eqnew,var.svar)
                                jointsolutions = [s.subs(symbols+[(var.svar,sin(var.var))]) for s in solutions]
                                solvedvars.append((var.var,SolverSolution(var.var.name, jointevalsin=jointsolutions), [self.codeComplexity(s) for s in jointsolutions]))
                        except (ValueError, AttributeError):
                            pass
                    
                    if numcvar > 0:
                        # substite sin
                        try:
                            if self.countVariables(eq.subs(sin(var.var),var.svar),var.svar) <= 1: # anything more than 1 implies quartic equation
                                eqnew = eq.subs(self.freevarsubs+[(sin(var.var),sqrt(Real(1,30)-var.cvar**2)),(cos(var.var),var.cvar)])
                                eqnew,symbols = self.factorLinearTerms(eqnew,[var.cvar])
                                solutions = self.customtsolve(eqnew,var.cvar)
                                jointsolutions = [s.subs(symbols+[(var.cvar,cos(var.var))]) for s in solutions]
                                solvedvars.append((var.var,SolverSolution(var.var.name, jointevalcos=jointsolutions), [self.codeComplexity(s) for s in jointsolutions]))
                        except (ValueError, AttributeError):
                            pass
                    
                    if numsvar == 0 and numcvar == 0:
                        eqnew = eq.subs(self.freevarsubs)
                        eqnew,symbols = self.factorLinearTerms(eqnew,[var.var])
                        solutions = self.customtsolve(eqnew,var.var)
                        jointsolutions = [s.subs(symbols) for s in solutions]
                        solvedvars.append((var.var,SolverSolution(var.var.name, jointeval=jointsolutions), [self.codeComplexity(s) for s in jointsolutions]))
        return solvedvars

    # solve for just the translation component
    def solveIKTranslation(self, Positions, Positionsee, rawvars):
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
                        if self.isExpressionUnique(eqns,p[j]) and self.isExpressionUnique(eqns,-p[j]):
                            eqns.append(p[j])

            if len(eqns) == 0:
                continue

            if len(eqns) > 1:
                neweqns = []
                listsymbols = []
                symbolgen = cse_main.numbered_symbols('const')
                for e in eqns:
                    enew, symbols = self.removeConstants(e.subs(self.freevarsubs+[(sin(var.var),var.svar),(cos(var.var),var.cvar)]),[var.cvar,var.svar,var.var], symbolgen)
                    enew2,symbols2 = self.factorLinearTerms(enew,[var.svar,var.cvar,var.var],symbolgen)
                    symbols += [(s[0],s[1].subs(symbols)) for s in symbols2]
                    rank = self.codeComplexity(enew2)+reduce(lambda x,y: x+self.codeComplexity(y[1]),symbols,0)
                    neweqns.append((rank,enew2))
                    listsymbols += symbols
                
                # since we're solving for two variables, we only want to use two equations, so
                # start trying all the equations starting from the least complicated ones to the most until a solution is found
                eqcombinations = []
                for eqs in xcombinations(neweqns,2):
                    eqcombinations.append((eqs[0][0]+eqs[1][0],[Eq(e[1],0) for e in eqs]))
                eqcombinations.sort(lambda x, y: x[0]-y[0])
                
                solution = None
                for comb in eqcombinations:
                    # try to solve for both sin and cos terms
                    s = solve(comb[1],[var.svar,var.cvar])
                    if s is not None and s.has_key(var.svar) and s.has_key(var.cvar):
                        
                        if self.chop((s[var.svar]-s[var.cvar]).subs(listsymbols),6) == 0:
                            continue
                        if s[var.svar].is_fraction() and s[var.cvar].is_fraction():
                            # check the numerator and denominator
                            if self.chop((s[var.svar].args[0]-s[var.cvar].args[0]).subs(listsymbols),6) == 0 and self.chop((s[var.svar].args[1]-s[var.cvar].args[1]).subs(listsymbols),6) == 0:
                                continue
                        solution = s
                        break
                if solution is not None:
                    jointsolution = [self.customtrigsimp(atan2(solution[var.svar],solution[var.cvar]).subs(listsymbols), deep=True).subs(freevarinvsubs)]
                    solvedvars.append((var.var,SolverSolution(var.var.name,jointsolution), [self.codeComplexity(s) for s in jointsolution]))
                    continue
            
            # solve one equation
            eq = eqns[0]
            symbolgen = cse_main.numbered_symbols('const')
            eqnew, symbols = self.removeConstants(eq.subs(self.freevarsubs+[(sin(var.var),var.svar),(cos(var.var),var.cvar)]), [var.cvar,var.svar,var.var], symbolgen)
            eqnew2,symbols2 = self.factorLinearTerms(eqnew,[var.cvar,var.svar,var.var], symbolgen)
            symbols += [(s[0],s[1].subs(symbols)) for s in symbols2]

            numcvar = self.countVariables(eqnew2,var.cvar)
            numsvar = self.countVariables(eqnew2,var.svar)
            if numcvar == 1 and numsvar == 1:
                a = Wild('a',exclude=[var.svar,var.cvar])
                b = Wild('b',exclude=[var.svar,var.cvar])
                c = Wild('c',exclude=[var.svar,var.cvar])
                m = eqnew2.match(a*var.cvar+b*var.svar+c)
                if m is not None:
                    symbols += [(var.svar,sin(var.var)),(var.cvar,cos(var.var))]+freevarinvsubs
                    asinsol = trigsimp(asin(-m[c]/sqrt(m[a]*m[a]+m[b]*m[b])).subs(symbols),deep=True)
                    constsol = -atan2(m[a],m[b]).subs(symbols).evalf()
                    jointsolutions = [constsol+asinsol,constsol+pi.evalf()-asinsol]
                    solvedvars.append((var.var,SolverSolution(var.var.name,jointeval=jointsolutions), [self.codeComplexity(s) for s in jointsolutions]))
                    continue
            
            if numcvar > 0:
                try:
                    # substitute cos
                    if self.countVariables(eqnew2,var.svar) <= 1: # anything more than 1 implies quartic equation
                        solutions = self.customtsolve(eqnew2.subs(var.svar,sqrt(1-var.cvar**2)),var.cvar)
                        jointsolutions = [s.subs(symbols+[(var.cvar,cos(var.var))]) for s in solutions]
                        solvedvars.append((var.var,SolverSolution(var.var.name, jointevalcos=jointsolutions), [self.codeComplexity(s) for s in jointsolutions]))
                except (ValueError):
                    pass

            if numsvar > 0:
                # substitute sin
                try:
                    if self.countVariables(eqnew2,var.svar) <= 1: # anything more than 1 implies quartic equation
                        solutions = self.customtsolve(eqnew2.subs(var.cvar,sqrt(1-var.svar**2)),var.svar)
                        jointsolutions = [trigsimp(s.subs(symbols+[(var.svar,sin(var.var))])) for s in solutions]
                        solvedvars.append((var.var,SolverSolution(var.var.name, jointevalsin=jointsolutions), [self.codeComplexity(s) for s in jointsolutions]))
                except (ValueError):
                    pass

            if numcvar == 0 and numsvar == 0:
                solutions = self.customtsolve(eqnew2,var.var)
                jointsolutions = [self.customtrigsimp(s.subs(symbols)) for s in solutions]
                solvedvars.append((var.var,SolverSolution(var.var.name, jointeval=jointsolutions), [self.codeComplexity(s) for s in jointsolutions]))
        return solvedvars

    # solve for just the rotation component
    def solveIKRotation(self, R, Ree, rawvars,endbranchtree=None,solvedvarsubs=[],ignorezerochecks=[]):
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
            nexttree = []
            for var,solutions in solvedvars2.iteritems():
                if not solutions[1]:
                    nexttree.append(SolverSolution(var.name, jointeval=solutions[0]))
                    nextvars = filter(lambda x: not x.var == var, nextvars)
            if len(nexttree) == 0:
                for var,solutions in solvedvars2.iteritems():
                    if solutions[1]:
                        nexttree.append(SolverSolution(var.name, jointeval=solutions[0], AddPi=True))
                        nextvars = filter(lambda x: not x.var == var, nextvars)
                        break

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
                                                # have to use the simplified solution
                                                solvedvars[var.var] = [[atan2(*fraction(simplified))],True,divisor]
                                            else:
                                                solvedvars[var.var] = [[varsolution],False,divisor]
                                                havegoodsol = True

                                    if havegoodsol:
                                        break
                                if havegoodsol:
                                    break
                            if havegoodsol:
                                break
                        if havegoodsol:
                            break
                    if havegoodsol:
                        break
                if havegoodsol:
                    break
        
        return solvedvars

    ## SymPy helper routines

    # factors linear terms together
    def factorLinearTerms(self,expr,vars,symbolgen = None):
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

    def removeConstants(self,expr,vars,symbolgen = None):
        """Separates all terms that do have var in them"""
        if symbolgen is None:
            symbolgen = cse_main.numbered_symbols('const')
        if expr.is_Add:
            newexpr = S.Zero
            cexpr = S.Zero
            symbols = []
            for term in expr.args:
                if term.has_any_symbols(*vars):
                    expr2, symbols2 = self.removeConstants(term,vars,symbolgen)
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
                    expr2, symbols2 = self.removeConstants(term,vars,symbolgen)
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

    def customtsolve(self,eq, sym):
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

            sol = self.customtsolve(lhs-rhs, sym)
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
                    cv_inv = self.customtsolve( t - f1, sym )[0]
                else:
                    cv_inv = solve( t - f1, sym )[0]
                
                sols = list()
                if cv_inv.is_Pow:
                    for sol in cv_sols:
                        neweq = (pow(sym,1/cv_inv.exp)-cv_inv.base.subs(t, sol)).expand()
                        symbolgen = cse_main.numbered_symbols('tsolveconst')
                        neweq2,symbols = self.factorLinearTerms(neweq,[sym],symbolgen)
                        neweq3,symbols2 = self.removeConstants(neweq2,[sym],symbolgen)
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
        
        raise ValueError('unable to solve the equation')

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
    parser.add_option('--usedummyjoints', action='store_true',dest='usedummyjoints',default=False,
                      help='Treat the unspecified joints in the kinematic chain as dummy and set them to 0. If not specified, treats all unspecified joints as free parameters.')

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
    code = kinematics.generateIkSolver(options.baselink,options.eelink,solvejoints,options.freeparams,options.usedummyjoints,solvefn=solvefn)

    success = True if len(code) > 0 else False

    print 'total time for ik generation of %s is %fs'%(options.savefile,time.time()-tstart)
    if success:
        open(options.savefile,'w').write(code)

    sys.exit(0 if success else 1)
