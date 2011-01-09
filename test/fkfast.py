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
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Lesser GPL, Version 3'

from sympy import *
import openravepy
import numpy
try:
    from openravepy.metaclass import AutoReloader
except:
    class AutoReloader:
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

class FKFastSolver(AutoReloader):
    """Solves the analytical forwards kinematics equations of a robot and outputs them in specified programming languages.
    Can handle closed-loops and mimic joints.
    """
    def __init__(self, kinbody=None,kinematicshash='',accuracy=None,precision=None):
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
        self.kinbody = kinbody
        self.iksolver = openravepy.ikfast.IKFastSolver(kinbody=self.kinbody,accuracy=self.accuracy,precision=self.precision)
        self.iksolver.IsHinge = self.IsHinge # temporary hack

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

    def affineSimplify(self, T):
        # yes, it is necessary to call self.trigsimp so many times since it gives up too easily
        values = [trigsimp(x.expand()) for x in T]
        for i in range(12):
            if (i%4)<3: # rotation should have bigger accuracy threshold
                values[i] = self.chop(values[i],accuracy=self.accuracy*10.0)
            else: # translation
                values[i] = self.chop(values[i])
        return Matrix(4,4,values)

    @staticmethod
    def affineInverse(affinematrix):
        T = eye(4)
        T[0:3,0:3] = affinematrix[0:3,0:3].transpose()
        T[0:3,3] = -affinematrix[0:3,0:3].transpose() * affinematrix[0:3,3]
        return T

    @staticmethod
    def normalizeRotation(M):
        right = M[0,0:3]/sqrt(M[0,0:3].dot(M[0,0:3]))
        up = M[1,0:3] - right*right.dot(M[1,0:3])
        up = up/sqrt(up.dot(up))
        M[0,0:3] = right
        M[1,0:3] = up
        M[2,0:3] = right.cross(up)
        return M

    @staticmethod
    def numpyMatrixToSympy(T):
        return FKFastSolver.normalizeRotation(Matrix(4,4,[Real(round(float(x),5),30) for x in T.flat]))

    @staticmethod
    def numpyVectorToSympy(v):
        return Matrix(3,1,[Real(round(float(x),4),30) for x in v])

    @staticmethod
    def rodrigues(axis, angle):
        skewsymmetric = Matrix(3, 3, [S.Zero,-axis[2],axis[1],axis[2],S.Zero,-axis[0],-axis[1],axis[0],S.Zero])
        return eye(3) + sin(angle) * skewsymmetric + (S.One-cos(angle))*skewsymmetric*skewsymmetric
        
    def forwardKinematicsChain(self, chainlinks, chainjoints):
        with self.kinbody:
            assert(len(chainjoints)+1==len(chainlinks))
            Links = []
            Tright = eye(4)
            jointvars = []
            isolvejointvars = []
            ifreejointvars = []
            jointinds = []
            self.jointindexmap = dict()
            jointindexmap_inv = dict()
            for i,joint in enumerate(chainjoints):
                if not joint.IsStatic():
                    if not joint in self.jointindexmap:
                        self.jointindexmap[len(jointvars)] = joint
                        jointindexmap_inv[joint] = len(jointvars)
                        var = Symbol(joint.GetName())
                    else:
                        var = Symbol(jointindexmap_inv[joint].GetName())

                    if chainjoints[i].GetHierarchyParentLink() == chainlinks[i]:
                        TLeftjoint = self.numpyMatrixToSympy(joint.GetInternalHierarchyLeftTransform())
                        TRightjoint = self.numpyMatrixToSympy(joint.GetInternalHierarchyRightTransform())
                        jaxis = FKFastSolver.numpyVectorToSympy(joint.GetInternalHierarchyAxis(0))
                    else:
                        TLeftjoint = self.affineInverse(self.numpyMatrixToSympy(joint.GetInternalHierarchyRightTransform()))
                        TRightjoint = self.affineInverse(self.numpyMatrixToSympy(joint.GetInternalHierarchyLeftTransform()))
                        jaxis = -FKFastSolver.numpyVectorToSympy(joint.GetInternalHierarchyAxis(0))

                    Tjoint = eye(4)
                    if joint.GetType() == openravepy.KinBody.Joint.Type.Hinge:
                        Tjoint[0:3,0:3] = self.rodrigues(jaxis,var)
                    elif joint.GetType() == openravepy.KinBody.Joint.Type.Slider:
                        Tjoint[0:3,3] = jaxis*(var)
                    else:
                        raise ValueError('failed to process joint type %s'%joint.GetType())

                    if i > 0 and chainjoints[i]==chainjoints[i-1]:
                        # the joint is the same as the last joint
                        Links[-1] = self.affineSimplify(Links[-1] * Tright * Tleftjoint * Tjoint)
                        Tright = TRightjoint
                    else:
                        # the joints are different, so add regularly
                        jointvars.append(var)
                        if False: # not free joint
                            ifreejointvars.append(len(jointvars)-1)
                            if len(isolvejointvars) == 0:
                                # no unknown joints have been found yet, so instead of adding a new entry, merge with previous matrix
                                Tright = Tright * TLeftjoint * Tjoint * TRightjoint
                                continue
                        else:
                            isolvejointvars.append(len(jointvars)-1)
                        Links.append(Tright * TLeftjoint)
                        jointinds.append(len(Links))
                        Links.append(Tjoint)
                        Tright = TRightjoint
                else:
                    Tright = self.affineSimplify(Tright * TLeftjoint * TRightjoint)
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

        return Links, jointvars, isolvejointvars, ifreejointvars

    def IsHinge(self,jointname):
        j = self.kinbody.GetJoint(jointname)
        if j is None:
            return True # dummy joint most likely for angles
        return j.GetType()==openravepy.KinBody.Joint.Type.Hinge

    @staticmethod
    def expandsincos(angles):
        if len(angles) == 1:
            return cos(angles[0]),sin(angles[0])
        othercos,othersin = FKFastSolver.expandsincos(angles[1:])
        return (cos(angles[0])*othercos - sin(angles[0])*othersin).expand(),( cos(angles[0])*othersin + sin(angles[0])*othercos).expand()

    def isZero(self,eq,vars,num=8):
        for i in range(num):
            subs = [(v,numpy.random.rand()) for v in vars]
            eq2=self.chop(eq.subs(subs).evalf()).evalf()
            if eq2 == S.Zero:
                return True
        return False
    def substituteVariable(self,eqs,var,othervars):
        cvar = Symbol('c%s'%var.name)
        svar = Symbol('s%s'%var.name)
        varsubs = [(cos(var),cvar),(sin(var),svar)]
        othervarsubs = [(sin(v)**2,1-cos(v)**2) for v in othervars]
        eqpolys = [Poly(eq.subs(varsubs),cvar,svar) for eq in eqs]
        eqpolys = [eq for eq in eqpolys if eq.degree == 1 and not eq.coeff(0,0).has_any_symbols(var)]
        #eqpolys.sort(lambda x,y: iksolver.codeComplexity(x) - iksolver.codeComplexity(y))
        partialsolutions = []
        neweqs = []
        for p0,p1 in combinations(eqpolys,2):
            M = Matrix(2,3,[p0.coeff(1,0),p0.coeff(0,1),p0.coeff(0,0),p1.coeff(1,0),p1.coeff(0,1),p1.coeff(0,0)])
            M = M.subs(othervarsubs).expand()
            partialsolution = [-M[1,1]*M[0,2]+M[0,1]*M[1,2],M[1,0]*M[0,2]-M[0,0]*M[1,2],M[0,0]*M[1,1]-M[0,1]*M[1,0]]
            partialsolution = [eq.expand().subs(othervarsubs).expand() for eq in partialsolution]
            rank = [self.iksolver.codeComplexity(eq) for eq in partialsolution]
            partialsolutions.append([rank,partialsolution])
            # cos(A)**2 + sin(A)**2 - 1 = 0
            neweqs.append(partialsolution[0]**2+partialsolution[1]**2-partialsolution[2]**2)
        # try to cross
        partialsolutions.sort(lambda x, y: int(min(x[0])-min(y[0])))
        for (rank0,ps0),(rank1,ps1) in combinations(partialsolutions,2):
            if self.isZero(ps0[0]*ps1[2]-ps1[0]*ps0[2],othervars):
                continue
            neweqs.append(ps0[0]*ps1[2]-ps1[0]*ps0[2])
            neweqs.append(ps0[1]*ps1[2]-ps1[1]*ps0[2])
            neweqs.append(ps0[0]*ps1[1]-ps1[0]*ps0[1])
            neweqs.append(ps0[0]*ps1[0]+ps0[1]*ps1[1]-ps0[2]*ps1[2])
            break;
        return [self.chop(self.chop(eq.expand().subs(othervarsubs).expand())) for eq in neweqs]

    def PlanarLoop(self):
        """Solve the equations knowing the loop is planar, work in progress"""
        tempa = Symbol('__A__') # total angle sum
        ca = Symbol('c__A__')
        sa = Symbol('s__A__')
        Asubs = [(cos(tempa),ca),(sin(tempa),sa)]
        solvejointvars.append(tempa)
        planenormal = Matrix(3,1,(axes[0][0],axes[0][1],axes[0][2]))
        if abs(planenormal.dot([1,0,0])) > S.One-1e-8:
            xaxis = Matrix(3,1,[0.0,1.0,0.0])
            yaxis = Matrix(3,1,[0.0,0.0,1.0])
        elif abs(planenormal.dot([0,1,0])) > S.One-1e-8:
            xaxis = Matrix(3,1,[1.0,0.0,0.0])
            yaxis = Matrix(3,1,[0.0,0.0,1.0])
        else:
            xaxis = Matrix(3,1,[1.0,0.0,0.0])
            yaxis = Matrix(3,1,[0.0,1.0,0.0])
        xaxis = (xaxis - planenormal*planenormal.dot(xaxis)).normalized()
        yaxis = (yaxis - planenormal*planenormal.dot(yaxis)).normalized()
        # extract all coordinates on this plane, note that rotations just become equations on the pure angles
        newtransformations = []
        for patheqs in pathseqs:
            vars=patheqs[2]
            joints = [self.kinbody.GetJoint(var.name) for var in vars]
            anglesigns = [sign(planenormal.dot(list(j.GetAxis(0)))) if j.GetType() == openravepy.KinBody.Joint.Type.Hinge else 0.0  for j in joints]
            angleeq = S.Zero
            for varsign,var in izip(anglesigns,patheqs[2]):
                angleeq += varsign*var
            xeq = self.chop(patheqs[0][0:3,3].dot(xaxis))
            yeq = self.chop(patheqs[0][0:3,3].dot(yaxis))
            xylength = iksolver.customtrigsimp(self.chop(iksolver.customtrigsimp(iksolver.customtrigsimp((xeq**2+yeq**2).expand()))))
            indices = [i for i,a in enumerate(anglesigns) if a != 0]
            if len(indices) == 0:
                newtransformations.append([xeq, yeq, angleeq,xylength])
                continue
            # A = sum(angles) => one_angle = A - rest_of_angles
            angles = [tempa]
            outangle = vars[indices[0]]
            othervarsubs = []
            othervars = []
            for i,anglesign in enumerate(anglesigns):
                if i != indices[0] and anglesign != 0:
                    angles.append(-anglesign*vars[i])
                    othervarsubs.append((sin(vars[i])**2,1-cos(vars[i])**2))
                    othervars.append(vars[i])
            newvaluecos,newvaluesin = self.expandsincos(angles)
            varsubs = [(cos(outangle),newvaluecos),(sin(outangle),newvaluesin)]
            xeq = xeq.subs(varsubs).expand()
            yeq = yeq.subs(varsubs).expand()
            xylength = xylength.subs(varsubs).expand()
            xeq = xeq.subs(othervarsubs).expand()
            yeq = yeq.subs(othervarsubs).expand()
            xylength = xylength.subs(othervarsubs).expand()
            eqs = [xeq, yeq, xylength]
            newtransformations.append([eqs, angleeq-tempa,othervars])
            solvejointvars.remove(outangle)
        AllEquations = []
        APartialSolutions = []
        for (eqs0,angleeq0,othervars0),(eqs1,angleeq1,othervars1) in combinations(newtransformations,2):
            othervars=othervars0+othervars1
            eqdiff = [p0-p1 for p0,p1 in izip(eqs0,eqs1)] # == 0
            aeqs = self.substituteVariable(eqdiff,tempa,othervars)
            self.iksolver.sortComplexity(aeqs)
            # only one linearly-independent equation can be extracted
            AllEquations += [simplify(eq) for eq in eqdiff]
            AllEquations.append(aeqs[0])
        iksolver.sortComplexity(AllEquations)

    def SolveFK(self):
        knot = 2
        knownlinks = [0]
        knotlink = self.kinbody.GetLinks()[knot]
        loops = self.kinbody.GetClosedLoops()

        # compute known link transformations at this point
        knownlink = self.kinbody.GetLinks()[knownlinks[0]]
        pathstoknot = [(self.kinbody.GetChain(knownlink.GetIndex(),knot,returnjoints=False), self.kinbody.GetChain(knownlink.GetIndex(),knot,returnjoints=True))]
        for loop in loops:
            lknotindex = [i for i,(link,joint) in enumerate(loop) if link.GetIndex() == knot]
            lknownindex = [i for i,(link,joint) in enumerate(loop) if link == knownlink]
            if len(lknotindex) > 0 and len(lknownindex) > 0:
                knotindex = lknotindex[0]
                knownindex = lknownindex[0]
                if knownindex < knotindex:
                    subloop = loop[knownindex:(knotindex+1)]
                    path = ([l for l,j in subloop],[j for l,j in subloop[:-1]])
                    if not path in pathstoknot:
                        pathstoknot.append(path)
                    subloop = loop[knownindex::-1]+loop[:knotindex:-1]+[loop[knotindex]]
                    path = ([l for l,j in subloop],[j for l,j in subloop[1:]])
                    if not path in pathstoknot:
                        pathstoknot.append(path)
                else:
                    assert False # not implemented yet

        pathseqs = []
        for path in pathstoknot:
            Links, jointvars, isolvejointvars, ifreejointvars = self.forwardKinematicsChain(path[0],path[1])
            Tfirstleft = Links.pop(0)
            Tfirstright = Links.pop()
            LinksInv = [self.affineInverse(link) for link in Links]
            #Tee = Tfirstleft.inv() * Tee_in * Tfirstright.inv()
            # LinksAccumLeftInv[x] * Tee = LinksAccumRight[x]
            # LinksAccumLeftInv[x] = InvLinks[x-1] * ... * InvLinks[0]
            # LinksAccumRight[x] = Links[x]*Links[x+1]...*Links[-1]
            LinksAccumLeftInvAll = [eye(4)]
            LinksAccumRightAll = [eye(4)]
            for i in range(len(Links)):
                LinksAccumLeftInvAll.append(LinksInv[i]*LinksAccumLeftInvAll[-1])
                LinksAccumRightAll.append(Links[len(Links)-i-1]*LinksAccumRightAll[-1])
            LinksAccumRightAll.reverse()
            pathseqs.append([self.affineSimplify(LinksAccumRightAll[0]),self.affineSimplify(LinksAccumLeftInvAll[-1]),[Tfirstleft,Tfirstright],jointvars,isolvejointvars,ifreejointvars])

        alljointvars = set()
        for patheqs in pathseqs:
            alljointvars = alljointvars.union(patheqs[3])
        alljoints = [self.kinbody.GetJoint(var.name) for var in alljointvars]
        # see if all hinge joints are on the same plane (allows us to simplify things a lot)
        axes = [j.GetAxis(0) for j in alljoints if j.GetType() == openravepy.KinBody.Joint.Type.Hinge]
        isplanar = all([abs(dot(axis,axes[0]))>0.999999 for axis in axes])
        if isplanar:
            print 'robot is planar, can add sum of angles of all loops as an equation!'
        iksolver = self.iksolver
        solvejointvars = list(alljointvars)

        #print 'mechanism is not planar, going to 3D equations...'
        AllEquations = []
        for patheqs0,patheqs1 in combinations(pathseqs,2):
            Tleft0,Tright0 = patheqs0[2]
            T0,T0inv = patheqs0[0:2]
            Tleft1,Tright1 = patheqs1[2]
            T1,T1inv = patheqs1[0:2]
            Tleft = self.affineInverse(Tleft1)*Tleft0
            Tleftinv = self.affineInverse(Tleft0)*Tleft1
            Tright = Tright0 * self.affineInverse(Tright1)
            Trightinv = Tright1 * self.affineInverse(Tright0)
            T = Tleft*T0*Tright - T1
            for i in range(12):
                AllEquations.append(T[i])
            T = T0 - Tleftinv*T1*Trightinv
            for i in range(12):
                AllEquations.append(T[i])
        self.iksolver.sortComplexity(AllEquations)
        i = 1
        while i < len(AllEquations):
            if not self.iksolver.isExpressionUnique(AllEquations[:i],AllEquations[i]):
                eq=AllEquations.pop(i)
            else:
                i += 1
        # create the length equations by moving the coordinate systems
        for patheqs,path in izip(pathseqs,pathstoknot):
            # find an appropriate coordinate system
            baselink,basejoint = path[0][0],path[1][0]
            endlink,endjoint = path[0][-1],path[1][-1]
            if basejoint.GetHierarchyChildLink() == baselink:
                Tleft = FKFastSolver.numpyMatrixToSympy(basejoint.GetInternalHierarchyRightTransform())
            elif basejoint.GetHierarchyParentLink() == baselink:
                Tleft = self.affineInverse(FKFastSolver.numpyMatrixToSympy(basejoint.GetInternalHierarchyLeftTransform()))
            else:
                Tleft = eye(4)
            if endjoint.GetHierarchyChildLink() == endlink:
                Tright = self.affineInverse(FKFastSolver.numpyMatrixToSympy(endjoint.GetInternalHierarchyRightTransform()))
            elif endjoint.GetHierarchyParentLink() == endlink:
                Tright = FKFastSolver.numpyMatrixToSympy(endjoint.GetInternalHierarchyLeftTransform())
            else:
                Tright = eye(4)
            Lengths = []
            for patheqs2 in pathseqs:
                T = Tleft*patheqs2[2][0]*patheqs2[0]*patheqs2[2][1]*Tright
                pos = T[0:3,3]
                eq = (pos[0]**2+pos[1]**2+pos[2]**2).expand()
                if self.iksolver.codeComplexity(eq) < 1500:
                    # sympy's trigsimp/customtrigsimp give up too easily
                    eq = self.iksolver.chop(self.iksolver.customtrigsimp(self.iksolver.customtrigsimp(self.iksolver.customtrigsimp(eq)).expand()))
                    if self.iksolver.isExpressionUnique(Lengths,eq) and self.iksolver.isExpressionUnique(Lengths,-eq):
                        Lengths.append(eq)
            for l0,l1 in combinations(Lengths,2):
                eq = l0-l1
                if self.iksolver.isExpressionUnique(AllEquations,eq) and self.iksolver.isExpressionUnique(AllEquations,-eq):
                    AllEquations.append(eq)
        self.iksolver.sortComplexity(AllEquations)

        dof = 1 # way to automatically determine?
        # pick free variables until equations become solvable
        possiblefreevars = [v for v in alljointvars if self.kinbody.GetJoint(v.name).GetDOFIndex()>=0]
        for othersolvedvars in combinations(possiblefreevars,dof):
            curvars = solvejointvars[:]
            solsubs = []
            for v in othersolvedvars:
                curvars.remove(v)
                solsubs += [(cos(v),Symbol('c%s'%v.name)),(sin(v),Symbol('s%s'%v.name))]
            endbranchtree = [openravepy.ikfast.SolverStoreSolution (jointvars)]
            try:
                tree = iksolver.solveAllEquations(AllEquations,curvars=curvars,othersolvedvars=othersolvedvars,solsubs=solsubs,endbranchtree=endbranchtree)
                print 'forward kinematics solved!'
                # sometimes there are multiple solutions to variables, do sanity checks knowing the initial values of the robot
                return tree
            except self.iksolver.CannotSolveError:
                pass
            
        return None

def test_bobcat():
    env.Reset()
    env.StopSimulation()
    robot = env.ReadRobotXMLFile('arm_bucket_assy.robot.xml')
    env.AddRobot(robot)

    import fkfast
    from fkfast import FKFastSolver
    fksolver = fkfast.FKFastSolver(robot)
    self=fksolver

    P0 = Symbol('P0')
    P1 = Symbol('P1')
    P2 = Symbol('P2')
    P3 = Symbol('P3')
    P4 = Symbol('P4')
    P5 = Symbol('P5')
    A=Symbol('A')
    __A__=Symbol('__A__')
    curvars=[P0,P5,A]
    othersolvedvars=[P1,P2,P3,P4]
    solsubs=[]
    for v in othersolvedvars:
        solsubs.append((cos(v),Symbol('c%s'%v.name)))
        solsubs.append((sin(v),Symbol('s%s'%v.name)))
    othersolvedvars.append(A)
    cursubs = []
    for v in othersolvedvars:
        if robot.GetJoint(v.name) is not None:
            value = robot.GetJoint(v.name).GetValues()[0]
        else:
            value = 0
        cursubs += [(v,value),(Symbol('c%s'%v.name,),cos(value).evalf()), (Symbol('s%s'%v.name,),sin(value).evalf())]
    allsubs = cursubs + [(P0,0)] + [(P5,0)] + [(A,0)]
