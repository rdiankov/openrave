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
        self.kinbody = kinbody
        self.iksolver = openravepy.ikfast.IKFastSolver(kinbody=self.kinbody,accuracy=accuracy,precision=precision)
        self.iksolver.IsHinge = self.IsHinge # temporary hack

    def normalizeRotation(self,M):
        right = M[0,0:3]/sqrt(M[0,0:3].dot(M[0,0:3]))
        for i in range(3):
            right[i] = self.iksolver.chop(right[i])
        up = M[1,0:3] - right*right.dot(M[1,0:3])
        up = up/sqrt(up.dot(up))
        for i in range(3):
            up[i] = self.iksolver.chop(up[i])
        M[0,0:3] = right
        M[1,0:3] = up
        M[2,0:3] = right.cross(up)
        for i in range(3):
            M[i,3] = self.iksolver.chop(M[i,3])
        return M

    def numpyMatrixToSympy(self,T):
        return self.normalizeRotation(Matrix(4,4,[Real(float(x),30) for x in T.flat]))

    def numpyVectorToSympy(self,v):
        return Matrix(3,1,[self.iksolver.chop(Real(float(x),30)) for x in v])
        
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
                if chainjoints[i].GetHierarchyParentLink() == chainlinks[i]:
                    TLeftjoint = self.numpyMatrixToSympy(joint.GetInternalHierarchyLeftTransform())
                    TRightjoint = self.numpyMatrixToSympy(joint.GetInternalHierarchyRightTransform())
                    jaxis = self.numpyVectorToSympy(joint.GetInternalHierarchyAxis(0))
                else:
                    TLeftjoint = self.iksolver.affineInverse(self.numpyMatrixToSympy(joint.GetInternalHierarchyRightTransform()))
                    TRightjoint = self.iksolver.affineInverse(self.numpyMatrixToSympy(joint.GetInternalHierarchyLeftTransform()))
                    jaxis = -self.numpyVectorToSympy(joint.GetInternalHierarchyAxis(0))
                if not joint.IsStatic():
                    if not joint in self.jointindexmap:
                        self.jointindexmap[len(jointvars)] = joint
                        jointindexmap_inv[joint] = len(jointvars)
                        var = Symbol(joint.GetName())
                    else:
                        var = Symbol(jointindexmap_inv[joint].GetName())

                    Tjoint = eye(4)
                    if joint.GetType() == openravepy.KinBody.Joint.Type.Hinge:
                        Tjoint[0:3,0:3] = self.iksolver.rodrigues(jaxis,var)
                    elif joint.GetType() == openravepy.KinBody.Joint.Type.Slider:
                        Tjoint[0:3,3] = jaxis*(var)
                    else:
                        raise ValueError('failed to process joint type %s'%joint.GetType())

                    if i > 0 and chainjoints[i]==chainjoints[i-1]:
                        # the joint is the same as the last joint
                        Links[-1] = self.iksolver.affineSimplify(Links[-1] * Tright * TLeftjoint * Tjoint)
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
                    Tright = self.iksolver.affineSimplify(Tright * TLeftjoint * TRightjoint)
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
            Links[iright-1] = Links[iright-1] * self.iksolver.affineInverse(Ttrans)
            print "moved translation ",Ttrans[0:3,3].transpose(),"to right end"
        
        if len(jointinds) > 1:
            ileft = jointinds[0]
            separated_trans = Links[ileft][0:3,0:3] * Links[ileft+1][0:3,3]
            Ttrans = eye(4)
            for j in range(0,3):
                if not separated_trans[j].has_any_symbols(*jointvars):
                    Ttrans[j,3] = separated_trans[j]
            Links[ileft-1] = Links[ileft-1] * Ttrans
            Links[ileft+1] = self.iksolver.affineInverse(Ttrans) * Links[ileft+1]
            print "moved translation ",Ttrans[0:3,3].transpose(),"to left end"

        if len(jointinds) > 3: # last 3 axes always have to be intersecting, move the translation of the first axis to the left
            ileft = jointinds[-3]
            separated_trans = Links[ileft][0:3,0:3] * Links[ileft+1][0:3,3]
            Ttrans = eye(4)
            for j in range(0,3):
                if not separated_trans[j].has_any_symbols(*jointvars):
                    Ttrans[j,3] = separated_trans[j]
            Links[ileft-1] = Links[ileft-1] * Ttrans
            Links[ileft+1] = self.iksolver.affineInverse(Ttrans) * Links[ileft+1]
            print "moved translation on intersecting axis ",Ttrans[0:3,3].transpose(),"to left"

        return Links, jointvars, isolvejointvars, ifreejointvars

    def IsHinge(self,jointname):
        j = self.kinbody.GetJoint(jointname)
        if j is not None:
            return j.GetType()==openravepy.KinBody.Joint.Type.Hinge
        if jointname.startswith('dummy') >= 0:
            return False
        return True # dummy joint most likely for angles

    def isZero(self,eq,vars,num=8):
        for i in range(num):
            subs = [(v,numpy.random.rand()) for v in vars]
            eq2=self.iksolver.chop(eq.subs(subs).evalf()).evalf()
            if eq2 == S.Zero:
                return True
        return False

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
            LinksInv = [self.iksolver.affineInverse(link) for link in Links]
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
            pathseqs.append([self.iksolver.affineSimplify(LinksAccumRightAll[0]),self.iksolver.affineSimplify(LinksAccumLeftInvAll[-1]),[Tfirstleft,Tfirstright],jointvars,isolvejointvars,ifreejointvars])

        alljointvars = set()
        for patheqs in pathseqs:
            alljointvars = alljointvars.union(patheqs[3])
        alljoints = [self.kinbody.GetJoint(var.name) for var in alljointvars]
        solvejointvars = list(alljointvars)

        # take all joints with a dof index as the free dof
        possiblefreevars = [v for v in alljointvars if self.kinbody.GetJoint(v.name).GetDOFIndex()>=0]
        unknownvars = solvejointvars[:]
        for v in possiblefreevars:
            unknownvars.remove(v)
        unknownsubs = []
        for v in unknownvars:
            if self.IsHinge(v.name):
                unknownsubs += [(cos(v),Symbol('c%s'%v.name)),(sin(v),Symbol('s%s'%v.name))]

        Equations = []
        eliminatevargroups = []
        MUL=Symbol('MUL')
        for patheqs0,patheqs1 in combinations(pathseqs,2):
            Tleft0,Tright0 = patheqs0[2]
            T0,T0inv = patheqs0[0:2]
            Tleft1,Tright1 = patheqs1[2]
            T1,T1inv = patheqs1[0:2]
            Tleft = self.iksolver.affineInverse(Tleft1)*Tleft0
            Tright = Tright0 * self.iksolver.affineInverse(Tright1)
            PolyEquations,numminvars = self.iksolver.buildRaghavanRothEquations(Tleft*T0*Tright, T1,unknownvars)
            Equations2 = [[eq[0].as_basic().expand(),eq[1].as_basic().expand()] for eq in PolyEquations]
            Equations += Equations2
            for vars in [patheqs0[3],patheqs1[3]]:
                eliminatevargroup = []
                for eq0,eq1 in Equations2:
                    if eq0.has_any_symbols(*vars):
                        if eq0 != S.Zero and eq0.count_ops() == 1 or eq0.count_ops() == 2+MUL:
                            eliminatevargroup.append((eq0,eq1,int(eq0.count_ops().subs(MUL,4).evalf())))
                if len(eliminatevargroup) > 0:
                    eliminatevargroup.sort(lambda x,y: -x[2]+y[2])
                    eliminatevargroups.append(eliminatevargroup)

        reducesubs = []
        for var in solvejointvars:
            if self.IsHinge(var.name):
                reducesubs.append((sin(var)**2,1-cos(var)**2))
        # create the length equations by moving the coordinate systems
        for patheqs,path in izip(pathseqs,pathstoknot):
            # find an appropriate coordinate system
            baselink,basejoint = path[0][0],path[1][0]
            endlink,endjoint = path[0][-1],path[1][-1]
            if basejoint.GetHierarchyChildLink() == baselink:
                Tleft = self.numpyMatrixToSympy(basejoint.GetInternalHierarchyRightTransform())
            elif basejoint.GetHierarchyParentLink() == baselink:
                Tleft = self.iksolver.affineInverse(self.numpyMatrixToSympy(basejoint.GetInternalHierarchyLeftTransform()))
            else:
                Tleft = eye(4)
            if endjoint.GetHierarchyChildLink() == endlink:
                Tright = self.iksolver.affineInverse(self.numpyMatrixToSympy(endjoint.GetInternalHierarchyRightTransform()))
            elif endjoint.GetHierarchyParentLink() == endlink:
                Tright = self.numpyMatrixToSympy(endjoint.GetInternalHierarchyLeftTransform())
            else:
                Tright = eye(4)
            Lengths = []
            for patheqs2 in pathseqs:
                T = Tleft*patheqs2[2][0]*patheqs2[0]*patheqs2[2][1]*Tright
                pos = T[0:3,3]
                eq = (pos[0]**2+pos[1]**2+pos[2]**2).expand().subs(reducesubs).expand()
                if self.iksolver.codeComplexity(eq) < 1500:
                    # sympy's trigsimp/customtrigsimp give up too easily
                    eq = self.iksolver.chop(self.iksolver.customtrigsimp(self.iksolver.customtrigsimp(self.iksolver.customtrigsimp(eq)).expand()))
                    if self.iksolver.isExpressionUnique(Lengths,eq) and self.iksolver.isExpressionUnique(Lengths,-eq):
                        Lengths.append(eq.expand())
            for l0,l1 in combinations(Lengths,2):
                Equations.append([l0,l1])

        AllEquations = [eq[0]-eq[1] for eq in Equations]
        self.iksolver.sortComplexity(AllEquations)
        reducedeqs = []
        i = 0
        while i < len(AllEquations):
            reducedeq = self.iksolver.removecommonexprs(AllEquations[i])
            if self.iksolver.isExpressionUnique(reducedeqs,reducedeq) and self.iksolver.isExpressionUnique(reducedeqs,-reducedeq):
                reducedeqs.append(reducedeq)
                i += 1
            else:
                eq=AllEquations.pop(i)

        # make all possible substitutions, not certain how many new constraints this introduces
#         OrgEquations = AllEquations[:]
#         for eliminatevargroup in eliminatevargroups[0:1]:
#             usedvars = [var for var in unknownvars if any([eq[0].has_any_symbols(var) for eq in eliminatevargroup])]
#             for eq in OrgEquations:
#                 if not eq.has_any_symbols(*usedvars):
#                     continue
#                 for oldvalue,newvalue,rank in eliminatevargroup:
#                     eq = eq.subs(oldvalue,newvalue)
#                 eq = self.iksolver.chop(eq.expand(),accuracy=self.iksolver.accuracy)
#                 reducedeq = self.iksolver.removecommonexprs(eq)
#                 if self.chop(eq,accuracy=self.iksolver.accuracy*1000) != S.Zero and self.iksolver.isExpressionUnique(reducedeqs,reducedeq):
#                     AllEquations.append(eq)
#                     reducedeqs.append(reducedeq)

        # see if all hinge joints are on the same plane (allows us to simplify things a lot)
        axes = [j.GetAxis(0) for j in alljoints if j.GetType() == openravepy.KinBody.Joint.Type.Hinge]
        isplanar = all([abs(dot(axis,axes[0]))>0.999999 for axis in axes])
        if isplanar:
            print 'robot is planar, adding sum of angles of all loops'
            angleeqs = []
            commonaxis = None
            for patheqs in pathseqs:
                vars=patheqs[3]
                R = (patheqs[2][0]*patheqs[0]*patheqs[2][1])[0:3,0:3]
                valuesubs = [[var,S.Zero] for var in vars]
                R0 = R.subs(valuesubs).evalf()
                axes = []
                # to find the angle signs, test pi/2 for each value
                for i in range(len(vars)):
                    valuesubs[i][1] = pi/2
                    R1 = R.subs(valuesubs)
                    valuesubs[i][1] = S.Zero
                    Rdelta=R0.transpose()*R1
                    axes.append(axisAngleFromRotationMatrix(Rdelta.tolist()))
                if commonaxis == None:
                    commonaxis = axes[0]
                angleeq = S.Zero
                for i,var in enumerate(vars):
                    if self.IsHinge(var.name):
                        angleeq += sign(commonaxis[0]*axes[i][0]+commonaxis[1]*axes[i][1]+commonaxis[2]*axes[i][2]) * var
                angleeqs.append(angleeq)
            for angeq0,angeq1 in combinations(angleeqs,2):
                AllEquations.append(angeq0-angeq1)
        self.iksolver.sortComplexity(AllEquations)

        # do a sanity check
        maxvalue = max([eq.subs(allsubs).evalf() for eq in AllEquations])
        # maybe too strict?
        assert(maxvalue < 5e-10)
        if maxvalue > self.iksolver.accuracy:
            self.iksolver.accuracy = maxvalue*1.1
            print 'setting new accuracy to: ',self.iksolver.accuracy
            AllEquations = [self.iksolver.chop(eq,accuracy=maxvalue*2) for eq in AllEquations]
            self.iksolver.sortComplexity(AllEquations)

        # pick free variables until equations become solvable
        try:
            endbranchtree = [openravepy.ikfast.SolverStoreSolution (jointvars)]
            solsubs = []
            for v in possiblefreevars:
                if self.IsHinge(v.name):
                    solsubs += [(cos(v),Symbol('c%s'%v.name)),(sin(v),Symbol('s%s'%v.name))]
            tree = self.iksolver.solveAllEquations(AllEquations,curvars=unknownvars,othersolvedvars=possiblefreevars,solsubs=solsubs,endbranchtree=endbranchtree)
            print 'forward kinematics solved, setting equations!'
            self.setMimicEquations(tree)
            return tree
        except self.iksolver.CannotSolveError:
            pass
            
        return None

    def setMimicEquations(self,tree):
        # sometimes there are multiple solutions to variables, do sanity checks knowing the initial values of the robot
        pass
    

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
