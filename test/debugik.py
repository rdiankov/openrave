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
# random code that helps with debugging/testing the python interfaces and examples
# this is not meant to be run by normal users
from __future__ import with_statement # for python 2.5
__copyright__ = 'Copyright (C) 2009-2010'
__license__ = 'Apache License, Version 2.0'

# random code that helps with debugging/testing the python interfaces and examples
# this is not meant to be run by normal users
from openravepy import *
import openravepy.examples
from openravepy.interfaces import *
from numpy import *
import numpy,time

def test_ikgeneration():
    import inversekinematics
    env = Environment()
    env.SetDebugLevel(DebugLevel.Debug)
    #robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    robot = env.ReadRobotXMLFile('robots/barrettwam4.robot.xml')
    robot.SetActiveManipulator('arm')
    env.AddRobot(robot)
    self = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)

    freejoints=None
    usedummyjoints=False
    accuracy = None
    precision = None
    iktype=inversekinematics.InverseKinematicsModel.Type_Direction3D
    self.generate(freejoints=freejoints,usedummyjoints=usedummyjoints,iktype=iktype)

    baselink=self.manip.GetBase().GetIndex()
    eelink = self.manip.GetEndEffector().GetIndex()
    solvejoints=solvejoints
    freeparams=freejoints
    usedummyjoints=usedummyjoints
    solvefn=solvefn

def test_handstraight_jacobian():
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
    env.AddRobot(robot)
    # use jacobians for validation
    with env:
        deltastep = 0.01
        thresh=1e-5
        lower,upper = robot.GetDOFLimits()
        manip = robot.GetActiveManipulator()
        ilink = manip.GetEndEffector().GetIndex()
        localtrans = [0.1,0.2,0.3]
        localquat = [1.0,0.0,0.0,0.0]
        stats = []
        robot.SetActiveDOFs(manip.GetArmIndices())
        while True:
            robot.SetDOFValues(random.rand()*(upper-lower)+lower)
            if not robot.CheckSelfCollision() and not env.CheckCollision(robot):
                break
        deltatrans = deltastep*(random.rand(3)-0.5)
        while True:
            values = robot.GetDOFValues(manip.GetArmIndices())
            T = manip.GetEndEffectorTransform()
            Tnew = array(T)
            Tnew[0:3,3] += deltatrans
            sols = manip.FindIKSolutions(Tnew,IkFilterOptions.CheckEnvCollisions)
            if len(sols) == 0:
                break
            dists = sum( (array(sols)-tile(values,(len(sols),1)))**2, 1)
            sol = sols[argmin(dists)]            
            J = robot.CalculateActiveJacobian(ilink,manip.GetEndEffectorTransform()[0:3,3])
            Jrot = robot.CalculateActiveAngularVelocityJacobian(ilink)
            Jtrans = r_[J,Jrot]
            JJt = dot(Jtrans,transpose(Jtrans))
            deltavalues = dot(transpose(Jtrans),dot(linalg.inv(JJt),r_[deltatrans,0,0,0]))
            #dtt = dot(r_[deltatrans,0,0,0],JJt)
            #alpha = dot(r_[deltatrans,0,0,0], dtt)/dot(dtt,dtt)
            #deltavalues = alpha*dot(transpose(Jtrans),r_[deltatrans,0,0,0])
            realvalues = sol-values
            realtransdelta = dot(J,realvalues)
            #err = sum(abs(sign(deltatrans)-sign(realtransdelta)))
            err = dot(deltatrans,realtransdelta)/(linalg.norm(deltatrans)*linalg.norm(realtransdelta))
            d = sqrt(sum(realvalues**2)/sum(deltavalues**2))
            if err < 0.95 or d > 10:
                print realvalues
                print deltavalues
            stats.append((err,d))
            print stats[-1]
            robot.SetDOFValues(sol,manip.GetArmIndices())

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.hist(stats,100)
    fig.show()

def test_ik():
    import inversekinematics
    env = Environment()
    env.SetDebugLevel(DebugLevel.Debug)
    robot = env.ReadRobotXMLFile('/home/rdiankov/ros/honda/binpicking/robots/tx90.robot.xml')
    env.AddRobot(robot)
    manip=robot.GetActiveManipulator()
    #manip=robot.SetActiveManipulator('leftarm_torso')
    self = inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
    self.load()
    self.perftiming(10)
    robot.SetDOFValues([-2.62361, 1.5708, -0.17691, -3.2652, 0, -3.33643],manip.GetArmJoints())
    T=manip.GetEndEffectorTransform()
    print robot.CheckSelfCollision()
    #[j.SetJointLimits([-pi],[pi]) for j in robot.GetJoints()]
    robot.SetDOFValues(zeros(robot.GetDOF()))
    values=manip.FindIKSolution(T,False)
    Tlocal = dot(dot(linalg.inv(manip.GetBase().GetTransform()),T),linalg.inv(manip.GetGraspTransform()))
    print ' '.join(str(f) for f in Tlocal[0:3,0:4].flatten())
    robot.SetDOFValues (values,manip.GetArmJoints())
    print manip.GetEndEffectorTransform()
    
    sols=manip.FindIKSolutions(T,False)
    for i,sol in enumerate(sols):
        robot.SetDOFValues(sol)
        Tnew = manip.GetEndEffectorTransform()
        if sum((Tnew-T)**2) > 0.0001:
            print i
            break
        
def debug_ik():
    env = Environment()
    env.Load('data/katanatable.env.xml')
    env.StopSimulation()
    robot = env.GetRobots()[0]

    print robot.GetTransform()[0:3,3]
    target=array([-0.34087322,  0.64355438,  1.01439696])
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Translation3D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    sol = ikmodel.manip.FindIKSolution(IkParameterization(target,IkParameterization.Type.Translation3D),IkFilterOptions.CheckEnvCollisions)
    print sol
    robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
    print linalg.norm(target - ikmodel.manip.GetEndEffectorTransform()[0:3,3])

def solvefailed1(self):
    coupledvars = RightEquations[0].symbols[0:2] + RightEquations[0].symbols[4:]
    leftvars = RightEquations[0].symbols[2:4]
    unknownmonoms = [(1,0,1,0),(1,0,0,1),(0,1,1,0),(0,1,0,1),(1,0,0,0),(0,1,0,0)]
    unknownvars = [Symbol('dummy%d'%i) for i in range(len(unknownmonoms))]
    unknownsubs = []
    for var,monom in izip(unknownvars,unknownmonoms):
        unknownsubs.append((var,Poly(S.Zero,*coupledvars).add_term(S.One,monom).as_basic()))
    leftsideeqs = []
    rightsideeqs = []
    for peq in RightEquations:
        peq = Poly(peq,*coupledvars)
        leftside = Poly(S.Zero,*unknownvars)
        rightside = S.Zero
        for c,m in peq.iter_terms():
            if m in unknownmonoms:
                leftside += c.subs(psubs)*unknownvars[unknownmonoms.index(m)]
            else:
                rightside -= c.subs(psubs)*Poly(S.Zero,*coupledvars).add_term(S.One,m).as_basic()
        leftsideeqs.append(leftside)
        rightsideeqs.append(rightside)
    #reducedeqs2 = self.reduceBothSides(leftsideeqs,rightsideeqs,maxsymbols=32,usesymbols=False)
    Mtemp = Matrix(6,6,[S.Zero]*36)
    for ipeq,peq in enumerate(leftsideeqs):
        for imonom in range(len(unknownmonoms)):
            monom = [S.Zero]*len(unknownmonoms)
            monom[imonom] = 1
            Mtemp[ipeq,imonom] = peq.coeff(*monom)
    P,L,DD,U= self.LUdecompositionFF(Mtemp,*RightEquations[0].symbols[2:])
    finalnums = S.One
    finaldenoms = S.One
    for i in range(6):
        n,d = self.recursiveFraction(L[i,i]*U[i,i]/DD[i,i])
        finalnums *= n
        finaldenoms *= d
    q,r = div(finalnums,finaldenoms,RightEquations[0].symbols[2:])
    q=q.subs(leftvars[1]**2,1-leftvars[0]**2).expand()
    x=Symbol('x')
    soleq=self.solveHighDegreeEquationHalfAngle(q.subs([(leftvars[0],cos(x)),(leftvars[1],sin(x))]),x)
    coeffs = []
    for i in range(soleq.poly.degree,-1,-1):
        coeffs.append(soleq.poly.coeff(i))
    roots = mpmath.polyroots(coeffs)
    sols = [2*atan(root) for root in roots]
    return soleq

def solvefailed2(self):
    testeqs = [eq.as_basic().subs(psubs+invsubs) for eq in RightEquations]
    testeqs2=self.solveSingleVariableLinearly(testeqs,usedvars[0],usedvars[1:],maxnumeqs=10,douniquecheck=True)
    testeqs3 = [Poly(eq.subs(symbolsubs),*symbols[2:]) for eq in testeqs2]

    # choose which leftvar can determine the singularity of the following equations!
    testeqs4 = []
    for ipeq,peq in enumerate(testeqs3):
        maxdenom = [0]*(len(testeqs3[0].symbols)/2)
        for monoms in peq.iter_monoms():
            for i in range(len(maxdenom)):
                maxdenom[i] = max(maxdenom[i],monoms[2*i]+monoms[2*i+1])
        eqnew = S.Zero
        #if ipeq >= 4: maxdenom[-1] = 0;
        for c,monoms in peq.iter_terms():
            term = c
            for i in range(len(testeqs3[0].symbols)):
                num,denom = fraction(dummysubs[i][1])
                term *= num**monoms[i]
            # the denoms for 0,1 and 2,3 are the same
            for i in range(len(maxdenom)):
                denom = fraction(dummysubs[2*i][1])[1]
                term *= denom**(maxdenom[i]-monoms[2*i]-monoms[2*i+1])
            eqnew += simplify(term)
        testeqs4.append(Poly(eqnew,*dummys[0:2]))
    # testeqs4[0] has 81 monomials!

def solvefailed3(self):
    if len(leftsideeqs0) >= 2:
        # can solve linearly!
        p0 = leftsideeqs0[0]
        p1 = leftsideeqs0[1]
        M = Matrix(2,3,[p0.coeff(1,0,0,0),p0.coeff(0,1,0,0),p0.coeff(0,0,0,0)-rightsideeqs0[0].as_basic(),p1.coeff(1,0,0,0),p1.coeff(0,1,0,0),p1.coeff(0,0,0,0)-rightsideeqs0[1].as_basic()])
        partialsolution = [-M[1,1]*M[0,2]+M[0,1]*M[1,2],M[1,0]*M[0,2]-M[0,0]*M[1,2],M[0,0]*M[1,1]-M[0,1]*M[1,0]]
        for i in range(3):
            partialsolution[i] = partialsolution[i].expand()
        for i in range(len(leftsideeqs)):
            left = leftsideeqs[i]
            right = rightsideeqs[i]
            c = left.coeff(1,0,0,0)
            if c != S.Zero:
                left = Poly(partialsolution[2]*left.sub_term(c,(1,0,0,0)).as_basic(),*left.symbols)
                right = Poly(partialsolution[2]*right.as_basic() - c*partialsolution[0],*right.symbols)
            c = left.coeff(0,1,0,0)
            if c != S.Zero:
                left = Poly(partialsolution[2]*left.sub_term(c,(0,1,0,0)).as_basic(),*left.symbols)
                right = Poly(partialsolution[2]*right.as_basic() - c*partialsolution[1],*right.symbols)
            leftsideeqs[i] = left
            rightsideeqs[i] = right

        unknownmonoms = [(1, 0, 1, 0), (1, 0, 0, 1), (0, 1, 1, 0), (0, 1, 0, 1)]
        Mtemp = Matrix(4,4,[S.Zero]*16)
        for ipeq,peq in enumerate(leftsideeqs):
            for imonom in range(len(unknownmonoms)):
                monom = [S.Zero]*len(unknownmonoms)
                monom[imonom] = 1
                Mtemp[ipeq,imonom] = peq.coeff(*unknownmonoms[imonom])
        #P,L,DD,U= self.LUdecompositionFF(Mtemp,*Tee[0:12])

def simplifyPolynomial(self,peq,leftvar,symbolgen,localsymbols,localsymbols_reduced=None,localsymbols_mult=None,Tee=None):
    neweq = Poly(S.Zero,*peq.symbols)
    if Tee is not None:
        othervars = list(Tee[0:3,3])
        if leftvar is not None:
            othervars.append(leftvar)
    for c,m in peq.iter_terms():
        if c != S.Zero and not c.is_number:
            if Tee is not None:
                c = self.simplifyTransform(c,Tee,othervars=othervars).expand()
            if leftvar is None:
                leftpolyterms = [(c,None)]
            else:
                leftpoly = Poly(c,leftvar)
                leftpolyterms = [t for t in leftpoly.iter_terms()]
            neweq2=S.Zero
            for c2,m2 in leftpolyterms:
                if localsymbols_reduced is not None:
                    c2clean,c2common = self.removecommonexprs(c2,returncommon=True,onlynumbers=True)
                    index = self.getCommonExpression(localsymbols_reduced,c2clean)
                    if index is None:
                        index = self.getCommonExpression(localsymbols_reduced,-c2clean)
                        if index is not None:
                            c2common = -c2common
                    if index is not None:
                        v = localsymbols[index][0]*c2common/localsymbols_mult[index]
                else:
                    index = None
                if index is None:
                    v=symbolgen.next()
                    localsymbols.append((v,c2))
                    if localsymbols_reduced is not None:
                        localsymbols_reduced.append(c2clean)
                        localsymbols_mult.append(c2common)
                if m2 is None:
                    neweq2 += v
                else:
                    neweq2 += v * leftvar**m2[0]
            neweq = neweq.add_term(neweq2,m)
        else:
            neweq = neweq.add_term(c,m)
    return neweq

def solveDialytically2(self,reducedeqs,ileftvar):
    allmonoms = set()
    for peq in reducedeqs:
        for m in peq.iter_monoms():
            mlist = list(m)
            degree=mlist.pop(ileftvar)
            allmonoms.add(tuple(mlist))
            mlist[0] += 1
            allmonoms.add(tuple(mlist))
    allmonoms = list(allmonoms)
    allmonoms.sort()
    assert(len(allmonoms)<=2*len(reducedeqs))
    leftvar = reducedeqs[0].symbols[ileftvar]
    Mall = zeros((2*len(reducedeqs),len(allmonoms)))
    for i,peq in enumerate(reducedeqs):
        for c,m in peq.iter_terms():
            mlist = list(m)
            degree = mlist.pop(ileftvar)
            c = c*leftvar**degree
            Mall[len(reducedeqs)+i,allmonoms.index(tuple(mlist))] += c
            mlist[0] += 1
            Mall[i,allmonoms.index(tuple(mlist))] += c

    Mconst = Matrix(len(newreducedeqs),len(allmonoms),[S.Zero]*len(newreducedeqs)*len(allmonoms))
    for i in range(len(newreducedeqs)):
        for j in range(len(allmonoms)):
            Mconst[i,j] = Poly(Mall[i,j],leftvar).coeff(2)

    alpha=Symbol('alpha')
    beta=Symbol('beta')
    x=[Symbol('x%d'%i) for i in range(11)]
    ns0=[Symbol('ns0_%d'%i) for i in range(11)]
    ns1=[Symbol('ns1_%d'%i) for i in range(11)]
    final = [x[i]+ns0[i]*alpha+ns1[i]*beta for i in range(11)]
    nexteqs = [final[0]*final[0]-final[1],final[0]*final[2]-final[3],final[1]*final[2]-final[4],final[2]*final[2]-final[5],final[0]*final[5]-final[6],final[1]*final[5]-final[7],final[2]*final[5]-final[8],final[0]*final[8]-final[9],final[1]*final[8]-final[10]]
    polyeqs = [Poly(eq,alpha,beta) for eq in nexteqs]            
    newmonoms = [(2,0),(1,1),(0,2),(1,0),(0,1)]
    Mnew = Matrix(len(polyeqs),len(newmonoms),[S.Zero]*(len(polyeqs)*len(newmonoms)))
    Mconst = Matrix(len(polyeqs),1,[S.Zero]*len(polyeqs))
    for i in range(len(polyeqs)):
        Mconst[i] = -polyeqs[i].coeff(0,0)
        for j in range(len(newmonoms)):
            Mnew[i,j] = polyeqs[i].coeff(*newmonoms[j])

    # should check if determinant vanishes for all valuespolynomial 
    # set determinant of M = 0 and solve for leftvar
    #characteristic_poly = Mall.det_bareis()
    raise self.CannotSolveError('dialytic solution not implemented')

def solveDialytically3(self,reducedeqs,ileftvar):
    allmonoms = set()
    for peq in reducedeqs:
        for m in peq.iter_monoms():
            mlist = list(m)
            degree=mlist.pop(ileftvar)
            allmonoms.add(tuple(mlist))
            mlist[0] += 1
            allmonoms.add(tuple(mlist))
    allmonoms = list(allmonoms)
    allmonoms.sort()
    assert(len(allmonoms)<=2*len(reducedeqs))
    leftvar = reducedeqs[0].symbols[ileftvar]
    A = [zeros((2*len(reducedeqs),len(allmonoms))),zeros((2*len(reducedeqs),len(allmonoms))),zeros((2*len(reducedeqs),len(allmonoms)))]
    for i,peq in enumerate(reducedeqs):
        for c,m in peq.iter_terms():
            mlist = list(m)
            degree = mlist.pop(ileftvar)
            A[degree][len(reducedeqs)+i,allmonoms.index(tuple(mlist))] += c
            mlist[0] += 1
            A[degree][i,allmonoms.index(tuple(mlist))] += c

    Av = [A[i].subs(psubs).evalf() for i in range(3)]
    M = zeros((24,24))
    M[0:12,12:24] = eye(12)
    M[12:24,0:12] = -Av[2].inv()*Av[0]
    M[12:24,12:24] = -Av[2].inv()*Av[1]
    w,v = linalg.eig(numpy.array(numpy.array(M),float64))
    index = w[12]
    v[:,12]

    lf = [1.0,-1.0,1.0,1.0]
    Av = [lf[1]*lf[1]*Av[2] + lf[1]*lf[3]*Av[1] + lf[3]*lf[3]*Av[0],
             2*lf[0]*lf[1]*Av[2] + (lf[0]*lf[3]+lf[1]*lf[2])*Av[1] + 2*lf[2]*lf[3]*Av[0],
             lf[0]*lf[0]*Av[2]+lf[0]*lf[2]*Av[1]+lf[2]*lf[2]*Av[0]]

def computeDixonResultant(self,orgeqs):
    """Computes the dixon resultant of the polynomial equations and find a non-singular sub-matrix such that its determinant can be used to get polynomial in one of the variables.
    See:
    Deepak Kapur, Tushar Saxena, and Lu Yang. "Algebraic and geometric reasoning using Dixon resultants". ISSAC '94 Proceedings of the international symposium on Symbolic and algebraic computation .
    """
    allmonoms = set()
    orgsymbols = orgeqs[0].symbols
    dixsymbols = [Symbol('d_%s'%s.name) for s in orgsymbols]
    dixmaxmonoms = [0]*len(orgsymbols)
    for peq in orgeqs:
        allmonoms = allmonoms.union(set(peq.monoms))
        for m in peq.iter_monoms():
            for j in range(len(dixmaxmonoms)):
                dixmaxmonoms[j] = max(dixmaxmonoms[j],m[j])
    allmonoms = list(allmonoms)
    allmonoms.sort()
    allmonoms.reverse() # this should put (0) at the end
    assert(len(orgsymbols) < len(orgeqs))
    neweqs = []
    dixonsymbolgen = symbolgen = cse_main.numbered_symbols('dconst')
    dixonsymbols = []
    for eq in orgeqs:
        neweq = Poly(S.Zero,*orgsymbols)
        for c,m in eq.iter_terms():
            v = symbolgen.next()
            dixonsymbols.append((v,c))
            neweq = neweq.add_term(v,m)
        neweqs.append(neweq)
    M = Matrix(len(orgsymbols)+1,len(orgsymbols)+1,[S.Zero]*((len(orgsymbols)+1)**2))
    for isymbol in range(len(orgsymbols)+1):
        subs = [(orgsymbols[i],dixsymbols[i]) for i in range(0,isymbol)]
        for i in range(len(orgsymbols)+1):
            M[isymbol,i] = neweqs[i].as_basic().subs(subs)
    allsymbols = list(dixsymbols)+list(orgsymbols)
    det = M.det_bareis()
    polydixon_raw = Poly(det,*dixsymbols)
    quotient = S.One
    for sym,dsym in izip(orgsymbols,dixsymbols):
        quotient *= (sym-dsym)
    polydixon,r = div(polydixon_raw,quotient)
    polydixon = Poly(polydixon,*dixsymbols)
    newmonoms = set()
    for c,m in polydixon.iter_terms():
        p = Poly(c,*orgsymbols)
        newmonoms = newmonoms.union(set(p.monoms))
    newmonoms = list(newmonoms)
    newmonoms.sort()
    newmonoms.reverse() # this should put (0) at the end

    dixonsymbolsvalues = [(s,v.subs(valsubs+psubs+localsymbolsvalues)) for s,v in dixonsymbols]
    Mdixon = Matrix(len(polydixon.monoms),len(newmonoms),[S.Zero]*(len(polydixon.monoms)*len(newmonoms)))
    i = 0
    for c,m in polydixon.iter_terms():
        p = Poly(c,*orgsymbols)
        for c2,m2 in p.iter_terms():
            Mdixon[i,newmonoms.index(m2)] = c2.subs(dixonsymbolsvalues).expand()
        i += 1

    s=linalg.svd(Mdixon.subs(leftvar,0),compute_uv=0)
    rank = numpy.sum(numpy.greater(abs(s),1e-14))
    Mdettest = Matrix(Mdixon.shape[0],Mdixon.shape[1]-1,[S.Zero]*(Mdixon.shape[0]*(Mdixon.shape[1]-1)))
    for j in range(Mdixon.shape[1]):
        Mtemp = Mnew.subs(leftvar,0)
        Mtemp.col_del(j)
        s=linalg.svd(Mtemp,compute_uv=0)
        if numpy.sum(numpy.greater(abs(s),1e-14)) < rank:
            print j
    Mvalues = Mdixon.subs(localsymbolsvalues)
    for i in range(Mvalues.shape[0]):
        for j in range(Mvalues.shape[1]):
            Mvalues[i,j] = Mvalues[i,j].expand()
    dfinal = Mdixon.det_bareis()
    solutionvector = Matrix(len(newmonoms),1,[Poly(S.Zero,*orgsymbols).add_term(S.One,m).subs(allsubs) for m in newmonoms])

def ComputeMatrix(self,eqs,allsubs,symbols=None):
    if symbols is None:
        symbols = eqs[0].symbols
    unknownmonoms = list(Poly(eqs[0],*symbols).monoms)
    unknownmonoms.sort()
    unknownmonoms.pop(0) # remove 0,0
    unknownvars = [Symbol('x%d'%i) for i in range(len(unknownmonoms))]
    Mtemp = Matrix(len(eqs),len(unknownmonoms),[S.Zero]*(len(eqs)*len(unknownmonoms)))
    for ipeq,peq in enumerate(eqs):
        peq = Poly(peq,*symbols)
        for im,m in enumerate(unknownmonoms):
            Mtemp[ipeq,im] = peq.coeff(*m).subs(allsubs)
    return Mtemp

def ComputeDeterminant(self,eqs,allsubs,symbols=None):
    Mtemp = self.ComputeMatrix(eqs,allsubs,symbols)
    Mtemp2 = Mtemp * Mtemp.transpose()
    return Mtemp2.det()

def characteristic_poly(self,eqs):
    #eqs=[eq.subs(psubs) for eq in reducedeqs[0:5]]
    x=eqs[0].symbols[0]
    ipower = 0
    remainders = []
    while True:
        print ipower
        changed = True
        f = Poly(x**ipower,*eqs[0].symbols)
        while changed:
            changed = False
            for eq in eqs:
                q,r = div(f,eq)
                if q != S.Zero:
                    print q
                    changed = True
                f = Poly(f,*eqs[0].symbols)
        print f
        remainders.append(f)
        ipower += 1

    def using_solvedetdialyticpoly12(self):
        name = 'solvedetdialyticpoly12'
        checkconsistency12=self.using_checkconsistency12()
        polyroots2=self.using_polyroots(2)
        if not name in self.functions:
            fcode = """
/// \\brief Solves a polynomial given its evaluation is the determinant of a matrix
///
/// matcoeffs is a 3*9*6 = 162 length vector
/// every 18 coeffs describe one equation in the following order:
/// [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2), (2, 0), (2, 1), (2, 2)]
/// let A have 
static inline void %s(const IKReal* matcoeffs, IKReal* rawroots, int& numroots)
{
    using std::complex;
    const IKReal tol = 128.0*std::numeric_limits<IKReal>::epsilon();
    const int maxsteps = 100;
    const int D = 24;
    const int matrixdim = 12;
    complex<IKReal> roots[D];
    complex<IKReal> IKFAST_ALIGNED16(A[matrixdim*matrixdim]);
    // can do this since A and Areal/Breal are used at different times
    IKReal* Areal = (IKReal*)&A;
    IKReal* Breal = &Areal[matrixdim*matrixdim];
    int ipiv[matrixdim]={0};
    numroots = 0;
    IKReal err[D];
    roots[0] = complex<IKReal>(1,0);
    roots[1] = complex<IKReal>(0.4,0.9); // any complex number not a root of unity is works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < D; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    int info, coeffindex;
    IKReal polymultiplier=1; // should be 1/lead coeff. Needed or otherwise convergence will be slow
    {     
        coeffindex = 0;
        for(int j = 0; j < 6; ++j) {
            for(int k = 0; k < 9; ++k) {
                Areal[j*matrixdim+k+3] = matcoeffs[coeffindex+2];
                Areal[(j+6)*matrixdim+k] = Areal[j*matrixdim+k+3];
                coeffindex += 3;
            }
            // fill the rest with 0s!
            for(int k = 0; k < 3; ++k) {
                Areal[j*matrixdim+k] = Areal[(j+6)*matrixdim+k+9] = 0;
            }
        }
        dgetrf_ (&matrixdim, &matrixdim, Areal, &matrixdim, &ipiv[0], &info);
        if( info != 0 ) {
            return; // failed
        }
        polymultiplier = ipiv[0] != 1 ? -Areal[0] : Areal[0];
        for(int j = 1; j < matrixdim; ++j) {
            polymultiplier *= Areal[j*matrixdim+j];
            if (ipiv[j] != (j+1)) {
                polymultiplier = -polymultiplier;
            }
        }
        if( isnan(polymultiplier) || polymultiplier == 0 ) {
            return;
        }
        polymultiplier = 1/polymultiplier;
    }
    int step;
    for(step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < D; ++i) {
            if ( err[i] >= tol && !isinf(real(roots[i])) && !isinf(imag(roots[i])) ) {
                changed = true;
                // evaluate the determinant
                complex<IKReal> x = roots[i], x2 = roots[i]*roots[i];
                coeffindex = 0;
                for(int j = 0; j < 6; ++j) {
                    for(int k = 0; k < 9; ++k) {
                        A[j*matrixdim+k+3] = matcoeffs[coeffindex]+matcoeffs[coeffindex+1]*x+matcoeffs[coeffindex+2]*x2;
                        A[(j+6)*matrixdim+k] = A[j*matrixdim+k+3];
                        coeffindex += 3;
                    }
                    // fill the rest with 0s!
                    A[j*matrixdim+0] = A[(j+6)*matrixdim+0+9] = 0;
                    A[j*matrixdim+1] = A[(j+6)*matrixdim+1+9] = 0;
                    A[j*matrixdim+2] = A[(j+6)*matrixdim+2+9] = 0;
                }
                zgetrf_ (&matrixdim, &matrixdim, A, &matrixdim, &ipiv[0], &info);
                if( info != 0 ) {
                    continue; // failed
                }
                complex<IKReal> det = ipiv[0] != 1 ? -A[0] : A[0];
                for(int j = 1; j < matrixdim; ++j) {
                    det *= A[j*matrixdim+j];
                    if (ipiv[j] != (j+1)) {
                        det = -det;
                    }
                }
                if( isnan(real(det)) ) {
                    continue; // failed;
                }
                det *= polymultiplier;
                // have to divide by (1+roots[i]^2)^4 to get a 16th degree polynomial (actually this is not always the case!)
                //complex<IKReal> denom = complex<IKReal>(1,0)+x2;
                //denom *= denom;
                //denom *= denom;
                //det /= denom;
                for(int j = 0; j < D; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] && !isinf(real(roots[j])) ) {
                            det /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= det;
                err[i] = abs(det);
            }
        }
        if( !changed ) {
            break;
        }
    }
    numroots = 0;
    for(int i = 0; i < D; ++i) {
        if( IKabs(imag(roots[i])) < 100*tol && IKabs(err[i]) < 100*tol ) {
            // found a real solution!, now solve the linear system
            IKReal curroot = real(roots[i]);
            IKReal curroot2 = curroot*curroot;
            coeffindex = 0;
            for(int j = 0; j < 6; ++j) {
                for(int k = 0; k < 9; ++k) {
                    IKReal x = matcoeffs[coeffindex]+matcoeffs[coeffindex+1]*curroot+matcoeffs[coeffindex+2]*curroot2;
                    Areal[j+matrixdim*(k+3-1)] = x;
                    if( k == 0 ) {
                        Breal[j+6] = -x;
                    }
                    else {
                        Areal[j+6+matrixdim*(k-1)] = x;
                    }
                    coeffindex += 3;
                }
                // fill the rest with 0s!
                Breal[j] = 0;
                Areal[j] = Areal[j+matrixdim*1] = 0;
                Areal[(j+6)+matrixdim*8] = Areal[(j+6)+matrixdim*9] = Areal[(j+6)+matrixdim*10] = 0;
            }
            // perform LU decomposition to solve for null space and solution simultaneously
            int n = matrixdim-1, nrhs=1;
            dgetrf_(&matrixdim, &n, Areal, &matrixdim, &ipiv[0], &info);
            if( info != 0 ) {
                continue;
            }
            dgetrs_("No transpose", &n, &nrhs, Areal, &matrixdim, &ipiv[0], Breal, &matrixdim, &info);
            if( info != 0 ) {
                continue;
            }
            if(info == 0) {
                // because Areal might have a null space, have to solve for it
                int nullspacedim = 0;
                for(int j = 0; j < matrixdim-1; ++j) {
                    if( IKabs(Areal[j+matrixdim*j]) < 1000*tol ) {
                        nullspacedim++;
                    }
                }
                if( nullspacedim >= 2 ) {
                    // have two nullspace vectors!
                    IKReal ns[2][matrixdim-1];
                    for(int ins = 0; ins < 2; ++ins) {
                        IKReal nsnum=1;
                        for(int j = matrixdim-2; j >= 0; --j) {
                            if( IKabs(Areal[j+matrixdim*j]) < 1000*tol ) {
                                ns[ins][j] = nsnum;
                                if( ins > 0 ) {
                                    nsnum += 1;
                                }
                            }
                            else {
                                IKReal accum = 0;
                                for(int k = j+1; k < matrixdim-1; ++k) {
                                    accum += ns[ins][k] * Areal[j+matrixdim*k];
                                }
                                ns[ins][j] = -accum/Areal[j+matrixdim*j];
                            }
                        }
                    }
                    // have to solve this with another least squares....
                    // [ns0_0**2,       2*ns1_0*ns0_0,             ns1_0**2,    -ns0_1 + 2*ns0_0*x0,          -ns1_1 + 2*ns1_0*x0]              [x1 - x0**2]
                    // [ns0_0*ns0_2,    ns1_0*ns0_2 + ns1_2*ns0_0, ns1_0*ns1_2, -ns0_3 + ns0_0*x2 + ns0_2*x0, -ns1_3 + ns1_0*x2 + ns1_2*x0]     [x3 - x0*x2]
                    // [ns0_1*ns0_2,    ns1_1*ns0_2 + ns1_2*ns0_1, ns1_1*ns1_2, -ns0_4 + ns0_1*x2 + ns0_2*x1, -ns1_4 + ns1_1*x2 + ns1_2*x1] X = [x4 - x1*x2]
                    // [ns0_2**2,       2*ns1_2*ns0_2,             ns1_2**2,    -ns0_5 + 2*ns0_2*x2,          -ns1_5 + 2*ns1_2*x2]              [x5 - x2**2]
                    // [ns0_0*ns0_5,    ns1_0*ns0_5 + ns1_5*ns0_0, ns1_0*ns1_5, -ns0_6 + ns0_0*x5 + ns0_5*x0, -ns1_6 + ns1_0*x5 + ns1_5*x0]     [x6 - x0*x5]
                    Areal[0] = ns[0][0]*ns[0][0];
                    Areal[1] = ns[0][0]*ns[0][2];
                    Areal[2] = ns[0][1]*ns[0][2];
                    Areal[3] = ns[0][2]*ns[0][2];
                    Areal[4] = ns[0][0]*ns[0][5];
                    Areal[5] = 2*ns[1][0]*ns[0][0];
                    Areal[6] = ns[1][0]*ns[0][2] + ns[1][2]*ns[0][0];
                    Areal[7] = ns[1][1]*ns[0][2] + ns[1][2]*ns[0][1];
                    Areal[8] = 2*ns[1][2]*ns[0][2];
                    Areal[9] = ns[1][0]*ns[0][5] + ns[1][5]*ns[0][0];
                    Areal[10] = ns[1][0]*ns[1][0];
                    Areal[11] = ns[1][0]*ns[1][2];
                    Areal[12] = ns[1][1]*ns[1][2];
                    Areal[13] = ns[1][2]*ns[1][2];
                    Areal[14] = ns[1][0]*ns[1][5];
                    Areal[15] = -ns[0][1] + 2*ns[0][0]*Breal[0];
                    Areal[16] = -ns[0][3] + ns[0][0]*Breal[2] + ns[0][2]*Breal[0];
                    Areal[17] = -ns[0][4] + ns[0][1]*Breal[2] + ns[0][2]*Breal[1];
                    Areal[18] = -ns[0][5] + 2*ns[0][2]*Breal[2];
                    Areal[19] = -ns[0][6] + ns[0][0]*Breal[5] + ns[0][5]*Breal[0];
                    Areal[20] = -ns[1][1] + 2*ns[1][0]+Breal[0];
                    Areal[21] = -ns[1][3] + ns[1][0]*Breal[2] + ns[1][2]*Breal[0];
                    Areal[22] = -ns[1][4] + ns[1][1]*Breal[2] + ns[1][2]*Breal[1];
                    Areal[23] = -ns[1][5] + 2*ns[1][2]*Breal[2];
                    Areal[24] = -ns[1][6] + ns[1][0]*Breal[5] + ns[1][5]*Breal[0];
                    int startindex = 25;
                    Areal[startindex] = Breal[1]-Breal[0]*Breal[0];
                    Areal[startindex+1] = Breal[3]-Breal[0]*Breal[2];
                    Areal[startindex+2] = Breal[4]-Breal[1]*Breal[2];
                    Areal[startindex+3] = Breal[5]-Breal[2]*Breal[2];
                    Areal[startindex+4] = Breal[6]-Breal[0]*Breal[5];
                    int nn=5;
                    dgesv_(&nn, &nrhs, Areal, &nn, ipiv, &Areal[startindex], &nn, &info);
                    if( info == 0 ) {
                        if( 1 ) {//IKabs(Areal[startindex]-Areal[startindex+3]*Areal[startindex+3]) < 1000*tol && IKabs(Areal[startindex+2]-Areal[startindex+4]*Areal[startindex+4]) < 1000*tol ) {
                            for(int k = 0; k < matrixdim-1; ++k) {
                                Breal[k] += Areal[startindex+3]*ns[0][k]+Areal[startindex+4]*ns[1][k];
                            }
                            if( %s(Breal) ) {
                                rawroots[numroots++] = curroot;
                                rawroots[numroots++] = Breal[2];
                                rawroots[numroots++] = Breal[0];
                            }
                        }
                    }
                }
                else if( nullspacedim == 1 ) {
                    // solve an angle with quadratic equation
                    IKReal nullspace[matrixdim-1];
                    for(int j = matrixdim-2; j >= 0; --j) {
                        if( IKabs(Areal[j+matrixdim*j]) < 1000*tol ) {
                            nullspace[j] = 1;
                        }
                        else {
                            IKReal accum = 0;
                            for(int k = j+1; k < matrixdim-1; ++k) {
                                accum += nullspace[k] * Areal[j+matrixdim*k];
                            }
                            nullspace[j] = -accum/Areal[j+matrixdim*j];
                        }
                    }
                    // take the biggest abs value between [0],[1] and [2],[5]
                    IKReal f0 = IKabs(nullspace[0])+IKabs(nullspace[1]);
                    IKReal f1 = IKabs(nullspace[2])+IKabs(nullspace[5]);
                    int nsnumroots;
                    IKReal nsroots[2], nscoeffs[3];
                    if( f0 == 0 && f1 == 0 ) {
                        if( %s(Breal) ) {
                            rawroots[numroots++] = curroot;
                            rawroots[numroots++] = Breal[2];
                            rawroots[numroots++] = Breal[0];
                        }
                    }
                    else {
                        if( f0 > f1 ) {
                            nscoeffs[0] = nullspace[0]*nullspace[0];
                            nscoeffs[1] = 2*nullspace[0]*Breal[0]-nullspace[1];
                            nscoeffs[2] = Breal[0]*Breal[0]-Breal[1];
                        }
                        else if( f1 > 0 ) {
                            nscoeffs[0] = nullspace[2]*nullspace[2];
                            nscoeffs[1] = 2*nullspace[2]*Breal[2]-nullspace[5];
                            nscoeffs[2] = Breal[2]*Breal[2]-Breal[5];
                        }
                        %s(nscoeffs,nsroots,nsnumroots);
                        nsroots[1] -= nsroots[0];
                        for(int j = 0; j < nsnumroots; ++j) {
                            for(int k = 0; k < matrixdim-1; ++k) {
                                Breal[k] += nsroots[j]*nullspace[k];
                            }
                            if( %s(Breal) ) {
                                rawroots[numroots++] = curroot;
                                rawroots[numroots++] = Breal[2];
                                rawroots[numroots++] = Breal[0];
                            }
                        }
                    }
                }
                else {
                    if( %s(Breal) ) {
                        rawroots[numroots++] = curroot;
                        rawroots[numroots++] = Breal[2];
                        rawroots[numroots++] = Breal[0];
                    }
                }
            }
        }
    }
}
"""%(name,checkconsistency12,checkconsistency12,polyroots2,checkconsistency12,checkconsistency12)
            self.functions[name] = fcode
        return name

def test_ik():
    from sympy import *
    import __builtin__
    from openravepy.ikfast import SolverStoreSolution, SolverSolution, combinations, SolverSequence, fmod, SolverRotation, SolverIKChainTransform6D, SolverBranchConds, SolverMatrixInverse, SolverCoeffFunction, SolverCheckZeros
    ikmodel=self
    self = solver
    
    chaintree = solver.generateIkSolver(baselink=baselink,eelink=eelink,freejointinds=freejointinds,solvefn=solvefn)
    code=ikfast_generator_cpp.CodeGenerator().generate(chaintree)
    open(sourcefilename,'w').write(code)

    # get values
    possibleangles = [S.Zero, pi.evalf()/2, asin(3.0/5).evalf(), asin(4.0/5).evalf(), asin(5.0/13).evalf(), asin(12.0/13).evalf()]
    jointvalues = [S.Zero]*len(jointvars)
    jointvalues[0] = possibleangles[2]
    jointvalues[1] = possibleangles[3]
    jointvalues[2] = possibleangles[2]
    jointvalues[3] = possibleangles[3]
    jointvalues[4] = possibleangles[3]
    valsubs = []
    for var,value in izip(jointvars,jointvalues):
        valsubs += [(var,value),(Symbol('c%s'%var.name),self.convertRealToRational(cos(value).evalf())),(Symbol('s%s'%var.name),self.convertRealToRational(sin(value).evalf()))]
    psubs = []
    for i in range(12):
        psubs.append((self.Tee[i],self.convertRealToRational(Tfinal[i].subs(valsubs).evalf())))
    for s,v in self.ppsubs+self.npxyzsubs+self.rxpsubs:
        psubs.append((s,v.subs(psubs)))
    if len(raghavansolutiontree) > 0:
        psubs += [(s,v.subs(psubs)) for s,v in raghavansolutiontree[0].subs]
    dummyvaluesubs = [(dvar,self.convertRealToRational(var.subs(valsubs).evalf())) for dvar,var in dummyvars]
    allsubs = valsubs+psubs+dummyvaluesubs
    localsymbolsvalues = [(var,value.subs(valsubs+psubs)) for var,value in localsymbols]
    #localsymbolsvalues = []
    #for i in range(len(localsymbols)):
    #    localsymbolsvalues.append((localsymbols[i][0],localsymbols[i][1].subs(localsymbolsvalues+psubs).evalf()))

    Mall.subs(localsymbolsvalues+[(leftvar,1.0)]).det()

    correctsols = [Poly(S.Zero,*newreducedeqs[0].symbols).add_term(S.One,m).subs(allsubs).evalf() for m in allmonoms]
    correctsols = Matrix(len(allmonoms),1,[Poly(S.Zero,*newreducedeqs[0].symbols).add_term(S.One,m).subs(allsubs).evalf() for m in allmonoms])
        
    Mvalues = Mall.subs(localsymbolsvalues)
    for i in range(size(Mvalues)):
        Mvalues[i] = self.chop(Mvalues[i])

    Mleadcoeff.subs(localsymbolsvalues).det()

    leftvarsubs = [(leftvar,tan(0.5).evalf())]
    Mnext = Mall.subs(localsymbolsvalues+leftvarsubs)
    A = Mnext[1:,1:]
    b = -Mnext[1:,0]
    x=numpy.linalg.solve(A,b)

    str([eq.subs(localsymbols).subs(allsubs).evalf() for eq in exportcoeffeqs])

    Mleadcoeff = Matrix(12,12,[S.Zero]*144)
    coeffindex = 0
    for j in range(6):
        for k in range(9):
            Mleadcoeff[j,k+3] = exportcoeffeqs[coeffindex+2].subs(localsymbolsvalues)
            Mleadcoeff[(j+6),k] = Mleadcoeff[j,k+3]
            coeffindex += 3
    Mleadcoeff.det()

    pickle.dump([symbolgen.next(),localsymbols,localsymbols_reduced,localsymbols_mult,newreducedeqs,reducedeqs,RightEquations,dummys,dummysubs,dummyvars],open('eqs.pp','w'))
    symbolgenconst,localsymbols,localsymbols_reduced,localsymbols_mult,newreducedeqs,reducedeqs,RightEquations,dummys,dummysubs,dummyvars = pickle.load(open('eqs.pp','r'))
    symbolgen = cse_main.numbered_symbols('const',start=int(symbolgenconst.name[5:]))


"""
ikfast notes;

pr2 with conic sections:
success rate: 0.955000, wrong solutions: 0.000000, no solutions: 0.145000, missing solution: 0.281000
mean: 0.000006s, median: 0.000007s, min: 0.000001s, max: 0.000015s

pr2 with half-angle transformation:
success rate: 0.993000, wrong solutions: 0.000000, no solutions: 0.011000, missing solution: 0.081000
mean: 0.000009s, median: 0.000009s, min: 0.000005s, max: 0.000016s
"""
