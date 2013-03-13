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

def computeDixonResultant(self,polyeqs,othersolvedvars):
    """Computes the dixon resultant of the polynomial equations and find a non-singular sub-matrix such that its determinant can be used to get polynomial in one of the variables.
    See:
    Deepak Kapur, Tushar Saxena, and Lu Yang. "Algebraic and geometric reasoning using Dixon resultants". ISSAC '94 Proceedings of the international symposium on Symbolic and algebraic computation .
    """
    allmonoms = set()
    orgsymbols = polyeqs[0].symbols
    dixsymbols = [Symbol('d_%s'%s.name) for s in orgsymbols]
    dixmaxmonoms = [0]*len(orgsymbols)
    for peq in polyeqs:
        allmonoms = allmonoms.union(set(peq.monoms))
        for m in peq.iter_monoms():
            for j in range(len(dixmaxmonoms)):
                dixmaxmonoms[j] = max(dixmaxmonoms[j],m[j])
    allmonoms = list(allmonoms)
    allmonoms.sort()
    allmonoms.reverse() # this should put (0) at the end
    assert(len(orgsymbols) < len(polyeqs))
    neweqs = []
    dixonsymbolgen = symbolgen = cse_main.numbered_symbols('dconst')
    dixonsymbols = []
    for eq in polyeqs:
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
    localvars = list(orgsymbols)+list(dixsymbols)+[s for s,v in dixonsymbols]
    det = self.det_bareis(M,*localvars)
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

    dixonsymbolsvalues = [(s,v.subs(newvalsubs+psubs)) for s,v in dixonsymbols]
    Mdixon = Matrix(len(polydixon.monoms),len(newmonoms),[S.Zero]*(len(polydixon.monoms)*len(newmonoms)))
    i = 0
    for c,m in polydixon.iter_terms():
        p = Poly(c,*orgsymbols)
        for c2,m2 in p.iter_terms():
            Mdixon[i,newmonoms.index(m2)] = c2.subs(dixonsymbols).expand()
        i += 1

    localvars2 = [var for var in self.pvars if self.has_any_symbols(Mdixon,var)]
    for othersolvedvar in othersolvedvars:
        for var in self.Variable(othersolvedvar).vars:
            if self.has_any_symbols(Mdixon,var):
                localvars2.append(var)
    dfinal = self.det_bareis(Mdixon,*localvars2)

    s=linalg.svd(Mdixon.subs(leftvar,0),compute_uv=0)
    rank = numpy.sum(numpy.greater(abs(s),1e-14))
    Mdettest = Matrix(Mdixon.shape[0],Mdixon.shape[1]-1,[S.Zero]*(Mdixon.shape[0]*(Mdixon.shape[1]-1)))
    for j in range(Mdixon.shape[1]):
        Mtemp = Mdixon.subs(leftvar,0)
        Mtemp.col_del(j)
        s=linalg.svd(Mtemp,compute_uv=0)
        if numpy.sum(numpy.greater(abs(s),1e-14)) < rank:
            print j
    Mvalues = Mdixon.subs(dixonsymbolsvalues)
    for i in range(Mvalues.shape[0]):
        for j in range(Mvalues.shape[1]):
            Mvalues[i,j] = Mvalues[i,j].expand()
    dfinal = Mdixon.det_bareis()
    solutionvector = Matrix(len(newmonoms),1,[Poly(S.Zero,*orgsymbols).add_term(S.One,m) for m in newmonoms])

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

def solveFailed(self):
    for c in C:
        reducedeqs.append(Poly(simplify(c.subs(htvarsubs)*(1+htvar[0]**2)*(1+htvar[1]**2)),htvars[0],htvars[1],tvar))
    x = Symbol('ht%s'%othersymbols[0].name[1:])
    dummyeq = eq.coeff(0,0)*(1+x**2) + eq.coeff(1,0)*(1-x**2) + eq.coeff(0,1)*2*x
    eq,symbolsubs = self.removeConstants(dummyeq,[x],symbolgen)

    # create a new matrix using the coefficients of reducedeqs
    newmonoms = set()
    origmonoms = set()
    maxdegree = 0
    for peq in reducedeqs:
        for m in peq.iter_monoms():
            mlist = list(m)
            newmonoms.add(tuple(mlist))
            origmonoms.add(tuple(mlist))
            mlist[0] += 1
            newmonoms.add(tuple(mlist))
    newmonoms = list(newmonoms)
    newmonoms.sort()
    origmonoms = list(origmonoms)
    origmonoms.sort()
    assert(len(newmonoms)<=2*len(reducedeqs))
    symbolgen = cse_main.numbered_symbols('const')
    localsymbols = []
    localexprs = []
    localcommon = []
    M = zeros((2*len(reducedeqs),len(newmonoms)))
    exportcoeffeqs = [S.Zero]*(len(reducedeqs)*len(newmonoms)*3)
    x = Symbol('ht%s'%othersymbols[0].name[1:])
    for ipeq,peq in enumerate(reducedeqs):
        for c,m in peq.iter_terms():
            #eq,symbolsubs = self.removeConstants(c,othersymbols[0:2],symbolgen)
            eq = Poly(c,othersymbols[0],othersymbols[1])
            assert(eq.degree<=1)
            dummyeq = eq.coeff(0,0)*(1+x**2) + eq.coeff(1,0)*(1-x**2) + eq.coeff(0,1)*2*x
            eq,symbolsubs = self.removeConstants(dummyeq,[x],symbolgen)
            for s,expr in symbolsubs:
                expr0,common0 = self.removecommonexprs(expr,returncommon=True)
                index = self.getCommonExpression(localexprs,expr0)
                if index is not None:
                    eq=eq.subs(s,localsymbols[index][0]/localcommon[index]*common0)
                else:
                    index = self.getCommonExpression(localexprs,-expr0)
                    if index is not None:
                        eq=eq.subs(s,-localsymbols[index][0]/localcommon[index]*common0)
                    else:
                        localsymbols.append((s,expr))
                        localexprs.append(expr0)
                        localcommon.append(common0)
            #eq = Poly(eq,othersymbols[0],othersymbols[1])
            exportindex = len(newmonoms)*ipeq+newmonoms.index(m)
            #exportcoeffeqs[exportindex] = eq.coeff(0,0)
            #exportcoeffeqs[len(newmonoms)*len(reducedeqs)+exportindex] = eq.coeff(1,0)
            #exportcoeffeqs[2*len(newmonoms)*len(reducedeqs)+exportindex] = eq.coeff(0,1)
            M[ipeq+len(reducedeqs),newmonoms.index(m)] = eq.as_basic()
            mlist = list(m)
            mlist[0] += 1
            M[ipeq,newmonoms.index(tuple(mlist))] = eq.as_basic()

    Mpowers = [zeros(M.shape)]
    for i in range(M.shape[0]):
        for j in range(M.shape[0]):
            Mpowers[0][i,j] = Poly(M[i,j],x)
        Mpowers[0][i,i] += S.One
    multcombs = [(0,0),(0,1),(1,1),(0,3),(1,3),(2,3),(3,3)]
    for indices in multcombs:
        print indices
        Mnew = Mpowers[indices[0]]*Mpowers[indices[1]]
        for i in range(M.shape[0]):
            for j in range(M.shape[0]):
                eq,symbolsubs = self.removeConstants(Mnew[i,j],[x],symbolgen)
                for s,expr in symbolsubs:
                    localsymbols.append((s,expr))
                    localexprs.append(expr)
                    localcommon.append(S.One)
                Mnew[i,j] = eq
        Mpowers.append(Mnew)
    # have M.shape[0] unknowns with constant term being 1
    characteristiccoeffs = [Symbol('dummyc%d'%i) for i in range(M.shape[0]+1)]
    characteristicpolys = []
    for i in range(M.shape[0]):
        for j in range(M.shape[0]):
            print i,j
            p = Poly(characteristiccoeffs[0],x,*characteristiccoeffs)
            for k in range(M.shape[0]):
                p = p + characteristiccoeffs[k+1]*Mpowers[k][i,j].as_basic()
            characteristicpolys.append(p)

    allmonoms = set()
    for peq in characteristicpolys:
        allmonoms = allmonoms.union(set(peq.monoms))
    allmonoms = list(allmonoms)
    allmonoms.sort()


    localsymbolsvalues = []
    for i in range(len(localsymbols)):
        localsymbolsvalues.append((localsymbols[i][0],localsymbols[i][1].subs(localsymbolsvalues+psubs).evalf()))

    Msub = [zeros(M.shape),zeros(M.shape),zeros(M.shape)]
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            peq = Poly(M[i,j],x)
            for k in range(peq.degree+1):
                Msub[k][i,j] = peq.coeff(k)

    #allsymbols = self.pvars+[s for s,v in localsymbols]+[x]
    #P,L,DD,U= self.LUdecompositionFF(M,*allsymbols)
    #det=self.det_bareis(M,*(self.pvars+othersymbols[0:2]))
    raise self.CannotSolveError('not implemented')

def isolatepair():
    print 'attempting to isolate a variable'
    finalsolutions = []
    for i in [1,3]: # for every variable, used to be range(4) but it is never the case that [1] fails and [0] succeeds
        # if variable ever appears, it should be alone
        complementvar = unknownvars[[1,0,3,2][i]]
        print 'var:',unknownvars[i]
        varsol = None
        for rank,eq in orgeqns:
            if eq.has_any_symbols(unknownvars[i]):
                # the equations can get big, so 'solve' does not work, also it doesn't make sense to solve for degree > 1
                eq2=Poly(eq,unknownvars[i])
                if eq2.degree == 1 and eq2.coeff(0) != S.Zero:
                    # dividing directly leaves common denominators, so use gcd
                    # long fractions can make the gcd computation, so first remove all numbers
                    tsymbolgen = cse_main.numbered_symbols('tconst')
                    coeff0,tsymbols0 = self.replaceNumbers(eq2.coeff(0),tsymbolgen)
                    coeff1,tsymbols1 = self.replaceNumbers(eq2.coeff(1),tsymbolgen)
                    common = gcd(coeff0,coeff1,unknownvars)
                    coeff0,r = div(coeff0,common,unknownvars)
                    coeff1,r = div(coeff1,common,unknownvars)
                    if not coeff1.has_any_symbols(complementvar):
                        varsol = (-coeff0/coeff1).subs(tsymbols0+tsymbols1)
                        break
        if varsol is not None:
            #eq=simplify(fraction(varsol)[0]**2 + fraction(varsol)[1]**2*(complementvar**2 - 1))
            varsolvalid=fraction(varsol)[1]
            valideqs = []
            valideqscheck = []
            for rank,eq in orgeqns:
                # find the max degree tha unknownvars[i] appears
                maxdegree = max([m[i] for m in eq.iter_monoms()])
                if maxdegree <= 1:
                    eqsub = Symbol('tempsym')**maxdegree*eq.as_basic().subs(allsymbols+[(unknownvars[i],varsol)]).subs(fraction(varsol)[1],Symbol('tempsym'))
                    if self.codeComplexity(eqsub) < 70: # bobcat fk has a 75 valued equation that does not simplify
                        #print eqsub,'complexity: ',self.codeComplexity(eqsub)
                        eqsub = simplify(eqsub)
                    else:
                        eqsub=eqsub.expand()
                        print 'solvePairVariables: could not simplify eqsub: ',eqsub
                    eqnew = eqsub.subs(Symbol('tempsym'),fraction(varsol)[1]).expand()
                    if self.codeComplexity(eqnew) < 120:
                        eqnew = simplify(eqnew)
                    else:
                        print 'solvePairVariables: could not simplify eqnew: ',eqnew
                    eqnew = eqnew.expand().subs(reducesubs).expand()
                    if self.codeComplexity(eqnew) < 120:
                        eqnewcheck = self.removecommonexprs(eqnew)
                    else:
                        eqnewcheck = eqnew
                    if eqnew != S.Zero and self.isExpressionUnique(valideqscheck,eqnewcheck) and self.isExpressionUnique(valideqscheck,-eqnewcheck):
                        valideqs.append(eqnew)
                        valideqscheck.append(eqnewcheck)
            if len(valideqs) <= 1:
                continue

            valideqs2 = []
            for eq in valideqs:
                eqnew, symbols = self.groupTerms(eq, unknownvars, symbolgen)
                # only accept simple equations
                if self.codeComplexity(eqnew) < 100:
                    allsymbols += symbols
                    valideqs2.append(eqnew)
            if len(valideqs2) <= 1:
                continue

            self.sortComplexity(valideqs2)
            complementvarsols = []
            othervarpoly = None                        
            othervars = unknownvars[0:2] if i >= 2 else unknownvars[2:4]
            postcheckforzeros = []
            postcheckforrange = []
            postcheckfornonzeros = []
            for eq in valideqs2:
                try:
                    peq = Poly(eq,complementvar)
                except PolynomialError,e:
                    try:
                        peq = Poly(eq,complementvar)
                    except PolynomialError,e:
                        print 'solvePairVariables: ',e
                        continue                                
                if peq.degree == 1: # degree > 1 adds sqrt's
                    solutions = [-peq.coeff(0).subs(allsymbols),peq.coeff(1).subs(allsymbols)]
                    if solutions[0] != S.Zero and solutions[1] != S.Zero and self.isValidSolution(solutions[0]/solutions[1]):
                        complementvarsols.append(solutions)
                        if len(complementvarsols) >= 2:
                            # test new solution with previous ones
                            eq0num,eq0denom = complementvarsols[-1]
                            for eq1num,eq1denom in complementvarsols[:-1]:
                                # although not apparent, this is actually a dangerous transformation that allows
                                # wrong solutions to pass through since complementvar is actually constrained, but the constraint
                                # is ignored. Therefore, this requires us to explicitly check denominator for zero and
                                # that each solution is within the [-1,1] range.
                                neweq = eq0num*eq1denom-eq1num*eq0denom
                                if self.codeComplexity(neweq.expand()) < 700:
                                    neweq = simplify(neweq)
                                neweq = neweq.expand() # added expand due to below Poly call failing
                                if neweq != S.Zero:
                                    try:
                                        othervarpoly = Poly(neweq,*othervars).subs(othervars[0]**2,1-othervars[1]**2).subs(othervars[1]**2,1-othervars[0]**2)
                                        if othervarpoly.expand() != S.Zero:
                                            postcheckforzeros = [varsolvalid, eq0denom, eq1denom]
                                            postcheckfornonzeros = [(eq1num/eq1denom)**2+varsol.subs(complementvar,eq1num/eq1denom)**2-1]
                                            break
                                        else:
                                            othervarpoly = None
                                    except PolynomialError,e:
                                        print e
                            if othervarpoly is not None:
                                break
            if othervarpoly is not None:
                # now we have one polynomial with only one variable (sin and cos)!
                solution = self.solveHighDegreeEquationsHalfAngle([othervarpoly],varsym1 if i < 2 else varsym0)
                solution.postcheckforzeros = [self.removecommonexprs(eq,onlygcd=False,onlynumbers=True) for eq in postcheckforzeros]
                solution.postcheckfornonzeros = [self.removecommonexprs(eq,onlygcd=False,onlynumbers=True) for eq in postcheckfornonzeros]
                solution.postcheckforrange = postcheckforrange
                finalsolutions.append(solution)
                if solution.poly.degree <= 2:
                    # found a really good solution, so choose it
                    break
                else:
                    print 'othervarpoly too complex: ',othervarpoly
    if len(finalsolutions) > 0:
        # find the equation with the minimal degree, and the least code complexity
        return [min(finalsolutions, key=lambda f: f.poly.degree*1e6 + self.codeComplexity(f.poly.as_basic()))]

def solveLinearly(self,raweqns,varsyms,othersolvedvars,maxdegree=1):
    varsubs = []
    unknownvars = []
    for varsym in varsyms:
        varsubs += varsym.subs
        unknownvars += [varsym.cvar,varsym.svar,varsym.var]
    polyeqs = [Poly(eq.subs(varsubs),*unknownvars) for eq in raweqns]
    allmonoms = set()
    newpolyeqs = []
    for peq in polyeqs:
        if peq.degree <= maxdegree:
            allmonoms = allmonoms.union(set(peq.monoms))
            newpolyeqs.append(peq)
    allmonoms = list(allmonoms)
    allmonoms.sort()
    if len(allmonoms) > len(newpolyeqs):
        raise self.CannotSolveError('not enough equations %d>%d'%(len(allmonoms),len(newpolyeqs)))

    if __builtin__.sum(allmonoms[0]) != 0:
        raise self.CannotSolveError('need null space')

    # try to solve for all pairwise variables
    systemofequations = []
    for peq in newpolyeqs:
        if peq.degree <= maxdegree:
            arr = [S.Zero]*len(allmonoms)
            for c,m in peq.iter_terms():
                arr[allmonoms.index(m)] = c
            systemofequations.append(arr)

    singleeqs = None
    M = zeros((len(allmonoms),len(allmonoms)))
    for eqs in combinations(systemofequations,len(allmonoms)):
        for i,arr in enumerate(eqs):
            for j in range(len(allmonoms)):
                M[i,j] = arr[j]
        if __builtin__.sum(allmonoms[0]) == 0:
            # can solve directly
            det = self.det_bareis(M)
            if det != S.Zero:
                break
            X = M[1:,1:].inv()*M[1:,0]
            print X
        else:
            # find a nullspace of M, this means that det(M) = 0

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

def detdialytically():
    M = Mall[2]*leftvar**2+Mall[1]*leftvar+Mall[0]

    tempsymbols = [Symbol('a%d'%i) for i in range(16)]
    tempsubs = []
    for i in range(16):
        if M[i] != S.Zero:
            tempsubs.append((tempsymbols[i],Poly(M[i],leftvar)))
        else:
            tempsymbols[i] = S.Zero
    Mtemp = Matrix(4,4,tempsymbols)                    
    dettemp=Mtemp.det()
    log.info('multiplying all determinant coefficients')
    eqadds = []
    for arg in dettemp.args:
        log.info('%s',arg)
        eqmuls = [Poly(arg2.subs(tempsubs),leftvar) for arg2 in arg.args]
        if eqmuls[0].degree == 0:
            eq = eqmuls.pop(0)
            eqmuls[0] = eqmuls[0]*eq
        while len(eqmuls) > 1:
            ioffset = 0
            eqmuls2 = []
            while ioffset < len(eqmuls)-1:
                eqmuls2.append(eqmuls[ioffset]*eqmuls[ioffset+1])
                ioffset += 2
            eqmuls = eqmuls2
        eqadds.append(eqmuls[0])
    log.info('adding all determinant coefficients')
    eqaddsorg=eqadds
    eqadds2 = []
    for eq in eqadds:
        print 'yo'
        eq2 = Poly(S.Zero,leftvar)
        for c,m in eq.iter_terms():
            eq2 = eq2.add_term(simplifyfn(c),m)
        eqadds2.append(eq2)
    # any further simplification will just freeze the generation process
    det = Poly(S.Zero,leftvar)
    for eq in eqadds2:
        for c,m in eq.iter_terms():
            sym=self.gsymbolgen.next()
            dictequations.append([sym,c])
            det += sym*leftvar**m[0]

    @staticmethod
    def _LUdecompositionFF(M,*vars):
        """
        Function from sympy.
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
                    raise IKFastSolver.CannotSolveError("Matrix is not full rank")
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
                        log.debug('LU %d,%d: %s',i,j,oldpivot)
                        q,r = div(Poly(Ukk * U[i,j] - U[k,j]*Uik,*vars),oldpivot)
                        assert(r==S.Zero)
                        U[i,j] = q
                U[i,k] = S.Zero
            oldpivot = Ukk
        DD[n-1,n-1] = oldpivot
        return P, L, DD, U

def test_ik():
    from sympy import *
    from sympy import S, pi, sin, cos, PolynomialError, Symbol
    import numpy
    import __builtin__
    from openravepy.ikfast import AST, combinations, fmod
    from itertools import izip
    from openravepy import axisAngleFromRotationMatrix
    numpy.set_printoptions(15)
    IkType=IkParameterizationType
    ikmodel=self
    self = solver
    self.ikfast_module = ikmodel.ikfast
    freeindices = ikmodel.freeindices
    log = ikmodel.ikfast.log
    log.setLevel(logging.DEBUG)
    rawglobaldir = [1.0,0.0,0.0]
    rawnormaldir = [0.0,0.0,1.0]
    rawbasedir=dot(ikmodel.manip.GetLocalToolTransform()[0:3,0:3],ikmodel.manip.GetDirection())
    rawbasepos=ikmodel.manip.GetLocalToolTransform()[0:3,3]
    
    chaintree = solver.generateIkSolver(baselink=baselink,eelink=eelink,freeindices=freeindices,solvefn=solvefn)
    code=ikmodel.ikfast.ikfast_generator_cpp.CodeGenerator().generate(chaintree)
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
        valsubs += [(var,value),(Symbol('c%s'%var.name),self.convertRealToRational(cos(value).evalf())),(Symbol('s%s'%var.name),self.convertRealToRational(sin(value).evalf())),(Symbol('t%s'%var.name),self.convertRealToRational(tan(value).evalf())),(Symbol('ht%s'%var.name),self.convertRealToRational(tan(value/2).evalf()))]
    psubs = []
    for i in range(12):
        psubs.append((self.Tee[i],self.convertRealToRational(Tfinal[i].subs(valsubs).evalf())))
    for s,v in self.ppsubs+self.npxyzsubs+self.rxpsubs:
        psubs.append((s,v.subs(psubs)))
    if len(self.globalsymbols) > 0:
        psubs += [(s,v.subs(psubs+valsubs)) for s,v in self.globalsymbols]

    if len(raghavansolutiontree) > 0:
        psubs += [(s,v.subs(psubs)) for s,v in raghavansolutiontree[0].subs]
    dummyvaluesubs = [(dvar,self.convertRealToRational(var.subs(valsubs).evalf())) for dvar,var in dummyvars]
    allsubs = valsubs+psubs+dummyvaluesubs
    localsymbolsvalues = [(var,value.subs(valsubs+psubs)) for var,value in localsymbols]

    for var,value in izip(jointvars,jointvalues):
        valsubs += [(var,value),(Symbol('c%s'%var.name),(cos(value).evalf())),(Symbol('s%s'%var.name),(sin(value).evalf())),(Symbol('t%s'%var.name),(tan(value).evalf())),(Symbol('ht%s'%var.name),(tan(value/2).evalf()))]
    psubs = []
    for i in range(12):
        psubs.append((self.Tee[i],(Tfinal[i].subs(psubs+valsubs).evalf())))
    for s,v in self.ppsubs+self.npxyzsubs+self.rxpsubs:
        psubs.append((s,v.subs(psubs)))

    newsubs=[(var,eq.subs(self.testconsistentvalues[1]).evalf()) for var,eq in coupledsolutions[0].dictequations]
    mpmath.polyroots(coupledsolutions[0].poly.subs(newsubs).coeffs)

    jointvalues = [-0.2898119639388401, 0.0, -5.913881500780583, 0.0, -3.116541584197247, 1.570796326794897]
"""
ikfast notes;

pr2 with conic sections:
success rate: 0.955000, wrong solutions: 0.000000, no solutions: 0.145000, missing solution: 0.281000
mean: 0.000006s, median: 0.000007s, min: 0.000001s, max: 0.000015s

pr2 with half-angle transformation:
success rate: 0.993000, wrong solutions: 0.000000, no solutions: 0.011000, missing solution: 0.081000
mean: 0.000009s, median: 0.000009s, min: 0.000005s, max: 0.000016s
"""
