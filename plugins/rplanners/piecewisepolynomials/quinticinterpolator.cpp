// -*- coding: utf-8 -*-
// Copyright (C) 2019 Puttichai Lertkultanon
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation, either version 3
// of the License, or at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
// even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
#include "quinticinterpolator.h"

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

QuinticInterpolator::QuinticInterpolator(size_t ndof, int envid)
{
    this->Initialize(ndof, envid);
}

void QuinticInterpolator::Initialize(size_t ndof, int envid)
{
    OPENRAVE_ASSERT_OP(ndof, >, 0);
    this->ndof = ndof;
    this->envid = envid;
    checker.Initialize(ndof, envid);

    _cache1DCoeffs.resize(6);

    _cacheDVect.resize(ndof);
    _cachePolynomials1.resize(1);
    _cachePolynomials2.resize(ndof);
}

//
// 1D problem
//
PolynomialCheckReturn QuinticInterpolator::Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(dReal x0, dReal x1,
                                                                                                   dReal vm, dReal am, dReal jm,
                                                                                                   std::vector<Polynomial>& polynomials)
{
    /*
       Describing a quintic polynomial as

          p(t) = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0,

       with t \in [0, T], when the terminal velocities and accelerations are all zero, the
       coefficients are as follows

          a5 =   6(x1 - x0)/T^5,
          a4 = -15(x1 - x0)/T^4,
          a3 =  10(x1 - x0)/T^3,
          a2 = 0,
          a1 = 0,
          a0 = x0.

       Substituting back all the coefficients into the polynomial function, we can then write p as
       p(t, T).

       Consider d^2/dt^2 p(t, T). By solving for t from the equation d^2/dt^2 p(t, T) = 0, we have the time
       instant t_vpeak, at which the velocity is maximum (or minimum). Therefore,

          v_peak = d/dt p(t_vpeak, T) = (15/8)*(x1 - x0)/T.

       The total duration T such that the peak velocity is exactly vm is then

          Tv = (15/8)*abs(x1 - x0)/vm.

       Using the same procedure as above, we can compute the total duration such that the peak
       acceleration and the peak jerk are at their limits as

          Ta = sqrt( (10/sqrt(3)) * abs(x1 - x0)/am ) and
          Tj = cbrt( 30 * abs(x1 - x0)/jm ).

       Finally, the duration T that we choose such that all limits are respected is determined by

          T = max(Tv, Ta, Tj).
     */
    dReal absd = Abs(x1 - x0);
    dReal Tv = _fifteenOverEight * absd/vm;
    dReal Ta = Sqrt( _tenOverSqrtThree * absd/am );
    dReal Tj = Cbrt( 30 * absd/jm );

    dReal T = Max(Max(Tv, Ta), Tj);
    dReal T2 = T*T;
    dReal T3 = T2*T;
    dReal T4 = T3*T;
    dReal T5 = T4*T;

    std::vector<dReal>& vcoeffs = _cache1DCoeffs;
    vcoeffs[0] = x0;
    vcoeffs[1] = 0;
    vcoeffs[2] = 0;
    vcoeffs[3] = 10*(x1 - x0)/T3;
    vcoeffs[4] = 15*(x0 - x1)/T4;
    vcoeffs[5] = 6*(x1 - x0)/T5;

    polynomials.resize(1);
    polynomials[0].Initialize(T, vcoeffs);
    return PolynomialCheckReturn::PCR_Normal;
}

PolynomialCheckReturn QuinticInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1, dReal T,
                                                                                                    dReal xmin, dReal xmax, dReal vm, dReal am, dReal jm,
                                                                                                    std::vector<Polynomial>& polynomials)
{
    dReal T2 = T*T;
    dReal T3 = T2*T;
    dReal T4 = T3*T;
    dReal T5 = T4*T;

    std::vector<dReal>& vcoeffs = _cache1DCoeffs;
    vcoeffs[0] = x0;
    vcoeffs[1] = v0;
    vcoeffs[2] = 0.5*a0;
    vcoeffs[3] = (T2*(a1 - 3.0*a0) - T*(12.0*v0 + 8.0*v1) + 20.0*(x1 - x0))/(2*T3);
    vcoeffs[4] = (T2*(3.0*a0 - 2.0*a1) + T*(16.0*v0 + 14.0*v1) + 30.0*(x0 - x1))/(2*T4);
    vcoeffs[5] = (T2*(a1 - a0) - 6.0*T*(v1 + v0) + 12.0*(x1 - x0))/(2*T5);
    polynomials.resize(1);
    polynomials[0].Initialize(T, vcoeffs);
    return checker.CheckPolynomial(polynomials[0], xmin, xmax, vm, am, jm);
}

//
// ND problem
//
PolynomialCheckReturn QuinticInterpolator::ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                                   const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                                   std::vector<Chunk>& chunks)
{
    OPENRAVE_ASSERT_OP(x0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(vmVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(amVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(jmVect.size(), ==, ndof);

    std::vector<dReal>& dVect = _cacheDVect;
    SubtractVector(x1Vect, x0Vect, dVect);

    // Compute the limiting velocity, acceleration, and jerk (sdMax, sddMax, sdddMax).
    dReal vMin = g_fPolynomialInf;
    dReal aMin = g_fPolynomialInf;
    dReal jMin = g_fPolynomialInf;
    dReal absDInv;
    for( size_t idof = 0; idof < ndof; ++idof ) {
        if( !FuzzyZero(dVect[idof], g_fPolynomialEpsilon) ) {
            absDInv = Abs(1/dVect[idof]);
            vMin = Min(vMin, vmVect[idof]*absDInv);
            aMin = Min(aMin, amVect[idof]*absDInv);
            jMin = Min(jMin, jmVect[idof]*absDInv);
        }
    }
    if( !(vMin < g_fPolynomialInf && aMin < g_fPolynomialInf && jMin < g_fPolynomialInf) ) {
        // Displacements are zero.
        chunks.resize(1);
        chunks[0].SetConstant(x0Vect, 0, 5);
        return PolynomialCheckReturn::PCR_Normal;
    }

    std::vector<Polynomial>& templatePolynomials = _cachePolynomials1;
    Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(0, 1, vMin, aMin, jMin, templatePolynomials);
    const Polynomial& p = templatePolynomials[0];

    std::vector<dReal>& vcoeffs = _cache1DCoeffs;
    std::vector<Polynomial>& vpolynomials = _cachePolynomials2; // should already have size=ndof
    for( size_t idof = 0; idof < ndof; ++idof ) {
        size_t icoeff = 0;
        for( std::vector<dReal>::const_iterator it = p.vcoeffs.begin(); it != p.vcoeffs.end(); ++it, ++icoeff ) {
            vcoeffs[icoeff] = dVect[idof] * (*it);
        }
        vcoeffs[0] += x0Vect[idof];
        vpolynomials[idof].Initialize(p.duration, vcoeffs);
    }
    chunks.resize(1);
    chunks[0].Initialize(p.duration, vpolynomials);
    return PolynomialCheckReturn::PCR_Normal;
}

PolynomialCheckReturn QuinticInterpolator::ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                                    const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                                    const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect, const dReal T,
                                                                                                    const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                                    const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                                    std::vector<Chunk>& chunks)
{
    OPENRAVE_ASSERT_OP(x0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(v0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(v1Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(a0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(a1Vect.size(), ==, ndof);

    std::vector<Polynomial>& resultPolynomials = _cachePolynomials1;
    std::vector<Polynomial>& finalPolynomials = _cachePolynomials2; // should already have size=ndof
    for( size_t idof = 0; idof < ndof; ++idof ) {
        Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], a0Vect[idof], a1Vect[idof], T,
                                                                 xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], jmVect[idof], resultPolynomials);
        finalPolynomials[idof] = resultPolynomials[0]; // can do this since for this quintic interpolator, resultPolynomials always has size=1
    }
    chunks.resize(1);
    chunks[0].Initialize(T, finalPolynomials);

    return checker.CheckChunk(chunks[0], xminVect, xmaxVect, vmVect, amVect, jmVect);
}

PolynomialCheckReturn QuinticInterpolator::ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                                        const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                                        const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect,
                                                                                                        const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                                        const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                                        const dReal T, std::vector<Chunk>& chunks)
{
    OPENRAVE_ASSERT_OP(x0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(v0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(v1Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(a0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(a1Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(xminVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(xmaxVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(vmVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(amVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(jmVect.size(), ==, ndof);

    bool bFound = false; // true if any of the interpolated trajectories is good.
    std::vector<Chunk>& tempChunks = _cacheChunks;

    // Try a greedy approach. Continue the interpolation with less duration even though the initial interpolation fails.
    PolynomialCheckReturn ret = ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, T,
                                                                                         xminVect, xmaxVect, vmVect, amVect, jmVect, tempChunks);
    if( ret == PolynomialCheckReturn::PCR_Normal ) {
        chunks = tempChunks;
        bFound = true;
    }

    dReal fStepSize = 0.5*T;
    dReal fCutoff = 1e-6;
    dReal Tcur = T;
    while( fStepSize >= fCutoff ) {
        dReal fTestDuration = Tcur - fStepSize;
        PolynomialCheckReturn ret2 = ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, fTestDuration,
                                                                                              xminVect, xmaxVect, vmVect, amVect, jmVect, tempChunks);
        if( ret2 == PolynomialCheckReturn::PCR_Normal ) {
            Tcur = fTestDuration;
            chunks = tempChunks;
            bFound = true;
        }
        fStepSize = 0.5*fStepSize;
    }
    if( bFound ) {
        return PolynomialCheckReturn::PCR_Normal;
    }
    else {
        return ret; // return the first error
    }
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE

