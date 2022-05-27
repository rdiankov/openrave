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

QuinticInterpolator::QuinticInterpolator(size_t ndof_, int envid_)
{
    __description = ":Interface Author: Puttichai Lertkultanon\n\nRoutines for quintic polynomial interpolation with specified boundary conditions.";
    this->Initialize(ndof_, envid_);
}

void QuinticInterpolator::Initialize(size_t ndof_, int envid_)
{
    OPENRAVE_ASSERT_OP(ndof_, >, 0);
    this->ndof = ndof_;
    this->envid = envid_;
    _pGeneralInterpolator.reset(new GeneralRecursiveInterpolator(envid_));
    checker.Initialize(ndof_, envid_);
    checker.SetEpsilonForJerkLimitsChecking(100*g_fPolynomialEpsilon);

    _cache1DCoeffs.resize(6);

    _cacheDVect.resize(ndof_);
    _cachePolynomials.resize(ndof_);
}

//
// 1D problem
//
PolynomialCheckReturn QuinticInterpolator::Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(const dReal x0, const dReal x1,
                                                                                                   const dReal vm, const dReal am, const dReal jm,
                                                                                                   PiecewisePolynomial& pwpoly)
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
          Tj = cbrt( 60 * abs(x1 - x0)/jm ).

       Finally, the duration T that we choose such that all limits are respected is determined by

          T = max(Tv, Ta, Tj).
     */
    dReal absd = Abs(x1 - x0);
    dReal Tv = _fifteenOverEight * absd/vm;
    dReal Ta = Sqrt( _tenOverSqrtThree * absd/am );
    dReal Tj = Cbrt( 60 * absd/jm );

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

    Polynomial& polynomial = _cachePolynomial;
    polynomial.Initialize(T, vcoeffs);
    pwpoly.Initialize(polynomial);
    return PolynomialCheckReturn::PCR_Normal;
}

PolynomialCheckReturn QuinticInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1,
                                                                                                        dReal xmin, dReal xmax, dReal vm, dReal am, dReal jm,
                                                                                                        PiecewisePolynomial& pwpoly)
{
    // TODO: Is it ok to set 4-th and 5-th derivatives at boundaries to zero?
    //       What values should the bounds are the 4-th and 5-th derivatives be?
    std::vector<dReal> initialState({x0, v0, a0, 0, 0});
    std::vector<dReal> finalState({x1, v1, a1, 0, 0});
    std::vector<dReal> lowerBounds({xmin, -vm, -am, -jm, -10*jm, -100*jm});
    std::vector<dReal> upperBounds({xmax, vm, am, jm, 10*jm, 100*jm});
    PolynomialCheckReturn ret = _pGeneralInterpolator->Compute1DTrajectory(5, initialState, finalState, lowerBounds, upperBounds, 0, pwpoly);
    if( ret != PolynomialCheckReturn::PCR_Normal ) {
        return ret;
    }
    return checker.CheckPiecewisePolynomial(pwpoly, xmin, xmax, vm, am, jm, x0, x1, v0, v1, a0, a1);
}

PolynomialCheckReturn QuinticInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(const dReal x0, const dReal x1, const dReal v0, const dReal v1, const dReal a0, const dReal a1, const dReal T,
                                                                                                    const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm,
                                                                                                    PiecewisePolynomial& pwpoly)
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

    Polynomial& polynomial = _cachePolynomial;
    polynomial.Initialize(T, vcoeffs);
    pwpoly.Initialize(polynomial);
    return checker.CheckPolynomial(pwpoly.GetPolynomial(0), xmin, xmax, vm, am, jm, x0, x1, v0, v1, a0, a1);
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

    PiecewisePolynomial& templatePWPolynomial = _cachePWPolynomial;
    Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(0.0, 1.0, vMin, aMin, jMin, templatePWPolynomial);
    const Polynomial& p = templatePWPolynomial.GetPolynomial(0);

    std::vector<dReal>& vcoeffs = _cache1DCoeffs;
    std::vector<Polynomial>& vpolynomials = _cachePolynomials; // should already have size=ndof
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

    PiecewisePolynomial& resultPWPolynomial = _cachePWPolynomial;
    std::vector<Polynomial>& finalPolynomials = _cachePolynomials; // should already have size=ndof
    for( size_t idof = 0; idof < ndof; ++idof ) {
        Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], a0Vect[idof], a1Vect[idof], T,
                                                                 xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], jmVect[idof], resultPWPolynomial);
        finalPolynomials[idof] = resultPWPolynomial.GetPolynomial(0); // can do this since for this quintic interpolator, resultPWPolynomial always has one polynomial
    }
    chunks.resize(1);
    chunks[0].Initialize(T, finalPolynomials);

    return checker.CheckChunk(chunks[0], xminVect, xmaxVect, vmVect, amVect, jmVect, x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect);
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
