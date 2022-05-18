// -*- coding: utf-8 -*-
// Copyright (C) 2021 Puttichai Lertkultanon
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
#include "cubicinterpolator.h"

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

CubicInterpolator::CubicInterpolator(size_t ndof_, int envid_)
{
    __description = ":Interface Author: Puttichai Lertkultanon\n\nRoutines for cubic polynomial interpolation with specified boundary conditions.";
    this->Initialize(ndof_, envid_);
}

void CubicInterpolator::Initialize(size_t ndof_, int envid_)
{
    OPENRAVE_ASSERT_OP(ndof_, >, 0);
    this->ndof = ndof_;
    this->envid = envid_;
    _pGeneralInterpolator.reset(new GeneralRecursiveInterpolator(envid_));
    checker.Initialize(ndof_, envid_);

    // Jerk limits might be a few order of magnitude larger than velocity/acceleration
    // limits. Subsequently, when evaluating a value of acceleration (which is linear in the jerk
    // limit), a small discrepancy in time might result in a relatively large discrepancy in
    // acceleration. Therefore, we increase the tolerance for acceleration checking.
    checker.SetEpsilonForAccelerationDiscrepancyChecking(100*g_fPolynomialEpsilon);

    _cache1DCoeffs.resize(4); // 4 coefficients for a cubic polynomial
    _cacheDVect.resize(ndof_);
    _cachePolynomials.resize(ndof_);
    _cacheXVect.resize(ndof_);
    _cachePWPolynomials.resize(ndof_);
}

//
// 1D Functions
//
PolynomialCheckReturn CubicInterpolator::Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(const dReal x0, const dReal x1,
                                                                                                 const dReal vm, const dReal am, const dReal jm,
                                                                                                 PiecewisePolynomial& pwpoly)
{
    const dReal d = RaveFabs(x1 - x0);
    const int icase = DetermineZeroTimeDerivativesCaseFromBoundaryConditions(d, vm, am, jm);
    dReal tj = -1;
    dReal ta = -1;
    dReal tv = -1;
    switch( icase ) {
    case 0:
        tj = am/jm;
        ta = vm/am - tj;
        tv = (d/vm) - (vm/am) - (am/jm);
        break;
    case 1:
        tj = am/jm;
        ta = 0.5*(RaveSqrt(tj*tj + 4*d/am)) - 1.5*tj;
        tv = 0.0;
        break;
    case 2:
        tj = RaveSqrt(vm/jm);
        ta = 0;
        tv = (d/vm) - 2*tj;
        break;
    case 3:
        tj = std::cbrt(0.5*d/jm);
        ta = 0;
        tv = 0;
        break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("Got unexpected icase=%d", icase, ORE_InvalidArguments);
    }

    // Depending on x0 and x1, we may need to start with negative v, a, and j.
    const bool startWithNegativeBounds = x0 > x1;
    const dReal v = startWithNegativeBounds ? -vm : vm;
    const dReal a = startWithNegativeBounds ? -am : am;
    const dReal j = startWithNegativeBounds ? -jm : jm;
    std::vector<Polynomial> vpolynomials; // resulting trajectory
    vpolynomials.reserve(7);
    Polynomial poly; // for temporarily holding the polynomial for each segment

    dReal prevPosition, prevVelocity, prevAcceleration;
    const dReal constantJerk = j/6.0; // segments that have non-zero jerk use this value.

    // Segment1
    poly.Initialize(tj, std::vector<dReal>({x0, 0, 0, constantJerk}));
    vpolynomials.push_back(poly);
    prevPosition = poly.Eval(poly.duration);
    prevVelocity = poly.Evald1(poly.duration);
    prevAcceleration = poly.Evald2(poly.duration);

    // Segment2
    if( ta > 0 ) {
        poly.Initialize(ta, std::vector<dReal>({prevPosition, prevVelocity, 0.5*a, 0.0}));
        vpolynomials.push_back(poly);
        prevPosition = poly.Eval(poly.duration);
        prevVelocity = poly.Evald1(poly.duration);
        prevAcceleration = a;
    }

    // Segment3
    poly.Initialize(tj, std::vector<dReal>({prevPosition, prevVelocity, 0.5*prevAcceleration, -constantJerk}));
    vpolynomials.push_back(poly);
    prevPosition = poly.Eval(poly.duration);
    prevVelocity = poly.Evald1(poly.duration);
    prevAcceleration = 0; // segment 3 ends with zero acceleration

    // Segment4
    if( tv > 0 ) {
        poly.Initialize(tv, std::vector<dReal>({prevPosition, v, 0.0, 0.0}));
        vpolynomials.push_back(poly);
        prevPosition = poly.Eval(poly.duration);
        prevVelocity = v;
        prevAcceleration = 0;
    }

    // Segment5
    poly.Initialize(tj, std::vector<dReal>({prevPosition, prevVelocity, 0.0, -constantJerk}));
    vpolynomials.push_back(poly);
    prevPosition = poly.Eval(poly.duration);
    prevVelocity = poly.Evald1(poly.duration);
    prevAcceleration = poly.Evald2(poly.duration);

    // Segment6
    if( ta > 0 ) {
        poly.Initialize(ta, std::vector<dReal>({prevPosition, prevVelocity, -0.5*a, 0.0}));
        vpolynomials.push_back(poly);
        prevPosition = poly.Eval(poly.duration);
        prevVelocity = poly.Evald1(poly.duration);
        prevAcceleration = -a;
    }

    // Segment7
    poly.Initialize(tj, std::vector<dReal>({prevPosition, prevVelocity, 0.5*prevAcceleration, constantJerk}));
    vpolynomials.push_back(poly);

    pwpoly.Initialize(vpolynomials);

    return PolynomialCheckReturn::PCR_Normal;
}

int CubicInterpolator::DetermineZeroTimeDerivativesCaseFromBoundaryConditions(const dReal d, const dReal vm, const dReal am, const dReal jm)
{
    BOOST_ASSERT(d >= 0);
    const bool condition1 = (vm >= am*am/jm);
    const bool condition2 = (d >= 2*am*am*am/(jm*jm));
    const bool condition3 = (d*d >= 4*vm*vm*vm/jm);
    const bool condition4 = (d >= (vm*vm/am) + (am*vm/jm));

    if( condition1 ) {
        if( condition2 ) {
            if( condition4 ) {
                return 0; // case A
            }
            else {
                return 1; // case B
            }
        }
        else {
            if( condition3 ) {
                return 0; // case A
            }
            else {
                return 3; // case D
            }
        }
    }
    else {
        if( condition2 ) {
            if( condition4 ) {
                return 2; // case C
            }
            else {
                return 1; // case B
            }
        }
        else {
            if( condition3 ) {
                return 2; // case C
            }
            else {
                return 3; // case D
            }
        }
    }
}

PolynomialCheckReturn CubicInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(const dReal x0, const dReal x1, const dReal v0, const dReal v1, const dReal a0, const dReal a1,
                                                                                                      const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm,
                                                                                                      PiecewisePolynomial& pwpoly)
{
    if( v0 == 0 && v1 == 0 && a0 == 0 && a1 == 0 ) {
        // Save some time by using a non-iterative method when possible.
        return Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(x0, x1, vm, am, jm, pwpoly);
    }
    const std::vector<dReal> initialState({x0, v0, a0});
    const std::vector<dReal> finalState({x1, v1, a1});
    const std::vector<dReal> lowerBounds({xmin, -vm, -am, -jm});
    const std::vector<dReal> upperBounds({xmax, vm, am, jm});
    const size_t degree = 3;
    PolynomialCheckReturn ret = _pGeneralInterpolator->Compute1DTrajectory(degree, initialState, finalState, lowerBounds, upperBounds, 0, pwpoly);
    if( ret != PolynomialCheckReturn::PCR_Normal ) {
        return ret;
    }
    pwpoly.CleanUp();
    ret = checker.CheckPiecewisePolynomial(pwpoly, xmin, xmax, vm, am, jm, x0, x1, v0, v1, a0, a1);
    return ret;
}

PolynomialCheckReturn CubicInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(const dReal x0, const dReal x1, const dReal v0, const dReal v1, const dReal a0, const dReal a1, const dReal T,
                                                                                                  const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm,
                                                                                                  PiecewisePolynomial& pwpoly)
{
    const std::vector<dReal> initialState({x0, v0, a0});
    const std::vector<dReal> finalState({x1, v1, a1});
    const std::vector<dReal> lowerBounds({xmin, -vm, -am, -jm});
    const std::vector<dReal> upperBounds({xmax, vm, am, jm});
    const size_t degree = 3;
    PolynomialCheckReturn ret = _pGeneralInterpolator->Compute1DTrajectory(degree, initialState, finalState, lowerBounds, upperBounds, T, pwpoly);
    if( ret != PolynomialCheckReturn::PCR_Normal ) {
        return ret;
    }
    pwpoly.CleanUp();
    ret = checker.CheckPiecewisePolynomial(pwpoly, xmin, xmax, vm, am, jm, x0, x1, v0, v1, a0, a1);
    return ret;
}

//
// ND Functions
//
PolynomialCheckReturn CubicInterpolator::ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
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
    const std::vector<Polynomial>& vTemplatePolynomials = templatePWPolynomial.GetPolynomials();
    chunks.resize(vTemplatePolynomials.size()); // output

    std::vector<dReal>& vcoeffs = _cache1DCoeffs; // for holding temporary coefficients for each dof. should already have size=4
    std::vector<Polynomial>& vpolynomials = _cachePolynomials; // should already have size=ndof
    std::vector<dReal>& xVect = _cacheXVect; // for holding the initial positions for each segment.
    xVect = x0Vect;

    size_t iCurrentChunk = 0;
    for( std::vector<Polynomial>::const_iterator itpoly = vTemplatePolynomials.begin(); itpoly != vTemplatePolynomials.end(); ++itpoly, ++iCurrentChunk ) {
        // For each of the template polynomial, initialize 1 chunk.

        for( size_t idof = 0; idof < ndof; ++idof ) {
            size_t icoeff = 0;
            for( std::vector<dReal>::const_iterator itTemplateCoeff = itpoly->vcoeffs.begin(); itTemplateCoeff != itpoly->vcoeffs.end(); ++itTemplateCoeff, ++icoeff ) {
                vcoeffs[icoeff] = dVect[idof] * (*itTemplateCoeff);
            }
            vcoeffs[0] = xVect[idof];
            vpolynomials[idof].Initialize(itpoly->duration, vcoeffs);
        }

        chunks[iCurrentChunk].Initialize(itpoly->duration, vpolynomials);
        chunks[iCurrentChunk].Eval(itpoly->duration, xVect);
    }
    return PolynomialCheckReturn::PCR_Normal;
}

PolynomialCheckReturn CubicInterpolator::ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
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
    OPENRAVE_ASSERT_OP(xminVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(xmaxVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(vmVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(amVect.size(), ==, ndof);

    // Check inputs
    for( size_t idof = 0; idof < ndof; ++idof ) {
        const dReal& xmin = xminVect[idof];
        const dReal& xmax = xmaxVect[idof];
        const dReal& vm = vmVect[idof];
        const dReal& am = amVect[idof];
        if( x0Vect[idof] > xmax + g_fPolynomialEpsilon || x0Vect[idof] < xmin - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_PositionLimitsViolation;
        }
        if( x1Vect[idof] > xmax + g_fPolynomialEpsilon || x1Vect[idof] < xmin - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_PositionLimitsViolation;
        }
        if( v0Vect[idof] > vm + g_fPolynomialEpsilon || v0Vect[idof] < -vm - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_VelocityLimitsViolation;
        }
        if( v1Vect[idof] > vm + g_fPolynomialEpsilon || v1Vect[idof] < -vm - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_VelocityLimitsViolation;
        }
        if( a0Vect[idof] > am + g_fPolynomialEpsilon || a0Vect[idof] < -am - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_AccelerationLimitsViolation;
        }
        if( a1Vect[idof] > am + g_fPolynomialEpsilon || a1Vect[idof] < -am - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_AccelerationLimitsViolation;
        }
    }

    for( size_t idof = 0; idof < ndof; ++idof ) {
        PolynomialCheckReturn ret = Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(
            x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], a0Vect[idof], a1Vect[idof], T,
            xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], jmVect[idof],
            _cachePWPolynomials[idof]);
        if( ret != PolynomialCheckReturn::PCR_Normal ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, interpolation failed idof=%d; T=%.15f; x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15f; a0=%.15f; a1=%.15f; xmin=%.15f; xmax=%.15f; vm=%.15f; am=%.15f; jm=%.15f; ret=%s",
                                   envid%idof%T%x0Vect[idof]%x1Vect[idof]%v0Vect[idof]%v1Vect[idof]%a0Vect[idof]%a1Vect[idof]
                                   %xminVect[idof]%xmaxVect[idof]%vmVect[idof]%amVect[idof]%jmVect[idof]
                                   %GetPolynomialCheckReturnString(ret));
            return ret;
        }
        if( !FuzzyEquals(T, _cachePWPolynomials[idof].GetDuration(), g_fPolynomialEpsilon) ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, interpolation failed idof=%d; T=%.15f; x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15f; a0=%.15f; a1=%.15f; xmin=%.15f; xmax=%.15f; vm=%.15f; am=%.15f; jm=%.15f; ret=%s",
                                   envid%idof%T%x0Vect[idof]%x1Vect[idof]%v0Vect[idof]%v1Vect[idof]%a0Vect[idof]%a1Vect[idof]
                                   %xminVect[idof]%xmaxVect[idof]%vmVect[idof]%amVect[idof]%jmVect[idof]
                                   %GetPolynomialCheckReturnString(ret));
            return PolynomialCheckReturn::PCR_DurationDiscrepancy;
        }
    }

    ConvertPiecewisePolynomialsToChunks(_cachePWPolynomials, chunks);
    return checker.CheckChunks(chunks, xminVect, xmaxVect, vmVect, amVect, jmVect,
                               x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect);
}

PolynomialCheckReturn CubicInterpolator::ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                                      const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                                      const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect,
                                                                                                      const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                                      const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                                      const dReal maxAllowedDuration, std::vector<Chunk>& chunks)
{
    OPENRAVE_ASSERT_OP(x0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(v0Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(v1Vect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(xminVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(xmaxVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(vmVect.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(amVect.size(), ==, ndof);

    // Check inputs
    for( size_t idof = 0; idof < ndof; ++idof ) {
        const dReal& xmin = xminVect[idof];
        const dReal& xmax = xmaxVect[idof];
        const dReal& vm = vmVect[idof];
        const dReal& am = amVect[idof];
        if( x0Vect[idof] > xmax + g_fPolynomialEpsilon || x0Vect[idof] < xmin - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_PositionLimitsViolation;
        }
        if( x1Vect[idof] > xmax + g_fPolynomialEpsilon || x1Vect[idof] < xmin - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_PositionLimitsViolation;
        }
        if( v0Vect[idof] > vm + g_fPolynomialEpsilon || v0Vect[idof] < -vm - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_VelocityLimitsViolation;
        }
        if( v1Vect[idof] > vm + g_fPolynomialEpsilon || v1Vect[idof] < -vm - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_VelocityLimitsViolation;
        }
        if( a0Vect[idof] > am + g_fPolynomialEpsilon || a0Vect[idof] < -am - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_AccelerationLimitsViolation;
        }
        if( a1Vect[idof] > am + g_fPolynomialEpsilon || a1Vect[idof] < -am - g_fPolynomialEpsilon ) {
            return PolynomialCheckReturn::PCR_AccelerationLimitsViolation;
        }
    }

    // First find out which joint takes the longest to reach the goal
    dReal maxDuration = 0;
    dReal curDuration; // cache
    size_t maxIndex= 0;
    for( size_t idof = 0; idof < ndof; ++idof ) {
        PolynomialCheckReturn ret = Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(
            x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], a0Vect[idof], a1Vect[idof],
            xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], jmVect[idof],
            _cachePWPolynomials[idof]);
        if( ret != PolynomialCheckReturn::PCR_Normal ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, failed at initial interpolation for idof=%d; x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15f; a0=%.15f; a1=%.15f; xmin=%.15f; xmax=%.15f; vm=%.15f; am=%.15f; jm=%.15f; ret=%s",
                                   envid%idof%x0Vect[idof]%x1Vect[idof]%v0Vect[idof]%v1Vect[idof]%a0Vect[idof]%a1Vect[idof]
                                   %xminVect[idof]%xmaxVect[idof]%vmVect[idof]%amVect[idof]%jmVect[idof]
                                   %GetPolynomialCheckReturnString(ret));
            return ret;
        }
        curDuration = _cachePWPolynomials[idof].GetDuration();
        if( maxAllowedDuration > 0 && curDuration > maxAllowedDuration ) {
            // Resulting duration too long. Reject it right away.
            RAVELOG_VERBOSE_FORMAT("env=%d, idof=%d rejected since duration too long: maxAllowedDuration=%.15f; duration=%.15f; x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15f; a0=%.15f; a1=%.15f; xmin=%.15f; xmax=%.15f; vm=%.15f; am=%.15f; jm=%.15f;",
                                   envid%idof%maxAllowedDuration%curDuration
                                   %x0Vect[idof]%x1Vect[idof]%v0Vect[idof]%v1Vect[idof]%a0Vect[idof]%a1Vect[idof]
                                   %xminVect[idof]%xmaxVect[idof]%vmVect[idof]%amVect[idof]%jmVect[idof]);
            return PolynomialCheckReturn::PCR_DurationTooLong;
        }
        if( curDuration > maxDuration ) {
            maxDuration = curDuration;
            maxIndex = idof;
        }
    }

    // Stretch all other 1D trajectories to the same duration maxDuration.
    for( size_t idof = 0; idof < ndof; ++idof ) {
        if( idof == maxIndex ) {
            continue;
        }

        PolynomialCheckReturn ret = Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(
            x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], a0Vect[idof], a1Vect[idof], maxDuration,
            xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], jmVect[idof],
            _cachePWPolynomials[idof]);
        if( ret != PolynomialCheckReturn::PCR_Normal ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, failed to stretch idof=%d; fixedDuration=%.15f; x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15f; a0=%.15f; a1=%.15f; xmin=%.15f; xmax=%.15f; vm=%.15f; am=%.15f; jm=%.15f; ret=%s",
                                   envid%idof%maxDuration%x0Vect[idof]%x1Vect[idof]%v0Vect[idof]%v1Vect[idof]%a0Vect[idof]%a1Vect[idof]
                                   %xminVect[idof]%xmaxVect[idof]%vmVect[idof]%amVect[idof]%jmVect[idof]
                                   %GetPolynomialCheckReturnString(ret));
            return ret;
        }
        if( !FuzzyEquals(maxDuration, _cachePWPolynomials[idof].GetDuration(), g_fPolynomialEpsilon) ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, failed to stretch idof=%d; fixedDuration=%.15f; x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15f; a0=%.15f; a1=%.15f; xmin=%.15f; xmax=%.15f; vm=%.15f; am=%.15f; jm=%.15f; ret=%s",
                                   envid%idof%maxDuration%x0Vect[idof]%x1Vect[idof]%v0Vect[idof]%v1Vect[idof]%a0Vect[idof]%a1Vect[idof]
                                   %xminVect[idof]%xmaxVect[idof]%vmVect[idof]%amVect[idof]%jmVect[idof]
                                   %GetPolynomialCheckReturnString(ret));
            return PolynomialCheckReturn::PCR_DurationDiscrepancy;
        }
    }

    ConvertPiecewisePolynomialsToChunks(_cachePWPolynomials, chunks);
    return checker.CheckChunks(chunks, xminVect, xmaxVect, vmVect, amVect, jmVect,
                               x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect);
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
