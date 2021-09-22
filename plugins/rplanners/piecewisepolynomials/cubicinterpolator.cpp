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

CubicInterpolator::CubicInterpolator(size_t ndof, int envid)
{
    this->Initialize(ndof, envid);
}

void CubicInterpolator::Initialize(size_t ndof, int envid)
{
    OPENRAVE_ASSERT_OP(ndof, >, 0);
    this->ndof = ndof;
    this->envid = envid;
    checker.Initialize(ndof, envid);

    // TODO
}

//
// 1D Functions
//
PolynomialCheckReturn CubicInterpolator::Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(dReal x0, dReal x1,
                                                                                                 dReal vm, dReal am, dReal jm,
                                                                                                 PiecewisePolynomial& pwpoly)
{
    const dReal d = RaveFabs(x0 - x1);
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
    std::vector<dReal> vcoeffs(4, 0);

    // Segment1
    poly.Initialize(tj, std::vector<dReal>({x0, 0, 0, j/6.0}));
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
    poly.Initialize(tj, std::vector<dReal>({prevPosition, prevVelocity, 0.5*prevAcceleration, -j/6.0}));
    vpolynomials.push_back(poly);
    prevPosition = poly.Eval(poly.duration);
    prevVelocity = poly.Evald1(poly.duration);
    prevAcceleration = 0; // segment 3 ends with zero acceleration

    // Segment4
    if( tv > 0 ) {
        poly.Initialize(ta, std::vector<dReal>({prevPosition, v, 0.0, 0.0}));
        vpolynomials.push_back(poly);
        prevPosition = poly.Eval(poly.duration);
        prevVelocity = v;
        prevAcceleration = 0;
    }

    // Segment5
    poly.Initialize(tj, std::vector<dReal>({prevPosition, prevVelocity, 0.0, -j/6.0}));
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
    poly.Initialize(tj, std::vector<dReal>({prevPosition, prevVelocity, 0.5*prevAcceleration, j/6.0}));
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
            if( condition3 ) {
                return 2; // case C
            }
            else {
                return 1; // case B
            }
        }
        else {
            if( condition3 ) {
                return 1; // case B
            }
            else {
                return 3; // case D
            }
        }
    }
}

PolynomialCheckReturn CubicInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1,
                                                                                                      dReal xmin, dReal xmax, dReal vm, dReal am, dReal jm,
                                                                                                      PiecewisePolynomial& pwpoly)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration not implemented", ORE_NotImplemented);
}

PolynomialCheckReturn CubicInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1, dReal T,
                                                                                                  dReal xmin, dReal xmax, dReal vm, dReal am, dReal jm,
                                                                                                  PiecewisePolynomial& pwpoly)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration not implemented", ORE_NotImplemented);
}

//
// ND Functions
//
PolynomialCheckReturn CubicInterpolator::ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                                 const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                                 std::vector<Chunk>& chunks)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration not implemented", ORE_NotImplemented);
}

PolynomialCheckReturn CubicInterpolator::ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                                  const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                                  const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect, const dReal T,
                                                                                                  const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                                  const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                                  std::vector<Chunk>& chunks)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration not implemented", ORE_NotImplemented);
}

PolynomialCheckReturn CubicInterpolator::ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                                      const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                                      const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect,
                                                                                                      const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                                      const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                                      const dReal T, std::vector<Chunk>& chunks)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration not implemented", ORE_NotImplemented);
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
