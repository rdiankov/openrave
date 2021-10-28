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
#include "generalrecursiveinterpolator.h"

// #define GENERALINTERPOLATOR_PROGRESS_DEBUG

namespace OpenRAVE {

namespace RampOptimizer = RampOptimizerInternal;

namespace PiecewisePolynomialsInternal {

GeneralRecursiveInterpolator::GeneralRecursiveInterpolator(int envid)
{
    __description = ":Interface Author: Puttichai Lertkultanon\n\nImplements polynomial interpolation routines. See\n\n\
Ezair, B., Tassa, T., & Shiller, Z. (2014). Planning high order trajectories with general initial and final conditions and asymmetric bounds. The International Journal of Robotics Research, 33(6), 898-916.";
    this->Initialize(envid);
}

void GeneralRecursiveInterpolator::Initialize(int envid)
{
    this->envid = envid;
    checker.Initialize(1, envid);
    parabolicInterpolator.Initialize(1, envid);
}

PolynomialCheckReturn GeneralRecursiveInterpolator::ComputeParabolic1DTrajectoryOptimizedDuration(
    const dReal x0, const dReal x1,
    const dReal v0, const dReal v1,
    const dReal xmin, const dReal xmax,
    const dReal vm, const dReal am,
    PiecewisePolynomial& pwpoly)
{
    RampOptimizer::ParabolicCurve curve;
    bool bInterpolationSuccess = parabolicInterpolator.Compute1DTrajectory(x0, x1, v0, v1, vm, am, curve);
    if( !bInterpolationSuccess ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, failed in ComputeParabolic1DTrajectoryOptimizedDuration", envid);
        return PolynomialCheckReturn::PCR_GenericError;
    }

    return PostProcessParabolic1DTrajectory(curve, x0, x1, v0, v1, xmin, xmax, vm, am, pwpoly);
}

PolynomialCheckReturn GeneralRecursiveInterpolator::ComputeParabolic1DTrajectoryFixedDuration(
    const dReal x0, const dReal x1,
    const dReal v0, const dReal v1,
    const dReal xmin, const dReal xmax,
    const dReal vm, const dReal am,
    const dReal fixedDuration,
    PiecewisePolynomial& pwpoly)
{
    RampOptimizer::ParabolicCurve curve;
    bool bInterpolationSuccess = parabolicInterpolator.Compute1DTrajectoryFixedDuration(x0, x1, v0, v1, vm, am, fixedDuration, curve);
    if( !bInterpolationSuccess ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, failed in ComputeParabolic1DTrajectoryFixedDuration", envid);
        return PolynomialCheckReturn::PCR_GenericError;
    }

    return PostProcessParabolic1DTrajectory(curve, x0, x1, v0, v1, xmin, xmax, vm, am, pwpoly);
}

PolynomialCheckReturn GeneralRecursiveInterpolator::PostProcessParabolic1DTrajectory(
    RampOptimizer::ParabolicCurve& curve,
    const dReal x0, const dReal x1,
    const dReal v0, const dReal v1,
    const dReal xmin, const dReal xmax,
    const dReal vm, const dReal am,
    PiecewisePolynomial& pwpoly)
{
    bool bFixLimits = parabolicInterpolator._ImposeJointLimitFixedDuration(curve, xmin, xmax, vm, am);
    if( !bFixLimits ) {
        return PolynomialCheckReturn::PCR_PositionLimitsViolation;
    }

    RampOptimizer::ParabolicCheckReturn parabolicret = RampOptimizer::CheckRamps(curve.GetRamps(), xmin, xmax, vm, am, x0, x1, v0, v1);
    if( parabolicret != RampOptimizer::ParabolicCheckReturn::PCR_Normal ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, failed in PostProcessParabolic1DTrajectory", envid);
        return PolynomialCheckReturn::PCR_GenericError;
    }
    ConvertParabolicCurveToPiecewisePolynomial(curve, pwpoly);
    return PolynomialCheckReturn::PCR_Normal;
}

void GeneralRecursiveInterpolator::ConvertParabolicCurveToPiecewisePolynomial(const RampOptimizer::ParabolicCurve& curve, PiecewisePolynomial& pwpoly)
{
    const size_t numRamps = curve.GetRamps().size();
    std::vector<Polynomial> vpolynomials(numRamps);
    for( size_t iramp = 0; iramp < numRamps; ++iramp ) {
        const RampOptimizer::Ramp& ramp = curve.GetRamp(iramp);
        vpolynomials[iramp].Initialize(ramp.duration, {ramp.x0, ramp.v0, 0.5*ramp.a});
    }
    pwpoly.Initialize(vpolynomials);
}

PolynomialCheckReturn GeneralRecursiveInterpolator::Compute1DTrajectory(
    const size_t degree,
    const std::vector<dReal>& initialState, const std::vector<dReal>& finalState,
    const std::vector<dReal>& lowerBounds, const std::vector<dReal>& upperBounds,
    const dReal fixedDuration,
    PiecewisePolynomial& pwpoly)
{
    BOOST_ASSERT(degree >= 2);
    BOOST_ASSERT(degree == initialState.size());
    BOOST_ASSERT(degree == finalState.size());
    BOOST_ASSERT(degree + 1 == lowerBounds.size());
    BOOST_ASSERT(degree + 1 == upperBounds.size());

    // Step 1
    if( degree == 2 ) {
        // Currently the method for parabolic trajectories only accepts symmetric bounds
        const dReal vm = Min(upperBounds.at(velocityIndex), RaveFabs(lowerBounds.at(velocityIndex)));
        const dReal am = Min(upperBounds.at(accelerationIndex), RaveFabs(lowerBounds.at(accelerationIndex)));

        if( fixedDuration > 0 ) {
            return ComputeParabolic1DTrajectoryFixedDuration(initialState.at(positionIndex), finalState.at(positionIndex),
                                                             initialState.at(velocityIndex), finalState.at(velocityIndex),
                                                             lowerBounds.at(positionIndex), upperBounds.at(positionIndex),
                                                             vm, am, fixedDuration, pwpoly);
        }
        else {
            return ComputeParabolic1DTrajectoryOptimizedDuration(initialState.at(positionIndex), finalState.at(positionIndex),
                                                                 initialState.at(velocityIndex), finalState.at(velocityIndex),
                                                                 lowerBounds.at(positionIndex), upperBounds.at(positionIndex),
                                                                 vm, am, pwpoly);
        }
    }

    // Step 2
    const dReal deltaX = finalState.at(positionIndex) - initialState.at(positionIndex);

    // Step 3
    dReal vmin = lowerBounds.at(velocityIndex);
    dReal vmax = upperBounds.at(velocityIndex);
    dReal v = vmax + 1;
    dReal vHat = vmax + 1;

#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
    RAVELOG_INFO_FORMAT("env=%d, new problem with degree=%d; deltaX=%.15e; vmin=%.15e; vmax=%.15e; fixedDuration=%.15e", envid%degree%deltaX%vmin%vmax%fixedDuration);
#endif

    // Step 4
    const size_t maxIters = 1000; // to prevent infinite loops
    dReal vLast;
    PiecewisePolynomial pwpoly1, pwpoly3; // the use of indices 1 and 3 are according to the paper
    PiecewisePolynomial pwpoly1Integrated, pwpoly3Integrated;
    PolynomialCheckReturn ret1, ret3;
    std::vector<dReal> newInitialState(initialState.size() - 1, 0.0), newFinalState(finalState.size() - 1, 0.0);
    std::vector<dReal> newLowerBounds(lowerBounds.size() - 1, 0.0), newUpperBounds(upperBounds.size() - 1, 0.0);
    newLowerBounds.assign(lowerBounds.begin() + velocityIndex, lowerBounds.end());
    newUpperBounds.assign(upperBounds.begin() + velocityIndex, upperBounds.end());

    dReal totalDuration = 0;
    dReal duration2 = 0;
    bool bSuccess = false;
    for( size_t iter = 0; iter < maxIters; ++iter ) {
        // Step 5
        vLast = v;
        // Step 6
        v = 0.5*(vmin + vmax);

        // Step 7
        newInitialState.assign(initialState.begin() + velocityIndex, initialState.end());
        std::fill(newFinalState.begin(), newFinalState.end(), 0.0);
        newFinalState[0] = v;
        ret1 = Compute1DTrajectory(degree - 1, newInitialState, newFinalState, newLowerBounds, newUpperBounds, /*fixedDuration*/ 0.0, pwpoly1);
        if( ret1 != PolynomialCheckReturn::PCR_Normal ) {
            return ret1;
        }

        // Step 8
        newInitialState.swap(newFinalState);
        newFinalState.assign(finalState.begin() + velocityIndex, finalState.end());
        ret3 = Compute1DTrajectory(degree - 1, newInitialState, newFinalState, newLowerBounds, newUpperBounds, /*fixedDuration*/ 0.0, pwpoly3);
        if( ret3 != PolynomialCheckReturn::PCR_Normal ) {
            return ret3;
        }

        // Step 9
        pwpoly1Integrated = pwpoly1.Integrate(0);
        // deltaX1: displacement covered by segment I
        dReal deltaX1 = pwpoly1Integrated.Eval(pwpoly1Integrated.GetDuration());
        pwpoly3Integrated = pwpoly3.Integrate(0);
        // deltaX3: displacement covered by segment III
        dReal deltaX3 = pwpoly3Integrated.Eval(pwpoly3Integrated.GetDuration());

        // Step 10
        // delta: displacement to be covered by segment II
        dReal delta = deltaX - deltaX1 - deltaX3;

        // Note: In the paper, there is no explicit consideration for the case when v is zero.
        bool bFreeDuration2 = false;
        if( FuzzyZero(v, g_fPolynomialEpsilon) ) {
            // In this case, v == 0 so we are free to choose the duration of this middle part.
            duration2 = 0; // leave it unchosen first. will compute an appropriate value for duration in the end.
        }
        else {
            duration2 = delta/v;
        }

#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
        RAVELOG_DEBUG_FORMAT("env=%d, degree=%d; iter=%d; deltaX1=%.15e; deltaX3=%.15e; deltaX=%.15e; delta=%.15e; duration2=%.15e;", envid%degree%iter%deltaX1%deltaX3%deltaX%delta%duration2);
#endif

        // Step 11
        if( duration2 >= 0 ) {
            vHat = v;
        }

        // Step 11.5
        if( fixedDuration > 0 ) {
            totalDuration = pwpoly1.GetDuration() + pwpoly3.GetDuration();
            if( duration2 > 0 ) {
                totalDuration += duration2;
                if( totalDuration < fixedDuration ) {
                    delta = -delta;
                }
            }
        }

        // Step 12
        // Note: need a tighter bound when checking these velocities vmax, vmin. Otherwise, it might
        // not converge due to totalDuration not equal to fixedDuration.
        if( FuzzyEquals(vmax, vmin, epsilon) ) {
            v = vHat;
            vmin = vHat;
            vmax = vHat;
            if( FuzzyEquals(vHat, upperBounds.at(velocityIndex) + 1, epsilon) ) {
                // Step 14
                RAVELOG_VERBOSE_FORMAT("env=%d, failed to find a valid solution", envid);
                return PolynomialCheckReturn::PCR_GenericError;
            }
        }
        else if( delta > 0 ) {
            vmin = v;
        }
        else if( delta < 0 ) {
            vmax = v;
        }
        else {
            vLast = v;
        }
#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
        RAVELOG_DEBUG_FORMAT("env=%d, degree=%d; iter=%d; vmin=%.15e; vmax=%.15e; vLast=%.15e; v=%.15e; delta=%.15e", envid%degree%iter%vmin%vmax%vLast%v%delta);
#endif

        if( FuzzyEquals(vLast, v, epsilon) ) {
            if( (fixedDuration == 0) ) {
                // No constraints on the total duration so can stop here
                bSuccess = true;
                break; // successful
            }
            else if( FuzzyZero(v, g_fPolynomialEpsilon) && totalDuration <= fixedDuration + g_fEpsilonForTimeInstant ) {
                // v is zero so we are free to choose duration2. (We have not chosen a value for duration2 yet.)
                duration2 = Max(0, fixedDuration - totalDuration);
                bSuccess = true;
                break;
            }
            else if( FuzzyEquals(fixedDuration, totalDuration, g_fEpsilonForTimeInstant) ) {
                // The computed velocity converges and the total duration meets the given fixed duration.
                bSuccess = true;
                break;
            }
            else {
#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, fixedDuration=%.15e; duration diff=%.15e", envid%fixedDuration%(fixedDuration - totalDuration));
#endif
                if( FuzzyEquals(vmax, vmin, epsilon) ) {
                    // Cannot do anything more since vmax and vmin are equal.
                    return PolynomialCheckReturn::PCR_GenericError;
                }
            }
        }
    } // end for

    if( !bSuccess ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, interpolation failed. max interations %d reached", envid%maxIters);
        return PolynomialCheckReturn::PCR_GenericError;
    }

    // Check soundness
    if( !FuzzyEquals(pwpoly1.Eval(pwpoly1.GetDuration()), v, epsilonFinalValidation) ) {
        RAVELOG_WARN_FORMAT("env=%d, interpolation successful but v1(%f)=%.15e is different from v=%.15e", envid%pwpoly1.GetDuration()%pwpoly1.Eval(pwpoly1.GetDuration())%v);
        return PolynomialCheckReturn::PCR_GenericError;
    }
    if( !FuzzyEquals(pwpoly3.Eval(0), v, epsilonFinalValidation) ) {
        RAVELOG_WARN_FORMAT("env=%d, interpolation successful but v3(0)=%.15e is different from v=%.15e", envid%pwpoly3.Eval(0)%v);
        return PolynomialCheckReturn::PCR_GenericError;
    }

    const bool bHasMiddleSegment = !FuzzyZero(duration2, g_fPolynomialEpsilon);
    const size_t numPolynomials = pwpoly1.GetPolynomials().size() + pwpoly3.GetPolynomials().size() + (bHasMiddleSegment ? 1 : 0);
    std::vector<Polynomial> vFinalPolynomials;
    vFinalPolynomials.reserve(numPolynomials);
    vFinalPolynomials.insert(vFinalPolynomials.end(), pwpoly1.GetPolynomials().begin(), pwpoly1.GetPolynomials().end());
    if( bHasMiddleSegment ) {
        Polynomial poly(duration2, {v});
        poly.PadCoefficients(degree - 1);
        vFinalPolynomials.emplace_back(poly);
    }
    vFinalPolynomials.insert(vFinalPolynomials.end(), pwpoly3.GetPolynomials().begin(), pwpoly3.GetPolynomials().end());
    PiecewisePolynomial pwpolyFinal(vFinalPolynomials);
    pwpoly = pwpolyFinal.Integrate(initialState[positionIndex]);
    return PolynomialCheckReturn::PCR_Normal; // Final piecewise polynomial is to be checked outside.
} // end Compute1DTrajectory

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
