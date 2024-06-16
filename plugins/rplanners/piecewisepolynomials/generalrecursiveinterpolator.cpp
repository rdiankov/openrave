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

GeneralRecursiveInterpolator::GeneralRecursiveInterpolator(int envid_)
{
    __description = ":Interface Author: Puttichai Lertkultanon\n\nImplements polynomial interpolation routines. The implementation of Compute1DTrajectory function follows\n\n\
Ezair, B., Tassa, T., & Shiller, Z. (2014). Planning high order trajectories with general initial and final conditions and asymmetric bounds. The International Journal of Robotics Research, 33(6), 898-916.\n\n\
with some modifications to make the algorithm more robust.";
    this->Initialize(envid_);
}

void GeneralRecursiveInterpolator::Initialize(int envid_)
{
    this->envid = envid_;
    checker.Initialize(1, envid_);
    parabolicInterpolator.Initialize(1, envid_);
    // Need to set a new epsilon for parabolic interpolator since its default value (g_fRampEpsilon
    // = 1e-10) is too loose compared epsilon = 1e-5*g_fPolynomialEpsilon used here.
    parabolicInterpolator.SetTolerance(epsilonFinalValidation);
    _cacheParabolicCoeffs.resize(3);
}

PolynomialCheckReturn GeneralRecursiveInterpolator::ComputeParabolic1DTrajectoryOptimizedDuration(
    const dReal x0, const dReal x1,
    const dReal v0, const dReal v1,
    const dReal xmin, const dReal xmax,
    const dReal vm, const dReal am,
    PiecewisePolynomial& pwpoly)
{
    bool bInterpolationSuccess = parabolicInterpolator.Compute1DTrajectory(x0, x1, v0, v1, vm, am, _cacheParabolicCurve);
    if( !bInterpolationSuccess ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, failed in ComputeParabolic1DTrajectoryOptimizedDuration", envid);
        return PolynomialCheckReturn::PCR_GenericError;
    }

    return PostProcessParabolic1DTrajectory(_cacheParabolicCurve, x0, x1, v0, v1, xmin, xmax, vm, am, pwpoly);
}

PolynomialCheckReturn GeneralRecursiveInterpolator::ComputeParabolic1DTrajectoryFixedDuration(
    const dReal x0, const dReal x1,
    const dReal v0, const dReal v1,
    const dReal xmin, const dReal xmax,
    const dReal vm, const dReal am,
    const dReal fixedDuration,
    PiecewisePolynomial& pwpoly)
{
    bool bInterpolationSuccess = parabolicInterpolator.Compute1DTrajectoryFixedDuration(x0, x1, v0, v1, vm, am, fixedDuration, _cacheParabolicCurve);
    if( !bInterpolationSuccess ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, failed in ComputeParabolic1DTrajectoryFixedDuration", envid);
        return PolynomialCheckReturn::PCR_GenericError;
    }

    return PostProcessParabolic1DTrajectory(_cacheParabolicCurve, x0, x1, v0, v1, xmin, xmax, vm, am, pwpoly);
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

    ConvertParabolicCurveToPiecewisePolynomial(curve, pwpoly);
    return PolynomialCheckReturn::PCR_Normal;
}

void GeneralRecursiveInterpolator::ConvertParabolicCurveToPiecewisePolynomial(const RampOptimizer::ParabolicCurve& curve, PiecewisePolynomial& pwpoly)
{
    const size_t numRamps = curve.GetRamps().size();
    std::vector<Polynomial>& vpolynomials = _cachePolynomials;
    vpolynomials.resize(numRamps);
    for( size_t iramp = 0; iramp < numRamps; ++iramp ) {
        const RampOptimizer::Ramp& ramp = curve.GetRamp(iramp);
        _cacheParabolicCoeffs[0] = ramp.x0;
        _cacheParabolicCoeffs[1] = ramp.v0;
        _cacheParabolicCoeffs[2] = 0.5*ramp.a;
        vpolynomials[iramp].Initialize(ramp.duration, _cacheParabolicCoeffs);
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
    const size_t nStateSize = initialState.size(); // for later use

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
    // A very small deltaX could be due to a floating point issue. So if deltaX is smaller than the epsilon, we assume
    // that it is zero. Use the same epsilon as what is used for piecewise polynomial checking.
    dReal tempDeltaX = finalState.at(positionIndex) - initialState.at(positionIndex);
    const dReal deltaX = RaveFabs(tempDeltaX) <= checker.GetEpsilonForPositionDiscrepancyChecking() ? 0 : tempDeltaX;

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
    PolynomialCheckReturn ret1, ret3;
    std::vector<dReal> newInitialState(nStateSize - 1, 0.0), newMidState(nStateSize - 1, 0.0), newFinalState(nStateSize - 1, 0.0); // nStateSize is guaranteed to be greater than 1.
    // The following newInitialState and newFinalState remain the same throughout, so initializing them here.
    newInitialState.assign(initialState.begin() + velocityIndex, initialState.end());
    newFinalState.assign(finalState.begin() + velocityIndex, finalState.end());

    std::vector<dReal> newLowerBounds(lowerBounds.size() - 1, 0.0), newUpperBounds(upperBounds.size() - 1, 0.0);
    newLowerBounds.assign(lowerBounds.begin() + velocityIndex, lowerBounds.end());
    newUpperBounds.assign(upperBounds.begin() + velocityIndex, upperBounds.end());

    dReal totalDuration = 0;
    dReal duration2 = 0;
    bool bSuccess = false;

    // The following values are added for use during the final attempt to solve the problem. See below for more details.
    dReal vLastPositiveRemainingDuration = vmax + 1; // the latest value of v computed in the loop that results in totalDuration being less than fixedDuration
    dReal remainingDuration = -1; // the value fixedDuration - totalDuration computed when v = vLastPositiveRemainingDuration
    dReal cachedDuration1 = -1;
    dReal cachedDuration3 = -1;

    for( size_t iter = 0; iter < maxIters; ++iter ) {
        // Step 5
        vLast = v;
        // Step 6
        v = 0.5*(vmin + vmax);

        // Step 7
        newMidState[0] = v;
        ret1 = Compute1DTrajectory(degree - 1, newInitialState, newMidState, newLowerBounds, newUpperBounds, /*fixedDuration*/ 0.0, pwpoly1);
        if( ret1 != PolynomialCheckReturn::PCR_Normal ) {
            return ret1;
        }

        // Step 8
        ret3 = Compute1DTrajectory(degree - 1, newMidState, newFinalState, newLowerBounds, newUpperBounds, /*fixedDuration*/ 0.0, pwpoly3);
        if( ret3 != PolynomialCheckReturn::PCR_Normal ) {
            return ret3;
        }

        // Step 9
        // deltaX1: displacement covered by segment I
        dReal deltaX1 = pwpoly1.Evali1(pwpoly1.GetDuration(), 0);
        // deltaX3: displacement covered by segment III
        dReal deltaX3 = pwpoly3.Evali1(pwpoly3.GetDuration(), 0);

        // Step 10
        // delta: displacement to be covered by segment II
        dReal delta = deltaX - deltaX1 - deltaX3;

        // Note: In the paper, there is no explicit consideration for the case when v is zero.
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

        // Record intermediate values for later use
        if( fixedDuration > 0 ) {
            if( totalDuration < fixedDuration ) {
                vLastPositiveRemainingDuration = v;
                remainingDuration = fixedDuration - totalDuration; // guaranteed to be positive
                cachedDuration1 = pwpoly1.GetDuration();
                cachedDuration3 = pwpoly3.GetDuration();
            }
        }

        // Step 12
        // Note: need a tighter bound when checking these velocities vmax, vmin. Otherwise, it might
        // not converge due to totalDuration not equal to fixedDuration.
        if( FuzzyEquals(vmax, vmin, epsilon) ) {
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
#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, successful: no duration constraint, so stopping.", envid);
#endif
                break; // successful
            }
            else if( FuzzyZero(v, g_fPolynomialEpsilon) && totalDuration <= fixedDuration + g_fEpsilonForTimeInstant ) {
                // v is zero so we are free to choose duration2. (We have not chosen a value for duration2 yet.)
                duration2 = Max(0, fixedDuration - totalDuration);
                bSuccess = true;
#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, successful: fixedDuration=%.15e; selected duration2=%.15e", envid%fixedDuration%duration2);
#endif
                break;
            }
            else if( FuzzyEquals(fixedDuration, totalDuration, g_fEpsilonForTimeInstant) ) {
                // The computed velocity converges and the total duration meets the given fixed duration.
                bSuccess = true;
#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, successful: totalDuration=%.15e converges to fixedDuration=%.15e, so stopping", envid%totalDuration%fixedDuration);
#endif
                break;
            }
            else {
#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, fixedDuration=%.15e; duration diff=%.15e", envid%fixedDuration%(fixedDuration - totalDuration));
#endif
                if( FuzzyEquals(vmax, vmin, epsilon) ) {
                    // Cannot do anything more since vmax and vmin are equal.
#ifdef GENERALINTERPOLATOR_PROGRESS_DEBUG
                    RAVELOG_DEBUG_FORMAT("env=%d, vmax=%.15e and vmin=%.15e are too close, so stopping", envid%vmax%vmin);
#endif

                    // Last attempt to solve the problem
                    if( fixedDuration > 0 ) {
                        // In some cases, the value of totalDuration is extremely sensitive to the value v such that in
                        // order for totalDuration be within the desired tolerance from fixedDuration, we would need to
                        // adjust v much more finely than epsilon. When this is the case, we will try to solve the
                        // problem differently.

                        // When the computed totalDuration is less than fixedDuration, instead of trying to adjust v
                        // further, we will try to increase the duration of Segment I and/or Segment III (while their
                        // boundary conditions remain untouched) so that the overall duration reaches fixedDuration.
                        // Currently, we will give 3 attempts (as described below).
                        if( !FuzzyEquals(vLastPositiveRemainingDuration, upperBounds.at(velocityIndex) + 1, epsilon) && remainingDuration > 0 && cachedDuration1 > 0 && cachedDuration3 > 0 ) {
                            newMidState[0] = vLastPositiveRemainingDuration;
                            // Attempt 1: trying to stretch out only Segment I
                            {
                                ret1 = Compute1DTrajectory(degree - 1, newInitialState, newMidState, newLowerBounds, newUpperBounds, cachedDuration1 + remainingDuration, pwpoly1);
                                if( ret1 == PolynomialCheckReturn::PCR_Normal ) {
                                    ret3 = Compute1DTrajectory(degree - 1, newMidState, newFinalState, newLowerBounds, newUpperBounds, cachedDuration3, pwpoly3);
                                    if( ret3 == PolynomialCheckReturn::PCR_Normal ) {
                                        bSuccess = true;
                                        duration2 = fixedDuration - (pwpoly1.GetDuration() + pwpoly3.GetDuration());
                                        v = vLastPositiveRemainingDuration;
                                        break;
                                    }
                                }
                            }
                            // Attempt 2: trying to stretch out only Segment III
                            {
                                ret1 = Compute1DTrajectory(degree - 1, newInitialState, newMidState, newLowerBounds, newUpperBounds, cachedDuration1, pwpoly1);
                                if( ret1 == PolynomialCheckReturn::PCR_Normal ) {
                                    ret3 = Compute1DTrajectory(degree - 1, newMidState, newFinalState, newLowerBounds, newUpperBounds, cachedDuration3 + remainingDuration, pwpoly3);
                                    if( ret3 == PolynomialCheckReturn::PCR_Normal ) {
                                        bSuccess = true;
                                        duration2 = fixedDuration - (pwpoly1.GetDuration() + pwpoly3.GetDuration());
                                        v = vLastPositiveRemainingDuration;
                                        break;
                                    }
                                }
                            }
                            // Attempt 3: trying to stretch out Segment I and Segment III equally
                            {
                                ret1 = Compute1DTrajectory(degree - 1, newInitialState, newMidState, newLowerBounds, newUpperBounds, cachedDuration1 + remainingDuration*0.5, pwpoly1);
                                if( ret1 == PolynomialCheckReturn::PCR_Normal ) {
                                    ret3 = Compute1DTrajectory(degree - 1, newMidState, newFinalState, newLowerBounds, newUpperBounds, cachedDuration3 + remainingDuration*0.5, pwpoly3);
                                    if( ret3 == PolynomialCheckReturn::PCR_Normal ) {
                                        bSuccess = true;
                                        duration2 = fixedDuration - (pwpoly1.GetDuration() + pwpoly3.GetDuration());
                                        v = vLastPositiveRemainingDuration;
                                        break;
                                    }
                                }
                            }
                        }
                    }
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

    const bool bHasMiddleSegment = !FuzzyZero(duration2, g_fEpsilonForTimeInstant);
    const size_t numPolynomials = pwpoly1.GetPolynomials().size() + pwpoly3.GetPolynomials().size() + (bHasMiddleSegment ? 1 : 0);
    std::vector<Polynomial>& vFinalPolynomials = _cacheFinalPolynomials;
    vFinalPolynomials.resize(0);
    vFinalPolynomials.reserve(numPolynomials);
    vFinalPolynomials.insert(vFinalPolynomials.end(), pwpoly1.GetPolynomials().begin(), pwpoly1.GetPolynomials().end());
    if( bHasMiddleSegment ) {
        Polynomial poly(duration2, {v});
        poly.PadCoefficients(degree - 1);
        vFinalPolynomials.emplace_back(poly);
    }
    vFinalPolynomials.insert(vFinalPolynomials.end(), pwpoly3.GetPolynomials().begin(), pwpoly3.GetPolynomials().end());
    PiecewisePolynomial& pwpolyFinal = _cacheFinalPWPoly;
    pwpolyFinal.Initialize(vFinalPolynomials);
    pwpoly = pwpolyFinal.Integrate(initialState[positionIndex]);
    return PolynomialCheckReturn::PCR_Normal; // Final piecewise polynomial is to be checked outside.
} // end Compute1DTrajectory

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
