// -*- coding: utf-8 -*-
// Copyright (C) 2016 Puttichai Lertkultanon <L.Puttichai@gmail.com>
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
#ifndef RAMP_OPTIM_INTERPOLATION_H
#define RAMP_OPTIM_INTERPOLATION_H

#include "ramp.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

/*
   Multi DOF interpolation
 */
/// Interpolate a trajectory which starts and stops at the two given waypoints. Path geometry remain
/// the same after the interpolation.
bool InterpolateZeroVelND(std::vector<dReal>& x0Vect, std::vector<dReal>& x1Vect, std::vector<dReal>& vmVect, std::vector<dReal>& amVect, ParabolicCurvesND& curvesndOut);

/// Interpolate a trajectory connecting (x0Vect, v0Vect) and (x1Vect, v1Vect).
bool InterpolateArbitraryVelND(std::vector<dReal>& x0Vect, std::vector<dReal>& x1Vect, std::vector<dReal>& v0Vect, std::vector<dReal>& v1Vect, std::vector<dReal>& xminVect, std::vector<dReal>& xmaxVect, std::vector<dReal>& vmVect, std::vector<dReal>& amVect, ParabolicCurvesND& curvesndOut, bool tryHarder);

/// Given a set of ParabolicCurves with different duration, try to stretch every curve to have the same duration.
bool ReinterpolateNDFixedDuration(std::vector<ParabolicCurve>& curvesVectIn, std::vector<dReal>& vmVect, std::vector<dReal>& amVect, size_t maxIndex, ParabolicCurvesND& curvesndOut, bool tryHarder);

/*
   Single DOF interpolation
 */
/// Interpolate a minimum-time 1D trajectory connecting (x0, v0) and (x1, v1).
bool Interpolate1D(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, ParabolicCurve& curveOut);

/// Interpolate a minimum-time 1D trajectory connecting (x0, v0) and (x1, v1), assuming infinite
/// velocity bounds.
bool Interpolate1DNoVelocityLimit(dReal x0, dReal x1, dReal v0, dReal v1, dReal am, ParabolicCurve& curveOut);

/// Given a minimum-time 1D trajectory, impose the given velocity bound on it. If the velocity bound
/// is violated, the modified trajectory will have a longer duration.
bool ImposeVelocityLimit(ParabolicCurve& curve, dReal vm);

/// Given a (not necessarily minimum-time) 1D trajectory, impose the given joint limits on it
/// while constraining the duration to be the same.
bool ImposeJointLimitFixedDuration(ParabolicCurve& curveIn, dReal xmin, dReal xmax, dReal vm, dReal am, ParabolicCurve& curveOut);

/// Stretch the given trajectory such that it ends at the given duration. It internally calls Interpolate1DFixedDuration.
bool Stretch1D(ParabolicCurve& curveIn, dReal vm, dReal am, dReal newDuration, ParabolicCurve& curveOut);

/// Interpolate a minimum-acceleration trajectory with the specified duration connecting (x0, v0) and (x1, v1).
bool Interpolate1DFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal duration, ParabolicCurve& curveOut);

/*
   Utilities
 */
/// Calculate the least upper bound of the inoperative interval(s), t, of the given
/// trajectory. The value t is such that stretching the trajectory with a duration of greater
/// than or equal to t will be successful.
bool CalculateLeastUpperBoundInoperativeInterval(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal& t);

/// Solve for a switchtime t0 \in [l, u]. For more detail, see the implementation.
bool SolveForT0(dReal A, dReal B, dReal t, dReal l, dReal u, dReal& t0);

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
