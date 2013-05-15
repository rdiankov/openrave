// -*- coding: utf-8 -*-
// Copyright (C) 2013 Cuong Pham <cuong.pham@normalesup.org>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "ParabolicPathSmooth/ParabolicRamp.h"

namespace ParabolicRamp = ParabolicRampInternal;

namespace mergewaypoints
{

/** Iteratively merge all ramps that are shorter than minswitchtime. Determine the optimal time duration that allows to do so
    \param origramps input ramps
    \param ramps result ramps
    \param maxcoef maximum timescaling coefficient to try
    \param precision precision in the dichotomy search for the best timescaling coef
    \param iters max number of random iterations
 */
bool IterativeMergeRamps(const std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& resramps, ConstraintTrajectoryTimingParametersPtr params,  dReal maxcoef, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler);

/** Timescale a ramp. Assume the ramp is unitary.
    \param origramps input ramp
    \param resramps result ramp
    \param coef timescaling coefficient
 */
void ScaleRampTime(const std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& ramps,dReal coef);

/** Determine the minimum switchtime in a ramp
    \param rampnd input ramp
 */
dReal DetermineMinswitchtime(const ParabolicRamp::ParabolicRampND& rampnd);

/** Count the number of pieces in a ramp
    \param rampnd input ramp
 */
size_t CountUnitaryRamps(const ParabolicRamp::ParabolicRampND& rampnd);

void PrintRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps,ConstraintTrajectoryTimingParametersPtr params,bool warning);

} // end namespace mergewaypoints
