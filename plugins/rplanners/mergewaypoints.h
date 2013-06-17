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
#include "ParabolicPathSmooth/DynamicPath.h"

namespace ParabolicRamp = ParabolicRampInternal;

namespace mergewaypoints
{

/** Iteratively merge all ramps that are shorter than minswitchtime. Determine the optimal time duration that allows to do so
    \param origramps input ramps
    \param ramps result ramps
    \param upperbound maximum time scale to try
    \param precision precision in the dichotomy search for the best timescaling coef
    \param iters max number of random iterations
 */
bool IterativeMergeRamps(const std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, int options = 0xffff);

/** Once the ramps are all OK, further merge ramps
    \param origramps input ramps
    \param ramps result ramps
    \param upperbound maximum time scale to try
    \param precision precision in the dichotomy search for the best timescaling coef
    \param iters max number of random iterations
 */
bool FurtherMergeRamps(const std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, int options = 0xffff);

/** Same as IterativeMergeRamps but run a straightforward line search on the trajectory duration instead of dichotomy search
**/
bool IterativeMergeRampsNoDichotomy(const std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, dReal stepsize, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, int options = 0xffff);

/** If the beginning or the end of the ramps are linear segments then modify them to pass minswitchtime, controller timestep, and other constraints coming from the check object.
**/
bool FixRampsEnds(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& resramps, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check, int options = 0xffff);

/** Compute a straight ramp between x0 and x1, with initial and final velocities equal to zero. Assume that the straight path is collision free. Scale up time duration until the trajectory passes the dynamics check and satisfies minswitchtime and fStepLength conditions
    \param newramp the resulting ramp
    \param x0, x1 initial and final joint values
    \param params planner parameters
    \param check checker for collision and dynamics
 **/
bool ComputeLinearRampsWithConstraints(std::list<ParabolicRamp::ParabolicRampND>& resramps, const ParabolicRamp::Vector x0, const ParabolicRamp::Vector x1, ConstraintTrajectoryTimingParametersPtr params,ParabolicRamp::RampFeasibilityChecker& check, int options = 0xffff);

/// \param fOriginalTrajectorySegmentTime time duration of the original trajectory segment that will be shortcutted
/// \param x0, x1, dx0, dx1 shortcutted trajectory segment endpoints
bool ComputeQuadraticRampsWithConstraints(std::list<ParabolicRamp::ParabolicRampND>& resramps, const ParabolicRamp::Vector x0, const ParabolicRamp::Vector dx0, const ParabolicRamp::Vector x1, const ParabolicRamp::Vector dx1, dReal fOriginalTrajectorySegmentTime, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check, int options = 0xffff);

/** Timescale a ramp. Assume the ramp is unitary.
    \param origramps input ramp
    \param resramps result ramp
    \param coef timescaling coefficient
    \param trysmart if false, modify all the ramps by the constant factor. if true, then modify ramps whos modified field is true.
    \param flag
 */
bool ScaleRampsTime(const std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& ramps,dReal coef,bool trysmart, ConstraintTrajectoryTimingParametersPtr params);

/** Determine the minimum switchtime in a ramp
    \param rampnd input ramp
 */
dReal DetermineMinswitchtime(const ParabolicRamp::ParabolicRampND& rampnd);
dReal DetermineMinswitchtime(const std::list<ParabolicRamp::ParabolicRampND>& ramps);

/** Compute time duration of ramps
    \param rampnd input ramp
 */
dReal ComputeRampsDuration(const std::list<ParabolicRamp::ParabolicRampND>& ramps);


/** Count the number of pieces in a ramp
    \param rampnd input ramp
 */
size_t CountUnitaryRamps(const ParabolicRamp::ParabolicRampND& rampnd);

size_t CountUnitaryRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps);

void PrintRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps,ConstraintTrajectoryTimingParametersPtr params,bool warning);

/** Break ramps into unitary ramps (in place)
    \param ramps the ramps to be broken
 */
void BreakIntoUnitaryRamps(std::list<ParabolicRamp::ParabolicRampND>& ramps);

/// check if all numbers in the vector are zero
bool CheckIfZero(const ParabolicRamp::Vector& v, dReal epsilon=g_fEpsilonLinear);

// Provides a measure of quality of a ramps
// Now set to the 1/sum(1/rampduration^2) toi penalize small ramps
dReal ComputeRampQuality(const std::list<ParabolicRamp::ParabolicRampND>& ramps);

// The parts of the ramps that are very close to qstart and qgoal are not checked with perturbations
bool SpecialCheckRamp(const ParabolicRamp::ParabolicRampND& ramp,const ParabolicRamp::Vector& qstart, const ParabolicRamp::Vector& qgoal, dReal radius, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check, int options);


// Special treatment for the first and last ramps of a traj
// The initial part of first ramp and the final part of last ramp are checked without perturbation
// position=1 : first ramp, position=-1 : last ramp
/* bool SpecialCheckRamp(const ParabolicRamp::ParabolicRampND& ramp, int position, ParabolicRamp::RampFeasibilityChecker& check, int options); */
/* bool SpecialCheckRamp(const ParabolicRamp::ParabolicRampND& ramp, int position, ParabolicRamp::Vector& qref, dReal radius, ParabolicRamp::RampFeasibilityChecker& check, int options); */

/* dReal dist(const ParabolicRamp::Vector& q1, const ParabolicRamp::Vector& q2); */
} // end namespace mergewaypoints
