// -*- coding: utf-8 -*-
// Copyright (C) 2016 Puttichai Lertkultanon & Rosen Diankov
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
#ifndef RAMP_OPTIM_PARABOLIC_CHECKER_H
#define RAMP_OPTIM_PARABOLIC_CHECKER_H

#include "ramp.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

enum ParabolicCheckReturn {
    PCR_Normal = 0,
    PCR_NegativeDuration = 1,
    PCR_XBoundViolated = 2,
    PCR_VBoundViolated = 3,
    PCR_ABoundViolated = 4,
    PCR_XDiscrepancy = 5,
    PCR_VDiscrepancy = 6,
    PCR_DurationDiscrepancy = 7,
};

/// \brief Calculate the maximum and minimum displacement occuring betweeb time 0 and t given (x0, x1, v0, v1, a)
void _GetPeaks(dReal x0, dReal x1, dReal v0, dReal v1, dReal a, dReal t, dReal& bmin, dReal& bmax);

/// \brief Check if all input values are consistent
ParabolicCheckReturn CheckSegment(dReal x0, dReal x1, dReal v0, dReal v1, dReal a, dReal t, dReal xmin, dReal xmax, dReal vm, dReal am);

/// \brief Check if the ramp violates joint limits, velocity constraint, or acceleration
/// constraint. Also check if the duration is negative.
ParabolicCheckReturn CheckRamp(const Ramp& ramp, dReal xmin, dReal xmax, dReal vm, dReal am);

/// \brief Call CheckRamp to check each ramp and check velocity continuity between consecutive ramps
ParabolicCheckReturn CheckRamps(const std::vector<Ramp>& ramps, dReal xmin, dReal xmax, dReal vm, dReal am, dReal x0, dReal x1, dReal v0, dReal v1);

/// \brief Check a RampND
ParabolicCheckReturn CheckRampND(const RampND& rampnd, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect);

/// \brief Check a vector of RampNDs
ParabolicCheckReturn CheckRampNDs(const std::vector<RampND>& rampnds, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect);

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
