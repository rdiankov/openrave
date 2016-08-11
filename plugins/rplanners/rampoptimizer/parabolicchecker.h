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
    PCR_Normal,
    PCR_Empty,
    PCR_NegativeDuration,
    PCR_XBoundViolated,
    PCR_VBoundViolated,
    PCR_ABoundViolated,
    PCR_VDiscontinuous,
    PCR_XDiscrepancy,
    PCR_VDiscrepancy,
    PCR_DurationDiscrepancy,
};

/*
   Checking function input order: the object to be checked, the bounds, and the boundary
   conditions. Every thing starts from the lowest order first, i.e., position.
 */

// Check if the Ramp violates the velocity and acceleration bounds. (Also check for non-negative
// duration.)
ParabolicCheckReturn CheckRamp(Ramp& ramp, dReal xmin, dReal xmax, dReal vm, dReal am);

// Check each Ramp in the vector as well as velocity continuity between consecutive ramps
ParabolicCheckReturn CheckRamps(std::vector<Ramp>& rampsVector, dReal xmin, dReal xmax, dReal vm, dReal am);

// Check a ParabolicCurve.
ParabolicCheckReturn CheckParabolicCurve(ParabolicCurve& curve, dReal xmin, dReal xmax, dReal vm, dReal am, dReal x0, dReal x1, dReal v0, dReal v1);

// Check a ParabolicCurvesND
ParabolicCheckReturn CheckParabolicCurvesND(ParabolicCurvesND& curvesnd, std::vector<dReal>& xminVect, std::vector<dReal>& xmaxVect, std::vector<dReal>& vmVect, std::vector<dReal>& amVect, std::vector<dReal>& x0Vect, std::vector<dReal>& x1Vect, std::vector<dReal>& v0Vect, std::vector<dReal>& v1Vect);

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
