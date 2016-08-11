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
#include "parabolicchecker.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

ParabolicCheckReturn CheckRamp(Ramp& ramp, dReal xmin, dReal xmax, dReal vm, dReal am) {
    if (ramp.duration < -epsilon) {
        RAMP_OPTIM_WARN("PCR_NegativeDuration: ramp.duration = %.15e", ramp.duration);
        return PCR_NegativeDuration;
    }
    dReal bmin, bmax;
    ramp.GetPeaks(bmin, bmax);
    if ((bmin < xmin - epsilon) || (bmax > xmax + epsilon)) {
        RAMP_OPTIM_WARN("PCR_XBoundViolated: bmin = %.15e; xmin = %.15e; bmax = %.15e; xmax = %.15e", bmin, xmin, bmax, xmax);
        return PCR_XBoundViolated;
    }
    if ((Abs(ramp.v0) > vm + epsilon) || (Abs(ramp.v1) > vm + epsilon)) {
        RAMP_OPTIM_WARN("PCR_VBoundViolated: ramp.v0 = %.15e; ramp.v1 = %.15e; vm = %.15e", ramp.v0, ramp.v1, vm);
        return PCR_VBoundViolated;
    }
    if (Abs(ramp.a) > am + epsilon) {
        RAMP_OPTIM_WARN("PCR_ABoundViolated: ramp.a = %.15e; am = %.15e", ramp.a, am);
        return PCR_ABoundViolated;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckRamps(std::vector<Ramp>& rampsVector, dReal xmin, dReal xmax, dReal vm, dReal am) {
    ParabolicCheckReturn ret = CheckRamp(rampsVector[0], xmin, xmax, vm, am);
    if (ret != PCR_Normal) {
        return ret;
    }

    size_t rampsSize = rampsVector.size();
    for (size_t i = 1; i < rampsSize; ++i) {
        if (!(FuzzyEquals(rampsVector[i - 1].v1, rampsVector[i].v0, epsilon))) {
            RAMP_OPTIM_WARN("PCR_VDiscontinuous: ramps[%d].v1 (%.15e) != ramps[%d].v0 (%.15e); diff = %.15e", (i - 1), rampsVector[i - 1].v1, i, rampsVector[i].v0, (rampsVector[i - 1].v1 - rampsVector[i].v0));
            return PCR_VDiscontinuous;
        }
        ret = CheckRamp(rampsVector[i], xmin, xmax, vm, am);
        if (ret != PCR_Normal) {
            return ret;
        }
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckParabolicCurve(ParabolicCurve& curve, dReal xmin, dReal xmax, dReal vm, dReal am, dReal x0, dReal x1, dReal v0, dReal v1) {
    ParabolicCheckReturn ret = CheckRamps(curve.ramps, xmin, xmax, vm, am);
    if (ret != PCR_Normal) {
        return ret;
    }
    if (!(FuzzyEquals(curve.v0, curve.ramps[0].v0, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.v0 (%.15e) != curve.ramps[0].v0 (%.15e); diff = %.15e", curve.v0, curve.ramps[0].v0, (curve.v0 - curve.ramps[0].v0));
        return PCR_VDiscrepancy;
    }
    if (!(FuzzyEquals(curve.v0, v0, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.v0 (%.15e) != v0 (%.15e); diff = %.15e", curve.v0, v0, (curve.v0 - v0));
        return PCR_VDiscrepancy;
    }
    if (!(FuzzyEquals(curve.v1, curve.ramps[curve.ramps.size() - 1].v1, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.v1 (%.15e) != curve.ramps[-1].v1 (%.15e); diff = %.15e", curve.v1, curve.ramps[curve.ramps.size() - 1].v1, (curve.v1 - curve.ramps[curve.ramps.size() - 1].v1));
        return PCR_VDiscrepancy;
    }
    if (!(FuzzyEquals(curve.v1, v1, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.v1 (%.15e) != v1 (%.15e); diff = %.15e", curve.v1, v1, (curve.v1 - v1));
        return PCR_VDiscrepancy;
    }
    if (!(FuzzyEquals(curve.x0, curve.ramps[0].x0, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.x0 (%.15e) != curve.ramps[0].x0 (%.15e); diff = %.15e", curve.x0, curve.ramps[0].x0, (curve.x0 - curve.ramps[0].x0));
        return PCR_XDiscrepancy;
    }
    if (!(FuzzyEquals(curve.x0, x0, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.x0 (%.15e) != x0 (%.15e); diff = %.15e", curve.x0, x0, (curve.x0 - x0));
        return PCR_XDiscrepancy;
    }
    if (!(FuzzyEquals(curve.x1, curve.ramps[curve.ramps.size() - 1].x1, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.x1 (%.15e) != curve.ramps[-1].x1 (%.15e); diff = %.15e", curve.x1, curve.ramps[curve.ramps.size() - 1].x1, (curve.x1 - curve.ramps[curve.ramps.size() - 1].x1));
        return PCR_XDiscrepancy;
    }
    if (!(FuzzyEquals(curve.x1, x1, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.x1 (%.15e) != x1 (%.15e); diff = %.15e", curve.x1, x1, (curve.x1 - x1));
        return PCR_XDiscrepancy;
    }
    if (!(FuzzyEquals(curve.d, x1 - x0, epsilon))) {
        RAMP_OPTIM_WARN("PCR_VDiscrepancy: curve.d (%.15e) != x1 - x0 (%.15e); diff = %.15e", curve.d, (x1 - x0), (curve.d - (x1 - x0)));
        return PCR_XDiscrepancy;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckParabolicCurvesND(ParabolicCurvesND& curvesnd, std::vector<dReal>& xminVect, std::vector<dReal>& xmaxVect, std::vector<dReal>& vmVect, std::vector<dReal>& amVect, std::vector<dReal>& x0Vect, std::vector<dReal>& x1Vect, std::vector<dReal>& v0Vect, std::vector<dReal>& v1Vect) {
    ParabolicCheckReturn ret;
    for (size_t i = 0; i < curvesnd.ndof; ++i) {
        ret = CheckParabolicCurve(curvesnd.curves[i], xminVect[i], xmaxVect[i], vmVect[i], amVect[i], x0Vect[i], x1Vect[i], v0Vect[i], v1Vect[i]);
        if (ret != PCR_Normal) {
            return ret;
        }
        if (!(FuzzyEquals(curvesnd.curves[i].duration, curvesnd.duration, epsilon))) {
            RAMP_OPTIM_WARN("PCR_DurationDiscrepancy: curvesnd.curves[%d].duration (%.15e) != curvesnd.duration (%.15e); diff = %.15e", i, curvesnd.curves[i].duration, curvesnd.duration, (curvesnd.curves[i].duration - curvesnd.duration));
            return PCR_DurationDiscrepancy;
        }
    }
    return PCR_Normal;
}


} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
