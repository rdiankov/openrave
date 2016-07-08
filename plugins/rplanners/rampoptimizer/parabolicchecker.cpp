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
#include "parabolicchecker.h"

namespace RampOptimizerInternal {

ParabolicCheckReturn CheckRamp(Ramp& ramp, Real xmin, Real xmax, Real vm, Real am) {
    if (ramp.duration < -epsilon) {
        return PCR_NegativeDuration;
    }
    Real bmin, bmax;
    ramp.GetPeaks(bmin, bmax);
    if ((bmin < xmin - epsilon) || (bmax > xmax + epsilon)) {
        return PCR_XBoundViolated;
    }
    if ((Abs(ramp.v0) > vm + epsilon) || (Abs(ramp.v1) > vm + epsilon)) {
        return PCR_VBoundViolated;
    }
    if (Abs(ramp.a) > am + epsilon) {
        return PCR_ABoundViolated;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckRamps(std::vector<Ramp>& rampsVector, Real xmin, Real xmax, Real vm, Real am) {
    ParabolicCheckReturn ret = CheckRamp(rampsVector[0], xmin, xmax, vm, am);
    if (ret != PCR_Normal) {
        return ret;
    }

    size_t rampsSize = rampsVector.size();
    for (size_t i = 1; i < rampsSize; ++i) {
        if (!(FuzzyEquals(rampsVector[i - 1].v1, rampsVector[i].v0, epsilon))) {
            return PCR_VDiscontinuous;
        }
        ret = CheckRamp(rampsVector[i], xmin, xmax, vm, am);
        if (ret != PCR_Normal) {
            return ret;
        }
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckParabolicCurve(ParabolicCurve& curve, Real xmin, Real xmax, Real vm, Real am, Real x0, Real x1, Real v0, Real v1) {
    ParabolicCheckReturn ret = CheckRamps(curve.ramps, xmin, xmax, vm, am);
    if (ret != PCR_Normal) {
        return ret;
    }
    if (!(FuzzyEquals(curve.v0, curve.ramps[0].v0, epsilon))) {
        return PCR_VDiscrepancy;
    }
    if (!(FuzzyEquals(curve.v0, v0, epsilon))) {
        return PCR_VDiscrepancy;
    }
    if (!(FuzzyEquals(curve.v1, curve.ramps[curve.ramps.size() - 1].v1, epsilon))) {
        return PCR_VDiscrepancy;
    }
    if (!(FuzzyEquals(curve.v1, v1, epsilon))) {
        return PCR_VDiscrepancy;
    }
    if (!(FuzzyEquals(curve.x0, curve.ramps[0].x0, epsilon))) {
        return PCR_XDiscrepancy;
    }
    if (!(FuzzyEquals(curve.x0, x0, epsilon))) {
        return PCR_XDiscrepancy;
    }
    if (!(FuzzyEquals(curve.EvalPos(curve.duration), x1, epsilon))) {
        return PCR_XDiscrepancy;
    }
    if (!(FuzzyEquals(curve.d, x1 - x0, epsilon))) {
        return PCR_XDiscrepancy;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckParabolicCurvesND(ParabolicCurvesND& curvesnd, std::vector<Real>& xminVect, std::vector<Real>& xmaxVect, std::vector<Real>& vmVect, std::vector<Real>& amVect,
                                            std::vector<Real>& x0Vect, std::vector<Real>& x1Vect, std::vector<Real>& v0Vect, std::vector<Real>& v1Vect) {
    ParabolicCheckReturn ret;
    for (int i = 0; i < curvesnd.ndof; ++i) {
        ret = CheckParabolicCurve(curvesnd.curves[i], xminVect[i], xmaxVect[i], vmVect[i], amVect[i], x0Vect[i], x1Vect[i], v0Vect[i], v1Vect[i]);
        if (ret != PCR_Normal) {
            return ret;
        }
        if (!(FuzzyEquals(curvesnd.curves[i].duration, curvesnd.duration, epsilon))) {
            return PCR_DurationDiscrepancy;
        }
    }
    return PCR_Normal;
}


} // end namespace RampOptimizerInternal
