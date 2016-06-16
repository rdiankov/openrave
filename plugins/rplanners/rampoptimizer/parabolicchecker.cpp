//-*- coding: utf-8 -*-
#include "parabolicchecker.h"

namespace RampOptimizerInternal {

ParabolicCheckReturn CheckRamp(Ramp& ramp, Real vm, Real am) {
    if (ramp.duration < -epsilon) {
        return PCR_NegativeDuration;
    }
    if ((Abs(ramp.v0) > vm + epsilon)or (Abs(ramp.v1) > vm + epsilon)) {
        return PCR_VBoundViolated;
    }
    if (Abs(ramp.a) > am + epsilon) {
        return PCR_ABoundViolated;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckRamps(std::vector<Ramp>& rampsVector, Real vm, Real am) {
    ParabolicCheckReturn ret = CheckRamp(rampsVector[0], vm, am);
    if (ret != PCR_Normal) {
        return ret;
    }
    for (size_t i = 1; i < rampsVector.size(); ++i) {
        if (!FuzzyEquals(rampsVector[i - 1].v1, rampsVector[i].v0, epsilon)) {
            return PCR_VDiscontinuous;
        }
        ret = CheckRamp(rampsVector[i], vm, am);
        if (ret != PCR_Normal) {
            return ret;
        }
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckParabolicCurve(ParabolicCurve& curve, Real vm, Real am, Real v0, Real v1, Real x0, Real x1) {
    ParabolicCheckReturn ret = CheckRamps(curve.ramps, vm, am);
    if (ret != PCR_Normal) {
        return ret;
    }
    // Check boundary conditions
    if (!FuzzyEquals(curve.v0, curve.ramps.front().v0, epsilon)) {
        return PCR_VDiscrepancy;
    }
    if (!FuzzyEquals(curve.v0, v0, epsilon)) {
        return PCR_VDiscrepancy;
    }
    if (!FuzzyEquals(curve.v1, curve.ramps.back().v1, epsilon)) {
        return PCR_VDiscrepancy;
    }
    if (!FuzzyEquals(curve.v1, v1, epsilon)) {
        return PCR_VDiscrepancy;
    }
    if (!FuzzyEquals(curve.x0, curve.ramps.front().x0, epsilon)) {
        return PCR_XDiscrepancy;
    }
    if (!FuzzyEquals(curve.x0, x0, epsilon)) {
        return PCR_XDiscrepancy;
    }
    if (!FuzzyEquals(curve.EvalPos(curve.duration), x1, epsilon)) {
        return PCR_XDiscrepancy;
    }
    if (!FuzzyEquals(curve.d, x1 - x0, epsilon)) {
        return PCR_XDiscrepancy;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckParabolicCurvesND(ParabolicCurvesND& curvesnd, std::vector<Real>& vmVect, std::vector<Real>& amVect,
                           std::vector<Real>& v0Vect, std::vector<Real>& v1Vect,
                           std::vector<Real>& x0Vect, std::vector<Real>& x1Vect) {
    ParabolicCheckReturn ret;
    for (int i = 0; i < curvesnd.ndof; ++i) {
        ret = CheckParabolicCurve(curvesnd.curves[i], vmVect[i], amVect[i], v0Vect[i], v1Vect[i], x0Vect[i], x1Vect[i]);
        if (ret != PCR_Normal) {
            return ret;
        }
        if (!FuzzyEquals(curvesnd.duration, curvesnd.curves[i].duration, epsilon)) {
            return PCR_DurationDiscrepancy;
        }
    }
    return PCR_Normal;
}

} // end namespace RampOptimizerInternal
