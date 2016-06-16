#ifndef PARABOLIC_CHECKER_H
#define PARABOLIC_CHECKER_H

#include "ramp.h"
#include "paraboliccommon.h"

namespace RampOptimizerInternal {

enum ParabolicCheckReturn {
    PCR_Normal,
    PCR_NegativeDuration,
    PCR_VBoundViolated,
    PCR_ABoundViolated,
    PCR_VDiscontinuous,
    PCR_XDiscrepancy,
    PCR_VDiscrepancy,
    PCR_DurationDiscrepancy,
};

// Check if the Ramp violates the velocity and acceleration bounds. (Also check for non-negative
// duration.)
ParabolicCheckReturn CheckRamp(Ramp& ramp, Real vm, Real am);

// Check each Ramp in the vector as well as velocity continuity between consecutive ramps
ParabolicCheckReturn CheckRamps(std::vector<Ramp>& rampsVector, Real vm, Real am);

// Check a ParabolicCurve.
ParabolicCheckReturn CheckParabolicCurve(ParabolicCurve& curve, Real vm, Real am, Real v0, Real v1, Real x0, Real x1);

// Check a ParabolicCurvesND
ParabolicCheckReturn CheckParabolicCurvesND(ParabolicCurvesND& curvesnd, std::vector<Real>& vmVect, std::vector<Real>& amVect, std::vector<Real>& v0Vect, std::vector<Real>& v1Vect, std::vector<Real>& x0Vect, std::vector<Real>& x1Vect);

} // end namespace RampOptimizerInternal
#endif
