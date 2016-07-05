#ifndef PARABOLIC_CHECKER_H
#define PARABOLIC_CHECKER_H

#include "ramp.h"
#include "paraboliccommon.h"

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
   Checking function input order: the object to be checked, the bounds, and the boundary conditions
 */

// Check if the Ramp violates the velocity and acceleration bounds. (Also check for non-negative
// duration.)
ParabolicCheckReturn CheckRamp(Ramp& ramp, Real xmin, Real xmax, Real vm, Real am);

// Check each Ramp in the vector as well as velocity continuity between consecutive ramps
ParabolicCheckReturn CheckRamps(std::vector<Ramp>& rampsVector, Real xmin, Real xmax, Real vm, Real am);

// Check a ParabolicCurve.
ParabolicCheckReturn CheckParabolicCurve(ParabolicCurve& curve, Real xmin, Real xmax, Real vm, Real am, Real x0, Real x1, Real v0, Real v1);

// Check a ParabolicCurvesND
ParabolicCheckReturn CheckParabolicCurvesND(ParabolicCurvesND& curvesnd, std::vector<Real>& xminVect, std::vector<Real>& xmaxVect, std::vector<Real>& vmVect, std::vector<Real>& amVect, std::vector<Real>& x0Vect, std::vector<Real>& x1Vect, std::vector<Real>& v0Vect, std::vector<Real>& v1Vect);

} // end namespace RampOptimizerInternal
#endif
