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
#include "interpolation.h"
#include "parabolicchecker.h"
#include <math.h>
#include <openrave/mathextra.h>

namespace OpenRAVE {

namespace RampOptimizerInternal {

/*
   Multi DOF interpolation
 */
bool InterpolateZeroVelND(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, ParabolicCurvesND& curvesndOut) {
    size_t ndof = x0Vect.size();
    BOOST_ASSERT(ndof == x1Vect.size());
    BOOST_ASSERT(ndof == vmVect.size());
    BOOST_ASSERT(ndof == amVect.size());

    std::vector<dReal> dVect(ndof);
    LinearCombination(1, x1Vect, -1, x0Vect, dVect);

    // Calculate sdMax (vMin) and sddMax (aMin)
    dReal vMin = inf;
    dReal aMin = inf;
    for (size_t i = 0; i < ndof; ++i) {
        if (!FuzzyZero(dVect[i], epsilon)) {
            vMin = Min(vMin, vmVect[i]/Abs(dVect[i]));
            aMin = Min(aMin, amVect[i]/Abs(dVect[i]));
        }
    }
    BOOST_ASSERT(vMin < inf);
    BOOST_ASSERT(aMin < inf);

    ParabolicCurve sdProfile;
    bool result = Interpolate1D(0, 1, 0, 0, vMin, aMin, sdProfile);
    if (!result) {
        return false;
    }

    std::vector<ParabolicCurve> curves(ndof);

    // Scale each input dof according to the obtained sd-profile
    for (std::vector<Ramp>::const_iterator itRamp = sdProfile.ramps.begin(); itRamp != sdProfile.ramps.end(); ++itRamp) {
        dReal sd0, sdd;
        dReal dur = itRamp->duration;
        for (size_t i = 0; i < ndof; ++i) {
            sd0 = dVect[i] * (itRamp->v0);
            sdd = dVect[i] * (itRamp->a);
            Ramp ramp(sd0, sdd, dur);
            std::vector<Ramp> ramps(1);
            ramps[0] = ramp;
            ParabolicCurve curve(ramps);
            curves[i].Append(curve);
        }
    }

    for (size_t i = 0; i < ndof; ++i) {
        curves[i].SetInitialValue(x0Vect[i]);
    }
    curvesndOut.Initialize(curves);
    return true;
}


bool InterpolateArbitraryVelND(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, ParabolicCurvesND& curvesndOut, bool tryHarder) {
    size_t ndof = x0Vect.size();
    BOOST_ASSERT(ndof == x1Vect.size());
    BOOST_ASSERT(ndof == v0Vect.size());
    BOOST_ASSERT(ndof == v1Vect.size());
    BOOST_ASSERT(ndof == vmVect.size());
    BOOST_ASSERT(ndof == amVect.size());

    std::vector<dReal> dVect(ndof);
    LinearCombination(1, x1Vect, -1, x0Vect, dVect);

    // First independently interpolate each DOF to find the slowest DOF
    std::vector<ParabolicCurve> curves(ndof);
    dReal maxDuration = 0;
    size_t maxIndex;
    bool result;
    for (size_t i = 0; i < ndof; ++i) {
        result = Interpolate1D(x0Vect[i], x1Vect[i], v0Vect[i], v1Vect[i], vmVect[i], amVect[i], curves[i]);
        if (!result) {
            return false;
        }
        if (curves[i].duration > maxDuration) {
            maxDuration = curves[i].duration;
            maxIndex = i;
        }
    }

    ParabolicCurvesND tempcurvesnd;

    RAMP_OPTIM_PLOG("joint %d has the longest duration of %.15e", maxIndex, maxDuration);
    result = ReinterpolateNDFixedDuration(curves, vmVect, amVect, maxIndex, tempcurvesnd, tryHarder);
    if (!result) {
        if (tryHarder) {
            RAMP_OPTIM_WARN("Interpolation with a fixed duration failed even when trying harder.");
        }
        return false;
    }
    RAMP_OPTIM_PLOG("Interpolation with a fixed duration successful");

    std::vector<ParabolicCurve> newCurves(ndof);
    for (size_t i = 0; i < ndof; ++i) {
        result = ImposeJointLimitFixedDuration(tempcurvesnd.curves[i], xminVect[i], xmaxVect[i], vmVect[i], amVect[i], newCurves[i]);
        if (!result) {
            return false;
        }
    }
    RAMP_OPTIM_PLOG("Imposing joint limits successful");

    curvesndOut.Initialize(newCurves);
    return true;
}


bool ReinterpolateNDFixedDuration(std::vector<ParabolicCurve>& curvesVectIn, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, size_t maxIndex, ParabolicCurvesND& curvesndOut, bool tryHarder) {
    size_t ndof = curvesVectIn.size();
    RAMP_OPTIM_ASSERT(ndof == vmVect.size());
    RAMP_OPTIM_ASSERT(ndof == amVect.size());
    RAMP_OPTIM_ASSERT(maxIndex < ndof);

    std::vector<ParabolicCurve> newCurves(ndof);
    if (!tryHarder) {
        dReal newDuration = curvesVectIn[maxIndex].duration;
        for (size_t i = 0; i < ndof; ++i) {
            if (i == maxIndex) {
                RAMP_OPTIM_PLOG("joint %d is already the slowest DOF, continue to the next DOF", i);
                newCurves[i] = curvesVectIn[i];
                continue;
            }

            ParabolicCurve tempCurve;
            bool result = Stretch1D(curvesVectIn[i], vmVect[i], amVect[i], newDuration, tempCurve);
            if (!result) {
                return false;
            }
            else {
                newCurves[i] = tempCurve;
            }
        }
        curvesndOut.Initialize(newCurves);
        return true;
    }
    else {
        bool isPrevDurationSafe = false;
        bool result;
        dReal newDuration = 0;

        for (size_t i = 0; i < ndof; ++i) {
            dReal t;
            result = CalculateLeastUpperBoundInoperativeInterval(curvesVectIn[i].x0, curvesVectIn[i].EvalPos(curvesVectIn[i].duration), curvesVectIn[i].v0, curvesVectIn[i].v1, vmVect[i], amVect[i], t);
            if (!result) {
                RAMP_OPTIM_PLOG("Calculating the least upper bound of inoperative intervals failed.");
                return false;
            }
            if (t > newDuration) {
                newDuration = t;
            }
        }
        RAMP_OPTIM_ASSERT(newDuration > 0);
        if (curvesVectIn[maxIndex].duration > newDuration) {
            newDuration = curvesVectIn[maxIndex].duration;
            isPrevDurationSafe = true;
        }
        else {
            RAMP_OPTIM_PLOG("The total duration has been changed to %.15e s.", newDuration);
        }

        for (size_t i = 0; i < ndof; ++i) {
            if (isPrevDurationSafe && (i == maxIndex)) {
                RAMP_OPTIM_PLOG("joint %d is already the slowest DOF, continue to the next DOF", i);
                newCurves[i] = curvesVectIn[i];
                continue;
            }

            ParabolicCurve tempCurve;
            bool result = Stretch1D(curvesVectIn[i], vmVect[i], amVect[i], newDuration, tempCurve);
            if (!result) {
                return false;
            }
            else {
                newCurves[i] = tempCurve;
            }
        }
        curvesndOut.Initialize(newCurves);
        return true;
    }
}

bool InterpolateNDFixedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, dReal duration, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, ParabolicCurvesND& curvesndOut) {
    RAMP_OPTIM_ASSERT(duration > 0);

    size_t ndof = x0Vect.size();
    BOOST_ASSERT(ndof == x1Vect.size());
    BOOST_ASSERT(ndof == v0Vect.size());
    BOOST_ASSERT(ndof == v1Vect.size());
    BOOST_ASSERT(ndof == vmVect.size());
    BOOST_ASSERT(ndof == amVect.size());

    ParabolicCurve tempCurve;
    std::vector<ParabolicCurve> curves(ndof);
    bool result;
    for (size_t idof = 0; idof < ndof; ++idof) {
        result = Interpolate1DFixedDuration(x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], vmVect[idof], amVect[idof], duration, tempCurve);
        if (!result) {
            return false;
        }
        result = ImposeJointLimitFixedDuration(tempCurve, xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], curves[idof]);
        if (!result) {
            return false;
        }
    }

    curvesndOut.Initialize(curves);
    return true;
}

/*
   single DOF interpolation
 */
bool Interpolate1D(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, ParabolicCurve& curveOut) {
    RAMP_OPTIM_ASSERT(vm > 0);
    RAMP_OPTIM_ASSERT(am > 0);
    RAMP_OPTIM_ASSERT(Abs(v0) <= vm + epsilon);
    RAMP_OPTIM_ASSERT(Abs(v1) <= vm + epsilon);

    bool result;
    result = Interpolate1DNoVelocityLimit(x0, x1, v0, v1, am, curveOut);
    if (!result) {
        return false;
    }
    if (curveOut.ramps.size() == 1) {
        // No more fixing required here
        return true;
    }

    result = ImposeVelocityLimit(curveOut, vm);
    if (!result) {
        RAMP_OPTIM_WARN("Cannot impose the given velocity limit to the trajectory");
        return false;
    }
    else {
        return true;
    }
}


bool Interpolate1DNoVelocityLimit(dReal x0, dReal x1, dReal v0, dReal v1, dReal am, ParabolicCurve& curveOut) {
    RAMP_OPTIM_ASSERT(am > 0);

    dReal d = x1 - x0;
    dReal dv = v1 - v0;
    dReal v0sqr = v0*v0;
    dReal v1sqr = v1*v1;
    dReal difVSqr = v1sqr - v0sqr;

    dReal dStraight; // displacement obtained when maximally accelerate/decelerate from v0 to v1
    if (Abs(dv) == 0) {
        if (Abs(d) == 0) {
            Ramp ramp0(0, 0, 0, x0);
            std::vector<Ramp> ramps(1);
            ramps[0] = ramp0;
            curveOut.Initialize(ramps);
            return true;
        }
        else {
            dStraight = 0;
        }
    }
    else {
        if (dv > 0) {
            dStraight = difVSqr / (2*am);
        }
        else {
            dStraight = -difVSqr / (2*am);
        }
    }

    if (FuzzyEquals(d, dStraight, epsilon)) {
        // We can maximally accelerate/decelerate from v0 to v1.
        /*
           Consider for example the case when deltad = d - dStraight > 0 and deltad < epsilon. Since
           we are estimating d to be dStraight instead, the computed x1 will be off by deltad (which
           is in turn bounded by epsilon).

           Let's consider the case when v1 > v0 > 0. If we are using the real value d instead, it is
           equivalent to adding two ramps at the end. Each extra ramp has duration deltat / 2, the
           first one going up from v1 to v' = v1 + am*deltat/2, the second one going down from v' to
           v1. From deltad that we have, we can calculate the resulting deltat:

                  deltad = v1*deltat + 0.25*am*deltat**2.

           Therefore, deltat will be in the magnitude of around 2*|log10(epsilon)| which is really
           too small.
         */
        dReal a0 = dv > 0 ? am : -am;
        Ramp ramp0(v0, a0, dv/a0, x0);
        std::vector<Ramp> ramps(1);
        ramps[0] = ramp0;
        curveOut.Initialize(ramps);
        ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, inf, am, x0, x1, v0, v1);
        if (ret == PCR_Normal) {
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; am = %.15e", x0, x1, v0, v1, am);
            return false;
        }
        return true;
    }

    dReal sumVSqr = v0sqr + v1sqr;
    dReal sigma = d - dStraight > 0 ? 1 : -1;
    dReal a0 = sigma*am;
    dReal vp = sigma*Sqrt((0.5*sumVSqr) + (a0*d));
    dReal a0inv = 1/a0;
    dReal t0 = (vp - v0)*a0inv;
    dReal t1 = (vp - v1)*a0inv;

    Ramp ramp0(v0, a0, t0, x0);
    Ramp ramp1(ramp0.v1, -a0, t1);

    std::vector<Ramp> ramps(2);
    ramps[0] = ramp0;
    ramps[1] = ramp1;
    curveOut.Initialize(ramps);

    ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf,  inf, inf, am, x0, x1, v0, v1);
    if (ret == PCR_Normal) {
        RAMP_OPTIM_PLOG("Interpolate1D with no velocity bound successful");
        return true;
    }
    else {
        RAMP_OPTIM_PLOG("Interpolate1D with no velocity bound failed: CheckParabolicCurve returns %d", ret);
        RAMP_OPTIM_PLOG("Boundary conditions: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; am = %.15e; (computed) duration = %.15e", x0, x1, v0, v1, am, curveOut.duration);
        return false;
    }
}


bool ImposeVelocityLimit(ParabolicCurve& curve, dReal vm) {
    RAMP_OPTIM_ASSERT(vm > 0);
    RAMP_OPTIM_ASSERT(curve.ramps.size() == 2);

    if (Abs(curve.ramps[0].v0) > vm + epsilon) {
        // Initial velocity violates the velocity constraint
        return false;
    }
    if (Abs(curve.ramps[1].v1) > vm + epsilon) {
        // Final velocity violates the velocity constraint
        return false;
    }

    dReal vp = curve.ramps[0].v1;
    if (Abs(vp) <= vm + epsilon) {
        // The initial curve does not violate the constraint
        RAMP_OPTIM_PLOG("The initial ParabolicCurve does not violate the velocity constraint");
        return true;
    }

    dReal a0 = curve.ramps[0].a;
    dReal h = Abs(vp) - vm;
    dReal t = h/Abs(a0); // we are sure that a0 is non-zero

    std::vector<Ramp> ramps(3);
    Ramp newRamp0(curve.ramps[0].v0, curve.ramps[0].a, curve.ramps[0].duration - t, curve.ramps[0].x0);
    ramps[0] = newRamp0;

    dReal nom = h*h;
    dReal denom = Abs(a0)*vm;
    dReal newVp = Sign(vp)*vm;
    Ramp newRamp1(newVp, 0, 2*t + (nom/denom));
    ramps[1] = newRamp1;

    Ramp newRamp2(newVp, curve.ramps[1].a, curve.ramps[1].duration - t);
    ramps[2] = newRamp2;

    curve.Initialize(ramps);
    RAMP_OPTIM_PLOG("Imposing velocity limits successful");
    return true;
}


inline dReal SolveBrakeTime(dReal x, dReal v, dReal xbound) {
    dReal bt;
    bool res = SafeEqSolve(v, 2*(xbound - x), epsilon, 0, inf, bt);
    if (!res) {
        RAMP_OPTIM_PLOG("Cannot solve the brake time equation: %.15e*t - %.15e = 0 with t being in [0, inf)", v, 2*(xbound - x));
        bt = 0;
    }
    return bt;
}


inline dReal SolveBrakeAccel(dReal x, dReal v, dReal xbound) {
    dReal ba;
    dReal coeff0 = 2*(xbound - x);
    dReal coeff1 = v*v;
    bool res = SafeEqSolve(coeff0, -coeff1, epsilon, -inf, inf, ba);
    if (!res) {
        RAMP_OPTIM_PLOG("Cannot solve the brake acceleration equation: %.15e*a + %.15e = 0 with a being in (-inf, inf)", coeff0, coeff1);
        ba = 0;
    }
    return ba;
}


bool ImposeJointLimitFixedDuration(ParabolicCurve& curveIn, dReal xmin, dReal xmax, dReal vm, dReal am, ParabolicCurve& curveOut) {
    /*
       This function for fixing x-bound violation is taken from OpenRAVE ParabolicPathSmooth library.
     */
    dReal bmin, bmax;
    curveIn.GetPeaks(bmin, bmax);
    if ((bmin >= xmin - epsilon) && (bmax <= xmax + epsilon)) {
        curveOut.Initialize(curveIn.ramps);
        RAMP_OPTIM_PLOG("The input ParabolicCurve does not violate joint limits");
        return true;
    }

    dReal duration = curveIn.duration;
    dReal x0 = curveIn.x0;
    dReal x1 = curveIn.x1;
    dReal v0 = curveIn.v0;
    dReal v1 = curveIn.v1;

    dReal bt0 = inf, ba0 = inf, bx0 = inf;
    dReal bt1 = inf, ba1 = inf, bx1 = inf;
    if (v0 > 0) {
        bt0 = SolveBrakeTime(x0, v0, xmax);
        bx0 = xmax;
        ba0 = SolveBrakeAccel(x0, v0, xmax);
    }
    else if (v0 < 0) {
        bt0 = SolveBrakeTime(x0, v0, xmin);
        bx0 = xmin;
        ba0 = SolveBrakeAccel(x0, v0, xmin);
    }

    if (v1 < 0) {
        bt1 = SolveBrakeTime(x1, -v1, xmax);
        bx1 = xmax;
        ba1 = SolveBrakeAccel(x1, -v1, xmax);
    }
    else if (v1 > 0) {
        bt1 = SolveBrakeTime(x1, -v1, xmin);
        bx1 = xmin;
        ba1 = SolveBrakeAccel(x1, -v1, xmin);
    }

    std::vector<Ramp> newRamps(0);
    dReal tempbmin, tempbmax;

    if ((bt0 < duration) && (Abs(ba0) <= am + epsilon)) {
        RAMP_OPTIM_PLOG("Case IIa: checking");
        Ramp firstRamp(v0, ba0, bt0, x0);
        if (Abs(x1 - bx0) < (duration - bt0)*vm) {
            ParabolicCurve tempCurve;
            if (Interpolate1DFixedDuration(bx0, x1, 0, v1, vm, am, duration - bt0, tempCurve)) {
                tempCurve.GetPeaks(tempbmin, tempbmax);
                if ((tempbmin >= xmin - epsilon) && (tempbmax <= xmax + epsilon)) {
                    RAMP_OPTIM_PLOG("Case IIa: successful");
                    newRamps.reserve(1 + tempCurve.ramps.size());
                    newRamps.push_back(firstRamp);
                    for (size_t i = 0; i < tempCurve.ramps.size(); ++i) {
                        newRamps.push_back(tempCurve.ramps[i]);
                    }
                }
            }
        }
    }

    if ((bt1 < duration) && (Abs(ba1) <= am + epsilon)) {
        RAMP_OPTIM_PLOG("Case IIb: checking");
        Ramp lastRamp(0, ba1, bt1, bx1);
        if (Abs(x0 - bx1) < (duration - bt1)*vm) {
            ParabolicCurve tempCurve;
            if (Interpolate1DFixedDuration(x0, bx1, v0, 0, vm, am, duration - bt1, tempCurve)) {
                tempCurve.GetPeaks(tempbmin, tempbmax);
                if ((tempbmin >= xmin - epsilon) && (tempbmax <= xmax + epsilon)) {
                    RAMP_OPTIM_PLOG("Case IIb: successful");
                    if (newRamps.size() > 0) {
                        newRamps.resize(0);
                    }
                    newRamps.reserve(1 + tempCurve.ramps.size());
                    for (size_t i = 0; i < tempCurve.ramps.size(); ++i) {
                        newRamps.push_back(tempCurve.ramps[i]);
                    }
                    newRamps.push_back(lastRamp);
                }
            }
        }
    }

    if (bx0 == bx1) {
        if ((bt0 + bt1 < duration) && (Max(Abs(ba0), Abs(ba1)) <= am + epsilon)) {
            RAMP_OPTIM_PLOG("Case III");
            Ramp ramp0(v0, ba0, bt0, x0);
            Ramp ramp1(0, 0, duration - (bt0 + bt1));
            Ramp ramp2(0, ba1, bt1);
            if (newRamps.size() > 0) {
                newRamps.resize(0);
            }
            newRamps.reserve(3);
            newRamps.push_back(ramp0);
            newRamps.push_back(ramp1);
            newRamps.push_back(ramp2);
        }
    }
    else {
        if ((bt0 + bt1 < duration) && (Max(Abs(ba0), Abs(ba1)) <= am + epsilon)) {
            RAMP_OPTIM_PLOG("Case IV: checking");
            Ramp firstRamp(v0, ba0, bt0, x0);
            Ramp lastRamp(0, ba1, bt1);
            if (Abs(bx0 - bx1) < (duration - (bt0 + bt1))*vm) {
                ParabolicCurve tempCurve;
                if (Interpolate1DFixedDuration(bx0, bx1, 0, 0, vm, am, duration - (bt0 + bt1), tempCurve)) {
                    tempCurve.GetPeaks(tempbmin, tempbmax);
                    if ((tempbmin >= xmin - epsilon) && (tempbmax <= xmax + epsilon)) {
                        RAMP_OPTIM_PLOG("Case IV: successful");
                        if (newRamps.size() > 0) {
                            newRamps.resize(0);
                        }
                        newRamps.reserve(2 + tempCurve.ramps.size());
                        newRamps.push_back(firstRamp);
                        for (size_t i = 0; i < tempCurve.ramps.size(); ++i) {
                            newRamps.push_back(tempCurve.ramps[i]);
                        }
                        newRamps.push_back(lastRamp);
                    }
                }
            }
        }
    }

    if (newRamps.empty()) {
        RAMP_OPTIM_PLOG("Cannot solve for a bounded trajectory");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", x0, x1, v0, v1, duration, xmin, xmax, vm, am);
        return false;
    }

    curveOut.Initialize(newRamps);
    ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, xmin, xmax, vm, am, x0, x1, v0, v1);
    if (ret == PCR_Normal) {
        return true;
    }
    else {
        RAMP_OPTIM_PLOG("Finished fixing joint limit violation but CheckParabolicCurve returns %d", ret);
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", x0, x1, v0, v1, duration, xmin, xmax, vm, am);
        return false;
    }
}


bool Stretch1D(ParabolicCurve& curveIn, dReal vm, dReal am, dReal newDuration, ParabolicCurve& curveOut) {
    return Interpolate1DFixedDuration(curveIn.x0, curveIn.x1, curveIn.v0, curveIn.v1, vm, am, newDuration, curveOut);
}


bool Interpolate1DFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal duration, ParabolicCurve& curveOut) {
    /*
       We want to 'stretch' this velocity profile to have a new duration of endTime. First, try
       re-interpolating this profile to have two ramps. If that doesn't work, try modifying the
       profile accordingly.

       Two-ramp case: let t = endTime (the new duration that we want), a0 and a1 the new
       accelerations of the profile, t0 the duration of the new first ramp.

       Starting from

              d = (v0*t0 + 0.5*a0*(t0*t0)) + ((v0 + a0*t0)*t1 + 0.5*a1*(t1*t1)),

       where d is the displacement done by this trajectory, t1 = duration - t0, i.e., the
       duration of the second ramp.  Then we can write a0 and a1 in terms of t0 as

              a0 = A + B/t0
              a1 = A - B/t1,

       where A = (v1 - v0)/t and B = (2d/t) - (v0 + v1). We want to get the velocity profile
       which has minimal acceleration: set the minimization objective to

              J(t0) = a0*a0 + a1*a1.

       We start by calculating feasible ranges of t0 due to various constraints.

       1) Acceleration constraints for the first ramp:

              -amax <= a0 <= amax.

         From this, we have

              -amax - A <= B/t0            ---   I)
              B/t0 >= amax - A.            ---  II)

         Let sum1 = -amax - A and sum2 = amax - A. We can obtain the feasible ranges of t0
         accordingly.

       2) Acceleration constraints for the second ramp:

              -amax <= a1 <= amax.

         From this, we have

              -amax - A <= -B/(t - t0)      --- III)
              -B/(t - t0) <= amax - A.      ---  IV)

         As before, the feasible ranges of t0 can be computed accordingly.

       We will obtain an interval iX for each constraint X. Since t0 needs to satisfy all the
       four constraints plus the initial feasible range [0, endTime], we will obtain only one single
       feasible range for t0. (Proof sketch: intersection operation is associative and
       intersection of two intervals gives either an interval or an empty set.)

     */
    if (duration < -epsilon) {
        RAMP_OPTIM_PLOG("duration = %.15e is negative", duration);
        return false;
    }
    if (duration <= epsilon) {
        // Check if this is a stationary trajectory
        if (FuzzyEquals(x0, x1, epsilon) & FuzzyEquals(v0, v1, epsilon)) {
            // This is actually a stationary trajectory
            Ramp ramp0(v0, 0, 0, x0);
            std::vector<Ramp> ramps(1);
            ramps[0] = ramp0;
            curveOut.Initialize(ramps);
            ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
            if (ret == PCR_Normal) {
                return true;
            }
            else {
                RAMP_OPTIM_PLOG("Case: stationary trajectory. Finished stretching but the profile does not pass the check: ret = %d", ret);
                RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                return false;
            }
        }
        else {
            RAMP_OPTIM_PLOG("The given duration is too short for any movement to be made.");
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            return false;
        }
    }

    // Correct small discrepancies in velocities (if any)
    if (v0 > vm) {
        if (FuzzyEquals(v0, vm, epsilon)) {
            // Acceptable
            v0 = vm;
        }
        else {
            RAMP_OPTIM_WARN("v0 (%.15e) > vm (%.15e)", v0, vm);
            return false;
        }
    }
    else if (v0 < -vm) {
        if (FuzzyEquals(v0, -vm, epsilon)) {
            // Acceptable
            v0 = -vm;
        }
        else {
            RAMP_OPTIM_WARN("v0 (%.15e) < -vm (%.15e)", v0, -vm);
            return false;
        }
    }
    if (v1 > vm) {
        if (FuzzyEquals(v1, vm, epsilon)) {
            // Acceptable
            v1 = vm;
        }
        else {
            RAMP_OPTIM_WARN("v1 (%.15e) > vm (%.15e)", v1, vm);
            return false;
        }
    }
    else if (v1 < -vm) {
        if (FuzzyEquals(v1, -vm, epsilon)) {
            // Acceptable
            v1 = -vm;
        }
        else {
            RAMP_OPTIM_WARN("v1 (%.15e) < -vm (%.15e)", v1, -vm);
            return false;
        }
    }

    dReal d = x1 - x0; // displacement made by this profile
    dReal t0, t1, vp, a0, a1;
    dReal A, B, C, D; // temporary variables for solving equations

    dReal durInverse = 1/duration;
    A = (v1 - v0)*durInverse;
    B = (2*d)*durInverse - (v0 + v1);

    /*
      A velocity profile having t = duration connecting (x0, v0) and (x1, v1) will have one ramp iff 
              
              x1 - x0 = dStraight
                    d = 0.5*(v0 + v1)*duration
                    
      The above equation is actually equivalent to
      
                    B = 0.

      Therefore, if B = 0 we can just interpolate the trajectory right away and return early.
     */
    if (FuzzyZero(B, epsilon)) {
        Ramp ramp0(v0, A, duration, x0);
        std::vector<Ramp> ramps(1);
        ramps[0] = ramp0;
        curveOut.Initialize(ramps);
        ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
        if (ret == PCR_Normal) {
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("Case: B == 0. Finished stretching but the profile does not pass the check: ret = %d", ret);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            return false;
        }
    }
    
    dReal sum1 = -am - A;
    dReal sum2 = am - A;
    C = B/sum1;
    D = B/sum2;

    // RAMP_OPTIM_PLOG("A = %.15e; B = %.15e, C = %.15e, D = %.15e; sum1 = %.15e; sum2 = %.15e", A, B, C, D, sum1, sum2);
    if (IS_DEBUGLEVEL(Level_Verbose)) {
        if ((Abs(A) <= epsilon) && (Abs(B) <= epsilon)) {
            RAMP_OPTIM_PLOG("A and B are zero");
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        }
    }

    // Now we need to check a number of feasible intervals of tswitch1 induced by constraints on the
    // acceleration. Instead of having a class representing an interval, we use the interval bounds
    // directly. Naming convention: iXl = lower bound of interval X, iXu = upper bound of interval X.
    dReal i0l = 0, i0u = duration;
    dReal i1l = -Inf, i1u = Inf;
    dReal i2l = -Inf, i2u = Inf;
    dReal i3l = -Inf, i3u = Inf;
    dReal i4l = -Inf, i4u = Inf;

    // Intervals 1 and 2 are derived from constraints on a0 (the acceleration of the first ramp)
    // I) sum1 <= B/t0
    if (sum1 == 0) {
        if (B == 0) {
            // t0 can be anything
        }
        else {
            i1l = Inf;
        }
    }
    else if (sum1 > 0) {
        RAMP_OPTIM_PLOG("sum1 > 0. This implies that duration is too short");
        RAMP_OPTIM_PLOG("ParabolicRamp1D info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        return false;
    }
    else {
        // i1 = [C, Inf)
        i1l = C;
    }

    // II) B/t0 <= sum2
    if (sum2 == 0) {
        if (B == 0) {
            // t0 can be anything
        }
        else {
            i2l = Inf;
        }
    }
    else if (sum2 > 0) {
        // i2 = [D, Inf)
        i2l = D;
    }
    else {
        RAMP_OPTIM_PLOG("sum2 < 0. This implies that duration is too short");
        RAMP_OPTIM_PLOG("ParabolicRamp1D info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        return false;
    }

    // Find the intersection between interval 1 and interval 2, store it in interval 2.
    if ((i1l > i2u) || (i1u < i2l)) {
        RAMP_OPTIM_PLOG("interval 1 and interval 2 do not have any intersection");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        return false;
    }
    else {
        i2l = Max(i1l, i2l);
        i2u = Min(i1u, i2u);
    }

    // Intervals 3 and 4 are derived from constraints on a1 (the acceleration of the second (last) ramp
    // III) sum1 <= B/(t0 - t)
    if (sum1 == 0) {
        if (B == 0) {
            // t0 can be anything
        }
        else {
            i3l = Inf;
        }
    }
    else if (sum1 > 0) {
        RAMP_OPTIM_PLOG("sum1 > 0. This implies that duration is too short");
        RAMP_OPTIM_PLOG("ParabolicRamp1D info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        return false;
    }
    else {
        // i3 = (-Inf, t + C]
        i3u = duration + C;
    }

    // IV)
    if (sum2 == 0) {
        if (B == 0) {
            // t0 can be anything
        }
        else {
            i4l = Inf;
        }
    }
    else if (sum2 > 0) {
        // i4 = (-Inf, t + D]
        i4u = duration + D;
    }
    else {
        RAMP_OPTIM_PLOG("sum2 < 0. This implies that duration is too short");
        RAMP_OPTIM_PLOG("ParabolicRamp1D info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        return false;
    }

    // Find the intersection between interval 3 and interval 4, store it in interval 4.
    if ((i3l > i4u) || (i3u < i4l)) {
        RAMP_OPTIM_PLOG("interval 3 and interval 4 do not have any intersection");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        return false;
    }
    else {
        i4l = Max(i3l, i4l);
        i4u = Min(i3u, i4u);
    }

    // Find the intersection between interval 2 and interval 4, store it in interval 4. This is a
    // bit tricky because if the given duration is actually the minimum time that this trajectory
    // can get, the two intervals will theoretically intersect at only one point.
    if (FuzzyEquals(i2l, i4u, epsilon) || FuzzyEquals(i2u, i4l, epsilon)) {
        RAMP_OPTIM_PLOG("interval 2 and interval 4 intersect at a point, most likely because the given endTime is actually its minimum time.");
        // Make sure that the above statement is true.
        if (!Interpolate1D(x0, x1, v0, v1, vm, am, curveOut)) {
            return false; // what ?
        }
        else {
            if (FuzzyEquals(curveOut.duration, duration, epsilon)) {
                RAMP_OPTIM_PLOG("The hypothesis is correct.");
                // The curve has already been validated in Interpolate1D
                return true;
            }
            else {
                RAMP_OPTIM_PLOG("The hypothesis is wrong. Something else just happened");
                RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                return false; // what ?
            }
        }
    }
    else if ((i2l > i4u) || (i2u < i4l)) {
        RAMP_OPTIM_PLOG("interval 2 and interval 4 do not have any intersection");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        return false;
    }
    else {
        i4l = Max(i2l, i4l);
        i4u = Min(i2u, i4u);
    }

    // Find the intersection between interval 0 and interval 4, store it in interval 4.
    if ((i0l > i4u) || (i0u < i4l)) {
        RAMP_OPTIM_PLOG("interval 0 and interval 4 do not have any intersection");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        return false;
    }
    else {
        i4l = Max(i0l, i4l);
        i4u = Min(i0u, i4u);
    }

    if (i4l > i4u) {
        RAMP_OPTIM_PLOG("interval 4 is empty but the algorithm cannot detect this.");
        RAMP_OPTIM_PLOG("interval4 = [%.15e, %.15e]", i4l, i4u);
        return false;
    }

    /*
       Now we have already obtained a range of feasible values for t0. We choose a value of
       t0 by selecting the one which minimize J(t0) := (a0^2 + a1^2).

       Now let x = t0 for convenience. We can write J(x) as

              J(x) = (A + B/x)^2 + (A - B/(t - x))^2.

       Then we find x which minimizes J(x) by examining the roots of dJ/dx.
     */
    bool res = SolveForT0(A, B, duration, i4l, i4u, t0);
    if (!res) {
        // Solving dJ/dx = 0 somehow failed. We just choose the midpoint of the feasible interval.
        t0 = 0.5*(i4l + i4u);
    }


    if (0) {
        // Here we need to take care of the cases when t0 has been rounded. In particular, we need to go
        // back to the original formulae to calculated other related values (such as a1). Otherwise, it
        // may cause discrepancies.
        if (FuzzyZero(t0, epsilon) || FuzzyEquals(t0, duration, epsilon)) {
            // t0 is either 0 or duration. This means the new trajectory will consist of only one
            // Ramp. Since v0 and v1 are withint the limits, we are safe.
            Ramp ramp0(v0, A, duration, x0);
            std::vector<Ramp> ramps(1);
            ramps[0] = ramp0;
            curveOut.Initialize(ramps);
            ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
            if (ret == PCR_Normal) {
                return true;
            }
            else {
                RAMP_OPTIM_PLOG("Case: either t0 or t1 is too small. Finished stretching but the profile does not pass the check: ret = %d", ret);
                RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                RAMP_OPTIM_PLOG("Calculated values: A = %.15e; B = %.15e; t0 = %.15e; t1 = %.15e; vp = %.15e; a0 = %.15e; a1 = %.15e", A, B, t0, t1, vp, a0, a1);
                return false;
            }
        }
    } // Old procedure which can cause large displacement discrepancy

    /*
      We skip rounding (to zero) of t0 and t1 here since
      1. it can cause too large displacement discrepancy
      2. the original reason for this rounding is just to prevent zero division when calculating
      accelerations, a0 and a1. When t0 or t1 is very small (but non-zero), although it will result
      in huge accelerations, those accelerations will be checked and limited to the bound anyway.
     */
    if (t0 == 0) {
        Ramp ramp0(v0, A, duration, x0);
        std::vector<Ramp> ramps(1);
        ramps[0] = ramp0;
        curveOut.Initialize(ramps);
        ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
        if (ret == PCR_Normal) {
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("Case: t0 == 0. Finished stretching but the profile does not pass the check: ret = %d", ret);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            return false;
        }
    }
    
    t1 = duration - t0;

    if (t1 == 0) {
        Ramp ramp0(v0, A, duration, x0);
        std::vector<Ramp> ramps(1);
        ramps[0] = ramp0;
        curveOut.Initialize(ramps);
        ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
        if (ret == PCR_Normal) {
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("Case: t1 == 0. Finished stretching but the profile does not pass the check: ret = %d", ret);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            return false;
        }
    }    
    
    a0 = A + B/t0;
    a1 = A - B/t1;
    vp = v0 + (a0*t0);

    // Consistency checking
    if (!FuzzyEquals(vp, v1 - (a1*t1), epsilon)) {
        RAMP_OPTIM_PLOG("Verification failed (vp != v1 - a1*d1): %.15e != %.15e", vp, v1 - (a1*t1));
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
        RAMP_OPTIM_PLOG("Calculated values: A = %.15e; B = %.15e; t0 = %.15e; t1 = %.15e; vp = %.15e; a0 = %.15e; a1 = %.15e", A, B, t0, t1, vp, a0, a1);
        return false;
    }

    // Velocity bound checking
    if (!(Abs(vp) > vm + epsilon)) {
        // The two-ramp profile works. Go for it.
        Ramp ramp0(v0, a0, t0, x0);
        Ramp ramp1(vp, a1, t1);
        std::vector<Ramp> ramps(2);
        ramps[0] = ramp0;
        ramps[1] = ramp1;
        curveOut.Initialize(ramps);
        // Check the curve without joint limits
        ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
        if (ret == PCR_Normal) {
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("Finished stretching but the profile does not pass the check: ret = %d", ret);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            RAMP_OPTIM_PLOG("Calculated values: A = %.15e; B = %.15e; t0 = %.15e; t1 = %.15e; vp = %.15e; a0 = %.15e; a1 = %.15e", A, B, t0, t1, vp, a0, a1);
            return false;
        }
    }
    else {
        // The two-ramp profile does not work because it violates the velocity bounds. Modify it
        // accordingly.
        dReal vmNew = vp > 0 ? vm : -vm;

        // a0 and a1 should not be zero if the velocity limit is violated. The first check is done
        // at the line: FuzzyEquals(vp, v1 - (a1*t1)) above.
        if ((FuzzyZero(a0, epsilon)) || (FuzzyZero(a1, epsilon))) {
            RAMP_OPTIM_PLOG("Velocity limit is violated but at least one acceleration is zero: a0 = %.15e; a1 = %.15e", a0, a1);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            return false;
        }
        dReal a0inv = 1/a0;
        dReal a1inv = 1/a1;

        dReal dv1 = vp - vmNew;
        dReal dv2 = vmNew - v0;
        dReal dv3 = vmNew - v1;
        dReal t0Trimmed = dv2*a0inv; // vmaxNew = dx0 + a0*t0Trimmed
        dReal t1Trimmed = -dv3*a1inv; // dx1 = vmaxNew + a1*t1Trimmed

        /*
           Idea: we cut the excessive area above the velocity limit and paste that on both sides of
           the velocity profile. We do not divide the area and paste it equally on both
           sides. Instead, we try to minimize the sum of the new accelerations squared:

                  minimize    a0New^2 + a1New^2.

           Let D2 be the area of the velocity profile above the velocity limit. We have

                  D2 = 0.5*dt1*dv2 + 0.5*dt2*dv3.

           Using the relations

                  a0New = dv2/(t0Trimmed - dt1)    and
                  a1New = -dv3/(t1Trimmed - dt2)

           we finally arrive at the equation

                  A2/a0New + B2/a1New = C2,

           where A2 = dv2^2, B2 = -dv3^2, and C2 = t0Trimmed*dv2 + t1Trimmed*dv3 - 2*D2.

           Let x = a0New and y = a1New for convenience, we can formulate the problem as

                  minimize(x, y)    x^2 + y^2
                  subject to        A2/x + B2/y = C2.

           From the above problem, we can see that the objective function is actually a circle while
           the constraint function is a hyperbola. (The hyperbola is centered at (A2/C2,
           B2/C2)). Therefore, the minimizer is the point where both curves touch.

           Let p = (x0, y0) be the point that the two curves touch. Then

                  (slope of the hyperbola at p)*(y0/x0) = -1,

           i.e., the tangent line of the hyperbola at p and the line connecting the origin and p are
           perpendicular. Solving the above equation gives us

                  x0 = (A2 + (A2*B2*B2)^(1/3))/C2.
         */

        dReal A2 = dv2*dv2;
        dReal B2 = -dv3*dv3;
        dReal D2 = 0.5*dv1*(duration - t0Trimmed - t1Trimmed); // area of the velocity profile above the velocity limit.
        dReal C2 = t0Trimmed*dv2 + t1Trimmed*dv3 - 2*D2;

        dReal root = cbrt(A2*B2*B2); // from math.h

        if (FuzzyZero(C2, epsilon)) {
            // This means the excessive area is too large such that after we paste it on both sides
            // of the original velocity profile, the whole profile becomes one-ramp with a = 0 and v
            // = vmNew.
            RAMP_OPTIM_PLOG("C2 == 0. Unable to fix this case.");
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            return false;
        }

        dReal C2inv = 1/C2;
        a0 = (A2 + root)*C2inv;
        if (Abs(a0) > am) {
            if (FuzzyZero(root, epsilon*epsilon)) {// This condition gives a1 = 0.
                // The computed a0 is exceeding the bound and its corresponding a1 is
                // zero. Therefore, we cannot fix this case. This is probably because the given
                // duration is actually less than the minimum duration that it can get.
                RAMP_OPTIM_PLOG("|a0| > am and a1 == 0: Unable to fix this case since the given duration is too short.");
                RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                return false;
            }

            // a0 exceeds the bound, try making it stays at the bound.
            a0 = a0 > 0 ? am : -am;
            // Recalculate the related variable
            root = C2*a0 - A2;
        }

        // Now compute a1
        // Special case: a0 == 0. Then this implies vm == dx0. Reevaluate those above equations
        // leads to a1 = B2/C2
        if (Abs(a0) <= epsilon) {
            a0 = 0;
            a1 = B2/C2;
            if (Abs(a1) > am) {
                // The computed a1 is exceeding the bound while a0 being zero. This is similar to
                // the case above when |a0| > am and a1 == 0.
                RAMP_OPTIM_PLOG("a0 == 0 and |a1| > am: Unable to fix this case since the given duration is too short.");
                RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                return false;
            }

            root = C2*a0 - A2;
            // RAVELOG_DEBUG_FORMAT("case1: a0 = %.15e; a1 = %.15e", a0%a1);
        }
        else {
            // From the hyperbola equation, we have y = B2*x/(C2*x - A2) = B2*x/root
            if (Abs(root) < epsilon*epsilon) {
                // Special case: a1 == 0. This implies vm == dx1. If we calculate back the value of a0,
                // we will get a0 = A2/C2 which is actually root = 0.
                a1 = 0;
                a0 = A2/C2;
                // RAVELOG_DEBUG_FORMAT("case2: a0 = %.15e; a1 = %.15e", a0%a1);
            }
            else {
                a1 = B2*a0/root;
                if (Abs(a1) > am) {
                    // a1 exceeds the bound, try making it stays at the bound.
                    a1 = a1 > 0 ? am : -am;
                    // Recalculate the related variable
                    if (C2*a1 - B2 == 0) {
                        // this case means a0 == 0 which shuold have been catched from above
                        RAMP_OPTIM_PLOG("(C2*a1 - B2 == 0) a0 shuold have been zero but a0 = %.15e", a0);
                        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                        return false;
                    }
                    a0 = A2*a1/(C2*a1 - B2);
                    // RAVELOG_DEBUG_FORMAT("case3: a0 = %.15e; a1 = %.15e", a0%a1);
                }
            }
        }

        // Final check on the accelerations
        if ((Abs(a0) > am) || (Abs(a1) > am)) {
            RAMP_OPTIM_PLOG("Cannot fix accelration bounds violation");
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            return false;
        }

        if ((Abs(a0) <= epsilon) && (Abs(a1) <= epsilon)) {
            RAMP_OPTIM_PLOG("Both accelerations are zero.");
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            RAMP_OPTIM_PLOG("A2 = %.15e; B2 = %.15e; C2 = %.15e; D2 = %.15e", A2, B2, C2, D2);
            return false;
        }

        if (Abs(a0) <= epsilon) {
            RAMP_OPTIM_PLOG("|a0| < epsilon");
            t0 = duration + dv3/a1;
            t1 = duration - t0;
            vp = vmNew;

            Ramp ramp0(v0, a0, t0, x0);
            Ramp ramp1(vp, a1, t1);
            std::vector<Ramp> ramps(2);
            ramps[0] = ramp0;
            ramps[1] = ramp1;
            curveOut.Initialize(ramps);
        }
        else if (Abs(a1) <= epsilon) {
            RAMP_OPTIM_PLOG("|a1| < epsilon");
            t0 = dv2/a0;
            t1 = duration - t0;
            vp = vmNew;

            Ramp ramp0(v0, a0, t0, x0);
            Ramp ramp1(vp, a1, t1);
            std::vector<Ramp> ramps(2);
            ramps[0] = ramp0;
            ramps[1] = ramp1;
            curveOut.Initialize(ramps);
        }
        else {
            t0 = dv2/a0;
            if (t0 < 0) {
                RAMP_OPTIM_PLOG("t0 < 0. The given duration is not achievable with the given bounds");
                RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                return false;
            }

            vp = vmNew;
            dReal tLastRamp = -dv3/a1;
            if (tLastRamp < 0) {
                RAMP_OPTIM_PLOG("tLastRamp < 0. The given duration is not achievable with the given bounds");
                RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                return false;
            }

            if (t0 + tLastRamp > duration) {
                // Final fix
                if (A == 0) {
                    RAMP_OPTIM_PLOG("(final fix) A = 0. Don't know how to deal with this case.");
                    RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                    return false;
                }
                t0 = (dv2 - B)/A; // note that we use A and B, not A2 and B2.
                if (t0 < 0) {
                    RAMP_OPTIM_PLOG("(final fix) t0 < 0. Don't know how to deal with this case.");
                    RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                    return false;
                }
                t1 = duration - t0;
                a0 = A + (B/t0);
                a1 = A - (B/t1);

                Ramp ramp0(v0, a0, t0, x0);
                Ramp ramp1(vp, a1, t1);
                std::vector<Ramp> ramps(2);
                ramps[0] = ramp0;
                ramps[1] = ramp1;
                curveOut.Initialize(ramps);
            }
            else {
                dReal tMiddle = duration - (t0 + tLastRamp);
                if (FuzzyZero(tMiddle, epsilon)) {
                    RAMP_OPTIM_PLOG("Three-ramp profile works but having too short middle ramp.");
                    // If we leave it like this, it may cause errors later on.
                    t0 = (2*d - (v1 + vmNew)*duration)/(v0 - v1);
                    t1 = duration - t0;
                    vp = vmNew;
                    a0 = dv2/t0;
                    a1 = -dv3/t1;
                    if ((Abs(a0) > am + epsilon) || (Abs(a1) > am + epsilon)) {
                        RAMP_OPTIM_PLOG("Cannot merge into two-ramp because of acceleration limits");
                        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
                        RAMP_OPTIM_PLOG("Calculated values: t0 = %.15e; t1 = %.15e; vp = %.15e; a0 = %.15e; a1 = %.15e", t0, t1, vp, a0, a1);
                        return false;
                    }

                    Ramp ramp0(v0, a0, t0, x0);
                    Ramp ramp1(vp, a1, t1);
                    std::vector<Ramp> ramps(2);
                    ramps[0] = ramp0;
                    ramps[1] = ramp1;
                    curveOut.Initialize(ramps);
                }
                else {
                    // Three-ramp profile really works now
                    Ramp ramp0(v0, a0, t0, x0);
                    Ramp ramp1(vp, 0, tMiddle);
                    Ramp ramp2(vp, a1, tLastRamp);
                    std::vector<Ramp> ramps(3);
                    ramps[0] = ramp0;
                    ramps[1] = ramp1;
                    ramps[2] = ramp2;
                    curveOut.Initialize(ramps);
                }
            }
        }
        // Check the curve without joint limits
        ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
        if (ret == PCR_Normal) {
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("Finished stretching but the profile does not pass the check: ret = %d", ret);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0, x1, v0, v1, vm, am, duration);
            RAMP_OPTIM_PLOG("Calculated values: A = %.15e; B = %.15e; t0 = %.15e; t1 = %.15e; vp = %.15e; a0 = %.15e; a1 = %.15e", A, B, t0, t1, vp, a0, a1);
            return false;
        }
    }
}

/*
   Utilities
 */
bool CalculateLeastUpperBoundInoperativeInterval(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal& t) {
    dReal d = x1 - x0;
    dReal T0, T1, T2, T3;

    /*
       Let t be the total duration of the velocity profile, a0 and a1 be the accelerations of both
       ramps. We write, in the way that has already been described in SolveMinAccel, a0 and a1 in
       terms of t0 as

              a0 = A + B/t0        and
              a1 = A - B/(t - t0).

       Imposing the acceleration bounds, we have the following inequalities:

          from -am <= a0 <= am, we have

                        t0*sum1 <= B
                              B <= sum2*t0

          from -am <= a1 <= am, we have

                  (t - t0)*sum1 <= -B
                             -B <= sum2*(t - t0),

       where sum1 = -am - A, sum2 = am - A.

       From those inequalities, we can deduce that a feasible value of t0 must fall in the
       intersection of

              [B/sum1, t + B/sum1]    and
              [B/sum2, t + B/sum2].

       Therefore, the total duration t must satisfy

              t >= B*(1/sum2 - 1/sum1)    and
              t >= B*(1/sum1 - 1/sum2).

       By substituting A = (v1 - v0)/t and B = 2*d/t - (v0 + v1) into the above inequalities, we
       have

              t >= (2*am*((2*d)/t - (v0 + v1)))/(am*am - ((v1 - v0)/t)**2)    and
              t >= -(2*am*((2*d)/t - (v0 + v1)))/(am*am - ((v1 - v0)/t)**2),

       (the inequalities are derived using Sympy). Finally, we have two solutions (for the total
       time) from each inequality. Then we select the maximum one.

       Important note: position limits are not taken into account here. The calculated upper bound
       may be invalidated because of position constraints.
     */

    dReal firstTerm = (v0 + v1)/am;

    dReal temp1 = 2*(-Sqr(am))*(2*am*d - Sqr(v0) - Sqr(v1));
    dReal secondTerm1 = Sqrt(temp1)/Sqr(am);
    if (temp1 < 0) {
        T0 = -1;
        T1 = -1;
    }
    else {
        T0 = firstTerm + secondTerm1;
        T1 = firstTerm - secondTerm1;
    }
    T1 = Max(T0, T1);

    dReal temp2 = 2*(Sqr(am))*(2*am*d + Sqr(v0) + Sqr(v1));
    dReal secondTerm2 = Sqrt(temp2)/Sqr(am);
    if (temp2 < 0) {
        T2 = -1;
        T3 = -1;
    }
    else {
        T2 = -firstTerm + secondTerm2;
        T3 = -firstTerm - secondTerm2;
    }
    T3 = Max(T2, T3);

    t = Max(T1, T3);
    if (t > epsilon) {
        // Sanity check

        // dStraight is the displacement produced if we were to travel with only one acceleration
        // from v0 to v1 in t. It is used to determine which direction we should aceelerate
        // first (posititve or negative acceleration).
        dReal dStraight = 0.5*(v0 + v1)*t;
        dReal amNew = d - dStraight > 0 ? am : -am;
        dReal vmNew = d - dStraight > 0 ? vm : -vm;

        dReal vp = 0.5*(amNew*t + v0 + v1); // the peak velocity
        if (Abs(vp) > vm) {
            dReal dExcess = (vp - vmNew)*(vp - vmNew)/am;
            dReal deltaTime = dExcess/vm;
            t += deltaTime; // the time increased from correcting the velocity bound violation
        }
        // Should be no problem now.
        t = t * 1.01; // for safety reasons, we don't make t too close to the bound
        return true;
    }
    else {
        if (FuzzyEquals(x1, x0, epsilon) && FuzzyZero(v0, epsilon) && FuzzyZero(v1, epsilon)) {
            t = 0;
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("Unable to calculate the least upper bound: T0 = %.15e, T1 = %.15e, T2 = %.15e, T3 = %.15e", T0, T1, T2, T3);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e", x0, x1, v0, v1, vm, am);
            return false;
        }
    }
}


bool SolveForT0(dReal A, dReal B, dReal t, dReal l, dReal u, dReal& t0) {
    /*
       Let x = t0 for convenience. The two accelerations can be written in terms of x as

          a0 = A + B/x    and
          a1 = A - B/(t - x),

       where t is the total duration. We want to solve the following optimization problem:

          minimize(x)    J(x) = a0^2 + a1^2.

       We find the minimizer by solving dJ/dx = 0. From

          J(x) = (A + B/x)^2 + (A - B/(t - x))^2,

       we have

          dJ/dx = (2*A)*x^4 + (2*B - 4*A*t)*x^3 + (3*A*t^2 - 3*B*t)*x^2 + (A*t^3 + 3*t^2)*x + (B*t^3).
     */
    if (l < 0) {
        if (u < 0) {
            RAMP_OPTIM_PLOG("The given interval is invalid: l = %.15e; u = %.15e", l, u);
            return false;
        }
        RAMP_OPTIM_PLOG("Invalid lower bound is given, so reset it to zero.");
        l = 0;
    }

    if ((Abs(A) < epsilon) && (Abs(B) < epsilon)) {
        if (l > 0) {
            return false;
        }
        else {
            t0 = 0;
            return true;
        }
    }

    dReal rawRoots[4];
    int numRoots;
    double tSqr = t*t;
    double tCube = tSqr*t;

    if (Abs(A) < epsilon) {
        dReal coeffs[4] = {2*B, -3*B*t, 3*B*tSqr, -B*tCube};
        OpenRAVE::mathextra::polyroots<dReal, 3>(&coeffs[0], &rawRoots[0], numRoots);
    }
    else {
        dReal coeffs[5] = {2*A, -4*A*t + 2*B, 3*A*tSqr - 3*B*t, -A*tCube + 3*B*tSqr, -B*tCube};
        OpenRAVE::mathextra::polyroots<dReal, 4>(&coeffs[0], &rawRoots[0], numRoots);
    }

    if (numRoots == 0) {
        return false;
    }

    // Among all the solutions, find the one that minimizes the objective function.
    dReal J = Inf;
    dReal bestT = -1;
    for (int i = 0; i < numRoots; ++i) {
        if ((rawRoots[i] <= u) && (rawRoots[i] >= l)) {
            dReal root = rawRoots[i];
            dReal firstTerm, secondTerm;
            if (Abs(root) < epsilon) {
                firstTerm = 0;
            }
            else {
                firstTerm = A + (B/root);
            }

            if (Abs(t - root) < epsilon) {
                secondTerm = 0;
            }
            else {
                secondTerm = A - (B/(t - root));
            }
            dReal curObj = firstTerm*firstTerm + secondTerm*secondTerm;
            if (curObj < J) {
                J = curObj;
                bestT = root;
            }
        }
    }
    if (bestT < 0) {
        return false;
    }
    else {
        t0 = bestT;
        return true;
    }
}


} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
