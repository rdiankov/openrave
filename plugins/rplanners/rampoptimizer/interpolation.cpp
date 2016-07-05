#include "interpolation.h"
#include "parabolicchecker.h"
#include <math.h>
#include <openrave/mathextra.h>

namespace RampOptimizerInternal {

/*
   Multi DOF interpolation
 */
bool InterpolateZeroVelND(std::vector<Real>& x0Vect, std::vector<Real>& x1Vect, std::vector<Real>& vmVect, std::vector<Real>& amVect, ParabolicCurvesND& curvesndOut) {
    int ndof = x0Vect.size();
    BOOST_ASSERT(ndof == (int) x1Vect.size());
    BOOST_ASSERT(ndof == (int) vmVect.size());
    BOOST_ASSERT(ndof == (int) amVect.size());

    std::vector<Real> dVect(ndof);
    LinearCombination(1, x1Vect, -1, x0Vect, dVect);

    // Calculate sdMax (vMin) and sddMax (aMin)
    Real vMin = inf;
    Real aMin = inf;
    for (int i = 0; i < ndof; ++i) {
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
        Real sd0, sdd;
        Real dur = itRamp->duration;
        for (int i = 0; i < ndof; ++i) {
            sd0 = dVect[i] * (itRamp->v0);
            sdd = dVect[i] * (itRamp->a);
            Ramp ramp(sd0, sdd, dur);
            std::vector<Ramp> ramps(1);
            ramps[0] = ramp;
            ParabolicCurve curve(ramps);
            curves[i].Append(curve);
        }
    }

    for (int i = 0; i < ndof; ++i) {
        curves[i].SetInitialValue(x0Vect[i]);
    }
    curvesndOut.Initialize(curves);
    return true;
}


bool InterpolateArbitraryVelND(std::vector<Real>& x0Vect, std::vector<Real>& x1Vect, std::vector<Real>& v0Vect, std::vector<Real>& v1Vect, std::vector<Real>& xminVect, std::vector<Real>& xmaxVect, std::vector<Real>& vmVect, std::vector<Real>& amVect, ParabolicCurvesND& curvesndOut, bool tryHarder) {
    int ndof = x0Vect.size();
    BOOST_ASSERT(ndof == (int) x1Vect.size());
    BOOST_ASSERT(ndof == (int) v0Vect.size());
    BOOST_ASSERT(ndof == (int) v1Vect.size());
    BOOST_ASSERT(ndof == (int) vmVect.size());
    BOOST_ASSERT(ndof == (int) amVect.size());

    std::vector<Real> dVect(ndof);
    LinearCombination(1, x1Vect, -1, x0Vect, dVect);

    // First independently interpolate each DOF to find the slowest DOF
    std::vector<ParabolicCurve> curves(ndof);
    Real maxDuration = 0;
    size_t maxIndex;
    bool result;
    for (int i = 0; i < ndof; ++i) {
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
            RAMP_OPTIM_PERROR("Interpolation with a fixed duration failed even when trying harder.");
        }
        return false;
    }

    std::vector<ParabolicCurve> newCurves(ndof);
    for (int i = 0; i < ndof; ++i) {
        result = ImposeJointLimitFixedDuration(tempcurvesnd.curves[i], xminVect[i], xmaxVect[i], vmVect[i], amVect[i], newCurves[i]);
        if (!result) {
            return false;
        }
    }

    curvesndOut.Initialize(newCurves);
    return true;
}


bool ReinterpolateNDFixedDuration(std::vector<ParabolicCurve>& curvesVectIn, std::vector<Real>& vmVect, std::vector<Real>& amVect, int maxIndex, ParabolicCurvesND& curvesndOut, bool tryHarder) {
    int ndof = curvesVectIn.size();
    RAMP_OPTIM_ASSERT(ndof == (int) vmVect.size());
    RAMP_OPTIM_ASSERT(ndof == (int) amVect.size());
    RAMP_OPTIM_ASSERT(maxIndex < ndof);

    std::vector<ParabolicCurve> newCurves(ndof);
    if (!tryHarder) {
        Real newDuration = curvesVectIn[maxIndex].duration;
        for (int i = 0; i < ndof; ++i) {
            if (i == maxIndex) {
                RAMP_OPTIM_PLOG("joint %d is already the slowest DOF, continue to the next DOF", i);
                newCurves[i] = curvesVectIn[i];
                continue;
            }

            ParabolicCurve tempCurve;
            bool result = Stretch1D(curvesVectIn[i], newDuration, vmVect[i], amVect[i], tempCurve);
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
        Real newDuration = 0;

        for (int i = 0; i < ndof; ++i) {
            Real t;
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

        for (int i = 0; i < ndof; ++i) {
            if (isPrevDurationSafe && (i == maxIndex)) {
                RAMP_OPTIM_PLOG("joint %d is already the slowest DOF, continue to the next DOF", i);
                newCurves[i] = curvesVectIn[i];
                continue;
            }

            ParabolicCurve tempCurve;
            bool result = Stretch1D(curvesVectIn[i], newDuration, vmVect[i], amVect[i], tempCurve);
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

/*
   Single DOF interpolation
 */
bool Interpolate1D(Real x0, Real x1, Real v0, Real v1, Real vm, Real am, ParabolicCurve& curveOut) {
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


bool Interpolate1DNoVelocityLimit(Real x0, Real x1, Real v0, Real v1, Real am, ParabolicCurve& curveOut) {
    RAMP_OPTIM_ASSERT(am > 0);

    Real d = x1 - x0;
    Real dv = v1 - v0;
    Real v0sqr = v0*v0;
    Real v1sqr = v1*v1;
    Real difVSqr = v1sqr - v0sqr;

    Real dStraight; // displacement obtained when maximally accelerate/decelerate from v0 to v1
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
        Real a0 = dv > 0 ? am : -am;
        Ramp ramp0(v0, a0, dv/a0, x0);
        std::vector<Ramp> ramps(1);
        ramps[0] = ramp0;
        curveOut.Initialize(ramps);
        return true;
    }

    Real sumVSqr = v0sqr + v1sqr;
    Real sigma = d - dStraight > 0 ? 1 : -1;
    Real a0 = sigma*am;
    Real vp = sigma*Sqrt((0.5*sumVSqr) + (a0*d));
    Real t0 = (vp - v0)/a0;
    Real t1 = (vp - v1)/a0;

    Ramp ramp0(v0, a0, t0, x0);
    Ramp ramp1(ramp0.v1, -a0, t1);

    std::vector<Ramp> ramps(2);
    ramps[0] = ramp0;
    ramps[1] = ramp1;
    curveOut.Initialize(ramps);

    ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf,  inf, inf, am, x0, x1, v0, v1);
    if (ret == PCR_Normal) {
        return true;
    }
    else {
        RAMP_OPTIM_PLOG("CheckParabolicCurve returns %d", ret);
        return false;
    }
}


bool ImposeVelocityLimit(ParabolicCurve& curve, Real vm) {
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

    Real vp = curve.ramps[0].v1;
    if (Abs(vp) <= vm + epsilon) {
        // The initial curve does not violate the constraint
        return true;
    }

    Real a0 = curve.ramps[0].a;
    Real h = Abs(vp) - vm;
    Real t = h/Abs(a0); // we are sure that a0 is non-zero

    std::vector<Ramp> ramps(3);
    Ramp newRamp0(curve.ramps[0].v0, curve.ramps[0].a, curve.ramps[0].duration - t, curve.ramps[0].x0);
    ramps[0] = newRamp0;

    Real nom = h*h;
    Real denom = Abs(a0)*vm;
    Real newVp = Sign(vp)*vm;
    Ramp newRamp1(newVp, 0, 2*t + (nom/denom));
    ramps[1] = newRamp1;

    Ramp newRamp2(newVp, curve.ramps[2].a, curve.ramps[2].duration - t);
    ramps[2] = newRamp2;

    curve.Initialize(ramps);
    return true;
}


inline Real SolveBrakeTime(Real x, Real v, Real xbound) {
    Real bt;
    bool res = SafeEqSolve(v, 2*(xbound - x), epsilon, 0, inf, bt);
    if (!res) {
        RAMP_OPTIM_WARN("Cannot solve the brake time equation: %.15e*t - %.15e = 0 with t being in [0, inf)", v, 2*(xbound - x));
        bt = 0;
    }
    return bt;
}


inline Real SolveBrakeAccel(Real x, Real v, Real xbound) {
    Real ba;
    Real coeff0 = 2*(xbound - x);
    Real coeff1 = v*v;
    bool res = SafeEqSolve(coeff0, -coeff1, epsilon, -inf, inf, ba);
    if (!res) {
        RAMP_OPTIM_WARN("Cannot solve the brake acceleration equation: %.15e*a + %.15e = 0 with a being in (-inf, inf)", coeff0, coeff1);
        ba = 0;
    }
    return ba;
}


bool ImposeJointLimitFixedDuration(ParabolicCurve& curveIn, Real xmin, Real xmax, Real vm, Real am, ParabolicCurve& curveOut) {
    /*
       This function for fixing x-bound violation is taken from OpenRAVE ParabolicPathSmooth library.
     */
    Real duration = curveIn.duration;
    Real x0 = curveIn.x0;
    Real x1 = curveIn.EvalPos(duration);
    Real v0 = curveIn.v0;
    Real v1 = curveIn.v1;

    Real bt0 = inf, ba0 = inf, bx0 = inf;
    Real bt1 = inf, ba1 = inf, bx1 = inf;
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
    Real tempbmin, tempbmax;

    if ((bt0 < duration) && (Abs(ba0) <= am + epsilon)) {
        RAMP_OPTIM_PLOG("Case IIa: checking");
        Ramp firstRamp(v0, ba0, bt0, x0);
        if (Abs(x1 - bx0) < (duration - bt0)*vm) {
            ParabolicCurve tempCurve1;
            if (Interpolate1D(bx0, x1, 0, v1, vm, am, tempCurve1)) {
                if ((duration - bt0) >= tempCurve1.duration) {
                    ParabolicCurve tempCurve2;
                    if (Stretch1D(tempCurve1, duration - bt0, vm, am, tempCurve2)) {
                        tempCurve2.GetPeaks(tempbmin, tempbmax);
                        if ((tempbmin >= xmin - epsilon) && (tempbmax <= xmax + epsilon)) {
                            RAMP_OPTIM_PLOG("Case IIa: successful");
                            newRamps.reserve(1 + tempCurve2.ramps.size());
                            newRamps.push_back(firstRamp);
                            for (size_t i = 0; i < tempCurve2.ramps.size(); ++i) {
                                newRamps.push_back(tempCurve2.ramps[i]);
                            }
                        }
                    }
                }
            }
        }
    }

    if ((bt1 < duration) && (Abs(ba1) <= am + epsilon)) {
        RAMP_OPTIM_PLOG("Case IIb: checking");
        Ramp lastRamp(0, ba1, bt1, bx1);
        if (Abs(x0 - bx1) < (duration - bt1)*vm) {
            ParabolicCurve tempCurve1;
            if (Interpolate1D(x0, bt1, v0, 0, vm, am, tempCurve1)) {
                if ((duration - bt1) >= tempCurve1.duration) {
                    ParabolicCurve tempCurve2;
                    if (Stretch1D(tempCurve1, duration - bt1, vm, am, tempCurve2)) {
                        tempCurve2.GetPeaks(tempbmin, tempbmax);
                        if ((tempbmin >= xmin - epsilon) && (tempbmax <= xmax + epsilon)) {
                            RAMP_OPTIM_PLOG("Case IIa: successful");
                            if (newRamps.size() > 0) {
                                newRamps.resize(0);
                            }
                            newRamps.reserve(1 + tempCurve2.ramps.size());
                            for (size_t i = 0; i < tempCurve2.ramps.size(); ++i) {
                                newRamps.push_back(tempCurve2.ramps[i]);
                            }
                            newRamps.push_back(lastRamp);
                        }
                    }
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
                ParabolicCurve tempCurve1;
                if (Interpolate1D(bx0, bx1, 0, 0, vm, am, tempCurve1)) {
                    if ((duration - (bt0 + bt1)) >= tempCurve1.duration) {
                        ParabolicCurve tempCurve2;
                        if (Stretch1D(tempCurve1, duration - (bt0 + bt1), vm, am, tempCurve2)) {
                            tempCurve2.GetPeaks(tempbmin, tempbmax);
                            if ((tempbmin >= xmin - epsilon) && (tempbmax <= xmax + epsilon)) {
                                RAMP_OPTIM_PLOG("Case IV: successful");
                                if (newRamps.size() > 0) {
                                    newRamps.resize(0);
                                }
                                newRamps.reserve(2 + tempCurve2.ramps.size());
                                newRamps.push_back(firstRamp);
                                for (size_t i = 0; i < tempCurve2.ramps.size(); ++i) {
                                    newRamps.push_back(tempCurve2.ramps[i]);
                                }
                                newRamps.push_back(lastRamp);
                            }
                        }
                    }
                }
            }
        }
    }

    if (newRamps.empty()) {
        RAMP_OPTIM_WARN("Cannot solve for a bounded trajectory");
        RAMP_OPTIM_WARN("Curve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", x0, x1, v0, v1, duration, xmin, xmax, vm, vm);
        return false;
    }

    curveOut.Initialize(newRamps);
    ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, xmin, xmax, vm, vm, x0, x1, v0, v1);
    if (ret == PCR_Normal) {
        return true;
    }
    else {
        RAMP_OPTIM_PLOG("CheckParabolicCurve returns %d", ret);
        return false;
    }
}


bool Stretch1D(ParabolicCurve& curveIn, Real newDuration, Real vm, Real am, ParabolicCurve& curveOut) {
    /*
       We want to 'stretch' this velocity profile to have a new duration of endTime. First, try
       re-interpolating this profile to have two ramps. If that doesn't work, try modifying the
       profile accordingly.

       Two-ramp case: let t = endTime (the new duration that we want), a0 and a1 the new
       accelerations of the profile, t0 the duration of the new first ramp.

       Starting from

              d = (v0*t0 + 0.5*a0*(t0*t0)) + ((v0 + a0*t0)*t1 + 0.5*a1*(t1*t1)),

       where d is the displacement done by this trajectory, t1 = newDuration - t0, i.e., the
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
    if (!(newDuration > 0)) {
        RAMP_OPTIM_PLOG("newDuration is negative");
        return false;
    }

    Real x0 = curveIn.x0;
    Real x1 = curveIn.EvalPos(curveIn.duration);
    Real v0 = curveIn.v0;
    Real v1 = curveIn.v1;

    Real d = x1 - x0; // displacement made by this profile
    Real t0, t1, vp, a0, a1;
    Real A, B, C, D; // temporary variables for solving equations

    Real newDurInverse = 1/newDuration;
    A = (v1 - v0)*newDurInverse;
    B = (2*d)*newDurInverse - (v0 + v1);
    Real sum1 = -am - A;
    Real sum2 = am - A;
    C = B/sum1;
    D = B/sum2;

    // RAMP_OPTIM_PLOG("A = %.15e; B = %.15e, C = %.15e, D = %.15e; sum1 = %.15e; sum2 = %.15e", A, B, C, D, sum1, sum2);
    if ((Abs(A) < epsilon) && (Abs(B) < epsilon)) {
        RAMP_OPTIM_PLOG("A and B are zero");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
    }

    // Now we need to check a number of feasible intervals of tswitch1 induced by constraints on the
    // acceleration. Instead of having a class representing an interval, we use the interval bounds
    // directly. Naming convention: iXl = lower bound of interval X, iXu = upper bound of interval X.
    Real i0l = 0, i0u = newDuration;
    Real i1l = -Inf, i1u = Inf;
    Real i2l = -Inf, i2u = Inf;
    Real i3l = -Inf, i3u = Inf;
    Real i4l = -Inf, i4u = Inf;

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
        // i1 = (-Inf, C]
        i1u = C;
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
        // i2 = (-Inf, D]
        i2u = D;
    }

    // Find the intersection between interval 1 and interval 2, store it in interval 2.
    if ((i1l > i2u) || (i1u < i2l)) {
        RAMP_OPTIM_PLOG("interval 1 and interval 2 do not have any intersection");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
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
        // i3 = [t + C, Inf)
        i3l = newDuration + C;
    }
    else {
        // i3 = (-Inf, t + C]
        i3u = newDuration + C;
    }

    // IV)
    if (sum2 == 0) {
        if (B == 0) {
            // t0 can be anything
        }
        else {
            i2l = Inf;
        }
    }
    else if (sum2 > 0) {
        // i4 = (-Inf, t + D]
        i4u = newDuration + D;
    }
    else {
        // i4 = [t + D, INf)
        i4l = newDuration + D;
    }

    // Find the intersection between interval 3 and interval 4, store it in interval 4.
    if ((i3l > i4u) || (i3u < i4l)) {
        RAMP_OPTIM_PLOG("interval 3 and interval 4 do not have any intersection");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
        return false;
    }
    else {
        i4l = Max(i3l, i4l);
        i4u = Min(i3u, i4u);
    }

    // Find the intersection between interval 2 and interval 4, store it in interval 4. This is a
    // bit tricky because if the given newDuration is actually the minimum time that this trajectory
    // can get, the two intervals will theoretically intersect at only one point.
    if (FuzzyEquals(i2l, i4u, epsilon) || FuzzyEquals(i2u, i4l, epsilon)) {
        RAMP_OPTIM_PLOG("interval 2 and interval 4 intersect at a point, most likely because the given endTime is actually its minimum time.");
        // Make sure that the above statement is true.
        if (!Interpolate1D(x0, x1, v0, v1, vm, am, curveOut)) {
            return false; // what ?
        }
        else {
            if (FuzzyEquals(curveOut.duration, newDuration, epsilon)) {
                RAMP_OPTIM_PLOG("The hypothesis is correct.");
                return true;
            }
            else {
                RAMP_OPTIM_PLOG("The hypothesis is wrong. Something else just happened");
                RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
                return false; // what ?
            }
        }
    }
    else if ((i2l > i4u) || (i2u < i4l)) {
        RAMP_OPTIM_PLOG("interval 2 and interval 4 do not have any intersection");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
        return false;
    }
    else {
        i4l = Max(i2l, i4l);
        i4u = Min(i2u, i4u);
    }

    // Find the intersection between interval 0 and interval 4, store it in interval 4.
    if ((i0l > i4u) || (i0u < i4l)) {
        RAMP_OPTIM_PLOG("interval 0 and interval 4 do not have any intersection");
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
        return false;
    }
    else {
        i4l = Max(i0l, i4l);
        i4u = Min(i0u, i4u);
    }

    /*
       Now we have already obtained a range of feasible values for t0. We choose a value of
       t0 by selecting the one which minimize J(t0) := (a0^2 + a1^2).

       Now let x = t0 for convenience. We can write J(x) as

              J(x) = (A + B/x)^2 + (A - B/(t - x))^2.

       Then we find x which minimizes J(x) by examining the roots of dJ/dx.
     */
    bool res = SolveForT0(A, B, newDuration, i4l, i4u, t0);
    if (!res) {
        // Solving dJ/dx = 0 somehow failed. We just choose the midpoint of the feasible interval.
        t0 = 0.5*(i4l + i4u);
    }

    // Here we need to take care of the cases when t0 has been rounded. In particular, we need to go
    // back to the original formulae to calculated other related values (such as a1). Otherwise, it
    // may cause discrepancies.
    if (FuzzyZero(t0, epsilon) || FuzzyEquals(t0, newDuration, epsilon)) {
        // t0 is either 0 or newDuration. This means the new trajectory will consist of only one
        // Ramp. Since v0 and v1 are withint the limits, we are safe.
        Ramp ramp0(v0, A, newDuration, x0);
        std::vector<Ramp> ramps(1);
        ramps[0] = ramp0;
        curveOut.Initialize(ramps);
        return true;
    }

    t1 = newDuration - t0;
    a0 = A + B/t0;
    a1 = A - B/t1;
    vp = v0 + (a0*t0);

    // Consistency checking
    if (!FuzzyEquals(vp, v1 - (a1*t1), epsilon)) {
        RAMP_OPTIM_PLOG("Verification failed (vp != v1 - a1*d1): %.15e != %.15e", vp, v1 - (a1*t1));
        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
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
        ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
        if (ret == PCR_Normal) {
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("Finished stretching but the profile does not pass the check: ret = %d", ret);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
            RAMP_OPTIM_PLOG("Calculated values: A = %.15e; B = %.15e; t0 = %.15e; t1 = %.15e; vp = %.15e; a0 = %.15e; a1 = %.15e", A, B, t0, t1, vp, a0, a1);
            return false;
        }
    }
    else {
        // The two-ramp profile does not work because it violates the velocity bounds. Modify it
        // accordingly.
        Real vmNew = vp > 0 ? vm : -vm;

        // a0 and a1 should not be zero if the velocity limit is violated. The first check is done
        // at the line: FuzzyEquals(vp, v1 - (a1*t1)) above.
        if ((FuzzyZero(a0, epsilon)) || (FuzzyZero(a1, epsilon))) {
            RAMP_OPTIM_PLOG("Velocity limit is violated but at least one acceleration is zero: a0 = %.15e; a1 = %.15e", a0, a1);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
            return false;
        }
        Real a0inv = 1/a0;
        Real a1inv = 1/a1;

        Real dv1 = vp - vmNew;
        Real dv2 = vmNew - v0;
        Real dv3 = vmNew - v1;
        Real t0Trimmed = dv2*a0inv; // vmaxNew = dx0 + a0*t0Trimmed
        Real t1Trimmed = -dv3*a1inv; // dx1 = vmaxNew + a1*t1Trimmed

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

        Real A2 = dv2*dv2;
        Real B2 = -dv3*dv3;
        Real D2 = 0.5*dv1*(newDuration - t0Trimmed - t1Trimmed); // area of the velocity profile above the velocity limit.
        Real C2 = t0Trimmed*dv2 + t1Trimmed*dv3 - 2*D2;

        Real root = cbrt(A2*B2*B2); // from math.h

        if (FuzzyZero(C2, epsilon)) {
            // This means the excessive area is too large such that after we paste it on both sides
            // of the original velocity profile, the whole profile becomes one-ramp with a = 0 and v
            // = vmNew.
            RAMP_OPTIM_PLOG("C2 == 0. Unable to fix this case.");
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
            return false;
        }

        Real C2inv = 1/C2;
        a0 = (A2 + root)*C2inv;
        if (Abs(a0) > am) {
            // a0 exceeds the bound, try making it stays at the bound.
            a0 = a0 > 0 ? am: -am;
            // Recalculate the related variable
            root = C2*a0 - A2;
        }

        // Now compute a1
        // Special case: a0 == 0. Then this implies vm == dx0. Reevaluate those above equations
        // leads to a1 = B2/C2
        if (Abs(a0) <= epsilon) {
            a0 = 0;
            a1 = B2/C2;
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
                        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
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
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
            return false;
        }

        if ((Abs(a0) <= epsilon) && (Abs(a1) <= epsilon)) {
            RAMP_OPTIM_PLOG("Both accelerations are zero.");
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
            RAMP_OPTIM_PLOG("A2 = %.15e; B2 = %.15e; C2 = %.15e; D2 = %.15e", A2, B2, C2, D2);
            return false;
        }

        if (Abs(a0) <= epsilon) {
            t0 = newDuration + dv3/a1;
            t1 = newDuration - t0;
            vp = vmNew;
            
            Ramp ramp0(v0, a0, t0, x0);
            Ramp ramp1(vp, a1, t1);
            std::vector<Ramp> ramps(2);
            ramps[0] = ramp0;
            ramps[1] = ramp1;
            curveOut.Initialize(ramps);
        }
        else if (Abs(a1) <= epsilon) {
            t0 = dv2/a0;
            t1 = newDuration - t0;
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
            vp = vmNew;
            Real tLastRamp = -dv3/a1;
            if (t0 + tLastRamp > newDuration) {
                // Final fix
                if (A == 0) {
                    RAMP_OPTIM_PLOG("(final fix) A = 0. Don't know how to deal with this case.");
                    RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
                    return false;
                }
                t0 = (dv2 - B)/A; // note that we use A and B, not A2 and B2.
                if (t0 < 0) {
                    RAMP_OPTIM_PLOG("(final fix) t0 < 0. Don't know how to deal with this case.");
                    RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
                    return false;
                }
                t1 = newDuration - t0;
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
                Real tMiddle = newDuration - (t0 + tLastRamp);
                if (FuzzyZero(tMiddle, epsilon)) {
                    RAMP_OPTIM_PLOG("Three-ramp profile works but having too short middle ramp.");
                    // If we leave it like this, it may cause errors later on.
                    t0 = (2*d - (v1 + vmNew)*newDuration)/(v0 - v1);
                    t1 = newDuration - t0;
                    vp = vmNew;
                    a0 = dv2/t0;
                    a1 = -dv3/t1;
                    if ((Abs(a0) > am + epsilon) || (Abs(a1) > am + epsilon)) {
                        RAMP_OPTIM_PLOG("Cannot merge into two-ramp because of acceleration limits");
                        RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
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

        ParabolicCheckReturn ret = CheckParabolicCurve(curveOut, -inf, inf, vm, am, x0, x1, v0, v1);
        if (ret == PCR_Normal) {
            return true;
        }
        else {
            RAMP_OPTIM_PLOG("Finished stretching but the profile does not pass the check: ret = %d", ret);
            RAMP_OPTIM_PLOG("ParabolicCurve info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; newDuration = %.15e", x0, x1, v0, v1, vm, am, newDuration);
            return false;
        }
    }
}

/*
   Utilities
 */
bool CalculateLeastUpperBoundInoperativeInterval(Real x0, Real x1, Real v0, Real v1, Real vm, Real am, Real& t) {
    Real d = x1 - x0;
    Real T0, T1, T2, T3;

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
     */

    Real firstTerm = (v0 + v1)/am;

    Real temp1 = 2*(-Sqr(am))*(2*am*d - Sqr(v0) - Sqr(v1));
    Real secondTerm1 = Sqrt(temp1)/Sqr(am);
    if (temp1 < 0) {
        T0 = -1;
        T1 = -1;
    }
    else {
        T0 = firstTerm + secondTerm1;
        T1 = firstTerm - secondTerm1;
    }
    T1 = Max(T0, T1);

    Real temp2 = 2*(Sqr(am))*(2*am*d + Sqr(v0) + Sqr(v1));
    Real secondTerm2 = Sqrt(temp2)/Sqr(am);
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
        Real dStraight = 0.5*(v0 + v1)*t;
        Real amNew = d - dStraight > 0 ? am : -am;
        Real vmNew = d - dStraight > 0 ? vm : -vm;

        Real vp = 0.5*(amNew*t + v0 + v1); // the peak velocity
        if (Abs(vp) > vm) {
            Real dExcess = (vp - vmNew)*(vp - vmNew)/am;
            Real deltaTime = dExcess/vm;
            t += deltaTime; // the time increased from correcting the velocity bound violation
        }
        // Should be no problem now.
        t = t * 1.01; // for safety reasons, we don't make t too close to the bound
        return true;
    }
    else {
        RAMP_OPTIM_PLOG("Unable to calculate the least upper bound: T0 = %.15e, T1 = %.15e, T2 = %.15e, T3 = %.15e", T0, T1, T2, T3);
        return false;
    }
}


bool SolveForT0(Real A, Real B, Real t, Real l, Real u, Real& t0) {
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

    Real rawRoots[4];
    int numRoots;
    double tSqr = t*t;
    double tCube = tSqr*t;

    if (Abs(A) < epsilon) {
        Real coeffs[4] = {2*B, -3*B*t, 3*B*tSqr, -B*tCube};
        OpenRAVE::mathextra::polyroots<Real, 3>(&coeffs[0], &rawRoots[0], numRoots);
    }
    else {
        Real coeffs[5] = {2*A, -4*A*t + 2*B, 3*A*tSqr - 3*B*t, -A*tCube + 3*B*tSqr, -B*tCube};
        OpenRAVE::mathextra::polyroots<Real, 4>(&coeffs[0], &rawRoots[0], numRoots);
    }

    if (numRoots == 0) {
        return false;
    }

    // Among all the solutions, find the one that minimizes the objective function.
    Real J = Inf;
    Real bestT = -1;
    for (int i = 0; i < numRoots; ++i) {
        if ((rawRoots[i] <= u) && (rawRoots[i] >= l)) {
            Real root = rawRoots[i];
            Real firstTerm, secondTerm;
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
            Real curObj = firstTerm*firstTerm + secondTerm*secondTerm;
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
