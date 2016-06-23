//-*- coding: utf-8 -*-
#include "interpolation.h"

#include <iostream>
#include <boost/format.hpp>

namespace RampOptimizerInternal {

////////////////////////////////////////////////////////////////////////////////
// Multi DOF
ParabolicCurvesND InterpolateZeroVelND(std::vector<Real>& x0Vect, std::vector<Real>& x1Vect,
                                       std::vector<Real>& vmVect, std::vector<Real>& amVect, Real delta) {
    int ndof = x0Vect.size();
    BOOST_ASSERT(ndof == (int) x1Vect.size());
    BOOST_ASSERT(ndof == (int) vmVect.size());
    BOOST_ASSERT(ndof == (int) amVect.size());

    std::vector<Real> dVect(ndof);
    std::vector<ParabolicCurve> curves;
    curves.reserve(ndof);
    // Calculate sdMax (vMin) and sddMax (aMin)
    Real vMin = inf;
    Real aMin = inf;
    for (int i = 0; i < ndof; ++i) {
        dVect[i] = x1Vect[i] - x0Vect[i]; ///<
        if (!FuzzyZero(dVect[i], epsilon)) {
            vMin = Min(vMin, vmVect[i]/Abs(dVect[i]));
            aMin = Min(aMin, amVect[i]/Abs(dVect[i]));
        }
        curves.push_back(*(new ParabolicCurve()));
    }
    BOOST_ASSERT(vMin < inf);
    BOOST_ASSERT(aMin < inf);

    ParabolicCurve sdProfile;
    if (FuzzyZero(delta, epsilon)) {
        sdProfile = Interpolate1D(0, 1, 0, 0, vMin, aMin);
    }
    else {
        // Not implemented yet
        BOOST_ASSERT(false);
    }

    // Scale each input dof according to the obtained sd-profile
    for (std::vector<Ramp>::const_iterator itRamp = sdProfile.ramps.begin(); itRamp != sdProfile.ramps.end(); ++itRamp) {
        Real sd0, sdd;
        Real dur = itRamp->duration;
        for (int i = 0; i < ndof; ++i) {
            sd0 = dVect[i] * itRamp->v0;
            sdd = dVect[i] * itRamp->a;
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

    ParabolicCurvesND curvesnd(curves);

    return curvesnd;
}

ParabolicCurvesND InterpolateArbitraryVelND(std::vector<Real>& x0Vect, std::vector<Real>& x1Vect,
                                            std::vector<Real>& v0Vect, std::vector<Real>& v1Vect,
                                            std::vector<Real>& vmVect, std::vector<Real>& amVect, Real delta) {
    int ndof = x0Vect.size();
    BOOST_ASSERT(ndof == (int) x1Vect.size());
    BOOST_ASSERT(ndof == (int) v0Vect.size());
    BOOST_ASSERT(ndof == (int) v1Vect.size());
    BOOST_ASSERT(ndof == (int) vmVect.size());
    BOOST_ASSERT(ndof == (int) amVect.size());

    std::vector<Real> dVect(ndof);
    for (int i = 0; i < ndof; ++i) {
        dVect[i] = x1Vect[i] - x0Vect[i]; ///<
    }

    // First independently interpolate each DOF to find the slowest DOF
    std::vector<ParabolicCurve> curves(ndof);
    Real maxDuration = 0;
    int maxIndex;
    if (FuzzyZero(delta, epsilon)) {
        for (int i = 0; i < ndof; ++i) {
            curves[i] = Interpolate1D(x0Vect[i], x1Vect[i], v0Vect[i], v1Vect[i], vmVect[i], amVect[i]);
            if (curves[i].duration > maxDuration) {
                maxDuration = curves[i].duration;
                maxIndex = i;
            }
        }
    }
    else {
        // Not implemented yet
        BOOST_ASSERT(false);
    }
    // std::cout << "maxIndex = " << maxIndex << std::endl;
    return ReinterpolateNDFixedDuration(curves, vmVect, amVect, maxIndex, delta);
}


ParabolicCurvesND ReinterpolateNDFixedDuration(std::vector<ParabolicCurve> curves,
                                               std::vector<Real> vmVect, std::vector<Real> amVect, int maxIndex, Real delta) {
    int ndof = curves.size();
    BOOST_ASSERT(ndof == (int) vmVect.size());
    BOOST_ASSERT(ndof == (int) amVect.size());
    BOOST_ASSERT(maxIndex < ndof);

    ParabolicCurvesND curvesnd;
    Real newDuration = curves[maxIndex].duration;
    std::vector<ParabolicCurve> newCurves(ndof);
    for (int i = 0; i < ndof; ++i) {
        if (i == maxIndex) {
            newCurves[i] = curves[i];
        }
        else {
            newCurves[i] = Stretch1D(curves[i], newDuration, vmVect[i], amVect[i]);
            if (newCurves[i].IsEmpty()) {
                return curvesnd; // return an empty ParabolicCurvesND
            }
        }
    }
    curvesnd.Initialize(newCurves);
    return curvesnd;
}

////////////////////////////////////////////////////////////////////////////////
// Single DOF
ParabolicCurve Interpolate1D(Real x0, Real x1, Real v0, Real v1, Real vm, Real am, Real delta) {
    BOOST_ASSERT(vm > 0);
    BOOST_ASSERT(am > 0);
    BOOST_ASSERT(delta > -epsilon);
    BOOST_ASSERT(Abs(v0) < vm + epsilon);
    BOOST_ASSERT(Abs(v1) < vm + epsilon);
    if (delta < 0) delta = 0;
    if (v0 > vm) v0 = vm;
    if (v1 > vm) v1 = vm;

    ParabolicCurve curve = Interpolate1DNoVelocityLimit(x0, x1, v0, v1, am);
    if (curve.ramps.size() == 1) {
        return curve;
    }
    if (delta > 0) {
        // Not implemented yet
        BOOST_ASSERT(false);
    }
    return ImposeVelocityLimit(curve, vm);
}

ParabolicCurve Interpolate1DNoVelocityLimit(Real x0, Real x1, Real v0, Real v1, Real am) {
    BOOST_ASSERT(am > 0);

    // Check for the appropriate direction of the acceleration of the first ramp.
    Real d = x1 - x0;
    Real dv = v1 - v0;
    Real difVSqr = (v1*v1) - (v0*v0);

    Real dStraight; // displacement obtained when maximally accelerate/decelerate from v0 to v1
    if (Abs(dv) < epsilon) {
        if (Abs(d) < epsilon) {
            Ramp ramp0(0, 0, 0, x0);
            std::vector<Ramp> ramps(1);
            ramps[0] = ramp0;
            return ParabolicCurve(ramps);
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
        Real a0 = Sign(dv) * am;
        Ramp ramp0(v0, a0, dv/a0, x0);
        std::vector<Ramp> ramps(1);
        ramps[0] = ramp0;
        return ParabolicCurve(ramps);
    }

    Real sumVSqr = (v0*v0) + (v1*v1);
    Real sigma = Sign(d - dStraight);
    Real a0 = sigma*am;
    Real vp = sigma*Sqrt((0.5*sumVSqr) + (a0*d));
    Real t0 = (vp - v0)/a0;
    Real t1 = (vp - v1)/a0;
    // std::cout << str(boost::format("t0 = %.15e, t1 = %.15e")%t0 %t1) << std::endl;
    Ramp ramp0(v0, a0, t0, x0);
    BOOST_ASSERT(FuzzyEquals(ramp0.v1, vp, epsilon));
    Ramp ramp1(ramp0.v1, -a0, t1);

    std::vector<Ramp> ramps(2);
    ramps[0] = ramp0;
    ramps[1] = ramp1;
    ParabolicCurve curve(ramps);
    BOOST_ASSERT(FuzzyEquals(curve.d, d, epsilon));
    return curve;
}

ParabolicCurve ImposeVelocityLimit(ParabolicCurve curve, Real vm) {
    BOOST_ASSERT(vm > 0);
    BOOST_ASSERT(curve.ramps.size() == 2); // may remove this later

    ParabolicCurve newCurve;
    if (Abs(curve.ramps[0].v0) > vm) {
        // Initial velocity violates the velocity constraint
        return newCurve;
    }
    if (Abs(curve.ramps[1].v1) > vm) {
        // Final velocity violates the velocity constraint
        return newCurve;
    }

    Real vp = curve.ramps[0].v1;
    if (Abs(vp) < vm) {
        // The initial curve does not violate the constraint
        return curve;
    }

    Real a0 = curve.ramps[0].a;
    Real h = Abs(vp) - vm;
    BOOST_ASSERT(h > 0);
    Real t = h/Abs(a0);

    std::vector<Ramp> ramps(3);
    std::vector<Ramp>::const_iterator itRamp = curve.ramps.begin();
    Ramp newRamp0(itRamp->v0, a0, itRamp->duration - t, itRamp->x0);
    ramps[0] = newRamp0;

    Real nom = h*h;
    Real denom = Abs(a0)*vm;
    Ramp newRamp1(Sign(vp)*vm, 0, 2*t + (nom/denom));
    ramps[1] = newRamp1;

    itRamp += 1;
    Ramp newRamp2(Sign(vp)*vm, itRamp->a, itRamp->duration - t);
    ramps[2] = newRamp2;

    newCurve.Initialize(ramps);
    return newCurve;
}

ParabolicCurve Stretch1D(ParabolicCurve curveIn, Real newDuration, Real vm, Real am) {
    BOOST_ASSERT(newDuration > curveIn.duration);
    BOOST_ASSERT(vm > 0);
    BOOST_ASSERT(am > 0);

    Real v0 = curveIn.ramps[0].v0;
    Real v1 = curveIn.ramps.back().v1;
    Real d = curveIn.d;

    // std::cout << str(boost::format("newDuration = %.15e")%newDuration) << std::endl;
    // std::cout << str(boost::format("vm = %.15e")%vm) << std::endl;
    // std::cout << str(boost::format("am = %.15e")%am) << std::endl;
    // std::cout << str(boost::format("v0 = %.15e")%v0) << std::endl;
    // std::cout << str(boost::format("v1 = %.15e")%v1) << std::endl;
    // std::cout << str(boost::format("d = %.15e")%d) << std::endl;

    ParabolicCurve curve;

    // First we assume no velocity bound -> the re-interpolated trajectory will have only two
    // ramps. Solve for a0 and a1 (the acceleration of the first and the last ramps).
    //         a1 = A + B/t0
    //         a2 = A + B/(t - t0)
    // where t is the (new) total duration, t0 is the (new) duration of the first ramp, and
    //         A = (v1 - v0)/t
    //         B = (2d/t) - (v0 + v1).
    Real A, B, C, D;
    Real newDurInverse = 1.0/newDuration;
    A = (v1 - v0) * newDurInverse;
    B = (2*d*newDurInverse) - (v0 + v1);

    // std::cout << str(boost::format("A = %.15e")%A) << std::endl;
    // std::cout << str(boost::format("B = %.15e")%B) << std::endl;
    // std::cout << str(boost::format("newDurInverse = %.15e")%newDurInverse) << std::endl;
    
    Interval interval0(0, newDuration); // initial interval for t0

    // Now consider the intervals computed from constraints on a0
    //         C = B/(-am - A)
    //         D = B/(am - A)
    Interval interval1, interval2;
    Real sum1 = -am - A;
    Real sum2 = am - A;
    C = B/sum1;
    D = B/sum2;
    // std::cout << str(boost::format("sum1 = %.15e")%sum1) << std::endl;
    // std::cout << str(boost::format("sum2 = %.15e")%sum2) << std::endl;
    // std::cout << str(boost::format("C = %.15e")%C) << std::endl;
    // std::cout << str(boost::format("D = %.15e")%D) << std::endl;
    if (FuzzyZero(sum1, epsilon)) {
        // Not implemented yet
        BOOST_ASSERT(false);
    }
    else if (sum1 > epsilon) {
        interval1.h = C;
    }
    else {
        interval1.l = C;
    }
    if (FuzzyZero(sum2, epsilon)) {
        // Not implemented yet
        BOOST_ASSERT(false);
    }
    else if (sum2 > epsilon) {
        interval2.l = D;
    }
    else {
        interval2.h = D;
    }
    // std::cout << str(boost::format("interval1 L = %.15e; H = %.15e")%interval1.l %interval1.h) << std::endl;
    // std::cout << str(boost::format("interval2 L = %.15e; H = %.15e")%interval2.l %interval2.h) << std::endl;
    interval2.Intersect(interval1);
    if (interval2.isVoid) {
        return curve;
    }
    

    // Now consider the intervals computed from constraints on a1
    Interval interval3, interval4;
    if (FuzzyZero(sum1, epsilon)) {
        // Not implemented yet
        BOOST_ASSERT(false);
    }
    else if (sum1 > epsilon) {
        interval3.l = C + newDuration;
    }
    else {
        interval3.h = C + newDuration;
    }
    if (FuzzyZero(sum2, epsilon)) {
        // Not implemented yet
        BOOST_ASSERT(false);
    }
    else if (sum2 > epsilon) {
        interval4.h = D + newDuration;
    }
    else {
        interval4.l = D + newDuration;
    }
    interval4.Intersect(interval3);
    if (interval4.isVoid) {
        return curve;
    }

    interval4.Intersect(interval2);
    if (interval4.isVoid) {
        return curve;
    }

    interval4.Intersect(interval0);
    if (interval4.isVoid) {
        return curve;
    }

    // Now we have already obtained a range of feasible values for t0. We choose a value of t0 by
    // selecting the one which minimize (a0^2 + a1^2).

    // An alternative is to choose t0 = 0.5(t0Max + t0Min), i.e., the midpoint of the interval (to
    // save computational effort). The midpoint, in my opinion, prevides a good approximation of the
    // real minimizer.
    Real t0;
    bool res = _SolveForT0(A, B, newDuration, interval4.l, interval4.h, t0);
    if (!res) {
        return curve;
    }

    Real t1 = newDuration - t0; ///<
    Real a0 = A + (B/t0);
    Real a1 = A - (B/t1);
    BOOST_ASSERT(am - Abs(a0) > epsilon); // check if a0 is really below the bound
    BOOST_ASSERT(am - Abs(a1) > epsilon); // check if a1 is really below the bound
    
    Real vp = v0 + (a0*t0); // the peak velocity
    if (Abs(vp) > vm + epsilon) {
        // Inside this if, we reuse the variables A, B, C, D
        // std::cout << "CURVE INFORMATION" << std::endl;
        // std::cout << str(boost::format("v0 = %.15e")%curveIn.v0) << std::endl;
        // std::cout << str(boost::format("v1 = %.15e")%curveIn.v1) << std::endl;
        // std::cout << str(boost::format("x0 = %.15e")%curveIn.x0) << std::endl;
        // std::cout << str(boost::format("x1 = %.15e")%curveIn.EvalPos(curveIn.duration)) << std::endl;
        // std::cout << str(boost::format("vm = %.15e")%vm) << std::endl;
        // std::cout << str(boost::format("am = %.15e")%am) << std::endl;
        // std::cout << str(boost::format("newDuration = %.15e")%newDuration) << std::endl;
        // std::cout << str(boost::format("a0 = %.15e")%a0) << std::endl;
        // std::cout << str(boost::format("a1 = %.15e")%a1) << std::endl;
        
        Real vmNew = Sign(vp)*vm; // the max velocity to be used in the preceding formulae (taking into account the sign)
        Real dv = vp - vmNew;
        D = 0.5*dv*dv*((1/a0) - (1/a1)); // area under the curve above vm (or below -vm)
        dv = vmNew - v0;
        A = dv*dv;
        Real t0Trimmed = dv/a0; // the duration of the trimmed first ramp (removing the part above vm)
        dv = v1 - vmNew;
        B = -dv*dv;
        Real t1Trimmed = dv/a1; // the duration of the trimmed last ramp (removing the part above vm)
        C = t0Trimmed*(vmNew - v0) + t1Trimmed*(vmNew - v1) - 2*D;
        Real temp = A*B*B;
        Real rawRoots[3];
        Real root;
        bool hasRoot = false;
        int numRoots;
        Real coeffs[4] = {1, 0, 0, -temp};
        FindPolyRoots3(coeffs, rawRoots, numRoots);
        if (numRoots == 0) {
            BOOST_ASSERT("CANNOT SOLVE A CUBIC EQUATION.");
        }
        else {
            // std::cout << str(boost::format("temp = %.15e")%temp) << std::endl;
            // std::cout << "NUM ROOTS" << numRoots << std::endl;
            for (int k = 0; k < numRoots; ++k) {
                // std::cout << "ROOT " << k << " " << rawRoots[k] << std::endl;
                if (FuzzyEquals(rawRoots[k]*rawRoots[k]*rawRoots[k], temp, temp*epsilon)) {
                    root = rawRoots[k];
                    hasRoot = true;
                }
            }
            // root = rawRoots[0];
            // std::cout << str(boost::format("temp = %.15e")%temp) << std::endl;
            // std::cout << str(boost::format("root = %.15e")%root) << std::endl;
            // std::cout << str(boost::format("cubed = %.15e")%(root*root*root)) << std::endl;
            // BOOST_ASSERT(FuzzyEquals(root*root*root, temp, epsilon));
        }
        BOOST_ASSERT(hasRoot);
        // std::cout << str(boost::format("A = %.15e")%A) << std::endl;
        // std::cout << str(boost::format("B = %.15e")%B) << std::endl;
        // std::cout << str(boost::format("C = %.15e")%C) << std::endl;
        // std::cout << str(boost::format("D = %.15e")%D) << std::endl;

        
        Real a0New = (A + root)/C;
        if (Abs(a0New) > am + epsilon) {
            // a0New exceeds the bound, make it stay at the bound
            a0New = Sign(a0New)*am;
        }
        Real a1New = (B/C)*(1 + A/(C*a0New - A));
        if (Abs(a1New) > am + epsilon) {
            // a1New exceeds the bound, make it stay at the bound. Also modify a0New acordingly.
            a1New = Sign(a1New)*am;
            a0New = (A/C)*(1 + B/(C*a1New - B));
        }

        if ((Abs(a0New) > am + epsilon) or (Abs(a1New) > am + epsilon)) {
            // After modification, the acceleration constraint is still violated.
            // No heuristic to handle this case yet.
            return curve;
        }
        // std::cout << str(boost::format("a0 = %.15e")%a0) << std::endl;
        // std::cout << str(boost::format("a0New = %.15e")%a0New) << std::endl;
        // std::cout << str(boost::format("a1 = %.15e")%a1) << std::endl;
        // std::cout << str(boost::format("a1New = %.15e")%a1New) << std::endl;

        Real t0New = (vmNew - v0)/a0New;
        // std::cout << str(boost::format("t0New = %.15e")%t0New) << std::endl;
        BOOST_ASSERT(t0New > 0);
        Ramp ramp0(v0, a0New, t0New, curveIn.x0);

        Real t1New = (v1 - vmNew)/a1New;
        // std::cout << str(boost::format("t1New = %.15e")%t1New) << std::endl;
        BOOST_ASSERT(t1New > 0);
        Ramp ramp2(ramp0.v1, a1New, t1New);

        Ramp ramp1(ramp0.v1, 0, newDuration - (t0New + t1New));

        std::vector<Ramp> ramps(3);
        ramps[0] = ramp0;
        ramps[1] = ramp1;
        ramps[2] = ramp2;
        curve.Initialize(ramps);
        return curve;
        
    }
    else {
        Ramp ramp0(v0, a0, t0, curveIn.x0);
        Ramp ramp1(ramp0.v1, a1, t1);
        std::vector<Ramp> ramps(2);
        ramps[0] = ramp0;
        ramps[1] = ramp1;
        curve.Initialize(ramps);
        return curve;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Utilities
bool _SolveForT0(Real A, Real B, Real t, Real tLow, Real tHigh, Real& t0) {
    Real rawRoots[4];
    int numRoots;
    double tSqr = t*t;
    double tCube = tSqr*t;
    Real coeffs[5] = {2*A, -4*A*t + 2*B, 3*A*tSqr - 3*B*t, -A*tCube + 3*B*tSqr, -B*tCube}; ///<
    FindPolyRoots4(coeffs, rawRoots, numRoots);
    if (numRoots == 0) {
        return false;
    }
    else {
        for (int i = 0; i < numRoots; ++i) {
            if ((rawRoots[i] <= tHigh) && (rawRoots[i] >= tLow)) {
                t0 = rawRoots[i];
                return true;
            }
        }
        return false; // solutions not in the provided range
    }
}

} // end namespace RampOptimizerInternal
