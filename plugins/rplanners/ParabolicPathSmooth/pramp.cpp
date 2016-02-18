// -*- coding: utf-8 -*-
#include "pramp.h"

namespace ParabolicRampInternal {

Real ParabolicRamp::Evaluate(Real t) const
{
    return x0 + t*dx0 + 0.5*a*Sqr(t);
}

Real ParabolicRamp::Derivative(Real t) const
{
    return dx0 + a*t;
}

Real ParabolicRamp::Accel(Real t) const
{
    return a;
}

bool ParabolicRamp::Solve(Real amax)
{
    //x0 + tv0 + t^2/2 a = x1
    //v0 + ta = v1
    //ta = v1-v0
    //t = 2*(x1-x0)/(v1+v0)
    //a = (v1-v0)/t
    Real numerator = 2.0*(x1-x0);
    Real denominator = dx1+dx0;
    bool res = SafeEqSolve(denominator,numerator,EpsilonT,0.0,Inf,ttotal);
    if(!res) {
        return false;
    }
    PARABOLIC_RAMP_ASSERT(ttotal >= 0);

    //pick some value of a that satisfies |ta - v1-v0| < epsilonV
    res = SafeEqSolve(ttotal,dx1-dx0,EpsilonV,-amax,amax,a);
    if(!res) {
        return false;
    }
    if( Abs(a) > amax+EpsilonA ) {
        return false;
    }
    if( a >= amax ) {
        a = amax;
    }
    else if( a <= -amax ) {
        a = -amax;
    }
    //PARABOLIC_RAMP_ASSERT(FuzzyEquals(Evaluate(ttotal),x1,EpsilonX) && FuzzyEquals(Derivative(ttotal),dx1,EpsilonV));

    if(FuzzyEquals(Evaluate(ttotal),x1,EpsilonX) && FuzzyEquals(Derivative(ttotal),dx1,EpsilonV)) {
        return true;
    }
    return false;
}

bool ParabolicRamp::SolveFixedTime(Real endTime)
{
    if(!FuzzyEquals(endTime*(dx1+dx0),2.0*(x1-x0),EpsilonX)) {
        return false;
    }
    ttotal = endTime;
    bool res = SafeEqSolve(ttotal,dx1-dx0,EpsilonV,-Inf,Inf,a);
    if(!res) {
        return false;
    }
    if(FuzzyEquals(Evaluate(ttotal),x1,EpsilonX) && FuzzyEquals(Derivative(ttotal),dx1,EpsilonV)) {
        return true;
    }
    return false;
}

Real ParabolicRamp::GetMaxSpeed() const
{
    Real f0 = fabs(dx0), f1=fabs(dx1);
    if(f0 > f1) {
        return f0;
    }
    else {
        return f1;
    }
}

}
