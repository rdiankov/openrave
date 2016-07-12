// -*- coding: utf-8 -*-
#ifndef PARABOLIC_PRAMP_H
#define PARABOLIC_PRAMP_H

#include "paraboliccommon.h"

namespace ParabolicRampInternal {

class ParabolicRamp
{
public:
    Real Evaluate(Real t) const;
    Real Derivative(Real t) const;
    Real Accel(Real t) const;
    bool Solve(Real amax);
    bool SolveFixedTime(Real endTime);

    /// \brief returns max speed (absolute value of velocity) along ramp
    Real GetMaxSpeed() const;

    //input
    Real x0,dx0;
    Real x1,dx1;

    //calculated
    Real a;
    Real ttotal;
};

}

#endif
