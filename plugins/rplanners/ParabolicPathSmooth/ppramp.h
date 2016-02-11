// -*- coding: utf-8 -*-
#ifndef PARABOLIC_PPRAMP_H
#define PARABOLIC_PPRAMP_H

#include "paraboliccommon.h"

namespace ParabolicRampInternal {

/// \brief ramp with two acclerations
class PPRamp
{
public:
    Real Evaluate(Real t) const;
    Real Derivative(Real t) const;
    Real Accel(Real t) const;
    bool SolveMinTime(Real amax);
    bool SolveMinTime2(Real amax,Real timeLowerBound);
    bool SolveFixedTime(Real amax, Real endTime);
    bool SolveMinAccel(Real endTime);

    /// \brief returns max speed (absolute value of velocity) along ramp
    Real GetMaxSpeed() const;

    Real CalcTotalTime(Real a) const;
    Real CalcSwitchTime(Real a) const;
    Real CalcMinAccel(Real endTime,Real sign,Real& switchTime) const;
    int CalcSwitchTimes(Real a,Real& t1,Real& t2) const;
    int CalcTotalTimes(Real a,Real& t1,Real& t2) const;

    //input
    Real x0,dx0;
    Real x1,dx1;

    //calculated
    Real _a1, _a2;
    Real tswitch,ttotal;
};

}

#endif
