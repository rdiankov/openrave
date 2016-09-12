// -*- coding: utf-8 -*-
#ifndef PARABOLIC_PLPRAMP_H
#define PARABOLIC_PLPRAMP_H

#include "paraboliccommon.h"

namespace ParabolicRampInternal {

class PLPRamp
{
public:
    Real Evaluate(Real t) const;
    Real Derivative(Real t) const;
    Real Accel(Real t) const;
    bool SolveMinTime(Real amax,Real vmax);
    bool SolveMinTime2(Real amax,Real vmax,Real timeLowerBound);
    bool SolveFixedTime(Real amax, Real vmax, Real endTime);
    bool SolveFixedAccelSwitch1Time(Real amax, Real vmax, Real a, Real tswitch1, Real endTime);
    bool SolveMinAccel(Real endTime,Real vmax);

    Real CalcTotalTime(Real a,Real v) const;
    Real CalcSwitchTime1(Real a,Real v) const;
    Real CalcSwitchTime2(Real a,Real v) const;
    Real CalcMinAccel(Real endTime,Real v) const;  //variable a
    Real CalcMinTimeVariableV(Real endTime,Real a,Real vmax) const;  //variable v

    /// \brief corrects minor epsilons in switch times
    bool _CorrectSwitchTimes();
    
    //input
    Real x0,dx0;
    Real x1,dx1;

    //calculated
    Real a,v;
    Real tswitch1,tswitch2,ttotal;
};

}

#endif
