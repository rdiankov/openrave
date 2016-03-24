// -*- coding: utf-8 -*-
#include "plpramp.h"

namespace ParabolicRampInternal {

Real PLPRamp::Evaluate(Real t) const
{
    Real tmT = t - ttotal;
    if(t < tswitch1) return x0 + 0.5*a*Sqr(t) + dx0*t;
    else if(t < tswitch2) {
        Real xswitch = x0 + 0.5*a*Sqr(tswitch1) + dx0*tswitch1;
        return xswitch + (t-tswitch1)*v;
    }
    else return x1 - 0.5*a*Sqr(tmT) + dx1*tmT;
}

Real PLPRamp::Derivative(Real t) const
{
    Real tmT = t - ttotal;
    if(t < tswitch1) return a*t + dx0;
    else if(t < tswitch2) return v;
    else return -a*tmT + dx1;
}

Real PLPRamp::Accel(Real t) const
{
    if(t < tswitch1) return a;
    else if(t < tswitch2) return 0;
    else return -a;
}

Real PLPRamp::CalcTotalTime(Real a,Real v) const
{
    Real t1 = (v-dx0)/a;
    Real t2mT = (dx1-v)/a;
    Real y1 = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
    Real y2 = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
    Real t2mt1 = (y2-y1)/v;
    //Real xswitch = x0 + 0.5*a*Sqr(t1) + dx0*t1;
    if(t1 < 0 || t2mT > 0 || t2mt1 < 0) return -1;
    if(!IsFinite(t1) || !IsFinite(t2mT)) return -1;
    return t1 + t2mt1 - t2mT;
}

Real PLPRamp::CalcSwitchTime1(Real a,Real v) const
{
    Real t1 = (v-dx0)/a;
    if(t1 < 0) return -1;
    return t1;
}

Real PLPRamp::CalcSwitchTime2(Real a,Real v) const
{
    Real t1 = (v-dx0)/a;
    Real y1 = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
    Real y2 = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
    Real t2mt1 = (y2-y1)/v;
    //Real t2mt1 = 0.5*(Sqr(dx1) + Sqr(dx0))/(v*a) - v/a + (x1 - x0)/v;
    //Real xswitch = x0 + 0.5*a*Sqr(t1) + dx0*t1;
    if(t1 < 0 || t2mt1 < 0) return -1;
    return t1 + t2mt1;
}

Real PLPRamp::CalcMinAccel(Real endTime,Real v) const
{
    Real den=endTime*v - (x1-x0);
    //straight line sections have den ~= 0
    if(FuzzyZero(den,EpsilonX)) {
        if(FuzzyEquals(dx0,v,EpsilonV) && FuzzyEquals(dx1,v,EpsilonV)) return 0;
        return Inf;
    }

    //Real a = (v - (dx0+dx1) + 0.5/v*(Sqr(dx0)+Sqr(dx1)))/(endTime - (x1-x0)/v);
    //Real a = (Sqr(v) - v*(dx0+dx1) + 0.5*(Sqr(dx0)+Sqr(dx1)))/den;
    Real a = (Sqr(v-dx0)+Sqr(v-dx1))/(den*2);
    /*
       Real t1 = (v-dx0)/a;
       Real t2mT = (dx1-v)/a;
       Real y1 = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
       Real y2 = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
       //Real t2mt1 = 0.5*(Sqr(dx1) + Sqr(dx0))/(v*a) - v/a + (x1 - x0)/v;
       Real t2mt1 = (y2-y1)/v;
       Real vold = v;
       //cout<<"EndTime "<<endTime<<", v "<<v<<endl;
       //cout<<"a = "<<a<<", t1="<<t1<<", t2mt1="<<t2mt1<<", t2mT="<<t2mT<<endl;
       if(t1 < 0 || t2mT > 0 || t2mt1 < 0) return Inf;
       if(!IsFinite(t1) || !IsFinite(t2mT)) return Inf;
     */
    if(!(CalcTotalTime(a,v) >= 0)) { return Inf; }
    return a;

    /*
       if(!(CalcTotalTime(a,v) >= 0)) {
       //this is very strange -- does it happen because of a compiler
       //optimization error?
       PARABOLICWARN("PLPRamp::CalcMinAccel: some numerical error prevented computing total time\n");
       PARABOLICWARN("  Ramp %.15e,%.15e -> %.15e,%.15e\n",x0,dx0,x1,dx1);
       PARABOLICWARN("  endTime %.15e, accel %.15e, vel %.15e, switch times %.15e %.15e, total time %.15e\n",endTime,a,v,CalcSwitchTime1(a,v),CalcSwitchTime2(a,v),CalcTotalTime(a,v));
       PARABOLIC_RAMP_ASSERT(v == vold);
       PARABOLIC_RAMP_PLOG("y1=%.15e, y2=%.15e, t2mt1 = %.15e\n",y1,y2,t2mt1);
       Real y1_test = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
       Real y2_test = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
       Real t2mt1_test = (y2_test-y1_test)/v;
       //Real t2mt1_test = 0.5*(Sqr(dx1) + Sqr(dx0))/(v*a) - v/a + (x1 - x0)/v;
       PARABOLIC_RAMP_PLOG("y1=%.15e, y2=%.15e, t2mt1 = %.15e\n",y1_test,y2_test,t2mt1_test);
       PARABOLIC_RAMP_PLOG("dy1=%.15e, dy2=%.15e, dt2mt1 = %.15e\n",y1-y1_test,y2-y2_test,t2mt1-t2mt1_test);
       return Inf;
       PARABOLIC_RAMP_ASSERT(y1 == y1_test);
       PARABOLIC_RAMP_ASSERT(y2 == y2_test);
       PARABOLIC_RAMP_ASSERT(y2-y1 == y2_test-y1_test);
       PARABOLIC_RAMP_ASSERT(t2mt1 == t2mt1_test);
       }
       PARABOLIC_RAMP_ASSERT(CalcTotalTime(a,v) >= 0);
       return a;
     */
}


bool PLPRamp::SolveMinTime(Real amax,Real vmax)
{
    return SolveMinTime2(amax,vmax,0);
}

bool PLPRamp::SolveMinTime2(Real amax,Real vmax,Real timeLowerBound)
{
    Real t1 = CalcTotalTime(amax,vmax);
    Real t2 = CalcTotalTime(-amax,vmax);
    Real t3 = CalcTotalTime(amax,-vmax);
    Real t4 = CalcTotalTime(-amax,-vmax);
    //cout<<"Time for PLP ++-: "<<t1<<", -++: "<<t2<<", +--: "<<t3<<", --+: "<<t4<<endl;
    ttotal = Inf;
    if(t1 >= timeLowerBound && t1 < ttotal) {
        a = amax;
        v = vmax;
        ttotal = t1;
    }
    if(t2 >= timeLowerBound && t2 < ttotal) {
        a = -amax;
        v = vmax;
        ttotal = t2;
    }
    if(t3 >= timeLowerBound && t3 < ttotal) {
        a = amax;
        v = -vmax;
        ttotal = t3;
    }
    if(t4 >= timeLowerBound && t4 < ttotal) {
        a = -amax;
        v = -vmax;
        ttotal = t4;
    }
    if(IsInf(ttotal)) {
        a = v = 0;
        tswitch1 = tswitch2 = ttotal = -1;

        //PARABOLIC_RAMP_PLOG("Times... %.15e %.15e %.15e %.15e\n",t1,t2,t3,t4);
        //PARABOLIC_RAMP_PLOG("Trying alternate MinTime2 solution technique...\n");
        Real v1 = CalcMinTimeVariableV(timeLowerBound,amax,vmax);
        Real v2 = CalcMinTimeVariableV(timeLowerBound,-amax,vmax);
        if(v1 != 0) {
            a = amax;
            v = v1;
            tswitch1 = (v1-dx0)/a;
            tswitch2 = timeLowerBound - (v1-dx1)/a;
            ttotal = timeLowerBound;
            if( !_CorrectSwitchTimes() ) {
                return false;
            }
            //PARABOLIC_RAMP_PLOG("Candidate 1 timing %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
            //PARABOLIC_RAMP_PLOG("Test 1 x %.15e %.15e\n",x1-(ttotal-tswitch2)*v1+0.5*Sqr(ttotal-tswitch2)*a,x0+tswitch1*dx0+0.5*Sqr(tswitch1)*a+(tswitch1-tswitch2)*v1);
            //PARABOLIC_RAMP_PLOG("x1, v1 = %.15e, %.15e\n",x0+dx0*tswitch1+0.5*Sqr(tswitch1)*a,dx0+tswitch1*a);
            //PARABOLIC_RAMP_PLOG("x2, v2 = %.15e, %.15e\n",x1-dx1*(ttotal-tswitch2)+0.5*Sqr(ttotal-tswitch2)*a,dx1+(ttotal-tswitch2)*a);
            return true;
        }
        if(v2 != 0) {
            a = -amax;
            v = v2;
            tswitch1 = (v2-dx0)/a;
            tswitch2 = timeLowerBound - (v2-dx1)/a;
            ttotal = timeLowerBound;
            if( !_CorrectSwitchTimes() ) {
                return false;
            }
            //PARABOLIC_RAMP_PLOG("Candidate 2 timing %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
            //PARABOLIC_RAMP_PLOG("Test 2 x %.15e %.15e\n",x1-(ttotal-tswitch2)*v1+0.5*Sqr(ttotal-tswitch2)*a,x0+tswitch1*dx0+0.5*Sqr(tswitch1)*a+(tswitch1-tswitch2)*v1);
            //PARABOLIC_RAMP_PLOG("x1, v1 = %.15e, %.15e\n",x0+dx0*tswitch1+0.5*Sqr(tswitch1)*a,dx0+tswitch1*a);
            //PARABOLIC_RAMP_PLOG("x2, v2 = %.15e, %.15e\n",x1-dx1*(ttotal-tswitch1)+0.5*Sqr(ttotal-tswitch1)*a,dx1+(ttotal-tswitch2)*a);
            return true;
        }
        return false;
    }
    tswitch1 = CalcSwitchTime1(a,v);
    tswitch2 = CalcSwitchTime2(a,v);

    if(tswitch1 > tswitch2 && FuzzyEquals(tswitch1,tswitch2,EpsilonT)) {
        tswitch1 = tswitch2 = 0.5*(tswitch1+tswitch2);
    }
    if(tswitch2 > ttotal && FuzzyEquals(tswitch2,ttotal,EpsilonT)) {
        tswitch2 = ttotal;
    }

    if( !_CorrectSwitchTimes()) {
        return false;
    }

    Real t2mT = tswitch2-ttotal;
    Real xswitch = x0 + 0.5*a*Sqr(tswitch1) + dx0*tswitch1;
    Real xswitch2 = xswitch + (tswitch2-tswitch1)*v;
    if(!FuzzyEquals(xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT,EpsilonX)) {
        PARABOLICWARN("PLP Ramp has incorrect switch 2 position: %.15e vs %.15e\n",xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT);
        PARABOLICWARN("Ramp %.15e,%.15e -> %.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLICWARN("Acceleration %.15e, vel %.15e, deceleration %.15e\n",a,v,-a);
        PARABOLICWARN("Switch times %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
        SaveRamp("PLP_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,timeLowerBound);
        return false;
    }
    return true;
}

bool PLPRamp::_CorrectSwitchTimes()
{
    if( tswitch1 > ttotal ) {
        if( tswitch1 > ttotal+EpsilonT ) {
            PARABOLICWARN("bad tswitch1 (%.15e) > ttotal (%.15e)", tswitch1, ttotal);
            return false;
        }
        tswitch1 = ttotal;
    }
    if( tswitch2 > ttotal ) {
        if( tswitch2 > ttotal+EpsilonT ) {
            PARABOLICWARN("bad tswitch2 (%.15e) > ttotal (%.15e)", tswitch2, ttotal);
            return false;
        }
        tswitch2 = ttotal;
    }

    return true;
}

/** only ttotal=t3 is known
   try to use max accel/velocity whenever possible
   constraints are:
   0 <= t1 <= t2 <= t3
   -amax <= a <= amax
   -vmax <= v <= vmax

   t1,t2,t3,a1,a2,v0,v1,dx = symbols('t1,t2,t3,a1,a2,v0,v1,dx')
   a2 = -a1
   veq = v0 + a1*t1 + a2*(t3-t2) - v1 = 0
   => t1 = -(a1*t2 - a1*t3 + v0 - v1)/a1
   => t2 = -(a1*t1 - a1*t3 + v0 - v1)/a1

   eq = -dx + v0*t1+0.5*a1*t1**2 + (v0+a1*t1)*(t2-t1) + (v0+a1*t1)*(t3-t2) + 0.5*a2*(t3-t2)**2
   eq2 = eq.subs(a2,-a1).subs(t2,-(a1*t1 - a1*t3 + v0 - v1)/a1)
   eq3 = (eq2*a1).expand()

   c2(a1)*t1**2 + c1(a1)*t1 + c0(a1) = 0
   c2(a1) = -a1*a1
   c1(a1) = a1*(a1*t3+v1-v0)
   c0(a1) = a1*(t3*v0-dx) - 0.5*(v0-v1)*(v0-v1);

   b2(t1)*a1**2 + b1(t1)*a1 + b0(t1) = 0

   t1a = -t1**2 + t1*t3
   t1b = -dx + t1*(v1-v0) + t3*v0
   t1c = -0.5*(v0-v1)*(v0-v1)

   if velocity is saturated to vmax, then

   v0 + a1*t1 = vmax
   a1 = (vmax-v0)/t1

   eq4 = (eq3.subs(a1,(vmax-v0)/t1)*t1).expand()
   t1 = (dx*v0 - dx*vmax - t3*v0*vmax + t3*vmax**2)/(0.5*v0**2 - 1.0*v0*vmax + 0.5*v1**2 - v1*vmax + vmax**2)

   if acceleration is minimized then

   besta1 = -0.5*t1b/t1a

   0.25*t1b**2/ta1 - 0.5*t1b**2/t1a + t1c = 0
   -0.25*t1b**2 + t1c*t1a = 0

   (-t1**2 + t1*t3)*(-0.5*v0 + 0.5*v1)*(v0 - v1) - 0.25*(-dx + t1*(-v0 + v1) + t3*v0)**2 = 0

   A*t1**2 + B*t1 + C = 0
   A = 0.25*v0**2 - 0.5*v0*v1 + 0.25*v1**2 = 0.25*(v0-v1)**2
   B = -0.5*dx*v0 + 0.5*dx*v1 + 0.5*t3*v0*v1 - 0.5*t3*v1**2 = 0.5*dx*(v1-v0) - 0.5*t3*v1*(v1-v0) = 0.5*(dx - t3*v1)*(v1-v0)
   C = -0.25*dx**2 + 0.5*dx*t3*v0 - 0.25*t3**2*v0**2 = -0.25*(t3*v0-dx)**2

   \author Rosen Diankov
 */
bool PLPRamp::SolveFixedTime(Real amax, Real vmax, Real endTime)
{
    if( endTime <= 0 ) {
        return false;
    }

    Real testa[2] = {amax,-amax};
    for(int ia = 0; ia < 2; ++ia) {
        a = testa[ia];
        Real c2 = -a*a;
        Real c1 = a*(a*endTime + dx1 - dx0);
        Real c0 = a*(endTime*dx0-x1+x0) - 0.5*(dx0-dx1)*(dx0-dx1);
        Real troots[2];
        int numroots = SolveQuadratic(c2, c1, c0, troots[0], troots[1]);
        for(int i = 0; i < numroots; ++i) {
            if( SolveFixedAccelSwitch1Time(amax, vmax, a, troots[i], endTime) ) {
                return true;
            }
        }
    }

    // amax didn't work, so see if there's a solution when velocity is saturated
    Real testv[2] = {vmax,-vmax};
    for(int iv = 0; iv < 2; ++iv) {
        Real denom = 0.5*((dx0 - testv[iv])*(dx0 - testv[iv]) + (dx1-testv[iv])*(dx1-testv[iv]));
        if( FuzzyZero(denom,EpsilonV*EpsilonV) ) {
            continue;
        }
        Real num = (x1-x0-endTime*testv[iv])*(dx0 - testv[iv]);
        tswitch1 = num/denom;
        a = (testv[iv]-dx0)/tswitch1;
        if( SolveFixedAccelSwitch1Time(amax, vmax, a, tswitch1, endTime) ) {
            return true;
        }
    }

    // cannot solve for a1, t1 so guessing some values
    Real testt[2] = {0.2*endTime, 0.4*endTime};
    for(int it = 0; it < 2; ++it) {
        Real tswitch1 = testt[it];
        Real b2 = -tswitch1*tswitch1;
        Real b1 = -x0 + x1 + tswitch1*(dx1-dx0) + endTime*dx0;
        Real b0 = -0.5*(dx0-dx1)*(dx0-dx1);
        Real aroots[2];
        int numroots = SolveQuadratic(b2, b1, b0, aroots[0], aroots[1]);
        for(int i = 0; i < numroots; ++i) {
            if( SolveFixedAccelSwitch1Time(amax, vmax, aroots[i], tswitch1, endTime) ) {
                return true;
            }
        }
    }

    // there's ramps that cannot be solved no matter what
    return false;
}

/* solve with t3=endTime, t1=tswitch1, and acceleration fixed
 */
bool PLPRamp::SolveFixedAccelSwitch1Time(Real amax, Real vmax, Real a, Real tswitch1, Real endTime)
{
    this->a = a;
    this->tswitch1 = tswitch1;
    if( tswitch1 < 0 ) {
        if( tswitch1 >= -EpsilonT ) {
            tswitch1 = 0;
        }
        else {
            return false;
        }
    }
    else if( tswitch1 > endTime ) {
        if( tswitch1 <= endTime+EpsilonT ) {
            tswitch2 = endTime;
        }
        else {
            return false;
        }
    }

    if( a < -amax ) {
        if( a > -amax-EpsilonA ) {
            a = -amax;
        }
        else {
            return false;
        }
    }
    if( a > amax ) {
        if( a < amax+EpsilonA ) {
            a = amax;
        }
        else {
            return false;
        }
    }

    tswitch2 = -tswitch1 + endTime + (dx1-dx0)/a;
    if( tswitch2 < tswitch1 ) {
        if( tswitch2 >= tswitch1-EpsilonT ) {
            tswitch2 = tswitch1;
        }
        else {
            return false;
        }
    }
    else if( tswitch2 > endTime ) {
        if( tswitch2 <= endTime+EpsilonT ) {
            tswitch2 = endTime;
        }
        else {
            return false;
        }
    }

    v = dx0 + a*tswitch1;
    if( Abs(v) > vmax + EpsilonV ) {
        return false;
    }

    // double check the values
    Real expecteddx = a*(tswitch1*(-0.5*tswitch1 + endTime) + tswitch2*(-0.5*tswitch2 + endTime) - 0.5*endTime*endTime) + endTime*dx0;
    if( !FuzzyEquals(expecteddx,x1-x0,EpsilonX) ) {
        return false;
    }

    ttotal = endTime;
    if( !_CorrectSwitchTimes()) {
        return false;
    }
    return true;
}

/*
   For minimum accel, solve for t1 first:

   t1,t2,t3,a1,a2,v0,v1,dx,vx = symbols('t1,t2,t3,a1,a2,v0,v1,dx,vx')
   a2 = -a1
   veq = v0 + a1*t1 + a2*(t3-t2) - v1 = 0
   => t1 = -(a1*t2 - a1*t3 + v0 - v1)/a1
   => t2 = -(a1*t1 - a1*t3 + v0 - v1)/a1

   eq = -dx + v0*t1+0.5*a1*t1**2 + (v0+a1*t1)*(t2-t1) + (v0+a1*t1)*(t3-t2) + 0.5*a2*(t3-t2)**2
   eq2 = eq.subs(a2,-a1).subs(t2,-(a1*t1 - a1*t3 + v0 - v1)/a1)
   eq3 = (eq2*a1).expand()

   let vx = a1*t1

   eq4=eq3.subs(a1*t1,vx)
   eq4 = -a1*dx + a1*t3*v0 + a1*t3*vx - 0.5*v0**2 + 1.0*v0*v1 - 1.0*v0*vx - 0.5*v1**2 + 1.0*v1*vx - 1.0*vx**2

   solve(eq4,a1) = (0.5*(v1-v0)**2 + v0*vx - v1*vx + vx**2)/(-dx + t3*(v0 + vx)) = ( (vx - (v1-v0)*0.5)**2 + (v1-v0)*0.5 ) / (t3*(v0+vx) - dx)
   then solve for da/dvx=0, which turns out to be:

   adiffsol=1.0*t3*vx**2 + (-2.0*dx + 2.0*t3*v0)*vx - 1.0*dx*v0 + 1.0*dx*v1 + 0.5*t3*v0**2 - 0.5*t3*v1**2

   \author Rosen Diankov and Kris Hauser
 */

bool PLPRamp::SolveMinAccel(Real endTime,Real vmax)
{
    Real coeffs[3];
    coeffs[0] = endTime;
    coeffs[1] = 2.0*(endTime*dx0-(x1-x0));
    coeffs[2] = (dx1-dx0)*((x1-x0) - 0.5*endTime*(dx1+dx0));
    Real vxroots[2];
    int numroots = SolveQuadratic(coeffs[0], coeffs[1], coeffs[2], vxroots[0], vxroots[1]);
    tswitch1 = tswitch2 = ttotal = -1;
    if( numroots == 0 ) {
        return false;
    }
    for(int iroot = 0; iroot < numroots; ++iroot) {
        if( vxroots[iroot] > 0 && vxroots[iroot] < endTime ) {
            Real vx = vxroots[iroot];
            Real newa = ( Sqr(vx - (dx1-dx0)*0.5) + Sqr((dx1-dx0)*0.5) ) / (endTime*(dx0+vx) - (x1-x0));
            if( fabs(newa) < EpsilonA ) {
                // no acceleration so not a PLP ramp
                continue;
            }
            // check if max velocity is exceeded
            Real newv = dx0+vx;
            if( fabs(newv) > vmax+EpsilonV ) {
                continue;
            }
            else if( newv > vmax ) {
                newv = vmax;
            }
            else if( newv < -vmax ) {
                newv = -vmax;
            }
            Real newtswitch1 = vx/newa;
            if( newtswitch1 < 0 ) {
                // tswitch1 is negative, so impossible;
                continue;
            }
            Real newtswitch2 = endTime - newtswitch1 + (dx1 - dx0)/newa;
            if( newtswitch2 >= newtswitch1 && newtswitch2 < endTime ) {
                if( tswitch1 == -1 || fabs(newa) < a ) {
                    a = newa;
                    v = newv;
                    tswitch1 = newtswitch1;
                    tswitch2 = newtswitch2;
                }
            }
        }
    }
    if( tswitch1 == -1 ) {
        // try a different method
        Real a1 = CalcMinAccel(endTime,vmax);
        Real a2 = CalcMinAccel(endTime,-vmax);
        a = Inf;
        if(fabs(a1) < a) {
            a = a1;
            v = vmax;
        }
        if(fabs(a2) < a) {
            a = a2;
            v = -vmax;
        }
        if(IsInf(a)) {
            a = 0;
            tswitch1 = tswitch2 = ttotal = -1;
            return false;
        }
        if(fabs(a) == 0) {
            tswitch1 = 0;
            tswitch2 = endTime;
            ttotal = endTime;
        }
        else {
            ttotal = CalcTotalTime(a,v);
            tswitch1 = CalcSwitchTime1(a,v);
            tswitch2 = CalcSwitchTime2(a,v);

            if(tswitch1 > tswitch2 && FuzzyEquals(tswitch1,tswitch2,EpsilonT)) {
                tswitch1 = tswitch2 = 0.5*(tswitch1+tswitch2);
            }
            if(tswitch2 > endTime && FuzzyEquals(tswitch2,endTime,EpsilonT)) {
                tswitch2 = endTime;
            }
            if(ttotal < 0) {  //there was an error computing the total time
                PARABOLICWARN("PLPRamp::SolveMinAccel: some numerical error prevented computing total time\n");
                PARABOLICWARN("  Ramp %.15e,%.15e -> %.15e,%.15e\n",x0,dx0,x1,dx1);
                PARABOLICWARN("  endTime %.15e, accel %.15e, vel %.15e, switch times %.15e %.15e, total time %.15e\n",endTime,a,v,tswitch1,tswitch2,ttotal);
                SaveRamp("PLP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
                return false;
            }
        }
        if(ttotal > endTime + EpsilonT) {
            PARABOLICWARN("PLPRamp::SolveMinAccel: total time greater than endTime!\n");
            PARABOLICWARN("  endTime %.15e, accel %.15e, vel %.15e, switch times %.15e %.15e, total time %.15e\n",endTime,a,v,tswitch1,tswitch2,ttotal);
            SaveRamp("PLP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
            return false;
        }
        if(fabs(ttotal-endTime) >= EpsilonT) {
            PARABOLICWARN("PLPRamp::SolveMinAccel: total time and endTime are different!\n");
            PARABOLICWARN("  endTime %.15e, accel %.15e, vel %.15e, switch times %.15e %.15e, total time %.15e\n",endTime,a,v,tswitch1,tswitch2,ttotal);
            SaveRamp("PLP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
        }
        PARABOLIC_RAMP_ASSERT(fabs(ttotal-endTime) < EpsilonT);
        //fiddle with the numerical errors
        ttotal = endTime;
        if(tswitch2 > ttotal) {
            tswitch2=ttotal;
        }
        if(tswitch1 > ttotal) {
            tswitch1=ttotal;
        }
    }

    ttotal = endTime;
    if( !_CorrectSwitchTimes() ) {
        return false;
    }
    
    Real t2mT = tswitch2-ttotal;
    Real xswitch = x0 + 0.5*a*Sqr(tswitch1) + dx0*tswitch1;
    Real xswitch2 = xswitch + (tswitch2-tswitch1)*v;
    if(!FuzzyEquals(xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT,EpsilonX)) {
        PARABOLICWARN("PLP Ramp has incorrect switch 2 position: %.15e vs %.15e\n",xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT);
        PARABOLICWARN("Ramp %.15e,%.15e -> %.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLICWARN("Acceleration %.15e, vel %.15e, deceleration %.15e\n",a,v,-a);
        PARABOLICWARN("Switch times %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
        SaveRamp("PLP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
        return false;
    }

    return true;
}

Real PLPRamp::CalcMinTimeVariableV(Real endTime,Real a,Real vmax) const
{
    //Try alternate solution technique with acceleration bounded, time fixed, velocity variable
    Real b = -a*endTime - (dx1+dx0);
    Real c = a*(x1-x0) + (Sqr(dx0)+Sqr(dx1))*0.5;
    Real v1,v2;
    //PARABOLIC_RAMP_PLOG("Quadratic coeffs %.15e, %.15e, %.15e\n",1.0,b,c);
    int res=SolveQuadratic(1.0,b,c,v1,v2);
    //PARABOLIC_RAMP_PLOG("Quadratic res %d, accel %.15e, velocities %.15e %.15e\n",res,a,v1,v2);
    if(res >= 1) {
        Real ts1 = (v1-dx0)/a;
        Real ts2 = endTime - (v1-dx1)/a;
        //PARABOLIC_RAMP_PLOG("Solution 1 times %.15e %.15e %.15e\n",ts1,ts2,endTime);
        if(Abs(v1) <= vmax+EpsilonV && ts1 >= 0 && ts2 >= ts1 && ts2 <= endTime+EpsilonT)
            //it's a valid solution!
            return v1;
    }
    if(res == 2) {
        Real ts1 = (v2-dx0)/a;
        Real ts2 = endTime - (v2-dx1)/a;
        //PARABOLIC_RAMP_PLOG("Solution 2 times %.15e %.15e %.15e\n",ts1,ts2,endTime);
        if(Abs(v2) <= vmax+EpsilonV && ts1 >= 0 && ts2 >= ts1 && ts2 <= endTime+EpsilonT)
            //it's a valid solution!
            return v2;
    }
    return 0;
}

}
