// -*- coding: utf-8 -*-
#include "ppramp.h"

namespace ParabolicRampInternal {

using namespace std;

Real PPRamp::Evaluate(Real t) const
{
    if(t < tswitch) {
        return x0 + 0.5*_a1*t*t + dx0*t;
    }
    else {
        Real tmT = t - ttotal;
        return x1 - 0.5*_a2*tmT*tmT + dx1*tmT;
    }
}

Real PPRamp::Derivative(Real t) const
{
    if(t < tswitch) {
        return _a1*t + dx0;
    }
    else {
        Real tmT = t - ttotal;
        return -_a2*tmT + dx1;
    }
}

Real PPRamp::Accel(Real t) const
{
    if(t < tswitch) {
        return _a1;
    }
    else {
        return _a2;
    }
}

bool PPRamp::SolveMinTime(Real amax)
{
    Real tpn = CalcTotalTime(amax), tnp = CalcTotalTime(-amax);
    //cout<<"Time for parabola +-: "<<tpn<<", parabola -+: "<<tnp<<endl;
    if(tpn >= 0) {
        if(tnp >= 0 && tnp < tpn) {
            _a1 = -amax;
            ttotal = tnp;
        }
        else {
            _a1 = amax;
            ttotal = tpn;
        }
    }
    else if(tnp >= 0) {
        _a1 = -amax;
        ttotal = tnp;
    }
    else {
        tswitch = -1;
        ttotal = -1;
        _a1 = _a2 = 0;
        return false;
    }
    _a2 = -_a1;
    tswitch = CalcSwitchTime(_a1);
    //uncomment for additional debugging
    if(!FuzzyEquals(x0 + 0.5*_a1*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*(-_a2)*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),EpsilonX)) {
        PARABOLICWARN("Error computing parabolic-parabolic min-time...\n");
        PARABOLICWARN("x0=%.15e; dx0=%.15e; x1=%.15e; dx1=%.15e\n",x0,dx0,x1,dx1);
        PARABOLICWARN("a = %.15e, tswitch = %.15e, ttotal = %.15e\n",_a1,tswitch,ttotal);
        PARABOLICWARN("Forward %.15e, backward %.15e, diff %.15e\n",x0 + 0.5*_a1*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*(-_a2)*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal), x0 + 0.5*_a1*Sqr(tswitch) + dx0*tswitch - (x1 - 0.5*(-_a2)*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal)));
        //Real b = 2.0*dx0; //2.0*(dx0-dx1);
        //Real c = (Sqr(dx0)-Sqr(dx1))*0.5/a+x0-x1;
        Real b = 2.0*_a1*dx0; //2.0*(dx0-dx1);
        Real c = (Sqr(dx0)-Sqr(dx1))*0.5+(x0-x1)*_a1;
        Real t1,t2;
        int res=SolveQuadratic(_a1*_a1,b,c,t1,t2);
        PARABOLICWARN("Quadratic equation %.15e x^2 + %.15e x + %.15e = 0\n",_a1*_a1,b,c);
        PARABOLICWARN("%d results, %.15e %.15e\n",res,t1,t2);
        SaveRamp("PP_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,Inf,-1);
        return false;
    }
    PARABOLIC_RAMP_ASSERT(FuzzyEquals(x0 + 0.5*_a1*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*(-_a2)*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),EpsilonX));

    return true;
}

bool PPRamp::SolveMinTime2(Real amax,Real timeLowerBound)
{
    PARABOLIC_RAMP_ASSERT(timeLowerBound >= 0);
    Real t1pn,t1np,t2pn,t2np;
    int respn = CalcTotalTimes(amax,t1pn,t2pn);
    int resnp = CalcTotalTimes(-amax,t1np,t2np);
    ttotal = Inf;
    if(respn >= 1) {
        if(t1pn >= timeLowerBound && t1pn < ttotal) {
            ttotal = t1pn;
            _a1 = amax;
        }
    }
    if(respn >= 2) {
        if(t2pn >= timeLowerBound && t2pn < ttotal) {
            ttotal = t2pn;
            _a1 = amax;
        }
    }
    if(resnp >= 1) {
        if(t1np >= timeLowerBound && t1np < ttotal) {
            ttotal = t1np;
            _a1 = -amax;
        }
    }
    if(resnp >= 2) {
        if(t2np >= timeLowerBound && t2np < ttotal) {
            ttotal = t2np;
            _a1 = -amax;
        }
    }
    if(IsInf(ttotal)) {
        _a1 = _a2 = 0;
        tswitch = ttotal = -1;
        return false;
    }
    _a2 = -_a1;
    PARABOLIC_RAMP_ASSERT(ttotal >= timeLowerBound);
    Real ts1,ts2;
    int res = CalcSwitchTimes(_a1,ts1,ts2);
    PARABOLIC_RAMP_ASSERT(res > 0);
    if(res == 1) {
        tswitch = ts1;
        PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal,ts1*2.0 - (dx1-dx0)/_a1,EpsilonT));
    }
    else {
        if(FuzzyEquals(ttotal,ts1*2.0 - (dx1-dx0)/_a1,EpsilonT)) {
            tswitch = ts1;
        }
        else {
            PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal,ts2*2.0 - (dx1-dx0)/_a1,EpsilonT));
            tswitch = ts2;
        }
    }

    //uncomment for additional debugging
    Real eps = EpsilonX;
    if(!FuzzyEquals(x0 + 0.5*_a1*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*_a1*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),eps)) {
        PARABOLICWARN("Error computing parabola min-time...\n");
        PARABOLICWARN("x0=%.15e,%.15e, x1=%.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLICWARN("a = %.15e, tswitch = %.15e, ttotal = %.15e\n",_a1,tswitch,ttotal);
        PARABOLICWARN("Forward %.15e, backward %.15e, diff %.15e\n",x0 + 0.5*_a1*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*_a1*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal), x0 + 0.5*_a1*Sqr(tswitch) + dx0*tswitch - (x1 - 0.5*_a1*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal)));
        //Real b = 2.0*dx0; //2.0*(dx0-dx1);
        //Real c = (Sqr(dx0)-Sqr(dx1))*0.5/a+x0-x1;
        Real b = 2.0*_a1*dx0; //2.0*(dx0-dx1);
        Real c = (Sqr(dx0)-Sqr(dx1))*0.5+(x0-x1)*_a1;
        Real t1,t2;
        int res=SolveQuadratic(_a1*_a1,b,c,t1,t2);
        PARABOLICWARN("Quadratic equation %.15e x^2 + %.15e x + %.15e = 0\n",_a1*_a1,b,c);
        PARABOLICWARN("%d results, %.15e %.15e\n",res,t1,t2);
        SaveRamp("PP_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,Inf,timeLowerBound);
        return false;
    }
    PARABOLIC_RAMP_ASSERT(FuzzyEquals(x0 + 0.5*_a1*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*_a1*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),eps));

    return true;
}

/** known is the final time endTime=t3

   t1,t3,a1,a2,v0,v1,dx = symbols('t1,t3,a1,a2,v0,v1,dx')
   v0 + a1*t1 + a2*(t3-t1) - v1 = 0
   a2 = -a1
   => 2*a1*t1 - a1*t3 + v0 - v1 = 0
   => a1 = -(v0 - v1)/(2*t1 - t3)
   => t1 = (a1*t3 - v0 + v1)/(2*a1)
   a2 = -a1

   eq = -dx + v0*t1 + 0.5*a1*t1**2 + (v0+a1*t1)*(t3-t1) + 0.5*a2*(t3-t1)**2

   assuming a1 != 0
   eq2 = eq.subs(a2,-a1).subs(t1,(a1*t3 - v0 + v1)/(2*a1))
   eq3 = (eq2*a1).expand()
   => 0.25*a1**2*t3**2 - a1*dx + a1*t3*v0/2 + 0.5*a1*t3*v1 - 0.25*v0**2 + v0*v1/2 - 0.25*v1**2

   => c2*a1**2 + c1*a1 + c0
   c2 = 0.25*t3**2
   c1 = -1.0*dx + 0.5*t3*v0 + 0.5*t3*v1
   c0 = - 0.25*v0**2 + 0.5*v0*v1 - 0.25*v1**2

   \author Rosen Diankov
 */
bool PPRamp::SolveFixedTime(Real amax,Real endTime)
{
    if( endTime <= 0 ) {
        return false;
    }
    Real c2 = 0.25 * endTime * endTime;
    Real c1 = x0 - x1 + 0.5*endTime*(dx0+dx1);
    Real c0 = -0.25*(dx0-dx1)*(dx0-dx1);
    Real aroots[2];
    int numroots = SolveQuadratic(c2, c1, c0, aroots[0], aroots[1]);
    if( numroots ==2 && Abs(aroots[0]) > Abs(aroots[1]) ) {
        swap(aroots[0],aroots[1]);
    }

    for(int i = 0; i < numroots; ++i) {
        if( Abs(aroots[i]) <= amax+EpsilonA ) {
            _a1 = aroots[i];
            if( aroots[i] > amax ) {
                _a1 = amax;
            }
            else if( aroots[i] < -amax ) {
                _a1 = -amax;
            }
            else {
                _a1 = aroots[i];
            }
            _a2 = -_a1;
            if( FuzzyZero(_a1,EpsilonA) ) {
                tswitch = 0;
            }
            else {
                tswitch = 0.5*(endTime + (dx1 - dx0)/_a1);
            }

            // check if tswitch is ok
            if( tswitch < 0 ) {
                if( tswitch >= -EpsilonT ) {
                    tswitch = 0;
                }
                else {
                    // bad root;
                    continue;
                }
            }
            else if( tswitch > endTime ) {
                if( tswitch <= endTime+EpsilonT ) {
                    tswitch = endTime;
                }
                else {
                    // bad root;
                    continue;
                }
            }

            // a=0 might pop up since we multiplied the equation by a
            Real expecteddx = _a1*(-tswitch*tswitch + endTime*(2.0*tswitch - 0.5*endTime)) + endTime*dx0;
            if( FuzzyEquals(expecteddx,x1-x0,EpsilonX) ) {
                ttotal = endTime;
                return true;
            }
        }
    }
    return false;
}

bool PPRamp::SolveMinAccel(Real endTime)
{
    Real switch1,switch2;
    Real apn = CalcMinAccel(endTime,1.0,switch1);
    Real anp = CalcMinAccel(endTime,-1.0,switch2);
    //cout<<"Accel for parabola +-: "<<apn<<", parabola -+: "<<anp<<endl;
    if(apn >= 0) {
        if(anp >= 0 && anp < apn) {
            _a1 = -anp;
        }
        else {
            _a1 = apn;
        }
    }
    else if(anp >= 0) {
        _a1 = -anp;
    }
    else {
        _a1=_a2=0;
        tswitch = -1;
        ttotal = -1;
        return false;
    }
    _a2=-_a1;
    ttotal = endTime;
    if(_a1 == apn) {
        tswitch = switch1;
    }
    else {
        tswitch = switch2;
    }
    //debug
    Real t2mT = tswitch-ttotal;
    if(!FuzzyEquals(x0 + tswitch*dx0 + 0.5*_a1*Sqr(tswitch),x1+t2mT*dx1-0.5*_a1*Sqr(t2mT),EpsilonX)) {
        PARABOLICWARN("PPRamp: Error solving min-accel!\n");
        PARABOLICWARN("Forward ramp: %.15e, backward %.15e, diff %.15e\n",x0 + tswitch*dx0 + 0.5*_a1*Sqr(tswitch),x1+t2mT*dx1-0.5*_a1*Sqr(t2mT),x0 + tswitch*dx0 + 0.5*_a1*Sqr(tswitch)-(x1+t2mT*dx1-0.5*_a1*Sqr(t2mT)));
        PARABOLICWARN("A+ = %.15e, A- = %.15e\n",apn,anp);
        PARABOLICWARN("ramp x0=%.15e; dx0=%.15e; x1=%.15e; dx1=%.15e\n",x0,dx0,x1,dx1);
        PARABOLICWARN("switch1=%.15e, switch2=%.15e, total=%.15e\n",switch1,switch2,ttotal);

        {
            Real sign = 1.0;
            Real a=Sqr(endTime);
            Real b=sign*(2.0*(dx0+dx1)*endTime+4.0*(x0-x1));
            Real c=-Sqr(dx1-dx0);
            PARABOLICWARN("Quadratic %.15e x^2 + %.15e x + %.15e = 0\n",a,b,c);
            Real t1,t2;
            int res = SolveQuadratic(a,b,c,t1,t2);
            PARABOLICWARN("Solutions: %d, %.15e and %.15e\n",res,t1,t2);
        }
        {
            Real sign = -1.0;
            Real a=Sqr(endTime);
            Real b=sign*(2.0*(dx0+dx1)*endTime+4.0*(x0-x1));
            Real c=-Sqr(dx1-dx0);
            PARABOLICWARN("Quadratic %.15e x^2 + %.15e x + %.15e = 0\n",a,b,c);
            Real t1,t2;
            int res = SolveQuadratic(a,b,c,t1,t2);
            PARABOLICWARN("Solutions: %d, %.15e and %.15e\n",res,t1,t2);
        }
        SaveRamp("PP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,Inf,endTime);
    }
    if(!FuzzyEquals(dx0 + _a1*tswitch,dx1-_a1*t2mT,EpsilonV)) {
        PARABOLICWARN("PPRamp: Error solving min-accel!\n");
        PARABOLICWARN("Velocity error %.15e vs %.15e, err %.15e\n",dx0+_a1*tswitch,dx1-_a1*t2mT,dx0+_a1*tswitch-(dx1-_a1*t2mT));
        PARABOLICWARN("ramp %.15e,%.15e -> %.15e, %.15e\n",x0,dx0,x1,dx1);
        PARABOLICWARN("Accel %.15e\n",_a1);
        PARABOLICWARN("Switch %.15e, total %.15e\n",tswitch,ttotal);
        SaveRamp("PP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,Inf,endTime);
        return false;
    }
    return true;
}

Real PPRamp::GetMaxSpeed() const
{
    Real f0 = fabs(dx0), f1 = fabs(dx0 + tswitch*_a1), f2 = fabs(dx1);
    if( f0 > f1 ) {
        return f0 > f2 ? f0 : f2;
    }
    else {
        return f1 > f2 ? f1 : f2;
    }
}

Real PPRamp::CalcTotalTime(Real a) const
{
    Real tswitch = CalcSwitchTime(a);
    //PARABOLIC_RAMP_PLOG("a = %.15e, switch time %.15e\n",a,tswitch);
    if(tswitch < 0) {
        return -1;
    }
    if(tswitch < (dx1-dx0)/a) {
        return -1;
    }
    return tswitch*2.0 - (dx1-dx0)/a;
}

int PPRamp::CalcTotalTimes(Real a,Real& t1,Real& t2) const
{
    Real ts1,ts2;
    int res=CalcSwitchTimes(a,ts1,ts2);
    if(res == 0) return res;
    else if(res == 1) {
        if(ts1 < (dx1-dx0)/a) return 0;
        t1 = ts1*2.0 - (dx1-dx0)/a;
        return 1;
    }
    else {
        //check limits
        if(ts1 < (dx1-dx0)/a) t1=-1;
        else t1 = ts1*2.0 - (dx1-dx0)/a;
        if(ts2 < (dx1-dx0)/a) t2=-1;
        else t2 = ts2*2.0 - (dx1-dx0)/a;
        if(t1 < 0) {
            if(t2 < 0) return 0;
            t1 = t2;
            return 1;
        }
        else {
            if(t2 < 0) return 1;
            else return 2;
        }
    }
}

int PPRamp::CalcSwitchTimes(Real a,Real& t1,Real& t2) const
{
    int res;
    if(Abs(a) > 1.0) {
        //this may be prone to numerical errors
        Real b = 2.0*dx0; //2.0*(dx0-dx1);
        Real c = (Sqr(dx0)-Sqr(dx1))*0.5/a+x0-x1;
        res=SolveQuadratic(a,b,c,t1,t2);
    }
    else {
        Real b = 2.0*a*dx0; //2.0*(dx0-dx1);
        Real c = (Sqr(dx0)-Sqr(dx1))*0.5+(x0-x1)*a;
        res=SolveQuadratic(a*a,b,c,t1,t2);
    }
    if(res == 0) {
        return res;
    }
    else if(res == 2) {
        if(t1 < 0 && t1 > -EpsilonT*0.1) {
            t1=0;
        }
        if(t2 < 0 && t2 > -EpsilonT*0.1) {
            t2=0;
        }
        if(t1 < 0 || t1*Abs(a) < (dx1-dx0)*Sign(a)) {
            if(t2 < 0 || t2*Abs(a) < (dx1-dx0)*Sign(a)) {
                return 0;
            }
            t1 = t2;
            return 1;
        }
        else if(t2 < 0 || t2*Abs(a) < (dx1-dx0)*Sign(a)) {
            return 1;
        }
        else {
            return 2; //both are ok
        }
    }
    else {
        if(t1 < 0 && t1 > -EpsilonT) {
            t1=0;
        }
        if(t1 < 0) {
            return 0;
        }
        return 1;
    }
}

Real PPRamp::CalcSwitchTime(Real a) const
{
    Real t1,t2;
    int res = CalcSwitchTimes(a,t1,t2);
    if(res == 0) {
        return -1;
    }
    else if(res == 2) {
        //check total time
        if(t2*Abs(a) < (dx1-dx0)*Sign(a)) {
            return t1;
        }
        else if(t1*Abs(a) < (dx1-dx0)*Sign(a)) {
            return t2;
        }
        else {
            //both are ok
            return Min(t1,t2);
        }
    }
    else {
        return t1;
    }
}

//for a PP ramp:
//xs = x0 + ts*dx0 + 0.5*z*ts^2
//xs = x1 - (T-ts)*dx1 - 0.5*z*(T-ts)^2
//xs' = dx0 + ts*z
//xs' = dx1 + (T-ts)*z
//z = sign*a
//(2ts-T)*z = dx1 - dx0
//ts = 1/2* (T+(dx1 - dx0)/z)
//T-ts = 1/2* (T-(dx1 - dx0)/z)
//2 T(dx0+dx1) + 4(x0 - x1) - (dx1 - dx0)^2/z + z*T^2 = 0
//what if z is close to 0?
//suppose dx1 ~= dx0, then the other solution is
//4 T dx0 + 4(x0 - x1) + z*T^2 = 0
//=>z = - 4 dx0 / T + 4(x1 - x0)/T^2
//
//alt: let y = (dx1 - dx0)/z, z = (dx1 - dx0)/y  (y is a time shift)
//ts = 1/2* (T+y)
//T-ts = 1/2* (T-y)
//x0 + 1/2(T+y)*dx0 + 0.5*z*1/4(T+y)^2 = x1 - 1/2(T-y)*dx1 - 0.5*z*1/4(T-y)^2
//4(x0-x1) + 2(T+y)*dx0 + 0.5*z*(T+y)^2 = - 2(T-y)*dx1 - 0.5*z*(T-y)^2
//[4(x0-x1)/T + 2(dx0+dx1)] y - y^2 (dx1 - dx0)/T + (dx1 - dx0) T = 0
Real PPRamp::CalcMinAccel(Real endTime,Real sign,Real& switchTime) const
{
    Real a = -(dx1 - dx0)/endTime;
    Real b = (2.0*(dx0+dx1)+4.0*(x0-x1)/endTime);
    Real c = (dx1 - dx0)*endTime;
    Real rat1,rat2;
    int res=SolveQuadratic(a,b,c,rat1,rat2);
    Real accel1 = (dx1-dx0)/rat1;
    Real accel2 = (dx1-dx0)/rat2;
    Real switchTime1 = endTime*0.5+0.5*rat1;
    Real switchTime2 = endTime*0.5+0.5*rat2;
    //fix up numerical errors
    if(switchTime1 >  endTime && switchTime1 < endTime+EpsilonT*1e-1) {
        switchTime1 = endTime;
    }
    if(switchTime2 >  endTime && switchTime2 < endTime+EpsilonT*1e-1) {
        switchTime2 = endTime;
    }
    if(switchTime1 < 0 && switchTime1 > EpsilonT*1e-1) {
        switchTime1 = 0;
    }
    if(switchTime2 < 0 && switchTime2 > EpsilonT*1e-1) {
        switchTime2 = 0;
    }
    if(res > 0 && FuzzyZero(rat1,EpsilonT)) {
        //consider it as a zero, ts = T/2
        //z = - 4*(x0-x1)/T^2 - 2 (dx0+dx1)/T
        if(!FuzzyZero(endTime,EpsilonT)) { //no good answer if endtime is small
            accel1=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
        }
    }
    if(res > 1 && FuzzyZero(rat2,EpsilonT)) {
        if(!FuzzyZero(endTime,EpsilonT)) { //no good answer if endtime is small
            accel2=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
        }
    }
    bool firstInfeas = false;
    if(res > 0 && (FuzzyZero(accel1,EpsilonA) || FuzzyZero(endTime/rat1,EpsilonA))) { //infer that accel must be small
        if(!FuzzyZero(dx0-dx1,EpsilonT)) { //no good answer if dx0=dx1
            switchTime1 = endTime*0.5;
        }
        if(!FuzzyEquals(x0 + switchTime1*dx0 + 0.5*Sqr(switchTime1)*accel1,x1 - (endTime-switchTime1)*dx1 - 0.5*Sqr(endTime-switchTime1)*accel1,EpsilonX) ||
           !FuzzyEquals(dx0+switchTime1*accel1,dx1+(endTime-switchTime1)*accel1,EpsilonV)) {
            firstInfeas = true;
        }
    }
    if(res > 1 && (FuzzyZero(accel2,EpsilonA) || FuzzyZero(endTime/rat2,EpsilonA))) {
        if(!FuzzyZero(dx0-dx1,EpsilonT)) { //no good answer if dx0=dx1
            switchTime2 = endTime*0.5;
        }
        if(!FuzzyEquals(x0 + switchTime2*dx0 + 0.5*Sqr(switchTime2)*accel2,x1 - (endTime-switchTime2)*dx1 - 0.5*Sqr(endTime-switchTime2)*accel2,EpsilonX) ||
           !FuzzyEquals(dx0+switchTime2*accel2,dx1+(endTime-switchTime2)*accel2,EpsilonV)) {
            res--;
        }
    }
    if(firstInfeas) {
        accel1 = accel2;
        rat1 = rat2;
        switchTime1 = switchTime2;
        res--;
    }
    if(res==0) {
        return -1;
    }
    else if(res==1) {
        if(switchTime1 >= 0 && switchTime1 <= endTime) {
            switchTime=switchTime1;
            return sign*accel1;
        }
        return -1.0;
    }
    else if(res==2) {
        if(switchTime1 >= 0 && switchTime1 <= endTime) {
            if(switchTime2 >= 0 && switchTime2 <= endTime) {
                if(accel1 < accel2) {
                    switchTime=switchTime1;
                    return sign*accel1;
                }
                else {
                    switchTime=switchTime2;
                    return sign*accel2;
                }
            }
            else {
                switchTime=switchTime1;
                return sign*accel1;
            }
        }
        else if(switchTime2 >= 0 && switchTime2 <= endTime) {
            switchTime=switchTime2; return sign*accel2;
        }
        return -1.0;
    }
    if(FuzzyZero(a,EpsilonT) && FuzzyZero(b,EpsilonT) && FuzzyZero(c,EpsilonT)) {
        switchTime = 0.5*endTime;
        return 0;
    }
    return -1.0;

    /*
       Real a=endTime;
       Real b=sign*(2.0*(dx0+dx1)+4.0*(x0-x1)/endTime);
       if(FuzzyZero(b,EpsilonX)) {
       //need to double check for numerical instability
       //if sign is +, this means we're switching directly to -
       //if sign is -, this means we're switching directly to +
       //if(sign > 0.0 && x1 > x0+dx0*endTime) return -1;
       //else if(sign < 0.0 && x1 < x0+dx0*endTime) return -1;
       switchTime = 0;
       Real a=(dx1-dx0)/endTime;
       if((sign > 0.0) == (a >= 0.0)) return -1;
       else return Abs(a);
       }
       Real c=-Sqr(dx1-dx0)/endTime;
       if(FuzzyEquals(dx1,dx0,EpsilonV)) {
       //one of the solutions will be very close to zero, use alt solution
       Real a=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
       PARABOLIC_RAMP_PLOG("only two solutions: 0 and %.15e\n",a);
       switchTime = 0.5*endTime;
       //try out the zero solution
       PARABOLIC_RAMP_PLOG("diff at 0 solution: %.15e\n",x0-x1 + switchTime*(dx0+dx1));
       if(FuzzyEquals(x0 + switchTime*dx0,x1 - switchTime*dx1,EpsilonX))
        return 0;
       PARABOLIC_RAMP_ASSERT(FuzzyEquals(dx0 + switchTime*a,dx1 + switchTime*a,EpsilonV));
       PARABOLIC_RAMP_ASSERT(FuzzyEquals(x0 + switchTime*dx0 + 0.5*a*Sqr(switchTime),x1 - switchTime*dx1-0.5*a*Sqr(switchTime),EpsilonX));
       if((sign > 0.0) != (a >= 0.0)) return -1;
       return Abs(a);
       }
       if(FuzzyZero(c,EpsilonA)) {
       //need better numerical performance when dx1 ~= dx0
       a = a/Abs(dx1-dx0);
       b = b/Abs(dx1-dx0);
       c = -Abs(dx1-dx0)/endTime;
       }
       Real accel1,accel2;
       int res=SolveQuadratic(a,b,c,accel1,accel2);
       //remove negative accelerations
       if(res >= 1 && accel1 < 0) {
       accel1 = accel2;
       res--;
       }
       if(res >= 2 && accel2 < 0) {
       res--;
       }

       Real switchTime1 = endTime*0.5+sign*0.5*(dx1-dx0)/accel1;
       Real switchTime2 = endTime*0.5+sign*0.5*(dx1-dx0)/accel2;
       //if(accel1 == 0 && x0 == x1) switchTime1 = 0;
       //if(accel2 == 0 && x0 == x1) switchTime2 = 0;
       if(res==0) return -1;
       else if(res==1) {
       if(!IsFinite(accel1)) {
        PARABOLIC_RAMP_PLOG("Error computing accelerations!\n");
        PARABOLIC_RAMP_PLOG("Quadratic %.15e x^2 + %.15e x + %.15e = 0\n",a,b,c);
        PARABOLIC_RAMP_PLOG("x0 %.15e, dx0 %.15e, x1 %.15e, dx1 %.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("EndTime %.15e, sign %.15e\n",endTime,sign);
        PARABOLIC_RAMP_PLOG("Results %.15e %.15e\n",accel1,accel2);
       }
       if(switchTime1 >= 0 && switchTime1 <= endTime) { switchTime=switchTime1; return accel1; }
       return -1.0;
       }
       else if(res==2) {
       if(!IsFinite(accel1) || !IsFinite(accel2)) {
        PARABOLIC_RAMP_PLOG("Error computing accelerations!\n");
        PARABOLIC_RAMP_PLOG("Quadratic %.15e x^2 + %.15e x + %.15e = 0\n",a,b,c);
        PARABOLIC_RAMP_PLOG("x0 %.15e, dx0 %.15e, x1 %.15e, dx1 %.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("EndTime %.15e, sign %.15e\n",endTime,sign);
        PARABOLIC_RAMP_PLOG("Results %.15e %.15e\n",accel1,accel2);
       }
       if(FuzzyZero(switchTime1,EpsilonT) || FuzzyZero(switchTime2,EpsilonT)) {
        //need to double check for numerical instability
        //if sign is +, this means we're switching directly to -
        //if sign is -, this means we're switching directly to +
        if(sign > 0.0 && x1 > x0+dx0*endTime) return -1;
        else if(sign < 0 && x1 < x0+dx0*endTime) return -1;
        switchTime = 0;
        if(FuzzyZero(switchTime1,EpsilonT)) return accel1;
        else return accel2;
       }
       if(switchTime1 >= 0 && switchTime1 <= endTime) {
        if(switchTime2 >= 0 && switchTime2 <= endTime) {
       if(switchTime1 < switchTime2) { switchTime=switchTime1; return accel1; }
       else { switchTime=switchTime2; return accel2; }
        }
        else { switchTime=switchTime1; return accel1; }
       }
       else if(switchTime2 >= 0 && switchTime2 <= endTime) { switchTime=switchTime2; return accel2; }
       return -1.0;
       }
       return -1.0;
     */
}

}
