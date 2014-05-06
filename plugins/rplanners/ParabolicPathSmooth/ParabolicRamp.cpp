/*****************************************************************************
 *
 * Copyright (c) 2010-2011, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#include "ParabolicRamp.h"
#include <iostream>
using namespace std;

namespace ParabolicRampInternal {

//a flag used during testing of failed ramps
static bool gSuppressSavingRamps = false;

//a flag used to stop SolveMinAccel from complaining during internal testing
static bool gMinAccelQuiet = false;

//a flag used to stop SolveMinTime2 from complaining during internal testing
static bool gMinTime2Quiet = false;


//solves the quadratic formula and returns the number of roots found
int SolveQuadratic(Real a, Real b, Real c, Real& x1, Real& x2)
{
    if(a == 0)
    {
        if(b == 0)
        {
            if(c == 0) {
                return -1;
            }
            return 0;
        }
        x1=-c/b;
        return 1;
    }
    if(c == 0) { //det = b^2
        x1 = 0;
        x2 = -b/a;
        return 2;
    }

    Real det = b*b-4.0*a*c;
    if(det < 0.0) {
        return 0;
    }
    if(det == 0.0) {
        x1 = -b/(2.0*a);
        return 1;
    }
    det = sqrt(det);
    if(Abs(-b - det) < Abs(a)) {
        x1 = 0.5 * (-b + det)/a;
    }
    else {
        x1 = 2.0 * c / (-b-det);
    }
    if(Abs(-b + det) < Abs(a)) {
        x2 = 0.5 * (-b-det) / a;
    }
    else {
        x2 = 2.0 * c / (-b+det);
    }
    return 2;
}


//return a value x in [xmin,xmax] such that |a*x - b| <= epsilon*max(|a||b|)
//for ill posed problems choose x=0
bool SafeEqSolve(Real a,Real b,Real epsilon,Real xmin,Real xmax,Real& x)
{
    if(a < 0) return SafeEqSolve(-a,-b,epsilon,xmin,xmax,x);
    //now we know a>=0
    Real epsScaled = epsilon*Max(a,Abs(b));

    //infinte range
    if(IsInf(xmin)==-1 && IsInf(xmax)==1) {
        if(a == 0) {
            x = 0.0;
            return Abs(b) <= epsScaled;
        }
        x = b/a;
        return true;
    }

    Real axmin=a*xmin,axmax=a*xmax;
    if(!(b + epsScaled >= axmin  && b - epsScaled <= axmax)) { //ranges don't intersect
        return false;
    }
    if(a != 0) {
        x = b/a;
        if(xmin <= x && x <= xmax) {
            return true;
        }
    }
    if (Abs(0.5*(axmin+axmax) - b) <= epsScaled) {
        //center of range is in epsilon range
        x = 0.5*(xmin+xmax);
        return true;
    }
    if(Abs(axmax - b) <= epsScaled) {
        x = xmax;
        return true;
    }
    assert(Abs(axmin - b) <= epsScaled);
    x = xmin;
    return true;
}

bool SaveRamp(const char* fn,Real x0,Real dx0,Real x1,Real dx1,
              Real a,Real v,Real t)
{
    if(gSuppressSavingRamps) return true;
    std::string fullfilename = GetDumpDirectory();
    fullfilename += '/';
    fullfilename += fn;
    PARABOLIC_RAMP_PLOG("Saving ramp to %s\n",fullfilename.c_str());
    FILE* f=fopen(fullfilename.c_str(),"ab");
    if(!f) {
        f = fopen(fullfilename.c_str(),"wb");
        if(!f) {
            PARABOLIC_RAMP_PLOG("Unable to open file %s for saving\n",fullfilename.c_str());
            return false;
        }
    }
    double vals[7]={x0,dx0,x1,dx1,a,v,t};
    fwrite(vals,sizeof(double),7,f);
    fclose(f);
    return true;
}


bool LoadRamp(FILE* f,Real& x0,Real& dx0,Real& x1,Real& dx1,
              Real& a,Real& v,Real& t)
{
    double vals[7];
    int size = fread(vals,sizeof(double),7,f);
    if(size != 7) return false;
    x0=vals[0];  dx0=vals[1];
    x1=vals[2];  dx1=vals[3];
    a=vals[4];  v=vals[5];
    t=vals[6];
    return true;
}


bool LoadRamp(const char* fn,Real& x0,Real& dx0,Real& x1,Real& dx1,
              Real& a,Real& v,Real& t)
{
    FILE* f=fopen(fn,"rb");
    if(!f) return false;
    bool res=LoadRamp(f,x0,dx0,x1,dx1,a,v,t);
    fclose(f);
    return res;
}

void TestRamps(const char* fn)
{
    FILE* f=fopen(fn,"rb");
    if(!f) return;

    gSuppressSavingRamps=true;
    ParabolicRamp1D ramp;
    Real a,v,t;
    int numRamps=0;
    while(LoadRamp(f,ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,a,v,t)) {
        if(t < 0) {
            PARABOLIC_RAMP_ASSERT( a >= 0 && v >= 0);
            bool res=ramp.SolveMinTime(a,v);
            PARABOLIC_RAMP_PLOG("Result %d: t=%.15e\n",(int)res,ramp.ttotal);
        }
        else if(a < 0) {
            PARABOLIC_RAMP_ASSERT( t >= 0 && v >= 0);
            bool res=ramp.SolveMinAccel(t,v);
            PARABOLIC_RAMP_PLOG("Result %d: a=%.15e\n",(int)res,ramp.a1);
        }
        else {
            bool res=ramp.SolveMinTime2(a,v,t);
            PARABOLIC_RAMP_PLOG("Result %d: t=%.15e\n",(int)res,ramp.ttotal);

            if(!res) {
                bool res=ramp.SolveMinAccel(t,v);
                PARABOLIC_RAMP_PLOG("SolveMinAccel result %d: a=%.15e\n",(int)res,ramp.a1);
            }
        }
        PARABOLIC_RAMP_PLOG("\n");
        numRamps++;
    }
    fclose(f);
    PARABOLIC_RAMP_PLOG("%d ramps loaded from file %s\n",numRamps,fn);
    gSuppressSavingRamps=false;
}

class ParabolicRamp
{
public:
    Real Evaluate(Real t) const;
    Real Derivative(Real t) const;
    Real Accel(Real t) const;
    bool Solve(Real amax);
    bool SolveFixedTime(Real endTime);
    Real MaxVelocity() const;

    //input
    Real x0,dx0;
    Real x1,dx1;

    //calculated
    Real a;
    Real ttotal;
};


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
    Real MaxVelocity() const;

    Real CalcTotalTime(Real a) const;
    Real CalcSwitchTime(Real a) const;
    Real CalcMinAccel(Real endTime,Real sign,Real& switchTime) const;
    int CalcSwitchTimes(Real a,Real& t1,Real& t2) const;
    int CalcTotalTimes(Real a,Real& t1,Real& t2) const;

    //input
    Real x0,dx0;
    Real x1,dx1;

    //calculated
    Real a;
    Real tswitch,ttotal;
};

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

    //input
    Real x0,dx0;
    Real x1,dx1;

    //calculated
    Real a,v;
    Real tswitch1,tswitch2,ttotal;
};



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
    /*
       if(Abs(numerator)*EpsilonT < Abs(denominator)) {
       ttotal = numerator/denominator;
       }
       else {
       //dx1 ~= dx0
       if(FuzzyZero(numerator,EpsilonX) && FuzzyZero(denominator,EpsilonV)) {
        a=0;
        ttotal = 0;
        return true;
       }
       else
        return false;
       }
       Real numerator = 0.5*(Sqr(dx0)-Sqr(dx1));
       Real denominator = x0-x1;
       //check potential numerical errors
       if(FuzzyZero(denominator,EpsilonX)) {
       if(FuzzyEquals(dx0,dx1,EpsilonV)) {
        a=0;
        ttotal = 0;
        return true;
       }
       else if(FuzzyEquals(dx0,-dx1,EpsilonV)) { //pure parabola, any acceleration works
        a=amax*Sign(dx1);
        ttotal = (dx1-dx0)/a;
        return true;
       }
       if(Abs(denominator) <= Abs(numerator)) {
        //no parabola will work
        return false;
       }
       }
       a = numerator/denominator;

       //pick the denominator less likely to result in numerical errors
       if(Abs(a) < Abs(dx0+dx1)) {
       if(Abs(dx0+dx1) < EpsilonV) {
        //danger of numerical errors
        //dx0 = - dx1
        //need x0 = x1
        return false;
       }
       else {
        ttotal = 2.0*(x1-x0)/(dx0+dx1);
       }
       }
       else {
       ttotal = (dx1-dx0)/a;
       }

       if(ttotal < 0 && ttotal > -EpsilonT) ttotal = 0;
       if(ttotal < 0) {
       ttotal = -1;
       a=0;
       return false;
       }

       //check for numerical errors
       if(Abs(a) > amax && Abs(a) <= amax+EpsilonA) {
       //double check if the capped version works
       a = Sign(a)*amax;
       }
     */
    PARABOLIC_RAMP_ASSERT(ttotal >= 0);

    //pick some value of a that satisfies |ta - v1-v0| < epsilonV
    res = SafeEqSolve(ttotal,dx1-dx0,EpsilonV,-amax,amax,a);
    if(!res) return false;
    PARABOLIC_RAMP_ASSERT(Abs(a) <= amax+EpsilonA);
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
    /*
       if(FuzzyEquals(x0,x1,EpsilonX)) {
       if(FuzzyEquals(dx0,dx1,EpsilonV)) {
        a = 0;
        return FuzzyEquals(endTime,0.0,EpsilonT);
       }
       else if(FuzzyEquals(dx0,-dx1,EpsilonV)) { //pure parabola, any acceleration works
        a=(dx1-dx0)/endTime;
        return true;
       }
       //no parabola will work
       return false;
       }
       a = 0.5*(Sqr(dx0)-Sqr(dx1))/(x0-x1);
       //pick the denominator less likely to result in numerical errors
       if(Abs(a) < Abs(dx0+dx1)) {
       if(Abs(dx0+dx1) < EpsilonV) {
        //danger of numerical errors
        //dx0 = - dx1
        //need x0 = x1
        return false;
       }
       else {
        ttotal = 2.0*(x1-x0)/(dx0+dx1);
       }
       }
       else {
       ttotal = (dx1-dx0)/a;
       }
       if(!FuzzyEquals(ttotal,endTime,EpsilonT)) return false;
       ttotal = endTime;
     */
    if(FuzzyEquals(Evaluate(ttotal),x1,EpsilonX) && FuzzyEquals(Derivative(ttotal),dx1,EpsilonV)) {
        return true;
    }
    return false;
}

Real ParabolicRamp::MaxVelocity() const
{
    if(fabs(dx0) > fabs(dx1)) return dx0;
    else return dx1;
}

Real PPRamp::Evaluate(Real t) const
{
    if(t < tswitch) return x0 + 0.5*a*t*t + dx0*t;
    else {
        Real tmT = t - ttotal;
        return x1 - 0.5*a*tmT*tmT + dx1*tmT;
    }
}

Real PPRamp::Derivative(Real t) const
{
    if(t < tswitch) return a*t + dx0;
    else {
        Real tmT = t - ttotal;
        return -a*tmT + dx1;
    }
}

Real PPRamp::Accel(Real t) const
{
    if(t < tswitch) return a;
    else return -a;
}

bool PPRamp::SolveMinTime(Real amax)
{
    Real tpn = CalcTotalTime(amax), tnp = CalcTotalTime(-amax);
    //cout<<"Time for parabola +-: "<<tpn<<", parabola -+: "<<tnp<<endl;
    if(tpn >= 0) {
        if(tnp >= 0 && tnp < tpn) {
            a = -amax;
            ttotal = tnp;
        }
        else {
            a = amax;
            ttotal = tpn;
        }
    }
    else if(tnp >= 0) {
        a = -amax;
        ttotal = tnp;
    }
    else {
        tswitch = -1;
        ttotal = -1;
        a = 0;
        return false;
    }
    tswitch = CalcSwitchTime(a);
    //uncomment for additional debugging
    if(!FuzzyEquals(x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),EpsilonX)) {
        PARABOLIC_RAMP_PLOG("Error computing parabolic-parabolic min-time...\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e,%.15e, x1=%.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("a = %.15e, tswitch = %.15e, ttotal = %.15e\n",a,tswitch,ttotal);
        PARABOLIC_RAMP_PLOG("Forward %.15e, backward %.15e, diff %.15e\n",x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal), x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch - (x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal)));
        //Real b = 2.0*dx0; //2.0*(dx0-dx1);
        //Real c = (Sqr(dx0)-Sqr(dx1))*0.5/a+x0-x1;
        Real b = 2.0*a*dx0; //2.0*(dx0-dx1);
        Real c = (Sqr(dx0)-Sqr(dx1))*0.5+(x0-x1)*a;
        Real t1,t2;
        int res=SolveQuadratic(a*a,b,c,t1,t2);
        PARABOLIC_RAMP_PLOG("Quadratic equation %.15e x^2 + %.15e x + %.15e = 0\n",a*a,b,c);
        PARABOLIC_RAMP_PLOG("%d results, %.15e %.15e\n",res,t1,t2);
        SaveRamp("PP_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,Inf,-1);
        return false;
    }
    PARABOLIC_RAMP_ASSERT(FuzzyEquals(x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),EpsilonX));

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
            a = amax;
        }
    }
    if(respn >= 2) {
        if(t2pn >= timeLowerBound && t2pn < ttotal) {
            ttotal = t2pn;
            a = amax;
        }
    }
    if(resnp >= 1) {
        if(t1np >= timeLowerBound && t1np < ttotal) {
            ttotal = t1np;
            a = -amax;
        }
    }
    if(resnp >= 2) {
        if(t2np >= timeLowerBound && t2np < ttotal) {
            ttotal = t2np;
            a = -amax;
        }
    }
    if(IsInf(ttotal)) {
        a = 0;
        tswitch = ttotal = -1;
        return false;
    }

    PARABOLIC_RAMP_ASSERT(ttotal >= timeLowerBound);
    Real ts1,ts2;
    int res = CalcSwitchTimes(a,ts1,ts2);
    PARABOLIC_RAMP_ASSERT(res > 0);
    if(res == 1) {
        tswitch = ts1;
        PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal,ts1*2.0 - (dx1-dx0)/a,EpsilonT));
    }
    else {
        if(FuzzyEquals(ttotal,ts1*2.0 - (dx1-dx0)/a,EpsilonT))
            tswitch = ts1;
        else {
            PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal,ts2*2.0 - (dx1-dx0)/a,EpsilonT));
            tswitch = ts2;
        }
    }

    //uncomment for additional debugging
    Real eps = EpsilonX;
    if(!FuzzyEquals(x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),eps)) {
        PARABOLIC_RAMP_PLOG("Error computing parabola min-time...\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e,%.15e, x1=%.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("a = %.15e, tswitch = %.15e, ttotal = %.15e\n",a,tswitch,ttotal);
        PARABOLIC_RAMP_PLOG("Forward %.15e, backward %.15e, diff %.15e\n",x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal), x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch - (x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal)));
        //Real b = 2.0*dx0; //2.0*(dx0-dx1);
        //Real c = (Sqr(dx0)-Sqr(dx1))*0.5/a+x0-x1;
        Real b = 2.0*a*dx0; //2.0*(dx0-dx1);
        Real c = (Sqr(dx0)-Sqr(dx1))*0.5+(x0-x1)*a;
        Real t1,t2;
        int res=SolveQuadratic(a*a,b,c,t1,t2);
        PARABOLIC_RAMP_PLOG("Quadratic equation %.15e x^2 + %.15e x + %.15e = 0\n",a*a,b,c);
        PARABOLIC_RAMP_PLOG("%d results, %.15e %.15e\n",res,t1,t2);
        SaveRamp("PP_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,Inf,timeLowerBound);
        return false;
    }
    PARABOLIC_RAMP_ASSERT(FuzzyEquals(x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),eps));

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
            a = aroots[i];
            if( aroots[i] > amax ) {
                a = amax;
            }
            else if( aroots[i] < -amax ) {
                a = -amax;
            }
            else {
                a = aroots[i];
            }

            if( FuzzyZero(a,EpsilonA) ) {
                tswitch = 0;
            }
            else {
                tswitch = 0.5*(endTime + (dx1 - dx0)/a);
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
            Real expecteddx = a*(-tswitch*tswitch + endTime*(2.0*tswitch - 0.5*endTime)) + endTime*dx0;
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
        if(anp >= 0 && anp < apn) a = -anp;
        else a = apn;
    }
    else if(anp >= 0) a = -anp;
    else {
        a=0;
        tswitch = -1;
        ttotal = -1;
        return false;
    }
    ttotal = endTime;
    if(a == apn)
        tswitch = switch1;
    else
        tswitch = switch2;

    //debug
    Real t2mT = tswitch-ttotal;
    if(!FuzzyEquals(x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch),x1+t2mT*dx1-0.5*a*Sqr(t2mT),EpsilonX)) {
        PARABOLIC_RAMP_PLOG("PPRamp: Error solving min-accel!\n");
        PARABOLIC_RAMP_PLOG("Forward ramp: %.15e, backward %.15e, diff %.15e\n",x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch),x1+t2mT*dx1-0.5*a*Sqr(t2mT),x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch)-(x1+t2mT*dx1-0.5*a*Sqr(t2mT)));
        PARABOLIC_RAMP_PLOG("A+ = %.15e, A- = %.15e\n",apn,anp);
        PARABOLIC_RAMP_PLOG("ramp %.15e,%.15e -> %.15e, %.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("Switch 1 %.15e, switch 2 %.15e, total %.15e\n",switch1,switch2,ttotal);

        {
            Real sign = 1.0;
            Real a=Sqr(endTime);
            Real b=sign*(2.0*(dx0+dx1)*endTime+4.0*(x0-x1));
            Real c=-Sqr(dx1-dx0);
            PARABOLIC_RAMP_PLOG("Quadratic %.15e x^2 + %.15e x + %.15e = 0\n",a,b,c);
            Real t1,t2;
            int res = SolveQuadratic(a,b,c,t1,t2);
            PARABOLIC_RAMP_PLOG("Solutions: %d, %.15e and %.15e\n",res,t1,t2);
        }
        {
            Real sign = -1.0;
            Real a=Sqr(endTime);
            Real b=sign*(2.0*(dx0+dx1)*endTime+4.0*(x0-x1));
            Real c=-Sqr(dx1-dx0);
            PARABOLIC_RAMP_PLOG("Quadratic %.15e x^2 + %.15e x + %.15e = 0\n",a,b,c);
            Real t1,t2;
            int res = SolveQuadratic(a,b,c,t1,t2);
            PARABOLIC_RAMP_PLOG("Solutions: %d, %.15e and %.15e\n",res,t1,t2);
        }
        SaveRamp("PP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,Inf,endTime);
    }
    if(!FuzzyEquals(dx0 + a*tswitch,dx1-a*t2mT,EpsilonV)) {
        PARABOLIC_RAMP_PLOG("PPRamp: Error solving min-accel!\n");
        PARABOLIC_RAMP_PLOG("Velocity error %.15e vs %.15e, err %.15e\n",dx0+a*tswitch,dx1-a*t2mT,dx0+a*tswitch-(dx1-a*t2mT));
        PARABOLIC_RAMP_PLOG("ramp %.15e,%.15e -> %.15e, %.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("Accel %.15e\n",a);
        PARABOLIC_RAMP_PLOG("Switch %.15e, total %.15e\n",tswitch,ttotal);
        SaveRamp("PP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,Inf,endTime);
        return false;
    }
    return true;
}

Real PPRamp::MaxVelocity() const
{
    return tswitch*a+dx0;
}

Real PPRamp::CalcTotalTime(Real a) const
{
    Real tswitch = CalcSwitchTime(a);
    //PARABOLIC_RAMP_PLOG("a = %.15e, switch time %.15e\n",a,tswitch);
    if(tswitch < 0) return -1;
    if(tswitch < (dx1-dx0)/a) return -1;
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
        if(t1 < 0 && t1 > -EpsilonT*0.1) t1=0;
        if(t2 < 0 && t2 > -EpsilonT*0.1) t2=0;
        if(t1 < 0 || t1*Abs(a) < (dx1-dx0)*Sign(a)) {
            if(t2 < 0 || t2*Abs(a) < (dx1-dx0)*Sign(a)) return 0;
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
        if(t1 < 0 && t1 > -EpsilonT) t1=0;
        if(t1 < 0) return 0;
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
        if(t2*Abs(a) < (dx1-dx0)*Sign(a)) return t1;
        else if(t1*Abs(a) < (dx1-dx0)*Sign(a)) return t2;
        else
            //both are ok
            return Min(t1,t2);
    }
    else return t1;
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
    if(switchTime1 >  endTime && switchTime1 < endTime+EpsilonT*1e-1)
        switchTime1 = endTime;
    if(switchTime2 >  endTime && switchTime2 < endTime+EpsilonT*1e-1)
        switchTime2 = endTime;
    if(switchTime1 < 0 && switchTime1 > EpsilonT*1e-1)
        switchTime1 = 0;
    if(switchTime2 < 0 && switchTime2 > EpsilonT*1e-1)
        switchTime2 = 0;
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
    if(res==0) return -1;
    else if(res==1) {
        if(switchTime1 >= 0 && switchTime1 <= endTime) { switchTime=switchTime1; return sign*accel1; }
        return -1.0;
    }
    else if(res==2) {
        if(switchTime1 >= 0 && switchTime1 <= endTime) {
            if(switchTime2 >= 0 && switchTime2 <= endTime) {
                if(accel1 < accel2) { switchTime=switchTime1; return sign*accel1; }
                else { switchTime=switchTime2; return sign*accel2; }
            }
            else { switchTime=switchTime1; return sign*accel1; }
        }
        else if(switchTime2 >= 0 && switchTime2 <= endTime) { switchTime=switchTime2; return sign*accel2; }
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

    Real t2mT = tswitch2-ttotal;
    Real xswitch = x0 + 0.5*a*Sqr(tswitch1) + dx0*tswitch1;
    Real xswitch2 = xswitch + (tswitch2-tswitch1)*v;
    if(!FuzzyEquals(xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT,EpsilonX)) {
        PARABOLICWARN("PLP Ramp has incorrect switch 2 position: %.15e vs %.15e\n",xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT);
        PARABOLIC_RAMP_PLOG("Ramp %.15e,%.15e -> %.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("Acceleration %.15e, vel %.15e, deceleration %.15e\n",a,v,-a);
        PARABOLIC_RAMP_PLOG("Switch times %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
        SaveRamp("PLP_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,timeLowerBound);
        return false;
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
    Real t2mT = tswitch2-ttotal;
    Real xswitch = x0 + 0.5*a*Sqr(tswitch1) + dx0*tswitch1;
    Real xswitch2 = xswitch + (tswitch2-tswitch1)*v;
    if(!FuzzyEquals(xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT,EpsilonX)) {
        PARABOLICWARN("PLP Ramp has incorrect switch 2 position: %.15e vs %.15e\n",xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT);
        PARABOLIC_RAMP_PLOG("Ramp %.15e,%.15e -> %.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("Acceleration %.15e, vel %.15e, deceleration %.15e\n",a,v,-a);
        PARABOLIC_RAMP_PLOG("Switch times %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
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


// Set a ramp with desired position, velocity and time duration (added on 2013/04/24)
void ParabolicRamp1D::SetPosVelTime(Real _x0,Real _dx0,Real _x1,Real _dx1,Real t)
{
    if( t <= 0 ) {
        PARABOLICWARN("invalid time %f", t);
    }
    PARABOLIC_RAMP_ASSERT(t > 0);
    x0 = _x0;
    dx0 = _dx0;
    x1 = _x1;
    dx1 = _dx1;
    a1 = (dx1-dx0)/t;
    v = a2 = 0;
    tswitch1 = t;
    tswitch2 = t;
    ttotal = t;
}



void ParabolicRamp1D::SetConstant(Real x,Real t)
{
    x0 = x1 = x;
    dx0 = dx1 = 0;
    tswitch1=tswitch2=ttotal=t;
    v = a1 = a2 = 0;
}

void ParabolicRamp1D::SetLinear(Real _x0,Real _x1,Real t)
{
    PARABOLIC_RAMP_ASSERT(t > 0);
    x0 = _x0;
    x1 = _x1;
    v = dx0 = dx1 = (_x1-_x0)/t;
    a1 = a2 = 0;
    tswitch1 = 0;
    tswitch2 = t;
    ttotal = t;
}

Real ParabolicRamp1D::Evaluate(Real t) const
{
    Real tmT = t - ttotal;
    if(t < tswitch1) return x0 + 0.5*a1*t*t + dx0*t;
    else if(t < tswitch2) {
        Real xswitch = x0 + 0.5*a1*tswitch1*tswitch1 + dx0*tswitch1;
        return xswitch + (t-tswitch1)*v;
    }
    else return x1 + 0.5*a2*tmT*tmT + dx1*tmT;
}

Real ParabolicRamp1D::Derivative(Real t) const
{
    if(t < tswitch1) return a1*t + dx0;
    else if(t < tswitch2) return v;
    else {
        Real tmT = t - ttotal;
        return a2*tmT + dx1;
    }
}

Real ParabolicRamp1D::Accel(Real t) const
{
    if(t < tswitch1) return a1;
    else if(t < tswitch2) return 0;
    else return a2;
}

bool ParabolicRamp1D::SolveMinAccel(Real endTime,Real vmax)
{
    ParabolicRamp p;
    PPRamp pp;
    PLPRamp plp;
    p.x0 = pp.x0 = plp.x0 = x0;
    p.x1 = pp.x1 = plp.x1 = x1;
    p.dx0 = pp.dx0 = plp.dx0 = dx0;
    p.dx1 = pp.dx1 = plp.dx1 = dx1;
    bool pres = p.SolveFixedTime(endTime);
    bool ppres = pp.SolveMinAccel(endTime);
    bool plpres = false;
    if(!IsInf(vmax)) {
        plpres = plp.SolveMinAccel(endTime,vmax);
    }
    //cout<<"PP a: "<<pp.a<<", max vel "<<pp.MaxVelocity()<<endl;
    //cout<<"PLP a: "<<plp.a<<", vel "<<plp.v<<endl;
    a1 = Inf;
    if(pres && FuzzyEquals(endTime,p.ttotal,EpsilonT) && Abs(p.MaxVelocity()) <= vmax+EpsilonV) {
        if(FuzzyEquals(p.Evaluate(endTime),x1,EpsilonX) && FuzzyEquals(p.Derivative(endTime),dx1,EpsilonV)) {
            a1 = p.a;
            //tswitch1 = tswitch2 = p.ttotal;
            //ttotal = p.ttotal;
            if( Abs(a1) < EpsilonA ) {
                // not accelerating
                v = dx0;
                tswitch1 = 0;
            }
            else {
                v = 0;
                tswitch1 = endTime;
            }
            tswitch2 = endTime;
            ttotal = endTime;
        }
    }
    if(ppres && Abs(pp.MaxVelocity()) <= vmax+EpsilonV && Abs(pp.a) < Abs(a1)) {
        a1 = pp.a;
        v = 0;
        tswitch1 = tswitch2 = pp.tswitch;
        ttotal = pp.ttotal;
    }
    if(plpres && Abs(plp.v) <= vmax+EpsilonV && Abs(plp.a) < Abs(a1)) {
        a1 = plp.a;
        v = plp.v;
        tswitch1 = plp.tswitch1;
        tswitch2 = plp.tswitch2;
        ttotal = plp.ttotal;
    }
    a2 = -a1;

    if(IsInf(a1)) {
        if(vmax == 0) {
            if(FuzzyEquals(x0,x1,EpsilonX) && FuzzyEquals(dx0,dx1,EpsilonV)) {
                a1 = a2 = v = 0;
                tswitch1 = 0;
                tswitch2 = ttotal = endTime;
                return true;
            }
        }
        if(ppres && Abs(pp.MaxVelocity()) <= vmax + EpsilonV) {
            //some slight numerical error caused velocity to exceed maximum
            a1 = pp.a;
            a2 = -pp.a;
            v = 0;
            tswitch1 = tswitch2 = pp.tswitch;
            ttotal = pp.ttotal;
            if(IsValid()) return true;
        }
        a1 = a2 = v = 0;
        tswitch1 = tswitch2 = ttotal = -1;
        PARABOLIC_RAMP_PLOG("No ramp equation could solve for min-accel!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("end time %.15e, vmax = %.15e\n",endTime,vmax);

        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        PARABOLIC_RAMP_PLOG("p.a = %.15e, max vel=%.15e, end x=%.15e, end dx=%.15e\n",p.a,p.MaxVelocity(),p.Evaluate(endTime),p.Derivative(endTime));
        PARABOLIC_RAMP_PLOG("pp.a = %.15e, max vel=%.15e\n",pp.a,pp.MaxVelocity());
        PARABOLIC_RAMP_PLOG("plp.a = %.15e, v=%.15e\n",plp.a,plp.v);

        Real switch1,switch2;
        Real apn = pp.CalcMinAccel(endTime,1.0,switch1);
        Real anp = pp.CalcMinAccel(endTime,-1.0,switch2);
        PARABOLIC_RAMP_PLOG("PP Calcuations: +: %.15e %.15e, -: %.15e %.15e\n",apn,switch1,anp,switch2);
        SaveRamp("Ramp_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
        return false;
    }
    PARABOLIC_RAMP_ASSERT(ttotal==endTime);
    if(!IsValid()) {
        PARABOLIC_RAMP_PLOG("Invalid min-accel!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("end time %.15e, vmax = %.15e\n",endTime,vmax);

        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        PARABOLIC_RAMP_PLOG("p.a = %.15e, max vel=%.15e\n",p.a,p.MaxVelocity());
        PARABOLIC_RAMP_PLOG("pp.a = %.15e, max vel=%.15e\n",pp.a,pp.MaxVelocity());
        PARABOLIC_RAMP_PLOG("plp.a = %.15e, v=%.15e\n",plp.a,plp.v);
        return false;
    }
    return true;
}

bool ParabolicRamp1D::SolveMinTime(Real amax,Real vmax)
{
    ParabolicRamp p;
    PPRamp pp;
    PLPRamp plp;
    p.x0 = pp.x0 = plp.x0 = x0;
    p.x1 = pp.x1 = plp.x1 = x1;
    p.dx0 = pp.dx0 = plp.dx0 = dx0;
    p.dx1 = pp.dx1 = plp.dx1 = dx1;
    bool pres = p.Solve(amax);
    bool ppres = pp.SolveMinTime(amax);
    bool plpres = false;
    if(!IsInf(vmax))
        plpres = plp.SolveMinTime(amax,vmax);
    //cout<<"P time: "<<p.ttotal<<", accel "<<p.a<<endl;
    //cout<<"PP time: "<<pp.ttotal<<", max vel "<<pp.MaxVelocity()<<endl;
    //cout<<"PLP time: "<<plp.ttotal<<", vel "<<plp.v<<endl;
    ttotal = Inf;
    if(pres && Abs(p.a) <= amax+EpsilonA && p.ttotal < ttotal) {
        if(Abs(p.a) <= amax) {
            a1 = p.a;
            if( Abs(a1) < EpsilonA ) {
                // not accelerating
                v = dx0;
                tswitch1 = 0;
            }
            else {
                v = 0;
                tswitch1 = p.ttotal;
            }
            tswitch2 = p.ttotal;
            ttotal = p.ttotal;
        }
        else {
            //double check
            p.a = Sign(p.a)*amax;
            if(FuzzyEquals(p.Evaluate(p.ttotal),x1,EpsilonX) && FuzzyEquals(p.Derivative(p.ttotal),dx1,EpsilonV)) {
                a1 = p.a;
                v = 0;
                tswitch1=tswitch2=p.ttotal;
                ttotal = p.ttotal;
            }
        }
    }
    if(ppres && Abs(pp.MaxVelocity()) <= vmax+EpsilonV && pp.ttotal < ttotal) {
        a1 = pp.a;
        v = 0;
        tswitch1 = tswitch2 = pp.tswitch;
        ttotal = pp.ttotal;
    }
    if(plpres && plp.ttotal < ttotal) {
        a1 = plp.a;
        v = plp.v;
        tswitch1 = plp.tswitch1;
        tswitch2 = plp.tswitch2;
        ttotal = plp.ttotal;
    }
    if(IsInf(ttotal)) {
        PARABOLIC_RAMP_PLOG("No ramp equation could solve for min-time!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("vmax = %.15e, amax = %.15e\n",vmax,amax);
        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        if(pres)
            PARABOLIC_RAMP_PLOG("  P a=%.15e, ttotal=%.15e\n",p.a,p.ttotal);
        if(ppres)
            PARABOLIC_RAMP_PLOG("  PP a=%.15e, tswitch=%.15e, ttotal=%.15e\n",pp.a,pp.tswitch,pp.ttotal);
        if(plpres)
            PARABOLIC_RAMP_PLOG("  PLP a=%.15e, tswitch=%.15e, %.15e, ttotal=%.15e\n",pp.a,plp.tswitch1,plp.tswitch2,plp.ttotal);
        SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,-1);
        a1 = a2 = v = 0;
        tswitch1 = tswitch2 = ttotal = -1;
        return false;
    }
    a2 = -a1;
    if( tswitch1 < 0 && tswitch1 >= -EpsilonT ) {
        tswitch1 = 0;
    }
    if( tswitch2 < 0 && tswitch2 >= -EpsilonT ) {
        tswitch2 = 0;
    }
    //cout<<"switch time 1: "<<tswitch1<<", 2: "<<tswitch2<<", total "<<ttotal<<endl;
    if(!IsValid()) {
        PARABOLIC_RAMP_PLOG("Failure to find valid path!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("vmax = %.15e, amax = %.15e\n",vmax,amax);
        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
    }
    return true;
}

bool ParabolicRamp1D::SolveMinTime2(Real amax,Real vmax,Real tLowerBound)
{
    ParabolicRamp p;
    PPRamp pp;
    PLPRamp plp;
    p.x0 = pp.x0 = plp.x0 = x0;
    p.x1 = pp.x1 = plp.x1 = x1;
    p.dx0 = pp.dx0 = plp.dx0 = dx0;
    p.dx1 = pp.dx1 = plp.dx1 = dx1;
    bool pres = p.Solve(amax);
    bool ppres = pp.SolveMinTime2(amax,tLowerBound);
    bool plpres = false;
    if(!IsInf(vmax))
        plpres = plp.SolveMinTime2(amax,vmax,tLowerBound);
    //cout<<"P time: "<<p.ttotal<<", accel "<<p.a<<endl;
    //cout<<"PP time: "<<pp.ttotal<<", max vel "<<pp.MaxVelocity()<<endl;
    //cout<<"PLP time: "<<plp.ttotal<<", vel "<<plp.v<<endl;
    ttotal = Inf;
    if(pres && Abs(p.a) <= amax+EpsilonA && p.ttotal < ttotal && p.ttotal >= tLowerBound) {
        if(Abs(p.a) <= amax) {
            a1 = p.a;
            if( Abs(a1) < EpsilonA ) {
                // not accelerating
                v = dx0;
                tswitch1 = 0;
            }
            else {
                v = 0;
                tswitch1 = p.ttotal;
            }
            tswitch2 = p.ttotal;
            ttotal = p.ttotal;
        }
        else {
            //double check
            p.a = Sign(p.a)*amax;
            if(FuzzyEquals(p.Evaluate(p.ttotal),x1,EpsilonX) && FuzzyEquals(p.Derivative(p.ttotal),dx1,EpsilonV)) {
                a1 = p.a;
                v = 0;
                tswitch1=tswitch2=p.ttotal;
                ttotal = p.ttotal;
            }
        }
    }
    if(ppres && Abs(pp.MaxVelocity()) <= vmax+EpsilonV && pp.ttotal < ttotal) {
        a1 = pp.a;
        v = 0;
        tswitch1 = tswitch2 = pp.tswitch;
        ttotal = pp.ttotal;
    }
    if(plpres && plp.ttotal < ttotal) {
        a1 = plp.a;
        v = plp.v;
        tswitch1 = plp.tswitch1;
        tswitch2 = plp.tswitch2;
        ttotal = plp.ttotal;
    }
    if(IsInf(ttotal)) {
        PARABOLIC_RAMP_PLOG("No ramp equation could solve for min-time (2)!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("vmax = %.15e, amax = %.15e, tmax = %.15e\n",vmax,amax,tLowerBound);
        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        if(pres)
            PARABOLIC_RAMP_PLOG("  P a=%.15e, ttotal=%.15e\n",p.a,p.ttotal);
        if(ppres)
            PARABOLIC_RAMP_PLOG("  PP a=%.15e, tswitch=%.15e, ttotal=%.15e\n",pp.a,pp.tswitch,pp.ttotal);
        if(plpres)
            PARABOLIC_RAMP_PLOG("  PLP a=%.15e, tswitch=%.15e, %.15e, ttotal=%.15e\n",pp.a,plp.tswitch1,plp.tswitch2,plp.ttotal);
        ppres = pp.SolveMinTime(amax);
        plpres = plp.SolveMinTime(amax,vmax);
        PARABOLIC_RAMP_PLOG("unconstrained PP (%d): %.15e, PLP (%d): %.15e\n",(int)ppres,pp.ttotal,(int)plpres,plp.ttotal);
        SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,tLowerBound);
        a1 = a2 = v = 0;
        tswitch1 = tswitch2 = ttotal = -1;
        return false;
    }
    a2 = -a1;
    //cout<<"switch time 1: "<<tswitch1<<", 2: "<<tswitch2<<", total "<<ttotal<<endl;
    if(!IsValid()) {
        PARABOLIC_RAMP_PLOG("ParabolicRamp1D::SolveMinTime: Failure to find valid path!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("vmax = %.15e, amax = %.15e\n",vmax,amax);
        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,tLowerBound);
    }
    PARABOLIC_RAMP_ASSERT(ttotal >= tLowerBound);
    return true;
}

void ParabolicRamp1D::SolveBraking(Real amax)
{
    tswitch1 = 0;
    tswitch2 = 0;
    a1 = Sign(dx0)*amax;
    v = 0;
    a2 = -Sign(dx0)*amax;
    ttotal = Abs(dx0)/amax;
    x1 = x0 + dx0*ttotal + 0.5*Sqr(ttotal)*a2;
    dx1 = 0;
    PARABOLIC_RAMP_ASSERT(IsValid());
}

/** only ttotal is known

   \author Rosen Diankov
 */
bool ParabolicRamp1D::SolveFixedTime(Real amax,Real vmax,Real endTime)
{
    ParabolicRamp p;
    PPRamp pp;
    PLPRamp plp;
    p.x0 = pp.x0 = plp.x0 = x0;
    p.x1 = pp.x1 = plp.x1 = x1;
    p.dx0 = pp.dx0 = plp.dx0 = dx0;
    p.dx1 = pp.dx1 = plp.dx1 = dx1;
    bool pres = p.SolveFixedTime(endTime);
    bool ppres = pp.SolveFixedTime(amax,endTime);
    bool plpres = false;
    if(!IsInf(vmax))
        plpres = plp.SolveFixedTime(amax,vmax,endTime);
    //cout<<"P time: "<<p.ttotal<<", accel "<<p.a<<endl;
    //cout<<"PP time: "<<pp.ttotal<<", max vel "<<pp.MaxVelocity()<<endl;
    //cout<<"PLP time: "<<plp.ttotal<<", vel "<<plp.v<<endl;
    ttotal = Inf;
    if(pres && Abs(p.a) <= amax+EpsilonA && FuzzyEquals(p.ttotal,endTime,EpsilonT) ) {
        if(Abs(p.a) <= amax) {
            a1 = p.a;
            if( Abs(a1) < EpsilonA ) {
                // not accelerating
                v = dx0;
                tswitch1 = 0;
            }
            else {
                v = 0;
                tswitch1 = p.ttotal;
            }
            tswitch2 = p.ttotal;
            ttotal = p.ttotal;
        }
        else {
            //double check
            p.a = Sign(p.a)*amax;
            if(FuzzyEquals(p.Evaluate(p.ttotal),x1,EpsilonX) && FuzzyEquals(p.Derivative(p.ttotal),dx1,EpsilonV)) {
                a1 = p.a;
                if( Abs(a1) < EpsilonA ) {
                    // not accelerating
                    v = dx0;
                    tswitch1 = 0;
                }
                else {
                    v = 0;
                    tswitch1 = p.ttotal;
                }
                tswitch2=p.ttotal;
                ttotal = p.ttotal;
            }
        }
    }
    if(ppres && Abs(pp.MaxVelocity()) <= vmax+EpsilonV && FuzzyEquals(pp.ttotal,endTime,EpsilonT) ) {
        a1 = pp.a;
        v = 0;
        tswitch1 = tswitch2 = pp.tswitch;
        ttotal = pp.ttotal;
    }
    if(plpres && FuzzyEquals(plp.ttotal, endTime, EpsilonT) ) {
        a1 = plp.a;
        v = plp.v;
        tswitch1 = plp.tswitch1;
        tswitch2 = plp.tswitch2;
        ttotal = plp.ttotal;
    }
    if(IsInf(ttotal)) {

        // finally remove the a2=-a1 constraint
        Real dx = x1-x0;
        for(tswitch1 = 0; tswitch1 < endTime; tswitch1 += 0.2*endTime) {
            for(tswitch2 = tswitch1; tswitch2 < endTime; tswitch2 += 0.2*endTime) {
                Real c1 = -0.5*Sqr(tswitch1)*tswitch2 + 0.5*Sqr(tswitch1)*endTime + 0.5*tswitch1*Sqr(tswitch2) - 0.5*tswitch1*Sqr(endTime);
                Real c0 = - 1.0*dx*tswitch2 + 1.0*dx*endTime + 0.5*Sqr(tswitch2)*dx0 - 0.5*Sqr(tswitch2)*dx1 + 1.0*tswitch2*endTime*dx1 - 0.5*Sqr(endTime)*dx0 - 0.5*Sqr(endTime)*dx1;
                if( Abs(c1) > 1e-8 ) {
                    a1 = -c0/c1;
                    if( a1 >= -amax && a1 <= amax ) {
                        v = dx0 + a1*tswitch1;
                        if( v >= -vmax && v <= vmax ) {
                            a2 = (a1*tswitch1 + dx0 - dx1)/(tswitch2 - endTime);
                            if( a2 >= -amax && a2 <= amax ) {
                                if(IsValid()) {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }

        PARABOLIC_RAMP_PLOG("No ramp equation could solve for min-time (2)!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("vmax = %.15e, amax = %.15e, tmax = %.15e\n",vmax,amax,endTime);
        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        if(pres)
            PARABOLIC_RAMP_PLOG("  P a=%.15e, ttotal=%.15e\n",p.a,p.ttotal);
        if(ppres)
            PARABOLIC_RAMP_PLOG("  PP a=%.15e, tswitch=%.15e, ttotal=%.15e\n",pp.a,pp.tswitch,pp.ttotal);
        if(plpres)
            PARABOLIC_RAMP_PLOG("  PLP a=%.15e, tswitch=%.15e, %.15e, ttotal=%.15e\n",pp.a,plp.tswitch1,plp.tswitch2,plp.ttotal);
        ppres = pp.SolveMinTime(amax);
        plpres = plp.SolveMinTime(amax,vmax);
        PARABOLIC_RAMP_PLOG("unconstrained PP (%d): %.15e, PLP (%d): %.15e\n",(int)ppres,pp.ttotal,(int)plpres,plp.ttotal);
        SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,endTime);
        a1 = a2 = v = 0;
        tswitch1 = tswitch2 = ttotal = -1;
        return false;
    }
    a2 = -a1;
    if( tswitch1 < 0 && tswitch1 >= -EpsilonT ) {
        tswitch1 = 0;
    }
    if( tswitch2 < 0 && tswitch2 >= -EpsilonT ) {
        tswitch2 = 0;
    }
    //cout<<"switch time 1: "<<tswitch1<<", 2: "<<tswitch2<<", total "<<ttotal<<endl;
    if(!IsValid()) {
        PARABOLIC_RAMP_PLOG("ParabolicRamp1D::SolveMinTime: Failure to find valid path!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("vmax = %.15e, amax = %.15e\n",vmax,amax);
        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,endTime);
    }
    PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal, endTime, EpsilonT));
    return true;
}

/** all the switch tmes are known tswitch1, tswitch2, and ttotal

   eq=v0*t1+0.5*a1*t1**2+(v0+a1*t1)*(t2-t1)+ (v0+a1*t1)*(t3-t2) + 0.5*(v1-v0-a1*t1)/(t3-t2)*(t3-t2)**2 - dx
   solve(eq.expand(),a1)
   solve for acceleration a1

   \author Rosen Diankov
 */
bool ParabolicRamp1D::SolveFixedSwitchTime(Real amax,Real vmax)
{
    Real denom = 0.5 * tswitch1 * (-tswitch1 +ttotal + tswitch2);
    Real num = x1-x0+0.5*tswitch2*(dx1-dx0)-0.5*ttotal*(dx1+dx0);
    if( FuzzyEquals(denom,0,EpsilonT) ) {
        if( !FuzzyEquals(num,0,EpsilonT) ) {
            // num is not 0, so really cannot solve
            return false;
        }
        a1 = 0;
    }
    else {
        a1 = num/denom;
    }
    v = dx0 + a1*tswitch1;
    if( FuzzyEquals(tswitch2,ttotal,EpsilonT) ) {
        a2 = 0;
        if( !FuzzyEquals(v,dx1,EpsilonV) ) {
            return false;
        }
    }
    else {
        a2 = (dx1-v)/(ttotal-tswitch2);
    }
    if( Abs(a1) > amax+EpsilonA || Abs(a2) > amax+EpsilonA) {
        return false;
    }
    if( Abs(v) > vmax+EpsilonV ) {
        return false;
    }
    if( tswitch1 < 0 && tswitch1 >= -EpsilonT ) {
        tswitch1 = 0;
    }
    if( tswitch2 < 0 && tswitch2 >= -EpsilonT ) {
        tswitch2 = 0;
    }
    PARABOLIC_RAMP_ASSERT(IsValid());
    return true;
}

/*
   deltaswitch1=tswitch1 and deltaswitch2=ttotal-tswitch2

   dt1,dt2,dt3,a1,a2,v0,v1,dx=symbols('dt1,dt2,dt3,a1,a2,v0,v1,dx')

   v0*dt1+0.5*a1*dt1**2  +  (v0+a1*dt1)*dt2  +  (v0+a1*dt1)*dt3 + 0.5*(v1-v0-a1*dt1)/dt3*dt3**2 - dx =
   => c0*a1*dt2 + c1*a1 + c2*dt2 + c3 = 0.

   know that c0 > 0, c1 > 0

   => dt2 = -(c3 + c1*a1)/(c0*a1+c2)

   dt2 = 0 when a1 = -c3/c1

   have to minimize dt2 with respect to this equation and the constraints.
   c0*a1 + c2 = 0

   \author Rosen Diankov
 */
bool ParabolicRamp1D::SolveFixedAccelSwitchTime(Real amax,Real vmax, Real deltaswitch1, Real deltaswitch3)
{
    Real c0 = deltaswitch1;
    Real c1 = 0.5*deltaswitch1*deltaswitch1 + 0.5*deltaswitch1*deltaswitch3;
    Real c2 = dx0;
    Real c3 = deltaswitch1*dx0 + 0.5*deltaswitch3*(dx0 + dx1) - (x1-x0);

    // the switch times also bound the a1 acceleration, so take them into account
    Real amin = -amax;
    if( deltaswitch1 > 0 ) {
        // know that abs(dx1-dx0-a1*deltaswitch1) <= amax*deltaswitch3
        // this will give better bounds on a1
        Real ideltaswitch1 = 1.0/deltaswitch1;
        Real apivot = (dx1-dx0)*ideltaswitch1;
        Real awidth = amax*deltaswitch3*ideltaswitch1;
        if( amax > apivot+awidth ) {
            amax = apivot+awidth;
        }
        if( amin < apivot-awidth ) {
            amin = apivot-awidth;
        }

        Real amaxtest = (vmax+Abs(dx0))/deltaswitch1;
        if( amax > amaxtest ) {
            amax = amaxtest;
        }
        if( amin < -amaxtest ) {
            amin = -amaxtest;
        }

        if( amin > amax ) {
            return false;
        }
    }
    a1 = -c3/c1;
    v = dx0 + a1*deltaswitch1;
    Real deltaswitch2 = 0;
    if( a1 < amin-EpsilonA || a1 > amax+EpsilonA || Abs(v) > vmax+EpsilonV ) {
        // try amax
        deltaswitch2 = -(c3 + c1*amax)/(c0*amax+c2);
        Real deltaswitch2_min = -(c3 + c1*amin)/(c0*amin+c2);
        if( deltaswitch2 >= 0 ) {
            a1 = amax;
            // see if amin yields better solution
            if( deltaswitch2_min >= 0 && deltaswitch2_min < deltaswitch2 ) {
                a1 = amin;
                deltaswitch2 = deltaswitch2_min;
            }
        }
        else if( deltaswitch2_min >= 0 ) {
            a1 = amin;
            deltaswitch2 = deltaswitch2_min;
        }
        else if( deltaswitch2 > -1e-10 ) {
            deltaswitch2 = 0;
        }
        else if( deltaswitch2_min > -1e-10 ) {
            a1 = amin;
            deltaswitch2 = 0;
        }
        else {
            return false;
        }

        v = dx0 + a1*deltaswitch1;
        if( Abs(v) > vmax+EpsilonV ) {
            // velocity is still bound, so compute new a1 depending on v
            v = v > 0 ? vmax : -vmax;
            a1 = (v-dx0)/deltaswitch1;
            deltaswitch2 = -(c3 + c1*a1)/(c0*a1+c2);
            if( deltaswitch2 < 0 ) {
                if( deltaswitch2 > -1e-10 ) {
                    deltaswitch2 = 0;
                }
                else {
                    return false;
                }
            }
        }
    }
    //v = dx0 + a1*tswitch1;
    if( deltaswitch3 <= EpsilonT ) {
        a2 = 0;
        if( !FuzzyEquals(v,dx1,EpsilonV) ) {
            return false;
        }
    }
    else {
        a2 = (dx1-v)/deltaswitch3;
    }

    tswitch1 = deltaswitch1;
    tswitch2 = deltaswitch1 + deltaswitch2;
    ttotal = tswitch2 + deltaswitch3;

    PARABOLIC_RAMP_ASSERT(IsValid());
    return true;
}

void ParabolicRamp1D::Dilate(Real timeScale)
{
    tswitch1*=timeScale;
    tswitch2*=timeScale;
    ttotal*=timeScale;
    a1 *= 1.0/Sqr(timeScale);
    a2 *= 1.0/Sqr(timeScale);
    Real invTimeScale = 1.0/timeScale;
    v *= invTimeScale;
    dx0 *= invTimeScale;
    dx1 *= invTimeScale;
}

void ParabolicRamp1D::TrimFront(Real tcut)
{
    if(tcut > ttotal) {
        PARABOLIC_RAMP_PLOG("Hmm... want to trim front of curve at time %.15e, end time %.15e\n",tcut,ttotal);
    }
    PARABOLIC_RAMP_ASSERT(tcut <= ttotal);
    x0 = Evaluate(tcut);
    dx0 = Derivative(tcut);
    ttotal -= tcut;
    tswitch1 -= tcut;
    tswitch2 -= tcut;
    if(tswitch1 < 0) tswitch1=0;
    if(tswitch2 < 0) tswitch2=0;
    PARABOLIC_RAMP_ASSERT(IsValid());

}

void ParabolicRamp1D::TrimBack(Real tcut)
{
    //PARABOLIC_RAMP_ASSERT(IsValid());
    x1 = Evaluate(ttotal-tcut);
    dx1 = Derivative(ttotal-tcut);
    ttotal -= tcut;
    tswitch1 = Min(tswitch1,ttotal);
    tswitch2 = Min(tswitch2,ttotal);
    PARABOLIC_RAMP_ASSERT(IsValid());
}

void ParabolicRamp1D::Bounds(Real& xmin,Real& xmax) const
{
    Bounds(0,ttotal,xmin,xmax);
}

void ParabolicRamp1D::Bounds(Real ta,Real tb,Real& xmin,Real& xmax) const
{
    if(ta > tb) {  //orient the interval
        return Bounds(tb,ta,xmin,xmax);
    }
    if(ta < 0) ta = 0;
    if(tb <= 0) {
        xmin = xmax = x0;
        return;
    }
    if(tb > ttotal) tb=ttotal;
    if(ta >= ttotal) {
        xmin = xmax = x1;
        return;
    }

    xmin = Evaluate(ta);
    xmax = Evaluate(tb);

    if(xmin > xmax) Swap(xmin,xmax);

    Real tflip1=0,tflip2=0;
    if(ta < tswitch1) {
        //x' = a1*t + v0 = 0 => t = -v0/a1
        tflip1 = -dx0/a1;
        if(tflip1 > tswitch1) tflip1 = 0;
    }
    if(tb > tswitch2) {
        //x' = a2*(T-t) + v1 = 0 => (T-t) = v1/a2
        tflip2 = ttotal-dx1/a2;
        if(tflip2 < tswitch2) tflip2 = 0;
    }
    if(ta < tflip1 && tb > tflip1) {
        Real xflip = Evaluate(tflip1);
        if(xflip < xmin) xmin = xflip;
        else if(xflip > xmax) xmax = xflip;
    }
    if(ta < tflip2 && tb > tflip2) {
        Real xflip = Evaluate(tflip2);
        if(xflip < xmin) xmin = xflip;
        else if(xflip > xmax) xmax = xflip;
    }
}

void ParabolicRamp1D::DerivBounds(Real& vmin,Real& vmax) const
{
    DerivBounds(0,ttotal,vmin,vmax);
}

void ParabolicRamp1D::DerivBounds(Real ta,Real tb,Real& vmin,Real& vmax) const
{
    if(ta > tb) {  //orient the interval
        return DerivBounds(tb,ta,vmin,vmax);
    }
    if(ta < 0) ta = 0;
    if(tb <= 0) {
        vmin = vmax = dx0;
        return;
    }
    if(tb > ttotal) tb=ttotal;
    if(ta >= ttotal) {
        vmin = vmax = dx1;
        return;
    }

    vmin = Derivative(ta);
    vmax = Derivative(tb);
    if(vmin > vmax) Swap(vmin,vmax);

    if(tswitch2 > tswitch1) { //consider linear part
        if(ta < tswitch2 && tb > tswitch1) {
            vmin = Min(vmin,v);
            vmax = Min(vmax,v);
        }
    }
    else if(ta < tswitch1 && tb > tswitch1) { //PP ramp
        //compute bending
        Real vbend = dx0 + tswitch1*a1;
        if(vbend < vmin) vmin = vbend;
        else if(vbend > vmax) vmax = vbend;
        vbend = dx1 + (tswitch2-ttotal)*a2;
        if(vbend < vmin) vmin = vbend;
        else if(vbend > vmax) vmax = vbend;
    }
}

bool ParabolicRamp1D::IsValid() const
{
    if(tswitch1 < 0 || tswitch2 < tswitch1 || ttotal < tswitch2) {
        PARABOLICWARN("Ramp has invalid timing %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
        return false;
    }

    // increate the epsilons just a little here
    Real t2mT = tswitch2 - ttotal;
    if(tswitch1 != tswitch2) {
        if(!FuzzyEquals(a1*tswitch1 + dx0,v,2*EpsilonV)) {
            PARABOLICWARN("Ramp has incorrect switch 1 speed: %.15e vs %.15e\n",a1*tswitch1 + dx0,v);
            return false;
        }
        if(!FuzzyEquals(a2*t2mT + dx1,v,2*EpsilonV)) {
            PARABOLICWARN("Ramp has incorrect switch 2 speed: %.15e vs %.15e\n",a2*t2mT + dx1,v);
            return false;
        }
    }
    //check switch2
    Real xswitch = x0 + 0.5*a1*Sqr(tswitch1) + dx0*tswitch1;
    Real xswitch2 = xswitch + (tswitch2-tswitch1)*v;
    if(!FuzzyEquals(xswitch2,x1 + 0.5*a2*Sqr(t2mT) + dx1*t2mT,2*EpsilonX)) {
        PARABOLICWARN("Ramp has incorrect switch 2 position: %.15e vs %.15e\n",xswitch2,x1 + 0.5*a2*Sqr(t2mT) + dx1*t2mT);
        PARABOLIC_RAMP_PLOG("Ramp %.15e,%.15e -> %.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLIC_RAMP_PLOG("Acceleration %.15e, vel %.15e, deceleration %.15e\n",a1,v,a2);
        PARABOLIC_RAMP_PLOG("Switch times %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
        return false;
    }
    return true;
}


// Set a ramp with desired position, velocity and time duration (added on 2013/04/24)
void ParabolicRampND::SetPosVelTime(const Vector& _x0, const Vector& _dx0, const Vector& _x1, const Vector& _dx1,Real t)
{
    x0 = _x0;
    dx0 = _dx0;
    x1 = _x1;
    dx1 = _dx1;
    endTime = t;
    ramps.resize(x0.size());
    for(size_t i=0; i<_x0.size(); i++) {
        ramps[i].SetPosVelTime(_x0[i],_dx0[i],_x1[i],_dx1[i],t);
    }
}


void ParabolicRampND::SetConstant(const Vector& x,Real t)
{
    x0 = x1 = x;
    dx0.resize(x.size());
    dx1.resize(x.size());
    fill(dx0.begin(),dx0.end(),0);
    fill(dx1.begin(),dx1.end(),0);
    endTime = t;
    ramps.resize(x.size());
    for(size_t i=0; i<x.size(); i++)
        ramps[i].SetConstant(x[i],t);
}

void ParabolicRampND::SetLinear(const Vector& _x0,const Vector& _x1,Real t)
{
    PARABOLIC_RAMP_ASSERT(_x0.size() == _x1.size());
    PARABOLIC_RAMP_ASSERT(t > 0);
    x0 = _x0;
    x1 = _x1;
    dx0.resize(_x1.size());
    for(size_t i=0; i<_x1.size(); i++)
        dx0[i] = (_x1[i]-_x0[i])/t;
    dx1 = dx0;
    endTime = t;
    ramps.resize(_x0.size());
    for(size_t i=0; i<_x0.size(); i++)
        ramps[i].SetLinear(_x0[i],_x1[i],t);
}

bool ParabolicRampND::SolveMinTimeLinear(const Vector& amax,const Vector& vmax)
{
    //PARABOLIC_RAMP_PLOG("Size x0 %d\n",(int)x0.size());
    //PARABOLIC_RAMP_PLOG("Size amax %d\n",(int)amax.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == dx0.size());
    PARABOLIC_RAMP_ASSERT(x1.size() == dx1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == x1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == amax.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == vmax.size());
    endTime = 0;
    ramps.resize(x0.size());
    ParabolicRamp1D sramp;
    sramp.x0 = 0;
    sramp.x1 = 1;
    sramp.dx0 = 0;
    sramp.dx1 = 0;
    Real scale=0.0;
    Real svmax=Inf,samax=Inf;
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].x0=x0[i];
        ramps[i].x1=x1[i];
        ramps[i].dx0=dx0[i];
        ramps[i].dx1=dx1[i];
        PARABOLIC_RAMP_ASSERT(dx0[i]==0.0);
        PARABOLIC_RAMP_ASSERT(dx1[i]==0.0);
        if(vmax[i]==0 || amax[i]==0) {
            if(!FuzzyEquals(x0[i],x1[i],EpsilonX)) {
                PARABOLIC_RAMP_PERROR("index %d vmax = %.15e, amax = %.15e, X0 != X1 (%.15e != %.15e)\n",i,vmax[i],amax[i],x0[i],x1[i]);
                PARABOLIC_RAMP_ASSERT(0);
            }
            ramps[i].tswitch1=ramps[i].tswitch2=ramps[i].ttotal=0;
            ramps[i].a1=ramps[i].a1=ramps[i].v=0;
            continue;
        }
        if(vmax[i] < svmax*Abs(x1[i]-x0[i]))
            svmax = vmax[i]/Abs(x1[i]-x0[i]);
        if(amax[i] < samax*Abs(x1[i]-x0[i]))
            samax = amax[i]/Abs(x1[i]-x0[i]);
        scale = Max(scale,Abs(x1[i]-x0[i]));
    }

    if(scale == 0.0) {
        //must have equal start/end state
        SetConstant(x0);
        return true;
    }

    //this scale factor makes the problem more numerically stable for start/end
    //locations close to one another
    sramp.x1 = scale;
    bool res=sramp.SolveMinTime(samax*scale,svmax*scale);
    if(!res) {
        PARABOLIC_RAMP_PERROR("Warning in straight-line parameter solve\n");
        return false;
    }

    endTime = sramp.ttotal;
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].v = svmax * (x1[i]-x0[i]);
        ramps[i].a1 = samax * (x1[i]-x0[i]);
        ramps[i].a2 = -samax * (x1[i]-x0[i]);
        ramps[i].tswitch1 = sramp.tswitch1;
        ramps[i].tswitch2 = sramp.tswitch2;
        ramps[i].ttotal = endTime;
        if(1 ) { //gValidityCheckLevel >= 2) {
            if(!ramps[i].IsValid()) {
                PARABOLIC_RAMP_PERROR("Warning, error in straight-line path formula\n");
                for(size_t j=0; j<dx0.size(); j++)
                    PARABOLIC_RAMP_PERROR("%.15e ",dx0[j]);
                for(size_t j=0; j<dx1.size(); j++)
                    PARABOLIC_RAMP_PERROR("%.15e ",dx1[j]);
            }
        }

        //correct for small numerical errors
        if(Abs(ramps[i].v) > vmax[i]) {
            if(Abs(ramps[i].v) > vmax[i]+EpsilonV) {
                PARABOLIC_RAMP_PERROR("Warning, numerical error in straight-line formula?\n");
                PARABOLIC_RAMP_PERROR("velocity |%.15e|>%.15e\n",ramps[i].v,vmax[i]);
            }
            else ramps[i].v = Sign(ramps[i].v)*vmax[i];
        }
        if(Abs(ramps[i].a1) > amax[i]) {
            if(Abs(ramps[i].a1) > amax[i]+EpsilonA) {
                PARABOLIC_RAMP_PERROR("Warning, numerical error in straight-line formula?\n");
                PARABOLIC_RAMP_PERROR("accel |%.15e|>%.15e\n",ramps[i].a1,amax[i]);
            }
            else ramps[i].a1 = Sign(ramps[i].a1)*amax[i];
        }
        if(Abs(ramps[i].a2) > amax[i]) {
            if(Abs(ramps[i].a2) > amax[i]+EpsilonA) {
                PARABOLIC_RAMP_PERROR("Warning, numerical error in straight-line formula?\n");
                PARABOLIC_RAMP_PERROR("accel |%.15e|>%.15e\n",ramps[i].a2,amax[i]);
            }
            else ramps[i].a2 = Sign(ramps[i].a2)*amax[i];
        }
    }
    return true;
}

bool ParabolicRampND::SolveMinTime(const Vector& amax,const Vector& vmax)
{
    PARABOLIC_RAMP_ASSERT(x0.size() == dx0.size());
    PARABOLIC_RAMP_ASSERT(x1.size() == dx1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == x1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == amax.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == vmax.size());
    endTime = 0;
    ramps.resize(x0.size());
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].x0=x0[i];
        ramps[i].x1=x1[i];
        ramps[i].dx0=dx0[i];
        ramps[i].dx1=dx1[i];
        if(vmax[i]==0 || amax[i]==0) {
            if(!FuzzyEquals(x0[i],x1[i],EpsilonX)) {
                PARABOLIC_RAMP_PLOG("index %d vmax = %.15e, amax = %.15e, X0 != X1 (%.15e != %.15e)\n",i,vmax[i],amax[i],x0[i],x1[i]);
                return false;
            }
            if(!FuzzyEquals(dx0[i],dx1[i],EpsilonV)) {
                PARABOLIC_RAMP_PLOG("index %d vmax = %.15e, amax = %.15e, DX0 != DX1 (%.15e != %.15e)\n",i,vmax[i],amax[i],dx0[i],dx1[i]);
                return false;
            }
            ramps[i].tswitch1=ramps[i].tswitch2=ramps[i].ttotal=0;
            ramps[i].a1=ramps[i].a2=ramps[i].v=0;
            continue;
        }
        if(!ramps[i].SolveMinTime(amax[i],vmax[i])) return false;
        if(ramps[i].ttotal > endTime) endTime = ramps[i].ttotal;
    }
    //now we have a candidate end time -- repeat looking through solutions
    //until we have solved all ramps
    while(true) {
        bool solved = true;
        for(size_t i=0; i<ramps.size(); i++) {
            if(ramps[i].ttotal == endTime) continue;
            if(vmax[i]==0 || amax[i]==0) {
                ramps[i].ttotal = endTime;
                continue;
            }
            if(!ramps[i].SolveMinAccel(endTime,vmax[i])) {
                PARABOLIC_RAMP_PLOG("Failed solving min accel for joint %d\n",i);
                ramps[i].SolveMinTime(amax[i],vmax[i]);
                PARABOLIC_RAMP_PLOG("its min time is %.15e\n",ramps[i].ttotal);
                if(ramps[i].tswitch1==ramps[i].tswitch2)
                    PARABOLIC_RAMP_PLOG("its type is PP\n");
                else if(Abs(ramps[i].v)==vmax[i])
                    PARABOLIC_RAMP_PLOG("its type is PLP (vmax)\n");
                else
                    PARABOLIC_RAMP_PLOG("its type is PLP (v=%.15e %%)\n",ramps[i].v/vmax[i]);
                SaveRamp("ParabolicRampND_SolveMinAccel_failure.dat",ramps[i].x0,ramps[i].dx0,ramps[i].x1,ramps[i].dx1,-1,vmax[i],endTime);
                PARABOLIC_RAMP_PLOG("Saving to failed_ramps.txt\n");
                FILE* f=fopen("failed_ramps.txt","w+");
                fprintf(f,"MinAccel T=%.15e, vmax=%.15e\n",endTime,vmax[i]);
                fprintf(f,"x0=%.15e, dx0=%.15e\n",ramps[i].x0,ramps[i].dx0);
                fprintf(f,"x1=%.15e, dx1=%.15e\n",ramps[i].x1,ramps[i].dx1);
                fprintf(f,"MinTime solution v=%.15e, t1=%.15e, t2=%.15e, T=%.15e\n",ramps[i].v,ramps[i].tswitch1,ramps[i].tswitch2,ramps[i].ttotal);
                fprintf(f,"\n");
                fclose(f);
                return false;
            }
            if(Abs(ramps[i].a1) > EpsilonA+amax[i] || Abs(ramps[i].a2) > EpsilonA+amax[i] || Abs(ramps[i].v) > EpsilonV+vmax[i]) {
                bool res=ramps[i].SolveMinTime2(amax[i],vmax[i],endTime);
                if(!res) {
                    PARABOLIC_RAMP_PLOG("Couldn't solve min-time with lower bound!\n");
                    return false;
                }
                PARABOLIC_RAMP_ASSERT(ramps[i].ttotal > endTime);
                endTime = ramps[i].ttotal;
                solved = false;
                break; //go back and re-solve
            }
            PARABOLIC_RAMP_ASSERT(Abs(ramps[i].a1) <= amax[i]+EpsilonA);
            PARABOLIC_RAMP_ASSERT(Abs(ramps[i].a2) <= amax[i]+EpsilonA);
            PARABOLIC_RAMP_ASSERT(Abs(ramps[i].v) <= vmax[i]+EpsilonV);
            PARABOLIC_RAMP_ASSERT(ramps[i].ttotal==endTime);
        }
        //done
        if(solved) break;
    }
    return true;
}

bool ParabolicRampND::SolveMinAccel(const Vector& vmax,Real time)
{
    PARABOLIC_RAMP_ASSERT(x0.size() == dx0.size());
    PARABOLIC_RAMP_ASSERT(x1.size() == dx1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == x1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == vmax.size());
    endTime = time;
    ramps.resize(x0.size());
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].x0=x0[i];
        ramps[i].x1=x1[i];
        ramps[i].dx0=dx0[i];
        ramps[i].dx1=dx1[i];
        if(vmax[i]==0) {
            PARABOLIC_RAMP_ASSERT(FuzzyEquals(x0[i],x1[i],EpsilonX));
            PARABOLIC_RAMP_ASSERT(FuzzyEquals(dx0[i],dx1[i],EpsilonV));
            ramps[i].tswitch1=ramps[i].tswitch2=ramps[i].ttotal=0;
            ramps[i].a1=ramps[i].a2=ramps[i].v=0;
            continue;
        }
        if(!ramps[i].SolveMinAccel(endTime,vmax[i])) {
            return false;
        }
    }
    return true;
}

bool ParabolicRampND::SolveMinAccelLinear(const Vector& vmax,Real time)
{
    PARABOLIC_RAMP_ASSERT(x0.size() == dx0.size());
    PARABOLIC_RAMP_ASSERT(x1.size() == dx1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == x1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == vmax.size());
    endTime = 0;
    ramps.resize(x0.size());
    ParabolicRamp1D sramp;
    sramp.x0 = 0;
    sramp.x1 = 1;
    sramp.dx0 = 0;
    sramp.dx1 = 0;
    Real svmax=Inf;
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].x0=x0[i];
        ramps[i].x1=x1[i];
        ramps[i].dx0=dx0[i];
        ramps[i].dx1=dx1[i];
        if(vmax[i]==0) {
            if(!FuzzyEquals(x0[i],x1[i],EpsilonX)) {
                PARABOLIC_RAMP_PLOG("index %d vmax = %.15e, X0 != X1 (%.15e != %.15e)\n",i,vmax[i],x0[i],x1[i]);
                return false;
            }
            if(!FuzzyEquals(dx0[i],dx1[i],EpsilonV)) {
                PARABOLIC_RAMP_PLOG("index %d vmax = %.15e, DX0 != DX1 (%.15e != %.15e)\n",i,vmax[i],dx0[i],dx1[i]);
                return false;
            }
            ramps[i].tswitch1=ramps[i].tswitch2=ramps[i].ttotal=0;
            ramps[i].a1=ramps[i].a1=ramps[i].v=0;
            continue;
        }
        if(vmax[i] < svmax*Abs(x1[i]-x0[i]))
            svmax = vmax[i]/Abs(x1[i]-x0[i]);
    }

    if(IsInf(svmax)) {
        //must have equal start/end state
        SetConstant(x0);
        return true;
    }

    bool res=sramp.SolveMinAccel(svmax,time);
    if(!res) {
        PARABOLICWARN("Warning in straight-line parameter solve\n");
        return false;
    }

    endTime = sramp.ttotal;
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].v = sramp.v * (x1[i]-x0[i]);
        ramps[i].a1 = sramp.a1 * (x1[i]-x0[i]);
        ramps[i].a2 = sramp.a2 * (x1[i]-x0[i]);
        ramps[i].tswitch1 = sramp.tswitch1;
        ramps[i].tswitch2 = sramp.tswitch2;
        ramps[i].ttotal = endTime;
        if(!ramps[i].IsValid()) {
            PARABOLICWARN("Warning, error in straight-line path formula\n");
            res=false;
        }
    }
    return res;
}

void ParabolicRampND::SolveBraking(const Vector& amax)
{
    PARABOLIC_RAMP_ASSERT(x0.size() == dx0.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == amax.size());
    x1.resize(x0.size());
    dx1.resize(x0.size());
    endTime = 0;
    ramps.resize(x0.size());
    for(size_t i=0; i<ramps.size(); i++) {
        if(amax[i]==0) {
            if(!FuzzyEquals(dx0[i],0.0,EpsilonV)) {
                PARABOLIC_RAMP_PLOG("index %d amax = %.15e, DX0 != 0 (%.15e != 0)\n",i,amax[i],dx0[i]);
                PARABOLIC_RAMP_ASSERT(0);
            }
            ramps[i].SetConstant(0);
            continue;
        }
        ramps[i].x0 = x0[i];
        ramps[i].dx0 = dx0[i];
        ramps[i].SolveBraking(amax[i]);
    }
    for(size_t i=0; i<ramps.size(); i++)
        endTime = Max(endTime,ramps[i].ttotal);
    for(size_t i=0; i<ramps.size(); i++) {
        if(amax[i] != 0 && ramps[i].ttotal != endTime) {
            //scale ramp acceleration to meet endTimeMax
            ramps[i].ttotal = endTime;
            //y(t) = x0 + t*dx0 + 1/2 t^2 a
            //y'(T) = dx0 + T a = 0
            ramps[i].a2 = -dx0[i] / endTime;
            ramps[i].a1 = -ramps[i].a2;
            ramps[i].x1 = ramps[i].x0 + endTime*ramps[i].dx0 + 0.5*Sqr(endTime)*ramps[i].a2;
        }
        else if(amax[i] == 0.0) {
            //end time of constant path
            ramps[i].ttotal = endTime;
        }
        x1[i]=ramps[i].x1;
        dx1[i]=0;
    }
    PARABOLIC_RAMP_ASSERT(IsValid());
}

void ParabolicRampND::Evaluate(Real t,Vector& x) const
{
    x.resize(ramps.size());
    for(size_t j=0; j<ramps.size(); j++)
        x[j]=ramps[j].Evaluate(t);
}

void ParabolicRampND::Derivative(Real t,Vector& x) const
{
    x.resize(ramps.size());
    for(size_t j=0; j<ramps.size(); j++)
        x[j]=ramps[j].Derivative(t);
}

void ParabolicRampND::Accel(Real t,Vector& x) const
{
    x.resize(ramps.size());
    for(size_t j=0; j<ramps.size(); j++)
        x[j]=ramps[j].Accel(t);
}

void ParabolicRampND::Output(Real dt,std::vector<Vector>& path) const
{
    PARABOLIC_RAMP_ASSERT(!ramps.empty());
    int size = (int)ceil(endTime/dt)+1;
    path.resize(size);
    if(size == 1) {
        path[0].resize(ramps.size());
        for(size_t j=0; j<ramps.size(); j++)
            path[0][j] = ramps[j].x0;
        return;
    }
    for(int i=0; i<size; i++) {
        Real t=endTime*Real(i)/Real(size-1);
        path[i].resize(ramps.size());
        for(size_t j=0; j<ramps.size(); j++)
            path[i][j]=ramps[j].Evaluate(t);
    }

    /*
       path[0].resize(ramps.size());
       for(size_t j=0;j<ramps.size();j++)
       path[0][j] = ramps[j].x0;
       for(int i=1;i+1<size;i++) {
       Real t=endTime*Real(i)/Real(size-1);
       path[i].resize(ramps.size());
       for(size_t j=0;j<ramps.size();j++)
        path[i][j]=ramps[j].Evaluate(t);
       }
       path[size-1].resize(ramps.size());
       for(size_t j=0;j<ramps.size();j++)
       path[size-1][j] = ramps[j].x1;
     */
}


void ParabolicRampND::Dilate(Real timeScale)
{
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].Dilate(timeScale);
        dx0[i] = ramps[i].dx0;
        dx1[i] = ramps[i].dx1;
    }
    endTime *= timeScale;
}

void ParabolicRampND::TrimFront(Real tcut)
{
    PARABOLIC_RAMP_ASSERT(tcut <= endTime);
    Evaluate(tcut,x0);
    Derivative(tcut,dx0);
    endTime -= tcut;
    for(size_t i=0; i<ramps.size(); i++)
        ramps[i].TrimFront(tcut);
    PARABOLIC_RAMP_ASSERT(IsValid());
}

void ParabolicRampND::TrimBack(Real tcut)
{
    for(size_t i=0; i<ramps.size(); i++)
        PARABOLIC_RAMP_ASSERT(endTime == ramps[i].ttotal);
    PARABOLIC_RAMP_ASSERT(tcut <= endTime);
    Evaluate(endTime-tcut,x1);
    Derivative(endTime-tcut,dx1);
    endTime -= tcut;
    for(size_t i=0; i<ramps.size(); i++)
        ramps[i].TrimBack(tcut);
    PARABOLIC_RAMP_ASSERT(IsValid());
}

void ParabolicRampND::Bounds(Vector& xmin,Vector& xmax) const
{
    xmin.resize(ramps.size());
    xmax.resize(ramps.size());
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].Bounds(xmin[i],xmax[i]);
    }
}

void ParabolicRampND::Bounds(Real ta,Real tb,Vector& xmin,Vector& xmax) const
{
    xmin.resize(ramps.size());
    xmax.resize(ramps.size());
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].Bounds(ta,tb,xmin[i],xmax[i]);
    }
}

void ParabolicRampND::DerivBounds(Vector& vmin,Vector& vmax) const
{
    vmin.resize(ramps.size());
    vmax.resize(ramps.size());
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].DerivBounds(vmin[i],vmax[i]);
    }
}

void ParabolicRampND::DerivBounds(Real ta,Real tb,Vector& vmin,Vector& vmax) const
{
    vmin.resize(ramps.size());
    vmax.resize(ramps.size());
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].DerivBounds(ta,tb,vmin[i],vmax[i]);
    }
}

bool ParabolicRampND::IsValid() const
{
    if(endTime < 0) {
        PARABOLICWARN("ParabolicRampND::IsValid(): endTime is negative\n");
        return false;
    }
    for(size_t i=0; i<ramps.size(); i++) {
        if(!ramps[i].IsValid()) {
            PARABOLICWARN("ParabolicRampND::IsValid(): element %d is invalid\n",i);
            return false;
        }
        if(!FuzzyEquals(ramps[i].ttotal,endTime,EpsilonT)) {
            PARABOLICWARN("ParabolicRampND::IsValid(): element %d has different end time %.15e != %.15e\n",i,ramps[i].ttotal,endTime);
            return false;
        }
        if(!FuzzyEquals(ramps[i].x0,x0[i],EpsilonX)) {
            PARABOLICWARN("ParabolicRampND::IsValid(): element %d has different x0 %.15e != %.15e\n",i,ramps[i].x0,x0[i]);
            return false;
        }
        if(!FuzzyEquals(ramps[i].x1,x1[i],EpsilonX)) {
            PARABOLICWARN("ParabolicRampND::IsValid(): element %d has different x1 %.15e != %.15e\n",i,ramps[i].x1,x1[i]);
            return false;
        }
        if(!FuzzyEquals(ramps[i].dx0,dx0[i],EpsilonV)) {
            PARABOLICWARN("ParabolicRampND::IsValid(): element %d has different dx0 %.15e != %.15e\n",i,ramps[i].dx0,dx0[i]);
            return false;
        }
        if(!FuzzyEquals(ramps[i].dx1,dx1[i],EpsilonV)) {
            PARABOLICWARN("ParabolicRampND::IsValid(): element %d has different dx1 %.15e != %.15e\n",i,ramps[i].dx1,dx1[i]);
            return false;
        }
    }
    return true;
}

bool SolveMinTimeBounded(Real x0,Real v0,Real x1,Real v1,Real amax,Real vmax,Real xmin,Real xmax,ParabolicRamp1D& ramp)
{
    PARABOLIC_RAMP_ASSERT(x0 >= xmin-EpsilonX && x0 <= xmax+EpsilonX && x1 >= xmin-EpsilonX && x1 <= xmax+EpsilonX);
    ramp.x0 = x0;
    ramp.dx0 = v0;
    ramp.x1 = x1;
    ramp.dx1 = v1;
    if(!ramp.SolveMinTime(amax,vmax)) return false;
    Real bmin,bmax;
    ramp.Bounds(bmin,bmax);
    if(bmin < xmin || bmax > xmax) return false;
    return true;
}

inline Real BrakeTime(Real x,Real v,Real xbound)
{
    Real t;
    bool res=SafeEqSolve(v,2.0*(xbound-x),EpsilonX,0,Inf,t);
    if(!res) {
        PARABOLICWARN("Warning, couldn't solve brake time equation:\n");
        PARABOLICWARN("%.15e*a = %.15e = 0\n",v,2.0*(xbound-x));
        return 0;
    }
    return t;
}

inline Real BrakeAccel(Real x,Real v,Real xbound)
{
    Real a;
    bool res=SafeEqSolve(2.0*(xbound-x),-v*v,EpsilonV,-Inf,Inf,a);
    if(!res) {
        PARABOLICWARN("Warning, couldn't solve braking acceleration equation:\n");
        PARABOLICWARN("%.15e*a + %.15e = 0\n",2.0*(xbound-x),v*v);
        return 0;
    }
    return a;
}

bool SolveMinAccelBounded(Real x0,Real v0,Real x1,Real v1,Real endTime,Real vmax,Real xmin,Real xmax,std::vector<ParabolicRamp1D>& ramps)
{
    PARABOLIC_RAMP_ASSERT(x0 >= xmin-EpsilonX && x0 <= xmax+EpsilonX && x1 >= xmin-EpsilonX && x1 <= xmax+EpsilonX);
    ParabolicRamp1D ramp;
    ramp.x0 = x0;
    ramp.dx0 = v0;
    ramp.x1 = x1;
    ramp.dx1 = v1;
    if(!ramp.SolveMinAccel(endTime,vmax)) return false;
    Real bmin,bmax;
    ramp.Bounds(bmin,bmax);
    if(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX) {
        ramps.resize(1);
        ramps[0] = ramp;
        return true;
    }

    //not within bounds, do the more complex procedure
    ramps.resize(0);
    vector<ParabolicRamp1D> temp;
    //Look at the IV cases
    Real bt0=Inf,bt1=Inf;
    Real ba0=Inf,ba1=Inf;
    Real bx0=Inf,bx1=Inf;
    if(v0 > 0) {
        bt0 = BrakeTime(x0,v0,xmax);
        bx0 = xmax;
        ba0 = BrakeAccel(x0,v0,xmax);
    }
    else if(v0 < 0) {
        bt0 = BrakeTime(x0,v0,xmin);
        bx0 = xmin;
        ba0 = BrakeAccel(x0,v0,xmin);
    }
    if(v1 < 0) {
        bt1 = BrakeTime(x1,-v1,xmax);
        bx1 = xmax;
        ba1 = BrakeAccel(x1,-v1,xmax);
    }
    else if(v1 > 0) {
        bt1 = BrakeTime(x1,-v1,xmin);
        bx1 = xmin;
        ba1 = BrakeAccel(x1,-v1,xmin);
    }
    Real amax=Inf;
    //Explore types II and III, or II and IV depending on the side
    //Type I path: no collision
    //Type II path: touches one side instantaneously
    //   (IIa: first segment is braking, IIb: last segment is braking)
    //Type III path: touches one side and remains there for some time
    //Type IV path: hits both top and bottom
    //consider braking to side, then solving to x1,v1
    if(bt0 < endTime && Abs(ba0) < amax+EpsilonA) {
        //type IIa
        temp.resize(2);
        temp[0].x0 = x0;
        temp[0].dx0 = v0;
        temp[0].x1 = bx0;
        temp[0].dx1 = 0;
        temp[0].a1 = ba0;
        temp[0].v = 0;
        temp[0].a2 = 0;
        temp[0].tswitch1 = bt0;
        temp[0].tswitch2 = bt0;
        temp[0].ttotal = bt0;
        temp[1].x0 = bx0;
        temp[1].dx0 = 0;
        temp[1].x1 = x1;
        temp[1].dx1 = v1;
        gMinAccelQuiet = true;
        //first check is a quick reject
        if(Abs(x1-bx0) < (endTime-bt0)*vmax) {
            if(temp[1].SolveMinAccel(endTime-bt0,vmax)) {
                if(Max(Abs(temp[1].a1),Abs(temp[1].a2)) < amax) {
                    temp[1].Bounds(bmin,bmax);
                    if(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX) {
                        //got a better path
                        ramps = temp;
                        amax = Max(Abs(ba0),Max(Abs(temp[1].a1),Abs(temp[1].a2)));
                    }
                }
            }
        }
        gMinAccelQuiet = false;
    }
    //consider reverse braking from x1,v1, then solving from x0,v0
    //consider braking to side, then solving to x1,v1
    if(bt1 < endTime && Abs(ba1) < amax+EpsilonA) {
        //type IIb
        temp.resize(2);
        temp[0].x0 = x0;
        temp[0].dx0 = v0;
        temp[0].x1 = bx1;
        temp[0].dx1 = 0;
        temp[1].x0 = bx1;
        temp[1].dx0 = 0;
        temp[1].x1 = x1;
        temp[1].dx1 = v1;
        temp[1].a1 = ba1;
        temp[1].v = 0;
        temp[1].a2 = 0;
        temp[1].tswitch1 = bt1;
        temp[1].tswitch2 = bt1;
        temp[1].ttotal = bt1;
        gMinAccelQuiet = true;
        //first perform a quick reject
        if(Abs(x0-bx1) < (endTime-bt1)*vmax) {
            if(temp[0].SolveMinAccel(endTime-bt1,vmax)) {
                if(Max(Abs(temp[0].a1),Abs(temp[0].a2)) < amax) {
                    temp[0].Bounds(bmin,bmax);
                    if(bmin >= xmin && bmax <= xmax) {
                        //got a better path
                        ramps = temp;
                        amax = Max(Abs(ba1),Max(Abs(temp[0].a1),Abs(temp[0].a2)));
                    }
                }
            }
        }
        gMinAccelQuiet = false;
    }
    if(bx0 == bx1) {
        //type III: braking to side, then continuing, then accelerating to x1
        if(bt0 + bt1 < endTime && Max(Abs(ba0),Abs(ba1)) < amax+EpsilonA) {
            temp.resize(1);
            temp[0].x0 = x0;
            temp[0].dx0 = v0;
            temp[0].x1 = x1;
            temp[0].dx1 = v1;
            temp[0].a1 = ba0;
            temp[0].v = 0;
            temp[0].a2 = ba1;
            temp[0].tswitch1 = bt0;
            temp[0].tswitch2 = endTime-bt1;
            temp[0].ttotal = endTime;
            ramps = temp;
            amax = Max(Abs(ba0),Abs(ba1));
            PARABOLIC_RAMP_ASSERT(temp[0].IsValid());
        }
    }
    else {
        //type IV paths
        if(bt0 + bt1 < endTime && Max(Abs(ba0),Abs(ba1)) < amax) {
            //first segment brakes to one side, last segment brakes to the other
            //first
            temp.resize(3);
            temp[0].x0 = x0;
            temp[0].dx0 = v0;
            temp[0].x1 = bx0;
            temp[0].dx1 = 0;
            temp[0].a1 = ba0;
            temp[0].v = 0;
            temp[0].a2 = 0;
            temp[0].tswitch1 = bt0;
            temp[0].tswitch2 = bt0;
            temp[0].ttotal = bt0;
            //last
            temp[2].x0 = bx1;
            temp[2].dx0 = 0;
            temp[2].x1 = x1;
            temp[2].dx1 = v1;
            temp[2].a1 = ba1;
            temp[2].v = 0;
            temp[2].a2 = 0;
            temp[2].tswitch1 = bt1;
            temp[2].tswitch2 = bt1;
            temp[2].ttotal = bt1;
            //middle section
            temp[1].x0 = bx0;
            temp[1].dx0 = 0;
            temp[1].x1 = bx1;
            temp[1].dx1 = 0;
            gMinAccelQuiet = true;
            if(Abs(bx0-bx1) < (endTime-bt0-bt1)*vmax) {
                if(temp[1].SolveMinAccel(endTime - bt0 - bt1,vmax)) {
                    temp[1].Bounds(bmin,bmax);
                    PARABOLIC_RAMP_ASSERT(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX);
                    if(Max(Abs(temp[1].a1),Abs(temp[1].a2)) < amax) {
                        ramps = temp;
                        amax = Max(Max(Abs(temp[1].a1),Abs(temp[1].a2)),Max(Abs(ba0),Abs(ba1)));
                    }
                }
            }
            gMinAccelQuiet = false;
        }
    }
    if(ramps.empty()) {
        PARABOLIC_RAMP_PLOG("SolveMinAccelBounded: Warning, can't find bounded trajectory?\n");
        PARABOLIC_RAMP_PLOG("x0 %.15e v0 %.15e, x1 %.15e v1 %.15e\n",x0,v0,x1,v1);
        PARABOLIC_RAMP_PLOG("endTime %.15e, vmax %.15e\n",endTime,vmax);
        PARABOLIC_RAMP_PLOG("x bounds [%.15e,%.15e]\n",xmin,xmax);
        return false;
    }
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].Bounds(bmin,bmax);
        if(bmin < xmin-EpsilonX || bmax > xmax+EpsilonX) {
            PARABOLIC_RAMP_PLOG("SolveMinAccelBounded: Warning, path exceeds bounds?\n");
            PARABOLIC_RAMP_PLOG("  ramp[%d] bounds %.15e %.15e, limits %.15e %.15e\n",i,bmin,bmax,xmin,xmax);
            return false;
        }
    }

    PARABOLIC_RAMP_ASSERT(ramps.front().x0 == x0);
    PARABOLIC_RAMP_ASSERT(ramps.front().dx0 == v0);
    PARABOLIC_RAMP_ASSERT(ramps.back().x1 == x1);
    PARABOLIC_RAMP_ASSERT(ramps.back().dx1 == v1);
    double ttotal = 0;
    for(size_t i=0; i<ramps.size(); i++)
        ttotal += ramps[i].ttotal;
    for(size_t i=0; i<ramps.size(); i++) {
        if(Abs(ramps[i].ttotal) == 0.0) {
            ramps.erase(ramps.begin()+i);
            i--;
        }
    }
    if(!FuzzyEquals(ttotal,endTime,EpsilonT*0.1)) {
        PARABOLIC_RAMP_PLOG("Ramp times: ");
        for(size_t i=0; i<ramps.size(); i++)
            PARABOLIC_RAMP_PLOG("%.15e ",ramps[i].ttotal);
        PARABOLIC_RAMP_PLOG("\n");
    }
    PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal,endTime,EpsilonT*0.1));
    return true;
}

bool SolveMaxAccel(Real x0,Real v0,Real x1,Real v1,Real endTime,Real amax, Real vmax,Real xmin,Real xmax,std::vector<ParabolicRamp1D>& ramps)
{
    PARABOLIC_RAMP_ASSERT(x0 >= xmin-EpsilonX && x0 <= xmax+EpsilonX && x1 >= xmin-EpsilonX && x1 <= xmax+EpsilonX);
    ParabolicRamp1D ramp;
    ramp.x0 = x0;
    ramp.dx0 = v0;
    ramp.x1 = x1;
    ramp.dx1 = v1;
    if(!ramp.SolveFixedTime(amax,vmax,endTime)) {
        return false;
    }
    Real bmin,bmax;
    ramp.Bounds(bmin,bmax);
    if(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX) {
        ramps.resize(1);
        ramps[0] = ramp;
        return true;
    }
    return false;
}


bool SolveMaxAccelBounded(Real x0,Real v0,Real x1,Real v1,Real endTime,Real amax, Real vmax,Real xmin,Real xmax,std::vector<ParabolicRamp1D>& ramps)
{
    PARABOLIC_RAMP_ASSERT(x0 >= xmin-EpsilonX && x0 <= xmax+EpsilonX && x1 >= xmin-EpsilonX && x1 <= xmax+EpsilonX);
    ParabolicRamp1D ramp;
    ramp.x0 = x0;
    ramp.dx0 = v0;
    ramp.x1 = x1;
    ramp.dx1 = v1;
    if(!ramp.SolveFixedTime(amax,vmax,endTime)) return false;
    Real bmin,bmax;
    ramp.Bounds(bmin,bmax);
    if(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX) {
        ramps.resize(1);
        ramps[0] = ramp;
        return true;
    }

    //not within bounds, do the more complex procedure
    ramps.resize(0);
    vector<ParabolicRamp1D> temp;
    //Look at the IV cases
    Real bt0=Inf,bt1=Inf;
    Real ba0=Inf,ba1=Inf;
    Real bx0=Inf,bx1=Inf;
    if(v0 > 0) {
        bt0 = BrakeTime(x0,v0,xmax);
        bx0 = xmax;
        ba0 = BrakeAccel(x0,v0,xmax);
    }
    else if(v0 < 0) {
        bt0 = BrakeTime(x0,v0,xmin);
        bx0 = xmin;
        ba0 = BrakeAccel(x0,v0,xmin);
    }
    if(v1 < 0) {
        bt1 = BrakeTime(x1,-v1,xmax);
        bx1 = xmax;
        ba1 = BrakeAccel(x1,-v1,xmax);
    }
    else if(v1 > 0) {
        bt1 = BrakeTime(x1,-v1,xmin);
        bx1 = xmin;
        ba1 = BrakeAccel(x1,-v1,xmin);
    }

    //PARABOLICWARN("max acceleration violates boundaries, solving with min accel");
    //Real amax=Inf;

    //Explore types II and III, or II and IV depending on the side
    //Type I path: no collision
    //Type II path: touches one side instantaneously
    //   (IIa: first segment is braking, IIb: last segment is braking)
    //Type III path: touches one side and remains there for some time
    //Type IV path: hits both top and bottom
    //consider braking to side, then solving to x1,v1
    if(bt0 < endTime && Abs(ba0) < amax) {
        //type IIa
        temp.resize(2);
        temp[0].x0 = x0;
        temp[0].dx0 = v0;
        temp[0].x1 = bx0;
        temp[0].dx1 = 0;
        temp[0].a1 = ba0;
        temp[0].v = 0;
        temp[0].a2 = 0;
        temp[0].tswitch1 = bt0;
        temp[0].tswitch2 = bt0;
        temp[0].ttotal = bt0;
        temp[1].x0 = bx0;
        temp[1].dx0 = 0;
        temp[1].x1 = x1;
        temp[1].dx1 = v1;
        gMinAccelQuiet = true;
        //first check is a quick reject
        if(Abs(x1-bx0) < (endTime-bt0)*vmax) {
            if(temp[1].SolveMinAccel(endTime-bt0,vmax)) {
                if(Max(Abs(temp[1].a1),Abs(temp[1].a2)) < amax) {
                    temp[1].Bounds(bmin,bmax);
                    if(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX) {
                        //got a better path
                        ramps = temp;
                        amax = Max(Abs(ba0),Max(Abs(temp[1].a1),Abs(temp[1].a2)));
                    }
                }
            }
        }
        gMinAccelQuiet = false;
    }
    //consider reverse braking from x1,v1, then solving from x0,v0
    //consider braking to side, then solving to x1,v1
    if(bt1 < endTime && Abs(ba1) < amax) {
        //type IIb
        temp.resize(2);
        temp[0].x0 = x0;
        temp[0].dx0 = v0;
        temp[0].x1 = bx1;
        temp[0].dx1 = 0;
        temp[1].x0 = bx1;
        temp[1].dx0 = 0;
        temp[1].x1 = x1;
        temp[1].dx1 = v1;
        temp[1].a1 = ba1;
        temp[1].v = 0;
        temp[1].a2 = 0;
        temp[1].tswitch1 = bt1;
        temp[1].tswitch2 = bt1;
        temp[1].ttotal = bt1;
        gMinAccelQuiet = true;
        //first perform a quick reject
        if(Abs(x0-bx1) < (endTime-bt1)*vmax) {
            if(temp[0].SolveMinAccel(endTime-bt1,vmax)) {
                if(Max(Abs(temp[0].a1),Abs(temp[0].a2)) < amax) {
                    temp[0].Bounds(bmin,bmax);
                    if(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX) {
                        //got a better path
                        ramps = temp;
                        amax = Max(Abs(ba1),Max(Abs(temp[0].a1),Abs(temp[0].a2)));
                    }
                }
            }
        }
        gMinAccelQuiet = false;
    }
    if(bx0 == bx1) {
        //type III: braking to side, then continuing, then accelerating to x1
        if(bt0 + bt1 < endTime && Max(Abs(ba0),Abs(ba1)) < amax) {
            temp.resize(1);
            temp[0].x0 = x0;
            temp[0].dx0 = v0;
            temp[0].x1 = x1;
            temp[0].dx1 = v1;
            temp[0].a1 = ba0;
            temp[0].v = 0;
            temp[0].a2 = ba1;
            temp[0].tswitch1 = bt0;
            temp[0].tswitch2 = endTime-bt1;
            temp[0].ttotal = endTime;
            ramps = temp;
            amax = Max(Abs(ba0),Abs(ba1));
            PARABOLIC_RAMP_ASSERT(temp[0].IsValid());
        }
    }
    else {
        //type IV paths
        if(bt0 + bt1 < endTime && Max(Abs(ba0),Abs(ba1)) < amax) {
            //first segment brakes to one side, last segment brakes to the other
            //first
            temp.resize(3);
            temp[0].x0 = x0;
            temp[0].dx0 = v0;
            temp[0].x1 = bx0;
            temp[0].dx1 = 0;
            temp[0].a1 = ba0;
            temp[0].v = 0;
            temp[0].a2 = 0;
            temp[0].tswitch1 = bt0;
            temp[0].tswitch2 = bt0;
            temp[0].ttotal = bt0;
            //last
            temp[2].x0 = bx1;
            temp[2].dx0 = 0;
            temp[2].x1 = x1;
            temp[2].dx1 = v1;
            temp[2].a1 = ba1;
            temp[2].v = 0;
            temp[2].a2 = 0;
            temp[2].tswitch1 = bt1;
            temp[2].tswitch2 = bt1;
            temp[2].ttotal = bt1;
            //middle section
            temp[1].x0 = bx0;
            temp[1].dx0 = 0;
            temp[1].x1 = bx1;
            temp[1].dx1 = 0;
            gMinAccelQuiet = true;
            if(Abs(bx0-bx1) < (endTime-bt0-bt1)*vmax) {
                if(temp[1].SolveMinAccel(endTime - bt0 - bt1,vmax)) {
                    temp[1].Bounds(bmin,bmax);
                    PARABOLIC_RAMP_ASSERT(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX);
                    if(Max(Abs(temp[1].a1),Abs(temp[1].a2)) < amax) {
                        ramps = temp;
                        amax = Max(Max(Abs(temp[1].a1),Abs(temp[1].a2)),Max(Abs(ba0),Abs(ba1)));
                    }
                }
            }
            gMinAccelQuiet = false;
        }
    }
    if(ramps.empty()) {
        PARABOLIC_RAMP_PLOG("SolveMinAccelBounded: Warning, can't find bounded trajectory?\n");
        PARABOLIC_RAMP_PLOG("x0 %.15e v0 %.15e, x1 %.15e v1 %.15e\n",x0,v0,x1,v1);
        PARABOLIC_RAMP_PLOG("endTime %.15e, vmax %.15e\n",endTime,vmax);
        PARABOLIC_RAMP_PLOG("x bounds [%.15e,%.15e]\n",xmin,xmax);
        return false;
    }
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].Bounds(bmin,bmax);
        if(bmin < xmin-EpsilonX || bmax > xmax+EpsilonX) {
            PARABOLIC_RAMP_PLOG("SolveMinAccelBounded: Warning, path exceeds bounds?\n");
            PARABOLIC_RAMP_PLOG("  ramp[%d] bounds %.15e %.15e, limits %.15e %.15e\n",i,bmin,bmax,xmin,xmax);
            return false;
        }
    }

    PARABOLIC_RAMP_ASSERT(ramps.front().x0 == x0);
    PARABOLIC_RAMP_ASSERT(ramps.front().dx0 == v0);
    PARABOLIC_RAMP_ASSERT(ramps.back().x1 == x1);
    PARABOLIC_RAMP_ASSERT(ramps.back().dx1 == v1);
    double ttotal = 0;
    for(size_t i=0; i<ramps.size(); i++)
        ttotal += ramps[i].ttotal;
    for(size_t i=0; i<ramps.size(); i++) {
        if(Abs(ramps[i].ttotal) == 0.0) {
            ramps.erase(ramps.begin()+i);
            i--;
        }
    }
    if(!FuzzyEquals(ttotal,endTime,EpsilonT*0.1)) {
        PARABOLIC_RAMP_PLOG("Ramp times: ");
        for(size_t i=0; i<ramps.size(); i++)
            PARABOLIC_RAMP_PLOG("%.15e ",ramps[i].ttotal);
        PARABOLIC_RAMP_PLOG("\n");
    }
    PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal,endTime,EpsilonT*0.1));
    return true;
}


Real SolveMinTimeBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1,
                         const Vector& amax,const Vector& vmax,const Vector& xmin,const Vector& xmax,
                         vector<vector<ParabolicRamp1D> >& ramps, int multidofinterp)
{
    //PARABOLIC_RAMP_PLOG("Size x0 %d\n",(int)x0.size());
    //PARABOLIC_RAMP_PLOG("Size amax %d\n",(int)amax.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == v0.size());
    PARABOLIC_RAMP_ASSERT(x1.size() == v1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == x1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == amax.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == vmax.size());
    for(size_t i=0; i<x0.size(); i++) {
        if(x0[i] < xmin[i]-EpsilonX || x0[i] > xmax[i]+EpsilonX) {
            PARABOLICWARN("Warning, start component %d=%.15e out of range [%.15e,%.15e]\n",i,x0[i],xmin[i],xmax[i]);
            return -1;
        }
        if(x1[i] < xmin[i]-EpsilonX || x1[i] > xmax[i]+EpsilonX) {
            PARABOLICWARN("Warning, goal component %d=%.15e out of range [%.15e,%.15e]\n",i,x1[i],xmin[i],xmax[i]);
            return -1;
        }
        PARABOLIC_RAMP_ASSERT(x0[i] >= xmin[i]-EpsilonX && x0[i] <= xmax[i]+EpsilonX);
        PARABOLIC_RAMP_ASSERT(x1[i] >= xmin[i]-EpsilonX && x1[i] <= xmax[i]+EpsilonX);
        if( Abs(v0[i]) > vmax[i]+EpsilonV ) {
            PARABOLICWARN("v0[%d] (%.15e) > vmax[%d] (%.15e)\n", i, Abs(v0[i]), i, vmax[i]);
        }
        PARABOLIC_RAMP_ASSERT(Abs(v0[i]) <= vmax[i]+EpsilonV);
        if( Abs(v1[i]) > vmax[i]+EpsilonV ) {
            PARABOLICWARN("v1[%d] (%.15e) > vmax[%d] (%.15e)\n", i, Abs(v1[i]), i, vmax[i]);
        }
        PARABOLIC_RAMP_ASSERT(Abs(v1[i]) <= vmax[i]+EpsilonV);
    }
    Real endTime = 0;
    ramps.resize(x0.size());
    for(size_t i=0; i<ramps.size(); i++) {
        ramps[i].resize(1);
        ramps[i][0].x0=x0[i];
        ramps[i][0].x1=x1[i];
        ramps[i][0].dx0=v0[i];
        ramps[i][0].dx1=v1[i];
        if(vmax[i]==0 || amax[i]==0) {
            if(!FuzzyEquals(x0[i],x1[i],EpsilonX)) {
                PARABOLIC_RAMP_PLOG("index %d vmax = %.15e, amax = %.15e, X0 != X1 (%.15e != %.15e)\n",i,vmax[i],amax[i],x0[i],x1[i]);
                return -1;
            }
            if(!FuzzyEquals(v0[i],v1[i],EpsilonV)) {
                PARABOLIC_RAMP_PLOG("index %d vmax = %.15e, amax = %.15e, DX0 != DX1 (%.15e != %.15e)\n",i,vmax[i],amax[i],v0[i],v1[i]);
                return -1;
            }
            ramps[i][0].tswitch1=ramps[i][0].tswitch2=ramps[i][0].ttotal=0;
            ramps[i][0].a1=ramps[i][0].a2=ramps[i][0].v=0;
            continue;
        }
        if(!ramps[i][0].SolveMinTime(amax[i],vmax[i])) {
            return -1;
        }
        Real bmin,bmax;
        ramps[i][0].Bounds(bmin,bmax);
        if(bmin < xmin[i]-EpsilonX || bmax > xmax[i]+EpsilonX) {
            Real originaltime = ramps[i][0].ttotal;
            bool bSuccess = false;
            for(Real itimemult = 1; itimemult <= 5; ++itimemult) {
                Real timemult = 1+itimemult*0.5; // perhaps should test different values?
                if( SolveMinAccelBounded(x0[i], v0[i], x1[i], v1[i], originaltime*timemult, vmax[i], xmin[i], xmax[i], ramps[i]) ) {
                    bSuccess = true;
                    // check acceleration limits
                    for(size_t j = 0; j < ramps[i].size(); ++j) {
                        if( Abs(ramps[i][j].a1) > amax[i]+EpsilonA ) {
                            PARABOLIC_RAMP_PLOG("min accel for joint %d is %.15e (> %.15e)\n",i, Abs(ramps[i][j].a1), amax[i]);
                            bSuccess = false;
                            break;
                        }
                        if( Abs(ramps[i][j].a2) > amax[i]+EpsilonA ) {
                            PARABOLIC_RAMP_PLOG("min accel for joint %d is %.15e (> %.15e)\n",i, Abs(ramps[i][j].a2), amax[i]);
                            bSuccess = false;
                            break;
                        }
                    }
                    if( bSuccess ) {
                        break;
                    }
                }
            }
            if( !bSuccess ) {
                PARABOLIC_RAMP_PLOG("ramp index %d failed due to boundary constraints (%.15e, %.15e) time=%f\n", i, bmin, bmax, originaltime);
                return -1;
            }
        }
        Real newtotal = 0;
        for(size_t j = 0; j < ramps[i].size(); ++j) {
            newtotal += ramps[i][j].ttotal;
        }
        if(newtotal > endTime) {
            endTime = newtotal;
        }
    }
    
    //now we have a candidate end time -- repeat looking through solutions
    //until we have solved all ramps
    int numiters = 0;
    int maxiters=10;
    bool solved = true;
    while(numiters < maxiters) {
        ++numiters;
        solved = true;
        for(size_t i=0; i<ramps.size(); i++) {
            PARABOLIC_RAMP_ASSERT(ramps[i].size() > 0);
            if(vmax[i]==0 || amax[i]==0) {
                // ?
                ramps[i][0].ttotal = endTime;
                continue;
            }
            //already at maximum
            Real ttotal = 0;
            for(size_t j=0; j<ramps[i].size(); j++) {
                ttotal += ramps[i][j].ttotal;
            }
            if(FuzzyEquals(ttotal,endTime,EpsilonT)) {
                continue;
            }

            //now solve minimum acceleration within bounds
            if( multidofinterp == 2 ) {
                if(!SolveMaxAccel(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],ramps[i])) {
                    // because SolveMaxAccel doesn't check for boundaries, this could fail
                    PARABOLIC_RAMP_PLOG("Failed solving bounded max accel for joint %d\n",i);
                    endTime *= 1.05;
                    solved = false;
                    break;
                }
            }
            else if( multidofinterp == 1 ) {
                if(!SolveMaxAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],ramps[i])) {
                    endTime *= 1.05;
                    solved = false;
                    break;
                }
            }
            else {
                if(!SolveMinAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,vmax[i],xmin[i],xmax[i],ramps[i])) {
                    PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d\n",i);
                    endTime *= 1.05;
                    solved = false;
                    break;
                }
            }

            //now check accel/velocity bounds
            bool inVelBounds = true;
            for(size_t j=0; j<ramps[i].size(); j++) {
                if(Abs(ramps[i][j].a1) > amax[i]+EpsilonA || Abs(ramps[i][j].a2) > amax[i]+EpsilonA || Abs(ramps[i][j].v) > vmax[i]+EpsilonV) {
                    //PARABOLIC_RAMP_PLOG("Ramp %d entry %d accels: %.15e %.15e, vel %.15e\n",i,j,ramps[i][j].a1,ramps[i][j].a2,ramps[i][j].v);
                    inVelBounds = false;
                    break;
                }
            }
            if(!inVelBounds) {
                ramps[i].resize(1);
                ramps[i][0].x0=x0[i];
                ramps[i][0].x1=x1[i];
                ramps[i][0].dx0=v0[i];
                ramps[i][0].dx1=v1[i];
                gMinTime2Quiet = true;
                bool res=ramps[i][0].SolveMinTime2(amax[i],vmax[i],endTime);
                gMinTime2Quiet = false;
                if(!res) {
                    return -1;
                }
                Real bmin,bmax;
                ramps[i][0].Bounds(bmin,bmax);
                if(bmin < xmin[i]-EpsilonX || bmax > xmax[i]+EpsilonX) {
                    //PARABOLIC_RAMP_PLOG("Couldn't solve min-time with lower bound while staying in bounds\n");
                    return -1;
                }

                //revise total time
                PARABOLIC_RAMP_ASSERT(ramps[i][0].ttotal >= endTime);
                endTime = ramps[i][0].ttotal;
                solved = false;
                break; //go back and re-solve
            }
            ttotal = 0;
            for(size_t j=0; j<ramps[i].size(); j++) {
                PARABOLIC_RAMP_ASSERT(Abs(ramps[i][j].a1) <= amax[i]+EpsilonA);
                PARABOLIC_RAMP_ASSERT(Abs(ramps[i][j].a2) <= amax[i]+EpsilonA);
                PARABOLIC_RAMP_ASSERT(Abs(ramps[i][j].v) <= vmax[i]+EpsilonV);
                ttotal += ramps[i][j].ttotal;
            }
            PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal,endTime,EpsilonT*0.1));
        }
        if( !solved ) {
            continue; //go back and re-solve
        }

        if( multidofinterp == 2 ) {
            // find the ramp that saturates the slowest and convert all switching points to it
            // because we're not computing with limits, there should only be one ramp
            for(size_t i=0; i < ramps.size(); ++i) {
                if(ramps[i].size()!=1) {
                    PARABOLICWARN("expected ramp size of 1\n");
                    return -1;
                }
            }

            size_t islowest = 0;
            for(size_t i=1; i < ramps.size(); ++i) {
                if( ramps[islowest][0].tswitch1 < ramps[i][0].tswitch1 ) {
                    islowest = i;
                }
            }

            for(size_t i=0; i < ramps.size(); ++i) {
                if( i != islowest ) {
                    ParabolicRamp1D newramp = ramps[i][0];
                    newramp.tswitch1 = ramps[islowest][0].tswitch1;
                    newramp.tswitch2 = ramps[islowest][0].tswitch2;
                    newramp.ttotal = ramps[islowest][0].ttotal;
                    if( newramp.SolveFixedSwitchTime(amax[i],vmax[i]) ) {
                        ramps[i][0] = newramp;
                    }
                    else {
                        PARABOLIC_RAMP_PLOG("failed to set correct ramp switch times %.15e, %.15e, %.15e for index %d, trying to re-time it!\n",newramp.tswitch1,newramp.tswitch2,newramp.ttotal,i);
                        if( 0 ) { //newramp.SolveFixedAccelSwitchTime(amax[i], vmax[i], ramps[islowest][0].tswitch1, ramps[islowest][0].ttotal - ramps[islowest][0].tswitch2) ) {
                            // time has to be slower
                            if( newramp.ttotal > endTime+EpsilonT ) {
                                PARABOLIC_RAMP_PLOG("set new time %.15es for index %d\n", newramp.ttotal);
                                endTime = newramp.ttotal;
                            }
                            else {
                                PARABOLIC_RAMP_PLOG("new time %.15es for index %d is not slower\n", newramp.ttotal, i);
                                // although there might be a way to compute this, just increase the time for now
                                endTime *= 1.05;
                            }
                        }
                        else {
                            // multiply by some ratio hoping to make things better?
                            endTime *= 1.05;
                        }
                        solved = false;
                        break;
                    }
                }
            }

            if( !solved ) {
                continue; //go back and re-solve
            }
        }

        if(solved) {
            break;
        }
    }
    if( !solved ) {
        return -1;
    }
    return endTime;
}

bool SolveMinAccelBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1,
                          Real endTime,const Vector& vmax,const Vector& xmin,const Vector& xmax,
                          vector<vector<ParabolicRamp1D> >& ramps)
{
    PARABOLIC_RAMP_ASSERT(x0.size() == v0.size());
    PARABOLIC_RAMP_ASSERT(x1.size() == v1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == x1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == vmax.size());
    for(size_t i=0; i<x0.size(); i++) {
        PARABOLIC_RAMP_ASSERT(x0[i] >= xmin[i]-EpsilonX && x0[i] <= xmax[i]+EpsilonX);
        PARABOLIC_RAMP_ASSERT(x1[i] >= xmin[i]-EpsilonX && x1[i] <= xmax[i]+EpsilonX);
        PARABOLIC_RAMP_ASSERT(Abs(v0[i]) <= vmax[i]+EpsilonV);
        PARABOLIC_RAMP_ASSERT(Abs(v1[i]) <= vmax[i]+EpsilonV);
    }
    ramps.resize(x0.size());
    for(size_t i=0; i<ramps.size(); i++) {
        if(vmax[i]==0) {
            ramps[i].resize(1);
            ramps[i][0].x0=x0[i];
            ramps[i][0].x1=x1[i];
            ramps[i][0].dx0=v0[i];
            ramps[i][0].dx1=v1[i];
            ramps[i][0].ttotal = endTime;
            continue;
        }
        //now solve minimum acceleration within bounds
        if(!SolveMinAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,vmax[i],xmin[i],xmax[i],ramps[i])) {
            PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d\n",i);
            return false;
        }
    }
    return true;
}

bool SolveAccelBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1,
                       Real endTime,const Vector& amax, const Vector& vmax,const Vector& xmin,const Vector& xmax,
                       vector<vector<ParabolicRamp1D> >& ramps, int multidofinterp)
{
    PARABOLIC_RAMP_ASSERT(x0.size() == v0.size());
    PARABOLIC_RAMP_ASSERT(x1.size() == v1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == x1.size());
    PARABOLIC_RAMP_ASSERT(x0.size() == vmax.size());
    for(size_t i=0; i<x0.size(); i++) {
        PARABOLIC_RAMP_ASSERT(x0[i] >= xmin[i]-EpsilonX && x0[i] <= xmax[i]+EpsilonX);
        PARABOLIC_RAMP_ASSERT(x1[i] >= xmin[i]-EpsilonX && x1[i] <= xmax[i]+EpsilonX);
        PARABOLIC_RAMP_ASSERT(Abs(v0[i]) <= vmax[i]+EpsilonV);
        PARABOLIC_RAMP_ASSERT(Abs(v1[i]) <= vmax[i]+EpsilonV);
    }
    ramps.resize(x0.size());
    for(size_t i=0; i<ramps.size(); i++) {
        if(vmax[i]==0) {
            ramps[i].resize(1);
            ramps[i][0].x0=x0[i];
            ramps[i][0].x1=x1[i];
            ramps[i][0].dx0=v0[i];
            ramps[i][0].dx1=v1[i];
            ramps[i][0].ttotal = endTime;
            continue;
        }
        //now solve minimum acceleration within bounds
        if( multidofinterp == 2 ) {
            if(!SolveMaxAccel(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],ramps[i])) {
                PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d, endTime=%f\n",i, endTime);
                return false;
            }
        }
        else if( multidofinterp == 1 ) {
            if(!SolveMaxAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],ramps[i])) {
                PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d\n",i);
                return false;
            }
        }
        else {
            if(!SolveMinAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,vmax[i],xmin[i],xmax[i],ramps[i])) {
                PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d\n",i);
                return false;
            }
            // check acceleration limits
            for(size_t j = 0; j < ramps[i].size(); ++j) {
                if( Abs(ramps[i][j].a1) > amax[i]+EpsilonA ) {
                    PARABOLIC_RAMP_PLOG("min accel for joint %d is %.15e (> %.15e)\n",i, Abs(ramps[i][j].a1), amax[i]);
                    return false;
                }
                if( Abs(ramps[i][j].a2) > amax[i]+EpsilonA ) {
                    PARABOLIC_RAMP_PLOG("min accel for joint %d is %.15e (> %.15e)\n",i, Abs(ramps[i][j].a2), amax[i]);
                    return false;
                }
            }
        }
    }

    if( multidofinterp == 2 && ramps.size() > 0 ) {
        // find the ramp that saturates the slowest and convert all switching points to it

        // because we're not computing with limits, there should only be one ramp
        for(size_t i=0; i < ramps.size(); ++i) {
            if(ramps[i].size()!=1) {
                PARABOLICWARN("expected ramp size of 1\n");
                return false;
            }
        }

        size_t islowest = 0;
        for(size_t i=1; i < ramps.size(); ++i) {
            if( ramps[islowest][0].tswitch1 < ramps[i][0].tswitch1 ) {
                islowest = i;
            }
        }

        for(size_t i=0; i < ramps.size(); ++i) {
            if( i != islowest ) {
                ParabolicRamp1D newramp = ramps[i][0];
                newramp.tswitch1 = ramps[islowest][0].tswitch1;
                newramp.tswitch2 = ramps[islowest][0].tswitch2;
                newramp.ttotal = ramps[islowest][0].ttotal;
                if( newramp.SolveFixedSwitchTime(amax[i],vmax[i]) ) {
                    ramps[i][0] = newramp;
                }
                else {
                    PARABOLIC_RAMP_PERROR("failed to set correct ramp switch times %e, %e, %e for index %d!\n",newramp.tswitch1,newramp.tswitch2,newramp.ttotal,i);
                    return false;
                }
            }
        }
    }
    return true;
}

} //namespace ParabolicRamp
