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
#include "pramp.h"
#include "ppramp.h"
#include "plpramp.h"

#include <iostream>
using namespace std;

namespace ParabolicRampInternal {

extern bool gSuppressSavingRamps;
extern bool gMinAccelQuiet;
extern bool gMinTime2Quiet;

void TestRamps(const char* fn)
{
    FILE* f=fopen(fn,"rb");
    if(!f) {
        return;
    }
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
    a1 = Inf;
    if(pres && FuzzyEquals(endTime,p.ttotal,EpsilonT) && p.GetMaxSpeed() <= vmax+EpsilonV) {
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
        a2 = -a1;
    }
    if(ppres && pp.GetMaxSpeed() <= vmax+EpsilonV && Abs(pp._a1) < Abs(a1) && Abs(pp._a2) < Abs(a1)) {
        a1 = pp._a1;
        a2 = pp._a2;
        v = 0;
        tswitch1 = tswitch2 = pp.tswitch;
        ttotal = pp.ttotal;
    }
    if(plpres && Abs(plp.v) <= vmax+EpsilonV && Abs(plp.a) < Abs(a1)) {
        a1 = plp.a;
        a2 = -a1;
        v = plp.v;
        tswitch1 = plp.tswitch1;
        tswitch2 = plp.tswitch2;
        ttotal = plp.ttotal;
    }

    if(IsInf(a1)) {
        if(vmax == 0) {
            if(FuzzyEquals(x0,x1,EpsilonX) && FuzzyEquals(dx0,dx1,EpsilonV)) {
                a1 = a2 = v = 0;
                tswitch1 = 0;
                tswitch2 = ttotal = endTime;
                return true;
            }
        }
        if(ppres && pp.GetMaxSpeed() <= vmax + EpsilonV) {
            //some slight numerical error caused velocity to exceed maximum
            a1 = pp._a1;
            a2 = pp._a2;
            v = 0;
            tswitch1 = tswitch2 = pp.tswitch;
            ttotal = pp.ttotal;
            // std::cout << "running IsValid in SolveMinAccel1D" << std::endl;
            if(IsValid()) return true;
        }
        a1 = a2 = v = 0;
        tswitch1 = tswitch2 = ttotal = -1;
        PARABOLIC_RAMP_PLOG("No ramp equation could solve for min-accel!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("end time %.15e, vmax = %.15e\n",endTime,vmax);

        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        PARABOLIC_RAMP_PLOG("p.a = %.15e, maxspeed=%.15e, end x=%.15e, end dx=%.15e\n",p.a,p.GetMaxSpeed(),p.Evaluate(endTime),p.Derivative(endTime));
        PARABOLIC_RAMP_PLOG("pp._a1 = %.15e, pp._a2 = %.15e, maxspeed=%.15e\n",pp._a1,pp._a2,pp.GetMaxSpeed());
        PARABOLIC_RAMP_PLOG("plp.a = %.15e, v=%.15e\n",plp.a,plp.v);

        Real switch1,switch2;
        Real apn = pp.CalcMinAccel(endTime,1.0,switch1);
        Real anp = pp.CalcMinAccel(endTime,-1.0,switch2);
        PARABOLIC_RAMP_PLOG("PP Calcuations: +: %.15e %.15e, -: %.15e %.15e\n",apn,switch1,anp,switch2);
        SaveRamp("Ramp_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
        return false;
    }
    PARABOLIC_RAMP_ASSERT(ttotal==endTime);
    // std::cout << "running IsValid in SolveMinAccel1D" << std::endl;
    if(!IsValid()) {
        PARABOLIC_RAMP_PLOG("Invalid min-accel!\n");
        PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
        PARABOLIC_RAMP_PLOG("end time %.15e, vmax = %.15e\n",endTime,vmax);

        PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
        PARABOLIC_RAMP_PLOG("p.a = %.15e, maxspeed=%.15e\n",p.a,p.GetMaxSpeed());
        PARABOLIC_RAMP_PLOG("pp._a1 = %.15e, pp._a2 = %.15e, maxspeed=%.15e\n",pp._a1,pp._a2,pp.GetMaxSpeed());
        PARABOLIC_RAMP_PLOG("plp.a = %.15e, v=%.15e\n",plp.a,plp.v);
        return false;
    }
    return true;
}

////////Puttichai
bool ParabolicRamp1D::SolveMinAccel(Real endTime, Real vmax, Real amax) {////////Puttichai
    if (endTime < ttotal) {
        RAVELOG_WARN_FORMAT("endTime = %.15f; ttotal = %.15f", endTime%ttotal);
    }
    
    // Try with interpolating a two-ramp profile first.
    Real d = x1 - x0; // displacement made by this profile
    Real A, B, C, D; // temporary variables for solving equations

    A = (dx1 - dx0) / endTime;
    B = (2*d)/endTime - (dx0 + dx1);
    Real sum1 = -amax - A;
    Real sum2 = amax - A;
    C = B/sum1;
    D = B/sum2;

    // std::cout << "Curve information (IN SOLVEMINACCEL)" << std::endl;
    // std::cout << str(boost::format("x0 = %.15e")%x0) << std::endl;
    // std::cout << str(boost::format("x1 = %.15e")%x1) << std::endl;
    // std::cout << str(boost::format("v0 = %.15e")%dx0) << std::endl;
    // std::cout << str(boost::format("v1 = %.15e")%dx1) << std::endl;
    // std::cout << str(boost::format("vm = %.15e")%vmax) << std::endl;
    // std::cout << str(boost::format("am = %.15e")%amax) << std::endl;
    // std::cout << str(boost::format("newDuration = %.15e")%endTime) << std::endl;

    // std::cout << str(boost::format("A = %.15e")%A) << std::endl;
    // std::cout << str(boost::format("B = %.15e")%B) << std::endl;
    // std::cout << str(boost::format("C = %.15e")%C) << std::endl;
    // std::cout << str(boost::format("D = %.15e")%D) << std::endl;
    // std::cout << str(boost::format("sum1 = %.15e")%sum1) << std::endl;
    // std::cout << str(boost::format("sum2 = %.15e")%sum2) << std::endl;

    // Now we need to check a number of feasible intervals of tswitch1 induced by constraints on the
    // acceleration. Instead of having a class representing an interval, we use the interval bounds
    // directly. Naming convention: iXl = lower bound of interval X, iXu = upper bound of interval X
    Real i0l = 0;
    Real i0u = endTime;
    Real i1l = -Inf;
    Real i1u = Inf;
    Real i2l = -Inf;
    Real i2u = Inf;
    Real i3l = -Inf;
    Real i3u = Inf;
    Real i4l = -Inf;
    Real i4u = Inf;

    // Intervals 1 and 2 are derived from constraints on a1 (the acceleration of the first ramp)
    if (FuzzyZero(sum1, EpsilonA)) {PARABOLICWARN("sum1 == 0"); return false;} // no idea yet what to do in this case.
    else if (sum1 > EpsilonA) i1u = C;
    else i1l = C;

    if (FuzzyZero(sum2, EpsilonA)) {PARABOLICWARN("sum1 == 0"); return false;} // no idea yet what to do in this case.
    else if (sum2 > EpsilonA) i2l = D;
    else i2u = D;

    // Find an intersection between interval 1 and interval 2
    if ((i1l > i2u) || (i1u < i2l)) {PARABOLICWARN("intervals 1 and 2 do not intersect"); return false;} // interval 1 and interval 2 do not intersect.
    else {
        i2l = Max(i1l, i2l);
        i2u = Min(i1u, i2u);
    }

    // Intervals 3 and 4 are derived from constraints on a2 (the acceleration of the last ramp)
    if (FuzzyZero(sum1, EpsilonA)) {PARABOLICWARN("sum1 == 0"); return false;}
    else if (sum1 > EpsilonA) i3l = C + endTime;
    else i3u = C + endTime;

    if (FuzzyZero(sum2, EpsilonA)) {PARABOLICWARN("sum2 == 0"); return false;}
    else if (sum2 > EpsilonA) i4u = D + endTime;
    else i4l = D + endTime;

    // Find an intersection between interval 3 and interval 4
    if ((i3l > i4u) || (i3u < i4l)) {PARABOLICWARN("intervals 3 and 4 do not intersect"); return false;} // interval 3 and interval 4 do not intersect.
    else {
        i4l = Max(i3l, i4l);
        i4u = Min(i3u, i4u);
    }

    // Find an intersection between interval 2 and interval 4
    if ((i2l > i4u) || (i2u < i4l)) {PARABOLICWARN("intervals 2 and 4 do not intersect"); return false;} // interval 2 and interval 4 do not intersect.
    else {
        i4l = Max(i2l, i4l);
        i4u = Min(i2u, i4u);
    }

    // Find an intersection between interval 0 and interval 4
    if ((i0l > i4u) || (i0u < i4l)) {PARABOLICWARN("intervals 3 and 4 do not intersect"); return false;} // interval 0 and interval 4 do not intersect.
    else {
        i4l = Max(i0l, i4l);
        i4u = Min(i0u, i4u);
    }

    // std::cout << str(boost::format("interval4 L = %.15e")%i4l) << std::endl;
    // std::cout << str(boost::format("interval4 U = %.15e")%i4u) << std::endl;

    // Now we have already obtained a range of feasible values for tswitch1. We choose a value of
    // tswitch1 by selecting the one which minimize (a1^2 + a2^2).

    // An alternative is to choose tswitch1 = 0.5(tswitch1Max + tswitch1Min), i.e., the midpoint of
    // the interval (to save computational effort). The midpoint, in my opinion, prevides a good
    // approximation of the real minimizer.
    bool res = SolveForTSwitch1(A, B, endTime, i4l, i4u);
    if ((!res) || FuzzyZero(tswitch1, EpsilonT)) {
        tswitch1 = 0.5*(i4u + i4l);
        // return false;
    }

    // std::cout << str(boost::format("t0 = %.15e")%tswitch1) << std::endl;

    // Now we already obtain a potential candidate for tswitch1. See if the velocity bound is
    // violated.
    // if (FuzzyZero(tswitch1, EpsilonT)) {
    //     PARABOLICWARN("the calculated tswitch1 is zero");
    //     RAVELOG_WARN_FORMAT("i4l = %.15e; i4u = %.15e", i4l%i4u);
    //     tswitch1 = 0.5*i4u;
    // }
    a1 = A + (B/tswitch1);
    if (FuzzyZero(endTime - tswitch1, EpsilonT)) {
        a2 = 0;
    }
    else {
        a2 = A - (B/(endTime - tswitch1));
    }
    v = dx0 + (a1*tswitch1); // peak velocity
    if (Abs(v) > vmax + EpsilonV) {
        // The two-ramp profile does not work. Need to modify it.
        Real vmaxNew = Sign(v)*vmax;
        Real dv, t1Trimmed, t2Trimmed, temp; // temporary variables to help in calculation

        // Here we are just reusing the variable A, B, C, and D.
        Real A2, B2, C2, D2;
        dv = v - vmaxNew;
        D2 = 0.5*dv*dv*((1/a1) - (1/a2));

        dv = vmaxNew - dx0;
        A2 = dv*dv;
        t1Trimmed = dv/a1;

        dv = dx1 - vmaxNew;
        B2 = -dv*dv;
        t2Trimmed = dv/a2;

        C2 = t1Trimmed*(vmaxNew - dx0) + t2Trimmed*(vmaxNew - dx1) - 2*D2;
        temp = A2*B2*B2;


        // RAVELOG_DEBUG_FORMAT("A2 = %.15e; B2 = %.15e; C2 = %.15e; D2 = %.15e", A2%B2%C2%D2);
        // RAVELOG_DEBUG_FORMAT("temp = %.15e", temp);
        // We need to find a cube root of A*B*B. Currently we are using FindPolyRoots functions
        // taken from ikfast generator. A direct cube-root finding method might be better?
        Real root = -Inf;
        if (FuzzyEquals(temp, 0, EpsilonT*EpsilonT*EpsilonT)) {
            root = 0;
        }
        else {
            Real rawRoots[3];

            int numRoots;
            bool modifiedTemp = false;
            Real rootDivisor = 1.0;
            int counter = 0;
            while (Abs(temp) < 1) {
                modifiedTemp = true;
                temp *= 1000;
                counter += 1;
                // RAVELOG_DEBUG_FORMAT("temp = %.15e", temp);
            }
            // if (Abs(temp) < 1e-2) {
            //     // FindPolyRoots3 has problems when the magnitude of A*B*B is too small
            //     temp = temp*1000;
            //     modifiedTemp = true;
            // }
            Real coeffs[4] = {1, 0, 0, -temp};
            FindPolyRoots3(coeffs, rawRoots, numRoots);
            if (numRoots == 0) {
                RAVELOG_WARN_FORMAT("FindPolyRoots3 failed (numRoots = 0): ABB = %.15f, root1 = %.15f, root2 = %.15f, root3 = %.15f", temp%rawRoots[0]%rawRoots[1]%rawRoots[2]);
                return false; // maybe we should try a bit harder here if this really fails.
            }
            // if (modifiedTemp) temp = temp*1000;
            // RAVELOG_WARN_FORMAT("FindPolyRoots3: numRoots = %d", numRoots);
            if (modifiedTemp) {
                for (int k = 0; k < counter; ++k) {
                    rootDivisor *= 10;
                }
            }
            for (int k = 0; k < 3; ++k) {
                // Note the epsilon that we use here. Sometimes the magnitude of temp is huge.
                if (FuzzyEquals(rawRoots[k]*rawRoots[k]*rawRoots[k], temp, temp*EpsilonT)) {
                    if (modifiedTemp) root = rawRoots[k]/rootDivisor;
                    else root = rawRoots[k];
                }
            }
            if (root == -Inf) {
                RAVELOG_WARN_FORMAT("FindPolyRoots3 failed: ABB = %.15f, root1 = %.15f, root2 = %.15f, root3 = %.15f", temp%rawRoots[0]%rawRoots[1]%rawRoots[2]);
                return false;
            } // solving cube root failed, somehow.
        }
        // RAVELOG_DEBUG_FORMAT("root = %.15e", root);
        
        a1 = (A2 + root)/C2;
        if (Abs(a1) > amax + EpsilonA) {
            // a1 exceeds the bound, fix it by making it staying at the bound.
            a1 = Sign(a1)*amax;
        }

        if (Abs(a1) < EpsilonA) {
            a2 = B2/C2;
            if (Abs(a2) > amax + EpsilonA) {
                // Cannot fix this case
                PARABOLICWARN("abs(a2) > amax");
                return false;
            }
        }
        else {
            if (FuzzyZero(C2*a1 - A2, EpsilonA)) {
                // RAVELOG_WARN_FORMAT("C*a1 - A = 0: x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15e; endTime=%.15f; vm=%.15f; am=%.15f", x0%x1%dx0%dx1%endTime%vmax%amax);
                a2 = 0;
            }
            else {
                a2 = (B2/C2)*(1 + A2/(C2*a1 - A2));
                if (Abs(a2) > amax + EpsilonA) {
                    // a2 exceeds the bound, fix it by making it staying at the bound.
                    a2 = Sign(a2)*amax;
                    // Resolve the value of a1
                    if (FuzzyZero(C2*a2 - B2, EpsilonA)) {
                        RAVELOG_WARN_FORMAT("C2*a2 - B2 = 0: x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15e; endTime=%.15f; vm=%.15f; am=%.15f", x0%x1%dx0%dx1%endTime%vmax%amax);
                    }
                    a1 = (A2/C2)*(1 + B2/(C2*a2 - B2));
                }
            }
        }
        RAVELOG_DEBUG_FORMAT("a1 = %.15f; a2 = %.15f", a1%a2);

        if ((Abs(a1) > amax + EpsilonA) || (Abs(a2) > amax + EpsilonA)) {
            PARABOLICWARN("Cannot fix acceleration bounds violation");
            return false;
        }

        if ((Abs(a1) < EpsilonA) && (Abs(a2) < EpsilonA)) {
            PARABOLICWARN("Both accelerations are zero. Shuold we allow this?");
            RAVELOG_WARN_FORMAT("A2 = %.15f; B2 = %.15f; C2 = %.15f; D2 = %.15f", A2%B2%C2%D2);
            return false;
        }

        if (Abs(a1) < EpsilonA) {
            a1 = 0;
            tswitch1 = 0;
            tswitch2 = endTime - (dx1 - vmaxNew)/a2;
            v = vmaxNew;
            ttotal = endTime;
        }
        else if (Abs(a2) < EpsilonA) {
            a2 = 0;
            tswitch1 = (vmaxNew - dx0)/a1;
            tswitch2 = endTime;
            v = vmaxNew;
            ttotal = endTime;
        }
        else {
            // No problem with those new accelerations
            tswitch1 = (vmaxNew - dx0)/a1;
            v = dx0 + a1*tswitch1;

            // (dx1 - vmaxNew)/a2 is actually the duration of the last ramp.
            Real tLastRamp = (dx1 - vmaxNew)/a2;
            if (tswitch1 + tLastRamp > endTime) {
                // Final fix.
                // RAVELOG_DEBUG_FORMAT("A = %.15f; B = %.15f; tswitch1 = %.15f; tswitch2 = %.15f", A%B%tswitch1%tswitch2);
                tswitch1 = (vmaxNew - dx0 - B)/A;
                tswitch2 = endTime - tswitch1;
                PARABOLIC_RAMP_ASSERT(tswitch2 > 0);

                a1 = A + (B/tswitch1);
                a2 = A - (B/(endTime - tswitch1));
                v = dx0 + a1*tswitch1;
            }
            else {
                tswitch2 = endTime - tLastRamp;
            }
            ttotal = endTime;
        }
        // RAVELOG_DEBUG_FORMAT("a1 = %.15f; a2 = %.15f; A2 = %.15f; B2 = %.15f; C2 = %.15f; D2 = %.15f", a1%a2%A2%B2%C2%D2);
        // PARABOLIC_RAMP_ASSERT(IsValid());
        if (IsValid()) {
            return true;
        }
        else {
            std::cout << str(boost::format("tswitch1 = %.15e")%tswitch1) << std::endl;
            std::cout << str(boost::format("tswitch2 = %.15e")%tswitch2) << std::endl;
            std::cout << str(boost::format("endTime = %.15e")%endTime) << std::endl;
            std::cout << str(boost::format("ttotal = %.15e")%ttotal) << std::endl;
            std::cout << str(boost::format("a1 = %.15e")%a1) << std::endl;
            std::cout << str(boost::format("a2 = %.15e")%a2) << std::endl;
            std::cout << str(boost::format("v = %.15e")%v) << std::endl;
                    
            return false;
        }
    }
    else {
        // RAVELOG_DEBUG_FORMAT("tswitch1 = %.15e; tswitch2 = %.15e; endTime = %.15e; ttotal = %.15e; a1 = %.15e; a2 = %.15e; v = %.15e", tswitch1%tswitch2%endTime%ttotal%a1%a1%v);
        
        // This two-ramp profile works. Go for it.
        tswitch2 = tswitch1;
        ttotal = endTime;
        // PARABOLIC_RAMP_ASSERT(IsValid());
        if (IsValid()) {
            return true;
        }
        else {
            std::cout << str(boost::format("tswitch1 = %.15e")%tswitch1) << std::endl;
            std::cout << str(boost::format("tswitch2 = %.15e")%tswitch2) << std::endl;
            std::cout << str(boost::format("endTime = %.15e")%endTime) << std::endl;
            std::cout << str(boost::format("ttotal = %.15e")%ttotal) << std::endl;
            std::cout << str(boost::format("a1 = %.15e")%a1) << std::endl;
            std::cout << str(boost::format("a2 = %.15e")%a2) << std::endl;
            std::cout << str(boost::format("v = %.15e")%v) << std::endl;
            
            return false;
        }
    }
}

bool ParabolicRamp1D::SolveForTSwitch1(Real A, Real B, Real endTime, Real l, Real u) {
    Real rawRoots[4];
    int numRoots;
    double tSqr = endTime*endTime;
    double tCube = tSqr*endTime;
    Real coeffs[5] = {2*A, -4*A*endTime + 2*B, 3*A*tSqr - 3*B*endTime, -A*tCube + 3*B*tSqr, -B*tCube};
    // std::cout << "    " << coeffs[0] << std::endl;
    // std::cout << "    " << coeffs[1] << std::endl;
    // std::cout << "    " << coeffs[2] << std::endl;
    // std::cout << "    " << coeffs[3] << std::endl;
    // std::cout << "    " << coeffs[4] << std::endl;
    FindPolyRoots4(coeffs, rawRoots, numRoots);

    // std::cout << numRoots << std::endl;
    // for (int i = 0; i < numRoots; ++i) {
    //     std::cout << "    Root " << i << " = " << rawRoots[i] << std::endl;
    // }


    if (numRoots == 0) return false;

    for (int i = 0; i < numRoots; ++i) {
        if ((rawRoots[i] <= u) && (rawRoots[i] >= l)) {
            tswitch1 = rawRoots[i];
            return true;
        }
    }
    return false;
}


bool ParabolicRamp1D::SolveMinTime(Real amax,Real vmax)
{
    if( Abs(amax) < 1e-7 ) {
        PARABOLICWARN("amax is really small %.15e", amax);
    }

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
    if(!IsInf(vmax)) {
        plpres = plp.SolveMinTime(amax,vmax);
    }
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
        a2 = -a1;
    }
    if(ppres && pp.GetMaxSpeed() <= vmax+EpsilonV && pp.ttotal < ttotal) {
        a1 = pp._a1;
        a2 = pp._a2;
        v = 0;
        tswitch1 = tswitch2 = pp.tswitch;
        ttotal = pp.ttotal;
    }
    if(plpres && plp.ttotal < ttotal) {
        a1 = plp.a;
        a2 = -a1;
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
        if(pres) {
            PARABOLIC_RAMP_PLOG("  P a=%.15e, ttotal=%.15e\n",p.a,p.ttotal);
        }
        if(ppres) {
            PARABOLIC_RAMP_PLOG("  PP a1=%.15e, a2=%.15e, tswitch=%.15e, ttotal=%.15e\n",pp._a1,pp._a2,pp.tswitch,pp.ttotal);
        }
        if(plpres) {
            PARABOLIC_RAMP_PLOG("  PLP a=%.15e, tswitch=%.15e, %.15e, ttotal=%.15e\n",plp.a,plp.tswitch1,plp.tswitch2,plp.ttotal);
        }
        SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,-1);
        a1 = a2 = v = 0;
        tswitch1 = tswitch2 = ttotal = -1;
        return false;
    }
    if( tswitch1 < 0 && tswitch1 >= -EpsilonT ) {
        tswitch1 = 0;
    }
    if( tswitch2 < 0 && tswitch2 >= -EpsilonT ) {
        tswitch2 = 0;
    }
    //cout<<"switch time 1: "<<tswitch1<<", 2: "<<tswitch2<<", total "<<ttotal<<endl;
    // std::cout << "running IsValid in SolveMinTime" << std::endl;
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
    if(!IsInf(vmax)) {
        plpres = plp.SolveMinTime2(amax,vmax,tLowerBound);
    }
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

        a2 = -a1;
    }
    if(ppres && pp.GetMaxSpeed() <= vmax+EpsilonV && pp.ttotal < ttotal) {
        a1 = pp._a1;
        a2 = pp._a2;
        v = 0;
        tswitch1 = tswitch2 = pp.tswitch;
        ttotal = pp.ttotal;
    }
    if(plpres && plp.ttotal < ttotal) {
        a1 = plp.a;
        a2 = -a1;
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
        if(pres) {
            PARABOLIC_RAMP_PLOG("  P a=%.15e, ttotal=%.15e\n",p.a,p.ttotal);
        }
        if(ppres) {
            PARABOLIC_RAMP_PLOG("  PP a1=%.15e, a1=%.15e, tswitch=%.15e, ttotal=%.15e\n",pp._a1,pp._a2,pp.tswitch,pp.ttotal);
        }
        if(plpres) {
            PARABOLIC_RAMP_PLOG("  PLP a=%.15e, tswitch=%.15e, %.15e, ttotal=%.15e\n",plp.a,plp.tswitch1,plp.tswitch2,plp.ttotal);
        }
        ppres = pp.SolveMinTime(amax);
        plpres = plp.SolveMinTime(amax,vmax);
        PARABOLIC_RAMP_PLOG("unconstrained PP (%d): %.15e, PLP (%d): %.15e\n",(int)ppres,pp.ttotal,(int)plpres,plp.ttotal);
        SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,tLowerBound);
        a1 = a2 = v = 0;
        tswitch1 = tswitch2 = ttotal = -1;
        return false;
    }
    //cout<<"switch time 1: "<<tswitch1<<", 2: "<<tswitch2<<", total "<<ttotal<<endl;
    // std::cout << "running IsValid in SolveMinTime2" << std::endl;
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
    // std::cout << "running IsValid in SolveBraking1D" << std::endl;
    PARABOLIC_RAMP_ASSERT(IsValid());
}

/** only ttotal is known

   \author Rosen Diankov
 */
bool ParabolicRamp1D::SolveFixedTime(Real amax,Real vmax,Real endTime)
{
    bool res = SolveMinAccel(endTime, vmax, amax);
    // std::cout << "===== SolveFixedTime =====" << std::endl;
    // std::cout << str(boost::format("x0 = %.15e")%x0) << std::endl;
    // std::cout << str(boost::format("v0 = %.15e")%dx0) << std::endl;
    // std::cout << str(boost::format("x1 = %.15e")%x1) << std::endl;
    // std::cout << str(boost::format("v1 = %.15e")%dx1) << std::endl;
    // std::cout << str(boost::format("tswitch1 = %.15e")%tswitch1) << std::endl;
    // std::cout << str(boost::format("tswitch2 = %.15e")%tswitch2) << std::endl;
    // std::cout << str(boost::format("ttotal = %.15e")%ttotal) << std::endl;
    // std::cout << str(boost::format("a1 = %.15e")%a1) << std::endl;
    // std::cout << str(boost::format("v = %.15e")%v) << std::endl;
    // std::cout << str(boost::format("a2 = %.15e")%a2) << std::endl;
    if (IsValid()) {
        return true;
    }
    else {
        return false;
    }
    // ParabolicRamp p;
    // PPRamp pp;
    // PLPRamp plp;
    // p.x0 = pp.x0 = plp.x0 = x0;
    // p.x1 = pp.x1 = plp.x1 = x1;
    // p.dx0 = pp.dx0 = plp.dx0 = dx0;
    // p.dx1 = pp.dx1 = plp.dx1 = dx1;
    // bool pres = p.SolveFixedTime(endTime);
    // bool ppres = pp.SolveFixedTime(amax,endTime);
    // bool plpres = false;
    // if(!IsInf(vmax)) {
    //     plpres = plp.SolveFixedTime(amax,vmax,endTime);
    // }
    // ttotal = Inf;
    // if(pres && Abs(p.a) <= amax+EpsilonA && FuzzyEquals(p.ttotal,endTime,EpsilonT) ) {
    //     if(Abs(p.a) <= amax) {
    //         a1 = p.a;
    //         if( Abs(a1) < EpsilonA ) {
    //             // not accelerating
    //             v = dx0;
    //             tswitch1 = 0;
    //         }
    //         else {
    //             v = 0;
    //             tswitch1 = p.ttotal;
    //         }
    //         tswitch2 = p.ttotal;
    //         ttotal = p.ttotal;
    //     }
    //     else {
    //         //double check
    //         p.a = Sign(p.a)*amax;
    //         if(FuzzyEquals(p.Evaluate(p.ttotal),x1,EpsilonX) && FuzzyEquals(p.Derivative(p.ttotal),dx1,EpsilonV)) {
    //             a1 = p.a;
    //             if( Abs(a1) < EpsilonA ) {
    //                 // not accelerating
    //                 v = dx0;
    //                 tswitch1 = 0;
    //             }
    //             else {
    //                 v = 0;
    //                 tswitch1 = p.ttotal;
    //             }
    //             tswitch2=p.ttotal;
    //             ttotal = p.ttotal;
    //         }
    //     }
    //     a2 = -a1;
    // }
    // if(ppres && pp.GetMaxSpeed() <= vmax+EpsilonV && FuzzyEquals(pp.ttotal,endTime,EpsilonT) ) {
    //     a1 = pp._a1;
    //     a2 = pp._a2;
    //     v = 0;
    //     tswitch1 = tswitch2 = pp.tswitch;
    //     ttotal = pp.ttotal;
    // }
    // if(plpres && FuzzyEquals(plp.ttotal, endTime, EpsilonT) ) {
    //     a1 = plp.a;
    //     a2 = -a1;
    //     v = plp.v;
    //     tswitch1 = plp.tswitch1;
    //     tswitch2 = plp.tswitch2;
    //     ttotal = plp.ttotal;
    // }
    // if(IsInf(ttotal)) {

    //     // finally remove the a2=-a1 constraint
    //     Real dx = x1-x0;
    //     for(tswitch1 = 0; tswitch1 < endTime; tswitch1 += 0.2*endTime) {
    //         for(tswitch2 = tswitch1; tswitch2 < endTime; tswitch2 += 0.2*endTime) {
    //             Real c1 = -0.5*Sqr(tswitch1)*tswitch2 + 0.5*Sqr(tswitch1)*endTime + 0.5*tswitch1*Sqr(tswitch2) - 0.5*tswitch1*Sqr(endTime);
    //             Real c0 = -1.0*dx*tswitch2 + 1.0*dx*endTime + 0.5*Sqr(tswitch2)*dx0 - 0.5*Sqr(tswitch2)*dx1 + 1.0*tswitch2*endTime*dx1 - 0.5*Sqr(endTime)*dx0 - 0.5*Sqr(endTime)*dx1;
    //             if( Abs(c1) > 1e-8 ) {
    //                 a1 = -c0/c1;
    //                 if( a1 >= -amax && a1 <= amax ) {
    //                     v = dx0 + a1*tswitch1;
    //                     if( v >= -vmax && v <= vmax ) {
    //                         a2 = (a1*tswitch1 + dx0 - dx1)/(tswitch2 - endTime);
    //                         if( a2 >= -amax && a2 <= amax ) {
    //                             // std::cout << "running IsValid in SolveFixedTime" << std::endl;
    //                             if(IsValid()) {
    //                                 return true;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }

    //     PARABOLIC_RAMP_PLOG("No ramp equation could solve for min-time (2)!\n");
    //     PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
    //     PARABOLIC_RAMP_PLOG("vmax = %.15e, amax = %.15e, tmax = %.15e\n",vmax,amax,endTime);
    //     PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
    //     if(pres) {
    //         PARABOLIC_RAMP_PLOG("  P a=%.15e, ttotal=%.15e\n",p.a,p.ttotal);
    //     }
    //     if(ppres) {
    //         PARABOLIC_RAMP_PLOG("  PP a1=%.15e, a2=%.15e, tswitch=%.15e, ttotal=%.15e\n",pp._a1, pp._a2, pp.tswitch,pp.ttotal);
    //     }
    //     if(plpres) {
    //         PARABOLIC_RAMP_PLOG("  PLP a=%.15e, tswitch=%.15e, %.15e, ttotal=%.15e\n",plp.a,plp.tswitch1,plp.tswitch2,plp.ttotal);
    //     }
    //     ppres = pp.SolveMinTime(amax);
    //     plpres = plp.SolveMinTime(amax,vmax);
    //     PARABOLIC_RAMP_PLOG("unconstrained PP (%d): %.15e, PLP (%d): %.15e\n",(int)ppres,pp.ttotal,(int)plpres,plp.ttotal);
    //     SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,endTime);
    //     a1 = a2 = v = 0;
    //     tswitch1 = tswitch2 = ttotal = -1;
    //     return false;
    // }
    // if( tswitch1 < 0 && tswitch1 >= -EpsilonT ) {
    //     tswitch1 = 0;
    // }
    // if( tswitch2 < 0 && tswitch2 >= -EpsilonT ) {
    //     tswitch2 = 0;
    // }
    // //cout<<"switch time 1: "<<tswitch1<<", 2: "<<tswitch2<<", total "<<ttotal<<endl;
    // // std::cout << "running IsValid in SolveFixedTime" << std::endl;
    // if(!IsValid()) {
    //     PARABOLIC_RAMP_PLOG("ParabolicRamp1D::SolveMinTime: Failure to find valid path!\n");
    //     PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, dx0=%.15e, dx1=%.15e\n",x0,x1,dx0,dx1);
    //     PARABOLIC_RAMP_PLOG("vmax = %.15e, amax = %.15e\n",vmax,amax);
    //     PARABOLIC_RAMP_PLOG("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
    //     SaveRamp("Ramp_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,endTime);
    // }
    // PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal, endTime, EpsilonT));
    // return true;
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
    // std::cout << "running IsValid in SolveFixedSwitchTime" << std::endl;
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

    // std::cout << "running IsValid in SolveFixedAccelSwitchTime" << std::endl;
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
    if (!IsValid()) {
        RAVELOG_WARN_FORMAT("Before trimming: x0=%.15e; x1=%.15e; v0=%.15e; v1=%.15e; a1=%.15e; a2=%.15e; v=%.15e; tswitch1=%.15e; tswitch2=%.15e", x0%x1%dx0%dx1%a1%a1%v%tswitch1%tswitch2);
    }
    x0 = Evaluate(tcut);
    dx0 = Derivative(tcut);
    ttotal -= tcut;
    tswitch1 -= tcut;
    tswitch2 -= tcut;
    if(tswitch1 < 0) tswitch1=0;
    if(tswitch2 < 0) tswitch2=0;
    // std::cout << "running IsValid in TrimFront" << std::endl;
    if (!IsValid()) {
        RAVELOG_WARN_FORMAT("x0=%.15e; x1=%.15e; v0=%.15e; v1=%.15e; a1=%.15e; a2=%.15e; v=%.15e; tswitch1=%.15e; tswitch2=%.15e", x0%x1%dx0%dx1%a1%a1%v%tswitch1%tswitch2);
    }
    PARABOLIC_RAMP_ASSERT(IsValid());
    // PARABOLIC_RAMP_PLOG("x0=%.15e; x1=%.15e; v0=%.15e; v1=%.15e; a1=%.15e; a2=%.15e; v=%.15e; tswitch1=%.15e; tswitch2=%.15e", x0, x1, dx0, dx1, a1, a1, v, tswitch1, tswitch2);

}

void ParabolicRamp1D::TrimBack(Real tcut)
{
    //PARABOLIC_RAMP_ASSERT(IsValid());
    x1 = Evaluate(ttotal-tcut);
    dx1 = Derivative(ttotal-tcut);
    ttotal -= tcut;
    tswitch1 = Min(tswitch1,ttotal);
    tswitch2 = Min(tswitch2,ttotal);
    // std::cout << "running IsValid in TrimBack" << std::endl;
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
    // PARABOLIC_RAMP_PLOG("current xmin = %.15e, xmax = %.15e", xmin, xmax);

    Real tflip1=0,tflip2=0;
    if(ta < tswitch1) {
        //x' = a1*t + v0 = 0 => t = -v0/a1
        tflip1 = -dx0/a1;
        // PARABOLIC_RAMP_PLOG("a1 = %.15e, tswitch1 = %.15e, tflip1 = %.15e", a1, tswitch1, tflip1);
        if(tflip1 > tswitch1) tflip1 = 0;
    }
    if(tb > tswitch2) {
        //x' = a2*(T-t) + v1 = 0 => (T-t) = v1/a2
        tflip2 = ttotal-dx1/a2;
        // PARABOLIC_RAMP_PLOG("a2 = %.15e, tswitch2 = %.15e, tflip2 = %.15e", a2, tswitch2, tflip2);
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

    // PARABOLIC_RAMP_PLOG("tflip1=%.15e, tflip2=%.15e", tflip1, tflip2);
    // PARABOLIC_RAMP_PLOG("x0=%.15e, x1=%.15e, v0=%.15e, v1=%.15e, tswitch1=%.15e, tswitch2=%.15e, ttotal=%.15e, a1=%.15e, a2=%.15e, v=%.15e\n",x0, x1, dx0, dx1, tswitch1, tswitch2, ttotal, a1, a2, v);

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
        PARABOLICWARN("Ramp %.15e,%.15e -> %.15e,%.15e\n",x0,dx0,x1,dx1);
        PARABOLICWARN("Acceleration %.15e, vel %.15e, deceleration %.15e\n",a1,v,a2);
        PARABOLICWARN("Switch times %.15e %.15e %.15e\n",tswitch1,tswitch2,ttotal);
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
            // std::cout << "running IsValid in SolveMinTimeLinear" << std::endl;
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
        if(!ramps[i].SolveMinTime(amax[i],vmax[i])) {
            return false;
        }
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
                // if(!ramps[i].SolveMinAccel(endTime,vmax[i],amax[i])) {////////Puttichai
                PARABOLIC_RAMP_PLOG("Failed solving min accel for joint %d\n",i);
                ramps[i].SolveMinTime(amax[i],vmax[i]);
                PARABOLIC_RAMP_PLOG("its min time is %.15e\n",ramps[i].ttotal);
                if(ramps[i].tswitch1==ramps[i].tswitch2) {
                    PARABOLIC_RAMP_PLOG("its type is PP\n");
                } else if(Abs(ramps[i].v)==vmax[i]) {
                    PARABOLIC_RAMP_PLOG("its type is PLP (vmax)\n");
                } else {
                    PARABOLIC_RAMP_PLOG("its type is PLP (v=%.15e %%)\n",ramps[i].v/vmax[i]);
                }
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
        // std::cout << "running IsValid in SolveMinAccelLinear" << std::endl;
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
    // std::cout << "running IsValid in SolveBraking" << std::endl;
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
    if( tcut > endTime ) {
        PARABOLICWARN("Warning, cut time (%.15e) needs to be <= to total time (%.15e)\n",tcut, endTime);
        PARABOLIC_RAMP_ASSERT(tcut <= endTime);
    }
    Evaluate(tcut,x0);
    Derivative(tcut,dx0);
    endTime -= tcut;
    for(size_t i=0; i<ramps.size(); i++)
        ramps[i].TrimFront(tcut);
    // std::cout << "running IsValid in TrimFront1D" << std::endl;
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
    // std::cout << "running IsValid in TrimBack1D" << std::endl;
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
    if(!ramp.SolveMinTime(amax,vmax)) {
        return false;
    }
    Real bmin,bmax;
    ramp.Bounds(bmin,bmax);
    if(bmin < xmin || bmax > xmax) {
        return false;
    }
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
    // there's a degenerate case here when both 2.0*(xbound-x) and v*v are close to 0
    Real coeff0 = 2.0*(xbound-x);
    Real coeff1 = v*v;
//    if( Abs(coeff0) <= EpsilonV && Abs(coeff1) <= EpsilonV ) {
//        return 0;
//    }
    Real a;
    bool res=SafeEqSolve(coeff0,-coeff1,EpsilonV,-Inf,Inf,a);
    if(!res) {
        PARABOLICWARN("Warning, couldn't solve braking acceleration equation:\n");
        PARABOLICWARN("%.15e*a + %.15e = 0\n", coeff0, coeff1);
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

////////Puttichai
bool SolveMinAccelBounded(Real x0,Real v0,Real x1,Real v1,Real endTime, Real vmax, Real amax, Real xmin,Real xmax,std::vector<ParabolicRamp1D>& ramps)
{
    PARABOLIC_RAMP_ASSERT(x0 >= xmin-EpsilonX && x0 <= xmax+EpsilonX && x1 >= xmin-EpsilonX && x1 <= xmax+EpsilonX);
    ParabolicRamp1D ramp;
    ramp.x0 = x0;
    ramp.dx0 = v0;
    ramp.x1 = x1;
    ramp.dx1 = v1;
    Real prevDuration = ramp.ttotal;
    if(!ramp.SolveMinAccel(endTime,vmax,amax)) {////////Puttichai
        RAVELOG_WARN_FORMAT("SolveMinAccel failed: x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15f; newDuration=%.15f; vm=%.15f; am=%.15f, prevDuration=%.15f", x0%x1%v0%v1%endTime%vmax%amax%prevDuration);
        return false;
    }
    Real bmin,bmax;
    ramp.Bounds(bmin,bmax);
    if(bmin >= xmin-EpsilonX && bmax <= xmax+EpsilonX) {
        // std::cout << "X BOUNDS NOT VIOLATED:: GREAT" << std::endl;
        ramps.resize(1);
        ramps[0] = ramp;
        return true;
    }
    // RAVELOG_WARN_FORMAT("SolveMinAccel passed but x bounds violated: xmin=%.15f; bmin=%.15f; xmax=%.15f; bmax=%.15f", xmin%bmin%xmax%bmax);
    PARABOLIC_RAMP_PLOG("SolveMinAccel passed but x bounds violated: xmin=%.15f; bmin=%.15f; xmax=%.15f; bmax=%.15f", xmin, bmin, xmax, bmax);
    PARABOLIC_RAMP_PLOG("x0=%.15f; x1=%.15f; v0=%.15f; v1=%.15f; newDuration=%.15f; vm=%.15f; am=%.15f", x0, x1, v0, v1, endTime, vmax, amax);
    
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
    // Real amax=Inf;
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
            if(temp[1].SolveMinAccel(endTime-bt0,vmax,amax)) {////////Puttichai
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
            if(temp[0].SolveMinAccel(endTime-bt1,vmax,amax)) {////////Puttichai
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
            // std::cout << "running IsValid in SolveMinAccelBounded" << std::endl;
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
                if(temp[1].SolveMinAccel(endTime - bt0 - bt1,vmax,amax)) {////////Puttichai
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
    PARABOLIC_RAMP_PLOG("Successfully fixed x bounds violation");
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

    PARABOLICWARN("X exceeds the bound");

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
            // std::cout << "running IsValid in SolveMinAccelBounded (original)" << std::endl;
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
    // std::cout << "================================================================================" << std::endl;
    // std::cout << "START SOLVEMINTIMEBOUNDED (ORIGINAL)" << std::endl;
    // std::cout << "================================================================================" << std::endl;

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
    std::vector<ParabolicRamp1D> tempramps;
    int numiters = 0;
    int maxiters=10;
    bool solved = true;
    while(numiters < maxiters) {
        ++numiters;
        solved = true;
        for(size_t i=0; i<ramps.size(); i++) {
            if( ramps[i].size() == 0 ) {
                PARABOLICWARN("ramp[%d] has 0 size, numiters=%d, multidofinterp=%d", i, numiters, multidofinterp);
            }
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
                if(!SolveMaxAccel(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],tempramps)) {
                    // because SolveMaxAccel doesn't check for boundaries, this could fail
                    PARABOLIC_RAMP_PLOG("Failed solving bounded max accel for joint %d\n",i);
                    endTime *= 1.05;
                    solved = false;
                    break;
                }
                ramps[i] = tempramps;
            }
            else if( multidofinterp == 1 ) {
                if(!SolveMaxAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],tempramps)) {
                    endTime *= 1.05;
                    solved = false;
                    break;
                }
                ramps[i] = tempramps;
            }
            else {
                if(!SolveMinAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,vmax[i],xmin[i],xmax[i],tempramps)) {
                    PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d\n",i);
                    endTime *= 1.05;
                    solved = false;
                    break;
                }
                ramps[i] = tempramps;
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

////////Puttichai
Real SolveMinTimeBounded2(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1,
                          const Vector& amax,const Vector& vmax,const Vector& xmin,const Vector& xmax,
                          vector<vector<ParabolicRamp1D> >& ramps, int multidofinterp)
{
    // std::cout << "================================================================================" << std::endl;
    // std::cout << "START SOLVEMINTIMEBOUNDED2 (PUTTICHAI)" << std::endl;
    // std::cout << "================================================================================" << std::endl;

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
                if( SolveMinAccelBounded(x0[i], v0[i], x1[i], v1[i], originaltime*timemult, vmax[i], amax[i], xmin[i], xmax[i], ramps[i]) ) {////////Puttichai
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
    PARABOLIC_RAMP_PLOG("Maximum duration = %.15f", endTime);
    // std::cout << "endTime = " << endTime << std::endl;////////Puttichai

    //now we have a candidate end time -- repeat looking through solutions
    //until we have solved all ramps
    std::vector<ParabolicRamp1D> tempramps;
    int numiters = 0;
    int maxiters=10;
    bool solved = true;
    while(numiters < maxiters) {
        ++numiters;
        solved = true;
        for(size_t i=0; i<ramps.size(); i++) {
            if( ramps[i].size() == 0 ) {
                PARABOLICWARN("ramp[%d] has 0 size, numiters=%d, multidofinterp=%d", i, numiters, multidofinterp);
            }
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
            // Don't do anything if this is the slowest ramp.
            if(FuzzyEquals(ttotal,endTime,EpsilonT)) {
                continue;
            }

            //now solve minimum acceleration within bounds
            if ( multidofinterp == 2 ) {
                if(!SolveMaxAccel(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],tempramps)) {
                    // because SolveMaxAccel doesn't check for boundaries, this could fail
                    PARABOLIC_RAMP_PLOG("Failed solving bounded max accel for joint %d\n",i);
                    endTime *= 1.05;
                    solved = false;
                    break;
                }
                ramps[i] = tempramps;
            }
            else if ( multidofinterp == 1 ) {
                if(!SolveMaxAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],tempramps)) {
                    endTime *= 1.05;
                    solved = false;
                    break;
                }
                ramps[i] = tempramps;
            }
            else {
                if(!SolveMinAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,vmax[i],amax[i],xmin[i],xmax[i],tempramps)) {////////Puttichai
                    PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d with endTime = %.15f\n",i ,endTime);
                    // std::cout << "SolveMinAccelBoundedFailed" << std::endl;
                    // std::cout << "Curve information" << std::endl;
                    std::cout << str(boost::format("x0 = %.15e")%x0[i]) << std::endl;
                    std::cout << str(boost::format("x1 = %.15e")%x1[i]) << std::endl;
                    std::cout << str(boost::format("v0 = %.15e")%v0[i]) << std::endl;
                    std::cout << str(boost::format("v1 = %.15e")%v1[i]) << std::endl;
                    std::cout << str(boost::format("vm = %.15e")%vmax[i]) << std::endl;
                    std::cout << str(boost::format("am = %.15e")%amax[i]) << std::endl;
                    // std::cout << str(boost::format("newDuration = %.15e")%endTime) << std::endl;
                    // std::cout << str(boost::format("tprev = %.15e")%ttotal) << std::endl;
                    Real newEndTime;
                    bool result = CalculateLeastBoundInoperativeInterval(x0[i], v0[i], x1[i], v1[i], vmax[i], amax[i], newEndTime);

                    if (!result) {
                        PARABOLICWARN("CALCULATING NEWENDTIME FAILED\n");
                        std::cout << str(boost::format("x0 = %.15e")%x0[i]) << std::endl;
                        std::cout << str(boost::format("x1 = %.15e")%x1[i]) << std::endl;
                        std::cout << str(boost::format("v0 = %.15e")%v0[i]) << std::endl;
                        std::cout << str(boost::format("v1 = %.15e")%v1[i]) << std::endl;
                        std::cout << str(boost::format("vm = %.15e")%vmax[i]) << std::endl;
                        std::cout << str(boost::format("am = %.15e")%amax[i]) << std::endl;
                        endTime *= 1.05;
                    }
                    else {
                        PARABOLIC_RAMP_PLOG("newEndTime = %.15f\n", newEndTime);
                        endTime = newEndTime;
                    }
                    solved = false;
                    break;
                }
                PARABOLIC_RAMP_PLOG("Successfully solving bounded min accel for joint %d (endTime = %.15f)\n",i, endTime);
                ramps[i] = tempramps;
            }

            // std::cout << "IsSolved " << solved << std::endl;////////Puttichai

            //now check accel/velocity bounds
            bool inVelBounds = true;
            for(size_t j=0; j<ramps[i].size(); j++) {
                if(Abs(ramps[i][j].a1) > amax[i]+EpsilonA || Abs(ramps[i][j].a2) > amax[i]+EpsilonA || Abs(ramps[i][j].v) > vmax[i]+EpsilonV) {
                    //PARABOLIC_RAMP_PLOG("Ramp %d entry %d accels: %.15e %.15e, vel %.15e\n",i,j,ramps[i][j].a1,ramps[i][j].a2,ramps[i][j].v);
                    inVelBounds = false;
                    break;
                }
            }
            // std::cout << "    inVelBounds " << inVelBounds << std::endl;
            // std::cout << "    solved " << solved << std::endl;
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
                // if (1) {//(Abs(ramps[i][j].a1) > amax[i]+EpsilonA) {
                //     PARABOLIC_RAMP_PLOG("ramps[%d][%d].a1 = %.15f; amax[%d] = %.15f", i, j, ramps[i][j].a1, i, amax[i]);
                // }
                // PARABOLIC_RAMP_ASSERT(Abs(ramps[i][j].a2) <= amax[i]+EpsilonA);
                // if (1) {//(Abs(ramps[i][j].a2) <= amax[i]+EpsilonA) {
                //     PARABOLIC_RAMP_PLOG("ramps[%d][%d].a2 = %.15f; amax[%d] = %.15f", i, j, ramps[i][j].a2, i, amax[i]);
                // }
                PARABOLIC_RAMP_ASSERT(Abs(ramps[i][j].v) <= vmax[i]+EpsilonV);
                ttotal += ramps[i][j].ttotal;
            }
            PARABOLIC_RAMP_ASSERT(FuzzyEquals(ttotal,endTime,EpsilonT*0.1));
        }
        if( !solved ) {
            continue; //go back and re-solve
        }

        if ( multidofinterp == 2 ) {
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

////////Puttichai
bool SolveMinAccelBounded(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1,
                          Real endTime,const Vector& vmax,const Vector& amax,const Vector& xmin,const Vector& xmax,
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
        if(!SolveMinAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,vmax[i],amax[i],xmin[i],xmax[i],ramps[i])) {
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

////////Puttichai
bool SolveAccelBounded2(const Vector& x0,const Vector& v0,const Vector& x1,const Vector& v1,
                        Real& endTime,const Vector& amax, const Vector& vmax,const Vector& xmin,const Vector& xmax,
                        vector<vector<ParabolicRamp1D> >& ramps, int multidofinterp, int& counterStart, int& counterEnd)
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
                PARABOLIC_RAMP_PLOG("multidofinterp = %d\n", multidofinterp);
                return false;
            }
        }
        else if( multidofinterp == 1 ) {
            if(!SolveMaxAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,amax[i], vmax[i],xmin[i],xmax[i],ramps[i])) {
                PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d\n",i);
                PARABOLIC_RAMP_PLOG("multidofinterp = %d\n", multidofinterp);
                return false;
            }
        }
        else {
            if(!SolveMinAccelBounded(x0[i],v0[i],x1[i],v1[i],endTime,vmax[i],amax[i],xmin[i],xmax[i],ramps[i])) {
                PARABOLIC_RAMP_PLOG("Failed solving bounded min accel for joint %d\n",i);
                std::cout << str(boost::format("x0 = %.15e")%x0[i]) << std::endl;
                std::cout << str(boost::format("x1 = %.15e")%x1[i]) << std::endl;
                std::cout << str(boost::format("v0 = %.15e")%v0[i]) << std::endl;
                std::cout << str(boost::format("v1 = %.15e")%v1[i]) << std::endl;
                std::cout << str(boost::format("vm = %.15e")%vmax[i]) << std::endl;
                std::cout << str(boost::format("am = %.15e")%amax[i]) << std::endl;
                std::cout << str(boost::format("newDuration = %.15e")%endTime) << std::endl;
                // std::cout << "redoiffails = " << redoiffails << std::endl;
                std::cout << counterStart << counterEnd << std::endl;

                if (counterStart < counterEnd) {
                    counterStart++;
                    Real newEndTime;
                    bool result = CalculateLeastBoundInoperativeInterval(x0[i], v0[i], x1[i], v1[i], vmax[i], amax[i], newEndTime);
                    if (!result) return false;
                    std::cout << str(boost::format("endtime = %.15e; newendtime = %.15e")%endTime % newEndTime) << std::endl;
                    endTime = newEndTime;
                    
                    result = SolveAccelBounded2(x0, v0, x1, v1, endTime, amax, vmax, xmin, xmax, ramps, multidofinterp, counterStart, counterEnd);
                    std::cout << "Resolving SolveAccelBounded2 : " << result << std::endl;
                    return result;
                }
                else {
                    return false;
                }
            }
            PARABOLIC_RAMP_PLOG("Successfully solving bounded min accel for joint %d\n",i);
            
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

bool CalculateLeastBoundInoperativeInterval(Real x0, Real v0, Real x1, Real v1, Real vmax, Real amax, Real& newEndTime) {
    Real d = x1 - x0;
    Real T0, T1, T2, T3, temp1, temp2;

    // Computation procedure here can be made more efficient
    temp1 = 2*(-Sqr(amax))*(2*amax*d - Sqr(v0) - Sqr(v1));
    if (temp1 < 0) {
        T0 = -1;
        T1 = -1;
    }
    else {                     
        T0 = (v0 + v1)/amax + Sqrt(temp1)/Sqr(amax);
        T1 = (v0 + v1)/amax - Sqrt(temp1)/Sqr(amax);
    }
    T1 = Max(T0, T1);

    temp2 = 2*(Sqr(amax))*(2*amax*d + Sqr(v0) + Sqr(v1));
    if (temp2 < 0) {
        T2 = -1;
        T3 = -1;
    }
    else {
        T2 = -(v0 + v1)/amax + Sqrt(temp2)/Sqr(amax);
        T3 = -(v0 + v1)/amax - Sqrt(temp2)/Sqr(amax);
    }
    T3 = Max(T2, T3);

    newEndTime = Max(T1, T3);
    if (newEndTime > EpsilonT) {
        newEndTime = newEndTime * 1.01; // for safety reasons, we don't make newEndTime too close to the bound
        return true;
    }
    else {
        RAVELOG_WARN_FORMAT("Unable to calculate the least upper bound: T0=%.15f, T1 = %.15f, T2 = %.15f, T3 = %.15f", T0 %T1 %T2 %T3);
        return false;
    }
}

} //namespace ParabolicRamp
