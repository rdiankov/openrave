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

#include "DynamicPath.h"
#include "Timer.h"
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <algorithm>
using namespace std;

namespace ParabolicRampInternal {

inline Real LInfDistance(const Vector& a,const Vector& b)
{
    PARABOLIC_RAMP_ASSERT(a.size()==b.size());
    Real d=0;
    for(size_t i=0; i<a.size(); i++) {
        d = Max(d,Abs(a[i]-b[i]));
    }
    return d;
}

inline bool InBounds(const Vector& x,const Vector& bmin,const Vector& bmax)
{
    PARABOLIC_RAMP_ASSERT(x.size()==bmin.size());
    PARABOLIC_RAMP_ASSERT(x.size()==bmax.size());
    for(size_t i=0; i<x.size(); i++) {
        if(x[i] < bmin[i] || x[i] > bmax[i]) {
            return false;
        }
    }
    return true;
}

inline Real MaxBBLInfDistance(const Vector& x,const Vector& bmin,const Vector& bmax)
{
    PARABOLIC_RAMP_ASSERT(x.size()==bmin.size());
    PARABOLIC_RAMP_ASSERT(x.size()==bmax.size());
    Real d=0;
    for(size_t i=0; i<x.size(); i++) {
        d = Max(d,Max(Abs(x[i]-bmin[i]),Abs(x[i]-bmax[i])));
    }
    return d;
}

bool SolveMinTime(const Vector& x0,const Vector& dx0,const Vector& x1,const Vector& dx1, const Vector& accMax,const Vector& velMax,const Vector& xMin,const Vector& xMax,DynamicPath& out, int multidofinterp)
{
    if(xMin.empty()) {
        out.ramps.resize(1);
        ParabolicRampND& temp=out.ramps[0];
        temp.x0=x0;
        temp.x1=x1;
        temp.dx0=dx0;
        temp.dx1=dx1;
        bool res=temp.SolveMinTime(accMax,velMax);
        if(!res) {
            return false;
        }
    }
    else {
        vector<std::vector<ParabolicRamp1D> > ramps;
        ////////Puttichai

        // we can switch between the original implementation (SolveMinTImeBounded) and a new
        // implementation (SolveMinTimeBounded2) here

        //Real res=SolveMinTimeBounded(x0,dx0,x1,dx1, accMax,velMax,xMin,xMax, ramps,multidofinterp);
        Real res=SolveMinTimeBounded(x0,dx0,x1,dx1, accMax,velMax,xMin,xMax, ramps,multidofinterp);////////Puttichai
        if(res < 0) {
            return false;
        }
        out.ramps.resize(0);
        CombineRamps(ramps,out.ramps);
    }
    out.accMax = accMax;
    out.velMax = velMax;
    PARABOLIC_RAMP_ASSERT(out.IsValid());
    return true;
}

DynamicPath::DynamicPath()
{
    _multidofinterp = 0;
}

void DynamicPath::Init(const Vector& _velMax,const Vector& _accMax)
{
    velMax = _velMax;
    accMax = _accMax;
    PARABOLIC_RAMP_ASSERT(velMax.size() == accMax.size());
}

void DynamicPath::SetJointLimits(const Vector& _xMin,const Vector& _xMax)
{
    xMin = _xMin;
    xMax = _xMax;
    PARABOLIC_RAMP_ASSERT(xMin.size() == xMax.size());
}

Real DynamicPath::GetTotalTime() const
{
    Real t=0;
    for(size_t i=0; i<ramps.size(); i++) {
        t+=ramps[i].endTime;
    }
    return t;
}

int DynamicPath::GetSegment(Real t,Real& u) const
{
    if(t < 0) return -1;
    for(size_t i=0; i<ramps.size(); i++) {
        if(t <= ramps[i].endTime) {
            u = t;
            return (int)i;
        }
        t -= ramps[i].endTime;
    }
    u = t;
    return (int)ramps.size();
}

void DynamicPath::Evaluate(Real t,Vector& x) const
{
    PARABOLIC_RAMP_ASSERT(!ramps.empty());
    if(t < 0) {
        x = ramps.front().x0;
    }
    else {
        for(size_t i=0; i<ramps.size(); i++) {
            if(t <= ramps[i].endTime) {
                ramps[i].Evaluate(t,x);
                return;
            }
            t -= ramps[i].endTime;
        }
        x = ramps.back().x1;
    }
}

void DynamicPath::Derivative(Real t,Vector& dx) const
{
    PARABOLIC_RAMP_ASSERT(!ramps.empty());
    if(t < 0) {
        dx = ramps.front().dx0;
    }
    else {
        for(size_t i=0; i<ramps.size(); i++) {
            if(t <= ramps[i].endTime) {
                ramps[i].Derivative(t,dx);
                return;
            }
            t -= ramps[i].endTime;
        }
        dx = ramps.back().dx1;
    }
}

void DynamicPath::SetMilestones(const vector<Vector>& x)
{
    if(x.empty()) {
        ramps.resize(0);
    }
    else if(x.size()==1) {
        ramps.resize(1);
        ramps[0].SetConstant(x[0]);
    }
    else {
        Vector zero(x[0].size(),0.0);
        ramps.resize(x.size()-1);
        for(size_t i=0; i<ramps.size(); i++) {
            ramps[i].x0 = x[i];
            ramps[i].x1 = x[i+1];
            ramps[i].dx0 = zero;
            ramps[i].dx1 = zero;
            bool res=ramps[i].SolveMinTimeLinear(accMax,velMax);
            PARABOLIC_RAMP_ASSERT(res && ramps[i].IsValid());
        }
    }
}

void DynamicPath::SetMilestones(const vector<Vector>& x,const vector<Vector>& dx)
{
    if(x.empty()) {
        ramps.resize(0);
    }
    else if(x.size()==1) {
        ramps.resize(1);
        ramps[0].SetConstant(x[0]);
    }
    else {
        if(xMin.empty()) {
            ramps.resize(x.size()-1);
            for(size_t i=0; i<ramps.size(); i++) {
                ramps[i].x0 = x[i];
                ramps[i].x1 = x[i+1];
                ramps[i].dx0 = dx[i];
                ramps[i].dx1 = dx[i+1];
                bool res=ramps[i].SolveMinTime(accMax,velMax);
                PARABOLIC_RAMP_ASSERT(res);
                PARABOLIC_RAMP_ASSERT(ramps[i].IsValid());
            }
        }
        else {
            //bounded solving
            ramps.resize(0);
            ramps.reserve(x.size()-1);
            std::vector<std::vector<ParabolicRamp1D> > tempRamps;
            std::vector<ParabolicRampND> tempRamps2;
            PARABOLIC_RAMP_ASSERT(InBounds(x[0],xMin,xMax));
            for(size_t i=0; i+1<x.size(); i++) {
                PARABOLIC_RAMP_ASSERT(InBounds(x[i+1],xMin,xMax));
                Real res=SolveMinTimeBounded(x[i],dx[i],x[i+1],dx[i+1], accMax,velMax,xMin,xMax, tempRamps,_multidofinterp);
                PARABOLIC_RAMP_ASSERT(res >= 0);
                tempRamps2.resize(0);
                CombineRamps(tempRamps,tempRamps2);
                for(size_t j = 0; j < tempRamps2.size(); ++j) {
                    PARABOLIC_RAMP_ASSERT(tempRamps2[j].IsValid());
                }
                ramps.insert(ramps.end(),tempRamps2.begin(),tempRamps2.end());
            }
        }
    }
}

void DynamicPath::GetMilestones(vector<Vector>& x,vector<Vector>& dx) const
{
    if(ramps.empty()) {
        x.resize(0);
        dx.resize(0);
        return;
    }
    x.resize(ramps.size()+1);
    dx.resize(ramps.size()+1);
    x[0] = ramps[0].x0;
    dx[0] = ramps[0].dx0;
    for(size_t i=0; i<ramps.size(); i++) {
        x[i+1] = ramps[i].x1;
        dx[i+1] = ramps[i].dx1;
    }
}

void DynamicPath::Append(const Vector& x)
{
    size_t n=ramps.size();
    size_t p=n-1;
    if(ramps.size()==0) {
        ramps.resize(1);
        ramps[0].SetConstant(x);
    }
    else {
        if(xMin.empty()) {
            ramps.resize(ramps.size()+1);
            ramps[n].x0 = ramps[p].x1;
            ramps[n].dx0 = ramps[p].dx1;
            ramps[n].x1 = x;
            ramps[n].dx1.resize(x.size());
            fill(ramps[n].dx1.begin(),ramps[n].dx1.end(),0);
            bool res=ramps[n].SolveMinTime(accMax,velMax);
            PARABOLIC_RAMP_ASSERT(res);
        }
        else {
            PARABOLIC_RAMP_ASSERT(InBounds(x,xMin,xMax));
            std::vector<std::vector<ParabolicRamp1D> > tempRamps;
            std::vector<ParabolicRampND> tempRamps2;
            Vector zero(x.size(),0.0);
            Real res=SolveMinTimeBounded(ramps[p].x1,ramps[p].dx1,x,zero, accMax,velMax,xMin,xMax,tempRamps,_multidofinterp);
            PARABOLIC_RAMP_ASSERT(res>=0);
            tempRamps2.resize(0);
            CombineRamps(tempRamps,tempRamps2);
            ramps.insert(ramps.end(),tempRamps2.begin(),tempRamps2.end());
        }
    }
}

void DynamicPath::Append(const Vector& x,const Vector& dx)
{
    size_t n=ramps.size();
    size_t p=n-1;
    PARABOLIC_RAMP_ASSERT(ramps.size()!=0);
    if(xMin.empty()) {
        ramps.resize(ramps.size()+1);
        ramps[n].x0 = ramps[p].x1;
        ramps[n].dx0 = ramps[p].dx1;
        ramps[n].x1 = x;
        ramps[n].dx1 = dx;
        bool res=ramps[n].SolveMinTime(accMax,velMax);
        PARABOLIC_RAMP_ASSERT(res);
    }
    else {
        PARABOLIC_RAMP_ASSERT(InBounds(x,xMin,xMax));
        std::vector<std::vector<ParabolicRamp1D> > tempRamps;
        std::vector<ParabolicRampND> tempRamps2;
        Real res=SolveMinTimeBounded(ramps[p].x1,ramps[p].dx1,x,dx, accMax,velMax,xMin,xMax,tempRamps,_multidofinterp);
        PARABOLIC_RAMP_ASSERT(res>=0);
        tempRamps2.resize(0);
        CombineRamps(tempRamps,tempRamps2);
        ramps.insert(ramps.end(),tempRamps2.begin(),tempRamps2.end());
    }
}

void DynamicPath::Concat(const DynamicPath& suffix)
{
    PARABOLIC_RAMP_ASSERT(&suffix != this);
    if(suffix.ramps.empty()) {
        return;
    }
    if(ramps.empty()) {
        *this=suffix;
        return;
    }
    //double check continuity
    if(ramps.back().x1 != suffix.ramps.front().x0 ||
       ramps.back().dx1 != suffix.ramps.front().dx0) {
        Real xmax=0,dxmax=0;
        for(size_t i=0; i<ramps.back().x1.size(); i++) {
            xmax=Max(xmax,Abs(ramps.back().x1[i]-suffix.ramps.front().x0[i]));
            dxmax=Max(dxmax,Abs(ramps.back().dx1[i]-suffix.ramps.front().dx0[i]));
        }
        if(Abs(xmax) > EpsilonX || Abs(dxmax) > EpsilonV) {
            PARABOLIC_RAMP_PLOG("Concat endpoint error\n");
            PARABOLIC_RAMP_PLOG("x:\n");
            for(size_t i=0; i<ramps.back().x1.size(); i++) {
                PARABOLIC_RAMP_PLOG("%g - %g = %g\n",ramps.back().x1[i],suffix.ramps.front().x0[i],ramps.back().x1[i]-suffix.ramps.front().x0[i]);
            }
            PARABOLIC_RAMP_PLOG("dx:\n");
            for(size_t i=0; i<ramps.back().x1.size(); i++) {
                PARABOLIC_RAMP_PLOG("%g - %g = %g\n",ramps.back().dx1[i],suffix.ramps.front().dx0[i],ramps.back().dx1[i]-suffix.ramps.front().dx0[i]);
            }
        }
        ramps.back().x1 = ramps.front().x0;
        ramps.back().dx1 = ramps.front().dx0;
        for(size_t i=0; i<ramps.back().x1.size(); i++) {
            ramps.back().ramps[i].x1 = suffix.ramps.front().x0[i];
            ramps.back().ramps[i].dx1 = suffix.ramps.front().dx0[i];
        }
    }
    PARABOLIC_RAMP_ASSERT(ramps.back().x1 == suffix.ramps.front().x0);
    PARABOLIC_RAMP_ASSERT(ramps.back().dx1 == suffix.ramps.front().dx0);
    ramps.insert(ramps.end(),suffix.ramps.begin(),suffix.ramps.end());
}

void DynamicPath::Split(Real t,DynamicPath& before,DynamicPath& after) const
{
    PARABOLIC_RAMP_ASSERT(IsValid());
    PARABOLIC_RAMP_ASSERT(&before != this);
    PARABOLIC_RAMP_ASSERT(&after != this);
    if(ramps.empty()) {
        before=*this;
        after=*this;
        return;
    }
    after.velMax = before.velMax = velMax;
    after.accMax = before.accMax = accMax;
    after.xMin = before.xMin = xMin;
    after.xMax = before.xMax = xMax;
    after.ramps.resize(0);
    before.ramps.resize(0);
    if(t < 0) {  //we're before the path starts
        before.ramps.resize(1);
        before.ramps[0].SetConstant(ramps[0].x0);
        //place a constant for time -t on the after path
        after.ramps.resize(1);
        after.ramps[0].SetConstant(ramps[0].x0,-t);
    }
    for(size_t i=0; i<ramps.size(); i++) {
        if(t < 0) {
            after.ramps.push_back(ramps[i]);
        }
        else {
            if(t < ramps[i].endTime) {
                //cut current path
                ParabolicRampND temp=ramps[i];
                temp.TrimBack(temp.endTime-t);
                before.ramps.push_back(temp);
                temp=ramps[i];
                temp.TrimFront(t);
                if(!after.ramps.empty()) {
                    PARABOLIC_RAMP_PLOG("DynamicPath::Split: Uh... weird, after is not empty?\n");
                    PARABOLIC_RAMP_PLOG("t = %g, i = %d, endtime = %g\n",t,i,ramps[i].endTime);
                }
                PARABOLIC_RAMP_ASSERT(after.ramps.size() == 0);
                after.ramps.push_back(temp);
            }
            else {
                before.ramps.push_back(ramps[i]);
            }
            t -= ramps[i].endTime;
        }
    }

    if(t > 0) {  //dt is longer than path
        ParabolicRampND temp;
        temp.SetConstant(ramps.back().x1,t);
        before.ramps.push_back(temp);
    }
    if(t >= 0) {
        ParabolicRampND temp;
        temp.SetConstant(ramps.back().x1);
        after.ramps.push_back(temp);
    }
    PARABOLIC_RAMP_ASSERT(before.IsValid());
    PARABOLIC_RAMP_ASSERT(after.IsValid());
}


struct RampSection
{
    Real ta,tb;
    Vector xa,xb;
    Real da,db;
};


int CheckRamp(const ParabolicRampND& ramp,FeasibilityCheckerBase* feas,DistanceCheckerBase* distance,int maxiters, __attribute__((unused)) int options)
{
    ramp.constraintchecked = 1;
    int ret0 = feas->ConfigFeasible(ramp.x0, ramp.dx0);
    if( ret0 != 0) {
        return ret0;
    }
    int ret1 = feas->ConfigFeasible(ramp.x1, ramp.dx1);
    if( ret1 != 0 ) {
        return ret1;
    }
    PARABOLIC_RAMP_ASSERT(distance->ObstacleDistanceNorm()==Inf);
    RampSection section;
    section.ta = 0;
    section.tb = ramp.endTime;
    section.xa = ramp.x0;
    section.xb = ramp.x1;
    section.da = distance->ObstacleDistance(ramp.x0);
    section.db = distance->ObstacleDistance(ramp.x1);
    if(section.da <= 0.0) {
        return 0xffff; // no code
    }
    if(section.db <= 0.0) {
        return 0xffff; // no code
    }
    list<RampSection> queue;
    queue.push_back(section);
    int iters=0;
    while(!queue.empty()) {
        section = queue.front();
        queue.erase(queue.begin());

        //check the bounds around this section
        if(LInfDistance(section.xa,section.xb) <= section.da+section.db) {
            Vector bmin,bmax;
            ramp.Bounds(section.ta,section.tb,bmin,bmax);
            if(MaxBBLInfDistance(section.xa,bmin,bmax) < section.da && MaxBBLInfDistance(section.xb,bmin,bmax) < section.db) {
                //can cull out the section
                continue;
            }
        }
        Real tc = (section.ta+section.tb)*0.5;
        Vector xc, dxc;
        ramp.Evaluate(tc,xc);
        if( feas->NeedDerivativeForFeasibility() ) {
            ramp.Derivative(tc,dxc);
        }
        //infeasible config
        int retseg = feas->ConfigFeasible(xc, dxc);
        if( retseg != 0 ) {
            return retseg;
        }
        //subdivide
        Real dc = distance->ObstacleDistance(xc);
        RampSection sa,sb;
        sa.ta = section.ta;
        sa.xa = section.xa;
        sa.da = section.da;
        sa.tb = sb.ta = tc;
        sa.xb = sb.xa = xc;
        sa.db = sb.da = dc;
        sb.tb = section.tb;
        sb.xb = section.xb;
        sb.db = section.db;

        //recurse on segments
        queue.push_back(sa);
        queue.push_back(sb);

        if(iters++ >= maxiters) {
            return 0xffff; // no code
        }
    }
    return 0;
}

int CheckRamp(const ParabolicRampND& ramp,FeasibilityCheckerBase* space,const Vector& tol,int options)
{
    PARABOLIC_RAMP_ASSERT(tol.size() == ramp.ramps.size());
    for(size_t i = 0; i < tol.size(); ++i) {
        PARABOLIC_RAMP_ASSERT(tol[i] > 0);
    }
    int ret0 = space->ConfigFeasible(ramp.x0, ramp.dx0, options);
    if( ret0 != 0 ) {
        return ret0;
    }
    int ret1 = space->ConfigFeasible(ramp.x1, ramp.dx1, options);
    if( ret1 != 0 ) {
        return ret1;
    }
    //PARABOLIC_RAMP_ASSERT(space->ConfigFeasible(ramp.x0));
    //PARABOLIC_RAMP_ASSERT(space->ConfigFeasible(ramp.x1));

    //for a parabola of form f(x) = a x^2 + b x, and the straight line
    //of form g(X,u) = u*f(X)
    //d^2(g(X,u),p) = |p - <f(X),p>/<f(X),f(X)> f(X)|^2 < tol^2
    //<p,p> - <f(X),p>^2/<f(X),f(X)>  = p^T (I-f(X)f(X)^T/f(X)^T f(X)) p
    //max_x d^2(f(x)) => f(x)^T (I-f(X)f(X)^T/f(X)^T f(X)) f'(x) = 0
    //(x^2 a^T + x b^T) A (2a x + b) = 0
    //(x a^T + b^T) A (2a x + b) = 0
    //2 x^2 a^T A a + 3 x b^T A a + b^T A b = 0

    //the max X for which f(x) deviates from g(X,x) by at most tol is...
    //max_x |g(X,x)-f(x)| = max_x x/X f(X)-f(x)
    //=> f(X)/X - f'(x) = 0
    //=>  X/2 = x
    //=> max_x |g(X,x)-f(x)| = |(X/2)/X f(X)-f(X/2)|
    //= |1/2 (aX^2+bX) - a(X/2)^2 - b(X/2) + c |
    //= |a| X^2 / 4
    //so... max X st max_x |g(X,x)-f(x)| < tol => X = 2*sqrt(tol/|a|)
    vector<Real> divs;
    Real t=0;
    divs.push_back(t);
    while(t < ramp.endTime) {
        Real tnext=t;
        Real amax = 0;
        Real switchNext=ramp.endTime;
        Real dtmin = 1e30;
        for(size_t i=0; i<ramp.ramps.size(); i++) {
            if(t < ramp.ramps[i].tswitch1) {  //ramp up
                switchNext =  Min(switchNext, ramp.ramps[i].tswitch1);
                amax = Max(amax,Max(Abs(ramp.ramps[i].a1),Abs(ramp.ramps[i].a2)));
                dtmin = Min(dtmin,2.0*Sqrt(tol[i]/amax));
            }
            else if(t < ramp.ramps[i].tswitch2) {  //constant vel
                switchNext = Min(switchNext, ramp.ramps[i].tswitch2);
            }
            else if(t < ramp.ramps[i].ttotal) {  //ramp down
                amax = Max(amax,Max(Abs(ramp.ramps[i].a1),Abs(ramp.ramps[i].a2)));
                dtmin = Min(dtmin,2.0*Sqrt(tol[i]/amax));
            }
        }
        if(t+dtmin > switchNext) {
            tnext = switchNext;
        }
        else {
            tnext = t+dtmin;
        }
        t = tnext;
        divs.push_back(tnext);
    }
    divs.push_back(ramp.endTime);

    //do a bisection thingie
    list<pair<int,int> > segs;
    segs.emplace_back(0,divs.size()-1);
    Vector q1,q2, dq1, dq2;
    while(!segs.empty()) {
        int i=segs.front().first;
        int j=segs.front().second;
        segs.erase(segs.begin());
        if(j == i+1) {
            //check path from t to tnext
            ramp.Evaluate(divs[i],q1);
            if( space->NeedDerivativeForFeasibility() ) {
                ramp.Derivative(divs[i],dq1);
            }
            ramp.Evaluate(divs[j],q2);
            if( space->NeedDerivativeForFeasibility() ) {
                ramp.Derivative(divs[j],dq2);
            }
            int retseg = space->SegmentFeasible(q1,q2, dq1, dq2, divs[j]-divs[i],options);
            if( retseg != 0 ) {
                return retseg;
            }
        }
        else {
            int k=(i+j)/2;
            ramp.Evaluate(divs[k],q1);
            if( space->NeedDerivativeForFeasibility() ) {
                ramp.Derivative(divs[k],dq1);
            }
            int retconf = space->ConfigFeasible(q1, dq1,options);
            if( retconf != 0 ) {
                return retconf;
            }
            segs.emplace_back(i,k);
            segs.emplace_back(k,j);
        }
    }
    return 0;
}

RampFeasibilityChecker::RampFeasibilityChecker(FeasibilityCheckerBase* _feas)
    : feas(_feas),tol(0),distance(NULL),maxiters(0), constraintsmask(0)
{
}

RampFeasibilityChecker::RampFeasibilityChecker(FeasibilityCheckerBase* _feas,DistanceCheckerBase* _distance,int _maxiters)
    : feas(_feas),tol(0),distance(_distance),maxiters(_maxiters), constraintsmask(0)
{
}

int RampFeasibilityChecker::Check(const ParabolicRampND& x,int options)
{
    // only set constraintchecked if all necessary constraints are checked
    if( (options & constraintsmask) == constraintsmask ) {
        x.constraintchecked = 1;
    }
    if(distance) {
        return CheckRamp(x,feas,distance,maxiters,options);
    }
    else {
        return CheckRamp(x,feas,tol,options);
    }
}

bool DynamicPath::TryShortcut(Real t1,Real t2,RampFeasibilityChecker& check)
{
    if(t1 > t2) {
        Swap(t1,t2);
    }
    Real u1 = -1.0, u2 = -1.0;
    int i1 = GetSegment(t1,u1);
    int i2 = GetSegment(t2,u2);
    if(i1 == i2) {
        return false;
    }
    PARABOLIC_RAMP_ASSERT(u1 >= 0);
    PARABOLIC_RAMP_ASSERT(u1 <= ramps[i1].endTime+EpsilonT);
    PARABOLIC_RAMP_ASSERT(u2 >= 0);
    PARABOLIC_RAMP_ASSERT(u2 <= ramps[i2].endTime+EpsilonT);
    u1 = Min(u1,ramps[i1].endTime);
    u2 = Min(u2,ramps[i2].endTime);
    DynamicPath intermediate;
    if(xMin.empty()) {
        intermediate.ramps.resize(1);
        ParabolicRampND& test=intermediate.ramps[0];
        ramps[i1].Evaluate(u1,test.x0);
        ramps[i2].Evaluate(u2,test.x1);
        ramps[i1].Derivative(u1,test.dx0);
        ramps[i2].Derivative(u2,test.dx1);
        bool res=test.SolveMinTime(accMax,velMax);
        if(!res) {
            return false;
        }
        PARABOLIC_RAMP_ASSERT(test.endTime >= 0);
        PARABOLIC_RAMP_ASSERT(test.IsValid());
    }
    else {
        Vector x0,x1,dx0,dx1;
        ramps[i1].Evaluate(u1,x0);
        ramps[i2].Evaluate(u2,x1);
        ramps[i1].Derivative(u1,dx0);
        ramps[i2].Derivative(u2,dx1);
        vector<std::vector<ParabolicRamp1D> > intramps;
        Real res=SolveMinTimeBounded(x0,dx0,x1,dx1, accMax,velMax,xMin,xMax, intramps,_multidofinterp);
        if(res < 0) {
            return false;
        }
        intermediate.ramps.resize(0);
        CombineRamps(intramps,intermediate.ramps);
        intermediate.accMax = accMax;
        intermediate.velMax = velMax;
        PARABOLIC_RAMP_ASSERT(intermediate.IsValid());
    }
    for(size_t i=0; i<intermediate.ramps.size(); i++) {
        if(check.Check(intermediate.ramps[i]) != 0) {
            return false;
        }
    }

    //perform shortcut
    //crop i1 and i2
    ramps[i1].TrimBack(ramps[i1].endTime-u1);
    ramps[i1].x1 = intermediate.ramps.front().x0;
    ramps[i1].dx1 = intermediate.ramps.front().dx0;
    ramps[i2].TrimFront(u2);
    ramps[i2].x0 = intermediate.ramps.back().x1;
    ramps[i2].dx0 = intermediate.ramps.back().dx1;

    //replace intermediate ramps with test
    for(int i=0; i<i2-i1-1; i++) {
        ramps.erase(ramps.begin()+i1+1);
    }
    ramps.insert(ramps.begin()+i1+1,intermediate.ramps.begin(),intermediate.ramps.end());

    //check for consistency
    for(size_t i=0; i+1<ramps.size(); i++) {
        PARABOLIC_RAMP_ASSERT(ramps[i].x1 == ramps[i+1].x0);
        PARABOLIC_RAMP_ASSERT(ramps[i].dx1 == ramps[i+1].dx0);
    }
    return true;
}

int DynamicPath::Shortcut(int numIters,RampFeasibilityChecker& check, Real mintimestep)
{
    RandomNumberGeneratorBase rng;
    return Shortcut(numIters,check,&rng,mintimestep);
}

int DynamicPath::Shortcut(int numIters,RampFeasibilityChecker& check,RandomNumberGeneratorBase* rng, Real mintimestep)
{
    int shortcuts = 0;
    vector<Real> rampStartTime(ramps.size());
    Real endTime=0;
    for(size_t i=0; i<ramps.size(); i++) {
        rampStartTime[i] = endTime;
        endTime += ramps[i].endTime;
    }
    Vector x0,x1,dx0,dx1;
    DynamicPath intermediate;
    for(int iters=0; iters<numIters; iters++) {
        Real t1=rng->Rand()*endTime,t2=rng->Rand()*endTime;
        if( iters == 0 ) {
            t1 = 0;
            t2 = endTime;
        }
        if(t1 > t2) {
            Swap(t1,t2);
        }
        int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
        int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
        if(i1 == i2) {
            continue;
        }
        //same ramp
        Real u1 = t1-rampStartTime[i1];
        Real u2 = t2-rampStartTime[i2];
        PARABOLIC_RAMP_ASSERT(u1 >= 0);
        PARABOLIC_RAMP_ASSERT(u1 <= ramps[i1].endTime+EpsilonT);
        PARABOLIC_RAMP_ASSERT(u2 >= 0);
        PARABOLIC_RAMP_ASSERT(u2 <= ramps[i2].endTime+EpsilonT);
        u1 = Min(u1,ramps[i1].endTime);
        u2 = Min(u2,ramps[i2].endTime);
        ramps[i1].Evaluate(u1,x0);
        ramps[i2].Evaluate(u2,x1);
        ramps[i1].Derivative(u1,dx0);
        ramps[i2].Derivative(u2,dx1);
        bool res=SolveMinTime(x0,dx0,x1,dx1,accMax,velMax,xMin,xMax,intermediate,_multidofinterp);
        if(!res) {
            continue;
        }
        // check the new ramp time makes significant steps
        Real newramptime = intermediate.GetTotalTime();
        if( newramptime+mintimestep > t2-t1 ) {
            // reject since it didn't make significant improvement
            PARABOLIC_RAMP_PLOG("shortcut iter=%d rejected time=%fs\n", iters, endTime-(t2-t1)+newramptime);
            continue;
        }

        bool feas=true;
        for(size_t i=0; i<intermediate.ramps.size(); i++)
            if(check.Check(intermediate.ramps[i]) != 0) {
                feas=false;
                break;
            }
        if(!feas) {
            continue;
        }
        //perform shortcut
        shortcuts++;
        ramps[i1].TrimBack(ramps[i1].endTime-u1);
        ramps[i1].x1 = intermediate.ramps.front().x0;
        ramps[i1].dx1 = intermediate.ramps.front().dx0;
        ramps[i2].TrimFront(u2);
        ramps[i2].x0 = intermediate.ramps.back().x1;
        ramps[i2].dx0 = intermediate.ramps.back().dx1;

        //replace intermediate ramps
        for(int i=0; i<i2-i1-1; i++) {
            ramps.erase(ramps.begin()+i1+1);
        }
        ramps.insert(ramps.begin()+i1+1,intermediate.ramps.begin(),intermediate.ramps.end());

        //check for consistency
        for(size_t i=0; i+1<ramps.size(); i++) {
            PARABOLIC_RAMP_ASSERT(ramps[i].x1 == ramps[i+1].x0);
            PARABOLIC_RAMP_ASSERT(ramps[i].dx1 == ramps[i+1].dx0);
        }

        //revise the timing
        rampStartTime.resize(ramps.size());
        endTime=0;
        for(size_t i=0; i<ramps.size(); i++) {
            rampStartTime[i] = endTime;
            endTime += ramps[i].endTime;
        }
        PARABOLIC_RAMP_PLOG("shortcut iter=%d endTime=%f\n",iters,endTime);
    }
    return shortcuts;
}

int DynamicPath::ShortCircuit(RampFeasibilityChecker& check)
{
    int shortcuts=0;
    DynamicPath intermediate;
    for(int i=0; i+1<(int)ramps.size(); i++) {
        bool res=SolveMinTime(ramps[i].x0,ramps[i].dx0,ramps[i].x1,ramps[i].dx1,accMax,velMax,xMin,xMax,intermediate,_multidofinterp);
        if(!res) {
            continue;
        }
        bool feas=true;
        for(size_t j=0; j<intermediate.ramps.size(); j++) {
            if(check.Check(intermediate.ramps[j]) != 0) {
                feas=false;
                break;
            }
        }
        if(!feas) {
            continue;
        }

        ramps.erase(ramps.begin()+i+1);
        ramps.insert(ramps.begin()+i+1,intermediate.ramps.begin(),intermediate.ramps.end());
        i += (int)intermediate.ramps.size()-2;
        shortcuts++;
    }
    return shortcuts;
}

int DynamicPath::OnlineShortcut(Real leadTime,Real padTime,RampFeasibilityChecker& check)
{
    RandomNumberGeneratorBase rng;
    return OnlineShortcut(leadTime,padTime,check,&rng);
}

int DynamicPath::OnlineShortcut(Real leadTime,Real padTime,RampFeasibilityChecker& check,RandomNumberGeneratorBase* rng)
{
    Timer timer;
    int shortcuts = 0;
    vector<Real> rampStartTime(ramps.size());
    Real endTime=0;
    for(size_t i=0; i<ramps.size(); i++) {
        rampStartTime[i] = endTime;
        endTime += ramps[i].endTime;
    }
    Vector x0,x1,dx0,dx1;
    DynamicPath intermediate;
    while(1) {
        //can only start from here
        Real starttime = timer.ElapsedTime()-leadTime;
        if(starttime+padTime >= endTime) {
            break;
        }
        starttime = Max(0.0,starttime+padTime);

        Real t1=starttime+Sqr(rng->Rand())*(endTime-starttime),t2=starttime+rng->Rand()*(endTime-starttime);
        if(t1 > t2) {
            Swap(t1,t2);
        }
        int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
        int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
        if(i1 == i2) {
            //same ramp
            continue;
        }
        Real u1 = t1-rampStartTime[i1];
        Real u2 = t2-rampStartTime[i2];
        PARABOLIC_RAMP_ASSERT(u1 >= 0);
        PARABOLIC_RAMP_ASSERT(u1 <= ramps[i1].endTime+EpsilonT);
        PARABOLIC_RAMP_ASSERT(u2 >= 0);
        PARABOLIC_RAMP_ASSERT(u2 <= ramps[i2].endTime+EpsilonT);
        u1 = Min(u1,ramps[i1].endTime);
        u2 = Min(u2,ramps[i2].endTime);
        ramps[i1].Evaluate(u1,x0);
        ramps[i2].Evaluate(u2,x1);
        ramps[i1].Derivative(u1,dx0);
        ramps[i2].Derivative(u2,dx1);
        bool res=SolveMinTime(x0,dx0,x1,dx1,accMax,velMax,xMin,xMax,intermediate,_multidofinterp);
        if(!res) {
            continue;
        }
        bool feas=true;
        for(size_t i=0; i<intermediate.ramps.size(); i++) {
            if(check.Check(intermediate.ramps[i]) != 0) {
                feas=false;
                break;
            }
        }
        if(!feas) {
            continue;
        }
        //check for time elapse, otherwise can't perform this shortcut
        if(timer.ElapsedTime()-leadTime > t1) {
            continue;
        }

        shortcuts++;
        //crop i1 and i2
        ramps[i1].TrimBack(ramps[i1].endTime-u1);
        ramps[i1].x1 = intermediate.ramps.front().x0;
        ramps[i1].dx1 = intermediate.ramps.front().dx0;
        ramps[i2].TrimFront(u2);
        ramps[i2].x0 = intermediate.ramps.back().x1;
        ramps[i2].dx0 = intermediate.ramps.back().dx1;
        PARABOLIC_RAMP_ASSERT(ramps[i1].IsValid());
        PARABOLIC_RAMP_ASSERT(ramps[i2].IsValid());

        //replace intermediate ramps
        for(int i=0; i<i2-i1-1; i++) {
            ramps.erase(ramps.begin()+i1+1);
        }
        ramps.insert(ramps.begin()+i1+1,intermediate.ramps.begin(),intermediate.ramps.end());

        //check for consistency
        for(size_t i=0; i+1<ramps.size(); i++) {
            PARABOLIC_RAMP_ASSERT(ramps[i].x1 == ramps[i+1].x0);
            PARABOLIC_RAMP_ASSERT(ramps[i].dx1 == ramps[i+1].dx0);
        }

        //revise the timing
        rampStartTime.resize(ramps.size());
        endTime=0;
        for(size_t i=0; i<ramps.size(); i++) {
            rampStartTime[i] = endTime;
            endTime += ramps[i].endTime;
        }
    }
    return shortcuts;
}

bool DynamicPath::IsValid() const
{
    if(ramps.empty()) {
        PARABOLICWARN("DynamicPath::IsValid: empty path\n");
        return false;
    }
    for(size_t i=0; i<ramps.size(); i++) {
        if(!ramps[i].IsValid()) {
            PARABOLICWARN("DynamicPath::IsValid: ramp %d is invalid\n",i);
            return false;
        }
        for(size_t j=0; j<ramps[i].ramps.size(); j++) {
            if(Abs(ramps[i].ramps[j].a1) > accMax.at(j)+EpsilonA || Abs(ramps[i].ramps[j].v) > velMax.at(j)+EpsilonV || Abs(ramps[i].ramps[j].a2) > accMax.at(j)+EpsilonA) {
                PARABOLICWARN("DynamicPath::IsValid: invalid acceleration or velocity on ramp %d\n",i);
                PARABOLICWARN("\ta1 %g, v %g, a2 %g.  amax %g, vmax %g\n",ramps[i].ramps[j].a1,ramps[i].ramps[j].v,ramps[i].ramps[j].a2,accMax[j],velMax[j]);
                return false;
            }
        }
    }
    for(size_t i=1; i<ramps.size(); i++) {
        if(ramps[i].x0 != ramps[i-1].x1) {
            PARABOLICWARN("DynamicPath::IsValid: discontinuity at ramp %d\n",i);
            for(size_t j=0; j<ramps[i].x0.size(); j++) {
                PARABOLICWARN("%g ",ramps[i].x0[j]);
            }
            PARABOLICWARN("\n");
            for(size_t j=0; j<ramps[i-1].x1.size(); j++) {
                PARABOLICWARN("%g ",ramps[i-1].x1[j]);
            }
            PARABOLICWARN("\n");
            return false;
        }
        if(ramps[i].dx0 != ramps[i-1].dx1) {
            PARABOLICWARN("DynamicPath::IsValid: derivative discontinuity at ramp %d\n",i);
            for(size_t j=0; j<ramps[i].dx0.size(); j++) {
                PARABOLICWARN("%g ",ramps[i].dx0[j]);
            }
            PARABOLICWARN("\n");
            for(size_t j=0; j<ramps[i-1].dx1.size(); j++) {
                PARABOLICWARN("%g ",ramps[i-1].dx1[j]);
            }
            PARABOLICWARN("\n");
            return false;
        }
    }
    return true;
}

////////Puttichai
void DynamicPath::Save(std::string filename) const {
    size_t ndof = ramps[0].ramps.size();
    for (size_t i = 1; i < ramps.size(); ++i) {
        // simple verification
        PARABOLIC_RAMP_ASSERT(ramps[i].ramps.size() == ndof);
    }

    std::string s = "";
    std::string dummy;

    for (size_t iramp = 0; iramp < ramps.size(); ++iramp) {
        ramps[iramp].ToString(dummy);
        s = s + dummy;
    }

    std::ofstream f(filename.c_str());
    f << s;
}

} //namespace ParabolicRamp
