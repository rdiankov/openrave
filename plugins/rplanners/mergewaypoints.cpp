// -*- coding: utf-8 -*-
// Copyright (C) 2013 Cuong Pham <cuong.pham@normalesup.org>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "plugindefs.h"
#include <openrave/planningutils.h>
#include "mergewaypoints.h"

#define INF 1e30
#define TINY g_fEpsilonLinear


namespace mergewaypoints {

/** Interval computations
 */

class Interval {
public:
    dReal lo,hi;
    bool isVoid;
    Interval();
    Interval(dReal,dReal);
};

Interval::Interval(){
    isVoid = true;
}

Interval::Interval(dReal a, dReal b){
    if (a>b) {
        isVoid = true;
    }
    else{
        isVoid = false;
        lo = a;
        hi = b;
    }
}

Interval Intersection(Interval interval1,Interval interval2)
{
    if(interval1.isVoid || interval2.isVoid) {
        return Interval();
    }
    return Interval(max(interval1.lo,interval2.lo),min(interval1.hi,interval2.hi));
}

//Solve the inequality ax => b
Interval SolveIneq(dReal a,dReal b){
    if (RaveFabs(a)<=TINY) {
        if (b >= 0) {
            return Interval();
        }
        else{
            return Interval(-INF,INF);
        }
    }
    else{
        if (a>0) {
            return Interval(b/a,INF);
        }
        else{
            return Interval(-INF,b/a);
        }
    }
}

/// checks the velocity, acceleration, and joint limits
bool CheckValidity(dReal Ta,dReal Tb,const std::vector<dReal>& q0,const std::vector<dReal>& v0,const std::vector<dReal>& q2,const std::vector<dReal>& v2,std::vector<dReal>& qres,std::vector<dReal>& vres,ConstraintTrajectoryTimingParametersPtr params){

    vector<dReal> amax = params->_vConfigAccelerationLimit;
    vector<dReal> vmax = params->_vConfigVelocityLimit;
    vector<dReal> qmin = params->_vConfigLowerLimit;
    vector<dReal> qmax = params->_vConfigUpperLimit;

    dReal q1,v1,a0,a1;
    qres.resize(q0.size(),0);
    vres.resize(q0.size(),0);

    for(size_t j=0; j<q0.size(); j++) {
        q1 = q0[j]+0.5*Ta*(Tb*(v0[j]-v2[j])+2*(q2[j]-q0[j]))/(Ta+Tb);
        v1 = (2*(q2[j]-q0[j])-(Ta*v0[j]+Tb*v2[j]))/(Ta+Tb);
        a0 = 1/Ta/(Ta+Tb)*(2*(q2[j]-q0[j])-(2*Ta+Tb)*v0[j]-Tb*v2[j]);
        a1 = 1/Tb/(Ta+Tb)*(-2*(q2[j]-q0[j])+(Ta+2*Tb)*v2[j]+Ta*v0[j]);
        if(!(v1>= -vmax[j]-TINY && v1 <= vmax[j]+TINY && a0>= -amax[j]-TINY && a0 <= amax[j]+TINY && a1>= -amax[j]-TINY && a1 <= amax[j]+TINY)) {
            return false;
        }
        if (!(q1>= qmin[j] && q1 <= qmax[j])) {
            return false;
        }
        dReal tm,qm;
        if (RaveFabs(a0)>TINY) {
            tm = -v0[j]/a0;
            qm = q0[j]+v0[j]*tm+0.5*a0*tm*tm;
            if (tm >0  && tm < Ta && (qm < qmin[j]-TINY || qm > qmax[j]+TINY)) {
                return false;
            }
        }
        if (RaveFabs(a1)>TINY) {
            tm = -v1/a1;
            qm = q1+v1*tm+0.5*a1*tm*tm;
            if (tm > 0 && tm < Tb && (qm < qmin[j]-TINY || qm > qmax[j]+TINY)) {
                return false;
            }
        }
        qres[j] = q1;
        vres[j] = v1;
    }
    return true;
}


/** Check whether an interval T1 can be merged into the two neighboring intervals T0 and T2. The new interval times of T0 and T2 will be Ta and Tb.
    \param q0 the configuration at the start of the ramp T0
    \param v0 the velocity at the start of the ramp T0
    \param q3 the configuration at the end of the ramp at T2
    \param v3 the velocity at the end of the ramp at T2
 */
bool CheckMerge(dReal T0,dReal T1,dReal T2,const std::vector<dReal>& q0,const std::vector<dReal>& v0,const std::vector<dReal>& q3,const std::vector<dReal>& v3,dReal& alpha,std::vector<dReal>& qres,std::vector<dReal>& vres,ConstraintTrajectoryTimingParametersPtr params){
    dReal T = T0+T1+T2;
    dReal Q,A0,B0lo,B0hi,A1lo,A1hi,B1,A2lo,B2lo,A2hi,B2hi;
    vector<dReal> amax = params->_vConfigAccelerationLimit;
    vector<dReal> vmax = params->_vConfigVelocityLimit;
    vector<dReal> qmin = params->_vConfigLowerLimit;
    vector<dReal> qmax = params->_vConfigUpperLimit;
    Interval sol = Interval(T0/T,(T0+T1)/T);

    for(size_t j=0; j<q0.size(); j++) {
        Q = 2*(q3[j]-q0[j])/T;
        // Constraints on v1
        A0 = v3[j]-v0[j];
        B0lo = -vmax[j]-(Q-v3[j]);
        B0hi = vmax[j]-(Q-v3[j]);
        // Constraints on a0
        A1lo = v3[j]-v0[j]+amax[j]*T;
        A1hi = v3[j]-v0[j]-amax[j]*T;
        B1 = v0[j]+v3[j]-Q;
        // Constraints on a1
        A2lo = v0[j]-v3[j]-amax[j]*T;
        B2lo = -amax[j]*T-(2*v3[j]-Q);
        A2hi = v0[j]-v3[j]+amax[j]*T;
        B2hi = amax[j]*T-(2*v3[j]-Q);
        // Solve the inequalities
        sol = Intersection(sol, SolveIneq(A0,B0lo));
        sol = Intersection(sol, SolveIneq(-A0,-B0hi));
        sol = Intersection(sol, SolveIneq(A1lo,B1));
        sol = Intersection(sol, SolveIneq(-A1hi,-B1));
        sol = Intersection(sol, SolveIneq(A2lo,B2lo));
        sol = Intersection(sol, SolveIneq(-A2hi,-B2hi));
    }
    if (sol.isVoid) {
        return false;
    }

    alpha = 0.5*(sol.lo+sol.hi);
    dReal Ta = alpha*T;
    dReal Tb = T-Ta;

    return CheckValidity(Ta,Tb,q0,v0,q3,v3,qres,vres,params);
}


/** Fix the time durations of 2 ramps. Assume that the ramps are unitary
    \param rampx input ramps with x=0,1,
    \param resrampx result ramp with x=0,1
    \param Ta,Tb desired final durations
 */
bool FixRamps(const ParabolicRamp::ParabolicRampND& ramp0,const ParabolicRamp::ParabolicRampND& ramp1,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1, dReal Ta, dReal Tb, ConstraintTrajectoryTimingParametersPtr params){
    vector<dReal> q0 = ramp0.x0, v0=ramp0.dx0, q2=ramp1.x1, v2=ramp1.dx1, qres, vres;
    dReal T0,T1;
    T0 = ramp0.endTime;
    T1 = ramp1.endTime;

    //printf("Try fixing: T0=%f, T1=%f, Ta=%f, Tb=%f...  ",T0,T1,Ta,Tb);
    bool res = CheckValidity(Ta,Tb,q0,v0,q2,v2,qres,vres,params);
    if(!res) {
        //printf("Failed\n");
        return false;
    }

    //printf("Succeded\n");
    resramp0 = ParabolicRamp::ParabolicRampND();
    resramp0.SetPosVelTime(q0,v0,qres,vres,Ta);
    resramp1 = ParabolicRamp::ParabolicRampND();
    resramp1.SetPosVelTime(qres,vres,q2,v2,Tb);
    PARABOLIC_RAMP_ASSERT(resramp0.IsValid()&&resramp1.IsValid());
    bool changed = (RaveFabs(Ta-ramp0.endTime)>TINY) || (RaveFabs(Tb-ramp1.endTime)>TINY);
    resramp0.modified = ramp0.modified || changed;
    resramp1.modified = ramp1.modified || changed;
    return true;
}



/** Merge 3 ramps into 2 ramps while respecting the continuities of joints and velocities on the borders
   Assume that the ramps are unitary
    \param rampx input ramps with x=0,1,2
    \param resrampx result ramp with x=0,1
 */
bool MergeRamps(const ParabolicRamp::ParabolicRampND& ramp0, const ParabolicRamp::ParabolicRampND& ramp1, const ParabolicRamp::ParabolicRampND& ramp2,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1,ConstraintTrajectoryTimingParametersPtr params){
    vector<dReal> q0 = ramp0.x0,v0 = ramp0.dx0, q3=ramp2.x1, v3=ramp2.dx1;
    dReal T,T0,T1,T2,Ta,Tb;
    T0 = ramp0.endTime;
    T1 = ramp1.endTime;
    T2 = ramp2.endTime;
    T = T0+T1+T2;

    //printf("Try to merge: T0=%f, T1=%f, T2=%f\n",T0,T1,T2);
    vector<dReal> qres;
    vector<dReal> vres;
    dReal alpha;
    bool res = CheckMerge(T0,T1,T2,q0,v0,q3,v3,alpha,qres,vres,params);

    if(!res) {
        //printf("Merger failed\n");
        return false;
    }

    Ta = alpha*T;
    Tb = T-Ta;

    //printf("Merger succeded: Ta=%f, Tb=%f\n",Ta,Tb);
    resramp0 = ParabolicRamp::ParabolicRampND();
    resramp0.SetPosVelTime(q0,v0,qres,vres,Ta);
    resramp1 = ParabolicRamp::ParabolicRampND();
    resramp1.SetPosVelTime(qres,vres,q3,v3,Tb);
    resramp0.modified = true;
    resramp1.modified = true;
    PARABOLIC_RAMP_ASSERT(resramp0.IsValid()&&resramp1.IsValid());
    return true;
}

// Small function to compute the factorial of a number
int factorial(int n){
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

/*Iteratively fix the time durations of the ramps
   Note that, by construction, there is no isolated modified ramps
   \param ramps input ramps
   \param desireddurations list of desired durations
 */
bool IterativeFixRamps(std::list<ParabolicRamp::ParabolicRampND>& ramps, std::list<dReal>& desireddurations, ConstraintTrajectoryTimingParametersPtr params){
    if( ramps.size() == 0) {
        return false;
    }
    if (ramps.size() == 1) {
        std::list<ParabolicRamp::ParabolicRampND> resramps;
        dReal coef = desireddurations.back()/ramps.back().endTime;
        bool res = ScaleRampsTime(ramps,resramps,coef,false,params);
        if(res) {
            ramps.swap(resramps);
            return true;
        }
        else{
            return false;
        }
    }
    ParabolicRamp::ParabolicRampND resramp0,resramp1;
    std::list<ParabolicRamp::ParabolicRampND>::iterator itrampprev = ramps.begin(), itrampnext;
    itrampnext = itrampprev;
    ++itrampnext;
    std::list<dReal>::iterator ittprev = desireddurations.begin(), ittnext;
    ittnext = ittprev;
    ++ittnext;

    while(itrampprev != ramps.end() && itrampnext != ramps.end() ) {
        if(RaveFabs(itrampprev->endTime-*ittprev)>TINY || RaveFabs(itrampnext->endTime-*ittnext)>TINY) {
            // Avoid using an unmodified ramp to fix durations
            if (RaveFabs(itrampprev->endTime-*ittprev)<=TINY && !itrampprev->modified) {
                itrampprev = itrampnext++;
                ittprev = ittnext++;
                continue;
            }
            if (RaveFabs(itrampnext->endTime-*ittnext)<=TINY && !itrampnext->modified) {
                itrampnext = itrampprev--;
                ittnext = ittprev--;
                continue;
            }
            //Fix time duration of current ramp and previous ramp
            bool res = FixRamps(*itrampprev,*itrampnext,resramp0,resramp1,*ittprev,*ittnext,params);
            if(!res) {
                return false;
            }
            else {
                *itrampprev = resramp0;
                *itrampnext = resramp1;
            }
        }
        ++itrampnext;
        itrampprev = itrampnext++;
        ++ittnext;
        ittprev = ittnext++;

        if( itrampnext == ramps.end() && itrampprev != ramps.end() ) {
            itrampnext = itrampprev;
            --itrampprev;
            ittnext = ittprev;
            --ittprev;
        }
    }
    return true;
}

/** Iteratively merge all ramps that are shorter than minswitchtime. Does not change the global time duration
    \param origramps input ramps
    \param ramps result ramps
    \param v3 the velocity at the end of the ramp at T2
 */
bool IterativeMergeRampsFixedTime(const std::list<ParabolicRamp::ParabolicRampND>& origramps, std::list<ParabolicRamp::ParabolicRampND>& ramps, ConstraintTrajectoryTimingParametersPtr params, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler){
    // Determine the number max of iterations as a function of the number of short ramps
    size_t i = 0;
    int nb_short_ramps = 0;
    FOREACHC(itramp,origramps){
        if(itramp->endTime < params->minswitchtime && i>0 && i<ramps.size()-1) {
            nb_short_ramps++;
        }
        i++;
    }
    int maxmergeiterations = min(params->maxmergeiterations, 2*factorial(nb_short_ramps));

    int itersi=0;
    bool solvedglobal = false;

    // This loop could be optimized by caching and re-using results of permutations that begin similarly
    // can also do a uniform random sampling of permutations
    while((!solvedglobal) && itersi < maxmergeiterations) {
        //printf("Iteration %d\n",itersi);
        ramps = origramps;
        itersi++;
        // Kill all ramps that are not the first and the last. For every three ramps where the middle ramp is smaller than minswitchtime, merge the ramp that is in the middle to form two new ramps.
        bool solved = false;
        while(true) {
            solved = true;
            size_t i = 0;
            FOREACHC(itramp,ramps){
                if(itramp->endTime < params->minswitchtime && i>0 && i<ramps.size()-1) {
                    solved = false;
                    break;
                }
                i++;
            }
            if (solved) {
                break;
            }
            if(ramps.size()<=2) {
                solved = false;
                break;
            }
            std::vector<int> invalidrampindices;
            invalidrampindices.resize(0);
            i = 0;
            FOREACHC(itramp,ramps){
                if(itramp->endTime < params->minswitchtime && i>0 && i<ramps.size()-1) {
                    invalidrampindices.push_back(i);
                }
                i++;
            }
            int jramp = invalidrampindices[uniformsampler->SampleSequenceOneUInt32()%invalidrampindices.size()];
            std::list<ParabolicRamp::ParabolicRampND>::iterator itlist = ramps.begin();
            std::advance(itlist,jramp+1);
            ParabolicRamp::ParabolicRampND ramp0,ramp1,ramp2,resramp0,resramp1;
            ramp2 =*itlist;
            itlist--;
            ramp1 = *itlist;
            itlist--;
            ramp0 = *itlist;
            bool res = MergeRamps(ramp0,ramp1,ramp2,resramp0,resramp1,params);
            if(!res) {
                solved = false;
                break;
            }
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            ramps.insert(itlist,resramp1);
            itlist--;
            ramps.insert(itlist,resramp0);
        }
        if(!solved) {
            continue;
        }
        // Handle the beginning the trajectory
        while(ramps.begin()->endTime<params->minswitchtime) {
            if (ramps.size()<=2) {
                solved = false;
                break;
            }
            std::list<ParabolicRamp::ParabolicRampND>::iterator itlist = ramps.begin();
            std::advance(itlist,2);
            ParabolicRamp::ParabolicRampND ramp0,ramp1,ramp2,resramp0,resramp1;
            ramp2 =*itlist;
            itlist--;
            ramp1 = *itlist;
            itlist--;
            ramp0 = *itlist;
            bool res = MergeRamps(ramp0,ramp1,ramp2,resramp0,resramp1,params);
            if(!res) {
                solved = false;
                break;
            }
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            ramps.insert(itlist,resramp1);
            itlist--;
            ramps.insert(itlist,resramp0);
        }
        if(!solved) {
            continue;
        }
        // Handle the end the trajectory
        while(ramps.back().endTime < params->minswitchtime) {
            if (ramps.size()<=2) {
                solved= false;
                break;
            }
            std::list<ParabolicRamp::ParabolicRampND>::iterator itlist = ramps.end();
            itlist--;
            ParabolicRamp::ParabolicRampND ramp0,ramp1,ramp2,resramp0,resramp1;
            ramp2 =*itlist;
            itlist--;
            ramp1 = *itlist;
            itlist--;
            ramp0 = *itlist;
            bool res = MergeRamps(ramp0,ramp1,ramp2,resramp0,resramp1,params);
            if(!res) {
                solved= false;
                break;
            }
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            ramps.insert(itlist,resramp1);
            itlist--;
            ramps.insert(itlist,resramp0);
        }
        if (solved) {
            solvedglobal=true;
        }
    }
    if( !solvedglobal ) {
        return false;
    }
    if (checkcontrollertime) {
        std::list<dReal> desireddurations;
        desireddurations.resize(0);
        FOREACHC(itramp, ramps) {
            desireddurations.push_back(ComputeStepSizeCeiling(itramp->endTime,params->_fStepLength));
        }
        return IterativeFixRamps(ramps,desireddurations,params);
    }
    else{
        return solvedglobal;
    }
}

bool ScaleRampsTime(const std::list<ParabolicRamp::ParabolicRampND>& origramps, std::list<ParabolicRamp::ParabolicRampND>& ramps, dReal coef, bool trysmart, ConstraintTrajectoryTimingParametersPtr params){
    ramps.resize(0);
    bool dodilate = RaveFabs(coef-1)>TINY;

    // Try being smart by timescaling only the modified ramps
    if(trysmart && dodilate && origramps.size()>=2) {
        dReal durationbeforemerge = ComputeRampsDuration(origramps);
        dReal durationmodifiedramps = 0;
        FOREACH(itramp,origramps){
            if(itramp->modified) {
                durationmodifiedramps += itramp->endTime;
            }
        }
        if(durationmodifiedramps>TINY) {
            coef = 1+ (coef-1)*durationbeforemerge/durationmodifiedramps;
            ramps = origramps;

            std::list<dReal> desireddurations;
            desireddurations.resize(0);
            FOREACHC(itramp, ramps) {
                dReal newtime = itramp->endTime;
                if(itramp->modified) {
                    newtime = newtime * coef;
                }
                desireddurations.push_back(newtime);
            }
            bool res = IterativeFixRamps(ramps,desireddurations,params);
            if(res) {
                return true;
            }
        }
    }

    // If dilatation is necessary but either the initial or the final trajectory is nonzero, cannot timescale
    if(dodilate && !(iszero(origramps.begin()->dx0) && iszero(origramps.back().dx1))) {
        return false;
    }

    // Time scale the whole trajectory
    FOREACH(itramp,origramps){
        PARABOLIC_RAMP_ASSERT(itramp->IsValid());
        dReal t = itramp->endTime;
        if(t>TINY) {
            ParabolicRamp::Vector q0=itramp->x0, v0=itramp->dx0, q1=itramp->x1, v1=itramp->dx1;
            if(dodilate) {
                dReal invcoef = 1/coef;
                FOREACH(itv,v0){
                    *itv *= invcoef;
                }
                FOREACH(itv,v1){
                    *itv *= invcoef;
                }
            }
            ramps.push_back(ParabolicRamp::ParabolicRampND());
            ramps.back().SetPosVelTime(q0,v0,q1,v1,t*coef);
            ramps.back().modified = itramp->modified || dodilate;
            BOOST_ASSERT(ramps.back().IsValid());
        }
    }
    return true;
}

// Check whether ramps satisfy constraints associated with checker
bool CheckRamps(std::list<ParabolicRamp::ParabolicRampND>&ramps, ParabolicRamp::RampFeasibilityChecker& check){
    FOREACHC(itramp,ramps){
        if(!check.Check(*itramp)) {
            return false;
        }
    }
    return true;
}

// Provides a measure of quality of a ramps
// Now set to the 1/sum(1/rampduration^2) toi penalize small ramps
dReal quality(const std::list<ParabolicRamp::ParabolicRampND>& ramps){
    dReal res=0;
    FOREACHC(itramp,ramps){
        dReal duration = itramp->endTime;
        res += 1/(duration*duration);
    }
    return 1/res;
}


bool FurtherMergeRamps(const std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, bool docheck){
    //int nitersfurthermerge = params->nitersfurthermerge;
    int nitersfurthermerge = 0;
    resramps = origramps;
    bool globalres = false;
    std::list<ParabolicRamp::ParabolicRampND> ramps;
    for(int rep = 0; rep<nitersfurthermerge; rep++) {
        ramps = origramps;
        while(true) {
            if(ramps.size()<3) {
                break;
            }
            std::list<ParabolicRamp::ParabolicRampND>::iterator itlist = ramps.begin();
            int randidx = uniformsampler->SampleSequenceOneUInt32()%(ramps.size()-2);
            std::advance(itlist,randidx);
            ParabolicRamp::ParabolicRampND ramp0,ramp1,ramp2,resramp0,resramp1,resramp0x,resramp1x;
            ramp0 = *itlist;
            itlist++;
            ramp1 = *itlist;
            itlist++;
            ramp2 = *itlist;
            bool resmerge = MergeRamps(ramp0,ramp1,ramp2,resramp0,resramp1,params);
            if(!resmerge) {
                break;
            }
            dReal t0 = ComputeStepSizeCeiling(resramp0.endTime,params->_fStepLength);
            dReal t1 = ComputeStepSizeCeiling(resramp1.endTime,params->_fStepLength);
            if(t1+t0-resramp0.endTime-resramp1.endTime+ComputeRampsDuration(ramps)>upperbound) {
                break;
            }
            bool resfix = FixRamps(resramp0,resramp1,resramp0x,resramp1x,t0,t1,params);
            if(!resfix || !check.Check(resramp0x) || !check.Check(resramp1x)) {
                break;
            }
            globalres = true;
            itlist--;
            itlist--;
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            ramps.insert(itlist,resramp1x);
            itlist--;
            ramps.insert(itlist,resramp0x);
        }
        //cout << "Rep: " << rep << "\n";
        //PrintRamps(ramps,params,true);
        if(ramps.size()<resramps.size() || (ramps.size()==resramps.size()&& quality(ramps)>quality(resramps))) {
            resramps = ramps;
        }
    }
    //cout << "Finally\n";
    //PrintRamps(resramps,params,true);
    return globalres;
}

bool IterativeMergeRamps(const std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, bool docheck){
    std::list<ParabolicRamp::ParabolicRampND> ramps,ramps2;
    dReal testcoef;

    //printf("Coef = 1\n");
    bool res = IterativeMergeRampsFixedTime(origramps, ramps2, params, checkcontrollertime, uniformsampler);
    res = res && (!docheck || CheckRamps(ramps2,check));
    if (res) {
        resramps.swap(ramps2);
        return true;
    }
    dReal durationbeforemerge = ComputeRampsDuration(origramps);
    dReal maxcoef = upperbound / durationbeforemerge;
    //printf("Coef = %f\n",maxcoef);
    bool canscale = ScaleRampsTime(origramps, ramps, maxcoef, true, params);
    if(!canscale) {
        return false;
    }
    res = IterativeMergeRampsFixedTime(ramps, ramps2, params, checkcontrollertime, uniformsampler);
    res = res && (!docheck || CheckRamps(ramps2,check));
    if (!res) {
        return false;
    }
    resramps.swap(ramps2);

    dReal hi = maxcoef;
    dReal lo = 1;
    while ((hi-lo)*durationbeforemerge > params->_fStepLength) {
        testcoef = (hi+lo)/2;
        //printf("Coef = %f\n",testcoef);
        bool canscale2 = ScaleRampsTime(origramps,ramps,testcoef,true,params);
        if(!canscale2) {
            lo = testcoef;
            continue;
        }
        res = IterativeMergeRampsFixedTime(ramps, ramps2, params, checkcontrollertime, uniformsampler);
        res = res && (!docheck || CheckRamps(ramps2,check));
        if(res) {
            hi = testcoef;
            resramps.swap(ramps2);
        }
        else{
            lo = testcoef;
        }
    }
    return true;
}


bool IterativeMergeRampsNoDichotomy(const std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, dReal stepsize, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, bool docheck){
    std::list<ParabolicRamp::ParabolicRampND> ramps;
    for(dReal testcoef=1; testcoef<=upperbound; testcoef+=stepsize) {
        bool canscale = ScaleRampsTime(origramps,ramps,testcoef,true,params);
        if(!canscale) {
            continue;
        }
        bool res = IterativeMergeRampsFixedTime(ramps, resramps, params, checkcontrollertime, uniformsampler);
        res = res && (!docheck || CheckRamps(resramps,check));
        if(res) {
            RAVELOG_DEBUG_FORMAT("Timescale coefficient: %f\n",testcoef);
            return true;
        }
    }
    return false;
}

// Break ramps into unitary ramps before checking each unitary ramp
// This is necessary because of the strange behavior of check (probably caused by sampling along parabolic segments)
bool checkunitary(const ParabolicRamp::ParabolicRampND& rampnd, ParabolicRamp::RampFeasibilityChecker& check){
    std::list<ParabolicRamp::ParabolicRampND> tmpramps;
    tmpramps.push_back(rampnd);
    BreakIntoUnitaryRamps(tmpramps);
    FOREACHC(itramp, tmpramps) {
        if(!check.Check(*itramp)) {
            return false;
        }
    }
    return true;
}

bool ComputeStraightRamp(ParabolicRamp::ParabolicRampND& newramp,const ParabolicRamp::Vector x0, const ParabolicRamp::Vector x1, ConstraintTrajectoryTimingParametersPtr params,ParabolicRamp::RampFeasibilityChecker& check){
    ParabolicRamp::Vector zero(x0.size(),0.0);
    newramp.x0 = x0;
    newramp.x1 = x1;
    newramp.dx0 = zero;
    newramp.dx1 = zero;
    if(params->minswitchtime==0) {
        bool res=newramp.SolveMinTimeLinear(params->_vConfigAccelerationLimit, params->_vConfigVelocityLimit);
        return res && newramp.IsValid();
    }
    else {
        // retime linear segments so that they satisfy the minswitch time condition the _fStepLength condition, and the dynamics condition (e.g. torque limits)
        // note that the collision condition should be satisfied before entering this function
        dReal hi = 1;
        dReal lo = 0;
        dReal coef = hi;
        ParabolicRamp::ParabolicRampND newramp2 = newramp;
        while(hi-lo>0.01) {
            std::vector<dReal> amax;
            size_t n = params->_vConfigAccelerationLimit.size();
            amax.resize(n);
            for(size_t j=0; j<n; j++) {
                amax[j]=params->_vConfigAccelerationLimit[j]*coef;
            }
            bool res=newramp2.SolveMinTimeLinear(amax,params->_vConfigVelocityLimit);
            if(res && DetermineMinswitchtime(newramp2)>=params->minswitchtime && CountUnitaryRamps(newramp2)<=2 && checkunitary(newramp2,check)) {
                newramp = newramp2;
                if(coef == hi) {
                    break;
                }
                lo = coef;
            }
            else{
                hi = coef;
            }
            coef = (hi+lo)/2;
        }
        bool resfinal = newramp.IsValid() && DetermineMinswitchtime(newramp) >= params->minswitchtime && CountUnitaryRamps(newramp)<=2 && checkunitary(newramp,check);
        return resfinal;
    }
}


bool iszero(const ParabolicRamp::Vector& v){
    FOREACHC(itv,v){
        if (RaveFabs(*itv) > TINY) {
            return false;
        }
    }
    return true;
}

bool formstraightline(ParabolicRamp::ParabolicRampND& ramp0,ParabolicRamp::ParabolicRampND& ramp1){
    dReal dotproduct=0, x0length2=0, x1length2=0;
    for(size_t i = 0; i < ramp0.x0.size(); ++i) {
        dReal dx0=ramp0.x1[i]-ramp0.x0[i];
        dReal dx1=ramp1.x1[i]-ramp1.x0[i];
        dotproduct += dx0*dx1;
        x0length2 += dx0*dx0;
        x1length2 += dx1*dx1;
    }
    return RaveFabs(dotproduct*dotproduct - x0length2*x1length2) <= TINY;
}


bool FixRampsEnds(std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check){
    if (origramps.size()<2) {
        return false;
    }
    bool isbeg = true;
    std::list<ParabolicRamp::ParabolicRampND>::iterator itramp0 = origramps.begin(),itramp1,itramp2;
    // Check whether the first two ramps come from a jittering operation
    itramp1 = itramp0;
    itramp1++;
    if (iszero(itramp1->dx1) && formstraightline(*itramp0,*itramp1)) {
        RAVELOG_DEBUG("First two ramps come from a jittering operation\n");
    }
    else{
        // Check whether the last two ramps come from a jittering operation
        itramp1 = origramps.end();
        itramp1--;
        itramp0 = itramp1;
        itramp0--;
        if (iszero(itramp1->dx1) && formstraightline(*itramp0,*itramp1)) {
            RAVELOG_DEBUG("Last two ramps come from a jittering operation\n");
            isbeg = false;
        }
        else{
            RAVELOG_WARN("This trajectory does not seem to come from a previous jittering...\n");
            return false;
        }
    }
    // Now make straight ramps with good time durations out of these two ramps
    ParabolicRamp::ParabolicRampND newramp;
    bool res = ComputeStraightRamp(newramp,itramp0->x0,itramp1->x1,params,check);
    if(!res) {
        RAVELOG_DEBUG("Could not make straight ramps out of the two ramps\n");
        return false;
    }
    std::list<ParabolicRamp::ParabolicRampND> tmpramps0, tmpramps1;
    tmpramps0.push_back(newramp);
    BreakIntoUnitaryRamps(tmpramps0);
    bool canscale = ScaleRampsTime(tmpramps0,tmpramps1,ComputeStepSizeCeiling(newramp.endTime,params->_fStepLength*2)/newramp.endTime,false,params);
    if(!canscale) {
        RAVELOG_DEBUG("Could not make straight ramps out of the two ramps\n");
        return false;
    }
    // Insert in the original trajectory
    resramps.resize(0);
    if(isbeg) {
        // Insert at the beginning
        resramps.splice(resramps.begin(),tmpramps1);
        itramp0 = origramps.begin();
        advance(itramp0,2);
        while(itramp0!=origramps.end()) {
            resramps.push_back(*itramp0);
            itramp0++;
        }
    }
    else{
        // Insert at the end
        itramp0 = origramps.begin();
        while(itramp0!=origramps.end()) {
            itramp1=itramp0;
            itramp1++;
            itramp2=itramp1;
            itramp2++;
            if(itramp1!=origramps.end() && itramp2!=origramps.end()) {
                resramps.push_back(*itramp0);
            }
            itramp0 = itramp1;
        }
        resramps.splice(resramps.end(),tmpramps1);
    }
    // Final check to make sure everything is right
    BreakIntoUnitaryRamps(resramps);
    std::list<dReal> desireddurations;
    desireddurations.resize(0);
    FOREACHC(itramp, resramps) {
        desireddurations.push_back(ComputeStepSizeCeiling(itramp->endTime,params->_fStepLength));
    }
    //PrintRamps(resramps,params,false);
    bool res2 = IterativeFixRamps(resramps,desireddurations,params);
    if( res2 && DetermineMinswitchtime(resramps) >= params->minswitchtime) {
        RAVELOG_DEBUG("Cool: First or last two ramps could be fixed\n");
        return true;
    }
    else{
        RAVELOG_DEBUG("Shit: First or last two ramps could not be fixed, try something more general...\n");
        return false;
    }
}




dReal DetermineMinswitchtime(const ParabolicRamp::ParabolicRampND& rampnd){
    vector<dReal> vswitchtimes;
    vswitchtimes.resize(0);
    vswitchtimes.push_back(rampnd.endTime);
    FOREACHC(itramp,rampnd.ramps) {
        vector<dReal>::iterator it;
        if( itramp->tswitch1 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
            if( *it != itramp->tswitch1) {
                vswitchtimes.insert(it,itramp->tswitch1);
            }
        }
        if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
            if( *it != itramp->tswitch2 ) {
                vswitchtimes.insert(it,itramp->tswitch2);
            }
        }
    }
    dReal tbeg,tend;
    tbeg = 0;
    dReal res = INF;
    for(size_t i=0; i<vswitchtimes.size(); i++) {
        tend = vswitchtimes[i];
        res = min(res,tend-tbeg);
        tbeg = tend;
    }
    return res;
}

dReal DetermineMinswitchtime(const std::list<ParabolicRamp::ParabolicRampND>&ramps){
    dReal mintime = 1e10;
    FOREACHC(itramp,ramps) {
        mintime=min (DetermineMinswitchtime(*itramp),mintime);
    }
    return mintime;
}

size_t CountUnitaryRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps){
    size_t nbunitramps = 0;
    FOREACHC(itramp,ramps){
        nbunitramps += CountUnitaryRamps(*itramp);
    }
    return nbunitramps;
}

size_t CountUnitaryRamps(const ParabolicRamp::ParabolicRampND& rampnd){
    vector<dReal> vswitchtimes;
    vswitchtimes.resize(0);
    vswitchtimes.push_back(rampnd.endTime);
    FOREACHC(itramp,rampnd.ramps) {
        vector<dReal>::iterator it;
        if( itramp->tswitch1 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
            if( *it != itramp->tswitch1) {
                vswitchtimes.insert(it,itramp->tswitch1);
            }
        }
        if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
            if( *it != itramp->tswitch2 ) {
                vswitchtimes.insert(it,itramp->tswitch2);
            }
        }
    }
    size_t result=0;
    dReal tbeg,tend;
    tbeg = 0;
    for(size_t i=0; i<vswitchtimes.size(); i++) {
        ParabolicRamp::Vector q0,v0,q1,v1;
        tend = vswitchtimes[i];
        //printf("vswitch[i] = %f\n",tend);
        //Throw away ramps that have tiny time durations
        if(tend-tbeg>TINY) {
            result++;
        }
        tbeg = tend;
    }
    return result;
}

dReal ComputeRampsDuration(const std::list<ParabolicRamp::ParabolicRampND>&ramps){
    dReal res=0;
    FOREACH(itramp, ramps) {
        res += itramp->endTime;
    }
    return res;
}

// For logging purpose
void PrintRamps(const std::list<ParabolicRamp::ParabolicRampND>&ramps,ConstraintTrajectoryTimingParametersPtr params,bool checkcontrollertimestep){
    int itx = 0;
    dReal totaltime = 0;
    FOREACH(itramp, ramps) {
        totaltime += itramp->endTime;
        string m = "-";
        if(itramp->modified) {
            m = "M";
        }
        dReal ratio = itramp->endTime/params->_fStepLength;
        RAVELOG_DEBUG_FORMAT("Ramp %d: |%s|%f|%f; ",itx%m%itramp->endTime%ratio);
        if(checkcontrollertimestep) {
            if(!IsMultipleOfStepSize(itramp->endTime,params->_fStepLength)) {
                RAVELOG_WARN("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
                dReal T = itramp->endTime;
                dReal step = params->_fStepLength;
                dReal ratio = T/step;
                dReal ceilratio = ceil(ratio);
                RAVELOG_WARN_FORMAT("Ratio= %d, CeilRatio= %d\n",ratio%ceilratio);
            }
        }
        itx++;
    }
    RAVELOG_DEBUG_FORMAT("Total time duration: %f\n",totaltime);

}



/** Break one ramp into unitary ramps. A unitary ramp consists of a single acceleration
    \param ramp input ramp
    \param resramp result ramp
 */
void BreakOneRamp(ParabolicRamp::ParabolicRampND ramp,std::list<ParabolicRamp::ParabolicRampND>&resramp){
    vector<dReal> vswitchtimes;
    vswitchtimes.resize(0);
    vswitchtimes.push_back(ramp.endTime);
    FOREACHC(itramp,ramp.ramps) {
        vector<dReal>::iterator it;
        if( itramp->tswitch1 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
            if( *it != itramp->tswitch1) {
                vswitchtimes.insert(it,itramp->tswitch1);
            }
        }
        if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
            if( *it != itramp->tswitch2 ) {
                vswitchtimes.insert(it,itramp->tswitch2);
            }
        }
    }
    dReal tbeg,tend;
    resramp.resize(0);
    tbeg = 0;
    for(size_t i=0; i<vswitchtimes.size(); i++) {
        ParabolicRamp::Vector q0,v0,q1,v1;
        tend = vswitchtimes[i];
        // Discard ramps that have tiny time durations
        if(tend-tbeg>g_fEpsilonLinear) {
            ramp.Evaluate(tbeg,q0);
            ramp.Derivative(tbeg,v0);
            ramp.Evaluate(tend,q1);
            ramp.Derivative(tend,v1);
            resramp.push_back(ParabolicRamp::ParabolicRampND());
            resramp.back().SetPosVelTime(q0,v0,q1,v1,(tend-tbeg));
            resramp.back().modified = ramp.modified;
            PARABOLIC_RAMP_ASSERT(resramp.back().IsValid());
        }
        tbeg = tend;
    }
}

/** Break ramps list into unitary ramps list
    \param ramps input ramp
    \param resramps result ramp
 */
void BreakIntoUnitaryRamps(std::list<ParabolicRamp::ParabolicRampND>& ramps){
    std::list<ParabolicRamp::ParabolicRampND> tmpramps,resramps;
    FOREACHC(itramp,ramps){
        BreakOneRamp(*itramp,tmpramps);
        resramps.splice(resramps.end(),tmpramps);
    }
    ramps.swap(resramps);
}


} // end namespace mergewaypoints


