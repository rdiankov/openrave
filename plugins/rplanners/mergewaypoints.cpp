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
#include "openraveplugindefs.h"
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

Interval::Interval()
{
    isVoid = true;
}

Interval::Interval(dReal a, dReal b)
{
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
Interval SolveIneq(dReal a,dReal b)
{
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

/** Check the velocity, acceleration, and joint limits of two neighboring ramps
   \param Ta, Tb time durations of ramp 1 and 2 respectively
   \param q0 the configuration at the start of ramp 1
   \param v0 the velocity at the start of ramp 1
   \param q2 the configuration at the end of ramp 2
   \param v2 the velocity at the end of ramp 2
 */
bool CheckValidity(dReal Ta,dReal Tb,const std::vector<dReal>& q0,const std::vector<dReal>& v0,const std::vector<dReal>& q2,const std::vector<dReal>& v2,std::vector<dReal>& qres,std::vector<dReal>& vres,ConstraintTrajectoryTimingParametersPtr params)
{

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
bool CheckMerge(dReal T0,dReal T1,dReal T2,const std::vector<dReal>& q0,const std::vector<dReal>& v0,const std::vector<dReal>& q3,const std::vector<dReal>& v3,dReal& alpha,std::vector<dReal>& qres,std::vector<dReal>& vres,ConstraintTrajectoryTimingParametersPtr params)
{
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

    // This checks for joint angle limits
    // Velocity and Accel limits should be satisfied from the interval computation
    return CheckValidity(Ta,Tb,q0,v0,q3,v3,qres,vres,params);
}


/** Fix the time durations of two neighboring ramps. Assume that the ramps are unitary
    \param rampx input ramps with x=0,1,
    \param resrampx result ramp with x=0,1
    \param Ta,Tb desired final durations
 */
bool FixRamps(const ParabolicRamp::ParabolicRampND& ramp0,const ParabolicRamp::ParabolicRampND& ramp1,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1, dReal Ta, dReal Tb, ConstraintTrajectoryTimingParametersPtr params)
{
    vector<dReal> q0 = ramp0.x0, v0=ramp0.dx0, q2=ramp1.x1, v2=ramp1.dx1, qres, vres;
    //dReal T0,T1;
    //T0 = ramp0.endTime;
    //T1 = ramp1.endTime;

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



/** Merge three ramps into two ramps while respecting the continuities of joints and velocities on the borders
   Assume that the ramps are unitary
    \param rampx input ramps with x=0,1,2
    \param resrampx result ramp with x=0,1
 */
bool MergeRamps(const ParabolicRamp::ParabolicRampND& ramp0, const ParabolicRamp::ParabolicRampND& ramp1, const ParabolicRamp::ParabolicRampND& ramp2,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1,ConstraintTrajectoryTimingParametersPtr params)
{
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
int factorial(int n)
{
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

/** Iteratively fix the time durations of the ramps
   Note that, by construction, there is no isolated modified ramps
   \param ramps input ramps
   \param desireddurations list of desired durations
 */
bool IterativeFixRamps(std::list<ParabolicRamp::ParabolicRampND>& ramps, std::list<dReal>& desireddurations, ConstraintTrajectoryTimingParametersPtr params)
{
    OPENRAVE_ASSERT_OP(ramps.size(),==,desireddurations.size());
    if( ramps.size() == 0) {
        return false;
    }
    if (ramps.size() == 1) {
        std::list<ParabolicRamp::ParabolicRampND> resramps;
        dReal coef = desireddurations.back()/ramps.back().endTime;
        // make sure trysmart is false, or otherwise can get into infinite loop
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

    int counter = 0;
    while(itrampprev != ramps.end() && itrampnext != ramps.end() && counter<1000) {
        counter++;
        if(RaveFabs(itrampprev->endTime-*ittprev)>TINY || RaveFabs(itrampnext->endTime-*ittnext)>TINY) {
            // Avoid using an unmodified ramp to fix durations
            if (RaveFabs(itrampprev->endTime-*ittprev)<=TINY && !itrampprev->modified) {
                itrampprev = itrampnext++;
                ittprev = ittnext++;
                if(itrampnext != ramps.end()) {
                    // force modification of itrampnext at next step (to avoid infinite loop)
                    itrampnext->modified = true;
                }
                continue;
            }
            // actually this is necessary because we move forward by two ramps (see below)
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
        // move forward by two ramps
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

    if(counter>=1000) {
        RAVELOG_ERROR("counter exceeded, most likely a bug in the code\n");
        return false;
    }

    return true;
}

/** Iteratively kill all ramps that are shorter than minswitchtime by merging them into neighboring ramps. Rounds all times to the controller timestep. Does not change the global time duration too much
    \param origramps input ramps
    \param ramps result ramps
    \param v3 the velocity at the end of the ramp at T2
 */
bool IterativeMergeRampsFixedTime(const std::list<ParabolicRamp::ParabolicRampND>& origramps, std::list<ParabolicRamp::ParabolicRampND>& ramps, ConstraintTrajectoryTimingParametersPtr params, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler)
{
    // Determine the number max of iterations as a function of the number of short ramps
    size_t i = 0;
    int nb_short_ramps = 0;
    FOREACHC(itramp,origramps) {
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
        // Kill all ramps that are not the first or the last. For every three ramps where the middle ramp is smaller than minswitchtime, merge the ramp that is in the middle to form two new ramps.
        bool solved = false;
        while(true) {
            solved = true;
            size_t i = 0;
            FOREACHC(itramp,ramps) {
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
            FOREACHC(itramp,ramps) {
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
        // Handle the first ramps in the trajectory
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
        // Handle the last ramps in the trajectory
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

bool ScaleRampsTime(const std::list<ParabolicRamp::ParabolicRampND>& origramps, std::list<ParabolicRamp::ParabolicRampND>& ramps, dReal coef, bool trysmart, ConstraintTrajectoryTimingParametersPtr params)
{
    ramps.resize(0);
    bool doscale = RaveFabs(coef-1)>TINY;

    // Try being smart by timescaling only the modified ramps
    if(trysmart && doscale && origramps.size()>=2) {
        dReal durationbeforescaling = ComputeRampsDuration(origramps);
        dReal durationmodifiedramps = 0;
        FOREACH(itramp,origramps) {
            if(itramp->modified) {
                durationmodifiedramps += itramp->endTime;
            }
        }
        if(durationmodifiedramps>TINY) {
            coef = 1+ (coef-1)*durationbeforescaling/durationmodifiedramps;
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

    // If scaling is necessary but either the initial or the final trajectory is nonzero, then return false
    if(doscale && !(CheckIfZero(origramps.begin()->dx0, ParabolicRamp::EpsilonV) && CheckIfZero(origramps.back().dx1, ParabolicRamp::EpsilonV))) {
        return false;
    }

    // Time scale the whole trajectory
    FOREACH(itramp,origramps) {
        PARABOLIC_RAMP_ASSERT(itramp->IsValid());
        dReal t = itramp->endTime;
        if(t>TINY) {
            ParabolicRamp::Vector q0=itramp->x0, v0=itramp->dx0, q1=itramp->x1, v1=itramp->dx1;
            if(doscale) {
                dReal invcoef = 1/coef;
                FOREACH(itv,v0) {
                    *itv *= invcoef;
                }
                FOREACH(itv,v1) {
                    *itv *= invcoef;
                }
            }
            ramps.push_back(ParabolicRamp::ParabolicRampND());
            ramps.back().SetPosVelTime(q0,v0,q1,v1,t*coef);
            ramps.back().modified = itramp->modified || doscale;
            BOOST_ASSERT(ramps.back().IsValid());
        }
    }
    return true;
}

// Check whether ramps satisfy constraints associated with checker
bool CheckRamps(std::list<ParabolicRamp::ParabolicRampND>&ramps, ParabolicRamp::RampFeasibilityChecker& check,int options = 0xffff)
{
    FOREACHC(itramp,ramps) {
        if(!itramp->IsValid() || !check.Check(*itramp,options)) {
            return false;
        }
    }
    return true;
}

bool SpecialCheckRamp(const ParabolicRamp::ParabolicRampND& ramp,const ParabolicRamp::Vector& qstart, const ParabolicRamp::Vector& qgoal, dReal radius, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check, int options)
{

    dReal dt = 0.01;
    dReal rampduration = ramp.endTime;
    int options_perturb = options | CFO_CheckWithPerturbation;
    ParabolicRamp::Vector q,qm,vm;
    ParabolicRamp::ParabolicRampND ramp1, ramp2;

    //Start of ramp is close to qstart
    if(params->_distmetricfn(ramp.x0,qstart)<radius) {
        if(rampduration<dt) {
            if(params->_distmetricfn(ramp.x1,qstart)>radius) {
                return check.Check(ramp,options_perturb);
            }
            else{
                return check.Check(ramp,options);
            }
        }
        dReal T=dt;
        while(T<=rampduration) {
            ramp.Evaluate(T,q);
            if(params->_distmetricfn(q,qstart)>radius) {
                break;
            }
            T += dt;
        }
        if(T>=ramp.endTime-g_fEpsilonLinear) {
            return check.Check(ramp,options);
        }
        ramp.Evaluate(T,qm);
        ramp.Derivative(T,vm);
        ramp1.SetPosVelTime(ramp.x0,ramp.dx0,qm,vm,T);
        ramp2.SetPosVelTime(qm,vm,ramp.x1,ramp.dx1,rampduration-T);
        return check.Check(ramp1,options) && check.Check(ramp2,options_perturb);
    }

    //End of ramp is close to qgoal
    if(params->_distmetricfn(ramp.x1,qgoal)<radius) {
        if(rampduration<dt) {
            if(params->_distmetricfn(ramp.x0,qgoal)>radius) {
                return check.Check(ramp,options_perturb);
            }
            else{
                return check.Check(ramp,options);
            }
        }
        dReal T=rampduration-dt;
        while(T>=0) {
            ramp.Evaluate(T,q);
            if(params->_distmetricfn(q,qgoal)>radius) {
                break;
            }
            T -= dt;
        }
        if(T<=g_fEpsilonLinear) {
            return check.Check(ramp,options);
        }
        ramp.Evaluate(T,qm);
        ramp.Derivative(T,vm);
        ramp1.SetPosVelTime(ramp.x0,ramp.dx0,qm,vm,T);
        ramp2.SetPosVelTime(qm,vm,ramp.x1,ramp.dx1,rampduration-T);
        return check.Check(ramp1,options_perturb) && check.Check(ramp2,options);
    }

    // Else check the entire ramp with perturbations
    return check.Check(ramp,options_perturb);
}


// dReal dist(const ParabolicRamp::Vector& q1, const ParabolicRamp::Vector& q2){
//     int n = q1.size();
//     dReal d = 0;
//     for(int i = 0; i<n; i++) {
//         d += pow(q2[i]-q1[i],2);
//     }
//     return sqrt(d);
// }


// bool SpecialCheckRamp(const ParabolicRamp::ParabolicRampND& ramp, int position, ParabolicRamp::Vector& qref, dReal radius, ParabolicRamp::RampFeasibilityChecker& check, int options){
//     int options_noperturb = options & (~CFO_CheckWithPerturbation);
//     //Beginning of traj
//     if(position==1) {
//         if(dist(ramp.x0,qref)<radius) {
//             return check.Check(ramp,options_noperturb);
//         }
//         else{
//             return check.Check(ramp,options);
//         }
//     }
//     //End of traj
//     else if (position==-1) {
//         if(dist(ramp.x1,qref)<radius) {
//             return check.Check(ramp,options_noperturb);
//         }
//         else{
//             return check.Check(ramp,options);
//         }
//     }
//     else{
//         return check.Check(ramp,options);
//     }
// }



dReal ComputeRampQuality(const std::list<ParabolicRamp::ParabolicRampND>& ramps)
{
    dReal res=0;
    FOREACHC(itramp,ramps) {
        dReal duration = itramp->endTime;
        if( duration <= ParabolicRamp::EpsilonT ) {
            RAVELOG_WARN("ramp has very small duration!");
            return 0;
        }
        else {
            res += 1/(duration*duration);
        }
    }
    return 1/res;
}

bool FurtherMergeRamps(const std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, int options)
{
    //int nitersfurthermerge = params->nitersfurthermerge;
    int nitersfurthermerge = 0;
    resramps = origramps;
    bool bHasChanged = false;
    std::list<ParabolicRamp::ParabolicRampND> ramps;
    dReal origrampsduration = ComputeRampsDuration(origramps);
    for(int rep = 0; rep<nitersfurthermerge; rep++) {
        ramps = origramps;
        while(ramps.size()>=3) {
            std::list<ParabolicRamp::ParabolicRampND>::iterator itlist = ramps.begin();
            int randidx = uniformsampler->SampleSequenceOneUInt32()%(ramps.size()-2);
            std::advance(itlist,randidx);
            ParabolicRamp::ParabolicRampND ramp0,ramp1,ramp2,resramp0,resramp1,resramp0x,resramp1x;
            ramp0 = *itlist++;
            ramp1 = *itlist++;
            ramp2 = *itlist++;
            bool resmerge = MergeRamps(ramp0,ramp1,ramp2,resramp0,resramp1,params);
            if(!resmerge) {
                break;
            }
            dReal t0 = ComputeStepSizeCeiling(resramp0.endTime,params->_fStepLength);
            dReal t1 = ComputeStepSizeCeiling(resramp1.endTime,params->_fStepLength);
            if(t1+t0-resramp0.endTime-resramp1.endTime+ComputeRampsDuration(ramps)>upperbound*origrampsduration) {
                break;
            }
            bool resfix = FixRamps(resramp0,resramp1,resramp0x,resramp1x,t0,t1,params);
            if(!resfix || !check.Check(resramp0x,options) || !check.Check(resramp1x,options)) {
                break;
            }
            bHasChanged = true;
            itlist--;
            itlist--;
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            itlist = ramps.erase(itlist);
            ramps.insert(itlist,resramp1x);
            itlist--;
            ramps.insert(itlist,resramp0x);
        }
        //PrintRamps(ramps,params,true);
        if(ramps.size()<resramps.size() || (ramps.size()==resramps.size()&& ComputeRampQuality(ramps)>ComputeRampQuality(resramps))) {
            resramps = ramps;
        }
    }
    //PrintRamps(resramps,params,true);
    return bHasChanged;
}

bool IterativeMergeRamps(const std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, int options)
{
    std::list<ParabolicRamp::ParabolicRampND> ramps,ramps2;
    dReal testcoef;

    //printf("Coef = 1\n");
    bool res = IterativeMergeRampsFixedTime(origramps, ramps2, params, checkcontrollertime, uniformsampler);
    res = res &&  CheckRamps(ramps2,check,options);
    if (res) {
        resramps.swap(ramps2);
        return true;
    }
    dReal durationbeforemerge = ComputeRampsDuration(origramps);
    dReal maxcoef = upperbound;
    //printf("Coef = %f\n",maxcoef);
    bool canscale = ScaleRampsTime(origramps, ramps, maxcoef, true, params);
    if(!canscale) {
        return false;
    }
    res = IterativeMergeRampsFixedTime(ramps, ramps2, params, checkcontrollertime, uniformsampler);
    res = res && CheckRamps(ramps2,check,options);
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
        res = res && CheckRamps(ramps2,check,options);
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


bool IterativeMergeRampsNoDichotomy(const std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, dReal upperbound, dReal stepsize, bool checkcontrollertime, SpaceSamplerBasePtr uniformsampler, ParabolicRamp::RampFeasibilityChecker& check, int options)
{
    std::list<ParabolicRamp::ParabolicRampND> ramps;
    for(dReal testcoef=1; testcoef<=upperbound; testcoef+=stepsize) {
        bool canscale = ScaleRampsTime(origramps,ramps,testcoef,true,params);
        if(!canscale) {
            continue;
        }
        bool res = IterativeMergeRampsFixedTime(ramps, resramps, params, checkcontrollertime, uniformsampler);
        res = res && CheckRamps(resramps,check,options);
        if(res) {
            RAVELOG_DEBUG_FORMAT("Timescale coefficient: %f succeeded\n",testcoef);
            return true;
        }
    }
    return false;
}

// Add two Ramp Vectors (not sure whether it has been implemented somewhere else)
ParabolicRamp::Vector ScaleVector(ParabolicRamp::Vector a,dReal ca){
    ParabolicRamp::Vector res(a.size(),0.0);
    for(size_t i=0; i<a.size(); i++) {
        res[i] = ca*a[i];
    }
    return res;
}

// Add two Ramp Vectors (not sure whether it has been implemented somewhere else)
ParabolicRamp::Vector AddVectors(ParabolicRamp::Vector a,ParabolicRamp::Vector b,dReal ca,dReal cb){
    ParabolicRamp::Vector res(a.size(),0.0);
    for(size_t i=0; i<a.size(); i++) {
        res[i] = ca*a[i]+cb*b[i];
    }
    return res;
}


bool ComputeLinearRampsWithConstraints(std::list<ParabolicRamp::ParabolicRampND>& resramps, const ParabolicRamp::Vector x0, const ParabolicRamp::Vector x1, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check,int options)
{
    ParabolicRamp::Vector zero(x0.size(),0.0);
    ParabolicRamp::Vector dx = AddVectors(x1,x0,1,-1);

    dReal vmaxs = INF, amaxs = INF;
    dReal delta = params->minswitchtime;
    int numdof = params->GetDOF();
    for(int i=0; i<numdof; i++) {
        vmaxs = min(vmaxs,params->_vConfigVelocityLimit[i]/RaveFabs(dx[i]));
        amaxs = min(amaxs,params->_vConfigAccelerationLimit[i]/RaveFabs(dx[i]));
    }

    dReal tp = sqrt(1/amaxs);

    // Two ramps are OK
    if(amaxs*tp <= vmaxs) {
        dReal t_,v_;
        if(tp >= delta) {
            t_ = tp;
            v_ = amaxs*tp;
        }
        else{
            t_ = delta;
            v_ = 1/delta;
        }
        // Create two new ramps and insert into resramps
        ParabolicRamp::ParabolicRampND newramp1, newramp2;
        ParabolicRamp::Vector xmid = AddVectors(x0,x1,0.5,0.5);
        ParabolicRamp::Vector vinter = ScaleVector(dx,v_);
        newramp1.SetPosVelTime(x0,zero,xmid,vinter,t_);
        newramp2.SetPosVelTime(xmid,vinter,x1,zero,t_);
        resramps.push_back(newramp1);
        resramps.push_back(newramp2);
    }

    // Require three ramps
    else{
        dReal t1 = vmaxs/amaxs;
        dReal t2 = 1/vmaxs - vmaxs/amaxs;
        dReal t1_,t2_,a_;
        if(t1 >= delta && t2 >= delta) {
            t1_ = t1;
            t2_ = t2;
            a_ = amaxs;
        }
        else{
            // Try first case delta,delta,delta
            if(1/delta < vmaxs && 1/(delta*delta)<amaxs) {
                t1_ = delta;
                t2_ = delta;
                a_ = 1/(delta*delta);
            }
            else{
                // case delta,ta,delta
                dReal tmpmax = max(delta,1/vmaxs-delta);
                dReal ta = max(tmpmax,1/(delta*amaxs)-delta);
                // case tb,delta,tb
                dReal tb = max(tmpmax,(sqrt(delta*delta+4/amaxs)-delta)/2);
                if(ta+delta*2<tb*2+delta) {
                    t1_ = delta;
                    t2_ = ta;
                    a_ = 1/(delta*delta+delta*ta);
                }
                else{
                    t1_ = tb;
                    t2_ = delta;
                    a_ = 1/(tb*tb+tb*delta);
                }
            }
        }
        // Create three new ramps and insert into resramps
        ParabolicRamp::ParabolicRampND newramp1, newramp2, newramp3;
        dReal v_ = a_*t1_;
        dReal s1 = 0.5*a_*t1_*t1_;
        dReal s2 = s1+v_*t2_;
        ParabolicRamp::Vector xinter1 = AddVectors(x0,dx,1,s1);
        ParabolicRamp::Vector xinter2 = AddVectors(x0,dx,1,s2);
        ParabolicRamp::Vector vinter = ScaleVector(dx,v_);
        newramp1.SetPosVelTime(x0,zero,xinter1,vinter,t1_);
        newramp2.SetPosVelTime(xinter1,vinter,xinter2,vinter,t2_);
        newramp3.SetPosVelTime(xinter2,vinter,x1,zero,t1_);
        resramps.push_back(newramp1);
        resramps.push_back(newramp2);
        resramps.push_back(newramp3);
    }

    // Last check just to make sure
    return resramps.size() > 0 && DetermineMinswitchtime(resramps) >= delta && CheckRamps(resramps,check,options);
}



bool ComputeLinearRampsWithConstraints2(std::list<ParabolicRamp::ParabolicRampND>& resramps, const ParabolicRamp::Vector x0, const ParabolicRamp::Vector x1, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check,int options)
{
    ParabolicRamp::Vector zero(x0.size(),0.0);
    ParabolicRamp::ParabolicRampND newramp;
    std::list<ParabolicRamp::ParabolicRampND> tmpramps;
    newramp.x0 = x0;
    newramp.x1 = x1;
    newramp.dx0 = zero;
    newramp.dx1 = zero;
    bool solved = false;
    std::vector<dReal> amax;
    int numdof = params->GetDOF();
    amax.resize(numdof);

    // Iteratively timescales up until the time-related constraints are satisfied
    // only scale the acceleration limits and don't touch the velocity limits
    // the reason is so that the resulting ramps can always be accelerating without saturating the velocity. this reduces the number of ramps and makes minswitchtime guarantee simpler.
    dReal small = 1e-6;
    dReal hi = 1;
    dReal lo = small;
    dReal coef = 0;  // coefficient multiplying the acceleration limits: if coef is small, traj duration will be larger
    int iter = 0;
    resramps.resize(0);
    while(hi-lo>1e-3 && iter<1000) {
        iter++;
        if(iter==1) {
            coef = 1;
        }
        if(iter==2) {
            coef = small;
        }
        for(int j=0; j<numdof; j++) {
            amax[j]=params->_vConfigAccelerationLimit[j]*coef;
        }
        bool res = newramp.SolveMinTimeLinear(amax,params->_vConfigVelocityLimit);
        if(!res) {
            if(coef <= small) {
                RAVELOG_DEBUG("Quasi-static traj failed (solvemintime), stops ComputeLinearRamps right away\n");
                return false;
            }
            hi = coef;
        }
        else {
            tmpramps.resize(0);
            tmpramps.push_back(newramp);
            BreakIntoUnitaryRamps(tmpramps);
            if(!(DetermineMinswitchtime(tmpramps)>=params->minswitchtime && CountUnitaryRamps(tmpramps)<=2 && CheckRamps(tmpramps,check,options))) {
                if(coef <= small) {
                    RAVELOG_INFO("Quasi-static traj failed (check), stops ComputeLinearRamps right away\n");
                    // RaveSetDebugLevel(Level_Verbose);
                    // CheckRamps(tmpramps,check,options);
                    // RaveSetDebugLevel(Level_Debug);
                    return false;
                }
                hi = coef;
            }
            else{
                lo = coef;
                resramps = tmpramps;
                solved = true;
                if(coef >= 1) {
                    break;
                }
            }
        }
        coef = (hi+lo)/2;
    }
    return solved && resramps.size() > 0 && DetermineMinswitchtime(resramps) >= params->minswitchtime && CountUnitaryRamps(resramps)<=2 && CheckRamps(resramps,check,options);
}


bool ComputeQuadraticRampsWithConstraints(std::list<ParabolicRamp::ParabolicRampND>& resramps, const ParabolicRamp::Vector x0, const ParabolicRamp::Vector dx0, const ParabolicRamp::Vector x1, const ParabolicRamp::Vector dx1, dReal fOriginalTrajectorySegmentTime, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check, int options)
{
    std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > tmpramps1d;
    std::list<ParabolicRamp::ParabolicRampND> tmpramps;
    dReal mintime;
    bool solved = false;
    int numdof = params->GetDOF();
    std::vector<dReal> amax, vmax = params->_vConfigVelocityLimit;
    amax.resize(numdof);

    // Iteratively timescales up until the time-related constraints are met
    const dReal small = 1e-3;
    dReal hi = 1;
    dReal lo = small;
    dReal coef = 0;  // coefficient multiplying the limits: if coef is small, traj duration will be larger
    int iter = 0;
    // TODO: cache max(RaveFabs(dx0[j]),RaveFabs(dx1[j]))
    while(hi-lo>small && iter<100) {
        iter++;
        if(iter==1) {
            coef = 1;
        }
        if(iter==2) {
            coef = small;
        }
        //RAVELOG_VERBOSE("Coef: %d\n", coef);
        for(int j=0; j<numdof; j++) {
            if(params->maxmanipspeed>0) {
                vmax[j]=max(params->_vConfigVelocityLimit[j]*coef,max(RaveFabs(dx0[j]),RaveFabs(dx1[j])));
            }
            amax[j]=params->_vConfigAccelerationLimit[j]*coef;
        }
        mintime = ParabolicRamp::SolveMinTimeBounded(x0,dx0,x1,dx1, amax, vmax, params->_vConfigLowerLimit, params->_vConfigUpperLimit, tmpramps1d, params->_multidofinterp);
        if(mintime > fOriginalTrajectorySegmentTime) {
            //RAVELOG_VERBOSE("Too long\n");
            if(coef>=1) {
                RAVELOG_VERBOSE("Traj with time-scaling 1 (amax = acceleration_limit) has time duration > fOriginalTrajectorySegmentTime, so stopping ComputeQuadraticRamps right away\n");
                return false;
            }
            lo = coef;
        }
        else if (mintime <= TINY) {
            RAVELOG_VERBOSE_FORMAT("Coef %f could not Solvemintime, so stopping ComputeQuadraticRamps right away\n",coef);
            lo = coef;
        }
        else{
            tmpramps.resize(0);
            CombineRamps(tmpramps1d,tmpramps);
            BreakIntoUnitaryRamps(tmpramps);
            if(!CheckRamps(tmpramps,check,options)) {
                //RAVELOG_VERBOSE("Not OK\n");
                if(coef <= small) {
                    RAVELOG_VERBOSE("Super slow traj (amax = small*acceleration_limit) failed check, so stopping ComputeQuadraticRamps right away\n");
                    return false;
                }
                hi = coef;
            }
            else{
                lo = coef;
                resramps = tmpramps;
                //RAVELOG_VERBOSE("OK\n");
                solved = true;
                if(coef>=1) {
                    break;
                }
            }
        }
        coef = (hi+lo)/2;
    }
    return solved && resramps.size()>0 && (ComputeRampsDuration(resramps)<= fOriginalTrajectorySegmentTime) && CheckRamps(resramps,check,options);
}


bool CheckIfZero(const ParabolicRamp::Vector& v, dReal epsilon)
{
    FOREACHC(itv,v) {
        if (RaveFabs(*itv) > epsilon) {
            return false;
        }
    }
    return true;
}

/// check if the velocity dervatives are collinear with the difference of the ramp endpoints
/// assumes ramp is unitary
bool CheckIfRampIsStraight(const ParabolicRamp::ParabolicRampND& ramp)
{
    dReal dotproduct0=0,dotproduct1=0, diffclength2=0, diff0length2=0, diff1length2=0;
    for(size_t i = 0; i < ramp.x0.size(); ++i) {
        BOOST_ASSERT(ramp.ramps.at(i).tswitch1 <= g_fEpsilonLinear || ramp.ramps.at(i).tswitch1 >= ramp.endTime-g_fEpsilonLinear);
        BOOST_ASSERT(ramp.ramps.at(i).tswitch2 <= g_fEpsilonLinear || ramp.ramps.at(i).tswitch2 >= ramp.endTime-g_fEpsilonLinear);
        dReal diffc=ramp.x1.at(i) - ramp.x0.at(i);
        dReal diff0=ramp.dx0[i];
        dReal diff1=ramp.dx1[i];
        diffclength2 += diffc*diffc;
        diff0length2 += diff0*diff0;
        diff1length2 += diff1*diff1;
        dotproduct0 += diffc*diff0;
        dotproduct1 += diffc*diff1;
    }
    return RaveFabs(dotproduct0*dotproduct0 - diff0length2*diffclength2) <= g_fEpsilonLinear && RaveFabs(dotproduct1*dotproduct1 - diff1length2*diffclength2) <= g_fEpsilonLinear;
}

// Check whether ramp0 and ramp1 are collinear. Assumes that ramp0 and ramp1 are already linear themselves.
bool AreRampsCollinear(ParabolicRamp::ParabolicRampND& ramp0,ParabolicRamp::ParabolicRampND& ramp1) {
    if( !CheckIfRampIsStraight(ramp0) || !CheckIfRampIsStraight(ramp1) ) {
        return false;
    }
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

bool FixRampsEnds(std::list<ParabolicRamp::ParabolicRampND>&origramps,std::list<ParabolicRamp::ParabolicRampND>&resramps, ConstraintTrajectoryTimingParametersPtr params, ParabolicRamp::RampFeasibilityChecker& check, int options)
{
    if (origramps.size()<2) {
        return false;
    }
    bool jittered = false;
    std::list<ParabolicRamp::ParabolicRampND>::iterator itramp0 = origramps.begin(),itramp1,itramp2;

    if(!params->verifyinitialpath) {
        RAVELOG_WARN("Initial path verification is disabled (in FixRampsEnds)\n");
        options  = options & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions);
    }

    // Check whether the first two ramps come from a jittering operation
    itramp1 = itramp0;
    itramp1++;
    // TODO should have a general function for checking if a ramp is a straight line in configuration space. Every ramp should be tested with this function.
    if (CheckIfZero(itramp0->dx0, ParabolicRamp::EpsilonV) && CheckIfZero(itramp1->dx1, ParabolicRamp::EpsilonV) && AreRampsCollinear(*itramp0,*itramp1)) {
        RAVELOG_DEBUG("First two ramps probably come from a jittering operation\n");
        jittered = true;
        std::list<ParabolicRamp::ParabolicRampND> tmpramps0, tmpramps1;
        bool res = ComputeLinearRampsWithConstraints(tmpramps0,itramp0->x0,itramp1->x1,params,check,options);
        if(!res) {
            RAVELOG_WARN("Could not make straight ramps out of the two ramps\n");
            return false;
        }
        dReal tmpduration = ComputeRampsDuration(tmpramps0);
        bool canscale = ScaleRampsTime(tmpramps0,tmpramps1,ComputeStepSizeCeiling(tmpduration,params->_fStepLength*2)/tmpduration,false,params);
        if(!canscale) {
            RAVELOG_WARN("Could not round up to controller timestep\n");
            return false;
        }
        // Insert at beginning
        resramps.resize(0);
        resramps.splice(resramps.begin(),tmpramps1);
        itramp0 = origramps.begin();
        advance(itramp0,2);
        while(itramp0!=origramps.end()) {
            resramps.push_back(*itramp0);
            itramp0++;
        }
    }
    if(!jittered) {
        resramps = origramps;
    }

    // Check whether the last two ramps come from a jittering operation
    itramp1 = resramps.end();
    itramp1--;
    itramp0 = itramp1;
    itramp0--;
    if (CheckIfZero(itramp0->dx0, ParabolicRamp::EpsilonV) && CheckIfZero(itramp1->dx1, ParabolicRamp::EpsilonV) && AreRampsCollinear(*itramp0,*itramp1)) {
        jittered = true;
        RAVELOG_DEBUG("Last two ramps probably come from a jittering operation\n");
        std::list<ParabolicRamp::ParabolicRampND> tmpramps0, tmpramps1;
        bool res = ComputeLinearRampsWithConstraints(tmpramps0,itramp0->x0,itramp1->x1,params,check,options);
        if(!res) {
            RAVELOG_WARN("Could not make straight ramps out of the two ramps\n");
            return false;
        }
        dReal tmpduration = mergewaypoints::ComputeRampsDuration(tmpramps0);
        bool canscale = mergewaypoints::ScaleRampsTime(tmpramps0,tmpramps1,ComputeStepSizeCeiling(tmpduration,params->_fStepLength*2)/tmpduration,false,params);
        if(!canscale) {
            RAVELOG_WARN("Could not round up to controller timestep\n");
            return false;
        }
        // Insert at end
        std::list<ParabolicRamp::ParabolicRampND> resramps2;
        itramp0 = resramps.begin();
        while(itramp0!=resramps.end()) {
            itramp1=itramp0;
            itramp1++;
            itramp2=itramp1;
            itramp2++;
            if(itramp1!=resramps.end() && itramp2!=resramps.end()) {
                resramps2.push_back(*itramp0);
            }
            itramp0 = itramp1;
        }
        resramps2.splice(resramps2.end(),tmpramps1);
        resramps.swap(resramps2);
    }

    // Final check to make sure everything is right
    if(jittered) {
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
            RAVELOG_WARN("Failed final test, normally this should not happen!\n");
            return false;
        }
    }
    else{
        RAVELOG_VERBOSE("This trajectory does not seem to come from a previous jittering...\n");
        return false;
    }
}

dReal DetermineMinswitchtime(const ParabolicRamp::ParabolicRampND& rampnd)
{
    vector<dReal> vswitchtimes;
    vswitchtimes.resize(0);
    vswitchtimes.push_back(rampnd.endTime);
    FOREACHC(itramp,rampnd.ramps) {
        vector<dReal>::iterator it;
        if( itramp->tswitch1 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
            if( it != vswitchtimes.end() && *it != itramp->tswitch1) {
                vswitchtimes.insert(it,itramp->tswitch1);
            }
        }
        if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
            if( it != vswitchtimes.end() && *it != itramp->tswitch2 ) {
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

dReal DetermineMinswitchtime(const std::list<ParabolicRamp::ParabolicRampND>&ramps)
{
    if( ramps.size() == 0 ) {
        return 0;
    }
    dReal mintime = 1e10;
    FOREACHC(itramp,ramps) {
        mintime=min (DetermineMinswitchtime(*itramp),mintime);
    }
    return mintime;
}

size_t CountUnitaryRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps)
{
    size_t nbunitramps = 0;
    FOREACHC(itramp,ramps) {
        nbunitramps += CountUnitaryRamps(*itramp);
    }
    return nbunitramps;
}

size_t CountUnitaryRamps(const ParabolicRamp::ParabolicRampND& rampnd)
{
    vector<dReal> vswitchtimes;
    vswitchtimes.resize(0);
    vswitchtimes.push_back(rampnd.endTime);
    FOREACHC(itramp,rampnd.ramps) {
        vector<dReal>::iterator it;
        if( itramp->tswitch1 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
            if( it != vswitchtimes.end() && *it != itramp->tswitch1) {
                vswitchtimes.insert(it,itramp->tswitch1);
            }
        }
        if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
            if( it != vswitchtimes.end() && *it != itramp->tswitch2 ) {
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
        //Throw away ramps that have tiny time durations
        if(tend-tbeg>TINY) {
            result++;
        }
        tbeg = tend;
    }
    return result;
}

dReal ComputeRampsDuration(const std::list<ParabolicRamp::ParabolicRampND>&ramps)
{
    dReal res=0;
    FOREACH(itramp, ramps) {
        res += itramp->endTime;
    }
    return res;
}

// For logging purpose
void PrintRamps(const std::list<ParabolicRamp::ParabolicRampND>&ramps,ConstraintTrajectoryTimingParametersPtr params,bool checkcontrollertimestep)
{
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
                dReal ceilratio = RaveCeil(ratio);
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
void BreakOneRamp(ParabolicRamp::ParabolicRampND ramp,std::list<ParabolicRamp::ParabolicRampND>&resramp)
{
    vector<dReal> vswitchtimes;
    vswitchtimes.resize(0);
    vswitchtimes.push_back(ramp.endTime);
    FOREACHC(itramp,ramp.ramps) {
        vector<dReal>::iterator it;
        if( itramp->tswitch1 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
            if( it != vswitchtimes.end() && *it != itramp->tswitch1) {
                vswitchtimes.insert(it,itramp->tswitch1);
            }
        }
        if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
            if( it != vswitchtimes.end() && *it != itramp->tswitch2 ) {
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
        if(tend-tbeg>ParabolicRamp::EpsilonT) {
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

void BreakIntoUnitaryRamps(std::list<ParabolicRamp::ParabolicRampND>& ramps)
{
    std::list<ParabolicRamp::ParabolicRampND> tmpramps,resramps;
    FOREACHC(itramp,ramps) {
        BreakOneRamp(*itramp,tmpramps);
        resramps.splice(resramps.end(),tmpramps);
    }
    ramps.swap(resramps);
}


} // end namespace mergewaypoints


