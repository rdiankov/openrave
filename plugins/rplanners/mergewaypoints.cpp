#include "plugindefs.h"
#include <openrave/planningutils.h>
#include "mergewaypoints.h"

#define INF 1e10
#define TINY g_fEpsilonLinear





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

Interval Intersection(Interval interval1,Interval interval2){
    if(interval1.isVoid || interval2.isVoid) {
        return Interval();
    }
    return Interval(max(interval1.lo,interval2.lo),min(interval1.hi,interval2.hi));
}

//Solve the inequality ax => b
Interval SolveIneq(dReal a,dReal b){
    if (RaveFabs(a)<TINY) {
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





/** Timescale a ramp. Assume the ramp is unitary.
    \param origramps input ramp
    \param resramps result ramp
    \param coef timescaling coefficient
 */
void TimeScale(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& ramps,dReal coef){
    ramps.resize(0);
    bool dilated = abs(coef-1)>TINY;
    FOREACH(itramp,origramps){
        PARABOLIC_RAMP_ASSERT(itramp->IsValid());
        dReal t = itramp->endTime;
        if(t>TINY) {
            ParabolicRamp::Vector q0,v0,q1,v1;
            itramp->Evaluate(0,q0);
            itramp->Derivative(0,v0);
            itramp->Evaluate(t,q1);
            itramp->Derivative(t,v1);
            if(dilated) {
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
            ramps.back().modified = itramp->modified;
            ramps.back().dilated = dilated || itramp->dilated;
            PARABOLIC_RAMP_ASSERT(ramps.back().IsValid());
        }
    }
}


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
            // cout << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            // cout << TINY<<"\n";
            // cout << "v1:" << -vmax[j]-TINY <<"<" << v1 << "<" << vmax[j]+TINY<<"\n";
            // cout << "a0:" << -amax[j]-TINY <<"<" << a0 << "<" << amax[j]+TINY<<"\n";
            // cout << "a1:" << -amax[j]-TINY <<"<" << a1 << "<" << amax[j]+TINY<<"\n";
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




/** Check whether a point inside the time interval T1 can be merged into the other two intervals T0, and T2. The new interval times of T0 and T2 will be Ta and Tb.
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



//Return the smallest multiple of step larger or equal to T
dReal ceiling(dReal T,dReal step){
    dReal ratio = T/step;
    dReal ceilratio = ceil(ratio-TINY);
    return ceilratio*step;
}

//Check whether T is a multiple of step
bool ismult(dReal T,dReal step){
    return RaveFabs(T-ceiling(T,step)) < TINY;
}



/** Fix the time durations of 2 ramps
   Assume that the ramps are unitary
    \param rampx input ramps with x=0,1,
    \param resrampx result ramp with x=0,1
 */
bool FixRamps(const ParabolicRamp::ParabolicRampND& ramp0,const ParabolicRamp::ParabolicRampND& ramp1,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1,ConstraintTrajectoryTimingParametersPtr params){
    vector<dReal> q0,v0,q2,v2;
    dReal T0,T1,Ta,Tb;
    ramp0.Evaluate(0,q0);
    ramp0.Derivative(0,v0);
    ramp1.Evaluate(ramp1.endTime,q2);
    ramp1.Derivative(ramp1.endTime,v2);
    T0 = ramp0.endTime;
    T1 = ramp1.endTime;
    Ta = ceiling(T0,params->_fStepLength);
    Tb = ceiling(T1,params->_fStepLength);
    cout << params->_fStepLength << "\n";

    //printf("Try fixing: T0=%f, T1=%f, Ta=%f, Tb=%f...  ",T0,T1,Ta,Tb);

    vector<dReal> qres;
    vector<dReal> vres;

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
    return true;
}



/** Merge 3 ramps into 2 ramps while respecting joint and velocity on the borders
   Assume that the ramps are unitary
    \param rampx input ramps with x=0,1,2
    \param resrampx result ramp with x=0,1
 */
bool MergeRamps(const ParabolicRamp::ParabolicRampND& ramp0,const ParabolicRamp::ParabolicRampND& ramp1,const ParabolicRamp::ParabolicRampND& ramp2,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1,ConstraintTrajectoryTimingParametersPtr params){
    vector<dReal> q0,v0,q3,v3;
    dReal T,T0,T1,T2,Ta,Tb;
    ramp0.Evaluate(0,q0);
    ramp0.Derivative(0,v0);
    ramp2.Evaluate(ramp2.endTime,q3);
    ramp2.Derivative(ramp2.endTime,v3);
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
    PARABOLIC_RAMP_ASSERT(resramp0.IsValid()&&resramp1.IsValid());
    return true;
}



// Small function to compute the factorial of a number
int factorial(int n){
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}








//Iteratively fix the time durations of the ramps
// Note that, by construction, there is no isolated modified ramps
bool IterativeFixRamps(std::list<ParabolicRamp::ParabolicRampND>& ramps, ConstraintTrajectoryTimingParametersPtr params){

    std::list<ParabolicRamp::ParabolicRampND>::iterator itramp = ramps.begin();
    while(itramp!=ramps.end()) {
        if(itramp->modified || itramp->dilated) {
            itramp++;
            while(itramp!=ramps.end()) {
                if((itramp->modified || itramp->dilated )&& (!ismult(itramp->endTime,params->_fStepLength))) {
                    //Fix time duration of current ramp and previous ramp
                    ParabolicRamp::ParabolicRampND ramp0,ramp1,resramp0,resramp1;
                    ramp1 = *itramp;
                    itramp--;
                    ramp0 = *itramp;
                    bool res = FixRamps(ramp0,ramp1,resramp0,resramp1,params);
                    if(!res) {
                        return false;
                    }
                    else{
                        // Update ramps
                        itramp = ramps.erase(itramp);
                        itramp = ramps.erase(itramp);
                        resramp0.modified = true;
                        resramp1.modified = true;
                        ramps.insert(itramp,resramp1);
                        itramp--;
                        ramps.insert(itramp,resramp0);
                    }
                    itramp++;
                }
                else{
                    itramp++;
                    break;
                }
            }
        }
        else{
            itramp++;
        }
    }
    return true;
}





/** Iteratively merge all ramps that are shorter than minswitchtime. Does not change the global time duration
    \param origramps input ramps
    \param ramps result ramps
    \param iters max number of random iterations
    \param v3 the velocity at the end of the ramp at T2
 */
bool IterativeMergeRampsFixedTime(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& ramps, dReal minswitchtime,ConstraintTrajectoryTimingParametersPtr params,int iters,bool checkcontrollertime){

    // Determine the number max of iterations as a function of the number of short ramps
    size_t i = 0;
    int nb_short_ramps = 0;
    FOREACHC(itramp,origramps){
        if(itramp->endTime < minswitchtime && i>0 && i<ramps.size()-1) {
            nb_short_ramps++;
        }
        i++;
    }
    iters = min(iters, 2*factorial(nb_short_ramps));

    int itersi=0;
    bool solvedglobal = false;

    // This loop could be optimized by caching and re-using results of permutations that begin similarly
    while((!solvedglobal) && itersi<iters) {
        //printf("Iteration %d\n",itersi);
        TimeScale(origramps,ramps,1);
        itersi++;
        // Kill all ramps that are not the first and the last
        bool solved = false;
        while(true) {
            solved = true;
            size_t i = 0;
            FOREACHC(itramp,ramps){
                if(itramp->endTime < minswitchtime && i>0 && i<ramps.size()-1) {
                    solved = false;
                    break;
                }
                i++;
            }
            if (solved) {
                break;
            }
            std::vector<dReal> deltatimes;
            deltatimes.resize(0);
            i = 0;
            FOREACHC(itramp,ramps){
                if(itramp->endTime < minswitchtime && i>0 && i<ramps.size()-1) {
                    deltatimes.push_back(i);
                }
                i++;
            }
            int jramp = deltatimes[rand()%deltatimes.size()];
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
            resramp0.modified = true;
            resramp1.modified = true;
            ramps.insert(itlist,resramp1);
            itlist--;
            ramps.insert(itlist,resramp0);
        }
        if(!solved) {
            continue;
        }
        // Handle the beginning the trajectory
        while(ramps.begin()->endTime<minswitchtime) {
            if (ramps.size()<2) {
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
            resramp0.modified = true;
            resramp1.modified = true;
            ramps.insert(itlist,resramp1);
            itlist--;
            ramps.insert(itlist,resramp0);
        }
        if(!solved) {
            continue;
        }
        // Handle the end the trajectory
        while(ramps.back().endTime<minswitchtime) {
            if (ramps.size()<2) {
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
            resramp0.modified = true;
            resramp1.modified = true;
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
        return IterativeFixRamps(ramps,params);
    }
    else{
        return solvedglobal;
    }
}





/** Iteratively merge all ramps that are shorter than minswitchtime. Determine the optimal time duration that allows to do so
    \param origramps input ramps
    \param ramps result ramps
    \param maxcoef maximum timescaling coefficient to try
    \param precision precision in the dichotomy search for the best timescaling coef
    \param iters max number of random iterations
 */
bool IterativeMergeRamps(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& resramps,dReal& hi, dReal minswitchtime,ConstraintTrajectoryTimingParametersPtr params,dReal maxcoef, dReal precision, int iters,bool checkcontrollertime){

    std::list<ParabolicRamp::ParabolicRampND> ramps,ramps2;
    dReal testcoef;

    //printf("Coef = 1\n");
    TimeScale(origramps,ramps,1);
    bool res = IterativeMergeRampsFixedTime(ramps,ramps2,minswitchtime,params,iters,checkcontrollertime);
    if (res) {
        hi = 1;
        resramps = ramps2;
        return true;
    }
    //printf("Coef = %f\n",maxcoef);
    TimeScale(origramps,ramps,maxcoef);
    res = IterativeMergeRampsFixedTime(ramps,ramps2,minswitchtime,params,iters,checkcontrollertime);
    if (!res) {
        hi = -1;
        return false;
    }
    resramps = ramps2;
    hi = maxcoef;
    dReal lo = 1;
    while (hi-lo>precision) {
        testcoef = (hi+lo)/2;
        //printf("Coef = %f\n",testcoef);
        TimeScale(origramps,ramps,testcoef);
        res = IterativeMergeRampsFixedTime(ramps,ramps2,minswitchtime,params,iters,checkcontrollertime);
        if(res) {
            hi = testcoef;
            resramps = ramps2;
        }
        else{
            lo = testcoef;
        }
    }

    return true;
}






/** Break one ramp into unitary ramps. A unitary ramp consists of a single acceleration
    \param ramp input ramp
    \param resramp result ramp
 */
void BreakOneRamp(ParabolicRamp::ParabolicRampND ramp,std::list<ParabolicRamp::ParabolicRampND>& resramp){

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
        //printf("vswitch[i] = %f\n",tend);
        //Discard ramps that have tiny time durations
        if(tend-tbeg>TINY) {
            ramp.Evaluate(tbeg,q0);
            ramp.Derivative(tbeg,v0);
            ramp.Evaluate(tend,q1);
            ramp.Derivative(tend,v1);
            resramp.push_back(ParabolicRamp::ParabolicRampND());
            resramp.back().SetPosVelTime(q0,v0,q1,v1,(tend-tbeg));
            resramp.back().modified = ramp.modified;
            resramp.back().dilated = ramp.dilated;
            PARABOLIC_RAMP_ASSERT(resramp.back().IsValid());
        }
        tbeg = tend;
    }
}





/** Determine the minimum switchtime in a ramp
    \param rampnd input ramp
 */
dReal DetermineMinswitchtime(ParabolicRamp::ParabolicRampND rampnd){
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



/** Count the number of pieces in a ramp
    \param rampnd input ramp
 */
size_t CountPieces(ParabolicRamp::ParabolicRampND rampnd){
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





/** Break ramps list into unitary ramps
    \param ramps input ramp
    \param resramps result ramp
 */
void BreakIntoUnitaryRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps,std::list<ParabolicRamp::ParabolicRampND>& resramps){
    resramps.resize(0);
    std::list<ParabolicRamp::ParabolicRampND> tempramp;
    FOREACHC(itramp,ramps){
        BreakOneRamp(*itramp,tempramp);
        resramps.splice(resramps.end(),tempramp);
    }
}


void PrintRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps,ConstraintTrajectoryTimingParametersPtr params,bool warning){
    int itx=0;
    cout << "[";
    FOREACH(itramp, ramps) {
        cout << itx << "/";
        if(itramp->modified) {
            cout << "M";
        }
        if(itramp->dilated) {
            cout << "D";
        }
        cout << "/"<<itramp->endTime;
        cout << "/"<<itramp->endTime/params->_fStepLength << ";  ";
        if(warning) {
            if(!ismult(itramp->endTime,params->_fStepLength)) {

                cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n";
                dReal T = itramp->endTime;
                dReal step = params->_fStepLength;
                dReal ratio = T/step;
                dReal ceilratio = ceil(ratio);
                cout << ratio << "\n";
                cout << ceilratio << "\n";
                cout << ceilratio-ratio << "\n";
            }
        }
        itx++;
    }
    cout << "]\n";
}
