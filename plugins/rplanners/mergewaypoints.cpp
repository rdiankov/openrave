#include "plugindefs.h"
#include <openrave/planningutils.h>
#include "mergewaypoints.h"

#define INF 1e10
#define TINY 1e-10


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
    if(interval1.isVoid or interval2.isVoid) {
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



//Timescale a unitary ramp, does not change the original ramp
void TimeScale(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& ramps,dReal coef){
    ramps.resize(0);
    FOREACH(itramp,origramps){
        PARABOLIC_RAMP_ASSERT(itramp->IsValid());
        dReal t = itramp->endTime;
        ParabolicRamp::Vector q0,v0,q1,v1;
        itramp->Evaluate(0,q0);
        itramp->Derivative(0,v0);
        itramp->Evaluate(t,q1);
        itramp->Derivative(t,v1);
        if(abs(coef-1)>TINY) {
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
        PARABOLIC_RAMP_ASSERT(ramps.back().IsValid());
    }
}


// //Copy a unitary ramp in place
// void CopyRamps(std::list<ParabolicRamp::ParabolicRampND>& ramps,std::list<ParabolicRamp::ParabolicRampND>& copyramps){
//     copyramps.resize(0);
//     FOREACHC(itramp,ramps){
//         dReal t = itramp->endTime;
//         ParabolicRamp::Vector q0,v0,q1,v1;
//         itramp->Evaluate(0,q0);
//         itramp->Derivative(0,v0);
//         itramp->Evaluate(t,q1);
//         itramp->Derivative(t,v1);
//         copyramps.push_back(ParabolicRamp::ParabolicRampND());
//         copyramps.back().SetPosVelTime(q0,v0,q1,v1,t);
//     }
// }


bool CheckMerge(dReal T0,dReal T1,dReal T2,std::vector<dReal>& q0,const std::vector<dReal>& v0,std::vector<dReal>& q3,std::vector<dReal>& v3,dReal& alpha,std::vector<dReal>& qres,std::vector<dReal>& vres,ConstraintTrajectoryTimingParametersPtr params){
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
    dReal q1,v1,a0,a1;

    qres.resize(q0.size(),0);
    vres.resize(q0.size(),0);

    for(size_t j=0; j<q0.size(); j++) {
        q1 = q0[j]+0.5*Ta*(Tb*(v0[j]-v3[j])+2*(q3[j]-q0[j]))/(Ta+Tb);
        v1 = (2*(q3[j]-q0[j])-(Ta*v0[j]+Tb*v3[j]))/(Ta+Tb);
        a0 = 1/Ta/(Ta+Tb)*(2*(q3[j]-q0[j])-(2*Ta+Tb)*v0[j]-Tb*v3[j]);
        a1 = 1/Tb/(Ta+Tb)*(-2*(q3[j]-q0[j])+(Ta+2*Tb)*v3[j]+Ta*v0[j]);
        if(not (v1>= -vmax[j]-TINY and v1 <= vmax[j]+TINY and a0>= -amax[j]-TINY and a0 <= amax[j]+TINY and a1>= -amax[j]-TINY and a1 <= amax[j]+TINY)) {
            cout << TINY<<"\n";
            cout << "v1:" << -vmax[j]-TINY <<"<" << v1 << "<" << vmax[j]+TINY<<"\n";
            cout << "a0:" << -amax[j]-TINY <<"<" << a0 << "<" << amax[j]+TINY<<"\n";
            cout << "a1:" << -amax[j]-TINY <<"<" << a1 << "<" << amax[j]+TINY<<"\n";
            BOOST_ASSERT(false);
        }
        if (not (q1>= qmin[j] and q1 <= qmax[j])) {
            return false;
        }
        dReal tm,qm;
        if (RaveFabs(a0)>TINY) {
            tm = -v0[j]/a0;
            qm = q0[j]+v0[j]*tm+0.5*a0*tm*tm;
            if (tm >0  and tm < Ta and (qm < qmin[j]-TINY or qm > qmax[j]+TINY)) {
                return false;
            }
        }
        if (RaveFabs(a1)>TINY) {
            tm = -v1/a1;
            qm = q1+v1*tm+0.5*a1*tm*tm;
            if (tm > 0 and tm < Tb and (qm < qmin[j]-TINY or qm > qmax[j]+TINY)) {
                return false;
            }
        }
        qres[j] = q1;
        vres[j] = v1;
    }
    return true;
}

//Merge 3 ramps into 2 ramps while respecting joint and velocity on the borders
//Note that the ramps must be unitary
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

    if(not res) {
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



bool IterativeMergeRampsFixedTime(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& ramps,dReal minswitchtime,ConstraintTrajectoryTimingParametersPtr params,int iters){
    int itersi=0;
    bool solvedglobal = false;
    while((not solvedglobal) and itersi<iters) {
        //printf("Iteration %d\n",itersi);
        TimeScale(origramps,ramps,1);
        itersi++;
        // Kill all ramps that are not the first and the last
        bool solved = false;
        while(true) {
            solved = true;
            size_t i = 0;
            FOREACHC(itramp,ramps){
                if(itramp->endTime < minswitchtime and i>0 and i<ramps.size()-1) {
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
                if(itramp->endTime < minswitchtime and i>0 and i<ramps.size()-1) {
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
            if(not res) {
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
        if(not solved) {
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
            if(not res) {
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
        if(not solved) {
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
            if(not res) {
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
    return solvedglobal;

}

bool IterativeMergeRamps(std::list<ParabolicRamp::ParabolicRampND>& origramps,std::list<ParabolicRamp::ParabolicRampND>& resramps,dReal& hi, dReal minswitchtime,ConstraintTrajectoryTimingParametersPtr params,dReal maxcoef, dReal precision, int iters){

    std::list<ParabolicRamp::ParabolicRampND> ramps,ramps2;
    dReal testcoef;

    //printf("Coef = 1\n");
    TimeScale(origramps,ramps,1);
    bool res = IterativeMergeRampsFixedTime(ramps,ramps2,minswitchtime,params,iters);
    if (res) {
        hi = 1;
        resramps = ramps2;
        return true;
    }
    //printf("Coef = %f\n",maxcoef);
    TimeScale(origramps,ramps,maxcoef);
    res = IterativeMergeRampsFixedTime(ramps,ramps2,minswitchtime,params,iters);
    if (not res) {
        hi = -1;
        return false;
    }
    dReal lo = 1;
    hi = maxcoef;
    while (hi-lo>precision) {
        testcoef = (hi+lo)/2;
        //printf("Coef = %f\n",testcoef);
        TimeScale(origramps,ramps,testcoef);
        res = IterativeMergeRampsFixedTime(ramps,ramps2,minswitchtime,params,iters);
        if(res) {
            hi = testcoef;
            resramps = ramps2;
        }
        else{
            lo = testcoef;
        }
    }

    // cout << "Final ramps\n";
    // FOREACHC(itramp,resramps){
    //     cout<< "Ramp duration:" << itramp->endTime << "\n";
    // }

    return true;







}

void BreakOneRamp(ParabolicRamp::ParabolicRampND rampnd,std::list<ParabolicRamp::ParabolicRampND>& tempramp, dReal minswitchtime){

    //if minswitchtime<0 no timescaling

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

    dReal coef=1;
    dReal tbeg,tend;
    if(minswitchtime>TINY) {
        tbeg = 0;
        for(size_t i=0; i<vswitchtimes.size(); i++) {
            tend = vswitchtimes[i];
            if (tend-tbeg<minswitchtime) {
                coef = max(coef,minswitchtime/(tend-tbeg));
            }
        }
    }

    tempramp.resize(0);
    tbeg = 0;
    for(size_t i=0; i<vswitchtimes.size(); i++) {
        ParabolicRamp::Vector q0,v0,q1,v1;
        tend = vswitchtimes[i];
        //printf("vswitch[i] = %f\n",tend);
        rampnd.Evaluate(tbeg,q0);
        rampnd.Derivative(tbeg,v0);
        rampnd.Evaluate(tend,q1);
        rampnd.Derivative(tend,v1);
        if(abs(coef-1)>TINY) {
            dReal invcoef = 1/coef;
            FOREACH(itv,v0){
                *itv *= invcoef;
            }
            FOREACH(itv,v1){
                *itv *= invcoef;
            }
        }
        tempramp.push_back(ParabolicRamp::ParabolicRampND());
        tempramp.back().SetPosVelTime(q0,v0,q1,v1,(tend-tbeg)*coef);
        PARABOLIC_RAMP_ASSERT(tempramp.back().IsValid());
        tbeg = tend;
    }
}


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



void BreakIntoUnitaryRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps,std::list<ParabolicRamp::ParabolicRampND>& resramps,dReal minswitchtime){
    resramps.resize(0);
    std::list<ParabolicRamp::ParabolicRampND> tempramp;
    FOREACHC(itramp,ramps){
        BreakOneRamp(*itramp,tempramp,minswitchtime);
        resramps.splice(resramps.end(),tempramp);
    }
}
