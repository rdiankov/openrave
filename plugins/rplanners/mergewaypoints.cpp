#include "plugindefs.h"
#include "mergewaypoints.h"

#define INF 1e10


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
    if (RaveFabs(a)<g_fEpsilonLinear) {
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


bool CheckMerge(dReal T0,dReal T1,dReal T2,std::vector<dReal>& q0,const std::vector<dReal>& v0,std::vector<dReal>& q3,std::vector<dReal>& v3,dReal& alpha,std::vector<dReal>& qres,std::vector<dReal>& vres,const PlannerParametersConstPtr params){
    dReal T = T0+T1+T2;
    dReal Q,A0,B0lo,B0hi,A1lo,A1hi,B1,A2lo,B2lo,A2hi,B2hi;
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
        BOOST_ASSERT(v1>= -vmax[j] and v1 <= vmax[j] and a0>= -amax[j] and a0 <= amax[j] and a1>= -amax[j] and a1 <= amax[j]);
        if (not (q1>= qmin[j] and q1 <= qmax[j])) {
            return false;
        }
        dReal tm,qm;
        if (RaveFabs(a0)>g_fEpsilonLinear) {
            tm = -v0[j]/a0;
            qm = q0[j]+v0[j]*tm+0.5*a0*tm*tm;
            if (tm >0  and tm < Ta and (qm < qmin[j] or qm > qmax[j])) {
                return false;
            }
        }
        if (RaveFabs(a1)>g_fEpsilonLinear) {
            tm = -v1/a1;
            qm = q1+v1*tm+0.5*a1*tm*tm;
            if (tm > 0 and tm < Tb and (qm < qmin[j] or qm > qmax[j])) {
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
bool MergeWaypoints(const ParabolicRamp::ParabolicRampND& ramp0,const ParabolicRamp::ParabolicRampND& ramp1,const ParabolicRamp::ParabolicRampND& ramp2,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1,const std::vector<dReal>& qmin,const std::vector<dReal>& qmax,const std::vector<dReal>& vmax,const std::vector<dReal>& amax){
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

    vector<dReal> qres;
    vector<dReal> vres;
    dReal alpha;
    bool res = CheckMerge(T0,T1,T2,q0,v0,q3,v3,qmin,qmax,vmax,amax,alpha,qres,vres);

    if(not res) {
        return false;
    }

    Ta = alpha*T;
    Tb = T-Ta;
    resramp0 = ParabolicRamp::ParabolicRampND();
    resramp0.SetPosVelTime(q0,v0,qres,vres,Ta);
    resramp1 = ParabolicRamp::ParabolicRampND();
    resramp1.SetPosVelTime(qres,vres,q3,v3,Tb);
    return true;
}



//Merge 3 ramps into 2 ramps while respecting joint and velocity on the borders
//Note that the ramps must be unitary
bool MergeRamps(const ParabolicRamp::ParabolicRampND& ramp0,const ParabolicRamp::ParabolicRampND& ramp1,const ParabolicRamp::ParabolicRampND& ramp2,ParabolicRamp::ParabolicRampND& resramp0,ParabolicRamp::ParabolicRampND& resramp1,const std::vector<dReal>& qmin,const std::vector<dReal>& qmax,const std::vector<dReal>& vmax,const std::vector<dReal>& amax){
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

    vector<dReal> qres;
    vector<dReal> vres;
    dReal alpha;
    bool res = CheckMerge(T0,T1,T2,q0,v0,q3,v3,qmin,qmax,vmax,amax,alpha,qres,vres);

    if(not res) {
        return false;
    }

    Ta = alpha*T;
    Tb = T-Ta;
    resramp0 = ParabolicRamp::ParabolicRampND();
    resramp0.SetPosVelTime(q0,v0,qres,vres,Ta);
    resramp1 = ParabolicRamp::ParabolicRampND();
    resramp1.SetPosVelTime(qres,vres,q3,v3,Tb);
    return true;
}


void IterativeMergeRampsFixedTime(std::list<ParabolicRamp::ParabolicRampND>& ramps,minswitchtime){
    bool solved = false;
    while(not solved) {
        dReal mintime = 1e10;
        FOREACH(itramp,ramps){
            mintime = min(mintime,itramp->endTime);
        }
        if(mintime >= minswitchtime) {
            solved = true;
            break;
        }
        if(ramps.size()<3) {
            fprint("Special case\n");
            BOOST_ASSERT(false);
        }

        size_t nramps = ramps.size();

        //Iteratively kill all clusters except at the beginning and the end of the trajectory
        std::list<ParabolicRamp::ParabolicRampND>::iterator itlist = ramps.begin();
        itlist++;
        while(itlist != ramps.end() ) {
            if (itlist->endTime<minswitchtime) {
                itlist++;
                if (itlist !=ramps.end()) {
                    ParabolicRamp::ParabolicRampND ramp0,ramp1,ramp2,resramp0,resramp1;
                    ramp2 = itlist;
                    itlist--;
                    ramp1 = itlist;
                    itlist--;
                    ramp0 = itlist;
                    MergeWaypoints(ramp0,ramp1,ramp2,resramp0,resramp1,params);
                }
            }
            if
            if( removecond ) {
                itlist = listobj.erase(itlist);
            }


            FOREACH(itramp,ramps){
                if (itramp->endTime < minswitchtime) {
                    if(i>0 and i<nramps-1) {

                    }
                }
            }


        }
    }
}
/*
   list<T>::iterator itlist = listobj.begin();
   while(itlist != listobj.end() ) {
    if( removecond ) {
        itlist = listobj.erase(itlist);
    }
    else {
   ++itlist;
    }
   }
 */


void BreakIntoUnitaryRamps(const std::list<ParabolicRamp::ParabolicRampND>& ramps,std::list<ParabolicRamp::ParabolicRampND>& resramps){
    resramps.resize(0);
    std::list<ParabolicRamp::ParabolicRampND> tempramp;
    FOREACHC(itramp,ramps){
        BreakOneRamp(*itramp,tempramp);
        resramps.splice(resramps.end(),tempramp);
    }
}


//Timescale a unitary ramp
void TimeScale(std::list<ParabolicRamp::ParabolicRampND>& ramps,dReal coef){
    FOREACH(itramp,ramps){
        dReal t = itramp->endTime;
        ParabolicRamp::Vector q0,v0,q1,v1;
        itramp->Evaluate(0,q0);
        itramp->Derivative(0,v0);
        itramp->Evaluate(t,q1);
        itramp->Derivative(t,v1);
        dReal invcoef = 1/coef;
        FOREACH(itv,v0){
            *itv *= invcoef;
        }
        FOREACH(itv,v1){
            *itv *= invcoef;
        }
        itramp->SetPosVelTime(q0,v0,q1,v1,t*coef);
    }
}
