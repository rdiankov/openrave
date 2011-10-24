// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#include "libopenrave.h"

#include <openrave/planningutils.h>

namespace OpenRAVE {
namespace planningutils {

int JitterActiveDOF(RobotBasePtr robot,int nMaxIterations,dReal fRand,const PlannerBase::PlannerParameters::NeighStateFn& neighstatefn)
{
    RAVELOG_VERBOSE("starting jitter active dof...\n");
    vector<dReal> curdof, newdof, deltadof, deltadof2;
    robot->GetActiveDOFValues(curdof);
    newdof.resize(curdof.size());
    deltadof.resize(curdof.size(),0);
    CollisionReport report;
    CollisionReportPtr preport(&report,null_deleter());
    bool bCollision = false;
    bool bConstraint = !!neighstatefn;

    // have to test with perturbations since very small changes in angles can produce collision inconsistencies
    boost::array<dReal,3> perturbations = { { 0,1e-5f,-1e-5f}};
    FOREACH(itperturbation,perturbations) {
        if( bConstraint ) {
            FOREACH(it,deltadof) {
                *it = *itperturbation;
            }
            newdof = curdof;
            if( !neighstatefn(newdof,deltadof,0) ) {
                return -1;
            }
        }
        else {
            for(size_t i = 0; i < newdof.size(); ++i) {
                newdof[i] = curdof[i]+*itperturbation;
            }
        }
        robot->SetActiveDOFValues(newdof,true);

        if(robot->CheckSelfCollision(preport)) {
            bCollision = true;
            RAVELOG_DEBUG(str(boost::format("JitterActiveDOFs: self collision: %s!\n")%report.__str__()));
            break;
        }
        if( robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot),preport) ) {
            bCollision = true;
            RAVELOG_DEBUG(str(boost::format("JitterActiveDOFs: collision: %s!\n")%report.__str__()));
            break;
        }
    }

    if( !bCollision || fRand <= 0 ) {
        return -1;
    }

    deltadof2.resize(curdof.size(),0);
    for(int iter = 0; iter < nMaxIterations; ++iter) {
        for(size_t j = 0; j < newdof.size(); ++j) {
            deltadof[j] = fRand * (RaveRandomFloat()-0.5f);
        }
        bCollision = false;
        FOREACH(itperturbation,perturbations) {
            for(size_t j = 0; j < deltadof.size(); ++j) {
                deltadof2[j] = deltadof[j] + *itperturbation;
            }
            if( bConstraint ) {
                newdof = curdof;
                if( !neighstatefn(newdof,deltadof2,0) ) {
                    RAVELOG_DEBUG("constraint function failed\n");
                    continue;
                }
            }
            else {
                for(size_t j = 0; j < deltadof.size(); ++j) {
                    newdof[j] = curdof[j] + deltadof2[j];
                }
            }
            robot->SetActiveDOFValues(newdof,true);
            if(robot->CheckSelfCollision() || robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot)) ) {
                bCollision = true;
                break;
            }
        }
        if( !bCollision ) {
            return 1;
        }
    }

    return 0;
}

bool JitterTransform(KinBodyPtr pbody, float fJitter, int nMaxIterations)
{
    RAVELOG_VERBOSE("starting jitter transform...\n");

    // randomly add small offset to the body until it stops being in collision
    Transform transorig = pbody->GetTransform();
    Transform transnew = transorig;
    int iter = 0;
    while(pbody->GetEnv()->CheckCollision(KinBodyConstPtr(pbody)) ) {
        if( iter > nMaxIterations ) {
            return false;
        }
        if( iter > 0 && fJitter <= 0 ) {
            return false;
        }
        transnew.trans = transorig.trans + fJitter * Vector(RaveRandomFloat()-0.5f, RaveRandomFloat()-0.5f, RaveRandomFloat()-0.5f);
        pbody->SetTransform(transnew);
        ++iter;
    }

    return true;
}

void VerifyTrajectory(PlannerBase::PlannerParametersConstPtr parameters, TrajectoryBaseConstPtr trajectory, dReal samplingstep)
{
    BOOST_ASSERT((int)parameters->_vConfigLowerLimit.size() == parameters->GetDOF());
    BOOST_ASSERT((int)parameters->_vConfigUpperLimit.size() == parameters->GetDOF());
    BOOST_ASSERT((int)parameters->_vConfigResolution.size() == parameters->GetDOF());

    dReal fthresh = 5e-5f;
    vector<dReal> deltaq(parameters->GetDOF(),0);
    std::vector<dReal> vdata;
    for(size_t ipoint = 0; ipoint < trajectory->GetNumWaypoints(); ++ipoint) {
        trajectory->GetWaypoint(ipoint,vdata,parameters->_configurationspecification);
        BOOST_ASSERT((int)vdata.size()==parameters->GetDOF());
        for(size_t i = 0; i < vdata.size(); ++i) {
            BOOST_ASSERT(vdata[i] >= parameters->_vConfigLowerLimit[i]-fthresh);
            BOOST_ASSERT(vdata[i] <= parameters->_vConfigUpperLimit[i]+fthresh);
        }
        parameters->_setstatefn(vdata);
        vector<dReal> newq;
        parameters->_getstatefn(newq);
        BOOST_ASSERT(vdata.size() == newq.size());
        for(size_t i = 0; i < newq.size(); ++i) {
            if( RaveFabs(vdata.at(i) - newq.at(i)) > 0.001 * parameters->_vConfigResolution[i] ) {
                ofstream f("failedtrajectory.xml");
                f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
                trajectory->serialize(f);
                throw OPENRAVE_EXCEPTION_FORMAT("setstate/getstate inconsistent configuration %d dof %d: %f != %f",ipoint%i%vdata.at(i)%newq.at(i),ORE_InconsistentConstraints);
            }
        }
        if( !!parameters->_neighstatefn ) {
            newq = vdata;
            if( !parameters->_neighstatefn(newq,deltaq,0) ) {
                ofstream f("failedtrajectory.xml");
                f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
                trajectory->serialize(f);
                throw OPENRAVE_EXCEPTION_FORMAT("neighstatefn is rejecting configuration %d",ipoint,ORE_InconsistentConstraints);
            }
        }
    }

    if( !!parameters->_checkpathconstraintsfn && trajectory->GetNumWaypoints() >= 2 ) {

        if( 0 ) { //trajectory->GetDuration() > 0 && samplingstep > 0 ) {
            // use sampling
            std::vector<dReal> vprevdata;
            PlannerBase::ConfigurationListPtr configs(new PlannerBase::ConfigurationList());
            trajectory->Sample(vprevdata,0,parameters->_configurationspecification);
            for(dReal ftime = 0; ftime < trajectory->GetDuration(); ftime += samplingstep ) {
                configs->clear();
                trajectory->Sample(vdata,ftime+samplingstep,parameters->_configurationspecification);
                if( !parameters->_checkpathconstraintsfn(vprevdata,vdata,IT_Closed, configs) ) {
                    ofstream f("failedtrajectory.xml");
                    f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
                    trajectory->serialize(f);
                    throw OPENRAVE_EXCEPTION_FORMAT("checkpathconstraintsfn failed at %fs-%fs",ftime%(ftime+samplingstep),ORE_InconsistentConstraints);
                }
                FOREACH(itconfig, *configs) {
                    if( !parameters->_neighstatefn(*itconfig,deltaq,0) ) {
                        ofstream f("failedtrajectory.xml");
                        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
                        trajectory->serialize(f);
                        throw OPENRAVE_EXCEPTION_FORMAT("neighstatefn is rejecting configurations from checkpathconstraintsfn %fs-%fs",ftime%(ftime+samplingstep),ORE_InconsistentConstraints);
                    }
                }
                vprevdata=vdata;
            }
        }
        else {
            for(size_t i = 0; i < trajectory->GetNumWaypoints(); ++i) {
                trajectory->GetWaypoint(i,vdata,parameters->_configurationspecification);
                if( !parameters->_checkpathconstraintsfn(vdata,vdata,IT_OpenStart, PlannerBase::ConfigurationListPtr()) ) {
                    ofstream f("failedtrajectory.xml");
                    f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
                    trajectory->serialize(f);
                    throw OPENRAVE_EXCEPTION_FORMAT("checkpathconstraintsfn failed at %d-%d",(i-1)%i,ORE_InconsistentConstraints);
                }
            }
        }
    }
}

void RetimeActiveDOFTrajectory(TrajectoryBasePtr traj, RobotBasePtr probot, bool hastimestamps, dReal fmaxvelmult, const std::string& plannername)
{
    PlannerBasePtr planner = RaveCreatePlanner(traj->GetEnv(),plannername.size() > 0 ? plannername : string("lineartrajectoryretimer"));
    PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
    params->SetRobotActiveJoints(probot);
    FOREACH(it,params->_vConfigVelocityLimit) {
        *it *= fmaxvelmult;
    }
    string interpolation = "linear";
    params->_sExtraParameters += str(boost::format("<interpolation>%s</interpolation><hastimestamps>%d</hastimestamps>")%interpolation%hastimestamps);
    if( !planner->InitPlan(probot,params) ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to InitPlan",ORE_Failed);
    }

    if( params->GetDOF() != traj->GetConfigurationSpecification().GetDOF() ) {
        // have to remove unnecessary DOFs in order for spaces to match
        ConvertTrajectorySpecification(traj,probot->GetActiveConfigurationSpecification());
    }
    if( planner->PlanPath(traj) != PS_HasSolution ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to PlanPath",ORE_Failed);
    }
}

static void diffstatefn(std::vector<dReal>& q1, const std::vector<dReal>& q2, const std::vector<int>& vrotaxes)
{
    BOOST_ASSERT(q1.size()==q2.size());
    for(size_t i = 0; i < q1.size(); ++i) {
        if( find(vrotaxes.begin(),vrotaxes.end(),i) != vrotaxes.end() ) {
            q1[i] = ANGLE_DIFF(q1[i],q2[i]);
        }
        else {
            q1[i] -= q2[i];
        }
    }
}

void RetimeAffineTrajectory(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, bool hastimestamps, const std::string& plannername)
{
    PlannerBasePtr planner = RaveCreatePlanner(traj->GetEnv(),plannername.size() > 0 ? plannername : string("lineartrajectoryretimer"));
    PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
    string interpolation = "linear";
    params->_vConfigVelocityLimit = maxvelocities;
    params->_vConfigAccelerationLimit = maxaccelerations;
    params->_configurationspecification = traj->GetConfigurationSpecification();
    params->_vConfigLowerLimit.resize(traj->GetConfigurationSpecification().GetDOF());
    params->_vConfigUpperLimit.resize(traj->GetConfigurationSpecification().GetDOF());
    for(size_t i = 0; i < params->_vConfigLowerLimit.size(); ++i) {
        params->_vConfigLowerLimit[i] = -1e6;
        params->_vConfigUpperLimit[i] = 1e6;
    }

    // analyze the configuration for identified dimensions
    std::vector<int> vrotaxes;
    FOREACHC(itgroup,traj->GetConfigurationSpecification()._vgroups) {
        if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            string tempname;
            int affinedofs=0;
            stringstream ss(itgroup->name.substr(16));
            ss >> tempname >> affinedofs;
            if( !!ss ) {
                if( affinedofs & DOF_RotationAxis ) {
                    vrotaxes.push_back(itgroup->offset+RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationAxis));
                }
            }
        }
        else if( itgroup->name.size() >= 14 && itgroup->name.substr(0,14) == "ikparam_values" ) {
            int iktypeint = 0;
            stringstream ss(itgroup->name.substr(14));
            ss >> iktypeint;
            if( !!ss ) {
                IkParameterizationType iktype=static_cast<IkParameterizationType>(iktypeint);
                switch(iktype) {
                case IKP_TranslationXYOrientation3D: vrotaxes.push_back(itgroup->offset+2); break;
                default:
                    break;
                }
            }
        }
    }
    params->_diffstatefn = boost::bind(diffstatefn,_1,_2,boost::ref(vrotaxes));
    //params->_distmetricfn;
    params->_sExtraParameters += str(boost::format("<interpolation>%s</interpolation><hastimestamps>%d</hastimestamps>")%interpolation%hastimestamps);
    if( !planner->InitPlan(RobotBasePtr(),params) ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to InitPlan",ORE_Failed);
    }
    if( planner->PlanPath(traj) != PS_HasSolution ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to PlanPath",ORE_Failed);
    }
}

void ConvertTrajectorySpecification(TrajectoryBasePtr traj, const ConfigurationSpecification& spec)
{
    if( traj->GetConfigurationSpecification() != spec ) {
        size_t numpoints = traj->GetConfigurationSpecification().GetDOF() > 0 ? traj->GetNumWaypoints() : 0;
        vector<dReal> data;
        if( numpoints > 0 ) {
            traj->GetWaypoints(0,numpoints,data,spec);
        }
        traj->Init(spec);
        if( numpoints > 0 ) {
            traj->Insert(0,data);
        }
    }
}

TrajectoryBasePtr ReverseTrajectory(TrajectoryBaseConstPtr sourcetraj)
{
    vector<dReal> sourcedata;
    size_t numpoints = sourcetraj->GetNumWaypoints();
    int dof = sourcetraj->GetConfigurationSpecification().GetDOF();
    vector<uint8_t> velocitydofs(dof,0);
    int timeoffset = -1;
    FOREACHC(itgroup, sourcetraj->GetConfigurationSpecification()._vgroups) {
        if( itgroup->name.find("_velocities") != string::npos ) {
            for(int i = 0; i < itgroup->dof; ++i) {
                velocitydofs.at(itgroup->offset+i) = 1;
            }
        }
        else if( itgroup->name == "deltatime" ) {
            timeoffset = itgroup->offset;
        }
    }
    sourcetraj->GetWaypoints(0,numpoints,sourcedata);
    vector<dReal> targetdata(sourcedata.size());
    dReal prevdeltatime = 0;
    for(size_t i = 0; i < numpoints; ++i) {
        vector<dReal>::iterator ittarget = targetdata.begin()+i*dof;
        vector<dReal>::iterator itsource = sourcedata.begin()+(numpoints-i-1)*dof;
        for(int j = 0; j < dof; ++j) {
            if( velocitydofs[j] ) {
                *(ittarget+j) = -*(itsource+j);
            }
            else {
                *(ittarget+j) = *(itsource+j);
            }
        }

        if( timeoffset >= 0 ) {
            *(ittarget+timeoffset) = prevdeltatime;
            prevdeltatime = *(itsource+timeoffset);
        }
    }

    TrajectoryBasePtr traj = RaveCreateTrajectory(sourcetraj->GetEnv(),sourcetraj->GetXMLId());
    traj->Init(sourcetraj->GetConfigurationSpecification());
    traj->Insert(0,targetdata);
    return traj;
}

LineCollisionConstraint::LineCollisionConstraint()
{
    _report.reset(new CollisionReport());
}

bool LineCollisionConstraint::Check(PlannerBase::PlannerParametersWeakPtr _params, RobotBasePtr robot, const std::vector<dReal>& pQ0, const std::vector<dReal>& pQ1, IntervalType interval, PlannerBase::ConfigurationListPtr pvCheckedConfigurations)
{
    // set the bounds based on the interval type
    PlannerBase::PlannerParametersPtr params = _params.lock();
    if( !params ) {
        return false;
    }
    int start=0;
    bool bCheckEnd=false;
    switch (interval) {
    case IT_Open:
        start = 1;  bCheckEnd = false;
        break;
    case IT_OpenStart:
        start = 1;  bCheckEnd = true;
        break;
    case IT_OpenEnd:
        start = 0;  bCheckEnd = false;
        break;
    case IT_Closed:
        start = 0;  bCheckEnd = true;
        break;
    default:
        BOOST_ASSERT(0);
    }

    // first make sure the end is free
    _vtempconfig.resize(params->GetDOF());
    if (bCheckEnd) {
        params->_setstatefn(pQ1);
        if (robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot),_report) ) {
            return false;
        }
        if( robot->CheckSelfCollision(_report) ) {
            return false;
        }
    }

    // compute  the discretization
    dQ = pQ1;
    params->_diffstatefn(dQ,pQ0);
    int i, numSteps = 1;
    std::vector<dReal>::const_iterator itres = params->_vConfigResolution.begin();
    int totalsteps = 0;
    for (i = 0; i < params->GetDOF(); i++,itres++) {
        int steps;
        if( *itres != 0 ) {
            steps = (int)(fabs(dQ[i]) / *itres);
        }
        else {
            steps = (int)(fabs(dQ[i]) * 100);
        }
        totalsteps += steps;
        if (steps > numSteps) {
            numSteps = steps;
        }
    }
    if((totalsteps == 0)&&(start > 0)) {
        return true;
    }

    dReal fisteps = dReal(1.0f)/numSteps;
    for(std::vector<dReal>::iterator it = dQ.begin(); it != dQ.end(); ++it) {
        *it *= fisteps;
    }

    // check for collision along the straight-line path
    // NOTE: this does not check the end config, and may or may
    // not check the start based on the value of 'start'
    for (i = 0; i < params->GetDOF(); i++) {
        _vtempconfig[i] = pQ0[i];
    }
    if( start > 0 ) {
        if( !params->_neighstatefn(_vtempconfig, dQ,0) ) {
            return false;
        }
    }
    for (int f = start; f < numSteps; f++) {
        params->_setstatefn(_vtempconfig);
        if( robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot)) ) {
            return false;
        }
        if( robot->CheckSelfCollision() ) {
            return false;
        }
        if( !!params->_getstatefn ) {
            params->_getstatefn(_vtempconfig);     // query again in order to get normalizations/joint limits
        }
        if( !!pvCheckedConfigurations ) {
            pvCheckedConfigurations->push_back(_vtempconfig);
        }
        if( !params->_neighstatefn(_vtempconfig,dQ,0) ) {
            return false;
        }
    }

    if( bCheckEnd && !!pvCheckedConfigurations ) {
        pvCheckedConfigurations->push_back(pQ1);
    }
    return true;
}

SimpleDistanceMetric::SimpleDistanceMetric(RobotBasePtr robot) : _robot(robot)
{
    _robot->GetActiveDOFWeights(weights2);
    for(std::vector<dReal>::iterator it = weights2.begin(); it != weights2.end(); ++it) {
        *it *= *it;
    }
}

dReal SimpleDistanceMetric::Eval(const std::vector<dReal>& c0, const std::vector<dReal>& c1)
{
    std::vector<dReal> c = c0;
    _robot->SubtractActiveDOFValues(c,c1);
    dReal dist = 0;
    for(int i=0; i < _robot->GetActiveDOF(); i++) {
        dist += weights2.at(i)*c.at(i)*c.at(i);
    }
    return RaveSqrt(dist);
}

SimpleNeighborhoodSampler::SimpleNeighborhoodSampler(SpaceSamplerBasePtr psampler, const boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn) : _psampler(psampler), _distmetricfn(distmetricfn)
{
}

bool SimpleNeighborhoodSampler::Sample(std::vector<dReal>& vNewSample, const std::vector<dReal>& vCurSample, dReal fRadius)
{
    _psampler->SampleSequence(vNewSample);
    size_t dof = vCurSample.size();
    BOOST_ASSERT(dof==vNewSample.size() && &vNewSample != &vCurSample);
    dReal fDist = _distmetricfn(vNewSample,vCurSample);
    while(fDist > fRadius) {
        for (size_t i = 0; i < dof; i++) {
            vNewSample[i] = 0.5f*vCurSample[i]+0.5f*vNewSample[i];
        }
        fDist = _distmetricfn(vNewSample,vCurSample);
    }
    for(int iter = 0; iter < 20; ++iter) {
        for (size_t i = 0; i < dof; i++) {
            vNewSample[i] = 1.2f*vNewSample[i]-0.2f*vCurSample[i];
        }
        if(_distmetricfn(vNewSample, vCurSample) > fRadius ) {
            // take the previous
            for (size_t i = 0; i < dof; i++) {
                vNewSample[i] = 0.833333333333333f*vNewSample[i]-0.16666666666666669*vCurSample[i];
            }
            break;
        }
    }

    //        vNewSample.resize(lower.size());
    //        for (size_t i = 0; i < dof; i++) {
    //            if( sample[i] < lower[i] ) {
    //                vNewSample[i] = lower[i];
    //            }
    //            else if( sample[i] > upper[i] ) {
    //                vNewSample[i] = upper[i];
    //            }
    //            else {
    //                vNewSample[i] = sample[i];
    //            }
    //        }

    return true;
}

bool SimpleNeighborhoodSampler::Sample(std::vector<dReal>& samples)
{
    _psampler->SampleSequence(samples,1,IT_Closed);
    return samples.size()>0;
}


ManipulatorIKGoalSampler::ManipulatorIKGoalSampler(RobotBase::ManipulatorConstPtr pmanip, const std::list<IkParameterization>& listparameterizations, int nummaxsamples, int nummaxtries, dReal fsampleprob) : _pmanip(pmanip), _nummaxsamples(nummaxsamples), _nummaxtries(nummaxtries), _fsampleprob(fsampleprob)
{
    _probot = _pmanip->GetRobot();
    _pindexsampler = RaveCreateSpaceSampler(_probot->GetEnv(),"mt19937");
    int orgindex = 0;
    for(std::list<IkParameterization>::const_iterator it = listparameterizations.begin(); it != listparameterizations.end(); ++it) {
        SampleInfo s;
        s._orgindex = orgindex++;
        s._ikparam = *it;
        s._numleft = _nummaxsamples;
        _listsamples.push_back(s);
    }
    _report.reset(new CollisionReport());
    pmanip->GetIkSolver()->GetFreeParameters(_vfreestart);
    // the halton sequence centers around 0.5, so make it center around vfreestart
    for(std::vector<dReal>::iterator it = _vfreestart.begin(); it != _vfreestart.end(); ++it) {
        *it -= 0.5;
    }
}

bool ManipulatorIKGoalSampler::Sample(std::vector<dReal>& vgoal)
{
    vgoal.resize(0);
    std::vector<dReal> vindex;
    _pindexsampler->SampleSequence(vindex,1,IT_OpenEnd);
    if( vindex.at(0) > _fsampleprob ) {
        return false;
    }
    if( _viksolutions.size() > 0 ) {
        vgoal = _viksolutions.back();
        _viksolutions.pop_back();
        return true;
    }

    for(int itry = 0; itry < _nummaxtries; ++itry ) {
        if( _listsamples.size() == 0 ) {
            return false;
        }
        _pindexsampler->SampleSequence(vindex,1,IT_OpenEnd);
        int isampleindex = (int)(vindex.at(0)*_listsamples.size());
        std::list<SampleInfo>::iterator itsample = _listsamples.begin();
        advance(itsample,isampleindex);

        bool bCheckEndEffector = itsample->_ikparam.GetType() == IKP_Transform6D;
        // if first grasp, quickly prune grasp is end effector is in collision
        if( itsample->_numleft == _nummaxsamples && bCheckEndEffector ) {
            if( _pmanip->CheckEndEffectorCollision(itsample->_ikparam.GetTransform6D(),_report) ) {
                RAVELOG_VERBOSE(str(boost::format("sampleiksolutions gripper in collision: %s.\n")%_report->__str__()));
                _listsamples.erase(itsample);
                continue;
            }
        }

        std::vector<dReal> vfree;
        int orgindex = itsample->_orgindex;
        if( _pmanip->GetIkSolver()->GetNumFreeParameters() > 0 ) {
            if( !itsample->_psampler ) {
                itsample->_psampler = RaveCreateSpaceSampler(_probot->GetEnv(),"halton");
                itsample->_psampler->SetSpaceDOF(_pmanip->GetIkSolver()->GetNumFreeParameters());
            }
            itsample->_psampler->SampleSequence(vfree,1);
            for(size_t i = 0; i < _vfreestart.size(); ++i) {
                vfree.at(i) += _vfreestart[i];
                if( vfree[i] < 0 ) {
                    vfree[i] += 1;
                }
                if( vfree[i] > 1 ) {
                    vfree[i] -= 1;
                }
            }
        }
        bool bsuccess = _pmanip->FindIKSolutions(itsample->_ikparam, vfree, _viksolutions, IKFO_CheckEnvCollisions|(bCheckEndEffector ? IKFO_IgnoreEndEffectorCollisions : 0));
        if( --itsample->_numleft <= 0 || vfree.size() == 0 ) {
            _listsamples.erase(itsample);
        }

        if( bsuccess ) {
            for(size_t j = 0; j < _viksolutions.size(); ++j) {
                _listreturnedsamples.push_back(orgindex);
            }
            vgoal = _viksolutions.back();
            _viksolutions.pop_back();
            return true;
        }
    }
    return false;
}

int ManipulatorIKGoalSampler::GetIkParameterizationIndex(int index)
{
    BOOST_ASSERT(index >= 0 && index < (int)_listreturnedsamples.size());
    std::list<int>::iterator it = _listreturnedsamples.begin();
    advance(it,index);
    return *it;
}

void ManipulatorIKGoalSampler::SetSamplingProb(dReal fsampleprob)
{
    _fsampleprob = fsampleprob;
}

} // planningutils
} // OpenRAVE
