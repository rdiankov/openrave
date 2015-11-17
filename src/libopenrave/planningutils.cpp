// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include <boost/lexical_cast.hpp>
#include <openrave/planningutils.h>
#include <openrave/plannerparameters.h>

//#include <boost/iostreams/device/file_descriptor.hpp>
//#include <boost/iostreams/stream.hpp>
//#include <boost/version.hpp>

namespace OpenRAVE {
namespace planningutils {

static const dReal g_fEpsilonQuadratic = RavePow(g_fEpsilon,0.55); // should be 0.6...perhaps this is related to parabolic smoother epsilons?

int JitterActiveDOF(RobotBasePtr robot,int nMaxIterations,dReal fRand,const PlannerBase::PlannerParameters::NeighStateFn& neighstatefn)
{
    RAVELOG_VERBOSE("starting jitter active dof...\n");
    vector<dReal> curdof, newdof, deltadof, deltadof2;
    robot->GetActiveDOFValues(curdof);
    newdof.resize(curdof.size());
    deltadof.resize(curdof.size(),0);
    CollisionReport report;
    CollisionReportPtr preport(&report,utils::null_deleter());
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
                robot->SetActiveDOFValues(curdof,KinBody::CLA_CheckLimitsSilent);
                return -1;
            }
        }
        else {
            for(size_t i = 0; i < newdof.size(); ++i) {
                newdof[i] = curdof[i]+*itperturbation;
            }
        }
        robot->SetActiveDOFValues(newdof,KinBody::CLA_CheckLimitsSilent);

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
        // have to restore to initial non-perturbed configuration!
        robot->SetActiveDOFValues(curdof,KinBody::CLA_Nothing);
        return -1;
    }

    deltadof2.resize(curdof.size(),0);
    for(int iter = 0; iter < nMaxIterations; ++iter) {
        for(size_t j = 0; j < newdof.size(); ++j) {
            deltadof[j] = fRand * (RaveRandomFloat()-0.5f);
        }
        bCollision = false;
        bool bConstraintFailed = false;
        FOREACH(itperturbation,perturbations) {
            for(size_t j = 0; j < deltadof.size(); ++j) {
                deltadof2[j] = deltadof[j] + *itperturbation;
            }
            if( bConstraint ) {
                newdof = curdof;
                robot->SetActiveDOFValues(newdof,KinBody::CLA_CheckLimitsSilent);
                if( !neighstatefn(newdof,deltadof2,0) ) {
                    if( *itperturbation != 0 ) {
                        RAVELOG_DEBUG(str(boost::format("constraint function failed, pert=%e\n")%*itperturbation));
                    }
                    bConstraintFailed = true;
                    break;
                }
            }
            else {
                for(size_t j = 0; j < deltadof.size(); ++j) {
                    newdof[j] = curdof[j] + deltadof2[j];
                }
            }
            robot->SetActiveDOFValues(newdof,KinBody::CLA_CheckLimitsSilent);
            if(robot->CheckSelfCollision() ) {
                bCollision = true;
                break;
            }
            if( robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot)) ) {
                bCollision = true;
                break;
            }
        }
        if( !bCollision && !bConstraintFailed ) {
            // have to restore to non-perturbed configuration!
            if( bConstraint ) {
                newdof = curdof;
                robot->SetActiveDOFValues(newdof,KinBody::CLA_Nothing);
                if( !neighstatefn(newdof,deltadof,0) ) {
                    RAVELOG_WARN("neighstatefn failed, but previously succeeded\n");
                    continue;
                }
            }
            else {
                for(size_t j = 0; j < deltadof.size(); ++j) {
                    newdof[j] = curdof[j] + deltadof[j];
                }
            }
            robot->SetActiveDOFValues(newdof,KinBody::CLA_CheckLimitsSilent);
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                robot->GetActiveDOFValues(newdof);
                stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                for(size_t i = 0; i < newdof.size(); ++i ) {
                    if( i > 0 ) {
                        ss << "," << newdof[i];
                    }
                    else {
                        ss << "jitteredvalues=[" << newdof[i];
                    }
                }
                ss << "]";
                RAVELOG_VERBOSE(ss.str());
            }

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
            // have to restore to initial non-perturbed transform!
            pbody->SetTransform(transorig);
            return false;
        }
        if( iter > 0 && fJitter <= 0 ) {
            // have to restore to initial non-perturbed transform!
            pbody->SetTransform(transorig);
            return false;
        }
        transnew.trans = transorig.trans + 2.0 * fJitter * Vector(RaveRandomFloat()-0.5f, RaveRandomFloat()-0.5f, RaveRandomFloat()-0.5f);
        pbody->SetTransform(transnew);
        ++iter;
    }

    return true;
}

int JitterCurrentConfiguration(PlannerBase::PlannerParametersConstPtr parameters, int maxiterations, dReal maxjitter, dReal perturbation)
{
    std::vector<dReal> curdof, newdof, deltadof, deltadof2, zerodof;
    parameters->_getstatefn(curdof);
    newdof.resize(curdof.size());
    deltadof.resize(curdof.size(),0);
    zerodof.resize(curdof.size(),0);
    CollisionReport report;
    CollisionReportPtr preport(&report,utils::null_deleter());
    bool bCollision = false;
    bool bConstraint = !!parameters->_neighstatefn;

    // have to test with perturbations since very small changes in angles can produce collision inconsistencies
    std::vector<dReal> perturbations;
    if( perturbation > 0 ) {
        perturbations.resize(3,0);
        perturbations[1] = perturbation;
        perturbations[2] = -perturbation;
    }
    else {
        perturbations.resize(1,0);
    }
    FOREACH(itperturbation,perturbations) {
        if( bConstraint ) {
            FOREACH(it,deltadof) {
                *it = *itperturbation;
            }
            newdof = curdof;
            if( !parameters->_neighstatefn(newdof,deltadof,0) ) {
                int setret = parameters->SetStateValues(curdof, 0);
                if( setret != 0 ) {
                    // state failed to set, this could mean the initial state is just really bad, so resume jittering
                    bCollision = true;
                    break;
                }
                return -1;
            }
        }
        else {
            for(size_t i = 0; i < newdof.size(); ++i) {
                newdof[i] = curdof[i]+*itperturbation;
                if( newdof[i] > parameters->_vConfigUpperLimit.at(i) ) {
                    newdof[i] = parameters->_vConfigUpperLimit.at(i);
                }
                else if( newdof[i] < parameters->_vConfigLowerLimit.at(i) ) {
                    newdof[i] = parameters->_vConfigLowerLimit.at(i);
                }
            }
        }

        // don't need to set state since CheckPathAllConstraints does it
        if( parameters->CheckPathAllConstraints(newdof,newdof,zerodof,zerodof,0,IT_OpenStart) != 0 ) {
            bCollision = true;
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                ss << "constraints failed, ";
                for(size_t i = 0; i < newdof.size(); ++i ) {
                    if( i > 0 ) {
                        ss << "," << newdof[i];
                    }
                    else {
                        ss << "colvalues=[" << newdof[i];
                    }
                }
                ss << "]";
                RAVELOG_VERBOSE(ss.str());
            }
            break;
        }
    }

    if( !bCollision || maxjitter <= 0 ) {
        // have to restore to initial non-perturbed configuration!
        if( parameters->SetStateValues(curdof, 0) != 0 ) {
            RAVELOG_WARN("failed to restore old values\n");
        }
        return -1;
    }

    FOREACHC(itsampler, parameters->_listInternalSamplers) {
        (*itsampler)->SetSeed(parameters->_nRandomGeneratorSeed);
    }
    // reset the samplers with the seed, possibly there's some way to cache?
    std::vector<dReal> vLimitOneThird(parameters->_vConfigLowerLimit.size()), vLimitTwoThirds(parameters->_vConfigLowerLimit.size());
    for(size_t i = 0; i < vLimitOneThird.size(); ++i) {
        vLimitOneThird[i] = (2*parameters->_vConfigLowerLimit[i] + parameters->_vConfigUpperLimit[i])/3.0;
        vLimitTwoThirds[i] = (parameters->_vConfigLowerLimit[i] + 2*parameters->_vConfigUpperLimit[i])/3.0;
    }
    deltadof2.resize(curdof.size(),0);
    dReal imaxiterations = 1.0/dReal(maxiterations);
    for(int iter = 0; iter < maxiterations; ++iter) {
        // ramp of the jitter as iterations increase
        dReal jitter = maxjitter;
        if( iter < maxiterations/2 ) {
            jitter = maxjitter*dReal(iter)*(2.0*imaxiterations);
        }
        parameters->_samplefn(deltadof);
        // check which third the sampled dof is in
        for(size_t j = 0; j < newdof.size(); ++j) {
            if( deltadof[j] < vLimitOneThird[j] ) {
                deltadof[j] = -jitter;
            }
            else if( deltadof[j] > vLimitTwoThirds[j] ) {
                deltadof[j] = jitter;
            }
            else {
                deltadof[j] = 0;
            }
        }
        bCollision = false;
        bool bConstraintFailed = false;
        FOREACH(itperturbation,perturbations) {
            for(size_t j = 0; j < deltadof.size(); ++j) {
                deltadof2[j] = deltadof[j] + *itperturbation;
            }
            if( bConstraint ) {
                newdof = curdof;
                if( parameters->SetStateValues(newdof, 0) != 0 ) {
                    bConstraintFailed = true;
                    break;
                }
                if( !parameters->_neighstatefn(newdof,deltadof2,0) ) {
                    if( *itperturbation != 0 ) {
                        RAVELOG_DEBUG(str(boost::format("constraint function failed, pert=%e\n")%*itperturbation));
                    }
                    bConstraintFailed = true;
                    break;
                }
            }
            else {
                for(size_t j = 0; j < deltadof.size(); ++j) {
                    newdof[j] = curdof[j] + deltadof2[j];
                    if( newdof[j] > parameters->_vConfigUpperLimit.at(j) ) {
                        newdof[j] = parameters->_vConfigUpperLimit.at(j);
                    }
                    else if( newdof[j] < parameters->_vConfigLowerLimit.at(j) ) {
                        newdof[j] = parameters->_vConfigLowerLimit.at(j);
                    }
                }
            }
            // don't need to set state since CheckPathAllConstraints does it
            if( parameters->CheckPathAllConstraints(newdof,newdof,zerodof,zerodof,0,IT_OpenStart) != 0 ) {
                bCollision = true;
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                    ss << "constraints failed, ";
                    for(size_t i = 0; i < newdof.size(); ++i ) {
                        if( i > 0 ) {
                            ss << "," << newdof[i];
                        }
                        else {
                            ss << "colvalues=[" << newdof[i];
                        }
                    }
                    ss << "]";
                    RAVELOG_VERBOSE(ss.str());
                }
                break;
            }
        }
        if( !bCollision && !bConstraintFailed ) {
            // have to restore to non-perturbed configuration!
            if( bConstraint ) {
                newdof = curdof;
                if( parameters->SetStateValues(newdof, 0) != 0 ) {
                    // get another state
                    continue;
                }
                if( !parameters->_neighstatefn(newdof,deltadof,0) ) {
                    RAVELOG_WARN("neighstatefn failed, but previously succeeded\n");
                    continue;
                }
            }
            else {
                for(size_t j = 0; j < deltadof.size(); ++j) {
                    newdof[j] = curdof[j] + deltadof[j];
                    if( newdof[j] > parameters->_vConfigUpperLimit.at(j) ) {
                        newdof[j] = parameters->_vConfigUpperLimit.at(j);
                    }
                    else if( newdof[j] < parameters->_vConfigLowerLimit.at(j) ) {
                        newdof[j] = parameters->_vConfigLowerLimit.at(j);
                    }
                }
            }
            if( parameters->SetStateValues(newdof, 0) != 0 ) {
                // get another state
                continue;
            }
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                parameters->_getstatefn(newdof);
                stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                for(size_t i = 0; i < newdof.size(); ++i ) {
                    if( i > 0 ) {
                        ss << "," << newdof[i];
                    }
                    else {
                        ss << "jitteredvalues=[" << newdof[i];
                    }
                }
                ss << "]";
                RAVELOG_VERBOSE(ss.str());
            }

            return 1;
        }
    }

    return 0;
}

class TrajectoryVerifier
{
public:
    TrajectoryVerifier(PlannerBase::PlannerParametersConstPtr parameters) : _parameters(parameters) {
        VerifyParameters();
    }

    void VerifyParameters() {
        OPENRAVE_ASSERT_FORMAT0(!!_parameters,"need planner parameters to verify trajectory",ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT0((int)_parameters->_vConfigLowerLimit.size(), ==, _parameters->GetDOF(), "unexpected size",ORE_InvalidState);
        OPENRAVE_ASSERT_OP_FORMAT0((int)_parameters->_vConfigUpperLimit.size(), ==, _parameters->GetDOF(), "unexpected size",ORE_InvalidState);
        OPENRAVE_ASSERT_OP_FORMAT0((int)_parameters->_vConfigResolution.size(), ==, _parameters->GetDOF(), "unexpected size",ORE_InvalidState);
    }

    void VerifyTrajectory(TrajectoryBaseConstPtr trajectory, dReal samplingstep)
    {
        OPENRAVE_ASSERT_FORMAT0(!!trajectory,"need valid trajectory",ORE_InvalidArguments);

        ConfigurationSpecification velspec =  _parameters->_configurationspecification.ConvertToVelocitySpecification();

        dReal fresolutionmean = 0;
        FOREACHC(it,_parameters->_vConfigResolution) {
            fresolutionmean += *it;
        }
        fresolutionmean /= _parameters->_vConfigResolution.size();

        dReal fthresh = 5e-5f;
        vector<dReal> deltaq(_parameters->GetDOF(),0);
        std::vector<dReal> vdata, vdatavel, vdiff;
        for(size_t ipoint = 0; ipoint < trajectory->GetNumWaypoints(); ++ipoint) {
            trajectory->GetWaypoint(ipoint,vdata,_parameters->_configurationspecification);
            trajectory->GetWaypoint(ipoint,vdatavel,velspec);
            BOOST_ASSERT((int)vdata.size()==_parameters->GetDOF());
            BOOST_ASSERT((int)vdatavel.size()==_parameters->GetDOF());
            for(size_t i = 0; i < vdata.size(); ++i) {
                if( !(vdata[i] >= _parameters->_vConfigLowerLimit[i]-fthresh) || !(vdata[i] <= _parameters->_vConfigUpperLimit[i]+fthresh) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("limits exceeded configuration %d dof %d: %f in [%f,%f]"), ipoint%i%vdata[i]%_parameters->_vConfigLowerLimit[i]%_parameters->_vConfigUpperLimit[i], ORE_InconsistentConstraints);
                }
            }
            for(size_t i = 0; i < _parameters->_vConfigVelocityLimit.size(); ++i) {
                if( !(RaveFabs(vdatavel.at(i)) <= _parameters->_vConfigVelocityLimit[i]+fthresh) ) { // !(x<=y) necessary for catching NaNs
                    throw OPENRAVE_EXCEPTION_FORMAT(_("velocity exceeded configuration %d dof %d: %f>%f"), ipoint%i%RaveFabs(vdatavel.at(i))%_parameters->_vConfigVelocityLimit[i], ORE_InconsistentConstraints);
                }
            }
            if( _parameters->SetStateValues(vdata, 0) != 0 ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("failed to set state values"), ORE_InconsistentConstraints);
            }
            vector<dReal> newq;
            _parameters->_getstatefn(newq);
            BOOST_ASSERT(vdata.size() == newq.size());
            vdiff = newq;
            _parameters->_diffstatefn(vdiff,vdata);
            for(size_t i = 0; i < vdiff.size(); ++i) {
                if( !(RaveFabs(vdiff.at(i)) <= 0.001 * _parameters->_vConfigResolution[i]) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("setstate/getstate inconsistent configuration %d dof %d: %f != %f, wrote trajectory to %s"),ipoint%i%vdata.at(i)%newq.at(i)%DumpTrajectory(trajectory),ORE_InconsistentConstraints);
                }
            }
            if( !!_parameters->_neighstatefn ) {
                newq = vdata;
                if( !_parameters->_neighstatefn(newq,vdiff,0) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("neighstatefn is rejecting configuration %d, wrote trajectory %s"),ipoint%DumpTrajectory(trajectory),ORE_InconsistentConstraints);
                }
                dReal fdist = _parameters->_distmetricfn(newq,vdata);
                OPENRAVE_ASSERT_OP_FORMAT(fdist,<=,0.01 * fresolutionmean, "neighstatefn is rejecting configuration %d, wrote trajectory %s",ipoint%DumpTrajectory(trajectory),ORE_InconsistentConstraints);
            }
        }

        if( !!_parameters->_checkpathvelocityconstraintsfn && trajectory->GetNumWaypoints() >= 2 ) {

            if( trajectory->GetDuration() > 0 && samplingstep > 0 ) {
                // use sampling and check segment constraints
                // have to make sure that sampling interval doesn't include a waypoint. otherwise interpolation could become inconsistent!
                std::vector<dReal> vabstimes, vsampletimes;
                vabstimes.reserve(trajectory->GetNumWaypoints() + (trajectory->GetDuration()/samplingstep) + 1);
                ConfigurationSpecification deltatimespec;
                deltatimespec.AddDeltaTimeGroup();
                trajectory->GetWaypoints(0, trajectory->GetNumWaypoints(), vabstimes, deltatimespec);
                dReal totaltime = 0;
                FOREACH(ittime, vabstimes) {
                    totaltime += *ittime;
                    *ittime = totaltime;
                }
                for(dReal ftime = 0; ftime < trajectory->GetDuration(); ftime += samplingstep ) {
                    vabstimes.push_back(ftime);
                }
                vsampletimes.resize(vabstimes.size());
                std::merge(vabstimes.begin(), vabstimes.begin()+trajectory->GetNumWaypoints(), vabstimes.begin()+trajectory->GetNumWaypoints(), vabstimes.end(), vsampletimes.begin());
                std::vector<dReal> vprevdata, vprevdatavel;
                ConstraintFilterReturnPtr filterreturn(new ConstraintFilterReturn());
                trajectory->Sample(vprevdata,0,_parameters->_configurationspecification);
                trajectory->Sample(vprevdatavel,0,velspec);
                std::vector<dReal>::iterator itprevtime = vsampletimes.begin();
                for(std::vector<dReal>::iterator itsampletime = vsampletimes.begin()+1; itsampletime != vsampletimes.end(); ++itsampletime) {
                    if (*itsampletime < *itprevtime + 1e-5 ) {
                        continue;
                    }
                    filterreturn->Clear();
                    trajectory->Sample(vdata,*itsampletime,_parameters->_configurationspecification);
                    trajectory->Sample(vdatavel,*itsampletime,velspec);
                    dReal deltatime = *itsampletime - *itprevtime;
                    vdiff = vdata;
                    _parameters->_diffstatefn(vdiff,vprevdata);
                    for(size_t i = 0; i < _parameters->_vConfigVelocityLimit.size(); ++i) {
                        dReal velthresh = _parameters->_vConfigVelocityLimit.at(i)*deltatime+fthresh;
                        OPENRAVE_ASSERT_OP_FORMAT(RaveFabs(vdiff.at(i)), <=, velthresh, "time %fs-%fs, dof %d traveled %f, but maxvelocity only allows %f, wrote trajectory to %s",*itprevtime%*itsampletime%i%RaveFabs(vdiff.at(i))%velthresh%DumpTrajectory(trajectory),ORE_InconsistentConstraints);
                    }
                    if( _parameters->CheckPathAllConstraints(vprevdata,vdata,vprevdatavel, vdatavel, deltatime, IT_Closed, 0xffff|CFO_FillCheckedConfiguration, filterreturn) != 0 ) {
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            _parameters->CheckPathAllConstraints(vprevdata,vdata,vprevdatavel, vdatavel, deltatime, IT_Closed, 0xffff|CFO_FillCheckedConfiguration, filterreturn);
                        }
                        throw OPENRAVE_EXCEPTION_FORMAT(_("time %fs-%fs, CheckPathAllConstraints failed, wrote trajectory to %s"),*itprevtime%*itsampletime%DumpTrajectory(trajectory),ORE_InconsistentConstraints);
                    }
                    OPENRAVE_ASSERT_OP(filterreturn->_configurations.size()%_parameters->GetDOF(),==,0);
                    std::vector<dReal>::iterator itprevconfig = filterreturn->_configurations.begin();
                    std::vector<dReal>::iterator itcurconfig = itprevconfig + _parameters->GetDOF();
                    for(; itcurconfig != filterreturn->_configurations.end(); itcurconfig += _parameters->GetDOF()) {
                        std::vector<dReal> vprevconfig(itprevconfig,itprevconfig+_parameters->GetDOF());
                        std::vector<dReal> vcurconfig(itcurconfig,itcurconfig+_parameters->GetDOF());
                        for(int i = 0; i < _parameters->GetDOF(); ++i) {
                            deltaq.at(i) = vcurconfig.at(i) - vprevconfig.at(i);
                        }
                        if( _parameters->SetStateValues(vprevconfig, 0) != 0 ) {
                            throw OPENRAVE_EXCEPTION_FORMAT0(_("time %fs-%fs, failed to set state values"), ORE_InconsistentConstraints);
                        }
                        vector<dReal> vtemp = vprevconfig;
                        if( !_parameters->_neighstatefn(vtemp,deltaq,0) ) {
                            throw OPENRAVE_EXCEPTION_FORMAT(_("time %fs-%fs, neighstatefn is rejecting configurations from CheckPathAllConstraints, wrote trajectory to %s"),*itprevtime%*itsampletime%DumpTrajectory(trajectory),ORE_InconsistentConstraints);
                        }
                        else {
                            dReal fprevdist = _parameters->_distmetricfn(vprevconfig,vtemp);
                            dReal fcurdist = _parameters->_distmetricfn(vcurconfig,vtemp);
                            if( fprevdist > g_fEpsilonLinear ) {
                                OPENRAVE_ASSERT_OP_FORMAT(fprevdist, >, fcurdist, "time %fs-%fs, neightstatefn returned a configuration closer to the previous configuration %f than the expected current %f, wrote trajectory to %s",*itprevtime%*itsampletime%fprevdist%fcurdist%DumpTrajectory(trajectory), ORE_InconsistentConstraints);
                            }
                        }
                        itprevconfig=itcurconfig;
                    }
                    vprevdata.swap(vdata);
                    vprevdatavel.swap(vdatavel);
                    itprevtime = itsampletime;
                }
            }
            else {
                for(size_t i = 0; i < trajectory->GetNumWaypoints(); ++i) {
                    trajectory->GetWaypoint(i,vdata,_parameters->_configurationspecification);
                    if( _parameters->CheckPathAllConstraints(vdata,vdata,std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart) != 0 ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("CheckPathAllConstraints, failed at %d, wrote trajectory to %s"),i%DumpTrajectory(trajectory),ORE_InconsistentConstraints);
                    }
                }
            }
        }
    }

    string DumpTrajectory(TrajectoryBaseConstPtr trajectory)
    {
        string filename = str(boost::format("%s/failedtrajectory%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%1000));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        trajectory->serialize(f);
        return filename;
    }

protected:
    PlannerBase::PlannerParametersConstPtr _parameters;
};

void VerifyTrajectory(PlannerBase::PlannerParametersConstPtr parameters, TrajectoryBaseConstPtr trajectory, dReal samplingstep)
{
    EnvironmentMutex::scoped_lock lockenv(trajectory->GetEnv()->GetMutex());
    if( !parameters ) {
        PlannerBase::PlannerParametersPtr newparams(new PlannerBase::PlannerParameters());
        newparams->SetConfigurationSpecification(trajectory->GetEnv(), trajectory->GetConfigurationSpecification().GetTimeDerivativeSpecification(0));
        parameters = newparams;
    }
    TrajectoryVerifier v(parameters);
    v.VerifyTrajectory(trajectory,samplingstep);
}

PlannerStatus _PlanActiveDOFTrajectory(TrajectoryBasePtr traj, RobotBasePtr probot, bool hastimestamps, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, bool bsmooth, const std::string& plannerparameters)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need velocities, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PS_HasSolution;
    }

    EnvironmentBasePtr env = traj->GetEnv();
    EnvironmentMutex::scoped_lock lockenv(env->GetMutex());
    CollisionOptionsStateSaver optionstate(env->GetCollisionChecker(),env->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);
    PlannerBasePtr planner = RaveCreatePlanner(env,plannername.size() > 0 ? plannername : string("parabolicsmoother"));
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->SetRobotActiveJoints(probot);
    FOREACH(it,params->_vConfigVelocityLimit) {
        *it *= fmaxvelmult;
    }
    FOREACH(it,params->_vConfigAccelerationLimit) {
        *it *= fmaxaccelmult;
    }
    if( !bsmooth ) {
        params->_setstatevaluesfn.clear();
        params->_setstatefn.clear();
        params->_checkpathconstraintsfn.clear();
        params->_checkpathvelocityconstraintsfn.clear();
    }

    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = hastimestamps;
    params->_sExtraParameters += plannerparameters;
    if( !planner->InitPlan(probot,params) ) {
        return PS_Failed;
    }
    if( planner->PlanPath(traj) != PS_HasSolution ) {
        return PS_Failed;
    }

    if( bsmooth && (RaveGetDebugLevel() & Level_VerifyPlans) ) {
        RobotBase::RobotStateSaver saver(probot);
        planningutils::VerifyTrajectory(params,traj);
    }
    return PS_HasSolution;
}

ActiveDOFTrajectorySmoother::ActiveDOFTrajectorySmoother(RobotBasePtr robot, const std::string& _plannername, const std::string& plannerparameters)
{
    std::string plannername = _plannername.size() > 0 ? _plannername : "parabolicsmoother";
    _robot = robot;
    EnvironmentMutex::scoped_lock lockenv(robot->GetEnv()->GetMutex());
    _vRobotActiveIndices = _robot->GetActiveDOFIndices();
    _nRobotAffineDOF = _robot->GetAffineDOF();
    _vRobotRotationAxis = _robot->GetAffineRotationAxis();
    _planner = RaveCreatePlanner(robot->GetEnv(),plannername);
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->SetRobotActiveJoints(_robot);
    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = false;
    params->_sExtraParameters += plannerparameters;
    if( !_planner->InitPlan(_robot,params) ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s"), plannername%_robot->GetName(), ORE_InvalidArguments);
    }
    _parameters=params; // necessary because SetRobotActiveJoints builds functions that hold weak_ptr to the parameters
    _changehandler = robot->RegisterChangeCallback(KinBody::Prop_JointAccelerationVelocityTorqueLimits|KinBody::Prop_JointLimits|KinBody::Prop_JointProperties, boost::bind(&ActiveDOFTrajectorySmoother::_UpdateParameters, this));
}

PlannerStatus ActiveDOFTrajectorySmoother::PlanPath(TrajectoryBasePtr traj)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need velocities, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PS_HasSolution;
    }

    EnvironmentBasePtr env = traj->GetEnv();
    CollisionOptionsStateSaver optionstate(env->GetCollisionChecker(),env->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);
    PlannerStatus status = _planner->PlanPath(traj);
    if( status & PS_HasSolution ) {
        if( RaveGetDebugLevel() & Level_VerifyPlans ) {
            RobotBase::RobotStateSaver saver(_robot);
            planningutils::VerifyTrajectory(_planner->GetParameters(),traj);
        }
    }
    return status;
}

void ActiveDOFTrajectorySmoother::_UpdateParameters()
{
    RobotBase::RobotStateSaver saver(_robot, KinBody::Save_ActiveDOF);
    _robot->SetActiveDOFs(_vRobotActiveIndices, _nRobotAffineDOF, _vRobotRotationAxis);
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->SetRobotActiveJoints(_robot);
    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = false;
    params->_sExtraParameters = _parameters->_sExtraParameters;
    if( !_planner->InitPlan(_robot,params) ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s"), _planner->GetXMLId()%_robot->GetName(), ORE_InvalidArguments);
    }
    _parameters=params; // necessary because SetRobotActiveJoints builds functions that hold weak_ptr to the parameters
}

ActiveDOFTrajectoryRetimer::ActiveDOFTrajectoryRetimer(RobotBasePtr robot, const std::string& _plannername, const std::string& plannerparameters)
{
    std::string plannername = _plannername.size() > 0 ? _plannername : "parabolicretimer";
    _robot = robot;
    EnvironmentMutex::scoped_lock lockenv(robot->GetEnv()->GetMutex());
    _vRobotActiveIndices = _robot->GetActiveDOFIndices();
    _nRobotAffineDOF = _robot->GetAffineDOF();
    _vRobotRotationAxis = _robot->GetAffineRotationAxis();
    _planner = RaveCreatePlanner(robot->GetEnv(),plannername);
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->SetRobotActiveJoints(_robot);
    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = false;
    params->_setstatevaluesfn.clear();
    params->_setstatefn.clear();
    params->_checkpathconstraintsfn.clear();
    params->_checkpathvelocityconstraintsfn.clear();
    params->_sExtraParameters = plannerparameters;
    if( !_planner->InitPlan(_robot,params) ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s"), plannername%_robot->GetName(), ORE_InvalidArguments);
    }
    _parameters=params; // necessary because SetRobotActiveJoints builds functions that hold weak_ptr to the parameters
    _changehandler = robot->RegisterChangeCallback(KinBody::Prop_JointAccelerationVelocityTorqueLimits|KinBody::Prop_JointLimits|KinBody::Prop_JointProperties, boost::bind(&ActiveDOFTrajectoryRetimer::_UpdateParameters, this));
}

PlannerStatus ActiveDOFTrajectoryRetimer::PlanPath(TrajectoryBasePtr traj, bool hastimestamps)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need velocities, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PS_HasSolution;
    }

    TrajectoryTimingParametersPtr parameters = boost::dynamic_pointer_cast<TrajectoryTimingParameters>(_parameters);
    if( parameters->_hastimestamps != hastimestamps ) {
        parameters->_hastimestamps = hastimestamps;
        if( !_planner->InitPlan(_robot,parameters) ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s"), _planner->GetXMLId()%_robot->GetName(), ORE_InvalidArguments);
        }
    }

    return _planner->PlanPath(traj);
}

void ActiveDOFTrajectoryRetimer::_UpdateParameters()
{
    RobotBase::RobotStateSaver saver(_robot, KinBody::Save_ActiveDOF);
    _robot->SetActiveDOFs(_vRobotActiveIndices, _nRobotAffineDOF, _vRobotRotationAxis);
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->SetRobotActiveJoints(_robot);
    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = false;
    params->_setstatevaluesfn.clear();
    params->_setstatefn.clear();
    params->_checkpathconstraintsfn.clear();
    params->_checkpathvelocityconstraintsfn.clear();
    params->_sExtraParameters = _parameters->_sExtraParameters;
    if( !_planner->InitPlan(_robot,params) ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s"), _planner->GetXMLId()%_robot->GetName(), ORE_InvalidArguments);
    }
    _parameters=params; // necessary because SetRobotActiveJoints builds functions that hold weak_ptr to the parameters
}

PlannerStatus _PlanTrajectory(TrajectoryBasePtr traj, bool hastimestamps, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, bool bsmooth, const std::string& plannerparameters)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need velocities, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PS_HasSolution;
    }

    EnvironmentMutex::scoped_lock lockenv(traj->GetEnv()->GetMutex());
    PlannerBasePtr planner = RaveCreatePlanner(traj->GetEnv(),plannername.size() > 0 ? plannername : string("parabolicsmoother"));
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->SetConfigurationSpecification(traj->GetEnv(),traj->GetConfigurationSpecification().GetTimeDerivativeSpecification(0));
    FOREACH(it,params->_vConfigVelocityLimit) {
        *it *= fmaxvelmult;
    }
    FOREACH(it,params->_vConfigAccelerationLimit) {
        *it *= fmaxaccelmult;
    }
    if( !bsmooth ) {
        params->_setstatevaluesfn.clear();
        params->_setstatefn.clear();
        params->_checkpathconstraintsfn.clear();
        params->_checkpathvelocityconstraintsfn.clear();
    }

    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = hastimestamps;
    params->_sExtraParameters += plannerparameters;
    if( !planner->InitPlan(RobotBasePtr(),params) ) {
        return PS_Failed;
    }
    if( planner->PlanPath(traj) != PS_HasSolution ) {
        return PS_Failed;
    }

    if( bsmooth && (RaveGetDebugLevel() & Level_VerifyPlans) ) {
        PlannerBase::PlannerParameters::StateSaver saver(params);
        planningutils::VerifyTrajectory(params,traj);
    }
    return PS_HasSolution;
}

static void diffstatefn(std::vector<dReal>& q1, const std::vector<dReal>& q2, const std::vector<int> vrotaxes)
{
    BOOST_ASSERT(q1.size()==q2.size());
    for(size_t i = 0; i < q1.size(); ++i) {
        if( find(vrotaxes.begin(),vrotaxes.end(),i) != vrotaxes.end() ) {
            q1[i] = utils::SubtractCircularAngle(q1[i],q2[i]);
        }
        else {
            q1[i] -= q2[i];
        }
    }
}

static void _SetTransformBody(std::vector<dReal>::const_iterator itvalues, KinBodyPtr pbody, int index, int affinedofs,const Vector& vActvAffineRotationAxis)
{
    Transform t = pbody->GetTransform();
    RaveGetTransformFromAffineDOFValues(t,itvalues+index,affinedofs,vActvAffineRotationAxis);
    pbody->SetTransform(t);
}

static void _GetTransformBody(std::vector<dReal>::iterator itvalues, KinBodyPtr pbody, int index, int affinedofs,const Vector& vActvAffineRotationAxis)
{
    Transform t = pbody->GetTransform();
    RaveGetAffineDOFValuesFromTransform(itvalues+index,t, affinedofs,vActvAffineRotationAxis);
}

static dReal _ComputeTransformBodyDistance(std::vector<dReal>::const_iterator itvalues0, std::vector<dReal>::const_iterator itvalues1, KinBodyPtr pbody, int index, int affinedofs,const Vector& vActvAffineRotationAxis)
{
    Transform t0 = pbody->GetTransform();
    Transform t1 = t0;
    RaveGetTransformFromAffineDOFValues(t0,itvalues0+index,affinedofs,vActvAffineRotationAxis);
    RaveGetTransformFromAffineDOFValues(t1,itvalues1+index,affinedofs,vActvAffineRotationAxis);
    return TransformDistance2(t0, t1, 0.3);
}

int _SetAffineState(const std::list< boost::function< void(std::vector<dReal>::const_iterator) > >& listsetfunctions, const std::vector<dReal>& v, int options)
{
    FOREACHC(itfn,listsetfunctions) {
        (*itfn)(v.begin());
    }
    return 0;
}

void _GetAffineState(std::vector<dReal>& v, size_t expectedsize, const std::list< boost::function< void(std::vector<dReal>::iterator) > >& listgetfunctions)
{
    v.resize(expectedsize);
    FOREACHC(itfn,listgetfunctions) {
        (*itfn)(v.begin());
    }
}

dReal _ComputeAffineDistanceMetric(const std::vector<dReal>& v0, const std::vector<dReal>& v1, const std::list< boost::function< dReal(std::vector<dReal>::const_iterator, std::vector<dReal>::const_iterator) > >& listdistfunctions)
{
    dReal dist = 0;
    FOREACHC(itfn,listdistfunctions) {
        dist += (*itfn)(v0.begin(), v1.begin());
    }
    return dist;
}

class PlannerStateSaver
{
public:
    PlannerStateSaver(int dof, const PlannerBase::PlannerParameters::SetStateValuesFn& setfn, const PlannerBase::PlannerParameters::GetStateFn& getfn) : _setfn(setfn) {
        _savedvalues.resize(dof);
        getfn(_savedvalues);
        BOOST_ASSERT(!!_setfn);
    }
    virtual ~PlannerStateSaver() {
        int ret = _setfn(_savedvalues, 0);
        if( ret != 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("failed to set state in PlannerStateSaver, return=%d"), ret, ORE_Assert);
        }
    }

private:
    const PlannerBase::PlannerParameters::SetStateValuesFn& _setfn;
    vector<dReal> _savedvalues;
};

// this function is very messed up...?
static PlannerStatus _PlanAffineTrajectory(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, bool hastimestamps, const std::string& plannername, bool bsmooth, const std::string& plannerparameters)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need retiming, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PS_HasSolution;
    }

    EnvironmentMutex::scoped_lock lockenv(traj->GetEnv()->GetMutex());
    ConfigurationSpecification newspec = traj->GetConfigurationSpecification().GetTimeDerivativeSpecification(0);
    if( newspec.GetDOF() != (int)maxvelocities.size() || newspec.GetDOF() != (int)maxaccelerations.size() ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("traj values (%d) do not match maxvelocity size (%d) or maxaccelerations size (%d)"),newspec.GetDOF()%maxvelocities.size()%maxaccelerations.size(), ORE_InvalidArguments);
    }
    // don't need to convert since the planner does that automatically
    //ConvertTrajectorySpecification(traj,newspec);
    PlannerBasePtr planner = RaveCreatePlanner(traj->GetEnv(),plannername.size() > 0 ? plannername : string("parabolicsmoother"));
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_vConfigVelocityLimit = maxvelocities;
    params->_vConfigAccelerationLimit = maxaccelerations;
    params->_configurationspecification = newspec;
    params->_vConfigLowerLimit.resize(newspec.GetDOF());
    params->_vConfigUpperLimit.resize(newspec.GetDOF());
    params->_vConfigResolution.resize(newspec.GetDOF());
    for(size_t i = 0; i < params->_vConfigLowerLimit.size(); ++i) {
        params->_vConfigLowerLimit[i] = -1e6;
        params->_vConfigUpperLimit[i] = 1e6;
        params->_vConfigResolution[i] = 0.01;
    }

    std::list< boost::function<void(std::vector<dReal>::const_iterator) > > listsetfunctions;
    std::list< boost::function<void(std::vector<dReal>::iterator) > > listgetfunctions;
    std::list< boost::function<dReal(std::vector<dReal>::const_iterator, std::vector<dReal>::const_iterator) > > listdistfunctions;
    std::vector<int> vrotaxes;
    // analyze the configuration for identified dimensions
    KinBodyPtr robot;
    FOREACHC(itgroup,newspec._vgroups) {
        if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            string tempname;
            int affinedofs=0;
            stringstream ss(itgroup->name.substr(16));
            ss >> tempname >> affinedofs;
            BOOST_ASSERT( !!ss );
            KinBodyPtr pbody = traj->GetEnv()->GetKinBody(tempname);
            // body doesn't hav eto be set
            if( !!pbody ) {
                Vector vaxis(0,0,1);
                if( affinedofs & DOF_RotationAxis ) {
                    vrotaxes.push_back(itgroup->offset+RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationAxis));
                    ss >> vaxis.x >> vaxis.y >> vaxis.z;
                }
                robot = pbody;
                listsetfunctions.push_back(boost::bind(_SetTransformBody,_1,pbody,itgroup->offset,affinedofs,vaxis));
                listgetfunctions.push_back(boost::bind(_GetTransformBody,_1,pbody,itgroup->offset,affinedofs,vaxis));
                listdistfunctions.push_back(boost::bind(_ComputeTransformBodyDistance, _1, _2, pbody, itgroup->offset,affinedofs,vaxis));
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
            if( bsmooth ) {
                RAVELOG_VERBOSE("cannot smooth state for IK configurations\n");
            }
        }
    }

    boost::shared_ptr<PlannerStateSaver> statesaver;
    if( bsmooth ) {
        if( listsetfunctions.size() > 0 ) {
            params->_setstatevaluesfn = boost::bind(_SetAffineState,boost::ref(listsetfunctions), _1, _2);
            params->_getstatefn = boost::bind(_GetAffineState,_1,params->GetDOF(), boost::ref(listgetfunctions));
            params->_distmetricfn = boost::bind(_ComputeAffineDistanceMetric,_1,_2,boost::ref(listdistfunctions));
            std::list<KinBodyPtr> listCheckCollisions; listCheckCollisions.push_back(robot);
            boost::shared_ptr<DynamicsCollisionConstraint> pcollision(new DynamicsCollisionConstraint(params, listCheckCollisions, 0xffffffff&~CFO_CheckTimeBasedConstraints));
            params->_checkpathvelocityconstraintsfn = boost::bind(&DynamicsCollisionConstraint::Check,pcollision,_1, _2, _3, _4, _5, _6, _7, _8);
            statesaver.reset(new PlannerStateSaver(newspec.GetDOF(), params->_setstatevaluesfn, params->_getstatefn));
        }
    }
    else {
        params->_setstatevaluesfn.clear();
        params->_setstatefn.clear();
        params->_getstatefn.clear();
        params->_checkpathconstraintsfn.clear();
        params->_checkpathvelocityconstraintsfn.clear();
    }

    params->_diffstatefn = boost::bind(diffstatefn,_1,_2,vrotaxes);

    params->_hastimestamps = hastimestamps;
    params->_multidofinterp = 2; // always force switch points to be the same
    params->_sExtraParameters = plannerparameters;
    if( !planner->InitPlan(RobotBasePtr(),params) ) {
        return PS_Failed;
    }
    if( planner->PlanPath(traj) != PS_HasSolution ) {
        return PS_Failed;
    }
    return PS_HasSolution;
}

AffineTrajectoryRetimer::AffineTrajectoryRetimer(const std::string& plannername, const std::string& plannerparameters)
{
    if( plannername.size() > 0 ) {
        _plannername = plannername;
    }
    else {
        _plannername = "parabolictrajectoryretimer";
    }
    _extraparameters = plannerparameters;
}

void AffineTrajectoryRetimer::SetPlanner(const std::string& plannername, const std::string& plannerparameters)
{
    if( plannername.size() == 0 ) {
        if( _plannername != string("parabolictrajectoryretimer") ) {
            _plannername = "parabolictrajectoryretimer";
            _planner.reset();
        }
    }
    else if( _plannername != plannername ) {
        _plannername = plannername;
        _planner.reset();
    }

    if( _extraparameters != plannerparameters ) {
        _extraparameters = plannerparameters;
        if( !!_parameters ) {
            _parameters->_sExtraParameters = _extraparameters;
            if( !!_planner ) {
                if( !_planner->InitPlan(RobotBasePtr(), _parameters) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s"), _plannername, ORE_InvalidArguments);
                }
            }
        }
    }
}

PlannerStatus AffineTrajectoryRetimer::PlanPath(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, bool hastimestamps)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need retiming, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PS_HasSolution;
    }

    EnvironmentBasePtr env = traj->GetEnv();
    EnvironmentMutex::scoped_lock lockenv(env->GetMutex());
    ConfigurationSpecification trajspec = traj->GetConfigurationSpecification().GetTimeDerivativeSpecification(0);
    if( trajspec.GetDOF() != (int)maxvelocities.size() || trajspec.GetDOF() != (int)maxaccelerations.size() ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("traj values (%d) do not match maxvelocity size (%d) or maxaccelerations size (%d)"),trajspec.GetDOF()%maxvelocities.size()%maxaccelerations.size(), ORE_InvalidArguments);
    }
    //ConvertTrajectorySpecification(traj,trajspec);
    TrajectoryTimingParametersPtr parameters;
    if( !_parameters ) {
        parameters.reset(new TrajectoryTimingParameters());
        parameters->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
        parameters->_setstatevaluesfn.clear();
        parameters->_setstatefn.clear();
        parameters->_getstatefn.clear();
        parameters->_checkpathconstraintsfn.clear();
        parameters->_checkpathvelocityconstraintsfn.clear();
        parameters->_sExtraParameters += _extraparameters;
        parameters->_multidofinterp = 2; // always force switch points to be the same
        _parameters = parameters;
    }
    else {
        parameters = boost::dynamic_pointer_cast<TrajectoryTimingParameters>(_parameters);
    }


    bool bInitPlan=false;
    if( bInitPlan || CompareRealVectors(parameters->_vConfigVelocityLimit, maxvelocities, g_fEpsilonLinear) != 0 ) {
        parameters->_vConfigVelocityLimit = maxvelocities;
        bInitPlan = true;
    }
    if( bInitPlan || CompareRealVectors(parameters->_vConfigAccelerationLimit, maxaccelerations, g_fEpsilonLinear) != 0 ) {
        parameters->_vConfigAccelerationLimit = maxaccelerations;
        bInitPlan = true;
    }
    if( bInitPlan || parameters->_configurationspecification != trajspec ) {
        parameters->_configurationspecification = trajspec;
        bInitPlan = true;
    }
    if( bInitPlan || (int)parameters->_vConfigLowerLimit.size() != trajspec.GetDOF() ) {
        parameters->_vConfigLowerLimit.resize(trajspec.GetDOF());
        parameters->_vConfigUpperLimit.resize(trajspec.GetDOF());
        parameters->_vConfigResolution.resize(trajspec.GetDOF());
        for(size_t i = 0; i < parameters->_vConfigLowerLimit.size(); ++i) {
            parameters->_vConfigLowerLimit[i] = -1e6;
            parameters->_vConfigUpperLimit[i] = 1e6;
            parameters->_vConfigResolution[i] = 0.01;
        }
        bInitPlan = true;
    }

    std::vector<int> vrotaxes;
    // analyze the configuration for identified dimensions
    FOREACHC(itgroup,trajspec._vgroups) {
        if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            string tempname;
            int affinedofs=0;
            stringstream ss(itgroup->name.substr(16));
            ss >> tempname >> affinedofs;
            BOOST_ASSERT( !!ss );
            KinBodyPtr pbody = env->GetKinBody(tempname);
            // body doesn't hav eto be set
            if( !!pbody ) {
                Vector vaxis(0,0,1);
                if( affinedofs & DOF_RotationAxis ) {
                    vrotaxes.push_back(itgroup->offset+RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationAxis));
                    ss >> vaxis.x >> vaxis.y >> vaxis.z;
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

    if( bInitPlan ) {
        parameters->_diffstatefn = boost::bind(diffstatefn,_1,_2,vrotaxes);
    }
    if( parameters->_hastimestamps != hastimestamps ) {
        parameters->_hastimestamps = hastimestamps;
        bInitPlan = true;
    }

    if( !_planner ) {
        _planner = RaveCreatePlanner(env,_plannername);
        bInitPlan = true;
    }
    if( bInitPlan ) {
        if( !_planner->InitPlan(RobotBasePtr(),parameters) ) {
            stringstream ss; ss << trajspec;
            throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with affine trajectory spec: %s"), _plannername%ss.str(), ORE_InvalidArguments);
        }
    }

    return _planner->PlanPath(traj);
}

PlannerStatus SmoothActiveDOFTrajectory(TrajectoryBasePtr traj, RobotBasePtr robot, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, const std::string& plannerparameters)
{
    return _PlanActiveDOFTrajectory(traj,robot,false,fmaxvelmult,fmaxaccelmult,plannername.size() > 0 ? plannername : "parabolicsmoother", true,plannerparameters);
}

PlannerStatus SmoothAffineTrajectory(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, const std::string& plannername, const std::string& plannerparameters)
{
    return _PlanAffineTrajectory(traj, maxvelocities, maxaccelerations, false, plannername.size() > 0 ? plannername : "parabolicsmoother", true, plannerparameters);
}

PlannerStatus SmoothTrajectory(TrajectoryBasePtr traj, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, const std::string& plannerparameters)
{
    return _PlanTrajectory(traj,false,fmaxvelmult,fmaxaccelmult,plannername.size() > 0 ? plannername : "parabolicsmoother", true,plannerparameters);
}

static std::string GetPlannerFromInterpolation(TrajectoryBasePtr traj, const std::string& forceplanner=std::string())
{
    if( forceplanner.size() > 0 ) {
        return forceplanner;
    }
    std::string interpolation;
    // check out the trajectory interpolation values and take it from there
    FOREACHC(itgroup,traj->GetConfigurationSpecification()._vgroups) {
        if( itgroup->name.size() >= 12 && itgroup->name.substr(0,12) == "joint_values" ) {
            interpolation = itgroup->interpolation;
            break;
        }
        else if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            interpolation = itgroup->interpolation;
        }
        else if( itgroup->name.size() >= 14 && itgroup->name.substr(0,14) == "ikparam_values" ) {
            interpolation = itgroup->interpolation;
        }
    }

    if( interpolation == "quadratic" ) {
        return "parabolictrajectoryretimer";
    }
    else if( interpolation == "linear" ) {
        return "lineartrajectoryretimer";
    }
    else {
        return "lineartrajectoryretimer";
    }
}

PlannerStatus RetimeActiveDOFTrajectory(TrajectoryBasePtr traj, RobotBasePtr robot, bool hastimestamps, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, const std::string& plannerparameters)
{
    return _PlanActiveDOFTrajectory(traj,robot,hastimestamps,fmaxvelmult,fmaxaccelmult,GetPlannerFromInterpolation(traj,plannername), false,plannerparameters);
}

PlannerStatus RetimeAffineTrajectory(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, bool hastimestamps, const std::string& plannername, const std::string& plannerparameters)
{
    return _PlanAffineTrajectory(traj, maxvelocities, maxaccelerations, hastimestamps, GetPlannerFromInterpolation(traj,plannername), false,plannerparameters);
}

PlannerStatus RetimeTrajectory(TrajectoryBasePtr traj, bool hastimestamps, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, const std::string& plannerparameters)
{
    return _PlanTrajectory(traj,hastimestamps,fmaxvelmult,fmaxaccelmult,GetPlannerFromInterpolation(traj,plannername), false,plannerparameters);
}

size_t ExtendActiveDOFWaypoint(int waypointindex, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, RobotBasePtr robot, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername)
{
    if( traj->GetNumWaypoints()<1) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("trajectory is void"),ORE_InvalidArguments);
    }
    ConfigurationSpecification spec = robot->GetActiveConfigurationSpecification();
    std::vector<dReal> jitteredvalues;
    if( waypointindex == 0 ) {
        traj->GetWaypoint(waypointindex,jitteredvalues, spec);
        robot->SubtractActiveDOFValues(jitteredvalues, dofvalues);
        dReal diff = 0;
        for(size_t i = 0; i < dofvalues.size(); ++i) {
            diff += jitteredvalues[i]*jitteredvalues[i];
        }
        if( diff <= g_fEpsilon ) {
            // not that much has changed
            return waypointindex;
        }
        RAVELOG_VERBOSE_FORMAT("Jitter distance^2 (init) = %f", diff);
        traj->Remove(waypointindex,waypointindex+1);
    }
    else if( waypointindex == (int)traj->GetNumWaypoints() ) {
        traj->GetWaypoint(waypointindex-1,jitteredvalues, spec);
        robot->SubtractActiveDOFValues(jitteredvalues, dofvalues);
        dReal diff = 0;
        for(size_t i = 0; i < dofvalues.size(); ++i) {
            diff += jitteredvalues[i]*jitteredvalues[i];
        }
        if( diff <= g_fEpsilon ) {
            // not that much has changed
            return waypointindex;
        }
        RAVELOG_VERBOSE_FORMAT("Jitter distance^2 (goal) = %f", diff);
        traj->Remove(waypointindex-1,waypointindex);
        waypointindex -= 1; // have to reduce by one for InsertActiveDOFWaypointWithRetiming
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot extend waypoints in middle of trajectories"),ORE_InvalidArguments);
    }
    return InsertActiveDOFWaypointWithRetiming(waypointindex,dofvalues,dofvelocities,traj,robot,fmaxvelmult,fmaxaccelmult,plannername);
}

size_t InsertActiveDOFWaypointWithRetiming(int waypointindex, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, RobotBasePtr robot, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername)
{
    BOOST_ASSERT((int)dofvalues.size()==robot->GetActiveDOF());
    BOOST_ASSERT(traj->GetEnv()==robot->GetEnv());
    vector<dReal> v1pos(robot->GetActiveDOF(),0), v1vel(robot->GetActiveDOF(),0);
    ConfigurationSpecification newspec = robot->GetActiveConfigurationSpecification();

    string interpolation = "";
    FOREACH(it,newspec._vgroups) {
        std::vector<ConfigurationSpecification::Group>::const_iterator itgroup = traj->GetConfigurationSpecification().FindCompatibleGroup(*it, false);
        if( itgroup == traj->GetConfigurationSpecification()._vgroups.end() ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("could not find group %s in trajectory"),newspec._vgroups.at(0).name,ORE_InvalidArguments);
        }
        if( itgroup->interpolation.size() > 0 ) {
            it->interpolation = itgroup->interpolation;
            interpolation = itgroup->interpolation;
        }
    }
    newspec.AddDerivativeGroups(1,false);

    vector<dReal> vwaypointstart, vwaypointend, vtargetvalues;
    if( waypointindex == 0 ) {
        vwaypointstart.resize(newspec.GetDOF());
        ConfigurationSpecification::ConvertData(vwaypointstart.begin(), newspec, dofvalues.begin(), robot->GetActiveConfigurationSpecification(), 1, traj->GetEnv(), true);
        if( dofvalues.size() == dofvelocities.size() ) {
            ConfigurationSpecification::ConvertData(vwaypointstart.begin(), newspec, dofvelocities.begin(), robot->GetActiveConfigurationSpecification().ConvertToVelocitySpecification(), 1, traj->GetEnv(), false);
        }
        traj->GetWaypoint(0,vwaypointend, newspec);
        traj->GetWaypoint(0,vtargetvalues); // in target spec
    }
    else if( waypointindex == (int)traj->GetNumWaypoints() ) {
        traj->GetWaypoint(waypointindex-1,vwaypointstart, newspec);
        traj->GetWaypoint(waypointindex-1,vtargetvalues); // in target spec

        vwaypointend.resize(newspec.GetDOF());
        ConfigurationSpecification::ConvertData(vwaypointend.begin(), newspec, dofvalues.begin(), robot->GetActiveConfigurationSpecification(), 1, traj->GetEnv(), true);
        if( dofvalues.size() == dofvelocities.size() ) {
            ConfigurationSpecification::ConvertData(vwaypointend.begin(), newspec, dofvelocities.begin(), robot->GetActiveConfigurationSpecification().ConvertToVelocitySpecification(), 1, traj->GetEnv(), false);
        }
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT(_("do no support inserting waypoint at %d in middle of trajectory (size=%d)"), waypointindex%traj->GetNumWaypoints(), ORE_InvalidArguments);
    }

    TrajectoryBasePtr trajinitial = RaveCreateTrajectory(traj->GetEnv(),traj->GetXMLId());
    trajinitial->Init(newspec);
    trajinitial->Insert(0,vwaypointstart);
    trajinitial->Insert(1,vwaypointend);

    std::string newplannername = plannername;

    if( newplannername.size() == 0 ) {
        if( interpolation == "linear" ) {
            newplannername = "lineartrajectoryretimer";
        }
        else if( interpolation.size() == 0 || interpolation == "quadratic" ) {
            newplannername = "parabolictrajectoryretimer";
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_("currently do not support retiming for %s interpolations"),interpolation,ORE_InvalidArguments);
        }
    }

    if( IS_DEBUGLEVEL(Level_Verbose) ) {
        int ran = RaveRandomInt()%10000;
        string filename = str(boost::format("/var/www/.openrave/beforeretime-%d.xml")%ran);
        RAVELOG_VERBOSE_FORMAT("Writing before retime traj to %s", filename);
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        trajinitial->serialize(f);
    }

    // This ensures that the beginning and final velocities will be preserved
    RetimeActiveDOFTrajectory(trajinitial,robot,false,fmaxvelmult,fmaxaccelmult,newplannername,"<hasvelocities>1</hasvelocities>");

    // retiming is done, now merge the two trajectories
    size_t nInitialNumWaypoints = trajinitial->GetNumWaypoints();
    size_t targetdof = vtargetvalues.size();
    vtargetvalues.resize(targetdof*nInitialNumWaypoints);
    for(size_t i = targetdof; i < vtargetvalues.size(); i += targetdof) {
        std::copy(vtargetvalues.begin(),vtargetvalues.begin()+targetdof,vtargetvalues.begin()+i);
    }
    trajinitial->GetWaypoints(0, nInitialNumWaypoints, vwaypointstart);

    // copy to the target values while preserving other data
    ConfigurationSpecification::ConvertData(vtargetvalues.begin(), traj->GetConfigurationSpecification(), vwaypointstart.begin(), trajinitial->GetConfigurationSpecification(), nInitialNumWaypoints, traj->GetEnv(), false);

    if( waypointindex == 0 ) {
        // have to insert the first N-1 and overwrite the Nth
        vwaypointstart.resize(targetdof);
        std::copy(vtargetvalues.begin()+vtargetvalues.size()-targetdof,vtargetvalues.end(),vwaypointstart.begin());
        traj->Insert(waypointindex,vwaypointstart,true);
        vtargetvalues.resize(vtargetvalues.size()-targetdof);
        if( vtargetvalues.size() > 0 ) {
            traj->Insert(waypointindex,vtargetvalues,false);
        }
    }
    else {
        // insert last N-1
        vtargetvalues.erase(vtargetvalues.begin(), vtargetvalues.begin()+targetdof);
        traj->Insert(waypointindex,vtargetvalues,false);
    }
    return waypointindex+nInitialNumWaypoints-1;
}

size_t ExtendWaypoint(int waypointindex, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, PlannerBasePtr planner){
    if( traj->GetNumWaypoints()<1) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("trajectory is void"),ORE_InvalidArguments);
    }
    if( waypointindex == 0 ) {
        //Remove the first waypoint
        traj->Remove(waypointindex,waypointindex+1);
    }
    else if( waypointindex == (int)traj->GetNumWaypoints() ) {
        //Remove the last waypoint
        traj->Remove(waypointindex-1,waypointindex);
        //Decrese waypointindex by 1
        waypointindex--;
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot extend waypoints in middle of trajectories"),ORE_InvalidArguments);
    }
    // Run Insertwaypoint
    return InsertWaypointWithRetiming(waypointindex,dofvalues,dofvelocities,traj,planner);
}

size_t InsertWaypointWithRetiming(int waypointindex, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, PlannerBasePtr planner)
{
    PlannerBase::PlannerParametersConstPtr parameters = planner->GetParameters();
    BOOST_ASSERT((int)dofvalues.size()==parameters->GetDOF());
    vector<dReal> v1pos(parameters->GetDOF(),0), v1vel(parameters->GetDOF(),0);
    ConfigurationSpecification newspec = parameters->_configurationspecification;
    newspec.AddDerivativeGroups(1,false);

    vector<dReal> vwaypointstart, vwaypointend, vtargetvalues;
    if( waypointindex == 0 ) {
        vwaypointstart.resize(newspec.GetDOF());
        ConfigurationSpecification::ConvertData(vwaypointstart.begin(), newspec, dofvalues.begin(), parameters->_configurationspecification, 1, traj->GetEnv(), true);
        if( dofvalues.size() == dofvelocities.size() ) {
            ConfigurationSpecification::ConvertData(vwaypointstart.begin(), newspec, dofvelocities.begin(), parameters->_configurationspecification.ConvertToVelocitySpecification(), 1, traj->GetEnv(), false);
        }
        traj->GetWaypoint(0,vwaypointend, newspec);
        traj->GetWaypoint(0,vtargetvalues); // in target spec
    }
    else if( waypointindex == (int)traj->GetNumWaypoints() ) {
        traj->GetWaypoint(waypointindex-1,vwaypointstart, newspec);
        traj->GetWaypoint(waypointindex-1,vtargetvalues); // in target spec

        vwaypointend.resize(newspec.GetDOF());
        ConfigurationSpecification::ConvertData(vwaypointend.begin(), newspec, dofvalues.begin(), parameters->_configurationspecification, 1, traj->GetEnv(), true);
        if( dofvalues.size() == dofvelocities.size() ) {
            ConfigurationSpecification::ConvertData(vwaypointend.begin(), newspec, dofvelocities.begin(), parameters->_configurationspecification.ConvertToVelocitySpecification(), 1, traj->GetEnv(), false);
        }
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("do no support inserting waypoints in middle of trajectories yet"),ORE_InvalidArguments);
    }

    TrajectoryBasePtr trajinitial = RaveCreateTrajectory(traj->GetEnv(),traj->GetXMLId());
    trajinitial->Init(newspec);
    trajinitial->Insert(0,vwaypointstart);
    trajinitial->Insert(1,vwaypointend);
    if( !(planner->PlanPath(trajinitial) & PS_HasSolution) ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("failed to plan path"), ORE_Assert);
    }

    // retiming is done, now merge the two trajectories
    size_t nInitialNumWaypoints = trajinitial->GetNumWaypoints();
    size_t targetdof = vtargetvalues.size();
    vtargetvalues.resize(targetdof*nInitialNumWaypoints);
    for(size_t i = targetdof; i < vtargetvalues.size(); i += targetdof) {
        std::copy(vtargetvalues.begin(),vtargetvalues.begin()+targetdof,vtargetvalues.begin()+i);
    }
    trajinitial->GetWaypoints(0, nInitialNumWaypoints, vwaypointstart);

    // copy to the target values while preserving other data
    ConfigurationSpecification::ConvertData(vtargetvalues.begin(), traj->GetConfigurationSpecification(), vwaypointstart.begin(), trajinitial->GetConfigurationSpecification(), nInitialNumWaypoints, traj->GetEnv(), false);

    if( waypointindex == 0 ) {
        // have to insert the first N-1 and overwrite the Nth
        vwaypointstart.resize(targetdof);
        std::copy(vtargetvalues.begin()+vtargetvalues.size()-targetdof,vtargetvalues.end(),vwaypointstart.begin());
        traj->Insert(waypointindex,vwaypointstart,true);
        vtargetvalues.resize(vtargetvalues.size()-targetdof);
        if( vtargetvalues.size() > 0 ) {
            traj->Insert(waypointindex,vtargetvalues,false);
        }
    }
    else {
        // insert last N-1
        vtargetvalues.erase(vtargetvalues.begin(), vtargetvalues.begin()+targetdof);
        traj->Insert(waypointindex,vtargetvalues,false);
    }
    return waypointindex+nInitialNumWaypoints-1;
}

size_t InsertWaypointWithSmoothing(int index, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername)
{
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    ConfigurationSpecification specpos = traj->GetConfigurationSpecification().GetTimeDerivativeSpecification(0);
    OPENRAVE_ASSERT_OP(specpos.GetDOF(),==,(int)dofvalues.size());
    params->SetConfigurationSpecification(traj->GetEnv(),specpos);
    FOREACH(it,params->_vConfigVelocityLimit) {
        *it *= fmaxvelmult;
    }
    FOREACH(it,params->_vConfigAccelerationLimit) {
        *it *= fmaxaccelmult;
    }

    params->_hasvelocities = true;
    params->_hastimestamps = false;

    PlannerBasePtr planner = RaveCreatePlanner(traj->GetEnv(),plannername.size() > 0 ? plannername : string("parabolictrajectoryretimer"));
    if( !planner->InitPlan(RobotBasePtr(),params) ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("failed to InitPlan"),ORE_Failed);
    }

    return InsertWaypointWithSmoothing(index, dofvalues, dofvelocities, traj, planner);
}

size_t InsertWaypointWithSmoothing(int index, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, PlannerBasePtr planner)
{
    if( index != (int)traj->GetNumWaypoints() ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("InsertWaypointWithSmoothing only supports adding waypoints at the end"),ORE_InvalidArguments);
    }
    OPENRAVE_ASSERT_OP(dofvalues.size(),==,dofvelocities.size());

    PlannerBase::PlannerParametersConstPtr params = planner->GetParameters();
    ConfigurationSpecification specpos = traj->GetConfigurationSpecification().GetTimeDerivativeSpecification(0);

    EnvironmentMutex::scoped_lock lockenv(traj->GetEnv()->GetMutex());

    dReal fSamplingTime = 0.01; // for collision checking
    dReal fTimeBuffer = 0.01; // if new trajectory increases within this time limit, then it will be accepted

    ConfigurationSpecification spectotal = specpos;
    spectotal.AddDerivativeGroups(1,false);
    OPENRAVE_ASSERT_OP(spectotal.GetDOF(),==,2*(int)dofvalues.size());
    TrajectoryBasePtr ptesttraj;
    TrajectoryBasePtr pBestTrajectory;
    dReal fBestDuration = 1e30;
    int iBestInsertionWaypoint = -1;
    std::vector<dReal> vstartdata(spectotal.GetDOF(),0), vwaypoint, vprevpoint;
    std::copy(dofvalues.begin(),dofvalues.end(),vstartdata.begin());
    std::copy(dofvelocities.begin(),dofvelocities.end(),vstartdata.begin()+dofvalues.size());
    // look for the waypoints in reverse
    int N = (int)traj->GetNumWaypoints();
    dReal fOriginalTime = traj->GetDuration();
    dReal fRemainingDuration=traj->GetDuration();
    int iTimeIndex = -1;
    std::vector<ConfigurationSpecification::Group>::const_iterator itdeltatimegroup = traj->GetConfigurationSpecification().FindCompatibleGroup("deltatime");
    if( itdeltatimegroup != traj->GetConfigurationSpecification()._vgroups.end() ) {
        iTimeIndex = itdeltatimegroup->offset;
    }

    // since inserting from the back, go through each of the waypoints from reverse and record collision free segments
    // perhaps a faster method would be to delay the collision checking until all the possible segments are already checked...?
    for(int iwaypoint = 0; iwaypoint < N; ++iwaypoint) {
        traj->GetWaypoint(N-1-iwaypoint,vwaypoint);
        dReal deltatime = 0;
        if( iTimeIndex >= 0 ) {
            // reset the time in case the trajectory retimer does not reset it
            deltatime = vwaypoint.at(iTimeIndex);
            vwaypoint.at(iTimeIndex) = 0;
        }
        if( !ptesttraj ) {
            ptesttraj = RaveCreateTrajectory(traj->GetEnv(),traj->GetXMLId());
        }
        ptesttraj->Init(traj->GetConfigurationSpecification());
        ptesttraj->Insert(0,vwaypoint);
        ptesttraj->Insert(1,vstartdata,spectotal);

        if( planner->PlanPath(ptesttraj) & PS_HasSolution ) {
            // before checking, make sure it is better than we currently have
            dReal fNewDuration = fRemainingDuration+ptesttraj->GetDuration();
            if( fNewDuration < fBestDuration ) {
                // have to check for collision by sampling
                ptesttraj->Sample(vprevpoint,0,specpos);
                bool bInCollision = false;
                for(dReal t = fSamplingTime; t < ptesttraj->GetDuration(); t+=fSamplingTime) {
                    ptesttraj->Sample(vwaypoint,t,specpos);
                    if( params->CheckPathAllConstraints(vprevpoint,vwaypoint,std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart) != 0 ) {
                        bInCollision = true;
                        break;
                    }
                    vprevpoint = vwaypoint;
                }
                if( !bInCollision ) {
                    iBestInsertionWaypoint = N-1-iwaypoint;
                    fBestDuration = fNewDuration;
                    swap(pBestTrajectory,ptesttraj);
                    if( iwaypoint > 0 && fBestDuration < fOriginalTime+fTimeBuffer ) {
                        break;
                    }
                }
                else {
                    if( !!pBestTrajectory ) {
                        // already have a good trajectory, and chances are that anything after this waypoint will also be in collision, so exit
                        break;
                    }
                }
            }
            else {
                // if it isn't better and we already have a best trajectory, then choose it. most likely things will get worse from now on...
                if( !!pBestTrajectory ) {
                    break;
                }
            }
        }
        fRemainingDuration -= deltatime;
    }

    if( !pBestTrajectory ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("failed to find connecting trajectory"),ORE_Assert);
    }
    if( fBestDuration > fOriginalTime+fTimeBuffer ) {
        RAVELOG_WARN(str(boost::format("new trajectory is greater than expected time %f > %f \n")%fBestDuration%fOriginalTime));
    }
    // splice in the new trajectory. pBestTrajectory's first waypoint matches traj's iBestInsertionWaypoint
    traj->Remove(iBestInsertionWaypoint+1,traj->GetNumWaypoints());
    //    traj->GetWaypoint(iBestInsertionWaypoint,vprevpoint);
    //    pBestTrajectory->GetWaypoint(0,vwaypoint,traj->GetConfigurationSpecification());
    //    for(size_t i = 0; i < vprevpoint.size(); ++i) {
    //        if( RaveFabs(vprevpoint[i]-vwaypoint.at(i)) > 0.0001 ) {
    //            RAVELOG_WARN(str(boost::format("start points differ at %d: %e != %e")%i%vprevpoint[i]%vwaypoint[i]));
    //        }
    //    }
    pBestTrajectory->GetWaypoints(1,pBestTrajectory->GetNumWaypoints(),vwaypoint,traj->GetConfigurationSpecification());
    traj->Insert(iBestInsertionWaypoint+1,vwaypoint);
    dReal fNewDuration = traj->GetDuration();
    OPENRAVE_ASSERT_OP( RaveFabs(fNewDuration-fBestDuration), <=, 0.001 );
    return iBestInsertionWaypoint+pBestTrajectory->GetNumWaypoints();
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

void ComputeTrajectoryDerivatives(TrajectoryBasePtr traj, int maxderiv)
{
    OPENRAVE_ASSERT_OP(maxderiv,>,0);
    ConfigurationSpecification newspec = traj->GetConfigurationSpecification();
    for(int i = 1; i <= maxderiv; ++i) {
        newspec.AddDerivativeGroups(i);
    }
    std::vector<dReal> data;
    traj->GetWaypoints(0,traj->GetNumWaypoints(),data, newspec);
    if( data.size() == 0 ) {
        traj->Init(newspec);
        return;
    }
    int numpoints = (int)data.size()/newspec.GetDOF();
    ConfigurationSpecification velspec = newspec.GetTimeDerivativeSpecification(maxderiv-1);
    ConfigurationSpecification accelspec = newspec.GetTimeDerivativeSpecification(maxderiv);
    std::vector<ConfigurationSpecification::Group>::const_iterator itdeltatimegroup = newspec.FindCompatibleGroup("deltatime");
    if(itdeltatimegroup == newspec._vgroups.end() ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("trajectory does not seem to have time stamps, so derivatives cannot be computed"), ORE_InvalidArguments);
    }
    OPENRAVE_ASSERT_OP(velspec.GetDOF(), ==, accelspec.GetDOF());
    int offset = 0;
    dReal nextdeltatime=0;
    for(int ipoint = 0; ipoint < numpoints; ++ipoint, offset += newspec.GetDOF()) {
        if( ipoint < numpoints-1 ) {
            nextdeltatime = data.at(offset+newspec.GetDOF()+itdeltatimegroup->offset);
        }
        else {
            nextdeltatime = 0;
        }
        for(size_t igroup = 0; igroup < velspec._vgroups.size(); ++igroup) {
            std::vector<ConfigurationSpecification::Group>::const_iterator itvel = newspec.FindCompatibleGroup(velspec._vgroups.at(igroup),true);
            std::vector<ConfigurationSpecification::Group>::const_iterator itaccel = newspec.FindCompatibleGroup(accelspec._vgroups.at(igroup),true);
            OPENRAVE_ASSERT_OP(itvel->dof,==,itaccel->dof);
            if( ipoint < numpoints-1 ) {
                for(int i = 1; i < itvel->dof; ++i) {
                    if( nextdeltatime > 0 ) {
                        data.at(offset+itaccel->offset+i) = (data.at(offset+newspec.GetDOF()+itvel->offset+i)-data.at(offset+itvel->offset+i))/nextdeltatime;
                    }
                    else {
                        // time is 0 so leave empty?
                    }
                }
            }
            else {
                // what to do with the last point?
            }
        }
    }

    traj->Init(newspec);
    traj->Insert(0,data);
}

TrajectoryBasePtr GetReverseTrajectory(TrajectoryBaseConstPtr sourcetraj)
{
    vector<dReal> sourcedata;
    size_t numpoints = sourcetraj->GetNumWaypoints();
    int dof = sourcetraj->GetConfigurationSpecification().GetDOF();
    vector<uint8_t> velocitydofs(dof,0);
    int timeoffset = -1;
    vector<uint8_t> velocitynextinterp(dof,0);
    FOREACHC(itgroup, sourcetraj->GetConfigurationSpecification()._vgroups) {
        if( itgroup->name.find("_velocities") != string::npos ) {
            bool bnext = itgroup->interpolation == "next";
            for(int i = 0; i < itgroup->dof; ++i) {
                velocitydofs.at(itgroup->offset+i) = 1;
                velocitynextinterp.at(itgroup->offset+i) = bnext;
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
                if( velocitynextinterp[j] ) {
                    if( i < numpoints-1 ) {
                        *(ittarget+j+dof) = -*(itsource+j);
                    }
                    else {
                        targetdata.at(j) = 0;
                    }
                }
                else {
                    *(ittarget+j) = -*(itsource+j);
                }
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
    traj->SetDescription(sourcetraj->GetDescription());
    return traj;
}

TrajectoryBasePtr ReverseTrajectory(TrajectoryBasePtr sourcetraj)
{
    // might need to change to in-memory reverse...
    return GetReverseTrajectory(sourcetraj);
}

void SegmentTrajectory(TrajectoryBasePtr traj, dReal starttime, dReal endtime)
{
    std::vector<dReal> values;
    if( endtime < traj->GetDuration() ) {
        size_t endindex = traj->GetFirstWaypointIndexAfterTime(endtime);
        if( endindex < traj->GetNumWaypoints() ) {
            traj->Sample(values, endtime);
            traj->Insert(endindex, values, true);
            traj->Remove(endindex+1, traj->GetNumWaypoints());
        }
    }
    if( starttime > 0 ) {
        size_t startindex = traj->GetFirstWaypointIndexAfterTime(starttime);
        if( startindex > 0 ) {
            ConfigurationSpecification deltatimespec;
            deltatimespec.AddDeltaTimeGroup();
            std::vector<dReal> vdeltatime;
            traj->GetWaypoint(startindex,vdeltatime,deltatimespec);
            traj->Sample(values, starttime);
            dReal fSampleDeltaTime;
            traj->GetConfigurationSpecification().ExtractDeltaTime(fSampleDeltaTime, values.begin());
            // check if the sampletime can be very close to an existing waypoint, in which case can ignore inserting a new point
            int endremoveindex = startindex;
            if( RaveFabs(fSampleDeltaTime-vdeltatime.at(0)) > g_fEpsilonLinear ) {
                traj->Insert(startindex-1, values, true);
                // have to write the new delta time
                vdeltatime[0] -= fSampleDeltaTime;
                traj->Insert(startindex, vdeltatime, deltatimespec, true);
                endremoveindex -= 1;
            }
            traj->Remove(0, endremoveindex);
        }
    }
}

TrajectoryBasePtr MergeTrajectories(const std::list<TrajectoryBaseConstPtr>& listtrajectories)
{
    // merge both deltatime and iswaypoint groups
    TrajectoryBasePtr presulttraj;
    if( listtrajectories.size() == 0 ) {
        return presulttraj;
    }
    if( listtrajectories.size() == 1 ) {
        presulttraj = RaveCreateTrajectory(listtrajectories.front()->GetEnv(),listtrajectories.front()->GetXMLId());
        presulttraj->Clone(listtrajectories.front(),0);
        return presulttraj;
    }

    ConfigurationSpecification spec;
    vector<dReal> vpointdata;
    vector<dReal> vtimes; vtimes.reserve(listtrajectories.front()->GetNumWaypoints());
    int totaldof = 1; // for delta time
    FOREACHC(ittraj,listtrajectories) {
        const ConfigurationSpecification& trajspec = (*ittraj)->GetConfigurationSpecification();
        ConfigurationSpecification::Group gtime = trajspec.GetGroupFromName("deltatime");
        spec += trajspec;
        totaldof += trajspec.GetDOF()-1;
        if( trajspec.FindCompatibleGroup("iswaypoint",true) != trajspec._vgroups.end() ) {
            totaldof -= 1;
        }
        dReal curtime = 0;
        for(size_t ipoint = 0; ipoint < (*ittraj)->GetNumWaypoints(); ++ipoint) {
            (*ittraj)->GetWaypoint(ipoint,vpointdata);
            curtime += vpointdata.at(gtime.offset);
            vector<dReal>::iterator it = lower_bound(vtimes.begin(),vtimes.end(),curtime);
            if( *it != curtime ) {
                vtimes.insert(it,curtime);
            }
        }
    }

    vector<ConfigurationSpecification::Group>::const_iterator itwaypointgroup = spec.FindCompatibleGroup("iswaypoint",true);
    vector<dReal> vwaypoints;
    if( itwaypointgroup != spec._vgroups.end() ) {
        totaldof += 1;
        vwaypoints.resize(vtimes.size(),0);
    }

//    if( totaldof != spec.GetDOF() ) {
//        throw OPENRAVE_EXCEPTION_FORMAT(_("merged configuration needs to have %d DOF, currently has %d"),totaldof%spec.GetDOF(),ORE_InvalidArguments);
//    }
    presulttraj = RaveCreateTrajectory(listtrajectories.front()->GetEnv(),listtrajectories.front()->GetXMLId());
    presulttraj->Init(spec);

    if( vtimes.size() == 0 ) {
        return presulttraj;
    }

    // need to find all waypoints
    vector<dReal> vtemp, vnewdata;
    stringstream sdesc;
    int deltatimeoffset = spec.GetGroupFromName("deltatime").offset;
    FOREACHC(ittraj,listtrajectories) {
        vector<ConfigurationSpecification::Group>::const_iterator itwaypointgrouptraj = (*ittraj)->GetConfigurationSpecification().FindCompatibleGroup("iswaypoint",true);
        int waypointoffset = -1;
        if( itwaypointgrouptraj != (*ittraj)->GetConfigurationSpecification()._vgroups.end() ) {
            waypointoffset = itwaypointgrouptraj->offset;
        }
        if( vnewdata.size() == 0 ) {
            vnewdata.reserve(vtimes.size()*spec.GetDOF());
            for(size_t i = 0; i < vtimes.size(); ++i) {
                (*ittraj)->Sample(vtemp,vtimes[i],spec);
                vnewdata.insert(vnewdata.end(),vtemp.begin(),vtemp.end());
                if( waypointoffset >= 0 ) {
                    vwaypoints[i] += vtemp[itwaypointgroup->offset]; // have to use the final spec's offset
                }
            }
        }
        else {
            vpointdata.resize(0);
            for(size_t i = 0; i < vtimes.size(); ++i) {
                (*ittraj)->Sample(vtemp,vtimes[i]);
                vpointdata.insert(vpointdata.end(),vtemp.begin(),vtemp.end());
                if( waypointoffset >= 0 ) {
                    vwaypoints[i] += vtemp[itwaypointgroup->offset]; // have to use the final spec's offset
                }
            }
            ConfigurationSpecification::ConvertData(vnewdata.begin(),spec,vpointdata.begin(),(*ittraj)->GetConfigurationSpecification(),vtimes.size(),presulttraj->GetEnv(),false);
        }

        sdesc << (*ittraj)->GetDescription() << endl;
    }

    vnewdata.at(deltatimeoffset) = vtimes[0];
    for(size_t i = 1; i < vtimes.size(); ++i) {
        vnewdata.at(i*spec.GetDOF()+deltatimeoffset) = vtimes[i]-vtimes[i-1];
    }
    if( itwaypointgroup != spec._vgroups.end() ) {
        vnewdata.at(itwaypointgroup->offset) = vwaypoints[0];
        for(size_t i = 1; i < vtimes.size(); ++i) {
            vnewdata.at(i*spec.GetDOF()+itwaypointgroup->offset) = vwaypoints[i];
        }
    }
    presulttraj->Insert(0,vnewdata);
    presulttraj->SetDescription(sdesc.str());
    return presulttraj;
}

void GetDHParameters(std::vector<DHParameter>& vparameters, KinBodyConstPtr pbody)
{
    EnvironmentMutex::scoped_lock lockenv(pbody->GetEnv()->GetMutex());
    Transform tbaseinv = pbody->GetTransform().inverse();
    vparameters.resize(pbody->GetDependencyOrderedJoints().size());
    std::vector<DHParameter>::iterator itdh = vparameters.begin();
    FOREACHC(itjoint,pbody->GetDependencyOrderedJoints()) {
        OPENRAVE_ASSERT_FORMAT((*itjoint)->GetType() == KinBody::JointHinge || (*itjoint)->GetType() == KinBody::JointSlider, "joint type 0x%x not supported for DH parameters", (*itjoint)->GetType(), ORE_Assert);
        KinBody::LinkConstPtr plink = (*itjoint)->GetHierarchyParentLink();
        Transform tparent, tparentinv;
        itdh->joint = *itjoint;
        itdh->parentindex = -1;
        if( !!plink ) {
            std::vector<KinBody::JointPtr> vparentjoints;
            pbody->GetChain(0,plink->GetIndex(),vparentjoints);
            if( vparentjoints.size() > 0 ) {
                // get the first non-passive joint
                int index = (int)vparentjoints.size()-1;
                while(index >= 0 && vparentjoints[index]->GetJointIndex() < 0 ) {
                    index--;
                }
                if( index >= 0 ) {
                    // search for it in pbody->GetDependencyOrderedJoints()
                    std::vector<KinBody::JointPtr>::const_iterator itjoint = find(pbody->GetDependencyOrderedJoints().begin(),pbody->GetDependencyOrderedJoints().end(),vparentjoints[index]);
                    BOOST_ASSERT( itjoint != pbody->GetDependencyOrderedJoints().end() );
                    itdh->parentindex = itjoint - pbody->GetDependencyOrderedJoints().begin();
                    tparent = vparameters.at(itdh->parentindex).transform;
                    tparentinv = tparent.inverse();
                }
            }
        }
        Vector vlocalanchor = tparentinv*(tbaseinv*(*itjoint)->GetAnchor());
        Vector vlocalaxis = tparentinv.rotate(tbaseinv.rotate((*itjoint)->GetAxis(0)));
        Vector vlocalx(-vlocalaxis.y,vlocalaxis.x,0);
        dReal fsinalpha = RaveSqrt(vlocalx.lengthsqr3());
        itdh->alpha = RaveAtan2(fsinalpha, vlocalaxis.z);
        if( itdh->alpha < 10*g_fEpsilon ) {
            // axes are parallel
            if( vlocalanchor.lengthsqr2() > 10*g_fEpsilon*g_fEpsilon ) {
                vlocalx.x = vlocalanchor.x;
                vlocalx.y = vlocalanchor.y;
                vlocalx.normalize3();
            }
            else {
                vlocalx = Vector(1,0,0);
            }
        }
        else {
            vlocalx /= fsinalpha;
        }
        itdh->d = vlocalanchor.z;
        itdh->theta = RaveAtan2(vlocalx.y,vlocalx.x);
        itdh->a = vlocalanchor.dot3(vlocalx);
        Vector zquat = quatFromAxisAngle(Vector(0,0,1),itdh->theta);
        Vector xquat = quatFromAxisAngle(Vector(1,0,0),itdh->alpha);
        bool bflip = false;
        //        if( itdh->a < -g_fEpsilon ) {
        //            // cannot have -a since a lot of formats specify a as positive
        //            bflip = true;
        //        }
        //        else {
        Vector worldrotquat = quatMultiply(itdh->transform.rot, quatMultiply(zquat, xquat));
        // always try to have the x-axis pointed positively towards x
        // (worldrotquat * (1,0,0))[0] < 0
        dReal xcomponent = (1-2*worldrotquat.z*worldrotquat.z-2*worldrotquat.w*worldrotquat.w);
        if( xcomponent < 0 ) {
            bflip = true;
        }
        //        }

        if( bflip ) {
            // normalize if theta is closer to PI
            if( itdh->theta > g_fEpsilon ) { // this is a really weird condition but it looks like people prefer theta to be negative
                itdh->alpha = -itdh->alpha;
                itdh->theta -= PI;
                itdh->a = -itdh->a;
            }
            else {
                itdh->alpha = -itdh->alpha;
                itdh->theta += PI;
                itdh->a = -itdh->a;
            }
            zquat = quatFromAxisAngle(Vector(0,0,1),itdh->theta);
            xquat = quatFromAxisAngle(Vector(1,0,0),itdh->alpha);
        }
        Transform Z_i(zquat, Vector(0,0,itdh->d));
        Transform X_i(xquat, Vector(itdh->a,0,0));
        itdh->transform = tparent*Z_i*X_i;
        ++itdh;
    }
}

DynamicsCollisionConstraint::DynamicsCollisionConstraint(PlannerBase::PlannerParametersPtr parameters, const std::list<KinBodyPtr>& listCheckBodies, int filtermask) : _listCheckBodies(listCheckBodies), _filtermask(filtermask), _perturbation(0.1)
{
    BOOST_ASSERT(listCheckBodies.size()>0);
    _report.reset(new CollisionReport());
    _parameters = parameters;
    if( !!parameters ) {
        _specvel = parameters->_configurationspecification.ConvertToVelocitySpecification();
        _setvelstatefn = _specvel.GetSetFn(_listCheckBodies.front()->GetEnv());
    }
}

void DynamicsCollisionConstraint::SetPlannerParameters(PlannerBase::PlannerParametersPtr parameters)
{
    _parameters = parameters;
    if( !!parameters ) {
        _specvel = parameters->_configurationspecification.ConvertToVelocitySpecification();
        _setvelstatefn = _specvel.GetSetFn(_listCheckBodies.front()->GetEnv());
    }
}

void DynamicsCollisionConstraint::SetUserCheckFunction(const boost::function<bool() >& usercheckfn, bool bCallAfterCheckCollision)
{
    _usercheckfns[bCallAfterCheckCollision] = usercheckfn;
}

void DynamicsCollisionConstraint::SetFilterMask(int filtermask)
{
    _filtermask = filtermask;
}

void DynamicsCollisionConstraint::SetPerturbation(dReal perturbation)
{
    _perturbation = perturbation;
}

int DynamicsCollisionConstraint::_SetAndCheckState(PlannerBase::PlannerParametersPtr params, const std::vector<dReal>& vdofvalues, const std::vector<dReal>& vdofvelocities, const std::vector<dReal>& vdofaccels, int options, ConstraintFilterReturnPtr filterreturn)
{
    if( params->SetStateValues(vdofvalues, 0) != 0 ) {
        return CFO_StateSettingError;
    }
    if( (options & CFO_CheckTimeBasedConstraints) && !!_setvelstatefn && vdofvelocities.size() == vdofvalues.size() ) {
        (*_setvelstatefn)(vdofvelocities);
    }
    int nstateret = _CheckState(vdofaccels, options, filterreturn);
    if( nstateret != 0 ) {
        return nstateret;
    }
    if( (options & CFO_CheckWithPerturbation) && _perturbation > 0 ) {
        // only check collision constraints with the perturbation since they are the only ones that don't have settable limits
        _vperturbedvalues.resize(vdofvalues.size());
        boost::array<dReal,3> perturbations = {{_perturbation,-_perturbation}};
        FOREACH(itperturbation,perturbations) {
            for(size_t i = 0; i < vdofvalues.size(); ++i) {
                _vperturbedvalues[i] = vdofvalues[i] + *itperturbation * params->_vConfigResolution.at(i);
                if( _vperturbedvalues[i] < params->_vConfigLowerLimit.at(i) ) {
                    _vperturbedvalues[i] = params->_vConfigLowerLimit.at(i);
                }
                if( _vperturbedvalues[i] > params->_vConfigUpperLimit.at(i) ) {
                    _vperturbedvalues[i] = params->_vConfigUpperLimit.at(i);
                }
            }
            if( params->SetStateValues(_vperturbedvalues, 0) != 0 ) {
                return CFO_StateSettingError|CFO_CheckWithPerturbation;
            }
            int nstateret = _CheckState(vdofaccels, options, filterreturn);
            if( nstateret != 0 ) {
                return nstateret;
            }
        }
    }
    return 0;
}

int DynamicsCollisionConstraint::_CheckState(const std::vector<dReal>& vdofaccels, int options, ConstraintFilterReturnPtr filterreturn)
{
    options &= _filtermask;
    if( (options&CFO_CheckUserConstraints) && !!_usercheckfns[0] ) {
        if( !_usercheckfns[0]() ) {
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                _PrintOnFailure("pre usercheckfn failed");
            }
            return CFO_CheckUserConstraints;
        }
    }
    if( options & CFO_CheckTimeBasedConstraints ) {
        // check dynamics
        FOREACHC(itbody, _listCheckBodies) {
            KinBodyPtr pbody = *itbody;
            _vtorquevalues.resize(0);
            FOREACHC(itjoint, pbody->GetJoints()) {
                for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                    dReal fmaxtorque = (*itjoint)->GetMaxTorque(idof);
                    if( fmaxtorque > 0 ) {
                        _vtorquevalues.push_back(make_pair((*itjoint)->GetDOFIndex()+idof,fmaxtorque));
                    }
                }
            }
            if( _vtorquevalues.size() > 0 && vdofaccels.size() > 0 ) {
                _doftorques.resize(pbody->GetDOF(),0);
                _dofaccelerations.resize(pbody->GetDOF(),0);
                _vdofindices.resize(pbody->GetDOF());
                for(int i = 0; i < pbody->GetDOF(); ++i) {
                    _vdofindices[i] = i;
                }

                // have to extract the correct accelerations from vdofaccels use specvel and timederivative=1
                _specvel.ExtractJointValues(_dofaccelerations.begin(), vdofaccels.begin(), pbody, _vdofindices, 1);

                // compute inverse dynamics and check
                pbody->ComputeInverseDynamics(_doftorques, _dofaccelerations);
                FOREACH(it, _vtorquevalues) {
                    int index = it->first;
                    dReal fmaxtorque = it->second;
                    if( RaveFabs(_doftorques.at(index)) > fmaxtorque ) {
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            _PrintOnFailure(str(boost::format("rejected torque due to joint %s (%d): %e > %e")%pbody->GetJointFromDOFIndex(index)->GetName()%index%RaveFabs(_doftorques.at(index))%fmaxtorque));
                        }
                        return CFO_CheckTimeBasedConstraints;
                    }
                }
            }
        }
    }
    FOREACHC(itbody, _listCheckBodies) {
        if( (options&CFO_CheckEnvCollisions) && (*itbody)->GetEnv()->CheckCollision(KinBodyConstPtr(*itbody),_report) ) {
            if( (options & CFO_FillCollisionReport) && !!filterreturn ) {
                filterreturn->_report = *_report;
            }
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                _PrintOnFailure(std::string("collision failed ")+_report->__str__());
            }
            return CFO_CheckEnvCollisions;
        }
        if( (options&CFO_CheckSelfCollisions) && (*itbody)->CheckSelfCollision(_report) ) {
            if( (options & CFO_FillCollisionReport) && !!filterreturn ) {
                filterreturn->_report = *_report;
            }
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                _PrintOnFailure(std::string("self-collision failed ")+_report->__str__());
            }
            return CFO_CheckSelfCollisions;
        }
    }
    if( (options&CFO_CheckUserConstraints) && !!_usercheckfns[1] ) {
        if( !_usercheckfns[1]() ) {
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                _PrintOnFailure("post usercheckfn failed");
            }
            return CFO_CheckUserConstraints;
        }
    }
    return 0;
}

void DynamicsCollisionConstraint::_PrintOnFailure(const std::string& prefix)
{
    if( IS_DEBUGLEVEL(Level_Verbose) ) {
        PlannerBase::PlannerParametersPtr params = _parameters.lock();
        std::vector<dReal> vcurrentvalues;
        params->_getstatefn(vcurrentvalues);
        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
        ss << prefix << ", ";
        for(size_t i = 0; i < vcurrentvalues.size(); ++i ) {
            if( i > 0 ) {
                ss << "," << vcurrentvalues[i];
            }
            else {
                ss << "colvalues=[" << vcurrentvalues[i];
            }
        }
        ss << "]";
        RAVELOG_VERBOSE(ss.str());
    }
}

int DynamicsCollisionConstraint::Check(const std::vector<dReal>& q0, const std::vector<dReal>& q1, const std::vector<dReal>& dq0, const std::vector<dReal>& dq1, dReal timeelapsed, IntervalType interval, int options, ConstraintFilterReturnPtr filterreturn)
{
    int maskoptions = options&_filtermask;
    if( !!filterreturn ) {
        filterreturn->Clear();
    }
    // set the bounds based on the interval type
    PlannerBase::PlannerParametersPtr params = _parameters.lock();
    if( !params ) {
        RAVELOG_WARN("parameters have been destroyed!\n");
        if( !!filterreturn ) {
            filterreturn->_returncode = CFO_StateSettingError;
        }
        return CFO_StateSettingError;
    }
    BOOST_ASSERT(_listCheckBodies.size()>0);
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
    _vtempvelconfig.resize(dq0.size());
    // if velocity is valid, compute the acceleration for every DOF
    if( timeelapsed > 0 && dq0.size() == _vtempconfig.size() && dq1.size() == _vtempconfig.size() ) {
        // do quadratic interpolation, so make sure the positions, velocities, and timeelapsed are consistent
        // v0 + timeelapsed*0.5*(dq0+dq1) - v1 = 0
        _vtempaccelconfig.resize(dq0.size());
        dReal itimeelapsed = 1.0/timeelapsed;
        for(size_t i = 0; i < _vtempaccelconfig.size(); ++i) {
            _vtempaccelconfig[i] = (dq1.at(i)-dq0.at(i))*itimeelapsed;
            if( IS_DEBUGLEVEL(Level_Verbose) || IS_DEBUGLEVEL(Level_VerifyPlans) ) {
                dReal consistencyerror = RaveFabs(q0.at(i) + timeelapsed*0.5*(dq0.at(i)+dq1.at(i)) - q1.at(i));
                if( RaveFabs(consistencyerror-2*PI) > g_fEpsilonQuadratic ) { // TODO, officially track circular joints
                    OPENRAVE_ASSERT_OP(consistencyerror,<=,g_fEpsilonQuadratic*100);
                }
            }
        }
    }
    else {
        // make sure size is set to DOF
        _vtempaccelconfig.resize(params->GetDOF());
        FOREACH(it, _vtempaccelconfig) {
            *it = 0;
        }
    }

    if (bCheckEnd) {
        int nstateret = _SetAndCheckState(params, q1, dq1, _vtempaccelconfig, maskoptions, filterreturn);
        if( nstateret != 0 ) {
            if( !!filterreturn ) {
                filterreturn->_returncode = nstateret;
                filterreturn->_invalidvalues = q1;
                filterreturn->_invalidvelocities = dq1;
                filterreturn->_fTimeWhenInvalid = timeelapsed > 0 ? timeelapsed : dReal(1.0);
                if( options & CFO_FillCheckedConfiguration ) {
                    filterreturn->_configurations = q1;
                    filterreturn->_configurationtimes.push_back(timeelapsed > 0 ? timeelapsed : dReal(1.0));
                }
            }
            return nstateret;
        }
    }

    // compute  the discretization
    dQ = q1;
    params->_diffstatefn(dQ,q0);
    _vtempveldelta = dq1;
    if( _vtempveldelta.size() == q1.size() ) {
        for(size_t i = 0; i < _vtempveldelta.size(); ++i) {
            _vtempveldelta.at(i) -= dq0.at(i);
        }
    }

    int i, numSteps = 0, nLargestStepIndex = -1;
    std::vector<dReal>::const_iterator itres = params->_vConfigResolution.begin();
    BOOST_ASSERT((int)params->_vConfigResolution.size()==params->GetDOF());
    int totalsteps = 0;
    if( timeelapsed > 0 && dq0.size() == _vtempconfig.size() && dq1.size() == _vtempconfig.size() ) {
        // quadratic equation, so total travelled distance for each joint is not as simple as taking the difference between the two endpoints.
        for (i = 0; i < params->GetDOF(); i++,itres++) {
            int steps = 0;
            if( RaveFabs(_vtempaccelconfig.at(i)) <= g_fEpsilonLinear ) {
                // not a quadratic
                if( *itres != 0 ) {
                    steps = (int)(RaveFabs(dQ[i]) / *itres);
                }
                else {
                    steps = (int)(RaveFabs(dQ[i]) * 100);
                }
            }
            else {
                // 0.5*a*t**2 + v0*t + x0 = x
                dReal inflectiontime = -dq0.at(i) / _vtempaccelconfig.at(i);
                if( inflectiontime >= 0 && inflectiontime < timeelapsed ) {
                    // have to count double
                    dReal inflectionpoint = 0.5*dq0.at(i)*inflectiontime;
                    dReal dist = RaveFabs(inflectionpoint) + RaveFabs(dQ.at(i)-inflectionpoint);
                    if (*itres != 0) {
                        steps = (int)(dist / *itres);
                    }
                    else {
                        steps = (int)(dist * 100);
                    }
                }
                else {
                    if( *itres != 0 ) {
                        steps = (int)(RaveFabs(dQ[i]) / *itres);
                    }
                    else {
                        steps = (int)(RaveFabs(dQ[i]) * 100);
                    }
                }
            }

            totalsteps += steps;
            if (steps > numSteps) {
                numSteps = steps;
                nLargestStepIndex = i;
            }
        }
    }
    else {
        for (i = 0; i < params->GetDOF(); i++,itres++) {
            int steps;
            if( *itres != 0 ) {
                steps = (int)(RaveFabs(dQ[i]) / *itres);
            }
            else {
                steps = (int)(RaveFabs(dQ[i]) * 100);
            }
            totalsteps += steps;
            if (steps > numSteps) {
                numSteps = steps;
                nLargestStepIndex = i;
            }
        }
    }

    if( totalsteps == 0 && start > 0 ) {
        if( !!filterreturn ) {
            if( bCheckEnd ) {
                if(options & CFO_FillCheckedConfiguration) {
                    filterreturn->_configurations = q1;
                    filterreturn->_configurationtimes.push_back(timeelapsed > 0 ? timeelapsed : dReal(1.0));
                }
            }

        }
        return 0;
    }

    if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
        if( (int)filterreturn->_configurations.capacity() < (1+numSteps)*params->GetDOF() ) {
            filterreturn->_configurations.reserve((1+numSteps)*params->GetDOF());
        }
        if( (int)filterreturn->_configurationtimes.capacity() < 1+numSteps) {
            filterreturn->_configurationtimes.reserve(1+numSteps);
        }
    }
    if (start == 0 ) {
        int nstateret = _SetAndCheckState(params, q0, dq0, _vtempaccelconfig, maskoptions, filterreturn);
        if( options & CFO_FillCheckedConfiguration ) {
            filterreturn->_configurations.insert(filterreturn->_configurations.begin(), q0.begin(), q0.end());
            filterreturn->_configurationtimes.insert(filterreturn->_configurationtimes.begin(), 0);
        }
        if( nstateret != 0 ) {
            if( !!filterreturn ) {
                filterreturn->_returncode = nstateret;
                filterreturn->_invalidvalues = q0;
                filterreturn->_invalidvelocities = dq0;
                filterreturn->_fTimeWhenInvalid = 0;
            }
            return nstateret;
        }
        start = 1;
    }

    if( numSteps == 0 ) {
        // everything is so small that there is no interpolation...
        if( bCheckEnd && !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
            filterreturn->_configurations.insert(filterreturn->_configurations.end(), q1.begin(), q1.end());
            filterreturn->_configurationtimes.push_back(timeelapsed);
        }

        return 0;
    }

    for (i = 0; i < params->GetDOF(); i++) {
        _vtempconfig.at(i) = q0.at(i);
    }
    if( dq0.size() == q0.size() ) {
        _vtempvelconfig = dq0;
    }

    if( timeelapsed > 0 && dq0.size() == _vtempconfig.size() && dq1.size() == _vtempconfig.size() ) {
        // just in case, have to set the current values to _vtempconfig since neightstatefn expects the state to be set.
        if( params->SetStateValues(_vtempconfig, 0) != 0 ) {
            return CFO_StateSettingError;
        }

        // quadratic interpolation
        // given the nLargestStepIndex, determine the timestep for all joints
        dReal fLargestStepAccel = _vtempaccelconfig.at(nLargestStepIndex);
        dReal fLargestStepInitialVelocity = dq0.at(nLargestStepIndex);
        dReal fLargestStep = dQ.at(nLargestStepIndex);
        dReal fLargestInflectionTime = timeelapsed; // if >= 0 and <= timeelapsed, then the step increment changes signs
        dReal fLargestInflection = dQ.at(nLargestStepIndex); // the max/min
        dReal fLargestTraveledDistance = fLargestInflection;
        dReal fLargestStepDelta;
        if( RaveFabs(fLargestStepAccel) > g_fEpsilonLinear ) {
            dReal fInflectionTime = -fLargestStepInitialVelocity / fLargestStepAccel;
            if( fInflectionTime >= 0 && fInflectionTime < timeelapsed ) {
                // quadratic hits its min/max during the time interval
                fLargestInflectionTime = fInflectionTime;
                fLargestInflection = 0.5*fLargestStepInitialVelocity*fLargestInflectionTime;
                fLargestTraveledDistance = RaveFabs(fLargestInflection) + RaveFabs(dQ.at(nLargestStepIndex) - fLargestInflection);
                fLargestStepDelta = fLargestTraveledDistance/numSteps;
                if( fLargestInflection < 0 || (RaveFabs(fLargestInflection)<=g_fEpsilonLinear && dQ.at(nLargestStepIndex)<0) ) {
                    fLargestStepDelta = -fLargestStepDelta;
                }
            }
            else {
                fLargestStepDelta = fLargestTraveledDistance/dReal(numSteps);
            }
        }
        else {
            fLargestStepDelta = fLargestTraveledDistance/dReal(numSteps);
        }

        dReal timesteproots[2], timestep=0;
        dReal fStep = 0;
        int istep = 0;
        dReal prevtimestep = 0;
        bool bSurpassedInflection = false;
        while(istep < numSteps && prevtimestep < timeelapsed) {
            //for (int istep = 0; istep < numSteps; istep++, fStep += fLargestStepDelta) {
            int nstateret = 0;
            if( istep >= start ) {
                nstateret = _SetAndCheckState(params, _vtempconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                if( !!params->_getstatefn ) {
                    params->_getstatefn(_vtempconfig);     // query again in order to get normalizations/joint limits
                }
                if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                    filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
                    filterreturn->_configurationtimes.push_back(timestep);
                }
            }
            if( nstateret != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = nstateret;
                }
                return nstateret;
            }

            dReal fBestNewStep = bSurpassedInflection ? (fStep-fLargestStepDelta) : (fStep+fLargestStepDelta);

            if( RaveFabs(fLargestStepAccel) <= g_fEpsilonLinear ) {
                OPENRAVE_ASSERT_OP(RaveFabs(fLargestStepInitialVelocity),>,g_fEpsilon);
                timestep = fStep/fLargestStepInitialVelocity;
            }
            else {
                // check whether to go positive or negative direction
                dReal fNewStep = bSurpassedInflection ? (fStep-fLargestStepDelta) : (fStep+fLargestStepDelta);
                bool bfound = false;

                int numroots = mathextra::solvequad(fLargestStepAccel*0.5, fLargestStepInitialVelocity, -fNewStep, timesteproots[0], timesteproots[1]);
                if( numroots == 0 ) {
                    if( RaveFabs(fNewStep-fLargestStep) < 1e-7 ) { // in order to avoid solvequat not returning any solutions
                        timestep = timeelapsed;
                        bfound = true;
                    }
                }
                if( !bfound ) {
                    for(int i = 0; i < numroots; ++i) {
                        if( timesteproots[i] > prevtimestep && (!bfound || timestep > timesteproots[i]) ) {
                            timestep = timesteproots[i];
                            bfound = true;
                        }
                    }

                    if( !bfound && !bSurpassedInflection ) {
                        fNewStep = fStep; //-fLargestStepDelta;
                        numroots = mathextra::solvequad(fLargestStepAccel*0.5, fLargestStepInitialVelocity, -fNewStep, timesteproots[0], timesteproots[1]);
                        for(int i = 0; i < numroots; ++i) {
                            if( timesteproots[i] > prevtimestep && (!bfound || timestep > timesteproots[i]) ) {
                                // going backwards!
                                timestep = timesteproots[i];
                                fBestNewStep = fNewStep;
                                bSurpassedInflection = true;
                                bfound = true;
                            }
                        }
                    }
                }
                if( !bfound ) {
                    RAVELOG_WARN("cannot take root for quadratic interpolation\n");
                    if( !!filterreturn ) {
                        filterreturn->_returncode = CFO_StateSettingError;
                    }
                    return CFO_StateSettingError;
                }
            }

//            if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
//                filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
//                filterreturn->_configurationtimes.push_back(timestep);
//            }
            if( nstateret != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = nstateret;
                    filterreturn->_invalidvalues = _vtempconfig;
                    filterreturn->_invalidvelocities = _vtempvelconfig;
                    filterreturn->_fTimeWhenInvalid = timestep;
                }
                return nstateret;
            }
            if( timestep > timeelapsed+1e-7 ) {
                if( istep+1 >= numSteps ) {
                    // expected...
                    break;
                }
                RAVELOG_WARN_FORMAT("timestep %.15e > total time of ramp %.15e, step %d/%d", timestep%timeelapsed%istep%numSteps);
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            else if( timestep > timeelapsed ) {
                timestep = timeelapsed; // get rid of small epsilons
            }
            for(size_t i = 0; i < _vtempconfig.size(); ++i) {
                dQ[i] = q0.at(i) + timestep * (dq0.at(i) + timestep * 0.5 * _vtempaccelconfig.at(i)) - _vtempconfig.at(i);
                _vtempvelconfig.at(i) = dq0.at(i) + timestep*_vtempaccelconfig.at(i);
            }
            if( !params->_neighstatefn(_vtempconfig, dQ,0) ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            fStep = fBestNewStep;
            ++istep;
            //RAVELOG_VERBOSE_FORMAT("fStep=%.15e, fLargestStep=%.15e, timestep=%.15e", fStep%fLargestStep%timestep);
            prevtimestep = timestep;
        }
        if( RaveFabs(fStep-fLargestStep) > RaveFabs(fLargestStepDelta) ) {
            RAVELOG_WARN_FORMAT("fStep (%.15e) did not reach fLargestStep (%.15e). %.15e > %.15e", fStep%fLargestStep%RaveFabs(fStep-fLargestStep)%fLargestStepDelta);
            if( !!filterreturn ) {
                filterreturn->_returncode = CFO_StateSettingError;
            }
            // this is a bug, just return false
            return CFO_StateSettingError;
        }
    }
    else {
        // check for collision along the straight-line path
        // NOTE: this does not check the end config, and may or may
        // not check the start based on the value of 'start'
        dReal fisteps = dReal(1.0f)/numSteps;
        for(std::vector<dReal>::iterator it = dQ.begin(); it != dQ.end(); ++it) {
            *it *= fisteps;
        }
        for(std::vector<dReal>::iterator it = _vtempveldelta.begin(); it != _vtempveldelta.end(); ++it) {
            *it *= fisteps;
        }

        if( start > 0 ) {
            if( !params->_neighstatefn(_vtempconfig, dQ,0) ) {
                return CFO_StateSettingError;
            }
            for(size_t i = 0; i < _vtempveldelta.size(); ++i) {
                _vtempvelconfig.at(i) += _vtempveldelta[i];
            }
        }
        for (int f = start; f < numSteps; f++) {
            int nstateret = _SetAndCheckState(params, _vtempconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
            if( !!params->_getstatefn ) {
                params->_getstatefn(_vtempconfig);     // query again in order to get normalizations/joint limits
            }
            if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
                filterreturn->_configurationtimes.push_back(f*fisteps);
            }
            if( nstateret != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = nstateret;
                    filterreturn->_invalidvalues = _vtempconfig;
                    filterreturn->_invalidvelocities = _vtempvelconfig;
                    filterreturn->_fTimeWhenInvalid = f*fisteps;
                }
                return nstateret;
            }

            if( !params->_neighstatefn(_vtempconfig,dQ,0) ) {
                return CFO_StateSettingError;
            }
            for(size_t i = 0; i < _vtempveldelta.size(); ++i) {
                _vtempvelconfig.at(i) += _vtempveldelta[i];
            }
        }
    }

    if( bCheckEnd && !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
        filterreturn->_configurations.insert(filterreturn->_configurations.end(), q1.begin(), q1.end());
        filterreturn->_configurationtimes.push_back(timeelapsed);
    }

    return 0;
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

SimpleNeighborhoodSampler::SimpleNeighborhoodSampler(SpaceSamplerBasePtr psampler, const PlannerBase::PlannerParameters::DistMetricFn& distmetricfn, const PlannerBase::PlannerParameters::DiffStateFn& diffstatefn) : _psampler(psampler), _distmetricfn(distmetricfn), _diffstatefn(diffstatefn)
{
}

bool SimpleNeighborhoodSampler::Sample(std::vector<dReal>& vNewSample, const std::vector<dReal>& vCurSample, dReal fRadius)
{
    if( fRadius <= g_fEpsilonLinear ) {
        vNewSample = vCurSample;
        return true;
    }
    _psampler->SampleSequence(vNewSample);
    size_t dof = vCurSample.size();
    BOOST_ASSERT(dof==vNewSample.size() && &vNewSample != &vCurSample);
    dReal fDist = _distmetricfn(vNewSample,vCurSample);
    // most distance metrics preserve the following property:
    // d=dist(x, x + y) ==> d*t == dist(x, x + t*y)
    if( fDist > fRadius ) {
        dReal fMult = fRadius/fDist;
        _diffstatefn(vNewSample, vCurSample);
        for(size_t i = 0; i < vNewSample.size(); ++i) {
            vNewSample[i] = vNewSample[i]*fMult + vCurSample[i];
        }
    }
    return true;
}

bool SimpleNeighborhoodSampler::Sample(std::vector<dReal>& samples)
{
    _psampler->SampleSequence(samples,1,IT_Closed);
    return samples.size()>0;
}

ManipulatorIKGoalSampler::ManipulatorIKGoalSampler(RobotBase::ManipulatorConstPtr pmanip, const std::list<IkParameterization>& listparameterizations, int nummaxsamples, int nummaxtries, dReal fsampleprob, bool searchfreeparameters, int ikfilteroptions) : _pmanip(pmanip), _nummaxsamples(nummaxsamples), _nummaxtries(nummaxtries), _fsampleprob(fsampleprob), _ikfilteroptions(ikfilteroptions), _searchfreeparameters(searchfreeparameters)
{
    _tempikindex = -1;
    _fjittermaxdist = 0;
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
    IkReturnPtr ikreturn = Sample();
    if( !ikreturn ) {
        vgoal.resize(0);
        return false;
    }
    vgoal.swap(ikreturn->_vsolution); // won't be using the ik return anymore
    return true;
}

IkReturnPtr ManipulatorIKGoalSampler::Sample()
{
    std::vector<dReal> vindex;
    _pindexsampler->SampleSequence(vindex,1,IT_OpenEnd);
    if( vindex.at(0) > _fsampleprob ) {
        return IkReturnPtr();
    }
    if( _vikreturns.size() > 0 ) {
        IkReturnPtr ikreturnlocal = _vikreturns.back();
        _vikreturns.pop_back();
        _listreturnedsamples.push_back(_tempikindex);
        if( _vikreturns.size() == 0 ) {
            _tempikindex = -1;
        }
        return ikreturnlocal;
    }
    IkReturnPtr ikreturnjittered;
    for(int itry = 0; itry < _nummaxtries; ++itry ) {
        if( _listsamples.size() == 0 ) {
            return IkReturnPtr();
        }
        _pindexsampler->SampleSequence(vindex,1,IT_OpenEnd);
        int isampleindex = (int)(vindex.at(0)*_listsamples.size());
        std::list<SampleInfo>::iterator itsample = _listsamples.begin();
        advance(itsample,isampleindex);

        int numRedundantSamplesForEEChecking = 0;
        if( (int)_pmanip->GetArmIndices().size() > itsample->_ikparam.GetDOF() ) {
            numRedundantSamplesForEEChecking = 40;
        }

        bool bFullEndEffectorKnown = itsample->_ikparam.GetType() == IKP_Transform6D || _pmanip->GetArmDOF() <= itsample->_ikparam.GetDOF();
        bool bCheckEndEffector = true;
        if( _ikfilteroptions & IKFO_IgnoreEndEffectorEnvCollisions ) {
            // use requested end effector to be always ignored
            bCheckEndEffector = false;
        }
        // if first grasp, quickly prune grasp is end effector is in collision
        IkParameterization ikparam = itsample->_ikparam;
        if( itsample->_numleft == _nummaxsamples && bCheckEndEffector ) { //!(_ikfilteroptions & IKFO_IgnoreEndEffectorEnvCollisions) ) {
            // because a goal can be colliding, have to always go in this loop and check if the end effector
            // could be jittered.
            // if bCheckEndEffector is true, then should call CheckEndEffectorCollision to quickly prune samples; otherwise, have to rely on calling FindIKSolution
            try {
                if( _pmanip->CheckEndEffectorCollision(ikparam,_report, numRedundantSamplesForEEChecking) ) {
                    bool bcollision=true;
                    if( _fjittermaxdist > 0 ) {
                        // try jittering the end effector out
                        RAVELOG_VERBOSE_FORMAT("starting jitter transform %f...", _fjittermaxdist);
                        // randomly add small offset to the ik until it stops being in collision
                        Transform tjitter;
                        // before random sampling, first try sampling along the axes. try order z,y,x since z is most likely gravity
                        int N = 4;
                        dReal delta = _fjittermaxdist/N;
                        for(int iaxis = 2; iaxis >= 0; --iaxis) {
                            tjitter.trans = Vector();
                            for(int iiter = 0; iiter < 2*N; ++iiter) {
                                tjitter.trans[iaxis] = _fjittermaxdist*delta*(1+iiter/2);
                                if( iiter & 1 ) {
                                    // revert sign
                                    tjitter.trans[iaxis] = -tjitter.trans[iaxis];
                                }
                                IkParameterization ikparamjittered = tjitter * ikparam;
                                try {
                                    if( !_pmanip->CheckEndEffectorCollision(ikparamjittered,_report, numRedundantSamplesForEEChecking) ) {
                                        // make sure at least one ik solution exists...
                                        if( !ikreturnjittered ) {
                                            ikreturnjittered.reset(new IkReturn(IKRA_Success));
                                        }
                                        bool biksuccess = _pmanip->FindIKSolution(ikparamjittered, _ikfilteroptions, ikreturnjittered);
                                        if( biksuccess ) {
                                            ikparam = ikparamjittered;
                                            bcollision = false;
                                            break;
                                        }
                                        else {
                                            RAVELOG_VERBOSE_FORMAT("jitter succeed position, but ik failed: 0x%.8x", ikreturnjittered->_action);
                                        }
                                    }
                                }
                                catch(const std::exception& ex) {
                                    // ignore most likely ik failed in CheckEndEffectorCollision
                                }
                            }
                            if( !bcollision ) {
                                break;
                            }
                        }

                        if( bcollision ) {
                            // try random samples, most likely will fail...
                            int nMaxIterations = 100;
                            std::vector<dReal> xyzsamples(3);
                            dReal delta = (_fjittermaxdist*2)/nMaxIterations;
                            for(int iiter = 1; iiter <= nMaxIterations; ++iiter) {
                                _pindexsampler->SampleSequence(xyzsamples,3,IT_Closed);
                                tjitter.trans = Vector(xyzsamples[0]-0.5f, xyzsamples[1]-0.5f, xyzsamples[2]-0.5f) * (delta*iiter);
                                IkParameterization ikparamjittered = tjitter * ikparam;
                                try {
                                    if( !_pmanip->CheckEndEffectorCollision(ikparamjittered, _report, numRedundantSamplesForEEChecking) ) {
                                        if( !ikreturnjittered ) {
                                            ikreturnjittered.reset(new IkReturn(IKRA_Success));
                                        }
                                        bool biksuccess = _pmanip->FindIKSolution(ikparamjittered, _ikfilteroptions, ikreturnjittered);
                                        if( biksuccess ) {
                                            ikparam = ikparamjittered;
                                            bcollision = false;
                                            break;
                                        }
                                        else {
                                            RAVELOG_VERBOSE_FORMAT("jitter succed position, but ik failed: 0x%.8x", ikreturnjittered->_action);
                                        }
                                    }
                                }
                                catch(const std::exception& ex) {
                                    // ignore most likely ik failed in CheckEndEffectorCollision
                                }
                            }
                        }
                    }
                    if( bcollision ) {
                        RAVELOG_VERBOSE(str(boost::format("sampleiksolutions gripper in collision: %s.\n")%_report->__str__()));
                        _listsamples.erase(itsample);
                        continue;
                    }
                }
            }
            catch(const std::exception& ex) {
                if( itsample->_ikparam.GetType() == IKP_Transform6D ) {
                    RAVELOG_WARN(str(boost::format("CheckEndEffectorCollision threw exception: %s")%ex.what()));
                }
                else {
                    // most likely the ik couldn't get solved
                    RAVELOG_VERBOSE(str(boost::format("sampleiksolutions failed to solve ik: %s.\n")%ex.what()));
                    _listsamples.erase(itsample);
                    continue;
                }
            }
        }

        std::vector<dReal> vfree;
        int orgindex = itsample->_orgindex;
        if( _pmanip->GetIkSolver()->GetNumFreeParameters() > 0 ) {

            if( _searchfreeparameters ) {
                if( !itsample->_psampler ) {
                    itsample->_psampler = RaveCreateSpaceSampler(_probot->GetEnv(),"halton");
                    itsample->_psampler->SetSpaceDOF(_pmanip->GetIkSolver()->GetNumFreeParameters());
                }
                itsample->_psampler->SampleSequence(vfree,1);
                // it's pretty dangerous to add _vfreestart since if it starts on a joint limit (0), then it will start exploring from each of the joint limits. rather, we want ik solutions that are away from joint limits
//                for(size_t i = 0; i < _vfreestart.size(); ++i) {
//                    vfree.at(i) += _vfreestart[i];
//                    if( vfree[i] < 0 ) {
//                        vfree[i] += 1;
//                    }
//                    if( vfree[i] > 1 ) {
//                        vfree[i] -= 1;
//                    }
//                }
            }
            else {
                _pmanip->GetIkSolver()->GetFreeParameters(vfree);
            }
        }
        bool bsuccess = _pmanip->FindIKSolutions(ikparam, vfree, _ikfilteroptions|(bFullEndEffectorKnown&&bCheckEndEffector ? IKFO_IgnoreEndEffectorEnvCollisions : 0), _vikreturns);
        if( --itsample->_numleft <= 0 || vfree.size() == 0 || !_searchfreeparameters ) {
            _listsamples.erase(itsample);
        }

        if( bsuccess ) {
            _tempikindex = orgindex;
            _listreturnedsamples.push_back(orgindex);
            IkReturnPtr ikreturnlocal = _vikreturns.back();
            _vikreturns.pop_back();
            if( _vikreturns.size() == 0 ) {
                _tempikindex = -1;
            }
            return ikreturnlocal;
        }
    }
    return IkReturnPtr();
}

bool ManipulatorIKGoalSampler::SampleAll(std::list<IkReturnPtr>& samples, int maxsamples, int maxchecksamples)
{
    // currently this is a very slow implementation...
    samples.clear();
    int numchecked=0;
    while(true) {
        IkReturnPtr ikreturn = Sample();
        if( !ikreturn ) {
            break;
        }
        samples.push_back(ikreturn);
        if( maxsamples > 0 && (int)samples.size() >= maxsamples ) {
            return true;
        }
        RAVELOG_VERBOSE(str(boost::format("computed %d samples")%samples.size()));
        numchecked += 1;
        if( maxchecksamples > 0 && numchecked >= maxchecksamples ) {
            break;
        }
    }
    return samples.size()>0;
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

void ManipulatorIKGoalSampler::SetJitter(dReal maxdist)
{
    _fjittermaxdist = maxdist;
}

} // planningutils
} // OpenRAVE
