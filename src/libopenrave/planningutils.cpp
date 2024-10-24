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

#include <boost/bind/bind.hpp>

using namespace boost::placeholders;

namespace OpenRAVE {
namespace planningutils {

static const dReal g_fEpsilonQuadratic = RavePow(g_fEpsilon,0.55); // should be 0.6...perhaps this is related to parabolic smoother epsilons?
static const dReal g_fEpsilonCubic = RavePow(g_fEpsilon,0.55);
static const dReal g_fEpsilonQuintic = RavePow(g_fEpsilon,0.55);

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
            if( neighstatefn(newdof,deltadof,0) == NSS_Failed ) {
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
                if( neighstatefn(newdof,deltadof2,0) == NSS_Failed ) {
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
                if( neighstatefn(newdof,deltadof,0) == NSS_Failed ) {
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
                stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
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
            if( parameters->_neighstatefn(newdof,deltadof,0) == NSS_Failed ) {
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
                stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
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
                if( parameters->_neighstatefn(newdof,deltadof2,0) == NSS_Failed ) {
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
                    stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
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
                if( parameters->_neighstatefn(newdof,deltadof,0) == NSS_Failed ) {
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
                stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
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
                if( _parameters->_neighstatefn(newq,vdiff,NSO_OnlyHardConstraints) == NSS_Failed ) {
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

                // Check if the trajectory has all-linear interpolation
                ConfigurationSpecification trajspec = trajectory->GetConfigurationSpecification();
                vector<ConfigurationSpecification::Group>::const_iterator itvaluesgroup = trajspec.FindCompatibleGroup("joint_values", false);
                vector<ConfigurationSpecification::Group>::const_iterator itvelocitiesgroup = trajspec.FindCompatibleGroup("joint_velocities", false);
                vector<ConfigurationSpecification::Group>::const_iterator itaccelerationsgroup = trajspec.FindCompatibleGroup("joint_accelerations", false);
                bool bHasAllLinearInterpolation = false;
                if( (itvaluesgroup == trajspec._vgroups.end() || itvaluesgroup->interpolation == "linear") &&
                    (itvelocitiesgroup == trajspec._vgroups.end() || itvelocitiesgroup->interpolation == "linear") &&
                    (itaccelerationsgroup == trajspec._vgroups.end() || itaccelerationsgroup->interpolation == "linear") ) {
                    bHasAllLinearInterpolation = true;
                }
                IntervalType interval = bHasAllLinearInterpolation ? (IntervalType)(IT_Closed | IT_AllLinear) : IT_Closed;

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
                    if( _parameters->CheckPathAllConstraints(vprevdata,vdata,vprevdatavel, vdatavel, deltatime, interval, 0xffff|CFO_FillCheckedConfiguration, filterreturn) != 0 ) {
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            _parameters->CheckPathAllConstraints(vprevdata,vdata,vprevdatavel, vdatavel, deltatime, interval, 0xffff|CFO_FillCheckedConfiguration, filterreturn);
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
                            throw OPENRAVE_EXCEPTION_FORMAT(_("time %fs-%fs, failed to set state values"), *itprevtime%*itsampletime, ORE_InconsistentConstraints);
                        }
                        vector<dReal> vtemp = vprevconfig;
                        if( _parameters->_neighstatefn(vtemp,deltaq,NSO_OnlyHardConstraints) == NSS_Failed ) {
                            throw OPENRAVE_EXCEPTION_FORMAT(_("time %fs-%fs, neighstatefn is rejecting configurations from CheckPathAllConstraints, wrote trajectory to %s"),*itprevtime%*itsampletime%DumpTrajectory(trajectory),ORE_InconsistentConstraints);
                        }
                        else {
                            dReal fprevdist = _parameters->_distmetricfn(vprevconfig,vtemp);
                            dReal fcurdist = _parameters->_distmetricfn(vcurconfig,vtemp);
                            if( fprevdist > g_fEpsilonLinear ) {
                                OPENRAVE_ASSERT_OP_FORMAT(fprevdist, >, fcurdist, "time %fs-%fs, neighstatefn returned a configuration closer to the previous configuration %f than the expected current %f, wrote trajectory to %s",*itprevtime%*itsampletime%fprevdist%fcurdist%DumpTrajectory(trajectory), ORE_InconsistentConstraints);
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
        return PlannerStatus(PS_HasSolution);
    }

    EnvironmentBasePtr env = traj->GetEnv();
    EnvironmentLock lockenv(env->GetMutex());
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
        params->_checkpathvelocityconstraintsfn.clear();
    }

    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = hastimestamps;
    params->_sExtraParameters += plannerparameters;

    PlannerStatus statusFromInit = planner->InitPlan(probot,params);
    if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
        return statusFromInit;
    }
    PlannerStatus plannerStatus = planner->PlanPath(traj);
    if( plannerStatus.GetStatusCode() != PS_HasSolution ) {
        return plannerStatus;
    }

    if( bsmooth && (RaveGetDebugLevel() & Level_VerifyPlans) ) {
        RobotBase::RobotStateSaver saver(probot);
        planningutils::VerifyTrajectory(params,traj);
    }
    return PlannerStatus(PS_HasSolution);
}

ActiveDOFTrajectorySmoother::ActiveDOFTrajectorySmoother(RobotBasePtr robot, const std::string& _plannername, const std::string& plannerparameters)
{
    std::string plannername = _plannername.size() > 0 ? _plannername : "parabolicsmoother";
    _robot = robot;
    EnvironmentLock lockenv(robot->GetEnv()->GetMutex());
    _vRobotActiveIndices = _robot->GetActiveDOFIndices();
    _nRobotAffineDOF = _robot->GetAffineDOF();
    _vRobotRotationAxis = _robot->GetAffineRotationAxis();
    _planner = RaveCreatePlanner(robot->GetEnv(),plannername);
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->SetRobotActiveJoints(_robot);
    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = false;
    params->_sExtraParameters += plannerparameters;

    PlannerStatus statusFromInit = _planner->InitPlan(_robot,params);
    if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
        rapidjson::Document rStatus(rapidjson::kObjectType);
        statusFromInit.SaveToJson(rStatus, rStatus.GetAllocator());
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s:%s"), plannername%_robot->GetName()%(orjson::DumpJson(rStatus)), ORE_InvalidArguments);
    }

    _parameters=params; // necessary because SetRobotActiveJoints builds functions that hold weak_ptr to the parameters
    _changehandler = robot->RegisterChangeCallback(KinBody::Prop_JointAccelerationVelocityTorqueLimits|KinBody::Prop_JointLimits|KinBody::Prop_JointProperties, boost::bind(&ActiveDOFTrajectorySmoother::_UpdateParameters, this));
}

PlannerStatus ActiveDOFTrajectorySmoother::PlanPath(TrajectoryBasePtr traj, int planningoptions)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need velocities, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PlannerStatus(PS_HasSolution);
    }

    EnvironmentBasePtr env = traj->GetEnv();
    CollisionOptionsStateSaver optionstate(env->GetCollisionChecker(),env->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);
    PlannerStatus status = _planner->PlanPath(traj, planningoptions);
    if( status.GetStatusCode() & PS_HasSolution ) {
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

    PlannerStatus statusFromInit = _planner->InitPlan(_robot,params);
    if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
        rapidjson::Document rStatus(rapidjson::kObjectType);
        statusFromInit.SaveToJson(rStatus, rStatus.GetAllocator());
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s:%s"), _planner->GetXMLId()%_robot->GetName()%(orjson::DumpJson(rStatus)), ORE_InvalidArguments);
    }

    _parameters=params; // necessary because SetRobotActiveJoints builds functions that hold weak_ptr to the parameters
}

ActiveDOFTrajectoryRetimer::ActiveDOFTrajectoryRetimer(RobotBasePtr robot, const std::string& _plannername, const std::string& plannerparameters)
{
    std::string plannername = _plannername.size() > 0 ? _plannername : "parabolicretimer";
    _robot = robot;
    EnvironmentLock lockenv(robot->GetEnv()->GetMutex());
    _vRobotActiveIndices = _robot->GetActiveDOFIndices();
    _nRobotAffineDOF = _robot->GetAffineDOF();
    _vRobotRotationAxis = _robot->GetAffineRotationAxis();
    _planner = RaveCreatePlanner(robot->GetEnv(),plannername);
    TrajectoryTimingParametersPtr params(new TrajectoryTimingParameters());
    params->SetRobotActiveJoints(_robot);
    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = false;
    params->_setstatevaluesfn.clear();
    params->_checkpathvelocityconstraintsfn.clear();
    params->_sExtraParameters = plannerparameters;

    PlannerStatus statusFromInit = _planner->InitPlan(_robot,params);
    if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
        rapidjson::Document rStatus(rapidjson::kObjectType);
        statusFromInit.SaveToJson(rStatus, rStatus.GetAllocator());
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s:%s"), _planner->GetXMLId()%_robot->GetName()%(orjson::DumpJson(rStatus)), ORE_InvalidArguments);
    }
    _parameters=params; // necessary because SetRobotActiveJoints builds functions that hold weak_ptr to the parameters
    _changehandler = robot->RegisterChangeCallback(KinBody::Prop_JointAccelerationVelocityTorqueLimits|KinBody::Prop_JointLimits|KinBody::Prop_JointProperties, boost::bind(&ActiveDOFTrajectoryRetimer::_UpdateParameters, this));
}

PlannerStatus ActiveDOFTrajectoryRetimer::PlanPath(TrajectoryBasePtr traj, bool hastimestamps, int planningoptions)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need velocities, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PlannerStatus(PS_HasSolution);
    }

    TrajectoryTimingParametersPtr parameters = boost::dynamic_pointer_cast<TrajectoryTimingParameters>(_parameters);
    if( parameters->_hastimestamps != hastimestamps ) {
        parameters->_hastimestamps = hastimestamps;
        PlannerStatus statusFromInit = _planner->InitPlan(_robot,parameters);
        if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
            rapidjson::Document rStatus(rapidjson::kObjectType);
            statusFromInit.SaveToJson(rStatus, rStatus.GetAllocator());
            throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s:%s"), _planner->GetXMLId()%_robot->GetName()%(orjson::DumpJson(rStatus)), ORE_InvalidArguments);
        }
    }

    return _planner->PlanPath(traj, planningoptions);
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
    params->_checkpathvelocityconstraintsfn.clear();
    params->_sExtraParameters = _parameters->_sExtraParameters;
    PlannerStatus statusFromInit = _planner->InitPlan(_robot,params);
    if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
        rapidjson::Document rStatus(rapidjson::kObjectType);
        statusFromInit.SaveToJson(rStatus, rStatus.GetAllocator());
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with robot %s:%s"), _planner->GetXMLId()%_robot->GetName()%(orjson::DumpJson(rStatus)), ORE_InvalidArguments);
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
        return PlannerStatus(PS_HasSolution);
    }

    EnvironmentLock lockenv(traj->GetEnv()->GetMutex());
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
        params->_checkpathvelocityconstraintsfn.clear();
    }

    params->_sPostProcessingPlanner = ""; // have to turn off the second post processing stage
    params->_hastimestamps = hastimestamps;
    params->_sExtraParameters += plannerparameters;

    PlannerStatus statusFromInit = planner->InitPlan(RobotBasePtr(),params);
    if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
        return statusFromInit;
    }
    PlannerStatus plannerStatus = planner->PlanPath(traj);
    if( !(plannerStatus.statusCode & PS_HasSolution) ) {
        return plannerStatus;
    }

    if( bsmooth && (RaveGetDebugLevel() & Level_VerifyPlans) ) {
        PlannerBase::PlannerParameters::StateSaver saver(params);
        planningutils::VerifyTrajectory(params,traj);
    }
    return PlannerStatus(PS_HasSolution);
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
            // probably should not throw here
            RAVELOG_WARN_FORMAT(_("failed to set state in PlannerStateSaver, return=%d"), ret);
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
        return PlannerStatus(PS_HasSolution);
    }

    EnvironmentLock lockenv(traj->GetEnv()->GetMutex());
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
        if( !listsetfunctions.empty() ) {
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
        params->_getstatefn.clear();
        params->_checkpathvelocityconstraintsfn.clear();
    }

    params->_diffstatefn = boost::bind(diffstatefn,_1,_2,vrotaxes);

    params->_hastimestamps = hastimestamps;
    params->_sExtraParameters = plannerparameters;

    PlannerStatus statusFromInit = planner->InitPlan(RobotBasePtr(),params);
    if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
        return statusFromInit;
    }

    PlannerStatus plannerStatus = planner->PlanPath(traj);
    if( plannerStatus.GetStatusCode() != PS_HasSolution ) {
        return plannerStatus;
    }
    return PlannerStatus(PS_HasSolution);
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
                PlannerStatus statusFromInit = _planner->InitPlan(RobotBasePtr(), _parameters);
                if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
                    rapidjson::Document rStatus(rapidjson::kObjectType);
                    statusFromInit.SaveToJson(rStatus, rStatus.GetAllocator());
                    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s:%s"), _plannername%(orjson::DumpJson(rStatus)), ORE_InvalidArguments);
                }
            }
        }
    }
}

PlannerStatus AffineTrajectoryRetimer::PlanPath(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, bool hastimestamps, int planningoptions)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // don't need retiming, but should at least add a time group
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        vector<dReal> data;
        traj->GetWaypoints(0,traj->GetNumWaypoints(),data,spec);
        traj->Init(spec);
        traj->Insert(0,data);
        return PlannerStatus(PS_HasSolution);
    }

    EnvironmentBasePtr env = traj->GetEnv();
    EnvironmentLock lockenv(env->GetMutex());
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
        parameters->_getstatefn.clear();
        parameters->_checkpathvelocityconstraintsfn.clear();
        parameters->_sExtraParameters += _extraparameters;
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
        PlannerStatus statusFromInit = _planner->InitPlan(RobotBasePtr(),parameters);
        if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
            stringstream ss; ss << trajspec;
            rapidjson::Document rStatus(rapidjson::kObjectType);
            statusFromInit.SaveToJson(rStatus, rStatus.GetAllocator());
            throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s with affine trajectory spec: %s, msg=%s"), _plannername%ss.str()%(orjson::DumpJson(rStatus)), ORE_InvalidArguments);
        }
    }

    return _planner->PlanPath(traj, planningoptions);
}

PlannerStatus AffineTrajectoryRetimer::PlanPath(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, const std::vector<dReal>& maxjerks, bool hastimestamps, int planningoptions)
{
    if( traj->GetNumWaypoints() == 1 ) {
        // Since traj has only one point, don't need retiming. Anyway, should at least add deltatime group.
        ConfigurationSpecification spec = traj->GetConfigurationSpecification();
        spec.AddDeltaTimeGroup();
        std::vector<dReal> data;
        traj->GetWaypoints(0, traj->GetNumWaypoints(), data, spec);
        traj->Init(spec);
        traj->Insert(0, data);
        return PlannerStatus(PS_HasSolution);
    }

    EnvironmentBasePtr env = traj->GetEnv();
    EnvironmentLock lockenv(env->GetMutex());
    ConfigurationSpecification trajspec = traj->GetConfigurationSpecification().GetTimeDerivativeSpecification(0);
    int trajdof = trajspec.GetDOF();
    if( trajdof != (int)maxvelocities.size() ||
        trajdof != (int)maxaccelerations.size() ||
        trajdof != (int)maxjerks.size() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("env=%s, traj dof=%d do not match maxvelocities size=%d or maxaccelerations size=%d or maxjerks size=%d", env->GetNameId()%trajdof%maxvelocities.size()%maxaccelerations.size()%maxjerks.size(), ORE_InvalidArguments);
    }

    TrajectoryTimingParametersPtr parameters;
    if( !_parameters ) {
        parameters.reset(new TrajectoryTimingParameters());
        parameters->_sPostProcessingPlanner = ""; // turn off the second post-processing stage
        parameters->_setstatevaluesfn.clear();
        parameters->_getstatefn.clear();
        parameters->_checkpathvelocityconstraintsfn.clear();
        parameters->_checkpathvelocityaccelerationconstraintsfn.clear();
        parameters->_sExtraParameters += _extraparameters;

        _parameters = parameters;
    }
    else {
        parameters = boost::dynamic_pointer_cast<TrajectoryTimingParameters>(_parameters);
    }

    // Check if we need to call InitPlan
    bool bInitPlan = false;
    if( bInitPlan || CompareRealVectors(parameters->_vConfigVelocityLimit, maxvelocities, g_fEpsilonLinear) != 0 ) {
        parameters->_vConfigVelocityLimit = maxvelocities;
        bInitPlan = true;
    }
    if( bInitPlan || CompareRealVectors(parameters->_vConfigAccelerationLimit, maxaccelerations, g_fEpsilonLinear) != 0 ) {
        parameters->_vConfigAccelerationLimit = maxaccelerations;
        bInitPlan = true;
    }
    if( bInitPlan || CompareRealVectors(parameters->_vConfigJerkLimit, maxjerks, g_fEpsilonLinear) != 0 ) {
        parameters->_vConfigJerkLimit = maxjerks;
        bInitPlan = true;
    }
    if( bInitPlan || parameters->_configurationspecification != trajspec ) {
        parameters->_configurationspecification = trajspec;
        bInitPlan = true;
    }
    if( bInitPlan || (int)parameters->_vConfigLowerLimit.size() != trajdof ) {
        parameters->_vConfigLowerLimit.resize(trajdof);
        parameters->_vConfigUpperLimit.resize(trajdof);
        parameters->_vConfigResolution.resize(trajdof);
        std::fill(parameters->_vConfigLowerLimit.begin(), parameters->_vConfigLowerLimit.end(), -1e6);
        std::fill(parameters->_vConfigUpperLimit.begin(), parameters->_vConfigUpperLimit.end(), 1e6);
        std::fill(parameters->_vConfigResolution.begin(), parameters->_vConfigResolution.end(), 0.01);
        bInitPlan = true;
    }

    std::vector<int> vRotAxes;
    // Analyze the configuration to identify dimensions
    FOREACHC(itgroup, trajspec._vgroups) {
        if( itgroup->name.size() >= 16 && itgroup->name.substr(0, 16) == "affine_transform" ) {
            std::string tempname;
            int affinedofs = 0;
            std::stringstream ss(itgroup->name.substr(16));
            ss >> tempname >> affinedofs;
            BOOST_ASSERT( !!ss );
            KinBodyPtr pbody = env->GetKinBody(tempname);
            if( !!pbody ) {
                Vector vAxis(0, 0, 1);
                if( affinedofs & DOF_RotationAxis ) {
                    vRotAxes.push_back(itgroup->offset + RaveGetIndexFromAffineDOF(affinedofs, DOF_RotationAxis));
                    ss >> vAxis.x >> vAxis.y >> vAxis.z;
                }
            }
        }
        else if( itgroup->name.size() >= 14 && itgroup->name.substr(0, 14) == "ikparam_values" ) {
            int ikTypeInt = 0;
            std::stringstream ss(itgroup->name.substr(14));
            ss >> ikTypeInt;
            if( !!ss ) {
                IkParameterizationType ikType = static_cast<IkParameterizationType>(ikTypeInt);
                switch( ikType ) {
                case IKP_TranslationXYOrientation3D: {
                    vRotAxes.push_back(itgroup->offset + 2);
                    break;
                }
                default:
                    break;
                }
            }
        }
    } // end FOREACHC itgroup

    if( bInitPlan ) {
        parameters->_diffstatefn = boost::bind(diffstatefn, _1, _2, vRotAxes);
    }

    if( parameters->_hastimestamps != hastimestamps ) {
        parameters->_hastimestamps = hastimestamps;
        bInitPlan = true;
    }

    if( !_planner ) {
        _planner = RaveCreatePlanner(env, _plannername);
        bInitPlan = true;
    }

    if( bInitPlan ) {
        if( !_planner->InitPlan(RobotBasePtr(), parameters).HasSolution() ) {
            std::stringstream ssdebug; ssdebug << trajspec;
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, failed to initialize planner %s with affine trajectory spec: %s"), env->GetNameId()%_plannername%ssdebug.str(), ORE_InvalidArguments);
        }
    }

    return _planner->PlanPath(traj, planningoptions);
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
    return _PlanActiveDOFTrajectory(traj, robot, hastimestamps, fmaxvelmult, fmaxaccelmult, GetPlannerFromInterpolation(traj, plannername), /*bsmooth*/ false, plannerparameters);
}

PlannerStatus RetimeRobotDOFTrajectory(TrajectoryBasePtr traj, RobotBasePtr robot, const std::vector<int>& vRobotDOFIndices, bool hastimestamps, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, const std::string& plannerparameters)
{
    RobotBase::RobotStateSaver activesaver(robot, RobotBase::Save_ActiveDOF);
    robot->SetActiveDOFs(vRobotDOFIndices);
    return _PlanActiveDOFTrajectory(traj, robot, hastimestamps, fmaxvelmult, fmaxaccelmult, GetPlannerFromInterpolation(traj, plannername), /*bsmooth*/ false, plannerparameters);
}

PlannerStatus RetimeAffineTrajectory(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, bool hastimestamps, const std::string& plannername, const std::string& plannerparameters)
{
    return _PlanAffineTrajectory(traj, maxvelocities, maxaccelerations, hastimestamps, GetPlannerFromInterpolation(traj, plannername), /*bsmooth*/ false, plannerparameters);
}

PlannerStatus RetimeTrajectory(TrajectoryBasePtr traj, bool hastimestamps, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, const std::string& plannerparameters)
{
    return _PlanTrajectory(traj, hastimestamps, fmaxvelmult, fmaxaccelmult, GetPlannerFromInterpolation(traj, plannername), /*bsmooth*/ false, plannerparameters);
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

size_t InsertActiveDOFWaypointWithRetiming(int waypointindex, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, RobotBasePtr robot, dReal fmaxvelmult, dReal fmaxaccelmult, const std::string& plannername, const std::string& plannerparameters)
{
    BOOST_ASSERT((int)dofvalues.size()==robot->GetActiveDOF());
    BOOST_ASSERT(traj->GetEnv()==robot->GetEnv());
    vector<dReal> v1pos(robot->GetActiveDOF(),0), v1vel(robot->GetActiveDOF(),0);
    ConfigurationSpecification newspec = robot->GetActiveConfigurationSpecification();

    ConfigurationSpecification inputtrajspec = traj->GetConfigurationSpecification();

    std::string interpolation = "";
    FOREACH(it,newspec._vgroups) {
        std::vector<ConfigurationSpecification::Group>::const_iterator itgroup = inputtrajspec.FindCompatibleGroup(*it, false);
        if( itgroup == inputtrajspec._vgroups.end() ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("could not find group %s in trajectory"),newspec._vgroups.at(0).name,ORE_InvalidArguments);
        }
        if( itgroup->interpolation.size() > 0 ) {
            it->interpolation = itgroup->interpolation;
            interpolation = itgroup->interpolation;
        }
    }
    newspec.AddDerivativeGroups(1,false);

    std::vector<ConfigurationSpecification::Group>::const_iterator itaccelerationsgroup = inputtrajspec.FindCompatibleGroup("joint_accelerations", false);
    const bool hasaccelerations = itaccelerationsgroup != inputtrajspec._vgroups.end();
    if( hasaccelerations ) {
        newspec.AddDerivativeGroups(2, /*adddeltatime*/ false);
    }
    // TODO: For now suppose that the acceleration of the new point is zero

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
        else if( interpolation == "cubic" ) {
            newplannername = "cubictrajectoryretimer2";
        }
        else if( interpolation == "quintic" ) {
            newplannername = "quintictrajectoryretimer";
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_("currently do not support retiming for %s interpolations"),interpolation,ORE_InvalidArguments);
        }
    }

    if( IS_DEBUGLEVEL(Level_Verbose) ) {
        int ran = RaveRandomInt()%10000;
        string filename = str(boost::format("%s/beforeretime-%d.xml")%RaveGetHomeDirectory()%ran);
        RAVELOG_VERBOSE_FORMAT("env=%d, Writing before retime traj to %s", robot->GetEnv()->GetId()%filename);
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        trajinitial->serialize(f);
    }

    // This ensures that the beginning and final velocities will be preserved
    //RAVELOG_VERBOSE_FORMAT("env=%d, inserting point into %d with planner %s and parameters %s", robot->GetEnv()->GetId()%waypointindex%newplannername%plannerparameters);
    // make sure velocities are set
    if( !(RetimeActiveDOFTrajectory(trajinitial, robot, /*hastimestamps*/ false, fmaxvelmult, fmaxaccelmult, newplannername, plannerparameters + boost::str(boost::format("<hasvelocities>1</hasvelocities><hasaccelerations>%d</hasaccelerations>")%hasaccelerations)).GetStatusCode() & PS_HasSolution) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("env=%d, failed to retime init traj", robot->GetEnv()->GetId(), ORE_Assert);
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
    if( !(planner->PlanPath(trajinitial).GetStatusCode() & PS_HasSolution) ) {
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

    PlannerStatus statusFromInit = planner->InitPlan(RobotBasePtr(),params);
    if( !(statusFromInit.GetStatusCode() & PS_HasSolution) ) {
        rapidjson::Document rStatus(rapidjson::kObjectType);
        statusFromInit.SaveToJson(rStatus, rStatus.GetAllocator());
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to init planner %s:%s"), plannername%(orjson::DumpJson(rStatus)), ORE_InvalidArguments);
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

    EnvironmentLock lockenv(traj->GetEnv()->GetMutex());

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

        if( planner->PlanPath(ptesttraj).GetStatusCode() & PS_HasSolution ) {
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
    if( starttime < 0 ) {
        RAVELOG_WARN_FORMAT("got an invalid start time %.15e", starttime);
        starttime = 0;
    }
    if( starttime > traj->GetDuration()+g_fEpsilonLinear ) {
        RAVELOG_WARN_FORMAT("got an invalid start time %.15e", starttime);
        starttime = traj->GetDuration();
    }
    if( endtime > traj->GetDuration()+g_fEpsilonLinear ) {
        RAVELOG_WARN_FORMAT("got an invalid end time %.15e", endtime);
        endtime = traj->GetDuration();
    }
    if( endtime < starttime ) {
        RAVELOG_WARN_FORMAT("got an invalid time range %.15e, %.15e, choosing the start time", starttime%endtime);
        endtime = starttime;
    }

    int nOriginalNumWaypoints = traj->GetNumWaypoints();

    std::vector<dReal> values;
    if( endtime < traj->GetDuration() ) {
        size_t endindex = traj->GetFirstWaypointIndexAfterTime(endtime);
        if( endindex < traj->GetNumWaypoints() ) {
            traj->Sample(values, endtime);
            traj->Insert(endindex, values, true);
            traj->Remove(endindex+1, traj->GetNumWaypoints());
        }
    }
    // TODO there might be a problem here if the first traj point has deltatime > 0
    if( starttime > 0 ) {
        size_t startindex = traj->GetFirstWaypointIndexAfterTime(starttime);
        if( startindex >= traj->GetNumWaypoints() ) {
            startindex = traj->GetNumWaypoints()-1;
        }

        ConfigurationSpecification deltatimespec;
        deltatimespec.AddDeltaTimeGroup();
        std::vector<dReal> vdeltatime;
        traj->GetWaypoint(startindex,vdeltatime,deltatimespec);
        traj->Sample(values, starttime);
        dReal fSampleDeltaTime=0;
        traj->GetConfigurationSpecification().ExtractDeltaTime(fSampleDeltaTime, values.begin());
        // check if the sampletime can be very close to an existing waypoint, in which case can ignore inserting a new point
        int endremoveindex = startindex;
        if( startindex > 0 && RaveFabs(fSampleDeltaTime-vdeltatime.at(0)) > g_fEpsilonLinear ) {
            traj->Insert(startindex-1, values, true);
            // have to write the new delta time
            vdeltatime[0] -= fSampleDeltaTime;
            traj->Insert(startindex, vdeltatime, deltatimespec, true);
            endremoveindex -= 1;
            // have to reset the delta time of the first point
            vdeltatime[0] = 0;
        }
        else {
            // take care of the case where the first trajectory point has > 0 deltatime
            vdeltatime[0] -= fSampleDeltaTime;
            if (vdeltatime[0] < 0.0) {
                vdeltatime[0] = 0;
            }
        }
        traj->Remove(0, endremoveindex);
        traj->Insert(0, vdeltatime, deltatimespec, true);
    }

    // for debugging purposes
    OPENRAVE_ASSERT_OP(RaveFabs(traj->GetDuration() - (endtime - starttime)),<=,g_fEpsilonLinear*nOriginalNumWaypoints); // normalize with respect to number of points!
}

TrajectoryBasePtr GetTrajectorySegment(TrajectoryBaseConstPtr traj, dReal starttime, dReal endtime)
{
    std::vector<dReal> values;
    const ConfigurationSpecification& spec = traj->GetConfigurationSpecification();
    TrajectoryBasePtr outtraj = RaveCreateTrajectory(traj->GetEnv(), traj->GetXMLId());
    outtraj->Init(spec);
    if( traj->GetNumWaypoints() == 0 ) {
        return outtraj;
    }

    if( starttime < 0 ) {
        RAVELOG_WARN_FORMAT("got an invalid start time %.15e", starttime);
        starttime = 0;
    }
    if( endtime > traj->GetDuration()+g_fEpsilonLinear ) {
        RAVELOG_WARN_FORMAT("got an invalid end time %.15e", endtime);
        endtime = traj->GetDuration();
    }
    if( endtime < starttime ) {
        RAVELOG_WARN_FORMAT("got an invalid time range %.15e, %.15e, choosing the start time", starttime%endtime);
        endtime = starttime;
    }

    int startindex = 0, endindex = traj->GetNumWaypoints()-1;

    if( endtime < traj->GetDuration() ) {
        endindex = traj->GetFirstWaypointIndexAfterTime(endtime);
        if( endindex >= (int)traj->GetNumWaypoints() ) {
            endindex = traj->GetNumWaypoints()-1;
        }
    }

    if( endindex == 0 ) {
        // the first point! handle this separately
        traj->GetWaypoint(endindex, values);
        spec.InsertDeltaTime(values.begin(), endtime - starttime); // the first point might have a deltatime, so the traj duration can actaully be non-zero...
        outtraj->Insert(0, values);
        return outtraj;
    }

    if( starttime > 0 ) {
        startindex = traj->GetFirstWaypointIndexAfterTime(starttime);
        if( startindex > 0 ) {
            startindex--;
        }
    }

    traj->GetWaypoints(startindex, endindex+1, values);
    outtraj->Insert(0, values);

    if( starttime > 0 ) {
        // have to overwrite the first point and
        dReal startdeltatime=0, waypointdeltatime = 0;
        if( !spec.ExtractDeltaTime(waypointdeltatime, values.begin()+spec.GetDOF()) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("could not extract deltatime, bad traj", ORE_InvalidArguments);
        }

        // have to change the first waypoint
        traj->Sample(values, starttime);
        // have to set deltatime to 0
        if( !spec.ExtractDeltaTime(startdeltatime, values.begin()) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("could not extract deltatime, bad traj", ORE_InvalidArguments);
        }

        if( waypointdeltatime - startdeltatime > g_fEpsilonLinear ) {
            // there is a delta time so set first point to time 0 and second point to waypoint-startdeltatime

            spec.InsertDeltaTime(values.begin(), 0); // first waypoint starts at 0 time
            outtraj->Insert(0,values,true);

            // the second waypoint holds the delta time between new first waypoint
            traj->GetWaypoint(startindex+1, values);
            spec.InsertDeltaTime(values.begin(), waypointdeltatime - startdeltatime);
            outtraj->Insert(1,values,true);
        }
        else {
            // time between first and second point is non-existent, so remove first point
            outtraj->Remove(0, 1);
            startindex++; // later on, we use startindex. but, if we 'Remove' the first point, corresponding 'startindex' should be incremented.
            outtraj->GetWaypoint(0, values);
            spec.InsertDeltaTime(values.begin(), 0);
            outtraj->Insert(0,values, true);
        }
    }

    if( endtime < traj->GetDuration() ) {
        // have to change the last endpoint, should sample from outtraj instead since both start and end can be within same waypoint range
        outtraj->Sample(values, endtime-starttime);
        outtraj->Insert(outtraj->GetNumWaypoints()-1,values,true); // assume last end point
    }

    // for debugging purposes
    OPENRAVE_ASSERT_OP(RaveFabs(outtraj->GetDuration() - (endtime - starttime)),<=,g_fEpsilonLinear*traj->GetNumWaypoints()); // normalize with respect to number of points!
    return outtraj;
}

TrajectoryBasePtr MergeTrajectories(const std::list<TrajectoryBaseConstPtr>& listtrajectories)
{
    // merge both deltatime and iswaypoint groups
    TrajectoryBasePtr presulttraj;
    if( listtrajectories.empty() ) {
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
    // int totaldof = 1; // for delta time
    FOREACHC(ittraj,listtrajectories) {
        const ConfigurationSpecification& trajspec = (*ittraj)->GetConfigurationSpecification();
        ConfigurationSpecification::Group gtime = trajspec.GetGroupFromName("deltatime");
        spec += trajspec;
        // totaldof += trajspec.GetDOF()-1;
        // if( trajspec.FindCompatibleGroup("iswaypoint",true) != trajspec._vgroups.end() ) {
        //     totaldof -= 1;
        // }
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
        // totaldof += 1;
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
    EnvironmentLock lockenv(pbody->GetEnv()->GetMutex());
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
                    std::vector<KinBody::JointPtr>::const_iterator pjoint = find(pbody->GetDependencyOrderedJoints().begin(),pbody->GetDependencyOrderedJoints().end(),vparentjoints[index]);
                    BOOST_ASSERT( pjoint != pbody->GetDependencyOrderedJoints().end() );
                    itdh->parentindex = pjoint - pbody->GetDependencyOrderedJoints().begin();
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

DynamicsCollisionConstraint::DynamicsCollisionConstraint(PlannerBase::PlannerParametersConstPtr parameters, const std::list<KinBodyPtr>& listCheckBodies, int filtermask) : _listCheckBodies(listCheckBodies), _filtermask(filtermask), _torquelimitmode(DC_NominalTorque), _perturbation(0.1)
{
    BOOST_ASSERT(listCheckBodies.size()>0);
    _report.reset(new CollisionReport());
    _parameters = parameters;
    if( !!parameters ) {
        _specvel = parameters->_configurationspecification.ConvertToVelocitySpecification();
        _setvelstatefn = _specvel.GetSetFn(_listCheckBodies.front()->GetEnv());
    }
}

void DynamicsCollisionConstraint::SetPlannerParameters(PlannerBase::PlannerParametersConstPtr parameters)
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

void DynamicsCollisionConstraint::SetTorqueLimitMode(DynamicsConstraintsType torquelimitmode)
{
    _torquelimitmode = torquelimitmode;
}

void DynamicsCollisionConstraint::SetPerturbation(dReal perturbation)
{
    _perturbation = perturbation;
}

int DynamicsCollisionConstraint::_SetAndCheckState(PlannerBase::PlannerParametersConstPtr params, const std::vector<dReal>& vdofvalues, const std::vector<dReal>& vdofvelocities, const std::vector<dReal>& vdofaccels, int options, ConstraintFilterReturnPtr filterreturn)
{
//    if( IS_DEBUGLEVEL(Level_Verbose) ) {
//        stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
//        ss << "checking values=[";
//            for(size_t i = 0; i < vdofvalues.size(); ++i ) {
//            if( i > 0 ) {
//                ss << "," << vdofvalues[i];
//            }
//            else {
//                ss << vdofvalues[i];
//            }
//        }
//        ss << "]";
//        RAVELOG_VERBOSE(ss.str());
//    }

    if( params->SetStateValues(vdofvalues, 0) != 0 ) {
        return CFO_StateSettingError;
    }
    if( (options & CFO_CheckTimeBasedConstraints) && !!_setvelstatefn && vdofvelocities.size() == vdofvalues.size() ) {
        (*_setvelstatefn)(vdofvelocities);
    }
    int nstateret = _CheckState(vdofvelocities, vdofaccels, options, filterreturn);
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
            nstateret = _CheckState(vdofvelocities, vdofaccels, options, filterreturn);
            if( nstateret != 0 ) {
                return nstateret;
            }
        }
    }
    return 0;
}

int DynamicsCollisionConstraint::_CheckState(const std::vector<dReal>& vdofvelocities, const std::vector<dReal>& vdofaccels, int options, ConstraintFilterReturnPtr filterreturn)
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
    if( options & CFO_CheckTimeBasedConstraints && vdofvelocities.size() > 0 && vdofaccels.size() > 0 && (_torquelimitmode != DC_Unknown && _torquelimitmode != DC_IgnoreTorque) ) {
        // check dynamics only when velocities and accelerations are given
        FOREACHC(itbody, _listCheckBodies) {
            KinBodyPtr pbody = *itbody;

            // if zero dof, do not check dynamic limits nor torque limits.
            if( pbody->GetDOF() == 0 ) {
                continue;
            }

            pbody->GetDOFValues(_vfulldofvalues);
            pbody->GetDOFVelocities(_vfulldofvelocities);
            if( pbody->GetDOFDynamicAccelerationJerkLimits(_vfulldofdynamicaccelerationlimits, _vfulldofdynamicjerklimits, _vfulldofvalues, _vfulldofvelocities) ) {
                // if dynamic limits are supported for this body, check it. for now, only check dynamic acceleration limits.

                _dofaccelerations.resize(pbody->GetDOF(),0);
                _vdofindices.resize(pbody->GetDOF());
                for(int i = 0; i < pbody->GetDOF(); ++i) {
                    _vdofindices[i] = i;
                }

                // have to extract the correct accelerations from vdofaccels use specvel and timederivative=1
                _specvel.ExtractJointValues(_dofaccelerations.begin(), vdofaccels.begin(), pbody, _vdofindices, 1);

                dReal fAccumulatedTimeBasedSurpassMult = 1.0; // accumulated square of fTimeBasedSurpassMult to suppress dynamic limits violation. square of speed mult, thus accel mult.
                bool bHasDynamicLimitsVioldatedJoint = false;
                for(int iDOF = 0; iDOF < pbody->GetDOF(); ++iDOF) {
                    // check if each dof supports the dynamic limits. even if the body supports dynamic limits, there might be unsupported dofs among full dofs. for example, the arm supports dynamic limits but gripper does not.
                    if( _vfulldofdynamicaccelerationlimits.at(iDOF) < g_fEpsilon ) { // if dynamic limits are close to zero, this dof is not supported. so, skip.
                        continue;
                    }

                    // check the dynamic acceleration limits
                    const dReal fDynamicAccLimit = _vfulldofdynamicaccelerationlimits.at(iDOF);
                    const dReal fAccelAbs = RaveFabs(_dofaccelerations.at(iDOF));
                    if( fDynamicAccLimit < fAccelAbs ) {
                        bHasDynamicLimitsVioldatedJoint = true;
                        fAccumulatedTimeBasedSurpassMult = std::min(fAccumulatedTimeBasedSurpassMult, fDynamicAccLimit / fAccelAbs); // here, we can assume that fDynamicAccLimit >= g_fEpsilon. thus, fAccelAbs > fDynamicAccLimit >= g_fEpsilon. so, no zero division

                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            std::stringstream sslog; sslog << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                            sslog << "doffullvalues=[";
                            FOREACHC(itval, _vfulldofvalues) {
                                sslog << *itval << ",";
                            }
                            sslog << "]; doffullvels=[";
                            FOREACHC(itvel, _vfulldofvelocities) {
                                sslog << *itvel << ",";
                            }
                            sslog << "]; doffullaccel=[";
                            FOREACHC(itaccel, _dofaccelerations) {
                                sslog << *itaccel << ",";
                            }
                            sslog << "]; dynacclim=[";
                            FOREACHC(itaccel, _vfulldofdynamicaccelerationlimits) {
                                sslog << *itaccel << ",";
                            }
                            sslog << "]; dynjerklim=[";
                            FOREACHC(itjerk, _vfulldofdynamicjerklimits) {
                                sslog << *itjerk << ",";
                            }
                            sslog << "]";
                            _PrintOnFailure(str(boost::format("rejected dynamic acceleration limits due to joint %s (%d): |%e| !< %e. %s")%pbody->GetJointFromDOFIndex(iDOF)->GetName()%iDOF%fAccelAbs%fDynamicAccLimit%sslog.str()));
                        }

                        // if filterreturn does not exist, we cannot return the _fTimeBasedSurpassMult. so, no need to check other joints' violation and break from the for loop.
                        if( !filterreturn ) {
                            break;
                        }
                    }
                }

                // if violated, return the accumulated results
                if( bHasDynamicLimitsVioldatedJoint ) {
                    if( !!filterreturn ) {
                        filterreturn->_fTimeBasedSurpassMult = std::min(filterreturn->_fTimeBasedSurpassMult, RaveSqrt(fAccumulatedTimeBasedSurpassMult)); // take min to respect the worst mult up to now. use sqrt to convert accmult to speedmult.
                    }
                    return CFO_CheckTimeBasedConstraints;
                }
            }
            else {
                // if no support for dynamic limits, try ComputeInverseDynamics
                _vtorquevalues.resize(0);
                FOREACHC(itjoint, pbody->GetJoints()) {
                    for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                        // TODO use the ElectricMotorActuatorInfo if present to get the real max torque depending on the speed
                        std::pair<dReal, dReal> torquelimits;
                        if( _torquelimitmode == DC_InstantaneousTorque ) {
                            torquelimits = (*itjoint)->GetInstantaneousTorqueLimits(idof);
                        }
                        else {
                            torquelimits = (*itjoint)->GetNominalTorqueLimits(idof);
                        }

                        if( torquelimits.first < torquelimits.second ) {
                            _vtorquevalues.emplace_back((*itjoint)->GetDOFIndex()+idof, torquelimits);
                        }
                    }
                }
                if( _vtorquevalues.size() > 0 ) {
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
                        const std::pair<dReal, dReal>& torquelimits = it->second;
                        dReal fcurtorque = _doftorques.at(index);
                        // TODO use the ElectricMotorActuatorInfo if present to get the real torque with friction
                        if( fcurtorque < torquelimits.first || fcurtorque > torquelimits.second ) {
                            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                std::stringstream sslog; sslog << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                                sslog << "dofvel=[";
                                FOREACHC(itvel, vdofvelocities) {
                                    sslog << *itvel << ",";
                                }
                                sslog << "]; doffullaccel=[";
                                FOREACHC(itaccel, _dofaccelerations) {
                                    sslog << *itaccel << ",";
                                }
                                sslog << "]";
                                _PrintOnFailure(str(boost::format("rejected torque due to joint %s (%d): %e !< %e !< %e. %s")%pbody->GetJointFromDOFIndex(index)->GetName()%index%torquelimits.first%fcurtorque%torquelimits.second%sslog.str()));
                            }
                            return CFO_CheckTimeBasedConstraints;
                        }
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
        PlannerBase::PlannerParametersConstPtr params = _parameters.lock();
        std::vector<dReal> vcurrentvalues;
        params->_getstatefn(vcurrentvalues);
        stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        if( _listCheckBodies.size() > 0 ) {
            ss << "env=" << _listCheckBodies.front()->GetEnv()->GetId() << ", ";
        }
        ss << prefix << "; colvalues=[";
        FOREACHC(itval, vcurrentvalues) {
            ss << *itval << ",";
        }
        ss << "]";
        RAVELOG_VERBOSE(ss.str());
    }
}

inline std::ostream& RaveSerializeTransform(std::ostream& O, const Transform& t, char delim=',')
{
    O << t.rot.x << delim << t.rot.y << delim << t.rot.z << delim << t.rot.w << delim << t.trans.x << delim << t.trans.y << delim << t.trans.z;
    return O;
}

inline std::ostream& RaveSerializeValues(std::ostream& O, const std::vector<dReal>& values, char delim=',')
{
    for(size_t i = 0; i < values.size(); ++i) {
        if( i > 0 ) {
            O << delim;
        }
        O << values[i];
    }
    return O;
}

int DynamicsCollisionConstraint::Check(const std::vector<dReal>& q0, const std::vector<dReal>& q1, const std::vector<dReal>& dq0, const std::vector<dReal>& dq1, dReal timeelapsed, IntervalType interval, int options, ConstraintFilterReturnPtr filterreturn)
{
    int maskoptions = options&_filtermask;
    int maskinterval = interval & IT_IntervalMask;
    int maskinterpolation = interval & IT_InterpolationMask;
    // Current flow:
    // if( maskinterpolation == IT_Default && velocity conditions valid ) {
    //     quadratic interpolation
    // }
    // else {
    //     if( maskinterpolation == IT_Default ) {
    //         normal linear interpolation
    //     }
    //     else {
    //         all-linear interpolation
    //     }
    // }
    int neighstateoptions = NSO_OnlyHardConstraints;
    if( options&CFO_FromPathSampling ) {
        neighstateoptions = NSO_FromPathSampling;
    }
    else if( options&CFO_FromPathShortcutting ) {
        neighstateoptions = NSO_FromPathShortcutting;
    }
    else if( options&CFO_FromTrajectorySmoother ) {
        neighstateoptions = NSO_OnlyHardConstraints|NSO_FromTrajectorySmoother; // for now, trajectory smoother uses hard constraints
    }

    // bHasRampDeviatedFromInterpolation indicates if all the checked configurations deviate from the expected interpolation connecting q0 and q1.
    //
    // In case the input segment includes terminal velocity dq0 and dq1, and timeelapsed > 0,
    // bHasRampDeviatedFromInterpolation is immediately marked false. Note that in case the initial path is
    // assumed to be linear (that is, q0 and q1 are different, dq0 and dq1 are empty, and
    // timeelapsed is zero), the configurations that we actually checked might not lie on the linear
    // segment connecting q0 and q1 due to constraints (from _neighstatefn).
    bool bHasRampDeviatedFromInterpolation = false;
    if( !!filterreturn ) {
        filterreturn->Clear();
    }
    // set the bounds based on the interval type
    PlannerBase::PlannerParametersConstPtr params = _parameters.lock();
    if( !params ) {
        RAVELOG_WARN("parameters have been destroyed!\n");
        if( !!filterreturn ) {
            filterreturn->_returncode = CFO_StateSettingError;
        }
        return CFO_StateSettingError;
    }
    BOOST_ASSERT(_listCheckBodies.size()>0);
    int start=0; // 0 if should check the first configuration, 1 if should skip the first configuration
    bool bCheckEnd=false;
    switch (maskinterval) {
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

    if( timeelapsed > 0 && dq0.size() == _vtempconfig.size() && dq1.size() == _vtempconfig.size() ) {
        // When valid velocities and timeelapsed are given, fill in accelerations.
        _vtempaccelconfig.resize(dq0.size());
        dReal itimeelapsed = 1.0/timeelapsed;

        if( maskinterpolation == IT_Default ) {
            // Quadratic interpolation. Need to make sure the positions, velocities, and timeelapsed are consistent
            //     v0 + timeelapsed*0.5*(dq0 + dq1) - v1 = 0
            for(size_t i = 0; i < _vtempaccelconfig.size(); ++i) {
                _vtempaccelconfig[i] = (dq1.at(i)-dq0.at(i))*itimeelapsed;
                if( IS_DEBUGLEVEL(Level_Verbose) || IS_DEBUGLEVEL(Level_VerifyPlans) ) {
                    dReal consistencyerror = RaveFabs(q0.at(i) + timeelapsed*0.5*(dq0.at(i)+dq1.at(i)) - q1.at(i));
                    if( RaveFabs(consistencyerror-2*PI) > g_fEpsilonQuadratic ) { // TODO, officially track circular joints
                        OPENRAVE_ASSERT_OP_FORMAT(consistencyerror,<=,g_fEpsilonQuadratic*100, "dof %d is not consistent with time elapsed", i, ORE_InvalidArguments);
                    }
                }
            }
        }
        else {
            // All-linear interpolation. Do not check consistency with the quadratic equation.
            OPENRAVE_ASSERT_OP_FORMAT(maskinterpolation, ==, IT_AllLinear, "invalid interpolationtype=0x%x", maskinterpolation, ORE_InvalidArguments);
            for(size_t i = 0; i < _vtempaccelconfig.size(); ++i) {
                _vtempaccelconfig[i] = (dq1.at(i)-dq0.at(i))*itimeelapsed;
            }
        }
    }
    else {
        // Do linear interpolation.
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

    // Compute the discretization.
    // dQ = q1 - q0 and _vtempveldelta = dq1 - dq0.
    dQ = q1;
    params->_diffstatefn(dQ,q0);
    _vtempveldelta = dq1;
    if( _vtempveldelta.size() == q1.size() ) {
        for(size_t i = 0; i < _vtempveldelta.size(); ++i) {
            _vtempveldelta.at(i) -= dq0.at(i);
        }
    }

    int numSteps = 0, nLargestStepIndex = -1;
    const std::vector<dReal>& vConfigResolution = params->_vConfigResolution;
    std::vector<dReal>::const_iterator itres = vConfigResolution.begin();
    BOOST_ASSERT((int)vConfigResolution.size()==params->GetDOF());
    int totalsteps = 0;
    // Find out which DOF takes the most steps according to their respective DOF resolutions.
    if( maskinterpolation == IT_Default && (timeelapsed > 0 && dq0.size() == _vtempconfig.size() && dq1.size() == _vtempconfig.size()) ) {
        // quadratic equation, so total travelled distance for each joint is not as simple as taking the difference between the two endpoints.
        for (int idof = 0; idof < params->GetDOF(); idof++,itres++) {
            int steps = 0;
            if( RaveFabs(_vtempaccelconfig.at(idof)) <= g_fEpsilonLinear ) {
                // not a quadratic
                if( *itres != 0 ) {
                    steps = (int)(RaveFabs(dQ[idof]) / *itres + 0.99);
                }
                else {
                    steps = (int)(RaveFabs(dQ[idof]) * 100);
                }
            }
            else {
                // 0.5*a*t**2 + v0*t + x0 = x
                dReal inflectiontime = -dq0.at(idof) / _vtempaccelconfig.at(idof);
                if( inflectiontime >= 0 && inflectiontime < timeelapsed ) {
                    // have to count double
                    dReal inflectionpoint = 0.5*dq0.at(idof)*inflectiontime;
                    dReal dist = RaveFabs(inflectionpoint) + RaveFabs(dQ.at(idof)-inflectionpoint);
                    if (*itres != 0) {
                        steps = (int)(dist / *itres + 0.99);
                    }
                    else {
                        steps = (int)(dist * 100);
                    }
                }
                else {
                    if( *itres != 0 ) {
                        steps = (int)(RaveFabs(dQ[idof]) / *itres + 0.99);
                    }
                    else {
                        steps = (int)(RaveFabs(dQ[idof]) * 100);
                    }
                }
            }

            totalsteps += steps;
            if (steps > numSteps) {
                numSteps = steps;
                nLargestStepIndex = idof;
            }
        }
    }
    else {
        for (int idof = 0; idof < params->GetDOF(); idof++,itres++) {
            int steps;
            if( *itres != 0 ) {
                steps = (int)(RaveFabs(dQ[idof]) / *itres + 0.99);
            }
            else {
                steps = (int)(RaveFabs(dQ[idof]) * 100);
            }
            totalsteps += steps;
            if (steps > numSteps) {
                numSteps = steps;
                nLargestStepIndex = idof;
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

    for (int idof = 0; idof < params->GetDOF(); idof++) {
        _vtempconfig.at(idof) = q0.at(idof);
    }
    if( dq0.size() == q0.size() ) {
        _vtempvelconfig = dq0;
    }

    if( maskinterpolation == IT_Default && (timeelapsed > 0 && dq0.size() == _vtempconfig.size() && dq1.size() == _vtempconfig.size()) ) {
        // just in case, have to set the current values to _vtempconfig since neighstatefn expects the state to be set.
        if( params->SetStateValues(_vtempconfig, 0) != 0 ) {
            if( !!filterreturn ) {
                filterreturn->_returncode = CFO_StateSettingError;
            }
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
                if( fLargestInflection < 0 || (RaveFabs(fLargestInflection)<=g_fEpsilonLinear && dQ.at(nLargestStepIndex)>0) ) {
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
        int numRepeating = 0;
        dReal fBestNewStep=0;
        bool bComputeNewStep = true; // if true, then compute fBestNewStep from fStep. Otherwise use the previous computed one
        bool bHasNewTempConfigToAdd = false; // if true, then should call _SetAndCheckState on _vtempconfig and add it to configurations
        while(istep < numSteps && prevtimestep < timeelapsed) {
            int nstateret = 0;
            if( bHasNewTempConfigToAdd ) {
                // The previous condition in the if statement above was istep >= start.
                //
                // In case bComputeNewStep is false, _vtempconfig has already been updated to a new value (bHasMoved is
                // true) but istep has not been incremented so that we keep using the same fBestNewStep in this
                // iteration, due to dqscale < 1. (Look at "!bHasMoved || (istep+1 < numSteps && numRepeating > 2) ||
                // dqscale >= 1" check)
                //
                // Since we expect to do all the checks for _vtempconfig here, we need to make sure that this
                // _SetAndCheckState is called for all unchecked configurations. With only (istep >= start) condition,
                // we can accidentally skip checking the updated _vtempconfig when istep = 0 and start = 1.
                nstateret = _SetAndCheckState(params, _vtempconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                if( !!params->_getstatefn ) {
                    params->_getstatefn(_vtempconfig);     // query again in order to get normalizations/joint limits
                }
                if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                    filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
                    filterreturn->_configurationtimes.push_back(timestep);
                }
                bHasNewTempConfigToAdd = false;
            }
            if( nstateret != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = nstateret;
                }
                return nstateret;
            }

            dReal fMinNextTimeStep = prevtimestep; // when computing the new timestep, have to make sure it is greater than this value
            if( bComputeNewStep ) {
                if( prevtimestep >= fLargestInflectionTime ) {
                    fBestNewStep = fStep-fLargestStepDelta;
                }
                else {
                    // could be straddling the inflection so have to compensate. note that this will skip checking collision at the inflection
                    if( (fLargestStepDelta > 0 && fStep+fLargestStepDelta > fLargestInflection) || (fLargestStepDelta < 0 && fStep+fLargestStepDelta < fLargestInflection) ) {
                        fBestNewStep = fLargestInflection - (fStep+fLargestStepDelta - fLargestInflection);
                        fMinNextTimeStep = fLargestInflectionTime-1e-7; // in order to force to choose a time after the inflection
                    }
                    else {
                        fBestNewStep = fStep+fLargestStepDelta;
                    }
                }
            }

            if( RaveFabs(fLargestStepAccel) <= g_fEpsilonLinear ) {
                OPENRAVE_ASSERT_OP_FORMAT(RaveFabs(fLargestStepInitialVelocity),>,g_fEpsilon, "axis %d does not move? %.15e->%.15e, numSteps=%d", nLargestStepIndex%q0[nLargestStepIndex]%q1[nLargestStepIndex]%numSteps, ORE_Assert);
                timestep = fBestNewStep/fLargestStepInitialVelocity;
                RAVELOG_VERBOSE_FORMAT("largest accel is 0 so timestep=%fs", timestep);
            }
            else {
                // check whether to go positive or negative direction
                dReal fNewStep = fBestNewStep;
                bool bfound = false;

                int numroots = mathextra::solvequad(fLargestStepAccel*0.5, fLargestStepInitialVelocity, -fNewStep, timesteproots[0], timesteproots[1]);
                if( numroots == 0 ) {
                    if( RaveFabs(fNewStep-fLargestStep) < 1e-7 ) { // in order to avoid solvequat not returning any solutions
                        timestep = timeelapsed;
                        bfound = true;
                    }
                }
                if( !bfound ) {
                    for( int iroot = 0; iroot < numroots; ++iroot) {
                        if( timesteproots[iroot] > fMinNextTimeStep && (!bfound || timestep > timesteproots[iroot]) ) {
                            timestep = timesteproots[iroot];
                            bfound = true;
                        }
                    }

                    if( !bfound && (fMinNextTimeStep < fLargestInflectionTime) ) {
                        // most likely there are two solutions so solve for fStep and get the furthest solution
                        fNewStep = fStep; //-fLargestStepDelta;
                        numroots = mathextra::solvequad(fLargestStepAccel*0.5, fLargestStepInitialVelocity, -fNewStep, timesteproots[0], timesteproots[1]);
                        for( int iroot = 0; iroot < numroots; ++iroot) {
                            if( timesteproots[iroot] > fMinNextTimeStep && (!bfound || timestep > timesteproots[iroot]) ) {
                                // going backwards!
                                timestep = timesteproots[iroot];
                                fBestNewStep = fNewStep;
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

            _vprevtempconfig = _vtempconfig;
            _vprevtempvelconfig = _vtempvelconfig;

            dReal dqscale = 1.0; // time step multiple. < 1 means that robot moves a lot more than the defined resolutions and therefore needs to be more finely sampled.
            int iScaledIndex = -1; // index into dQ of the DOF that has changed the most and affected dqscale
            for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                dQ[idof] = q0.at(idof) + timestep * (dq0.at(idof) + timestep * 0.5 * _vtempaccelconfig.at(idof)) - _vtempconfig.at(idof);
                if( RaveFabs(dQ[idof]) > vConfigResolution[idof]*1.01 ) { // have to multiply by small mult since quadratic sampling doesn't guarantee exactly...
                    // have to solve for the earliest timestep such that q0.at(i) + t * (dq0.at(i) + t * 0.5 * _vtempaccelconfig.at(i)) - _vtempconfig.at(i) = vConfigResolution where t >= prevtimestep
                    // 0.5*_vtempaccelconfig.at(i)*t*t + dq0.at(i) * t + q0.at(i) - _vtempconfig.at(i) - vConfigResolution[i] = 0
                    int numroots = mathextra::solvequad(_vtempaccelconfig[idof]*0.5, dq0[idof],q0[idof] - _vtempconfig[idof] - vConfigResolution[idof], timesteproots[0], timesteproots[1]);
                    dReal scaledtimestep = 0;
                    bool bfoundscaled = false;
                    for(int iroot = 0; iroot < numroots; ++iroot) {
                        if( timesteproots[iroot] >= fMinNextTimeStep && timesteproots[iroot] <= timestep ) {
                            if( !bfoundscaled || timesteproots[iroot] < scaledtimestep ) {
                                scaledtimestep = timesteproots[iroot];
                            }
                            bfoundscaled = true;
                        }
                    }

//                    if( nLargestStepIndex == (int)i ) {
//                        // common when having jacobian constraints
//                        RAVELOG_VERBOSE_FORMAT("got huge delta abs(%f) > %f for dof %d even though it is the largest index!", dQ[i]%vConfigResolution[i]%i);
//                    }

                    if( bfoundscaled && timestep > prevtimestep ) {
                        dReal s = (scaledtimestep-prevtimestep)/(timestep-prevtimestep);
                        if( s < dqscale ) {
                            dqscale = s;
                            iScaledIndex = idof;
                        }
                    }
                    else {
                        // the delta distance is greater than expected, so have to divide the time!
                        dReal s = RaveFabs(vConfigResolution[idof]/dQ[idof]);
                        if( s < dqscale ) {
                            dqscale = s;
                            iScaledIndex = idof;
                        }
                    }
                }
            }

            if( dqscale < 1 ) {
                numRepeating++;
                if( dqscale <= 0.01 ) {
                    stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                    ss << "x0=[";
                    RaveSerializeValues(ss, q0);
                    ss << "]; x1=[";
                    RaveSerializeValues(ss, q1);
                    ss << "]; dx0=[";
                    RaveSerializeValues(ss, dq0);
                    ss << "]; dx1=[";
                    RaveSerializeValues(ss, dq1);
                    ss << "]; deltatime=" << timeelapsed;
                    RAVELOG_WARN_FORMAT("got very small dqscale %f, so returning failure %s", dqscale%ss.str());
                    if( !!filterreturn ) {
                        filterreturn->_returncode = CFO_StateSettingError;
                    }
                    return CFO_StateSettingError;
                }
                if( numRepeating > numSteps*2 ) {
                    stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                    ss << "x0=[";
                    RaveSerializeValues(ss, q0);
                    ss << "]; x1=[";
                    RaveSerializeValues(ss, q1);
                    ss << "]; dx0=[";
                    RaveSerializeValues(ss, dq0);
                    ss << "]; dx1=[";
                    RaveSerializeValues(ss, dq1);
                    ss << "]; deltatime=" << timeelapsed;
                    RAVELOG_WARN_FORMAT("num repeating is %d/%d, dqscale=%f, iScaledIndex=%d, nLargestStepIndex=%d, timestep=%f, fBestNewStep=%f, fLargestStep=%f, so returning failure %s", numRepeating%(numSteps*2)%dqscale%iScaledIndex%nLargestStepIndex%timestep%fBestNewStep%fLargestStep%ss.str());
                    if( !!filterreturn ) {
                        filterreturn->_returncode = CFO_StateSettingError;
                    }
                    return CFO_StateSettingError;
                }
                // scaled! so have to change dQ and make sure not to increment istep/fStep
                timestep = prevtimestep + (timestep-prevtimestep)*dqscale;
                for( int idof = 0; idof < (int)dQ.size(); ++idof) {
                    // have to recompute a new dQ based on the new timestep
                    dQ[idof] = q0.at(idof) + timestep * (dq0.at(idof) + timestep * 0.5 * _vtempaccelconfig.at(idof)) - _vtempconfig.at(idof);

                    // shouldn't check here since it is before the _neighstatefn, which could also change dQ
//                    if( RaveFabs(dQ[i]) > vConfigResolution[i]*1.01 ) { // have to multiply by small mult since quadratic sampling doesn't guarantee exactly...
//                        RAVELOG_WARN_FORMAT("new dQ[%d]/%.15e = %.15e", i%vConfigResolution[i]%(dQ[i]/vConfigResolution[i]));
//                    }
                    //dQ[i] *= dqscale;
                    _vtempvelconfig.at(idof) = dq0.at(idof) + timestep*_vtempaccelconfig.at(idof);
                }
                RAVELOG_VERBOSE_FORMAT("scaled by dqscale=%f, iScaledIndex=%d, timestep=%f, value=%f, dq=%f", dqscale%iScaledIndex%timestep%_vtempconfig.at(iScaledIndex)%dQ.at(iScaledIndex));
            }
            else {
                for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                    _vtempvelconfig.at(idof) = dq0.at(idof) + timestep*_vtempaccelconfig.at(idof);
                }
                numRepeating = 0;
                // only check time scale if at the current point
                if( timestep > timeelapsed+1e-7 ) {
                    if( istep+1 >= numSteps ) {
                        // expected...
                        break;
                    }

                    {
                        stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                        ss << "x0=[";
                        RaveSerializeValues(ss, q0);
                        ss << "]; x1=[";
                        RaveSerializeValues(ss, q1);
                        ss << "]; dx0=[";
                        RaveSerializeValues(ss, dq0);
                        ss << "]; dx1=[";
                        RaveSerializeValues(ss, dq1);
                        ss << "]; deltatime=" << timeelapsed;
                        RAVELOG_WARN_FORMAT("timestep %.15e > total time of ramp %.15e, step %d/%d %s", timestep%timeelapsed%istep%numSteps%ss.str());
                    }

                    if( !!filterreturn ) {
                        filterreturn->_returncode = CFO_StateSettingError;
                    }
                    return CFO_StateSettingError;
                }
                else if( timestep > timeelapsed ) {
                    timestep = timeelapsed; // get rid of small epsilons
                }
            }

            int neighstatus = params->_neighstatefn(_vtempconfig, dQ, neighstateoptions);
            if( neighstatus == NSS_Failed ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            if( neighstatus == NSS_SuccessfulWithDeviation ) {
                bHasRampDeviatedFromInterpolation = true;
            }
            bHasNewTempConfigToAdd = true;

            bool bHasMoved = false; // true if _vtempconfig is different from the previous tempconfig  _vprevtempconfig) by a significant amount
            {
                // the neighbor function could be a constraint function and might move _vtempconfig by more than the specified dQ! so double check the straight light distance between them justin case?
                // TODO check if acceleration limits are satisfied between _vtempconfig, _vprevtempconfig, and _vprevtempvelconfig
                int numPostNeighSteps = 1;
                for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                    dReal f = RaveFabs(_vtempconfig[idof] - _vprevtempconfig[idof]);
                    if( f > vConfigResolution[idof]*1.01 ) {
                        int poststeps = int(f/vConfigResolution[idof] + 0.9999);
                        if( poststeps > numPostNeighSteps ) {
                            numPostNeighSteps = poststeps;
                        }
                    }
                    if( f > 0.0001 ) {
                        bHasMoved = true;
                    }
                }

                if( numPostNeighSteps > 1 ) {
                    RAVELOG_VERBOSE_FORMAT("have to divide the arc in %d steps post neigh, timestep=%f", numPostNeighSteps%timestep);
                    // this case should be rare, so can create a vector here. don't look at constraints since we would never converge...
                    // note that circular constraints would break here
                    std::vector<dReal> vpostdq(_vtempconfig.size()), vpostddq(_vtempconfig.size());
                    dReal fiNumPostNeighSteps = 1/(dReal)numPostNeighSteps;
                    for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                        vpostdq[idof] = (_vtempconfig[idof] - _vprevtempconfig[idof]) * fiNumPostNeighSteps;
                        vpostddq[idof] = (_vtempvelconfig[idof] - _vprevtempvelconfig[idof]) * fiNumPostNeighSteps;
                    }

                    // do only numPostNeighSteps-1 since the last step should be checked by _vtempconfig
                    for(int ipoststep = 0; ipoststep+1 < numPostNeighSteps; ++ipoststep) {
                        for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                            _vprevtempconfig[idof] += vpostdq[idof];
                            _vprevtempvelconfig[idof] += vpostddq[idof]; // probably not right with the way interpolation works out, but it is a reasonable approximation
                        }

                        nstateret = _SetAndCheckState(params, _vprevtempconfig, _vprevtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
//                        if( !!params->_getstatefn ) {
//                            params->_getstatefn(_vprevtempconfig);     // query again in order to get normalizations/joint limits
//                        }
                        // since the timeelapsed is not clear, it is dangerous to write filterreturn->_configurations and filterreturn->_configurationtimes since it could force programing using those times to accelerate too fast. so don't write
//                        if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
//                            filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
//                            filterreturn->_configurationtimes.push_back(timestep);
//                        }
                        if( nstateret != 0 ) {
                            if( !!filterreturn ) {
                                filterreturn->_returncode = nstateret;
                            }
                            return nstateret;
                        }
                    }
                }
            }

            //RAVELOG_VERBOSE_FORMAT("dqscale=%f fStep=%.15e, fLargestStep=%.15e, timestep=%.15e", dqscale%fBestNewStep%fLargestStep%timestep);
            if( !bHasMoved || (istep+1 < numSteps && numRepeating > 2) || dqscale >= 1 ) {//dqscale >= 1 ) {
                // scaled! so have to change dQ and make sure not to increment istep/fStep
                fStep = fBestNewStep;
                bComputeNewStep = true;
                ++istep;
            }
            else { // bHasMoved && (istep+1 >= numSteps || numRepeating <= 2) && dqscale < 1
                bComputeNewStep = false;
            }
            prevtimestep = timestep; // have to always update since it serves as the basis for the next timestep chosen
        }
        if( RaveFabs(fStep-fLargestStep) > RaveFabs(fLargestStepDelta) ) {
            RAVELOG_WARN_FORMAT("fStep (%.15e) did not reach fLargestStep (%.15e). %.15e > %.15e", fStep%fLargestStep%RaveFabs(fStep-fLargestStep)%fLargestStepDelta);
            if( !!filterreturn ) {
                filterreturn->_returncode = CFO_StateSettingError;
            }
            // this is a bug, just return false
            return CFO_StateSettingError;
        }

        {
            // the neighbor function could be a constraint function and might move _vtempconfig by more than the specified dQ! so double check the straight light distance between them justin case?
            // TODO check if acceleration limits are satisfied between _vtempconfig, _vprevtempconfig, and _vprevtempvelconfig
            int numPostNeighSteps = 1;
            for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                dReal f = RaveFabs(q1[idof] - _vtempconfig[idof]);
                if( f > vConfigResolution[idof]*1.01 ) {
                    int poststeps = int(f/vConfigResolution[idof] + 0.9999);
                    if( poststeps > numPostNeighSteps ) {
                        numPostNeighSteps = poststeps;
                    }
                }
            }

            if( numPostNeighSteps > 1 ) {
                // should never happen, but just in case _neighstatefn is some non-linear constraint projection
                RAVELOG_WARN_FORMAT("have to divide the arc in %d steps even after original interpolation is done, timestep=%f", numPostNeighSteps%timestep);
                // this case should be rare, so can create a vector here. don't look at constraints since we would never converge...
                // note that circular constraints would break here
                std::vector<dReal> vpostdq(_vtempconfig.size()), vpostddq(_vtempconfig.size());
                dReal fiNumPostNeighSteps = 1/(dReal)numPostNeighSteps;
                for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                    vpostdq[idof] = (q1[idof] - _vtempconfig[idof]) * fiNumPostNeighSteps;
                    vpostddq[idof] = (dq1[idof] - _vtempvelconfig[idof]) * fiNumPostNeighSteps;
                }

                _vprevtempconfig = _vtempconfig;
                _vprevtempvelconfig = _vtempvelconfig;
                // do only numPostNeighSteps-1 since the last step should be checked by _vtempconfig
                for(int ipoststep = 0; ipoststep+1 < numPostNeighSteps; ++ipoststep) {
                    for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                        _vprevtempconfig[idof] += vpostdq[idof];
                        _vprevtempvelconfig[idof] += vpostddq[idof]; // probably not right with the way interpolation works out, but it is a reasonable approximation
                    }

                    int nstateret = _SetAndCheckState(params, _vprevtempconfig, _vprevtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
//                        if( !!params->_getstatefn ) {
//                            params->_getstatefn(_vprevtempconfig);     // query again in order to get normalizations/joint limits
//                        }
                    // since the timeelapsed is not clear, it is dangerous to write filterreturn->_configurations and filterreturn->_configurationtimes since it could force programing using those times to accelerate too fast. so don't write
//                        if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
//                            filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
//                            filterreturn->_configurationtimes.push_back(timestep);
//                        }
                    if( nstateret != 0 ) {
                        if( !!filterreturn ) {
                            filterreturn->_returncode = nstateret;
                        }
                        return nstateret;
                    }
                }
            }
        }

        if( bHasNewTempConfigToAdd ) {
            // At this point, _vtempconfig is expected to have reached q1. But if not, then need to check _vtempconfig.
            const dReal dist = params->_distmetricfn(_vtempconfig, q1);
            if( dist > 1e-7 ) {
                bCheckEnd = false; // _vtempconfig ends up far from q1 and we haven't checked the segment connecting _vtempconfig and q1 so prevent adding q1 to the list of checked configuration to tell the caller that the checked segment ends here.
                int nstateret = 0;
                nstateret = _SetAndCheckState(params, _vtempconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                bHasNewTempConfigToAdd = false;
                if( !!params->_getstatefn ) {
                    params->_getstatefn(_vtempconfig);     // query again in order to get normalizations/joint limits
                }
                if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                    filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
                    filterreturn->_configurationtimes.push_back(timestep);
                }
                if( nstateret != 0 ) {
                    if( !!filterreturn ) {
                        filterreturn->_returncode = nstateret;
                        filterreturn->_invalidvalues = _vtempconfig;
                        filterreturn->_invalidvelocities = _vtempvelconfig;
                        filterreturn->_fTimeWhenInvalid = timestep;
                    }
                    return nstateret;
                }
            }
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

        const bool validVelocities = (timeelapsed > 0) && (dq0.size() == _vtempconfig.size()) && (dq1.size() == _vtempconfig.size());

        _vdiffconfig.resize(dQ.size());
        _vstepconfig.resize(dQ.size());
        _vtempconfig2 = _vtempconfig; // keep record of _vtempconfig before being modified in _neighstatefn
        if( start > 0 ) {
            // just in case, have to set the current values to _vtempconfig since neighstatefn expects the state to be set.
            if( params->SetStateValues(_vtempconfig, 0) != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            int neighstatus = params->_neighstatefn(_vtempconfig, dQ, neighstateoptions);
            if( neighstatus == NSS_Failed ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            else if( neighstatus == NSS_SuccessfulWithDeviation ) {
                // When non-linear constraints are enforced, _neighstatefn(q, qdelta) can return a
                // configuration qnew far from both q and q + qdelta. When qnew is further from q
                // than the specified stepsize. We ensure that the segment (q, qnew) is at least
                // collision-free.
                //
                // Although being collision-free, the configurations along the segment (q, qnew) may
                // not satisfy other constraints. Therefore, we do *not* add them to filterreturn.
                bHasRampDeviatedFromInterpolation = true;
                int maxnumsteps = 0, steps;
                itres = vConfigResolution.begin();
                for( int idof = 0; idof < params->GetDOF(); idof++, itres++ ) {
                    _vdiffconfig[idof] = _vtempconfig[idof] - _vtempconfig2[idof];
                    if( *itres != 0 ) {
                        steps = (int)(RaveFabs(_vdiffconfig[idof]) / *itres + 0.99);
                    }
                    else {
                        steps = (int)(RaveFabs(_vdiffconfig[idof]) * 100);
                    }
                    if( steps > maxnumsteps ) {
                        maxnumsteps = steps;
                    }
                }
                _vdiffvelconfig = _vtempveldelta;

                // We need to check only configurations in between _vtempconfig2 and _vtempconfig
                if( maxnumsteps > 1 ) {
                    dReal imaxnumsteps = (1.0f)/maxnumsteps;
                    for( std::vector<dReal>::iterator itdiff = _vdiffconfig.begin(); itdiff != _vdiffconfig.end(); ++itdiff ) {
                        *itdiff *= imaxnumsteps;
                    }
                    for( std::vector<dReal>::iterator itveldiff = _vdiffvelconfig.begin(); itveldiff != _vdiffvelconfig.end(); ++itveldiff ) {
                        *itveldiff *= imaxnumsteps;
                    }
                    for( int s = 1; s < maxnumsteps; s++ ) {
                        for( int idof = 0; idof < params->GetDOF(); ++idof ) {
                            _vstepconfig[idof] = _vtempconfig2[idof] + s*_vdiffconfig[idof];
                        }
                        for( size_t idof = 0; idof < _vtempvelconfig.size(); ++idof ) {
                            _vtempvelconfig.at(idof) += _vdiffvelconfig.at(idof);
                        }
                        if( s == (maxnumsteps - 1) ) {
                            break;
                        }
                        int ret = _SetAndCheckState(params, _vstepconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                        if( !!params->_getstatefn ) {
                            params->_getstatefn(_vstepconfig); // query again in order to get normalizations/joint limits
                        }
                        // if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                        //  filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vstepconfig.begin(), _vstepconfig.end());
                        //  filterreturn->_configurationtimes.push_back((s * imaxnumsteps)*fisteps);
                        // }
                        if( ret != 0 ) {
                            if( !!filterreturn ) {
                                filterreturn->_returncode = ret;
                                filterreturn->_invalidvalues = _vstepconfig;
                                filterreturn->_invalidvelocities = _vtempvelconfig;
                                filterreturn->_fTimeWhenInvalid = (s * imaxnumsteps)*fisteps;
                            }
                            return ret;
                        }
                    } // end checking configurations between _vtempconfig2 (the previous _vtempconfig) and _vtempconfig (the new one)
                } // end if maxnumsteps > 1
            } // end check neighstatus
            if( validVelocities ) {
                // Compute the next velocity
                for( size_t idof = 0; idof < dQ.size(); ++idof ) {
                    _vtempvelconfig.at(idof) = dq0.at(idof) + _vtempveldelta.at(idof);
                }
            }
        }

        _vprevtempconfig.resize(dQ.size());
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

            // have to recompute the delta based on f and dQ
            dReal fnewscale = 1;
            for(size_t idof = 0; idof < dQ.size(); ++idof) {
                _vprevtempconfig[idof] = q0[idof] + (f+1)*dQ[idof] - _vtempconfig[idof];
                // _vprevtempconfig[idof] cannot be too high
                if( RaveFabs(_vprevtempconfig[idof]) > vConfigResolution[idof] ) {
                    dReal fscale = vConfigResolution[idof]/RaveFabs(_vprevtempconfig[idof]);
                    if( fscale < fnewscale ) {
                        fnewscale = fscale;
                    }
                }
            }
            for(size_t idof = 0; idof < dQ.size(); ++idof) {
                _vprevtempconfig[idof] *= fnewscale;
            }

            _vtempconfig2 = _vtempconfig; // keep record of the original _vtempconfig before being modified by _neighstatefn
            // Make sure that the state is set before calling _neighstatefn
            if( params->SetStateValues(_vtempconfig, 0) != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            int neighstatus = params->_neighstatefn(_vtempconfig, _vprevtempconfig, neighstateoptions);
            if( neighstatus == NSS_Failed ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            else if( neighstatus == NSS_SuccessfulWithDeviation ) {
                // When non-linear constraints are enforced, _neighstatefn(q, qdelta) can return a
                // configuration qnew far from both q and q + qdelta. When qnew is further from q than
                // the specified stepsize. We ensure that at least the segment (q, qnew) is
                // collision-free.
                //
                // Although being collision-free, the configurations along the segment (q, qnew) may not
                // satisfy other constraints. Therefore, we do *not* add them to filterreturn.
                bHasRampDeviatedFromInterpolation = true;
                int maxnumsteps = 0, steps;
                itres = vConfigResolution.begin();
                for( int idof = 0; idof < params->GetDOF(); idof++, itres++ ) {
                    _vdiffconfig[idof] = _vtempconfig[idof] - _vtempconfig2[idof];
                    if( *itres != 0 ) {
                        steps = (int)(RaveFabs(_vdiffconfig[idof]) / *itres + 0.99);
                    }
                    else {
                        steps = (int)(RaveFabs(_vdiffconfig[idof]) * 100);
                    }
                    if( steps > maxnumsteps ) {
                        maxnumsteps = steps;
                    }
                }
                _vdiffvelconfig = _vtempveldelta;
                dReal imaxnumsteps = (1.0f)/maxnumsteps;
                for( std::vector<dReal>::iterator itdiff = _vdiffconfig.begin(); itdiff != _vdiffconfig.end(); ++itdiff ) {
                    *itdiff *= imaxnumsteps;
                }
                for( std::vector<dReal>::iterator itveldiff = _vdiffvelconfig.begin(); itveldiff != _vdiffvelconfig.end(); ++itveldiff ) {
                    *itveldiff *= imaxnumsteps;
                }

                if( maxnumsteps > 1 ) {
                    // Need collision checking here but only check from the second configuration to the
                    // second-to-last configuration.
                    for( int s = 1; s < maxnumsteps; s++ ) {
                        for( int idof = 0; idof < params->GetDOF(); idof++ ) {
                            _vstepconfig[idof] = _vtempconfig2[idof] + s*_vdiffconfig[idof];
                        }
                        for( size_t idof = 0; idof < _vtempvelconfig.size(); ++idof ) {
                            _vtempvelconfig.at(idof) += _vdiffvelconfig.at(idof);
                        }
                        if( s == (maxnumsteps - 1) ) {
                            break;
                        }
                        int ret = _SetAndCheckState(params, _vstepconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                        if( !!params->_getstatefn ) {
                            params->_getstatefn(_vstepconfig); // query again in order to get normalizations/joint limits
                        }
                        // if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                        //  filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vstepconfig.begin(), _vstepconfig.end());
                        //  filterreturn->_configurationtimes.push_back((s * imaxnumsteps)*fisteps);
                        // }
                        if( ret != 0 ) {
                            if( !!filterreturn ) {
                                filterreturn->_returncode = ret;
                                filterreturn->_invalidvalues = _vstepconfig;
                                filterreturn->_invalidvelocities = _vtempvelconfig;
                                filterreturn->_fTimeWhenInvalid = (f + (s * imaxnumsteps))*fisteps;
                            }
                            return ret;
                        }
                    } // end collision checking
                } // end if maxnumsteps > 1
            } // end check neighstatus
            if( validVelocities ) {
                // Compute the next velocity
                for( size_t idof = 0; idof < dQ.size(); ++idof ) {
                    _vtempvelconfig.at(idof) = dq0.at(idof) + dReal(f + 1)*_vtempveldelta.at(idof);
                }
            }
        } // end for

        // At this point, _vtempconfig is not checked yet!
        const dReal dist = params->_distmetricfn(_vtempconfig, q1);
        if( dist > 1e-7 ) {
            // _vtempconfig is different from q1 so must check it.
            int nstateret = 0;
            nstateret = _SetAndCheckState(params, _vtempconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
            if( !!params->_getstatefn ) {
                params->_getstatefn(_vtempconfig);     // query again in order to get normalizations/joint limits
            }
            if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
                filterreturn->_configurationtimes.push_back(1.0);
            }
            if( nstateret != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = nstateret;
                    filterreturn->_invalidvalues = _vtempconfig;
                    if( validVelocities ) {
                        filterreturn->_invalidvelocities = _vtempvelconfig;
                    }
                    filterreturn->_fTimeWhenInvalid = 1.0;
                }
                return nstateret;
            }

            // the neighbor function could be a constraint function and might move _vtempconfig by more than the specified dQ! so double check the straight light distance between them justin case?
            // TODO check if acceleration limits are satisfied between _vtempconfig, _vprevtempconfig, and _vprevtempvelconfig
            int numPostNeighSteps = 1;
            for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                dReal f = RaveFabs(q1[idof] - _vtempconfig[idof]);
                if( f > vConfigResolution[idof]*1.01 ) {
                    // RAVELOG_DEBUG_FORMAT("scale ratio=%f", (f/vConfigResolution[i]));
                    int poststeps = int(f/vConfigResolution[idof] + 0.9999);
                    if( poststeps > numPostNeighSteps ) {
                        numPostNeighSteps = poststeps;
                    }
                }
            }

            if( numPostNeighSteps > 1 ) {
                bHasRampDeviatedFromInterpolation = true; // set here again just in case
                RAVELOG_WARN_FORMAT("env=%s, have to divide the arc in %d steps even after original interpolation is done, interval=%d", _listCheckBodies.front()->GetEnv()->GetNameId()%numPostNeighSteps%interval);

                // this case should be rare, so can create a vector here. don't look at constraints since we would never converge...
                // note that circular constraints would break here
                std::vector<dReal> vpostdq(_vtempconfig.size()), vpostddq(dq1.size());
                dReal fiNumPostNeighSteps = 1/(dReal)numPostNeighSteps;
                for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                    vpostdq[idof] = (q1[idof] - _vtempconfig[idof]) * fiNumPostNeighSteps;
                    if( dq1.size() == _vtempconfig.size() && _vtempvelconfig.size() == _vtempconfig.size() ) {
                        vpostddq[idof] = (dq1[idof] - _vtempvelconfig[idof]) * fiNumPostNeighSteps;
                    }
                }

                _vprevtempconfig = _vtempconfig;
                _vprevtempvelconfig = _vtempvelconfig;
                // do only numPostNeighSteps-1 since the last step should be checked by _vtempconfig
                for(int ipoststep = 0; ipoststep < numPostNeighSteps; ++ipoststep) {
                    for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                        _vprevtempconfig[idof] += vpostdq[idof];
                    }
                    if( _vprevtempconfig.size() == _vtempconfig.size() && vpostddq.size() == _vtempconfig.size() ) {
                        for( int idof = 0; idof < (int)_vtempconfig.size(); ++idof) {
                            _vprevtempvelconfig[idof] += vpostddq[idof]; // probably not right with the way interpolation works out, but it is a reasonable approximation
                        }
                    }

                    int npostneighret = _SetAndCheckState(params, _vprevtempconfig, _vprevtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                    if( npostneighret != 0 ) {
                        if( !!filterreturn ) {
                            filterreturn->_returncode = npostneighret;
                            filterreturn->_invalidvalues = _vprevtempconfig;
                            if( validVelocities ) {
                                filterreturn->_invalidvelocities = _vprevtempvelconfig;
                            }
                            filterreturn->_fTimeWhenInvalid = 1.0;
                        }
                        return npostneighret;
                    }
                } // end for ipoststep
            } // end numPostNeighSteps > 1
        } // end dist > 1e-7
    }

    if( !!filterreturn ) {
        filterreturn->_bHasRampDeviatedFromInterpolation = bHasRampDeviatedFromInterpolation;
        if( options & CFO_FillCheckedConfiguration ) {
            if( bCheckEnd ) {
                // Insert the last configuration only when we have checked it.
                filterreturn->_configurations.insert(filterreturn->_configurations.end(), q1.begin(), q1.end());
                if( timeelapsed > 0 ) {
                    filterreturn->_configurationtimes.push_back(timeelapsed);
                }
                else {
                    filterreturn->_configurationtimes.push_back(1.0);
                }
            }
        }
    }

    return 0;
}

int DynamicsCollisionConstraint::Check(const std::vector<dReal>& q0, const std::vector<dReal>& q1,
                                       const std::vector<dReal>& dq0, const std::vector<dReal>& dq1,
                                       const std::vector<dReal>& ddq0, const std::vector<dReal>& ddq1,
                                       dReal timeelapsed, IntervalType interval, int options, ConstraintFilterReturnPtr filterreturn)
{
    if( !!filterreturn ) {
        filterreturn->Clear();
    }

    //
    PlannerBase::PlannerParametersConstPtr params = _parameters.lock();
    if( !params ) {
        RAVELOG_WARN("parameters have been destroyed\n");
        if( !!filterreturn ) {
            filterreturn->_returncode = CFO_StateSettingError;
        }
        return CFO_StateSettingError;
    }

    BOOST_ASSERT(_listCheckBodies.size()>0);
    const int _environmentid = _listCheckBodies.front()->GetEnv()->GetId();
    const int maskoptions = options & _filtermask;
    const int maskinterval = interval & IT_IntervalMask;
    const int maskinterpolation = interval & IT_InterpolationMask;
    const size_t ndof = params->GetDOF();

    int neighstateoptions = NSO_OnlyHardConstraints;
    if( options & CFO_FromPathSampling ) {
        neighstateoptions = NSO_FromPathSampling;
    }
    else if( options & CFO_FromPathShortcutting ) {
        neighstateoptions = NSO_FromPathShortcutting;
    }
    else if( options & CFO_FromTrajectorySmoother ) {
        neighstateoptions = NSO_OnlyHardConstraints | NSO_FromTrajectorySmoother; // for now, trajectory smoother uses hard constraints
    }

    int start = 0;
    bool bCheckEnd= false;
    switch (maskinterval) {
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

    // Make sure that the final configuration is fine.
    if( bCheckEnd ) {
        int nstateret = _SetAndCheckState(params, q1, dq1, ddq1, maskoptions, filterreturn);
        if( nstateret != 0 ) {
            if( !!filterreturn ) {
                filterreturn->_returncode = nstateret;
                filterreturn->_invalidvalues = q1;
                filterreturn->_invalidvelocities = dq1;
                filterreturn->_invalidaccelerations = ddq1;
                filterreturn->_fTimeWhenInvalid = timeelapsed > 0 ? timeelapsed : dReal(1.0);
                if( options & CFO_FillCheckedConfiguration ) {
                    filterreturn->_configurations = q1;
                    filterreturn->_configurationtimes.push_back(timeelapsed > 0 ? timeelapsed : dReal(1.0));
                }
            }
            return nstateret;
        }
    }

    // Compute coefficients for all dofs. **Strongest term first**
    _valldofscoeffs.resize(params->GetDOF());

    dReal epsilon = g_fEpsilon; // used as a threshold for various checks to follow

    const bool bUseAllLinearInterpolation = (maskinterpolation & IT_AllLinear) == IT_AllLinear;
    OPENRAVE_ASSERT_OP(dq0.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(dq1.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(ddq0.size(), ==, ndof);
    OPENRAVE_ASSERT_OP(ddq1.size(), ==, ndof);

    // Compute polynomial coefficients
    if( !bUseAllLinearInterpolation && timeelapsed > 0 ) {
        switch( maskinterpolation ) {
        case IT_Cubic:
            epsilon = g_fEpsilonCubic;
            for( size_t idof = 0; idof < ndof; ++idof ) {
                _valldofscoeffs[idof].resize(4); // 4 coefficients for a cubic polynomial
                mathextra::computecubiccoeffs(q0.at(idof), q1.at(idof), dq0.at(idof), dq1.at(idof), ddq0.at(idof), ddq1.at(idof), timeelapsed, &_valldofscoeffs[idof][0]);

                // Since the problem is overdetermined, need to make sure that the given boundary conditions
                // (positions, velocities, and accelerations) are consistent.
                if( true ) {//( IS_DEBUGLEVEL(Level_Verbose) || IS_DEBUGLEVEL(Level_VerifyPlans) ) {
                    std::vector<dReal>& vcoeffs = _valldofscoeffs[idof];
                    dReal temp1 = 3*vcoeffs[0]*timeelapsed;
                    dReal temp2 = 2*vcoeffs[1];

                    // p'(t) = 3*a*t^2 + 2*b*t + c. Check |p'(timeelapsed) - v1|.
                    dReal velocityError = RaveFabs( ((temp1 + temp2)*timeelapsed + vcoeffs[2]) - dq1.at(idof) );
                    OPENRAVE_ASSERT_OP_FORMAT(velocityError, <=, g_fEpsilonCubic, "dof %d final velocity is not consistent", idof, ORE_InvalidArguments);

                    // p''(t) = 6*a*t + 2*b. Check |p''(timeelapsed) - a1|.
                    dReal accelerationError = RaveFabs( (2*temp1 + temp2) - ddq1.at(idof) );
                    OPENRAVE_ASSERT_OP_FORMAT(accelerationError, <=, 100*g_fEpsilonCubic, "dof %d final acceleration is not consistent", idof, ORE_InvalidArguments);
                }
            }
            break;
        case IT_Quintic:
            epsilon = g_fEpsilonQuintic;
            for( size_t idof = 0; idof < ndof; ++idof ) {
                _valldofscoeffs[idof].resize(6);
                mathextra::computequinticcoeffs(q0.at(idof), q1.at(idof), dq0.at(idof), dq1.at(idof), ddq0.at(idof), ddq1.at(idof), timeelapsed, &_valldofscoeffs[idof][0]);
            }
            // There is a unique set of coefficients for a set of boundary conditions. No further
            // consistency check is needed.
            break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("Unrecognized interpolation type %d", maskinterpolation, ORE_InvalidArguments);
        } // end switch maskinterpolation
    }
    else {
    }

    _valldofscriticalpoints.resize(params->GetDOF());
    _valldofscriticalvalues.resize(params->GetDOF());

    // Some intermediate variables
    // dQ = diff(q1, q0)
    dQ = q1;
    params->_diffstatefn(dQ, q0);
    // _vtempveldelta = dq1 - dq0 and _vtempacceldelta = ddq1 - ddq0
    _vtempveldelta = dq1;
    _vtempacceldelta = ddq1;
    for( size_t idof = 0; idof < ndof; ++idof ) {
        _vtempveldelta[idof] -= dq0[idof];
        _vtempacceldelta[idof] -= ddq0[idof];
    }

    //
    // See which DOF requires the most number of steps to go along this path. The distance that dof
    // i travels is simply sum_k |p(t_{k + 1}) - p(t_k)| where t_k is the k-th critical point. Also
    // let the first critical point be t = 0 and the last one t = timeelapsed.
    //
    int numSteps = 0; // the number of steps the dof nLargestStepIndex takes along this path
    int totalSteps = 0; // the number of steps of all DOFs combined
    const std::vector<dReal>& vConfigResolution = params->_vConfigResolution;
    std::vector<dReal>::const_iterator itres = vConfigResolution.begin();
    BOOST_ASSERT(vConfigResolution.size() == ndof);
    if( !bUseAllLinearInterpolation && timeelapsed > 0 ) {
        for( int idof = 0; idof < (int)ndof; ++idof, ++itres ) {
            dReal fabsdist = 0; // the total distance this DOF travels along this path
            int steps = 0;      // the number of steps this dof needs to travel along the path
            int numcriticalpoints = 0;
            // TODO: maybe need a better way to handle zero leading coeff
            switch( maskinterpolation ) {
            case IT_Cubic:
                _valldofscriticalpoints[idof].resize(4); // there could be no more than 2 inflection points for a cubic polynomial
                _valldofscriticalvalues[idof].resize(4); // there could be no more than 2 inflection points for a cubic polynomial
                if( _valldofscoeffs[idof][0] != 0 ) {
                    mathextra::computecubiccriticalpoints(&_valldofscoeffs[idof][0], &_valldofscriticalpoints[idof][0], numcriticalpoints);
                }
                else if( _valldofscoeffs[idof][1] != 0 ) {
                    mathextra::computequadraticcriticalpoints(&_valldofscoeffs[idof][1], &_valldofscriticalpoints[idof][0], numcriticalpoints);
                }
                break;
            case IT_Quintic:
                _valldofscriticalpoints[idof].resize(6); // there could be no more than 4 inflection points for a quintic polynomial
                _valldofscriticalvalues[idof].resize(6); // there could be no more than 4 inflection points for a quintic polynomial
                if( _valldofscoeffs[idof][0] != 0 ) {
                    mathextra::computequinticcriticalpoints(&_valldofscoeffs[idof][0], &_valldofscriticalpoints[idof][0], numcriticalpoints);
                }
                else if( _valldofscoeffs[idof][1] != 0 ) {
                    mathextra::computequarticcriticalpoints(&_valldofscoeffs[idof][1], &_valldofscriticalpoints[idof][0], numcriticalpoints);
                }
                else if( _valldofscoeffs[idof][2] != 0 ) {
                    mathextra::computecubiccriticalpoints(&_valldofscoeffs[idof][2], &_valldofscriticalpoints[idof][0], numcriticalpoints);
                }
                else if( _valldofscoeffs[idof][3] != 0 ) {
                    mathextra::computequadraticcriticalpoints(&_valldofscoeffs[idof][3], &_valldofscriticalpoints[idof][0], numcriticalpoints);
                }
                break;
            default:
                BOOST_ASSERT(0);
            } // end switch maskinterpolation

            if( numcriticalpoints == 0 ) {
                // This dof moves monotonically from q0[idof] to q1[idof]
                fabsdist = RaveFabs(dQ[idof]);

                _valldofscriticalpoints[idof].resize(2);
                _valldofscriticalpoints[idof][0] = 0;
                _valldofscriticalpoints[idof][1] = timeelapsed;

                _valldofscriticalvalues[idof].resize(2);
                _valldofscriticalvalues[idof][0] = q0[idof];
                _valldofscriticalvalues[idof][1] = q1[idof];
            }
            else {
                BOOST_ASSERT(numcriticalpoints > 0);
                // Sort the values and filter out the values that are not in the range [0, timeelapsed].
                std::sort(_valldofscriticalpoints[idof].begin(), _valldofscriticalpoints[idof].begin() + numcriticalpoints);
                size_t writeIndex = 0;
                for( size_t readIndex = 0; readIndex < (size_t)numcriticalpoints; ++readIndex ) {
                    if( _valldofscriticalpoints[idof][readIndex] >= 0 && _valldofscriticalpoints[idof][readIndex] <= timeelapsed ) {
                        if( readIndex > writeIndex ) {
                            _valldofscriticalpoints[idof][writeIndex] = _valldofscriticalpoints[idof][readIndex];
                        }

                        switch( maskinterpolation ) {
                        case IT_Cubic:
                            mathextra::evaluatecubic(&_valldofscoeffs[idof][0], _valldofscriticalpoints[idof][writeIndex], _valldofscriticalvalues[idof][writeIndex]);
                            break;
                        case IT_Quintic:
                            mathextra::evaluatequintic(&_valldofscoeffs[idof][0], _valldofscriticalpoints[idof][writeIndex], _valldofscriticalvalues[idof][writeIndex]);
                            break;
                        default:
                            BOOST_ASSERT(0);
                        } // end switch maskinterpolation

                        ++writeIndex;
                    }
                }
                if( writeIndex == 0 ) {
                    // No inflection point in [0, timeelapsed]. Just add t = 0 and t = timeelapsed to the list.
                    _valldofscriticalpoints[idof].resize(2);
                    _valldofscriticalpoints[idof][0] = 0;
                    _valldofscriticalpoints[idof][1] = timeelapsed;

                    _valldofscriticalvalues[idof].resize(2);
                    _valldofscriticalvalues[idof][0] = q0[idof];
                    _valldofscriticalvalues[idof][1] = q1[idof];
                }
                else {
                    // Need to check if t = 0 and t = timeelapsed have already been added to the list
                    if( RaveFabs(_valldofscriticalpoints[idof].front()) > epsilon ) {
                        _valldofscriticalpoints[idof].insert(_valldofscriticalpoints[idof].begin(), 0);
                        _valldofscriticalvalues[idof].insert(_valldofscriticalvalues[idof].begin(), q0[idof]);
                        writeIndex++;
                    }
                    if( RaveFabs(_valldofscriticalpoints[idof].back() - timeelapsed) > epsilon ) {
                        _valldofscriticalpoints[idof][writeIndex] = timeelapsed;
                        _valldofscriticalvalues[idof][writeIndex] = q1[idof];
                        writeIndex++;
                    }
                    _valldofscriticalpoints[idof].resize(writeIndex);
                    _valldofscriticalvalues[idof].resize(writeIndex);
                }

                // Compute accumulated distance
                dReal fprevvalue = q0[idof];
                for( size_t ipoint = 1; ipoint < _valldofscriticalpoints[idof].size(); ++ipoint ) {
                    fabsdist += RaveFabs(_valldofscriticalvalues[idof][ipoint] - fprevvalue);
                    fprevvalue = _valldofscriticalvalues[idof][ipoint];
                }
            }
            if( *itres != 0 ) {
                steps = (int)(fabsdist / *itres + 0.99);
            }
            else {
                steps = (int)(fabsdist * 100);
            }

            totalSteps += steps;
            if( steps > numSteps ) {
                numSteps = steps;
            }
        } // end for loop iterating through each dof to find discretization steps.

        // RAVELOG_VERBOSE_FORMAT("env=%d, nLargestStepIndex=%d; numSteps=%d; totalSteps=%d", _environmentid%nLargestStepIndex%numSteps%totalSteps);
    }
    else {
        for( int idof = 0; idof < (int)ndof; ++idof, ++itres) {
            int steps;
            if( *itres != 0 ) {
                steps = (int)(RaveFabs(dQ[idof]) / *itres + 0.99);
            }
            else {
                steps = (int)(RaveFabs(dQ[idof]) * 100);
            }
            totalSteps += steps;
            if (steps > numSteps) {
                numSteps = steps;
            }
        }
    }

    // If nothing moves, just return.
    if( totalSteps == 0 && start > 0 ) {
        if( !!filterreturn ) {
            if( bCheckEnd ) {
                if( options & CFO_FillCheckedConfiguration ) {
                    filterreturn->_configurations = q1;
                    filterreturn->_configurationtimes.push_back(timeelapsed > 0 ? timeelapsed : dReal(1.0));
                }
            }
        }
        return 0;
    }

    // Prepare filterreturn
    if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
        if( (int)filterreturn->_configurations.capacity() < (1 + totalSteps)*params->GetDOF() ) {
            filterreturn->_configurations.reserve((1 + totalSteps)*params->GetDOF());
        }
        if( (int)filterreturn->_configurationtimes.capacity() < 1 + totalSteps) {
            filterreturn->_configurationtimes.reserve(1 + totalSteps);
        }
    }

    // Start filling filterreturn
    if( start == 0 ) {
        int nstateret = _SetAndCheckState(params, q0, dq0, ddq0, maskoptions, filterreturn);
        if( options & CFO_FillCheckedConfiguration ) {
            filterreturn->_configurations.insert(filterreturn->_configurations.begin(), q0.begin(), q0.end());
            filterreturn->_configurationtimes.insert(filterreturn->_configurationtimes.begin(), 0);
        }
        if( nstateret != 0 ) {
            if( !!filterreturn ) {
                filterreturn->_returncode = nstateret;
                filterreturn->_invalidvalues = q0;
                filterreturn->_invalidvelocities = dq0;
                filterreturn->_invalidaccelerations = ddq0;
                filterreturn->_fTimeWhenInvalid = 0;
            }
            return nstateret;
        }
        start = 1;
    }

    // Nothing moves, so return
    if( numSteps == 0 ) {
        // everything is so small that there is no interpolation...
        if( bCheckEnd && !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
            filterreturn->_configurations.insert(filterreturn->_configurations.end(), q1.begin(), q1.end());
            filterreturn->_configurationtimes.push_back(timeelapsed);
        }
        return 0;
    }

    // Fill in _vtempconfig, _vtempvelconfig, _vtempaccelconfig
    _vtempconfig = q0;
    _vtempvelconfig = dq0;
    _vtempaccelconfig = ddq0;

    // Just in case, set the current values to _vtempconfig since _neighstatefn expects the state to be set.
    if( params->SetStateValues(_vtempconfig, 0) != 0 ) {
        if( !!filterreturn ) {
            filterreturn->_returncode = CFO_StateSettingError;
        }
        return CFO_StateSettingError;
    }

    // Checking the rest of the segment starts here
    bool bHasRampDeviatedFromInterpolation = false; // TODO: write a correct description for this variable later

    if( !bUseAllLinearInterpolation && timeelapsed > 0 ) {
        bool bComputeNewTimeStep = true; // TODO: write a correct description for this variable later
        int numRepeating = 0; // indicates how many times we get dqscale < 1 consecutively

        dReal tcur = 0;  // the current time instant we are checking
        dReal tprev = 0; // the previous time instant
        dReal tnext = 0; // the next time instant we are aiming for
        dReal fMinNextTimeStep = 0; // when computing tnext, it should be such that tnext > fMinNextTimeStep
        int istep = 0;   // the number of steps we have taken along the path.
        bool bHasNewTempConfigToAdd = false; // if true, then should call _SetAndCheckState on _vtempconfig and add it to configurations

        std::vector<size_t> vnextcriticalpointindices;
        vnextcriticalpointindices.resize(ndof, 1); // vnextcriticalpointindices[i] is the index of the next closest critical point for dof i considering that we are at the time instant tcur

        // Use totalSteps as an upperbound of the number of steps instead of using numSteps. numSteps can actually be lower than the actual number of steps needed.
        while( istep < totalSteps && tcur < timeelapsed ) {
            // Check the current state (q, qd, qdd)
            int nstateret = 0;
            if( istep >= start || bHasNewTempConfigToAdd ) {
                nstateret = _SetAndCheckState(params, _vtempconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                if( !!params->_getstatefn ) {
                    params->_getstatefn(_vtempconfig);
                }
                if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                    filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
                    filterreturn->_configurationtimes.push_back(tcur);
                }
                bHasNewTempConfigToAdd = false;
            }
            if( nstateret != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = nstateret;
                    filterreturn->_fTimeWhenInvalid = tcur;
                }
                return nstateret;
            }

            // Compute a time step tdelta such that at tcur + tdelta, no dof has moved more than its resolution.
            int earliestindex = -1;
            if( bComputeNewTimeStep ) {
                dReal tdelta = timeelapsed - tcur;
                dReal tdeltaonedof = 0;
                for( size_t idof = 0; idof < ndof; ++idof ) {
                    dReal fcurvalue = 0;

                    switch( maskinterpolation ) {
                    case IT_Cubic:
                        mathextra::evaluatecubic(&_valldofscoeffs[idof][0], tcur, fcurvalue);
                        break;
                    case IT_Quintic:
                        mathextra::evaluatequintic(&_valldofscoeffs[idof][0], tcur, fcurvalue);
                        break;
                    default:
                        BOOST_ASSERT(0);
                    } // end switch maskinterpolation

                    size_t criticalpointindex = vnextcriticalpointindices[idof];
                    dReal fcriticalvalue = _valldofscriticalvalues[idof][criticalpointindex];

                    // fdiff is how far between the current joint value and the value at the next critical point.
                    dReal fdiffvalue = fcriticalvalue - fcurvalue;
                    // if( RaveFabs(fdiffvalue) <= epsilon ) {
                    //     // This dof does not move.
                    //     continue;
                    // }

                    dReal fdistanceallowance = *(vConfigResolution.begin() + idof); // dof is allowed to move as much as its resolution
                    dReal fnextvalue = fcurvalue; // in the end, we want to solve for tdelta such that p(t + tdelta) = fnextvalue

                    // If frem > 0, this dof is reaching the next extrema before having moved for its resolution.
                    dReal frem = fdistanceallowance - RaveFabs(fdiffvalue);
                    bool bFound = true; // false if we cannot find a suitable fnextvalue
                    while( frem > 0 ) {
                        // In this case, the next critical point is nearer than fdistanceallowance
                        if( criticalpointindex + 1 == _valldofscriticalpoints[idof].size() ) {
                            // The end of the segment is reached before having moved for the dof resolution.
                            bFound = false;
                            break;
                        }

                        // Update fdistanceallowance
                        fdistanceallowance = fdistanceallowance - RaveFabs(fdiffvalue);
                        fnextvalue = _valldofscriticalvalues[idof][criticalpointindex];

                        criticalpointindex++;
                        vnextcriticalpointindices[idof]++;

                        fcriticalvalue = _valldofscriticalvalues[idof][criticalpointindex];
                        fdiffvalue = fcriticalvalue - fnextvalue;
                        frem = fdistanceallowance - RaveFabs(fdiffvalue);
                    }
                    if( !bFound ) {
                        continue;
                    }

                    if( fdiffvalue < 0 ) {
                        fnextvalue -= fdistanceallowance;
                    }
                    else {
                        fnextvalue += fdistanceallowance;
                    }

                    switch( maskinterpolation ) {
                    case IT_Cubic:
                        bFound = mathextra::computecubicnextdiscretizedstep(&_valldofscoeffs[idof][0], fnextvalue - fcurvalue, tcur, tdeltaonedof);
                        break;
                    case IT_Quintic:
                        bFound = mathextra::computequinticnextdiscretizedstep(&_valldofscoeffs[idof][0], fnextvalue - fcurvalue, tcur, tdeltaonedof);
                        break;
                    default:
                        BOOST_ASSERT(0);
                    } // end switch maskinterpolation

                    if( !bFound ) {
                        continue;
                    }
                    if( tdeltaonedof < tdelta ) {
                        tdelta = tdeltaonedof;
                        tnext = tcur + tdelta;
                        earliestindex = (int)idof;
                    }
                } // end iterating through all dofs
            } // end if( bComputeNewTimeStep )

            // RAVELOG_VERBOSE_FORMAT("env=%d, checking t=%f/%f; bComputeNewTimeStep=%d, earliestindex=%d; tprev=%f; tnext=%f; fMinNextTimeStep=%f;", _environmentid%tcur%timeelapsed%bComputeNewTimeStep%earliestindex%tprev%tnext%fMinNextTimeStep);
            if( earliestindex == -1 ) {
                // In this case, all dofs reach the end of this segment before having moved by their resolutions.
                tnext = timeelapsed;
            }

            _vprevtempconfig = _vtempconfig;
            _vprevtempvelconfig = _vtempvelconfig;
            _vprevtempaccelconfig = _vtempaccelconfig;

            // Compute dQ for use in _neighstatefn. Now we are going to move for dQ within time tnext - tprev.
            dReal fnextvalue;
            switch( maskinterpolation ) {
            case IT_Cubic:
                for( size_t idof = 0; idof < ndof; ++idof ) {
                    mathextra::evaluatecubic(&_valldofscoeffs[idof][0], tnext, fnextvalue);
                    dQ[idof] = fnextvalue - _vtempconfig[idof];
                }
                break;
            case IT_Quintic:
                for( size_t idof = 0; idof < ndof; ++idof ) {
                    mathextra::evaluatequintic(&_valldofscoeffs[idof][0], tnext, fnextvalue);
                    dQ[idof] = fnextvalue - _vtempconfig[idof];
                }
                break;
            default:
                BOOST_ASSERT(0);
            } // end switch maskinterpolation

            dReal dqscale = 1.0;  // TODO: write a correct description for this variable later
            dReal fitdiff = 1/(tnext - tprev);
            for( size_t idof = 0; idof < ndof; ++idof ) {
                if( RaveFabs(dQ[idof]) > vConfigResolution[idof] * 1.01 ) {
                    // The computed dQ[idof] exceeds the joint resolution. Find the earliest timestep t
                    // > fMinNextTimeStep such that this joint has moved exactly for its
                    // resolution. Then we scale down the time duration that we move. That is, instead
                    // of moving for dQ[idof] within time tnext - tprev, we are going to move for
                    // jointres within time (t - tprev)/(tnext - tprev).
                    dReal fExpectedValue;
                    if( dQ[idof] > 0 ) {
                        fExpectedValue = _vtempconfig[idof] + vConfigResolution[idof];
                    }
                    else {
                        fExpectedValue = _vtempconfig[idof] - vConfigResolution[idof];
                    }
                    int numroots = 0;
                    // TODO: maybe need a better way to handle zero leading coeff
                    switch( maskinterpolation ) {
                    case IT_Cubic:
                        _vrawroots.resize(3); // there are at most 3 roots
                        _vrawcoeffs = _valldofscoeffs[idof];
                        _vrawcoeffs[3] -= fExpectedValue;
                        if( _vrawcoeffs[0] != 0 ) {
                            mathextra::polyroots<dReal, 3>(&_vrawcoeffs[0], &_vrawroots[0], numroots);
                        }
                        else if( _vrawcoeffs[1] != 0 ) {
                            mathextra::polyroots<dReal, 2>(&_vrawcoeffs[1], &_vrawroots[0], numroots);
                        }
                        else if( _vrawcoeffs[2] != 0 ) {
                            mathextra::polyroots<dReal, 1>(&_vrawcoeffs[2], &_vrawroots[0], numroots);
                        }
                        break;
                    case IT_Quintic:
                        _vrawroots.resize(5); // there are at most 5 roots
                        _vrawcoeffs = _valldofscoeffs[idof];
                        _vrawcoeffs[5] -= fExpectedValue;
                        if( _vrawcoeffs[0] != 0 ) {
                            mathextra::polyroots<dReal, 5>(&_vrawcoeffs[0], &_vrawroots[0], numroots);
                        }
                        else if( _vrawcoeffs[1] != 0 ) {
                            mathextra::polyroots<dReal, 4>(&_vrawcoeffs[1], &_vrawroots[0], numroots);
                        }
                        else if( _vrawcoeffs[2] != 0 ) {
                            mathextra::polyroots<dReal, 3>(&_vrawcoeffs[2], &_vrawroots[0], numroots);
                        }
                        else if( _vrawcoeffs[3] != 0 ) {
                            mathextra::polyroots<dReal, 2>(&_vrawcoeffs[3], &_vrawroots[0], numroots);
                        }
                        else if( _vrawcoeffs[4] != 0 ) {
                            mathextra::polyroots<dReal, 1>(&_vrawcoeffs[4], &_vrawroots[0], numroots);
                        }
                        break;
                    default:
                        BOOST_ASSERT(0);
                    } // end switch maskinterpolation
                    bool bFoundTimeInstant = false;
                    dReal root = 0;
                    if( numroots > 0 ) {
                        std::sort(_vrawroots.begin(), _vrawroots.begin() + numroots);
                        for( int iroot = 0; iroot < numroots; ++iroot ) {
                            if( _vrawroots.at(iroot) > fMinNextTimeStep ) {
                                if( !bFoundTimeInstant || _vrawroots[iroot] < root ) {
                                    root = _vrawroots[iroot];
                                    bFoundTimeInstant = true;
                                }
                            }
                        }
                    }

                    dReal s;
                    if( bFoundTimeInstant ) {
                        s = (root - tprev)*fitdiff;
                    }
                    else {
                        s = RaveFabs(vConfigResolution[idof]/dQ[idof]);
                    }
                    if( s < dqscale ) {
                        dqscale = s;
                    }
                }
                else {
                    // In this case, we can go to t = tnext as no dof will have moved for more than its
                    // own resolution.
                }
            }
            // RAVELOG_VERBOSE_FORMAT("env=%d, dqscale=%f", _environmentid%dqscale);

            if( dqscale < 1 ) {
                numRepeating++;
                if( dqscale < 0.01 ) {
                }
                if( numRepeating > numSteps * 2 ) {
                }
                tcur = tprev + (tnext - tprev)*dqscale;
                // Recompute dQ based on the newly computed time instant
                dReal fNextValue;
                switch( maskinterpolation ) {
                case IT_Cubic:
                    for( size_t idof = 0; idof < ndof; ++idof ) {
                        mathextra::evaluatecubic(&_valldofscoeffs[idof][0], tcur, fNextValue);
                        dQ[idof] = fNextValue - _vtempconfig[idof];
                    }
                    break;
                case IT_Quintic:
                    for( size_t idof = 0; idof < ndof; ++idof ) {
                        mathextra::evaluatequintic(&_valldofscoeffs[idof][0], tcur, fNextValue);
                        dQ[idof] = fNextValue - _vtempconfig[idof];
                    }
                    break;
                default:
                    BOOST_ASSERT(0);
                } // end switch maskinterpolation
            }
            else {
                // dqscale >= 1
                tcur = tnext;
                numRepeating = 0; // reset
                if( tcur > timeelapsed +  1e-7 ) {
                    if( istep + 1 >= totalSteps ) {
                        break; // expected
                    }
                    RAVELOG_WARN_FORMAT("env=%d, timestep=%f; timeelapsed=%f; istep=%d; numSteps=%d; totalSteps=%d", _environmentid%tcur%timeelapsed%istep%numSteps%totalSteps);
                    if( !!filterreturn ) {
                        filterreturn->_returncode = CFO_StateSettingError;
                        filterreturn->_fTimeWhenInvalid = tcur;
                    }
                    return CFO_StateSettingError;
                }
                else if( tcur > timeelapsed ) {
                    tcur = timeelapsed;
                }
            }

            int neighstatus = params->_neighstatefn(_vtempconfig, dQ, neighstateoptions);
            if( neighstatus == NSS_Failed ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                    filterreturn->_fTimeWhenInvalid = tcur;
                }
                return CFO_StateSettingError;
            }
            if( neighstatus == NSS_SuccessfulWithDeviation ) {
                bHasRampDeviatedFromInterpolation = true;
            }
            bHasNewTempConfigToAdd = true;

            // Fill in _vtempvelconfig and _vtempaccelconfig
            switch( maskinterpolation ) {
            case IT_Cubic:
                for( size_t idof = 0; idof < ndof; ++idof ) {
                    mathextra::evaluatecubicderiv1(&_valldofscoeffs[idof][0], tcur, _vtempvelconfig[idof]);
                    mathextra::evaluatecubicderiv2(&_valldofscoeffs[idof][0], tcur, _vtempaccelconfig[idof]);
                }
                break;
            case IT_Quintic:
                for( size_t idof = 0; idof < ndof; ++idof ) {
                    mathextra::evaluatequinticderiv1(&_valldofscoeffs[idof][0], tcur, _vtempvelconfig[idof]);
                    mathextra::evaluatequinticderiv2(&_valldofscoeffs[idof][0], tcur, _vtempaccelconfig[idof]);
                }
                break;
            default:
                BOOST_ASSERT(0);
            } // end switch maskinterpolation

            // Check the config returned from _neighstatefn if it is far from the one we start with
            bool bHasMoved = false;
            {
                int numPostNeighSteps = 1; // the number of steps (in terms of joint resolutions) that a joint needs to move from _vprevtempconfig to _vtempconfig.
                for( size_t idof = 0; idof < ndof; ++idof ) {
                    dReal fabsdiffvalue = RaveFabs(_vtempconfig[idof] - _vprevtempconfig[idof]);
                    if( fabsdiffvalue > 1.01*vConfigResolution[idof] ) {
                        int postSteps = int( fabsdiffvalue/vConfigResolution[idof] + 0.9999 );
                        if( postSteps > numPostNeighSteps ) {
                            numPostNeighSteps = postSteps;
                        }
                    }
                    if( fabsdiffvalue > 0.0001 ) { // TODO: parameter for this 0.0001?
                        bHasMoved = true;
                    }
                }
                if( numPostNeighSteps > 1 ) {
                    // RAVELOG_DEBUG_FORMAT("env=%d, istep=%d; numPostNeighSteps=%d", _environmentid%istep%numPostNeighSteps);
                    // std::vector<dReal> vpostdq(ndof), vpostddq(ndof), vpostdddq(ndof); // TODO: cache this
                    // dReal fiNumPostNeighSteps = 1/(dReal)numPostNeighSteps;
                    // for( size_t idof = 0; idof < ndof; ++idof ) {
                    //     vpostdq[idof] = (_vtempconfig[idof] - _vprevtempconfig[idof]) * fiNumPostNeighSteps;
                    //     vpostddq[idof] = (_vtempvelconfig[idof] - _vprevtempvelconfig[idof]) * fiNumPostNeighSteps;
                    //     vpostdddq[idof] = (_vtempaccelconfig[idof] - _vprevtempaccelconfig[idof]) * fiNumPostNeighSteps;
                    // }

                    // // Approximate everything between _vprevtempconfig and _vtempconfig (the projected
                    // // configuration) using linear interpolation. TODO: maybe fix this later???
                    // for( int ipoststep = 0; ipoststep + 1 < numPostNeighSteps; ++ipoststep ) {
                    //     for( size_t idof = 0; idof < ndof; ++idof ) {
                    //         _vprevtempconfig[idof] += vpostdq[idof];
                    //         _vprevtempvelconfig[idof] += vpostddq[idof];
                    //     }
                    //     nstateret = _SetAndCheckState(params, _vprevtempconfig, _vprevtempvelconfig, _vprevtempaccelconfig, maskoptions, filterreturn);
                    //     if( nstateret != 0 ) {
                    //         if( !!filterreturn ) {
                    //             filterreturn->_returncode = nstateret;
                    //         }
                    //         return nstateret;
                    //     }
                    // }

                    RAVELOG_DEBUG_FORMAT("env=%d, the projected configuration is too far from the expected one. numPostNeighSteps=%d", _environmentid%numPostNeighSteps);
                    if( !!filterreturn ) {
                        filterreturn->_returncode = CFO_FinalValuesNotReached;
                        filterreturn->_fTimeWhenInvalid = tcur;
                    }
                    return CFO_FinalValuesNotReached;
                }
            }

            if( !bHasMoved || (istep + 1 < totalSteps && numRepeating > 2) || dqscale >= 1 ) {
                bComputeNewTimeStep = true;
                fMinNextTimeStep = tnext;
                ++istep;
            }
            else {
                bComputeNewTimeStep = false;
            }

            tprev = tcur;
        } // end while tcur < timeelapsed

        // Check if _vtempconfig (which is the last configuration we checked in the loop above) is close to q1
        {
            int numPostNeighSteps = 1;
            for( size_t idof = 0; idof < ndof; ++idof ) {
                dReal fabsdiffvalue = RaveFabs(q1[idof] - _vtempconfig[idof]);
                if( fabsdiffvalue > 1.01*vConfigResolution[idof] ) {
                    int postSteps = int( fabsdiffvalue/vConfigResolution[idof] + 0.9999 );
                    if( postSteps > numPostNeighSteps ) {
                        numPostNeighSteps = postSteps;
                    }
                }
            }

            if( numPostNeighSteps > 1 ) {
                // This is not very uncommon, after all, if _neighstatefn is some non-linear projection constraint.
                RAVELOG_DEBUG_FORMAT("env=%d, have to divide the arc into %d steps even after the original interpolation is done.", _environmentid%numPostNeighSteps);
                // std::vector<dReal> vpostdq(ndof), vpostddq(ndof), vpostdddq(ndof);
                // dReal fiNumPostNeighSteps = 1/(dReal)numPostNeighSteps;
                // for( size_t idof = 0; idof < ndof; ++idof ) {
                //     vpostdq[idof] = (q1[idof] - _vtempconfig[idof]) * fiNumPostNeighSteps;
                //     vpostddq[idof] = (dq1[idof] - _vtempvelconfig[idof]) * fiNumPostNeighSteps;
                //     vpostdddq[idof] = (ddq1[idof] - _vtempaccelconfig[idof]) * fiNumPostNeighSteps;
                // }

                // // Approximate everything between _vprevtempconfig and _vtempconfig (the projected
                // // configuration) using linear interpolation. TODO: maybe fix this later???
                // for( int ipoststep = 0; ipoststep + 1 < numPostNeighSteps; ++ipoststep ) {
                //     for( size_t idof = 0; idof < ndof; ++idof ) {
                //         _vprevtempconfig[idof] += vpostdq[idof];
                //         _vprevtempvelconfig[idof] += vpostddq[idof];
                //     }
                //     int nstateret = _SetAndCheckState(params, _vprevtempconfig, _vprevtempvelconfig, _vprevtempaccelconfig, maskoptions, filterreturn);
                //     if( nstateret != 0 ) {
                //         if( !!filterreturn ) {
                //             filterreturn->_returncode = nstateret;
                //         }
                //         return nstateret;
                //     }
                // }

                // May be too dangerous to allow it to pass when the final configuration does not reach q1.
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_FinalValuesNotReached;
                    filterreturn->_fTimeWhenInvalid = tcur;
                }
                return CFO_FinalValuesNotReached;
            }
        }
    }
    else {
        // Linear interpolation for positions, velocities, and accelerations
        // Compute the steps
        const dReal fisteps = dReal(1.0f)/numSteps;
        for( std::vector<dReal>::iterator it = dQ.begin(); it != dQ.end(); ++it ) {
            *it *= fisteps;
        }
        for( std::vector<dReal>::iterator it = _vtempveldelta.begin(); it != _vtempveldelta.end(); ++it ) {
            *it *= fisteps;
        }
        for( std::vector<dReal>::iterator it = _vtempacceldelta.begin(); it != _vtempacceldelta.end(); ++it ) {
            *it *= fisteps;
        }

        // Set current state to _vtempconfig since neighstatefn expects the state to be set.
        if( params->SetStateValues(_vtempconfig, 0) != 0 ) {
            if( !!filterreturn ) {
                filterreturn->_returncode = CFO_StateSettingError;
            }
            return CFO_StateSettingError;
        }

        _vtempconfig2 = _vtempconfig; // _vtempconfig2 keeps track of _vtempconfig before _vtempconfig is modified by neighstatefn
        _vdiffconfig.resize(ndof);
        _vstepconfig.resize(ndof);

        if( start > 0 ) {
            int neighstatus = params->_neighstatefn(_vtempconfig, dQ, neighstateoptions);
            if( neighstatus == NSS_Failed ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                    filterreturn->_fTimeWhenInvalid = 0;
                }
                return CFO_StateSettingError;
            }
            else if( neighstatus == NSS_SuccessfulWithDeviation ) {
                bHasRampDeviatedFromInterpolation = true;
                // Now _vtempconfig is updated but is different from _vtempconfig2 + dQ. Therefore, need to make sure
                // again that the segment (_vtempconfig2, _vtempconfig) is at least collision-free.

                int maxNumSteps = 0;
                int currentDOFSteps = 0;
                itres = vConfigResolution.begin(); // reset itres
                for( size_t idof = 0; idof < ndof; ++idof, ++itres ) {
                    _vdiffconfig[idof] = _vtempconfig[idof] - _vtempconfig2[idof];
                    if( *itres != 0 ) {
                        currentDOFSteps = (int)(RaveFabs(_vdiffconfig[idof]) / *itres + 0.99);
                    }
                    else {
                        currentDOFSteps = (int)(RaveFabs(_vdiffconfig[idof]) * 100);
                    }
                    if( currentDOFSteps > maxNumSteps ) {
                        maxNumSteps = currentDOFSteps;
                    }
                }

                if( maxNumSteps > 1 ) {
                    // The new configuration returned from neighstatefn is too far from the originally expected one.
                    const dReal fiMaxNumSteps = dReal(1.0f)/maxNumSteps;
                    _vdiffvelconfig = _vtempveldelta;
                    _vdiffaccelconfig = _vtempacceldelta;
                    for( std::vector<dReal>::iterator itdiff = _vdiffconfig.begin(); itdiff != _vdiffconfig.end(); ++itdiff ) {
                        *itdiff *= fiMaxNumSteps;
                    }
                    for( std::vector<dReal>::iterator itveldiff = _vdiffvelconfig.begin(); itveldiff != _vdiffvelconfig.end(); ++itveldiff ) {
                        *itveldiff *= fiMaxNumSteps;
                    }
                    for( std::vector<dReal>::iterator itacceldiff = _vdiffaccelconfig.begin(); itacceldiff != _vdiffaccelconfig.end(); ++itacceldiff ) {
                        *itacceldiff *= fiMaxNumSteps;
                    }
                    for( int iStep = 1; iStep < maxNumSteps; ++iStep ) {
                        // Linearly interpolate values
                        for( size_t idof = 0; idof < ndof; ++idof ) {
                            _vstepconfig[idof] = _vtempconfig2[idof] + iStep*_vdiffconfig[idof];
                            _vtempvelconfig[idof] += _vdiffvelconfig[idof];
                            _vtempaccelconfig[idof] += _vdiffaccelconfig[idof];
                        }
                        if( iStep == (maxNumSteps - 1) ) {
                            break; // break from for iStep
                        }

                        int ret = _SetAndCheckState(params, _vstepconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                        if( !!params->_getstatefn ) {
                            params->_getstatefn(_vstepconfig);
                        }
                        if( ret != 0 ) {
                            if( !!filterreturn ) {
                                filterreturn->_returncode = ret;
                                filterreturn->_invalidvalues = _vstepconfig;
                                filterreturn->_invalidvelocities = _vtempvelconfig;
                                filterreturn->_fTimeWhenInvalid = (iStep * fiMaxNumSteps) * fisteps;
                            }
                            return ret;
                        }
                    } // end checking configurations between _vtempconfig2 (original configuration before calling neighstatefn) and _vtempconfig (the configuration returned from neighstatefn).
                } // end if maxNumSteps > 1
            }
            else {
                // neighstatefn returns _vtempconfig + dQ, so no problem.
            }
        } // end if start > 0

        _vprevtempconfig.resize(ndof); // for storing the delta values for neighstatefn
        for( int iStep = start; iStep < numSteps; ++iStep ) {
            int nstateret = _SetAndCheckState(params, _vtempconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
            if( !!params->_getstatefn ) {
                params->_getstatefn(_vtempconfig);
            }
            if( !!filterreturn && (options & CFO_FillCheckedConfiguration) ) {
                filterreturn->_configurations.insert(filterreturn->_configurations.end(), _vtempconfig.begin(), _vtempconfig.end());
                filterreturn->_configurationtimes.push_back(iStep*fisteps);
            }

            if( nstateret != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = nstateret;
                    filterreturn->_invalidvalues = _vtempconfig;
                    filterreturn->_invalidvelocities = _vtempvelconfig;
                    filterreturn->_fTimeWhenInvalid = iStep * fisteps;
                }
                return nstateret;
            }

            // _vtempconfig passes the check. Now compute the next config.
            dReal fNewScale = 1.0; // the scaling factor to apply to _vprevtempconfig in case the computed _vprevtempconfig is too large.
            for( size_t idof = 0; idof < ndof; ++idof ) {
                // If there are no deviations from neighstatefn calls, then _vprevtempconfig will be exactly dQ since q0
                // + iStep*dQ = _vtempconfig.
                _vprevtempconfig[idof] = q0[idof] + (iStep + 1)*dQ[idof] - _vtempconfig[idof];
                if( RaveFabs(_vprevtempconfig[idof]) > vConfigResolution[idof] ) {
                    dReal fDOFScale = vConfigResolution[idof] / RaveFabs(_vprevtempconfig[idof]);
                    if( fDOFScale < fNewScale ) {
                        fNewScale = fDOFScale;
                    }
                }
            }
            for( size_t idof = 0; idof < ndof; ++idof ) {
                _vprevtempconfig[idof] *= fNewScale;
            }

            _vtempconfig2 = _vtempconfig; // _vtempconfig2 keeps track of _vtempconfig before _vtempconfig is modified by neighstatefn
            if( params->SetStateValues(_vtempconfig, 0) != 0 ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            int neighstatefn = params->_neighstatefn(_vtempconfig, _vprevtempconfig, neighstateoptions);
            if( neighstatefn == NSS_Failed ) {
                if( !!filterreturn ) {
                    filterreturn->_returncode = CFO_StateSettingError;
                }
                return CFO_StateSettingError;
            }
            else if( neighstatefn == NSS_SuccessfulWithDeviation ) {
                bHasRampDeviatedFromInterpolation = true;
                // Now _vtempconfig is updated but is different from _vtempconfig2 + dQ. Therefore, need to make sure
                // again that the segment (_vtempconfig2, _vtempconfig) is at least collision-free.

                int maxNumSteps = 0;
                int currentDOFSteps = 0;
                itres = vConfigResolution.begin(); // reset itres
                for( size_t idof = 0; idof < ndof; ++idof, ++itres ) {
                    _vdiffconfig[idof] = _vtempconfig[idof] - _vtempconfig2[idof];
                    if( *itres != 0 ) {
                        currentDOFSteps = (int)(RaveFabs(_vdiffconfig[idof]) / *itres + 0.99);
                    }
                    else {
                        currentDOFSteps = (int)(RaveFabs(_vdiffconfig[idof]) * 100);
                    }
                    if( currentDOFSteps > maxNumSteps ) {
                        maxNumSteps = currentDOFSteps;
                    }
                }

                if( maxNumSteps > 1 ) {
                    // The new configuration returned from neighstatefn is too far from the originally expected one.
                    const dReal fiMaxNumSteps = dReal(1.0f)/maxNumSteps;
                    _vdiffvelconfig = _vtempveldelta;
                    _vdiffaccelconfig = _vtempacceldelta;
                    for( std::vector<dReal>::iterator itdiff = _vdiffconfig.begin(); itdiff != _vdiffconfig.end(); ++itdiff ) {
                        *itdiff *= fiMaxNumSteps;
                    }
                    for( std::vector<dReal>::iterator itveldiff = _vdiffvelconfig.begin(); itveldiff != _vdiffvelconfig.end(); ++itveldiff ) {
                        *itveldiff *= fiMaxNumSteps;
                    }
                    for( std::vector<dReal>::iterator itacceldiff = _vdiffaccelconfig.begin(); itacceldiff != _vdiffaccelconfig.end(); ++itacceldiff ) {
                        *itacceldiff *= fiMaxNumSteps;
                    }
                    for( int jStep = 1; jStep < maxNumSteps; ++jStep ) {
                        // Linearly interpolate values
                        for( size_t idof = 0; idof < ndof; ++idof ) {
                            _vstepconfig[idof] = _vtempconfig2[idof] + jStep*_vdiffconfig[idof];
                            _vtempvelconfig[idof] += _vdiffvelconfig[idof];
                            _vtempaccelconfig[idof] += _vdiffaccelconfig[idof];
                        }
                        if( jStep == (maxNumSteps - 1) ) {
                            break; // break from for jStep
                        }

                        int ret = _SetAndCheckState(params, _vstepconfig, _vtempvelconfig, _vtempaccelconfig, maskoptions, filterreturn);
                        if( !!params->_getstatefn ) {
                            params->_getstatefn(_vstepconfig);
                        }
                        if( ret != 0 ) {
                            if( !!filterreturn ) {
                                filterreturn->_returncode = ret;
                                filterreturn->_invalidvalues = _vstepconfig;
                                filterreturn->_invalidvelocities = _vtempvelconfig;
                                filterreturn->_fTimeWhenInvalid = (jStep * fiMaxNumSteps) * fisteps;
                            }
                            return ret;
                        }
                    } // end checking configurations between _vtempconfig2 (original configuration before calling neighstatefn) and _vtempconfig (the configuration returned from neighstatefn).
                } // end if maxNumSteps > 1
            }
            else {
                // neighstatefn returns _vtempconfig + dQ, so no problem.
            }
        } // end for iStep

        // Check if _vtempconfig (which is the last configuration we checked in the loop above) is close to q1
        {
            int numPostNeighSteps = 1;
            for( size_t idof = 0; idof < ndof; ++idof ) {
                dReal fabsdiffvalue = RaveFabs(q1[idof] - _vtempconfig[idof]);
                if( fabsdiffvalue > 1.01*vConfigResolution[idof] ) {
                    int postSteps = int( fabsdiffvalue/vConfigResolution[idof] + 0.9999 );
                    if( postSteps > numPostNeighSteps ) {
                        numPostNeighSteps = postSteps;
                    }
                }
            }

            if( numPostNeighSteps > 1 ) {
                // This is not very uncommon, after all, if _neighstatefn is some non-linear projection constraint.
                RAVELOG_DEBUG_FORMAT("env=%d, have to divide the arc into %d steps even after the original interpolation is done.", _environmentid%numPostNeighSteps);
                // std::vector<dReal> vpostdq(ndof), vpostddq(ndof), vpostdddq(ndof);
                // dReal fiNumPostNeighSteps = 1/(dReal)numPostNeighSteps;
                // for( size_t idof = 0; idof < ndof; ++idof ) {
                //     vpostdq[idof] = (q1[idof] - _vtempconfig[idof]) * fiNumPostNeighSteps;
                //     vpostddq[idof] = (dq1[idof] - _vtempvelconfig[idof]) * fiNumPostNeighSteps;
                //     vpostdddq[idof] = (ddq1[idof] - _vtempaccelconfig[idof]) * fiNumPostNeighSteps;
                // }

                // // Approximate everything between _vprevtempconfig and _vtempconfig (the projected
                // // configuration) using linear interpolation. TODO: maybe fix this later???
                // for( int ipoststep = 0; ipoststep + 1 < numPostNeighSteps; ++ipoststep ) {
                //     for( size_t idof = 0; idof < ndof; ++idof ) {
                //         _vprevtempconfig[idof] += vpostdq[idof];
                //         _vprevtempvelconfig[idof] += vpostddq[idof];
                //     }
                //     int nstateret = _SetAndCheckState(params, _vprevtempconfig, _vprevtempvelconfig, _vprevtempaccelconfig, maskoptions, filterreturn);
                //     if( nstateret != 0 ) {
                //         if( !!filterreturn ) {
                //             filterreturn->_returncode = nstateret;
                //         }
                //         return nstateret;
                //     }
                // }

                // May be too dangerous to allow it to pass when the final configuration does not reach q1.
                return CFO_FinalValuesNotReached;
            }
        }
    }

    if( !!filterreturn ) {
        filterreturn->_bHasRampDeviatedFromInterpolation = bHasRampDeviatedFromInterpolation;
        if( options & CFO_FillCheckedConfiguration ) {
            if( bCheckEnd ) {
                filterreturn->_configurations.insert(filterreturn->_configurations.end(), q1.begin(), q1.end());
                if( timeelapsed > 0 ) {
                    filterreturn->_configurationtimes.push_back(timeelapsed);
                }
                else {
                    filterreturn->_configurationtimes.push_back(1.0);
                }
            }
        }
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

ManipulatorIKGoalSampler::ManipulatorIKGoalSampler(RobotBase::ManipulatorConstPtr pmanip, const std::list<IkParameterization>& listparameterizations, int nummaxsamples, int nummaxtries, dReal fsampleprob, bool searchfreeparameters, int ikfilteroptions, const std::vector<dReal>& freevalues) : _pmanip(pmanip), _nummaxsamples(nummaxsamples), _nummaxtries(nummaxtries), _fsampleprob(fsampleprob), _ikfilteroptions(ikfilteroptions), _searchfreeparameters(searchfreeparameters), _vfreegoalvalues(freevalues)
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

    std::stringstream ssout, ssin;
    ssin << "GetFreeIndices";
    if( !pmanip->GetIkSolver()->SendCommand(ssout, ssin) ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to GetFreeIndices from iksolver", ORE_Assert);
    }
    std::vector<int> vfreeindices((istream_iterator<int>(ssout)), istream_iterator<int>());
    if( (int)vfreeindices.size() != pmanip->GetIkSolver()->GetNumFreeParameters() ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("free parameters from iksolver do not match", ORE_Assert);
    }

    // have to convert to roobt dof
    for(size_t i = 0; i < vfreeindices.size(); ++i) {
        vfreeindices[i] = pmanip->GetArmIndices().at(vfreeindices[i]); // have to convert to robot dof!
    }
    _probot->GetDOFWeights(_vfreeweights, vfreeindices);

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

static bool ComparePriorityPair(const std::pair<int, dReal>& p0, const std::pair<int, dReal>& p1)
{
    return p0.second > p1.second;
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
    int numfree = _pmanip->GetIkSolver()->GetNumFreeParameters();
    std::vector<dReal> vfree;
    IkReturnPtr ikreturnjittered;
    for(int itry = 0; itry < _nummaxtries; ++itry ) {
        if( _listsamples.size() == 0 ) {
            return IkReturnPtr();
        }
        _pindexsampler->SampleSequence(vindex,1,IT_OpenEnd);
        int isampleindex = (int)(vindex.at(0)*_listsamples.size());
        std::list<SampleInfo>::iterator itsample = _listsamples.begin();
        advance(itsample,isampleindex);

        SampleInfo& sampleinfo = *itsample;

        int numRedundantSamplesForEEChecking = 0;
        if( (int)_pmanip->GetArmIndices().size() > sampleinfo._ikparam.GetDOF() ) {
            numRedundantSamplesForEEChecking = 40;
        }

        bool bFullEndEffectorKnown = sampleinfo._ikparam.GetType() == IKP_Transform6D || _pmanip->GetArmDOF() <= sampleinfo._ikparam.GetDOF();
        bool bCheckEndEffector = true;
        bool bCheckEndEffectorSelf = true;
        if( _ikfilteroptions & IKFO_IgnoreEndEffectorEnvCollisions ) {
            // use requested end effector to be always ignored
            bCheckEndEffector = false;
        }
        if( _ikfilteroptions & IKFO_IgnoreEndEffectorSelfCollisions ) {
            // use requested end effector to be always ignored
            bCheckEndEffectorSelf = false;
        }

        // if first grasp, quickly prune grasp is end effector is in collision
        IkParameterization ikparam = sampleinfo._ikparam;
        if( sampleinfo._numleft == _nummaxsamples && (bCheckEndEffector || bCheckEndEffectorSelf) ) { //!(_ikfilteroptions & IKFO_IgnoreEndEffectorEnvCollisions) ) {
            // because a goal can be colliding, have to always go in this loop and check if the end effector
            // could be jittered.
            // if bCheckEndEffector is true, then should call CheckEndEffectorCollision to quickly prune samples; otherwise, have to rely on calling FindIKSolution
            try {
                if( (bCheckEndEffector && _pmanip->CheckEndEffectorCollision(ikparam,_report, numRedundantSamplesForEEChecking)) || (bCheckEndEffectorSelf && _pmanip->CheckEndEffectorSelfCollision(ikparam,_report, numRedundantSamplesForEEChecking,true))) {
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
                                    if( (!bCheckEndEffector || !_pmanip->CheckEndEffectorCollision(ikparamjittered,_report, numRedundantSamplesForEEChecking)) && (!bCheckEndEffectorSelf || !_pmanip->CheckEndEffectorSelfCollision(ikparamjittered,_report, numRedundantSamplesForEEChecking,true)) ) {
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
                            dReal tryDelta = (_fjittermaxdist*2)/nMaxIterations;
                            for(int iiter = 1; iiter <= nMaxIterations; ++iiter) {
                                _pindexsampler->SampleSequence(xyzsamples,3,IT_Closed);
                                tjitter.trans = Vector(xyzsamples[0]-0.5f, xyzsamples[1]-0.5f, xyzsamples[2]-0.5f) * (tryDelta*iiter);
                                IkParameterization ikparamjittered = tjitter * ikparam;
                                try {
                                    if( (!bCheckEndEffector || !_pmanip->CheckEndEffectorCollision(ikparamjittered, _report, numRedundantSamplesForEEChecking)) && (!bCheckEndEffectorSelf || !_pmanip->CheckEndEffectorSelfCollision(ikparamjittered, _report, numRedundantSamplesForEEChecking,true)) ) {
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
                if( sampleinfo._ikparam.GetType() == IKP_Transform6D ) {
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

        vfree.resize(0);
        int orgindex = sampleinfo._orgindex;
        if( numfree > 0 ) {
            if( _searchfreeparameters ) {
                // halton sampler
                if( !sampleinfo._psampler ) {
                    sampleinfo._psampler = RaveCreateSpaceSampler(_probot->GetEnv(),"halton");
                    sampleinfo._psampler->SetSpaceDOF(numfree);
#if 1
                    // read all the samples and order them so that samples close to 0.5 are sampled first!
                    sampleinfo._psampler->SampleSequence(sampleinfo._vfreesamples,_nummaxsamples);
                    OPENRAVE_ASSERT_OP((int)sampleinfo._vfreesamples.size(),==,_nummaxsamples*numfree);
                    sampleinfo._vcachedists.resize(_nummaxsamples);
                    for(int isample = 0; isample < _nummaxsamples; ++isample) {
                        dReal dist = 0;
                        for(int jfree = 0; jfree < numfree; ++jfree) {
                            dist += _vfreeweights[jfree]*RaveFabs(sampleinfo._vfreesamples[isample*numfree+jfree] - 0.5);
                        }

                        sampleinfo._vcachedists[isample].first = isample;
                        sampleinfo._vcachedists[isample].second = dist;
                    }
                    std::sort(sampleinfo._vcachedists.begin(), sampleinfo._vcachedists.end(), ComparePriorityPair);
#endif
                }

#if 1
                if( sampleinfo._vcachedists.size() > 0 ) {
                    int isample = sampleinfo._vcachedists.back().first;
                    sampleinfo._vcachedists.pop_back();
                    vfree.resize(numfree);
                    std::copy(sampleinfo._vfreesamples.begin() + isample*numfree, sampleinfo._vfreesamples.begin() + (isample+1)*numfree, vfree.begin());
                }
#else
                sampleinfo._psampler->SampleSequence(vfree,1);
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
#endif
            }
            else if (!_vfreegoalvalues.empty()) {
                vfree = _vfreegoalvalues;
            }
            else {
                _pmanip->GetIkSolver()->GetFreeParameters(vfree);
            }
        }
        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            std::stringstream ss; ss << "free=[";
            FOREACHC(itfree, vfree) {
                ss << *itfree << ", ";
            }
            ss << "]";
            RAVELOG_VERBOSE(ss.str());
        }
        bool bsuccess = _pmanip->FindIKSolutions(ikparam, vfree, _ikfilteroptions|(bFullEndEffectorKnown&&bCheckEndEffector ? IKFO_IgnoreEndEffectorEnvCollisions : 0), _vikreturns);
        if( --sampleinfo._numleft <= 0 || vfree.size() == 0 || !_searchfreeparameters ) {
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
