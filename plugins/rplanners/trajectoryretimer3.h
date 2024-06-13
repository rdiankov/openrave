// -*- coding: utf-8 -*-
// Copyright (C) 2019 Puttichai Lertkultanon and Rosen Diankov
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation, either version 3
// of the License, or at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
// even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
#ifndef OPENRAVE_TRAJECTORY_RETIMER3
#define OPENRAVE_TRAJECTORY_RETIMER3

#include "openraveplugindefs.h"
#include "manipconstraints3.h"

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave_plugins_rplanners", msgid)

namespace rplanners {

class TrajectoryRetimer3 : public PlannerBase
{
public:
    enum CheckLimitsOptions
    {
        Check_Positions = 1,
        Check_Velocities = 2,
        Check_Accelerations = 4,
        Check_Jerks = 8,
    };

protected:
    class GroupInfo
    {
public:
        GroupInfo(int degree_, const ConfigurationSpecification::Group& gPos_, const ConfigurationSpecification::Group& gVel_, const ConfigurationSpecification::Group& gAccel_) : degree(degree_), gPos(gPos_), gVel(gVel_), gAccel(gAccel_), orgPosOffset(-1), orgVelOffset(-1), orgAccelOffset(-1)
        {
        }
        virtual ~GroupInfo()
        {
        }

        int degree;
        const ConfigurationSpecification::Group &gPos, &gVel, &gAccel;
        int orgPosOffset, orgVelOffset, orgAccelOffset;
        std::vector<dReal> _vConfigLowerLimit, _vConfigUpperLimit, _vConfigVelocityLimit, _vConfigAccelerationLimit, _vConfigJerkLimit;
    }; // end class GroupInfo
    typedef boost::shared_ptr<GroupInfo> GroupInfoPtr;
    typedef boost::shared_ptr<GroupInfo const> GroupInfoConstPtr;

public:
    TrajectoryRetimer3(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Puttichai Lertkultanon and Rosen Diankov\nTrajectory re-timing without modifying any of the points. Overwrites the velocities, accelerations, and timestamps.";
        _bManipConstraints = false;
    }

    virtual PlannerStatus InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params) override
    {
        EnvironmentLock lock(GetEnv()->GetMutex());
        params->Validate();
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        _parameters->copy(params);
        // Reset the cache
        _cachedOldSpec = ConfigurationSpecification();
        _cachedNewSpec = ConfigurationSpecification();
        return _InitPlan() ? PlannerStatus(PS_HasSolution) : PlannerStatus(PS_Failed);
    }

    virtual PlannerStatus InitPlan(RobotBasePtr pbase, std::istream& isParameters) override
    {
        EnvironmentLock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        isParameters >> *_parameters;
        _parameters->Validate();
        // Reset the cache
        _cachedOldSpec = ConfigurationSpecification();
        _cachedNewSpec = ConfigurationSpecification();
        return _InitPlan() ? PlannerStatus(PS_HasSolution) : PlannerStatus(PS_Failed);
    }

    virtual bool _InitPlan()
    {
        _timeOffset = -1;
        if( (int)_parameters->_vConfigVelocityLimit.size() != _parameters->GetDOF() ) {
            return false;
        }
        _viMaxVel.resize(_parameters->_vConfigVelocityLimit.size());
        for( size_t idof = 0; idof < _viMaxVel.size(); ++idof ) {
            _viMaxVel[idof] = 1/_parameters->_vConfigVelocityLimit[idof];
        }
        _viMaxAccel.resize(_parameters->_vConfigAccelerationLimit.size());
        for( size_t idof = 0; idof < _viMaxAccel.size(); ++idof ) {
            _viMaxAccel[idof] = 1/_parameters->_vConfigAccelerationLimit[idof];
        }
        _viMaxJerk.resize(_parameters->_vConfigJerkLimit.size());
        for( size_t idof = 0; idof < _viMaxJerk.size(); ++idof ) {
            _viMaxJerk[idof] = 1/_parameters->_vConfigJerkLimit[idof];
        }

        // Initialize workspace manip constraints checker
        _bManipConstraints = _parameters->manipname.size() > 0 && (_parameters->maxmanipspeed>0 || _parameters->maxmanipaccel>0);
        if(_bManipConstraints ) {
            if( !_manipConstraintChecker ) {
                _manipConstraintChecker.reset(new ManipConstraintChecker3(GetEnv()));
            }
            _manipConstraintChecker->Init(_parameters->manipname, _parameters->_configurationspecification, _parameters->maxmanipspeed, _parameters->maxmanipaccel);
        }

        return _SupportInterpolation();
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        // TODO: only works with quintic interpolation now
        // Idea: this trajectory retimer supports interpolation of order at least quintic. velocities and accelerations if not provided will be set to zero. there is no need to have ComputeVelocityX functions since any velocities and accelerations will always result in a valid interpolation.
        BOOST_ASSERT(!!_parameters && !!ptraj && ptraj->GetEnv() == GetEnv());
        BOOST_ASSERT(_parameters->GetDOF() == _parameters->_configurationspecification.GetDOF());

        std::vector<ConfigurationSpecification::Group>::const_iterator itTimeGroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup("deltatime", false);
        if( _parameters->_hastimestamps && itTimeGroup == ptraj->GetConfigurationSpecification()._vgroups.end() ) {
            RAVELOG_WARN_FORMAT("env=%d, trajectory does not have timestamps even though parameters say they are needed", GetEnv()->GetId());
            return PS_Failed;
        }

        size_t numWaypoints = ptraj->GetNumWaypoints();
        if( numWaypoints == 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, trajectory does not have any waypoints", GetEnv()->GetId());
            return PS_Failed;
        }

        ConfigurationSpecification velSpec = _parameters->_configurationspecification.ConvertToVelocitySpecification();
        if( _parameters->_hasvelocities ) {
            FOREACHC(itgroup, velSpec._vgroups) {
                if( ptraj->GetConfigurationSpecification().FindCompatibleGroup(*itgroup, true) == ptraj->GetConfigurationSpecification()._vgroups.end() ) {
                    RAVELOG_WARN_FORMAT("env=%d, trajectory does not have velocity group %s even though parameters say it is needed", GetEnv()->GetId()%itgroup->name);
                    return PS_Failed;
                }
            }
        }

        ConfigurationSpecification accelSpec = _parameters->_configurationspecification.ConvertToDerivativeSpecification(2);
        if( _parameters->_hasaccelerations ) {
            FOREACHC(itgroup, accelSpec._vgroups) {
                if( ptraj->GetConfigurationSpecification().FindCompatibleGroup(*itgroup, true) == ptraj->GetConfigurationSpecification()._vgroups.end() ) {
                    RAVELOG_WARN_FORMAT("env=%d, trajectory does not have acceleration group %s even though parameters say it is needed", GetEnv()->GetId()%itgroup->name);
                    return PS_Failed;
                }
            }
        }

        // Initialize ConfigurationSpecification for the resulting trajectory
        ConfigurationSpecification newSpec = _parameters->_configurationspecification;
        newSpec.AddDerivativeGroups(1, false);
        newSpec.AddDerivativeGroups(2, false);
        newSpec.AddDeltaTimeGroup();

        // Check input joint values and clamp them (if limits are slightly violated). This hopefully helps the retimers
        // that only do simple <= and >= checkings.
        ptraj->GetWaypoints(0, numWaypoints, _vDiffData, _parameters->_configurationspecification);
        for( size_t i = 0; i < _vDiffData.size(); i += _parameters->GetDOF() ) {
            for( int jdof = 0; jdof < _parameters->GetDOF(); ++jdof ) {
                dReal l = _parameters->_vConfigLowerLimit.at(jdof);
                dReal u = _parameters->_vConfigUpperLimit.at(jdof);
                if( _vDiffData.at(i + jdof) < l ) {
                    if( _vDiffData[i + jdof] < l - g_fEpsilonJointLimit ) {
                        RAVELOG_WARN_FORMAT("env=%d, traj point %d idof=%d does not follow lower limit (%.15e < %.15e)", GetEnv()->GetId()%(i/_parameters->GetDOF())%jdof%_vDiffData[i + jdof]%l);
                        return PS_Failed;
                    }
                    _vDiffData[i + jdof] = l;
                }
                else if( _vDiffData.at(i + jdof) > u ) {
                    if( _vDiffData[i + jdof] > u + g_fEpsilonJointLimit ) {
                        RAVELOG_WARN_FORMAT("env=%d, traj point %d idof=%d does not follow upper limit (%.15e > %.15e)", GetEnv()->GetId()%(i/_parameters->GetDOF())%jdof%_vDiffData[i + jdof]%u);
                        return PS_Failed;
                    }
                    _vDiffData[i + jdof] = u;
                }
            }
        }

        _vData.resize(numWaypoints*newSpec.GetDOF());
        std::fill(_vData.begin(), _vData.end(), 0);
        ConfigurationSpecification::ConvertData(_vData.begin(), newSpec, _vDiffData.begin(), _parameters->_configurationspecification, numWaypoints, GetEnv());
        int degree = 1;

        if( numWaypoints > 1 ) {
            // Manage cached data
            const string& posInterpolation = _parameters->_interpolation;
            // TODO: check this if statement
            if( _cachedOldSpec != _parameters->_configurationspecification || posInterpolation != _cachedPosInterpolation ) {
                _listGroupInfos.clear();
                _listComputeMinTimeFns.clear();
                _listComputeVelocitiesAccelerationsFns.clear();
                _listCheckFns.clear();
                _listWriteFns.clear();
                _cachedNewSpec = newSpec;
                _cachedOldSpec = _parameters->_configurationspecification;
                _cachedPosInterpolation = posInterpolation;

                std::string velInterpolation = ConfigurationSpecification::GetInterpolationDerivative(posInterpolation);
                std::string accelInterpolation = ConfigurationSpecification::GetInterpolationDerivative(velInterpolation);

                const boost::array<std::string, 3> supportedGroups = {{"joint_values", "affine_transform", "ikparam_values"}};

                for( size_t igroup = 0; igroup < _cachedNewSpec._vgroups.size(); ++igroup ) {
                    ConfigurationSpecification::Group& gPos = _cachedNewSpec._vgroups[igroup];
                    size_t iGroupType;
                    for( iGroupType = 0; iGroupType < supportedGroups.size(); ++iGroupType ) {
                        if( gPos.name.size() >= supportedGroups[iGroupType].size() && gPos.name.substr(0, supportedGroups[iGroupType].size()) == supportedGroups[iGroupType] ) {
                            // Found.
                            break;
                        }
                    }
                    if( iGroupType >= supportedGroups.size() ) {
                        // Not supported.
                        continue;
                    }

                    std::vector<ConfigurationSpecification::Group>::const_iterator itGroup = _cachedOldSpec.FindCompatibleGroup(gPos);
                    BOOST_ASSERT(itGroup != _cachedOldSpec._vgroups.end());
                    int orgPosOffset = itGroup->offset;
                    BOOST_ASSERT(orgPosOffset + gPos.dof <= _parameters->GetDOF());

                    std::vector<ConfigurationSpecification::Group>::iterator itVelGroup = _cachedNewSpec._vgroups.begin() + (_cachedNewSpec.FindTimeDerivativeGroup(gPos) - _cachedNewSpec._vgroups.begin());
                    BOOST_ASSERT(itVelGroup != _cachedNewSpec._vgroups.end());

                    // Cannot use FindTimeDerivativeGroup since it returns const_iterator
                    std::vector<ConfigurationSpecification::Group>::iterator itAccelGroup = _cachedNewSpec._vgroups.begin() + (_cachedNewSpec.FindTimeDerivativeGroup(*itVelGroup) - _cachedNewSpec._vgroups.begin());
                    BOOST_ASSERT(itAccelGroup != _cachedNewSpec._vgroups.end());

                    _listGroupInfos.push_back(CreateGroupInfo(degree, _cachedNewSpec, gPos, *itVelGroup, *itAccelGroup));
                    _listGroupInfos.back()->orgPosOffset = orgPosOffset;
                    _listGroupInfos.back()->_vConfigLowerLimit = std::vector<dReal>(_parameters->_vConfigLowerLimit.begin() + itGroup->offset, _parameters->_vConfigLowerLimit.begin() + itGroup->offset + itGroup->dof);
                    _listGroupInfos.back()->_vConfigUpperLimit = std::vector<dReal>(_parameters->_vConfigUpperLimit.begin() + itGroup->offset, _parameters->_vConfigUpperLimit.begin() + itGroup->offset + itGroup->dof);
                    _listGroupInfos.back()->_vConfigVelocityLimit = std::vector<dReal>(_parameters->_vConfigVelocityLimit.begin() + itGroup->offset, _parameters->_vConfigVelocityLimit.begin() + itGroup->offset + itGroup->dof);
                    _listGroupInfos.back()->_vConfigAccelerationLimit = std::vector<dReal>(_parameters->_vConfigAccelerationLimit.begin() + itGroup->offset, _parameters->_vConfigAccelerationLimit.begin() + itGroup->offset + itGroup->dof);
                    _listGroupInfos.back()->_vConfigJerkLimit = std::vector<dReal>(_parameters->_vConfigJerkLimit.begin() + itGroup->offset, _parameters->_vConfigJerkLimit.begin() + itGroup->offset + itGroup->dof);

                    itGroup = _cachedOldSpec.FindCompatibleGroup(*itVelGroup);
                    if( itGroup != _cachedOldSpec._vgroups.end() ) {
                        _listGroupInfos.back()->orgVelOffset = itGroup->offset;
                    }

                    itGroup = _cachedOldSpec.FindCompatibleGroup(*itAccelGroup);
                    if( itGroup != _cachedOldSpec._vgroups.end() ) {
                        _listGroupInfos.back()->orgAccelOffset = itGroup->offset;
                    }

                    std::stringstream ss(gPos.name.substr(supportedGroups[iGroupType].size()));
                    std::string bodyName;
                    int affineDofs = 0;
                    IkParameterizationType ikType = IKP_None;
                    if( iGroupType == 1 ) {
                        ss >> bodyName >> affineDofs;
                    }
                    else if( iGroupType == 2 ) {
                        int nIkType = 0;
                        ss >> nIkType;
                        ikType = static_cast<IkParameterizationType>(nIkType);
                    }

                    // Set up Compute/Check/Write functions
                    {
                        if( iGroupType == 0 ) {
                            _listComputeMinTimeFns.push_back(boost::bind(&TrajectoryRetimer3::_ComputeMinimumTimeJointValues, this, _listGroupInfos.back(), _1, _2, _3));
                        }
                        else if( iGroupType == 1 ) {
                            _listComputeMinTimeFns.push_back(boost::bind(&TrajectoryRetimer3::_ComputeMinimumTimeAffine, this, _listGroupInfos.back(), affineDofs, _1, _2, _3));
                        }
                        else if( iGroupType == 2 ) {
                            _listComputeMinTimeFns.push_back(boost::bind(&TrajectoryRetimer3::_ComputeMinimumTimeIk, this, _listGroupInfos.back(), ikType, _1, _2, _3));
                        }
                    }

                    if( iGroupType == 0 ) {
                        if( _parameters->_hasvelocities || _parameters->_hasaccelerations ) {
                            _listCheckFns.push_back(boost::bind(&TrajectoryRetimer3::_CheckJointValues, this, _listGroupInfos.back(), _1, _2, _3));
                        }
                        else {
                            _listComputeVelocitiesAccelerationsFns.push_back(boost::bind(&TrajectoryRetimer3::_ComputeVelocitiesAccelerationsJointValues, this, _listGroupInfos.back(), _1, _2, _3));
                        }
                        _listWriteFns.push_back(boost::bind(&TrajectoryRetimer3::_WriteJointValues, this, _listGroupInfos.back(), _1, _2, _3));
                    }
                    else if( iGroupType == 1 ) {
                        if( _parameters->_hasvelocities || _parameters->_hasaccelerations ) {
                            _listCheckFns.push_back(boost::bind(&TrajectoryRetimer3::_CheckAffine, this, _listGroupInfos.back(), affineDofs, _1, _2, _3));
                        }
                        else {
                            _listComputeVelocitiesAccelerationsFns.push_back(boost::bind(&TrajectoryRetimer3::_ComputeVelocitiesAccelerationsAffine, this, _listGroupInfos.back(), affineDofs, _1, _2, _3));
                        }
                        _listWriteFns.push_back(boost::bind(&TrajectoryRetimer3::_WriteAffine, this, _listGroupInfos.back(), affineDofs, _1, _2, _3));
                    }
                    else if( iGroupType == 2 ) {
                        if( _parameters->_hasvelocities || _parameters->_hasaccelerations ) {
                            _listCheckFns.push_back(boost::bind(&TrajectoryRetimer3::_CheckIk, this, _listGroupInfos.back(), ikType, _1, _2, _3));
                        }
                        else {
                            _listComputeVelocitiesAccelerationsFns.push_back(boost::bind(&TrajectoryRetimer3::_ComputeVelocitiesAccelerationsIk, this, _listGroupInfos.back(), ikType, _1, _2, _3));
                        }
                        _listWriteFns.push_back(boost::bind(&TrajectoryRetimer3::_WriteIk, this, _listGroupInfos.back(), ikType, _1, _2, _3));
                    }

                    gPos.interpolation = posInterpolation;
                    itVelGroup->interpolation = velInterpolation;
                    itAccelGroup->interpolation = accelInterpolation;
                } // end iterating through all groups in newspec

                _timeOffset = -1;
                FOREACHC(itgroup, _cachedNewSpec._vgroups) {
                    if( itgroup->name == "deltatime" ) {
                        _timeOffset = itgroup->offset;
                    }
                }
            }
            else {
                FOREACH(it, _listGroupInfos) {
                    ResetCachedGroupInfo(*it);
                }
            }

            int dof = _cachedNewSpec.GetDOF();
            std::vector<dReal>::iterator itData = _vData.begin();
            _vData.at(_timeOffset) = 0;

            // Set velocities and accelerations to zero at boundaries
            FOREACH(it, _listGroupInfos) {
                int velOffset = (*it)->gVel.offset;
                for( int jdof = 0; jdof < (*it)->gVel.dof; ++jdof ) {
                    _vData.at(velOffset + jdof) = 0; // initial point
                    _vData.at(_vData.size() - dof + velOffset + jdof) = 0; // final point
                }
                int accelOffset = (*it)->gAccel.offset;
                for( int jdof = 0; jdof < (*it)->gAccel.dof; ++jdof ) {
                    _vData.at(accelOffset + jdof) = 0; // initial point
                    _vData.at(_vData.size() - dof + accelOffset + jdof) = 0; // final point
                }
            }

            // Insert the valid original data into _vData. Note the difference from trajectoryretimer2 (this insertion
            // is done prior to diff state computation. TODO: verify this
            if( _parameters->_hastimestamps ) {
                ptraj->GetWaypoints(0, numWaypoints, _vTempData0, *itTimeGroup);
                ConfigurationSpecification::ConvertData(_vData.begin(), _cachedNewSpec, _vTempData0.begin(), *itTimeGroup, numWaypoints, GetEnv(), false);
            }
            if( _parameters->_hasvelocities ) {
                ptraj->GetWaypoints(0, numWaypoints, _vTempData0, velSpec);
                ConfigurationSpecification::ConvertData(_vData.begin(), _cachedNewSpec, _vTempData0.begin(), velSpec, numWaypoints, GetEnv(), false);
            }
            if( _parameters->_hasaccelerations ) {
                ptraj->GetWaypoints(0, numWaypoints, _vTempData0, accelSpec);
                ConfigurationSpecification::ConvertData(_vData.begin(), _cachedNewSpec, _vTempData0.begin(), accelSpec, numWaypoints, GetEnv(), false);
            }

            // Compute diff states
            std::vector<dReal>& vPrev = _vTempData0;
            std::vector<dReal>& vNext = _vTempData1;
            vPrev.resize(_cachedOldSpec.GetDOF());
            vNext.resize(_cachedOldSpec.GetDOF());
            std::copy(_vDiffData.end() - _cachedOldSpec.GetDOF(), _vDiffData.end(), vNext.begin());
            for( size_t iWaypoint = numWaypoints - 1; iWaypoint > 0; --iWaypoint ) {
                std::copy(_vDiffData.begin() + (iWaypoint - 1)*_cachedOldSpec.GetDOF(), _vDiffData.begin() + (iWaypoint)*_cachedOldSpec.GetDOF(), vPrev.begin());
                _parameters->_diffstatefn(vNext, vPrev);
                std::copy(vNext.begin(), vNext.end(), _vDiffData.begin() + (iWaypoint)*_cachedOldSpec.GetDOF());
                vNext = vPrev;
            }

            try {
                std::vector<dReal>::iterator itOrgDiff = _vDiffData.begin() + _cachedOldSpec.GetDOF();
                std::vector<dReal>::iterator itDataPrev = itData;
                itData += dof;

                for( size_t iWaypoint = 1; iWaypoint < numWaypoints; ++iWaypoint, itData += dof, itOrgDiff += _cachedOldSpec.GetDOF()) {
                    dReal minTime = 0;
                    // At this point, positions, velocities, and accelerations should already be filled (either with
                    // zeros or with the existing data from ptraj).
                    if( _parameters->_hastimestamps && (_parameters->_hasvelocities && _parameters->_hasaccelerations) ) {
                        // Arbitrary boundary velocities and accelerations: Since we have timestamps, we have full
                        // description of this trajectory segment. We can check if all limits (positions,
                        // velocities, accelerations, jerks, etc.) are respected.
                        FOREACH(itCheckFn, _listCheckFns) {
                            if( !(*itCheckFn)(itDataPrev, itData, Check_Positions|Check_Velocities|Check_Accelerations|Check_Jerks) ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, traj point %d/%d failed checking", GetEnv()->GetId()%iWaypoint%numWaypoints);
                                return PS_Failed;
                            }
                        }
                    }
                    else {
                        // Compute the largest minimum time
                        FOREACH(itMinTimeFn, _listComputeMinTimeFns) {
                            dReal fGroupTime = (*itMinTimeFn)(itOrgDiff, itDataPrev, itData);
                            if( fGroupTime < 0 ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, traj point %d/%d has uncomputable min time.", GetEnv()->GetId()%iWaypoint%numWaypoints);
                                return PS_Failed;
                            }

                            if( _parameters->_fStepLength > 0 ) {
                                if( fGroupTime < _parameters->_fStepLength ) {
                                    fGroupTime = _parameters->_fStepLength;
                                }
                                else {
                                    // Make fGroupTime multiple of steplength.
                                    fGroupTime = std::ceil((fGroupTime / _parameters->_fStepLength) - g_fEpsilonJointLimit) * _parameters->_fStepLength;
                                }
                            }
                            if( minTime < fGroupTime ) {
                                minTime = fGroupTime;
                            }
                        }

                        if( _parameters->_hastimestamps ) {
                            // Timestamps already exist. Check if the given data is feasible (i.e. the given timestamp
                            // is not less than minimum time).
                            // TODO: is this ok??? Previously we just ceil the computed minTime to be a multiple of
                            // steplengths.
                            if( *(itData + _timeOffset) < minTime - g_fEpsilonJointLimit ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, traj point %d/%d has unreachable minimum time (%.15e > %.15e)", GetEnv()->GetId()%iWaypoint%numWaypoints%(*(itData + _timeOffset))%minTime);
                                return PS_Failed;
                            }
                        }
                        else {
                            *(itData + _timeOffset) = minTime;
                        }

                        // Check the computed timestamp
                        FOREACH(itCheckFn, _listCheckFns) {
                            if( !(*itCheckFn)(itDataPrev, itData, Check_Positions|Check_Velocities|Check_Accelerations|Check_Jerks) ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, traj point %d/%d failed checking", GetEnv()->GetId()%iWaypoint%numWaypoints);
                                return PS_Failed;
                            }
                        }
                    }

                    FOREACH(itFn, _listWriteFns) {
                        if( !(*itFn)(itOrgDiff, itDataPrev, itData) ) {
                            RAVELOG_VERBOSE_FORMAT("env=%d, traj point %d/%d has unreachable new time %.15e s.", GetEnv()->GetId()%iWaypoint%numWaypoints%(*(itData + _timeOffset)));
                            return PS_Failed;
                        }
                    }
                    itDataPrev = itData;
                }
            }
            catch( const std::exception& ex ) {
                std::string filename = str(boost::format("%s/failedsmoothing%d.traj.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%1000));
                RAVELOG_WARN_FORMAT("env=%d, retimer failed with exception %s. writing original trajectory to %s", GetEnv()->GetId()%ex.what()%filename);
                std::ofstream f(filename.c_str());
                f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                ptraj->serialize(f);
                return PS_Failed;
            }
        }
        else {
            // Trajectory only has one point. Still need to set interpolation
            FOREACH(itgroup, _cachedNewSpec._vgroups) {
                itgroup->interpolation = _parameters->_interpolation;
            }
        }

        _WriteTrajectory(ptraj, _cachedNewSpec, _vData);
        return PS_HasSolution;
    }

protected:
    //
    // Methods to be overridden by individual timing types
    //

    /// \brief Create a GroupInfo
    virtual GroupInfoPtr CreateGroupInfo(int degree, const ConfigurationSpecification& spec, const ConfigurationSpecification::Group& gPos, const ConfigurationSpecification::Group& gVel, const ConfigurationSpecification::Group& gAccel) {
        return GroupInfoPtr(new GroupInfo(degree, gPos, gVel, gAccel));
    }

    /// \breif Reset any cached data in the given GroupInfo
    virtual void ResetCachedGroupInfo(GroupInfoPtr g) {
    }

    virtual bool _SupportInterpolation() = 0;

    /// \brief _ComputeMinimumTimeX (where X is one of JointValues, Affine, or Ik): computes the minimum time to go to
    ///        the point. returns mintime >= 0 if succeeded, otherwise returns a value mintime < 0.
    virtual dReal _ComputeMinimumTimeJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData) = 0;

    virtual dReal _ComputeMinimumTimeAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData) = 0;

    virtual dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData) = 0;

    /// \brief _ComputeVelocitiesAccelerationsX (where X is one of JointValues, Affine, or Ik): given deltaTime,
    ///        computes velocities and accelerations
    virtual void _ComputeVelocitiesAccelerationsJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData) = 0;
    virtual void _ComputeVelocitiesAccelerationsAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData) = 0;
    virtual void _ComputeVelocitiesAccelerationsIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData) = 0;

    /// \brief _CheckX (where X is one of JointValues, Affine, or Ik): given deltaTime, velocities, and accelerations,
    ///        check limits.
    virtual bool _CheckJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        return true;
    }

    virtual bool _CheckAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        return true;
    }

    virtual bool _CheckIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        return true;
    }

    /// \brief _WriteX
    virtual bool _WriteJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        return true;
    }

    virtual dReal _WriteAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        return true;
    }

    virtual dReal _WriteIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        return true;
    }

    virtual void _WriteTrajectory(TrajectoryBasePtr ptraj, const ConfigurationSpecification& newSpec, const std::vector<dReal>& data)
    {
        ptraj->Init(newSpec);
        ptraj->Insert(0, data);
    }

    //
    // Members
    //
    ConstraintTrajectoryTimingParametersPtr _parameters;
    boost::shared_ptr<ManipConstraintChecker3> _manipConstraintChecker;
    bool _bManipConstraints; // if true, then check workspace manip constraints

    // Cache
    ConfigurationSpecification _cachedOldSpec, _cachedNewSpec;
    std::string _cachedPosInterpolation;
    std::list< boost::function< dReal(std::vector<dReal>::const_iterator, std::vector<dReal>::const_iterator, std::vector<dReal>::const_iterator) > > _listComputeMinTimeFns;
    std::list< boost::function< void(std::vector<dReal>::const_iterator, std::vector<dReal>::const_iterator, std::vector<dReal>::iterator) > > _listComputeVelocitiesAccelerationsFns;
    std::list< boost::function< bool(std::vector<dReal>::const_iterator, std::vector<dReal>::const_iterator, int) > > _listCheckFns;
    std::list< boost::function< bool(std::vector<dReal>::const_iterator, std::vector<dReal>::const_iterator, std::vector<dReal>::iterator) > > _listWriteFns;
    std::vector<dReal> _viMaxVel, _viMaxAccel, _viMaxJerk;
    std::vector<dReal> _vDiffData, _vData;
    int _timeOffset;
    std::list<GroupInfoPtr> _listGroupInfos;
    std::vector<dReal> _vTempData0, _vTempData1;

}; // end class TrajectoryRetimer3

} // end namespace rplanners
#endif
