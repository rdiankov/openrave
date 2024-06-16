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
#include "trajectoryretimer3.h"
#include <openrave/planningutils.h>

#include "piecewisepolynomials/quinticinterpolator.h"

namespace rplanners {

namespace PiecewisePolynomials = PiecewisePolynomialsInternal;

class QuinticTrajectoryRetimer : public TrajectoryRetimer3
{
public:
    class QuinticGroupInfo : public GroupInfo
    {
public:
        QuinticGroupInfo(int degree_, const ConfigurationSpecification::Group& gPos_, const ConfigurationSpecification::Group& gVel_, const ConfigurationSpecification::Group& gAccel_) : GroupInfo(degree_, gPos_, gVel_, gAccel_)
        {
        }

        TrajectoryBasePtr ptraj;
        int waypointIndex;
        int timeIndex;
        int posIndex, velIndex, accelIndex;
    }; // end class QuinticGroupInfo
    typedef boost::shared_ptr<QuinticGroupInfo> QuinticGroupInfoPtr;
    typedef boost::shared_ptr<QuinticGroupInfo const> QuinticGroupInfoConstPtr;

    QuinticTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : TrajectoryRetimer3(penv, sinput)
    {
        __description = ":Interface Author: Puttichai Lertkultanon and Rosen Diankov\n\nSimple quintic trajectory retiming while passing through all the waypoints. Waypoints will not be modified. This assumes that all waypoints have zero velocity and acceleration (unless the start and final points are forced). Overwrites the velocities, accelerations, and timestamps of the input trajectory.";
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        _pinterpolator.reset(new PiecewisePolynomials::QuinticInterpolator(_parameters->GetDOF(), GetEnv()->GetId()));
        _ptranslationInterpolator.reset(new PiecewisePolynomials::QuinticInterpolator(3, GetEnv()->GetId()));
        // TODO: _pikInterpolator
        _checker.Initialize(_parameters->GetDOF(), GetEnv()->GetId());
        _trajXmlId = ptraj->GetXMLId();
        return TrajectoryRetimer3::PlanPath(ptraj);
    }

protected:
    GroupInfoPtr CreateGroupInfo(int degree, const ConfigurationSpecification& origSpec, const ConfigurationSpecification::Group& gPos, const ConfigurationSpecification::Group& gVel, const ConfigurationSpecification::Group& gAccel)
    {
        QuinticGroupInfoPtr g(new QuinticGroupInfo(degree, gPos, gVel, gAccel));
        ConfigurationSpecification spec;

        g->posIndex = spec.AddGroup(gPos.name, gPos.dof, "quintic");
        g->velIndex = spec.AddGroup(gVel.name, gVel.dof, "quartic");
        g->accelIndex = spec.AddGroup(gAccel.name, gAccel.dof, "cubic");
        g->waypointIndex = spec.AddGroup("iswaypoint", 1, "next");
        g->timeIndex = spec.AddDeltaTimeGroup();
        g->ptraj = RaveCreateTrajectory(GetEnv(), _trajXmlId);
        g->ptraj->Init(spec);
        return g;
    }

    void ResetCachedGroupInfo(GroupInfoPtr g)
    {
        QuinticGroupInfoPtr qgroup = boost::dynamic_pointer_cast<QuinticGroupInfo>(g);
        if( qgroup->ptraj->GetNumWaypoints() > 0 ) {
            qgroup->ptraj->Remove(0, qgroup->ptraj->GetNumWaypoints());
        }
    }

    bool _SupportInterpolation()
    {
        if( _parameters->_interpolation.size() == 0 ) {
            _parameters->_interpolation = "quintic";
            return true;
        }
        else {
            return _parameters->_interpolation == "quintic";
        }
    }

    //
    // _ComputeMinimumTimeX functions
    //
    dReal _ComputeMinimumTimeJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData)
    {
        _v0pos.resize(info->gPos.dof);
        _v1pos.resize(info->gPos.dof);
        for (int i = 0; i < info->gPos.dof; ++i) {
            _v0pos[i] = *(itDataPrev + info->gPos.offset + i);
            _v1pos[i] = _v0pos[i] + *(itOrgDiff + info->orgPosOffset + i);
        }
        bool bZeroVelAccel = true;
        _v0vel.resize(info->gVel.dof);
        _v1vel.resize(info->gVel.dof);
        for (int i = 0; i < info->gVel.dof; ++i) {
            dReal vel0 = *(itDataPrev + info->gVel.offset + i);
            if( bZeroVelAccel && RaveFabs(vel0) > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                bZeroVelAccel = false;
            }
            _v0vel[i] = vel0;

            dReal vel1 = *(itData + info->gVel.offset + i);
            if( bZeroVelAccel && RaveFabs(vel1) > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                bZeroVelAccel = false;
            }
            _v1vel[i] = vel1;
        }
        _v0acc.resize(info->gAccel.dof);
        _v1acc.resize(info->gAccel.dof);
        for (int i = 0; i < info->gAccel.dof; ++i) {
            dReal acc0 = *(itDataPrev + info->gAccel.offset + i);
            if( bZeroVelAccel && RaveFabs(acc0) > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                bZeroVelAccel = false;
            }
            _v0acc[i] = acc0;

            dReal acc1 = *(itData + info->gAccel.offset + i);
            if( bZeroVelAccel && RaveFabs(acc1) > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                bZeroVelAccel = false;
            }
            _v1acc[i] = acc1;
        }

        if( !bZeroVelAccel ) {
            RAVELOG_WARN_FORMAT("env=%s, quinticretimer does not handle non-zero velocities and accelerations yet", GetEnv()->GetNameId());
            return -1;
        }

        bool bSuccess = false;
        dReal duration = -1;
        if( _bManipConstraints && !!_manipConstraintChecker ) {
            OPENRAVE_ASSERT_OP(_parameters->GetDOF(), ==, info->gPos.dof);
            OPENRAVE_ASSERT_OP(_manipConstraintChecker->GetCheckManips().size(), ==, 1);
            RobotBase::ManipulatorPtr pmanip = _manipConstraintChecker->GetCheckManips().front().pmanip;
            OPENRAVE_ASSERT_OP(pmanip->GetArmDOF(), ==, info->gPos.dof);

            // Look at the first pose and try to determine proper velocity limits
            std::vector<dReal> &vellimits = _cachevellimits, &accellimits = _cacheaccellimits, &jerklimits = _cachejerklimits;
            vellimits = info->_vConfigVelocityLimit;
            accellimits = info->_vConfigAccelerationLimit;
            jerklimits = info->_vConfigJerkLimit;

            // Cannot use _parameters->SetStateValues... Note that here vellimits and accellimtis
            // are modified according to manipulator constraints
            pmanip->GetRobot()->SetDOFValues(_v0pos, KinBody::CLA_CheckLimits, pmanip->GetArmIndices());
            _manipConstraintChecker->GetMaxVelocitiesAccelerations(_v0vel, vellimits, accellimits);

            pmanip->GetRobot()->SetDOFValues(_v1pos, KinBody::CLA_CheckLimits, pmanip->GetArmIndices());
            _manipConstraintChecker->GetMaxVelocitiesAccelerations(_v1vel, vellimits, accellimits);

            for (size_t j = 0; j < info->_vConfigVelocityLimit.size(); ++j) {
                // Adjust vellimits and accellimits so that it does not fall below boundary
                // velocities
                dReal fminvel = max(RaveFabs(_v0vel[j]), RaveFabs(_v1vel[j]));
                if( vellimits[j] < fminvel ) {
                    vellimits[j] = fminvel;
                }
                else {
                    dReal f = max(fminvel, info->_vConfigVelocityLimit[j]);
                    if (vellimits[j] > f) {
                        vellimits[j] = f;
                    }
                }
                {
                    dReal f = info->_vConfigAccelerationLimit[j];
                    if( accellimits[j] > f ) {
                        accellimits[j] = f;
                    }
                }
                {
                    dReal f = info->_vConfigJerkLimit[j];
                    if( jerklimits[j] > f ) {
                        jerklimits[j] = f;
                    }
                }
            }

            size_t maxSlowDownTries = 10;
            for (size_t iSlowDown = 0; iSlowDown < maxSlowDownTries; ++iSlowDown) {
                if( bZeroVelAccel ) {
                    _pinterpolator->ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(_v0pos, _v1pos, vellimits, accellimits, jerklimits, _cacheInterpolatedChunks);
                    bSuccess = true;
                }
                else {
                    dReal tryDuration = 100.0; // TODO: this is bad
                    PiecewisePolynomials::PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(_v0pos, _v1pos, _v0vel, _v1vel, _v0acc, _v1acc, info->_vConfigLowerLimit, info->_vConfigUpperLimit, vellimits, accellimits, jerklimits, tryDuration, _cacheInterpolatedChunks);
                    if( ret != PiecewisePolynomials::PolynomialCheckReturn::PCR_Normal ) {
                        // Stop right away
                        break;
                    }
                    else {
                        bSuccess = true;
                    }
                }

                duration = 0;
                FOREACHC(itchunk, _cacheInterpolatedChunks) {
                    duration += itchunk->duration;
                }
                if( duration < g_fEpsilon ) {
                    // Stop right away
                    bSuccess = false;
                    break;
                }

                // NOTE: For now suppose that _cacheInterpolatedChunks contains only one chunk
                PiecewisePolynomials::CheckReturn manipret = _manipConstraintChecker->CheckChunkManipConstraints(_cacheInterpolatedChunks.front());
                if( manipret.retcode == 0 ) {
                    // Successful, so breaking right away.
                    break;
                }

                bSuccess = false;
                RAVELOG_VERBOSE_FORMAT("env=%s, manip constraints invalidated. iSlowDown = %d/%d, slowing down by %.15e", GetEnv()->GetNameId()%iSlowDown%maxSlowDownTries%manipret.fTimeBasedSurpassMult);
                // Slow down by reducing vellimits and accellimits.
                for (size_t j = 0; j < vellimits.size(); ++j ) {
                    dReal fminvel = max(RaveFabs(_v0vel[j]), RaveFabs(_v1vel[j]));
                    vellimits[j] = max(vellimits[j]*manipret.fTimeBasedSurpassMult, fminvel);
                    accellimits[j] *= manipret.fTimeBasedSurpassMult;
                    jerklimits[j] *= manipret.fTimeBasedSurpassMult;
                }
            }
        }
        else {
            // No manip constraint.
            if( bZeroVelAccel ) {
                _pinterpolator->ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(_v0pos, _v1pos, info->_vConfigVelocityLimit, info->_vConfigAccelerationLimit, info->_vConfigJerkLimit, _cacheInterpolatedChunks);
                bSuccess = true;
            }
            else {
                dReal tryDuration = 100.0; // TODO: this is bad.
                PiecewisePolynomials::PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(_v0pos, _v1pos, _v0vel, _v1vel, _v0acc, _v1acc, info->_vConfigLowerLimit, info->_vConfigUpperLimit, info->_vConfigVelocityLimit, info->_vConfigAccelerationLimit, info->_vConfigJerkLimit, tryDuration, _cacheInterpolatedChunks);
                if( ret == PiecewisePolynomials::PolynomialCheckReturn::PCR_Normal ) {
                    bSuccess = true;
                }
            }
            if( bSuccess ) {
                duration = 0;
                FOREACHC(itchunk, _cacheInterpolatedChunks) {
                    duration += itchunk->duration;
                }
            }
        }

        if( bSuccess ) {
            return duration;
        }
        else {
            return -1;
        }
    }

    dReal _ComputeMinimumTimeAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_ComputeMinimumTimeAffine not implemented"), ORE_NotImplemented);
    }

    // TODO
    dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData)
    {
        return -1;
    }

    void _ComputeVelocitiesAccelerationsJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        if( info->orgVelOffset >= 0 ) {
            for( int i = 0; i < info->gVel.dof; ++i ) {
                *(itData + info->gVel.offset + i) = *(itOrgDiff + info->orgVelOffset + i);
            }
        }
        else {
            for( int i = 0; i < info->gVel.dof; ++i ) {
                *(itData + info->gVel.offset + i) = 0;
            }
        }

        if( info->orgAccelOffset >= 0 ) {
            for( int i = 0; i < info->gAccel.dof; ++i ) {
                *(itData + info->gAccel.offset + i) = *(itOrgDiff + info->orgAccelOffset + i);
            }
        }
        else {
            for( int i = 0; i < info->gAccel.dof; ++i ) {
                *(itData + info->gAccel.offset + i) = 0;
            }
        }
    }

    void _ComputeVelocitiesAccelerationsAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_ComputeVelocitiesAccelerationsAffine not implemented"), ORE_NotImplemented);
    }

    void _ComputeVelocitiesAccelerationsIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        if( info->orgVelOffset >= 0 ) {
            for( int i = 0; i < info->gVel.dof; ++i ) {
                *(itData + info->gVel.offset + i) = *(itOrgDiff + info->orgVelOffset + i);
            }
        }
        else {
            for( int i = 0; i < info->gVel.dof; ++i ) {
                *(itData + info->gVel.offset + i) = 0;
            }
        }

        if( info->orgAccelOffset >= 0 ) {
            for( int i = 0; i < info->gAccel.dof; ++i ) {
                *(itData + info->gAccel.offset + i) = *(itOrgDiff + info->orgAccelOffset + i);
            }
        }
        else {
            for( int i = 0; i < info->gAccel.dof; ++i ) {
                *(itData + info->gAccel.offset + i) = 0;
            }
        }
    }

    //
    // _CheckX functions
    //
    bool _CheckJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        dReal deltatime = *(itData + _timeOffset);
        for(int i = 0; i < info->gVel.dof; ++i) {
            dReal fVel = *(itData + info->gVel.offset + i);
            if( checkOptions & 2 ) {
                if( RaveFabs(fVel) > info->_vConfigVelocityLimit.at(i) + PiecewisePolynomials::g_fPolynomialEpsilon ) {
                    return false;
                }
            }
        }
        for(int i = 0; i < info->gAccel.dof; ++i) {
            dReal fAcc = RaveFabs(*(itData + info->gAccel.offset + i));
            if( checkOptions & 4 ) {
                if( RaveFabs(fAcc) > info->_vConfigAccelerationLimit.at(i) + PiecewisePolynomials::g_fPolynomialEpsilon ) {
                    return false;
                }
            }
            if( checkOptions & 8 ) {
                dReal diff = RaveFabs(*(itDataPrev + info->gAccel.offset + i) - fAcc);
                if( RaveFabs(diff) > info->_vConfigJerkLimit.at(i) * deltatime + PiecewisePolynomials::g_fPolynomialEpsilon) {
                    return false;
                }
            }
        }
        return true;
    }

    bool _CheckAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_CheckAffine not implemented"), ORE_NotImplemented);
    }

    // TODO
    bool _CheckIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        return false;
    }

    //
    // _WriteX functions
    //
    bool _WriteJointValues(GroupInfoConstPtr infoRaw, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        QuinticGroupInfoConstPtr info = boost::dynamic_pointer_cast<QuinticGroupInfo const>(infoRaw);
        if( _parameters->_outputaccelchanges ) {
            _v0pos.resize(info->gPos.dof);
            _v1pos.resize(info->gPos.dof);
            for(int i = 0; i < info->gPos.dof; ++i) {
                _v0pos[i] = *(itDataPrev + info->gPos.offset + i);
                _v1pos[i] = _v0pos[i] + *(itOrgDiff + info->orgPosOffset + i);
            }
            _v0vel.resize(info->gVel.dof);
            _v1vel.resize(info->gVel.dof);
            for(int i = 0; i < info->gVel.dof; ++i) {
                _v0vel[i] = *(itDataPrev + info->gVel.offset + i);
                _v1vel[i] = *(itData + info->gVel.offset + i);
            }
            _v0acc.resize(info->gAccel.dof);
            _v1acc.resize(info->gAccel.dof);
            for(int i = 0; i < info->gAccel.dof; ++i) {
                _v0acc[i] = *(itDataPrev + info->gAccel.offset + i);
                _v1acc[i] = *(itData + info->gAccel.offset + i);
            }

            dReal deltatime = *(itData + _timeOffset);
            if( deltatime == 0 ) {
                if( info->ptraj->GetNumWaypoints() == 0 ) {
                    // add the first point, copy all the data rather than just this group
                    _vtrajpoints.resize(info->ptraj->GetConfigurationSpecification().GetDOF());
                    std::copy(itData + info->gPos.offset, itData + info->gPos.offset + info->gPos.dof, _vtrajpoints.begin() + info->posIndex);
                    std::copy(itData + info->gVel.offset, itData + info->gVel.offset + info->gVel.dof, _vtrajpoints.begin() + info->velIndex);
                    std::copy(itData + info->gAccel.offset, itData + info->gAccel.offset + info->gAccel.dof, _vtrajpoints.begin() + info->accelIndex);
                    _vtrajpoints.at(info->timeIndex) = deltatime;
                    _vtrajpoints.at(info->waypointIndex) = 1;
                    info->ptraj->Insert(info->ptraj->GetNumWaypoints(), _vtrajpoints);
                }
                return true;
            }
            OPENRAVE_ASSERT_OP(deltatime,>,0);
            if( deltatime < PiecewisePolynomials::g_fPolynomialEpsilon ) {
                RAVELOG_WARN_FORMAT("env=%s, delta time is really ill-conditioned: %e", GetEnv()->GetNameId()%deltatime);
            }

            PiecewisePolynomials::PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration
                                                                  (_v0pos, _v1pos, _v0vel, _v1vel, _v0acc, _v1acc, deltatime,
                                                                  info->_vConfigLowerLimit, info->_vConfigUpperLimit,
                                                                  info->_vConfigVelocityLimit, info->_vConfigAccelerationLimit, info->_vConfigJerkLimit,
                                                                  _cacheInterpolatedChunks);
            if( ret != PiecewisePolynomials::PolynomialCheckReturn::PCR_Normal ) {
                return false;
            }

            bool bIncludeFirstPoint = false;
            int ndof = info->ptraj->GetConfigurationSpecification().GetDOF();
            if( info->ptraj->GetNumWaypoints() == 0) {
                // Add the initial switch point only for the first point in the trajectory
                bIncludeFirstPoint = true;
                _vtrajpoints.resize(ndof*2);
            }
            else {
                _vtrajpoints.resize(ndof);
            }
            std::vector<dReal>::iterator ittargetdata = _vtrajpoints.begin();

            // For now suppose that _cacheInterpolatedChunks contains only one chunk
            const PiecewisePolynomials::Chunk& resultChunk = _cacheInterpolatedChunks.front();

            if( bIncludeFirstPoint ) {
                for (int j = 0; j < info->gPos.dof; ++j) {
                    *(ittargetdata + info->posIndex + j) = resultChunk.vpolynomials.at(j).Eval(0);
                    *(ittargetdata + info->velIndex + j) = resultChunk.vpolynomials.at(j).Evald1(0);
                    *(ittargetdata + info->accelIndex + j) = resultChunk.vpolynomials.at(j).Evald2(0);
                }
                *(ittargetdata + info->timeIndex) = 0;
                *(ittargetdata + info->waypointIndex) = 1;
                ittargetdata += ndof;
            }

            for (int j = 0; j < info->gPos.dof; ++j) {
                *(ittargetdata + info->posIndex + j) = resultChunk.vpolynomials.at(j).Eval(resultChunk.duration);
                *(ittargetdata + info->velIndex + j) = resultChunk.vpolynomials.at(j).Evald1(resultChunk.duration);
                *(ittargetdata + info->accelIndex + j) = resultChunk.vpolynomials.at(j).Evald2(resultChunk.duration);
            }
            *(ittargetdata + info->timeIndex) = resultChunk.duration;
            *(ittargetdata + info->waypointIndex) = 1;
            ittargetdata += ndof;

            _vtrajpoints.at(_vtrajpoints.size() - ndof + info->waypointIndex) = 1;
            info->ptraj->Insert(info->ptraj->GetNumWaypoints(), _vtrajpoints);
        }
        return true;
    }

    // TODO
    dReal _WriteAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        return true;
    }

    // TODO
    dReal _WriteIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        return true;
    }

    void _WriteTrajectory(TrajectoryBasePtr ptraj, const ConfigurationSpecification& newspec, const std::vector<dReal>& data)
    {
        ptraj->Init(newspec);
        if( _parameters->_outputaccelchanges ) {
            std::list<TrajectoryBaseConstPtr> listTrajectories;
            FOREACH(it, _listGroupInfos) {
                listTrajectories.push_back(boost::dynamic_pointer_cast<QuinticGroupInfo>(*it)->ptraj);
            }
            TrajectoryBasePtr pmergedtraj = planningutils::MergeTrajectories(listTrajectories);
            if( pmergedtraj->GetNumWaypoints() > 0 ) {
                vector<dReal> vmergeddata;
                pmergedtraj->GetWaypoints(0, pmergedtraj->GetNumWaypoints(), vmergeddata, newspec);
                ptraj->Insert(0, vmergeddata);
            }
        }
        else {
            ptraj->Insert(0, data);
        }
    }

    //
    // Members
    //
    std::string _trajXmlId;
    PiecewisePolynomials::InterpolatorBasePtr _pinterpolator, _ptranslationInterpolator, _pikInterpolator;
    PiecewisePolynomials::PolynomialChecker _checker;

    // cache
    vector<dReal> _v0pos, _v0vel, _v1pos, _v1vel, _v0acc, _v1acc;
    vector<dReal> _vtrajpoints;
    std::vector<dReal> _cachevellimits, _cacheaccellimits, _cachejerklimits;
    std::vector<PiecewisePolynomials::Chunk> _cacheInterpolatedChunks;

}; // end class QuinticTrajectoryRetimer

PlannerBasePtr CreateQuinticTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new QuinticTrajectoryRetimer(penv, sinput));
}

} // end namespace rplanners
