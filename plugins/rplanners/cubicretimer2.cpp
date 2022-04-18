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

#include "piecewisepolynomials/cubicinterpolator.h"

namespace rplanners {

namespace PiecewisePolynomials = PiecewisePolynomialsInternal;

class CubicTrajectoryRetimer2 : public TrajectoryRetimer3
{
public:
    class CubicGroupInfo : public GroupInfo
    {
public:
        CubicGroupInfo(int degree_, const ConfigurationSpecification::Group& gPos_, const ConfigurationSpecification::Group& gVel_, const ConfigurationSpecification::Group& gAccel_) : GroupInfo(degree_, gPos_, gVel_, gAccel_)
        {
        }

        TrajectoryBasePtr ptraj;
        int waypointIndex;
        int timeIndex;
        int posIndex, velIndex, accelIndex;
    }; // end class CubicGroupInfo
    typedef boost::shared_ptr<CubicGroupInfo> CubicGroupInfoPtr;
    typedef boost::shared_ptr<CubicGroupInfo const> CubicGroupInfoConstPtr;

    CubicTrajectoryRetimer2(EnvironmentBasePtr penv, std::istream& sinput) : TrajectoryRetimer3(penv, sinput)
    {
        __description = ":Interface Author: Puttichai Lertkultanon and Rosen Diankov\n\nSimple cubic trajectory retiming while passing through all the waypoints. Waypoints will not be modified. This assumes that all waypoints have zero velocity and acceleration (unless the start and final points are forced). Overwrites the velocities, accelerations, and timestamps of the input trajectory.";
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        _pinterpolator.reset(new PiecewisePolynomials::CubicInterpolator(_parameters->GetDOF(), GetEnv()->GetId()));
        _ptranslationInterpolator.reset(new PiecewisePolynomials::CubicInterpolator(3, GetEnv()->GetId()));
        _checker.Initialize(_parameters->GetDOF(), GetEnv()->GetId());
        _trajXmlId = ptraj->GetXMLId();
        return TrajectoryRetimer3::PlanPath(ptraj);
    }

protected:
    GroupInfoPtr CreateGroupInfo(int degree, const ConfigurationSpecification& origSpec, const ConfigurationSpecification::Group& gPos, const ConfigurationSpecification::Group& gVel, const ConfigurationSpecification::Group& gAccel)
    {
        CubicGroupInfoPtr g(new CubicGroupInfo(degree, gPos, gVel, gAccel));
        ConfigurationSpecification spec;

        g->posIndex = spec.AddGroup(gPos.name, gPos.dof, "cubic");
        g->velIndex = spec.AddGroup(gVel.name, gVel.dof, "quadratic");
        g->accelIndex = spec.AddGroup(gAccel.name, gAccel.dof, "linear");
        g->waypointIndex = spec.AddGroup("iswaypoint", 1, "next");
        g->timeIndex = spec.AddDeltaTimeGroup();
        g->ptraj = RaveCreateTrajectory(GetEnv(), _trajXmlId);
        g->ptraj->Init(spec);
        return g;
    }

    void ResetCachedGroupInfo(GroupInfoPtr g)
    {
        CubicGroupInfoPtr qgroup = boost::dynamic_pointer_cast<CubicGroupInfo>(g);
        if( qgroup->ptraj->GetNumWaypoints() > 0 ) {
            qgroup->ptraj->Remove(0, qgroup->ptraj->GetNumWaypoints());
        }
    }

    bool _SupportInterpolation()
    {
        if( _parameters->_interpolation.size() == 0 ) {
            _parameters->_interpolation = "cubic";
            return true;
        }
        else {
            return _parameters->_interpolation == "cubic";
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
                    dReal maxAllowedDuration = 0; // not constrained
                    PiecewisePolynomials::PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(_v0pos, _v1pos, _v0vel, _v1vel, _v0acc, _v1acc, info->_vConfigLowerLimit, info->_vConfigUpperLimit, vellimits, accellimits, jerklimits, maxAllowedDuration, _cacheInterpolatedChunks);
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
                dReal maxAllowedDuration = 0; // not constrained
                PiecewisePolynomials::PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(_v0pos, _v1pos, _v0vel, _v1vel, _v0acc, _v1acc, info->_vConfigLowerLimit, info->_vConfigUpperLimit, info->_vConfigVelocityLimit, info->_vConfigAccelerationLimit, info->_vConfigJerkLimit, maxAllowedDuration, _cacheInterpolatedChunks);
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

    dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData)
    {
        IkParameterization ikParamPrev, ikParam; // TODO: cache these
        ikParamPrev.Set(itDataPrev + info->gPos.offset, ikType);
        ikParam.Set(itData + info->gPos.offset, ikType);

        Vector trans0, trans1; // translation values of the previous and the current points, respectively
        int transOffset = -1; // offset index of the start of the translation values, which is different depending on the type of ikparam.
        dReal minTime = -1.0; // the computed minimum time. valid if >= 0.
        PiecewisePolynomials::PiecewisePolynomial pwpoly; // TODO: cache this

        // Compute the minimum time needed for rotational part and translational part separately.
        switch( ikType ) {
        case IKP_Transform6D: {
            dReal cosAngle = RaveFabs(ikParamPrev.GetTransform6D().rot.dot(ikParam.GetTransform6D().rot));
            if( cosAngle > 1.0 ) {
                cosAngle = 1.0;
            }
            else if( cosAngle < -1.0 ) {
                cosAngle = -1.0;
            }
            // Skip min-time computation if no movement is required.
            if( cosAngle < 1.0 - g_fEpsilon ) {
                // TODO: write descriptions for all derivations of angular velocity/acceleration from quaternions
                const dReal angleDelta = 2.0*RaveAcos(cosAngle);
                const Vector angularVelocityPrev = 2.0*quatMultiply(Vector(*(itDataPrev + info->gVel.offset + 0), *(itDataPrev + info->gVel.offset + 1), *(itDataPrev + info->gVel.offset + 2), *(itDataPrev + info->gVel.offset + 3)),
                                                                    quatInverse(ikParamPrev.GetTransform6D().rot));
                const dReal angleVelPrev = RaveSqrt(utils::Sqr(angularVelocityPrev.y) + utils::Sqr(angularVelocityPrev.z) + utils::Sqr(angularVelocityPrev.w));

                const Vector angularVelocity = 2.0*quatMultiply(Vector(*(itData + info->gVel.offset + 0), *(itData + info->gVel.offset + 1), *(itData + info->gVel.offset + 2), *(itData + info->gVel.offset + 3)),
                                                                quatInverse(ikParam.GetTransform6D().rot));
                const dReal angleVel = RaveSqrt(utils::Sqr(angularVelocity.y) + utils::Sqr(angularVelocity.z) + utils::Sqr(angularVelocity.w));

                const Vector angularAccelerationPrev = 2.0*quatMultiply(Vector(*(itDataPrev + info->gAccel.offset + 0), *(itDataPrev + info->gAccel.offset + 1), *(itDataPrev + info->gAccel.offset + 2), *(itDataPrev + info->gAccel.offset + 3)),
                                                                        quatInverse(ikParamPrev.GetTransform6D().rot));
                const dReal angleAccelPrev = RaveSqrt(utils::Sqr(angularAccelerationPrev.y) + utils::Sqr(angularAccelerationPrev.z) + utils::Sqr(angularAccelerationPrev.w));

                const Vector angularAcceleration = 2.0*quatMultiply(Vector(*(itData + info->gAccel.offset + 0), *(itData + info->gAccel.offset + 1), *(itData + info->gAccel.offset + 2), *(itData + info->gAccel.offset + 3)),
                                                                    quatInverse(ikParam.GetTransform6D().rot));
                const dReal angleAccel = RaveSqrt(utils::Sqr(angularAcceleration.y) + utils::Sqr(angularAcceleration.z) + utils::Sqr(angularAcceleration.w));
                const dReal lowerLimit = -1000, upperLimit = angleDelta + 1000; // arbitrary limit
                PiecewisePolynomials::PolynomialCheckReturn ret = _pinterpolator->Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(0, angleDelta, angleVelPrev, angleVel, angleAccelPrev, angleAccel, lowerLimit, upperLimit, info->_vConfigVelocityLimit.at(0), info->_vConfigAccelerationLimit.at(0), info->_vConfigJerkLimit.at(0), pwpoly);
                if( ret != PiecewisePolynomials::PolynomialCheckReturn::PCR_Normal ) {
                    return -1.0;
                }
                minTime = pwpoly.GetDuration();
            }
            else {
                minTime = 0.0; // no rotational motion required
            }

            trans0 = ikParamPrev.GetTransform6D().trans;
            trans1 = ikParam.GetTransform6D().trans;
            transOffset = 4;
            break;
        }
        case IKP_Rotation3D: {
            dReal cosAngle = RaveFabs(ikParamPrev.GetRotation3D().dot(ikParam.GetRotation3D()));
            if( cosAngle > 1.0 ) {
                cosAngle = 1.0;
            }
            else if( cosAngle < -1.0 ) {
                cosAngle = -1.0;
            }
            // Skip min-time computation if no movement is required.
            if( cosAngle < 1.0 - g_fEpsilon ) {
                // TODO: write descriptions for all derivations of angular velocity/acceleration from quaternions
                const dReal angleDelta = 2.0*RaveAcos(cosAngle);
                const Vector angularVelocityPrev = 2.0*quatMultiply(Vector(*(itDataPrev + info->gVel.offset + 0), *(itDataPrev + info->gVel.offset + 1), *(itDataPrev + info->gVel.offset + 2), *(itDataPrev + info->gVel.offset + 3)),
                                                                    quatInverse(ikParamPrev.GetRotation3D()));
                const dReal angleVelPrev = RaveSqrt(utils::Sqr(angularVelocityPrev.y) + utils::Sqr(angularVelocityPrev.z) + utils::Sqr(angularVelocityPrev.w));

                const Vector angularVelocity = 2.0*quatMultiply(Vector(*(itData + info->gVel.offset + 0), *(itData + info->gVel.offset + 1), *(itData + info->gVel.offset + 2), *(itData + info->gVel.offset + 3)),
                                                                quatInverse(ikParam.GetRotation3D()));
                const dReal angleVel = RaveSqrt(utils::Sqr(angularVelocity.y) + utils::Sqr(angularVelocity.z) + utils::Sqr(angularVelocity.w));

                const Vector angularAccelerationPrev = 2.0*quatMultiply(Vector(*(itDataPrev + info->gAccel.offset + 0), *(itDataPrev + info->gAccel.offset + 1), *(itDataPrev + info->gAccel.offset + 2), *(itDataPrev + info->gAccel.offset + 3)),
                                                                        quatInverse(ikParamPrev.GetRotation3D()));
                const dReal angleAccelPrev = RaveSqrt(utils::Sqr(angularAccelerationPrev.y) + utils::Sqr(angularAccelerationPrev.z) + utils::Sqr(angularAccelerationPrev.w));

                const Vector angularAcceleration = 2.0*quatMultiply(Vector(*(itData + info->gAccel.offset + 0), *(itData + info->gAccel.offset + 1), *(itData + info->gAccel.offset + 2), *(itData + info->gAccel.offset + 3)),
                                                                    quatInverse(ikParam.GetRotation3D()));
                const dReal angleAccel = RaveSqrt(utils::Sqr(angularAcceleration.y) + utils::Sqr(angularAcceleration.z) + utils::Sqr(angularAcceleration.w));
                const dReal lowerLimit = -1000, upperLimit = angleDelta + 1000; // arbitrary limit
                PiecewisePolynomials::PolynomialCheckReturn ret = _pinterpolator->Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(0, angleDelta, angleVelPrev, angleVel, angleAccelPrev, angleAccel, lowerLimit, upperLimit, info->_vConfigVelocityLimit.at(0), info->_vConfigAccelerationLimit.at(0), info->_vConfigJerkLimit.at(0), pwpoly);
                if( ret != PiecewisePolynomials::PolynomialCheckReturn::PCR_Normal ) {
                    return -1.0;
                }
                minTime = pwpoly.GetDuration();
            }
            else {
                minTime = 0.0; // no rotational motion required
            }
            break;
        }
        case IKP_Translation3D: {
            trans0 = ikParamPrev.GetTranslation3D();
            trans1 = ikParam.GetTranslation3D();
            transOffset = 0;
            break;
        }
        case IKP_TranslationDirection5D: {
            // TODO:
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            // TODO:
        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            // TODO:
        }
        case IKP_TranslationZAxisAngleYNorm4D: {
            // TODO:
        }
        case IKP_TranslationZAxisAngle4D: {
            // TODO:
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, cubic retimer does not support ikparam type 0x%x"), GetEnv()->GetNameId()%ikParam.GetType(), ORE_InvalidArguments);
        } // end switch

        if( transOffset >= 0 ) {
            std::vector<dReal> xyzPrev(3, 0), xyz(3), xyzVelPrev(3), xyzVel(3), xyzAccelPrev(3), xyzAccel(3);
            std::vector<dReal> vLowerLimits(3), vUpperLimits(3);
            dReal totalDisplacement = 0.0;
            for( size_t idof = 0; idof < 3; ++idof ) {
                xyzPrev[idof] = trans0[idof];
                xyz[idof] = trans1[idof];
                xyzVelPrev[idof] = *(itDataPrev + info->gVel.offset + transOffset + idof);
                xyzVel[idof] = *(itData + info->gVel.offset + transOffset + idof);
                xyzAccelPrev[idof] = *(itDataPrev + info->gAccel.offset + transOffset + idof);
                xyzAccel[idof] = *(itData + info->gAccel.offset + transOffset + idof);

                dReal fMin, fMax;
                if( trans0[idof] > trans1[idof] ) {
                    fMin = trans1[idof];
                    fMax = trans0[idof];
                }
                else {
                    fMin = trans0[idof];
                    fMax = trans1[idof];
                }
                vLowerLimits[idof] = fMin - 1000;
                vUpperLimits[idof] = fMax + 1000;

                totalDisplacement += (fMax - fMin); // this is always non-negative
            }
            // Skip min-time computation if no movement is required.
            if( totalDisplacement > g_fEpsilon ) {
                std::vector<dReal> vVelLimits(info->_vConfigVelocityLimit.begin() + transOffset, info->_vConfigVelocityLimit.begin() + transOffset + 3);
                std::vector<dReal> vAccelLimits(info->_vConfigAccelerationLimit.begin() + transOffset, info->_vConfigAccelerationLimit.begin() + transOffset + 3);
                std::vector<dReal> vJerkLimits(info->_vConfigJerkLimit.begin() + transOffset, info->_vConfigJerkLimit.begin() + transOffset + 3);

                const dReal maxAllowedDuration = 0; // not constrained
                PiecewisePolynomials::PolynomialCheckReturn ret = _ptranslationInterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(xyzPrev, xyz, xyzVelPrev, xyzVel, xyzAccelPrev, xyzAccel, vLowerLimits, vUpperLimits, vVelLimits, vAccelLimits, vJerkLimits, maxAllowedDuration, _cacheInterpolatedChunks);
                if( ret != PiecewisePolynomials::PolynomialCheckReturn::PCR_Normal ) {
                    return -1.0;
                }
                dReal duration = 0;
                FOREACHC(itchunk, _cacheInterpolatedChunks) {
                    duration += itchunk->duration;
                }
                if( minTime < duration ) {
                    minTime = duration;
                }
            }
            else {
                if( minTime < 0 ) {
                    minTime = 0;
                }
            }
        } // end if transOffset >= 0

        return minTime;
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

    bool _CheckIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        const dReal deltaTime = *(itData + _timeOffset);
        int transOffset = -1;
        IkParameterization ikParamPrev, ikParam;
        switch( ikType ) {
        case IKP_Transform6D: {
            // TODO: verify the following computation
            ikParamPrev.Set(itDataPrev + info->gPos.offset, ikType);
            ikParam.Set(itDataPrev + info->gPos.offset, ikType);
            // const Vector angularVelocityPrev = 2.0*quatMultiply(Vector(*(itDataPrev + info->gVel.offset + 0), *(itDataPrev + info->gVel.offset + 1), *(itDataPrev + info->gVel.offset + 2), *(itDataPrev + info->gVel.offset + 3)),
            //                                                  quatInverse(ikParamPrev.GetTransform6D().rot));
            // const dReal fVelPrev2 = utils::Sqr(angularVelocityPrev.y) + utils::Sqr(angularVelocityPrev.z) + utils::Sqr(angularVelocityPrev.w);

            const Vector angularVelocity = 2.0*quatMultiply(Vector(*(itData + info->gVel.offset + 0), *(itData + info->gVel.offset + 1), *(itData + info->gVel.offset + 2), *(itData + info->gVel.offset + 3)),
                                                            quatInverse(ikParam.GetTransform6D().rot));
            const dReal fVel2 = utils::Sqr(angularVelocity.y) + utils::Sqr(angularVelocity.z) + utils::Sqr(angularVelocity.w);

            if( fVel2 > utils::Sqr(info->_vConfigVelocityLimit.at(0)) + PiecewisePolynomials::g_fPolynomialEpsilon ) {
                return false;
            }

            const Vector angularAccelerationPrev = 2.0*quatMultiply(Vector(*(itDataPrev + info->gAccel.offset + 0), *(itDataPrev + info->gAccel.offset + 1), *(itDataPrev + info->gAccel.offset + 2), *(itDataPrev + info->gAccel.offset + 3)),
                                                                    quatInverse(ikParamPrev.GetTransform6D().rot));
            const dReal fAccelPrev2 = utils::Sqr(angularAccelerationPrev.y) + utils::Sqr(angularAccelerationPrev.z) + utils::Sqr(angularAccelerationPrev.w);

            const Vector angularAcceleration = 2.0*quatMultiply(Vector(*(itData + info->gAccel.offset + 0), *(itData + info->gAccel.offset + 1), *(itData + info->gAccel.offset + 2), *(itData + info->gAccel.offset + 3)),
                                                                quatInverse(ikParam.GetTransform6D().rot));
            const dReal fAccel2 = utils::Sqr(angularAcceleration.y) + utils::Sqr(angularAcceleration.z) + utils::Sqr(angularAcceleration.w);
            if( fAccel2 > utils::Sqr(info->_vConfigAccelerationLimit.at(0)) + PiecewisePolynomials::g_fPolynomialEpsilon ) {
                return false;
            }
            if( (fAccel2 + fAccelPrev2 - 2*RaveSqrt(fAccel2*fAccelPrev2)) > utils::Sqr(deltaTime * (info->_vConfigJerkLimit.at(0) + PiecewisePolynomials::g_fPolynomialEpsilon)) ) {
                return false;
            }
            transOffset = 4;
            break;
        }
        case IKP_Rotation3D: {
            // TODO:
        }
        case IKP_Translation3D: {
            // TODO:
        }
        case IKP_TranslationDirection5D: {
            // TODO:
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            // TODO:
        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            // TODO:
        }
        case IKP_TranslationZAxisAngleYNorm4D: {
            // TODO:
        }
        case IKP_TranslationZAxisAngle4D: {
            // TODO:
        }
        default: {
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, cubic retimer does not support ikparam type 0x%x"), GetEnv()->GetNameId()%ikParam.GetType(), ORE_InvalidArguments);
        }
        } // end switch

        if( transOffset >= 0 ) {
            for( size_t idof = 0; idof < 3; ++idof ) {
                const dReal fVel = *(itData + info->gVel.offset + transOffset + idof);
                if( RaveFabs(fVel) > info->_vConfigVelocityLimit.at(transOffset + idof) + PiecewisePolynomials::g_fPolynomialEpsilon ) {
                    return false;
                }

                const dReal fAccelPrev = *(itDataPrev + info->gAccel.offset + transOffset + idof);
                const dReal fAccel = *(itData + info->gAccel.offset + transOffset + idof);
                if( RaveFabs(fAccel) > info->_vConfigAccelerationLimit.at(transOffset + idof) + PiecewisePolynomials::g_fPolynomialEpsilon ) {
                    return false;
                }
                if( RaveFabs(fAccel - fAccelPrev) > deltaTime*(info->_vConfigJerkLimit.at(transOffset + idof) + PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                    return false;
                }
            }
        }
        return true;
    }

    //
    // _WriteX functions
    //
    bool _WriteJointValues(GroupInfoConstPtr infoRaw, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        CubicGroupInfoConstPtr info = boost::dynamic_pointer_cast<CubicGroupInfo const>(infoRaw);
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
                _vtrajpoints.resize(ndof*(_cacheInterpolatedChunks.size() + 1));
            }
            else {
                _vtrajpoints.resize(ndof*_cacheInterpolatedChunks.size());
            }
            std::vector<dReal>::iterator ittargetdata = _vtrajpoints.begin();

            if( bIncludeFirstPoint ) {
                const PiecewisePolynomials::Chunk& firstChunk = _cacheInterpolatedChunks.front();
                for (int j = 0; j < info->gPos.dof; ++j) {
                    *(ittargetdata + info->posIndex + j) = firstChunk.vpolynomials.at(j).Eval(0);
                    *(ittargetdata + info->velIndex + j) = firstChunk.vpolynomials.at(j).Evald1(0);
                    *(ittargetdata + info->accelIndex + j) = firstChunk.vpolynomials.at(j).Evald2(0);
                }
                *(ittargetdata + info->timeIndex) = 0;
                *(ittargetdata + info->waypointIndex) = 1;
                ittargetdata += ndof;
            }

            FOREACHC(itchunk, _cacheInterpolatedChunks) {
                for (int j = 0; j < info->gPos.dof; ++j) {
                    *(ittargetdata + info->posIndex + j) = itchunk->vpolynomials.at(j).Eval(itchunk->duration);
                    *(ittargetdata + info->velIndex + j) = itchunk->vpolynomials.at(j).Evald1(itchunk->duration);
                    *(ittargetdata + info->accelIndex + j) = itchunk->vpolynomials.at(j).Evald2(itchunk->duration);
                }
                *(ittargetdata + info->timeIndex) = itchunk->duration;
                *(ittargetdata + info->waypointIndex) = 1;
                ittargetdata += ndof;
            }

            _vtrajpoints.at(_vtrajpoints.size() - ndof + info->waypointIndex) = 1;
            info->ptraj->Insert(info->ptraj->GetNumWaypoints(), _vtrajpoints);
        }
        return true;
    }

    dReal _WriteAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_WriteAffine not implemented"), ORE_NotImplemented);
    }

    dReal _WriteIk(GroupInfoConstPtr infoRaw, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
        CubicGroupInfoConstPtr info = boost::dynamic_pointer_cast<CubicGroupInfo const>(infoRaw);
        if( _parameters->_outputaccelchanges ) {
            const dReal deltaTime = *(itData + _timeOffset);
            if( deltaTime == 0 ) {
                if( info->ptraj->GetNumWaypoints() == 0 ) {
                    // Add the first waypoint to the traj. Copy all the data rather than just this ik group
                    _vtrajpoints.resize(info->ptraj->GetConfigurationSpecification().GetDOF());
                    std::copy(itData, itData + _vtrajpoints.size(), _vtrajpoints.begin());
                    _vtrajpoints.at(info->waypointIndex) = 1;
                    info->ptraj->Insert(info->ptraj->GetNumWaypoints(), _vtrajpoints);
                }
                return true;
            }
            OPENRAVE_ASSERT_OP(deltaTime, >, 0);
            if( deltaTime < PiecewisePolynomials::g_fEpsilonForTimeInstant ) {
                RAVELOG_WARN_FORMAT("env=%s, delta time=%.15e is ill-conditioned", GetEnv()->GetNameId()%deltaTime);
            }

            IkParameterization ikParamPrev, ikParam;
            ikParamPrev.Set(itDataPrev + info->gPos.offset, ikType);
            ikParam.Set(itData + info->gPos.offset, ikType);

            _v0pos.resize(0);
            _v1pos.resize(0);
            _v0vel.resize(0);
            _v1vel.resize(0);
            _v0acc.resize(0);
            _v1acc.resize(0);
            Vector trans0, trans1, axisAngle;
            int transOffset = -1, transIndex = -1;
            std::vector<dReal> vmaxvel, vmaxaccel, vmaxjerk, vlower, vupper;

            switch( ikType ) {
            case IKP_Transform6D: {
                _pikInterpolator.reset(new PiecewisePolynomials::CubicInterpolator(4, GetEnv()->GetId()));

                _v0pos.resize(4);
                _v1pos.resize(4);
                _v0vel.resize(4);
                _v1vel.resize(4);
                _v0acc.resize(4);
                _v1acc.resize(4);
                vmaxvel.resize(4);
                vmaxaccel.resize(4);
                vmaxjerk.resize(4);
                axisAngle = axisAngleFromQuat(quatMultiply(quatInverse(ikParamPrev.GetTransform6D().rot), ikParam.GetTransform6D().rot));
                if( axisAngle.lengthsqr3() > g_fEpsilon ) {
                    axisAngle.normalize3();
                }
                _v0pos[0] = 0;
                dReal cosAngle = RaveFabs(ikParamPrev.GetTransform6D().rot.dot(ikParam.GetTransform6D().rot));
                if( cosAngle > 1.0 ) {
                    cosAngle = 1.0;
                }
                _v1pos[0] = 2.0*RaveAcos(cosAngle);

                const Vector angularVelocityPrev = 2.0*quatMultiply(Vector(*(itDataPrev + info->gVel.offset + 0), *(itDataPrev + info->gVel.offset + 1), *(itDataPrev + info->gVel.offset + 2), *(itDataPrev + info->gVel.offset + 3)),
                                                                    quatInverse(ikParamPrev.GetTransform6D().rot));
                _v0vel[0] = RaveSqrt(utils::Sqr(angularVelocityPrev.y) + utils::Sqr(angularVelocityPrev.z) + utils::Sqr(angularVelocityPrev.w));

                const Vector angularVelocity = 2.0*quatMultiply(Vector(*(itData + info->gVel.offset + 0), *(itData + info->gVel.offset + 1), *(itData + info->gVel.offset + 2), *(itData + info->gVel.offset + 3)),
                                                                quatInverse(ikParam.GetTransform6D().rot));
                _v1vel[0] = RaveSqrt(utils::Sqr(angularVelocity.y) + utils::Sqr(angularVelocity.z) + utils::Sqr(angularVelocity.w));

                const Vector angularAccelerationPrev = 2.0*quatMultiply(Vector(*(itDataPrev + info->gAccel.offset + 0), *(itDataPrev + info->gAccel.offset + 1), *(itDataPrev + info->gAccel.offset + 2), *(itDataPrev + info->gAccel.offset + 3)),
                                                                        quatInverse(ikParamPrev.GetTransform6D().rot));
                _v0acc[0] = RaveSqrt(utils::Sqr(angularAccelerationPrev.y) + utils::Sqr(angularAccelerationPrev.z) + utils::Sqr(angularAccelerationPrev.w));

                const Vector angularAcceleration = 2.0*quatMultiply(Vector(*(itData + info->gAccel.offset + 0), *(itData + info->gAccel.offset + 1), *(itData + info->gAccel.offset + 2), *(itData + info->gAccel.offset + 3)),
                                                                    quatInverse(ikParam.GetTransform6D().rot));
                _v1acc[0] = RaveSqrt(utils::Sqr(angularAcceleration.y) + utils::Sqr(angularAcceleration.z) + utils::Sqr(angularAcceleration.w));

                vmaxvel[0] = info->_vConfigVelocityLimit.at(0);
                vmaxaccel[0] = info->_vConfigAccelerationLimit.at(0);
                vmaxjerk[0] = info->_vConfigJerkLimit.at(0);

                trans0 = ikParamPrev.GetTransform6D().trans;
                trans1 = ikParam.GetTransform6D().trans;

                transOffset = 4;
                transIndex = 1;
                break;
            }
            case IKP_Rotation3D: {
                // TODO:
            }
            case IKP_Translation3D: {
                // TODO:
            }
            case IKP_TranslationDirection5D: {
                // TODO:
            }
            case IKP_TranslationXAxisAngleZNorm4D: {
                // TODO:
            }
            case IKP_TranslationYAxisAngleXNorm4D: {
                // TODO:
            }
            case IKP_TranslationZAxisAngleYNorm4D: {
                // TODO:
            }
            case IKP_TranslationZAxisAngle4D: {
                // TODO:
            }
            default: {
                throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, cubic retimer does not support ikparam type 0x%x"), GetEnv()->GetNameId()%ikParam.GetType(), ORE_InvalidArguments);
            }
            } // end switch

            if( transOffset >= 0 ) {
                for( size_t idof = 0; idof < 3; ++idof ) {
                    _v0pos.at(transIndex + idof) = trans0[idof];
                    _v1pos.at(transIndex + idof) = trans1[idof];
                    _v0vel.at(transIndex + idof) = *(itDataPrev + info->gVel.offset + transOffset + idof);
                    _v1vel.at(transIndex + idof) = *(itData + info->gVel.offset + transOffset + idof);
                    _v0acc.at(transIndex + idof) = *(itDataPrev + info->gAccel.offset + transOffset + idof);
                    _v1acc.at(transIndex + idof) = *(itData + info->gAccel.offset + transOffset + idof);
                    vmaxvel.at(transIndex + idof) = info->_vConfigVelocityLimit.at(transOffset + idof);
                    vmaxaccel.at(transIndex + idof) = info->_vConfigAccelerationLimit.at(transOffset + idof);
                    vmaxjerk.at(transIndex + idof) = info->_vConfigJerkLimit.at(transOffset + idof);
                }
            }

            vlower.resize(_v0pos.size());
            vupper.resize(_v0pos.size());
            for( size_t idof = 0; idof < vlower.size(); ++idof ) {
                dReal fMin, fMax;
                if( _v0pos[idof] > _v1pos[idof] ) {
                    fMin = _v1pos[idof];
                    fMax = _v0pos[idof];
                }
                else {
                    fMin = _v0pos[idof];
                    fMax = _v1pos[idof];
                }
                vlower[idof] = fMin - 1000;
                vupper[idof] = fMax + 1000;
            }

            PiecewisePolynomials::PolynomialCheckReturn ret = _pikInterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(_v0pos, _v1pos, _v0vel, _v1vel, _v0acc, _v1acc, deltaTime, vlower, vupper, vmaxvel, vmaxaccel, vmaxjerk, _cacheInterpolatedChunks);
            if( 0 ) {
                std::stringstream ssdebug;
                ssdebug << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                ssdebug << "x0Vect=[";
                SerializeValues(ssdebug, _v0pos);
                ssdebug << "]; x1Vect=[";
                SerializeValues(ssdebug, _v1pos);
                ssdebug << "]; v0Vect=[";
                SerializeValues(ssdebug, _v0vel);
                ssdebug << "]; v1Vect=[";
                SerializeValues(ssdebug, _v1vel);
                ssdebug << "]; a0Vect=[";
                SerializeValues(ssdebug, _v0acc);
                ssdebug << "]; a1Vect=[";
                SerializeValues(ssdebug, _v1acc);
                ssdebug << "]; xminVect=[";
                SerializeValues(ssdebug, vlower);
                ssdebug << "]; xmaxVect=[";
                SerializeValues(ssdebug, vupper);
                ssdebug << "]; vmVect=[";
                SerializeValues(ssdebug, vmaxvel);
                ssdebug << "]; amVect=[";
                SerializeValues(ssdebug, vmaxaccel);
                ssdebug << "]; jmVect=[";
                SerializeValues(ssdebug, vmaxjerk);
                ssdebug << "];";
                RAVELOG_INFO_FORMAT("env=%s, ret=%s; %s", GetEnv()->GetNameId()%PiecewisePolynomials::GetPolynomialCheckReturnString(ret)%ssdebug.str());
            }
            if( ret != PiecewisePolynomials::PolynomialCheckReturn::PCR_Normal ) {
                return false;
            }

            bool bIncludeFirstPoint = false;
            int ndof = info->ptraj->GetConfigurationSpecification().GetDOF();
            if( info->ptraj->GetNumWaypoints() == 0 ) {
                bIncludeFirstPoint = true;
                _vtrajpoints.resize(ndof*(_cacheInterpolatedChunks.size() + 1));
            }
            else {
                _vtrajpoints.resize(ndof*_cacheInterpolatedChunks.size());
            }

            std::vector<dReal>::iterator ittargetdata = _vtrajpoints.begin();
            std::vector<dReal> vPos(_v0pos.size()), vVel(_v0vel.size()), vAccel(_v0acc.size()); // TODO: write descriptions
            if( bIncludeFirstPoint ) {
                const PiecewisePolynomials::Chunk& firstChunk = _cacheInterpolatedChunks.front();
                firstChunk.Eval(0, vPos);
                firstChunk.Evald1(0, vVel);
                firstChunk.Evald2(0, vAccel);

                switch( ikType ) {
                case IKP_Transform6D: {
                    // TODO: verify the following computation
                    const dReal t = _v1pos.at(0) > 0 ? (vPos.at(0)/_v1pos.at(0)) : 0;
                    const Vector q = quatSlerp(ikParamPrev.GetTransform6D().rot, ikParam.GetTransform6D().rot, t);
                    const Vector w = vVel[0] * Vector(0, axisAngle.x, axisAngle.y, axisAngle.z);
                    const Vector qd = 0.5*quatMultiply(w, q);
                    const Vector a = vAccel[0] * Vector(0, axisAngle.x, axisAngle.y, axisAngle.z);
                    const Vector qdd = 0.5*(quatMultiply(a, q) + quatMultiply(w, qd));
                    for( size_t idof = 0; idof < 4; ++idof ) {
                        *(ittargetdata + info->posIndex + idof) = q[idof];
                        *(ittargetdata + info->velIndex + idof) = qd[idof];
                        *(ittargetdata + info->accelIndex + idof) = qdd[idof];
                    }
                    break;
                }
                case IKP_Rotation3D: {
                    // TODO:
                }
                case IKP_Translation3D: {
                    // TODO:
                }
                case IKP_TranslationDirection5D: {
                    // TODO:
                }
                case IKP_TranslationXAxisAngleZNorm4D: {
                    // TODO:
                }
                case IKP_TranslationYAxisAngleXNorm4D: {
                    // TODO:
                }
                case IKP_TranslationZAxisAngleYNorm4D: {
                    // TODO:
                }
                case IKP_TranslationZAxisAngle4D: {
                    // TODO:
                }
                default: {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, cubic retimer does not support ikparam type 0x%x"), GetEnv()->GetNameId()%ikParam.GetType(), ORE_InvalidArguments);
                }
                } // end switch

                if( transOffset >= 0 && transIndex >= 0 ) {
                    for( size_t idof = 0; idof < 3; ++idof ) {
                        *(ittargetdata + info->posIndex + transOffset + idof) = vPos.at(transIndex + idof);
                        *(ittargetdata + info->velIndex + transOffset + idof) = vVel.at(transIndex + idof);
                        *(ittargetdata + info->accelIndex + transOffset + idof) = vAccel.at(transIndex + idof);
                    }
                }

                *(ittargetdata + info->timeIndex) = 0;
                *(ittargetdata + info->waypointIndex) = 0;
                ittargetdata += ndof;
            } // end if bIncludeFirstPoint

            FOREACHC(itchunk, _cacheInterpolatedChunks) {
                itchunk->Eval(itchunk->duration, vPos);
                itchunk->Evald1(itchunk->duration, vVel);
                itchunk->Evald2(itchunk->duration, vAccel);

                switch( ikType ) {
                case IKP_Transform6D: {
                    // TODO: verify the following computation
                    const dReal t = _v1pos.at(0) > 0 ? (vPos.at(0)/_v1pos.at(0)) : 0;
                    const Vector q = quatSlerp(ikParamPrev.GetTransform6D().rot, ikParam.GetTransform6D().rot, t);
                    const Vector w = vVel[0] * Vector(0, axisAngle.x, axisAngle.y, axisAngle.z);
                    const Vector qd = 0.5*quatMultiply(w, q);
                    const Vector a = vAccel[0] * Vector(0, axisAngle.x, axisAngle.y, axisAngle.z);
                    const Vector qdd = 0.5*(quatMultiply(a, q) + quatMultiply(w, qd));
                    for( size_t idof = 0; idof < 4; ++idof ) {
                        *(ittargetdata + info->posIndex + idof) = q[idof];
                        *(ittargetdata + info->velIndex + idof) = qd[idof];
                        *(ittargetdata + info->accelIndex + idof) = qdd[idof];
                    }
                    break;
                }
                case IKP_Rotation3D: {
                    // TODO:
                }
                case IKP_Translation3D: {
                    // TODO:
                }
                case IKP_TranslationDirection5D: {
                    // TODO:
                }
                case IKP_TranslationXAxisAngleZNorm4D: {
                    // TODO:
                }
                case IKP_TranslationYAxisAngleXNorm4D: {
                    // TODO:
                }
                case IKP_TranslationZAxisAngleYNorm4D: {
                    // TODO:
                }
                case IKP_TranslationZAxisAngle4D: {
                    // TODO:
                }
                default: {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, cubic retimer does not support ikparam type 0x%x"), GetEnv()->GetNameId()%ikParam.GetType(), ORE_InvalidArguments);
                }
                } // end switch

                if( transOffset >= 0 && transIndex >= 0 ) {
                    for( size_t idof = 0; idof < 3; ++idof ) {
                        *(ittargetdata + info->posIndex + transOffset + idof) = vPos.at(transIndex + idof);
                        *(ittargetdata + info->velIndex + transOffset + idof) = vVel.at(transIndex + idof);
                        *(ittargetdata + info->accelIndex + transOffset + idof) = vAccel.at(transIndex + idof);
                    }
                }

                *(ittargetdata + info->timeIndex) = itchunk->duration;
                *(ittargetdata + info->waypointIndex) = 0;
                ittargetdata += ndof;
            } // end for each itChunk

            if( bIncludeFirstPoint ) {
                _vtrajpoints.at(info->waypointIndex) = 1;
            }

            _vtrajpoints.at(_vtrajpoints.size() - ndof + info->waypointIndex) = 1;
            info->ptraj->Insert(info->ptraj->GetNumWaypoints(), _vtrajpoints);
        } // end if _parameters->_outputaccelchanges
        return true;
    }

    void _WriteTrajectory(TrajectoryBasePtr ptraj, const ConfigurationSpecification& newspec, const std::vector<dReal>& data)
    {
        ptraj->Init(newspec);
        if( _parameters->_outputaccelchanges ) {
            std::list<TrajectoryBaseConstPtr> listTrajectories;
            FOREACH(it, _listGroupInfos) {
                listTrajectories.push_back(boost::dynamic_pointer_cast<CubicGroupInfo>(*it)->ptraj);
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

}; // end class CubicTrajectoryRetimer2

PlannerBasePtr CreateCubicTrajectoryRetimer2(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new CubicTrajectoryRetimer2(penv, sinput));
}

} // end namespace rplanners
