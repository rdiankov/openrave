// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "trajectoryretimer.h"
#include <openrave/planningutils.h>
#include "ParabolicPathSmooth/ParabolicRamp.h"

namespace ParabolicRamp = ParabolicRampInternal;

class ParabolicTrajectoryRetimer : public TrajectoryRetimer
{
public:
    class ParabolicGroupInfo : public GroupInfo
    {
public:
        ParabolicGroupInfo(int degree, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) : GroupInfo(degree, gpos, gvel) {
        }

        TrajectoryBasePtr ptraj;
        int posindex, velindex, waypointindex, timeindex;
    };
    typedef boost::shared_ptr<ParabolicGroupInfo> ParabolicGroupInfoPtr;
    typedef boost::shared_ptr<ParabolicGroupInfo const> ParabolicGroupInfoConstPtr;

    ParabolicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : TrajectoryRetimer(penv,sinput)
    {
        __description = ":Interface Author: Rosen Diankov\n\nSimple parabolic trajectory re-timing while passing through all the waypoints, waypoints will not be modified. This assumes all waypoints have velocity 0 (unless the start and final points are forced). Overwrites the velocities and timestamps of input trajectory.";
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        _trajxmlid = ptraj->GetXMLId();
        return TrajectoryRetimer::PlanPath(ptraj);
    }

protected:
    GroupInfoPtr CreateGroupInfo(int degree, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) {
        ParabolicGroupInfoPtr g(new ParabolicGroupInfo(degree, gpos, gvel));
        ConfigurationSpecification spec;
        g->posindex = spec.AddGroup(gpos.name, gpos.dof, "quadratic");
        g->velindex = spec.AddGroup(gvel.name, gvel.dof, "linear");
        g->waypointindex = spec.AddGroup("iswaypoint", 1, "next");
        g->timeindex = spec.AddDeltaTimeGroup();
        g->ptraj = RaveCreateTrajectory(GetEnv(),_trajxmlid);
        g->ptraj->Init(spec);
        return g;
    }

    bool _SupportInterpolation() {
        if( _parameters->_interpolation.size() == 0 ) {
            _parameters->_interpolation = "quadratic";
            return true;
        }
        else {
            return _parameters->_interpolation == "quadratic";
        }
    }

    dReal _ComputeMinimumTimeJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) {
        _v0pos.resize(info->gpos.dof);
        _v1pos.resize(info->gpos.dof);
        for(int i = 0; i < info->gpos.dof; ++i) {
            _v0pos[i] = *(itdataprev+info->gpos.offset+i);
            _v1pos[i] = _v0pos[i] + *(itorgdiff+info->orgposoffset+i);
        }
        _v0vel.resize(info->gvel.dof);
        _v1vel.resize(info->gvel.dof);
        for(int i = 0; i < info->gvel.dof; ++i) {
            _v0vel[i] = *(itdataprev+info->gvel.offset+i);
            if( bUseEndVelocity ) {
                _v1vel[i] = *(itdata+info->gvel.offset+i);
            }
            else {
                _v1vel[i] = 0;
            }
        }
        _ramps.resize(info->gpos.dof);
        dReal mintime = ParabolicRamp::SolveMinTimeBounded(_v0pos, _v0vel, _v1pos, _v1vel, info->_vConfigAccelerationLimit, info->_vConfigVelocityLimit, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps,_parameters->_multidofinterp);
#ifdef _DEBUG
        if( mintime < 0 && IS_DEBUGLEVEL(Level_Verbose) ) {
            // do again for debugging reproduction
            mintime = ParabolicRamp::SolveMinTimeBounded(_v0pos, _v0vel, _v1pos, _v1vel, info->_vConfigAccelerationLimit, info->_vConfigVelocityLimit, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps,_parameters->_multidofinterp);
        }
#endif
        return mintime;
    }

    void _ComputeVelocitiesJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        if( info->orgveloffset >= 0 ) {
            for(int i=0; i < info->gvel.dof; ++i) {
                *(itdata+info->gvel.offset+i) = *(itorgdiff+info->orgveloffset+i);
            }
        }
        else {
            for(int i=0; i < info->gvel.dof; ++i) {
                *(itdata+info->gvel.offset+i) = 0;
            }
        }
    }

    bool _CheckVelocitiesJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        dReal deltatime = *(itdata+_timeoffset);
        for(int i=0; i < info->gvel.dof; ++i) {
            dReal fvel = *(itdata+info->gvel.offset+i);
            if( RaveFabs(fvel) > info->_vConfigVelocityLimit.at(i) + ParabolicRamp::EpsilonV ) {
                return false;
            }
            dReal diff = RaveFabs(*(itdataprev+info->gvel.offset+i)-fvel);
            if( RaveFabs(diff) > info->_vConfigAccelerationLimit.at(i) * deltatime + ParabolicRamp::EpsilonA ) {
                return false;
            }
        }
        return true;
    }

    bool _WriteJointValues(GroupInfoConstPtr inforaw, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
        ParabolicGroupInfoConstPtr info = boost::dynamic_pointer_cast<ParabolicGroupInfo const>(inforaw);
        if( _parameters->_outputaccelchanges ) {
            _v0pos.resize(info->gpos.dof);
            _v1pos.resize(info->gpos.dof);
            for(int i = 0; i < info->gpos.dof; ++i) {
                _v0pos[i] = *(itdataprev+info->gpos.offset+i);
                _v1pos[i] = _v0pos[i] + *(itorgdiff+info->orgposoffset+i);
            }
            _v0vel.resize(info->gvel.dof);
            _v1vel.resize(info->gvel.dof);
            for(int i = 0; i < info->gvel.dof; ++i) {
                _v0vel[i] = *(itdataprev+info->gvel.offset+i);
                _v1vel[i] = *(itdata+info->gvel.offset+i);
            }
            _ramps.resize(info->gpos.dof);
            dReal deltatime = *(itdata+_timeoffset);
            if( deltatime == 0 ) {
                if( info->ptraj->GetNumWaypoints() == 0 ) {
                    // add the first point, copy all the data rather than just this group
                    _vtrajpoints.resize(info->ptraj->GetConfigurationSpecification().GetDOF());
                    std::copy(itdata,itdata+_vtrajpoints.size(),_vtrajpoints.begin());
                    _vtrajpoints.at(info->waypointindex) = 1;
                    info->ptraj->Insert(info->ptraj->GetNumWaypoints(),_vtrajpoints);
                }
                return true;
            }
            OPENRAVE_ASSERT_OP(deltatime,>,0);
            if( deltatime < ParabolicRamp::EpsilonT ) {
                RAVELOG_WARN(str(boost::format("delta time is really ill-conditioned: %e")%deltatime));
            }

            bool success = ParabolicRamp::SolveAccelBounded(_v0pos, _v0vel, _v1pos, _v1vel, deltatime, info->_vConfigAccelerationLimit, info->_vConfigVelocityLimit, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps, _parameters->_multidofinterp);
            if( !success ) {
#ifdef _DEBUG
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    // for debugging
                    success = ParabolicRamp::SolveAccelBounded(_v0pos, _v0vel, _v1pos, _v1vel, deltatime, info->_vConfigAccelerationLimit, info->_vConfigVelocityLimit, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps, _parameters->_multidofinterp);
                }
#endif
                return false;
            }

            vector<dReal> vswitchtimes;
            if( info->ptraj->GetNumWaypoints() == 0 ) {
                // add initial point of the segment only for the first point in the trajectory
                vswitchtimes.push_back(0);
            }
            vswitchtimes.push_back(deltatime);
            for(size_t i=0; i < _ramps.size(); ++i) {
                std::vector<ParabolicRamp::ParabolicRamp1D>& ramp = _ramps[i];
                dReal maxaccel = info->_vConfigAccelerationLimit[i]+ParabolicRamp::EpsilonA;
                dReal maxvel = info->_vConfigVelocityLimit[i]+ParabolicRamp::EpsilonV;
                dReal curtime = 0;
                for(size_t j=0; j < ramp.size(); j++) {
                    if(RaveFabs(ramp[j].a1) > maxaccel+1e-5 || RaveFabs(ramp[j].a2) > maxaccel+1e-5 || RaveFabs(ramp[j].v) > maxvel+1e-5) {
                        RAVELOG_WARN(str(boost::format("ramp violates limits: %f>%f || %f>%f || %f>%f")%RaveFabs(ramp[j].a1)%maxaccel%RaveFabs(ramp[j].a2)%maxaccel%RaveFabs(ramp[j].v)%maxvel));
                        return false;
                    }

                    vector<dReal>::iterator it;
                    dReal tswitch1 = curtime+ramp[j].tswitch1;
                    dReal tswitch2 = curtime+ramp[j].tswitch2;
                    if( tswitch1 != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),tswitch1);
                        if( *it != tswitch1) {
                            vswitchtimes.insert(it,tswitch1);
                        }
                    }
                    if( tswitch1 != tswitch2 && tswitch2 != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),tswitch2);
                        if( *it != tswitch2 ) {
                            vswitchtimes.insert(it,tswitch2);
                        }
                    }

                    curtime += ramp[j].ttotal;
                }
            }

            int dof = info->ptraj->GetConfigurationSpecification().GetDOF();
            _vtrajpoints.resize(dof*vswitchtimes.size());
            vector<dReal>::iterator ittargetdata = _vtrajpoints.begin();
            dReal prevtime = 0;
            FOREACHC(itswitchtime, vswitchtimes) {
                for(int j = 0; j < info->gpos.dof; ++j) {
                    dReal curtime = *itswitchtime;
                    for(size_t iramp = 0; iramp < _ramps.at(j).size(); ++iramp) {
                        if( curtime > _ramps[j][iramp].ttotal ) {
                            curtime -= _ramps[j][iramp].ttotal;
                        }
                        else {
                            *(ittargetdata + info->posindex + j) = _ramps[j][iramp].Evaluate(curtime);
                            *(ittargetdata + info->velindex + j) = _ramps[j][iramp].Derivative(curtime);
                            break;
                        }
                    }
                }

                *(ittargetdata+info->timeindex) = *itswitchtime-prevtime;
                *(ittargetdata+info->waypointindex) = 0;
                ittargetdata += dof;
                prevtime = *itswitchtime;
            }
            if( vswitchtimes.front() == 0 ) {
                _vtrajpoints.at(info->waypointindex) = 1;
            }
            _vtrajpoints.at(_vtrajpoints.size()-dof+info->waypointindex) = 1;
            info->ptraj->Insert(info->ptraj->GetNumWaypoints(),_vtrajpoints);
        }
        return true;
    }

    dReal _ComputeMinimumTimeAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_ComputeMinimumTimeAffine not implemented", ORE_NotImplemented);
    }

    void _ComputeVelocitiesAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_ComputeVelocitiesAffine not implemented", ORE_NotImplemented);
    }

    bool _CheckVelocitiesAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_CheckVelocitiesAffine not implemented", ORE_NotImplemented);
    }

    bool _WriteAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_WriteAffine not implemented", ORE_NotImplemented);
    }

    // speed of rotations is always the speed of the angle along the minimum rotation
    // speed of translations is always the combined xyz speed
    // TODO ParabolicRamp::SolveMinTimeBounded can only consider max vel/accel per dimension rather than combined together....
    dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) {
        ParabolicRamp::ParabolicRamp1D ramp;
        IkParameterization ikparamprev, ikparam;
        ikparamprev.Set(itdataprev+info->gpos.offset,iktype);
        ikparam.Set(itdata+info->gpos.offset,iktype);
        Vector transdelta;
        int transoffset = -1;
        dReal mintime = -1;

        switch(iktype) {
        case IKP_Transform6D: {
            dReal angledelta = 2.0*RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetTransform6D().rot.dot(ikparam.GetTransform6D().rot))));
            Vector angularvelocityprev = quatMultiply(Vector(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2), *(itdataprev+info->gvel.offset+3)),quatInverse(ikparamprev.GetTransform6D().rot))*2;
            dReal anglevelprev = RaveSqrt(utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w));
            Vector angularvelocity = quatMultiply(Vector(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2), *(itdata+info->gvel.offset+3)), quatInverse(ikparamprev.GetTransform6D().rot))*2;
            dReal anglevel = RaveSqrt(utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w));
            if( !ParabolicRamp::SolveMinTimeBounded(0,anglevelprev,angledelta,anglevel, info->_vConfigAccelerationLimit.at(0), info->_vConfigVelocityLimit.at(0), -1000,angledelta+1000, ramp) ) {
                RAVELOG_WARN("failed solving mintime for angles\n");
                return -1;
            }
            mintime = ramp.EndTime();
            transdelta = ikparam.GetTransform6D().trans-ikparamprev.GetTransform6D().trans;
            transoffset = 4;
            break;
        }
        case IKP_Rotation3D: {
            dReal angledelta = 2.0*RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetRotation3D().dot(ikparam.GetRotation3D()))));
            Vector angularvelocityprev = quatMultiply(Vector(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2), *(itdataprev+info->gvel.offset+3)),quatInverse(ikparamprev.GetRotation3D()))*2;
            dReal anglevelprev = RaveSqrt(utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w));
            Vector angularvelocity = quatMultiply(Vector(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2), *(itdata+info->gvel.offset+3)), quatInverse(ikparamprev.GetRotation3D()))*2;
            dReal anglevel = RaveSqrt(utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w));
            if( !ParabolicRamp::SolveMinTimeBounded(0,anglevelprev,angledelta,anglevel, info->_vConfigAccelerationLimit.at(0), info->_vConfigVelocityLimit.at(0), -1000,angledelta+1000, ramp) ) {
                RAVELOG_WARN("failed solving mintime for angles\n");
                return -1;
            }
            mintime = ramp.EndTime();
            break;
        }
        case IKP_Translation3D:
            transdelta = ikparam.GetTranslation3D()-ikparamprev.GetTranslation3D();
            transoffset = 0;
            break;
        case IKP_TranslationDirection5D: {
            dReal angledelta = RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetTranslationDirection5D().dir.dot3(ikparam.GetTranslationDirection5D().dir))));
            Vector angularvelocityprev(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2));
            dReal anglevelprev = RaveSqrt(utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w));
            Vector angularvelocity(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2));
            dReal anglevel = RaveSqrt(utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w));
            if( !ParabolicRamp::SolveMinTimeBounded(0,anglevelprev,angledelta,anglevel, info->_vConfigAccelerationLimit.at(0), info->_vConfigVelocityLimit.at(0), -1000,angledelta+1000, ramp) ) {
                RAVELOG_WARN("failed solving mintime for angles\n");
                return -1;
            }
            mintime = ramp.EndTime();
            transdelta = ikparam.GetTranslationDirection5D().pos-ikparamprev.GetTranslationDirection5D().pos;
            transoffset = 3;
            break;
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            dReal angledelta = utils::SubtractCircularAngle(ikparam.GetTranslationXAxisAngleZNorm4D().second,ikparamprev.GetTranslationXAxisAngleZNorm4D().second);
            if( !ParabolicRamp::SolveMinTimeBounded(0,*(itdataprev+info->gvel.offset+0),angledelta,*(itdata+info->gvel.offset+0), info->_vConfigAccelerationLimit.at(0), info->_vConfigVelocityLimit.at(0), -1000,angledelta+1000, ramp) ) {
                RAVELOG_WARN("failed solving mintime for angles\n");
                return false;
            }
            mintime = ramp.EndTime();
            transdelta = ikparam.GetTranslationXAxisAngleZNorm4D().first-ikparamprev.GetTranslationXAxisAngleZNorm4D().first;
            transoffset = 1;
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", ikparam.GetType(),ORE_InvalidArguments);
        }

        if( transoffset >= 0 ) {
            dReal xyzdelta = RaveSqrt(transdelta.lengthsqr3());
            dReal xyzvelprev = RaveSqrt(utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+0)) + utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+1)) + utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+2)));
            dReal xyzvel = RaveSqrt(utils::Sqr(*(itdata+info->gvel.offset+transoffset+0)) + utils::Sqr(*(itdata+info->gvel.offset+transoffset+1)) + utils::Sqr(*(itdata+info->gvel.offset+transoffset+2)));
            if( !ParabolicRamp::SolveMinTimeBounded(0,xyzvelprev,xyzdelta,xyzvel, info->_vConfigAccelerationLimit.at(transoffset), info->_vConfigVelocityLimit.at(transoffset), -1000,xyzdelta+1000, ramp) ) {
                RAVELOG_WARN("failed solving mintime for xyz\n");
                return -1;
            }
            dReal xyzmintime = ramp.EndTime();
            mintime = max(mintime,xyzmintime);
        }
        return mintime;
    }

    void _ComputeVelocitiesIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        if( info->orgveloffset >= 0 ) {
            for(int i=0; i < info->gvel.dof; ++i) {
                *(itdata+info->gvel.offset+i) = *(itorgdiff+info->orgveloffset+i);
            }
        }
        else {
            for(int i=0; i < info->gvel.dof; ++i) {
                *(itdata+info->gvel.offset+i) = 0;
            }
        }
    }

    bool _CheckVelocitiesIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        dReal deltatime = *(itdata+_timeoffset);
        int transoffset = -1;
        IkParameterization ikparamprev;
        switch(iktype) {
        case IKP_Transform6D: {
            ikparamprev.Set(itdataprev+info->gpos.offset,iktype);
            Vector angularvelocityprev = quatMultiply(Vector(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2), *(itdataprev+info->gvel.offset+3)),quatInverse(ikparamprev.GetTransform6D().rot))*2;
            dReal fvelprev2 = utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w);
            Vector angularvelocity = quatMultiply(Vector(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2), *(itdata+info->gvel.offset+3)), quatInverse(ikparamprev.GetTransform6D().rot))*2;
            dReal fvel2 = utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w);
            if( fvel2 > utils::Sqr(info->_vConfigVelocityLimit.at(0)) + ParabolicRamp::EpsilonV ) {
                return false;
            }
            if( fvel2+fvelprev2-2*RaveSqrt(fvel2*fvelprev2) > info->_vConfigAccelerationLimit.at(0)*deltatime + ParabolicRamp::EpsilonA ) {
                return false;
            }
            transoffset = 4;
            break;
        }
        case IKP_Rotation3D: {
            ikparamprev.Set(itdataprev+info->gpos.offset,iktype);
            Vector angularvelocityprev = quatMultiply(Vector(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2), *(itdataprev+info->gvel.offset+3)),quatInverse(ikparamprev.GetRotation3D()))*2;
            dReal fvelprev2 = utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w);
            Vector angularvelocity = quatMultiply(Vector(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2), *(itdata+info->gvel.offset+3)), quatInverse(ikparamprev.GetRotation3D()))*2;
            dReal fvel2 = utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w);
            if( fvel2 > utils::Sqr(info->_vConfigVelocityLimit.at(0)) + ParabolicRamp::EpsilonV ) {
                return false;
            }
            if( fvel2+fvelprev2-2*RaveSqrt(fvel2*fvelprev2) > info->_vConfigAccelerationLimit.at(0)*deltatime + ParabolicRamp::EpsilonA ) {
                return false;
            }
            break;
        }
        case IKP_Translation3D:
            transoffset = 0;
            break;
        case IKP_TranslationDirection5D: {
            Vector angularvelocityprev(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2));
            dReal fvelprev2 = utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w);
            Vector angularvelocity(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2));
            dReal fvel2 = utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w);
            if( fvel2 > utils::Sqr(info->_vConfigVelocityLimit.at(0)) + ParabolicRamp::EpsilonV ) {
                return false;
            }
            if( fvel2+fvelprev2-2*RaveSqrt(fvel2*fvelprev2) > info->_vConfigAccelerationLimit.at(0)*deltatime + ParabolicRamp::EpsilonA ) {
                return false;
            }
            transoffset = 3;
            break;
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            dReal fvelprev = *(itdataprev+info->gvel.offset+0);
            dReal fvel = *(itdata+info->gvel.offset+0);
            if( RaveFabs(fvel) > info->_vConfigVelocityLimit.at(0) + ParabolicRamp::EpsilonV ) {
                return false;
            }
            if( RaveFabs(fvel-fvelprev) > info->_vConfigAccelerationLimit.at(0)*deltatime + ParabolicRamp::EpsilonA ) {
                return false;
            }
            transoffset = 1;
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", iktype,ORE_InvalidArguments);
        }
        if( transoffset >= 0 ) {
            dReal fvelprev2 = utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+0)) + utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+1)) + utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+2));
            dReal fvel2 = utils::Sqr(*(itdata+info->gvel.offset+transoffset+0)) + utils::Sqr(*(itdata+info->gvel.offset+transoffset+1)) + utils::Sqr(*(itdata+info->gvel.offset+transoffset+2));

            if( fvel2 > utils::Sqr(info->_vConfigVelocityLimit.at(transoffset)) + ParabolicRamp::EpsilonV ) {
                return false;
            }
            if( fvel2+fvelprev2-2*RaveSqrt(fvel2*fvelprev2) > info->_vConfigAccelerationLimit.at(transoffset)*deltatime + ParabolicRamp::EpsilonA ) {
                return false;
            }
        }
        return true;
    }

    bool _WriteIk(GroupInfoConstPtr inforaw, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
        ParabolicGroupInfoConstPtr info = boost::dynamic_pointer_cast<ParabolicGroupInfo const>(inforaw);
        if( _parameters->_outputaccelchanges ) {
            dReal deltatime = *(itdata+_timeoffset);
            if( deltatime == 0 ) {
                if( info->ptraj->GetNumWaypoints() == 0 ) {
                    // add the first point, copy all the data rather than just this group
                    _vtrajpoints.resize(info->ptraj->GetConfigurationSpecification().GetDOF());
                    std::copy(itdata,itdata+_vtrajpoints.size(),_vtrajpoints.begin());
                    _vtrajpoints.at(info->waypointindex) = 1;
                    info->ptraj->Insert(info->ptraj->GetNumWaypoints(),_vtrajpoints);
                }
                return true;
            }
            OPENRAVE_ASSERT_OP(deltatime,>,0);
            if( deltatime < ParabolicRamp::EpsilonT ) {
                RAVELOG_WARN(str(boost::format("delta time is really ill-conditioned: %e")%deltatime));
            }

            IkParameterization ikparamprev, ikparam;
            ikparamprev.Set(itdataprev+info->gpos.offset,iktype);
            ikparam.Set(itdata+info->gpos.offset,iktype);
            _v0pos.resize(0); _v0vel.resize(0); _v1pos.resize(0); _v1vel.resize(0);
            Vector transdelta, axisangle;
            int transoffset = -1, transindex = -1;
            vector<dReal> vmaxvel, vmaxaccel, vlower, vupper;

            switch(iktype) {
            case IKP_Transform6D: {
                _v0pos.resize(2); _v0vel.resize(2); _v1pos.resize(2); _v1vel.resize(2); vmaxvel.resize(2); vmaxaccel.resize(2);
                axisangle = axisAngleFromQuat(quatMultiply(quatInverse(ikparamprev.GetTransform6D().rot), ikparam.GetTransform6D().rot));
                if( axisangle.lengthsqr3() > g_fEpsilon ) {
                    axisangle.normalize3();
                }
                _v0pos[0] = 0;
                _v1pos[0] = 2.0*RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetTransform6D().rot.dot(ikparam.GetTransform6D().rot))));
                Vector angularvelocityprev = quatMultiply(Vector(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2), *(itdataprev+info->gvel.offset+3)),quatInverse(ikparamprev.GetTransform6D().rot))*2;
                _v0vel[0] = RaveSqrt(utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w));
                Vector angularvelocity = quatMultiply(Vector(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2), *(itdata+info->gvel.offset+3)), quatInverse(ikparamprev.GetTransform6D().rot))*2;
                _v1vel[0] = RaveSqrt(utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w));
                vmaxvel[0] = info->_vConfigVelocityLimit.at(0);
                vmaxaccel[0] = info->_vConfigAccelerationLimit.at(0);
                transdelta = ikparam.GetTransform6D().trans-ikparamprev.GetTransform6D().trans;
                transoffset = 4;
                transindex = 1;
                break;
            }
            case IKP_Rotation3D: {
                _v0pos.resize(1); _v0vel.resize(1); _v1pos.resize(1); _v1vel.resize(1); vmaxvel.resize(1); vmaxaccel.resize(1);
                axisangle = axisAngleFromQuat(quatMultiply(quatInverse(ikparamprev.GetRotation3D()), ikparam.GetRotation3D()));
                if( axisangle.lengthsqr3() > g_fEpsilon ) {
                    axisangle.normalize3();
                }
                _v0pos[0] = 0;
                _v1pos[0] = 2.0*RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetRotation3D().dot(ikparam.GetRotation3D()))));
                Vector angularvelocityprev = quatMultiply(Vector(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2), *(itdataprev+info->gvel.offset+3)),quatInverse(ikparamprev.GetRotation3D()))*2;
                _v0vel[0] = RaveSqrt(utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w));
                Vector angularvelocity = quatMultiply(Vector(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2), *(itdata+info->gvel.offset+3)), quatInverse(ikparamprev.GetRotation3D()))*2;
                _v1vel[0] = RaveSqrt(utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w));
                vmaxvel[0] = info->_vConfigVelocityLimit.at(0);
                vmaxaccel[0] = info->_vConfigAccelerationLimit.at(0);
                break;
            }
            case IKP_Translation3D:
                _v0pos.resize(1); _v0vel.resize(1); _v1pos.resize(1); _v1vel.resize(1); vmaxvel.resize(1); vmaxaccel.resize(1);
                transdelta = ikparam.GetTranslation3D()-ikparamprev.GetTranslation3D();
                transoffset = 0;
                transindex = 0;
                break;
            case IKP_TranslationDirection5D: {
                _v0pos.resize(2); _v0vel.resize(2); _v1pos.resize(2); _v1vel.resize(2); vmaxvel.resize(2); vmaxaccel.resize(2);
                axisangle = ikparamprev.GetTranslationDirection5D().dir.cross(ikparam.GetTranslationDirection5D().dir);
                if( axisangle.lengthsqr3() > g_fEpsilon ) {
                    axisangle.normalize3();
                }
                _v0pos[0] = 0;
                _v1pos[0] = RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetTranslationDirection5D().dir.dot3(ikparam.GetTranslationDirection5D().dir))));
                Vector angularvelocityprev(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2));
                _v0vel[0] = RaveSqrt(utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w));
                Vector angularvelocity(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2));
                _v1vel[0] = RaveSqrt(utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w));
                vmaxvel[0] = info->_vConfigVelocityLimit.at(0);
                vmaxaccel[0] = info->_vConfigAccelerationLimit.at(0);
                transdelta = ikparam.GetTranslationDirection5D().pos-ikparamprev.GetTranslationDirection5D().pos;
                transoffset = 3;
                transindex = 1;
                break;
            }
            case IKP_TranslationXAxisAngleZNorm4D: {
                _v0pos.resize(2); _v0vel.resize(2); _v1pos.resize(2); _v1vel.resize(2); vmaxvel.resize(2); vmaxaccel.resize(2);
                _v0pos[0] = 0;
                _v1pos[0] = utils::SubtractCircularAngle(ikparam.GetTranslationXAxisAngleZNorm4D().second,ikparamprev.GetTranslationXAxisAngleZNorm4D().second);
                _v0vel[0] = *(itdataprev+info->gvel.offset+0);
                _v1vel[0] = *(itdata+info->gvel.offset+0);
                vmaxvel[0] = info->_vConfigVelocityLimit.at(0);
                vmaxaccel[0] = info->_vConfigAccelerationLimit.at(0);
                transdelta = ikparam.GetTranslationXAxisAngleZNorm4D().first-ikparamprev.GetTranslationXAxisAngleZNorm4D().first;
                transoffset = 1;
                transindex = 1;
                break;
            }
            default:
                throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", ikparam.GetType(),ORE_InvalidArguments);
            }

            if( transoffset >= 0 ) {
                _v0pos.at(transindex) = 0;
                _v1pos.at(transindex) = RaveSqrt(transdelta.lengthsqr3());
                _v0vel.at(transindex) = RaveSqrt(utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+0)) + utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+1)) + utils::Sqr(*(itdataprev+info->gvel.offset+transoffset+2)));
                _v1vel.at(transindex) = RaveSqrt(utils::Sqr(*(itdata+info->gvel.offset+transoffset+0)) + utils::Sqr(*(itdata+info->gvel.offset+transoffset+1)) + utils::Sqr(*(itdata+info->gvel.offset+transoffset+2)));
                vmaxvel.at(transindex) = info->_vConfigVelocityLimit.at(transoffset);
                vmaxaccel.at(transindex) = info->_vConfigAccelerationLimit.at(transoffset);
            }

            _ramps.resize(_v0pos.size());
            vlower.resize(_v0pos.size());
            vupper.resize(_v0pos.size());
            for(size_t i = 0; i < vlower.size(); ++i) {
                vlower[i] = -1000;
                vupper[i] = 1000+_v1pos[i];
            }

            bool success = ParabolicRamp::SolveAccelBounded(_v0pos, _v0vel, _v1pos, _v1vel, deltatime, vmaxaccel, vmaxvel, vlower, vupper, _ramps,_parameters->_multidofinterp);
            if( !success ) {
#ifdef _DEBUG
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    success = ParabolicRamp::SolveAccelBounded(_v0pos, _v0vel, _v1pos, _v1vel, deltatime, vmaxaccel, vmaxvel, vlower, vupper, _ramps,_parameters->_multidofinterp);
                }
#endif
                return false;
            }

            vector<dReal> vswitchtimes;
            if( info->ptraj->GetNumWaypoints() == 0 ) {
                // add initial point of the segment only for the first point in the trajectory
                vswitchtimes.push_back(0);
            }
            vswitchtimes.push_back(deltatime);
            for(size_t i=0; i < _ramps.size(); ++i) {
                std::vector<ParabolicRamp::ParabolicRamp1D>& ramp = _ramps[i];
                //dReal maxaccel = info->_vConfigAccelerationLimit[i]+ParabolicRamp::EpsilonA;
                dReal maxvel = vmaxvel[i]+ParabolicRamp::EpsilonV;
                dReal curtime = 0;
                for(size_t j=0; j < ramp.size(); j++) {
                    if(RaveFabs(ramp[j].v) > maxvel+1e-5) {
                        RAVELOG_WARN(str(boost::format("ramp violates limits: %f>%f")%RaveFabs(ramp[j].v)%maxvel));
                        return false;
                    }
                    // why is this commented out?
//                    if( RaveFabs(ramp[j].a1) > maxaccel+1e-5 || RaveFabs(ramp[j].a2) > maxaccel+1e-5 ) {
//                        throw OPENRAVE_EXCEPTION_FORMAT("ramp violates limits: %f>%f || %f>%f",RaveFabs(ramp[j].a1)%maxaccel%RaveFabs(ramp[j].a2)%maxaccel, ORE_InconsistentConstraints);
//                    }

                    vector<dReal>::iterator it;
                    dReal tswitch1 = curtime+ramp[j].tswitch1;
                    dReal tswitch2 = curtime+ramp[j].tswitch2;
                    if( tswitch1 != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),tswitch1);
                        if( *it != tswitch1) {
                            vswitchtimes.insert(it,tswitch1);
                        }
                    }
                    if( tswitch1 != tswitch2 && tswitch2 != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),tswitch2);
                        if( *it != tswitch2 ) {
                            vswitchtimes.insert(it,tswitch2);
                        }
                    }

                    curtime += ramp[j].ttotal;
                }
            }

            int dof = info->ptraj->GetConfigurationSpecification().GetDOF();
            _vtrajpoints.resize(dof*vswitchtimes.size());
            vector<dReal>::iterator ittargetdata = _vtrajpoints.begin();
            dReal prevtime = 0;
            vector<dReal> vpos(_v0pos.size()), vvel(_v0pos.size());
            FOREACHC(itswitchtime, vswitchtimes) {
                for(size_t j = 0; j < _ramps.size(); ++j) {
                    dReal curtime = *itswitchtime;
                    for(size_t iramp = 0; iramp < _ramps.at(j).size(); ++iramp) {
                        if( curtime > _ramps[j][iramp].ttotal ) {
                            curtime -= _ramps[j][iramp].ttotal;
                        }
                        else {
                            vpos[j] = _ramps[j][iramp].Evaluate(curtime);
                            vvel[j] = _ramps[j][iramp].Derivative(curtime);
                            break;
                        }
                    }
                }

                Vector translation;
                // fill (ittargetdata + info->posindex) and (ittargetdata + info->velindex)
                switch(iktype) {
                case IKP_Transform6D: {
                    dReal t = _v1pos.at(0) > 0 ? (vpos.at(0)/_v1pos.at(0)) : 0;
                    Vector q = quatSlerp(ikparamprev.GetTransform6D().rot, ikparam.GetTransform6D().rot, t);
                    // quaternion derivative of slerp, note that the angular velocity is wrong
                    // qslerpdot = 0.5 * angularvel * q
                    // oldangularvel = 2*norm(qslerpdot * q^-1)
                    // vvel[0]/oldangularvel * qdot
                    Vector qslerpdot = quatMultiply(q,Vector(0,axisangle.x,axisangle.y,axisangle.z));
                    //Vector angularvel = quatMultiply(qslerpdot, quatInverse(q));
                    //BOOST_ASSERT(RaveFabs(angularvel[0]) < 100*g_fEpsilon);
                    Vector qvel = qslerpdot * vvel.at(0) * 0.5; // axisangle already normalized so just need to multiply by vvel
                    for(size_t j = 0; j < 4; ++j) {
                        *(ittargetdata + info->posindex + j) = q[j];
                        *(ittargetdata + info->velindex + j) = qvel[j];
                    }
                    translation = ikparamprev.GetTransform6D().trans;
                    break;
                }
                case IKP_Rotation3D: {
                    dReal t = _v1pos.at(0) > 0 ? (vpos.at(0)/_v1pos.at(0)) : 0;
                    Vector q = quatSlerp(ikparamprev.GetRotation3D(), ikparam.GetRotation3D(), t);
                    // quaternion derivative of slerp, note that the angular velocity is wrong
                    // qslerpdot = 0.5 * angularvel * q
                    // oldangularvel = 2*norm(qslerpdot * q^-1)
                    // vvel[0]/oldangularvel * qdot
                    Vector qslerpdot = quatMultiply(q,Vector(0,axisangle.x,axisangle.y,axisangle.z));
                    //Vector angularvel = quatMultiply(qslerpdot, quatInverse(q));
                    //BOOST_ASSERT(RaveFabs(angularvel[0]) < 100*g_fEpsilon);
                    Vector qvel = qslerpdot * vvel.at(0) * 0.5; // axisangle already normalized so just need to multiply by vvel
                    for(size_t j = 0; j < 4; ++j) {
                        *(ittargetdata + info->posindex + j) = q[j];
                        *(ittargetdata + info->velindex + j) = qvel[j];
                    }
                    break;
                }
                case IKP_Translation3D: {
                    translation = ikparamprev.GetTranslation3D();
                    break;
                }
                case IKP_TranslationDirection5D: {
                    Vector angularvel = axisangle*vvel.at(0);
                    Vector newdir =  quatRotate(quatFromAxisAngle(axisangle*vpos.at(0)),ikparamprev.GetTranslationDirection5D().dir);
                    for(size_t j = 0; j < 3; ++j) {
                        *(ittargetdata + info->posindex + j) = newdir[j];
                        *(ittargetdata + info->velindex + j) = angularvel[j];
                    }
                    translation = ikparamprev.GetTranslationDirection5D().pos;
                    break;
                }
                case IKP_TranslationXAxisAngleZNorm4D: {
                    *(ittargetdata + info->posindex + 0) = ikparamprev.GetTranslationXAxisAngleZNorm4D().second + vpos.at(0);
                    *(ittargetdata + info->velindex + 0) = vvel.at(0);
                    translation = ikparamprev.GetTranslationXAxisAngleZNorm4D().first;
                    break;
                }
                default:
                    throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", ikparam.GetType(),ORE_InvalidArguments);
                }
                if( transoffset >= 0 ) {
                    vpos.at(transindex);
                    dReal t = _v1pos.at(transindex) > 0 ? (vpos.at(transindex)/_v1pos.at(transindex)) : 0;
                    dReal fvelocity = vvel.at(transindex)/_v1pos.at(transindex);
                    for(size_t j = 0; j < 3; ++j) {
                        *(ittargetdata + info->posindex + transoffset + j) = transdelta[j]*t + translation[j];
                        *(ittargetdata + info->velindex + transoffset + j) = transdelta[j] * fvelocity;
                    }
                }

                *(ittargetdata+info->timeindex) = *itswitchtime-prevtime;
                *(ittargetdata+info->waypointindex) = 0;
                ittargetdata += dof;
                prevtime = *itswitchtime;
            }
            if( vswitchtimes.front() == 0 ) {
                _vtrajpoints.at(info->waypointindex) = 1;
            }
            _vtrajpoints.at(_vtrajpoints.size()-dof+info->waypointindex) = 1;
            info->ptraj->Insert(info->ptraj->GetNumWaypoints(),_vtrajpoints);
        }
        return true;
    }

    void _WriteTrajectory(TrajectoryBasePtr ptraj, const ConfigurationSpecification& newspec, const std::vector<dReal>& data) {
        ptraj->Init(newspec);
        if( _parameters->_outputaccelchanges ) {
            std::list<TrajectoryBaseConstPtr> listtrajectories;
            FOREACH(it,_listgroupinfo) {
                listtrajectories.push_back(boost::dynamic_pointer_cast<ParabolicGroupInfo>(*it)->ptraj);
            }
            TrajectoryBasePtr pmergedtraj = planningutils::MergeTrajectories(listtrajectories);
            if( pmergedtraj->GetNumWaypoints() > 0 ) {
                vector<dReal> vmergeddata;
                pmergedtraj->GetWaypoints(0,pmergedtraj->GetNumWaypoints(),vmergeddata,newspec);
                ptraj->Insert(0,vmergeddata);
            }
        }
        else {
            ptraj->Insert(0,data);
        }
    }

    string _trajxmlid;
    ParabolicRamp::Vector _v0pos, _v0vel, _v1pos, _v1vel;
    vector<dReal> _vtrajpoints;
    std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > _ramps;
};

PlannerBasePtr CreateParabolicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new ParabolicTrajectoryRetimer(penv, sinput));
}
