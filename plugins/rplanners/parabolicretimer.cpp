// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov <rosen.diankov@gmail.com>
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

namespace rplanners {

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

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        _trajxmlid = ptraj->GetXMLId();
        return TrajectoryRetimer::PlanPath(ptraj, planningoptions);
    }

protected:
    GroupInfoPtr CreateGroupInfo(int degree, const ConfigurationSpecification& origspec, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) {
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

    void ResetCachedGroupInfo(GroupInfoPtr g)
    {
        // remove all the points
        ParabolicGroupInfoPtr parabolicgroup = boost::dynamic_pointer_cast<ParabolicGroupInfo>(g);
        if( parabolicgroup->ptraj->GetNumWaypoints() > 0 ) {
            parabolicgroup->ptraj->Remove(0, parabolicgroup->ptraj->GetNumWaypoints());
        }
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
        dReal mintime = -1;

        // succeeded, check if manipulator constraints are in effect
        if( _bmanipconstraints && !!_manipconstraintchecker ) {
            // only possible to check manip constraints if robot is the only configuration!
            OPENRAVE_ASSERT_OP(_parameters->GetDOF(), ==, info->gpos.dof);
            OPENRAVE_ASSERT_OP(_manipconstraintchecker->GetCheckManips().size(),==,1);
            RobotBase::ManipulatorPtr pmanip = _manipconstraintchecker->GetCheckManips().front().pmanip;
            OPENRAVE_ASSERT_OP(pmanip->GetArmDOF(),==,info->gpos.dof);

            // look at the first pose and try to determine proper velocity limits
            std::vector<dReal>& vellimits=_cachevellimits, &accellimits=_cacheaccellimits;
            vellimits = info->_vConfigVelocityLimit;
            accellimits = info->_vConfigAccelerationLimit;

            // cannot use _parameters->SetStateValues...
            pmanip->GetRobot()->SetDOFValues(_v0pos, KinBody::CLA_CheckLimits, pmanip->GetArmIndices());
            _manipconstraintchecker->GetMaxVelocitiesAccelerations(_v0vel, vellimits, accellimits);

            pmanip->GetRobot()->SetDOFValues(_v1pos, KinBody::CLA_CheckLimits, pmanip->GetArmIndices());
            _manipconstraintchecker->GetMaxVelocitiesAccelerations(_v1vel, vellimits, accellimits);

            for(size_t j = 0; j < info->_vConfigVelocityLimit.size(); ++j) {
                // have to watch out that velocities don't drop under dx0 & dx1!
                dReal fminvel = max(RaveFabs(_v0vel[j]), RaveFabs(_v1vel[j]));
                if( vellimits[j] < fminvel ) {
                    vellimits[j] = fminvel;
                }
                else {
                    dReal f = max(fminvel, info->_vConfigVelocityLimit[j]);
                    if( vellimits[j] > f ) {
                        vellimits[j] = f;
                    }
                }
                {
                    dReal f = info->_vConfigAccelerationLimit[j];
                    if( accellimits[j] > f ) {
                        accellimits[j] = f;
                    }
                }
            }

            size_t maxSlowDowns = 10;
            for(size_t islowdowntry = 0; islowdowntry < maxSlowDowns; ++islowdowntry ) {
                mintime = ParabolicRamp::SolveMinTimeBounded(_v0pos, _v0vel, _v1pos, _v1vel, accellimits, vellimits, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps, _parameters->_multidofinterp);
                if( mintime < 0 ) {
                    break;
                }
                if( mintime < g_fEpsilon ) {
                    break;
                }

                _rampsnd.resize(0);
                ParabolicRampInternal::CombineRamps(_ramps, _rampsnd);
                ParabolicRampInternal::CheckReturn retseg = _manipconstraintchecker->CheckManipConstraints2(_rampsnd);
                if( retseg.retcode == 0 ) {
                    break;
                }
                RAVELOG_VERBOSE_FORMAT("env=%d, manip constraints invalidated try %d, slowing down by %f", GetEnv()->GetId()%islowdowntry%retseg.fTimeBasedSurpassMult);
                // have to slow down the ramp and check again
                for(size_t j = 0; j < vellimits.size(); ++j) {
                    // have to watch out that velocities don't drop under dx0 & dx1!
                    dReal fminvel = max(RaveFabs(_v0vel[j]), RaveFabs(_v1vel[j]));
                    vellimits[j] = max(vellimits[j]*retseg.fTimeBasedSurpassMult, fminvel);
                    accellimits[j] *= retseg.fTimeBasedSurpassMult;
                }
                mintime = -1; // have to reset in case this is the last try
            }
            if( mintime < 0 ) {
                RAVELOG_WARN_FORMAT("env=%d, manip constraints failed to slow ramp down by", GetEnv()->GetId());
            }
        }
        else {
            // no manip constraints
            mintime = ParabolicRamp::SolveMinTimeBounded(_v0pos, _v0vel, _v1pos, _v1vel, info->_vConfigAccelerationLimit, info->_vConfigVelocityLimit, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps,_parameters->_multidofinterp);
#ifdef _DEBUG
            if( mintime < 0 && IS_DEBUGLEVEL(Level_Verbose) ) {
                // do again for debugging reproduction
                mintime = ParabolicRamp::SolveMinTimeBounded(_v0pos, _v0vel, _v1pos, _v1vel, info->_vConfigAccelerationLimit, info->_vConfigVelocityLimit, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps,_parameters->_multidofinterp);
            }
#endif
        }

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

    bool _CheckJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata, int checkoptions) {
        dReal deltatime = *(itdata+_timeoffset);
        for(int i=0; i < info->gvel.dof; ++i) {
            dReal fvel = *(itdata+info->gvel.offset+i);
            if( checkoptions & 2 ) {

                if( RaveFabs(fvel) > info->_vConfigVelocityLimit.at(i) + ParabolicRamp::EpsilonV ) {
                    return false;
                }
            }
            if( checkoptions & 4 ) {
                dReal diff = RaveFabs(*(itdataprev+info->gvel.offset+i)-fvel);
                if( RaveFabs(diff) > info->_vConfigAccelerationLimit.at(i) * deltatime + ParabolicRamp::EpsilonA ) {
                    return false;
                }
            }
        }
        return true;
    }

    bool _WriteJointValues(GroupInfoConstPtr inforaw, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
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
                    std::copy(itdata+info->gpos.offset, itdata+info->gpos.offset+info->gpos.dof, _vtrajpoints.begin()+info->posindex);
                    std::copy(itdata+info->gvel.offset, itdata+info->gvel.offset+info->gvel.dof, _vtrajpoints.begin()+info->velindex);
                    _vtrajpoints.at(info->timeindex) = deltatime;
                    _vtrajpoints.at(info->waypointindex) = 1;
                    info->ptraj->Insert(info->ptraj->GetNumWaypoints(),_vtrajpoints);
                }
                return true;
            }
            OPENRAVE_ASSERT_OP(deltatime,>,0);
            if( deltatime < ParabolicRamp::EpsilonT ) {
                RAVELOG_WARN_FORMAT("env=%d, delta time is really ill-conditioned: %e", GetEnv()->GetId()%deltatime);
            }

            bool success = ParabolicRamp::SolveAccelBounded(_v0pos, _v0vel, _v1pos, _v1vel, deltatime, info->_vConfigAccelerationLimit, info->_vConfigVelocityLimit, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps, _parameters->_multidofinterp);
            if( !success ) {
#ifdef _DEBUG
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    // for debugging
                    success = ParabolicRamp::SolveAccelBounded(_v0pos, _v0vel, _v1pos, _v1vel, deltatime, info->_vConfigAccelerationLimit, info->_vConfigVelocityLimit, info->_vConfigLowerLimit,info->_vConfigUpperLimit, _ramps, _parameters->_multidofinterp);
                }
#endif
                // PARABOLICWARN("SOLVEACCELBOUNDED in _WRITEJOINTVALUES FAILED");
                return false;
            }
            // PARABOLICWARN("SOLVEACCELBOUNDED in _WRITEJOINTVALUES SUCCEEDED");

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
                        RAVELOG_WARN_FORMAT("env=%d, ramp violates limits: %f>%f || %f>%f || %f>%f", GetEnv()->GetId()%RaveFabs(ramp[j].a1)%maxaccel%RaveFabs(ramp[j].a2)%maxaccel%RaveFabs(ramp[j].v)%maxvel);
                        return false;
                    }

                    vector<dReal>::iterator it;
                    dReal tswitch1 = curtime+ramp[j].tswitch1;
                    dReal tswitch2 = curtime+ramp[j].tswitch2;
                    dReal ttotal = curtime+ramp[j].ttotal;
                    if( tswitch1 != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),tswitch1);
                        if( it != vswitchtimes.end() && RaveFabs(*it - tswitch1) > ParabolicRamp::EpsilonT ) {//*it != tswitch1) {
                            vswitchtimes.insert(it,tswitch1);
                        }
                    }
                    if( RaveFabs(tswitch1 - tswitch2) > ParabolicRamp::EpsilonT && RaveFabs(tswitch2) > ParabolicRamp::EpsilonT ) {//( tswitch1 != tswitch2 && tswitch2 != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),tswitch2);
                        if( it != vswitchtimes.end() && RaveFabs(*it - tswitch2) > ParabolicRamp::EpsilonT ) {//*it != tswitch2 ) {
                            vswitchtimes.insert(it,tswitch2);
                        }
                    }
                    if( RaveFabs(tswitch2 - ttotal) > ParabolicRamp::EpsilonT && RaveFabs(ttotal) > ParabolicRamp::EpsilonT ) {//( tswitch2 != ttotal && ttotal != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),ttotal);
                        if( it != vswitchtimes.end() && RaveFabs(*it - ttotal) > ParabolicRamp::EpsilonT ) {//*it != ttotal ) {
                            vswitchtimes.insert(it,ttotal);
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
                            // in case of epsilons, set the last point as the values
                            *(ittargetdata + info->posindex + j) = _ramps[j][iramp].x1;
                            *(ittargetdata + info->velindex + j) = _ramps[j][iramp].dx1;
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
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_ComputeMinimumTimeAffine not implemented"), ORE_NotImplemented);
    }

    void _ComputeVelocitiesAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_ComputeVelocitiesAffine not implemented"), ORE_NotImplemented);
    }

    bool _CheckAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata, int checkoptions) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("not implemented"), ORE_NotImplemented);
    }

    bool _WriteAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_WriteAffine not implemented"), ORE_NotImplemented);
    }

    // speed of rotations is always the speed of the angle along the minimum rotation
    // speed of translations is always the combined xyz speed
    // TODO ParabolicRamp::SolveMinTimeBounded can only consider max vel/accel per dimension rather than combined together....
    dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) {
        ParabolicRamp::ParabolicRamp1D ramp;
        IkParameterization ikparamprev, ikparam;
        ikparamprev.Set(itdataprev+info->gpos.offset,iktype);
        ikparam.Set(itdata+info->gpos.offset,iktype);
        Vector trans0, trans1;
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
                RAVELOG_WARN_FORMAT("env=%d, failed solving mintime for angles", GetEnv()->GetId());
                return -1;
            }
            mintime = ramp.EndTime();
            trans1 = ikparam.GetTransform6D().trans;
            trans0 = ikparamprev.GetTransform6D().trans;
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
                RAVELOG_WARN_FORMAT("env=%d, failed solving mintime for angles", GetEnv()->GetId());
                return -1;
            }
            mintime = ramp.EndTime();
            break;
        }
        case IKP_Translation3D:
            trans1 = ikparam.GetTranslation3D();
            trans0 = ikparamprev.GetTranslation3D();
            transoffset = 0;
            break;
        case IKP_TranslationDirection5D: {
            dReal angledelta = RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetTranslationDirection5D().dir.dot3(ikparam.GetTranslationDirection5D().dir))));
            // the acos really messes up the precision even if the two dirs are within 1e-16, the acos error could be > 1e-8
            if( angledelta > ParabolicRamp::EpsilonX*100 ) {
                Vector angularvelocityprev(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2));
                dReal anglevelprev = RaveSqrt(utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w));
                Vector angularvelocity(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2));
                dReal anglevel = RaveSqrt(utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w));
                if( !ParabolicRamp::SolveMinTimeBounded(0,anglevelprev,angledelta,anglevel, info->_vConfigAccelerationLimit.at(0), info->_vConfigVelocityLimit.at(0), -1000,angledelta+1000, ramp) ) {
                    RAVELOG_WARN_FORMAT("env=%d, failed solving mintime for angles", GetEnv()->GetId());
                    return -1;
                }
                mintime = ramp.EndTime();
            }
            else {
                mintime = 0;
            }
            trans1 = ikparam.GetTranslationDirection5D().pos;
            trans0 = ikparamprev.GetTranslationDirection5D().pos;
            transoffset = 3;
            break;
        }
        case IKP_TranslationXAxisAngleZNorm4D: {
            dReal angledelta = utils::SubtractCircularAngle(ikparam.GetTranslationXAxisAngleZNorm4D().second,ikparamprev.GetTranslationXAxisAngleZNorm4D().second);
            if( !ParabolicRamp::SolveMinTimeBounded(0,*(itdataprev+info->gvel.offset+0),angledelta,*(itdata+info->gvel.offset+0), info->_vConfigAccelerationLimit.at(0), info->_vConfigVelocityLimit.at(0), -1000,angledelta+1000, ramp) ) {
                RAVELOG_WARN_FORMAT("env=%d, failed solving mintime for angles", GetEnv()->GetId());
                return false;
            }
            mintime = ramp.EndTime();
            trans1 = ikparam.GetTranslationXAxisAngleZNorm4D().first;
            trans0 = ikparamprev.GetTranslationXAxisAngleZNorm4D().first;
            transoffset = 1;
            break;
        }
        case IKP_TranslationYAxisAngleXNorm4D: {
            dReal angledelta = utils::SubtractCircularAngle(ikparam.GetTranslationYAxisAngleXNorm4D().second,ikparamprev.GetTranslationYAxisAngleXNorm4D().second);
            if( !ParabolicRamp::SolveMinTimeBounded(0,*(itdataprev+info->gvel.offset+0),angledelta,*(itdata+info->gvel.offset+0), info->_vConfigAccelerationLimit.at(0), info->_vConfigVelocityLimit.at(0), -1000,angledelta+1000, ramp) ) {
                RAVELOG_WARN_FORMAT("env=%d, failed solving mintime for angles", GetEnv()->GetId());
                return false;
            }
            mintime = ramp.EndTime();
            trans1 = ikparam.GetTranslationYAxisAngleXNorm4D().first;
            trans0 = ikparamprev.GetTranslationYAxisAngleXNorm4D().first;
            transoffset = 1;
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, does not support parameterization 0x%x"), GetEnv()->GetId()%ikparam.GetType(),ORE_InvalidArguments);
        }
        if( transoffset >= 0 ) {
            std::vector<dReal> xyz0(3, 0), xyz1(3), xyzvelprev(3), xyzvel(3), vlowerlimit(3), vupperlimit(3);
            for(int j = 0; j < 3; ++j) {
                xyz0[j] = trans0[j];
                xyz1[j] = trans1[j];
                xyzvelprev[j] = *(itdataprev+info->gvel.offset+transoffset+j);
                xyzvel[j] = *(itdata+info->gvel.offset+transoffset+j);
                vlowerlimit[j] = -1000-RaveFabs(trans0[j]);
                vupperlimit[j] = 1000+RaveFabs(trans1[j]);
            }
            std::vector<dReal> vaccellimit(info->_vConfigAccelerationLimit.begin()+transoffset, info->_vConfigAccelerationLimit.begin()+transoffset+3);
            std::vector<dReal> vvellimit(info->_vConfigVelocityLimit.begin()+transoffset, info->_vConfigVelocityLimit.begin()+transoffset+3);
            _ramps.resize(3);
            dReal xyzmintime =  ParabolicRamp::SolveMinTimeBounded(xyz0,xyzvelprev,xyz1,xyzvel, vaccellimit, vvellimit, vlowerlimit, vupperlimit, _ramps, _parameters->_multidofinterp);
            if( xyzmintime < 0 ) {
                RAVELOG_WARN_FORMAT("env=%d, failed solving mintime for xyz", GetEnv()->GetId());
                return -1;
            }
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

    bool _CheckIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata, int checkoptions)
    {
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
        case IKP_TranslationXAxisAngleZNorm4D:
        case IKP_TranslationYAxisAngleXNorm4D:
        {
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
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, does not support parameterization 0x%x"), GetEnv()->GetId()%iktype,ORE_InvalidArguments);
        }
        if( transoffset >= 0 ) {
            for(int j = 0; j < 3; ++j) {
                dReal fvelprev = *(itdataprev+info->gvel.offset+transoffset+j);
                dReal fvel = *(itdata+info->gvel.offset+transoffset+j);
                if( RaveFabs(fvel) > info->_vConfigVelocityLimit.at(transoffset+j) + ParabolicRamp::EpsilonV ) {
                    return false;
                }
                if( RaveFabs(fvel - fvelprev) > info->_vConfigAccelerationLimit.at(transoffset+j)*deltatime + ParabolicRamp::EpsilonA ) {
                    return false;
                }
            }
        }
        return true;
    }

    bool _WriteIk(GroupInfoConstPtr inforaw, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
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
                RAVELOG_WARN_FORMAT("env=%d, delta time is really ill-conditioned: %e", GetEnv()->GetId()%deltatime);
            }

            IkParameterization ikparamprev, ikparam;
            ikparamprev.Set(itdataprev+info->gpos.offset,iktype);
            ikparam.Set(itdata+info->gpos.offset,iktype);
            _v0pos.resize(0); _v0vel.resize(0); _v1pos.resize(0); _v1vel.resize(0);
            Vector trans0, trans1, axisangle;
            int transoffset = -1, transindex = -1;
            vector<dReal> vmaxvel, vmaxaccel, vlower, vupper;

            switch(iktype) {
            case IKP_Transform6D: {
                _v0pos.resize(4); _v0vel.resize(4); _v1pos.resize(4); _v1vel.resize(4); vmaxvel.resize(4); vmaxaccel.resize(4);
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
                trans1 = ikparam.GetTransform6D().trans;
                trans0 = ikparamprev.GetTransform6D().trans;
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
                _v0pos.resize(3); _v0vel.resize(3); _v1pos.resize(3); _v1vel.resize(3); vmaxvel.resize(3); vmaxaccel.resize(3);
                trans1 = ikparam.GetTranslation3D();
                trans0 = ikparamprev.GetTranslation3D();
                transoffset = 0;
                transindex = 0;
                break;
            case IKP_TranslationDirection5D: {
                _v0pos.resize(4); _v0vel.resize(4); _v1pos.resize(4); _v1vel.resize(4); vmaxvel.resize(4); vmaxaccel.resize(4);
                axisangle = ikparamprev.GetTranslationDirection5D().dir.cross(ikparam.GetTranslationDirection5D().dir);
                if( axisangle.lengthsqr3() > g_fEpsilon ) {
                    axisangle.normalize3();
                    _v1pos[0] = RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetTranslationDirection5D().dir.dot3(ikparam.GetTranslationDirection5D().dir))));
                }
                else {
                    _v1pos[0] = 0;
                }
                _v0pos[0] = 0;
                Vector angularvelocityprev(*(itdataprev+info->gvel.offset+0), *(itdataprev+info->gvel.offset+1), *(itdataprev+info->gvel.offset+2));
                _v0vel[0] = RaveSqrt(utils::Sqr(angularvelocityprev.y) + utils::Sqr(angularvelocityprev.z) + utils::Sqr(angularvelocityprev.w));
                Vector angularvelocity(*(itdata+info->gvel.offset+0), *(itdata+info->gvel.offset+1), *(itdata+info->gvel.offset+2));
                _v1vel[0] = RaveSqrt(utils::Sqr(angularvelocity.y) + utils::Sqr(angularvelocity.z) + utils::Sqr(angularvelocity.w));
                vmaxvel[0] = info->_vConfigVelocityLimit.at(0);
                vmaxaccel[0] = info->_vConfigAccelerationLimit.at(0);
                trans1 = ikparam.GetTranslationDirection5D().pos;
                trans0 = ikparamprev.GetTranslationDirection5D().pos;
                transoffset = 3;
                transindex = 1;
                break;
            }
            case IKP_TranslationXAxisAngleZNorm4D: {
                _v0pos.resize(4); _v0vel.resize(4); _v1pos.resize(4); _v1vel.resize(4); vmaxvel.resize(4); vmaxaccel.resize(4);
                _v0pos[0] = 0;
                _v1pos[0] = utils::SubtractCircularAngle(ikparam.GetTranslationXAxisAngleZNorm4D().second,ikparamprev.GetTranslationXAxisAngleZNorm4D().second);
                _v0vel[0] = *(itdataprev+info->gvel.offset+0);
                _v1vel[0] = *(itdata+info->gvel.offset+0);
                vmaxvel[0] = info->_vConfigVelocityLimit.at(0);
                vmaxaccel[0] = info->_vConfigAccelerationLimit.at(0);
                trans1 = ikparam.GetTranslationXAxisAngleZNorm4D().first;
                trans0 = ikparamprev.GetTranslationXAxisAngleZNorm4D().first;
                transoffset = 1;
                transindex = 1;
                break;
            }
            case IKP_TranslationYAxisAngleXNorm4D: {
                _v0pos.resize(4); _v0vel.resize(4); _v1pos.resize(4); _v1vel.resize(4); vmaxvel.resize(4); vmaxaccel.resize(4);
                _v0pos[0] = 0;
                _v1pos[0] = utils::SubtractCircularAngle(ikparam.GetTranslationYAxisAngleXNorm4D().second,ikparamprev.GetTranslationYAxisAngleXNorm4D().second);
                _v0vel[0] = *(itdataprev+info->gvel.offset+0);
                _v1vel[0] = *(itdata+info->gvel.offset+0);
                vmaxvel[0] = info->_vConfigVelocityLimit.at(0);
                vmaxaccel[0] = info->_vConfigAccelerationLimit.at(0);
                trans1 = ikparam.GetTranslationYAxisAngleXNorm4D().first;
                trans0 = ikparamprev.GetTranslationYAxisAngleXNorm4D().first;
                transoffset = 1;
                transindex = 1;
                break;
            }
            default:
                throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, does not support parameterization 0x%x"), GetEnv()->GetId()%ikparam.GetType(),ORE_InvalidArguments);
            }

            if( transoffset >= 0 ) {
                for(int j = 0; j < 3; ++j) {
                    _v0pos.at(transindex+j) = trans0[j];
                    _v1pos.at(transindex+j) = trans1[j];
                    _v0vel.at(transindex+j) = *(itdataprev+info->gvel.offset+transoffset+j);
                    _v1vel.at(transindex+j) = *(itdata+info->gvel.offset+transoffset+j);
                    vmaxvel.at(transindex+j) = info->_vConfigVelocityLimit.at(transoffset+j);
                    vmaxaccel.at(transindex+j) = info->_vConfigAccelerationLimit.at(transoffset+j);
                }
            }

            _ramps.resize(_v0pos.size());
            vlower.resize(_v0pos.size());
            vupper.resize(_v0pos.size());
            for(size_t i = 0; i < vlower.size(); ++i) {
                vlower[i] = -1000-RaveFabs(_v1pos[i]);
                vupper[i] = 1000+RaveFabs(_v1pos[i]);
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
                        RAVELOG_WARN_FORMAT("env=%d, ramp violates limits: %f>%f", GetEnv()->GetId()%RaveFabs(ramp[j].v)%maxvel);
                        return false;
                    }
                    // why is this commented out?
//                    if( RaveFabs(ramp[j].a1) > maxaccel+1e-5 || RaveFabs(ramp[j].a2) > maxaccel+1e-5 ) {
//                        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, ramp violates limits: %f>%f || %f>%f"),GetEnv()->GetId()%RaveFabs(ramp[j].a1)%maxaccel%RaveFabs(ramp[j].a2)%maxaccel, ORE_InconsistentConstraints);
//                    }

                    vector<dReal>::iterator it;
                    dReal tswitch1 = curtime+ramp[j].tswitch1;
                    dReal tswitch2 = curtime+ramp[j].tswitch2;
                    dReal ttotal = curtime+ramp[j].ttotal;
                    if( tswitch1 != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),tswitch1);
                        if( it != vswitchtimes.end() && *it != tswitch1) {
                            vswitchtimes.insert(it,tswitch1);
                        }
                    }
                    if( tswitch1 != tswitch2 && tswitch2 != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),tswitch2);
                        if( it != vswitchtimes.end() && *it != tswitch2 ) {
                            vswitchtimes.insert(it,tswitch2);
                        }
                    }
                    if( tswitch2 != ttotal && ttotal != 0 ) {
                        it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),ttotal);
                        if( it != vswitchtimes.end() && *it != ttotal ) {
                            vswitchtimes.insert(it,ttotal);
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

                //Vector translation;
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
                    //translation = ikparamprev.GetTransform6D().trans;
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
                    //translation = ikparamprev.GetTranslation3D();
                    break;
                }
                case IKP_TranslationDirection5D: {
                    Vector angularvel = axisangle*vvel.at(0);
                    Vector newdir =  quatRotate(quatFromAxisAngle(axisangle*vpos.at(0)),ikparamprev.GetTranslationDirection5D().dir);
                    for(size_t j = 0; j < 3; ++j) {
                        *(ittargetdata + info->posindex + j) = newdir[j];
                        *(ittargetdata + info->velindex + j) = angularvel[j];
                    }
                    //translation = ikparamprev.GetTranslationDirection5D().pos;
                    break;
                }
                case IKP_TranslationXAxisAngleZNorm4D: {
                    *(ittargetdata + info->posindex + 0) = ikparamprev.GetTranslationXAxisAngleZNorm4D().second + vpos.at(0);
                    *(ittargetdata + info->velindex + 0) = vvel.at(0);
                    //translation = ikparamprev.GetTranslationXAxisAngleZNorm4D().first;
                    break;
                }
                case IKP_TranslationYAxisAngleXNorm4D: {
                    *(ittargetdata + info->posindex + 0) = ikparamprev.GetTranslationYAxisAngleXNorm4D().second + vpos.at(0);
                    *(ittargetdata + info->velindex + 0) = vvel.at(0);
                    //translation = ikparamprev.GetTranslationYAxisAngleXNorm4D().first;
                    break;
                }
                default:
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, does not support parameterization 0x%x"), GetEnv()->GetId()%ikparam.GetType(),ORE_InvalidArguments);
                }
                if( transoffset >= 0 && transindex >= 0 ) {
                    for(size_t j = 0; j < 3; ++j) {
                        *(ittargetdata + info->posindex + transoffset + j) = vpos.at(transindex+j);
                        *(ittargetdata + info->velindex + transoffset + j) = vvel.at(transindex+j);
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

    // cache
    ParabolicRamp::Vector _v0pos, _v0vel, _v1pos, _v1vel;
    vector<dReal> _vtrajpoints;
    std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > _ramps;
    std::vector<ParabolicRamp::ParabolicRampND> _rampsnd;
    std::vector<dReal> _cachevellimits, _cacheaccellimits;
};

PlannerBasePtr CreateParabolicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new ParabolicTrajectoryRetimer(penv, sinput));
}

} // end namespace rplanners
