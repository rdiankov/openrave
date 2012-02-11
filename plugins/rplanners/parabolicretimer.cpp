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
        dReal mintime = ParabolicRamp::SolveMinTimeBounded(_v0pos, _v0vel, _v1pos, _v1vel, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigLowerLimit,_parameters->_vConfigUpperLimit, _ramps);
        BOOST_ASSERT(mintime>=0);
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

    void _WriteJointValues(GroupInfoConstPtr inforaw, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
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
                    // add the first point
                    _vtrajpoints.resize(info->ptraj->GetConfigurationSpecification().GetDOF());
                    std::copy(itdata,itdata+_vtrajpoints.size(),_vtrajpoints.begin());
                    _vtrajpoints.at(info->waypointindex) = 1;
                    info->ptraj->Insert(info->ptraj->GetNumWaypoints(),_vtrajpoints);
                }
                return;
            }
            OPENRAVE_ASSERT_OP(deltatime,>,0);
            if( deltatime < ParabolicRamp::EpsilonT ) {
                RAVELOG_WARN(str(boost::format("delta time is really ill-conditioned: %e")%deltatime));
            }

            bool success = ParabolicRamp::SolveMinAccelBounded(_v0pos, _v0vel, _v1pos, _v1vel, deltatime, _parameters->_vConfigVelocityLimit, _parameters->_vConfigLowerLimit,_parameters->_vConfigUpperLimit, _ramps);
            BOOST_ASSERT(success);

            vector<dReal> vswitchtimes;
            if( info->ptraj->GetNumWaypoints() == 0 ) {
                // add initial point of the segment only for the first point in the trajectory
                vswitchtimes.push_back(0);
            }
            vswitchtimes.push_back(deltatime);
            for(size_t i=0; i < _ramps.size(); ++i) {
                std::vector<ParabolicRamp::ParabolicRamp1D>& ramp = _ramps[i];
                dReal maxaccel = _parameters->_vConfigAccelerationLimit[i]+ParabolicRamp::EpsilonA;
                dReal maxvel = _parameters->_vConfigVelocityLimit[i]+ParabolicRamp::EpsilonV;
                dReal curtime = 0;
                for(size_t j=0; j < ramp.size(); j++) {
                    if(RaveFabs(ramp[j].a1) > maxaccel || RaveFabs(ramp[j].a2) > maxaccel || RaveFabs(ramp[j].v) > maxvel) {
                        throw OPENRAVE_EXCEPTION_FORMAT("ramp violates limits: %f>%f || %f>%f || %f>%f",RaveFabs(ramp[j].a1)%maxaccel%RaveFabs(ramp[j].a2)%maxaccel%RaveFabs(ramp[j].v)%maxvel, ORE_InconsistentConstraints);
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
    }

    dReal _ComputeMinimumTimeAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_ComputeMinimumTimeAffine not implemented", ORE_NotImplemented);
    }

    void _ComputeVelocitiesAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_ComputeVelocitiesAffine not implemented", ORE_NotImplemented);
    }

    void _WriteAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_WriteAffine not implemented", ORE_NotImplemented);
    }

    dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_ComputeMinimumTimeIk not implemented", ORE_NotImplemented);
    }

    void _ComputeVelocitiesIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_ComputeVelocitiesIk not implemented", ORE_NotImplemented);
    }

    void _WriteIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0("_WriteIk not implemented", ORE_NotImplemented);
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
