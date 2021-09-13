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

#include <openrave/mathextra.h>

namespace rplanners {

/**
   t,dt,v0,v1,px = symbols('t,dt,v0,v1,px')

   c3 = (v1*dt + v0*dt - 2*px)/(dt**3)
   c2 = (3*px - 2*v0*dt - v1*dt)/(dt**2)
   c1 = v0
   c0 = 0
   p = c3*t**3 + c2*t**2 + c1*t + c0
 */
class CubicTrajectoryRetimer : public TrajectoryRetimer
{
public:
    class CubicGroupInfo : public GroupInfo
    {
public:
        CubicGroupInfo(int degree, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) : GroupInfo(degree, gpos, gvel) {
        }

        //ConfigurationSpecification::Group gaccel;
    };
    typedef boost::shared_ptr<CubicGroupInfo> CubicGroupInfoPtr;
    typedef boost::shared_ptr<CubicGroupInfo const> CubicGroupInfoConstPtr;

    CubicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : TrajectoryRetimer(penv,sinput)
    {
        __description = ":Interface Author: Rosen Diankov\n\nSingle cubic trajectory re-timing while passing through the waypoints, waypoints will not be modified. Computing fastest time is slow";
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        _trajxmlid = ptraj->GetXMLId();
        return TrajectoryRetimer::PlanPath(ptraj, planningoptions);
    }

protected:
    GroupInfoPtr CreateGroupInfo(int degree, const ConfigurationSpecification& spec, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) {
        CubicGroupInfoPtr g(new CubicGroupInfo(degree, gpos, gvel));
//        if( gvel.name.size() >= 16 && gvel.name.substr(0,16) == std::string("joint_velocities") ) {
//            std::string accelname = std::string("joint_accelerations") + gvel.name.substr(16);
//            g->gaccel = spec.GetGroupFromName(accelname);
//        }
        return g;
    }

    void ResetCachedGroupInfo(GroupInfoPtr g)
    {
    }

    bool _SupportInterpolation() {
        if( _parameters->_interpolation.size() == 0 ) {
            _parameters->_interpolation = "cubic";
            return true;
        }
        else {
            return _parameters->_interpolation == "cubic";
        }
    }

    bool _ValidateCubicSpline(const std::vector<dReal>& v0pos, const std::vector<dReal>& v0vel, const std::vector<dReal>& v1pos, const std::vector<dReal>& v1vel, dReal deltatime)
    {
        dReal ideltatime = 1/deltatime;
        dReal ideltatime2 = ideltatime*ideltatime;
        for(size_t idof = 0; idof < v0pos.size(); ++idof) {
            dReal px = v1pos.at(idof)-v0pos.at(idof);
//            c3 = (v1vel.at(idof)*deltatime + v0vel.at(idof)*deltatime - 2*px)/(deltatime**3);
//            c2 = (3*px - 2*v0vel.at(idof)*deltatime - v1vel.at(idof)*deltatime)/(deltatime**2);
            dReal c3 = (v1vel.at(idof) + v0vel.at(idof) - 2*px*ideltatime)*ideltatime2;
            dReal c2 = (3*px*ideltatime - (2*v0vel.at(idof) + v1vel.at(idof)))*ideltatime;
            dReal c1 = v0vel.at(idof);
            dReal c0 = v0pos.at(idof);
            // check position limits
            dReal times[2];
            int nroots = mathextra::solvequad(3*c3, 2*c2, c1, times[0], times[1]);
            for(int iroot = 0; iroot < nroots; ++iroot ) {
                if( times[iroot] > 0 && times[iroot] < deltatime ) {
                    dReal pos = c0 + times[iroot] * (c1 + times[iroot] * (c2 + times[iroot]*c3));
                    if( pos < _parameters->_vConfigLowerLimit[idof]-g_fEpsilonJointLimit || pos > _parameters->_vConfigUpperLimit[idof]+g_fEpsilonJointLimit ) {
                        return false;
                    }
                }
            }
            // check velocity limits a1*t + a0 = 0
            dReal a1 = 6*c3, a0 = 2*c2;
            // check acceleration limits
            if( RaveFabs(a0) > _parameters->_vConfigAccelerationLimit.at(idof)+g_fEpsilonJointLimit ) {
                return false;
            }
            if( RaveFabs(a0+a1*deltatime) > _parameters->_vConfigAccelerationLimit.at(idof)+g_fEpsilonJointLimit ) {
                return false;
            }

            if( RaveFabs(a1) > 0 ) {
                dReal vtime = -a0/a1;
                if( vtime > 0 && vtime < deltatime ) {
                    dReal vellimit = c1 + vtime * (2*c2 + vtime * 3*c3 );
                    if( RaveFabs(vellimit) > _parameters->_vConfigVelocityLimit.at(idof)+g_fEpsilonJointLimit ) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    dReal _ComputeMinimumTimeJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) {
        dReal mintime = 0;
        _v0pos.resize(info->gpos.dof);
        _v1pos.resize(info->gpos.dof);
        for(int i = 0; i < info->gpos.dof; ++i) {
            _v0pos[i] = *(itdataprev+info->gpos.offset+i);
            dReal posdiff = *(itorgdiff+info->orgposoffset+i);
            _v1pos[i] = _v0pos[i] + posdiff;
            dReal testmintime = RaveFabs(posdiff) / _parameters->_vConfigVelocityLimit.at(i);
            if( mintime < testmintime ) {
                mintime = testmintime;
            }
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
            dReal testmintime = RaveFabs(_v1vel[i]-_v0vel[i]) / _parameters->_vConfigAccelerationLimit.at(i);
            if( mintime < testmintime ) {
                mintime = testmintime;
            }
        }

        for(size_t itry = 0; itry += 40; ++itry) {
            if( _ValidateCubicSpline(_v0pos, _v0vel, _v1pos, _v1vel, mintime) ) {
                return mintime;
            }
            mintime *= 1.05; //mintime += _parameters->_fStepLength;
        }
        return -1;
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

    bool _CheckJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata, int checkoptions=0xffffffff) {
        dReal deltatime = *(itdata+_timeoffset);
        dReal ideltatime = 1/deltatime;
        dReal ideltatime2 = ideltatime*ideltatime;
        for(int i=0; i < info->gvel.dof; ++i) {
            dReal px = *(itdata+info->gpos.offset+i) - *(itdataprev+info->gpos.offset+i);
            dReal v0 = *(itdataprev+info->gvel.offset+i);
            dReal v1 = *(itdata+info->gvel.offset+i);
            dReal c3 = (v1 + v0 - 2*px*ideltatime)*ideltatime2;
            dReal c2 = (3*px*ideltatime - (2*v0 + v1))*ideltatime;
            dReal c1 = v0;

            if( checkoptions & 1 ) {
                // check position limits
                dReal c0 = *(itdataprev+info->gpos.offset+i);
                dReal times[2];
                int nroots = mathextra::solvequad(3*c3, 2*c2, c1, times[0], times[1]);
                for(int iroot = 0; iroot < nroots; ++iroot ) {
                    if( times[iroot] > 0 && times[iroot] < deltatime ) {
                        dReal pos = c0 + times[iroot] * (c1 + times[iroot] * (c2 + times[iroot]*c3));
                        if( pos < info->_vConfigLowerLimit[i]-g_fEpsilonJointLimit || pos > info->_vConfigUpperLimit[i]+g_fEpsilonJointLimit ) {
                            RAVELOG_VERBOSE_FORMAT("env=%d, pos constraints dof=%d, %e (limit) < %e < %e (limit)", GetEnv()->GetId()%i%info->_vConfigLowerLimit[i]%pos%info->_vConfigUpperLimit[i]);
                            return false;
                        }
                    }
                }
            }

            // check velocity limits a1*t + a0 = 0
            dReal a1 = 6*c3, a0 = 2*c2;
            if(checkoptions&4) {
                // check acceleration limits
                if( RaveFabs(a0) > info->_vConfigAccelerationLimit.at(i)+g_fEpsilonJointLimit ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, acceleration constraints dof=%d, abs(%e) > %e (limit)", GetEnv()->GetId()%i%a0%(info->_vConfigAccelerationLimit.at(i)));
                    return false;
                }
                if( RaveFabs(a0+a1*deltatime) > info->_vConfigAccelerationLimit.at(i)+g_fEpsilonJointLimit ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, acceleration constraints dof=%d, abs(%e) > %e (limit)", GetEnv()->GetId()%i%(a0+a1*deltatime)%(info->_vConfigAccelerationLimit.at(i)));
                    return false;
                }
            }
            if(checkoptions&2) {
                if( RaveFabs(a1) > 0 ) {
                    dReal vtime = -a0/a1;
                    if( vtime > 0 && vtime < deltatime ) {
                        dReal vellimit = c1 + vtime * (2*c2 + vtime * 3*c3 );
                        if( RaveFabs(vellimit) > info->_vConfigVelocityLimit.at(i)+g_fEpsilonJointLimit ) {
                            RAVELOG_VERBOSE_FORMAT("env=%d, velocity constraints dof=%d, abs(%e) > %e (limit)", GetEnv()->GetId()%i%vellimit%info->_vConfigVelocityLimit.at(i));
                            return false;
                        }
                    }
                }
                if( RaveFabs(v0) > info->_vConfigVelocityLimit[i]+g_fEpsilonJointLimit ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, velocity constraints dof=%d, abs(%e) > %e (limit)", GetEnv()->GetId()%i%v0%info->_vConfigVelocityLimit[i]);
                    return false;
                }
                if( RaveFabs(v1) > info->_vConfigVelocityLimit[i]+g_fEpsilonJointLimit ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, velocity constraints dof=%d, abs(%e) > %e (limit)", GetEnv()->GetId()%i%v1%info->_vConfigVelocityLimit[i]);
                    return false;
                }
            }
        }
        return true;
    }

    bool _WriteJointValues(GroupInfoConstPtr inforaw, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        CubicGroupInfoConstPtr info = boost::dynamic_pointer_cast<CubicGroupInfo const>(inforaw);
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
        dReal deltatime = *(itdata+_timeoffset);
        dReal ideltatime = 1/deltatime;
        dReal ideltatime2 = ideltatime*ideltatime;
        for(int i=0; i < info->gvel.dof; ++i) {
            dReal px = *(itdata+info->gpos.offset+i) - *(itdataprev+info->gpos.offset+i);
            dReal v0 = *(itdataprev+info->gvel.offset+i);
            dReal v1 = *(itdata+info->gvel.offset+i);
            dReal c3 = (v1 + v0 - 2*px*ideltatime)*ideltatime2;
            dReal c2 = (3*px*ideltatime - (2*v0 + v1))*ideltatime;
//            dReal a1 = 6*c3, a0 = 2*c2;
//            *(itdata+info->gaccel.offset+i) = a0 + deltatime*a1;
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

    dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_ComputeMinimumTimeIk not implemented"), ORE_NotImplemented);
    }

    void _ComputeVelocitiesIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_ComputeVelocitiesIk not implemented"), ORE_NotImplemented);
    }

    bool _CheckIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata, int checkoptions) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("not implemented"), ORE_NotImplemented);
        return true;
    }

    bool _WriteIk(GroupInfoConstPtr inforaw, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("_WriteIk not implemented"), ORE_NotImplemented);
    }

    void _WriteTrajectory(TrajectoryBasePtr ptraj, const ConfigurationSpecification& newspec, const std::vector<dReal>& data) {
        ptraj->Init(newspec);
        ptraj->Insert(0,data);
    }

    string _trajxmlid;
    std::vector<dReal> _v0pos, _v0vel, _v1pos, _v1vel;
};

PlannerBasePtr CreateCubicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new CubicTrajectoryRetimer(penv, sinput));
}

} // end namespace rplanners
