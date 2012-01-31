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
#include "plugindefs.h"
#include "plannerparameters.h"
#include "trajectoryretimer.h"
#include "ParabolicPathSmooth/ParabolicRamp.h"

class ParabolicTrajectoryRetimer : public TrajectoryRetimer
{
public:
    ParabolicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : TrajectoryRetimer(penv,sinput)
    {
        __description = ":Interface Author: Rosen Diankov\n\nParabolic trajectory re-timing without modifying any of the points. Overwrites the velocities and timestamps.";
    }

protected:
    virtual bool _SupportInterpolation() {
        if( _parameters->_interpolation.size() == 0 ) {
            _parameters->_interpolation = "quadratic";
            return true;
        }
        else {
            return _parameters->_interpolation == "quadratic";
        }
    }

    dReal _ComputeMinimumTimeJointValues(GroupInfo& info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        _v0pos.resize(info.gpos.dof);
        _v1pos.resize(info.gpos.dof);
        for(int i = 0; i < info.gpos.dof; ++i) {
            _v0pos[i] = *(itdataprev+info.gpos.offset+i);
            _v1pos[i] = _v0pos[i] + *(itorgdiff+info.orgdofoffset+i);
        }
        _v0vel.resize(info.gvel.dof);
        _v1vel.resize(info.gvel.dof);
        for(int i = 0; i < info.gvel.dof; ++i) {
            _v0vel[i] = *(itdataprev+info.gvel.offset+i);
            if( bUseEndVelocity ) {
                _v1vel[i] = *(itdata+info.gvel.offset+i);
            }
            else {
                _v1vel[i] = 0;
            }
        }
        _ramps.resize(info.gpos.dof);
        dReal mintime = ParabolicRamp::SolveMinTimeBounded(_v0pos, _v0vel, _v1pos, _v1vel, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigLowerLimit,_parameters->_vConfigUpperLimit, _ramps);
        BOOST_ASSERT(mintime>=0);
        return mintime;
    }

    void _ComputeVelocitiesJointValues(GroupInfo& info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        _v0pos.resize(info.gpos.dof);
        _v1pos.resize(info.gpos.dof);
        for(int i = 0; i < info.gpos.dof; ++i) {
            _v0pos[i] = *(itdataprev+info.gpos.offset+i);
            _v1pos[i] = _v0pos[i] + *(itorgdiff+info.orgdofoffset+i);
        }
        _v0vel.resize(info.gvel.dof);
        _v1vel.resize(info.gvel.dof);
        for(int i = 0; i < info.gvel.dof; ++i) {
            _v0vel[i] = *(itdataprev+info.gvel.offset+i);
            _v1vel[i] = 0;
        }
        _ramps.resize(info.gpos.dof);
        dReal deltatime = *(itdata+_timeoffset);
        bool success = ParabolicRamp::SolveMinAccelBounded(_v0pos, _v0vel, _v1pos, _v1vel, deltatime, _parameters->_vConfigVelocityLimit, _parameters->_vConfigLowerLimit,_parameters->_vConfigUpperLimit, _ramps);
        BOOST_ASSERT(success);
        for(size_t i=0; i < _ramps.size(); ++i) {
            std::vector<ParabolicRamp::ParabolicRamp1D>& ramp = _ramps[i];
            dReal maxaccel = _parameters->_vConfigAccelerationLimit[i]+ParabolicRamp::EpsilonA;
            dReal maxvel = _parameters->_vConfigVelocityLimit[i]+ParabolicRamp::EpsilonV;
            for(size_t j=0; j < ramp.size(); j++) {
                if(RaveFabs(ramp[j].a1) > maxaccel || RaveFabs(ramp[j].a2) > maxaccel || RaveFabs(ramp[j].v) > maxvel) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("ramp violates limits",ORE_InconsistentConstraints);
                }
            }

            // this is 0 most of the time
            *(itdata+info.gvel.offset+i) = ramp.back().dx1;
        }
    }

    dReal _ComputeMinimumTimeAffine(GroupInfo& info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        BOOST_ASSERT(0);
        return 0;
    }

    void _ComputeVelocitiesAffine(GroupInfo& info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        BOOST_ASSERT(0);
    }

    dReal _ComputeMinimumTimeIk(GroupInfo& info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        BOOST_ASSERT(0);
        return 0;
    }

    void _ComputeVelocitiesIk(GroupInfo& info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        BOOST_ASSERT(0);
    }

    ParabolicRamp::Vector _v0pos, _v0vel, _v1pos, _v1vel;
    std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > _ramps;
};

PlannerBasePtr CreateParabolicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new ParabolicTrajectoryRetimer(penv, sinput));
}
