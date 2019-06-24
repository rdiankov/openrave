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

namespace Piecewisepolynomials = PiecewisePolynomialsInternal;

class QuinticTrajectoryRetimer : public TrajectoryRetimer3
{
public:
    class QuinticGroupInfo : public GroupInfo
    {
public:
        QuinticGroupInfo(int degree, const ConfigurationSpecification::Group& gPos, const ConfigurationSpecification::Group& gVel, const ConfigurationSpecification::Group& gAccel) : GroupInfo(degree, gPos, gVel, gAccel)
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

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        _interpolator.Initialize(_parameters->GetDOF(), GetEnv()->GetId());
        _translationInterpolator.Initialize(3, GetEnv()->GetId());
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
    // TODO
    dReal _ComputeMinimumTimeJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData)
    {
        return 0;
    }

    // TODO
    dReal _ComputeMinimumTimeAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData)
    {
        return 0;
    }

    // TODO
    dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData)
    {
        return 0;
    }

    //
    // _CheckX functions
    //
    // TODO
    bool _CheckJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        return true;
    }

    // TODO
    bool _CheckAffine(GroupInfoConstPtr info, int affineDofs, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        return true;
    }

    // TODO
    bool _CheckIk(GroupInfoConstPtr info, IkParameterizationType ikType, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::const_iterator itData, int checkOptions=0xffffffff)
    {
        return true;
    }

    //
    // _WriteX functions
    //
    // TODO
    bool _WriteJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itOrgDiff, std::vector<dReal>::const_iterator itDataPrev, std::vector<dReal>::iterator itData)
    {
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


    //
    // Members
    //
    std::string _trajXmlId;
    Piecewisepolynomials::QuinticInterpolator _interpolator, _translationInterpolator, _ikInterpolator;
}; // end class QuinticTrajectoryRetimer

PlannerBasePtr CreateQuinticTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new QuinticTrajectoryRetimer(penv, sinput));
}

} // end namespace rplanners
