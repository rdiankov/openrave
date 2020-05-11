// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov (rosen.diankov@gmail.com)
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

namespace OpenRAVE {

bool RobotBase::SetRobotPostureDescriber(LinkPtr pBaseLink, LinkPtr pEndEffectorLink, RobotPostureDescriberBasePtr pDescriber)
{
    if (!pDescriber) {
        const std::pair<LinkPtr, LinkPtr> kinematicChain(make_pair(pBaseLink, pEndEffectorLink));
        const std::map<std::pair<LinkPtr, LinkPtr>, RobotPostureDescriberBasePtr>::const_iterator itPostureDecriber = _robotPostureDescribers.find(kinematicChain);
        if (itPostureDecriber != _robotPostureDescribers.end())
        {
            // remove instead of setting null solver
            _robotPostureDescribers.erase(itPostureDecriber);
        }
        return true;
    }
        
    if (pDescriber->Supports(pBaseLink, pEndEffectorLink)) {
        const std::pair<LinkPtr, LinkPtr> kinematicChain(make_pair(pBaseLink, pEndEffectorLink));
        _robotPostureDescribers[kinematicChain] = pDescriber;
        return true;
    }
    return false;
}

RobotPostureDescriberBasePtr RobotBase::GetRobotPostureDescriber(LinkPtr pBaseLink, LinkPtr pEndEffectorLink) const
{
    const std::pair<LinkPtr, LinkPtr> kinematicChain(make_pair(pBaseLink, pEndEffectorLink));
    const std::map<std::pair<LinkPtr, LinkPtr>, RobotPostureDescriberBasePtr>::const_iterator itPostureDecriber = _robotPostureDescribers.find(kinematicChain);
    if (itPostureDecriber != _robotPostureDescribers.end()) {
        return itPostureDecriber->second;
    }
    return RobotPostureDescriberBasePtr();
}

bool RobotBase::ComputePostureValue(std::vector<uint16_t>& values) const
{
    ManipulatorConstPtr pmanip = GetActiveManipulator();
    return ComputePostureValue(pmanip->GetBase(), pmanip->GetEndEffector(), values);
}

bool RobotBase::ComputePostureValue(ManipulatorConstPtr pmanip, std::vector<uint16_t>& values) const
{
    return ComputePostureValue(pmanip->GetBase(), pmanip->GetEndEffector(), values);
}

bool RobotBase::ComputePostureValue(LinkPtr pBaseLink, LinkPtr pEndEffectorLink, std::vector<uint16_t>& values) const
{
    // TODO fill with default implementation
    const std::pair<LinkPtr, LinkPtr> kinematicChain(make_pair(pBaseLink, pEndEffectorLink));
    const std::map<std::pair<LinkPtr, LinkPtr>, RobotPostureDescriberBasePtr>::const_iterator itPostureDecriber = _robotPostureDescribers.find(kinematicChain);
    if (itPostureDecriber != _robotPostureDescribers.end()) {
        return itPostureDecriber->second->ComputePostureValue(values);
    }
    
    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find robot posture describer for links from \"%s\" to \"%s\" for robot \"%s\""), GetName()%pBaseLink->GetName()%pEndEffectorLink->GetName(), ORE_InvalidArguments);
}

} // end namespace OpenRAVE
