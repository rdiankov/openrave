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

bool RobotBase::UnregisterRobotPostureDescriber(const std::array<LinkPtr, 2>& kinematicsChain) {
    return this->SetRobotPostureDescriber(kinematicsChain, nullptr);
}

bool RobotBase::SetRobotPostureDescriber(const std::array<LinkPtr, 2>& kinematicsChain, RobotPostureDescriberBasePtr pDescriber)
{
    if (!pDescriber) {
        if (_mRobotPostureDescribers.count(kinematicsChain)) {
            _mRobotPostureDescribers.erase(kinematicsChain); // remove instead of setting null solver
        }
        return true;
    }
        
    if (pDescriber->Supports(kinematicsChain)) {
        _mRobotPostureDescribers[kinematicsChain] = pDescriber;
        return true;
    }
    return false;
}

RobotPostureDescriberBasePtr RobotBase::GetRobotPostureDescriber(const std::array<LinkPtr, 2>& kinematicsChain) const
{
    return _mRobotPostureDescribers.count(kinematicsChain) ? _mRobotPostureDescribers.at(kinematicsChain) : RobotPostureDescriberBasePtr();
}

bool RobotBase::ComputePostureValue(std::vector<uint16_t>& values, ManipulatorConstPtr pmanip) const
{
    if(pmanip == nullptr) {
        pmanip = this->GetActiveManipulator();
    }
    const std::array<LinkPtr, 2> kinematicsChain {pmanip->GetBase(), pmanip->GetEndEffector()};
    return this->ComputePostureValue(values, kinematicsChain);
}

bool RobotBase::ComputePostureValue(std::vector<uint16_t>& values, const std::array<LinkPtr, 2>& kinematicsChain) const
{
    // TODO fill with default implementation
    if(_mRobotPostureDescribers.count(kinematicsChain)) {
        return _mRobotPostureDescribers.at(kinematicsChain)->ComputePostureValue(values);
    }
    
    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find robot posture describer for links from \"%s\" to \"%s\" for robot \"%s\""),
                                    GetName() % kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName(), ORE_InvalidArguments);
    return false;
}

} // end namespace OpenRAVE
