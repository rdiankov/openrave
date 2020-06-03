// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Guangning Tan, Kei Usui & Rosen Diankov (rosen.diankov@gmail.com)
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
#include <openrave/posturedescriber.h>

namespace OpenRAVE {

bool RobotBase::UnregisterPostureDescriber(const LinkPair& kinematicsChain) {
    return this->SetPostureDescriber(kinematicsChain, nullptr);
}

bool RobotBase::UnregisterPostureDescriber(ManipulatorConstPtr pmanip) {
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    return this->UnregisterPostureDescriber(GetEssentialKinematicsChain(pmanip));
}


bool RobotBase::SetPostureDescriber(const LinkPair& kinematicsChain, PostureDescriberBasePtr pDescriber)
{
    if (pDescriber == nullptr) {
        if (_mPostureDescribers.count(kinematicsChain)) {
            _mPostureDescribers.erase(kinematicsChain); // remove instead of setting null solver
        }
        return true;
    }

    if (pDescriber->Supports(kinematicsChain)) {
        _mPostureDescribers[kinematicsChain] = pDescriber;
        return true;
    }
    return false;
}

bool RobotBase::SetPostureDescriber(ManipulatorConstPtr pmanip, PostureDescriberBasePtr pDescriber)
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    return this->SetPostureDescriber(GetEssentialKinematicsChain(pmanip), pDescriber);
}

PostureDescriberBasePtr RobotBase::GetPostureDescriber(const LinkPair& kinematicsChain) const
{
    return _mPostureDescribers.count(kinematicsChain) ? _mPostureDescribers.at(kinematicsChain) : PostureDescriberBasePtr();
}

PostureDescriberBasePtr RobotBase::GetPostureDescriber(ManipulatorConstPtr pmanip) const
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    return this->GetPostureDescriber(GetEssentialKinematicsChain(pmanip));
}

bool RobotBase::ComputePostureStates(std::vector<PostureStateInt>& posturestates, const LinkPair& kinematicsChain, const std::vector<double>& dofvalues) const
{
    if(!_mPostureDescribers.count(kinematicsChain)) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find robot posture describer for links from \"%s\" to \"%s\" for robot \"%s\""),
                                        GetName() % kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName(), ORE_InvalidArguments);
    }
    return _mPostureDescribers.at(kinematicsChain)->ComputePostureStates(posturestates, dofvalues);
}

bool RobotBase::ComputePostureStates(std::vector<PostureStateInt>& posturestates, ManipulatorConstPtr pmanip, const std::vector<double>& dofvalues) const
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    return this->ComputePostureStates(posturestates, GetEssentialKinematicsChain(pmanip), dofvalues);
}

} // end namespace OpenRAVE
