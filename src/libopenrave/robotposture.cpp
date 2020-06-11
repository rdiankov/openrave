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

void RobotBase::_EnsureEssentialKinematicsChainRegisteredOnManipulator(ManipulatorConstPtr pmanip) {
    if(!_mEssentialLinkPairs.count(pmanip)) {
        _mEssentialLinkPairs[pmanip] = ExtractEssentialKinematicsChain(pmanip);
    }
}

bool RobotBase::UnregisterPostureDescriber(const LinkPair& kinematicsChain) {
    return this->SetPostureDescriber(kinematicsChain, nullptr);
}

bool RobotBase::UnregisterPostureDescriber(ManipulatorConstPtr pmanip) {
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    _EnsureEssentialKinematicsChainRegisteredOnManipulator(pmanip);
    return this->UnregisterPostureDescriber(_mEssentialLinkPairs.at(pmanip));
}

bool RobotBase::SetPostureDescriber(const LinkPair& kinematicsChain, PostureDescriberBasePtr pDescriber)
{
    if (pDescriber == nullptr) {
        if (_mPostureDescribers.count(kinematicsChain)) {
            _mPostureDescribers.erase(kinematicsChain); // remove instead of setting null solver
        }
        return true;
    }

    const LinkPtr& baselink = kinematicsChain[0];
    const LinkPtr& eelink = kinematicsChain[1];

    if (!pDescriber->Supports(kinematicsChain)) {
        throw OPENRAVE_EXCEPTION_FORMAT("Describer does not support kinematics chain from \"%s\" to eelink \"%s\"",
                                        baselink->GetName() % eelink->GetName(),
                                        OpenRAVEErrorCode::ORE_InvalidArguments);
    }

    const LinkPair& kinematicsChainDescribed = pDescriber->GetEssentialKinematicsChain();
    if(kinematicsChainDescribed != kinematicsChain) {
        throw OPENRAVE_EXCEPTION_FORMAT("Kinematics chains do not match: describer has baselink \"%s\" and eelink \"%s\"; "
                                        "input has baselink \"%s\" and eelink \"%s\"",
                                        kinematicsChainDescribed[0]->GetName() % kinematicsChainDescribed[1]->GetName() %
                                        baselink->GetName() % eelink->GetName(),
                                        OpenRAVEErrorCode::ORE_InvalidArguments);
    }

    _mPostureDescribers[kinematicsChain] = pDescriber;
    return true;
}

bool RobotBase::SetPostureDescriber(ManipulatorConstPtr pmanip, PostureDescriberBasePtr pDescriber)
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    _EnsureEssentialKinematicsChainRegisteredOnManipulator(pmanip);
    return this->SetPostureDescriber(_mEssentialLinkPairs.at(pmanip), pDescriber);
}

PostureDescriberBasePtr RobotBase::GetPostureDescriber(const LinkPair& kinematicsChain)
{
    if(_mPostureDescribers.count(kinematicsChain)) {
        return _mPostureDescribers.at(kinematicsChain);
    }
    const LinkPair essentialKinematicsChain = ExtractEssentialKinematicsChain(kinematicsChain);
    return _mPostureDescribers.count(essentialKinematicsChain) ? _mPostureDescribers.at(essentialKinematicsChain) : PostureDescriberBasePtr();
}

PostureDescriberBasePtr RobotBase::GetPostureDescriber(ManipulatorConstPtr pmanip)
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be nullptr", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    _EnsureEssentialKinematicsChainRegisteredOnManipulator(pmanip);
    return this->GetPostureDescriber(_mEssentialLinkPairs.at(pmanip));
}

bool RobotBase::ComputePostureStates(std::vector<PostureStateInt>& posturestates, const LinkPair& kinematicsChain, const std::vector<double>& dofvalues) const
{
    if(!_mPostureDescribers.count(kinematicsChain)) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find robot posture describer for links from \"%s\" to \"%s\" for robot \"%s\""),
                                        GetName() % kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName(), OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    const PostureDescriberBasePtr& pDescriber = _mPostureDescribers.at(kinematicsChain);
    const LinkPair& kinematicsChainDescribed = pDescriber->GetEssentialKinematicsChain();
    if(kinematicsChainDescribed != kinematicsChain) {
        throw OPENRAVE_EXCEPTION_FORMAT("Kinematics chains do not match: describer has baselink \"%s\" and eelink \"%s\"; "
                                        "input has baselink \"%s\" and eelink \"%s\"",
                                        kinematicsChainDescribed[0]->GetName() % kinematicsChainDescribed[1]->GetName() %
                                        kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName(),
                                        OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    return pDescriber->ComputePostureStates(posturestates, dofvalues);
}

bool RobotBase::ComputePostureStates(std::vector<PostureStateInt>& posturestates, ManipulatorConstPtr pmanip, const std::vector<double>& dofvalues) const
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    if(!_mEssentialLinkPairs.count(pmanip)) {
        throw OPENRAVE_EXCEPTION_FORMAT("Have not included the mapping from manipulator %s to its essential kinematics chain yet",
                                        pmanip->GetName(),
                                        OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    return this->ComputePostureStates(posturestates, _mEssentialLinkPairs.at(pmanip), dofvalues);
}

} // end namespace OpenRAVE
