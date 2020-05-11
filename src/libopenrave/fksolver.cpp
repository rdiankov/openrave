// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Rosen Diankov (rosen.diankov@gmail.com)
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

RobotPostureDescriberBase::RobotPostureDescriberBase(EnvironmentBasePtr penv) : InterfaceBase(PT_ForwardKinematicsSolver, penv)
{
}

RobotPostureDescriberBase::~RobotPostureDescriberBase() {
}

const char* RobotPostureDescriberBase::GetHash() const {
    return OPENRAVE_FORWARDKINEMATICSSOLVER_HASH;
}

bool RobotPostureDescriberBase::Init(const std::array<OpenRAVE::RobotBase::LinkPtr, 2>& kinematicsChain){
    return true; // TO-DO
}

InterfaceType RobotPostureDescriberBase::GetInterfaceTypeStatic() {
    return PT_ForwardKinematicsSolver;
}

const std::vector<KinBody::JointPtr>& RobotPostureDescriberBase::GetJoints() const {
	return _vjoints;
}

bool RobotPostureDescriberBase::Supports(const std::array<OpenRAVE::RobotBase::LinkPtr, 2>& kinematicsChain) const {
    return true; // TO-DO
}


bool RobotPostureDescriberBase::ComputePostureValue(std::vector<uint16_t>& values) const {
    return true; // TO-DO
}

}
