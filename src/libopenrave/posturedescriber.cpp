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
#include <openrave/posturedescriber.h>

namespace OpenRAVE {

PostureDescriberBase::PostureDescriberBase(EnvironmentBasePtr penv) : InterfaceBase(PT_PostureDescriber, penv)
{
}

PostureDescriberBase::~PostureDescriberBase() {
}

const char* PostureDescriberBase::GetHash() const {
    return OPENRAVE_POSTUREDESCRIBER_HASH;
}

bool PostureDescriberBase::Supports(const RobotBase::ManipulatorPtr& pmanip) const {
    const std::array<RobotBase::LinkPtr, 2> kinematicsChain {pmanip->GetBase(), pmanip->GetEndEffector()};
    return this->Supports(kinematicsChain);
}

bool PostureDescriberBase::Init(const RobotBase::ManipulatorPtr& pmanip) {
    const std::array<RobotBase::LinkPtr, 2> kinematicsChain {pmanip->GetBase(), pmanip->GetEndEffector()};
    return this->Supports(kinematicsChain) ? this->Init(kinematicsChain) : false;
}

}
