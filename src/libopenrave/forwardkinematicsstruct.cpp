// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Guangning Tan (tgntanguangning@gmail.com), Rosen Diankov (rosen.diankov@gmail.com)
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
#include <algorithm>

namespace OpenRAVE {

KinBody::ForwardKinematicsStruct::ForwardKinematicsStruct () {
    pSetSingleLinkTransformFn = KinBody::ForwardKinematicsStruct::_SetSingleLinkTransform;
}

void KinBody::ForwardKinematicsStruct::_SetSingleLinkTransform(Link& link, const Transform& t) {
    link._info._t = t;
}

bool KinBody::_DeriveCurrentForwardKinematicsStruct(bool bOverWrite) {
    if(bOverWrite) {
        _pCurrentForwardKinematicsStruct.reset();
    }
    if(!!_pCurrentForwardKinematicsStruct) {
        return true;
    }
    const std::string& sKinematicsGeometry = this->GetKinematicsGeometryHash();
    const std::map<std::string, ForwardKinematicsStruct>::iterator it = _mHash2ForwardKinematicsStruct.find(sKinematicsGeometry);
    if(it != _mHash2ForwardKinematicsStruct.end()) {
        _pCurrentForwardKinematicsStruct.reset(&it->second, utils::null_deleter());
        return true;
    }
    return false;
}

bool KinBody::RegisterForwardKinematicsStruct(const ForwardKinematicsStruct& fkstruct, const bool bOverWrite) {
    const std::string& sKinematicsGeometry = this->GetKinematicsGeometryHash();
    const bool bRegistered = _mHash2ForwardKinematicsStruct.count(sKinematicsGeometry);
    if(bRegistered) {
        RAVELOG_DEBUG_FORMAT("Already registered ForwardKinematicsStruct at body \"%s\" with hash \"%s\"",
            this->GetName() % sKinematicsGeometry
        );
        if(!bOverWrite) {
            return true; // do not overwrite, so return
        }
        RAVELOG_WARN_FORMAT("Requested to replace the registered ForwardKinematicsStruct by a new one at body \"%s\" with hash \"%s\"",
            this->GetName() % sKinematicsGeometry
        );
    }
    const bool bCheck = (
        !!fkstruct.pCalculatorModule 
        && !!fkstruct.pSetLinkTransformsFn
    );
    if(!bCheck) {
        RAVELOG_ERROR_FORMAT("Does not pass check for ForwardKinematicsStruct at body \"%s\" with hash \"%s\"",
            this->GetName() % sKinematicsGeometry
        );
    }
    else {
        _mHash2ForwardKinematicsStruct[sKinematicsGeometry] = fkstruct;
    }
    return bCheck;
}

} // end namespace OpenRAVE