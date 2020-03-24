// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "jsoncommon.h"

void SerializeDiffJSON(const KinBody::LinkInfo& lhs, const KinBody::LinkInfo& rhs, rapidjson::Document::AllocatorType allocator, rapidjson::Value& value)
{
    OpenRAVE::JSON::SetJsonValueByKey(value, "name", lhs._name, allocator);

    if (lhs._transform != rhs._transform) {
        Transform tmpTransform {_t};
        tmpTransform.trans *= fUnitScale;
        OpenRAVE::JSON::SetJsonValueByKey(value, "transform", tmpTransform, allocator);
    }

    if (lhs._massTransform != rhs._massTransform) {
        Transform tmpMassTransform {_tMassFrame};
        tmpMassTransform.trans *= fUnitScale;
        OpenRAVE::JSON::SetJsonValueByKey(value, "massTransform", tmpMassTransform, allocator);
    }

    if (lhs._mass != rhs.mass) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "mass", lhs._mass, allocator);
    }
    if (lhs._vinertiamoments != rhs._vinertiamoments) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "intertialMoments", lhs._vinertiamoments, allocator);
    }

}
