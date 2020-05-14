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

void EnvironmentBase::EnvironmentBaseInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    value.SetObject();

    if (_vKinBodyInfos.size() > 0) {
        rapidjson::Value rBodiesValue;
        rBodiesValue.SetArray();
        rBodiesValue.Reserve(_vKinBodyInfos.size(), allocator);
        FOREACHC(it, _vKinBodyInfos) {
            rapidjson::Value bodyValue;
            (*it)->SerializeJSON(bodyValue, allocator, fUnitScale, options);
            rBodiesValue.PushBack(bodyValue, allocator);
        }
        value.AddMember("bodies", rBodiesValue, allocator);
    }
}

void EnvironmentBase::EnvironmentBaseInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale)
{
    _vKinBodyInfos.clear();
    if (value.HasMember("bodies")) {
        _vKinBodyInfos.reserve(value["bodies"].Size());
        for (size_t iBodyInfo = 0; iBodyInfo < value["bodies"].Size(); iBodyInfo++) {
            if (OpenRAVE::JSON::GetJsonValueByKey<bool>(value["bodies"][iBodyInfo], "isRobot")) {
                RobotBase::RobotBaseInfoPtr pRobotBaseInfo(new RobotBase::RobotBaseInfo());
                pRobotBaseInfo->DeserializeJSON(value["bodies"][iBodyInfo], fUnitScale);
                _vKinBodyInfos.push_back(pRobotBaseInfo);
            } else {
                KinBody::KinBodyInfoPtr pKinBodyInfo(new KinBody::KinBodyInfo());
                pKinBodyInfo->DeserializeJSON(value["bodies"][iBodyInfo], fUnitScale);
                _vKinBodyInfos.push_back(pKinBodyInfo);
            }
        }
    }
}
