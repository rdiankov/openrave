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

void EnvironmentBase::EnvironmentBaseInfo::Reset()
{
    _vBodyInfos.clear();
    _revision = 0;
}

void EnvironmentBase::EnvironmentBaseInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    // for all SerializeJSON, we clear the output
    value.SetObject();

    orjson::SetJsonValueByKey(value, "name", _name, allocator);
    orjson::SetJsonValueByKey(value, "keywords", _keywords, allocator);
    orjson::SetJsonValueByKey(value, "description", _description, allocator);
    orjson::SetJsonValueByKey(value, "gravity", _gravity, allocator);

    if (_vBodyInfos.size() > 0) {
        rapidjson::Value rBodiesValue;
        rBodiesValue.SetArray();
        rBodiesValue.Reserve(_vBodyInfos.size(), allocator);
        FOREACHC(it, _vBodyInfos) {
            rapidjson::Value bodyValue;
            (*it)->SerializeJSON(bodyValue, allocator, fUnitScale, options);
            rBodiesValue.PushBack(bodyValue, allocator);
        }
        value.AddMember("bodies", rBodiesValue, allocator);
    }
}

void EnvironmentBase::EnvironmentBaseInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    // for DeserializeJSON, there are two possibilities: 1. full json passed in 2. diff json passed in
    // for example, do not clear _vBodyInfos.clear(), since we could be dealing with partial json

    if (value.HasMember("revision")) {
        orjson::LoadJsonValueByKey(value, "revision", _revision);
    }

    if (value.HasMember("name")) {
        orjson::LoadJsonValueByKey(value, "name", _name);
    }

    if (value.HasMember("keywords")) {
        orjson::LoadJsonValueByKey(value, "keywords", _keywords);
    }

    if (value.HasMember("description")) {
        orjson::LoadJsonValueByKey(value, "description", _description);
    }

    if (value.HasMember("gravity")) {
        orjson::LoadJsonValueByKey(value, "gravity", _gravity);
    }

    if (value.HasMember("bodies")) {
        _vBodyInfos.reserve(_vBodyInfos.size() + value["bodies"].Size());
        for (rapidjson::Value::ConstValueIterator it = value["bodies"].Begin(); it != value["bodies"].End(); ++it) {
            const rapidjson::Value& rKinBodyInfo = *it;

            std::string id = orjson::GetStringJsonValueByKey(rKinBodyInfo, "id");
            bool isDeleted = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "__deleted__", false);

            // then find previous body
            bool isExistingRobot = false;
            std::vector<KinBody::KinBodyInfoPtr>::iterator itExistingBodyInfo = _vBodyInfos.end();
            if (!id.empty()) {
                FOREACH(itBodyInfo, _vBodyInfos) {
                    if ((*itBodyInfo)->_id == id ) {
                        itExistingBodyInfo = itBodyInfo;
                        isExistingRobot = !!OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(*itBodyInfo);
                        RAVELOG_VERBOSE_FORMAT("found existing body: %s, isRobot = %d", id%isExistingRobot);
                        break;
                    }
                }
            }

            bool isRobot = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "isRobot", isExistingRobot);
            RAVELOG_VERBOSE_FORMAT("body '%s', isRobot=%d", id%isRobot);
            if (isRobot) {
                if (itExistingBodyInfo == _vBodyInfos.end()) {
                    // in case no such id
                    if (!isDeleted) {
                        RobotBase::RobotBaseInfoPtr pRobotBaseInfo(new RobotBase::RobotBaseInfo());
                        pRobotBaseInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, options);
                        pRobotBaseInfo->_id = id;
                        _vBodyInfos.push_back(pRobotBaseInfo);
                    }
                    continue;
                }
                // in case same id exists before
                if (isDeleted) {
                    RAVELOG_VERBOSE_FORMAT("deleted robot: %s", id);
                    _vBodyInfos.erase(itExistingBodyInfo);
                    continue;
                }
                KinBody::KinBodyInfoPtr pKinBodyInfo = *itExistingBodyInfo;
                RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
                if (!pRobotBaseInfo) {
                    // previous body was not a robot
                    // need to replace with a new RobotBaseInfo
                    pRobotBaseInfo.reset(new RobotBase::RobotBaseInfo());
                    *itExistingBodyInfo = pRobotBaseInfo;
                    *((KinBody::KinBodyInfo*)pRobotBaseInfo.get()) = *pKinBodyInfo;
                    RAVELOG_VERBOSE_FORMAT("replaced body as a robot: %s", id);
                }
                pRobotBaseInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, options);
                pRobotBaseInfo->_id = id;
            }
            else {
                // not a robot
                if (itExistingBodyInfo == _vBodyInfos.end()) {
                    // in case no such id
                    if (!isDeleted) {
                        KinBody::KinBodyInfoPtr pKinBodyInfo(new KinBody::KinBodyInfo());
                        pKinBodyInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, options);
                        pKinBodyInfo->_id = id;
                        _vBodyInfos.push_back(pKinBodyInfo);
                    }
                    continue;
                }
                // in case same id exists before
                if (isDeleted) {
                    RAVELOG_VERBOSE_FORMAT("deleted body: %s", id);
                    _vBodyInfos.erase(itExistingBodyInfo);
                    continue;
                }
                KinBody::KinBodyInfoPtr pKinBodyInfo = *itExistingBodyInfo;
                RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
                if (!!pRobotBaseInfo) {
                    // previous body was a robot
                    // need to replace with a new KinBodyInfo
                    pKinBodyInfo.reset(new KinBody::KinBodyInfo());
                    *itExistingBodyInfo = pKinBodyInfo;
                    *pKinBodyInfo = *((KinBody::KinBodyInfo*)pRobotBaseInfo.get());
                    RAVELOG_VERBOSE_FORMAT("replaced robot as a body: %s", id);
                }
                pKinBodyInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, options);
                pKinBodyInfo->_id = id;
            }
        }
    }
}
