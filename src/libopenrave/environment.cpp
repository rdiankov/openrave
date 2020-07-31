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

    if (value.HasMember("bodies")) {
        _vBodyInfos.reserve(_vBodyInfos.size() + value["bodies"].Size());
        size_t iBody = 0;
        for (rapidjson::Value::ConstValueIterator it = value["bodies"].Begin(); it != value["bodies"].End(); ++it, ++iBody) {
            const rapidjson::Value& rKinBodyInfo = *it;

            // first figure an id
            std::string id = orjson::GetStringJsonValueByKey(rKinBodyInfo, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(rKinBodyInfo, "name");
                RAVELOG_DEBUG_FORMAT("used name as id for body: %s", id);
            }
            if (id.empty()) {
                id = boost::str(boost::format("body%d")%iBody);
                RAVELOG_DEBUG_FORMAT("assigned new id for body: %s", id);
            }

            // then find previous body
            bool isExistingRobot = false;
            std::vector<KinBody::KinBodyInfoPtr>::iterator itExistingBodyInfo = _vBodyInfos.end();
            FOREACH(itBodyInfo, _vBodyInfos) {
                if ((*itBodyInfo)->_id == id ) {
                    itExistingBodyInfo = itBodyInfo;
                    isExistingRobot = !!OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(*itBodyInfo);
                    RAVELOG_DEBUG_FORMAT("found existing body: %s, isRobot = %d", id%isExistingRobot);
                    break;
                }
            }

            bool isDeleted = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "__deleted__", false);
            if (isDeleted) {
                RAVELOG_DEBUG_FORMAT("deleted body: %s", id);
            }

            bool isRobot = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "isRobot", isExistingRobot);
            RAVELOG_DEBUG_FORMAT("body '%s', isRobot=%d", id%isRobot);
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
                    RAVELOG_DEBUG_FORMAT("replaced body as a robot: %s", id);
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
                    RAVELOG_DEBUG_FORMAT("replaced robot as a body: %s", id);
                }
                pKinBodyInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, options);
                pKinBodyInfo->_id = id;
            }
        }
    }
}
