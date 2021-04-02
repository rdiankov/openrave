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

EnvironmentBase::EnvironmentBaseInfo::EnvironmentBaseInfo()
{
    _gravity = Vector(0,0,-9.797930195020351);
}

EnvironmentBase::EnvironmentBaseInfo::EnvironmentBaseInfo(const EnvironmentBaseInfo& other)
{
    *this = other;
}

bool EnvironmentBase::EnvironmentBaseInfo::operator==(const EnvironmentBaseInfo& other) const
{
    return _vBodyInfos == other._vBodyInfos
           && _revision == other._revision
           && _description == other._description
           && _keywords == other._keywords
           && _gravity == other._gravity
           && _uri == other._uri
           && _referenceUri == other._referenceUri;
    // TODO: deep compare infos
}

bool EnvironmentBase::EnvironmentBaseInfo::operator!=(const EnvironmentBaseInfo& other) const
{
    return !operator==(other);
}

void EnvironmentBase::EnvironmentBaseInfo::Reset()
{
    _description.clear();
    _keywords.clear();
    _gravity = Vector(0,0,-9.797930195020351);
    _uri.clear();
    _referenceUri.clear();
    _vBodyInfos.clear();
    _revision = 0;
}

void EnvironmentBase::EnvironmentBaseInfo::SerializeJSON(rapidjson::Value& rEnvInfo, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    // for all SerializeJSON, we clear the output
    rEnvInfo.SetObject();

    orjson::SetJsonValueByKey(rEnvInfo, "keywords", _keywords, allocator);
    if( !_description.empty() ) {
        orjson::SetJsonValueByKey(rEnvInfo, "description", _description, allocator);
    }
    orjson::SetJsonValueByKey(rEnvInfo, "gravity", _gravity, allocator);
    if( !_referenceUri.empty() ) {
        orjson::SetJsonValueByKey(rEnvInfo, "referenceUri", _referenceUri, allocator);
    }
    if( !_uri.empty() ) {
        orjson::SetJsonValueByKey(rEnvInfo, "uri", _uri, allocator);
    }

    if (_vBodyInfos.size() > 0) {
        rapidjson::Value rBodiesValue;
        rBodiesValue.SetArray();
        rBodiesValue.Reserve(_vBodyInfos.size(), allocator);
        for (const KinBody::KinBodyInfoPtr& pinfo : _vBodyInfos) {
            if (!pinfo) {
                continue;
            }
            rapidjson::Value bodyValue;
            pinfo->SerializeJSON(bodyValue, allocator, fUnitScale, options);
            rBodiesValue.PushBack(bodyValue, allocator);
        }
        rEnvInfo.AddMember("bodies", rBodiesValue, allocator);
    }
}

void EnvironmentBase::EnvironmentBaseInfo::DeserializeJSON(const rapidjson::Value& rEnvInfo, dReal fUnitScale, int options)
{
    std::vector<int> vInputToBodyInfoMapping;
    DeserializeJSONWithMapping(rEnvInfo, fUnitScale, options, vInputToBodyInfoMapping);
}

void EnvironmentBase::EnvironmentBaseInfo::DeserializeJSONWithMapping(const rapidjson::Value& rEnvInfo, dReal fUnitScale, int options, const std::vector<int>& vInputToBodyInfoMapping)
{
    if( !rEnvInfo.IsObject() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("Passed in JSON '%s' is not a valid EnvironmentInfo object", orjson::DumpJson(rEnvInfo), ORE_InvalidArguments);
    }

    // for DeserializeJSON, there are two possibilities: 1. full json passed in 2. diff json passed in
    // for example, do not clear _vBodyInfos.clear(), since we could be dealing with partial json

    if (rEnvInfo.HasMember("revision")) {
        orjson::LoadJsonValueByKey(rEnvInfo, "revision", _revision);
    }

    if (rEnvInfo.HasMember("keywords")) {
        orjson::LoadJsonValueByKey(rEnvInfo, "keywords", _keywords);
    }

    if (rEnvInfo.HasMember("description")) {
        orjson::LoadJsonValueByKey(rEnvInfo, "description", _description);
    }
    if( rEnvInfo.HasMember("referenceUri") ) {
        orjson::LoadJsonValueByKey(rEnvInfo, "referenceUri", _referenceUri);
    }
    if( rEnvInfo.HasMember("uri") ) {
        orjson::LoadJsonValueByKey(rEnvInfo, "uri", _uri);
    }

    if (rEnvInfo.HasMember("gravity")) {
        orjson::LoadJsonValueByKey(rEnvInfo, "gravity", _gravity);
    }


    if (rEnvInfo.HasMember("bodies")) {
        _vBodyInfos.reserve(_vBodyInfos.size() + rEnvInfo["bodies"].Size());
        const rapidjson::Value& rBodies = rEnvInfo["bodies"];
        for(int iInputBodyIndex = 0; iInputBodyIndex < (int)rBodies.Size(); ++iInputBodyIndex) {
            const rapidjson::Value& rKinBodyInfo = rBodies[iInputBodyIndex];

            std::string id = orjson::GetStringJsonValueByKey(rKinBodyInfo, "id");
            bool isDeleted = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "__deleted__", false);

            // then find previous body
            bool isExistingRobot = false;
            std::vector<KinBody::KinBodyInfoPtr>::iterator itExistingBodyInfo = _vBodyInfos.end();

            if( iInputBodyIndex < (int)vInputToBodyInfoMapping.size() && vInputToBodyInfoMapping[iInputBodyIndex] >= 0 ) {
                itExistingBodyInfo = _vBodyInfos.begin() + vInputToBodyInfoMapping[iInputBodyIndex];
            }
            else if (!id.empty()) {
                // only try to find old info if id is not empty
                FOREACH(itBodyInfo, _vBodyInfos) {
                    if ((*itBodyInfo)->_id == id ) {
                        itExistingBodyInfo = itBodyInfo;
                        break;
                    }
                }
            }

            if( itExistingBodyInfo != _vBodyInfos.end() ) {
                isExistingRobot = !!OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(*itExistingBodyInfo);
                RAVELOG_VERBOSE_FORMAT("found existing body '%s' with id='%s', isRobot = %d", (*itExistingBodyInfo)->_name%id%isExistingRobot);
            }

            // here we allow body infos with empty id to be created because
            // when we load things from json, some id could be missing on file
            // and for the partial update case, the id should be non-empty

            bool isRobot = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "isRobot", isExistingRobot);
            RAVELOG_VERBOSE_FORMAT("body id='%s', isRobot=%d", id%isRobot);
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
                    RAVELOG_VERBOSE_FORMAT("deleted robot id ='%s'", id);
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
                    RAVELOG_VERBOSE_FORMAT("replaced body as a robot id='%s'", id);
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
                        RAVELOG_VERBOSE_FORMAT("created new body id='%s'", id);
                    }
                    continue;
                }
                // in case same id exists before
                if (isDeleted) {
                    RAVELOG_VERBOSE_FORMAT("deleted body id='%s'", id);
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
                    RAVELOG_VERBOSE_FORMAT("replaced robot as a body id='%s'", id);
                }
                pKinBodyInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, options);
                pKinBodyInfo->_id = id;
            }
        }
    }
}
