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
    _unitInfo.lengthUnit = LU_Meter;
    _unitInfo.angleUnit = AU_Radian;
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
    orjson::SetJsonValueByKey(rEnvInfo, "unitInfo", _unitInfo, allocator);
    orjson::SetJsonValueByKey(rEnvInfo, "gravity", _gravity, allocator);
    if( !_referenceUri.empty() ) {
        orjson::SetJsonValueByKey(rEnvInfo, "referenceUri", _referenceUri, allocator);
    }
    if( !_uri.empty() ) {
        orjson::SetJsonValueByKey(rEnvInfo, "uri", _uri, allocator);
    }

    if( _uInt64Parameters.size() > 0 ) {
        rapidjson::Value rUInt64Parameters;
        rUInt64Parameters.SetArray();
        FOREACHC(it, _uInt64Parameters) {
            rapidjson::Value rInt64Parameter;
            rInt64Parameter.SetObject();
            orjson::SetJsonValueByKey(rInt64Parameter, "id", it->first, allocator);
            orjson::SetJsonValueByKey(rInt64Parameter, "value", it->second, allocator);
            rUInt64Parameters.PushBack(rInt64Parameter, allocator);
        }
        rEnvInfo.AddMember("uint64Parameters", rUInt64Parameters, allocator);
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

/// Actual update logic for a single body info json record within DeserializeJSONWithMapping.
/// InfoType must be one of RobotBase::RobotBaseInfo or KinBody::KinBodyInfo, depending on whether rKinBodyInfo describes a robot or a body.
template <typename InfoType>
void _DeserializeKinBodyInfo(const rapidjson::Value& rKinBodyInfo, const float fUnitScale, const int options, const bool isDeleted,
                             const int existingBodyIndex, const string_view& id,
                             std::vector<KinBody::KinBodyInfoPtr>& vBodyInfos,
                             std::unordered_map<string_view, int>& existingBodyInfoIndicesById,
                             std::unordered_map<string_view, int>& existingBodyInfoIndicesByName)
{
    // Pull out whether or not we expect to be creating a robot or body info
    const bool newTypeIsRobot = std::is_same<InfoType, RobotBase::RobotBaseInfo>::value;

    // If we didn't have a matching entry, we may need to just create a new one
    if (existingBodyIndex == -1) {
        // If the new record isn't deleted but is missing a name, we don't want to read it
        boost::shared_ptr<InfoType> pBaseInfo(new InfoType());
        pBaseInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, options);
        if (pBaseInfo->_name.empty()) {
            RAVELOG_WARN_FORMAT("new %s id='%s' does not have a name, so skip creating", (newTypeIsRobot ? "robot" : "body")%id);
            return;
        }

        // If the record is valid, create a new info and be done
        pBaseInfo->_id.assign(id.data(), id.size());
        const int bodyInfoIndex = (int)vBodyInfos.size(); // Stash the index we will be inserting this record at
        vBodyInfos.push_back(pBaseInfo);
        RAVELOG_VERBOSE_FORMAT("created new %s id='%s'", (newTypeIsRobot ? "robot" : "body")%id);

        // Ensure that we update our name/id -> body info mapping, just in case there is a later info that re-updates this body.
        // Note the need to use the info version of the id here since this map uses string_view, and 'id' is a local.
        existingBodyInfoIndicesById.emplace(pBaseInfo->_id, bodyInfoIndex);
        existingBodyInfoIndicesByName.emplace(pBaseInfo->_name, bodyInfoIndex);
        return;
    }

    // If we _did_ have a matching entry, and it's now deleted, then just remove the mapping for this body from our lookup tables and null out that entry in vBodyInfos
    // We don't want to erase here because it will mess with our lookup table - we will do a cleanup pass on vBodyInfos later to remove any holes.
    KinBody::KinBodyInfoPtr& existingBodyInfo = vBodyInfos[existingBodyIndex];
    if (isDeleted) {
        // Erase the lookup table entries for this info record before deletion, since the info is what stores the actual backing string data
        existingBodyInfoIndicesById.erase(existingBodyInfo->_id);
        existingBodyInfoIndicesByName.erase(existingBodyInfo->_name);

        // Clear it from the body list
        RAVELOG_VERBOSE_FORMAT("deleted %s id ='%s'", (newTypeIsRobot ? "robot" : "body")%id);
        existingBodyInfo.reset();
        return;
    }

    // If we have a matching entry, it may be the wrong kind (kinbody instead of robot, or vice versa).
    // If this is the case, we need to recreate with the correct class.
    // Since we preserve the index in the array, no need to modify our lookup tables here
    RobotBase::RobotBaseInfoPtr existingRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(existingBodyInfo);
    const bool mustRecreateInfo = ((!!existingRobotBaseInfo && !newTypeIsRobot) || // If existing info is robot but the new info type isn't
                                   (!existingRobotBaseInfo && newTypeIsRobot) // If existing info isn't robot but new info type is
    );
    if (mustRecreateInfo) {
        // Create a new info struct of the correct type
        boost::shared_ptr<InfoType> newInfoPtr{new InfoType()};

        // Copy across the base kinbody info fields from the old record
        *((KinBody::KinBodyInfo*)newInfoPtr.get()) = *existingBodyInfo;

        // Replace the old record in vBodyInfos
        existingBodyInfo = newInfoPtr;
        RAVELOG_VERBOSE_FORMAT("recreated %s as a %s id='%s'", (newTypeIsRobot ? "body" : "robot")%(newTypeIsRobot ? "robot" : "body")%id);
    }

    // Since it's possible that the name / id of this entry will change as a result of this update,
    // remove the lookup table entry for it pre-update and restore it post-update
    existingBodyInfoIndicesById.erase(existingBodyInfo->_id);
    existingBodyInfoIndicesByName.erase(existingBodyInfo->_name);

    // Update the info struct with the new input json
    existingBodyInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, options);
    existingBodyInfo->_id.assign(id.data(), id.size());

    // Restore the index mapping for this entry
    existingBodyInfoIndicesById.emplace(existingBodyInfo->_id, existingBodyIndex);
    existingBodyInfoIndicesByName.emplace(existingBodyInfo->_name, existingBodyIndex);
}

void EnvironmentBase::EnvironmentBaseInfo::DeserializeJSONWithMapping(const rapidjson::Value& rEnvInfo, dReal fUnitScale, int options, const std::vector<int>& vInputToBodyInfoMapping)
{
    if( !rEnvInfo.IsObject() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("Passed in JSON '%s' is not a valid EnvironmentInfo object", orjson::DumpJson(rEnvInfo), ORE_InvalidArguments);
    }

    // for DeserializeJSON, there are two possibilities: 1. full json passed in 2. diff json passed in
    // for example, do not clear _vBodyInfos.clear(), since we could be dealing with partial json

    orjson::LoadJsonValueByKey(rEnvInfo, "revision", _revision);
    std::pair<std::string, dReal> unit;
    if (orjson::LoadJsonValueByKey(rEnvInfo, "unit", unit)) {
        _unitInfo.lengthUnit = GetLengthUnitFromString(unit.first, LU_Meter);
    }
    orjson::LoadJsonValueByKey(rEnvInfo, "unitInfo", _unitInfo);
    orjson::LoadJsonValueByKey(rEnvInfo, "keywords", _keywords);
    orjson::LoadJsonValueByKey(rEnvInfo, "description", _description);
    orjson::LoadJsonValueByKey(rEnvInfo, "referenceUri", _referenceUri);
    orjson::LoadJsonValueByKey(rEnvInfo, "uri", _uri);
    orjson::LoadJsonValueByKey(rEnvInfo, "gravity", _gravity);

    {
        rapidjson::Value::ConstMemberIterator itModifiedAt = rEnvInfo.FindMember("modifiedAt");
        if( itModifiedAt != rEnvInfo.MemberEnd() && itModifiedAt->value.IsString() ) {
            const char *const modifiedAt = itModifiedAt->value.GetString();
            if ( modifiedAt != nullptr ) {
                _lastModifiedAtUS = ConvertIsoFormatDateTimeToLinuxTimeUS(modifiedAt);
            }
        }
    }

    orjson::LoadJsonValueByKey(rEnvInfo, "revisionId", _revisionId);

    if (rEnvInfo.HasMember("uint64Parameters") && rEnvInfo["uint64Parameters"].IsArray()) {
        for (rapidjson::Value::ConstValueIterator it = rEnvInfo["uint64Parameters"].Begin(); it != rEnvInfo["uint64Parameters"].End(); ++it) {
            std::string id;
            if( it->HasMember("id") ) {
                orjson::LoadJsonValueByKey(*it, "id", id);
            }
            if (id.empty()) {
                RAVELOG_WARN_FORMAT("ignored an entry in uint64Parameters in environment \"%s\" due to missing or empty id", _uri);
                continue;
            }
            // delete
            if (OpenRAVE::orjson::GetJsonValueByKey<bool>(*it, "__deleted__", false)) {
                _uInt64Parameters.erase(id);
                continue;
            }
            orjson::LoadJsonValueByKey(*it, "value", _uInt64Parameters[id]);
        }
    }

    if (rEnvInfo.HasMember("bodies")) {
        // Reserve space for the additional body infos, assuming they're all unique
        const rapidjson::Value& rBodies = rEnvInfo["bodies"];
        BOOST_ASSERT(rBodies.IsArray());
        _vBodyInfos.reserve(_vBodyInfos.size() + rBodies.Size());

        // Build a lookup table of name/id to existing body info index in _vBodyInfos
        // Later, when checking if a new json record should be merged with an existing body info, we can do one hash lookup instead of a sequential scan
        std::unordered_map<string_view, int> existingBodyInfoIndicesById;
        std::unordered_map<string_view, int> existingBodyInfoIndicesByName;
        for (int infoIndex = 0; infoIndex < (int)_vBodyInfos.size(); infoIndex++) {
            const KinBody::KinBodyInfoPtr& pExistingBodyInfo = _vBodyInfos[infoIndex];
            existingBodyInfoIndicesById.emplace(pExistingBodyInfo->_id, infoIndex);
            existingBodyInfoIndicesByName.emplace(pExistingBodyInfo->_name, infoIndex);
        }

        // For each record in the input, check to see if we can map it to an existing body info
        for (int iInputBodyIndex = 0; iInputBodyIndex < (int)rBodies.Size(); ++iInputBodyIndex) {
            const rapidjson::Value& rKinBodyInfo = rBodies[iInputBodyIndex];
            const string_view id = orjson::GetCStringViewJsonValueByKey(rKinBodyInfo, "id");
            bool isDeleted = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "__deleted__", false);

            // Do we have an explicit mapping for this body?
            int existingBodyIndex = -1;
            if (iInputBodyIndex < (int)vInputToBodyInfoMapping.size() && vInputToBodyInfoMapping[iInputBodyIndex] >= 0) {
                existingBodyIndex = vInputToBodyInfoMapping[iInputBodyIndex];
            }

            // Do we have an existing record with the same ID?
            else if (!id.empty()) {
                const std::unordered_map<string_view, int>::const_iterator existingIdIt = existingBodyInfoIndicesById.find(id);
                if (existingIdIt != existingBodyInfoIndicesById.end()) {
                    existingBodyIndex = existingIdIt->second;
                }
            }

            // Do we have an existing record with the same name?
            else {
                rapidjson::Value::ConstMemberIterator itName = rKinBodyInfo.FindMember("name");
                if (itName != rKinBodyInfo.MemberEnd() && itName->value.IsString()) {
                    const std::unordered_map<string_view, int>::const_iterator existingNameIt = existingBodyInfoIndicesByName.find(itName->value.GetString());
                    if (existingNameIt != existingBodyInfoIndicesByName.end()) {
                        existingBodyIndex = existingNameIt->second;
                    }
                }
            }

            // If we don't have an existing entry, and the new entry is deleted, then this is a no-op.
            if (existingBodyIndex == -1 && isDeleted) {
                continue;
            }

            // If we found an existing entry, emit a log on which one we are going to update
            bool isExistingRobot = false;
            if (existingBodyIndex >= 0) {
                const KinBody::KinBodyInfoPtr& existingBodyInfo = _vBodyInfos[existingBodyIndex];
                isExistingRobot = !!OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(existingBodyInfo);
                RAVELOG_VERBOSE_FORMAT("found existing body '%s' with id='%s', isRobot = %d", existingBodyInfo->_name%id%isExistingRobot);
            }

            // here we allow body infos with empty id to be created because
            // when we load things from json, some id could be missing on file

            bool isRobot = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "isRobot", isExistingRobot);
            RAVELOG_VERBOSE_FORMAT("body id='%s', isRobot=%d", id%isRobot);

            // If the new body info is for a robot, we need to parse the info using a RobotBase::RobotBaseInfo.
            // If it isn't, we should just us a KinBody::KinBodyInfo.
            // The update logic for both is pretty much the same, so we can just template it out.
            if (isRobot) {
                _DeserializeKinBodyInfo<RobotBase::RobotBaseInfo>(
                    rKinBodyInfo, fUnitScale, options, isDeleted, existingBodyIndex, id,
                    /* mutable */ _vBodyInfos, /* mutable */ existingBodyInfoIndicesById, /* mutable */ existingBodyInfoIndicesByName);
            }
            else {
                _DeserializeKinBodyInfo<KinBody::KinBodyInfo>(
                    rKinBodyInfo, fUnitScale, options, isDeleted, existingBodyIndex, id,
                    /* mutable */ _vBodyInfos, /* mutable */ existingBodyInfoIndicesById, /* mutable */ existingBodyInfoIndicesByName);
            }
        }

        // Now that we're done processing, filter _vBodyInfos to remove any holes left from deleting bodies
        _vBodyInfos.erase(std::remove(_vBodyInfos.begin(), _vBodyInfos.end(), KinBody::KinBodyInfoPtr()), _vBodyInfos.end());
    }
}
