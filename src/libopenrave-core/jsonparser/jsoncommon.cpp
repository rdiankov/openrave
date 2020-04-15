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

namespace OpenRAVE {

// \brief extract info from rapidjson Value and merge it into vector
template<typename T>
void _ExtractInfo(const rapidjson::Value& value, std::vector<OPENRAVE_SHARED_PTR<T>>& infos, dReal fUnitScale) {
    std::string name;
    OpenRAVE::JSON::LoadJsonValueByKey(value, "name", name);
    bool found = false;
    OPENRAVE_SHARED_PTR<T> ref;
    FOREACHC(itRef, infos) {
        if ((*itRef)->_name == name) {
            found = true;
            ref = *itRef;
            infos.erase(itRef);
            break;
        }
    }
    OPENRAVE_SHARED_PTR<T> info(new T());
    if (!found) {
        info->DeserializeJSON(value, fUnitScale);  // TODO: DeserializeJSON  is equal to DeSerializeDiffJSON with an empty refInfo
    }
    else {
        if (value.HasMember("__deleted__") && value["__deleted__"].GetBool() == true) {
            return;
        }
        info->SetReferenceInfo(ref);
        DeserializeDiffJSON(*info, value, fUnitScale);
    }
    infos.push_back(info);
}

template void _ExtractInfo(const rapidjson::Value& value, std::vector<KinBody::LinkInfoPtr>& infos, dReal fUnitScale=1.0);
template void _ExtractInfo(const rapidjson::Value& value, std::vector<KinBody::JointInfoPtr>& infos, dReal fUnitScale=1.0);
template void _ExtractInfo(const rapidjson::Value& value, std::vector<RobotBase::ManipulatorInfoPtr>& infos, dReal fUnitScale=1.0);



void SerializeDiffJSON(const KinBody::GeometryInfo& coverInfo, const KinBody::GeometryInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options)
{
    // TODO
    OpenRAVE::JSON::SetJsonValueByKey(value, "id", coverInfo._id, allocator);
}


void SerializeDiffJSON(const KinBody::JointInfo& coverInfo, const KinBody::JointInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options)
{
    // TODO
    OpenRAVE::JSON::SetJsonValueByKey(value, "id", coverInfo._id, allocator);
}

void SerializeDiffJSON(const RobotBase::ManipulatorInfo& coverInfo, const RobotBase::ManipulatorInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options)
{
    // TODO
    OpenRAVE::JSON::SetJsonValueByKey(value, "id", coverInfo._id, allocator);
}

void SerializeDiffJSON(const RobotBase::AttachedSensorInfo& coverInfo, const RobotBase::AttachedSensorInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) {
    // TODO
    OpenRAVE::JSON::SetJsonValueByKey(value, "id", coverInfo._id, allocator);
}


void SerializeDiffJSON(const RobotBase::ConnectedBodyInfo& coverInfo, const RobotBase::ConnectedBodyInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) {
    // TODO
    OpenRAVE::JSON::SetJsonValueByKey(value, "id", coverInfo._id, allocator);
}

// overwrite refInfo with value and save result into targetInfo
void DeserializeDiffJSON(KinBody::LinkInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale) {
    // TODO assert name/id is equal
    OpenRAVE::JSON::LoadJsonValueByKey(value, "id", targetInfo._id);
    OpenRAVE::JSON::LoadJsonValueByKey(value, "name", targetInfo._name);

    if (value.HasMember("geometries")) {
        targetInfo._vgeometryinfos.reserve(targetInfo._vgeometryinfos.size() + value["geometries"].Size());
        for (rapidjson::Value::ConstValueIterator itr = value["geometries"].Begin(); itr != value["geometries"].End(); ++itr){
            _ExtractInfo(*itr, targetInfo._vgeometryinfos, fUnitScale);
        }
        targetInfo._vgeometryinfos.shrink_to_fit();
    }
    if (value.HasMember("transform")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "transform", targetInfo._t);
        targetInfo._t.trans *= fUnitScale;
    }
    if (value.HasMember("massTransform")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "massTransform", targetInfo._tMassFrame);
        targetInfo._tMassFrame.trans *= fUnitScale;
    }
    if (value.HasMember("mass")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "mass", targetInfo._mass);
    }
    if (value.HasMember("intertialMoments")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "intertialMoments", targetInfo._vinertiamoments);
    }
    if (value.HasMember("floatParameters")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "floatParameters", targetInfo._mapFloatParameters);
    }
    if (value.HasMember("intParameters")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "intParameters", targetInfo._mapIntParameters);
    }
    if (value.HasMember("stringParameters")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "stringParameters", targetInfo._mapStringParameters);
    }
    if (value.HasMember("forcedAdjacentLinks")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "forcedAdjacentLinks", targetInfo._vForcedAdjacentLinks);
    }

    // TODO: extraGeometries

    if (value.HasMember("isStatic")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "isStatic", targetInfo._bStatic);
    }
    if (value.HasMember("isEnabled")) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, "isEnabled", targetInfo._bIsEnabled);
    }
}

void DeserializeDiffJSON(KinBody::JointInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale) {

}

void DeserializeDiffJSON(KinBody::GeometryInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale) {

}


void DeserializeDiffJSON(RobotBase::ManipulatorInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale) {

}


// \brief Serialize the different between coverInfo and baseInfo, save the result into rapidjson value;
void SerializeDiffJSON(const KinBody::LinkInfo& coverInfo, const KinBody::LinkInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options)
{
    value.SetObject();
    if (coverInfo._bIsDeleted) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "__deleted__", coverInfo._bIsDeleted, allocator);
    }

    if (coverInfo._t != baseInfo._t) {
        Transform tmpTransform {coverInfo._t};
        tmpTransform.trans *= fUnitScale;
        OpenRAVE::JSON::SetJsonValueByKey(value, "transform", tmpTransform, allocator);
    }
    if (coverInfo._tMassFrame != baseInfo._tMassFrame) {
        Transform tmpMassTransform {coverInfo._tMassFrame};
        tmpMassTransform.trans *= fUnitScale;
        OpenRAVE::JSON::SetJsonValueByKey(value, "massTransform", tmpMassTransform, allocator);
    }
    if (coverInfo._mass != baseInfo._mass) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "mass", coverInfo._mass, allocator);
    }
    if (coverInfo._vinertiamoments != baseInfo._vinertiamoments) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "intertialMoments", coverInfo._vinertiamoments, allocator);
    }
    if (coverInfo._mapFloatParameters.size() > 0) {
        if (coverInfo._mapFloatParameters != baseInfo._mapFloatParameters) {
            OpenRAVE::JSON::SetJsonValueByKey(value, "floatParameters", coverInfo._mapFloatParameters, allocator);
        }
    }

    if (coverInfo._mapIntParameters != baseInfo._mapIntParameters) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "intParameters", coverInfo._mapIntParameters, allocator);
    }
    if (coverInfo._mapStringParameters != baseInfo._mapStringParameters) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "stringParameters", coverInfo._mapStringParameters, allocator);
    }

    if (coverInfo._vForcedAdjacentLinks != baseInfo._vForcedAdjacentLinks) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "forcedAdjacentLinks", coverInfo._vForcedAdjacentLinks, allocator);
    }

    if (coverInfo._vgeometryinfos.size() > 0) {
        std::vector<KinBody::GeometryInfoPtr> diffGeometries;
        diffGeometries.reserve(coverInfo._vgeometryinfos.size());
        FOREACHC(it, coverInfo._vgeometryinfos) {
            bool found = false;
            FOREACHC(itRef, baseInfo._vgeometryinfos) {
                if ((*it)->_id == (*itRef)->_id) {
                    found = true;
                    if ((*it) != (*itRef)) {
                        diffGeometries.push_back(*it);
                    }
                }
            }
            if (!found) {
                diffGeometries.push_back(*it);
            }
        }
        diffGeometries.shrink_to_fit();

        // serialize all the different geometries
        rapidjson::Value geometriesValue;
        geometriesValue.SetArray();
        geometriesValue.Reserve(diffGeometries.size(), allocator);
        FOREACHC(it, diffGeometries) {
            rapidjson::Value geometryValue;
            if (!!(*it)->_referenceInfo) {
                SerializeDiffJSON(**it, *(*it)->_referenceInfo, geometryValue, allocator);
            }
            else {
                (*it)->SerializeJSON(geometryValue, allocator, options);
            }

            geometriesValue.PushBack(geometryValue, allocator);
        }
        if (value.HasMember("geometries")) {
            value.RemoveMember("geometries");
        }
        value.AddMember("geometries", geometriesValue, allocator);
    }

    if (coverInfo._mapExtraGeometries.size() > 0) {
        rapidjson::Value extraGeometriesValue;
        extraGeometriesValue.SetObject();

        FOREACHC(im, coverInfo._mapExtraGeometries) {
            rapidjson::Value geometriesValue;
            if (baseInfo._mapExtraGeometries.count(im->first) > 0) {
                FOREACHC(iGeom, (*im).second) {
                    bool found = false;
                    rapidjson::Value geometryValue;
                    FOREACHC(iRefGeom, baseInfo._mapExtraGeometries.find(im->first)->second) {
                        if ((*iGeom)->_id == (*iRefGeom)->_id) {
                            found = true;
                            SerializeDiffJSON(**iGeom, **iRefGeom, geometryValue, allocator);
                            break;
                        }
                    }
                    if (!found) {
                        (*iGeom)->SerializeJSON(geometryValue, allocator, fUnitScale, options);
                    }
                    geometriesValue.PushBack(geometryValue, allocator);
                }
            }
            else{
                FOREACH(iGeom, (*im).second) {
                    rapidjson::Value geometryValue;
                    (*iGeom)->SerializeJSON(geometryValue, allocator, fUnitScale);
                    geometriesValue.PushBack(geometryValue, allocator);
                }
            }
            extraGeometriesValue.AddMember(rapidjson::Value(im->first.c_str(), allocator).Move(), geometriesValue, allocator);
        }
        value.AddMember("extraGeometries", extraGeometriesValue, allocator);
    }
    if (coverInfo._bStatic != baseInfo._bStatic) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "isStatic", coverInfo._bStatic, allocator);
    }
    if (coverInfo._bIsEnabled != baseInfo._bIsEnabled) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "isEnabled", coverInfo._bIsEnabled, allocator);
    }
    // set name and id only if there is difference and some value are set to converInfo
    if (value.Size() > 0) {
        OpenRAVE::JSON::SetJsonValueByKey(value, "name", coverInfo._name, allocator);
        OpenRAVE::JSON::SetJsonValueByKey(value, "id", coverInfo._id, allocator);
    }
}

}


