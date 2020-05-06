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
    std::string id;
    OpenRAVE::JSON::LoadJsonValueByKey(value, "id", id);

    if (id.empty()) {
        // generate a new id for this structure
        std::set<std::string> usedId;
        FOREACH(it, infos) {
            usedId.insert((*it)->_id);
        }
        unsigned int suffix = 0;
        std::string name = "";
        OpenRAVE::JSON::LoadJsonValueByKey(value, "name", name);
        do {
            id = name + std::to_string(suffix);
            suffix += 1;
        } while(usedId.find(id) != usedId.end());
        RAVELOG_WARN_FORMAT("missing id in info %s, create a new id %s", name%id);
    }

    typename std::vector<OPENRAVE_SHARED_PTR<T>>::iterator itRef = infos.begin();
    while (itRef != infos.end()) {
        if ((*itRef)->_id == id) {
            break;
        }
        itRef++;
    }
    if (itRef != infos.end()){
        if (value.HasMember("__deleted__") && value["__deleted__"].GetBool() == true) {
            infos.erase(itRef); // remove from vector
            return;
        }
        DeserializeDiffJSON(**itRef, value, fUnitScale);
    }
    else{
        OPENRAVE_SHARED_PTR<T> info(new T());
        info->DeserializeJSON(value, fUnitScale);
        info->_id = id;
        infos.push_back(info);
    }
}

template void _ExtractInfo(const rapidjson::Value& value, std::vector<KinBody::LinkInfoPtr>& infos, dReal fUnitScale=1.0);
template void _ExtractInfo(const rapidjson::Value& value, std::vector<KinBody::JointInfoPtr>& infos, dReal fUnitScale=1.0);
template void _ExtractInfo(const rapidjson::Value& value, std::vector<RobotBase::ManipulatorInfoPtr>& infos, dReal fUnitScale=1.0);
template void _ExtractInfo(const rapidjson::Value& value, std::vector<RobotBase::AttachedSensorInfoPtr>& infos, dReal fUnitScale=1.0);
template void _ExtractInfo(const rapidjson::Value& value, std::vector<RobotBase::ConnectedBodyInfoPtr>& infos, dReal fUnitScale=1.0);
template void _ExtractInfo(const rapidjson::Value& value, std::vector<RobotBase::GripperInfoPtr>& infos, dReal fUnitScale=1.0);

template<typename T> inline bool OverwriteJsonValueByKey(const rapidjson::Value& value, const char* key, T& field) {
    if (value.HasMember(key)) {
        OpenRAVE::JSON::LoadJsonValueByKey(value, key, field);
        return true;
    }
    return false;
}


//overwrite refInfo with value and save result into targetInfo
void DeserializeDiffJSON(KinBody::LinkInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale) {

}

void DeserializeDiffJSON(KinBody::JointInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale){

}
void DeserializeDiffJSON(KinBody::GeometryInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale) {
    OverwriteJsonValueByKey(value, "name", targetInfo._name);
    if (OverwriteJsonValueByKey(value, "transform", targetInfo._t)) {
        targetInfo._t.trans *= fUnitScale;
    }

    std::string typestr;
    if(OverwriteJsonValueByKey(value, "type", typestr)) {
        // reload geometry data
        targetInfo.DeserializeGeomData(value, typestr, fUnitScale);
    }

    OverwriteJsonValueByKey(value, "transparency", targetInfo._fTransparency);
    OverwriteJsonValueByKey(value, "visible", targetInfo._bVisible);
    OverwriteJsonValueByKey(value, "diffuseColor", targetInfo._vDiffuseColor);
    OverwriteJsonValueByKey(value, "ambientColor", targetInfo._vAmbientColor);
    OverwriteJsonValueByKey(value, "modifiable", targetInfo._bModifiable);
}

void DeserializeDiffJSON(RobotBase::ManipulatorInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale) {

}

void DeserializeDiffJSON(RobotBase::AttachedSensorInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale) {
    OverwriteJsonValueByKey(value, "name", targetInfo._name);
    OverwriteJsonValueByKey(value, "linkName", targetInfo._linkname);
    OverwriteJsonValueByKey(value, "transform", targetInfo._trelative);
    OverwriteJsonValueByKey(value, "type", targetInfo._sensorname);

    if (!!targetInfo._sensorgeometry) {
        targetInfo._sensorgeometry.reset();
    }

    if (value.HasMember("sensorGeometry")) {
        BaseJSONReaderPtr pReader = RaveCallJSONReader(PT_Sensor, targetInfo._sensorname, InterfaceBasePtr(), AttributesList());
        if (!!pReader) {
            pReader->DeserializeJSON(value["sensorGeometry"], fUnitScale);
            JSONReadablePtr pReadable = pReader->GetReadable();
            if (!!pReadable) {
                targetInfo._sensorgeometry = OPENRAVE_DYNAMIC_POINTER_CAST<SensorBase::SensorGeometry>(pReadable);
            }
        } else {
            RAVELOG_WARN_FORMAT("failed to get json reader for sensor type \"%s\"", targetInfo._sensorname);
        }
    }
}

void DeserializeDiffJSON(RobotBase::ConnectedBodyInfo& targetinfo, const rapidjson::Value& value, dReal fUnitScale) {
}

void DeserializeDiffJSON(RobotBase::GripperInfo& targetinfo, const rapidjson::Value& value, dReal fUnitScale) {
}

}


