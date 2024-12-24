// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov (rosen.diankov@gmail.com)
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

namespace OpenRAVE {

void KinBody::ExtraGeometryInfo::Reset()
{
    _vgeometryinfos.clear();
    _id.clear();
    _name.clear();
}

void KinBody::ExtraGeometryInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    value.SetObject();
    orjson::SetJsonValueByKey(value, "id", _id, allocator);
    orjson::SetJsonValueByKey(value, "name", _name, allocator);

    if (_vgeometryinfos.size() > 0) {
        rapidjson::Value geometriesValue;
        geometriesValue.SetArray();
        geometriesValue.Reserve(_vgeometryinfos.size(), allocator);
        FOREACHC(it, _vgeometryinfos) {
            rapidjson::Value geometryValue;
            (*it)->SerializeJSON(geometryValue, allocator, fUnitScale, options);
            geometriesValue.PushBack(geometryValue, allocator);
        }
        value.AddMember("geometries", geometriesValue, allocator);
    }
}

void KinBody::ExtraGeometryInfo::DeserializeJSON(const rapidjson::Value &value, dReal fUnitScale, int options)
{
    std::string id;
    std::string name;
    if (value.HasMember("id")) {
        orjson::LoadJsonValueByKey(value, "id", id);
    }
    if (value.HasMember("name")) {
        orjson::LoadJsonValueByKey(value, "name", name);
    }
    
    if (id.empty() && name.empty()) {
        RAVELOG_WARN("failed to deserialize the json due to missing or empty id and name");
        return;
    }
    else if (id.empty()) {
        id = name;
    }
    else if (name.empty()) {
        name = id;
    }
    this->_id = id;
    this->_name = name;

    if (value.HasMember("geometries")) {
        _vgeometryinfos.reserve(value["geometries"].Size() + _vgeometryinfos.size());
        for (rapidjson::Value::ConstValueIterator it = value["geometries"].Begin(); it != value["geometries"].End(); ++it) {
            UpdateOrCreateInfoWithNameCheck(*it, _vgeometryinfos, "name", fUnitScale, options);
        }
    }
}

}