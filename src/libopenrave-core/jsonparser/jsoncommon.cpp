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

void RemoveShadowRapidJSON(rapidjson::Value& value, const rapidjson::Value& shadowValue, rapidjson::Document::AllocatorType& alloc) {
    for (rapidjson::Value::MemberIterator it = value.MemberBegin(); it != value.MemberEnd(); it++){
        std::string name = it->name.GetString();
        if (!shadowValue.HasMember(name.c_str())) {
            continue; // no remove
        }

        const rapidjson::Value& rShadow = shadowValue[name.c_str()];
        std::string jsonType = OpenRAVE::orjson::GetJsonTypeName(it->value);

        if (jsonType != OpenRAVE::orjson::GetJsonTypeName(rShadow)) {
            continue; // no remove
        }

        if (jsonType == "True") {
            if (OpenRAVE::orjson::GetJsonValueByKey<bool>(rShadow, name.c_str(), false) == true) {
                value.RemoveMember(name.c_str());
            }
        }
        else if (jsonType == "False") {
            if (OpenRAVE::orjson::GetJsonValueByKey<bool>(rShadow, name.c_str(), true) == false) {
                value.RemoveMember(name.c_str());
            }
        }
        else if (jsonType == "Object") {
            RemoveShadowRapidJSON(it->value, rShadow, alloc);
            if (it->value.MemberCount() == 0) {
                value.RemoveMember(name.c_str());
            }
        }
        else if (jsonType == "Array") {
            rapidjson::Value rItemOrigin, rItemShadow; // a wrapper for each item object.
            rItemOrigin.SetObject();
            rItemShadow.SetObject();
            for(rapidjson::Value::ValueIterator itemValue = it->value.Begin(); itemValue != it->value.End(); itemValue++) {
                std::string itemId = OpenRAVE::orjson::GetJsonValueByKey<std::string>(*itemValue, "id", "");
                if (itemId.empty()) {
                    continue;
                }
                for(rapidjson::Value::ConstValueIterator shadowItemValue = rShadow.Begin(); shadowItemValue != rShadow.End(); shadowItemValue++) {
                    std::string shadowItemId = OpenRAVE::orjson::GetJsonValueByKey<std::string>(*shadowItemValue, "id", "");
                    if (shadowItemId == itemId) {
                        OpenRAVE::orjson::SetJsonValueByKey(rItemOrigin, "item", itemValue, alloc);
                        OpenRAVE::orjson::SetJsonValueByKey(rItemShadow, "item", itemValue, alloc);
                        RemoveShadowRapidJSON(rItemOrigin, rItemShadow, alloc);
                        if (rItemOrigin.MemberCount() == 0) {
                            it->value.Erase(itemValue);
                        }
                    }
                }
            }
            if (it->value.Size() == 0) {
                value.RemoveMember(name.c_str());
            }
        }
        else if (jsonType == "String") {
            if (OpenRAVE::orjson::GetJsonValueByKey<std::string>(rShadow, name.c_str(), "original") == OpenRAVE::orjson::GetJsonValueByKey<std::string>(it->value, name.c_str(), "shadow")) {
                value.RemoveMember(name.c_str());
            }
        }
        else if (jsonType == "Number") {
            if (rShadow.IsUint()) {
                if (it->value.GetUint() == rShadow.GetUint()) {
                    value.RemoveMember(name.c_str());
                }
            }
            else if (rShadow.IsInt()) {
                if (it->value.GetInt() == rShadow.GetInt()) {
                    value.RemoveMember(name.c_str());
                }
            }
            else if (rShadow.IsUint64()) {
                if (it->value.GetUint64() == rShadow.GetUint64()) {
                    value.RemoveMember(name.c_str());
                }
            }
            else if (rShadow.IsInt64()) {
                if (it->value.GetInt64() == rShadow.GetInt64()) {
                    value.RemoveMember(name.c_str());
                }
            }
            else if (rShadow.IsDouble()) {
                if (it->value.GetDouble() == rShadow.GetDouble()) {
                    value.RemoveMember(name.c_str());
                }
            }
            else {
               // raise
            }
        }
        else {
            //  raise;
        }
    }
}
}



