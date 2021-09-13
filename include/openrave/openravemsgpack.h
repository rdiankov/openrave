// -*- coding: utf-8 -*-
// Copyright (C) 2020 OpenRAVE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** \file openravemsgpack.h
    \brief Wrapper for msgpack.
 */
#ifndef OPENRAVE_MSGPACK_H
#define OPENRAVE_MSGPACK_H

#include <openrave/config.h>

#include <vector>
#include <string>
#include <iostream>

#include <rapidjson/document.h>

namespace OpenRAVE {

namespace MsgPack {

OPENRAVE_API void DumpMsgPack(const rapidjson::Value& value, std::ostream& os);
OPENRAVE_API void DumpMsgPack(const rapidjson::Value& value, std::vector<char>& output);

OPENRAVE_API void ParseMsgPack(rapidjson::Document& d, const std::string& str);
OPENRAVE_API void ParseMsgPack(rapidjson::Document& d, std::istream& is);

} // namespace MsgPack

} // namespace OpenRAVE

#endif // OPENRAVE_MSGPACK_H
