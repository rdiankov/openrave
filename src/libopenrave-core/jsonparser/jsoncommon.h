// -*- coding: utf-8 -*-
// Copyright (C) 2012-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file jsoncommon.h
    \brief Common external definitions for the json reader/writer
 */

#ifndef OPENRAVE_JSON_COMMON_H
#define OPENRAVE_JSON_COMMON_H

#include "../ravep.h"
#include <openrave/openravejson.h>

namespace OpenRAVE
{

bool RaveParseJSON(EnvironmentBasePtr penv, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSON(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSON(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONURI(EnvironmentBasePtr penv, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONData(EnvironmentBasePtr penv, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackURI(EnvironmentBasePtr penv, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackData(EnvironmentBasePtr penv, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteJSON(EnvironmentBasePtr penv, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);
void RaveWriteJSON(KinBodyPtr pbody, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);
void RaveWriteJSON(const std::list<KinBodyPtr>& listbodies, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);

void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONStream(EnvironmentBasePtr penv, const ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONStream(KinBodyPtr pbody, const ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONStream(const std::list<KinBodyPtr>& listbodies, const ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONMemory(KinBodyPtr pbody, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackStream(EnvironmentBasePtr penv, const ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackStream(KinBodyPtr pbody, const ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackStream(const std::list<KinBodyPtr>& listbodies, const ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackMemory(KinBodyPtr pbody, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

}
#endif
