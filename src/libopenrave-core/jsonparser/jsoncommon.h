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
#include <unordered_set>

namespace OpenRAVE
{

bool RaveParseJSON(const EnvironmentBasePtr& penv, const std::string &uri, const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSON(const EnvironmentBasePtr& penv, const std::string &uri, KinBodyPtr& ppbody, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSON(const EnvironmentBasePtr& penv, const std::string &uri, RobotBasePtr& pprobot, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONURI(const EnvironmentBasePtr& penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONURI(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONURI(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONFile(const EnvironmentBasePtr& penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONFile(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONFile(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONData(const EnvironmentBasePtr& penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONData(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseJSONData(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackURI(const EnvironmentBasePtr& penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackURI(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackURI(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackFile(const EnvironmentBasePtr& penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackFile(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackFile(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackData(const EnvironmentBasePtr& penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackData(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseMsgPackData(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONFile(const EnvironmentBasePtr& penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONFile(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONFile(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackFile(const EnvironmentBasePtr& penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackFile(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackFile(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONData(const EnvironmentBasePtr& penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONData(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONData(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackData(const EnvironmentBasePtr& penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackData(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackData(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONURI(const EnvironmentBasePtr& penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONURI(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedJSONURI(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackURI(const EnvironmentBasePtr& penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackURI(const EnvironmentBasePtr& penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
bool RaveParseEncryptedMsgPackURI(const EnvironmentBasePtr& penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteJSON(const EnvironmentBasePtr& penv, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);
void RaveWriteJSON(const std::list<KinBodyPtr>& listbodies, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);

void RaveWriteJSONFile(const EnvironmentBasePtr& penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONStream(const EnvironmentBasePtr& penv, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONStream(const std::list<KinBodyPtr>& listbodies, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONMemory(const EnvironmentBasePtr& penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteMsgPackFile(const EnvironmentBasePtr& penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackStream(const EnvironmentBasePtr& penv, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackStream(const std::list<KinBodyPtr>& listbodies, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackMemory(const EnvironmentBasePtr& penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteEncryptedJSONFile(const EnvironmentBasePtr& penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONMemory(const EnvironmentBasePtr& penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONStream(const EnvironmentBasePtr& penv, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONStream(const std::list<KinBodyPtr>& listbodies, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteEncryptedMsgPackFile(const EnvironmentBasePtr& penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackMemory(const EnvironmentBasePtr& penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackStream(const EnvironmentBasePtr& penv, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackStream(const std::list<KinBodyPtr>& listbodies, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

bool GpgDecrypt(std::istream& inputStream, std::ostream& outputData);
bool GpgEncrypt(std::istream& inputStream, std::ostream& outputData, const std::unordered_set<string>& keyIds);

}
#endif
