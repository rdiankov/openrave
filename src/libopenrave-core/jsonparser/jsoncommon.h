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

/// \brief Cached context that can be passed to successive Load/LoadURI/ReadKinBodyURI calls to cache referenced objects across calls
class EnvironmentLoadContextJSON : public EnvironmentLoadContext
{
public:
    virtual ~EnvironmentLoadContextJSON() {
    }

    void Reset() override {
        rapidjsonDocuments.clear();
        rapidjsonAllocator.Clear();
    }

    /// How much space should be preallocated for our rapidjson objects
    static const size_t JSON_ALLOCATOR_PREALLOC_BYTES = 64 * 1024;

    /// Cached set of rapidjson documents that have been loaded, indexed by filename
    /// These documents are linked to the allocator that is part of this structure.
    typedef std::unordered_map<std::string, boost::shared_ptr<const rapidjson::Document>> MapRapidJsonDocuments;
    MapRapidJsonDocuments rapidjsonDocuments;

    /// Preallocated region for JSON parsing
    std::array<uint8_t, JSON_ALLOCATOR_PREALLOC_BYTES> rapidjsonAllocatorBuffer;

    /// Allocator for cached documents. Must not be cleared for the lifetime of the load context.
    rapidjson::MemoryPoolAllocator<> rapidjsonAllocator{&rapidjsonAllocatorBuffer[0], JSON_ALLOCATOR_PREALLOC_BYTES};
};

bool RaveParseJSON(EnvironmentBasePtr penv, const std::string &uri, const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSON(EnvironmentBasePtr penv, const std::string &uri, KinBodyPtr& ppbody, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSON(EnvironmentBasePtr penv, const std::string &uri, RobotBasePtr& pprobot, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONFile(EnvironmentBasePtr penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONData(EnvironmentBasePtr penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackData(EnvironmentBasePtr penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseMsgPackData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONFile(EnvironmentBasePtr penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONData(EnvironmentBasePtr penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackData(EnvironmentBasePtr penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedJSONURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);
bool RaveParseEncryptedMsgPackURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc, EnvironmentLoadContextJSON& loadContext);

void RaveWriteJSON(EnvironmentBasePtr penv, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);
void RaveWriteJSON(const std::list<KinBodyPtr>& listbodies, rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);

void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONStream(EnvironmentBasePtr penv, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONStream(const std::list<KinBodyPtr>& listbodies, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackStream(EnvironmentBasePtr penv, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackStream(const std::list<KinBodyPtr>& listbodies, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteMsgPackMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteEncryptedJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONStream(EnvironmentBasePtr penv, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedJSONStream(const std::list<KinBodyPtr>& listbodies, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

void RaveWriteEncryptedMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackStream(EnvironmentBasePtr penv, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);
void RaveWriteEncryptedMsgPackStream(const std::list<KinBodyPtr>& listbodies, std::ostream& os, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc);

bool GpgDecrypt(std::istream& inputStream, std::ostream& outputData);
bool GpgEncrypt(std::istream& inputStream, std::ostream& outputData, const std::unordered_set<string>& keyIds);

}
#endif
