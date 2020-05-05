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

#include <openrave/openrave.h>
#include <openrave/openravejson.h>
#include <boost/make_shared.hpp>
#include <boost/variant/variant.hpp>

#include "../ravep.h"

namespace OpenRAVE
{


template<typename T> void _ExtractInfo(const rapidjson::Value& value, std::vector<OPENRAVE_SHARED_PTR<T>>& infos, dReal fUnitScale=1.0);
template<typename T> void _MergeInfo(std::string id, T& info, const rapidjson::Value value, dReal fUnitScale);

bool RaveParseJSON(EnvironmentBasePtr penv, const rapidjson::Document& doc, const AttributesList& atts);
bool RaveParseJSON(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const rapidjson::Document& doc, const AttributesList& atts);
bool RaveParseJSON(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const rapidjson::Document& doc, const AttributesList& atts);
bool RaveParseJSONURI(EnvironmentBasePtr penv, const std::string& uri, const AttributesList& atts);
bool RaveParseJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts);
bool RaveParseJSONURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts);
bool RaveParseJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts);
bool RaveParseJSONFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts);
bool RaveParseJSONFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts);
bool RaveParseJSONData(EnvironmentBasePtr penv, const std::string& data, const AttributesList& atts);
bool RaveParseJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts);
bool RaveParseJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts);

void RaveWriteJSON(EnvironmentBasePtr penv, rapidjson::Value& rScene, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);
void RaveWriteJSON(KinBodyPtr pbody, rapidjson::Value& rScene, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);
void RaveWriteJSON(const std::list<KinBodyPtr>& listbodies, rapidjson::Value& rScene, rapidjson::Document::AllocatorType& allocator, const AttributesList& atts);
void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts);
void RaveWriteJSONFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts);
void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts);
void RaveWriteJSONStream(EnvironmentBasePtr penv, const ostream& os, const AttributesList& atts);
void RaveWriteJSONStream(KinBodyPtr pbody, const ostream& os, const AttributesList& atts);
void RaveWriteJSONStream(const std::list<KinBodyPtr>& listbodies, const ostream& os, const AttributesList& atts);
void RaveWriteJSONMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts);
void RaveWriteJSONMemory(KinBodyPtr pbody, std::vector<char>& output, const AttributesList& atts);
void RaveWriteJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts);

void SerializeDiffJSON(const KinBody::LinkInfo& coverInfo, const KinBody::LinkInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0);
void SerializeDiffJSON(const KinBody::GeometryInfo& coverInfo, const KinBody::GeometryInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0);
void SerializeDiffJSON(const KinBody::JointInfo& coverInfo, const KinBody::JointInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0);
void SerializeDiffJSON(const RobotBase::ManipulatorInfo& coverInfo, const RobotBase::ManipulatorInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0);
void SerializeDiffJSON(const RobotBase::AttachedSensorInfo& coverInfo, const RobotBase::AttachedSensorInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0);
void SerializeDiffJSON(const RobotBase::ConnectedBodyInfo& coverInfo, const RobotBase::ConnectedBodyInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0);
void SerializeDiffJSON(const RobotBase::GripperInfo& coverInfo, const RobotBase::GripperInfo& baseInfo, rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0);

void DeserializeDiffJSON(KinBody::LinkInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale=1.0);
void DeserializeDiffJSON(KinBody::JointInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale=1.0);
void DeserializeDiffJSON(RobotBase::ManipulatorInfo& targetInfo, const rapidjson::Value& value, dReal fUnitScale=1.0);

}

#endif
