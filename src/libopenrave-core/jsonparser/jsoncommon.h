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

namespace OpenRAVE
{

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

void RaveWriteJSON(EnvironmentBasePtr penv, rapidjson::Document& doc, const AttributesList& atts);
void RaveWriteJSON(KinBodyPtr pbody, rapidjson::Document& doc, const AttributesList& atts);
void RaveWriteJSON(const std::list<KinBodyPtr>& listbodies, rapidjson::Document& doc, const AttributesList& atts);
void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts);
void RaveWriteJSONFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts);
void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts);
void RaveWriteJSONStream(EnvironmentBasePtr penv, const ostream& os, const AttributesList& atts);
void RaveWriteJSONStream(KinBodyPtr pbody, const ostream& os, const AttributesList& atts);
void RaveWriteJSONStream(const std::list<KinBodyPtr>& listbodies, const ostream& os, const AttributesList& atts);
void RaveWriteJSONMemory(EnvironmentBasePtr penv, std::vector<char>& output, const AttributesList& atts);
void RaveWriteJSONMemory(KinBodyPtr pbody, std::vector<char>& output, const AttributesList& atts);
void RaveWriteJSONMemory(const std::list<KinBodyPtr>& listbodies, std::vector<char>& output, const AttributesList& atts);

}
#endif
