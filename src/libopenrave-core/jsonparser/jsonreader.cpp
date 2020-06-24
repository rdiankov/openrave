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

#include <openrave/openravejson.h>
#include <openrave/openravemsgpack.h>
#include <openrave/openrave.h>
#include <rapidjson/istreamwrapper.h>
#include <string>
#include <fstream>

namespace OpenRAVE {

/// \brief get the scheme of the uri, e.g. file: or openrave:
static void ParseURI(const std::string& uri, std::string& scheme, std::string& path, std::string& fragment)
{
    path = uri;
    size_t hashindex = path.find_last_of('#');
    if (hashindex != std::string::npos) {
        fragment = path.substr(hashindex + 1);
        path = path.substr(0, hashindex);
    }

    size_t colonindex = path.find_first_of(':');
    if (colonindex != std::string::npos) {
        // notice: in python code, like realtimerobottask3.py, it pass scheme as {openravescene: mujin}. No colon,
        scheme = path.substr(0, colonindex);
        path = path.substr(colonindex + 1);
    }
}

static std::string ResolveURI(const std::string& scheme, const std::string& path, const std::vector<std::string>& vOpenRAVESchemeAliases)
{
    if (scheme.empty() && path.empty() ) {
        return std::string();
    }
    else if (scheme == "file")
    {
        return RaveFindLocalFile(path);
    }
    else if (find(vOpenRAVESchemeAliases.begin(), vOpenRAVESchemeAliases.end(), scheme) != vOpenRAVESchemeAliases.end())
    {
        return RaveFindLocalFile(path);
    }

    return std::string();
}

/// \brief resolve a uri
static std::string ResolveURI(const std::string& uri, const std::vector<std::string>& vOpenRAVESchemeAliases)
{
    std::string scheme, path, fragment;
    ParseURI(uri, scheme, path, fragment);
    return ResolveURI(scheme, path, vOpenRAVESchemeAliases);
}

/// \brief open and cache a json document
static void OpenRapidJsonDocument(const std::string& filename, rapidjson::Document& doc)
{
    std::ifstream ifs(filename.c_str());
    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::ParseResult ok = doc.ParseStream<rapidjson::kParseFullPrecisionFlag>(isw);
    if (!ok) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to parse json document \"%s\"", filename, ORE_InvalidArguments);
    }
}

/// \brief open and cache a msgpack document
static void OpenMsgPackDocument(const std::string& filename, rapidjson::Document& doc)
{
    std::ifstream ifs(filename.c_str());
    OpenRAVE::MsgPack::ParseMsgPack(doc, ifs);
}

class JSONReader
{
public:

    JSONReader(const AttributesList& atts, EnvironmentBasePtr penv): _penv(penv)
    {
        FOREACHC(itatt, atts) {
            if (itatt->first == "openravescheme") {
                std::stringstream ss(itatt->second);
                _vOpenRAVESchemeAliases = std::vector<std::string>((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
            }
        }
        if (_vOpenRAVESchemeAliases.size() == 0) {
            _vOpenRAVESchemeAliases.push_back("openrave");
        }

        // set global scale when initalize jsonreader.
        _fGlobalScale = 1.0 / _penv->GetUnit().second;
    }

    virtual ~JSONReader()
    {
    }

    const std::vector<std::string>& GetOpenRAVESchemeAliases() const {
        return _vOpenRAVESchemeAliases;
    }


    bool IsExpandableRapidJSON(const rapidjson::Value& value) {
        std::string uri = OpenRAVE::orjson::GetStringJsonValueByKey(value, "uri", "");
        if (uri.empty()) {
            return false;
        }
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);
        if (fragment.empty()) {
            return false;
        }
        return true;
    }

    // TODO: 1. add cache for each opened document 2. check circular reference.
    bool ExpandRapidJSON(EnvironmentBase::EnvironmentBaseInfo& envInfo, std::string uri, rapidjson::Document::AllocatorType& alloc) {
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);

        std::string fullFilename = ResolveURI(uri, GetOpenRAVESchemeAliases());
        rapidjson::Document expandedDoc;  // we don't re-use alloc here because we only want part of the doc. otherwise the expandedDoc will remain in the memory until alloc is destroyed.

        if (_openedRapidJSONDocs.find(fullFilename) != _openedRapidJSONDocs.end()) {
            expandedDoc = _openedRapidJSONDocs[fullFilename];  // TODO: make it reference instead of copy
        }
        else {
            OpenRapidJsonDocument(fullFilename, expandedDoc);
            _openedRapidJSONDocs[fullFilename] = expandedDoc;  // TODO: make it reference instead of copy
        }

        if (!expandedDoc.HasMember("bodies")) {
            return false;
        }

        for(rapidjson::Value::ValueIterator it = expandedDoc["bodies"].Begin(); expandedDoc["bodies"].End(); it++) {
            rapidjson::Value& bodyValue = *it;
            std::string id = OpenRAVE::orjson::GetStringJsonValueByKey(bodyValue, "id");
            if ( id == fragment) {
                if (IsExpandableRapidJSON(*it)) {
                    std::string uri = OpenRAVE::orjson::GetStringJsonValueByKey(value, "uri");
                    ExpandRapidJSON(envInfo, uri, alloc);
                }
                envInfo.DeserializeJSON(bodyValue, 1.0);
                return true;
            }
        }
        return false;
    }

    /// \brief Extrat all bodies and add them into environment
    bool ExtractAll(const rapidjson::Value& doc, rapidjson::Document::AllocatorType& alloc)
    {
        EnvironmentBase::EnvironmentBaseInfo envInfo;
        dReal fUnitScale = _GetUnitScale(doc);
        _penv->ExtractInfo(envInfo);

        if (doc.HasMember("bodies")) {
            for (rapidjson::Value::ConstValueIterator it = doc["bodies"].Begin(); it != doc["bodies"].End(); it++) {
                if (IsExpandableRapidJSON(*it)) {
                    std::string uri = OpenRAVE::orjson::GetStringJsonValueByKey(value, "uri");
                    if (!ExpandRapidJSON(envInfo, uri, alloc)) {
                        return false;
                    }
                    // TODO: save the reference for later serialize
                    // std::string id = OpenRAVE::orjson::GetStringJsonValueByKey(*it, "id");
                    // _penv->_mExpandedBodyValue[id] = shadowValue; // shadowValue is moved
                }
            }
        }
        envInfo.DeserializeJSON(doc);
        _penv->UpdateFromInfo(envInfo);
        return true;
    }

    bool ExtractFirst(const rapidjson::Value& doc, KinBodyPtr& ppbody)
    {
        // extract the first articulated system found.
        dReal fUnitScale = _GetUnitScale(doc);
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, ppbody, fUnitScale);
            }
        }
        return false;
    }

    bool ExtractFirst(const rapidjson::Value& doc, RobotBasePtr& pprobot)
    {
        // extract the first robot
        dReal fUnitScale = _GetUnitScale(doc);
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, pprobot, fUnitScale);
            }
        }
        return false;
    }

    bool ExtractOne(const rapidjson::Value& doc, KinBodyPtr& ppbody, const string& uri) {
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);
        if (fragment == "") {
            return ExtractFirst(doc, ppbody);
        }

        // find the body by uri fragment
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            dReal fUnitScale = _GetUnitScale(doc);
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                std::string bodyId;
                OpenRAVE::orjson::LoadJsonValueByKey(*itr, "id", bodyId);
                if (bodyId == fragment) {
                    return _Extract(*itr, ppbody, fUnitScale);
                }
            }
        }
        return false;
    }

    bool ExtractOne(const rapidjson::Value& doc, RobotBasePtr& pprobot, const string& uri) {
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);
        if (fragment == "") {
            return ExtractFirst(doc, pprobot);
        }

        // find the body by uri fragment
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            dReal fUnitScale = _GetUnitScale(doc);
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                std::string bodyId;
                OpenRAVE::orjson::LoadJsonValueByKey(*itr, "id", bodyId);
                if (bodyId == fragment) {
                    return _Extract(*itr, pprobot, fUnitScale);
                }
            }
        }
        return false;
    }

    void SetURI(const std::string& uri) {
        _uri = uri;
    }
    void SetFilename(const std::string& filename) {
        _filename = filename;
    }

protected:
    inline dReal _GetUnitScale(const rapidjson::Value& doc)
    {
        std::pair<std::string, dReal> unit = {"meter", 1};
        OpenRAVE::orjson::LoadJsonValueByKey(doc, "unit", unit);

        // TODO: for now we just set defautl to ["meter", 1]
        if (unit.first.empty()){
            unit.first = "meter";
            unit.second = 1;
        }

        if (unit.first == "mm") {
            unit.second *= 0.001;
        }
        else if (unit.first == "cm") {
            unit.second *= 0.01;
        }

        dReal scale = unit.second / _fGlobalScale;
        return scale;
    }

    std::string _CanonicalizeURI(const std::string& uri)
    {
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);

        if (scheme == "" && path == "") {
            if (_uri != "") {
                std::string scheme2, path2, fragment2;
                ParseURI(_uri, scheme2, path2, fragment2);
                return scheme2 + ":" + path2 + "#" + fragment;
            }
            return std::string("file:") + _filename + "#" + fragment;
        }
        return uri;
    }

    void _EnsureUniqueIdAndUri(KinBody::KinBodyInfo& bodyInfo)
    {
        if (bodyInfo._id.empty()) {
            RAVELOG_WARN("info id is empty");
            bodyInfo._id = "body0";
        }
        int suffix = 1;
        while (_bodyUniqueIds.find(bodyInfo._id) != _bodyUniqueIds.end()) {
            bodyInfo._id = "body" + std::to_string(suffix);
            suffix += 1;
        }
        _bodyUniqueIds.insert(bodyInfo._id);

        bodyInfo._uri = "#" + bodyInfo._id;
        _CanonicalizeURI(bodyInfo._uri);
    }

    template<typename T>
    void _ExtractTransform(const rapidjson::Value& bodyValue, boost::shared_ptr<T> pbody, dReal fUnitScale)
    {
        Transform transform;
        if (bodyValue.HasMember("transform")) {
            OpenRAVE::orjson::LoadJsonValueByKey(bodyValue, "transform", transform);
        }
        transform.trans *= fUnitScale;
        pbody->SetTransform(transform);
    }

    bool _Extract(const rapidjson::Value& bodyValue, KinBodyPtr& pBodyOut, dReal fUnitScale)
    {
        // extract for robot
        if (OpenRAVE::orjson::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            RobotBasePtr pRobot;
            if (_Extract(bodyValue, pRobot, fUnitScale)) { // TODO: change robot part to iterator
                pBodyOut = pRobot;
                return true;
            }
            return false;
        }

        KinBody::KinBodyInfoPtr pKinBodyInfo(new KinBody::KinBodyInfo());
        pKinBodyInfo->DeserializeJSON(bodyValue, fUnitScale);
        _EnsureUniqueIdAndUri(*pKinBodyInfo);

        KinBodyPtr pBody = RaveCreateKinBody(_penv, "");
        if (!pBody->InitFromInfo(*pKinBodyInfo)) {
            return false;
        }
        pBody->SetName(pKinBodyInfo->_name);
        _ExtractTransform(bodyValue, pBody, fUnitScale);
        _ExtractReadableInterfaces(bodyValue, pBody, fUnitScale);
        pBodyOut = pBody;
        return true;
    }

    bool _Extract(const rapidjson::Value& bodyValue, RobotBasePtr& pRobotOut, dReal fUnitScale)
    {
        if (!OpenRAVE::orjson::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            return false;
        }

        RobotBase::RobotBaseInfoPtr pRobotBaseInfo(new RobotBase::RobotBaseInfo());
        pRobotBaseInfo->DeserializeJSON(bodyValue, fUnitScale);
        _EnsureUniqueIdAndUri(*pRobotBaseInfo);

        RobotBasePtr pRobot = RaveCreateRobot(_penv, "GenericRobot");
        if (!pRobot) {
            pRobot = RaveCreateRobot(_penv, "");
        }
        if (!pRobot->InitFromInfo(*pRobotBaseInfo)) {
            return false;
        }
        pRobot->SetName(pRobotBaseInfo->_name);
        _ExtractTransform(bodyValue, pRobot, fUnitScale);
        _ExtractReadableInterfaces(bodyValue, pRobot, fUnitScale);
        pRobotOut = pRobot;
        return true;
    }

    void _ExtractReadableInterfaces(const rapidjson::Value &objectValue, InterfaceBasePtr pInterface, dReal fUnitScale)
    {
        if (objectValue.HasMember("readableInterfaces") && objectValue["readableInterfaces"].IsArray()) {
            for (rapidjson::Value::ConstValueIterator it = objectValue["readableInterfaces"].Begin(); it != objectValue["readableInterfaces"].End(); it++) {
                std::string id;
                OpenRAVE::orjson::LoadJsonValueByKey(*it, "id", id);
                BaseJSONReaderPtr pReader = RaveCallJSONReader(pInterface->GetInterfaceType(), id, pInterface, AttributesList());
                if (!!pReader) {
                    pReader->DeserializeJSON(*it, fUnitScale);
                    JSONReadablePtr pReadable = pReader->GetReadable();
                    if (!!pReadable) {
                        pInterface->SetReadableInterface(id, pReadable);
                    }
                }
                else if (it->HasMember("string")) {
                    // TODO: current json data is not able to help distinguish the type. So we try to use string readable if the value is string and no reader is found.
                    std::string stringValue;
                    OpenRAVE::orjson::LoadJsonValueByKey(*it, "string", stringValue);
                    StringReadablePtr pReadable(new StringReadable(id, stringValue));
                    pInterface->SetReadableInterface(id, pReadable);
                }
            }
        }
    }

    dReal _fGlobalScale;
    EnvironmentBasePtr _penv;
    std::string _filename;
    std::string _uri;
    std::vector<std::string> _vOpenRAVESchemeAliases;

    std::set<std::string> _bodyUniqueIds; ///< unique id for bodies in current doc

    std::map<std::string, rapidjson::Document> _openedDocuments; ///< cache for opened rapidjson Documents
};

bool RaveParseJSON(EnvironmentBasePtr penv, const rapidjson::Value& doc, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    rapidjson::Document document;
    return reader.ExtractAll(doc, document.GetAllocator());
}

bool RaveParseJSON(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const rapidjson::Value& doc, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, ppbody);
}

bool RaveParseJSON(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const rapidjson::Value& doc, const AttributesList& atts)
{
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, pprobot);
}

bool RaveParseJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    JSONReader reader(atts, penv);
    reader.SetFilename(fullFilename);
    rapidjson::Document doc(&alloc);
    OpenRapidJsonDocument(fullFilename, doc);
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseJSONFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    rapidjson::Document doc(&alloc);
    OpenRapidJsonDocument(fullFilename, doc);
    JSONReader reader(atts, penv);
    reader.SetFilename(fullFilename);
    return reader.ExtractFirst(doc, ppbody);
}

bool RaveParseJSONFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    rapidjson::Document doc(&alloc);
    OpenRapidJsonDocument(fullFilename, doc);
    JSONReader reader(atts, penv);
    reader.SetFilename(fullFilename);
    return reader.ExtractFirst(doc, pprobot);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = ResolveURI(uri, reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
    OpenRapidJsonDocument(fullFilename, doc);
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = ResolveURI(uri, reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
    OpenRapidJsonDocument(fullFilename, doc);
    return reader.ExtractOne(doc, ppbody, uri);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = ResolveURI(uri, reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
    OpenRapidJsonDocument(fullFilename, doc);
    return reader.ExtractOne(doc, pprobot, uri);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    orjson::ParseJson(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    orjson::ParseJson(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, ppbody);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    orjson::ParseJson(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, pprobot);
}

bool RaveParseMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    rapidjson::Document doc(&alloc);
    OpenMsgPackDocument(fullFilename, doc);
    JSONReader reader(atts, penv);
    reader.SetFilename(fullFilename);
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseMsgPackFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    rapidjson::Document doc(&alloc);
    OpenMsgPackDocument(fullFilename, doc);
    JSONReader reader(atts, penv);
    reader.SetFilename(fullFilename);
    return reader.ExtractFirst(doc, ppbody);
}

bool RaveParseMsgPackFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    rapidjson::Document doc(&alloc);
    OpenMsgPackDocument(fullFilename, doc);
    JSONReader reader(atts, penv);
    reader.SetFilename(fullFilename);
    return reader.ExtractFirst(doc, pprobot);
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = ResolveURI(uri, reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
    OpenMsgPackDocument(fullFilename, doc);
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = ResolveURI(uri, reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
    OpenMsgPackDocument(fullFilename, doc);
    return reader.ExtractOne(doc, ppbody, uri);
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    std::string fullFilename = ResolveURI(uri, reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
    OpenMsgPackDocument(fullFilename, doc);
    return reader.ExtractOne(doc, pprobot, uri);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    OpenRAVE::MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    OpenRAVE::MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, ppbody);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    OpenRAVE::MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, pprobot);
}

}
