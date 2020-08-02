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

/// \brief open and cache a msgpack document
static void OpenMsgPackDocument(const std::string& filename, rapidjson::Document& doc)
{
    std::ifstream ifs(filename.c_str());
    MsgPack::ParseMsgPack(doc, ifs);
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

class JSONReader
{
public:

    JSONReader(const AttributesList& atts, EnvironmentBasePtr penv) : _penv(penv)
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
        _deserializeOptions = 0;
    }

    virtual ~JSONReader()
    {
    }

    const std::vector<std::string>& GetOpenRAVESchemeAliases() const {
        return _vOpenRAVESchemeAliases;
    }


    bool IsExpandableRapidJSON(const rapidjson::Value& value) {
        std::string uri = orjson::GetStringJsonValueByKey(value, "referenceUri", "");
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

    boost::shared_ptr<const rapidjson::Document> _GetJsonDocumentFromUri(const std::string& fullFilename, rapidjson::Document::AllocatorType& alloc)
    {
        boost::shared_ptr<const rapidjson::Document> expandedDoc;
        // TODO: optimize this. for the first time expandedDoc is cached, all the expandable object will never get cached, because we are not update document cache after expand any body
        if (_rapidJSONDocuments.find(fullFilename) != _rapidJSONDocuments.end()) {
            expandedDoc = _rapidJSONDocuments[fullFilename];
        }
        else {
            boost::shared_ptr<rapidjson::Document> newDoc;
            newDoc.reset(new rapidjson::Document(&alloc));
            OpenRapidJsonDocument(fullFilename, *newDoc);
            expandedDoc = newDoc;
            _rapidJSONDocuments[fullFilename] = expandedDoc;
            _rapidJSONObjects[expandedDoc] = std::map<std::string, rapidjson::Value::ConstValueIterator>();
        }
        return expandedDoc;
    }

    bool _ExpandRapidJSON(EnvironmentBase::EnvironmentBaseInfo& envInfo, const rapidjson::Value& currentDoc, std::string bodyId, std::string uri, std::set<std::string>& circularReference, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc) {
        if (circularReference.find(uri) != circularReference.end()) {
            RAVELOG_WARN("Load scene failed: circular reference is found");
            return false;
        }
        circularReference.insert(uri);

        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);
        std::string fullFilename = ResolveURI(uri, GetOpenRAVESchemeAliases());
        if (fullFilename.empty() && fragment.empty()) {
            return false;
        }
        rapidjson::Document rEnv(&alloc);
        rEnv.SetObject();

        bool bFoundBody = false;
        rapidjson::Value bodyValue;

        if(fullFilename.empty() && !fragment.empty()) {
            // reference to itself
            if (currentDoc.HasMember("bodies")) {
                for(rapidjson::Value::ConstValueIterator it = currentDoc["bodies"].Begin(); it != currentDoc["bodies"].End(); it++) {
                    std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                    if (id == fragment) {
                        if (IsExpandableRapidJSON(*it)) {
                            std::string bodyUri = orjson::GetJsonValueByKey<std::string>(*it, "referenceUri", "");
                            if (!_ExpandRapidJSON(envInfo, currentDoc, id, bodyUri, circularReference, fUnitScale, alloc)) {
                                return false;
                            }
                        }
                        bodyValue.CopyFrom(*it, rEnv.GetAllocator());
                        bFoundBody = true;
                        break;
                    }
                }
            }
        }

        if (!bFoundBody) {
            boost::shared_ptr<const rapidjson::Document> expandedDoc = _GetJsonDocumentFromUri(fullFilename, alloc);
            if (!expandedDoc || !(*expandedDoc).HasMember("bodies")) {
                return false;
            }
            if (_rapidJSONObjects[expandedDoc].find(fragment) == _rapidJSONObjects[expandedDoc].end()) {
                for(rapidjson::Value::ConstValueIterator it = (*expandedDoc)["bodies"].Begin(); it != (*expandedDoc)["bodies"].End(); it++) {
                    std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                    if ( id == fragment) {
                        if (IsExpandableRapidJSON(*it)) {
                            std::string bodyUri = orjson::GetJsonValueByKey<std::string>(*it, "referenceUri", "");
                            if (!_ExpandRapidJSON(envInfo, *expandedDoc, bodyId, bodyUri, circularReference, fUnitScale, alloc)) {
                                return false;
                            }
                        }
                        _rapidJSONObjects[expandedDoc][fragment] = it;  // only cache the body after it's fully expanded
                        bodyValue.CopyFrom(*(_rapidJSONObjects[expandedDoc][fragment]), rEnv.GetAllocator());
                        bFoundBody = true;
                        break;
                    }
                }
            }
            else {
                bodyValue.CopyFrom(*(_rapidJSONObjects[expandedDoc][fragment]), rEnv.GetAllocator());
                bFoundBody = true;
            }
        }

        if (bFoundBody) {
            rapidjson::Value rBodies;
            rBodies.SetArray();
            // keep the original id, otherwise DeserializeJSON will create a new body instead.
            orjson::SetJsonValueByKey(bodyValue, "id", bodyId, rEnv.GetAllocator());
            rBodies.PushBack(bodyValue, rEnv.GetAllocator());
            rEnv.AddMember("bodies", rBodies, rEnv.GetAllocator());
            envInfo.DeserializeJSON(rEnv, fUnitScale, _deserializeOptions);
            return true;
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
                    std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                    std::string uri = orjson::GetJsonValueByKey<std::string>(*it, "referenceUri", "");
                    std::set<std::string> circularReference;
                    if (!_ExpandRapidJSON(envInfo, doc, id, uri, circularReference, fUnitScale, alloc)) {
                        return false;
                    }
                }
            }
            envInfo.DeserializeJSON(doc, fUnitScale, _deserializeOptions);
            FOREACH(itBodyInfo, envInfo._vBodyInfos) {
                KinBody::KinBodyInfoPtr& pKinBodyInfo = *itBodyInfo;
                RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
                if( !!pRobotBaseInfo ) {
                    _ProcessURIsInInfo(*pRobotBaseInfo, fUnitScale, alloc);
                }
            }
            _penv->UpdateFromInfo(envInfo);
        }
        return true;
    }

    bool ExtractFirst(const rapidjson::Value& doc, KinBodyPtr& ppbody, rapidjson::Document::AllocatorType& alloc)
    {
        // extract the first articulated system found.
        dReal fUnitScale = _GetUnitScale(doc);
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, ppbody, fUnitScale, alloc);
            }
        }
        return false;
    }

    bool ExtractFirst(const rapidjson::Value& doc, RobotBasePtr& pprobot, rapidjson::Document::AllocatorType& alloc)
    {
        // extract the first robot
        dReal fUnitScale = _GetUnitScale(doc);
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, pprobot, fUnitScale, alloc);
            }
        }
        return false;
    }

    bool ExtractOne(const rapidjson::Value& doc, KinBodyPtr& ppbody, const string& uri, rapidjson::Document::AllocatorType& alloc)
    {
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);
        if (fragment == "") {
            return ExtractFirst(doc, ppbody, alloc);
        }

        // find the body by uri fragment
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            dReal fUnitScale = _GetUnitScale(doc);
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                std::string bodyId;
                orjson::LoadJsonValueByKey(*itr, "id", bodyId);
                if (bodyId == fragment) {
                    return _Extract(*itr, ppbody, fUnitScale, alloc);
                }
            }
        }
        return false;
    }

    bool ExtractOne(const rapidjson::Value& doc, RobotBasePtr& pprobot, const string& uri, rapidjson::Document::AllocatorType& alloc)
    {
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);
        if (fragment == "") {
            return ExtractFirst(doc, pprobot, alloc);
        }

        // find the body by uri fragment
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            dReal fUnitScale = _GetUnitScale(doc);
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                std::string bodyId;
                orjson::LoadJsonValueByKey(*itr, "id", bodyId);
                if (bodyId == fragment) {
                    return _Extract(*itr, pprobot, fUnitScale, alloc);
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
        orjson::LoadJsonValueByKey(doc, "unit", unit);

        // TODO: for now we just set defautl to ["meter", 1]
        if (unit.first.empty()) {
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
            orjson::LoadJsonValueByKey(bodyValue, "transform", transform);
        }
        transform.trans *= fUnitScale;
        pbody->SetTransform(transform);
    }

    bool _Extract(const rapidjson::Value& bodyValue, KinBodyPtr& pBodyOut, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc)
    {
        // extract for robot
        if (orjson::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            RobotBasePtr pRobot;
            if (_Extract(bodyValue, pRobot, fUnitScale, alloc)) { // TODO: change robot part to iterator
                pBodyOut = pRobot;
                return true;
            }
            return false;
        }

        KinBody::KinBodyInfoPtr pKinBodyInfo(new KinBody::KinBodyInfo());
        pKinBodyInfo->DeserializeJSON(bodyValue, fUnitScale, _deserializeOptions);
        _EnsureUniqueIdAndUri(*pKinBodyInfo);
        RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);

        KinBodyPtr pBody;
        if( !!pRobotBaseInfo ) {
            _ProcessURIsInInfo(*pRobotBaseInfo, fUnitScale, alloc);
            RobotBasePtr pRobot;
            pRobot = RaveCreateRobot(_penv, pRobotBaseInfo->_interfaceType);
            if( !pRobot ) {
                pRobot = RaveCreateRobot(_penv, "");
            }
            if( !pRobot ) {
                return false;
            }
            if (!pRobot->InitFromRobotInfo(*pRobotBaseInfo)) {
                return false;
            }
        }
        else {
            pBody = RaveCreateKinBody(_penv, pKinBodyInfo->_interfaceType);
            if( !pBody ) {
                pBody = RaveCreateKinBody(_penv, "");
            }
            if( !pBody ) {
                return false;
            }
            if (!pBody->InitFromKinBodyInfo(*pKinBodyInfo)) {
                return false;
            }
        }
        pBody->SetName(pKinBodyInfo->_name);
        _ExtractTransform(bodyValue, pBody, fUnitScale);
        pBodyOut = pBody;
        return true;
    }

    bool _Extract(const rapidjson::Value& bodyValue, RobotBasePtr& pRobotOut, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc)
    {
        if (!orjson::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            return false;
        }

        RobotBase::RobotBaseInfoPtr pRobotBaseInfo(new RobotBase::RobotBaseInfo());
        pRobotBaseInfo->DeserializeJSON(bodyValue, fUnitScale, _deserializeOptions);
        _EnsureUniqueIdAndUri(*pRobotBaseInfo);

        RobotBasePtr pRobot = RaveCreateRobot(_penv, pRobotBaseInfo->_interfaceType);
        if (!pRobot) {
            pRobot = RaveCreateRobot(_penv, "");
        }
        if( !pRobot ) {
            return false;
        }
        if (!pRobot->InitFromRobotInfo(*pRobotBaseInfo)) {
            return false;
        }
        pRobot->SetName(pRobotBaseInfo->_name);
        _ExtractTransform(bodyValue, pRobot, fUnitScale);
        pRobotOut = pRobot;
        return true;
    }

    /// \brief processes any URIs in the info structures
    void _ProcessURIsInInfo(RobotBase::RobotBaseInfo& robotinfo, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc)
    {
        FOREACH(itConnected, robotinfo._vConnectedBodyInfos) {
            RobotBase::ConnectedBodyInfoPtr& pConnected = *itConnected;
            if( !pConnected->_uri.empty() ) {
                std::set<std::string> circularReference;
                _ProcessConnectedBodyURI(*pConnected, pConnected->_uri, fUnitScale, circularReference, alloc);
            }
        }
    }

    bool _ProcessConnectedBodyURI(RobotBase::ConnectedBodyInfo& connectedInfo, const std::string& uri, dReal fUnitScale, std::set<std::string>& circularReference, rapidjson::Document::AllocatorType& alloc)
    {
        if( uri.empty() ) {
            return false;
        }
        if( _deserializeOptions & IDO_IgnoreReferenceUri ) {
            return false;
        }
        if (circularReference.find(uri) != circularReference.end()) {
            RAVELOG_ERROR_FORMAT("Load connected body failed: circular reference in '%s' found", uri);
            return false;
        }

        //pConnected->_uri;
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);

        std::string fullFilename = ResolveURI(uri, GetOpenRAVESchemeAliases());
        if (fullFilename.empty() && fragment.empty()) {
            return false;
        }

        circularReference.insert(uri);
        boost::shared_ptr<const rapidjson::Document> prConnectedBody = _GetJsonDocumentFromUri(fullFilename, alloc);

        if (!!prConnectedBody && prConnectedBody->HasMember("bodies")) {
            const rapidjson::Document& rConnectedBody = *prConnectedBody;
            for(rapidjson::Value::ConstValueIterator itBody = rConnectedBody["bodies"].Begin(); itBody != rConnectedBody["bodies"].End(); itBody++) {
                std::string id = orjson::GetStringJsonValueByKey(*itBody, "id");
                if ( id == fragment || fragment.empty() ) { // if fragment is empty, deserialize the first one
                    std::string newReferenceUri = orjson::GetJsonValueByKey<std::string>(*itBody, "referenceUri", "");
                    if (!newReferenceUri.empty()) {
                        return _ProcessConnectedBodyURI(connectedInfo, newReferenceUri, fUnitScale, circularReference, alloc);
                    }
                    else {
                        connectedInfo.DeserializeJSON(*itBody, fUnitScale, _deserializeOptions);
                        return true;
                    }
                }
            }
        }

        return false; // couldn't find anything
    }

    dReal _fGlobalScale;
    EnvironmentBasePtr _penv;
    int _deserializeOptions; ///< options used for deserializing
    std::string _filename;
    std::string _uri;
    std::vector<std::string> _vOpenRAVESchemeAliases;

    std::set<std::string> _bodyUniqueIds; ///< unique id for bodies in current doc

    std::map<std::string, boost::shared_ptr<const rapidjson::Document> > _rapidJSONDocuments; ///< cache for opened rapidjson Documents
    std::map<boost::shared_ptr<const rapidjson::Document>, std::map<std::string, rapidjson::Value::ConstValueIterator> > _rapidJSONObjects;  ///< cache for opened rapidjson objects
};

bool RaveParseJSON(EnvironmentBasePtr penv, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseJSON(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseJSON(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, pprobot, alloc);
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
    return reader.ExtractFirst(doc, ppbody, alloc);
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
    return reader.ExtractFirst(doc, pprobot, alloc);
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
    return reader.ExtractOne(doc, ppbody, uri, alloc);
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
    return reader.ExtractOne(doc, pprobot, uri, alloc);
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
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    orjson::ParseJson(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, pprobot, alloc);
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
    return reader.ExtractFirst(doc, ppbody, alloc);
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
    return reader.ExtractFirst(doc, pprobot, alloc);
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
    return reader.ExtractOne(doc, ppbody, uri, alloc);
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
    return reader.ExtractOne(doc, pprobot, uri, alloc);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv);
    return reader.ExtractFirst(doc, pprobot, alloc);
}

}
