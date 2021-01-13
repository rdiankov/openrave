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

static bool _EndsWith(const std::string& fullString, const std::string& endString) {
    if (fullString.length() >= endString.length()) {
        return fullString.compare(fullString.length() - endString.length(), endString.length(), endString) == 0;
    }
    return false;
}

/// \brief if filename endswith oldSuffix, replace it with newSuffix
///
/// \return true if replaced oldSuffix with newSuffix
static bool _ReplaceFilenameSuffix(std::string& filename, const std::string& oldSuffix, const std::string& newSuffix) {
    // fix extension, replace dae with json
    // this is done for ease of migration
    if (_EndsWith(filename, oldSuffix)) {
        size_t len = filename.size();
        size_t suffixLen = oldSuffix.size();
        filename = filename.substr(0, len-suffixLen) + newSuffix;
        return true;
    }

    return false;
}

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

    JSONReader(const AttributesList& atts, EnvironmentBasePtr penv, const std::string& defaultSuffix) : _penv(penv), _defaultSuffix(defaultSuffix)
    {
        FOREACHC(itatt, atts) {
            if (itatt->first == "openravescheme") {
                std::stringstream ss(itatt->second);
                _vOpenRAVESchemeAliases = std::vector<std::string>((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
            }
            else if (itatt->first == "mustresolveuri") {
                _bMustResolveURI = _stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
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

    /// \brief Extrat all bodies and add them into environment
    bool ExtractAll(const rapidjson::Value& rEnvInfo, rapidjson::Document::AllocatorType& alloc)
    {
        EnvironmentBase::EnvironmentBaseInfo envInfo;
        _penv->ExtractInfo(envInfo);

        std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;

        std::string referenceUri = orjson::GetJsonValueByKey<std::string>(rEnvInfo, "referenceUri", "");
        if (_IsExpandableReferenceUri(referenceUri)) {
            std::string scheme, path, fragment;
            ParseURI(referenceUri, scheme, path, fragment);
            std::string fullFilename = ResolveURI(scheme, path, GetOpenRAVESchemeAliases());
            if (fullFilename.empty()) {
                RAVELOG_ERROR_FORMAT("env=%d, failed to resolve a filename from env referenceUri='%s'", _penv->GetId()%referenceUri);
                return false;
            }

            boost::shared_ptr<const rapidjson::Document> prReferenceEnvInfo = _GetDocumentFromFilename(fullFilename, alloc);
            if (!prReferenceEnvInfo) {
                RAVELOG_WARN_FORMAT("failed to load referenced body from filename '%s'", fullFilename);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load referenced body from uri '%s'", fullFilename, ORE_InvalidURI);
                }
                return false;
            }

            if( prReferenceEnvInfo->IsObject() ) {
                _ProcessEnvInfoBodies(envInfo, *prReferenceEnvInfo, alloc, mapProcessedConnectedBodyUris);
            }
        }

        _ProcessEnvInfoBodies(envInfo, rEnvInfo, alloc, mapProcessedConnectedBodyUris);
        std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
        _penv->UpdateFromInfo(envInfo, vCreatedBodies, vModifiedBodies, vRemovedBodies);
        
        return true;
    }

    bool ExtractFirst(const rapidjson::Value& doc, KinBodyPtr& ppbody, rapidjson::Document::AllocatorType& alloc)
    {
        // extract the first articulated system found.
        dReal fUnitScale = _GetUnitScale(doc);
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, ppbody, doc, fUnitScale, alloc, mapProcessedConnectedBodyUris);
            }
        }
        return false;
    }

    bool ExtractFirst(const rapidjson::Value& doc, RobotBasePtr& pprobot, rapidjson::Document::AllocatorType& alloc)
    {
        // extract the first robot
        dReal fUnitScale = _GetUnitScale(doc);
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, pprobot, doc, fUnitScale, alloc, mapProcessedConnectedBodyUris);
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
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;
            dReal fUnitScale = _GetUnitScale(doc);
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                std::string bodyId;
                orjson::LoadJsonValueByKey(*itr, "id", bodyId);
                if (bodyId == fragment) {
                    return _Extract(*itr, ppbody, doc, fUnitScale, alloc, mapProcessedConnectedBodyUris);
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
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;
            dReal fUnitScale = _GetUnitScale(doc);
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                std::string bodyId;
                orjson::LoadJsonValueByKey(*itr, "id", bodyId);
                if (bodyId == fragment) {
                    return _Extract(*itr, pprobot, doc, fUnitScale, alloc, mapProcessedConnectedBodyUris);
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

        // need to convert absolute filename back to openrave:/filename
        std::string newFilename;
        if (RaveInvertFileLookup(newFilename, filename)) {
            _uri = _vOpenRAVESchemeAliases[0] + ":/" + newFilename;
        }
    }

protected:

    /// \brief returns true if the referenceUri is a valid URI that can be loaded
    bool _IsExpandableReferenceUri(const std::string& referenceUri) const
    {
        if (_deserializeOptions & IDO_IgnoreReferenceUri) {
            return false;
        }
        if (referenceUri.empty()) {
            return false;
        }
        std::string scheme, path, fragment;
        ParseURI(referenceUri, scheme, path, fragment);
        if (!fragment.empty()) {
            return true;
        }
        else if (!scheme.empty() && !path.empty()) {
            return true;
        }
        return false;
    }

    boost::shared_ptr<const rapidjson::Document> _GetDocumentFromFilename(const std::string& fullFilename, rapidjson::Document::AllocatorType& alloc)
    {
        boost::shared_ptr<const rapidjson::Document> doc;
        // TODO: optimize this. for the first time doc is cached, all the expandable object will never get cached, because we are not update document cache after expand any body
        if (_rapidJSONDocuments.find(fullFilename) != _rapidJSONDocuments.end()) {
            doc = _rapidJSONDocuments[fullFilename];
        }
        else {
            boost::shared_ptr<rapidjson::Document> newDoc;
            if (_EndsWith(fullFilename, ".json")) {
                newDoc.reset(new rapidjson::Document(&alloc));
                OpenRapidJsonDocument(fullFilename, *newDoc);
            }
            else if (_EndsWith(fullFilename, ".msgpack")) {
                newDoc.reset(new rapidjson::Document(&alloc));
                OpenMsgPackDocument(fullFilename, *newDoc);
            }
            if (!!newDoc) {
                doc = newDoc;
                _rapidJSONDocuments[fullFilename] = doc;
            }
        }
        return doc;
    }

    void _ProcessEnvInfoBodies(EnvironmentBase::EnvironmentBaseInfo& envInfo, const rapidjson::Value& rEnvInfo, rapidjson::Document::AllocatorType& alloc, std::map<RobotBase::ConnectedBodyInfoPtr, std::string>& mapProcessedConnectedBodyUris)
    {
        dReal fUnitScale = _GetUnitScale(rEnvInfo);
        if (rEnvInfo.HasMember("bodies")) {
            for (rapidjson::Value::ConstValueIterator it = rEnvInfo["bodies"].Begin(); it != rEnvInfo["bodies"].End(); it++) {
                std::string bodyId = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                std::string referenceUri = orjson::GetJsonValueByKey<std::string>(*it, "referenceUri", "");
                if (_IsExpandableReferenceUri(referenceUri)) {
                    std::set<std::string> circularReference;
                    if (!_ExpandRapidJSON(envInfo, bodyId, rEnvInfo, referenceUri, circularReference, fUnitScale, alloc)) {
                        RAVELOG_WARN_FORMAT("failed to load referenced body from uri '%s'", referenceUri);
                        if (_bMustResolveURI) {
                            throw OPENRAVE_EXCEPTION_FORMAT("failed to load referenced body from uri '%s'", referenceUri, ORE_InvalidURI);
                        }
                    }
                }
            }
        }

        envInfo.DeserializeJSON(rEnvInfo, fUnitScale, _deserializeOptions);
        FOREACH(itBodyInfo, envInfo._vBodyInfos) {
            KinBody::KinBodyInfoPtr& pKinBodyInfo = *itBodyInfo;
            // ensure uri is set
            if (pKinBodyInfo->_uri.empty() && !pKinBodyInfo->_id.empty()) {
                pKinBodyInfo->_uri = _CanonicalizeURI("#" + pKinBodyInfo->_id);
            }
            RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
            if( !!pRobotBaseInfo ) {
                _ProcessURIsInRobotBaseInfo(*pRobotBaseInfo, rEnvInfo, fUnitScale, alloc, mapProcessedConnectedBodyUris);
            }
        }
    }

    /// \param rEnvInfo[in] used for resolving references pointing to the current environment
    bool _ExpandRapidJSON(EnvironmentBase::EnvironmentBaseInfo& envInfo, const std::string& originBodyId, const rapidjson::Value& rEnvInfo, const std::string& referenceUri, std::set<std::string>& circularReference, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc) {
        if (circularReference.find(referenceUri) != circularReference.end()) {
            RAVELOG_ERROR_FORMAT("failed to load scene, circular reference to '%s' found on body %s", referenceUri%originBodyId);
            return false;
        }
        RAVELOG_DEBUG_FORMAT("adding '%s' for tracking circular reference, so far %d uris tracked", referenceUri%circularReference.size());
        circularReference.insert(referenceUri);


        rapidjson::Value rKinBodyInfo; // holds the read data from referenceUri

        // parse the uri
        std::string scheme, path, fragment;
        ParseURI(referenceUri, scheme, path, fragment);

        // deal with uri that has just #originBodyId
        if(scheme.empty() && path.empty() && !fragment.empty()) {
            // reference to itself
            bool bFoundBody = false;
            if (rEnvInfo.HasMember("bodies")) {
                for(rapidjson::Value::ConstValueIterator it = rEnvInfo["bodies"].Begin(); it != rEnvInfo["bodies"].End(); it++) {
                    std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                    if (id == fragment) {
                        rKinBodyInfo.CopyFrom(*it, alloc);
                        bFoundBody = true;
                        break;
                    }
                }
            }
            if (!bFoundBody) {
                RAVELOG_ERROR_FORMAT("failed to find body using referenceUri '%s' in body %s", referenceUri%originBodyId);
                return false;
            }

            std::string nextReferenceUri = orjson::GetJsonValueByKey<std::string>(rKinBodyInfo, "referenceUri", "");
            if (_IsExpandableReferenceUri(nextReferenceUri)) {
                if (!_ExpandRapidJSON(envInfo, originBodyId, rEnvInfo, nextReferenceUri, circularReference, fUnitScale, alloc)) {
                    return false;
                }
            }
        }
        // deal with uri with scheme:/path#fragment
        else if (!scheme.empty() && !path.empty()) {
            // replace .dae to .json or .msgpack, depends on orignal document file defaultSuffix
            if( _ReplaceFilenameSuffix(path, ".dae", _defaultSuffix) ) {
                RAVELOG_WARN_FORMAT("env=%d, filename had '.dae' suffix, so changed to %s", _penv->GetId()%path);
            }
            std::string fullFilename = ResolveURI(scheme, path, GetOpenRAVESchemeAliases());
            if (fullFilename.empty()) {
                RAVELOG_ERROR_FORMAT("env=%d, failed to resolve referenceUri '%s' in body %s", _penv->GetId()%referenceUri%originBodyId);
                return false;
            }

            boost::shared_ptr<const rapidjson::Document> referenceDoc = _GetDocumentFromFilename(fullFilename, alloc);
            if (!referenceDoc || !(*referenceDoc).HasMember("bodies")) {
                RAVELOG_ERROR_FORMAT("referenced document cannot be loaded, or has no bodies: %s", fullFilename);
                return false;
            }

            bool bFoundBody = false;
            for(rapidjson::Value::ConstValueIterator it = (*referenceDoc)["bodies"].Begin(); it != (*referenceDoc)["bodies"].End(); it++) {
                std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                if (id == fragment || fragment.empty()) {
                    rKinBodyInfo.CopyFrom(*it, alloc);
                    bFoundBody = true;
                    break;
                }
            }
            if (!bFoundBody) {
                RAVELOG_ERROR_FORMAT("failed to find body using referenceUri '%s' in body %s", referenceUri%originBodyId);
                return false;
            }

            std::string nextReferenceUri = orjson::GetJsonValueByKey<std::string>(rKinBodyInfo, "referenceUri", "");
            if (_IsExpandableReferenceUri(nextReferenceUri)) {
                if (!_ExpandRapidJSON(envInfo, originBodyId, *referenceDoc, nextReferenceUri, circularReference, fUnitScale, alloc)) {
                    return false;
                }
            }
        }
        else {
            RAVELOG_WARN_FORMAT("ignoring invalid referenceUri '%s' in body %s", referenceUri%originBodyId);
            return false;
        }

        // search for originBodyId
        bool bFoundMatch = false;
        for(KinBody::KinBodyInfoPtr& pExistingBodyInfo : envInfo._vBodyInfos) {
            if( pExistingBodyInfo->_id == originBodyId ) {
                bool isExistingRobot = !!OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pExistingBodyInfo);
                bool isNewRobot = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "isRobot", isExistingRobot);

                if( isExistingRobot && !isNewRobot ) {
                    // new is not a robot, but previous was
                    KinBody::KinBodyInfoPtr pNewKinBodyInfo(new KinBody::KinBodyInfo());
                    *pNewKinBodyInfo= *((KinBody::KinBodyInfo*)pExistingBodyInfo.get());
                    pExistingBodyInfo = pNewKinBodyInfo;
                }
                else if( !isExistingRobot && isNewRobot ) {
                    RobotBase::RobotBaseInfoPtr pNewRobotInfo(new RobotBase::RobotBaseInfo());
                    *((KinBody::KinBodyInfo*)pNewRobotInfo.get())  = *pExistingBodyInfo;
                    pExistingBodyInfo = pNewRobotInfo;
                }

                pExistingBodyInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, _deserializeOptions);
                bFoundMatch = true;
                break;
            }
        }

        if( bFoundMatch ) {
            RAVELOG_DEBUG_FORMAT("loaded referenced body from uri '%s' for body %s", referenceUri%originBodyId);
        }
        else {
            KinBody::KinBodyInfoPtr pNewKinBodyInfo;
            bool isNewRobot = orjson::GetJsonValueByKey<bool>(rKinBodyInfo, "isRobot", false);
            if( isNewRobot ) {
                pNewKinBodyInfo.reset(new RobotBase::RobotBaseInfo());
            }
            else {
                pNewKinBodyInfo.reset(new KinBody::KinBodyInfo());
            }
            pNewKinBodyInfo->DeserializeJSON(rKinBodyInfo, fUnitScale, _deserializeOptions);
            pNewKinBodyInfo->_id = originBodyId;
            envInfo._vBodyInfos.push_back(pNewKinBodyInfo);
            RAVELOG_DEBUG_FORMAT("env=%d, could not find existing body with id='%s', so inserting it", _penv->GetId()%originBodyId);
        }
        return true;
    }

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

    bool _Extract(const rapidjson::Value& bodyValue, KinBodyPtr& pBodyOut, const rapidjson::Value& doc, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc, std::map<RobotBase::ConnectedBodyInfoPtr, std::string>& mapProcessedConnectedBodyUris)
    {
        // extract for robot
        if (orjson::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            RobotBasePtr pRobot;
            if (_Extract(bodyValue, pRobot, doc, fUnitScale, alloc, mapProcessedConnectedBodyUris)) { // TODO: change robot part to iterator
                pBodyOut = pRobot;
                return true;
            }
            return false;
        }

        KinBody::KinBodyInfoPtr pKinBodyInfo(new KinBody::KinBodyInfo());
        pKinBodyInfo->DeserializeJSON(bodyValue, fUnitScale, _deserializeOptions);
        RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);

        KinBodyPtr pBody;
        if( !!pRobotBaseInfo ) {
            _ProcessURIsInRobotBaseInfo(*pRobotBaseInfo, doc, fUnitScale, alloc, mapProcessedConnectedBodyUris);
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

    bool _Extract(const rapidjson::Value& bodyValue, RobotBasePtr& pRobotOut, const rapidjson::Value& doc, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc, std::map<RobotBase::ConnectedBodyInfoPtr, std::string>& mapProcessedConnectedBodyUris)
    {
        if (!orjson::GetJsonValueByKey<bool>(bodyValue, "isRobot")) {
            return false;
        }

        RobotBase::RobotBaseInfoPtr pRobotBaseInfo(new RobotBase::RobotBaseInfo());
        pRobotBaseInfo->DeserializeJSON(bodyValue, fUnitScale, _deserializeOptions);

        _ProcessURIsInRobotBaseInfo(*pRobotBaseInfo, doc, fUnitScale, alloc, mapProcessedConnectedBodyUris);

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
    ///
    /// \param[in] rEnvInfo used for resolving references pointing to the current environment
    /// \param[inout] mapProcessedConnectedBodyUris a map of the already processed connected bodies with their URIs. This is to prevent multiple passes from opening the same files
    void _ProcessURIsInRobotBaseInfo(RobotBase::RobotBaseInfo& robotInfo, const rapidjson::Value& rEnvInfo, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc, std::map<RobotBase::ConnectedBodyInfoPtr, std::string>& mapProcessedConnectedBodyUris)
    {
        FOREACH(itConnected, robotInfo._vConnectedBodyInfos) {
            RobotBase::ConnectedBodyInfoPtr& pConnected = *itConnected;
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string>::iterator itProcessedEntry = mapProcessedConnectedBodyUris.find(pConnected);
            if( itProcessedEntry != mapProcessedConnectedBodyUris.end() && itProcessedEntry->second == pConnected->_uri ) {
                continue;
            }

            if( !_IsExpandableReferenceUri(pConnected->_uri) ) {
                continue;
            }
            std::set<std::string> circularReference;
            EnvironmentBase::EnvironmentBaseInfo envInfo;
            if (!_ExpandRapidJSON(envInfo, "__connectedBody__", rEnvInfo, pConnected->_uri, circularReference, fUnitScale, alloc)) {
                RAVELOG_ERROR_FORMAT("env=%d,failed to load connected body from uri '%s'", _penv->GetId()%pConnected->_uri);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load connected body from uri '%s'", pConnected->_uri, ORE_InvalidURI);
                }
                continue;
            }
            if (envInfo._vBodyInfos.size() != 1) {
                RAVELOG_ERROR_FORMAT("failed to load connected body from uri '%s', number of bodies loaded %d", pConnected->_uri%envInfo._vBodyInfos.size());
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load connected body from uri '%s', number of bodies loaded %d", pConnected->_uri%envInfo._vBodyInfos.size(), ORE_InvalidURI);
                }
                continue;
            }
            KinBody::KinBodyInfoPtr pKinBodyInfo = envInfo._vBodyInfos[0];
            RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
            if (!pRobotBaseInfo) {
                RAVELOG_ERROR_FORMAT("failed to load connected body from uri '%s', referenced body not a robot", pConnected->_uri);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load connected body from uri '%s', referenced body not a robot", pConnected->_uri, ORE_InvalidURI);
                }
                continue;
            }
            pConnected->_vLinkInfos = pRobotBaseInfo->_vLinkInfos;
            pConnected->_vJointInfos = pRobotBaseInfo->_vJointInfos;
            pConnected->_vManipulatorInfos = pRobotBaseInfo->_vManipulatorInfos;
            pConnected->_vAttachedSensorInfos = pRobotBaseInfo->_vAttachedSensorInfos;
            pConnected->_vGripperInfos = pRobotBaseInfo->_vGripperInfos;
            mapProcessedConnectedBodyUris[pConnected] = pConnected->_uri;
        }
    }

    dReal _fGlobalScale = 1.0;
    EnvironmentBasePtr _penv;
    int _deserializeOptions = 0; ///< options used for deserializing
    std::string _filename; ///< original filename used to open reader
    std::string _uri; ///< original uri used to open reader
    std::string _defaultSuffix; ///< defaultSuffix of the main document, either ".json" or ".msgpack"
    std::vector<std::string> _vOpenRAVESchemeAliases;
    bool _bMustResolveURI = false; ///< if true, throw exception if uri does not resolve

    std::map<std::string, boost::shared_ptr<const rapidjson::Document> > _rapidJSONDocuments; ///< cache for opened rapidjson Documents
};

bool RaveParseJSON(EnvironmentBasePtr penv, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseJSON(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseJSON(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    return reader.ExtractFirst(doc, pprobot, alloc);
}

bool RaveParseJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    JSONReader reader(atts, penv, ".json");
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
    JSONReader reader(atts, penv, ".json");
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
    JSONReader reader(atts, penv, ".json");
    reader.SetFilename(fullFilename);
    return reader.ExtractFirst(doc, pprobot, alloc);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
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
    JSONReader reader(atts, penv, ".json");
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
    JSONReader reader(atts, penv, ".json");
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
    JSONReader reader(atts, penv, ".json");
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    orjson::ParseJson(doc, data);
    JSONReader reader(atts, penv, ".json");
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    orjson::ParseJson(doc, data);
    JSONReader reader(atts, penv, ".json");
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
    JSONReader reader(atts, penv, ".msgpack");
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
    JSONReader reader(atts, penv, ".msgpack");
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
    JSONReader reader(atts, penv, ".msgpack");
    reader.SetFilename(fullFilename);
    return reader.ExtractFirst(doc, pprobot, alloc);
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".msgpack");
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
    JSONReader reader(atts, penv, ".msgpack");
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
    JSONReader reader(atts, penv, ".msgpack");
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
    JSONReader reader(atts, penv, ".msgpack");
    return reader.ExtractAll(doc, alloc);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv, ".msgpack");
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv, ".msgpack");
    return reader.ExtractFirst(doc, pprobot, alloc);
}

}
