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

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

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
    try {
        MsgPack::ParseMsgPack(doc, ifs);
    }
    catch(const std::exception& ex) {
        throw OPENRAVE_EXCEPTION_FORMAT("Failed to parse msgpack format for file '%s': %s", filename%ex.what(), ORE_Failed);
    }
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

static std::string ResolveURI(const std::string& scheme, const std::string& path, const std::string& curdir, const std::vector<std::string>& vOpenRAVESchemeAliases)
{
    if (scheme.empty() && path.empty() ) {
        return std::string();
    }
    else if (scheme == "file")
    {
        return RaveFindLocalFile(path, curdir);
    }
    else if (find(vOpenRAVESchemeAliases.begin(), vOpenRAVESchemeAliases.end(), scheme) != vOpenRAVESchemeAliases.end())
    {
        return RaveFindLocalFile(path, curdir);
    }

    return std::string();
}
/// \brief resolve a uri
static std::string ResolveURI(const std::string& uri, const std::string& curdir, const std::vector<std::string>& vOpenRAVESchemeAliases)
{
    std::string scheme, path, fragment;
    ParseURI(uri, scheme, path, fragment);
    return ResolveURI(scheme, path, curdir, vOpenRAVESchemeAliases);
}

static std::string CanonicalizeURI(const std::string& suburi, const std::string& parentUri, const std::string& parentFilename)
{
    std::string scheme, path, fragment;
    ParseURI(suburi, scheme, path, fragment);

    if (scheme.empty() && path.empty() ) {
        if (!parentUri.empty()) {
            std::string scheme2, path2, fragment2;
            ParseURI(parentUri, scheme2, path2, fragment2);
            return scheme2 + ":" + path2 + "#" + fragment;
        }
        return std::string("file:") + parentFilename + "#" + fragment;
    }
    return suburi;
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
            else if (itatt->first == "mustresolveenvuri") {
                _bMustResolveEnvironmentURI = _stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if (itatt->first == "scalegeometry") {
                stringstream ss(itatt->second);
                // take the first argument given from scalegeometry to set as the overall geometry scale
                ss >> _fGeomScale;
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
    bool ExtractAll(const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, rapidjson::Document::AllocatorType& alloc)
    {
        uint64_t starttimeus = utils::GetMonotonicTime();
        vCreatedBodies.clear();
        vModifiedBodies.clear();
        vRemovedBodies.clear();
        if( !rEnvInfo.IsObject() ) {
            throw OPENRAVE_EXCEPTION_FORMAT("The environment data needs to be a valid dictionary. Currently it is '%s'", orjson::DumpJson(rEnvInfo), ORE_InvalidArguments);
        }

        EnvironmentBase::EnvironmentBaseInfo envInfo;
        if( updateMode == UFIM_OnlySpecifiedBodiesExact ) {
            _ExtractSpecifiedBodies(envInfo, rEnvInfo);
        }
        else {
            // extract everything
            _penv->ExtractInfo(envInfo);
        }

        std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;

        std::string referenceUri = orjson::GetJsonValueByKey<std::string>(rEnvInfo, "referenceUri", "");
        if (_IsExpandableReferenceUri(referenceUri)) {
            std::string scheme, path, fragment;
            ParseURI(referenceUri, scheme, path, fragment);
            std::string fullFilename = ResolveURI(scheme, path, std::string(), GetOpenRAVESchemeAliases());
            if (fullFilename.empty()) {
#ifdef HAVE_BOOST_FILESYSTEM
                // check using the current _filename dir as the current dir
                fullFilename = ResolveURI(scheme, path, boost::filesystem::path(_filename).parent_path().string(), GetOpenRAVESchemeAliases());
#endif
                if (fullFilename.empty()) {
                    RAVELOG_ERROR_FORMAT("env=%d, failed to resolve a filename from env referenceUri='%s'", _penv->GetId()%referenceUri);
                    return false;
                }
            }

            boost::shared_ptr<const rapidjson::Document> prReferenceEnvInfo = _GetDocumentFromFilename(fullFilename, alloc);
            if (!prReferenceEnvInfo) {
                RAVELOG_WARN_FORMAT("env=%d, failed to load referenced body from filename '%s'", _penv->GetId()%fullFilename);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("env=%d, failed to load referenced body from referenceUri='%s'", _penv->GetId()%referenceUri, ORE_InvalidURI);
                }
                return false;
            }
            RAVELOG_VERBOSE_FORMAT("resolved fullFilename=%s", fullFilename);

            if( updateMode == UFIM_OnlySpecifiedBodiesExact ) {
                _ExtractSpecifiedBodies(envInfo, *prReferenceEnvInfo);
            }

            if( prReferenceEnvInfo->IsObject() ) {
                _ProcessEnvInfoBodies(envInfo, *prReferenceEnvInfo, alloc, referenceUri, fullFilename, mapProcessedConnectedBodyUris);
            }
        }
        else if( !referenceUri.empty() ) {
            if( _bMustResolveEnvironmentURI ) {
                throw OPENRAVE_EXCEPTION_FORMAT("Failed to load env referenceUri='%s' from file '%s'", referenceUri%_filename, ORE_InvalidURI);
            }

            RAVELOG_ERROR_FORMAT("Failed to load env referenceUri='%s' from file '%s'", referenceUri%_filename);
        }

        _ProcessEnvInfoBodies(envInfo, rEnvInfo, alloc, _uri, _filename, mapProcessedConnectedBodyUris);

        std::vector<KinBody::KinBodyInfoPtr>::iterator itBodyInfo = envInfo._vBodyInfos.begin();
        while(itBodyInfo != envInfo._vBodyInfos.end()) {
            const KinBody::KinBodyInfoPtr& pKinBodyInfo = *itBodyInfo;
            if( pKinBodyInfo->_name.empty() ) {
                if( _bIgnoreInvalidBodies ) {
                    itBodyInfo = envInfo._vBodyInfos.erase(itBodyInfo);
                    continue;
                }
                else {
                    int options = 0;
                    dReal fUnitScale = 1;
                    rapidjson::Document rTempKinBodyInfo;
                    pKinBodyInfo->SerializeJSON(rTempKinBodyInfo, rTempKinBodyInfo.GetAllocator(), fUnitScale, options);
                    throw OPENRAVE_EXCEPTION_FORMAT("Has body with no name in file '%s', so cannot load the scene: %s", _filename%orjson::DumpJson(rTempKinBodyInfo), ORE_InvalidArguments);
                }
            }

            ++itBodyInfo;
        }

        // have to remove any duplicate names, prioritize ones that have higher index
        for(int iBody = 0; iBody+1 < (int) envInfo._vBodyInfos.size(); ++iBody) {
            bool bFoundLast = false;
            int iTest = envInfo._vBodyInfos.size()-1;
            while(iTest > iBody) {
                const std::string& bodyname = envInfo._vBodyInfos[iBody]->_name;
                if( envInfo._vBodyInfos[iTest]->_name == bodyname ) {
                    if( !bFoundLast ) {
                        RAVELOG_WARN_FORMAT("env=%d, remove redundant entry %d and replace with %d with body name '%s'", _penv->GetId()%iBody%iTest%bodyname);
                        envInfo._vBodyInfos[iBody] = envInfo._vBodyInfos[iTest];
                        bFoundLast = true;
                    }
                    else {
                        RAVELOG_WARN_FORMAT("env=%d, remove redundant entry %d with body name '%s'", _penv->GetId()%iTest%bodyname);
                    }
                    envInfo._vBodyInfos.erase(envInfo._vBodyInfos.begin()+iTest);
                }
                --iTest;
            }
        }

        // clean up any invalid connected bodies
        for(int iBody = 0; iBody < (int) envInfo._vBodyInfos.size(); ++iBody) {
            RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(envInfo._vBodyInfos[iBody]);
            if( !!pRobotBaseInfo ) {
                std::vector<RobotBase::ConnectedBodyInfoPtr>::iterator it = pRobotBaseInfo->_vConnectedBodyInfos.begin();
                while(it != pRobotBaseInfo->_vConnectedBodyInfos.end()) {
                    RobotBase::ConnectedBodyInfoPtr& pconnected = *it;
                    if( pconnected->_linkname.empty() ) {
                        RAVELOG_WARN_FORMAT("env=%d erasing connected body id='%s' since it has empty linkname", _penv->GetId()%pconnected->_id);
                        it = pRobotBaseInfo->_vConnectedBodyInfos.erase(it);
                    }
                    else if( pconnected->_name.empty() ) {
                        RAVELOG_WARN_FORMAT("env=%d erasing connected body id='%s' since it has empty name", _penv->GetId()%pconnected->_id);
                        it = pRobotBaseInfo->_vConnectedBodyInfos.erase(it);
                    }
                    else {
                        ++it;
                    }
                }
            }
        }

        _penv->UpdateFromInfo(envInfo, vCreatedBodies, vModifiedBodies, vRemovedBodies, updateMode);
        RAVELOG_DEBUG_FORMAT("env=%d, loaded %d bodies in %u[us]", _penv->GetId()%envInfo._vBodyInfos.size()%(utils::GetMonotonicTime()-starttimeus));
        return true;
    }

    bool ExtractFirst(const rapidjson::Value& doc, KinBodyPtr& ppbody, rapidjson::Document::AllocatorType& alloc)
    {
        // extract the first articulated system found.
        dReal fUnitScale = _GetUnitScale(doc, 1.0);
        if (doc.HasMember("bodies") && (doc)["bodies"].IsArray()) {
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;
            for (rapidjson::Value::ConstValueIterator itr = (doc)["bodies"].Begin(); itr != (doc)["bodies"].End(); ++itr) {
                return _Extract(*itr, ppbody, doc, fUnitScale, alloc, mapProcessedConnectedBodyUris);
            }
        }
        return false;
    }

    bool ExtractOne(const rapidjson::Value& rEnvInfo, KinBodyPtr& ppbody, const string& uri, rapidjson::Document::AllocatorType& alloc)
    {
        std::string scheme, path, fragment;
        ParseURI(uri, scheme, path, fragment);
        if (fragment == "") {
            return ExtractFirst(rEnvInfo, ppbody, alloc);
        }

        // find the body by uri fragment
        if (rEnvInfo.HasMember("bodies") && (rEnvInfo)["bodies"].IsArray()) {
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;
            dReal fUnitScale = _GetUnitScale(rEnvInfo, 1.0);
            for (rapidjson::Value::ConstValueIterator itr = (rEnvInfo)["bodies"].Begin(); itr != (rEnvInfo)["bodies"].End(); ++itr) {
                std::string bodyId;
                orjson::LoadJsonValueByKey(*itr, "id", bodyId);
                if (bodyId == fragment) {
                    return _Extract(*itr, ppbody, rEnvInfo, fUnitScale, alloc, mapProcessedConnectedBodyUris);
                }
            }
        }
        return false;
    }

    void SetURI(const std::string& uri) {
        _uri = uri;
    }

    void SetFilename(const std::string& filename)
    {
        _filename = filename;

        // need to convert absolute filename back to openrave:/filename
        std::string newFilename;
        if (RaveInvertFileLookup(newFilename, filename)) {
            _uri = _vOpenRAVESchemeAliases[0] + ":/" + newFilename;
            RAVELOG_VERBOSE_FORMAT("Set filename to '%s' and uri to '%s'", filename%_uri);
        }
        else {
            RAVELOG_VERBOSE_FORMAT("Set filename to '%s'", filename);
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

    void _ProcessEnvInfoBodies(EnvironmentBase::EnvironmentBaseInfo& envInfo, const rapidjson::Value& rEnvInfo, rapidjson::Document::AllocatorType& alloc, const std::string& currentUri, const std::string& currentFilename, std::map<RobotBase::ConnectedBodyInfoPtr, std::string>& mapProcessedConnectedBodyUris)
    {
        dReal fUnitScale = _GetUnitScale(rEnvInfo, 1.0);
        std::vector<int> vInputToBodyInfoMapping;
        if (rEnvInfo.HasMember("bodies")) {
            const rapidjson::Value& rBodies = rEnvInfo["bodies"];
            vInputToBodyInfoMapping.resize(rBodies.Size(),-1); // -1, no mapping by default
            for(int iInputBodyIndex = 0; iInputBodyIndex < (int)rBodies.Size(); ++iInputBodyIndex) {
                const rapidjson::Value& rBodyInfo = rBodies[iInputBodyIndex];
                std::string bodyId = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "id", "");
                std::string bodyName = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "name", "");
                std::string referenceUri = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "referenceUri", "");
                if (_IsExpandableReferenceUri(referenceUri)) {
                    std::set<std::string> circularReference;
                    int insertIndex = _ExpandRapidJSON(envInfo, bodyId, bodyName, rEnvInfo, referenceUri, circularReference, fUnitScale, alloc, _filename);
                    if( insertIndex < 0 ) {
                        RAVELOG_WARN_FORMAT("failed to load referenced body from uri '%s' inside file '%s'", referenceUri%_filename);
                        if (_bMustResolveURI) {
                            throw OPENRAVE_EXCEPTION_FORMAT("failed to load referenced body from referenceUri='%s'", referenceUri, ORE_InvalidURI);
                        }
                    }
                    else {
                        vInputToBodyInfoMapping.at(iInputBodyIndex) = insertIndex;
                    }
                }
                else if( !referenceUri.empty() ) {
                    if (_bMustResolveURI) {
                        throw OPENRAVE_EXCEPTION_FORMAT("body '%s' has invalid referenceUri='%s", bodyId%referenceUri, ORE_InvalidURI);
                    }

                    RAVELOG_WARN_FORMAT("env=%d, body '%s' has invalid referenceUri='%s'", _penv->GetId()%bodyId%referenceUri);
                }
            }
        }

        envInfo.DeserializeJSONWithMapping(rEnvInfo, fUnitScale, _deserializeOptions, vInputToBodyInfoMapping);

        FOREACH(itBodyInfo, envInfo._vBodyInfos) {
            KinBody::KinBodyInfoPtr& pKinBodyInfo = *itBodyInfo;
            // ensure uri is set
            if (pKinBodyInfo->_uri.empty() && !pKinBodyInfo->_id.empty() ) {
                pKinBodyInfo->_uri = CanonicalizeURI("#" + pKinBodyInfo->_id, currentUri, currentFilename);
            }
            RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
            if( !!pRobotBaseInfo ) {
                _ProcessURIsInRobotBaseInfo(*pRobotBaseInfo, rEnvInfo, fUnitScale, alloc, mapProcessedConnectedBodyUris);
            }
        }
    }

    /// \param originBodyId optional parameter to search into the current envInfo. If empty, then always create a new object
    /// \param rEnvInfo[in] used for resolving references pointing to the current environment
    ///
    /// \return the index into envInfo._vBodyInfos where the entry was edited. If failed, then return -1
    int _ExpandRapidJSON(EnvironmentBase::EnvironmentBaseInfo& envInfo, const std::string& originBodyId, const std::string& originBodyName, const rapidjson::Value& rEnvInfo, const std::string& referenceUri, std::set<std::string>& circularReference, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc, const std::string& currentFilename) {
        if (circularReference.find(referenceUri) != circularReference.end()) {
            RAVELOG_ERROR_FORMAT("failed to load scene, circular reference to '%s' found on body %s", referenceUri%originBodyId);
            return -1;
        }
        RAVELOG_DEBUG_FORMAT("env=%d, adding '%s' for tracking circular reference, so far %d uris tracked. Scope is '%s'", _penv->GetId()%referenceUri%circularReference.size()%currentFilename);
        circularReference.insert(referenceUri);

        dReal fRefUnitScale = fUnitScale;  // unit scale for rRefKinBodyInfo
        rapidjson::Value rRefKinBodyInfo; // holds the read data from referenceUri

        // parse the uri
        std::string scheme, path, fragment;
        ParseURI(referenceUri, scheme, path, fragment);

        int insertIndex = -1;

        // deal with uri that has just #originBodyId
        if(scheme.empty() && path.empty() && !fragment.empty()) {
            // reference to itself
            bool bFoundBody = false;
            if (rEnvInfo.HasMember("bodies")) {
                for(rapidjson::Value::ConstValueIterator it = rEnvInfo["bodies"].Begin(); it != rEnvInfo["bodies"].End(); it++) {
                    std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                    if (id == fragment) {
                        rRefKinBodyInfo.CopyFrom(*it, alloc);
                        bFoundBody = true;
                        break;
                    }
                }
            }
            if (!bFoundBody) {
                RAVELOG_ERROR_FORMAT("failed to find body using referenceUri '%s' in body id=%s, name=%s", referenceUri%originBodyId%originBodyName);
                return -1;
            }

            std::string nextReferenceUri = orjson::GetJsonValueByKey<std::string>(rRefKinBodyInfo, "referenceUri", "");
            if (_IsExpandableReferenceUri(nextReferenceUri)) {
                insertIndex = _ExpandRapidJSON(envInfo, originBodyId, originBodyName, rEnvInfo, nextReferenceUri, circularReference, fUnitScale, alloc, currentFilename);
                // regardless of insertIndex, should fall through so can process rEnvInfo
            }
            else if( !nextReferenceUri.empty() ) {
                RAVELOG_ERROR_FORMAT("nextReferenceUri='%s' is not a valid URI. Scope is '%s'", nextReferenceUri%currentFilename);
            }
        }
        // deal with uri with scheme:/path#fragment
        else if (!scheme.empty() && !path.empty()) {
            // replace .dae to .json or .msgpack, depends on orignal document file defaultSuffix
            if( _ReplaceFilenameSuffix(path, ".dae", _defaultSuffix) ) {
                RAVELOG_WARN_FORMAT("env=%d, filename had '.dae' suffix, so changed to %s", _penv->GetId()%path);
            }
            std::string fullFilename = ResolveURI(scheme, path, std::string(), GetOpenRAVESchemeAliases());
            if (fullFilename.empty()) {
#ifdef HAVE_BOOST_FILESYSTEM
                fullFilename = ResolveURI(scheme, path, boost::filesystem::path(currentFilename).parent_path().string(), GetOpenRAVESchemeAliases());
#endif
                if (fullFilename.empty()) {
                    RAVELOG_ERROR_FORMAT("env=%d, failed to resolve referenceUri '%s' into a file. Coming from bodyId='%s', bodyName='%s' in file '%s'", _penv->GetId()%referenceUri%originBodyId%originBodyName%currentFilename);
                    if (_bMustResolveURI) {
                        throw OPENRAVE_EXCEPTION_FORMAT("Failed to resolve referenceUri='%s' in body definition '%s' from file '%s'", referenceUri%originBodyId%currentFilename, ORE_InvalidURI);
                    }

                    return -1;
                }
            }

            uint64_t beforeOpenStampUS = utils::GetMonotonicTime();

            boost::shared_ptr<const rapidjson::Document> referenceDoc = _GetDocumentFromFilename(fullFilename, alloc);
            if (!referenceDoc || !(*referenceDoc).HasMember("bodies")) {
                RAVELOG_ERROR_FORMAT("referenced document cannot be loaded, or has no bodies: %s", fullFilename);
                return -1;
            }

            fRefUnitScale = _GetUnitScale(*referenceDoc, 1.0); // for now default has to be meters... fUnitScale);

            bool bFoundBody = false;
            for(rapidjson::Value::ConstValueIterator it = (*referenceDoc)["bodies"].Begin(); it != (*referenceDoc)["bodies"].End(); it++) {
                std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                if (id == fragment || fragment.empty()) {
                    rRefKinBodyInfo.CopyFrom(*it, alloc);
                    bFoundBody = true;
                    break;
                }
            }
            if (!bFoundBody) {
                RAVELOG_ERROR_FORMAT("failed to find body using referenceUri '%s' in body id=%s, name=%s", referenceUri%originBodyId%originBodyName);
                return -1;
            }

            std::string nextReferenceUri = orjson::GetJsonValueByKey<std::string>(rRefKinBodyInfo, "referenceUri", "");

            if (_IsExpandableReferenceUri(nextReferenceUri)) {
                RAVELOG_DEBUG_FORMAT("env=%d, opened file '%s', found body from fragment='%s', and now processing its referenceUri='%s, took %u[us]'", _penv->GetId()%fullFilename%fragment%nextReferenceUri%(utils::GetMonotonicTime()-beforeOpenStampUS));
                insertIndex = _ExpandRapidJSON(envInfo, originBodyId, originBodyName, *referenceDoc, nextReferenceUri, circularReference, fUnitScale, alloc, fullFilename);
                // regardless of insertIndex, should fall through so can process rEnvInfo
            }
            else {
                RAVELOG_DEBUG_FORMAT("env=%d, opened file '%s', found body from fragment='%s', took %u[us]", _penv->GetId()%fullFilename%fragment%(utils::GetMonotonicTime()-beforeOpenStampUS));
            }

            if( insertIndex >= 0 ) {
                envInfo._vBodyInfos.at(insertIndex)->DeserializeJSON(rRefKinBodyInfo, fUnitScale, _deserializeOptions);
            }
        }
        else {
            RAVELOG_WARN_FORMAT("ignoring invalid referenceUri '%s' in body id=%s, name=%s", referenceUri%originBodyId%originBodyName);
            return -1;
        }

        // search for originBodyId if it isn't empty
        if( insertIndex < 0 && !originBodyId.empty() ) {
            for(int ibody = 0; ibody < (int)envInfo._vBodyInfos.size(); ++ibody) {
                KinBody::KinBodyInfoPtr& pExistingBodyInfo = envInfo._vBodyInfos[ibody];
                if( pExistingBodyInfo->_id == originBodyId ) {
                    bool isExistingRobot = !!OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pExistingBodyInfo);
                    bool isNewRobot = orjson::GetJsonValueByKey<bool>(rRefKinBodyInfo, "isRobot", isExistingRobot);

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

                    pExistingBodyInfo->DeserializeJSON(rRefKinBodyInfo, fRefUnitScale, _deserializeOptions);
                    insertIndex = ibody;
                    break;
                }
            }
        }

        if( insertIndex >= 0 ) {
            RAVELOG_DEBUG_FORMAT("env=%d, loaded referenced body '%s' with id='%s' from uri '%s'. Scope is '%s'", _penv->GetId()%envInfo._vBodyInfos.at(insertIndex)->_name%originBodyId%referenceUri%currentFilename);
        }
        else {
            KinBody::KinBodyInfoPtr pNewKinBodyInfo;
            bool isNewRobot = orjson::GetJsonValueByKey<bool>(rRefKinBodyInfo, "isRobot", false);
            if( isNewRobot ) {
                pNewKinBodyInfo.reset(new RobotBase::RobotBaseInfo());
            }
            else {
                pNewKinBodyInfo.reset(new KinBody::KinBodyInfo());
            }
            pNewKinBodyInfo->DeserializeJSON(rRefKinBodyInfo, fRefUnitScale, _deserializeOptions);

            if( !originBodyId.empty() ) {
                pNewKinBodyInfo->_id = originBodyId;
            }

            if( pNewKinBodyInfo->_name.empty() ) {
                RAVELOG_DEBUG_FORMAT("env=%d, kinbody id='%s' has empty name, coming from file '%s', perhaps it will get overwritten later. Data is %s. Scope is '%s'", _penv->GetId()%originBodyId%currentFilename%orjson::DumpJson(rRefKinBodyInfo)%currentFilename);
            }

            if( originBodyId.empty() ) {
                // try matching with names
                for(int ibody = 0; ibody < (int)envInfo._vBodyInfos.size(); ++ibody) {
                    KinBody::KinBodyInfoPtr& pExistingBodyInfo = envInfo._vBodyInfos[ibody];
                    if( !originBodyName.empty() && pExistingBodyInfo->_name == originBodyName ) {
                        RAVELOG_VERBOSE_FORMAT("env=%d, found existing body with id='%s', name='%s', so overwriting it. Scope is '%s'", _penv->GetId()%originBodyId%pNewKinBodyInfo->_name%currentFilename);
                        envInfo._vBodyInfos[ibody] = pNewKinBodyInfo;
                        insertIndex = ibody;
                        break;
                    }
                }
            }

            if( insertIndex < 0 ) {
                // might get overwritten later, so ok if name is empty
                insertIndex = envInfo._vBodyInfos.size();
                envInfo._vBodyInfos.push_back(pNewKinBodyInfo);
                RAVELOG_DEBUG_FORMAT("env=%d, could not find existing body with id='%s', name='%s', so inserting it. Scope is '%s'", _penv->GetId()%originBodyId%pNewKinBodyInfo->_name%currentFilename);
            }
        }
        return insertIndex;
    }

    inline dReal _GetUnitScale(const rapidjson::Value& doc, dReal defaultScale)
    {
        std::pair<std::string, dReal> unit = {"", defaultScale};
        orjson::LoadJsonValueByKey(doc, "unit", unit);
        return unit.second * _fGlobalScale * _fGeomScale;
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

    bool _Extract(const rapidjson::Value& rBodyInfo, KinBodyPtr& pBodyOut, const rapidjson::Value& rEnvInfo, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc, std::map<RobotBase::ConnectedBodyInfoPtr, std::string>& mapProcessedConnectedBodyUris)
    {
        // extract for robot
        bool isRobot = orjson::GetJsonValueByKey<bool>(rBodyInfo, "isRobot");
        std::string bodyId = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "id", "");
        std::string bodyName = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "name", "");
        std::string referenceUri = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "referenceUri", "");
        EnvironmentBase::EnvironmentBaseInfo envInfo; // dummy for reference uris
        int insertIndex = -1;
        if (_IsExpandableReferenceUri(referenceUri)) {
            std::set<std::string> circularReference; // dummy
            insertIndex = _ExpandRapidJSON(envInfo, bodyId, bodyName, rEnvInfo, referenceUri, circularReference, fUnitScale, alloc, _filename);
            if( insertIndex < 0 ) {
                RAVELOG_WARN_FORMAT("failed to load referenced body from uri '%s' inside file '%s'", referenceUri%_filename);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load referenced body from referenceUri='%s'", referenceUri, ORE_InvalidURI);
                }
            }
        }
        else if( !referenceUri.empty() ) {
            if (_bMustResolveURI) {
                throw OPENRAVE_EXCEPTION_FORMAT("body '%s' has invalid referenceUri='%s", bodyId%referenceUri, ORE_InvalidURI);
            }

            RAVELOG_WARN_FORMAT("env=%d, body '%s' has invalid referenceUri='%s'", _penv->GetId()%bodyId%referenceUri);
        }

        KinBody::KinBodyInfoPtr pKinBodyInfo;
        RobotBase::RobotBaseInfoPtr pRobotBaseInfo;
        if( insertIndex >= 0 ) {
            pKinBodyInfo = envInfo._vBodyInfos.at(insertIndex);
            if( isRobot ) {
                pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
            }
        }
        else {
            if( isRobot ) {
                pRobotBaseInfo.reset(new RobotBase::RobotBaseInfo());
                pKinBodyInfo = pRobotBaseInfo;
            }
            else {
                pKinBodyInfo.reset(new KinBody::KinBodyInfo());
            }
        }

        pKinBodyInfo->DeserializeJSON(rBodyInfo, fUnitScale, _deserializeOptions);

        KinBodyPtr pBody;
        if( !!pRobotBaseInfo ) {
            _ProcessURIsInRobotBaseInfo(*pRobotBaseInfo, rEnvInfo, fUnitScale, alloc, mapProcessedConnectedBodyUris);
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

            pBody = pRobot;
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
        _ExtractTransform(rBodyInfo, pBody, fUnitScale);
        pBodyOut = pBody;
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
            int insertIndex = _ExpandRapidJSON(envInfo, "__connectedBody__", "", rEnvInfo, pConnected->_uri, circularReference, fUnitScale, alloc, _filename);
            if( insertIndex < 0 ) {
                RAVELOG_ERROR_FORMAT("env=%d, failed to load connected body from uri '%s'", _penv->GetId()%pConnected->_uri);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load connected body from referenceUri='%s'", pConnected->_uri, ORE_InvalidURI);
                }
                continue;
            }
//            if (envInfo._vBodyInfos.size() != 1) {
//                RAVELOG_ERROR_FORMAT("failed to load connected body from uri '%s', number of bodies loaded %d", pConnected->_uri%envInfo._vBodyInfos.size());
//                if (_bMustResolveURI) {
//                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load connected body from uri '%s', number of bodies loaded %d", pConnected->_uri%envInfo._vBodyInfos.size(), ORE_InvalidURI);
//                }
//                continue;
//            }
            KinBody::KinBodyInfoPtr pKinBodyInfo = envInfo._vBodyInfos.at(insertIndex);
            RobotBase::RobotBaseInfoPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
            if (!pRobotBaseInfo) {
                RAVELOG_ERROR_FORMAT("env=%d, failed to load connected body from uri '%s', referenced body not a robot", _penv->GetId()%pConnected->_uri);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load connected body from referenceUri='%s', referenced body not a robot", pConnected->_uri, ORE_InvalidURI);
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

    /// \brief extracts the specified bodies of rEnvInfo into envInfo.
    ///
    /// If envInfo already holds bodies, tries to extract ones that do not conflict
    void _ExtractSpecifiedBodies(EnvironmentBase::EnvironmentBaseInfo& envInfo, const rapidjson::Value& rEnvInfo)
    {
        int numOriginalBodyInfos = (int)envInfo._vBodyInfos.size();

        // extract only the bodies used in rEnvInfo
        rapidjson::Value::ConstMemberIterator itBodies = rEnvInfo.FindMember("bodies");
        if( itBodies != rEnvInfo.MemberEnd() && itBodies->value.IsArray() ) {
            const rapidjson::Value& rBodies = itBodies->value;
            std::string id; // cache
            envInfo._vBodyInfos.reserve(rBodies.Size());
            for (rapidjson::Value::ConstValueIterator itBodyInfo = rBodies.Begin(); itBodyInfo != rBodies.End(); ++itBodyInfo) {
                const rapidjson::Value& rBodyInfo = *itBodyInfo;

                rapidjson::Value::ConstMemberIterator itId = rBodyInfo.FindMember("id");
                id.clear();
                if( itId != rBodyInfo.MemberEnd() ) {
                    orjson::LoadJsonValue(itId->value, id);
                }

                KinBodyPtr pbody;
                if( !id.empty() ) {
                    pbody = _penv->GetKinBodyById(id);
                }
                else {
                    // try the name
                    std::string& name = id; // cache
                    rapidjson::Value::ConstMemberIterator itName = rBodyInfo.FindMember("name");
                    if( itName != rBodyInfo.MemberEnd() ) {
                        orjson::LoadJsonValue(itName->value, name);
                        if( !name.empty() ) {
                            pbody = _penv->GetKinBody(name);
                        }
                    }
                }

                if( !!pbody && !envInfo._uri.empty() ) {
                    // make sure the URIs match
                    if( !pbody->GetURI().empty() ) {
                        bool bURIMatch = pbody->GetURI().size() >= envInfo._uri.size() && strncmp(pbody->GetURI().c_str(), envInfo._uri.c_str(), envInfo._uri.size()) == 0;
                        if( !bURIMatch ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, cannot update body '%s' with id='%s' since its uri '%s' does not match the env uri '%s'", _penv->GetId()%pbody->GetName()%pbody->GetId()%pbody->GetURI()%envInfo._uri);
                            continue;
                        }
                    }
                }

                if( !!pbody ) {
                    // check if pbody is already in envInfo (up to numOriginalBodyInfos)
                    bool bHasInfo = false;
                    for(int iCheckIndex = 0; iCheckIndex < numOriginalBodyInfos; ++iCheckIndex) {
                        const KinBody::KinBodyInfo& checkInfo = *envInfo._vBodyInfos[iCheckIndex];
                        if( !pbody->GetId().empty() ) {
                            if( checkInfo._id == pbody->GetId() ) {
                                bHasInfo = true;
                                break;
                            }
                        }
                        else {
                            if( checkInfo._name == pbody->GetName() ) {
                                bHasInfo = true;
                                break;
                            }
                        }
                    }

                    if( !bHasInfo ) {
                        bool isDeleted = orjson::GetJsonValueByKey<bool>(rBodyInfo, "__deleted__", false);
                        if( isDeleted ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, removing body '%s' since got __deleted__", _penv->GetId()%pbody->GetName());
                            _penv->Remove(pbody);
                        }
                        else {
                            KinBody::KinBodyInfoPtr pKinBodyInfo;
                            if (pbody->IsRobot()) {
                                RobotBase::RobotBaseInfoPtr pRobotInfo(new RobotBase::RobotBaseInfo());
                                RaveInterfaceCast<RobotBase>(pbody)->ExtractInfo(*pRobotInfo);
                                pKinBodyInfo = pRobotInfo;
                            }
                            else {
                                pKinBodyInfo.reset(new KinBody::KinBodyInfo());
                                pbody->ExtractInfo(*pKinBodyInfo);
                            }
                            envInfo._vBodyInfos.push_back(pKinBodyInfo);
                        }
                    }
                }
            }
        }
    }

    dReal _fGlobalScale = 1.0;
    dReal _fGeomScale = 1.0;
    EnvironmentBasePtr _penv;
    int _deserializeOptions = 0; ///< options used for deserializing
    std::string _filename; ///< original filename used to open reader
    std::string _uri; ///< original uri used to open reader
    std::string _defaultSuffix; ///< defaultSuffix of the main document, either ".json" or ".msgpack"
    std::vector<std::string> _vOpenRAVESchemeAliases;
    bool _bMustResolveURI = false; ///< if true, throw exception if object uri does not resolve
    bool _bMustResolveEnvironmentURI = false; ///< if true, throw exception if environment uri does not resolve
    bool _bIgnoreInvalidBodies = false; ///< if true, ignores any invalid bodies

    std::map<std::string, boost::shared_ptr<const rapidjson::Document> > _rapidJSONDocuments; ///< cache for opened rapidjson Documents
};

bool RaveParseJSON(EnvironmentBasePtr penv, const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseJSON(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseJSON(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    KinBodyPtr pbody;
    if( reader.ExtractFirst(doc, pbody, alloc) ) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseJSONFile(EnvironmentBasePtr penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    JSONReader reader(atts, penv, ".json");
    reader.SetFilename(fullFilename);
    rapidjson::Document rEnvInfo(&alloc);
    OpenRapidJsonDocument(fullFilename, rEnvInfo);
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
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
    KinBodyPtr pbody;
    if( reader.ExtractFirst(doc, pbody, alloc) ) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    std::string fullFilename = ResolveURI(uri, std::string(), reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document rEnvInfo(&alloc);
    OpenRapidJsonDocument(fullFilename, rEnvInfo);
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    std::string fullFilename = ResolveURI(uri, std::string(), reader.GetOpenRAVESchemeAliases());
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
    std::string fullFilename = ResolveURI(uri, std::string(), reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
    OpenRapidJsonDocument(fullFilename, doc);
    KinBodyPtr pbody;
    if( reader.ExtractOne(doc, pbody, uri, alloc) ) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseJSONData(EnvironmentBasePtr penv, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document rEnvInfo(&alloc);
    orjson::ParseJson(rEnvInfo, data);
    JSONReader reader(atts, penv, ".json");
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
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
    KinBodyPtr pbody;
    if( reader.ExtractFirst(doc, pbody, alloc) ) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.size() == 0 ) {
        return false;
    }
    rapidjson::Document rEnvInfo(&alloc);
    OpenMsgPackDocument(fullFilename, rEnvInfo);
    JSONReader reader(atts, penv, ".msgpack");
    reader.SetFilename(fullFilename);
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
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
    KinBodyPtr pbody;
    if( reader.ExtractFirst(doc, pbody, alloc) ) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".msgpack");
    std::string fullFilename = ResolveURI(uri, std::string(), reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document rEnvInfo(&alloc);
    OpenMsgPackDocument(fullFilename, rEnvInfo);
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".msgpack");
    std::string fullFilename = ResolveURI(uri, std::string(), reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        RAVELOG_DEBUG_FORMAT("could not resolve uri='%s' into a path", uri);
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
    std::string fullFilename = ResolveURI(uri, std::string(), reader.GetOpenRAVESchemeAliases());
    if (fullFilename.size() == 0 ) {
        return false;
    }
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
    OpenMsgPackDocument(fullFilename, doc);
    KinBodyPtr pbody;
    if( reader.ExtractOne(doc, pbody, uri, alloc) ) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document rEnvInfo(&alloc);
    MsgPack::ParseMsgPack(rEnvInfo, data);
    JSONReader reader(atts, penv, ".msgpack");
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
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
    KinBodyPtr pbody;
    if( reader.ExtractFirst(doc, pbody, alloc) ) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

}
