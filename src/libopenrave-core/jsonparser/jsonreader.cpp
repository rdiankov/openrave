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
#define RAPIDJSON_HAS_STDSTRING 1
#include "jsoncommon.h"
#include "stringutils.h"

#if OPENRAVE_CURL
#include "jsondownloader.h"
#endif

#include <openrave/openravejson.h>
#include <openrave/openravemsgpack.h>
#include <openrave/openrave.h>
#include <openrave/openraveexception.h>
#include <rapidjson/istreamwrapper.h>
#include <string>
#include <fstream>
#include <boost/unordered_set.hpp>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

namespace OpenRAVE {

int64_t ConvertIsoFormatDateTimeToLinuxTimeUS(const char* pIsoFormatDateTime); // declared in libopenrave.h

/// \brief if filename endswith oldSuffix, replace it with newSuffix
///
/// \return true if replaced oldSuffix with newSuffix
static bool _ReplaceFilenameSuffix(std::string& filename, const std::string& oldSuffix, const std::string& newSuffix) {
    // fix extension, replace dae with json
    // this is done for ease of migration
    if (RemoveSuffix(filename, oldSuffix)) {
        filename += newSuffix;
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

/// \brief open and cache an encrypted msgpack document
static void OpenEncryptedMsgPackDocument(const std::string& filename, rapidjson::Document& doc)
{
    std::ifstream ifs(filename);
    std::stringstream ss;
    if (GpgDecrypt(ifs, ss)) {
        try {
            MsgPack::ParseMsgPack(doc, ss);
        }
        catch(const std::exception& ex) {
            throw OPENRAVE_EXCEPTION_FORMAT("Failed to parse msgpack format for file '%s': %s", filename%ex.what(), ORE_Failed);
        }
    }
}

/// \brief open and cache an encrypted json document
static void OpenEncryptedJSONDocument(const std::string& filename, rapidjson::Document& doc)
{
    std::ifstream ifs(filename);
    std::stringstream ss;
    if (GpgDecrypt(ifs, ss)) {
        rapidjson::IStreamWrapper isw(ss);
        rapidjson::ParseResult ok = doc.ParseStream<rapidjson::kParseFullPrecisionFlag>(isw);
        if (!ok) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to parse json document \"%s\"", filename, ORE_InvalidArguments);
        }
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
static std::string ResolveURI(const char* pUri, const std::string& curdir, const std::vector<std::string>& vOpenRAVESchemeAliases)
{
    std::string scheme, path, fragment;
    ParseURI(pUri, scheme, path, fragment);
    return ResolveURI(scheme, path, curdir, vOpenRAVESchemeAliases);
}

static std::string CanonicalizeURI(const std::string& suburi, const std::string& parentUri, const std::string& parentFilename)
{
    std::string scheme, path, fragment;
    ParseURI(suburi.c_str(), scheme, path, fragment);

    if (scheme.empty() && path.empty() ) {
        if (!parentUri.empty()) {
            std::string scheme2, path2, fragment2;
            ParseURI(parentUri.c_str(), scheme2, path2, fragment2);
            return scheme2 + ":" + path2 + "#" + fragment;
        }
        if (!parentFilename.empty()) {
            return std::string("file:") + parentFilename + "#" + fragment;
        }
    }
    return suburi;
}

class JSONReader
{
public:

    JSONReader(const AttributesList& atts, EnvironmentBasePtr penv, const std::string& defaultSuffix) : _penv(penv), _defaultSuffix(defaultSuffix)
    {
        std::string remoteUrl;
        std::string unixEndpoint;

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
            else if (itatt->first == "remoteurl") {
                remoteUrl = itatt->second;
            }
            else if (itatt->first == "unixendpoint") {
                unixEndpoint = itatt->second;
            }
            else if (itatt->first == "timeout") {
                dReal timeout = 0;
                stringstream ss(itatt->second);
                ss >> timeout;
                _downloadTimeoutUS = timeout * 1000000; // convert timeout from seconds to microseconds
            }
            else if (itatt->first == "excludeBodyId") {
                _excludeBodyIds.emplace(itatt->second);
            }
        }
        if (_vOpenRAVESchemeAliases.size() == 0) {
            _vOpenRAVESchemeAliases.push_back("openrave");
        }

        // set global scale when initalize jsonreader.
        _fGlobalScale = GetLengthUnitStandardValue<dReal>(_penv->GetUnitInfo().lengthUnit);
        _deserializeOptions = 0;

        if (!remoteUrl.empty()) {
#if OPENRAVE_CURL
            _pDownloader = boost::make_shared<JSONDownloader>(_rapidJSONDocuments, _vOpenRAVESchemeAliases, remoteUrl, unixEndpoint);
#else
            throw OPENRAVE_EXCEPTION_FORMAT("\"remoteurl\" option is not supported, have to compile openrave with CURL support first", _filename, ORE_InvalidArguments);
#endif
        }
    }

    virtual ~JSONReader()
    {
    }

    const std::vector<std::string>& GetOpenRAVESchemeAliases() const {
        return _vOpenRAVESchemeAliases;
    }

    /// \brief Extract all bodies and add them into environment
    bool ExtractAll(const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, rapidjson::Document::AllocatorType& alloc)
    {
        uint64_t starttimeus = utils::GetMonotonicTime();
        _contextdesc = str(boost::format("env=%s, ExtractAll, ")%_penv->GetNameId());
        vCreatedBodies.clear();
        vModifiedBodies.clear();
        vRemovedBodies.clear();
        if( !rEnvInfo.IsObject() ) {
            throw OPENRAVE_EXCEPTION_FORMAT("The environment data needs to be a valid dictionary. Currently it is '%s'", orjson::DumpJson(rEnvInfo), ORE_InvalidArguments);
        }

        std::vector<KinBodyPtr> vRemovedBodiesExtra; // cannot use vRemovedBodies because UpdateFromInfo later clears it
        EnvironmentBase::EnvironmentBaseInfo envInfo;
        if( updateMode == UFIM_OnlySpecifiedBodiesExact ) {
            _ExtractSpecifiedBodies(envInfo, rEnvInfo, vRemovedBodiesExtra);
        }
        else {
            // extract everything
            _penv->ExtractInfo(envInfo);
        }

        {
            const char *const modifiedAt = orjson::GetCStringJsonValueByKey(rEnvInfo, "modifiedAt");
            if ( modifiedAt != nullptr ) {
                envInfo._lastModifiedAtUS = ConvertIsoFormatDateTimeToLinuxTimeUS(modifiedAt);
            }
        }
        {
            rapidjson::Value::ConstMemberIterator itRevisionId = rEnvInfo.FindMember("revisionId");
            if( itRevisionId != rEnvInfo.MemberEnd() ) {
                int64_t revisionId = 0;
                orjson::LoadJsonValue(itRevisionId->value, revisionId);
                envInfo._revisionId = revisionId;
            }
        }

        std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;

        const char* pReferenceUri = orjson::GetCStringJsonValueByKey(rEnvInfo, "referenceUri", "");

        // If remote URL is provided, start the process to download everything and load it into the json map
        if (IsDownloadingFromRemote()) {
#if OPENRAVE_CURL
            JSONDownloaderScope jsonDownload(_contextdesc, *_pDownloader, alloc, !(_deserializeOptions & IDO_IgnoreReferenceUri));
            jsonDownload.QueueDownloadReferenceURIs(rEnvInfo);
            if( !jsonDownload.WaitForDownloads(_bMustResolveURI, _downloadTimeoutUS) ) {
                RAVELOG_VERBOSE("failed downloads");
                //return false;
            }
#endif
        }

        // Keep going up the line and checking for all referenceUri bodies
        if (_IsExpandableReferenceUri(pReferenceUri)) {
            std::string scheme, path, fragment;
            ParseURI(pReferenceUri, scheme, path, fragment);

            std::string fullFilename;
            if (IsDownloadingFromRemote()) {
                // here we use fullFilename as key to look up in _rapidJSONDocuments
                if (scheme == "file") {
                    fullFilename = path;
                } else {
                    fullFilename = scheme + ":" + path;
                }
            }
            else {
                fullFilename = ResolveURI(scheme, path, std::string(), GetOpenRAVESchemeAliases());
            }
            if (fullFilename.empty()) {
#ifdef HAVE_BOOST_FILESYSTEM
                // check using the current _filename dir as the current dir
                fullFilename = ResolveURI(scheme, path, boost::filesystem::path(_filename).parent_path().string(), GetOpenRAVESchemeAliases());
#endif
                if (fullFilename.empty()) {
                    RAVELOG_ERROR_FORMAT("env=%d, failed to resolve a filename from env referenceUri='%s'", _penv->GetId()%pReferenceUri);
                    return false;
                }
            }

            boost::shared_ptr<const rapidjson::Document> prReferenceEnvInfo = _GetDocumentFromFilename(fullFilename, alloc);
            if (!prReferenceEnvInfo || !prReferenceEnvInfo->IsObject() ) {
                RAVELOG_WARN_FORMAT("env=%d, failed to load referenced body from filename '%s'", _penv->GetId()%fullFilename);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("env=%d, failed to load referenced body from referenceUri='%s'", _penv->GetId()%pReferenceUri, ORE_InvalidURI);
                }
                return false;
            }

            // if parent rEnvInfo is going to overwrite bodies in the environment of the referenceUri, do not deserialize them. otherwise, they would be merged instead of overwritten.
            rapidjson::Value::ConstMemberIterator itEnvBodies = rEnvInfo.FindMember("bodies");
            if( itEnvBodies != rEnvInfo.MemberEnd() && itEnvBodies->value.IsArray() && itEnvBodies->value.Size() > 0 ) {
                if( prReferenceEnvInfo->HasMember("bodies") && (*prReferenceEnvInfo)["bodies"].Size() > 0 ) {
                    boost::shared_ptr<rapidjson::Document> prFilteredReferenceEnvInfo = boost::make_shared<rapidjson::Document>(&alloc);
                    prFilteredReferenceEnvInfo->CopyFrom(*prReferenceEnvInfo, alloc);
                    const rapidjson::Value& rEnvBodies = itEnvBodies->value;
                    rapidjson::Value& rRefBodies = (*prFilteredReferenceEnvInfo)["bodies"];
                    BOOST_ASSERT(rRefBodies.IsArray());
                    for (rapidjson::Value::ConstValueIterator itEnvBody = rEnvBodies.Begin(); itEnvBody != rEnvBodies.End(); ++itEnvBody) {
                        BOOST_ASSERT((*itEnvBody).IsObject());
                        rapidjson::Value::ValueIterator itIdMatchRefBody = rRefBodies.End();
                        rapidjson::Value::ValueIterator itNameMatchRefBody = rRefBodies.End();
                        const char* envBodyId = "";
                        const char* envBodyName = "";

                        rapidjson::Value::ConstMemberIterator itEnvBodyId = itEnvBody->FindMember("id");
                        if( itEnvBodyId != itEnvBody->MemberEnd() && itEnvBodyId->value.IsString() && itEnvBodyId->value.GetStringLength() > 0 ) {
                            envBodyId = itEnvBodyId->value.GetString();
                            const size_t envBodyIdLen = itEnvBodyId->value.GetStringLength();
                            for (rapidjson::Value::ValueIterator itRefBody = rRefBodies.Begin(); itRefBody != rRefBodies.End(); ++itRefBody) {
                                rapidjson::Value::ConstMemberIterator itRefBodyId = itRefBody->FindMember("id");
                                if( itRefBodyId == itRefBody->MemberEnd() || itRefBodyId->value.GetStringLength() == 0 ) {
                                    continue;
                                }
                                const char* refBodyId = itRefBodyId->value.GetString();
                                const size_t refBodyIdLen = itRefBodyId->value.GetStringLength();
                                if( envBodyIdLen == refBodyIdLen && strncmp(envBodyId, refBodyId, envBodyIdLen) == 0 ) {
                                    itIdMatchRefBody = itRefBody;
                                    break;
                                }
                            }
                        }
                        rapidjson::Value::ConstMemberIterator itEnvBodyName = itEnvBody->FindMember("name");
                        if( itEnvBodyName != itEnvBody->MemberEnd() && itEnvBodyName->value.GetStringLength() > 0 ) {
                            envBodyName = itEnvBodyName->value.GetString();
                            const size_t envBodyNameLen = itEnvBodyName->value.GetStringLength();
                            for (rapidjson::Value::ValueIterator itRefBody = rRefBodies.Begin(); itRefBody != rRefBodies.End(); ++itRefBody) {
                                rapidjson::Value::ConstMemberIterator itRefBodyName = itRefBody->FindMember("name");
                                if( itRefBodyName == itRefBody->MemberEnd() || itRefBodyName->value.GetStringLength() == 0 ) {
                                    continue;
                                }
                                const char* refBodyName = itRefBodyName->value.GetString();
                                const size_t refBodyNameLen = itRefBodyName->value.GetStringLength();
                                if( envBodyNameLen == refBodyNameLen && strncmp(envBodyName, refBodyName, envBodyNameLen) == 0 ) {
                                    itNameMatchRefBody = itRefBody;
                                    break;
                                }
                            }
                        }
                        if( itNameMatchRefBody != rRefBodies.End() ) {
                            if( itNameMatchRefBody == itIdMatchRefBody ) {
                                RAVELOG_VERBOSE_FORMAT("Found body with the same name='%s' and id='%s' in referenceUri '%s'. Excluding it from deserialization.", envBodyName % envBodyId % pReferenceUri);
                                rRefBodies.Erase(itNameMatchRefBody);
                            }
                            else if( itIdMatchRefBody == rRefBodies.End() ) {
                                RAVELOG_VERBOSE_FORMAT("Found body with the same name='%s' but different id='%s' in referenceUri '%s'. Excluding it from deserialization.", envBodyName % envBodyId % pReferenceUri);
                                rRefBodies.Erase(itNameMatchRefBody);
                            }
                            else {
                                BOOST_ASSERT((*itIdMatchRefBody).IsObject());
                                const char* idMatchRefBodyId = (*itIdMatchRefBody)["id"].GetString();
                                const char* idMatchRefBodyName = "";
                                rapidjson::Value::ConstMemberIterator itIdMatchRefBodyName = itIdMatchRefBody->FindMember("name");
                                if( itIdMatchRefBodyName != itIdMatchRefBody->MemberEnd() ) {
                                    idMatchRefBodyName = itIdMatchRefBodyName->value.GetString();
                                }
                                throw OPENRAVE_EXCEPTION_FORMAT("Found body with the same name='%s' in referenceUri '%s' but there is body with the same id='%s' with different name='%s'. Cannot replace the body.", envBodyName % pReferenceUri % idMatchRefBodyId % idMatchRefBodyName, ORE_BodyIdConflict);
                            }
                        }
                        else if( itIdMatchRefBody != rRefBodies.End() )  {
                            RAVELOG_VERBOSE_FORMAT("Found body with the same id='%s' but different name='%s' in referenceUri '%s'. Excluding it from deserialization.", envBodyId % envBodyName % pReferenceUri);
                            rRefBodies.Erase(itIdMatchRefBody);
                        }
                    }
                    prReferenceEnvInfo = prFilteredReferenceEnvInfo;
                }
            }
            RAVELOG_VERBOSE_FORMAT("resolved fullFilename=%s", fullFilename);

            if( updateMode == UFIM_OnlySpecifiedBodiesExact ) {
                _ExtractSpecifiedBodies(envInfo, *prReferenceEnvInfo, vRemovedBodiesExtra);
            }

            if( prReferenceEnvInfo->IsObject() ) {
                _ProcessEnvInfoBodies(envInfo, *prReferenceEnvInfo, alloc, pReferenceUri, fullFilename, mapProcessedConnectedBodyUris);
            }
        }
        else if( !!pReferenceUri[0] ) {
            if( _bMustResolveEnvironmentURI ) {
                throw OPENRAVE_EXCEPTION_FORMAT("Failed to load env referenceUri='%s' from file '%s'", pReferenceUri%_filename, ORE_InvalidURI);
            }

            RAVELOG_ERROR_FORMAT("Failed to load env referenceUri='%s' from file '%s'", pReferenceUri%_filename);
        }

        _ProcessEnvInfoBodies(envInfo, rEnvInfo, alloc, _uri.c_str(), _filename, mapProcessedConnectedBodyUris);

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
            if ( _excludeBodyIds.find(pKinBodyInfo->_id) != _excludeBodyIds.end() ) {
                itBodyInfo = envInfo._vBodyInfos.erase(itBodyInfo);
                continue;
            }

            ++itBodyInfo;
        }

        // have to remove any duplicate names, prioritize ones that have higher index
        {
            std::unordered_set<OpenRAVE::string_view> existingBodyNames;
            ssize_t validBodyCount = envInfo._vBodyInfos.size();
            for (ssize_t iBody = envInfo._vBodyInfos.size() - 1; iBody >= 0; iBody--) {
                const std::string& bodyName = envInfo._vBodyInfos[iBody]->_name;

                // If this is the first time we've seen this body, add the name to the existing name set and continue
                if (existingBodyNames.find(bodyName) == existingBodyNames.end()) {
                    existingBodyNames.emplace(bodyName);
                    continue;
                }

                // If this body duplicates a body with a higher index, remove it by swapping it to the end of our info list and decrementing the valid body count.
                // This saves us having to shuffle all of the pointers down each time we call ::erase().
                RAVELOG_WARN_FORMAT("env=%d, remove redundant entry %d with body name '%s'", _penv->GetId()%iBody%bodyName);
                std::swap(envInfo._vBodyInfos[iBody], envInfo._vBodyInfos[--validBodyCount]);
            }

            // Shrink our set of bodies to cut off the duplicate bodies, which are all past the validBodyCount index.
            envInfo._vBodyInfos.resize(validBodyCount);
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
        vRemovedBodies.insert(vRemovedBodies.end(), vRemovedBodiesExtra.begin(), vRemovedBodiesExtra.end());
        RAVELOG_DEBUG_FORMAT("env=%d, loaded %d bodies in %u[us]", _penv->GetId()%envInfo._vBodyInfos.size()%(utils::GetMonotonicTime()-starttimeus));
        return true;
    }

    bool ExtractFirst(const rapidjson::Value& rEnvInfo, KinBodyPtr& ppbody, rapidjson::Document::AllocatorType& alloc)
    {
        _contextdesc = str(boost::format("env=%s, ExtractFirst, ")%_penv->GetNameId());

        // If remote URL is provided, start the process to download everything and load it into the json map
        if (IsDownloadingFromRemote()) {
#if OPENRAVE_CURL
            JSONDownloaderScope jsonDownload(_contextdesc, *_pDownloader, alloc, !(_deserializeOptions & IDO_IgnoreReferenceUri));
            jsonDownload.QueueDownloadReferenceURIs(rEnvInfo);
            if( !jsonDownload.WaitForDownloads(_bMustResolveEnvironmentURI, _downloadTimeoutUS) ) {
                //return false;
            }
#endif
        }

        // extract the first articulated system found.
        dReal fUnitScale = _GetUnitScale(rEnvInfo, 1.0);
        if (rEnvInfo.HasMember("bodies") && (rEnvInfo)["bodies"].IsArray()) {
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string> mapProcessedConnectedBodyUris;
            for (rapidjson::Value::ConstValueIterator itr = (rEnvInfo)["bodies"].Begin(); itr != (rEnvInfo)["bodies"].End(); ++itr) {
                return _Extract(*itr, ppbody, rEnvInfo, fUnitScale, alloc, mapProcessedConnectedBodyUris);
            }
        }
        return false;
    }

    bool ExtractOne(const rapidjson::Value& rEnvInfo, KinBodyPtr& ppbody, const string& uri, rapidjson::Document::AllocatorType& alloc)
    {
        std::string scheme, path, fragment;
        ParseURI(uri.c_str(), scheme, path, fragment);
        if (fragment == "") {
            return ExtractFirst(rEnvInfo, ppbody, alloc);
        }

        _contextdesc = str(boost::format("env=%s, ExtractOne, ")%_penv->GetNameId());

        // If remote URL is provided, start the process to download everything and load it into the json map
        if (IsDownloadingFromRemote()) {
#if OPENRAVE_CURL
            JSONDownloaderScope jsonDownload(_contextdesc, *_pDownloader, alloc, !(_deserializeOptions & IDO_IgnoreReferenceUri));
            jsonDownload.QueueDownloadReferenceURIs(rEnvInfo);
            if( !jsonDownload.WaitForDownloads(_bMustResolveEnvironmentURI, _downloadTimeoutUS) ) {
                //return false;
            }
#endif
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

    void SetURI(const std::string& uri)
    {
        if (uri.empty()) {
            _uri.clear();
            return;
        }
        std::string scheme, path, fragment;
        ParseURI(uri.c_str(), scheme, path, fragment);
        if (!scheme.empty() && !path.empty()) {
            _uri = uri;
            return;
        }
        RAVELOG_WARN_FORMAT("SetURI ignore invalid uri '%s'", uri);
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

    bool IsDownloadingFromRemote() const
    {
#if OPENRAVE_CURL
        return !!_pDownloader;
#else
        return false;
#endif
    }

    /// \brief open and cache a json document remotly
    void OpenRemoteDocument(const std::string& uri, rapidjson::Document& doc)
    {
#if OPENRAVE_CURL
        _contextdesc = str(boost::format("env=%s, OpenRemoteDocument, ")%_penv->GetNameId());
        // download only one document, do not recurse
        JSONDownloaderScope jsonDownload(_contextdesc, *_pDownloader, doc.GetAllocator(), false);
        jsonDownload.Download(uri.c_str(), doc, true, _downloadTimeoutUS);
#else
        throw OPENRAVE_EXCEPTION_FORMAT("Do not support downloading remote document '%s'.", uri, ORE_NotImplemented);
#endif
    }

protected:

    /// \brief returns true if the referenceUri is a valid URI that can be loaded
    bool _IsExpandableReferenceUri(const char* pReferenceUri) const
    {
        if (_deserializeOptions & IDO_IgnoreReferenceUri) {
            return false;
        }
        if (!pReferenceUri || !pReferenceUri[0]) {
            return false;
        }
        std::string scheme, path, fragment;
        ParseURI(pReferenceUri, scheme, path, fragment);
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
        else if (!IsDownloadingFromRemote()) {
            boost::shared_ptr<rapidjson::Document> newDoc;
            if (StringEndsWith(fullFilename, ".json")) {
                newDoc.reset(new rapidjson::Document(&alloc));
                OpenRapidJsonDocument(fullFilename, *newDoc);
            }
            else if (StringEndsWith(fullFilename, ".msgpack")) {
                newDoc.reset(new rapidjson::Document(&alloc));
                OpenMsgPackDocument(fullFilename, *newDoc);
            }
            else if (StringEndsWith(fullFilename, ".json.gpg")) {
                newDoc.reset(new rapidjson::Document(&alloc));
                OpenEncryptedJSONDocument(fullFilename, *newDoc);
            }
            else if (StringEndsWith(fullFilename, ".msgpack.gpg")) {
                newDoc.reset(new rapidjson::Document(&alloc));
                OpenEncryptedMsgPackDocument(fullFilename, *newDoc);
            }
            if (!!newDoc) {
                doc = newDoc;
                _rapidJSONDocuments[fullFilename] = doc;
            }
        }
        return doc;
    }

    void _ProcessEnvInfoBodies(EnvironmentBase::EnvironmentBaseInfo& envInfo, const rapidjson::Value& rEnvInfo, rapidjson::Document::AllocatorType& alloc, const char* pCurrentUri, const std::string& currentFilename, std::map<RobotBase::ConnectedBodyInfoPtr, std::string>& mapProcessedConnectedBodyUris)
    {
        dReal fUnitScale = _GetUnitScale(rEnvInfo, 1.0);
        std::vector<int> vInputToBodyInfoMapping;
        BOOST_ASSERT(rEnvInfo.IsObject());
        rapidjson::Value::ConstMemberIterator itEnvBodies = rEnvInfo.FindMember("bodies");
        if( itEnvBodies != rEnvInfo.MemberEnd() && itEnvBodies->value.IsArray()) {
            const rapidjson::Value& rEnvBodies = itEnvBodies->value;
            vInputToBodyInfoMapping.resize(rEnvBodies.Size(),-1); // -1, no mapping by default

            for(int iInputBodyIndex = 0; iInputBodyIndex < (int)rEnvBodies.Size(); ++iInputBodyIndex) {
                const rapidjson::Value& rBodyInfo = rEnvBodies[iInputBodyIndex];
                const std::string bodyId = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "id", "");
                const std::string bodyName = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "name", "");
                const char* pReferenceUri = orjson::GetCStringJsonValueByKey(rBodyInfo, "referenceUri", "");
                if (_IsExpandableReferenceUri(pReferenceUri)) {
                    std::set<std::string> circularReference;
                    RAVELOG_VERBOSE_FORMAT("env=%s, processing reference uri '%s' from iInputBodyIndex%d", _penv->GetNameId()%pReferenceUri%iInputBodyIndex);
                    int insertIndex = _ExpandRapidJSON(envInfo, bodyId, bodyName, rEnvInfo, pReferenceUri, circularReference, fUnitScale, alloc, _filename);
                    if( insertIndex < 0 ) {
                        RAVELOG_WARN_FORMAT("failed to load referenced body from uri '%s' inside file '%s'", pReferenceUri%_filename);
                        if (_bMustResolveURI) {
                            throw OPENRAVE_EXCEPTION_FORMAT("failed to load referenced body from referenceUri='%s'", pReferenceUri, ORE_InvalidURI);
                        }
                    }
                    else {
                        vInputToBodyInfoMapping.at(iInputBodyIndex) = insertIndex;
                    }
                }
                else if( !!pReferenceUri[0] ) {
                    if (_bMustResolveURI) {
                        throw OPENRAVE_EXCEPTION_FORMAT("body '%s' has invalid referenceUri='%s", bodyId%pReferenceUri, ORE_InvalidURI);
                    }

                    RAVELOG_WARN_FORMAT("env=%d, body '%s' has invalid referenceUri='%s'", _penv->GetId()%bodyId%pReferenceUri);
                }
            }
        }

        RAVELOG_VERBOSE_FORMAT("env=%s, deserializing '%s'", _penv->GetNameId()%orjson::DumpJson(rEnvInfo));
        envInfo.DeserializeJSONWithMapping(rEnvInfo, fUnitScale, _deserializeOptions, vInputToBodyInfoMapping);
        FOREACH(itBodyInfo, envInfo._vBodyInfos) {
            KinBody::KinBodyInfoPtr& pKinBodyInfo = *itBodyInfo;
            // ensure uri is set
            if (pKinBodyInfo->_uri.empty() && !pKinBodyInfo->_id.empty() ) {
                // only set the URI if the current uri or current filename are not empty. Otherwise will get a fragment "#???", which cannot be loaded
                pKinBodyInfo->_uri = CanonicalizeURI("#" + pKinBodyInfo->_id, pCurrentUri, currentFilename);
            }
            if (pKinBodyInfo->_uri.empty() ) {
                pKinBodyInfo->_uri = _uri;
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
    int _ExpandRapidJSON(EnvironmentBase::EnvironmentBaseInfo& envInfo, const std::string& originBodyId, const std::string& originBodyName, const rapidjson::Value& rEnvInfo, const char* pReferenceUri, std::set<std::string>& circularReference, dReal fUnitScale, rapidjson::Document::AllocatorType& alloc, const std::string& currentFilename) {
        if (circularReference.find(pReferenceUri) != circularReference.end()) {
            RAVELOG_ERROR_FORMAT("failed to load scene, circular reference to uri '%s' found on originBodyId '%s', originBodyName '%s'", pReferenceUri%originBodyId%originBodyName);
            return -1;
        }
        RAVELOG_DEBUG_FORMAT("env=%s, adding '%s' for tracking circular reference, so far %d uris tracked. Scope is '%s'", _penv->GetNameId()%pReferenceUri%circularReference.size()%currentFilename);
        circularReference.insert(pReferenceUri);

        BOOST_ASSERT(rEnvInfo.IsObject());
        dReal fRefUnitScale = fUnitScale;  // unit scale for rRefKinBodyInfo
        rapidjson::Value rRefKinBodyInfo; // holds the read data from pReferenceUri

        // parse the uri
        std::string scheme, path, fragment;
        ParseURI(pReferenceUri, scheme, path, fragment);


        int insertIndex = -1;

        // deal with uri that has just #fragment
        if(scheme.empty() && path.empty() && !fragment.empty()) {
            // reference to itself
            bool bFoundBody = false;
            rapidjson::Value::ConstMemberIterator itEnvBodies = rEnvInfo.FindMember("bodies");
            if( itEnvBodies != rEnvInfo.MemberEnd() && itEnvBodies->value.IsArray() && itEnvBodies->value.Size() > 0 ) {
                const rapidjson::Value& rEnvBodies = itEnvBodies->value;
                for(rapidjson::Value::ConstValueIterator it = rEnvBodies.Begin(); it != rEnvBodies.End(); it++) {
                    std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                    if (id == fragment) {
                        rRefKinBodyInfo.CopyFrom(*it, alloc);
                        bFoundBody = true;
                        break;
                    }
                }
            }
            if (!bFoundBody) {
                RAVELOG_ERROR_FORMAT("env=%s, failed to find body using referenceUri '%s' in originBodyId '%s', originBodyName '%s'", _penv->GetNameId()%pReferenceUri%originBodyId%originBodyName);
                return -1;
            }

            const char* pNextReferenceUri = orjson::GetCStringJsonValueByKey(rRefKinBodyInfo, "referenceUri", "");
            if (_IsExpandableReferenceUri(pNextReferenceUri)) {
                RAVELOG_VERBOSE_FORMAT("env=%s, processing next reference uri '%s'", _penv->GetNameId()%pNextReferenceUri);
                insertIndex = _ExpandRapidJSON(envInfo, originBodyId, originBodyName, rEnvInfo, pNextReferenceUri, circularReference, fUnitScale, alloc, currentFilename);
                // regardless of insertIndex, should fall through so can process rEnvInfo
            }
            else if( !!pNextReferenceUri[0] ) {
                RAVELOG_ERROR_FORMAT("nextReferenceUri='%s' is not a valid URI. Scope is '%s'", pNextReferenceUri%currentFilename);
            }
        }
        // deal with uri with scheme:/path#fragment
        else if (!scheme.empty() && !path.empty()) {
            // replace .dae to .json or .msgpack, depends on orignal document file defaultSuffix
            if( _ReplaceFilenameSuffix(path, ".dae", _defaultSuffix) ) {
                RAVELOG_WARN_FORMAT("env=%d, filename had '.dae' suffix, so changed to %s", _penv->GetId()%path);
            }
            std::string fullFilename;

            if (IsDownloadingFromRemote()) {
                // here we use fullFilename as key to look up in _rapidJSONDocuments
                if (scheme == "file") {
                    fullFilename = path;
                } else {
                    fullFilename = scheme + ":" + path;
                }
            }
            else {
                fullFilename = ResolveURI(scheme, path, std::string(), GetOpenRAVESchemeAliases());
            }
            if (fullFilename.empty()) {

#ifdef HAVE_BOOST_FILESYSTEM
                fullFilename = ResolveURI(scheme, path, boost::filesystem::path(currentFilename).parent_path().string(), GetOpenRAVESchemeAliases());
#endif
                if (fullFilename.empty()) {
                    RAVELOG_ERROR_FORMAT("env=%d, failed to resolve referenceUri '%s' into a file. Coming from bodyId='%s', bodyName='%s' in file '%s'", _penv->GetId()%pReferenceUri%originBodyId%originBodyName%currentFilename);
                    if (_bMustResolveURI) {
                        throw OPENRAVE_EXCEPTION_FORMAT("Failed to resolve referenceUri='%s' in body definition '%s' from file '%s'", pReferenceUri%originBodyId%currentFilename, ORE_InvalidURI);
                    }

                    return -1;
                }
            }

            uint64_t beforeOpenStampUS = utils::GetMonotonicTime();

            boost::shared_ptr<const rapidjson::Document> pReferenceScene = _GetDocumentFromFilename(fullFilename, alloc);
            if (!pReferenceScene || !pReferenceScene->IsObject() ) {
                RAVELOG_ERROR_FORMAT("referenced document from file '%s' cannot be loaded.", fullFilename);
                return -1;
            }

            rapidjson::Value::ConstMemberIterator itReferenceBodies = pReferenceScene->FindMember("bodies");
            if ( itReferenceBodies == pReferenceScene->MemberEnd() || !itReferenceBodies->value.IsArray() ) {
                RAVELOG_ERROR_FORMAT("referenced document from file '%s' has no valid 'bodies' field: %s", fullFilename%orjson::DumpJson(*pReferenceScene));
                return -1;
            }
            fRefUnitScale = _GetUnitScale(*pReferenceScene, 1.0); // for now default has to be meters... fUnitScale);

            bool bFoundBody = false;
            for(rapidjson::Value::ConstValueIterator it = itReferenceBodies->value.Begin(); it != itReferenceBodies->value.End(); it++) {
                std::string id = orjson::GetJsonValueByKey<std::string>(*it, "id", "");
                if (id == fragment || fragment.empty()) {
                    rRefKinBodyInfo.CopyFrom(*it, alloc);
                    bFoundBody = true;
                    break;
                }
            }
            if (!bFoundBody) {
                RAVELOG_ERROR_FORMAT("failed to find body using referenceUri '%s' in body id=%s, name=%s", pReferenceUri%originBodyId%originBodyName);
                return -1;
            }

            const char* pNextReferenceUri = orjson::GetCStringJsonValueByKey(rRefKinBodyInfo, "referenceUri", "");

            if (_IsExpandableReferenceUri(pNextReferenceUri)) {
                RAVELOG_DEBUG_FORMAT("env=%d, opened file '%s', found body from fragment='%s', and now processing its referenceUri='%s, took %u[us]'", _penv->GetId()%fullFilename%fragment%pNextReferenceUri%(utils::GetMonotonicTime()-beforeOpenStampUS));
                insertIndex = _ExpandRapidJSON(envInfo, originBodyId, originBodyName, *pReferenceScene, pNextReferenceUri, circularReference, fUnitScale, alloc, fullFilename);
                // regardless of insertIndex, should fall through so can process rEnvInfo
            }
            else {
                RAVELOG_DEBUG_FORMAT("env=%d, opened file '%s', found body from fragment='%s', took %u[us]", _penv->GetId()%fullFilename%fragment%(utils::GetMonotonicTime()-beforeOpenStampUS));
            }
        }
        else {
            RAVELOG_WARN_FORMAT("ignoring invalid referenceUri '%s' in body id=%s, name=%s", pReferenceUri%originBodyId%originBodyName);
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
                    insertIndex = ibody;
                    break;
                }
            }
        }

        KinBody::KinBodyInfoPtr pNewKinBodyInfo;
        if( insertIndex >= 0 ) {
            RAVELOG_DEBUG_FORMAT("env=%d, loaded referenced body '%s' with id='%s' from uri '%s'. Scope is '%s'", _penv->GetId()%envInfo._vBodyInfos.at(insertIndex)->_name%originBodyId%pReferenceUri%currentFilename);
            pNewKinBodyInfo = envInfo._vBodyInfos[insertIndex];

            const bool isPartial = orjson::GetJsonValueByKey<bool>(rRefKinBodyInfo, "__isPartial__", true);
            RAVELOG_VERBOSE_FORMAT("Deserializing rRefKinBodyInfo=%s into insertIndex=%d, isPartial=%d", orjson::DumpJson(rRefKinBodyInfo)%insertIndex%isPartial);
            if (!isPartial) {
                pNewKinBodyInfo->Reset();
            }
        } else {
            bool isNewRobot = orjson::GetJsonValueByKey<bool>(rRefKinBodyInfo, "isRobot", false);
            if( isNewRobot ) {
                pNewKinBodyInfo.reset(new RobotBase::RobotBaseInfo());
            }
            else {
                pNewKinBodyInfo.reset(new KinBody::KinBodyInfo());
            }
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

        // insert the new body info now if it hasn't been inserted
        if( insertIndex < 0 ) {
            // might get overwritten later, so ok if name is empty
            insertIndex = envInfo._vBodyInfos.size();
            envInfo._vBodyInfos.push_back(pNewKinBodyInfo);
            RAVELOG_DEBUG_FORMAT("env=%d, could not find existing body with id='%s', name='%s', so inserting it. Scope is '%s'", _penv->GetId()%originBodyId%pNewKinBodyInfo->_name%currentFilename);
        }
        return insertIndex;
    }

    inline dReal _GetUnitScale(const rapidjson::Value& doc, dReal defaultScale)
    {
        if (doc.HasMember("unitInfo")) {
            UnitInfo unitInfo;
            orjson::LoadJsonValueByKey(doc, "unitInfo", unitInfo);
            return 1.0 / GetLengthUnitStandardValue<dReal>(unitInfo.lengthUnit) * _fGlobalScale * _fGeomScale;
        }
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
        std::string bodyId = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "id", "");
        std::string bodyName = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "name", "");
        const char* pReferenceUri = orjson::GetCStringJsonValueByKey(rBodyInfo, "referenceUri", "");

        EnvironmentBase::EnvironmentBaseInfo envInfo; // dummy for reference uris
        int insertIndex = -1;
        if (_IsExpandableReferenceUri(pReferenceUri)) {
            if (IsDownloadingFromRemote()) {
#if OPENRAVE_CURL
                JSONDownloaderScope jsonDownload(_contextdesc, *_pDownloader, alloc, !(_deserializeOptions & IDO_IgnoreReferenceUri));
                jsonDownload.QueueDownloadURI(pReferenceUri);
                if( !jsonDownload.WaitForDownloads(_bMustResolveURI, _downloadTimeoutUS) ) {
                    RAVELOG_WARN_FORMAT("env=%s, body '%s' has invalid referenceUri '%s'", _penv->GetNameId()%bodyId%pReferenceUri);
                }
#endif
            }
            std::set<std::string> circularReference; // dummy
            RAVELOG_VERBOSE_FORMAT("env=%s, processing reference uri '%s'", _penv->GetNameId()%pReferenceUri);
            insertIndex = _ExpandRapidJSON(envInfo, bodyId, bodyName, rEnvInfo, pReferenceUri, circularReference, fUnitScale, alloc, _filename);
            if( insertIndex < 0 ) {
                RAVELOG_WARN_FORMAT("env=%s, failed to load referenced body from uri '%s' inside file '%s'", _penv->GetNameId()%pReferenceUri%_filename);
                if (_bMustResolveURI) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to load referenced body from referenceUri='%s'", pReferenceUri, ORE_InvalidURI);
                }

            }
        }
        else if( !!pReferenceUri[0] ) {
            if (_bMustResolveURI) {
                throw OPENRAVE_EXCEPTION_FORMAT("body '%s' has invalid referenceUri='%s", bodyId%pReferenceUri, ORE_InvalidURI);
            }

            RAVELOG_WARN_FORMAT("env=%d, body '%s' has invalid referenceUri '%s'", _penv->GetId()%bodyId%pReferenceUri);
        }

        KinBody::KinBodyInfoPtr pKinBodyInfo;
        RobotBase::RobotBaseInfoPtr pRobotBaseInfo;
        if( insertIndex >= 0 ) {
            pKinBodyInfo = envInfo._vBodyInfos.at(insertIndex);
            bool isRobot = orjson::GetJsonValueByKey<bool>(rBodyInfo, "isRobot", pKinBodyInfo->_isRobot); // fallback to extract isRobot flag from the reference body info
            if( isRobot ) {
                pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pKinBodyInfo);
            }
        }
        else {
            bool isRobot = orjson::GetJsonValueByKey<bool>(rBodyInfo, "isRobot");
            if( isRobot ) {
                pRobotBaseInfo.reset(new RobotBase::RobotBaseInfo());
                pKinBodyInfo = pRobotBaseInfo;
            }
            else {
                pKinBodyInfo.reset(new KinBody::KinBodyInfo());
            }
        }
        pKinBodyInfo->DeserializeJSON(rBodyInfo, fUnitScale, _deserializeOptions);

        if (pKinBodyInfo->_uri.empty() && !pKinBodyInfo->_id.empty() ) {
            // only set the URI if the current uri or current filename are not empty. Otherwise will get a fragment "#???", which cannot be loaded
            pKinBodyInfo->_uri = CanonicalizeURI("#" + pKinBodyInfo->_id, _uri, _filename);
        }
        if (pKinBodyInfo->_uri.empty() ) {
            pKinBodyInfo->_uri = _uri;
        }

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
        {
            const char *const modifiedAt = orjson::GetCStringJsonValueByKey(rEnvInfo, "modifiedAt");
            if ( modifiedAt != nullptr ) {
                pBody->SetLastModifiedAtUS(ConvertIsoFormatDateTimeToLinuxTimeUS(modifiedAt));
            }
        }
        {
            rapidjson::Value::ConstMemberIterator itRevisionId = rEnvInfo.FindMember("revisionId");
            if( itRevisionId != rEnvInfo.MemberEnd() ) {
                int64_t revisionId = 0;
                orjson::LoadJsonValue(itRevisionId->value, revisionId);
                pBody->SetRevisionId(revisionId);
            }
        }
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

        if (IsDownloadingFromRemote()) {
#if OPENRAVE_CURL
            JSONDownloaderScope jsonDownload(_contextdesc, *_pDownloader, alloc, !(_deserializeOptions & IDO_IgnoreReferenceUri));
            FOREACH(itConnected, robotInfo._vConnectedBodyInfos) {
                RobotBase::ConnectedBodyInfoPtr& pConnected = *itConnected;
                if (_IsExpandableReferenceUri(pConnected->_uri.c_str())) {
                    jsonDownload.QueueDownloadURI(pConnected->_uri.c_str());
                }
            }
            if( !jsonDownload.WaitForDownloads(_bMustResolveURI, _downloadTimeoutUS) ) {
                RAVELOG_WARN_FORMAT("env=%s, robot '%s' has invalid uris", _penv->GetNameId()%robotInfo._name);
                //return;
            }
#endif
        }

        FOREACH(itConnected, robotInfo._vConnectedBodyInfos) {
            RobotBase::ConnectedBodyInfoPtr& pConnected = *itConnected;
            std::map<RobotBase::ConnectedBodyInfoPtr, std::string>::iterator itProcessedEntry = mapProcessedConnectedBodyUris.find(pConnected);
            if( itProcessedEntry != mapProcessedConnectedBodyUris.end() && itProcessedEntry->second == pConnected->_uri ) {
                continue;
            }

            if( !_IsExpandableReferenceUri(pConnected->_uri.c_str()) ) {
                continue;
            }

            std::set<std::string> circularReference;
            EnvironmentBase::EnvironmentBaseInfo envInfo;
            int insertIndex = _ExpandRapidJSON(envInfo, "__connectedBody__", "", rEnvInfo, pConnected->_uri.c_str(), circularReference, fUnitScale, alloc, _filename);
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
    void _ExtractSpecifiedBodies(EnvironmentBase::EnvironmentBaseInfo& envInfo, const rapidjson::Value& rEnvInfo, std::vector<KinBodyPtr>& vRemovedBodies)
    {
        int numOriginalBodyInfos = (int)envInfo._vBodyInfos.size();

        // extract only the bodies used in rEnvInfo
        BOOST_ASSERT(rEnvInfo.IsObject());
        rapidjson::Value::ConstMemberIterator itBodies = rEnvInfo.FindMember("bodies");
        if( itBodies != rEnvInfo.MemberEnd() && itBodies->value.IsArray() ) {
            const rapidjson::Value& rBodies = itBodies->value;
            std::string id; // cache
            envInfo._vBodyInfos.reserve(rBodies.Size()); // reserve at least this much
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
                            vRemovedBodies.push_back(pbody);
                        }
                        else {
                            // no need to add an entry if partial is false since all the data will be in rBodyInfo
                            bool isPartial = orjson::GetJsonValueByKey<bool>(rBodyInfo, "__isPartial__", true);
                            if( isPartial ) {
                                KinBody::KinBodyInfoPtr pKinBodyInfo;
                                if (pbody->IsRobot()) {
                                    RobotBase::RobotBaseInfoPtr pRobotInfo(new RobotBase::RobotBaseInfo());
                                    RaveInterfaceCast<RobotBase>(pbody)->ExtractInfo(*pRobotInfo, EIO_Everything);
                                    pKinBodyInfo = pRobotInfo;
                                }
                                else {
                                    pKinBodyInfo.reset(new KinBody::KinBodyInfo());
                                    pbody->ExtractInfo(*pKinBodyInfo, EIO_Everything);
                                }
                                envInfo._vBodyInfos.push_back(pKinBodyInfo);
                            }
                        }
                    }
                }
            }
        }
    }

    dReal _fGlobalScale = 1.0;
    dReal _fGeomScale = 1.0;
    uint64_t _downloadTimeoutUS = 10000000; ///< download timeout in microseconds
    EnvironmentBasePtr _penv;
    int _deserializeOptions = 0; ///< options used for deserializing
    std::string _filename; ///< original filename used to open reader
    std::string _uri; ///< original uri used to open reader
    std::string _defaultSuffix; ///< defaultSuffix of the main document, either ".json" or ".msgpack"
    std::vector<std::string> _vOpenRAVESchemeAliases;
    std::unordered_set<std::string> _excludeBodyIds; ///< list of body ids to exclude from importing
    std::string _contextdesc; ///< context where download is created
    bool _bMustResolveURI = false; ///< if true, throw exception if object uri does not resolve
    bool _bMustResolveEnvironmentURI = false; ///< if true, throw exception if environment uri does not resolve
    bool _bIgnoreInvalidBodies = false; ///< if true, ignores any invalid bodies

    std::map<std::string, boost::shared_ptr<const rapidjson::Document> > _rapidJSONDocuments; ///< cache for opened rapidjson Documents

#if OPENRAVE_CURL
    JSONDownloaderPtr _pDownloader; ///< downloader for downloading remote files, only non-null if remoteurl is set
#endif
};


bool RaveParseJSON(EnvironmentBasePtr penv, const std::string& uri, const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    reader.SetURI(uri);
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseJSON(EnvironmentBasePtr penv, const std::string& uri, KinBodyPtr& ppbody, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    reader.SetURI(uri);
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseJSON(EnvironmentBasePtr penv, const std::string& uri, RobotBasePtr& pprobot, const rapidjson::Value& doc, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    reader.SetURI(uri);
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
    KinBodyPtr pbody;
    if (RaveParseJSONFile(penv, pbody, filename, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    reader.SetURI(uri);
    rapidjson::Document rEnvInfo(&alloc);
#if OPENRAVE_CURL
    if (reader.IsDownloadingFromRemote()) {
        reader.OpenRemoteDocument(uri, rEnvInfo);
    } else
#endif
    {
        std::string fullFilename = ResolveURI(uri.c_str(), std::string(), reader.GetOpenRAVESchemeAliases());
        if (fullFilename.size() == 0 ) {
            return false;
        }
        OpenRapidJsonDocument(fullFilename, rEnvInfo);
    }
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".json");
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
#if OPENRAVE_CURL
    if (reader.IsDownloadingFromRemote()) {
        reader.OpenRemoteDocument(uri, doc);
    } else
#endif
    {
        std::string fullFilename = ResolveURI(uri.c_str(), std::string(), reader.GetOpenRAVESchemeAliases());
        if (fullFilename.size() == 0 ) {
            return false;
        }
        OpenRapidJsonDocument(fullFilename, doc);
    }
    return reader.ExtractOne(doc, ppbody, uri, alloc);
}

bool RaveParseJSONURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    KinBodyPtr pbody;
    if (RaveParseJSONURI(penv, pbody, uri, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseJSONData(EnvironmentBasePtr penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document rEnvInfo(&alloc);
    orjson::ParseJson(rEnvInfo, data);
    JSONReader reader(atts, penv, ".json");
    reader.SetURI(uri);
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    orjson::ParseJson(doc, data);
    JSONReader reader(atts, penv, ".json");
    reader.SetURI(uri);
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    KinBodyPtr pbody;
    if (RaveParseJSONData(penv, pbody, uri, data, atts, alloc)) {
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
    KinBodyPtr pbody;
    if(RaveParseMsgPackFile(penv, pbody, filename, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseEncryptedJSONFile(EnvironmentBasePtr penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.empty() ) {
        return false;
    }
    rapidjson::Document rEnvInfo(&alloc);
    OpenEncryptedJSONDocument(fullFilename, rEnvInfo);
    JSONReader reader(atts, std::move(penv), ".json.gpg");
    reader.SetFilename(fullFilename);
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseEncryptedJSONFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.empty() ) {
        return false;
    }
    rapidjson::Document doc(&alloc);
    OpenEncryptedJSONDocument(fullFilename, doc);
    JSONReader reader(atts, std::move(penv), ".json.gpg");
    reader.SetFilename(fullFilename);
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseEncryptedJSONFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    KinBodyPtr pbody;
    if(RaveParseEncryptedJSONFile(std::move(penv), pbody, filename, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseEncryptedMsgPackFile(EnvironmentBasePtr penv, const std::string& filename, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.empty() ) {
        return false;
    }
    rapidjson::Document rEnvInfo(&alloc);
    OpenEncryptedMsgPackDocument(fullFilename, rEnvInfo);
    JSONReader reader(atts, std::move(penv), ".msgpack.gpg");
    reader.SetFilename(fullFilename);
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseEncryptedMsgPackFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::string fullFilename = RaveFindLocalFile(filename);
    if (fullFilename.empty() ) {
        return false;
    }
    rapidjson::Document doc(&alloc);
    OpenEncryptedMsgPackDocument(fullFilename, doc);
    JSONReader reader(atts, std::move(penv), ".msgpack.gpg");
    reader.SetFilename(fullFilename);
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseEncryptedMsgPackFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    KinBodyPtr pbody;
    if(RaveParseEncryptedMsgPackFile(std::move(penv), pbody, filename, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".msgpack");
    reader.SetURI(uri);
    rapidjson::Document rEnvInfo(&alloc);
#if OPENRAVE_CURL
    if (reader.IsDownloadingFromRemote()) {
        reader.OpenRemoteDocument(uri, rEnvInfo);
    } else
#endif
    {
        std::string fullFilename = ResolveURI(uri.c_str(), std::string(), reader.GetOpenRAVESchemeAliases());
        if (fullFilename.size() == 0 ) {
            return false;
        }
        OpenMsgPackDocument(fullFilename, rEnvInfo);
    }
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, penv, ".msgpack");
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
#if OPENRAVE_CURL
    if (reader.IsDownloadingFromRemote()) {
        reader.OpenRemoteDocument(uri, doc);
    } else
#endif
    {
        std::string fullFilename = ResolveURI(uri.c_str(), std::string(), reader.GetOpenRAVESchemeAliases());
        if (fullFilename.size() == 0 ) {
            if( IS_DEBUGLEVEL(Level_Debug) ) {
                std::stringstream ssatts;
                for(const std::pair<std::string, std::string>& attribute : atts) {
                    ssatts << "{" << attribute.first << ":" << attribute.second << "}, ";
                }
                RAVELOG_DEBUG_FORMAT("could not resolve uri='%s' into a path using atts=[%s]", uri%ssatts.str());
            }
            return false;
        }
        OpenMsgPackDocument(fullFilename, doc);
    }
    return reader.ExtractOne(doc, ppbody, uri, alloc);
}

bool RaveParseMsgPackURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    KinBodyPtr pbody;
    if (RaveParseMsgPackURI(penv, pbody, uri, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document rEnvInfo(&alloc);
    MsgPack::ParseMsgPack(rEnvInfo, data);
    JSONReader reader(atts, penv, ".msgpack");
    reader.SetURI(uri);
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    rapidjson::Document doc(&alloc);
    MsgPack::ParseMsgPack(doc, data);
    JSONReader reader(atts, penv, ".msgpack");
    reader.SetURI(uri);
    return reader.ExtractFirst(doc, ppbody, alloc);
}

bool RaveParseMsgPackData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    KinBodyPtr pbody;
    if (RaveParseMsgPackData(penv, pbody, uri, data, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}


bool RaveParseEncryptedJSONData(EnvironmentBasePtr penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::istringstream iss(data);
    std::ostringstream oss;
    if (!GpgDecrypt(iss, oss)) {
        return false;
    }
    return RaveParseJSONData(std::move(penv), oss.str(), uri, updateMode, atts, alloc);
}

bool RaveParseEncryptedJSONData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::istringstream iss(data);
    std::ostringstream oss;
    if (!GpgDecrypt(iss, oss)) {
        return false;
    }
    return RaveParseJSONData(std::move(penv), ppbody, uri, oss.str(), atts, alloc);
}

bool RaveParseEncryptedJSONData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::istringstream iss(data);
    std::ostringstream oss;
    if (!GpgDecrypt(iss, oss)) {
        return false;
    }
    return RaveParseJSONData(std::move(penv), pprobot, uri, oss.str(), atts, alloc);
}

bool RaveParseEncryptedMsgPackData(EnvironmentBasePtr penv, const std::string& uri, const std::string& data, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::istringstream iss(data);
    std::ostringstream oss;
    if (!GpgDecrypt(iss, oss)) {
        return false;
    }
    return RaveParseMsgPackData(std::move(penv), uri, oss.str(), updateMode, atts, alloc);
}

bool RaveParseEncryptedMsgPackData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::istringstream iss(data);
    std::ostringstream oss;
    if (!GpgDecrypt(iss, oss)) {
        return false;
    }
    return RaveParseMsgPackData(std::move(penv), ppbody, uri, oss.str(), atts, alloc);
}

bool RaveParseEncryptedMsgPackData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const std::string& data, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    std::istringstream iss(data);
    std::ostringstream oss;
    if (!GpgDecrypt(iss, oss)) {
        return false;
    }
    return RaveParseMsgPackData(std::move(penv), pprobot, uri, oss.str(), atts, alloc);
}

///// Support for PGP-encrypted data

bool RaveParseEncryptedJSONURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, std::move(penv), ".json.gpg");
    reader.SetURI(uri);
    rapidjson::Document rEnvInfo(&alloc);
#if OPENRAVE_CURL
    if (reader.IsDownloadingFromRemote()) {
        reader.OpenRemoteDocument(uri, rEnvInfo);
    } else
#endif
    {
        std::string fullFilename = ResolveURI(uri.c_str(), std::string(), reader.GetOpenRAVESchemeAliases());
        if (fullFilename.empty() ) {
            return false;
        }
        OpenEncryptedJSONDocument(fullFilename, rEnvInfo);
    }
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseEncryptedJSONURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, std::move(penv), ".json.gpg");
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
#if OPENRAVE_CURL
    if (reader.IsDownloadingFromRemote()) {
        reader.OpenRemoteDocument(uri, doc);
    } else
#endif
    {
        std::string fullFilename = ResolveURI(uri.c_str(), std::string(), reader.GetOpenRAVESchemeAliases());
        if (fullFilename.empty() ) {
            return false;
        }
        OpenEncryptedJSONDocument(fullFilename, doc);
    }
    return reader.ExtractOne(doc, ppbody, uri, alloc);
}

bool RaveParseEncryptedJSONURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    KinBodyPtr pbody;
    if (RaveParseEncryptedJSONURI(std::move(penv), pbody, uri, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

bool RaveParseEncryptedMsgPackURI(EnvironmentBasePtr penv, const std::string& uri, UpdateFromInfoMode updateMode, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, std::move(penv), ".msgpack.gpg");
    reader.SetURI(uri);
    rapidjson::Document rEnvInfo(&alloc);
#if OPENRAVE_CURL
    if (reader.IsDownloadingFromRemote()) {
        reader.OpenRemoteDocument(uri, rEnvInfo);
    } else
#endif
    {
        std::string fullFilename = ResolveURI(uri.c_str(), std::string(), reader.GetOpenRAVESchemeAliases());
        if (fullFilename.empty() ) {
            return false;
        }
        OpenEncryptedMsgPackDocument(fullFilename, rEnvInfo);
    }
    std::vector<KinBodyPtr> vCreatedBodies, vModifiedBodies, vRemovedBodies;
    return reader.ExtractAll(rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, alloc);
}

bool RaveParseEncryptedMsgPackURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    JSONReader reader(atts, std::move(penv), ".msgpack.gpg");
    reader.SetURI(uri);
    rapidjson::Document doc(&alloc);
#if OPENRAVE_CURL
    if (reader.IsDownloadingFromRemote()) {
        reader.OpenRemoteDocument(uri, doc);
    } else
#endif
    {
        std::string fullFilename = ResolveURI(uri.c_str(), std::string(), reader.GetOpenRAVESchemeAliases());
        if (fullFilename.empty() ) {
            return false;
        }
        OpenEncryptedMsgPackDocument(fullFilename, doc);
    }
    return reader.ExtractOne(doc, ppbody, uri, alloc);
}

bool RaveParseEncryptedMsgPackURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts, rapidjson::Document::AllocatorType& alloc)
{
    KinBodyPtr pbody;
    if (RaveParseEncryptedMsgPackURI(std::move(penv), pbody, uri, atts, alloc)) {
        pprobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody);
        return true;
    }
    return false;
}

} // namespace OpenRAVE
