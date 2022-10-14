// -*- coding: utf-8 -*-
// Copyright (C) 2022 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "jsondownloader.h"
#include <openrave/openravejson.h>
#include <openrave/openravemsgpack.h>
#include <openrave/openrave.h>
#include <rapidjson/istreamwrapper.h>
#include <string>
#include <fstream>
#include <curl/curl.h> /// Required for remote URI



static bool EndsWith(const std::string& fullString, const std::string& endString) {
    if (fullString.length() >= endString.length()) {
        return fullString.compare(fullString.length() - endString.length(), endString.length(), endString) == 0;
    }
    return false;
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



// Need to remove fragement to check for .json or .msgpack
static std::string GetPath(const std::string& uri)
{
    std::string path = uri;
    size_t hashindex = path.find_last_of('#');

    if (hashindex != std::string::npos) {
        path = path.substr(0, hashindex);
    }
    return path;
}




JSONRemoteHelper::JSONRemoteHelper(std::map<std::string, boost::shared_ptr<const rapidjson::Document> > &rapidJSONMap, std::string &remoteUrl, std::vector<std::string>& schemeVector){
    _rapidJSONDocuments = &rapidJSONMap;
    _remoteUrl = remoteUrl;
    _vOpenRAVESchemeAliases = schemeVector;

    // Sets up curl multi handle to be used
    _curlMultiHandle = curl_multi_init();

}
JSONRemoteHelper::~JSONRemoteHelper(){
    curl_multi_cleanup(_curlMultiHandle);
}


/// @brief Downloads all the remote files, parse them, download other references, then store them in document map
/// @param rEnvInfo top layer
void JSONRemoteHelper::DownloadRecursively(const rapidjson::Value &rEnvInfo)
{
    std::vector<int> vInputToBodyInfoMapping;
    // TODO make function to replace it with remote URL

    // Parsing the top level
    _ParseDocumentForNewURLs(rEnvInfo);
    int stillRunning, msgInQueue;

    // Loop to download then parse in parallel
    do {

        CURLMcode mc = curl_multi_perform(_curlMultiHandle, &stillRunning);
        CURLMsg  *curlmsg = curl_multi_info_read(_curlMultiHandle, &msgInQueue);
        if(!mc && stillRunning) {
            /* wait for activity, timeout or "nothing" */
            mc = curl_multi_poll(_curlMultiHandle, NULL, 0, 1000, NULL);
        }
        if(mc) {
            break;
        }

        while(curlmsg != NULL) {
            if (curlmsg->msg == CURLMSG_DONE) {
                for ( unsigned long i = 0; i <  _curlDataVector.size(); i++) {
                    if (_curlDataVector.at(i).get()->curl == curlmsg->easy_handle) {
                        boost::shared_ptr<rapidjson::Document> document = boost::shared_ptr<rapidjson::Document>(new rapidjson::Document);
                        if (EndsWith(GetPath( _curlDataVector.at(i).get()->uri), ".json")) {
                            rapidjson::ParseResult ok = document.get()->Parse(_curlDataVector.at(i).get()->buffer.GetString());
                            if (!ok) {
                                throw OPENRAVE_EXCEPTION_FORMAT("failed to parse json document \"%s\"", _curlDataVector.at(i).get()->uri, ORE_InvalidArguments);
                            }
                        }
                        else if (EndsWith(GetPath( _curlDataVector.at(i).get()->uri), ".msgpack")) {

                            try {
                                MsgPack::ParseMsgPack(*document, (const void*)_curlDataVector.at(i).get()->buffer.GetString(), _curlDataVector.at(i).get()->buffer.GetSize());
                            }
                            catch(const std::exception& ex) {
                                throw OPENRAVE_EXCEPTION_FORMAT("Failed to parse msgpack format for file '%s': %s", _curlDataVector.at(i).get()->uri%ex.what(), ORE_Failed);
                            }

                        }

                        _PutDocumentIntoRapidJSONMap(_curlDataVector.at(i).get()->uri, document);
                        _ParseDocumentForNewURLs(*document);



                        curl_multi_remove_handle(_curlMultiHandle,_curlDataVector.at(i).get()->curl);
                        _curlDataVector.erase(_curlDataVector.begin() + i);

                        stillRunning++; // This is to make sure the while loop continues
                        i--;
                    }
                }
            }
            curlmsg = curl_multi_info_read(_curlMultiHandle, &msgInQueue);
        }

    } while(stillRunning);
}

/// @brief Downloads all the remote files, parse them, download other references, then store them in document map
/// @param referenceUri if needed to download starting from a string
void JSONRemoteHelper::DownloadRecursively(const std::string &referenceUri)
{
    AddReferenceURIToDownload(referenceUri);
    int stillRunning, msgInQueue;
    // Loop to download then parse in parallel
    do {

        CURLMcode mc = curl_multi_perform(_curlMultiHandle, &stillRunning);
        CURLMsg  *curlmsg = curl_multi_info_read(_curlMultiHandle, &msgInQueue);
        if(!mc && stillRunning) {
            /* wait for activity, timeout or "nothing" */
            mc = curl_multi_poll(_curlMultiHandle, NULL, 0, 1000, NULL);
        }
        if(mc) {
            break;
        }

        while(curlmsg != NULL) {
            if (curlmsg->msg == CURLMSG_DONE) {
                for ( unsigned long i = 0; i <  _curlDataVector.size(); i++) {
                    if (_curlDataVector.at(i).get()->curl == curlmsg->easy_handle) {
                        boost::shared_ptr<rapidjson::Document> document = boost::shared_ptr<rapidjson::Document>(new rapidjson::Document);
                        if (EndsWith(GetPath( _curlDataVector.at(i).get()->uri), ".json")) {
                            rapidjson::ParseResult ok = document.get()->Parse(_curlDataVector.at(i).get()->buffer.GetString());
                            if (!ok) {
                                throw OPENRAVE_EXCEPTION_FORMAT("failed to parse json document \"%s\"", _curlDataVector.at(i).get()->uri, ORE_InvalidArguments);
                            }
                        }
                        else if (EndsWith(GetPath( _curlDataVector.at(i).get()->uri), ".msgpack")) {

                            try {
                                MsgPack::ParseMsgPack(*document, (const void*)_curlDataVector.at(i).get()->buffer.GetString(), _curlDataVector.at(i).get()->buffer.GetSize());
                            }
                            catch(const std::exception& ex) {
                                throw OPENRAVE_EXCEPTION_FORMAT("Failed to parse msgpack format for file '%s': %s", _curlDataVector.at(i).get()->uri%ex.what(), ORE_Failed);
                            }

                        }

                        _PutDocumentIntoRapidJSONMap(_curlDataVector.at(i).get()->uri, document);
                        _ParseDocumentForNewURLs(*document);

                        curl_multi_remove_handle(_curlMultiHandle,_curlDataVector.at(i).get()->curl);
                        _curlDataVector.erase(_curlDataVector.begin() + i);
                        stillRunning++; // This is to make sure the while loop continues
                        i--;
                    }
                }
            }
            curlmsg = curl_multi_info_read(_curlMultiHandle, &msgInQueue);
        }

    } while(stillRunning);
}

void JSONRemoteHelper::DownloadOne(const std::string& currentUri, rapidjson::Document& doc)
{

    std::vector<int> vInputToBodyInfoMapping;
    // TODO make function to replace it with remote URL
    AddReferenceURIToDownload(currentUri);
    int stillRunning = 0;
    int msgInQueue = 0;
    do {

        CURLMcode mc = curl_multi_perform(_curlMultiHandle, &stillRunning);
        CURLMsg  *curlmsg = curl_multi_info_read(_curlMultiHandle, &msgInQueue);
        if(!mc && stillRunning) {
            /* wait for activity, timeout or "nothing" */
            mc = curl_multi_poll(_curlMultiHandle, NULL, 0, 1000, NULL);
        }
        if(mc) {
            break;
        }

        while(curlmsg != NULL) {
            if (curlmsg->msg == CURLMSG_DONE) {
                unsigned long size = _curlDataVector.size();
                for ( unsigned long i = 0; i < size; i++) {
                    if (_curlDataVector.at(i).get()->curl == curlmsg->easy_handle) {

                        if (EndsWith(GetPath( _curlDataVector.at(i).get()->uri), ".json")) {
                            rapidjson::ParseResult ok = doc.Parse<rapidjson::kParseFullPrecisionFlag>(_curlDataVector.at(i).get()->buffer.GetString());
                            if (!ok) {
                                throw OPENRAVE_EXCEPTION_FORMAT("failed to parse json document \"%s\"", currentUri, ORE_InvalidArguments);
                            }
                        }

                        else if (EndsWith(GetPath( _curlDataVector.at(i).get()->uri), ".msgpack")) {
                            try {
                                MsgPack::ParseMsgPack(doc, (const void*)_curlDataVector.at(i).get()->buffer.GetString(), _curlDataVector.at(i).get()->buffer.GetSize());
                            }
                            catch(const std::exception& ex) {
                                throw OPENRAVE_EXCEPTION_FORMAT("Failed to parse msgpack format for file '%s': %s", _curlDataVector.at(i).get()->uri%ex.what(), ORE_Failed);
                            }
                        }


                        curl_multi_remove_handle(_curlMultiHandle,_curlDataVector.at(i).get()->curl);
                        _curlDataVector.erase(_curlDataVector.begin() + i);
                        return;
                    }
                }
            }
            curlmsg = curl_multi_info_read(_curlMultiHandle, &msgInQueue);
        }
    } while(stillRunning != 0);

}

void JSONRemoteHelper::DownloadConnectedBodies()
{
    if (!_curlDataVector.size()) {
        return;
    }

    std::vector<int> vInputToBodyInfoMapping;
    // TODO make function to replace it with remote URL

    int stillRunning = 0;
    int msgInQueue = 0;
    do {
        CURLMcode mc = curl_multi_perform(_curlMultiHandle, &stillRunning);
        CURLMsg  *curlmsg = curl_multi_info_read(_curlMultiHandle, &msgInQueue);
        if(!mc && stillRunning) {
            /* wait for activity, timeout or "nothing" */
            mc = curl_multi_poll(_curlMultiHandle, NULL, 0, 1000, NULL);
        }
        if(mc) {
            break;
        }

        while(curlmsg != NULL) {
            if (curlmsg->msg == CURLMSG_DONE) {
                for ( unsigned long i = 0; i < _curlDataVector.size(); i++) {
                    if (_curlDataVector.at(i).get()->curl == curlmsg->easy_handle) {
                        boost::shared_ptr<rapidjson::Document> sharedDocument = boost::shared_ptr<rapidjson::Document>(new rapidjson::Document);

                        if (EndsWith(GetPath( _curlDataVector.at(i).get()->uri), ".json")) {
                            rapidjson::ParseResult ok = sharedDocument.get()->Parse(_curlDataVector.at(i).get()->buffer.GetString());
                            if (!ok) {
                                throw OPENRAVE_EXCEPTION_FORMAT("failed to parse json document \"%s\"", _curlDataVector.at(i).get()->uri, ORE_InvalidArguments);
                            }
                        }
                        else if (EndsWith(GetPath( _curlDataVector.at(i).get()->uri), ".msgpack")) {

                            try {
                                MsgPack::ParseMsgPack(*sharedDocument, (const void*)_curlDataVector.at(i).get()->buffer.GetString(), _curlDataVector.at(i).get()->buffer.GetSize());
                            }
                            catch(const std::exception& ex) {
                                throw OPENRAVE_EXCEPTION_FORMAT("Failed to parse msgpack format for file '%s': %s", _curlDataVector.at(i).get()->uri%ex.what(), ORE_Failed);
                            }


                        }

                        _PutDocumentIntoRapidJSONMap(_curlDataVector.at(i).get()->uri, sharedDocument);
                        _ParseDocumentForNewURLs( *sharedDocument);

                        curl_multi_remove_handle(_curlMultiHandle,_curlDataVector.at(i).get()->curl);
                        _curlDataVector.erase(_curlDataVector.begin() + i);

                        i--;
                        stillRunning++; // Increase still running by 1 to start new curl downloads even if the other ones have finished
                    }
                }
            }
            curlmsg = curl_multi_info_read(_curlMultiHandle, &msgInQueue);
        }
    } while(stillRunning != 0);

    return;
}

/// @brief Adds referenceUri to download
/// @param referenceUri URI to download
/// @return
int JSONRemoteHelper::AddReferenceURIToDownload(const std::string& referenceUri)
{
    boost::shared_ptr<JSONRemoteHelper::CurlData> data = boost::shared_ptr<JSONRemoteHelper::CurlData>(new JSONRemoteHelper::CurlData);
    data.get()->uri = referenceUri;
    if(data.get()->curl) {
        // Set up easy curl to download the correlating files
        curl_easy_setopt(data.get()->curl, CURLOPT_URL, _ResolveRemoteUri(data.get()->uri).c_str()); // c_str() is required
        curl_easy_setopt(data.get()->curl, CURLOPT_HTTPGET, 1);
        curl_easy_setopt(data.get()->curl, CURLOPT_WRITEFUNCTION, _WriteBackDataFromCurl);
        curl_easy_setopt(data.get()->curl, CURLOPT_WRITEDATA, data.get());
        curl_multi_add_handle(_curlMultiHandle, data.get()->curl);
        _curlDataVector.push_back(data); // These are active curls or soon to be active
        _urlsAlreadyStaged.push_back(referenceUri); // to keep track of which have been downloaded
    }
    else{
        return -1;
    }
    return 0;
}

bool JSONRemoteHelper::IsUrlAlreadyStaged(std::string uri)
{
    return std::find(_urlsAlreadyStaged.begin(), _urlsAlreadyStaged.end(), uri) == _urlsAlreadyStaged.end();
}





/// @brief Places document into the map of loaded documents, does a check to make sure that it is not already in there
/// @param fullURLname map key
/// @param document  map value document
void JSONRemoteHelper::_PutDocumentIntoRapidJSONMap(const std::string& fullURLname, boost::shared_ptr<const rapidjson::Document> document)
{
    if (_rapidJSONDocuments->find(GetPath(fullURLname)) == _rapidJSONDocuments->end()) {
        (*_rapidJSONDocuments)[GetPath(fullURLname)] = document;
    }
}

/// @brief This will parse the given document for reference URIs then put them in a queue to down in parallel
/// @param doc document to parse
void JSONRemoteHelper::_ParseDocumentForNewURLs(const rapidjson::Value& doc)
{
    std::vector<int> vInputToBodyInfoMapping;
    if (doc.HasMember("bodies")) {
        const rapidjson::Value& rBodies = doc["bodies"];
        vInputToBodyInfoMapping.resize(rBodies.Size(),-1); // -1, no mapping by default
        for(int iInputBodyIndex = 0; iInputBodyIndex < (int)rBodies.Size(); ++iInputBodyIndex) {
            const rapidjson::Value& rBodyInfo = rBodies[iInputBodyIndex];
            std::string bodyId = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "id", "");
            std::string bodyName = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "name", "");
            std::string referenceUri = orjson::GetJsonValueByKey<std::string>(rBodyInfo, "referenceUri", "");
            if (_IsExpandableReferenceUri(referenceUri) && std::find(_urlsAlreadyStaged.begin(), _urlsAlreadyStaged.end(), referenceUri) == _urlsAlreadyStaged.end()) {
                // Right now this checks to see if it is remote or loading from disk with file:/
                // "this will add the uri reference to download and warns if it fails
                if(AddReferenceURIToDownload(referenceUri)) {
                    RAVELOG_WARN_FORMAT("failed to create curl handler from %s", referenceUri);
                }
            }
            else if( !referenceUri.empty() && std::find(_urlsAlreadyStaged.begin(), _urlsAlreadyStaged.end(), referenceUri) == _urlsAlreadyStaged.end()) {
                throw OPENRAVE_EXCEPTION_FORMAT("body '%s' has invalid referenceUri='%s", bodyId%referenceUri, ORE_InvalidURI);
            }

        }
    }

}

std::string JSONRemoteHelper::_ResolveRemoteUri(const std::string &uri)
{

    std::string scheme, path, fragment;
    ParseURI(uri, scheme, path, fragment );

    if (scheme == "file") {
        return ("file://" + ResolveURI(scheme, path, std::string(), _vOpenRAVESchemeAliases));
    }

    return _remoteUrl + uri.substr(uri.find_last_of(":/")+1,  uri.find_last_of("#") - uri.find_last_of(":/") -1);
}

/// \brief returns true if the referenceUri is a valid URI that can be loaded
bool JSONRemoteHelper::_IsExpandableReferenceUri(const std::string& referenceUri) const
{

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

