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

#include "jsondownloader.h"

#if OPENRAVE_CURL

#include <openrave/openravemsgpack.h>

namespace OpenRAVE {

static bool _EndsWith(const std::string& fullString, const std::string& endString)
{
    if (fullString.length() >= endString.length()) {
        return fullString.compare(fullString.length() - endString.length(), endString.length(), endString) == 0;
    }
    return false;
}

/// \brief get the scheme of the uri, e.g. file: or openrave:
static void _ParseURI(const std::string& uri, std::string& scheme, std::string& path, std::string& fragment)
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

static void _ParseMsgPackDocument(const rapidjson::StringBuffer& buffer, const std::string& uri, rapidjson::Document& doc)
{
    try {
        MsgPack::ParseMsgPack(doc, buffer.GetString(), buffer.GetSize());
    }
    catch(const std::exception& ex) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to parse msgpack document \"%\"': %s", uri%ex.what(), ORE_CurlInvalidResponse);
    }
}

static void _ParseJsonDocument(const rapidjson::StringBuffer& buffer, const std::string& uri, rapidjson::Document& doc)
{
    rapidjson::ParseResult ok = doc.Parse<rapidjson::kParseFullPrecisionFlag>(buffer.GetString());
    if (!ok) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to parse json document \"%s\"", uri, ORE_CurlInvalidResponse);
    }
}

JSONDownloadContext::JSONDownloadContext()
{
    curl = curl_easy_init();
    if (!curl) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to init curl handle", ORE_CurlInvalidHandle);
    }
}

JSONDownloadContext::~JSONDownloadContext()
{
    if (!!curl) {
        curl_easy_cleanup(curl);
        curl = nullptr;
    }
}

JSONDownloader::JSONDownloader(rapidjson::Document::AllocatorType& alloc, std::map<std::string, boost::shared_ptr<const rapidjson::Document> >& rapidJSONDocuments, const std::string& remoteUrl, const std::vector<std::string>& vOpenRAVESchemeAliases, bool downloadRecursively) :
    _alloc(alloc),
    _rapidJSONDocuments(rapidJSONDocuments),
    _remoteUrl(remoteUrl),
    _vOpenRAVESchemeAliases(vOpenRAVESchemeAliases),
    _downloadRecursively(downloadRecursively)
{
    _curlm = curl_multi_init();
    if (!_curlm) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to create curl handle", ORE_CurlInvalidHandle);
    }

    _userAgent = boost::str(boost::format("OpenRAVE/%s")%OPENRAVE_VERSION_STRING);
}
JSONDownloader::~JSONDownloader()
{
    if (!!_curlm) {
        curl_multi_cleanup(_curlm);
        _curlm = nullptr;
    }
}

void JSONDownloader::Download(const std::string& uri, rapidjson::Document& doc, uint64_t timeoutUS)
{
    _QueueDownloadURI(uri, &doc);
    WaitForDownloads(timeoutUS);
}

void JSONDownloader::WaitForDownloads(uint64_t timeoutUS)
{
    if (_mapDownloadContexts.empty()) {
        return;
    }

    const uint64_t startTimestampUS = utils::GetMonotonicTime();
    int numDownloads = 0;
    while (!_mapDownloadContexts.empty()) {
        // check if any handle still running
        int numRunningHandles = 0;
        const CURLMcode performCode = curl_multi_perform(_curlm, &numRunningHandles);
        if (performCode) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to download, curl_multi_perform() failed with code %d", (int)performCode, ORE_CurlInvalidHandle);
        }
        RAVELOG_VERBOSE_FORMAT("curl_multi_perform(): numRunningHandles = %d", numRunningHandles);
        if (numRunningHandles > 0) {
            // poll for 100ms
            const CURLMcode pollCode = curl_multi_poll(_curlm, NULL, 0, 100, NULL);
            if (pollCode) {
                throw OPENRAVE_EXCEPTION_FORMAT("failed to download, curl_multi_poll() failed with code %d", (int)pollCode, ORE_CurlInvalidHandle);
            }
        }

        // check for timeout
        const uint64_t currentTimestampUS = utils::GetMonotonicTime();
        if (currentTimestampUS > startTimestampUS + timeoutUS) {
            throw OPENRAVE_EXCEPTION_FORMAT("timed out waiting for download to finish, timeout is %d[us]", timeoutUS, ORE_CurlTimeout);
        }

        // process all messages
        for (;;) {
            int numMessagesInQueue = 0;
            const CURLMsg *curlMessage = curl_multi_info_read(_curlm, &numMessagesInQueue);
            RAVELOG_VERBOSE_FORMAT("curl_multi_info_read(): numMessagesInQueue = %d", numMessagesInQueue);
            if (!curlMessage) {
                break;
            }
            RAVELOG_VERBOSE_FORMAT("curl_multi_info_read(): curlMessage->msg = %d", (int)curlMessage->msg);
            if (curlMessage->msg != CURLMSG_DONE) {
                continue;
            }

            // remove handle
            const CURLMcode removeHandleCode = curl_multi_remove_handle(_curlm, curlMessage->easy_handle);
            if (removeHandleCode) {
                throw OPENRAVE_EXCEPTION_FORMAT("failed to download, curl_multi_remove_handle() failed with code %d", (int)removeHandleCode, ORE_CurlInvalidHandle);
            }
            const std::map<CURL*, JSONDownloadContextPtr>::iterator it = _mapDownloadContexts.find(curlMessage->easy_handle);
            if (it == _mapDownloadContexts.end()) {
                // if this happens, it is probably a bug in this code
                throw OPENRAVE_EXCEPTION_FORMAT0("curl download finished, but failed to find corresponding context, cannot continue", ORE_CurlInvalidHandle);
            }
            JSONDownloadContextPtr pContext = it->second;
            _mapDownloadContexts.erase(it);

            // check result
            const CURLcode curlCode = curlMessage->data.result;
            if (curlCode != CURLE_OK) {
                throw OPENRAVE_EXCEPTION_FORMAT("failed to download uri \"%s\": %s", pContext->uri%curl_easy_strerror(curlCode), ORE_CurlInvalidResponse);
            }

            // check http response code
            int responseCode = 0;
            const CURLcode getInfoCode = curl_easy_getinfo(pContext->curl, CURLINFO_RESPONSE_CODE, &responseCode);
            if (getInfoCode != CURLE_OK) {
                throw OPENRAVE_EXCEPTION_FORMAT("failed to get response status code for uri \"%s\": %s", pContext->uri%curl_easy_strerror(getInfoCode), ORE_CurlInvalidHandle);
            }
            if (responseCode != 0 && responseCode != 200) {
                // file scheme downloads have a zero response code
                throw OPENRAVE_EXCEPTION_FORMAT("failed to download uri \"%s\", received http %d response", pContext->uri%responseCode, ORE_CurlInvalidResponse);
            }

            // parse data
            if (_EndsWith(pContext->uri, ".json")) {
                _ParseJsonDocument(pContext->buffer, pContext->uri, *pContext->pDoc);
                if (_downloadRecursively) {
                    QueueDownloadReferenceURIs(*pContext->pDoc);
                }
            }
            else if (_EndsWith(pContext->uri, ".msgpack")) {
                _ParseMsgPackDocument(pContext->buffer, pContext->uri, *pContext->pDoc);
                if (_downloadRecursively) {
                    QueueDownloadReferenceURIs(*pContext->pDoc);
                }
            }

            RAVELOG_DEBUG_FORMAT("successfully downloaded \"%s\", took %d[us]", pContext->uri%(currentTimestampUS-pContext->startTimestampUS));
            ++numDownloads;
        }
    }

    const uint64_t stopTimestampUS = utils::GetMonotonicTime();
    RAVELOG_DEBUG_FORMAT("downloaded %d files, took %d[us]", numDownloads%(stopTimestampUS-startTimestampUS));
}

static size_t _WriteBackDataFromCurl(const char *data, size_t size, size_t dataSize, JSONDownloadContext* pContext)
{
    const size_t numBytes = size * dataSize;
    std::memcpy(pContext->buffer.Push(numBytes), data, numBytes);
    return numBytes;
}

void JSONDownloader::_QueueDownloadURI(const std::string& uri, rapidjson::Document* pDoc)
{
    std::string scheme, path, fragment;
    _ParseURI(uri, scheme, path, fragment);

    if (scheme.empty() && path.empty()) {
        RAVELOG_WARN_FORMAT("unknown uri \"%s\"", uri);
        return; // unknown uri
    }
    std::string canonicalUri;
    std::string url;
    if (scheme == "file") {
        std::string fullFilename = RaveFindLocalFile(path);
        if (fullFilename.empty()) {
            RAVELOG_WARN_FORMAT("failed to resolve uri \"%s\" to local file", uri);
            return; // no such file to download
        }
        url = "file://" + fullFilename;
        canonicalUri = path;
    }
    else if (std::find(_vOpenRAVESchemeAliases.begin(), _vOpenRAVESchemeAliases.end(), scheme) != _vOpenRAVESchemeAliases.end()) {
        url = _remoteUrl + path;
        canonicalUri = scheme + ":" + path;
    }
    else {
        RAVELOG_WARN_FORMAT("unable to handle uri \"%s\"", uri);
        return; // do not understand this uri
    }

    if (!pDoc) {
        if (_rapidJSONDocuments.find(canonicalUri) != _rapidJSONDocuments.end()) {
            RAVELOG_VERBOSE_FORMAT("uri \"%s\" already in cache", canonicalUri);
            return; // already in _rapidJSONDocuments
        }
        // create a doc and insert to map first
        boost::shared_ptr<rapidjson::Document> pNewDoc = boost::make_shared<rapidjson::Document>(&_alloc);
        _rapidJSONDocuments[canonicalUri] = pNewDoc;
        pDoc = pNewDoc.get();
    }

    JSONDownloadContextPtr pContext = boost::make_shared<JSONDownloadContext>();
    pContext->uri = canonicalUri;
    pContext->pDoc = pDoc;
    pContext->startTimestampUS = utils::GetMonotonicTime();

    // set curl options
    CURLcode curlCode;
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_USERAGENT, _userAgent.c_str());
    if (curlCode != CURLE_OK) { 
        throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_USERAGENT) for uri \"%s\": %s", canonicalUri%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
    }
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_FOLLOWLOCATION, 1);
    if (curlCode != CURLE_OK) { 
        throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_FOLLOWLOCATION) for uri \"%s\": %s", canonicalUri%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
    }
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_MAXREDIRS, 10);
    if (curlCode != CURLE_OK) { 
        throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_MAXREDIRS) for uri \"%s\": %s", canonicalUri%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
    }
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_NOSIGNAL, 1);
    if (curlCode != CURLE_OK) { 
        throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_NOSIGNAL) for uri \"%s\": %s", canonicalUri%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
    }
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_URL, url.c_str());
    if (curlCode != CURLE_OK) { 
        throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_URL) for uri \"%s\": %s", canonicalUri%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
    }
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_HTTPGET, 1);
    if (curlCode != CURLE_OK) { 
        throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_HTTPGET) for uri \"%s\": %s", canonicalUri%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
    }
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_WRITEFUNCTION, _WriteBackDataFromCurl);
    if (curlCode != CURLE_OK) { 
        throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_WRITEFUNCTION) for uri \"%s\": %s", canonicalUri%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
    }
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_WRITEDATA, pContext.get());
    if (curlCode != CURLE_OK) { 
        throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_WRITEDATA) for uri \"%s\": %s", canonicalUri%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
    }

    // add handle to curl multi
    const CURLMcode addHandleCode = curl_multi_add_handle(_curlm, pContext->curl);
    if (!!addHandleCode) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to download uri \"%s\", curl_multi_add_handle() failed with code %d", canonicalUri%(int)addHandleCode, ORE_CurlInvalidHandle);
    }

    _mapDownloadContexts[pContext->curl] = pContext;
    RAVELOG_DEBUG_FORMAT("start to download uri \"%s\" from \"%s\"", canonicalUri%url);
}

void JSONDownloader::QueueDownloadReferenceURIs(const rapidjson::Value& rEnvInfo)
{
    if (!rEnvInfo.IsObject()) {
        return;
    }

    const rapidjson::Value::ConstMemberIterator itBodies = rEnvInfo.FindMember("bodies");
    if (itBodies == rEnvInfo.MemberEnd()) {
        return;
    }
    const rapidjson::Value& rBodies = itBodies->value;
    for (rapidjson::Value::ConstValueIterator itBody = rBodies.Begin(); itBody != rBodies.End(); ++itBody) {
        const rapidjson::Value& rBody = *itBody;
        if (!rBody.IsObject()) {
            continue;
        }
        const std::string referenceUri = orjson::GetJsonValueByKey<std::string>(rBody, "referenceUri", "");
        if (referenceUri.empty()) {
            continue;
        }
        if (!_IsExpandableReferenceUri(referenceUri)) {
            throw OPENRAVE_EXCEPTION_FORMAT("body '%s' has invalid referenceUri='%s", orjson::GetJsonValueByKey<std::string>(rBody, "id", "")%referenceUri, ORE_InvalidURI);
        }
        QueueDownloadURI(referenceUri);
    }
}

/// \brief returns true if the referenceUri is a valid URI that can be loaded
bool JSONDownloader::_IsExpandableReferenceUri(const std::string& referenceUri) const
{
    if (referenceUri.empty()) {
        return false;
    }
    std::string scheme, path, fragment;
    _ParseURI(referenceUri, scheme, path, fragment);
    if (!fragment.empty()) {
        return true;
    }
    else if (!scheme.empty() && !path.empty()) {
        return true;
    }
    return false;
}

}

#endif // OPENRAVE_CURL
