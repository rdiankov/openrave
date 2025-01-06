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
#include "stringutils.h"

#if OPENRAVE_CURL

#include <openrave/openravemsgpack.h>

namespace OpenRAVE {

// Need to forward declare this
static void _ParseDocument(OpenRAVE::JSONDownloadContextPtr pContext);

static void _DecryptDocument(OpenRAVE::JSONDownloadContextPtr pContext)
{
    std::istringstream iss(pContext->buffer, std::ios::in | std::ios::binary);
    std::ostringstream oss;
    if (GpgDecrypt(iss, oss)) {
        if (RemoveSuffix(pContext->uri, ".gpg")) {
            pContext->buffer = oss.str();
            _ParseDocument(pContext);
        }
    } else {
        RAVELOG_ERROR("Failed to decrypt document.");
    }
}

static void _ParseDocument(OpenRAVE::JSONDownloadContextPtr pContext)
{
    if (StringEndsWith(pContext->uri, ".json")) {
        rapidjson::ParseResult ok = pContext->pDoc->Parse<rapidjson::kParseFullPrecisionFlag>(pContext->buffer.data(), pContext->buffer.size());
        if (!ok) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to parse json document \"%s\"", pContext->uri, ORE_CurlInvalidResponse);
        }
    }
    else if (StringEndsWith(pContext->uri, ".msgpack")) {
        MsgPack::ParseMsgPack(*(pContext->pDoc), pContext->buffer.data(), pContext->buffer.size());
    }
    else if (StringEndsWith(pContext->uri, ".gpg")) {
        _DecryptDocument(pContext);
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT("Do not know how to parse data from uri '%s', supported is json/msgpack", pContext->uri, ORE_EnvironmentFormatUnrecognized);
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

JSONDownloader::JSONDownloader(std::map<std::string, boost::shared_ptr<const rapidjson::Document> >& rapidJSONDocuments, const std::vector<std::string>& vOpenRAVESchemeAliases, const std::string& remoteUrl, const std::string& unixEndpoint) :
    _rapidJSONDocuments(rapidJSONDocuments),
    _vOpenRAVESchemeAliases(vOpenRAVESchemeAliases),
    _remoteUrl(remoteUrl),
    _unixEndpoint(unixEndpoint)
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

JSONDownloaderScope::JSONDownloaderScope(const std::string& contextdesc, JSONDownloader& downloader, rapidjson::Document::AllocatorType& alloc, bool downloadRecursively) :
    _downloader(downloader),
    _alloc(alloc),
    _downloadRecursively(downloadRecursively)
{
    _contextdesc = contextdesc;
}

JSONDownloaderScope::~JSONDownloaderScope()
{
}

bool JSONDownloaderScope::Download(const char* pUri, rapidjson::Document& doc, bool bMustResolveURI, uint64_t timeoutUS)
{
    _QueueDownloadURI(pUri, &doc);
    return WaitForDownloads(bMustResolveURI, timeoutUS);
}

bool JSONDownloaderScope::WaitForDownloads(bool bMustResolveURI, uint64_t timeoutUS)
{
    if (_mapDownloadContexts.empty()) {
        return true;
    }

    bool bHasInvalidURI = false;
    const uint64_t startTimestampUS = utils::GetMonotonicTime();
    int numDownloads = 0;
    while (!_mapDownloadContexts.empty()) {
        // check if any handle still running
        int numRunningHandles = 0;
        const CURLMcode performCode = curl_multi_perform(_downloader._curlm, &numRunningHandles);
        if (performCode) {
            throw OPENRAVE_EXCEPTION_FORMAT("%s failed to download, curl_multi_perform() failed with code %d", _contextdesc%(int)performCode, ORE_CurlInvalidHandle);
        }
        RAVELOG_VERBOSE_FORMAT("%s curl_multi_perform(): numRunningHandles = %d", _contextdesc%numRunningHandles);
        if (numRunningHandles > 0) {
            // poll for 100ms
            const CURLMcode pollCode = curl_multi_poll(_downloader._curlm, NULL, 0, 100, NULL);
            if (pollCode) {
                throw OPENRAVE_EXCEPTION_FORMAT("%s failed to download, curl_multi_poll() failed with code %d", _contextdesc%(int)pollCode, ORE_CurlInvalidHandle);
            }
        }

        // check for timeout
        const uint64_t currentTimestampUS = utils::GetMonotonicTime();
        if (currentTimestampUS > startTimestampUS + timeoutUS) {
            throw OPENRAVE_EXCEPTION_FORMAT("%s timed out waiting for download to finish, timeout is %d[us]", _contextdesc%timeoutUS, ORE_CurlTimeout);
        }

        // process all messages
        for (;;) {
            int numMessagesInQueue = 0;
            const CURLMsg *curlMessage = curl_multi_info_read(_downloader._curlm, &numMessagesInQueue);
            RAVELOG_VERBOSE_FORMAT("%s curl_multi_info_read(): numMessagesInQueue=%d", _contextdesc%numMessagesInQueue);
            if (!curlMessage) {
                break;
            }
            RAVELOG_VERBOSE_FORMAT("%s curl_multi_info_read(): curlMessage->msg=%d", _contextdesc%(int)curlMessage->msg);
            if (curlMessage->msg != CURLMSG_DONE) {
                // go to the next message in the queue
                continue;
            }

            // remove handle
            const CURLMcode removeHandleCode = curl_multi_remove_handle(_downloader._curlm, curlMessage->easy_handle);
            if (removeHandleCode) {
                throw OPENRAVE_EXCEPTION_FORMAT("%s failed to download, curl_multi_remove_handle() failed with code %d", _contextdesc%(int)removeHandleCode, ORE_CurlInvalidHandle);
            }
            const std::map<CURL*, JSONDownloadContextPtr>::iterator it = _mapDownloadContexts.find(curlMessage->easy_handle);
            if (it == _mapDownloadContexts.end()) {
                // if this happens, it is probably a bug in this code
                throw OPENRAVE_EXCEPTION_FORMAT("%s curl download finished failed to find corresponding context, cannot continue", _contextdesc, ORE_CurlInvalidHandle);
            }
            JSONDownloadContextPtr pContext;
            pContext.swap(it->second);
            _mapDownloadContexts.erase(it);

            // check result
            const CURLcode curlCode = curlMessage->data.result;
            if (curlCode != CURLE_OK) {
                throw OPENRAVE_EXCEPTION_FORMAT("%s failed to download uri '%s': %s", _contextdesc%pContext->uri%curl_easy_strerror(curlCode), ORE_CurlInvalidResponse);
            }

            // check http response code
            long responseCode = 0;
            const CURLcode getInfoCode = curl_easy_getinfo(pContext->curl, CURLINFO_RESPONSE_CODE, &responseCode);
            if (getInfoCode != CURLE_OK) {
                throw OPENRAVE_EXCEPTION_FORMAT("%s failed to get response status code for uri '%s': %s", _contextdesc%pContext->uri%curl_easy_strerror(getInfoCode), ORE_CurlInvalidHandle);
            }
            if (responseCode != 0 && responseCode != 200) {
                // file scheme downloads have a zero response code
                if (bMustResolveURI || responseCode != 404 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("%s failed to download uri '%s', received http %d response", _contextdesc%pContext->uri%responseCode, responseCode == 404 ? ORE_InvalidURI : ORE_CurlInvalidResponse);
                }
                else {
                    RAVELOG_DEBUG_FORMAT("%s failed to download uri '%s', received http %d response, continuing since mustresolveuri is not true", _contextdesc%pContext->uri%responseCode);
                    bHasInvalidURI = true;
                    continue;
                }
            }

            // parse data
            _ParseDocument(pContext);

            RAVELOG_DEBUG_FORMAT("%s successfully downloaded '%s', took %d[us]", _contextdesc%pContext->uri%(currentTimestampUS-pContext->startTimestampUS));
            ++numDownloads;

            // reuse the context object later
            const rapidjson::Document& doc = *pContext->pDoc;
            pContext->pDoc = nullptr;
            pContext->buffer.clear();
            _downloader._vDownloadContextPool.emplace_back();
            _downloader._vDownloadContextPool.back().swap(pContext);

            // queue other resources to be downloaded
            if (_downloadRecursively) {
                QueueDownloadReferenceURIs(doc);
            }
        }
    }

    const uint64_t stopTimestampUS = utils::GetMonotonicTime();
    RAVELOG_DEBUG_FORMAT("%s downloaded %d files, took %d[us]", _contextdesc%numDownloads%(stopTimestampUS-startTimestampUS));
    return !bHasInvalidURI;
}

static size_t _WriteBackDataFromCurl(const char *data, size_t size, size_t dataSize, JSONDownloadContext* pContext)
{
    const size_t numBytes = size * dataSize;
    pContext->buffer.append(data, numBytes);
    return numBytes;
}

void JSONDownloaderScope::_QueueDownloadURI(const char* pUri, rapidjson::Document* pDoc)
{
    if( !pUri[0] ) {
        return;
    }
    std::string scheme, path, fragment;
    ParseURI(pUri, scheme, path, fragment);

    if (scheme.empty() && path.empty()) {
        RAVELOG_VERBOSE_FORMAT("skipping uri '%s' due to empty path/scheme.", pUri);
        return; // unknown uri
    }
    RAVELOG_VERBOSE_FORMAT("downloading uri '%s'.", pUri);
    std::string canonicalUri;
    std::string url;
    if (scheme == "file") {
        std::string fullFilename = RaveFindLocalFile(path);
        if (fullFilename.empty()) {
            RAVELOG_WARN_FORMAT("failed to resolve uri \"%s\" to local file", pUri);
            return; // no such file to download
        }
        url = "file://" + fullFilename;
        canonicalUri = path;
    }
    else if (std::find(_downloader._vOpenRAVESchemeAliases.begin(), _downloader._vOpenRAVESchemeAliases.end(), scheme) != _downloader._vOpenRAVESchemeAliases.end()) {
        url = _downloader._remoteUrl + path;
        canonicalUri = scheme + ":" + path;
    }
    else {
        RAVELOG_WARN_FORMAT("unable to handle uri \"%s\"", pUri);
        return; // do not understand this uri
    }

    if (!pDoc) {
        if (_downloader._rapidJSONDocuments.find(canonicalUri) != _downloader._rapidJSONDocuments.end()) {
            RAVELOG_VERBOSE_FORMAT("uri \"%s\" already in cache", canonicalUri);
            return; // already in _rapidJSONDocuments
        }
        // create a doc and insert to map first
        boost::shared_ptr<rapidjson::Document> pNewDoc = boost::make_shared<rapidjson::Document>(&_alloc);
        _downloader._rapidJSONDocuments[canonicalUri] = pNewDoc;
        pDoc = pNewDoc.get();
    }

    JSONDownloadContextPtr pContext;
    if (!_downloader._vDownloadContextPool.empty()) {
        pContext.swap(_downloader._vDownloadContextPool.back());
        _downloader._vDownloadContextPool.pop_back();
        RAVELOG_VERBOSE_FORMAT("re-used download context from pool, %d left in pool", _downloader._vDownloadContextPool.size());
    } else {
        pContext = boost::make_shared<JSONDownloadContext>();
    }
    pContext->uri = canonicalUri;
    pContext->pDoc = pDoc;
    pContext->startTimestampUS = utils::GetMonotonicTime();

    // set curl options
    CURLcode curlCode;
    curlCode = curl_easy_setopt(pContext->curl, CURLOPT_USERAGENT, _downloader._userAgent.c_str());
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
    if (!_downloader._unixEndpoint.empty()) {
        curlCode = curl_easy_setopt(pContext->curl, CURLOPT_UNIX_SOCKET_PATH, _downloader._unixEndpoint.c_str());
        if (curlCode != CURLE_OK) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to curl_easy_setopt(CURLOPT_UNIX_SOCKET_PATH) for uri \"%s\" unix endpoint \"%s\": %s", canonicalUri%_downloader._unixEndpoint%curl_easy_strerror(curlCode), ORE_CurlInvalidHandle);
        }
    }

    // add handle to curl multi
    const CURLMcode addHandleCode = curl_multi_add_handle(_downloader._curlm, pContext->curl);
    if (!!addHandleCode) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to download uri \"%s\", curl_multi_add_handle() failed with code %d", canonicalUri%(int)addHandleCode, ORE_CurlInvalidHandle);
    }

    _mapDownloadContexts[pContext->curl].swap(pContext);
    RAVELOG_VERBOSE_FORMAT("%s start to download uri '%s' from '%s'", _contextdesc%canonicalUri%url);
}

void JSONDownloaderScope::QueueDownloadReferenceURIs(const rapidjson::Value& rEnvInfo)
{
    if (!rEnvInfo.IsObject()) {
        return;
    }

    const rapidjson::Value::ConstMemberIterator itBodies = rEnvInfo.FindMember("bodies");
    if (itBodies == rEnvInfo.MemberEnd()) {
        return;
    }
    const rapidjson::Value& rBodies = itBodies->value;
    if( !rBodies.IsArray() ) {
        return;
    }
    for (rapidjson::Value::ConstValueIterator itBody = rBodies.Begin(); itBody != rBodies.End(); ++itBody) {
        const rapidjson::Value& rBody = *itBody;
        if (!rBody.IsObject()) {
            continue;
        }
        const char* pReferenceUri = orjson::GetCStringJsonValueByKey(rBody, "referenceUri", "");
        if (!pReferenceUri[0]) {
            continue;
        }
        if (!_IsExpandableReferenceUri(pReferenceUri)) {
            const char* pId = orjson::GetCStringJsonValueByKey(rBody, "id","");
            throw OPENRAVE_EXCEPTION_FORMAT("bodyId '%s' has invalid referenceUri='%s", pId%pReferenceUri, ORE_InvalidURI);
        }
        QueueDownloadURI(pReferenceUri);
    }
}

/// \brief returns true if the referenceUri is a valid URI that can be loaded
bool JSONDownloaderScope::_IsExpandableReferenceUri(const char* pReferenceUri) const
{
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

}

#endif // OPENRAVE_CURL
