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
/** \file jsondownloader.h
    \brief Helper class for jsonreader to download files remotely
 */
#include "jsoncommon.h"

#include <openrave/openravejson.h>
#include <openrave/openravemsgpack.h>
#include <openrave/openrave.h>
#include <rapidjson/istreamwrapper.h>
#include <string>
#include <fstream>
#include <curl/curl.h> /// Required for remote URI


#ifndef OPENRAVE_JSON_DOWNLOADER_H
#define OPENRAVE_JSON_DOWNLOADER_H

namespace OpenRAVE {

/// \brief data type that is passed to curl handler containing information on parsing incoming files
struct JSONDownloadContext
{
public:
    JSONDownloadContext();
    ~JSONDownloadContext();

    std::string uri; // url of the download
    CURL * curl = nullptr; // the curl handle for this download
    rapidjson::StringBuffer buffer; // buffer used to receive downloaded data
    rapidjson::Document *pDoc = nullptr; // if non-null, the caller-supplied document to put results into
    uint64_t startTimestampUS = 0;
};
typedef boost::shared_ptr<JSONDownloadContext> JSONDownloadContextPtr;

class JSONDownloader {

public:
    JSONDownloader(rapidjson::Document::AllocatorType& alloc, std::map<std::string, boost::shared_ptr<const rapidjson::Document> >& rapidJSONDocuments, const std::string& remoteUrl, const std::vector<std::string>& vOpenRAVESchemeAliases, int deserializeOptions);
    ~JSONDownloader();

    void Download(const std::string& uri, rapidjson::Document& doc);    

    /// \brief Queues uri to download
    /// \param uri URI to download
    void QueueDownloadURI(const std::string& uri) {
        _QueueDownloadURI(uri, nullptr);
    }

    /// \brief Downloads all the remote files, parse them, download other references, then store them in document map
    /// \param rEnvInfo top layer
    void QueueDownloadReferenceURIs(const rapidjson::Value& rEnvInfo);

    /// \brief Wait for queued downloads to finish
    void WaitForDownloads();

protected: 

    void _QueueDownloadURI(const std::string& uri, rapidjson::Document* pDoc);

    /// \brief returns true if the referenceUri is a valid URI that can be loaded
    bool _IsExpandableReferenceUri(const std::string& referenceUri) const;

    rapidjson::Document::AllocatorType& _alloc;
    std::map<std::string, boost::shared_ptr<const rapidjson::Document> >& _rapidJSONDocuments; ///< cache for opened rapidjson Documents
    const std::string& _remoteUrl; ///< remote url for scheme
    const std::vector<std::string>& _vOpenRAVESchemeAliases;
    int _deserializeOptions = 0;

    CURLM *_curlm = nullptr; ///< curl multi handler, used to downlod files simultaneously
    std::map<CURL*, JSONDownloadContextPtr> _mapDownloadContexts; ///< map from curl handle to download context
};

}

#endif
