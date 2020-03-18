// -*- coding: utf-8 -*-
// Copyright (C) 2020 OpenRAVE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** \file openravemsgpack.h
    \brief Wrapper for msgpack.
 */
#ifndef OPENRAVE_MSGPACK_H
#define OPENRAVE_MSGPACK_H

#include <openrave/config.h>

#include <vector>
#include <string>
#include <iostream>

#include <rapidjson/document.h>

namespace openravemsgpack {

enum OpenRAVEMsgPackErrorCode
{
    ORMPE_Failed=0,
    ORMPE_InvalidArguments=1  ///< passed in input arguments are not valid
};

inline const char* GetErrorCodeString(OpenRAVEMsgPackErrorCode error)
{
    switch(error) {
    case ORMPE_Failed: return "Failed";
    case ORMPE_InvalidArguments: return "Invalid arguments";
    }
    // throw an exception?
    return "";
}

/// \brief Exception that OpenRAVEMsgPack internal methods throw; the error codes are held in \ref OpenRAVEMsgPackErrorCode.
class OpenRAVEMsgPackException : public std::exception
{
public:
    OpenRAVEMsgPackException() : std::exception(), _s(""), _error(ORMPE_Failed) {
    }
    OpenRAVEMsgPackException(const std::string& s, OpenRAVEMsgPackErrorCode error=ORMPE_Failed ) : std::exception() {
        _error = error;
        _s = "openrave msgpack (";
        _s += GetErrorCodeString(error);
        _s += "): ";
        _s += s;
        _ssub = s;
    }
    virtual ~OpenRAVEMsgPackException() throw() {
    }

    /// \brief outputs everything
    char const* what() const throw() {
        return _s.c_str();
    }

    /// \brief outputs everything
    const std::string& message() const {
        return _s;
    }

    /// \briefs just the sub-message
    const std::string& GetSubMessage() const {
        return _ssub;
    }

    OpenRAVEMsgPackErrorCode GetCode() const {
        return _error;
    }

private:
    std::string _s, _ssub;
    OpenRAVEMsgPackErrorCode _error;
};


OPENRAVE_API void DumpMsgPack(const rapidjson::Value& value, std::ostream& os);
OPENRAVE_API void DumpMsgPack(const rapidjson::Value& value, std::vector<char>& output);

OPENRAVE_API void ParseMsgPack(rapidjson::Document& d, const std::string& str);
OPENRAVE_API void ParseMsgPack(rapidjson::Document& d, std::istream& is);

} // namespace openravemsgpack

#endif // OPENRAVE_MSGPACK_H
