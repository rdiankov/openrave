// -*- coding: utf-8 -*-
// Copyright (C) 2019 OpenRAVE
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
/** \file openravejson.h
    \brief Wrapper for rapidjson.
 */
#ifndef OPENRAVE_JSON_H
#define OPENRAVE_JSON_H

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <stdint.h>
#include <string>
#include <stdexcept>
#include <vector>
#include <map>
#include <iostream>

#include <rapidjson/pointer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/error/en.h>
#include <rapidjson/prettywriter.h>

namespace openravejson {

enum OpenraveJSONErrorCode
{
    MJE_Failed=0,
};

inline const char* GetErrorCodeString(OpenraveJSONErrorCode error)
{
    switch(error) {
    case MJE_Failed: return "Failed";
    }
    // throw an exception?
    return "";
}

/// \brief Exception that OpenraveJSON internal methods throw; the error codes are held in \ref OpenraveJSONErrorCode.
class OpenraveJSONException : public std::exception
{
public:
    OpenraveJSONException() : std::exception(), _s(""), _error(MJE_Failed) {
    }
    OpenraveJSONException(const std::string& s, OpenraveJSONErrorCode error=MJE_Failed ) : std::exception() {
        _error = error;
        _s = "openrave (";
        _s += "): ";
        _s += s;
        _ssub = s;
    }
    virtual ~OpenraveJSONException() throw() {
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

    OpenraveJSONErrorCode GetCode() const {
        return _error;
    }

private:
    std::string _s, _ssub;
    OpenraveJSONErrorCode _error;
};


template<class T> inline std::string GetJsonString(const T& t);


/// \brief gets a string of the Value type for debugging purposes
inline std::string GetJsonTypeName(const rapidjson::Value& v) {
    int type = v.GetType();
    switch (type) {
    case 0:
        return "Null";
    case 1:
        return "False";
    case 2:
        return "True";
    case 3:
        return "Object";
    case 4:
        return "Array";
    case 5:
        return "String";
    case 6:
        return "Number";
    default:
        return "Unknown";
    }
}

template<class T> inline std::string GetJsonString(const T& t);

inline std::string DumpJson(const rapidjson::Value& value, const unsigned int indent=0) {
    rapidjson::StringBuffer stringbuffer;
    if (indent == 0) {
        rapidjson::Writer<rapidjson::StringBuffer> writer(stringbuffer);
        value.Accept(writer);
    } else {
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(stringbuffer);
        writer.SetIndent(' ', indent);
        value.Accept(writer);
    }
    return std::string(stringbuffer.GetString(), stringbuffer.GetSize());
}

inline void DumpJson(const rapidjson::Value& value, std::ostream& os, const unsigned int indent=0) {
    rapidjson::OStreamWrapper osw(os);
    if (indent == 0) {
        rapidjson::Writer<rapidjson::OStreamWrapper> writer(osw);
        value.Accept(writer);
    } else {
        rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
        writer.SetIndent(' ', indent);
        value.Accept(writer);
    }
}

inline void ParseJson(rapidjson::Document& d, const std::string& str) {
    // repeatedly calling Parse on the same rapidjson::Document will not release previsouly allocated memory, memory will accumulate until the object is destroyed
    // we use a new temporary Document to parse, and swap content with the original one, so that memory in original Document will be released when this function ends
    // see: https://github.com/Tencent/rapidjson/issues/1333
    rapidjson::Document tempDoc;
    tempDoc.Parse<rapidjson::kParseFullPrecisionFlag>(str.c_str()); // parse float in full precision mode
    if (tempDoc.HasParseError()) {
        std::string substr;
        if (str.length()> 200) {
            substr = str.substr(0, 200);
        } else {
            substr = str;
        }
        throw OpenraveJSONException(boost::str(boost::format("Json string is invalid (offset %u) %s str=%s")%((unsigned)d.GetErrorOffset())%GetParseError_En(d.GetParseError())%substr), MJE_Failed);
    }
    d.Swap(tempDoc);
}

inline void ParseJson(rapidjson::Document& d, std::istream& is) {
    rapidjson::IStreamWrapper isw(is);
    // see note in: void ParseJson(rapidjson::Document& d, const std::string& str)
    rapidjson::Document(tempDoc);
    tempDoc.ParseStream<rapidjson::kParseFullPrecisionFlag>(isw); // parse float in full precision mode
    if (tempDoc.HasParseError()) {
        throw OpenraveJSONException(boost::str(boost::format("Json stream is invalid (offset %u) %s")%((unsigned)tempDoc.GetErrorOffset())%GetParseError_En(tempDoc.GetParseError())), MJE_Failed);
    }
    d.Swap(tempDoc);
}

class JsonSerializable {
public:
    virtual void LoadFromJson(const rapidjson::Value& v) = 0;
    virtual void SaveToJson(rapidjson::Value& v, rapidjson::Document::AllocatorType& alloc) const = 0;
    virtual void SaveToJson(rapidjson::Document& d) const {
        SaveToJson(d, d.GetAllocator());
    }
};

//store a json value to local data structures
//for compatibility with ptree, type conversion is made. will remove them in the future
inline void LoadJsonValue(const rapidjson::Value& v, JsonSerializable& t) {
    t.LoadFromJson(v);
}

inline void LoadJsonValue(const rapidjson::Value& v, std::string& t) {
    if (v.IsString()) {
        t = v.GetString();
    } else if (v.IsInt64()) {
        //TODO: add warnings on all usages of lexical_cast
        t = boost::lexical_cast<std::string>(v.GetInt64());
    } else {
        throw OpenraveJSONException("Cannot convert json value " + GetJsonString(v) + " to String", MJE_Failed);
    }

}

inline void LoadJsonValue(const rapidjson::Value& v, int& t) {
    if (v.IsInt()) {
        t = v.GetInt();
    } else if (v.IsString()) {
        t = boost::lexical_cast<int>(v.GetString());
    } else {
        throw OpenraveJSONException("Cannot convert json value " + GetJsonString(v) + " to Int", MJE_Failed);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, unsigned int& t) {
    if (v.IsUint()) {
        t = v.GetUint();
    } else if (v.IsString()) {
        t = boost::lexical_cast<unsigned int>(v.GetString());
    } else {
        throw OpenraveJSONException("Cannot convert json value " + GetJsonString(v) + " to Int", MJE_Failed);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, unsigned long long& t) {
    if (v.IsUint64()) {
        t = v.GetUint64();
    } else if (v.IsString()) {
        t = boost::lexical_cast<unsigned long long>(v.GetString());
    } else {
        throw OpenraveJSONException("Cannot convert json value " + GetJsonString(v) + " to Int64", MJE_Failed);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, uint64_t& t) {
    if (v.IsUint64()) {
        t = v.GetUint64();
    } else if (v.IsString()) {
        t = boost::lexical_cast<uint64_t>(v.GetString());
    } else {
        throw OpenraveJSONException("Cannot convert json value " + GetJsonString(v) + " to Int64", MJE_Failed);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, int64_t& t) {
    if (v.IsInt64()) {
        t = v.GetInt64();
    } else if (v.IsString()) {
        t = boost::lexical_cast<int64_t>(v.GetString());
    } else {
        throw OpenraveJSONException("Cannot convert json type " + GetJsonString(v) + " to Int64", MJE_Failed);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, double& t) {
    if (v.IsNumber()) {
        t = v.GetDouble();
    } else if (v.IsString()) {
        t = boost::lexical_cast<double>(v.GetString());
    } else {
        throw OpenraveJSONException("Cannot convert json value " + GetJsonString(v) + " to Double", MJE_Failed);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, float& t) {
    if (v.IsNumber()) {
        t = v.GetDouble();
    } else if (v.IsString()) {
        t = boost::lexical_cast<double>(v.GetString());
    } else {
        throw OpenraveJSONException("Cannot convert json value " + GetJsonString(v) + " to Double", MJE_Failed);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, bool& t) {
    if (v.IsInt()) t = v.GetInt();
    else if (v.IsBool()) t = v.GetBool();
    else if (v.IsString())  {
        t = boost::lexical_cast<bool>(v.GetString());
    } else {
        throw OpenraveJSONException("Cannot convert json value " + GetJsonString(v) + " to Bool", MJE_Failed);
    }
}

template<class T> inline void LoadJsonValue(const rapidjson::Value& v, std::vector<T>& t);

template<class T, size_t N> inline void LoadJsonValue(const rapidjson::Value& v, std::array<T, N>& t);

template<class T> inline void LoadJsonValue(const rapidjson::Value& v, boost::shared_ptr<T>& ptr) {
    // this way prevents copy constructor from getting called
    ptr = boost::shared_ptr<T>(new T());
    LoadJsonValue(v, *ptr);
}

template<class T> inline void LoadJsonValue(const rapidjson::Value& v, std::pair<std::string, T>& t) {
    if (v.IsArray()) {
        if (v.GetArray().Size() == 2) {
            LoadJsonValue(v[0], t.first);
            LoadJsonValue(v[1], t.second);
        } else {
            throw OpenraveJSONException("List-based map has entry with size != 2", MJE_Failed);
        }
    } else {
        throw OpenraveJSONException("Cannot convert json type " + GetJsonTypeName(v) + " to Pair", MJE_Failed);
    }
}

template<class T, size_t N> inline void LoadJsonValue(const rapidjson::Value& v, T (&p)[N]) {
    if (v.IsArray()) {
        if (v.GetArray().Size() != N) {
            throw OpenraveJSONException("Json array size doesn't match", MJE_Failed);
        }
        size_t i = 0;
        for (rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
            LoadJsonValue(*it, p[i]);
            i++;
        }
    } else {
        throw OpenraveJSONException("Cannot convert json type " + GetJsonTypeName(v) + " to Array", MJE_Failed);
    }
}

template<class T> inline void LoadJsonValue(const rapidjson::Value& v, std::vector<T>& t) {
    if (v.IsArray()) {
        t.clear();
        t.resize(v.GetArray().Size());
        size_t i = 0;
        for (rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
            LoadJsonValue(*it, t[i]);
            i++;
        }
    } else {
        throw OpenraveJSONException("Cannot convert json type " + GetJsonTypeName(v) + " to Array", MJE_Failed);
    }
}

template<class T, size_t N> inline void LoadJsonValue(const rapidjson::Value& v, std::array<T, N>& t)
{
    if (v.IsArray()) {
        if (v.GetArray().Size() != N) {
            throw OpenraveJSONException((boost::format("Cannot convert json type " + GetJsonTypeName(v) + " to Array. Array length does not match (%d != %d)") % N % v.GetArray().Size()).str(), MJE_Failed);
        }
        size_t i = 0;
        for (rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
            LoadJsonValue(*it, t[i]);
            i++;
        }
    }
    else {
        throw OpenraveJSONException("Cannot convert json type " + GetJsonTypeName(v) + " to Array", MJE_Failed);
    }
}

template<class U> inline void LoadJsonValue(const rapidjson::Value& v, std::map<std::string, U>& t) {
    if (v.IsArray()) {
        // list based map
        // TODO: is it dangerous?
        for (rapidjson::Value::ConstValueIterator itr = v.Begin(); itr != v.End(); ++itr) {
            std::pair<std::string, U> value;
            LoadJsonValue((*itr), value);
            t[value.first] = value.second;
        }
    } else if (v.IsObject()) {
        t.clear();
        U value;
        for (rapidjson::Value::ConstMemberIterator it = v.MemberBegin();
             it != v.MemberEnd(); ++it) {
            LoadJsonValue(it->value, value);
            t[it->name.GetString()] = value;
        }
    } else {
        throw OpenraveJSONException("Cannot convert json type " + GetJsonTypeName(v) + " to Map", MJE_Failed);
    }
}

//Save a data structure to rapidjson::Value format

/*template<class T> inline void SaveJsonValue(rapidjson::Value& v, const T& t, rapidjson::Document::AllocatorType& alloc) {*/
/*JsonWrapper<T>::SaveToJson(v, t, alloc);*/
/*}*/

inline void SaveJsonValue(rapidjson::Value& v, const JsonSerializable& t, rapidjson::Document::AllocatorType& alloc) {
    t.SaveToJson(v, alloc);
}

inline void SaveJsonValue(rapidjson::Value& v, const std::string& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetString(t.c_str(), alloc);
}

inline void SaveJsonValue(rapidjson::Value& v, const char* t, rapidjson::Document::AllocatorType& alloc) {
    v.SetString(t, alloc);
}

inline void SaveJsonValue(rapidjson::Value& v, int t, rapidjson::Document::AllocatorType& alloc) {
    v.SetInt(t);
}

inline void SaveJsonValue(rapidjson::Value& v, unsigned int t, rapidjson::Document::AllocatorType& alloc) {
    v.SetUint(t);
}

inline void SaveJsonValue(rapidjson::Value& v, long long t, rapidjson::Document::AllocatorType& alloc) {
    v.SetInt64(t);
}

inline void SaveJsonValue(rapidjson::Value& v, int64_t t, rapidjson::Document::AllocatorType& alloc) {
    v.SetInt64(t);
}

inline void SaveJsonValue(rapidjson::Value& v, unsigned long long t, rapidjson::Document::AllocatorType& alloc) {
    v.SetUint64(t);
}

inline void SaveJsonValue(rapidjson::Value& v, uint64_t t, rapidjson::Document::AllocatorType& alloc) {
    v.SetUint64(t);
}

inline void SaveJsonValue(rapidjson::Value& v, bool t, rapidjson::Document::AllocatorType& alloc) {
    v.SetBool(t);
}

inline void SaveJsonValue(rapidjson::Value& v, double t, rapidjson::Document::AllocatorType& alloc) {
    v.SetDouble(t);
}

inline void SaveJsonValue(rapidjson::Value& v, float t, rapidjson::Document::AllocatorType& alloc) {
    v.SetDouble(t);
}

inline void SaveJsonValue(rapidjson::Value& v, const rapidjson::Value& t, rapidjson::Document::AllocatorType& alloc) {
    v.CopyFrom(t, alloc);
}

template<class T> inline void SaveJsonValue(rapidjson::Value& v, const std::vector<T>& t, rapidjson::Document::AllocatorType& alloc);

template<class T, size_t N> inline void SaveJsonValue(rapidjson::Value& v, const std::array<T, N>& t, rapidjson::Document::AllocatorType& alloc);

/** do not remove: otherwise boost::shared_ptr could be treated as bool
 */
template<class T> inline void SaveJsonValue(rapidjson::Value& v, const boost::shared_ptr<T>& ptr, rapidjson::Document::AllocatorType& alloc) {
    SaveJsonValue(v, *ptr, alloc);
}

template<class T> inline void SaveJsonValue(rapidjson::Value& v, const std::vector<T>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for (size_t i = 0; i < t.size(); ++i) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[i], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<class T, size_t N> inline void SaveJsonValue(rapidjson::Value& v, const std::array<T, N>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(N, alloc);
    for (size_t i = 0; i < N; ++i) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[i], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<> inline void SaveJsonValue(rapidjson::Value& v, const std::vector<double>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for (size_t i = 0; i < t.size(); ++i) {
        v.PushBack(t[i], alloc);
    }
}

template<size_t N> inline void SaveJsonValue(rapidjson::Value& v, const double (&t)[N], rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(N, alloc);
    for (size_t i = 0; i < N; ++i) {
        v.PushBack(t[i], alloc);
    }
}

template<size_t N> inline void SaveJsonValue(rapidjson::Value& v, const std::array<double, N>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(N, alloc);
    for (size_t i = 0; i < N; ++i) {
        v.PushBack(t[i], alloc);
    }
}

template<class U> inline void SaveJsonValue(rapidjson::Value& v, const std::map<std::string, U>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetObject();
    for (typename std::map<std::string, U>::const_iterator it = t.begin(); it != t.end(); ++it) {
        rapidjson::Value name, value;
        SaveJsonValue(name, it->first, alloc);
        SaveJsonValue(value, it->second, alloc);
        v.AddMember(name, value, alloc);
    }
}

template<class T> inline void SaveJsonValue(rapidjson::Document& v, const T& t) {
    // rapidjson::Value::CopyFrom also doesn't free up memory, need to clear memory
    // see note in: void ParseJson(rapidjson::Document& d, const std::string& str)
    rapidjson::Document tempDoc;
    SaveJsonValue(tempDoc, t, tempDoc.GetAllocator());
    v.Swap(tempDoc);
}

template<class T, class U> inline void SetJsonValueByKey(rapidjson::Value& v, const U& key, const T& t, rapidjson::Document::AllocatorType& alloc);

//get one json value by key, and store it in local data structures
template<class T> void inline LoadJsonValueByKey(const rapidjson::Value& v, const char* key, T& t) {
    if (!v.IsObject()) {
        throw OpenraveJSONException("Cannot load value of non-object.", MJE_Failed);
    }
    if (v.HasMember(key)) {
        LoadJsonValue(v[key], t);
    }
}
template<class T, class U> inline void LoadJsonValueByKey(const rapidjson::Value& v, const char* key, T& t, const U& d) {
    if (!v.IsObject()) {
        throw OpenraveJSONException("Cannot load value of non-object.", MJE_Failed);
    }
    if (v.HasMember(key)) {
        LoadJsonValue(v[key], t);
    }
    else {
        t = d;
    }
}

//work the same as LoadJsonValueByKey, but the value is returned instead of being passed as reference
template<class T, class U> T GetJsonValueByKey(const rapidjson::Value& v, const char* key, const U& t) {
    if (!v.IsObject()) {
        throw OpenraveJSONException("Cannot get value of non-object.", MJE_Failed);
    }
    if (v.HasMember(key)) {
        const rapidjson::Value& child = v[key];
        if (!child.IsNull()) {
            T r;
            LoadJsonValue(v[key], r);
            return r;
        }
    }
    return T(t);
}
template<class T> inline T GetJsonValueByKey(const rapidjson::Value& v, const char* key) {
    if (!v.IsObject()) {
        throw OpenraveJSONException("Cannot load value of non-object.", MJE_Failed);
    }
    T r = T();
    if (v.HasMember(key)) {
        const rapidjson::Value& child = v[key];
        if (!child.IsNull()) {
            LoadJsonValue(v[key], r);
        }
    }
    return r;
}

inline std::string GetStringJsonValueByKey(const rapidjson::Value& v, const char* key, const std::string& defaultValue=std::string()) {
    return GetJsonValueByKey<std::string, std::string>(v, key, defaultValue);
}

template<class T> inline T GetJsonValueByPath(const rapidjson::Value& v, const char* key) {
    T r;
    const rapidjson::Value *child = rapidjson::Pointer(key).Get(v);
    if (child && !child->IsNull()) {
        LoadJsonValue(*child, r);
    }
    return r;
}

template<class T, class U> T GetJsonValueByPath(const rapidjson::Value& v, const char* key, const U& t) {
    const rapidjson::Value *child = rapidjson::Pointer(key).Get(v);
    if (child && !child->IsNull()) {
        T r;
        LoadJsonValue(*child, r);
        return r;
    }
    else {
        return T(t);
    }
}

template<class T> inline void SetJsonValueByPath(rapidjson::Document& d, const char* path, const T& t) {
    rapidjson::Value v;
    SaveJsonValue(v, t, d.GetAllocator());
    rapidjson::Pointer(path).Swap(d, v, d.GetAllocator());
}

template<class T, class U> inline void SetJsonValueByKey(rapidjson::Value& v, const U& key, const T& t, rapidjson::Document::AllocatorType& alloc)
{
    if (!v.IsObject()) {
        throw OpenraveJSONException("Cannot set value for non-object.", MJE_Failed);
    }
    if (v.HasMember(key)) {
        SaveJsonValue(v[key], t, alloc);
    }
    else {
        rapidjson::Value value, name;
        SaveJsonValue(name, key, alloc);
        SaveJsonValue(value, t, alloc);
        v.AddMember(name, value, alloc);
    }
}

template<class T>
inline void SetJsonValueByKey(rapidjson::Document& d, const char* key, const T& t)
{
    SetJsonValueByKey(d, key, t, d.GetAllocator());
}

template<class T>
inline void SetJsonValueByKey(rapidjson::Document& d, const std::string& key, const T& t)
{
    SetJsonValueByKey(d, key.c_str(), t, d.GetAllocator());
}

inline void ValidateJsonString(const std::string& str) {
    rapidjson::Document d;
    if (d.Parse(str.c_str()).HasParseError()) {
        throw OpenraveJSONException("json string " + str + " is invalid." + GetParseError_En(d.GetParseError()), MJE_Failed);
    }
}

template<class T> inline std::string GetJsonString(const T& t) {
    rapidjson::Document d;
    SaveJsonValue(d, t);
    return DumpJson(d);
}

} // namespace openravejson
#endif
