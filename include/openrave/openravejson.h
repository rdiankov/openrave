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

#include <openrave/config.h>
#include <openrave/openraveexception.h>

#include <array>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <stdint.h>
#include <string>
#include <stdexcept>
#include <vector>
#include <map>
#include <iostream>

#include <rapidjson/document.h>
#include <rapidjson/pointer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/writer.h>
#include <rapidjson/error/en.h>
#include <rapidjson/prettywriter.h>

namespace OpenRAVE {

namespace orjson {

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

/// \brief dump json to a std::string
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

/// \brief dump json to ostream
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

class VectorWrapper {
public:
    typedef char Ch;

    VectorWrapper(std::vector<Ch>& v) : _v(v) { }

    Ch Peek() const { BOOST_ASSERT(0); return '\0'; }
    Ch Take() { BOOST_ASSERT(0); return '\0'; }
    size_t Tell() const { return _v.size(); }
 
    Ch* PutBegin() { BOOST_ASSERT(0); return 0; }
    void Put(Ch c) { _v.push_back(c); }
    void Flush() { }
    size_t PutEnd(Ch*) { BOOST_ASSERT(0); return 0; }
 
private:
    VectorWrapper(const VectorWrapper&);
    VectorWrapper& operator=(const VectorWrapper&);
 
    std::vector<Ch>& _v;
};

/// \brief dump json to vector buffer
inline void DumpJson(const rapidjson::Value& value, std::vector<char>& output, const unsigned int indent=0) {
    VectorWrapper wrapper(output);
    if (indent == 0) {
        rapidjson::Writer<VectorWrapper> writer(wrapper);
        value.Accept(writer);
    } else {
        rapidjson::PrettyWriter<VectorWrapper> writer(wrapper);
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
        throw OPENRAVE_EXCEPTION_FORMAT("JSON string is invalid (offset %u) %s str=%s", ((unsigned)d.GetErrorOffset())%GetParseError_En(d.GetParseError())%substr, OpenRAVE::ORE_InvalidArguments);
    }
    tempDoc.Swap(d);
}

inline void ParseJson(rapidjson::Document& d, std::istream& is) {
    rapidjson::IStreamWrapper isw(is);
    // see note in: void ParseJson(rapidjson::Document& d, const std::string& str)
    rapidjson::Document(tempDoc);
    tempDoc.ParseStream<rapidjson::kParseFullPrecisionFlag>(isw); // parse float in full precision mode
    if (tempDoc.HasParseError()) {
        throw OPENRAVE_EXCEPTION_FORMAT("JSON stream is invalid (offset %u) %s", ((unsigned)tempDoc.GetErrorOffset())%GetParseError_En(tempDoc.GetParseError()), OpenRAVE::ORE_InvalidArguments);
    }
    tempDoc.Swap(d);
}

class JsonSerializable {
public:
    virtual void LoadFromJson(const rapidjson::Value& v) = 0;
    virtual void SaveToJson(rapidjson::Value& v, rapidjson::Document::AllocatorType& alloc) const = 0;
    virtual void SaveToJson(rapidjson::Document& d) const {
        SaveToJson(d, d.GetAllocator());
    }
};

template<class T> inline std::string GetJsonString(const T& t);

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
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to String", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, int& t) {
    if (v.IsInt()) {
        t = v.GetInt();
    } else if (v.IsString()) {
        t = boost::lexical_cast<int>(v.GetString());
    } else if (v.IsBool()) {
        t = v.GetBool() ? 1 : 0;
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Int", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, int8_t& t) {
    if (v.IsInt()) {
        t = v.GetInt();
    }
    else if (v.IsUint()) {
        t = v.GetUint();
    }
    else if (v.IsString()) {
        t = boost::lexical_cast<unsigned int>(v.GetString());
    }
    else if (v.IsBool()) {
        t = v.GetBool() ? 1 : 0;
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert json type %s to Int", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, uint8_t& t) {
    if (v.IsUint()) {
        t = v.GetUint();
    } else if (v.IsString()) {
        t = boost::lexical_cast<unsigned int>(v.GetString());
    } else if (v.IsBool()) {
        t = v.GetBool() ? 1 : 0;
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Int", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, unsigned long long& t) {
    if (v.IsUint64()) {
        t = v.GetUint64();
    } else if (v.IsString()) {
        t = boost::lexical_cast<unsigned long long>(v.GetString());
    } else if (v.IsBool()) {
        t = v.GetBool() ? 1 : 0;
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Int64", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, uint64_t& t) {
    if (v.IsUint64()) {
        t = v.GetUint64();
    } else if (v.IsString()) {
        t = boost::lexical_cast<uint64_t>(v.GetString());
    } else if (v.IsBool()) {
        t = v.GetBool() ? 1 : 0;
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to UInt64", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, int64_t& t) {
    if (v.IsInt64()) {
        t = v.GetInt64();
    } else if (v.IsString()) {
        t = boost::lexical_cast<int64_t>(v.GetString());
    } else if (v.IsBool()) {
        t = v.GetBool() ? 1 : 0;
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Int64", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, double& t) {
    if (v.IsNumber()) {
        t = v.GetDouble();
    } else if (v.IsString()) {
        t = boost::lexical_cast<double>(v.GetString());
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert json JSON %s to Double", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, float& t) {
    if (v.IsNumber()) {
        t = v.GetDouble();
    } else if (v.IsString()) {
        t = boost::lexical_cast<double>(v.GetString());
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Double", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, bool& t) {
    if (v.IsInt()) t = v.GetInt();
    else if (v.IsBool()) t = v.GetBool();
    else if (v.IsString())  {
        t = boost::lexical_cast<bool>(v.GetString());
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Bool", GetJsonString(v), OpenRAVE::ORE_InvalidArguments);
    }
}

template<class T>
inline void LoadJsonValue(const rapidjson::Value& v, OpenRAVE::RaveVector<T>& t) {
    if(!v.IsArray() || (v.Size() != 3 && v.Size() != 4)) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a RaveVector", OpenRAVE::ORE_InvalidArguments);
    }

    LoadJsonValue(v[0], t[0]);
    LoadJsonValue(v[1], t[1]);
    LoadJsonValue(v[2], t[2]);
    if (v.Size() == 4)
    {
        LoadJsonValue(v[3], t[3]);
    }
    else {
        t[3] = 0; // have to reset
    }
}

template<class T>
inline void LoadJsonValue(const rapidjson::Value& v, boost::shared_ptr<T>& ptr) {
    // this way prevents copy constructor from getting called
    ptr = boost::shared_ptr<T>(new T());
    LoadJsonValue(v, *ptr);
}

inline void LoadJsonValue(const rapidjson::Value& v, OpenRAVE::IkParameterization& t) {
    t.DeserializeJSON(v);
}

/// \brief serialize an OpenRAVE IkParameterization as json
inline void SaveJsonValue(rapidjson::Value &v, const OpenRAVE::IkParameterization& t, rapidjson::Document::AllocatorType& alloc) {
    t.SerializeJSON(v, alloc);
}

template<class T, size_t N>
inline void LoadJsonValue(const rapidjson::Value& v, boost::array<T, N>& t) {
    if (v.IsArray()) {
        for (std::size_t ivalue = 0; ivalue < v.Size(); ivalue++) {
            LoadJsonValue(v[ivalue], t[ivalue]);
        }
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Array", GetJsonTypeName(v), OpenRAVE::ORE_InvalidArguments);
    }
}

template<class T, size_t N>
inline void LoadJsonValue(const rapidjson::Value& v, T (&p)[N]) {
    if (v.IsArray()) {
        if (v.GetArray().Size() != N) {
            throw OPENRAVE_EXCEPTION_FORMAT0("JSON array size doesn't match", OpenRAVE::ORE_InvalidArguments);
        }
        size_t i = 0;
        for (rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
            LoadJsonValue(*it, p[i]);
            i++;
        }
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Array", GetJsonTypeName(v), OpenRAVE::ORE_InvalidArguments);
    }
}

template<class T> inline void LoadJsonValue(const rapidjson::Value& v, std::vector<T>& t);
template<class T, class U>
inline void LoadJsonValue(const rapidjson::Value& v, std::pair<T, U>& t) {
    if (v.IsArray()) {
        if (v.GetArray().Size() == 2) {
            LoadJsonValue(v[0], t.first);
            LoadJsonValue(v[1], t.second);
        } else {
            throw OPENRAVE_EXCEPTION_FORMAT0("List-based map has entry with size != 2", OpenRAVE::ORE_InvalidArguments);
        }
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Pair", GetJsonTypeName(v), OpenRAVE::ORE_InvalidArguments);
    }
}

template<class T>
inline void LoadJsonValue(const rapidjson::Value& v, std::vector<T>& t) {
    if (v.IsArray()) {
        t.clear();
        t.resize(v.GetArray().Size());
        size_t i = 0;
        for (rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
            LoadJsonValue(*it, t[i]);
            i++;
        }
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Array", GetJsonTypeName(v), OpenRAVE::ORE_InvalidArguments);
    }
}

template<class T, size_t N>
inline void LoadJsonValue(const rapidjson::Value& v, std::array<T, N>& t) {
    if (v.IsArray()) {
        size_t i = 0;
        for (rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
            LoadJsonValue(*it, t[i]);
            i++;
        }
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Array", GetJsonTypeName(v), OpenRAVE::ORE_InvalidArguments);
    }
}


template<class T>
inline void LoadJsonValue(const rapidjson::Value& v, std::set<T>& t) {
    if (!v.IsArray()) {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to std::set", GetJsonTypeName(v), OpenRAVE::ORE_InvalidArguments);
    }
    for(rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
        T temp;
        LoadJsonValue(*it, temp);
        t.insert(temp);
    }
}

template<class T>
inline void LoadJsonValue(const rapidjson::Value& v, std::map<std::string, T>& t) {
    if (v.IsArray()) {
        // list based map
        // TODO: is it dangerous?
        for (rapidjson::Value::ConstValueIterator itr = v.Begin(); itr != v.End(); ++itr) {
            std::pair<std::string, T> value;
            LoadJsonValue((*itr), value);
            t[value.first] = value.second;
        }
    } else if (v.IsObject()) {
        t.clear();
        for (rapidjson::Value::ConstMemberIterator it = v.MemberBegin(); it != v.MemberEnd(); ++it) {
            T value;
            LoadJsonValue(it->value, value);
            t[it->name.GetString()] = value;
        }
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Map", GetJsonTypeName(v), OpenRAVE::ORE_InvalidArguments);
    }
}

template<class T>
inline void LoadJsonValue(const rapidjson::Value& v, OpenRAVE::RaveTransform<T>& t) {
    if (v.IsArray()) {
        if(v.Size() != 7) {
            throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to Transform. Array length does not match (%d != 7)", GetJsonTypeName(v)%v.Size(), OpenRAVE::ORE_InvalidArguments);
        }
        LoadJsonValue(v[0], t.rot.x);
        LoadJsonValue(v[1], t.rot.y);
        LoadJsonValue(v[2], t.rot.z);
        LoadJsonValue(v[3], t.rot.w);
        LoadJsonValue(v[4], t.trans.x);
        LoadJsonValue(v[5], t.trans.y);
        LoadJsonValue(v[6], t.trans.z);
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot convert JSON type %s to RaveTransform", GetJsonTypeName(v), OpenRAVE::ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, OpenRAVE::TriMesh& t)
{
    if (!v.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Cannot load value of non-object.", OpenRAVE::ORE_InvalidArguments);
    }

    if (!v.HasMember("vertices") || !v["vertices"].IsArray() || v["vertices"].Size() % 3 != 0) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a TriMesh, \"vertices\" malformatted", OpenRAVE::ORE_InvalidArguments);
    }

    t.vertices.clear();
    t.vertices.reserve(v["vertices"].Size() / 3);

    for (rapidjson::Value::ConstValueIterator it = v["vertices"].Begin(); it != v["vertices"].End(); ) {
        OpenRAVE::Vector vertex;
        LoadJsonValue(*(it++), vertex.x);
        LoadJsonValue(*(it++), vertex.y);
        LoadJsonValue(*(it++), vertex.z);
        t.vertices.push_back(vertex);
    }
    LoadJsonValue(v["indices"], t.indices);
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

/** do not remove: otherwise boost::shared_ptr could be treated as bool
 */
template<class T>
inline void SaveJsonValue(rapidjson::Value& v, const boost::shared_ptr<T>& ptr, rapidjson::Document::AllocatorType& alloc) {
    SaveJsonValue(v, *ptr, alloc);
}

template<class T, class U>
inline void SaveJsonValue(rapidjson::Value& v, const std::pair<T, U>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(2, alloc);
    rapidjson::Value first, second;
    SaveJsonValue(first, t.first, alloc);
    SaveJsonValue(second, t.second, alloc);
    v.PushBack(first, alloc);
    v.PushBack(second, alloc);
}

template<class T>
inline void SaveJsonValue(rapidjson::Value& v, const std::set<T>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for(typename std::set<T>::const_iterator it = t.begin(); it != t.end(); it++) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, *it, alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<class T>
inline void SaveJsonValue(rapidjson::Value& rTransform, const OpenRAVE::RaveTransform<T>& t, rapidjson::Document::AllocatorType& alloc) {
    rTransform.SetArray();
    rTransform.Reserve(7, alloc);
    rTransform.PushBack(t.rot[0], alloc);
    rTransform.PushBack(t.rot[1], alloc);
    rTransform.PushBack(t.rot[2], alloc);
    rTransform.PushBack(t.rot[3], alloc);
    rTransform.PushBack(t.trans[0], alloc);
    rTransform.PushBack(t.trans[1], alloc);
    rTransform.PushBack(t.trans[2], alloc);
}

template<class T>
inline void SaveJsonValue(rapidjson::Value& v, const std::vector<T>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for (size_t ivec = 0; ivec < t.size(); ++ivec) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[ivec], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<class T>
inline void SaveJsonValue(rapidjson::Value& v, const std::vector<T>& t, rapidjson::Document::AllocatorType& alloc, size_t n) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for (size_t ivec = 0; ivec < t.size() && ivec < n; ++ivec) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[ivec], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<class T>
inline void SaveJsonValue(rapidjson::Value& v, const OpenRAVE::RaveVector<T>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    // TODO, what's a better way to serialize?
    bool bHas4Values = t[3] != 0;
    int numvalues = bHas4Values ? 4 : 3;
    v.Reserve(numvalues, alloc);
    for (int ivalue = 0; ivalue < numvalues; ++ivalue) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[ivalue], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<class T, size_t N>
inline void SaveJsonValue(rapidjson::Value& v, const std::array<T, N>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(N, alloc);
    for (size_t iarray = 0; iarray < N; ++iarray) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[iarray], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<class T, size_t N>
inline void SaveJsonValue(rapidjson::Value& v, const std::array<T, N>& t, rapidjson::Document::AllocatorType& alloc, size_t n) {
    v.SetArray();
    v.Reserve(N, alloc);
    for (size_t iarray = 0; iarray < N && iarray < n; ++iarray) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[iarray], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<class T, size_t N>
inline void SaveJsonValue(rapidjson::Value& v, const boost::array<T, N>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(N, alloc);
    for(size_t iarray = 0; iarray < t.size(); iarray++) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[iarray], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<class T, size_t N>
inline void SaveJsonValue(rapidjson::Value& v, const boost::array<T, N>& t, rapidjson::Document::AllocatorType& alloc, size_t n) {
    v.SetArray();
    v.Reserve(N, alloc);
    for(size_t iarray = 0; iarray < N && iarray < n; ++iarray) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[iarray], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<>
inline void SaveJsonValue(rapidjson::Value& v, const std::vector<double>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for (size_t ivec = 0; ivec < t.size(); ++ivec) {
        v.PushBack(t[ivec], alloc);
    }
}
template<>
inline void SaveJsonValue(rapidjson::Value& v, const std::vector<double>& t, rapidjson::Document::AllocatorType& alloc, size_t n) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for (size_t ivec = 0; ivec < t.size() && ivec < n; ++ivec) {
        v.PushBack(t[ivec], alloc);
    }
}

template<size_t N>
inline void SaveJsonValue(rapidjson::Value& v, const double (&t)[N], rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(N, alloc);
    for (size_t iarray = 0; iarray < N; ++iarray) {
        v.PushBack(t[iarray], alloc);
    }
}

template<size_t N>
inline void SaveJsonValue(rapidjson::Value& v, const std::array<double, N>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(N, alloc);
    for (size_t i = 0; i < N; ++i) {
        v.PushBack(t[i], alloc);
    }
}

template<class T>
inline void SaveJsonValue(rapidjson::Value& v, const std::map<std::string, T>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetObject();
    for (typename std::map<std::string, T>::const_iterator it = t.begin(); it != t.end(); ++it) {
        rapidjson::Value name, value;
        SaveJsonValue(name, it->first, alloc);
        SaveJsonValue(value, it->second, alloc);
        v.AddMember(name, value, alloc);
    }
}

template<class T>
inline void SaveJsonValue(rapidjson::Document& v, const T& t) {
    // rapidjson::Value::CopyFrom also doesn't free up memory, need to clear memory
    // see note in: void ParseJson(rapidjson::Document& d, const std::string& str)
    rapidjson::Document tempDoc;
    SaveJsonValue(tempDoc, t, tempDoc.GetAllocator());
    v.Swap(tempDoc);
}
template<class T> void inline LoadJsonValueByKey(const rapidjson::Value& v, const char* key, T& t);
inline void LoadJsonValue(const rapidjson::Value& v, OpenRAVE::SensorBase::CameraIntrinsics& t) {
    if (!v.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Cannot load value of non-object to SensorBase::CameraIntrinsics.", OpenRAVE::ORE_InvalidArguments);
    }
    LoadJsonValueByKey(v, "fx", t.fx);
    LoadJsonValueByKey(v, "fy", t.fy);
    LoadJsonValueByKey(v, "cx", t.cx);
    LoadJsonValueByKey(v, "cy", t.cy);
    LoadJsonValueByKey(v, "focalLength", t.focal_length);
    LoadJsonValueByKey(v, "distortionModel", t.distortion_model);
    LoadJsonValueByKey(v, "distortionCoeffs", t.distortion_coeffs);
}

//get one json value by key, and store it in local data structures
template<class T>
void inline LoadJsonValueByKey(const rapidjson::Value& v, const char* key, T& t) {
    if (!v.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Cannot load value of non-object.", OpenRAVE::ORE_InvalidArguments);
    }
    if (v.HasMember(key)) {
        LoadJsonValue(v[key], t);
    }
}

template<class T, class U>
inline void LoadJsonValueByKey(const rapidjson::Value& v, const char* key, T& t, const U& d) {
    if (!v.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Cannot load value of non-object.", OpenRAVE::ORE_InvalidArguments);
    }
    if (v.HasMember(key)) {
        LoadJsonValue(v[key], t);
    }
    else {
        t = d;
    }
}

//work the same as LoadJsonValueByKey, but the value is returned instead of being passed as reference
template<class T, class U>
T GetJsonValueByKey(const rapidjson::Value& v, const char* key, const U& t) {
    if (!v.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Cannot get value of non-object.", OpenRAVE::ORE_InvalidArguments);
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

template<class T>
inline T GetJsonValueByKey(const rapidjson::Value& v, const char* key) {
    if (!v.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Cannot load value of non-object.", OpenRAVE::ORE_InvalidArguments);
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

template<class T>
inline T GetJsonValueByPath(const rapidjson::Value& v, const char* key) {
    T r;
    const rapidjson::Value *child = rapidjson::Pointer(key).Get(v);
    if (child && !child->IsNull()) {
        LoadJsonValue(*child, r);
    }
    return r;
}

template<class T, class U>
T GetJsonValueByPath(const rapidjson::Value& v, const char* key, const U& t) {
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

template<class T, class U>
inline void SetJsonValueByKey(rapidjson::Value& v, const U& key, const T& t, rapidjson::Document::AllocatorType& alloc);
/// \brief serialize an OpenRAVE CameraIntrinsics as json
inline void SaveJsonValue(rapidjson::Value& v, const OpenRAVE::SensorBase::CameraIntrinsics& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetObject();
    SetJsonValueByKey(v, "fx", t.fx, alloc);
    SetJsonValueByKey(v, "fy", t.fy, alloc);
    SetJsonValueByKey(v, "cx", t.cx, alloc);
    SetJsonValueByKey(v, "cy", t.cy, alloc);
    SetJsonValueByKey(v, "focalLength", t.focal_length, alloc);
    SetJsonValueByKey(v, "distortionModel", t.distortion_model, alloc);
    SetJsonValueByKey(v, "distortionCoeffs", t.distortion_coeffs, alloc);
}

inline void SaveJsonValue(rapidjson::Value &rTriMesh, const OpenRAVE::TriMesh& t, rapidjson::Document::AllocatorType& alloc) {
    rTriMesh.SetObject();
    rapidjson::Value rVertices;
    rVertices.SetArray();
    rVertices.Reserve(t.vertices.size()*3, alloc);
    for(size_t ivertex = 0; ivertex < t.vertices.size(); ++ivertex) {
        rVertices.PushBack(t.vertices[ivertex][0], alloc);
        rVertices.PushBack(t.vertices[ivertex][1], alloc);
        rVertices.PushBack(t.vertices[ivertex][2], alloc);
    }
    rTriMesh.AddMember("vertices", rVertices, alloc);
    SetJsonValueByKey(rTriMesh, "indices", t.indices, alloc);
}

template<class T, class U>
inline void SetJsonValueByKey(rapidjson::Value& v, const U& key, const T& t, rapidjson::Document::AllocatorType& alloc)
{
    if (!v.IsObject()) {
        v.SetObject();
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

template<class T, class U>
inline void SetJsonValueByKey(rapidjson::Value& v, const U& key, const T& t, rapidjson::Document::AllocatorType& alloc, size_t n) {
    if (!v.IsObject()) {
        v.SetObject();
    }
    if (v.HasMember(key)) {
        SaveJsonValue(v[key], t, alloc, n);
    }
    else {
        rapidjson::Value value, name;
        SaveJsonValue(name, key, alloc);
        SaveJsonValue(value, t, alloc, n);
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

template<class T>
inline void SetJsonValueByPath(rapidjson::Document& d, const char* path, const T& t) {
    rapidjson::Value v;
    SaveJsonValue(v, t, d.GetAllocator());
    rapidjson::Pointer(path).Swap(d, v, d.GetAllocator());
}

inline void ValidateJsonString(const std::string& str) {
    rapidjson::Document d;
    if (d.Parse(str.c_str()).HasParseError()) {
        throw OPENRAVE_EXCEPTION_FORMAT("JSON string %s is invalid: %s", str%GetParseError_En(d.GetParseError()), OpenRAVE::ORE_InvalidArguments);
    }
}

template<class T> inline std::string GetJsonString(const T& t) {
    rapidjson::Document d;
    SaveJsonValue(d, t);
    return OpenRAVE::orjson::DumpJson(d);
}

/** update a json object with another one, new key-value pair will be added, existing ones will be overwritten
 */
inline void UpdateJson(rapidjson::Document& a, const rapidjson::Value& b) {
    if (!a.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT("json object should be a dict to be updated: %s", GetJsonString(a), OpenRAVE::ORE_InvalidArguments);
    }
    if (!b.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT("json object should be a dict to update another dict: %s", GetJsonString(b), OpenRAVE::ORE_InvalidArguments);
    }
    for (rapidjson::Value::ConstMemberIterator it = b.MemberBegin(); it != b.MemberEnd(); ++it) {
        SetJsonValueByKey(a, it->name.GetString(), it->value);
    }
}

} // namespace JSON

} // namespace OpenRAVE

#endif // OPENRAVE_JSON_H
