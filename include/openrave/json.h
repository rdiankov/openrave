// -*- coding: utf-8 -*-
// Copyright (C) 2006-2016 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file json.h
    \brief Defines json serialization and deserailization related utilities. This file is automatically included by openrave.h.
 */
#ifndef OPENRAVE_SERIALIZE_JSON_H
#define OPENRAVE_SERIALIZE_JSON_H

namespace OpenRAVE {

#define RAVE_SERIALIZEJSON(value, allocator, ...) rapidjson::Value value; RaveSerializeJSON(value, allocator, ##__VA_ARGS__)
#define RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, ...) { do { RAVE_SERIALIZEJSON(__v, allocator, ##__VA_ARGS__); (value).PushBack(__v, allocator); } while(false); };
#define RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, key, ...) { do { RAVE_SERIALIZEJSON(__v, allocator, ##__VA_ARGS__); (value).AddMember(key, __v, allocator); } while(false); };

#define RAVE_SERIALIZEJSON_ENSURE_OBJECT(value) { if (!(value).IsObject()) { (value).SetObject(); } }
#define RAVE_SERIALIZEJSON_ENSURE_ARRAY(value) { if (!(value).IsArray()) { (value).SetArray(); } }

#define RAVE_SERIALIZEJSON_CLEAR_OBJECT(value) { do { (value).SetObject(); } while (false); }
#define RAVE_SERIALIZEJSON_CLEAR_ARRAY(value) { do { (value).SetArray(); } while (false); }

#define RAVE_DESERIALIZEJSON_ENSURE_OBJECT(value) { \
    if (!(value).IsObject()) { \
        throw OPENRAVE_EXCEPTION_FORMAT0("failed deserialize json, an object is expected", ORE_InvalidArguments); \
    } \
}
#define RAVE_DESERIALIZEJSON_ENSURE_ARRAY(value) { \
    if (!(value).IsArray()) { \
        throw OPENRAVE_EXCEPTION_FORMAT0("failed deserialize json, an array is expected", ORE_InvalidArguments); \
    } \
}
#define RAVE_DESERIALIZEJSON_REQUIRED(value, key, destination) { \
    if (!(value).HasMember(key)) { \
        throw OPENRAVE_EXCEPTION_FORMAT("failed deserialize json due to missing key \"%s\"", key, ORE_InvalidArguments); \
    } \
    RaveDeserializeJSON((value)[key], destination); \
}
#define RAVE_DESERIALIZEJSON_OPTIONAL(value, key, destination) { \
    if ((value).HasMember(key)) { \
        RaveDeserializeJSON((value)[key], destination); \
    } \
}

inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, bool v);
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, int v);
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, double v);
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, float v);
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const char* v);
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::string& v);

template <typename T1, typename T2>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::pair<T1, T2>& p);

template <typename K, typename V>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::map<K, V>& m);

template <typename T, std::size_t N>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const boost::array<T, N>& a, std::size_t n = (std::size_t)-1);

template <typename T>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::vector<T>& v, std::size_t n = (std::size_t)-1);

template <typename T>
inline void RaveSerailizeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::set<T>& s);

template <typename T>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const RaveVector<T>& v, bool quat = false);

template <typename T>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const RaveTransform<T>& t);

inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const TriMesh& trimesh);
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const IkParameterization& ikparam);
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const SensorBase::CameraIntrinsics& intrinsics);

/// \brief serialize a bool as json, these functions are overloaded to allow for templates
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, bool v)
{
    value = rapidjson::Value(v).Move();
}

/// \brief serialize a int as json, these functions are overloaded to allow for templates
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, int v)
{
    value = rapidjson::Value(v).Move();
}

/// \brief serialize a double as json, these functions are overloaded to allow for templates
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, double v)
{
    value = rapidjson::Value(v).Move();
}

/// \brief serialize a float as json, these functions are overloaded to allow for templates
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, float v)
{
    value = rapidjson::Value(v).Move();
}

/// \brief serialize a c string as json, these functions are overloaded to allow for templates
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const char* v)
{
    value = rapidjson::Value().SetString(v, allocator);
}

/// \brief serialize a std::string as json, these functions are overloaded to allow for templates
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::string& v)
{
    value = rapidjson::Value().SetString(v.c_str(), allocator);
}

/// \brief serialize a std::pair as json
template <typename T1, typename T2>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::pair<T1, T2>& p)
{
    RAVE_SERIALIZEJSON_CLEAR_ARRAY(value);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, p.first);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, p.second);
}

/// \brief serialize a std::map as json
template <typename K, typename V>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::map<K, V>& m)
{
    RAVE_SERIALIZEJSON_CLEAR_OBJECT(value);

    for (typename std::map<K, V>::const_iterator it = m.begin(); it != m.end(); ++it) {
        RAVE_SERIALIZEJSON(key, allocator, it->first);
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, key, it->second);
    }
}

/// \brief serialize a std::vector as json
template <typename T>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::vector<T>& v, std::size_t n)
{
    RAVE_SERIALIZEJSON_CLEAR_ARRAY(value);
    for (std::size_t i = 0; i < v.size() && i < n; ++i)
    {
        RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, v[i]);
    }
}

/// \brief serailize a std::set
template <typename T>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const std::set<T>& s)
{
    RAVE_SERIALIZEJSON_CLEAR_ARRAY(value);
    for(typename std::set<T>::const_iterator it = s.begin(); it != s.end(); ++it)
    {
        RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, *it);
    }
}

/// \brief serialize a boost::array as json
template <typename T, std::size_t N>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const boost::array<T, N>& a, std::size_t n)
{
    RAVE_SERIALIZEJSON_CLEAR_ARRAY(value);
    for (std::size_t i = 0; i < a.size() && i < n; ++i)
    {
        RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, a[i]);
    }
}

/// \brief serialize an OpenRAVE Vector as json
template <typename T>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const RaveVector<T>& v, bool quat)
{
    RAVE_SERIALIZEJSON_CLEAR_ARRAY(value);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, v[0]);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, v[1]);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, v[2]);
    if (quat)
    {
        RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, v[3]);
    }
}

/// \brief serialize an OpenRAVE Transform as json
template <typename T>
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const RaveTransform<T>& t)
{
    RAVE_SERIALIZEJSON_CLEAR_ARRAY(value);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, t.rot[0]);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, t.rot[1]);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, t.rot[2]);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, t.rot[3]);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, t.trans[0]);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, t.trans[1]);
    RAVE_SERIALIZEJSON_PUSHBACK(value, allocator, t.trans[2]);
}

/// \brief serialize an OpenRAVE TriMesh as json
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const TriMesh& trimesh)
{
    RAVE_SERIALIZEJSON_CLEAR_OBJECT(value);
    {
        rapidjson::Value verticesValue;
        verticesValue.SetArray();
        for (std::vector<Vector>::const_iterator it = trimesh.vertices.begin(); it != trimesh.vertices.end(); ++it) {
            RAVE_SERIALIZEJSON_PUSHBACK(verticesValue, allocator, (*it)[0]);
            RAVE_SERIALIZEJSON_PUSHBACK(verticesValue, allocator, (*it)[1]);
            RAVE_SERIALIZEJSON_PUSHBACK(verticesValue, allocator, (*it)[2]);
        }
        value.AddMember("vertices", verticesValue, allocator);
    }

    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "indices", trimesh.indices);
}

/// \brief serialize an OpenRAVE IkParameterization as json
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const IkParameterization& ikparam)
{
    RAVE_SERIALIZEJSON_CLEAR_OBJECT(value);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "type", ikparam.GetName());
    switch(ikparam.GetType()) {
    case IKP_Transform6D:
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "rotate", ikparam.GetTransform6D().rot);
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "translate", ikparam.GetTransform6D().trans);
        break;
    case IKP_TranslationDirection5D:
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "translate", ikparam.GetTranslationDirection5D().pos);
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "direction", ikparam.GetTranslationDirection5D().dir);
        break;
    default:
        break;
    }
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "customData", ikparam.GetCustomDataMap());
}

/// \brief serialize an OpenRAVE CameraIntrinsics as json
inline void RaveSerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, const SensorBase::CameraIntrinsics& intrinsics)
{
    RAVE_SERIALIZEJSON_CLEAR_OBJECT(value);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "fx", intrinsics.fx);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "fy", intrinsics.fy);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "cx", intrinsics.cx);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "cy", intrinsics.cy);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "focalLength", intrinsics.focal_length);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "distortionModel", intrinsics.distortion_model);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "distortionCoeffs", intrinsics.distortion_coeffs);
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, bool &v);
inline void RaveDeserializeJSON(const rapidjson::Value &value, int &v);
inline void RaveDeserializeJSON(const rapidjson::Value &value, double &v);
inline void RaveDeserializeJSON(const rapidjson::Value &value, float &v);
inline void RaveDeserializeJSON(const rapidjson::Value &value, std::string &v);
inline void RaveDeserializeJSON(const rapidjson::Value &value, uint8_t &v);
template <typename T1, typename T2>
inline void RaveDeserializeJSON(const rapidjson::Value &value, std::pair<T1, T2>& p);
template <typename T>
inline void RaveDeserializeJSON(const rapidjson::Value &value, std::vector<T>& v);
template <typename K, typename V>
inline void RaveDeserializeJSON(const rapidjson::Value &value, std::map<K, V>& m);
template <typename T>
inline void RaveDeserializeJSON(const rapidjson::Value &value, std::set<T>& s);
template <typename T, std::size_t N>
inline void RaveDeserializeJSON(const rapidjson::Value &value, boost::array<T, N>& a);
template <typename T>
inline void RaveDeserializeJSON(const rapidjson::Value &value, RaveVector<T>& v);
template <typename T>
inline void RaveDeserializeJSON(const rapidjson::Value &value, RaveTransform<T>& t);
inline void RaveDeserializeJSON(const rapidjson::Value &value, TriMesh& trimesh);
inline void RaveDeserializeJSON(const rapidjson::Value &value, SensorBase::CameraIntrinsics& intrinsics);

inline void RaveDeserializeJSON(const rapidjson::Value &value, bool &v)
{
    if (!value.IsBool()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a bool", ORE_InvalidArguments);
    }
    v = value.GetBool();
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, int &v)
{
    if (!value.IsInt()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as an int", ORE_InvalidArguments);
    }
    v = value.GetInt();
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, double &v)
{
    if (!value.IsNumber()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a double", ORE_InvalidArguments);
    }
    v = value.GetDouble();
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, float &v)
{
    if (!value.IsNumber()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a float", ORE_InvalidArguments);
    }
    v = static_cast<float>(value.GetDouble());
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, std::string &v)
{
    if (!value.IsString()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a string", ORE_InvalidArguments);
    }
    v = value.GetString();
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, uint8_t &v)
{
    if (!value.IsUint()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as an uint8_t", ORE_InvalidArguments);
    }
    v = value.GetUint();
}

template <typename T1, typename T2>
inline void RaveDeserializeJSON(const rapidjson::Value &value, std::pair<T1, T2>& p)
{
    if (!value.IsArray() || value.Size() != 2) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a pair", ORE_InvalidArguments);
    }
    RaveDeserializeJSON(value[0], p.first);
    RaveDeserializeJSON(value[1], p.second);
}

template <typename T>
inline void RaveDeserializeJSON(const rapidjson::Value &value, std::vector<T>& v)
{
    if (!value.IsArray()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a vector", ORE_InvalidArguments);
    }
    v.resize(value.Size());
    for (rapidjson::SizeType i = 0; i < value.Size(); ++i) {
        RaveDeserializeJSON(value[i], v[i]);
    }
}

template <typename K, typename V>
inline void RaveDeserializeJSON(const rapidjson::Value &value, std::map<K, V>& m)
{
    if (!value.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a map", ORE_InvalidArguments);
    }

    m.clear();
    for (rapidjson::Value::ConstMemberIterator it = value.MemberBegin(); it != value.MemberEnd(); ++it) {
        K k;
        RaveDeserializeJSON(it->name, k);
        RaveDeserializeJSON(it->value, m[k]);
    }
}

template <typename T>
inline void RaveDeserializeJSON(const rapidjson::Value& value, std::set<T>& s)
{
    if (!value.IsArray()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a set", ORE_InvalidArguments);
    }
    s.clear();
    for(rapidjson::SizeType i=0; i<value.Size(); ++i) {
        T result;
        RaveDeserializeJSON(value[i], result);
        s.insert(result);
    }
}

template <typename T, std::size_t N>
inline void RaveDeserializeJSON(const rapidjson::Value &value, boost::array<T, N>& a)
{
    if (!value.IsArray() || value.Size() > N) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as an array", ORE_InvalidArguments);
    }

    for (rapidjson::SizeType i = 0; i < value.Size(); ++i) {
        RaveDeserializeJSON(value[i], a[i]);
    }
}

template <typename T>
inline void RaveDeserializeJSON(const rapidjson::Value &value, RaveVector<T>& v)
{
    if (!value.IsArray() || (value.Size() != 3 && value.Size() != 4)) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a RaveVector", ORE_InvalidArguments);
    }
    RaveDeserializeJSON(value[0], v[0]);
    RaveDeserializeJSON(value[1], v[1]);
    RaveDeserializeJSON(value[2], v[2]);
    if (value.Size() == 4)
    {
        RaveDeserializeJSON(value[3], v[3]);
    }
}

template <typename T>
inline void RaveDeserializeJSON(const rapidjson::Value &value, RaveTransform<T>& t)
{
    if (!value.IsArray() || value.Size() != 7) {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a RaveTransform", ORE_InvalidArguments);
    }
    RaveDeserializeJSON(value[0], t.rot[0]);
    RaveDeserializeJSON(value[1], t.rot[1]);
    RaveDeserializeJSON(value[2], t.rot[2]);
    RaveDeserializeJSON(value[3], t.rot[3]);
    RaveDeserializeJSON(value[4], t.trans[0]);
    RaveDeserializeJSON(value[5], t.trans[1]);
    RaveDeserializeJSON(value[6], t.trans[2]);
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, TriMesh& trimesh)
{
    RAVE_DESERIALIZEJSON_ENSURE_OBJECT(value);
    if (!value.HasMember("vertices") || !value["vertices"].IsArray() || value["vertices"].Size() % 3 != 0)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("failed to deserialize json, value cannot be decoded as a TriMesh, \"vertices\" malformatted", ORE_InvalidArguments);
    }

    trimesh.vertices.clear();
    trimesh.vertices.reserve(value["vertices"].Size() / 3);

    for (rapidjson::Value::ConstValueIterator it = value["vertices"].Begin(); it != value["vertices"].End();)
    {
        Vector vertex;
        RaveDeserializeJSON(*(it++), vertex.x);
        RaveDeserializeJSON(*(it++), vertex.y);
        RaveDeserializeJSON(*(it++), vertex.z);
        trimesh.vertices.push_back(vertex);
    }

    RaveDeserializeJSON(value["indices"], trimesh.indices);
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, IkParameterization& ikparam)
{
    RAVE_DESERIALIZEJSON_ENSURE_OBJECT(value);

    std::string typestr;
    RAVE_DESERIALIZEJSON_REQUIRED(value, "type", typestr)

    if (typestr == "Transform6D")
    {
        Transform transform;
        RAVE_DESERIALIZEJSON_REQUIRED(value, "rotate", transform.rot);
        RAVE_DESERIALIZEJSON_REQUIRED(value, "translate", transform.trans);
        ikparam.SetTransform6D(transform);
    }
    else if (typestr == "TranslationDirection5D")
    {
        RAY ray;
        RAVE_DESERIALIZEJSON_REQUIRED(value, "translate", ray.pos);
        RAVE_DESERIALIZEJSON_REQUIRED(value, "direction", ray.dir);
        ikparam.SetTranslationDirection5D(ray);
    }
    else
    {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to deserialize json, unsupported IkParameterization type \"%s\"", typestr, ORE_InvalidArguments);
    }

    std::map<std::string, std::vector<dReal> > customData;
    RAVE_DESERIALIZEJSON_REQUIRED(value, "customData", customData);

    ikparam.ClearCustomValues();
    for (std::map<std::string, std::vector<dReal> >::const_iterator it = customData.begin(); it != customData.end(); ++it) {
        ikparam.SetCustomValues(it->first, it->second);
    }
}

inline void RaveDeserializeJSON(const rapidjson::Value &value, SensorBase::CameraIntrinsics& intrinsics)
{
    RAVE_DESERIALIZEJSON_ENSURE_OBJECT(value);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "fx", intrinsics.fx);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "fy", intrinsics.fy);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "cx", intrinsics.cx);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "cy", intrinsics.cy);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "focalLength", intrinsics.focal_length);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "distortionModel", intrinsics.distortion_model);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "distortionCoeffs", intrinsics.distortion_coeffs);
}

}

#endif // OPENRAVE_SERIALIZE_JSON_H
