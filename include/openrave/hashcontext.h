// -*- coding: utf-8 -*-
// Copyright (C) Rosen Diankov <rosen.diankov@gmail.com>
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

#pragma once

#include <cmath>

#include <array>
#include <string>
#include <vector>

#include <openrave/config.h>
#include <openrave/geometry.h>

#include <boost/assert.hpp>

#include <rapidjson/document.h>

// Forward declarations for md5 internals
struct md5_state_s;
typedef struct md5_state_s md5_state_t;

namespace OpenRAVE {

class OPENRAVE_API HashContext
{
public:
    HashContext(const int precision = 4);
    ~HashContext();

    const std::string& HexDigest();

    // Serialize integers by just treating them as raw bytes
    template <typename IntegerT>
    typename std::enable_if<std::is_integral<IntegerT>::value, HashContext&>::type operator<<(IntegerT value)
    {
        _Append(reinterpret_cast<const uint8_t*>(&value), sizeof(IntegerT));
        return *this;
    }

    // Serialize floating point by rounding _then_ interpreting as bytes
    template <typename FloatT>
    typename std::enable_if<std::is_floating_point<FloatT>::value, HashContext&>::type operator<<(FloatT value)
    {
        FloatT result = round(value * _roundingScale) / _roundingScale;
        if (result == -0) {     // don't want to disinguish -0.00001 and 0, so set it to 0.
            result = 0;
        }
        _Append(reinterpret_cast<const uint8_t*>(&result), sizeof(FloatT));
        return *this;
    }

    // Serialize strings by just forwarding the string data
    HashContext& operator<<(const std::string& data)
    {
        _Append(reinterpret_cast<const uint8_t*>(data.data()), data.size());
        return *this;
    }

    // For container types we can just apply the base operator for all contents
    template <class T>
    HashContext& operator<<(const geometry::RaveVector<T>& v)
    {
        return (*this) << v.x << v.y << v.z;
    }

    template <class T>
    HashContext& operator<<(const geometry::RaveTransform<T>& t)
    {
        return (*this) << t.rot << t.trans;
    }

    template <class T>
    HashContext& operator<<(const std::vector<T>& values)
    {
        for (const T& v : values) {
            (*this) << v;
        }
        return (*this);
    }

    // Allow directly updating with rapidjson values
    HashContext& operator<<(const rapidjson::Value& rValue)
    {
        switch (rValue.GetType()) {
        // Don't count a null for anything
        case rapidjson::kNullType: {
            break;
        }

        // For true/false, treat them as a single byte boolean
        case rapidjson::kFalseType: {
            *this << (uint8_t)0;
            break;
        }
        case rapidjson::kTrueType: {
            *this << (uint8_t)1;
            break;
        }

        // For strings, hash the underlying string data
        case rapidjson::kStringType: {
            _Append((const uint8_t*)rValue.GetString(), rValue.GetStringLength());
            break;
        }

        // For numbers, delegate to other overrides
        case rapidjson::kNumberType: {
            if (rValue.IsDouble()) {
                *this << rValue.GetDouble();
            }
            else if (rValue.IsInt()) {
                *this << rValue.GetInt();
            }
            else if (rValue.IsUint()) {
                *this << rValue.GetUint();
            }
            else if (rValue.IsInt64()) {
                *this << rValue.GetInt64();
            }
            else {
                *this << rValue.GetUint64();
            }
            break;
        }

        // For arrays, recurse on each element
        case rapidjson::kArrayType: {
            for (rapidjson::Value::ConstValueIterator valueIt = rValue.Begin(); valueIt != rValue.End(); valueIt++) {
                *this << *valueIt;
            }
            break;
        }

        // For objects, recurse for each k:v pair
        case rapidjson::kObjectType: {
            for (rapidjson::Value::ConstMemberIterator memberIt = rValue.MemberBegin(); memberIt != rValue.MemberEnd(); memberIt++) {
                *this << memberIt->name << memberIt->value;
            }
            break;
        }
        }
        return *this;
    }

private:
    void _Append(const uint8_t* data, size_t len);

private:
    const double _roundingScale;
    md5_state_t* _md5_state;
    std::string _hexDigest;
};

} // namespace OpenRAVE
