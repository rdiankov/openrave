// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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

/** \file jsonreaders.h
    \brief classes for reading common OpenRAVE objects from JSON

    This file is optional and not automatically included with openrave.h
 */
#ifndef OPENRAVE_JSONREADERS_H
#define OPENRAVE_JSONREADERS_H

#include <openrave/openrave.h>

#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/ostreamwrapper.h>

namespace OpenRAVE {
namespace jsonreaders {

/// \brief writer that writes compact json into a buffer that can be retrieved by calling GetString().
class OPENRAVE_API StringJSONWriter : public BaseJSONWriter
{
public:
    StringJSONWriter() : _stringbuffer(), _writer(_stringbuffer) {
    }
    virtual ~StringJSONWriter() {
    }

    virtual std::string GetString() {
        return std::string(_stringbuffer.GetString());
    }

    virtual void StartArray() {
        _writer.StartArray();
    }
    virtual void EndArray() {
        _writer.EndArray();
    }
    virtual void StartObject() {
        _writer.StartObject();
    }
    virtual void EndObject() {
        _writer.EndObject();
    }

    virtual void WriteNull() {
        _writer.Null();
    }
    virtual void WriteBool(bool value) {
        _writer.Bool(value);
    }
    virtual void WriteInt(int value) {
        _writer.Int(value);
    }
    virtual void WriteDouble(double value) {
        _writer.Double(value);
    }
    virtual void WriteString(const std::string& value) {
        _writer.String(value);
    }
    virtual void WriteString(const char* value) {
        _writer.String(value);
    }

protected:

    rapidjson::StringBuffer _stringbuffer;
    rapidjson::Writer<rapidjson::StringBuffer> _writer;
};

/// \brief writer that writes pretty print formatted json into a buffer that can be retrieved by calling GetString().
class OPENRAVE_API StringPrettyJSONWriter : public BaseJSONWriter
{
public:
    StringPrettyJSONWriter() : _stringbuffer(), _writer(_stringbuffer) {
    }
    virtual ~StringPrettyJSONWriter() {
    }

    virtual std::string GetString() {
        return std::string(_stringbuffer.GetString());
    }

    virtual void StartArray() {
        _writer.StartArray();
    }
    virtual void EndArray() {
        _writer.EndArray();
    }
    virtual void StartObject() {
        _writer.StartObject();
    }
    virtual void EndObject() {
        _writer.EndObject();
    }

    virtual void WriteNull() {
        _writer.Null();
    }
    virtual void WriteBool(bool value) {
        _writer.Bool(value);
    }
    virtual void WriteInt(int value) {
        _writer.Int(value);
    }
    virtual void WriteDouble(double value) {
        _writer.Double(value);
    }
    virtual void WriteString(const std::string& value) {
        _writer.String(value);
    }
    virtual void WriteString(const char* value) {
        _writer.String(value);
    }

protected:

    rapidjson::StringBuffer _stringbuffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> _writer;
};

/// \brief writer that writes compact json into a std::ostream
class OPENRAVE_API StreamJSONWriter : public BaseJSONWriter
{
public:
    StreamJSONWriter(std::ostream& ostream) : _ostreamwrapper(ostream), _writer(_ostreamwrapper) {
    }
    virtual ~StreamJSONWriter() {
    }

    virtual void StartArray() {
        _writer.StartArray();
    }
    virtual void EndArray() {
        _writer.EndArray();
    }
    virtual void StartObject() {
        _writer.StartObject();
    }
    virtual void EndObject() {
        _writer.EndObject();
    }

    virtual void WriteNull() {
        _writer.Null();
    }
    virtual void WriteBool(bool value) {
        _writer.Bool(value);
    }
    virtual void WriteInt(int value) {
        _writer.Int(value);
    }
    virtual void WriteDouble(double value) {
        _writer.Double(value);
    }
    virtual void WriteString(const std::string& value) {
        _writer.String(value);
    }
    virtual void WriteString(const char* value) {
        _writer.String(value);
    }

protected:

    rapidjson::OStreamWrapper _ostreamwrapper;
    rapidjson::Writer<rapidjson::OStreamWrapper> _writer;
};

/// \brief writer that writes pretty print formatted json into a std::ostream
class OPENRAVE_API StreamPrettyJSONWriter : public BaseJSONWriter
{
public:
    StreamPrettyJSONWriter(std::ostream& ostream) : _ostreamwrapper(ostream), _writer(_ostreamwrapper) {
    }
    virtual ~StreamPrettyJSONWriter() {
    }

    virtual void StartArray() {
        _writer.StartArray();
    }
    virtual void EndArray() {
        _writer.EndArray();
    }
    virtual void StartObject() {
        _writer.StartObject();
    }
    virtual void EndObject() {
        _writer.EndObject();
    }

    virtual void WriteNull() {
        _writer.Null();
    }
    virtual void WriteBool(bool value) {
        _writer.Bool(value);
    }
    virtual void WriteInt(int value) {
        _writer.Int(value);
    }
    virtual void WriteDouble(double value) {
        _writer.Double(value);
    }
    virtual void WriteString(const std::string& value) {
        _writer.String(value);
    }
    virtual void WriteString(const char* value) {
        _writer.String(value);
    }

protected:

    rapidjson::OStreamWrapper _ostreamwrapper;
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> _writer;
};

} // jsonreaders
} // OpenRAVE

#endif
