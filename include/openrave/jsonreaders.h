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
#include <openrave/utils.h>

#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/writer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/ostreamwrapper.h>

namespace OpenRAVE {
namespace jsonreaders {

template <typename Writer>
class RapidJSONWriter: public BaseJSONWriter
{
public:

    RapidJSONWriter() {
    }
    virtual ~RapidJSONWriter() {
    }

    virtual void StartArray() {
        _pwriter->StartArray();
    }
    virtual void EndArray() {
        _pwriter->EndArray();
    }
    virtual void StartObject() {
        _pwriter->StartObject();
    }
    virtual void EndObject() {
        _pwriter->EndObject();
    }

    virtual void WriteNull() {
        _pwriter->Null();
    }
    virtual void WriteBool(bool value) {
        _pwriter->Bool(value);
    }
    virtual void WriteInt(int value) {
        _pwriter->Int(value);
    }
    virtual void WriteDouble(double value) {
        _pwriter->Double(value);
    }
    virtual void WriteString(const std::string& value) {
        _pwriter->String(value);
    }
    virtual void WriteString(const char* value) {
        _pwriter->String(value);
    }

protected:

    boost::shared_ptr<Writer> _pwriter;
};

class OPENRAVE_API BufferJSONWriter : public RapidJSONWriter<rapidjson::Writer<rapidjson::StringBuffer> >
{
public:
    BufferJSONWriter() : _writer(_buffer)
    {
        _pwriter.reset(&_writer, utils::null_deleter());
    }
    virtual ~BufferJSONWriter()
    {
    }

    virtual const char* GetBuffer()
    {
        return _buffer.GetString();
    }

protected:

    rapidjson::StringBuffer _buffer;
    rapidjson::Writer<rapidjson::StringBuffer> _writer;
};


class OPENRAVE_API BufferPrettyJSONWriter : public RapidJSONWriter<rapidjson::PrettyWriter<rapidjson::StringBuffer> >
{
public:
    BufferPrettyJSONWriter() : _writer(_buffer)
    {
        _pwriter.reset(&_writer, utils::null_deleter());
    }
    virtual ~BufferPrettyJSONWriter()
    {
    }

    virtual const char* GetBuffer()
    {
        return _buffer.GetString();
    }

protected:

    rapidjson::StringBuffer _buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> _writer;
};

class OPENRAVE_API StreamJSONWriter : public RapidJSONWriter<rapidjson::Writer<rapidjson::OStreamWrapper> >
{
public:
    StreamJSONWriter(std::ostream& ostream) : _ostreamwrapper(ostream), _writer(_ostreamwrapper)
    {
        _pwriter.reset(&_writer, utils::null_deleter());
    }
    virtual ~StreamJSONWriter()
    {
    }

protected:

    rapidjson::OStreamWrapper _ostreamwrapper;
    rapidjson::Writer<rapidjson::OStreamWrapper> _writer;
};

class OPENRAVE_API StreamPrettyJSONWriter : public RapidJSONWriter<rapidjson::PrettyWriter<rapidjson::OStreamWrapper> >
{
public:
    StreamPrettyJSONWriter(std::ostream& ostream)
     : _ostreamwrapper(ostream), _writer(_ostreamwrapper)
    {
        _pwriter.reset(&_writer, utils::null_deleter());
    }
    virtual ~StreamPrettyJSONWriter()
    {
    }

protected:

    rapidjson::OStreamWrapper _ostreamwrapper;
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> _writer;
};

} // jsonreaders
} // OpenRAVE

#endif
