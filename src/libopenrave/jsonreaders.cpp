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
#include "libopenrave.h"
#include <openrave/jsonreaders.h>

#include <boost/lexical_cast.hpp>

namespace OpenRAVE {
namespace jsonreaders {

BufferJSONWriter::BufferJSONWriter() : _buffer(), _writer(_buffer)
{
}

BufferJSONWriter::~BufferJSONWriter()
{
}

const std::string& BufferJSONWriter::GetFormat() const
{
    static const std::string format("json");
    return format;
}

const char* BufferJSONWriter::SerializeJSON()
{
    return _buffer.GetString();
}

void BufferJSONWriter::StartArray()
{
    _writer.StartArray();
}

void BufferJSONWriter::EndArray()
{
    _writer.EndArray();
}

void BufferJSONWriter::StartObject()
{
    _writer.StartObject();
}

void BufferJSONWriter::EndObject()
{
    _writer.EndObject();
}

void BufferJSONWriter::WriteNull()
{
    _writer.Null();
}

void BufferJSONWriter::WriteBool(bool value)
{
    _writer.Bool(value);
}

void BufferJSONWriter::WriteInt(int value)
{
    _writer.Int(value);
}

void BufferJSONWriter::WriteDouble(double value)
{
    _writer.Double(value);
}

void BufferJSONWriter::WriteString(const std::string& value)
{
    _writer.String(value);
}

void BufferJSONWriter::WriteString(const char* value)
{
    _writer.String(value);
}

void BufferJSONWriter::WriteVector(const Vector& v, bool quat) {
    StartArray();
    WriteDouble(v[0]);
    WriteDouble(v[1]);
    WriteDouble(v[2]);
    if (quat) {
        WriteDouble(v[3]);
    }
    EndArray();
}

void BufferJSONWriter::WriteTransform(const Transform& t) {
    StartObject();
    WriteString("rotate");
    WriteVector(t.rot, true);
    WriteString("translate");
    WriteVector(t.trans);
    EndObject();
}

void BufferJSONWriter::WriteTriMesh(const TriMesh& trimesh) {
    StartObject();
    WriteString("vertices");
    StartArray();
    FOREACHC(itv, trimesh.vertices) {
        WriteVector(*itv);
    }
    EndArray();

    WriteString("indices");
    StartArray();
    for (int index=0; index < trimesh.indices.size(); ++index) {
        WriteInt(index);
    }
    EndArray();

    EndObject();
}

} // jsonreaders
} // OpenRAVE
