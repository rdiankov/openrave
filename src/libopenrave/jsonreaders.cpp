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

const std::string& BufferJSONWriter::GetFormat() const
{
    static const std::string format("json");
    return format;
}

void BufferJSONWriter::Null()
{
    _writer.Null();
}

void BufferJSONWriter::Bool(bool value)
{
    _writer.Bool(value);
}

void BufferJSONWriter::Int(int value)
{
    _writer.Int(value);
}

void BufferJSONWriter::Double(double value)
{
    _writer.Double(value);
}

void BufferJSONWriter::String(const std::string& value)
{
    _writer.String(value);
}

void BufferJSONWriter::String(const char* value)
{
    _writer.String(value);
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

const char* BufferJSONWriter::SerializeJSON()
{
    return _buffer.GetString();
}

void BufferJSONWriter::SerializeVector(const Vector& v, bool quat) {
    _writer.StartArray();
    _writer.Double(v[0]);
    _writer.Double(v[1]);
    _writer.Double(v[2]);
    if (quat) {
        _writer.Double(v[3]);
    }
    _writer.EndArray();
}

void BufferJSONWriter::SerializeTransform(const Transform& t) {
    _writer.StartObject();
    _writer.String("rotate");
    SerializeVector(t.rot, true);
    _writer.String("translate");
    SerializeVector(t.trans);
    _writer.EndObject();
}

void BufferJSONWriter::SerializeTriMesh(const TriMesh& trimesh) {
    _writer.StartObject();
    _writer.String("vertices");
    _writer.StartArray();
    FOREACHC(itv, trimesh.vertices) {
        SerializeVector(*itv);
    }
    _writer.EndArray();

    _writer.String("indices");
    _writer.StartArray();
    for (int index=0; index < trimesh.indices.size(); ++index) {
        _writer.Int(index);
    }
    _writer.EndArray();

    _writer.EndObject();
}



} // jsonreaders
} // OpenRAVE
