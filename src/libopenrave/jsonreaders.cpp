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

#include <boost/uuid/uuid_io.hpp>

namespace OpenRAVE {

const std::string& BaseJSONWriter::GetFormat() const
{
    static const std::string format("json");
    return format;
}

void BaseJSONWriter::WriteVector(const Vector& v, bool quat) {
    StartArray();
    WriteDouble(v[0]);
    WriteDouble(v[1]);
    WriteDouble(v[2]);
    if (quat) {
        WriteDouble(v[3]);
    }
    EndArray();
}
void BaseJSONWriter::WriteTransform(const Transform& t) {
    StartObject();
    WriteString("rotate");
    WriteVector(t.rot, true);
    WriteString("translate");
    WriteVector(t.trans);
    EndObject();
}
void BaseJSONWriter::WriteTriMesh(const TriMesh& trimesh) {
    StartObject();
    WriteString("vertices");
    StartArray();
    FOREACHC(itv, trimesh.vertices) {
        WriteVector(*itv);
    }
    EndArray();

    WriteString("indices");
    StartArray();
    for (size_t index = 0; index < trimesh.indices.size(); ++index) {
        WriteInt(index);
    }
    EndArray();

    EndObject();
}

void BaseJSONWriter::WriteCameraIntrinsics(const SensorBase::CameraIntrinsics& intrinsics)
{
    WriteString("fx");
    WriteDouble(intrinsics.fx);

    WriteString("fy");
    WriteDouble(intrinsics.fy);

    WriteString("cx");
    WriteDouble(intrinsics.cx);

    WriteString("cy");
    WriteDouble(intrinsics.cy);

    WriteString("distortion_model");
    WriteString(intrinsics.distortion_model);

    WriteString("distortion_coeffs");
    WriteArray(intrinsics.distortion_coeffs);

    WriteString("focal_length");
    WriteDouble(intrinsics.focal_length);
}

void BaseJSONWriter::WriteBoostUUID(const boost::uuids::uuid& uuid) {
    WriteString(boost::uuids::to_string(uuid));
}
void BaseJSONWriter::WriteBoost3Array(const boost::array<dReal, 3>& a) {
    StartArray();
    for (size_t i=0; i<a.size(); ++i) {
        WriteDouble(a[i]);
    }
    EndArray();
}
void BaseJSONWriter::WriteBoost3Array(const boost::array<uint8_t, 3>& a) {
    StartArray();
    for (size_t i=0; i<a.size(); ++i) {
        WriteInt(a[i]);
    }
    EndArray();
}
void BaseJSONWriter::WriteArray(const std::vector<dReal>& a) {
    StartArray();
    for (size_t i=0; i<a.size(); ++i) {
        WriteDouble(a[i]);
    }
    EndArray();
}
void BaseJSONWriter::WriteArray(const std::vector<int>& a) {
    StartArray();
    for (size_t i=0; i<a.size(); ++i) {
        WriteInt(a[i]);
    }
    EndArray();
}
void BaseJSONWriter::WriteArray(const std::vector<std::string>& a) {
    StartArray();
    for (size_t i=0; i<a.size(); ++i) {
        WriteString(a[i]);
    }
    EndArray();
}
void BaseJSONWriter::WritePair(const std::pair<dReal, dReal>& p) {
    StartArray();
    WriteDouble(p.first);
    WriteDouble(p.second);
    EndArray();
}

} // OpenRAVE
