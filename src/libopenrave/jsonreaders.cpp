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
    StartArray();
    WriteDouble(t.rot[0]);
    WriteDouble(t.rot[1]);
    WriteDouble(t.rot[2]);
    WriteDouble(t.rot[3]);
    WriteDouble(t.trans[0]);
    WriteDouble(t.trans[1]);
    WriteDouble(t.trans[2]);
    EndArray();
}
void BaseJSONWriter::WriteTriMesh(const TriMesh& trimesh) {
    StartObject();
    WriteString("vertices");
    StartArray();
    FOREACHC(itv, trimesh.vertices) {
        WriteDouble((*itv)[0]);
        WriteDouble((*itv)[1]);
        WriteDouble((*itv)[2]);
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

void BaseJSONWriter::WriteIkParameterization(const IkParameterization& ikparam) {
    StartObject();

    WriteString("type");
    WriteString(ikparam.GetName());

    switch(ikparam.GetType()) {
    case IKP_Transform6D:
        WriteString("rotate");
        WriteVector(ikparam.GetTransform6D().rot, true);
        WriteString("translate");
        WriteVector(ikparam.GetTransform6D().trans);
        break;
    case IKP_TranslationDirection5D:
        WriteString("translate");
        WriteVector(ikparam.GetTranslationDirection5D().pos);
        WriteString("direction");
        WriteVector(ikparam.GetTranslationDirection5D().dir);
        break;
    }

    EndObject();
}

void BaseJSONWriter::WriteCameraIntrinsics(const SensorBase::CameraIntrinsics& intrinsics)
{
    StartObject();

    WriteString("fx");
    WriteDouble(intrinsics.fx);

    WriteString("fy");
    WriteDouble(intrinsics.fy);

    WriteString("cx");
    WriteDouble(intrinsics.cx);

    WriteString("cy");
    WriteDouble(intrinsics.cy);

    WriteString("distortionModel");
    WriteString(intrinsics.distortion_model);

    WriteString("distortionCoeffs");
    WriteArray(intrinsics.distortion_coeffs);

    WriteString("focalLength");
    WriteDouble(intrinsics.focal_length);

    EndObject();
}

void BaseJSONWriter::WriteBoost3Array(const boost::array<dReal, 3>& a, size_t n) {
    StartArray();
    for (size_t i=0; i<a.size() && i < n; ++i) {
        WriteDouble(a[i]);
    }
    EndArray();
}
void BaseJSONWriter::WriteBoost3Array(const boost::array<uint8_t, 3>& a, size_t n) {
    StartArray();
    for (size_t i=0; i<a.size() && i < n; ++i) {
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
