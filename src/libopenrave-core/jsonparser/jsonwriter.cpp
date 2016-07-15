// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "jsoncommon.h"
#include <fstream>
#include <openrave/jsonreaders.h>

namespace OpenRAVE {

void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    OpenRAVE::jsonreaders::StreamPrettyJSONWriter streamwriter(ofstream);
    OpenRAVE::BaseJSONWriterPtr writer(&streamwriter, OpenRAVE::utils::null_deleter());

    writer->StartObject();
    penv->SerializeJSON(writer, 0);
    writer->EndObject();
}
void RaveWriteJSONFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    OpenRAVE::jsonreaders::StreamPrettyJSONWriter streamwriter(ofstream);
    OpenRAVE::BaseJSONWriterPtr writer(&streamwriter, OpenRAVE::utils::null_deleter());

    writer->StartObject();

    writer->WriteString("bodies");
    writer->StartArray();
    writer->StartObject();
    pbody->SerializeJSON(writer, 0);
    writer->EndObject();
    writer->EndArray();

    writer->EndObject();
}
void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    OpenRAVE::jsonreaders::StreamPrettyJSONWriter streamwriter(ofstream);
    OpenRAVE::BaseJSONWriterPtr writer(&streamwriter, OpenRAVE::utils::null_deleter());

    writer->StartObject();

    if (listbodies.size() > 0) {
        writer->WriteString("bodies");
        writer->StartArray();
        FOREACHC (it,listbodies) {
            writer->StartObject();
            (*it)->SerializeJSON(writer, 0);
            writer->EndObject();
        }
        writer->EndArray();
    }

    writer->EndObject();
}

}