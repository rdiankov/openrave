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
#include <rapidjson/writer.h>
#include <rapidjson/ostreamwrapper.h>

namespace OpenRAVE {

void RaveWriteJSONFile(EnvironmentBasePtr penv, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::OStreamWrapper ostreamwrapper(ofstream);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    penv->SerializeJSON(doc, doc.GetAllocator(), 0);

    doc.Accept(writer);
}
void RaveWriteJSONFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::OStreamWrapper ostreamwrapper(ofstream);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    {
        rapidjson::Value bodiesValue;
        bodiesValue.SetArray();

        {
            rapidjson::Value bodyValue;
            pbody->SerializeJSON(bodyValue, doc.GetAllocator(), 0);
            bodiesValue.PushBack(bodyValue, doc.GetAllocator());
        }

        doc.AddMember("bodies", bodiesValue, doc.GetAllocator());
    }

    doc.Accept(writer);
}

void RaveWriteJSONFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename, const AttributesList& atts)
{
    std::ofstream ofstream(filename.c_str());
    rapidjson::OStreamWrapper ostreamwrapper(ofstream);
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(ostreamwrapper);
    rapidjson::Document doc;

    {
        rapidjson::Value bodiesValue;
        bodiesValue.SetArray();

        if (listbodies.size() > 0) {
            FOREACHC (it,listbodies) {
                rapidjson::Value bodyValue;
                (*it)->SerializeJSON(bodyValue, doc.GetAllocator(), 0);
                bodiesValue.PushBack(bodyValue, doc.GetAllocator());
            }
        }

        doc.AddMember("bodies", bodiesValue, doc.GetAllocator());
    }

    doc.Accept(writer);
}

}