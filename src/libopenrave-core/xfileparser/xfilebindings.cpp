// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com), Stefan Ulbrich, Gustavo Rodriguez
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
// functions that allow plugins to program for the RAVE simulator
#include "../ravep.h"

#include "XFileParser.h"

using namespace OpenRAVE;
using namespace std;

bool RaveParseXFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename,const AttributesList& atts)
{
    std::ifstream f(filename.c_str());
    f.seekg(0,ios::end);
    string filedata; filedata.resize(f.tellg());
    f.seekg(0,ios::beg);
    f.read(&filedata[0], filedata.size());
    return RaveParseXData(penv,ppbody,filedata,atts);
}

bool RaveParseXFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename,const AttributesList& atts)
{
    std::ifstream f(filename.c_str());
    f.seekg(0,ios::end);
    string filedata; filedata.resize(f.tellg());
    f.seekg(0,ios::beg);
    f.read(&filedata[0], filedata.size());
    return RaveParseXData(penv,pprobot,filedata,atts);
}

bool RaveParseXData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data,const AttributesList& atts)
{
    if( !ppbody ) {
        ppbody = RaveCreateKinBody(penv,"");
    }
    Assimp::XFileParser parser(data.c_str());
    const Assimp::XFile::Scene* scene = parser.GetImportedData();
    return false;
}

bool RaveParseXData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data,const AttributesList& atts)
{
    if( !pprobot ) {
        pprobot = RaveCreateRobot(penv,"GenericRobot");
    }
    KinBodyPtr pbody(pprobot);
    return RaveParseXData(penv,pbody,data,atts);
}
