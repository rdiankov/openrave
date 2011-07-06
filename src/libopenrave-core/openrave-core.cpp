// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#include "ravep.h"
#include "environment-core.h"

namespace OpenRAVE {
EnvironmentBasePtr RaveCreateEnvironment() {
    boost::shared_ptr<Environment> p(new Environment());
    p->Init();
    return p;
}

EnvironmentBasePtr CreateEnvironment(bool bLoadAllPlugins) {
    return RaveCreateEnvironment();
}
}

#ifndef OPENRAVE_COLLADA_SUPPORT

bool RaveParseColladaFile(EnvironmentBasePtr penv, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBasePtr penv, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}


void RaveWriteColladaFile(EnvironmentBasePtr penv, const string& filename)
{
    RAVELOG_ERROR("collada files not supported\n");
}

void RaveWriteColladaFile(KinBodyPtr pbody, const string& filename)
{
    RAVELOG_ERROR("collada files not supported\n");
}

void RaveWriteColladaFile(RobotBasePtr probot, const string& filename)
{
    RAVELOG_ERROR("collada files not supported\n");
}

#endif
