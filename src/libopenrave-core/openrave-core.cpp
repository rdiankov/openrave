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
EnvironmentBasePtr RaveCreateEnvironment(int options) {
    std::shared_ptr<Environment> p(new Environment());
    p->Init(!!(options&ECO_StartSimulationThread));
    return p;
}

EnvironmentBasePtr CreateEnvironment(bool bLoadAllPlugins) {
    return RaveCreateEnvironment();
}
}

#if !defined(OPENRAVE_IS_ASSIMP3) && !defined(OPENRAVE_ASSIMP)

bool RaveParseXFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("x-files not supported\n");
    return false;
}

bool RaveParseXFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("x-files not supported\n");
    return false;
}

bool RaveParseXData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::vector<char>& data,const AttributesList& atts)
{
    RAVELOG_ERROR("x-files not supported\n");
    return false;
}

bool RaveParseXData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::vector<char>& data,const AttributesList& atts)
{
    RAVELOG_ERROR("x-files not supported\n");
    return false;
}

#endif
