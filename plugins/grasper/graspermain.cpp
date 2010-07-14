// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
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
#include "plugindefs.h"
#include "grasperplanner.h"
#include "grasper.h"

RAVE_PLUGIN_API InterfaceBasePtr CreateInterface(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv)
{
    if( strcmp(pluginhash,RaveGetInterfaceHash(type)) ) {
        RAVELOG_WARNA("plugin type hash is wrong\n");
        throw openrave_exception("bad plugin hash");
    }
    if( !penv )
        return InterfaceBasePtr();
    
    stringstream ss(name);
    string interfacename;
    ss >> interfacename;
    std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);

    switch(type) {
    case PT_Planner:
        if( interfacename == "grasper" )
            return InterfaceBasePtr(new GrasperPlanner(penv));
        break;
    case PT_ProblemInstance:
        if( interfacename == "grasperproblem") {
            RAVELOG_WARN("grasperproblem name deprecated, please use Grasper\n");
            return InterfaceBasePtr(new GrasperProblem(penv));
        }
        else if( interfacename == "grasper")
            return InterfaceBasePtr(new GrasperProblem(penv));
        break;
    default:
        break;
    }

    return InterfaceBasePtr();
}

RAVE_PLUGIN_API bool GetPluginAttributes(PLUGININFO* pinfo, int size)
{
    if( pinfo == NULL ) return false;
    if( size != sizeof(PLUGININFO) ) {
        RAVELOG_ERRORA("bad plugin info sizes %d != %d\n", size, sizeof(PLUGININFO));
        return false;
    }

    // fill pinfo
    pinfo->interfacenames[OpenRAVE::PT_Planner].push_back("Grasper");
    pinfo->interfacenames[OpenRAVE::PT_ProblemInstance].push_back("GrasperProblem");
    return true;
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
