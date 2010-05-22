// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "plugindefs.h"

#include "randomized-astar.h"
#include "rrt.h"
#include "graspgradient.h"
#include "pathoptimizers.h"

RAVE_PLUGIN_API InterfaceBasePtr CreateInterface(PluginType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv)
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
    case OpenRAVE::PT_Planner:
        if( interfacename == "ra*")
            return InterfaceBasePtr(new RandomizedAStarPlanner(penv));
        else if( interfacename == "birrt")
            return InterfaceBasePtr(new BirrtPlanner(penv));
        else if( interfacename == "rbirrt") {
            RAVELOG_WARNA("rBiRRT is deprecated, use BiRRT\n");
            return InterfaceBasePtr(new BirrtPlanner(penv));
        }
        else if( interfacename == "basicrrt")
            return InterfaceBasePtr(new BasicRrtPlanner(penv));
        else if( interfacename == "explorationrrt" )
            return InterfaceBasePtr(new ExplorationPlanner(penv));
        else if( interfacename == "graspgradient" )
            return InterfaceBasePtr(new GraspGradientPlanner(penv));
        else if( interfacename == "shortcut_linear" )
            return InterfaceBasePtr(new ShortcutLinearPlanner(penv));
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
    pinfo->interfacenames[OpenRAVE::PT_Planner].push_back("RA*");
    pinfo->interfacenames[OpenRAVE::PT_Planner].push_back("BiRRT");
    pinfo->interfacenames[OpenRAVE::PT_Planner].push_back("BasicRRT");
    pinfo->interfacenames[OpenRAVE::PT_Planner].push_back("ExplorationRRT");
    pinfo->interfacenames[OpenRAVE::PT_Planner].push_back("GraspGradient");
    pinfo->interfacenames[OpenRAVE::PT_Planner].push_back("shortcut_linear");
    return true;
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
