// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
#include "workspacetrajectorytracker.h"

#include <rave/plugin.h>

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_Planner:
        if( interfacename == "ra*")
            return InterfaceBasePtr(new RandomizedAStarPlanner(penv));
        else if( interfacename == "birrt") {
            return InterfaceBasePtr(new BirrtPlanner(penv));
        }
        else if( interfacename == "rbirrt") {
            RAVELOG_WARN("rBiRRT is deprecated, use BiRRT\n");
            return InterfaceBasePtr(new BirrtPlanner(penv));
        }
        else if( interfacename == "basicrrt") {
            return InterfaceBasePtr(new BasicRrtPlanner(penv));
        }
        else if( interfacename == "explorationrrt" ) {
            return InterfaceBasePtr(new ExplorationPlanner(penv));
        }
        else if( interfacename == "graspgradient" ) {
            return InterfaceBasePtr(new GraspGradientPlanner(penv));
        }
        else if( interfacename == "shortcut_linear" ) {
            return InterfaceBasePtr(new ShortcutLinearPlanner(penv));
        }
        else if( interfacename == "workspacetrajectorytracker" ) {
            return InterfaceBasePtr(new WorkspaceTrajectoryTracker(penv));
        }
        break;
    default:
        break;
    }    
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_Planner].push_back("RA*");
    info.interfacenames[OpenRAVE::PT_Planner].push_back("BiRRT");
    info.interfacenames[OpenRAVE::PT_Planner].push_back("BasicRRT");
    info.interfacenames[OpenRAVE::PT_Planner].push_back("ExplorationRRT");
    info.interfacenames[OpenRAVE::PT_Planner].push_back("GraspGradient");
    info.interfacenames[OpenRAVE::PT_Planner].push_back("shortcut_linear");
    info.interfacenames[OpenRAVE::PT_Planner].push_back("workspacetrajectorytracker");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
