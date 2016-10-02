// -*- coding: utf-8 --*
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "openraveplugindefs.h"
#include "rrt.h"

#include <openrave/plugin.h>

PlannerBasePtr CreateShortcutLinearPlanner(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateGraspGradientPlanner(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateRandomizedAStarPlanner(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateWorkspaceTrajectoryTracker(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateLinearSmoother(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateConstraintParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput);

namespace rplanners {
PlannerBasePtr CreateParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateParabolicSmoother2(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateLinearTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateParabolicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateParabolicTrajectoryRetimer2(EnvironmentBasePtr penv, std::istream& sinput);
PlannerBasePtr CreateCubicTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput);
}

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_Planner:
        if( interfacename == "rastar" || interfacename == "ra*" ) {
            return CreateRandomizedAStarPlanner(penv,sinput);
        }
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
            return CreateGraspGradientPlanner(penv,sinput);
        }
        else if( interfacename == "shortcut_linear" ) {
            return CreateShortcutLinearPlanner(penv,sinput);
        }
        else if( interfacename == "lineartrajectoryretimer" ) {
            return rplanners::CreateLinearTrajectoryRetimer(penv,sinput);
        }
        else if( interfacename == "parabolictrajectoryretimer" ) {
            return rplanners::CreateParabolicTrajectoryRetimer(penv,sinput);
        }
        else if( interfacename == "parabolictrajectoryretimer2" ) {
            return rplanners::CreateParabolicTrajectoryRetimer2(penv,sinput);
        }
        else if( interfacename == "cubictrajectoryretimer" ) {
            return rplanners::CreateCubicTrajectoryRetimer(penv,sinput);
        }
        else if( interfacename == "workspacetrajectorytracker" ) {
            return CreateWorkspaceTrajectoryTracker(penv,sinput);
        }
        else if( interfacename == "linearsmoother" ) {
            return CreateLinearSmoother(penv,sinput);
        }
        else if( interfacename == "parabolicsmoother" ) {
            return rplanners::CreateParabolicSmoother(penv,sinput);
        }
        else if( interfacename == "parabolicsmoother2" ) {
            return rplanners::CreateParabolicSmoother2(penv,sinput);
        }
        else if( interfacename == "constraintparabolicsmoother" ) {
            return CreateConstraintParabolicSmoother(penv,sinput);
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Planner].push_back("RAStar");
    info.interfacenames[PT_Planner].push_back("BiRRT");
    info.interfacenames[PT_Planner].push_back("BasicRRT");
    info.interfacenames[PT_Planner].push_back("ExplorationRRT");
    info.interfacenames[PT_Planner].push_back("GraspGradient");
    info.interfacenames[PT_Planner].push_back("shortcut_linear");
    info.interfacenames[PT_Planner].push_back("LinearTrajectoryRetimer");
    info.interfacenames[PT_Planner].push_back("ParabolicTrajectoryRetimer");
    info.interfacenames[PT_Planner].push_back("ParabolicTrajectoryRetimer2");
    info.interfacenames[PT_Planner].push_back("CubicTrajectoryRetimer");
    info.interfacenames[PT_Planner].push_back("WorkspaceTrajectoryTracker");
    info.interfacenames[PT_Planner].push_back("LinearSmoother");
    info.interfacenames[PT_Planner].push_back("ParabolicSmoother");
    info.interfacenames[PT_Planner].push_back("ParabolicSmoother2");
    info.interfacenames[PT_Planner].push_back("ConstraintParabolicSmoother");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
