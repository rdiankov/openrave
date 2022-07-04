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
#include "rplannersrave.h"

#include "openraveplugindefs.h"
#include "rrt.h"

OpenRAVE::PlannerBasePtr CreateShortcutLinearPlanner(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
//OpenRAVE::PlannerBasePtr CreateGraspGradientPlanner(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateRandomizedAStarPlanner(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateWorkspaceTrajectoryTracker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateLinearSmoother(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateConstraintParabolicSmoother(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);

namespace rplanners {
OpenRAVE::PlannerBasePtr CreateParabolicSmoother(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateParabolicSmoother2(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateLinearTrajectoryRetimer(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateParabolicTrajectoryRetimer(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateParabolicTrajectoryRetimer2(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateCubicTrajectoryRetimer(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateCubicTrajectoryRetimer2(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateCubicSmoother(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateQuinticSmoother(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::PlannerBasePtr CreateQuinticTrajectoryRetimer(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
}

const std::string RPlannersPlugin::_pluginname = "RPlannersPlugin";

RPlannersPlugin::RPlannersPlugin()
{
    _interfaces[PT_Planner].push_back("RAStar");
    _interfaces[PT_Planner].push_back("BiRRT");
    _interfaces[PT_Planner].push_back("BasicRRT");
    _interfaces[PT_Planner].push_back("ExplorationRRT");
    _interfaces[PT_Planner].push_back("GraspGradient");
    _interfaces[PT_Planner].push_back("shortcut_linear");
    _interfaces[PT_Planner].push_back("LinearTrajectoryRetimer");
    _interfaces[PT_Planner].push_back("ParabolicTrajectoryRetimer");
    _interfaces[PT_Planner].push_back("ParabolicTrajectoryRetimer2");
    _interfaces[PT_Planner].push_back("CubicTrajectoryRetimer");
    _interfaces[PT_Planner].push_back("CubicTrajectoryRetimer2");
    _interfaces[PT_Planner].push_back("WorkspaceTrajectoryTracker");
    _interfaces[PT_Planner].push_back("LinearSmoother");
    _interfaces[PT_Planner].push_back("ParabolicSmoother");
    _interfaces[PT_Planner].push_back("ParabolicSmoother2");
    _interfaces[PT_Planner].push_back("ConstraintParabolicSmoother");
    _interfaces[PT_Planner].push_back("CubicSmoother");
    _interfaces[PT_Planner].push_back("QuinticSmoother");
    _interfaces[PT_Planner].push_back("QuinticTrajectoryRetimer");
}

RPlannersPlugin::~RPlannersPlugin() {}

OpenRAVE::InterfaceBasePtr RPlannersPlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_Planner:
        if( interfacename == "rastar" || interfacename == "ra*" ) {
            return CreateRandomizedAStarPlanner(penv,sinput);
        }
        else if( interfacename == "birrt") {
            return boost::make_shared<BirrtPlanner>(penv);
        }
        else if( interfacename == "rbirrt") {
            RAVELOG_WARN("rBiRRT is deprecated, use BiRRT\n");
            return boost::make_shared<BirrtPlanner>(penv);
        }
        else if( interfacename == "basicrrt") {
            return boost::make_shared<BasicRrtPlanner>(penv);
        }
        else if( interfacename == "explorationrrt" ) {
            return boost::make_shared<ExplorationPlanner>(penv);
        }
        //else if( interfacename == "graspgradient" ) {
        //    return CreateGraspGradientPlanner(penv,sinput);
        //}
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
        else if( interfacename == "cubictrajectoryretimer2" ) {
            return rplanners::CreateCubicTrajectoryRetimer2(penv,sinput);
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
        else if( interfacename == "cubicsmoother" ) {
            return rplanners::CreateCubicSmoother(penv, sinput);
        }
        else if( interfacename == "quinticsmoother" ) {
            return rplanners::CreateQuinticSmoother(penv, sinput);
        }
        else if( interfacename == "quintictrajectoryretimer" ) {
            return rplanners::CreateQuinticTrajectoryRetimer(penv, sinput);
        }
        break;
    default:
        break;
    }
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& RPlannersPlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& RPlannersPlugin::GetPluginName() const
{
    return _pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new RPlannersPlugin();
}