// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "grasper.h"
#include "plugindefs.h"

OpenRAVE::PlannerBasePtr CreateGrasperPlanner(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::ModuleBasePtr CreateGrasperModule(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);

const std::string GrasperPlugin::_pluginname = "GrasperPlugin";

GrasperPlugin::GrasperPlugin()
{
    _interfaces[OpenRAVE::PT_Planner].push_back("Grasper");
    _interfaces[OpenRAVE::PT_Module].push_back("Grasper");
}

GrasperPlugin::~GrasperPlugin() {}

OpenRAVE::InterfaceBasePtr GrasperPlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_Planner:
        if( interfacename == "grasper" ) {
            return CreateGrasperPlanner(penv,sinput);
        }
        break;
    case OpenRAVE::PT_Module:
        if( interfacename == "grasper") {
            return CreateGrasperModule(penv,sinput);
        }
        break;
    default:
        break;
    }
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& GrasperPlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& GrasperPlugin::GetPluginName() const
{
    return _pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new GrasperPlugin();
}