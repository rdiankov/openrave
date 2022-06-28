// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_PLUGIN_RMANIPULATION_H
#define OPENRAVE_PLUGIN_RMANIPULATION_H

#include "plugindefs.h"
#include <openrave/plugin.h>

OpenRAVE::ModuleBasePtr CreateBaseManipulation(OpenRAVE::EnvironmentBasePtr penv);
//OpenRAVE::ModuleBasePtr CreateTaskCaging(OpenRAVE::EnvironmentBasePtr penv);
OpenRAVE::ModuleBasePtr CreateTaskManipulation(OpenRAVE::EnvironmentBasePtr penv);
OpenRAVE::ModuleBasePtr CreateVisualFeedback(OpenRAVE::EnvironmentBasePtr penv);

struct RManipulationPlugin : public RavePlugin {
    RManipulationPlugin()
    {
        _interfaces[PT_Module].push_back("BaseManipulation");
        _interfaces[PT_Module].push_back("TaskManipulation");
        _interfaces[PT_Module].push_back("TaskCaging");
        _interfaces[PT_Module].push_back("VisualFeedback");
    }

    ~RManipulationPlugin() override {}

    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override
    {
        switch(type) {
        case PT_Module:
            if( interfacename == "basemanipulation") {
                return CreateBaseManipulation(penv);
            }
            else if( interfacename == "taskmanipulation" ) {
                return CreateTaskManipulation(penv);
            }
            //else if( interfacename == "taskcaging") {
            //    return CreateTaskCaging(penv);
            //}
            else if( interfacename == "visualfeedback") {
                return CreateVisualFeedback(penv);
            }
            break;
        default:
            break;
        }
        return OpenRAVE::InterfaceBasePtr();
    }

    const InterfaceMap& GetInterfaces() const override
    {
        return _interfaces;
    }

    const std::string& GetPluginName() const override
    {
        static std::string pluginname = "RManipulationPlugin";
        return pluginname;
    }

private:
    InterfaceMap _interfaces;
};

#endif // OPENRAVE_PLUGIN_RMANIPULATION_H