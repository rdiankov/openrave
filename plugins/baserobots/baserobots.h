// -*- coding: utf-8 --*
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_PLUGIN_BASEROBOTS_H
#define OPENRAVE_PLUGIN_BASEROBOTS_H

#include "plugindefs.h"
#include <openrave/plugin.h>

OpenRAVE::RobotBasePtr CreateCollisionMapRobot(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::RobotBasePtr CreateConveyorRobot(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);

void RegisterCollisionMapRobotReaders(std::list< OpenRAVE::UserDataPtr >& listRegisteredReaders);
void RegisterConveyorReaders(std::list< OpenRAVE::UserDataPtr >& listRegisteredReaders);

struct BaseRobotsPlugin : public RavePlugin {
    BaseRobotsPlugin()
    {
        RegisterCollisionMapRobotReaders(s_listRegisteredReaders);
        RegisterConveyorReaders(s_listRegisteredReaders);
        _interfaces[PT_Robot].push_back("CollisionMapRobot");
        _interfaces[PT_Robot].push_back("Conveyor");
    }

    ~BaseRobotsPlugin() override {}

    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override
    {
        switch(type) {
        case PT_Robot:
            if( interfacename == "collisionmaprobot") {
                return CreateCollisionMapRobot(penv,sinput);
            }
            else if( interfacename == "conveyor" ) {
                return CreateConveyorRobot(penv,sinput);
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
        static std::string pluginname = "BaseRobotsPlugin";
        return pluginname;
    }

private:
    InterfaceMap _interfaces;
    std::list<OpenRAVE::UserDataPtr> s_listRegisteredReaders;
};

#endif // OPENRAVE_PLUGIN_BASEROBOTS_H