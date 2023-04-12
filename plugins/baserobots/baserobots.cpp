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
#include "baserobots.h"
//#include "plugindefs.h"
#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions

OpenRAVE::RobotBasePtr CreateCollisionMapRobot(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::RobotBasePtr CreateConveyorRobot(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);

void RegisterCollisionMapRobotReaders(std::list< OpenRAVE::UserDataPtr >& listRegisteredReaders);
void RegisterConveyorReaders(std::list< OpenRAVE::UserDataPtr >& listRegisteredReaders);

const std::string BaseRobotsPlugin::_pluginname = "BaseRobotsPlugin";

BaseRobotsPlugin::BaseRobotsPlugin()
{
    RegisterCollisionMapRobotReaders(s_listRegisteredReaders);
    RegisterConveyorReaders(s_listRegisteredReaders);
    _interfaces[OpenRAVE::PT_Robot].push_back("CollisionMapRobot");
    _interfaces[OpenRAVE::PT_Robot].push_back("Conveyor");
}

BaseRobotsPlugin::~BaseRobotsPlugin() {}

OpenRAVE::InterfaceBasePtr BaseRobotsPlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_Robot:
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

const RavePlugin::InterfaceMap& BaseRobotsPlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& BaseRobotsPlugin::GetPluginName() const
{
    return _pluginname;
}

#if !OPENRAVE_STATIC_PLUGINS

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new BaseRobotsPlugin();
}

#endif // OPENRAVE_STATIC_PLUGINS