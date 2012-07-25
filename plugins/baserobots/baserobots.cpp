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
#include "plugindefs.h"
#include <openrave/plugin.h>

static UserDataPtr s_RegisteredReader;

RobotBasePtr CreateCollisionMapRobot(EnvironmentBasePtr penv, std::istream& sinput);
RobotBasePtr CreateConveyorRobot(EnvironmentBasePtr penv, std::istream& sinput);

void RegisterCollisionMapRobotReaders(std::list< UserDataPtr >& listRegisteredReaders);
void RegisterConveyorReaders(std::list< UserDataPtr >& listRegisteredReaders);

static std::list< UserDataPtr >* s_listRegisteredReaders = NULL; ///< have to make it a pointer in order to prevent static object destruction from taking precedence

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( !s_listRegisteredReaders ) {
        s_listRegisteredReaders = new list< UserDataPtr >();
        RegisterCollisionMapRobotReaders(*s_listRegisteredReaders);
        RegisterConveyorReaders(*s_listRegisteredReaders);
    }

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
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Robot].push_back("CollisionMapRobot");
    info.interfacenames[PT_Robot].push_back("Conveyor");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    delete s_listRegisteredReaders;
    s_listRegisteredReaders = NULL;
}
