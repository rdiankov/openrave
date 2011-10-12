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
#include "plugindefs.h"
#include "collisionmaprobot.h"
#include <openrave/plugin.h>

static boost::shared_ptr<void> s_RegisteredReader;

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( !s_RegisteredReader ) {
        /// as long as this pointer is valid, the reader will remain registered
        s_RegisteredReader = RaveRegisterXMLReader(PT_Robot,"collisionmap",CollisionMapRobot::CreateXMLReader);
    }

    switch(type) {
    case PT_Robot:
        if( interfacename == "collisionmaprobot") {
            return InterfaceBasePtr(new CollisionMapRobot(penv));
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
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    s_RegisteredReader.reset(); // unregister the reader
}
