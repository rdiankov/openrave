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
#include "plugindefs.h"

#include <openrave/plugin.h>

CollisionCheckerBasePtr CreateBulletCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput);
PhysicsEngineBasePtr CreateBulletPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput);

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_CollisionChecker:
        if( interfacename == "bullet") {
            return CreateBulletCollisionChecker(penv,sinput);
        }
        break;
    case OpenRAVE::PT_PhysicsEngine:
        if( interfacename == "bullet") {
            return CreateBulletPhysicsEngine(penv,sinput);
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_CollisionChecker].push_back("bullet");
    info.interfacenames[OpenRAVE::PT_PhysicsEngine].push_back("bullet");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
