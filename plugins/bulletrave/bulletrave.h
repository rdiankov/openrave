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
#ifndef OPENRAVE_PLUGIN_BULLETRAVE_H
#define OPENRAVE_PLUGIN_BULLETRAVE_H

#include "plugindefs.h"
#include "bulletphysics.h"
#include "bulletcollision.h"
#include <openrave/plugin.h>

OpenRAVE::CollisionCheckerBasePtr CreateBulletCollisionChecker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);

struct BulletRavePlugin : public RavePlugin {
    BulletRavePlugin()
    {
        s_listRegisteredReaders.push_back(RaveRegisterXMLReader(OpenRAVE::PT_PhysicsEngine,"bulletproperties",BulletPhysicsEngine::CreateXMLReader));
        _interfaces[OpenRAVE::PT_CollisionChecker].push_back("bullet");
        _interfaces[OpenRAVE::PT_PhysicsEngine].push_back("bullet");
    }

    ~BulletRavePlugin() override {}

    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override
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
        return OpenRAVE::InterfaceBasePtr();
    }

    const InterfaceMap& GetInterfaces() const override
    {
        return _interfaces;
    }

    const std::string& GetPluginName() const override
    {
        static std::string pluginname = "BulletRavePlugin";
        return pluginname;
    }

private:
    InterfaceMap _interfaces;
    std::list<OpenRAVE::UserDataPtr> s_listRegisteredReaders;
};

#endif // OPENRAVE_PLUGIN_BULLETRAVE_H