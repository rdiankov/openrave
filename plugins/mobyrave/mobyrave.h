// -*- coding: utf-8 -*-
// Copyright (c) 2015 James Taylor, Rosen Diankov
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
#ifndef OPENRAVE_PLUGIN_MOBYRAVE_H
#define OPENRAVE_PLUGIN_MOBYRAVE_H

#include "mobyphysics.h"
#include "mobycontroller.h"
#include "mobyreplaycontroller.h"

// create moby physics shared pointer
OpenRAVE::PhysicsEngineBasePtr CreateMobyPhysics(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput)
{
    return boost::make_shared<MobyPhysics>(penv, sinput);
}

struct MobyRavePlugin : public RavePlugin {
    MobyRavePlugin()
    {
        s_listRegisteredReaders.push_back(RaveRegisterXMLReader(OpenRAVE::PT_PhysicsEngine,"mobyphysics",MobyPhysics::CreateXMLReader));
        s_listRegisteredReaders.push_back(RaveRegisterXMLReader(OpenRAVE::PT_Controller,"mobycontroller",MobyController::CreateXMLReader));
        s_listRegisteredReaders.push_back(RaveRegisterXMLReader(OpenRAVE::PT_Controller,"mobyreplaycontroller",MobyReplayController::CreateXMLReader));
        _interfaces[PT_PhysicsEngine].push_back("moby");
        _interfaces[OpenRAVE::PT_Controller].push_back("moby");
        _interfaces[OpenRAVE::PT_Controller].push_back("mobyreplay");
    }

    ~MobyRavePlugin() override {}

    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override
    {
        switch( type ) {
        case PT_PhysicsEngine:
            if( interfacename == "moby" )
            {
                return CreateMobyPhysics(penv,sinput);
            }
            break;
        case OpenRAVE::PT_Controller:
            if( interfacename == "moby") {
                return boost::make_shared<MobyController>(penv,sinput);
            }
            else if( interfacename == "mobyreplay") {
                return boost::make_shared<MobyReplayController>(penv,sinput);
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
        static std::string pluginname = "MobyRavePlugin";
        return pluginname;
    }

private:
    InterfaceMap _interfaces;
    std::list<OpenRAVE::UserDataPtr> s_listRegisteredReaders;
};

#endif // OPENRAVE_PLUGIN_MOBYRAVE_H