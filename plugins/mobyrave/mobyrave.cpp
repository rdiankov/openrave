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

#include "mobyrave.h"

#include "mobyphysics.h"
#include "mobycontroller.h"
#include "mobyreplaycontroller.h"

// create moby physics shared pointer
OpenRAVE::PhysicsEngineBasePtr CreateMobyPhysics(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput)
{
    return boost::make_shared<MobyPhysics>(penv, sinput);
}

static std::string MobyRavePlugin::_pluginname = "MobyRavePlugin";

MobyRavePlugin::MobyRavePlugin()
{
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(OpenRAVE::PT_PhysicsEngine,"mobyphysics",MobyPhysics::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(OpenRAVE::PT_Controller,"mobycontroller",MobyController::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(OpenRAVE::PT_Controller,"mobyreplaycontroller",MobyReplayController::CreateXMLReader));
    _interfaces[OpenRAVE::PT_PhysicsEngine].push_back("moby");
    _interfaces[OpenRAVE::PT_Controller].push_back("moby");
    _interfaces[OpenRAVE::PT_Controller].push_back("mobyreplay");
}

MobyRavePlugin::~MobyRavePlugin() {}

OpenRAVE::InterfaceBasePtr MobyRavePlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch( type ) {
    case OpenRAVE::PT_PhysicsEngine:
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

const RavePlugin::InterfaceMap& MobyRavePlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& MobyRavePlugin::GetPluginName() const
{
    return _pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new MobyRavePlugin();
}