// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#include "oderave.h"

#include "plugindefs.h"
#include "odecollision.h"
#include "odephysics.h"
#include "odecontroller.h"

const std::string ODERavePlugin::_pluginname = "ODERavePlugin";

ODERavePlugin::ODERavePlugin()
{
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(OpenRAVE::PT_PhysicsEngine,"odeproperties",ODEPhysicsEngine::CreateXMLReader));
    _interfaces[OpenRAVE::PT_CollisionChecker].push_back("ode");
    _interfaces[OpenRAVE::PT_PhysicsEngine].push_back("ode");
    _interfaces[OpenRAVE::PT_Controller].push_back("odevelocity");
}

ODERavePlugin::~ODERavePlugin() {}

OpenRAVE::InterfaceBasePtr ODERavePlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_CollisionChecker:
        if( interfacename == "ode")
            return boost::make_shared<ODECollisionChecker>(penv);
        break;
    case OpenRAVE::PT_PhysicsEngine:
        if( interfacename == "ode" )
            return boost::make_shared<ODEPhysicsEngine>(penv);
        break;
    case OpenRAVE::PT_Controller:
        if( interfacename == "odevelocity")
            return boost::make_shared<ODEVelocityController>(penv);
        break;
    default:
        break;
    }
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& ODERavePlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& ODERavePlugin::GetPluginName() const
{
    return _pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new ODERavePlugin();
}