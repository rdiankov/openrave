// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#include "basecontrollers.h"
#include <openrave/openrave.h>

OpenRAVE::ControllerBasePtr CreateIdealController(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::ControllerBasePtr CreateIdealVelocityController(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::ControllerBasePtr CreateRedirectController(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);

const std::string BaseControllersPlugin::_pluginname = "BaseControllersPlugin";

BaseControllersPlugin::BaseControllersPlugin()
{
    _interfaces[OpenRAVE::PT_Controller].push_back("IdealController");
    _interfaces[OpenRAVE::PT_Controller].push_back("IdealVelocityController");
    _interfaces[OpenRAVE::PT_Controller].push_back("RedirectController");
}

BaseControllersPlugin::~BaseControllersPlugin() {}

OpenRAVE::InterfaceBasePtr BaseControllersPlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_Controller:
        if( interfacename == "idealcontroller") {
            return CreateIdealController(penv,sinput);
        }
        else if( interfacename == "idealvelocitycontroller") {
            return CreateIdealVelocityController(penv,sinput);
        }
        else if( interfacename == "redirectcontroller" ) {
            return CreateRedirectController(penv,sinput);
        }
        break;
    default:
        break;
    }
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& BaseControllersPlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& BaseControllersPlugin::GetPluginName() const
{
    return _pluginname;
}

#if !OPENRAVE_STATIC_PLUGINS

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new BaseControllersPlugin();
}

#endif // OPENRAVE_STATIC_PLUGINS