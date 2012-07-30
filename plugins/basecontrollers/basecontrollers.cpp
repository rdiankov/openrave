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
#include "plugindefs.h"
#include <openrave/plugin.h>

ControllerBasePtr CreateIdealController(EnvironmentBasePtr penv, std::istream& sinput);
ControllerBasePtr CreateIdealVelocityController(EnvironmentBasePtr penv, std::istream& sinput);
ControllerBasePtr CreateRedirectController(EnvironmentBasePtr penv, std::istream& sinput);

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_Controller:
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
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Controller].push_back("IdealController");
    info.interfacenames[PT_Controller].push_back("IdealVelocityController");
    info.interfacenames[PT_Controller].push_back("RedirectController");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
