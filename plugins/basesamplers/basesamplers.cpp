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
#include <openrave/plugin.h>
#include "mt19937ar.h"
#include "halton.h"
#include "robotconfiguration.h"
#include "bodyconfiguration.h"

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_SpaceSampler:
        if( interfacename == "mt19937") {
            return InterfaceBasePtr(new MT19937Sampler(penv,sinput));
        }
        else if( interfacename == "halton" ) {
            return InterfaceBasePtr(new HaltonSampler(penv,sinput));
        }
        else if( interfacename == "robotconfiguration" ) {
            return InterfaceBasePtr(new RobotConfigurationSampler(penv,sinput));
        }
        else if( interfacename == "bodyconfiguration" ) {
            return InterfaceBasePtr(new BodyConfigurationSampler(penv,sinput));
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_SpaceSampler].push_back("MT19937");
    info.interfacenames[PT_SpaceSampler].push_back("Halton");
    info.interfacenames[PT_SpaceSampler].push_back("RobotConfiguration");
    info.interfacenames[PT_SpaceSampler].push_back("BodyConfiguration");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
