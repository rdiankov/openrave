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
#include "basesamplers.h"

#include "mt19937ar.h"
#include "halton.h"
#include "robotconfiguration.h"
#include "bodyconfiguration.h"

BaseSamplersPlugin::BaseSamplersPlugin()
{
    _interfaces[OpenRAVE::PT_SpaceSampler].push_back("MT19937");
    _interfaces[OpenRAVE::PT_SpaceSampler].push_back("Halton");
    _interfaces[OpenRAVE::PT_SpaceSampler].push_back("RobotConfiguration");
    _interfaces[OpenRAVE::PT_SpaceSampler].push_back("BodyConfiguration");
}

BaseSamplersPlugin::~BaseSamplersPlugin() {}

OpenRAVE::InterfaceBasePtr BaseSamplersPlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_SpaceSampler:
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
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& BaseSamplersPlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& BaseSamplersPlugin::GetPluginName() const
{
    static std::string pluginname = "BaseSamplersPlugin";
    return pluginname;
}

#if !OPENRAVE_STATIC_PLUGINS

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new BaseSamplersPlugin();
}

#endif // OPENRAVE_STATIC_PLUGINS