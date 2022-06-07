// -*- coding: utf-8 -*-
// Copyright (C) 2014 Alejandro Perez & Rosen Diankov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OPENRAVE_PLUGIN_CONFIGURATIONCACHE_H
#define OPENRAVE_PLUGIN_CONFIGURATIONCACHE_H

#include <openrave/plugin.h>
#include "configurationcachetree.h"

namespace configurationcache
{
OpenRAVE::CollisionCheckerBasePtr CreateCacheCollisionChecker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::SpaceSamplerBasePtr CreateConfigurationJitterer(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::SpaceSamplerBasePtr CreateWorkspaceConfigurationJitterer(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
}

struct ConfigurationCachePlugin : public RavePlugin {
    ConfigurationCachePlugin()
    {
        _interfaces[PT_CollisionChecker].push_back("CacheChecker");
        _interfaces[PT_SpaceSampler].push_back("ConfigurationJitterer");
        _interfaces[PT_SpaceSampler].push_back("WorkspaceConfigurationJitterer");
    }

    ~ConfigurationCachePlugin() override {}

    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override
    {
        switch(type) {
        case PT_CollisionChecker:
            if( interfacename == "cachechecker") {
                return configurationcache::CreateCacheCollisionChecker(penv,sinput);
            }
            break;
        case PT_SpaceSampler:
            if( interfacename == "configurationjitterer" ) {
                return configurationcache::CreateConfigurationJitterer(penv,sinput);
            }
            if( interfacename == "workspaceconfigurationjitterer" ) {
                return configurationcache::CreateWorkspaceConfigurationJitterer(penv,sinput);
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
        static std::string pluginname = "ConfigurationCachePlugin";
        return pluginname;
    }

private:
    InterfaceMap _interfaces;
};

#endif // OPENRAVE_PLUGIN_CONFIGURATIONCACHE_H