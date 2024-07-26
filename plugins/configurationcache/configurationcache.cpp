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
#include "configurationcache.h"
#include "configurationcachetree.h"

namespace configurationcache
{
OpenRAVE::CollisionCheckerBasePtr CreateCacheCollisionChecker(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::SpaceSamplerBasePtr CreateConfigurationJitterer(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
OpenRAVE::SpaceSamplerBasePtr CreateWorkspaceConfigurationJitterer(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
}

const std::string ConfigurationCachePlugin::_pluginname = "ConfigurationCachePlugin";

ConfigurationCachePlugin::ConfigurationCachePlugin()
{
    _interfaces[OpenRAVE::PT_CollisionChecker].push_back("CacheChecker");
    _interfaces[OpenRAVE::PT_SpaceSampler].push_back("ConfigurationJitterer");
    _interfaces[OpenRAVE::PT_SpaceSampler].push_back("WorkspaceConfigurationJitterer");
}

ConfigurationCachePlugin::~ConfigurationCachePlugin() {}

OpenRAVE::InterfaceBasePtr ConfigurationCachePlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_CollisionChecker:
        if( interfacename == "cachechecker") {
            return configurationcache::CreateCacheCollisionChecker(penv,sinput);
        }
        break;
    case OpenRAVE::PT_SpaceSampler:
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

const RavePlugin::InterfaceMap& ConfigurationCachePlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& ConfigurationCachePlugin::GetPluginName() const
{
    return _pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new ConfigurationCachePlugin();
}