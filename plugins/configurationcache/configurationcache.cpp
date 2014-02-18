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
#include <openrave/plugin.h>
#include "configurationcachetree.h"

using namespace OpenRAVE;

namespace configurationcache
{
CollisionCheckerBasePtr CreateCacheCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput);
SpaceSamplerBasePtr CreateConfigurationJitterer(EnvironmentBasePtr penv, std::istream& sinput);
}

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_CollisionChecker:
        if( interfacename == "cachechecker") {
            return InterfaceBasePtr(configurationcache::CreateCacheCollisionChecker(penv,sinput));
        }
        break;
    case PT_SpaceSampler:
        if( interfacename == "configurationjitterer" ) {
            return configurationcache::CreateConfigurationJitterer(penv,sinput);
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_CollisionChecker].push_back("CacheChecker");
    info.interfacenames[PT_SpaceSampler].push_back("ConfigurationJitterer");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
