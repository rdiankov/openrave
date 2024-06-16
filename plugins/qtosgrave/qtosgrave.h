// -*- coding: utf-8 -*-
// Copyright (C) 2012 Gustavo Puche, Rosen Diankov, OpenGrasp Team
//
// OpenRAVE Qt/OpenSceneGraph Viewer is licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OPENRAVE_PLUGIN_QTOSGRAVE_H
#define OPENRAVE_PLUGIN_QTOSGRAVE_H

#include <openrave/plugin.h>

struct QtOSGRavePlugin : public RavePlugin {
    QtOSGRavePlugin();
    ~QtOSGRavePlugin() override;

    void Destroy() override;
    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override;
    const InterfaceMap& GetInterfaces() const override;
    const std::string& GetPluginName() const override;

private:
    InterfaceMap _interfaces;
};

#endif // OPENRAVE_PLUGIN_QTOSGRAVE_H
