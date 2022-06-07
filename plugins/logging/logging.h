// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_PLUGIN_LOGGING_H
#define OPENRAVE_PLUGIN_LOGGING_H

// Plugin exposes 3 functions to OpenRAVE.
#include "plugindefs.h"
#include <openrave/plugin.h>

#ifdef ENABLE_VIDEORECORDING
OpenRAVE::ModuleBasePtr CreateViewerRecorder(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
void DestroyViewerRecordingStaticResources();
#endif

struct LoggingPlugin : public RavePlugin {
    LoggingPlugin()
    {
#ifdef ENABLE_VIDEORECORDING
        _interfaces[OpenRAVE::PT_Module].push_back("ViewerRecorder");
#endif
    }

    ~LoggingPlugin() override
    {
        Destroy();
    }

    void Destroy() override
    {
#ifdef ENABLE_VIDEORECORDING
    DestroyViewerRecordingStaticResources();
#endif
    }

    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override
    {
        switch(type) {
        case OpenRAVE::PT_Module:
    #ifdef ENABLE_VIDEORECORDING
            if( interfacename == "viewerrecorder" ) {
                return CreateViewerRecorder(penv,sinput);
            }
    #endif
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
        static std::string pluginname = "LoggingPlugin";
        return pluginname;
    }

private:
    InterfaceMap _interfaces;
};

#endif // OPENRAVE_PLUGIN_LOGGING_H