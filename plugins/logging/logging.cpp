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

// Plugin exposes 3 functions to OpenRAVE.
#include "logging.h"
#include "plugindefs.h"

#ifdef ENABLE_VIDEORECORDING
OpenRAVE::ModuleBasePtr CreateViewerRecorder(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);
void DestroyViewerRecordingStaticResources();
#endif

const std::string LoggingPlugin::_pluginname = "LoggingPlugin";

LoggingPlugin::LoggingPlugin()
{
#ifdef ENABLE_VIDEORECORDING
    _interfaces[OpenRAVE::PT_Module].push_back("ViewerRecorder");
#endif
}

LoggingPlugin::~LoggingPlugin()
{
    Destroy();
}

void LoggingPlugin::Destroy()
{
#ifdef ENABLE_VIDEORECORDING
DestroyViewerRecordingStaticResources();
#endif
}

OpenRAVE::InterfaceBasePtr LoggingPlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
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

const RavePlugin::InterfaceMap& LoggingPlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& LoggingPlugin::GetPluginName() const
{
    return _pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new LoggingPlugin();
}