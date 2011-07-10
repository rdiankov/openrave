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
#include "plugindefs.h"
#include "loggingmodule.h"
#include <openrave/plugin.h>

#ifdef ENABLE_VIDEORECORDING
ModuleBasePtr CreateViewerRecorder(EnvironmentBasePtr penv, std::istream& sinput);
void DestroyViewerRecordingStaticResources();
#endif

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_Module:
        if( interfacename == "logging") {
            return InterfaceBasePtr(new LoggingModule(penv));
        }
#ifdef ENABLE_VIDEORECORDING
        else if( interfacename == "viewerrecorder" ) {
            return CreateViewerRecorder(penv,sinput);
        }
#endif
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_Module].push_back("Logging");
#ifdef ENABLE_VIDEORECORDING
    info.interfacenames[OpenRAVE::PT_Module].push_back("ViewerRecorder");
#endif
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
#ifdef ENABLE_VIDEORECORDING
    DestroyViewerRecordingStaticResources();
#endif
}
