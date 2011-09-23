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
#include "qtcoin.h"
#include "qtcameraviewer.h"
#include <openrave/plugin.h>

ModuleBasePtr CreateIvModelLoader(EnvironmentBasePtr penv);

boost::mutex g_mutexsoqt;

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    static int s_SoQtArgc = 0; // has to be static!!
    switch(type) {
    case PT_Viewer:
        if( interfacename == "qtcoin" ) {
            boost::mutex::scoped_lock lock(g_mutexsoqt);
            SoDBWriteLock dblock;
            if( QtCoinViewer::s_InitRefCount == 0 ) {
                ++QtCoinViewer::s_InitRefCount;
                SoQt::init(s_SoQtArgc, NULL, NULL);
            }
            return InterfaceBasePtr(new QtCoinViewer(penv));
        }
        else if( interfacename == "qtcameraviewer" ) {
            return InterfaceBasePtr(new QtCameraViewer(penv,sinput));
        }
        break;
    case PT_Module:
        if( interfacename == "ivmodelloader" ) {
            return CreateIvModelLoader(penv);
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Viewer].push_back("QtCoin");
    info.interfacenames[PT_Viewer].push_back("QtCameraViewer");
    info.interfacenames[PT_Module].push_back("IvModelLoader");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
