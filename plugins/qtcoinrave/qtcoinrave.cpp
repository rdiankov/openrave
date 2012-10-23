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
#if defined(HAVE_X11_XLIB_H) && defined(Q_WS_X11)
#include "X11/Xlib.h"
#endif

#include <QApplication>

ModuleBasePtr CreateIvModelLoader(EnvironmentBasePtr penv);

boost::mutex g_mutexsoqt;
static int s_InitRefCount = 0;
static int s_SoQtArgc = 0; // has to be static!!
void EnsureSoQtInit()
{
    if( s_InitRefCount == 0 ) {
        ++s_InitRefCount;
        SoQt::init(s_SoQtArgc, NULL, NULL);
    }
}

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_Viewer:
#if defined(HAVE_X11_XLIB_H) && defined(Q_WS_X11)
        // always check viewers since DISPLAY could change
        if ( XOpenDisplay( NULL ) == NULL ) {
            RAVELOG_WARN("no display detected, so cannot load viewer");
            return InterfaceBasePtr();
        }
#endif
        if( interfacename == "qtcoin" ) {
            // have to lock after initialized since call relies on SoDBP::globalmutex
            boost::mutex::scoped_lock lock(g_mutexsoqt);
            EnsureSoQtInit();
            //SoDBWriteLock dblock;
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
    if( s_InitRefCount > 0 ) {
        RAVELOG_WARN("SoQt releasing all memory\n");
        SoQt::done();
        s_InitRefCount = 0;
        // necessary since QApplication does not destroy all threads when last SoQt viewer is done
        //removePostedEvents - sometimes freezes on this function
        QApplication::quit();
    }
}
