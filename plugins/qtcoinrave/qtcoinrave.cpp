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
#include "qtcoinrave.h"

#include "qtcoin.h"

#include <QApplication>
#include <QLabel>

#include "qtcameraviewer.h"
#include <openrave/plugin.h>
#if defined(HAVE_X11_XLIB_H) && defined(Q_WS_X11)
#include <X11/Xlib.h>
#endif

OpenRAVE::ModuleBasePtr CreateIvModelLoader(OpenRAVE::EnvironmentBasePtr penv);

std::mutex g_mutexsoqt;
static int s_InitRefCount = 0;
static int s_SoQtArgc = 0; // has to be static!!
void EnsureSoQtInit()
{
    if( s_InitRefCount == 0 ) {
        ++s_InitRefCount;
        SoQt::init(s_SoQtArgc, NULL, NULL);
    }
}

QtCoinRavePlugin::QtCoinRavePlugin()
{
    _interfaces[OpenRAVE::PT_Viewer].push_back("QtCoin");
    _interfaces[OpenRAVE::PT_Viewer].push_back("QtCameraViewer");
    _interfaces[OpenRAVE::PT_Module].push_back("IvModelLoader");
}

QtCoinRavePlugin::~QtCoinRavePlugin()
{
    Destroy();
}

void QtCoinRavePlugin::Destroy()
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

OpenRAVE::InterfaceBasePtr QtCoinRavePlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
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
            std::lock_guard<std::mutex> lock(g_mutexsoqt);
            EnsureSoQtInit();
            //SoDBWriteLock dblock;
            return boost::make_shared<QtCoinViewer>(penv, sinput);
        }
        else if( interfacename == "qtcameraviewer" ) {
            return boost::make_shared<QtCameraViewer>(penv, sinput);
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
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& QtCoinRavePlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& QtCoinRavePlugin::GetPluginName() const
{
    static std::string pluginname = "QtCoinRavePlugin";
    return pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new QtCoinRavePlugin();
}