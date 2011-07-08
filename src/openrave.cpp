// -*- coding: utf-8 -*-
// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "libopenrave-core/openrave-core.h"
#include <set>

using namespace OpenRAVE;
using namespace std;

#include <stdio.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

#include <signal.h>
void sigint_handler(int sig);
void MainOpenRAVEThread();

#ifdef _WIN32
#define usleep(micro) Sleep((micro)/1000)
#else
#define strnicmp strncasecmp
#define stricmp strcasecmp
#include <sys/time.h>
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

static bool bDisplayGUI = true, bShowGUI = true;

static EnvironmentBasePtr s_penv;
static boost::shared_ptr<boost::thread> s_mainThread;
static string s_sceneFile;
static string s_saveScene; // if not NULL, saves the scene and exits
static boost::shared_ptr<string> s_viewerName;

static list< pair<string, string> > s_listModules; // modules to initially create
static vector<string> vResourceFiles; // xml files to open
static int s_WindowWidth = 1024, s_WindowHeight = 768;
static int s_WindowPosX, s_WindowPosY;
static bool s_bSetWindowPosition = false;
int g_argc;
char** g_argv;
static bool s_bThreadDestroyed = false;
static const char* s_geometryextentsions[] = { "iv","vrml","wrl","stl","blend","3ds","ase","obj","ply","dxf","lwo","lxo","ac","ms3d","x","mesh.xml","irrmesh","irr","nff","off","raw"};
#ifndef _WIN32
#include <sys/stat.h>
#include <sys/types.h>
#endif

int main(int argc, char ** argv)
{
    g_argc = argc;
    g_argv = argv;

    int nServPort = 4765;

    DebugLevel debuglevel = Level_Info;
    list<string> listLoadPlugins;
    string collisionchecker, physicsengine, servername="textserver";
    bool bListPlugins = false;

    std::set<std::string> geometryextensions;
    for(size_t iext = 0; iext < sizeof(s_geometryextentsions)/sizeof(s_geometryextentsions[0]); ++iext) {
        geometryextensions.insert(s_geometryextentsions[iext]);
    }

    // parse command line arguments
    int i = 1;
    while(i < argc) {
        if((stricmp(argv[i], "-h") == 0)||(stricmp(argv[i], "-?") == 0)||(stricmp(argv[i], "/?") == 0)||(stricmp(argv[i], "--help") == 0)||(stricmp(argv[i], "-help") == 0)) {
            RAVELOG_INFO("OpenRAVE Usage\n"
                         "--nogui             Run without a GUI (does not initialize the graphics engine nor communicate with any window manager)\n"
                         "--hidegui           Run with a hidden GUI, this allows 3D rendering and images to be captured\n"
                         "--listplugins       List all plugins and the interfaces they provide\n"
                         "--loadplugin [path] load a plugin at the following path\n"
                         "--serverport [port] start up the server on a specific port (default is 4765)\n"
                         "--collision [name]  Default collision checker to use\n"
                         "--viewer [name]     Default viewer to use\n"
                         "--server [name]     Default server to use\n"
                         "--physics [name]    Default physics engine to use\n"
                         "-d [debug-level]   start up OpenRAVE with the specified debug level (higher numbers print more).\n"
                         "                   Default level is 2 for release builds, and 4 for debug builds.\n"
                         "-wdims [width] [height] start up the GUI window with these dimensions\n"
                         "-wpos x y set the position of the GUI window\n"
                         "--module [modulename] [args] Start openrave with a module. If args involves spaces, surround it with double quotes. args is optional.\n"
                         "--version          Output the current openrave version\n\n"
                         "-f [scene]         Load a openrave environment file\n\n");
            return 0;
        }
        else if((stricmp(argv[i], "--loadplugin") == 0)||(stricmp(argv[i], "-loadplugin") == 0)) {
            listLoadPlugins.push_back(argv[i+1]);
            i += 2;
        }
        else if((stricmp(argv[i], "--listplugins") == 0)||(stricmp(argv[i], "-listplugins") == 0)) {
            bListPlugins = true;
            i++;
        }
        else if( stricmp(argv[i], "--version") == 0 ) {
            printf("%s\n",OPENRAVE_VERSION_STRING);
            return 0;
        }
        else if( stricmp(argv[i], "-f") == 0 ) {
            s_sceneFile = argv[i+1];
            i += 2;
        }
        else if( stricmp(argv[i], "-save") == 0 ) {
            s_saveScene = argv[i+1];
            i += 2;
        }
        else if((stricmp(argv[i], "--collision") == 0)||(stricmp(argv[i], "-collision") == 0)) {
            collisionchecker = argv[i+1];
            i += 2;
        }
        else if((stricmp(argv[i], "--viewer") == 0)||(stricmp(argv[i], "-viewer") == 0)) {
            s_viewerName.reset(new string(argv[i+1]));
            i += 2;
        }
        else if((stricmp(argv[i], "--physics") == 0)||(stricmp(argv[i], "-physics") == 0)) {
            physicsengine = argv[i+1];
            i += 2;
        }
        else if((stricmp(argv[i],"--server") == 0)||(stricmp(argv[i],"-server") == 0)) {
            servername = argv[i+1];
            i += 2;
        }
        else if((stricmp(argv[i], "--module") == 0)||(stricmp(argv[i], "-problem") == 0)) {
            s_listModules.push_back(pair<string, string>(argv[i+1], ""));
            i += 2;

            if((i < argc)&&(argv[i][0] != '-')) {
                // set the args
                s_listModules.back().second = argv[i];
                i++;
            }
        }
        else if((stricmp(argv[i], "--nogui") == 0)||(stricmp(argv[i], "-nogui") == 0)) {
            bDisplayGUI = false;
            i++;
        }
        else if((stricmp(argv[i], "--hidegui") == 0)||(stricmp(argv[i], "-hidegui") == 0)) {
            bShowGUI = false;
            i++;
        }
        else if( stricmp(argv[i], "-d") == 0 ) {
            debuglevel = (DebugLevel)atoi(argv[i+1]);
            i += 2;
        }
        else if( stricmp(argv[i], "-wdims") == 0 ) {
            s_WindowWidth = atoi(argv[i+1]);
            s_WindowHeight = atoi(argv[i+2]);
            i += 3;
        }
        else if( stricmp(argv[i], "-wpos") == 0 ) {
            s_WindowPosX = atoi(argv[i+1]);
            s_WindowPosY = atoi(argv[i+2]);
            s_bSetWindowPosition = true;
            i += 3;
        }
        else if((stricmp(argv[i], "--serverport") == 0)||(stricmp(argv[i], "-serverport") == 0)) {
            nServPort = atoi(argv[i+1]);
            i += 2;
        }
        else {
            // try to load as xml files
            vResourceFiles.push_back(argv[i]);
            i++;
        }
    }

    RaveInitialize(true);

    // create environment and start a command-line controlled simulation
    RaveSetDebugLevel(debuglevel);
    s_penv = RaveCreateEnvironment();
    RaveSetDebugLevel(debuglevel);
    for(list<string>::iterator it = listLoadPlugins.begin(); it != listLoadPlugins.end(); ++it) {
        RaveLoadPlugin(*it);
    }

    // add a signal handler
    signal(SIGINT,sigint_handler); // control C

    if( bListPlugins ) {

        std::list< std::pair<std::string, PLUGININFO> > plugins;
        std::list< std::pair<std::string, PLUGININFO> >::iterator itplugin;
        RaveGetPluginInfo(plugins);

        // output all the plugins and exit
        vector<string>::const_iterator itnames;

        vector<string>::iterator itname;
        stringstream ss;

        ss << endl << "Number of plugins: " << plugins.size() << endl;

        stringstream buf;
        std::map<InterfaceType,std::string>::const_iterator ittype;
        FORIT(ittype,RaveGetInterfaceNamesMap()) {
            vector<string> names;
            FORIT(itplugin, plugins) {
                FORIT(itnames, itplugin->second.interfacenames[ittype->first]) {
                    buf.str("");
#ifdef _WIN32
                    buf << "  " << *itnames << " - " << itplugin->first;
#else
                    buf << "  " << ChangeTextColor(0,OPENRAVECOLOR_DEBUGLEVEL) << *itnames << ResetTextColor() << " - " << itplugin->first;
#endif
                    names.push_back(buf.str());
                }
            }
            sort(names.begin(), names.end());
#ifdef _WIN32
            ss << ittype->second << ": " << names.size() << endl;
#else
            ss << ChangeTextColor(0,OPENRAVECOLOR_WARNLEVEL) << ittype->second << ": " << ResetTextColor() << names.size() << endl;
#endif
            FORIT(itname, names) {
                ss << itname->c_str() << endl;
            }
        }
        printf("%s",ss.str().c_str());
        s_penv->Destroy();
        return 0;
    }

    if((servername.size() > 0)&&(nServPort > 0)) {
        ModuleBasePtr pserver = RaveCreateModule(s_penv, servername);
        if( !!pserver ) {
            stringstream ss;
            ss << nServPort;
            if( s_penv->AddModule(pserver,ss.str()) != 0 )
                RAVELOG_WARN("failed to load server\n");
        }
    }

    if( collisionchecker.size() > 0 ) {
        CollisionCheckerBasePtr p = RaveCreateCollisionChecker(s_penv, collisionchecker);
        if( !!p ) {
            s_penv->SetCollisionChecker(p);
        }
    }
    if( physicsengine.size() > 0 ) {
        PhysicsEngineBasePtr p = RaveCreatePhysicsEngine(s_penv, physicsengine);
        if( !!p ) {
            s_penv->SetPhysicsEngine(p);
        }
    }

    s_bThreadDestroyed = false;
    s_mainThread.reset(new boost::thread(boost::bind(MainOpenRAVEThread)));
    s_mainThread->join();
    s_penv.reset();
    RaveDestroy();
    return 0;
}

// use to control openrave
void MainOpenRAVEThread()
{
    EnvironmentBasePtr penv = s_penv; // need to do this since s_penv can be reset at any time
    ViewerBasePtr pviewer;
    if( bDisplayGUI && (!s_viewerName ||(s_viewerName->size()>0)) ) {
        // find a viewer
        if( !!s_viewerName &&(s_viewerName->size() > 0)) {
            pviewer = RaveCreateViewer(penv, *s_viewerName);
        }
        if( !pviewer ) {
            boost::array<string,1> viewer_prefs = { { "qtcoin"}};
            for(size_t i = 0; i < viewer_prefs.size(); ++i) {
                pviewer = RaveCreateViewer(penv, viewer_prefs[i]);
                if( !!pviewer ) {
                    break;
                }
            }
        }

        if( !pviewer ) { // take any viewer
            std::map<InterfaceType, std::vector<std::string> > interfacenames;
            RaveGetLoadedInterfaces(interfacenames);
            std::vector<std::string>::const_iterator itname;
            FORIT(itname, interfacenames[PT_Viewer]) {
                pviewer = RaveCreateViewer(penv, *itname);
                if( !!pviewer ) {
                    break;
                }
            }
        }

        if( !pviewer ) {
            RAVELOG_WARN("failed to find an OpenRAVE viewer.\n");
        }
        else {
            RAVELOG_DEBUG("using %s viewer\n", pviewer->GetXMLId().c_str());
            pviewer->SetSize(s_WindowWidth, s_WindowHeight);
            if( s_bSetWindowPosition ) {
                pviewer->Move(s_WindowPosX,s_WindowPosY);
            }
            penv->AddViewer(pviewer);
        }
    }

    {
        EnvironmentMutex::scoped_lock lock(penv->GetMutex());

        if( s_sceneFile.size() > 0 ) {
            penv->Load(s_sceneFile);
        }
        vector<string>::iterator it;
        FORIT(it, vResourceFiles) {
            penv->Load(*it);
        }
        list< pair<string, string> >::iterator itprob;
        FORIT(itprob, s_listModules) {
            ModuleBasePtr prob = RaveCreateModule(penv, itprob->first);
            if( !!prob ) {
                penv->AddModule(prob, itprob->second);
            }
        }

        if( s_saveScene.size() > 0 ) {
            penv->Save(s_saveScene);
            s_bThreadDestroyed = true;
            return;
        }
    }

    // need to keep a local pointer around to guarantee destruction order
    if( !!pviewer ) {
        pviewer->main(bShowGUI);
        s_bThreadDestroyed = true;
    }
    else {
        while(!s_bThreadDestroyed) {
            usleep(10000);
        }
    }
    if( !!penv ) {
        penv->Remove(pviewer);
        penv.reset();
        s_penv.reset();
        int iter = 0;
        while(pviewer.use_count() > 1 && iter < 200) {
            RAVELOG_DEBUG("viewer use count > 1, waiting for others to release viewer so can guarantee destruction in correct thread\n");
            usleep(10000);
            ++iter;
        }
    }
}

void sigint_handler(int sig)
{
    RaveDestroy();
    s_penv.reset();
#ifndef _WIN32
    // have to let the default sigint properly shutdown the program
    signal(SIGINT, SIG_DFL);
    kill(getpid(), SIGINT);
#endif
}
