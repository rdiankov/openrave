// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

/*!
 \mainpage OpenRAVE - Open Robotics and Animation Virtual Environment
                \b A testbed for planning algorithms

 \section intro Introduction

 This software package contains an interactive simulation application
 useful for visualizing, programming, and testing motion generation
 and control algorithms for mobile robots, humanoids, or autonomous
 animated characters.
 
 OpenRAVE is cross platform package written in C++ and uses Coin3d, OpenGL, Qt
 libraries for the graphics, GUI, and simulation. It is based on a plugin architecture.
 This makes it very easy to keep the core functionality and a basic set of modules open
 source while being able to extend functionality privately without interfering with the core
 code or core executable. The simulator itself will provide the glue and a common format
 for all plugins to talk to each other. Currently the plugins are arranged in 5 groups:
 robots, planners, controllers, sensors, and problems. 
*/

#include "libopenrave-core/openrave-core.h"
using namespace OpenRAVE;
using namespace std;

#include <stdio.h>
#include <boost/shared_ptr.hpp>

#ifndef _WIN32
#include <signal.h>
#endif

#ifdef _MSC_VER
#define PRIdS "Id"
#else
#define PRIdS "zd"
#endif

#ifndef _WIN32
#define strnicmp strncasecmp
#define stricmp strcasecmp
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x)/(sizeof( (x)[0] )))
#endif

inline string _stdwcstombs(const wchar_t* pname)
{
    string s;
    size_t len = wcstombs(NULL, pname, 0);
    if( len != (size_t)-1 ) {
        s.resize(len);
        wcstombs(&s[0], pname, len);
    }

    return s;
}

void sigint_handler(int);
void* MainOpenRAVEThread(void*p);

static bool bDisplayGUI = true, bShowGUI = true;

static EnvironmentBase* penv = NULL;
static boost::shared_ptr<RaveViewerBase> s_viewer;
static boost::shared_ptr<RaveServerBase> s_server;
static pthread_t s_mainThread;
static char *sceneFile = NULL;
static char *s_saveScene = NULL; // if not NULL, saves the scene and exits
static bool s_bTestScene = false;

static list< pair<char*, char*> > s_listProblems; // problems to initially create
static vector<char*> vIvFiles; // iv files to open
static vector<char*> vXMLFiles; // xml files to open
static int s_WindowWidth = 1024, s_WindowHeight = 768;
static int s_WindowPosX, s_WindowPosY;
static bool s_bSetWindowPosition = false;
int g_argc;
char** g_argv;
bool bThreadDestroyed = false;

#define OUTPUT_PLUGINS(vtype) { \
        names.resize(0); \
        FORIT(itplugin, plugins) { \
            FORIT(itnames, vtype) { \
                sprintf(buf, "  %S - %s", itnames->c_str(), itplugin->first.c_str()); \
                names.push_back(buf); \
            } \
        } \
        /* sort */ \
        sort(names.begin(), names.end()); \
        FORIT(itname, names) ss << itname->c_str() << endl; \
    }

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/types.h>
#endif

int main(int argc, char ** argv)
{
    g_argc = argc;
    g_argv = argv;

    int nServPort = 4765;
    
#ifndef _WIN32
    mkdir("~/.openrave",644);
#endif

    DebugLevel debuglevel = Level_Info;    
    list<string> listLoadPlugins;
    bool bListPlugins = false;
    // parse command line arguments
    int i = 1;
    while(i < argc) {
        if( stricmp(argv[i], "-h") == 0 || stricmp(argv[i], "-?") == 0 || stricmp(argv[i], "/?") == 0 || stricmp(argv[i], "--help") == 0 || stricmp(argv[i], "-help") == 0 ) {
            RAVEPRINT(L"RAVE Simulator Usage\n"
                L"-nogui             Run without a GUI (does not initialize the graphics engine nor communicate with any window manager)\n"
                L"-hidegui           Run with a hidden GUI, this allows 3D rendering and images to be captured\n"
                L"-listplugins       List all plugins and the interfaces they provide\n"
                L"-loadplugin [path] load a plugin at the following path\n"
                L"-server [port]     start up the server on a specific port (default is 4765)\n"
                L"-d [debug-level]   start up OpenRAVE with the specified debug level (higher numbers print more).\n"
                L"                   Default level is 2 for release builds, and 4 for debug builds.\n"
                L"--generateik robot [robotfile] ikfast [ikfast_executable] [freeparam [joint index]] output [filename] manipulator [index]"
                L"-wdims [width] [height] start up the GUI window with these dimensions\n"
                L"-wpos x y set the position of the GUI window\n"
                L"-problem [problemname] [args] Start openrave with a problem instance. If args involves spaces, surround it with double quotes. args is optional.\n"
                L"-f [scene]         Load a openrave environment file\n"
                      L"-testscene         If specified, openrave will exit after the scene is loaded. A non-zero values means something went wrong with the loading process.");
            return 0;
        }
        else if( stricmp(argv[i], "-loadplugin") == 0 ) {
            listLoadPlugins.push_back(argv[i+1]);
            i += 2;
        }
        else if( stricmp(argv[i], "-listplugins") == 0 ) {
            bListPlugins = true;
            i++;
        }
        else if( stricmp(argv[i], "-f") == 0 ) {
            sceneFile = argv[i+1];
            i += 2;
        }
        else if( stricmp(argv[i], "-save") == 0 ) {
            s_saveScene = argv[i+1];
            i += 2;
        }
        else if( stricmp(argv[i], "-problem") == 0 ) {
            s_listProblems.push_back(pair<char*, char*>(argv[i+1], NULL));
            i += 2;

            if( i < argc && argv[i][0] != '-' ) {
                // set the args
                s_listProblems.back().second = argv[i];
                i++;
            }
        }
        else if( stricmp(argv[i], "-nogui") == 0 ) {
            bDisplayGUI = false;
            i++;
        }
        else if( stricmp(argv[i], "-hidegui") == 0 ) {
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
        else if( stricmp(argv[i], "-server") == 0 ) {
            nServPort = atoi(argv[i+1]);
            i += 2;
        }
        else if( strstr(argv[i], ".iv") != NULL || strstr(argv[i], ".wrl") != NULL || strstr(argv[i], ".vrml") != NULL) {
            vIvFiles.push_back(argv[i]);
            i++;
        }
        else if( strstr(argv[i], ".xml") != NULL || strstr(argv[i], ".dae") != NULL ) {
            vXMLFiles.push_back(argv[i]);
            i++;
        }
        else if( stricmp(argv[i], "-testscene") == 0 ) {
            s_bTestScene = true;
            i++;
        }
        else if( stricmp(argv[i], "--generateik") == 0 ) {

            // create environment and start a command-line controlled simulation 
            penv = CreateEnvironment(false);
            if( penv == NULL )
                return -1;
            penv->SetDebugLevel(debuglevel);
            // parse until a regular option arrives
            penv->StopSimulation(); // don't want simulation eating up time
            vector<string> vfreeparamnames;
            string robotfile, ikfast_executable = IKFAST_EXECUTABLE, outputfile = "ik.cpp";
            int manipindex = 0;
            bool bRotation3DOnly = false;

            while(++i < argc) {
                if( argv[i][0] == 0 || argv[i][0] == '-')
                    break;
                
                if( stricmp(argv[i],"robot") == 0 ) {
                    robotfile = argv[i+1];
                    i++;
                }
                else if( stricmp(argv[i],"ikfast") == 0 ) {
                    ikfast_executable = argv[i+1];
                    i++;
                }
                else if( stricmp(argv[i],"freeparam") == 0 ) {
                    vfreeparamnames.push_back(argv[i+1]);
                    i++;
                }
                else if( stricmp(argv[i],"output") == 0 ) {
                    outputfile = argv[i+1];
                    i++;
                }
                else if( stricmp(argv[i],"manipulator") == 0 ) {
                    manipindex = atoi(argv[i+1]);
                    i++;
                }
                else if( stricmp(argv[i],"rotation3donly") == 0 )
                    bRotation3DOnly = true;
            }
            
            if( ikfast_executable.size() == 0 ) {
                RAVELOG_ERRORA("Generate IK: ikfast exectuable not specified\n");
                return -2;
            }

            penv->SetDebugLevel(debuglevel);
            if( !penv->Load(robotfile.c_str()) ) {
                RAVELOG_ERRORA("Generate IK: failed to open %s robot file.\n", robotfile.c_str());
                return -4;
            }

            // get the first robot
            if( penv->GetRobots().size() == 0 )
                return -3;

            RobotBase* probot = penv->GetRobots().front();
            if( manipindex >= (int)probot->GetManipulators().size() ) {
                RAVELOG_ERRORA("Generate IK: manipulator index %d not valid.\n", manipindex);
                return -5;
            }

            const RobotBase::Manipulator& manip = probot->GetManipulators()[manipindex];
            if( manip.pBase == NULL || manip.pEndEffector == NULL || (!bRotation3DOnly && manip._vecarmjoints.size() < 6) ) {
                RAVELOG_ERRORA("Generate IK: Manipulator not valid, needs to be >= 6 joints (%"PRIdS").\n", manip._vecarmjoints.size());
                return -6;
            }

#ifndef _WIN32
            string strhome = string(getenv("HOME"))+string("/.openrave");
            char* tempfilename = tempnam(strhome.c_str(), "ik");
#else
            char* tempfilename = tempnam(NULL, "ik");
#endif
            std::ofstream f(tempfilename);
            if( !f ) {
                RAVELOG_ERRORA("failed to create temp file %s\n", tempfilename);
                return -9;
            }

            probot->WriteForwardKinematics(f);
            f.close();

            stringstream sscmd;
            sscmd << ikfast_executable << " --fkfile=" << tempfilename
                  << " --baselink=" << manip.pBase->GetIndex()
                  << " --eelink=" << manip.pEndEffector->GetIndex();
            if( bRotation3DOnly )
                sscmd << " --rotation3donly ";
            vector<int> vfreeparams, vsolvejoints;
            for(vector<int>::const_iterator it = manip._vecarmjoints.begin(); it != manip._vecarmjoints.end(); ++it) {
                string jointname = _stdwcstombs(probot->GetJoints()[*it]->GetName());
                if( find(vfreeparamnames.begin(), vfreeparamnames.end(), jointname) == vfreeparamnames.end() )
                    vsolvejoints.push_back(*it);
                else
                    vfreeparams.push_back(*it);
            }

            if( !bRotation3DOnly && vsolvejoints.size() != 6 ) {
                RAVELOG_ERRORA("need 6 solve joints, currently have %"PRIdS"\n", vsolvejoints.size());
                remove(tempfilename);
                return -7;
            }

            sscmd << " --savefile=" << outputfile << " ";
            for(vector<int>::iterator it = vfreeparams.begin(); it != vfreeparams.end(); ++it)
                sscmd << " --freeparam=" << *it << " ";
            sscmd << " --usedummyjoints ";
            for(vector<int>::iterator it = vsolvejoints.begin(); it != vsolvejoints.end(); ++it)
                sscmd << *it << " ";

            RAVELOG_INFOA("executing: %s\n", sscmd.str().c_str());
            RAVELOG_FATALA("generating ik could take up to a minute...\n");
            int status = system(sscmd.str().c_str());
#ifdef _WIN32
            int exitcode = status;
#else
            int exitcode = WEXITSTATUS(status);
#endif
            remove(tempfilename);

            if( exitcode == 0 ) {
                RAVELOG_DEBUGA("ikfast success, generated %s\n", outputfile.c_str());
                return 0;
            }
            else {
                RAVELOG_ERRORA("ikfast failed for robot %s\n", robotfile.c_str());
                return -8;
            }
        }
        else {
            RAVEPRINT(L"Error in input parameters at %s\ntype --help to see a list of command line options\n", argv[i]);
            return 0;
        }
    }

    // create environment and start a command-line controlled simulation 
    penv = CreateEnvironment(true);
    if( penv == NULL )
        return -1;
    penv->SetDebugLevel(debuglevel);
    for(list<string>::iterator it = listLoadPlugins.begin(); it != listLoadPlugins.end(); ++it)
        penv->LoadPlugin(it->c_str());

#ifndef _WIN32
    // add a signal handler
    signal(SIGINT,sigint_handler); // control C
#endif

    if( bListPlugins ) {

        std::list< std::pair<std::string, PLUGININFO> > plugins;
        std::list< std::pair<std::string, PLUGININFO> >::iterator itplugin;
        penv->GetPluginInfo(plugins);

        // output all the plugins and exit
        vector<wstring>::const_iterator itnames;     
        vector<string> names;
        vector<string>::iterator itname;
        wstringstream ss;
            
        ss << endl << L"Number of plugins: " << plugins.size() << endl;

        char  buf[256];
        ss << L"Collision Checkers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.collisioncheckers);

        ss << L"Controllers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.controllers);
            
        ss << L"Inverse Kinematics Solvers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.iksolvers);
            
        ss << L"Physics Engines:" << endl;
        OUTPUT_PLUGINS(itplugin->second.physicsengines);
            
        ss << L"Planners:" << endl;
        OUTPUT_PLUGINS(itplugin->second.planners);
            
        ss << L"Problems:" << endl;
        OUTPUT_PLUGINS(itplugin->second.problems);
            
        ss << L"Robots:" << endl;
        OUTPUT_PLUGINS(itplugin->second.robots);
            
        ss << L"Sensors:" << endl;
        OUTPUT_PLUGINS(itplugin->second.sensors);
            
        ss << L"Sensor Systems:" << endl;
        OUTPUT_PLUGINS(itplugin->second.sensorsystems);
            
        ss << L"Trajectories:" << endl;
        OUTPUT_PLUGINS(itplugin->second.trajectories);

        ss << L"Viewers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.viewers);

        ss << L"Servers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.servers);

        RAVEPRINT(ss.str().c_str());

        return 0;
    }

    if( nServPort > 0 ) {
#ifdef _WIN32
        WORD      wVersionRequested;
        WSADATA   wsaData;

        wVersionRequested = MAKEWORD(1,1);
        if (WSAStartup(wVersionRequested, &wsaData) != 0) {
            RAVEPRINT(L"Failed to start win socket\n");
            return -1;
        }
#endif

        s_server.reset(CreateSimpleTextServer(penv));
        if( !!s_server && s_server->Init(nServPort) ) {
            penv->AttachServer(s_server.get());
        }
    }

    bThreadDestroyed = false;
    if( pthread_create(&s_mainThread, NULL, MainOpenRAVEThread, NULL) ) {
        RAVEPRINT(L"failed to create main openrave thread\n");
    }
 
    while(!bThreadDestroyed) {
#ifdef _WIN32
        Sleep(100);
#else
        usleep(100000);
        //pthread_yield();
#endif
    }

    if( penv != NULL ) {
        penv->AttachViewer(NULL);
        s_viewer.reset();
        penv->AttachServer(NULL);
        s_server.reset();

        RAVELOG_DEBUGA("deleting the environment\n");
        delete penv; penv = NULL;
    }

    return 0;
}

// use to control openrave
void* MainOpenRAVEThread(void*p)
{
    if( bDisplayGUI ) {
        // find a viewer
        const char* viewer_prefs[] = {"qtcoin"};
        for(int i = 0; i < (int)ARRAYSIZE(viewer_prefs); ++i) {
            s_viewer.reset(penv->CreateViewer(viewer_prefs[i]));
            if( !!s_viewer )
                break;
        }

        if( !s_viewer ) { // take any collision checker
            PLUGININFO info;
            penv->GetLoadedInterfaces(info);
            std::vector<std::wstring>::const_iterator itname;
            FORIT(itname, info.viewers) {
                s_viewer.reset(penv->CreateViewer(itname->c_str()));
                if( !!s_viewer )
                    break;
            }
        }
        
        if( !s_viewer )
            RAVEPRINT(L"failed to find an OpenRAVE viewer.\n");
        else {
            RAVELOG(L"using %s viewer\n", s_viewer->GetXMLId());

            s_viewer->ViewerSetSize(s_WindowWidth, s_WindowHeight);
            if( s_bSetWindowPosition )
                s_viewer->ViewerMove(s_WindowPosX,s_WindowPosY);
            penv->AttachViewer(s_viewer.get());

            for(size_t i = 0; i < vIvFiles.size(); ++i) {
                if( !s_viewer->LoadModel(vIvFiles[i]) )
                    RAVEPRINT(L"failed to open %s\n", vIvFiles[i]);
            }
        }
    }

    penv->LockPhysics(true);

    if( sceneFile != NULL )
        penv->Load(sceneFile);
        
    vector<char*>::iterator it;
    FORIT(it, vXMLFiles)
        penv->Load(*it);

    list< pair<char*, char*> >::iterator itprob;
    FORIT(itprob, s_listProblems) {
        ProblemInstance* prob = penv->CreateProblem(itprob->first);
        if( prob != NULL )
            penv->LoadProblem(prob, itprob->second);
    }

    if( s_bTestScene ) {
        int xmlerror = GetXMLErrorCount();
        RAVEPRINT(L"xml error count: %d\n", xmlerror);
        exit(xmlerror);
    }

    if( s_saveScene != NULL ) {
        bool bSaveScene = penv->Save(s_saveScene);
//        if( !bSaveScene )
//            RAVELOG_ERRORA("save scene at file %s failed\n", s_saveScene);
//        else
//            RAVELOG_INFOA("save scene at file %s succeeded\n", s_saveScene);

        penv->LockPhysics(false);
        bThreadDestroyed = true;
        return NULL;
    }

    penv->LockPhysics(false);
    
    penv->GetViewer()->main(bShowGUI);

    if( !bThreadDestroyed ) {
        penv->AttachViewer(NULL);
        s_viewer.reset();
        penv->AttachServer(NULL);
        s_server.reset();
        
//        if( penv != NULL ) {        
//            delete penv; penv = NULL;
//        }

        bThreadDestroyed = true;
    }
    
    return NULL;
}

void sigint_handler(int)
{
    if( !bThreadDestroyed ) {
#ifndef _WIN32
        pthread_kill(s_mainThread, SIGINT);
#else
        pthread_kill(s_mainThread, 0);
#endif
        bThreadDestroyed = true;
    }
}
