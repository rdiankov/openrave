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
#include <boost/thread/thread.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>

//#ifndef _WIN32
//#include <signal.h>
//#endif


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

//void sigint_handler(int);
void MainOpenRAVEThread();

static bool bDisplayGUI = true, bShowGUI = true;

static EnvironmentBasePtr penv;
static RaveViewerBasePtr s_viewer;
static ProblemInstancePtr s_server;
static boost::shared_ptr<boost::thread> s_mainThread;
static string s_sceneFile;
static string s_saveScene; // if not NULL, saves the scene and exits
static string s_viewerName;
static bool s_bTestScene = false;

static list< pair<string, string> > s_listProblems; // problems to initially create
static vector<string> vIvFiles; // iv files to open
static vector<string> vXMLFiles; // xml files to open
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
                sprintf(buf, "  %s - %s", itnames->c_str(), itplugin->first.c_str()); \
                names.push_back(buf); \
            } \
        } \
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
    
    DebugLevel debuglevel = Level_Info;    
    list<string> listLoadPlugins;
    string collisionchecker, physicsengine;
    bool bListPlugins = false;
    // parse command line arguments
    int i = 1;
    while(i < argc) {
        if( stricmp(argv[i], "-h") == 0 || stricmp(argv[i], "-?") == 0 || stricmp(argv[i], "/?") == 0 || stricmp(argv[i], "--help") == 0 || stricmp(argv[i], "-help") == 0 ) {
            RAVELOG_INFO("RAVE Simulator Usage\n"
                         "-nogui             Run without a GUI (does not initialize the graphics engine nor communicate with any window manager)\n"
                         "-hidegui           Run with a hidden GUI, this allows 3D rendering and images to be captured\n"
                         "-listplugins       List all plugins and the interfaces they provide\n"
                         "-loadplugin [path] load a plugin at the following path\n"
                         "-server [port]     start up the server on a specific port (default is 4765)\n"
                         "-collision [name]  Default collision checker to use\n"
                         "-viewer [name]     Default viewer to use\n"
                         "-physics [name]    Default physics engine to use\n"
                         "-d [debug-level]   start up OpenRAVE with the specified debug level (higher numbers print more).\n"
                         "                   Default level is 2 for release builds, and 4 for debug builds.\n"
                         "--generateik robot [robotfile] [freeparam [joint name]] manipulator [index] manipulatorname [name] output [filename] ikfast [ikfast_executable] [translation3donly] [rotation3donly]\n"
                         "-wdims [width] [height] start up the GUI window with these dimensions\n"
                         "-wpos x y set the position of the GUI window\n"
                         "-problem [problemname] [args] Start openrave with a problem instance. If args involves spaces, surround it with double quotes. args is optional.\n"
                         "-f [scene]         Load a openrave environment file\n"
                         "-testscene         If specified, openrave will exit after the scene is loaded. A non-zero values means something went wrong with the loading process.\n\n");
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
            s_sceneFile = argv[i+1];
            i += 2;
        }
        else if( stricmp(argv[i], "-save") == 0 ) {
            s_saveScene = argv[i+1];
            i += 2;
        }
        else if( stricmp(argv[i], "-collision") == 0 ) {
            collisionchecker = argv[i+1];
            i += 2;
        }
        else if( stricmp(argv[i], "-viewer") == 0 ) {
            s_viewerName = argv[i+1];
            i += 2;
        }
        else if( stricmp(argv[i], "-physics") == 0 ) {
            physicsengine = argv[i+1];
            i += 2;
        }
        else if( stricmp(argv[i], "-problem") == 0 ) {
            s_listProblems.push_back(pair<string, string>(argv[i+1], ""));
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
            if( !penv )
                return -1;
            penv->SetDebugLevel(debuglevel);
            // parse until a regular option arrives
            
            EnvironmentMutex::scoped_lock lock(penv->GetMutex());

            penv->StopSimulation(); // don't want simulation eating up time
            vector<string> vfreeparamnames;
            string robotfile, ikfast_executable = IKFAST_EXECUTABLE, outputfile = "ik.cpp";
            int manipindex = 0;
            string manipname;
            bool bRotation3DOnly = false;
            bool bTranslation3DOnly = false;

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
                else if( stricmp(argv[i],"manipulatorname") == 0 ) {
                    manipname = argv[i+1];
                    i++;
                }
                else if( stricmp(argv[i],"rotation3donly") == 0 )
                    bRotation3DOnly = true;
                else if( stricmp(argv[i],"translation3donly") == 0 )
                    bTranslation3DOnly = true;
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
            vector<RobotBasePtr> vrobots;
            penv->GetRobots(vrobots);
            if( vrobots.size() == 0 )
                return -3;

            RobotBasePtr probot = vrobots[0];

            if( manipname.size() > 0 ) {
                for(manipindex = 0; manipindex < (int)probot->GetManipulators().size(); ++manipindex) {
                    if( probot->GetManipulators()[manipindex]->GetName() == manipname )
                        break;
                }
            }

            if( manipindex >= (int)probot->GetManipulators().size() ) {
                RAVELOG_ERRORA("Generate IK: manipulator index %d not valid.\n", manipindex);
                return -5;
            }

            RobotBase::ManipulatorConstPtr manip = probot->GetManipulators()[manipindex];
            if( !manip->GetBase() || !manip->GetEndEffector() || (!bRotation3DOnly && !bTranslation3DOnly && manip->GetArmJoints().size() < 6) ) {
                RAVELOG_ERRORA("Generate IK: Manipulator not valid, needs to be >= 6 joints (%"PRIdS").\n", manip->GetArmJoints().size());
                return -6;
            }

#ifndef _WIN32
            char* tempfilename = tempnam(penv->GetHomeDirectory().c_str(), "ik");
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
                  << " --baselink=" << manip->GetBase()->GetIndex()
                  << " --eelink=" << manip->GetEndEffector()->GetIndex();
            if( bRotation3DOnly )
                sscmd << " --rotation3donly ";
            if( bTranslation3DOnly )
                sscmd << " --translation3donly ";
            vector<int> vfreeparams, vsolvejoints;
            for(vector<int>::const_iterator it = manip->GetArmJoints().begin(); it != manip->GetArmJoints().end(); ++it) {
                if( find(vfreeparamnames.begin(), vfreeparamnames.end(), probot->GetJoints()[*it]->GetName()) == vfreeparamnames.end() )
                    vsolvejoints.push_back(*it);
                else
                    vfreeparams.push_back(*it);
            }

            if( !bRotation3DOnly && !bTranslation3DOnly && vsolvejoints.size() != 6 ) {
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
            RAVELOG_FATALA("generating ik could take up to 10 minutes...\n");
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
            RAVELOG_INFOA("Error in input parameters at %s\ntype --help to see a list of command line options\n", argv[i]);
            return 0;
        }
    }

    // create environment and start a command-line controlled simulation 
    RaveSetDebugLevel(debuglevel);
    penv = CreateEnvironment(true);
    if( !penv )
        return -1;
    penv->SetDebugLevel(debuglevel);
    for(list<string>::iterator it = listLoadPlugins.begin(); it != listLoadPlugins.end(); ++it)
        penv->LoadPlugin(it->c_str());

//#ifndef _WIN32
//    // add a signal handler
//    signal(SIGINT,sigint_handler); // control C
//#endif

    if( bListPlugins ) {

        std::list< std::pair<std::string, PLUGININFO> > plugins;
        std::list< std::pair<std::string, PLUGININFO> >::iterator itplugin;
        penv->GetPluginInfo(plugins);

        // output all the plugins and exit
        vector<string>::const_iterator itnames;     
        vector<string> names;
        vector<string>::iterator itname;
        stringstream ss;
            
        ss << endl << "Number of plugins: " << plugins.size() << endl;

        char  buf[256];
        ss << "Collision Checkers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_CollisionChecker]);

        ss << "Controllers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_Controller]);
            
        ss << "Inverse Kinematics Solvers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_InverseKinematicsSolver]);
            
        ss << "Physics Engines:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_PhysicsEngine]);
            
        ss << "Planners:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_Planner]);
            
        ss << "Problems:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_ProblemInstance]);
            
        ss << "Robots:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_Robot]);
            
        ss << "Sensors:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_Sensor]);
            
        ss << "Sensor Systems:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_SensorSystem]);
            
        ss << "Trajectories:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_Trajectory]);

        ss << "Viewers:" << endl;
        OUTPUT_PLUGINS(itplugin->second.interfacenames[PT_Viewer]);

        RAVELOG_INFOA(ss.str().c_str());

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

        s_server = CreateSimpleTextServer(penv);
        if( !!s_server ) {
            stringstream ss;
            ss << nServPort;
            if( penv->LoadProblem(s_server,ss.str()) != 0 )
                RAVELOG_WARNA("failed to load server\n");
        }
    }

    if( collisionchecker.size() > 0 ) {
        CollisionCheckerBasePtr p = penv->CreateCollisionChecker(collisionchecker);
        if( !!p )
            penv->SetCollisionChecker(p);
    }
    if( physicsengine.size() > 0 ) {
        PhysicsEngineBasePtr p = penv->CreatePhysicsEngine(physicsengine);
        if( !!p )
            penv->SetPhysicsEngine(p);
    }

    bThreadDestroyed = false;
    s_mainThread.reset(new boost::thread(boost::bind(MainOpenRAVEThread)));
    s_mainThread->join();

    penv->AttachViewer(RaveViewerBasePtr());
    s_viewer.reset();
    s_server.reset();
    penv->Destroy();
    penv.reset();
    return 0;
}

// use to control openrave
void MainOpenRAVEThread()
{
    if( bDisplayGUI ) {
        // find a viewer
        if( s_viewerName.size() > 0 )
            s_viewer = penv->CreateViewer(s_viewerName);

        if( !s_viewer ) {
            boost::array<string,1> viewer_prefs = {{"qtcoin"}};
            for(size_t i = 0; i < viewer_prefs.size(); ++i) {
                s_viewer = penv->CreateViewer(viewer_prefs[i]);
                if( !!s_viewer )
                    break;
            }
        }

        if( !s_viewer ) { // take any viewer
            PLUGININFO info;
            penv->GetLoadedInterfaces(info);
            std::vector<std::string>::const_iterator itname;
            FORIT(itname, info.interfacenames[PT_Viewer]) {
                s_viewer = penv->CreateViewer(*itname);
                if( !!s_viewer )
                    break;
            }
        }
        
        if( !s_viewer )
            RAVELOG_WARNA("failed to find an OpenRAVE viewer.\n");
        else {
            RAVELOG_INFOA("using %s viewer\n", s_viewer->GetXMLId().c_str());

            s_viewer->ViewerSetSize(s_WindowWidth, s_WindowHeight);
            if( s_bSetWindowPosition )
                s_viewer->ViewerMove(s_WindowPosX,s_WindowPosY);
            penv->AttachViewer(s_viewer);

            for(size_t i = 0; i < vIvFiles.size(); ++i) {
                if( !s_viewer->LoadModel(vIvFiles[i]) )
                    RAVELOG_INFOA("failed to open %s\n", vIvFiles[i].c_str());
            }
        }
    }

    {
        EnvironmentMutex::scoped_lock lock(penv->GetMutex());

        if( s_sceneFile.size() > 0 )
            penv->Load(s_sceneFile);

        vector<string>::iterator it;
        FORIT(it, vXMLFiles)
            penv->Load(*it);

        list< pair<string, string> >::iterator itprob;
        FORIT(itprob, s_listProblems) {
            ProblemInstancePtr prob = penv->CreateProblem(itprob->first);
            if( !!prob )
                penv->LoadProblem(prob, itprob->second);
        }

        if( s_bTestScene ) {
            int xmlerror = GetXMLErrorCount();
            RAVELOG_ERRORA("xml error count: %d\n", xmlerror);
            exit(xmlerror);
        }

        if( s_saveScene.size() > 0 ) {
            penv->Save(s_saveScene);
    //        if( !bSaveScene )
    //            RAVELOG_ERRORA("save scene at file %s failed\n", s_saveScene);
    //        else
    //            RAVELOG_INFOA("save scene at file %s succeeded\n", s_saveScene);

            bThreadDestroyed = true;
            return;
        }
    }
    
    penv->GetViewer()->main(bShowGUI);

    if( !bThreadDestroyed ) {
        penv->AttachViewer(RaveViewerBasePtr());
        s_viewer.reset();
        penv->RemoveProblem(s_server);
        s_server.reset();
        
//        if( penv != NULL ) {        
//            delete penv; penv = NULL;
//        }

        bThreadDestroyed = true;
    }
}

//void sigint_handler(int)
//{
//    if( !bThreadDestroyed ) {
//#ifndef _WIN32
//        s_mainThread.kill(SIGINT);
//#else
//        s_mainThread.kill(SIGINT);
//#endif
//        bThreadDestroyed = true;
//    }
//}
