// Copyright (c) 2008-2010 Rosen Diankov (rosen.diankov@gmail.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;


void printhelp()
{
    RAVELOG_INFOA("orloadviewer [--num n] [--scene filename] viewername\n"
                  "  Shows how to load a robot into the openrave environment\n"
                  //"--num     Number of environments/viewers to create simultaneously\n"
                  "--scene          The filename of the scene to load\n"
                  );
}

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    RaveViewerBasePtr viewer = penv->CreateViewer(viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AttachViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);
    
}

int main(int argc, char ** argv)
{
    //int num = 1;
    string scenefilename = "data/lab1.env.xml";
    string viewername = "qtcoin";

    // parse the command line options
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-?") == 0 || strcmp(argv[i], "/?") == 0 || strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
            printhelp();
            return 0;
        }
//        else if( strcmp(argv[i], "--num") == 0 ) {
//            num = atoi(argv[i+1]);
//            i += 2;
//        }
        else if( strcmp(argv[i], "--scene") == 0 ) {
            scenefilename = argv[i+1];
            i += 2;
        }
        else
            break;
    }

    if( i < argc )
        viewername = argv[i++];
    
    // create the main environment
    EnvironmentBasePtr penv = CreateEnvironment(true);
    penv->SetDebugLevel(Level_Debug);

    boost::thread thviewer(boost::bind(SetViewer,penv,viewername));

    {
        // lock the environment to prevent changes
        EnvironmentMutex::scoped_lock lock(penv->GetMutex());

        // load the scene
        if( !penv->Load(scenefilename) ) {
            printhelp();
            return -2;
        }
    }

    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroy
    return 0;
}
