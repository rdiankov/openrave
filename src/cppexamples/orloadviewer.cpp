/** \example orloadviewer.cpp
    \author Rosen Diankov

    Shows how to load a robot into the openrave environment and start a viewer.

    Usage:
    \verbatim
    orloadviewer [--num n] [--scene filename] viewername
    \endverbatim

    - \b --num - Number of environments/viewers to create simultaneously
    - \b --scene - The filename of the scene to load.

    Example:
    \verbatim
    ./orloadviewer --scene data/lab1.env.xml qtcoin
    \endverbatim

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AddViewer(viewer);

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
        if((strcmp(argv[i], "-h") == 0)||(strcmp(argv[i], "-?") == 0)||(strcmp(argv[i], "/?") == 0)||(strcmp(argv[i], "--help") == 0)||(strcmp(argv[i], "-help") == 0)) {
            RAVELOG_INFO("orloadviewer [--num n] [--scene filename] viewername\n");
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
    if( i < argc ) {
        viewername = argv[i++];
    }

    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    RaveSetDebugLevel(Level_Debug);

    boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
    penv->Load(scenefilename); // load the scene
    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroy
    return 0;
}
