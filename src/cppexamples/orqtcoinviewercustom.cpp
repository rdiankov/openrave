/** \example orqtcoinviewercustom.cpp
    \author Rosen Diankov

    Shows how to execute commands inside the viewer thread and cast viewer to a QMainWindow. Note that this
    relies on the qtcoin viewer being derived from qt4's QMainWindow class

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <QMainWindow>

using namespace OpenRAVE;
using namespace std;

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->Add(viewer);

    // finally call the viewer's infinite loop (this is why a separate thread is needed)
    bool showgui = true;
    viewer->main(showgui);
}

int g_counter=0;
void ViewerCallback(ViewerBasePtr pviewer)
{
    ++g_counter;
    // this is only true for the current qtcoinviewer implementation
    QMainWindow* wnd = dynamic_cast<QMainWindow*>(pviewer.get());
    if( (g_counter/60) & 1 ) {
        wnd->show();
    }
    else {
        wnd->hide();
    }
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


    UserDataPtr pregistration;
    while(!pregistration) {
        if( !pregistration && !!penv->GetViewer() ) {
            pregistration = penv->GetViewer()->RegisterViewerThreadCallback(boost::bind(ViewerCallback,penv->GetViewer()));
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }

    thviewer.join(); // wait for the viewer thread to exit
    pregistration.reset();
    penv->Destroy(); // destroy
    return 0;
}
