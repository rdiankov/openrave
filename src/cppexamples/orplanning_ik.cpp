/** \example orplanning_ik.cpp
    \author Rosen Diankov

    Shows how to use inverse kinematics and planners to move a robot's end-effector safely through the environment.
    The default manipulator is used for the robot.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define usleep(micro) Sleep(micro/1000)
#endif

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    penv->AddViewer(viewer);
    viewer->main(true);
}

int main(int argc, char ** argv)
{
    string scenefilename = "data/pa10grasp2.env.xml";
    string viewername = "qtcoin";
    RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();

    boost::thread thviewer(boost::bind(SetViewer,penv,viewername)); // create the viewer
    usleep(200000); // wait for the viewer to init
    penv->Load(scenefilename);
    usleep(100000); // wait for the viewer to init

    vector<RobotBasePtr> vrobots;
    penv->GetRobots(vrobots);
    RobotBasePtr probot = vrobots.at(0);

    // find a manipulator chain to move
    for(size_t i = 0; i < probot->GetManipulators().size(); ++i) {
        if( probot->GetManipulators()[i]->GetName().find("arm") != string::npos ) {
            probot->SetActiveManipulator(i);
            break;
        }
    }
    RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();

    // load inverse kinematics using ikfast
    ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
    penv->AddModule(pikfast,"");
    stringstream ssin,ssout;
    vector<dReal> vsolution;
    ssin << "LoadIKFastSolver " << probot->GetName() << " " << (int)IKP_Transform6D;
    if( !pikfast->SendCommand(ssout,ssin) ) {
        RAVELOG_ERROR("failed to load iksolver\n");
    }
    if( !pmanip->GetIkSolver()) {
        penv->Destroy();
        return 1;
    }

    ModuleBasePtr pbasemanip = RaveCreateModule(penv,"basemanipulation"); // create the module
    penv->AddModule(pbasemanip,probot->GetName()); // load the module

    while(1) {
        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

            // find a new manipulator position and feed that into the planner. If valid, robot will move to it safely.
            Transform t = pmanip->GetEndEffectorTransform();
            t.trans += Vector(RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f);
            t.rot = quatMultiply(t.rot,quatFromAxisAngle(Vector(RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f)*0.2f));
            ssin.str("");
            ssin.clear();
            ssin << "MoveToHandPosition pose " << t;
            // start the planner and run the robot
            RAVELOG_INFO("%s\n",ssin.str().c_str());
            if( !pbasemanip->SendCommand(ssout,ssin) ) {
                continue;
            }
        }

        // unlock the environment and wait for the robot to finish
        while(!probot->GetController()->IsDone()) {
            usleep(1000);
        }
    }

    RaveDestroy();
    thviewer.join(); // wait for the viewer thread to exit
    return 0;
}
