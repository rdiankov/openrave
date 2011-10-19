/** \example orplanning_planner.cpp
    \author Rosen Diankov

    Shows how to use a planner by directly creating the planner and setting the module parameters.
    The default values plan for the arm joints of a particular manipulator.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

using namespace OpenRAVE;
using namespace std;

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
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
    string scenefilename = "data/hanoi_complex2.env.xml";
    string viewername = "qtcoin";
    RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();
    RaveSetDebugLevel(Level_Debug);

    boost::thread thviewer(boost::bind(SetViewer,penv,viewername)); // create the viewer
    usleep(200000); // wait for the viewer to init
    penv->Load(scenefilename);
    usleep(100000); // wait for the viewer to init

    vector<RobotBasePtr> vrobots;
    penv->GetRobots(vrobots);
    RobotBasePtr probot = vrobots.at(0);
    // find the longest manipulator chain to move
    RobotBase::ManipulatorPtr pmanip = probot->GetManipulators().at(0);
    for(size_t i = 1; i < probot->GetManipulators().size(); ++i) {
        if( pmanip->GetArmIndices().size() < probot->GetManipulators()[i]->GetArmIndices().size() ) {
            pmanip = probot->GetManipulators()[i];
        }
    }
    RAVELOG_INFO(str(boost::format("planning with manipulator %s\n")%pmanip->GetName()));

    probot->SetActiveDOFs(pmanip->GetArmIndices());
    vector<dReal> vlower,vupper;
    probot->GetActiveDOFLimits(vlower,vupper);

    // create a planner
    PlannerBasePtr planner = RaveCreatePlanner(penv,"birrt");
    PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
    params->_nMaxIterations = 4000; // max iterations before failure
    params->SetRobotActiveJoints(probot); // set planning configuration space to current active dofs
    params->vgoalconfig.resize(probot->GetActiveDOF());

    while(1) {
        GraphHandlePtr pgraph;
        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

            // find a set of free joint values for the robot
            {
                RobotBase::RobotStateSaver saver(probot); // save the state
                while(1) {
                    for(size_t i = 0; i < vlower.size(); ++i) {
                        params->vgoalconfig[i] = vlower[i] + (vupper[i]-vlower[i])*RaveRandomFloat();
                    }
                    probot->SetActiveDOFValues(params->vgoalconfig);
                    if( !penv->CheckCollision(probot) && !probot->CheckSelfCollision() ) {
                        break;
                    }
                }
                // robot state is restored
            }

            RAVELOG_INFO("starting to plan\n");
            probot->GetActiveDOFValues(params->vinitialconfig);
            if( !planner->InitPlan(probot,params) ) {
                continue;
            }

            // create a new output trajectory
            TrajectoryBasePtr ptraj = RaveCreateTrajectory(penv,"");
            if( !planner->PlanPath(ptraj) ) {
                RAVELOG_WARN("plan failed, trying again\n");
                continue;
            }

            // draw the end effector of the trajectory
            {
                RobotBase::RobotStateSaver saver(probot); // save the state of the robot since will be setting joint values
                vector<RaveVector<float> > vpoints;
                vector<dReal> vtrajdata;
                for(dReal ftime = 0; ftime <= ptraj->GetDuration(); ftime += 0.01) {
                    ptraj->Sample(vtrajdata,ftime,probot->GetActiveConfigurationSpecification());
                    probot->SetActiveDOFValues(vtrajdata);
                    vpoints.push_back(pmanip->GetEndEffectorTransform().trans);
                }
                pgraph = penv->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),1.0f);
            }

            // send the trajectory to the robot
            probot->GetController()->SetPath(ptraj);
        }


        // wait for the robot to finish
        while(!probot->GetController()->IsDone()) {
            usleep(1000);
        }
    }

    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroy
    return 0;
}
