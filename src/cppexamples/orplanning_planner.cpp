/** \example orplanning_planner.cpp
    \author Rosen Diankov

    Shows how to use a planner by directly creating the planner and setting the problem parameters.
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
#define usleep(micro) Sleep(micro/1000)
#endif

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    penv->AttachViewer(viewer);
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

    // create the output trajectory
    boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(penv,probot->GetActiveDOF()));

    while(1) {
        EnvironmentBase::GraphHandlePtr pgraph;
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

            ptraj->Clear();
            if( !planner->PlanPath(ptraj) ) {
                RAVELOG_WARN("plan failed, trying again\n");
                continue;
            }

            // re-timing the trajectory with cubic interpolation
            ptraj->CalcTrajTiming(probot,TrajectoryBase::CUBIC,true,true);

            /// draw the end effector of the trajectory
            {
                RobotBase::RobotStateSaver saver(probot); // save the state of the robot since
                vector<RaveVector<float> > vpoints;
                for(dReal ftime = 0; ftime <= ptraj->GetTotalDuration(); ftime += 0.01) {
                    TrajectoryBase::TPOINT tp;
                    ptraj->SampleTrajectory(ftime,tp);
                    probot->SetActiveDOFValues(tp.q);
                    vpoints.push_back(pmanip->GetEndEffectorTransform().trans);
                }
                pgraph = penv->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),0.01f);
            }
        }

        // send the trajectory to the robot
        probot->SetActiveMotion(ptraj);
        
        // wait for the robot to finish
        while(!probot->GetController()->IsDone()) {
            usleep(1000);
        }
    }

    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroy
    return 0;
}
