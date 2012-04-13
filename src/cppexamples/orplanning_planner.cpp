/** \example orplanning_planner.cpp
    \author Rosen Diankov

    Shows how to use a planner by directly creating the planner and setting the module parameters.
    The default values plan for the arm joints of a particular manipulator.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/format.hpp>

#include "orexample.h"

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

class PlanningPlannerExample : public OpenRAVEExample
{
public:

    PlannerAction PlanCallback(const PlannerBase::PlannerProgress& progress)
    {
        // plan callback
        return PA_None;
    }

    virtual void demothread(int argc, char ** argv) {
        string scenefilename = "data/hanoi_complex2.env.xml";
        RaveSetDebugLevel(Level_Debug);
        penv->Load(scenefilename);

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

        // create the planner parameters
        PlannerBasePtr planner = RaveCreatePlanner(penv,"birrt");

        // register an optional function to be called for every planner iteration
        UserDataPtr handle = planner->RegisterPlanCallback(boost::bind(&PlanningPlannerExample::PlanCallback,this,_1));

        while(IsOk()) {
            GraphHandlePtr pgraph;
            {
                EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

                probot->SetActiveDOFs(pmanip->GetArmIndices());

                PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
                params->_nMaxIterations = 4000; // max iterations before failure
                params->SetRobotActiveJoints(probot); // set planning configuration space to current active dofs
                params->vgoalconfig.resize(probot->GetActiveDOF());

                vector<dReal> vlower,vupper;
                probot->GetActiveDOFLimits(vlower,vupper);

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

            // unlock the environment and wait for the robot to finish
            while(!probot->GetController()->IsDone() && IsOk()) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
        }
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::PlanningPlannerExample example;
    return example.main(argc,argv);
}


