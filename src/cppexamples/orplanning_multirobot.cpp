/** \example orplanning_multirobot.cpp
    \author Rosen Diankov

    Shows how to plan for two different robots simultaneously and control them simultaneously.

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

/// \brief builds up the configuration space of multiple robots
class PlanningPlannerExample : public OpenRAVEExample
{
public:
    virtual void demothread(int argc, char ** argv) {
        // create a scene with two robots
        string robotfilename = "robots/barrettwam.robot.xml";
        RaveSetDebugLevel(Level_Debug);
        RobotBasePtr probot1 = penv->ReadRobotURI(RobotBasePtr(), robotfilename);
        penv->Add(probot1,true);
        RobotBasePtr probot2 = penv->ReadRobotURI(RobotBasePtr(), robotfilename);
        penv->Add(probot2,true);
        Transform trobot2 = probot2->GetTransform(); trobot2.trans.y += 0.5;
        probot2->SetTransform(trobot2);

        std::vector<RobotBasePtr> vrobots(2);
        vrobots[0] = probot1;
        vrobots[1] = probot2;

        // create the configuration space using the manipulator indices
        ConfigurationSpecification spec = probot1->GetActiveManipulator()->GetArmConfigurationSpecification() + probot2->GetActiveManipulator()->GetArmConfigurationSpecification();
        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->SetConfigurationSpecification(penv,spec); // set the joint configuration
        params->_nMaxIterations = 4000; // max iterations before failure
        params->Validate();

        // create the planner parameters
        PlannerBasePtr planner = RaveCreatePlanner(penv,"birrt");

        while(IsOk()) {
            list<GraphHandlePtr> listgraphs;
            {
                EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

                params->_getstatefn(params->vinitialconfig);
                params->vgoalconfig.resize(params->GetDOF());

                // find a set of free joint values for the robot
                {
                    while(1) {
                        for(int i = 0; i < params->GetDOF(); ++i) {
                            params->vgoalconfig[i] = params->_vConfigLowerLimit[i] + (params->_vConfigUpperLimit[i]-params->_vConfigLowerLimit[i])*RaveRandomFloat();
                        }
                        params->_setstatefn(params->vgoalconfig);
                        if( params->_checkpathconstraintsfn(params->vgoalconfig,params->vgoalconfig,IT_OpenStart,PlannerBase::ConfigurationListPtr()) ) {
                            break;
                        }
                    }
                    // restore robot state
                    params->_setstatefn(params->vinitialconfig);
                }

                RAVELOG_INFO("starting to plan\n");
                if( !planner->InitPlan(RobotBasePtr(),params) ) {
                    continue;
                }

                // create a new output trajectory
                TrajectoryBasePtr ptraj = RaveCreateTrajectory(penv,"");
                if( !planner->PlanPath(ptraj) ) {
                    RAVELOG_WARN("plan failed, trying again\n");
                    continue;
                }

                // draw the end effector of the trajectory
                listgraphs.clear();
                for(std::vector<RobotBasePtr>::iterator itrobot = vrobots.begin(); itrobot != vrobots.end(); ++itrobot) {
                    RobotBase::RobotStateSaver saver(*itrobot); // save the state of the robot since will be setting joint values
                    RobotBase::ManipulatorPtr manip = (*itrobot)->GetActiveManipulator();
                    vector<RaveVector<float> > vpoints;
                    vector<dReal> vtrajdata;
                    for(dReal ftime = 0; ftime <= ptraj->GetDuration(); ftime += 0.01) {
                        ptraj->Sample(vtrajdata,ftime,manip->GetArmConfigurationSpecification());
                        (*itrobot)->SetDOFValues(vtrajdata,true,manip->GetArmIndices());
                        vpoints.push_back(manip->GetEndEffectorTransform().trans);
                    }
                    listgraphs.push_back(penv->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),1.0f));
                }

                // send the trajectory to each robot
                probot1->GetController()->SetPath(ptraj);
                probot2->GetController()->SetPath(ptraj);
            }

            // unlock the environment and wait for the robot to finish
            while(IsOk()) {
                if( probot1->GetController()->IsDone() || probot2->GetController()->IsDone() ) {
                    break;
                }
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

