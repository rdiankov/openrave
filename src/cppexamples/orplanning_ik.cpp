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

#include "orexample.h"

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

class PlanningIkExample : public OpenRAVEExample
{
public:
    virtual void demothread(int argc, char ** argv) {
        string scenefilename = "data/pa10grasp2.env.xml";
        penv->Load(scenefilename);

        vector<RobotBasePtr> vrobots;
        penv->GetRobots(vrobots);
        RobotBasePtr probot = vrobots.at(0);

        // find a manipulator chain to move
        for(size_t i = 0; i < probot->GetManipulators().size(); ++i) {
            if( probot->GetManipulators()[i]->GetName().find("arm") != string::npos ) {
                probot->SetActiveManipulator(probot->GetManipulators()[i]);
                break;
            }
        }
        RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();

        // load inverse kinematics using ikfast
        ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
        penv->Add(pikfast,true,"");
        stringstream ssin,ssout;
        vector<dReal> vsolution;
        ssin << "LoadIKFastSolver " << probot->GetName() << " " << (int)IKP_Transform6D;
        if( !pikfast->SendCommand(ssout,ssin) ) {
            RAVELOG_ERROR("failed to load iksolver\n");
        }
        if( !pmanip->GetIkSolver()) {
            throw OPENRAVE_EXCEPTION_FORMAT0("need ik solver",ORE_Assert);
        }

        ModuleBasePtr pbasemanip = RaveCreateModule(penv,"basemanipulation"); // create the module
        penv->Add(pbasemanip,true,probot->GetName()); // load the module

        while(IsOk()) {
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
            while(!probot->GetController()->IsDone() && IsOk()) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
        }
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::PlanningIkExample example;
    return example.main(argc,argv);
}
