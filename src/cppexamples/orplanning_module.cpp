/** \example orplanning_module.cpp
    \author Rosen Diankov

    Shows how to use a planner from a module to move the arm withut colliding into anything.
    The default values plan for all the joints of the robot.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>

#include "orexample.h"

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

class PlanningModuleExample : public OpenRAVEExample
{
public:
    virtual void demothread(int argc, char ** argv) {

        string scenefilename = "data/wamtest1.env.xml";
        penv->Load(scenefilename);

        vector<RobotBasePtr> vrobots;
        penv->GetRobots(vrobots);
        RobotBasePtr probot = vrobots.at(0);
        vector<dReal> vlower,vupper,v(probot->GetDOF());
        probot->GetDOFLimits(vlower,vupper);

        // set all dofs as active
        vector<int> vindices(probot->GetDOF());
        for(size_t i = 0; i < vindices.size(); ++i) {
            vindices[i] = i;
        }
        probot->SetActiveDOFs(vindices);

        ModuleBasePtr pbasemanip = RaveCreateModule(penv,"basemanipulation"); // create the module
        penv->Add(pbasemanip,true,probot->GetName()); // load the module

        while(IsOk()) {
            {
                EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

                // find a set of free joint values for the robot
                {
                    RobotBase::RobotStateSaver saver(probot); // save the state
                    while(1) {
                        for(size_t i = 0; i < vlower.size(); ++i) {
                            v[i] = vlower[i] + (vupper[i]-vlower[i])*RaveRandomFloat();
                        }
                        probot->SetActiveDOFValues(v);
                        if( !penv->CheckCollision(probot) && !probot->CheckSelfCollision() ) {
                            break;
                        }
                    }
                    // robot state is restored
                }

                stringstream cmdin,cmdout;
                cmdin << "MoveActiveJoints goal ";
                for(size_t i = 0; i < v.size(); ++i) {
                    cmdin << v[i] << " ";
                }

                // start the planner and run the robot
                RAVELOG_INFO("%s\n",cmdin.str().c_str());
                if( !pbasemanip->SendCommand(cmdout,cmdin) ) {
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
    cppexamples::PlanningModuleExample example;
    return example.main(argc,argv);
}
