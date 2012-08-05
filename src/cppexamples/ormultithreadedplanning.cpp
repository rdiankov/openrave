/** \example ormultithreadedplanning.cpp
    \author Rosen Diankov

    Shows how to execute different planners simultaneously on different threads using environment cloning.

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

class MultithreadedPlanningExample : public OpenRAVEExample
{
public:
    MultithreadedPlanningExample() : OpenRAVEExample("") {
    }

    void _PlanningThread(const std::string& robotname)
    {
        EnvironmentBasePtr pclondedenv = penv->CloneSelf(Clone_Bodies);
        RobotBasePtr probot = pclondedenv->GetRobot(robotname);
        RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
        if( !pmanip->GetIkSolver()) {
            throw OPENRAVE_EXCEPTION_FORMAT0("need ik solver",ORE_Assert);
        }

        ModuleBasePtr pbasemanip = RaveCreateModule(pclondedenv,"basemanipulation"); // create the module
        pclondedenv->Add(pbasemanip,true,probot->GetName()); // load the module

        TrajectoryBasePtr ptraj = RaveCreateTrajectory(pclondedenv,"");


        while(IsOk()) {
            EnvironmentMutex::scoped_lock lock(pclondedenv->GetMutex()); // lock environment

            // find a new manipulator position and feed that into the planner. If valid, robot will move to it safely.
            Transform t = pmanip->GetEndEffectorTransform();
            t.trans += Vector(RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f);
            t.rot = quatMultiply(t.rot,quatFromAxisAngle(Vector(RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f,RaveRandomFloat()-0.5f)*0.2f));

            stringstream ssin,ssout;
            ssin << "MoveToHandPosition execute 0 outputtraj pose " << t;
            // start the planner and run the robot
            if( !pbasemanip->SendCommand(ssout,ssin) ) {
                continue;
            }

            ptraj->deserialize(ssout);
            RAVELOG_INFO("trajectory duration %fs\n",ptraj->GetDuration());
        }

        RAVELOG_INFO("destroying cloned environment...\n");
        pclondedenv->Destroy();
    }

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

        int numthreads = 2;

        // start worker threads
        vector<boost::shared_ptr<boost::thread> > vthreads(numthreads);
        for(size_t i = 0; i < vthreads.size(); ++i) {
            vthreads[i].reset(new boost::thread(boost::bind(&MultithreadedPlanningExample::_PlanningThread,this,probot->GetName())));
        }

        while(IsOk()) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }

        RAVELOG_INFO("wait for threads to finish\n");
        for(size_t i = 0; i < vthreads.size(); ++i) {
            vthreads[i]->join();
        }
        RAVELOG_INFO("threads finished\n");
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::MultithreadedPlanningExample example;
    return example.main(argc,argv);
}
