/** \example orikfilter.cpp
    \author Rosen Diankov

    Shows how to use set a custom inverse kinematics filter to add extra constraints.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <openrave/utils.h>
#include <vector>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;

// quit after 100 milliseconds
IkReturn MyTimeoutFilter(std::vector<dReal>&, RobotBase::ManipulatorConstPtr, const IkParameterization&, uint32_t starttime)
{
    if( utils::GetMilliTime()-starttime > 100 ) {
        RAVELOG_INFO("quitting\n");
        return IKRA_Quit;
    }
    return IKRA_Success;
}

int main(int argc, char ** argv)
{
    string scenefilename = "data/pr2test1.env.xml";
    RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();
    penv->Load(scenefilename);

    vector<RobotBasePtr> vrobots;
    penv->GetRobots(vrobots);
    RobotBasePtr probot = vrobots.at(0);
    probot->SetActiveManipulator("leftarm_torso");
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
        penv->Destroy();
        return 1;
    }

    probot->SetActiveDOFs(pmanip->GetArmIndices());
    vector<dReal> vlower,vupper;

    while(1) {
        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

            // move robot randomly
            probot->GetActiveDOFLimits(vlower,vupper);
            vector<dReal> v(pmanip->GetArmIndices().size());
            for(size_t i = 0; i < vlower.size(); ++i) {
                v[i] = vlower[i] + (vupper[i]-vlower[i])*RaveRandomFloat();
            }
            probot->SetActiveDOFValues(v);
            bool bincollision = !penv->CheckCollision(probot) && !probot->CheckSelfCollision();

            uint32_t starttime = utils::GetMilliTime();
            UserDataPtr filterhandle = pmanip->GetIkSolver()->RegisterCustomFilter(0,boost::bind(MyTimeoutFilter,_1,_2,_3,starttime));
            bool bsuccess = pmanip->FindIKSolution(pmanip->GetIkParameterization(IKP_Transform6D),v,IKFO_CheckEnvCollisions);
            RAVELOG_INFO("in collision: %d, real success %d, time passed: %d\n",bincollision,bsuccess,utils::GetMilliTime()-starttime);
        }
    }

    RaveDestroy();
    return 0;
}
