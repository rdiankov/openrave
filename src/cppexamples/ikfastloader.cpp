/** \example ikfastloader.cpp
    \author Rosen Diankov

    Usage:
    \verbatim
    ikloader [robot filename] [iktype]
    \endverbatim
    
    Example:
    \verbatim
    ikloader robots/barrettwam.robot.xml Transform6D
    \endverbatim
    
    Show how to load an ikfast solver from C++ by specifying the robot and iktype.
 */
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <stdio.h>

#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/format.hpp>

using namespace OpenRAVE;
using namespace std;

int main(int argc, char ** argv)
{
    if( argc < 3 ) {
        RAVELOG_INFO("ikloader robot iktype\n");
        return 1;
    }
    
    string robotname = argv[1];
    string iktype = argv[2];

    // create the main environment
    EnvironmentBasePtr penv = CreateEnvironment(true);

    {
        // lock the environment to prevent changes
        EnvironmentMutex::scoped_lock lock(penv->GetMutex());
        // load the scene
        RobotBasePtr probot = penv->ReadRobotXMLFile(robotname);
        if( !probot ) {
            penv->Destroy();
            return 2;
        }
        penv->AddRobot(probot);

        ProblemInstancePtr pikfast = penv->CreateProblem("ikfast");
        penv->LoadProblem(pikfast,"");
        stringstream ssin,ssout;
        ssin << "LoadIKFastSolver " << probot->GetName() << " " << (int)IkParameterization::Type_Transform6D;
        // if necessary, add free inc for degrees of freedom
        //ssin << " " << 0.04f;
        // set the active manipulator
        probot->SetActiveManipulator(probot->GetManipulators().at(0)->GetName());
        if( !pikfast->SendCommand(ssout,ssin) ) {
            RAVELOG_ERROR("failed to load iksolver\n");
            penv->Destroy();
            return 1;
        }

        RAVELOG_INFO("testing random ik\n");
        vector<dReal> vsolution;
        if( !probot->GetActiveManipulator()->FindIKSolution(IkParameterization(probot->GetActiveManipulator()->GetEndEffectorTransform()),vsolution,true) ) {
            RAVELOG_INFO("failed to get solution\n");
        }
        else {
            stringstream ss; ss << "solution is: ";
            for(size_t i = 0; i < vsolution.size(); ++i) {
                ss << vsolution[i] << " ";
            }
            ss << endl;
            RAVELOG_INFO(ss.str());
        }
    }
    
    penv->Destroy(); // destroy
    return 0;
}
