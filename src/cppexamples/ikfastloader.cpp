/** \example ikfastloader.cpp
    \author Rosen Diankov

    Usage:
    \verbatim
    ikloader [robot filename] [iktype]
    \endverbatim

    Example:
    \verbatim
    ikfastloader robots/barrettwam.robot.xml Transform6D
    \endverbatim

    Show how to load an ikfast solver from C++ by specifying the robot and iktype.

    <b>Full Example Code:</b>
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
    RaveInitialize(true); // start openrave core

    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment

    {
        // lock the environment to prevent changes
        EnvironmentMutex::scoped_lock lock(penv->GetMutex());
        // load the scene
        RobotBasePtr probot = penv->ReadRobotXMLFile(robotname);
        if( !probot ) {
            penv->Destroy();
            return 2;
        }
        penv->Add(probot);

        ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
        penv->Add(pikfast,true,"");
        stringstream ssin,ssout;
        ssin << "LoadIKFastSolver " << probot->GetName() << " " << iktype;
        // if necessary, add free inc for degrees of freedom
        //ssin << " " << 0.04f;
        // get the active manipulator
        RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
        if( !pikfast->SendCommand(ssout,ssin) ) {
            RAVELOG_ERROR("failed to load iksolver\n");
            penv->Destroy();
            return 1;
        }

        RAVELOG_INFO("testing random ik\n");
        while(1) {
            Transform trans;
            trans.rot = quatFromAxisAngle(Vector(RaveRandomFloat()-0.5,RaveRandomFloat()-0.5,RaveRandomFloat()-0.5));
            trans.trans = Vector(RaveRandomFloat()-0.5,RaveRandomFloat()-0.5,RaveRandomFloat()-0.5)*2;

            vector<dReal> vsolution;
            if( pmanip->FindIKSolution(IkParameterization(trans),vsolution,IKFO_CheckEnvCollisions) ) {
                stringstream ss; ss << "solution is: ";
                for(size_t i = 0; i < vsolution.size(); ++i) {
                    ss << vsolution[i] << " ";
                }
                ss << endl;
                RAVELOG_INFO(ss.str());
            }
            else {
                // could fail due to collisions, etc
            }
        }
    }

    RaveDestroy(); // destroy
    return 0;
}
