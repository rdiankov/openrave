// Copyright (c) 2008-2010 Rosen Diankov (rosen.diankov@gmail.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
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

void printhelp()
{
    RAVELOG_INFOA("ikloader robot iktype\n"
                  "  Shows how to load an ikfast solver from C++ by specifying the robot and iktype\n"
                  "  For example: ikloader robots/barrettwam.robot.xml Transform6D\n"
                  );
}
int main(int argc, char ** argv)
{
    if( argc < 3 ) {
        printhelp();
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
            printhelp();
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
