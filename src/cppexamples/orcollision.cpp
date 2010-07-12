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

using namespace OpenRAVE;
using namespace std;


void printhelp()
{
    RAVELOG_INFOA("orcollision [--list] [--checker checker_name] [--joints #values [values]] robot_model\n"
                  "  Load a robot into the openrave environment, set it at [joint values] and\n"
                  "  check for self collisions. Returns number of contact points.\n"
                  "--list             List all the loadable interfaces (ie, collision checkers).\n"
                  "--checker name            Load a different collision checker instead of the default one\n"
                  "--joints #values [values] Set the robot to specific joint values\n");
}

void printinterfaces(EnvironmentBasePtr penv)
{
    PLUGININFO info;
    penv->GetLoadedInterfaces(info);

    stringstream ss;
            
    ss << endl << "Loadable interfaces: " << endl;
    for(std::map<PluginType, std::vector<std::string> >::iterator itinterface = info.interfacenames.begin(); itinterface != info.interfacenames.end(); ++itinterface) {
        ss << RaveGetInterfaceName(itinterface->first) << "(" << itinterface->second.size() << "):" << endl;
        for(vector<string>::iterator it = itinterface->second.begin(); it != itinterface->second.end(); ++it)
            ss << " " << *it << endl;
        ss << endl;
    }
    RAVELOG_INFO(ss.str());
}

int main(int argc, char ** argv)
{
    if( argc < 2 ) {
        printhelp();
        return -1; // no robots to load
    }

    // create the main environment
    EnvironmentBasePtr penv = CreateEnvironment(true);
    vector<dReal> vsetvalues; 

    // parse the command line options
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-?") == 0 || strcmp(argv[i], "/?") == 0 || strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 ) {
            printhelp();
            return 0;
        }
        else if( strcmp(argv[i], "--checker") == 0 ) {
            // create requested collision checker
            CollisionCheckerBasePtr pchecker = penv->CreateCollisionChecker(argv[i+1]);
            if( !pchecker ) {
                RAVELOG_ERRORA("failed to create checker %s\n", argv[i+1]);
                return -3;
            }
            penv->SetCollisionChecker(pchecker);
            i += 2;
        }
        else if( strcmp(argv[i], "--list") == 0 ) {
            printinterfaces(penv);
            return 0;
        }
        else if( strcmp(argv[i], "--joints") == 0 ) {
            vsetvalues.resize(atoi(argv[i+1]));
            for(int j = 0; j < (int)vsetvalues.size(); ++j)
                vsetvalues[j] = atoi(argv[i+j+2]);

            i += 2+vsetvalues.size();
        }
        else
            break;
    }
    
    if( i >= argc ) {
        RAVELOG_ERRORA("not enough parameters\n");
        printhelp();
        return -1;
    }

    // load the scene
    if( !penv->Load(argv[i]) ) {
        printhelp();
        return -2;
    }
    
    // lock the environment to prevent thigns from changes
    EnvironmentMutex::scoped_lock lock(penv->GetMutex());

    vector<RobotBasePtr> vrobots;
    penv->GetRobots(vrobots);
    // get the first robot
    if( vrobots.size() == 0 ) {
        RAVELOG_ERRORA("no robots loaded\n");
        return -3;
    }

    RobotBasePtr probot = vrobots.at(0);
    vector<dReal> values;
    probot->GetJointValues(values);
    
    // set new values
    for(int i = 0; i < (int)vsetvalues.size() && i < (int)values.size(); ++i)
        values[i] = vsetvalues[i];
    probot->SetJointValues(values,true);

    int contactpoints = 0;
    CollisionReportPtr report(new CollisionReport());
    penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
    if( probot->CheckSelfCollision(report) ) {
        contactpoints = (int)report->contacts.size();
        stringstream ss;
        ss << "robot in self-collision "
           << (!!report->plink1 ? report->plink1->GetName() : "") << ":"
           << (!!report->plink2 ? report->plink2->GetName() : "") << " at "
           << contactpoints << "contacts" << endl;
        for(int i = 0; i < contactpoints; ++i) {
            CollisionReport::CONTACT& c = report->contacts[i];
            ss << "contact" << i << ": pos=("
               << c.pos.x << ", " << c.pos.y << ", " << c.pos.z << "), norm=("
               << c.norm.x << ", " << c.norm.y << ", " << c.norm.z << ")" << endl;
        }
        
        RAVELOG_INFOA(ss.str());
    }
    else RAVELOG_INFOA("robot not in collision\n");

    // get the transformations of all the links
    vector<Transform> vlinktransforms;
    probot->GetBodyTransformations(vlinktransforms);

    penv->Destroy(); // destroy
    return contactpoints;
}
