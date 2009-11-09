// Software License Agreement (BSD License)
// Copyright (c) 2008, Willow Garage, Inc.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// author: Rosen Diankov
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
    boost::shared_ptr<COLLISIONREPORT> report(new COLLISIONREPORT());
    penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
    if( probot->CheckSelfCollision(report) ) {
        contactpoints = (int)report->contacts.size();
        stringstream ss;
        ss << "robot in self-collision "
           << (!!report->plink1 ? report->plink1->GetName() : "") << ":"
           << (!!report->plink2 ? report->plink2->GetName() : "") << " at "
           << contactpoints << "contacts" << endl;
        for(int i = 0; i < contactpoints; ++i) {
            COLLISIONREPORT::CONTACT& c = report->contacts[i];
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
