/** \example orcollision.cpp
    \author Rosen Diankov

    Load a robot into the openrave environment, set it at [joint values] and check for self
    collisions. Returns number of contact points.

    Usage:
    \verbatim
    orcollision [--list] [--checker checker_name] [--joints #values [values]] body_model
    \endverbatim

    - \b --list - List all the loadable interfaces (ie, collision checkers).
    - \b --checker - name Load a different collision checker instead of the default one.
    - <b>--joints \#values [values]</b> - Set the robot to specific joint values

    Example:
    \verbatim
    orcollision --checker ode robots/barrettwam.robot.xml
    \endverbatim

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>

using namespace OpenRAVE;
using namespace std;

void printhelp()
{
    RAVELOG_INFO("orcollision [--list] [--checker checker_name] [--joints #values [values]] body_model\n");
}

void printinterfaces(EnvironmentBasePtr penv)
{
    std::map<InterfaceType, std::vector<std::string> > interfacenames;
    RaveGetLoadedInterfaces(interfacenames);
    stringstream ss;

    ss << endl << "Loadable interfaces: " << endl;
    for(std::map<InterfaceType, std::vector<std::string> >::iterator itinterface = interfacenames.begin(); itinterface != interfacenames.end(); ++itinterface) {
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

    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment
    vector<dReal> vsetvalues;

    // parse the command line options
    int i = 1;
    while(i < argc) {
        if((strcmp(argv[i], "-h") == 0)||(strcmp(argv[i], "-?") == 0)||(strcmp(argv[i], "/?") == 0)||(strcmp(argv[i], "--help") == 0)||(strcmp(argv[i], "-help") == 0)) {
            printhelp();
            return 0;
        }
        else if( strcmp(argv[i], "--checker") == 0 ) {
            // create requested collision checker
            CollisionCheckerBasePtr pchecker = RaveCreateCollisionChecker(penv,argv[i+1]);
            if( !pchecker ) {
                RAVELOG_ERROR("failed to create checker %s\n", argv[i+1]);
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
        RAVELOG_ERROR("not enough parameters\n");
        printhelp();
        return 1;
    }

    // load the scene
    if( !penv->Load(argv[i]) ) {
        return 2;
    }

    int contactpoints = 0;
    {
        // lock the environment to prevent data from changing
        EnvironmentMutex::scoped_lock lock(penv->GetMutex());

        vector<KinBodyPtr> vbodies;
        penv->GetBodies(vbodies);
        // get the first body
        if( vbodies.size() == 0 ) {
            RAVELOG_ERROR("no bodies loaded\n");
            return -3;
        }

        KinBodyPtr pbody = vbodies.at(0);
        vector<dReal> values;
        pbody->GetDOFValues(values);

        // set new values
        for(int i = 0; i < (int)vsetvalues.size() && i < (int)values.size(); ++i) {
            values[i] = vsetvalues[i];
        }
        pbody->SetDOFValues(values,true);

        CollisionReportPtr report(new CollisionReport());
        penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
        if( pbody->CheckSelfCollision(report) ) {
            contactpoints = (int)report->contacts.size();
            stringstream ss;
            ss << "body in self-collision "
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
        else {
            RAVELOG_INFO("body not in collision\n");
        }

        // get the transformations of all the links
        vector<Transform> vlinktransforms;
        pbody->GetLinkTransformations(vlinktransforms);
    }

    RaveDestroy(); // destroy
    return contactpoints;
}
