/** \example plugincpp.cpp
    \author Rosen Diankov

    Creates a simple OpenRAVE::ProblemInstance interface.
*/
#include <rave/rave.h>
#include <rave/plugin.h>
#include <boost/bind.hpp>

using namespace std;
using namespace OpenRAVE;

class MyProblemInstance : public ProblemInstance
{
public:
    MyProblemInstance(EnvironmentBasePtr penv) : ProblemInstance(penv)
    {
        __description = "A very simple plugin.";
        RegisterCommand("numbodies",boost::bind(&MyProblemInstance::NumBodies,this,_1,_2),"returns bodies");
        RegisterCommand("load",boost::bind(&MyProblemInstance::Load, this,_1,_2),"loads a given file");
    }

    void Destroy() {
        RAVELOG_INFOA("problem unloaded from environment\n");
    }

    int main(const string& cmd)
    {
        RAVELOG_INFOA("problem initialized cmd; %s\n", cmd.c_str());
        return 0;
    }

    bool NumBodies(ostream& sout, istream& sinput)
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        sout << vbodies.size();
        return true;
    }

    bool Load(ostream& sout, istream& sinput)
    {
        string filename;
        sinput >> filename;
        bool bSuccess = GetEnv()->Load(filename.c_str()); // load the file
        sout << bSuccess; // publish the results
        return true;
    } //
};

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_ProblemInstance && interfacename == "myproblem" ) {
        return InterfaceBasePtr(new MyProblemInstance(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_ProblemInstance].push_back("MyProblem");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}
