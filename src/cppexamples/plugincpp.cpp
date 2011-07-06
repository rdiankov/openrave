/** \example plugincpp.cpp
    \author Rosen Diankov

    Creates a simple OpenRAVE::ModuleBase interface.

    <b>Full Example Code:</b>
 */
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>

using namespace std;
using namespace OpenRAVE;

class MyModule : public ModuleBase
{
public:
    MyModule(EnvironmentBasePtr penv) : ModuleBase(penv)
    {
        __description = "A very simple plugin.";
        RegisterCommand("numbodies",boost::bind(&MyModule::NumBodies,this,_1,_2),"returns bodies");
        RegisterCommand("load",boost::bind(&MyModule::Load, this,_1,_2),"loads a given file");
    }
    virtual ~MyModule() {
    }

    void Destroy() {
        RAVELOG_INFO("module unloaded from environment\n");
    }

    int main(const string& cmd)
    {
        RAVELOG_INFO("module initialized cmd; %s\n", cmd.c_str());
        return 0;
    }

    bool NumBodies(ostream& sout, istream& sinput)
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        sout << vbodies.size();     // publish the results
        return true;
    }

    bool Load(ostream& sout, istream& sinput)
    {
        string filename;
        sinput >> filename;
        bool bSuccess = GetEnv()->Load(filename.c_str());     // load the file
        return bSuccess;
    }
};

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if((type == PT_Module)&&(interfacename == "mymodule")) {
        return InterfaceBasePtr(new MyModule(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("MyModule");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}
