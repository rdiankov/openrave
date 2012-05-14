/** \example plugincpp.cpp
   \author Rosen Diankov

   Every plugin contains a bunch of openrave interfaces, the plugincpp plugin creates a simple OpenRAVE::ModuleBase interface named \b mymodule.

   Inside programs, load the plugin using the RaveLoadPlugin, and then create the module the plugin offers using

   \verbatim
   m=RaveCreateModule(env,"mymodule");
   \endverbatim

   To test things through the command line, do:

   \verbatim
   openrave --loadplugin libplugincpp.so --module mymodule "my args"
   \endverbatim

   This will load liboplugincpp.so and startup module "mymodule". From plugincpp, notice that mymodule
   supports some "commands". These are in-process string-based calls invoked through
   interface->SendCommand function.

   If you are using octave or matlab, then can communicate with openrave through tcp/ip, check out: http://openrave.programmingvision.com/wiki/index.php/OctaveMATLAB

   Most openrave users use python to dynamically interact with openrave. For example:

   \verbatim
   openrave.py -i  --loadplugin libplugincpp.so data/lab1.env.xml
   \endverbatim

   drops into the python promp with the plugin loaded and a scene loaded. Then it is possible to execute the following python commands to create the interface and call a command:

   \verbatim
   m=RaveCreateModule(env,'mymodule')
   env.Add(m,true,'my args')
   m.SendCommand('numbodies')
   \endverbatim

   <b>Full Example Code:</b>
 */
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>

using namespace std;
using namespace OpenRAVE;

namespace cppexamples {

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

} // end namespace cppexamples

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "mymodule" ) {
        return InterfaceBasePtr(new cppexamples::MyModule(penv));
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
