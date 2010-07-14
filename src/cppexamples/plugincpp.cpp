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

/// A simple openrave plugin that creates a problem instance and registers two functions
/// for that problem instance which can then be called through the network/scripts
#include <rave/rave.h>
#include <boost/bind.hpp>

using namespace std;
using namespace OpenRAVE;

class MyProblemInstance : public ProblemInstance
{
public:
    MyProblemInstance(EnvironmentBasePtr penv) : ProblemInstance(penv)
    {        
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
    }
};

RAVE_PLUGIN_API InterfaceBasePtr CreateInterface(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv)
{
    if( strcmp(pluginhash,RaveGetInterfaceHash(type)) ) {
        RAVELOG_WARNA("plugin type hash is wrong\n");
        throw openrave_exception("bad plugin hash");
    }
    if( !penv )
        return InterfaceBasePtr();
    
    stringstream ss(name);
    string interfacename;
    ss >> interfacename;
    std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);

    switch(type) {
    case PT_ProblemInstance:
        if( interfacename == "myproblem")
            return InterfaceBasePtr(new MyProblemInstance(penv));
        break;
    default:
        break;
    }

    return InterfaceBasePtr();
}

RAVE_PLUGIN_API bool GetPluginAttributes(PLUGININFO* pinfo, int size)
{
    if( pinfo == NULL ) return false;
    if( size != sizeof(PLUGININFO) ) {
        RAVELOG_ERRORA("bad plugin info sizes %d != %d\n", size, sizeof(PLUGININFO));
        return false;
    }

    // fill pinfo
    pinfo->interfacenames[PT_ProblemInstance].push_back("MyProblem");
    return true;
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
