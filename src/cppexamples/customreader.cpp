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

/// A simple openrave plugin that registers a custom XML reader. This allows users
/// to extend the XML files to add their own tags. 
/// Once the plugin is compiled, cd to where the plugin was installed and open customreader.env.xml with openrave
#include <rave/rave.h>

using namespace std;
using namespace OpenRAVE;

class CustomController : public ControllerBase
{
public:
    class XMLData : public XMLReadable
    {
    public:
        XMLData() : XMLReadable("piddata") {}
        vector<dReal> pgains,igains;
    };

    class PIDXMLReader : public BaseXMLReader
    {
    public:
        PIDXMLReader(boost::shared_ptr<XMLData> piddata, const std::list<std::pair<std::string,std::string> >& atts) {
            _piddata = piddata;
            if( !_piddata )
                _piddata.reset(new XMLData());
            RAVELOG_INFOA("the attributes piddata is created with are:\n");
            for(std::list<std::pair<std::string,std::string> >::const_iterator itatt = atts.begin(); itatt != atts.end(); ++itatt)
                RAVELOG_INFOA("%s=%s\n",itatt->first.c_str(),itatt->second.c_str());
        }

        virtual XMLReadablePtr GetReadable() { return _piddata; }

        virtual void startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts) {
            BaseXMLReader::startElement(name,atts);
        }

        virtual bool endElement(const std::string& name)
        {
            BaseXMLReader::endElement(name);

            if( name == "piddata" )
                return true;
            else if( name == "pgains" )
                // read all the float values into a vector
                _piddata->pgains = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
            else if( name == "igains" )
                // read all the float values into a vector
                _piddata->igains = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
            else
                RAVELOG_ERRORA("unknown field %s\n", name.c_str());

            return false;
        }

        virtual void characters(const std::string& ch)
        {
            BaseXMLReader::characters(ch);
            _ss.clear();
            _ss.str(ch);
        }

    protected:
        boost::shared_ptr<XMLData> _piddata;
        stringstream _ss;
    };

    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const std::list<std::pair<std::string,std::string> >& atts)
    {
        // ptr is the robot interface that this reader is being created for
        return BaseXMLReaderPtr(new PIDXMLReader(boost::shared_ptr<XMLData>(),atts));
    }

    CustomController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
    }

    virtual bool Init(RobotBasePtr robot, const std::string& args)
    {
        _probot = robot;

        // read the gains from the XML
        boost::shared_ptr<XMLData> piddata = boost::dynamic_pointer_cast<XMLData>(GetReadableInterface("piddata"));
        if( !!piddata ) {
            stringstream ss;
            ss << "piddata from custom XML reader is" << endl << "pgains: ";
            for(vector<dReal>::iterator it = piddata->pgains.begin(); it != piddata->pgains.end(); ++it)
                ss << *it << " ";
            ss << endl << "igains: ";
            for(vector<dReal>::iterator it = piddata->igains.begin(); it != piddata->igains.end(); ++it)
                ss << *it << " ";
            ss << endl;
            RAVELOG_INFOA(ss.str());
        }
        else
            RAVELOG_WARNA("failed to find piddata\n");
        return true;
    }

    virtual void Reset(int options) {}
    virtual bool SetDesired(const std::vector<dReal>& values) { return false; }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) { return false; }
    virtual bool SimulationStep(dReal fTimeElapsed) { return false; }
    virtual bool IsDone() { return false; }
    virtual dReal GetTime() const { return 0; }
    virtual RobotBasePtr GetRobot() const { return _probot; }

protected:
    RobotBasePtr _probot;
};

static boost::shared_ptr<void> s_RegisteredReader;

RAVE_PLUGIN_API InterfaceBasePtr CreateInterface(PluginType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv)
{
    if( strcmp(pluginhash,RaveGetInterfaceHash(type)) ) {
        RAVELOG_WARNA("plugin type hash is wrong\n");
        throw openrave_exception("bad plugin hash");
    }
    if( !penv )
        return InterfaceBasePtr();
 
    if( !s_RegisteredReader ) {
        /// as long as this pointer is valid, the reader will remain registered
        s_RegisteredReader = penv->RegisterXMLReader(PT_Controller,"piddata",CustomController::CreateXMLReader);
    }

    stringstream ss(name);
    string interfacename;
    ss >> interfacename;
    std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);

    switch(type) {
    case PT_Controller:
        if( interfacename == "customcontroller")
            return InterfaceBasePtr(new CustomController(penv));
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
    pinfo->interfacenames[PT_Controller].push_back("CustomController");
    return true;
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    s_RegisteredReader.reset(); // unregister the reader
}
