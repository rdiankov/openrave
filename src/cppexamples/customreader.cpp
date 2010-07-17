/** \example customreader.cpp
    \author Rosen Diankov

    Creates a simple OpenRAVE::ControllerBase and shows how to add a custom XML reader to it.
*/
#include <rave/rave.h>
#include <rave/plugin.h>

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

        virtual ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts) {
            _ss.str("");
            return (name == "pgains" || name=="igains") ? PE_Support : PE_Pass;
        }

        virtual bool endElement(const std::string& name)
        {
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
            _ss.clear();
            _ss << ch;
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

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( !s_RegisteredReader ) {
        /// as long as this pointer is valid, the reader will remain registered
        s_RegisteredReader = penv->RegisterXMLReader(PT_Controller,"piddata",CustomController::CreateXMLReader);
    }
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

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Controller].push_back("CustomController");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    s_RegisteredReader.reset(); // unregister the reader
}
