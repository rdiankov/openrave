/** \example customreader.cpp
    \author Rosen Diankov

    Creates a simple OpenRAVE::ControllerBase and shows how to add a custom XML reader to it.

    <b>Full Example Code:</b>
 */
#include <openrave/openrave.h>
#include <openrave/plugin.h>

using namespace std;
using namespace OpenRAVE;

namespace cppexamples {

class CustomController : public ControllerBase
{
public:
    class XMLData : public XMLReadable
    {
public:
        XMLData() : XMLReadable("piddata") {
        }
        vector<dReal> pgains,igains;
    };

    class PIDXMLReader : public BaseXMLReader
    {
public:
        PIDXMLReader(std::shared_ptr<XMLData> piddata, const AttributesList& atts) {
            _piddata = piddata;
            if( !_piddata )
                _piddata.reset(new XMLData());
            RAVELOG_INFO("the attributes piddata is created with are:\n");
            for(AttributesList::const_iterator itatt = atts.begin(); itatt != atts.end(); ++itatt)
                RAVELOG_INFO("%s=%s\n",itatt->first.c_str(),itatt->second.c_str());
        }

        virtual XMLReadablePtr GetReadable() {
            return _piddata;
        }

        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) {
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
                RAVELOG_ERROR("unknown field %s\n", name.c_str());

            return false;
        }

        virtual void characters(const std::string& ch)
        {
            _ss.clear();
            _ss << ch;
        }

protected:
        std::shared_ptr<XMLData> _piddata;
        stringstream _ss;
    };

    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
        // ptr is the robot interface that this reader is being created for
        return BaseXMLReaderPtr(new PIDXMLReader(std::shared_ptr<XMLData>(),atts));
    }

    CustomController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
    }
    virtual ~CustomController() {
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        _dofindices = dofindices;
        _nControlTransformation = nControlTransformation;

        // read the gains from the XML
        std::shared_ptr<XMLData> piddata = std::dynamic_pointer_cast<XMLData>(GetReadableInterface("piddata"));
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
            RAVELOG_WARN("failed to find piddata\n");
        return true;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return _nControlTransformation;
    }

    virtual void Reset(int options) {
    }
    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans) {
        return false;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) {
        return false;
    }
    virtual void SimulationStep(dReal fTimeElapsed) {
    }
    virtual bool IsDone() {
        return false;
    }
    virtual dReal GetTime() const {
        return 0;
    }
    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }

protected:
    RobotBasePtr _probot;
    std::vector<int> _dofindices;
    int _nControlTransformation;
};

} // end namespace cppexamples

static std::shared_ptr<void> s_RegisteredReader;

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( !s_RegisteredReader ) {
        /// as long as this pointer is valid, the reader will remain registered
        s_RegisteredReader = RaveRegisterXMLReader(PT_Controller,"piddata",cppexamples::CustomController::CreateXMLReader);
    }
    switch(type) {
    case PT_Controller:
        if( interfacename == "customcontroller")
            return InterfaceBasePtr(new cppexamples::CustomController(penv));
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

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    s_RegisteredReader.reset(); // unregister the reader
}
