// A simple openrave plugin that registers a custom XML reader. This allows users
// to extend the XML files to add their own tags. 
// Once the plugin is compiled, cd to where the plugin was installed and open customreader.env.xml with openrave
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
        PIDXMLReader(XMLData* piddata, const char **atts) {
            _piddata = piddata;
            if( _piddata == NULL )
                _piddata = new XMLData();
        }
        virtual ~PIDXMLReader() { delete _piddata; }
        
        void* Release() { XMLData* temp = _piddata; _piddata = NULL; return temp; }

        virtual void startElement(void *ctx, const char *name, const char **atts) {}
        virtual bool endElement(void *ctx, const char *name)
        {
            if( strcmp((const char*)name, "piddata") == 0 )
                return true;

            if( strcmp((const char*)name, "pgains") == 0 ) {
                _piddata->pgains.clear();
                while(!ss.eof()) {
                    dReal f;
                    ss >> f;
                    if( !ss )
                        break;
                    _piddata->pgains.push_back(f);
                }
            }
            else if( strcmp((const char*)name, "igains") == 0 ) {
                _piddata->igains.clear();
                while(!ss.eof()) {
                    dReal f;
                    ss >> f;
                    if( !ss )
                        break;
                    _piddata->igains.push_back(f);
                }
            }
            else
                RAVELOG_ERRORA("unknown field %s\n", name);

            if( !ss )
                RAVELOG_ERRORA("PIDXMLReader error parsing %s\n", name);

            return false;
        }

        virtual void characters(void *ctx, const char *ch, int len)
        {
            if( len > 0 ) {
                ss.clear();
                ss.str(string(ch, len));
            }
            else
                ss.str(""); // reset
        }

    protected:
        XMLData* _piddata;
        stringstream ss;
    };

    CustomController(EnvironmentBase* penv) : ControllerBase(penv), _probot(NULL)
    {
        RegisterXMLReader(GetEnv());
    }

    static void RegisterXMLReader(EnvironmentBase* penv)
    {
        if( penv != NULL )
            penv->RegisterXMLReader(PT_Controller, "piddata", CustomController::CreateXMLReader);
    }

    static BaseXMLReader* CreateXMLReader(InterfaceBase* pinterface, const char **atts)
    {
        return new PIDXMLReader(NULL, atts);
    }

    virtual bool Init(RobotBase* robot, const char* args = NULL) {
        _probot = robot;

        // read the gains from the XML
        std::map<std::string, XMLReadable* >::const_iterator it = GetReadableInterfaces().find("piddata");
        if( it != GetReadableInterfaces().end() ) {
            stringstream ss;
            ss << "piddata from custom XML reader is" << endl << "pgains: ";
            XMLData* pdata = (XMLData*)it->second;
            for(vector<dReal>::iterator it = pdata->pgains.begin(); it != pdata->pgains.end(); ++it)
                ss << *it << " ";
            ss << endl << "igains: ";
            for(vector<dReal>::iterator it = pdata->igains.begin(); it != pdata->igains.end(); ++it)
                ss << *it << " ";
            ss << endl;
            RAVELOG_INFOA(ss.str().c_str());
        }
        else
            RAVELOG_WARNA("failed to find piddata\n");
        return true;
    }
    virtual void Reset(int options) {}
    virtual bool SetDesired(const dReal* pValues) { return false; }
    virtual bool SetPath(const Trajectory* ptraj) { return false; }
    virtual bool SetPath(const Trajectory* ptraj, int nTrajectoryId, float fDivergenceTime) { return false; }
    virtual bool SimulationStep(dReal fTimeElapsed) { return false; }
    virtual bool IsDone() { return false; }
    virtual float GetTime() const { return 0; }
    virtual RobotBase* GetRobot() const { return _probot; }

protected:
    RobotBase* _probot;
};

#ifdef _MSC_VER
#define PROT_STDCALL(name, paramlist) __stdcall name paramlist
#define DECL_STDCALL(name, paramlist) __stdcall name paramlist
#else
#ifdef __x86_64__
#define DECL_STDCALL(name, paramlist) name paramlist
#else
#define DECL_STDCALL(name, paramlist) __attribute__((stdcall)) name paramlist
#endif
#endif // _MSC_VER

extern "C" InterfaceBase* DECL_STDCALL(ORCreate, (PluginType type, wchar_t* name, EnvironmentBase* penv))
{
    if( name == NULL ) return NULL;
    
    switch(type) {
        case PT_Controller:
            if( wcscmp(name, L"CustomController") == 0 )
                return new CustomController(penv);
            break;
        default:
            break;
    }

    return NULL;
}

extern "C" bool DECL_STDCALL(GetPluginAttributes, (PLUGININFO* pinfo, int size))
{
    if( pinfo == NULL ) return false;
    if( size != sizeof(PLUGININFO) ) {
        printf("bad plugin info sizes %d != %d\n", size, sizeof(PLUGININFO));
        return false;
    }

    // fill pinfo
    pinfo->controllers.push_back(L"CustomController");
    return true;
}

extern "C"void DECL_STDCALL(DestroyPlugin, ())
{
    RAVELOG_INFOA("destroying customreader plugin");
}
