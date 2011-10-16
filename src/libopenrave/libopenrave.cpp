// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "libopenrave.h"

#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/thread/once.hpp>

#include <streambuf>
#include "md5.h"

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include <locale>

#include "plugindatabase.h"

#include <boost/lexical_cast.hpp>

#define LIBXML_SAX1_ENABLED
#include <libxml/globals.h>
#include <libxml/xmlerror.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h> // only for xmlNewInputFromFile()
#include <libxml/tree.h>

#include <libxml/debugXML.h>
#include <libxml/xmlmemory.h>

namespace OpenRAVE {

#ifdef _WIN32
const char s_filesep = '\\';
#else
const char s_filesep = '/';
#endif

#if defined(USE_CRLIBM)

#ifdef LIBM_ACCURACY_RESULTS_H
#include LIBM_ACCURACY_RESULTS_H
#endif

// use round-nearest versions since it is the default
#ifdef LIBM_EXP_ACCURATE
dReal RaveExp(dReal f) {
    return exp(f);
}
#else
dReal RaveExp(dReal f) {
    return exp_rn(f);
}
#endif
#ifdef LIBM_LOG_ACCURATE
dReal RaveLog(dReal f) {
    return log(f);
}
#else
dReal RaveLog(dReal f) {
    return log_rn(f);
}
#endif
#ifdef LIBM_COS_ACCURATE
dReal RaveCos(dReal f) {
    return cos(f);
}
#else
dReal RaveCos(dReal f) {
    return cos_rn(f);
}
#endif
#ifdef LIBM_SIN_ACCURATE
dReal RaveSin(dReal f) {
    return sin(f);
}
#else
dReal RaveSin(dReal f) {
    return sin_rn(f);
}
#endif
#ifdef LIBM_TAN_ACCURATE
dReal RaveTan(dReal f) {
    return tan(f);
}
#else
dReal RaveTan(dReal f) {
    return tan_rn(f);
}
#endif
#ifdef LIBM_LOG2_ACCURATE
dReal RaveLog2(dReal f) {
    return log2(f);
}
#else
dReal RaveLog2(dReal f) {
    return log2_rn(f);
}
#endif
#ifdef LIBM_LOG10_ACCURATE
dReal RaveLog10(dReal f) {
    return log10(f);
}
#else
dReal RaveLog10(dReal f) {
    return log10_rn(f);
}
#endif
#ifdef LIBM_ACOS_ACCURATE
dReal RaveAcos(dReal f) {
    return acos(f);
}
#else
dReal RaveAcos(dReal f) {
    return acos_rn(f);
}
#endif
#ifdef LIBM_ASIN_ACCURATE
dReal RaveAsin(dReal f) {
    return asin(f);
}
#else
dReal RaveAsin(dReal f) {
    return asin_rn(f);
}
#endif
#ifdef LIBM_ATAN2_ACCURATE
dReal RaveAtan2(dReal y, dReal x) {
    return atan2(y,x);
}
#else
dReal RaveAtan2(dReal y, dReal x) // unfortunately no atan2 in crlibm...
{
    dReal absx, absy, val;
    if ((x == 0)&&(y == 0)) {
        return 0;
    }
    absy = y < 0 ? -y : y;
    absx = x < 0 ? -x : x;
    if (absy - absx == absy) {
        // x negligible compared to y
        return y < 0 ? -M_PI_2 : M_PI_2;
    }
    if (absx - absy == absx) {
        // y negligible compared to x
        val = 0.0;
    }
    else val = atan_rn(y/x);
    if (x > 0) {
        // first or fourth quadrant; already correct
        return val;
    }
    if (y < 0) {
        // third quadrant
        return val - M_PI;
    }
    return val + M_PI;
}
#endif
#ifdef LIBM_POW_ACCURATE
dReal RavePow(dReal x, dReal y) {
    return pow(x,y);
}
#else
dReal RavePow(dReal x, dReal y) {
    return pow_rn(x,y);
}
#endif
#ifdef LIBM_SQRT_ACCURATE
dReal RaveSqrt(dReal f) {
    return sqrt(f);
}
#else
dReal RaveSqrt(dReal f) {
    return sqrt(f);
}
//dReal RaveSqrt(dReal f) { return pow_rn(f,0.5); } // NOTE: this is really slow, is it really worth the precision?
#endif
dReal RaveFabs(dReal f) {
    return fabs(f);
}

#else // use all standard libm

#if OPENRAVE_PRECISION == 0 // floating-point
dReal RaveExp(dReal f) {
    return expf(f);
}
dReal RaveLog(dReal f) {
    return logf(f);
}
dReal RaveCos(dReal f) {
    return cosf(f);
}
dReal RaveSin(dReal f) {
    return sinf(f);
}
dReal RaveTan(dReal f) {
    return tanf(f);
}
#ifdef HAS_LOG2
dReal RaveLog2(dReal f) {
    return log2f(f);
}
#else
dReal RaveLog2(dReal f) {
    return logf(f)/logf(2.0f);
}
#endif
dReal RaveLog10(dReal f) {
    return log10f(f);
}
dReal RaveAcos(dReal f) {
    return acosf(f);
}
dReal RaveAsin(dReal f) {
    return asinf(f);
}
dReal RaveAtan2(dReal fy, dReal fx) {
    return atan2f(fy,fx);
}
dReal RavePow(dReal fx, dReal fy) {
    return powf(fx,fy);
}
dReal RaveSqrt(dReal f) {
    return sqrtf(f);
}
dReal RaveFabs(dReal f) {
    return fabsf(f);
}
#else
dReal RaveExp(dReal f) {
    return exp(f);
}
dReal RaveLog(dReal f) {
    return log(f);
}
dReal RaveCos(dReal f) {
    return cos(f);
}
dReal RaveSin(dReal f) {
    return sin(f);
}
dReal RaveTan(dReal f) {
    return tan(f);
}
#ifdef HAS_LOG2
dReal RaveLog2(dReal f) {
    return log2(f);
}
#else
dReal RaveLog2(dReal f) {
    return log(f)/log(2.0f);
}
#endif
dReal RaveLog10(dReal f) {
    return log10(f);
}
dReal RaveAcos(dReal f) {
    return acos(f);
}
dReal RaveAsin(dReal f) {
    return asin(f);
}
dReal RaveAtan2(dReal fy, dReal fx) {
    return atan2(fy,fx);
}
dReal RavePow(dReal fx, dReal fy) {
    return pow(fx,fy);
}
dReal RaveSqrt(dReal f) {
    return sqrt(f);
}
dReal RaveFabs(dReal f) {
    return fabs(f);
}
#endif

#endif

static boost::once_flag _onceRaveInitialize = BOOST_ONCE_INIT;

/// there is only once global openrave state. It is created when openrave
/// is first used, and destroyed when the program quits or RaveDestroy is called.
class RaveGlobal : private boost::noncopyable, public boost::enable_shared_from_this<RaveGlobal>, public UserData
{
    typedef std::map<std::string, CreateXMLReaderFn, CaseInsensitiveCompare> READERSMAP;

    RaveGlobal()
    {
        // is this really necessary? just makes bugs hard to reproduce...
        //srand(GetMilliTime());
        //RaveInitRandomGeneration(GetMilliTime());
        _nDebugLevel = Level_Info;
        _nGlobalEnvironmentId = 0;

        _mapinterfacenames[PT_Planner] = "planner";
        _mapinterfacenames[PT_Robot] = "robot";
        _mapinterfacenames[PT_SensorSystem] = "sensorsystem";
        _mapinterfacenames[PT_Controller] = "controller";
        _mapinterfacenames[PT_Module] = "module";
        _mapinterfacenames[PT_IkSolver] = "iksolver";
        _mapinterfacenames[PT_KinBody] = "kinbody";
        _mapinterfacenames[PT_PhysicsEngine] = "physicsengine";
        _mapinterfacenames[PT_Sensor] = "sensor";
        _mapinterfacenames[PT_CollisionChecker] = "collisionchecker";
        _mapinterfacenames[PT_Trajectory] = "trajectory";
        _mapinterfacenames[PT_Viewer] = "viewer";
        _mapinterfacenames[PT_SpaceSampler] = "spacesampler";
        BOOST_ASSERT(_mapinterfacenames.size()==PT_NumberOfInterfaces);

        _mapikparameterization[IKP_Transform6D] = "Transform6d";
        _mapikparameterization[IKP_Rotation3D] = "Rotation3D";
        _mapikparameterization[IKP_Translation3D] = "Translation3D";
        _mapikparameterization[IKP_Direction3D] = "Direction3D";
        _mapikparameterization[IKP_Ray4D] = "Ray4D";
        _mapikparameterization[IKP_Lookat3D] = "Lookat3D";
        _mapikparameterization[IKP_TranslationDirection5D] = "TranslationDirection5D";
        _mapikparameterization[IKP_TranslationXY2D] = "TranslationXY2D";
        _mapikparameterization[IKP_TranslationXYOrientation3D] = "TranslationXYOrientation3D";
        _mapikparameterization[IKP_TranslationLocalGlobal6D] = "TranslationLocalGlobal6D";
        BOOST_ASSERT(_mapikparameterization.size()==IKP_NumberOfParameterizations);
    }
public:
    virtual ~RaveGlobal() {
        Destroy();
    }

    static boost::shared_ptr<RaveGlobal>& instance()
    {
        boost::call_once(_create,_onceRaveInitialize);
        return _state;
    }

    int Initialize(bool bLoadAllPlugins, int level)
    {
        if( _IsInitialized() ) {
            return 0;     // already initialized
        }

#ifdef USE_CRLIBM
        _crlibm_fpu_state = crlibm_init();
#endif
        try {
            // set to the classic locale so that number serialization/hashing works correctly
            std::locale::global(std::locale::classic());
        }
        catch(const std::runtime_error& e) {
            RAVELOG_WARN("failed to set to C locale: %s\n",e.what());
        }

        _nDebugLevel = level;
        _pdatabase.reset(new RaveDatabase());
        if( !_pdatabase->Init(bLoadAllPlugins) ) {
            RAVELOG_FATAL("failed to create the openrave plugin database\n");
        }

        char* phomedir = getenv("OPENRAVE_HOME");
        if( phomedir == NULL ) {
#ifndef _WIN32
            _homedirectory = string(getenv("HOME"))+string("/.openrave");
#else
            _homedirectory = string(getenv("HOMEDRIVE"))+string(getenv("HOMEPATH"))+string("\\.openrave");
#endif
        }
        else {
            _homedirectory = phomedir;
        }
#ifndef _WIN32
        mkdir(_homedirectory.c_str(),S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH | S_IRWXU);
#else
        CreateDirectory(_homedirectory.c_str(),NULL);
#endif

        _vdbdirectories.clear();
        if( !RaveParseDirectories(getenv("OPENRAVE_DATABASE"), _vdbdirectories) ) {
            _vdbdirectories.push_back(_homedirectory);
        }

        return 0;
    }

    void Destroy()
    {
        // don't use any log statements since global instance might be null
        // environments have to be destroyed carefully since their destructors can be called, which will attempt to unregister the environment
        std::map<int, EnvironmentBase*> mapenvironments;
        {
            boost::mutex::scoped_lock lock(_mutexXML);
            mapenvironments = _mapenvironments;
        }
        FOREACH(itenv,mapenvironments) {
            // equire a shared pointer to prevent environment from getting deleted during Destroy loop
            EnvironmentBasePtr penv = itenv->second->shared_from_this();
            penv->Destroy();
        }
        mapenvironments.clear();
        _mapenvironments.clear();
        _pdefaultsampler.reset();
        _mapreaders.clear();
        _pdatabase.reset();
#ifdef USE_CRLIBM
        crlibm_exit(_crlibm_fpu_state);
#endif
    }

    std::string GetHomeDirectory()
    {
        return _homedirectory;
    }

    std::string FindDatabaseFile(const std::string& filename, bool bRead)
    {
        FOREACH(itdirectory,_vdbdirectories) {
#ifdef HAVE_BOOST_FILESYSTEM
            std::string fullfilename = boost::filesystem::system_complete(boost::filesystem::path(*itdirectory)/filename).string();
#else
            std::string fullfilename = *itdirectory;
            fullfilename += s_filesep;
            fullfilename += filename;
#endif
            if( bRead ) {
                if( !!ifstream(fullfilename.c_str()) ) {
                    return fullfilename;
                }
            }
            else {
                return fullfilename;
            }
        }
        return "";
    }

    void SetDebugLevel(int level)
    {
        _nDebugLevel = level;
    }

    int GetDebugLevel()
    {
        return _nDebugLevel;
    }

    class XMLReaderFunctionData : public UserData
    {
public:
        XMLReaderFunctionData(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn, boost::shared_ptr<RaveGlobal> global) : _global(global), _type(type), _xmltag(xmltag)
        {
            boost::mutex::scoped_lock lock(global->_mutexXML);
            _oldfn = global->_mapreaders[_type][_xmltag];
            global->_mapreaders[_type][_xmltag] = fn;
        }
        virtual ~XMLReaderFunctionData()
        {
            boost::shared_ptr<RaveGlobal> global = _global.lock();
            if( !!global ) {
                boost::mutex::scoped_lock lock(global->_mutexXML);
                global->_mapreaders[_type][_xmltag] = _oldfn;
            }
        }
protected:
        CreateXMLReaderFn _oldfn;
        boost::weak_ptr<RaveGlobal> _global;
        InterfaceType _type;
        std::string _xmltag;
    };

    UserDataPtr RegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn)
    {
        return UserDataPtr(new XMLReaderFunctionData(type,xmltag,fn,shared_from_this()));
    }

    const BaseXMLReaderPtr CallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts)
    {
        READERSMAP::iterator it = _mapreaders[type].find(xmltag);
        if( it == _mapreaders[type].end() ) {
            //throw openrave_exception(str(boost::format("No function registered for interface %s xml tag %s")%GetInterfaceName(type)%xmltag),ORE_InvalidArguments);
            return BaseXMLReaderPtr();
        }
        return it->second(pinterface,atts);
    }

    boost::shared_ptr<RaveDatabase> GetDatabase() const {
        return _pdatabase;
    }
    const std::map<InterfaceType,std::string>& GetInterfaceNamesMap() const {
        return _mapinterfacenames;
    }
    const std::map<IkParameterizationType,std::string>& GetIkParameterizationMap() {
        return _mapikparameterization;
    }

    const std::string& GetInterfaceName(InterfaceType type)
    {
        std::map<InterfaceType,std::string>::const_iterator it = _mapinterfacenames.find(type);
        if( it == _mapinterfacenames.end() ) {
            throw OPENRAVE_EXCEPTION_FORMAT("Invalid type %d specified", type, ORE_Failed);
        }
        return it->second;
    }

    // have to take in pointer instead of shared_ptr since method will be called in EnvironmentBase constructor
    int RegisterEnvironment(EnvironmentBase* penv)
    {
        BOOST_ASSERT(!!_pdatabase);
        boost::mutex::scoped_lock lock(_mutexXML);
        _mapenvironments[++_nGlobalEnvironmentId] = penv;
        return _nGlobalEnvironmentId;
    }

    void UnregisterEnvironment(EnvironmentBase* penv)
    {
        boost::mutex::scoped_lock lock(_mutexXML);
        FOREACH(it, _mapenvironments) {
            if( it->second == penv ) {
                _mapenvironments.erase(it);
                break;
            }
        }
    }

    int GetEnvironmentId(EnvironmentBasePtr penv)
    {
        boost::mutex::scoped_lock lock(_mutexXML);
        FOREACH(it,_mapenvironments) {
            if( it->second == penv.get() ) {
                return it->first;
            }
        }
        return 0;
    }

    EnvironmentBasePtr GetEnvironment(int id)
    {
        boost::mutex::scoped_lock lock(_mutexXML);
        std::map<int, EnvironmentBase*>::iterator it = _mapenvironments.find(id);
        if( it == _mapenvironments.end() ) {
            return EnvironmentBasePtr();
        }
        return it->second->shared_from_this();
    }

    void GetEnvironments(std::list<EnvironmentBasePtr>& listenvironments)
    {
        listenvironments.clear();
        boost::mutex::scoped_lock lock(_mutexXML);
        FOREACH(it,_mapenvironments) {
            EnvironmentBasePtr penv = it->second->shared_from_this();
            if( !!penv ) {
                listenvironments.push_back(penv);
            }
        }
    }

    SpaceSamplerBasePtr GetDefaultSampler()
    {
        if( !_pdefaultsampler ) {
            boost::mutex::scoped_lock lock(_mutexXML);
            BOOST_ASSERT( _mapenvironments.size() > 0 );
            _pdefaultsampler = GetDatabase()->CreateSpaceSampler(_mapenvironments.begin()->second->shared_from_this(),"MT19937");
        }
        return _pdefaultsampler;
    }

protected:
    static void _create()
    {
        if( !_state ) {
            _state.reset(new RaveGlobal());
        }
    }

    bool _IsInitialized() const {
        return !!_pdatabase;
    }

private:
    static boost::shared_ptr<RaveGlobal> _state;
    // state that is always present

    // state that is initialized/destroyed
    boost::shared_ptr<RaveDatabase> _pdatabase;
    int _nDebugLevel;
    boost::mutex _mutexXML;
    std::map<InterfaceType, READERSMAP > _mapreaders;
    std::map<InterfaceType,string> _mapinterfacenames;
    std::map<IkParameterizationType,string> _mapikparameterization;
    std::map<int, EnvironmentBase*> _mapenvironments;
    std::string _homedirectory;
    std::vector<std::string> _vdbdirectories;
    int _nGlobalEnvironmentId;
    SpaceSamplerBasePtr _pdefaultsampler;
#ifdef USE_CRLIBM
    long long _crlibm_fpu_state;
#endif

    friend void RaveInitializeFromState(UserDataPtr);
    friend UserDataPtr RaveGlobalState();
};

boost::shared_ptr<RaveGlobal> RaveGlobal::_state;

void RaveSetDebugLevel(int level)
{
    RaveGlobal::instance()->SetDebugLevel(level);
}

int RaveGetDebugLevel()
{
    return RaveGlobal::instance()->GetDebugLevel();
}

const std::map<InterfaceType,std::string>& RaveGetInterfaceNamesMap()
{
    return RaveGlobal::instance()->GetInterfaceNamesMap();
}

const std::map<IkParameterizationType,std::string>& RaveGetIkParameterizationMap()
{
    return RaveGlobal::instance()->GetIkParameterizationMap();
}

const std::string& RaveGetInterfaceName(InterfaceType type)
{
    return RaveGlobal::instance()->GetInterfaceName(type);
}

std::string RaveGetHomeDirectory()
{
    return RaveGlobal::instance()->GetHomeDirectory();
}

std::string RaveFindDatabaseFile(const std::string& filename, bool bRead)
{
    return RaveGlobal::instance()->FindDatabaseFile(filename,bRead);
}

int RaveInitialize(bool bLoadAllPlugins, int level)
{
    return RaveGlobal::instance()->Initialize(bLoadAllPlugins,level);
}

void RaveInitializeFromState(UserDataPtr globalstate)
{
    RaveGlobal::_state = boost::dynamic_pointer_cast<RaveGlobal>(globalstate);
}

UserDataPtr RaveGlobalState()
{
    // only return valid pointer if initialized!
    boost::shared_ptr<RaveGlobal> state = RaveGlobal::_state;
    if( !!state && state->_IsInitialized() ) {
        return state;
    }
    return UserDataPtr();
}

void RaveDestroy()
{
    RaveGlobal::instance()->Destroy();
}

int RaveGetEnvironmentId(EnvironmentBasePtr penv)
{
    return RaveGlobal::instance()->GetEnvironmentId(penv);
}

EnvironmentBasePtr RaveGetEnvironment(int id)
{
    return RaveGlobal::instance()->GetEnvironment(id);
}

void RaveGetEnvironments(std::list<EnvironmentBasePtr>& listenvironments)
{
    RaveGlobal::instance()->GetEnvironments(listenvironments);
}

void RaveGetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins)
{
    RaveGlobal::instance()->GetDatabase()->GetPluginInfo(plugins);
}

void RaveGetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames)
{
    RaveGlobal::instance()->GetDatabase()->GetLoadedInterfaces(interfacenames);
}

void RaveReloadPlugins()
{
    RaveGlobal::instance()->GetDatabase()->ReloadPlugins();
}

bool RaveLoadPlugin(const std::string& libraryname)
{
    return RaveGlobal::instance()->GetDatabase()->LoadPlugin(libraryname);
}

bool RaveHasInterface(InterfaceType type, const std::string& interfacename)
{
    return RaveGlobal::instance()->GetDatabase()->HasInterface(type,interfacename);
}

InterfaceBasePtr RaveCreateInterface(EnvironmentBasePtr penv, InterfaceType type,const std::string& interfacename)
{
    return RaveGlobal::instance()->GetDatabase()->Create(penv, type,interfacename);
}

RobotBasePtr RaveCreateRobot(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateRobot(penv,name);
}

PlannerBasePtr RaveCreatePlanner(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreatePlanner(penv, name);
}

SensorSystemBasePtr RaveCreateSensorSystem(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateSensorSystem(penv, name);
}

ControllerBasePtr RaveCreateController(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateController(penv, name);
}

ModuleBasePtr RaveCreateModule(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateModule(penv, name);
}

ModuleBasePtr RaveCreateProblem(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateModule(penv, name);
}

ModuleBasePtr RaveCreateProblemInstance(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateModule(penv, name);
}

IkSolverBasePtr RaveCreateIkSolver(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateIkSolver(penv, name);
}

PhysicsEngineBasePtr RaveCreatePhysicsEngine(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreatePhysicsEngine(penv, name);
}

SensorBasePtr RaveCreateSensor(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateSensor(penv, name);
}

CollisionCheckerBasePtr RaveCreateCollisionChecker(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateCollisionChecker(penv, name);
}

ViewerBasePtr RaveCreateViewer(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateViewer(penv, name);
}

KinBodyPtr RaveCreateKinBody(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateKinBody(penv, name);
}

TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateTrajectory(penv, name);
}

TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, int dof)
{
    return RaveCreateTrajectory(penv,"");
}

SpaceSamplerBasePtr RaveCreateSpaceSampler(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateSpaceSampler(penv, name);
}

UserDataPtr RaveRegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn)
{
    return RaveGlobal::instance()->GetDatabase()->RegisterInterface(type, name, interfacehash,envhash,createfn);
}

UserDataPtr RaveRegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn)
{
    return RaveGlobal::instance()->RegisterXMLReader(type,xmltag,fn);
}

BaseXMLReaderPtr RaveCallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts)
{
    return RaveGlobal::instance()->CallXMLReader(type,xmltag,pinterface,atts);
}

ConfigurationSpecification IkParameterization::GetConfigurationSpecification(IkParameterizationType iktype)
{
    ConfigurationSpecification spec;
    spec._vgroups.resize(1);
    spec._vgroups[0].offset = 0;
    spec._vgroups[0].dof = IkParameterization::GetNumberOfValues(iktype);
    spec._vgroups[0].name = str(boost::format("ikparam_values %d")%iktype);
    spec._vgroups[0].interpolation = "linear";
    return spec;
}

std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam)
{
    O << ikparam._type << " ";
    switch(ikparam._type) {
    case IKP_Transform6D:
        O << ikparam.GetTransform6D();
        break;
    case IKP_Rotation3D:
        O << ikparam.GetRotation3D();
        break;
    case IKP_Translation3D: {
        Vector v = ikparam.GetTranslation3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IKP_Direction3D: {
        Vector v = ikparam.GetDirection3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IKP_Ray4D: {
        O << ikparam.GetRay4D();
        break;
    }
    case IKP_Lookat3D: {
        Vector v = ikparam.GetLookat3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IKP_TranslationDirection5D:
        O << ikparam.GetTranslationDirection5D();
        break;
    case IKP_TranslationXY2D: {
        Vector v = ikparam.GetTranslationXY2D();
        O << v.x << " " << v.y << " ";
        break;
    }
    case IKP_TranslationXYOrientation3D: {
        Vector v = ikparam.GetTranslationXYOrientation3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IKP_TranslationLocalGlobal6D: {
        std::pair<Vector,Vector> p = ikparam.GetTranslationLocalGlobal6D();
        O << p.first.x << " " << p.first.y << " " << p.first.z << " " << p.second.x << " " << p.second.y << " " << p.second.z << " ";
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", ikparam.GetType(),ORE_InvalidArguments);
    }
    return O;
}

std::istream& operator>>(std::istream& I, IkParameterization& ikparam)
{
    int type=IKP_None;
    I >> type;
    ikparam._type = static_cast<IkParameterizationType>(type);
    switch(ikparam._type) {
    case IKP_Transform6D: { Transform t; I >> t; ikparam.SetTransform6D(t); break; }
    case IKP_Rotation3D: { Vector v; I >> v; ikparam.SetRotation3D(v); break; }
    case IKP_Translation3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetTranslation3D(v); break; }
    case IKP_Direction3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetDirection3D(v); break; }
    case IKP_Ray4D: { RAY r; I >> r; ikparam.SetRay4D(r); break; }
    case IKP_Lookat3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetLookat3D(v); break; }
    case IKP_TranslationDirection5D: { RAY r; I >> r; ikparam.SetTranslationDirection5D(r); break; }
    case IKP_TranslationXY2D: { Vector v; I >> v.y >> v.y; ikparam.SetTranslationXY2D(v); break; }
    case IKP_TranslationXYOrientation3D: { Vector v; I >> v.y >> v.y >> v.z; ikparam.SetTranslationXYOrientation3D(v); break; }
    case IKP_TranslationLocalGlobal6D: { Vector localtrans, trans; I >> localtrans.x >> localtrans.y >> localtrans.z >> trans.x >> trans.y >> trans.z; ikparam.SetTranslationLocalGlobal6D(localtrans,trans); break; }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", ikparam.GetType(),ORE_InvalidArguments);
    }
    return I;
}

int RaveGetIndexFromAffineDOF(int affinedofs, DOFAffine _dof)
{
    int dof = static_cast<int>(_dof);
    dof &= affinedofs;
    int index = 0;
    if( dof&DOF_X ) {
        return index;
    }
    else if( affinedofs & DOF_X ) {
        ++index;
    }
    if( dof&DOF_Y ) {
        return index;
    }
    else if( affinedofs & DOF_Y ) {
        ++index;
    }
    if( dof&DOF_Z ) {
        return index;
    }
    else if( affinedofs & DOF_Z ) {
        ++index;
    }
    if( dof&DOF_RotationAxis ) {
        return index;
    }
    if( dof&DOF_Rotation3D ) {
        return index;
    }
    if( dof&DOF_RotationQuat ) {
        return index;
    }
    throw OPENRAVE_EXCEPTION_FORMAT("unspecified dof 0x%x, 0x%x",affinedofs%dof,ORE_InvalidArguments);
}

DOFAffine RaveGetAffineDOFFromIndex(int affinedofs, int requestedindex)
{
    BOOST_ASSERT(requestedindex >= 0);
    int index = 0;
    if( index == requestedindex && (affinedofs&DOF_X) ) {
        return DOF_X;
    }
    else if( affinedofs & DOF_X ) {
        ++index;
    }
    if( index == requestedindex && (affinedofs&DOF_Y) ) {
        return DOF_Y;
    }
    else if( affinedofs & DOF_Y ) {
        ++index;
    }
    if( index == requestedindex  && (affinedofs&DOF_Z)) {
        return DOF_Z;
    }
    else if( affinedofs & DOF_Z ) {
        ++index;
    }
    if( index <= requestedindex && index+3 > requestedindex && (affinedofs&DOF_RotationAxis) ) {
        return DOF_RotationAxis;
    }
    if( index <= requestedindex && index+3 > requestedindex && (affinedofs&DOF_Rotation3D) ) {
        return DOF_Rotation3D;
    }
    if( index <= requestedindex && index+4 > requestedindex && (affinedofs&DOF_RotationQuat) ) {
        return DOF_RotationQuat;
    }
    throw OPENRAVE_EXCEPTION_FORMAT("requested index out of bounds %d (affinemask=0x%x)",requestedindex%affinedofs, ORE_InvalidArguments);
}

int RaveGetAffineDOF(int affinedofs)
{
    if( affinedofs & DOF_RotationAxis ) {
        BOOST_ASSERT( !(affinedofs & (DOF_Rotation3D|DOF_RotationQuat)) );
    }
    else if( affinedofs & DOF_Rotation3D ) {
        BOOST_ASSERT( !(affinedofs & (DOF_RotationAxis|DOF_RotationQuat)) );
    }
    else if( affinedofs & DOF_RotationQuat ) {
        BOOST_ASSERT( !(affinedofs & (DOF_Rotation3D|DOF_RotationAxis)) );
    }
    int dof = 0;
    if( affinedofs & DOF_X ) {
        dof++;
    }
    if( affinedofs & DOF_Y ) {
        dof++;
    }
    if( affinedofs & DOF_Z ) {
        dof++;
    }
    if( affinedofs & DOF_RotationAxis ) {
        dof++;
    }
    else if( affinedofs & DOF_Rotation3D ) {
        dof += 3;
    }
    else if( affinedofs & DOF_RotationQuat ) {
        dof += 4;
    }
    return dof;
}

void RaveGetAffineDOFValuesFromTransform(std::vector<dReal>::iterator itvalues, const Transform& t, int affinedofs, const Vector& vActvAffineRotationAxis)
{
    if( affinedofs & DOF_X ) {
        *itvalues++ = t.trans.x;
    }
    if( affinedofs & DOF_Y ) {
        *itvalues++ = t.trans.y;
    }
    if( affinedofs & DOF_Z ) {
        *itvalues++ = t.trans.z;
    }
    if( affinedofs & DOF_RotationAxis ) {
        // assume that rot.yzw ~= vActvAffineRotationAxis
        dReal fsin = RaveSqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);

        // figure out correct sign
        if( (t.rot.y > 0) != (vActvAffineRotationAxis.x>0) || (t.rot.z > 0) != (vActvAffineRotationAxis.y>0) || (t.rot.w > 0) != (vActvAffineRotationAxis.z>0) ) {
            fsin = -fsin;
        }
        *itvalues++ = 2 * RaveAtan2(fsin, t.rot.x);
    }
    else if( affinedofs & DOF_Rotation3D ) {
        dReal fsin = RaveSqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);
        dReal fangle = 2 * atan2(fsin, t.rot.x);
        if( fsin > 0 ) {
            dReal normalizer = fangle / fsin;
            *itvalues++ = normalizer * t.rot.y;
            *itvalues++ = normalizer * t.rot.z;
            *itvalues++ = normalizer * t.rot.w;
        }
        else {
            *itvalues++ = 0;
            *itvalues++ = 0;
            *itvalues++ = 0;
        }
    }
    else if( affinedofs & DOF_RotationQuat ) {
        *itvalues++ = t.rot.x;
        *itvalues++ = t.rot.y;
        *itvalues++ = t.rot.z;
        *itvalues++ = t.rot.w;
    }
}

void RaveGetTransformFromAffineDOFValues(Transform& t, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& vActvAffineRotationAxis)
{
    if( affinedofs & DOF_X ) {
        t.trans.x = *itvalues++;
    }
    if( affinedofs & DOF_Y ) {
        t.trans.y = *itvalues++;
    }
    if( affinedofs & DOF_Z ) {
        t.trans.z = *itvalues++;
    }
    if( affinedofs & DOF_RotationAxis ) {
        dReal angle = *itvalues++;
        dReal fsin = RaveSin(angle*(dReal)0.5);
        t.rot.x = RaveCos(angle*(dReal)0.5);
        t.rot.y = vActvAffineRotationAxis.x * fsin;
        t.rot.z = vActvAffineRotationAxis.y * fsin;
        t.rot.w = vActvAffineRotationAxis.z * fsin;
    }
    else if( affinedofs & DOF_Rotation3D ) {
        dReal x = *itvalues++;
        dReal y = *itvalues++;
        dReal z = *itvalues++;
        dReal fang = RaveSqrt(x*x + y*y + z*z);
        if( fang > 0 ) {
            dReal fnormalizer = sin((dReal)0.5 * fang) / fang;
            t.rot.x = cos((dReal)0.5 * fang);
            t.rot.y = fnormalizer * x;
            t.rot.z = fnormalizer * y;
            t.rot.w = fnormalizer * z;
        }
        else {
            t.rot = Vector(1,0,0,0); // identity
        }
    }
    else if( affinedofs & DOF_RotationQuat ) {
        // have to normalize since user might not be aware of this particular parameterization of rotations
        t.rot.x = *itvalues++;
        t.rot.y = *itvalues++;
        t.rot.z = *itvalues++;
        t.rot.w = *itvalues++;
        t.rot.normalize4();
    }
}

ConfigurationSpecification RaveGetAffineConfigurationSpecification(int affinedofs,KinBodyConstPtr pbody)
{
    ConfigurationSpecification spec;
    spec._vgroups.resize(1);
    spec._vgroups[0].offset = 0;
    spec._vgroups[0].dof = RaveGetAffineDOF(affinedofs);
    spec._vgroups[0].interpolation = "linear";
    if( !!pbody ) {
        spec._vgroups[0].name = str(boost::format("affine_transform %s %d")%pbody->GetName()%affinedofs);
    }
    else {
        spec._vgroups[0].name = str(boost::format("affine_transform __dummy__ %d")%affinedofs);
    }
    return spec;
}

int ConfigurationSpecification::GetDOF() const
{
    int maxdof = 0;
    FOREACHC(it,_vgroups) {
        maxdof = max(maxdof,it->offset+it->dof);
    }
    return maxdof;
}

bool ConfigurationSpecification::IsValid() const
{
    vector<uint8_t> occupied(GetDOF(),0);
    FOREACHC(it,_vgroups) {
        if(it->offset < 0 || it->dof <= 0 || it->offset+it->dof > (int)occupied.size()) {
            return false;
        }
        for(int i = it->offset; i < it->offset+it->dof; ++i) {
            if( occupied[i] ) {
                return false;
            }
            occupied[i] = 1;
        }
    }
    FOREACH(it,occupied) {
        if( *it == 0 ) {
            return false;
        }
    }
    // check for repeating names
    FOREACHC(it,_vgroups) {
        for(std::vector<Group>::const_iterator it2 = it+1; it2 != _vgroups.end(); ++it2) {
            if( it->name == it2->name ) {
                return false;
            }
        }
    }
    return true;
}

bool ConfigurationSpecification::operator==(const ConfigurationSpecification& r) const
{
    if( _vgroups.size() != r._vgroups.size() ) {
        return false;
    }
    // the groups could be out of order
    for(size_t i = 0; i < _vgroups.size(); ++i) {
        size_t j;
        for(j=0; j < r._vgroups.size(); ++j) {
            if( _vgroups[i].offset == r._vgroups[j].offset ) {
                if( _vgroups[i] != r._vgroups[j] ) {
                    return false;
                }
                break;
            }
        }
        if( j >= r._vgroups.size() ) {
            return false;
        }
    }
    return true;
}

bool ConfigurationSpecification::operator!=(const ConfigurationSpecification& r) const
{
    return !this->operator==(r);
}

std::vector<ConfigurationSpecification::Group>::const_iterator ConfigurationSpecification::FindCompatibleGroup(const ConfigurationSpecification::Group& g, bool exactmatch) const
{
    std::vector<ConfigurationSpecification::Group>::const_iterator itsemanticmatch = _vgroups.end();
    uint32_t bestmatchscore = 0;
    stringstream ss(g.name);
    std::vector<std::string> tokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
    if( tokens.size() == 0 ) {
        return _vgroups.end();
    }
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name == g.name && itgroup->dof == g.dof) {
            return itgroup;
        }
        if( exactmatch ) {
            continue;
        }
        ss.clear();
        ss.str(itgroup->name);
        std::vector<std::string> curtokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
        if( curtokens.size() == 0 ) {
            continue;
        }
        if( curtokens.at(0) == tokens.at(0) ) {
            uint32_t matchscore=1;
            if( curtokens.size() > 1 && tokens.size() > 1 ) {
                if( curtokens.at(1) == tokens.at(1) ) {
                    matchscore += 0x80000000;
                }
                if( curtokens.size() > 2 && tokens.size() > 2 ) {
                    for(size_t i = 2; i < tokens.size(); ++i) {
                        if( find(curtokens.begin()+2,curtokens.end(),tokens[i]) != curtokens.end() ) {
                            matchscore += 1;
                        }
                    }
                }
            }
            if( bestmatchscore < matchscore ) {
                itsemanticmatch = itgroup;
            }
        }
    }
    return itsemanticmatch;
}

std::vector<ConfigurationSpecification::Group>::const_iterator ConfigurationSpecification::FindTimeDerivativeGroup(const ConfigurationSpecification::Group& g, bool exactmatch) const
{
    ConfigurationSpecification::Group gderivative;
    if( g.name.size() >= 12 && g.name.substr(0,12) == "joint_values" ) {
        gderivative.name = string("joint_velocities") + g.name.substr(12);
    }
    else if( g.name.size() >= 16 && g.name.substr(0,16) == "joint_velocities" ) {
        gderivative.name = string("joint_accelerations") + g.name.substr(16);
    }
    else if( g.name.size() >= 16 && g.name.substr(0,16) == "affine_transform" ) {
        gderivative.name = string("affine_velocities") + g.name.substr(16);
    }
    else if( g.name.size() >= 17 && g.name.substr(0,17) == "affine_velocities" ) {
        gderivative.name = string("affine_accelerations") + g.name.substr(17);
    }
    else if( g.name.size() >= 14 && g.name.substr(0,14) == "ikparam_values" ) {
        gderivative.name = string("ikparam_velocities") + g.name.substr(14);
    }
    else {
        return _vgroups.end();
    }
    gderivative.dof = g.dof;
    return FindCompatibleGroup(gderivative,exactmatch);
}

void ConfigurationSpecification::AddVelocityGroups(bool adddeltatime)
{
    if( _vgroups.size() == 0 ) {
        return;
    }
    std::list<std::vector<ConfigurationSpecification::Group>::iterator> listtoremove;
    std::list<ConfigurationSpecification::Group> listadd;
    int offset = GetDOF();
    bool hasdeltatime = false;
    FOREACH(itgroup,_vgroups) {
        string replacename;
        int offset = -1;
        if( itgroup->name.size() >= 12 && itgroup->name.substr(0,12) == "joint_values" ) {
            replacename = "joint_velocities";
            offset = 12;
        }
        else if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            replacename = "affine_velocities";
            offset = 16;
        }
        else if( itgroup->name.size() >= 14 && itgroup->name.substr(0,14) == "ikparam_values" ) {
            replacename = "ikparam_velocities";
            offset = 14;
        }

        if( offset > 0 ) {
            ConfigurationSpecification::Group g;
            g.name = replacename + itgroup->name.substr(offset);
            g.dof = itgroup->dof;
            std::vector<ConfigurationSpecification::Group>::const_iterator itcompat = FindCompatibleGroup(g);
            if( itcompat != _vgroups.end() ) {
                if( itcompat->dof == g.dof ) {
                    ConfigurationSpecification::Group& gmodify = _vgroups.at(itcompat-_vgroups.begin());
                    gmodify.name = g.name;
                }
                else {
                    listtoremove.push_back(_vgroups.begin()+(itcompat-_vgroups.begin()));
                    listadd.push_back(g);
                }
            }
            else {
                listadd.push_back(g);
            }
        }
        else {
            if( !hasdeltatime ) {
                hasdeltatime = itgroup->name.size() >= 9 && itgroup->name.substr(0,9) == "deltatime";
            }
        }
    }
    if( listtoremove.size() > 0 ) {
        FOREACH(it,listtoremove) {
            _vgroups.erase(*it);
        }
        ResetGroupOffsets();
        offset = GetDOF();
    }
    FOREACH(itadd, listadd) {
        itadd->offset = offset;
        offset += itadd->dof;
        _vgroups.push_back(*itadd);
    }
    if( !hasdeltatime && adddeltatime ) {
        AddDeltaTime();
    }
}

ConfigurationSpecification ConfigurationSpecification::ConvertToVelocitySpecification() const
{
    ConfigurationSpecification vspec;
    vspec._vgroups = _vgroups;
    FOREACH(itgroup,vspec._vgroups) {
        if( itgroup->name.size() >= 12 && itgroup->name.substr(0,12) == "joint_values" ) {
            itgroup->name = string("joint_velocities") + itgroup->name.substr(12);
        }
        else if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            itgroup->name = string("affine_velocities") + itgroup->name.substr(16);
        }
        else if( itgroup->name.size() >= 14 && itgroup->name.substr(0,14) == "ikparam_values" ) {
            itgroup->name = string("ikparam_velocities") + itgroup->name.substr(14);
        }
    }
    return vspec;
}

ConfigurationSpecification ConfigurationSpecification::GetTimeDerivativeSpecification(int timederivative) const
{
    ConfigurationSpecification vspec;
    vspec._vgroups = _vgroups;
    const boost::array<string,3> posgroups = {{"joint_values","affine_transform","ikparam_values"}};
    const boost::array<string,3> velgroups = {{"joint_velocities","affine_velocities","ikparam_velocities"}};
    const boost::array<string,3> accgroups = {{"joint_accelerations","affine_accelerations","ikparam_accelerations"}};
    const boost::array<string,3>* pgroup=NULL;
    if( timederivative == 0 ) {
        pgroup = &posgroups;
    }
    else if( timederivative == 1 ) {
        pgroup = &velgroups;
    }
    else if( timederivative == 2 ) {
        pgroup = &accgroups;
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0("invalid timederivative",ORE_InvalidArguments);
    }

    FOREACH(itgroup,vspec._vgroups) {
        for(size_t i = 0; i < pgroup->size(); ++i) {
            const string& name = pgroup->at(i);
            if( itgroup->name.size() >= name.size() && itgroup->name.substr(0,name.size()) == name ) {
                vspec._vgroups.push_back(*itgroup);
                break;
            }
        }
    }
    vspec.ResetGroupOffsets();
    return vspec;
}

void ConfigurationSpecification::ResetGroupOffsets()
{
    int offset = 0;
    FOREACH(it,_vgroups) {
        it->offset = offset;
        offset += it->dof;
    }
}

int ConfigurationSpecification::AddDeltaTime()
{
    int dof = 0;
    for(size_t i = 0; i < _vgroups.size(); ++i) {
        dof = max(dof,_vgroups[i].offset+_vgroups[i].dof);
        if( _vgroups[i].name == "deltatime" ) {
            return _vgroups[i].offset;
        }
    }
    ConfigurationSpecification::Group g;
    g.name = "deltatime";
    g.offset = dof;
    g.dof = 1;
    _vgroups.push_back(g);
    return g.offset;
}

bool ConfigurationSpecification::ExtractTransform(Transform& t, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody) const
{
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            stringstream ss(itgroup->name.substr(16));
            string bodyname;
            int affinedofs=0;
            ss >> bodyname >> affinedofs;
            if( !!ss ) {
                if( !!pbody && bodyname != pbody->GetName() ) {
                    continue;
                }
                RaveGetTransformFromAffineDOFValues(t,itdata+itgroup->offset,affinedofs);
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::ExtractIkParameterization(IkParameterization& ikparam, std::vector<dReal>::const_iterator itdata, int timederivative) const
{
    bool bfound = false;
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "ikparam_values"; break;
    case 1: searchname = "ikparam_velocities"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT0("bad time derivative",ORE_InvalidArguments);
    };
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            int iktype = IKP_None;
            ss >> iktype;
            if( !!ss ) {
                ikparam.Set(itdata+itgroup->offset,static_cast<IkParameterizationType>(iktype));
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::ExtractAffineValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int affinedofs, int timederivative) const
{
    if( affinedofs == 0 ) {
        return false;
    }
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "affine_values"; break;
    case 1: searchname = "affine_velocities"; break;
    case 2: searchname = "affine_accelerations"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT0("bad time derivative",ORE_InvalidArguments);
    };
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            string bodyname;
            int sourceaffinedofs=0;
            ss >> bodyname >> sourceaffinedofs;
            if( !!ss ) {
                if( !!pbody && bodyname != pbody->GetName() ) {
                    continue;
                }

                for(int index = 0; index < RaveGetAffineDOF(affinedofs); ++index) {
                    DOFAffine dof = RaveGetAffineDOFFromIndex(affinedofs,index);
                    int startindex = RaveGetIndexFromAffineDOF(affinedofs,dof);
                    if( sourceaffinedofs & dof ) {
                        int sourceindex = RaveGetIndexFromAffineDOF(sourceaffinedofs,dof);
                        *(itvalues+index) = *(itdata + sourceindex + (index-startindex));
                    }
                }
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::ExtractJointValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative) const
{
    if( indices.size() == 0 ) {
        return false;
    }
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "joint_values"; break;
    case 1: searchname = "joint_velocities"; break;
    case 2: searchname = "joint_accelerations"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT0("bad time derivative",ORE_InvalidArguments);
    };
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            string bodyname;
            ss >> bodyname;
            if( !!ss ) {
                if( !!pbody && bodyname != pbody->GetName() ) {
                    continue;
                }
                vector<int> vgroupindices((istream_iterator<int>(ss)), istream_iterator<int>());
                for(size_t i = 0; i < indices.size(); ++i) {
                    std::vector<int>::iterator it = find(vgroupindices.begin(),vgroupindices.end(),indices[i]);
                    if( it != vgroupindices.end() ) {
                        *(itvalues+i) = *(itdata+itgroup->offset+(it-vgroupindices.begin()));
                    }
                }
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::ExtractDeltaTime(dReal& deltatime, std::vector<dReal>::const_iterator itdata) const
{
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name == "deltatime" ) {
            deltatime = *(itdata+itgroup->offset);
            return true;
        }
    }
    return false;
}

bool ConfigurationSpecification::InsertJointValues(std::vector<dReal>::iterator itdata, std::vector<dReal>::const_iterator itvalues, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative) const
{
    if( indices.size() == 0 ) {
        return false;
    }
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "joint_values"; break;
    case 1: searchname = "joint_velocities"; break;
    case 2: searchname = "joint_accelerations"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT0("bad time derivative",ORE_InvalidArguments);
    };
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            string bodyname;
            ss >> bodyname;
            if( !!ss ) {
                if( !!pbody && bodyname != pbody->GetName() ) {
                    continue;
                }
                vector<int> vgroupindices((istream_iterator<int>(ss)), istream_iterator<int>());
                for(size_t i = 0; i < vgroupindices.size(); ++i) {
                    std::vector<int>::const_iterator it = find(indices.begin(),indices.end(),vgroupindices[i]);
                    if( it != indices.end() ) {
                        *(itdata+itgroup->offset+i) = *(itvalues+*it);
                    }
                }
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::InsertDeltaTime(std::vector<dReal>::iterator itdata, dReal deltatime)
{
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name == "deltatime" ) {
            *(itdata+itgroup->offset) = deltatime;
            bfound = true;
        }
    }
    return bfound;
}

static void ConvertDOFRotation_AxisFrom3D(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource, const Vector& vaxis)
{
    Vector axisangle(*(itsource+0),*(itsource+1),*(itsource+2));
    *ittarget = normalizeAxisRotation(vaxis,quatFromAxisAngle(axisangle)).first;
}

static void ConvertDOFRotation_AxisFromQuat(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource, const Vector& vaxis)
{
    Vector quat(*(itsource+0),*(itsource+1),*(itsource+2),*(itsource+3));
    *ittarget = normalizeAxisRotation(vaxis,quat).first;
}

static void ConvertDOFRotation_3DFromAxis(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource, const Vector& vaxis)
{
    *(ittarget+0) = vaxis[0]* *itsource;
    *(ittarget+1) = vaxis[1]* *itsource;
    *(ittarget+2) = vaxis[2]* *itsource;
}
static void ConvertDOFRotation_3DFromQuat(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource)
{
    Vector quat(*(itsource+0),*(itsource+1),*(itsource+2),*(itsource+3));
    Vector axisangle = quatFromAxisAngle(quat);
    *(ittarget+0) = axisangle[0];
    *(ittarget+1) = axisangle[1];
    *(ittarget+2) = axisangle[2];
}
static void ConvertDOFRotation_QuatFromAxis(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource, const Vector& vaxis)
{
    Vector axisangle = vaxis * *itsource;
    Vector quat = quatFromAxisAngle(axisangle);
    *(ittarget+0) = quat[0];
    *(ittarget+1) = quat[1];
    *(ittarget+2) = quat[2];
    *(ittarget+3) = quat[3];
}

static void ConvertDOFRotation_QuatFrom3D(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource)
{
    Vector axisangle(*(itsource+0),*(itsource+1),*(itsource+2));
    Vector quat = quatFromAxisAngle(axisangle);
    *(ittarget+0) = quat[0];
    *(ittarget+1) = quat[1];
    *(ittarget+2) = quat[2];
    *(ittarget+3) = quat[3];
}

void ConfigurationSpecification::ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const ConfigurationSpecification::Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const ConfigurationSpecification::Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv)
{
    if( numpoints > 1 ) {
        BOOST_ASSERT(targetstride != 0 && sourcestride != 0 );
    }
    if( gsource.name == gtarget.name ) {
        BOOST_ASSERT(gsource.dof==gtarget.dof);
        for(size_t i = 0; i < numpoints; ++i, itsourcedata += sourcestride, ittargetdata += targetstride) {
            std::copy(itsourcedata,itsourcedata+gsource.dof,ittargetdata);
        }
    }
    else {
        stringstream ss(gtarget.name);
        std::vector<std::string> targettokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
        ss.clear();
        ss.str(gsource.name);
        std::vector<std::string> sourcetokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());

        BOOST_ASSERT(targettokens.at(0) == sourcetokens.at(0));
        vector<int> vtransferindices; vtransferindices.reserve(gtarget.dof);
        std::vector<dReal> vdefaultvalues;
        if( targettokens.at(0).size() >= 6 && targettokens.at(0).substr(0,6) == "joint_") {
            std::vector<int> vsourceindices(gsource.dof), vtargetindices(gtarget.dof);
            if( (int)sourcetokens.size() < gsource.dof+2 ) {
                RAVELOG_DEBUG(str(boost::format("source tokens '%s' do not have %d dof indices, guessing....")%gsource.name%gsource.dof));
                for(int i = 0; i < gsource.dof; ++i) {
                    vsourceindices[i] = i;
                }
            }
            else {
                for(int i = 0; i < gsource.dof; ++i) {
                    vsourceindices[i] = boost::lexical_cast<int>(sourcetokens.at(i+2));
                }
            }
            if( (int)targettokens.size() < gtarget.dof+2 ) {
                RAVELOG_WARN(str(boost::format("target tokens '%s' do not match dof '%d', guessing....")%gtarget.name%gtarget.dof));
                for(int i = 0; i < gtarget.dof; ++i) {
                    vtargetindices[i] = i;
                }
            }
            else {
                for(int i = 0; i < gtarget.dof; ++i) {
                    vtargetindices[i] = boost::lexical_cast<int>(targettokens.at(i+2));
                }
            }

            bool bUninitializedData=false;
            FOREACH(ittargetindex,vtargetindices) {
                std::vector<int>::iterator it = find(vsourceindices.begin(),vsourceindices.end(),*ittargetindex);
                if( it == vsourceindices.end() ) {
                    bUninitializedData = true;
                    vtransferindices.push_back(-1);
                }
                else {
                    vtransferindices.push_back(static_cast<int>(it-vsourceindices.begin()));
                }
            }

            if( bUninitializedData ) {
                KinBodyPtr pbody;
                if( targettokens.size() > 1 ) {
                    pbody = penv->GetKinBody(targettokens.at(1));
                }
                if( !pbody && sourcetokens.size() > 1 ) {
                    pbody = penv->GetKinBody(sourcetokens.at(1));
                }
                if( !pbody ) {
                    RAVELOG_WARN(str(boost::format("could not find body '%s' or '%s'")%gtarget.name%gsource.name));
                    vdefaultvalues.resize(vtargetindices.size(),0);
                }
                else {
                    std::vector<dReal> vbodyvalues;
                    vdefaultvalues.resize(vtargetindices.size(),0);
                    if( targettokens[0] == "joint_values" ) {
                        pbody->GetDOFValues(vbodyvalues);
                    }
                    else if( targettokens[0] == "joint_velocities" ) {
                        pbody->GetDOFVelocities(vbodyvalues);
                    }
                    if( vbodyvalues.size() > 0 ) {
                        for(size_t i = 0; i < vdefaultvalues.size(); ++i) {
                            vdefaultvalues[i] = vbodyvalues.at(vtargetindices[i]);
                        }
                    }
                }
            }
        }
        else if( targettokens.at(0).size() >= 7 && targettokens.at(0).substr(0,7) == "affine_") {
            int affinesource = 0, affinetarget = 0;
            Vector sourceaxis(0,0,1), targetaxis(0,0,1);
            if( sourcetokens.size() < 3 ) {
                if( targettokens.size() < 3 && gsource.dof == gtarget.dof ) {
                    for(size_t i = 0; i < targettokens.size(); ++i) {
                        vtransferindices[i] = i;
                    }
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT("source affine information not present '%s'\n",gsource.name,ORE_InvalidArguments);
                }
            }
            else {
                affinesource = boost::lexical_cast<int>(sourcetokens.at(2));
                BOOST_ASSERT(RaveGetAffineDOF(affinesource) == gsource.dof);
                if( (affinesource & DOF_RotationAxis) && sourcetokens.size() >= 6 ) {
                    sourceaxis.x = boost::lexical_cast<dReal>(sourcetokens.at(3));
                    sourceaxis.y = boost::lexical_cast<dReal>(sourcetokens.at(4));
                    sourceaxis.z = boost::lexical_cast<dReal>(sourcetokens.at(5));
                }
            }
            if( vtransferindices.size() == 0 ) {
                if( targettokens.size() < 3 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("target affine information not present '%s'\n",gtarget.name,ORE_InvalidArguments);
                }
                else {
                    affinetarget = boost::lexical_cast<int>(targettokens.at(2));
                    BOOST_ASSERT(RaveGetAffineDOF(affinetarget) == gtarget.dof);
                    if( (affinetarget & DOF_RotationAxis) && targettokens.size() >= 6 ) {
                        targetaxis.x = boost::lexical_cast<dReal>(targettokens.at(3));
                        targetaxis.y = boost::lexical_cast<dReal>(targettokens.at(4));
                        targetaxis.z = boost::lexical_cast<dReal>(targettokens.at(5));
                    }
                }

                int commondata = affinesource&affinetarget;
                int uninitdata = affinetarget&(~commondata);
                int sourcerotationstart = -1, targetrotationstart = -1, targetrotationend = -1;
                boost::function< void(std::vector<dReal>::iterator, std::vector<dReal>::const_iterator) > rotconverterfn;
                if( (uninitdata & DOF_RotationMask) && (affinetarget & DOF_RotationMask) && (affinesource & DOF_RotationMask) ) {
                    // both hold rotations, but need to convert
                    uninitdata &= ~DOF_RotationMask;
                    sourcerotationstart = RaveGetIndexFromAffineDOF(affinesource,DOF_RotationMask);
                    targetrotationstart = RaveGetIndexFromAffineDOF(affinetarget,DOF_RotationMask);
                    targetrotationend = targetrotationstart+RaveGetAffineDOF(affinetarget&DOF_RotationMask);
                    if( affinetarget & DOF_RotationAxis ) {
                        if( affinesource & DOF_Rotation3D ) {
                            rotconverterfn = boost::bind(ConvertDOFRotation_AxisFrom3D,_1,_2,targetaxis);
                        }
                        else if( affinesource & DOF_RotationQuat ) {
                            rotconverterfn = boost::bind(ConvertDOFRotation_AxisFromQuat,_1,_2,targetaxis);
                        }
                    }
                    else if( affinetarget & DOF_Rotation3D ) {
                        if( affinesource & DOF_RotationAxis ) {
                            rotconverterfn = boost::bind(ConvertDOFRotation_3DFromAxis,_1,_2,sourceaxis);
                        }
                        else if( affinesource & DOF_RotationQuat ) {
                            rotconverterfn = ConvertDOFRotation_3DFromQuat;
                        }
                    }
                    else if( affinetarget & DOF_RotationQuat ) {
                        if( affinesource & DOF_RotationAxis ) {
                            rotconverterfn = boost::bind(ConvertDOFRotation_QuatFromAxis,_1,_2,sourceaxis);
                        }
                        else if( affinesource & DOF_Rotation3D ) {
                            rotconverterfn = ConvertDOFRotation_QuatFrom3D;
                        }
                    }
                    BOOST_ASSERT(!!rotconverterfn);
                }
                if( uninitdata ) {
                    // initialize with the current body values
                    KinBodyPtr pbody;
                    if( targettokens.size() > 1 ) {
                        pbody = penv->GetKinBody(targettokens.at(1));
                    }
                    if( !pbody && sourcetokens.size() > 1 ) {
                        pbody = penv->GetKinBody(sourcetokens.at(1));
                    }
                    if( !pbody ) {
                        RAVELOG_WARN(str(boost::format("could not find body '%s' or '%s'")%gtarget.name%gsource.name));
                        vdefaultvalues.resize(gtarget.dof,0);
                    }
                    else {
                        vdefaultvalues.resize(gtarget.dof);
                        RaveGetAffineDOFValuesFromTransform(vdefaultvalues.begin(),pbody->GetTransform(),affinetarget);
                    }
                }

                for(int index = 0; index < gtarget.dof; ++index) {
                    DOFAffine dof = RaveGetAffineDOFFromIndex(affinetarget,index);
                    int startindex = RaveGetIndexFromAffineDOF(affinetarget,dof);
                    if( affinesource & dof ) {
                        int sourceindex = RaveGetIndexFromAffineDOF(affinesource,dof);
                        vtransferindices.push_back(sourceindex + (index-startindex));
                    }
                    else {
                        vtransferindices.push_back(-1);
                    }
                }

                for(size_t i = 0; i < numpoints; ++i, itsourcedata += sourcestride, ittargetdata += targetstride) {
                    for(int j = 0; j < (int)vtransferindices.size(); ++j) {
                        if( vtransferindices[j] >= 0 ) {
                            *(ittargetdata+j) = *(itsourcedata+vtransferindices[j]);
                        }
                        else {
                            if( j >= targetrotationstart && j < targetrotationend ) {
                                if( j == targetrotationstart ) {
                                    // only convert when at first index
                                    rotconverterfn(ittargetdata+targetrotationstart,itsourcedata+sourcerotationstart);
                                }
                            }
                            else {
                                *(ittargetdata+j) = vdefaultvalues.at(j);
                            }
                        }
                    }
                }
                return;
            }
        }
        else if( targettokens.at(0).size() >= 8 && targettokens.at(0).substr(0,8) == "ikparam_") {
            IkParameterizationType iktypesource, iktypetarget;
            if( sourcetokens.size() >= 2 ) {
                iktypesource = static_cast<IkParameterizationType>(boost::lexical_cast<int>(sourcetokens[1]));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT("ikparam type not present '%s'\n",gsource.name,ORE_InvalidArguments);
            }
            if( targettokens.size() >= 2 ) {
                iktypetarget = static_cast<IkParameterizationType>(boost::lexical_cast<int>(targettokens[1]));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT("ikparam type not present '%s'\n",gtarget.name,ORE_InvalidArguments);
            }

            if( iktypetarget == iktypesource ) {
                vtransferindices.resize(IkParameterization::GetDOF(iktypetarget));
                for(size_t i = 0; i < vtransferindices.size(); ++i) {
                    vtransferindices[i] = i;
                }
            }
            else {
                RAVELOG_WARN("ikparam types do not match");
            }
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("unsupported token conversion: %s",gtarget.name,ORE_InvalidArguments);
        }

        for(size_t i = 0; i < numpoints; ++i, itsourcedata += sourcestride, ittargetdata += targetstride) {
            for(size_t j = 0; j < vtransferindices.size(); ++j) {
                if( vtransferindices[j] >= 0 ) {
                    *(ittargetdata+j) = *(itsourcedata+vtransferindices[j]);
                }
                else {
                    *(ittargetdata+j) = vdefaultvalues.at(j);
                }
            }
        }
    }
}

void ConfigurationSpecification::ConvertData(std::vector<dReal>::iterator ittargetdata, const ConfigurationSpecification &targetspec, std::vector<dReal>::const_iterator itsourcedata, const ConfigurationSpecification &sourcespec, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized)
{
    for(size_t igroup = 0; igroup < targetspec._vgroups.size(); ++igroup) {
        std::vector<ConfigurationSpecification::Group>::const_iterator itcompatgroup = sourcespec.FindCompatibleGroup(targetspec._vgroups[igroup]);
        if( itcompatgroup != sourcespec._vgroups.end() ) {
            ConfigurationSpecification::ConvertGroupData(ittargetdata+targetspec._vgroups[igroup].offset, targetspec.GetDOF(), targetspec._vgroups[igroup], itsourcedata+itcompatgroup->offset, sourcespec.GetDOF(), *itcompatgroup,numpoints,penv);
        }
        else if( filluninitialized ) {
            vector<dReal> vdefaultvalues(targetspec._vgroups[igroup].dof,0);
            const string& name = targetspec._vgroups[igroup].name;
            if( name.size() >= 12 && name.substr(0,12) == "joint_values" ) {
                string bodyname;
                stringstream ss(name.substr(12));
                ss >> bodyname;
                if( !!ss ) {
                    if( !!penv ) {
                        KinBodyPtr body = penv->GetKinBody(bodyname);
                        if( !!body ) {
                            vector<dReal> values;
                            body->GetDOFValues(values);
                            std::vector<int> indices((istream_iterator<int>(ss)), istream_iterator<int>());
                            for(size_t i = 0; i < indices.size(); ++i) {
                                vdefaultvalues.at(i) = values.at(indices[i]);
                            }
                        }
                    }
                }
            }
            else if( name.size() >= 16 && name.substr(0,16) == "affine_transform" ) {
                string bodyname;
                int affinedofs;
                stringstream ss(name.substr(16));
                ss >> bodyname >> affinedofs;
                if( !!ss ) {
                    Transform tdefault;
                    if( !!penv ) {
                        KinBodyPtr body = penv->GetKinBody(bodyname);
                        if( !!body ) {
                            tdefault = body->GetTransform();
                        }
                    }
                    BOOST_ASSERT((int)vdefaultvalues.size() == RaveGetAffineDOF(affinedofs));
                    RaveGetAffineDOFValuesFromTransform(vdefaultvalues.begin(),tdefault,affinedofs);
                }
            }
            else if( name != "deltatime" ) {
                RAVELOG_VERBOSE(str(boost::format("cannot initialize unknown group '%s'")%name));
            }
            int offset = targetspec._vgroups[igroup].offset;
            for(size_t i = 0; i < numpoints; ++i, offset += targetspec.GetDOF()) {
                for(size_t j = 0; j < vdefaultvalues.size(); ++j) {
                    *(ittargetdata+offset+j) = vdefaultvalues[j];
                }
            }
        }
    }
}

ConfigurationSpecification::Reader::Reader(ConfigurationSpecification& spec) : _spec(spec)
{
    _spec = ConfigurationSpecification(); // reset
}

BaseXMLReader::ProcessElement ConfigurationSpecification::Reader::startElement(const std::string& name, const AttributesList &atts)
{
    _ss.str(""); // have to clear the string
    if( name == "group" ) {
        _spec._vgroups.resize(_spec._vgroups.size()+1);
        ConfigurationSpecification::Group& g = _spec._vgroups.back();
        FOREACHC(itatt,atts) {
            if( itatt->first == "name" ) {
                g.name = itatt->second;
            }
            else if( itatt->first == "interpolation" ) {
                g.interpolation = itatt->second;
            }
            else if( itatt->first == "offset" ) {
                g.offset = boost::lexical_cast<int>(itatt->second);
            }
            else if( itatt->first == "dof" ) {
                g.dof = boost::lexical_cast<int>(itatt->second);
            }
        }
        return PE_Support;
    }
    return PE_Pass;
}

bool ConfigurationSpecification::Reader::endElement(const std::string& name)
{
    if( name == "configuration" ) {
        return true;
    }
    return false;
}

void ConfigurationSpecification::Reader::characters(const std::string& ch)
{
    _ss.clear();
    _ss << ch;
}

std::ostream& operator<<(std::ostream& O, const ConfigurationSpecification &spec)
{
    O << "<configuration>" << endl;
    FOREACHC(it,spec._vgroups) {
        O << "<group name=\"" << it->name << "\" offset=\"" << it->offset << "\" dof=\"" << it->dof << "\" interpolation=\"" << it->interpolation << "\"/>" << endl;
    }
    O << "</configuration>" << endl;
    return O;
}

std::istream& operator>>(std::istream& I, ConfigurationSpecification& spec)
{
    if( !!I) {
        stringbuf buf;
        stringstream::streampos pos = I.tellg();
        I.get(buf, 0); // get all the data, yes this is inefficient, not sure if there anyway to search in streams

        string pbuf = buf.str();
        const char* p = strcasestr(pbuf.c_str(), "</configuration>");
        int ppsize=-1;
        if( p != NULL ) {
            I.clear();
            ppsize=(p-pbuf.c_str())+20;
            I.seekg((size_t)pos+ppsize);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("error, failed to find </configuration> in %s",buf.str(),ORE_InvalidArguments);
        }
        ConfigurationSpecification::Reader reader(spec);
        LocalXML::ParseXMLData(BaseXMLReaderPtr(&reader,null_deleter()), pbuf.c_str(), ppsize);
        BOOST_ASSERT(spec.IsValid());
    }

    return I;
}

void CollisionReport::Reset(int coloptions)
{
    options = coloptions;
    minDistance = 1e20f;
    numCols = 0;
    numWithinTol = 0;
    contacts.resize(0);
    vLinkColliding.resize(0);
}

std::string CollisionReport::__str__() const
{
    stringstream s;
    s << "(";
    if( !!plink1 ) {
        s << plink1->GetParent()->GetName() << ":" << plink1->GetName();
    }
    s << ")x(";
    if( !!plink2 ) {
        s << plink2->GetParent()->GetName() << ":" << plink2->GetName();
    }
    s << ") contacts="<<contacts.size();
    return s.str();
}

// Dummy Reader
DummyXMLReader::DummyXMLReader(const std::string& fieldname, const std::string& pparentname, boost::shared_ptr<std::ostream> osrecord) : _fieldname(fieldname), _osrecord(osrecord)
{
    _parentname = pparentname;
    _parentname += ":";
    _parentname += _fieldname;
}

BaseXMLReader::ProcessElement DummyXMLReader::startElement(const std::string& name, const AttributesList &atts)
{
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(name, atts) == PE_Support )
            return PE_Support;
        return PE_Ignore;
    }

    if( !!_osrecord ) {
        *_osrecord << "<" << name << " ";
        FOREACHC(itatt, atts)
        *_osrecord << itatt->first << "=\"" << itatt->second << "\" ";
        *_osrecord << ">" << endl;
    }

    // create a new parser
    _pcurreader.reset(new DummyXMLReader(name, _parentname,_osrecord));
    return PE_Support;
}

bool DummyXMLReader::endElement(const std::string& name)
{
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(name) ) {
            _pcurreader.reset();
            if( !!_osrecord )
                *_osrecord << "</" << name << ">" << endl;
        }
        return false;
    }

    if( name == _fieldname )
        return true;
    RAVELOG_ERROR(str(boost::format("invalid xml tag %s\n")%name));
    return false;
}

void DummyXMLReader::characters(const std::string& ch)
{
    if( !_pcurreader ) {
        if( !!_osrecord )
            *_osrecord << ch;
    }
    else {
        _pcurreader->characters(ch);
    }
}

EnvironmentBase::EnvironmentBase()
{
    if( !RaveGlobalState() ) {
        RAVELOG_WARN("OpenRAVE global state not initialized! Need to call RaveInitialize before any OpenRAVE services can be used. For now, initializing with default parameters.\n");
        RaveInitialize(true);
    }
    __nUniqueId = RaveGlobal::instance()->RegisterEnvironment(this);
}

EnvironmentBase::~EnvironmentBase()
{
    RaveGlobal::instance()->UnregisterEnvironment(this);
}


bool SensorBase::SensorData::serialize(std::ostream& O) const
{
    RAVELOG_WARN("SensorData XML serialization not implemented\n");
    return true;
}

bool SensorBase::LaserSensorData::serialize(std::ostream& O) const
{
    RAVELOG_WARN("LaserSensorData XML serialization not implemented\n");
    return true;
}

bool SensorBase::CameraSensorData::serialize(std::ostream& O) const
{
    RAVELOG_WARN("CameraSensorData XML serialization not implemented\n");
    return true;
}

CollisionOptionsStateSaver::CollisionOptionsStateSaver(CollisionCheckerBasePtr p, int newoptions, bool required)
{
    _oldoptions = p->GetCollisionOptions();
    _p = p;
    if( !_p->SetCollisionOptions(newoptions) ) {
        if( required ) {
            throw openrave_exception(str(boost::format("Failed to set collision options %d in checker %s\n")%newoptions%_p->GetXMLId()));
        }
    }
}

CollisionOptionsStateSaver::~CollisionOptionsStateSaver()
{
    _p->SetCollisionOptions(_oldoptions);
}

void RaveInitRandomGeneration(uint32_t seed)
{
    RaveGlobal::instance()->GetDefaultSampler()->SetSeed(seed);
}

uint32_t RaveRandomInt()
{
    std::vector<uint32_t> sample;
    RaveGlobal::instance()->GetDefaultSampler()->SampleSequence(sample);
    return sample.at(0);
}

float RaveRandomFloat(IntervalType interval)
{
    std::vector<dReal> sample;
    RaveGlobal::instance()->GetDefaultSampler()->SampleSequence(sample,1,interval);
    return sample.at(0);
}

double RaveRandomDouble(IntervalType interval)
{
    std::vector<dReal> sample;
    RaveGlobal::instance()->GetDefaultSampler()->SampleSequence(sample,1,interval);
    return sample.at(0);
}

std::string GetMD5HashString(const std::string& s)
{
    if( s.size() == 0 )
        return "";

    md5_state_t state;
    md5_byte_t digest[16];

    md5_init(&state);
    md5_append(&state, (const md5_byte_t *)s.c_str(), s.size());
    md5_finish(&state, digest);
    string hex_output;
    hex_output.resize(32);
    for (int di = 0; di < 16; ++di) {
        int n = (digest[di]&0xf);
        hex_output[2*di+1] = n > 9 ? ('a'+n-10) : ('0'+n);
        n = (digest[di]&0xf0)>>4;
        hex_output[2*di+0] = n > 9 ? ('a'+n-10) : ('0'+n);
    }
    return hex_output;
}

std::string GetMD5HashString(const std::vector<uint8_t>&v)
{
    if( v.size() == 0 )
        return "";

    md5_state_t state;
    md5_byte_t digest[16];

    md5_init(&state);
    md5_append(&state, (const md5_byte_t *)&v[0], v.size());
    md5_finish(&state, digest);
    string hex_output;
    hex_output.resize(32);
    for (int di = 0; di < 16; ++di) {
        int n = (digest[di]&0xf);
        hex_output[2*di+0] = n > 9 ? ('a'+n-10) : ('0'+n);
        n = (digest[di]&0xf0)>>4;
        hex_output[2*di+1] = n > 9 ? ('a'+n-10) : ('0'+n);
    }
    return hex_output;
}

bool PairStringLengthCompare(const std::pair<std::string, std::string>&p0, const std::pair<std::string, std::string>&p1)
{
    return p0.first.size() > p1.first.size();
}

std::string& SearchAndReplace(std::string& out, const std::string& in, const std::vector< std::pair<std::string, std::string> >&_pairs)
{
    BOOST_ASSERT(&out != &in);
    FOREACHC(itp,_pairs) {
        BOOST_ASSERT(itp->first.size()>0);
    }
    std::vector< std::pair<std::string, std::string> > pairs = _pairs;
    stable_sort(pairs.begin(),pairs.end(),PairStringLengthCompare);
    out.resize(0);
    size_t startindex = 0;
    while(startindex < in.size()) {
        size_t nextindex=std::string::npos;
        std::vector< std::pair<std::string, std::string> >::const_iterator itbestp;
        FOREACHC(itp,pairs) {
            size_t index = in.find(itp->first,startindex);
            if((nextindex == std::string::npos)|| ((index != std::string::npos)&&(index < nextindex)) ) {
                nextindex = index;
                itbestp = itp;
            }
        }
        if( nextindex == std::string::npos ) {
            out += in.substr(startindex);
            break;
        }
        out += in.substr(startindex,nextindex-startindex);
        out += itbestp->second;
        startindex = nextindex+itbestp->first.size();
    }
    return out;
}

namespace LocalXML {

void RaveXMLErrorFunc(void *ctx, const char *msg, ...)
{
    va_list args;

    va_start(args, msg);
    RAVELOG_ERROR("XML Parse error: ");
    vprintf(msg,args);
    va_end(args);
}

struct XMLREADERDATA
{
    XMLREADERDATA(BaseXMLReaderPtr preader, xmlParserCtxtPtr ctxt) : _preader(preader), _ctxt(ctxt) {
    }
    BaseXMLReaderPtr _preader, _pdummy;
    xmlParserCtxtPtr _ctxt;
};

void DefaultStartElementSAXFunc(void *ctx, const xmlChar *name, const xmlChar **atts)
{
    AttributesList listatts;
    if( atts != NULL ) {
        for (int i = 0; (atts[i] != NULL); i+=2) {
            listatts.push_back(make_pair(string((const char*)atts[i]),string((const char*)atts[i+1])));
            std::transform(listatts.back().first.begin(), listatts.back().first.end(), listatts.back().first.begin(), ::tolower);
        }
    }

    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( !!pdata->_pdummy ) {
        RAVELOG_VERBOSE(str(boost::format("unknown field %s\n")%s));
        pdata->_pdummy->startElement(s,listatts);
    }
    else {
        if( ((XMLREADERDATA*)ctx)->_preader->startElement(s, listatts) != BaseXMLReader::PE_Support ) {
            // not handling, so create a temporary class to handle it
            pdata->_pdummy.reset(new DummyXMLReader(s,"(libxml)"));
        }
    }
}

void DefaultEndElementSAXFunc(void *ctx, const xmlChar *name)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    string s = (const char*)name;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if( !!pdata->_pdummy ) {
        if( pdata->_pdummy->endElement(s) ) {
            pdata->_pdummy.reset();
        }
    }
    else {
        if( pdata->_preader->endElement(s) ) {
            //RAVEPRINT(L"%s size read %d\n", name, data->_ctxt->input->consumed);
            xmlStopParser(pdata->_ctxt);
        }
    }
}

void DefaultCharactersSAXFunc(void *ctx, const xmlChar *ch, int len)
{
    XMLREADERDATA* pdata = (XMLREADERDATA*)ctx;
    if( !!pdata->_pdummy ) {
        pdata->_pdummy->characters(string((const char*)ch, len));
    }
    else {
        pdata->_preader->characters(string((const char*)ch, len));
    }
}

bool xmlDetectSAX2(xmlParserCtxtPtr ctxt)
{
    if (ctxt == NULL) {
        return false;
    }
#ifdef LIBXML_SAX1_ENABLED
    if ((ctxt->sax) &&  (ctxt->sax->initialized == XML_SAX2_MAGIC) && ((ctxt->sax->startElementNs != NULL) || (ctxt->sax->endElementNs != NULL))) {
        ctxt->sax2 = 1;
    }
#else
    ctxt->sax2 = 1;
#endif

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ((ctxt->str_xml==NULL) || (ctxt->str_xmlns==NULL) || (ctxt->str_xml_ns == NULL)) {
        return false;
    }
    return true;
}

bool ParseXMLData(BaseXMLReaderPtr preader, const char* buffer, int size)
{
    static xmlSAXHandler s_DefaultSAXHandler = { 0};
    if( size <= 0 ) {
        size = strlen(buffer);
    }
    if( !s_DefaultSAXHandler.initialized ) {
        // first time, so init
        s_DefaultSAXHandler.startElement = DefaultStartElementSAXFunc;
        s_DefaultSAXHandler.endElement = DefaultEndElementSAXFunc;
        s_DefaultSAXHandler.characters = DefaultCharactersSAXFunc;
        s_DefaultSAXHandler.error = RaveXMLErrorFunc;
        s_DefaultSAXHandler.initialized = 1;
    }

    xmlSAXHandlerPtr sax = &s_DefaultSAXHandler;
    int ret = 0;
    xmlParserCtxtPtr ctxt;

    ctxt = xmlCreateMemoryParserCtxt(buffer, size);
    if (ctxt == NULL) {
        return false;
    }
    if (ctxt->sax != (xmlSAXHandlerPtr) &xmlDefaultSAXHandler) {
        xmlFree(ctxt->sax);
    }
    ctxt->sax = sax;
    xmlDetectSAX2(ctxt);

    XMLREADERDATA reader(preader, ctxt);
    ctxt->userData = &reader;

    xmlParseDocument(ctxt);

    if (ctxt->wellFormed) {
        ret = 0;
    }
    else {
        if (ctxt->errNo != 0) {
            ret = ctxt->errNo;
        }
        else {
            ret = -1;
        }
    }
    if (sax != NULL) {
        ctxt->sax = NULL;
    }
    if (ctxt->myDoc != NULL) {
        xmlFreeDoc(ctxt->myDoc);
        ctxt->myDoc = NULL;
    }
    xmlFreeParserCtxt(ctxt);

    return ret==0;
}

}

} // end namespace OpenRAVE
