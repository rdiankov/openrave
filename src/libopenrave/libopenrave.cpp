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
#include <openrave/planningutils.h>

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

        _mapikparameterization[IkParameterization::Type_Transform6D] = "Transform6d";
        _mapikparameterization[IkParameterization::Type_Rotation3D] = "Rotation3D";
        _mapikparameterization[IkParameterization::Type_Translation3D] = "Translation3D";
        _mapikparameterization[IkParameterization::Type_Direction3D] = "Direction3D";
        _mapikparameterization[IkParameterization::Type_Ray4D] = "Ray4D";
        _mapikparameterization[IkParameterization::Type_Lookat3D] = "Lookat3D";
        _mapikparameterization[IkParameterization::Type_TranslationDirection5D] = "TranslationDirection5D";
        _mapikparameterization[IkParameterization::Type_TranslationXY2D] = "TranslationXY2D";
        _mapikparameterization[IkParameterization::Type_TranslationXYOrientation3D] = "TranslationXYOrientation3D";
        _mapikparameterization[IkParameterization::Type_TranslationLocalGlobal6D] = "TranslationLocalGlobal6D";
        BOOST_ASSERT(_mapikparameterization.size()==IkParameterization::Type_NumberOfParameterizations);
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

    boost::shared_ptr<void> RegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn)
    {
        boost::mutex::scoped_lock lock(_mutexXML);
        CreateXMLReaderFn oldfn = _mapreaders[type][xmltag];
        _mapreaders[type][xmltag] = fn;
        return boost::shared_ptr<void>((void*)1, boost::bind(&RaveGlobal::_UnregisterXMLReader,boost::weak_ptr<RaveGlobal>(shared_from_this()),type,xmltag,oldfn));
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
    const std::map<IkParameterization::Type,std::string>& GetIkParameterizationMap() {
        return _mapikparameterization;
    }

    const std::string& GetInterfaceName(InterfaceType type)
    {
        std::map<InterfaceType,std::string>::const_iterator it = _mapinterfacenames.find(type);
        if( it == _mapinterfacenames.end() ) {
            throw openrave_exception(str(boost::format("Invalid type %d specified")%type));
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
    static void _UnregisterXMLReader(boost::weak_ptr<RaveGlobal> pweakstate, InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& oldfn)
    {
        boost::shared_ptr<RaveGlobal> pstate = pweakstate.lock();
        if( !!pstate ) {
            boost::mutex::scoped_lock lock(pstate->_mutexXML);
            pstate->_mapreaders[type][xmltag] = oldfn;
        }
    }

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
    std::map<IkParameterization::Type,string> _mapikparameterization;
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

const std::map<IkParameterization::Type,std::string>& RaveGetIkParameterizationMap()
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

TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, int nDOF)
{
    TrajectoryBasePtr ptraj = RaveGlobal::instance()->GetDatabase()->CreateTrajectory(penv,"");
    if( !!ptraj ) {
        ptraj->Reset(nDOF);
    }
    return ptraj;
}

TrajectoryBasePtr RaveCreateTrajectory(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateTrajectory(penv, name);
}

SpaceSamplerBasePtr RaveCreateSpaceSampler(EnvironmentBasePtr penv, const std::string& name)
{
    return RaveGlobal::instance()->GetDatabase()->CreateSpaceSampler(penv, name);
}

boost::shared_ptr<void> RaveRegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn)
{
    return RaveGlobal::instance()->GetDatabase()->RegisterInterface(type, name, interfacehash,envhash,createfn);
}

boost::shared_ptr<void> RaveRegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn)
{
    return RaveGlobal::instance()->RegisterXMLReader(type,xmltag,fn);
}

BaseXMLReaderPtr RaveCallXMLReader(InterfaceType type, const std::string& xmltag, InterfaceBasePtr pinterface, const AttributesList& atts)
{
    return RaveGlobal::instance()->CallXMLReader(type,xmltag,pinterface,atts);
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
    if( !!plink1 )
        s << plink1->GetParent()->GetName() << ":" << plink1->GetName();
    s << ")x(";
    if( !!plink2 )
        s << plink2->GetParent()->GetName() << ":" << plink2->GetName();
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

BaseXMLReader::ProcessElement DummyXMLReader::startElement(const std::string& name, const AttributesList& atts)
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

void subtractstates(std::vector<dReal>& q1, const std::vector<dReal>& q2)
{
    BOOST_ASSERT(q1.size()==q2.size());
    for(size_t i = 0; i < q1.size(); ++i) {
        q1[i] -= q2[i];
    }
}

bool addstates(std::vector<dReal>& q, const std::vector<dReal>& qdelta, int fromgoal)
{
    BOOST_ASSERT(q.size()==qdelta.size());
    for(size_t i = 0; i < q.size(); ++i) {
        q[i] += qdelta[i];
    }
    return true;
}

PlannerBase::PlannerParameters::PlannerParameters() : XMLReadable("plannerparameters"), _fStepLength(0.04f), _nMaxIterations(0), _sPathOptimizationPlanner("shortcut_linear")
{
    _diffstatefn = subtractstates;
    _neighstatefn = addstates;
    _vXMLParameters.reserve(10);
    _vXMLParameters.push_back("_vinitialconfig");
    _vXMLParameters.push_back("_vgoalconfig");
    _vXMLParameters.push_back("_vconfiglowerlimit");
    _vXMLParameters.push_back("_vconfigupperlimit");
    _vXMLParameters.push_back("_vconfigresolution");
    _vXMLParameters.push_back("_nmaxiterations");
    _vXMLParameters.push_back("_fsteplength");
    _vXMLParameters.push_back("_pathoptimization");
}

PlannerBase::PlannerParameters& PlannerBase::PlannerParameters::operator=(const PlannerBase::PlannerParameters& r)
{
    // reset
    _costfn = r._costfn;
    _goalfn = r._goalfn;
    _distmetricfn = r._distmetricfn;
    _checkpathconstraintsfn = r._checkpathconstraintsfn;
    _samplefn = r._samplefn;
    _sampleneighfn = r._sampleneighfn;
    _samplegoalfn = r._samplegoalfn;
    _sampleinitialfn = r._sampleinitialfn;
    _setstatefn = r._setstatefn;
    _getstatefn = r._getstatefn;
    _diffstatefn = r._diffstatefn;
    _neighstatefn = r._neighstatefn;

    vinitialconfig.resize(0);
    vgoalconfig.resize(0);
    _vConfigLowerLimit.resize(0);
    _vConfigUpperLimit.resize(0);
    _vConfigResolution.resize(0);
    _sPathOptimizationPlanner = "shortcut_linear";
    _sPathOptimizationParameters.resize(0);
    _sExtraParameters.resize(0);
    _nMaxIterations = 0;
    _fStepLength = 0.04f;
    _plannerparametersdepth = 0;

    // transfer data
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1); /// have to do this or otherwise precision gets lost and planners' initial conditions can vioalte constraints
    ss << r;
    ss >> *this;
    return *this;
}

void PlannerBase::PlannerParameters::copy(boost::shared_ptr<PlannerParameters const> r)
{
    *this = *r;
}

bool PlannerBase::PlannerParameters::serialize(std::ostream& O) const
{
    O << "<_vinitialconfig>";
    FOREACHC(it, vinitialconfig) {
        O << *it << " ";
    }
    O << "</_vinitialconfig>" << endl;
    O << "<_vgoalconfig>";
    FOREACHC(it, vgoalconfig) {
        O << *it << " ";
    }
    O << "</_vgoalconfig>" << endl;
    O << "<_vconfiglowerlimit>";
    FOREACHC(it, _vConfigLowerLimit) {
        O << *it << " ";
    }
    O << "</_vconfiglowerlimit>" << endl;
    O << "<_vconfigupperlimit>";
    FOREACHC(it, _vConfigUpperLimit) {
        O << *it << " ";
    }
    O << "</_vconfigupperlimit>" << endl;
    O << "<_vconfigresolution>";
    FOREACHC(it, _vConfigResolution) {
        O << *it << " ";
    }
    O << "</_vconfigresolution>" << endl;

    O << "<_nmaxiterations>" << _nMaxIterations << "</_nmaxiterations>" << endl;
    O << "<_fsteplength>" << _fStepLength << "</_fsteplength>" << endl;
    O << "<_pathoptimization planner=\"" << _sPathOptimizationPlanner << "\">" << _sPathOptimizationParameters << "</_pathoptimization>" << endl;
    O << _sExtraParameters << endl;
    return !!O;
}

BaseXMLReader::ProcessElement PlannerBase::PlannerParameters::startElement(const std::string& name, const AttributesList& atts)
{
    _ss.str(""); // have to clear the string
    if( !!__pcurreader ) {
        if( __pcurreader->startElement(name, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    if( __processingtag.size() > 0 ) {
        return PE_Ignore;
    }
    if( name=="plannerparameters" ) {
        _plannerparametersdepth++;
        return PE_Support;
    }

    if( name == "_pathoptimization" ) {
        _sslocal.reset(new std::stringstream());
        _sPathOptimizationPlanner="";
        _sPathOptimizationParameters="";
        FOREACHC(itatt,atts) {
            if( itatt->first == "planner" ) {
                _sPathOptimizationPlanner = itatt->second;
            }
        }
        __pcurreader.reset(new DummyXMLReader(name,GetXMLId(),_sslocal));
        return PE_Support;
    }

    if( find(_vXMLParameters.begin(),_vXMLParameters.end(),name) == _vXMLParameters.end() ) {
        _sslocal.reset(new std::stringstream());
        *_sslocal << "<" << name << " ";
        FOREACHC(itatt, atts) {
            *_sslocal << itatt->first << "=\"" << itatt->second << "\" ";
        }
        *_sslocal << ">" << endl;
        __pcurreader.reset(new DummyXMLReader(name,GetXMLId(),_sslocal));
        return PE_Support;
    }

    if((name=="_vinitialconfig")||(name=="_vgoalconfig")||(name=="_vconfiglowerlimit")||(name=="_vconfigupperlimit")||(name=="_vconfigresolution")||(name=="_nmaxiterations")||(name=="_fsteplength")||(name=="_pathoptimization")) {
        __processingtag = name;
        return PE_Support;
    }
    return PE_Pass;
}

bool PlannerBase::PlannerParameters::endElement(const std::string& name)
{
    if( !!__pcurreader ) {
        if( __pcurreader->endElement(name) ) {
            boost::shared_ptr<DummyXMLReader> pdummy = boost::dynamic_pointer_cast<DummyXMLReader>(__pcurreader);
            if( !!pdummy ) {
                if( pdummy->GetFieldName() == "_pathoptimization" ) {
                    _sPathOptimizationParameters = _sslocal->str();
                    _sslocal.reset();
                }
                else {
                    *_sslocal << "</" << name << ">" << endl;
                    _sExtraParameters += _sslocal->str();
                    _sslocal.reset();
                }
            }
            __pcurreader.reset();
        }
    }
    else if( name == "plannerparameters" ) {
        return --_plannerparametersdepth < 0;
    }
    else if( __processingtag.size() > 0 ) {
        if( name == "_vinitialconfig") {
            vinitialconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vgoalconfig") {
            vgoalconfig = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vconfiglowerlimit") {
            _vConfigLowerLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vconfigupperlimit") {
            _vConfigUpperLimit = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_vconfigresolution") {
            _vConfigResolution = vector<dReal>((istream_iterator<dReal>(_ss)), istream_iterator<dReal>());
        }
        else if( name == "_nmaxiterations") {
            _ss >> _nMaxIterations;
        }
        else if( name == "_fsteplength") {
            _ss >> _fStepLength;
        }
        if( name !=__processingtag ) {
            RAVELOG_WARN(str(boost::format("invalid tag %s!=%s\n")%name%__processingtag));
        }
        __processingtag = "";
        return false;
    }

    return false;
}

void PlannerBase::PlannerParameters::characters(const std::string& ch)
{
    if( !!__pcurreader ) {
        __pcurreader->characters(ch);
    }
    else {
        _ss.clear();
        _ss << ch;
    }
}

std::ostream& operator<<(std::ostream& O, const PlannerBase::PlannerParameters& v)
{
    O << "<" << v.GetXMLId() << ">" << endl;
    v.serialize(O);
    O << "</" << v.GetXMLId() << ">" << endl;
    return O;
}

void PlannerBase::PlannerParameters::SetRobotActiveJoints(RobotBasePtr robot)
{
    using namespace planningutils;
    _distmetricfn = boost::bind(&SimpleDistanceMetric::Eval,boost::shared_ptr<SimpleDistanceMetric>(new SimpleDistanceMetric(robot)),_1,_2);
    SpaceSamplerBasePtr pconfigsampler = RaveCreateSpaceSampler(robot->GetEnv(),str(boost::format("robotconfiguration %s")%robot->GetName()));
    boost::shared_ptr<SimpleNeighborhoodSampler> defaultsamplefn(new SimpleNeighborhoodSampler(pconfigsampler,_distmetricfn));
    _samplefn = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1);
    _sampleneighfn = boost::bind(&SimpleNeighborhoodSampler::Sample,defaultsamplefn,_1,_2,_3);
    _setstatefn = boost::bind(&RobotBase::SetActiveDOFValues,robot,_1,false);
    _getstatefn = boost::bind(&RobotBase::GetActiveDOFValues,robot,_1);
    _diffstatefn = boost::bind(&RobotBase::SubtractActiveDOFValues,robot,_1,_2);
    _neighstatefn = addstates; // probably ok...

    boost::shared_ptr<LineCollisionConstraint> pcollision(new LineCollisionConstraint());
    _checkpathconstraintsfn = boost::bind(&LineCollisionConstraint::Check,pcollision,PlannerParametersWeakPtr(shared_parameters()), robot, _1, _2, _3, _4);

    robot->GetActiveDOFLimits(_vConfigLowerLimit,_vConfigUpperLimit);
    robot->GetActiveDOFResolutions(_vConfigResolution);
    robot->GetActiveDOFValues(vinitialconfig);
    BOOST_ASSERT((int)_vConfigResolution.size()==robot->GetActiveDOF());
}

bool PlannerBase::InitPlan(RobotBasePtr pbase, std::istream& isParameters)
{
    RAVELOG_WARN(str(boost::format("using default planner parameters structure to de-serialize parameters data inside %s, information might be lost!! Please define a InitPlan(robot,stream) function!\n")%GetXMLId()));
    boost::shared_ptr<PlannerParameters> localparams(new PlannerParameters());
    isParameters >> *localparams;
    return InitPlan(pbase,localparams);
}

bool PlannerBase::_OptimizePath(RobotBasePtr probot, TrajectoryBasePtr ptraj)
{
    if( GetParameters()->_sPathOptimizationPlanner.size() == 0 ) {
        return true;
    }
    PlannerBasePtr planner = RaveCreatePlanner(GetEnv(), GetParameters()->_sPathOptimizationPlanner);
    if( !planner ) {
        planner = RaveCreatePlanner(GetEnv(), "shortcut_linear");
        if( !planner ) {
            return false;
        }
    }
    PlannerParametersPtr params(new PlannerParameters());
    params->copy(GetParameters());
    params->_sExtraParameters += GetParameters()->_sPathOptimizationParameters;
    params->_sPathOptimizationPlanner = "";
    params->_sPathOptimizationParameters = "";
    params->_nMaxIterations = 0; // have to reset since path optimizers also use it and new parameters could be in extra parameters
    bool bSuccess = planner->InitPlan(probot, params) && planner->PlanPath(ptraj);

    if( !bSuccess && planner->GetXMLId() != "shortcut_linear") {
        RAVELOG_DEBUG("trying shortcut_linear\n");
        planner = RaveCreatePlanner(GetEnv(), "shortcut_linear");
        if( !!planner ) {
            bSuccess = planner->InitPlan(probot, params) && planner->PlanPath(ptraj);
        }
    }
    return bSuccess;
}

#ifdef _WIN32
const char *strcasestr(const char *s, const char *find)
{
    register char c, sc;
    register size_t len;

    if ((c = *find++) != 0) {
        c = tolower((unsigned char)c);
        len = strlen(find);
        do {
            do {
                if ((sc = *s++) == 0) {
                    return (NULL);
                }
            } while ((char)tolower((unsigned char)sc) != c);
        } while (strnicmp(s, find, len) != 0);
        s--;
    }
    return ((char *) s);
}
#endif

#define LIBXML_SAX1_ENABLED
#include <libxml/globals.h>
#include <libxml/xmlerror.h>
#include <libxml/parser.h>
#include <libxml/parserInternals.h> // only for xmlNewInputFromFile()
#include <libxml/tree.h>

#include <libxml/debugXML.h>
#include <libxml/xmlmemory.h>

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

std::istream& operator>>(std::istream& I, PlannerBase::PlannerParameters& pp)
{
    if( !!I) {
        stringbuf buf;
        stringstream::streampos pos = I.tellg();
        I.get(buf, 0); // get all the data, yes this is inefficient, not sure if there anyway to search in streams

        string pbuf = buf.str();
        const char* p = strcasestr(pbuf.c_str(), "</PlannerParameters>");
        int ppsize=-1;
        if( p != NULL ) {
            I.clear();
            ppsize=(p-pbuf.c_str())+20;
            I.seekg((size_t)pos+ppsize);
        }
        else
            throw openrave_exception(str(boost::format("error, failed to find </PlannerParameters> in %s")%buf.str()),ORE_InvalidArguments);
        pp._plannerparametersdepth = 0;
        LocalXML::ParseXMLData(PlannerBase::PlannerParametersPtr(&pp,null_deleter()), pbuf.c_str(), ppsize);
    }

    return I;
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

InterfaceBase::InterfaceBase(InterfaceType type, EnvironmentBasePtr penv) : __type(type), __penv(penv)
{
    RaveInitializeFromState(penv->GlobalState()); // make sure global state is set
    RegisterCommand("help",boost::bind(&InterfaceBase::_GetCommandHelp,this,_1,_2),
                    "display help commands.");
    __description = "Not documented yet.";
}

InterfaceBase::~InterfaceBase()
{
    boost::mutex::scoped_lock lock(_mutexInterface);
    __mapCommands.clear();
    __pUserData.reset();
    __mapReadableInterfaces.clear();
    __penv.reset();
}

void InterfaceBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    if( !preference ) {
        throw openrave_exception("invalid cloning reference",ORE_InvalidArguments);
    }
    __pUserData = preference->__pUserData;
    __struri = preference->__struri;
    __mapReadableInterfaces = preference->__mapReadableInterfaces;
}

bool InterfaceBase::SendCommand(ostream& sout, istream& sinput)
{
    string cmd;
    sinput >> cmd;
    if( !sinput ) {
        throw openrave_exception("invalid command",ORE_InvalidArguments);
    }
    boost::shared_ptr<InterfaceCommand> interfacecmd;
    {
        boost::mutex::scoped_lock lock(_mutexInterface);
        CMDMAP::iterator it = __mapCommands.find(cmd);
        if( it == __mapCommands.end() ) {
            throw openrave_exception(str(boost::format("failed to find command '%s' in interface %s\n")%cmd.c_str()%GetXMLId()),ORE_CommandNotSupported);
        }
        interfacecmd = it->second;
    }
    if( !interfacecmd->fn(sout,sinput) ) {
        RAVELOG_VERBOSE(str(boost::format("command failed in interface %s: %s\n")%GetXMLId()%cmd));
        return false;
    }
    return true;
}

void InterfaceBase::RegisterCommand(const std::string& cmdname, InterfaceBase::InterfaceCommandFn fncmd, const std::string& strhelp)
{
    boost::mutex::scoped_lock lock(_mutexInterface);
    if((cmdname.size() == 0)|| !IsValidName(cmdname) ||(stricmp(cmdname.c_str(),"commands") == 0)) {
        throw openrave_exception(str(boost::format("command '%s' invalid")%cmdname),ORE_InvalidArguments);
    }
    if( __mapCommands.find(cmdname) != __mapCommands.end() ) {
        throw openrave_exception(str(boost::format("command '%s' already registered")%cmdname),ORE_InvalidArguments);
    }
    __mapCommands[cmdname] = boost::shared_ptr<InterfaceCommand>(new InterfaceCommand(fncmd, strhelp));
}

void InterfaceBase::UnregisterCommand(const std::string& cmdname)
{
    boost::mutex::scoped_lock lock(_mutexInterface);
    CMDMAP::iterator it = __mapCommands.find(cmdname);
    if( it != __mapCommands.end() ) {
        __mapCommands.erase(it);
    }
}

bool InterfaceBase::_GetCommandHelp(std::ostream& o, std::istream& sinput) const
{
    boost::mutex::scoped_lock lock(_mutexInterface);
    string cmd, label;
    CMDMAP::const_iterator it;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput ) {
            break;
        }
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if( cmd == "commands" ) {
            for(it = __mapCommands.begin(); it != __mapCommands.end(); ++it) {
                o << it->first << " ";
            }
            return true;
        }
        else if( cmd == "label" ) {
            sinput >> label;
        }
        else {
            it = __mapCommands.find(cmd);
            if( it != __mapCommands.end() ) {
                o << it->second->help;
                return true;
            }
        }

        if( !sinput ) {
            RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
            return false;
        }
    }

    // display full help string
    o << endl << GetXMLId() << " Commands" << endl;
    for(size_t i = 0; i < GetXMLId().size(); ++i) {
        o << "=";
    }
    o << "=========" << endl << endl;
    for(it = __mapCommands.begin(); it != __mapCommands.end(); ++it) {
        if( label.size() > 0 ) {
            string strlower = it->first;
            std::transform(strlower.begin(), strlower.end(), strlower.begin(), ::tolower);
            o << endl << ".. _" << label << strlower << ":" << endl << endl;
        }
        o << endl << it->first << endl;
        for(size_t i = 0; i < it->first.size(); ++i) {
            o << "~";
        }
        o << endl << endl << it->second->help << endl << endl << "~~~~" << endl << endl;
    }
    return true;
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

/// SimpleSensorSystem
SimpleSensorSystem::SimpleXMLReader::SimpleXMLReader(boost::shared_ptr<XMLData> p) : _pdata(p)
{
}

BaseXMLReader::ProcessElement SimpleSensorSystem::SimpleXMLReader::startElement(const std::string& name, const AttributesList& atts)
{
    ss.str("");
    if((name != _pdata->GetXMLId())&&(name != "offsetlink")&&(name != "id")&&(name != "sid")&&(name != "translation")&&(name != "rotationmat")&&(name != "rotationaxis")&&(name != "quat")&&(name != "pretranslation")&&(name != "prerotation")&&(name != "prerotationaxis")&&(name != "prequat")) {
        return PE_Pass;
    }
    return PE_Support;
}

bool SimpleSensorSystem::SimpleXMLReader::endElement(const std::string& name)
{
    if( name == "offsetlink" )
        ss >> _pdata->strOffsetLink;
    else if( name == "id" )
        ss >> _pdata->id;
    else if( name == "sid" )
        ss >> _pdata->sid;
    else if( name == "translation" )
        ss >> _pdata->transOffset.trans.x >> _pdata->transOffset.trans.y >> _pdata->transOffset.trans.z;
    else if( name == "rotationmat" ) {
        TransformMatrix m;
        ss >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[4] >> m.m[5] >> m.m[6] >> m.m[8] >> m.m[9] >> m.m[10];
        _pdata->transOffset.rot = Transform(m).rot;
    }
    else if( name == "rotationaxis" ) {
        Vector axis; dReal fang;
        ss >> axis.x >> axis.y >> axis.z >> fang;
        _pdata->transOffset.rot = quatFromAxisAngle(axis,fang*dReal(PI/180.0));
    }
    else if( name == "quat" )
        ss >> _pdata->transOffset.rot;
    else if( name == "pretranslation")
        ss >> _pdata->transPreOffset.trans.x >> _pdata->transPreOffset.trans.y >> _pdata->transPreOffset.trans.z;
    else if( name == "prerotationmat") {
        TransformMatrix m;
        ss >> m.m[0] >> m.m[1] >> m.m[2] >> m.m[4] >> m.m[5] >> m.m[6] >> m.m[8] >> m.m[9] >> m.m[10];
        _pdata->transPreOffset.rot = Transform(m).rot;
    }
    else if( name == "prerotationaxis") {
        Vector axis; dReal fang;
        ss >> axis.x >> axis.y >> axis.z >> fang;
        _pdata->transPreOffset.rot = quatFromAxisAngle(axis,fang*dReal(PI/180.0));
    }
    else if( name == "prequat")
        ss >> _pdata->transPreOffset.rot;
    else if( name == tolowerstring(_pdata->GetXMLId()) )
        return true;

    if( !ss )
        RAVELOG_WARN(str(boost::format("error parsing %s\n")%name));
    return false;
}

void SimpleSensorSystem::SimpleXMLReader::characters(const std::string& ch)
{
    ss.clear();
    ss << ch;
}

BaseXMLReaderPtr SimpleSensorSystem::CreateXMLReaderId(const string& xmlid, InterfaceBasePtr ptr, const AttributesList& atts)
{
    return BaseXMLReaderPtr(new SimpleXMLReader(boost::shared_ptr<XMLData>(new XMLData(xmlid))));
}

boost::shared_ptr<void> SimpleSensorSystem::RegisterXMLReaderId(EnvironmentBasePtr penv, const string& xmlid)
{
    return RaveRegisterXMLReader(PT_KinBody,xmlid, boost::bind(&SimpleSensorSystem::CreateXMLReaderId,xmlid, _1,_2));
}

SimpleSensorSystem::SimpleSensorSystem(const std::string& xmlid, EnvironmentBasePtr penv) : SensorSystemBase(penv), _expirationtime(2000000), _bShutdown(false), _threadUpdate(boost::bind(&SimpleSensorSystem::_UpdateBodiesThread,this))
{
    _xmlid = xmlid;
    std::transform(_xmlid.begin(), _xmlid.end(), _xmlid.begin(), ::tolower);
}

SimpleSensorSystem::~SimpleSensorSystem()
{
    Reset();
    _bShutdown = true;
    _threadUpdate.join();
}

void SimpleSensorSystem::Reset()
{
    boost::mutex::scoped_lock lock(_mutex);
    _mapbodies.clear();
}

void SimpleSensorSystem::AddRegisteredBodies(const std::vector<KinBodyPtr>& vbodies)
{
    // go through all bodies in the environment and check for mocap data
    FOREACHC(itbody, vbodies) {
        boost::shared_ptr<XMLData> pmocapdata = boost::dynamic_pointer_cast<XMLData>((*itbody)->GetReadableInterface(_xmlid));
        if( !!pmocapdata ) {
            KinBody::ManageDataPtr p = AddKinBody(*itbody, pmocapdata);
            if( !!p ) {
                p->Lock(true);
            }
        }
    }
}

KinBody::ManageDataPtr SimpleSensorSystem::AddKinBody(KinBodyPtr pbody, XMLReadableConstPtr _pdata)
{
    BOOST_ASSERT(pbody->GetEnv()==GetEnv());
    boost::shared_ptr<XMLData const> pdata = boost::static_pointer_cast<XMLData const>(_pdata);
    if( !pdata ) {
        pdata = boost::dynamic_pointer_cast<XMLData const>(pbody->GetReadableInterface(_xmlid));
        if( !pdata ) {
            RAVELOG_VERBOSE(str(boost::format("failed to find manage data for body %s\n")%pbody->GetName()));
            return KinBody::ManageDataPtr();
        }
    }

    boost::mutex::scoped_lock lock(_mutex);
    if( _mapbodies.find(pbody->GetEnvironmentId()) != _mapbodies.end() ) {
        RAVELOG_WARN(str(boost::format("body %s already added\n")%pbody->GetName()));
        return KinBody::ManageDataPtr();
    }

    boost::shared_ptr<BodyData> b = CreateBodyData(pbody, pdata);
    b->lastupdated = GetMicroTime();
    _mapbodies[pbody->GetEnvironmentId()] = b;
    RAVELOG_VERBOSE(str(boost::format("system adding body %s (%s), total: %d\n")%pbody->GetName()%pbody->GetURI()%_mapbodies.size()));
    SetManageData(pbody,b);
    return b;
}

bool SimpleSensorSystem::RemoveKinBody(KinBodyPtr pbody)
{
    boost::mutex::scoped_lock lock(_mutex);
    bool bSuccess = _mapbodies.erase(pbody->GetEnvironmentId())>0;
    RAVELOG_VERBOSE(str(boost::format("system removing body %s %s\n")%pbody->GetName()%(bSuccess ? "succeeded" : "failed")));
    return bSuccess;
}

bool SimpleSensorSystem::IsBodyPresent(KinBodyPtr pbody)
{
    boost::mutex::scoped_lock lock(_mutex);
    return _mapbodies.find(pbody->GetEnvironmentId()) != _mapbodies.end();
}

bool SimpleSensorSystem::EnableBody(KinBodyPtr pbody, bool bEnable)
{
    boost::mutex::scoped_lock lock(_mutex);
    BODIES::iterator it = _mapbodies.find(pbody->GetEnvironmentId());
    if( it == _mapbodies.end() ) {
        RAVELOG_WARN("trying to %s body %s that is not in system\n", bEnable ? "enable" : "disable", pbody->GetName().c_str());
        return false;
    }

    it->second->bEnabled = bEnable;
    return true;
}

bool SimpleSensorSystem::SwitchBody(KinBodyPtr pbody1, KinBodyPtr pbody2)
{
    //boost::mutex::scoped_lock lock(_mutex);
    BODIES::iterator it = _mapbodies.find(pbody1->GetEnvironmentId());
    boost::shared_ptr<BodyData> pb1,pb2;
    if( it != _mapbodies.end() ) {
        pb1 = it->second;
    }
    it = _mapbodies.find(pbody2->GetEnvironmentId());
    if( it != _mapbodies.end() ) {
        pb2 = it->second;
    }
    if( !pb1 || !pb2 ) {
        return false;
    }
    if( !!pb1 ) {
        pb1->SetBody(pbody2);
    }
    if( !!pb2 ) {
        pb2->SetBody(pbody1);
    }
    return true;
}

boost::shared_ptr<SimpleSensorSystem::BodyData> SimpleSensorSystem::CreateBodyData(KinBodyPtr pbody, boost::shared_ptr<XMLData const> pdata)
{
    boost::shared_ptr<XMLData> pnewdata(new XMLData(_xmlid));
    pnewdata->copy(pdata);
    return boost::shared_ptr<BodyData>(new BodyData(RaveInterfaceCast<SimpleSensorSystem>(shared_from_this()),pbody, pnewdata));
}

void SimpleSensorSystem::_UpdateBodies(list<SimpleSensorSystem::SNAPSHOT>& listbodies)
{
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex()); // always lock environment to preserve mutex order
    uint64_t curtime = GetMicroTime();
    if( listbodies.size() > 0 ) {

        FOREACH(it, listbodies) {
            BOOST_ASSERT( it->first->IsEnabled() );

            KinBody::LinkPtr plink = it->first->GetOffsetLink();
            if( !plink )
                continue;

            // transform with respect to offset link
            TransformMatrix tlink = plink->GetTransform();
            TransformMatrix tbase = plink->GetParent()->GetTransform();
            TransformMatrix toffset = tbase * tlink.inverse() * it->first->_initdata->transOffset;
            TransformMatrix tfinal = toffset * it->second*it->first->_initdata->transPreOffset;

            plink->GetParent()->SetTransform(tfinal);
            it->first->lastupdated = curtime;
            it->first->tnew = it->second;

            //RAVELOG_DEBUG("%f %f %f\n", tfinal.trans.x, tfinal.trans.y, tfinal.trans.z);

            if( !it->first->IsPresent() )
                RAVELOG_VERBOSE(str(boost::format("updating body %s\n")%plink->GetParent()->GetName()));
            it->first->bPresent = true;
        }
    }

    boost::mutex::scoped_lock lock(_mutex);
    BODIES::iterator itbody = _mapbodies.begin();
    while(itbody != _mapbodies.end()) {
        KinBody::LinkPtr plink = itbody->second->GetOffsetLink();
        if( !!plink &&(plink->GetParent()->GetEnvironmentId()==0)) {
            _mapbodies.erase(itbody++);
            continue;
        }
        else if( curtime-itbody->second->lastupdated > _expirationtime ) {
            if( !itbody->second->IsLocked() ) {
                if( !!plink ) {
                    //RAVELOG_VERBOSE(str(boost::format("object %s expired %fs\n")%plink->GetParent()->GetName()*((curtime-itbody->second->lastupdated)*1e-6f)));
                    GetEnv()->Remove(plink->GetParent());
                }
                _mapbodies.erase(itbody++);
                continue;
            }

            if( itbody->second->IsPresent() && !!plink ) {
                RAVELOG_VERBOSE(str(boost::format("body %s not present\n")%plink->GetParent()->GetName()));
            }
            itbody->second->bPresent = false;
        }

        ++itbody;
    }
}

void SimpleSensorSystem::_UpdateBodiesThread()
{
    list< SNAPSHOT > listbodies;

    while(!_bShutdown) {
        {
            _UpdateBodies(listbodies);
        }
        Sleep(10); // 10ms
    }
}

MultiController::MultiController(EnvironmentBasePtr penv) : ControllerBase(penv), _nControlTransformation(0)
{
}

MultiController::~MultiController()
{
}

bool MultiController::Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
{
    boost::mutex::scoped_lock lock(_mutex);
    _probot=robot;
    if( !_probot ) {
        return false;
    }
    _listcontrollers.clear();
    _dofindices=dofindices;
    // reverse the mapping
    _dofreverseindices.resize(_probot->GetDOF());
    FOREACH(it,_dofreverseindices) {
        *it = -1;
    }
    int index = 0;
    FOREACH(it,_dofindices) {
        _dofreverseindices.at(*it) = index++;
    }
    _vcontrollersbydofs.resize(0); _vcontrollersbydofs.resize(_dofindices.size());
    _nControlTransformation = nControlTransformation;
    _ptransformcontroller.reset();
    return true;
}

const std::vector<int>& MultiController::GetControlDOFIndices() const {
    return _dofindices;
}
int MultiController::IsControlTransformation() const {
    return _nControlTransformation;
}
RobotBasePtr MultiController::GetRobot() const {
    return _probot;
}

bool MultiController::AttachController(ControllerBasePtr controller, const std::vector<int>& dofindices, int nControlTransformation)
{
    boost::mutex::scoped_lock lock(_mutex);
    if( nControlTransformation && !!_ptransformcontroller ) {
        throw openrave_exception("controller already attached for transformation",ORE_InvalidArguments);
    }
    FOREACHC(it,dofindices) {
        if( !!_vcontrollersbydofs.at(*it) ) {
            throw openrave_exception(str(boost::format("controller already attached to dof %d")%*it));
        }
    }
    if( !controller->Init(_probot,dofindices,nControlTransformation) ) {
        return false;
    }
    if( nControlTransformation ) {
        _ptransformcontroller = controller;
    }
    FOREACHC(it,dofindices) {
        _vcontrollersbydofs.at(*it) = controller;
    }
    _listcontrollers.push_back(controller);
    return true;
}

void MultiController::RemoveController(ControllerBasePtr controller)
{
    boost::mutex::scoped_lock lock(_mutex);
    _listcontrollers.remove(controller);
    if( _ptransformcontroller == controller ) {
        _ptransformcontroller.reset();
    }
    FOREACH(it,_vcontrollersbydofs) {
        if( *it == controller ) {
            it->reset();
        }
    }
}

ControllerBasePtr MultiController::GetController(int dof) const
{
    boost::mutex::scoped_lock lock(_mutex);
    if( dof < 0 ) {
        return _ptransformcontroller;
    }
    int index=0;
    FOREACHC(it,_dofindices) {
        if( *it == dof ) {
            return _vcontrollersbydofs.at(index);
        }
        index++;
    }
    return ControllerBasePtr();
}

void MultiController::Reset(int options)
{
    boost::mutex::scoped_lock lock(_mutex);
    FOREACH(itcontroller,_listcontrollers) {
        (*itcontroller)->Reset(options);
    }
}

bool MultiController::SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
{
    boost::mutex::scoped_lock lock(_mutex);
    vector<dReal> v;
    bool bsuccess = true;
    FOREACH(itcontroller,_listcontrollers) {
        v.resize(0);
        FOREACHC(it, (*itcontroller)->GetControlDOFIndices()) {
            v.push_back(values.at(_dofreverseindices.at(*it)));
        }
        bsuccess &= (*itcontroller)->SetDesired(v,trans);
    }
    return bsuccess;
}

bool MultiController::SetPath(TrajectoryBaseConstPtr ptraj)
{
    boost::mutex::scoped_lock lock(_mutex);
    bool bsuccess = true;
    if( !ptraj ) {
        FOREACH(itcontroller,_listcontrollers) {
            bsuccess &= (*itcontroller)->SetPath(TrajectoryBaseConstPtr());
        }
    }
    else {
        if( !_ptraj ||(_ptraj->GetXMLId() != ptraj->GetXMLId())) {
            _ptraj = RaveCreateTrajectory(ptraj->GetEnv(),ptraj->GetXMLId());
        }
    }
    return bsuccess;
}

void MultiController::SimulationStep(dReal fTimeElapsed)
{
    boost::mutex::scoped_lock lock(_mutex);
    FOREACH(it,_listcontrollers) {
        (*it)->SimulationStep(fTimeElapsed);
    }
}

bool MultiController::IsDone()
{
    boost::mutex::scoped_lock lock(_mutex);
    bool bdone=true;
    FOREACH(it,_listcontrollers) {
        bdone &= (*it)->IsDone();
    }
    return bdone;
}

dReal MultiController::GetTime() const
{
    boost::mutex::scoped_lock lock(_mutex);
    dReal t = 0;
    FOREACHC(it,_listcontrollers) {
        if( it == _listcontrollers.begin() ) {
            t = (*it)->GetTime();
        }
        else {
            dReal tnew = (*it)->GetTime();
            if( RaveFabs(t-tnew) > 0.000001 ) {
                RAVELOG_WARN(str(boost::format("multi-controller time is different! %f!=%f\n")%t%tnew));
            }
            t = max(t,tnew);
        }
    }
    return t;
}

void MultiController::GetVelocity(std::vector<dReal>& vel) const
{
    boost::mutex::scoped_lock lock(_mutex);
    vel.resize(_dofindices.size());
    FOREACH(it,vel) {
        *it = 0;
    }
    vector<dReal> v;
    FOREACHC(itcontroller,_listcontrollers) {
        (*itcontroller)->GetVelocity(v);
        int index=0;
        FOREACH(it,v) {
            vel.at(_dofreverseindices.at((*itcontroller)->GetControlDOFIndices().at(index++))) = *it;
        }
    }
}

void MultiController::GetTorque(std::vector<dReal>& torque) const
{
    boost::mutex::scoped_lock lock(_mutex);
    torque.resize(_dofindices.size());
    FOREACH(it,torque) {
        *it = 0;
    }
    vector<dReal> v;
    FOREACHC(itcontroller,_listcontrollers) {
        (*itcontroller)->GetTorque(v);
        int index=0;
        FOREACH(it,v) {
            torque.at(_dofreverseindices.at((*itcontroller)->GetControlDOFIndices().at(index++))) = *it;
        }
    }
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

//void RobotBase::GetControlMaxTorques(std::vector<dReal>& maxtorques) const
//{
//    if( _nActiveDOF < 0 ) {
//        GetDOFMaxTorque(maxtorques);
//        return;
//    }
//    maxtorques.resize(GetActiveDOF());
//    if( maxtorques.size() == 0 ) {
//        return;
//    }
//    dReal* pMaxTorques = &maxtorques[0];
//
//    if( _vActiveJointIndices.size() != 0 ) {
//        GetDOFMaxTorque(_vTempRobotJoints);
//
//        FOREACHC(it, _vActiveJointIndices)
//            *pMaxTorques++ = _vTempRobotJoints[*it];
//    }
//
//    if( _nAffineDOFs == DOF_NoTransform )
//        return;
//
//    if( _nAffineDOFs & DOF_X ) *pMaxTorques++ = 0;
//    if( _nAffineDOFs & DOF_Y ) *pMaxTorques++ = 0;
//    if( _nAffineDOFs & DOF_Z ) *pMaxTorques++ = 0;
//    if( _nAffineDOFs & DOF_RotationAxis ) *pMaxTorques++ = 0;
//    else if( _nAffineDOFs & DOF_Rotation3D ) {
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//    }
//    else if( _nAffineDOFs & DOF_RotationQuat ) {
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//    }
//}
//
//void RobotBase::SetControlTorques(const std::vector<dReal>& vtorques)
//{
//    if(_nActiveDOF < 0) {
//        SetJointTorques(vtorques, false);
//        return;
//    }
//
//    if( _vActiveJointIndices.size() > 0 ) {
//        _vTempRobotJoints.resize(GetDOF());
//        std::vector<dReal>::const_iterator ittorque = vtorques.begin();
//        FOREACHC(it, _vActiveJointIndices)
//            _vTempRobotJoints[*it] = *ittorque++;
//        SetJointTorques(_vTempRobotJoints,false);
//    }
//}

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

std::string GetMD5HashString(const std::vector<uint8_t>& v)
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

bool PairStringLengthCompare(const std::pair<std::string, std::string>& p0, const std::pair<std::string, std::string>& p1)
{
    return p0.first.size() > p1.first.size();
}

std::string& SearchAndReplace(std::string& out, const std::string& in, const std::vector< std::pair<std::string, std::string> >& _pairs)
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

} // end namespace OpenRAVE
