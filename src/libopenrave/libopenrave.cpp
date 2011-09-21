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
    if( name == "offsetlink" ) {
        ss >> _pdata->strOffsetLink;
    }
    else if( name == "id" ) {
        ss >> _pdata->id;
    }
    else if( name == "sid" ) {
        ss >> _pdata->sid;
    }
    else if( name == "translation" ) {
        ss >> _pdata->transOffset.trans.x >> _pdata->transOffset.trans.y >> _pdata->transOffset.trans.z;
    }
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
    else if( name == "quat" ) {
        ss >> _pdata->transOffset.rot;
    }
    else if( name == "pretranslation") {
        ss >> _pdata->transPreOffset.trans.x >> _pdata->transPreOffset.trans.y >> _pdata->transPreOffset.trans.z;
    }
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
    else if( name == "prequat") {
        ss >> _pdata->transPreOffset.rot;
    }
    else if( name == tolowerstring(_pdata->GetXMLId()) ) {
        return true;
    }
    if( !ss ) {
        RAVELOG_WARN(str(boost::format("error parsing %s\n")%name));
    }
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
            if( !plink ) {
                continue;
            }
            // transform with respect to offset link
            TransformMatrix tlink = plink->GetTransform();
            TransformMatrix tbase = plink->GetParent()->GetTransform();
            TransformMatrix toffset = tbase * tlink.inverse() * it->first->_initdata->transOffset;
            TransformMatrix tfinal = toffset * it->second*it->first->_initdata->transPreOffset;

            plink->GetParent()->SetTransform(tfinal);
            it->first->lastupdated = curtime;
            it->first->tnew = it->second;

            if( !it->first->IsPresent() ) {
                RAVELOG_VERBOSE(str(boost::format("updating body %s\n")%plink->GetParent()->GetName()));
            }
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
