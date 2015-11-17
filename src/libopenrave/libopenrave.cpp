// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/types.h>
#include <libintl.h>
#endif

#include <locale>
#include <set>

#include "plugindatabase.h"

#include <boost/algorithm/string/trim.hpp>
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

#ifdef HAS_FENV_H
#include <fenv.h>
#endif

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
dReal RaveCeil(dReal f) {
    return ceil(f);
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
dReal RaveCeil(dReal f) {
    return ceilf(f);
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
dReal RaveCeil(dReal f) {
    return ceil(f);
}
#endif

#endif

static std::set<std::string> _gettextDomainsInitialized;
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
        _nDataAccessOptions = 0;

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

        _mapikparameterization[IKP_Transform6D] = "Transform6D";
        _mapikparameterization[IKP_Rotation3D] = "Rotation3D";
        _mapikparameterization[IKP_Translation3D] = "Translation3D";
        _mapikparameterization[IKP_Direction3D] = "Direction3D";
        _mapikparameterization[IKP_Ray4D] = "Ray4D";
        _mapikparameterization[IKP_Lookat3D] = "Lookat3D";
        _mapikparameterization[IKP_TranslationDirection5D] = "TranslationDirection5D";
        _mapikparameterization[IKP_TranslationXY2D] = "TranslationXY2D";
        _mapikparameterization[IKP_TranslationXYOrientation3D] = "TranslationXYOrientation3D";
        _mapikparameterization[IKP_TranslationLocalGlobal6D] = "TranslationLocalGlobal6D";
        _mapikparameterization[IKP_TranslationXAxisAngle4D] = "TranslationXAxisAngle4D";
        _mapikparameterization[IKP_TranslationYAxisAngle4D] = "TranslationYAxisAngle4D";
        _mapikparameterization[IKP_TranslationZAxisAngle4D] = "TranslationZAxisAngle4D";
        _mapikparameterization[IKP_TranslationXAxisAngleZNorm4D] = "TranslationXAxisAngleZNorm4D";
        _mapikparameterization[IKP_TranslationYAxisAngleXNorm4D] = "TranslationYAxisAngleXNorm4D";
        _mapikparameterization[IKP_TranslationZAxisAngleYNorm4D] = "TranslationZAxisAngleYNorm4D";
        BOOST_ASSERT(_mapikparameterization.size()==IKP_NumberOfParameterizations);
        FOREACH(it,_mapikparameterization) {
            std::string name = it->second;
            std::transform(name.begin(), name.end(), name.begin(), ::tolower);
            _mapikparameterizationlower[it->first] = name;
        }
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
            // TODO: eventually we should remove this call to set global locale for the process
            // and imbue each stringstream with the correct locale.

            // set to the classic locale so that number serialization/hashing works correctly
            // std::locale::global(std::locale::classic());
            std::locale::global(std::locale(std::locale(""), std::locale::classic(), std::locale::numeric));
        }
        catch(const std::runtime_error& e) {
            RAVELOG_WARN("failed to set to C locale: %s\n",e.what());
        }

        _nDebugLevel = level;
        _pdatabase.reset(new RaveDatabase());
        if( !_pdatabase->Init(bLoadAllPlugins) ) {
            RAVELOG_FATAL("failed to create the openrave plugin database\n");
        }

        char* phomedir = getenv("OPENRAVE_HOME"); // getenv not thread-safe?
        if( phomedir == NULL ) {
#ifndef _WIN32
            _homedirectory = string(getenv("HOME"))+string("/.openrave"); // getenv not thread-safe?
#else
            _homedirectory = string(getenv("HOMEDRIVE"))+string(getenv("HOMEPATH"))+string("\\.openrave"); // getenv not thread-safe?
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

#ifdef _WIN32
        const char* delim = ";";
#else
        const char* delim = ":";
#endif
        _vdbdirectories.clear();
        char* pOPENRAVE_PLUGINS = getenv("OPENRAVE_DATABASE"); // getenv not thread-safe?
        if( pOPENRAVE_PLUGINS != NULL ) {
            utils::TokenizeString(pOPENRAVE_PLUGINS, delim, _vdbdirectories);
        }
        _vdbdirectories.push_back(_homedirectory);

        _UpdateDataDirs();
        return 0;
    }

    void Destroy()
    {
        // don't use any log statements since global instance might be null
        // environments have to be destroyed carefully since their destructors can be called, which will attempt to unregister the environment
        std::map<int, EnvironmentBase*> mapenvironments;
        {
            boost::mutex::scoped_lock lock(_mutexinternal);
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

        // process the callbacks
        std::list<boost::function<void()> > listDestroyCallbacks;
        {
            boost::mutex::scoped_lock lock(_mutexinternal);
            listDestroyCallbacks.swap(_listDestroyCallbacks);
        }
        FOREACH(itcallback, listDestroyCallbacks) {
            (*itcallback)();
        }
        listDestroyCallbacks.clear();

        if( !!_pdatabase ) {
            // force destroy in case some one is holding a pointer to it
            _pdatabase->Destroy();
            _pdatabase.reset();
        }
#ifdef USE_CRLIBM

#ifdef HAS_FENV_H
        feclearexcept(-1); // clear any cached exceptions
#endif
        crlibm_exit(_crlibm_fpu_state);
#endif
    }

    void AddCallbackForDestroy(const boost::function<void()>& fn)
    {
        boost::mutex::scoped_lock lock(_mutexinternal);
        _listDestroyCallbacks.push_back(fn);
    }

    std::string GetHomeDirectory()
    {
        return _homedirectory;
    }

    std::string FindDatabaseFile(const std::string& filename, bool bRead)
    {
        FOREACH(itdirectory,_vdbdirectories) {
#ifdef HAVE_BOOST_FILESYSTEM
            std::string fullfilename = boost::filesystem::absolute(boost::filesystem::path(*itdirectory)/filename).string();
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
            boost::mutex::scoped_lock lock(global->_mutexinternal);
            _oldfn = global->_mapreaders[_type][_xmltag];
            global->_mapreaders[_type][_xmltag] = fn;
        }
        virtual ~XMLReaderFunctionData()
        {
            boost::shared_ptr<RaveGlobal> global = _global.lock();
            if( !!global ) {
                boost::mutex::scoped_lock lock(global->_mutexinternal);
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
            //throw openrave_exception(str(boost::format(_("No function registered for interface %s xml tag %s"))%GetInterfaceName(type)%xmltag),ORE_InvalidArguments);
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
    const std::map<IkParameterizationType,std::string>& GetIkParameterizationMap(int alllowercase=0) {
        if( alllowercase ) {
            return _mapikparameterizationlower;
        }
        return _mapikparameterization;
    }

    const std::string& GetInterfaceName(InterfaceType type)
    {
        std::map<InterfaceType,std::string>::const_iterator it = _mapinterfacenames.find(type);
        if( it == _mapinterfacenames.end() ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("Invalid type %d specified"), type, ORE_Failed);
        }
        return it->second;
    }

    // have to take in pointer instead of shared_ptr since method will be called in EnvironmentBase constructor
    int RegisterEnvironment(EnvironmentBase* penv)
    {
        BOOST_ASSERT(!!_pdatabase);
        boost::mutex::scoped_lock lock(_mutexinternal);
        _mapenvironments[++_nGlobalEnvironmentId] = penv;
        return _nGlobalEnvironmentId;
    }

    void UnregisterEnvironment(EnvironmentBase* penv)
    {
        boost::mutex::scoped_lock lock(_mutexinternal);
        FOREACH(it, _mapenvironments) {
            if( it->second == penv ) {
                _mapenvironments.erase(it);
                break;
            }
        }
    }

    int GetEnvironmentId(EnvironmentBaseConstPtr penv)
    {
        return !!penv ? penv->GetId() : 0;
//        boost::mutex::scoped_lock lock(_mutexinternal);
//        FOREACH(it,_mapenvironments) {
//            if( it->second == penv.get() ) {
//                return it->first;
//            }
//        }
//        return 0;
    }

    EnvironmentBasePtr GetEnvironment(int id)
    {
        boost::mutex::scoped_lock lock(_mutexinternal);
        std::map<int, EnvironmentBase*>::iterator it = _mapenvironments.find(id);
        if( it == _mapenvironments.end() ) {
            return EnvironmentBasePtr();
        }
        return it->second->shared_from_this();
    }

    void GetEnvironments(std::list<EnvironmentBasePtr>& listenvironments)
    {
        listenvironments.clear();
        boost::mutex::scoped_lock lock(_mutexinternal);
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
            boost::mutex::scoped_lock lock(_mutexinternal);
            BOOST_ASSERT( _mapenvironments.size() > 0 );
            _pdefaultsampler = GetDatabase()->CreateSpaceSampler(_mapenvironments.begin()->second->shared_from_this(),"MT19937");
        }
        return _pdefaultsampler;
    }

    std::string FindLocalFile(const std::string& _filename, const std::string& curdir)
    {
#ifndef HAVE_BOOST_FILESYSTEM
        throw OPENRAVE_EXCEPTION_FORMAT0(_("need to compile with boost::filesystem"),ORE_Assert);
#else
        if( _filename.size() == 0 ) {
            return std::string();
        }

        boost::mutex::scoped_lock lock(_mutexinternal);
        boost::filesystem::path fullfilename;
        boost::filesystem::path filename(_filename);

        if( filename.is_complete() ) {
            fullfilename = filename;
        }
        else if( curdir.size() > 0 ) {
            fullfilename = boost::filesystem::absolute(boost::filesystem::path(curdir)) / filename;
        }
        else {
            fullfilename = boost::filesystem::current_path() / filename;
        }

        if( boost::filesystem::exists(fullfilename) ) {
            if( !_ValidateFilename(fullfilename,boost::filesystem::path()) ) {
                RAVELOG_WARN(str(boost::format("acess denied to file %s\n")%fullfilename));
                return std::string();
            }
            return fullfilename.string();
        }

        // try the openrave directories
        FOREACHC(itdir, _vBoostDataDirs) {
            fullfilename = *itdir / filename;
            if( _ValidateFilename(fullfilename,boost::filesystem::path()) ) {
                return fullfilename.string();
            }
        }

        RAVELOG_WARN(str(boost::format("could not find file %s\n")%filename));
        return std::string();
#endif
    }

    bool InvertFileLookup(std::string& newfilename, const std::string& filename)
    {
#ifndef HAVE_BOOST_FILESYSTEM
        RAVELOG_WARN("need to compile with boost::filesystem\n");
#else
        // check if filename is within _vBoostDataDirs
        boost::filesystem::path fullfilename = boost::filesystem::absolute(filename);
        _CustomNormalizePath(fullfilename);
        FOREACHC(itpath,_vBoostDataDirs) {
            std::list<boost::filesystem::path> listfilenameparts;
            boost::filesystem::path testfilename = fullfilename.parent_path();
            while( testfilename >= *itpath ) {
                if( testfilename == *itpath ) {
                    boost::filesystem::path relpath;
                    FOREACH(itpart,listfilenameparts) {
                        relpath /= *itpart;
                    }
                    relpath /= fullfilename.filename();
                    newfilename=relpath.string();
                    return true;
                }
                listfilenameparts.push_front(testfilename.filename());
                testfilename = testfilename.parent_path();
            }
        }
#endif
        return false;
    }

    void SetDataAccess(int options) {
        boost::mutex::scoped_lock lock(_mutexinternal);
        _nDataAccessOptions = options;
    }
    int GetDataAccess() {
        boost::mutex::scoped_lock lock(_mutexinternal);
        return _nDataAccessOptions;
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

    void _UpdateDataDirs()
    {
        _vdatadirs.resize(0);

        bool bExists=false;
#ifdef _WIN32
        const char* delim = ";";
#else
        const char* delim = ":";
#endif
        char* pOPENRAVE_DATA = getenv("OPENRAVE_DATA"); // getenv not thread-safe?
        if( pOPENRAVE_DATA != NULL ) {
            utils::TokenizeString(pOPENRAVE_DATA, delim, _vdatadirs);
        }
        string installdir = OPENRAVE_DATA_INSTALL_DIR;
#ifdef HAVE_BOOST_FILESYSTEM
        if( !boost::filesystem::is_directory(boost::filesystem::path(installdir)) ) {
#ifdef _WIN32
            HKEY hkey;
            if(RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\OpenRAVE\\" OPENRAVE_VERSION_STRING), 0, KEY_QUERY_VALUE, &hkey) == ERROR_SUCCESS) {
                DWORD dwType = REG_SZ;
                CHAR szInstallRoot[4096];     // dont' take chances, it is windows
                DWORD dwSize = sizeof(szInstallRoot);
                RegQueryValueEx(hkey, TEXT("InstallRoot"), NULL, &dwType, (PBYTE)szInstallRoot, &dwSize);
                RegCloseKey(hkey);
                installdir.assign(szInstallRoot);
                installdir += str(boost::format("%cshare%copenrave-%d.%d")%s_filesep%s_filesep%OPENRAVE_VERSION_MAJOR%OPENRAVE_VERSION_MINOR);
                RAVELOG_VERBOSE(str(boost::format("window registry data dir '%s'")%installdir));
            }
            else
#endif
            {
                RAVELOG_WARN(str(boost::format("%s doesn't exist")%installdir));
            }
        }

        boost::filesystem::path datafilename = boost::filesystem::absolute(boost::filesystem::path(installdir));
        FOREACH(itname, _vdatadirs) {
            if( datafilename == boost::filesystem::absolute(boost::filesystem::path(*itname)) ) {
                bExists = true;
                break;
            }
        }
#else
        string datafilename=installdir;
        FOREACH(itname, _vdatadirs) {
            if( datafilename == installdir ) {
                bExists = true;
                break;
            }
        }
#endif
        if( !bExists ) {
            _vdatadirs.push_back(installdir);
        }
        FOREACHC(itdir,_vdatadirs) {
            RAVELOG_VERBOSE(str(boost::format("data dir: %s")%*itdir));
        }

#ifdef HAVE_BOOST_FILESYSTEM
        _vBoostDataDirs.resize(0);
        FOREACHC(itfilename,_vdatadirs) {
            boost::filesystem::path fullfilename = boost::filesystem::absolute(boost::filesystem::path(*itfilename));
            _CustomNormalizePath(fullfilename);
            if( fullfilename.filename() == "." ) {
                // fullfilename ends in '/', so remove it
                fullfilename = fullfilename.parent_path();
            }
            _vBoostDataDirs.push_back(fullfilename);
        }
#endif
    }

#ifdef HAVE_BOOST_FILESYSTEM

    void _CustomNormalizePath(boost::filesystem::path& p)
    {
#ifndef BOOST_FILESYSTEM_NO_DEPRECATED
        p.normalize();
#else
        boost::filesystem::path result;
        for(boost::filesystem::path::iterator it=p.begin(); it!=p.end(); ++it)
        {
            if(*it == "..") {
                // /a/b/.. is not necessarily /a if b is a symbolic link
                if(boost::filesystem::is_symlink(result) ) {
                    result /= *it;
                }
                // /a/b/../.. is not /a/b/.. under most circumstances
                // We can end up with ..s in our result because of symbolic links
                else if(result.filename() == "..") {
                    result /= *it;
                }
                // Otherwise it should be safe to resolve the parent
                else {
                    result = result.parent_path();
                }
            }
            else if(*it == ".") {
                // Ignore
            }
            else {
                // Just cat other path entries
                result /= *it;
            }
        }
        p = result;
#endif
    }

    bool _ValidateFilename(const boost::filesystem::path& filename, const boost::filesystem::path& curdir)
    {
        if( !boost::filesystem::exists(filename) ) {
            return false;
        }

        if( _nDataAccessOptions & 1 ) {
            // check if filename is within _vBoostDataDirs
            boost::filesystem::path fullfilename = boost::filesystem::absolute(filename,curdir.empty() ? boost::filesystem::current_path() : curdir);
            _CustomNormalizePath(fullfilename);
            bool bfound = false;
            FOREACHC(itpath,_vBoostDataDirs) {
                boost::filesystem::path testfilename = fullfilename.parent_path();
                while( testfilename >= *itpath ) {
                    if( testfilename == *itpath ) {
                        bfound = true;
                        break;
                    }
                    testfilename = testfilename.parent_path();
                }
                if( bfound ) {
                    break;
                }
            }
            if( !bfound ) {
                return false;
            }
        }
        return true;
    }

#endif

private:
    static boost::shared_ptr<RaveGlobal> _state;
    // state that is always present

    // state that is initialized/destroyed
    boost::shared_ptr<RaveDatabase> _pdatabase;
    int _nDebugLevel;
    boost::mutex _mutexinternal;
    std::map<InterfaceType, READERSMAP > _mapreaders;
    std::map<InterfaceType,string> _mapinterfacenames;
    std::map<IkParameterizationType,string> _mapikparameterization, _mapikparameterizationlower;
    std::map<int, EnvironmentBase*> _mapenvironments;
    std::list<boost::function<void()> > _listDestroyCallbacks;
    std::string _homedirectory;
    std::vector<std::string> _vdbdirectories;
    int _nGlobalEnvironmentId;
    SpaceSamplerBasePtr _pdefaultsampler;
#ifdef USE_CRLIBM
    long long _crlibm_fpu_state;
#endif
    int _nDataAccessOptions;

    std::vector<string> _vdatadirs;
#ifdef HAVE_BOOST_FILESYSTEM
    std::vector<boost::filesystem::path> _vBoostDataDirs; ///< \brief returns absolute filenames of the data
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

const std::map<IkParameterizationType,std::string>& RaveGetIkParameterizationMap(int alllowercase)
{
    return IkParameterization::GetIkParameterizationMap(alllowercase);
}

IkParameterizationType RaveGetIkTypeFromUniqueId(int uniqueid)
{
    return IkParameterization::GetIkTypeFromUniqueId(uniqueid);
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

void RaveAddCallbackForDestroy(const boost::function<void()>& fn)
{
    RaveGlobal::instance()->AddCallbackForDestroy(fn);
}

int RaveGetEnvironmentId(EnvironmentBaseConstPtr penv)
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

MultiControllerBasePtr RaveCreateMultiController(EnvironmentBasePtr env, const std::string& rawname)
{
    std::string name;
    if( rawname == "" ) {
        name = "genericmulticontroller";
    }
    else {
        name = rawname;
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    }
    // TODO remove hack once MultiController is a registered interface
    ControllerBasePtr pcontroller = RaveGlobal::instance()->GetDatabase()->CreateController(env, name);
    if( name == "genericmulticontroller" ) {
        return boost::static_pointer_cast<MultiControllerBase>(pcontroller);
    }
    // don't support anything else
    return MultiControllerBasePtr();
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

std::string RaveFindLocalFile(const std::string& filename, const std::string& curdir)
{
    return RaveGlobal::instance()->FindLocalFile(filename,curdir);
}

bool RaveInvertFileLookup(std::string& newfilename, const std::string& filename)
{
    return RaveGlobal::instance()->InvertFileLookup(newfilename,filename);
}

void RaveSetDataAccess(int options)
{
    RaveGlobal::instance()->SetDataAccess(options);
}

int RaveGetDataAccess()
{
    return RaveGlobal::instance()->GetDataAccess();
}

const char *RaveGetLocalizedTextForDomain(const std::string& domainname, const char *msgid)
{
#ifndef _WIN32
    if (_gettextDomainsInitialized.find(domainname) == _gettextDomainsInitialized.end())
    {
        bindtextdomain(domainname.c_str(), OPENRAVE_LOCALE_INSTALL_DIR);
        _gettextDomainsInitialized.insert(domainname);
    }
    return dgettext(domainname.c_str(), msgid);
#else
    return msgid;
#endif
}

const std::map<IkParameterizationType,std::string>& IkParameterization::GetIkParameterizationMap(int alllowercase)
{
    return RaveGlobal::instance()->GetIkParameterizationMap(alllowercase);
}

IkParameterizationType IkParameterization::GetIkTypeFromUniqueId(int uniqueid)
{
    uniqueid &= IKP_UniqueIdMask;
    FOREACHC(it, RaveGlobal::instance()->GetIkParameterizationMap()) {
        if( (it->first & (IKP_UniqueIdMask&~IKP_VelocityDataBit)) == (uniqueid&(IKP_UniqueIdMask&~IKP_VelocityDataBit)) ) {
            return static_cast<IkParameterizationType>(it->first|(uniqueid&IKP_VelocityDataBit));
        }
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_("no ik exists of unique id 0x%x"),uniqueid,ORE_InvalidArguments);
}

ConfigurationSpecification IkParameterization::GetConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation, const std::string& robotname, const std::string& manipname)
{
    ConfigurationSpecification spec;
    spec._vgroups.resize(1);
    spec._vgroups[0].offset = 0;
    spec._vgroups[0].dof = IkParameterization::GetNumberOfValues(iktype);
    spec._vgroups[0].name = str(boost::format("ikparam_values %d")%iktype);
    if( robotname.size() > 0 ) {
        spec._vgroups[0].name += robotname;
        spec._vgroups[0].name += " ";
        if( manipname.size() > 0 ) {
            spec._vgroups[0].name += manipname;
        }
    }
    spec._vgroups[0].interpolation = interpolation;

    // remove any trailing whitespace from missing robot or manipulator names
    boost::algorithm::trim(spec._vgroups[0].name);
    return spec;
}

std::ostream& operator<<(std::ostream& O, const IkParameterization &ikparam)
{
    int type = ikparam._type;
    BOOST_ASSERT( !(type & IKP_CustomDataBit) );
    if( ikparam._mapCustomData.size() > 0 ) {
        type |= IKP_CustomDataBit;
    }
    O << type << " ";
    switch(ikparam._type & ~IKP_VelocityDataBit) {
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
        RAY r = ikparam.GetRay4D();
        O << r.dir.x << " " << r.dir.y << " " << r.dir.z << " " << r.pos.x << " " << r.pos.y << " " << r.pos.z << " ";
        break;
    }
    case IKP_Lookat3D: {
        Vector v = ikparam.GetLookat3D();
        O << v.x << " " << v.y << " " << v.z << " ";
        break;
    }
    case IKP_TranslationDirection5D: {
        RAY r = ikparam.GetTranslationDirection5D();
        O << r.dir.x << " " << r.dir.y << " " << r.dir.z << " " << r.pos.x << " " << r.pos.y << " " << r.pos.z << " ";
        break;
    }
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
    case IKP_TranslationXAxisAngle4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationXAxisAngle4D();
        O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationYAxisAngle4D();
        O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationZAxisAngle4D();
        O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationXAxisAngleZNorm4D();
        O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationYAxisAngleXNorm4D();
        O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        std::pair<Vector,dReal> p = ikparam.GetTranslationZAxisAngleYNorm4D();
        O << p.second << " " << p.first.x << " " << p.first.y << " " << p.first.z << " ";
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_("does not support parameterization 0x%x"), ikparam.GetType(),ORE_InvalidArguments);
    }
    if( ikparam._mapCustomData.size() > 0 ) {
        O << ikparam._mapCustomData.size() << " ";
        FOREACHC(it, ikparam._mapCustomData) {
            O << it->first << " " << it->second.size() << " ";
            FOREACHC(itvalue, it->second) {
                O << *itvalue << " ";
            }
        }
    }
    return O;
}

std::istream& operator>>(std::istream& I, IkParameterization& ikparam)
{
    int type=IKP_None;
    I >> type;
    ikparam._type = static_cast<IkParameterizationType>(type&~IKP_CustomDataBit);
    switch(ikparam._type) {
    case IKP_Transform6D: {
        Transform t; I >> t;
        ikparam.SetTransform6D(t);
        break;
    }
    case IKP_Transform6DVelocity:
        I >> ikparam._transform;
        break;
    case IKP_Rotation3D: { Vector v; I >> v; ikparam.SetRotation3D(v); break; }
    case IKP_Rotation3DVelocity:
        I >> ikparam._transform.rot;
        break;
    case IKP_Translation3D: {
        Vector v;
        I >> v.x >> v.y >> v.z;
        ikparam.SetTranslation3D(v);
        break;
    }
    case IKP_Lookat3DVelocity:
    case IKP_Translation3DVelocity:
    case IKP_TranslationXYOrientation3DVelocity:
        I >> ikparam._transform.trans.x >> ikparam._transform.trans.y >> ikparam._transform.trans.z;
        break;
    case IKP_Direction3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetDirection3D(v); break; }
    case IKP_Direction3DVelocity:
        I >> ikparam._transform.rot.x >> ikparam._transform.rot.y >> ikparam._transform.rot.z;
        break;
    case IKP_Ray4D: { RAY r; I >> r.dir.x >> r.dir.y >> r.dir.z >> r.pos.x >> r.pos.y >> r.pos.z; ikparam.SetRay4D(r); break; }
    case IKP_Ray4DVelocity:
    case IKP_TranslationDirection5DVelocity:
        I >> ikparam._transform.trans.x >> ikparam._transform.trans.y >> ikparam._transform.trans.z >> ikparam._transform.rot.x >> ikparam._transform.rot.y >> ikparam._transform.rot.z;
        break;
    case IKP_Lookat3D: { Vector v; I >> v.x >> v.y >> v.z; ikparam.SetLookat3D(v); break; }
    case IKP_TranslationDirection5D: { RAY r; I >> r.dir.x >> r.dir.y >> r.dir.z >> r.pos.x >> r.pos.y >> r.pos.z; ikparam.SetTranslationDirection5D(r); break; }
    case IKP_TranslationXY2D: { Vector v; I >> v.y >> v.y; ikparam.SetTranslationXY2D(v); break; }
    case IKP_TranslationXY2DVelocity:
        I >> ikparam._transform.trans.x >> ikparam._transform.trans.y;
        break;
    case IKP_TranslationXYOrientation3D: { Vector v; I >> v.y >> v.y >> v.z; ikparam.SetTranslationXYOrientation3D(v); break; }
    case IKP_TranslationLocalGlobal6D: { Vector localtrans, trans; I >> localtrans.x >> localtrans.y >> localtrans.z >> trans.x >> trans.y >> trans.z; ikparam.SetTranslationLocalGlobal6D(localtrans,trans); break; }
    case IKP_TranslationLocalGlobal6DVelocity:
        I >> ikparam._transform.rot.x >> ikparam._transform.rot.y >> ikparam._transform.rot.z >> ikparam._transform.trans.x >> ikparam._transform.trans.y >> ikparam._transform.trans.z;
        break;
    case IKP_TranslationXAxisAngle4D: {
        Vector trans; dReal angle=0;
        I >> angle >> trans.x >> trans.y >> trans.z;
        ikparam.SetTranslationXAxisAngle4D(trans,angle);
        break;
    }
    case IKP_TranslationYAxisAngle4D: {
        Vector trans; dReal angle=0;
        I >> angle >> trans.x >> trans.y >> trans.z;
        ikparam.SetTranslationYAxisAngle4D(trans,angle);
        break;
    }
    case IKP_TranslationZAxisAngle4D: {
        Vector trans; dReal angle=0;
        I >> angle >> trans.x >> trans.y >> trans.z;
        ikparam.SetTranslationZAxisAngle4D(trans,angle);
        break;
    }
    case IKP_TranslationXAxisAngleZNorm4D: {
        Vector trans; dReal angle=0;
        I >> angle >> trans.x >> trans.y >> trans.z;
        ikparam.SetTranslationXAxisAngleZNorm4D(trans,angle);
        break;
    }
    case IKP_TranslationYAxisAngleXNorm4D: {
        Vector trans; dReal angle=0;
        I >> angle >> trans.x >> trans.y >> trans.z;
        ikparam.SetTranslationYAxisAngleXNorm4D(trans,angle);
        break;
    }
    case IKP_TranslationZAxisAngleYNorm4D: {
        Vector trans; dReal angle=0;
        I >> angle >> trans.x >> trans.y >> trans.z;
        ikparam.SetTranslationZAxisAngleYNorm4D(trans,angle);
        break;
    }
    case IKP_TranslationXAxisAngle4DVelocity:
    case IKP_TranslationYAxisAngle4DVelocity:
    case IKP_TranslationZAxisAngle4DVelocity:
    case IKP_TranslationXAxisAngleZNorm4DVelocity:
    case IKP_TranslationYAxisAngleXNorm4DVelocity:
    case IKP_TranslationZAxisAngleYNorm4DVelocity:
        I >> ikparam._transform.rot.x >> ikparam._transform.trans.x >> ikparam._transform.trans.y >> ikparam._transform.trans.z;
        break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_("does not support parameterization 0x%x"), ikparam.GetType(),ORE_InvalidArguments);
    }
    ikparam._mapCustomData.clear();
    if( type & IKP_CustomDataBit ) {
        size_t numcustom = 0, numvalues=0;
        std::string name;
        I >> numcustom;
        if( !I ) {
            return I;
        }
        for(size_t i = 0; i < numcustom; ++i) {
            I >> name >> numvalues;
            if( !I ) {
                return I;
            }
            std::vector<dReal>& v = ikparam._mapCustomData[name];
            v.resize(numvalues);
            for(size_t j = 0; j < v.size(); ++j) {
                I >> v[j];
            }
        }
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
    throw OPENRAVE_EXCEPTION_FORMAT(_("unspecified dof 0x%x, 0x%x"),affinedofs%dof,ORE_InvalidArguments);
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
    throw OPENRAVE_EXCEPTION_FORMAT(_("requested index out of bounds %d (affinemask=0x%x)"),requestedindex%affinedofs, ORE_InvalidArguments);
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
        *itvalues++ = -normalizeAxisRotation(vActvAffineRotationAxis, t.rot).first;
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

void RaveGetAffineDOFValuesFromVelocity(std::vector<dReal>::iterator itvalues, const Vector& linearvel, const Vector& angularvel, const Vector& quatrotation, int affinedofs, const Vector& axis)
{
    if( affinedofs & DOF_X ) {
        *itvalues++ = linearvel.x;
    }
    if( affinedofs & DOF_Y ) {
        *itvalues++ = linearvel.y;
    }
    if( affinedofs & DOF_Z ) {
        *itvalues++ = linearvel.z;
    }
    if( affinedofs & DOF_RotationAxis ) {
        *itvalues++ = axis.dot3(angularvel);
    }
    else if( affinedofs & DOF_Rotation3D ) {
        *itvalues++ = angularvel.x;
        *itvalues++ = angularvel.y;
        *itvalues++ = angularvel.z;
    }
    else if( affinedofs & DOF_RotationQuat ) {
        Vector qdot = 0.5 * quatMultiply(Vector(0,angularvel.x,angularvel.y,angularvel.z), quatrotation);
        *itvalues++ = qdot.x;
        *itvalues++ = qdot.y;
        *itvalues++ = qdot.z;
        *itvalues++ = qdot.w;
    }
}

void RaveGetTransformFromAffineDOFValues(Transform& t, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& vActvAffineRotationAxis, bool normalize)
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
        if( normalize ) {
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
        else {
            // normalization turned off, but affine dofs have only DOF_Rotation3D. In order to convert this to quaternion velocity
            // will need current quaternion value (qrot), which is unknown, so assume identity
            // qvel = [0,axisangle] * qrot * 0.5
            t.rot[0] = 0;
            t.rot[1] = 0.5 * *itvalues++;
            t.rot[2] = 0.5 * *itvalues++;
            t.rot[3] = 0.5 * *itvalues++;
        }
    }
    else if( affinedofs & DOF_RotationQuat ) {
        // have to normalize since user might not be aware of this particular parameterization of rotations
        t.rot.x = *itvalues++;
        t.rot.y = *itvalues++;
        t.rot.z = *itvalues++;
        t.rot.w = *itvalues++;
        if( normalize ) {
            t.rot.normalize4();
        }
    }
}

void RaveGetVelocityFromAffineDOFVelocities(Vector& linearvel, Vector& angularvel, std::vector<dReal>::const_iterator itvalues, int affinedofs, const Vector& axis, const Vector& quatrotation)
{
    if( affinedofs & DOF_X ) {
        linearvel.x = *itvalues++;
    }
    if( affinedofs & DOF_Y ) {
        linearvel.y = *itvalues++;
    }
    if( affinedofs & DOF_Z ) {
        linearvel.z = *itvalues++;
    }
    if( affinedofs & DOF_RotationAxis ) {
        dReal angle = *itvalues++;
        angularvel = axis*angle;
    }
    else if( affinedofs & DOF_Rotation3D ) {
        angularvel.x = *itvalues++;
        angularvel.y = *itvalues++;
        angularvel.z = *itvalues++;
    }
    else if( affinedofs & DOF_RotationQuat ) {
        // have to normalize since user might not be aware of this particular parameterization of rotations
        Vector q;
        q.x = *itvalues++;
        q.y = *itvalues++;
        q.z = *itvalues++;
        q.w = *itvalues++;
        Vector angularvel2 = quatMultiply(q, quatInverse(quatrotation)); // qdot = 0.5 * angularvel * q
        angularvel.x = angularvel2.y * 2;
        angularvel.y = angularvel2.z * 2;
        angularvel.z = angularvel2.w * 2;
    }
}

void CollisionReport::Reset(int coloptions)
{
    options = coloptions;
    if( !(nKeepPrevious & 1) ) {
        minDistance = 1e20f;
        numWithinTol = 0;
        contacts.resize(0);
        vLinkColliding.resize(0);
        plink1.reset();
        plink2.reset();
    }
}

std::string CollisionReport::__str__() const
{
    stringstream s;
    if( vLinkColliding.size() > 0 ) {
        s << "pairs=" << vLinkColliding.size();
        int index = 0;
        FOREACH(itlinkpair, vLinkColliding) {
            s << ", [" << index << "](";
            if( !!itlinkpair->first ) {
                s << itlinkpair->first->GetParent()->GetName() << ":" << itlinkpair->first->GetName();
            }
            s << ")x(";
            if( !!itlinkpair->second ) {
                s << itlinkpair->second->GetParent()->GetName() << ":" << itlinkpair->second->GetName();
            }
            s << ") ";
            ++index;
        }
    }
    else {
        s << "(";
        if( !!plink1 ) {
            s << plink1->GetParent()->GetName() << ":" << plink1->GetName();
        }
        s << ")x(";
        if( !!plink2 ) {
            s << plink2->GetParent()->GetName() << ":" << plink2->GetName();
        }
        s << ")";
    }
    s << ", contacts="<<contacts.size();
    if( minDistance < 1e10 ) {
        s << ", mindist="<<minDistance;
    }
    return s.str();
}

bool PhysicsEngineBase::GetLinkForceTorque(KinBody::LinkConstPtr plink, Vector& force, Vector& torque)
{
    force = Vector(0,0,0);
    torque = Vector(0,0,0);
    //Loop over all joints in the parent body
    FOREACHC(itjoint,plink->GetParent()->GetJoints()) {
        //if this joint's parent or child body is the current body
        bool bParentLink=(*itjoint)->GetHierarchyParentLink() == plink;
        bool bChildLink=(*itjoint)->GetHierarchyChildLink() == plink;
        if( bParentLink || bChildLink ) {
            Vector f;
            Vector T;
            GetJointForceTorque(*itjoint, f,T);
            if( bParentLink ) {
                Vector r = (*itjoint)->GetAnchor()-plink->GetGlobalCOM();
                force += f;
                torque += T;
                //Re-add moment due to equivalent load at body COM
                torque += r.cross(f);
            }
            else {
                //Equal but opposite sign
                Vector r = (*itjoint)->GetAnchor()-plink->GetGlobalCOM();
                force -= f;
                torque -= T;
                torque -= r.cross(f);
            }
        }
    }
    FOREACHC(itjoint,plink->GetParent()->GetPassiveJoints()) {
        bool bParentLink=(*itjoint)->GetHierarchyParentLink() == plink;
        bool bChildLink=(*itjoint)->GetHierarchyChildLink() == plink;
        if( bParentLink || bChildLink ) {
            Vector f;
            Vector T;
            GetJointForceTorque(*itjoint, f,T);

            if( bParentLink ) {
                Vector r = (*itjoint)->GetAnchor()-plink->GetGlobalCOM();
                force += f;
                torque += T;
                //Re-add moment due to equivalent load at body COM
                torque += r.cross(f);
            }
            else {
                //Equal but opposite sign
                Vector r = (*itjoint)->GetAnchor()-plink->GetGlobalCOM();
                force -= f;
                torque -= T;
                torque -= r.cross(f);
            }
        }
    }
    return true;
}

void TriMesh::ApplyTransform(const Transform& t)
{
    FOREACH(it, vertices) {
        *it = t * *it;
    }
}

void TriMesh::ApplyTransform(const TransformMatrix& t)
{
    FOREACH(it, vertices) {
        *it = t * *it;
    }
}

void TriMesh::Append(const TriMesh& mesh)
{
    int offset = (int)vertices.size();
    vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
    if( indices.capacity() < indices.size()+mesh.indices.size() ) {
        indices.reserve(indices.size()+mesh.indices.size());
    }
    FOREACHC(it, mesh.indices) {
        indices.push_back(*it+offset);
    }
}

void TriMesh::Append(const TriMesh& mesh, const Transform& trans)
{
    int offset = (int)vertices.size();
    vertices.resize(vertices.size() + mesh.vertices.size());
    for(size_t i = 0; i < mesh.vertices.size(); ++i) {
        vertices[i+offset] = trans * mesh.vertices[i];
    }
    if( indices.capacity() < indices.size()+mesh.indices.size() ) {
        indices.reserve(indices.size()+mesh.indices.size());
    }
    FOREACHC(it, mesh.indices) {
        indices.push_back(*it+offset);
    }
}

AABB TriMesh::ComputeAABB() const
{
    AABB ab;
    if( vertices.size() == 0 ) {
        return ab;
    }
    Vector vmin, vmax;
    vmin = vmax = vertices.at(0);
    FOREACHC(itv, vertices) {
        Vector v = *itv;
        if( vmin.x > v.x ) {
            vmin.x = v.x;
        }
        if( vmin.y > v.y ) {
            vmin.y = v.y;
        }
        if( vmin.z > v.z ) {
            vmin.z = v.z;
        }
        if( vmax.x < v.x ) {
            vmax.x = v.x;
        }
        if( vmax.y < v.y ) {
            vmax.y = v.y;
        }
        if( vmax.z < v.z ) {
            vmax.z = v.z;
        }
    }

    ab.extents = (dReal)0.5*(vmax-vmin);
    ab.pos = (dReal)0.5*(vmax+vmin);
    return ab;
}

void TriMesh::serialize(std::ostream& o, int options) const
{
    o << vertices.size() << " ";
    FOREACHC(it,vertices) {
        SerializeRound3(o, *it);
    }
    o << indices.size() << " ";
    FOREACHC(it,indices) {
        o << *it << " ";
    }
}


void Grabbed::_ProcessCollidingLinks(const std::set<int>& setRobotLinksToIgnore)
{
    _setRobotLinksToIgnore = setRobotLinksToIgnore;
    _listNonCollidingLinks.clear();
    _mapLinkIsNonColliding.clear();
    KinBodyPtr pgrabbedbody(_pgrabbedbody);
    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(_plinkrobot->GetParent());
    EnvironmentBasePtr penv = probot->GetEnv();
    CollisionCheckerBasePtr pchecker = probot->GetSelfCollisionChecker();
    if( !pchecker ) {
        pchecker = penv->GetCollisionChecker();
    }
    CollisionOptionsStateSaver colsaver(pchecker,0); // have to reset the collision options
    {
        // have to enable all the links in order to compute accurate _mapLinkIsNonColliding info
        KinBody::KinBodyStateSaver grabbedbodysaver(pgrabbedbody, KinBody::Save_LinkEnable);
        pgrabbedbody->Enable(true);
        KinBody::KinBodyStateSaver robotsaver(probot, KinBody::Save_LinkEnable);
        probot->Enable(true);

        //uint64_t starttime = utils::GetMicroTime();

        // check collision with all links to see which are valid
        int numchecked = 0;
        FOREACHC(itlink, probot->GetLinks()) {
            int noncolliding = 0;
            if( find(_vattachedlinks.begin(),_vattachedlinks.end(), *itlink) == _vattachedlinks.end() ) {
                if( setRobotLinksToIgnore.find((*itlink)->GetIndex()) == setRobotLinksToIgnore.end() ) {
                    ++numchecked;
                    //uint64_t localstarttime = utils::GetMicroTime();
                    if( !pchecker->CheckCollision(KinBody::LinkConstPtr(*itlink), pgrabbedbody) ) {
                        noncolliding = 1;
                    }
                    //RAVELOG_DEBUG_FORMAT("check %s col %s %s %fs", pchecker->GetXMLId()%(*itlink)->GetName()%pgrabbedbody->GetName()%(1e-6*(utils::GetMicroTime()-localstarttime)));
                }
            }
            _mapLinkIsNonColliding[*itlink] = noncolliding;
        }

        //uint64_t starttime1 = utils::GetMicroTime();
        
        std::vector<KinBody::LinkPtr > vbodyattachedlinks;
        FOREACHC(itgrabbed, probot->_vGrabbedBodies) {
            boost::shared_ptr<Grabbed const> pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
            bool bsamelink = find(_vattachedlinks.begin(),_vattachedlinks.end(), pgrabbed->_plinkrobot) != _vattachedlinks.end();
            KinBodyPtr pothergrabbedbody(pgrabbed->_pgrabbedbody);
            if( bsamelink ) {
                pothergrabbedbody->GetLinks().at(0)->GetRigidlyAttachedLinks(vbodyattachedlinks);
            }
            if( pothergrabbedbody != pgrabbedbody ) {
                KinBody::KinBodyStateSaver othergrabbedbodysaver(pothergrabbedbody, KinBody::Save_LinkEnable);
                pothergrabbedbody->Enable(true);
                FOREACHC(itlink, pothergrabbedbody->GetLinks()) {
                    int noncolliding = 0;
                    if( bsamelink && find(vbodyattachedlinks.begin(),vbodyattachedlinks.end(), *itlink) != vbodyattachedlinks.end() ) {
                    }
                    else if( !pchecker->CheckCollision(KinBody::LinkConstPtr(*itlink), pgrabbedbody) ) {
                        noncolliding = 1;
                    }
                    _mapLinkIsNonColliding[*itlink] = noncolliding;
                }
            }
        }

        //uint64_t starttime2 = utils::GetMicroTime();
        //RAVELOG_DEBUG_FORMAT("env=%d, process links %d %fs %fs", probot->GetEnv()->GetId()%numchecked%(1e-6*(starttime1-starttime))%(1e-6*(starttime2-starttime)));
    }

    if( pgrabbedbody->IsEnabled() ) {
        FOREACH(itnoncolliding, _mapLinkIsNonColliding) {
            if( itnoncolliding->second && itnoncolliding->first->IsEnabled() ) {
                //RAVELOG_VERBOSE(str(boost::format("non-colliding link %s for grabbed body %s")%(*itlink)->GetName()%pgrabbedbody->GetName()));
                _listNonCollidingLinks.push_back(itnoncolliding->first);
            }
        }
    }
}

std::ostream& operator<<(std::ostream& O, const TriMesh& trimesh)
{
    trimesh.serialize(O,0);
    return O;
}

std::istream& operator>>(std::istream& I, TriMesh& trimesh)
{
    trimesh.vertices.resize(0);
    trimesh.indices.resize(0);
    size_t s=0;
    I >> s;
    if( !I ) {
        return I;
    }
    trimesh.vertices.resize(s);
    FOREACH(it,trimesh.vertices) {
        I >> it->x >> it->y >> it->z;
    }
    I >> s;
    if( !I ) {
        return I;
    }
    trimesh.indices.resize(s);
    FOREACH(it,trimesh.indices) {
        I >> *it;
    }
    return I;
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
        if( _pcurreader->startElement(name, atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }

    if( !!_osrecord ) {
        *_osrecord << "<" << name << " ";
        FOREACHC(itatt, atts) {
            *_osrecord << itatt->first << "=\"" << itatt->second << "\" ";
        }
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
            if( !!_osrecord ) {
                *_osrecord << "</" << name << ">" << endl;
            }
        }
        return false;
    }

    if( name == _fieldname ) {
        return true;
    }
    RAVELOG_ERROR(str(boost::format("invalid xml tag %s\n")%name));
    return false;
}

void DummyXMLReader::characters(const std::string& ch)
{
    if( !_pcurreader ) {
        if( !!_osrecord ) {
            *_osrecord << ch;
        }
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

void SensorBase::SensorGeometry::Serialize(BaseXMLWriterPtr writer, int options) const
{
    AttributesList atts;
    if( hardware_id.size() > 0 ) {
        writer->AddChild("hardware_id",atts)->SetCharData(hardware_id);
    }
}

void SensorBase::CameraGeomData::Serialize(BaseXMLWriterPtr writer, int options) const
{
    SensorGeometry::Serialize(writer, options);
    AttributesList atts;
    stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
    ss << KK.fx << " 0 " << KK.cx << " 0 " << KK.fy << " " << KK.cy;
    writer->AddChild("intrinsic",atts)->SetCharData(ss.str());
    ss.str("");
    ss << KK.focal_length;
    writer->AddChild("focal_length",atts)->SetCharData(ss.str());
    if( KK.distortion_model.size() > 0 ) {
        writer->AddChild("distortion_model",atts)->SetCharData(KK.distortion_model);
        if( KK.distortion_coeffs.size() > 0 ) {
            ss.str("");
            FOREACHC(it, KK.distortion_coeffs) {
                ss << *it << " ";
            }
            writer->AddChild("distortion_coeffs",atts)->SetCharData(ss.str());
        }
    }
    ss.str("");
    ss << width << " " << height; // _numchannels=3
    writer->AddChild("image_dimensions",atts)->SetCharData(ss.str());
    writer->AddChild("measurement_time",atts)->SetCharData(boost::lexical_cast<std::string>(measurement_time));
    writer->AddChild("gain",atts)->SetCharData(boost::lexical_cast<std::string>(gain));
    //writer->AddChild("format",atts)->SetCharData(_channelformat.size() > 0 ? _channelformat : std::string("uint8"));
    if( sensor_reference.size() > 0 ) {
        atts.push_back(std::make_pair("url", sensor_reference));
        writer->AddChild("sensor_reference",atts);
        atts.clear();
    }
    if( target_region.size() > 0 ) {
        atts.push_back(std::make_pair("url", target_region));
        writer->AddChild("target_region",atts);
        atts.clear();
    }
}

void SensorBase::Serialize(BaseXMLWriterPtr writer, int options) const
{
    RAVELOG_WARN(str(boost::format("sensor %s does not implement Serialize")%GetXMLId()));
}

class CustomSamplerCallbackData : public boost::enable_shared_from_this<CustomSamplerCallbackData>, public UserData
{
public:
    CustomSamplerCallbackData(const SpaceSamplerBase::StatusCallbackFn& callbackfn, SpaceSamplerBasePtr sampler) : _callbackfn(callbackfn), _samplerweak(sampler) {
    }
    virtual ~CustomSamplerCallbackData() {
        SpaceSamplerBasePtr sampler = _samplerweak.lock();
        if( !!sampler ) {
            sampler->__listRegisteredCallbacks.erase(_iterator);
        }
    }

    SpaceSamplerBase::StatusCallbackFn _callbackfn;
    SpaceSamplerBaseWeakPtr _samplerweak;
    std::list<UserDataWeakPtr>::iterator _iterator;
};

typedef boost::shared_ptr<CustomSamplerCallbackData> CustomSamplerCallbackDataPtr;

UserDataPtr SpaceSamplerBase::RegisterStatusCallback(const StatusCallbackFn& callbackfn)
{
    CustomSamplerCallbackDataPtr pdata(new CustomSamplerCallbackData(callbackfn,shared_sampler()));
    pdata->_iterator = __listRegisteredCallbacks.insert(__listRegisteredCallbacks.end(),pdata);
    return pdata;
}

int SpaceSamplerBase::_CallStatusFunctions(int sampleiteration)
{
    int ret = 0;
    FOREACHC(it,__listRegisteredCallbacks) {
        CustomSamplerCallbackDataPtr pitdata = boost::dynamic_pointer_cast<CustomSamplerCallbackData>(it->lock());
        if( !!pitdata) {
            ret |= pitdata->_callbackfn(sampleiteration);
        }
    }
    return ret;
}

CollisionOptionsStateSaver::CollisionOptionsStateSaver(CollisionCheckerBasePtr p, int newoptions, bool required)
{
    _oldoptions = p->GetCollisionOptions();
    _p = p;
    if( !_p->SetCollisionOptions(newoptions) ) {
        if( required ) {
            throw openrave_exception(str(boost::format(_("Failed to set collision options %d in checker %s\n"))%newoptions%_p->GetXMLId()));
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
    return float(sample.at(0));
}

double RaveRandomDouble(IntervalType interval)
{
    std::vector<dReal> sample;
    RaveGlobal::instance()->GetDefaultSampler()->SampleSequence(sample,1,interval);
    return double(sample.at(0));
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
        BaseXMLReader::ProcessElement pestatus = pdata->_preader->startElement(s, listatts);
        if( pestatus != BaseXMLReader::PE_Support ) {
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
    if (ctxt->sax &&  ctxt->sax->initialized == XML_SAX2_MAGIC && (ctxt->sax->startElementNs != NULL || ctxt->sax->endElementNs != NULL)) {
        ctxt->sax2 = 1;
    }
#else
    ctxt->sax2 = 1;
#endif

    ctxt->str_xml = xmlDictLookup(ctxt->dict, BAD_CAST "xml", 3);
    ctxt->str_xmlns = xmlDictLookup(ctxt->dict, BAD_CAST "xmlns", 5);
    ctxt->str_xml_ns = xmlDictLookup(ctxt->dict, XML_XML_NAMESPACE, 36);
    if ( ctxt->str_xml==NULL || ctxt->str_xmlns==NULL || ctxt->str_xml_ns == NULL) {
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
