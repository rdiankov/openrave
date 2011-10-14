// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef RAVE_PLUGIN_DATABASE_H
#define RAVE_PLUGIN_DATABASE_H

#include <errno.h>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem.hpp>
#endif
#include <boost/version.hpp>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define PLUGIN_EXT ".dll"
#define OPENRAVE_LAZY_LOADING false
#else
#define OPENRAVE_LAZY_LOADING true
#include <dlfcn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#ifdef __APPLE_CC__
#define PLUGIN_EXT ".dylib"
#else
#define PLUGIN_EXT ".so"
#endif

#endif

#define INTERFACE_DELETER boost::bind(&RaveDatabase::_InterfaceDestroyCallbackShared,shared_from_this(),_1)

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

#ifdef _WIN32
const char s_filesep = '\\';
#else
const char s_filesep = '/';
#endif

namespace OpenRAVE {

/// \brief database of interfaces from plugins
class RaveDatabase : public boost::enable_shared_from_this<RaveDatabase>
{
    struct RegisteredInterface : public UserData
    {
        RegisteredInterface(InterfaceType type, const std::string& name, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn, boost::shared_ptr<RaveDatabase> database) : _type(type), _name(name), _createfn(createfn), _database(database) {
        }
        virtual ~RegisteredInterface() {
            boost::shared_ptr<RaveDatabase> database = _database.lock();
            if( !!database ) {
                boost::mutex::scoped_lock lock(database->_mutex);
                database->_listRegisteredInterfaces.erase(_iterator);
            }
        }

        InterfaceType _type;
        std::string _name;
        boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)> _createfn;
        std::list< boost::weak_ptr<RegisteredInterface> >::iterator _iterator;
protected:
        boost::weak_ptr<RaveDatabase> _database;
    };
    typedef boost::shared_ptr<RegisteredInterface> RegisteredInterfacePtr;

public:
    class Plugin : public boost::enable_shared_from_this<Plugin>
    {
public:
        Plugin(boost::shared_ptr<RaveDatabase> pdatabase) : _pdatabase(pdatabase), plibrary(NULL), pfnCreate(NULL), pfnCreateNew(NULL), pfnGetPluginAttributes(NULL), pfnGetPluginAttributesNew(NULL), pfnDestroyPlugin(NULL), _bShutdown(false) {
        }
        virtual ~Plugin() {
            Destroy();
        }

        virtual void Destroy() {
            if( plibrary ) {
                Load_DestroyPlugin();
            }
            boost::mutex::scoped_lock lock(_mutex);
            // do some more checking here, there still might be instances of robots, planners, and sensors out there
            if (plibrary) {
                RAVELOG_DEBUG("RaveDatabase: closing plugin %s\n", ppluginname.c_str());        // Sleep(10);
                if( pfnDestroyPlugin != NULL ) {
                    pfnDestroyPlugin();
                }
                boost::shared_ptr<RaveDatabase> pdatabase = _pdatabase.lock();
                if( !!pdatabase ) {
                    pdatabase->_QueueLibraryDestruction(plibrary);
                }
                plibrary = NULL;
            }
            pfnCreate = NULL;
            pfnCreateNew = NULL;
            pfnDestroyPlugin = NULL;
            pfnGetPluginAttributes = NULL;
            pfnGetPluginAttributesNew = NULL;
            _bShutdown = true;
        }

        virtual bool IsValid() {
            return !_bShutdown;
        }

        const string& GetName() const {
            return ppluginname;
        }
        bool GetInfo(PLUGININFO& info) {
            info = _infocached;
            return true;
        }

        virtual bool Load_CreateInterfaceGlobal()
        {
            _confirmLibrary();
            if(( pfnCreateNew == NULL) &&( pfnCreate == NULL) ) {
                if( pfnCreateNew == NULL ) {
                    pfnCreateNew = (PluginExportFn_OpenRAVECreateInterface)_SysLoadSym(plibrary, "OpenRAVECreateInterface");
                }

                if( pfnCreateNew == NULL ) {
#ifdef _MSC_VER
                    pfnCreate = (PluginExportFn_CreateInterface)_SysLoadSym(plibrary, "?CreateInterface@@YA?AV?$shared_ptr@VInterfaceBase@OpenRAVE@@@boost@@W4InterfaceType@OpenRAVE@@ABV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@PBDV?$shared_ptr@VEnvironmentBase@OpenRAVE@@@2@@Z");
#else
                    pfnCreate = (PluginExportFn_CreateInterface)_SysLoadSym(plibrary, "_Z15CreateInterfaceN8OpenRAVE10InterfaceTypeERKSsPKcN5boost10shared_ptrINS_15EnvironmentBaseEEE");
#endif
                    if( pfnCreate == NULL ) {
                        pfnCreate = (PluginExportFn_CreateInterface)_SysLoadSym(plibrary, "CreateInterface");
                        if( pfnCreate == NULL ) {
                            return false;
                        }
                    }
                }
            }
            return pfnCreateNew!=NULL||pfnCreate!=NULL;
        }

        virtual bool Load_GetPluginAttributes()
        {
            _confirmLibrary();
            if(( pfnGetPluginAttributesNew == NULL) ||( pfnGetPluginAttributes == NULL) ) {
                if( pfnGetPluginAttributesNew == NULL ) {
                    pfnGetPluginAttributesNew = (PluginExportFn_OpenRAVEGetPluginAttributes)_SysLoadSym(plibrary,"OpenRAVEGetPluginAttributes");
                }
                if( pfnGetPluginAttributesNew == NULL ) {
#ifdef _MSC_VER
                    pfnGetPluginAttributes = (PluginExportFn_GetPluginAttributes)_SysLoadSym(plibrary, "?GetPluginAttributes@@YA_NPAUPLUGININFO@OpenRAVE@@H@Z");
#else
                    pfnGetPluginAttributes = (PluginExportFn_GetPluginAttributes)_SysLoadSym(plibrary, "_Z19GetPluginAttributesPN8OpenRAVE10PLUGININFOEi");
#endif
                    if( !pfnGetPluginAttributes ) {
                        pfnGetPluginAttributes = (PluginExportFn_GetPluginAttributes)_SysLoadSym(plibrary, "GetPluginAttributes");
                        if( !pfnGetPluginAttributes ) {
                            return false;
                        }
                    }
                }
            }
            return pfnGetPluginAttributesNew!=NULL||pfnGetPluginAttributes!=NULL;
        }

        virtual bool Load_DestroyPlugin()
        {
            _confirmLibrary();
            if( pfnDestroyPlugin == NULL ) {
#ifdef _MSC_VER
                pfnDestroyPlugin = (PluginExportFn_DestroyPlugin)_SysLoadSym(plibrary, "?DestroyPlugin@@YAXXZ");
#else
                pfnDestroyPlugin = (PluginExportFn_DestroyPlugin)_SysLoadSym(plibrary, "_Z13DestroyPluginv");
#endif
                if( pfnDestroyPlugin == NULL ) {
                    pfnDestroyPlugin = (PluginExportFn_DestroyPlugin)_SysLoadSym(plibrary, "DestroyPlugin");
                    if( pfnDestroyPlugin == NULL ) {
                        RAVELOG_WARN(str(boost::format("%s: can't load DestroyPlugin function, passing...\n")%ppluginname));
                        return false;
                    }
                }
            }
            return pfnDestroyPlugin!=NULL;
        }

        bool HasInterface(InterfaceType type, const string& name)
        {
            if( name.size() == 0 ) {
                return false;
            }
            std::map<InterfaceType, std::vector<std::string> >::iterator itregisterednames = _infocached.interfacenames.find(type);
            if( itregisterednames == _infocached.interfacenames.end() ) {
                return false;
            }
            FOREACH(it,itregisterednames->second) {
                if(( name.size() >= it->size()) &&( strnicmp(name.c_str(),it->c_str(),it->size()) == 0) ) {
                    return true;
                }
            }
            return false;
        }

        InterfaceBasePtr CreateInterface(InterfaceType type, const std::string& name, const char* interfacehash, EnvironmentBasePtr penv) {
            pair< InterfaceType, string> p(type,tolowerstring(name));
            if( _setBadInterfaces.find(p) != _setBadInterfaces.end() ) {
                return InterfaceBasePtr();
            }

            if( !HasInterface(type,name) ) {
                return InterfaceBasePtr();
            }

            try {
                if( !Load_CreateInterfaceGlobal() ) {
                    throw openrave_exception(str(boost::format("%s: can't load CreateInterface function\n")%ppluginname),ORE_InvalidPlugin);
                }
                InterfaceBasePtr pinterface;
                if( pfnCreateNew != NULL ) {
                    pinterface = pfnCreateNew(type,name,interfacehash,OPENRAVE_ENVIRONMENT_HASH,penv);
                }
                else if( pfnCreate != NULL ) {
                    pinterface = pfnCreate(type,name,interfacehash,penv);
                }
                return pinterface;
            }
            catch(const openrave_exception& ex) {
                RAVELOG_ERROR(str(boost::format("Create Interface: openrave exception , plugin %s: %s\n")%ppluginname%ex.what()));
                if( ex.GetCode() == ORE_InvalidPlugin ) {
                    RAVELOG_DEBUG(str(boost::format("shared object %s is not a valid openrave plugin\n")%ppluginname));
                    Destroy();
                }
                else if( ex.GetCode() == ORE_InvalidInterfaceHash ) {
                    _setBadInterfaces.insert(p);
                }
            }
            catch(...) {
                RAVELOG_ERROR(str(boost::format("Create Interface: unknown exception, plugin %s\n")%ppluginname));
            }
            return InterfaceBasePtr();
        }

protected:
        /// if the library is not loaded yet, wait for it.
        void _confirmLibrary()
        {
            // first test the library before locking
            if( plibrary == NULL ) {
                boost::mutex::scoped_lock lock(_mutex);
                _pdatabase.lock()->_AddToLoader(shared_from_this());
                do {
                    if( plibrary ) {
                        return;
                    }
                    if( _bShutdown ) {
                        throw openrave_exception("library is shutting down",ORE_InvalidPlugin);
                    }
                    _cond.wait(lock);
                } while(1);
            }
        }

        boost::weak_ptr<RaveDatabase> _pdatabase;
        std::set<pair< InterfaceType, string> > _setBadInterfaces;         ///< interfaces whose hash is wrong and shouldn't be tried for this plugin
        string ppluginname;

        void* plibrary;         // loaded library (NULL if not loaded)
        PluginExportFn_CreateInterface pfnCreate;
        PluginExportFn_OpenRAVECreateInterface pfnCreateNew;
        PluginExportFn_GetPluginAttributes pfnGetPluginAttributes;
        PluginExportFn_OpenRAVEGetPluginAttributes pfnGetPluginAttributesNew;
        PluginExportFn_DestroyPlugin pfnDestroyPlugin;
        PLUGININFO _infocached;
        boost::mutex _mutex;         ///< locked when library is getting updated, only used when plibrary==NULL
        boost::condition _cond;
        bool _bShutdown;         ///< managed by plugin database

        friend class RaveDatabase;
    };
    typedef boost::shared_ptr<Plugin> PluginPtr;
    typedef boost::shared_ptr<Plugin const> PluginConstPtr;
    friend class Plugin;

    RaveDatabase() : _bShutdown(false) {
    }
    virtual ~RaveDatabase() {
        Destroy();
    }

    RobotBasePtr CreateRobot(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<RobotBase>(Create(penv, PT_Robot, name));
    }
    KinBodyPtr CreateKinBody(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<KinBody>(Create(penv, PT_KinBody, name));
    }
    PlannerBasePtr CreatePlanner(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<PlannerBase>(Create(penv, PT_Planner, name));
    }
    SensorSystemBasePtr CreateSensorSystem(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<SensorSystemBase>(Create(penv, PT_SensorSystem, name));
    }
    ControllerBasePtr CreateController(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<ControllerBase>(Create(penv, PT_Controller, name));
    }
    ModuleBasePtr CreateModule(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<ModuleBase>(Create(penv, PT_Module, name));
    }
    IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<IkSolverBase>(Create(penv, PT_IkSolver, name));
    }
    PhysicsEngineBasePtr CreatePhysicsEngine(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<PhysicsEngineBase>(Create(penv, PT_PhysicsEngine, name));
    }
    SensorBasePtr CreateSensor(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<SensorBase>(Create(penv, PT_Sensor, name));
    }
    CollisionCheckerBasePtr CreateCollisionChecker(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<CollisionCheckerBase>(Create(penv, PT_CollisionChecker, name));
    }
    ViewerBasePtr CreateViewer(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<ViewerBase>(Create(penv, PT_Viewer, name));
    }
    TrajectoryBasePtr CreateTrajectory(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<TrajectoryBase>(Create(penv, PT_Trajectory, name));
    }
    SpaceSamplerBasePtr CreateSpaceSampler(EnvironmentBasePtr penv, const std::string& name) {
        return RaveInterfaceCast<SpaceSamplerBase>(Create(penv, PT_SpaceSampler, name));
    }

    virtual bool Init(bool bLoadAllPlugins)
    {
        _threadPluginLoader.reset(new boost::thread(boost::bind(&RaveDatabase::_PluginLoaderThread, this)));
        std::vector<std::string> vplugindirs;
        RaveParseDirectories(getenv("OPENRAVE_PLUGINS"), vplugindirs);
        bool bExists=false;
        string installdir = OPENRAVE_PLUGINS_INSTALL_DIR;
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
                installdir += str(boost::format("%cshare%copenrave-%d.%d%cplugins")%s_filesep%s_filesep%OPENRAVE_VERSION_MAJOR%OPENRAVE_VERSION_MINOR%s_filesep);
                RAVELOG_VERBOSE(str(boost::format("window registry plugin dir '%s'")%installdir));
            }
            else
#endif
            {
                RAVELOG_WARN(str(boost::format("%s doesn't exist")%installdir));
            }
        }
        boost::filesystem::path pluginsfilename = boost::filesystem::system_complete(boost::filesystem::path(installdir));
        FOREACH(itname, vplugindirs) {
            if( pluginsfilename == boost::filesystem::system_complete(boost::filesystem::path(*itname)) ) {
                bExists = true;
                break;
            }
        }
#else
        string pluginsfilename=installdir;
        FOREACH(itname, vplugindirs) {
            if( pluginsfilename == *itname ) {
                bExists = true;
                break;
            }
        }
#endif
        if( !bExists ) {
            vplugindirs.push_back(installdir);
        }
        FOREACH(it, vplugindirs) {
            if( it->size() > 0 ) {
                _listplugindirs.push_back(*it);
                RAVELOG_VERBOSE(str(boost::format("plugin dir: %s")%*it));
            }
        }
        if( bLoadAllPlugins ) {
            FOREACH(it, vplugindirs) {
                if( it->size() > 0 ) {
                    AddDirectory(it->c_str());
                }
            }
        }
        return true;
    }

    /// Destroy all plugins and directories
    virtual void Destroy()
    {
        RAVELOG_DEBUG("plugin database shutting down...\n");
        {
            boost::mutex::scoped_lock lock(_mutexPluginLoader);
            _bShutdown = true;
            _condLoaderHasWork.notify_all();
        }
        if( !!_threadPluginLoader ) {
            _threadPluginLoader->join();
            _threadPluginLoader.reset();
        }
        {
            boost::mutex::scoped_lock lock(_mutex);
            _listplugins.clear();
        }
        // cannot lock mutex due to __erase_iterator
        _listRegisteredInterfaces.clear();
        {
            boost::mutex::scoped_lock lock(_mutex);
            _CleanupUnusedLibraries();
        }
        _listplugindirs.clear();
        RAVELOG_DEBUG("openrave plugin database destroyed\n");
    }

    void GetPlugins(std::list<PluginPtr>& listplugins) const
    {
        boost::mutex::scoped_lock lock(_mutex);
        listplugins = _listplugins;
    }

    InterfaceBasePtr Create(EnvironmentBasePtr penv, InterfaceType type, const std::string& _name)
    {
        std::string name=_name;
        InterfaceBasePtr pointer;
        if( name.size() == 0 ) {
            switch(type) {
            case PT_KinBody: {
                pointer.reset(new KinBody(PT_KinBody,penv));
                pointer->__strxmlid = "KinBody";
                break;
            }
            case PT_PhysicsEngine: name = "GenericPhysicsEngine"; break;
            case PT_CollisionChecker: name = "GenericCollisionChecker"; break;
            case PT_Robot: name = "GenericRobot"; break;
            case PT_Trajectory: name = "GenericTrajectory"; break;
            default: break;
            }
        }

        if( !pointer ) {
            size_t nInterfaceNameLength = name.find_first_of(' ');
            if( nInterfaceNameLength == string::npos ) {
                nInterfaceNameLength = name.size();
            }
            if( nInterfaceNameLength == 0 ) {
                RAVELOG_WARN(str(boost::format("interface name \"%s\" needs to start with a valid character\n")%name));
                return InterfaceBasePtr();
            }

            // have to copy in order to allow plugins to register stuff inside their creation methods
            std::list< boost::weak_ptr<RegisteredInterface> > listRegisteredInterfaces;
            list<PluginPtr> listplugins;
            {
                boost::mutex::scoped_lock lock(_mutex);
                listRegisteredInterfaces = _listRegisteredInterfaces;
                listplugins = _listplugins;
            }
            FOREACH(it, listRegisteredInterfaces) {
                RegisteredInterfacePtr registration = it->lock();
                if( !!registration ) {
                    if(( nInterfaceNameLength >= registration->_name.size()) &&( strnicmp(name.c_str(),registration->_name.c_str(),registration->_name.size()) == 0) ) {
                        std::stringstream sinput(name);
                        std::string interfacename;
                        sinput >> interfacename;
                        std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);
                        pointer = registration->_createfn(penv,sinput);
                        if( !!pointer ) {
                            if( pointer->GetInterfaceType() != type ) {
                                RAVELOG_FATAL(str(boost::format("plugin interface name %s, type %s, types do not match\n")%name%RaveGetInterfaceName(type)));
                                pointer.reset();
                            }
                            else {
                                pointer = InterfaceBasePtr(pointer.get(), smart_pointer_deleter<InterfaceBasePtr>(pointer,INTERFACE_DELETER));
                                pointer->__strpluginname = "__internal__";
                                pointer->__strxmlid = name;
                                //pointer->__plugin; // need to protect resources?
                                break;
                            }
                        }
                    }
                }
            }

            if( !pointer ) {
                const char* hash = RaveGetInterfaceHash(type);
                list<PluginPtr>::iterator itplugin = listplugins.begin();
                while(itplugin != listplugins.end()) {
                    pointer = (*itplugin)->CreateInterface(type, name, hash, penv);
                    if( !!pointer ) {
                        if( strcmp(pointer->GetHash(), hash) ) {
                            RAVELOG_FATAL(str(boost::format("plugin interface name %s, %s has invalid hash, might be compiled with stale openrave files\n")%name%RaveGetInterfaceName(type)));
                            (*itplugin)->_setBadInterfaces.insert(make_pair(type,tolowerstring(name)));
                            pointer.reset();
                        }
                        else if( pointer->GetInterfaceType() != type ) {
                            RAVELOG_FATAL(str(boost::format("plugin interface name %s, type %s, types do not match\n")%name%RaveGetInterfaceName(type)));
                            (*itplugin)->_setBadInterfaces.insert(make_pair(type,tolowerstring(name)));
                            pointer.reset();
                        }
                        else {
                            pointer = InterfaceBasePtr(pointer.get(), smart_pointer_deleter<InterfaceBasePtr>(pointer,INTERFACE_DELETER));
                            pointer->__strpluginname = (*itplugin)->ppluginname;
                            pointer->__strxmlid = name;
                            pointer->__plugin = *itplugin;
                            break;
                        }
                    }
                    if( !(*itplugin)->IsValid() ) {
                        boost::mutex::scoped_lock lock(_mutex);
                        _listplugins.remove(*itplugin);
                    }
                    ++itplugin;
                }
            }
        }

        if( !!pointer ) {
            if( type == PT_Robot ) {
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pointer);
                if( strcmp(probot->GetKinBodyHash(), OPENRAVE_KINBODY_HASH) ) {
                    RAVELOG_FATAL(str(boost::format("plugin interface Robot, name %s has invalid hash, might be compiled with stale openrave files\n")%name));
                    pointer.reset();
                }
                if( !probot->IsRobot() ) {
                    RAVELOG_FATAL(str(boost::format("interface Robot, name %s should have IsRobot() return true\n")%name));
                    pointer.reset();
                }
            }
        }
        if( !pointer ) {
            RAVELOG_WARN("Failed to create name %s, interface %s\n", name.c_str(), RaveGetInterfaceNamesMap().find(type)->second.c_str());
        }
        return pointer;
    }

    /// loads all the plugins in this dir
    /// If pdir is already specified, reloads all
    bool AddDirectory(const std::string& pdir)
    {
#ifdef _WIN32
        WIN32_FIND_DATAA FindFileData;
        HANDLE hFind;
        string strfind = pdir;
        strfind += "\\*";
        strfind += PLUGIN_EXT;

        hFind = FindFirstFileA(strfind.c_str(), &FindFileData);
        if (hFind == INVALID_HANDLE_VALUE) {
            RAVELOG_DEBUG("No plugins in dir: %s (GetLastError reports %d)\n", pdir.c_str(), GetLastError ());
            return false;
        }
        else  {
            do {
                RAVELOG_DEBUG("Adding plugin %s\n", FindFileData.cFileName);
                string strplugin = pdir;
                strplugin += "\\";
                strplugin += FindFileData.cFileName;
                LoadPlugin(strplugin.c_str());
            } while (FindNextFileA(hFind, &FindFileData) != 0);
            FindClose(hFind);
        }
#else
        // linux
        DIR *dp;
        struct dirent *ep;
        dp = opendir (pdir.c_str());
        if (dp != NULL) {
            while ( (ep = readdir (dp)) != NULL ) {
                // check for a .so in every file
                if( strstr(ep->d_name, PLUGIN_EXT) != NULL ) {
                    string strplugin = pdir;
                    strplugin += "/";
                    strplugin += ep->d_name;
                    LoadPlugin(strplugin.c_str());
                }
            }
            (void) closedir (dp);
        }
        else {
            RAVELOG_DEBUG("Couldn't open directory %s\n", pdir.c_str());
        }
#endif
        return true;
    }

    void ReloadPlugins()
    {
        boost::mutex::scoped_lock lock(_mutex);
        FOREACH(itplugin,_listplugins) {
            PluginPtr newplugin = _LoadPlugin((*itplugin)->ppluginname);
            if( !!newplugin ) {
                *itplugin = newplugin;
            }
        }
        _CleanupUnusedLibraries();
    }

    bool LoadPlugin(const std::string& pluginname)
    {
        boost::mutex::scoped_lock lock(_mutex);
        std::list<PluginPtr>::iterator it = _GetPlugin(pluginname);
        string newpluginname;
        if( it != _listplugins.end() ) {
            // since we got a match, use the old name and remove the old library
            newpluginname = (*it)->ppluginname;
            _listplugins.erase(it);
        }
        else {
            newpluginname = pluginname;
        }
        PluginPtr p = _LoadPlugin(newpluginname);
        if( !!p ) {
            _listplugins.push_back(p);
        }
        _CleanupUnusedLibraries();
        return !!p;
    }

    bool RemovePlugin(const std::string& pluginname)
    {
        boost::mutex::scoped_lock lock(_mutex);
        std::list<PluginPtr>::iterator it = _GetPlugin(pluginname);
        if( it == _listplugins.end() ) {
            return false;
        }
        _listplugins.erase(it);
        _CleanupUnusedLibraries();
        return true;
    }

    virtual bool HasInterface(InterfaceType type, const string& interfacename)
    {
        boost::mutex::scoped_lock lock(_mutex);
        FOREACHC(it,_listRegisteredInterfaces) {
            RegisteredInterfacePtr registration = it->lock();
            if( !!registration ) {
                if(( interfacename.size() >= registration->_name.size()) &&( strnicmp(interfacename.c_str(),registration->_name.c_str(),registration->_name.size()) == 0) ) {
                    return true;
                }
            }
        }
        FOREACHC(itplugin, _listplugins) {
            if( (*itplugin)->HasInterface(type,interfacename) ) {
                return true;
            }
        }
        return false;
    }

    void GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins) const
    {
        plugins.clear();
        boost::mutex::scoped_lock lock(_mutex);
        FOREACHC(itplugin, _listplugins) {
            PLUGININFO info;
            if( (*itplugin)->GetInfo(info) ) {
                plugins.push_back(pair<string,PLUGININFO>((*itplugin)->GetName(),info));
            }
        }
        if( _listRegisteredInterfaces.size() > 0 ) {
            plugins.push_back(make_pair(string("__internal__"),PLUGININFO()));
            plugins.back().second.version = OPENRAVE_VERSION;
            FOREACHC(it,_listRegisteredInterfaces) {
                RegisteredInterfacePtr registration = it->lock();
                if( !!registration ) {
                    plugins.back().second.interfacenames[registration->_type].push_back(registration->_name);
                }
            }
        }
    }

    void GetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames) const
    {
        interfacenames.clear();
        boost::mutex::scoped_lock lock(_mutex);
        FOREACHC(it,_listRegisteredInterfaces) {
            RegisteredInterfacePtr registration = it->lock();
            if( !!registration ) {
                interfacenames[registration->_type].push_back(registration->_name);
            }
        }
        FOREACHC(itplugin, _listplugins) {
            PLUGININFO localinfo;
            if( !(*itplugin)->GetInfo(localinfo) ) {
                RAVELOG_WARN(str(boost::format("failed to get plugin info: %s\n")%(*itplugin)->GetName()));
            }
            else {
                // for now just return the cached info (so quering is faster)
                FOREACH(it,localinfo.interfacenames) {
                    std::vector<std::string>& vnames = interfacenames[it->first];
                    vnames.insert(vnames.end(),it->second.begin(),it->second.end());
                }
            }
        }
    }

    UserDataPtr RegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn) {
        BOOST_ASSERT(interfacehash != NULL && envhash != NULL);
        BOOST_ASSERT(!!createfn);
        BOOST_ASSERT(name.size()>0);
        if( strcmp(envhash, OPENRAVE_ENVIRONMENT_HASH) ) {
            throw openrave_exception(str(boost::format("environment invalid hash %s!=%s\n")%envhash%OPENRAVE_ENVIRONMENT_HASH),ORE_InvalidInterfaceHash);
        }
        if( strcmp(interfacehash, RaveGetInterfaceHash(type)) ) {
            throw openrave_exception(str(boost::format("interface %s invalid hash %s!=%s\n")%RaveGetInterfaceName(type)%interfacehash%RaveGetInterfaceHash(type)),ORE_InvalidInterfaceHash);
        }
        boost::mutex::scoped_lock lock(_mutex);
        RegisteredInterfacePtr pdata(new RegisteredInterface(type,name,createfn,shared_from_this()));
        pdata->_iterator = _listRegisteredInterfaces.insert(_listRegisteredInterfaces.end(),pdata);
        return pdata;
    }

    static const char* GetInterfaceHash(InterfaceBasePtr pint) {
        return pint->GetHash();
    }

protected:
    void _CleanupUnusedLibraries()
    {
        FOREACH(it,_listDestroyLibraryQueue) {
            RaveDatabase::_SysCloseLibrary(*it);
        }
        _listDestroyLibraryQueue.clear();
    }

    /// \brief Deletes the plugin from the database
    ///
    /// It is safe to delete a plugin even if interfaces currently reference it because this function just decrements
    /// the reference count instead of unloading from memory.
    std::list<PluginPtr>::iterator _GetPlugin(const std::string& pluginname)
    {
        FOREACH(it,_listplugins) {
            if( pluginname == (*it)->ppluginname ) {
                return it;
            }
        }
#if defined(HAVE_BOOST_FILESYSTEM) && BOOST_VERSION >= 103600 // stem() was introduced in 1.36
        // try matching partial base names without path and extension
        boost::filesystem::path pluginpath(pluginname);
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
        string stem = pluginpath.stem().string();
#else
        string stem = pluginpath.stem();
#endif
        FOREACH(it, _listplugins) {
            if( stem == boost::filesystem::path((*it)->ppluginname).stem() ) {
                return it;
            }
        }
#endif
        return _listplugins.end();
    }

    PluginPtr _LoadPlugin(const string& _libraryname)
    {
        string libraryname = _libraryname;
        void* plibrary = _SysLoadLibrary(libraryname.c_str(),OPENRAVE_LAZY_LOADING);
        if( plibrary == NULL ) {
            // check if PLUGIN_EXT is missing
            if( libraryname.find(PLUGIN_EXT) == string::npos ) {
                libraryname += PLUGIN_EXT;
                plibrary = _SysLoadLibrary(libraryname.c_str(),OPENRAVE_LAZY_LOADING);
            }
        }
#ifndef _WIN32
        if( plibrary == NULL ) {
            // unix libraries are prefixed with 'lib', first have to split
#if defined(HAVE_BOOST_FILESYSTEM) && BOOST_VERSION >= 103600 // stem() was introduced in 1.36
            boost::filesystem::path _librarypath(libraryname);
            string librarypath = _librarypath.parent_path().string();
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
            string libraryfilename = _librarypath.filename().string();
#else
            string libraryfilename = _librarypath.filename();
#endif
            if(( libraryfilename.size() > 3) &&( libraryfilename.substr(0,3) != string("lib")) ) {
                libraryname = librarypath;
                if( libraryname.size() > 0 ) {
                    libraryname += s_filesep;
                }
                libraryname += string("lib");
                libraryname += libraryfilename;
                plibrary = _SysLoadLibrary(libraryname.c_str(),OPENRAVE_LAZY_LOADING);
            }
#endif
        }
#endif

#ifdef HAVE_BOOST_FILESYSTEM
        if( plibrary == NULL ) {
            // try adding from the current plugin libraries
            FOREACH(itdir,_listplugindirs) {
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
                string newlibraryname = boost::filesystem::absolute(libraryname,*itdir).string();
#else
                string newlibraryname = boost::filesystem::complete(libraryname,*itdir).string();
#endif
                plibrary = _SysLoadLibrary(newlibraryname.c_str(),OPENRAVE_LAZY_LOADING);
                if( !!plibrary ) {
                    libraryname = newlibraryname;
                    break;
                }
            }
        }
#endif
        if( plibrary == NULL ) {
            RAVELOG_WARN("failed to load: %s\n", _libraryname.c_str());
            return PluginPtr();
        }

        PluginPtr p(new Plugin(shared_from_this()));
        p->ppluginname = libraryname;
        p->plibrary = plibrary;

        try {
            if( !p->Load_GetPluginAttributes() ) {
                RAVELOG_WARN(str(boost::format("%s: can't load GetPluginAttributes function\n")%libraryname));
                return PluginPtr();
            }

            if( p->pfnGetPluginAttributesNew != NULL ) {
                p->pfnGetPluginAttributesNew(&p->_infocached, sizeof(p->_infocached),OPENRAVE_PLUGININFO_HASH);
            }
            else {
                if( !p->pfnGetPluginAttributes(&p->_infocached, sizeof(p->_infocached)) ) {
                    RAVELOG_WARN(str(boost::format("%s: GetPluginAttributes failed\n")%libraryname));
                    return PluginPtr();
                }
            }
        }
        catch(const openrave_exception& ex) {
            if( OPENRAVE_LAZY_LOADING ) {
                p->plibrary = NULL;     // NOTE: for some reason, closing the lazy loaded library can make the system crash, so instead keep the pointer around, but create a new one with RTLD_NOW
            }
            RAVELOG_WARN(str(boost::format("%s failed to load: %s\n")%libraryname%ex.what()));
            return PluginPtr();
        }
        catch(...) {
            if( OPENRAVE_LAZY_LOADING ) {
                p->plibrary = NULL;     // NOTE: for some reason, closing the lazy loaded library can make the system crash, so instead keep the pointer around, but create a new one with RTLD_NOW
            }
            RAVELOG_WARN(str(boost::format("%s: unknown exception\n")%libraryname));
            return PluginPtr();
        }

#ifndef _WIN32
        Dl_info info;
        if( p->pfnGetPluginAttributesNew != NULL ) {
            dladdr((void*)p->pfnGetPluginAttributesNew, &info);
        }
        else {
            dladdr((void*)p->pfnGetPluginAttributes, &info);
        }
        RAVELOG_DEBUG("loading plugin: %s\n", info.dli_fname);
#endif
        if( OPENRAVE_LAZY_LOADING ) {
            // have confirmed that plugin is ok, so reload with no-lazy loading
            p->plibrary = NULL;     // NOTE: for some reason, closing the lazy loaded library can make the system crash, so instead keep the pointer around, but create a new one with RTLD_NOW
            p->Destroy();
            p->_bShutdown = false;
        }

        return p;
    }

    static void* _SysLoadLibrary(const std::string& lib, bool bLazy=false)
    {
        // check if file exists first
        if( !ifstream(lib.c_str()) ) {
            return NULL;
        }
#ifdef _WIN32
        void* plib = LoadLibraryA(lib.c_str());
        if( plib == NULL ) {
            RAVELOG_WARN("Failed to load %s\n", lib.c_str());
        }
#else
        dlerror();     // clear error
        void* plib = dlopen(lib.c_str(), bLazy ? RTLD_LAZY : RTLD_NOW);
        char* pstr = dlerror();
        if( pstr != NULL ) {
            RAVELOG_WARN("%s: %s\n",lib.c_str(),pstr);
            if( plib != NULL ) {
                dlclose(plib);     //???
            }
            return NULL;
        }
#endif
        return plib;
    }

    static void* _SysLoadSym(void* lib, const std::string& sym)
    {
#ifdef _WIN32
        return GetProcAddress((HINSTANCE)lib, sym.c_str());
#else
        dlerror();     // clear existing error
        void* psym = dlsym(lib, sym.c_str());
        char* errorstring = dlerror();
        if( errorstring != NULL ) {
            return psym;
        }
        if( psym != NULL ) {
            // check for errors if something valid is returned since we'll be executing it
            if( errorstring != NULL ) {
                throw openrave_exception(errorstring,ORE_InvalidPlugin);
            }
        }
        return psym;
#endif
    }

    static void _SysCloseLibrary(void* lib)
    {
#ifdef _WIN32
        FreeLibrary((HINSTANCE)lib);
#else
        // can segfault if opened library clashes with other
        // need to use some combination of setjmp, longjmp to get this to work corectly
        //sighandler_t tprev = signal(SIGSEGV,fault_handler);
        dlclose(lib);
        //signal(SIGSEGV,tprev);
#endif
    }

    void _QueueLibraryDestruction(void* lib)
    {
        _listDestroyLibraryQueue.push_back(lib);
    }

    void _InterfaceDestroyCallback(InterfaceBase* pbody)
    {
        if( pbody != NULL ) {
            // post-processing?
            delete pbody;
        }
    }
    void _InterfaceDestroyCallbackShared(void const* pinterface)
    {
        if( pinterface != NULL ) {
            // post-processing for deleting interfaces
        }
    }

    void _AddToLoader(PluginPtr p)
    {
        boost::mutex::scoped_lock lock(_mutexPluginLoader);
        _listPluginsToLoad.push_back(p);
        _condLoaderHasWork.notify_all();
    }

    void _PluginLoaderThread()
    {
        while(!_bShutdown) {
            list<PluginPtr> listPluginsToLoad;
            {
                boost::mutex::scoped_lock lock(_mutexPluginLoader);
                if( _listPluginsToLoad.size() == 0 ) {
                    _condLoaderHasWork.wait(lock);
                    if( _bShutdown ) {
                        break;
                    }
                }
                listPluginsToLoad.swap(_listPluginsToLoad);
            }
            FOREACH(itplugin,listPluginsToLoad) {
                if( _bShutdown ) {
                    break;
                }
                boost::mutex::scoped_lock lockplugin((*itplugin)->_mutex);
                if( _bShutdown ) {
                    break;
                }
                (*itplugin)->plibrary = _SysLoadLibrary((*itplugin)->ppluginname,false);
                if( (*itplugin)->plibrary == NULL ) {
                    // for some reason cannot load the library, so shut it down
                    (*itplugin)->_bShutdown = true;
                }
                (*itplugin)->_cond.notify_all();
            }
        }
    }

    list<PluginPtr> _listplugins;
    mutable boost::mutex _mutex;     ///< changing plugin database
    std::list<void*> _listDestroyLibraryQueue;
    std::list< boost::weak_ptr<RegisteredInterface> > _listRegisteredInterfaces;
    std::list<std::string> _listplugindirs;

    /// \name plugin loading
    //@{
    mutable boost::mutex _mutexPluginLoader;     ///< specifically for loading shared objects
    boost::condition _condLoaderHasWork;
    std::list<PluginPtr> _listPluginsToLoad;
    boost::shared_ptr<boost::thread> _threadPluginLoader;
    bool _bShutdown;
    //@}
};

} // end namespace OpenRAVE

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RaveDatabase::Plugin)
#endif

#endif
