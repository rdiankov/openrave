// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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


/// database of planners, obstacles, sensors, and problem from plugins
class RaveDatabase : public boost::enable_shared_from_this<RaveDatabase>
{
public:
    class Plugin : public boost::enable_shared_from_this<Plugin>
    {
    public:
        Plugin(boost::shared_ptr<RaveDatabase> pdatabase) : _pdatabase(pdatabase), plibrary(NULL), pfnCreate(NULL), pfnCreateNew(NULL), pfnGetPluginAttributes(NULL), pfnGetPluginAttributesNew(NULL), pfnDestroyPlugin(NULL), _bShutdown(false) {}
        virtual ~Plugin() {
            Destroy();
        }

        virtual void Destroy() {
            boost::mutex::scoped_lock lock(_mutex);
            // do some more checking here, there still might be instances of robots, planners, and sensors out there
            if (plibrary) {
                RAVELOG_DEBUGA("RaveDatabase: closing plugin %s\n", ppluginname.c_str()); Sleep(10);
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

        virtual bool IsValid() { return !_bShutdown; }

        const string& GetName() const { return ppluginname; }
        bool GetInfo(PLUGININFO& info) {
            // for now just return the cached info (so quering is faster)
            FOREACH(it,_infocached.interfacenames) {
                std::vector<std::string>& vnames = info.interfacenames[it->first];
                vnames.insert(vnames.end(),it->second.begin(),it->second.end());
            }
            return true;
            // Load_GetPluginAttributes()
            //return !!pfnGetPluginAttributes && pfnGetPluginAttributes(&info, sizeof(info));
        }

        virtual bool Load_CreateInterfaceGlobal()
        {
            _confirmLibrary();
            if( pfnCreateNew == NULL && pfnCreate == NULL ) {
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
            if( pfnGetPluginAttributesNew == NULL || pfnGetPluginAttributes == NULL ) {
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

        /// Check that name is actually supported.
        bool hasInterface(InterfaceType type, const string& name)
        {
            if( name.size() == 0 )
                return false;
            std::map<InterfaceType, std::vector<std::string> >::iterator itregisterednames = _infocached.interfacenames.find(type);
            if( itregisterednames == _infocached.interfacenames.end() ) {
                return false;
            }
            FOREACH(it,itregisterednames->second) {
                if( name.size() >= it->size() && strnicmp(&name[0],it->c_str(),it->size()) == 0 ) {
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

            if( !hasInterface(type,name) ) {
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
                if( !!pinterface ) {
                    if( strcmp(pinterface->GetHash(), interfacehash) ) {
                        RAVELOG_FATALA("plugin interface name %s, %s has invalid hash, might be compiled with stale openrave files\n", name.c_str(), RaveGetInterfaceNamesMap().find(type)->second.c_str());
                        _setBadInterfaces.insert(p);
                        return InterfaceBasePtr();
                    }
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
                    _cond.wait(_mutex);
                } while(1);
            }
        }
        
        boost::weak_ptr<RaveDatabase> _pdatabase;
        std::set<pair< InterfaceType, string> > _setBadInterfaces; ///< interfaces whose hash is wrong and shouldn't be tried for this plugin
        string ppluginname;

        void* plibrary; // loaded library (NULL if not loaded)
        PluginExportFn_CreateInterface pfnCreate;
        PluginExportFn_OpenRAVECreateInterface pfnCreateNew;
        PluginExportFn_GetPluginAttributes pfnGetPluginAttributes;
        PluginExportFn_OpenRAVEGetPluginAttributes pfnGetPluginAttributesNew;
        PluginExportFn_DestroyPlugin pfnDestroyPlugin;
        PLUGININFO _infocached;
        boost::mutex _mutex; ///< locked when library is getting updated, only used when plibrary==NULL
        boost::condition _cond;
        bool _bShutdown; ///< managed by plugin database

        friend class RaveDatabase;
    };
    typedef boost::shared_ptr<Plugin> PluginPtr;
    typedef boost::shared_ptr<Plugin const> PluginConstPtr;
    friend class Plugin;

    RaveDatabase() : _bShutdown(false) {
        _threadPluginLoader = boost::thread(boost::bind(&RaveDatabase::_PluginLoaderThread, this));
    }
    virtual ~RaveDatabase() { Destroy(); }

    RobotBasePtr CreateRobot(EnvironmentBasePtr penv, const std::string& pname)
    {
        RobotBasePtr probot = RaveInterfaceCast<RobotBase>(Create(penv, PT_Robot, pname));
        if( !!probot ) {
            if( strcmp(probot->GetKinBodyHash(), OPENRAVE_KINBODY_HASH) ) {
                RAVELOG_FATALA("plugin interface Robot, name %s has invalid hash, might be compiled with stale openrave files\n", pname.c_str());
                probot.reset();
            }
        }

        BOOST_ASSERT( !probot || probot->IsRobot() );
        return probot;
    }

    KinBodyPtr CreateKinBody(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<KinBody>(Create(penv, PT_KinBody, pname)); }
    PlannerBasePtr CreatePlanner(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<PlannerBase>(Create(penv, PT_Planner, pname)); }
    SensorSystemBasePtr CreateSensorSystem(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<SensorSystemBase>(Create(penv, PT_SensorSystem, pname)); }
    ControllerBasePtr CreateController(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<ControllerBase>(Create(penv, PT_Controller, pname)); }
    ProblemInstancePtr CreateProblem(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<ProblemInstance>(Create(penv, PT_ProblemInstance, pname)); }
    IkSolverBasePtr CreateIkSolver(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<IkSolverBase>(Create(penv, PT_InverseKinematicsSolver, pname)); }
    PhysicsEngineBasePtr CreatePhysicsEngine(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<PhysicsEngineBase>(Create(penv, PT_PhysicsEngine, pname)); }
    SensorBasePtr CreateSensor(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<SensorBase>(Create(penv, PT_Sensor, pname)); }
    CollisionCheckerBasePtr CreateCollisionChecker(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<CollisionCheckerBase>(Create(penv, PT_CollisionChecker, pname)); }
    ViewerBasePtr CreateViewer(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<ViewerBase>(Create(penv, PT_Viewer, pname)); }

    /// Destroy all plugins and directories
    virtual void Destroy()
    {
        RAVELOG_DEBUG("shutting down\n");
        {
            boost::mutex::scoped_lock lock(_mutexPluginLoader);
            _bShutdown = true;
            _condLoaderHasWork.notify_all();
        }
        _threadPluginLoader.join();
        {
            EnvironmentMutex::scoped_lock lock(_mutex);
            _listplugins.clear();
            vplugindirs.clear();
        }
        RAVELOG_DEBUG("cleaning libraries\n");
        CleanupUnusedLibraries();
    }

    void GetPlugins(std::list<PluginPtr>& listplugins)
    {
        EnvironmentMutex::scoped_lock lock(_mutex);
        listplugins = _listplugins;
    }
    
    InterfaceBasePtr Create(EnvironmentBasePtr penv, InterfaceType type, const std::string& name)
    {
        if( name.size() == 0 )
            return InterfaceBasePtr();

        size_t nInterfaceNameLength = name.find_first_of(' ');
        if( nInterfaceNameLength == string::npos )
            nInterfaceNameLength = name.size();
        if( nInterfaceNameLength == 0 ) {
            RAVELOG_WARN(str(boost::format("interface name \"%s\" needs to start with a valid character\n")%name));
            return InterfaceBasePtr();
        }
                
        EnvironmentMutex::scoped_lock lock(_mutex);
        const char* hash = RaveGetInterfaceHash(type);
        list<PluginPtr>::iterator itplugin = _listplugins.begin();
        while(itplugin != _listplugins.end()) {
            InterfaceBasePtr pointer = (*itplugin)->CreateInterface(type, name, hash, penv);
            if( !!pointer ) {
                pointer = InterfaceBasePtr(pointer.get(), smart_pointer_deleter<InterfaceBasePtr>(pointer,INTERFACE_DELETER));
                pointer->__strpluginname = (*itplugin)->ppluginname;
                pointer->__strxmlid = name;
                pointer->__plugin = *itplugin;
                return pointer;
            }
            
            if( (*itplugin)->IsValid() ) {
                ++itplugin;
            }
            else {
                itplugin = _listplugins.erase(itplugin);
            }
        }

        if( name.size() == 0 ) {
            switch(type) {
            case PT_KinBody: return penv->CreateKinBody();
            case PT_Trajectory: return penv->CreateTrajectory(0);
            default:
                break;
            }
        }

        RAVELOG_WARNA("Failed to create name %s, interface %s\n", name.c_str(), RaveGetInterfaceNamesMap().find(type)->second.c_str());
        return InterfaceBasePtr();
    }

    /// loads all the plugins in this dir
    /// If pdir is already specified, reloads all 
    bool AddDirectory(const std::string& pdir)
    {
        EnvironmentMutex::scoped_lock lock(_mutex);
#ifdef _WIN32
        WIN32_FIND_DATAA FindFileData;
        HANDLE hFind;
        string strfind = pdir;
        strfind += "\\*";
        strfind += PLUGIN_EXT;

        hFind = FindFirstFileA(strfind.c_str(), &FindFileData);
        if (hFind == INVALID_HANDLE_VALUE) {
            RAVELOG_DEBUGA("No plugins in dir: %s (GetLastError reports %d)\n", pdir.c_str(), GetLastError ());
            return false;
        } 
        else 
            {
                do {
                    RAVELOG_DEBUGA("Adding plugin %s\n", FindFileData.cFileName);
                    string strplugin = pdir;
                    strplugin += "\\";
                    strplugin += FindFileData.cFileName;
                    AddPlugin(strplugin.c_str());
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
                    AddPlugin(strplugin.c_str());
                }
            }
            (void) closedir (dp);
        }
        else
            RAVELOG_WARNA("Couldn't open directory %s\n", pdir.c_str());
#endif

        return true;
    }

    void ReloadPlugins()
    {
        EnvironmentMutex::scoped_lock lock(_mutex);
        FOREACH(itplugin,_listplugins) {
            PluginPtr newplugin = _LoadPlugin((*itplugin)->ppluginname);
            if( !!newplugin )
                *itplugin = newplugin;
        }
        CleanupUnusedLibraries();   
    }

    bool AddPlugin(const std::string& libraryname)
    {
        EnvironmentMutex::scoped_lock lock(_mutex);
        DeletePlugin(libraryname); // first delete it
        PluginPtr p = _LoadPlugin(libraryname);
        if( !!p )
            _listplugins.push_back(p);
        CleanupUnusedLibraries();
        return !!p;
    }

    /// deletes the plugin from memory. Note that the function doesn't check if there's any object
    /// pointers that were instantiated from the specific plugin. Those objects should be removed
    /// before calling this function in order to avoid segmentation faults later on.
    bool DeletePlugin(const std::string& name)
    {
        EnvironmentMutex::scoped_lock lock(_mutex);
        bool bFound = false;
        list<PluginPtr>::iterator it = _listplugins.begin();
        while(it != _listplugins.end()) {
            if( name == (*it)->ppluginname ) {
                it = _listplugins.erase(it);
                bFound = true;
            }
            else ++it;
        }

        return bFound;
    }

    virtual bool HasInterface(InterfaceType type, const string& interfacename)
    {
        EnvironmentMutex::scoped_lock lock(_mutex);
        size_t ind = interfacename.find_first_of(' ');
        if( ind == string::npos ) {
            ind = interfacename.size();
        }
        FOREACHC(itplugin, _listplugins) {
            PLUGININFO info;
            if( (*itplugin)->GetInfo(info) ) {
                FOREACHC(itname, info.interfacenames[type]) {
                    if( ind >= itname->size() && strnicmp(itname->c_str(),interfacename.c_str(),ind) == 0 )
                        return true;
                }
            }
            else {
                RAVELOG_WARNA(str(boost::format("%s: GetPluginAttributes failed\n")%(*itplugin)->ppluginname));
            }
        }
        return false;
    }

    void CleanupUnusedLibraries()
    {
        EnvironmentMutex::scoped_lock lock(_mutex);
        FOREACH(it,_listDestroyLibraryQueue) {
            RaveDatabase::_SysCloseLibrary(*it);
        }
        _listDestroyLibraryQueue.clear();
    }

    static const char* GetInterfaceHash(InterfaceBasePtr pint) { return pint->GetHash(); }

protected:
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
            
            if( plibrary == NULL ) {
                RAVELOG_WARNA("failed to load: %s\n", _libraryname.c_str());
                return PluginPtr();
            }
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
                p->plibrary = NULL; // NOTE: for some reason, closing the lazy loaded library can make the system crash, so instead keep the pointer around, but create a new one with RTLD_NOW
            }
            RAVELOG_WARN(str(boost::format("%s failed to load: %s\n")%libraryname%ex.what()));
            return PluginPtr();
        }
        catch(...) {
            if( OPENRAVE_LAZY_LOADING ) {
                p->plibrary = NULL; // NOTE: for some reason, closing the lazy loaded library can make the system crash, so instead keep the pointer around, but create a new one with RTLD_NOW
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
            p->plibrary = NULL; // NOTE: for some reason, closing the lazy loaded library can make the system crash, so instead keep the pointer around, but create a new one with RTLD_NOW
            p->Destroy();
            p->_bShutdown = false;
        }
        
        return p;
    }

    static void* _SysLoadLibrary(const std::string& lib, bool bLazy=false)
    {
#ifdef _WIN32
        void* plib = LoadLibraryA(lib.c_str());
        if( plib == NULL ) {
            RAVELOG_WARNA("Failed to load %s\n", lib.c_str());
        }
#else
        dlerror(); // clear error
        void* plib = dlopen(lib.c_str(), bLazy ? RTLD_LAZY : RTLD_NOW);
        char* pstr = dlerror();
        if( pstr != NULL ) {
            RAVELOG_WARN("%s: %s\n",lib.c_str(),pstr);
            if( plib != NULL ) {
                dlclose(plib); //???
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
        dlerror(); // clear existing error
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
        EnvironmentMutex::scoped_lock lock(_mutex);
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
                    _condLoaderHasWork.wait(_mutexPluginLoader);
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
    vector<string> vplugindirs; ///< local directory for all plugins
    EnvironmentMutex _mutex; ///< changing plugin database
    list<void*> _listDestroyLibraryQueue;

    /// \name plugin loading
    //@{
    mutable boost::mutex _mutexPluginLoader; ///< specifically for loading shared objects
    boost::condition _condLoaderHasWork;
    list<PluginPtr> _listPluginsToLoad;
    boost::thread _threadPluginLoader;
    bool _bShutdown;
    //@}
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RaveDatabase::Plugin)
#endif

#endif
