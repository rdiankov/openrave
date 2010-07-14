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
#else
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
    /// create the interfaces
    typedef InterfaceBasePtr (*CreateInterfaceFn)(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv);

    /// called to get information about what the plugin provides
    typedef bool (*GetPluginAttributesFn)(PLUGININFO* pinfo, int size);

    /// called when plugin is about to be destroyed
    typedef void (*DestroyPluginFn)();

    class Plugin
    {
    public:
        Plugin(boost::shared_ptr<RaveDatabase> pdatabase) : _pdatabase(pdatabase), plibrary(NULL), pfnCreate(NULL), pfnGetPluginAttributes(NULL), pfnDestroyPlugin(NULL) {}
        virtual ~Plugin() {
            // do some more checking here, there still might be instances of robots, planners, and sensors out there
            if (plibrary) {
                RAVELOG_DEBUGA("RaveDatabase: closing plugin %s\n", ppluginname.c_str()); Sleep(10);
                
                if( pfnDestroyPlugin != NULL )
                    pfnDestroyPlugin();
                
                boost::shared_ptr<RaveDatabase> pdatabase = _pdatabase.lock();
                if( !!pdatabase )
                    pdatabase->QueueLibraryDestruction(plibrary);
            }
        }

        const string& GetName() const { return ppluginname; }
        bool GetInfo(PLUGININFO& info) {
            return !!pfnGetPluginAttributes && pfnGetPluginAttributes(&info, sizeof(info));
        }

        InterfaceBasePtr CreateInterface(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv) {
            if( pfnCreate == NULL ) {
#ifdef _MSC_VER
                pfnCreate = (CreateInterfaceFn)SysLoadSym(plibrary, "?CreateInterface@@YA?AV?$shared_ptr@VInterfaceBase@OpenRAVE@@@boost@@W4InterfaceType@OpenRAVE@@ABV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@PBDV?$shared_ptr@VEnvironmentBase@OpenRAVE@@@2@@Z");
#else
                pfnCreate = (CreateInterfaceFn)SysLoadSym(plibrary, "_Z15CreateInterfaceN8OpenRAVE10InterfaceTypeERKSsPKcN5boost10shared_ptrINS_15EnvironmentBaseEEE");
#endif
                if( pfnCreate == NULL ) {
                    pfnCreate = (CreateInterfaceFn)SysLoadSym(plibrary, "CreateInterface");
                    if( pfnCreate == NULL ) {
                        throw openrave_exception(str(boost::format("%s: can't load CreateInterface function\n")%ppluginname));
                    }
                }
            }

            return pfnCreate(type,name,pluginhash,penv);
        }
        
    protected:
        boost::weak_ptr<RaveDatabase> _pdatabase;
        string ppluginname;

        void* plibrary; // loaded library (NULL if not loaded)
        CreateInterfaceFn pfnCreate;
        GetPluginAttributesFn pfnGetPluginAttributes;
        DestroyPluginFn pfnDestroyPlugin;

        friend class RaveDatabase;
    };
    typedef boost::shared_ptr<Plugin> PluginPtr;
    typedef boost::shared_ptr<Plugin const> PluginConstPtr;

    RaveDatabase() {}
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
    RaveViewerBasePtr CreateViewer(EnvironmentBasePtr penv, const std::string& pname) { return RaveInterfaceCast<RaveViewerBase>(Create(penv, PT_Viewer, pname)); }

    /// Destroy all plugins and directories
    virtual void Destroy()
    {
        {
            EnvironmentMutex::scoped_lock lock(_mutex);
            _listplugins.clear();
            vplugindirs.clear();
        }
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
            try {
                InterfaceBasePtr pointer = (*itplugin)->CreateInterface(type, name, hash, penv);
                pointer = InterfaceBasePtr(pointer.get(), smart_pointer_deleter<InterfaceBasePtr>(pointer,INTERFACE_DELETER));
                if( !!pointer ) {
                    if( strcmp(pointer->GetHash(), hash) ) {
                        RAVELOG_FATALA("plugin interface name %s, %s has invalid hash, might be compiled with stale openrave files\n", name.c_str(), RaveGetInterfaceNamesMap().find(type)->second.c_str());
                        ++itplugin;
                        continue;
                    }
                    
                    pointer->__strpluginname = (*itplugin)->ppluginname;
                    pointer->__strxmlid = name;
                    pointer->__plugin = *itplugin;
                    return pointer;
                }
            }
            catch(const openrave_exception& ex) {
                RAVELOG_ERROR(str(boost::format("Create Interface: openrave exception , plugin %s: %s\n")%(*itplugin)->ppluginname%ex.what()));
                if( ex.GetCode() == ORE_PluginInvalid ) {
                    itplugin = _listplugins.erase(itplugin);
                    continue;
                }
            }
            catch(...) {
                RAVELOG_ERROR(str(boost::format("Create Interface: unknown exception, plugin %s\n")%(*itplugin)->ppluginname));
            }

            ++itplugin;
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
    }

    bool AddPlugin(const std::string& libraryname)
    {
        EnvironmentMutex::scoped_lock lock(_mutex);
        DeletePlugin(libraryname); // first delete it
        PluginPtr p = _LoadPlugin(libraryname);
        if( !!p )
            _listplugins.push_back(p);
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
        FOREACH(it,_listDestroyLibraryQueue)
            RaveDatabase::SysCloseLibrary(*it);
        _listDestroyLibraryQueue.clear();
    }

    static const char* GetInterfaceHash(InterfaceBasePtr pint) { return pint->GetHash(); }

protected:
    PluginPtr _LoadPlugin(const string& _libraryname)
    {
        string libraryname = _libraryname;
        void* plibrary = SysLoadLibrary(libraryname.c_str());
        if( plibrary == NULL ) {
            // check if PLUGIN_EXT is missing
            if( libraryname.find(PLUGIN_EXT) == string::npos ) {
                libraryname += PLUGIN_EXT;
                plibrary = SysLoadLibrary(libraryname.c_str());
            }
            
            if( plibrary == NULL ) {
                RAVELOG_WARNA("failed to load: %s\n", _libraryname.c_str());
                return PluginPtr();
            }
        }

        PluginPtr p(new Plugin(shared_from_this()));
        p->ppluginname = libraryname;
        p->plibrary = plibrary;
      
#ifdef _MSC_VER
        p->pfnGetPluginAttributes = (GetPluginAttributesFn)SysLoadSym(p->plibrary, "?GetPluginAttributes@@YA_NPAUPLUGININFO@OpenRAVE@@H@Z");
#else
        p->pfnGetPluginAttributes = (GetPluginAttributesFn)SysLoadSym(p->plibrary, "_Z19GetPluginAttributesPN8OpenRAVE10PLUGININFOEi");
#endif
        if( !p->pfnGetPluginAttributes ) {
            p->pfnGetPluginAttributes = (GetPluginAttributesFn)SysLoadSym(p->plibrary, "GetPluginAttributes");
            if( !p->pfnGetPluginAttributes ) {
                RAVELOG_ERRORA(str(boost::format("%s: can't load GetPluginAttributes function\n")%p->ppluginname));
                return PluginPtr();
            }
        }

#ifdef _MSC_VER
        p->pfnDestroyPlugin = (DestroyPluginFn)SysLoadSym(p->plibrary, "?DestroyPlugin@@YAXXZ");
#else
        p->pfnDestroyPlugin = (DestroyPluginFn)SysLoadSym(p->plibrary, "_Z13DestroyPluginv");
#endif
        if( p->pfnDestroyPlugin == NULL ) {
            p->pfnDestroyPlugin = (DestroyPluginFn)SysLoadSym(p->plibrary, "DestroyPlugin");
            if( p->pfnDestroyPlugin == NULL ) {
                RAVELOG_WARNA(str(boost::format("%s: can't load DestroyPlugin function, passing...\n")%p->ppluginname));
            }
        }
        
#ifndef _WIN32
        Dl_info info;
        dladdr((void*)p->pfnGetPluginAttributes, &info);
        RAVELOG_DEBUGA("loading plugin: %s\n", info.dli_fname);
#endif

        return p;
    }

    static void* SysLoadLibrary(const std::string& lib)
    {
#ifdef _WIN32
        void* plib = LoadLibraryA(lib.c_str());
        if( plib == NULL ) {
            RAVELOG_WARNA("Failed to load %s\n", lib.c_str());
        }
#else
        void* plib = dlopen(lib.c_str(), RTLD_LAZY);
        if( plib == NULL )
            RAVELOG_WARNA("%s\n", dlerror());
#endif
        return plib;
    }

    static void* SysLoadSym(void* lib, const std::string& sym)
    {
#ifdef _WIN32
        return GetProcAddress((HINSTANCE)lib, sym.c_str());
#else
        return dlsym(lib, sym.c_str());
#endif
    }
    
    static void SysCloseLibrary(void* lib)
    {
#ifdef _WIN32
        FreeLibrary((HINSTANCE)lib);
#else
        dlclose(lib);
#endif
    }

    void QueueLibraryDestruction(void* lib)
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

    list<PluginPtr> _listplugins;
    vector<string> vplugindirs; ///< local directory for all plugins
    EnvironmentMutex _mutex;

    list<void*> _listDestroyLibraryQueue;

    friend class Plugin;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RaveDatabase::Plugin)
#endif

#endif
