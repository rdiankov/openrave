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

#include "openrave/openrave.h"
#include "openrave/plugininfo.h"

#include <boost/shared_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

namespace OpenRAVE {

/// \brief database of interfaces from plugins
class RaveDatabase : public boost::enable_shared_from_this<RaveDatabase>
{
    struct RegisteredInterface : public UserData
    {
        RegisteredInterface(InterfaceType type, const std::string& name, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn, boost::shared_ptr<RaveDatabase> database);
        virtual ~RegisteredInterface();

        InterfaceType _type;
        std::string _name;
        boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)> _createfn;
        std::list< boost::weak_ptr<RegisteredInterface> >::iterator _iterator;
    protected:
        boost::weak_ptr<RaveDatabase> _database;
    };
    typedef boost::shared_ptr<RegisteredInterface> RegisteredInterfacePtr;

public:
    class Plugin : public UserData, public boost::enable_shared_from_this<Plugin>
    {
    public:
        Plugin(boost::shared_ptr<RaveDatabase> pdatabase);
        virtual ~Plugin();

        virtual void Destroy();

        virtual bool IsValid();

        const std::string& GetName() const;

        bool GetInfo(PLUGININFO& info);

        virtual bool Load_CreateInterfaceGlobal();

        virtual bool Load_GetPluginAttributes();

        virtual bool Load_DestroyPlugin();

        virtual bool Load_OnRaveInitialized();

        virtual bool Load_OnRavePreDestroy();

        bool HasInterface(InterfaceType type, const std::string& name);

        InterfaceBasePtr CreateInterface(InterfaceType type, const std::string& name, const char* interfacehash, EnvironmentBasePtr penv);

        /// \brief call to initialize the plugin, if initialized already, then ignore the call.
        void OnRaveInitialized();

        void OnRavePreDestroy();

    protected:
        /// if the library is not loaded yet, wait for it.
        void _confirmLibrary();

        boost::weak_ptr<RaveDatabase> _pdatabase;
        std::set<std::pair< InterfaceType, std::string> > _setBadInterfaces;         ///< interfaces whose hash is wrong and shouldn't be tried for this plugin
        std::string ppluginname;

        void* plibrary;         // loaded library (NULL if not loaded)
        PluginExportFn_CreateInterface pfnCreate;
        PluginExportFn_OpenRAVECreateInterface pfnCreateNew;
        PluginExportFn_GetPluginAttributes pfnGetPluginAttributes;
        PluginExportFn_OpenRAVEGetPluginAttributes pfnGetPluginAttributesNew;
        PluginExportFn_DestroyPlugin pfnDestroyPlugin;
        PluginExportFn_OnRaveInitialized pfnOnRaveInitialized;
        PluginExportFn_OnRavePreDestroy pfnOnRavePreDestroy;
        PLUGININFO _infocached;
        std::mutex _mutex;         ///< locked when library is getting updated, only used when plibrary==NULL
        std::condition_variable _cond;
        bool _bShutdown;         ///< managed by plugin database
        bool _bInitializing; ///< still in the initialization phase
        bool _bHasCalledOnRaveInitialized; ///< if true, then OnRaveInitialized has been called and does not need to call it again.

        friend class RaveDatabase;
    };
    typedef boost::shared_ptr<Plugin> PluginPtr;
    typedef boost::shared_ptr<Plugin const> PluginConstPtr;
    friend class Plugin;

    RaveDatabase();
    virtual ~RaveDatabase();

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

    virtual bool Init(bool bLoadAllPlugins);

    /// Destroy all plugins and directories
    virtual void Destroy();

    void GetPlugins(std::list<PluginPtr>& listplugins) const;

    InterfaceBasePtr Create(EnvironmentBasePtr penv, InterfaceType type, const std::string& _name);

    /// loads all the plugins in this dir
    /// If pdir is already specified, reloads all
    bool AddDirectory(const std::string& pdir);

    void ReloadPlugins();

    void OnRaveInitialized();

    void OnRavePreDestroy();

    bool LoadPlugin(const std::string& pluginname);

    bool RemovePlugin(const std::string& pluginname);

    virtual bool HasInterface(InterfaceType type, const std::string& interfacename);

    void GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins) const;

    void GetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames) const;

    UserDataPtr RegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn);

protected:
    void _CleanupUnusedLibraries();

    /// \brief Deletes the plugin from the database
    ///
    /// It is safe to delete a plugin even if interfaces currently reference it because this function just decrements
    /// the reference count instead of unloading from memory.
    std::list<PluginPtr>::iterator _GetPlugin(const std::string& pluginname);

    PluginPtr _LoadPlugin(const std::string& _libraryname);

    static void* _SysLoadLibrary(const std::string& lib, bool bLazy=false);

    static void* _SysLoadSym(void* lib, const std::string& sym);

    static void _SysCloseLibrary(void* lib);

    void _QueueLibraryDestruction(void* lib);

    void _InterfaceDestroyCallbackShared(void const* pinterface);

    /// \brief makes sure plugin is in scope until after pointer is completely deleted
    void _InterfaceDestroyCallbackSharedPost(std::string name, UserDataPtr plugin);

    void _AddToLoader(PluginPtr p);

    void _PluginLoaderThread();

    std::list<PluginPtr> _listplugins;
    mutable boost::mutex _mutex;     ///< changing plugin database
    std::list<void*> _listDestroyLibraryQueue;
    std::list< boost::weak_ptr<RegisteredInterface> > _listRegisteredInterfaces;
    std::list<std::string> _listplugindirs;

    /// \name plugin loading
    //@{
    mutable std::mutex _mutexPluginLoader;     ///< specifically for loading shared objects
    std::condition_variable _condLoaderHasWork;
    std::list<PluginPtr> _listPluginsToLoad;
    boost::shared_ptr<std::thread> _threadPluginLoader;
    bool _bShutdown;
    //@}
};

} // end namespace OpenRAVE

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RaveDatabase::Plugin)
#endif

#endif

#include "plugindatabase_impl.hpp"