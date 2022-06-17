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
#include "plugindatabase_virtual.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <boost/shared_ptr.hpp>

namespace OpenRAVE {

class DynamicRaveDatabase;

void* _SysLoadLibrary(const std::string& lib, bool bLazy=false);
void* _SysLoadSym(void* lib, const std::string& sym);
void  _SysCloseLibrary(void* lib);

// TODO: Deprecated, remove later
/// \deprecated (12/01/01)
typedef InterfaceBasePtr(*PluginExportFn_CreateInterface)(InterfaceType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr env) RAVE_DEPRECATED;

// TODO: Deprecated, remove later
/// \deprecated (12/01/01)
typedef bool(*PluginExportFn_GetPluginAttributes)(PLUGININFO* pinfo, int size) RAVE_DEPRECATED;

// Holds information about a plugin, for plugins loaded dynamically at runtime.
class Plugin : public PluginBase, public boost::enable_shared_from_this<Plugin>
{
public:
    friend class DynamicRaveDatabase;

    Plugin(boost::shared_ptr<DynamicRaveDatabase> pdatabase);
    virtual ~Plugin() override;

    virtual void Destroy() override;

    virtual bool IsValid() override;

    const std::string& GetName() const override;

    bool GetInfo(PLUGININFO& info) override;

    bool HasInterface(InterfaceType type, const std::string& name) override;

    InterfaceBasePtr CreateInterface(InterfaceType type, const std::string& name, const char* interfacehash, EnvironmentBasePtr penv) override;

    /// \brief call to initialize the plugin, if initialized already, then ignore the call.
    void OnRaveInitialized() override;

    void OnRavePreDestroy() override;

protected:
    virtual bool _Load_CreateInterfaceGlobal();

    virtual bool _Load_GetPluginAttributes();

    virtual bool _Load_DestroyPlugin();

    virtual bool _Load_OnRaveInitialized();

    virtual bool _Load_OnRavePreDestroy();

    /// if the library is not loaded yet, wait for it.
    void _confirmLibrary();

    boost::weak_ptr<DynamicRaveDatabase> _pdatabase;
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
};
typedef boost::shared_ptr<Plugin> PluginPtr;
typedef boost::shared_ptr<Plugin const> PluginConstPtr;

class DynamicRaveDatabase : public RaveDatabase, public boost::enable_shared_from_this<DynamicRaveDatabase>
{
public:
    friend class Plugin;
    friend class RegisteredInterface;

    DynamicRaveDatabase();
    virtual ~DynamicRaveDatabase();

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

    void _QueueLibraryDestruction(void* lib);

    void _InterfaceDestroyCallbackShared(void const* pinterface);

    /// \brief makes sure plugin is in scope until after pointer is completely deleted
    void _InterfaceDestroyCallbackSharedPost(std::string name, UserDataPtr plugin);

    void _AddToLoader(PluginPtr p);

    void _PluginLoaderThread();

    std::list<PluginPtr> _listplugins;
    std::list<void*> _listDestroyLibraryQueue;
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
BOOST_TYPEOF_REGISTER_TYPE(DynamicRaveDatabase::Plugin)
#endif

#endif