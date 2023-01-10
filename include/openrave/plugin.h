// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
/** \file   plugin.h
    \brief  Provides helper functions for creating plugins. Defines all the necessary functions to export.

    This file is optional and not automatically included with openrave.h
 */
#ifndef OPENRAVE_PLUGIN_H
#define OPENRAVE_PLUGIN_H

#include <boost/algorithm/string.hpp>
#include <utility>
#include <openrave/openrave.h>

// export symbol prefix for plugin functions
#define OPENRAVE_PLUGIN_API extern "C" OPENRAVE_HELPER_DLL_EXPORT

struct OPENRAVE_HELPER_DLL_EXPORT RavePlugin : public OpenRAVE::UserData, public boost::enable_shared_from_this<RavePlugin>
{
    using InterfaceMap = std::map<OpenRAVE::InterfaceType, std::vector<std::string>>;

    virtual ~RavePlugin() {}

    /// \brief \b <b>[export]</b> Called when OpenRAVE global runtime is finished initializing.
    virtual void OnRaveInitialized() {}

    /// \brief \b <b>[export]</b> Called when OpenRAVE global runtime is about to be destroyed.
    virtual void OnRavePreDestroy() {}

    /// \brief \b <b>[export]</b> Called when plugin is about to be deleted.
    virtual void Destroy() {}

    virtual const InterfaceMap& GetInterfaces() const = 0;

    virtual const std::string& GetPluginName() const = 0;

    /// \brief <b>[export]</b> Definition of a plugin export. Requires \ref CreateInterfaceValidated to be defined.
    /// \ingroup plugin_exports
    /// \brief \b <b>[helper]</b> Validated function callback for creating an interface function. No checks need to be made on the parmaeters.
    ///
    /// \ingroup plugin_exports
    /// If possible, always returns a valid pointer regardless of initialization failure since the actual interface
    /// pointer stores documentation information and is used in introspection.
    /// \param[in] type the interface type
    /// \param[in] name the lowercase letters of the interface name
    /// \param[in] sinput a stream to the rest of the input args to \ref OpenRAVECreateInterface
    /// \param[in] penv the environment pointer
    /// \return a pointer to the interface if one could have been created.
    inline OpenRAVE::InterfaceBasePtr OpenRAVECreateInterface(OpenRAVE::InterfaceType type, std::string name, const char* interfacehash, const char* envhash, OpenRAVE::EnvironmentBasePtr penv);

    /// \brief \b <b>[export]</b> Definition of a plugin export. Requires \ref GetPluginAttributesValidated to be defined.
    /// \ingroup plugin_exports
    /** \brief \b <b>[helper]</b> Validated function callback for returning a plugin's information. No checks need to be made on the parmaeters.

        \ingroup plugin_exports
        This function is called only once initially to determine what the plugin offers. It should be
        the safest funcdtion and should not create any static resources for the plugin.
        Only use when \ref rave/plugin.h is included.
        \param[out] info Holds information on what services this plugin provides.
    */
    inline void OpenRAVEGetPluginAttributes(OpenRAVE::PLUGININFO& pinfo, const char* infohash) const;

    inline bool HasInterface(OpenRAVE::InterfaceType type, const std::string& name) const;

    /// If an interface has a bad hash, add it to this set so it is not tried again.
    /// HasInterface() will not report the existence of the interface subsequently.
    inline void AddBadInterface(OpenRAVE::InterfaceType type, std::string name);

    inline void SetPluginPath(const std::string& path);
    inline const std::string& GetPluginPath() const;

    static int GetOpenRAVEVersion() noexcept {
        return OPENRAVE_VERSION;
    }

    static bool IsEnvHashValid(const char* envHash) noexcept {
        return true; //0 == strncmp(OPENRAVE_ENVIRONMENT_HASH, envHash, sizeof(OPENRAVE_ENVIRONMENT_HASH));
    }

    static bool IsPluginHashValid(const char* infoHash) noexcept {
        return true; //0 == strncmp(OPENRAVE_PLUGININFO_HASH, infoHash, sizeof(OPENRAVE_PLUGININFO_HASH));
    }

protected:
    std::string _pluginpath; ///< The path of the shared object that this plugin resides in.
    std::set<std::pair<OpenRAVE::InterfaceType, std::string>> _setBadInterfaces; ///< interfaces whose hash is wrong and shouldn't be tried for this plugin

    // Plugins must supply an implementation of this method.
    // OpenRAVECreateInterface will call this method to create the interface.
    virtual OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) = 0;
};

using PluginPtr = boost::shared_ptr<RavePlugin>;
using PluginConstPtr = boost::shared_ptr<const RavePlugin>;

// The entry point of plugins must have a function signature RavePlugin* CreatePlugin()
using PluginExportFn_Create = RavePlugin*(*)();


// Implementations


OpenRAVE::InterfaceBasePtr RavePlugin::OpenRAVECreateInterface(OpenRAVE::InterfaceType type, std::string name, const char* /*interfacehash*/, const char* /*envhash*/, OpenRAVE::EnvironmentBasePtr penv) try
{
    std::for_each(name.begin(), name.end(), ::tolower);
    std::pair<OpenRAVE::InterfaceType, std::string> p(type, name);

    if( !penv ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("need to set environment",OpenRAVE::ORE_InvalidArguments);
    }

    if(_setBadInterfaces.find(p) != _setBadInterfaces.end()) {
        return OpenRAVE::InterfaceBasePtr();
    }

    OpenRAVE::RaveInitializeFromState(penv->GlobalState()); // make sure global state is set
    std::stringstream sinput(name);
    std::string interfacename;
    sinput >> interfacename;
    std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);
    return CreateInterface(type, interfacename, sinput, penv);
} catch (const OpenRAVE::openrave_exception& ex) {
    const std::string& pluginname = GetPluginName();
    RAVELOG_ERROR_FORMAT("OpenRAVECreateInterface: %s: %s\n", pluginname % ex.what());
    if( ex.GetCode() == OpenRAVE::ORE_InvalidPlugin ) {
        RAVELOG_DEBUG_FORMAT("shared object %s is not a valid openrave plugin\n", pluginname);
        Destroy();
    }
    return OpenRAVE::InterfaceBasePtr();
} catch (const std::exception& e) {
    RAVELOG_ERROR_FORMAT("Create Interface: caught exception, plugin %s: %s\n", GetPluginName() % e.what());
    return OpenRAVE::InterfaceBasePtr();
} catch (...) {
    RAVELOG_ERROR_FORMAT("Create Interface: unknown exception, plugin %s\n", GetPluginName());
    return OpenRAVE::InterfaceBasePtr();
}

void RavePlugin::OpenRAVEGetPluginAttributes(OpenRAVE::PLUGININFO& pinfo, const char* infohash) const
{
    if( !IsPluginHashValid(infohash) ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("bad plugin info hash",OpenRAVE::ORE_InvalidPlugin);
    }
    const InterfaceMap& interfaces = GetInterfaces();
    pinfo.interfacenames.insert(interfaces.begin(), interfaces.end());
    pinfo.version = GetOpenRAVEVersion();
}

bool RavePlugin::HasInterface(OpenRAVE::InterfaceType type, const std::string& name) const
{
    if( name.empty() ) {
        return false;
    }
    const InterfaceMap& interfaces = GetInterfaces();
    std::map<OpenRAVE::InterfaceType, std::vector<std::string>>::const_iterator itregisterednames = interfaces.find(type);
    if( itregisterednames == interfaces.end() ) {
        return false;
    }
    for (const std::string& entry : itregisterednames->second) {
        if (boost::iequals(entry, name)) {
            return true;
        }
    }
    return false;
}

void RavePlugin::AddBadInterface(OpenRAVE::InterfaceType type, std::string name)
{
    _setBadInterfaces.emplace(std::make_pair(type, std::move(name)));
}

void RavePlugin::SetPluginPath(const std::string& path)
{
    _pluginpath.assign(path);
}

const std::string& RavePlugin::GetPluginPath() const
{
    return _pluginpath;
}

#endif
