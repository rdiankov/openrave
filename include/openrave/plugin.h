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
 */
#ifndef OPENRAVE_PLUGIN_H
#define OPENRAVE_PLUGIN_H

#include <openrave/openrave.h>
#include <boost/format.hpp>

// export symbol prefix for plugin functions
#define OPENRAVE_PLUGIN_API extern "C" OPENRAVE_HELPER_DLL_EXPORT

/// \deprecated
#define RAVE_PLUGIN_API OPENRAVE_PLUGIN_API

/// \brief \b <b>[helper]</b> Validated function callback for creating an interface function. No checks need to be made on the parmaeters.
///
/// \ingroup plugin_exports
/// If possible, always returns a valid pointer regardless of initialization failure since the actual interface
/// pointer stores documentation information and is used in introspection.
/// Only use when \ref rave/plugin.h is included.
/// \param[in] type the interface type
/// \param[in] name the lowercase letters of the interface name
/// \param[in] sinput a stream to the rest of the input args to \ref OpenRAVECreateInterface
/// \param[in] penv the environment pointer
/// \return a pointer to the interface if one could have been created.
OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& name, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv);

/** \brief \b <b>[helper]</b> Validated function callback for returning a plugin's information. No checks need to be made on the parmaeters.

    \ingroup plugin_exports
    This function is called only once initially to determine what the plugin offers. It should be
    the safest funcdtion and should not create any static resources for the plugin.
    Only use when \ref rave/plugin.h is included.
    \param[out] info Holds information on what services this plugin provides.
 */
void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info);

/// \brief <b>[export]</b> Definition of a plugin export. Requires \ref CreateInterfaceValidated to be defined.
/// \ingroup plugin_exports
OPENRAVE_PLUGIN_API OpenRAVE::InterfaceBasePtr OpenRAVECreateInterface(OpenRAVE::InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, OpenRAVE::EnvironmentBasePtr penv)
{
    if( strcmp(interfacehash,OpenRAVE::RaveGetInterfaceHash(type)) ) {
    throw OPENRAVE_EXCEPTION_FORMAT("bad interface %s hash: %s!=%s",RaveGetInterfaceName(type)%interfacehash%OpenRAVE::RaveGetInterfaceHash(type),OpenRAVE::ORE_InvalidInterfaceHash);
    }
    if( !penv ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("need to set environment",OpenRAVE::ORE_InvalidArguments);
    }
    if( strcmp(envhash,OPENRAVE_ENVIRONMENT_HASH) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("bad environment hash: %s!=%s",envhash%OPENRAVE_ENVIRONMENT_HASH,OpenRAVE::ORE_InvalidPlugin);
    }
    OpenRAVE::RaveInitializeFromState(penv->GlobalState()); // make sure global state is set
    std::stringstream sinput(name);
    std::string interfacename;
    sinput >> interfacename;
    std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);
    return CreateInterfaceValidated(type,interfacename,sinput,penv);
}

/// \brief \b <b>[export]</b> Definition of a plugin export. Requires \ref GetPluginAttributesValidated to be defined.
/// \ingroup plugin_exports
OPENRAVE_PLUGIN_API void OpenRAVEGetPluginAttributes(OpenRAVE::PLUGININFO* pinfo, int size, const char* infohash)
{
    if( pinfo == NULL ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("bad data",OpenRAVE::ORE_InvalidArguments);
    }
    if( size != sizeof(OpenRAVE::PLUGININFO) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("bad plugin info sizes %d != %d", size%sizeof(OpenRAVE::PLUGININFO),OpenRAVE::ORE_InvalidPlugin);
    }
    if( strcmp(infohash,OPENRAVE_PLUGININFO_HASH) ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("bad plugin info hash",OpenRAVE::ORE_InvalidPlugin);
    }
    GetPluginAttributesValidated(*pinfo);
    pinfo->version = OPENRAVE_VERSION;
}

/// \brief \b <b>[export]</b> Stub function to be defined by plugin that includes \ref rave/plugin.h.
/// \ingroup plugin_exports
OPENRAVE_PLUGIN_API void DestroyPlugin();

//@}

#endif
