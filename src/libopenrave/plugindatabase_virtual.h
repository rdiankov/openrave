// -*- coding: utf-8 -*-
// Copyright (C) 2022 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef RAVE_PLUGIN_DATABASE_VIRTUAL_H
#define RAVE_PLUGIN_DATABASE_VIRTUAL_H

#include <sstream>
#include "openrave/openrave.h"
#include "openrave/plugin.h"

namespace OpenRAVE {

class RaveDatabase
{
public:
    virtual ~RaveDatabase() {}

    virtual void Init() = 0;
    virtual void Destroy() = 0;
    virtual InterfaceBasePtr Create(EnvironmentBasePtr penv, InterfaceType type, std::string name) = 0;
    virtual void OnRaveInitialized() = 0;
    virtual void OnRavePreDestroy() = 0;
    virtual bool HasInterface(InterfaceType type, const std::string& interfacename) const = 0;
    virtual void GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins) const = 0;
    virtual void GetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames) const = 0;

    virtual void ReloadPlugins() {}
    virtual bool LoadPlugin(const std::string& libraryname) = 0;

    virtual UserDataPtr AddVirtualPlugin(InterfaceType type, std::string name, std::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)> createfn) = 0;

    // Old interface
    UserDataPtr RegisterInterface(InterfaceType type, const std::string& name, const char* /*interfacehash*/, const char* /*envhash*/, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn)
    {
        return AddVirtualPlugin(type, name, createfn);
    }
};

// VirtualPlugin replaces RegisteredInterface in utility but gives it a common interface as a regular Plugin.
struct VirtualPlugin final : public RavePlugin, public boost::enable_shared_from_this<VirtualPlugin>
{
    VirtualPlugin(OpenRAVE::InterfaceType type, std::string name, std::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)> createfn)
        : _pluginname(std::move(name))
        , _createFn(std::move(createfn))
    {
        _interfaces[type].push_back(_pluginname);
        SetPluginPath("__internal__");
    }

    const InterfaceMap& GetInterfaces() const override
    {
        return _interfaces;
    }

    const std::string& GetPluginName() const override
    {
        return _pluginname;
    }

protected:
    virtual InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override
    {
        //std::istringstream sinput(_pluginname);
        //std::string interfacename;
        //sinput >> interfacename;
        //std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);
        return _createFn(penv, sinput);
    }

private:
    std::string _pluginname;
    InterfaceMap _interfaces;
    std::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)> _createFn;
};

} // namespace OpenRAVE

#endif // RAVE_PLUGIN_DATABASE_VIRTUAL_H