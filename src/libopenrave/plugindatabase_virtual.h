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

#include "openrave/openrave.h"

namespace OpenRAVE {

class RegisteredInterface;

class PluginBase : public UserData
{
public:
    virtual ~PluginBase() {}
    virtual void Destroy() = 0;
    virtual bool IsValid() = 0;
    virtual const std::string& GetName() const = 0;
    virtual bool GetInfo(PLUGININFO& info) = 0;
    virtual bool HasInterface(InterfaceType type, const std::string& name) = 0;
    virtual InterfaceBasePtr CreateInterface(InterfaceType type, const std::string& name, const char* interfacehash, EnvironmentBasePtr penv) = 0;
    virtual void OnRaveInitialized() = 0;
    virtual void OnRavePreDestroy() = 0;
};

class RaveDatabase
{
public:
    friend class RegisteredInterface;
    virtual ~RaveDatabase() {}

    virtual bool Init(bool bLoadAllPlugins) = 0;
    virtual void Destroy() = 0;
    //virtual void GetPlugins(std::list<PluginPtr>& listplugins) const = 0;
    virtual InterfaceBasePtr Create(EnvironmentBasePtr penv, InterfaceType type, const std::string& _name) = 0;
    //virtual bool AddDirectory(const std::string& pdir) = 0;
    virtual void ReloadPlugins() = 0;
    virtual void OnRaveInitialized() = 0;
    virtual void OnRavePreDestroy() = 0;
    virtual bool LoadPlugin(const std::string& pluginname) = 0;
    virtual bool RemovePlugin(const std::string& pluginname) = 0;
    virtual bool HasInterface(InterfaceType type, const std::string& interfacename) = 0;
    virtual void GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins) const = 0;
    virtual void GetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames) const = 0;
    virtual UserDataPtr RegisterInterface(InterfaceType type, const std::string& name, const char* interfacehash, const char* envhash, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn) = 0;

    // Locks the mutex of this database and return the lock.
    inline std::unique_lock<std::mutex> Lock() const noexcept {
        return std::unique_lock<std::mutex>(_mutex);
    }

protected:
    mutable std::mutex _mutex; ///< changing plugin database

    std::list<boost::weak_ptr<RegisteredInterface>> _listRegisteredInterfaces;
};

struct RegisteredInterface : public UserData
{
    RegisteredInterface(InterfaceType type, const std::string& name, const boost::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)>& createfn, boost::shared_ptr<RaveDatabase> database)
        : UserData()
        , _type(type)
        , _name(name)
        , _createfn(createfn)
        , _database(database)
    {
    }

    virtual ~RegisteredInterface()
    {
        boost::shared_ptr<RaveDatabase> database = _database.lock();
        if( !!database ) {
            std::unique_lock<std::mutex> lock = database->Lock();
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

} // namespace OpenRAVE

#endif // RAVE_PLUGIN_DATABASE_VIRTUAL_H