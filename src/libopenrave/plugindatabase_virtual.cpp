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

#include "plugindatabase_virtual.h"

namespace OpenRAVE {

void RaveDatabase::Destroy()
{
    std::lock_guard<std::mutex> lock(_mutex);
    while (!_vPlugins.empty()) {
        _vPlugins.back()->Destroy();
        _vPlugins.pop_back();
    }
}

void RaveDatabase::OnRaveInitialized()
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const PluginPtr& pluginPtr : _vPlugins) {
        if (pluginPtr) {
            pluginPtr->OnRaveInitialized();
        }
    }
}

void RaveDatabase::OnRavePreDestroy()
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const PluginPtr& pluginPtr : _vPlugins) {
        if (pluginPtr) {
            pluginPtr->OnRavePreDestroy();
        }
    }
}

bool RaveDatabase::HasInterface(InterfaceType type, const std::string& interfacename) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const PluginPtr& plugin : _vPlugins) {
        if (plugin->HasInterface(type, interfacename)) {
            return true;
        }
    }
    return false;
}

void RaveDatabase::GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& pluginInfo) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const PluginPtr& entry : _vPlugins) {
        PLUGININFO info;
        info.interfacenames = entry->GetInterfaces();
        info.version = entry->GetOpenRAVEVersion();
        pluginInfo.emplace_back(std::make_pair(entry->GetPluginPath(), std::move(info)));
    }
}

void RaveDatabase::GetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string>>& interfacenames) const
{
    interfacenames.clear();
    std::lock_guard<std::mutex> lock(_mutex);
    for (const PluginPtr& plugin : _vPlugins) {
        const RavePlugin::InterfaceMap& interfaces = plugin->GetInterfaces();
        for (const std::pair<const InterfaceType, std::vector<std::string>>& entry : interfaces) {
            interfacenames[entry.first].insert(interfacenames[entry.first].end(), entry.second.begin(), entry.second.end());
        }
    }
}

UserDataPtr RaveDatabase::AddVirtualPlugin(InterfaceType type, std::string name, std::function<InterfaceBasePtr(EnvironmentBasePtr, std::istream&)> createfn)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _vPlugins.emplace_back(boost::make_shared<VirtualPlugin>(type, std::move(name), std::move(createfn)));
    return _vPlugins.back();
}

InterfaceBasePtr RaveDatabase::Create(EnvironmentBasePtr penv, InterfaceType type, std::string name)
{
    InterfaceBasePtr pointer;
    if (name.empty()) {
        switch(type) {
        case PT_KinBody: {
            pointer.reset(new KinBody(PT_KinBody,penv));
            pointer->__strxmlid = ""; // don't set to KinBody since there's no officially registered interface
            break;
        }
        case PT_PhysicsEngine: name = "GenericPhysicsEngine"; break;
        case PT_CollisionChecker: name = "GenericCollisionChecker"; break;
        case PT_Robot: name = "GenericRobot"; break;
        case PT_Trajectory: name = "GenericTrajectory"; break;
        default: break;
        }
    }

    if (!pointer) {
        // Some plugins have its creation parameters in the string after its name
        size_t position = std::min<size_t>(name.find_first_of(' '), name.size());
        if (position == 0) {
            RAVELOG_WARN_FORMAT("interface %s name \"%s\" needs to start with a valid character\n", RaveGetInterfaceName(type) % name);
            return InterfaceBasePtr();
        }
        std::string interfacename = name.substr(0, position);
        std::vector<PluginPtr> vPlugins;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            vPlugins = _vPlugins; // Copy
        }
        for (const PluginPtr& plugin : vPlugins) {
            if (plugin->HasInterface(type, interfacename)) {
                try {
                    pointer = plugin->OpenRAVECreateInterface(type, name, RaveGetInterfaceHash(type), OPENRAVE_ENVIRONMENT_HASH, penv);
                } catch (const std::exception& e) {
                    RAVELOG_WARN_FORMAT("Failed to create interface from %s at %s", plugin->GetPluginName() % plugin->GetPluginPath());
                    plugin->AddBadInterface(type, name); // Bad interface, no cookie
                }
                if (pointer) {
                    if (pointer->GetInterfaceType() != type) {
                        RAVELOG_FATAL_FORMAT("plugin interface name %s, type %s, types do not match\n", name%RaveGetInterfaceName(type));
                        plugin->AddBadInterface(type, interfacename);
                        pointer.reset();
                    } else {
                        pointer->__strpluginname = plugin->GetPluginPath(); // __internal__ if it's a virtual plugin
                        pointer->__strxmlid = name;
                        break;
                    }
                }
            }
        }
    }

    // Some extra checks on validity of Robot objects
    if (pointer && type == PT_Robot) {
        RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pointer);
        //if ( strncmp(probot->GetKinBodyHash(), OPENRAVE_KINBODY_HASH) != 0 ) {}
        if ( !probot->IsRobot() ) {
            RAVELOG_FATAL_FORMAT("interface Robot, name %s should have IsRobot() return true", name);
            pointer.reset();
        }
    }

    if (!pointer) {
        RAVELOG_WARN_FORMAT("env=%d failed to create name %s, interface %s\n", penv->GetId()%name%RaveGetInterfaceNamesMap().find(type)->second);
        return pointer;
    }

    if (pointer->GetInterfaceType() == type) {
        // No-op, this is correct
    } else if ((pointer->GetInterfaceType() == PT_Robot) && (type == PT_KinBody)) {
        // Special case: Robots are also KinBodies.
        // No-op, this is correct
    } else {
        // Return an empty pointer; behaviour inherited from `RaveInterfaceCast`
        pointer.reset();
    }
    return pointer;
}

} // namespace OpenRAVE