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

#if !OPENRAVE_STATIC_PLUGINS

#include "openrave/openrave.h"
#include "openrave/plugin.h"
#include "openrave/plugininfo.h"
#include "plugindatabase_virtual.h"

#include <mutex>
#include <boost/shared_ptr.hpp>
#include <unordered_map>

namespace OpenRAVE {

class DynamicRaveDatabase final : public RaveDatabase, public boost::enable_shared_from_this<DynamicRaveDatabase> {
public:
    DynamicRaveDatabase();
    DynamicRaveDatabase(const DynamicRaveDatabase&) = delete; // Copying not allowed
    DynamicRaveDatabase(DynamicRaveDatabase&&) = default;
    ~DynamicRaveDatabase() override;

    void Init() override; ///< Initializes by identifying environment variables and loading paths from $OPENRAVE_PLUGINS, then loads plugins

    void ReloadPlugins() override;
    bool LoadPlugin(const std::string& libraryname) override;

private:
    struct DynamicLibrary final
    {
        DynamicLibrary(const std::string& path);
        DynamicLibrary(const DynamicLibrary&) = delete;
        DynamicLibrary(DynamicLibrary&&);
        ~DynamicLibrary() noexcept;
        operator bool() const noexcept
        {
            return !!_handle;
        }
        void* LoadSymbol(const std::string& name, std::string& errstr) const;
    private:
        void* _handle;
    };

    void _LoadPluginsFromPath(const std::string&, bool recurse = false);
    bool _LoadPlugin(const std::string&); ///< Attempts to load a RavePlugin from a shared object, fails liberally if the right symbols cannot be found. Locks _mutex.

    std::vector<std::string> _vPluginDirs; ///< List of plugin directories
    std::unordered_map<std::string, DynamicLibrary> _mapLibraryHandles; ///< A map of paths to *open* shared object handles.
};

} // end namespace OpenRAVE

#endif // !OPENRAVE_STATIC_PLUGINS

#endif // RAVE_PLUGIN_DATABASE_H