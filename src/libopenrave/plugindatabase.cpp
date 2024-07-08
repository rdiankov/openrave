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
#if !OPENRAVE_STATIC_PLUGINS

#include <cstdarg>
#include <cstring>
#include <functional>
#include <mutex>

#include <openrave/openraveexception.h>
#include <openrave/logging.h>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif
#include <boost/version.hpp>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <dlfcn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#endif

#ifdef _WIN32
const char s_filesep = '\\';
const char* s_delimiter = ";";
#else
const char s_filesep = '/';
const char* s_delimiter = ":";
#endif

#include "libopenrave.h"
#include "plugindatabase.h"

namespace OpenRAVE {

static const std::string PLUGIN_EXT =
#if defined (__APPLE_CC__)
    ".dylib";
#elif defined (_WIN32)
    ".dll";
#else
    ".so";
#endif

DynamicRaveDatabase::DynamicLibrary::DynamicLibrary(const std::string& path)
{
#ifdef _WIN32
    _handle = LoadLibraryA(path.c_str());
    if( _handle == NULL ) {
        RAVELOG_WARN_FORMAT("Failed to load %s\n", path);
    }
#else
    dlerror(); // clear error
    _handle = dlopen(path.c_str(), RTLD_NOW);
    char* pstr = dlerror();
    if( pstr != NULL ) {
        RAVELOG_WARN_FORMAT("%s: %s\n", path % pstr);
        if( _handle != NULL ) {
            dlclose(_handle);
        }
        _handle = NULL;
    }
#endif
}

DynamicRaveDatabase::DynamicLibrary::DynamicLibrary(DynamicRaveDatabase::DynamicLibrary&& other)
{
    this->_handle = other._handle;
    other._handle = NULL;
}

DynamicRaveDatabase::DynamicLibrary::~DynamicLibrary() noexcept
{
    if (_handle) {
#ifdef _WIN32
        FreeLibrary((HINSTANCE)_handle);
#else
        // Eagerly closing the library handle will cause segfaults during testing as tests don't fully reset openrave.
        // The problem can be alleviated by adding RTLD_NODELETE flag in dlopen(), but this has to be done everywhere
        // (including other programs that also load dynamic libraries).
        // In order to minimize the amount of changes we have to make, we will simply omit this call.
        // It is safe anyway, as the OS maintains it's own refcount of opened libraries.
        //dlclose(_handle);
#endif
    }
}

void* DynamicRaveDatabase::DynamicLibrary::LoadSymbol(const std::string& name, std::string& errstr) const
{
#ifdef _WIN32
    return GetProcAddress((HINSTANCE)_handle, name.c_str());
#else
    dlerror(); // clear error
    void* psym = dlsym(_handle, name.c_str());
    if (psym == NULL) {
        char* pstr = dlerror();
        errstr.assign(pstr);
    }
    return psym;
#endif
}

DynamicRaveDatabase::DynamicRaveDatabase()
{
}

DynamicRaveDatabase::~DynamicRaveDatabase()
{
    Destroy();
}

void DynamicRaveDatabase::Init()
{
    const char* pOPENRAVE_PLUGINS = getenv("OPENRAVE_PLUGINS"); // getenv not thread-safe?
    std::vector<std::string> vplugindirs;
    if (!!pOPENRAVE_PLUGINS) {
        utils::TokenizeString(pOPENRAVE_PLUGINS, s_delimiter, vplugindirs);
        for (int iplugindir = vplugindirs.size() - 1; iplugindir > 0; iplugindir--) {
            int jplugindir = 0;
            for(; jplugindir < iplugindir; jplugindir++) {
                if(vplugindirs[iplugindir] == vplugindirs[jplugindir]) {
                    break;
                }
            }
            if (jplugindir < iplugindir) {
                vplugindirs.erase(vplugindirs.begin()+iplugindir);
            }
        }
    }
    else {
        RAVELOG_WARN("Failed to read environment variable OPENRAVE_PLUGINS");
    }
    bool bExists = false;
    std::string installdir = OPENRAVE_PLUGINS_INSTALL_DIR;
#ifdef HAVE_BOOST_FILESYSTEM
    if( !boost::filesystem::is_directory(boost::filesystem::path(installdir)) ) {
#ifdef _WIN32
        HKEY hkey;
        if(RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\OpenRAVE\\" OPENRAVE_VERSION_STRING), 0, KEY_QUERY_VALUE, &hkey) == ERROR_SUCCESS) {
            DWORD dwType = REG_SZ;
            CHAR szInstallRoot[4096];     // dont' take chances, it is windows
            DWORD dwSize = sizeof(szInstallRoot);
            RegQueryValueEx(hkey, TEXT("InstallRoot"), NULL, &dwType, (PBYTE)szInstallRoot, &dwSize);
            RegCloseKey(hkey);
            installdir.assign(szInstallRoot);
            installdir += str(boost::format("%cshare%copenrave-%d.%d%cplugins")%s_filesep%s_filesep%OPENRAVE_VERSION_MAJOR%OPENRAVE_VERSION_MINOR%s_filesep);
            RAVELOG_VERBOSE(str(boost::format("window registry plugin dir '%s'")%installdir));
        }
        else
#endif
        {
            RAVELOG_WARN_FORMAT("%s doesn't exist", installdir);
        }
    }
    boost::filesystem::path pluginsfilename = boost::filesystem::absolute(boost::filesystem::path(installdir));
    FOREACH(itname, vplugindirs) {
        if( pluginsfilename == boost::filesystem::absolute(boost::filesystem::path(*itname)) ) {
            bExists = true;
            break;
        }
    }
#else
    std::string pluginsfilename=installdir;
    FOREACH(itname, vplugindirs) {
        if( pluginsfilename == *itname ) {
            bExists = true;
            break;
        }
    }
#endif
    if( !bExists ) {
        vplugindirs.push_back(installdir);
    }
    for (std::string& entry : vplugindirs) {
        if (entry.empty()) {
            continue;
        }
        _vPluginDirs.emplace_back(std::move(entry));
    }
    for (const std::string& entry : _vPluginDirs) {
        RAVELOG_DEBUG_FORMAT("Looking for plugins in %s", entry);
        _LoadPluginsFromPath(entry);
    }
}

void DynamicRaveDatabase::ReloadPlugins()
{
    std::vector<PluginPtr> vPlugins;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        vPlugins = _vPlugins; // Copy
    }
    for (const PluginPtr& plugin : vPlugins) {
        _LoadPlugin(plugin->GetPluginPath());
    }
}

bool DynamicRaveDatabase::LoadPlugin(const std::string& libraryname)
{
    std::string canonicalizedLibraryname = libraryname;
#ifndef _WIN32
    if(canonicalizedLibraryname.substr(0, 3) != "lib") {
        canonicalizedLibraryname = "lib" + canonicalizedLibraryname;
    }
#endif
    if(canonicalizedLibraryname.substr(canonicalizedLibraryname.size() - PLUGIN_EXT.size()) != PLUGIN_EXT) {
        canonicalizedLibraryname += PLUGIN_EXT;
    }
    // If the canonicalizedLibraryname matches any of the existing loaded libraries, then reload it
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _vPlugins.erase(std::remove_if(_vPlugins.begin(), _vPlugins.end(), [&canonicalizedLibraryname](const PluginPtr& plugin) {
            return (plugin->GetPluginName() == canonicalizedLibraryname || plugin->GetPluginPath() == canonicalizedLibraryname);
        }), _vPlugins.end());
    }
    return _LoadPlugin(canonicalizedLibraryname);
}

void DynamicRaveDatabase::_LoadPluginsFromPath(const std::string& strpath, bool recurse) try
{
#ifdef HAVE_BOOST_FILESYSTEM
    const fs::path path(strpath);
    if (fs::is_empty(path)) {
        return;
    } else if (fs::is_directory(path)) {
#if BOOST_VERSION >= 107200
        for (const fs::directory_entry& entry : fs::directory_iterator(path, fs::directory_options::skip_permission_denied)) {
#else
        for (const fs::directory_entry& entry : fs::directory_iterator(path)) {
#endif
            if (fs::is_directory(entry) && recurse) {
                _LoadPluginsFromPath(entry.path().string(), true);
            } else {
                _LoadPluginsFromPath(entry.path().string(), false);
            }
        }
    } else if (fs::is_regular_file(path)) {
        // Check that the file has a platform-appropriate extension
        if (0 == strpath.compare(strpath.size() - PLUGIN_EXT.size(), PLUGIN_EXT.size(), PLUGIN_EXT)) {
            _LoadPlugin(path.string());
        }
    } else {
        RAVELOG_WARN_FORMAT("Path is not a valid directory or file: %s", strpath);
    }
#else
    int err = 0;
    struct stat sb;
    err = ::stat(strpath.c_str(), &sb);
    if (err != 0) {
        err = errno;
        throw std::runtime_error("Failed to stat path at " + strpath + ": " + ::strerror(err));
    }
    if (S_ISDIR(sb.st_mode)) {
        DIR* dirptr = ::opendir(strpath.c_str());
        if (dirptr == NULL) {
            err = errno;
            throw std::runtime_error("Failed to open directory at " + strpath + ": " + ::strerror(err));
        }
        struct dirent* entry = NULL;
        std::string pathstr;
        for (entry = ::readdir(dirptr); entry != NULL; entry = ::readdir(dirptr)) {
            switch (entry->d_type) {
            case DT_DIR: {
                if (!recurse) {
                    break;
                }
            }
            case DT_REG: {
                pathstr.assign(entry->d_name, sizeof(entry->d_name));
                _LoadPluginsFromPath(pathstr, recurse);
                break;
            }
            default: continue;
            }
        }
        ::closedir(dirptr);
    } else if (S_ISREG(sb.st_mode)) {
        _LoadPlugin(strpath);
    } else {
        // Not a directory or file, ignore it
    }
#endif

} catch (const std::exception& e) {
    // Some paths have permissions issues, just skip those paths.
    RAVELOG_VERBOSE_FORMAT("%s", e.what());
}

bool DynamicRaveDatabase::_LoadPlugin(const std::string& strpath)
{
    DynamicLibrary dylib(strpath);
    if (!dylib) {
        RAVELOG_DEBUG_FORMAT("Failed to load shared object %s", strpath);
        return false;
    }
    std::string errstr;
    void* psym = dylib.LoadSymbol("CreatePlugin", errstr);
    if (!psym) {
        RAVELOG_DEBUG_FORMAT("%s, might not be an OpenRAVE plugin.", errstr);
        return false;
    }
    RavePlugin* plugin = nullptr;
    try {
        plugin = reinterpret_cast<PluginExportFn_Create>(psym)();
    } catch (const std::exception& e) {
        RAVELOG_WARN_FORMAT("Failed to construct a RavePlugin from %s: %s", strpath % e.what());
    }
    if (!plugin) {
        return false;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _mapLibraryHandles.emplace(strpath, std::move(dylib)); // Keep the library handle around in case we need it
    _vPlugins.emplace_back(PluginPtr(plugin)); // Ownership passed to the shared_ptr
    _vPlugins.back()->SetPluginPath(strpath);
    RAVELOG_DEBUG_FORMAT("Found %s at %s.", _vPlugins.back()->GetPluginName() % strpath);
    return true;
}

} // namespace OpenRAVE

#endif // !OPENRAVE_STATIC_PLUGINS
