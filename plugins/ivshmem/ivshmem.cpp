// -*- coding: utf-8 -*-
// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
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
#include "ivshmem.h"
#include "plugindefs.h"
#include "ivshmem_interface.hpp"

const std::string IVShMem::_pluginname = "ivshmem";

IVShMem::IVShMem()
{
    _interfaces[OpenRAVE::PT_CollisionChecker].push_back("ivshmem");
}

IVShMem::~IVShMem() {}

OpenRAVE::InterfaceBasePtr IVShMem::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if( type == OpenRAVE::PT_CollisionChecker && interfacename == "ivshmem" ) {
        return boost::make_shared<ivshmem::IVShMemInterface>(penv, sinput);
    }
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& IVShMem::GetInterfaces() const
{
    return _interfaces;
}

const std::string& IVShMem::GetPluginName() const
{
    return _pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new IVShMem();
}