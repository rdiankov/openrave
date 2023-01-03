// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu), Carnegie Mellon University
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
#include "pqprave.h"

#include "plugindefs.h"
#include "collisionPQP.h"

const std::string PQPRavePlugin::_pluginname = "PQPRavePlugin";

PQPRavePlugin::PQPRavePlugin()
{
    _interfaces[OpenRAVE::PT_CollisionChecker].push_back("pqp");
}

PQPRavePlugin::~PQPRavePlugin() {}
 
OpenRAVE::InterfaceBasePtr PQPRavePlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case OpenRAVE::PT_CollisionChecker:
        if( interfacename == "pqp")
            return boost::make_shared<CollisionCheckerPQP>(penv);
        break;
    default:
        break;
    }
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& PQPRavePlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& PQPRavePlugin::GetPluginName() const
{
    return _pluginname;
}

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new PQPRavePlugin();
}