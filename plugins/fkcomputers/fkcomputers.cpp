// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Rosen Diankov <rosen.diankov@gmail.com>
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

#include <openrave/plugin.h> // OPENRAVE_PLUGIN_API
#include <openrave/fksolver.h> // RobotPostureDescriberBasePtr
#include "plugindefs.h" //  FKCOMPUTERS_MODULE_NAME, ROBOTPOSTUREDESCRIBER_MODULE_NAME 
// #include <boost/lexical_cast.hpp>

using OpenRAVE::PLUGININFO;
using OpenRAVE::PT_Module;
using OpenRAVE::PT_ForwardKinematicsSolver;
using OpenRAVE::InterfaceType;
using OpenRAVE::InterfaceBasePtr;
using OpenRAVE::ModuleBasePtr;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::RobotPostureDescriberBasePtr;

class FkComputerModule : public OpenRAVE::ModuleBase
{
public:
    FkComputerModule() = delete; // disable default constructor
    FkComputerModule(const OpenRAVE::EnvironmentBasePtr& penv) : OpenRAVE::ModuleBase(penv) {}
    virtual ~FkComputerModule() = default;
};

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{    
    switch(type) {
    case PT_ForwardKinematicsSolver: {
        if( interfacename == ROBOTPOSTUREDESCRIBER_MODULE_NAME ) {
            return RobotPostureDescriberBasePtr();
        }
        // may support other type of forward kinematics computing modules?
        break;
    }
    case PT_Module: {
        if( interfacename == FKCOMPUTERS_MODULE_NAME) {
            return ModuleBasePtr(new FkComputerModule(penv));
        }
        break;
    }
    default: {
        break;
    }
    } // end-switch type

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back(FKCOMPUTERS_MODULE_NAME);
    info.interfacenames[PT_ForwardKinematicsSolver].push_back(ROBOTPOSTUREDESCRIBER_MODULE_NAME);
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    // release static variables in this plugin
}
