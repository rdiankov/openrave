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
#include "robotposturedescriber.h"
// #include <boost/lexical_cast.hpp>

using OpenRAVE::PLUGININFO;
using OpenRAVE::PT_Module;
using OpenRAVE::PT_ForwardKinematicsSolver;
using OpenRAVE::InterfaceType;
using OpenRAVE::InterfaceBasePtr;
using OpenRAVE::ModuleBasePtr;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::RobotBasePtr;
using LinkPtr = OpenRAVE::RobotBase::LinkPtr;

using OpenRAVE::OpenRAVEErrorCode;
using OpenRAVE::OpenRAVEErrorCode::ORE_InvalidArguments;  // 0x01

// forward kinematics
using OpenRAVE::RobotPostureDescriberBasePtr;
using OpenRAVE::RobotPostureDescriber;

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
            // take robot name, base link name, ee link name
            std::string robotname, baselinkname, eelinkname;
            sinput >> robotname >> baselinkname >> eelinkname;

            const RobotBasePtr probot = penv->GetRobot(robotname);
            if(probot == nullptr) {
                throw OPENRAVE_EXCEPTION_FORMAT("interfacename=%s, env=%d has no robot %s", interfacename % penv->GetId() % robotname, ORE_InvalidArguments);
            }
            const LinkPtr baselink = probot->GetLink(baselinkname);
            const LinkPtr eelink = probot->GetLink(eelinkname);
            if(baselink == nullptr || eelink == nullptr) {
                throw OPENRAVE_EXCEPTION_FORMAT("interfacename=%s, env=%d, robot %s has no link %s or %s", interfacename % penv->GetId() % robotname % baselinkname % eelink, ORE_InvalidArguments);   
            }

            const std::array<LinkPtr, 2> kinematicsChain {baselink, eelink};
            return RobotPostureDescriberBasePtr(new RobotPostureDescriber(penv, kinematicsChain));
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
