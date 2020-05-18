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
#include <openrave/posturedescriber.h> // PostureDescriberBasePtr
#include "plugindefs.h" //  POSTUREDESCRIBER_MODULE_NAME, POSTUREDESCRIBER_CLASS_NAME
#include "posturedescriber.h"
#include "posturedescribermodule.h"

// #include <boost/lexical_cast.hpp>

using OpenRAVE::PLUGININFO;
using OpenRAVE::PT_Module;
using OpenRAVE::PT_PostureDescriber;
using OpenRAVE::InterfaceType;
using OpenRAVE::InterfaceBasePtr;
using OpenRAVE::ModuleBasePtr;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::RobotBasePtr;
using ManipulatorPtr = OpenRAVE::RobotBase::ManipulatorPtr;
using LinkPtr = OpenRAVE::RobotBase::LinkPtr;

using OpenRAVE::OpenRAVEErrorCode;
using OpenRAVE::OpenRAVEErrorCode::ORE_InvalidArguments;  // 0x01

// forward kinematics
using OpenRAVE::PostureDescriberBasePtr;
using OpenRAVE::PostureDescriber;
using OpenRAVE::PostureDescriberModule;

namespace OpenRAVE {

PostureDescriberModule::PostureDescriberModule(const EnvironmentBasePtr& penv) : ModuleBase(penv)
{
    __description =
        ":Interface Author: Guangning Tan & Kei Usui & Rosen Diankov\n\n"
        "Loads a robot posture describer onto a (base link, end-effector link) pair, or onto a manipulator that prescribes the pair";

    // `SendCommand` APIs
    this->RegisterCommand("LoadPostureDescriber",
                          boost::bind(&PostureDescriberModule::_LoadPostureDescriberCommand, this, _1, _2),
                          "Loads a robot posture describer onto a (base link, end-effector link) pair, or onto a manipulator that prescribes the pair");
}

bool PostureDescriberModule::_LoadPostureDescriberCommand(std::ostream& ssout, std::istream& ssin) {
    std::string robotname, manipname, baselinkname, eelinkname;
    ssin >> robotname >> manipname;

    const EnvironmentBasePtr penv = GetEnv();
    const int envId = penv->GetId();
    const RobotBasePtr probot = penv->GetRobot(robotname);
    if(probot == nullptr) {
        RAVELOG_WARN_FORMAT("env=%d has no robot %s", envId % robotname);
        return false;
    }

    const ManipulatorPtr pmanip = probot->GetManipulator(manipname);
    LinkPtr baselink, eelink;
    if(pmanip == nullptr) {
        baselinkname = manipname; // manipname is in fact the baselink's name
        manipname = ""; // reset
        ssin >> eelinkname;
        baselink = probot->GetLink(baselinkname);
        eelink = probot->GetLink(eelinkname);
    }
    else {
        baselink = pmanip->GetBase();
        eelink = pmanip->GetEndEffector();
        baselinkname = baselink->GetName();
        eelinkname = eelink->GetName();
    }

    if(baselink == nullptr || eelink == nullptr) {
        RAVELOG_WARN_FORMAT("env=%d, robot %s has no link %s or %s", envId % robotname % baselinkname % eelinkname);
        return false;
    }

    const PostureDescriberBasePtr probotposture = RaveCreateFkSolver(penv, POSTUREDESCRIBER_CLASS_NAME);
    if(probotposture == nullptr) {
        RAVELOG_WARN_FORMAT("env=%d, cannot create robot posture describer for robot %s from links %s to %s (manipname=\"%s\")", envId % robotname % baselinkname % eelinkname % manipname);
        return false;
    }

    const std::array<LinkPtr, 2> kinematicsChain {baselink, eelink};
    if(!probotposture->Init(kinematicsChain)) {
        RAVELOG_WARN_FORMAT("env=%d, cannot initialize robot posture describer for robot %s from links %s to %s (manipname=\"%s\")", envId % robotname % baselinkname % eelinkname % manipname);
    }

    const bool bset = probot->SetPostureDescriber(kinematicsChain, probotposture);
    if(!bset) {
        RAVELOG_WARN_FORMAT("env=%d, cannot set robot posture describer for robot %s from links %s to %s (manipname=\"%s\")", envId % robotname % baselinkname % eelinkname % manipname);
        return false;
    }

    return true;
}

} // namespace OpenRAVE

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& ssin, EnvironmentBasePtr penv)
{    
    switch(type) {
    case PT_PostureDescriber: {
        if( interfacename == POSTUREDESCRIBER_CLASS_NAME ) {
            return PostureDescriberBasePtr(new PostureDescriber(penv));
        }
        break;
    }
    case PT_Module: {
        if( interfacename == POSTUREDESCRIBER_MODULE_NAME) {
            return ModuleBasePtr(new PostureDescriberModule(penv));
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
    info.interfacenames[PT_Module].push_back(POSTUREDESCRIBER_MODULE_NAME);
    info.interfacenames[PT_PostureDescriber].push_back(POSTUREDESCRIBER_CLASS_NAME);
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    // release static variables in this plugin
}
