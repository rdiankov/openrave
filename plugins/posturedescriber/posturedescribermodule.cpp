// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Guangning Tan, Kei Usui, Rosen Diankov <rosen.diankov@gmail.com>
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
#include "posturedescriberinterface.h" // PostureDescriberBasePtr
#include "posturedescribermodule.h" // PostureDescriberModule

namespace OpenRAVE {

using ManipulatorPtr = RobotBase::ManipulatorPtr;
using LinkPtr = RobotBase::LinkPtr;

PostureDescriberModule::PostureDescriberModule(const EnvironmentBasePtr& penv) : ModuleBase(penv)
{
    __description =
        ":Module Author: Guangning Tan & Kei Usui & Rosen Diankov\n\n"
        "Loads a robot posture describer onto a kinematics chain, either in form of a (base link, end-effector link) pair, or in form of a manipulator";

    // `SendCommand` APIs
    this->RegisterCommand("GetInterfaceName",
                          boost::bind(&PostureDescriberModule::_GetInterfaceNameCommand, this, _1, _2),
                          "Gets the interface name of the posture describer plugin");
    this->RegisterCommand("SetInterfaceName",
                          boost::bind(&PostureDescriberModule::_SetInterfaceNameCommand, this, _1, _2),
                          "Sets the interface name of the posture describer plugin");

    // `SendJSONCommand` APIs
    this->RegisterJSONCommand("LoadPostureDescriber",
                              boost::bind(&PostureDescriberModule::_LoadPostureDescriberJSONCommand, this, _1, _2, _3),
                              "Loads a robot posture describer onto a (base link, end-effector link) pair, or onto a manipulator that prescribes the pair");
}

bool PostureDescriberModule::_SetInterfaceNameCommand(std::ostream& ssout, std::istream& ssin) {
    ssin >> _interfacename;
    return !!ssin;
}

bool PostureDescriberModule::_GetInterfaceNameCommand(std::ostream& ssout, std::istream& ssin) const {
    ssout << _interfacename;
    return true;
}

bool PostureDescriberModule::_LoadPostureDescriberJSONCommand(const rapidjson::Value& input,
                                                              rapidjson::Value& output,
                                                              rapidjson::Document::AllocatorType& allocator) {
    const EnvironmentBasePtr penv = GetEnv();
    const int envId = penv->GetId();

    if(input.HasMember("interfacename")) {
        _interfacename = input["interfacename"].GetString();
    }

    // check robot
    if(!input.HasMember("robotname")) {
        RAVELOG_WARN_FORMAT("env=%d, Rapidjson input has no robotname", envId);
        return false;
    }
    const std::string robotname = input["robotname"].GetString();
    const RobotBasePtr probot = penv->GetRobot(robotname);
    if(probot == nullptr) {
        RAVELOG_WARN_FORMAT("env=%d has no robot %s", envId % robotname);
        return false;
    }

    // check manipulator, or link pair    
    const bool bUseManip = input.HasMember("manipname");
    const std::string manipname = bUseManip ? input["manipname"].GetString() : std::string();
    std::string baselinkname, eelinkname;
    if(bUseManip) {
        if(input.HasMember("baselinkname")) {
            baselinkname = input["baselinkname"].GetString();
            RAVELOG_WARN_FORMAT("We have manipulator \"%s\", so ignore baselinkname %s", manipname % baselinkname);
        }
        if(input.HasMember("endeffectorlinkname")) {
            eelinkname = input["endeffectorlinkname"].GetString();
            RAVELOG_WARN_FORMAT("We have manipulator \"%s\", so ignore endeffectorlinkname %s", manipname % eelinkname);
        }
    }
    else if((input.HasMember("baselinkname") && input.HasMember("endeffectorlinkname"))) {
        baselinkname = input["baselinkname"].GetString();
        eelinkname = input["endeffectorlinkname"].GetString();
    }
    else {
        RAVELOG_WARN("We have neither manipulator nor the (baselinkname, endeffectorlinkname) pair");
        return false;
    }
    
    if(bUseManip) {
        const ManipulatorPtr pmanip = probot->GetManipulator(manipname);
        if(pmanip == nullptr) {
            RAVELOG_WARN_FORMAT("env=%d, robot %s has no manipulator %s", envId % robotname % manipname);
            return false;
        }
        const PostureDescriberBasePtr pDescriber = RaveCreatePostureDescriber(penv, _interfacename);
        if(pDescriber == nullptr) {
            RAVELOG_WARN_FORMAT("env=%d, cannot create robot posture describer interface %s for robot %s, manipulator %s",
                                            envId % _interfacename % robotname % manipname);
            return false;
        }
        if(!pDescriber->Init(pmanip)) {
            RAVELOG_WARN_FORMAT("env=%d, cannot initialize robot posture describer for robot %s, manipulator %s", envId % robotname % manipname);
            return false;
        }
        if(!probot->SetPostureDescriber(pmanip, pDescriber)) {
            RAVELOG_WARN_FORMAT("env=%d, cannot set robot posture describer for robot %s, manipulator %s", envId % robotname % manipname);
            return false;
        }
    }
    else {
        const LinkPtr baselink = probot->GetLink(baselinkname);
        const LinkPtr eelink = probot->GetLink(eelinkname);
        if(baselink == nullptr || eelink == nullptr) {
            RAVELOG_WARN_FORMAT("env=%d, robot %s has no link %s or %s", envId % robotname % baselinkname % eelinkname);
            return false;
        }
        const PostureDescriberBasePtr pDescriber = RaveCreatePostureDescriber(penv, _interfacename);
        if(pDescriber == nullptr) {
            RAVELOG_WARN_FORMAT("env=%d, cannot create robot posture describer interface %s for robot %s, from baselink %s to eelink %s",
                                            envId % _interfacename % robotname % baselinkname % eelinkname);
            return false;
        }

        const LinkPair kinematicsChain_init {baselink, eelink};
        const LinkPair kinematicsChain = ExtractEssentialKinematicsChain(kinematicsChain_init);
        if(kinematicsChain_init != kinematicsChain) {
            RAVELOG_INFO_FORMAT("Extracted essential kinematics chain: was from baselink \"%s\" to eelink \"%s\"; now from baselink \"%s\" to eelink \"%s\"",
                                kinematicsChain_init[0]->GetName() % kinematicsChain_init[1]->GetName() % kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName());
        }

        if(!pDescriber->Init(kinematicsChain)) {
            RAVELOG_WARN_FORMAT("env=%d, cannot initialize robot posture describer for robot %s from baselink \"%s\" to eelink \"%s\"", envId % robotname % kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName());
            return false;
        }

        if(!probot->SetPostureDescriber(kinematicsChain, pDescriber)) {
            RAVELOG_WARN_FORMAT("env=%d, cannot set robot posture describer for robot %s, from baselink %s to %s", envId % robotname % kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName());
            return false;
        }
    }

    return true;
}

} // namespace OpenRAVE

using OpenRAVE::PLUGININFO;
using OpenRAVE::PT_Module;
using OpenRAVE::PT_PostureDescriber;
using OpenRAVE::InterfaceType;
using OpenRAVE::InterfaceBasePtr;
using OpenRAVE::ModuleBasePtr;
using OpenRAVE::EnvironmentBasePtr;

// posture describer interfaces
using OpenRAVE::PostureDescriberBasePtr;
using OpenRAVE::PostureDescriber;
using OpenRAVE::PostureDescriberModule;

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
