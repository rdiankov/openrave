// -*- coding: utf-8 -*-
// Copyright (C) 2019
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
#include "libopenrave.h"
#include <boost/make_shared.hpp>

namespace OpenRAVE {

RobotBase::ConnectedBodyInfo::ConnectedBodyInfo() : _bIsActive(0)
{
}

void RobotBase::ConnectedBodyInfo::InitInfoFromBody(RobotBase& robot)
{
    _vLinkInfos.clear();
    _vJointInfos.clear();
    _vManipulatorInfos.clear();
    _vAttachedSensorInfos.clear();
    _vGripperInfos.clear();

    // have to set to the identity before extracting info
    KinBody::KinBodyStateSaverRef statesaver(robot, Save_LinkTransformation);
    std::vector<dReal> vzeros(robot.GetDOF());
    robot.SetDOFValues(vzeros, Transform());

    FOREACH(itlink, robot._veclinks) {
        _vLinkInfos.push_back(boost::make_shared<KinBody::LinkInfo>((*itlink)->UpdateAndGetInfo()));
    }

    FOREACH(itjoint, robot._vecjoints) {
        _vJointInfos.push_back(boost::make_shared<KinBody::JointInfo>((*itjoint)->UpdateAndGetInfo()));
    }
    FOREACH(itjoint, robot._vPassiveJoints) {
        _vJointInfos.push_back(boost::make_shared<KinBody::JointInfo>((*itjoint)->UpdateAndGetInfo()));
    }

    FOREACH(itmanip, robot.GetManipulators()) {
        _vManipulatorInfos.push_back(boost::make_shared<RobotBase::ManipulatorInfo>((*itmanip)->GetInfo()));
    }

    FOREACH(itattachedsensor, robot.GetAttachedSensors()) {
        _vAttachedSensorInfos.push_back(boost::make_shared<RobotBase::AttachedSensorInfo>((*itattachedsensor)->UpdateAndGetInfo()));
    }
    FOREACH(itGripperInfo, robot.GetGripperInfos()) {
        RobotBase::GripperInfoPtr pGripperInfo(new RobotBase::GripperInfo());
        *pGripperInfo = **itGripperInfo;
        _vGripperInfos.push_back(pGripperInfo);
    }
}

void RobotBase::ConnectedBodyInfo::Reset()
{
    _id.clear();
    _name.clear();
    _linkname.clear();
    _uri.clear();
    _trelative = Transform();
    _vLinkInfos.clear();
    _vJointInfos.clear();
    _vManipulatorInfos.clear();
    _vAttachedSensorInfos.clear();
    _vGripperInfos.clear();
    _bIsActive = 0;
}

void RobotBase::ConnectedBodyInfo::SerializeJSON(rapidjson::Value &rConnectedBodyInfo, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    orjson::SetJsonValueByKey(rConnectedBodyInfo, "id", _id, allocator);
    orjson::SetJsonValueByKey(rConnectedBodyInfo, "name", _name, allocator);
    orjson::SetJsonValueByKey(rConnectedBodyInfo, "linkName", _linkname, allocator);
    if (!_uri.empty()) {
        if( options & ISO_ReferenceUriHint ) {
            orjson::SetJsonValueByKey(rConnectedBodyInfo, "uriHint", _uri, allocator);
        }
        else {
            orjson::SetJsonValueByKey(rConnectedBodyInfo, "uri", _uri, allocator);
        }
    }

    orjson::SetJsonValueByKey(rConnectedBodyInfo, "transform", _trelative, allocator);

    rapidjson::Value linkInfosValue;
    linkInfosValue.SetArray();
    FOREACH(it, _vLinkInfos)
    {
        rapidjson::Value info;
        (*it)->SerializeJSON(info, allocator, fUnitScale, options);
        linkInfosValue.PushBack(info, allocator);
    }
    if (linkInfosValue.Size() > 0) {
        rConnectedBodyInfo.AddMember("links", linkInfosValue, allocator);
    }

    rapidjson::Value jointInfosValue;
    jointInfosValue.SetArray();
    FOREACH(it, _vJointInfos)
    {
        rapidjson::Value v;
        (*it)->SerializeJSON(v, allocator, fUnitScale, options);
        jointInfosValue.PushBack(v, allocator);
    }
    if (jointInfosValue.Size()) {
        rConnectedBodyInfo.AddMember("joints", jointInfosValue, allocator);
    }

    rapidjson::Value manipulatorInfosValue;
    manipulatorInfosValue.SetArray();
    FOREACH(it, _vManipulatorInfos)
    {
        rapidjson::Value info;
        (*it)->SerializeJSON(info, allocator, fUnitScale, options);
        manipulatorInfosValue.PushBack(info, allocator);
    }
    if (manipulatorInfosValue.Size() > 0) {
        rConnectedBodyInfo.AddMember("tools", manipulatorInfosValue, allocator);
    }

    rapidjson::Value attachedSensorInfosValue;
    attachedSensorInfosValue.SetArray();
    FOREACH(it, _vAttachedSensorInfos)
    {
        rapidjson::Value info;
        (*it)->SerializeJSON(info, allocator, fUnitScale, options);
        attachedSensorInfosValue.PushBack(info, allocator);
    }
    if (attachedSensorInfosValue.Size() > 0) {
        rConnectedBodyInfo.AddMember("attachedSensors", attachedSensorInfosValue, allocator);
    }

    rapidjson::Value rGripperInfos;
    rGripperInfos.SetArray();
    FOREACH(it, _vGripperInfos)
    {
        rapidjson::Value info;
        (*it)->SerializeJSON(info, allocator, fUnitScale, options);
        rGripperInfos.PushBack(info, allocator);
    }
    if (rGripperInfos.Size() > 0) {
        rConnectedBodyInfo.AddMember("gripperInfos", rGripperInfos, allocator);
    }

    orjson::SetJsonValueByKey(rConnectedBodyInfo, "isActive", (int)_bIsActive, allocator);
}

void RobotBase::ConnectedBodyInfo::DeserializeJSON(const rapidjson::Value &value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "name", _name);
    orjson::LoadJsonValueByKey(value, "id", _id);
    if( _id.empty() ) {
        _id = _name;
    }
    orjson::LoadJsonValueByKey(value, "linkName", _linkname);

    if( !(options & IDO_IgnoreReferenceUri) ) {
        orjson::LoadJsonValueByKey(value, "uri", _uri);
    }

    orjson::LoadJsonValueByKey(value, "transform", _trelative);

    if(value.HasMember("links") && value["links"].IsArray()) {
        _vLinkInfos.reserve(value["links"].Size() + _vLinkInfos.size());
        size_t iLink = 0;
        for (rapidjson::Value::ConstValueIterator it = value["links"].Begin(); it != value["links"].End(); ++it, ++iLink) {
            const rapidjson::Value& linkValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(linkValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(linkValue, "name");
            }
            if (id.empty()) {
                id = boost::str(boost::format("link%d") % iLink);
            }
            UpdateOrCreateInfo(linkValue, id, _vLinkInfos, fUnitScale, options);
        }
    }

    if(value.HasMember("joints") && value["joints"].IsArray()) {
        _vJointInfos.reserve(value["joints"].Size() + _vJointInfos.size());
        size_t iJoint = 0;
        for (rapidjson::Value::ConstValueIterator it = value["joints"].Begin(); it != value["joints"].End(); ++it, ++iJoint) {
            const rapidjson::Value& jointValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(jointValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(jointValue, "name");
            }
            if (id.empty()) {
                id = boost::str(boost::format("joint%d") % iJoint);
            }
            UpdateOrCreateInfo(jointValue, id, _vJointInfos, fUnitScale, options);
        }
    }

    if(value.HasMember("tools") && value["tools"].IsArray()) {
        _vManipulatorInfos.reserve(value["tools"].Size() + _vManipulatorInfos.size());
        size_t iManipualtor = 0;
        for (rapidjson::Value::ConstValueIterator it = value["tools"].Begin(); it != value["tools"].End(); ++it, ++iManipualtor) {
            const rapidjson::Value& manipulatorValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(manipulatorValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(manipulatorValue, "name");
            }
            if (id.empty()) {
                id = boost::str(boost::format("tool%d") % iManipualtor);
            }
            UpdateOrCreateInfo(manipulatorValue, id, _vManipulatorInfos, fUnitScale, options);
        }
    }

    if(value.HasMember("attachedSensors") && value["attachedSensors"].IsArray()) {
        _vAttachedSensorInfos.reserve(value["attachedSensors"].Size() + _vAttachedSensorInfos.size());
        size_t iAttachedSensor = 0;
        for (rapidjson::Value::ConstValueIterator it = value["attachedSensors"].Begin(); it != value["attachedSensors"].End(); ++it, ++iAttachedSensor) {
            const rapidjson::Value& attachedSensorValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(attachedSensorValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(attachedSensorValue, "name");
            }
            if (id.empty()) {
                id = boost::str(boost::format("attachedSensor%d") % iAttachedSensor);
            }
            UpdateOrCreateInfo(attachedSensorValue, id, _vAttachedSensorInfos, fUnitScale, options);
        }
    }

    if(value.HasMember("gripperInfos") && value["gripperInfos"].IsArray()) {
        _vGripperInfos.reserve(value["gripperInfos"].Size() + _vGripperInfos.size());
        size_t iGripperInfo = 0;
        for (rapidjson::Value::ConstValueIterator it = value["gripperInfos"].Begin(); it != value["gripperInfos"].End(); ++it, ++iGripperInfo) {
            const rapidjson::Value& gripperInfoValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(gripperInfoValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(gripperInfoValue, "name");
            }
            if (id.empty()) {
                id = boost::str(boost::format("gripperInfo%d") % iGripperInfo);
            }
            UpdateOrCreateInfo(gripperInfoValue, id, _vGripperInfos, fUnitScale, options);
        }
    }

    orjson::LoadJsonValueByKey(value, "isActive", _bIsActive);
}


bool RobotBase::ConnectedBodyInfo::operator==(const RobotBase::ConnectedBodyInfo& other) const {
    return _id == other._id
           && _name == other._name
           && _linkname == other._linkname
           && _uri == other._uri
           && _trelative == other._trelative
           && _vLinkInfos == other._vLinkInfos
           && _vJointInfos == other._vJointInfos
           && _vManipulatorInfos == other._vManipulatorInfos
           && _vAttachedSensorInfos == other._vAttachedSensorInfos
           && _vGripperInfos == other._vGripperInfos
           && _bIsActive == other._bIsActive
           && IsInfoVectorEqual(_vLinkInfos, other._vLinkInfos)
           && IsInfoVectorEqual(_vJointInfos, other._vJointInfos)
           && IsInfoVectorEqual(_vManipulatorInfos, other._vManipulatorInfos)
           && IsInfoVectorEqual(_vAttachedSensorInfos, other._vAttachedSensorInfos)
           && IsInfoVectorEqual(_vGripperInfos, other._vGripperInfos);
}

RobotBase::ConnectedBodyInfo& RobotBase::ConnectedBodyInfo::operator=(const RobotBase::ConnectedBodyInfo& other) {
    _id = other._id;
    _name = other._name;
    _linkname = other._linkname;
    _uri = other._uri;
    _trelative = other._trelative;
    _vLinkInfos.resize(other._vLinkInfos.size());
    for (size_t iLink = 0; iLink < other._vLinkInfos.size(); iLink++) {
        _vLinkInfos[iLink].reset(new LinkInfo(*other._vLinkInfos[iLink]));
    }
    _vJointInfos.resize(other._vJointInfos.size());
    for (size_t iJoint = 0; iJoint < other._vJointInfos.size(); iJoint++) {
        _vJointInfos[iJoint].reset(new JointInfo(*other._vJointInfos[iJoint]));
    }
    _vManipulatorInfos.resize(other._vManipulatorInfos.size());
    for (size_t iManipulator = 0; iManipulator < other._vManipulatorInfos.size(); iManipulator++) {
        _vManipulatorInfos[iManipulator].reset(new ManipulatorInfo(*other._vManipulatorInfos[iManipulator]));
    }
    _vAttachedSensorInfos.resize(other._vAttachedSensorInfos.size());
    for (size_t iAttachedSensor = 0; iAttachedSensor < other._vAttachedSensorInfos.size(); iAttachedSensor++) {
        _vAttachedSensorInfos[iAttachedSensor].reset(new AttachedSensorInfo(*other._vAttachedSensorInfos[iAttachedSensor]));
    }
    _vGripperInfos.resize(other._vGripperInfos.size());
    for (size_t iGripperInfo = 0; iGripperInfo < other._vGripperInfos.size(); iGripperInfo++) {
        _vGripperInfos[iGripperInfo].reset(new GripperInfo(*other._vGripperInfos[iGripperInfo]));
    }
    _bIsActive = other._bIsActive;
    return *this;
}


RobotBase::ConnectedBody::ConnectedBody(RobotBasePtr probot) : _pattachedrobot(probot)
{
}

RobotBase::ConnectedBody::ConnectedBody(RobotBasePtr probot, const RobotBase::ConnectedBodyInfo &info)
    : _info(info), _pattachedrobot(probot)
{
    if (!!probot) {
        LinkPtr attachedLink = probot->GetLink(_info._linkname);
        if( !attachedLink ) {
            throw OPENRAVE_EXCEPTION_FORMAT("Link \"%s\" to which ConnectedBody %s is attached does not exist in robot %s", info._linkname%GetName()%probot->GetName(), ORE_InvalidArguments);
        }
        _pattachedlink = attachedLink;
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT("Valid robot is not given for ConnectedBody %s", GetName(), ORE_InvalidArguments);
    }
}


RobotBase::ConnectedBody::ConnectedBody(RobotBasePtr probot, const ConnectedBody &connectedBody, int cloningoptions)
{
    *this = connectedBody;
    _pDummyJointCache = probot->GetJoint(_dummyPassiveJointName);
    FOREACH(itinfo, _vResolvedLinkNames) {
        itinfo->second = probot->GetLink(itinfo->first);
    }
    FOREACH(itinfo, _vResolvedJointNames) {
        itinfo->second = probot->GetJoint(itinfo->first);
    }
    FOREACH(itinfo, _vResolvedManipulatorNames) {
        itinfo->second = probot->GetManipulator(itinfo->first);
    }
    FOREACH(itinfo, _vResolvedAttachedSensorNames) {
        itinfo->second = probot->GetAttachedSensor(itinfo->first);
    }
    FOREACH(itinfo, _vResolvedGripperInfoNames) {
        itinfo->second = probot->GetGripperInfo(itinfo->first);
    }
    _pattachedrobot = probot;
    _pattachedlink = probot->GetLink(LinkPtr(connectedBody._pattachedlink)->GetName());
}

RobotBase::ConnectedBody::~ConnectedBody()
{
}

bool RobotBase::ConnectedBody::SetActive(int8_t active)
{
    if (_info._bIsActive == active) {
        return false;
    }

    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        if( pattachedrobot->_nHierarchyComputed != 0 ) {
            // robot is already added, check to see if its state is getting in the way of changing the active state. right now -1 and 1 both enable the robot
            if( (_info._bIsActive == 0) != (active == 0) ) {
                throw OPENRAVE_EXCEPTION_FORMAT("Cannot set ConnectedBody %s active to %s since robot %s is still in the environment", _info._name%(int)active%pattachedrobot->GetName(), ORE_InvalidState);
            }
        }
    }
    _info._bIsActive = active;
    return true; // changed
}

int8_t RobotBase::ConnectedBody::IsActive()
{
    return _info._bIsActive;
}

void RobotBase::ConnectedBody::SetLinkEnable(bool benable)
{
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        std::vector<uint8_t> enablestates;
        pattachedrobot->GetLinkEnableStates(enablestates);
        bool bchanged = false;

        FOREACH(itlinkname, _vResolvedLinkNames) {
            KinBody::LinkPtr plink = pattachedrobot->GetLink(itlinkname->first);
            if( !!plink ) {
                if( enablestates.at(plink->GetIndex()) != benable ) {
                    enablestates.at(plink->GetIndex()) = benable;
                    bchanged = true;
                }
            }
        }

        if( bchanged ) {
            pattachedrobot->SetLinkEnableStates(enablestates);
        }
    }
}

void RobotBase::ConnectedBody::SetLinkVisible(bool bvisible)
{
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        FOREACH(itlinkname, _vResolvedLinkNames) {
            KinBody::LinkPtr plink = pattachedrobot->GetLink(itlinkname->first);
            if( !!plink ) {
                plink->SetVisible(bvisible);
            }
        }
    }
}

void RobotBase::ConnectedBody::GetResolvedLinks(std::vector<KinBody::LinkPtr>& links)
{
    links.resize(_vResolvedLinkNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t ilink = 0; ilink < _vResolvedLinkNames.size(); ++ilink) {
            links[ilink] = pattachedrobot->GetLink(_vResolvedLinkNames[ilink].first);
        }
    }
    else {
        FOREACH(itlink, links) {
            itlink->reset();
        }
    }
}

void RobotBase::ConnectedBody::GetResolvedJoints(std::vector<KinBody::JointPtr>& joints)
{
    joints.resize(_vResolvedJointNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t ijoint = 0; ijoint < _vResolvedJointNames.size(); ++ijoint) {
            joints[ijoint] = pattachedrobot->GetJoint(_vResolvedJointNames[ijoint].first);
        }
    }
    else {
        FOREACH(itjoint, joints) {
            itjoint->reset();
        }
    }
}

KinBody::JointPtr RobotBase::ConnectedBody::GetResolvedDummyPassiveJoint()
{
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        return pattachedrobot->GetJoint(_dummyPassiveJointName);
    }
    return KinBody::JointPtr();
}

void RobotBase::ConnectedBody::GetResolvedManipulators(std::vector<RobotBase::ManipulatorPtr>& manipulators)
{
    manipulators.resize(_vResolvedManipulatorNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t imanipulator = 0; imanipulator < _vResolvedManipulatorNames.size(); ++imanipulator) {
            manipulators[imanipulator] = pattachedrobot->GetManipulator(_vResolvedManipulatorNames[imanipulator].first);
        }
    }
    else {
        FOREACH(itmanipulator, manipulators) {
            itmanipulator->reset();
        }
    }
}

void RobotBase::ConnectedBody::GetResolvedAttachedSensors(std::vector<RobotBase::AttachedSensorPtr>& attachedSensors)
{
    attachedSensors.resize(_vResolvedAttachedSensorNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t iattachedSensor = 0; iattachedSensor < _vResolvedAttachedSensorNames.size(); ++iattachedSensor) {
            attachedSensors[iattachedSensor] = pattachedrobot->GetAttachedSensor(_vResolvedAttachedSensorNames[iattachedSensor].first);
        }
    }
    else {
        FOREACH(itattachedSensor, attachedSensors) {
            itattachedSensor->reset();
        }
    }
}

void RobotBase::ConnectedBody::GetResolvedGripperInfos(std::vector<RobotBase::GripperInfoPtr>& gripperInfos)
{
    gripperInfos.resize(_vResolvedGripperInfoNames.size());
    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        for(size_t igripperInfo = 0; igripperInfo < _vResolvedGripperInfoNames.size(); ++igripperInfo) {
            gripperInfos[igripperInfo] = pattachedrobot->GetGripperInfo(_vResolvedGripperInfoNames[igripperInfo].first);
        }
    }
    else {
        FOREACH(itgripperInfo, gripperInfos) {
            itgripperInfo->reset();
        }
    }
}

void RobotBase::ConnectedBody::ExtractInfo(RobotBase::ConnectedBodyInfo& info) const
{
    info = _info;
}

UpdateFromInfoResult RobotBase::ConnectedBody::UpdateFromInfo(const RobotBase::ConnectedBodyInfo& info)
{
    BOOST_ASSERT(info._id == _info._id);
    // name
    if (_info._name != info._name) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // linkname
    if (_info._linkname != info._linkname) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _trelative
    if (GetRelativeTransform() != info._trelative) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vLinkInfos, links has orders, so only compare one by one
    std::vector<KinBody::LinkPtr> links;
    GetResolvedLinks(links);
    if (links.size() != info._vLinkInfos.size()) {
        return UFIR_RequireRemoveFromEnvironment;
    }
    for(size_t iLink = 0; iLink < links.size(); iLink++) {
        KinBody::LinkInfo linkInfo;
        links[iLink]->ExtractInfo(linkInfo);  // might be slow
        if (linkInfo != (*info._vLinkInfos[iLink])) {
            return UFIR_RequireRemoveFromEnvironment;
        }
    }

    // _vJointInfos
    std::vector<KinBody::JointPtr> joints;
    GetResolvedJoints(joints);
    std::vector<KinBody::JointInfoPtr> jointInfos;
    jointInfos.reserve(joints.size());
    for(std::vector<KinBody::JointPtr>::iterator itJoint = joints.begin(); itJoint != joints.end(); itJoint++) {
        KinBody::JointInfoPtr pJointInfo(new KinBody::JointInfo());
        (*itJoint)->ExtractInfo(*pJointInfo);  // might be slow
        jointInfos.push_back(pJointInfo);
    }
    std::vector<KinBody::JointInfoPtr> diffJointInfo;
    GetInfoVectorDiff(jointInfos, info._vJointInfos, diffJointInfo);
    if (diffJointInfo.size() > 0) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vManipulatorInfos
    std::vector<RobotBase::ManipulatorPtr> manipulators;
    GetResolvedManipulators(manipulators);

    std::vector<RobotBase::ManipulatorInfoPtr> manipulatorInfos;
    manipulatorInfos.reserve(manipulators.size());
    for(std::vector<RobotBase::ManipulatorPtr>::iterator itManip = manipulators.begin(); itManip != manipulators.end(); itManip++) {
        RobotBase::ManipulatorInfoPtr pManipInfo(new RobotBase::ManipulatorInfo());
        (*itManip)->ExtractInfo(*pManipInfo);
        manipulatorInfos.push_back(pManipInfo);
    }
    std::vector<RobotBase::ManipulatorInfoPtr> diffManipInfo;
    GetInfoVectorDiff(manipulatorInfos, info._vManipulatorInfos, diffManipInfo);

    if (diffManipInfo.size() > 0) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vAttachedSensorInfos
    std::vector<RobotBase::AttachedSensorPtr> attachedSensors;
    GetResolvedAttachedSensors(attachedSensors);
    std::vector<RobotBase::AttachedSensorInfoPtr> attachedSensorInfos;
    attachedSensorInfos.reserve(attachedSensors.size());
    for(std::vector<RobotBase::AttachedSensorPtr>::iterator itAttachedSensor = attachedSensors.begin(); itAttachedSensor != attachedSensors.end(); itAttachedSensor++) {
        RobotBase::AttachedSensorInfoPtr pAttachedSensorInfo(new RobotBase::AttachedSensorInfo());
        (*itAttachedSensor)->ExtractInfo(*pAttachedSensorInfo);
        attachedSensorInfos.push_back(pAttachedSensorInfo);
    }
    std::vector<RobotBase::AttachedSensorInfoPtr> diffAttachedSensorInfo;
    GetInfoVectorDiff(attachedSensorInfos, info._vAttachedSensorInfos, diffAttachedSensorInfo);

    if (diffAttachedSensorInfo.size() > 0) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vGripperInfos
    std::vector<RobotBase::GripperInfoPtr> gripperInfos;
    GetResolvedGripperInfos(gripperInfos);
    std::vector<RobotBase::GripperInfoPtr> diffGripperInfo;
    GetInfoVectorDiff(gripperInfos, info._vGripperInfos, diffGripperInfo);
    if (diffGripperInfo.size() > 0) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _bIsActive
    if (IsActive() != info._bIsActive) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    return UFIR_Success;
}

bool RobotBase::ConnectedBody::CanProvideManipulator(const std::string& resolvedManipulatorName) const
{
    if( _info._vManipulatorInfos.size() == 0 ) {
        return false;
    }
    if( resolvedManipulatorName.size() <= _nameprefix.size() ) {
        return false;
    }
    if( strncmp(resolvedManipulatorName.c_str(), _nameprefix.c_str(), _nameprefix.size()) != 0 ) {
        return false;
    }

    const char* pStartCheckName = resolvedManipulatorName.c_str() + _nameprefix.size();
    int nCheckNameLength = resolvedManipulatorName.size() - _nameprefix.size();
    //std::string submanipname = resolvedManipulatorName.substr(_nameprefix.size());
    FOREACH(itmanip, _info._vManipulatorInfos) {
        const RobotBase::ManipulatorInfo& manipinfo = **itmanip;
        if( (int)manipinfo._name.size() == nCheckNameLength && strncmp(manipinfo._name.c_str(), pStartCheckName, nCheckNameLength) == 0 ) {
            return true;
        }
    }

    return false;

}

const std::string& RobotBase::ConnectedBody::GetInfoHash() const
{
    // _info currently is only set from constructor, so we don't need to invalidate __hashinfo yet
    // isActive is ignored in the _info
    if (__hashinfo.size() == 0) {
        rapidjson::Document doc;
        dReal fUnitScale = 1.0;
        int options = 0;
        _info.SerializeJSON(doc, doc.GetAllocator(), fUnitScale, options);
        // set isActive to -1 so that its state does not affect the hash
        orjson::SetJsonValueByKey(doc, "isActive", -1, doc.GetAllocator());
        __hashinfo = utils::GetMD5HashString(orjson::DumpJson(doc));
    }
    return __hashinfo;
}

RobotBase::ConnectedBodyPtr RobotBase::AddConnectedBody(const RobotBase::ConnectedBodyInfo& connectedBodyInfo, bool removeduplicate)
{
    if( _nHierarchyComputed != 0 ) {
        throw OPENRAVE_EXCEPTION_FORMAT("Cannot add connected body while robot %s is added to the environment", GetName(), ORE_InvalidState);
    }

    OPENRAVE_ASSERT_OP(connectedBodyInfo._name.size(),>,0);
    int iremoveindex = -1;
    for(int iconnectedbody = 0; iconnectedbody < (int)_vecConnectedBodies.size(); ++iconnectedbody) {
        if( _vecConnectedBodies[iconnectedbody]->GetName() == connectedBodyInfo._name ) {
            if( removeduplicate ) {
                iremoveindex = iconnectedbody;
                break;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("attached sensor with name %s already exists"),connectedBodyInfo._name,ORE_InvalidArguments);
            }
        }
    }
    ConnectedBodyPtr newConnectedBody(new ConnectedBody(shared_robot(),connectedBodyInfo));
//    if( _nHierarchyComputed ) {
//        newConnectedBody->_ComputeInternalInformation();
//    }
    if( iremoveindex >= 0 ) {
        // replace the old one
        _vecConnectedBodies[iremoveindex] = newConnectedBody;
    }
    else {
        _vecConnectedBodies.push_back(newConnectedBody);
    }
    //newConnectedBody->UpdateInfo(); // just in case
    __hashrobotstructure.resize(0);
    return newConnectedBody;
}

RobotBase::ConnectedBodyPtr RobotBase::GetConnectedBody(const std::string& name) const
{
    FOREACHC(itconnectedbody, _vecConnectedBodies) {
        if( (*itconnectedbody)->GetName() == name ) {
            return *itconnectedbody;
        }
    }
    return RobotBase::ConnectedBodyPtr();
}

bool RobotBase::RemoveConnectedBody(RobotBase::ConnectedBody &connectedBody)
{
    FOREACH(itconnectedBody,_vecConnectedBodies) {
        if( itconnectedBody->get() == &connectedBody ) {
            _vecConnectedBodies.erase(itconnectedBody);
            __hashrobotstructure.clear();
            return true;
        }
    }
    return false;
}

/// \brief Match field names with matchFieldSuffix with case insensitivity
bool MatchFieldsCaseInsensitive(const char* pfieldname, const std::string& matchFieldSuffix)
{
    if( !pfieldname ) {
        return false;
    }

    int fieldlength = strlen(pfieldname);
    if( fieldlength < (int)matchFieldSuffix.size() ) {
        return false;
    }

    return _strnicmp(pfieldname + (fieldlength - (int)matchFieldSuffix.size()), matchFieldSuffix.c_str(), matchFieldSuffix.size()) == 0;
}

typedef boost::function<bool (const char*)> FieldMatcher;

/// \brief recursive looks for field names that match with fieldMatcherFn and sets a new prefixed value.
void RecursivePrefixMatchingField(const std::string& nameprefix, const FieldMatcher& fieldMatcherFn, rapidjson::Value& rValue, rapidjson::Document::AllocatorType& allocator, bool bIsMatching)
{
    switch (rValue.GetType()) {
    case rapidjson::kObjectType: {
        for (rapidjson::Value::MemberIterator it = rValue.MemberBegin(); it != rValue.MemberEnd(); ++it) {
            bool bSubIsMatching = fieldMatcherFn(it->name.GetString());
            RecursivePrefixMatchingField(nameprefix, fieldMatcherFn, it->value, allocator, bSubIsMatching);
        }
        break;
    }
    case rapidjson::kArrayType: {
        for (rapidjson::Value::ValueIterator it = rValue.Begin(); it != rValue.End(); ++it) {
            RecursivePrefixMatchingField(nameprefix, fieldMatcherFn, *it, allocator, bIsMatching);
        }
        break;
    }
    case rapidjson::kStringType: {
        if( bIsMatching ) {
            std::string newname = nameprefix + std::string(rValue.GetString());
            rValue.SetString(newname.c_str(), allocator);
        }
        break;
    }
    case rapidjson::kTrueType:
    case rapidjson::kFalseType:
    case rapidjson::kNumberType:
    case rapidjson::kNullType:
        // skip
        break;
    default: {
        RAVELOG_WARN_FORMAT("unsupported JSON type: %s", orjson::DumpJson(rValue));
    }
    }
}

void RobotBase::_ComputeConnectedBodiesInformation()
{
    // resolve duplicate names for links and joints in connected body info
    // reinitialize robot with combined infos
    if (_vecConnectedBodies.empty()) {
        return;
    }

    // should have already done adding the necessary link etc
    // during cloning, we should not add links and joints again
    if (_nHierarchyComputed != 0) {
        return;
    }

    FOREACH(itconnectedBody, _vecConnectedBodies) {
        ConnectedBody& connectedBody = **itconnectedBody;
        const ConnectedBodyInfo& connectedBodyInfo = connectedBody._info;

        if( !connectedBody.GetAttachingLink() ) {
            throw OPENRAVE_EXCEPTION_FORMAT("ConnectedBody %s for robot %s does not have a valid pointer to link %s", connectedBody.GetName()%GetName()%connectedBodyInfo._linkname, ORE_InvalidArguments);
        }

        Transform tBaseLinkInWorld = connectedBody.GetTransform(); // transform all links and joints by this

        if( connectedBody.GetName().size() == 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT("ConnectedBody %s attached to link %s has no name initialized", connectedBodyInfo._uri%connectedBodyInfo._linkname, ORE_InvalidArguments);
        }

        vector<ConnectedBodyPtr>::iterator itconnectedBody2 = itconnectedBody; ++itconnectedBody2;
        for(; itconnectedBody2 != _vecConnectedBodies.end(); ++itconnectedBody2) {
            if( connectedBody.GetName() == (*itconnectedBody2)->GetName() ) {
                throw OPENRAVE_EXCEPTION_FORMAT("robot %s has two ConnectedBody with the same name %s!", GetName()%connectedBody.GetName(), ORE_InvalidArguments);
            }
        }

        connectedBody._nameprefix = connectedBody.GetName() + "_";
        if( connectedBody.IsActive() == 0 ) {
            // skip
            continue;
        }

        if( connectedBodyInfo._vLinkInfos.size() == 0 ) {
            RAVELOG_WARN_FORMAT("ConnectedBody %s for robot %s has no link infos, so cannot add anything", connectedBody.GetName()%GetName());
            continue;
        }

        // check if connectedBodyInfo._linkname exists
        bool bExists = false;
        FOREACH(ittestlink, _veclinks) {
            if( connectedBodyInfo._linkname == (*ittestlink)->GetName() ) {
                bExists = true;
                break;
            }
        }
        if( !bExists ) {
            throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, the attaching link '%s' on robot does not exist!", connectedBody.GetName()%GetName()%connectedBodyInfo._linkname, ORE_InvalidArguments);
        }

        // Links
        connectedBody._vResolvedLinkNames.resize(connectedBodyInfo._vLinkInfos.size());
        for(int ilink = 0; ilink < (int)connectedBodyInfo._vLinkInfos.size(); ++ilink) {
            KinBody::LinkPtr& plink = connectedBody._vResolvedLinkNames[ilink].second;
            if( !plink ) {
                plink.reset(new KinBody::Link(shared_kinbody()));
            }
            plink->_info = *connectedBodyInfo._vLinkInfos[ilink]; // copy
            plink->_info._name = connectedBody._nameprefix + plink->_info._name;
            plink->_info._t = tBaseLinkInWorld * plink->_info._t;
            _InitAndAddLink(plink);
            connectedBody._vResolvedLinkNames[ilink].first = plink->_info._name;
        }

        // Joints
        std::vector<KinBody::JointPtr> vNewJointsToAdd;
        std::vector<std::pair<std::string, std::string> > jointNamePairs;
        connectedBody._vResolvedJointNames.resize(connectedBodyInfo._vJointInfos.size());
        for(int ijoint = 0; ijoint < (int)connectedBodyInfo._vJointInfos.size(); ++ijoint) {
            KinBody::JointPtr& pjoint = connectedBody._vResolvedJointNames[ijoint].second;
            if( !pjoint ) {
                pjoint.reset(new KinBody::Joint(shared_kinbody()));
            }
            pjoint->_info = *connectedBodyInfo._vJointInfos[ijoint]; // copy
            pjoint->_info._name = connectedBody._nameprefix + pjoint->_info._name;

            // search for the correct resolved _linkname0 and _linkname1
            bool bfoundlink0 = false, bfoundlink1 = false;
            for(size_t ilink = 0; ilink < connectedBodyInfo._vLinkInfos.size(); ++ilink) {
                if( pjoint->_info._linkname0 == connectedBodyInfo._vLinkInfos[ilink]->_name ) {
                    pjoint->_info._linkname0 = connectedBody._vResolvedLinkNames.at(ilink).first;
                    bfoundlink0 = true;
                }
                if( pjoint->_info._linkname1 == connectedBodyInfo._vLinkInfos[ilink]->_name ) {
                    pjoint->_info._linkname1 = connectedBody._vResolvedLinkNames.at(ilink).first;
                    bfoundlink1 = true;
                }
            }

            if( !bfoundlink0 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for joint %s, could not find linkname0 %s in connected body link infos!", connectedBody.GetName()%GetName()%pjoint->_info._name%pjoint->_info._linkname0, ORE_InvalidArguments);
            }
            if( !bfoundlink1 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for joint %s, could not find linkname1 %s in connected body link infos!", connectedBody.GetName()%GetName()%pjoint->_info._name%pjoint->_info._linkname1, ORE_InvalidArguments);
            }
            jointNamePairs.emplace_back(connectedBodyInfo._vJointInfos[ijoint]->_name,  pjoint->_info._name);
            vNewJointsToAdd.push_back(pjoint);
            connectedBody._vResolvedJointNames[ijoint].first = pjoint->_info._name;
        }

        FOREACH(itnewjoint, vNewJointsToAdd) {
            KinBody::JointInfo& jointinfo = (*itnewjoint)->_info;
            FOREACH(itmimic, jointinfo._vmimic) {
                if (!(*itmimic)) {
                    continue;
                }
                for (std::size_t iequation = 0; iequation < (*itmimic)->_equations.size(); ++iequation) {
                    std::string eq;
                    utils::SearchAndReplace(eq, (*itmimic)->_equations[iequation], jointNamePairs);
                    (*itmimic)->_equations[iequation] = eq;
                }
            }

            _InitAndAddJoint(*itnewjoint);
        }

        // Manipulators
        connectedBody._vResolvedManipulatorNames.resize(connectedBodyInfo._vManipulatorInfos.size());
        for(int imanipulator = 0; imanipulator < (int)connectedBodyInfo._vManipulatorInfos.size(); ++imanipulator) {
            RobotBase::ManipulatorPtr& pnewmanipulator = connectedBody._vResolvedManipulatorNames[imanipulator].second;
            if( !pnewmanipulator ) {
                pnewmanipulator.reset(new RobotBase::Manipulator(shared_robot(), *connectedBodyInfo._vManipulatorInfos[imanipulator]));
            }
            else {
                pnewmanipulator->_info = *connectedBodyInfo._vManipulatorInfos[imanipulator];
            }
            pnewmanipulator->_info._name = connectedBody._nameprefix + pnewmanipulator->_info._name;
            if( pnewmanipulator->_info._grippername.size() > 0 ) {
                pnewmanipulator->_info._grippername = connectedBody._nameprefix + pnewmanipulator->_info._grippername;
            }

            bool bHasSameTool = false;
            FOREACH(ittestmanipulator, _vecManipulators) {
                if( pnewmanipulator->_info._name == (*ittestmanipulator)->GetName() ) {
                    bHasSameTool = true;
                    break;
                }
            }

            if( bHasSameTool ) {
                RAVELOG_INFO_FORMAT("When adding ConnectedBody %s for robot %s, got resolved manipulator with same name '%s'. Perhaps trying to overwrite? For now, passing through.", connectedBody.GetName()%GetName()%pnewmanipulator->_info._name);
                pnewmanipulator.reset(); // will not be adding it
                continue;
            }

            {
                LinkPtr pArmBaseLink = !GetLinks().empty() ? GetLinks()[0] : LinkPtr();
                if( !pArmBaseLink ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for manipulator %s, could not find a base link of the robot.", connectedBody.GetName()%GetName()%pnewmanipulator->_info._name, ORE_InvalidArguments);
                }
                pnewmanipulator->_info._sBaseLinkName = pArmBaseLink->_info._name;
            }

            // search for the correct resolved _sEffectorLinkName
            bool bFoundEffectorLink = false;
            for(size_t ilink = 0; ilink < connectedBodyInfo._vLinkInfos.size(); ++ilink) {
                if( pnewmanipulator->_info._sEffectorLinkName == connectedBodyInfo._vLinkInfos[ilink]->_name ) {
                    pnewmanipulator->_info._sEffectorLinkName = connectedBody._vResolvedLinkNames.at(ilink).first;
                    bFoundEffectorLink = true;
                }
            }

            if( !bFoundEffectorLink ) {
                throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for manipulator %s, could not find linkname1 %s in connected body link infos!", connectedBody.GetName()%GetName()%pnewmanipulator->_info._name%pnewmanipulator->_info._sEffectorLinkName, ORE_InvalidArguments);
            }

            // search for the correct resolved joint name
            for(size_t iGripperJoint = 0; iGripperJoint < pnewmanipulator->_info._vGripperJointNames.size(); ++iGripperJoint) {
                std::string& gripperJointName = pnewmanipulator->_info._vGripperJointNames[iGripperJoint];
                bool bFoundJoint = false;
                for(size_t ijoint = 0; ijoint < connectedBodyInfo._vJointInfos.size(); ++ijoint) {
                    if( gripperJointName == connectedBodyInfo._vJointInfos[ijoint]->_name ) {
                        gripperJointName = connectedBody._vResolvedJointNames.at(ijoint).first;
                        bFoundJoint = true;
                    }
                }

                if( !bFoundJoint ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for Manipulator %s, could not find joint %s in connected body joint infos!", connectedBody.GetName()%GetName()%pnewmanipulator->_info._name%gripperJointName, ORE_InvalidArguments);
                }
            }

            _vecManipulators.push_back(pnewmanipulator);
            connectedBody._vResolvedManipulatorNames[imanipulator].first = pnewmanipulator->_info._name;
        }

        // AttachedSensors
        connectedBody._vResolvedAttachedSensorNames.resize(connectedBodyInfo._vAttachedSensorInfos.size());
        for(int iattachedsensor = 0; iattachedsensor < (int)connectedBodyInfo._vAttachedSensorInfos.size(); ++iattachedsensor) {
            RobotBase::AttachedSensorPtr& pnewattachedSensor = connectedBody._vResolvedAttachedSensorNames[iattachedsensor].second;
            if( !pnewattachedSensor ) {
                pnewattachedSensor.reset(new RobotBase::AttachedSensor(shared_robot(), *connectedBodyInfo._vAttachedSensorInfos[iattachedsensor]));
            }
            else {
                pnewattachedSensor->_info = *connectedBodyInfo._vAttachedSensorInfos[iattachedsensor];
            }
            pnewattachedSensor->_info._name = connectedBody._nameprefix + pnewattachedSensor->_info._name;

            FOREACH(ittestattachedSensor, _vecAttachedSensors) {
                if( pnewattachedSensor->_info._name == (*ittestattachedSensor)->GetName() ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, got resolved attachedSensor with same name %s!", connectedBody.GetName()%GetName()%pnewattachedSensor->_info._name, ORE_InvalidArguments);
                }
            }

            // search for the correct resolved _linkname and _sEffectorLinkName
            bool bFoundLink = false;
            for(size_t ilink = 0; ilink < connectedBodyInfo._vLinkInfos.size(); ++ilink) {
                if( pnewattachedSensor->_info._linkname == connectedBodyInfo._vLinkInfos[ilink]->_name ) {
                    pnewattachedSensor->_info._linkname = connectedBody._vResolvedLinkNames.at(ilink).first;
                    bFoundLink = true;
                }
            }

            if( !bFoundLink ) {
                throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for attachedSensor %s, could not find linkname0 %s in connected body link infos!", connectedBody.GetName()%GetName()%pnewattachedSensor->_info._name%pnewattachedSensor->_info._linkname, ORE_InvalidArguments);
            }

            _vecAttachedSensors.push_back(pnewattachedSensor);
            connectedBody._vResolvedAttachedSensorNames[iattachedsensor].first = pnewattachedSensor->_info._name;
        }

        // GripperInfos
        connectedBody._vResolvedGripperInfoNames.resize(connectedBodyInfo._vGripperInfos.size());
        for(int iGripperInfo = 0; iGripperInfo < (int)connectedBodyInfo._vGripperInfos.size(); ++iGripperInfo) {
            RobotBase::GripperInfoPtr& pnewgripperInfo = connectedBody._vResolvedGripperInfoNames[iGripperInfo].second;
            if( !pnewgripperInfo ) {
                pnewgripperInfo.reset(new RobotBase::GripperInfo());
            }
            *pnewgripperInfo = *connectedBodyInfo._vGripperInfos[iGripperInfo];
            pnewgripperInfo->name = connectedBody._nameprefix + pnewgripperInfo->name;

            FOREACH(ittestgripperInfo, _vecGripperInfos) {
                if( pnewgripperInfo->name == (*ittestgripperInfo)->name ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, got resolved gripperInfo with same name %s!", connectedBody.GetName()%GetName()%pnewgripperInfo->name, ORE_InvalidArguments);
                }
            }

            // search for the correct resolved joint name
            for(size_t iGripperJoint = 0; iGripperJoint < pnewgripperInfo->gripperJointNames.size(); ++iGripperJoint) {
                std::string& gripperJointName = pnewgripperInfo->gripperJointNames[iGripperJoint];
                bool bFoundJoint = false;
                for(size_t ijoint = 0; ijoint < connectedBodyInfo._vJointInfos.size(); ++ijoint) {
                    if( gripperJointName == connectedBodyInfo._vJointInfos[ijoint]->_name ) {
                        gripperJointName = connectedBody._vResolvedJointNames.at(ijoint).first;
                        bFoundJoint = true;
                    }
                }

                if( !bFoundJoint ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, for gripperInfo %s, could not find joint %s in connected body joint infos!", connectedBody.GetName()%GetName()%pnewgripperInfo->name%gripperJointName, ORE_InvalidArguments);
                }
            }

            // look recursively for fields that end in "linkname" (case insensitive) and resolve their names
            if(connectedBodyInfo._vGripperInfos[iGripperInfo]->_docGripperInfo.IsObject()) {
                rapidjson::Document newGripperInfoDoc;
                newGripperInfoDoc.CopyFrom(connectedBodyInfo._vGripperInfos[iGripperInfo]->_docGripperInfo, newGripperInfoDoc.GetAllocator());
                RecursivePrefixMatchingField(connectedBody._nameprefix, boost::bind(MatchFieldsCaseInsensitive, _1, std::string("linkname")), newGripperInfoDoc, newGripperInfoDoc.GetAllocator(), false);
                RecursivePrefixMatchingField(connectedBody._nameprefix, boost::bind(MatchFieldsCaseInsensitive, _1, std::string("linknames")), newGripperInfoDoc, newGripperInfoDoc.GetAllocator(), false);
                RecursivePrefixMatchingField(connectedBody._nameprefix, boost::bind(MatchFieldsCaseInsensitive, _1, std::string("links")), newGripperInfoDoc, newGripperInfoDoc.GetAllocator(), false);
                pnewgripperInfo->_docGripperInfo.Swap(newGripperInfoDoc);
            }
            else {
                pnewgripperInfo->_docGripperInfo.Clear();
            }

            _vecGripperInfos.push_back(pnewgripperInfo);
            connectedBody._vResolvedGripperInfoNames[iGripperInfo].first = pnewgripperInfo->name;
        }

        connectedBody._dummyPassiveJointName = connectedBody._nameprefix + "_dummyconnectedbody__";

        if( !connectedBody._pDummyJointCache ) {
            connectedBody._pDummyJointCache.reset(new KinBody::Joint(shared_kinbody()));
        }
        KinBody::JointInfo& dummyJointInfo = connectedBody._pDummyJointCache->_info;
        dummyJointInfo._name = connectedBody._dummyPassiveJointName;
        dummyJointInfo._bIsActive = false;
        dummyJointInfo._type = KinBody::JointType::JointPrismatic;
        dummyJointInfo._vmaxaccel[0] = 0.0;
        dummyJointInfo._vmaxvel[0] = 0.0;
        dummyJointInfo._vupperlimit[0] = 0;

        dummyJointInfo._linkname0 = connectedBodyInfo._linkname;
        dummyJointInfo._linkname1 = connectedBody._vResolvedLinkNames.at(0).first;
        _InitAndAddJoint(connectedBody._pDummyJointCache);
    }
}

void RobotBase::_DeinitializeConnectedBodiesInformation()
{
    if (_vecConnectedBodies.empty()) {
        return;
    }

    std::vector<uint8_t> vConnectedLinks; vConnectedLinks.resize(_veclinks.size(),0);
    std::vector<uint8_t> vConnectedJoints; vConnectedJoints.resize(_vecjoints.size(),0);
    std::vector<uint8_t> vConnectedPassiveJoints; vConnectedPassiveJoints.resize(_vPassiveJoints.size(),0);

    // have to remove any of the added links
    FOREACH(itconnectedBody, _vecConnectedBodies) {
        ConnectedBody& connectedBody = **itconnectedBody;

        // unfortunately cannot save info as easily since have to restore origin and names
        for(int iresolvedlink = 0; iresolvedlink < (int)connectedBody._vResolvedLinkNames.size(); ++iresolvedlink) {
            LinkPtr presolvedlink = GetLink(connectedBody._vResolvedLinkNames[iresolvedlink].first);
            if( !!presolvedlink ) {
                vConnectedLinks.at(presolvedlink->GetIndex()) = 1;
            }
            connectedBody._vResolvedLinkNames[iresolvedlink].first.clear();
        }

        for(int iresolvedjoint = 0; iresolvedjoint < (int)connectedBody._vResolvedJointNames.size(); ++iresolvedjoint) {
            for(int ijointindex = 0; ijointindex < (int)_vecjoints.size(); ++ijointindex) {
                if( _vecjoints[ijointindex]->GetName() == connectedBody._vResolvedJointNames[iresolvedjoint].first ) {
                    vConnectedJoints[ijointindex] = 1;
                }
            }
            for(int ijointindex = 0; ijointindex < (int)_vPassiveJoints.size(); ++ijointindex) {
                if( _vPassiveJoints[ijointindex]->GetName() == connectedBody._vResolvedJointNames[iresolvedjoint].first ) {
                    vConnectedPassiveJoints[ijointindex] = 1;
                }
            }
            connectedBody._vResolvedJointNames[iresolvedjoint].first.clear();
        }

        for(int iresolvedmanipulator = 0; iresolvedmanipulator < (int)connectedBody._vResolvedManipulatorNames.size(); ++iresolvedmanipulator) {
            ManipulatorPtr presolvedmanipulator = GetManipulator(connectedBody._vResolvedManipulatorNames[iresolvedmanipulator].first);
            if( !!presolvedmanipulator ) {
                RemoveManipulator(presolvedmanipulator);
            }
            connectedBody._vResolvedManipulatorNames[iresolvedmanipulator].first.clear();
        }

        for(int iresolvedattachedSensor = 0; iresolvedattachedSensor < (int)connectedBody._vResolvedAttachedSensorNames.size(); ++iresolvedattachedSensor) {
            AttachedSensorPtr presolvedattachedSensor = GetAttachedSensor(connectedBody._vResolvedAttachedSensorNames[iresolvedattachedSensor].first);
            if( !!presolvedattachedSensor ) {
                RemoveAttachedSensor(*presolvedattachedSensor);
            }
            connectedBody._vResolvedAttachedSensorNames[iresolvedattachedSensor].first.clear();
        }

        for(int iresolvedGripperInfo = 0; iresolvedGripperInfo < (int)connectedBody._vResolvedGripperInfoNames.size(); ++iresolvedGripperInfo) {
            GripperInfoPtr presolvedGripperInfo = GetGripperInfo(connectedBody._vResolvedGripperInfoNames[iresolvedGripperInfo].first);
            if( !!presolvedGripperInfo ) {
                RemoveGripperInfo(presolvedGripperInfo->name);
            }
            connectedBody._vResolvedGripperInfoNames[iresolvedGripperInfo].first.clear();
        }

        for(int ijointindex = 0; ijointindex < (int)_vPassiveJoints.size(); ++ijointindex) {
            if( _vPassiveJoints[ijointindex]->GetName() == connectedBody._dummyPassiveJointName ) {
                vConnectedPassiveJoints[ijointindex] = 1;
            }
        }
        connectedBody._dummyPassiveJointName.clear();
    }

    int iwritelink = 0;
    for(int ireadlink = 0; ireadlink < (int)vConnectedLinks.size(); ++ireadlink) {
        if( !vConnectedLinks[ireadlink] ) {
            // preserve as original
            _veclinks[iwritelink++] = _veclinks[ireadlink];
        }
    }
    _veclinks.resize(iwritelink);

    int iwritejoint = 0;
    for(int ireadjoint = 0; ireadjoint < (int)vConnectedJoints.size(); ++ireadjoint) {
        if( !vConnectedJoints[ireadjoint] ) {
            // preserve as original
            _vecjoints[iwritejoint++] = _vecjoints[ireadjoint];
        }
    }
    _vecjoints.resize(iwritejoint);

    int iwritepassiveJoint = 0;
    for(int ireadpassiveJoint = 0; ireadpassiveJoint < (int)vConnectedPassiveJoints.size(); ++ireadpassiveJoint) {
        if( !vConnectedPassiveJoints[ireadpassiveJoint] ) {
            // preserve as original
            _vPassiveJoints[iwritepassiveJoint++] = _vPassiveJoints[ireadpassiveJoint];
        }
    }
    _vPassiveJoints.resize(iwritepassiveJoint);
}

void RobotBase::GetConnectedBodyActiveStates(std::vector<int8_t>& activestates) const
{
    activestates.resize(_vecConnectedBodies.size());
    for(size_t iconnectedbody = 0; iconnectedbody < _vecConnectedBodies.size(); ++iconnectedbody) {
        activestates[iconnectedbody] = _vecConnectedBodies[iconnectedbody]->IsActive();
    }
}

void RobotBase::SetConnectedBodyActiveStates(const std::vector<int8_t>& activestates)
{
    OPENRAVE_ASSERT_OP(activestates.size(),==,_vecConnectedBodies.size());
    for(size_t iconnectedbody = 0; iconnectedbody < _vecConnectedBodies.size(); ++iconnectedbody) {
        _vecConnectedBodies[iconnectedbody]->SetActive(activestates[iconnectedbody]);
    }
}

} // end namespace OpenRAVE
