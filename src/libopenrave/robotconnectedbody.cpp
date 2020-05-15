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

RobotBase::ConnectedBodyInfo::ConnectedBodyInfo() : _bIsActive(false)
{
}

void RobotBase::ConnectedBodyInfo::InitInfoFromBody(RobotBase& robot)
{
    _vLinkInfos.clear();
    _vJointInfos.clear();
    _vManipulatorInfos.clear();
    _vAttachedSensorInfos.clear();

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
}

RobotBase::ConnectedBody::ConnectedBody(OpenRAVE::RobotBasePtr probot) : _pattachedrobot(probot)
{
}

RobotBase::ConnectedBody::ConnectedBody(OpenRAVE::RobotBasePtr probot, const OpenRAVE::RobotBase::ConnectedBodyInfo &info)
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


RobotBase::ConnectedBody::ConnectedBody(OpenRAVE::RobotBasePtr probot, const ConnectedBody &connectedBody, int cloningoptions)
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
    _pattachedrobot = probot;
    _pattachedlink = probot->GetLink(LinkPtr(connectedBody._pattachedlink)->GetName());
}

RobotBase::ConnectedBody::~ConnectedBody()
{
}

bool RobotBase::ConnectedBody::SetActive(bool active)
{
    if (_info._bIsActive == active) {
        return false;
    }

    RobotBasePtr pattachedrobot = _pattachedrobot.lock();
    if( !!pattachedrobot ) {
        if( pattachedrobot->_nHierarchyComputed != 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT("Cannot set ConnectedBody %s active to %s since robot %s is still in the environment", _info._name%active%pattachedrobot->GetName(), ORE_InvalidState);
        }
    }
    _info._bIsActive = active;
    return true; // changed
}

bool RobotBase::ConnectedBody::IsActive()
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
            throw OPENRAVE_EXCEPTION_FORMAT("ConnectedBody %s attached to link %s has no name initialized", connectedBodyInfo._url%connectedBodyInfo._linkname, ORE_InvalidArguments);
        }

        vector<ConnectedBodyPtr>::iterator itconnectedBody2 = itconnectedBody; ++itconnectedBody2;
        for(; itconnectedBody2 != _vecConnectedBodies.end(); ++itconnectedBody2) {
            if( connectedBody.GetName() == (*itconnectedBody2)->GetName() ) {
                throw OPENRAVE_EXCEPTION_FORMAT("robot %s has two ConnectedBody with the same name %s!", GetName()%connectedBody.GetName(), ORE_InvalidArguments);
            }
        }

        if( !connectedBody.IsActive() ) {
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

        connectedBody._nameprefix = connectedBody.GetName() + "_";

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

            FOREACH(ittestmanipulator, _vecManipulators) {
                if( pnewmanipulator->_info._name == (*ittestmanipulator)->GetName() ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("When adding ConnectedBody %s for robot %s, got resolved manipulator with same name %s!", connectedBody.GetName()%GetName()%pnewmanipulator->_info._name, ORE_InvalidArguments);
                }
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

void RobotBase::GetConnectedBodyActiveStates(std::vector<uint8_t>& activestates) const
{
    activestates.resize(_vecConnectedBodies.size());
    for(size_t iconnectedbody = 0; iconnectedbody < _vecConnectedBodies.size(); ++iconnectedbody) {
        activestates[iconnectedbody] = _vecConnectedBodies[iconnectedbody]->IsActive();
    }
}

void RobotBase::SetConnectedBodyActiveStates(const std::vector<uint8_t>& activestates)
{
    OPENRAVE_ASSERT_OP(activestates.size(),==,_vecConnectedBodies.size());
    for(size_t iconnectedbody = 0; iconnectedbody < _vecConnectedBodies.size(); ++iconnectedbody) {
        _vecConnectedBodies[iconnectedbody]->SetActive(!!activestates[iconnectedbody]);
    }
}

} // end namespace OpenRAVE
