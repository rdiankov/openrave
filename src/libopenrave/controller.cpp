// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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

namespace OpenRAVE {

MultiController::MultiController(EnvironmentBasePtr penv) : ControllerBase(penv), _nControlTransformation(0)
{
}

MultiController::~MultiController()
{
}

bool MultiController::Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
{
    boost::mutex::scoped_lock lock(_mutex);
    _probot=robot;
    if( !_probot ) {
        return false;
    }
    _listcontrollers.clear();
    _dofindices=dofindices;
    // reverse the mapping
    _dofreverseindices.resize(_probot->GetDOF());
    FOREACH(it,_dofreverseindices) {
        *it = -1;
    }
    int index = 0;
    FOREACH(it,_dofindices) {
        _dofreverseindices.at(*it) = index++;
    }
    _vcontrollersbydofs.resize(0); _vcontrollersbydofs.resize(_dofindices.size());
    _nControlTransformation = nControlTransformation;
    _ptransformcontroller.reset();
    return true;
}

const std::vector<int>& MultiController::GetControlDOFIndices() const {
    return _dofindices;
}
int MultiController::IsControlTransformation() const {
    return _nControlTransformation;
}
RobotBasePtr MultiController::GetRobot() const {
    return _probot;
}

bool MultiController::AttachController(ControllerBasePtr controller, const std::vector<int>& dofindices, int nControlTransformation)
{
    boost::mutex::scoped_lock lock(_mutex);
    if( nControlTransformation && !!_ptransformcontroller ) {
        throw openrave_exception("controller already attached for transformation",ORE_InvalidArguments);
    }
    FOREACHC(it,dofindices) {
        if( !!_vcontrollersbydofs.at(*it) ) {
            throw openrave_exception(str(boost::format("controller already attached to dof %d")%*it));
        }
    }
    if( !controller->Init(_probot,dofindices,nControlTransformation) ) {
        return false;
    }
    if( nControlTransformation ) {
        _ptransformcontroller = controller;
    }
    FOREACHC(it,dofindices) {
        _vcontrollersbydofs.at(*it) = controller;
    }
    _listcontrollers.push_back(controller);
    return true;
}

void MultiController::RemoveController(ControllerBasePtr controller)
{
    boost::mutex::scoped_lock lock(_mutex);
    _listcontrollers.remove(controller);
    if( _ptransformcontroller == controller ) {
        _ptransformcontroller.reset();
    }
    FOREACH(it,_vcontrollersbydofs) {
        if( *it == controller ) {
            it->reset();
        }
    }
}

ControllerBasePtr MultiController::GetController(int dof) const
{
    boost::mutex::scoped_lock lock(_mutex);
    if( dof < 0 ) {
        return _ptransformcontroller;
    }
    int index=0;
    FOREACHC(it,_dofindices) {
        if( *it == dof ) {
            return _vcontrollersbydofs.at(index);
        }
        index++;
    }
    return ControllerBasePtr();
}

void MultiController::Reset(int options)
{
    boost::mutex::scoped_lock lock(_mutex);
    FOREACH(itcontroller,_listcontrollers) {
        (*itcontroller)->Reset(options);
    }
}

bool MultiController::SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
{
    boost::mutex::scoped_lock lock(_mutex);
    vector<dReal> v;
    bool bsuccess = true;
    FOREACH(itcontroller,_listcontrollers) {
        v.resize(0);
        FOREACHC(it, (*itcontroller)->GetControlDOFIndices()) {
            v.push_back(values.at(_dofreverseindices.at(*it)));
        }
        bsuccess &= (*itcontroller)->SetDesired(v,trans);
    }
    return bsuccess;
}

bool MultiController::SetPath(TrajectoryBaseConstPtr ptraj)
{
    boost::mutex::scoped_lock lock(_mutex);
    bool bsuccess = true;
    if( !ptraj ) {
        FOREACH(itcontroller,_listcontrollers) {
            bsuccess &= (*itcontroller)->SetPath(TrajectoryBaseConstPtr());
        }
    }
    else {
        if( !_ptraj ||(_ptraj->GetXMLId() != ptraj->GetXMLId())) {
            _ptraj = RaveCreateTrajectory(ptraj->GetEnv(),ptraj->GetXMLId());
        }
    }
    return bsuccess;
}

void MultiController::SimulationStep(dReal fTimeElapsed)
{
    boost::mutex::scoped_lock lock(_mutex);
    FOREACH(it,_listcontrollers) {
        (*it)->SimulationStep(fTimeElapsed);
    }
}

bool MultiController::IsDone()
{
    boost::mutex::scoped_lock lock(_mutex);
    bool bdone=true;
    FOREACH(it,_listcontrollers) {
        bdone &= (*it)->IsDone();
    }
    return bdone;
}

dReal MultiController::GetTime() const
{
    boost::mutex::scoped_lock lock(_mutex);
    dReal t = 0;
    FOREACHC(it,_listcontrollers) {
        if( it == _listcontrollers.begin() ) {
            t = (*it)->GetTime();
        }
        else {
            dReal tnew = (*it)->GetTime();
            if( RaveFabs(t-tnew) > 0.000001 ) {
                RAVELOG_WARN(str(boost::format("multi-controller time is different! %f!=%f\n")%t%tnew));
            }
            t = max(t,tnew);
        }
    }
    return t;
}

void MultiController::GetVelocity(std::vector<dReal>& vel) const
{
    boost::mutex::scoped_lock lock(_mutex);
    vel.resize(_dofindices.size());
    FOREACH(it,vel) {
        *it = 0;
    }
    vector<dReal> v;
    FOREACHC(itcontroller,_listcontrollers) {
        (*itcontroller)->GetVelocity(v);
        int index=0;
        FOREACH(it,v) {
            vel.at(_dofreverseindices.at((*itcontroller)->GetControlDOFIndices().at(index++))) = *it;
        }
    }
}

void MultiController::GetTorque(std::vector<dReal>& torque) const
{
    boost::mutex::scoped_lock lock(_mutex);
    torque.resize(_dofindices.size());
    FOREACH(it,torque) {
        *it = 0;
    }
    vector<dReal> v;
    FOREACHC(itcontroller,_listcontrollers) {
        (*itcontroller)->GetTorque(v);
        int index=0;
        FOREACH(it,v) {
            torque.at(_dofreverseindices.at((*itcontroller)->GetControlDOFIndices().at(index++))) = *it;
        }
    }
}


//void RobotBase::GetControlMaxTorques(std::vector<dReal>& maxtorques) const
//{
//    if( _nActiveDOF < 0 ) {
//        GetDOFMaxTorque(maxtorques);
//        return;
//    }
//    maxtorques.resize(GetActiveDOF());
//    if( maxtorques.size() == 0 ) {
//        return;
//    }
//    dReal* pMaxTorques = &maxtorques[0];
//
//    if( _vActiveJointIndices.size() != 0 ) {
//        GetDOFMaxTorque(_vTempRobotJoints);
//
//        FOREACHC(it, _vActiveJointIndices)
//            *pMaxTorques++ = _vTempRobotJoints[*it];
//    }
//
//    if( _nAffineDOFs == DOF_NoTransform )
//        return;
//
//    if( _nAffineDOFs & DOF_X ) *pMaxTorques++ = 0;
//    if( _nAffineDOFs & DOF_Y ) *pMaxTorques++ = 0;
//    if( _nAffineDOFs & DOF_Z ) *pMaxTorques++ = 0;
//    if( _nAffineDOFs & DOF_RotationAxis ) *pMaxTorques++ = 0;
//    else if( _nAffineDOFs & DOF_Rotation3D ) {
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//    }
//    else if( _nAffineDOFs & DOF_RotationQuat ) {
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//        *pMaxTorques++ = 0;
//    }
//}
//
//void RobotBase::SetControlTorques(const std::vector<dReal>& vtorques)
//{
//    if(_nActiveDOF < 0) {
//        SetJointTorques(vtorques, false);
//        return;
//    }
//
//    if( _vActiveJointIndices.size() > 0 ) {
//        _vTempRobotJoints.resize(GetDOF());
//        std::vector<dReal>::const_iterator ittorque = vtorques.begin();
//        FOREACHC(it, _vActiveJointIndices)
//            _vTempRobotJoints[*it] = *ittorque++;
//        SetJointTorques(_vTempRobotJoints,false);
//    }
//}

}
