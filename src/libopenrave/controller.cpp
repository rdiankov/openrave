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

ControllerBase::ControllerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Controller, penv) {
}
ControllerBase::~ControllerBase() {
}

MultiControllerBase::MultiControllerBase(EnvironmentBasePtr penv) : ControllerBase(penv) {
}

MultiControllerBase::~MultiControllerBase() {
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
