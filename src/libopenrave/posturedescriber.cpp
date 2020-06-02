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

#include "libopenrave.h"
#include <openrave/posturedescriber.h>

namespace OpenRAVE {

using JointPtr = OpenRAVE::KinBody::JointPtr;
using LinkPtr = OpenRAVE::KinBody::LinkPtr;

PostureDescriberBase::PostureDescriberBase(EnvironmentBasePtr penv) : InterfaceBase(PT_PostureDescriber, penv)
{
}

PostureDescriberBase::~PostureDescriberBase() {
}

const char* PostureDescriberBase::GetHash() const {
    return OPENRAVE_POSTUREDESCRIBER_HASH;
}

bool PostureDescriberBase::Supports(const RobotBase::ManipulatorPtr& pmanip) const {
    return this->Supports(GetKinematicsChain(pmanip));
}

bool PostureDescriberBase::Init(const RobotBase::ManipulatorPtr& pmanip) {
    return this->Supports(GetKinematicsChain(pmanip)) ? this->Init(GetKinematicsChain(pmanip)) : false;
}

std::string ComputeKinematicsChainHash(const RobotBase::ManipulatorPtr& pmanip, std::vector<int>& armindices) {
    return ComputeKinematicsChainHash(GetKinematicsChain(pmanip), armindices);
}

// refer to libopenrave.h and
// void RobotBase::Manipulator::serialize(std::ostream& o, int options, IkParameterizationType iktype) const
std::string ComputeKinematicsChainHash(const LinkPair& kinematicsChain, std::vector<int>& armindices)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION); // SERIALIZATION_PRECISION = 4 from libopenrave.h 

    const RobotBase::LinkPtr& baselink = kinematicsChain[0];
    const RobotBase::LinkPtr& eelink = kinematicsChain[1];
    const int baselinkind = baselink->GetIndex();
    const int eelinkind = eelink->GetIndex();
    const KinBodyPtr probot = baselink->GetParent();

    // collect armindices
    std::vector<RobotBase::JointPtr> joints;
    probot->GetChain(baselinkind, eelinkind, joints);

    armindices.clear();
    for(const RobotBase::JointPtr& joint : joints) {
        const int dofindex = joint->GetDOFIndex();
        if(!(joint->IsStatic() || dofindex==-1)) {
            armindices.push_back(dofindex);
        }
    }
    ss << armindices.size() << " ";
    const std::set<int> indexset(begin(armindices), end(armindices));

    // due to backward compatibility issues, we have to compute the end effector transform first
    Transform tcur;
    for(const RobotBase::JointPtr& joint : joints) {
        tcur = tcur * joint->GetInternalHierarchyLeftTransform() * joint->GetInternalHierarchyRightTransform();
    }
    // treat it like 6D transform IK by not inlucding the local tool transform!
    SerializeRound(ss, tcur);
    tcur = Transform();
    int index = 0;
    for(const RobotBase::JointPtr& joint : joints) {
        if( !joint->IsStatic() ) {
            ss << joint->GetType() << " ";
        }

        const int dofindex = joint->GetDOFIndex();
        if( dofindex >= 0 && indexset.count(dofindex) ) {
            tcur = tcur * joint->GetInternalHierarchyLeftTransform();
            for(int idof = 0; idof < joint->GetDOF(); ++idof) {
                SerializeRound3(ss, tcur.trans);
                SerializeRound3(ss, tcur.rotate(joint->GetInternalHierarchyAxis(idof)));
            }
            tcur = tcur * joint->GetInternalHierarchyRightTransform();
        }
        else {
            // not sure if this is correct for a mimic joint...
            tcur = tcur * joint->GetInternalHierarchyLeftTransform() * joint->GetInternalHierarchyRightTransform();
            if( joint->IsMimic() ) {
                for(int idof = 0; idof < joint->GetDOF(); ++idof) {
                    if( joint->IsMimic(idof) ) {
                        ss << "mimic " << index << " ";
                        for(int ieq = 0; ieq < 3; ++ieq) {
                            ss << joint->GetMimicEquation(idof, ieq) << " ";
                        }
                    }
                }
            }
        }
        index += 1;
    }

    const std::string chainhash = OpenRAVE::utils::GetMD5HashString(ss.str()); // kinematics chain hash
    return chainhash;
}

LinkPair GetKinematicsChainHelper(const RobotBase::ManipulatorConstPtr& pmanip) {
    // links subject to later change
    const LinkPtr baselink = pmanip->GetBase();
    const LinkPtr eelink = pmanip->GetEndEffector();
    const int baselinkind = baselink->GetIndex();
    const int eelinkind = eelink->GetIndex();

    const KinBodyPtr probot = baselink->GetParent();
    std::vector<JointPtr> joints;
    probot->GetChain(baselinkind, eelinkind, joints);

    std::string typestr;
    for(std::vector<JointPtr>::iterator it = begin(joints); it != end(joints); ) {
        if((*it)->IsStatic() || (*it)->GetDOFIndex()==-1) {
            it = joints.erase(it);
        }
        else {
            if((*it)->IsPrismatic(0)) {
                typestr.push_back('P');
            }
            else if((*it)->IsRevolute(0)){
                typestr.push_back('R');
            }
            ++it;
        }
    }
    const size_t firstR = typestr.find_first_of('R');
    if(firstR == std::string::npos) {
        RAVELOG_WARN("No revolute joints at all");
        joints.clear();
        return {nullptr, nullptr};
    }
    const size_t lastR = typestr.find_last_of('R');
    joints = std::vector<JointPtr>(begin(joints) + firstR, begin(joints) + lastR + 1);
    return {joints[0]->GetHierarchyParentLink(), joints.back()->GetHierarchyChildLink()};

}

LinkPair GetKinematicsChain(const RobotBase::ManipulatorPtr& pmanip) {
    return GetKinematicsChainHelper(pmanip);
}

LinkPair GetKinematicsChain(const RobotBase::ManipulatorConstPtr& pmanip) {
    return GetKinematicsChainHelper(pmanip);
}

} // namespace OpenRAVE
