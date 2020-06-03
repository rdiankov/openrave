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

#include <openrave/openrave.h>
#include <openrave/openravejson.h> // openravejson
#include "posturedescriberinterface.h" // PostureDescriber
#include "plugindefs.h" // POSTUREDESCRIBER_CLASS_NAME
#include "openraveplugindefs.h" // SerializeValues

// #define POSTUREDESCRIBER_DEBUG

namespace OpenRAVE {

using JointPtr = OpenRAVE::KinBody::JointPtr;
using LinkPtr = OpenRAVE::KinBody::LinkPtr;

PostureDescriber::PostureDescriber(const EnvironmentBasePtr& penv,
                                   const double fTol) :
    PostureDescriberBase(penv),
    _fTol(fTol)
{
    __description =
        ":Interface Author: Guangning Tan & Kei Usui & Rosen Diankov\n\n"
        "Posture describer for a kinematics chain using the geometry of joint axes and anchors";

    // `SendCommand` APIs
    this->RegisterCommand("SetPostureValueThreshold",
                          boost::bind(&PostureDescriber::_SetPostureValueThresholdCommand, this, _1, _2),
                          "Sets the tolerance for determining whether a robot posture value is close to 0 and hence would have hybrid states");

    this->RegisterCommand("GetPostureValueThreshold",
                          boost::bind(&PostureDescriber::_GetPostureValueThresholdCommand, this, _1, _2),
                          "Gets the tolerance for determining whether a robot posture value is close to 0 and hence would have hybrid states");

    this->RegisterCommand("GetArmIndices",
                          boost::bind(&PostureDescriber::_GetArmIndicesCommand, this, _1, _2),
                          "Gets the shared object library name for computing the robot posture values and states");

    this->RegisterCommand("GetSupportType",
                          boost::bind(&PostureDescriber::_GetSupportTypeCommand, this, _1, _2),
                          "Gets the robot posture support type");

    // `SendJSONCommand` APIs
    this->RegisterJSONCommand("Interpret",
                              boost::bind(&PostureDescriber::_InterpretJSONCommand, this, _1, _2, _3),
                              "Interpret a single posture state");
}

PostureDescriber::~PostureDescriber() {
}

/// \brief Checks whether joints match their expected joint types
/// \return true the joint types match
bool CheckJointTypes(const std::vector<JointPtr>& joints, const std::vector<KinBody::JointType>& jointtypes) {
    const size_t njoints = joints.size();
    if(njoints != jointtypes.size()) {
        throw OPENRAVE_EXCEPTION_FORMAT("Size mismatch: %d != %d", joints.size() % jointtypes.size(), ORE_InvalidArguments);
    }
    for(size_t i = 0; i < njoints; ++i) {
        if(joints[i]->GetType() != jointtypes[i]) {
            return false;
        }
    }
    return true;
}

/// \brief Derives the relation between joint axes of two consecutive joints using the transform between them
/// \return a NeighbouringTwoJointsRelation enum for the joint axes' relation
NeighbouringTwoJointsRelation AnalyzeTransformBetweenNeighbouringJoints(const Transform& t, const double tol = 2e-15) {
    // tol was increased for densowave-VS087A4-AV6
    const Vector zaxis0(0, 0, 1); // z-axis of the first joint
    const Vector zaxis1 = t.rotate(zaxis0); // z-axis of the second joint
    const double dotprod = zaxis1.dot3(zaxis0);

    NeighbouringTwoJointsRelation o = NeighbouringTwoJointsRelation::NTJR_Unknown;
    if(1.0 - fabs(dotprod) <= tol) {
        o |= NeighbouringTwoJointsRelation::NTJR_Parallel; // TO-DO: check overlapping
        if(zaxis0.cross(t.trans).lengthsqr3() <= tol) {
            o |= NeighbouringTwoJointsRelation::NTJR_Intersect;
        }
    }
    else {
        // not parallel
        if (fabs(dotprod) <= tol) {
            o |= NeighbouringTwoJointsRelation::NTJR_Perpendicular;
        }
        if(fabs(zaxis0.cross(zaxis1).dot3(t.trans)) <= tol) {
            o |= NeighbouringTwoJointsRelation::NTJR_Intersect;
        }
    }

#if defined(POSTUREDESCRIBER_DEBUG)
    std::stringstream ss;
    ss << std::setprecision(16);
    ss << "o = " << static_cast<int>(o) << ", t = " << t << ", dotprod = " << dotprod;
    RAVELOG_VERBOSE_FORMAT("%s", ss.str());
#endif // defined(POSTUREDESCRIBER_DEBUG)
    return o;
}

/// \brief Derives the robot posture support type for a sequence of joints along a kinematics chain
/// \return a RobotPostureSupportType enum for the kinematics chain
    // const ref
RobotPostureSupportType DeriveRobotPostureSupportType(const std::vector<JointPtr>& joints) {
    const size_t njoints = joints.size();
    const std::vector<KinBody::JointType> jointtypes(njoints, KinBody::JointType::JointRevolute); ///< so far these are all revolute
    switch(njoints) {
        case 6: {
            if(!CheckJointTypes(joints, jointtypes)) {
                RAVELOG_WARN("Not all 6 joints are purely revolute");
                return RobotPostureSupportType::RPST_NoSupport;
            }
            const Transform tJ1J2 = joints[0]->GetInternalHierarchyRightTransform() * joints[1]->GetInternalHierarchyLeftTransform();
            const Transform tJ2J3 = joints[1]->GetInternalHierarchyRightTransform() * joints[2]->GetInternalHierarchyLeftTransform();
            const Transform tJ3J4 = joints[2]->GetInternalHierarchyRightTransform() * joints[3]->GetInternalHierarchyLeftTransform();
            const Transform tJ4J5 = joints[3]->GetInternalHierarchyRightTransform() * joints[4]->GetInternalHierarchyLeftTransform();
            const Transform tJ5J6 = joints[4]->GetInternalHierarchyRightTransform() * joints[5]->GetInternalHierarchyLeftTransform();
            if(
                   ((AnalyzeTransformBetweenNeighbouringJoints(tJ1J2) & NeighbouringTwoJointsRelation::NTJR_Perpendicular) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ2J3) & NeighbouringTwoJointsRelation::NTJR_Parallel)      != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ3J4) & NeighbouringTwoJointsRelation::NTJR_Perpendicular) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ4J5) & NeighbouringTwoJointsRelation::NTJR_Perpendicular) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ5J6) & NeighbouringTwoJointsRelation::NTJR_Perpendicular) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                ) {
                return RobotPostureSupportType::RPST_6R_General; ///< general 6R robots with the last joint axes intersecting at a point
            }

            break;
        }

        case 4: {
            if(!CheckJointTypes(joints, jointtypes)) {
                RAVELOG_WARN("Not all 4 joints are purely revolute");
                return RobotPostureSupportType::RPST_NoSupport;
            }
            const Transform tJ1J2 = joints[0]->GetInternalHierarchyRightTransform() * joints[1]->GetInternalHierarchyLeftTransform();
            const Transform tJ2J3 = joints[1]->GetInternalHierarchyRightTransform() * joints[2]->GetInternalHierarchyLeftTransform();
            const Transform tJ3J4 = joints[2]->GetInternalHierarchyRightTransform() * joints[3]->GetInternalHierarchyLeftTransform();
            if(
                   AnalyzeTransformBetweenNeighbouringJoints(tJ1J2) == NeighbouringTwoJointsRelation::NTJR_Intersect_Perpendicular
                && AnalyzeTransformBetweenNeighbouringJoints(tJ2J3) == NeighbouringTwoJointsRelation::NTJR_Parallel
                && AnalyzeTransformBetweenNeighbouringJoints(tJ3J4) == NeighbouringTwoJointsRelation::NTJR_Parallel
                ) {
                return RobotPostureSupportType::RPST_4R_Type_A; ///< a special type of 4R robot the last three parallel joint axes perpendicular to the first joint axis
            }
            break;
        }

        case 3: {
            if(!CheckJointTypes(joints, jointtypes)) {
                RAVELOG_WARN("Not all 3 joints are purely revolute");
                return RobotPostureSupportType::RPST_NoSupport;
            }

            const Transform tJ1J2 = joints[0]->GetInternalHierarchyRightTransform() * joints[1]->GetInternalHierarchyLeftTransform();
            const Transform tJ2J3 = joints[1]->GetInternalHierarchyRightTransform() * joints[2]->GetInternalHierarchyLeftTransform();
            if(
                   ((AnalyzeTransformBetweenNeighbouringJoints(tJ1J2) & NeighbouringTwoJointsRelation::NTJR_Parallel) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ2J3) & NeighbouringTwoJointsRelation::NTJR_Parallel) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                ) {
                return RobotPostureSupportType::RPST_RRR_Parallel; ///< a type of robot that has only three revolute joints whose axes are parallel
            }
            break;
        }

        default: {
            break;
        }
    }

    return RobotPostureSupportType::RPST_NoSupport;
}

void ComputePostureStates6RGeneral(const std::vector<JointPtr>& joints, const double fTol, std::vector<PostureStateInt>& posturestates) {
    const Vector axis0 = joints[0]->GetAxis();
    const Vector axis1 = joints[1]->GetAxis();
    const Vector axis3 = joints[3]->GetAxis();
    const Vector axis4 = joints[4]->GetAxis();
    const Vector axis5 = joints[5]->GetAxis();
    const Vector anchor0 = joints[0]->GetAnchor();
    const Vector anchor1 = joints[1]->GetAnchor();
    const Vector anchor2 = joints[2]->GetAnchor();
    const Vector anchor4 = joints[4]->GetAnchor();

    const std::array<double, 3> posturevalues {
        // shoulder: {{0, -1}, {1, -1}, {0, 4}}
        axis0.cross(axis1).dot(anchor4-anchor0),
        // elbow: {{1, -1}, {1, 2}, {2, 4}}
        axis1.cross(anchor2-anchor1).dot(anchor4-anchor2),
        // wrist: {3, -1}, {4, -1}, {5, -1}}
        axis3.cross(axis4).dot(axis5)
    };
    compute_robot_posture_states<3>(posturevalues, fTol, posturestates);
}

void ComputePostureStates4RTypeA(const std::vector<JointPtr>& joints, const double fTol, std::vector<PostureStateInt>& posturestates) {
    const Vector axis0 = joints[0]->GetAxis();
    const Vector axis1 = joints[1]->GetAxis();
    const Vector anchor1 = joints[1]->GetAnchor();
    const Vector anchor2 = joints[2]->GetAnchor();
    const Vector anchor3 = joints[3]->GetAnchor();

    const std::array<double, 2> posturevalues {
        // shoulder pose: {{0, -1}, {1, -1}, {1, 3}}
        axis0.cross(axis1).dot(anchor3-anchor1),
        // elbow: {{1, -1}, {1, 2}, {2, 3}}
        axis1.cross(anchor2-anchor1).dot(anchor3-anchor2),
    };
    compute_robot_posture_states<2>(posturevalues, fTol, posturestates);
}

void ComputePostureStatesRRRParallel(const std::vector<JointPtr>& joints, const double fTol, std::vector<PostureStateInt>& posturestates) {
    const Vector axis0 = joints[0]->GetAxis();
    const Vector anchor0 = joints[0]->GetAnchor();
    const Vector anchor1 = joints[1]->GetAnchor();
    const Vector anchor2 = joints[2]->GetAnchor();

    const std::array<double, 1> posturevalues {
        // elbow: {{0, -1}, {0, 1}, {1, 2}}
        axis0.cross(anchor1-anchor0).dot(anchor2-anchor1),
    };
    compute_robot_posture_states<1>(posturevalues, fTol, posturestates);
}

bool PostureDescriber::Init(const LinkPair& kinematicsChain) {
    if(!this->Supports(kinematicsChain)) {
        RAVELOG_WARN("Does not support kinematics chain");
        return false;
    }

    this->Destroy(); // clear internal contents
    
    _kinematicsChain = kinematicsChain;
    _GetJointsFromKinematicsChain(_kinematicsChain, _joints);
    for(const JointPtr& joint : _joints) {
        _armindices.push_back(joint->GetDOFIndex()); // collect arm indices
    }

    _supporttype = DeriveRobotPostureSupportType(_joints);
    switch(_supporttype) {
    case RobotPostureSupportType::RPST_6R_General: {
        _posturefn = ComputePostureStates6RGeneral;
        break;
    }
    case RobotPostureSupportType::RPST_4R_Type_A: {
        _posturefn = ComputePostureStates4RTypeA;
        break;
    }
    case RobotPostureSupportType::RPST_RRR_Parallel: {
        _posturefn = ComputePostureStatesRRRParallel;
        break;
    }
    default: {
        return false;
    }
    }
    return static_cast<bool>(_posturefn);
}

void PostureDescriber::_GetJointsFromKinematicsChain(const LinkPair& kinematicsChain,
                                                     std::vector<JointPtr>& joints) const {
    const int baselinkind = kinematicsChain[0]->GetIndex();
    const int eelinkind = kinematicsChain[1]->GetIndex();
    const KinBodyPtr probot = kinematicsChain[0]->GetParent();
    probot->GetChain(baselinkind, eelinkind, joints);
    // std::string typestr;
    for(std::vector<JointPtr>::iterator it = begin(joints); it != end(joints); ) {
        if((*it)->IsStatic() || (*it)->GetDOFIndex()==-1) {
            it = joints.erase(it);
        }
        else {
            ++it;
        }
    }
}

bool PostureDescriber::Supports(const LinkPair& kinematicsChain) const {
    const LinkPtr baselink = kinematicsChain[0];
    const LinkPtr eelink = kinematicsChain[1];

    if( baselink == nullptr || eelink == nullptr ) {
        RAVELOG_WARN("kinematics chain is not valid as having nullptr");
        return false;
    }

    std::vector<JointPtr> joints;
    _GetJointsFromKinematicsChain(kinematicsChain, joints);
    const RobotPostureSupportType supporttype = DeriveRobotPostureSupportType(joints);
    if(supporttype != RobotPostureSupportType::RPST_NoSupport) {
        return true;
    }

    const KinBodyPtr probot = kinematicsChain[0]->GetParent();
    // minor, const ref?
    const std::string& robotname = probot->GetName();
    const std::string& baselinkname = baselink->GetName();
    const std::string& eelinkname = eelink->GetName();
    RAVELOG_WARN_FORMAT("Cannot handle robot %s with armdof=%d for now: baselink=%s, eelink=%s", robotname % joints.size() % baselinkname % eelinkname);
    return false;
}


bool PostureDescriber::ComputePostureStates(std::vector<PostureStateInt>& posturestates, const std::vector<double>& dofvalues) {
    if(!_posturefn) {
        RAVELOG_WARN("No supported posture describer; _posturefn is not set");
        posturestates.clear();
        return false;
    }
    if(!dofvalues.empty()) {
        const KinBodyPtr probot = _kinematicsChain[0]->GetParent();
        if(dofvalues.size() != _joints.size()) {
            RAVELOG_WARN_FORMAT("dof values size does not match joint size: %d!=%d", dofvalues.size() % _joints.size());
            posturestates.clear();
            return false;
        }
        const KinBody::CheckLimitsAction claoption = KinBody::CheckLimitsAction::CLA_Nothing;
        const KinBody::KinBodyStateSaver saver(probot); // options = Save_LinkTransformation | Save_LinkEnable
        probot->SetDOFValues(dofvalues, claoption, _armindices);
        _posturefn(_joints, _fTol, posturestates);
    }
    else {
        _posturefn(_joints, _fTol, posturestates);
    }
    return true;
}

bool PostureDescriber::SetPostureValueThreshold(double fTol) {
    if(fTol < 0.0) {
        RAVELOG_WARN_FORMAT("Cannot set fTol=%.4d<0.0; do not change its current value %.4e", fTol % _fTol);
        return false;
    }
    _fTol = fTol;
    return true;
}

std::string PostureDescriber::GetMapDataKey() const {
    return std::string(POSTUREDESCRIBER_CLASS_NAME);
}

const LinkPair& PostureDescriber::GetEssentialKinematicsChain() const {
    return _kinematicsChain;
}

void PostureDescriber::Destroy() {
    _kinematicsChain  = {nullptr, nullptr};
    _joints.clear();
    _armindices.clear();
    _posturefn = nullptr;
    _supporttype = RobotPostureSupportType::RPST_NoSupport;
}

bool PostureDescriber::_SetPostureValueThresholdCommand(std::ostream& ssout, std::istream& ssin) {
    double fTol = 0.0;
    ssin >> fTol;
    return this->SetPostureValueThreshold(fTol);
}

bool PostureDescriber::_GetPostureValueThresholdCommand(std::ostream& ssout, std::istream& ssin) const {
    ssout << _fTol;
    return true;
}

bool PostureDescriber::_GetArmIndicesCommand(std::ostream& ssout, std::istream& ssin) const {
    SerializeValues(ssout, _armindices);
    return !_armindices.empty();
}

bool PostureDescriber::_GetSupportTypeCommand(std::ostream& ssout, std::istream& ssin) const {
    ssout << static_cast<int>(_supporttype);
    return _supporttype != RobotPostureSupportType::RPST_NoSupport;
}

bool PostureDescriber::_InterpretJSONCommand(const rapidjson::Value& input,
                                             rapidjson::Value& output,
                                             rapidjson::Document::AllocatorType& allocator) {
    std::vector<std::string> vfeatures;
    switch(_supporttype) {
        case RobotPostureSupportType::RPST_6R_General: {
            vfeatures = {"shoulder", "elbow", "wrist"};
            break;
        }
        case RobotPostureSupportType::RPST_4R_Type_A: {
            vfeatures = {"shoulder", "elbow"};
            break;
        }
        case RobotPostureSupportType::RPST_RRR_Parallel: {
            vfeatures = {"elbow"};
            break;
        }
        default: {
            RAVELOG_WARN("Unsupported posture type, cannot explain");
            return false;
        }
    }

    if (!input.HasMember("posturestate")) {
        RAVELOG_WARN("RapidJSON input does not have posturestate");
        return false;
    }
    const int state = input["posturestate"].GetInt();
    int pow2 = (1 << vfeatures.size());
    if(state < 0 || state >= pow2 ) {
        RAVELOG_WARN_FORMAT("Posture state should be in range [0, %d); now it is %d", pow2 % state);
        return false;
    }

    std::map<std::string, PostureStateInt> mfeaturestate;
    for(const std::string& feature : vfeatures) {
        pow2 >>= 1;
        mfeaturestate[feature] = (state & pow2) ? 1 : 0;
    }

    openravejson::SetJsonValueByKey(output,       "features",     vfeatures, allocator);
    openravejson::SetJsonValueByKey(output, "interpretation", mfeaturestate, allocator);

    return true;
}

} // namespace OpenRAVE
