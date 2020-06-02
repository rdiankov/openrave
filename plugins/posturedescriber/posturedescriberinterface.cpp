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

#include "posturedescriberinterface.h" // PostureDescriber
#include "plugindefs.h" // POSTUREDESCRIBER_CLASS_NAME
#include "openraveplugindefs.h" // SerializeValues

// #define POSTUREDESCRIBER_DEBUG

namespace OpenRAVE {

using JointPtr = OpenRAVE::KinBody::JointPtr;

PostureDescriber::PostureDescriber(EnvironmentBasePtr penv,
                                   const double fTol) :
    PostureDescriberBase(penv),
    _fTol(fTol)
{
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
}

PostureDescriber::~PostureDescriber() {
}

inline bool IsActiveRevolute(const JointPtr& joint) {
    return joint->IsRevolute(0) && !joint->IsCircular(0) && joint->GetDOF() == 1;
}

inline bool IsActivePrismatic(const JointPtr& joint) {
    return joint->IsPrismatic(0) && joint->GetDOF() == 1;
}

bool CheckJointTypes(const std::vector<JointPtr>& joints, const std::string& typestr) {
    const size_t njoints = joints.size();
    if(njoints != typestr.size()) {
        throw OPENRAVE_EXCEPTION_FORMAT("Size mismatch: %d != %d", joints.size() % typestr.size(), ORE_InvalidArguments);
    }
    for(size_t i = 0; i < njoints; ++i) {
        if(typestr[i] == 'P') {
            if(!IsActivePrismatic(joints[i])) {
                return false;
            }
        }
        else if(typestr[i] == 'R') {
            if(!IsActiveRevolute(joints[i])) {
                return false;
            }
        }
        else {
            return false;
        }
    }
    return true;
}

/// \brief Checks whether all joints are purely revolute (not prismatic nor circular).
/// \return true if all joints are purely revolute
bool CheckAllJointsPurelyRevolute(const std::vector<JointPtr>& joints) {
    std::stringstream ss;
    for(size_t i = 0; i < joints.size(); ++i) {
        const JointPtr& joint = joints[i];
        if(!(IsActiveRevolute(joint))) {
            ss << joint->GetDOFIndex() << ",";
        }
    }
    if(!ss.str().empty()) {
        RAVELOG_WARN_FORMAT("Joints with DOF indices %s are not purely revolute with 1 dof each", ss.str());
        return false;
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
    RAVELOG_WARN_FORMAT("%s", ss.str());
#endif // defined(POSTUREDESCRIBER_DEBUG)
    return o;
}

/// \brief Derives the robot posture support type for a sequence of joints along a kinematics chain
/// \return a RobotPostureSupportType enum for the kinematics chain
RobotPostureSupportType DeriveRobotPostureSupportType(const std::vector<JointPtr> joints) {
    if(joints.size() == 6) {
        if(!CheckAllJointsPurelyRevolute(joints)) {
            RAVELOG_WARN("Not all joints are purely revolute");
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
        else {
            return RobotPostureSupportType::RPST_NoSupport; ///< unsupported
        }
    }
    else if(joints.size() == 4) {
        if(!CheckAllJointsPurelyRevolute(joints)) {
            RAVELOG_WARN("Not all joints are purely revolute");
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
        else {
            return RobotPostureSupportType::RPST_NoSupport; ///< unsupported
        }
    }
    return RobotPostureSupportType::RPST_NoSupport;
}

/// \brief Obtains either a joint axis or a translate between two joint anchors.
/// \return a joint axis, if vecinfo[1]==-1; otherwise a translate between two joint anchors.
Vector GetVectorFromInfo(const std::vector<JointPtr>& joints, const std::array<int, 2>& vecinfo) {
    // std::cout << "GetVectorFromInfo: " << vecinfo[0] << ", " << vecinfo[1] << std::endl;
    return (vecinfo[1]==-1) ?
           /* joint axis   */ joints[vecinfo[0]]->GetAxis() :
           /* joint anchor */ (joints[vecinfo[1]]->GetAnchor()-joints[vecinfo[0]]->GetAnchor());
}

/// \brief Generates a posture value function from a N-vector (array) of posture formulations.
/// Each posture value is a triple product in form (a x b) ∙ c, where each vector is either a joint axis or a translate between two joint anchors.
/// \return a posture value function, PostureValueFn=std::function<...> converted from a lambda expression.
template <size_t N>
PostureValueFn PostureValuesFunctionGenerator(const std::array<PostureFormulation, N>& postureforms) {
    return [= /* pass postureforms *by value* */](const std::vector<JointPtr>&joints, const double fTol, std::vector<PostureStateInt>&posturestates) {
               std::array<double, N> posturevalues;
               for(size_t i = 0; i < N; ++i) {
                   const PostureFormulation& postureform = postureforms[i];
                   posturevalues[i] = GetVectorFromInfo(joints, postureform[0]).
                                      cross(GetVectorFromInfo(joints, postureform[1])).
                                      dot(GetVectorFromInfo(joints, postureform[2]));
               }
               compute_robot_posture_states<N>(posturevalues, fTol, posturestates);
    };
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
        // j1 pose: {{0, -1}, {1, -1}, {1, 3}}
        axis0.cross(axis1).dot(anchor3-anchor1),
        // elbow: {{1, -1}, {1, 2}, {2, 3}}
        axis1.cross(anchor2-anchor1).dot(anchor3-anchor2),
    };
    compute_robot_posture_states<2>(posturevalues, fTol, posturestates);
}

bool PostureDescriber::Init(const LinkPair& kinematicsChain) {
    if(!this->Supports(kinematicsChain)) {
        RAVELOG_WARN("Does not support kinematics chain");
        return false;
    }
    _kinematicsChain = kinematicsChain;
    _GetJointsFromKinematicsChain(_kinematicsChain, _joints);
    for(const JointPtr& joint : _joints) {
        _armindices.push_back(joint->GetDOFIndex()); // collect arm indices
    }

    _supporttype = DeriveRobotPostureSupportType(_joints);
    switch(_supporttype) {
    case RobotPostureSupportType::RPST_6R_General: {
        const PostureFormulation
            shoulderform {{
                              {0, -1},
                              {1, -1},
                              {0, 4},
                          }},
        elbowform {{
                       {1, -1},
                       {1, 2},
                       {2, 4}
                   }},
        wristform {{
                       {3, -1},
                       {4, -1},
                       {5, -1}
                   }}
        ;
        const std::array<PostureFormulation, 3> postureforms = {
            shoulderform,
            elbowform,
            wristform
        };
        _posturefn = PostureValuesFunctionGenerator<3>(postureforms);
        _posturefn = ComputePostureStates6RGeneral;
        break;
    }
    case RobotPostureSupportType::RPST_4R_Type_A: {
        const PostureFormulation
            shoulderform {{
                        {0, -1},
                        {1, -1},
                        {1, 3},
                    }},
        elbowform {{
                       {1, -1},
                       {1, 2},
                       {2, 3}
                   }};
        const std::array<PostureFormulation, 2> postureforms = {
            shoulderform,
            elbowform
        };
        _posturefn = PostureValuesFunctionGenerator<2>(postureforms);
        _posturefn = ComputePostureStates4RTypeA;
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
    if( kinematicsChain[0] == nullptr || kinematicsChain[1] == nullptr ) {
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
    const std::string robotname = probot->GetName();
    const std::string baselinkname = kinematicsChain[0]->GetName();
    const std::string eelinkname = kinematicsChain[1]->GetName();
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

} // namespace OpenRAVE
