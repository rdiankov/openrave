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

    this->RegisterCommand("ComputePostureValues",
                          boost::bind(&PostureDescriber::_ComputePostureValuesCommand, this, _1, _2),
                          "Compute the posture values for all features");

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
NeighbouringTwoJointsRelation AnalyzeTransformBetweenNeighbouringJoints(const Transform& t, const double tol) {
    // tol was increased for densowave-VS087A4-AV6 (2e-15), and then RV-35FM (4e-15)
    const Vector zaxis0(0, 0, 1); // z-axis of the first joint
    const Vector zaxis1 = t.rotate(zaxis0); // z-axis of the second joint
    const double dotprod = zaxis1.dot3(zaxis0);

    NeighbouringTwoJointsRelation o = NeighbouringTwoJointsRelation::NTJR_Unknown;
    if(1.0 - fabs(dotprod) <= tol) {
        o |= NeighbouringTwoJointsRelation::NTJR_Parallel;
        if(zaxis0.cross(t.trans).lengthsqr3() <= tol) {
            o |= NeighbouringTwoJointsRelation::NTJR_Intersect; // NeighbouringTwoJointsRelation::NTJR_Overlap
        }
    }
    else {
        // not parallel
        if (fabs(dotprod) <= tol) {
            o |= NeighbouringTwoJointsRelation::NTJR_Perpendicular;
        }
        if(fabs(zaxis0.cross(zaxis1).dot3(t.trans)) <= tol) {
            o |= NeighbouringTwoJointsRelation::NTJR_Intersect; // NeighbouringTwoJointsRelation::NTJR_Intersect_Perpendicular
        }
    }

#if defined(POSTUREDESCRIBER_DEBUG)
    std::stringstream ss;
    ss << std::setprecision(16);
    ss << "o = " << static_cast<int>(o) << ", t = " << t << ", dotprod = " << dotprod;
    RAVELOG_INFO_FORMAT("%s", ss.str());
#endif // defined(POSTUREDESCRIBER_DEBUG)
    return o;
}

/// \brief Derives the robot posture support type for a sequence of joints along a kinematics chain
/// \return a RobotPostureSupportType enum for the kinematics chain
    // const ref
RobotPostureSupportType DeriveRobotPostureSupportType(const std::vector<JointPtr>& joints, const double tol) {
    const size_t njoints = joints.size();
    const std::vector<KinBody::JointType> jointtypes(njoints, KinBody::JointType::JointRevolute); ///< so far these are all revolute
    switch(njoints) {
        case 6: {
            if(!CheckJointTypes(joints, jointtypes)) {
                RAVELOG_WARN_FORMAT("Not all %d joints are purely revolute", njoints);
                return RobotPostureSupportType::RPST_NoSupport;
            }
            const Transform tJ1J2 = joints[0]->GetInternalHierarchyRightTransform() * joints[1]->GetInternalHierarchyLeftTransform();
            const Transform tJ2J3 = joints[1]->GetInternalHierarchyRightTransform() * joints[2]->GetInternalHierarchyLeftTransform();
            const Transform tJ3J4 = joints[2]->GetInternalHierarchyRightTransform() * joints[3]->GetInternalHierarchyLeftTransform();
            const Transform tJ4J5 = joints[3]->GetInternalHierarchyRightTransform() * joints[4]->GetInternalHierarchyLeftTransform();
            const Transform tJ5J6 = joints[4]->GetInternalHierarchyRightTransform() * joints[5]->GetInternalHierarchyLeftTransform();
            if(
                   ((AnalyzeTransformBetweenNeighbouringJoints(tJ1J2, tol) & NeighbouringTwoJointsRelation::NTJR_Perpendicular) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ2J3, tol) & NeighbouringTwoJointsRelation::NTJR_Parallel)      != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ3J4, tol) & NeighbouringTwoJointsRelation::NTJR_Perpendicular) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ4J5, tol) == NeighbouringTwoJointsRelation::NTJR_Intersect_Perpendicular))
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ5J6, tol) == NeighbouringTwoJointsRelation::NTJR_Intersect_Perpendicular))
                ) {

                // Test J4's and J6's axes also intersect by rotating J5 by 0, pi/2, pi, 3*pi/2
                // All transforms and vectors below are in J4 frame
                const Vector zaxis(0, 0, 1);
                const Vector& J4axis = zaxis;
                Transform tJ5, tJ4J6;
                Vector J6axis;
                double triprod = 0.0;
                const int nRotations = 4;
                const double fIncrement = M_PI * 2.0/(double) nRotations;
                for(int i = 0; i < nRotations; ++i) {
                    tJ5.rot = quatFromAxisAngle(zaxis, /*J5 = */fIncrement * i);
                    tJ4J6 = tJ4J5 * tJ5 * tJ5J6;
                    J6axis = tJ4J6.rotate(zaxis);
                    triprod = J4axis.cross(J6axis).dot(tJ4J6.trans);
                    if(fabs(triprod) > tol) {
                        return RobotPostureSupportType::RPST_NoSupport;
                    }
                }
                return RobotPostureSupportType::RPST_6R_General; ///< general 6R robots with the last joint axes intersecting at a point
            }
            break;
        }

        case 4: {
            if(!CheckJointTypes(joints, jointtypes)) {
                RAVELOG_WARN_FORMAT("Not all %d joints are purely revolute", njoints);
                return RobotPostureSupportType::RPST_NoSupport;
            }
            const Transform tJ1J2 = joints[0]->GetInternalHierarchyRightTransform() * joints[1]->GetInternalHierarchyLeftTransform();
            const Transform tJ2J3 = joints[1]->GetInternalHierarchyRightTransform() * joints[2]->GetInternalHierarchyLeftTransform();
            const Transform tJ3J4 = joints[2]->GetInternalHierarchyRightTransform() * joints[3]->GetInternalHierarchyLeftTransform();
            if(
                   AnalyzeTransformBetweenNeighbouringJoints(tJ1J2, tol) == NeighbouringTwoJointsRelation::NTJR_Intersect_Perpendicular
                && AnalyzeTransformBetweenNeighbouringJoints(tJ2J3, tol) == NeighbouringTwoJointsRelation::NTJR_Parallel
                && AnalyzeTransformBetweenNeighbouringJoints(tJ3J4, tol) == NeighbouringTwoJointsRelation::NTJR_Parallel
                ) {
                return RobotPostureSupportType::RPST_4R_Type_A; ///< a special type of 4R robot the last three parallel joint axes perpendicular to the first joint axis
            }
            break;
        }

        case 3: {
            if(!CheckJointTypes(joints, jointtypes)) {
                RAVELOG_WARN_FORMAT("Not all %d joints are purely revolute", njoints);
                return RobotPostureSupportType::RPST_NoSupport;
            }

            const Transform tJ1J2 = joints[0]->GetInternalHierarchyRightTransform() * joints[1]->GetInternalHierarchyLeftTransform();
            const Transform tJ2J3 = joints[1]->GetInternalHierarchyRightTransform() * joints[2]->GetInternalHierarchyLeftTransform();
            if(
                   ((AnalyzeTransformBetweenNeighbouringJoints(tJ1J2, tol) & NeighbouringTwoJointsRelation::NTJR_Parallel) != NeighbouringTwoJointsRelation::NTJR_Unknown)
                && ((AnalyzeTransformBetweenNeighbouringJoints(tJ2J3, tol) & NeighbouringTwoJointsRelation::NTJR_Parallel) != NeighbouringTwoJointsRelation::NTJR_Unknown)
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

/// \brief determines whether a robot posture value can be considered as 0.0, postive, or negative
/// \param [in] x      a posture value
/// \param [in] tol    tolerance to determine whether x is considered 0.0, so that this value means a hybrid state.
/// \return 0 if x is considered positive, 1 if considered negative, and 2 (meaning hybrid states) if considered 0.0
inline PostureStateInt ComputeFeatureState(const double x, const double fTol) {
    return (x > fTol) ? 0 : (x < -fTol) ? 1 : 2; // TGN: >= or <= ?
}

/// \brief Computes a vector of posture state integers using N posture values.
/// \param [in]  posturevalues    an array of posture values
/// \param [in]  tol              tolerance to determine whether x is considered 0.0, so that this value means a hybrid state.
/// \param [out] posturestates    a vector of posture state (unsigned) integers, whose size is always a power of 2
inline void ComputeRobotPostureStates(const double fTol,
                                      const std::vector<double>& posturevalues,
                                      std::vector<PostureStateInt>& featurestates,
                                      std::vector<PostureStateInt>& posturestates
                                      ) {
    const size_t N = posturevalues.size();
    for(size_t i = 0; i < N; ++i) {
        featurestates.at(i) = ComputeFeatureState(posturevalues.at(i), fTol);
    }

    posturestates = {0};
    posturestates.reserve(1 << N);
    for(size_t i = 0; i < N; ++i) {
        for(PostureStateInt &state : posturestates) {
            state <<= 1;
        }
        if(featurestates.at(i) == 1) {
            for(PostureStateInt &state : posturestates) {
                state |= 1;
            }
        }
        else if (featurestates.at(i) == 2) {
            const size_t nstates = posturestates.size();
            posturestates.insert(end(posturestates), begin(posturestates), end(posturestates));
            for(size_t j = nstates; j < 2 * nstates; ++j) {
                posturestates.at(j) |= 1;
            }
        }
    }
}

void ComputePostureStates6RGeneral(const std::vector<JointPtr>& joints,
                                   const double fTol,
                                   std::vector<double>& posturevalues,
                                   std::vector<PostureStateInt>& featurestates,
                                   std::vector<PostureStateInt>& posturestates
                                   ) {
    const Vector axis0 = joints[0]->GetAxis();
    const Vector axis1 = joints[1]->GetAxis();
    const Vector axis3 = joints[3]->GetAxis();
    const Vector axis4 = joints[4]->GetAxis();
    const Vector axis5 = joints[5]->GetAxis();
    const Vector anchor0 = joints[0]->GetAnchor();
    const Vector anchor1 = joints[1]->GetAnchor();
    const Vector anchor2 = joints[2]->GetAnchor();
    const Vector anchor3 = joints[3]->GetAnchor();
    // instead of using J5's anchor (anchor4) directly, we project it onto J4's axis (axis3)
    const Vector anchor4 = anchor3 + (joints[4]->GetAnchor() - anchor3).dot(axis3) * axis3;
    
    posturevalues.at(0) = axis0.cross(axis1).dot(anchor4-anchor0);           ///< shoulder: {{0, -1}, {1, -1}, {0,  4}}
    posturevalues.at(1) = axis1.cross(anchor2-anchor1).dot(anchor4-anchor2); ///<    elbow: {{1, -1}, {1,  2}, {2,  4}}
    posturevalues.at(2) = axis3.cross(axis4).dot(axis5);                     ///<    wrist: {{3, -1}, {4, -1}, {5, -1}}
    ComputeRobotPostureStates(fTol, posturevalues, featurestates, posturestates);
}

void ComputePostureStates4RTypeA(const std::vector<JointPtr>& joints,
                                 const double fTol,
                                 std::vector<double>& posturevalues,
                                 std::vector<PostureStateInt>& featurestates,
                                 std::vector<PostureStateInt>& posturestates
                                 ) {
    const Vector axis0 = joints[0]->GetAxis();
    const Vector axis1 = joints[1]->GetAxis();
    const Vector anchor1 = joints[1]->GetAnchor();
    const Vector anchor2 = joints[2]->GetAnchor();
    const Vector anchor3 = joints[3]->GetAnchor();

    posturevalues.at(0) = axis0.cross(axis1).dot(anchor3-anchor1);           ///< shoulder: {{0, -1}, {1, -1}, {1, 3}}
    posturevalues.at(1) = axis1.cross(anchor2-anchor1).dot(anchor3-anchor2); ///<    elbow: {{1, -1}, {1,  2}, {2, 3}}
    ComputeRobotPostureStates(fTol, posturevalues, featurestates, posturestates);
}

void ComputePostureStatesRRRParallel(const std::vector<JointPtr>& joints,
                                     const double fTol,
                                     std::vector<double>& posturevalues,
                                     std::vector<PostureStateInt>& featurestates,
                                     std::vector<PostureStateInt>& posturestates
                                     ) {
    const Vector axis0 = joints[0]->GetAxis();
    const Vector anchor0 = joints[0]->GetAnchor();
    const Vector anchor1 = joints[1]->GetAnchor();
    const Vector anchor2 = joints[2]->GetAnchor();

    posturevalues.at(0) = axis0.cross(anchor1-anchor0).dot(anchor2-anchor1); ///< elbow: {{0, -1}, {0, 1}, {1, 2}}
    ComputeRobotPostureStates(fTol, posturevalues, featurestates, posturestates);
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

    _supporttype = DeriveRobotPostureSupportType(_joints, _fGeometryTol);
    switch(_supporttype) {
    case RobotPostureSupportType::RPST_6R_General: {
        _posturefn = ComputePostureStates6RGeneral;
        _posturevalues.resize(3);
        _featurestates.resize(3);        
        break;
    }
    case RobotPostureSupportType::RPST_4R_Type_A: {
        _posturefn = ComputePostureStates4RTypeA;
        _posturevalues.resize(2);
        _featurestates.resize(2);        
        break;
    }
    case RobotPostureSupportType::RPST_RRR_Parallel: {
        _posturefn = ComputePostureStatesRRRParallel;
        _posturevalues.resize(1);
        _featurestates.resize(1);
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

    // Step 1: Check whether all mimic joints are "virtual", in the sense that we can combine each with its next active joint
    int njoints = joints.size();
    for(int ijoint = 0; ijoint < njoints - 1; ++ijoint) {
        const JointPtr& currJoint = joints[ijoint];
        if(!currJoint->IsMimic()) {
            continue;
        }
        const JointPtr& nextJoint = joints[ijoint + 1];
        const Transform t = currJoint->GetInternalHierarchyRightTransform() * nextJoint->GetInternalHierarchyLeftTransform();
        if(t.trans.lengthsqr3() > _fTol) {
            std::stringstream ss;
            ss << "Transform between mimic joint " << currJoint->GetName() << " and its next joint " << nextJoint->GetName() << " has a nonzero translate, so we cannot support; transform is " << t;
            RAVELOG_WARN_FORMAT("%s", ss.str());
            joints.clear();
            return;
        }
    }

    // Step 2: Remove static and mimic joints
    for(std::vector<JointPtr>::iterator it = begin(joints); it != end(joints); ) {
        if((*it)->IsStatic() || (*it)->IsMimic() || (*it)->GetDOFIndex()==-1) {
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
    const RobotPostureSupportType supporttype = DeriveRobotPostureSupportType(joints, _fGeometryTol);
    if(supporttype != RobotPostureSupportType::RPST_NoSupport) {
        return true;
    }

    const KinBodyPtr probot = kinematicsChain[0]->GetParent();
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
        _posturefn(_joints, _fTol, _posturevalues, _featurestates, posturestates);
        // restore KinBodyState automatically
    }
    else {
        _posturefn(_joints, _fTol, _posturevalues, _featurestates, posturestates);
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
    return _posturestatename;
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

bool PostureDescriber::_ComputePostureValuesCommand(std::ostream& ssout, std::istream& ssin) {
    if(!_posturefn) {
        RAVELOG_WARN("No supported posture describer; _posturefn is not set");
        return false;
    }
    _posturefn(_joints, _fTol, _posturevalues, _featurestates, _posturestates); // compute using all cached variables
    SerializeValues(ssout, _posturevalues);
    return !_posturestates.empty();
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
