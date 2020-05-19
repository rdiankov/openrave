#include "posturedescriber.h" // PostureDescriber
#include "posturesupporttype.h" // NeighbouringTwoJointsRelation
#include "openraveplugindefs.h" // SerializeValues

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
}

PostureDescriber::~PostureDescriber() {}

bool EnsureAllJointsPurelyRevolute(const std::vector<JointPtr>& joints) {
    std::stringstream ss;
    for(size_t i = 0; i < joints.size(); ++i) {
        const JointPtr& joint = joints[i];
        if(!joint->IsRevolute(0) || joint->IsCircular(0) || joint->GetDOF() != 1) {
            ss << joint->GetDOFIndex() << ",";
        }
    }
    if(!ss.str().empty()) {
        RAVELOG_WARN_FORMAT("Joints with DOF indices %s are not purely revolute with 1 dof each", ss.str());
        return false;
    }
    return true;
}

NeighbouringTwoJointsRelation AnalyzeTransformBetweenNeighbouringJoints(const Transform& t) {
    const double tol = 2e-15; // increase for densowave-VS087A4-AV6
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

    // std::stringstream ss;
    // ss << std::setprecision(16);
    // ss << "o = " << static_cast<int>(o) << ", t = " << t << ", dotprod = " << dotprod;
    // RAVELOG_WARN_FORMAT("%s", ss.str());
    return o;
}

RobotPostureSupportType DeriveRobotPostureSupportType(const std::vector<JointPtr> joints) {
    if(joints.size() == 6) {
        if(!EnsureAllJointsPurelyRevolute(joints)) {
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
            return RobotPostureSupportType::RPST_6R_General;
        }
        else {
            return RobotPostureSupportType::RPST_NoSupport;
        }
    }
    else if(joints.size() == 4) {
        if(!EnsureAllJointsPurelyRevolute(joints)) {
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
            return RobotPostureSupportType::RPST_4R_Type_A;
        }
        else {
            return RobotPostureSupportType::RPST_NoSupport;
        }
    }
    return RobotPostureSupportType::RPST_NoSupport;
}

bool PostureDescriber::Init(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) {
    if(!this->Supports(kinematicsChain)) {
        RAVELOG_WARN("Does not support kinematics chain");
        return false;
    }
    _kinematicsChain = kinematicsChain;
    _GetJointsFromKinematicsChain(_kinematicsChain, _joints);
    for(const JointPtr& joint : _joints) {
        _armindices.push_back(joint->GetDOFIndex()); // collect arm indices
    }

    const RobotPostureSupportType supporttype = DeriveRobotPostureSupportType(_joints);
    switch(supporttype) {
        case RobotPostureSupportType::RPST_6R_General: {
            _posturefn = ComputePostureStates6RGeneral;
            break;
        }
        case RobotPostureSupportType::RPST_4R_Type_A: {
            _posturefn = ComputePostureStates4RTypeA;
            break;
        }
        default: {
            return false;
        }
    }

    // std::stringstream ss;
    // for(int i : _armindices) {
    //     ss << i << ", ";
    // }
    // RAVELOG_WARN(ss.str());
    return static_cast<bool>(_posturefn);
}

void PostureDescriber::_GetJointsFromKinematicsChain(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain,
                                                          std::vector<JointPtr>& joints) const {
    const int baselinkind = kinematicsChain[0]->GetIndex();
    const int eelinkind = kinematicsChain[1]->GetIndex();
    const KinBodyPtr probot = kinematicsChain[0]->GetParent();
    probot->GetChain(baselinkind, eelinkind, joints);
    for(std::vector<JointPtr>::iterator it = begin(joints); it != end(joints);) {
        if((*it)->IsStatic() || (*it)->GetDOFIndex()==-1) {
            it = joints.erase(it);
        }
        else {
            ++it;
        }
    }
}

bool PostureDescriber::Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const {
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


bool PostureDescriber::ComputePostureStates(std::vector<uint16_t>& posturestates, const std::vector<double>& jointvalues) {
    if(!_posturefn) {
        RAVELOG_WARN("No supported posture describer; _posturefn is not set");
        posturestates.clear();
        return false;
    }
    if(!jointvalues.empty()) {
        const KinBodyPtr probot = _kinematicsChain[0]->GetParent();
        if(jointvalues.size() != _joints.size()) {
            RAVELOG_WARN_FORMAT("jointvalues size does not match joint size: %d!=%d", jointvalues.size() % _joints.size());
            posturestates.clear();
            return false;
        }
        const KinBody::CheckLimitsAction claoption = KinBody::CheckLimitsAction::CLA_Nothing;
        const KinBody::KinBodyStateSaver saver(probot); // options = Save_LinkTransformation | Save_LinkEnable
        probot->SetDOFValues(jointvalues, claoption, _armindices);
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
    SerializeValues(ssout, _armindices, ' ');
    return !_armindices.empty();
}

} // namespace OpenRAVE
