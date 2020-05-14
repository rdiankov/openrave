#include "neighbouringtwojointsrelations.h" // NeighbouringTwoJointsRelations
#include "robotposturedescriber.h" // RobotPostureDescriber

namespace OpenRAVE {

using JointPtr = OpenRAVE::KinBody::JointPtr;

RobotPostureDescriber::RobotPostureDescriber(EnvironmentBasePtr penv,
                                             const double fTol) :
    RobotPostureDescriberBase(penv),
    _fTol(fTol) 
{
    // `SendCommand` APIs
    this->RegisterCommand("SetPostureValueThreshold",
                          boost::bind(&RobotPostureDescriber::_SetPostureValueThresholdCommand, this, _1, _2),
                          "Sets the tolerance for determining whether a robot posture value is close to 0 and hence would have hybrid states");

    this->RegisterCommand("GetPostureValueThreshold",
                          boost::bind(&RobotPostureDescriber::_GetPostureValueThresholdCommand, this, _1, _2),
                          "Gets the tolerance for determining whether a robot posture value is close to 0 and hence would have hybrid states");
}

RobotPostureDescriber::~RobotPostureDescriber() {}

bool RobotPostureDescriber::Init(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) {
    if(!this->Supports(kinematicsChain)) {
        return false;
    }
    _kinematicsChain = kinematicsChain;
    _GetJointsFromKinematicsChain(_kinematicsChain, _joints);

    const size_t njoints = _joints.size();
    for(const JointPtr& joint : _joints) {
        _armindices.push_back(joint->GetDOFIndex());
    }
    if(njoints == 6) {
        _posturefn = Compute6RRobotPostureStates0;
    }
    else if (njoints == 4) {
        _posturefn = Compute4DofRobotPostureStates0;
    }

    // std::stringstream ss;
    // for(int i : _armindices) {
    //     ss << i << ", ";
    // }
    // RAVELOG_WARN(ss.str());
    return static_cast<bool>(_posturefn);
}

void RobotPostureDescriber::_GetJointsFromKinematicsChain(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain,
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

NeighbouringTwoJointsRelations AnalyzeTransformBetweenNeighbouringJoints(const Transform& t) {
    const double tol = 1e-15;
    const Vector zaxis0(0, 0, 1); // z-axis of the first joint
    const Vector zaxis1 = t.rotate(zaxis0); // z-axis of the second joint
    const double dotprod = zaxis1.dot3(zaxis0);

    NeighbouringTwoJointsRelations o = NeighbouringTwoJointsRelations::NTJR_UNKNOWN;
    if(1.0 - fabs(dotprod) <= tol) {
        o |= NeighbouringTwoJointsRelations::NTJR_PARALLEL; // TO-DO: check overlapping
        if(zaxis0.cross(t.trans).lengthsqr3() <= tol) {
            o |= NeighbouringTwoJointsRelations::NTJR_INTERSECT;
        }
    }
    else {
        // not parallel
        if (fabs(dotprod) <= tol) {
            o |= NeighbouringTwoJointsRelations::NTJR_PERPENDICULAR;
        }
        if(fabs(zaxis0.cross(zaxis1).dot3(t.trans)) <= tol) {
            o |= NeighbouringTwoJointsRelations::NTJR_INTERSECT;
        }
    }

    // std::stringstream ss;
    // ss << std::setprecision(16);
    // ss << "o = " << static_cast<int>(o) << ", t = " << t;
    // RAVELOG_WARN_FORMAT("%s", ss.str());
    return o;
}

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

bool AnalyzeSixRevoluteJoints0(const std::vector<JointPtr>& joints) {
    if(joints.size() != 6) {
        // throw OPENRAVE_EXCEPTION_FORMAT("number of joints is not 6: %d!=6", joints.size(), ORE_InvalidArguments);
        RAVELOG_WARN_FORMAT("Number of joints is not 6: %d!=6", joints.size());
        return false;
    }

    if(!EnsureAllJointsPurelyRevolute(joints)) {
        RAVELOG_WARN("Not all joints are purely revolute");
        return false;
    }

    const Transform tJ1J2 = joints[0]->GetInternalHierarchyRightTransform() * joints[1]->GetInternalHierarchyLeftTransform();
    const Transform tJ2J3 = joints[1]->GetInternalHierarchyRightTransform() * joints[2]->GetInternalHierarchyLeftTransform();
    const Transform tJ3J4 = joints[2]->GetInternalHierarchyRightTransform() * joints[3]->GetInternalHierarchyLeftTransform();
    const Transform tJ4J5 = joints[3]->GetInternalHierarchyRightTransform() * joints[4]->GetInternalHierarchyLeftTransform();
    const Transform tJ5J6 = joints[4]->GetInternalHierarchyRightTransform() * joints[5]->GetInternalHierarchyLeftTransform();

    return ((AnalyzeTransformBetweenNeighbouringJoints(tJ1J2) & NeighbouringTwoJointsRelations::NTJR_PERPENDICULAR) != NeighbouringTwoJointsRelations::NTJR_UNKNOWN)
        && AnalyzeTransformBetweenNeighbouringJoints(tJ2J3) == NeighbouringTwoJointsRelations::NTJR_PARALLEL
        && ((AnalyzeTransformBetweenNeighbouringJoints(tJ3J4) & NeighbouringTwoJointsRelations::NTJR_PERPENDICULAR) != NeighbouringTwoJointsRelations::NTJR_UNKNOWN)
        && AnalyzeTransformBetweenNeighbouringJoints(tJ4J5) == NeighbouringTwoJointsRelations::NTJR_INTERSECT_PERPENDICULAR
        // && AnalyzeTransformBetweenNeighbouringJoints(tJ5J6) == NeighbouringTwoJointsRelations::NTJR_INTERSECT_PERPENDICULAR
        // TGN: not necessarily intersect?
        && ((AnalyzeTransformBetweenNeighbouringJoints(tJ5J6) & NeighbouringTwoJointsRelations::NTJR_PERPENDICULAR) != NeighbouringTwoJointsRelations::NTJR_UNKNOWN)
        ;
}

bool AnalyzeFourRevoluteJoints0(const std::vector<JointPtr>& joints) {
    if(joints.size() != 4) {
        // throw OPENRAVE_EXCEPTION_FORMAT("number of joints is not 4: %d!=4", joints.size(), ORE_InvalidArguments);
        RAVELOG_WARN_FORMAT("Number of joints is not 4: %d!=4", joints.size());
        return false;
    }

    if(!EnsureAllJointsPurelyRevolute(joints)) {
        RAVELOG_WARN("Not all joints are purely revolute");
        return false;
    }

    const Transform tJ1J2 = joints[0]->GetInternalHierarchyRightTransform() * joints[1]->GetInternalHierarchyLeftTransform();
    const Transform tJ2J3 = joints[1]->GetInternalHierarchyRightTransform() * joints[2]->GetInternalHierarchyLeftTransform();
    const Transform tJ3J4 = joints[2]->GetInternalHierarchyRightTransform() * joints[3]->GetInternalHierarchyLeftTransform();

    return AnalyzeTransformBetweenNeighbouringJoints(tJ1J2) == NeighbouringTwoJointsRelations::NTJR_INTERSECT_PERPENDICULAR
        && AnalyzeTransformBetweenNeighbouringJoints(tJ2J3) == NeighbouringTwoJointsRelations::NTJR_PARALLEL
        && AnalyzeTransformBetweenNeighbouringJoints(tJ3J4) == NeighbouringTwoJointsRelations::NTJR_PARALLEL
        ;
}

bool RobotPostureDescriber::Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const {
    std::vector<JointPtr> joints;
    _GetJointsFromKinematicsChain(kinematicsChain, joints);
    const size_t armdof = joints.size();
    if(armdof == 6 && AnalyzeSixRevoluteJoints0(joints)) {
        return true;
    }
    const KinBodyPtr probot = kinematicsChain[0]->GetParent();
    const std::string robotname = probot->GetName();
    if(armdof == 4 && AnalyzeFourRevoluteJoints0(joints)) {
        return true;
    }

    const std::string baselinkname = kinematicsChain[0]->GetName();
    const std::string eelinkname = kinematicsChain[1]->GetName();
    RAVELOG_WARN_FORMAT("Cannot handle robot %s with armdof=%d for now: baselink=%s, eelink=%s", robotname % armdof % baselinkname % eelinkname);
    return false;
}


bool RobotPostureDescriber::ComputePostureValues(std::vector<uint16_t>& posturestates, const std::vector<double>& jointvalues) {
    if(!jointvalues.empty()) {
        const KinBodyPtr probot = _kinematicsChain[0]->GetParent();
        if(jointvalues.size() != _joints.size()) {
            RAVELOG_WARN_FORMAT("jointvalues size does not match joint size: %d!=%d", jointvalues.size() % _joints.size());
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

bool RobotPostureDescriber::SetPostureValueThreshold(double fTol) {
    if(fTol < 0.0) {
        RAVELOG_WARN_FORMAT("Cannot set fTol=%.4d<0.0; do not change its current value %.4e", fTol % _fTol);
        return false;
    }
    _fTol = fTol;
    return true;
}

bool RobotPostureDescriber::_SetPostureValueThresholdCommand(std::ostream& ssout, std::istream& ssin) {
    double fTol = 0.0;
    ssin >> fTol;
    return this->SetPostureValueThreshold(fTol);
}

bool RobotPostureDescriber::_GetPostureValueThresholdCommand(std::ostream& ssout, std::istream& ssin) const {
    ssout << _fTol;
    return true;
}

} // namespace OpenRAVE
