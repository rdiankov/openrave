#include "robotposturedescriber.h" // RobotPostureDescriber

namespace OpenRAVE {

using JointPtr = OpenRAVE::KinBody::JointPtr;

inline uint16_t compute_single_state(const double x, const double fTol) {
    return (x > fTol) ? 0 : (x < -fTol) ? 1 : 2; // >= or <= ?
}

template <size_t N>
inline void compute_robot_posture_states(const std::array<double, N>& posturevalues,
                                         const double fTol,
                                         std::vector<uint16_t>& posturestates) {
    std::array<uint16_t, N> singlestates;
    for(size_t i = 0; i < N; ++i) {
        singlestates[i] = compute_single_state(posturevalues[i], fTol);
    }

    posturestates = {0};
    posturestates.reserve(1 << N);
    for(size_t i = 0; i < N; ++i) {
        for(uint16_t &state : posturestates) {
            state <<= 1;
        }
        if(singlestates[i] == 1) {
            for(uint16_t &state : posturestates) {
                state |= 1;
            }
        }
        else if (singlestates[i] == 2) {
            const size_t ninds = posturestates.size();
            posturestates.insert(end(posturestates), begin(posturestates), end(posturestates));
            for(size_t j = ninds; j < 2 * ninds; ++j) {
                posturestates[j] |= 1;
            }
        }
    }
}

RobotPostureDescriber::RobotPostureDescriber(EnvironmentBasePtr penv,
                                            const double fTol) :
    RobotPostureDescriberBase(penv),
    _fTol(fTol) 
{
}

RobotPostureDescriber::~RobotPostureDescriber() {}

bool RobotPostureDescriber::Init(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) {
    if(!this->Supports(kinematicsChain)) {
        return false;
    }
    _GetJointsFromKinematicsChain(kinematicsChain, _vjoints);
    if(_vjoints.size() == 6) {
        _posturefn = Compute6RRobotPostureStates0;
    }
    else {
        _posturefn = Compute4DofRobotPostureStates0;
    }
    return static_cast<bool>(_posturefn);
}

void RobotPostureDescriber::_GetJointsFromKinematicsChain(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain,
                                                          std::vector<KinBody::JointPtr>& vjoints) const {
    const int baselinkid = kinematicsChain[0]->GetIndex();
    const int eelinkid = kinematicsChain[1]->GetIndex();
    const KinBodyPtr probot = kinematicsChain[0]->GetParent();
    probot->GetChain(baselinkid, eelinkid, vjoints);
}

bool RobotPostureDescriber::Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const {

    std::vector<KinBody::JointPtr> vjoints;
    _GetJointsFromKinematicsChain(kinematicsChain, vjoints);
    const size_t armdof = vjoints.size();
    if(armdof == 6) {
        return true;
    }
    const KinBodyPtr probot = kinematicsChain[0]->GetParent();
    const std::string robotname = probot->GetName();
    if(armdof == 4 && robotname == "okuma") {
        return true;
    }
    RAVELOG_WARN_FORMAT("Cannot handle robot %s with armdof=%d for now", robotname % armdof);
    return false;
}


bool RobotPostureDescriber::ComputePostureValues(std::vector<uint16_t>& posturestates, const std::vector<double>& jointvalues) const {
    _posturefn(_vjoints, _fTol, posturestates);
    return true;
}

void Compute6RRobotPostureStates0(const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates) {
    const Vector axis0 = vjoints[0]->GetAxis();
    const Vector axis1 = vjoints[1]->GetAxis();
    // const Vector axis2 = vjoints[2]->GetAxis(); // the same as axis1
    const Vector axis3 = vjoints[3]->GetAxis();
    const Vector axis4 = vjoints[4]->GetAxis();
    const Vector axis5 = vjoints[5]->GetAxis();
    const Vector anchor0 = vjoints[0]->GetAnchor();
    const Vector anchor1 = vjoints[1]->GetAnchor();
    const Vector anchor2 = vjoints[2]->GetAnchor();
    // const Vector anchor3 = vjoints[3]->GetAnchor();
    const Vector anchor4 = vjoints[4]->GetAnchor();
    // const Vector anchor5 = vjoints[5]->GetAnchor();

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

void Compute4DofRobotPostureStates0(const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates) {
    const Vector axis0 = vjoints[0]->GetAxis();
    const Vector axis1 = vjoints[1]->GetAxis();
    // const Vector axis2 = vjoints[2]->GetAxis();
    // const Vector axis3 = vjoints[3]->GetAxis();
    // const Vector anchor0 = vjoints[0]->GetAnchor();
    const Vector anchor1 = vjoints[1]->GetAnchor();
    const Vector anchor2 = vjoints[2]->GetAnchor();
    const Vector anchor3 = vjoints[3]->GetAnchor();

    const std::array<double, 2> posturevalues {
        // j1 pose: {{0, -1}, {1, -1}, {1, 3}}
        axis0.cross(axis1).dot(anchor3-anchor1),
        // elbow: {{1, -1}, {1, 2}, {2, 3}}
        axis1.cross(anchor2-anchor1).dot(anchor3-anchor2),
    };
    compute_robot_posture_states<2>(posturevalues, fTol, posturestates);
}

} // namespace OpenRAVE
