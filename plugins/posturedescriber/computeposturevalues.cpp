#include "posturedescriber.h"
#include "openraveplugindefs.h" // SerializeValues

namespace OpenRAVE {

using JointPtr = OpenRAVE::KinBody::JointPtr;

void ComputePostureStates6RGeneral(const std::vector<JointPtr>& joints, const double fTol, std::vector<uint16_t>& posturestates) {
    const Vector axis0 = joints[0]->GetAxis();
    const Vector axis1 = joints[1]->GetAxis();
    // const Vector axis2 = joints[2]->GetAxis(); // the same as axis1
    const Vector axis3 = joints[3]->GetAxis();
    const Vector axis4 = joints[4]->GetAxis();
    const Vector axis5 = joints[5]->GetAxis();
    const Vector anchor0 = joints[0]->GetAnchor();
    const Vector anchor1 = joints[1]->GetAnchor();
    const Vector anchor2 = joints[2]->GetAnchor();
    // const Vector anchor3 = joints[3]->GetAnchor();
    const Vector anchor4 = joints[4]->GetAnchor();
    // const Vector anchor5 = joints[5]->GetAnchor();

    const std::array<double, 3> posturevalues {
        // shoulder: {{0, -1}, {1, -1}, {0, 4}}
        axis0.cross(axis1).dot(anchor4-anchor0),
        // elbow: {{1, -1}, {1, 2}, {2, 4}}
        axis1.cross(anchor2-anchor1).dot(anchor4-anchor2),
        // wrist: {3, -1}, {4, -1}, {5, -1}}
        axis3.cross(axis4).dot(axis5)
    };
    compute_robot_posture_states<3>(posturevalues, fTol, posturestates);
    // std::stringstream ss;
    // ss << std::setprecision(16) << std::endl;
    // ss << "posturevalues = [";
    // SerializeValues(ss, posturevalues, ' ') << "]" << std::endl;
    // ss << "posturestates = [";
    // SerializeValues(ss, posturestates, ' ') << "]" << std::endl;
    // RAVELOG_WARN(ss.str());
}

void ComputePostureStates4RTypeA(const std::vector<JointPtr>& joints, const double fTol, std::vector<uint16_t>& posturestates) {
    const Vector axis0 = joints[0]->GetAxis();
    const Vector axis1 = joints[1]->GetAxis();
    // const Vector axis2 = joints[2]->GetAxis();
    // const Vector axis3 = joints[3]->GetAxis();
    // const Vector anchor0 = joints[0]->GetAnchor();
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

} // namespace OpenRAVE