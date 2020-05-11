#include "robotposturedescriber.h" // RobotPostureDescriber

namespace OpenRAVE {

using JointPtr = OpenRAVE::KinBody::JointPtr;

RobotPostureDescriber::RobotPostureDescriber(EnvironmentBasePtr penv,
                                            const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) :
    RobotPostureDescriberBase(penv) {
    this->Init(kinematicsChain);
}

RobotPostureDescriber::~RobotPostureDescriber() {}

bool RobotPostureDescriber::Init(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) {
    const int baselinkid = kinematicsChain[0]->GetIndex();
    const int eelinkid = kinematicsChain[1]->GetIndex();
    const KinBodyPtr pbody = kinematicsChain[0]->GetParent();
    pbody->GetChain(baselinkid, eelinkid, _vjoints);
    return true;
}

bool RobotPostureDescriber::Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const {
    return true;
}

bool RobotPostureDescriber::ComputePostureValue(std::vector<uint16_t>& values) const {
    return true;
}

} // namespace OpenRAVE
