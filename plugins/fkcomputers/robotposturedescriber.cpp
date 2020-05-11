#include "robotposturedescriber.h" // RobotPostureDescriber

namespace OpenRAVE {

RobotPostureDescriber::RobotPostureDescriber(EnvironmentBasePtr penv,
                                            const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) :
    RobotPostureDescriberBase(penv) {
    this->Init(kinematicsChain);
}

RobotPostureDescriber::~RobotPostureDescriber() {}

bool RobotPostureDescriber::Init(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) {
    return true;
}

bool RobotPostureDescriber::Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const {
    return true;
}

bool RobotPostureDescriber::ComputePostureValue(std::vector<uint16_t>& values) const {
    return true;
}

} // namespace OpenRAVE
