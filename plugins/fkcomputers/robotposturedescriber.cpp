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
    const KinBodyPtr probot = kinematicsChain[0]->GetParent();
    probot->GetChain(baselinkid, eelinkid, _vjoints);
    return true;
}

bool RobotPostureDescriber::Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const {
    const int armdof = _vjoints.size();
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

bool RobotPostureDescriber::ComputePostureValues(std::vector<uint16_t>& posturevalues, const std::vector<double>& jointvalues) const {
    return true;
}

} // namespace OpenRAVE
