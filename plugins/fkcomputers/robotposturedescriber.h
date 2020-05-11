#ifndef PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBER_H
#define PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBER_H

#include <openrave/fksolver.h> // RobotPostureDescriberBasePtr

namespace OpenRAVE {

class OPENRAVE_API RobotPostureDescriber : public RobotPostureDescriberBase
{
public:
    RobotPostureDescriber(EnvironmentBasePtr penv);
    virtual ~RobotPostureDescriber();

    /// \brief Initialize with a kinematics chain
    virtual bool Init(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain);

    /// \brief Checks if this class can be used to compute posture values for this robot
    /// \return true if can handle this kinematics chain
    virtual bool Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const;

    /// \brief Computes an integer value to describe current robot posture
    /// Computes a value describing descrete posture of robot kinematics between base link and endeffector link
    virtual bool ComputePostureValue(std::vector<uint16_t>& values) const;
};

} // namespace OpenRAVE

#endif // PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBER_H