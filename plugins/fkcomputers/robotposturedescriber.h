#ifndef PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBER_H
#define PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBER_H

#include <openrave/fksolver.h> // RobotPostureDescriberBasePtr

namespace OpenRAVE {

using PostureValueFn = std::function<void(const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates)>;

class OPENRAVE_API RobotPostureDescriber : public RobotPostureDescriberBase
{
public:
    RobotPostureDescriber() = delete;
    RobotPostureDescriber(EnvironmentBasePtr penv, const double fTol = 1e-6);
    virtual ~RobotPostureDescriber();

    /// \brief Initialize with a kinematics chain
    virtual bool Init(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) override;

    /// \brief Checks if this class can be used to compute posture values for this robot
    /// \return true if can handle this kinematics chain
    virtual bool Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const override;

    /// \brief Computes an integer value to describe current robot posture
    /// Computes a value describing descrete posture of robot kinematics between baselink and eelink
    virtual bool ComputePostureValues(std::vector<uint16_t>& values, const std::vector<double>& jointvalues = {}) override;

    /// \brief Set the tolerance for determining whether a robot posture value is close to 0 (i.e. singularity, branch point)
    bool SetPostureValueThreshold(const double fTol);

    const std::vector<KinBody::JointPtr>& GetJoints() const { return _joints; }

protected:
    /// \brief Gets joints along a kinematics chain from baselink to eelink
    void _GetJointsFromKinematicsChain(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain,
                                       std::vector<KinBody::JointPtr>& vjoints) const;

    /// \brief `SendCommand` APIs
    bool _SetPostureValueThresholdCommand(std::ostream& ssout, std::istream& ssin);
    bool _GetPostureValueThresholdCommand(std::ostream& ssout, std::istream& ssin) const;
    bool _GetArmIndicesCommand(std::ostream& ssout, std::istream& ssin) const;

    std::array<RobotBase::LinkPtr, 2> _kinematicsChain; ///< baselink and eelink
    std::vector<KinBody::JointPtr> _joints; ///< joints from baselink to eelink
    std::vector<int> _armindices; ///< dof indices from baselink to eelink
    double _fTol = 1e-6; ///< tolerance for computing robot posture values
    PostureValueFn _posturefn; ///< function that computes posture values and states for a kinematics chain
};

typedef boost::shared_ptr<RobotPostureDescriber> RobotPostureDescriberPtr;

OPENRAVE_API void Compute6RRobotPostureStates0  (const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates);
OPENRAVE_API void Compute4DofRobotPostureStates0(const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates);

} // namespace OpenRAVE

#endif // PLUGINS_FKCOMPUTERS_ROBOTPOSTUREDESCRIBER_H