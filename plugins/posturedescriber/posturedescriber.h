#ifndef PLUGINS_POSTUREDESCRIBER_POSTUREDESCRIBER_H
#define PLUGINS_POSTUREDESCRIBER_POSTUREDESCRIBER_H

#include <openrave/posturedescriber.h> // PostureDescriberBasePtr

namespace OpenRAVE {

using PostureValueFn = std::function<void(const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates)>;

class OPENRAVE_API PostureDescriber : public PostureDescriberBase
{
public:
    PostureDescriber() = delete;
    PostureDescriber(EnvironmentBasePtr penv, const double fTol = 1e-6);
    virtual ~PostureDescriber();

    /// \brief Initialize with a kinematics chain
    virtual bool Init(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) override;

    /// \brief Checks if this class can be used to compute posture values for this robot
    /// \return true if can handle this kinematics chain
    virtual bool Supports(const std::array<RobotBase::LinkPtr, 2>& kinematicsChain) const override;

    /// \brief Computes an integer value to describe current robot posture
    /// Computes a value describing descrete posture of robot kinematics between baselink and eelink
    virtual bool ComputePostureStates(std::vector<uint16_t>& values, const std::vector<double>& jointvalues = {}) override;

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

typedef boost::shared_ptr<PostureDescriber> PostureDescriberPtr;

OPENRAVE_API void ComputePostureStates6RGeneral  (const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates);
OPENRAVE_API void ComputePostureStates4RTypeA(const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates);

} // namespace OpenRAVE

#endif // PLUGINS_POSTUREDESCRIBER_POSTUREDESCRIBER_H