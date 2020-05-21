// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Guangning Tan, Kei Usui, Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef PLUGINS_POSTUREDESCRIBER_POSTUREDESCRIBERINTERFACE_H
#define PLUGINS_POSTUREDESCRIBER_POSTUREDESCRIBERINTERFACE_H

#include <openrave/posturedescriber.h> // PostureDescriberBasePtr
#include "posturesupporttype.h" // NeighbouringTwoJointsRelation, RobotPostureSupportType

namespace OpenRAVE {

using PostureValueFn = std::function<void(const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<uint16_t>& posturestates)>;

class OPENRAVE_API PostureDescriber : public PostureDescriberBase
{
public:
    PostureDescriber() = delete;
    PostureDescriber(EnvironmentBasePtr penv, const double fTol = 1e-6);
    virtual ~PostureDescriber();

    /// \brief Checks if we can use this describer class to compute posture values for a kinematics chain from baselink to eelink.
    /// \return true if this describer class can support the posture description of this kinematics chain.
    virtual bool Supports(const LinkPair& kinematicsChain) const override;

    /// \brief Initializes class members for a kinematics chain from baselink to eelink, provided this class supports the posture description.
    /// \return true if this describer class can support the posture description AND the initialization is successful.
    virtual bool Init(const LinkPair& kinematicsChain) override;

    /// \brief Computes posture state integers for a kinematics chain at either the current of specified dof values.
    /// \param [in]  jointvalues     if empty, then use the current dof values; otherwise these specified dof values must have the same size as the number of dofs in the kinematics chain.
    /// \param [out] posturestates   posture states, whose size is a power of 2. Always non-empty if (1) this class is properly initialized AND (2) jointvalues is either empty or has the correct size.
    /// \return true if (1) this describer class is properly initialized AND (2) jointvalues is either empty or has the correct size.
    virtual bool ComputePostureStates(std::vector<uint16_t>& values, const std::vector<double>& jointvalues = {}) override;

    /// \brief Sets the tolerance for determining whether a robot posture value (shoulder, elbow, wrist, etc.) is close to 0
    bool SetPostureValueThreshold(const double fTol);

    const std::vector<KinBody::JointPtr>& GetJoints() const {
        return _joints;
    }

protected:
    /// \brief Gets joints along a kinematics chain from baselink to eelink
    void _GetJointsFromKinematicsChain(const LinkPair& kinematicsChain,
                                       std::vector<KinBody::JointPtr>& vjoints) const;

    /* ========== `SendCommand` APIs ========== */
    /// \brief Sets the tolerance for determining whether a robot posture value (shoulder, elbow, wrist, etc.) is close to 0
    bool _SetPostureValueThresholdCommand(std::ostream& ssout, std::istream& ssin);

    /// \brief Gets the tolerance for determining whether a robot posture value (shoulder, elbow, wrist, etc.) is close to 0
    bool _GetPostureValueThresholdCommand(std::ostream& ssout, std::istream& ssin) const;

    /// \brief Gets the dof indices along a kinematics chain from baselink to eelink
    bool _GetArmIndicesCommand(std::ostream& ssout, std::istream& ssin) const;

    LinkPair _kinematicsChain; ///< the baselink-eelink pair of a kinematics chain
    std::vector<KinBody::JointPtr> _joints; ///< non-static joints from baselink to eelink
    std::vector<int> _armindices; ///< dof indices from baselink to eelink
    double _fTol = 1e-6; ///< tolerance for determining if a robot posture value is considered 0
    PostureValueFn _posturefn; ///< function that computes posture values and states for a kinematics chain
};

using PostureDescriberPtr = boost::shared_ptr<PostureDescriber>;
using PostureFormulation = std::array<std::array<int, 2>, 3>; ///< a posture value is computed by a triple product

/// \brief determines whether a robot posture value can be considered as 0.0, postive, or negative
/// \param [in] x      a posture value
/// \param [in] tol    tolerance to determine whether x is considered 0.0, so that this value means a hybrid state.
/// \return 0 if x is considered positive, 1 if considered negative, and 2 (meaning hybrid states) if considered 0.0
inline uint16_t compute_single_state(const double x, const double fTol) {
    return (x > fTol) ? 0 : (x < -fTol) ? 1 : 2; // >= or <= ?
}

/// \brief Computes a vector of posture state integers using N posture values.
/// \param [in]  posturevalues    an array of posture values
/// \param [in]  tol              tolerance to determine whether x is considered 0.0, so that this value means a hybrid state.
/// \param [out] posturestates    a vector of posture state (unsigned) integers, whose size is always a power of 2
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
            const size_t nstates = posturestates.size();
            posturestates.insert(end(posturestates), begin(posturestates), end(posturestates));
            for(size_t j = nstates; j < 2 * nstates; ++j) {
                posturestates[j] |= 1;
            }
        }
    }
}
} // namespace OpenRAVE

#endif // PLUGINS_POSTUREDESCRIBER_POSTUREDESCRIBERINTERFACE_H