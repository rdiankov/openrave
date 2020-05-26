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

#ifndef RAVE_POSTUREDESCRIBER_PLUGINDEFS_H
#define RAVE_POSTUREDESCRIBER_PLUGINDEFS_H

#define POSTUREDESCRIBER_CLASS_NAME   "posturedescriber"  // PostureDescriber
#define POSTUREDESCRIBER_MODULE_NAME  "posturedescriber" // PostureDescriberModule

#include <openrave/posturedescriber.h> // PostureDescriberBasePtr

namespace OpenRAVE {

// https://stackoverflow.com/questions/12059774/c11-standard-conformant-bitmasks-using-enum-class
enum class NeighbouringTwoJointsRelation : uint16_t {
    NTJR_Unknown                 = 0x0,
    NTJR_Parallel                = 0x1,
    NTJR_Perpendicular           = 0x2,
    NTJR_Intersect               = 0x4,
    NTJR_Overlap                 = NTJR_Intersect | NTJR_Parallel,      // 0x5
    NTJR_Intersect_Perpendicular = NTJR_Intersect | NTJR_Perpendicular, // 0x6
};

enum class RobotPostureSupportType : uint16_t {
    RPST_NoSupport  = 0x0, ///< unsupported
    RPST_6R_General = 0x1, ///< general 6R robots with the last joint axes intersecting at a point
    RPST_4R_Type_A  = 0x2, ///< a special type of 4R robot the last three parallel joint axes perpendicular to the first joint axis
};

/// can do bit operations with enum class
template <typename T>
inline constexpr T operator&(T x, T y)
{
    using UT = typename std::underlying_type<T>::type;
    return static_cast<T>(static_cast<UT>(x) & static_cast<UT>(y));
}

template <typename T>
inline constexpr T operator|(T x, T y)
{
    using UT = typename std::underlying_type<T>::type;
    return static_cast<T>(static_cast<UT>(x) | static_cast<UT>(y));
}

template <typename T>
inline T operator&=(T& x, T y)
{
    return x = x & y;
}

template <typename T>
inline T operator|=(T& x, T y)
{
    return x = x | y;
}

using PostureValueFn = std::function<void(const std::vector<KinBody::JointPtr>& vjoints, const double fTol, std::vector<PostureStateInt>& posturestates)>;

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
    /// \param [in]  dofvalues       if empty, then use the current dof values; otherwise these specified dof values must have the same size as the number of dofs in the kinematics chain.
    /// \param [out] posturestates   posture states, whose size is a power of 2. Always non-empty if (1) this class is properly initialized AND (2) dofvalues is either empty or has the correct size.
    /// \return true if (1) this describer class is properly initialized AND (2) dofvalues is either empty or has the correct size.
    virtual bool ComputePostureStates(std::vector<PostureStateInt>& values, const std::vector<double>& dofvalues = {}) override;

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
inline PostureStateInt compute_single_state(const double x, const double fTol) {
    return (x > fTol) ? 0 : (x < -fTol) ? 1 : 2; // >= or <= ?
}

/// \brief Computes a vector of posture state integers using N posture values.
/// \param [in]  posturevalues    an array of posture values
/// \param [in]  tol              tolerance to determine whether x is considered 0.0, so that this value means a hybrid state.
/// \param [out] posturestates    a vector of posture state (unsigned) integers, whose size is always a power of 2
template <size_t N>
inline void compute_robot_posture_states(const std::array<double, N>& posturevalues,
                                         const double fTol,
                                         std::vector<PostureStateInt>& posturestates) {
    std::array<PostureStateInt, N> singlestates;
    for(size_t i = 0; i < N; ++i) {
        singlestates[i] = compute_single_state(posturevalues[i], fTol);
    }

    posturestates = {0};
    posturestates.reserve(1 << N);
    for(size_t i = 0; i < N; ++i) {
        for(PostureStateInt &state : posturestates) {
            state <<= 1;
        }
        if(singlestates[i] == 1) {
            for(PostureStateInt &state : posturestates) {
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

#endif // RAVE_POSTUREDESCRIBER_PLUGINDEFS_H
