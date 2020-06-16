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
#include "plugindefs.h" // POSTUREDESCRIBER_CLASS_NAME, POSTUREDESCRIBER_MODULE_NAME, POSTUREDESCRIBER_STATE_NAME

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
    RPST_NoSupport    = 0x0, ///< unsupported
    RPST_6R_General   = 0x1, ///< general 6R robots whose last three joint axes intersecting at a point
    RPST_4R_Type_A    = 0x2, ///< a special type of 4R robot whose last three parallel joint axes are perpendicular to the first joint axis
    RPST_RRR_Parallel = 0x3, ///< a type of robot that has only three revolute joints whose axes are parallel
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

using PostureValueFn = std::function<void(const std::vector<KinBody::JointPtr>& joints,
                                          const double fTol,
                                          std::vector<double>& posturevalues,
                                          std::vector<PostureStateInt>& featurestates,
                                          std::vector<PostureStateInt>& posturestates ///< most needed
                                          )>;

class OPENRAVE_API PostureDescriber : public PostureDescriberBase
{
public:
    PostureDescriber() = delete;
    PostureDescriber(const EnvironmentBasePtr& penv, const double fTol = 1e-6);
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

    /// \brief Gets the key used in map data (of type CustomData) in IkReturn
    virtual std::string GetMapDataKey() const override;

    /// \brief Gets essential kinematics chain associated with this describer
    virtual const LinkPair& GetEssentialKinematicsChain() const override;

    /// \brief Gets joints (with nonzero dofs) along the kinematics chain from baselink to eelink
    const std::vector<KinBody::JointPtr>& GetJoints() const {
        return _joints;
    }

    /// \brief Cleans internal setup after before calling Init
    virtual void Destroy() override;

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

    /// \brief Gets robot posture support type cast into int
    bool _GetSupportTypeCommand(std::ostream& ssout, std::istream& ssin) const;

    /// \brief Computes posture values
    bool _ComputePostureValuesCommand(std::ostream& ssout, std::istream& ssin);

    /* ========== `SendJSONCommand` APIs ========== */
    /// \brief `SendJSONCommand` API
    bool _InterpretJSONCommand(const rapidjson::Value& input, rapidjson::Value& output, rapidjson::Document::AllocatorType& allocator);

    LinkPair _kinematicsChain {nullptr, nullptr}; ///< the baselink-eelink pair of a kinematics chain
    std::vector<KinBody::JointPtr> _joints; ///< non-static joints from baselink to eelink
    std::vector<int> _armindices; ///< dof indices from baselink to eelink
    double _fTol = 1e-6; ///< tolerance for determining if a robot posture value is considered 0
    double _fGeometryTol = 4e-15;
    PostureValueFn _posturefn; ///< function that computes posture values and states for a kinematics chain
    RobotPostureSupportType _supporttype = RobotPostureSupportType::RPST_NoSupport;
    std::string _posturestatename = POSTUREDESCRIBER_STATE_NAME;

    /* ========== cached values ========== */
    std::vector<double>          _posturevalues; ///< cached posture values
    std::vector<PostureStateInt> _featurestates; ///< cached feature states
    std::vector<PostureStateInt> _posturestates; ///< cached posture states
};

using PostureDescriberPtr = boost::shared_ptr<PostureDescriber>;

} // namespace OpenRAVE

#endif // PLUGINS_POSTUREDESCRIBER_POSTUREDESCRIBERINTERFACE_H
