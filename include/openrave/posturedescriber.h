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

#ifndef OPENRAVE_POSTUREDESCRIBER_H
#define OPENRAVE_POSTUREDESCRIBER_H

#include <openrave/openrave.h>

namespace OpenRAVE {

using LinkPair = RobotBase::LinkPair; ///< a baselink-eelink pair
using PostureStateInt = RobotBase::PostureStateInt;

/** \brief <b>[interface]</b> Abstract, base class for robot posture describers.
 */

/** A base class for robot posture describers. For a kinematics chain of a robot, from the baselink to the end-effector link, we aim to use one or several (unsigned) integers to describe the posture of this kinematics chain. A description involves several "features" along the kinematics chain. Each feature usually depends on some geometric relations of certain joint axes and anchors, and thence have its feature "formulation". At a robot configuration, we plug in the set of joint values into the formulation, and compute a "posture value" of this feature. Comparing a posture value with some branch point (usually 0.0), we assign a "feature state" for this feature. A feature may have more than one feature state, provided we consider that a posture value close to a branch point within some tolerance (say tol=1.0e-6) can take both states from two sides of a branch point. Such a feature state is usually referred to as a "hybrid feature state". Combining states for all features, we derive the kinematics chain's "posture states". They are a set of unsigned integers, whose set size is always a power of 2: at least one state, at most 2^(number of features).

Usually we choose the floating-point number 0.0 as the branch point for a posture value. A feature state is "0" if its posture value is > tol, "1" if < -tol, and can take both states "0 and 1" if its absolute value is <= tol. The number of bits in a posture state integer is exactly the number of "features" for describing the kinematics chain. The feature closer to the robot's base has the more significant (higher) bit.

For example, to describe the kinematics chain of a general 6R (six-revolute-joint) robot, we use three features: shoulder, elbow, and wrist. The formulation of the wrist feature can be the triple product of J4's axis, J5's axis, and J6's axis, that is

dot(cross(J4's axis, J5's axis), J6's axis).

With joint values plugged into this formula, we compute a wrist (feature) posture value, and then determine its wrist feature state(s). Since among the three features, the wrist is the farthest from the base, the wrist feature state hence takes the least significant (lowest) bit in a posture state for the kinematics chain being described.

Here each posture state has three bits: the most significant bit is for the shoulder, the least significant bit for the wrist, the middle bit for the elbow feature. A posture state can take values 0, 1, ..., 7. Given a set of 6 joint values, the kinematics chain can have 1, or 2, or 4, or 8 posture states.

 */
class OPENRAVE_API PostureDescriberBase : public InterfaceBase
{
public:
    PostureDescriberBase(EnvironmentBasePtr penv);
    virtual ~PostureDescriberBase();

    /// \brief Checks if we can use a describer class to compute posture values for a kinematics chain from baselink to eelink.
    /// \return true if this describer class can support the posture description of this kinematics chain.
    virtual bool Supports(const LinkPair& kinematicsChain) const = 0;

    /// \brief Checks if we can use a describer class to compute posture values for a kinematics chain from the manipulator's baselink to its eelink.
    /// \return true if this describer class can support the posture description of this kinematics chain.
    virtual bool Supports(const RobotBase::ManipulatorPtr& pmanip) const;

    /// \brief Initializes class members for a kinematics chain from baselink to eelink, provided this class supports the posture description.
    /// \return true if this describer class can support the posture description AND the initialization is successful.
    virtual bool Init(const LinkPair& kinematicsChain) = 0;

    /// \brief Initializes class members for a kinematics chain from the manipulator's baselink to its eelink, provided this class supports the posture description.
    /// \return true if this describer class can support the posture description AND the initialization is successful.
    virtual bool Init(const RobotBase::ManipulatorPtr& pmanip);

    /// \brief Computes posture state integers for a kinematics chain at either the current of specified dof values.
    /// \param [in]  dofvalues       if empty, then use the current dof values; otherwise these specified dof values must have the same size as the number of dofs in the kinematics chain.
    /// \param [out] posturestates   posture states, whose size is a power of 2. Always non-empty if (1) this class is properly initialized AND (2) dofvalues is either empty or has the correct size.
    /// \return true if (1) this describer class is properly initialized AND (2) dofvalues is either empty or has the correct size.
    virtual bool ComputePostureStates(std::vector<PostureStateInt>& posturestates,
                                     const std::vector<double>& dofvalues = {},
                                     const KinBody::CheckLimitsAction claoption = KinBody::CheckLimitsAction::CLA_Nothing,
                                     const std::vector<int>& dofindices = {}
                                     ) = 0;

    /// \brief Gets the key used in map data (of type CustomData) in IkReturn
    virtual std::string GetMapDataKey() const = 0;

    /// \brief Cleans internal setup in the early stage of doing initializaion by Init()
    virtual void Destroy() = 0;

    /// \brief Gets essential kinematics chain associated with this describer
    virtual const LinkPair& GetEssentialKinematicsChain() const = 0;

    /// \return the static interface type this class points to (used for safe casting)
    static InterfaceType GetInterfaceTypeStatic() {
        return PT_PostureDescriber;
    }

private:
    virtual const char* GetHash() const final;
};

using PostureDescriberBasePtr = boost::shared_ptr<PostureDescriberBase>;

///< \brief Acquires from the manipulator the essential kinematics chain in the form of a link pair.
/// An essential kinematics chain, if not empty, is always such that both the first joint and the last joint are active and revolute.
OPENRAVE_API LinkPair ExtractEssentialKinematicsChain(const LinkPair& kinematicsChain);
OPENRAVE_API LinkPair ExtractEssentialKinematicsChain(const RobotBase::ManipulatorPtr& pmanip);
OPENRAVE_API LinkPair ExtractEssentialKinematicsChain(const RobotBase::ManipulatorConstPtr& pmanip);

///< \brief Acquires from the manipulator the baselink-eelink pair.
OPENRAVE_API LinkPair GetKinematicsChain(const RobotBase::ManipulatorConstPtr& pmanip);

///< \brief Computes a kinematics hash from the baselink to eelink.
OPENRAVE_API std::string ComputeKinematicsChainHash(const LinkPair& kinematicsChain, std::vector<int>& armindices);

///< \brief Computes a kinematics hash from the manipulator's baselink to its eelink.
OPENRAVE_API std::string ComputeKinematicsChainHash(const RobotBase::ManipulatorPtr& pmanip, std::vector<int>& armindices);

} // end namespace OpenRAVE

#endif // OPENRAVE_POSTUREDESCRIBER_H
