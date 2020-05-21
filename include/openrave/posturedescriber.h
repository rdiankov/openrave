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

using LinkPair = std::array<RobotBase::LinkPtr, 2>; ///< a baselink-eelink pair

/** \brief <b>[interface]</b> Base class for robot posture describers. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_fksolver.
   \ingroup interfaces
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
    /// \param [in]  jointvalues     if empty, then use the current dof values; otherwise these specified dof values must have the same size as the number of dofs in the kinematics chain.
    /// \param [out] posturestates   posture states, whose size is a power of 2. Always non-empty if (1) this class is properly initialized AND (2) jointvalues is either empty or has the correct size.
    /// \return true if (1) this describer class is properly initialized AND (2) jointvalues is either empty or has the correct size.
    virtual bool ComputePostureStates(std::vector<uint16_t>& posturestates, const std::vector<double>& jointvalues = {}) = 0;

    /// \return the static interface type this class points to (used for safe casting)
    static InterfaceType GetInterfaceTypeStatic() {
        return PT_PostureDescriber;
    }

private:
    virtual const char* GetHash() const final;
};

using PostureDescriberBasePtr = boost::shared_ptr<PostureDescriberBase>;

///< \brief Acquires from the manipulator the kinematics chain in the form of a baselink-eelink pair.
OPENRAVE_API LinkPair GetKinematicsChain(const RobotBase::ManipulatorPtr& pmanip);
OPENRAVE_API LinkPair GetKinematicsChain(const RobotBase::ManipulatorConstPtr& pmanip);

///< \brief Computes a kinematics hash from the baselink to eelink.
OPENRAVE_API std::string ComputeKinematicsChainHash(const LinkPair& kinematicsChain, std::vector<int>& armindices);

///< \brief Computes a kinematics hash from the manipulator's baselink to its eelink.
OPENRAVE_API std::string ComputeKinematicsChainHash(const RobotBase::ManipulatorPtr& pmanip, std::vector<int>& armindices);

} // end namespace OpenRAVE

#endif // OPENRAVE_POSTUREDESCRIBER_H
