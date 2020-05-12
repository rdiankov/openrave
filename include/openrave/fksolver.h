// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
/** \file fksolver.h
    \brief Forward kinematics related definitions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_FKSOLVER_H
#define OPENRAVE_FKSOLVER_H

#include <openrave/openrave.h>

namespace OpenRAVE {

/** \brief <b>[interface]</b> Base class for all Forward Kinematic solvers. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_fksolver.
   \ingroup interfaces
 */
class OPENRAVE_API RobotPostureDescriberBase : public InterfaceBase
{
public:
    RobotPostureDescriberBase(EnvironmentBasePtr penv);
    virtual ~RobotPostureDescriberBase();

    /// \brief Initialize with a kinematics chain
    virtual bool Init(const std::array<OpenRAVE::RobotBase::LinkPtr, 2>& kinematicsChain);

    /// \brief Checks if we can use this describer to compute posture values from baselink to eelink prescribed by a kinematics chain
    /// \return true if can handle this kinematics chain
    virtual bool Supports(const std::array<OpenRAVE::RobotBase::LinkPtr, 2>& kinematicsChain) const;

    /// \brief Checks if we can use this describer to compute posture values from baselink to eelink prescribed by a manipulator
    /// \return true if can handle this kinematics chain
    virtual bool Supports(const RobotBase::ManipulatorPtr& pmanip) const final;

    /// \brief Computes an integer value to describe current robot posture
    /// Computes a value describing descrete posture of robot kinematics between base link and endeffector link
    virtual bool ComputePostureValues(std::vector<uint16_t>& posturevalues, const std::vector<double>& jointvalues = {}) const;

    virtual const std::vector<KinBody::JointPtr>& GetJoints() const final;

    /// \return the static interface type this class points to (used for safe casting)
    static InterfaceType GetInterfaceTypeStatic() {
        return PT_ForwardKinematicsSolver;
    }

private:
    virtual const char* GetHash() const final;

protected:
    std::vector<KinBody::JointPtr> _vjoints; ///< joints from baselink to eelink
};

} // end namespace OpenRAVE

#endif
