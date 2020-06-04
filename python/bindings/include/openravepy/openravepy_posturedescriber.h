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
#ifndef OPENRAVEPY_POSTUREDESCRIBER_H
#define OPENRAVEPY_POSTUREDESCRIBER_H

#define NO_IMPORT_ARRAY
#include <openrave/posturedescriber.h> // PostureDescriberBase
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_robotbase.h> // PyRobotBase::PyManipulatorPtr

namespace openravepy {

class OPENRAVEPY_API PyPostureDescriberBase : public PyInterfaceBase
{
protected:
    PostureDescriberBasePtr _pDescriber;

public:
    PyPostureDescriberBase() = delete;
    PyPostureDescriberBase(PostureDescriberBasePtr pDescriber, PyEnvironmentBasePtr pyenv);
    ~PyPostureDescriberBase();

    /// \brief Gets the underlying posture describer pointer
    PostureDescriberBasePtr GetPostureDescriber() const;

    /// \brief Initializes with a kinematics chain from baselink to eelink
    /// \return true if there exists a posture describer that supports the kinematics chain from baselink to eelink
    bool Supports(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink) const;

    /// \brief Initializes with a kinematics chain prescribed by a manipulator
    /// \return true if there exists a posture describer that supports the kinematics chain for the manipulator
    bool Supports(PyRobotBase::PyManipulatorPtr pmanip) const;

    /// \brief Initializes with a kinematics chain from baselink to eelink
    /// \return true if a posture describer is successfully initialized for the kinematics chain from baselink to eelink
    bool Init(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink);

    /// \brief Initializes with a kinematics chain prescribed by a manipulator
    /// \return true if a posture describer is successfully initialized for the kinematics chain for the manipulator
    bool Init(PyRobotBase::PyManipulatorPtr pmanip);

    /// \brief Computes posture states at the current dof values using the describer set at the manipulator
    /// \return a py::list of posture states (integers) if a supportive posture describer is loaded onto the manipulator; else an empty list
    py::object ComputePostureStates();

    /// \brief Computes posture states at the input dof values using the describer set at the manipulator
    /// \return a py::list of posture states (integers) if a supportive posture describer is loaded onto the manipulator; else an empty list
    py::object ComputePostureStates(py::object pyjointvalues);

    /// \brief Gets the "solution indices name" we use in the custom data map of IkReturn
    std::string GetMapDataKey() const;

    /// \brief Interprets what a posture state means in the describer
    py::object Interpret(const PostureStateInt state) const;

private:
    std::vector<PostureStateInt> _posturestates; // cache
};

using PyPostureDescriberBasePtr = OPENRAVE_SHARED_PTR<PyPostureDescriberBase>;
/// \brief generates a posture describer specified by interface name, and load automatically if required
OPENRAVEPY_API PyPostureDescriberBasePtr GeneratePostureDescriber(const PyRobotBase::PyManipulatorPtr& pymanip,
                                                                  std::string interfacename = "",
                                                                  const bool load = false);
} // openravepy

#endif // OPENRAVEPY_POSTUREDESCRIBER_H