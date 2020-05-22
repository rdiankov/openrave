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

class OPENRAVEPY_API PyPostureDescriber : public PyInterfaceBase
{
protected:
    PostureDescriberBasePtr _pDescriber;

public:
    PyPostureDescriber(PostureDescriberBasePtr pDescriber, PyEnvironmentBasePtr pyenv);
    ~PyPostureDescriber();

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
    object ComputePostureStates();

    /// \brief Computes posture states at the input dof values using the describer set at the manipulator
    /// \return a py::list of posture states (integers) if a supportive posture describer is loaded onto the manipulator; else an empty list
    object ComputePostureStates(object pyjointvalues);

private:
    std::vector<PostureStateInt> _posturestates; // cache
};

using PyPostureDescriberPtr = OPENRAVE_SHARED_PTR<PyPostureDescriber>;
OPENRAVEPY_API PyPostureDescriberPtr GeneratePostureDescriber(const PyRobotBase::PyManipulatorPtr& pymanip);

// to-do, put to bindings.h
template <typename T>
inline py::list StdVectorToPyList(const std::vector<T>& v) {
    py::list l;
    const size_t N = v.size();
    for(size_t i = 0; i < N; i++) {
        l.append(v[i]);
    }
    return l;
};

} // openravepy

#endif // OPENRAVEPY_POSTUREDESCRIBER_H