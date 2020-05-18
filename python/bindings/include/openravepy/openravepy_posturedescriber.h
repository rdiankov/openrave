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
#include <openrave/posturedescriberbase.h> // PostureDescriberBase
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_robotbase.h> // PyRobotBase::PyManipulatorPtr

namespace openravepy {

class PyPostureDescriber : public PyInterfaceBase
{
protected:
    PostureDescriberBasePtr _pDescriber;

public:
    PyPostureDescriber(PostureDescriberBasePtr pDescriber, PyEnvironmentBasePtr pyenv);
    ~PyPostureDescriber();

    PostureDescriberBasePtr GetPostureDescriber() const;

    /// \brief Initialize with a kinematics chain
    bool Supports(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink) const;

    /// \brief Initialize with a kinematics chain prescribed by a manipulator
    bool Supports(PyRobotBase::PyManipulatorPtr pmanip) const;

    /// \brief Initialize with a kinematics chain
    bool Init(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink);

    /// \brief Initialize with a kinematics chain prescribed by a manipulator
    bool Init(PyRobotBase::PyManipulatorPtr pmanip);
};

using PyPostureDescriberPtr = OPENRAVE_SHARED_PTR<PyPostureDescriber>;

} // openravepy

#endif // OPENRAVEPY_POSTUREDESCRIBER_H