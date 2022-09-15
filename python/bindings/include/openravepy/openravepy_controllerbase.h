// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVEPY_INTERNAL_CONTROLLERBASE_H
#define OPENRAVEPY_INTERNAL_CONTROLLERBASE_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class PyControllerBase : public PyInterfaceBase
{
protected:
    ControllerBasePtr _pcontroller;
public:
    PyControllerBase(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv);
    virtual ~PyControllerBase();

    ControllerBasePtr GetOpenRAVEController();
    bool Init(PyRobotBasePtr pyrobot, const string& PY_ARGS);

    bool Init(PyRobotBasePtr pyrobot, object odofindices, int nControlTransformation);

    object GetControlDOFIndices();
    int IsControlTransformation();
    object GetRobot();

    void Reset(int options=0);

    bool SetDesired(object o);

    bool SetDesired(object o, object otransform);
    bool SetPath(PyTrajectoryBasePtr pytraj);

    void SimulationStep(dReal fTimeElapsed);

    bool IsDone();
    dReal GetTime();

    object GetVelocity();

    object GetTorque();
};

class PyMultiControllerBase : public PyControllerBase
{
private:
    MultiControllerBasePtr _pmulticontroller;

public:
    PyMultiControllerBase(MultiControllerBasePtr pmulticontroller, PyEnvironmentBasePtr pyenv);
    virtual ~PyMultiControllerBase();

    bool AttachController(PyControllerBasePtr ocontroller, object odofindices, int nControlTransformation);

    void RemoveController(PyControllerBasePtr ocontroller);

    object GetController(int dof);
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_CONTROLLERBASE_H