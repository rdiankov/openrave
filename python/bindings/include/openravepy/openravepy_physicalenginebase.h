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
#ifndef OPENRAVEPY_INTERNAL_PHYSICSENGINE_H
#define OPENRAVEPY_INTERNAL_PHYSICSENGINE_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class PyPhysicsEngineBase : public PyInterfaceBase
{
protected:
    PhysicsEngineBasePtr _pPhysicsEngine;
public:
    PyPhysicsEngineBase(PhysicsEngineBasePtr pPhysicsEngine, PyEnvironmentBasePtr pyenv);
    virtual ~PyPhysicsEngineBase();
    PhysicsEngineBasePtr GetPhysicsEngine();

    bool SetPhysicsOptions(int physicsoptions);
    int GetPhysicsOptions() const;
    bool InitEnvironment();
    void DestroyEnvironment();
    bool InitKinBody(PyKinBodyPtr pbody);

    bool SetLinkVelocity(object pylink, object linearvel, object angularvel);

    bool SetLinkVelocities(PyKinBodyPtr pykinbody, object ovelocities);

    object GetLinkVelocity(object pylink);
    object GetLinkVelocities(PyKinBodyPtr pykinbody);

    bool SetBodyForce(object pylink, object force, object position, bool bAdd);

    bool SetBodyTorque(object pylink, object torque, bool bAdd);

    bool AddJointTorque(object pyjoint, object torques);

    object GetLinkForceTorque(object pylink);

    object GetJointForceTorque(object pyjoint);

    void SetGravity(object gravity);
    object GetGravity();

    void SimulateStep(dReal fTimeElapsed);
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_PHYSICSENGINE_H
