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
#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_kinbody.h>
#include <openravepy/openravepy_environmentbase.h>
#include <openravepy/openravepy_physicalenginebase.h>

namespace openravepy {

using py::object;
using py::extract;
using py::extract_;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::init;
using py::scope_; // py::object if USE_PYBIND11_PYTHON_BINDINGS
using py::scope;
using py::args;
using py::return_value_policy;

#ifndef USE_PYBIND11_PYTHON_BINDINGS
using py::no_init;
using py::bases;
using py::copy_const_reference;
using py::docstring_options;
using py::pickle_suite;
using py::manage_new_object;
using py::def;
#endif // USE_PYBIND11_PYTHON_BINDINGS

namespace numeric = py::numeric;

PyPhysicsEngineBase::PyPhysicsEngineBase(PhysicsEngineBasePtr pPhysicsEngine, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pPhysicsEngine, pyenv),_pPhysicsEngine(pPhysicsEngine) {
}
PyPhysicsEngineBase::~PyPhysicsEngineBase() {
}

PhysicsEngineBasePtr PyPhysicsEngineBase::GetPhysicsEngine() {
    return _pPhysicsEngine;
}

bool PyPhysicsEngineBase::SetPhysicsOptions(int physicsoptions) {
    return _pPhysicsEngine->SetPhysicsOptions(physicsoptions);
}
int PyPhysicsEngineBase::GetPhysicsOptions() const {
    return _pPhysicsEngine->GetPhysicsOptions();
}
bool PyPhysicsEngineBase::InitEnvironment() {
    return _pPhysicsEngine->InitEnvironment();
}
void PyPhysicsEngineBase::DestroyEnvironment() {
    _pPhysicsEngine->DestroyEnvironment();
}
bool PyPhysicsEngineBase::InitKinBody(PyKinBodyPtr pbody) {
    CHECK_POINTER(pbody); return _pPhysicsEngine->InitKinBody(openravepy::GetKinBody(pbody));
}

bool PyPhysicsEngineBase::SetLinkVelocity(object pylink, object linearvel, object angularvel)
{
    CHECK_POINTER(pylink);
    return _pPhysicsEngine->SetLinkVelocity(openravepy::GetKinBodyLink(pylink),ExtractVector3(linearvel),ExtractVector3(angularvel));
}

bool PyPhysicsEngineBase::SetLinkVelocities(PyKinBodyPtr pykinbody, object ovelocities)
{
    std::vector<std::pair<Vector,Vector> > velocities;
    velocities.resize(len(ovelocities));
    for(size_t i = 0; i < velocities.size(); ++i) {
        std::vector<dReal> v = ExtractArray<dReal>(ovelocities[i]);
        BOOST_ASSERT(v.size()==6);
        velocities[i].first.x = v[0];
        velocities[i].first.y = v[1];
        velocities[i].first.z = v[2];
        velocities[i].second.x = v[3];
        velocities[i].second.y = v[4];
        velocities[i].second.z = v[5];
    }
    return _pPhysicsEngine->SetLinkVelocities(openravepy::GetKinBody(pykinbody), velocities);
}

object PyPhysicsEngineBase::GetLinkVelocity(object pylink)
{
    CHECK_POINTER(pylink);
    Vector linearvel, angularvel;
    if( !_pPhysicsEngine->GetLinkVelocity(openravepy::GetKinBodyLink(pylink),linearvel,angularvel) ) {
        return py::none_();
    }
    return py::make_tuple(toPyVector3(linearvel),toPyVector3(angularvel));
}

object PyPhysicsEngineBase::GetLinkVelocities(PyKinBodyPtr pykinbody)
{
    CHECK_POINTER(pykinbody);
    KinBodyPtr pbody = openravepy::GetKinBody(pykinbody);
    if( pbody->GetLinks().size() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<std::pair<Vector,Vector> > velocities;
    if( !_pPhysicsEngine->GetLinkVelocities(pbody,velocities) ) {
        return py::none_();
    }
    const size_t nvelocities = velocities.size();
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pyvel({(int) nvelocities, 6});
    py::buffer_info bufvel = pyvel.request();
    dReal* pfvel = (dReal*) bufvel.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = {npy_intp(nvelocities), 6};
    PyObject *pyvel = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pfvel = (dReal*)PyArray_DATA(pyvel);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    for(size_t i = 0; i < nvelocities; ++i) {
        pfvel[6*i+0] = velocities[i].first.x;
        pfvel[6*i+1] = velocities[i].first.y;
        pfvel[6*i+2] = velocities[i].first.z;
        pfvel[6*i+3] = velocities[i].second.x;
        pfvel[6*i+4] = velocities[i].second.y;
        pfvel[6*i+5] = velocities[i].second.z;
    }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return pyvel;
#else
    return py::to_array_astype<dReal>(pyvel);
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

bool PyPhysicsEngineBase::SetBodyForce(object pylink, object force, object position, bool bAdd)
{
    CHECK_POINTER(pylink);
    return _pPhysicsEngine->SetBodyForce(openravepy::GetKinBodyLink(pylink),ExtractVector3(force),ExtractVector3(position),bAdd);
}

bool PyPhysicsEngineBase::SetBodyTorque(object pylink, object torque, bool bAdd)
{
    CHECK_POINTER(pylink);
    return _pPhysicsEngine->SetBodyTorque(openravepy::GetKinBodyLink(pylink),ExtractVector3(torque),bAdd);
}

bool PyPhysicsEngineBase::AddJointTorque(object pyjoint, object torques)
{
    CHECK_POINTER(pyjoint);
    return _pPhysicsEngine->AddJointTorque(openravepy::GetKinBodyJoint(pyjoint),ExtractArray<dReal>(torques));
}

object PyPhysicsEngineBase::GetLinkForceTorque(object pylink)
{
    CHECK_POINTER(pylink);
    Vector force, torque;
    if( !_pPhysicsEngine->GetLinkForceTorque(openravepy::GetKinBodyLink(pylink),force,torque) ) {
        return py::none_();
    }
    return py::make_tuple(toPyVector3(force),toPyVector3(torque));
}

object PyPhysicsEngineBase::GetJointForceTorque(object pyjoint)
{
    CHECK_POINTER(pyjoint);
    Vector force, torque;
    if( !_pPhysicsEngine->GetJointForceTorque(openravepy::GetKinBodyJoint(pyjoint),force,torque) ) {
        return py::none_();
    }
    return py::make_tuple(toPyVector3(force),toPyVector3(torque));
}

void PyPhysicsEngineBase::SetGravity(object gravity) {
    _pPhysicsEngine->SetGravity(ExtractVector3(gravity));
}
object PyPhysicsEngineBase::GetGravity() {
    return toPyVector3(_pPhysicsEngine->GetGravity());
}

void PyPhysicsEngineBase::SimulateStep(dReal fTimeElapsed) {
    _pPhysicsEngine->SimulateStep(fTimeElapsed);
}

PhysicsEngineBasePtr GetPhysicsEngine(PyPhysicsEngineBasePtr pyPhysicsEngine)
{
    return !pyPhysicsEngine ? PhysicsEngineBasePtr() : pyPhysicsEngine->GetPhysicsEngine();
}

PyInterfaceBasePtr toPyPhysicsEngine(PhysicsEngineBasePtr pPhysicsEngine, PyEnvironmentBasePtr pyenv)
{
    return !pPhysicsEngine ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyPhysicsEngineBase(pPhysicsEngine,pyenv));
}

PyPhysicsEngineBasePtr RaveCreatePhysicsEngine(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    PhysicsEngineBasePtr p = OpenRAVE::RaveCreatePhysicsEngine(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyPhysicsEngineBasePtr();
    }
    return PyPhysicsEngineBasePtr(new PyPhysicsEngineBase(p,pyenv));
}

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_physicsengine(py::module& m)
#else
void init_openravepy_physicsengine()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
    class_<PyPhysicsEngineBase, OPENRAVE_SHARED_PTR<PyPhysicsEngineBase>, PyInterfaceBase>(m, "PhysicsEngine", DOXY_CLASS(PhysicsEngineBase))
    .def(init<PhysicsEngineBasePtr, PyEnvironmentBasePtr>(), "physicsengine"_a, "env"_a)
#else
    class_<PyPhysicsEngineBase, OPENRAVE_SHARED_PTR<PyPhysicsEngineBase>, bases<PyInterfaceBase> >("PhysicsEngine", DOXY_CLASS(PhysicsEngineBase), no_init)
#endif
    .def("GetPhysicsOptions",&PyPhysicsEngineBase::GetPhysicsOptions, DOXY_FN(PhysicsEngineBase,GetPhysicsOptions))
    .def("SetPhysicsOptions",&PyPhysicsEngineBase::SetPhysicsOptions, DOXY_FN(PhysicsEngineBase,SetPhysicsOptions "int"))
    .def("InitEnvironment",&PyPhysicsEngineBase::InitEnvironment, DOXY_FN(PhysicsEngineBase,InitEnvironment))
    .def("DestroyEnvironment",&PyPhysicsEngineBase::DestroyEnvironment, DOXY_FN(PhysicsEngineBase,DestroyEnvironment))
    .def("InitKinBody",&PyPhysicsEngineBase::InitKinBody, DOXY_FN(PhysicsEngineBase,InitKinBody))
    .def("SetLinkVelocity",&PyPhysicsEngineBase::SetLinkVelocity, PY_ARGS("link", "linearvel", "angularvel") DOXY_FN(PhysicsEngineBase,SetLinkVelocity))
    .def("SetLinkVelocities",&PyPhysicsEngineBase::SetLinkVelocities, PY_ARGS("body","velocities") DOXY_FN(PhysicsEngineBase,SetLinkVelocities))
    .def("GetLinkVelocity",&PyPhysicsEngineBase::GetLinkVelocity, DOXY_FN(PhysicsEngineBase,GetLinkVelocity))
    .def("GetLinkVelocities",&PyPhysicsEngineBase::GetLinkVelocity, DOXY_FN(PhysicsEngineBase,GetLinkVelocities))
    .def("SetBodyForce",&PyPhysicsEngineBase::SetBodyForce, DOXY_FN(PhysicsEngineBase,SetBodyForce))
    .def("SetBodyTorque",&PyPhysicsEngineBase::SetBodyTorque, DOXY_FN(PhysicsEngineBase,SetBodyTorque))
    .def("AddJointTorque",&PyPhysicsEngineBase::AddJointTorque, DOXY_FN(PhysicsEngineBase,AddJointTorque))
    .def("GetLinkForceTorque",&PyPhysicsEngineBase::GetLinkForceTorque, DOXY_FN(PhysicsEngineBase,GetLinkForceTorque))
    .def("GetJointForceTorque",&PyPhysicsEngineBase::GetJointForceTorque, DOXY_FN(PhysicsEngineBase,GetJointForceTorque))
    .def("SetGravity",&PyPhysicsEngineBase::SetGravity, DOXY_FN(PhysicsEngineBase,SetGravity))
    .def("GetGravity",&PyPhysicsEngineBase::GetGravity, DOXY_FN(PhysicsEngineBase,GetGravity))
    .def("SimulateStep",&PyPhysicsEngineBase::SimulateStep, DOXY_FN(PhysicsEngineBase,SimulateStep))
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreatePhysicsEngine",openravepy::RaveCreatePhysicsEngine, PY_ARGS("env","name") DOXY_FN1(RaveCreatePhysicsEngine));
#else
    def("RaveCreatePhysicsEngine",openravepy::RaveCreatePhysicsEngine, PY_ARGS("env","name") DOXY_FN1(RaveCreatePhysicsEngine));
#endif
}

}
