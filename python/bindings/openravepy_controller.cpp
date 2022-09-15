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
#include <openravepy/openravepy_environmentbase.h>
#include <openravepy/openravepy_robotbase.h>
#include <openravepy/openravepy_trajectorybase.h>
#include <openravepy/openravepy_controllerbase.h>

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

PyControllerBase::PyControllerBase(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pcontroller, pyenv), _pcontroller(pcontroller) {
}
PyControllerBase::~PyControllerBase() {
}

ControllerBasePtr PyControllerBase::GetOpenRAVEController() {
    return _pcontroller;
}

bool PyControllerBase::Init(PyRobotBasePtr pyrobot, const string& PY_ARGS)
{
    RAVELOG_WARN("PyControllerBase::Init(robot,PY_ARGS) deprecated!\n");
    CHECK_POINTER(pyrobot);
    RobotBasePtr probot = openravepy::GetRobot(pyrobot);
    std::vector<int> dofindices;
    for(int i = 0; i < probot->GetDOF(); ++i) {
        dofindices.push_back(i);
    }
    return _pcontroller->Init(probot, dofindices,1);
}

bool PyControllerBase::Init(PyRobotBasePtr pyrobot, object odofindices, int nControlTransformation)
{
    CHECK_POINTER(pyrobot);
    std::vector<int> dofindices = ExtractArray<int>(odofindices);
    return _pcontroller->Init(openravepy::GetRobot(pyrobot),dofindices,nControlTransformation);
}

object PyControllerBase::GetControlDOFIndices() {
    return toPyArray(_pcontroller->GetControlDOFIndices());
}
int PyControllerBase::IsControlTransformation() {
    return _pcontroller->IsControlTransformation();
}
object PyControllerBase::GetRobot()
{
    return py::to_object(openravepy::toPyRobot(_pcontroller->GetRobot(),_pyenv));
}

void PyControllerBase::Reset(int options) {
    _pcontroller->Reset(options);
}

bool PyControllerBase::SetDesired(object o)
{
    std::vector<dReal> values = ExtractArray<dReal>(o);
    if( values.size() == 0 ) {
        throw openrave_exception(_("no values specified"));
    }
    return _pcontroller->SetDesired(values);
}

bool PyControllerBase::SetDesired(object o, object otransform)
{
    if( IS_PYTHONOBJECT_NONE(otransform) ) {
        return SetDesired(o);
    }
    return _pcontroller->SetDesired(ExtractArray<dReal>(o),TransformConstPtr(new Transform(ExtractTransform(otransform))));
}

bool PyControllerBase::SetPath(PyTrajectoryBasePtr pytraj)
{
    CHECK_POINTER(pytraj);
    return _pcontroller->SetPath (openravepy::GetTrajectory(pytraj));
}

void PyControllerBase::SimulationStep(dReal fTimeElapsed) {
    _pcontroller->SimulationStep(fTimeElapsed);
}

bool PyControllerBase::IsDone() {
    return _pcontroller->IsDone();
}
dReal PyControllerBase::GetTime() {
    return _pcontroller->GetTime();
}

object PyControllerBase::GetVelocity()
{
    std::vector<dReal> velocity;
    _pcontroller->GetVelocity(velocity);
    return toPyArray(velocity);
}

object PyControllerBase::GetTorque()
{
    std::vector<dReal> torque;
    _pcontroller->GetTorque(torque);
    return toPyArray(torque);
}

PyMultiControllerBase::PyMultiControllerBase(MultiControllerBasePtr pmulticontroller, PyEnvironmentBasePtr pyenv) : PyControllerBase(pmulticontroller, pyenv), _pmulticontroller(pmulticontroller) {
}
PyMultiControllerBase::~PyMultiControllerBase() {
}

bool PyMultiControllerBase::AttachController(PyControllerBasePtr ocontroller, object odofindices, int nControlTransformation) {
    CHECK_POINTER(ocontroller);
    std::vector<int> dofindices = ExtractArray<int>(odofindices);
    return _pmulticontroller->AttachController(ocontroller->GetOpenRAVEController(), dofindices, nControlTransformation);
}

void PyMultiControllerBase::RemoveController(PyControllerBasePtr ocontroller) {
    CHECK_POINTER(ocontroller);
    _pmulticontroller->RemoveController(ocontroller->GetOpenRAVEController());
}

object PyMultiControllerBase::GetController(int dof) {
    CHECK_POINTER(_pmulticontroller);
    ControllerBasePtr pcontroller = _pmulticontroller->GetController(dof);
    return py::to_object(openravepy::toPyController(pcontroller, _pyenv));
}

ControllerBasePtr GetController(PyControllerBasePtr pycontroller)
{
    return !pycontroller ? ControllerBasePtr() : pycontroller->GetOpenRAVEController();
}

PyInterfaceBasePtr toPyController(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv)
{
    if( !pcontroller )  {
        return PyInterfaceBasePtr();
    }
    // TODO this is a hack
    // unfortunately dynamic_pointer_cast will not work. The most ideal situation is to have MultiControllerBase registered as its own individual interface....
    else if( pcontroller->GetXMLId() == std::string("MultiController") ) {
        return PyInterfaceBasePtr(new PyMultiControllerBase(OPENRAVE_STATIC_POINTER_CAST<MultiControllerBase>(pcontroller), pyenv));
    }
    else {
        return PyInterfaceBasePtr(new PyControllerBase(pcontroller, pyenv));
    }
}

PyControllerBasePtr RaveCreateController(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    ControllerBasePtr pcontroller = OpenRAVE::RaveCreateController(GetEnvironment(pyenv), name);
    if( !pcontroller ) {
        return PyControllerBasePtr();
    }
    return PyControllerBasePtr(new PyControllerBase(pcontroller, pyenv));
}

PyMultiControllerBasePtr RaveCreateMultiController(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    MultiControllerBasePtr pcontroller = OpenRAVE::RaveCreateMultiController(GetEnvironment(pyenv), name);
    if( !pcontroller ) {
        return PyMultiControllerBasePtr();
    }
    return PyMultiControllerBasePtr(new PyMultiControllerBase(pcontroller, pyenv));
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Reset_overloads, Reset, 0, 1)
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_controller(py::module& m)
#else
void init_openravepy_controller()
#endif
{
    {
        bool (PyControllerBase::*init1)(PyRobotBasePtr,const string &) = &PyControllerBase::Init;
        bool (PyControllerBase::*init2)(PyRobotBasePtr,object,int) = &PyControllerBase::Init;
        bool (PyControllerBase::*setdesired1)(object) = &PyControllerBase::SetDesired;
        bool (PyControllerBase::*setdesired2)(object,object) = &PyControllerBase::SetDesired;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        using namespace py::literals;  // "..."_a
        class_<PyControllerBase, OPENRAVE_SHARED_PTR<PyControllerBase>, PyInterfaceBase>(m, "Controller", DOXY_CLASS(ControllerBase))
#else
        class_<PyControllerBase, OPENRAVE_SHARED_PTR<PyControllerBase>, bases<PyInterfaceBase> >("Controller", DOXY_CLASS(ControllerBase), no_init)
#endif
        .def("Init",init1, DOXY_FN(ControllerBase,Init))
        .def("Init",init2, PY_ARGS("robot","dofindices","controltransform") DOXY_FN(ControllerBase,Init))
        .def("GetControlDOFIndices",&PyControllerBase::GetControlDOFIndices,DOXY_FN(ControllerBase,GetControlDOFIndices))
        .def("IsControlTransformation",&PyControllerBase::IsControlTransformation, DOXY_FN(ControllerBase,IsControlTransformation))
        .def("GetRobot",&PyControllerBase::GetRobot, DOXY_FN(ControllerBase,GetRobot))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("Reset", &PyControllerBase::Reset,
            "options"_a = 0,
            DOXY_FN(ControllerBase, Reset)
        )
#else
        .def("Reset",&PyControllerBase::Reset, Reset_overloads(PY_ARGS("options") DOXY_FN(ControllerBase,Reset)))
#endif
        .def("SetDesired",setdesired1, PY_ARGS("values") DOXY_FN(ControllerBase,SetDesired))
        .def("SetDesired",setdesired2, PY_ARGS("values","transform") DOXY_FN(ControllerBase,SetDesired))
        .def("SetPath",&PyControllerBase::SetPath, DOXY_FN(ControllerBase,SetPath))
        .def("SimulationStep",&PyControllerBase::SimulationStep, DOXY_FN(ControllerBase,SimulationStep "dReal"))
        .def("IsDone",&PyControllerBase::IsDone, DOXY_FN(ControllerBase,IsDone))
        .def("GetTime",&PyControllerBase::GetTime, DOXY_FN(ControllerBase,GetTime))
        .def("GetVelocity",&PyControllerBase::GetVelocity, DOXY_FN(ControllerBase,GetVelocity))
        .def("GetTorque",&PyControllerBase::GetTorque, DOXY_FN(ControllerBase,GetTorque))
        ;
    }

    {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyMultiControllerBase, OPENRAVE_SHARED_PTR<PyMultiControllerBase>, PyControllerBase/*, PyInterfaceBase*/ >(m, "MultiController", DOXY_CLASS(MultiControllerBase))
#else
        class_<PyMultiControllerBase, OPENRAVE_SHARED_PTR<PyMultiControllerBase>, bases<PyControllerBase, PyInterfaceBase> >("MultiController", DOXY_CLASS(MultiControllerBase), no_init)
#endif
        .def("AttachController",&PyMultiControllerBase::AttachController, PY_ARGS("controller","dofindices","controltransform") DOXY_FN(MultiControllerBase,AttachController))
        .def("RemoveController",&PyMultiControllerBase::RemoveController, PY_ARGS("controller") DOXY_FN(MultiControllerBase,RemoveController))
        .def("GetController",&PyMultiControllerBase::GetController, PY_ARGS("dof") DOXY_FN(MultiControllerBase,GetController))
        ;
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreateController",openravepy::RaveCreateController, PY_ARGS("env","name") DOXY_FN1(RaveCreateController));
    m.def("RaveCreateMultiController",openravepy::RaveCreateMultiController, PY_ARGS("env","name") DOXY_FN1(RaveCreateMultiController));
#else
    def("RaveCreateController",openravepy::RaveCreateController, PY_ARGS("env","name") DOXY_FN1(RaveCreateController));
    def("RaveCreateMultiController",openravepy::RaveCreateMultiController, PY_ARGS("env","name") DOXY_FN1(RaveCreateMultiController));
#endif
}

}
