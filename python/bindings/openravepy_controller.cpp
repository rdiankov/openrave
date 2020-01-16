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
#include "openravepy_int.h"

namespace openravepy {

using py::object;
using py::extract;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::no_init;
using py::bases;
using py::init;
using py::scope;
using py::args;
using py::return_value_policy;
using py::copy_const_reference;
using py::docstring_options;
using py::def;
using py::pickle_suite;
namespace numeric = py::numeric;

class PyControllerBase : public PyInterfaceBase
{
protected:
    ControllerBasePtr _pcontroller;
public:
    PyControllerBase(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pcontroller, pyenv), _pcontroller(pcontroller) {
    }
    virtual ~PyControllerBase() {
    }

    ControllerBasePtr GetOpenRAVEController() {
        return _pcontroller;
    }

    bool Init(PyRobotBasePtr pyrobot, const string& args)
    {
        RAVELOG_WARN("PyControllerBase::Init(robot,args) deprecated!\n");
        CHECK_POINTER(pyrobot);
        RobotBasePtr probot = openravepy::GetRobot(pyrobot);
        std::vector<int> dofindices;
        for(int i = 0; i < probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _pcontroller->Init(probot, dofindices,1);
    }

    bool Init(PyRobotBasePtr pyrobot, object odofindices, int nControlTransformation)
    {
        CHECK_POINTER(pyrobot);
        vector<int> dofindices = ExtractArray<int>(odofindices);
        return _pcontroller->Init(openravepy::GetRobot(pyrobot),dofindices,nControlTransformation);
    }

    object GetControlDOFIndices() {
        return toPyArray(_pcontroller->GetControlDOFIndices());
    }
    int IsControlTransformation() {
        return _pcontroller->IsControlTransformation();
    }
    object GetRobot()
    {
        return object(openravepy::toPyRobot(_pcontroller->GetRobot(),_pyenv));
    }

    void Reset(int options=0) {
        _pcontroller->Reset(options);
    }

    bool SetDesired(object o)
    {
        vector<dReal> values = ExtractArray<dReal>(o);
        if( values.size() == 0 ) {
            throw openrave_exception(_("no values specified"));
        }
        return _pcontroller->SetDesired(values);
    }

    bool SetDesired(object o, object otransform)
    {
        if( IS_PYTHONOBJECT_NONE(otransform) ) {
            return SetDesired(o);
        }
        return _pcontroller->SetDesired(ExtractArray<dReal>(o),TransformConstPtr(new Transform(ExtractTransform(otransform))));
    }

    bool SetPath(PyTrajectoryBasePtr pytraj)
    {
        CHECK_POINTER(pytraj);
        return _pcontroller->SetPath (openravepy::GetTrajectory(pytraj));
    }

    void SimulationStep(dReal fTimeElapsed) {
        _pcontroller->SimulationStep(fTimeElapsed);
    }

    bool IsDone() {
        return _pcontroller->IsDone();
    }
    dReal GetTime() {
        return _pcontroller->GetTime();
    }

    object GetVelocity()
    {
        vector<dReal> velocity;
        _pcontroller->GetVelocity(velocity);
        return toPyArray(velocity);
    }

    object GetTorque()
    {
        vector<dReal> torque;
        _pcontroller->GetTorque(torque);
        return toPyArray(torque);
    }
};

class PyMultiControllerBase : public PyControllerBase
{
private:
    MultiControllerBasePtr _pmulticontroller;

public:
    PyMultiControllerBase(MultiControllerBasePtr pmulticontroller, PyEnvironmentBasePtr pyenv) : PyControllerBase(pmulticontroller, pyenv), _pmulticontroller(pmulticontroller) {
    }
    virtual ~PyMultiControllerBase() {
    }

    bool AttachController(PyControllerBasePtr ocontroller, object odofindices, int nControlTransformation) {
        CHECK_POINTER(ocontroller);
        vector<int> dofindices = ExtractArray<int>(odofindices);
        return _pmulticontroller->AttachController(ocontroller->GetOpenRAVEController(), dofindices, nControlTransformation);
    }

    void RemoveController(PyControllerBasePtr ocontroller) {
        CHECK_POINTER(ocontroller);
        _pmulticontroller->RemoveController(ocontroller->GetOpenRAVEController());
    }

    object GetController(int dof) {
        CHECK_POINTER(_pmulticontroller);
        ControllerBasePtr pcontroller = _pmulticontroller->GetController(dof);
        return object(openravepy::toPyController(pcontroller, _pyenv));
    }
};

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
        return PyInterfaceBasePtr(new PyMultiControllerBase(boost::static_pointer_cast<MultiControllerBase>(pcontroller), pyenv));
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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Reset_overloads, Reset, 0, 1)

void init_openravepy_controller()
{
    {
        bool (PyControllerBase::*init1)(PyRobotBasePtr,const string &) = &PyControllerBase::Init;
        bool (PyControllerBase::*init2)(PyRobotBasePtr,object,int) = &PyControllerBase::Init;
        bool (PyControllerBase::*setdesired1)(object) = &PyControllerBase::SetDesired;
        bool (PyControllerBase::*setdesired2)(object,object) = &PyControllerBase::SetDesired;
        class_<PyControllerBase, boost::shared_ptr<PyControllerBase>, bases<PyInterfaceBase> >("Controller", DOXY_CLASS(ControllerBase), no_init)
        .def("Init",init1, DOXY_FN(ControllerBase,Init))
        .def("Init",init2, args("robot","dofindices","controltransform"), DOXY_FN(ControllerBase,Init))
        .def("GetControlDOFIndices",&PyControllerBase::GetControlDOFIndices,DOXY_FN(ControllerBase,GetControlDOFIndices))
        .def("IsControlTransformation",&PyControllerBase::IsControlTransformation, DOXY_FN(ControllerBase,IsControlTransformation))
        .def("GetRobot",&PyControllerBase::GetRobot, DOXY_FN(ControllerBase,GetRobot))
        .def("Reset",&PyControllerBase::Reset, Reset_overloads(args("options"), DOXY_FN(ControllerBase,Reset)))
        .def("SetDesired",setdesired1, args("values"), DOXY_FN(ControllerBase,SetDesired))
        .def("SetDesired",setdesired2, args("values","transform"), DOXY_FN(ControllerBase,SetDesired))
        .def("SetPath",&PyControllerBase::SetPath, DOXY_FN(ControllerBase,SetPath))
        .def("SimulationStep",&PyControllerBase::SimulationStep, DOXY_FN(ControllerBase,SimulationStep "dReal"))
        .def("IsDone",&PyControllerBase::IsDone, DOXY_FN(ControllerBase,IsDone))
        .def("GetTime",&PyControllerBase::GetTime, DOXY_FN(ControllerBase,GetTime))
        .def("GetVelocity",&PyControllerBase::GetVelocity, DOXY_FN(ControllerBase,GetVelocity))
        .def("GetTorque",&PyControllerBase::GetTorque, DOXY_FN(ControllerBase,GetTorque))
        ;
    }

    {
        class_<PyMultiControllerBase, boost::shared_ptr<PyMultiControllerBase>, bases<PyControllerBase, PyInterfaceBase> >("MultiController", DOXY_CLASS(MultiControllerBase), no_init)
        .def("AttachController",&PyMultiControllerBase::AttachController, args("controller","dofindices","controltransform"), DOXY_FN(MultiControllerBase,AttachController))
        .def("RemoveController",&PyMultiControllerBase::RemoveController, args("controller"), DOXY_FN(MultiControllerBase,RemoveController))
        .def("GetController",&PyMultiControllerBase::GetController, args("dof"), DOXY_FN(MultiControllerBase,GetController))
        ;
    }

    def("RaveCreateController",openravepy::RaveCreateController,args("env","name"),DOXY_FN1(RaveCreateController));
    def("RaveCreateMultiController",openravepy::RaveCreateMultiController,args("env","name"),DOXY_FN1(RaveCreateMultiController));
}

}
