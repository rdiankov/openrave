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
#include <openravepy/openravepy_iksolverbase.h>
#include <openrave/utils.h>

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

PyIkReturn::PyIkReturn(const IkReturn& ret) : _ret(ret) {
}
PyIkReturn::PyIkReturn(IkReturnPtr pret) : _ret(*pret) {
}
PyIkReturn::PyIkReturn(IkReturnAction action) : _ret(action) {
}
IkReturnAction PyIkReturn::GetAction() {
    return _ret._action;
}
object PyIkReturn::GetSolution() {
    return toPyArray(_ret._vsolution);
}
object PyIkReturn::GetUserData() {
    return openravepy::GetUserData(_ret._userdata);
}
object PyIkReturn::GetMapData(const std::string& key) {
    IkReturn::CustomData::const_iterator it = _ret._mapdata.find(key);
    if( it == _ret._mapdata.end() ) {
        return py::none_();
    }
    return toPyArray(it->second);
}
object PyIkReturn::GetMapDataDict() {
    py::dict odata;
    FOREACHC(it,_ret._mapdata) {
        odata[it->first] = toPyArray(it->second);
    }
    return odata;
}

void PyIkReturn::SetUserData(PyUserData pdata) {
    _ret._userdata = pdata._handle;
}
void PyIkReturn::SetSolution(object osolution) {
    _ret._vsolution = ExtractArray<dReal>(osolution);
}
void PyIkReturn::SetMapKeyValue(const std::string& key, object ovalues) {
    _ret._mapdata[key] = ExtractArray<dReal>(ovalues);
}

typedef OPENRAVE_SHARED_PTR<PyIkReturn> PyIkReturnPtr;

IkReturn PyIkSolverBase::_CallCustomFilter(object fncallback, PyEnvironmentBasePtr pyenv, IkSolverBasePtr pIkSolver, std::vector<dReal>& values, RobotBase::ManipulatorConstPtr pmanip, const IkParameterization& ikparam)
{
    object res;
    PyGILState_STATE gstate = PyGILState_Ensure();
    std::string errmsg;
    try {
        RobotBase::ManipulatorPtr pmanip2 = OPENRAVE_CONST_POINTER_CAST<RobotBase::Manipulator>(pmanip);
        res = fncallback(toPyArray(values), openravepy::toPyRobotManipulator(pmanip2,pyenv),toPyIkParameterization(ikparam));
    }
    catch(...) {
        errmsg = boost::str(boost::format("exception occured in python custom filter callback of iksolver %s: %s")%pIkSolver->GetXMLId()%GetPyErrorString());
    }
    IkReturn ikfr(IKRA_Success);
    if( IS_PYTHONOBJECT_NONE(res) ) {
        ikfr._action = IKRA_Reject;
    }
    else {
        if( !openravepy::ExtractIkReturn(res,ikfr) ) {
            extract_<IkReturnAction> ikfra(res);
            if( ikfra.check() ) {
                ikfr._action = (IkReturnAction)ikfra;
            }
            else {
                errmsg = "failed to convert return type of filter to IkReturn";
            }
        }
    }

    PyGILState_Release(gstate);
    if( errmsg.size() > 0 ) {
        throw openrave_exception(errmsg,ORE_Assert);
    }
    return ikfr;
}

PyIkSolverBase::PyIkSolverBase(IkSolverBasePtr pIkSolver, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pIkSolver, pyenv), _pIkSolver(pIkSolver) {
}
PyIkSolverBase::~PyIkSolverBase() {
}

IkSolverBasePtr PyIkSolverBase::GetIkSolver() {
    return _pIkSolver;
}

int PyIkSolverBase::GetNumFreeParameters() const {
    return _pIkSolver->GetNumFreeParameters();
}
object PyIkSolverBase::GetFreeParameters() const {
    if( _pIkSolver->GetNumFreeParameters() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _pIkSolver->GetFreeParameters(values);
    return toPyArray(values);
}

PyIkReturnPtr PyIkSolverBase::Solve(object oparam, object oq0, int filteroptions)
{
    PyIkReturnPtr pyreturn(new PyIkReturn(IKRA_Reject));
    IkReturnPtr preturn(&pyreturn->_ret, utils::null_deleter());
    std::vector<dReal> q0;
    if( !IS_PYTHONOBJECT_NONE(oq0) ) {
        q0 = ExtractArray<dReal>(oq0);
    }
    IkParameterization ikparam;
    if( !ExtractIkParameterization(oparam,ikparam) ) {
        throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
    }
    _pIkSolver->Solve(ikparam, q0, filteroptions, preturn);
    return pyreturn;
}

object PyIkSolverBase::SolveAll(object oparam, int filteroptions)
{
    py::list pyreturns;
    std::vector<IkReturnPtr> vikreturns;
    IkParameterization ikparam;
    if( !ExtractIkParameterization(oparam,ikparam) ) {
        throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
    }
    _pIkSolver->SolveAll(ikparam, filteroptions, vikreturns);
    FOREACH(itikreturn,vikreturns) {
        pyreturns.append(py::to_object(PyIkReturnPtr(new PyIkReturn(*itikreturn))));
    }
    return pyreturns;
}

PyIkReturnPtr PyIkSolverBase::Solve(object oparam, object oq0, object oFreeParameters, int filteroptions)
{
    PyIkReturnPtr pyreturn(new PyIkReturn(IKRA_Reject));
    IkReturnPtr preturn(&pyreturn->_ret, utils::null_deleter());
    std::vector<dReal> q0, vFreeParameters;
    if( !IS_PYTHONOBJECT_NONE(oq0) ) {
        q0 = ExtractArray<dReal>(oq0);
    }
    if( !IS_PYTHONOBJECT_NONE(oFreeParameters) ) {
        vFreeParameters = ExtractArray<dReal>(oFreeParameters);
    }
    IkParameterization ikparam;
    if( !ExtractIkParameterization(oparam,ikparam) ) {
        throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
    }
    _pIkSolver->Solve(ikparam, q0, vFreeParameters,filteroptions, preturn);
    return pyreturn;
}

object PyIkSolverBase::SolveAll(object oparam, object oFreeParameters, int filteroptions)
{
    py::list pyreturns;
    std::vector<IkReturnPtr> vikreturns;
    IkParameterization ikparam;
    if( !ExtractIkParameterization(oparam,ikparam) ) {
        throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
    }
    std::vector<dReal> vFreeParameters;
    if( !IS_PYTHONOBJECT_NONE(oFreeParameters) ) {
        vFreeParameters = ExtractArray<dReal>(oFreeParameters);
    }
    _pIkSolver->SolveAll(ikparam, vFreeParameters, filteroptions, vikreturns);
    FOREACH(itikreturn,vikreturns) {
        pyreturns.append(py::to_object(PyIkReturnPtr(new PyIkReturn(*itikreturn))));
    }
    return pyreturns;
}

PyIkReturnPtr PyIkSolverBase::CallFilters(object oparam)
{
    PyIkReturnPtr pyreturn(new PyIkReturn(IKRA_Reject));
    IkReturnPtr preturn(&pyreturn->_ret, utils::null_deleter());
    IkParameterization ikparam;
    if( !ExtractIkParameterization(oparam,ikparam) ) {
        throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
    }
    _pIkSolver->CallFilters(ikparam, preturn);
    return pyreturn;
}

bool PyIkSolverBase::Supports(IkParameterizationType type) {
    return _pIkSolver->Supports(type);
}

object PyIkSolverBase::RegisterCustomFilter(int priority, object fncallback)
{
    if( !fncallback ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("callback not specified"),ORE_InvalidArguments);
    }
    return toPyUserData(_pIkSolver->RegisterCustomFilter(priority,boost::bind(&PyIkSolverBase::_CallCustomFilter,fncallback,_pyenv,_pIkSolver,_1,_2,_3)));
}

bool ExtractIkReturn(object o, IkReturn& ikfr)
{
    extract_<PyIkReturnPtr > pyikfr(o);
    if( pyikfr.check() ) {
        ikfr = ((PyIkReturnPtr)pyikfr)->_ret;
        return true;
    }
    return false;
}

object toPyIkReturn(const IkReturn& ret)
{
    return py::to_object(PyIkReturnPtr(new PyIkReturn(ret)));
}

IkSolverBasePtr GetIkSolver(object oiksolver)
{
    extract_<PyIkSolverBasePtr> pyiksolver(oiksolver);
    if( pyiksolver.check() ) {
        return ((PyIkSolverBasePtr)pyiksolver)->GetIkSolver();
    }
    return IkSolverBasePtr();
}

IkSolverBasePtr GetIkSolver(PyIkSolverBasePtr pyIkSolver)
{
    return !pyIkSolver ? IkSolverBasePtr() : pyIkSolver->GetIkSolver();
}

PyInterfaceBasePtr toPyIkSolver(IkSolverBasePtr pIkSolver, PyEnvironmentBasePtr pyenv)
{
    return !pIkSolver ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyIkSolverBase(pIkSolver,pyenv));
}

object toPyIkSolver(IkSolverBasePtr pIkSolver, object opyenv)
{
    extract_<PyEnvironmentBasePtr> pyenv(opyenv);
    if( pyenv.check() ) {
        return py::to_object(toPyIkSolver(pIkSolver,(PyEnvironmentBasePtr)pyenv));
    }
    return py::none_();
}

PyIkSolverBasePtr RaveCreateIkSolver(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    IkSolverBasePtr p = OpenRAVE::RaveCreateIkSolver(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyIkSolverBasePtr();
    }
    return PyIkSolverBasePtr(new PyIkSolverBase(p,pyenv));
}

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_iksolver(py::module& m)
#else
void init_openravepy_iksolver()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals;  // "..."_a
    enum_<IkFilterOptions>(m, "IkFilterOptions", py::arithmetic() DOXY_ENUM(IkFilterOptions))
#else
    enum_<IkFilterOptions>("IkFilterOptions" DOXY_ENUM(IkFilterOptions))
#endif
    .value("CheckEnvCollisions",IKFO_CheckEnvCollisions)
    .value("IgnoreSelfCollisions",IKFO_IgnoreSelfCollisions)
    .value("IgnoreJointLimits",IKFO_IgnoreJointLimits)
    .value("IgnoreCustomFilters",IKFO_IgnoreCustomFilters)
    .value("IgnoreEndEffectorCollisions",IKFO_IgnoreEndEffectorCollisions)
    .value("IgnoreEndEffectorEnvCollisions",IKFO_IgnoreEndEffectorEnvCollisions)
    .value("IgnoreEndEffectorSelfCollisions",IKFO_IgnoreEndEffectorSelfCollisions)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<IkReturnAction>(m, "IkReturnAction" DOXY_ENUM(IkReturnAction))
#else
    enum_<IkReturnAction>("IkReturnAction" DOXY_ENUM(IkReturnAction))
#endif
    .value("Success",IKRA_Success)
    .value("Reject",IKRA_Reject)
    .value("Quit",IKRA_Quit)
    .value("QuitEndEffectorCollision",IKRA_QuitEndEffectorCollision)
    .value("RejectKinematics",IKRA_RejectKinematics)
    .value("RejectSelfCollision",IKRA_RejectSelfCollision)
    .value("RejectEnvCollision",IKRA_RejectEnvCollision)
    .value("RejectJointLimits",IKRA_RejectJointLimits)
    .value("RejectKinematicsPrecision",IKRA_RejectKinematicsPrecision)
    .value("RejectCustomFilter",IKRA_RejectCustomFilter)
    ;

    {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ ikreturn = class_<PyIkReturn, PyIkReturnPtr>(m, "IkReturn", DOXY_CLASS(IkReturn))
                                 .def(init<IkReturnAction>(), "action"_a)
#else
        scope_ ikreturn = class_<PyIkReturn, PyIkReturnPtr>("IkReturn", DOXY_CLASS(IkReturn), no_init)
                                 .def(init<IkReturnAction>(py::args("action")))
#endif
                         .def("GetAction",&PyIkReturn::GetAction, "Retuns IkReturn::_action")
                         .def("GetSolution",&PyIkReturn::GetSolution, "Retuns IkReturn::_vsolution")
                         .def("GetUserData",&PyIkReturn::GetUserData, "Retuns IkReturn::_userdata")
                         .def("GetMapData",&PyIkReturn::GetMapData, PY_ARGS("key") "Indexes into the map and returns an array of numbers. If key doesn't exist, returns None")
                         .def("GetMapDataDict",&PyIkReturn::GetMapDataDict, "Returns a dictionary copy for IkReturn::_mapdata")
                         .def("SetUserData",&PyIkReturn::SetUserData,PY_ARGS("data") "Set IKReturn::_userdata")
                         .def("SetSolution",&PyIkReturn::SetSolution,PY_ARGS("solution") "Set IKReturn::_vsolution")
                         .def("SetMapKeyValue",&PyIkReturn::SetMapKeyValue,PY_ARGS("key", "value") "Adds key/value pair to IKReturn::_mapdata")
        ;
    }

    {
        PyIkReturnPtr (PyIkSolverBase::*Solve)(object, object, int) = &PyIkSolverBase::Solve;
        PyIkReturnPtr (PyIkSolverBase::*SolveFree)(object, object, object, int) = &PyIkSolverBase::Solve;
        object (PyIkSolverBase::*SolveAll)(object, int) = &PyIkSolverBase::SolveAll;
        object (PyIkSolverBase::*SolveAllFree)(object, object, int) = &PyIkSolverBase::SolveAll;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyIkSolverBase, OPENRAVE_SHARED_PTR<PyIkSolverBase>, PyInterfaceBase>(m, "IkSolver", DOXY_CLASS(IkSolverBase))
#else
        class_<PyIkSolverBase, OPENRAVE_SHARED_PTR<PyIkSolverBase>, bases<PyInterfaceBase> >("IkSolver", DOXY_CLASS(IkSolverBase), no_init)
#endif
        .def("Solve",Solve, PY_ARGS("ikparam","q0","filteroptions") DOXY_FN(IkSolverBase, Solve "const IkParameterization&; const std::vector; int; IkReturnPtr"))
        .def("Solve",SolveFree, PY_ARGS("ikparam","q0","freeparameters", "filteroptions") DOXY_FN(IkSolverBase, Solve "const IkParameterization&; const std::vector; const std::vector; int; IkReturnPtr"))
        .def("SolveAll",SolveAll, PY_ARGS("ikparam","filteroptions") DOXY_FN(IkSolverBase, SolveAll "const IkParameterization&; int; std::vector<IkReturnPtr>"))
        .def("SolveAll",SolveAllFree, PY_ARGS("ikparam","freeparameters","filteroptions") DOXY_FN(IkSolverBase, SolveAll "const IkParameterization&; const std::vector; int; std::vector<IkReturnPtr>"))
        .def("GetNumFreeParameters",&PyIkSolverBase::GetNumFreeParameters, DOXY_FN(IkSolverBase,GetNumFreeParameters))
        .def("GetFreeParameters",&PyIkSolverBase::GetFreeParameters, DOXY_FN(IkSolverBase,GetFreeParameters))
        .def("Supports",&PyIkSolverBase::Supports, PY_ARGS("iktype") DOXY_FN(IkSolverBase,Supports))
        .def("CallFilters",&PyIkSolverBase::CallFilters, PY_ARGS("ikparam") DOXY_FN(IkSolverBase,CallFilters))
        .def("RegisterCustomFilter",&PyIkSolverBase::RegisterCustomFilter, PY_ARGS("priority","callback") DOXY_FN(IkSolverBase,RegisterCustomFilter))
        ;
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreateIkSolver", openravepy::RaveCreateIkSolver, PY_ARGS("env","name") DOXY_FN1(RaveCreateIkSolver));
#else
    def("RaveCreateIkSolver",openravepy::RaveCreateIkSolver, PY_ARGS("env","name") DOXY_FN1(RaveCreateIkSolver));
#endif
}

}
