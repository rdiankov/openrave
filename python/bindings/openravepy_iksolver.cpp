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
#include <openrave/utils.h>

namespace openravepy {

class PyIkReturn
{
public:
    PyIkReturn(const IkReturn& ret) : _ret(ret) {
    }
    PyIkReturn(IkReturnPtr pret) : _ret(*pret) {
    }
    PyIkReturn(IkReturnAction action) : _ret(action) {
    }
    IkReturnAction GetAction() {
        return _ret._action;
    }
    object GetSolution() {
        return toPyArray(_ret._vsolution);
    }
    object GetUserData() {
        return openravepy::GetUserData(_ret._userdata);
    }
    object GetMapData(const std::string& key) {
        IkReturn::CustomData::const_iterator it = _ret._mapdata.find(key);
        if( it == _ret._mapdata.end() ) {
            return object();
        }
        return toPyArray(it->second);
    }
    object GetMapDataDict() {
        boost::python::dict odata;
        FOREACHC(it,_ret._mapdata) {
            odata[it->first] = toPyArray(it->second);
        }
        return odata;
    }

    void SetUserData(PyUserData pdata) {
        _ret._userdata = pdata._handle;
    }
    void SetSolution(object osolution) {
        _ret._vsolution = ExtractArray<dReal>(osolution);
    }
    void SetMapKeyValue(const std::string& key, object ovalues) {
        _ret._mapdata[key] = ExtractArray<dReal>(ovalues);
    }

    IkReturn _ret;
};

typedef boost::shared_ptr<PyIkReturn> PyIkReturnPtr;

class PyIkSolverBase : public PyInterfaceBase
{
protected:
    IkSolverBasePtr _pIkSolver;

    static IkReturn _CallCustomFilter(object fncallback, PyEnvironmentBasePtr pyenv, IkSolverBasePtr pIkSolver, std::vector<dReal>& values, RobotBase::ManipulatorConstPtr pmanip, const IkParameterization& ikparam)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        std::string errmsg;
        try {
            RobotBase::ManipulatorPtr pmanip2 = boost::const_pointer_cast<RobotBase::Manipulator>(pmanip);
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
                extract<IkReturnAction> ikfra(res);
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

public:
    PyIkSolverBase(IkSolverBasePtr pIkSolver, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pIkSolver, pyenv), _pIkSolver(pIkSolver) {
    }
    virtual ~PyIkSolverBase() {
    }

    IkSolverBasePtr GetIkSolver() {
        return _pIkSolver;
    }

    int GetNumFreeParameters() const {
        return _pIkSolver->GetNumFreeParameters();
    }
    object GetFreeParameters() const {
        if( _pIkSolver->GetNumFreeParameters() == 0 ) {
            return numpy::array(boost::python::list());
        }
        vector<dReal> values;
        _pIkSolver->GetFreeParameters(values);
        return toPyArray(values);
    }

    PyIkReturnPtr Solve(object oparam, object oq0, int filteroptions)
    {
        PyIkReturnPtr pyreturn(new PyIkReturn(IKRA_Reject));
        IkReturnPtr preturn(&pyreturn->_ret, utils::null_deleter());
        vector<dReal> q0;
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

    object SolveAll(object oparam, int filteroptions)
    {
        boost::python::list pyreturns;
        std::vector<IkReturnPtr> vikreturns;
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
        }
        _pIkSolver->SolveAll(ikparam, filteroptions, vikreturns);
        FOREACH(itikreturn,vikreturns) {
            pyreturns.append(object(PyIkReturnPtr(new PyIkReturn(*itikreturn))));
        }
        return pyreturns;
    }

    PyIkReturnPtr Solve(object oparam, object oq0, object oFreeParameters, int filteroptions)
    {
        PyIkReturnPtr pyreturn(new PyIkReturn(IKRA_Reject));
        IkReturnPtr preturn(&pyreturn->_ret, utils::null_deleter());
        vector<dReal> q0, vFreeParameters;
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

    object SolveAll(object oparam, object oFreeParameters, int filteroptions)
    {
        boost::python::list pyreturns;
        std::vector<IkReturnPtr> vikreturns;
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
        }
        vector<dReal> vFreeParameters;
        if( !IS_PYTHONOBJECT_NONE(oFreeParameters) ) {
            vFreeParameters = ExtractArray<dReal>(oFreeParameters);
        }
        _pIkSolver->SolveAll(ikparam, vFreeParameters, filteroptions, vikreturns);
        FOREACH(itikreturn,vikreturns) {
            pyreturns.append(object(PyIkReturnPtr(new PyIkReturn(*itikreturn))));
        }
        return pyreturns;
    }

    PyIkReturnPtr CallFilters(object oparam)
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

    bool Supports(IkParameterizationType type) {
        return _pIkSolver->Supports(type);
    }

    object RegisterCustomFilter(int priority, object fncallback)
    {
        if( !fncallback ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("callback not specified"),ORE_InvalidArguments);
        }
        return toPyUserData(_pIkSolver->RegisterCustomFilter(priority,boost::bind(&PyIkSolverBase::_CallCustomFilter,fncallback,_pyenv,_pIkSolver,_1,_2,_3)));
    }
};

bool ExtractIkReturn(object o, IkReturn& ikfr)
{
    extract<PyIkReturnPtr > pyikfr(o);
    if( pyikfr.check() ) {
        ikfr = ((PyIkReturnPtr)pyikfr)->_ret;
        return true;
    }
    return false;
}

object toPyIkReturn(const IkReturn& ret)
{
    return object(PyIkReturnPtr(new PyIkReturn(ret)));
}

IkSolverBasePtr GetIkSolver(PyIkSolverBasePtr pyIkSolver)
{
    return !pyIkSolver ? IkSolverBasePtr() : pyIkSolver->GetIkSolver();
}

PyInterfaceBasePtr toPyIkSolver(IkSolverBasePtr pIkSolver, PyEnvironmentBasePtr pyenv)
{
    return !pIkSolver ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyIkSolverBase(pIkSolver,pyenv));
}

PyIkSolverBasePtr RaveCreateIkSolver(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    IkSolverBasePtr p = OpenRAVE::RaveCreateIkSolver(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyIkSolverBasePtr();
    }
    return PyIkSolverBasePtr(new PyIkSolverBase(p,pyenv));
}

void init_openravepy_iksolver()
{
    enum_<IkFilterOptions>("IkFilterOptions" DOXY_ENUM(IkFilterOptions))
    .value("CheckEnvCollisions",IKFO_CheckEnvCollisions)
    .value("IgnoreSelfCollisions",IKFO_IgnoreSelfCollisions)
    .value("IgnoreJointLimits",IKFO_IgnoreJointLimits)
    .value("IgnoreCustomFilters",IKFO_IgnoreCustomFilters)
    .value("IgnoreEndEffectorCollisions",IKFO_IgnoreEndEffectorCollisions)
    .value("IgnoreEndEffectorEnvCollisions",IKFO_IgnoreEndEffectorEnvCollisions)
    .value("IgnoreEndEffectorSelfCollisions",IKFO_IgnoreEndEffectorSelfCollisions)
    ;

    enum_<IkReturnAction>("IkReturnAction" DOXY_ENUM(IkReturnAction))
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
        scope ikreturn = class_<PyIkReturn, PyIkReturnPtr>("IkReturn", DOXY_CLASS(IkReturn), no_init)
                         .def(init<IkReturnAction>(args("action")))
                         .def("GetAction",&PyIkReturn::GetAction, "Retuns IkReturn::_action")
                         .def("GetSolution",&PyIkReturn::GetSolution, "Retuns IkReturn::_vsolution")
                         .def("GetUserData",&PyIkReturn::GetUserData, "Retuns IkReturn::_userdata")
                         .def("GetMapData",&PyIkReturn::GetMapData, args("key"), "Indexes into the map and returns an array of numbers. If key doesn't exist, returns None")
                         .def("GetMapDataDict",&PyIkReturn::GetMapDataDict, "Returns a dictionary copy for IkReturn::_mapdata")
                         .def("SetUserData",&PyIkReturn::SetUserData,args("data"),"Set IKReturn::_userdata")
                         .def("SetSolution",&PyIkReturn::SetSolution,args("solution"),"Set IKReturn::_vsolution")
                         .def("SetMapKeyValue",&PyIkReturn::SetMapKeyValue,args("key,value"),"Adds key/value pair to IKReturn::_mapdata")
        ;
    }

    {
        PyIkReturnPtr (PyIkSolverBase::*Solve)(object, object, int) = &PyIkSolverBase::Solve;
        PyIkReturnPtr (PyIkSolverBase::*SolveFree)(object, object, object, int) = &PyIkSolverBase::Solve;
        object (PyIkSolverBase::*SolveAll)(object, int) = &PyIkSolverBase::SolveAll;
        object (PyIkSolverBase::*SolveAllFree)(object, object, int) = &PyIkSolverBase::SolveAll;
        class_<PyIkSolverBase, boost::shared_ptr<PyIkSolverBase>, bases<PyInterfaceBase> >("IkSolver", DOXY_CLASS(IkSolverBase), no_init)
        .def("Solve",Solve,args("ikparam","q0","filteroptions"), DOXY_FN(IkSolverBase, Solve "const IkParameterization&; const std::vector; int; IkReturnPtr"))
        .def("Solve",SolveFree,args("ikparam","q0","freeparameters", "filteroptions"), DOXY_FN(IkSolverBase, Solve "const IkParameterization&; const std::vector; const std::vector; int; IkReturnPtr"))
        .def("SolveAll",SolveAll,args("ikparam","filteroptions"), DOXY_FN(IkSolverBase, SolveAll "const IkParameterization&; int; std::vector<IkReturnPtr>"))
        .def("SolveAll",SolveAllFree,args("ikparam","freeparameters","filteroptions"), DOXY_FN(IkSolverBase, SolveAll "const IkParameterization&; const std::vector; int; std::vector<IkReturnPtr>"))
        .def("GetNumFreeParameters",&PyIkSolverBase::GetNumFreeParameters, DOXY_FN(IkSolverBase,GetNumFreeParameters))
        .def("GetFreeParameters",&PyIkSolverBase::GetFreeParameters, DOXY_FN(IkSolverBase,GetFreeParameters))
        .def("Supports",&PyIkSolverBase::Supports, args("iktype"), DOXY_FN(IkSolverBase,Supports))
        .def("CallFilters",&PyIkSolverBase::CallFilters, args("ikparam"), DOXY_FN(IkSolverBase,CallFilters))
        .def("RegisterCustomFilter",&PyIkSolverBase::RegisterCustomFilter, args("priority","callback"), DOXY_FN(IkSolverBase,RegisterCustomFilter))
        ;
    }

    def("RaveCreateIkSolver",openravepy::RaveCreateIkSolver,args("env","name"),DOXY_FN1(RaveCreateIkSolver));
}

}
