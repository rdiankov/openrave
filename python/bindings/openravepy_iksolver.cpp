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

class PyIkReturn
{
public:
    PyIkReturn(IkReturn& ret) : _ret(ret) {
    }
    PyIkReturn(IkReturnAction action) : _ret(action) {
    }
    IkReturnAction GetAction() {
        return _ret._action;
    }
    object GetUserData() {
        return openravepy::GetUserData(_ret._userdata);
    }
    object GetMapData() {
        boost::python::dict odata;
        FOREACHC(it,_ret._mapdata) {
            odata[it->first] = toPyArray(it->second);
        }
        return odata;
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
        PyGILState_Release(gstate);
        if( errmsg.size() > 0 ) {
            throw openrave_exception(errmsg,ORE_Assert);
        }
        if( res == object() ) {
            return IKRA_Reject;
        }
        IkReturn ikfr(IKRA_Success);
        if( openravepy::ExtractIkReturn(res,ikfr) ) {
            return ikfr;
        }
        extract<IkReturnAction> ikfra(res);
        if( ikfra.check() ) {
            ikfr._action = (IkReturnAction)ikfra;
            return ikfr;
        }
        throw openrave_exception("failed to convert return type of filter to IkReturn",ORE_Assert);
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
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _pIkSolver->GetFreeParameters(values);
        return toPyArray(values);
    }

    object Solve(object oparam, object oq0, int filteroptions, bool ikreturn=false)
    {
        vector<dReal> vsolution;
        boost::shared_ptr< std::vector<dReal> > psolution(&vsolution, utils::null_deleter());
        IkReturnPtr preturn;
        PyIkReturnPtr pyreturn;
        if( ikreturn ) {
            pyreturn.reset(new PyIkReturn(IKRA_Reject));
            preturn = IkReturnPtr(&pyreturn->_ret, utils::null_deleter());
        }
        vector<dReal> q0;
        if( !(oq0 == object()) ) {
            q0 = ExtractArray<dReal>(oq0);
        }
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception("first argument to IkSolver.Solve needs to be IkParameterization",ORE_InvalidArguments);
        }
        if( _pIkSolver->Solve(ikparam, q0, filteroptions, psolution, preturn) ) {
            if( ikreturn ) {
                return boost::python::make_tuple(toPyArray(vsolution),pyreturn);
            }
            else {
                return toPyArray(vsolution);
            }
        }
        if( ikreturn ) {
            return boost::python::make_tuple(object(), pyreturn);
        }
        else {
            return object();
        }
    }

    object SolveAll(object oparam, int filteroptions, bool ikreturn=true)
    {
        //std::vector< std::vector<dReal> > vsolutions;
        //boost::shared_ptr< std::vector<IkReturnPtr> > ikreturns=boost::shared_ptr< std::vector<IkReturnPtr> >();
        return object();
    }

    object Solve(object oparam, object oq0, object oFreeParameters, int filteroptions, bool ikreturn=true)
    {
        vector<dReal> vsolution;
        boost::shared_ptr< std::vector<dReal> > psolution(&vsolution, utils::null_deleter());
        IkReturnPtr preturn;
        PyIkReturnPtr pyreturn;
        if( ikreturn ) {
            pyreturn.reset(new PyIkReturn(IKRA_Reject));
            preturn = IkReturnPtr(&pyreturn->_ret, utils::null_deleter());
        }
        vector<dReal> q0, vFreeParameters;
        if( !(oq0 == object()) ) {
            q0 = ExtractArray<dReal>(oq0);
        }
        if( !(oFreeParameters == object()) ) {
            vFreeParameters = ExtractArray<dReal>(oFreeParameters);
        }
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception("first argument to IkSolver.Solve needs to be IkParameterization",ORE_InvalidArguments);
        }
        if( _pIkSolver->Solve(ikparam, q0, vFreeParameters, filteroptions, psolution, preturn) ) {
            if( ikreturn ) {
                return boost::python::make_tuple(toPyArray(vsolution),pyreturn);
            }
            else {
                return toPyArray(vsolution);
            }
        }
        if( ikreturn ) {
            return boost::python::make_tuple(object(), pyreturn);
        }
        else {
            return object();
        }
    }

    object SolveAll(object oparam, object oFreeParameters, int filteroptions, bool ikreturn=false)
    {
        return object();
    }


    bool Supports(IkParameterizationType type) {
        return _pIkSolver->Supports(type);
    }

    object RegisterCustomFilter(int priority, object fncallback)
    {
        if( !fncallback ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("callback not specified",ORE_InvalidArguments);
        }
        return toPyUserData(_pIkSolver->RegisterCustomFilter(priority,boost::bind(&PyIkSolverBase::_CallCustomFilter,fncallback,_pyenv,_pIkSolver,_1,_2,_3)));
    }
};

namespace openravepy {

bool ExtractIkReturn(object o, IkReturn& ikfr)
{
    extract<PyIkReturnPtr > pyikfr(o);
    if( pyikfr.check() ) {
        ikfr = ((PyIkReturnPtr)pyikfr)->_ret;
        return true;
    }
    return false;
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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Solve_overloads, Solve, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SolveFree_overloads, Solve, 4, 5)

void init_openravepy_iksolver()
{
    enum_<IkFilterOptions>("IkFilterOptions" DOXY_ENUM(IkFilterOptions))
    .value("CheckEnvCollisions",IKFO_CheckEnvCollisions)
    .value("IgnoreSelfCollisions",IKFO_IgnoreSelfCollisions)
    .value("IgnoreJointLimits",IKFO_IgnoreJointLimits)
    .value("IgnoreCustomFilters",IKFO_IgnoreCustomFilters)
    .value("IgnoreEndEffectorCollisions",IKFO_IgnoreEndEffectorCollisions)
    ;

    enum_<IkReturnAction>("IkReturnAction" DOXY_ENUM(IkReturnAction))
    .value("Success",IKRA_Success)
    .value("Reject",IKRA_Reject)
    .value("Quit",IKRA_Quit)
    .value("QuitEndEffectorCollision",IKRA_QuitEndEffectorCollision)
    .value("RejectKinematics",IKRA_RejectKinematics)
    .value("RejectEnvCollision",IKRA_RejectEnvCollision)
    .value("RejectJointLimits",IKRA_RejectJointLimits)
    .value("RejectKinematicsPrecision",IKRA_RejectKinematicsPrecision)
    .value("RejectCustomFilter",IKRA_RejectCustomFilter)
    ;

    {
        scope ikreturn = class_<PyIkReturn, PyIkReturnPtr>("IkReturn", DOXY_CLASS(IkReturn), no_init)
                         .def(init<IkReturnAction>(args("action")))
                         .def("GetAction",&PyIkReturn::GetAction, "Retuns IkReturn::_action")
                         .def("GetUserData",&PyIkReturn::GetUserData, "Retuns IkReturn::_userdata")
                         .def("GetMapData",&PyIkReturn::GetMapData, "Returns a dictionary copy for IkReturn::_mapdata")
        ;
    }

    {
        object (PyIkSolverBase::*Solve)(object, object, int, bool) = &PyIkSolverBase::Solve;
        object (PyIkSolverBase::*SolveFree)(object, object, object, int, bool) = &PyIkSolverBase::Solve;
        class_<PyIkSolverBase, boost::shared_ptr<PyIkSolverBase>, bases<PyInterfaceBase> >("IkSolver", DOXY_CLASS(IkSolverBase), no_init)
        .def("Solve",Solve,Solve_overloads(args("ikparam","q0","filteroptions","ikreturn"), DOXY_FN(IkSolverBase, Solve "const IkParameterization&; const std::vector; int; boost::shared_ptr; IkReturnPtr")))
        .def("Solve",SolveFree,SolveFree_overloads(args("ikparam","q0","freeparameters", "filteroptions","ikreturn"), DOXY_FN(IkSolverBase, Solve "const IkParameterization&; const std::vector; const std::vector; int; boost::shared_ptr; IkReturnPtr")))
        .def("GetNumFreeParameters",&PyIkSolverBase::GetNumFreeParameters, DOXY_FN(IkSolverBase,GetNumFreeParameters))
        .def("GetFreeParameters",&PyIkSolverBase::GetFreeParameters, DOXY_FN(IkSolverBase,GetFreeParameters))
        .def("Supports",&PyIkSolverBase::Supports, args("iktype"), DOXY_FN(IkSolverBase,Supports))
        .def("RegisterCustomFilter",&PyIkSolverBase::RegisterCustomFilter, args("priority","callback"), DOXY_FN(IkSolverBase,RegisterCustomFilter))
        ;
    }

    def("RaveCreateIkSolver",openravepy::RaveCreateIkSolver,args("env","name"),DOXY_FN1(RaveCreateIkSolver));
}

}
