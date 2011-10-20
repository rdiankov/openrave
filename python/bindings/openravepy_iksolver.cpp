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

class PyIkSolverBase : public PyInterfaceBase
{
protected:
    IkSolverBasePtr _pIkSolver;

    static IkFilterReturn _CallCustomFilter(object fncallback, PyEnvironmentBasePtr pyenv, std::vector<dReal>& values, RobotBase::ManipulatorPtr pmanip, const IkParameterization& ikparam)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            res = fncallback(toPyArray(values), openravepy::toPyRobotManipulator(pmanip,pyenv),toPyIkParameterization(ikparam));
        }
        catch(...) {
            RAVELOG_ERROR("exception occured in python viewer callback:\n");
            PyErr_Print();
        }
        PyGILState_Release(gstate);
        if( res == object() ) {
            return IKFR_Reject;
        }
        extract<IkFilterReturn> ikfr(res);
        return (IkFilterReturn)ikfr;
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

    bool Supports(IkParameterizationType type) {
        return _pIkSolver->Supports(type);
    }

    object RegisterCustomFilter(int priority, object fncallback)
    {
        if( !fncallback ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("callback not specified",ORE_InvalidArguments);
        }
        return toPyUserData(_pIkSolver->RegisterCustomFilter(priority,boost::bind(&PyIkSolverBase::_CallCustomFilter,fncallback,_pyenv,_1,_2,_3)));
    }
};

namespace openravepy {

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
    class_<PyIkSolverBase, boost::shared_ptr<PyIkSolverBase>, bases<PyInterfaceBase> >("IkSolver", DOXY_CLASS(IkSolverBase), no_init)
    .def("GetNumFreeParameters",&PyIkSolverBase::GetNumFreeParameters, DOXY_FN(IkSolverBase,GetNumFreeParameters))
    .def("GetFreeParameters",&PyIkSolverBase::GetFreeParameters, DOXY_FN(IkSolverBase,GetFreeParameters))
    .def("Supports",&PyIkSolverBase::Supports, args("iktype"), DOXY_FN(IkSolverBase,Supports))
    .def("RegisterCustomFilter",&PyIkSolverBase::RegisterCustomFilter, args("priority","callback"), DOXY_FN(IkSolverBase,RegisterCustomFilter))
    ;

    def("RaveCreateIkSolver",openravepy::RaveCreateIkSolver,args("env","name"),DOXY_FN1(RaveCreateIkSolver));
}

}
