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

class PyPlannerBase : public PyInterfaceBase
{
protected:
    PlannerBasePtr _pplanner;
public:
    class PyPlannerParameters
    {
        PlannerBase::PlannerParametersPtr _paramswrite;
        PlannerBase::PlannerParametersConstPtr _paramsread;
public:
        PyPlannerParameters(PlannerBase::PlannerParametersPtr params) : _paramswrite(params), _paramsread(params) {
        }
        PyPlannerParameters(PlannerBase::PlannerParametersConstPtr params) : _paramsread(params) {
        }
        virtual ~PyPlannerParameters() {
        }

        PlannerBase::PlannerParametersConstPtr GetParameters() const {
            return _paramsread;
        }

        string __repr__() {
            return boost::str(boost::format("<PlannerParameters(dof=%d)>")%_paramsread->GetDOF());
        }
        string __str__() {
            stringstream ss;
            ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);         /// have to do this or otherwise precision gets lost
            ss << *_paramsread << endl;
            return ss.str();
        }
        bool __eq__(boost::shared_ptr<PyPlannerParameters> p) {
            return !!p && _paramsread == p->_paramsread;
        }
        bool __ne__(boost::shared_ptr<PyPlannerParameters> p) {
            return !p || _paramsread != p->_paramsread;
        }
    };

    PyPlannerBase(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pplanner, pyenv), _pplanner(pplanner) {
    }
    virtual ~PyPlannerBase() {
    }

    bool InitPlan(PyRobotBasePtr pbase, boost::shared_ptr<PyPlannerParameters> pparams)
    {
        return _pplanner->InitPlan(openravepy::GetRobot(pbase),pparams->GetParameters());
    }

    bool InitPlan(PyRobotBasePtr pbase, const string& params)
    {
        stringstream ss(params);
        return _pplanner->InitPlan(openravepy::GetRobot(pbase),ss);
    }

    bool PlanPath(PyTrajectoryBasePtr pytraj)
    {
        return _pplanner->PlanPath(openravepy::GetTrajectory(pytraj));
    }

    boost::shared_ptr<PyPlannerParameters> GetParameters() const
    {
        PlannerBase::PlannerParametersConstPtr params = _pplanner->GetParameters();
        if( !params ) {
            return boost::shared_ptr<PyPlannerParameters>();
        }
        return boost::shared_ptr<PyPlannerParameters>(new PyPlannerParameters(params));
    }

    PlannerBasePtr GetPlanner()
    {
        return _pplanner;
    }
};

namespace openravepy {

PlannerBasePtr GetPlanner(PyPlannerBasePtr pyplanner)
{
    return !pyplanner ? PlannerBasePtr() : pyplanner->GetPlanner();
}

PyInterfaceBasePtr toPyPlanner(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv)
{
    return !pplanner ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyPlannerBase(pplanner,pyenv));
}

PyPlannerBasePtr RaveCreatePlanner(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    PlannerBasePtr p = OpenRAVE::RaveCreatePlanner(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyPlannerBasePtr();
    }
    return PyPlannerBasePtr(new PyPlannerBase(p,pyenv));
}

void init_openravepy_planner()
{
    {
        bool (PyPlannerBase::*InitPlan1)(PyRobotBasePtr, boost::shared_ptr<PyPlannerBase::PyPlannerParameters>) = &PyPlannerBase::InitPlan;
        bool (PyPlannerBase::*InitPlan2)(PyRobotBasePtr, const string &) = &PyPlannerBase::InitPlan;
        scope planner = class_<PyPlannerBase, boost::shared_ptr<PyPlannerBase>, bases<PyInterfaceBase> >("Planner", DOXY_CLASS(PlannerBase), no_init)
                        .def("InitPlan",InitPlan1,args("robot","params"), DOXY_FN(PlannerBase,InitPlan "RobotBasePtr; PlannerParametersConstPtr"))
                        .def("InitPlan",InitPlan2,args("robot","xmlparams"), DOXY_FN(PlannerBase,InitPlan "RobotBasePtr; std::istream"))
                        .def("PlanPath",&PyPlannerBase::PlanPath,args("traj"), DOXY_FN(PlannerBase,PlanPath))
                        .def("GetParameters",&PyPlannerBase::GetParameters, DOXY_FN(PlannerBase,GetParameters))
        ;

        class_<PyPlannerBase::PyPlannerParameters, boost::shared_ptr<PyPlannerBase::PyPlannerParameters> >("PlannerParameters", DOXY_CLASS(PlannerBase::PlannerParameters),no_init)
        .def("__str__",&PyPlannerBase::PyPlannerParameters::__str__)
        .def("__repr__",&PyPlannerBase::PyPlannerParameters::__repr__)
        .def("__eq__",&PyPlannerBase::PyPlannerParameters::__eq__)
        .def("__ne__",&PyPlannerBase::PyPlannerParameters::__ne__)
        ;
    }

    def("RaveCreatePlanner",openravepy::RaveCreatePlanner,args("env","name"),DOXY_FN1(RaveCreatePlanner));
}

}
