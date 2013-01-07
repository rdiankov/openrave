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

class PyPlannerProgress
{
public:
    PyPlannerProgress() : _iteration(0) {
    }
    PyPlannerProgress(const PlannerBase::PlannerProgress& progress) {
        _iteration = progress._iteration;
    }
    string __str__() {
        return boost::str(boost::format("<PlannerProgress: iter=%d>")%_iteration);
    }

    int _iteration;
};

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
        PyPlannerParameters() {
            _paramswrite.reset(new PlannerBase::PlannerParameters());
            _paramsread = _paramswrite;
        }
        PyPlannerParameters(boost::shared_ptr<PyPlannerParameters> pyparameters) {
            _paramswrite.reset(new PlannerBase::PlannerParameters());
            if( !!pyparameters ) {
                _paramswrite->copy(pyparameters->GetParameters());
            }
            _paramsread = _paramswrite;
        }
        PyPlannerParameters(PlannerBase::PlannerParametersPtr params) : _paramswrite(params), _paramsread(params) {
        }
        PyPlannerParameters(PlannerBase::PlannerParametersConstPtr params) : _paramsread(params) {
        }
        virtual ~PyPlannerParameters() {
        }

        PlannerBase::PlannerParametersConstPtr GetParameters() const {
            return _paramsread;
        }

        void SetRobotActiveJoints(PyRobotBasePtr robot)
        {
            if( !_paramswrite ) {
                throw OPENRAVE_EXCEPTION_FORMAT0("PlannerParameters needs to be non-const",ORE_Failed);
            }
            _paramswrite->SetRobotActiveJoints(openravepy::GetRobot(robot));
        }

        void SetConfigurationSpecification(PyEnvironmentBasePtr pyenv, PyConfigurationSpecificationPtr pyspec) {
            _paramswrite->SetConfigurationSpecification(openravepy::GetEnvironment(pyenv), openravepy::GetConfigurationSpecification(pyspec));
        }

        void SetExtraParameters(const std::string& s) {
            _paramswrite->_sExtraParameters = s;
        }

        void SetGoalConfig(object o)
        {
            _paramswrite->vgoalconfig = ExtractArray<dReal>(o);
        }

        void SetInitialConfig(object o)
        {
            _paramswrite->vinitialconfig = ExtractArray<dReal>(o);
        }

        string __repr__() {
            stringstream ss;
            ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);         /// have to do this or otherwise precision gets lost
            ss << "Planner.PlannerParameters(\"\"\"";
            ss << *_paramsread << "\"\"\")" << endl;
            return ss.str();
        }
        string __str__() {
            return boost::str(boost::format("<PlannerParameters, dof=%d>")%_paramsread->GetDOF());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(boost::shared_ptr<PyPlannerParameters> p) {
            return !!p && _paramsread == p->_paramsread;
        }
        bool __ne__(boost::shared_ptr<PyPlannerParameters> p) {
            return !p || _paramsread != p->_paramsread;
        }
    };

    typedef boost::shared_ptr<PyPlannerParameters> PyPlannerParametersPtr;
    typedef boost::shared_ptr<PyPlannerParameters const> PyPlannerParametersConstPtr;

    PyPlannerBase(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pplanner, pyenv), _pplanner(pplanner) {
    }
    virtual ~PyPlannerBase() {
    }

    bool InitPlan(PyRobotBasePtr pyrobot, PyPlannerParametersPtr pparams, bool releasegil=false)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        PlannerBase::PlannerParametersConstPtr parameters = pparams->GetParameters();
        RobotBasePtr probot = openravepy::GetRobot(pyrobot);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        return _pplanner->InitPlan(probot,parameters);
    }

    bool InitPlan(PyRobotBasePtr pbase, const string& params)
    {
        stringstream ss(params);
        return _pplanner->InitPlan(openravepy::GetRobot(pbase),ss);
    }

    PlannerStatus PlanPath(PyTrajectoryBasePtr pytraj,bool releasegil=true)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        return _pplanner->PlanPath(ptraj);
    }

    PyPlannerParametersPtr GetParameters() const
    {
        PlannerBase::PlannerParametersConstPtr params = _pplanner->GetParameters();
        if( !params ) {
            return PyPlannerParametersPtr();
        }
        return PyPlannerParametersPtr(new PyPlannerParameters(params));
    }

    static PlannerAction _PlanCallback(object fncallback, PyEnvironmentBasePtr pyenv, const PlannerBase::PlannerProgress& progress)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            boost::shared_ptr<PyPlannerProgress> pyprogress(new PyPlannerProgress(progress));
            res = fncallback(object(pyprogress));
        }
        catch(...) {
            RAVELOG_ERROR("exception occured in _PlanCallback:\n");
            PyErr_Print();
        }
        PyGILState_Release(gstate);
        if(( res == object()) || !res ) {
            return PA_None;
        }
        extract<PlannerAction> xb(res);
        if( xb.check() ) {
            return (PlannerAction)xb;
        }
        RAVELOG_WARN("plan callback nothing returning, so executing default action\n");
        return PA_None;
    }

    object RegisterPlanCallback(object fncallback)
    {
        if( !fncallback ) {
            throw openrave_exception("callback not specified");
        }
        UserDataPtr p = _pplanner->RegisterPlanCallback(boost::bind(&PyPlannerBase::_PlanCallback,fncallback,_pyenv,_1));
        if( !p ) {
            throw openrave_exception("no registration callback returned");
        }
        return toPyUserData(p);
    }

    PlannerBasePtr GetPlanner()
    {
        return _pplanner;
    }
};

PlannerBasePtr GetPlanner(PyPlannerBasePtr pyplanner)
{
    return !pyplanner ? PlannerBasePtr() : pyplanner->GetPlanner();
}

PyInterfaceBasePtr toPyPlanner(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv)
{
    return !pplanner ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyPlannerBase(pplanner,pyenv));
}

PlannerBase::PlannerParametersConstPtr GetPlannerParametersConst(object o)
{
    extract<PyPlannerBase::PyPlannerParametersPtr> pyparams(o);
    if( pyparams.check() ) {
        return ((PyPlannerBase::PyPlannerParametersPtr)pyparams)->GetParameters();
    }
    return PlannerBase::PlannerParametersPtr();
}

PyPlannerBasePtr RaveCreatePlanner(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    PlannerBasePtr p = OpenRAVE::RaveCreatePlanner(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyPlannerBasePtr();
    }
    return PyPlannerBasePtr(new PyPlannerBase(p,pyenv));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitPlan_overloads, InitPlan, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PlanPath_overloads, PlanPath, 1, 2)

void init_openravepy_planner()
{
    object plannerstatus = enum_<PlannerStatus>("PlannerStatus" DOXY_ENUM(PlannerStatus))
                           .value("Failed",PS_Failed)
                           .value("HasSolution",PS_HasSolution)
                           .value("Interrupted",PS_Interrupted)
                           .value("InterruptedWithSolution",PS_InterruptedWithSolution)
    ;

    object planneraction = enum_<PlannerAction>("PlannerAction" DOXY_ENUM(PlannerAction))
                           .value("None",PA_None)
                           .value("Interrupt",PA_Interrupt)
                           .value("ReturnWithAnySolution",PA_ReturnWithAnySolution)
    ;
    class_<PyPlannerProgress, boost::shared_ptr<PyPlannerProgress> >("PlannerProgress", DOXY_CLASS(PlannerBase::PlannerProgress))
    .def_readwrite("_iteration",&PyPlannerProgress::_iteration)
    ;

    {
        bool (PyPlannerBase::*InitPlan1)(PyRobotBasePtr, PyPlannerBase::PyPlannerParametersPtr,bool) = &PyPlannerBase::InitPlan;
        bool (PyPlannerBase::*InitPlan2)(PyRobotBasePtr, const string &) = &PyPlannerBase::InitPlan;
        scope planner = class_<PyPlannerBase, boost::shared_ptr<PyPlannerBase>, bases<PyInterfaceBase> >("Planner", DOXY_CLASS(PlannerBase), no_init)
                        .def("InitPlan",InitPlan1,InitPlan_overloads(args("robot","params","releasegil"), DOXY_FN(PlannerBase,InitPlan "RobotBasePtr; PlannerParametersConstPtr")))
                        .def("InitPlan",InitPlan2,args("robot","xmlparams"), DOXY_FN(PlannerBase,InitPlan "RobotBasePtr; std::istream"))
                        .def("PlanPath",&PyPlannerBase::PlanPath,PlanPath_overloads(args("traj","releasegil"), DOXY_FN(PlannerBase,PlanPath)))
                        .def("GetParameters",&PyPlannerBase::GetParameters, DOXY_FN(PlannerBase,GetParameters))
        ;

        class_<PyPlannerBase::PyPlannerParameters, PyPlannerBase::PyPlannerParametersPtr >("PlannerParameters", DOXY_CLASS(PlannerBase::PlannerParameters))
        .def(init<>())
        .def(init<PyPlannerBase::PyPlannerParametersPtr>(args("parameters")))
        .def("SetRobotActiveJoints",&PyPlannerBase::PyPlannerParameters::SetRobotActiveJoints, args("robot"), DOXY_FN(PlannerBase::PlannerParameters, SetRobotActiveJoints))
        .def("SetConfigurationSpecification",&PyPlannerBase::PyPlannerParameters::SetConfigurationSpecification, args("env","spec"), DOXY_FN(PlannerBase::PlannerParameters, SetConfigurationSpecification))
        .def("SetExtraParameters",&PyPlannerBase::PyPlannerParameters::SetExtraParameters, args("extra"), DOXY_FN(PlannerBase::PlannerParameters, SetExtraParameters))
        .def("SetGoalConfig",&PyPlannerBase::PyPlannerParameters::SetGoalConfig,args("values"),"sets PlannerParameters::vgoalconfig")
        .def("SetInitialConfig",&PyPlannerBase::PyPlannerParameters::SetInitialConfig,args("values"),"sets PlannerParameters::vinitialconfig")
        .def("__str__",&PyPlannerBase::PyPlannerParameters::__str__)
        .def("__unicode__",&PyPlannerBase::PyPlannerParameters::__unicode__)
        .def("__repr__",&PyPlannerBase::PyPlannerParameters::__repr__)
        .def("__eq__",&PyPlannerBase::PyPlannerParameters::__eq__)
        .def("__ne__",&PyPlannerBase::PyPlannerParameters::__ne__)
        ;
    }

    def("RaveCreatePlanner",openravepy::RaveCreatePlanner,args("env","name"),DOXY_FN1(RaveCreatePlanner));
}

}
