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
#include <openravepy/openravepy_robotbase.h>
#include <openravepy/openravepy_configurationspecification.h>
#include <openravepy/openravepy_environmentbase.h>
#include <openravepy/openravepy_collisionreport.h>
#include <openravepy/openravepy_trajectorybase.h>
#include <openravepy/openravepy_plannerbase.h>

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

PyPlannerProgress::PyPlannerProgress() {
}
PyPlannerProgress::PyPlannerProgress(const PlannerBase::PlannerProgress& progress) {
    _iteration = progress._iteration;
}
std::string PyPlannerProgress::__str__() {
    return boost::str(boost::format("<PlannerProgress: iter=%d>")%_iteration);
}

PyPlannerStatus::PyPlannerStatus() {
}

PyPlannerStatus::PyPlannerStatus(const PlannerStatus& status) {
    // Just serializeToJason?
    description = ConvertStringToUnicode(status.description);
    errorOrigin = ConvertStringToUnicode(status.errorOrigin);
    statusCode = status.statusCode;
    jointValues = toPyArray(status.jointValues);

    if( !status.report ) {
        //_report = "";
        report = py::none_();
    }
    else {
        //_report = status._report->__str__();
        report = py::to_object(openravepy::toPyCollisionReport(status.report, NULL));
    }

    ikparam = toPyIkParameterization(status.ikparam);
}

object toPyPlannerStatus(const PlannerStatus& status)
{
    return py::to_object(OPENRAVE_SHARED_PTR<PyPlannerStatus>(new PyPlannerStatus(status)));
}

PyPlannerBase::PyPlannerParameters::PyPlannerParameters() {
    _paramswrite.reset(new PlannerBase::PlannerParameters());
    _paramsread = _paramswrite;
}
PyPlannerBase::PyPlannerParameters::PyPlannerParameters(OPENRAVE_SHARED_PTR<PyPlannerParameters> pyparameters) {
    _paramswrite.reset(new PlannerBase::PlannerParameters());
    if( !!pyparameters ) {
        _paramswrite->copy(pyparameters->GetParameters());
    }
    _paramsread = _paramswrite;
}
PyPlannerBase::PyPlannerParameters::PyPlannerParameters(PlannerBase::PlannerParametersPtr params) : _paramswrite(params), _paramsread(params) {
}
PyPlannerBase::PyPlannerParameters::PyPlannerParameters(PlannerBase::PlannerParametersConstPtr params) : _paramsread(params) {
}
PyPlannerBase::PyPlannerParameters::~PyPlannerParameters() {
}

PlannerBase::PlannerParametersConstPtr PyPlannerBase::PyPlannerParameters::GetParameters() const {
    return _paramsread;
}

PlannerBase::PlannerParametersPtr PyPlannerBase::PyPlannerParameters::GetParameters() {
    return _paramswrite;
}

void PyPlannerBase::PyPlannerParameters::SetRobotActiveJoints(PyRobotBasePtr robot)
{
    if( !_paramswrite ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("PlannerParameters needs to be non-const"),ORE_Failed);
    }
    _paramswrite->SetRobotActiveJoints(openravepy::GetRobot(robot));
}

void PyPlannerBase::PyPlannerParameters::SetConfigurationSpecification(PyEnvironmentBasePtr pyenv, PyConfigurationSpecificationPtr pyspec) {
    _paramswrite->SetConfigurationSpecification(openravepy::GetEnvironment(pyenv), openravepy::GetConfigurationSpecification(pyspec));
}

object PyPlannerBase::PyPlannerParameters::GetConfigurationSpecification() const {
    return py::to_object(openravepy::toPyConfigurationSpecification(_paramswrite->_configurationspecification));
}

void PyPlannerBase::PyPlannerParameters::SetExtraParameters(const std::string& s) {
    _paramswrite->_sExtraParameters = s;
}

void PyPlannerBase::PyPlannerParameters::SetRandomGeneratorSeed(uint32_t seed) {
    _paramswrite->_nRandomGeneratorSeed = seed;
}

void PyPlannerBase::PyPlannerParameters::SetGoalConfig(object o)
{
    _paramswrite->vgoalconfig = ExtractArray<dReal>(o);
}

void PyPlannerBase::PyPlannerParameters::SetInitialConfig(object o)
{
    _paramswrite->vinitialconfig = ExtractArray<dReal>(o);
}

void PyPlannerBase::PyPlannerParameters::SetInitialConfigVelocities(object o)
{
    _paramswrite->_vInitialConfigVelocities = ExtractArray<dReal>(o);
}

void PyPlannerBase::PyPlannerParameters::SetGoalConfigVelocities(object o)
{
    _paramswrite->_vGoalConfigVelocities = ExtractArray<dReal>(o);
}

void PyPlannerBase::PyPlannerParameters::SetConfigVelocityLimit(object o)
{
    _paramswrite->_vConfigVelocityLimit = ExtractArray<dReal>(o);
}

void PyPlannerBase::PyPlannerParameters::SetConfigAccelerationLimit(object o)
{
    _paramswrite->_vConfigAccelerationLimit = ExtractArray<dReal>(o);
}

void PyPlannerBase::PyPlannerParameters::SetConfigResolution(object o)
{
    _paramswrite->_vConfigResolution = ExtractArray<dReal>(o);
}

void PyPlannerBase::PyPlannerParameters::SetMaxIterations(int nMaxIterations)
{
    _paramswrite->_nMaxIterations = nMaxIterations;
}

object PyPlannerBase::PyPlannerParameters::CheckPathAllConstraints(object oq0, object oq1, object odq0, object odq1, dReal timeelapsed, IntervalType interval, uint32_t options, bool filterreturn)
{
    const std::vector<dReal> q0, q1, dq0, dq1;
    ConstraintFilterReturnPtr pfilterreturn;
    if( filterreturn ) {
        pfilterreturn.reset(new ConstraintFilterReturn());
    }
    int ret = _paramswrite->CheckPathAllConstraints(ExtractArray<dReal>(oq0), ExtractArray<dReal>(oq1), ExtractArray<dReal>(odq0), ExtractArray<dReal>(odq1), timeelapsed, interval, options, pfilterreturn);
    if( filterreturn ) {
        py::dict ofilterreturn;
        ofilterreturn["configurations"] = toPyArray(pfilterreturn->_configurations);
        ofilterreturn["configurationtimes"] = toPyArray(pfilterreturn->_configurationtimes);
        ofilterreturn["invalidvalues"] = toPyArray(pfilterreturn->_invalidvalues);
        ofilterreturn["invalidvelocities"] = toPyArray(pfilterreturn->_invalidvelocities);
        ofilterreturn["fTimeWhenInvalid"] = pfilterreturn->_fTimeWhenInvalid;
        ofilterreturn["returncode"] = pfilterreturn->_returncode;
        ofilterreturn["reportstr"] = pfilterreturn->_report.__str__();
        return ofilterreturn;
    }
    return py::to_object(ret);
}

void PyPlannerBase::PyPlannerParameters::SetPostProcessing(const std::string& plannername, const std::string& plannerparameters)
{
    _paramswrite->_sPostProcessingPlanner = plannername;
    _paramswrite->_sPostProcessingParameters = plannerparameters;
}

std::string PyPlannerBase::PyPlannerParameters::__repr__() {
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);         /// have to do this or otherwise precision gets lost
    ss << "Planner.PlannerParameters(\"\"\"";
    ss << *_paramsread << "\"\"\")" << endl;
    return ss.str();
}
std::string PyPlannerBase::PyPlannerParameters::__str__() {
    return boost::str(boost::format("<PlannerParameters, dof=%d>")%_paramsread->GetDOF());
}
object PyPlannerBase::PyPlannerParameters::__unicode__() {
    return ConvertStringToUnicode(__str__());
}
bool PyPlannerBase::PyPlannerParameters::__eq__(OPENRAVE_SHARED_PTR<PyPlannerParameters> p) {
    return !!p && _paramsread == p->_paramsread;
}
bool PyPlannerBase::PyPlannerParameters::__ne__(OPENRAVE_SHARED_PTR<PyPlannerParameters> p) {
    return !p || _paramsread != p->_paramsread;
}

typedef OPENRAVE_SHARED_PTR<PyPlannerBase::PyPlannerParameters> PyPlannerParametersPtr;
typedef OPENRAVE_SHARED_PTR<PyPlannerBase::PyPlannerParameters const> PyPlannerParametersConstPtr;

PyPlannerBase::PyPlannerBase(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pplanner, pyenv), _pplanner(pplanner) {
}
PyPlannerBase::~PyPlannerBase() {
}

bool PyPlannerBase::InitPlan(PyRobotBasePtr pyrobot, PyPlannerParametersPtr pparams, bool releasegil)
{
    openravepy::PythonThreadSaverPtr statesaver;
    PlannerBase::PlannerParametersConstPtr parameters = pparams->GetParameters();
    RobotBasePtr probot = openravepy::GetRobot(pyrobot);
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pplanner->InitPlan(probot,parameters);
}

bool PyPlannerBase::InitPlan(PyRobotBasePtr pbase, const string& params)
{
    std::stringstream ss(params);
    return _pplanner->InitPlan(openravepy::GetRobot(pbase),ss);
}

object PyPlannerBase::PlanPath(PyTrajectoryBasePtr pytraj, bool releasegil)
{
    openravepy::PythonThreadSaverPtr statesaver;
    TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    PlannerStatus status = _pplanner->PlanPath(ptraj);
    statesaver.reset(); // re-lock GIL
    return openravepy::toPyPlannerStatus(status);
}

PyPlannerParametersPtr PyPlannerBase::GetParameters() const
{
    PlannerBase::PlannerParametersConstPtr params = _pplanner->GetParameters();
    if( !params ) {
        return PyPlannerParametersPtr();
    }
    return PyPlannerParametersPtr(new PyPlannerParameters(params));
}

PlannerAction PyPlannerBase::_PlanCallback(object fncallback, PyEnvironmentBasePtr pyenv, const PlannerBase::PlannerProgress& progress)
{
    object res;
    PyGILState_STATE gstate = PyGILState_Ensure();
    try {
        OPENRAVE_SHARED_PTR<PyPlannerProgress> pyprogress(new PyPlannerProgress(progress));
        res = fncallback(py::to_object(pyprogress));
    }
    catch(...) {
        RAVELOG_ERROR("exception occured in _PlanCallback:\n");
        PyErr_Print();
    }
    PlannerAction ret = PA_None;
    if( IS_PYTHONOBJECT_NONE(res) || !res ) {
        ret = PA_None;
        RAVELOG_WARN("plan callback nothing returning, so executing default action\n");
    }
    else {
        extract_<PlannerAction> xb(res);
        if( xb.check() ) {
            ret = (PlannerAction)xb;
        }
        else {
            RAVELOG_WARN("plan callback nothing returning, so executing default action\n");
        }
    }
    PyGILState_Release(gstate);
    return ret;
}

object PyPlannerBase::RegisterPlanCallback(object fncallback)
{
    if( !fncallback ) {
        throw openrave_exception(_("callback not specified"));
    }
    UserDataPtr p = _pplanner->RegisterPlanCallback(boost::bind(&PyPlannerBase::_PlanCallback,fncallback,_pyenv,_1));
    if( !p ) {
        throw openrave_exception(_("no registration callback returned"));
    }
    return toPyUserData(p);
}

PlannerBasePtr PyPlannerBase::GetPlanner()
{
    return _pplanner;
}


PlannerBasePtr GetPlanner(PyPlannerBasePtr pyplanner)
{
    return !pyplanner ? PlannerBasePtr() : pyplanner->GetPlanner();
}

PyInterfaceBasePtr toPyPlanner(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv)
{
    return !pplanner ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyPlannerBase(pplanner,pyenv));
}

PlannerBase::PlannerParametersPtr GetPlannerParameters(object o)
{
    extract_<PyPlannerBase::PyPlannerParametersPtr> pyparams(o);
    if( pyparams.check() ) {
        return ((PyPlannerBase::PyPlannerParametersPtr)pyparams)->GetParameters();
    }
    return PlannerBase::PlannerParametersPtr();
}

PlannerBase::PlannerParametersConstPtr GetPlannerParametersConst(object o)
{
    extract_<PyPlannerBase::PyPlannerParametersPtr> pyparams(o);
    if( pyparams.check() ) {
        return ((PyPlannerBase::PyPlannerParametersPtr)pyparams)->GetParameters();
    }
    return PlannerBase::PlannerParametersPtr();
}

object toPyPlannerParameters(PlannerBase::PlannerParametersPtr params)
{
    if( !params ) {
        return py::none_();
    }
    return py::to_object(PyPlannerBase::PyPlannerParametersPtr(new PyPlannerBase::PyPlannerParameters(params)));
}

PyPlannerBasePtr RaveCreatePlanner(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    PlannerBasePtr p = OpenRAVE::RaveCreatePlanner(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyPlannerBasePtr();
    }
    return PyPlannerBasePtr(new PyPlannerBase(p,pyenv));
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitPlan_overloads, InitPlan, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PlanPath_overloads, PlanPath, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckPathAllConstraints_overloads, CheckPathAllConstraints, 6, 8)
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_planner(py::module& m)
#else
void init_openravepy_planner()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals;
    object plannerstatuscode = enum_<PlannerStatusCode>(m, "PlannerStatusCode", py::arithmetic() DOXY_ENUM(PlannerStatusCode))
#else
    object plannerstatuscode = enum_<PlannerStatusCode>("PlannerStatusCode" DOXY_ENUM(PlannerStatusCode))
#endif
                               .value("Failed",PS_Failed)
                               .value("HasSolution",PS_HasSolution)
                               .value("Interrupted",PS_Interrupted)
                               .value("InterruptedWithSolution",PS_InterruptedWithSolution)
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    object planneraction = enum_<PlannerAction>(m, "PlannerAction", py::arithmetic() DOXY_ENUM(PlannerAction))
#else
    object planneraction = enum_<PlannerAction>("PlannerAction" DOXY_ENUM(PlannerAction))
#endif
                           .value("None",PA_None)
                           .value("Interrupt",PA_Interrupt)
                           .value("ReturnWithAnySolution",PA_ReturnWithAnySolution)
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPlannerStatus, OPENRAVE_SHARED_PTR<PyPlannerStatus> >(m, "PlannerStatus", DOXY_CLASS(PlannerStatus))
#else
    class_<PyPlannerStatus, OPENRAVE_SHARED_PTR<PyPlannerStatus> >("PlannerStatus", DOXY_CLASS(PlannerStatus))
#endif
    .def_readwrite("report",&PyPlannerStatus::report)
    .def_readwrite("description",&PyPlannerStatus::description)
    .def_readwrite("errorOrigin",&PyPlannerStatus::errorOrigin)
    .def_readwrite("jointValues",&PyPlannerStatus::jointValues)
    .def_readwrite("ikparam",&PyPlannerStatus::ikparam)
    .def_readwrite("statusCode",&PyPlannerStatus::statusCode)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPlannerProgress, OPENRAVE_SHARED_PTR<PyPlannerProgress> >(m, "PlannerProgress", DOXY_CLASS(PlannerBase::PlannerProgress))
#else
    class_<PyPlannerProgress, OPENRAVE_SHARED_PTR<PyPlannerProgress> >("PlannerProgress", DOXY_CLASS(PlannerBase::PlannerProgress))
#endif
    .def_readwrite("_iteration",&PyPlannerProgress::_iteration)
    ;

    {
        bool (PyPlannerBase::*InitPlan1)(PyRobotBasePtr, PyPlannerBase::PyPlannerParametersPtr,bool) = &PyPlannerBase::InitPlan;
        bool (PyPlannerBase::*InitPlan2)(PyRobotBasePtr, const string &) = &PyPlannerBase::InitPlan;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ planner = class_<PyPlannerBase, OPENRAVE_SHARED_PTR<PyPlannerBase>, PyInterfaceBase>(m, "Planner", DOXY_CLASS(PlannerBase))
#else
        scope_ planner = class_<PyPlannerBase, OPENRAVE_SHARED_PTR<PyPlannerBase>, bases<PyInterfaceBase> >("Planner", DOXY_CLASS(PlannerBase), no_init)
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                         .def("InitPlan", InitPlan1,
                              "robot"_a,
                              "params"_a,
                              "releasegil"_a = false,
                              DOXY_FN(PlannerBase, InitPlan "RobotBasePtr; PlannerParametersConstPtr")
                              )
#else
                         .def("InitPlan",InitPlan1,InitPlan_overloads(PY_ARGS("robot","params","releasegil") DOXY_FN(PlannerBase,InitPlan "RobotBasePtr; PlannerParametersConstPtr")))
#endif
                         .def("InitPlan",InitPlan2, PY_ARGS("robot","xmlparams") DOXY_FN(PlannerBase,InitPlan "RobotBasePtr; std::istream"))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                         .def("PlanPath", &PyPlannerBase::PlanPath,
                              "traj"_a,
                              "releasegil"_a = true,
                              DOXY_FN(PlannerBase, PlanPath)
                              )
#else
                         .def("PlanPath",&PyPlannerBase::PlanPath,PlanPath_overloads(PY_ARGS("traj","releasegil") DOXY_FN(PlannerBase,PlanPath)))
#endif
                         .def("GetParameters",&PyPlannerBase::GetParameters, DOXY_FN(PlannerBase,GetParameters))
                         .def("RegisterPlanCallback",&PyPlannerBase::RegisterPlanCallback, DOXY_FN(PlannerBase,RegisterPlanCallback))
        ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // PlannerParameters belongs to Planner
        class_<PyPlannerBase::PyPlannerParameters, PyPlannerBase::PyPlannerParametersPtr >(planner, "PlannerParameters", DOXY_CLASS(PlannerBase::PlannerParameters))
#else
        class_<PyPlannerBase::PyPlannerParameters, PyPlannerBase::PyPlannerParametersPtr >("PlannerParameters", DOXY_CLASS(PlannerBase::PlannerParameters))
#endif
        .def(init<>())
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def(init<PyPlannerBase::PyPlannerParametersPtr>(), "parameters"_a)
#else
        .def(init<PyPlannerBase::PyPlannerParametersPtr>(py::args("parameters")))
#endif
        .def("SetRobotActiveJoints",&PyPlannerBase::PyPlannerParameters::SetRobotActiveJoints, PY_ARGS("robot") DOXY_FN(PlannerBase::PlannerParameters, SetRobotActiveJoints))
        .def("SetConfigurationSpecification",&PyPlannerBase::PyPlannerParameters::SetConfigurationSpecification, PY_ARGS("env","spec") DOXY_FN(PlannerBase::PlannerParameters, SetConfigurationSpecification))
        .def("GetConfigurationSpecification",&PyPlannerBase::PyPlannerParameters::GetConfigurationSpecification, DOXY_FN(PlannerBase::PlannerParameters, GetConfigurationSpecification))
        .def("SetExtraParameters",&PyPlannerBase::PyPlannerParameters::SetExtraParameters, PY_ARGS("extra") DOXY_FN(PlannerBase::PlannerParameters, SetExtraParameters))
        .def("SetRandomGeneratorSeed",&PyPlannerBase::PyPlannerParameters::SetRandomGeneratorSeed, PY_ARGS("seed") DOXY_FN(PlannerBase::PlannerParameters, SetRandomGeneratorSeed))
        .def("SetGoalConfig",&PyPlannerBase::PyPlannerParameters::SetGoalConfig, PY_ARGS("values") "sets Planne Parameters::vgoalconfig")
        .def("SetInitialConfig",&PyPlannerBase::PyPlannerParameters::SetInitialConfig, PY_ARGS("values") "sets PlannerParameters::vinitialconfig")
        .def("SetInitialConfigVelocities",&PyPlannerBase::PyPlannerParameters::SetInitialConfigVelocities, PY_ARGS("velocities") "sets PlannerParameters::_vInitialConfigVelocities")
        .def("SetGoalConfigVelocities",&PyPlannerBase::PyPlannerParameters::SetGoalConfigVelocities, PY_ARGS("velocities") "sets PlannerParameters::_vGoalConfigVelocities")
        .def("SetConfigVelocityLimit",&PyPlannerBase::PyPlannerParameters::SetConfigVelocityLimit, PY_ARGS("velocities") "sets PlannerParameters::_vConfigVelocityLimit")
        .def("SetConfigAccelerationLimit",&PyPlannerBase::PyPlannerParameters::SetConfigAccelerationLimit, PY_ARGS("accelerations") "sets PlannerParameters::_vConfigAccelerationLimit")
        .def("SetConfigResolution",&PyPlannerBase::PyPlannerParameters::SetConfigResolution, PY_ARGS("resolutions") "sets PlannerParameters::_vConfigResolution")
        .def("SetMaxIterations",&PyPlannerBase::PyPlannerParameters::SetMaxIterations, PY_ARGS("maxiterations") "sets PlannerParameters::_nMaxIterations")
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("CheckPathAllConstraints", &PyPlannerBase::PyPlannerParameters::CheckPathAllConstraints,
             "q0"_a,
             "q1"_a,
             "dq0"_a,
             "dq1"_a,
             "timeelapsed"_a,
             "interval"_a,
             "options"_a = 0xffff,
             "filterreturn"_a = false,
             DOXY_FN(PlannerBase::PlannerParameters, CheckPathAllConstraints)
             )
#else
        .def("CheckPathAllConstraints",&PyPlannerBase::PyPlannerParameters::CheckPathAllConstraints,CheckPathAllConstraints_overloads(PY_ARGS("q0","q1","dq0","dq1","timeelapsed","interval","options", "filterreturn") DOXY_FN(PlannerBase::PlannerParameters, CheckPathAllConstraints)))
#endif
        .def("SetPostProcessing", &PyPlannerBase::PyPlannerParameters::SetPostProcessing, PY_ARGS("plannername", "plannerparameters") "sets the post processing parameters")
        .def("__str__",&PyPlannerBase::PyPlannerParameters::__str__)
        .def("__unicode__",&PyPlannerBase::PyPlannerParameters::__unicode__)
        .def("__repr__",&PyPlannerBase::PyPlannerParameters::__repr__)
        .def("__eq__",&PyPlannerBase::PyPlannerParameters::__eq__)
        .def("__ne__",&PyPlannerBase::PyPlannerParameters::__ne__)
        ;
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreatePlanner", openravepy::RaveCreatePlanner, PY_ARGS("env","name") DOXY_FN1(RaveCreatePlanner));
#else
    def("RaveCreatePlanner",openravepy::RaveCreatePlanner, PY_ARGS("env","name") DOXY_FN1(RaveCreatePlanner));
#endif
}

}
