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


class PyPlannerStatus
{
public:
    PyPlannerStatus() {
        statusCode = 0;
        jointValues = numeric::array(py::list());
    }

    PyPlannerStatus(const PlannerStatus& status) {
        // Just serializeToJason?
        description = ConvertStringToUnicode(status.description);
        errorOrigin = ConvertStringToUnicode(status.errorOrigin);
        statusCode = status.statusCode;
        jointValues = toPyArray(status.jointValues);

        if( !status.report ) {
            //_report = "";
            report = object();
        }
        else {
            //_report = status._report->__str__();
            report = object(openravepy::toPyCollisionReport(status.report, NULL));
        }

        ikparam = toPyIkParameterization(status.ikparam);
    }

    object report;
    //std::string _report;
    object description;
    object errorOrigin;
    object jointValues;
    object ikparam;
    uint32_t statusCode;
};

object toPyPlannerStatus(const PlannerStatus& status)
{
    return object(boost::shared_ptr<PyPlannerStatus>(new PyPlannerStatus(status)));
}

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

        PlannerBase::PlannerParametersPtr GetParameters() {
            return _paramswrite;
        }

        void SetRobotActiveJoints(PyRobotBasePtr robot)
        {
            if( !_paramswrite ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("PlannerParameters needs to be non-const"),ORE_Failed);
            }
            _paramswrite->SetRobotActiveJoints(openravepy::GetRobot(robot));
        }

        void SetConfigurationSpecification(PyEnvironmentBasePtr pyenv, PyConfigurationSpecificationPtr pyspec) {
            _paramswrite->SetConfigurationSpecification(openravepy::GetEnvironment(pyenv), openravepy::GetConfigurationSpecification(pyspec));
        }

        object GetConfigurationSpecification() const {
            return object(openravepy::toPyConfigurationSpecification(_paramswrite->_configurationspecification));
        }

        void SetExtraParameters(const std::string& s) {
            _paramswrite->_sExtraParameters = s;
        }

        void SetRandomGeneratorSeed(uint32_t seed) {
            _paramswrite->_nRandomGeneratorSeed = seed;
        }

        void SetGoalConfig(object o)
        {
            _paramswrite->vgoalconfig = ExtractArray<dReal>(o);
        }

        void SetInitialConfig(object o)
        {
            _paramswrite->vinitialconfig = ExtractArray<dReal>(o);
        }

        void SetInitialConfigVelocities(object o)
        {
            _paramswrite->_vInitialConfigVelocities = ExtractArray<dReal>(o);
        }

        void SetGoalConfigVelocities(object o)
        {
            _paramswrite->_vGoalConfigVelocities = ExtractArray<dReal>(o);
        }

        void SetConfigVelocityLimit(object o)
        {
            _paramswrite->_vConfigVelocityLimit = ExtractArray<dReal>(o);
        }

        void SetConfigAccelerationLimit(object o)
        {
            _paramswrite->_vConfigAccelerationLimit = ExtractArray<dReal>(o);
        }

        void SetConfigResolution(object o)
        {
            _paramswrite->_vConfigResolution = ExtractArray<dReal>(o);
        }

        void SetMaxIterations(int nMaxIterations)
        {
            _paramswrite->_nMaxIterations = nMaxIterations;
        }

        object CheckPathAllConstraints(object oq0, object oq1, object odq0, object odq1, dReal timeelapsed, IntervalType interval, uint32_t options=0xffff, bool filterreturn=false)
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
            return object(ret);
        }

        void SetPostProcessing(const std::string& plannername, const std::string& plannerparameters)
        {
            _paramswrite->_sPostProcessingPlanner = plannername;
            _paramswrite->_sPostProcessingParameters = plannerparameters;
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

    object PlanPath(PyTrajectoryBasePtr pytraj,bool releasegil=true)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        PlannerStatus status = _pplanner->PlanPath(ptraj);
        statesaver.reset(); // unlock
        return openravepy::toPyPlannerStatus(status);
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
        PlannerAction ret = PA_None;
        if( IS_PYTHONOBJECT_NONE(res) || !res ) {
            ret = PA_None;
            RAVELOG_WARN("plan callback nothing returning, so executing default action\n");
        }
        else {
            extract<PlannerAction> xb(res);
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

    object RegisterPlanCallback(object fncallback)
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

PlannerBase::PlannerParametersPtr GetPlannerParameters(object o)
{
    extract<PyPlannerBase::PyPlannerParametersPtr> pyparams(o);
    if( pyparams.check() ) {
        return ((PyPlannerBase::PyPlannerParametersPtr)pyparams)->GetParameters();
    }
    return PlannerBase::PlannerParametersPtr();
}

PlannerBase::PlannerParametersConstPtr GetPlannerParametersConst(object o)
{
    extract<PyPlannerBase::PyPlannerParametersPtr> pyparams(o);
    if( pyparams.check() ) {
        return ((PyPlannerBase::PyPlannerParametersPtr)pyparams)->GetParameters();
    }
    return PlannerBase::PlannerParametersPtr();
}

object toPyPlannerParameters(PlannerBase::PlannerParametersPtr params)
{
    if( !params ) {
        return object();
    }
    return object(PyPlannerBase::PyPlannerParametersPtr(new PyPlannerBase::PyPlannerParameters(params)));
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
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckPathAllConstraints_overloads, CheckPathAllConstraints, 6, 8)

void init_openravepy_planner()
{

    object plannerstatuscode = enum_<PlannerStatusCode>("PlannerStatusCode" DOXY_ENUM(PlannerStatusCode))
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

    class_<PyPlannerStatus, boost::shared_ptr<PyPlannerStatus> >("PlannerStatus", DOXY_CLASS(PlannerStatus))
    .def_readwrite("report",&PyPlannerStatus::report)
    .def_readwrite("description",&PyPlannerStatus::description)
    .def_readwrite("errorOrigin",&PyPlannerStatus::errorOrigin)
    .def_readwrite("jointValues",&PyPlannerStatus::jointValues)
    .def_readwrite("ikparam",&PyPlannerStatus::ikparam)
    .def_readwrite("statusCode",&PyPlannerStatus::statusCode)
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
                        .def("RegisterPlanCallback",&PyPlannerBase::RegisterPlanCallback, DOXY_FN(PlannerBase,RegisterPlanCallback))
        ;

        class_<PyPlannerBase::PyPlannerParameters, PyPlannerBase::PyPlannerParametersPtr >("PlannerParameters", DOXY_CLASS(PlannerBase::PlannerParameters))
        .def(init<>())
        .def(init<PyPlannerBase::PyPlannerParametersPtr>(args("parameters")))
        .def("SetRobotActiveJoints",&PyPlannerBase::PyPlannerParameters::SetRobotActiveJoints, args("robot"), DOXY_FN(PlannerBase::PlannerParameters, SetRobotActiveJoints))
        .def("SetConfigurationSpecification",&PyPlannerBase::PyPlannerParameters::SetConfigurationSpecification, args("env","spec"), DOXY_FN(PlannerBase::PlannerParameters, SetConfigurationSpecification))
        .def("GetConfigurationSpecification",&PyPlannerBase::PyPlannerParameters::GetConfigurationSpecification, DOXY_FN(PlannerBase::PlannerParameters, GetConfigurationSpecification))
        .def("SetExtraParameters",&PyPlannerBase::PyPlannerParameters::SetExtraParameters, args("extra"), DOXY_FN(PlannerBase::PlannerParameters, SetExtraParameters))
        .def("SetRandomGeneratorSeed",&PyPlannerBase::PyPlannerParameters::SetRandomGeneratorSeed, args("seed"), DOXY_FN(PlannerBase::PlannerParameters, SetRandomGeneratorSeed))
        .def("SetGoalConfig",&PyPlannerBase::PyPlannerParameters::SetGoalConfig,args("values"),"sets PlannerParameters::vgoalconfig")
        .def("SetInitialConfig",&PyPlannerBase::PyPlannerParameters::SetInitialConfig,args("values"),"sets PlannerParameters::vinitialconfig")
        .def("SetInitialConfigVelocities",&PyPlannerBase::PyPlannerParameters::SetInitialConfigVelocities,args("velocities"),"sets PlannerParameters::_vInitialConfigVelocities")
        .def("SetGoalConfigVelocities",&PyPlannerBase::PyPlannerParameters::SetGoalConfigVelocities,args("velocities"),"sets PlannerParameters::_vGoalConfigVelocities")
        .def("SetConfigVelocityLimit",&PyPlannerBase::PyPlannerParameters::SetConfigVelocityLimit,args("velocities"),"sets PlannerParameters::_vConfigVelocityLimit")
        .def("SetConfigAccelerationLimit",&PyPlannerBase::PyPlannerParameters::SetConfigAccelerationLimit,args("accelerations"),"sets PlannerParameters::_vConfigAccelerationLimit")
        .def("SetConfigResolution",&PyPlannerBase::PyPlannerParameters::SetConfigResolution,args("resolutions"),"sets PlannerParameters::_vConfigResolution")
        .def("SetMaxIterations",&PyPlannerBase::PyPlannerParameters::SetMaxIterations,args("maxiterations"),"sets PlannerParameters::_nMaxIterations")
        .def("CheckPathAllConstraints",&PyPlannerBase::PyPlannerParameters::CheckPathAllConstraints,CheckPathAllConstraints_overloads(args("q0","q1","dq0","dq1","timeelapsed","interval","options", "filterreturn"),DOXY_FN(PlannerBase::PlannerParameters, CheckPathAllConstraints)))
        .def("SetPostProcessing", &PyPlannerBase::PyPlannerParameters::SetPostProcessing, args("plannername", "plannerparameters"), "sets the post processing parameters")
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
