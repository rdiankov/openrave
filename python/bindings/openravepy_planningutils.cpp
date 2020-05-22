// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include <openravepy/openravepy_collisionreport.h>
#include <openravepy/openravepy_trajectorybase.h>
#include <openravepy/openravepy_kinbody.h>
#include <openravepy/openravepy_configurationspecification.h>
#include <openravepy/openravepy_robotbase.h>
#include <openravepy/openravepy_plannerbase.h>
#include <openrave/planningutils.h>

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
using py::optional;
using py::manage_new_object;
using py::def;
#endif // USE_PYBIND11_PYTHON_BINDINGS

namespace numeric = py::numeric;

namespace planningutils
{

class PyStaticClass
{
public:
};

bool pyJitterCurrentConfiguration(object pyplannerparameters, int maxiterations=5000, dReal maxjitter=0.015, dReal perturbation=1e-5)
{
    return OpenRAVE::planningutils::JitterCurrentConfiguration(openravepy::GetPlannerParametersConst(pyplannerparameters), maxiterations, maxjitter, perturbation);
}

bool pyJitterTransform(PyKinBodyPtr pybody, dReal fJitter, int nMaxIterations=1000)
{
    return OpenRAVE::planningutils::JitterTransform(openravepy::GetKinBody(pybody), fJitter, nMaxIterations);
}

void pyConvertTrajectorySpecification(PyTrajectoryBasePtr pytraj, PyConfigurationSpecificationPtr pyspec)
{
    OpenRAVE::planningutils::ConvertTrajectorySpecification(openravepy::GetTrajectory(pytraj),openravepy::GetConfigurationSpecification(pyspec));
}

void pyComputeTrajectoryDerivatives(PyTrajectoryBasePtr pytraj, int maxderiv)
{
    OpenRAVE::planningutils::ComputeTrajectoryDerivatives(openravepy::GetTrajectory(pytraj),maxderiv);
}

object pyReverseTrajectory(PyTrajectoryBasePtr pytraj)
{
    return py::to_object(openravepy::toPyTrajectory(OpenRAVE::planningutils::ReverseTrajectory(openravepy::GetTrajectory(pytraj)),openravepy::toPyEnvironment(pytraj)));
}

object pyGetReverseTrajectory(PyTrajectoryBasePtr pytraj)
{
    return py::to_object(openravepy::toPyTrajectory(OpenRAVE::planningutils::GetReverseTrajectory(openravepy::GetTrajectory(pytraj)),openravepy::toPyEnvironment(pytraj)));
}

void pyVerifyTrajectory(object pyparameters, PyTrajectoryBasePtr pytraj, dReal samplingstep)
{
    OpenRAVE::planningutils::VerifyTrajectory(openravepy::GetPlannerParametersConst(pyparameters), openravepy::GetTrajectory(pytraj),samplingstep);
}

// GIL is assumed locked
object pySmoothActiveDOFTrajectory(PyTrajectoryBasePtr pytraj, PyRobotBasePtr pyrobot, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return openravepy::toPyPlannerStatus(OpenRAVE::planningutils::SmoothActiveDOFTrajectory(openravepy::GetTrajectory(pytraj),openravepy::GetRobot(pyrobot),fmaxvelmult,fmaxaccelmult,plannername,plannerparameters));
}

class PyActiveDOFTrajectorySmoother
{
public:
    PyActiveDOFTrajectorySmoother(PyRobotBasePtr pyrobot, const std::string& plannername, const std::string& plannerparameters) : _smoother(openravepy::GetRobot(pyrobot), plannername, plannerparameters) {
    }
    virtual ~PyActiveDOFTrajectorySmoother() {
    }

    object PlanPath(PyTrajectoryBasePtr pytraj, bool releasegil=true)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        PlannerStatus status = _smoother.PlanPath(ptraj);
        statesaver.reset(); // re-lock GIL
        return openravepy::toPyPlannerStatus(status);
    }

    OpenRAVE::planningutils::ActiveDOFTrajectorySmoother _smoother;
};

typedef OPENRAVE_SHARED_PTR<PyActiveDOFTrajectorySmoother> PyActiveDOFTrajectorySmootherPtr;

// assume python GIL is locked
object pySmoothAffineTrajectory(PyTrajectoryBasePtr pytraj, object omaxvelocities, object omaxaccelerations, const std::string& plannername="", const std::string& plannerparameters="")
{
    return openravepy::toPyPlannerStatus(OpenRAVE::planningutils::SmoothAffineTrajectory(openravepy::GetTrajectory(pytraj),ExtractArray<dReal>(omaxvelocities), ExtractArray<dReal>(omaxaccelerations),plannername,plannerparameters));
}

// assume python GIL is locked
object pySmoothTrajectory(PyTrajectoryBasePtr pytraj, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return openravepy::toPyPlannerStatus(OpenRAVE::planningutils::SmoothTrajectory(openravepy::GetTrajectory(pytraj),fmaxvelmult,fmaxaccelmult,plannername,plannerparameters));
}

// assume python GIL is locked
object pyRetimeActiveDOFTrajectory(PyTrajectoryBasePtr pytraj, PyRobotBasePtr pyrobot, bool hastimestamps=false, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return openravepy::toPyPlannerStatus(OpenRAVE::planningutils::RetimeActiveDOFTrajectory(openravepy::GetTrajectory(pytraj),openravepy::GetRobot(pyrobot),hastimestamps,fmaxvelmult,fmaxaccelmult,plannername,plannerparameters));
}

class PyActiveDOFTrajectoryRetimer
{
public:
    PyActiveDOFTrajectoryRetimer(PyRobotBasePtr pyrobot, const std::string& plannername, const std::string& plannerparameters) : _retimer(openravepy::GetRobot(pyrobot), plannername, plannerparameters) {
    }
    virtual ~PyActiveDOFTrajectoryRetimer() {
    }

    object PlanPath(PyTrajectoryBasePtr pytraj, bool hastimestamps=false, bool releasegil=true)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        PlannerStatus status = _retimer.PlanPath(ptraj, hastimestamps);
        statesaver.reset(); // re-lock GIL
        return openravepy::toPyPlannerStatus(status);
    }

    OpenRAVE::planningutils::ActiveDOFTrajectoryRetimer _retimer;
};

typedef OPENRAVE_SHARED_PTR<PyActiveDOFTrajectoryRetimer> PyActiveDOFTrajectoryRetimerPtr;

class PyAffineTrajectoryRetimer
{
public:
    PyAffineTrajectoryRetimer(const std::string& plannername, const std::string& plannerparameters) : _retimer(plannername, plannerparameters) {
    }
    virtual ~PyAffineTrajectoryRetimer() {
    }

    object PlanPath(PyTrajectoryBasePtr pytraj, object omaxvelocities, object omaxaccelerations, bool hastimestamps=false, bool releasegil=true)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
        std::vector<dReal> vmaxvelocities = ExtractArray<dReal>(omaxvelocities);
        std::vector<dReal> vmaxaccelerations = ExtractArray<dReal>(omaxaccelerations);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        PlannerStatus status = _retimer.PlanPath(ptraj,vmaxvelocities, vmaxaccelerations, hastimestamps);
        statesaver.reset(); // to re-lock the GIL
        return openravepy::toPyPlannerStatus(status);
    }

    OpenRAVE::planningutils::AffineTrajectoryRetimer _retimer;
};

typedef OPENRAVE_SHARED_PTR<PyAffineTrajectoryRetimer> PyAffineTrajectoryRetimerPtr;

class PyDynamicsCollisionConstraint
{
public:
    PyDynamicsCollisionConstraint(object oparameters, object olistCheckBodies, uint32_t filtermask=0xffffffff)
    {
        PlannerBase::PlannerParametersConstPtr parameters = openravepy::GetPlannerParametersConst(oparameters);
        std::list<KinBodyPtr> listCheckBodies;
        for(size_t i = 0; i < len(olistCheckBodies); ++i) {
            KinBodyPtr pbody = openravepy::GetKinBody(olistCheckBodies[i]);
            BOOST_ASSERT(!!pbody);
            _pyenv = GetPyEnvFromPyKinBody(olistCheckBodies[i]);
            listCheckBodies.push_back(pbody);
        }
        _pconstraints.reset(new OpenRAVE::planningutils::DynamicsCollisionConstraint(parameters, listCheckBodies, filtermask));
    }

    virtual ~PyDynamicsCollisionConstraint() {
    }

    object Check(object oq0, object oq1, object odq0, object odq1, dReal timeelapsed, IntervalType interval=IT_Closed, uint32_t options=0xffff, bool filterreturn=false)//, ConstraintFilterReturnPtr filterreturn = ConstraintFilterReturnPtr())
    {
        std::vector<dReal> q0 = ExtractArray<dReal>(oq0);
        std::vector<dReal> q1 = ExtractArray<dReal>(oq1);
        std::vector<dReal> dq0 = ExtractArray<dReal>(odq0);
        std::vector<dReal> dq1 = ExtractArray<dReal>(odq1);
        if( filterreturn ) {
            ConstraintFilterReturnPtr pfilterreturn(new ConstraintFilterReturn());
            _pconstraints->Check(q0, q1, dq0, dq1, timeelapsed, interval, options, pfilterreturn);
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
        else {
            return py::to_object(_pconstraints->Check(q0, q1, dq0, dq1, timeelapsed, interval, options));
        }
    }

    object GetReport() const {
        if( !_pconstraints->GetReport() ) {
            return py::none_();
        }
        return py::to_object(openravepy::toPyCollisionReport(_pconstraints->GetReport(), _pyenv));
    }

    void SetPlannerParameters(object oparameters)
    {
        _pconstraints->SetPlannerParameters(openravepy::GetPlannerParameters(oparameters));
    }

    void SetFilterMask(int filtermask) {
        _pconstraints->SetFilterMask(filtermask);
    }

    void SetPerturbation(dReal perturbation) {
        _pconstraints->SetPerturbation(perturbation);
    }

    void SetTorqueLimitMode(int torquelimitmode) {
        _pconstraints->SetTorqueLimitMode(static_cast<DynamicsConstraintsType>(torquelimitmode));
    }


    PyEnvironmentBasePtr _pyenv;
    OpenRAVE::planningutils::DynamicsCollisionConstraintPtr _pconstraints;
};

typedef OPENRAVE_SHARED_PTR<PyDynamicsCollisionConstraint> PyDynamicsCollisionConstraintPtr;

object pyRetimeAffineTrajectory(PyTrajectoryBasePtr pytraj, object omaxvelocities, object omaxaccelerations, bool hastimestamps=false, const std::string& plannername="", const std::string& plannerparameters="")
{
    return openravepy::toPyPlannerStatus(OpenRAVE::planningutils::RetimeAffineTrajectory(openravepy::GetTrajectory(pytraj),ExtractArray<dReal>(omaxvelocities), ExtractArray<dReal>(omaxaccelerations),hastimestamps,plannername,plannerparameters));
}

object pyRetimeTrajectory(PyTrajectoryBasePtr pytraj, bool hastimestamps=false, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return openravepy::toPyPlannerStatus(OpenRAVE::planningutils::RetimeTrajectory(openravepy::GetTrajectory(pytraj),hastimestamps,fmaxvelmult,fmaxaccelmult,plannername,plannerparameters));
}

size_t pyExtendWaypoint(int index, object odofvalues, object odofvelocities, PyTrajectoryBasePtr pytraj, PyPlannerBasePtr pyplanner)
{
    return OpenRAVE::planningutils::ExtendWaypoint(index, ExtractArray<dReal>(odofvalues), ExtractArray<dReal>(odofvelocities), openravepy::GetTrajectory(pytraj), openravepy::GetPlanner(pyplanner));
}

size_t pyExtendActiveDOFWaypoint(int index, object odofvalues, object odofvelocities, PyTrajectoryBasePtr pytraj, PyRobotBasePtr pyrobot, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="")
{
    return OpenRAVE::planningutils::ExtendActiveDOFWaypoint(index, ExtractArray<dReal>(odofvalues), ExtractArray<dReal>(odofvelocities), openravepy::GetTrajectory(pytraj), openravepy::GetRobot(pyrobot), fmaxvelmult, fmaxaccelmult, plannername);
}

size_t pyInsertActiveDOFWaypointWithRetiming(int index, object odofvalues, object odofvelocities, PyTrajectoryBasePtr pytraj, PyRobotBasePtr pyrobot, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="", const std::string& plannerparameters="")
{
    return OpenRAVE::planningutils::InsertActiveDOFWaypointWithRetiming(index, ExtractArray<dReal>(odofvalues), ExtractArray<dReal>(odofvelocities), openravepy::GetTrajectory(pytraj), openravepy::GetRobot(pyrobot), fmaxvelmult, fmaxaccelmult, plannername, plannerparameters);
}

size_t pyInsertWaypointWithSmoothing(int index, object odofvalues, object odofvelocities, PyTrajectoryBasePtr pytraj, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="")
{
    return OpenRAVE::planningutils::InsertWaypointWithSmoothing(index,ExtractArray<dReal>(odofvalues),ExtractArray<dReal>(odofvelocities),openravepy::GetTrajectory(pytraj),fmaxvelmult,fmaxaccelmult,plannername);
}

void pySegmentTrajectory(PyTrajectoryBasePtr pytraj, dReal starttime, dReal endtime)
{
    OpenRAVE::planningutils::SegmentTrajectory(openravepy::GetTrajectory(pytraj), starttime, endtime);
}

object pyGetTrajectorySegment(PyTrajectoryBasePtr pytraj, dReal starttime, dReal endtime)
{
    PyEnvironmentBasePtr pyenv = openravepy::toPyEnvironment(pytraj);
    return py::to_object(openravepy::toPyTrajectory(OpenRAVE::planningutils::GetTrajectorySegment(openravepy::GetTrajectory(pytraj), starttime, endtime), pyenv));
}

object pyMergeTrajectories(object pytrajectories)
{
    std::list<TrajectoryBaseConstPtr> listtrajectories;
    PyEnvironmentBasePtr pyenv;
    for(size_t i = 0; i < len(pytrajectories); ++i) {
        extract_<PyTrajectoryBasePtr> epytrajectory(pytrajectories[i]);
        PyTrajectoryBasePtr pytrajectory = (PyTrajectoryBasePtr)epytrajectory;
        if( !pyenv ) {
            pyenv = openravepy::toPyEnvironment(pytrajectory);
        }
        else {
            BOOST_ASSERT(pyenv == openravepy::toPyEnvironment(pytrajectory));
        }
        listtrajectories.push_back(openravepy::GetTrajectory(pytrajectory));
    }
    return py::to_object(openravepy::toPyTrajectory(OpenRAVE::planningutils::MergeTrajectories(listtrajectories),pyenv));
}

class PyDHParameter
{
public:
    PyDHParameter() {
    }
    PyDHParameter(const OpenRAVE::planningutils::DHParameter& p, PyEnvironmentBasePtr pyenv) : joint(toPyKinBodyJoint(OPENRAVE_CONST_POINTER_CAST<KinBody::Joint>(p.joint), pyenv)), parentindex(p.parentindex), transform(ReturnTransform(p.transform)), d(p.d), a(p.a), theta(p.theta), alpha(p.alpha) {
    }
    PyDHParameter(object joint, int parentindex, object transform, dReal d, dReal a, dReal theta, dReal alpha) : joint(joint), parentindex(parentindex), transform(transform), d(d), a(a), theta(theta), alpha(alpha) {
    }
    virtual ~PyDHParameter() {
    }
    std::string __repr__() {
        return boost::str(boost::format("<DHParameter(joint=%s, parentindex=%d, d=%f, a=%f, theta=%f, alpha=%f)>")%reprPyKinBodyJoint(joint)%parentindex%d%a%theta%alpha);
    }
    std::string __str__() {
        TransformMatrix tm = ExtractTransformMatrix(transform);
        return boost::str(boost::format("<joint %s, transform [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f]], parentindex %d>")%strPyKinBodyJoint(joint)%tm.m[0]%tm.m[1]%tm.m[2]%tm.trans[0]%tm.m[4]%tm.m[5]%tm.m[6]%tm.trans[1]%tm.m[8]%tm.m[9]%tm.m[10]%tm.trans[2]%parentindex);
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    object joint = py::none_();
    int parentindex = -1;
    object transform = ReturnTransform(Transform());
    dReal d = 0.0;
    dReal a = 0.0;
    dReal theta = 0.0;
    dReal alpha = 0.0;
};

object toPyDHParameter(const OpenRAVE::planningutils::DHParameter& p, PyEnvironmentBasePtr pyenv)
{
    return py::to_object(OPENRAVE_SHARED_PTR<PyDHParameter>(new PyDHParameter(p,pyenv)));
}

class DHParameter_pickle_suite
#ifndef USE_PYBIND11_PYTHON_BINDINGS
    : public pickle_suite
#endif
{
public:
    static py::tuple getinitargs(const PyDHParameter& p)
    {
        return py::make_tuple(py::none_(), p.parentindex, p.transform, p.d, p.a, p.theta, p.alpha);
    }
};

py::list pyGetDHParameters(PyKinBodyPtr pybody)
{
    py::list oparameters;
    std::vector<OpenRAVE::planningutils::DHParameter> vparameters;
    OpenRAVE::planningutils::GetDHParameters(vparameters,openravepy::GetKinBody(pybody));
    PyEnvironmentBasePtr pyenv = toPyEnvironment(pybody);
    FOREACH(itp,vparameters) {
        oparameters.append(toPyDHParameter(*itp,pyenv));
    }
    return oparameters;
}

class PyManipulatorIKGoalSampler
{
public:
    PyManipulatorIKGoalSampler(object pymanip, object oparameterizations, int nummaxsamples=20, int nummaxtries=10, dReal jitter=0, bool searchfreeparameters=true, uint32_t ikfilteroptions = IKFO_CheckEnvCollisions, object freevalues = py::none_()) {
        std::list<IkParameterization> listparameterizationsPtr;
        size_t num = len(oparameterizations);
        for(size_t i = 0; i < num; ++i) {
            IkParameterization ikparam;
            if( ExtractIkParameterization(oparameterizations[i],ikparam) ) {
                listparameterizationsPtr.push_back(ikparam);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("ManipulatorIKGoalSampler parameterizations need to be all IkParameterization objeccts"),ORE_InvalidArguments);
            }
        }
        dReal fsampleprob=1;
        std::vector<dReal> vfreevalues = ExtractArray<dReal>(freevalues);
        _sampler.reset(new OpenRAVE::planningutils::ManipulatorIKGoalSampler(GetRobotManipulator(pymanip), listparameterizationsPtr, nummaxsamples, nummaxtries, fsampleprob, searchfreeparameters, ikfilteroptions, vfreevalues));
        _sampler->SetJitter(jitter);
    }
    virtual ~PyManipulatorIKGoalSampler() {
    }

    object Sample(bool ikreturn = false, bool releasegil = false)
    {
        if( ikreturn ) {
            IkReturnPtr pikreturn = _sampler->Sample();
            if( !!pikreturn ) {
                return openravepy::toPyIkReturn(*pikreturn);
            }
        }
        else {
            std::vector<dReal> vgoal;
            if( _sampler->Sample(vgoal) ) {
                return toPyArray(vgoal);
            }
        }
        return py::none_();
    }

    object SampleAll(int maxsamples=0, int maxchecksamples=0, bool releasegil = false)
    {
        py::list oreturns;
        std::list<IkReturnPtr> listreturns;
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            _sampler->SampleAll(listreturns, maxsamples, maxchecksamples);
        }
        FOREACH(it,listreturns) {
            oreturns.append(openravepy::toPyIkReturn(**it));
        }
        return oreturns;
    }

    int GetIkParameterizationIndex(int index)
    {
        return _sampler->GetIkParameterizationIndex(index);
    }

    OpenRAVE::planningutils::ManipulatorIKGoalSamplerPtr _sampler;
};

typedef OPENRAVE_SHARED_PTR<PyManipulatorIKGoalSampler> PyManipulatorIKGoalSamplerPtr;


} // end namespace planningutils

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Sample_overloads, Sample, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SampleAll_overloads, SampleAll, 0, 3)

BOOST_PYTHON_FUNCTION_OVERLOADS(JitterCurrentConfiguration_overloads, planningutils::pyJitterCurrentConfiguration, 1, 4);
BOOST_PYTHON_FUNCTION_OVERLOADS(JitterTransform_overloads, planningutils::pyJitterTransform, 2, 3);
BOOST_PYTHON_FUNCTION_OVERLOADS(SmoothActiveDOFTrajectory_overloads, planningutils::pySmoothActiveDOFTrajectory, 2, 6)
BOOST_PYTHON_FUNCTION_OVERLOADS(SmoothAffineTrajectory_overloads, planningutils::pySmoothAffineTrajectory, 3, 5)
BOOST_PYTHON_FUNCTION_OVERLOADS(SmoothTrajectory_overloads, planningutils::pySmoothTrajectory, 1, 5)
BOOST_PYTHON_FUNCTION_OVERLOADS(RetimeActiveDOFTrajectory_overloads, planningutils::pyRetimeActiveDOFTrajectory, 2, 7)
BOOST_PYTHON_FUNCTION_OVERLOADS(RetimeAffineTrajectory_overloads, planningutils::pyRetimeAffineTrajectory, 3, 6)
BOOST_PYTHON_FUNCTION_OVERLOADS(RetimeTrajectory_overloads, planningutils::pyRetimeTrajectory, 1, 6)
BOOST_PYTHON_FUNCTION_OVERLOADS(ExtendActiveDOFWaypoint_overloads, planningutils::pyExtendActiveDOFWaypoint, 5, 8)
BOOST_PYTHON_FUNCTION_OVERLOADS(InsertActiveDOFWaypointWithRetiming_overloads, planningutils::pyInsertActiveDOFWaypointWithRetiming, 5, 9)
BOOST_PYTHON_FUNCTION_OVERLOADS(InsertWaypointWithSmoothing_overloads, planningutils::pyInsertWaypointWithSmoothing, 4, 7)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Check_overloads, Check, 5, 8)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PlanPath_overloads, PlanPath, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PlanPath_overloads2, PlanPath, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PlanPath_overloads3, PlanPath, 1, 3)
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void InitPlanningUtils(py::module& m)
#else
void InitPlanningUtils()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
#endif
    {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ planningutils = class_<planningutils::PyStaticClass>(m, "planningutils")
#else
        scope_ planningutils = class_<planningutils::PyStaticClass>("planningutils")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("JitterTransform", planningutils::pyJitterTransform,
                                           "body"_a,
                                           "jitter"_a,
                                           "maxiterations"_a = 1000,
                                           DOXY_FN1(JitterTransform)
                                           )
#else
                               .def("JitterTransform",planningutils::pyJitterTransform,JitterTransform_overloads(PY_ARGS("body","jitter","maxiterations") DOXY_FN1(JitterTransform)))
                               .staticmethod("JitterTransform")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("JitterCurrentConfiguration", planningutils::pyJitterCurrentConfiguration,
                                           "plannerparameters"_a,
                                           "maxiterations"_a = 5000,
                                           "jitter"_a = 0.015,
                                           "perturbation"_a = 1e-5,
                                           DOXY_FN1(JitterCurrentConfiguration)
                                           )
#else
                               .def("JitterCurrentConfiguration",planningutils::pyJitterCurrentConfiguration,JitterCurrentConfiguration_overloads(PY_ARGS("plannerparameters","maxiterations", "jitter", "perturbation") DOXY_FN1(JitterCurrentConfiguration)))
                               .staticmethod("JitterCurrentConfiguration")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("ConvertTrajectorySpecification",planningutils::pyConvertTrajectorySpecification, PY_ARGS("trajectory","spec") DOXY_FN1(ConvertTrajectorySpecification))
#else
                               .def("ConvertTrajectorySpecification",planningutils::pyConvertTrajectorySpecification, PY_ARGS("trajectory","spec") DOXY_FN1(ConvertTrajectorySpecification))
                               .staticmethod("ConvertTrajectorySpecification")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("ComputeTrajectoryDerivatives",planningutils::pyComputeTrajectoryDerivatives, PY_ARGS("trajectory","maxderiv") DOXY_FN1(ComputeTrajectoryDerivatives))
#else
                               .def("ComputeTrajectoryDerivatives",planningutils::pyComputeTrajectoryDerivatives, PY_ARGS("trajectory","maxderiv") DOXY_FN1(ComputeTrajectoryDerivatives))
                               .staticmethod("ComputeTrajectoryDerivatives")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("ReverseTrajectory",planningutils::pyReverseTrajectory, PY_ARGS("trajectory") DOXY_FN1(ReverseTrajectory))
#else
                               .def("ReverseTrajectory",planningutils::pyReverseTrajectory, PY_ARGS("trajectory") DOXY_FN1(ReverseTrajectory))
                               .staticmethod("ReverseTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("VerifyTrajectory",planningutils::pyVerifyTrajectory, PY_ARGS("parameters","trajectory","samplingstep") DOXY_FN1(VerifyTrajectory))
#else
                               .def("VerifyTrajectory",planningutils::pyVerifyTrajectory, PY_ARGS("parameters","trajectory","samplingstep") DOXY_FN1(VerifyTrajectory))
                               .staticmethod("VerifyTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("SmoothActiveDOFTrajectory", planningutils::pySmoothActiveDOFTrajectory,
                                           "trajectory"_a,
                                           "robot"_a,
                                           "maxvelmult"_a = 1.0,
                                           "maxaccelmult"_a = 1.0,
                                           "plannername"_a = "",
                                           "plannerparameters"_a = "",
                                           DOXY_FN1(SmoothActiveDOFTrajectory)
                                           )
#else
                               .def("SmoothActiveDOFTrajectory",planningutils::pySmoothActiveDOFTrajectory, SmoothActiveDOFTrajectory_overloads(PY_ARGS("trajectory","robot","maxvelmult","maxaccelmult","plannername","plannerparameters") DOXY_FN1(SmoothActiveDOFTrajectory)))
                               .staticmethod("SmoothActiveDOFTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("SmoothAffineTrajectory", planningutils::pySmoothAffineTrajectory,
                                           "trajectory"_a,
                                           "maxvelocities"_a,
                                           "maxaccelerations"_a,
                                           "plannername"_a = "",
                                           "plannerparameters"_a = "",
                                           DOXY_FN1(SmoothAffineTrajectory)
                                           )
#else
                               .def("SmoothAffineTrajectory",planningutils::pySmoothAffineTrajectory, SmoothAffineTrajectory_overloads(PY_ARGS("trajectory","maxvelocities","maxaccelerations","plannername","plannerparameters") DOXY_FN1(SmoothAffineTrajectory)))
                               .staticmethod("SmoothAffineTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("SmoothTrajectory", planningutils::pySmoothTrajectory,
                                           "trajectory"_a,
                                           "maxvelmult"_a = 1.0,
                                           "maxaccelmult"_a = 1.0,
                                           "plannername"_a = "",
                                           "plannerparameters"_a = "",
                                           DOXY_FN1(SmoothTrajectory)
                                           )
#else
                               .def("SmoothTrajectory",planningutils::pySmoothTrajectory, SmoothTrajectory_overloads(PY_ARGS("trajectory","maxvelmult","maxaccelmult","plannername","plannerparameters") DOXY_FN1(SmoothTrajectory)))
                               .staticmethod("SmoothTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("RetimeActiveDOFTrajectory", planningutils::pyRetimeActiveDOFTrajectory,
                                           "trajectory"_a,
                                           "robot"_a,
                                           "hastimestamps"_a = false,
                                           "maxvelmult"_a = 1.0,
                                           "maxaccelmult"_a = 1.0,
                                           "plannername"_a = "",
                                           "plannerparameters"_a = "",
                                           DOXY_FN1(RetimeActiveDOFTrajectory)
                                           )
#else
                               .def("RetimeActiveDOFTrajectory",planningutils::pyRetimeActiveDOFTrajectory, RetimeActiveDOFTrajectory_overloads(PY_ARGS("trajectory","robot","hastimestamps","maxvelmult","maxaccelmult","plannername","plannerparameters") DOXY_FN1(RetimeActiveDOFTrajectory)))
                               .staticmethod("RetimeActiveDOFTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("RetimeAffineTrajectory", planningutils::pyRetimeAffineTrajectory,
                                           "trajectory"_a,
                                           "maxvelocities"_a,
                                           "maxaccelerations"_a,
                                           "hastimestamps"_a = false,
                                           "plannername"_a = "",
                                           "plannerparameters"_a = "",
                                           DOXY_FN1(RetimeAffineTrajectory)
                                           )
#else
                               .def("RetimeAffineTrajectory",planningutils::pyRetimeAffineTrajectory, RetimeAffineTrajectory_overloads(PY_ARGS("trajectory","maxvelocities","maxaccelerations","hastimestamps","plannername","plannerparameters") DOXY_FN1(RetimeAffineTrajectory)))
                               .staticmethod("RetimeAffineTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("RetimeTrajectory", planningutils::pyRetimeTrajectory,
                                           "trajectory"_a,
                                           "hastimestamps"_a = false,
                                           "maxvelmult"_a = 1.0,
                                           "maxaccelmult"_a = 1.0,
                                           "plannername"_a = "",
                                           "plannerparameters"_a = "",
                                           DOXY_FN1(RetimeTrajectory)
                                           )
#else
                               .def("RetimeTrajectory",planningutils::pyRetimeTrajectory, RetimeTrajectory_overloads(PY_ARGS("trajectory","hastimestamps","maxvelmult","maxaccelmult","plannername","plannerparameters") DOXY_FN1(RetimeTrajectory)))
                               .staticmethod("RetimeTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("ExtendWaypoint",planningutils::pyExtendWaypoint, PY_ARGS("index","dofvalues", "dofvelocities", "trajectory", "planner") DOXY_FN1(ExtendWaypoint))
#else
                               .def("ExtendWaypoint",planningutils::pyExtendWaypoint, PY_ARGS("index","dofvalues", "dofvelocities", "trajectory", "planner") DOXY_FN1(ExtendWaypoint))
                               .staticmethod("ExtendWaypoint")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("ExtendActiveDOFWaypoint", planningutils::pyExtendActiveDOFWaypoint,
                                           "index"_a,
                                           "dofvalues"_a,
                                           "dofvelocities"_a,
                                           "trajectory"_a,
                                           "robot"_a,
                                           "maxvelmult"_a = 1.0,
                                           "maxaccelmult"_a = 1.0,
                                           "plannername"_a = "",
                                           DOXY_FN1(ExtendActiveDOFWaypoint)
                                           )
#else
                               .def("ExtendActiveDOFWaypoint",planningutils::pyExtendActiveDOFWaypoint, ExtendActiveDOFWaypoint_overloads(PY_ARGS("index","dofvalues", "dofvelocities", "trajectory", "robot", "maxvelmult", "maxaccelmult", "plannername") DOXY_FN1(ExtendActiveDOFWaypoint)))
                               .staticmethod("ExtendActiveDOFWaypoint")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("InsertActiveDOFWaypointWithRetiming", planningutils::pyInsertActiveDOFWaypointWithRetiming,
                                           "index"_a,
                                           "dofvalues"_a,
                                           "dofvelocities"_a,
                                           "trajectory"_a,
                                           "robot"_a,
                                           "maxvelmult"_a = 1.0,
                                           "maxaccelmult"_a = 1.0,
                                           "plannername"_a = "",
                                           "plannerparameters"_a = "",
                                           DOXY_FN1(InsertActiveDOFWaypointWithRetiming)
                                           )
#else
                               .def("InsertActiveDOFWaypointWithRetiming",planningutils::pyInsertActiveDOFWaypointWithRetiming, InsertActiveDOFWaypointWithRetiming_overloads(PY_ARGS("index","dofvalues", "dofvelocities", "trajectory", "robot", "maxvelmult", "maxaccelmult", "plannername", "plannerparameters") DOXY_FN1(InsertActiveDOFWaypointWithRetiming)))
                               .staticmethod("InsertActiveDOFWaypointWithRetiming")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("InsertWaypointWithSmoothing",planningutils::pyInsertWaypointWithSmoothing,
                                           "index"_a,
                                           "dofvalues"_a,
                                           "dofvelocities"_a,
                                           "trajectory"_a,
                                           "maxvelmult"_a = 1.0,
                                           "maxaccelmult"_a = 1.0,
                                           "plannername"_a = "",
                                           DOXY_FN1(InsertWaypointWithSmoothing)
                                           )
#else
                               .def("InsertWaypointWithSmoothing",planningutils::pyInsertWaypointWithSmoothing, InsertWaypointWithSmoothing_overloads(PY_ARGS("index","dofvalues","dofvelocities","trajectory","maxvelmult","maxaccelmult","plannername") DOXY_FN1(InsertWaypointWithSmoothing)))
                               .staticmethod("InsertWaypointWithSmoothing")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("SegmentTrajectory",planningutils::pySegmentTrajectory, PY_ARGS("trajectory","starttime", "endtime") DOXY_FN1(SegmentTrajectory))
#else
                               .def("SegmentTrajectory",planningutils::pySegmentTrajectory, PY_ARGS("trajectory","starttime", "endtime") DOXY_FN1(SegmentTrajectory))
                               .staticmethod("SegmentTrajectory")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("GetTrajectorySegment",planningutils::pyGetTrajectorySegment, PY_ARGS("trajectory","starttime", "endtime") DOXY_FN1(GetTrajectorySegment))
#else
                               .def("GetTrajectorySegment",planningutils::pyGetTrajectorySegment, PY_ARGS("trajectory","starttime", "endtime") DOXY_FN1(GetTrajectorySegment))
                               .staticmethod("GetTrajectorySegment")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("MergeTrajectories",planningutils::pyMergeTrajectories, PY_ARGS("trajectories") DOXY_FN1(MergeTrajectories))
#else
                               .def("MergeTrajectories",planningutils::pyMergeTrajectories, PY_ARGS("trajectories") DOXY_FN1(MergeTrajectories))
                               .staticmethod("MergeTrajectories")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def_static("GetDHParameters",planningutils::pyGetDHParameters, PY_ARGS("body") DOXY_FN1(GetDHParameters))
#else
                               .def("GetDHParameters",planningutils::pyGetDHParameters, PY_ARGS("body") DOXY_FN1(GetDHParameters))
                               .staticmethod("GetDHParameters")
#endif
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<planningutils::PyDHParameter, OPENRAVE_SHARED_PTR<planningutils::PyDHParameter> >(planningutils, "DHParameter", DOXY_CLASS(planningutils::DHParameter))
        .def(init<>())
        .def(init<object, int, object, dReal, dReal, dReal, dReal>(), "joint"_a, "parentindex"_a, "transform"_a, "d"_a, "a"_a, "theta"_a, "alpha"_a)
#else
        class_<planningutils::PyDHParameter, OPENRAVE_SHARED_PTR<planningutils::PyDHParameter> >("DHParameter", DOXY_CLASS(planningutils::DHParameter))
        .def(init<>())
        .def(init<object, int, object, dReal, dReal, dReal, dReal>(py::args("joint","parentindex","transform","d","a","theta","alpha")))
#endif
        .def_readwrite("joint",&planningutils::PyDHParameter::joint)
        .def_readwrite("transform",&planningutils::PyDHParameter::transform)
        .def_readwrite("d",&planningutils::PyDHParameter::d)
        .def_readwrite("a",&planningutils::PyDHParameter::a)
        .def_readwrite("theta",&planningutils::PyDHParameter::theta)
        .def_readwrite("alpha",&planningutils::PyDHParameter::alpha)
        .def_readwrite("parentindex",&planningutils::PyDHParameter::parentindex)
        .def("__str__",&planningutils::PyDHParameter::__str__)
        .def("__unicode__",&planningutils::PyDHParameter::__unicode__)
        .def("__repr__",&planningutils::PyDHParameter::__repr__)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def(py::pickle(
                 [](const planningutils::PyDHParameter &pyparams) {
                // __getstate__
                return planningutils::DHParameter_pickle_suite::getinitargs(pyparams);
            },
                 [](py::tuple state) {
                // __setstate__
                /* Create a new C++ instance */
                if(state.size() != 7) {
                    RAVELOG_WARN("Invalid state!");
                }
                planningutils::PyDHParameter pyparams;
                pyparams.joint = extract<object>(state[0]); // py::none_()?
                pyparams.parentindex = extract<int>(state[1]);
                pyparams.transform = extract<object>(state[2]);
                pyparams.d = extract<dReal>(state[3]);
                pyparams.a = extract<dReal>(state[4]);
                pyparams.theta = extract<dReal>(state[5]);
                pyparams.alpha = extract<dReal>(state[6]);
                return pyparams;
            }
                 ))
#else
        .def_pickle(planningutils::DHParameter_pickle_suite())
#endif
        ;


#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<planningutils::PyManipulatorIKGoalSampler, planningutils::PyManipulatorIKGoalSamplerPtr >(planningutils, "ManipulatorIKGoalSampler", DOXY_CLASS(planningutils::ManipulatorIKGoalSampler))
        .def(init<object, object, int, int, dReal, bool, int, object>(),
             "manip"_a,
             "parameterizations"_a,
             "nummaxsamples"_a = 20,
             "nummaxtries"_a = 10,
             "jitter"_a = 0,
             "searchfreeparameters"_a = true,
             // In openravepy_iksolver.cpp binds IkFilterOptions::IKFO_CheckEnvCollisions
             // How to use it here?
             "ikfilteroptions"_a = (int) IKFO_CheckEnvCollisions,
             "freevalues"_a = py::none_()
             )
#else
        class_<planningutils::PyManipulatorIKGoalSampler, planningutils::PyManipulatorIKGoalSamplerPtr >("ManipulatorIKGoalSampler", DOXY_CLASS(planningutils::ManipulatorIKGoalSampler), no_init)
        .def(init<object, object, optional<int, int, dReal, bool, int, object> >(py::args("manip", "parameterizations", "nummaxsamples", "nummaxtries", "jitter","searchfreeparameters","ikfilteroptions","freevalues")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("Sample", &planningutils::PyManipulatorIKGoalSampler::Sample,
             "ikreturn"_a = false,
             "releasegil"_a = false,
             DOXY_FN(planningutils::ManipulatorIKGoalSampler, Sample)
             )
#else
        .def("Sample",&planningutils::PyManipulatorIKGoalSampler::Sample, Sample_overloads(PY_ARGS("ikreturn","releasegil") DOXY_FN(planningutils::ManipulatorIKGoalSampler, Sample)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("SampleAll", &planningutils::PyManipulatorIKGoalSampler::SampleAll,
             "maxsamples"_a = 0,
             "maxchecksamples"_a = 0,
             "releasegil"_a = false,
             DOXY_FN(planningutils::ManipulatorIKGoalSampler, SampleAll)
             )
#else
        .def("SampleAll",&planningutils::PyManipulatorIKGoalSampler::SampleAll, SampleAll_overloads(PY_ARGS("maxsamples", "maxchecksamples", "releasegil") DOXY_FN(planningutils::ManipulatorIKGoalSampler, SampleAll)))

#endif
        .def("GetIkParameterizationIndex", &planningutils::PyManipulatorIKGoalSampler::GetIkParameterizationIndex, PY_ARGS("index") DOXY_FN(planningutils::ManipulatorIKGoalSampler, GetIkParameterizationIndex))
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<planningutils::PyActiveDOFTrajectorySmoother, planningutils::PyActiveDOFTrajectorySmootherPtr >(planningutils, "ActiveDOFTrajectorySmoother", DOXY_CLASS(planningutils::ActiveDOFTrajectorySmoother))
        .def(init<PyRobotBasePtr, const std::string&, const std::string&>(), "robot"_a, "plannername"_a, "plannerparameters"_a)
#else
        class_<planningutils::PyActiveDOFTrajectorySmoother, planningutils::PyActiveDOFTrajectorySmootherPtr >("ActiveDOFTrajectorySmoother", DOXY_CLASS(planningutils::ActiveDOFTrajectorySmoother), no_init)
        .def(init<PyRobotBasePtr, const std::string&, const std::string&>(py::args("robot", "plannername", "plannerparameters")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("PlanPath", &planningutils::PyActiveDOFTrajectorySmoother::PlanPath,
             "traj"_a,
             "releasegil"_a = true,
             DOXY_FN(planningutils::ActiveDOFTrajectorySmoother, PlanPath)
             )
#else
        .def("PlanPath",&planningutils::PyActiveDOFTrajectorySmoother::PlanPath,PlanPath_overloads(PY_ARGS("traj","releasegil") DOXY_FN(planningutils::ActiveDOFTrajectorySmoother,PlanPath)))
#endif
        ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<planningutils::PyActiveDOFTrajectoryRetimer, planningutils::PyActiveDOFTrajectoryRetimerPtr >(planningutils, "ActiveDOFTrajectoryRetimer", DOXY_CLASS(planningutils::ActiveDOFTrajectoryRetimer))
        .def(init<PyRobotBasePtr, const std::string&, const std::string&>(), "robot"_a, "plannername"_a, "plannerparameters"_a)
#else
        class_<planningutils::PyActiveDOFTrajectoryRetimer, planningutils::PyActiveDOFTrajectoryRetimerPtr >("ActiveDOFTrajectoryRetimer", DOXY_CLASS(planningutils::ActiveDOFTrajectoryRetimer), no_init)
        .def(init<PyRobotBasePtr, const std::string&, const std::string&>(py::args("robot", "plannername", "plannerparameters")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("PlanPath", &planningutils::PyActiveDOFTrajectoryRetimer::PlanPath,
             "traj"_a,
             "hastimestamps"_a = false,
             "releasegil"_a = true,
             DOXY_FN(planningutils::ActiveDOFTrajectoryRetimer, PlanPath)
             )
#else
        .def("PlanPath",&planningutils::PyActiveDOFTrajectoryRetimer::PlanPath,PlanPath_overloads3(PY_ARGS("traj","hastimestamps", "releasegil") DOXY_FN(planningutils::ActiveDOFTrajectoryRetimer,PlanPath)))
#endif
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<planningutils::PyAffineTrajectoryRetimer, planningutils::PyAffineTrajectoryRetimerPtr >(planningutils, "AffineTrajectoryRetimer", DOXY_CLASS(planningutils::AffineTrajectoryRetimer))
        .def(init<const std::string&, const std::string&>(), "plannername"_a, "plannerparameters"_a)
#else
        class_<planningutils::PyAffineTrajectoryRetimer, planningutils::PyAffineTrajectoryRetimerPtr >("AffineTrajectoryRetimer", DOXY_CLASS(planningutils::AffineTrajectoryRetimer), no_init)
        .def(init<const std::string&, const std::string&>(py::args("plannername", "plannerparameters")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("PlanPath", &planningutils::PyAffineTrajectoryRetimer::PlanPath,
             "traj"_a,
             "maxvelocities"_a,
             "maxaccelerations"_a,
             "hastimestamps"_a = false,
             "releasegil"_a = true,
             DOXY_FN(planningutils::AffineTrajectoryRetimer, PlanPath)
             )
#else
        .def("PlanPath",&planningutils::PyAffineTrajectoryRetimer::PlanPath,PlanPath_overloads2(PY_ARGS("traj","maxvelocities", "maxaccelerations", "hastimestamps", "releasegil") DOXY_FN(planningutils::AffineTrajectoryRetimer,PlanPath)))
#endif
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<planningutils::PyDynamicsCollisionConstraint, planningutils::PyDynamicsCollisionConstraintPtr >(planningutils, "DynamicsCollisionConstraint", DOXY_CLASS(planningutils::DynamicsCollisionConstraint))
        .def(init<object, object, uint32_t>(),
             "plannerparameters"_a,
             "checkbodies"_a,
             "filtermask"_a = 0xffffffff
             )
#else
        class_<planningutils::PyDynamicsCollisionConstraint, planningutils::PyDynamicsCollisionConstraintPtr >("DynamicsCollisionConstraint", DOXY_CLASS(planningutils::DynamicsCollisionConstraint), no_init)
        .def(init<object, object, uint32_t>(py::args("plannerparameters", "checkbodies", "filtermask")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("Check", &planningutils::PyDynamicsCollisionConstraint::Check,
             "q0"_a,
             "q1"_a,
             "dq0"_a,
             "dq1"_a,
             "timeelapsed"_a,
             "interval"_a,
             "options"_a = 0xffff,
             "filterreturn"_a = false,
             DOXY_FN(planningutils::DynamicsCollisionConstraint,Check)
             )
#else
        .def("Check",&planningutils::PyDynamicsCollisionConstraint::Check,Check_overloads(PY_ARGS("q0","q1", "dq0", "dq1", "timeelapsed", "interval", "options", "filterreturn") DOXY_FN(planningutils::DynamicsCollisionConstraint,Check)))
#endif
        .def("GetReport", &planningutils::PyDynamicsCollisionConstraint::GetReport, DOXY_FN(planningutils::DynamicsCollisionConstraint,GetReport))
        .def("SetPlannerParameters", &planningutils::PyDynamicsCollisionConstraint::SetPlannerParameters, PY_ARGS("parameters") DOXY_FN(planningutils::DynamicsCollisionConstraint,SetPlannerParameters))
        .def("SetFilterMask", &planningutils::PyDynamicsCollisionConstraint::SetFilterMask, PY_ARGS("filtermask") DOXY_FN(planningutils::DynamicsCollisionConstraint,SetFilterMask))
        .def("SetPerturbation", &planningutils::PyDynamicsCollisionConstraint::SetPerturbation, PY_ARGS("parameters") DOXY_FN(planningutils::DynamicsCollisionConstraint,SetPerturbation))
        .def("SetTorqueLimitMode", &planningutils::PyDynamicsCollisionConstraint::SetTorqueLimitMode, PY_ARGS("torquelimitmode") DOXY_FN(planningutils::DynamicsCollisionConstraint,SetTorqueLimitMode))
        ;
    }
}

} // end namespace openravepy
