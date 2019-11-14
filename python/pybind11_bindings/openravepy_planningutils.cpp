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
#include "openravepy_int.h"

#include <openrave/planningutils.h>

namespace openravepy {

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
    return object(openravepy::toPyTrajectory(OpenRAVE::planningutils::ReverseTrajectory(openravepy::GetTrajectory(pytraj)),openravepy::toPyEnvironment(pytraj)));
}

object pyGetReverseTrajectory(PyTrajectoryBasePtr pytraj)
{
    return object(openravepy::toPyTrajectory(OpenRAVE::planningutils::GetReverseTrajectory(openravepy::GetTrajectory(pytraj)),openravepy::toPyEnvironment(pytraj)));
}

void pyVerifyTrajectory(object pyparameters, PyTrajectoryBasePtr pytraj, dReal samplingstep)
{
    OpenRAVE::planningutils::VerifyTrajectory(openravepy::GetPlannerParametersConst(pyparameters), openravepy::GetTrajectory(pytraj),samplingstep);
}

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
        return openravepy::toPyPlannerStatus(_smoother.PlanPath(ptraj));
    }

    OpenRAVE::planningutils::ActiveDOFTrajectorySmoother _smoother;
};

typedef boost::shared_ptr<PyActiveDOFTrajectorySmoother> PyActiveDOFTrajectorySmootherPtr;

object pySmoothAffineTrajectory(PyTrajectoryBasePtr pytraj, object omaxvelocities, object omaxaccelerations, const std::string& plannername="", const std::string& plannerparameters="")
{
    return openravepy::toPyPlannerStatus(OpenRAVE::planningutils::SmoothAffineTrajectory(openravepy::GetTrajectory(pytraj),ExtractArray<dReal>(omaxvelocities), ExtractArray<dReal>(omaxaccelerations),plannername,plannerparameters));
}

object pySmoothTrajectory(PyTrajectoryBasePtr pytraj, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return openravepy::toPyPlannerStatus(OpenRAVE::planningutils::SmoothTrajectory(openravepy::GetTrajectory(pytraj),fmaxvelmult,fmaxaccelmult,plannername,plannerparameters));
}

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
        return openravepy::toPyPlannerStatus(_retimer.PlanPath(ptraj, hastimestamps));
    }

    OpenRAVE::planningutils::ActiveDOFTrajectoryRetimer _retimer;
};

typedef boost::shared_ptr<PyActiveDOFTrajectoryRetimer> PyActiveDOFTrajectoryRetimerPtr;

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
        return openravepy::toPyPlannerStatus(_retimer.PlanPath(ptraj,vmaxvelocities, vmaxaccelerations, hastimestamps));
    }

    OpenRAVE::planningutils::AffineTrajectoryRetimer _retimer;
};

typedef boost::shared_ptr<PyAffineTrajectoryRetimer> PyAffineTrajectoryRetimerPtr;

class PyDynamicsCollisionConstraint
{
public:
    PyDynamicsCollisionConstraint(object oparameters, object olistCheckBodies, uint32_t filtermask=0xffffffff)
    {
        PlannerBase::PlannerParametersConstPtr parameters = openravepy::GetPlannerParametersConst(oparameters);
        std::list<KinBodyPtr> listCheckBodies;
        for(int i = 0; i < len(olistCheckBodies); ++i) {
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
            boost::python::dict ofilterreturn;
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
            return object(_pconstraints->Check(q0, q1, dq0, dq1, timeelapsed, interval, options));
        }
    }

    object GetReport() const {
        if( !_pconstraints->GetReport() ) {
            return object();
        }
        return object(openravepy::toPyCollisionReport(_pconstraints->GetReport(), _pyenv));
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
        _pconstraints->SetTorqueLimitMode(torquelimitmode);
    }


    PyEnvironmentBasePtr _pyenv;
    OpenRAVE::planningutils::DynamicsCollisionConstraintPtr _pconstraints;
};

typedef boost::shared_ptr<PyDynamicsCollisionConstraint> PyDynamicsCollisionConstraintPtr;

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
    return object(openravepy::toPyTrajectory(OpenRAVE::planningutils::GetTrajectorySegment(openravepy::GetTrajectory(pytraj), starttime, endtime), pyenv));
}

object pyMergeTrajectories(object pytrajectories)
{
    std::list<TrajectoryBaseConstPtr> listtrajectories;
    PyEnvironmentBasePtr pyenv;
    for(int i = 0; i < len(pytrajectories); ++i) {
        extract<PyTrajectoryBasePtr> epytrajectory(pytrajectories[i]);
        PyTrajectoryBasePtr pytrajectory = (PyTrajectoryBasePtr)epytrajectory;
        if( !pyenv ) {
            pyenv = openravepy::toPyEnvironment(pytrajectory);
        }
        else {
            BOOST_ASSERT(pyenv == openravepy::toPyEnvironment(pytrajectory));
        }
        listtrajectories.push_back(openravepy::GetTrajectory(pytrajectory));
    }
    return object(openravepy::toPyTrajectory(OpenRAVE::planningutils::MergeTrajectories(listtrajectories),pyenv));
}

class PyDHParameter
{
public:
    PyDHParameter() : parentindex(-1), transform(ReturnTransform(Transform())), d(0), a(0), theta(0), alpha(0) {
    }
    PyDHParameter(const OpenRAVE::planningutils::DHParameter& p, PyEnvironmentBasePtr pyenv) : joint(toPyKinBodyJoint(boost::const_pointer_cast<KinBody::Joint>(p.joint), pyenv)), parentindex(p.parentindex), transform(ReturnTransform(p.transform)), d(p.d), a(p.a), theta(p.theta), alpha(p.alpha) {
    }
    PyDHParameter(object joint, int parentindex, object transform, dReal d, dReal a, dReal theta, dReal alpha) : joint(joint), parentindex(parentindex), transform(transform), d(d), a(a), theta(theta), alpha(alpha) {
    }
    virtual ~PyDHParameter() {
    }
    string __repr__() {
        return boost::str(boost::format("<DHParameter(joint=%s, parentindex=%d, d=%f, a=%f, theta=%f, alpha=%f)>")%reprPyKinBodyJoint(joint)%parentindex%d%a%theta%alpha);
    }
    string __str__() {
        TransformMatrix tm = ExtractTransformMatrix(transform);
        return boost::str(boost::format("<joint %s, transform [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f]], parentindex %d>")%strPyKinBodyJoint(joint)%tm.m[0]%tm.m[1]%tm.m[2]%tm.trans[0]%tm.m[4]%tm.m[5]%tm.m[6]%tm.trans[1]%tm.m[8]%tm.m[9]%tm.m[10]%tm.trans[2]%parentindex);
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    object joint;
    int parentindex;
    object transform;
    dReal d, a, theta, alpha;
};

object toPyDHParameter(const OpenRAVE::planningutils::DHParameter& p, PyEnvironmentBasePtr pyenv)
{
    return object(boost::shared_ptr<PyDHParameter>(new PyDHParameter(p,pyenv)));
}

class DHParameter_pickle_suite : public pickle_suite
{
public:
    static boost::python::tuple getinitargs(const PyDHParameter& p)
    {
        return boost::python::make_tuple(object(), p.parentindex, p.transform, p.d, p.a, p.theta, p.alpha);
    }
};

boost::python::list pyGetDHParameters(PyKinBodyPtr pybody)
{
    boost::python::list oparameters;
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
    PyManipulatorIKGoalSampler(object pymanip, object oparameterizations, int nummaxsamples=20, int nummaxtries=10, dReal jitter=0, bool searchfreeparameters=true, uint32_t ikfilteroptions = IKFO_CheckEnvCollisions, object freevalues = object()) {
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
        vector<dReal> vfreevalues = ExtractArray<dReal>(freevalues);
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
        return object();
    }

    object SampleAll(int maxsamples=0, int maxchecksamples=0, bool releasegil = false)
    {
        boost::python::list oreturns;
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

typedef boost::shared_ptr<PyManipulatorIKGoalSampler> PyManipulatorIKGoalSamplerPtr;


} // end namespace planningutils
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

void InitPlanningUtils()
{
    {
        scope x = class_<planningutils::PyStaticClass>("planningutils")
                  .def("JitterTransform",planningutils::pyJitterTransform,JitterTransform_overloads(args("body","jitter","maxiterations"),DOXY_FN1(JitterTransform)))
                  .staticmethod("JitterTransform")
                  .def("JitterCurrentConfiguration",planningutils::pyJitterCurrentConfiguration,JitterCurrentConfiguration_overloads(args("plannerparameters","maxiterations", "jitter", "perturbation"),DOXY_FN1(JitterCurrentConfiguration)))
                  .staticmethod("JitterCurrentConfiguration")
                  .def("ConvertTrajectorySpecification",planningutils::pyConvertTrajectorySpecification,args("trajectory","spec"),DOXY_FN1(ConvertTrajectorySpecification))
                  .staticmethod("ConvertTrajectorySpecification")
                  .def("ComputeTrajectoryDerivatives",planningutils::pyComputeTrajectoryDerivatives,args("trajectory","maxderiv"),DOXY_FN1(ComputeTrajectoryDerivatives))
                  .staticmethod("ComputeTrajectoryDerivatives")
                  .def("ReverseTrajectory",planningutils::pyReverseTrajectory,args("trajectory"),DOXY_FN1(ReverseTrajectory))
                  .staticmethod("ReverseTrajectory")
                  .def("VerifyTrajectory",planningutils::pyVerifyTrajectory,args("parameters","trajectory","samplingstep"),DOXY_FN1(VerifyTrajectory))
                  .staticmethod("VerifyTrajectory")
                  .def("SmoothActiveDOFTrajectory",planningutils::pySmoothActiveDOFTrajectory, SmoothActiveDOFTrajectory_overloads(args("trajectory","robot","maxvelmult","maxaccelmult","plannername","plannerparameters"),DOXY_FN1(SmoothActiveDOFTrajectory)))
                  .staticmethod("SmoothActiveDOFTrajectory")
                  .def("SmoothAffineTrajectory",planningutils::pySmoothAffineTrajectory, SmoothAffineTrajectory_overloads(args("trajectory","maxvelocities","maxaccelerations","plannername","plannerparameters"),DOXY_FN1(SmoothAffineTrajectory)))
                  .staticmethod("SmoothAffineTrajectory")
                  .def("SmoothTrajectory",planningutils::pySmoothTrajectory, SmoothTrajectory_overloads(args("trajectory","maxvelmult","maxaccelmult","plannername","plannerparameters"),DOXY_FN1(SmoothTrajectory)))
                  .staticmethod("SmoothTrajectory")
                  .def("RetimeActiveDOFTrajectory",planningutils::pyRetimeActiveDOFTrajectory, RetimeActiveDOFTrajectory_overloads(args("trajectory","robot","hastimestamps","maxvelmult","maxaccelmult","plannername","plannerparameters"),DOXY_FN1(RetimeActiveDOFTrajectory)))
                  .staticmethod("RetimeActiveDOFTrajectory")
                  .def("RetimeAffineTrajectory",planningutils::pyRetimeAffineTrajectory, RetimeAffineTrajectory_overloads(args("trajectory","maxvelocities","maxaccelerations","hastimestamps","plannername","plannerparameters"),DOXY_FN1(RetimeAffineTrajectory)))
                  .staticmethod("RetimeAffineTrajectory")
                  .def("RetimeTrajectory",planningutils::pyRetimeTrajectory, RetimeTrajectory_overloads(args("trajectory","hastimestamps","maxvelmult","maxaccelmult","plannername","plannerparameters"),DOXY_FN1(RetimeTrajectory)))
                  .staticmethod("RetimeTrajectory")
                  .def("ExtendWaypoint",planningutils::pyExtendWaypoint, args("index","dofvalues", "dofvelocities", "trajectory", "planner"),DOXY_FN1(ExtendWaypoint))
                  .staticmethod("ExtendWaypoint")
                  .def("ExtendActiveDOFWaypoint",planningutils::pyExtendActiveDOFWaypoint, ExtendActiveDOFWaypoint_overloads(args("index","dofvalues", "dofvelocities", "trajectory", "robot", "maxvelmult", "maxaccelmult", "plannername"),DOXY_FN1(ExtendActiveDOFWaypoint)))
                  .staticmethod("ExtendActiveDOFWaypoint")
                  .def("InsertActiveDOFWaypointWithRetiming",planningutils::pyInsertActiveDOFWaypointWithRetiming, InsertActiveDOFWaypointWithRetiming_overloads(args("index","dofvalues", "dofvelocities", "trajectory", "robot", "maxvelmult", "maxaccelmult", "plannername", "plannerparameters"),DOXY_FN1(InsertActiveDOFWaypointWithRetiming)))
                  .staticmethod("InsertActiveDOFWaypointWithRetiming")
                  .def("InsertWaypointWithSmoothing",planningutils::pyInsertWaypointWithSmoothing, InsertWaypointWithSmoothing_overloads(args("index","dofvalues","dofvelocities","trajectory","maxvelmult","maxaccelmult","plannername"),DOXY_FN1(InsertWaypointWithSmoothing)))
                  .staticmethod("InsertWaypointWithSmoothing")
                  .def("SegmentTrajectory",planningutils::pySegmentTrajectory,args("trajectory","starttime", "endtime"),DOXY_FN1(SegmentTrajectory))
                  .staticmethod("SegmentTrajectory")
                  .def("GetTrajectorySegment",planningutils::pyGetTrajectorySegment,args("trajectory","starttime", "endtime"),DOXY_FN1(GetTrajectorySegment))
                  .staticmethod("GetTrajectorySegment")
                  .def("MergeTrajectories",planningutils::pyMergeTrajectories,args("trajectories"),DOXY_FN1(MergeTrajectories))
                  .staticmethod("MergeTrajectories")
                  .def("GetDHParameters",planningutils::pyGetDHParameters,args("body"),DOXY_FN1(GetDHParameters))
                  .staticmethod("GetDHParameters")
        ;

        class_<planningutils::PyDHParameter, boost::shared_ptr<planningutils::PyDHParameter> >("DHParameter", DOXY_CLASS(planningutils::DHParameter))
        .def(init<>())
        .def(init<object, int, object, dReal, dReal, dReal, dReal>(args("joint","parentindex","transform","d","a","theta","alpha")))
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
        .def_pickle(planningutils::DHParameter_pickle_suite())
        ;


        class_<planningutils::PyManipulatorIKGoalSampler, planningutils::PyManipulatorIKGoalSamplerPtr >("ManipulatorIKGoalSampler", DOXY_CLASS(planningutils::ManipulatorIKGoalSampler), no_init)
        .def(init<object, object, optional<int, int, dReal, bool, int, object> >(args("manip", "parameterizations", "nummaxsamples", "nummaxtries", "jitter","searchfreeparameters","ikfilteroptions","freevalues")))
        .def("Sample",&planningutils::PyManipulatorIKGoalSampler::Sample, Sample_overloads(args("ikreturn","releasegil"),DOXY_FN(planningutils::ManipulatorIKGoalSampler, Sample)))
        .def("SampleAll",&planningutils::PyManipulatorIKGoalSampler::SampleAll, SampleAll_overloads(args("maxsamples", "maxchecksamples", "releasegil"),DOXY_FN(planningutils::ManipulatorIKGoalSampler, SampleAll)))
        .def("GetIkParameterizationIndex", &planningutils::PyManipulatorIKGoalSampler::GetIkParameterizationIndex, args("index"), DOXY_FN(planningutils::ManipulatorIKGoalSampler, GetIkParameterizationIndex))
        ;

        class_<planningutils::PyActiveDOFTrajectorySmoother, planningutils::PyActiveDOFTrajectorySmootherPtr >("ActiveDOFTrajectorySmoother", DOXY_CLASS(planningutils::ActiveDOFTrajectorySmoother), no_init)
        .def(init<PyRobotBasePtr, const std::string&, const std::string&>(args("robot", "plannername", "plannerparameters")))
        .def("PlanPath",&planningutils::PyActiveDOFTrajectorySmoother::PlanPath,PlanPath_overloads(args("traj","releasegil"), DOXY_FN(planningutils::ActiveDOFTrajectorySmoother,PlanPath)))
        ;

        class_<planningutils::PyActiveDOFTrajectoryRetimer, planningutils::PyActiveDOFTrajectoryRetimerPtr >("ActiveDOFTrajectoryRetimer", DOXY_CLASS(planningutils::ActiveDOFTrajectoryRetimer), no_init)
        .def(init<PyRobotBasePtr, const std::string&, const std::string&>(args("robot", "hastimestamps", "plannername", "plannerparameters")))
        .def("PlanPath",&planningutils::PyActiveDOFTrajectoryRetimer::PlanPath,PlanPath_overloads3(args("traj","hastimestamps", "releasegil"), DOXY_FN(planningutils::ActiveDOFTrajectoryRetimer,PlanPath)))
        ;

        class_<planningutils::PyAffineTrajectoryRetimer, planningutils::PyAffineTrajectoryRetimerPtr >("AffineTrajectoryRetimer", DOXY_CLASS(planningutils::AffineTrajectoryRetimer), no_init)
        .def(init<const std::string&, const std::string&>(args("plannername", "plannerparameters")))
        .def("PlanPath",&planningutils::PyAffineTrajectoryRetimer::PlanPath,PlanPath_overloads2(args("traj","maxvelocities", "maxaccelerations", "hastimestamps", "releasegil"), DOXY_FN(planningutils::AffineTrajectoryRetimer,PlanPath)))
        ;

        class_<planningutils::PyDynamicsCollisionConstraint, planningutils::PyDynamicsCollisionConstraintPtr >("DynamicsCollisionConstraint", DOXY_CLASS(planningutils::DynamicsCollisionConstraint), no_init)
        .def(init<object, object, uint32_t>(args("plannerparameters", "checkbodies", "filtermask")))
        .def("Check",&planningutils::PyDynamicsCollisionConstraint::Check,Check_overloads(args("q0","q1", "dq0", "dq1", "timeelapsed", "interval", "options", "filterreturn"), DOXY_FN(planningutils::DynamicsCollisionConstraint,Check)))
        .def("GetReport", &planningutils::PyDynamicsCollisionConstraint::GetReport, DOXY_FN(planningutils::DynamicsCollisionConstraint,GetReport))
        .def("SetPlannerParameters", &planningutils::PyDynamicsCollisionConstraint::SetPlannerParameters, args("parameters"), DOXY_FN(planningutils::DynamicsCollisionConstraint,SetPlannerParameters))
        .def("SetFilterMask", &planningutils::PyDynamicsCollisionConstraint::SetFilterMask, args("filtermask"), DOXY_FN(planningutils::DynamicsCollisionConstraint,SetFilterMask))
        .def("SetPerturbation", &planningutils::PyDynamicsCollisionConstraint::SetPerturbation, args("parameters"), DOXY_FN(planningutils::DynamicsCollisionConstraint,SetPerturbation))
        .def("SetTorqueLimitMode", &planningutils::PyDynamicsCollisionConstraint::SetTorqueLimitMode, args("torquelimitmode"), DOXY_FN(planningutils::DynamicsCollisionConstraint,SetTorqueLimitMode))
        ;
    }
}

} // end namespace openravepy
