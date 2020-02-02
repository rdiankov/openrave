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
#ifndef OPENRAVEPY_INTERNAL_PLANNERBASE_H
#define OPENRAVEPY_INTERNAL_PLANNERBASE_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class PyPlannerProgress
{
public:
    PyPlannerProgress();
    PyPlannerProgress(const PlannerBase::PlannerProgress& progress);
    std::string __str__();
    int _iteration = 0;
};


class PyPlannerStatus
{
public:
    PyPlannerStatus();
    PyPlannerStatus(const PlannerStatus& status);

    object report = py::none_();
    //std::string _report;
    object description = py::none_();
    object errorOrigin = py::none_();
    object jointValues = py::none_();
    object ikparam = py::none_();
    uint32_t statusCode = 0;
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
        PyPlannerParameters();
        PyPlannerParameters(OPENRAVE_SHARED_PTR<PyPlannerParameters> pyparameters);
        PyPlannerParameters(PlannerBase::PlannerParametersPtr params);
        PyPlannerParameters(PlannerBase::PlannerParametersConstPtr params);
        virtual ~PyPlannerParameters();

        PlannerBase::PlannerParametersConstPtr GetParameters() const;

        PlannerBase::PlannerParametersPtr GetParameters();

        void SetRobotActiveJoints(PyRobotBasePtr robot);

        void SetConfigurationSpecification(PyEnvironmentBasePtr pyenv, PyConfigurationSpecificationPtr pyspec);

        object GetConfigurationSpecification() const;

        void SetExtraParameters(const std::string& s);

        void SetRandomGeneratorSeed(uint32_t seed);

        void SetGoalConfig(object o);

        void SetInitialConfig(object o);

        void SetInitialConfigVelocities(object o);

        void SetGoalConfigVelocities(object o);

        void SetConfigVelocityLimit(object o);

        void SetConfigAccelerationLimit(object o);

        void SetConfigResolution(object o);

        void SetMaxIterations(int nMaxIterations);

        object CheckPathAllConstraints(object oq0, object oq1, object odq0, object odq1, dReal timeelapsed, IntervalType interval, uint32_t options=0xffff, bool filterreturn=false);

        void SetPostProcessing(const std::string& plannername, const std::string& plannerparameters);

        std::string __repr__();
        std::string __str__();
        object __unicode__();
        bool __eq__(OPENRAVE_SHARED_PTR<PyPlannerParameters> p);
        bool __ne__(OPENRAVE_SHARED_PTR<PyPlannerParameters> p);
    };

    typedef OPENRAVE_SHARED_PTR<PyPlannerParameters> PyPlannerParametersPtr;
    typedef OPENRAVE_SHARED_PTR<PyPlannerParameters const> PyPlannerParametersConstPtr;

    PyPlannerBase(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv);
    virtual ~PyPlannerBase();

    bool InitPlan(PyRobotBasePtr pyrobot, PyPlannerParametersPtr pparams, bool releasegil=false);
    bool InitPlan(PyRobotBasePtr pbase, const std::string& params);

    object PlanPath(PyTrajectoryBasePtr pytraj,bool releasegil=true);

    PyPlannerParametersPtr GetParameters() const;

    static PlannerAction _PlanCallback(object fncallback, PyEnvironmentBasePtr pyenv, const PlannerBase::PlannerProgress& progress);

    object RegisterPlanCallback(object fncallback);

    PlannerBasePtr GetPlanner();
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_PLANNERBASE_H
