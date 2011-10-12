// -*- coding: utf-8 -*-
// Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
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
#include "plugindefs.h"
#include "plannerparameters.h"
#include <fstream>

#include "ParabolicPathSmooth/DynamicPath.h"

//namespace ParabolicRamp
//{
//
//static SpaceSamplerBasePtr s_sampler;
//Real Rand();
//}

class ParabolicSmoother : public PlannerBase, public ParabolicRamp::FeasibilityCheckerBase, public ParabolicRamp::RandomSamplerBase
{
public:
    ParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nInterfae to `Indiana University Intelligent Motion Laboratory <http://www.iu.edu/~motion/software.html>`_ parabolic smoothing library (Kris Hauser).\n";
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        _parameters->copy(params);
        _robot = pbase;
        if( !_CheckParameters() ) {
            return false;
        }
        _puniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        return true;
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        isParameters >> *_parameters;
        _robot = pbase;
        return _CheckParameters();
    }

    bool _CheckParameters()
    {
        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 100;
        }
        if(_robot->GetActiveDOF() != _parameters->GetDOF() ) {
            return false;
        }
        if(_robot->GetActiveDOF() != _parameters->GetDOF() ) {
            return false;
        }
        return true;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        BOOST_ASSERT(!!_parameters && !!ptraj);
        if( ptraj->GetNumWaypoints() < 2 ) {
            return PS_Failed;
        }

        uint32_t basetime = GetMilliTime();
        TrajectoryTimingParametersConstPtr parameters = boost::dynamic_pointer_cast<TrajectoryTimingParameters const>(GetParameters());

        vector<ParabolicRamp::Vector> path;
        boost::array<dReal,9> AtA;
        path.reserve(ptraj->GetNumWaypoints());
        vector<dReal> vtrajpoints;
        ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),_parameters->_configurationspecification,vtrajpoints);
        ParabolicRamp::Vector q(_parameters->GetDOF());
        for(size_t i = 0; i < ptraj->GetNumWaypoints(); ++i) {
            std::copy(vtrajpoints.begin()+i*_parameters->GetDOF(),vtrajpoints.begin()+(i+1)*_parameters->GetDOF(),q.begin());
            if( path.size() >= 2 ) {
                // check if collinear by taking determinant
                // compute A^t*A
                for(size_t i = 0; i < 9; ++i) {
                    AtA[i] = 0;
                }
                boost::array<ParabolicRamp::Vector*,3> A = {{&path[path.size()-2],&path[path.size()-1],&q}};
                for(size_t i = 0; i < 3; ++i) {
                    for(size_t j = i; j <3; ++j) {
                        for(size_t k = 0; k < q.size(); ++k) {
                            AtA[3*i+j] += A[i]->at(k) * A[j]->at(k);
                        }
                    }
                }
                dReal det = AtA[0]*AtA[4]*AtA[8] - AtA[0]*AtA[5]*AtA[5] -AtA[1]*AtA[1]*AtA[8] + 2*AtA[1]*AtA[2]*AtA[5] - AtA[2]*AtA[2]*AtA[4];
                if( RaveFabs(det) < 1e-8 ) {
                    path.back() = q;
                    continue;
                }
            }
            path.push_back(q);
        }
        try {
            ParabolicRamp::DynamicPath dynamicpath;
            dynamicpath.Init(parameters->_vConfigVelocityLimit,parameters->_vConfigAccelerationLimit);
            dynamicpath.SetJointLimits(parameters->_vConfigLowerLimit,parameters->_vConfigUpperLimit);
            dynamicpath.SetMilestones(path);   //now the trajectory starts and stops at every milestone
            RAVELOG_DEBUG(str(boost::format("initial path size %d, duration: %f")%path.size()%dynamicpath.GetTotalTime()));
            ParabolicRamp::RampFeasibilityChecker checker(this,parameters->_pointtolerance);
            int res=dynamicpath.Shortcut(parameters->_nMaxIterations,checker);
            RAVELOG_DEBUG(str(boost::format("after shortcutting: duration %f")%dynamicpath.GetTotalTime()));

            ptraj->Init(_parameters->_configurationspecification);
            Trajectory::TPOINT tp;
            for(dReal t = 0; t < dynamicpath.GetTotalTime(); ++t) {
                dynamicpath.Evaluate(t,tp.q);
                dynamicpath.Derivative(t,tp.qdot);
                tp.time = t;
                ptraj->AddPoint(tp);
            }
            tp.q = dynamicpath.ramps.back().x1;
            tp.qdot = dynamicpath.ramps.back().dx1;
            tp.time = dynamicpath.GetTotalTime();
            ptraj->AddPoint(tp);

//            tp.q = dynamicpath.ramps.at(0).x0;
//            tp.qdot = dynamicpath.ramps.at(0).dx0;
//            tp.time = 0;
//            ptraj->AddPoint(tp);
//            FOREACHC(itramp,dynamicpath.ramps) {
//                tp.q = itramp->x1;
//                tp.qdot = itramp->dx1;
//                tp.time += itramp->endTime;
//                ptraj->AddPoint(tp);
//            }
        }
        catch (const openrave_exception& ex) {
            RAVELOG_WARN(str(boost::format("parabolic planner failed: %s")%ex.what()));
            ofstream f("failedsmoothing.traj");
            f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            ptraj->Write(f,0);
            return PS_Failed;
        }
        ptraj->CalcTrajTiming(_robot,TrajectoryBase::LINEAR,false,true);
        RAVELOG_DEBUG(str(boost::format("path optimizing - time=%fs\n")%(0.001f*(float)(GetMilliTime()-basetime))));
        return PS_HasSolution;
    }

    virtual bool ConfigFeasible(const ParabolicRamp::Vector& a)
    {
        _parameters->_setstatefn(a);
        return _parameters->_checkpathconstraintsfn(a,a,IT_OpenStart,PlannerBase::ConfigurationListPtr());
    }

    virtual bool SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b)
    {
        _parameters->_setstatefn(a);
        return _parameters->_checkpathconstraintsfn(a,b,IT_Open,PlannerBase::ConfigurationListPtr());
    }

    virtual dReal Rand()
    {
        std::vector<dReal> vsamples;
        _puniformsampler->SampleSequence(vsamples,1,IT_OpenEnd);
        return vsamples.at(0);
    }

protected:
    RobotBasePtr _robot;
    TrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _puniformsampler;
};


PlannerBasePtr CreateParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new ParabolicSmoother(penv,sinput));
}
