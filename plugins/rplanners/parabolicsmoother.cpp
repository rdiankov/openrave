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

#include <openrave/planningutils.h>

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
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        isParameters >> *_parameters;
        return _InitPlan();
    }

    bool _InitPlan()
    {
        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 100;
        }
        _puniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        return !!_puniformsampler;
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
        path.reserve(ptraj->GetNumWaypoints());
        vector<dReal> vtrajpoints;
        ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajpoints,_parameters->_configurationspecification);
        ParabolicRamp::Vector q(_parameters->GetDOF());
        for(size_t i = 0; i < ptraj->GetNumWaypoints(); ++i) {
            std::copy(vtrajpoints.begin()+i*_parameters->GetDOF(),vtrajpoints.begin()+(i+1)*_parameters->GetDOF(),q.begin());
            if( path.size() >= 2 ) {
                // check if collinear by taking angle
                const ParabolicRamp::Vector& x0 = path[path.size()-2];
                const ParabolicRamp::Vector& x1 = path[path.size()-1];
                dReal dotproduct=0,x0length2=0,x1length2=0;
                for(size_t i = 0; i < q.size(); ++i) {
                    dReal dx0=x0[i]-q[i];
                    dReal dx1=x1[i]-q[i];
                    dotproduct += dx0*dx1;
                    x0length2 += dx0*dx0;
                    x1length2 += dx1*dx1;
                }
                if( RaveFabs(dotproduct * dotproduct - x0length2*x1length2) < 1e-8 ) {
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
            int numshortcuts=dynamicpath.Shortcut(parameters->_nMaxIterations,checker);

            ConfigurationSpecification oldspec = ptraj->GetConfigurationSpecification();
            ConfigurationSpecification velspec = oldspec.ConvertToVelocitySpecification();
            ConfigurationSpecification newspec = oldspec;
            newspec.AddVelocityGroups(true);

            int timeoffset=-1;
            FOREACH(itgroup,newspec._vgroups) {
                if( itgroup->name == "deltatime" ) {
                    timeoffset = itgroup->offset;
                }
                else if( velspec.FindCompatibleGroup(*itgroup) != velspec._vgroups.end() ) {
                    itgroup->interpolation = "linear";
                }
                else if( oldspec.FindCompatibleGroup(*itgroup) != oldspec._vgroups.end() ) {
                    itgroup->interpolation = "quadratic";
                }
            }
            ptraj->Init(newspec);

            // separate all the acceleration switches into individual points
            vector<dReal> vtrajpoints(newspec.GetDOF());
            ConfigurationSpecification::ConvertData(vtrajpoints.begin(),newspec,dynamicpath.ramps.at(0).x0.begin(),oldspec,1,GetEnv(),true);
            ConfigurationSpecification::ConvertData(vtrajpoints.begin(),newspec,dynamicpath.ramps.at(0).dx0.begin(),velspec,1,GetEnv(),false);
            vtrajpoints.at(timeoffset) = 0;
            ptraj->Insert(ptraj->GetNumWaypoints(),vtrajpoints);
            vector<dReal> vswitchtimes;
            ParabolicRamp::Vector vconfig;
            FOREACHC(itrampnd,dynamicpath.ramps) {
                vswitchtimes.resize(0);
                vswitchtimes.push_back(itrampnd->endTime);
                if( _parameters->_outputaccelchanges ) {
                    FOREACHC(itramp,itrampnd->ramps) {
                        vector<dReal>::iterator it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
                        if( *it != itramp->tswitch1 ) {
                            vswitchtimes.insert(it,itramp->tswitch1);
                        }
                        if( itramp->tswitch1 != itramp->tswitch2 ) {
                            it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
                            if( *it != itramp->tswitch2 ) {
                                vswitchtimes.insert(it,itramp->tswitch2);
                            }
                        }
                    }
                }
                vtrajpoints.resize(newspec.GetDOF()*vswitchtimes.size());
                vector<dReal>::iterator ittargetdata = vtrajpoints.begin();
                dReal prevtime = 0;
                FOREACH(ittime,vswitchtimes) {
                    itrampnd->Evaluate(*ittime,vconfig);
                    ConfigurationSpecification::ConvertData(ittargetdata,newspec,vconfig.begin(),oldspec,1,GetEnv(),true);
                    itrampnd->Derivative(*ittime,vconfig);
                    ConfigurationSpecification::ConvertData(ittargetdata,newspec,vconfig.begin(),velspec,1,GetEnv(),false);
                    *(ittargetdata+timeoffset) = *ittime-prevtime;
                    ittargetdata += newspec.GetDOF();
                    prevtime = *ittime;
                }
                ptraj->Insert(ptraj->GetNumWaypoints(),vtrajpoints);
            }

            BOOST_ASSERT(RaveFabs(dynamicpath.GetTotalTime()-ptraj->GetDuration())<0.001);
            RAVELOG_DEBUG(str(boost::format("after shortcutting %d times: path waypoints=%d, traj waypoints=%d, length=%fs")%numshortcuts%dynamicpath.ramps.size()%ptraj->GetNumWaypoints()%dynamicpath.GetTotalTime()));
        }
        catch (const openrave_exception& ex) {
            RAVELOG_WARN(str(boost::format("parabolic planner failed: %s")%ex.what()));
            ofstream f("failedsmoothing.traj");
            f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            ptraj->serialize(f);
            return PS_Failed;
        }
        RAVELOG_DEBUG(str(boost::format("path optimizing - computation time=%fs\n")%(0.001f*(float)(GetMilliTime()-basetime))));
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
        return _parameters->_checkpathconstraintsfn(a,b,IT_OpenStart,PlannerBase::ConfigurationListPtr());
    }

    virtual dReal Rand()
    {
        std::vector<dReal> vsamples;
        _puniformsampler->SampleSequence(vsamples,1,IT_OpenEnd);
        return vsamples.at(0);
    }

protected:
    TrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _puniformsampler;
};


PlannerBasePtr CreateParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new ParabolicSmoother(penv,sinput));
}

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(ParabolicRamp::ParabolicRamp1D)
BOOST_TYPEOF_REGISTER_TYPE(ParabolicRamp::ParabolicRampND)
#endif
