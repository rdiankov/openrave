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

/** \file planningutils.h
    \brief Planning related utilities likes samplers, distance metrics, etc.
*/
#ifndef OPENRAVE_PLANNINGUTIL_H
#define OPENRAVE_PLANNINGUTIL_H

#include <openrave/openrave.h>

namespace OpenRAVE {

namespace planningutils {
        
    /// \brief Jitters the active joint angles of the robot until it escapes collision.
    /// 
    /// Return 0 if jitter failed and robot is in collision, -1 if robot originally not in collision, 1 if jitter succeeded.
    OPENRAVE_API int JitterActiveDOF(RobotBasePtr robot,int nMaxIterations=5000,dReal fRand=0.03f,const PlannerBase::PlannerParameters::NeighStateFn& neighstatefn = PlannerBase::PlannerParameters::NeighStateFn());

    /// \brief Jitters the transform of a body until it escapes collision.
    OPENRAVE_API bool JitterTransform(KinBodyPtr pbody, float fJitter, int nMaxIterations=1000);

    /// \brief Line collision
    class OPENRAVE_API LineCollisionConstraint
    {
    public:
        LineCollisionConstraint();
        bool Check(PlannerBase::PlannerParametersWeakPtr _params, RobotBasePtr robot, const std::vector<dReal>& pQ0, const std::vector<dReal>& pQ1, IntervalType interval, PlannerBase::ConfigurationListPtr pvCheckedConfigurations);
        
    protected:
        std::vector<dReal> _vtempconfig, dQ;
        CollisionReportPtr _report;
    };

    class OPENRAVE_API SimpleDistanceMetric
    {
    public:
        SimpleDistanceMetric(RobotBasePtr robot);
        dReal Eval(const std::vector<dReal>& c0, const std::vector<dReal>& c1);
    protected:
        RobotBasePtr _robot;
        std::vector<dReal> weights2;
    };

    /// \brief samples the neighborhood of a configuration using the configuration space distance metric and sampler.
    class OPENRAVE_API SimpleNeighborhoodSampler
    {
    public:
        SimpleNeighborhoodSampler(SpaceSamplerBasePtr psampler, const boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn);

        bool Sample(std::vector<dReal>& vNewSample, const std::vector<dReal>& vCurSample, dReal fRadius);
        bool Sample(std::vector<dReal>& samples);
    protected:
        SpaceSamplerBasePtr _psampler;
        boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;
    };

    /// \brief Samples numsamples of solutions and each solution to vsolutions
    class OPENRAVE_API ManipulatorIKGoalSampler
    {
    public:
        ManipulatorIKGoalSampler(RobotBase::ManipulatorConstPtr pmanip, const std::list<IkParameterization>& listparameterizations, int nummaxsamples=20, int nummaxtries=10, dReal fsampleprob=0.05f);
        //void SetCheckPathConstraintsFn(const PlannerBase::PlannerParameters::CheckPathConstraintFn& checkfn)
        bool Sample(std::vector<dReal>& vgoal);
        int GetIkParameterizationIndex(int index);

    protected:
        struct SampleInfo
        {
            IkParameterization _ikparam;
            int _numleft;
            SpaceSamplerBasePtr _psampler;
            int _orgindex;
        };
        RobotBasePtr _probot;
        RobotBase::ManipulatorConstPtr _pmanip;
        int _nummaxsamples, _nummaxtries;
        std::list<SampleInfo> _listsamples;
        SpaceSamplerBasePtr _pindexsampler;
        dReal _fsampleprob;
        CollisionReportPtr _report;
        std::vector< std::vector<dReal> > _viksolutions;
        std::list<int> _listreturnedsamples;
        std::vector<dReal> _vfreestart;
    };

} // planningutils
} // OpenRAVE

#endif
