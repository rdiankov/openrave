// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "rplanners.h"

class LinearSmoother : public PlannerBase
{
public:
    LinearSmoother(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nPath optimizer using linear shortcuts assuming robot has no constraints and _neighstatefn is just regular addition. Should be faster than shortcut_linear";
        _linearretimer = RaveCreatePlanner(GetEnv(), "LinearTrajectoryRetimer");
    }
    virtual ~LinearSmoother() {
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        _parameters->copy(params);
        _probot = pbase;
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        isParameters >> *_parameters;
        _probot = pbase;
        return _InitPlan();
    }

    bool _InitPlan()
    {
        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 100;
        }
        if( _parameters->_fStepLength <= 0 ) {
            _parameters->_fStepLength = 0.04;
        }
        _puniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        //_puniformsampler->SetSeed(utils::GetMilliTime()); // use only for testing
        _linearretimer->InitPlan(RobotBasePtr(), _parameters);
        return !!_puniformsampler;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        BOOST_ASSERT(!!_parameters && !!ptraj );
        if( ptraj->GetNumWaypoints() < 2 ) {
            return PS_Failed;
        }

        RobotBase::RobotStateSaverPtr statesaver;
        if( !!_probot ) {
            statesaver.reset(new RobotBase::RobotStateSaver(_probot));
        }

        uint32_t basetime = utils::GetMilliTime();
        PlannerParametersConstPtr parameters = GetParameters();

        // subsample trajectory and add to list
        list< std::pair< vector<dReal>, dReal> > listpath;
        vector<dReal> vtrajdata(parameters->GetDOF());
        ptraj->GetWaypoint(0,vtrajdata,parameters->_configurationspecification);
        dReal totaldist = 0;
        listpath.push_back(make_pair(vtrajdata,0));
        for(size_t i = 1; i < ptraj->GetNumWaypoints(); ++i) {
            ptraj->GetWaypoint(i,vtrajdata,parameters->_configurationspecification);
            dReal dist = parameters->_distmetricfn(listpath.back().first, vtrajdata);
            if( dist > 0 ) {
                listpath.push_back(make_pair(vtrajdata,dist));
                totaldist += dist;
            }
        }

        if( listpath.size() > 1 ) {
            dReal newdist1 = _OptimizePath(listpath, totaldist, parameters->_nMaxIterations*9/10);
            RAVELOG_DEBUG(str(boost::format("path optimizing first stage - dist %f->%f, computation time=%fs\n")%totaldist%newdist1%(0.001f*(float)(utils::GetMilliTime()-basetime))));
            dReal newdist2 = _OptimizePathSingleDOF(listpath, newdist1, parameters->_nMaxIterations*1/10);
            RAVELOG_DEBUG(str(boost::format("path optimizing second stage - dist %f->%f computation time=%fs\n")%newdist1%newdist2%(0.001f*(float)(utils::GetMilliTime()-basetime))));
        }
        else {
            // trajectory contains similar points, so at least add another point and send to the next post-processing stage
            listpath.push_back(make_pair(vtrajdata,0));
        }
        ptraj->Init(parameters->_configurationspecification);
        FOREACH(it, listpath) {
            ptraj->Insert(ptraj->GetNumWaypoints(),it->first);
        }
        if( parameters->_sPostProcessingPlanner.size() == 0 ) {
            // no other planner so at least retime
            PlannerStatus status = _linearretimer->PlanPath(ptraj);
            if( status != PS_HasSolution ) {
                return status;
            }
            return PS_HasSolution;
        }

        // leave to post processing to set timing (like parabolicsmoother)
        return _ProcessPostPlanners(RobotBasePtr(),ptraj);
    }

protected:
    string _DumpTrajectory(TrajectoryBaseConstPtr trajectory)
    {
        string filename = str(boost::format("%s/failedtrajectory%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%1000));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        trajectory->serialize(f);
        RAVELOG_DEBUG(str(boost::format("trajectory dumped to %s")%filename));
        return filename;
    }

    dReal _OptimizePath(list< std::pair< vector<dReal>, dReal> >& listpath, dReal totaldist, int nMaxIterations)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        list< std::pair< vector<dReal>, dReal> >::iterator itstartnode, itstartnodeprev, itendnode, itendnodeprev;
        vector<dReal> vstartvalues(parameters->GetDOF()), vendvalues(parameters->GetDOF());

        int nrejected = 0;
        for(int curiter = 0; curiter < nMaxIterations; ++curiter ) {
            if( nrejected >= 20 ) {
                RAVELOG_VERBOSE("smoothing quitting early\n");
                break;
            }
            dReal fstartdist = max(dReal(0),totaldist-parameters->_fStepLength)*_puniformsampler->SampleSequenceOneReal(IT_OpenEnd);
            dReal fenddist = fstartdist + (totaldist-fstartdist)*_puniformsampler->SampleSequenceOneReal(IT_OpenStart);
            dReal fstartdistdelta=0, fenddistdelta=0;
            dReal fcurdist = 0;
            itstartnodeprev = itstartnode = listpath.begin();
            while(itstartnode != listpath.end() ) {
                if( fstartdist >= fcurdist && fstartdist < fcurdist+itstartnode->second ) {
                    fstartdistdelta = fstartdist-fcurdist;
                    break;
                }
                fcurdist += itstartnode->second;
                itstartnodeprev = itstartnode;
                ++itstartnode;
            }

            itendnodeprev = itstartnodeprev;
            itendnode = itstartnode;
            int numnodes=0;
            while(itendnode != listpath.end() ) {
                if( fenddist >= fcurdist && fenddist < fcurdist+itendnode->second ) {
                    fenddistdelta = fenddist-fcurdist;
                    break;
                }
                fcurdist += itendnode->second;
                itendnodeprev = itendnode;
                ++itendnode;
                ++numnodes;
            }

            if( itstartnode == itendnode ) {
                // choose a line, so ignore
                continue;
            }

            nrejected++;
            BOOST_ASSERT(itstartnode != listpath.end());
            BOOST_ASSERT(itendnode != listpath.end());

            // compute the actual node values
            if( itstartnode == listpath.begin() ) {
                vstartvalues = itstartnode->first;
            }
            else {
                dReal f = fstartdistdelta/itstartnode->second;
                for(size_t i = 0; i < vstartvalues.size(); ++i) {
                    vstartvalues[i] = itstartnode->first.at(i)*f + itstartnodeprev->first.at(i)*(1-f);
                }
            }

            if( itendnode == --listpath.end() ) {
                vendvalues = itendnode->first;
            }
            else {
                dReal f = fenddistdelta/itendnode->second;
                for(size_t i = 0; i < vendvalues.size(); ++i) {
                    vendvalues[i] = itendnode->first.at(i)*f + itendnodeprev->first.at(i)*(1-f);
                }
            }

            dReal fnewsegmentdist = parameters->_distmetricfn(vstartvalues, vendvalues);
            if( fnewsegmentdist > fenddist-fstartdist-0.5*parameters->_fStepLength ) {
                // expected total distance is not that great
                continue;
            }

            // check if the nodes can be connected by a straight line
            if (!SegmentFeasible(vstartvalues, vendvalues, IT_Open)) {
                continue;
            }

            listpath.insert(itstartnode, make_pair(vstartvalues, fstartdistdelta));
            itendnode->second -= fenddistdelta;
            itendnode = listpath.insert(itendnode, make_pair(vendvalues, fnewsegmentdist)); // get new endnode
            listpath.erase(itstartnode, itendnode);
            totaldist += fnewsegmentdist - (fenddist-fstartdist);
            RAVELOG_VERBOSE(str(boost::format("smoother iter %d, totaldist=%f")%curiter%totaldist));
            nrejected = 0;
        }
        // double check the distances
        dReal dist = 0;
        FOREACH(it, listpath) {
            dist += it->second;
        }
        OPENRAVE_ASSERT_OP(RaveFabs(totaldist-dist),<=,1e-7);
        return totaldist;
    }

    dReal _OptimizePathSingleDOF(list< std::pair< vector<dReal>, dReal> >& listpath, dReal totaldist, int nMaxIterations)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        std::vector< std::pair<std::vector<dReal>, dReal> > vpathvalues;
        list< std::pair< vector<dReal>, dReal> >::iterator itstartnode, itstartnodeprev, itendnode, itendnodeprev, itnode;
        size_t numdof = parameters->GetDOF();
        int nrejected = 0;
        for(int curiter = 0; curiter < nMaxIterations; ++curiter ) {
            if( nrejected >= 20 ) {
                RAVELOG_VERBOSE("smoothing quitting early\n");
                break;
            }
            dReal fstartdist = max(dReal(0),totaldist-parameters->_fStepLength)*_puniformsampler->SampleSequenceOneReal(IT_OpenEnd);
            dReal fenddist = fstartdist + (totaldist-fstartdist)*_puniformsampler->SampleSequenceOneReal(IT_OpenStart);
            uint32_t ioptdof = _puniformsampler->SampleSequenceOneUInt32()%uint32_t(numdof); // dof to optimize

            dReal fstartdistdelta=0, fenddistdelta=0;
            dReal fcurdist = 0;
            itstartnodeprev = itstartnode = listpath.begin();
            while(itstartnode != listpath.end() ) {
                if( fstartdist >= fcurdist && fstartdist < fcurdist+itstartnode->second ) {
                    fstartdistdelta = fstartdist-fcurdist;
                    break;
                }
                fcurdist += itstartnode->second;
                itstartnodeprev = itstartnode;
                ++itstartnode;
            }

            itendnodeprev = itstartnodeprev;
            itendnode = itstartnode;
            int numnodes=0;
            while(itendnode != listpath.end() ) {
                if( fenddist >= fcurdist && fenddist < fcurdist+itendnode->second ) {
                    fenddistdelta = fenddist-fcurdist;
                    break;
                }
                fcurdist += itendnode->second;
                itendnodeprev = itendnode;
                ++itendnode;
                ++numnodes;
            }

            if( itstartnode == itendnode ) {
                // choose a line, so ignore
                continue;
            }
            nrejected++;
            BOOST_ASSERT(itstartnode != listpath.end());
            BOOST_ASSERT(itendnode != listpath.end());

            vpathvalues.resize(numnodes+2);

            // compute the actual node values
            if( RaveFabs(fstartdistdelta) <= g_fEpsilonLinear ) {
                vpathvalues.at(0).first = itstartnode->first;
                vpathvalues.at(0).second = 0;
            }
            else {
                std::vector<dReal>& v = vpathvalues.at(0).first;
                v.resize(numdof);
                vpathvalues.at(0).second = 0;
                dReal f = fstartdistdelta/itstartnode->second;
                for(size_t i = 0; i < numdof; ++i) {
                    v[i] = itstartnode->first.at(i)*f + itstartnodeprev->first.at(i)*(1-f);
                }
            }

            if( RaveFabs(fenddistdelta-itendnode->second) <= g_fEpsilonLinear ) {
                vpathvalues.at(numnodes+1) = *itendnode;
            }
            else {
                std::vector<dReal>& v = vpathvalues.at(numnodes+1).first;
                v.resize(numdof);
                vpathvalues.at(numnodes+1).second = fenddistdelta;
                dReal f = fenddistdelta/itendnode->second;
                for(size_t i = 0; i < numdof; ++i) {
                    v[i] = itendnode->first.at(i)*f + itendnodeprev->first.at(i)*(1-f);
                }
            }

            dReal fstartdofvalue = vpathvalues.at(0).first.at(ioptdof), flastdofvalue = vpathvalues.at(numnodes+1).first.at(ioptdof);
            dReal fdelta = (flastdofvalue-fstartdofvalue)/(fenddist-fstartdist);
            bool bsuccess = true;
            fcurdist = 0;
            dReal fnewsegmentdist = 0;
            itnode = itstartnode;
            int pathindex = 0;
            do {
                if( pathindex == 0 ) {
                    fcurdist = itnode->second - fstartdistdelta;
                }
                else {
                    fcurdist += itnode->second;
                }
                vpathvalues.at(pathindex+1).first = itnode->first;
                vpathvalues.at(pathindex+1).first.at(ioptdof) = fstartdofvalue + fcurdist*fdelta;
                dReal fdist = parameters->_distmetricfn(vpathvalues.at(pathindex).first, vpathvalues.at(pathindex+1).first);
                vpathvalues.at(pathindex+1).second = fdist;
                fnewsegmentdist += fdist;
                ++itnode;
                ++pathindex;
            } while(itnode != itendnode);

            // have to process the time on the last node
            dReal fdist = parameters->_distmetricfn(vpathvalues.at(numnodes).first, vpathvalues.at(numnodes+1).first);
            vpathvalues.at(numnodes+1).second = fdist;
            fnewsegmentdist += fdist;

            if( fnewsegmentdist > fenddist-fstartdist-0.5*parameters->_fStepLength ) {
                // expected total distance is not that great
                continue;
            }

            for(size_t i = 0; i+1 < vpathvalues.size(); ++i) {
                IntervalType interval = i+2==vpathvalues.size() ? IT_Open : IT_OpenStart;
                if (!SegmentFeasible(vpathvalues.at(i).first, vpathvalues.at(i+1).first, interval)) {
                    bsuccess = false;
                    break;
                }
            }
            if( !bsuccess ) {
                // rejected due to constraints
                continue;
            }

            // only insert if not at start
            if( RaveFabs(fstartdistdelta) > g_fEpsilonLinear ) {
                vpathvalues.at(0).second = fstartdistdelta;
                listpath.insert(itstartnode, vpathvalues.at(0));
            }

            // replace all the values with the new path
            itnode = itstartnode;
            pathindex = 1;
            do {
                *itnode = vpathvalues.at(pathindex++);
                ++itnode;
            } while(itnode != itendnode);

            // only insert if not at end
            if( RaveFabs(fenddistdelta-itendnode->second) > g_fEpsilonLinear ) {
                listpath.insert(itendnode, vpathvalues.at(numnodes+1));
                itendnode->second -= fenddistdelta;
            }
            totaldist += fnewsegmentdist - (fenddist-fstartdist);
            dReal dist = 0;
            FOREACH(it, listpath) {
                dist += it->second;
            }
            OPENRAVE_ASSERT_OP(RaveFabs(totaldist-dist),<=,1e-7);

            RAVELOG_VERBOSE(str(boost::format("singledof iter %d, totaldist=%f")%curiter%totaldist));
            nrejected = 0;
        }
        // double check the distances
        dReal dist = 0;
        FOREACH(it, listpath) {
            dist += it->second;
        }
        OPENRAVE_ASSERT_OP(RaveFabs(totaldist-dist),<=,1e-7);
        return totaldist;
    }

    /// \brief checks if a segment is feasible using pointtolerance
    inline bool SegmentFeasible(const std::vector<dReal>& a,const std::vector<dReal>& b, IntervalType interval)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        // have to also test with tolerances!
        boost::array<dReal,3> perturbations = {{ 0,_parameters->_pointtolerance,-_parameters->_pointtolerance }}; // note that it is using _parameters in order to avoid casting parameters, which might not work
        std::vector<dReal> anew(a.size()), bnew(b.size());
        FOREACH(itperturbation,perturbations) {
            for(size_t i = 0; i < a.size(); ++i) {
                anew[i] = a[i] + *itperturbation * parameters->_vConfigResolution.at(i);
                bnew[i] = b[i] + *itperturbation * parameters->_vConfigResolution.at(i);
            }
            //parameters->_setstatefn(anew);
            if( !parameters->_checkpathconstraintsfn(anew,bnew,interval,PlannerBase::ConfigurationListPtr()) ) {
                return false;
            }
        }
        return true;
    }

    TrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _puniformsampler;
    RobotBasePtr _probot;
    PlannerBasePtr _linearretimer;
};

PlannerBasePtr CreateLinearSmoother(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new LinearSmoother(penv, sinput));
}
