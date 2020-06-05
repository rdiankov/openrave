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
#include "openraveplugindefs.h"

class LinearSmoother : public PlannerBase
{
public:
    LinearSmoother(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nPath optimizer using linear shortcuts assuming robot has no constraints and _neighstatefn is just regular addition. Should be faster than shortcut_linear.\n\nIf passing 0 or 1 to the constructor, can enable/disable single-dof smoothing.";
        _linearretimer = RaveCreatePlanner(GetEnv(), "LinearTrajectoryRetimer");
        _nUseSingleDOFSmoothing = 1;
        sinput >> _nUseSingleDOFSmoothing;
        RAVELOG_INFO_FORMAT("env=%d, _nUseSingleDOFSmoothing=%d", GetEnv()->GetId()%_nUseSingleDOFSmoothing);
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
        if( !!_puniformsampler ) {
            _puniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        }
        _linearretimer->InitPlan(RobotBasePtr(), _parameters);

        _vConfigVelocityLimitInv.resize(_parameters->_vConfigVelocityLimit.size());
        for(int i = 0; i < (int)_vConfigVelocityLimitInv.size(); ++i) {
            _vConfigVelocityLimitInv[i] = 1/_parameters->_vConfigVelocityLimit[i];
        }
        return !!_puniformsampler;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    dReal _ComputeExpectedVelocity(const std::vector<dReal>& vstart, const std::vector<dReal>& vend) {
        std::vector<dReal> vdiff = vstart;
        _parameters->_diffstatefn(vdiff, vend);
        dReal fmaxtime = 0;
        for(int i = 0; i < (int)_vConfigVelocityLimitInv.size(); ++i) {
            dReal f = RaveFabs(vdiff[i])*_vConfigVelocityLimitInv[i];
            if( f > fmaxtime ) {
                fmaxtime = f;
            }
        }
        return fmaxtime;
    }

    dReal _ComputeExpectedVelocityGroup(const std::vector<dReal>& vstart, const std::vector<dReal>& vend, int igroup) {
        std::vector<dReal> vdiff = vstart;
        _parameters->_diffstatefn(vdiff, vend);
        int iGroupStartIndex = (int)_vConfigVelocityLimitInv.size()/2;
        dReal fmaxtime = 0;
        for(int i = 0; i < (int)_vConfigVelocityLimitInv.size(); ++i) {
            if( (i < iGroupStartIndex) ^ (igroup>0) ) {
                dReal f = RaveFabs(vdiff[i])*_vConfigVelocityLimitInv[i];
                if( f > fmaxtime ) {
                    fmaxtime = f;
                }
            }
        }
        return fmaxtime;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        BOOST_ASSERT(!!_parameters && !!ptraj );
        if( ptraj->GetNumWaypoints() < 2 ) {
            return OPENRAVE_PLANNER_STATUS(PS_Failed);
        }

        RobotBase::RobotStateSaverPtr statesaver;
        if( !!_probot ) {
            statesaver.reset(new RobotBase::RobotStateSaver(_probot));
        }

        uint32_t basetime = utils::GetMilliTime();
        PlannerParametersConstPtr parameters = GetParameters();

        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            _DumpTrajectory(ptraj);
        }
        vector<dReal> vtrajdata(parameters->GetDOF());
        ptraj->GetWaypoint(0,vtrajdata,parameters->_configurationspecification);

        // subsample trajectory and add to list
        if( ptraj->GetNumWaypoints() > 1 ) {
            list< std::pair< vector<dReal>, dReal> > listpath;
            dReal totaldist = 0;

            bool isBranching = false;
            if(!!_probot && _nUseSingleDOFSmoothing == 1) {
                std::set<KinBody::LinkConstPtr> setJoints;
                FOREACHC(it, _probot->GetActiveDOFIndices()) {
                    KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);
                    bool canInsert = setJoints.insert(pjoint->GetHierarchyParentLink()).second;
                    if(!canInsert) {
                        isBranching = true;
                        break;
                    }
                }
            }

            if( _nUseSingleDOFSmoothing == 3 or (isBranching and _nUseSingleDOFSmoothing == 1)) {
                uint32_t basetime1 = utils::GetMilliTime();
                list< vector<dReal> > listsimplepath;
                for(size_t i = 0; i < ptraj->GetNumWaypoints(); ++i) {
                    ptraj->GetWaypoint(i,vtrajdata,parameters->_configurationspecification);
                    listsimplepath.push_back(vtrajdata);
                }

                dReal totalshiftdist = _ComputePathDurationOnVelocity(listsimplepath);
                dReal newdist1 = _OptimizePathSingleGroupShift(listsimplepath, totalshiftdist, parameters->_nMaxIterations*10);
                if( newdist1 < 0 ) {
                    return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
                }
                RAVELOG_DEBUG_FORMAT("env=%d, path optimizing shift smoothing - dist %f->%f computation time=%fs", GetEnv()->GetId()%totalshiftdist%newdist1%(0.001f*(float)(utils::GetMilliTime()-basetime1)));

//                if( listsimplepath.size() <= 1 ) {
//                    // trajectory contains similar points, so at least add another point and send to the next post-processing stage
//                    listsimplepath.push_back(vtrajdata);
//                }

//                ptraj->Init(parameters->_configurationspecification);
//                FOREACH(it, listsimplepath) {
//                    ptraj->Insert(ptraj->GetNumWaypoints(),*it);
//                }

                totaldist = 0;
                FOREACH(itvalues, listsimplepath) {
                    dReal dist = 0;
                    if( listpath.size() > 0 ) {
                        dist = parameters->_distmetricfn(listpath.back().first, *itvalues);
                        if( dist > 0 ) {
                            listpath.emplace_back(*itvalues, dist);
                            totaldist += dist;
                        }
                    }
                    else {
                        vtrajdata = *itvalues;
                        listpath.emplace_back(*itvalues, 0);
                    }
                }
            }
            else {
                listpath.emplace_back(vtrajdata, 0);
                for(size_t i = 1; i < ptraj->GetNumWaypoints(); ++i) {
                    ptraj->GetWaypoint(i,vtrajdata,parameters->_configurationspecification);
                    dReal dist;
                    dist = parameters->_distmetricfn(listpath.back().first, vtrajdata);
                    if( dist > 0 ) {
                        listpath.emplace_back(vtrajdata, dist);
                        totaldist += dist;
                    }
                }
            }

            {

                if( _nUseSingleDOFSmoothing == 1 ) {
                    dReal newdist1 = _OptimizePath(listpath, totaldist, parameters->_nMaxIterations*8/10);
                    if( newdist1 < 0 ) {
                        return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
                    }
                    RAVELOG_DEBUG_FORMAT("env=%d, path optimizing first stage - dist %f->%f, computation time=%fs, num=%d", GetEnv()->GetId()%totaldist%newdist1%(0.001f*(float)(utils::GetMilliTime()-basetime))%listpath.size());
                    uint32_t basetime2 = utils::GetMilliTime();
                    dReal newdist2 = _OptimizePathSingleDOF(listpath, newdist1, parameters->_nMaxIterations*2/10);
                    if( newdist2 < 0 ) {
                        return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
                    }
                    RAVELOG_DEBUG_FORMAT("env=%d, path optimizing second stage - dist %f->%f computation time=%fs, num=%d", GetEnv()->GetId()%newdist1%newdist2%(0.001f*(float)(utils::GetMilliTime()-basetime2))%listpath.size());
                }
                else if( _nUseSingleDOFSmoothing == 2 ) {
                    uint32_t basetime1 = utils::GetMilliTime();
                    int nIterationGroup = parameters->_nMaxIterations/100;
                    int nCurIterations = 0;
                    while(nCurIterations < parameters->_nMaxIterations) {
                        dReal newdist1 = _OptimizePathSingleGroup(listpath, totaldist, nIterationGroup);
                        if( newdist1 < 0 ) {
                            return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
                        }
                        dReal newdist2 = _OptimizePath(listpath,  newdist1, nIterationGroup);
                        if( newdist2 < 0 ) {
                            return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
                        }
                        RAVELOG_DEBUG_FORMAT("env=%d, path optimizing second stage - dist %f->%f computation time=%fs", GetEnv()->GetId()%totaldist%newdist2%(0.001f*(float)(utils::GetMilliTime()-basetime1)));
                        totaldist = newdist2;
                        nCurIterations += 2*nIterationGroup;
                    }
                }
                else {
                    dReal newdist1 = _OptimizePath(listpath, totaldist, parameters->_nMaxIterations);
                    RAVELOG_DEBUG_FORMAT("env=%d, path optimizing stage - dist %f->%f, computation time=%fs", GetEnv()->GetId()%totaldist%newdist1%(0.001f*(float)(utils::GetMilliTime()-basetime)));
                }

                if( listpath.size() <= 1 ) {
                    // trajectory contains similar points, so at least add another point and send to the next post-processing stage
                    listpath.emplace_back(vtrajdata, 0);
                }
                ptraj->Init(parameters->_configurationspecification);
                FOREACH(it, listpath) {
                    ptraj->Insert(ptraj->GetNumWaypoints(),it->first);
                }
            }
        }
        else {
            // trajectory contains similar points, so at least add another point and send to the next post-processing stage
            ptraj->Init(parameters->_configurationspecification);
            ptraj->Insert(0,vtrajdata);
        }

        if( parameters->_sPostProcessingPlanner.size() == 0 ) {
            // no other planner so at least retime
            PlannerStatus status = _linearretimer->PlanPath(ptraj);
            if( status.GetStatusCode() != PS_HasSolution ) {
                return status;
            }
            return OPENRAVE_PLANNER_STATUS(PS_HasSolution);
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
        RAVELOG_DEBUG_FORMAT("env=%d, trajectory dumped to %s", GetEnv()->GetId()%filename);
        return filename;
    }

    dReal _OptimizePath(list< std::pair< vector<dReal>, dReal> >& listpath, dReal totaldist, int nMaxIterations)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        list< std::pair< vector<dReal>, dReal> >::iterator itstartnode, itstartnodeprev, itendnode, itendnodeprev;
        vector<dReal> vstartvalues(parameters->GetDOF()), vendvalues(parameters->GetDOF());

        PlannerProgress progress;

        int nrejected = 0;
        for(int curiter = 0; curiter < nMaxIterations; ++curiter ) {
            if( nrejected >= 20 ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, smoothing quitting early", GetEnv()->GetId());
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

            progress._iteration=curiter;
            if( _CallCallbacks(progress) == PA_Interrupt ) {
                return -1;
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
            RAVELOG_VERBOSE_FORMAT("env=%d, smoother iter %d, totaldist=%f", GetEnv()->GetId()%curiter%totaldist);
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
        PlannerProgress progress;
        for(int curiter = 0; curiter < nMaxIterations; ++curiter ) {
            if( nrejected >= 20 ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, smoothing quitting early", GetEnv()->GetId());
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

            progress._iteration=curiter;
            if( _CallCallbacks(progress) == PA_Interrupt ) {
                return -1;
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

            progress._iteration=curiter;
            if( _CallCallbacks(progress) == PA_Interrupt ) {
                return -1;
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

            RAVELOG_VERBOSE_FORMAT("env=%d, singledof iter %d, totaldist=%f", GetEnv()->GetId()%curiter%totaldist);
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

    dReal _OptimizePathSingleGroup(list< std::pair< vector<dReal>, dReal> >& listpath, dReal totaldist, int nMaxIterations)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        std::vector< std::pair<std::vector<dReal>, dReal> > vpathvalues;
        list< std::pair< vector<dReal>, dReal> >::iterator itstartnode, itstartnodeprev, itendnode, itendnodeprev, itnode;
        size_t numdof = parameters->GetDOF();
        int iGroupStartIndex = numdof/2;
        int nrejected = 0;
        PlannerProgress progress;
        for(int curiter = 0; curiter < nMaxIterations; ++curiter ) {
            if( nrejected >= 20 ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, smoothing quitting early", GetEnv()->GetId());
                break;
            }
            dReal fstartdist = max(dReal(0),totaldist-parameters->_fStepLength)*_puniformsampler->SampleSequenceOneReal(IT_OpenEnd);
            dReal fenddist = fstartdist + (totaldist-fstartdist)*_puniformsampler->SampleSequenceOneReal(IT_OpenStart);
            uint32_t ioptgroup = _puniformsampler->SampleSequenceOneUInt32()%uint32_t(2); // group to optimize

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

            progress._iteration=curiter;
            if( _CallCallbacks(progress) == PA_Interrupt ) {
                return -1;
            }

            std::vector<dReal> vstartdofvalues, vlastdofvalues, vdelta;
            if( ioptgroup == 0 ) {
                vstartdofvalues.insert(vstartdofvalues.end(), vpathvalues.at(0).first.begin(), vpathvalues.at(0).first.begin()+iGroupStartIndex);
                vlastdofvalues.insert(vlastdofvalues.end(), vpathvalues.at(numnodes+1).first.begin(), vpathvalues.at(numnodes+1).first.begin()+iGroupStartIndex);
            }
            else {
                vstartdofvalues.insert(vstartdofvalues.end(), vpathvalues.at(0).first.begin()+iGroupStartIndex, vpathvalues.at(0).first.end());
                vlastdofvalues.insert(vlastdofvalues.end(), vpathvalues.at(numnodes+1).first.begin()+iGroupStartIndex, vpathvalues.at(numnodes+1).first.end());
            }

            vdelta.resize(vlastdofvalues.size());
            for(int index = 0; index < (int)vdelta.size(); ++index) {
                vdelta[index] = (vlastdofvalues[index]-vstartdofvalues[index])/(fenddist-fstartdist);
            }
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
                if( ioptgroup == 0 ) {
                    for(int subindex = 0; subindex < iGroupStartIndex; ++subindex) {
                        vpathvalues.at(pathindex+1).first.at(subindex) = vstartdofvalues[subindex] + fcurdist*vdelta[subindex];
                    }
                }
                else {
                    for(int subindex = 0; subindex < iGroupStartIndex; ++subindex) {
                        vpathvalues.at(pathindex+1).first.at(subindex+iGroupStartIndex) = vstartdofvalues[subindex] + fcurdist*vdelta[subindex];
                    }
                }
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

            progress._iteration=curiter;
            if( _CallCallbacks(progress) == PA_Interrupt ) {
                return -1;
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

            RAVELOG_VERBOSE_FORMAT("env=%d, singledof iter %d, totaldist=%f", GetEnv()->GetId()%curiter%totaldist);
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

    dReal _ComputePathDurationOnVelocity(const list< vector<dReal> >& listpath)
    {
        dReal ft = 0;
        list< vector<dReal> >::const_iterator itprev = listpath.begin();
        list< vector<dReal> >::const_iterator it = itprev; ++it;
        while(it != listpath.end() ) {
            ft += _ComputeExpectedVelocity(*itprev, *it);
            itprev = it;
            ++it;
        }
        return ft;
    }

    struct SampleInfo
    {
        SampleInfo() : fabsnodedist(0), fdeltadist(0), inode(0) {
        }
        SampleInfo(std::list< vector<dReal> >::iterator itnode, const vector<dReal>& vsample, dReal fabsnodedist, dReal fdeltadist, int inode) : itnode(itnode), vsample(vsample), fabsnodedist(fabsnodedist), fdeltadist(fdeltadist), inode(inode) {
        }
        std::list< vector<dReal> >::iterator itnode;
        vector<dReal> vsample; /// the interpolated data
        dReal fabsnodedist; // absolute distance of itnode
        dReal fdeltadist; // the delta distance between itnode and itnode+1
        int inode; /// index of the node from listpath.begin()
    };

    SampleInfo _SampleBasedOnVelocity(list< vector<dReal> >& listpath, dReal fsearchdist)
    {
        int inode = 0;
        dReal fcurdist = 0;
        list< vector<dReal> >::iterator itprev = listpath.begin();
        list< vector<dReal> >::iterator it = itprev; ++it;
        while(it != listpath.end() ) {
            dReal fdeltadist = _ComputeExpectedVelocity(*itprev, *it);
            if( fsearchdist >= fcurdist && fsearchdist < fcurdist+fdeltadist ) {
                SampleInfo s(itprev, std::vector<dReal>(), fcurdist, fdeltadist, inode);
                s.vsample.resize(it->size());
                dReal f = fdeltadist > 0 ? (fsearchdist-fcurdist)/fdeltadist : 0;
                for(int j = 0; j < (int)s.vsample.size(); ++j) {
                    s.vsample[j] = it->at(j)*f + itprev->at(j)*(1-f);
                }
                return s;
            }
            ++inode;
            fcurdist += fdeltadist;
            itprev = it;
            ++it;
        }
        return SampleInfo(itprev, listpath.back(), fcurdist, 0, inode); // not found, so use last point
    }

    /// \brief interpolates v0*(1-f) + v1*f only for the specific group determined by ioptgroup
    void _InterpolateValuesGroup(const std::vector<dReal>& v0, const std::vector<dReal>& v1, dReal f, int ioptgroup, std::vector<dReal>& vout)
    {
        if( f < 0 || f > 1 ) {
            RAVELOG_WARN_FORMAT("env=%d, bad interpolation value %f!", GetEnv()->GetId()%f);
        }
        int iGroupStartIndex = v0.size()/2;
        OPENRAVE_ASSERT_OP(vout.size(), ==, v0.size());
        for(int i = 0; i < (int)v0.size(); ++i) {
            if( (i < iGroupStartIndex) ^ (ioptgroup>0) ) {
                vout[i] = v0[i]*(1-f) + v1[i]*f;
            }
        }
    }

    void _SetValuesGroup(const std::vector<dReal>& v0, int ioptgroup, std::vector<dReal>& vout)
    {
        int iGroupStartIndex = v0.size()/2;
        OPENRAVE_ASSERT_OP(vout.size(), ==, v0.size());
        for(int i = 0; i < (int)v0.size(); ++i) {
            if( (i < iGroupStartIndex) ^ (ioptgroup>0) ) {
                vout[i] = v0[i];
            }
        }
    }

    bool _AddAndCheck(std::vector<dReal>& v, std::list< vector<dReal> >& listNewNodes, bool bCheckSelfOnly)
    {
        OPENRAVE_ASSERT_OP(listNewNodes.size(),>,0);
        bool bsuccess;
        if( bCheckSelfOnly ) {
            bsuccess = SegmentFeasibleNoTol(listNewNodes.back(), v, IT_OpenStart, CFO_CheckSelfCollisions);
        }
        else {
            bsuccess = SegmentFeasible(listNewNodes.back(), v, IT_OpenStart, 0xffff);
        }

        // get the distance
        dReal f = 0;
        for(size_t i = 0; i < v.size(); ++i) {
            f += (v[i]-listNewNodes.back()[i])*(v[i]-listNewNodes.back()[i]);
            if( v[i] < _parameters->_vConfigLowerLimit[i] ) {
                RAVELOG_WARN_FORMAT("env=%d, dof %d does not follow lower limit %f < %f", GetEnv()->GetId()%i%v[i]%_parameters->_vConfigLowerLimit[i]);
                v[i] = _parameters->_vConfigLowerLimit[i];
            }
            if( v[i] > _parameters->_vConfigUpperLimit[i] ) {
                RAVELOG_WARN_FORMAT("env=%d, dof %d does not follow upper limit %f > %f", GetEnv()->GetId()%i%v[i]%_parameters->_vConfigUpperLimit[i]);
                v[i]  = _parameters->_vConfigUpperLimit[i];
            }
        }
        if( f > 1e-10 ) {
            if (!bsuccess) {
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                    ss << "vstartvalues=["; SerializeValues(ss, listNewNodes.back());
                    ss << "]; vendvalues=[";
                    SerializeValues(ss, v);
                    ss << "]";
                    RAVELOG_VERBOSE_FORMAT("env=%d, not feasible %s", GetEnv()->GetId()%ss.str());
                }
                return false;
            }
        }

        listNewNodes.push_back(v);
        return true;
    }

    dReal _OptimizePathSingleGroupShift(list< vector<dReal> >& listpath, dReal totaldist, int nMaxIterations)
    {
        const PlannerParameters& parameters = *GetParameters();
        list< vector<dReal> >::iterator itmidnode, itmidnodeprev;
        SampleInfo startInfo, endInfo, midInfo;
        vector<dReal> vmidvalues(parameters.GetDOF());

        PlannerProgress progress;

        dReal fMinDistThresh = 0.005;

        int nrejected = 0;
        for(int curiter = 0; curiter < nMaxIterations; ++curiter ) {
            if( nrejected >= 40 ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, smoothing quitting early", GetEnv()->GetId());
                break;
            }
            dReal fstartdist = max(dReal(0),totaldist-parameters._fStepLength)*_puniformsampler->SampleSequenceOneReal(IT_OpenEnd);
            dReal fenddist = fstartdist + (totaldist-fstartdist)*_puniformsampler->SampleSequenceOneReal(IT_OpenStart);
            uint32_t ioptgroup = _puniformsampler->SampleSequenceOneUInt32()%uint32_t(2); // group to optimize
            int iOtherOptGroup = (ioptgroup+1)%2;
            startInfo = _SampleBasedOnVelocity(listpath, fstartdist);


            endInfo = _SampleBasedOnVelocity(listpath, fenddist);

            if( startInfo.itnode == endInfo.itnode || startInfo.itnode == listpath.end() || endInfo.itnode == listpath.end() ) {
                // choose a line, so ignore
                RAVELOG_VERBOSE_FORMAT("env=%d, sampled same node", GetEnv()->GetId());
                continue;
            }

            nrejected++;

            dReal fTimeToOptGroup = _ComputeExpectedVelocityGroup(startInfo.vsample, endInfo.vsample, ioptgroup);

            // compute the time from optgroup to go from startInfo to endInfo
            list< vector<dReal> >::const_iterator itoptgroup = startInfo.itnode; itoptgroup++;
            list< vector<dReal> >::const_iterator itoptgroupprev = startInfo.itnode;

            dReal fOrigOptGroupTime = startInfo.fabsnodedist + startInfo.fdeltadist - fstartdist;

            if(itoptgroup != listpath.end() ) {
                itoptgroupprev = itoptgroup;
                itoptgroup++;
                while(itoptgroup != listpath.end() ) {
                    fOrigOptGroupTime += _ComputeExpectedVelocityGroup(*itoptgroupprev, *itoptgroup, ioptgroup);
                    if( itoptgroup == endInfo.itnode ) {
                        break;
                    }
                    itoptgroupprev = itoptgroup;
                    itoptgroup++;
                }
            }
            fOrigOptGroupTime = fenddist - endInfo.fabsnodedist;

            if( fOrigOptGroupTime + fMinDistThresh > fTimeToOptGroup ) {
                continue;
            }

            dReal fmiddist = fstartdist + fTimeToOptGroup;
            midInfo = _SampleBasedOnVelocity(listpath, fmiddist);
            if( midInfo.itnode == listpath.end() ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, could not find at %f/%f", GetEnv()->GetId()%fmiddist%totaldist);
                continue;
            }

//            for(int i = 0; i < (int)vendvalues.size(); ++i) {
//                if( (i < iGroupStartIndex) ^ (ioptgroup>0) ) {
//                    //
//                }
//                else {
//                    vendvalues[i] = vmidvalues[i];
//                }
//            }

//            dReal fnewsegmentdist = _ComputeExpectedVelocityGroup(vstartvalues, vendvalues, ioptgroup);
//            if( fnewsegmentdist > fenddist-fstartdist+1e-7 ) {
//                RAVELOG_INFO_FORMAT("env=%d, expected total distance is not that great: %f > %f, (fTimeToOptGroup=%f)", GetEnv()->GetId()%fnewsegmentdist%(fenddist-fstartdist)%fTimeToOptGroup);
//                continue;
//            }

            progress._iteration=curiter;
            if( _CallCallbacks(progress) == PA_Interrupt ) {
                return -1;
            }

            // create a new path of optgroup and check with the other group
            // there are two segments here:
            // 1) the new shortened segment for ioptgroup between vstartvalues and vendvalues with a time of fTimeToOptGroup
            // 2) the other group will go all the way to midInfo, which is fstartdist+fTimeToOptGroup
            bool bIsFeasible = true;
            list< vector<dReal> > listNewNodes;
            listNewNodes.push_back(startInfo.vsample);

            {
                list< vector<dReal> >::const_iterator itinterpnode, itinterpnodeprev;

                //vprev = vstartvalues;

                itinterpnodeprev = itinterpnode = startInfo.itnode;
                itinterpnode++;
                if(itinterpnode==listpath.end()) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, could not find interp", GetEnv()->GetId());
                    continue;
                }

                //dReal fLeftOverDistFromFirstNode = startInfo.fdeltadist - (fstartdist - startInfo.fabsnodedist);
                dReal fOtherDeltaDist = _ComputeExpectedVelocityGroup(startInfo.vsample, *itinterpnode, iOtherOptGroup);
                //dReal fAlongOtherDeltaDist = _ComputeExpectedVelocityGroup(*itinterpnodeprev, startInfo.vsample, iOtherOptGroup); // always with respect to itinterpnodeprev and itinterpnode

                dReal fTravelTimeToOptGroup = 0;
                std::vector<dReal> vmidvalues;
                if( fOtherDeltaDist <= fTimeToOptGroup-fTravelTimeToOptGroup ) {

                    if( fOtherDeltaDist > 0 ) {
                        // ioptgroup -> interpolate by (fTravelTimeToOptGroup+fOtherDeltaDist)/fTimeToOptGroup
                        // iOtherOptGroup -> use *itinterpnode
                        vmidvalues = *itinterpnode;
                        fTravelTimeToOptGroup += fOtherDeltaDist;
                        _InterpolateValuesGroup(startInfo.vsample, endInfo.vsample, fTravelTimeToOptGroup/fTimeToOptGroup, ioptgroup, vmidvalues);
                        if( !_AddAndCheck(vmidvalues, listNewNodes, false) ) {
                            bIsFeasible = false;
                            continue;
                        }
                    }

                    itinterpnodeprev = itinterpnode;
                    ++itinterpnode;
                    while(itinterpnode != listpath.end() && fTravelTimeToOptGroup < fTimeToOptGroup) {
                        fOtherDeltaDist = _ComputeExpectedVelocityGroup(*itinterpnodeprev, *itinterpnode, iOtherOptGroup);
                        if( fOtherDeltaDist > 0 ) {
                            if( fOtherDeltaDist <= fTimeToOptGroup-fTravelTimeToOptGroup ) {
                                // ioptgroup -> interpolate by (fTravelTimeToOptGroup+fOtherDeltaDist)/fTimeToOptGroup
                                // iOtherOptGroup -> use *itinterpnode
                                vmidvalues = *itinterpnode;
                                fTravelTimeToOptGroup += fOtherDeltaDist;
                                _InterpolateValuesGroup(startInfo.vsample, endInfo.vsample, fTravelTimeToOptGroup/fTimeToOptGroup, ioptgroup, vmidvalues);
                                if( !_AddAndCheck(vmidvalues, listNewNodes, false) ) {
                                    bIsFeasible = false;
                                    break;
                                }
                            }
                            else {
                                _InterpolateValuesGroup(*itinterpnodeprev, *itinterpnode, (fTimeToOptGroup-fTravelTimeToOptGroup)/fOtherDeltaDist, iOtherOptGroup, vmidvalues);
                                fTravelTimeToOptGroup = fTimeToOptGroup;
                                //fAlongOtherDeltaDist = fTimeToOptGroup-fTravelTimeToOptGroup;
                                if( !_AddAndCheck(vmidvalues, listNewNodes, false) ) {
                                    bIsFeasible = false;
                                    break;
                                }
                                break;
                            }
                        }

                        itinterpnodeprev = itinterpnode;
                        ++itinterpnode;
                    }
                }
                else if( fOtherDeltaDist > fTimeToOptGroup ) {
                    vmidvalues = endInfo.vsample;
                    _InterpolateValuesGroup(startInfo.vsample, *itinterpnode, fTimeToOptGroup/fOtherDeltaDist, iOtherOptGroup, vmidvalues);
                    fTravelTimeToOptGroup = fTimeToOptGroup;
                    //fAlongOtherDeltaDist = fTimeToOptGroup;
                    if( !_AddAndCheck(vmidvalues, listNewNodes, false) ) {
                        bIsFeasible = false;
                        continue;
                    }
                }

                if( !bIsFeasible ) {
                    continue;
                }

                if( fTravelTimeToOptGroup < fTimeToOptGroup ) {
                    // only time this can happen is if itinterpnode is at the end, in that case add the end
                    BOOST_ASSERT(itinterpnode==listpath.end());
                    vmidvalues = endInfo.vsample;
                    _SetValuesGroup(listpath.back(), iOtherOptGroup, vmidvalues);
                    fTravelTimeToOptGroup = fTimeToOptGroup;
                    if( !_AddAndCheck(vmidvalues, listNewNodes, false) ) {
                        bIsFeasible = false;
                        continue;
                    }
                }

                // at this point listNewNodes.back() has the new ramp finished for ioptgroup (up to vendvalues) and otherGroup is between itinterpnodeprev and itinterpnode with a dist of fAlongOtherDeltaDist

                //RAVELOG_INFO_FORMAT("env=%d, new path %d", GetEnv()->GetId()%listNewNodes.size());

                // now need to get a path to the end
                itoptgroup = endInfo.itnode;
                itoptgroupprev = endInfo.itnode;
                list< vector<dReal> >::const_iterator itothergroup = itinterpnode;
                list< vector<dReal> >::const_iterator itothergroupprev = itinterpnodeprev;
                itoptgroup++;

                while(itothergroup != listpath.end() ) { // based on other group
                    // figure out the min dist until the next node
                    dReal fOtherDist = _ComputeExpectedVelocityGroup(listNewNodes.back(), *itothergroup, iOtherOptGroup);
                    if( fOtherDist <= fMinDistThresh ) {
                        itothergroupprev = itothergroup;
                        ++itothergroup;
                        continue;
                    }

                    //dReal fOtherNextDist = fOtherNodeDist + fOtherDist;
                    if( itoptgroup != listpath.end() ) {
                        dReal fOptDist = 0;
                        while(itoptgroup != listpath.end() ) {
                            fOptDist = _ComputeExpectedVelocityGroup(listNewNodes.back(), *itoptgroup, ioptgroup);
                            if( fOptDist <= fMinDistThresh ) {
                                itoptgroupprev = itoptgroup;
                                ++itoptgroup;
                                continue;
                            }

                            if( fOtherDist < fOptDist ) {
                                // the other group hits the node first
                                break;
                            }

                            // fOtherDist >= fOptDist

                            // sample at opt dist
                            //RAVELOG_INFO_FORMAT("env=%d, add new path %f/%f = %f", GetEnv()->GetId()%fOptDist%fOtherDist%(fOptDist/fOtherDist));

                            vmidvalues = *itoptgroup;
                            _InterpolateValuesGroup(listNewNodes.back(), *itothergroup, fOptDist/fOtherDist, iOtherOptGroup, vmidvalues);
                            if( !_AddAndCheck(vmidvalues, listNewNodes, true) ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, not feasible group=%d, start=%f (%d), end=%f (%d), total=%f", GetEnv()->GetId()%ioptgroup%fstartdist%startInfo.inode%fenddist%endInfo.inode%totaldist);
                                bIsFeasible = false;
                                break;
                            }

                            fOtherDist = _ComputeExpectedVelocityGroup(listNewNodes.back(), *itothergroup, iOtherOptGroup);

                            itoptgroupprev = itoptgroup;
                            ++itoptgroup;
                        }

                        //dReal fTestOptDist = _ComputeExpectedVelocityGroup(listNewNodes.back(), *itoptgroup, ioptgroup);

                        // fOtherDist < fOptDist

                        if( fOptDist > g_fEpsilonLinear ) {
                            dReal f = fOtherDist/fOptDist;
                            if( f > 1 ) {
                                f = 1;
                            }
                            vmidvalues = *itothergroup;
                            if( itoptgroup != listpath.end() ) {
                                _InterpolateValuesGroup(listNewNodes.back(), *itoptgroup, f, ioptgroup, vmidvalues);
                            }
                            else {
                                _InterpolateValuesGroup(listNewNodes.back(), listpath.back(), f, ioptgroup, vmidvalues);
                            }

                            if( !_AddAndCheck(vmidvalues, listNewNodes, true) ) {
                                RAVELOG_VERBOSE_FORMAT("env=%d, not feasible group=%d, start=%f, end=%f", GetEnv()->GetId()%ioptgroup%fstartdist%fenddist);
                                bIsFeasible = false;
                                break;
                            }

                            fOtherDist = _ComputeExpectedVelocityGroup(listNewNodes.back(), *itothergroup, iOtherOptGroup);
                        }
                    }

                    if( fOtherDist > g_fEpsilonLinear ) {
                        vmidvalues = *itothergroup;
                        _SetValuesGroup(listpath.back(),ioptgroup,vmidvalues);
                        if( !_AddAndCheck(vmidvalues, listNewNodes, true) ) {
                            RAVELOG_VERBOSE_FORMAT("env=%d, not feasible group=%d, start=%f, end=%f", GetEnv()->GetId()%ioptgroup%fstartdist%fenddist);
                            bIsFeasible = false;
                            break;
                        }
                    }

                    itothergroupprev = itothergroup;
                    ++itothergroup;
                }

                if( !bIsFeasible ) {
                    //RAVELOG_DEBUG_FORMAT("env=%d, not feasible %f->%f", GetEnv()->GetId()%curiter%dist);
                    continue;
                }


                if( itoptgroup != listpath.end() ) {
                    while(itoptgroup != listpath.end() ) { // based on other group
                        vmidvalues = *itoptgroup;
                        _SetValuesGroup(listpath.back(),iOtherOptGroup,vmidvalues);
                        if( !_AddAndCheck(vmidvalues, listNewNodes, true) ) {
                            RAVELOG_VERBOSE_FORMAT("env=%d, not feasible group=%d, start=%f, end=%f", GetEnv()->GetId()%ioptgroup%fstartdist%fenddist);
                            bIsFeasible = false;
                            break;
                        }

                        itoptgroupprev = itoptgroup;
                        ++itoptgroup;
                    }

                    if( !bIsFeasible ) {
                        //RAVELOG_DEBUG_FORMAT("env=%d, not feasible %f->%f", GetEnv()->GetId()%curiter%dist);
                        continue;
                    }

                    // make sure that the last point in listpath is covered!
                    dReal fFinalDist = _ComputeExpectedVelocity(listNewNodes.back(), listpath.back());
                    if( fFinalDist > g_fEpsilonLinear ) {
                        RAVELOG_WARN_FORMAT("env=%d, still some distance left %f with optgroup %d, so fill it", GetEnv()->GetId()%fFinalDist%ioptgroup);
                        if( !_AddAndCheck(listpath.back(), listNewNodes, true) ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, final not feasible group=%d, start=%f, end=%f", GetEnv()->GetId()%ioptgroup%fstartdist%fenddist);
                            bIsFeasible = false;
                            continue;
                        }
                    }
                }
                else {
                    // make sure that the last point in listpath is covered!
                    dReal fFinalDist = _ComputeExpectedVelocity(listNewNodes.back(), listpath.back());
                    if( fFinalDist > g_fEpsilonLinear ) {
                        if( !_AddAndCheck(listpath.back(), listNewNodes, true) ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, final not feasible group=%d, start=%f, end=%f", GetEnv()->GetId()%ioptgroup%fstartdist%fenddist);
                            bIsFeasible = false;
                            continue;
                        }
                    }
                }

                if( !bIsFeasible ) {
                    //RAVELOG_DEBUG_FORMAT("env=%d, not feasible %f->%f", GetEnv()->GetId()%curiter%dist);
                    continue;
                }
            }

            ++startInfo.itnode;
            listpath.erase(startInfo.itnode, listpath.end());
            listpath.splice(listpath.end(), listNewNodes, listNewNodes.begin(), listNewNodes.end());
            dReal newtotaldist = _ComputePathDurationOnVelocity(listpath);
            RAVELOG_INFO_FORMAT("env=%d, smoother iter %d, totaldist=%f -> %f, new path %d", GetEnv()->GetId()%curiter%totaldist%newtotaldist%listpath.size());
            totaldist = newtotaldist;
            nrejected = 0;
        }
        // double check the distances
//        dReal dist = 0;
//        FOREACH(it, listpath) {
//            dist += it->second;
//        }
//        if( RaveFabs(totaldist-dist) > 0 ) {
//            RAVELOG_WARN_FORMAT("env=%d, totaldist=%f, dist=%f", GetEnv()->GetId()%totaldist%dist);//OPENRAVE_ASSERT_OP(RaveFabs(totaldist-dist),<=,1e-7);
//        }
        return totaldist;
    }

    /// \brief checks if a segment is feasible using pointtolerance
    inline bool SegmentFeasible(const std::vector<dReal>& a,const std::vector<dReal>& b, IntervalType interval, int options=0xffff)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        // have to also test with tolerances!
        boost::array<dReal,3> perturbations = {{ 0,_parameters->_pointtolerance,-_parameters->_pointtolerance }}; // note that it is using _parameters in order to avoid casting parameters, which might not work
        std::vector<dReal> anew(a.size()), bnew(b.size());
        FOREACH(itperturbation,perturbations) {
            for(size_t i = 0; i < a.size(); ++i) {
                anew[i] = a[i] + *itperturbation * parameters->_vConfigResolution.at(i);
                if( anew[i] < _parameters->_vConfigLowerLimit[i] ) {
                    anew[i] = _parameters->_vConfigLowerLimit[i];
                }
                if( anew[i] > _parameters->_vConfigUpperLimit[i] ) {
                    anew[i]  = _parameters->_vConfigUpperLimit[i];
                }

                bnew[i] = b[i] + *itperturbation * parameters->_vConfigResolution.at(i);
                if( bnew[i] < _parameters->_vConfigLowerLimit[i] ) {
                    bnew[i] = _parameters->_vConfigLowerLimit[i];
                }
                if( bnew[i] > _parameters->_vConfigUpperLimit[i] ) {
                    bnew[i]  = _parameters->_vConfigUpperLimit[i];
                }
            }
            if( parameters->CheckPathAllConstraints(anew,bnew,std::vector<dReal>(), std::vector<dReal>(), 0, interval, options) != 0 ) {
                return false;
            }
        }
        return true;
    }

    inline bool SegmentFeasibleNoTol(const std::vector<dReal>& a,const std::vector<dReal>& b, IntervalType interval, int options=0xffff)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        // have to also test with tolerances!
        if( parameters->CheckPathAllConstraints(a,b,std::vector<dReal>(), std::vector<dReal>(), 0, interval, options) != 0 ) {
            return false;
        }
        return true;
    }

    TrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _puniformsampler;
    RobotBasePtr _probot;
    PlannerBasePtr _linearretimer;
    std::vector<dReal> _vConfigVelocityLimitInv;
    int _nUseSingleDOFSmoothing;
};

PlannerBasePtr CreateLinearSmoother(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new LinearSmoother(penv, sinput));
}
