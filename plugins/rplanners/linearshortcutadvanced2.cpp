// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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

class ShortcutLinearPlanner2 : public PlannerBase
{
public:
    ShortcutLinearPlanner2(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\npath optimizer using linear shortcuts.";
        _linearretimer = RaveCreatePlanner(GetEnv(), "LinearTrajectoryRetimer");
    }
    virtual ~ShortcutLinearPlanner2() {
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
        _linearretimer->InitPlan(RobotBasePtr(), _parameters);
        _puniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        if( !!_puniformsampler ) {
            _puniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        }
        _logginguniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        if( !!_logginguniformsampler ) {
            _logginguniformsampler->SetSeed(utils::GetMicroTime());
        }
        {
            _probot->GetDOFWeights(_vdofweights);
            _vdofweightsaccumulated.resize(_vdofweights.size());
            _totaldofweights = 0;
            for (size_t i = 0; i < _vdofweights.size(); ++i) {
                OPENRAVE_ASSERT_OP(_vdofweights[i], >, 0);
                _totaldofweights += _vdofweights[i];
                _vdofweightsaccumulated[i] = _totaldofweights;
            }
        }
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
        _SubsampleTrajectory(ptraj,listpath);

        TrajectoryBasePtr ptraj0, ptraj1, ptraj2;
        ptraj0 = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
        ptraj0->Init(parameters->_configurationspecification);
        ptraj1 = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
        ptraj1->Init(parameters->_configurationspecification);
        ptraj2 = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
        ptraj2->Init(parameters->_configurationspecification);

        FOREACH(it, listpath) {
            ptraj0->Insert(ptraj0->GetNumWaypoints(), it->first);
        }
        _OptimizePath(listpath);
        FOREACH(it, listpath) {
            ptraj1->Insert(ptraj1->GetNumWaypoints(), it->first);
        }
        
        _OptimizePathSingleDOF(listpath);
        
        FOREACH(it, listpath) {
            ptraj2->Insert(ptraj2->GetNumWaypoints(), it->first);
        }

        _DumpTrajectory(ptraj0, Level_Debug);
        _DumpTrajectory(ptraj1, Level_Debug);
        _DumpTrajectory(ptraj2, Level_Debug);        

        ptraj->Init(parameters->_configurationspecification);
        FOREACH(it, listpath) {
            ptraj->Insert(ptraj->GetNumWaypoints(),it->first);
        }
        RAVELOG_DEBUG(str(boost::format("path optimizing - computation time=%fs\n")%(0.001f*(float)(utils::GetMilliTime()-basetime))));
        if( parameters->_sPostProcessingPlanner.size() == 0 ) {
            // no other planner so at least retime
            PlannerStatus status = _linearretimer->PlanPath(ptraj);
            if( status != PS_HasSolution ) {
                return status;
            }
            return PS_HasSolution;
        }
        return _ProcessPostPlanners(RobotBasePtr(),ptraj);
    }

protected:
    void _OptimizePath(list< std::pair< vector<dReal>, dReal> >& listpath)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        list< std::pair< vector<dReal>, dReal> >::iterator itstartnode, itendnode;
        if( !_filterreturn ) {
            _filterreturn.reset(new ConstraintFilterReturn());
        }

        int dof = parameters->GetDOF();
        int nrejected = 0;
        int iiter = parameters->_nMaxIterations;
        std::vector<dReal> vnewconfig0(dof), vnewconfig1(dof);
        while(iiter > 0  && nrejected < (int)listpath.size()+4 && listpath.size() > 2 ) {
            --iiter;

            // pick a random node on the listpath, and a random jump ahead
            uint32_t endIndex = 2+(_puniformsampler->SampleSequenceOneUInt32()%((uint32_t)listpath.size()-2));
            uint32_t startIndex = _puniformsampler->SampleSequenceOneUInt32()%(endIndex-1);

            itstartnode = listpath.begin();
            advance(itstartnode, startIndex);
            itendnode = itstartnode;
            dReal totaldistance = 0;
            for(uint32_t j = 0; j < endIndex-startIndex; ++j) {
                ++itendnode;
                totaldistance += itendnode->second;
            }
            nrejected++;

            dReal expectedtotaldistance = parameters->_distmetricfn(itstartnode->first, itendnode->first);
            if( expectedtotaldistance > totaldistance-0.1*parameters->_fStepLength ) {
                // expected total distance is not that great
                continue;
            }

            // check if the nodes can be connected by a straight line
            _filterreturn->Clear();
            if (parameters->CheckPathAllConstraints(itstartnode->first, itendnode->first, std::vector<dReal>(), std::vector<dReal>(), 0, IT_Open, 0xffff|CFO_FillCheckedConfiguration, _filterreturn) != 0 ) {
                if( nrejected++ > (int)listpath.size()+8 ) {
                    break;
                }
                continue;
            }
            if(_filterreturn->_configurations.size() == 0 ) {
                continue;
            }
            OPENRAVE_ASSERT_OP(_filterreturn->_configurations.size()%dof, ==, 0);
            // check how long the new path is
            _vtempdists.resize(_filterreturn->_configurations.size()/dof+1);
            std::vector<dReal>::iterator itdist = _vtempdists.begin();
            std::vector<dReal>::iterator itnewconfig = _filterreturn->_configurations.begin();
            std::copy(itnewconfig, itnewconfig+dof, vnewconfig0.begin());
            dReal newtotaldistance = parameters->_distmetricfn(itstartnode->first, vnewconfig0);
            *itdist++ = newtotaldistance;
            itnewconfig += dof;
            while(itnewconfig != _filterreturn->_configurations.end() ) {
                std::copy(itnewconfig, itnewconfig+dof, vnewconfig1.begin());
                *itdist = parameters->_distmetricfn(vnewconfig0, vnewconfig1);
                newtotaldistance += *itdist;
                ++itdist;
                vnewconfig0.swap(vnewconfig1);
                itnewconfig += dof;
            }
            *itdist = parameters->_distmetricfn(vnewconfig0, itendnode->first);
            newtotaldistance += *itdist;
            ++itdist;
            BOOST_ASSERT(itdist==_vtempdists.end());

            if( newtotaldistance > totaldistance-0.1*parameters->_fStepLength ) {
                // new path is not that good, so reject
                nrejected++;
                continue;
            }

            // finally add
            itdist = _vtempdists.begin();
            ++itstartnode;
            itnewconfig = _filterreturn->_configurations.begin();
            while(itnewconfig != _filterreturn->_configurations.end()) {
                std::copy(itnewconfig, itnewconfig+dof, vnewconfig1.begin());
                listpath.insert(itstartnode, make_pair(vnewconfig1, *itdist++));
                itnewconfig += dof;
            }
            itendnode->second = *itdist++;
            BOOST_ASSERT(itdist==_vtempdists.end());

            // splice out in-between nodes in path
            listpath.erase(itstartnode, itendnode);
            nrejected = 0;

            if( listpath.size() <= 2 ) {
                break;
            }

            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                dReal newdistance = 0;
                FOREACH(ittempnode, listpath) {
                    newdistance += ittempnode->second;
                }
                RAVELOG_VERBOSE_FORMAT("shortcut iter=%d, path=%d, dist=%f", (parameters->_nMaxIterations-iiter)%listpath.size()%newdistance);
            }
        }
    }

    void _OptimizePathSingleDOF(list< std::pair< vector<dReal>, dReal> >& listpath) {
        RAVELOG_DEBUG("start _OptimizePathSingleDOF");
        if (listpath.size() <= 3) {
            RAVELOG_DEBUG("listpath is already too short. Skipping.");
            return;
        }
        
        PlannerParametersConstPtr parameters = GetParameters();

        dReal totalDist = 0;
        FOREACH(itnode, listpath) {
            totalDist += itnode->second;
        }
        dReal newTotalDist = 0;
        int nsuccess = 0;

        int nrejected = 0;
        int maxconsecutiverejection = 20;
        int maxIters = parameters->_nMaxIterations;

        std::vector<std::pair<std::vector<dReal>, dReal> > vnodes;

        int iter = 0;
        for (iter = 0; iter < maxIters && listpath.size() > 3; ++iter) {
            // Pick a random node on the listpath and a random jump ahead.
            uint32_t endIndex = 2 + (_puniformsampler->SampleSequenceOneUInt32()%((uint32_t)listpath.size() - 3));
            uint32_t startIndex = _puniformsampler->SampleSequenceOneUInt32()%(endIndex - 1);
            if (endIndex - startIndex <= 1) {
                continue;
            }

            // Pick a dof to shortcut
            dReal randNum = _puniformsampler->SampleSequenceOneReal(IT_Closed)*_totaldofweights;
            std::vector<dReal>::const_iterator itdof = std::lower_bound(_vdofweightsaccumulated.begin(), _vdofweightsaccumulated.end(), randNum);
            int ioptdof = itdof - _vdofweightsaccumulated.begin();

            RAVELOG_VERBOSE_FORMAT("it = %d/%d: dof = %d; startIndex = %d/%d; endIndex = %d/%d", iter%maxIters%ioptdof%startIndex%listpath.size()%endIndex%listpath.size());

            std::list<std::pair<std::vector<dReal>, dReal> >::iterator itstart, itend;
            itstart = listpath.begin();
            std::advance(itstart, startIndex);
            
            itend = listpath.begin();
            std::advance(itend, endIndex);
            
            dReal inv = 1.0/(endIndex - startIndex);
            
            dReal ffirstdofval = itstart->first[ioptdof], flastdofval = itend->first[ioptdof];
            dReal fdofdisplacement = flastdofval - ffirstdofval;
            // Prepare the container.
            vnodes.resize(endIndex - startIndex + 1);

            ++itend;
            std::copy(itstart, itend, vnodes.begin());

            std::vector<std::pair<std::vector<dReal>, dReal> >::iterator itnode, itprevnode;
            itnode = vnodes.begin() + 1;
            itprevnode = vnodes.begin();

            bool bsuccess = true;
            while (itnode != vnodes.end()) {
                RAVELOG_VERBOSE_FORMAT("vnodes at %d = [%.15e, %.15e, %.15e, %.15e, %.15e, %.15e];", (itnode - vnodes.begin())%(itnode->first[0])%(itnode->first[1])%(itnode->first[2])%(itnode->first[3])%(itnode->first[4])%(itnode->first[5]));

                // Compute the new DOF value for the joint ioptdof. We want to replace the original path for DOF ioptdof by a straight line.
                dReal fdofval = ffirstdofval + ((itnode - vnodes.begin())*inv)*fdofdisplacement;

                // Replace the old value by the computed one
                itnode->first[ioptdof] = fdofval;

                // Check feasibility
                IntervalType intervaltype = (itprevnode - vnodes.begin() + 2 == (int)vnodes.size()) ? IT_Open : IT_OpenStart;
                if (!SegmentFeasible(itprevnode->first, itnode->first, intervaltype)) {
                    bsuccess = false;
                    break;
                }

                // Update the overall distance
                dReal fdist = parameters->_distmetricfn(itprevnode->first, itnode->first);
                itnode->second = fdist;
                
                ++itprevnode;
                ++itnode;
            }
            if (!bsuccess) {
                continue;
            }

            // Update listpath
            std::list<std::pair<std::vector<dReal>, dReal> >::iterator itlist = itstart;
            for (size_t i = 1; i < vnodes.size(); ++i) {
                ++itlist;
                itlist->first = vnodes.at(i).first;
                itlist->second = vnodes.at(i).second;
            }
            nsuccess += 1;
            RAVELOG_VERBOSE_FORMAT("iter = %d: successful", iter);
        }
        FOREACH(itnode, listpath) {
            newTotalDist += itnode->second;
        }
        RAVELOG_DEBUG_FORMAT("finished optimizing. successful = %d/%d; total distance %.15e -> %.15e; diff = %.15e", nsuccess%maxIters%totalDist%newTotalDist%(totalDist - newTotalDist));
    }

    void _SubsampleTrajectory(TrajectoryBasePtr ptraj, list< std::pair< vector<dReal>, dReal> >& listpath) const
    {
        PlannerParametersConstPtr parameters = GetParameters();
        vector<dReal> q0(parameters->GetDOF()), q1(parameters->GetDOF()), dq(parameters->GetDOF()), qcur(parameters->GetDOF()), dq2;
        vector<dReal> vtrajdata;
        ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajdata,parameters->_configurationspecification);

        std::copy(vtrajdata.begin(),vtrajdata.begin()+parameters->GetDOF(),q0.begin());
        listpath.push_back(make_pair(q0,dReal(0)));
        qcur = q0;
        
        for(size_t ipoint = 1; ipoint < ptraj->GetNumWaypoints(); ++ipoint) {
            std::copy(vtrajdata.begin()+(ipoint)*parameters->GetDOF(),vtrajdata.begin()+(ipoint+1)*parameters->GetDOF(),q1.begin());
            dq = q1;            
            parameters->_diffstatefn(dq,q0);
            int i, numSteps = 1;
            vector<dReal>::const_iterator itres = parameters->_vConfigResolution.begin();
            for (i = 0; i < parameters->GetDOF(); i++,itres++) {
                int steps;
                if( *itres != 0 ) {
                    steps = (int)(RaveFabs(dq[i]) / *itres);
                }
                else {
                    steps = (int)(RaveFabs(dq[i]) * 100);
                }
                if (steps > numSteps) {
                    numSteps = steps;
                }
            }
            dReal fisteps = dReal(1.0f)/numSteps;
            FOREACH(it,dq) {
                *it *= fisteps;
            }
            int mult = 1;
            for (int f = 1; f < numSteps; f++) {
                bool bsuccess = false;
                if( mult > 1 ) {
                    dq2 = dq;
                    FOREACHC(it, dq2) {
                        *it *= mult;
                    }
                    bsuccess = parameters->_neighstatefn(qcur,dq2,NSO_OnlyHardConstraints);
                }
                else {
                    bsuccess = parameters->_neighstatefn(qcur,dq,NSO_OnlyHardConstraints);
                }
                if( !bsuccess ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, neighstatefn failed mult=%d, perhaps non-linear constraints are used?", GetEnv()->GetId()%mult);
                    mult++;
                    continue;
                }
                dReal dist = parameters->_distmetricfn(listpath.back().first,qcur);
                listpath.push_back(make_pair(qcur, dist));
                mult = 1;
            }
            // always add the last point
            dReal dist = parameters->_distmetricfn(listpath.back().first,q1);
            listpath.push_back(make_pair(q1, dist));
            q0.swap(q1);
            qcur = q1;
        }

        std::copy(vtrajdata.end()-parameters->GetDOF(),vtrajdata.end(),q0.begin());
        dReal dist = parameters->_distmetricfn(listpath.back().first,q0);
        listpath.push_back(make_pair(q0,dist));
    }

    inline bool SegmentFeasible(const std::vector<dReal>& a,const std::vector<dReal>& b, IntervalType interval)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        // // have to also test with tolerances!
        // boost::array<dReal,3> perturbations = {{ 0, _parameters->_pointtolerance, -_parameters->_pointtolerance }}; // note that it is using _parameters in order to avoid casting parameters, which might not work
        // std::vector<dReal> anew(a.size()), bnew(b.size());
        // FOREACH(itperturbation,perturbations) {
        //     for(size_t i = 0; i < a.size(); ++i) {
        //         anew[i] = a[i] + *itperturbation * parameters->_vConfigResolution.at(i);
        //         bnew[i] = b[i] + *itperturbation * parameters->_vConfigResolution.at(i);
        //     }
        //     if( parameters->CheckPathAllConstraints(anew, bnew, std::vector<dReal>(), std::vector<dReal>(), 0, interval) != 0 ) {
        //         return false;
        //     }
        // }
        if (parameters->CheckPathAllConstraints(a, b, std::vector<dReal>(), std::vector<dReal>(), 0, interval) != 0) {
            return false;
        }
        return true;
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj, DebugLevel level)
    {
        if( IS_DEBUGLEVEL(level) ) {
            std::string filename = _DumpTrajectory(traj);
            RavePrintfA(str(boost::format("env=%d, wrote linearshortcutadvanced trajectory to %s")%GetEnv()->GetId()%filename), level);
            return filename;
        }
        return std::string();
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj)
    {
        // store the trajectory
        uint32_t randnum;
        if( !!_puniformsampler ) {
            randnum = _logginguniformsampler->SampleSequenceOneUInt32();
        }
        else {
            randnum = RaveRandomInt();
        }
        string filename = str(boost::format("%s/linearshortcutadvanced%d.traj.xml")%RaveGetHomeDirectory()%(randnum%1000));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        traj->serialize(f);
        return filename;
    }

    TrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _puniformsampler, _logginguniformsampler;
    RobotBasePtr _probot;
    PlannerBasePtr _linearretimer;
    ConstraintFilterReturnPtr _filterreturn;
    std::vector<dReal> _vtempdists;

    dReal _totaldofweights;
    std::vector<dReal> _vdofweights, _vdofweightsaccumulated;
};

PlannerBasePtr CreateShortcutLinearPlanner2(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new ShortcutLinearPlanner2(penv, sinput));
}
