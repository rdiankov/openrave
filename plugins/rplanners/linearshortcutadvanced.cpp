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

class ShortcutLinearPlanner : public PlannerBase
{
public:
    ShortcutLinearPlanner(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\npath optimizer using linear shortcuts.";
        _linearretimer = RaveCreatePlanner(GetEnv(), "LinearTrajectoryRetimer");
    }
    virtual ~ShortcutLinearPlanner() {
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

        _OptimizePath(listpath);

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

    TrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _puniformsampler;
    RobotBasePtr _probot;
    PlannerBasePtr _linearretimer;
    ConstraintFilterReturnPtr _filterreturn;
    std::vector<dReal> _vtempdists;
};

PlannerBasePtr CreateShortcutLinearPlanner(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new ShortcutLinearPlanner(penv, sinput));
}
