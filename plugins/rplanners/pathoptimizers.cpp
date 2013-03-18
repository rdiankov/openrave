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
#include "rplanners.h"

class ShortcutLinearPlanner : public PlannerBase
{
public:
    ShortcutLinearPlanner(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\npath optimizer using linear shortcuts.";
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
        return true;
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
        _ProcessPostPlanners(RobotBasePtr(),ptraj);
        return PS_HasSolution;
    }

protected:
    void _OptimizePath(list< std::pair< vector<dReal>, dReal> >& listpath)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        std::vector<dReal> vtempdists;
        list< std::pair< vector<dReal>, dReal> >::iterator itstartnode, itendnode;
        ConfigurationListPtr listNewConfigs(new ConfigurationList());

        int nrejected = 0;
        int i = parameters->_nMaxIterations;
        while(i > 0 && nrejected < (int)listpath.size()+4 && listpath.size() > 2 ) {
            --i;

            // pick a random node on the listpath, and a random jump ahead
            int endIndex = 2+(RaveRandomInt()%((int)listpath.size()-2));
            int startIndex = RaveRandomInt()%(endIndex-1);

            itstartnode = listpath.begin();
            advance(itstartnode, startIndex);
            itendnode = itstartnode;
            dReal totaldistance = 0;
            for(int j = 0; j < endIndex-startIndex; ++j) {
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
            listNewConfigs->clear();
            if (!parameters->_checkpathconstraintsfn(itstartnode->first, itendnode->first, IT_Open, listNewConfigs)) {
                if( nrejected++ > (int)listpath.size()+8 ) {
                    break;
                }
                continue;
            }
            if(listNewConfigs->size() == 0 ) {
                continue;
            }

            // check how long the new path is
            ConfigurationList::iterator itnewconfig1 = listNewConfigs->begin();
            ConfigurationList::iterator itnewconfig0 = itnewconfig1++;
            vtempdists.resize(listNewConfigs->size()+1);
            std::vector<dReal>::iterator itdist = vtempdists.begin();
            dReal newtotaldistance = parameters->_distmetricfn(itstartnode->first, *itnewconfig0);
            *itdist++ = newtotaldistance;
            while(itnewconfig1 != listNewConfigs->end() ) {
                *itdist = parameters->_distmetricfn(*itnewconfig0, *itnewconfig1);
                newtotaldistance += *itdist;
                ++itdist;
                itnewconfig0 = itnewconfig1;
                ++itnewconfig1;
            }
            *itdist = parameters->_distmetricfn(*itnewconfig0, itendnode->first);
            newtotaldistance += *itdist;
            ++itdist;
            BOOST_ASSERT(itdist==vtempdists.end());

            if( newtotaldistance > totaldistance-0.1*parameters->_fStepLength ) {
                // new path is not that good, so reject
                continue;
            }

            // finally add
            itdist = vtempdists.begin();
            ++itstartnode;
            FOREACHC(itc, *listNewConfigs) {
                listpath.insert(itstartnode, make_pair(*itc,*itdist++));
            }
            itendnode->second = *itdist++;
            BOOST_ASSERT(itdist==vtempdists.end());

            // splice out in-between nodes in path
            listpath.erase(itstartnode, itendnode);
            nrejected = 0;

            if( listpath.size() <= 2 ) {
                break;
            }
        }
    }

    void _SubsampleTrajectory(TrajectoryBasePtr ptraj, list< std::pair< vector<dReal>, dReal> >& listpath) const
    {
        PlannerParametersConstPtr parameters = GetParameters();
        vector<dReal> q0(parameters->GetDOF()), dq(parameters->GetDOF());
        vector<dReal> vtrajdata;
        ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajdata,parameters->_configurationspecification);

        std::copy(vtrajdata.begin(),vtrajdata.begin()+parameters->GetDOF(),q0.begin());
        listpath.push_back(make_pair(q0,dReal(0)));

        for(size_t ipoint = 1; ipoint < ptraj->GetNumWaypoints(); ++ipoint) {
            std::copy(vtrajdata.begin()+(ipoint-1)*parameters->GetDOF(),vtrajdata.begin()+(ipoint)*parameters->GetDOF(),dq.begin());
            std::copy(vtrajdata.begin()+(ipoint)*parameters->GetDOF(),vtrajdata.begin()+(ipoint+1)*parameters->GetDOF(),q0.begin());
            parameters->_diffstatefn(dq,q0);
            int i, numSteps = 1;
            vector<dReal>::const_iterator itres = parameters->_vConfigResolution.begin();
            for (i = 0; i < parameters->GetDOF(); i++,itres++) {
                int steps;
                if( *itres != 0 ) {
                    steps = (int)(fabs(dq[i]) / *itres);
                }
                else {
                    steps = (int)(fabs(dq[i]) * 100);
                }
                if (steps > numSteps) {
                    numSteps = steps;
                }
            }
            dReal fisteps = dReal(1.0f)/numSteps;
            FOREACH(it,dq) {
                *it *= fisteps;
            }
            for (int f = 0; f < numSteps; f++) {
                dReal dist = parameters->_distmetricfn(listpath.back().first,q0);
                listpath.push_back(make_pair(q0, dist));
                if( !parameters->_neighstatefn(q0,dq,0) ) {
                    RAVELOG_DEBUG("neighstatefn failed, perhaps non-linear constraints are used?\n");
                    break;
                }
            }
        }

        std::copy(vtrajdata.end()-parameters->GetDOF(),vtrajdata.end(),q0.begin());
        dReal dist = parameters->_distmetricfn(listpath.back().first,q0);
        listpath.push_back(make_pair(q0,dist));
    }

    TrajectoryTimingParametersPtr _parameters;
    RobotBasePtr _probot;
};

PlannerBasePtr CreateShortcutLinearPlanner(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new ShortcutLinearPlanner(penv, sinput));
}
