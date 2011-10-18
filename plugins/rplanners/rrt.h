// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
#ifndef  BIRRT_PLANNER_H
#define  BIRRT_PLANNER_H

#include "rplanners.h"

template <typename Node>
class RrtPlanner : public PlannerBase
{
public:

    RrtPlanner(EnvironmentBasePtr penv) : PlannerBase(penv), _treeForward(0)
    {
        __description = "\
:Interface Author:  Rosen Diankov\n\n\
Uses the Rapidly-Exploring Random Trees Algorithm.\n\
";
        RegisterCommand("GetGoalIndex",boost::bind(&RrtPlanner<Node>::GetGoalIndexCommand,this,_1,_2),
                        "returns the goal index of the plan");
    }
    virtual ~RrtPlanner() {
    }

    virtual bool _InitPlan(RobotBasePtr pbase, PlannerParametersPtr params)
    {
        _goalindex = -1;
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        _robot = pbase;

        RobotBase::RobotStateSaver savestate(_robot);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        if( (int)params->vinitialconfig.size() % params->GetDOF() ) {
            RAVELOG_ERROR(str(boost::format("initial config wrong dim: %d\n")%params->vinitialconfig.size()));
            return false;
        }

        _sampleConfig.resize(params->GetDOF());
        _treeForward.Reset(shared_planner(), params->GetDOF());
        _treeForward._fStepLength = params->_fStepLength;
        _treeForward._distmetricfn = params->_distmetricfn;
        std::vector<dReal> vinitialconfig(params->GetDOF());
        for(size_t index = 0; index < params->vinitialconfig.size(); index += params->GetDOF()) {
            std::copy(params->vinitialconfig.begin()+index,params->vinitialconfig.begin()+index+params->GetDOF(),vinitialconfig.begin());
            if( !params->_checkpathconstraintsfn(vinitialconfig,vinitialconfig,IT_OpenStart,ConfigurationListPtr()) ) {
                continue;
            }
            _treeForward.AddNode(-1, vinitialconfig);
        }

        if( _treeForward._nodes.size() == 0 && !params->_sampleinitialfn ) {
            RAVELOG_WARN("no initial configurations\n");
            return false;
        }

        return true;
    }

    // simple path optimization
    virtual void _SimpleOptimizePath(list<Node*>& path, int numiterations)
    {
        if( path.size() <= 2 ) {
            return;
        }
        PlannerParametersConstPtr params = GetParameters();

        typename list<Node*>::iterator startNode, endNode;
        ConfigurationListPtr listconfigs(new ConfigurationList());
        SpaceSamplerBasePtr psampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        std::vector<uint32_t> vindexsamples;
        int nrejected = 0;
        int i = numiterations;
        while(i > 0 && nrejected < (int)path.size()+4 ) {
            --i;

            // pick a random node on the path, and a random jump ahead
            psampler->SampleSequence(vindexsamples,2);
            int endIndex = 2+(vindexsamples.at(0)%((int)path.size()-2));
            int startIndex = vindexsamples.at(1)%(endIndex-1);

            startNode = path.begin();
            advance(startNode, startIndex);
            endNode = startNode;
            advance(endNode, endIndex-startIndex);
            nrejected++;

            // check if the nodes can be connected by a straight line
            listconfigs->clear();
            if (!params->_checkpathconstraintsfn((*startNode)->q, (*endNode)->q, IT_Open, listconfigs)) {
                if( nrejected++ > (int)path.size()+8 ) {
                    break;
                }
                continue;
            }

            ++startNode;
            FOREACHC(itc, *listconfigs) {
                path.insert(startNode, _treeForward._nodes.at(_treeForward.AddNode(-1,*itc)));
            }
            // splice out in-between nodes in path
            path.erase(startNode, endNode);
            nrejected = 0;

            if( path.size() <= 2 ) {
                return;
            }
        }
    }

    virtual RobotBasePtr GetRobot() const {
        return _robot;
    }

    bool GetGoalIndexCommand(std::ostream& os, std::istream& is)
    {
        os << _goalindex;
        return !!os;
    }

protected:
    RobotBasePtr _robot;
    std::vector<dReal>         _sampleConfig;
    int _goalindex;

    SpatialTree< RrtPlanner<Node>, Node > _treeForward;

    inline boost::shared_ptr<RrtPlanner> shared_planner() {
        return boost::static_pointer_cast<RrtPlanner>(shared_from_this());
    }
    inline boost::shared_ptr<RrtPlanner const> shared_planner_const() const {
        return boost::static_pointer_cast<RrtPlanner const>(shared_from_this());
    }
};

class BirrtPlanner : public RrtPlanner<SimpleNode>
{
public:
    BirrtPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv), _treeBackward(1)
    {
        __description += "Bi-directional RRTs. See\n\n\
- J.J. Kuffner and S.M. LaValle. RRT-Connect: An efficient approach to single-query path planning. In Proc. IEEE Int'l Conf. on Robotics and Automation (ICRA'2000), pages 995-1001, San Francisco, CA, April 2000.";
    }
    virtual ~BirrtPlanner() {
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new PlannerParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }

        RobotBase::RobotStateSaver savestate(_robot);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        _treeBackward.Reset(shared_planner(), _parameters->GetDOF());
        _treeBackward._fStepLength = _parameters->_fStepLength;
        _treeBackward._distmetricfn = _parameters->_distmetricfn;

        //read in all goals
        _numgoals = 0;
        if( (_parameters->vgoalconfig.size() % _parameters->GetDOF()) != 0 ) {
            RAVELOG_ERROR("BirrtPlanner::InitPlan - Error: goals are improperly specified:\n");
            _parameters.reset();
            return false;
        }

        vector<dReal> vgoal(_parameters->GetDOF());
        for(size_t igoal = 0; igoal < _parameters->vgoalconfig.size(); igoal += _parameters->GetDOF()) {
            std::copy(_parameters->vgoalconfig.begin()+igoal,_parameters->vgoalconfig.begin()+igoal+_parameters->GetDOF(),vgoal.begin());
            if( _parameters->_checkpathconstraintsfn(vgoal,vgoal,IT_OpenStart,ConfigurationListPtr()) ) {
                _treeBackward.AddNode(-_numgoals-1, vgoal);
                _numgoals++;
            }
            else {
                RAVELOG_WARN(str(boost::format("goal %d fails constraints\n")%igoal));
            }
        }

        if(( _numgoals == 0) && !_parameters->_samplegoalfn ) {
            RAVELOG_WARN("no goals specified\n");
            _parameters.reset();
            return false;
        }

        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 10000;
        }

        RAVELOG_DEBUG("BirrtPlanner::InitPlan - RRT Planner Initialized\n");
        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        _goalindex = -1;
        if(!_parameters) {
            RAVELOG_ERROR("BirrtPlanner::PlanPath - Error, planner not initialized\n");
            return PS_Failed;
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = GetMilliTime();

        // the main planning loop
        bool bConnected = false;

        RobotBase::RobotStateSaver savestate(_robot);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        SpatialTreeBase* TreeA = &_treeForward;
        SpatialTreeBase* TreeB = &_treeBackward;
        int iConnectedA, iConnectedB;
        int iter = 0;

        while(!bConnected && iter < 3*_parameters->_nMaxIterations) {
            RAVELOG_VERBOSE("iter: %d\n", iter);
            ++iter;

            if( !!_parameters->_samplegoalfn ) {
                vector<dReal> vgoal;
                if( _parameters->_samplegoalfn(vgoal) ) {
                    RAVELOG_VERBOSE("found goal\n");
                    _treeBackward.AddNode(-_numgoals-1,vgoal);
                    _numgoals += 1;
                }
            }
            if( !!_parameters->_sampleinitialfn ) {
                vector<dReal> vinitial;
                if( _parameters->_sampleinitialfn(vinitial) ) {
                    RAVELOG_VERBOSE("found initial\n");
                    _treeForward.AddNode(-1,vinitial);
                }
            }

            if(( iter == 1) &&( _parameters->GetDOF() <= (int)_parameters->vgoalconfig.size()) ) {
                _sampleConfig.resize(_parameters->GetDOF());
                std::copy(_parameters->vgoalconfig.begin(),_parameters->vgoalconfig.begin()+_parameters->GetDOF(),_sampleConfig.begin());
            }
            else if( !_parameters->_samplefn(_sampleConfig) ) {
                continue;
            }

            // extend A
            ExtendType et = TreeA->Extend(_sampleConfig, iConnectedA);

            // although check isn't necessary, having it improves running times
            if( et == ET_Failed ) {
                // necessary to increment iterator in case spaces are not connected
                if( iter > 3*_parameters->_nMaxIterations ) {
                    RAVELOG_WARN("iterations exceeded\n");
                    break;
                }
                continue;
            }

            et = TreeB->Extend(TreeA->GetConfig(iConnectedA), iConnectedB);     // extend B toward A

            if( et == ET_Connected ) {
                // if connected, success
                bConnected = true;
                break;
            }

            swap(TreeA, TreeB);
            iter += 3;
            if( iter > 3*_parameters->_nMaxIterations ) {
                RAVELOG_WARN("iterations exceeded\n");
                break;
            }
        }

        if( !bConnected ) {
            RAVELOG_WARN("plan failed, %fs\n",0.001f*(float)(GetMilliTime()-basetime));
            return PS_Failed;
        }

        list<SimpleNode*> vecnodes;

        // add nodes from the forward tree
        SimpleNode* pforward = _treeForward._nodes.at(TreeA == &_treeForward ? iConnectedA : iConnectedB);
        while(1) {
            vecnodes.push_front(pforward);
            if(pforward->parent < 0) {
                break;
            }
            pforward = _treeForward._nodes.at(pforward->parent);
        }

        // add nodes from the backward tree
        _goalindex = -1;

        SimpleNode *pbackward = _treeBackward._nodes.at(TreeA == &_treeBackward ? iConnectedA : iConnectedB);
        while(1) {
            vecnodes.push_back(pbackward);
            if(pbackward->parent < 0) {
                _goalindex = -pbackward->parent-1;
                break;
            }
            pbackward = _treeBackward._nodes.at(pbackward->parent);
        }

        BOOST_ASSERT( _goalindex >= 0 );
        _SimpleOptimizePath(vecnodes,10);

        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        FOREACH(itnode, vecnodes) {
            ptraj->Insert(ptraj->GetNumWaypoints(),(*itnode)->q,_parameters->_configurationspecification);
        }

        RAVELOG_DEBUG(str(boost::format("plan success, path=%d points in %fs\n")%ptraj->GetNumWaypoints()%(0.001f*(float)(GetMilliTime()-basetime))));
        return _ProcessPostPlanners(_robot,ptraj);
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

protected:
    PlannerParametersPtr _parameters;
    SpatialTree< RrtPlanner<SimpleNode>, SimpleNode > _treeBackward;
    int _numgoals;
};

class BasicRrtPlanner : public RrtPlanner<SimpleNode>
{
public:
    BasicRrtPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv)
    {
        __description = "Rosen's BiRRT planner";
        _fGoalBiasProb = 0.05f;
        _bOneStep = false;
    }
    virtual ~BasicRrtPlanner() {
    }

    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new BasicRRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        //read in all goals
        int goal_index = 0;
        vector<dReal> vgoal(_parameters->GetDOF());
        _vecGoals.resize(0);

        while(_parameters->vgoalconfig.size() > 0) {
            for(int i = 0; i < _parameters->GetDOF(); i++) {
                if(goal_index < (int)_parameters->vgoalconfig.size())
                    vgoal[i] = _parameters->vgoalconfig[goal_index];
                else {
                    RAVELOG_ERROR("BirrtPlanner::InitPlan - Error: goals are improperly specified:\n");
                    _parameters.reset();
                    return false;
                }
                goal_index++;
            }

            if( GetParameters()->_checkpathconstraintsfn(vgoal,vgoal,IT_OpenStart,ConfigurationListPtr()) ) {
                _vecGoals.push_back(vgoal);
            }
            else {
                RAVELOG_WARN("goal in collision\n");
            }

            if(goal_index == (int)_parameters->vgoalconfig.size()) {
                break;
            }
        }

        if(( _vecGoals.size() == 0) && !_parameters->_goalfn ) {
            RAVELOG_WARN("no goals or goal function specified\n");
            _parameters.reset();
            return false;
        }

        RAVELOG_DEBUG("RrtPlanner::InitPlan - RRT Planner Initialized\n");
        return true;
    }

    PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        if(!_parameters) {
            RAVELOG_WARN("RrtPlanner::PlanPath - Error, planner not initialized\n");
            return PS_Failed;
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = GetMilliTime();

        int lastnode = 0;
        bool bSuccess = false;

        // the main planning loop
        RobotBase::RobotStateSaver savestate(_robot);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        int iter = 0;
        _goalindex = -1;

        while(!bSuccess && iter < _parameters->_nMaxIterations) {
            iter++;

            if( !!_parameters->_samplegoalfn ) {
                vector<dReal> vgoal;
                if( _parameters->_samplegoalfn(vgoal) ) {
                    RAVELOG_VERBOSE("found goal\n");
                    _vecGoals.push_back(vgoal);
                }
            }
            if( !!_parameters->_sampleinitialfn ) {
                vector<dReal> vinitial;
                if( _parameters->_sampleinitialfn(vinitial) ) {
                    RAVELOG_VERBOSE("found initial\n");
                    _treeForward.AddNode(-1,vinitial);
                }
            }

            if( (( iter == 1) ||( RaveRandomFloat() < _fGoalBiasProb) ) &&( _vecGoals.size() > 0) ) {
                _sampleConfig = _vecGoals[RaveRandomInt()%_vecGoals.size()];
            }
            else if( !_parameters->_samplefn(_sampleConfig) ) {
                continue;
            }

            // extend A
            ExtendType et = _treeForward.Extend(_sampleConfig, lastnode, _bOneStep);

            if( et == ET_Connected ) {
                FOREACH(itgoal, _vecGoals) {
                    if( _treeForward._distmetricfn(*itgoal, _treeForward._nodes.at(lastnode)->q) < 2*_treeForward._fStepLength ) {
                        bSuccess = true;
                        _goalindex = (int)(itgoal-_vecGoals.begin());
                        RAVELOG_DEBUG(str(boost::format("found goal index: %d\n")%_goalindex));
                        break;
                    }
                }
            }

            // check the goal heuristic more often
            if(( et != ET_Failed) && !!_parameters->_goalfn ) {
                if( _parameters->_goalfn(_treeForward._nodes.at(lastnode)->q) <= 1e-4f ) {
                    bSuccess = true;
                    _goalindex = -1;
                    RAVELOG_DEBUG("node at goal\n");
                    break;
                }
            }

            // check if reached any goals
            if( iter > _parameters->_nMaxIterations ) {
                RAVELOG_WARN("iterations exceeded %d\n", _parameters->_nMaxIterations);
                break;
            }
        }

        if( !bSuccess ) {
            RAVELOG_DEBUG("plan failed, %fs\n",0.001f*(float)(GetMilliTime()-basetime));
            return PS_Failed;
        }

        list<SimpleNode*> vecnodes;

        // add nodes from the forward tree
        SimpleNode* pforward = _treeForward._nodes.at(lastnode);
        while(1) {
            vecnodes.push_front(pforward);
            if(pforward->parent < 0) {
                break;
            }
            pforward = _treeForward._nodes.at(pforward->parent);
        }

        _SimpleOptimizePath(vecnodes,10);
        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        FOREACH(itnode, vecnodes) {
            ptraj->Insert(ptraj->GetNumWaypoints(),(*itnode)->q,_parameters->_configurationspecification);
        }

        PlannerStatus status = _ProcessPostPlanners(_robot,ptraj);
        RAVELOG_DEBUG(str(boost::format("plan success, path=%d points in %fs\n")%ptraj->GetNumWaypoints()%((0.001f*(float)(GetMilliTime()-basetime)))));
        return status;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

protected:
    boost::shared_ptr<BasicRRTParameters> _parameters;
    float _fGoalBiasProb;
    bool _bOneStep;
    std::vector< std::vector<dReal> > _vecGoals;
};

class ExplorationPlanner : public RrtPlanner<SimpleNode>
{
public:
    class ExplorationParameters : public PlannerBase::PlannerParameters {
public:
        ExplorationParameters() : _fExploreProb(0), _nExpectedDataSize(100), _bProcessingExploration(false) {
            _vXMLParameters.push_back("exploreprob");
            _vXMLParameters.push_back("expectedsize");
        }

        dReal _fExploreProb;
        int _nExpectedDataSize;

protected:
        bool _bProcessingExploration;
        // save the extra data to XML
        virtual bool serialize(std::ostream& O) const
        {
            if( !PlannerParameters::serialize(O) ) {
                return false;
            }
            O << "<exploreprob>" << _fExploreProb << "</exploreprob>" << endl;
            O << "<expectedsize>" << _nExpectedDataSize << "</expectedsize>" << endl;
            return !!O;
        }

        ProcessElement startElement(const std::string& name, const AttributesList& atts)
        {
            if( _bProcessingExploration ) {
                return PE_Ignore;
            }
            switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
            }

            _bProcessingExploration = name=="exploreprob"||name=="expectedsize";
            return _bProcessingExploration ? PE_Support : PE_Pass;
        }

        // called at the end of every XML tag, _ss contains the data
        virtual bool endElement(const std::string& name)
        {
            // _ss is an internal stringstream that holds the data of the tag
            if( _bProcessingExploration ) {
                if( name == "exploreprob") {
                    _ss >> _fExploreProb;
                }
                else if( name == "expectedsize" ) {
                    _ss >> _nExpectedDataSize;
                }
                else {
                    RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
                }
                _bProcessingExploration = false;
                return false;
            }

            // give a chance for the default parameters to get processed
            return PlannerParameters::endElement(name);
        }
    };

    ExplorationPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nRRT-based exploration planner";
    }
    virtual ~ExplorationPlanner() {
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ExplorationParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }
        RAVELOG_DEBUG("ExplorationPlanner::InitPlan - RRT Planner Initialized\n");
        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        _goalindex = -1;
        if( !_parameters ) {
            return PS_Failed;
        }
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        vector<dReal> vSampleConfig;

        RobotBase::RobotStateSaver saver(_robot);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        int iter = 0;
        while(iter < _parameters->_nMaxIterations && (int)_treeForward._nodes.size() < _parameters->_nExpectedDataSize ) {
            ++iter;

            if( RaveRandomFloat() < _parameters->_fExploreProb ) {
                // explore
                int inode = RaveRandomInt()%_treeForward._nodes.size();
                SimpleNode* pnode = _treeForward._nodes.at(inode);

                if( !_parameters->_sampleneighfn(vSampleConfig,pnode->q,_parameters->_fStepLength) ) {
                    return PS_Failed;
                }
                if( GetParameters()->_checkpathconstraintsfn(pnode->q, vSampleConfig, IT_OpenStart,ConfigurationListPtr()) ) {
                    _treeForward.AddNode(inode,vSampleConfig);
                    GetEnv()->UpdatePublishedBodies();
                    RAVELOG_DEBUG(str(boost::format("size %d\n")%_treeForward._nodes.size()));
                }
            }
            else {     // rrt extend
                if( !_parameters->_samplefn(vSampleConfig) ) {
                    continue;
                }
                int lastindex;
                if( _treeForward.Extend(vSampleConfig,lastindex,true) == ET_Connected ) {
                    RAVELOG_DEBUG(str(boost::format("size %d\n")%_treeForward._nodes.size()));
                }
            }
        }

        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        // save nodes to trajectory
        FOREACH(itnode, _treeForward._nodes) {
            ptraj->Insert(ptraj->GetNumWaypoints(),(*itnode)->q,_parameters->_configurationspecification);
        }
        return PS_HasSolution;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

private:
    boost::shared_ptr<ExplorationParameters> _parameters;

};

#endif
