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
    
 RrtPlanner(EnvironmentBasePtr penv) : PlannerBase(penv)
    {
        __description = "\
:Interface Author:  Rosen Diankov\n\n\
Uses the Rapidly-Exploring Random Trees Algorithm.\n\
";
        _report.reset(new CollisionReport());
    }
    virtual ~RrtPlanner() {}

    virtual bool _InitPlan(RobotBasePtr pbase, PlannerParametersPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        
        _robot = pbase;

        RobotBase::RobotStateSaver savestate(_robot);
        if( (int)params->vinitialconfig.size() % params->GetDOF() ) {
            RAVELOG_ERROR(str(boost::format("initial config wrong dim: %d\n")%params->vinitialconfig.size()));
            return false;
        }

        _sampleConfig.resize(params->GetDOF());
        _treeForward.Reset(shared_planner(), params->GetDOF());;
        _treeForward._fStepLength = params->_fStepLength;
        _treeForward._distmetricfn = params->_distmetricfn;
        std::vector<dReal> vinitialconfig(params->GetDOF());
        for(size_t index = 0; index < params->vinitialconfig.size(); index += params->GetDOF()) {
            std::copy(params->vinitialconfig.begin()+index,params->vinitialconfig.begin()+index+params->GetDOF(),vinitialconfig.begin());
            if(CollisionFunctions::CheckCollision(params,_robot,vinitialconfig, _report)) {
                RAVELOG_DEBUG(str(boost::format("initial configuration %d in collision")%(index/params->GetDOF())));
                continue;
            }
    
            // set up the initial state
            if( !!params->_constraintfn ) {
                params->_setstatefn(vinitialconfig);
                if( !params->_constraintfn(vinitialconfig, vinitialconfig,0) ) {
                    RAVELOG_WARN(str(boost::format("initial state %d rejected by constraint fn")%(index/params->GetDOF())));
                    continue;
                }
            }

            _treeForward.AddNode(-1, vinitialconfig);
        }

        return _treeForward._nodes.size()>0;
    }

    // simple path optimization
    virtual void _SimpleOptimizePath(list<Node*>& path, int numiterations)
    {
        if( path.size() <= 2 ) {
            return;
        }
        PlannerParametersConstPtr params = GetParameters();

        typename list<Node*>::iterator startNode, endNode;
        vector< vector<dReal> > vconfigs;

        int nrejected = 0;
        int i = numiterations;
        while(i > 0 && nrejected < (int)path.size()+4 ) {
            --i;

            // pick a random node on the path, and a random jump ahead
            int endIndex = 2+(RaveRandomInt()%((int)path.size()-2));
            int startIndex = RaveRandomInt()%(endIndex-1);
        
            startNode = path.begin();
            advance(startNode, startIndex);
            endNode = startNode;
            advance(endNode, endIndex-startIndex);
            nrejected++;

            // check if the nodes can be connected by a straight line
            vconfigs.resize(0);
            if (CollisionFunctions::CheckCollision(params,_robot,(*startNode)->q, (*endNode)->q, IT_Open, &vconfigs)) {
                if( nrejected++ > (int)path.size()+8 ) {
                    break;
                }
                continue;
            }

            ++startNode;
            FOREACHC(itc, vconfigs) {
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

    virtual RobotBasePtr GetRobot() const { return _robot; }
protected:
    RobotBasePtr         _robot;

    std::vector<dReal>         _sampleConfig;
    CollisionReportPtr _report;

    SpatialTree< RrtPlanner<Node>, Node > _treeForward;

    inline boost::shared_ptr<RrtPlanner> shared_planner() { return boost::static_pointer_cast<RrtPlanner>(shared_from_this()); }
    inline boost::shared_ptr<RrtPlanner const> shared_planner_const() const { return boost::static_pointer_cast<RrtPlanner const>(shared_from_this()); }
};

class BirrtPlanner : public RrtPlanner<SimpleNode>
{
 public:
 BirrtPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv)
    {
        __description += "Bi-directional RRTs. See\n\n\
- J.J. Kuffner and S.M. LaValle. RRT-Connect: An efficient approach to single-query path planning. In Proc. IEEE Int'l Conf. on Robotics and Automation (ICRA'2000), pages 995-1001, San Francisco, CA, April 2000.";
    }
    virtual ~BirrtPlanner() {}

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

        _treeBackward.Reset(shared_planner(), _parameters->GetDOF());
        _treeBackward._fStepLength = _parameters->_fStepLength;
        _treeBackward._distmetricfn = _parameters->_distmetricfn;
    
        //read in all goals
        int goal_index = 0;
        int num_goals = 0;
        vector<dReal> vgoal(_parameters->GetDOF());

        while(goal_index < (int)_parameters->vgoalconfig.size()) {
            for(int i = 0 ; i < _parameters->GetDOF(); i++) {
                if(goal_index < (int)_parameters->vgoalconfig.size())
                    vgoal[i] = _parameters->vgoalconfig[goal_index];
                else {
                    RAVELOG_ERROR("BirrtPlanner::InitPlan - Error: goals are improperly specified:\n");
                    _parameters.reset();
                    return false;
                }
                goal_index++;
            }
        
            if(!CollisionFunctions::CheckCollision(_parameters,_robot,vgoal)) {

                bool bSuccess = true;     
                if( !!_parameters->_constraintfn ) {
                    // filter
                    if( !_parameters->_constraintfn(vgoal, vgoal, 0) ) {
                        // failed
                        stringstream ss; ss << "goal state rejected by constraint fn: ";
                        FOREACH(it,vgoal) {
                            ss << *it << " ";
                        }
                        ss << endl;
                        RAVELOG_WARN(ss.str());
                        //_parameters->_constraintfn(vgoal, vgoal, 0);
                        bSuccess = false;
                    }
                }
            
                if( bSuccess ) {
                    // set up the goal states
                    _treeBackward.AddNode(-num_goals-1, vgoal);
                    num_goals++;
                }
            }
            else {
                RAVELOG_WARN("goal in collision %s\n", _robot->CheckSelfCollision()?"(self)":NULL);
                if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot), _report) || _robot->CheckSelfCollision(_report) ) {
                    RAVELOG_WARN(str(boost::format("birrt: robot initially in collision %s!\n")%_report->__str__()));
                }
            }
        }
    
        if( num_goals == 0 && !_parameters->_samplegoalfn ) {
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

    /// \param pOutStream returns which goal was chosen
    virtual bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream)
    {
        if(!_parameters) {
            RAVELOG_ERROR("BirrtPlanner::PlanPath - Error, planner not initialized\n");
            return false;
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

            if( !!_parameters->_samplegoalfn ) {//  _treeBackward._nodes.size() == 0 ) {
                vector<dReal> vgoal;
                if( _parameters->_samplegoalfn(vgoal) ) {
                    RAVELOG_VERBOSE("found goal\n");
                    _treeBackward.AddNode(-10000,vgoal);
                }
            }

            if( iter == 1 && _parameters->GetDOF() <= (int)_parameters->vgoalconfig.size() ) {
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

            // extend B toward A
            et = TreeB->Extend(TreeA->GetConfig(iConnectedA), iConnectedB);
            //GetEnv()->UpdatePublishedBodies();
            // if connected, success
            if( et == ET_Connected ) {
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
            return false;
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
        int goalindex = -1;

        SimpleNode *pbackward = _treeBackward._nodes.at(TreeA == &_treeBackward ? iConnectedA : iConnectedB);
        while(1) {
            vecnodes.push_back(pbackward);
            if(pbackward->parent < 0) {
                goalindex = -pbackward->parent-1;
                break;
            }
            pbackward = _treeBackward._nodes.at(pbackward->parent);
        }

        BOOST_ASSERT( goalindex >= 0 );
        if( pOutStream != NULL ) {
            *pOutStream << goalindex;
        }
        _SimpleOptimizePath(vecnodes,10);
    
        Trajectory::TPOINT pt; pt.q.resize(_parameters->GetDOF());
    
        FOREACH(itnode, vecnodes) {
            for(int i = 0; i < _parameters->GetDOF(); ++i) {
                pt.q[i] = (*itnode)->q[i];
            }
            ptraj->AddPoint(pt);
        }

        RAVELOG_DEBUG(str(boost::format("plan success, path=%d points in %fs\n")%ptraj->GetPoints().size()%(0.001f*(float)(GetMilliTime()-basetime))));
        return _OptimizePath(_robot,ptraj);
    }

    virtual PlannerParametersConstPtr GetParameters() const { return _parameters; }

 protected:
    PlannerParametersPtr _parameters;
    SpatialTree< RrtPlanner<SimpleNode>, SimpleNode > _treeBackward;
};

class BasicRrtPlanner : public RrtPlanner<SimpleNode>
{
 public:
 BasicRrtPlanner(EnvironmentBasePtr penv) : RrtPlanner<SimpleNode>(penv)
    {
        __description = "Rosen's BiRRT planner";
        _fGoalBiasProb = 0.05f;
        _bOneStep = true;
    }
    virtual ~BasicRrtPlanner() {}

    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new BasicRRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<SimpleNode>::_InitPlan(pbase,_parameters) ) {
            _parameters.reset();
            return false;
        }
        //_bOneStep = parameters->vnParameters[0]>0;
    
        //read in all goals
        int goal_index = 0;
        vector<dReal> vgoal(_parameters->GetDOF());
        _vecGoals.resize(0);

        while(_parameters->vgoalconfig.size() > 0) {
            for(int i = 0 ; i < _parameters->GetDOF(); i++) {
                if(goal_index < (int)_parameters->vgoalconfig.size())
                    vgoal[i] = _parameters->vgoalconfig[goal_index];
                else {
                    RAVELOG_ERROR("BirrtPlanner::InitPlan - Error: goals are improperly specified:\n");
                    _parameters.reset();
                    return false;
                }
                goal_index++;
            }
        
            if(!CollisionFunctions::CheckCollision(GetParameters(),_robot,vgoal)) {

                bool bSuccess = true;     
                if( !!_parameters->_constraintfn ) {
                    // filter
                    if( !_parameters->_constraintfn(vgoal, vgoal, 0) ) {
                        // failed
                        RAVELOG_WARN("goal state rejected by constraint fn\n");
                        bSuccess = false;
                    }
                }
            
                if( bSuccess ) {
                    // set up the goal states
                    _vecGoals.push_back(vgoal);
                }
            }
            else {
                RAVELOG_WARN("goal in collision %s\n", _robot->CheckSelfCollision()?"(self)":NULL);
                if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot), _report) || _robot->CheckSelfCollision(_report)) {
                    RAVELOG_WARN(str(boost::format("birrt: robot initially in collision %s!\n")%_report->__str__()));
                }
            }
        
            if(goal_index == (int)_parameters->vgoalconfig.size()) {
                break;
            }
        }
        
        if( _vecGoals.size() == 0 && !_parameters->_goalfn ) {
            RAVELOG_WARN("no goals or goal function specified\n");
            _parameters.reset();
            return false;
        }
        
        RAVELOG_DEBUG("RrtPlanner::InitPlan - RRT Planner Initialized\n");
        return true;
    }

    bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream)
    {
        if(!_parameters) {
            RAVELOG_WARN("RrtPlanner::PlanPath - Error, planner not initialized\n");
            return false;
        }
    
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = GetMilliTime();

        int lastnode = 0;    
        bool bSuccess = false;

        // the main planning loop 
        RobotBase::RobotStateSaver savestate(_robot);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        int iter = 0;
        int igoalindex = -1;

        while(!bSuccess && iter < _parameters->_nMaxIterations) {
            iter++;

            if( !!_parameters->_samplegoalfn ) {
                vector<dReal> vgoal;
                if( _parameters->_samplegoalfn(vgoal) ) {
                    RAVELOG_VERBOSE("found goal\n");
                    _vecGoals.push_back(vgoal);
                }
            }

            if( (iter == 1 || RaveRandomFloat() < _fGoalBiasProb) && _vecGoals.size() > 0 ) {
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
                        igoalindex = (int)(itgoal-_vecGoals.begin());
                        break;
                    }
                }
            }

            // check the goal heuristic more often
            if( et != ET_Failed && !!_parameters->_goalfn ) {
                if( _parameters->_goalfn(_treeForward._nodes.at(lastnode)->q) <= 1e-4f ) {
                    bSuccess = true;
                    igoalindex = 0;
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
            return false;
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
        
        BOOST_ASSERT( igoalindex >= 0 );
        if( pOutStream != NULL ) {
            *pOutStream << igoalindex;
        }
        Trajectory::TPOINT pt; pt.q.resize(_parameters->GetDOF());
        FOREACH(itnode, vecnodes) {
            for(int i = 0; i < _parameters->GetDOF(); ++i) {
                pt.q[i] = (*itnode)->q[i];
            }
            ptraj->AddPoint(pt);
        }

        bSuccess = _OptimizePath(_robot,ptraj);
        RAVELOG_DEBUG(str(boost::format("plan success, path=%d points in %fs\n")%ptraj->GetPoints().size()%((0.001f*(float)(GetMilliTime()-basetime)))));
    
        return bSuccess;
    }

    virtual PlannerParametersConstPtr GetParameters() const { return _parameters; }
    
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
    virtual ~ExplorationPlanner() {}

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

    virtual bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream)
    {
        if( !_parameters ) {
            return false;
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
                    return false;
                }
                if( !!_parameters->_constraintfn ) {
                    _parameters->_setstatefn(vSampleConfig);
                    if( !_parameters->_constraintfn(pnode->q, vSampleConfig, 0) ) {
                        continue;
                    }
                }

                if( !CollisionFunctions::CheckCollision(GetParameters(),_robot,pnode->q, vSampleConfig, IT_OpenStart) ) {
                    _treeForward.AddNode(inode,vSampleConfig);
                    GetEnv()->UpdatePublishedBodies();
                    RAVELOG_DEBUG(str(boost::format("size %d\n")%_treeForward._nodes.size()));
                }
            }
            else { // rrt extend
                if( !_parameters->_samplefn(vSampleConfig) ) {
                    continue;
                }
                int lastindex;
                if( _treeForward.Extend(vSampleConfig,lastindex,true) == ET_Connected ) {
                    RAVELOG_DEBUG(str(boost::format("size %d\n")%_treeForward._nodes.size()));
                }
            }
        }
    
        // save nodes to trajectory
        FOREACH(itnode, _treeForward._nodes) {
            ptraj->AddPoint(Trajectory::TPOINT((*itnode)->q,0));
        }
        return true;
    }

    virtual PlannerParametersConstPtr GetParameters() const { return _parameters; }
        
private:
    boost::shared_ptr<ExplorationParameters> _parameters;
    
};

#endif
