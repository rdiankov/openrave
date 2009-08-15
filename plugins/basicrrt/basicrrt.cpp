// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
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

#include "basicrrt.h"

#ifndef _WIN32
#include <alloca.h>
#define _alloca alloca
#endif

const int   NUM_OPT_ITERATIONS = 100;  // optimization iterations

RrtPlanner::SpatialTree::SpatialTree()
{
    _planner = NULL;
    _fStepLength = 0.04f;
    _dof = 0;
    _fBestDist = 0;
    _pDistMetric = NULL;
    _nodes.reserve(5000);
}

void RrtPlanner::SpatialTree::Reset(RrtPlanner* planner, int dof)
{
    _planner = planner;

    vector<Node*>::iterator it;
    FORIT(it, _nodes) {
        (*it)->~Node();
        free(*it);
    }
    _nodes.resize(0);
    if( dof > 0 ) {
        _vNewConfig.resize(dof);
        _dof = dof;
    }   
}

int RrtPlanner::SpatialTree::AddNode(int parent, const dReal* pfConfig)
{
    assert( pfConfig != NULL );

    void* pmem = malloc(sizeof(Node)+sizeof(dReal)*_dof);
    Node* p = ::new(pmem) Node(); // call constructor explicitly
    p->parent = parent;
    memcpy(p->q, pfConfig, sizeof(dReal)*_dof);
    _nodes.push_back(p);
    return (int)_nodes.size()-1;
}

int RrtPlanner::SpatialTree::GetNN(const dReal* q)
{
    assert( _pDistMetric!= NULL );
    if( _nodes.size() == 0 )
        return -1;

    vector<Node*>::iterator itnode = _nodes.begin();
    int ibest = -1;
    float fbest = 0;

    while(itnode != _nodes.end()) {

        float f = _pDistMetric->Eval(q, (*itnode)->q);
        if( ibest < 0 || f < fbest ) {
            ibest = (int)(itnode-_nodes.begin());
            fbest = f;
        }
        ++itnode;
    }

    _fBestDist = fbest;
    return ibest;
}

RrtPlanner::SpatialTree::ExtendType RrtPlanner::SpatialTree::Extend(const dReal* pTargetConfig, bool bOneStep, int& lastindex)
{
    assert( pTargetConfig != NULL );

    dReal* pNewConfig = &_vNewConfig[0];

    // get the nearest neighbor
    lastindex = GetNN(pTargetConfig);
    assert(lastindex >= 0 );
    Node* pnode = _nodes[lastindex];
    bool bHasAdded = false;

    // extend
    while(1) {
        float fdist = _pDistMetric->Eval(pnode->q, pTargetConfig);

        if( fdist > _fStepLength ) fdist = _fStepLength / fdist;
        else {
            return ET_Connected;
        }
        
        for(int i = 0; i < _dof; ++i)
            pNewConfig[i] = pnode->q[i] + (pTargetConfig[i]-pnode->q[i])*fdist;
        
        // project to constraints
        if( _planner->_parameters.pconstraintfn != NULL ) {
            if( !_planner->_parameters.pconstraintfn->Constraint(pnode->q, pNewConfig, NULL, 0) ) {
                if(bHasAdded)
                    return ET_Sucess;
                return ET_Failed;
            }
        }

        if( _planner->_CheckCollision(pnode->q, pNewConfig, OPEN_START) ) {
            if(bHasAdded)
                return ET_Sucess;
            return ET_Failed;
        }

        lastindex = AddNode(lastindex, pNewConfig);
        pnode = _nodes[lastindex];
        bHasAdded = true;
        if( bOneStep )
            return ET_Connected;
    }
    
    return ET_Failed;
}

RrtPlanner::RrtPlanner(EnvironmentBase* penv) : PlannerBase(penv)
{
    _pRobot = NULL;
    bInit = false;
    _fGoalBiasProb = 0.05f;
    _bOneStep = false;
}

RrtPlanner::~RrtPlanner()
{
}

bool RrtPlanner::InitPlan(RobotBase* pbase, const PlannerParameters* pparams)
{
    RAVELOG_DEBUGA("Initializing Planner\n");
    if( pparams != NULL )
        _parameters = *pparams;
    else {
        RAVELOG_ERRORA("BiRRT::InitPlan - Error: No parameters passed in to initialization");
        return false;
    }

    _pRobot = pbase;
    if( _pRobot == NULL )
        return false;

    RobotBase::RobotStateSaver savestate(_pRobot);

    if( _parameters.pdistmetric == NULL ) {
        RAVELOG_DEBUGA("RrtPlanner: Using Default Distance Function\n");
        _parameters.pdistmetric = &_defaultdist;
    }

    if( _parameters.pSampleFn == NULL ) {
        RAVELOG_DEBUGA("using default sampling function\n");
        _parameters.pSampleFn = &_defaultsamplefn;
        _defaultsamplefn.Init(_pRobot);
    }

    if( _parameters.pConfigState == NULL ) {
        _parameters.pConfigState = &_defaultstate;
    }

    _parameters.pConfigState->SetRobot(_pRobot);
    _parameters.pdistmetric->SetRobot(_pRobot);
    
    if( (int)_parameters.vinitialconfig.size() != GetDOF() ) {
        RAVELOG_ERRORA("initial config wrong dim: %"PRIdS"\n", _parameters.vinitialconfig.size());
        return false;
    }

    if(_CheckCollision(&_parameters.vinitialconfig[0], true)) {
        RAVELOG_ERRORA("RrtPlanner::InitPlan - Error: Initial configuration in collision\n");
        return false;
    }

    _tree.Reset(this, GetDOF());
    
    _tree._fStepLength = 0.01f;
    if( _parameters.vParameters.size() > 0 )
        _tree._fStepLength = _parameters.vParameters[0];
    if( _parameters.vnParameters.size() > 0 )
        _bOneStep = _parameters.vnParameters[0]>0;
    
    _tree._pDistMetric = _parameters.pdistmetric;

    // invert for speed
    _jointResolutionInv.resize(GetDOF());
    FOREACH(it, _jointResolutionInv)
        *it = 50.0f;

    //_pRobot->GetActiveDOFResolutions(_jointResolution);
//    _jointResolutionInv.resize(0);
//    FOREACH(itj, _jointResolution) {
//        _jointResolutionInv.push_back(*itj != 0  ? 1 / *itj : 1.0f);
//    }
            
    // initialize the joint limits
    _parameters.pConfigState->GetLimits(_lowerLimit, _upperLimit);
    
    _randomConfig.resize(GetDOF());
    _validRange.resize(GetDOF());
    _jointIncrement.resize(GetDOF());
    
    for (int i = 0; i < GetDOF(); i++) {
        _validRange[i] = _upperLimit[i] - _lowerLimit[i];
        assert(_validRange[i] > 0);
    }
        
    // set up the initial state
    if( _parameters.pconstraintfn != NULL ) {
        // filter
        _parameters.pConfigState->SetState(&_parameters.vinitialconfig[0]);
        if( !_parameters.pconstraintfn->Constraint(&_parameters.vinitialconfig[0], &_parameters.vinitialconfig[0],NULL,0) ) {
            // failed
            RAVEPRINT(L"initial state rejected by constraint fn\n");
            return false;
        }
    }

    _tree.AddNode(-1, &_parameters.vinitialconfig[0]);
    
    //read in all goals
    int goal_index = 0;
    vector<dReal> vgoal(GetDOF());
    _vecGoals.resize(0);

    while(_parameters.vgoalconfig.size() > 0) {
        for(int i = 0 ; i < GetDOF(); i++) {
            if(goal_index < (int)_parameters.vgoalconfig.size())
                vgoal[i] = _parameters.vgoalconfig[goal_index];
            else {
                RAVELOG_ERRORA("RrtPlanner::InitPlan - Error: goals are improperly specified:\n");
                return false;
            }
            goal_index++;
        }
        
        if(!_CheckCollision(&vgoal[0])) {

            bool bSuccess = true;     
            if( _parameters.pconstraintfn != NULL ) {
                // filter
                if( !_parameters.pconstraintfn->Constraint(&vgoal[0], &vgoal[0], NULL, 0) ) {
                    // failed
                    RAVELOG_WARNA("goal state rejected by constraint fn\n");
                    bSuccess = false;
                }
            }
            
            if( bSuccess ) {
                // set up the goal states
                _vecGoals.push_back(vgoal);
            }
        }
        else {
            RAVELOG_WARNA("goal in collision %s\n", _pRobot->CheckSelfCollision()?"(self)":NULL);

            COLLISIONREPORT report;
            if( GetEnv()->CheckCollision(_pRobot, &report) ) {
                RAVELOG_WARNA("birrt: robot initially in collision %S:%S!\n",
                          report.plink1!=NULL?report.plink1->GetName():L"(NULL)",
                          report.plink2!=NULL?report.plink2->GetName():L"(NULL)");
            }
            
        }
        
        if(goal_index == (int)_parameters.vgoalconfig.size())
            break;
    }
    
    if( _vecGoals.size() == 0 && _parameters.pgoalfn == NULL ) {
        RAVEPRINT(L"no goals or goal function specified\n");
        return false;
    }
        
    bInit = true;
    RAVELOG_DEBUGA("RrtPlanner::InitPlan - RRT Planner Initialized\n");
    return true;
}

bool RrtPlanner::PlanPath(Trajectory* ptraj, std::ostream* pOutStream)
{
    if(!bInit) {
        RAVEPRINT(L"RrtPlanner::PlanPath - Error, planner not initialized\n");
        return false;
    }
    
    uint32_t basetime = timeGetTime();

    int lastnode = 0;    
    bool bSuccess = false;

    // the main planning loop 
    RobotBase::RobotStateSaver savestate(_pRobot);

    int iter = 0;
    int igoalindex = -1;

    if( _parameters.nMaxIterations <= 0 )
        _parameters.nMaxIterations = 10000;
    
    while(!bSuccess) {
        
        if( GetEnv()->RandomFloat() < _fGoalBiasProb && _vecGoals.size() > 0 )
            _randomConfig = _vecGoals[GetEnv()->RandomInt()%_vecGoals.size()];
        else
            _parameters.pSampleFn->Sample(&_randomConfig[0]);

        // extend A
        SpatialTree::ExtendType et = _tree.Extend(&_randomConfig[0], _bOneStep, lastnode);

        if( et == SpatialTree::ET_Connected ) {
            FOREACH(itgoal, _vecGoals) {
                if( _tree._pDistMetric->Eval(&(*itgoal)[0], _tree._nodes[lastnode]->q) < 2*_tree._fStepLength ) {
                    bSuccess = true;
                    igoalindex = (int)(itgoal-_vecGoals.begin());
                    break;
                }
            }
        }

        // check the goal heuristic more often
        if( et != SpatialTree::ET_Failed && _parameters.pgoalfn != NULL ) {
            if( _parameters.pgoalfn->Eval(_tree._nodes[lastnode]->q) <= _parameters.pgoalfn->GetGoalThresh() ) {
                bSuccess = true;
                igoalindex = 0;
                break;
            }
        }
        
        // check if reached any goals
        if( iter++ > _parameters.nMaxIterations ) {
            RAVEPRINT(L"iterations exceeded %d\n", _parameters.nMaxIterations);
            break;
        }
    }
    
    if( !bSuccess ) {
        RAVELOG_DEBUGA("plan failed, %fs\n",0.001f*(float)(timeGetTime()-basetime));
        return false;
    }
    
    list<Node*> vecnodes;
    
    // add nodes from the forward tree
    Node* pforward = _tree._nodes[lastnode];
    while(1) {
        vecnodes.push_front(pforward);
        if(pforward->parent < 0)
            break;
        pforward = _tree._nodes[pforward->parent];
    }

    _OptimizePath(vecnodes);
    
    assert( igoalindex >= 0 );
    if( pOutStream != NULL )
        *pOutStream << igoalindex;

    Trajectory::TPOINT pt; pt.q.resize(GetDOF());
    FOREACH(itnode, vecnodes) {
        for(int i = 0; i < GetDOF(); ++i)
            pt.q[i] = (*itnode)->q[i];
        ptraj->AddPoint(pt);
    }

    RAVELOG_DEBUGA("plan success, path=%"PRIdS" points in %fs\n", ptraj->GetPoints().size(), 0.001f*(float)(timeGetTime()-basetime));
    
    return true;
}

bool RrtPlanner::_CheckCollision(const dReal* pq, bool bReport)
{
    _parameters.pConfigState->SetState(pq);
    COLLISIONREPORT report;

    bool bCol = GetEnv()->CheckCollision(_pRobot, bReport?&report:NULL) || _pRobot->CheckSelfCollision(bReport?&report:NULL);
    if( bCol && bReport ) {
        RAVELOG_WARNA("fcollision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
    }
    return bCol;
}

bool RrtPlanner::_CheckCollision(const dReal* pQ0, const dReal* pQ1, IntervalType interval, vector< vector<dReal> >* pvCheckedConfigurations)
{
    // set the bounds based on the interval type
    int start;
    bool bCheckEnd;
    switch (interval) {
        case OPEN:
            start = 1;  bCheckEnd = false;
            break;
        case OPEN_START:
            start = 1;  bCheckEnd = true;
            break;
        case OPEN_END:
            start = 0;  bCheckEnd = false;
            break;
        case CLOSED:
            start = 0;  bCheckEnd = true;
            break;
        default:
            assert(0);
    }

    // first make sure the end is free
    static vector<dReal> vtempconfig;
    vtempconfig.resize(GetDOF());

    if (bCheckEnd) {
        memcpy(&vtempconfig[0], pQ1,sizeof(pQ1[0])*vtempconfig.size());
        if( pvCheckedConfigurations != NULL )
            pvCheckedConfigurations->push_back(vtempconfig);
        _parameters.pConfigState->SetState(&vtempconfig[0]);
        if (GetEnv()->CheckCollision(_pRobot) || _pRobot->CheckSelfCollision() )
            return true;
    }

    // compute  the discretization
    int i, numSteps = 1;
    dReal* pfresolution = &_jointResolutionInv[0];
    for (i = 0; i < (int)vtempconfig.size(); i++) {
        int steps = (int)(fabs(pQ1[i] - pQ0[i]) * pfresolution[i]);
        if (steps > numSteps)
            numSteps = steps;
    }

    // compute joint increments
    for (i = 0; i < (int)vtempconfig.size(); i++)
        _jointIncrement[i] = (pQ1[i] - pQ0[i])/((float)numSteps);

    // check for collision along the straight-line path
    // NOTE: this does not check the end config, and may or may
    // not check the start based on the value of 'start'
    for (int f = start; f < numSteps; f++) {

        for (i = 0; i < (int)vtempconfig.size(); i++)
            vtempconfig[i] = pQ0[i] + (_jointIncrement[i] * f);
        
        if( pvCheckedConfigurations != NULL )
            pvCheckedConfigurations->push_back(vtempconfig);
        _parameters.pConfigState->SetState(&vtempconfig[0]);
        if( GetEnv()->CheckCollision(_pRobot) || _pRobot->CheckSelfCollision() )
            return true;
    }

    return false;
}

void RrtPlanner::_OptimizePath(list<Node*>& path)
{
    if( path.size() <= 2 )
        return;

    list<Node*>::iterator startNode, endNode;
    vector< vector<dReal> > vconfigs;

    int nrejected = 0;
    int i = NUM_OPT_ITERATIONS;
    while(i > 0 && nrejected < (int)path.size()+4 ) {
        --i;

        // pick a random node on the path, and a random jump ahead
        int endIndex = 2+RANDOM_INT((int)path.size()-2);
        int startIndex = RANDOM_INT(endIndex-1);
        
        startNode = path.begin();
        advance(startNode, startIndex);
        endNode = startNode;
        advance(endNode, endIndex-startIndex);
        nrejected++;

        // check if the nodes can be connected by a straight line
        vconfigs.resize(0);
        if (_CheckCollision((*startNode)->q, (*endNode)->q, OPEN, &vconfigs)) {

            if( nrejected++ > (int)path.size()+16 )
                break;
            continue;
        }

        ++startNode;
        FOREACHC(itc, vconfigs)
            path.insert(startNode, _tree._nodes[_tree.AddNode(-1,&(*itc)[0])]);

        // splice out in-between nodes in path
        path.erase(startNode, endNode);
        nrejected = 0;

        if( path.size() <= 2 )
            return;
    }
}
