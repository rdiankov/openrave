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

#include "rbirrt.h"

#ifndef _WIN32
#include <alloca.h>
#define _alloca alloca
#endif

const int   NUM_OPT_ITERATIONS = 100;  // optimization iterations

BirrtPlanner::SpatialTree::SpatialTree()
{
    _planner = NULL;
    _fStepLength = 0.04f;
    _dof = 0;
    _fBestDist = 0;
    _pDistMetric = NULL;
    _nodes.reserve(5000);
}

void BirrtPlanner::SpatialTree::Reset(BirrtPlanner* planner, int dof)
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

int BirrtPlanner::SpatialTree::AddNode(int parent, const dReal* pfConfig)
{
    assert( pfConfig != NULL );

    void* pmem = malloc(sizeof(Node)+sizeof(dReal)*_dof);
    Node* p = ::new(pmem) Node(); // call constructor explicitly
    p->parent = parent;
    memcpy(p->q, pfConfig, sizeof(dReal)*_dof);
    _nodes.push_back(p);
    return (int)_nodes.size()-1;
}

int BirrtPlanner::SpatialTree::GetNN(const dReal* q)
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

BirrtPlanner::SpatialTree::ExtendType BirrtPlanner::SpatialTree::Extend(const dReal* pTargetConfig, int& lastindex)
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
    }
    
    return ET_Failed;
}

BirrtPlanner::BirrtPlanner(EnvironmentBase* penv) : PlannerBase(penv)
{
    _pRobot = NULL;
    bInit = false;
}

BirrtPlanner::~BirrtPlanner()
{
}

bool BirrtPlanner::InitPlan(RobotBase* pbase, const PlannerParameters* pparams)
{
    RAVELOG(L"Initializing Planner\n");
    if( pparams != NULL )
        _parameters = *pparams;
    else {
        RAVELOGA("BiRRT::InitPlan - Error: No parameters passed in to initialization");
        return false;
    }

    _pRobot = pbase;
    if( _pRobot == NULL )
        return false;

    RobotBase::RobotStateSaver savestate(_pRobot);
    
    if( (int)_parameters.vinitialconfig.size() != _pRobot->GetActiveDOF() ) {
        RAVELOG_ERRORA("initial config wrong dim: %"PRIdS"\n", _parameters.vinitialconfig.size());
        return false;
    }

    if(_CheckCollision(&_parameters.vinitialconfig[0], true)) {
        RAVELOG(L"BirrtPlanner::InitPlan - Error: Initial configuration in collision\n");
        return false;
    }
    
    if( _parameters.pdistmetric == NULL ) {
        RAVELOG(L"RrtPlanner: Using Default Distance Function\n");
        _parameters.pdistmetric = &_defaultdist;
    }

    if( _parameters.pSampleFn == NULL ) {
        RAVELOG(L"using default sampling function\n");
        _parameters.pSampleFn = &_defaultsamplefn;
        _defaultsamplefn.Init(pbase);
    }

    _parameters.pdistmetric->SetRobot(pbase);
    
    _treeForward.Reset(this, _pRobot->GetActiveDOF());
    _treeBackward.Reset(this, _pRobot->GetActiveDOF());
    
    _treeForward._fStepLength = 0.01f;
    if( _parameters.vParameters.size() > 0 )
        _treeForward._fStepLength = _parameters.vParameters[0];
    
    _treeBackward._fStepLength = _treeForward._fStepLength;
    _treeForward._pDistMetric = _parameters.pdistmetric;
    _treeBackward._pDistMetric = _parameters.pdistmetric;

    // invert for speed
    _pRobot->GetActiveDOFResolutions(_jointResolution);
    _jointResolutionInv.resize(0);
    FOREACH(itj, _jointResolution) {
        if( *itj != 0 )
            _jointResolutionInv.push_back(1 / *itj);
        else {
            RAVELOG_WARNA("resolution is 0!\n");
            _jointResolutionInv.push_back(100);
        }
    }
            
    // initialize the joint limits
    _pRobot->GetActiveDOFLimits(_lowerLimit,_upperLimit);
    
    _randomConfig.resize(_pRobot->GetActiveDOF());
    _validRange.resize(_pRobot->GetActiveDOF());
    _jointIncrement.resize(_pRobot->GetActiveDOF());
    
    for (int i = 0; i < _pRobot->GetActiveDOF(); i++) {
        _validRange[i] = _upperLimit[i] - _lowerLimit[i];
        assert(_validRange[i] > 0);
    }
        
    // set up the initial state
    if( _parameters.pconstraintfn != NULL ) {
        // filter
        _pRobot->SetActiveDOFValues(NULL, &_parameters.vinitialconfig[0]);
        if( !_parameters.pconstraintfn->Constraint(&_parameters.vinitialconfig[0], &_parameters.vinitialconfig[0],NULL,0) ) {
            // failed
            RAVELOG_WARNA("initial state rejected by constraint fn\n");
            return false;
        }
    }

    _treeForward.AddNode(-1, &_parameters.vinitialconfig[0]);
    
    //read in all goals
    int goal_index = 0;
    int num_goals = 0;
    vector<dReal> vgoal(_pRobot->GetActiveDOF());

    while(1) {
        for(int i = 0 ; i < _pRobot->GetActiveDOF(); i++) {
            if(goal_index < (int)_parameters.vgoalconfig.size())
                vgoal[i] = _parameters.vgoalconfig[goal_index];
            else {
                RAVELOG_ERRORA("BirrtPlanner::InitPlan - Error: goals are improperly specified:\n");
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
                _treeBackward.AddNode(-num_goals-1, &vgoal[0]);
                num_goals++;
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
    
    if( num_goals == 0 ) {
        RAVELOG_WARNA("no goals specified\n");
        return false;
    }    
        
    bInit = true;
    RAVELOG(L"RrtPlanner::InitPlan - RRT Planner Initialized\n");
    return true;
}

bool BirrtPlanner::PlanPath(Trajectory* ptraj, std::ostream* pOutStream)
{
    if(!bInit) {
        RAVELOG_ERRORA("BirrtPlanner::PlanPath - Error, planner not initialized\n");
        return false;
    }
    
    uint32_t basetime = timeGetTime();

    // the main planning loop
    bool bConnected = false;
 
    RobotBase::RobotStateSaver savestate(_pRobot);

    SpatialTree* TreeA = &_treeForward;
    SpatialTree* TreeB = &_treeBackward;
    int iConnectedA, iConnectedB;
    int iter = 0;

    if( _parameters.nMaxIterations <= 0 )
        _parameters.nMaxIterations = 10000;
    
    while(!bConnected) {
        
        RAVELOG_VERBOSEA("iter: %d\n", iter);

        _parameters.pSampleFn->Sample(&_randomConfig[0]);
    
        // extend A
        SpatialTree::ExtendType et = TreeA->Extend(&_randomConfig[0], iConnectedA);
        assert( iConnectedA >= 0 && iConnectedA < (int)TreeA->_nodes.size());

        // although check isn't necessary, having it improves running times
        if( et == SpatialTree::ET_Failed ) {
            // necessary to increment iterator in case spaces are not connected
            if( ++iter > 3*_parameters.nMaxIterations ) {
                RAVELOG_WARNA("iterations exceeded\n");
                break;
            }
            continue;
        }

        // extend B toward A
        et = TreeB->Extend(TreeA->_nodes[iConnectedA]->q, iConnectedB);
        assert( iConnectedB >= 0 && iConnectedB < (int)TreeB->_nodes.size());
        
        // if connected, success
        if( et == SpatialTree::ET_Connected ) {
            bConnected = true;
            break;
        }

        swap(TreeA, TreeB);
        iter += 3;
        if( iter > 3*_parameters.nMaxIterations ) {
            RAVELOG_WARNA("iterations exceeded\n");
            break;
        }
    }
    
    if( !bConnected ) {
        RAVELOG(L"plan failed, %fs\n",0.001f*(float)(timeGetTime()-basetime));
        return false;
    }
    
    list<Node*> vecnodes;
    
    // add nodes from the forward tree
    Node* pforward = TreeA == &_treeForward ? TreeA->_nodes[iConnectedA] :TreeB->_nodes[iConnectedB];
    while(1) {
        vecnodes.push_front(pforward);
        if(pforward->parent < 0)
            break;
        pforward = _treeForward._nodes[pforward->parent];
    }

    // add nodes from the backward tree
    int goalindex = -1;

    Node *pbackward = TreeA == &_treeBackward ? TreeA->_nodes[iConnectedA] :TreeB->_nodes[iConnectedB];
    while(1) {
        vecnodes.push_back(pbackward);
        if(pbackward->parent < 0) {
            goalindex = -pbackward->parent-1;
            break;
        }
        pbackward = _treeBackward._nodes[pbackward->parent];
    }

    assert( goalindex >= 0 );
    if( pOutStream != NULL )
        *pOutStream << goalindex;

    _OptimizePath(vecnodes);
    
    Trajectory::TPOINT pt; pt.q.resize(_pRobot->GetActiveDOF());
    
    FOREACH(itnode, vecnodes) {
        for(int i = 0; i < _pRobot->GetActiveDOF(); ++i)
            pt.q[i] = (*itnode)->q[i];
        ptraj->AddPoint(pt);
    }

    RAVELOG_DEBUGA("plan success, path=%"PRIdS" points in %fs\n", ptraj->GetPoints().size(), 0.001f*(float)(timeGetTime()-basetime));
    
    return true;
}

bool BirrtPlanner::_CheckCollision(const dReal* pq, bool bReport)
{
    _pRobot->SetActiveDOFValues(NULL, pq);
    COLLISIONREPORT report;

    bool bCol = GetEnv()->CheckCollision(_pRobot, bReport?&report:NULL) || _pRobot->CheckSelfCollision(bReport?&report:NULL);
    if( bCol && bReport ) {
        RAVELOG(L"fcollision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
    }
    return bCol;
}

bool BirrtPlanner::_CheckCollision(const dReal* pQ0, const dReal* pQ1, IntervalType interval, vector< vector<dReal> >* pvCheckedConfigurations)
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
    vtempconfig.resize(_pRobot->GetActiveDOF());

    if (bCheckEnd) {
        memcpy(&vtempconfig[0], pQ1,sizeof(pQ1[0])*vtempconfig.size());
        assert( _pRobot->GetActiveDOF() == (int)vtempconfig.size() );
        if( pvCheckedConfigurations != NULL )
            pvCheckedConfigurations->push_back(vtempconfig);
        _pRobot->SetActiveDOFValues(NULL, &vtempconfig[0]);
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
        _pRobot->SetActiveDOFValues(NULL, &vtempconfig[0]);
        if( GetEnv()->CheckCollision(_pRobot) || _pRobot->CheckSelfCollision() )
            return true;
    }

    return false;
}

void BirrtPlanner::_OptimizePath(list<Node*>& path)
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

            if( nrejected++ > (int)path.size()+8 )
                break;
            continue;
        }

        ++startNode;
        FOREACHC(itc, vconfigs)
            path.insert(startNode, _treeForward._nodes[_treeForward.AddNode(-1,&(*itc)[0])]);
        // splice out in-between nodes in path
        path.erase(startNode, endNode);
        nrejected = 0;

        if( path.size() <= 2 )
            return;
    }
}
