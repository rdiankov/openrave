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
//
// GraspExplorationPlanner: a planner biased towards exploration of the free space
// author: Rosen Diankov
#include "plugindefs.h"

#include <algorithm>

#ifndef _WIN32
#include <sys/time.h>
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#include "graspexplorationplanner.h"

void GraspExplorationPlanner::SpatialTree::Reset(GraspExplorationPlanner* planner, int dof)
{
    _planner = planner;

    vector<Node*>::iterator it;
    FORIT(it, _nodes) {
        (*it)->~Node();
        free(*it);
    }
    _nodes.resize(0);
    if( dof > 0 ) {
        _dof = dof;
        _vLowerLimit.resize(_dof);
        _vUpperLimit.resize(_dof);
        _vDOFRange.resize(_dof);
    }   
}

int GraspExplorationPlanner::SpatialTree::AddNode(float fcost, int parent, const dReal* pfConfig)
{
    assert( pfConfig != NULL );

    void* pmem = malloc(sizeof(Node)+sizeof(dReal)*_dof);
    Node* p = ::new(pmem) Node(); // call constructor explicitly
    p->parent = parent;
    memcpy(p->q, pfConfig, sizeof(dReal)*_dof);
    p->fcost = fcost;
    _nodes.push_back(p);
    return (int)_nodes.size()-1;
}

int GraspExplorationPlanner::SpatialTree::GetNN(const dReal* q)
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

int GraspExplorationPlanner::SpatialTree::Extend(dReal* pNewConfig)
{
    assert( _pSampleFn != NULL );
    _pSampleFn->Sample(pNewConfig);

    // get the nearest neighbor
    int inode = GetNN(pNewConfig);
    assert(inode >= 0 );
    Node* pnode = _nodes[inode];

    // extend
    float fdist = _pDistMetric->Eval(pnode->q, pNewConfig);

    if( fdist > _fStepLength ) fdist = _fStepLength / fdist;
    else fdist = 1;

    for(int i = 0; i < _dof; ++i)
        pNewConfig[i] = pnode->q[i] + (pNewConfig[i]-pnode->q[i])*fdist;

    // project to constraints
    if( _planner->_parameters.pconstraintfn != NULL ) {
        if( !_planner->_parameters.pconstraintfn->Constraint(pnode->q, pNewConfig, NULL, 0) ) {
            return -1;
        }
    }

    return inode;
}

GraspExplorationPlanner::GraspExplorationPlanner(EnvironmentBase* penv) : PlannerBase(penv), _robot(NULL)
{
    _fExploreProb = 0;
    nDumpIndex = 0;
    _pConfigDistMetric = NULL;
}

GraspExplorationPlanner::~GraspExplorationPlanner()
{
    Destroy();
}

void GraspExplorationPlanner::Destroy()
{
    Reset();
}

void GraspExplorationPlanner::Reset()
{
    _configtree.Reset(this, -1);
}

bool GraspExplorationPlanner::InitPlan(RobotBase* pbase, const PlannerParameters* pparams)
{
    _SetTime();

    nDumpIndex = 0;
    Destroy();
    _robot = pbase;

    if( pparams != NULL )
        _parameters = *pparams;

    if( _robot == NULL )
        return false;

    if( _parameters.pcostfn == NULL )
        _parameters.pcostfn = &_costmetric;
    if( _parameters.pdistmetric == NULL )
        _parameters.pdistmetric = &_distmetric;

    _costmetric.fconst = 1;//fConfigFollowProb;
    
    _parameters.pdistmetric->SetRobot(pbase);

    _configtree.Reset(this, _robot->GetActiveDOF());
    _configtree._fStepLength = 0.01f;
    if( _parameters.vParameters.size() > 0 )
        _configtree._fStepLength = _parameters.vParameters[0];
    _fExploreProb = _parameters.vParameters.size() > 1 ? _parameters.vParameters[1] : 0;
    
    _vSampleConfig.resize(_robot->GetActiveDOF());
    _jointIncrement.resize(_robot->GetActiveDOF());

    // forward search limits
    _robot->GetActiveDOFLimits(_configtree._vLowerLimit, _configtree._vUpperLimit);

    _configtree._vDOFRange.resize(_configtree._vLowerLimit.size());
    for(size_t i = 0; i < _configtree._vLowerLimit.size(); ++i)
        _configtree._vDOFRange[i] = _configtree._vUpperLimit[i] - _configtree._vLowerLimit[i];

    _configtree._pDistMetric = _pConfigDistMetric = _parameters.pdistmetric;
    _configtree._pSampleFn = _parameters.pSampleFn != NULL ? _parameters.pSampleFn : &_samplerforward;

    _robot->GetActiveDOFResolutions(_configtree._jointResolution);
    _configtree._jointResolutionInv.resize(0);
    FOREACH(itj, _configtree._jointResolution) {
        _configtree._jointResolutionInv.push_back(*itj != 0  ? 1 / *itj : 1.0f);
    }

    _samplerforward.Init(&_configtree);

    Reset();

    // forward
    _configtree._nodes[_AddFwdNode(-1, &_parameters.vinitialconfig[0])];

    // forward
    RAVEPRINT(L"GraspExploration: step=%f, size=%d, explore=%f\n", _configtree._fStepLength, _parameters.nMaxIterations, _fExploreProb);
    _bInit = true;
    return true;
}

bool GraspExplorationPlanner::PlanPath(Trajectory* ptraj, std::ostream* pOutStream)
{
    assert( _robot != NULL && _parameters.pcostfn != NULL && _parameters.pdistmetric != NULL );

    if( !_bInit )
        return false;

    RobotBase::RobotStateSaver saver(_robot);

    _parameters.pdistmetric->SetRobot(_robot);
    
    _robot->SetActiveDOFValues(NULL, &_parameters.vinitialconfig[0]);
    COLLISIONREPORT report;
    if( GetEnv()->CheckCollision(_robot, &report) ) {
        RAVEPRINT(L"robot initially in collision %S:%S!\n",
                  report.plink1!=NULL?report.plink1->GetName():L"(NULL)",
                  report.plink2!=NULL?report.plink2->GetName():L"(NULL)");
        return false;
    }
    else if( _robot->CheckSelfCollision() ) {
        RAVEPRINT(L"robot self-collision!\n");
        return false;
    }

    int nMaxIter = _parameters.nMaxIterations > 0 ? _parameters.nMaxIterations : 1000;
    int iter = 0;

    while(iter < nMaxIter) {

        if( GetEnv()->RandomFloat() < _fExploreProb ) {
            // explore
            int inode = GetEnv()->RandomInt()%_configtree._nodes.size();            
            Node* pnode = _configtree._nodes[inode];

            // pick a random direction and go to it
            //int irandcoord = GetEnv()->RandomInt()%_vSampleConfig.size();
            //for(size_t i = 0; i < _vSampleConfig.size(); ++i)
            //    _vSampleConfig[i] = pnode->q[i];
            //_vSampleConfig[irandcoord] += 2.0f*_configtree._fStepLength*(GetEnv()->RandomFloat() - 0.5f);

            for(size_t i = 0; i < _vSampleConfig.size(); ++i)
                _vSampleConfig[i] = pnode->q[i] + GetEnv()->RandomFloat() - 0.5f;
            
            float fdist = _configtree._pDistMetric->Eval(pnode->q, &_vSampleConfig[0]);
            
            if( fdist > _configtree._fStepLength ) {
                fdist = _configtree._fStepLength / fdist;
            
                for(size_t i = 0; i < _vSampleConfig.size(); ++i)
                    _vSampleConfig[i] = pnode->q[i] + (_vSampleConfig[i]-pnode->q[i])*fdist;
            }
            
            // project to constraints
            if( _parameters.pconstraintfn != NULL ) {
                if( !_parameters.pconstraintfn->Constraint(pnode->q, &_vSampleConfig[0], NULL, 0) ) {
                    // sample again
                    continue;
                }
            }

            if( !_CheckCollision(pnode->q, &_vSampleConfig[0], OPEN_START) ) {
                _AddFwdNode(inode, &_vSampleConfig[0]);
                ++iter;
                RAVELOG(L"iter %d\n", iter);
            }
        }
        else { // rrt extend
            int inode = _configtree.Extend(&_vSampleConfig[0]);
            
            if( inode >= 0 && !_CheckCollision(_configtree._nodes[inode]->q, &_vSampleConfig[0], OPEN_START) ) {
                _AddFwdNode(inode, &_vSampleConfig[0]);
                ++iter;
                RAVELOG(L"iter %d\n", iter);
            }
        }
    }
    
    // save nodes to trajectory
    Trajectory::TPOINT tp; tp.q.resize(_robot->GetActiveDOF());
    FOREACH(itnode, _configtree._nodes) {
        for(int i = 0; i < _robot->GetActiveDOF(); ++i)
            tp.q[i] = (*itnode)->q[i];
        ptraj->AddPoint(tp);
    }

    return true;
}

int GraspExplorationPlanner::_AddFwdNode(int iparent, dReal* pconfig)
{
    // have to set and get in case limit and other normalization checks are required
    _robot->SetActiveDOFValues(NULL, pconfig, true);
    _robot->GetActiveDOFValues(pconfig);

    int inode = _configtree.AddNode(0, iparent, pconfig);
    _configtree._nodes[inode]->fcost = _parameters.pcostfn->Eval(NULL);
    return inode;
}

bool GraspExplorationPlanner::_CheckCollision(const dReal* pQ0, const dReal* pQ1, IntervalType interval)
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
    vtempconfig.resize(_configtree.GetDOF());

    if (bCheckEnd) {
        memcpy(&vtempconfig[0], pQ1,sizeof(pQ1[0])*vtempconfig.size());
        _SetRobotConfig(&vtempconfig[0]);
        if (GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
            return true;
    }

    // compute  the discretization
    int i, numSteps = 1;
    dReal* pfresolution = &_configtree._jointResolutionInv[0];
    for (i = 0; i < (int)vtempconfig.size(); i++) {
        int steps = (int)(fabs(pQ1[i] - pQ0[i]) * pfresolution[i]);
        if (steps > numSteps)
            numSteps = steps;
    }

    // compute joint increments
    for (i = 0; i < (int)vtempconfig.size(); i++)
        _jointIncrement[i] = (pQ1[i] - pQ0[i])/((dReal)numSteps);

    // check for collision along the straight-line path
    // NOTE: this does not check the end config, and may or may
    // not check the start based on the value of 'start'
    for (int f = start; f < numSteps; f++) {

        for (i = 0; i < (int)vtempconfig.size(); i++)
            vtempconfig[i] = pQ0[i] + (_jointIncrement[i] * f);

        _SetRobotConfig(&vtempconfig[0]);
        if( GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
            return true;
    }

    return false;
}

bool GraspExplorationPlanner::_CheckCollision(const dReal* pq, bool bReport)
{
    _SetRobotConfig(pq);
    COLLISIONREPORT report;

    bool bCol = GetEnv()->CheckCollision(_robot, bReport?&report:NULL) || _robot->CheckSelfCollision(bReport?&report:NULL);
    if( bCol && bReport ) {
        RAVELOG(L"fcollision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
    }
    return bCol;
}

void GraspExplorationPlanner::_SetRobotConfig(const dReal* pconfig)
{
    _robot->SetActiveDOFValues(NULL, &pconfig[0]);
}

void GraspExplorationPlanner::_SetTime()
{
    _nBaseStartTime = GetMicroTime();
}

dReal GraspExplorationPlanner::_GetTime()
{
    return (dReal)(GetMicroTime()-_nBaseStartTime) * 1e-6f;
}
