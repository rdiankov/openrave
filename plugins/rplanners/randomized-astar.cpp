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

// A continuous version of A*. See:
// Rosen Diankov, James Kuffner.
// Randomized Statistical Path Planning. Intl. Conf. on Intelligent Robots and Systems, October 2007. 
#include "plugindefs.h"

#include <algorithm>
#include <stdlib.h>
#include "randomized-astar.h"

void RandomizedAStarPlanner::SpatialTree::Destroy()
{
    list<Node*>::iterator it;
    FORIT(it, _nodes) {
        (*it)->~Node();
        free(*it);
    }
    FORIT(it, _dead) {
        (*it)->~Node();
        free(*it);
    }
    _nodes.clear();
}

RandomizedAStarPlanner::Node* RandomizedAStarPlanner::SpatialTree::GetNN(const dReal* q)
{
    assert( _pDistMetric!= NULL );
    if( _nodes.size() == 0 )
        return NULL;

    list<Node*>::iterator itnode = _nodes.begin(), itbest = _nodes.begin();
    dReal fbest = _pDistMetric->Eval(q, _nodes.front()->q);
    ++itnode;

    while(itnode != _nodes.end()) {
        dReal f = _pDistMetric->Eval(q, (*itnode)->q);
        if( f < fbest ) {
            itbest = itnode;
            fbest = f;
        }
        ++itnode;
    }

    _fBestDist = fbest;
    return *itbest;
}

RandomizedAStarPlanner::RandomizedAStarPlanner(EnvironmentBase* penv) : PlannerBase(penv), _robot(NULL)
{
    bUseGauss = false;
    nIndex = 0;
}

RandomizedAStarPlanner::~RandomizedAStarPlanner()
{
    Destroy();
}

void RandomizedAStarPlanner::Destroy()
{
    _spatialtree.Destroy();
    _sortedtree.Reset();

//    for(size_t i = 0; i < _vdeadnodes.size(); ++i) {
//        _vdeadnodes[i]->~Node();
//        free(_vdeadnodes[i]);
//    }
    _vdeadnodes.clear();
    _vdeadnodes.reserve(1<<16);
}

bool RandomizedAStarPlanner::InitPlan(RobotBase* pbase, const PlannerParameters* pparams)
{
    Destroy();
    _robot = pbase;
    if( pparams != NULL )
        _parameters.copy(*pparams);

    if( _robot == NULL )
        return false;

    if( _parameters.pgoalfn == NULL )
        _parameters.pgoalfn = &_goalmetric;
    if( _parameters.pcostfn == NULL )
        _parameters.pcostfn = &_costmetric;
    if( _parameters.pdistmetric == NULL )
        _parameters.pdistmetric = &_distmetric;
    if( _parameters.pConfigState == NULL )
        _parameters.pConfigState = &_defaultstate;

    _parameters.pConfigState->SetRobot(_robot);

    if( _parameters.pSampleFn == NULL ) {
        RAVELOG(L"using default sampling function\n");
        _defaultsampler.Init(_parameters.pConfigState, _parameters.pdistmetric);
        _parameters.pSampleFn = &_defaultsampler;
    }
    
    _parameters.pgoalfn->SetRobot(_robot);
    _parameters.pdistmetric->SetRobot(_robot);

    _vSampleConfig.resize(GetDOF());
    _jointIncrement.resize(GetDOF());
    _vzero.resize(GetDOF());
    memset(&_vzero[0], 0, sizeof(dReal)*GetDOF());

    _spatialtree._pDistMetric = _parameters.pdistmetric;

    //_robot->GetActiveDOFResolutions(_jointResolutionInv);
//    for(size_t i = 0; i < _jointResolutionInv.size(); ++i) {
//        if( _jointResolutionInv[i] != 0 ) _jointResolutionInv[i] = 1 / _jointResolutionInv[i];
//    }
    _jointResolutionInv.resize(GetDOF());
    FOREACH(it, _jointResolutionInv)
        *it = 50.0f;

    nIndex = 0;

    return true;
}

bool SortChildGoal(const RandomizedAStarPlanner::Node* p1, const RandomizedAStarPlanner::Node* p2)
{
    return p1->ftotal < p2->ftotal;//p1->ftotal-p1->fcost < p2->ftotal-p2->fcost;
}

bool RandomizedAStarPlanner::PlanPath(Trajectory* ptraj, std::ostream* pOutStream)
{
    assert( _robot != NULL && _parameters.pgoalfn!= NULL && _parameters.pcostfn != NULL && _parameters.pdistmetric != NULL );

    Destroy();

    RobotBase::RobotStateSaver saver(_robot);
    Node* pcurrent, *pbest = NULL;

    _parameters.pConfigState->SetState(_parameters.vinitialconfig);

    _parameters.pgoalfn->SetRobot(_robot);
    _parameters.pdistmetric->SetRobot(_robot);

    pcurrent = CreateNode(0, NULL, &_parameters.vinitialconfig[0]);

    COLLISIONREPORT report;
    if( GetEnv()->CheckCollision(_robot, &report) ) {
        RAVEPRINT(L"RA*: robot initially in collision %S:%S!\n",
                  report.plink1!=NULL?report.plink1->GetName():L"(NULL)",
                  report.plink2!=NULL?report.plink2->GetName():L"(NULL)");
        return false;
    }
    else if( _robot->CheckSelfCollision() ) {
        RAVEPRINT(L"RA*: robot self-collision!\n");
        return false;
    }
    
    vector<dReal> tempconfig;
    tempconfig.resize(GetDOF());

    int nMaxIter = _parameters.nMaxIterations > 0 ? _parameters.nMaxIterations : 8000;

    while(1) {

        if( _sortedtree.blocks.size() == 0 ) {
            break;
        }
        else {
            pcurrent = _sortedtree.blocks.back();
        }

        // delete from current lists
        _sortedtree.blocks.pop_back();
        _vdeadnodes.push_back(pcurrent);
        assert( pcurrent->numchildren < _parameters.nMaxChildren );

        if( pcurrent->ftotal - pcurrent->fcost < _parameters.fGoalCoeff*_parameters.pgoalfn->GetGoalThresh() ) {
            pbest = pcurrent;
            break;
        }

        list<Node*> children;
        int i;
        
        for(i = 0; i < _parameters.nMaxChildren && pcurrent->numchildren < _parameters.nMaxChildren; ++i) {

            // keep on sampling until a valid config
            int sample;
            for(sample = 0; sample < _parameters.nMaxSampleTries; ++sample) {
                if( !_parameters.pSampleFn->Sample(&_vSampleConfig[0], pcurrent->q, _parameters.fRadius) ) {
                    sample = 1000;
                    break;
                }

                // colision check
                _parameters.pConfigState->SetState(&_vSampleConfig[0]);

                if( _parameters.pconstraintfn != NULL ) {
                    if( !_parameters.pconstraintfn->Constraint(&pcurrent->q[0], &_vSampleConfig[0], &_vectrans[0],0) )
                        continue;
                }

                if (_CheckCollision(pcurrent->q, &_vSampleConfig[0], OPEN))
                    continue;
                
                _parameters.pConfigState->SetState(&_vSampleConfig[0]);
                if( GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
                    continue;

                break;
                
            }
            if( sample >= _parameters.nMaxSampleTries )
                continue;
                        
            //while (getchar() != '\n') usleep(1000);

            Node* nearestnode = _spatialtree.GetNN(&_vSampleConfig[0]);
            if( _spatialtree._fBestDist > _parameters.fDistThresh ) {
                dReal* qnew = &_vSampleConfig[0];
                dReal fdist = _parameters.pdistmetric->Eval(pcurrent->q, qnew);
                CreateNode(nearestnode->fcost + fdist * _parameters.pcostfn->Eval(qnew), nearestnode, qnew, true);
                pcurrent->numchildren++;

                if( (_spatialtree._nodes.size() % 50) == 0 ) {
                    //DumpNodes();
                    RAVELOG_VERBOSEA("trees at %"PRIdS"(%"PRIdS") : to goal at %f,%f\n", _sortedtree.blocks.size(), _spatialtree._nodes.size(), (pcurrent->ftotal-pcurrent->fcost)/_parameters.fGoalCoeff, pcurrent->fcost);
                }
            }
        }

        if( (int)_spatialtree._nodes.size() > nMaxIter ) {
            break;
        }
    }

    //DumpNodes();

    if( pbest == NULL ) {
        return false;
    }

    //RAVEPRINT(L"Path found, final node: %f, %f\n", pbest->fcost, pbest->ftotal-pbest->fcost);

    _parameters.pConfigState->SetState(&pbest->q[0]);
    if( GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
        RAVEPRINT(L"RA* collision\n");
    
    wstringstream ss;
    ss << endl << "Path found, final node: cost: " << pbest->fcost << ", goal: " << (pbest->ftotal-pbest->fcost)/_parameters.fGoalCoeff << endl;
    for(int i = 0; i < GetDOF(); ++i)
        ss << pbest->q[i] << " ";
    ss << "\n-------\n";
    RAVEPRINT(ss.str().c_str());

    list<Node*> vecnodes;

    while(pcurrent != NULL) {
        vecnodes.push_back(pcurrent);
        pcurrent = pcurrent->parent;
    }

    _OptimizePath(vecnodes);

    Trajectory::TPOINT p;
    p.q = _parameters.vinitialconfig;
    ptraj->AddPoint(p);

    list<Node*>::reverse_iterator itcur, itprev;
    itcur = vecnodes.rbegin();
    itprev = itcur++;
    while(itcur != vecnodes.rend() ) {
        _InterpolateNodes((*itprev)->q, (*itcur)->q, ptraj);
        itprev = itcur;
        ++itcur;
    }

    return true;
}

RandomizedAStarPlanner::Node* RandomizedAStarPlanner::CreateNode(dReal fcost, Node* parent, const dReal* pfConfig, bool add)
{
    assert( pfConfig != NULL );

    void* pmem = malloc(sizeof(Node)+sizeof(dReal)*GetDOF());
    Node* p = ::new(pmem) Node(); // call constructor explicitly
    p->parent = parent;
    if( parent != NULL )
        p->level = parent->level + 1;

    memcpy(p->q, pfConfig, sizeof(dReal)*GetDOF());
    p->fcost = fcost;
    p->ftotal = _parameters.fGoalCoeff*_parameters.pgoalfn->Eval(pfConfig) + fcost;

    if( add ) {
        _spatialtree.AddNode(p);
        _sortedtree.Add(p);
    }
    return p;
}

void RandomizedAStarPlanner::_InterpolateNodes(const dReal* pQ0, const dReal* pQ1, Trajectory* ptraj)
{
    // compute  the discretization
    int i, numSteps = 1;
    for (i = 0; i < GetDOF(); i++) {
        int steps = (int)(fabs(pQ1[i] - pQ0[i]) * _jointResolutionInv[i]);
        if (steps > numSteps)
            numSteps = steps;
    }

    // compute joint increments
    for (i = 0; i < GetDOF(); i++)
        _jointIncrement[i] = (pQ1[i] - pQ0[i])/((dReal)numSteps);

    Trajectory::TPOINT p;
    p.q.resize(GetDOF());

    // compute the straight-line path
    for (int f = 1; f <= numSteps; f++) {
        for (i = 0; i < GetDOF(); i++)
            p.q[i] = pQ0[i] + (_jointIncrement[i] * f);
        ptraj->AddPoint(p);
    }
}

void RandomizedAStarPlanner::DumpNodes()
{
    char filename[255];
    sprintf(filename, "matlab/nodes%d.m", nIndex++);
    FILE* f = fopen(filename, "w");
    if( f == NULL ) {
        //RAVEPRINT(L"failed to dump nodes\n");
        return;
    }

    vector<Node*>::iterator it;

    vector<Node*>* allnodes[2] = { &_vdeadnodes, &_sortedtree.blocks };

    fprintf(f, "allnodes = [");

    for(size_t n = 0; n < ARRAYSIZE(allnodes); ++n) {
    
        FORIT(it, *allnodes[n]) {
            for(int i = 0; i < GetDOF(); ++i) {
                fprintf(f, "%f ", (*it)->q[i]);
            }

            int index = 0;
            if( (*it)->parent != NULL ) {

                index = 0;
                for(size_t j = 0; j < ARRAYSIZE(allnodes); ++j) {
                    vector<Node*>::iterator itfound = find(allnodes[j]->begin(), allnodes[j]->end(), (*it)->parent);
                    if( itfound != allnodes[j]->end() ) {
                        index += (int)(itfound-allnodes[j]->begin());
                        break;
                    }
                    index += (int)_vdeadnodes.size();
                }
            }

            fprintf(f, "%f %d\n", ((*it)->ftotal-(*it)->fcost)/_parameters.fGoalCoeff, index+1);
        }
    }

    fprintf(f ,"];\r\n\r\n");
    fprintf(f, "startindex = %"PRIdS, _vdeadnodes.size()+1);
    
    fclose(f);
}

//! optimize the computed path for a single step
void RandomizedAStarPlanner::_OptimizePath(list<Node*>& path)
{
    if( path.size() <= 2 )
        return;

    list<Node*>::iterator startNode, endNode;
    
    for(int i = 2 * (int)path.size(); i > 0; --i) {
        // pick a random node on the path, and a random jump ahead
        int startIndex = RANDOM_INT((int)path.size() - 2);
        int endIndex   = startIndex + (RANDOM_INT(5) + 2);
        if (endIndex >= (int)path.size()) endIndex = (int)path.size() - 1;
        
        startNode = path.begin();
        advance(startNode, startIndex);
        endNode = startNode;
        advance(endNode, endIndex-startIndex);

        // check if the nodes can be connected by a straight line
        if (_CheckCollision((*startNode)->q, (*endNode)->q, OPEN)) {
            continue;
        }

        // splice out in-between nodes in path
        path.erase(++startNode, endNode);

        if( path.size() <= 2 )
            return;
    }
}

bool RandomizedAStarPlanner::_CheckCollision(const dReal *pQ0, const dReal *pQ1, IntervalType interval)
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
    if (bCheckEnd) {
        _parameters.pConfigState->SetState(pQ1);
        if (GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
            return true;
    }

    // compute  the discretization
    int i, numSteps = 1;
    for (i = 0; i < GetDOF(); i++) {
        int steps = (int)(fabs(pQ1[i] - pQ0[i]) * _jointResolutionInv[i]);
        if (steps > numSteps)
            numSteps = steps;
    }

    // compute joint increments
    for (i = 0; i < GetDOF(); i++)
        _jointIncrement[i] = (pQ1[i] - pQ0[i])/((dReal)numSteps);

    static vector<dReal> v;
    v.resize(GetDOF());

    // check for collision along the straight-line path
    // NOTE: this does not check the end config, and may or may
    // not check the start based on the value of 'start'
    for (int f = start; f < numSteps; f++) {
        for (i = 0; i < GetDOF(); i++)
            v[i] = pQ0[i] + (_jointIncrement[i] * f);

        _parameters.pConfigState->SetState(&v[0]);
        if( GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
            return true;
    }

    return false;
}
