// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef RAVE_RANDOMIZED_ASTAR
#define RAVE_RANDOMIZED_ASTAR

#include "rplanners.h"

class RandomizedAStarPlanner : public PlannerBase
{
public:
    struct Node
    {
        Node() { parent = NULL; level = 0; numchildren = 0; }

        // negative because those are most likely to be popped first
        dReal getvalue() { return -ftotal; }

        bool compare(const Node* r) { BOOST_ASSERT(r != NULL); return ftotal < r->ftotal; }
        
        dReal fcost, ftotal;
        int level;
        Node* parent;
        int numchildren;
        vector<dReal> q; // the configuration immediately follows the struct
    };

    static bool SortChildGoal(const RandomizedAStarPlanner::Node* p1, const RandomizedAStarPlanner::Node* p2)
    {
        return p1->ftotal < p2->ftotal;//p1->ftotal-p1->fcost < p2->ftotal-p2->fcost;
    }

    // implement kd-tree or approx-nn in the future, deallocates memory from Node
    class SpatialTree
    {
    public:
        SpatialTree() {_fBestDist = 0; }
        ~SpatialTree() { Destroy(); }

        void Destroy()
        {
            list<Node*>::iterator it;
            FORIT(it, _nodes)
                delete *it;
            FORIT(it, _dead)
                delete *it;
            _nodes.clear();
        }

        inline void AddNode(Node* pnode) { _nodes.push_back(pnode); }
        Node* GetNN(const vector<dReal>& q)
        {
            if( _nodes.size() == 0 )
                return NULL;

            list<Node*>::iterator itnode = _nodes.begin(), itbest = _nodes.begin();
            dReal fbest = _pDistMetric(q, _nodes.front()->q);
            ++itnode;

            while(itnode != _nodes.end()) {
                dReal f = _pDistMetric(q, (*itnode)->q);
                if( f < fbest ) {
                    itbest = itnode;
                    fbest = f;
                }
                ++itnode;
            }

            _fBestDist = fbest;
            return *itbest;
        }
        inline void RemoveNode(Node* pnode) { _nodes.remove(pnode); }

        list<Node*> _nodes;
        list<Node*> _dead;
        boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _pDistMetric;
        dReal _fBestDist; ///< valid after a call to GetNN
    };

    enum IntervalType {
        OPEN = 0,
        OPEN_START,
        OPEN_END,
        CLOSED
    };

 RandomizedAStarPlanner(EnvironmentBasePtr penv) : PlannerBase(penv)
    {
        __description = "Constrained Grasp Planning Randomized A*";
        _report.reset(new COLLISIONREPORT());
        bUseGauss = false;
        nIndex = 0;
    }
    
    virtual ~RandomizedAStarPlanner() {}

    void Destroy()
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

    virtual RobotBasePtr GetRobot() { return _robot; }

    // Planning Methods
    ///< manipulator state is also set
    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        Destroy();
        _robot = pbase;
        _parameters.copy(pparams);

        RobotBase::RobotStateSaver savestate(_robot);

        if( !_parameters._goalfn )
            _parameters._goalfn = boost::bind(&SimpleGoalMetric::Eval,boost::shared_ptr<SimpleGoalMetric>(new SimpleGoalMetric(_robot)),_1);
        if( !_parameters._costfn )
            _parameters._costfn = boost::bind(&SimpleCostMetric::Eval,boost::shared_ptr<SimpleCostMetric>(new SimpleCostMetric(_robot)),_1);

        _vSampleConfig.resize(GetDOF());
        _jointIncrement.resize(GetDOF());
        _vzero.resize(GetDOF(),0);
        _spatialtree._pDistMetric = _parameters._distmetricfn;

        _jointResolutionInv.resize(0);
        FOREACH(itj, _parameters._vConfigResolution) {
            if( *itj != 0 )
                _jointResolutionInv.push_back(1 / *itj);
            else {
                RAVELOG_WARNA("resolution is 0!\n");
                _jointResolutionInv.push_back(100);
            }
        }

        nIndex = 0;
        return true;
    }

    virtual bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        Destroy();

        RobotBase::RobotStateSaver saver(_robot);
        Node* pcurrent=NULL, *pbest = NULL;

        _parameters._setstatefn(_parameters.vinitialconfig);
        pcurrent = CreateNode(0, NULL, _parameters.vinitialconfig);

        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot), _report) ) {
            RAVELOG_WARNA("RA*: robot initially in collision %s:%s!\n",
                          _report->plink1!=NULL?_report->plink1->GetName().c_str():"(NULL)",
                          _report->plink2!=NULL?_report->plink2->GetName().c_str():"(NULL)");
            return false;
        }
        else if( _parameters._bCheckSelfCollisions && _robot->CheckSelfCollision() ) {
            RAVELOG_WARNA("RA*: robot self-collision!\n");
            return false;
        }
    
        vector<dReal> tempconfig(GetDOF());
        int nMaxIter = _parameters._nMaxIterations > 0 ? _parameters._nMaxIterations : 8000;

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
            BOOST_ASSERT( pcurrent->numchildren < _parameters.nMaxChildren );

            if( pcurrent->ftotal - pcurrent->fcost < 1e-4f ) {
                pbest = pcurrent;
                break;
            }

            list<Node*> children;
            int i;
        
            for(i = 0; i < _parameters.nMaxChildren && pcurrent->numchildren < _parameters.nMaxChildren; ++i) {

                // keep on sampling until a valid config
                int sample;
                for(sample = 0; sample < _parameters.nMaxSampleTries; ++sample) {
                    if( !_parameters._sampleneighfn(_vSampleConfig, pcurrent->q, _parameters.fRadius) ) {
                        sample = 1000;
                        break;
                    }

                    if( !!_parameters._constraintfn ) {
                        _parameters._setstatefn(_vSampleConfig);
                        if( !_parameters._constraintfn(pcurrent->q, _vSampleConfig, 0) )
                            continue;
                    }

                    if (_CheckCollision(pcurrent->q, _vSampleConfig, OPEN))
                        continue;
                
                    _parameters._setstatefn(_vSampleConfig);
                    if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) || (_parameters._bCheckSelfCollisions&&_robot->CheckSelfCollision()) )
                        continue;

                    break;
                
                }
                if( sample >= _parameters.nMaxSampleTries )
                    continue;
                        
                //while (getchar() != '\n') usleep(1000);

                Node* nearestnode = _spatialtree.GetNN(_vSampleConfig);
                if( _spatialtree._fBestDist > _parameters.fDistThresh ) {
                    dReal fdist = _parameters._distmetricfn(pcurrent->q, _vSampleConfig);
                    CreateNode(nearestnode->fcost + fdist * _parameters._costfn(_vSampleConfig), nearestnode, _vSampleConfig, true);
                    pcurrent->numchildren++;

                    if( (_spatialtree._nodes.size() % 50) == 0 ) {
                        //DumpNodes();
                        RAVELOG_VERBOSEA(str(boost::format("trees at %d(%d) : to goal at %f,%f\n")%_sortedtree.blocks.size()%_spatialtree._nodes.size()%((pcurrent->ftotal-pcurrent->fcost)/_parameters.fGoalCoeff)%pcurrent->fcost));
                    }
                }
            }

            if( (int)_spatialtree._nodes.size() > nMaxIter ) {
                break;
            }
        }

        if( pbest == NULL )
            return false;

        RAVELOG_DEBUGA("Path found, final node: %f, %f\n", pbest->fcost, pbest->ftotal-pbest->fcost);

        _parameters._setstatefn(pbest->q);
        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) || (_parameters._bCheckSelfCollisions&&_robot->CheckSelfCollision()) )
            RAVELOG_WARNA("RA* collision\n");
    
        stringstream ss;
        ss << endl << "Path found, final node: cost: " << pbest->fcost << ", goal: " << (pbest->ftotal-pbest->fcost)/_parameters.fGoalCoeff << endl;
        for(int i = 0; i < GetDOF(); ++i)
            ss << pbest->q[i] << " ";
        ss << "\n-------\n";
        RAVELOG_DEBUGA(ss.str());

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

    int GetTotalNodes() { return (int)_sortedtree.blocks.size(); }
    
    bool bUseGauss;
    
private:

    Node* CreateNode(dReal fcost, Node* parent, const vector<dReal>& pfConfig, bool add = true)
    {
        Node* p = new Node();
        p->parent = parent;
        if( parent != NULL )
            p->level = parent->level + 1;

        p->q = pfConfig;
        p->fcost = fcost;
        p->ftotal = _parameters.fGoalCoeff*_parameters._goalfn(pfConfig) + fcost;

        if( add ) {
            _spatialtree.AddNode(p);
            _sortedtree.Add(p);
        }
        return p;
    }

    void _InterpolateNodes(const vector<dReal>& pQ0, const vector<dReal>& pQ1, TrajectoryBasePtr ptraj)
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

    bool _CheckCollision(const vector<dReal>& pQ0, const vector<dReal>& pQ1, IntervalType interval, vector< vector<dReal> >* pvCheckedConfigurations = NULL)
    {
        // set the bounds based on the interval type
        int start=0;
        bool bCheckEnd=false;
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
            BOOST_ASSERT(0);
        }

        // first make sure the end is free
        vector<dReal> vtempconfig(_parameters.GetDOF());
        if (bCheckEnd) {
            if( pvCheckedConfigurations != NULL )
                pvCheckedConfigurations->push_back(pQ1);
            _parameters._setstatefn(pQ1);
            if (GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) || (_parameters._bCheckSelfCollisions&&_robot->CheckSelfCollision()) )
                return true;
        }

        // compute  the discretization
        int i, numSteps = 1;
        dReal* pfresolution = &_jointResolutionInv[0];
        for (i = 0; i < _parameters.GetDOF(); i++) {
            int steps = (int)(fabs(pQ1[i] - pQ0[i]) * pfresolution[i]);
            if (steps > numSteps)
                numSteps = steps;
        }

        // compute joint increments
        for (i = 0; i < _parameters.GetDOF(); i++)
            _jointIncrement[i] = (pQ1[i] - pQ0[i])/((float)numSteps);

        // check for collision along the straight-line path
        // NOTE: this does not check the end config, and may or may
        // not check the start based on the value of 'start'
        for (int f = start; f < numSteps; f++) {

            for (i = 0; i < _parameters.GetDOF(); i++)
                vtempconfig[i] = pQ0[i] + (_jointIncrement[i] * f);
        
            if( pvCheckedConfigurations != NULL )
                pvCheckedConfigurations->push_back(vtempconfig);
            _parameters._setstatefn(vtempconfig);
            if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) || (_parameters._bCheckSelfCollisions&&_robot->CheckSelfCollision()) )
                return true;
        }

        return false;
    }

    void _OptimizePath(list<Node*>& path)
    {
        if( path.size() <= 2 )
            return;

        list<Node*>::iterator startNode, endNode;
    
        for(int i = 2 * (int)path.size(); i > 0; --i) {
            // pick a random node on the path, and a random jump ahead
            int startIndex = RaveRandomInt()%((int)path.size() - 2);
            int endIndex   = startIndex + ((RaveRandomInt()%5) + 2);
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

    void DumpNodes()
    {
        char filename[255];
        sprintf(filename, "matlab/nodes%d.m", nIndex++);
        FILE* f = fopen(filename, "w");
        if( f == NULL ) {
            return;
        }

        vector<Node*>::iterator it;

        vector<Node*>* allnodes[2] = { &_vdeadnodes, &_sortedtree.blocks };

        fprintf(f, "allnodes = [");

        for(int n = 0; n < 2; ++n) {
    
            FORIT(it, *allnodes[n]) {
                for(int i = 0; i < GetDOF(); ++i) {
                    fprintf(f, "%f ", (*it)->q[i]);
                }

                int index = 0;
                if( (*it)->parent != NULL ) {

                    index = 0;
                    for(size_t j = 0; j < 2; ++j) {
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
        fprintf(f, "%s", str(boost::format("startindex = %d")%(_vdeadnodes.size()+1)).c_str());
    
        fclose(f);
    }

    inline int GetDOF() const { return _parameters.GetDOF(); }
    
    RAStarParameters _parameters;
    SpatialTree _spatialtree;
    BinarySearchTree<Node*, dReal> _sortedtree;   // sorted by decreasing value
    CollisionReportPtr _report;

    RobotBasePtr _robot;

    vector<Node*> _vdeadnodes; ///< dead nodes
    vector<dReal> _vSampleConfig;
    vector<dReal> _jointIncrement, _jointResolutionInv;
    vector<dReal> _vzero;

    vector<Transform> _vectrans; ///< cache
    int nIndex;
};

#endif
