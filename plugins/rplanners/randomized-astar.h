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
#ifndef RAVE_RANDOMIZED_ASTAR
#define RAVE_RANDOMIZED_ASTAR

class RandomizedAStarPlanner : public PlannerBase
{   
public:
    class RAStarParameters : public PlannerParameters
    {
    public:
        RAStarParameters() : fRadius(0.1f), fDistThresh(0.03f), fGoalCoeff(1), nMaxChildren(5), nMaxSampleTries(10) {}
        
        dReal fRadius;      ///< _pDistMetric thresh is the radius that children must be within parents
        dReal fDistThresh;  ///< gamma * _pDistMetric->thresh is the sampling radius
        dReal fGoalCoeff;   ///< balancees exploratino vs cost
        int nMaxChildren;   ///< limit on number of children
        int nMaxSampleTries; ///< max sample tries before giving up on creating a child
    protected:
        virtual bool serialize(std::ostream& O) const
        {
            if( !PlannerParameters::serialize(O) )
                return false;

            O << "<radius>" << fRadius << "</radius>" << endl;
            O << "<distthresh>" << fDistThresh << "</distthresh>" << endl;
            O << "<goalcoeff>" << fGoalCoeff << "</goalcoeff>" << endl;
            O << "<maxchildren>" << nMaxChildren << "</maxchildren>" << endl;
            O << "<maxsampletries>" << nMaxSampleTries << "</maxsampletries>" << endl;
    
            return !!O;
        }
        
        virtual bool endElement(void *ctx, const char *name)
        {
            if( stricmp(name,"radius") == 0 )
                _ss >> fRadius;
            else if( stricmp(name,"distthresh") == 0 )
                _ss >> fDistThresh;
            else if( stricmp(name, "goalcoeff") == 0 )
                _ss >> fGoalCoeff;
            else if( stricmp(name, "maxchildren") == 0 )
                _ss >> nMaxChildren;
            else if( stricmp(name, "maxsampletries") == 0 )
                _ss >> nMaxSampleTries;
            else
                return PlannerParameters::endElement(ctx, name);
            return false;
        }
    };

    struct Node
    {
        Node() { parent = NULL; level = 0; numchildren = 0; }

        // negative because those are most likely to be popped first
        dReal getvalue() { return -ftotal; }

        bool compare(const Node* r) { assert(r != NULL); return ftotal < r->ftotal; }
        
        dReal fcost, ftotal;
        int level;
        Node* parent;
        int numchildren;
        dReal q[0]; // the configuration immediately follows the struct
    };

    // implement kd-tree or approx-nn in the future, deallocates memory from Node
    class SpatialTree
    {
    public:
        SpatialTree() {_fBestDist = 0; _pDistMetric = NULL; }
        ~SpatialTree() { Destroy(); }

        void Destroy();
        inline void AddNode(Node* pnode) { _nodes.push_back(pnode); }
        Node* GetNN(const dReal* q); ///< return the nearest neighbor
        inline void RemoveNode(Node* pnode) { _nodes.remove(pnode); }

        list<Node*> _nodes;
        list<Node*> _dead;
        DistanceMetric* _pDistMetric;
        dReal _fBestDist; ///< valid after a call to GetNN
    };

    enum IntervalType {
        OPEN = 0,
        OPEN_START,
        OPEN_END,
        CLOSED
    };

    RandomizedAStarPlanner(EnvironmentBase* penv);
    ~RandomizedAStarPlanner();

    void Destroy();
    virtual RobotBase* GetRobot() const { return _robot; }

    // Planning Methods
    ///< manipulator state is also set
    virtual bool InitPlan(RobotBase* pbase, const PlannerParameters* pparams);
    virtual bool PlanPath(Trajectory* ptraj, std::ostream* pOutStream);

    virtual const wchar_t* GetDescription() const { return L"Constrained Grasp Planning Randomized A*"; }

    int GetTotalNodes() { return (int)_sortedtree.blocks.size(); }
    
    bool bUseGauss;
    
private:

    Node* CreateNode(dReal fcost, Node* parent, const dReal* pfConfig, bool add = true);
    inline void Sample(dReal B, const dReal* pfConfig);
    void _InterpolateNodes(const dReal* pQ0, const dReal* pQ1, Trajectory* ptraj);
    bool _CheckCollision(const dReal *pQ0, const dReal *pQ1, IntervalType interval);
    void _OptimizePath(list<Node*>& path);

    void DumpNodes();

    int GetDOF() const { return _parameters.pConfigState->GetDOF(); }
    
    RAStarParameters _parameters;
    SpatialTree _spatialtree;
    BinarySearchTree<Node*, dReal> _sortedtree;   // sorted by decreasing value

    RobotBase* _robot;

    vector<Node*> _vdeadnodes; ///< dead nodes
    vector<dReal> _vSampleConfig;
    vector<dReal> _jointIncrement, _jointResolutionInv;
    vector<dReal> _vzero;

    vector<Transform> _vectrans; ///< cache
    int nIndex;

    // default metrics

    class SimpleCostMetric : public PlannerBase::CostFunction
    {
    public:
        void Init(RobotBase* probot);
        virtual float Eval(const void* pConfiguration) { return 1; }
    };

    class SimpleGoalMetric : public PlannerBase::GoalFunction
    {
    public:
        
        SimpleGoalMetric() : PlannerBase::GoalFunction() { thresh = 0.01f; _robot = NULL; }

        //checks if pConf is within this cone (note: only works in 3D)
        float Eval(const void* c1)
        {
            assert( _robot != NULL && _robot->GetActiveManipulator() != NULL && _robot->GetActiveManipulator()->pEndEffector != NULL );
            
            _robot->SetActiveDOFValues(NULL,(const dReal *) c1);
            Transform cur = _robot->GetActiveManipulator()->GetEndEffectorTransform();

            return sqrtf(lengthsqr3(tgoal.trans - cur.trans));
        }
        virtual float GetGoalThresh() { return thresh; }
        virtual void SetRobot(RobotBase* robot) { _robot = robot; }

        Transform tgoal; // workspace goal

    private:
        float thresh;
        RobotBase* _robot;
    };

    class SimpleDistMetric : public PlannerBase::DistanceMetric
    {
    public:
        SimpleDistMetric() : PlannerBase::DistanceMetric() { thresh = 0.01f; _robot = NULL; }

        virtual void SetRobot(RobotBase* robot)
        {
            _robot = robot;
            if( _robot == NULL )
                return;

            weights.resize(0);
            vector<int>::const_iterator it;
            FORIT(it, _robot->GetActiveJointIndices()) weights.push_back(_robot->GetJointWeight(*it));
        }

        virtual float Eval(const void* c0, const void* c1)
        {
            assert( _robot->GetActiveDOF() == (int)weights.size() );

            float out = 0;
            for(int i=0; i < _robot->GetActiveDOF(); i++)
                out += weights[i] * (((dReal *)c0)[i]-((dReal *)c1)[i])*(((dReal *)c0)[i]-((dReal *)c1)[i]);
            
            return sqrtf(out);
        }

    private:
        vector<dReal> weights;
    };

    /// used to set the configuration state and gets its degrees of freedom
    class ActiveConfigurationState : public PlannerBase::ConfigurationState
    {
    public:
        ActiveConfigurationState() : robot(NULL) {}
        virtual void SetRobot(RobotBase* probot) { robot = probot; }
        virtual void SetState(const dReal* pstate) { robot->SetActiveDOFValues(NULL, pstate); }
        virtual void GetState(dReal* pstate) { robot->GetActiveDOFValues(pstate); }
        virtual void GetLimits(dReal* plower, dReal* pupper) { robot->GetActiveDOFLimits(plower, pupper); }
        
        virtual int GetDOF() const { return robot->GetActiveDOF(); }

    private:
        RobotBase* robot;
    };

    class SimpleSampler : public PlannerBase::SampleFunction
    {
    public:

        void Init(PlannerBase::ConfigurationState* pstate, PlannerBase::DistanceMetric* pmetric)
        {
            assert( pstate != NULL && pmetric != NULL );
            _pmetric = pmetric;
            pstate->GetLimits(lower, upper);
            range.resize(lower.size());
            for(int i = 0; i < (int)range.size(); ++i)
                range[i] = upper[i] - lower[i];
            _vSampleConfig.resize(lower.size());
            _vzero.resize(lower.size());
            memset(&_vzero[0],0,sizeof(_vzero[0])*_vzero.size());
        }
        virtual void Sample(dReal* pNewSample) {
            for (int i = 0; i < (int)lower.size(); i++) {
                pNewSample[i] = lower[i] + RANDOM_FLOAT()*range[i];
            }
        }

        virtual bool Sample(dReal* pNewSample, const dReal* pCurSample, dReal fRadius)
        {
            int dof = lower.size();
            for (int i = 0; i < dof; i++) {
                _vSampleConfig[i] = RANDOM_FLOAT()-0.5f;
            }
            
            // normalize
            dReal fRatio = RANDOM_FLOAT(fRadius);
            
            //assert(_robot->ConfigDist(&_vzero[0], &_vSampleConfig[0]) < B+1);
            while(_pmetric->Eval(&_vzero[0], &_vSampleConfig[0]) > fRatio ) {
                for (int i = 0; i < dof; i++) {
                    _vSampleConfig[i] *= 0.5f;
                }
            }
            
            while(_pmetric->Eval(&_vzero[0], &_vSampleConfig[0]) < fRatio ) {
                for (int i = 0; i < dof; i++) {
                    _vSampleConfig[i] *= 1.2f;
                }
            }

            // project to constraints
            for (int i = 0; i < dof; i++) {
                pNewSample[i] = CLAMP_ON_RANGE(pCurSample[i]+_vSampleConfig[i], lower[i], upper[i]);
            }

            return true;
        }

    private:
        PlannerBase::DistanceMetric* _pmetric;
        vector<dReal> lower, upper, range, _vSampleConfig, _vzero;
    };

    SimpleCostMetric _costmetric;
    SimpleGoalMetric _goalmetric;
    SimpleDistMetric _distmetric;
    SimpleSampler _defaultsampler;
    ActiveConfigurationState _defaultstate;
};

#endif
