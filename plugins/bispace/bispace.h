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
// BiSpace: A planner that searches in both the configuration space and workspace at the same time. See
// Rosen Diankov, Nathan Ratliff, Dave Ferguson, Siddhartha Srinivasa, James Kuffner.
// BiSpace Planning: Concurrent Multi-Space Exploration. Robotics: Science and Systems Conference, June 2008

#ifndef RAVE_BISPACE_PLANNER
#define RAVE_BISPACE_PLANNER

#include <boost/shared_ptr.hpp>

class BiSpacePlanner : public PlannerBase
{
public:
    // the bispace planner has several modes to emulate other planning algorithms
    enum SearchType
    {
        ST_bispace=0,     ///< regular bispace algorithm (assumes backspace tree is in the workspace)
        ST_rrtjt,         ///< RRTjt implementation
        ST_rrtwork,       ///< RRT workspace goal bias
        ST_birrt,         ///< BiRRT implementation, backwpace is assumed to be configuration space (goals specified in config space)
    };

    enum SearchSpaceType
    {
        SST_3D=0, ///< just translation
        SST_6D,   ///< translation with quaternion (7 values!)
        SST_Active,
    };

    enum TreeStatus
    {
        TS_BackwardSearch=1,
    };

    enum NodeStatus
    {
        NS_FollowDead = 1,
        NS_ExtendDead = 2,
    };

    struct Node
    {
        Node() { parent = -1; info = 0; numext = 0; }

        //dReal fgoal;
        dReal fcost;
        int parent;
        short info; // if 1, node is dead, 2 if workspace node is dead
        short numext;
        dReal q[0]; // the configuration immediately follows the struct
    };

    // implement kd-tree or approx-nn in the future, deallocates memory from Node
    class SpatialTree
    {
    public:
        SpatialTree() { _planner = NULL; _fStepLength = 0.04f; _dof = 0; info = 0; _fBestDist = 0; _pDistMetric = NULL; _pSampleFn = NULL; _nodes.reserve(5000); }
        ~SpatialTree() { Reset(NULL, 0); }

        void Reset(BiSpacePlanner* planner, int dof=0);

        int AddNode(dReal fcost, int parent, const dReal* pfConfig);
        int GetNN(const dReal* q); ///< return the nearest neighbor

        /// samples random config and extends
        /// \return parent index, new config is in pNewConfig
        int Extend(dReal* pNewConfig);
        inline int GetDOF() { return _dof; }

        vector<Node*> _nodes;
        vector< vector<dReal> > _worknodes; // workspace goals of the nodes, valid for forward tree only

        DistanceMetric* _pDistMetric;
        SampleFunction* _pSampleFn;

        dReal _fBestDist; ///< valid after a call to GetNN

        dReal _fStepLength;
        SearchSpaceType _spacetype;
        vector<dReal> _vzero, _vLowerLimit, _vUpperLimit, _vDOFRange; ///< joint limitations
        vector<dReal> _jointResolution, _jointResolutionInv;
        int info; // TreeStatus mask
    
    private:
        int _dof;
        BiSpacePlanner* _planner;
    };

    enum IntervalType {
        OPEN = 0,
        OPEN_START,
        OPEN_END,
        CLOSED
    };

    class BiSpaceParameters : public PlannerParameters
    {
    public:
        BiSpaceParameters() : puserdata(NULL), EvalExtendDistance(NULL), EvalFollowProbability(NULL), pWorkspaceSampler(NULL) {}
        
        void* puserdata;
        
        /// distance metric from current robot config for the RRT extension step
        /// (weights all joints with respect to how close robot is to the goal)
        /// fCost - cost computed from cost metric
        dReal (*EvalExtendDistance)(void* puserdata, const dReal* q0, const dReal* q1, dReal fProximity);
        dReal (*EvalFollowProbability)(void* puserdata, const dReal* q, dReal fcost);
        PlannerBase::SampleFunction* pWorkspaceSampler;

    protected:
        virtual bool serialize(std::ostream& O) const;
        virtual bool endElement(void *ctx, const char *name);
    };

    BiSpacePlanner(EnvironmentBase* penv);
    ~BiSpacePlanner();

    void Destroy();
    void Reset();
    virtual RobotBase* GetRobot() const { return _robot; }

    /// InitPlan
    /// parameters format
    /// vnParameters:   *forward search type: 0 - rrt, 1 - cost rrt, 2 - RA*
    ///                 *backward space type: 0 - 3D translation, 1 - 6D trans/rotation, 2 - active dof
    ///                 manipulator index whose end effector effects the forward and backward search trees (tGrasp is ignored)
    ///                 *expansion multiplier
    ///                 *stochastic gradient samples
    /// vParameters:    *forward step length
    ///                 *backward step length
    ///                 *Configuration Follow Probability
    ///                 *Stochastic Follow Neighborhood Radius
    /// vinitialconfig: active dof values
    /// vgoalconfig:    goal values (depends on backward search type), size = N*dimension where N is the number of goals
    /// pdistmetric - distance between 
    /// pExtraParameters - BisapceParametersStruct*
    virtual bool InitPlan(RobotBase* pbase, const PlannerParameters* pparams);
    virtual bool PlanPath(Trajectory* ptraj, std::ostream* pOutStream);

    virtual const wchar_t* GetDescription() const;
    
private:

    bool _CheckCollision(const dReal* pQ0, const dReal* pQ1, IntervalType interval, bool bWorkspace, vector< vector<dReal> >* pvCheckedConfigurations=NULL);
    bool _CheckCollision(const dReal* pq, bool bWorkspace, bool bReport = false);
    void _OptimizePath(list<Node*>& path, int nTries, bool bWorkspace);
    void _OptimizeAcrossDimensions(list<Node*>& path, int nTries);

    int _AddFwdNode(int iparent, const dReal* pconfig);
    void _DumpNodes();

    void _SetRobotConfig(const dReal* pconfig, bool bWorkspace);

    /// if success, returns true and sets pforwardnode with the final configuration space node 
    bool _FollowPath(int& ifwdnode, Node* pworknode);
    bool _FollowPathJt(int& ifwdnode, Node* pworknode);

    /// uses the jacobian to approach
    /// \param ifwdnode Specify the node to start descending on. Once function terminates,
    ///        the ifwdnode is set to the node the search ended on.
    /// \return if target was successfully approached retursn true
    bool _ApproachTarget(const Transform& target, int& ifwdnode);
    void _GetJacboianTransposeStep(dReal* pnewconfig, const dReal* pcurconfig, dReal* ptarget, dReal fStep);

    void _SetWorkspaceFromFwdConfig(vector<dReal>& vworkconfig, const dReal* pfwdconfig);
    void _SetWorkspace3D(dReal* pworkconfig, const Transform& t);
    void _SetWorkspace6D(dReal* pworkconfig, const Transform& t);    

    void _StoreRobot();
    void _RestoreRobot();

    void _SetTime();
    dReal _GetTime(); // get elapsed time

    BiSpaceParameters _parameters;

    SpatialTree _configtree; ///< forward search tree
    SpatialTree _workspacetree; ///< backward search tree
    SearchType _searchtype;

    RobotBase* _robot;
    PlannerBase* _pbirrt;
    DistanceMetric* _pConfigDistMetric;
    uint64_t _nBaseStartTime; ///< start 

    vector<dReal> _vSampleConfig;
    vector<dReal> _jointIncrement;
    vector<dReal> _vzero;
    vector< pair<KinBody::Link*, Transform> > _vHandLinks; ///< links of the hand involved in workspace collision, each transformation is the offset from the workspace transformation

    vector< pair<int, int> > _vIKtoConfigMap; ///< used to transfer ik configuration solution to the active robot config

    vector<Transform> _vtransRobotStored, _vtransWorkGoals;
    int nNumBackGoals;

    dReal fConfigFollowProb, fStochasticFollowRadius, fGoalVariance;
    int nExpansionMultiplier, nStochasticGradSamples;

    vector<Transform> _vectrans; ///< cache

    int nDumpIndex;
    const RobotBase::Manipulator* _pmanip;
    bool _bInit; ///< true if the planner has been initialized
    bool _bWorkspaceCollision;

    // default metrics
    class SimpleCostMetric : public PlannerBase::CostFunction
    {
    public:
        void Init(RobotBase* probot);
        virtual float Eval(const void* pConfiguration) { return fconst; }
        dReal fconst;
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

            dReal ftransweight = 2;
            weights.resize(0);
            vector<int>::const_iterator it;
            FORIT(it, _robot->GetActiveJointIndices()) weights.push_back(_robot->GetJointWeight(*it));
            if( _robot->GetAffineDOF() & RobotBase::DOF_X ) weights.push_back(ftransweight);
            if( _robot->GetAffineDOF() & RobotBase::DOF_Y ) weights.push_back(ftransweight);
            if( _robot->GetAffineDOF() & RobotBase::DOF_Z ) weights.push_back(ftransweight);
            if( _robot->GetAffineDOF() & RobotBase::DOF_RotationAxis ) weights.push_back(ftransweight);
        }

        virtual float Eval(const void* c0, const void* c1)
        {
            assert( _robot->GetActiveDOF() == (int)weights.size() );

            dReal out = 0;
            for(int i=0; i < _robot->GetActiveDOF(); i++)
                out += weights[i] * (((dReal *)c0)[i]-((dReal *)c1)[i])*(((dReal *)c0)[i]-((dReal *)c1)[i]);
            
            return sqrtf(out);
        }

        vector<dReal> weights;
    };

    class ExtendDistMetric : public PlannerBase::DistanceMetric
    {
    public:
        ExtendDistMetric() : PlannerBase::DistanceMetric() { thresh = 0.01f; _robot = NULL; }
    };

    class Workspace6DDistMetric : public PlannerBase::DistanceMetric
    {
    public:
        Workspace6DDistMetric() : PlannerBase::DistanceMetric() { frotweight = 1; }

        virtual float Eval(const void* c0, const void* c1)
        {
            const dReal* pf0 = (const dReal*)c0;
            const dReal* pf1 = (const dReal*)c1;
            dReal frotdist1 = (*(Vector*)&pf0[3] - *(Vector*)&pf1[3]).lengthsqr4();
            dReal frotdist2 = (*(Vector*)&pf0[3] + *(Vector*)&pf1[3]).lengthsqr4();
            return sqrtf( (*(Vector*)pf0 - *(Vector*)pf1).lengthsqr3() + frotweight * min(frotdist1,frotdist2) );
        }

        dReal frotweight;
    };

    class Workspace3DDistMetric : public PlannerBase::DistanceMetric
    {
    public:
        Workspace3DDistMetric() : PlannerBase::DistanceMetric() {}

        virtual float Eval(const void* c0, const void* c1)
        {
            return sqrtf( (*(Vector*)c0 - *(Vector*)c1).lengthsqr3() );
        }
    };

    class ForwardSampleFunction : public PlannerBase::SampleFunction
    {
    public:
        ForwardSampleFunction() : _ptree(NULL), plower(NULL), pupper(NULL), prange(NULL) {}

        void Init(SpatialTree* ptree) {
            _ptree = ptree;
            dof = _ptree->GetDOF();
            plower = &ptree->_vLowerLimit;
            pupper = &ptree->_vUpperLimit;
            prange = &ptree->_vDOFRange;
        }
        virtual void Sample(dReal* pNewSample) {
            for (int i = 0; i < dof; i++) {
                pNewSample[i] = (*plower)[i] + RANDOM_FLOAT()*(*prange)[i];
            }
        }

        virtual bool Sample(dReal* pNewSample, const dReal* pCurSample, dReal fRadius) {

            if( pCurSample == NULL ) {
                Sample(pNewSample);
                return true;
            }

            for (int i = 0; i < dof; i++) {
                pNewSample[i] = pCurSample[i] + (RANDOM_FLOAT()-0.5f)*fRadius*_ptree->_jointResolution[i];
                if( pNewSample[i] < (*plower)[i] )
                    pNewSample[i] = (*plower)[i];
                else if( pNewSample[i] > (*pupper)[i] )
                    pNewSample[i] = (*pupper)[i];
            }
            return true;
        }

    private:
        int dof;
        SpatialTree* _ptree;
        vector<dReal>* plower, *pupper, *prange;
    };

    SimpleCostMetric _costmetric;
    SimpleGoalMetric _goalmetric;
    SimpleDistMetric _distmetric;
    Workspace6DDistMetric _work6ddistmetric;
    Workspace3DDistMetric _work3ddistmetric;
    ForwardSampleFunction _samplerforward, _samplerback;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()

BOOST_TYPEOF_REGISTER_TYPE(BiSpacePlanner)
BOOST_TYPEOF_REGISTER_TYPE(BiSpacePlanner::SpatialTree)
BOOST_TYPEOF_REGISTER_TYPE(BiSpacePlanner::Node)

#endif

#endif
