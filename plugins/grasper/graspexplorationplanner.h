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

#ifndef RAVE_BISPACE_PLANNER
#define RAVE_BISPACE_PLANNER

#include <boost/shared_ptr.hpp>

class GraspExplorationPlanner : public PlannerBase
{
public:
    struct Node
    {
        Node() { parent = -1; }

        float fcost;
        int parent;
        dReal q[0]; // the configuration immediately follows the struct
    };

    // implement kd-tree or approx-nn in the future, deallocates memory from Node
    class SpatialTree
    {
    public:
        SpatialTree() { _planner = NULL; _fStepLength = 0.04f; _dof = 0; info = 0; _fBestDist = 0; _pDistMetric = NULL; _pSampleFn = NULL; _nodes.reserve(5000); }
        ~SpatialTree() { Reset(NULL, 0); }

        void Reset(GraspExplorationPlanner* planner, int dof=0);

        int AddNode(float fcost, int parent, const dReal* pfConfig);
        int GetNN(const dReal* q); ///< return the nearest neighbor

        /// samples random config and extends
        /// \return parent index, new config is in pNewConfig
        int Extend(dReal* pNewConfig);
        inline int GetDOF() { return _dof; }

        vector<Node*> _nodes;

        DistanceMetric* _pDistMetric;
        SampleFunction* _pSampleFn;

        dReal _fBestDist; ///< valid after a call to GetNN

        dReal _fStepLength;
        vector<dReal> _vLowerLimit, _vUpperLimit, _vDOFRange; ///< joint limitations
        vector<dReal> _jointResolution, _jointResolutionInv;
        int info; // TreeStatus mask
    
    private:
        int _dof;
        GraspExplorationPlanner* _planner;
    };

    enum IntervalType {
        OPEN = 0,
        OPEN_START,
        OPEN_END,
        CLOSED
    };

    GraspExplorationPlanner(EnvironmentBase* penv);
    ~GraspExplorationPlanner();

    void Destroy();
    void Reset();
    virtual RobotBase* GetRobot() const { return _robot; }

    /// InitPlan
    /// parameters format
    /// vinitialconfig: active dof values
    /// vgoalconfig:    ignored
    /// pdistmetric - distance between configurations
    virtual bool InitPlan(RobotBase* pbase, const PlannerParameters* pparams);
    virtual bool PlanPath(Trajectory* ptraj, std::ostream* pOutStream);

    virtual const wchar_t* GetDescription() const { return L"Grasp exploration planner"; }
    
private:

    bool _CheckCollision(const dReal* pQ0, const dReal* pQ1, IntervalType interval);
    bool _CheckCollision(const dReal* pq, bool bReport = false);

    int _AddFwdNode(int iparent, dReal* pconfig);

    void _SetRobotConfig(const dReal* pconfig);

    void _SetTime();
    dReal _GetTime(); // get elapsed time

    PlannerParameters _parameters;

    RobotBase* _robot;
    PlannerBase::DistanceMetric* _pConfigDistMetric;
    uint64_t _nBaseStartTime; ///< start 

    SpatialTree _configtree; ///< forward search tree

    dReal _fExploreProb;
    vector<dReal> _vSampleConfig;
    vector<dReal> _jointIncrement;
    
    vector<Transform> _vtransRobotStored;

    int nDumpIndex;
    const RobotBase::Manipulator* _pmanip;
    bool _bInit; ///< true if the planner has been initialized

    // default metrics
    class SimpleCostMetric : public PlannerBase::CostFunction
    {
    public:
        void Init(RobotBase* probot);
        virtual float Eval(const void* pConfiguration) { return fconst; }
        float fconst;
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
            else if( _robot->GetAffineDOF() & RobotBase::DOF_RotationQuat ) {
                weights.push_back(0.4f);
                weights.push_back(0.4f);
                weights.push_back(0.4f);
                weights.push_back(0.4f);
            }
        }

        virtual float Eval(const void* c0, const void* c1)
        {
            assert( _robot->GetActiveDOF() == (int)weights.size() );

            float out = 0;
            for(int i=0; i < _robot->GetActiveDOF(); i++)
                out += weights[i] * (((dReal *)c0)[i]-((dReal *)c1)[i])*(((dReal *)c0)[i]-((dReal *)c1)[i]);
            
            return sqrtf(out);
        }

        vector<dReal> weights;
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
    SimpleDistMetric _distmetric;
    ForwardSampleFunction _samplerforward;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()

BOOST_TYPEOF_REGISTER_TYPE(GraspExplorationPlanner)
BOOST_TYPEOF_REGISTER_TYPE(GraspExplorationPlanner::SpatialTree)
BOOST_TYPEOF_REGISTER_TYPE(GraspExplorationPlanner::Node)

#endif

#endif
