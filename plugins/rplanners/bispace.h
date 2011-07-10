// -*- coding: utf-8 -*-
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
//
// BiSpace: A planner that searches in both the configuration space and workspace at the same time. See
// Rosen Diankov, Nathan Ratliff, Dave Ferguson, Siddhartha Srinivasa, James Kuffner.
// BiSpace Planning: Concurrent Multi-Space Exploration. Robotics: Science and Systems Conference, June 2008
#ifndef RAVE_BISPACE_PLANNER
#define RAVE_BISPACE_PLANNER

#include <algorithm>
// planning constants
#define JACOBIAN_TRANS_MAXSTEP 0.01f
#define JACOBIAN_TRANS_THRESH 0.002f
#define GOAL_THRESH (0.07)

//#define JACOBIAN_ROT_MAXSTEP 0.06f
//#define JACOBIAN_ROT_THRESH 0.02f

class BiSpaceParameters : public PlannerParameters
{
public:
    BiSpaceParameters() : puserdata(NULL), EvalExtendDistance(NULL), EvalFollowProbability(NULL), pWorkspaceSampler(NULL) {
    }

    void* puserdata;

    /// distance metric from current robot config for the RRT extension step
    /// (weights all joints with respect to how close robot is to the goal)
    /// fCost - cost computed from cost metric
    dReal (*EvalExtendDistance)(void* puserdata, const dReal* q0, const dReal* q1, dReal fProximity);
    dReal (*EvalFollowProbability)(void* puserdata, const dReal* q, dReal fcost);
    PlannerBase::SampleFunction* pWorkspaceSampler;

protected:
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;

        O << "<EvalExtendDistance>" << (void*)EvalExtendDistance << "</EvalExtendDistance>" << endl;
        O << "<EvalFollowProbability>" << (void*)EvalFollowProbability << "</EvalFollowProbability>" << endl;
        O << "<WorkspaceSampler>" << (void*)pWorkspaceSampler << "</WorkspaceSampler>" << endl;
        O << "<userdata>" << puserdata << "</userdata>" << endl;

        return !!O;
    }
    virtual bool endElement(void *ctx, const char *name)
    {
        if( stricmp(name, "EvalExtendDistance") == 0 ) {
            void* p; _ss >> p;
            *(void**)&EvalExtendDistance = p;
        }
        else if( stricmp(name, "EvalFollowProbability") == 0 ) {
            void* p; _ss >> p;
            *(void**)&EvalFollowProbability = p;
        }
        else if( stricmp(name, "WorkspaceSampler") == 0 ) {
            void* p; _ss >> p;
            *(void**)&pWorkspaceSampler = p;
        }
        else if( stricmp(name, "userdata") == 0 )
            _ss >> puserdata;
        else
            return PlannerParameters::endElement(ctx, name);

        return false;
    }
};

class BiSpacePlanner : public PlannerBase
{
    static dReal DummyEvalFollowProbability(void* puserdata, const dReal* q, dReal fcost) {
        return 1;
    }
public:
    // the bispace planner has several modes to emulate other planning algorithms
    enum SearchType
    {
        ST_bispace=0,         ///< regular bispace algorithm (assumes backspace tree is in the workspace)
        ST_rrtjt,             ///< RRTjt implementation
        ST_rrtwork,           ///< RRT workspace goal bias
        ST_birrt,             ///< BiRRT implementation, backwpace is assumed to be configuration space (goals specified in config space)
    };

    enum SearchSpaceType
    {
        SST_3D=0,     ///< just translation
        SST_6D,       ///< translation with quaternion (7 values!)
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
        Node() {
            parent = -1; info = 0; numext = 0;
        }

        //dReal fgoal;
        dReal fcost;
        int parent;
        short info;     // if 1, node is dead, 2 if workspace node is dead
        short numext;
        dReal q[0];     // the configuration immediately follows the struct
    };

    // implement kd-tree or approx-nn in the future, deallocates memory from Node
    class SpatialTree
    {
public:
        SpatialTree() {
            _planner = NULL; _fStepLength = 0.04f; _dof = 0; info = 0; _fBestDist = 0; _pDistMetric = NULL; _pSampleFn = NULL; _nodes.reserve(5000);
        }
        ~SpatialTree() {
            Reset(NULL, 0);
        }

        void Reset(BiSpacePlanner* planner, int dof=0)
        {
            _planner = planner;

            vector<Node*>::iterator it;
            FORIT(it, _nodes) {
                (*it)->~Node();
                free(*it);
            }
            _nodes.resize(0);
            _worknodes.resize(0);
            if( dof > 0 ) {
                _dof = dof;
                _vLowerLimit.resize(_dof);
                _vUpperLimit.resize(_dof);
                _vDOFRange.resize(_dof);
                _vzero.resize(_dof);
            }
        }

        int AddNode(dReal fcost, int parent, const dReal* pfConfig)
        {
            BOOST_ASSERT( pfConfig != NULL );

            void* pmem = malloc(sizeof(Node)+sizeof(dReal)*_dof);
            Node* p = ::new (pmem) Node();        // call constructor explicitly
            p->parent = parent;
            memcpy(p->q, pfConfig, sizeof(dReal)*_dof);
            p->fcost = fcost;
            _nodes.push_back(p);
            _worknodes.push_back(vector<dReal>());
            return (int)_nodes.size()-1;
        }

        int GetNN(const dReal* q)
        {
            DVSTARTPROFILE();

            BOOST_ASSERT( _pDistMetric!= NULL );
            if( _nodes.size() == 0 )
                return -1;

            vector<Node*>::iterator itnode = _nodes.begin();
            int ibest = -1;
            dReal fbest = 0;

            if(( _planner->_searchtype == ST_bispace) && !(info & TS_BackwardSearch) &&( _planner->_parameters.EvalExtendDistance != NULL) ) {
                // use the special distance metric
                while(itnode != _nodes.end()) {

                    //if( !((*itnode)->info & NS_Dead) ) {
                    //if( (*itnode)->info == 0 || RANDOM_DREAL() < (*itnode)->fcost ) {
                    //if( (*itnode)->numext < 3 )
                    {
                        dReal f = _planner->_parameters.EvalExtendDistance(_planner->_parameters.puserdata, q, (*itnode)->q, (*itnode)->fcost);
                        if(( ibest < 0) ||( f < fbest) ) {
                            ibest = (int)(itnode-_nodes.begin());
                            fbest = f;
                        }
                    }
                    ++itnode;
                }

                if( ibest < 0 ) ibest = 0;
                //_nodes[ibest]->numext++;

                //        if( ibest < 0 ) {
                //            itnode = _nodes.begin();
                //            while(itnode != _nodes.end()) {
                //
                //                dReal f = _planner->_parameters.EvalExtendDistance(_planner->_parameters.puserdata, q, (*itnode)->q, (*itnode)->fcost);
                //                if( ibest < 0 || f < fbest ) {
                //                    ibest = (int)(itnode-_nodes.begin());
                //                    fbest = f;
                //                }
                //                ++itnode;
                //            }
                //        }
            }
            else {
                if(( _planner->_searchtype == ST_rrtwork) &&( RANDOM_FLOAT() < _planner->fConfigFollowProb) ) {
                    BOOST_ASSERT(!(info & TS_BackwardSearch));
                    BOOST_ASSERT(_worknodes.size() == _nodes.size() );

                    // sample a goal node and take the closest neighbor
                    int index = RANDOM_INT((int)_planner->_vtransWorkGoals.size());
                    dReal* pworkconfig = _planner->_workspacetree._nodes[index]->q;
                    vector< vector<dReal> >::iterator itworknode = _worknodes.begin();

                    while(itworknode != _worknodes.end()) {

                        dReal f = _planner->_workspacetree._pDistMetric->Eval(pworkconfig, &(*itworknode)[0]);
                        if(( ibest < 0) ||( f < fbest) ) {
                            ibest = (int)(itworknode-_worknodes.begin());
                            fbest = f;
                        }
                        ++itworknode;
                    }
                }
                else {
                    while(itnode != _nodes.end()) {

                        //if( !((*itnode)->info & NS_Dead) ) {
                        dReal f = _pDistMetric->Eval(q, (*itnode)->q);
                        if(( ibest < 0) ||( f < fbest) ) {
                            ibest = (int)(itnode-_nodes.begin());
                            fbest = f;
                        }
                        //}
                        ++itnode;
                    }
                }
            }

            _fBestDist = fbest;
            return ibest;
        }

        /// samples random config and extends
        /// \return parent index, new config is in pNewConfig
        int Extend(dReal* pNewConfig)
        {
            DVSTARTPROFILE();

            BOOST_ASSERT( _pSampleFn != NULL );
            _pSampleFn->Sample(pNewConfig);

            // get the nearest neighbor
            int inode = GetNN(pNewConfig);
            BOOST_ASSERT(inode >= 0 );
            Node* pnode = _nodes[inode];

            // extend
            dReal fdist;
            if(( _planner->_searchtype == ST_bispace) && !(info & TS_BackwardSearch) &&( _planner->_parameters.EvalExtendDistance != NULL) ) {
                fdist = _planner->_parameters.EvalExtendDistance(_planner->_parameters.puserdata, pnode->q, pNewConfig, pnode->fcost);
            }
            else {
                fdist = _pDistMetric->Eval(pnode->q, pNewConfig);
            }

            if( fdist > _fStepLength ) fdist = _fStepLength / fdist;
            else fdist = 1;

            for(int i = 0; i < _dof; ++i)
                pNewConfig[i] = pnode->q[i] + (pNewConfig[i]-pnode->q[i])*fdist;

            if( _spacetype == SST_6D ) {
                normalize4(pNewConfig+3,pNewConfig+3);
            }
            else if( _spacetype == SST_Active ) {
                // project to constraints
                if( _planner->_parameters.pconstraintfn != NULL ) {
                    if( !_planner->_parameters.pconstraintfn->Constraint(pnode->q, pNewConfig, NULL, 0) ) {
                        // sample again
                        return Extend(pNewConfig);
                    }
                }
            }

            return inode;
        }

        inline int GetDOF() {
            return _dof;
        }

        vector<Node*> _nodes;
        vector< vector<dReal> > _worknodes;         // workspace goals of the nodes, valid for forward tree only

        DistanceMetric* _pDistMetric;
        SampleFunction* _pSampleFn;

        dReal _fBestDist;         ///< valid after a call to GetNN

        dReal _fStepLength;
        SearchSpaceType _spacetype;
        vector<dReal> _vzero, _vLowerLimit, _vUpperLimit, _vDOFRange;         ///< joint limitations
        vector<dReal> _jointResolution, _jointResolutionInv;
        int info;         // TreeStatus mask

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

    BiSpacePlanner(EnvironmentBasePtr penv) : PlannerBase(penv)
    {
        _workspacetree.info = TS_BackwardSearch;
        nDumpIndex = 0;
        _pConfigDistMetric = NULL;
        _pmanip = NULL;
        nStochasticGradSamples = 64;
        fStochasticFollowRadius = 20.0f;
        nExpansionMultiplier = 4;
        fConfigFollowProb = 0.1f;
        fGoalVariance = 1/(0.3f*0.3f);
        nNumBackGoals = 0;
        _pbirrt = NULL;
    }

    ~BiSpacePlanner();

    void Reset()
    {
        _configtree.Reset(this, -1);
        _workspacetree.Reset(this, -1);
    }

    virtual RobotBasePtr GetRobot() {
        return _robot;
    }

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
    virtual bool InitPlan(RobotBase* pbase, const PlannerParameters* pparams)
    {
        _SetTime();

        nDumpIndex = 0;
        Destroy();
        _robot = pbase;
        _pmanip = NULL;

        if( pparams != NULL )
            _parameters.copy(*pparams);

        if( _robot == NULL )
            return false;

        if( _pbirrt == NULL ) {
            _pbirrt = GetEnv()->CreatePlanner(L"rBiRRT");
            if( _pbirrt == NULL )
                RAVEPRINT(L"failed to init sub-birrt\n");
        }

        if( _parameters.pgoalfn == NULL )
            _parameters.pgoalfn = &_goalmetric;
        if( _parameters.pcostfn == NULL )
            _parameters.pcostfn = &_costmetric;
        if( _parameters.pdistmetric == NULL )
            _parameters.pdistmetric = &_distmetric;

        dReal fTransResolution = 0.005f;
        _searchtype = ST_bispace;
        _configtree._spacetype = SST_Active;
        _workspacetree._spacetype = SST_6D;
        if( _parameters.vnParameters.size() > 0 )
            _searchtype = (SearchType)_parameters.vnParameters[0];
        if( _parameters.vnParameters.size() > 1 )
            _workspacetree._spacetype = (SearchSpaceType)_parameters.vnParameters[1];
        if( _parameters.vnParameters.size() > 2 )
            nExpansionMultiplier = _parameters.vnParameters[2];
        if( _parameters.vnParameters.size() > 3 )
            nStochasticGradSamples = _parameters.vnParameters[3];

        if( _parameters.vParameters.size() > 0 )
            _configtree._fStepLength = _parameters.vParameters[0];
        if( _parameters.vParameters.size() > 1 )
            _workspacetree._fStepLength = _parameters.vParameters[1];
        if( _parameters.vParameters.size() > 2 )
            fConfigFollowProb = _parameters.vParameters[2];
        if( _parameters.vParameters.size() > 3 )
            fStochasticFollowRadius = _parameters.vParameters[3];
        if( _parameters.vParameters.size() > 4 )
            fTransResolution = _parameters.vParameters[4];

        if( _parameters.EvalFollowProbability == NULL )
            _parameters.EvalFollowProbability = DummyEvalFollowProbability;

        _costmetric.fconst = 1;    //fConfigFollowProb;
        _work6ddistmetric.frotweight = 0.2f;

        if(( _workspacetree._spacetype == SST_3D) ||( _workspacetree._spacetype == SST_6D) ) {
            if( _robot->GetActiveManipulator() == NULL ) {
                RAVEPRINT(L"no active manip\n");
                return false;
            }
        }
        _parameters.pgoalfn->SetRobot(pbase);
        _parameters.pdistmetric->SetRobot(pbase);

        _configtree.Reset(this, _robot->GetActiveDOF());

        _vSampleConfig.resize(max(7,_robot->GetActiveDOF()));
        _jointIncrement.resize(_robot->GetActiveDOF());
        _vzero.resize(_robot->GetActiveDOF());
        memset(&_vzero[0], 0, sizeof(dReal)*_robot->GetActiveDOF());

        // compute environment bounding box
        Vector venvmin, venvmax;
        {
            AABB ab = _robot->ComputeAABB();
            venvmin = ab.pos - ab.extents;
            venvmax = ab.pos + ab.extents;

            FOREACHC(itbody, GetEnv()->GetBodies()) {
                if( (*itbody)->IsEnabled() ) {
                    ab = (*itbody)->ComputeAABB();
                    if( venvmin.x > ab.pos.x - ab.extents.x ) venvmin.x = ab.pos.x - ab.extents.x;
                    if( venvmin.y > ab.pos.y - ab.extents.y ) venvmin.y = ab.pos.y - ab.extents.y;
                    if( venvmin.z > ab.pos.z - ab.extents.z ) venvmin.z = ab.pos.z - ab.extents.z;
                    if( venvmax.x < ab.pos.x + ab.extents.x ) venvmax.x = ab.pos.x + ab.extents.x;
                    if( venvmax.y < ab.pos.y + ab.extents.y ) venvmax.y = ab.pos.y + ab.extents.y;
                    if( venvmax.z < ab.pos.z + ab.extents.z ) venvmax.z = ab.pos.z + ab.extents.z;
                }
            }

            const dReal fext=3;
            venvmin -= Vector(fext,fext,fext);
            venvmax += Vector(fext,fext,fext);
        }

        // forward search limits
        _robot->GetActiveDOFLimits(_configtree._vLowerLimit, _configtree._vUpperLimit);
        // replace with better affine limits
        RobotBase::DOFAffine affinedofs[3] = { RobotBase::DOF_X, RobotBase::DOF_Y, RobotBase::DOF_Z};
        for(int i = 0; i < 3; ++i) {
            if( _robot->GetAffineDOF() & affinedofs[i] ) {
                _configtree._vLowerLimit[_robot->GetAffineDOFIndex(affinedofs[i])] = venvmin[i];
                _configtree._vUpperLimit[_robot->GetAffineDOFIndex(affinedofs[i])] = venvmax[i];
            }
        }

        if( _robot->GetAffineDOF() & RobotBase::DOF_RotationAxis ) {
            _configtree._vLowerLimit[_robot->GetAffineDOFIndex(RobotBase::DOF_RotationAxis)] = 0;
            _configtree._vUpperLimit[_robot->GetAffineDOFIndex(RobotBase::DOF_RotationAxis)] = 2*PI;
        }

        _configtree._vDOFRange.resize(_configtree._vLowerLimit.size());
        for(size_t i = 0; i < _configtree._vLowerLimit.size(); ++i)
            _configtree._vDOFRange[i] = _configtree._vUpperLimit[i] - _configtree._vLowerLimit[i];

        _configtree._pDistMetric = _pConfigDistMetric = _parameters.pdistmetric;
        _configtree._pSampleFn = _parameters.pSampleFn != NULL ? _parameters.pSampleFn : &_samplerforward;
        _workspacetree._pSampleFn = &_samplerback;
        if( _parameters.pWorkspaceSampler != NULL )
            _workspacetree._pSampleFn = _parameters.pWorkspaceSampler;

        // set backsearch metrics and limits
        switch(_workspacetree._spacetype) {
        case SST_3D:
        {
            _workspacetree._pDistMetric = &_work3ddistmetric;
            _workspacetree.Reset(this,3);
            *(Vector*)&_workspacetree._vLowerLimit[0] = venvmin;
            *(Vector*)&_workspacetree._vUpperLimit[0] = venvmax;
            *(Vector*)&_workspacetree._vDOFRange[0] = venvmax-venvmin;
            break;
        }
        case SST_6D:
        {
            _workspacetree._pDistMetric = &_work6ddistmetric;
            _workspacetree.Reset(this, 7);
            for(int i = 0; i < 7; ++i) {
                _workspacetree._vLowerLimit[i] = -1;
                _workspacetree._vUpperLimit[i] = 1;
                _workspacetree._vDOFRange[i] = 2;
            }
            *(Vector*)&_workspacetree._vLowerLimit[0] = venvmin;
            *(Vector*)&_workspacetree._vUpperLimit[0] = venvmax;
            *(Vector*)&_workspacetree._vDOFRange[0] = venvmax-venvmin;
            break;
        }
        case SST_Active:
            _workspacetree._pDistMetric = _configtree._pDistMetric;
            _workspacetree.Reset(this, _robot->GetActiveDOF());
            _workspacetree._vLowerLimit = _configtree._vLowerLimit;
            _workspacetree._vUpperLimit = _configtree._vUpperLimit;
            _workspacetree._vDOFRange = _configtree._vDOFRange;
            break;
        default:
            RAVEPRINT(L"bad space: %d\n", _workspacetree._spacetype);
            return false;
        }

        _robot->GetActiveDOFResolutions(_configtree._jointResolution);
        int doftrans = (int)_robot->GetActiveJointIndices().size();
        if( _robot->GetAffineDOF() & RobotBase::DOF_X ) _configtree._jointResolution[doftrans++] = fTransResolution;
        if( _robot->GetAffineDOF() & RobotBase::DOF_Y ) _configtree._jointResolution[doftrans++] = fTransResolution;
        if( _robot->GetAffineDOF() & RobotBase::DOF_Z ) _configtree._jointResolution[doftrans++] = fTransResolution;

        _configtree._jointResolutionInv.resize(0);
        FOREACH(itj, _configtree._jointResolution) {
            _configtree._jointResolutionInv.push_back(*itj != 0  ? 1 / *itj : 1.0f);
        }

        if( _workspacetree._spacetype == SST_Active ) {
            _workspacetree._jointResolution = _configtree._jointResolution;
            _workspacetree._jointResolutionInv = _configtree._jointResolutionInv;
        }
        else {
            _workspacetree._jointResolution.resize(_workspacetree.GetDOF());
            _workspacetree._jointResolutionInv.resize(_workspacetree.GetDOF());
            dReal ftrans = 0.004f, frot = 0.015f;
            for(int i = 0; i < 3; ++i) {
                _workspacetree._jointResolution[i] = ftrans;
                _workspacetree._jointResolutionInv[i] = 1/ftrans;
            }
            if( _workspacetree._spacetype == SST_6D ) {
                for(int i = 3; i < 7; ++i) {
                    _workspacetree._jointResolution[i] = frot;
                    _workspacetree._jointResolutionInv[i] = 1/frot;
                }
            }
        }


        // if a link is not affected by any of the unused indices, then disable it during workspace trajectories
        _pmanip = _robot->GetActiveManipulator();
        _vHandLinks.clear();

        if( _workspacetree._spacetype != SST_Active ) {

            BOOST_ASSERT( _pmanip != NULL );
            vector<int>::const_iterator itjoint;

            Transform tEEinv = _pmanip->GetTransform().inverse();

            FOREACHC(itlink, _robot->GetLinks()) {

                FORIT(itjoint, _robot->GetActiveJointIndices()) {
                    if( !_robot->DoesAffect(*itjoint, (*itlink)->GetIndex()) )
                        break;
                }

                if( itjoint == _robot->GetActiveJointIndices().end() )
                    // part of hand
                    _vHandLinks.push_back( pair<KinBody::Link*, Transform>(*itlink, tEEinv * (*itlink)->GetTransform()) );
            }

            // make sure the end effector is always in the list
            bool bFound = false;
            FOREACH(ithandlink, _vHandLinks) {
                if( ithandlink->first == _pmanip->pEndEffector ) {
                    bFound = true;
                    break;
                }
            }

            if( !bFound ) {
                RAVEPRINT(L"End effector %S not part of auto-detected hand links!\n", _pmanip->pEndEffector->GetName());
                return false;
            }
        }

        _vIKtoConfigMap.clear();
        if(( _pmanip != NULL) && _pmanip->HasIKSolver() ) {
            FOREACHC(itactive, _robot->GetActiveJointIndices()) {
                vector<int>::const_iterator itik = find(_pmanip->_vecarmjoints.begin(), _pmanip->_vecarmjoints.end(), *itactive);
                if( itik != _pmanip->_vecarmjoints.end() )
                    _vIKtoConfigMap.push_back(pair<int,int>((int)(itactive-_robot->GetActiveJointIndices().begin()), (int)(itik-_pmanip->_vecarmjoints.begin())));
            }
        }

        _samplerforward.Init(&_configtree);
        _samplerback.Init(&_workspacetree);
        Reset();

        // forward
        Node* pcurrent = _configtree._nodes[_AddFwdNode(-1, &_parameters.vinitialconfig[0])];
        //pcurrent->fgoal = 0;

        _vtransWorkGoals.resize(0);

        // backward
        for(int i = 0; i < (int)_parameters.vgoalconfig.size(); ) {
            pcurrent = _workspacetree._nodes[_workspacetree.AddNode(0, -1, &_parameters.vgoalconfig[i])];
            //pcurrent->fgoal = 0;

            if( _workspacetree._spacetype == SST_3D ) {
                _vtransWorkGoals.push_back(Transform(Vector(1,0,0,0), Vector(pcurrent->q)));
                i += 3;
            }
            else if( _workspacetree._spacetype == SST_6D ) {
                i += 7;
                _vtransWorkGoals.push_back(Transform(Vector(pcurrent->q[3], pcurrent->q[4], pcurrent->q[5], pcurrent->q[6]), Vector(pcurrent->q)));
            }
            else i += _robot->GetActiveDOF();
        }

        nNumBackGoals = (int)_workspacetree._nodes.size();

        wstringstream ss;
        ss << "----BiSpace Planner----" << endl;
        ss << "expmult: " << nExpansionMultiplier << endl
           << "gradsamples: " << nStochasticGradSamples << endl
           << "configstep: " << _configtree._fStepLength << endl
           << "workstep: " << _workspacetree._fStepLength << endl
           << "followprob: " << fConfigFollowProb << endl
           << "followradius: " << fStochasticFollowRadius << endl
           << "maxiter: " << _parameters.nMaxIterations << endl;
        RAVEPRINT(ss.str().c_str());

        _bInit = true;
        return nNumBackGoals > 0;
    }

    virtual bool PlanPath(Trajectory* ptraj, std::ostream* pOutStream)
    {
        BOOST_ASSERT( _robot != NULL && _parameters.pgoalfn!= NULL && _parameters.pcostfn != NULL && _parameters.pdistmetric != NULL );

        if( !_bInit )
            return false;

        Node *pbestconfig=NULL;

        _StoreRobot();
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        _parameters.pgoalfn->SetRobot(_robot);
        _parameters.pdistmetric->SetRobot(_robot);

        _pmanip = _robot->GetActiveManipulator();

        if(( _pmanip == NULL) && (( _workspacetree._spacetype == SST_3D) ||( _workspacetree._spacetype == SST_6D) ) ) {
            RAVEPRINT(L"no manipulator specified\n");
            return false;
        }

        _robot->SetActiveDOFValues(NULL, &_parameters.vinitialconfig[0]);
        CollisionReport report;
        if( GetEnv()->CheckCollision(_robot, &report) ) {
            RAVEPRINT(L"BiSpace: robot initially in collision %S:%S!\n",
                      report.plink1!=NULL ? report.plink1->GetName() : L"(NULL)",
                      report.plink2!=NULL ? report.plink2->GetName() : L"(NULL)");
            _RestoreRobot();
            return false;
        }
        else if( _robot->CheckSelfCollision() ) {
            RAVEPRINT(L"BiSpace: robot self-collision!\n");
            _RestoreRobot();
            return false;
        }

        // check all goals
        vector<Node*>::iterator itgoal = _workspacetree._nodes.begin();
        while(itgoal != _workspacetree._nodes.end()) {
            if( _CheckCollision((*itgoal)->q, true, true) ) {
                RAVEPRINT(L"goal in collision!\n");
                itgoal = _workspacetree._nodes.erase(itgoal);
            }
            else ++itgoal;
        }

        if( _workspacetree._nodes.size() == 0 ) {
            _RestoreRobot();
            return false;
        }

        _bWorkspaceCollision = false;

        int nMaxIter = _parameters.nMaxIterations > 0 ? _parameters.nMaxIterations : 8000;
        bool bSampleForward = true;
        int iter = 0;

        vector<dReal> vworkconfig(_workspacetree.GetDOF()), vworkdelta(_workspacetree.GetDOF()), vworktemp(_workspacetree.GetDOF());
        dReal* ptargworkconfig;

        DVProfClear();
        dReal fbest = 10000;     // for stats

        while(iter < nMaxIter && pbestconfig == NULL ) {

            int iwork = -1;
            int iconfignode = -1;

            // still growing the two trees
            if( bSampleForward ) {

                int inode = _configtree.Extend(&_vSampleConfig[0]);

                if(( inode >= 0) && !_CheckCollision(_configtree._nodes[inode]->q, &_vSampleConfig[0], OPEN_START, false) ) {
                    inode = _AddFwdNode(inode, &_vSampleConfig[0]);

                    switch(_searchtype) {
                    case ST_bispace:
                        // check if should FollowPath
                        if( RANDOM_FLOAT() < fConfigFollowProb*_parameters.EvalFollowProbability(_parameters.puserdata, NULL, _configtree._nodes[inode]->fcost) ) {
                            iconfignode = inode;
                            if( _pmanip != NULL ) {
                                ptargworkconfig = &vworktemp[0];
                                if( _workspacetree._spacetype == SST_3D ) _SetWorkspace3D(ptargworkconfig, _pmanip->GetTransform());
                                else {
                                    BOOST_ASSERT( _workspacetree._spacetype == SST_6D );
                                    _SetWorkspace6D(ptargworkconfig, _pmanip->GetTransform());
                                }
                            }
                            else ptargworkconfig = &_vSampleConfig[0];
                            // connect
                            iwork = _workspacetree.GetNN(ptargworkconfig);

                            // this is REALLY necessary, perhaps make the 0.7 a parameter??
                            if( lengthsqr3(*(Vector*)_workspacetree._nodes[iwork]->q - *(Vector*)ptargworkconfig) > 1 )
                                iwork = -1;
                            //else RAVEPRINT(L"accepted\n");
                        }
                        break;

                    //                        case ST_rrtjt:
                    //                            if( RANDOM_FLOAT() < fConfigFollowProb ) {
                    //                                int jiter = 0;
                    //                                while(jiter++ < 100) {
                    //                                    _GetJacboianTransposeStep(&vnewconfig[0], _configtree._nodes[inode]->q, _workspacetree._nodes[index]->q, 0.2f*_configtree._fStepLength);
                    //                                    if( _CheckCollision(qprev, &vnewconfig[0], OPEN_START, false) )
                    //                                        break;
                    //
                    //                                    inode = _AddFwdNode(inode, &vnewconfig[0]);
                    //                                    qprev = _configtree._nodes[inode]->q;
                    //                                    _configtree._nodes[inode]->info = 1; // make sure never to sample from it again
                    //                                }
                    //                            }
                    case ST_rrtwork:
                    {
                        // check for end of goal
                        for(size_t i = 0; i < _vtransWorkGoals.size(); ++i) {
                            dReal fdist = _workspacetree._pDistMetric->Eval(&_configtree._worknodes[inode][0], _workspacetree._nodes[i]->q);
                            if( fdist < GOAL_THRESH ) {
                                // good enough, perhaps use Jacobian to merge even further
                                dReal* q = _workspacetree._nodes[i]->q;
                                _ApproachTarget(Transform(Vector(q[3],q[4],q[5],q[6]),Vector(q[0],q[1],q[2])), inode);
                                pbestconfig = _configtree._nodes[inode];
                                break;
                            }

                            if( fbest > fdist )
                                fbest = fdist;
                        }
                    }
                    break;
                    default:
                        break;
                    }
                }

                if( _searchtype == ST_bispace )
                    bSampleForward = false;
            }
            else {
                // expand
                for(int i = 0; i < nExpansionMultiplier; ++i) {
                    int inode = _workspacetree.Extend(&_vSampleConfig[0]);

                    if(( inode >= 0) && !_CheckCollision(_workspacetree._nodes[inode]->q, &_vSampleConfig[0], OPEN_START, true) ) {
                        _workspacetree.AddNode(0, inode, &_vSampleConfig[0]);
                    }
                }

                bSampleForward = true;     // revert
            }

            if( (( iwork >= 0) &&( iconfignode >= 0) ) ) {
                // use the workspace path to bias
                if( _FollowPath(iconfignode, _workspacetree._nodes[iwork]) ) {
                    // done
                    pbestconfig = _configtree._nodes[iconfignode];
                    break;
                }

                // break the connection
                _DumpNodes();
            }
            else if( 0&&(_searchtype == ST_bispace)&&(RANDOM_FLOAT() < fConfigFollowProb)) {
                // choose a random goal
                int index = RANDOM_INT((int)_vtransWorkGoals.size());

                // pick the nearest node in configuration space
                int inode = -1;
                dReal fbestdist = 10000;
                for(size_t i = 0; i < _configtree._worknodes.size(); ++i) {

                    if( !(_configtree._nodes[i]->info&NS_FollowDead) ) {
                        dReal fdist = _workspacetree._pDistMetric->Eval(&_configtree._worknodes[i][0], _workspacetree._nodes[index]->q);
                        if( fbestdist > fdist ) {
                            inode = i;
                            fbestdist = fdist;
                        }
                    }
                }

                if(( inode >= 0) &&( RANDOM_FLOAT() < _configtree._nodes[inode]->fcost) ) {
                    // use the workspace path to bias
                    if( _FollowPath(inode, _workspacetree._nodes[index]) ) {
                        // done
                        pbestconfig = _configtree._nodes[inode];
                        break;
                    }
                }
            }
            // check if should follow the Jt
            else if( (/*_searchtype == ST_bispace || */ _searchtype == ST_rrtjt) &&(RANDOM_FLOAT() < fConfigFollowProb)) {
                // choose a random goal
                int index = RANDOM_INT((int)_vtransWorkGoals.size());

                // pick the nearest node in configuration space
                int inode = 0;
                dReal fbestdist = 10000;
                for(size_t i = 0; i < _configtree._worknodes.size(); ++i) {

                    if( !(_configtree._nodes[i]->info&NS_FollowDead) ) {
                        dReal fdist = _workspacetree._pDistMetric->Eval(&_configtree._worknodes[i][0], _workspacetree._nodes[index]->q);
                        if( fbestdist > fdist ) {
                            inode = i;
                            fbestdist = fdist;
                        }
                    }
                }

                // follow the Jt gradient until collision
                static vector<dReal> vnewconfig; vnewconfig.resize(_configtree.GetDOF());
                dReal* qprev = _configtree._nodes[inode]->q;

                _configtree._nodes[inode]->info |= NS_FollowDead;     // make sure never to sample from it again

                int jiter = _searchtype == ST_rrtjt ? 100 : 10;
                while(jiter-- > 0) {
                    _GetJacboianTransposeStep(&vnewconfig[0], _configtree._nodes[inode]->q, _workspacetree._nodes[index]->q, 0.2f*_configtree._fStepLength);
                    if( _CheckCollision(qprev, &vnewconfig[0], OPEN_START, false) )
                        break;

                    inode = _AddFwdNode(inode, &vnewconfig[0]);
                    qprev = _configtree._nodes[inode]->q;
                    _configtree._nodes[inode]->info |= NS_FollowDead;     // make sure never to sample from it again
                }

                // check for end of goal
                for(size_t i = 0; i < _vtransWorkGoals.size(); ++i) {
                    dReal fdist = _workspacetree._pDistMetric->Eval(&_configtree._worknodes[inode][0], _workspacetree._nodes[i]->q);
                    if( fdist < GOAL_THRESH ) {
                        // good enough, perhaps use Jacobian to merge even further
                        dReal* q = _workspacetree._nodes[i]->q;
                        _ApproachTarget(Transform(Vector(q[3],q[4],q[5],q[6]),Vector(q[0],q[1],q[2])), inode);
                        pbestconfig = _configtree._nodes[inode];
                        break;
                    }

                    if( fbest > fdist )
                        fbest = fdist;
                }
            }

            if( pbestconfig != NULL )
                // found solution
                break;

            if( ((iter++) % 100 ) == 99 ) {
                RAVEPRINT(L"iter: %d, fbest: %f, time: %f\n", iter, fbest, _GetTime());
                _DumpNodes();
            }
        }

        _DumpNodes();
        DVProfWrite("bispaceprof.txt",1);

        if( pbestconfig == NULL ) {

            RAVEPRINT(L"failed to converge, timed out\n");
            _RestoreRobot();
            return false;
        }

        _robot->SetActiveDOFValues(NULL, &pbestconfig->q[0]);
        if( GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
            RAVEPRINT(L"BiSpace collision\n");

        list<Node*> vecnodes;
        while(pbestconfig != NULL) {
            vecnodes.push_back(pbestconfig);
            if( pbestconfig->parent < 0 )
                break;
            pbestconfig = _configtree._nodes[pbestconfig->parent];
        }

        size_t oldsize = vecnodes.size();
        //_OptimizePath(vecnodes, 0, false);
        _OptimizeAcrossDimensions(vecnodes, 0);

        wstringstream ss;
        ss << endl << "Path found, old size: " << oldsize << ", new size: " << vecnodes.size() << endl;
        for(int i = 0; i < _robot->GetActiveDOF(); ++i)
            ss << vecnodes.front()->q[i] << " ";
        ss << "\n-------\n";
        RAVEPRINT(ss.str().c_str());

        BOOST_ASSERT(vecnodes.size() > 0);

        Trajectory::TPOINT p;
        p.q = _parameters.vinitialconfig;
        ptraj->AddPoint(p);

        Trajectory::TPOINT pt; pt.q.resize(_robot->GetActiveDOF());

        list<Node*>::reverse_iterator itcur, itprev;
        itcur = vecnodes.rbegin();
        itprev = itcur++;
        while(itcur != vecnodes.rend() ) {
            //_InterpolateNodes((*itprev)->q, (*itcur)->q, ptraj);
            for(int i = 0; i < _robot->GetActiveDOF(); ++i)
                pt.q[i] = (*itcur)->q[i];
            ptraj->AddPoint(pt);
            itprev = itcur;
            ++itcur;
        }

        ptraj->CalcTrajTiming(_robot, Trajectory::LINEAR, true, true);

        _RestoreRobot();

        return true;
    }

private:

    bool _CheckCollision(const dReal* pQ0, const dReal* pQ1, IntervalType interval, bool bWorkspace, vector< vector<dReal> >* pvCheckedConfigurations=NULL)
    {
        DVSTARTPROFILE();

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
            BOOST_ASSERT(0);
        }

        // first make sure the end is free
        static vector<dReal> vtempconfig;
        vtempconfig.resize(bWorkspace ? _workspacetree.GetDOF() : _configtree.GetDOF());

        if (bCheckEnd) {
            DVProfileFunc _pf("col1");
            memcpy(&vtempconfig[0], pQ1,sizeof(pQ1[0])*vtempconfig.size());
            if( pvCheckedConfigurations != NULL )
                pvCheckedConfigurations->push_back(vtempconfig);
            _SetRobotConfig(&vtempconfig[0], bWorkspace);
            if (GetEnv()->CheckCollision(_robot) || (!bWorkspace && _robot->CheckSelfCollision()) )
                return true;
        }

        // compute  the discretization
        int i, numSteps = 1;
        dReal* pfresolution = bWorkspace ? &_workspacetree._jointResolutionInv[0] : &_configtree._jointResolutionInv[0];
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

        if( bWorkspace &&( _workspacetree._spacetype == SST_6D) ) {
            DVProfileFunc _pf("colback", numSteps);

            // interpolating quaternions
            dReal fd = 1 / (dReal)numSteps;
            dReal time = (dReal)start * fd;

            Vector q0 = *(Vector*)&pQ0[3];
            Vector q1 = *(Vector*)&pQ1[3];
            dReal theta = dot4(q0, q1);
            if( theta < 0 ) {
                q1 = -q1;
                theta = -theta;
            }
            theta = acosf(theta);

            for (int f = start; f < numSteps; f++) {
                vtempconfig[0] = pQ0[0] + (_jointIncrement[0] * f);
                vtempconfig[1] = pQ0[1] + (_jointIncrement[1] * f);
                vtempconfig[2] = pQ0[2] + (_jointIncrement[2] * f);
                *(Vector*)&vtempconfig[3] = q0 * sin((1-time)*theta) + q1 * sin(time*theta);
                normalize4(&vtempconfig[3], &vtempconfig[3]);
                //BOOST_ASSERT( fabsf(1-((Vector*)&vtempconfig[3])->lengthsqr4()) < 0.001f );

                if( pvCheckedConfigurations != NULL )
                    pvCheckedConfigurations->push_back(vtempconfig);
                _SetRobotConfig(&vtempconfig[0], bWorkspace);
                if( GetEnv()->CheckCollision(_robot) || (!bWorkspace && _robot->CheckSelfCollision()) )
                    return true;
            }
        }
        else {
            DVProfileFunc _pf("colfwd", numSteps);

            for (int f = start; f < numSteps; f++) {

                for (i = 0; i < (int)vtempconfig.size(); i++)
                    vtempconfig[i] = pQ0[i] + (_jointIncrement[i] * f);

                if( pvCheckedConfigurations != NULL )
                    pvCheckedConfigurations->push_back(vtempconfig);
                _SetRobotConfig(&vtempconfig[0], bWorkspace);
                if( GetEnv()->CheckCollision(_robot) || (!bWorkspace&&_robot->CheckSelfCollision()) )
                    return true;
            }
        }

        return false;
    }

    bool _CheckCollision(const dReal* pq, bool bWorkspace, bool bReport = false)
    {
        DVSTARTPROFILE();

        _SetRobotConfig(pq, bWorkspace);
        CollisionReport report;

        if( bWorkspace ) {
            FOREACH(itlink, _vHandLinks) {
                if( GetEnv()->CheckCollision(itlink->first, bReport ? &report : NULL) ) {
                    if( bReport ) {
                        RAVEPRINT(L"bispace: wcollision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
                    }
                    return true;
                }
            }
            return false;
        }

        bool bCol = GetEnv()->CheckCollision(_robot, bReport ? &report : NULL) || _robot->CheckSelfCollision(bReport ? &report : NULL);
        if( bCol && bReport ) {
            RAVEPRINT(L"bispace: fcollision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
        }
        return bCol;
    }

    void _OptimizePath(list<Node*>& path, int nTries, bool bWorkspace)
    {
        DVSTARTPROFILE();

        if( path.size() <= 2 )
            return;

        list<Node*>::iterator startNode, endNode;
        vector< vector<dReal> > vconfigs;
        if( nTries <= 0 )
            nTries = 2 * (int)path.size();

        int nrejected = 0;
        int i = nTries;
        while(i > 0 && nrejected < (int)path.size() ) {

            --i;

            // pick a random node on the path, and a random jump ahead
            int startIndex = RANDOM_INT((int)path.size() - 2);
            int endIndex   = startIndex + (RANDOM_INT(5) + 2);
            if (endIndex >= (int)path.size()) endIndex = (int)path.size() - 1;

            startNode = path.begin();
            advance(startNode, startIndex);
            endNode = startNode;
            advance(endNode, endIndex-startIndex);
            nrejected++;

            // check if the nodes can be connected by a straight line
            vconfigs.resize(0);
            if (_CheckCollision((*startNode)->q, (*endNode)->q, OPEN, bWorkspace)) {
                continue;
            }

            ++startNode;
            FOREACHC(itc, vconfigs)
            path.insert(startNode, _configtree._nodes[_AddFwdNode(-1,&(*itc)[0])]);

            // splice out in-between nodes in path
            path.erase(++startNode, endNode);
            nrejected = 0;

            if( path.size() <= 2 )
                return;
        }
    }

    void _OptimizeAcrossDimensions(list<Node*>& path, int nTries)
    {
        DVSTARTPROFILE();

        if( path.size() <= 2 )
            return;

        dReal fSingleDimProb = 0.1f;
        const dReal fIncProb = 2.0f;
        const dReal fDecProb = 0.8f;

        vector<dReal> vweights;
        list<Node*>::iterator startNode, endNode;
        if( nTries <= 0 )
            nTries = 2 * (int)path.size();

        int nrejected = 0;
        int i = nTries;
        while(i-- > 0 && nrejected < 2*(int)path.size() ) {

            // pick a random node on the path, and a random jump ahead
            int startIndex = RANDOM_INT((int)path.size() - 2);
            int endIndex   = startIndex + (RANDOM_INT(5) + 2);
            if (endIndex >= (int)path.size()) endIndex = (int)path.size() - 1;

            startNode = path.begin();
            advance(startNode, startIndex);
            endNode = startNode;
            advance(endNode, endIndex-startIndex);
            nrejected++;

            if( RANDOM_FLOAT() <= fSingleDimProb ) {
                int irand;

                fSingleDimProb *= fDecProb;

                for(int iter = 0; iter < 4; ++iter) {
                    irand = RANDOM_INT(_robot->GetActiveDOF());
                    if( fabsf( (*startNode)->q[irand] - (*endNode)->q[irand]) > 4*_configtree._jointResolution[irand] )
                        break;
                }

                // only smooth a single dimension
                dReal fdist = (*endNode)->q[irand] - (*startNode)->q[irand];

                // gather the distances
                list<Node*>::iterator it = startNode;
                dReal* prevq = (*it)->q;
                dReal oldval=0;
                vweights.resize(0);
                dReal fsum = 0;
                do {
                    ++it;
                    oldval = prevq[irand];
                    prevq[irand] = (*it)->q[irand];
                    vweights.push_back(_configtree._pDistMetric->Eval(prevq, (*it)->q));
                    fsum += vweights.back();
                    prevq[irand] = oldval;
                    prevq = (*it)->q;
                } while(it != endNode);

                if( fsum > 1e-4f )
                    fsum = fdist/fsum;

                // actually modify the nodes
                it = startNode;
                prevq = (*it)->q;
                FOREACH(itw, vweights) {
                    ++it;
                    dReal fweight = *itw * fsum;
                    *itw = (*it)->q[irand];
                    (*it)->q[irand] = prevq[irand] + fweight;

                    if (_CheckCollision(prevq, (*it)->q, OPEN_START, false)) {
                        // restore all changed nodes and quit
                        (*it)->q[irand] = *itw;
                        it = startNode;
                        prevq = (*it)->q;
                        for(vector<dReal>::iterator itw2 = vweights.begin(); itw2 != itw; ++itw2) {
                            ++it;
                            (*it)->q[irand] = *itw2;

                            if (_CheckCollision(prevq, (*it)->q, OPEN_START, false)) {
                                RAVEPRINT(L"errorrrrr!!!!\n");
                            }
                            prevq = (*it)->q;
                        }
                        break;
                    }

                    prevq = (*it)->q;
                }
            }
            else {
                // check if the nodes can be connected by a straight line
                if (_CheckCollision((*startNode)->q, (*endNode)->q, OPEN, false)) {
                    fSingleDimProb *= fIncProb;
                    continue;
                }

                // splice out in-between nodes in path
                path.erase(++startNode, endNode);
                nrejected = 0;
            }

            if( path.size() <= 2 )
                return;
        }
    }

    int _AddFwdNode(int iparent, const dReal* pconfig)
    {
        int inode = _configtree.AddNode(0, iparent, pconfig);
        BOOST_ASSERT(inode >= 0 && inode < (int)_configtree._worknodes.size());

        // assume the config has already been set
        vector<dReal>& vworkconfig = _configtree._worknodes[inode];
        switch( _workspacetree._spacetype ) {
        case SST_3D:
            vworkconfig.resize(3);
            _SetWorkspace3D(&vworkconfig[0], _pmanip->GetTransform());
            break;
        case SST_6D:
            vworkconfig.resize(7);
            _SetWorkspace6D(&vworkconfig[0], _pmanip->GetTransform());
            break;
        default:
            vworkconfig.resize(_robot->GetActiveDOF());
            memcpy(&vworkconfig[0], pconfig, sizeof(pconfig[0])*_robot->GetActiveDOF());
            break;
        }

        if( _searchtype == ST_bispace )
            _configtree._nodes[inode]->fcost = _parameters.pcostfn->Eval(NULL);
        return inode;
    }

    void _DumpNodes()
    {
        DVSTARTPROFILE();

        //RAVEPRINT(L"time=%f\n", _GetTime());
        //return;

        char filename[255];
        sprintf(filename, "matlab/nodes%d.m", nDumpIndex++);
        FILE* f = fopen(filename, "w");
        if( f == NULL ) {
            RAVEPRINT(L"failed to dump nodes\n");
            return;
        }

        TransformMatrix t;

        if( _pmanip != NULL ) {
            t = _pmanip->pBase->GetTransform();
            fprintf(f, "Tbase = [");
            for(int i = 0; i < 3; ++i) {
                fprintf(f, "%f %f %f %f\n", t.m[4*i+0], t.m[4*i+1], t.m[4*i+2], t.trans[i]);
            }
            fprintf(f, "];\n\n");
        }

        fprintf(f, "fwdnodes = [");

        FOREACH(it, _configtree._nodes) {
            for(int i = 0; i < _configtree.GetDOF(); ++i) {
                fprintf(f, "%f ", (*it)->q[i]);
            }

            fprintf(f, "%f %d\n", (*it)->fcost, (*it)->parent+1);
        }

        if( _pmanip != NULL ) {
            fprintf(f, "];\n\nfwdnodesT = [");

            FOREACH(it, _configtree._nodes) {
                _SetRobotConfig((*it)->q, false);
                t = TransformMatrix(_pmanip->GetTransform());

                for(int i = 0; i < 3; ++i)
                    fprintf(f, "%f %f %f ", t.m[4*i+0], t.m[4*i+1], t.m[4*i+2]);
                fprintf(f, "%f %f %f\n", t.trans.x, t.trans.y, t.trans.z);
            }
        }

        fprintf(f, "];\n\nbacknodes = [");

        FOREACH(it, _workspacetree._nodes) {

            if( _workspacetree._spacetype == SST_3D ) {
                t = TransformMatrix(Transform(Vector(1,0,0,0), Vector(&(*it)->q[0])));

                for(int i = 0; i < 3; ++i)
                    fprintf(f, "%f %f %f ", t.m[4*i+0], t.m[4*i+1], t.m[4*i+2]);
                fprintf(f, "%f %f %f ", t.trans.x, t.trans.y, t.trans.z);
            }
            else if( _workspacetree._spacetype == SST_6D ) {
                t = TransformMatrix(Transform(Vector(&(*it)->q[3]), Vector(&(*it)->q[0])));

                for(int i = 0; i < 3; ++i)
                    fprintf(f, "%f %f %f ", t.m[4*i+0], t.m[4*i+1], t.m[4*i+2]);
                fprintf(f, "%f %f %f ", t.trans.x, t.trans.y, t.trans.z);
            }
            else {
                for(int i = 0; i < _workspacetree.GetDOF(); ++i) {
                    fprintf(f, "%f ", (*it)->q[i]);
                }
            }

            fprintf(f, "%f %d\n", (*it)->fcost, (*it)->parent+1);
        }

        fprintf(f, "];\n");
        fclose(f);

        RAVELOG_DEBUG("dump %s: fwd=%" PRIdS " work=%" PRIdS ", time=%f\n", filename, _configtree._nodes.size(), _workspacetree._nodes.size(), _GetTime());
    }

    void _SetRobotConfig(const dReal* pconfig, bool bWorkspace)
    {
        if( bWorkspace &&( _workspacetree._spacetype != SST_Active) ) {

            Transform t;
            t.trans = Vector(pconfig[0], pconfig[1], pconfig[2]);

            if( _workspacetree._spacetype == SST_6D ) {
                t.rot = *(Vector*)&pconfig[3];
            }

            FOREACH(itlink, _vHandLinks) {
                itlink->first->SetTransform(t*itlink->second);
            }
        }
        else _robot->SetActiveDOFValues(NULL, &pconfig[0]);
    }

    /// if success, returns true and sets pforwardnode with the final configuration space node
    bool _FollowPath(int& ifwdnode, Node* pworknode)
    {
        DVSTARTPROFILE();

        // extract the path and smooth it
        list<Node*> vecnodes;
        while(pworknode != NULL) {
            vecnodes.push_back(pworknode);
            if( pworknode->parent < 0 )
                break;
            pworknode = _workspacetree._nodes[pworknode->parent];
        }

        //RAVEPRINT(L"optimizing path %d\n", vecnodes.size());
        _OptimizePath(vecnodes, (int)vecnodes.size()/2, true);
        bool bSuccess = false;
        RAVELOG_DEBUG("path follow: fwd: %" PRIdS "\n", _configtree._nodes.size());

        // * bias the sampling around the path
        // * have to know when to stop expanding,
        // * need to have a way to invalidate bad nodes
        vector<dReal> vworkconfig, vbestconfig;
        Node* porgfwdnode = _configtree._nodes[ifwdnode];
        Node* pfwdnode = porgfwdnode;
        int NMaxNodes = 128;

        //pfwdnode->info |= NS_FollowDead; // make sure never to sample from it again

        list<Node*>::iterator itnode = vecnodes.begin();
        while(itnode != vecnodes.end()) {
            // sample configurations and see if any make an improvement before going to the next
            _SetWorkspaceFromFwdConfig(vworkconfig, pfwdnode->q);
            dReal fOrgBestDist = _workspacetree._pDistMetric->Eval(&vworkconfig[0], (*itnode)->q);
            dReal fCurBestDist = 1.5f * fOrgBestDist;

            bool bSimplerMethod = true;
            if( bSimplerMethod ) {
                for(int nodeiter = 0; nodeiter < _configtree.GetDOF()+1; ++nodeiter) {
                    if( nodeiter < _configtree.GetDOF() ) {
                        memcpy(&_vSampleConfig[0], pfwdnode->q, sizeof(pfwdnode->q[0])*_configtree.GetDOF());
                        int irand = nodeiter;    //RANDOM_INT(_configtree.GetDOF());
                        _vSampleConfig[irand] += 0.4f*_configtree._fStepLength * (RANDOM_FLOAT()-0.5f) / _distmetric.weights[nodeiter];
                        if( _vSampleConfig[irand] < _configtree._vLowerLimit[irand] )
                            _vSampleConfig[irand] = _configtree._vLowerLimit[irand];
                        else if( _vSampleConfig[irand] > _configtree._vUpperLimit[irand] )
                            _vSampleConfig[irand] = _configtree._vUpperLimit[irand];
                    }
                    else {    //if( nodeiter == _configtree.GetDOF() ) {
                        _GetJacboianTransposeStep(&_vSampleConfig[0], pfwdnode->q, (*itnode)->q, 0.4f*_configtree._fStepLength);
                    }
                    //                else {
                    //                    _configtree._pSampleFn->Sample(&_vSampleConfig[0], pfwdnode->q, 0.4f * _configtree._fStepLength);
                    //                }

                    _SetWorkspaceFromFwdConfig(vworkconfig, &_vSampleConfig[0]);

                    if( _CheckCollision(pfwdnode->q, &_vSampleConfig[0], OPEN_START, false) )
                        continue;

                    dReal fdist = _workspacetree._pDistMetric->Eval(&vworkconfig[0], (*itnode)->q);
                    if( fdist < fCurBestDist ) {
                        fCurBestDist = fdist;
                        vbestconfig = _vSampleConfig;
                    }
                }
            }
            else {
                dReal fRatio = fStochasticFollowRadius*_workspacetree._pDistMetric->Eval(&vworkconfig[0], vecnodes.back()->q);

                for(int nodeiter = 0; nodeiter < nStochasticGradSamples; ++nodeiter) {

                    if( nodeiter < _configtree.GetDOF() ) {
                        memcpy(&_vSampleConfig[0], pfwdnode->q, sizeof(pfwdnode->q[0])*_configtree.GetDOF());
                        int irand = nodeiter;    //RANDOM_INT(_configtree.GetDOF());
                        _vSampleConfig[irand] += _configtree._jointResolution[irand] * fRatio * (RANDOM_FLOAT()-0.5f);
                        if( _vSampleConfig[irand] < _configtree._vLowerLimit[irand] )
                            _vSampleConfig[irand] = _configtree._vLowerLimit[irand];
                        else if( _vSampleConfig[irand] > _configtree._vUpperLimit[irand] )
                            _vSampleConfig[irand] = _configtree._vUpperLimit[irand];
                    }
                    else if( nodeiter == _configtree.GetDOF() ) {
                        _GetJacboianTransposeStep(&_vSampleConfig[0], pfwdnode->q, (*itnode)->q, 0.2f*_configtree._fStepLength);
                    }
                    else {
                        for(int i = 0; i < _configtree.GetDOF(); ++i) {
                            _vSampleConfig[i] = pfwdnode->q[i] + _configtree._jointResolution[i] * fRatio * (RANDOM_FLOAT()-0.5f);
                            if( _vSampleConfig[i] < _configtree._vLowerLimit[i] )
                                _vSampleConfig[i] = _configtree._vLowerLimit[i];
                            else if( _vSampleConfig[i] > _configtree._vUpperLimit[i] )
                                _vSampleConfig[i] = _configtree._vUpperLimit[i];
                        }
                    }

                    _SetWorkspaceFromFwdConfig(vworkconfig, &_vSampleConfig[0]);

                    if( _CheckCollision(pfwdnode->q, &_vSampleConfig[0], OPEN_START, false) )
                        continue;

                    dReal fdist = _workspacetree._pDistMetric->Eval(&vworkconfig[0], (*itnode)->q);
                    if( fdist < fCurBestDist ) {
                        fCurBestDist = fdist;
                        vbestconfig = _vSampleConfig;
                    }
                }
            }

            if( fCurBestDist > fOrgBestDist ) {
                // increment the nodes
                ++itnode;

                if( itnode == vecnodes.end() )
                    break;

                continue;
            }

            // otherwise add
            NMaxNodes--;
            ifwdnode = _AddFwdNode(ifwdnode, &vbestconfig[0]);
            pfwdnode = _configtree._nodes[ifwdnode];
            //pfwdnode->info |= NS_FollowDead; // make sure never to sample from it again
            if( NMaxNodes < 0 ) {
                RAVEPRINT(L"node limit reached\n");
                break;
            }
        }

        _SetWorkspaceFromFwdConfig(vworkconfig, pfwdnode->q);
        dReal fdist = _workspacetree._pDistMetric->Eval(&vworkconfig[0], vecnodes.back()->q);
        RAVELOG_DEBUG("final %f, fwd: %" PRIdS "\n", fdist, _configtree._nodes.size());

        // test out IK to see if close to goal
        if(( _pmanip != NULL) && _pmanip->HasIKSolver() ) {
            _SetRobotConfig(pfwdnode->q, false);
            vector<dReal> viksolution, vikconfig;
            _robot->GetActiveDOFValues(vikconfig);
            PlannerBase::PlannerParameters params;

            vector<int> vprevactive = _robot->GetActiveJointIndices();
            int affinedof = _robot->GetAffineDOF();
            Vector affaxis = _robot->GetAffineRotationAxis();

            FOREACH(ittrans, _vtransWorkGoals) {

                if( _pmanip->FindIKSolution(*ittrans, viksolution, true) ) {
                    // found, set the joints of the robot that correspond to the IK nodes, and return success
                    //cout << "hand trans: " << endl << *ittrans << endl;
                    wstringstream ss;
                    ss << "iksol: ";
                    FOREACH(it, viksolution) ss << *it << " ";
                    ss << endl;
                    RAVEPRINT(ss.str().c_str());

                    if( _pbirrt != NULL ) {
                        params.vgoalconfig.insert(params.vgoalconfig.end(), viksolution.begin(), viksolution.end());
                    }
                    else {
                        FOREACH(itmap, _vIKtoConfigMap)
                        vikconfig[itmap->first] = viksolution[itmap->second];

                        int inode = ifwdnode;
                        for(int icount = 0; icount < 200; ++icount) {

                            if( !_CheckCollision(_configtree._nodes[inode]->q, &vikconfig[0], OPEN, false) ) {
                                bSuccess = true;
                                ifwdnode = _AddFwdNode(inode, &vikconfig[0]);
                                break;
                            }
                            inode = _configtree._nodes[inode]->parent;
                            if( inode < 0 )
                                break;
                        }

                        if( bSuccess ) {
                            RAVEPRINT(L"IK path success\n");
                            break;
                        }
                    }
                }
            }

            if(( _pbirrt != NULL) &&( params.vgoalconfig.size() > 0) ) {
                _robot->SetActiveDOFs(_pmanip->_vecarmjoints);
                _robot->GetActiveDOFValues(params.vinitialconfig);

                params.nMaxIterations = 200;

                boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(_robot->GetActiveDOF()));

                do {
                    if( !_pbirrt->InitPlan(_robot, &params) ) {
                        RAVEPRINT(L"InitPlan failed\n");
                        break;
                    }

                    if( !_pbirrt->PlanPath(ptraj.get()) ) {
                        RAVEPRINT(L"PlanPath failed\n");
                        break;
                    }

                    // add the nodes to the configuration space and declare success
                    _robot->SetActiveDOFs(vprevactive, affinedof, &affaxis);

                    bSuccess = true;
                    FOREACHC(itpt, ptraj->GetPoints()) {
                        FOREACH(itmap, _vIKtoConfigMap)
                        vikconfig[itmap->first] = itpt->q[itmap->second];
                        ifwdnode = _AddFwdNode(ifwdnode, &vikconfig[0]);
                    }

                } while(0);

                if( bSuccess ) {
                    RAVEPRINT(L"IK path success\n");
                }
            }

            _robot->SetActiveDOFs(vprevactive, affinedof, &affaxis);
        }

        if( !bSuccess ) {
            if( fdist < GOAL_THRESH ) {
                // good enough, perhaps use Jacobian to merge even further
                dReal* q = vecnodes.back()->q;
                _ApproachTarget(Transform(Vector(q[3],q[4],q[5],q[6]),Vector(q[0],q[1],q[2])), ifwdnode);
                bSuccess = true;
            }
        }

        if( bSuccess )
            return true;

        // invalidate path, find the closest node to current robot position
        //    BOOST_ASSERT( ifwdnode >= 0 );
        //
        //    vector<dReal> vtargetconfig;
        //    _SetWorkspaceFromFwdConfig(vtargetconfig, _configtree._nodes[ifwdnode]->q);
        //
        //    list<Node*>::iterator itclosest;
        //    int nclosest = -1, i = 0;
        //    dReal fbestdist;
        //    FOREACH(it, vecnodes) {
        //        dReal fdist = _workspacetree._pDistMetric->Eval(&vtargetconfig[0], (*it)->q);
        //        if( nclosest < 0 || (fdist < fbestdist) ) {
        //            itclosest = it;
        //            nclosest = i;
        //            fbestdist = fdist;
        //        }
        //
        //        ++i;
        //    }
        //
        //    BOOST_ASSERT( nclosest >= 0 );
        //    advance(itclosest, ((int)vecnodes.size()-nclosest)/2); // go half to the end
        //
        //    // invalidate everything up to itclosest
        //    Node* pcur = vecnodes.front();
        //    while(pcur != *itclosest) {
        //        pcur->info |= NS_WorkspaceDead;
        //        if( pcur->parent < 0 )
        //            break;
        //        pcur = _workspacetree._nodes[pcur->parent];
        //    }

        return false;
    }

    bool _FollowPathJt(int& ifwdnode, Node* pworknode)
    {
        // increment the nodes
        Node* pfwdnode = _configtree._nodes[ifwdnode];

        RAVEPRINT(L"start follow\n");

        int jiter = 0;
        while(1) {
            _GetJacboianTransposeStep(&_vSampleConfig[0], pfwdnode->q, pworknode->q, 0.2f*_configtree._fStepLength);
            if( _CheckCollision(pfwdnode->q, &_vSampleConfig[0], OPEN_START, false) ) {
                if( pworknode->parent < 0 )
                    break;

                jiter = 0;
                pworknode = _workspacetree._nodes[pworknode->parent];

                // backup a little
                int backup = 2;
                while(backup-- > 0 ) {
                    if( pfwdnode->parent < 0 )
                        break;
                    ifwdnode = pfwdnode->parent;
                    pfwdnode = _configtree._nodes[ifwdnode];
                }

                continue;
            }

            ifwdnode = _AddFwdNode(ifwdnode, &_vSampleConfig[0]);
            pfwdnode = _configtree._nodes[ifwdnode];

            if( jiter++ > 32 ) {
                if( pworknode->parent < 0 )
                    break;

                jiter = 0;
                pworknode = _workspacetree._nodes[pworknode->parent];
            }
        }

        vector<dReal> vworkconfig;
        _SetWorkspaceFromFwdConfig(vworkconfig, &_vSampleConfig[0]);

        dReal fbest = 10000;

        // check for end of goal
        for(size_t i = 0; i < _vtransWorkGoals.size(); ++i) {
            dReal fdist = _workspacetree._pDistMetric->Eval(&vworkconfig[0], _workspacetree._nodes[i]->q);
            if( fdist < GOAL_THRESH ) {
                // good enough, perhaps use Jacobian to merge even further
                dReal* q = _workspacetree._nodes[i]->q;
                _ApproachTarget(Transform(Vector(q[3],q[4],q[5],q[6]),Vector(q[0],q[1],q[2])), ifwdnode);
                return true;
            }

            if( fbest > fdist )
                fbest = fdist;
        }

        RAVEPRINT(L"follow: %f\n", fbest);

        return false;
    }

    /// uses the jacobian to approach
    /// \param ifwdnode Specify the node to start descending on. Once function terminates,
    ///        the ifwdnode is set to the node the search ended on.
    /// \return if target was successfully approached retursn true
    bool _ApproachTarget(const Transform& target, int& ifwdnode)
    {
        BOOST_ASSERT( _pmanip != NULL );

        static vector<dReal> J;
        J.resize(_robot->GetActiveDOF()*4);
        dReal JJt[16], invJJt[16];
        const dReal flambda = 1e-4f;

        memset(&J[0], 0, sizeof(dReal)*J.size());

        vector<dReal> vcurvalues(_robot->GetActiveDOF());
        Node* pcurnode = _configtree._nodes[ifwdnode];
        _SetRobotConfig(pcurnode->q, false);

        // descend on the jacobian
        while(1) {
            // get the translation jacobian
            Transform tEE = _pmanip->GetTransform();

            if( (target.trans-tEE.trans).lengthsqr3() <= JACOBIAN_TRANS_THRESH*JACOBIAN_TRANS_THRESH )
                // done
                break;

            _robot->CalculateActiveJacobian(_pmanip->pEndEffector->GetIndex(), tEE.trans, &J[0]);

            multtrans_to2<dReal, dReal, dReal>(&J[0], &J[0], 3, _robot->GetActiveDOF(), 3, JJt, false);
            JJt[0] += flambda*flambda;
            JJt[4] += flambda*flambda;
            JJt[8] += flambda*flambda;
            inv3(JJt, invJJt, NULL, 3);

            Vector e = target.trans - tEE.trans, v;
            dReal flength = sqrtf(e.lengthsqr3());
            // take constant steps if length is big
            if( flength > JACOBIAN_TRANS_MAXSTEP )
                e = e * (JACOBIAN_TRANS_MAXSTEP / flength);

            transnorm3(v, invJJt, e);
            dReal f = 0;
            bool bsuccess = true;
            for(int i = 0; i < _robot->GetActiveDOF(); ++i) {
                dReal fdir = J[0*_robot->GetActiveDOF()+i] * v.x + J[1*_robot->GetActiveDOF()+i] * v.y + J[2*_robot->GetActiveDOF()+i] * v.z;
                vcurvalues[i] = pcurnode->q[i] + fdir;

                if(( vcurvalues[i] < _configtree._vLowerLimit[i]) ||( vcurvalues[i] > _configtree._vUpperLimit[i]) ) {
                    bsuccess = false;
                    break;
                }
                f += fdir * fdir;
            }

            if( !bsuccess )
                return false;

            _robot->SetActiveDOFValues(NULL, &vcurvalues[0]);

            if( GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
                return false;

            ifwdnode = _AddFwdNode(ifwdnode, &vcurvalues[0]);
            pcurnode = _configtree._nodes[ifwdnode];

            if( f < 1e-7 )
                // gradient descend won't move it anymore
                return false;
        }

        return true;
        //    if( _workspacetree._spacetype != SST_6D )
        //        return true;
        //
        //    // now perform the rotation (need to break up because math gets difficult)
        //    while(1) {
        //        // get the translation jacobian
        //        Transform tEE = _pmanip->GetTransform();
        //
        //        if( (target.rot-tEE.rot).lengthsqr4() <= SQR(JACOBIAN_ROT_THRESH) )
        //            // done
        //            break;
        //
        //        _robot->CalculateActiveJacobian(_pmanip->pEndEffector->GetIndex(), tEE.trans, &J[0]);
        //
        //        multtrans_to2<dReal, dReal, dReal>(&J[0], &J[0], 4, _robot->GetActiveDOF(), 4, JJt, false);
        //        JJt[0] += flambda*flambda;
        //        JJt[5] += flambda*flambda;
        //        JJt[10] += flambda*flambda;
        //        JJt[15] += flambda*flambda;
        //        inv4(JJt, invJJt, NULL);
        //
        //        Vector e = target.rot - tEE.rot, v;
        //        dReal flength = sqrtf(e.lengthsqr4());
        //        // take constant steps if length is big
        //        if( flength > JACOBIAN_ROT_MAXSTEP )
        //            e = e * (JACOBIAN_ROT_MAXSTEP / flength);
        //
        //        for(int i = 0; i < 4; ++i)
        //            v[i] = invJJt[4*i+0]*e[0]+invJJt[4*i+1]*e[1]+invJJt[4*i+2]*e[2]+invJJt[4*i+3]*e[3];
        //
        //        dReal f = 0;
        //        bool bsuccess = true;
        //        for(int i = 0; i < _robot->GetActiveDOF(); ++i) {
        //            dReal fdir = J[0*_robot->GetActiveDOF()+i] * v.x + J[1*_robot->GetActiveDOF()+i] * v.y + J[2*_robot->GetActiveDOF()+i] * v.z + J[3*_robot->GetActiveDOF()+i]*v.w;
        //            vcurvalues[i] = pcurnode->q[i] + fdir;
        //
        //            if( vcurvalues[i] < _configtree._vLowerLimit[i] || vcurvalues[i] > _configtree._vUpperLimit[i]) {
        //                bsuccess = false;
        //                break;
        //            }
        //            f += fdir * fdir;
        //        }
        //
        //        if( !bsuccess )
        //            return false;
        //
        //        _robot->SetActiveDOFValues(NULL, &vcurvalues[0]);
        //
        //        if( GetEnv()->CheckCollision(_robot) || _robot->CheckSelfCollision() )
        //            return false;
        //
        //        ifwdnode = _AddFwdNode(_parameters.pcostfn->Eval(&vcurvalues[0]), ifwdnode, &vcurvalues[0]);
        //        pcurnode = _configtree._nodes[ifwdnode];
        //
        //        if( f < 1e-7 )
        //            // gradient descend won't move it anymore
        //            return false;
        //    }
        //
        //    return true;
    }

    void _GetJacboianTransposeStep(dReal* pnewconfig, const dReal* pcurconfig, dReal* ptarget, dReal fStep)
    {
        BOOST_ASSERT( _pmanip != NULL );

        static vector<dReal> J;
        J.resize(_robot->GetActiveDOF()*7);

        _SetRobotConfig(pcurconfig, false);
        Transform tEE = _pmanip->GetTransform();

        // get the translation jacobian
        Transform tTarget;
        if( _workspacetree._spacetype == SST_6D )
            tTarget.rot = Vector(ptarget[3], ptarget[4], ptarget[5], ptarget[6]);
        tTarget.trans = Vector(ptarget[0], ptarget[1], ptarget[2]);

        _robot->CalculateActiveJacobian(_pmanip->pEndEffector->GetIndex(), tEE.trans, &J[0]);
        _robot->CalculateActiveRotationJacobian(_pmanip->pEndEffector->GetIndex(), tEE.rot, &J[3*_robot->GetActiveDOF()]);

        Vector et = tTarget.trans - tEE.trans;
        Vector er = tTarget.rot - tEE.rot;
        for(int j = 0; j < _robot->GetActiveDOF(); ++j) {

            dReal f = 0;
            for(int i = 0; i < 3; ++i)
                f += J[i*_robot->GetActiveDOF()+j] * et[i];
            for(int i = 0; i < 4; ++i)
                f += J[(3+i)*_robot->GetActiveDOF()+j] * er[i];
            pnewconfig[j] = pcurconfig[j]+f;
        }

        dReal flength = _configtree._pDistMetric->Eval(pcurconfig, pnewconfig);
        // take constant steps if length is big
        if( flength > fStep ) {
            dReal fratio = fStep / flength;

            for(int j = 0; j < _robot->GetActiveDOF(); ++j) {
                pnewconfig[j] = pcurconfig[j] + (pnewconfig[j]-pcurconfig[j])*fratio;
            }
        }
    }

    void _SetWorkspaceFromFwdConfig(vector<dReal>& vworkconfig, const dReal* pfwdconfig)
    {
        switch( _workspacetree._spacetype ) {
        case SST_3D:
            vworkconfig.resize(3);
            _SetRobotConfig(pfwdconfig, false);
            _SetWorkspace3D(&vworkconfig[0], _pmanip->GetTransform());
            break;
        case SST_6D:
            vworkconfig.resize(7);
            _SetRobotConfig(pfwdconfig, false);
            _SetWorkspace6D(&vworkconfig[0], _pmanip->GetTransform());
            break;
        default:
            vworkconfig.resize(_robot->GetActiveDOF());
            memcpy(&vworkconfig[0], pfwdconfig, sizeof(pfwdconfig[0])*_robot->GetActiveDOF());
            break;
        }
    }

    void _SetWorkspace3D(dReal* pworkconfig, const Transform& t)
    {
        pworkconfig[0] = t.trans.x;
        pworkconfig[1] = t.trans.y;
        pworkconfig[2] = t.trans.z;
    }

    void _SetWorkspace6D(dReal* pworkconfig, const Transform& t)
    {
        pworkconfig[0] = t.trans.x;
        pworkconfig[1] = t.trans.y;
        pworkconfig[2] = t.trans.z;
        pworkconfig[3] = t.rot.x;
        pworkconfig[4] = t.rot.y;
        pworkconfig[5] = t.rot.z;
        pworkconfig[6] = t.rot.w;
    }

    void _SetTime()
    {
        _nBaseStartTime = GetMicroTime();
    }

    dReal _GetTime()     // get elapsed time
    {
        return (dReal)(GetMicroTime()-_nBaseStartTime) * 1e-6f;
    }

    BiSpaceParameters _parameters;

    SpatialTree _configtree;     ///< forward search tree
    SpatialTree _workspacetree;     ///< backward search tree
    SearchType _searchtype;

    RobotBase* _robot;
    PlannerBase* _pbirrt;
    DistanceMetric* _pConfigDistMetric;
    uint64_t _nBaseStartTime;     ///< start

    vector<dReal> _vSampleConfig;
    vector<dReal> _jointIncrement;
    vector<dReal> _vzero;
    vector< pair<KinBody::Link*, Transform> > _vHandLinks;     ///< links of the hand involved in workspace collision, each transformation is the offset from the workspace transformation

    vector< pair<int, int> > _vIKtoConfigMap;     ///< used to transfer ik configuration solution to the active robot config

    vector<Transform> _vtransRobotStored, _vtransWorkGoals;
    int nNumBackGoals;

    dReal fConfigFollowProb, fStochasticFollowRadius, fGoalVariance;
    int nExpansionMultiplier, nStochasticGradSamples;

    vector<Transform> _vectrans;     ///< cache

    int nDumpIndex;
    const RobotBase::Manipulator* _pmanip;
    bool _bInit;     ///< true if the planner has been initialized
    bool _bWorkspaceCollision;

    // default metrics
    class SimpleCostMetric : public PlannerBase::CostFunction
    {
public:
        void Init(RobotBase* probot);
        virtual float Eval(const void* pConfiguration) {
            return fconst;
        }
        dReal fconst;
    };

    class SimpleGoalMetric : public PlannerBase::GoalFunction
    {
public:

        SimpleGoalMetric() : PlannerBase::GoalFunction() {
            thresh = 0.01f; _robot = NULL;
        }

        //checks if pConf is within this cone (note: only works in 3D)
        float Eval(const void* c1)
        {
            BOOST_ASSERT( _robot != NULL && _robot->GetActiveManipulator() != NULL && _robot->GetActiveManipulator()->pEndEffector != NULL );

            _robot->SetActiveDOFValues(NULL,(const dReal *) c1);
            Transform cur = _robot->GetActiveManipulator()->GetTransform();

            return sqrtf(lengthsqr3(tgoal.trans - cur.trans));
        }

        virtual float GetGoalThresh() {
            return thresh;
        }
        virtual void SetRobot(RobotBase* robot) {
            _robot = robot;
        }

        Transform tgoal;             // workspace goal

private:
        float thresh;
        RobotBase* _robot;
    };

    class SimpleDistMetric : public PlannerBase::DistanceMetric
    {
public:
        SimpleDistMetric() : PlannerBase::DistanceMetric() {
            thresh = 0.01f; _robot = NULL;
        }

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
            BOOST_ASSERT( _robot->GetActiveDOF() == (int)weights.size() );

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
        ExtendDistMetric() : PlannerBase::DistanceMetric() {
            thresh = 0.01f; _robot = NULL;
        }
    };

    class Workspace6DDistMetric : public PlannerBase::DistanceMetric
    {
public:
        Workspace6DDistMetric() : PlannerBase::DistanceMetric() {
            frotweight = 1;
        }

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
        Workspace3DDistMetric() : PlannerBase::DistanceMetric() {
        }

        virtual float Eval(const void* c0, const void* c1)
        {
            return sqrtf( (*(Vector*)c0 - *(Vector*)c1).lengthsqr3() );
        }
    };

    class ForwardSampleFunction : public PlannerBase::SampleFunction
    {
public:
        ForwardSampleFunction() : _ptree(NULL), plower(NULL), pupper(NULL), prange(NULL) {
        }

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
