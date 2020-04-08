// -*- coding: utf-8 -*-
// Copyright (C) 2020 Puttichai Lertkultanon
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
#ifndef BIRRT_WORKSPACE_PLANNER_H
#define BIRRT_WORKSPACE_PLANNER_H

#include "rrt.h"
#include "rplanners.h"
#include <boost/algorithm/string.hpp>

/// \brief (TODO)
class NodeWithTransform : public NodeBase
{
public:
    /// \brief Initialize a node with a parent node and a config
    NodeWithTransform(NodeWithTransform* pparent, const std::vector<dReal>& vconfig) : rrtparent(pparent) {
        std::copy(vconfig.begin(), vconfig.end(), q);
        _level = 0;
        _hasselfchild = 0;
        _usenn = 1;
        _userdata = 0;
        _transformComputed = 0;
    }

    NodeWithTransform(NodeWithTransform* pparent, const dReal* pconfig, int ndof) : rrtparent(pparent) {
        std::copy(pconfig, pconfig + ndof, q);
        _level = 0;
        _hasselfchild = 0;
        _usenn = 1;
        _userdata = 0;
        _transformComputed = 0;
    }

    /// \brief Initialize a node with a parent node, a config, and a corresponding transform
    NodeWithTransform(NodeWithTransform* pparent, const std::vector<dReal>& vconfig, const Transform& pose) : rrtparent(pparent) {
        std::copy(vconfig.begin(), vconfig.end(), q);
        p.rot = pose.rot;
        p.trans = pose.trans;
        _level = 0;
        _hasselfchild = 0;
        _usenn = 1;
        _userdata = 0;
        _transformComputed = 1;
    }

    NodeWithTransform(NodeWithTransform* pparent, const dReal* pconfig, int ndof, const Transform& pose) : rrtparent(pparent) {
        std::copy(pconfig, pconfig + ndof, q);
        p.rot = pose.rot;
        p.trans = pose.trans;
        _level = 0;
        _hasselfchild = 0;
        _usenn = 1;
        _userdata = 0;
        _transformComputed = 1;
    }

    ~NodeWithTransform() {
    }

    NodeWithTransform* rrtparent;
    std::vector<NodeWithTransform*> _vchildren;
    int16_t _level;
    uint8_t _hasselfchild;
    uint8_t _usenn;
    uint8_t _userdata;

    uint8_t _transformComputed;
    Transform p;
    dReal q[0];
};

/// \brief (TODO)
template <typename Node>
class SpatialTree2 : public SpatialTree<Node>
{
    typedef Node* NodePtr;

    ~SpatialTree2()
    {
        Reset();
    }

    virtual void Init(boost::weak_ptr<PlannerBase> planner, int dof, boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn, dReal fStepLength, dReal maxdistance)
    {
        Reset();
        if( !!_pNodesPool ) {
            // See if the pool can be preserved.
            if( _dof != dof ) {
                _pNodesPool.reset();
            }
        }
        if( !_pNodesPool ) {
            _pNodesPool.reset(new boost::pool<>(sizeof(Node) + _dof*sizeof(dReal)));
        }

        _planner = planner;
        _distmetricfn = distmetricfn;
        _fStepLength = fStepLength;
        _dof = dof;
        _vNewConfig.resize(dof);
        _vDeltaConfig.resize(dof);
        _vTempConfig.resize(dof);
        _maxdistance = maxdistance;
        _mindistance = 0.001*fStepLength;
        _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
        _minlevel = _maxlevel - 1;
        _fMaxLevelBound = RavePow(_base, _maxlevel);
        int enclevel = _EncodeLevel(_maxlevel);
        if( enclevel >= (int)_vsetLevelNodes.size() ) {
            _vsetLevelNodes.resize(enclevel + 1);
        }
        _constraintreturn.reset(new ConstraintFilterReturn());

        {
            _vAccumWeights.resize(_maxlevel); // need numlevels - 1 values
            dReal weight = 2.0;
            dReal fTotalWeight = 0;
            for( int ilevel = _maxlevel - 1; ilevel >= 0; --ilevel ) {
                _vAccumWeights[ilevel] = RavePow(weight, _maxlevel - 1 - ilevel);
                fTotalWeight += _vAccumWeights[ilevel];
            }
            dReal temp = 0;
            for( int ilevel = _maxlevel - 1; ilevel >= 0; --ilevel ) {
                _vAccumWeights[ilevel] /= fTotalWeight;
                _vAccumWeights[ilevel] += temp;
                temp = _vAccumWeights[ilevel];
            }
        }
    }

    virtual void Reset()
    {
        if( !!_pNodesPool ) {
            // make sure all children are deleted
            for(size_t ilevel = 0; ilevel < _vsetLevelNodes.size(); ++ilevel) {
                FOREACH(itnode, _vsetLevelNodes[ilevel]) {
                    (*itnode)->~Node();
                }
            }
            FOREACH(itchildren, _vsetLevelNodes) {
                itchildren->clear();
            }
            //_pNodesPool->purge_memory();
            _pNodesPool.reset(new boost::pool<>(sizeof(Node) + _dof*sizeof(dReal)));
        }
        _numnodes = 0;
    }

    /// \brief Perform a normal tree extension operation given a
    /// target config vTargetConfig. The difference is that it also
    /// keeps track of the transform tTarget corresponding to
    /// vTargetConfig.
    virtual ExtendType Extend(const std::vector<dReal>& vTargetConfig, NodeBasePtr& plastnode, bool bOneStep=false)
    {
        // Get the nearnest neighbor of vTargetConfig on the tree
        std::pair<NodePtr, dReal> nn = _FindNearestNode(vTargetConfig);
        if( !nn.first ) {
            return ET_Failed;
        }

        NodePtr pnode = nn.first;
        plastnode = nn.first;

        bool bHasAdded = false;
        boost::shared_ptr<PlannerBase> planner(_planner);
        PlannerBase::PlannerParametersConstPtr params = planner->GetParameters();

        _vCurConfig.resize(_dof);
        std::copy(pnode->q, pnode->q + _dof, _vCurConfig.begin());


        // Extend the tree
        for( int iter = 0; iter < 100; ++iter ) {
            dReal fdist = _ComputeDistance(&_vCurConfig[0], vTargetConfig);
            if( fdist > _fStepLength ) {
                fdist = _fStepLength / fdist;
            }
            else if( fdist <= dReal(0.01) * _fStepLength ) {
                // Return connected if the distance is very close.
                return ET_Connected;
            }
            else {
                fdist = 1;
            }

            _vNewConfig = _vCurConfig;
            _vDeltaConfig = vTargetConfig;
            params->_diffstatefn(_vDeltaConfig, _vCurConfig);
            for( int idof = 0; idof < _dof; ++idof ) {
                _vDeltaConfig[idof] *= fdist;
            }

            if( params->SetStateValues(_vNewConfig) != 0 ) {
                return bHasAdded ? ET_Success : ET_Failed;
            }

            if( params->_neighstatefn(_vNewConfig, _vDeltaConfig, _fromgoal ? NSO_GoalToInitial : 0) == NSS_Failed ) {
                return bHasAdded ? ET_Success : ET_Failed;
            }

            if( _fromgoal ) {
                if( params->CheckPathAllConstraints(_vNewConfig, _vCurConfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenEnd, 0xffff|CFO_FillCheckedConfiguration, _constraintreturn) != 0 ) {
                    return bHasAdded ? ET_Success : ET_Failed;
                }
            }
            else { // not _fromgoal
                if( params->CheckPathAllConstraints(_vCurConfig, _vNewConfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenEnd, 0xffff|CFO_FillCheckedConfiguration, _constraintreturn) != 0 ) {
                    return bHasAdded ? ET_Success : ET_Failed;
                }
            }

            int iAdded = 0; // number of nodes added to the tree
            if( _constraintreturn->_bHasRampDeviatedFromInterpolation ) {
                // Since the path checked by CheckPathAllConstraints can be different from the
                // straight line connecting _vNewConfig and _vCurConfig, we add all checked
                // configurations along the checked segment to the tree.
                if( _fromgoal ) {
                    for( int iconfig = ((int)_constraintreturn->_configurations.size()) - _dof; iconfig >= 0; iconfig -= _dof ) {
                        std::copy(_constraintreturn->_configurations.begin() + iconfig,
                                  _constraintreturn->_configurations.begin() + iconfig + _dof,
                                  _vNewConfig.begin());
                        NodePtr pnewnode = _InsertNode(pnode, _vNewConfig, 0);
                        if( !!pnewnode ) {
                            bHasAdded = true;
                            pnode = pnewnode;
                            plastnode = pnode;
                            ++iAdded;
                        }
                        else {
                            break;
                        }
                    } // end for iconfig
                }
                else { // not _fromgoal
                    for( int iconfig = 0; iconfig + _dof - 1 < (int)_constraintreturn->_configurations.size(); iconfig += _dof ) {
                        std::copy(_constraintreturn->_configurations.begin() + iconfig,
                                  _constraintreturn->_configurations.begin() + iconfig + _dof,
                                  _vNewConfig.begin());
                        NodePtr pnewnode = _InsertNode(pnode, _vNewConfig, 0);
                        if( !!pnewnode ) {
                            bHasAdded = true;
                            pnode = pnewnode;
                            plastnode = pnode;
                            ++iAdded;
                        }
                        else {
                            break;
                        }
                    } // end for iconfig
                }
            }
            else {
                // not _bHasRampDeviatedFromInterpolation

                // Since the final configuration is dircetly _vNewConfig, simply add only _vNewConfig to the tree
                NodePtr pnewnode = _InsertNode(pnode, _vNewConfig, 0);
                if( !!pnewnode ) {
                    pnode = pnewnode;
                    plastnode = pnode;
                    bHasAdded = true;
                }
            }

            if( bHasAdded && bOneStep ) {
                return ET_Connected; //
            }
            _vCurConfig.swap(_vNewConfig);
        }

        return bHasAdded ? ET_Success : ET_Failed;
    }

    // TODO
    /// \brief
    virtual ExtendType ExtendWithTransform(const std::vector<dReal>& vTargetConfig, const Transform& tTarget, NodeBasePtr& plastnode, bool bOneStep=false)
    {
        return ET_Success;
    }

    /// \brief Select a node in the tree based on the given fSampledValue.
    virtual NodePtr SampleNode(dReal fSampledValue)
    {
        dReal fPrevLevelBound = 0;
        for( int ilevel = 0; ilevel < _maxlevel; ++ilevel ) {
            if( fSampledValue <= _vAccumWeights[ilevel] ) {
                // Select ilevel
                std::vector<NodePtr>& setLevelChildren = _vsetLevelNodes.at(ilevel);
                size_t numLevelNodes = setLevelChildren.size();
                int iSelectedNode = (int)((fSampledValue - fPrevLevelBound)/(_vAccumWeights[ilevel] - fPrevLevelBound));
                return _vsetLevelNodes.at(ilevel).at(iSelectedNode);
            }
            fPrevLevelBound = _vAccumWeights[ilevel];
        }
        //
        return _vsetLevelNodes.at(_maxlevel).at(0);
    }

    inline int _EncodeLevel(int level) const {
        if( level <= 0 ) {
            return -2*level;
        }
        else {
            return 2*level+1;
        }
    }

    inline dReal _ComputeDistance(const dReal* config0, const dReal* config1) const
    {
        return _distmetricfn(VectorWrapper<dReal>(config0, config0+_dof), VectorWrapper<dReal>(config1, config1+_dof));
    }

    inline dReal _ComputeDistance(const dReal* config0, const std::vector<dReal>& config1) const
    {
        return _distmetricfn(VectorWrapper<dReal>(config0,config0+_dof), config1);
    }

    inline dReal _ComputeDistance(NodePtr node0, NodePtr node1) const
    {
        return _distmetricfn(VectorWrapper<dReal>(node0->q, &node0->q[_dof]), VectorWrapper<dReal>(node1->q, &node1->q[_dof]));
    }

    std::pair<NodePtr, dReal> _FindNearestNode(const std::vector<dReal>& vquerystate) const
    {
        std::pair<NodePtr, dReal> bestnode;
        bestnode.first = NULL;
        bestnode.second = std::numeric_limits<dReal>::infinity();
        if( _numnodes == 0 ) {
            return bestnode;
        }
        OPENRAVE_ASSERT_OP((int)vquerystate.size(),==,_dof);

        int currentlevel = _maxlevel; // where the root node is
        // traverse all levels gathering up the children at each level
        dReal fLevelBound = _fMaxLevelBound;
        _vCurrentLevelNodes.resize(1);
        _vCurrentLevelNodes[0].first = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
        _vCurrentLevelNodes[0].second = _ComputeDistance(_vCurrentLevelNodes[0].first->q, vquerystate);
        if( _vCurrentLevelNodes[0].first->_usenn ) {
            bestnode = _vCurrentLevelNodes[0];
        }
        while(_vCurrentLevelNodes.size() > 0 ) {
            _vNextLevelNodes.resize(0);
            //RAVELOG_VERBOSE_FORMAT("level %d (%f) has %d nodes", currentlevel%fLevelBound%_vCurrentLevelNodes.size());
            dReal minchilddist=std::numeric_limits<dReal>::infinity();
            FOREACH(itcurrentnode, _vCurrentLevelNodes) {
                // only take the children whose distances are within the bound
                FOREACHC(itchild, itcurrentnode->first->_vchildren) {
                    dReal curdist = _ComputeDistance((*itchild)->q, vquerystate);
                    if( !bestnode.first || (curdist < bestnode.second && bestnode.first->_usenn)) {
                        bestnode = make_pair(*itchild, curdist);
                    }
                    _vNextLevelNodes.emplace_back(*itchild,  curdist);
                    if( minchilddist > curdist ) {
                        minchilddist = curdist;
                    }
                }
            }

            _vCurrentLevelNodes.resize(0);
            dReal ftestbound = minchilddist + fLevelBound;
            FOREACH(itnode, _vNextLevelNodes) {
                if( itnode->second < ftestbound ) {
                    _vCurrentLevelNodes.push_back(*itnode);
                }
            }
            currentlevel -= 1;
            fLevelBound *= _fBaseInv;
        }
        //RAVELOG_VERBOSE_FORMAT("query went through %d levels", (_maxlevel-currentlevel));
        return bestnode;
    }

    /// \brief
    inline NodePtr _CreateNode(NodePtr pparent, const std::vector<dReal>& vconfig, uint32_t userdata)
    {
        // Allocate memory for the structure and the internal state vectors
        void* pmemory = _pNodesPool->malloc();
        NodePtr pnode = new (pmemory) Node(pparent, vconfig);
        pnode->_userdata = userdata;
        return pnode;
    }

    /// \brief
    inline NodePtr _CreateNodeWithTransform(NodePtr pparent, const std::vector<dReal>& vconfig, const Transform& pose, uint32_t userdata)
    {
        // Allocate memory for the structure and the internal state vectors
        void* pmemory = _pNodesPool->malloc();
        NodePtr pnode = new (pmemory) Node(pparent, vconfig, pose);
        pnode->_userdata = userdata;
        return pnode;
    }

    /// \brief
    NodePtr _InsertNode(NodePtr pparent, const std::vector<dReal>& vconfig, uint32_t userdata)
    {
        NodePtr pnewnode = _CreateNode(pparent, vconfig, userdata);
        if( _numnodes == 0 ) {
            _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).insert(pnewnode);
            pnewnode->_level = _maxlevel;
            _numnodes += 1;
        }
        else { // _numnodes > 0
            _vCurrentLevelNodes.resize(1);
            _vCurrentLevelNodes[0].first = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
            _vCurrentLevelNodes[0].second = _ComputeDistance(_vCurrentLevelNodes[0].first->q, vconfig);
            int nParentFound = _InsertRecursive(pnewnode, _vCurrentLevelNodes, _maxlevel, _fMaxLevelBound);
            if( nParentFound == 0 ) {
                // This could possibly happen with circular joints. See https://github.com/rdiankov/openrave/issues/323
                std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                FOREACHC(it, vconfig) {
                    ss << *it << ",";
                }
                throw OPENRAVE_EXCEPTION_FORMAT("Could not insert config=[%s] into the cover tree. perhaps cover tree _maxdistance=%f is not enough from the root", ss.str()%_maxdistance, ORE_Assert);
            }
            if( nParentFound < 0 ) {
                return NodePtr();
            }
        }
        return pnewnode;
    }

    /// \brief
    NodePtr _InsertNodeWithTransform(NodePtr pparent, const std::vector<dReal>& vconfig, const Transform& pose, uint32_t userdata)
    {
        NodePtr pnewnode = _CreateNodeWithTransform(pparent, vconfig, pose, userdata);
        if( _numnodes == 0 ) {
            _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).insert(pnewnode);
            pnewnode->_level = _maxlevel;
            _numnodes += 1;
        }
        else { // _numnodes > 0
            _vCurrentLevelNodes.resize(1);
            _vCurrentLevelNodes[0].first = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
            _vCurrentLevelNodes[0].second = _ComputeDistance(_vCurrentLevelNodes[0].first->q, vconfig);
            int nParentFound = _InsertRecursive(pnewnode, _vCurrentLevelNodes, _maxlevel, _fMaxLevelBound);
            if( nParentFound == 0 ) {
                // This could possibly happen with circular joints. See https://github.com/rdiankov/openrave/issues/323
                std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                FOREACHC(it, vconfig) {
                    ss << *it << ",";
                }
                throw OPENRAVE_EXCEPTION_FORMAT("could not insert config=[%s] into the cover tree. perhaps cover tree _maxdistance=%f is not enough from the root", ss.str()%_maxdistance, ORE_Assert);
            }
            if( nParentFound < 0 ) {
                return NodePtr();
            }
        }
        return pnewnode;
    }

private:
    boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;
    boost::weak_ptr<PlannerBase> _planner;
    dReal _fStepLength;
    int _dof; ///< the number of values of each state
    int _fromgoal;

    // cover tree data structures
    boost::shared_ptr< boost::pool<> > _pNodesPool; ///< pool nodes are created from

    std::vector< std::set<NodePtr> > _vsetLevelNodes; ///< _vsetLevelNodes[enc(level)][node] holds the indices of the children of "node" of a given the level. enc(level) maps (-inf,inf) into [0,inf) so it can be indexed by the vector. Every node has an entry in a map here. If the node doesn't hold any children, then it is at the leaf of the tree. _vsetLevelNodes.at(_EncodeLevel(_maxlevel)) is the root.

    dReal _maxdistance; ///< maximum possible distance between two states. used to balance the tree. Has to be > 0.
    dReal _mindistance; ///< minimum possible distance between two states until they are declared the same
    dReal _base, _fBaseInv, _fBaseChildMult; ///< a constant used to control the max level of traversion. _fBaseInv = 1/_base, _fBaseChildMult=1/(_base-1)
    int _maxlevel; ///< the maximum allowed levels in the tree, this is where the root node starts (inclusive)
    int _minlevel; ///< the minimum allowed levels in the tree (inclusive)
    int _numnodes; ///< the number of nodes in the current tree starting at the root at _vsetLevelNodes.at(_EncodeLevel(_maxlevel))
    dReal _fMaxLevelBound; // pow(_base, _maxlevel)

    // cache
    vector<NodePtr> _vchildcache;
    set<NodePtr> _setchildcache;
    vector<dReal> _vNewConfig, _vDeltaConfig, _vCurConfig;
    mutable vector<dReal> _vTempConfig;
    ConstraintFilterReturnPtr _constraintreturn;

    mutable std::vector< std::pair<NodePtr, dReal> > _vCurrentLevelNodes, _vNextLevelNodes;
    mutable std::vector< std::vector<NodePtr> > _vvCacheNodes;
    std::vector<dReal> _vAccumWeights;
};

// (TODO)
class BirrtPlanner2 : public RrtPlanner<NodeWithTransform>
{
public:
    BirrtPlanner2(EnvironmentBasePtr penv) : RrtPlanner<NodeWithTransform>(penv), _treeBackward(1)
    {
        __description += "";
        RegisterCommand("DumpTree", boost::bind(&BirrtPlanner2::_DumpTreeCommand, this, _1, _2), "");

        _nValidGoals = 0;
    }

    virtual ~BirrtPlanner2() {
    }

    struct GOALPATH
    {
        GOALPATH() : startindex(-1), goalindex(-1), length(0) {
        }
        std::vector<dReal> qall;
        int startindex, goalindex;
        dReal length;
    };

    // TODO
    /// \brief
    virtual bool InitPlan(RobotBasePtr probot, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new RRTParameters());
        _parameters->copy(pparams);
        if( !RrtPlanner<NodeWithTransform>::_InitPlan(probot, _parameters) ) {
            _parameters.reset();
            return false;
        }

        _fGoalBiasProb = dReal(0.01);
        _fWorkspaceSamplingBiasProb = dReal(0.01);

        // TODO
        return true;
    }

    // TODO
    /// \brief
    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        _goalindex = -1;
        _startindex = -1;
        if( !_parameters ) {
            return PlannerStatus("BirrtPlanner2::PlanPath - Error, planner not initialized", PS_Failed);
        }

        // TODO
        PlannerStatus status;
        return status;
    }

    // TODO
    /// \brief
    virtual void _ExtractPath(GOALPATH& goalpath, NodeBase* iConnectedForward, NodeBase* iConnectedBackward)
    {
    }

    /// \brief
    virtual PlannerParametersConstPtr GetParameters() const
    {
        return _parameters;
    }

    /// \brief
    virtual bool _DumpTreeCommand(std::ostream& os, std::istream& is)
    {
        std::string filename = RaveGetHomeDirectory() + std::string("/birrt2dump.txt");
        std::getline(is, filename);
        boost::trim(filename);
        RAVELOG_VERBOSE_FORMAT("env=%d, dumping RRT tree to %s", GetEnv()->GetId()%filename);
        std::ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        _treeForward.DumpTree(f);
        _treeBackward.DumpTree(f);
        return true;
    }

protected:
    RRTParametersPtr _parameters;
    SpatialTree< NodeWithTransform > _treeBackward;
    dReal _fGoalBiasProb;
    dReal _fWorkspaceSamplingBiasProb;
    std::vector< NodeBase* > _vGoalNodes;
    size_t _nValidGoals;
    std::vector< GOALPATH > _vGoalPaths;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(NodeWithTransform)
BOOST_TYPEOF_REGISTER_TYPE(BirrtPlanner2::GOALPATH)
#endif

#endif // #ifndef BIRRT_WORKSPACE_PLANNER_H
