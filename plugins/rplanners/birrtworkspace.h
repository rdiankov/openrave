// -*- coding: utf-8 -*-
// Copyright (C) 2020 Puttichai Lertkultanon and Rosen Diankov
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

/// \brief Node in free configuration space.
class NodeWithTransform : public NodeBase
{
public:
    /// \brief Initialize a node with a parent node and a config
    NodeWithTransform(NodeWithTransform* pparent, const std::vector<dReal>& vconfig) : rrtparent(pparent) {
        std::copy(vconfig.begin(), vconfig.end(), q);
    }

    NodeWithTransform(NodeWithTransform* pparent, const dReal* pconfig, int ndof) : rrtparent(pparent) {
        std::copy(pconfig, pconfig + ndof, q);
    }

    /// \brief Initialize a node with a parent node, a config, and a corresponding transform
    NodeWithTransform(NodeWithTransform* pparent, const std::vector<dReal>& vconfig, const Transform& pose) : rrtparent(pparent) {
        std::copy(vconfig.begin(), vconfig.end(), q);
        // Assign rot, trans separately to skip checking of rot normalization. Is it ok, though?
        this->pose.rot = pose.rot;
        this->pose.trans = pose.trans;
        _transformComputed = 1;
    }

    NodeWithTransform(NodeWithTransform* pparent, const dReal* pconfig, int ndof, const Transform& pose) : rrtparent(pparent) {
        std::copy(pconfig, pconfig + ndof, q);
        // Assign rot, trans separately to skip checking of rot normalization. Is it ok, though?
        this->pose.rot = pose.rot;
        this->pose.trans = pose.trans;
        _transformComputed = 1;
    }

    ~NodeWithTransform() {
    }

    NodeWithTransform* rrtparent; ///< pointer to the parent of this node in RRT tree.
    std::vector<NodeWithTransform*> _vchildren; ///< direct children of this node in cover tree (nodes in the level below this). This is not related to RRT tree's children of this node.
    int16_t _level = 0; ///< cover tree's level of this node
    uint8_t _hasselfchild = 0; ///< if 1, then _vchildren contains a clone of this node in the level below it.
    uint8_t _usenn = 1; ///< if 1, then include this node in the nearest neighbor search. otherwise, ignore this node.
    uint8_t _userdata = 0;

#ifdef _DEBUG
    int id;
#endif

    uint8_t _transformComputed = 0; ///< if 1, the transform corresponding to this node (`pose`) has been computed.
    Transform pose; ///< a transform corresponding to this node.
    dReal q[0]; ///< the configuration immediately following this struct.
};

/// \brief Data structure storing configuration information based on the Cover Tree (Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf)
template <typename Node>
class SpatialTree2 : public SpatialTree<Node>
{
public:
    typedef Node* NodePtr;

    SpatialTree2(int fromgoal) : SpatialTree<Node>(fromgoal)
    {
        _fromgoal = fromgoal;
    }

    ~SpatialTree2()
    {
        Reset();
    }

    virtual void Init(boost::weak_ptr<PlannerBase> planner,
                      int dof,
                      boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn,
                      boost::function<bool(const std::vector<dReal>&, Transform&)> fkfn,
                      boost::function<bool(const Transform&, const std::vector<dReal>&, std::vector<dReal>&)> ikfn,
                      dReal fStepLength,
                      dReal fWorkspaceStepLength,
                      dReal maxdistance)
    {
        Reset();
        if( !!_pNodesPool ) {
            // See if the pool can be preserved.
            if( _dof != dof ) {
                _pNodesPool.reset();
            }
        }
        if( !_pNodesPool ) {
            _pNodesPool.reset(new boost::pool<>(sizeof(Node) + dof*sizeof(dReal)));
        }

        _planner = planner;
        _distmetricfn = distmetricfn;
        _fkfn = fkfn;
        _ikfn = ikfn;
        _fStepLength = fStepLength;
        _fWorkspaceStepLength = fWorkspaceStepLength;
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

        // Is it ok to compute the weights for random sampling (SampleNode) based on the cover tree instead of the RRT?
        {
            // Probability of level ilevel being selected in SampleNode is p_select = weight^(_maxlevel - 1 -
            // ilevel)/totalweight. That is, the level ilevel is `weight` times more likely to be sampled that level
            // (ilevel + 1).
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
            for(const std::set<NodePtr>& setLevelNodes : _vsetLevelNodes) {
                for(const NodePtr& node : setLevelNodes) {
                    node->~Node();
                }
            }
            for(std::set<NodePtr>& setLevelNodes : _vsetLevelNodes) {
                setLevelNodes.clear();
            }
            //_pNodesPool->purge_memory();
            _pNodesPool.reset(new boost::pool<>(sizeof(Node) + _dof*sizeof(dReal)));
        }
        _numnodes = 0;
    }

    /// \brief Perform a normal tree extension operation given a target config vTargetConfig. The difference is that it
    ///        also keeps track of the transform tTarget corresponding to vTargetConfig.
    virtual ExtendType Extend(const std::vector<dReal>& vTargetConfig, NodePtr& plastnode, bool bOneStep=false)
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
            // There could be cases when the node did not move anywhere.
            if( _ComputeDistance(&_vCurConfig[0], _vNewConfig) <= dReal(0.01)*_fStepLength ) {
                return bHasAdded ? ET_Success : ET_Failed;
            }

            if( _fromgoal ) {
                if( params->CheckPathAllConstraints(_vNewConfig, _vCurConfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenEnd, 0xffff|CFO_FillCheckedConfiguration, _constraintreturn) != 0 ) {
                    return bHasAdded ? ET_Success : ET_Failed;
                }
            }
            else { // not _fromgoal
                if( params->CheckPathAllConstraints(_vCurConfig, _vNewConfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart, 0xffff|CFO_FillCheckedConfiguration, _constraintreturn) != 0 ) {
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
                            plastnode = pnode = pnewnode;
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
                            plastnode = pnode = pnewnode;
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
                    plastnode = pnode = pnewnode;
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

    /// \brief Perform a tree extension operation where a randomly selected node is extended such that the tool moves
    ///        along the given vdirection.
    virtual ExtendType ExtendWithDirection(const Vector& vdirection, NodePtr& plastnode, dReal fSampledValue, bool bOneStep=false)
    {
        NodePtr pnode = SampleNode(fSampledValue);
        if( !pnode ) {
            return ET_Failed;
        }
        if( pnode->_transformComputed == 0) {
            if( !_fkfn(VectorWrapper<dReal>(pnode->q, pnode->q + _dof), pnode->pose) ) {
                return ET_Failed;
            }
            pnode->_transformComputed = 1;
        }
        plastnode = pnode;

        bool bHasAdded = false;
        boost::shared_ptr<PlannerBase> planner(_planner);
        PlannerBase::PlannerParametersConstPtr params = planner->GetParameters();

        _vCurConfig.resize(_dof);
        std::copy(pnode->q, pnode->q + _dof, _vCurConfig.begin());
        _curpose.rot = pnode->pose.rot;
        _curpose.trans = pnode->pose.trans;

        _newpose.rot = _curpose.rot;

        // Suppose vdirection is normalized.
        _vstepdirection = vdirection * _fWorkspaceStepLength;

        // Extend the tree
        int maxExtensionIters = 100;
        for( int iter = 0; iter < maxExtensionIters; ++iter ) {
            _newpose.trans = _curpose * _vstepdirection;
            if( !_ikfn(_newpose, _vCurConfig, _vNewConfig) ) {
                return bHasAdded ? ET_Success : ET_Failed;
            }

            _vDeltaConfig = _vNewConfig;
            params->_diffstatefn(_vDeltaConfig, _vCurConfig);
            // Should we still adjust _vDeltaConfig based on _fStepLength here?

            _vNewConfig = _vCurConfig;
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
                            plastnode = pnode = pnewnode;
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
                            plastnode = pnode = pnewnode;
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
                    plastnode = pnode = pnewnode;
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

    /// \brief Select a node in the tree based on the given fSampledValue.
    virtual NodePtr SampleNode(dReal fSampledValue)
    {
        dReal fPrevLevelBound = 0;
        for( int ilevel = 0; ilevel < _maxlevel; ++ilevel ) {
            if( fSampledValue <= _vAccumWeights[ilevel] ) {
                // Select ilevel
                std::set<NodePtr>& setLevelChildren = _vsetLevelNodes.at(ilevel);
                const size_t numLevelNodes = setLevelChildren.size();
                const int iSelectedNode = floorf((fSampledValue - fPrevLevelBound)*numLevelNodes/(_vAccumWeights[ilevel] - fPrevLevelBound));
                typename std::set<NodePtr>::const_iterator itnode = setLevelChildren.begin();
                std::advance(itnode, iSelectedNode);
                return *itnode;
            }
            fPrevLevelBound = _vAccumWeights[ilevel];
        }
        //
        return *(_vsetLevelNodes.at(_maxlevel).begin());
    }

    inline int _EncodeLevel(int level) const {
        return (level <= 0) ?  (-2*level) : (2*level+1);
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
        std::pair<NodePtr, dReal> bestnode {NULL, std::numeric_limits<dReal>::infinity()};
        if( _numnodes == 0 ) {
            return bestnode;
        }
        OPENRAVE_ASSERT_OP((int)vquerystate.size(),==,_dof);

        int currentlevel = _maxlevel; // where the root node is
        // traverse all levels gathering up the children at each level
        dReal fLevelBound = _fMaxLevelBound;
        _vCurrentLevelNodes = {{*_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin(), _ComputeDistance(_vCurrentLevelNodes[0].first->q, vquerystate)}};
        if( _vCurrentLevelNodes[0].first->_usenn ) {
            bestnode = _vCurrentLevelNodes[0];
        }
        while(!_vCurrentLevelNodes.empty()) {
            _vNextLevelNodes.clear();
            //RAVELOG_VERBOSE_FORMAT("level %d (%f) has %d nodes", currentlevel%fLevelBound%_vCurrentLevelNodes.size());
            dReal minchilddist = std::numeric_limits<dReal>::infinity();
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

            _vCurrentLevelNodes.clear();
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

    static int GetNewStaticId() {
        static int s_id = 0;
        int retid = s_id++;
        return retid;
    }

    virtual int GetNumNodes() const {
        return _numnodes;
    }

    virtual const std::vector<dReal>& GetVectorConfig(NodePtr pnode) const
    {
        _vTempConfig.resize(_dof);
        std::copy(pnode->q, pnode->q + _dof, _vTempConfig.begin());
        return _vTempConfig;
    }

    virtual void GetVectorConfig(NodePtr pnode, std::vector<dReal>& v) const
    {
        v.resize(_dof);
        std::copy(pnode->q, pnode->q + _dof, v.begin());
    }

    /// \brief
    inline NodePtr _CreateNode(NodePtr pparent, const std::vector<dReal>& vconfig, uint32_t userdata)
    {
        // Allocate memory for the structure and the internal state vectors
        void* pmemory = _pNodesPool->malloc();
        NodePtr pnode = new (pmemory) Node(pparent, vconfig);
        pnode->_userdata = userdata;
#ifdef _DEBUG
        pnode->id = GetNewStaticId();
#endif
        return pnode;
    }

    /// \brief
    inline NodePtr _CreateNodeWithTransform(NodePtr pparent, const std::vector<dReal>& vconfig, const Transform& pose, uint32_t userdata)
    {
        // Allocate memory for the structure and the internal state vectors
        void* pmemory = _pNodesPool->malloc();
        NodePtr pnode = new (pmemory) Node(pparent, vconfig, pose);
        pnode->_userdata = userdata;
#ifdef _DEBUG
        pnode->id = GetNewStaticId();
#endif
        return pnode;
    }

    virtual NodePtr InsertNode(NodePtr pparent, const std::vector<dReal>& vconfig, uint32_t userdata)
    {
        return _InsertNode(pparent, vconfig, userdata);
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
            _vCurrentLevelNodes = {{*_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin(), _ComputeDistance(_vCurrentLevelNodes[0].first->q, vconfig)}};
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

    /// \brief the recursive function that inserts a configuration into the cache tree
    ///
    /// \param[in] nodein the input node to insert
    /// \param[in] vCurrentLevelNodes the tree nodes at "level" with the respecitve distances computed for them
    /// \param[in] currentlevel the current level traversing
    /// \param[in] fLevelBound pow(_base, level)
    /// \return 1 if point is inserted and parent found. 0 if no parent found and point is not inserted. -1 if parent found but point not inserted since it is close to _mindistance
    int _InsertRecursive(NodePtr nodein, const std::vector< std::pair<NodePtr, dReal> >& vCurrentLevelNodes, int currentlevel, dReal fLevelBound)
    {
        dReal closestDist = std::numeric_limits<dReal>::infinity();
        NodePtr closestNodeInRange = NULL; // one of the nodes in vCurrentLevelNodes such that its distance to nodein is <= fLevelBound
        int enclevel = _EncodeLevel(currentlevel);
        if( enclevel < (int)_vsetLevelNodes.size() ) {
            // build the level below
            _vNextLevelNodes.clear(); // for currentlevel-1
            FOREACHC(itcurrentnode, vCurrentLevelNodes) {
                if( itcurrentnode->second <= fLevelBound ) {
                    if( !closestNodeInRange ) {
                        closestNodeInRange = itcurrentnode->first;
                        closestDist = itcurrentnode->second;
                    }
                    else {
                        if(  itcurrentnode->second < closestDist-g_fEpsilonLinear ) {
                            closestNodeInRange = itcurrentnode->first;
                            closestDist = itcurrentnode->second;
                        }
                        // if distances are close, get the node on the lowest level...
                        else if( itcurrentnode->second < closestDist+_mindistance && itcurrentnode->first->_level < closestNodeInRange->_level ) {
                            closestNodeInRange = itcurrentnode->first;
                            closestDist = itcurrentnode->second;
                        }
                    }
                    if ( (closestDist <= _mindistance) ) {
                        // pretty close, so return as if node was added
                        return -1;
                    }
                }
                if( itcurrentnode->second <= fLevelBound*_fBaseChildMult ) {
                    // node is part of all sets below its level
                    _vNextLevelNodes.push_back(*itcurrentnode);
                }
                // only take the children whose distances are within the bound
                if( itcurrentnode->first->_level == currentlevel ) {
                    FOREACHC(itchild, itcurrentnode->first->_vchildren) {
                        dReal curdist = _ComputeDistance(nodein, *itchild);
                        if( curdist <= fLevelBound*_fBaseChildMult ) {
                            _vNextLevelNodes.emplace_back(*itchild,  curdist);
                        }
                    }
                }
            }

            if( !_vNextLevelNodes.empty() ) {
                _vCurrentLevelNodes.swap(_vNextLevelNodes); // invalidates vCurrentLevelNodes
                // note that after _Insert call, _vCurrentLevelNodes could be complete lost/reset
                int nParentFound = _InsertRecursive(nodein, _vCurrentLevelNodes, currentlevel-1, fLevelBound*_fBaseInv);
                if( nParentFound != 0 ) {
                    return nParentFound;
                }
            }
        }
        else {
            FOREACHC(itcurrentnode, vCurrentLevelNodes) {
                if( itcurrentnode->second <= fLevelBound ) {
                    if( !closestNodeInRange ) {
                        closestNodeInRange = itcurrentnode->first;
                        closestDist = itcurrentnode->second;
                    }
                    else {
                        if(  itcurrentnode->second < closestDist-g_fEpsilonLinear ) {
                            closestNodeInRange = itcurrentnode->first;
                            closestDist = itcurrentnode->second;
                        }
                        // if distances are close, get the node on the lowest level...
                        else if( itcurrentnode->second < closestDist+_mindistance && itcurrentnode->first->_level < closestNodeInRange->_level ) {
                            closestNodeInRange = itcurrentnode->first;
                            closestDist = itcurrentnode->second;
                        }
                    }
                    if ( (closestDist < _mindistance) ) {
                        // pretty close, so return as if node was added
                        return -1;
                    }
                }
            }
        }

        if( !closestNodeInRange ) {
            return 0;
        }

        _InsertDirectly(nodein, closestNodeInRange, closestDist, currentlevel-1, fLevelBound*_fBaseInv);
        _numnodes += 1;
        return 1;
    }

    /// \brief inerts a node directly to parentnode
    ///
    /// If parentnode's configuration is too close to nodein, or parentnode's level is too high, will create dummy child nodes
    bool _InsertDirectly(NodePtr nodein, NodePtr parentnode, dReal parentdist, int maxinsertlevel, dReal fInsertLevelBound)
    {
        int insertlevel = maxinsertlevel;
        if( parentdist <= _mindistance ) {
            // pretty close, so notify parent that there's a similar child already underneath it
            if( parentnode->_hasselfchild ) {
                // already has a similar child, so go one level below...?
                FOREACH(itchild, parentnode->_vchildren) {
                    dReal childdist = _ComputeDistance(nodein, *itchild);
                    if( childdist <= _mindistance ) {
                        return _InsertDirectly(nodein, *itchild, childdist, maxinsertlevel-1, fInsertLevelBound*_fBaseInv);
                    }
                }
                RAVELOG_WARN("inconsistent node found\n");
                return false;
            }
        }
        else {
            // depending on parentdist, might have to insert at a lower level in order to keep the sibling invariant
            dReal fChildLevelBound = fInsertLevelBound;
            while(parentdist < fChildLevelBound) {
                fChildLevelBound *= _fBaseInv;
                insertlevel--;
            }
        }

        // have to add at insertlevel. If currentNodeInRange->_level is > insertlevel+1, will have to clone it. note that it will still represent the same RRT node with same rrtparent
        while( parentnode->_level > insertlevel+1 ) {
            NodePtr clonenode = _CloneNode(parentnode);
            clonenode->_level = parentnode->_level-1;
            parentnode->_vchildren.push_back(clonenode);
            parentnode->_hasselfchild = 1;
            int encclonelevel = _EncodeLevel(clonenode->_level);
            if( encclonelevel >= (int)_vsetLevelNodes.size() ) {
                _vsetLevelNodes.resize(encclonelevel+1);
            }
            _vsetLevelNodes.at(encclonelevel).insert(clonenode);
            _numnodes +=1;
            parentnode = clonenode;
        }

        if( parentdist <= _mindistance ) {
            parentnode->_hasselfchild = 1;
        }
        nodein->_level = insertlevel;
        int enclevel2 = _EncodeLevel(nodein->_level);
        if( enclevel2 >= (int)_vsetLevelNodes.size() ) {
            _vsetLevelNodes.resize(enclevel2+1);
        }
        _vsetLevelNodes.at(enclevel2).insert(nodein);
        parentnode->_vchildren.push_back(nodein);

        if( _minlevel > nodein->_level ) {
            _minlevel = nodein->_level;
        }
        return true;
    }

    inline NodePtr _CloneNode(NodePtr refnode)
    {
        // allocate memory for the structur and the internal state vectors
        void* pmemory = _pNodesPool->malloc();
        NodePtr pnode = new (pmemory) Node(refnode->rrtparent, refnode->q, _dof);
        pnode->_userdata = refnode->_userdata;
#ifdef _DEBUG
        pnode->id = GetNewStaticId();
#endif
        return pnode;
    }

private:
    boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;
    boost::weak_ptr<PlannerBase> _planner;
    dReal _fStepLength = 0.04;
    int _dof = 0; ///< the number of values of each state
    int _fromgoal;

    // cover tree data structures
    boost::shared_ptr< boost::pool<> > _pNodesPool; ///< pool nodes are created from

    std::vector< std::set<NodePtr> > _vsetLevelNodes; ///< _vsetLevelNodes[enc(level)][node] holds the indices of the children of "node" of a given the level. enc(level) maps (-inf,inf) into [0,inf) so it can be indexed by the vector. Every node has an entry in a map here. If the node doesn't hold any children, then it is at the leaf of the tree. _vsetLevelNodes.at(_EncodeLevel(_maxlevel)) is the root.

    dReal _maxdistance = 0.0; ///< maximum possible distance between two states. used to balance the tree. Has to be > 0.
    dReal _mindistance = 0.0; ///< minimum possible distance between two states until they are declared the same
    dReal _base = 1.5, _fBaseInv = 1.0/_base, _fBaseChildMult = 1.0/(_base-1.0); ///< a constant used to control the max level of traversion. _fBaseInv = 1/_base, _fBaseChildMult=1/(_base-1)
    int _maxlevel = 0; ///< the maximum allowed levels in the tree, this is where the root node starts (inclusive)
    int _minlevel = 0; ///< the minimum allowed levels in the tree (inclusive)
    int _numnodes = 0; ///< the number of nodes in the current tree starting at the root at _vsetLevelNodes.at(_EncodeLevel(_maxlevel))
    dReal _fMaxLevelBound = 0.0; // pow(_base, _maxlevel)

    // cache
    std::vector<NodePtr> _vchildcache;
    std::set<NodePtr> _setchildcache;
    std::vector<dReal> _vNewConfig, _vDeltaConfig, _vCurConfig;
    mutable std::vector<dReal> _vTempConfig;
    ConstraintFilterReturnPtr _constraintreturn;

    mutable std::vector< std::pair<NodePtr, dReal> > _vCurrentLevelNodes, _vNextLevelNodes;
    mutable std::vector< std::vector<NodePtr> > _vvCacheNodes;

    // extra stuff for workspace sampling
    dReal _fWorkspaceStepLength;
    boost::function<bool(const std::vector<dReal>&, Transform&)> _fkfn; ///< forward kinematics function
    boost::function<bool(const Transform&, const std::vector<dReal>&, std::vector<dReal>&)> _ikfn; ///< inverse kinematics function
    std::vector<dReal> _vAccumWeights; ///< weights for each level of the tree, used for sampling tree nodes.
    Transform _curpose, _newpose;
    Vector _vstepdirection;
};

class BirrtPlanner2 : public RrtPlanner<NodeWithTransform>
{
public:
    BirrtPlanner2(EnvironmentBasePtr penv) : RrtPlanner<NodeWithTransform>(penv), _treeForward(0), _treeBackward(1)
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

    /// \brief
    virtual bool InitPlan(RobotBasePtr probot, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _environmentid = GetEnv()->GetId();

        _parameters.reset(new RRTWorkspaceSamplingParameters());
        _parameters->copy(pparams);
        // Do not call RrtPlanner::_InitPlan
        // if( !RrtPlanner<NodeWithTransform>::_InitPlan(probot, _parameters) ) {
        //     _parameters.reset();
        //     return false;
        // }
        _robot = probot;
        if( _parameters->_fWorkspaceSamplingBiasProb > 0 ) {
            // _pmanip and stuff are only needed when using workspace sampling
            _pmanip = _robot->GetManipulator(_parameters->manipname);
            if( !_pmanip ) {
                RAVELOG_ERROR_FORMAT("env=%d, failed to get manip %s from robot %s", _environmentid%_parameters->manipname%_robot->GetName());
                return false;
            }

            // Need to set iksolver beforehand
            _piksolver = _pmanip->GetIkSolver();
            if( !_piksolver ) {
                RAVELOG_ERROR_FORMAT("env=%d, no ik solver set for manip %s", _environmentid%_parameters->manipname);
                return false;
            }
        }

        // Note: for now, only support iktype transform 6d.
        _iktype = IKP_Transform6D;
        _vcacheikreturns.reserve(8);

        if( !_uniformsampler ) {
            _uniformsampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        }
        _uniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        FOREACHC(it, _parameters->_listInternalSamplers) {
            (*it)->SetSeed(_parameters->_nRandomGeneratorSeed);
        }

        PlannerParameters::StateSaver savestate(_parameters);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(), GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs, false);

        dReal fMaxDistance = _parameters->_distmetricfn(_parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit);
        _treeForward.Init(shared_planner(), _parameters->GetDOF(), _parameters->_distmetricfn,
                          boost::bind(&BirrtPlanner2::FK, this, _1, _2),
                          boost::bind(&BirrtPlanner2::IK, this, _1, _2, _3),
                          _parameters->_fStepLength, _parameters->_fWorkspaceStepLength, fMaxDistance);
        _treeBackward.Init(shared_planner(), _parameters->GetDOF(), _parameters->_distmetricfn,
                           boost::bind(&BirrtPlanner2::FK, this, _1, _2),
                           boost::bind(&BirrtPlanner2::IK, this, _1, _2, _3),
                           _parameters->_fStepLength, _parameters->_fWorkspaceStepLength, fMaxDistance);

        // Read all initials
        if( (_parameters->vinitialconfig.size() % _parameters->GetDOF()) != 0 ) {
            RAVELOG_ERROR_FORMAT("env=%d, initials are not properly specified. vgoalconfig.size()=%d; ndof=%d", _environmentid%_parameters->vinitialconfig.size()%_parameters->GetDOF());
            _parameters.reset();
            return false;
        }

        std::vector<dReal> vinitial(_parameters->GetDOF());
        _vecInitialNodes.clear();
        _sampleConfig.resize(_parameters->GetDOF());
        for( size_t iinitial = 0; iinitial < _parameters->vinitialconfig.size(); iinitial += _parameters->GetDOF() ) {
            std::copy(_parameters->vinitialconfig.begin() + iinitial,
                      _parameters->vinitialconfig.begin() + iinitial + _parameters->GetDOF(),
                      vinitial.begin());
            _filterreturn->Clear();
            int ret = _parameters->CheckPathAllConstraints(vinitial, vinitial, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart, CFO_FillCollisionReport, _filterreturn);
            if( ret == 0 ) {
                // userdata being the initial index
                _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitial, _vecInitialNodes.size()));
            }
            else {
                // initial invalidated due to constraints
                RAVELOG_WARN_FORMAT("env=%d, initial %d fails constraints ret=0x%x: %s", _environmentid%iinitial%ret%_filterreturn->_report.__str__());
            }
        } // end for iinitial
        if( _treeForward.GetNumNodes() == 0 && !_parameters->_sampleinitialfn ) {
            RAVELOG_WARN_FORMAT("env=%d, no initials specified", _environmentid);
            _parameters.reset();
            return false;
        }

        // Read all goals
        if( (_parameters->vgoalconfig.size() % _parameters->GetDOF()) != 0 ) {
            RAVELOG_ERROR_FORMAT("env=%d, goals are not properly specified. vgoalconfig.size()=%d; ndof=%d", _environmentid%_parameters->vgoalconfig.size()%_parameters->GetDOF());
            _parameters.reset();
            return false;
        }

        std::vector<dReal> vgoal(_parameters->GetDOF());
        _vecGoalNodes.clear();
        _nValidGoals = 0;
        for( size_t igoal = 0; igoal < _parameters->vgoalconfig.size(); igoal += _parameters->GetDOF() ) {
            std::copy(_parameters->vgoalconfig.begin() + igoal,
                      _parameters->vgoalconfig.begin() + igoal + _parameters->GetDOF(),
                      vgoal.begin());
            _filterreturn->Clear();
            int ret = _parameters->CheckPathAllConstraints(vgoal, vgoal, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart, CFO_FillCollisionReport, _filterreturn);
            if( ret == 0 ) {
                // userdata being the goal index
                _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
                ++_nValidGoals;
            }
            else {
                // goal invalidated due to constraints
                RAVELOG_WARN_FORMAT("env=%d, goal %d fails constraints ret=0x%x: %s", _environmentid%igoal%ret%_filterreturn->_report.__str__());
                _vecGoalNodes.push_back(NULL); // have to push back a dummy so that indices are not messed up.
            }
        } // end for igoal

        if( _treeBackward.GetNumNodes() == 0 && !_parameters->_samplegoalfn ) {
            RAVELOG_WARN_FORMAT("env=%d, no goals specified", _environmentid);
            _parameters.reset();
            return false;
        }

        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 10000;
        }

        _vgoalpaths.clear();
        if( _vgoalpaths.capacity() < _parameters->_minimumgoalpaths ) {
            _vgoalpaths.reserve(_parameters->_minimumgoalpaths);
        }

        RAVELOG_DEBUG_FORMAT("env=%d, BiRRT planner with workspace sampling initialized, numinitials=%d; numgoals=%d; fStepLength=%f; fGoalBiasProb=%f; fWorkspaceSamplingProb=%f; fWorkspaceStepLength=%f", _environmentid%_vecInitialNodes.size()%_treeBackward.GetNumNodes()%
                             _parameters->_fStepLength%_parameters->_fGoalBiasProb%_parameters->_fWorkspaceSamplingBiasProb%_parameters->_fWorkspaceStepLength);
        return true;
    }

    /// \brief Compute forward kinematics of the given configuration.
    virtual bool FK(const std::vector<dReal>& vconfig, Transform& pose)
    {
        if( _parameters->SetStateValues(vconfig, 0) != 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, failed to set state", _environmentid);
            return false;
        }
        pose = _pmanip->GetTransform();
        return true;
    }

    /// \brief Compute inverse kinematics of the given tool transform.
    ///
    /// \param[in] pose, the given tool transform.
    /// \param[in] vrefconfig, the returned configuration is the one closest to this reference.
    /// \param[out] vconfig, a configuration that gives the specified tool pose and is closest to vrefconfig.
    virtual bool IK(const Transform& pose, const std::vector<dReal>& vrefconfig, std::vector<dReal>& vconfig)
    {
        _ikparam.SetTransform6D(pose);
        _pmanip->FindIKSolutions(_ikparam, IKFO_CheckEnvCollisions, _vcacheikreturns);
        if( _vcacheikreturns.empty() ) {
            return false;
        }

        dReal fMinConfigDist = 1e30;
        dReal fCurConfigDist;
        IkReturnPtr bestikret;
        for( const IkReturnPtr& ikret : _vcacheikreturns ) {
            fCurConfigDist = _parameters->_distmetricfn(vrefconfig, ikret->_vsolution);
            if( fCurConfigDist < fMinConfigDist ) {
                bestikret = ikret;
            }
        }
        if( !bestikret ) {
            RAVELOG_WARN_FORMAT("env=%d, !bestikret is true even though _vcacheikreturns.size()=%d", _environmentid%_vcacheikreturns.size());
            return false;
        }
        vconfig.swap(bestikret->_vsolution);
        return true;
    }

    /// \brief Find a piecewise linear path that connects an initial configuration to a goal configuration.
    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        _goalindex = -1;
        _startindex = -1;
        if( !_parameters ) {
            return PlannerStatus("BirrtPlanner2::PlanPath - Error, planner not initialized", PS_Failed);
        }

        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        uint32_t basetime = utils::GetMilliTime();

        PlannerParameters::StateSaver savestate(_parameters);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(), GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs, false);

        SpatialTree2<NodeWithTransform>* TreeA = &_treeForward;
        SpatialTree2<NodeWithTransform>* TreeB = &_treeBackward;
        NodeWithTransform* iConnectedA = NULL;
        NodeWithTransform* iConnectedB = NULL;
        int iter = 0;

        bool bSampleGoal = true;
        PlannerProgress progress;
        PlannerAction callbackaction = PA_None;
        ExtendType et;
        while( _vgoalpaths.size() < _parameters->_minimumgoalpaths && iter < 3*_parameters->_nMaxIterations ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, iter=%d, forward=%d, backward=%d", _environmentid%(iter/3)%_treeForward.GetNumNodes()%_treeBackward.GetNumNodes());
            ++iter;

            callbackaction = _CallCallbacks(progress);
            if( callbackaction == PA_Interrupt ) {
                return PlannerStatus("Planning was interrupted", PS_Interrupted);
            }
            else if( callbackaction == PA_ReturnWithAnySolution ) {
                if( !_vgoalpaths.empty() ) {
                    break;
                }
            }

            if( _parameters->_nMaxPlanningTime > 0 ) {
                uint32_t elapsedtime = utils::GetMilliTime() - basetime;
                if( elapsedtime > _parameters->_nMaxPlanningTime ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, time exceeded (%dms) so breaking. iter=%d < %d", _environmentid%elapsedtime%(iter/3)%_parameters->_nMaxIterations);
                    break;
                }
            }

            if( !!_parameters->_samplegoalfn ) {
                std::vector<dReal> vgoal;
                if( _parameters->_samplegoalfn(vgoal) ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, inserting new goal index %d", _environmentid%_vecGoalNodes.size());
                    _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
                    ++_nValidGoals;
                }
            }

            if( !!_parameters->_sampleinitialfn ) {
                std::vector<dReal> vinitial;
                if( _parameters->_sampleinitialfn(vinitial) ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, inserting new initial index %d", _environmentid%_vecInitialNodes.size());
                    _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitial, _vecInitialNodes.size()));
                }
            }

            // Only call SampleSequenceOneReal when !bSampleGoal so that the behavior is exactly the
            // same with the original rrt in case _fWorkspaceSamplingBiasProb = 0.
            dReal fSampledValue1 = bSampleGoal ? 0 : _uniformsampler->SampleSequenceOneReal();
            if( fSampledValue1 >= _parameters->_fGoalBiasProb && fSampledValue1 < _parameters->_fGoalBiasProb + _parameters->_fWorkspaceSamplingBiasProb ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, iter=%d, fSampledValue=%f so doing workspace sampling", _environmentid%(iter/3)%fSampledValue1);
                // Sample one out of six directions in tool local frame.
                uint32_t sampleindex = _uniformsampler->SampleSequenceOneUInt32();
                uint32_t directionindex = sampleindex % 6;
                switch( directionindex ) {
                case 0:
                    _vcachedirection.x = 1;
                    _vcachedirection.y = 0;
                    _vcachedirection.z = 0;
                    break;
                case 1:
                    _vcachedirection.x = 0;
                    _vcachedirection.y = 1;
                    _vcachedirection.z = 0;
                    break;
                case 2:
                    _vcachedirection.x = 0;
                    _vcachedirection.y = 0;
                    _vcachedirection.z = 1;
                    break;
                case 3:
                    _vcachedirection.x = -1;
                    _vcachedirection.y = 0;
                    _vcachedirection.z = 0;
                    break;
                case 4:
                    _vcachedirection.x = 0;
                    _vcachedirection.y = -1;
                    _vcachedirection.z = 0;
                    break;
                case 5:
                    _vcachedirection.x = 0;
                    _vcachedirection.y = 0;
                    _vcachedirection.z = -1;
                    break;
                }
                // Sample another number for selecting a node on a tree.
                dReal fSampledValue2 = _uniformsampler->SampleSequenceOneReal();

                // Extend A
                et = TreeA->ExtendWithDirection(_vcachedirection, iConnectedA, fSampledValue2);
                if( et == ET_Failed ) {
                    if( iter > 3*_parameters->_nMaxIterations ) {
                        RAVELOG_WARN_FORMAT("env=%d, iterations exceeded %d", _environmentid%_parameters->_nMaxIterations);
                        break;
                    }
                    continue;
                }
            }
            else {
                // Use normal RRT logic
                _sampleConfig.clear();
                if( (bSampleGoal || fSampledValue1 < _parameters->_fGoalBiasProb) && _nValidGoals > 0 ) {
                    bSampleGoal = false;

                    uint32_t bestgoalindex = -1;
                    for( size_t testiter = 0; testiter < _vecGoalNodes.size()*3; ++testiter ) {
                        uint32_t sampleindex = _uniformsampler->SampleSequenceOneUInt32();
                        uint32_t goalindex = sampleindex%_vecGoalNodes.size();
                        if( !_vecGoalNodes.at(goalindex) ) {
                            // Sampled a dummy goal so continue.
                            continue;
                        }

                        bool bfound = false;
                        FOREACHC(itpath, _vgoalpaths) {
                            if( goalindex == (uint32_t)itpath->goalindex ) {
                                bfound = true;
                                break;
                            }
                        }
                        if( !bfound ) {
                            bestgoalindex = goalindex;
                            break;
                        }
                    } // end for testiter
                    if( bestgoalindex != uint32_t(-1) ) {
                        _treeBackward.GetVectorConfig(_vecGoalNodes.at(bestgoalindex), _sampleConfig);
                    }
                }

                if( _sampleConfig.empty() ) {
                    if( !_parameters->_samplefn(_sampleConfig) ) {
                        continue;
                    }
                }

                // Extend A
                et = TreeA->Extend(_sampleConfig, iConnectedA);
                // Although this check is not necessary, having it improves running time
                if( et == ET_Failed ) {
                    if( iter > 3*_parameters->_nMaxIterations ) {
                        RAVELOG_WARN_FORMAT("env=%d, iterations exceeded %d", _environmentid%_parameters->_nMaxIterations);
                        break;
                    }
                    continue;
                }
            } // end if fSampledValue1

            // Extend B towards A
            et = TreeB->Extend(TreeA->GetVectorConfig(iConnectedA), iConnectedB);
            if( et == ET_Connected ) {
                // Trees are connected. Now process goals.
                _vgoalpaths.push_back(GOALPATH());
                _ExtractPath(_vgoalpaths.back(),
                             TreeA == &_treeForward ? iConnectedA : iConnectedB,
                             TreeA == &_treeBackward ? iConnectedA : iConnectedB);
                int goalindex = _vgoalpaths.back().goalindex;
                int startindex = _vgoalpaths.back().startindex;
                if( IS_DEBUGLEVEL(Level_Debug) ) {
                    std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                    ss << "startvalues=[";
                    for( int idof = 0; idof < _parameters->GetDOF(); ++idof ) {
                        ss << _vgoalpaths.back().qall.at(idof) << ", ";
                    }
                    ss << "]; goalvalues=[";
                    for( int idof = 0; idof < _parameters->GetDOF(); ++idof ) {
                        ss << _vgoalpaths.back().qall.at(_vgoalpaths.back().qall.size() - _parameters->GetDOF() + idof) << ", ";
                    }
                    ss << "]; ";
                    RAVELOG_DEBUG_FORMAT("env=%d, found a goal, startindex=%d; goalindex=%d; pathlength=%f; %s", _environmentid%startindex%goalindex%_vgoalpaths.back().length%ss.str());
                }
                if( _vgoalpaths.size() >= _parameters->_minimumgoalpaths || _vgoalpaths.size() >= _nValidGoals ) {
                    break;
                }

                bSampleGoal = true;
                // More goals requested, so make sure to remove all the nodes pointing to the newly found goal
                _treeBackward.InvalidateNodesWithParent(_vecGoalNodes.at(goalindex));

            } // end if et == ET_Connected

            std::swap(TreeA, TreeB);
            iter += 3;
            if( iter >= 3*_parameters->_nMaxIterations ) {
                RAVELOG_WARN_FORMAT("env=%d, iterations exceeded %d", _environmentid%_parameters->_nMaxIterations);
                break;
            }
            progress._iteration = iter/3;

        } // end while (main planning loop)

        if( _vgoalpaths.empty() ) {
            std::string description = str(boost::format(_("env=%d, plan failed in %fs, iter=%d, nMaxIterations=%d"))%_environmentid%(0.001f*(float)(utils::GetMilliTime() - basetime))%(iter/3)%_parameters->_nMaxIterations);
            RAVELOG_WARN(description);
            return PlannerStatus(description, PS_Failed);
        }

        std::vector<GOALPATH>::iterator itbest = _vgoalpaths.begin();
        FOREACH(itpath, _vgoalpaths) {
            if( itpath->length < itbest->length ) {
                itbest = itpath;
            }
        }
        _goalindex = itbest->goalindex;
        _startindex = itbest->startindex;
        if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        ptraj->Insert(ptraj->GetNumWaypoints(), itbest->qall, _parameters->_configurationspecification);

        std::string description = str(boost::format(_("env=%d, plan success, iters=%d, path=%d points, computation time=%fs"))%_environmentid%progress._iteration%ptraj->GetNumWaypoints()%(0.001f*(float)(utils::GetMilliTime() - basetime)));
        RAVELOG_DEBUG(description);

        PlannerStatus status = _ProcessPostPlanners(_robot, ptraj);
        status.description = description;
        return status;
    }

    /// \brief Extract the path connecting the roots of _treeForward and _treeBackward.
    virtual void _ExtractPath(GOALPATH& goalpath, NodeBase* iConnectedForward, NodeBase* iConnectedBackward)
    {
        const int ndof = _parameters->GetDOF();
        _cachedpath.clear();

        // Add nodes from treeForward
        goalpath.startindex = -1;
        NodeWithTransform* pforwardnode = (NodeWithTransform*)iConnectedForward;
        while( true ) {
            _cachedpath.insert(_cachedpath.begin(), pforwardnode->q, pforwardnode->q + ndof);
            if( !pforwardnode->rrtparent ) {
                goalpath.startindex = pforwardnode->_userdata;
                break;
            }
            pforwardnode = pforwardnode->rrtparent;
        }

        // Add nodes from treeBackward
        goalpath.goalindex = -1;
        NodeWithTransform* pbackwardnode = (NodeWithTransform*)iConnectedBackward;
        while( true ) {
            _cachedpath.insert(_cachedpath.end(), pbackwardnode->q, pbackwardnode->q + ndof);
            if( !pbackwardnode->rrtparent ) {
                goalpath.goalindex = pbackwardnode->_userdata;
                break;
            }
            pbackwardnode = pbackwardnode->rrtparent;
        }

        BOOST_ASSERT(goalpath.goalindex >= 0 && goalpath.goalindex < (int)_vecGoalNodes.size());

        _SimpleOptimizePath(_cachedpath, 10);

        goalpath.qall.resize(_cachedpath.size());
        std::copy(_cachedpath.begin(), _cachedpath.end(), goalpath.qall.begin());

        goalpath.length = 0;
        std::vector<dReal> vdiff(goalpath.qall.begin(), goalpath.qall.begin() + ndof);
        _parameters->_diffstatefn(vdiff, std::vector<dReal>(goalpath.qall.end() - ndof, goalpath.qall.end()));
        for( int idof = 0; idof < ndof; ++idof ) {
            dReal fLength = RaveFabs(vdiff.at(idof));
            if( _parameters->_vConfigVelocityLimit.at(idof) != 0 ) {
                fLength /= _parameters->_vConfigVelocityLimit[idof];
            }
            goalpath.length += fLength;
        }
        return;
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
        RAVELOG_VERBOSE_FORMAT("env=%d, dumping RRT tree to %s", _environmentid%filename);
        std::ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        _treeForward.DumpTree(f);
        _treeBackward.DumpTree(f);
        return true;
    }

protected:
    int _environmentid;
    RobotBase::ManipulatorPtr _pmanip;
    IkSolverBasePtr _piksolver;

    RRTWorkspaceSamplingParametersPtr _parameters;

    SpatialTree2< NodeWithTransform > _treeForward, _treeBackward;

    std::vector< NodeWithTransform* > _vecInitialNodes, _vecGoalNodes;
    size_t _nValidGoals;
    std::vector< GOALPATH > _vgoalpaths;

    IkParameterization _ikparam;
    IkParameterizationType _iktype;
    std::vector<IkReturnPtr> _vcacheikreturns;
    Vector _vcachedirection;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(NodeWithTransform)
BOOST_TYPEOF_REGISTER_TYPE(BirrtPlanner2::GOALPATH)
#endif

#endif // #ifndef BIRRT_WORKSPACE_PLANNER_H
