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
        // Assign rot, trans separately to skip checking of rot normalization. Is it ok, though?
        this->pose.rot = pose.rot;
        this->pose.trans = pose.trans;
        _level = 0;
        _hasselfchild = 0;
        _usenn = 1;
        _userdata = 0;
        _transformComputed = 1;
    }

    NodeWithTransform(NodeWithTransform* pparent, const dReal* pconfig, int ndof, const Transform& pose) : rrtparent(pparent) {
        std::copy(pconfig, pconfig + ndof, q);
        this->pose.rot = pose.rot;
        this->pose.trans = pose.trans;
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
    Transform pose;
    dReal q[0];
};

/// \brief (TODO)
template <typename Node>
class SpatialTree2 : public SpatialTree<Node>
{
public:
    typedef Node* NodePtr;

    SpatialTree2(int fromgoal) : SpatialTree<Node>(fromgoal)
    {
        _fromgoal = fromgoal;
        _fStepLength = 0.04f;
        _dof = 0;
        _numnodes = 0;
        _base = 1.5; // optimal is 1.3?
        _fBaseInv = 1/_base;
        _fBaseChildMult = 1/(_base-1);
        _maxdistance = 0;
        _mindistance = 0;
        _maxlevel = 0;
        _minlevel = 0;
        _fMaxLevelBound = 0;
    }

    ~SpatialTree2()
    {
        Reset();
    }

    virtual void Init(boost::weak_ptr<PlannerBase> planner,
                      int dof,
                      boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn,
                      boost::function<bool(const std::vector<dReal>&, Transform&)>& fkfn,
                      boost::function<bool(const Transform&, std::vector<dReal>&)>& ikfn,
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
            _pNodesPool.reset(new boost::pool<>(sizeof(Node) + _dof*sizeof(dReal)));
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
    virtual ExtendType ExtendWithDirection(const Vector& vdirection, NodeBasePtr& plastnode, dReal fSampledValue, bool bOneStep=false)
    {
        NodePtr pnode = SampleNode(fSampledValue);
        if( !pnode ) {
            return ET_Failed;
        }
        if( pnode->_transformComputed == 0) {
            _fkfn(VectorWrapper<dReal>(pnode->q, pnode->q + _dof), pnode->pose);
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
            _newpose.trans = _curpose.trans + _curpose.rotate(_vstepdirection);
            if( !_ikfn(_newpose, _vNewConfig) ) {
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

    /// \brief Select a node in the tree based on the given fSampledValue.
    virtual NodePtr SampleNode(dReal fSampledValue)
    {
        dReal fPrevLevelBound = 0;
        for( int ilevel = 0; ilevel < _maxlevel; ++ilevel ) {
            if( fSampledValue <= _vAccumWeights[ilevel] ) {
                // Select ilevel
                std::set<NodePtr>& setLevelChildren = _vsetLevelNodes.at(ilevel);
                size_t numLevelNodes = setLevelChildren.size();
                int iSelectedNode = floorf((fSampledValue - fPrevLevelBound)*numLevelNodes/(_vAccumWeights[ilevel] - fPrevLevelBound));
                typename std::set<NodePtr>::iterator itnode = setLevelChildren.begin();
                std::advance(itnode, iSelectedNode);
                return *itnode;
            }
            fPrevLevelBound = _vAccumWeights[ilevel];
        }
        //
        return *(_vsetLevelNodes.at(_maxlevel).begin());
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
            _vNextLevelNodes.resize(0); // for currentlevel-1
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

            if( _vNextLevelNodes.size() > 0 ) {
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
        NodePtr node = new (pmemory) Node(refnode->rrtparent, refnode->q, _dof);
        node->_userdata = refnode->_userdata;
        return node;
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
    std::vector<NodePtr> _vchildcache;
    std::set<NodePtr> _setchildcache;
    std::vector<dReal> _vNewConfig, _vDeltaConfig, _vCurConfig;
    mutable std::vector<dReal> _vTempConfig;
    ConstraintFilterReturnPtr _constraintreturn;

    mutable std::vector< std::pair<NodePtr, dReal> > _vCurrentLevelNodes, _vNextLevelNodes;
    mutable std::vector< std::vector<NodePtr> > _vvCacheNodes;

    // extra stuff for workspace sampling
    dReal _fWorkspaceStepLength;
    boost::function<bool(const std::vector<dReal>&, Transform&)> _fkfn; // forward kinematics
    boost::function<bool(const Transform&, std::vector<dReal>&)> _ikfn; // inverse kinematics
    std::vector<dReal> _vAccumWeights;
    Transform _curpose, _newpose;
    Vector _vstepdirection;
};

// (TODO)
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

    /// \brief
    virtual void _ExtractPath(GOALPATH& goalpath, NodeBase* iConnectedForward, NodeBase* iConnectedBackward)
    {
        const int ndof = _parameters->GetDOF();
        _cachedpath.resize(0);

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
        RAVELOG_VERBOSE_FORMAT("env=%d, dumping RRT tree to %s", GetEnv()->GetId()%filename);
        std::ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        _treeForward.DumpTree(f);
        _treeBackward.DumpTree(f);
        return true;
    }

protected:
    RRTParametersPtr _parameters;
    SpatialTree2< NodeWithTransform > _treeForward, _treeBackward;
    dReal _fGoalBiasProb;
    dReal _fWorkspaceSamplingBiasProb;
    std::vector< NodeBase* > _vecGoalNodes;
    size_t _nValidGoals;
    std::vector< GOALPATH > _vGoalPaths;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(NodeWithTransform)
BOOST_TYPEOF_REGISTER_TYPE(BirrtPlanner2::GOALPATH)
#endif

#endif // #ifndef BIRRT_WORKSPACE_PLANNER_H
