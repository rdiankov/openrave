// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#ifndef RAVE_PLANNERS_H
#define RAVE_PLANNERS_H

#include "openraveplugindefs.h"

#include <boost/pool/pool.hpp>

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave_plugins_rplanners", msgid)

enum ExtendType {
    ET_Failed=0,
    ET_Sucess=1,
    ET_Connected=2
};

#ifndef __clang__
/// \brief wraps a static array of T onto a std::vector. Destructor just NULLs out the pointers. Any dynamic resizing operations on this vector wrapper would probably cause the problem to segfault, so use as if it is constant.
///
/// This is an optimization, so if there's a compiler this doesn't compile for, #ifdef it with the regular vector.
template <class T>
class VectorWrapper : public std::vector<T>
{
public:
    VectorWrapper() {
        this->_M_impl._M_start = this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = NULL;
    }

    VectorWrapper(T* sourceArray, T* sourceArrayEnd)
    {
        this->_M_impl._M_start = sourceArray;
        this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = sourceArrayEnd;
    }

    // dangerous! user has to make sure not to modify anything...
    VectorWrapper(const T* sourceArray, const T* sourceArrayEnd)
    {
        this->_M_impl._M_start = const_cast<T*>(sourceArray);
        this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = const_cast<T*>(sourceArrayEnd);
    }

    ~VectorWrapper() {
        this->_M_impl._M_start = this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = NULL;
    }

    void WrapArray(T* sourceArray, int arraySize)
    {
        this->_M_impl._M_start = sourceArray;
        this->_M_impl._M_finish = this->_M_impl._M_end_of_storage = sourceArray + arraySize;
    }
};
#else // __clang__
#define VectorWrapper std::vector
#endif // __clang__

class NodeBase
{
public:
};
typedef NodeBase* NodeBasePtr;

/// \brief node in freespace. be careful when constructing since placement new operator is needed.
class SimpleNode : public NodeBase
{
public:
    SimpleNode(SimpleNode* parent, const vector<dReal>& config) : rrtparent(parent) {
        std::copy(config.begin(), config.end(), q);
        _level = 0;
        _hasselfchild = 0;
        _usenn = 1;
        _userdata = 0;
    }
    SimpleNode(SimpleNode* parent, const dReal* pconfig, int dof) : rrtparent(parent) {
        std::copy(pconfig, pconfig+dof, q);
        _level = 0;
        _hasselfchild = 0;
        _usenn = 1;
        _userdata = 0;
    }
    ~SimpleNode() {
    }

    SimpleNode* rrtparent; ///< pointer to the RRT tree parent
    std::vector<SimpleNode*> _vchildren; ///< cache tree direct children of this node (for the next cache level down). Has nothing to do with the RRT tree.
    int16_t _level; ///< the level the node belongs to
    uint8_t _hasselfchild; ///< if 1, then _vchildren has contains a clone of this node in the level below it.
    uint8_t _usenn; ///< if 1, then use part of the nearest neighbor search, otherwise ignore
    uint32_t _userdata; ///< user specified data tagging this node

#ifdef _DEBUG
    int id;
#endif
    dReal q[0]; // the configuration immediately follows the struct
};

class SpatialTreeBase
{
public:
    virtual void Init(boost::weak_ptr<PlannerBase> planner, int dof, boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn, dReal fStepLength, dReal maxdistance) = 0;

    /// inserts a node in the try
    virtual NodeBasePtr InsertNode(NodeBasePtr parent, const vector<dReal>& config, uint32_t userdata) = 0;

    /// returns the nearest neighbor
    virtual std::pair<NodeBasePtr, dReal> FindNearestNode(const vector<dReal>& q) const = 0;

    /// \brief returns a temporary config stored on the local class. Next time this function is called, it will overwrite the config
    virtual const vector<dReal>& GetVectorConfig(NodeBasePtr node) const = 0;

    virtual void GetVectorConfig(NodeBasePtr nodebase, std::vector<dReal>& v) const = 0;

    /// extends toward pNewConfig
    /// \return true if extension reached pNewConfig
    virtual ExtendType Extend(const std::vector<dReal>& pTargetConfig, NodeBasePtr& lastnode, bool bOneStep=false) = 0;

    /// \brief the dof configured for
    virtual int GetDOF() = 0;

    virtual bool Validate() const = 0;

    virtual int GetNumNodes() const = 0;

    /// invalidates any nodes that point to parentbase. nodes can still be references from outside, but just won't be used as part of the nearest neighbor search
    virtual void InvalidateNodesWithParent(NodeBasePtr parentbase) = 0;
};

/// Cache stores configuration information in a data structure based on the Cover Tree (Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf)
template <typename Node>
class SpatialTree : public SpatialTreeBase
{
public:
    typedef Node* NodePtr;

    SpatialTree(int fromgoal) {
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

    ~SpatialTree() {
        Reset();
    }

    virtual void Init(boost::weak_ptr<PlannerBase> planner, int dof, boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn, dReal fStepLength, dReal maxdistance)
    {
        Reset();
        if( !!_pNodesPool ) {
            // see if pool can be preserved
            if( _dof != dof ) {
                _pNodesPool.reset();
            }
        }
        if( !_pNodesPool ) {
            _pNodesPool.reset(new boost::pool<>(sizeof(Node)+dof*sizeof(dReal)));
        }
        _planner = planner;
        _distmetricfn = distmetricfn;
        _dof = dof;
        _vNewConfig.resize(dof);
        _vDeltaConfig.resize(dof);
        _vTempConfig.resize(dof);
        _maxdistance = maxdistance;
        _mindistance = 0.001*fStepLength; ///< is it ok?
        _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
        _minlevel = _maxlevel - 1;
        _fMaxLevelBound = RavePow(_base, _maxlevel);
        int enclevel = _EncodeLevel(_maxlevel);
        if( enclevel >= (int)_vsetLevelNodes.size() ) {
            _vsetLevelNodes.resize(enclevel+1);
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
            _pNodesPool.reset(new boost::pool<>(sizeof(Node)+_dof*sizeof(dReal)));
        }
        _numnodes = 0;
    }

    inline dReal _ComputeDistance(const dReal* config0, const dReal* config1) const
    {
        return _distmetricfn(VectorWrapper<const dReal>(config0, config0+_dof), VectorWrapper<dReal>(config1, config1+_dof));
    }

    inline dReal _ComputeDistance(const dReal* config0, const std::vector<dReal>& config1) const
    {
        return _distmetricfn(VectorWrapper<dReal>(config0,config0+_dof), config1);
    }

    inline dReal _ComputeDistance(NodePtr node0, NodePtr node1) const
    {
        return _distmetricfn(VectorWrapper<dReal>(node0->q, &node0->q[_dof]), VectorWrapper<dReal>(node1->q, &node1->q[_dof]));
    }

    std::pair<NodeBasePtr, dReal> FindNearestNode(const std::vector<dReal>& vquerystate) const
    {
        return _FindNearestNode(vquerystate);
    }

    virtual NodeBasePtr InsertNode(NodeBasePtr parent, const vector<dReal>& config, uint32_t userdata)
    {
        return _InsertNode((NodePtr)parent, config, userdata);
    }

    virtual void InvalidateNodesWithParent(NodeBasePtr parentbase)
    {
        //BOOST_ASSERT(Validate());
        uint64_t starttime = utils::GetNanoPerformanceTime();
        // first gather all the nodes, and then delete them in reverse order they were originally added in
        NodePtr parent = (NodePtr)parentbase;
        parent->_usenn = 0;
        _setchildcache.clear(); _setchildcache.insert(parent);
        int numruns=0;
        bool bchanged=true;
        while(bchanged) {
            bchanged=false;
            FOREACHC(itchildren, _vsetLevelNodes) {
                FOREACHC(itchild, *itchildren) {
                    if( _setchildcache.find(*itchild) == _setchildcache.end() && _setchildcache.find((*itchild)->rrtparent) != _setchildcache.end() ) {
                        (*itchild)->_usenn = 0;
                        _setchildcache.insert(*itchild);
                        bchanged=true;
                    }
                }
            }
            ++numruns;
        }
        RAVELOG_VERBOSE("computed in %fs", (1e-9*(utils::GetNanoPerformanceTime()-starttime)));
    }

    /// deletes all nodes that have parentindex as their parent
    virtual void _DeleteNodesWithParent(NodeBasePtr parentbase)
    {
        BOOST_ASSERT(Validate());
        uint64_t starttime = utils::GetNanoPerformanceTime();
        // first gather all the nodes, and then delete them in reverse order they were originally added in
        NodePtr parent = (NodePtr)parentbase;
        if( _vchildcache.capacity() == 0 ) {
            _vchildcache.reserve(128);
        }
        _vchildcache.resize(0); _vchildcache.push_back(parent);
        _setchildcache.clear(); _setchildcache.insert(parent);
        int numruns=0;
        bool bchanged=true;
        while(bchanged) {
            bchanged=false;
            FOREACHC(itchildren, _vsetLevelNodes) {
                FOREACHC(itchild, *itchildren) {
                    if( _setchildcache.find(*itchild) == _setchildcache.end() && _setchildcache.find((*itchild)->rrtparent) != _setchildcache.end() ) {
                        //if( !std::binary_search(_vchildcache.begin(),_vchildcache.end(),(*itchild)->rrtparent) ) {
                        _vchildcache.push_back(*itchild);
                        _setchildcache.insert(*itchild);
                        bchanged=true;
                    }
                }
            }
            ++numruns;
        }

        int nremove=0;
        // systematically remove backwards
        for(typename vector<NodePtr>::reverse_iterator itnode = _vchildcache.rbegin(); itnode != _vchildcache.rend(); ++itnode) {
            bool bremoved = _RemoveNode(*itnode);
            BOOST_ASSERT(bremoved);
            ++nremove;
        }
        BOOST_ASSERT(Validate());
        RAVELOG_VERBOSE("computed in %fs", (1e-9*(utils::GetNanoPerformanceTime()-starttime)));
    }

    virtual ExtendType Extend(const vector<dReal>& vTargetConfig, NodeBasePtr& lastnode, bool bOneStep=false)
    {
        // get the nearest neighbor
        std::pair<NodePtr, dReal> nn = _FindNearestNode(vTargetConfig);
        if( !nn.first ) {
            return ET_Failed;
        }
        NodePtr pnode = nn.first;
        lastnode = nn.first;
        bool bHasAdded = false;
        boost::shared_ptr<PlannerBase> planner(_planner);
        PlannerBase::PlannerParametersConstPtr params = planner->GetParameters();
        _vCurConfig.resize(_dof);
        std::copy(pnode->q, pnode->q+_dof, _vCurConfig.begin());
        // extend
        for(int iter = 0; iter < 100; ++iter) {     // to avoid infinite loops
            dReal fdist = _ComputeDistance(&_vCurConfig[0], vTargetConfig);
            if( fdist > _fStepLength ) {
                fdist = _fStepLength / fdist;
            }
            else if( fdist <= dReal(0.01) * _fStepLength ) {
                // return connect if the distance is very close
                return ET_Connected;
            }
            else {
                fdist = 1;
            }

            _vNewConfig = _vCurConfig;
            _vDeltaConfig = vTargetConfig;
            params->_diffstatefn(_vDeltaConfig, _vCurConfig);
            for(int i = 0; i < _dof; ++i) {
                _vDeltaConfig[i] *= fdist;
            }
            if( params->SetStateValues(_vNewConfig) != 0 ) {
                if(bHasAdded) {
                    return ET_Sucess;
                }
                return ET_Failed;
            }
            if( !params->_neighstatefn(_vNewConfig,_vDeltaConfig,_fromgoal) ) {
                if(bHasAdded) {
                    return ET_Sucess;
                }
                return ET_Failed;
            }

            // it could be the case that the node didn't move anywhere, in which case we would go into an infinite loop
            if( _ComputeDistance(&_vCurConfig[0], _vNewConfig) <= dReal(0.01)*_fStepLength ) {
                if(bHasAdded) {
                    return ET_Sucess;
                }
                return ET_Failed;
            }

            if( _fromgoal ) {
                if( params->CheckPathAllConstraints(_vNewConfig, _vCurConfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenEnd) != 0 ) {
                    return bHasAdded ? ET_Sucess : ET_Failed;
                }
            }
            else {
                if( params->CheckPathAllConstraints(_vCurConfig, _vNewConfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart) != 0 ) {
                    return bHasAdded ? ET_Sucess : ET_Failed;
                }
            }

            NodePtr pnewnode = _InsertNode(pnode, _vNewConfig, 0); ///< set userdata to 0
            if( !!pnewnode ) {
                pnode = pnewnode;
                lastnode = pnode;
                bHasAdded = true;
                if( bOneStep ) {
                    return ET_Connected; // is it ok to return ET_Connected rather than ET_Sucess. BasicRRT relies on ET_Connected
                }
            }
            _vCurConfig.swap(_vNewConfig);
        }

        return bHasAdded ? ET_Sucess : ET_Failed;
    }

    virtual int GetNumNodes() const {
        return _numnodes;
    }

    virtual const vector<dReal>& GetVectorConfig(NodeBasePtr nodebase) const
    {
        NodePtr node = (NodePtr)nodebase;
        _vTempConfig.resize(_dof);
        std::copy(node->q, node->q+_dof, _vTempConfig.begin());
        return _vTempConfig;
    }

    virtual void GetVectorConfig(NodeBasePtr nodebase, std::vector<dReal>& v) const
    {
        NodePtr node = (NodePtr)nodebase;
        v.resize(_dof);
        std::copy(node->q, node->q+_dof, v.begin());
    }

    virtual int GetDOF() {
        return _dof;
    }

    /// \brief for debug purposes, validates the tree
    virtual bool Validate() const
    {
        if( _numnodes == 0 ) {
            return _numnodes==0;
        }

        if( _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).size() != 1 ) {
            RAVELOG_WARN("more than 1 root node\n");
            return false;
        }

        dReal fLevelBound = _fMaxLevelBound;
        std::vector<NodePtr> vAccumNodes; vAccumNodes.reserve(_numnodes);
        size_t nallchildren = 0;
        size_t numnodes = 0;
        for(int currentlevel = _maxlevel; currentlevel >= _minlevel; --currentlevel, fLevelBound *= _fBaseInv ) {
            int enclevel = _EncodeLevel(currentlevel);
            if( enclevel >= (int)_vsetLevelNodes.size() ) {
                continue;
            }

            const std::set<NodePtr>& setLevelRawChildren = _vsetLevelNodes.at(enclevel);
            FOREACHC(itnode, setLevelRawChildren) {
                FOREACH(itchild, (*itnode)->_vchildren) {
                    dReal curdist = _ComputeDistance(*itnode, *itchild);
                    if( curdist > fLevelBound+g_fEpsilonLinear ) {
#ifdef _DEBUG
                        RAVELOG_WARN_FORMAT("invalid parent child nodes %d, %d at level %d (%f), dist=%f", (*itnode)->id%(*itchild)->id%currentlevel%fLevelBound%curdist);
#else
                        RAVELOG_WARN_FORMAT("invalid parent child nodes at level %d (%f), dist=%f", currentlevel%fLevelBound%curdist);
#endif
                        return false;
                    }
                }
                nallchildren += (*itnode)->_vchildren.size();
                if( !(*itnode)->_hasselfchild ) {
                    vAccumNodes.push_back(*itnode);
                }

                if( currentlevel < _maxlevel ) {
                    // find its parents
                    int nfound = 0;
                    FOREACH(ittestnode, _vsetLevelNodes.at(_EncodeLevel(currentlevel+1))) {
                        if( find((*ittestnode)->_vchildren.begin(), (*ittestnode)->_vchildren.end(), *itnode) != (*ittestnode)->_vchildren.end() ) {
                            ++nfound;
                        }
                    }
                    BOOST_ASSERT(nfound==1);
                }
            }

            numnodes += setLevelRawChildren.size();

            for(size_t i = 0; i < vAccumNodes.size(); ++i) {
                for(size_t j = i+1; j < vAccumNodes.size(); ++j) {
                    dReal curdist = _ComputeDistance(vAccumNodes[i], vAccumNodes[j]);
                    if( curdist <= fLevelBound ) {
#ifdef _DEBUG
                        RAVELOG_WARN_FORMAT("invalid sibling nodes %d, %d  at level %d (%f), dist=%f", vAccumNodes[i]->id%vAccumNodes[j]->id%currentlevel%fLevelBound%curdist);
#else
                        RAVELOG_WARN_FORMAT("invalid sibling nodes %d, %d  at level %d (%f), dist=%f", i%j%currentlevel%fLevelBound%curdist);
#endif
                        return false;
                    }
                }
            }
        }

        if( _numnodes != (int)numnodes ) {
            RAVELOG_WARN_FORMAT("num predicted nodes (%d) does not match computed nodes (%d)", _numnodes%numnodes);
            return false;
        }
        if( _numnodes != (int)nallchildren+1 ) {
            RAVELOG_WARN_FORMAT("num predicted nodes (%d) does not match computed nodes from children (%d)", _numnodes%(nallchildren+1));
            return false;
        }

        return true;
    }

    void DumpTree(std::ostream& o) const
    {
        o << _numnodes << endl;
        // first organize all nodes into a vector struct with indices
        std::vector<NodePtr> vnodes; vnodes.reserve(_numnodes);
        FOREACHC(itchildren, _vsetLevelNodes) {
            vnodes.insert(vnodes.end(), itchildren->begin(), itchildren->end());
        }
        // there's a bug here when using FOREACHC
        //FOREACHC(itnode,vnodes) {
        for(size_t inode = 0; inode < vnodes.size(); ++inode) {
            NodePtr node = vnodes[inode];
            for(int i = 0; i < _dof; ++i) {
                o << node->q[i] << ",";
            }
            typename std::vector<NodePtr>::iterator itnode = find(vnodes.begin(), vnodes.end(), node->rrtparent);
            if( itnode == vnodes.end() ) {
                o << "-1" << endl;
            }
            else {
                o << (size_t)(itnode-vnodes.begin()) << endl;
            }
        }
    }

    /// \brief given random index 0 <= inode < _numnodes, return a node. If tree changes, indices might change
    NodeBase* GetNodeFromIndex(size_t inode) const
    {
        if( (int)inode >= _numnodes ) {
            return NodePtr();
        }
        FOREACHC(itchildren, _vsetLevelNodes) {
            if( inode < itchildren->size() ) {
                typename std::set<NodePtr>::iterator itchild = itchildren->begin();
                advance(itchild, inode);
                return *itchild;
            }
            else {
                inode -= itchildren->size();
            }
        }
        return NodePtr();
    }

    void GetNodesVector(std::vector<NodeBase*>& vnodes)
    {
        vnodes.resize(0);
        if( (int)vnodes.capacity() < _numnodes ) {
            vnodes.reserve(_numnodes);
        }
        FOREACHC(itchildren, _vsetLevelNodes) {
            vnodes.insert(vnodes.end(), itchildren->begin(), itchildren->end());
        }
    }

private:
    static int GetNewStaticId() {
        static int s_id = 0;
        int retid = s_id++;
        return retid;
    }
    inline NodePtr _CreateNode(NodePtr rrtparent, const vector<dReal>& config, uint32_t userdata)
    {
        // allocate memory for the structur and the internal state vectors
        void* pmemory = _pNodesPool->malloc();
        NodePtr node = new (pmemory) Node(rrtparent, config);
        node->_userdata = userdata;
#ifdef _DEBUG
        node->id = GetNewStaticId();
#endif
        return node;
    }

    inline NodePtr _CloneNode(NodePtr refnode)
    {
        // allocate memory for the structur and the internal state vectors
        void* pmemory = _pNodesPool->malloc();
        NodePtr node = new (pmemory) Node(refnode->rrtparent, refnode->q, _dof);
        node->_userdata = refnode->_userdata;
#ifdef _DEBUG
        node->id = GetNewStaticId();
#endif
        return node;
    }

    void _DeleteNode(Node* p)
    {
        if( !!p ) {
            p->~Node();
            _pNodesPool->free(p);
        }
    }

    inline int _EncodeLevel(int level) const {
        if( level <= 0 ) {
            return -2*level;
        }
        else {
            return 2*level+1;
        }
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
                    _vNextLevelNodes.push_back(make_pair(*itchild, curdist));
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

    NodePtr _InsertNode(NodePtr parent, const vector<dReal>& config, uint32_t userdata)
    {
        NodePtr newnode = _CreateNode(parent, config, userdata);
        if( _numnodes == 0 ) {
            // no root
            _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).insert(newnode); // add to the level
            newnode->_level = _maxlevel;
            _numnodes += 1;
        }
        else {
            _vCurrentLevelNodes.resize(1);
            _vCurrentLevelNodes[0].first = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
            _vCurrentLevelNodes[0].second = _ComputeDistance(_vCurrentLevelNodes[0].first->q, config);
            int nParentFound = _InsertRecursive(newnode, _vCurrentLevelNodes, _maxlevel, _fMaxLevelBound);
            BOOST_ASSERT(nParentFound!=0);
            if( nParentFound < 0 ) {
                return NodePtr();
            }
        }
        //BOOST_ASSERT(Validate());
        return newnode;
    }

    int _InsertRecursive(NodePtr nodein, const std::vector< std::pair<NodePtr, dReal> >& vCurrentLevelNodes, int currentlevel, dReal fLevelBound)
    {
#ifdef _DEBUG
        // copy for debugging
        std::vector< std::pair<NodePtr, dReal> > vLocalLevelNodes = vCurrentLevelNodes;
#endif
        dReal closestDist=std::numeric_limits<dReal>::infinity();
        NodePtr closestNodeInRange=NULL; /// one of the nodes in vCurrentLevelNodes such that its distance to nodein is <= fLevelBound
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
                            _vNextLevelNodes.push_back(make_pair(*itchild, curdist));
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

    bool _RemoveNode(NodePtr removenode)
    {
        if( _numnodes == 0 ) {
            return false;
        }

        NodePtr proot = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
        if( _numnodes == 1 && removenode == proot ) {
            Reset();
            return true;
        }

        if( _maxlevel-_minlevel >= (int)_vvCacheNodes.size() ) {
            _vvCacheNodes.resize(_maxlevel-_minlevel+1);
        }
        FOREACH(it, _vvCacheNodes) {
            it->resize(0);
        }
        _vvCacheNodes.at(0).push_back(proot);
        bool bRemoved = _Remove(removenode, _vvCacheNodes, _maxlevel, _fMaxLevelBound);
        if( bRemoved ) {
            _DeleteNode(removenode);
        }
        if( removenode == proot ) {
            BOOST_ASSERT(_vvCacheNodes.at(0).size()==2); // instead of root, another node should have been added
            BOOST_ASSERT(_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).size()==1);
            //_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).clear();
            _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).erase(proot);
            bRemoved = true;
            _numnodes--;
        }
        return bRemoved;
    }

    bool _Remove(NodePtr removenode, std::vector< std::vector<NodePtr> >& vvCoverSetNodes, int currentlevel, dReal fLevelBound)
    {
        int enclevel = _EncodeLevel(currentlevel);
        if( enclevel >= (int)_vsetLevelNodes.size() ) {
            return false;
        }

        // build the level below
        std::set<NodePtr>& setLevelRawChildren = _vsetLevelNodes.at(enclevel);
        int coverindex = _maxlevel-(currentlevel-1);
        if( coverindex >= (int)vvCoverSetNodes.size() ) {
            vvCoverSetNodes.resize(coverindex+(_maxlevel-_minlevel)+1);
        }
        std::vector<NodePtr>& vNextLevelNodes = vvCoverSetNodes[coverindex];
        vNextLevelNodes.resize(0);

        bool bfound = false;
        FOREACH(itcurrentnode, vvCoverSetNodes.at(coverindex-1)) {
            // only take the children whose distances are within the bound
            if( setLevelRawChildren.find(*itcurrentnode) != setLevelRawChildren.end() ) {
                typename std::vector<NodePtr>::iterator itchild = (*itcurrentnode)->_vchildren.begin();
                while(itchild != (*itcurrentnode)->_vchildren.end() ) {
                    dReal curdist = _ComputeDistance(removenode, *itchild);
                    if( *itchild == removenode ) {
                        //vNextLevelNodes.resize(0);
                        vNextLevelNodes.push_back(*itchild);
                        itchild = (*itcurrentnode)->_vchildren.erase(itchild);
                        if( (*itcurrentnode)->_hasselfchild && _ComputeDistance(*itcurrentnode, *itchild) <= _mindistance) {
                            (*itcurrentnode)->_hasselfchild = 0;
                        }
                        bfound = true;
                        //break;
                    }
                    else {
                        if( curdist <= fLevelBound*_fBaseChildMult ) {
                            vNextLevelNodes.push_back(*itchild);
                        }
                        ++itchild;
                    }
                }
//                if( bfound ) {
//                    break;
//                }
            }
        }

        bool bRemoved = _Remove(removenode, vvCoverSetNodes, currentlevel-1, fLevelBound*_fBaseInv);

        if( !bRemoved && removenode->_level == currentlevel && find(vvCoverSetNodes.at(coverindex-1).begin(), vvCoverSetNodes.at(coverindex-1).end(), removenode) != vvCoverSetNodes.at(coverindex-1).end() ) {
            // for each child, find a more suitable parent
            FOREACH(itchild, removenode->_vchildren) {
                int parentlevel = currentlevel;
                dReal fParentLevelBound = fLevelBound;
                dReal closestdist=0;
                NodePtr closestNode = NULL;
                //int maxaddlevel = currentlevel-1;
                while(parentlevel <= _maxlevel  ) {
                    FOREACHC(itnode, vvCoverSetNodes.at(_maxlevel-parentlevel)) {
                        if( *itnode == removenode ) {
                            continue;
                        }
                        dReal curdist = _ComputeDistance(*itchild, *itnode);
                        if( curdist < fParentLevelBound ) {
                            if( !closestNode || curdist < closestdist ) {
                                closestdist = curdist;
                                closestNode = *itnode;
                            }
                        }
                    }
                    if( !!closestNode ) {
                        NodePtr nodechild = *itchild;
                        while( nodechild->_level < closestNode->_level-1 ) {
                            NodePtr clonenode = _CloneNode(nodechild);
                            clonenode->_level = nodechild->_level+1;
                            clonenode->_vchildren.push_back(nodechild);
                            clonenode->_hasselfchild = 1;
                            int encclonelevel = _EncodeLevel(clonenode->_level);
                            if( encclonelevel >= (int)_vsetLevelNodes.size() ) {
                                _vsetLevelNodes.resize(encclonelevel+1);
                            }
                            _vsetLevelNodes.at(encclonelevel).insert(clonenode);
                            _numnodes +=1;
                            vvCoverSetNodes.at(_maxlevel-clonenode->_level).push_back(clonenode);
                            nodechild = clonenode;
                        }

                        if( closestdist <= _mindistance ) {
                            closestNode->_hasselfchild = 1;
                        }

                        closestNode->_vchildren.push_back(nodechild);
                        break;
                    }

                    // try a higher level
                    parentlevel += 1;
                    fParentLevelBound *= _base;
                }
                if( !closestNode ) {
                    BOOST_ASSERT(parentlevel>_maxlevel);
                    // occurs when root node is being removed and new children have no where to go?
                    _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).insert(*itchild);
                    vvCoverSetNodes.at(0).push_back(*itchild);
                }
            }
            // remove the node
            size_t erased = setLevelRawChildren.erase(removenode);
            BOOST_ASSERT(erased==1);
            bRemoved = true;
            _numnodes--;
        }
        return bRemoved;
    }


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

    mutable std::vector< std::pair<NodePtr, dReal> > _vCurrentLevelNodes, _vNextLevelNodes;
    mutable std::vector< std::vector<NodePtr> > _vvCacheNodes;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(SimpleNode)
BOOST_TYPEOF_REGISTER_TYPE(SpatialTree)
#endif

#endif
