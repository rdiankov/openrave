// -*- Coding: utf-8 -*-
// Copyright (C) 2014 Alejandro Perez & Rosen Diankov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/// \author Alejandro Perez & Rosen Diankov
#include "configurationcachetree.h"
#include <sstream>
#include <boost/lexical_cast.hpp>

#include <boost/multi_array.hpp>
#include <algorithm>

using boost::multi_array;
using boost::extents;

namespace configurationcache {

inline dReal Sqr(dReal x) {
    return x*x;
}

CacheTreeNode::CacheTreeNode(const std::vector<dReal>& cs, Vector* plinkspheres)
{
    std::copy(cs.begin(), cs.end(), _pcstate);
    _plinkspheres = plinkspheres;
//    _approxdispersion.first = CacheTreeNodePtr();
//    _approxdispersion.second = std::numeric_limits<float>::infinity();
//    _approxnn.first = CacheTreeNodePtr();
//    _approxnn.second = std::numeric_limits<float>::infinity();
    _conftype = CNT_Unknown;
    _robotlinkindex = -1;
    _level = 0;
    _hasselfchild = 0;
    _usenn = 1;
    _hitcount = 0;
}

CacheTreeNode::CacheTreeNode(const dReal* pstate, int dof, Vector* plinkspheres)
{
    std::copy(pstate, pstate+dof, _pcstate);
    _plinkspheres = plinkspheres;
    _conftype = CNT_Unknown;
    _robotlinkindex = -1;
    _level = 0;
    _hasselfchild = 0;
    _usenn = 1;
    _hitcount = 0;
}

void CacheTreeNode::SetCollisionInfo(CollisionReportPtr report)
{
    if( !!report ) {
        _collidinglinktrans = report->plink1->GetTransform();
        _robotlinkindex = report->plink1->GetIndex();
        _collidinglink = report->plink2;
        _conftype = CNT_Collision;
    }
    else {
        _conftype = CNT_Free;
        _collidinglink.reset();
        _robotlinkindex = 0;
    }
}

void CacheTreeNode::SetCollisionInfo(int robotlinkindex, int type)
{
    _robotlinkindex = robotlinkindex;
    if(type == 1) {
        _conftype = CNT_Collision;
    }

    if(type == 2) {
        _conftype = CNT_Free;
    }
}

//void CacheTreeNode::UpdateApproximates(dReal distance, CacheTreeNodePtr v)
//{
//    // if both are same type, update nn
//    if (v->GetType() == _conftype) {
//        if (distance < _approxnn.second) {
//            _approxnn.first = v;
//            _approxnn.second = distance;
//        }
//
//    }
//    // if they are different types, update dispersion
//    else{
//        if ( distance < _approxdispersion.second) {
//            _approxdispersion.first = v;
//            //dReal oldd = _approxdispersion.second;
//            //RAVELOG_DEBUG_FORMAT("dispersion %f to %f\n",oldd%distance);
//            _approxdispersion.second = distance;
//        }
//    }
//}

CacheTree::CacheTree(int statedof)
{
    _poolNodes.reset(new boost::pool<>(sizeof(CacheTreeNode)+sizeof(dReal)*statedof));
    _vnodes.resize(0);
    _dummycs.resize(0);
    _fulldirname.resize(0);
    _mapNodeIndices.clear();
    _collidingbodyname.resize(0);

    _statedof=statedof;
    _weights.resize(_statedof, 1.0);
    Init(_weights, 1);
}

CacheTree::~CacheTree()
{
    Reset();
    _weights.clear();
}

void CacheTree::Init(const std::vector<dReal>& weights, dReal maxdistance)
{
    Reset();
    _weights = weights;
    _statedof = (int)_weights.size();
    _numnodes = 0;
    _base = 2.0;
    _fBaseInv = 1/_base;
    _fBaseInv2 = 1/Sqr(_base);
    _fBaseChildMult = 1/(_base-1);
    _maxdistance = maxdistance;
    _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
    _minlevel = _maxlevel - 1;
    _fMaxLevelBound = RavePow(_base, _maxlevel);
    int enclevel = _EncodeLevel(_maxlevel);
    if( enclevel >= (int)_vsetLevelNodes.size() ) {
        _vsetLevelNodes.resize(enclevel+1);
    }
}

void CacheTree::Reset()
{

    _vnodes.resize(0);
    _dummycs.resize(0);
    _fulldirname.resize(0);
    _mapNodeIndices.clear();
    _collidingbodyname.resize(0);

    // make sure all children are deleted
    for(size_t ilevel = 0; ilevel < _vsetLevelNodes.size(); ++ilevel) {
        FOREACH(itnode, _vsetLevelNodes[ilevel]) {
            (*itnode)->~CacheTreeNode();
        }
    }
    FOREACH(itchildren, _vsetLevelNodes) {
        itchildren->clear();
    }
    FOREACH(itnode, _vnodes) {
        (*itnode)->~CacheTreeNode();
    }
    // purge_memory leaks!
    //_poolNodes.purge_memory();
    _poolNodes.reset(new boost::pool<>(sizeof(CacheTreeNode)+sizeof(dReal)*_statedof));
    //_pNodesPool.reset(new boost::pool<>(sizeof(Node)+_dof*sizeof(dReal)));
    _numnodes = 0;
}

#ifdef _DEBUG
static int s_CacheTreeId = 0;
#endif

CacheTreeNodePtr CacheTree::_CreateCacheTreeNode(const std::vector<dReal>& cs, CollisionReportPtr report)
{
    // allocate memory for the structure and the internal state vectors
    void* pmemory;
    {
        //boost::mutex::scoped_lock lock(_mutexpool);
        pmemory = _poolNodes->malloc();
    }
    //Vector* plinkspheres = (Vector*)((uint8_t*)pmemory + sizeof(CacheTreeNode) + sizeof(dReal)*_statedof);
    CacheTreeNodePtr newnode = new (pmemory) CacheTreeNode(cs, NULL);
#ifdef _DEBUG
    newnode->id = s_CacheTreeId++;
#endif
    newnode->SetCollisionInfo(report);
    return newnode;
}

CacheTreeNodePtr CacheTree::_CloneCacheTreeNode(CacheTreeNodeConstPtr refnode)
{
    // allocate memory for the structure and the internal state vectors
    void* pmemory;
    {
        //boost::mutex::scoped_lock lock(_mutexpool);
        pmemory = _poolNodes->malloc();
    }
    //Vector* plinkspheres = (Vector*)((uint8_t*)pmemory + sizeof(CacheTreeNode) + sizeof(dReal)*_statedof);
    CacheTreeNodePtr clonenode = new (pmemory) CacheTreeNode(refnode->GetConfigurationState(), _statedof, refnode->_plinkspheres);
#ifdef _DEBUG
    clonenode->id = s_CacheTreeId++;
#endif
    clonenode->_conftype = refnode->_conftype;
    clonenode->_hitcount = refnode->_hitcount;
    if( clonenode->IsInCollision() ) {
        clonenode->_collidinglink = refnode->_collidinglink;
        clonenode->_collidinglinktrans = refnode->_collidinglinktrans;
        clonenode->_robotlinkindex = refnode->_robotlinkindex;
    }

    return clonenode;
}

void CacheTree::_DeleteCacheTreeNode(CacheTreeNodePtr pnode)
{
    pnode->~CacheTreeNode();
    _poolNodes->free(pnode);
}

dReal CacheTree::ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef) const
{
    return RaveSqrt(_ComputeDistance2(&cstatei[0], &cstatef[0]));
}

dReal CacheTree::_ComputeDistance2(const dReal* cstatei, const dReal* cstatef) const
{
    dReal distance = 0;
    for (size_t i = 0; i < _weights.size(); ++i) {
        dReal f = (cstatei[i] - cstatef[i]) * _weights[i];
        distance += f*f;
    }
    return distance;
}

void CacheTree::SetWeights(const std::vector<dReal>& weights)
{
    Reset();
    _weights = weights;
}

void CacheTree::SetMaxDistance(dReal maxdistance)
{
    Reset();
    _maxdistance = maxdistance;
    _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
    _minlevel = _maxlevel - 1;
    _fMaxLevelBound = RavePow(_base, _maxlevel);
    int enclevel = _EncodeLevel(_maxlevel);
    if( enclevel >= (int)_vsetLevelNodes.size() ) {
        _vsetLevelNodes.resize(enclevel+1);
    }
}

void CacheTree::SetBase(dReal base)
{
    Reset();
    _statedof = (int)_weights.size();
    _base = base;
    _fBaseInv = 1/_base;
    _fBaseInv2 = 1/Sqr(_base);
    _fBaseChildMult = 1/(_base-1);
    _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
    _minlevel = _maxlevel - 1;
    _fMaxLevelBound = RavePow(_base, _maxlevel);
    int enclevel = _EncodeLevel(_maxlevel);
    if( enclevel >= (int)_vsetLevelNodes.size() ) {
        _vsetLevelNodes.resize(enclevel+1);
    }
}

std::pair<CacheTreeNodeConstPtr, dReal> CacheTree::FindNearestNode(const std::vector<dReal>& vquerystate, dReal distancebound, ConfigurationNodeType conftype) const
{
    if( _numnodes == 0 ) {
        return make_pair(CacheTreeNodeConstPtr(), dReal(0));
    }

    CacheTreeNodeConstPtr pbestnode=NULL;
    dReal bestdist2 = std::numeric_limits<dReal>::infinity();
    OPENRAVE_ASSERT_OP(vquerystate.size(),==,_weights.size());
    const dReal* pquerystate = &vquerystate[0];

    dReal distancebound2 = Sqr(distancebound);
    int currentlevel = _maxlevel; // where the root node is
    // traverse all levels gathering up the children at each level
    dReal fLevelBound2 = Sqr(_fMaxLevelBound);
    _vCurrentLevelNodes.resize(1);
    _vCurrentLevelNodes[0].first = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
    _vCurrentLevelNodes[0].second = _ComputeDistance2(pquerystate, _vCurrentLevelNodes[0].first->GetConfigurationState());
    if( (conftype == CNT_Any || _vCurrentLevelNodes[0].first->GetType() == conftype) && _vCurrentLevelNodes[0].first->_usenn ) {
        pbestnode = _vCurrentLevelNodes[0].first;
        bestdist2 = _vCurrentLevelNodes[0].second;
    }
    while(_vCurrentLevelNodes.size() > 0 ) {
        _vNextLevelNodes.resize(0);
        dReal minchilddist2 = std::numeric_limits<dReal>::infinity();
        FOREACH(itcurrentnode, _vCurrentLevelNodes) {
            // only take the children whose distances are within the bound
            FOREACHC(itchild, itcurrentnode->first->_vchildren) {
                dReal curdist2 = _ComputeDistance2(pquerystate, (*itchild)->GetConfigurationState());
                if( curdist2 < bestdist2 ) {
                    if( (*itchild)->_usenn && (conftype == CNT_Any || (*itchild)->GetType() == conftype) ) {
                        bestdist2 = curdist2;
                        pbestnode = *itchild;
                        if( distancebound > 0 && bestdist2 <= distancebound2 ) {
                            (*itchild)->IncreaseHitCount();
                            return make_pair(pbestnode, RaveSqrt(bestdist2));
                        }
                    }
                }
                _vNextLevelNodes.emplace_back(*itchild,  curdist2);
                if( minchilddist2 > curdist2 ) {
                    minchilddist2 = curdist2;
                }
            }
        }

        _vCurrentLevelNodes.resize(0);
        // have to compute dist < RaveSqrt(minchilddist2) + fLevelBound
        // dist2 < m2 + 2mL + L2

        dReal ftestbound2 = 4*minchilddist2*fLevelBound2;
        FOREACH(itnode, _vNextLevelNodes) {
            dReal f = itnode->second - minchilddist2 - fLevelBound2;
            if( f <= 0 || Sqr(f) <= ftestbound2 ) {
                _vCurrentLevelNodes.push_back(*itnode);
            }
        }
        currentlevel -= 1;
        fLevelBound2 *= _fBaseInv2;
    }
    if( !!pbestnode && (distancebound2 <= 0 || bestdist2 <= distancebound2) ) {
        return make_pair(pbestnode, RaveSqrt(bestdist2));
    }
    // failed radius search, so should return empty
    return make_pair(CacheTreeNodeConstPtr(), dReal(0));
}

std::pair<CacheTreeNodeConstPtr, dReal> CacheTree::FindNearestNode(const std::vector<dReal>& vquerystate, dReal collisionthresh, dReal freespacethresh) const
{
    std::pair<CacheTreeNodeConstPtr, dReal> bestnode;
    bestnode.first = NULL;
    bestnode.second = std::numeric_limits<dReal>::infinity();
    if( _numnodes == 0 ) {
        return bestnode;
    }

    OPENRAVE_ASSERT_OP(vquerystate.size(),==,_weights.size());
    // first localmax is distance from this node to the root
    const dReal* pquerystate = &vquerystate[0];

    dReal collisionthresh2 = Sqr(collisionthresh), freespacethresh2 = Sqr(freespacethresh);
    // traverse all levels gathering up the children at each level
    int currentlevel = _maxlevel; // where the root node is
    dReal fLevelBound = _fMaxLevelBound;
    {
        CacheTreeNodePtr proot = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
        dReal curdist2 = _ComputeDistance2(pquerystate, proot->GetConfigurationState());
        if( proot->_usenn ) {
            ConfigurationNodeType cntype = proot->GetType();
            if( cntype == CNT_Collision && curdist2 <= collisionthresh2 ) {
                proot->_hitcount++;
                return make_pair(proot,RaveSqrt(curdist2));
            }
            else if( cntype == CNT_Free && curdist2 <= freespacethresh2 ) {
                // there still could be a node lower in the hierarchy whose collision is closer...
                bestnode = make_pair(proot,RaveSqrt(curdist2));
            }
        }
        _vCurrentLevelNodes.resize(1);
        _vCurrentLevelNodes[0].first = proot;
        _vCurrentLevelNodes[0].second = curdist2;
    }
    dReal pruneradius2 = Sqr(_maxdistance); // the radius to prune all _vCurrentLevelNodes when going through them. Equivalent to min(query,children) + levelbound from the previous iteration
    while(_vCurrentLevelNodes.size() > 0 ) {
        _vNextLevelNodes.resize(0);
        dReal minchilddist=_maxdistance;
        FOREACH(itcurrentnode, _vCurrentLevelNodes) {
            if( itcurrentnode->second > pruneradius2 ) {
                continue;
            }
            dReal comparedist2 = Sqr(minchilddist + fLevelBound);
            // only take the children whose distances are within the bound
            FOREACHC(itchild, itcurrentnode->first->_vchildren) {
                dReal curdist2 = _ComputeDistance2(pquerystate, (*itchild)->GetConfigurationState());
                if( (*itchild)->_usenn ) {
                    ConfigurationNodeType cntype = (*itchild)->GetType();
                    if( cntype == CNT_Collision && curdist2 <= collisionthresh2 ) {
                        (*itchild)->_hitcount++;
                        return make_pair(*itchild, RaveSqrt(curdist2));
                    }
                    else if( cntype == CNT_Free && curdist2 <= freespacethresh2 ) {
                        // there still could be a node lower in the hierarchy whose collision is closer...
                        if( curdist2 < bestnode.second ) {
                            bestnode = make_pair(*itchild, curdist2);
                        }
                    }
                }
                if( curdist2 < comparedist2 ) {
                    _vNextLevelNodes.emplace_back(*itchild,  curdist2);
                    if( Sqr(minchilddist) > curdist2 ) {
                        minchilddist = RaveSqrt(curdist2);
                        comparedist2 = Sqr(minchilddist + fLevelBound);
                    }
                }
            }
        }

        _vCurrentLevelNodes.swap(_vNextLevelNodes);
        pruneradius2 = Sqr(minchilddist + fLevelBound);
        currentlevel -= 1;
        fLevelBound *= _fBaseInv;
    }
    // if here, then either found a free node within the bounds, or could not find any nodes
    // failed radius search, so should return empty
    if( !!bestnode.first ) {
        bestnode.second = RaveSqrt(bestnode.second);
    }
    return bestnode;
}

int CacheTree::InsertNode(const std::vector<dReal>& cs, CollisionReportPtr report, dReal fMinSeparationDist)
{

    OPENRAVE_ASSERT_OP(cs.size(),==,_weights.size());
    CacheTreeNodePtr nodein = _CreateCacheTreeNode(cs, report);
    // if there is no root, make this the root, otherwise call the lowlevel  insert
    if( _numnodes == 0 ) {
        // no root
        _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).insert(nodein); // add to the level
        _numnodes += 1;
        nodein->_level = _maxlevel;
        return 1;
    }

    _vCurrentLevelNodes.resize(1);
    _vCurrentLevelNodes[0].first = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
    _vCurrentLevelNodes[0].second = _ComputeDistance2(_vCurrentLevelNodes[0].first->GetConfigurationState(), &cs[0]);
    int nParentFound = _Insert(nodein, _vCurrentLevelNodes, _maxlevel, Sqr(_fMaxLevelBound), Sqr(fMinSeparationDist));
    if( nParentFound != 1 ) {
        _DeleteCacheTreeNode(nodein);
    }
    return nParentFound;
}

int CacheTree::_Insert(CacheTreeNodePtr nodein, const std::vector< std::pair<CacheTreeNodePtr, dReal> >& vCurrentLevelNodes, int currentlevel, dReal fLevelBound2, dReal fMinSeparationDist2)
{
#ifdef _DEBUG
    // copy for debugging
    std::vector< std::pair<CacheTreeNodePtr, dReal> > vLocalLevelNodes = vCurrentLevelNodes;
#endif

    dReal closestDist=0;
    CacheTreeNodePtr closestNodeInRange=NULL; /// one of the nodes in vCurrentLevelNodes such that its distance to nodein is <= fLevelBound
    int enclevel = _EncodeLevel(currentlevel);
    dReal fChildLevelBound2 = fLevelBound2*Sqr(_fBaseChildMult);
    dReal fEpsilon = g_fEpsilon*_maxdistance; // min distance
    if( enclevel < (int)_vsetLevelNodes.size() ) {
        // build the level below
        _vNextLevelNodes.resize(0);
        FOREACHC(itcurrentnode, vCurrentLevelNodes) {
            if( itcurrentnode->second <= fLevelBound2 ) {
                if( !closestNodeInRange ) {
                    closestNodeInRange = itcurrentnode->first;
                    closestDist = itcurrentnode->second;
                }
                else {
                    if(  itcurrentnode->second < closestDist-fEpsilon ) {
                        closestNodeInRange = itcurrentnode->first;
                        closestDist = itcurrentnode->second;
                    }
                    // if distances are close, get the node on the lowest level...
                    else if( itcurrentnode->second <= closestDist+fEpsilon && itcurrentnode->first->_level < closestNodeInRange->_level ) {
                        closestNodeInRange = itcurrentnode->first;
                        closestDist = itcurrentnode->second;
                    }
                }
                if( (closestDist < fMinSeparationDist2)  ) {
                    // pretty close, so return as if node was added
                    return -1;
                }
            }
            if( itcurrentnode->second <= fChildLevelBound2 ) {
                // node is part of all sets below its level
                _vNextLevelNodes.push_back(*itcurrentnode);
            }
            // only take the children whose distances are within the bound
            if( itcurrentnode->first->_level == currentlevel ) {
                FOREACHC(itchild, itcurrentnode->first->_vchildren) {
                    dReal curdist = _ComputeDistance2(nodein->GetConfigurationState(), (*itchild)->GetConfigurationState());
                    if( curdist <= fChildLevelBound2 ) {
                        _vNextLevelNodes.emplace_back(*itchild,  curdist);
                    }
                }
            }
        }

        if( _vNextLevelNodes.size() > 0 ) {
            _vCurrentLevelNodes.swap(_vNextLevelNodes); // invalidates vCurrentLevelNodes
            // note that after _Insert call, _vCurrentLevelNodes could be complete lost/reset
            int nParentFound = _Insert(nodein, _vCurrentLevelNodes, currentlevel-1, fLevelBound2*_fBaseInv2, fMinSeparationDist2);
            if( nParentFound != 0 ) {
                return nParentFound;
            }
        }
    }
    else {
        FOREACHC(itcurrentnode, vCurrentLevelNodes) {
            if( itcurrentnode->second <= fLevelBound2 ) {
                if( !closestNodeInRange ) {
                    closestNodeInRange = itcurrentnode->first;
                    closestDist = itcurrentnode->second;
                }
                else {
                    if(  itcurrentnode->second < closestDist-fEpsilon ) {
                        closestNodeInRange = itcurrentnode->first;
                        closestDist = itcurrentnode->second;
                    }
                    // if distances are close, get the node on the lowest level...
                    else if( itcurrentnode->second < closestDist+fEpsilon && itcurrentnode->first->_level < closestNodeInRange->_level ) {
                        closestNodeInRange = itcurrentnode->first;
                        closestDist = itcurrentnode->second;
                    }
                }
                if( (closestDist < fMinSeparationDist2 ) ) {
                    // pretty close, so return as if node was added
                    return -1;
                }
            }
        }
    }

    if( !closestNodeInRange ) {
        return 0;
    }

    _InsertDirectly(nodein, closestNodeInRange, closestDist, currentlevel-1, fLevelBound2*_fBaseInv2);
    _numnodes += 1;

    return 1;
}

bool CacheTree::_InsertDirectly(CacheTreeNodePtr nodein, CacheTreeNodePtr parentnode, dReal parentdist, int maxinsertlevel, dReal fInsertLevelBound2)
{
    int insertlevel = maxinsertlevel;
    dReal fEpsilon = g_fEpsilon*_maxdistance; // min distance
    if( parentdist <= fEpsilon ) {
        // pretty close, so notify parent that there's a similar child already underneath it
        if( parentnode->_hasselfchild ) {
            // already has a similar child, so go one level below...?
            FOREACH(itchild, parentnode->_vchildren) {
                dReal childdist = _ComputeDistance2(nodein->GetConfigurationState(), (*itchild)->GetConfigurationState());
                if( childdist <= fEpsilon ) {
                    return _InsertDirectly(nodein, *itchild, childdist, maxinsertlevel-1, fInsertLevelBound2*_fBaseInv2);
                }
            }
            RAVELOG_WARN("inconsistent node found\n");
            return false;
        }
    }
    else {
        // depending on parentdist, might have to insert at a lower level in order to keep the sibling invariant
        dReal fChildLevelBound2 = fInsertLevelBound2;
        while(parentdist < fChildLevelBound2) {
            fChildLevelBound2 *= _fBaseInv2;
            insertlevel--;
        }
    }

    // have to add at insertlevel. If currentNodeInRange->_level is > insertlevel+1, will have to clone it. note that it will still represent the same RRT node with same rrtparent
    while( parentnode->_level > insertlevel+1 ) {
        CacheTreeNodePtr clonenode = _CloneCacheTreeNode(parentnode);
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

    if( parentdist <= fEpsilon ) {
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

bool CacheTree::RemoveNode(CacheTreeNodeConstPtr _removenode)
{
    if( _numnodes == 0 ) {
        return false;
    }

    CacheTreeNodePtr removenode = const_cast<CacheTreeNodePtr>(_removenode);

    CacheTreeNodePtr proot = *_vsetLevelNodes.at(_EncodeLevel(_maxlevel)).begin();
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
    bool bRemoved = _Remove(removenode, _vvCacheNodes, _maxlevel, Sqr(_fMaxLevelBound));
    if( bRemoved ) {
        _DeleteCacheTreeNode(removenode);
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

bool CacheTree::_Remove(CacheTreeNodePtr removenode, std::vector< std::vector<CacheTreeNodePtr> >& vvCoverSetNodes, int currentlevel, dReal fLevelBound2)
{
    int enclevel = _EncodeLevel(currentlevel);
    if( enclevel >= (int)_vsetLevelNodes.size() ) {
        return false;
    }

    // build the level below
    std::set<CacheTreeNodePtr>& setLevelRawChildren = _vsetLevelNodes.at(enclevel);
    int coverindex = _maxlevel-(currentlevel-1);
    if( coverindex >= (int)vvCoverSetNodes.size() ) {
        vvCoverSetNodes.resize(coverindex+(_maxlevel-_minlevel)+1);
    }
    std::vector<CacheTreeNodePtr>& vNextLevelNodes = vvCoverSetNodes[coverindex];
    vNextLevelNodes.resize(0);

    bool bfound = false;
    FOREACH(itcurrentnode, vvCoverSetNodes.at(coverindex-1)) {
        // only take the children whose distances are within the bound
        if( setLevelRawChildren.find(*itcurrentnode) != setLevelRawChildren.end() ) {
            std::vector<CacheTreeNodePtr>::iterator itchild = (*itcurrentnode)->_vchildren.begin();
            while(itchild != (*itcurrentnode)->_vchildren.end() ) {
                dReal curdist = _ComputeDistance2(removenode->GetConfigurationState(), (*itchild)->GetConfigurationState());
                if( *itchild == removenode ) {
                    vNextLevelNodes.resize(0);
                    vNextLevelNodes.push_back(*itchild);
                    itchild = (*itcurrentnode)->_vchildren.erase(itchild);
                    bfound = true;
                }
                else {
                    if( curdist <= fLevelBound2 ) {
                        vNextLevelNodes.push_back(*itchild);
                    }
                    ++itchild;
                }
            }
        }
    }

    bool bRemoved = _Remove(removenode, vvCoverSetNodes, currentlevel-1, fLevelBound2*_fBaseInv2);

    if( !bRemoved && removenode->_level == currentlevel && find(vvCoverSetNodes.at(coverindex-1).begin(), vvCoverSetNodes.at(coverindex-1).end(), removenode) != vvCoverSetNodes.at(coverindex-1).end() ) {
        dReal fEpsilon = g_fEpsilon*_maxdistance; // min distance
        // for each child, find a more suitable parent
        FOREACH(itchild, removenode->_vchildren) {
            int parentlevel = currentlevel;
            dReal fParentLevelBound2 = fLevelBound2;
            dReal closestdist=0;
            CacheTreeNodePtr closestNode = NULL;
            while(parentlevel <= _maxlevel  ) {
                FOREACHC(itnode, vvCoverSetNodes.at(_maxlevel-parentlevel)) {
                    if( *itnode == removenode ) {
                        continue;
                    }
                    dReal curdist = _ComputeDistance2((*itchild)->GetConfigurationState(), (*itnode)->GetConfigurationState());
                    if( curdist < fParentLevelBound2 ) {
                        if( !closestNode || curdist < closestdist ) {
                            closestdist = curdist;
                            closestNode = *itnode;
                        }
                    }
                }
                if( !!closestNode ) {
                    CacheTreeNodePtr nodechild = *itchild;
                    while( nodechild->_level < closestNode->_level-1 ) {
                        CacheTreeNodePtr clonenode = _CloneCacheTreeNode(nodechild);
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

                    if( closestdist <= fEpsilon ) {
                        closestNode->_hasselfchild = 1;
                    }

                    //_vsetLevelNodes.at(enclevel2).insert(nodechild);
                    closestNode->_vchildren.push_back(nodechild);

                    // closest node was found in parentlevel, so add to the children
                    break;
                }

                // try a higher level
                parentlevel += 1;
                fParentLevelBound2 *= Sqr(_base);
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

void CacheTree::GetNodeValues(std::vector<dReal>& vals) const
{
    vals.resize(0);
    if( (int)vals.capacity() < _numnodes*_statedof) {
        vals.reserve(_numnodes*_statedof);
    }
    FOREACH(itlevelnodes, _vsetLevelNodes) {
        FOREACH(itnode, *itlevelnodes) {
            vals.insert(vals.end(), (*itnode)->GetConfigurationState(), (*itnode)->GetConfigurationState()+_statedof);
        }
    }
}

void CacheTree::GetNodeValuesList(std::vector<CacheTreeNodePtr>& lvals)
{
    lvals.resize(0);
    if (_numnodes > 0) {
        FOREACH(itlevelnodes, _vsetLevelNodes) {
            lvals.insert(lvals.end(), itlevelnodes->begin(), itlevelnodes->end());
        }
    }
}
int CacheTree::RemoveCollisionConfigurations()
{

    int nremoved=0;
    if (_numnodes > 0) {
        FOREACH(itlevelnodes, _vsetLevelNodes) {
            FOREACH(itnode, *itlevelnodes) {
                (*itnode)->SetType(CNT_Unknown);
                nremoved += 1;
            }
        }
    }
    return nremoved;
}

int CacheTree::SaveCache(std::string filename)
{
    //boost::mutex::scoped_lock lock(_mutexpool);
    _mapNodeIndices.clear();
    int index=0;
    int knownnodes=0;
    FOREACH(itlevelnodes, _vsetLevelNodes) {
        FOREACH(itnode, *itlevelnodes) {
            if( (*itnode)->_conftype != CNT_Unknown) {
                knownnodes++;
            }
            _mapNodeIndices[*itnode] = index++;
        }
    }

    _numnodes = knownnodes;
    _fulldirname = RaveFindDatabaseFile(std::string("selfcache.")+filename,false);

    RAVELOG_DEBUG_FORMAT("Writing cache to %s, size=%d", _fulldirname%_numnodes);
    
    FILE* pfile;
    pfile = fopen(_fulldirname.c_str(),"wb");

    fwrite(&_statedof, sizeof(_statedof), 1, pfile);
    fwrite(&_weights[0], sizeof(_weights[0])*_weights.size(), 1, pfile);
    fwrite(&_base, sizeof(_base), 1, pfile);
    fwrite(&_fBaseInv, sizeof(_fBaseInv), 1, pfile);
    fwrite(&_fBaseInv2, sizeof(_fBaseInv2), 1, pfile);
    fwrite(&_fBaseChildMult, sizeof(_fBaseChildMult), 1, pfile);
    fwrite(&_maxdistance, sizeof(_maxdistance), 1, pfile);
    fwrite(&_maxlevel, sizeof(_maxlevel), 1, pfile);
    fwrite(&_minlevel, sizeof(_minlevel), 1, pfile);
    fwrite(&_numnodes, sizeof(_numnodes), 1, pfile);
    fwrite(&_fMaxLevelBound, sizeof(_fMaxLevelBound), 1, pfile);

    FOREACH(itlevelnodes, _vsetLevelNodes) {
        FOREACH(itnode, *itlevelnodes) {
            _newnode = *itnode;
            // TODO save only configuration != CNT_Unknown without losing the validity of the tree
            if (true) { //_newnode->_conftype != CNT_Unknown) {
                fwrite(&_newnode->_level, sizeof(_newnode->_level), 1, pfile);
                fwrite(_newnode->GetConfigurationState(), sizeof(dReal)*_statedof, 1, pfile);
                fwrite(&_newnode->_conftype, sizeof(_newnode->_conftype), 1, pfile);

                if (_newnode->_conftype == CNT_Collision ) {
                    // note, this assumes the colliding body name never changes across environments, which is a false assumption
                    _collidingbodyname = _newnode->GetCollidingLink()->GetParent()->GetName();
                    int namelength = _collidingbodyname.size();
                    fwrite(&namelength, sizeof(namelength), 1, pfile);
                    fwrite(_collidingbodyname.c_str(), _collidingbodyname.size(), 1, pfile);
                    int linkindex = _newnode->GetCollidingLink()->GetIndex();
                    fwrite(&linkindex, sizeof(linkindex), 1, pfile);
                    fwrite(&_newnode->_robotlinkindex, sizeof(_newnode->_robotlinkindex), 1, pfile);
                }

                fwrite(&_newnode->_hasselfchild, sizeof(_newnode->_hasselfchild), 1, pfile);
                fwrite(&_newnode->_usenn, sizeof(_newnode->_usenn), 1, pfile);
                int numchildren = _newnode->_vchildren.size();
                fwrite(&numchildren, sizeof(numchildren), 1, pfile);

                FOREACHC(itchild, (*itnode)->_vchildren) {
                    int cindex = _mapNodeIndices[*itchild];
                    fwrite(&cindex, sizeof(cindex), 1, pfile);
                }
            }
        }
    }

    fclose(pfile);

    return 1;
}

int CacheTree::LoadCache(std::string filename, EnvironmentBasePtr penv)
{
    //boost::mutex::scoped_lock lock(_mutexpool);
    _fulldirname = RaveFindDatabaseFile(std::string("selfcache.")+filename,false);

    FILE* pfile = fopen(_fulldirname.c_str(),"rb");
    size_t outs;

    if (!pfile) {
        return 0;
    }

    Reset();
    outs = fread(&_statedof, sizeof(_statedof), 1, pfile);

    _weights.resize(_statedof,1.0);
    _curconf.resize(_statedof,1.0);
    outs = fread(&_weights[0], sizeof(_weights[0])*_weights.size(), 1, pfile);

    outs = fread(&_base, sizeof(_base), 1, pfile);
    outs = fread(&_fBaseInv, sizeof(_fBaseInv), 1, pfile);
    outs = fread(&_fBaseInv2, sizeof(_fBaseInv2), 1, pfile);
    outs = fread(&_fBaseChildMult, sizeof(_fBaseChildMult), 1, pfile);
    outs = fread(&_maxdistance, sizeof(_maxdistance), 1, pfile);
    outs = fread(&_maxlevel, sizeof(_maxlevel), 1, pfile);
    outs = fread(&_minlevel, sizeof(_minlevel), 1, pfile);
    outs = fread(&_numnodes, sizeof(_numnodes), 1, pfile);
    outs = fread(&_fMaxLevelBound, sizeof(_fMaxLevelBound), 1, pfile);

    int maxenclevel = max(_EncodeLevel(_maxlevel), _EncodeLevel(_minlevel));
    _vsetLevelNodes.resize(maxenclevel+1);

    _vnodes.resize(_numnodes);
    _dummycs.resize(_statedof, 0);

    for(int i = 0; i < _numnodes; ++i) {
        _vnodes[i] = _CreateCacheTreeNode(_dummycs, CollisionReportPtr());
    }

    for (int inode = 0; inode < _numnodes; ++inode)
    {
        _newnode = _vnodes.at(inode);
        outs = fread(&_newnode->_level, sizeof(_newnode->_level), 1, pfile);
        outs = fread(_newnode->_pcstate, sizeof(dReal)*_statedof, 1, pfile);
        outs = fread(&_newnode->_conftype, sizeof(_newnode->_conftype), 1, pfile);

        if( _newnode->_conftype == CNT_Collision ) {
            int namelength;
            outs = fread(&namelength, sizeof(namelength), 1, pfile);
            _collidingbodyname.resize(namelength);
            outs = fread(&_collidingbodyname[0], namelength, 1, pfile);
            int collidinglinkindex;
            outs = fread(&collidinglinkindex, sizeof(collidinglinkindex), 1, pfile);
            _pcollidingbody = penv->GetKinBody(_collidingbodyname);
            if( !_pcollidingbody ) {
                RAVELOG_WARN_FORMAT("loading cache expected colliding body %s, but none found", _collidingbodyname);
            }
            else {
                _newnode->_collidinglink = _pcollidingbody->GetLinks().at(collidinglinkindex);
            }
            outs = fread(&_newnode->_robotlinkindex, sizeof(_newnode->_robotlinkindex), 1, pfile);
        }

        outs = fread(&_newnode->_hasselfchild, sizeof(_newnode->_hasselfchild), 1, pfile);
        outs = fread(&_newnode->_usenn, sizeof(_newnode->_usenn), 1, pfile);
        int numchildren;
        outs = fread(&numchildren, sizeof(numchildren), 1, pfile);
        _newnode->_vchildren.resize(numchildren);

        // create the copies of the children and insert them
        for(int i = 0; i < numchildren; ++i) {
            int childid;
            outs = fread(&childid, sizeof(childid), 1, pfile);
            _newnode->_vchildren[i] = _vnodes.at(childid);
        }

        _vsetLevelNodes.at(_EncodeLevel(_newnode->_level)).insert(_newnode);
    }

    fclose(pfile);

    _vnodes.resize(0);

    int numloaded = GetNumKnownNodes();
    OPENRAVE_ASSERT_OP(numloaded,==,_numnodes);
    return 1;
}

int CacheTree::UpdateCollisionConfigurations(KinBodyPtr pbody)
{
    int nremoved=0;
    if (_numnodes > 0) {
        FOREACH(itlevelnodes, _vsetLevelNodes) {
            FOREACH(itnode, *itlevelnodes) {
                _newnode = *itnode;
                if ((_newnode->GetType() == CNT_Collision) && (pbody == _newnode->GetCollidingLink()->GetParent())) {
                    _newnode->SetType(CNT_Unknown);
                    nremoved += 1;
                }
            }
        }
        int knum = GetNumKnownNodes();
        RAVELOG_VERBOSE_FORMAT("removed %d nodes, %d known nodes left",nremoved%knum);
    }
    return nremoved;
}

int CacheTree::UpdateFreeConfigurations(KinBodyPtr pbody) //todo only remove those with overlaping linkspheres
{
    int nremoved=0;
    if (_numnodes > 0) {

        FOREACH(itlevelnodes, _vsetLevelNodes) {
            FOREACH(itnode, *itlevelnodes) {
                if (((*itnode)->GetType() == CNT_Free)) {
                    (*itnode)->SetType(CNT_Unknown);
                    nremoved += 1;
                }
            }
        }

        int knum = GetNumKnownNodes();
        RAVELOG_VERBOSE_FORMAT("removed %d nodes, %d known nodes left",nremoved%knum);
    }

    return nremoved;
}

int CacheTree::RemoveFreeConfigurations()
{
    int nremoved=0;
    if (_numnodes > 0) {
        FOREACH(itlevelnodes, _vsetLevelNodes) {
            FOREACH(itnode, *itlevelnodes) {
                if (!!(*itnode)) {
                    if (((*itnode)->GetType() == CNT_Free)) {
                        (*itnode)->SetType(CNT_Unknown);
                        nremoved += 1;
                    }
                }
            }
        }

        int knum = GetNumKnownNodes();
        RAVELOG_VERBOSE_FORMAT("removed %d nodes, %d known nodes left",nremoved%knum);
    }

    return nremoved;
}

int CacheTree::GetNumKnownNodes()
{
    int nknown=0;
    if (_numnodes > 0) {
        FOREACH(itlevelnodes, _vsetLevelNodes) {
            FOREACH(itnode, *itlevelnodes) {
                if (((*itnode)->GetType() != CNT_Unknown) ) {
                    nknown += 1;
                }
            }
        }
    }
    return nknown;
}

bool CacheTree::Validate()
{
    if( _numnodes == 0 ) {
        return _numnodes==0;
    }

    if( _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).size() != 1 ) {
        int nroots = _vsetLevelNodes.at(_EncodeLevel(_maxlevel)).size();
        RAVELOG_WARN_FORMAT("more than 1 root node (%d)\n",nroots);
        return false;
    }

    dReal fLevelBound = _fMaxLevelBound;
    std::vector<CacheTreeNodePtr> vAccumNodes; vAccumNodes.reserve(_numnodes);
    std::map<CacheTreeNodePtr, CacheTreeNodePtr> mapNodeParents;
    size_t nallchildren = 0;
    size_t numnodes = 0;
    dReal fEpsilon = g_fEpsilon*_maxdistance; // min distance
    for(int currentlevel = _maxlevel; currentlevel >= _minlevel; --currentlevel, fLevelBound *= _fBaseInv ) {
        int enclevel = _EncodeLevel(currentlevel);
        if( enclevel >= (int)_vsetLevelNodes.size() ) {
            continue;
        }

        const std::set<CacheTreeNodePtr>& setLevelRawChildren = _vsetLevelNodes.at(enclevel);
        FOREACHC(itnode, setLevelRawChildren) {
            FOREACH(itchild, (*itnode)->_vchildren) {
                dReal curdist = RaveSqrt(_ComputeDistance2((*itnode)->GetConfigurationState(), (*itchild)->GetConfigurationState()));
                if( curdist > fLevelBound+fEpsilon ) {
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
                        mapNodeParents[*itnode] = *ittestnode;
                    }
                }
                BOOST_ASSERT(nfound==1);

                // check that the child is not that far away from the parent
                CacheTreeNodePtr nodeparent = mapNodeParents[*itnode];
                while(!!nodeparent) {
                    dReal dist = RaveSqrt(_ComputeDistance2(nodeparent->GetConfigurationState(), (*itnode)->GetConfigurationState()));
                    if( dist > RavePow(_base,nodeparent->_level+1)*_fBaseChildMult ) {
#ifdef _DEBUG
                        RAVELOG_WARN_FORMAT("node %d (level=%d) has parent %d (level=%d) such that dist %f > %f", (*itnode)->id%(*itnode)->_level%nodeparent->id%nodeparent->_level%dist%(RavePow(_base,nodeparent->_level+1)*_fBaseChildMult));
#else
                        RAVELOG_WARN_FORMAT("node (level=%d) has parent (level=%d) such that dist %f > %f", (*itnode)->_level%nodeparent->_level%dist%(RavePow(_base,nodeparent->_level+1)*_fBaseChildMult));
#endif
                        return false;
                    }
                    nodeparent = mapNodeParents[nodeparent];
                }
            }
        }
        numnodes += setLevelRawChildren.size();

        for(size_t i = 0; i < vAccumNodes.size(); ++i) {
            for(size_t j = i+1; j < vAccumNodes.size(); ++j) {
                dReal curdist = RaveSqrt(_ComputeDistance2(vAccumNodes[i]->GetConfigurationState(), vAccumNodes[j]->GetConfigurationState()));
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

ConfigurationCache::ConfigurationCache(RobotBasePtr pstaterobot, bool envupdates) : _cachetree(pstaterobot->GetDOF())
{
    _userdatakey = std::string("configurationcache") + boost::lexical_cast<std::string>(this);
    _pstaterobot = pstaterobot;
    _penv = pstaterobot->GetEnv();

    _envupdates = envupdates;

    _vgrabbedbodies.resize(0);
    _vnewenvbodies.resize(0);
    _vnewgrabbedbodies.resize(0);
    _vweights.resize(0);
    _setgrabbedbodies.clear();

    _pstaterobot->GetGrabbed(_vgrabbedbodies);
    _setgrabbedbodies.insert(_vgrabbedbodies.begin(), _vgrabbedbodies.end());

    if (_envupdates) {
        _handleBodyAddRemove = _penv->RegisterBodyCallback(boost::bind(&ConfigurationCache::_UpdateAddRemoveBodies, this, _1, _2));

        _penv->GetBodies(_vnewenvbodies);
        FOREACHC(itbody, _vnewenvbodies) {
            if( *itbody != pstaterobot && !pstaterobot->IsGrabbing(**itbody) ) {
                KinBodyCachedDataPtr pinfo(new KinBodyCachedData());
                pinfo->_changehandle = (*itbody)->RegisterChangeCallback(KinBody::Prop_LinkGeometry|KinBody::Prop_LinkEnable|KinBody::Prop_LinkTransforms, boost::bind(&ConfigurationCache::_UpdateUntrackedBody, this, *itbody));
                (*itbody)->SetUserData(_userdatakey, pinfo);
                _listCachedData.push_back(pinfo);
            }
        }

    }

    if (_envupdates) {
        _vRobotActiveIndices = _pstaterobot->GetActiveDOFIndices();
        _pstaterobot->GetActiveDOFResolutions(_vweights);
        _pstaterobot->GetActiveDOFLimits(_lowerlimit, _upperlimit);
    }
    else{
        _vRobotActiveIndices = _pstaterobot->GetActiveDOFIndices();
        _pstaterobot->GetDOFResolutions(_vweights);
        _pstaterobot->GetDOFLimits(_lowerlimit, _upperlimit);
    }

    _nRobotAffineDOF = pstaterobot->GetAffineDOF();
    _vRobotRotationAxis = pstaterobot->GetAffineRotationAxis();

    // if weights are zero, used a default value
    FOREACH(itweight, _vweights) {
        if( *itweight > 0 ) {
            *itweight = 1 / *itweight;
        }
        else {
            *itweight = 100;
        }
    }

    // threshparams
    // set default values for collisionthresh and insertiondistance
    _collisionthresh = 1.0; // discretization distance used by the original collisionchecker
    _freespacethresh = 0.2; // half disc. distance used by the original collisionchecker
    _insertiondistancemult = 0.5;


    _handleJointLimitChange = pstaterobot->RegisterChangeCallback(KinBody::Prop_JointLimits, boost::bind(&ConfigurationCache::_UpdateRobotJointLimits, this));
    _handleGrabbedChange = pstaterobot->RegisterChangeCallback(KinBody::Prop_RobotGrabbed, boost::bind(&ConfigurationCache::_UpdateRobotGrabbed, this));

    // distance has to be computed in the same way as CacheTreeNode.GetDistance()
    // otherwise, distances larger than this value could be inserted into the tree
    dReal maxdistance = 0;
    for (size_t i = 0; i < _vweights.size(); ++i) {
        dReal f = (_upperlimit[i] - _lowerlimit[i]) * _vweights[i];
        maxdistance += f*f;
    }

    _cachetree.Init(_vweights, RaveSqrt(maxdistance));

    if (IS_DEBUGLEVEL(Level_Verbose)) {
        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
        ss << "Initializing cache,  maxdistance " << _cachetree.GetMaxDistance() << ", weights [";
        for (size_t i = 0; i < _vweights.size(); ++i) {
            ss << _vweights[i] << " ";
        }
        ss << " (" << _vweights.size() << ") ";
        ss << "]\nupperlimit [";

        for (size_t i = 0; i < _upperlimit.size(); ++i) {
            ss << _upperlimit[i] << " ";
        }

        ss << "]\nlowerlimit [";

        for (size_t i = 0; i < _lowerlimit.size(); ++i) {
            ss << _lowerlimit[i] << " ";
        }
        ss << "]\n";
        RAVELOG_VERBOSE(ss.str());
    }
}

ConfigurationCache::~ConfigurationCache()
{
    _cachetree.Reset();
    // have to destroy all the change callbacks!
    FOREACH(it, _listCachedData) {
        KinBodyCachedDataPtr pdata = it->lock();
        if( !!pdata ) {
            pdata->_changehandle.reset();
        }
    }

}

void ConfigurationCache::SetWeights(const std::vector<dReal>& weights)
{
    _cachetree.SetWeights(weights);
}

bool ConfigurationCache::InsertConfiguration(const std::vector<dReal>& conf, CollisionReportPtr report, dReal distin)
{
    if( !!report ) {
        if( !!report->plink2 && report->plink2->GetParent() == _pstaterobot ) {
            std::swap(report->plink1, report->plink2);
        }
    }
    int ret = _cachetree.InsertNode(conf, report, !report ? _freespacethresh*_insertiondistancemult : _collisionthresh*_insertiondistancemult);
    BOOST_ASSERT(ret!=0);
    return ret==1;
}

int ConfigurationCache::GetNumKnownNodes()
{
    return _cachetree.GetNumKnownNodes();
}

int ConfigurationCache::RemoveCollisionConfigurations()
{
    return _cachetree.RemoveCollisionConfigurations();
}

int ConfigurationCache::UpdateCollisionConfigurations(KinBodyPtr pbody)
{
    return _cachetree.UpdateCollisionConfigurations(pbody);
}

int ConfigurationCache::UpdateFreeConfigurations(KinBodyPtr pbody)
{
    return _cachetree.UpdateFreeConfigurations(pbody);
}

int ConfigurationCache::RemoveFreeConfigurations()
{
    return _cachetree.RemoveFreeConfigurations();
}

void ConfigurationCache::GetDOFValues(std::vector<dReal>& values)
{
    // try to get the values without setting state
    if (_envupdates) {
        _pstaterobot->GetDOFValues(values, _vRobotActiveIndices);

        values.resize(_lowerlimit.size());
        if( _nRobotAffineDOF != 0 ) {
            RaveGetAffineDOFValuesFromTransform(values.begin()+_vRobotActiveIndices.size(), _pstaterobot->GetTransform(), _nRobotAffineDOF, _vRobotRotationAxis);
        }
    }
    else{

        values.resize(_lowerlimit.size());
        _pstaterobot->GetDOFValues(values);
        //values.resize(_lowerlimit.size());
        /*if( _nRobotAffineDOF != 0 ) {
            RaveGetAffineDOFValuesFromTransform(values.begin()+values.size(), _pstaterobot->GetTransform(), _nRobotAffineDOF, _vRobotRotationAxis);
           }*/
    }
}

int ConfigurationCache::CheckCollision(const std::vector<dReal>& conf, KinBody::LinkConstPtr& robotlink, KinBody::LinkConstPtr& collidinglink, dReal& closestdist)
{
    std::pair<CacheTreeNodeConstPtr, dReal> knn = _cachetree.FindNearestNode(conf, _collisionthresh, _freespacethresh);

    if( !!knn.first ) {

        closestdist = knn.second;
        if( knn.first->IsInCollision()) {

            if ((int)_pstaterobot->GetLinks().size() <= knn.first->GetRobotLinkIndex()) {
                robotlink = KinBody::LinkConstPtr(); //patch
            }
            else{
                robotlink = _pstaterobot->GetLinks().at(knn.first->GetRobotLinkIndex());
            }
            collidinglink = knn.first->GetCollidingLink();
            return 1;
        }
        return 0;
    }
    return -1;
}

std::pair<std::vector<dReal>, dReal> ConfigurationCache::FindNearestNode(const std::vector<dReal>& conf, dReal dist)
{
    std::pair<CacheTreeNodeConstPtr, dReal> knn = _cachetree.FindNearestNode(conf, dist, CNT_Any);

    if( !!knn.first ) {
        return make_pair(std::vector<dReal>(knn.first->GetConfigurationState(), knn.first->GetConfigurationState()+_lowerlimit.size()), knn.second);
    }
    return make_pair(std::vector<dReal>(0), dReal(0));
}

int ConfigurationCache::CheckCollision(KinBody::LinkConstPtr& robotlink, KinBody::LinkConstPtr& collidinglink, dReal& closestdist)
{
    std::vector<dReal> conf;
    GetDOFValues(conf);
    return CheckCollision(conf, robotlink, collidinglink, closestdist);
}

void ConfigurationCache::Reset()
{
    RAVELOG_DEBUG("Resetting cache\n");
    _cachetree.Reset();
}

bool ConfigurationCache::Validate()
{
    return _cachetree.Validate();
}

void ConfigurationCache::_UpdateUntrackedBody(KinBodyPtr pbody)
{
    // body's state has changed, so remove collision space and invalidate free space.
    if(_envupdates) {
        RAVELOG_VERBOSE_FORMAT("%s %s","Updating untracked bodies"%pbody->GetName());
        UpdateCollisionConfigurations(pbody);
        RemoveFreeConfigurations();
    }
}

void ConfigurationCache::_UpdateAddRemoveBodies(KinBodyPtr pbody, int action)
{
    if( action == 1 ) {
        if (_envupdates) {
            // invalidate the freespace of a cache given a new body in the scene
            if (RemoveFreeConfigurations() > 0) {
                RAVELOG_DEBUG_FORMAT("%s %s %d","Updating add/remove bodies"%pbody->GetName()%action);
            }
            KinBodyCachedDataPtr pinfo(new KinBodyCachedData());
            pinfo->_changehandle = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry|KinBody::Prop_LinkEnable|KinBody::Prop_LinkTransforms, boost::bind(&ConfigurationCache::_UpdateUntrackedBody, this, pbody));
            pbody->SetUserData(_userdatakey, pinfo);
            _listCachedData.push_back(pinfo);
        }
    }
    else if( action == 0 ) {
        if (_envupdates) {
            if ( UpdateCollisionConfigurations(pbody) > 0) {
                RAVELOG_DEBUG_FORMAT("%s %s %d","Updating add/remove bodies"%pbody->GetName()%action);
                // remove all configurations that collide with this body
            }
            pbody->RemoveUserData(_userdatakey);
        }
    }
}

void ConfigurationCache::_UpdateRobotJointLimits()
{

    if (_envupdates) {
        RAVELOG_VERBOSE("Updating robot joint limits\n");

        _pstaterobot->SetActiveDOFs(_vRobotActiveIndices, _nRobotAffineDOF);
        _pstaterobot->GetActiveDOFLimits(_newlowerlimit, _newupperlimit);

        if (_newlowerlimit != _lowerlimit || _newupperlimit != _upperlimit)
        {
            // compute new max distance for cache
            // distance has to be computed in the same way as CacheTreeNode.GetDistance()
            // otherwise, distances larger than this value could be inserted into the tree
            dReal maxdistance = 0;
            for (size_t i = 0; i < _lowerlimit.size(); ++i) {
                dReal f = (_upperlimit[i] - _lowerlimit[i]) * _cachetree.GetWeights().at(i);
                maxdistance += f*f;
            }
            maxdistance = RaveSqrt(maxdistance);
            if( maxdistance > _cachetree.GetMaxDistance()+g_fEpsilonLinear ) {
                _cachetree.SetMaxDistance(maxdistance);
            }

            _lowerlimit = _newlowerlimit;
            _upperlimit = _newupperlimit;
        }
    }
}

void ConfigurationCache::_UpdateRobotGrabbed()
{
    bool newGrab = false;

    _vnewgrabbedbodies.resize(0);
    _pstaterobot->GetGrabbed(_vnewgrabbedbodies);
    FOREACH(oldbody, _setgrabbedbodies){
        FOREACH(newbody, _vnewgrabbedbodies){
            if ((*oldbody) != (*newbody)) {
                newGrab = true;
                break;
            }
        }
    }

    if (newGrab) {
        RAVELOG_DEBUG("Updating robot grabbed\n");
        FOREACH(newbody, _vnewgrabbedbodies){
            UpdateCollisionConfigurations((*newbody));
        }
        _setgrabbedbodies.insert(_vnewgrabbedbodies.begin(), _vnewgrabbedbodies.end());
        RemoveFreeConfigurations();
    }
}

}
