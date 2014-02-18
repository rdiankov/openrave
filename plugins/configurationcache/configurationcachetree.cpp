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

// High
// find a way to lose the map for children

//Low

//KinBodyPtr body;
//std::vector<dReal> values;
//std::vector<uint8_t> venablestates;
//body->GetConfigurationValues(values);
//body->GetLinkEnables(venablestates);

//std::map<KinBodyPtr, std::pair<std::vector<dReal>, std::vector<uint8_t> > > mapEnvironmentState;
namespace configurationcache {

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
}

void CacheTreeNode::SetCollisionInfo(CollisionReportPtr report)
{
    if(!!report && report->numCols > 0) {
        _collidinglinktrans = report->plink1->GetTransform();
        _robotlinkindex = report->plink1->GetIndex();
        _collidinglink = report->plink2;
        _conftype = CNT_Collision;
    }
    else {
        _conftype = CNT_Free;
        _collidinglink.reset();
        _robotlinkindex = -1;
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

CacheTree::CacheTree(int statedof) : _poolNodes(sizeof(CacheTreeNode)+sizeof(dReal)*statedof)
{
    _root = NULL;
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
    OPENRAVE_ASSERT_OP((int)weights.size(),==,_statedof);
    Reset();
    _weights = weights;
    _numnodes = 0;
    _base = 2.0;
    _fBaseInv = 1/_base;
    _maxdistance = maxdistance;
    _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
    _minlevel = _maxlevel - 1;
    _fMaxLevelBound = RavePow(_base, _maxlevel);
    int enclevel = _EncodeLevel(_maxlevel);
    if( enclevel >= (int)_vmapNodeChildren.size() ) {
        _vmapNodeChildren.resize(enclevel+1);
    }
}

void CacheTree::Reset()
{
    // make sure all children are deleted
    std::set<CacheTreeNodeConstPtr> _setdeleted;
    if( !!_root ) {
        _root->~CacheTreeNode();
        _setdeleted.insert(_root);
        _root = NULL;
    }

    for(size_t ilevel = 0; ilevel < _vmapNodeChildren.size(); ++ilevel) {
        FOREACH(itnodes, _vmapNodeChildren[ilevel]) {
            if( _setdeleted.find(itnodes->first) != _setdeleted.end() ) {
                itnodes->first->~CacheTreeNode();
                _setdeleted.insert(itnodes->first);
            }
            FOREACH(itchild, itnodes->second) {
                if( _setdeleted.find(*itchild) != _setdeleted.end() ) {
                    (*itchild)->~CacheTreeNode();
                    _setdeleted.insert(*itchild);
                }
            }
        }
    }
    FOREACH(itchildren, _vmapNodeChildren) {
        itchildren->clear();
    }
    _poolNodes.purge_memory();
    _numnodes = 0;
}

CacheTreeNodePtr CacheTree::_CreateCacheTreeNode(const std::vector<dReal>& cs, CollisionReportPtr report)
{
    // allocate memory for the structur and the internal state vectors
    void* pmemory = _poolNodes.malloc();
    //Vector* plinkspheres = (Vector*)((uint8_t*)pmemory + sizeof(CacheTreeNode) + sizeof(dReal)*_statedof);
    CacheTreeNodePtr newnode = new (pmemory) CacheTreeNode(cs, NULL);
    newnode->SetCollisionInfo(report);
    return newnode;
}

void CacheTree::_DeleteCacheTreeNode(CacheTreeNodePtr pnode)
{
    pnode->~CacheTreeNode();
    _poolNodes.free(pnode);
}

//dReal CacheTree::ComputeDistance(CacheTreeNodePtr vi, CacheTreeNodePtr vf)
//{
//    dReal distance = _ComputeDistance(vi->GetConfigurationState(), vf->GetConfigurationState());
//
//    // use this distance information to update the upper bounds stored on the node
//    vi->UpdateApproximates(distance,vf);
//    vf->UpdateApproximates(distance,vi);
//
//    return distance;
//}

dReal CacheTree::ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef) const
{
    return _ComputeDistance(&cstatei[0], &cstatef[0]);
}

dReal CacheTree::_ComputeDistance(const dReal* cstatei, const dReal* cstatef) const
{
    dReal distance = 0;
    for (size_t i = 0; i < _weights.size(); ++i) {
        distance += RaveFabs(cstatei[i] - cstatef[i]) * _weights[i];
    }
    return distance;
}

void CacheTree::SetWeights(const std::vector<dReal>& weights)
{
    OPENRAVE_ASSERT_OP((int)weights.size(),==,_statedof);
    Reset();
    _weights = weights;
}

void CacheTree::SetMaxDistance(dReal maxdistance)
{
    Reset();
    _maxdistance = maxdistance;
    // have to update the max level?
}

std::pair<CacheTreeNodeConstPtr, dReal> CacheTree::FindNearestNode(const std::vector<dReal>& vquerystate, dReal distancebound, ConfigurationNodeType conftype) const
{
    CacheTreeNodeConstPtr pbestnode=NULL;
    dReal bestdist = std::numeric_limits<dReal>::infinity();
    if( !_root ) {
        return make_pair(pbestnode, bestdist);
    }
    OPENRAVE_ASSERT_OP(vquerystate.size(),==,_weights.size());
    // first localmax is distance from this node to the root
    const dReal* pquerystate = &vquerystate[0];

    dReal curdist = _ComputeDistance(pquerystate, _root->GetConfigurationState());
    _vCurrentLevelNodes.resize(1);
    _vCurrentLevelNodes[0] = make_pair(_root, curdist);
    if( conftype == CNT_Any || _root->GetType() == conftype ) {
        bestdist = curdist;
        pbestnode = _root;
        if( distancebound > 0 && bestdist <= distancebound ) {
            return make_pair(pbestnode, bestdist);
        }
    }
    int currentlevel = _maxlevel; // where the root node is
    // traverse all levels gathering up the children at each level
    dReal fLevelBound = _fMaxLevelBound;
    std::map<CacheTreeNodeConstPtr, std::vector<CacheTreeNodePtr> >::const_iterator itchildren;
    while(_vCurrentLevelNodes.size() > 0 ) {
        const std::map<CacheTreeNodeConstPtr, std::vector<CacheTreeNodePtr> >& mapLevelRawChildren = _vmapNodeChildren.at(_EncodeLevel(currentlevel));
        _vNextLevelNodes.resize(0);
        FOREACH(itcurrentnode, _vCurrentLevelNodes) {
            // only take the children whose distances are within the bound
            itchildren = mapLevelRawChildren.find(itcurrentnode->first);
            if( itchildren != mapLevelRawChildren.end() ) {
                FOREACHC(itchild, itchildren->second) {
                    dReal curdist = _ComputeDistance(pquerystate, (*itchild)->GetConfigurationState());
                    if( conftype == CNT_Any || (*itchild)->GetType() == conftype ) {
                        if( curdist < bestdist ) {
                            bestdist = curdist;
                            pbestnode = *itchild;
                            if( distancebound > 0 && bestdist <= distancebound ) {
                                return make_pair(pbestnode, bestdist);
                            }
                        }
                    }
                    if( curdist < itcurrentnode->second + fLevelBound ) {
                        _vNextLevelNodes.push_back(make_pair(*itchild, curdist));
                    }
                }
            }
        }
        _vCurrentLevelNodes.swap(_vNextLevelNodes);
        currentlevel -= 1;
        fLevelBound *= _fBaseInv;
    }
    return make_pair(pbestnode, bestdist);
}

int CacheTree::InsertNode(const std::vector<dReal>& cs, CollisionReportPtr report, dReal fMaxSeparationDist)
{
    OPENRAVE_ASSERT_OP(cs.size(),==,_weights.size());
    CacheTreeNodePtr nodein = _CreateCacheTreeNode(cs, report);
    // if there is no root, make this the root, otherwise call the lowlevel  insert
    if( !_root ) {
        _root = nodein;
        _numnodes += 1;
        return 1;
    }
    _vmapNodeChildren.at(_EncodeLevel(_maxlevel));

    _vCurrentLevelNodes.resize(1);
    _vCurrentLevelNodes[0].first = _root;
    _vCurrentLevelNodes[0].second = _ComputeDistance(_root->GetConfigurationState(), &cs[0]);
    int nParentFound = _Insert(nodein, _vCurrentLevelNodes, _maxlevel, _fMaxLevelBound, fMaxSeparationDist);
    if( nParentFound != 1 ) {
        _DeleteCacheTreeNode(nodein);
    }
    return nParentFound;
}

int CacheTree::_Insert(CacheTreeNodePtr nodein, const std::vector< std::pair<CacheTreeNodePtr, dReal> >& nodesin, int currentlevel, dReal fLevelBound, dReal fMaxSeparationDist)
{
    dReal closestDist=0;
    CacheTreeNodePtr closestNodeInRange=NULL; /// one of the nodes in nodesin such that its distance to nodein is <= fLevelBound
    int enclevel = _EncodeLevel(currentlevel);
    if( enclevel < (int)_vmapNodeChildren.size() ) {
        // build the level below
        std::map<CacheTreeNodeConstPtr, std::vector<CacheTreeNodePtr> >::const_iterator itchildren;
        const std::map<CacheTreeNodeConstPtr, std::vector<CacheTreeNodePtr> >& mapLevelRawChildren = _vmapNodeChildren.at(enclevel);
        _vNextLevelNodes.resize(0);

        FOREACH(itcurrentnode, nodesin) {
            if( itcurrentnode->second <= fLevelBound ) {
                if(  !closestNodeInRange || itcurrentnode->second < closestDist ) {
                    closestNodeInRange = itcurrentnode->first;
                    closestDist = itcurrentnode->second;
                    if( closestDist < fMaxSeparationDist && (!nodein->IsInCollision() || closestNodeInRange->IsInCollision()) ) {
                        // pretty close, so return as if node was added
                        return -1;
                    }
                }
            }
            // only take the children whose distances are within the bound
            itchildren = mapLevelRawChildren.find(itcurrentnode->first);
            if( itchildren != mapLevelRawChildren.end() ) {
                FOREACHC(itchild, itchildren->second) {
                    dReal curdist = _ComputeDistance(nodein->GetConfigurationState(), (*itchild)->GetConfigurationState());
                    if( curdist <= fLevelBound ) {
                        _vNextLevelNodes.push_back(make_pair(*itchild, curdist));
                    }
                }
            }
        }

        if( _vNextLevelNodes.size() > 0 ) {
            _vCurrentLevelNodes.swap(_vNextLevelNodes);
            // note that after _Insert call, _vCurrentLevelNodes could be complete lost/reset
            int nParentFound = _Insert(nodein, _vCurrentLevelNodes, currentlevel-1, fLevelBound*_fBaseInv, fMaxSeparationDist);
            if( nParentFound != 0 ) {
                return nParentFound;
            }
        }
    }
    else {
        FOREACH(itcurrentnode, nodesin) {
            if( itcurrentnode->second <= fLevelBound ) {
                if(  !closestNodeInRange || itcurrentnode->second < closestDist ) {
                    closestNodeInRange = itcurrentnode->first;
                    closestDist = itcurrentnode->second;
                    if( closestDist < fMaxSeparationDist && (!nodein->IsInCollision() || closestNodeInRange->IsInCollision()) ) {
                        // pretty close, so return as if node was added
                        return -1;
                    }
                }
            }
        }
    }

    if( !closestNodeInRange ) {
        return 0;
    }

    // insert nodein into children of currentNodeInRange
    if( enclevel >= (int)_vmapNodeChildren.size() ) {
        _vmapNodeChildren.resize(enclevel+1);
    }

    _vmapNodeChildren[enclevel][closestNodeInRange].push_back(nodein);
    if( _minlevel > currentlevel-1 ) {
        _minlevel = currentlevel-1;
    }
    _numnodes += 1;
    return 1;
}

bool CacheTree::RemoveNode(CacheTreeNodeConstPtr removenode)
{
    if( !_root ) {
        return false;
    }

    if( _numnodes == 1 && removenode == _root ) {
        Reset();
        return true;
    }

    if( _maxlevel-_minlevel < (int)_vvCacheNodes.size() ) {
        _vvCacheNodes.resize(_maxlevel-_minlevel+1);
    }
    FOREACH(it, _vvCacheNodes) {
        it->resize(0);
    }
    _vvCacheNodes.at(0).push_back(_root);
    bool bRemoved = _Remove(removenode, _vvCacheNodes, _maxlevel, _fMaxLevelBound);
    if( bRemoved ) {
        _DeleteCacheTreeNode(const_cast<CacheTreeNodePtr>(removenode));
    }
    if( removenode == _root ) {
        BOOST_ASSERT(_vvCacheNodes.at(0).size()==2); // instead of root, another node should have been added
        _root = _vvCacheNodes.at(0).at(1);
        bRemoved = true;
    }
    if( bRemoved ) {
        _numnodes -= 1;
    }

    return bRemoved;
}

bool CacheTree::_Remove(CacheTreeNodeConstPtr removenode, std::vector< std::vector<CacheTreeNodePtr> >& vvCoverSetNodes, int currentlevel, dReal fLevelBound)
{
    int enclevel = _EncodeLevel(currentlevel);
    if( enclevel >= (int)_vmapNodeChildren.size() ) {
        return false;
    }

    // build the level below
    std::map<CacheTreeNodeConstPtr, std::vector<CacheTreeNodePtr> >::iterator itchildren;
    std::map<CacheTreeNodeConstPtr, std::vector<CacheTreeNodePtr> >& mapLevelRawChildren = _vmapNodeChildren.at(enclevel);
    int coverindex = _maxlevel-(currentlevel-1);
    if( coverindex >= (int)vvCoverSetNodes.size() ) {
        vvCoverSetNodes.resize(coverindex+(_maxlevel-_minlevel)+1);
    }
    std::vector<CacheTreeNodePtr>& vNextLevelNodes = vvCoverSetNodes[coverindex];
    vNextLevelNodes.resize(0);

    bool bfound = false;
    FOREACH(itcurrentnode, vvCoverSetNodes.at(coverindex-1)) {
        // only take the children whose distances are within the bound
        itchildren = mapLevelRawChildren.find(*itcurrentnode);
        if( itchildren != mapLevelRawChildren.end() ) {
            std::vector<CacheTreeNodePtr>::iterator itchild = itchildren->second.begin();
            while(itchild != itchildren->second.end() ) {
                dReal curdist = _ComputeDistance(removenode->GetConfigurationState(), (*itchild)->GetConfigurationState());
                if( curdist <= fLevelBound ) {
                    vNextLevelNodes.push_back(*itchild);
                }
                if( *itchild == removenode ) {
                    itchild = itchildren->second.erase(itchild);
                    bfound = true;
                }
                else {
                    ++itchild;
                }
            }
            if( bfound ) {
                break;
            }
        }
    }

    bool bRemoved = _Remove(removenode, vvCoverSetNodes, currentlevel-1, fLevelBound*_fBaseInv);

    itchildren = mapLevelRawChildren.find(removenode);
    if( itchildren != mapLevelRawChildren.end() ) {
        // for each child, find a more suitable parent
        FOREACH(itchild, itchildren->second) {
            const dReal* pchildstate = (*itchild)->GetConfigurationState();
            int parentlevel = currentlevel-1;
            dReal fParentLevelBound = fLevelBound*_fBaseInv;
            dReal closestdist=0;
            CacheTreeNodePtr closestNode = NULL;
            //int maxaddlevel = currentlevel-1;
            while(parentlevel <= _maxlevel && vvCoverSetNodes.at(_maxlevel-parentlevel).size() > 0 ) {
                FOREACHC(itnode, vvCoverSetNodes.at(_maxlevel-parentlevel)) {
                    if( *itnode == removenode ) {
                        continue;
                    }
                    dReal curdist = _ComputeDistance(pchildstate, (*itnode)->GetConfigurationState());
                    if( curdist < fParentLevelBound ) {
                        if( !closestNode || curdist < closestdist ) {
                            closestdist = curdist;
                            closestNode = *itnode;
                        }
                    }
                }
                if( !!closestNode ) {
                    // closest node was found in parentlevel, so add to the children
                    _vmapNodeChildren[_EncodeLevel(parentlevel)][closestNode].push_back(*itchild);
                    // should also add to vvCoverSetNodes..?
                    //vvCoverSetNodes.at(_maxlevel-(parentlevel-1)).push_back(*itchild);
                    break;
                }
                // try a higher level
                parentlevel += 1;
                fParentLevelBound *= _base;
            }
            if( !closestNode ) {
                // occurs when root node is being removed and new children have no where to go
                vvCoverSetNodes.at(0).push_back(*itchild);
            }
        }
        // remove the children from node
        mapLevelRawChildren.erase(itchildren);
    }
    return bRemoved;
}

void CacheTree::UpdateTree()
{

}

ConfigurationCache::ConfigurationCache(RobotBasePtr pstaterobot) : _cachetree(pstaterobot->GetActiveDOF())
{
    _userdatakey = std::string("configurationcache") + boost::lexical_cast<std::string>(this);
    _qtime = 0;
    _itime = 0;
    _profile = false;
    _pstaterobot = pstaterobot;
    _penv = pstaterobot->GetEnv();
    _handleBodyAddRemove = _penv->RegisterBodyCallback(boost::bind(&ConfigurationCache::_UpdateAddRemoveBodies, this, _1, _2));

    std::vector<KinBodyPtr> vGrabbedBodies;
    _pstaterobot->GetGrabbed(vGrabbedBodies);
    _setGrabbedBodies.insert(vGrabbedBodies.begin(), vGrabbedBodies.end());
    std::vector<KinBodyPtr> vnewenvbodies;
    _penv->GetBodies(vnewenvbodies);
    FOREACHC(itbody, vnewenvbodies) {
        if( *itbody != pstaterobot && !pstaterobot->IsGrabbing(*itbody) ) {
            KinBodyCachedDataPtr pinfo(new KinBodyCachedData());
            pinfo->_changehandle = (*itbody)->RegisterChangeCallback(KinBody::Prop_LinkGeometry|KinBody::Prop_LinkEnable|KinBody::Prop_LinkTransforms, boost::bind(&ConfigurationCache::_UpdateUntrackedBody, this, *itbody));
            (*itbody)->SetUserData(_userdatakey, pinfo);
        }
    }

    _vRobotActiveIndices = pstaterobot->GetActiveDOFIndices();
    _nRobotAffineDOF = pstaterobot->GetAffineDOF();
    _vRobotRotationAxis = pstaterobot->GetAffineRotationAxis();

    std::vector<dReal> vweights;
    pstaterobot->GetActiveDOFResolutions(vweights);

    // if weights are zero, used a default value
    FOREACH(itweight, vweights) {
        if( *itweight > 0 ) {
            *itweight = 1 / *itweight;
        }
        else {
            *itweight = 100;
        }
    }

    // set default values for collisionthresh and insertiondistance
    _collisionthresh = 1.0; //1.0; //minres; //smallest resolution
    _freespacethresh = 0.2;
    _insertiondistancemult = 0.5; //0.5; //0.02;  //???

    _pstaterobot->GetActiveDOFLimits(_lowerlimit, _upperlimit);

    _jointchangehandle = pstaterobot->RegisterChangeCallback(KinBody::Prop_JointLimits, boost::bind(&ConfigurationCache::_UpdateRobotJointLimits, this));
    _jointchangehandle = pstaterobot->RegisterChangeCallback(KinBody::Prop_RobotGrabbed, boost::bind(&ConfigurationCache::_UpdateRobotGrabbed, this));

    // using L1, get the maximumdistance
    // distance has to be computed in the same way as CacheTreeNode.GetDistance()
    // otherwise, distances larger than this value could be inserted into the tree
    dReal bound = 0;
    for (size_t i = 0; i < _lowerlimit.size(); ++i) {
        bound += RaveFabs(_upperlimit[i] - _lowerlimit[i]) * vweights[i];
    }

    _cachetree.Init(vweights, bound);

    if (IS_DEBUGLEVEL(Level_Debug)) {
        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
        ss << "Initializing cache,  maxdistance " << _cachetree.GetMaxDistance() << ", collisionthresh " << _collisionthresh << ", _insertiondistancemult "<< _insertiondistancemult << ", weights [";
        for (size_t i = 0; i < vweights.size(); ++i) {
            ss << vweights[i] << " ";
        }
        ss << "]\nupperlimit [";

        for (size_t i = 0; i < _upperlimit.size(); ++i) {
            ss << _upperlimit[i] << " ";
        }

        ss << "]\nlowerlimit [";

        for (size_t i = 0; i < _lowerlimit.size(); ++i) {
            ss << _lowerlimit[i] << " ";
        }
        ss << "]\n";
        RAVELOG_DEBUG(ss.str());
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

int ConfigurationCache::RemoveConfigurations(const std::vector<dReal>& cs, dReal radius)
{
    // slow implementation for now
    int nremoved=0;
    while(1) {
        std::pair<CacheTreeNodeConstPtr, dReal> neigh = _cachetree.FindNearestNode(cs, radius, CNT_Any);
        if( !neigh.first ) {
            break;
        }
        OPENRAVE_ASSERT_OP(neigh.second,<=,radius);
        bool bremoved = _cachetree.RemoveNode(neigh.first);
        BOOST_ASSERT(bremoved);
        nremoved += 1;
    }
    return nremoved;
}

void ConfigurationCache::GetDOFValues(std::vector<dReal>& values)
{
    // try to get the values without setting state
    _pstaterobot->GetDOFValues(values, _vRobotActiveIndices);
    values.resize(_lowerlimit.size());
    if( _nRobotAffineDOF != 0 ) {
        RaveGetAffineDOFValuesFromTransform(values.begin()+_vRobotActiveIndices.size(), _pstaterobot->GetTransform(), _nRobotAffineDOF, _vRobotRotationAxis);
    }
}

int ConfigurationCache::CheckCollision(KinBody::LinkConstPtr& robotlink, KinBody::LinkConstPtr& collidinglink, dReal& closestdist)
{
    if( _cachetree.GetNumNodes() == 0 ) {
        return -1;
    }
    std::vector<dReal> conf;
    GetDOFValues(conf);
    
    // have to do radius search for both colliding and free nodes. Perhaps there's a way to merge the functionality...
    std::pair<CacheTreeNodeConstPtr, dReal> knncollision = _cachetree.FindNearestNode(conf, _collisionthresh, CNT_Collision);
    if( !!knncollision.first ) {
        BOOST_ASSERT(knncollision.first->IsInCollision());
        robotlink = _pstaterobot->GetLinks().at(knncollision.first->GetRobotLinkIndex());
        collidinglink = knncollision.first->GetCollidingLink();
        closestdist = knncollision.second;
        return 1;
    }

    std::pair<CacheTreeNodeConstPtr, dReal> knnfree = _cachetree.FindNearestNode(conf, _freespacethresh, CNT_Free);
    if( !!knnfree.first ) {
        BOOST_ASSERT(!knnfree.first->IsInCollision());
        closestdist = knnfree.second;
        return 0;
    }
    
    return -1;
}

void ConfigurationCache::Reset()
{
    _cachetree.Reset();
}

void ConfigurationCache::_UpdateUntrackedBody(KinBodyPtr pbody)
{
    // body's state has changed, so remove collision space and invalidate free space.

    //SynchronizeAll(_pstaterobot);
}

void ConfigurationCache::_UpdateAddRemoveBodies(KinBodyPtr pbody, int action)
{
    if( action == 1 ) {
        KinBodyCachedDataPtr pinfo(new KinBodyCachedData());
        pinfo->_changehandle = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry|KinBody::Prop_LinkEnable|KinBody::Prop_LinkTransforms, boost::bind(&ConfigurationCache::_UpdateUntrackedBody, this, pbody));
        pbody->SetUserData(_userdatakey, pinfo);

        // TODO invalide the freespace of a cache given a new body in the scene
    }
    else if( action == 0 ) {
        pbody->RemoveUserData(_userdatakey);
        // TODO invalidate the collision space of a cache given a body has been removed
    }
}

void ConfigurationCache::_UpdateRobotJointLimits()
{
    _pstaterobot->SetActiveDOFs(_vRobotActiveIndices, _nRobotAffineDOF);
    _pstaterobot->GetActiveDOFLimits(_lowerlimit, _upperlimit);

    // compute new max distance for cache
    // using L1, get the maximumdistance
    // distance has to be computed in the same way as CacheTreeNode.GetDistance()
    // otherwise, distances larger than this value could be inserted into the tree
    dReal bound = 0;
    for (size_t i = 0; i < _lowerlimit.size(); ++i) {
        bound += RaveFabs(_upperlimit[i] - _lowerlimit[i]) * _cachetree.GetWeights().at(i);
    }

    if( bound > _cachetree.GetMaxDistance()+g_fEpsilonLinear ) {
        _cachetree.SetMaxDistance(bound);
    }
}

void ConfigurationCache::_UpdateRobotGrabbed()
{
    _cachetree.Reset();
}

}
