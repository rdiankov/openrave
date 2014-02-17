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

CacheTreeNode::CacheTreeNode(dReal* pcstate, Vector* plinkspheres)
{
    _pcstate = pcstate;
    _plinkspheres = plinkspheres;
//    _approxdispersion.first = CacheTreeNodePtr();
//    _approxdispersion.second = std::numeric_limits<float>::infinity();
//    _approxnn.first = CacheTreeNodePtr();
//    _approxnn.second = std::numeric_limits<float>::infinity();
    _conftype = CNT_Unknown;
    _robotlinkindex = -1;
//    SetCollisionInfo(report);
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


//void CacheTreeNode::RemoveChild(CacheTreeNodePtr child, int level)
//{
//    std::vector<CacheTreeNodePtr>:: iterator it = _children[level].begin();
//    for (; it != _children[level].end(); ) {
//        if (*it == child) {
//            it = _children[level].erase(it);
//        }
//        else{
//            ++it;
//        }
//    }
//}

CacheTree::CacheTree(int statedof) : _poolNodes(sizeof(CacheTreeNode)+sizeof(dReal)*statedof)
{
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
    //_insertiondistance = 0;
    int enclevel = _EncodeLevel(_maxlevel);
    if( enclevel >= (int)_vmapNodeChildren.size() ) {
        _vmapNodeChildren.resize(enclevel+1);
    }
}

void CacheTree::Reset()
{
    // make sure all children are deleted
    std::set<CacheTreeNodePtr> _setdeleted;
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
    _vmapNodeChildren.resize(0);
    _poolNodes.purge_memory();
}

CacheTreeNodePtr CacheTree::_CreateCacheTreeNode(const std::vector<dReal>& cs, CollisionReportPtr report)
{
    // allocate memory for the structur and the internal state vectors
    void* pmemory = _poolNodes.malloc();
    dReal* pcstate = (dReal*)((uint8_t*)pmemory + sizeof(CacheTreeNode));
    std::copy(cs.begin(), cs.end(), pcstate);
    CacheTreeNodePtr newnode = new (pmemory) CacheTreeNode(pcstate, NULL);
    if( !!report ) {
        newnode->SetCollisionInfo(report);
    }
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
        if( bestdist <= distancebound ) {
            return make_pair(pbestnode, bestdist);
        }
    }
    int currentlevel = _maxlevel; // where the root node is
    // traverse all levels gathering up the children at each level
    dReal fLevelBound = _fMaxLevelBound;
    std::map<CacheTreeNodePtr, std::vector<CacheTreeNodePtr> >::const_iterator itchildren;
    while(_vCurrentLevelNodes.size() > 0 ) {
        const std::map<CacheTreeNodePtr, std::vector<CacheTreeNodePtr> >& mapLevelRawChildren = _vmapNodeChildren.at(_EncodeLevel(currentlevel));
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
                            if( bestdist <= distancebound ) {
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

bool CacheTree::InsertNode(const std::vector<dReal>& cs, CollisionReportPtr report, dReal fMaxSeparationDist)
{
    OPENRAVE_ASSERT_OP(cs.size(),==,_weights.size());
    CacheTreeNodePtr nodein = _CreateCacheTreeNode(cs, report);
    // if there is no root, make this the root, otherwise call the lowlevel  insert
    if( !_root ) {
        _root = nodein;
        return true;
    }
    _vmapNodeChildren.at(_EncodeLevel(_maxlevel));

    dReal fLevelBound = _fMaxLevelBound;
    _vCurrentLevelNodes.resize(1);
    _vCurrentLevelNodes[0].first = _root;
    _vCurrentLevelNodes[0].second = _ComputeDistance(_root->GetConfigurationState(), &cs[0]);
    return _Insert(nodein, _vCurrentLevelNodes, _maxlevel, fLevelBound, fMaxSeparationDist);
}

bool CacheTree::_Insert(CacheTreeNodePtr nodein, const std::vector< std::pair<CacheTreeNodePtr, dReal> >& nodesin, int currentlevel, dReal fLevelBound, dReal fMaxSeparationDist)
{
    dReal closestDist=0;
    CacheTreeNodePtr closestNodeInRange=NULL; /// one of the nodes in nodesin such that its distance to nodein is <= fLevelBound
    int enclevel = _EncodeLevel(currentlevel);
    if( enclevel < (int)_vmapNodeChildren.size() ) {
        // build the level below
        std::map<CacheTreeNodePtr, std::vector<CacheTreeNodePtr> >::const_iterator itchildren;
        const std::map<CacheTreeNodePtr, std::vector<CacheTreeNodePtr> >& mapLevelRawChildren = _vmapNodeChildren.at(enclevel);
        _vNextLevelNodes.resize(0);

        FOREACH(itcurrentnode, nodesin) {
            if( itcurrentnode->second <= fLevelBound ) {
                if(  !closestNodeInRange || itcurrentnode->second < closestDist ) {
                    closestNodeInRange = itcurrentnode->first;
                    closestDist = itcurrentnode->second;
                    if( closestDist < fMaxSeparationDist ) {
                        // pretty close, so return as if node was added
                        return true;
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

        if( _vNextLevelNodes.size() == 0 ) {
            return false;
        }

        _vCurrentLevelNodes.swap(_vNextLevelNodes);
        // note that after _Insert call, _vCurrentLevelNodes could be complete lost/reset
        bool bParentFound = _Insert(nodein, _vCurrentLevelNodes, currentlevel-1, fLevelBound*_fBaseInv, fMaxSeparationDist);
        if( bParentFound ) {
            return true;
        }
    }
    else {
        FOREACH(itcurrentnode, nodesin) {
            if( itcurrentnode->second <= fLevelBound ) {
                if(  !closestNodeInRange || itcurrentnode->second < closestDist ) {
                    closestNodeInRange = itcurrentnode->first;
                    closestDist = itcurrentnode->second;
                    if( closestDist < fMaxSeparationDist ) {
                        // pretty close, so return as if node was added
                        return true;
                    }
                }
            }
        }
    }

    if( !closestNodeInRange ) {
        return false;
    }

    // insert nodein into children of currentNodeInRange
    if( enclevel >= (int)_vmapNodeChildren.size() ) {
        _vmapNodeChildren.resize(enclevel+1);
    }

    _vmapNodeChildren[enclevel][closestNodeInRange].push_back(nodein);
    if( _minlevel > currentlevel-1 ) {
        _minlevel = currentlevel-1;
    }
    return true;
}

//int CacheTree::RemoveNode(CacheTreeNodePtr nodein)
//{
//    // todo: if there is no root, make this the root
//    // otherwise call the lowlevel  insert
//
//    std::vector<CacheTreeNodePtr> nodesin;
//    std::vector<dReal> distin;
//
//    distin.push_back(ComputeDistance(_root, nodein));
//    nodesin.push_back(_root); //this is always the same, perhaps keep make this vector at construction?
//
//    _Remove(nodein, nodesin, distin, _maxlevel);
//
//    return 1;
//}

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

int CacheTree::_Remove(CacheTreeNodePtr in, const std::vector<CacheTreeNodePtr>& nodesin, const std::vector<dReal>& leveldists,  int level)
{
    // 2^{i}
    /*dReal qleveldistance = RavePow(_base, level);

       dReal minimumdistance = std::numeric_limits<float>::infinity();
       CacheTreeNodePtr minimumdistancev;
       dReal localmindist = std::numeric_limits<float>::infinity();

       std::vector<CacheTreeNodePtr> nextlevelnodes;
       std::vector<dReal> nextleveldists;
       CacheTreeNodePtr parent;

       for (size_t i = 0; i < nodesin.size(); ++i) {

        if (leveldists[i] < minimumdistance) {
            minimumdistance = leveldists[i];
            minimumdistancev = nodesin[i];
        }

        if (leveldists[i] <= qleveldistance) {
            nextleveldists.push_back(leveldists[i]);
            nextlevelnodes.push_back(nodesin[i]);
        }

        // now consider children
        const std::vector<CacheTreeNodePtr>& levelchildren = nodesin[i]->GetChildren(level);

        for (size_t j = 0; j < levelchildren.size(); ++j) {

            dReal curdist = ComputeDistance(in, levelchildren[j]);

            if (curdist < minimumdistance) {
                minimumdistance = curdist;
                minimumdistancev = levelchildren[j];
                if (curdist == 0) {
                    parent = levelchildren[j];
                }
            }

            if (curdist <= qleveldistance)
            {
                nextleveldists.push_back(curdist);
                nextlevelnodes.push_back(levelchildren[j]);
            }
        }
       }

       if (level > _minlevel) {
        _Remove(in, nextlevelnodes, nextleveldists, level-1);
       }

       if (!!parent) {
        //parent->removeChild(level, minimumdistancev);
       }

       const std::vector<CacheTreeNodePtr>& minchildren = minimumdistancev->GetChildren(level-1);*/

    return 0;
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
    _insertiondistance = 0.5; //0.5; //0.02;  //???

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
    //_cachetree.SetInsertionDistance(_insertiondistance);

    if (IS_DEBUGLEVEL(Level_Debug)) {
        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
        ss << "Initializing cache,  maxdistance " << _cachetree.GetMaxDistance() << ", collisionthresh " << _collisionthresh << ", _insertiondistance "<< _insertiondistance << ", weights [";
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

int ConfigurationCache::InsertConfiguration(const std::vector<dReal>& conf, CollisionReportPtr report, dReal distin)
{
    if( !!report->plink2 && report->plink2->GetParent() == _pstaterobot ) {
        std::swap(report->plink1, report->plink2);
    }
    //CacheTreeNodePtr vptr(new CacheTreeNode(conf, report));
    RAVELOG_VERBOSE_FORMAT("%s","about to insert");
    // tree is not initialized yet
    if (_cachetree.GetNumNodes() == 0) {
        _cachetree.InsertNode(conf, report, _insertiondistance);
        return 1;
    }
    else{

        // if distance to nearest has not been computed, calculate the distance
        dReal dist;
        if (distin > 0 ) {
            dist = distin;
        }
        else{
            std::pair<CacheTreeNodeConstPtr, dReal> knn = _cachetree.FindNearestNode(conf);
            dist = knn.second;
        }

        // check if configuration is at least _insertiondistance from the nearest node
        RAVELOG_VERBOSE_FORMAT("dist %e insertiondistance %e\n",dist%_insertiondistance);
        if( dist > _insertiondistance ) {

            if (IS_DEBUGLEVEL(Level_Verbose)) {
                stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                ss << "Inserted (InsertConfiguration) [ ";
                FOREACHC(itvalue, conf) {
                    ss << *itvalue << ", ";
                }
                ss << "]\n";
                RAVELOG_VERBOSE(ss.str());
            }

            if(_profile) {
                uint64_t start = utils::GetNanoPerformanceTime();
                _cachetree.InsertNode(conf, report, _insertiondistance);
                uint64_t end = utils::GetNanoPerformanceTime();
                _itime = end - start;
            }
            else {
                _cachetree.InsertNode(conf, report, _insertiondistance);
            }

            return 1;
        }
    }

    return 0;
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
    //CacheTreeNode testconf(conf, CollisionReportPtr());
    //CacheTreeNodePtr ptestconf(&testconf, utils::null_deleter());
    //std::list<std::pair<dReal, CacheTreeNodePtr> > knn;

    uint64_t start = utils::GetNanoPerformanceTime();
    // should also do radius search
    std::pair<CacheTreeNodeConstPtr, dReal> knn = _cachetree.FindNearestNode(conf);
    _qtime += utils::GetNanoPerformanceTime() - start;

    if( !knn.first ) {
        return -1;
    }

    dReal dist = knn.second;
    closestdist = dist;
    CacheTreeNodeConstPtr vfound = knn.first;
    //RAVELOG_VERBOSE_FORMAT("dist %e colllisionthresh %e\n",dist%_collisionthresh);

    if( vfound->IsInCollision() && dist <= _collisionthresh ) {
        robotlink = _pstaterobot->GetLinks().at(vfound->GetRobotLinkIndex());
        collidinglink = vfound->GetCollidingLink();
        return 1;
    }
    else if( !vfound->IsInCollision() && dist <= _freespacethresh ) {
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
