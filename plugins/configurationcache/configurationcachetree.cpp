// -*- coding: utf-8 -*-
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

CacheTreeVertex::CacheTreeVertex(const std::vector<dReal>& cs, CollisionReportPtr report)
{
    _cstate = cs;
    _approxdispersion.first = CacheTreeVertexPtr();
    _approxdispersion.second = std::numeric_limits<float>::infinity();
    _approxnn.first = CacheTreeVertexPtr();
    _approxnn.second = std::numeric_limits<float>::infinity();
    _conftype = CT_Unknown;
    _robotlinkindex = -1;
    SetCollisionInfo(report);
}

void CacheTreeVertex::SetCollisionInfo(CollisionReportPtr report)
{
    if(!!report && report->numCols > 0) {
        _collidinglinktrans = report->plink1->GetTransform();
        _robotlinkindex = report->plink1->GetIndex();
        _collidinglink = report->plink2;
        _conftype = CT_Collision;
    }
    _conftype = CT_Free;
    _collidinglink.reset();
    _robotlinkindex = -1;
}

void CacheTreeVertex::UpdateApproximates(dReal distance, CacheTreeVertexPtr v)
{
    // if both are same type, update nn
    if (v->GetConfigurationType() == _conftype) {
        if (distance < _approxnn.second) {
            _approxnn.first = v;
            _approxnn.second = distance;
        }

    }
    // if they are different types, update dispersion
    else{
        if ( distance < _approxdispersion.second) {
            _approxdispersion.first = v;
            //dReal oldd = _approxdispersion.second;
            //RAVELOG_DEBUG_FORMAT("dispersion %f to %f\n",oldd%distance);
            _approxdispersion.second = distance;
        }
    }
}

void CacheTreeVertex::RemoveChild(CacheTreeVertexPtr child, int level)
{
    std::vector<CacheTreeVertexPtr>:: iterator it = _children[level].begin();
    for (; it != _children[level].end(); ) {
        if (*it == child) {
            it = _children[level].erase(it);
        }
        else{
            ++it;
        }
    }
}

CacheTree::CacheTree(const std::vector<dReal>& weights, dReal maxdistance)
{
    _weights = weights;
    _numvertices = 0;
    _base = 2.0;
    _maxdistance = maxdistance;
    _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
    _minlevel = _maxlevel - 1;
    _numvertices = 0;
    _insertiondistance = 0;
}

void CacheTree::Reset()
{
    BOOST_ASSERT(0);
}

dReal CacheTree::ComputeDistance(CacheTreeVertexPtr vi, CacheTreeVertexPtr vf)
{
    dReal distance = _ComputeDistance(vi->GetConfigurationState(), vf->GetConfigurationState());

    // use this distance information to update the upper bounds stored on the vertex
    vi->UpdateApproximates(distance,vf);
    vf->UpdateApproximates(distance,vi);

    return distance;
}

dReal CacheTree::_ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef)
{
    dReal distance = 0;
    for (size_t i = 0; i < cstatef.size(); ++i) {
        distance += RaveFabs(cstatei[i] - cstatef[i]) * _weights[i];
    }
    return distance;
}

int CacheTree::InsertVertex(CacheTreeVertexPtr vertexin)
{
    // todo: if there is no root, make this the root
    // otherwise call the lowlevel  insert
    if( !_root ) {
        _root = vertexin;
        return 1;
    }
    else {
        std::vector<CacheTreeVertexPtr> verticesin;
        std::vector<dReal> distin;

        distin.push_back(ComputeDistance(_root, vertexin));
        verticesin.push_back(_root); //this is always the same, perhaps keep make this vector at construction?

        return _Insert(vertexin, verticesin, distin, _maxlevel);
    }
}

int CacheTree::_Insert(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& verticesin, const std::vector<dReal>& leveldists,  int level)
{
    if (level < _minlevel-1) {
        return 1;
    }

    // 2^{i}
    dReal qleveldistance = RavePow(_base, level);

    dReal minimumdistance = std::numeric_limits<float>::infinity();
    int minimumdistancev = 0;
    dReal localmindist = std::numeric_limits<float>::infinity();

    std::vector<CacheTreeVertexPtr> nextlevelvertices;
    std::vector<dReal> nextleveldists;

    for (size_t i = 0; i < verticesin.size(); ++i) {
        if (leveldists[i] < minimumdistance) {
            minimumdistance = leveldists[i];
            minimumdistancev = i;
        }

        if (leveldists[i] < localmindist) {
            localmindist = leveldists[i];
        }

        if (leveldists[i] <= qleveldistance) {
            nextleveldists.push_back(leveldists[i]);
            nextlevelvertices.push_back(verticesin[i]);
        }

        // now consider children
        const std::vector<CacheTreeVertexPtr>& levelchildren = verticesin[i]->GetChildren(level);

        int cz = levelchildren.size();
        RAVELOG_VERBOSE_FORMAT("%d children at level %d\n",cz%level);
        for (size_t j = 0; j < levelchildren.size(); ++j) {

            dReal curdist = ComputeDistance(in, levelchildren[j]);

            if (curdist < localmindist) {
                localmindist = curdist;
            }

            if (curdist <= qleveldistance)
            {
                nextleveldists.push_back(curdist);
                nextlevelvertices.push_back(levelchildren[j]);
            }
        }
    }

    if (localmindist > qleveldistance) {
        return 1;
    }
    else{

        int parent = _Insert(in, nextlevelvertices, nextleveldists, level-1);

        if ((parent == 1) && (minimumdistance <= qleveldistance) && (minimumdistancev != -1)) {
            if (level-1 < _minlevel) {
                _minlevel = level-1;
            }

            _numvertices++;
            verticesin[minimumdistancev]->AddChildVertex(in, level);
            return 0;
        }
        else{
            return parent;
        }
    }
}

int CacheTree::RemoveVertex(CacheTreeVertexPtr vertexin)
{
    // todo: if there is no root, make this the root
    // otherwise call the lowlevel  insert

    std::vector<CacheTreeVertexPtr> verticesin;
    std::vector<dReal> distin;

    distin.push_back(ComputeDistance(_root, vertexin));
    verticesin.push_back(_root); //this is always the same, perhaps keep make this vector at construction?

    _Remove(vertexin, verticesin, distin, _maxlevel);

    return 1;
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
}

int CacheTree::_Remove(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& verticesin, const std::vector<dReal>& leveldists,  int level)
{
    // 2^{i}
    /*dReal qleveldistance = RavePow(_base, level);

       dReal minimumdistance = std::numeric_limits<float>::infinity();
       CacheTreeVertexPtr minimumdistancev;
       dReal localmindist = std::numeric_limits<float>::infinity();

       std::vector<CacheTreeVertexPtr> nextlevelvertices;
       std::vector<dReal> nextleveldists;
       CacheTreeVertexPtr parent;

       for (size_t i = 0; i < verticesin.size(); ++i) {

        if (leveldists[i] < minimumdistance) {
            minimumdistance = leveldists[i];
            minimumdistancev = verticesin[i];
        }

        if (leveldists[i] <= qleveldistance) {
            nextleveldists.push_back(leveldists[i]);
            nextlevelvertices.push_back(verticesin[i]);
        }

        // now consider children
        const std::vector<CacheTreeVertexPtr>& levelchildren = verticesin[i]->GetChildren(level);

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
                nextlevelvertices.push_back(levelchildren[j]);
            }
        }
       }

       if (level > _minlevel) {
        _Remove(in, nextlevelvertices, nextleveldists, level-1);
       }

       if (!!parent) {
        //parent->removeChild(level, minimumdistancev);
       }

       const std::vector<CacheTreeVertexPtr>& minchildren = minimumdistancev->GetChildren(level-1);*/

    return 0;
}

std::list<std::pair<dReal, CacheTreeVertexPtr> > CacheTree::GetNearestKVertices(CacheTreeVertexPtr in, int k, ConfigurationType conftype)
{
    // first localmax is distance from this vertex to the root
    dReal localmax = ComputeDistance(_root, in);

    // create the first level with only the root

    _listDistanceVertices.resize(0);
    _nextleveldists.resize(0);
    _nextlevelvertices.resize(0);
    _currentleveldists.resize(0);
    _currentlevelvertices.resize(0);
    _currentleveldists.push_back(localmax);
    _currentlevelvertices.push_back(_root);

    dReal qbound = std::numeric_limits<float>::infinity();

    // traverse all levels
    for (int qi = _maxlevel; qi >= _minlevel; --qi)
    {
        RAVELOG_VERBOSE_FORMAT("level %d qbound %e localmax %e _maxlevel %d _minlevel %d vertices %d\n", qi%qbound%localmax%_maxlevel%_minlevel%_numvertices);

        // current level loop
        for (size_t i = 0; i < _currentleveldists.size(); ++i) {

            const std::vector<CacheTreeVertexPtr>& levelchildren = _currentlevelvertices[i]->GetChildren(qi);

            // children at this level
            for (size_t j = 0; j < levelchildren.size(); ++j) {


                dReal curdist = ComputeDistance(in, levelchildren[j]);


                if ((curdist < localmax) && (curdist > 0)) {

                    // only return neighbors of the same time, check all if type is unknown
                    if (levelchildren[j]->GetConfigurationType() != conftype) {
                        continue;
                    }
                    // if below localmax, insert into knn
                    _listDistanceVertices.push_back(make_pair(curdist, levelchildren[j]));
                    if (curdist < localmax) {
                        localmax = curdist;
                    }

                    // if over k, delete the vertex with the largest distance (should be the first one)
                    if ((int)_listDistanceVertices.size() > k) {
                        _listDistanceVertices.pop_front();
                    }
                }

                // possibly consider this vertex, to be checked against bound later
                if (curdist < qbound) {
                    _currentlevelvertices.push_back(levelchildren[j]);
                    _currentleveldists.push_back(curdist);
                }
            }
        }

        // update distance bound
        qbound = localmax + RavePow(_base,qi); //d(p,q) + 2^{i}


        for (size_t q = 0; q < _currentleveldists.size(); ++q) {
            if (_currentleveldists[q] < qbound) {
                _nextlevelvertices.push_back(_currentlevelvertices[q]);
                _nextleveldists.push_back(_currentleveldists[q]);
            }
        }

        // next iteration only considers vertices below this bound
        _currentleveldists.resize(0);
        _currentlevelvertices.resize(0);

        _currentleveldists = _nextleveldists;
        _currentlevelvertices = _nextlevelvertices;

        _nextleveldists.resize(0);
        _nextlevelvertices.resize(0);

    }

    int ksize = _listDistanceVertices.size();

    if (ksize > 0) {

        if (IS_DEBUGLEVEL(Level_Verbose)) {
            RAVELOG_VERBOSE_FORMAT("k size %d\n", ksize);
            stringstream ss;
            size_t index = 0;
            FOREACH(itdistvertex, _listDistanceVertices) {
                ss << "\nk" << index << "[";
                FOREACHC(itvalue, itdistvertex->second->GetConfigurationState()) {
                    ss << *itvalue << ", ";
                }
                ss << "] dist: " << itdistvertex->first << "\n";
            }

            RAVELOG_VERBOSE(ss.str());
        }

    }

    return _listDistanceVertices;
}

void CacheTree::UpdateTree()
{
 

}

ConfigurationCache::ConfigurationCache(RobotBasePtr pstaterobot)
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
    // distance has to be computed in the same way as CacheTreeVertex.GetDistance()
    // otherwise, distances larger than this value could be inserted into the tree
    dReal bound = 0;
    for (size_t i = 0; i < _lowerlimit.size(); ++i) {
        bound += RaveFabs(_upperlimit[i] - _lowerlimit[i]) * vweights[i];
    }

    _cachetree = CacheTree(vweights, bound);
    _cachetree.SetInsertionDistance(_insertiondistance);

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
    CacheTreeVertexPtr vptr(new CacheTreeVertex(conf, report));
    RAVELOG_VERBOSE_FORMAT("%s","about to insert");
    // tree is not initialized yet
    if (_cachetree.GetNumVertices() == 0) {
        _cachetree.InsertVertex(vptr);
        return 1;
    }
    else{

        // if distance to nearest has not been computed, calculate the distance
        dReal dist;
        if (distin > 0 ) {
            dist = distin;
        }
        else{
            std::list<std::pair<dReal, CacheTreeVertexPtr> > knn = _cachetree.GetNearestKVertices(vptr, 1);
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
                _cachetree.InsertVertex(vptr);
                uint64_t end = utils::GetNanoPerformanceTime();
                _itime = end - start;
            }
            else {
                _cachetree.InsertVertex(vptr);
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
    if( _cachetree.GetNumVertices() == 0 ) {
        return -1;
    }
    std::vector<dReal> conf;
    GetDOFValues(conf);
    CacheTreeVertex testconf(conf, CollisionReportPtr());
    CacheTreeVertexPtr ptestconf(&testconf, utils::null_deleter());
    std::list<std::pair<dReal, CacheTreeVertexPtr> > knn;

    uint64_t start = utils::GetNanoPerformanceTime();
    // should also do radius search
    knn = _cachetree.GetNearestKVertices(ptestconf, 1);
    _qtime += utils::GetNanoPerformanceTime() - start;

    if( knn.size() == 0 ) {
        return -1;
    }

    dReal dist = knn.front().first;
    closestdist = dist;
    CacheTreeVertexPtr vfound = knn.front().second;
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

std::list<std::pair<dReal, CacheTreeVertexPtr> > ConfigurationCache::GetNearestKVertices(CacheTreeVertexPtr in, int k, ConfigurationType conftype)
{
    // sync before the query
    return _cachetree.GetNearestKVertices(in, k, conftype);

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
    // distance has to be computed in the same way as CacheTreeVertex.GetDistance()
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
