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
#include "plugindefs.h"
#include "configurationcachetree.h"

#include <sstream>

//High
//TODO extract collision reports from CheckPathAllConstraints
//TODO use cache for smoothing and planning
//TODO find a way to lose the map for children


//Low
//TODO add environmentstate to configurationcache
//TODO add circle data


//KinBodyPtr body;
//std::vector<dReal> values;
//std::vector<uint8_t> venablestates;
//body->GetConfigurationValues(values);
//body->GetLinkEnables(venablestates);

//std::map<KinBodyPtr, std::pair<std::vector<dReal>, std::vector<uint8_t> > > mapEnvironmentState;
namespace configurationcache {

CacheTreeVertex::CacheTreeVertex(const std::vector<dReal>& cs, CollisionReportPtr report)
{
    _report = report;
    _cstate = cs;
}

CacheTreeVertex::CacheTreeVertex(const std::vector<dReal>& cs, CollisionReportPtr report, const std::vector<Vector>& linkspheres)
{
    _report = report;
    _cstate = cs;
    _linkspheres = linkspheres;
}

CacheTree::CacheTree(const std::vector<dReal>& weights)
{
    _weights = weights;
    _numvertices = 0;
    _maxdistance = 1;
    _base = 2.0;
    _maxlevel = 1;
    _minlevel = 0;
    _numvertices = 0;

};

int CacheTree::Create(dReal maxdist, CacheTreeVertexPtr v){
    // check if root already exists, and simply insert the vertex if this is the case
    _base = 2.0; // this can be something else, i.e., 1.3, using two for now
    _root = v;
    _maxdistance = maxdist;
    _maxlevel = ceilf(RaveLog(_maxdistance)/RaveLog(_base));
    RAVELOG_DEBUG_FORMAT("maxlevel %e\n", _maxlevel);
    _minlevel = _maxlevel - 1;
    _numvertices = 1;
    return 1;
}

void CacheTree::Reset()
{
    BOOST_ASSERT(0);
}

dReal CacheTree::ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef){

    dReal distance = 0;

    for (size_t i = 0; i < cstatef.size(); ++i) {
        distance += RaveFabs(cstatei[i] - cstatef[i]) * _weights[i];
    }

    return distance;
}

int CacheTree::InsertVertex(CacheTreeVertexPtr vertexin){

    // todo: if there is no root, make this the root
    // otherwise call the lowlevel  insert
    std::vector<dReal> distin;
    std::vector<CacheTreeVertexPtr> verticesin;

    distin.push_back(ComputeDistance(_root->GetConfigurationState(), vertexin->GetConfigurationState()));
    verticesin.push_back(_root); //this is always the same, perhaps keep make this vector at construction?

    _Insert(vertexin, verticesin, distin, _maxlevel);

    return 1;
}

int CacheTree::_Insert(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& verticesin, const std::vector<dReal>& leveldists,  int level){

    if (level < _minlevel-1) {
        return 1;
    }

    // 2^{i}
    dReal qleveldistance = RavePow(_base, level);

    dReal minimumdistance = std::numeric_limits<float>::infinity();
    int minimumdistancev = 0;
    dReal localmindist = std::numeric_limits<float>::infinity();

    std::vector<dReal> nextleveldists;
    std::vector<CacheTreeVertexPtr> nextlevelvs;

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
            nextlevelvs.push_back(verticesin[i]);
        }

        // now consider children
        const std::vector<CacheTreeVertexPtr>& levelchildren = verticesin[i]->GetChildren(level);

        for (size_t j = 0; j < levelchildren.size(); ++j) {

            dReal curdist = ComputeDistance(in->GetConfigurationState(), levelchildren[j]->GetConfigurationState());

            if (curdist < localmindist) {
                localmindist = curdist;
            }

            if (curdist <= qleveldistance)
            {
                nextleveldists.push_back(curdist);
                nextlevelvs.push_back(levelchildren[j]);
            }
        }
    }

    if (localmindist > qleveldistance) {
        return 1;
    }
    else{

        int parent = _Insert(in, nextlevelvs, nextleveldists, level-1);

        if (parent == 1 && minimumdistance <= qleveldistance && minimumdistancev != -1) {
            if (level-1 < _minlevel) {
                _minlevel = level-1;
            }

            _numvertices++;
            CacheTreeVertexPtr vptr(new CacheTreeVertex(in->GetConfigurationState()));
            verticesin[minimumdistancev]->AddChildVertex(vptr, level);
            return 0;
        }
        else{
            return parent;
        }
    }
}


std::list<std::pair<dReal, CacheTreeVertexPtr> > CacheTree::GetNearestKVertices(CacheTreeVertexPtr in, int k)
{

    // todo: first check if there is no root

    // first localmax is distance from this vertex to the root
    dReal localmax = ComputeDistance(_root->GetConfigurationState(), in->GetConfigurationState());

    std::list< std::pair<dReal, CacheTreeVertexPtr> > listDistanceVertices;

    // create the first level with only the root
    std::vector<dReal> currentleveldists;
    std::vector<CacheTreeVertexPtr> currentlevelvertices;

    currentleveldists.push_back(localmax);
    currentlevelvertices.push_back(_root);

    dReal qbound = std::numeric_limits<float>::infinity();

    std::vector<dReal> nextleveldists;
    std::vector<CacheTreeVertexPtr> nextlevelvertices;

    // traverse all levels
    for (int qi = _maxlevel; qi >= _minlevel; --qi)
    {
        RAVELOG_VERBOSE_FORMAT("level %d qbound %e localmax %e\n", qi%qbound%localmax);

        // current level loop
        for (size_t i = 0; i < currentleveldists.size(); ++i) {

            // TODO, figure out how to use a single map stored in the tree here
            const std::vector<CacheTreeVertexPtr>& levelchildren = currentlevelvertices[i]->GetChildren(qi);

            // children at this level
            for (size_t j = 0; j < levelchildren.size(); ++j) {

                //bool bincollision = levelchildren[j]->IsInCollision();
                //RAVELOG_VERBOSE_FORMAT("child %d type %d in vertex type %d\n", j%bincollision%vertextype);

                // only look for vertices of a certain type, or consider all if vertextype is 0
//                if (vtype != 0 && levelchildren[j]->GetType()!=vtype) {
//                    continue;
//                }

                dReal curdist = ComputeDistance(in->GetConfigurationState(), levelchildren[j]->GetConfigurationState());

                if ((curdist < localmax) && (curdist > 0)) {

                    // if below localmax, insert into knn
                    listDistanceVertices.push_back(make_pair(curdist, levelchildren[j]));
                    if (curdist < localmax) {
                        localmax = curdist;
                    }

                    // if over k, delete the vertex with the largest distance (should be the first one)
                    if ((int)listDistanceVertices.size() > k) {
                        listDistanceVertices.pop_front();
                    }
                }

                // possibly consider this vertex, to be checked against bound later
                if (curdist < qbound) {
                    currentlevelvertices.push_back(levelchildren[j]);
                    currentleveldists.push_back(curdist);
                }
            }
        }

        // update distance bound
        qbound = localmax + RavePow(_base,qi); //d(p,q) + 2^{i}

        nextleveldists.clear();
        nextlevelvertices.clear();

        for (size_t q = 0; q < currentleveldists.size(); ++q) {
            if (currentleveldists[q] < qbound) {
                nextlevelvertices.push_back(currentlevelvertices[q]);
                nextleveldists.push_back(currentleveldists[q]);
            }
        }

        // next iteration only considers vertices below this bound
        currentleveldists.clear();
        currentlevelvertices.clear();

        currentleveldists = nextleveldists;
        currentlevelvertices = nextlevelvertices;

    }

    int ksize = listDistanceVertices.size();

    if (ksize > 0) {

        if (IS_DEBUGLEVEL(Level_Verbose)) {
            RAVELOG_VERBOSE_FORMAT("k size %d\n", ksize);
            stringstream ss;
            size_t index=0;
            FOREACH(itdistvertex, listDistanceVertices) {
                ss << "\nk" << index << "[";
                FOREACHC(itvalue, itdistvertex->second->GetConfigurationState()) {
                    ss << *itvalue << ", ";
                }
                ss << "] dist: " << itdistvertex->first << "\n";
            }

            RAVELOG_VERBOSE(ss.str());
        }

    }

    return listDistanceVertices;
}


ConfigurationCache::ConfigurationCache(RobotBasePtr probotstate)
{
    _probotstate = probotstate;
    _penv = probotstate->GetEnv();
    _vbodyindices = probotstate->GetActiveDOFIndices();
    _nBodyAffineDOF = probotstate->GetAffineDOF();
    _dim = probotstate->GetActiveDOF();

    probotstate->GetActiveDOFResolutions(_weights);

    // if weights are zero, used a default value
    FOREACH(itweight, _weights) {
        if( *itweight > 0 ) {
            *itweight = 1 / *itweight;
        }
        else {
            *itweight = 100;
        }
    }

    // set default values for collisionthresh and insertiondistance
    _collisionthresh = 1.0; //1.0; //minres; //smallest resolution
    _insertiondistance = 0.5; //0.5; //0.02;  //???

    _probotstate->GetActiveDOFLimits(_lowerlimit, _upperlimit);

    _jointchangehandle = probotstate->RegisterChangeCallback(KinBody::Prop_JointLimits, boost::bind(&ConfigurationCache::_UpdateJointLimits, this));

    // using L1, get the maximumdistance
    // distance has to be computed in the same way as CacheTreeVertex.GetDistance()
    // otherwise, distances larger than this value could be inserted into the tree
    dReal bound = 0;
    for (int i = 0; i < _dim; ++i)
    {
        bound += RaveFabs(_upperlimit[i] - _lowerlimit[i]) * _weights[i];
    }

    _maxdistance = bound;
    _collisioncount = 0;

    _qtime = 0;
    _itime = 0;
    _profile = false;

    _cachetree = CacheTree(_weights);

    if (IS_DEBUGLEVEL(Level_Debug)) {
        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
        ss << "Initializing cache,  maxdistance " << _maxdistance << ", collisionthresh " << _collisionthresh << ", _insertiondistance "<< _insertiondistance << ", weights [";
        for (int i = 0; i < _dim; ++i) {
            ss << _weights[i] << " ";
        }
        ss << "]\nupperlimit [";

        for (int i = 0; i < _dim; ++i) {
            ss << _upperlimit[i] << " ";
        }

        ss << "]\nlowerlimit [";

        for (int i = 0; i < _dim; ++i) {
            ss << _lowerlimit[i] << " ";
        }
        ss << "]\n";
        RAVELOG_DEBUG(ss.str());
    }
}

void ConfigurationCache::_UpdateJointLimits()
{
    _probotstate->SetActiveDOFs(_vbodyindices, _nBodyAffineDOF);
    _probotstate->GetActiveDOFLimits(_lowerlimit, _upperlimit);

    // compute new max distance for cache
    // using L1, get the maximumdistance
    // distance has to be computed in the same way as CacheTreeVertex.GetDistance()
    // otherwise, distances larger than this value could be inserted into the tree
    dReal bound = 0;
    for (int i = 0; i < _dim; ++i)
    {
        bound += RaveFabs(_upperlimit[i] - _lowerlimit[i]) * _weights[i];
    }

    if( bound > _maxdistance+g_fEpsilonLinear ) {
        // have to reset the tree
        _cachetree.Reset();
    }
    else {
        // can use as is
        _maxdistance = bound;
    }

}

void ConfigurationCache::SetWeights(const std::vector<dReal>& weights){
    _weights = weights;
}


int ConfigurationCache::InsertConfiguration(const std::vector<dReal>& conf, dReal dist){

    CacheTreeVertexPtr vptr(new CacheTreeVertex(conf));

    // tree is not initialized yet
    if (_cachetree.GetNumVertices() == 0) {

        // initialize tree with this conf and the maxdistance
        RAVELOG_VERBOSE_FORMAT("Initiliazed tree with maxdistance %e\n",_maxdistance);
        _cachetree.Create(_maxdistance, vptr);
        _collisioncount = _cachetree.GetNumVertices();

        return 1;
    }
    else{

        // if distance to nearest has not been computed, calculate the distance
        if (dist == -1) {
            dist = _GetNearestDist(conf);
        }

        // check if configuration is at least _insertiondistance from the nearest node
        RAVELOG_VERBOSE_FORMAT("dist %e insertiondistance %e\n",dist%_insertiondistance);
        if(dist > _insertiondistance || _cachetree.GetNumVertices()==1) {


            if(_profile) {
                uint64_t start = utils::GetNanoPerformanceTime();
                _cachetree.InsertVertex(vptr);
                uint64_t end = utils::GetNanoPerformanceTime();
                _itime = end - start;
            }
            else{
                _cachetree.InsertVertex(vptr);
            }
            _collisioncount = _cachetree.GetNumVertices();

            return 1;
        }

    }

    if (IS_DEBUGLEVEL(Level_Verbose)) {
        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
        ss << "Inserted (InsertConfiguration) [ ";
        FOREACHC(itvalue, conf) {
            ss << *itvalue << ", ";
        }
        ss << "]\n";
        RAVELOG_VERBOSE(ss.str());
    }

    return 0;
}

int ConfigurationCache::CheckCollision(const std::vector<dReal>& conf){

    dReal dist;
    if (_profile) {
        uint64_t start = utils::GetNanoPerformanceTime();
        dist = _GetNearestDist(conf);
        uint64_t end = utils::GetNanoPerformanceTime();
        _qtime += end - start;
    }
    else{
        dist = _GetNearestDist(conf);
    }

    RAVELOG_VERBOSE_FORMAT("dist %e colllisionthresh %e\n",dist%_collisionthresh);

    if(dist < _collisionthresh && dist > 0) {
        //configuration assumed to be in collision, no checking needed
        return 1;
    }

    return -1;
    //todo_ if very close to feasible tree, return 0
}

dReal ConfigurationCache::_GetNearestDist(const std::vector<dReal>& cs) {

    CacheTreeVertexPtr vptr(new CacheTreeVertex(cs));
    // get 1 knn
    std::list<std::pair<dReal, CacheTreeVertexPtr> > knn = _cachetree.GetNearestKVertices(vptr, 1);

    return knn.front().first;
}

std::list<std::pair<dReal, CacheTreeVertexPtr> > ConfigurationCache::GetNearestKVertices(CacheTreeVertexPtr in, int k)
{
    return _cachetree.GetNearestKVertices(in, k);

}

}
