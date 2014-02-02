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

CacheTreeVertex::CacheTreeVertex(const std::vector<dReal>& cs)//, KinBody::LinkWeakPtr collidinglink)
{
    _cstate = cs;
    _approxdispersion.first = CacheTreeVertexPtr();
    _approxdispersion.second = std::numeric_limits<float>::infinity();
    _approxnn.first = CacheTreeVertexPtr();
    _approxnn.second = std::numeric_limits<float>::infinity();
    _conftype = CT_Unknown;

}

void CacheTreeVertex::SetCollisionInfo(CollisionReportPtr report){
    if (!!report){
        if (report->numCols > 0){
            _collidinglinktrans = report->plink1->GetTransform();
            _collidinglink = report->plink1;
            _conftype = CT_Collision;
        }
        if (report->numCols == 0){
            _conftype = CT_Free;
        }

    }
    else{
        _conftype = CT_Free;
    }

}

void CacheTreeVertex::UpdateApproximates(dReal distance, CacheTreeVertexPtr v){

    // if both are same type, update nn
    if (v->GetConfigurationType() == _conftype){
        if (distance < _approxnn.second){
            _approxnn.first = v;
            _approxnn.second = distance;
        }

    }
    // if they are different types, update dispersion
    else{
        if ( distance < _approxdispersion.second){
            _approxdispersion.first = v;
            //dReal oldd = _approxdispersion.second;
            //RAVELOG_DEBUG_FORMAT("dispersion %f to %f\n",oldd%distance);
            _approxdispersion.second = distance;
        }
    }

}

void CacheTreeVertex::RemoveChild(CacheTreeVertexPtr child, int level){

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

CacheTree::CacheTree(const std::vector<dReal>& weights)
{
    _weights = weights;
    _numvertices = 0;
    _maxdistance = 1;
    _base = 2.0;
    _maxlevel = 1;
    _minlevel = 0;
    _numvertices = 0;
    _insertiondistance = 0;

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

dReal CacheTree::ComputeDistance(CacheTreeVertexPtr vi, CacheTreeVertexPtr vf){

    dReal distance = _ComputeDistance(vi->GetConfigurationState(), vf->GetConfigurationState());

    // use this distance information to update the upper bounds stored on the vertex
    vi->UpdateApproximates(distance,vf);
    vf->UpdateApproximates(distance,vi);
    
    return distance;
}

dReal CacheTree::_ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef){

    dReal distance = 0;

    for (size_t i = 0; i < cstatef.size(); ++i) {
        distance += RaveFabs(cstatei[i] - cstatef[i]) * _weights[i];
    }

    return distance;
}

int CacheTree::InsertVertex(CacheTreeVertexPtr vertexin){

    // todo: if there is no root, make this the root
    // otherwise call the lowlevel  insert

    std::vector<CacheTreeVertexPtr> verticesin;
    std::vector<dReal> distin;

    distin.push_back(ComputeDistance(_root, vertexin));
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

int CacheTree::RemoveVertex(CacheTreeVertexPtr vertexin){

    // todo: if there is no root, make this the root
    // otherwise call the lowlevel  insert

    std::vector<CacheTreeVertexPtr> verticesin;
    std::vector<dReal> distin;

    distin.push_back(ComputeDistance(_root, vertexin));
    verticesin.push_back(_root); //this is always the same, perhaps keep make this vector at construction?

    _Remove(vertexin, verticesin, distin, _maxlevel);

    return 1;
}

int CacheTree::_Remove(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& verticesin, const std::vector<dReal>& leveldists,  int level){

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
                    if (levelchildren[j]->GetConfigurationType() != conftype){ 
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

void CacheTree::UpdateTree(){

    // TODO: take in information regarding what changed in the environment and update vertices accordingly (i.e., change vertices no longer known to be in collision to CT_Unknown, remove vertices in collision with body no longer in the environment, check enclosing spheres for links to see if they overlap with new bodies in the scene, etc.) Note that parent/child relations must be updated as described in Beygelzimer, et al. 2006. 

}

ConfigurationCache::ConfigurationCache(RobotBasePtr probotstate)
{
    _probotstate = probotstate;
    _penv = probotstate->GetEnv();

    _probotstate->GetAttached(_vattachedbodies);
    FOREACHC(itgrab, _vattachedbodies){
        std::string sg = (*itgrab)->GetName();
        RAVELOG_WARN_FORMAT("%s is attached\n",sg);
    }

    _penv->GetBodies(_vnewenvbodies);
    FOREACHC(itbody, _vnewenvbodies){

        if(!((*itbody)->IsRobot() || (*itbody)->IsAttached(_probotstate))){
            _menvbodyhandles[(*itbody)] = (*itbody)->RegisterChangeCallback(KinBody::Prop_LinkGeometry|KinBody::Prop_LinkEnable , boost::bind(&ConfigurationCache::_UpdateBodies, this));
        }
        std::string name = (*itbody)->GetName();
        bool enabled = (*itbody)->IsEnabled();
        RAVELOG_DEBUG_FORMAT("sync %s enabled=%d\n",name%enabled);
        _menvbodystamps[(*itbody)] = (*itbody)->GetUpdateStamp();
        _menvbodyenabled[(*itbody)] = (*itbody)->IsEnabled();
        _menvbodyattached[(*itbody)] = (*itbody)->IsAttached(_probotstate);
        
    }

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
void ConfigurationCache::_UpdateBodies()
{
    SynchronizeAll(_lockedbody);
    //SetLockedBody(KinBodyConstPtr());
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


int ConfigurationCache::InsertConfiguration(const std::vector<dReal>& conf, CollisionReportPtr report, dReal distin){

    // sync before insertion
    //SynchronizeAll();

    CacheTreeVertexPtr vptr(new CacheTreeVertex(conf));
    vptr->SetCollisionInfo(report);

    RAVELOG_VERBOSE_FORMAT("%s","about to insert");
    // tree is not initialized yet
    if (_cachetree.GetNumVertices() == 0) {

        // initialize tree with this conf and the maxdistance
        RAVELOG_VERBOSE_FORMAT("Initiliazed tree with maxdistance %e\n",_maxdistance);
        _cachetree.Create(_maxdistance, vptr);
        _cachetree.SetInsertionDistance(_insertiondistance);
        _collisioncount = _cachetree.GetNumVertices();

        return 1;
    }
    else{

        // if distance to nearest has not been computed, calculate the distance
        dReal dist;
        if (distin != -1) {
            dist = distin;
        }
        else{
            dist = _GetNearestDist(conf);
        }

        // check if configuration is at least _insertiondistance from the nearest node
        RAVELOG_VERBOSE_FORMAT("dist %e insertiondistance %e\n",dist%_insertiondistance);
        if((dist > _insertiondistance || _cachetree.GetNumVertices()==1)) {

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
            else{
                _cachetree.InsertVertex(vptr);
            }
            _collisioncount = _cachetree.GetNumVertices();

            return 1;
        }

    }

    return 0;
}

void ConfigurationCache::CheckCollision(const std::vector<dReal>& conf, std::pair<int,dReal>& result){


    dReal dist;
    int res;

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
        res = 1;
    }
    else{
        res = 0;
    }

    result.first = res;
    result.second = dist;
    //todo_ if very close to feasible tree, return 0
}

dReal ConfigurationCache::_GetNearestDist(const std::vector<dReal>& cs) {

    CacheTreeVertexPtr vptr(new CacheTreeVertex(cs));
    // get 1 knn
    std::list<std::pair<dReal, CacheTreeVertexPtr> > knn = _cachetree.GetNearestKVertices(vptr, 1);

    if (knn.size() < 1){
        return -1;
    }

    return knn.front().first;
}

std::list<std::pair<dReal, CacheTreeVertexPtr> > ConfigurationCache::GetNearestKVertices(CacheTreeVertexPtr in, int k, ConfigurationType conftype)
{
    // sync before the query
    return _cachetree.GetNearestKVertices(in, k, conftype);

}

int ConfigurationCache::SynchronizeAll(KinBodyConstPtr pbody){

    /*if(!pbody){
        return 0;
    }*/
    if(!!pbody){
        std::string locked  = pbody->GetName();
        RAVELOG_WARN_FORMAT("call %s", locked);
    }
    _penv->GetBodies(_vnewenvbodies);

    // replace with loop that uses GetBodyFromEnvironmentId for all stored ids
    if (_vnewenvbodies.size() < _menvbodystamps.size()){
        //body was removed

        // find which one(s) 
        typedef std::map<KinBodyPtr, int>::iterator mapit;
        for(mapit iterator = _menvbodystamps.begin(); iterator != _menvbodystamps.end(); iterator++){
            
            // new bodies
            bool found = false;
            std::map<KinBodyPtr, int>::iterator it;
            FOREACHC(itbody, _vnewenvbodies){

                if (iterator->first->GetEnvironmentId() == (*itbody)->GetEnvironmentId()){
                    found = true;
                    break;
                }

            }
            if (!found){
                std::string missing = iterator->first->GetName();
                RAVELOG_WARN_FORMAT("%s was removed\n", missing);
                
                //update cache accordingly

                //remove from map of current bodies
                _menvbodystamps.erase(iterator->first);
                _menvbodyenabled.erase(iterator->first);
                _menvbodyattached.erase(iterator->first);
                _menvbodyhandles.erase(iterator->first);
            }

        }

    }

    std::map<KinBodyPtr, int>::iterator it;
    FOREACHC(itbody, _vnewenvbodies){

        // synchronize everything except for the body currently being collision checked (as its state will be changed by the collision checker)
        if(!!pbody){
            if ((*itbody)->GetEnvironmentId() == pbody->GetEnvironmentId() || (*itbody)->IsAttached(pbody)){
                continue;
            }
        }

        it = _menvbodystamps.find(*itbody);

        if (IS_DEBUGLEVEL(Level_Verbose)){
            int oldst = it->second;
            int st = (*itbody)->GetUpdateStamp();
            std::string nm = (*itbody)->GetName();
            RAVELOG_VERBOSE_FORMAT("%s checking stamp %d/%d",nm%oldst%st);
        }

        if (it == _menvbodystamps.end()) {
            //new body
            //update accordingly

            std::string name = (*itbody)->GetName();
            bool enabled = (*itbody)->IsEnabled();
            bool attached = (*itbody)->IsAttached(_probotstate);
            RAVELOG_WARN_FORMAT("new body %s enabled = %d attached to robot = %d",name%enabled%attached);
            //add to map
            _menvbodystamps[(*itbody)] = (*itbody)->GetUpdateStamp();
            _menvbodyenabled[(*itbody)] = (*itbody)->IsEnabled();
            _menvbodyattached[(*itbody)] = (*itbody)->IsAttached(_probotstate);
            _menvbodyhandles[(*itbody)] = (*itbody)->RegisterChangeCallback(KinBody::Prop_LinkGeometry|KinBody::Prop_LinkEnable , boost::bind(&ConfigurationCache::_UpdateBodies, this));
            // invalidate cache
        }
        else{
            // if body was found, check if update stamp has changed
            if(it->second != (*itbody)->GetUpdateStamp())
            {
                //body changed

                int oldst = it->second;
                int st = (*itbody)->GetUpdateStamp();
                std::string nm = (*itbody)->GetName();
                RAVELOG_WARN_FORMAT("%s change in update stamp %d/%d",nm%oldst%st);
                //update accordingly
                _menvbodystamps[(*itbody)] = (*itbody)->GetUpdateStamp();
                _menvbodyenabled[(*itbody)] = (*itbody)->IsEnabled();
                _menvbodyattached[(*itbody)] = (*itbody)->IsAttached(_probotstate);
                // if not robot and not grabbed bodies, invalidate cache
            }
        }
    }

    return 0;

}

}
