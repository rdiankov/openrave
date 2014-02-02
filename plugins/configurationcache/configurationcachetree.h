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
#ifndef OPENRAVE_CACHETREE_H
#define OPENRAVE_CACHETREE_H

#include <openrave/openrave.h>


namespace configurationcache {

using namespace OpenRAVE;

enum ConfigurationType{
    CT_Collision = -1,
    CT_Free = 1,
    CT_Unknown = 0
};

class CacheTreeVertex
{
    typedef boost::shared_ptr<CacheTreeVertex> CacheTreeVertexPtr;

public:
    CacheTreeVertex(const std::vector<dReal>& cs);

    void AddChildVertex(CacheTreeVertexPtr child, int level){
        _children[level].push_back(child);
    }

    const std::vector<CacheTreeVertexPtr>& GetChildren(int level) {
        return _children[level];
    }

    void RemoveChild(CacheTreeVertexPtr child, int level);

    const std::vector<dReal>& GetConfigurationState() const {
        return _cstate;
    }

    void SetCollisionInfo(CollisionReportPtr report);

    inline KinBody::LinkConstPtr GetCollidingLink() {
        return _collidinglink;
    }

    bool IsInCollision(){
        return !!_collidinglink;
    }

    void ResetCollisionInfo(){
        _collidinglink = KinBody::LinkConstPtr();
        _collidinglinktrans = Transform();
    }

    ConfigurationType GetConfigurationType(){
        return _conftype;
    }

    // returns closest distance to a configuration of the opposite type seen so far
    dReal GetUpperBound(){
        return _approxdispersion.second;
    }

    // returns the closest neighbor seen so far
    CacheTreeVertexPtr GetApproxNN(){
        return _approxnn.first;
    }

    void UpdateApproximates(dReal distance, CacheTreeVertexPtr v);

private:
    std::map<int,std::vector<CacheTreeVertexPtr> > _children; //maybe use a vector for each level somehow
    std::vector<Vector> _linkspheres; ///< xyz is center, w is radius^2
    std::vector<dReal> _cstate;

    ConfigurationType _conftype; 
    KinBody::LinkConstPtr _collidinglink;
    Transform _collidinglinktrans;
    std::pair<CacheTreeVertexPtr, dReal> _approxdispersion; //minimum distance and configuration of the opposite type seen so far
    std::pair<CacheTreeVertexPtr, dReal> _approxnn; //nearest distance and neighbor seen so far (same type)

    // todo, keep k nearest neighbors and update k every now and then, k = (e + e/dim) * log(n+1) where n is the size of the tree
};

typedef boost::shared_ptr<CacheTreeVertex> CacheTreeVertexPtr;

class CacheTree
{
    // Cache stores configuration information in a data structure based on the Cover Tree (Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf) 
    // The tree contains vertices with configurations, collision/free-space information, distance/nn statistics (e.g., dispersion, upper bounds on minimum distance to collisions, and admissible nearest neighbor), collision reports, etc. To be expanded to include a lean workspace representation for each vertex, i.e., enclosing spheres for each link, and an approximation of a connected graph (there is a path from every configuration to every other configuration, possible by considering log(n) neighbors) that is constructed from collision checking procedures (of the form qi to qf) and can be used to attempt to plan with the cache before sampling new configurations. 
public:

    CacheTree(){
    };
    CacheTree(const std::vector<dReal>& weights);
    // d(p,q) < (1 + e)d(p,S)
    // 2^(1+i) (1 + 1/e) <= d(p,Qi)

    /// \brief traverse all levels, vertices, and children and remove everything
    virtual ~CacheTree(){
    };

    int Create(dReal maxdistance, CacheTreeVertexPtr v);

    /// \brief resets the nodes for the cache tree
    void Reset();

    dReal ComputeDistance(CacheTreeVertexPtr vi, CacheTreeVertexPtr vf);
    /// \brief finds the k nearest neighbors in the cover tree
    /// \param[in] the vertex to find neighbors for
    /// \param[in] the number of neighbors
    /// \param[in] the type of neighbors, CT_Collision, CT_Free, or CT_Unknown.
    /// \param[in] option to only return vertices with a parent
    /// \param[out] pair with k nearest distances and vertices
    std::list<std::pair<dReal, CacheTreeVertexPtr> > GetNearestKVertices(CacheTreeVertexPtr in, int k, ConfigurationType conftype = CT_Collision);

    /// \brief insert vertex into cover. Vertex is made the child of different parents in the cover sets at the existing levels that meet the criteria. 
    int InsertVertex(CacheTreeVertexPtr in);
    int RemoveVertex(CacheTreeVertexPtr in);

    /// \brief distance a configuration must have from the nearest configuration in the tree in order for it be inserted
    void SetInsertionDistance(dReal dist){
        _insertiondistance = dist;
    }

    /// \brief number of vertices in the tree; todo: also count vertices by type
    int GetNumVertices(){
        return _numvertices;
    }

    /// \brief todo: update the tree when the environment changes 
    void UpdateTree();

private:
    std::vector<dReal> _weights; ///< weights used by the distance function

    // cache cache
    std::vector<dReal> _nextleveldists;
    std::vector<CacheTreeVertexPtr> _nextlevelvertices;
    std::vector<dReal> _currentleveldists;
    std::vector<CacheTreeVertexPtr> _currentlevelvertices;
    std::vector<CacheTreeVertexPtr> _levelchildren;
    std::list< std::pair<dReal, CacheTreeVertexPtr> > _listDistanceVertices;
    
    dReal _maxdistance; ///< maximum possible distance between two states. used to balance the tree.
    dReal _base; ///< a constant used to control the max level of traversion
    dReal _insertiondistance; ///< distance a configuration must have from the nearest configuration in the tree of the same type in order for it to be inserted

    int _maxlevel; ///< the maximum allowed levels in the tree
    int _minlevel; ///< the minimum allowed levels in the tree
    int _numvertices; ///< the number of vertices in the current tree starting at _root

    CacheTreeVertexPtr _root; // root vertex

    /// \brief takes in the configurations of two vertices and returns the distance, currently using an L1 weighted metric 
    dReal _ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef);
    /// \param inserts a configuration into the cache tree
    ///
    /// \param[in] in the input vertex to insert
    /// \param[in] vs the tree vertices at a particular level
    /// \param[in] leveldists the distances of the vs nodes
    /// \param[in] the current level traversing
    int _Insert(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& vs, const std::vector<dReal>& leveldists, int level);

    int _Remove(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& vs, const std::vector<dReal>& leveldists, int level);
};

typedef boost::shared_ptr<CacheTree> CacheVertexTreePtr;

class ConfigurationCache
{

public:
    /// \brief start tracking the active DOFs of the robot
    ConfigurationCache(RobotBasePtr probotstate);

    /// \brief insert a configuration into the cache
    /// function verifies if the configuration is at least _insertiondistance
    /// from the closest node in the cache
    /// \param cs, configuration
    /// \param ndists, a vector with the distances from the nearest nodes to cs
    /// returns 1 if configuration was inserted and 0 otherwise
    /// \param indist, nearest distance for this configuration; optional parameter if it has already been computed
    int InsertConfiguration(const std::vector<dReal>& cs, CollisionReportPtr report = CollisionReportPtr(), dReal distin = -1);

    /// \brief determine if configuration is whithin threshold of a collision in the cache (_collisionthresh), known to be in collision, or requires an explicit collision check
    void CheckCollision(const std::vector<dReal>& conf, std::pair<int,dReal>& result);

    /// \brief number of nodes currently in the cover tree
    int GetSize(){
        return _cachetree.GetNumVertices();
    }

    /// \brief the cache will assume a new configuration is in collision if the nearest node in the tree is below this distance
    void SetCollisionThresh(dReal colthresh)
    {
        colthresh = _collisionthresh;
    }

    void SetWeights(const std::vector<dReal>& weights);
    /// \brief the cache will not insert configuration if their distance from the nearest node in the tree is not larger than this value
    void SetInsertionDistance(dReal indist)
    {
        indist = _insertiondistance;
    }

    RobotBasePtr GetRobot(){
        return _probotstate;
    }

    /// \brief invalidate the entire cache if necessary
    //void InvalidateCache(){}

    /// \brief prune certain vertices in the cache tree, possibly takes in link, kinbody or environment as a parameter
    ///void PruneCache(){}

    std::list<std::pair<dReal, CacheTreeVertexPtr> > GetNearestKVertices(CacheTreeVertexPtr in, int k, ConfigurationType conftype = CT_Collision); //needs to be tested, possibly changed
    virtual ~ConfigurationCache(){
    }

    int SynchronizeAll(KinBodyConstPtr pbody = KinBodyConstPtr());
    void SetLockedBody(KinBodyConstPtr pbody = KinBodyConstPtr()){
        _lockedbody = pbody;
    }

private:
    /// \brief get the distance to the nearest vertex in the cover tree
    /// \param cs, configuration
    /// \param vertextype, nearest neighbor of what type (1 free, -1 collision, 0 search all)

    CacheTree _cachetree; //collisions

    std::vector<dReal> _weights; //weights for each joint, inverse of joint resolutions
    std::vector<dReal> _upperlimit, _lowerlimit; //joint limits, used to calculate maxdistance
    std::vector<int> _vbodyindices;
    std::map<KinBodyPtr, int> _menvbodystamps;
    std::map<KinBodyPtr, int> _menvbodyenabled;
    std::map<KinBodyPtr, int> _menvbodyattached;
    std::map<KinBodyPtr, UserDataPtr> _menvbodyhandles;
    std::vector<KinBodyPtr> _vnewenvbodies;
    std::set<KinBodyPtr> _vattachedbodies;
    void _UpdateJointLimits();
    void _UpdateBodies();

    KinBodyConstPtr _lockedbody; //body currently being collision checked

    EnvironmentBasePtr _penv;
    RobotBasePtr _probotstate;

    int _nBodyAffineDOF;
    int _collisioncount; //number of configurations currently in the cache
    int _dim; //dimensions/DOF

    dReal _maxdistance; //maximum distance possible in the space, used by cover trees
    dReal _collisionthresh; //collision in this distance range will be assumed to be in collision
    dReal _insertiondistance; //only insert nodes if they are at least this far from the nearest node in the tree
    dReal _GetNearestDist(const std::vector<dReal>& cs);

    UserDataPtr _jointchangehandle;
    //std::vector<UserDataPtr> _bodyhandles;

    /// \brief used for debugging/profiling
    uint64_t _qtime; //total query time
    uint64_t _itime; //total insertion time

    bool _profile;

};

typedef boost::shared_ptr<ConfigurationCache> ConfigurationCachePtr;

}

#endif
