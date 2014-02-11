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

#include "openraveplugindefs.h"

namespace configurationcache {

using namespace OpenRAVE;

enum ConfigurationType {
    CT_Collision = -1,
    CT_Free = 1,
    CT_Unknown = 0
};

class CacheTreeVertex
{
    typedef boost::shared_ptr<CacheTreeVertex> CacheTreeVertexPtr;

public:
    /// assumes in the report, plink1 is the robot and plink2 is the colliding link
    CacheTreeVertex(const std::vector<dReal>& cs, CollisionReportPtr report);

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

    inline KinBody::LinkConstPtr GetCollidingLink() const {
        return _collidinglink;
    }
    inline int GetRobotLinkIndex() const {
        return _robotlinkindex;
    }

    bool IsInCollision() const {
        return !!_collidinglink;
    }

    void ResetCollisionInfo() {
        _collidinglink = KinBody::LinkConstPtr();
        _collidinglinktrans = Transform();
    }

    ConfigurationType GetConfigurationType() const {
        return _conftype;
    }

    // returns closest distance to a configuration of the opposite type seen so far
    dReal GetUpperBound() const {
        return _approxdispersion.second;
    }

    // returns the closest neighbor seen so far
    CacheTreeVertexPtr GetApproxNN() const {
        return _approxnn.first;
    }

    void UpdateApproximates(dReal distance, CacheTreeVertexPtr v);

private:
    std::vector<dReal> _cstate;
    std::vector<Vector> _linkspheres; ///< xyz is center, w is radius^2 of every link on the robot
    KinBody::LinkConstPtr _collidinglink;
    Transform _collidinglinktrans;
    int _robotlinkindex; ///< the robot link index that is colliding with _collidinglink
    ConfigurationType _conftype;
    std::map<int,std::vector<CacheTreeVertexPtr> > _children; //maybe use a vector for each level somehow
    std::pair<CacheTreeVertexPtr, dReal> _approxdispersion; //minimum distance and configuration of the opposite type seen so far
    std::pair<CacheTreeVertexPtr, dReal> _approxnn; //nearest distance and neighbor seen so far (same type)

    // todo, keep k nearest neighbors and update k every now and then, k = (e + e/dim) * log(n+1) where n is the size of the tree
};

typedef boost::shared_ptr<CacheTreeVertex> CacheTreeVertexPtr;

// Cache stores configuration information in a data structure based on the Cover Tree (Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf)
// The tree contains vertices with configurations, collision/free-space information, distance/nn statistics (e.g., dispersion, upper bounds on minimum distance to collisions, and admissible nearest neighbor), collision reports, etc. To be expanded to include a lean workspace representation for each vertex, i.e., enclosing spheres for each link, and an approximation of a connected graph (there is a path from every configuration to every other configuration, possible by considering log(n) neighbors) that is constructed from collision checking procedures (of the form qi to qf) and can be used to attempt to plan with the cache before sampling new configurations.
// d(p,q) < (1 + e)d(p,S)
// 2^(1+i) (1 + 1/e) <= d(p,Qi)
class CacheTree
{
public:
    CacheTree(){
    }
    CacheTree(const std::vector<dReal>& weights, dReal maxdistance);

    /// \brief traverse all levels, vertices, and children and remove everything
    virtual ~CacheTree(){
    };

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
    int GetNumVertices() const {
        return _numvertices;
    }

    /// \brief todo: update the tree when the environment changes
    void UpdateTree();

    void SetWeights(const std::vector<dReal>& weights);
    const std::vector<dReal>& GetWeights() const {
        return _weights;
    }

    void SetMaxDistance(dReal maxdistance);
    dReal GetMaxDistance() const {
        return _maxdistance;
    }

private:
    /// \brief takes in the configurations of two vertices and returns the distance, currently using an L1 weighted metric
    dReal _ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef);

    /// \param inserts a configuration into the cache tree
    ///
    /// \param[in] in the input vertex to insert
    /// \param[in] vs the tree vertices at a particular level
    /// \param[in] leveldists the distances of the vs nodes
    /// \param[in] the current level traversing
    /// \return 1 if point is not inserted, 0 if point is inserted
    int _Insert(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& vs, const std::vector<dReal>& leveldists, int level);

    int _Remove(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& vs, const std::vector<dReal>& leveldists, int level);

    std::vector<dReal> _weights; ///< weights used by the distance function

    // cache cache
    std::vector<dReal> _nextleveldists;
    std::vector<CacheTreeVertexPtr> _nextlevelvertices;
    std::vector<dReal> _currentleveldists;
    std::vector<CacheTreeVertexPtr> _currentlevelvertices;
    std::vector<CacheTreeVertexPtr> _levelchildren;
    std::list< std::pair<dReal, CacheTreeVertexPtr> > _listDistanceVertices;

    dReal _maxdistance; ///< maximum possible distance between two states. used to balance the tree.
    dReal _base; ///< a constant used to control the max level of traversion.
    dReal _insertiondistance; ///< distance a configuration must have from the nearest configuration in the tree of the same type in order for it to be inserted

    int _maxlevel; ///< the maximum allowed levels in the tree
    int _minlevel; ///< the minimum allowed levels in the tree
    int _numvertices; ///< the number of vertices in the current tree starting at _root

    CacheTreeVertexPtr _root; // root vertex
};

typedef boost::shared_ptr<CacheTree> CacheVertexTreePtr;

class ConfigurationCache
{

public:
    /// \brief start tracking the active DOFs of the robot
    ConfigurationCache(RobotBasePtr probotstate);
    virtual ~ConfigurationCache(){
    }

    /// \brief insert a configuration into the cache
    /// function verifies if the configuration is at least _insertiondistance
    /// from the closest node in the cache
    /// \param cs, configuration
    /// \param ndists, a vector with the distances from the nearest nodes to cs
    /// returns 1 if configuration was inserted and 0 otherwise
    /// \param indist, nearest distance for this configuration; optional parameter if it has already been computed
    int InsertConfiguration(const std::vector<dReal>& cs, CollisionReportPtr report = CollisionReportPtr(), dReal distin = -1);

    /// \brief determine if current configuration is whithin threshold of a collision in the cache (_collisionthresh), known to be in collision, or requires an explicit collision check
    /// \return 1 if in collision, 0 if not in collision, -1 if unknown
    int CheckCollision(KinBody::LinkConstPtr& robotlink, KinBody::LinkConstPtr& collidinglink, dReal& closestdist);

    /// \brief number of nodes currently in the cover tree
    int GetSize(){
        return _cachetree.GetNumVertices();
    }

    /// \brief the cache will assume a new configuration is in collision if the nearest node in the tree is below this distance
    void SetCollisionThresh(dReal colthresh)
    {
        _collisionthresh = colthresh;
    }

    void SetWeights(const std::vector<dReal>& weights);
    /// \brief the cache will not insert configuration if their distance from the nearest node in the tree is not larger than this value
    void SetInsertionDistance(dReal indist)
    {
        _insertiondistance = indist;
    }

    RobotBasePtr GetRobot() const {
        return _pstaterobot;
    }

    /// \brief body's state has changed, so remove collision space and invalidate free space.
    void UpdateCacheFromChangedBody(KinBodyPtr pbody);

    /// \brief invalidate the entire cache
    void Reset();

    /// \brief prune certain vertices in the cache tree, possibly takes in link, kinbody or environment as a parameter
    ///void PruneCache(){}

    std::list<std::pair<dReal, CacheTreeVertexPtr> > GetNearestKVertices(CacheTreeVertexPtr in, int k, ConfigurationType conftype = CT_Collision); //needs to be tested, possibly changed

    void GetDOFValues(std::vector<dReal>& values);

    //int SynchronizeAll(KinBodyConstPtr pbody = KinBodyConstPtr());

private:
    /// \brief called when body has changed state.
    void _UpdateUntrackedBody(KinBodyPtr pbody);

    /// \brief called when a body has been added/removed from the environment. action=1 is add, action=0 is remove
    void _UpdateAddRemoveBodies(KinBodyPtr pbody, int action);

    /// \brief called when tracking robot's joint limits have changed (invalidate cache)
    void _UpdateRobotJointLimits();
    
    void _UpdateRobotGrabbed();

    dReal _GetNearestDist(const std::vector<dReal>& cs);

    /// \brief return true if update stamps on cache are synced, used for debugging
    //bool _CheckSynchronized();
    
    /// \brief get the distance to the nearest vertex in the cover tree
    /// \param cs, configuration
    /// \param vertextype, nearest neighbor of what type (1 free, -1 collision, 0 search all)

    CacheTree _cachetree; //collisions

    RobotBasePtr _pstaterobot;
    std::vector<int> _vRobotActiveIndices;
    int _nRobotAffineDOF;
    Vector _vRobotRotationAxis;
    std::set<KinBodyPtr> _setGrabbedBodies;
    std::vector<dReal> _upperlimit, _lowerlimit; //joint limits, used to calculate maxdistance

    class KinBodyCachedData : public UserData
    {
    public:
        //std::vector<uint8_t> linkenables;
        //std::vector<Transform> _vlinktransforms;
        UserDataPtr _changehandle;
    };

    typedef boost::shared_ptr<KinBodyCachedData> KinBodyCachedDataPtr;

    EnvironmentBasePtr _penv;

    dReal _collisionthresh; //collision in this distance range will be assumed to be in collision
    dReal _freespacethresh;
    dReal _insertiondistance; //only insert nodes if they are at least this far from the nearest node in the tree
    std::string _userdatakey;
    UserDataPtr _jointchangehandle;
    UserDataPtr _handleBodyAddRemove;
    //std::vector<UserDataPtr> _bodyhandles;

    /// \brief used for debugging/profiling
    uint64_t _qtime; //total query time
    uint64_t _itime; //total insertion time

    bool _profile;

};

typedef boost::shared_ptr<ConfigurationCache> ConfigurationCachePtr;

}

#endif
