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

class CacheTreeVertex
{
    typedef boost::shared_ptr<CacheTreeVertex> CacheTreeVertexPtr;

public:
    CacheTreeVertex(const std::vector<dReal>& cs, CollisionReportPtr report=CollisionReportPtr());
    CacheTreeVertex(const std::vector<dReal>& cs, CollisionReportPtr report, const std::vector<Vector>& linkspheres);

    void AddChildVertex(CacheTreeVertexPtr child, int level){
        _children[level].push_back(child);
    }

    const std::vector<CacheTreeVertexPtr>& GetChildren(int level) {
        return _children[level];
    }

    const std::vector<dReal>& GetConfigurationState() const {
        return _cstate;
    }

    /// \brief to be used to prune and invalidate cache
    //void RemoveChildVertex(){}

    /// \brief delete all children
    //~virtual CacheTreeVertex(){}

    inline CollisionReportPtr GetCollisionReport() const {
        return _report;
    }

    inline bool IsInCollision() const {
        return !!_report;
    }

private:
    CollisionReportPtr _report; ///< if initialized, then a collision occured at this state. Otherwise state is in freespace.
    std::vector<Vector> _linkspheres; ///< xyz is center, w is radius^2
    std::map<int,std::vector<CacheTreeVertexPtr> > _children; //maybe use a vector for each level somehow
    std::vector<dReal> _cstate;
};

typedef boost::shared_ptr<CacheTreeVertex> CacheTreeVertexPtr;

class CacheTree
{
    // distance function to be set directly in the tree
    //
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

    dReal ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef);
    /// \brief finds the k nearest neighbors in the cover tree
    /// \param[in] the vertex to find neighbors for
    /// \param[in] the number of neighbors
    /// \param[in] the type of neighbors, -1 = collision configurations, 1 = free configurations, 0 = both
    /// \param[in] option to only return vertices with a parent
    std::list<std::pair<dReal, CacheTreeVertexPtr> > GetNearestKVertices(CacheTreeVertexPtr in, int k);
    int InsertVertex(CacheTreeVertexPtr in);
    int GetNumVertices(){
        return _numvertices;
    }

    // TODO called by ConfigurationCache when environment changes
    //void PruneTree(){};

private:
    dReal _maxdistance; ///< maximum possible distance between two states. used to balance the tree.
    dReal _base; ///< a constant used to control the max level of traversion
    int _maxlevel; ///< the maximum allowed levels in the tree
    int _minlevel; ///< the minimum allowed levels in the tree
    int _numvertices; ///< the number of vertices in the current tree starting at _root
    std::vector<dReal> _weights; ///< weights used by the distance function
    CacheTreeVertexPtr _root; // the cache tree

    /// \param inserts a configuration into the cache tree
    ///
    /// \param[in] in the input vertex to insert
    /// \param[in] vs the tree vertices at a particular level
    /// \param[in] leveldists the distances of the vs nodes
    /// \param[in] the current level traversing
    int _Insert(CacheTreeVertexPtr in, const std::vector<CacheTreeVertexPtr>& vs, const std::vector<dReal>& leveldists, int level);
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
    int InsertConfiguration(const std::vector<dReal>& cs, dReal dist = -1.0);

    /// \brief determine if configuration is whithin threshold of a collision in the cache (_collisionthresh), known to be in collision, or requires an explicit collision check
    int CheckCollision(const std::vector<dReal>& conf);

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

    //TODO

    /// \brief invalidate the entire cache if necessary
    //void InvalidateCache(){}

    /// \brief prune certain vertices in the cache tree, possibly takes in link, kinbody or environment as a parameter
    ///void PruneCache(){}

    std::list<std::pair<dReal, CacheTreeVertexPtr> > GetNearestKVertices(CacheTreeVertexPtr in, int k); //needs to be tested, possibly changed
    virtual ~ConfigurationCache(){
    }

private:
    /// \brief get the distance to the nearest vertex in the cover tree
    /// \param cs, configuration
    /// \param vertextype, nearest neighbor of what type (1 free, -1 collision, 0 search all)
    dReal _GetNearestDist(const std::vector<dReal>& cs);

    void _UpdateJointLimits();

    /// \brief functions that scale and expand configuration with weights
    //void _ScaleConfiguration(std::vector<dReal>& csout);
    //void _ExpandConfiguration(std::vector<dReal>& dcsout);

    EnvironmentBasePtr _penv;
    RobotBasePtr _probotstate;
    std::vector<int> _vbodyindices;
    int _nBodyAffineDOF;

    CacheTree _cachetree; //collisions
    int _collisioncount; //number of configurations currently in the cache
    std::vector<dReal> _weights; //weights for each joint, inverse of joint resolutions
    std::vector<dReal> _upperlimit, _lowerlimit; //joint limits, used to calculate maxdistance
    dReal _maxdistance; //maximum distance possible in the space, used by cover trees
    int _dim; //dimensions/DOF

    dReal _collisionthresh; //collision in this distance range will be assumed to be in collision
    dReal _insertiondistance; //only insert nodes if they are at least this far from the nearest node in the tree

    UserDataPtr _jointchangehandle;

    /// cache
    std::vector<dReal> _sconf;

    /// \brief used for debugging/profiling
    uint64_t _qtime; //total query time
    uint64_t _itime; //total insertion time

    bool _profile;
};

typedef boost::shared_ptr<ConfigurationCache> ConfigurationCachePtr;

}

#endif
