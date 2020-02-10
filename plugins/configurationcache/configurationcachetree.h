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
#ifndef OPENRAVE_CACHETREE_H
#define OPENRAVE_CACHETREE_H

#include "openraveplugindefs.h"
#include <deque>
#include <boost/pool/pool.hpp>

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave_plugins_configurationcache", msgid)

namespace configurationcache {

using namespace OpenRAVE;

enum ConfigurationNodeType {
    CNT_Unknown = 0,
    CNT_Collision = 1,
    CNT_Free = 2,
    CNT_Any = 3, /// used to target any node. not a node type
};

class CacheTreeNode
{
public:
    /// \brief returns the configuration state
    const dReal* GetConfigurationState() const {
        return _pcstate;
    }

    /// \param report assumes in the report, plink1 is the robot and plink2 is the colliding link
    void SetCollisionInfo(CollisionReportPtr report);

    /// \brief sets the collision info with int values (used by the load/save)
    void SetCollisionInfo(int index, int type);

    /// \brief returns the index of the colliding link
    inline int GetCollidingLinkIndex() const {
        return _collidinglink->GetIndex();
    }

    /// \brief returns the colliding link
    inline KinBody::LinkConstPtr GetCollidingLink() const {
        return _collidinglink;
    }

    /// \brief returns the robot link index in the collision report
    inline int GetRobotLinkIndex() const {
        return _robotlinkindex;
    }

    /// \brief returns true if configuration is in collision
    bool IsInCollision() const {
        return _conftype == CNT_Collision;
    }

    /// \brief returns the configuration type
    ConfigurationNodeType GetType() const {
        return _conftype;
    }

    /// \brief sets the configuration type
    void SetType(ConfigurationNodeType conftype) {
        _conftype = conftype;
        if (_conftype == CNT_Unknown) {
            _usenn = 0;
        }
    }

    /// \brief returns the number of children in this node
    int GetNumChildren() {
        return _vchildren.size();
    }

    /// \brief returns the level this node is in
    int16_t GetLevel(){
        return _level;
    }

    /// \brief returns true if node has itself as a child
    uint8_t HasSelfChild(){
        return _hasselfchild;
    }

    /// \brief returns true if this node is used for nearest neighbor queries
    uint8_t IsNN(){
        return _usenn;
    }

    /// \brief function used to update the hitcount for this node, TODO use this information to prune cache when it gets too big/slow
    inline int IncreaseHitCount(){
        return _hitcount++;
    }

    // returns closest distance to a configuration of the opposite type seen so far
//    dReal GetUpperBound() const {
//        return _approxdispersion.second;
//    }
//
//    // returns the closest neighbor seen so far
//    CacheTreeNodePtr GetApproxNN() const {
//        return _approxnn.first;
//    }

    //void UpdateApproximates(dReal distance, CacheTreeNodePtr v);

protected:
    std::vector<CacheTreeNode*> _vchildren; ///< direct children of this node (for the next level down)
    ConfigurationNodeType _conftype; ///< configuration type for this node
    KinBody::LinkConstPtr _collidinglink; ///< collidinglink in the collision report for this node
    Transform _collidinglinktrans; ///< the colliding link's transform. Valid if _conftype is CNT_Collision
    int _robotlinkindex; ///< the robot link index that is colliding with _collidinglink. Valid if _conftype is CNT_Collision

    // idea: keep k nearest neighbors and update k every now and then, k = (e + e/dim) * log(n+1) where n is the size of the tree?
    //std::map<int,std::vector<CacheTreeNodePtr> > _children; //maybe use a vector for each level somehow
    //std::pair<CacheTreeNodePtr, dReal> _approxdispersion; //minimum distance and configuration of the opposite type seen so far
    //std::pair<CacheTreeNodePtr, dReal> _approxnn; //nearest distance and neighbor seen so far (same type)

    int16_t _level; ///< the level the node belongs to
    uint8_t _hasselfchild; ///< if 1, then _vchildren has contains a clone of this node in the level below it.
    uint8_t _usenn; ///< if 1, then use part of the nearest neighbor search, otherwise ignore
    int _hitcount; /// number of cache hits

    // managed by pool
#ifdef _DEBUG
    int id;
#endif
    Vector* _plinkspheres; ///< xyz is center, w is radius^2 of every link on the robot, pointer managed by outside pool so do not delete
    dReal _pcstate[0]; ///< the state values, pointer managed by outside pool so do not delete. The values always follow the allocation of the structure.

private:
    /// \brief cache tree node needs to be created by a separte memory pool in order to initialize correct pointers
    CacheTreeNode(const std::vector<dReal>& cs, Vector* plinkspheres);
    CacheTreeNode(const dReal* pstate, int dof, Vector* plinkspheres);
    //~CacheTreeNode();

    friend class CacheTree;
};

typedef CacheTreeNode* CacheTreeNodePtr; ///< OPENRAVE_SHARED_PTR might be too slow, and we never expose the pointers outside of CacheTree, so can use raw pointers.
typedef const CacheTreeNode* CacheTreeNodeConstPtr;

/** Cache stores configuration information in a data structure based on the Cover Tree (Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf)

    The tree contains nodes with configurations, collision/free-space information, distance/nn statistics (e.g., dispersion, upper bounds on minimum distance to collisions, and admissible nearest neighbor), collision reports, etc. To be expanded to include a lean workspace representation for each node, i.e., enclosing spheres for each link, and an approximation of a connected graph (there is a path from every configuration to every other configuration, possible by considering log(n) neighbors) that is constructed from collision checking procedures (of the form qi to qf) and can be used to attempt to plan with the cache before sampling new configurations.

    Shouldn't know anything about the openrave environment.

    d(p,q) < (1 + e)d(p,S)
    2^(1+i) (1 + 1/e) <= d(p,Qi)
 */
class CacheTree
{
public:

    CacheTree(int statedof);

    virtual ~CacheTree();

    /// \brief initializes the cache tree with specific weights and max distance
    void Init(const std::vector<dReal>& weights, dReal maxdistance);

    /// \brief resets the nodes for the cache tree to 0
    void Reset();

    /// \brief finds the nearest neighbor in the cover tree of a particular type.
    ///
    /// \param distancebound If > 0, the distance bound such that any points as close as distancebound will be immediately returned
    /// \param conftype the type of node to find. If CNT_Any, will return any type.
    std::pair<CacheTreeNodeConstPtr, dReal> FindNearestNode(const std::vector<dReal>& cs, dReal distancebound=-1, ConfigurationNodeType conftype = CNT_Any) const;

    /// \brief finds the nearest node searching both collision and free nodes. collision nodes takes priority.
    ///
    /// if it is a collision node, it is within collisionthresh. If it is a freespace node, distance is within freespacethresh
    /// \param collisionthresh assumes > 0
    /// \param freespacethresh assumes > 0
    std::pair<CacheTreeNodeConstPtr, dReal> FindNearestNode(const std::vector<dReal>& cs, dReal collisionthresh, dReal freespacethresh) const;

    /// \brief inserts node in the tree. If node is too close to other nodes in the tree, then does not insert.
    ///
    /// \param[in] fMinSeparationDist the max distance a node should be separated from its closest neighbor. If node is collision, then only applies to collision neighbors, free neighbors are ignored.
    /// \return 1 if point is inserted and parent found. 0 if no parent found and point is not inserted. -1 if parent found but point not inserted since it is close to fMinSeparationDist
    int InsertNode(const std::vector<dReal>& cs, CollisionReportPtr report, dReal fMinSeparationDist);

    /// \brief removes node from the tree
    ///
    /// \return true if node is removed
    bool RemoveNode(CacheTreeNodeConstPtr nodein);

    /// \brief remove all nodes that were in collision with pbody
    void UpdateCollisionNodes(KinBodyPtr pbody);

    /// \brief number of nodes in the tree; todo: also count nodes by type
    int GetNumNodes() const {
        return _numnodes;
    }

    /// \brief return the configuration values for all nodes in the tree
    void GetNodeValues(std::vector<dReal>& vals) const;

    /// \brief retuns the values for all nodes in the tree
    void GetNodeValuesList(std::vector<CacheTreeNodePtr>& lvals);

    /// \brief sets the weights
    void SetWeights(const std::vector<dReal>& weights);

    /// \brief returns the current weights
    const std::vector<dReal>& GetWeights() const {
        return _weights;
    }

    /// \brief set the maxdistance parameter for this tree
    void SetMaxDistance(dReal maxdistance);

    /// \brief returns the maxdistance parameter for this tree
    dReal GetMaxDistance() const {
        return _maxdistance;
    }

    /// \brief returns the base parameter for this tree
    dReal GetBase() const {
        return _base;
    }

    /// \brief sets the base parameter, 2.0 is the default value, if the value is lower, nn queries are faster at the cost of slower insertion times
    void SetBase(dReal base);

    /// \brief returns the weighted euclidean distance between two configurations
    dReal ComputeDistance(const std::vector<dReal>& cstatei, const std::vector<dReal>& cstatef) const;

    /// \brief for debug purposes, validates the tree as described in Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf
    bool Validate();

    /// \brief sets all collision configurations in the tree to CNT_Unknown
    int RemoveCollisionConfigurations();

    /// \brief sets all free configurations in the tree to CNT_Unknown
    int RemoveFreeConfigurations();

    /// \brief sets all collision configurations with pbody in its report to CNT_Unknown
    int UpdateCollisionConfigurations(KinBodyPtr pbody);

    /// \brief sets all free configurations to CNT_Unknown, TODO, only set those that overlap with the new body
    int UpdateFreeConfigurations(KinBodyPtr pbody);

    /// \brief returns the number of configurations in the tree that are not CNT_Unknown
    int GetNumKnownNodes();

    /// \brief save cache to disk
    int SaveCache(std::string filename);

    /// \brief load cache from disk
    int LoadCache(std::string filename, EnvironmentBasePtr penv);

private:
    /// \brief creates new node on the pool
    CacheTreeNodePtr _CreateCacheTreeNode(const std::vector<dReal>& cs, CollisionReportPtr report);
    CacheTreeNodePtr _CloneCacheTreeNode(CacheTreeNodeConstPtr refnode);

    /// \brief deletes the node from the pool and calls its destructor.
    void _DeleteCacheTreeNode(CacheTreeNodePtr pnode);

    /// \brief takes in the configurations of two nodes and returns the distance, currently returning square of L2 norm.
    ///
    /// note the distance metric has to satisfy triangle inequality
    dReal _ComputeDistance2(const dReal* cstatei, const dReal* cstatef) const;

    /// \brief inserts a configuration into the cache tree
    ///
    /// \param[in] node the input node to insert
    /// \param[in] nodesin the tree nodes at "level" with the respecitve distances computed for them
    /// \param[in] level the current level traversing
    /// \param[in] levelbound pow(_base, level)
    /// \param[in] fMinSeparationDist the min distance a node should be separated from its closest neighbor
    /// \return 1 if point is inserted and parent found. 0 if no parent found and point is not inserted. -1 if parent found but point not inserted since it is close to fMinSeparationDist
    int _Insert(CacheTreeNodePtr node, const std::vector< std::pair<CacheTreeNodePtr, dReal> >& nodesin, int level, dReal levelbound2, dReal fMinSeparationDist2);

    /// \brief inerts a node directly to parentnode
    ///
    /// If parentnode's configuration is too close to nodein, or parentnode's level is too high, will create dummy child nodes
    /// \param fInsetLevelBound pow(_base,maxinsertlevel)
    bool _InsertDirectly(CacheTreeNodePtr nodein, CacheTreeNodePtr parentnode, dReal parentdist, int maxinsertlevel, dReal fInsetLevelBound2);

    /// \param[inout] coversetnodes for every level starting at the max, the parent cover sets. coversetnodes[i] is the _maxlevel-i level
    bool _Remove(CacheTreeNodePtr node, std::vector< std::vector<CacheTreeNodePtr> >& vvCoverSetNodes, int level, dReal levelbound2);

    inline int _EncodeLevel(int level) const {
        if( level <= 0 ) {
            return -2*level;
        }
        else {
            return 2*level+1;
        }
    }

    inline int _DecodeLevel(int enclevel) const {
        if( enclevel & 1 ) {
            return enclevel/2;
        }
        else {
            return -(enclevel/2);
        }
    }

    std::vector<dReal> _weights; ///< weights used by the distance function
    std::vector<dReal> _curconf;

    std::string _fulldirname;
    CacheTreeNodePtr _newnode; ///< for loading
    std::string _collidingbodyname;
    KinBodyPtr _pcollidingbody;

    std::map<CacheTreeNodePtr, int> _mapNodeIndices;
    std::vector< std::set<CacheTreeNodePtr> > _vsetLevelNodes; ///< _vsetLevelNodes[enc(level)][node] holds the indices of the children of "node" of a given the level. enc(level) maps (-inf,inf) into [0,inf) so it can be indexed by the vector. Every node has an entry in a map here. If the node doesn't hold any children, then it is at the leaf of the tree. _vsetLevelNodes.at(_EncodeLevel(_maxlevel)) is the root.

    OPENRAVE_SHARED_PTR<boost::pool<> > _poolNodes; ///< the dynamically growing memory pool of nodes. Since each node's size is determined during run-time, the pool constructor has to be called with the correct node size

    dReal _maxdistance; ///< maximum possible distance between two states. used to balance the tree.
    dReal _base, _fBaseInv, _fBaseInv2, _fBaseChildMult; ///< a constant used to control the max level of traversion. _fBaseInv = 1/_base, _fBaseInv2=Sqr(_fBaseInv), _fBaseChildMult=1/(_base-1)

    int _statedof; ///< the state space DOF tree is configured for
    int _maxlevel; ///< the maximum allowed levels in the tree, this is where the root node starts (inclusive)
    int _minlevel; ///< the minimum allowed levels in the tree (inclusive)
    int _numnodes; ///< the number of nodes in the current tree starting at the root at _vsetLevelNodes.at(_EncodeLevel(_maxlevel))
    dReal _fMaxLevelBound; ///< pow(_base, _maxlevel)

    // cache cache
    mutable std::vector< std::pair<CacheTreeNodePtr, dReal> > _vCurrentLevelNodes, _vNextLevelNodes;
    mutable std::vector< std::vector<CacheTreeNodePtr> > _vvCacheNodes;

    std::vector<CacheTreeNodePtr> _vnodes; ///< for loading
    std::vector<dReal> _dummycs; ///< for loading
};

typedef OPENRAVE_SHARED_PTR<CacheTree> CacheTreePtr;

/** Maintains an up-to-date cache tree synchronized to the openrave environment. Tracks bodies being added removed, states changing, etc.
   The state of cache consists of the active DOFs of the robot that is passed in at constructor time.
 */
class ConfigurationCache
{
public:
    /// \brief start tracking the active DOFs of the robot
    /// \param envupdates, if set to true, cache is updated when the environment changes, this is set to false for selfcollision caches for example
    ConfigurationCache(RobotBasePtr probotstate, bool envupdates = true);
    virtual ~ConfigurationCache();

    /// \brief insert a configuration into the cache
    /// function verifies if the configuration is at least _insertiondistancemult
    /// from the closest node in the cache
    /// \param cs, configuration
    /// \param ndists, a vector with the distances from the nearest nodes to cs
    /// \param indist, If > 0, nearest distance for this configuration already computed by CheckCollision
    /// \return true if configuration was inserted
    bool InsertConfiguration(const std::vector<dReal>& cs, CollisionReportPtr report = CollisionReportPtr(), dReal indist = -1);

    /// \brief removes all collision configurations colliding with pbody, used to update cache when bodies are removed or moved
    int UpdateCollisionConfigurations(KinBodyPtr pbody);

    /// \brief removes all collision configurations
    int RemoveCollisionConfigurations();

    /// \brief removes all free configurations
    int RemoveFreeConfigurations();

    /// \brief removes all free configurations, to be updated to only remove those with linkspheres that overlap with the body
    int UpdateFreeConfigurations(KinBodyPtr pbody);

    /// \brief determine if current configuration is whithin threshold of a collision in the cache (_collisionthresh), known to be in collision, or requires an explicit collision check
    /// \return 1 if in collision, 0 if not in collision, -1 if unknown
    int CheckCollision(const std::vector<dReal>& cs, KinBody::LinkConstPtr& robotlink, KinBody::LinkConstPtr& collidinglink, dReal& closestdist);

    int CheckCollision(KinBody::LinkConstPtr& robotlink, KinBody::LinkConstPtr& collidinglink, dReal& closestdist);

    /// \brief invalidate the entire cache
    void Reset();

    void GetDOFValues(std::vector<dReal>& values);

    //int SynchronizeAll(KinBodyConstPtr pbody = KinBodyConstPtr());

    /// \brief number of nodes currently in the cover tree
    int GetNumNodes() const {
        return _cachetree.GetNumNodes();
    }

    /// \brief number of nodes with known type, i.e., != CNT_Unknown
    int GetNumKnownNodes();

    /// \brief return configuration values for all nodes in the tree, calls cachetree's function
    void GetNodeValues(std::vector<dReal>& vals) const {
        _cachetree.GetNodeValues(vals);
    }

    /// \brief return nearest configuration and distance
    std::pair<std::vector<dReal>, dReal> FindNearestNode(const std::vector<dReal>& conf, dReal dist = 0.0);

    /// \brief return distance between two configurations as computed by the tree (for testing)
    dReal ComputeDistance(const std::vector<dReal>& qi, const std::vector<dReal>& qf) const {
        return _cachetree.ComputeDistance(qi,qf);
    }

    /// \brief the cache will assume a new configuration is in collision if the nearest node in the tree is below this distance
    inline void SetCollisionThresh(dReal colthresh)
    {
        _collisionthresh = colthresh;
    }

    /// \brief set the freespacethresh parameter
    inline void SetFreeSpaceThresh(dReal freespacethresh)
    {
        _freespacethresh = freespacethresh;
    }

    /// \brief set the base parameter
    inline void SetBase(dReal base)
    {
        _cachetree.SetBase(base);
    }

    /// \brief disable environment updates
    inline void DisableEnvUpdates()
    {
        _envupdates = false;
    }

    /// \brief enables environment updates
    inline void EnableEnvUpdates()
    {
        _envupdates = true;
    }

    /// \brief set weights
    void SetWeights(const std::vector<dReal>& weights);

    /// \brief the cache will not insert configuration if their distance from the nearest node in the tree is not larger than this value
    inline void SetInsertionDistanceMult(dReal indist)
    {
        _insertiondistancemult = indist;
    }

    /// \brief returns the collision threshold, i.e., configurations within this distance are assumed to be in collision
    inline dReal GetCollisionThresh() const
    {
        return _collisionthresh;
    }

    /// \brief returns the free threshold value, i.e., configurations within this distance are assumed to not be in collision
    inline dReal GetFreeSpaceThresh() const
    {
        return _freespacethresh;
    }

    /// \brief returns the insertation distance, i.e., configurations must have a distance greater than this in order for them to be added to the tree
    inline dReal GetInsertionDistanceMult() const
    {
        return _insertiondistancemult;
    }

    /// \brief returns the base parameter
    inline dReal GetBase() const
    {
        return _cachetree.GetBase();
    }

    /// \brief returns the robot
    inline RobotBasePtr GetRobot() const {
        return _pstaterobot;
    }

    /// \brief for debug purposes, validates the tree as described in Beygelzimer et al. 2006 http://hunch.net/~jl/projects/cover_tree/icml_final/final-icml.pdf
    bool Validate();

    /// \brief remove all nodes in collision with pbody, for testing
    inline void UpdateCollisionNodes(KinBodyPtr pbody)
    {
        _cachetree.UpdateCollisionNodes(pbody);
    }

    /// \brief saves the cache to disk
    inline void SaveCache(std::string filename)
    {
        _cachetree.SaveCache(filename);
    }

    /// \brief loads cache from disk
    inline void LoadCache(std::string filename, EnvironmentBasePtr penv)
    {
        _cachetree.LoadCache(filename, penv);
    }

private:
    /// \brief called when body has changed state.
    void _UpdateUntrackedBody(KinBodyPtr pbody);

    /// \brief called when a body has been added/removed from the environment. action=1 is add, action=0 is remove
    void _UpdateAddRemoveBodies(KinBodyPtr pbody, int action);

    /// \brief called when tracking robot's joint limits have changed (invalidate cache)
    void _UpdateRobotJointLimits();

    /// \brief called when grabbeb bodies are updated
    void _UpdateRobotGrabbed();

    CacheTree _cachetree; ///< cache tree datastructure with configurations and their collision information

    RobotBasePtr _pstaterobot;
    std::vector<int> _vRobotActiveIndices;
    int _nRobotAffineDOF;
    Vector _vRobotRotationAxis;
    std::set<KinBodyPtr> _setgrabbedbodies;

    std::vector<KinBodyPtr> _vgrabbedbodies;
    std::vector<KinBodyPtr> _vnewgrabbedbodies;
    std::vector<KinBodyPtr> _vnewenvbodies;
    std::vector<dReal> _upperlimit, _lowerlimit; ///< joint limits, used to calculate maxdistance
    std::vector<dReal> _newupperlimit, _newlowerlimit;
    std::vector<CacheTreeNodePtr> _cachetreenodes;
    std::vector<dReal> _vweights;

    class KinBodyCachedData : public UserData
    {
public:
        UserDataPtr _changehandle;
    };

    typedef OPENRAVE_SHARED_PTR<KinBodyCachedData> KinBodyCachedDataPtr;
    typedef OPENRAVE_WEAK_PTR<KinBodyCachedData> KinBodyCachedDataWeakPtr;

    EnvironmentBasePtr _penv; ///< environment

    dReal _collisionthresh; ///< configurations in this distance range (from a collsion configuration in the tree) will be assumed to be in collision
    dReal _freespacethresh; ///< configurations in this distance range (from a free configuration in the tree)  will be assumed to not be in collision
    dReal _insertiondistancemult; ///< only insert nodes if they are far from the nearest node in the tree. The distance is computed by multiplying this number of _collisionthresh or _freespacethresh. Distance a configuration must have from the nearest configuration in the tree in order for it be inserted
    std::string _userdatakey;
    UserDataPtr _handleJointLimitChange, _handleGrabbedChange; ///< handles for changes in the robot's joint limits and grabbed bodies
    UserDataPtr _handleBodyAddRemove; ///< handle for bodies added or removed to the environment
    std::list<KinBodyCachedDataWeakPtr> _listCachedData; ///< necessary to keep a list of all the data created in order to force reset the change callbacks

    bool _envupdates; ///< if set to true, cache will update itself when the environment changes; should be set to false for selfcollision cache

};

typedef OPENRAVE_SHARED_PTR<ConfigurationCache> ConfigurationCachePtr;

}

#endif
