// -*- coding: utf-8 -*-
#ifndef OPENRAVE_IVSHMEM_INTERFACE
#define OPENRAVE_IVSHMEM_INTERFACE

#include <boost/unordered_set.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/utils.h>
#include <boost/function_output_iterator.hpp>

#include <openrave/openrave.h>

#include "fclspace.h"
#include "fclmanagercache.h"

#include "triple_buffered_shared_memory.hpp"
#include "ivshmem_server.hpp"

namespace ivshmem {

#define START_TIMING_OPT(statistics, label, options, isRobot);           \
    START_TIMING(statistics, boost::str(boost::format("%s,%x,%d")%label%options%isRobot))

typedef FCLSpace::FCLKinBodyInfoConstPtr FCLKinBodyInfoConstPtr;
typedef FCLSpace::FCLKinBodyInfoPtr FCLKinBodyInfoPtr;
typedef FCLSpace::LinkInfoPtr LinkInfoPtr;

template<typename T>
inline bool IsIn(T const& x, std::vector<T> const &collection) {
    return std::find(collection.begin(), collection.end(), x) != collection.end();
}

class IVShMemInterface : public OpenRAVE::CollisionCheckerBase
{
public:
    class CollisionCallbackData {
public:
        CollisionCallbackData(boost::shared_ptr<IVShMemInterface> pchecker, CollisionReportPtr report, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<LinkConstPtr>& vlinkexcluded);

        const std::list<EnvironmentBase::CollisionCallbackFn>& GetCallbacks();

        boost::shared_ptr<IVShMemInterface> _pchecker;
        fcl::CollisionRequest _request;
        fcl::CollisionResult _result;
        fcl::DistanceRequest _distanceRequest;
        fcl::DistanceResult _distanceResult;
        CollisionReportPtr _report;
        std::vector<KinBodyConstPtr> const& _vbodyexcluded;
        std::vector<LinkConstPtr> const& _vlinkexcluded;
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        bool bselfCollision;  ///< true if currently checking for self collision.
        bool _bStopChecking;  ///< if true, then stop the collision checking loop
        bool _bCollision;  ///< result of the collision

        bool _bHasCallbacks; ///< true if there's callbacks registered in the environment
        std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    };

    typedef boost::shared_ptr<CollisionCallbackData> CollisionCallbackDataPtr;

    IVShMemInterface(OpenRAVE::EnvironmentBasePtr penv, std::istream& sinput);

    ~IVShMemInterface() override;

    void Clone(InterfaceBaseConstPtr preference, int cloningoptions);

    void SetNumMaxContacts(int numMaxContacts) {
        _numMaxContacts = numMaxContacts;
    }

    int GetNumMaxContacts() const {
        return _numMaxContacts;
    }

    void SetGeometryGroup(const std::string& groupname)
    {
        _fclspace->SetGeometryGroup(groupname);
    }

    const std::string& GetGeometryGroup() const
    {
        return _fclspace->GetGeometryGroup();
    }

    bool SetBodyGeometryGroup(KinBodyConstPtr pbody, const std::string& groupname)
    {
        return _fclspace->SetBodyGeometryGroup(pbody, groupname);
    }

    const std::string& GetBodyGeometryGroup(KinBodyConstPtr pbody) const
    {
        return _fclspace->GetBodyGeometryGroup(*pbody);
    }

    virtual bool SetCollisionOptions(int collision_options);

    virtual int GetCollisionOptions() const
    {
        return _options;
    }

    virtual void SetTolerance(OpenRAVE::dReal tolerance)
    {
    }


    /// Sets the broadphase algorithm for collision checking
    /// The input algorithm can be one of : Naive, SaP, SSaP, IntervalTree, DynamicAABBTree{,1,2,3}, DynamicAABBTree_Array{,1,2,3}, SpatialHashing
    /// e.g. "SetBroadPhaseAlgorithm DynamicAABBTree"
    bool SetBroadphaseAlgorithmCommand(ostream& sout, istream& sinput);

    void _SetBroadphaseAlgorithm(const std::string &algorithm);

    const std::string & GetBroadphaseAlgorithm() const {
        return _broadPhaseCollisionManagerAlgorithm;
    }

    /// Sets the bounding volume hierarchy representation which can be one of
    /// AABB, OBB, RSS, OBBRSS, kDOP16, kDOP18, kDOP24, kIOS
    /// e.g. "SetBVHRepresentation OBB"
    bool _SetBVHRepresentation(ostream& sout, istream& sinput);

    std::string const& GetBVHRepresentation() const {
        return _fclspace->GetBVHRepresentation();
    }


    bool InitEnvironment() override;

    void DestroyEnvironment() override;

    bool InitKinBody(OpenRAVE::KinBodyPtr pbody) override;

    void RemoveKinBody(OpenRAVE::KinBodyPtr pbody) override;

    bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(LinkConstPtr plink1, LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(LinkConstPtr plink, KinBodyConstPtr pbody,CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(LinkConstPtr plink,CollisionReportPtr report = CollisionReportPtr()) override;
    bool CheckCollision(LinkConstPtr plink, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr()) override;
    bool CheckCollision(KinBodyConstPtr pbody, std::vector<KinBodyConstPtr> const &vbodyexcluded, std::vector<LinkConstPtr> const &vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(const OpenRAVE::TriMesh& trimesh, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(const OpenRAVE::TriMesh& trimesh, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(const OpenRAVE::AABB& ab, const OpenRAVE::Transform& aabbPose, const std::vector<OpenRAVE::KinBodyConstPtr>& vIncludedBodies, OpenRAVE::CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckCollision(const RAY& ray, LinkConstPtr plink,CollisionReportPtr report = CollisionReportPtr()) override
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) override
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr()) override
    {
        RAVELOG_WARN("fcl doesn't support Ray collisions\n");
        return false; //TODO
    }

    bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) override;

    bool CheckStandaloneSelfCollision(LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) override;


private:
    inline boost::shared_ptr<IVShMemInterface> shared_checker() {
        return boost::static_pointer_cast<IVShMemInterface>(shared_from_this());
    }

    static bool CheckNarrowPhaseCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data);
    static bool CheckNarrowPhaseGeomCollision(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data);
    static bool CheckNarrowPhaseDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist);
    static bool CheckNarrowPhaseGeomDistance(fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data, fcl::FCL_REAL& dist);

    static LinkPair MakeLinkPair(LinkConstPtr plink1, LinkConstPtr plink2);

    std::pair<FCLSpace::FCLKinBodyInfo::LinkInfo*, LinkConstPtr> GetCollisionLink(const fcl::CollisionObject &collObj);

    std::pair<FCLSpace::FCLKinBodyInfo::FCLGeometryInfo*, GeometryConstPtr> GetCollisionGeometry(const fcl::CollisionObject &collObj);

    FCLCollisionManagerInstance& _GetBodyManager(KinBodyConstPtr pbody, bool bactiveDOFs);

    /// \brief gets environment manager corresponding to excludedBodyEnvIndices
    /// \param excludedBodyEnvIndices vector of environment body indices for excluded bodies. sorted in ascending order
    FCLCollisionManagerInstance& _GetEnvManager(const std::vector<int>& excludedBodyEnvIndices);

    inline bool _IsEnabled(const KinBody& body)
    {
        if( body.IsEnabled() ) {
            return true;
        }

        // check if body has any enabled bodies
        body.GetGrabbed(_vCachedGrabbedBodies);
        FOREACH(itbody, _vCachedGrabbedBodies) {
            if( (*itbody)->IsEnabled() ) {
                return true;
            }
        }

        return false;
    }

    // ===== Diverted collision functions
    std::size_t collide(const fcl::CollisionObject* o1, const fcl::CollisionObject* o2,
                        const fcl::CollisionRequest& request,
                        fcl::CollisionResult& result);

    std::size_t collide(const fcl::CollisionGeometry* o1, const fcl::Transform3f& tf1,
                        const fcl::CollisionGeometry* o2, const fcl::Transform3f& tf2,
                        const fcl::CollisionRequest& request,
                        fcl::CollisionResult& result);

    TripleBufferedSharedIOMemory _shmem;
    IVShMemServer _ivshmem_server;
    std::thread _ivshmem_thread;
    uint64_t _query_id;

    int _options;
    boost::shared_ptr<FCLSpace> _fclspace;
    int _numMaxContacts;
    std::string _userdatakey;
    std::string _broadPhaseCollisionManagerAlgorithm; ///< broadphase algorithm to use to create a manager. tested: Naive, DynamicAABBTree2

    typedef std::map< std::pair<const void*, int>, FCLCollisionManagerInstancePtr> BODYMANAGERSMAP; ///< Maps pairs of (body, bactiveDOFs) to oits manager
    BODYMANAGERSMAP _bodymanagers; ///< managers for each of the individual bodies. each manager should be called with InitBodyManager. Cannot use KinBodyPtr here since that will maintain a reference to the body!
    int _maxNumBodyManagers = 0; ///< for debug, record max size of _bodymanagers.

    typedef std::map<std::vector<int>, FCLCollisionManagerInstancePtr> EnvManagersMap; ///< Maps vector of excluded body indices to FCLCollisionManagerInstancePtr
    EnvManagersMap _envmanagers; // key is sorted vector of environment body indices of excluded bodies
    int _nGetEnvManagerCacheClearCount; ///< count down until cache can be cleared
    int _maxNumEnvManagers = 0; ///< for debug, record max size of _envmanagers.

    // In order to reduce allocations during collision checking

    CollisionReport _reportcache;
    std::vector<fcl::Vec3f> _fclPointsCache;
    std::vector<fcl::Triangle> _fclTrianglesCache;
    std::vector<KinBodyPtr> _vCachedGrabbedBodies;

    std::vector<int> _attachedBodyIndicesCache;

    bool _bIsSelfCollisionChecker; // Currently not used
    bool _bParentlessCollisionObject; ///< if set to true, the last collision command ran into colliding with an unknown object
};

// TODO : This is becoming really stupid, I should just add optional additional data for DynamicAABBTree
BroadPhaseCollisionManagerPtr CreateManagerFromBroadphaseAlgorithm(std::string const &algorithm);

} // namespace ivshmem

#endif // OPENRAVE_IVSHMEM_INTERFACE
