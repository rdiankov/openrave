// -*- coding: utf-8 -*-
#ifndef OPENRAVE_IVSHMEM_FCL_MANAGERCACHE
#define OPENRAVE_IVSHMEM_FCL_MANAGERCACHE

#include "plugindefs.h"
#include "fclspace.h"

namespace ivshmem {

typedef boost::shared_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerPtr;
typedef boost::weak_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerWeakPtr;

/// \brief A broadphase collision manager together with cache data of the bodies it contains
/// used to maintain the correct state of the broadphase manager
class FCLCollisionManagerInstance : public boost::enable_shared_from_this<FCLCollisionManagerInstance>
{
    ///< cache data of body that is managed
    struct KinBodyCache
    {
        KinBodyCache();

        KinBodyCache(const KinBodyConstPtr& pbody, const FCLSpace::FCLKinBodyInfoPtr& pinfo);

        void SetBodyData(const KinBodyConstPtr& pbody, const FCLSpace::FCLKinBodyInfoPtr& pinfo, const std::vector<uint64_t>& linkEnableStateCache);

        ~KinBodyCache() {
            // KinBodyCache is stored in vector, and resizing it causes destructor be called, but do not want warning on vcolobjs being non-empty.
            Invalidate(false);
        }

        void Invalidate(bool warnOnNonEmptyColObjs = true);

        /// checks if weak pointer is expired (either reset or KinBody that it was pointing to is destructed
        // note, this doesn't check if KinBody has non-zero env body index, so it's not the same check as (pwbody.lock() && pwbody.lock()->GetEnvironmentBodyIndex())
        inline bool IsValid() const
        {
            return !pwbody.expired(); // expired is slightly faster than lock
        }

        KinBodyConstWeakPtr pwbody; ///< weak pointer to body
        FCLSpace::FCLKinBodyInfoWeakPtr pwinfo; ///< weak pointer to info
        int nLastStamp; ///< copyied from FCLSpace::FCLKinBodyInfo when body was last updated
        int nLinkUpdateStamp; ///< copied from FCLSpace::FCLKinBodyInfo when body was last updated
        int nGeometryUpdateStamp; ///< copied from FCLSpace::FCLKinBodyInfo when geometry was last updated
        int nAttachedBodiesUpdateStamp; /// copied from FCLSpace::FCLKinBodyInfo when attached bodies was last updated
        int nActiveDOFUpdateStamp; ///< update stamp when the active dof changed
        std::vector<uint64_t> linkEnableStatesBitmasks; ///< links that are currently inside the manager
        std::vector<CollisionObjectPtr> vcolobjs; ///< collision objects used for each link (use link index). have to hold pointers so that FCLKinBodyInfo does not remove them!
        std::string geometrygroup; ///< cached geometry group
    };

public:
    FCLCollisionManagerInstance(FCLSpace& fclspace, BroadPhaseCollisionManagerPtr pmanager_);
    ~FCLCollisionManagerInstance();

    /// \brief sets up manager for body checking
    /// \param bTrackActiveDOF true if should be tracking the active dof
    void InitBodyManager(KinBodyConstPtr pbody, bool bTrackActiveDOF);

    /// \brief sets up manager for environment checking
    /// \param excludedEnvBodyIndices Index corresponds to the environement body index. value 1 means excluded. if value at index 5 is 1, KinBody with env body index 5 is excluded
    void InitEnvironment(const std::vector<int8_t>& excludedEnvBodyIndices);

    /// \brief makes sure that all the bodies are currently in the scene (if they are not explicitly excluded)
    void EnsureBodies(const std::vector<KinBodyConstPtr>& vbodies);

    /// \brief remove tracking of the body, return true if body was removed
    bool RemoveBody(const KinBody &body);

    /// \brief Synchronizes the element of the manager instance whose update stamps are outdated
    void Synchronize();

    inline BroadPhaseCollisionManagerPtr GetManager() const {
        return pmanager;
    }

    inline uint32_t GetLastSyncTimeStamp() const {
        return _lastSyncTimeStamp;
    }

    void PrintStatus(uint32_t debuglevel)
    {
        if( IS_DEBUGLEVEL(debuglevel) ) {
            std::stringstream ss;
            ss << "bodies=[";
            for (KinBodyCache& cache : _vecCachedBodies) {
                KinBodyConstPtr pbody = cache.pwbody.lock();
                if( !!pbody ) {
                    ss << pbody->GetName() << ", ";
                }
            }
            ss << "]";
            OpenRAVE::RavePrintfA(ss.str(), debuglevel);
        }
    }

private:
    /// \brief adds a body to the manager, returns true if something was added
    /// should not add anything to _vecCachedBodies! insert to _tmpSortedBuffer
    bool _AddBody(const KinBody& body, const FCLSpace::FCLKinBodyInfoPtr& pinfo, std::vector<CollisionObjectPtr>& vcolobjs, std::vector<uint64_t>& linkEnableStatesBitmasks, bool bTrackActiveDOF);

    void _UpdateActiveLinks(const RobotBase& robot);

    FCLSpace& _fclspace; ///< reference for speed
    BroadPhaseCollisionManagerPtr pmanager;
    std::vector<KinBodyCache> _vecCachedBodies; ///< vector of KinBodyCache(weak body, updatestamp)) where index is KinBody::GetEnvironmentBodyIndex. Index 0 has invalid entry because valid env id starts from 1.
    uint32_t _lastSyncTimeStamp; ///< timestamp when last synchronized

    std::vector<int8_t> _vecExcludeBodyIndices; ///< any bodies that should not be considered inside the manager, used with environment mode. includes environment body index of of bodies who should be excluded.
    CollisionGroup _tmpSortedBuffer; ///< cache, sorted so that we can efficiently search

    KinBodyConstWeakPtr _ptrackingbody; ///< if set, then only tracking the attached bodies if this body
    std::vector<int> _vTrackingActiveLinks; ///< indices of which links are active for tracking body
    std::vector<uint64_t> _linkEnableStatesBitmasks; ///< links that are currently inside the manager
    std::vector<uint64_t> _linkEnableStatesCache; ///< memory holder for receiving return value of GetLinkEnableStatesMasks in Synchronize.

    std::vector<KinBodyPtr> _vecAttachedEnvBodiesCache;
    std::vector<int> _vecAttachedEnvBodyIndicesCache;

    bool _bTrackActiveDOF; ///< if true and _ptrackingbody is valid, then should be tracking the active dof of the _ptrackingbody
};

typedef boost::shared_ptr<FCLCollisionManagerInstance> FCLCollisionManagerInstancePtr;
typedef boost::weak_ptr<FCLCollisionManagerInstance> FCLCollisionManagerInstanceWeakPtr;

} // namespace ivshmem

#endif // OPENRAVE_IVSHMEM_FCL_MANAGERCACHE
