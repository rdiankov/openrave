// -*- coding: utf-8 -*-
#ifndef OPENRAVE_FCL_MANAGERCACHE
#define OPENRAVE_FCL_MANAGERCACHE

#include "plugindefs.h"
#include "fclspace.h"

namespace fclrave {

//static bool CheckForObj(fcl::DynamicAABBTreeCollisionManager::DynamicAABBNode* root, fcl::CollisionObject* pobj)
//{
//    if( !root ) {
//        return false;
//    }
//    if( root->data == pobj ) {
//        return true;
//    }
//    if( !!root->children[0] ) {
//        if( CheckForObj(root->children[0], pobj) ) {
//            return true;
//        }
//    }
//    if( !!root->children[1] ) {
//        if( CheckForObj(root->children[1], pobj) ) {
//            return true;
//        }
//    }
//    return false;
//}
//
//static int CountForObj(fcl::DynamicAABBTreeCollisionManager::DynamicAABBNode* root, fcl::CollisionObject* pobj)
//{
//    if( !root ) {
//        return 0;
//    }
//    int count = 0;
//    if( root->data == pobj ) {
//        ++count;
//    }
//    count += CheckForObj(root->children[0], pobj);
//    count += CheckForObj(root->children[1], pobj);
//    return count;
//}

typedef boost::shared_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerPtr;
typedef boost::weak_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerWeakPtr;

/// \brief A broadphase collision manager together with cache data of the bodies it contains
///
/// used to maintain the correct state of the broadphase manager
class FCLCollisionManagerInstance : public boost::enable_shared_from_this<FCLCollisionManagerInstance>
{
    ///< cache data of body that is managed
    struct KinBodyCache
    {
        KinBodyCache()
            : nLastStamp(0), nLinkUpdateStamp(0), nGeometryUpdateStamp(0), nAttachedBodiesUpdateStamp(0), nActiveDOFUpdateStamp(0)
        {
        }

        KinBodyCache(const KinBodyConstPtr& pbody, const FCLSpace::KinBodyInfoPtr& pinfo)
        {
            Set(pbody, pinfo);
        }

        void Set(const KinBodyConstPtr& pbody, const FCLSpace::KinBodyInfoPtr& pinfo)
        {
            pwbody = pbody;
            pwinfo = pinfo;
            nLastStamp = pinfo->nLastStamp;
            nLinkUpdateStamp = pinfo->nLinkUpdateStamp;
            nGeometryUpdateStamp = pinfo->nGeometryUpdateStamp;
            geometrygroup = pinfo->_geometrygroup;
            nAttachedBodiesUpdateStamp = pinfo->nAttachedBodiesUpdateStamp;
            nActiveDOFUpdateStamp = pinfo->nActiveDOFUpdateStamp;
            pbody->GetLinkEnableStates(linkEnableStates);

            if (!vcolobjs.empty()) {
                std::stringstream ss;
                ss << "env=" << pbody->GetEnv()->GetId() << ", FCLCollisionManagerInstance 0x" << hex << this;
                ss << " has " << dec << vcolobjs.size() << " collion objects (";
                for (const CollisionObjectPtr& obj : vcolobjs) {
                    ss << "0x" << hex << obj << ", ";
                }
                ss << "), but leaving untouched.";
                RAVELOG_WARN_FORMAT("%s", ss.str());
            }
        }

        ~KinBodyCache() {
            Invalidate();
        }

        inline void Invalidate()
        {
            if (!IsValid()) {
                //RAVELOG_INFO_FORMAT("0x%x is previously invalidated, or was never valid.", this);
                return;
            }
            if( vcolobjs.size() > 0 ) { // should never happen
                KinBodyConstPtr pbody = pwbody.lock();
                std::string name;
                if( !!pbody ) {
                    name = pbody->GetName();
                }
                RAVELOG_WARN_FORMAT("there are %d fcl collision objects left for body %s", vcolobjs.size()%name);
            }

            pwbody.reset();
            pwinfo.reset();
            nLastStamp = 0;
            nLinkUpdateStamp = 0;
            nGeometryUpdateStamp = 0;
            nAttachedBodiesUpdateStamp = 0;
            nActiveDOFUpdateStamp = 0;
            linkEnableStates.clear();
            // vcolobjs is left as is without clearing on purpose
            // clearing should happen together with manager unregisterObject
            //vcolobjs.clear();

            geometrygroup.clear();
        }

        /// checks if weak pointer is expired (either reset or KinBody that it was pointing to is destructed
        // note, this doesn't check if KinBody has non-zero env body index, so it's not the same check as (pwbody.lock() && pwbody.lock()->GetEnvironmentBodyIndex())
        inline bool IsValid() const
        {
            return !pwbody.expired(); // expired is slightly faster than lock
        }

        KinBodyConstWeakPtr pwbody; ///< weak pointer to body
        FCLSpace::KinBodyInfoWeakPtr pwinfo; ///< weak pointer to info
        int nLastStamp; ///< copyied from FCLSpace::KinBodyInfo when body was last updated
        int nLinkUpdateStamp; ///< copied from FCLSpace::KinBodyInfo when body was last updated
        int nGeometryUpdateStamp; ///< copied from FCLSpace::KinBodyInfo when geometry was last updated
        int nAttachedBodiesUpdateStamp; /// copied from FCLSpace::KinBodyInfo when attached bodies was last updated
        int nActiveDOFUpdateStamp; ///< update stamp when the active dof changed
        std::vector<uint8_t> linkEnableStates; ///< links that are currently inside the manager
        std::vector<CollisionObjectPtr> vcolobjs; ///< collision objects used for each link (use link index). have to hold pointers so that KinBodyInfo does not remove them!
        std::string geometrygroup; ///< cached geometry group
    };

public:
    FCLCollisionManagerInstance(FCLSpace& fclspace, BroadPhaseCollisionManagerPtr pmanager) : _fclspace(fclspace), pmanager(pmanager) {
        _lastSyncTimeStamp = OpenRAVE::utils::GetMilliTime();
    }
    ~FCLCollisionManagerInstance() {
        if( _tmpSortedBuffer.size() > 0 ) {
            RAVELOG_WARN_FORMAT("_tmpSortedBuffer has left over objects %d", _tmpSortedBuffer.size());
        }
        _tmpSortedBuffer.resize(0);

        pmanager->clear();
        // should clear all vcolobjs notifying the destructor that manager has the objects unregistered
        for (KinBodyCache& cache : _vecCachedBodies) {
            cache.vcolobjs.clear();
        }
        if (_vecCachedBodies.size() > 10000) { // don't know good threshold
            RAVELOG_WARN_FORMAT("env=%d, 0x%x: vecCachedBodies size=%d, and is probably too large. This should not grow more than maximum number of bodies simultaneously present in env. Maybe there is a bug.", _fclspace.GetEnvironmentId()%this%_vecCachedBodies.size());
        }
        _vecCachedBodies.clear();
    }

    /// \brief sets up manager for body checking
    ///
    /// \param bTrackActiveDOF true if should be tracking the active dof
    void InitBodyManager(KinBodyConstPtr pbody, bool bTrackActiveDOF)
    {
        _ptrackingbody = pbody;
        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        _bTrackActiveDOF = false;
        if( bTrackActiveDOF && pbody->IsRobot() ) {
            RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(pbody);
            if( !!probot ) {
                _UpdateActiveLinks(probot);
                _bTrackActiveDOF = true;
            }
        }
        //RAVELOG_VERBOSE_FORMAT("env=%d, %x init with body %s, activedof=%d, att=%d", pbody->GetEnv()->GetId()%this%pbody->GetName()%(int)bTrackActiveDOF%attachedBodies.size());

        pmanager->clear();
        _tmpSortedBuffer.resize(0);
        // should clear all vcolobjs notifying the destructor that manager has the objects unregistered
        for (KinBodyCache& cache : _vecCachedBodies) {
            cache.vcolobjs.resize(0);
        }
        std::vector<CollisionObjectPtr> vcolobjs;
        for (KinBodyCache& bodyCache : _vecCachedBodies) {
            bodyCache.Invalidate();
        }
        int maxBodyIndex = pbody->GetEnv()->GetMaxEnvironmentBodyIndex();
        EnsureVectorSize(_vecCachedBodies, maxBodyIndex+1);

        FOREACH(itbody, attachedBodies) {
            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(**itbody);
            if( !pinfo ) {
                // don't init something that isn't initialized in this checker.
                RAVELOG_VERBOSE_FORMAT("body %s has attached body %s which is not initialized in this checker, ignoring for now", pbody->GetName()%(*itbody)->GetName());
                continue;
            }

            bool bsetUpdateStamp = false;
            _linkEnableStates.resize((*itbody)->GetLinks().size()); ///< links that are currently inside the manager
            std::fill(_linkEnableStates.begin(), _linkEnableStates.end(), 0);
            vcolobjs.clear(); // reset any existing collision objects
            vcolobjs.resize((*itbody)->GetLinks().size(),CollisionObjectPtr());
            FOREACH(itlink, (*itbody)->GetLinks()) {
                if( (*itlink)->IsEnabled() && (*itbody != pbody || !_bTrackActiveDOF || _vTrackingActiveLinks.at((*itlink)->GetIndex())) ) {
                    CollisionObjectPtr pcol = _fclspace.GetLinkBV(*pinfo, (*itlink)->GetIndex());
                    vcolobjs[(*itlink)->GetIndex()] = pcol;
                    if( !!pcol ) {
                        CollisionGroup::const_iterator it = std::lower_bound(_tmpSortedBuffer.begin(), _tmpSortedBuffer.end(), pcol.get());
                        // keep _tmpSortedBuffer sorted so that we can efficiently search
                        if (it == _tmpSortedBuffer.end() || *it != pcol.get()) {
                            _tmpSortedBuffer.insert(it, pcol.get());
                        }
                        else {
                            RAVELOG_WARN_FORMAT("env=%d(%s) body %s link %s is added multiple times", pbody->GetEnv()->GetId()%pbody->GetEnv()->GetName()%pbody->GetName()%(*itlink)->GetName());
                        }
                    }
                    bsetUpdateStamp = true;
                    _linkEnableStates.at((*itlink)->GetIndex()) = 1;
                }
            }

            // regardless if the linkmask, have to always add to cache in order to track!
            if( 1 ) {//bsetUpdateStamp ) {
                //RAVELOG_VERBOSE_FORMAT("env=%d, %x adding body %s (%d) linkmask=0x%x, _tmpSortedBuffer.size()=%d", (*itbody)->GetEnv()->GetId()%this%(*itbody)->GetName()%pbody->GetEnvironmentBodyIndex()%_GetLinkMask(_linkEnableStates)%_tmpSortedBuffer.size());
                const int bodyIndex = (*itbody)->GetEnvironmentBodyIndex();
                //EnsureVectorSize(_vecCachedBodies, bodyIndex);
                KinBodyCache& cache = _vecCachedBodies.at(bodyIndex);
                cache.Set(*itbody, pinfo);
                cache.linkEnableStates = _linkEnableStates;
                cache.vcolobjs.swap(vcolobjs);
            }
            else {
//                if( IS_DEBUGLEVEL(OpenRAVE::Level_Verbose) ) {
//                    std::stringstream ss;
//                    for(size_t ilink = 0; ilink < (*itbody)->GetLinks().size(); ++ilink) {
//                        ss << (int)(*itbody)->GetLinks()[ilink]->IsEnabled();
//                        if( pbody == *itbody ) {
//                            ss << "(" << _vTrackingActiveLinks.at(ilink) << ")";
//                        }
//                        ss << ",";
//                    }
//                    RAVELOG_VERBOSE_FORMAT("env=%d, %x not tracking adding body %s: links=[%s]", (*itbody)->GetEnv()->GetId()%this%(*itbody)->GetName()%ss.str());
//                }
            }
        }
        if( _tmpSortedBuffer.size() > 0 ) {
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
            SaveCollisionObjectDebugInfos();
#endif
            pmanager->registerObjects(_tmpSortedBuffer); // bulk update
        }
        pmanager->setup();
    }

    /// \brief sets up manager for environment checking
    void InitEnvironment(const std::set<KinBodyConstPtr>& excludedbodies)
    {
        _ptrackingbody.reset();
        _setExcludeBodyIndices.clear();
        pmanager->clear();
        for (KinBodyCache& bodyCache : _vecCachedBodies) {
            bodyCache.Invalidate();
        }

        FOREACH(itbody, excludedbodies) {
            _setExcludeBodyIndices.insert((*itbody)->GetEnvironmentBodyIndex());
        }
        pmanager->setup();
    }

    /// \brief makes sure that all the bodies are currently in the scene (if they are not explicitly excluded)
    void EnsureBodies(const std::set<KinBodyConstPtr>& vbodies)
    {
        _tmpSortedBuffer.resize(0);
        if (vbodies.empty()) {
            return;
        }
        bool ensuredVecCachedBodies = false;
        std::vector<CollisionObjectPtr> vcolobjs;
        for (const KinBodyConstPtr& pbody : vbodies) {
            const KinBody& body = *pbody;
            if (!ensuredVecCachedBodies) {
                EnsureVectorSize(_vecCachedBodies, body.GetEnv()->GetMaxEnvironmentBodyIndex() + 1);
                ensuredVecCachedBodies = true;
            }
            int bodyIndex = body.GetEnvironmentBodyIndex();
            if( _setExcludeBodyIndices.count(bodyIndex) == 0 ) {
                bool bIsValid = _vecCachedBodies.at(bodyIndex).IsValid();
                if( !bIsValid) {
                    FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(body);
                    if( _AddBody(body, pinfo, vcolobjs, _linkEnableStates, false) ) { // new collision objects are already added to _tmpSortedBuffer
                        KinBodyCache& cache = _vecCachedBodies.at(bodyIndex);
                        cache.Set(pbody, pinfo);
                        cache.vcolobjs.swap(vcolobjs);
                        cache.linkEnableStates = _linkEnableStates;
                    }
                }
            }
        }
        if( _tmpSortedBuffer.size() > 0 ) {
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
            SaveCollisionObjectDebugInfos();
#endif
            pmanager->registerObjects(_tmpSortedBuffer); // bulk update
        }
    }

    /// \brief ensures that pbody is being tracked inside the manager
//    void EnsureBody(KinBodyConstPtr pbody)
//    {
//        _tmpSortedBuffer.resize(0);
//        std::map<int, KinBodyCache>::iterator it = _vecCachedBodies.find(pbody->GetEnvironmentBodyIndex());
//        if( it == _vecCachedBodies.end() ) {
//            std::vector<CollisionObjectPtr> vcolobjs;
//            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(pbody);
//            uint64_t linkmask=0;
//            if( _AddBody(pbody, pinfo, vcolobjs, linkmask, false) ) { // new collision objects are already added to _tmpSortedBuffer
//                _vecCachedBodies[(pbody)->GetEnvironmentBodyIndex()] = KinBodyCache(pbody, pinfo);
//                _vecCachedBodies[(pbody)->GetEnvironmentBodyIndex()].vcolobjs.swap(vcolobjs);
//                _vecCachedBodies[(pbody)->GetEnvironmentBodyIndex()].linkmask = linkmask;
//            }
//        }
//        if( _tmpSortedBuffer.size() > 0 ) {
//#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
//            SaveCollisionObjectDebugInfos();
//#endif
//            pmanager->registerObjects(_tmpSortedBuffer); // bulk update
//        }
//
//    }

    /// \brief remove tracking of the body, return true if body was removed
    bool RemoveBody(const KinBody &body)
    {
        RAVELOG_VERBOSE_FORMAT("%u removing body %s", _lastSyncTimeStamp% body.GetName());

        const int bodyIndex = body.GetEnvironmentBodyIndex();
        if( (int)_vecCachedBodies.size() > bodyIndex) {
            KinBodyCache& cache = _vecCachedBodies.at(bodyIndex);
            if (cache.IsValid()) {
                for (CollisionObjectPtr& col : cache.vcolobjs) {
                    if( !!col.get() ) {
                        pmanager->unregisterObject(col.get());
                    }
                }
                cache.vcolobjs.resize(0);
                cache.Invalidate();

                return true;
            }
        }

        return false;
    }

    uint64_t _GetLinkMask(const std::vector<uint8_t>& linkEnableStates)
    {
        uint64_t linkmask=0;
        for(size_t ilink = 0; ilink < linkEnableStates.size(); ++ilink) {
            if( linkEnableStates[ilink] ) {
                linkmask |= 1 << ilink;
            }
        }
        return linkmask;
    }

    /// \brief Synchronizes the element of the manager instance whose update stamps are outdated
    void Synchronize()
    {
        _tmpSortedBuffer.resize(0);
        _lastSyncTimeStamp = OpenRAVE::utils::GetMilliTime();
        bool bcallsetup = false;
        bool bAttachedBodiesChanged = false;
        KinBodyConstPtr ptrackingbody = _ptrackingbody.lock();
        if( !!ptrackingbody && _bTrackActiveDOF ) {
            const KinBody& trackingbody = *ptrackingbody;
            EnsureVectorSize(_vecCachedBodies, trackingbody.GetEnv()->GetMaxEnvironmentBodyIndex() + 1);

            const int trackingBodyIndex = trackingbody.GetEnvironmentBodyIndex();
            KinBodyCache& trackingCache = _vecCachedBodies.at(trackingBodyIndex);
            bool isValid = trackingCache.IsValid();
            if (!isValid) {
                std::string ssinfo;
                for(int bodyIndexCached = 0; bodyIndexCached < (int)_vecCachedBodies.size(); ++bodyIndexCached) {
                    const KinBodyCache& cache = _vecCachedBodies[bodyIndexCached];
                    if( cache.IsValid() ) {
                        KinBodyConstPtr pbody = cache.pwbody.lock(); ///< weak pointer to body
                        if( !!pbody ) {
                            ssinfo += str(boost::format("(id=%d, linkmask=0x%x, numcols=%d, name=%s), ")%bodyIndexCached%_GetLinkMask(cache.linkEnableStates)%cache.vcolobjs.size()%pbody->GetName());
                        }
                        else {
                            ssinfo += str(boost::format("id=%d, linkmask=0x%x, numcols=%d")%bodyIndexCached%_GetLinkMask(cache.linkEnableStates)%cache.vcolobjs.size());
                        }
                    }
                }
                RAVELOG_WARN_FORMAT("%x tracking body not in current cached bodies (valid=%d) (tracking body %s (id=%d)) (env %d). Current cache is: %s", this%isValid%trackingbody.GetName()%trackingBodyIndex%trackingbody.GetEnv()->GetId()%ssinfo);
            }
            else {
                FCLSpace::KinBodyInfoPtr pinfo = trackingCache.pwinfo.lock();
                FCLSpace::KinBodyInfoPtr pnewinfo = _fclspace.GetInfo(trackingbody); // necessary in case pinfos were swapped!
                if( trackingCache.nActiveDOFUpdateStamp != pnewinfo->nActiveDOFUpdateStamp ) {
                    if( trackingbody.IsRobot() ) {
                        RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(ptrackingbody);
                        if( !!probot ) {
                            if( pinfo != pnewinfo ) {
                                // going to recreate everything in below step anyway, so no need to update collision objects
                                _UpdateActiveLinks(probot);
                            }
                            else {
                                // check for any tracking link changes
                                _vTrackingActiveLinks.resize(probot->GetLinks().size(), 0);
                                // the active links might have changed
                                _linkEnableStates.resize(probot->GetLinks().size()); ///< links that are currently inside the manager
                                std::fill(_linkEnableStates.begin(), _linkEnableStates.end(), 0);
                                for(size_t ilink = 0; ilink < probot->GetLinks().size(); ++ilink) {
                                    int isLinkActive = 0;
                                    FOREACH(itindex, probot->GetActiveDOFIndices()) {
                                        if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(), ilink) ) {
                                            isLinkActive = 1;
                                            break;
                                        }
                                    }

                                    bool bIsActiveLinkEnabled = probot->GetLinks()[ilink]->IsEnabled() && isLinkActive;

                                    if( bIsActiveLinkEnabled ) {
                                        _linkEnableStates.at(ilink) = 1;
                                    }
                                    if( _vTrackingActiveLinks[ilink] != isLinkActive ) {
                                        _vTrackingActiveLinks[ilink] = isLinkActive;
                                        CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pnewinfo, probot->GetLinks()[ilink]->GetIndex());
                                        if( bIsActiveLinkEnabled && !!pcolobj ) {
#ifdef FCLRAVE_USE_REPLACEOBJECT
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                            SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                            if( !!trackingCache.vcolobjs.at(ilink) ) {
                                                pmanager->replaceObject(trackingCache.vcolobjs.at(ilink).get(), pcolobj.get(), false);
                                            }
                                            else {
                                                pmanager->registerObject(pcolobj.get());
                                            }
#else
                                            // no replace
                                            if( !!trackingCache.vcolobjs.at(ilink) ) {
                                                pmanager->unregisterObject(trackingCache.vcolobjs.at(ilink).get());
                                            }
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                            SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                            pmanager->registerObject(pcolobj.get());
#endif
                                            bcallsetup = true;
                                        }
                                        else {
                                            if( !!trackingCache.vcolobjs.at(ilink) ) {
                                                pmanager->unregisterObject(trackingCache.vcolobjs.at(ilink).get());
                                            }
                                        }
                                        trackingCache.vcolobjs.at(ilink) = pcolobj;
                                    }
                                    else {
                                        if( !bIsActiveLinkEnabled && !!trackingCache.vcolobjs.at(ilink) ) {
                                            //RAVELOG_VERBOSE_FORMAT("env=%d %x resetting cached colobj %s %d", probot->GetEnv()->GetId()%this%probot->GetName()%ilink);
                                            pmanager->unregisterObject(trackingCache.vcolobjs.at(ilink).get());
                                            trackingCache.vcolobjs.at(ilink).reset();
                                        }
                                    }
                                }

                                trackingCache.nActiveDOFUpdateStamp = pnewinfo->nActiveDOFUpdateStamp;
                                trackingCache.linkEnableStates = _linkEnableStates;
                            }
                        }
                    }
                }
            }
        }


        for (KinBodyCache& cache : _vecCachedBodies) {
            KinBodyConstPtr pbody = cache.pwbody.lock();
            if( !pbody || pbody->GetEnvironmentBodyIndex() == 0 ) {
                // should happen when parts are removed
                // RAVELOG_VERBOSE_FORMAT("env=%d, %u manager contains invalid body %s, removing for now", _fclspace.GetEnvironmentId()%_lastSyncTimeStamp%(!pbody ? std::string() : pbody->GetName()));
                FOREACH(itcolobj, cache.vcolobjs) {
                    if( !!itcolobj->get() ) {
                        pmanager->unregisterObject(itcolobj->get());
                    }
                }
                cache.vcolobjs.resize(0);
                cache.Invalidate();
                continue;
            }

            FCLSpace::KinBodyInfoPtr pinfo = cache.pwinfo.lock();
            const KinBody& body = *pbody;
            FCLSpace::KinBodyInfoPtr pnewinfo = _fclspace.GetInfo(body); // necessary in case pinfos were swapped!
            if( pinfo != pnewinfo ) {
                // everything changed!
                RAVELOG_VERBOSE_FORMAT("%u body %s entire KinBodyInfo changed", _lastSyncTimeStamp%pbody->GetName());
                FOREACH(itcolobj, cache.vcolobjs) {
                    if( !!itcolobj->get() ) {
                        pmanager->unregisterObject(itcolobj->get());
                    }
                }
                cache.vcolobjs.resize(0);
                _AddBody(body, pnewinfo, cache.vcolobjs, cache.linkEnableStates, _bTrackActiveDOF&&pbody==ptrackingbody);
                cache.pwinfo = pnewinfo;
                //cache.ResetStamps();
                // need to update the stamps here so that we do not try to unregisterObject below and get into an error
                cache.nLastStamp = pnewinfo->nLastStamp;
                cache.nLinkUpdateStamp = pnewinfo->nLinkUpdateStamp;
                cache.nGeometryUpdateStamp = pnewinfo->nGeometryUpdateStamp;
                cache.nAttachedBodiesUpdateStamp = -1;
                cache.nActiveDOFUpdateStamp = pnewinfo->nActiveDOFUpdateStamp;
                cache.geometrygroup = pnewinfo->_geometrygroup;
                pinfo = pnewinfo;
                if( _tmpSortedBuffer.size() > 0 ) {
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                    SaveCollisionObjectDebugInfos();
#endif
                    pmanager->registerObjects(_tmpSortedBuffer); // bulk update
                    _tmpSortedBuffer.resize(0);
                }
            }

            if( pinfo->nLinkUpdateStamp != cache.nLinkUpdateStamp ) {
                // links changed
                std::vector<uint8_t> newLinkEnableStates;
                body.GetLinkEnableStates(newLinkEnableStates);
                if( _bTrackActiveDOF && ptrackingbody == pbody ) {
                    for(size_t itestlink = 0; itestlink < _vTrackingActiveLinks.size(); ++itestlink) {
                        if( !_vTrackingActiveLinks[itestlink] ) {
                            newLinkEnableStates.at(itestlink) = 0;
                        }
                    }
                }

                //uint64_t changed = cache.linkmask ^ newlinkmask;
                //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), lastsync=%u body %s (%d) for cache changed link %d != %d, linkmask=0x%x", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%body.GetName()%body.GetEnvironmentBodyIndex()%pinfo->nLinkUpdateStamp%cache.nLinkUpdateStamp%_GetLinkMask(newLinkEnableStates));
                for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                    uint8_t changed = cache.linkEnableStates.at(ilink) != newLinkEnableStates.at(ilink);
                    if( changed ) {
                        if( newLinkEnableStates.at(ilink) ) {
                            CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pinfo, ilink);
                            if( !!pcolobj ) {
#ifdef FCLRAVE_USE_REPLACEOBJECT
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                if( !!cache.vcolobjs.at(ilink) ) {
                                    pmanager->replaceObject(cache.vcolobjs.at(ilink).get(), pcolobj.get(), false);
                                }
                                else {
                                    pmanager->registerObject(pcolobj.get());
                                }
#else

                                // no replace
                                if( !!cache.vcolobjs.at(ilink) ) {
                                    pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                                }
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                pmanager->registerObject(pcolobj.get());
#endif
                                bcallsetup = true;
                            }
                            else {
                                if( !!cache.vcolobjs.at(ilink) ) {
                                    pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                                }
                            }
                            cache.vcolobjs.at(ilink) = pcolobj;
                        }
                        else {
                            if( !!cache.vcolobjs.at(ilink) ) {
                                pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                                cache.vcolobjs.at(ilink).reset();
                            }
                        }
                    }
                }

                cache.linkEnableStates = newLinkEnableStates;
                cache.nLinkUpdateStamp = pinfo->nLinkUpdateStamp;
            }
            if( pinfo->nGeometryUpdateStamp != cache.nGeometryUpdateStamp ) {

                if( cache.geometrygroup.size() == 0 || cache.geometrygroup != pinfo->_geometrygroup ) {
                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), lastsync=%u body %s (%d) for cache changed geometry %d != %d, linkmask=0x%x", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%body.GetName()%body.GetEnvironmentBodyIndex()%pinfo->nGeometryUpdateStamp%cache.nGeometryUpdateStamp%_GetLinkMask(cache.linkEnableStates));
                    // vcolobjs most likely changed
                    for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                        if( cache.linkEnableStates.at(ilink) ) {
                            CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pinfo, ilink);
                            if( !!pcolobj ) {
                                //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s adding obj %x from link %d", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%pcolobj.get()%ilink);
#ifdef FCLRAVE_USE_REPLACEOBJECT
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                if( !!cache.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s replacing cached obj %x with %x ", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%cache.vcolobjs.at(ilink).get()%pcolobj.get());
                                    pmanager->replaceObject(cache.vcolobjs.at(ilink).get(), pcolobj.get(), false);
                                }
                                else {
                                    pmanager->registerObject(pcolobj.get());
                                }
#else

                                // no replace
                                if( !!cache.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s unregister cached obj %x ", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%cache.vcolobjs.at(ilink).get());
                                    fcl::CollisionObject* ptestobj = cache.vcolobjs.at(ilink).get();
                                    pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                                }
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                pmanager->registerObject(pcolobj.get());
#endif
                                bcallsetup = true;
                                cache.vcolobjs.at(ilink) = pcolobj;
                            }
                            else {
                                if( !!cache.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x, body %s removing old obj from link %d", body.GetEnv()->GetId()%this%body.GetName()%ilink);
                                    pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                                    cache.vcolobjs.at(ilink).reset();
                                }
                            }
                        }
                    }
                    cache.geometrygroup = pinfo->_geometrygroup;
                }
                else {
                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x, lastsync=%u body %s (%d) for cache changed geometry but no update %d != %d, geometrygroup=%s", body.GetEnv()->GetId()%this%_lastSyncTimeStamp%body.GetName()%body.GetEnvironmentBodyIndex()%pinfo->nGeometryUpdateStamp%cache.nGeometryUpdateStamp%cache.geometrygroup);
                }
                cache.nGeometryUpdateStamp = pinfo->nGeometryUpdateStamp;
            }
            if( pinfo->nLastStamp != cache.nLastStamp ) {
                if( IS_DEBUGLEVEL(OpenRAVE::Level_Verbose) ) {
                    //Transform tpose = body.GetTransform();
                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d) %u body %s (%for cache changed transform %d != %d, num=%d, mask=0x%x, trans=(%.3f, %.3f, %.3f)", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%body.GetName()%body.GetEnvironmentBodyIndex()%pinfo->nLastStamp%cache.nLastStamp%pinfo->vlinks.size()%_GetLinkMask(cache.linkEnableStates)%tpose.trans.x%tpose.trans.y%tpose.trans.z);
                }
                // transform changed
                for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                    if( cache.linkEnableStates.at(ilink) ) {
                        CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pinfo, ilink);
                        if( !!pcolobj ) {
                            //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s adding obj %x from link %d", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%pcolobj.get()%ilink);
                            if( cache.vcolobjs.at(ilink) == pcolobj ) {
#ifdef FCLRAVE_USE_BULK_UPDATE
                                // same object, so just update
                                pmanager->update(cache.vcolobjs.at(ilink).get(), false);
#else
                                // Performance issue !!
                                pmanager->update(cache.vcolobjs.at(ilink).get());
#endif
                            }
                            else {
#ifdef FCLRAVE_USE_REPLACEOBJECT
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif

                                // different object, have to replace
                                if( !!cache.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s replacing cached obj %x with %x ", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%cache.vcolobjs.at(ilink).get()%pcolobj.get());
                                    pmanager->replaceObject(cache.vcolobjs.at(ilink).get(), pcolobj.get(), false);
                                }
                                else {
                                    pmanager->registerObject(pcolobj.get());
                                }
#else // FCLRAVE_USE_REPLACEOBJECT

                                // no replace
                                if( !!cache.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s unregister cached obj %x ", body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecke()%body.GetName()%cache.vcolobjs.at(ilink).get());
                                    fcl::CollisionObject* ptestobj = cache.vcolobjs.at(ilink).get();
                                    pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                                }
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                pmanager->registerObject(pcolobj.get());
#endif // FCLRAVE_USE_REPLACEOBJECT
                            }

                            bcallsetup = true;
                            cache.vcolobjs.at(ilink) = pcolobj;
                        }
                        else {
                            if( !!cache.vcolobjs.at(ilink) ) {
                                //RAVELOG_VERBOSE_FORMAT("env=%d, %x, body %s removing old obj from link %d", body.GetEnv()->GetId()%this%body.GetName()%ilink);
                                pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                                cache.vcolobjs.at(ilink).reset();
                            }
                        }
                    }
                }

                cache.nLastStamp = pinfo->nLastStamp;
            }
            if( pinfo->nAttachedBodiesUpdateStamp != cache.nAttachedBodiesUpdateStamp ) {
                //RAVELOG_VERBOSE_FORMAT("env=%d, %u the nAttachedBodiesUpdateStamp changed %d != %d, trackingbody is %s", body.GetEnv()->GetId()%_lastSyncTimeStamp%pinfo->nAttachedBodiesUpdateStamp%cache.nAttachedBodiesUpdateStamp%(!!ptrackingbody ? trackingbody.GetName() : std::string()));
                // bodies changed!
                if( !!ptrackingbody ) {
                    bAttachedBodiesChanged = true;
                }

                cache.nAttachedBodiesUpdateStamp = pinfo->nAttachedBodiesUpdateStamp;
            }
        }

        if( bAttachedBodiesChanged && !!ptrackingbody ) {
            // since tracking have to update all the bodies
            std::set<KinBodyConstPtr> attachedBodies;
            ptrackingbody->GetAttached(attachedBodies);
            std::vector<CollisionObjectPtr> vcolobjs;
            //RAVELOG_VERBOSE_FORMAT("env=%d, %x %u setting %d attached bodies for body %s (%d)", ptrackingbody->GetEnv()->GetId()%this%_lastSyncTimeStamp%attachedBodies.size()%ptrackingbody->GetName()%ptrackingbody->GetEnvironmentBodyIndex());
            // add any new bodies
            bool vectorSizeIsEnsured = false;
            for (const KinBodyConstPtr& pattached : attachedBodies) {
                const KinBody& attached = *pattached;
                if (!vectorSizeIsEnsured) {
                    EnsureVectorSize(_vecCachedBodies, attached.GetEnv()->GetMaxEnvironmentBodyIndex() + 1);
                    vectorSizeIsEnsured = true;
                }
                const int attachedBodyIndex = attached.GetEnvironmentBodyIndex();

                KinBodyCache& cache = _vecCachedBodies.at(attachedBodyIndex);
                if (cache.IsValid()) {
                    continue;
                }

                FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(attached);
                vcolobjs.resize(0);
                if( _AddBody(attached, pinfo, vcolobjs, _linkEnableStates, _bTrackActiveDOF&&(pattached == ptrackingbody)) ) {
                    bcallsetup = true;
                }
                //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d) %u adding body %s (%d)", cache.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%cache.GetName()%attachedBodyIndex);
                cache.Set(pattached, pinfo);
                cache.vcolobjs.swap(vcolobjs);
                cache.linkEnableStates = _linkEnableStates;
                //                    else {
                //                        RAVELOG_VERBOSE_FORMAT("env=%d %x could not add attached body %s for tracking body %s", trackingbody.GetEnv()->GetId()%this%cache.GetName()%trackingbody.GetName());
                //                    }
            }

            // remove bodies not attached anymore

            for(int bodyIndex = 0; bodyIndex < (int)_vecCachedBodies.size(); ++bodyIndex) {
                KinBodyCache& cache = _vecCachedBodies[bodyIndex];
                KinBodyConstPtr pbody = cache.pwbody.lock();
                // could be the case that the same pointer was re-added to the environment so have to check the environment id
                // make sure we don't need to make this asusmption of body id change when bodies are removed and re-added
                if( !pbody || attachedBodies.count(pbody) == 0 || pbody->GetEnvironmentBodyIndex() != bodyIndex ) {
                    if( !!pbody && IS_DEBUGLEVEL(OpenRAVE::Level_Verbose) ) {
                        RAVELOG_VERBOSE_FORMAT("env=%d, %x, %u removing old cache %d", pbody->GetEnv()->GetId()%this%_lastSyncTimeStamp%bodyIndex);
                    }
                    // not in attached bodies so should remove
                    FOREACH(itcol, cache.vcolobjs) {
                        if( !!itcol->get() ) {
                            pmanager->unregisterObject(itcol->get());
                        }
                    }
                    cache.vcolobjs.resize(0);
                    cache.Invalidate();
                }
            }
        }

        if( _tmpSortedBuffer.size() > 0 ) {
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
            SaveCollisionObjectDebugInfos();
#endif
            pmanager->registerObjects(_tmpSortedBuffer); // bulk update
        }
        if( bcallsetup ) {
            pmanager->setup();
        }
    }

    inline BroadPhaseCollisionManagerPtr GetManager() const {
        return pmanager;
    }

    inline uint32_t GetLastSyncTimeStamp() const {
        return _lastSyncTimeStamp;
    }

    inline const std::set<int>& GetExcludeBodyIndices() const {
        return _setExcludeBodyIndices;
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
    ///
    /// should not add anything to _vecCachedBodies! insert to _tmpSortedBuffer
    bool _AddBody(const KinBody& body, const FCLSpace::KinBodyInfoPtr& pinfo, std::vector<CollisionObjectPtr>& vcolobjs, std::vector<uint8_t>& linkEnableStates, bool bTrackActiveDOF)
    {
        vcolobjs.resize(0); // reset so that existing collision objects can go away
        vcolobjs.resize(body.GetLinks().size(), CollisionObjectPtr());
        bool bsetUpdateStamp = false;
        body.GetLinkEnableStates(linkEnableStates);
        for (const KinBody::LinkPtr& plink : body.GetLinks()) {
            const KinBody::Link& link = *plink;
            const int linkIndex = link.GetIndex();
            linkEnableStates[linkIndex] &= (!bTrackActiveDOF || _vTrackingActiveLinks.at(linkIndex));
            if( linkEnableStates[linkIndex]) {
                //pinfo->vlinks.at(linkIndex).listRegisteredManagers.push_back(shared_from_this());

                CollisionObjectPtr pcol = _fclspace.GetLinkBV(*pinfo, linkIndex);
                if( !!pcol ) {
                    CollisionGroup::const_iterator it = std::lower_bound(_tmpSortedBuffer.begin(), _tmpSortedBuffer.end(), pcol.get());
                    // keep _tmpSortedBuffer sorted so that we can efficiently search
                    if (it == _tmpSortedBuffer.end() || *it != pcol.get()) {
                        _tmpSortedBuffer.insert(it, pcol.get());
                    }
                    else {
                        RAVELOG_WARN_FORMAT("env=%d(%s) body %s link %s is added multiple times", body.GetEnv()->GetId()%body.GetEnv()->GetName()%body.GetName()%link.GetName());
                    }
                    vcolobjs[linkIndex] = pcol;
                    bsetUpdateStamp = true;
                }
            }
        }
        return bsetUpdateStamp;
    }

    void _UpdateActiveLinks(RobotBaseConstPtr probot)
    {
        _vTrackingActiveLinks.resize(probot->GetLinks().size());
        for(size_t i = 0; i < probot->GetLinks().size(); ++i) {
            int isLinkActive = 0;
            FOREACH(itindex, probot->GetActiveDOFIndices()) {
                if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(), i) ) {
                    isLinkActive = 1;
                    break;
                }
            }
            _vTrackingActiveLinks[i] = isLinkActive;
        }
    }

//    void CheckCount()
//    {
//        // count how many entries
//        std::vector<fcl::CollisionObject*> vobjs;
//        pmanager->getObjects(vobjs);
//        FOREACH(itobj, vobjs) {
//            if( *itobj == 0 ) {
//                continue;
//            }
//            int c = CountForObj(((fcl::DynamicAABBTreeCollisionManager*)pmanager.get())->getTree().getRoot(), *itobj);
//            if( c != 1 ) {
//                RAVELOG_WARN("asdfsadfasdf\n");
//            }
//        }
//    }
//
//    bool CheckForObjInManager(fcl::CollisionObject* pobj)
//    {
//        CheckCount();
//        bool bexists = CheckForObj(((fcl::DynamicAABBTreeCollisionManager*)pmanager.get())->getTree().getRoot(), pobj);
//        if( bexists ) {
//            RAVELOG_WARN("should not be in\n");
//        }
//        return bexists;
//    }


    FCLSpace& _fclspace; ///< reference for speed
    BroadPhaseCollisionManagerPtr pmanager;
    std::vector<KinBodyCache> _vecCachedBodies; ///< vector of KinBodyCache(weak body, updatestamp)) where index is KinBody::GetEnvironmentBodyIndex. Index 0 has invalid entry because valid env id starts from 1.
    uint32_t _lastSyncTimeStamp; ///< timestamp when last synchronized

    std::set<int> _setExcludeBodyIndices; ///< any bodies that should not be considered inside the manager, used with environment mode
    CollisionGroup _tmpSortedBuffer; ///< cache, sorted so that we can efficiently search

    KinBodyConstWeakPtr _ptrackingbody; ///< if set, then only tracking the attached bodies if this body
    std::vector<int> _vTrackingActiveLinks; ///< indices of which links are active for tracking body
    std::vector<uint8_t> _linkEnableStates; ///< links that are currently inside the manager

    bool _bTrackActiveDOF; ///< if true and _ptrackingbody is valid, then should be tracking the active dof of the _ptrackingbody

#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
    void SaveCollisionObjectDebugInfos() {
        FOREACH(itpcollobj, _tmpSortedBuffer) {
            SaveCollisionObjectDebugInfos(*itpcollobj);
        }
    }

    void SaveCollisionObjectDebugInfos(fcl::CollisionObject* pcollobj) {
        FCLSpace::KinBodyInfo::LinkInfo* pLINK = static_cast<FCLSpace::KinBodyInfo::LinkInfo*>(pcollobj->getUserData());
        _mapDebugCollisionObjects.insert(std::make_pair(pcollobj, std::make_pair(pLINK->bodylinkname, _fclspace.GetInfo(pLINK->GetLink()->GetParent())->_geometrygroup)));
    }

    std::map< fcl::CollisionObject*, std::pair<std::string, std::string> > _mapDebugCollisionObjects;
#endif
};

typedef boost::shared_ptr<FCLCollisionManagerInstance> FCLCollisionManagerInstancePtr;
typedef boost::weak_ptr<FCLCollisionManagerInstance> FCLCollisionManagerInstanceWeakPtr;

} // end namespace fclrave

#endif
