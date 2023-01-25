#include "fclmanagercache.h"

namespace ivshmem {

FCLCollisionManagerInstance::KinBodyCache::KinBodyCache()
    : nLastStamp(0)
    , nLinkUpdateStamp(0)
    , nGeometryUpdateStamp(0)
    , nAttachedBodiesUpdateStamp(0)
    , nActiveDOFUpdateStamp(0) {}

FCLCollisionManagerInstance::KinBodyCache::KinBodyCache(const KinBodyConstPtr& pbody, const FCLSpace::FCLKinBodyInfoPtr& pinfo) {
    std::vector<uint64_t> linkEnableStateCacheDummy;
    SetBodyData(pbody, pinfo, linkEnableStateCacheDummy);
}

void FCLCollisionManagerInstance::KinBodyCache::SetBodyData(
    const KinBodyConstPtr& pbody,
    const FCLSpace::FCLKinBodyInfoPtr& pinfo,
    const std::vector<uint64_t>& linkEnableStateCache) {
    pwbody = pbody;
    pwinfo = pinfo;
    nLastStamp = pinfo->nLastStamp;
    nLinkUpdateStamp = pinfo->nLinkUpdateStamp;
    nGeometryUpdateStamp = pinfo->nGeometryUpdateStamp;

    geometrygroup = pinfo->_geometrygroup;
    nAttachedBodiesUpdateStamp = pinfo->nAttachedBodiesUpdateStamp;
    nActiveDOFUpdateStamp = pinfo->nActiveDOFUpdateStamp;
    if (linkEnableStateCache.empty()) {
        linkEnableStatesBitmasks = pbody->GetLinkEnableStatesMasks();
    } else {
        linkEnableStatesBitmasks = linkEnableStateCache;
    }
}

void FCLCollisionManagerInstance::KinBodyCache::Invalidate(bool warnOnNonEmptyColObjs) {
    if (!IsValid()) {
        // RAVELOG_INFO_FORMAT("0x%x is previously invalidated, or was never valid.", this);
        return;
    }
    if (!vcolobjs.empty() && warnOnNonEmptyColObjs) { // should never happen
        std::stringstream ss;
        KinBodyConstPtr pbody = pwbody.lock();
        std::string bodyName("unknown_body");
        std::string envNameId("unknown_env");
        if (!!pbody) {
            bodyName = pbody->GetName();
            envNameId = pbody->GetEnv()->GetNameId();
        }
        std::string unknown_geomgroup("unknown_geomgroup");
        const FCLSpace::FCLKinBodyInfoPtr pinfo = pwinfo.lock();
        if (!!pinfo) {
            unknown_geomgroup = pinfo->_geometrygroup;
        }
        ss << "env=" << envNameId << ", "
           << "body=" << bodyName << ", "
           << "geomgroup=\"" << unknown_geomgroup << "\", "
           << "FCLCollisionManagerInstance 0x" << hex << this << " has " << dec << vcolobjs.size() << " collision objects (";
        for (const CollisionObjectPtr& obj : vcolobjs) {
            ss << "0x" << hex << obj << ", ";
            if (!!obj) {
                const fcl::Quaternion3f& q = obj->getQuatRotation();
                const fcl::Vec3f& t = obj->getTranslation();
                const fcl::AABB& aabb = obj->getAABB();
                ss << "pose=[" << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," << t[0] << "," << t[1] << "," << t[2] << "]; "
                   << "aabb_min=[" << aabb.min_[0] << "," << aabb.min_[1] << "," << aabb.min_[2] << "]; "
                   << "aabb_max=[" << aabb.max_[0] << "," << aabb.max_[1] << "," << aabb.max_[2] << "]; ";
            }
        }
        ss << "), but leaving untouched.";
        RAVELOG_WARN_FORMAT("%s", ss.str());
    }

    pwbody.reset();
    pwinfo.reset();
    nLastStamp = 0;
    nLinkUpdateStamp = 0;
    nGeometryUpdateStamp = 0;
    nAttachedBodiesUpdateStamp = 0;
    nActiveDOFUpdateStamp = 0;
    linkEnableStatesBitmasks.clear();
    // vcolobjs is left as is without clearing on purpose
    // clearing should happen together with manager unregisterObject
    // vcolobjs.clear();

    geometrygroup.clear();
}

FCLCollisionManagerInstance::FCLCollisionManagerInstance(FCLSpace& fclspace, BroadPhaseCollisionManagerPtr pmanager_)
    : _fclspace(fclspace)
    , pmanager(pmanager_)
    , _bTrackActiveDOF(false) {
    _lastSyncTimeStamp = OpenRAVE::utils::GetMilliTime();
}

FCLCollisionManagerInstance::~FCLCollisionManagerInstance() {
    if (_tmpSortedBuffer.size() > 0) {
        RAVELOG_INFO_FORMAT(
            "env=%s 0x%x: _tmpSortedBuffer has left over objects %d, maybe EnsureBodies was never called",
            _fclspace.GetEnvironmentId() % this % _tmpSortedBuffer.size());
    }
    _tmpSortedBuffer.resize(0);

    pmanager->clear();
    // should clear all vcolobjs notifying the destructor that manager has the objects unregistered
    for (KinBodyCache& cache : _vecCachedBodies) {
        cache.vcolobjs.clear();
    }
    if (_vecCachedBodies.size() > 10000) { // don't know good threshold
        RAVELOG_WARN_FORMAT(
            "env=%d, 0x%x: vecCachedBodies size=%d, and is probably too large. This should not grow more than maximum number of bodies simultaneously present "
            "in env. Maybe there is a bug.",
            _fclspace.GetEnvironmentId() % this % _vecCachedBodies.size());
    }
    _vecCachedBodies.clear();
}

void FCLCollisionManagerInstance::InitBodyManager(KinBodyConstPtr pbody, bool bTrackActiveDOF) {
    _ptrackingbody = pbody;
    _bTrackActiveDOF = false;
    if (bTrackActiveDOF && pbody->IsRobot()) {
        RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(pbody);
        if (!!probot) {
            _UpdateActiveLinks(*probot);
            _bTrackActiveDOF = true;
        }
    }
    // RAVELOG_VERBOSE_FORMAT("env=%d, %x init with body %s, activedof=%d, att=%d",
    // pbody->GetEnv()->GetId()%this%pbody->GetName()%(int)bTrackActiveDOF%attachedBodies.size());

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
    const EnvironmentBase& env = *pbody->GetEnv();
    int maxBodyIndex = env.GetMaxEnvironmentBodyIndex();
    EnsureVectorSize(_vecCachedBodies, maxBodyIndex + 1);

    std::vector<KinBodyPtr> vecAttachedBodies;
    std::vector<int> vecAttachedEnvBodyIndices;
    pbody->GetAttachedEnvironmentBodyIndices(vecAttachedEnvBodyIndices);
    env.GetBodiesFromEnvironmentBodyIndices(vecAttachedEnvBodyIndices, vecAttachedBodies);

    std::vector<KinBodyPtr>::const_iterator itrAttached = vecAttachedBodies.begin();
    for (int envBodyIndex : vecAttachedEnvBodyIndices) {
        const KinBodyPtr& pAttachedBody = *itrAttached++;
        const KinBody& attachedBody = *pAttachedBody;
        const FCLSpace::FCLKinBodyInfoPtr& pinfo = _fclspace.GetInfo(attachedBody);
        if (!pinfo) {
            // don't init something that isn't initialized in this checker.
            RAVELOG_VERBOSE_FORMAT(
                "body %s has attached body %s which is not initialized in this checker, ignoring for now", pbody->GetName() % attachedBody.GetName());
            continue;
        }

        bool bsetUpdateStamp = false;
        _linkEnableStatesBitmasks = attachedBody.GetLinkEnableStatesMasks();
        vcolobjs.clear(); // reset any existing collision objects
        vcolobjs.resize(attachedBody.GetLinks().size(), CollisionObjectPtr());
        for (const KinBody::LinkPtr& plink : attachedBody.GetLinks()) {
            const KinBody::Link& link = *plink;
            const int linkIndex = link.GetIndex();
            if (OpenRAVE::IsLinkStateBitEnabled(_linkEnableStatesBitmasks, linkIndex)) {
                if (pAttachedBody != pbody || !_bTrackActiveDOF || _vTrackingActiveLinks.at(linkIndex)) {
                    CollisionObjectPtr pcol = _fclspace.GetLinkBV(*pinfo, linkIndex);
                    vcolobjs[linkIndex] = pcol;
                    if (!!pcol) {
                        fcl::CollisionObject* pcolObj = pcol.get();
                        CollisionGroup::const_iterator it = std::lower_bound(_tmpSortedBuffer.begin(), _tmpSortedBuffer.end(), pcolObj);
                        // keep _tmpSortedBuffer sorted so that we can efficiently search
                        if (it == _tmpSortedBuffer.end() || *it != pcolObj) {
                            _tmpSortedBuffer.insert(it, pcolObj);
                        } else {
                            RAVELOG_WARN_FORMAT(
                                "env=%s body %s link %s is added multiple times", pbody->GetEnv()->GetNameId() % pbody->GetName() % link.GetName());
                        }
                    }
                    bsetUpdateStamp = true;
                } else {
                    OpenRAVE::DisableLinkStateBit(_linkEnableStatesBitmasks, linkIndex);
                }
            }
        }

        // regardless if the linkmask, have to always add to cache in order to track!
        if (1) { // bsetUpdateStamp ) {
            // RAVELOG_VERBOSE_FORMAT("env=%d, %x adding body %s (%d) linkmask=0x%x, _tmpSortedBuffer.size()=%d",
            // attachedBody.GetEnv()->GetId()%this%attachedBody.GetName()%pbody->GetEnvironmentBodyIndex()%_GetLinkMask(_linkEnableStatesBitmasks)%_tmpSortedBuffer.size());
            // EnsureVectorSize(_vecCachedBodies, bodyIndex);
            KinBodyCache& cache = _vecCachedBodies.at(envBodyIndex);
            cache.SetBodyData(pAttachedBody, pinfo, _linkEnableStatesBitmasks);
            cache.vcolobjs.swap(vcolobjs);
        } else {
            // if( IS_DEBUGLEVEL(OpenRAVE::Level_Verbose) ) {
            //     std::stringstream ss;
            //     for(size_t ilink = 0; ilink < attachedBody.GetLinks().size(); ++ilink) {
            //         ss << (int)attachedBody.GetLinks()[ilink]->IsEnabled();
            //         if( pbody == pAttachedBody ) {
            //             ss << "(" << _vTrackingActiveLinks.at(ilink) << ")";
            //         }
            //         ss << ",";
            //     }
            //     RAVELOG_VERBOSE_FORMAT("env=%d, %x not tracking adding body %s: links=[%s]",
            //     attachedBody.GetEnv()->GetId()%this%attachedBody.GetName()%ss.str());
            // }
        }
    }
    if (_tmpSortedBuffer.size() > 0) {
        pmanager->registerObjects(_tmpSortedBuffer); // bulk update
    }
    pmanager->setup();
}

void FCLCollisionManagerInstance::InitEnvironment(const std::vector<int8_t>& excludedEnvBodyIndices) {
    _ptrackingbody.reset();
    pmanager->clear();
    for (KinBodyCache& bodyCache : _vecCachedBodies) {
        bodyCache.Invalidate();
    }

    _vecExcludeBodyIndices = excludedEnvBodyIndices;
    pmanager->setup();
}

void FCLCollisionManagerInstance::EnsureBodies(const std::vector<KinBodyConstPtr>& vbodies) {
    _tmpSortedBuffer.resize(0);
    if (vbodies.empty()) {
        return;
    }
    bool ensuredVecCachedBodies = false;
    std::vector<CollisionObjectPtr> vcolobjs;
    for (const KinBodyConstPtr& pbody : vbodies) {
        if (!pbody) {
            continue;
        }
        const KinBody& body = *pbody;
        if (!ensuredVecCachedBodies) {
            EnsureVectorSize(_vecCachedBodies, body.GetEnv()->GetMaxEnvironmentBodyIndex() + 1);
            ensuredVecCachedBodies = true;
        }
        int bodyIndex = body.GetEnvironmentBodyIndex();
        if (bodyIndex >= (int)_vecExcludeBodyIndices.size() || !_vecExcludeBodyIndices[bodyIndex]) {
            bool bIsValid = _vecCachedBodies.at(bodyIndex).IsValid();
            if (!bIsValid) {
                const FCLSpace::FCLKinBodyInfoPtr& pinfo = _fclspace.GetInfo(body);
                if (_AddBody(body, pinfo, vcolobjs, _linkEnableStatesBitmasks, false)) { // new collision objects are already added to _tmpSortedBuffer
                    KinBodyCache& cache = _vecCachedBodies.at(bodyIndex);
                    cache.SetBodyData(pbody, pinfo, _linkEnableStatesBitmasks);
                    cache.vcolobjs.swap(vcolobjs);
                }
            }
        }
    }
    if (_tmpSortedBuffer.size() > 0) {
        pmanager->registerObjects(_tmpSortedBuffer); // bulk update
    }
}

bool FCLCollisionManagerInstance::RemoveBody(const KinBody& body) {
    RAVELOG_VERBOSE_FORMAT("%u removing body %s", _lastSyncTimeStamp % body.GetName());

    const int bodyIndex = body.GetEnvironmentBodyIndex();
    if ((int)_vecCachedBodies.size() > bodyIndex && bodyIndex > 0) {
        KinBodyCache& cache = _vecCachedBodies.at(bodyIndex);
        if (cache.IsValid()) {
            for (CollisionObjectPtr& col : cache.vcolobjs) {
                if (!!col.get()) {
                    pmanager->unregisterObject(col.get());
                }
            }
            cache.vcolobjs.resize(0);
            cache.Invalidate();

            return true;
        } else {
            RAVELOG_VERBOSE_FORMAT(
                "env=%s 0x%x body %s(envBodyIndex=%d) is invalidated in cache, maybe previously removed or never added",
                body.GetEnv()->GetNameId() % this % body.GetName() % bodyIndex);
        }
    } else {
        RAVELOG_VERBOSE_FORMAT(
            "env=%s body %s has invalid envBodyIndex=%d, cache size is %d", body.GetEnv()->GetNameId() % body.GetName() % bodyIndex % _vecCachedBodies.size());
    }
    return false;
}

/// \brief Synchronizes the element of the manager instance whose update stamps are outdated
void FCLCollisionManagerInstance::Synchronize() {
    _tmpSortedBuffer.resize(0);
    _lastSyncTimeStamp = OpenRAVE::utils::GetMilliTime();
    bool bcallsetup = false;
    bool bAttachedBodiesChanged = false;
    KinBodyConstPtr ptrackingbody = _ptrackingbody.lock();
    if (!!ptrackingbody && _bTrackActiveDOF) {
        const KinBody& trackingbody = *ptrackingbody;
        EnsureVectorSize(_vecCachedBodies, trackingbody.GetEnv()->GetMaxEnvironmentBodyIndex() + 1);

        const int trackingBodyIndex = trackingbody.GetEnvironmentBodyIndex();
        KinBodyCache& trackingCache = _vecCachedBodies.at(trackingBodyIndex);
        bool isValid = trackingCache.IsValid();
        if (!isValid) {
            std::string ssinfo;
            for (int bodyIndexCached = 0; bodyIndexCached < (int)_vecCachedBodies.size(); ++bodyIndexCached) {
                const KinBodyCache& cache = _vecCachedBodies[bodyIndexCached];
                if (cache.IsValid()) {
                    ssinfo += str(boost::format("(id=%d, linkmask="));
                    for (uint64_t bitmask : cache.linkEnableStatesBitmasks) {
                        ssinfo += str(boost::format("(0x%x:") % bitmask);
                    }
                    ssinfo += str(boost::format("(, numcols=%d") % cache.vcolobjs.size());

                    KinBodyConstPtr pbody = cache.pwbody.lock(); ///< weak pointer to body
                    if (!!pbody) {
                        ssinfo += str(boost::format("(, name=%s), ") % pbody->GetName());
                    }
                }
            }
            RAVELOG_WARN_FORMAT(
                "%x tracking body not in current cached bodies (valid=%d) (tracking body %s (id=%d)) (env %d). Current cache is: %s",
                this % isValid % trackingbody.GetName() % trackingBodyIndex % trackingbody.GetEnv()->GetId() % ssinfo);
        } else {
            FCLSpace::FCLKinBodyInfoPtr pinfo = trackingCache.pwinfo.lock();
            const FCLSpace::FCLKinBodyInfoPtr& pnewinfo = _fclspace.GetInfo(trackingbody); // necessary in case pinfos were swapped!
            if (trackingCache.nActiveDOFUpdateStamp != pnewinfo->nActiveDOFUpdateStamp) {
                if (trackingbody.IsRobot()) {
                    RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(ptrackingbody);
                    if (!!probot) {
                        const RobotBase& robot = *probot;
                        if (pinfo != pnewinfo) {
                            // going to recreate everything in below step anyway, so no need to update collision objects
                            _UpdateActiveLinks(robot);
                        } else {
                            // check for any tracking link changes
                            _vTrackingActiveLinks.resize(robot.GetLinks().size(), 0);
                            // the active links might have changed
                            _linkEnableStatesBitmasks = robot.GetLinkEnableStatesMasks();
                            for (size_t ilink = 0; ilink < robot.GetLinks().size(); ++ilink) {
                                int isLinkActive = 0;
                                FOREACH(itindex, robot.GetActiveDOFIndices()) {
                                    if (robot.DoesAffect(robot.GetJointFromDOFIndex(*itindex)->GetJointIndex(), ilink)) {
                                        isLinkActive = 1;
                                        break;
                                    }
                                }
                                if (!isLinkActive) {
                                    OpenRAVE::DisableLinkStateBit(_linkEnableStatesBitmasks, ilink);
                                }
                                bool bIsActiveLinkEnabled = OpenRAVE::IsLinkStateBitEnabled(_linkEnableStatesBitmasks, ilink) && isLinkActive;

                                if (_vTrackingActiveLinks[ilink] != isLinkActive) {
                                    _vTrackingActiveLinks[ilink] = isLinkActive;
                                    CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pnewinfo, robot.GetLinks()[ilink]->GetIndex());
                                    if (bIsActiveLinkEnabled && !!pcolobj) {
                                        fcl::CollisionObject* pColObjRaw = pcolobj.get();
                                        if (!!trackingCache.vcolobjs.at(ilink)) {
                                            pmanager->replaceObject(trackingCache.vcolobjs.at(ilink).get(), pColObjRaw, false);
                                        } else {
                                            pmanager->registerObject(pColObjRaw);
                                        }
                                        bcallsetup = true;
                                    } else {
                                        if (!!trackingCache.vcolobjs.at(ilink)) {
                                            pmanager->unregisterObject(trackingCache.vcolobjs.at(ilink).get());
                                        }
                                    }
                                    trackingCache.vcolobjs.at(ilink) = pcolobj;
                                } else {
                                    if (!bIsActiveLinkEnabled && !!trackingCache.vcolobjs.at(ilink)) {
                                        // RAVELOG_VERBOSE_FORMAT("env=%d %x resetting cached colobj %s %d",
                                        // probot->GetEnv()->GetId()%this%probot->GetName()%ilink);
                                        pmanager->unregisterObject(trackingCache.vcolobjs.at(ilink).get());
                                        trackingCache.vcolobjs.at(ilink).reset();
                                    }
                                }
                            }

                            trackingCache.nActiveDOFUpdateStamp = pnewinfo->nActiveDOFUpdateStamp;
                            trackingCache.linkEnableStatesBitmasks = _linkEnableStatesBitmasks;
                        }
                    }
                }
            }
        }
    }

    FCLSpace::FCLKinBodyInfoPtr pinfo;
    KinBodyConstPtr pbody;
    for (int bodyIndexCached = 1; bodyIndexCached < (int)_vecCachedBodies.size(); ++bodyIndexCached) {
        KinBodyCache& cache = _vecCachedBodies[bodyIndexCached];
        pbody = cache.pwbody.lock();
        if (!pbody || pbody->GetEnvironmentBodyIndex() == 0) {
            // should happen when parts are removed
            // RAVELOG_VERBOSE_FORMAT("env=%d, %u manager contains invalid body %s, removing for now", _fclspace.GetEnvironmentId()%_lastSyncTimeStamp%(!pbody ?
            // std::string() : pbody->GetName()));
            FOREACH(itcolobj, cache.vcolobjs) {
                if (!!itcolobj->get()) {
                    pmanager->unregisterObject(itcolobj->get());
                }
            }
            cache.vcolobjs.resize(0);
            cache.Invalidate();
            continue;
        }

        const KinBody& body = *pbody;
        {
            const int bodyIndex = body.GetEnvironmentBodyIndex();
            if (bodyIndexCached != bodyIndex) {
                RAVELOG_WARN_FORMAT(
                    "env=%s body %s has envBodyIndex=%d, but stored at wrong index=%d",
                    body.GetEnv()->GetNameId() % body.GetName() % bodyIndex % bodyIndexCached);
            }
        }

        pinfo = cache.pwinfo.lock();
        const FCLSpace::FCLKinBodyInfoPtr& pnewinfo = _fclspace.GetInfo(body); // necessary in case pinfos were swapped!
        if (pinfo != pnewinfo) {
            // everything changed!
            RAVELOG_VERBOSE_FORMAT("%u body %s entire FCLKinBodyInfo changed", _lastSyncTimeStamp % pbody->GetName());
            FOREACH(itcolobj, cache.vcolobjs) {
                if (!!itcolobj->get()) {
                    pmanager->unregisterObject(itcolobj->get());
                }
            }
            cache.vcolobjs.resize(0);
            _AddBody(body, pnewinfo, cache.vcolobjs, cache.linkEnableStatesBitmasks, _bTrackActiveDOF && pbody == ptrackingbody);
            cache.pwinfo = pnewinfo;
            // cache.ResetStamps();
            //  need to update the stamps here so that we do not try to unregisterObject below and get into an error
            cache.nLastStamp = pnewinfo->nLastStamp;
            cache.nLinkUpdateStamp = pnewinfo->nLinkUpdateStamp;
            cache.nGeometryUpdateStamp = pnewinfo->nGeometryUpdateStamp;
            cache.nAttachedBodiesUpdateStamp = -1;
            cache.nActiveDOFUpdateStamp = pnewinfo->nActiveDOFUpdateStamp;
            cache.geometrygroup = pnewinfo->_geometrygroup;
            pinfo = pnewinfo;
            if (_tmpSortedBuffer.size() > 0) {
                pmanager->registerObjects(_tmpSortedBuffer); // bulk update
                _tmpSortedBuffer.resize(0);
            }
        }

        FCLSpace::FCLKinBodyInfo& kinBodyInfo = *pinfo;
        if (kinBodyInfo.nLinkUpdateStamp != cache.nLinkUpdateStamp) {
            // links changed
            std::vector<uint64_t>& newLinkEnableStates = _linkEnableStatesCache;
            newLinkEnableStates = body.GetLinkEnableStatesMasks();
            if (_bTrackActiveDOF && ptrackingbody == pbody) {
                for (size_t itestlink = 0; itestlink < _vTrackingActiveLinks.size(); ++itestlink) {
                    if (!_vTrackingActiveLinks[itestlink]) {
                        OpenRAVE::DisableLinkStateBit(newLinkEnableStates, itestlink);
                    }
                }
            }

            // uint64_t changed = cache.linkmask ^ newlinkmask;
            // RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), lastsync=%u body %s (%d) for cache changed link %d != %d, linkmask=0x%x",
            // body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%body.GetName()%body.GetEnvironmentBodyIndex()%kinBodyInfo.nLinkUpdateStamp%cache.nLinkUpdateStamp%_GetLinkMask(newLinkEnableStates));
            for (uint64_t ilink = 0; ilink < kinBodyInfo.vlinks.size(); ++ilink) {
                uint8_t changed =
                    OpenRAVE::IsLinkStateBitEnabled(cache.linkEnableStatesBitmasks, ilink) != OpenRAVE::IsLinkStateBitEnabled(newLinkEnableStates, ilink);
                if (changed) {
                    if (OpenRAVE::IsLinkStateBitEnabled(newLinkEnableStates, ilink)) {
                        CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(kinBodyInfo, ilink);
                        if (!!pcolobj) {
                            fcl::CollisionObject* pColObjRaw = pcolobj.get();
                            if (!!cache.vcolobjs.at(ilink)) {
                                pmanager->replaceObject(cache.vcolobjs.at(ilink).get(), pColObjRaw, false);
                            } else {
                                pmanager->registerObject(pColObjRaw);
                            }
                            bcallsetup = true;
                        } else {
                            if (!!cache.vcolobjs.at(ilink)) {
                                pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                            }
                        }
                        cache.vcolobjs.at(ilink) = pcolobj;
                    } else {
                        if (!!cache.vcolobjs.at(ilink)) {
                            pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                            cache.vcolobjs.at(ilink).reset();
                        }
                    }
                }
            }

            cache.linkEnableStatesBitmasks = newLinkEnableStates;
            cache.nLinkUpdateStamp = kinBodyInfo.nLinkUpdateStamp;
        }
        if (kinBodyInfo.nGeometryUpdateStamp != cache.nGeometryUpdateStamp) {

            if (cache.geometrygroup.size() == 0 || cache.geometrygroup != kinBodyInfo._geometrygroup) {
                // RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), lastsync=%u body %s (%d) for cache changed geometry %d != %d, linkmask=0x%x",
                // body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%body.GetName()%body.GetEnvironmentBodyIndex()%kinBodyInfo.nGeometryUpdateStamp%cache.nGeometryUpdateStamp%_GetLinkMask(cache.linkEnableStatesBitmasks));
                //  vcolobjs most likely changed
                for (uint64_t ilink = 0; ilink < kinBodyInfo.vlinks.size(); ++ilink) {
                    if (OpenRAVE::IsLinkStateBitEnabled(cache.linkEnableStatesBitmasks, ilink)) {
                        CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pinfo, ilink);
                        if (!!pcolobj) {
                            // RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s adding obj %x from link %d",
                            // body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%pColObjRaw%ilink);
                            fcl::CollisionObject* pColObjRaw = pcolobj.get();
                            if (!!cache.vcolobjs.at(ilink)) {
                                // RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s replacing cached obj %x with %x ",
                                // body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%cache.vcolobjs.at(ilink).get()%pColObjRaw);
                                pmanager->replaceObject(cache.vcolobjs.at(ilink).get(), pColObjRaw, false);
                            } else {
                                pmanager->registerObject(pColObjRaw);
                            }
                            bcallsetup = true;
                            cache.vcolobjs.at(ilink) = pcolobj;
                        } else {
                            if (!!cache.vcolobjs.at(ilink)) {
                                // RAVELOG_VERBOSE_FORMAT("env=%d, %x, body %s removing old obj from link %d",
                                // body.GetEnv()->GetId()%this%body.GetName()%ilink);
                                pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                                cache.vcolobjs.at(ilink).reset();
                            }
                        }
                    }
                }
                cache.geometrygroup = kinBodyInfo._geometrygroup;
            } else {
                // RAVELOG_VERBOSE_FORMAT("env=%d, %x, lastsync=%u body %s (%d) for cache changed geometry but no update %d != %d, geometrygroup=%s",
                // body.GetEnv()->GetId()%this%_lastSyncTimeStamp%body.GetName()%body.GetEnvironmentBodyIndex()%kinBodyInfo.nGeometryUpdateStamp%cache.nGeometryUpdateStamp%cache.geometrygroup);
            }
            cache.nGeometryUpdateStamp = kinBodyInfo.nGeometryUpdateStamp;
        }
        if (kinBodyInfo.nLastStamp != cache.nLastStamp) {
            if (IS_DEBUGLEVEL(OpenRAVE::Level_Verbose)) {
                // Transform tpose = body.GetTransform();
                // RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d) %u body %s (%for cache changed transform %d != %d, num=%d, mask=0x%x, trans=(%.3f, %.3f, %.3f)",
                // body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%body.GetName()%body.GetEnvironmentBodyIndex()%kinBodyInfo.nLastStamp%cache.nLastStamp%kinBodyInfo.vlinks.size()%_GetLinkMask(cache.linkEnableStatesBitmasks)%tpose.trans.x%tpose.trans.y%tpose.trans.z);
            }
            // transform changed
            CollisionObjectPtr pcolobj;
            for (uint64_t ilink = 0; ilink < kinBodyInfo.vlinks.size(); ++ilink) {
                if (OpenRAVE::IsLinkStateBitEnabled(cache.linkEnableStatesBitmasks, ilink)) {
                    pcolobj = _fclspace.GetLinkBV(*pinfo, ilink);
                    if (!!pcolobj) {
                        // RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s adding obj %x from link %d",
                        // body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%pColObjRaw%ilink);
                        if (cache.vcolobjs.at(ilink) == pcolobj) {
                            // same object, so just update
                            pmanager->update(cache.vcolobjs.at(ilink).get(), false);
                        } else {
                            fcl::CollisionObject* pColObjRaw = pcolobj.get();
                            // different object, have to replace
                            if (!!cache.vcolobjs.at(ilink)) {
                                // RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s replacing cached obj %x with %x ",
                                // body.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%body.GetName()%cache.vcolobjs.at(ilink).get()%pColObjRaw);
                                pmanager->replaceObject(cache.vcolobjs.at(ilink).get(), pColObjRaw, false);
                            } else {
                                pmanager->registerObject(pColObjRaw);
                            }
                        }

                        bcallsetup = true;
                        cache.vcolobjs.at(ilink) = pcolobj;
                    } else {
                        if (!!cache.vcolobjs.at(ilink)) {
                            // RAVELOG_VERBOSE_FORMAT("env=%d, %x, body %s removing old obj from link %d", body.GetEnv()->GetId()%this%body.GetName()%ilink);
                            pmanager->unregisterObject(cache.vcolobjs.at(ilink).get());
                            cache.vcolobjs.at(ilink).reset();
                        }
                    }
                }
            }

            cache.nLastStamp = kinBodyInfo.nLastStamp;
        }
        if (kinBodyInfo.nAttachedBodiesUpdateStamp != cache.nAttachedBodiesUpdateStamp) {
            // RAVELOG_VERBOSE_FORMAT("env=%d, %u the nAttachedBodiesUpdateStamp changed %d != %d, trackingbody is %s",
            // body.GetEnv()->GetId()%_lastSyncTimeStamp%kinBodyInfo.nAttachedBodiesUpdateStamp%cache.nAttachedBodiesUpdateStamp%(!!ptrackingbody ?
            // trackingbody.GetName() : std::string()));
            //  bodies changed!
            if (!!ptrackingbody) {
                bAttachedBodiesChanged = true;
            }

            cache.nAttachedBodiesUpdateStamp = kinBodyInfo.nAttachedBodiesUpdateStamp;
        }
    }

    if (bAttachedBodiesChanged && !!ptrackingbody) {
        // since tracking have to update all the bodies
        std::vector<KinBodyPtr>& vecAttachedEnvBodies = _vecAttachedEnvBodiesCache;
        std::vector<int>& vecAttachedEnvBodyIndices = _vecAttachedEnvBodyIndicesCache;
        const EnvironmentBase& env = *ptrackingbody->GetEnv();
        ptrackingbody->GetAttachedEnvironmentBodyIndices(vecAttachedEnvBodyIndices);
        env.GetBodiesFromEnvironmentBodyIndices(vecAttachedEnvBodyIndices, vecAttachedEnvBodies);

        // RAVELOG_VERBOSE_FORMAT("env=%d, %x %u setting %d attached bodies for body %s (%d)",
        // ptrackingbody->GetEnv()->GetId()%this%_lastSyncTimeStamp%attachedBodies.size()%ptrackingbody->GetName()%ptrackingbody->GetEnvironmentBodyIndex());
        //  add any new bodies
        bool vectorSizeIsEnsured = false;
        for (const KinBodyPtr& pattached : vecAttachedEnvBodies) {
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

            const FCLSpace::FCLKinBodyInfoPtr& pInfo = _fclspace.GetInfo(attached);
            if (_AddBody(attached, pInfo, cache.vcolobjs, _linkEnableStatesBitmasks, _bTrackActiveDOF && (pattached == ptrackingbody))) {
                bcallsetup = true;
            }
            // RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d) %u adding body %s (%d)",
            // cache.GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%cache.GetName()%attachedBodyIndex);
            cache.SetBodyData(pattached, pInfo, _linkEnableStatesBitmasks);
            // cache.vcolobjs.swap(vcolobjs);
            //                     else {
            //                         RAVELOG_VERBOSE_FORMAT("env=%d %x could not add attached body %s for tracking body %s",
            //                         trackingbody.GetEnv()->GetId()%this%cache.GetName()%trackingbody.GetName());
            //                     }
        }

        // remove bodies not attached anymore
        for (int cachedBodyIndex = 1; cachedBodyIndex < (int)_vecCachedBodies.size(); ++cachedBodyIndex) {
            KinBodyCache& cache = _vecCachedBodies[cachedBodyIndex];
            KinBodyConstPtr pBody = cache.pwbody.lock();
            // could be the case that the same pointer was re-added to the environment so have to check the environment id
            // make sure we don't need to make this asusmption of body id change when bodies are removed and re-added
            bool isInvalid = !pBody;
            if (!isInvalid) {
                const KinBody& body = *pBody;
                const int currentBodyEnvBodyIndex = body.GetEnvironmentBodyIndex();
                if (currentBodyEnvBodyIndex != cachedBodyIndex) {
                    RAVELOG_WARN_FORMAT(
                        "env=%s body %s has envBodyIndex=%d, but stored at wrong index=%d",
                        body.GetEnv()->GetNameId() % body.GetName() % currentBodyEnvBodyIndex % cachedBodyIndex);
                }
                const vector<int>::const_iterator it = lower_bound(vecAttachedEnvBodyIndices.begin(), vecAttachedEnvBodyIndices.end(), currentBodyEnvBodyIndex);
                isInvalid = (it == vecAttachedEnvBodyIndices.end() || *it != currentBodyEnvBodyIndex) || currentBodyEnvBodyIndex != cachedBodyIndex;
            }
            if (isInvalid) {
                if (!!pBody && IS_DEBUGLEVEL(OpenRAVE::Level_Verbose)) {
                    RAVELOG_VERBOSE_FORMAT("env=%s, %x, %u removing old cache %d", pBody->GetEnv()->GetNameId() % this % _lastSyncTimeStamp % cachedBodyIndex);
                }
                // not in attached bodies so should remove
                FOREACH(itcol, cache.vcolobjs) {
                    if (!!itcol->get()) {
                        pmanager->unregisterObject(itcol->get());
                    }
                }
                cache.vcolobjs.resize(0);
                cache.Invalidate();
            }
        }
    }

    if (_tmpSortedBuffer.size() > 0) {
        pmanager->registerObjects(_tmpSortedBuffer); // bulk update
    }
    if (bcallsetup) {
        pmanager->setup();
    }
}

bool FCLCollisionManagerInstance::_AddBody(
    const KinBody& body,
    const FCLSpace::FCLKinBodyInfoPtr& pinfo,
    std::vector<CollisionObjectPtr>& vcolobjs,
    std::vector<uint64_t>& linkEnableStatesBitmasks,
    bool bTrackActiveDOF) {
    // reset so that existing collision objects can go away
    for (CollisionObjectPtr& pcolObj : vcolobjs) {
        pcolObj.reset();
    }
    vcolobjs.resize(body.GetLinks().size(), CollisionObjectPtr());
    bool bsetUpdateStamp = false;
    linkEnableStatesBitmasks = body.GetLinkEnableStatesMasks();
    for (const KinBody::LinkPtr& plink : body.GetLinks()) {
        const KinBody::Link& link = *plink;
        const int linkIndex = link.GetIndex();

        if (bTrackActiveDOF && !_vTrackingActiveLinks.at(linkIndex)) {
            OpenRAVE::DisableLinkStateBit(linkEnableStatesBitmasks, linkIndex);
        }
        if (OpenRAVE::IsLinkStateBitEnabled(linkEnableStatesBitmasks, linkIndex)) {
            // pinfo->vlinks.at(linkIndex).listRegisteredManagers.push_back(shared_from_this());

            CollisionObjectPtr pcol = _fclspace.GetLinkBV(*pinfo, linkIndex);
            if (!!pcol) {
                CollisionGroup::const_iterator it = std::lower_bound(_tmpSortedBuffer.begin(), _tmpSortedBuffer.end(), pcol.get());
                // keep _tmpSortedBuffer sorted so that we can efficiently search
                if (it == _tmpSortedBuffer.end() || *it != pcol.get()) {
                    _tmpSortedBuffer.insert(it, pcol.get());
                } else {
                    RAVELOG_WARN_FORMAT("env=%s body %s link %s is added multiple times", body.GetEnv()->GetNameId() % body.GetName() % link.GetName());
                }
                vcolobjs[linkIndex] = pcol;
                bsetUpdateStamp = true;
            }
        }
    }
    return bsetUpdateStamp;
}

void FCLCollisionManagerInstance::_UpdateActiveLinks(const RobotBase& robot) {
    _vTrackingActiveLinks.resize(robot.GetLinks().size());
    for (size_t i = 0; i < robot.GetLinks().size(); ++i) {
        int isLinkActive = 0;
        FOREACH(itindex, robot.GetActiveDOFIndices()) {
            if (robot.DoesAffect(robot.GetJointFromDOFIndex(*itindex)->GetJointIndex(), i)) {
                isLinkActive = 1;
                break;
            }
        }
        _vTrackingActiveLinks[i] = isLinkActive;
    }
}

} // namespace ivshmem
