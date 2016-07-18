// -*- coding: utf-8 -*-
#ifndef OPENRAVE_FCL_MANAGERCACHE
#define OPENRAVE_FCL_MANAGERCACHE

#include "plugindefs.h"
#include "fclspace.h"

namespace fclrave {

typedef boost::shared_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerPtr;
typedef boost::weak_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerWeakPtr;

struct EnvironmentState
{
    std::vector< std::pair<int, uint64_t> > bodyids; ///< the bodies with the enabled/disabled links inside
};

/// \brief A broadphase collision manager together with cache data of the bodies it contains
///
/// used to maintain the correct state of the broadphase manager
class FCLCollisionManagerInstance : public boost::enable_shared_from_this<FCLCollisionManagerInstance>
{
    ///< cache data of body that is managed
    struct KinBodyCache
    {
        KinBodyCache() : nLastStamp(0), nLinkUpdateStamp(0), nGeometryUpdateStamp(0), nAttachedBodiesUpdateStamp(0), nActiveDOFUpdateStamp(0), linkmask(0) {
        }
        KinBodyCache(KinBodyConstPtr pbody, FCLSpace::KinBodyInfoPtr pinfo) {
            pwbody = pbody;
            pwinfo = pinfo;
            nLastStamp = pinfo->nLastStamp;
            nLinkUpdateStamp = pinfo->nLinkUpdateStamp;
            nGeometryUpdateStamp = pinfo->nGeometryUpdateStamp;
            geometrygroup = pinfo->_geometrygroup;
            nAttachedBodiesUpdateStamp = pinfo->nAttachedBodiesUpdateStamp;
            nActiveDOFUpdateStamp = pinfo->nActiveDOFUpdateStamp;
            linkmask = pbody->GetLinkEnableStatesMask();
        }

        KinBodyConstWeakPtr pwbody; ///< weak pointer to body
        FCLSpace::KinBodyInfoWeakPtr pwinfo; ///< weak pointer to info
        int nLastStamp; ///< copyied from FCLSpace::KinBodyInfo when body was last updated
        int nLinkUpdateStamp; ///< copied from FCLSpace::KinBodyInfo when body was last updated
        int nGeometryUpdateStamp; ///< copied from FCLSpace::KinBodyInfo when geometry was last updated
        int nAttachedBodiesUpdateStamp; /// copied from FCLSpace::KinBodyInfo when attached bodies was last updated
        int nActiveDOFUpdateStamp; ///< update stamp when the active dof changed
        uint64_t linkmask; ///< links that are currently inside the manager
        std::vector<CollisionObjectPtr> vcolobjs; ///< collision objects used for each link (use link index). have to hold pointers so that KinBodyInfo does not remove them!
        std::string geometrygroup; ///< cached geometry group
    };

public:
    FCLCollisionManagerInstance(FCLSpace& fclspace, BroadPhaseCollisionManagerPtr pmanager) : _fclspace(fclspace), pmanager(pmanager) {
        _lastSyncTimeStamp = OpenRAVE::utils::GetMilliTime();
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
        pmanager->clear();
        _tmpbuffer.resize(0);
        mapCachedBodies.clear();
        FOREACH(itbody, attachedBodies) {
            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(*itbody);
            bool bsetUpdateStamp = false;
            uint64_t linkmask = 0;
            std::vector<CollisionObjectPtr> vcolobjs((*itbody)->GetLinks().size());
            FOREACH(itlink, (*itbody)->GetLinks()) {
                if( (*itlink)->IsEnabled() && (*itbody != pbody || !_bTrackActiveDOF || _vTrackingActiveLinks.at((*itlink)->GetIndex())) ) {
                    CollisionObjectPtr pcol = _fclspace.GetLinkBV(pinfo, (*itlink)->GetIndex());
                    vcolobjs[(*itlink)->GetIndex()] = pcol;
                    if( !!pcol ) {
                        _tmpbuffer.push_back(pcol.get());
                    }
                    bsetUpdateStamp = true;
                    linkmask |= ((uint64_t)1 << (uint64_t)(*itlink)->GetIndex());
                }
            }
            if( bsetUpdateStamp ) {
                mapCachedBodies[(*itbody)->GetEnvironmentId()] = KinBodyCache(*itbody, pinfo);
                mapCachedBodies[(*itbody)->GetEnvironmentId()].linkmask = linkmask;
                mapCachedBodies[(*itbody)->GetEnvironmentId()].vcolobjs.swap(vcolobjs);
            }
        }
        if( _tmpbuffer.size() > 0 ) {
            pmanager->registerObjects(_tmpbuffer); // bulk update
        }
        pmanager->setup();
    }

    /// \brief sets up manager for environment checking
    void InitEnvironment(const std::set<KinBodyConstPtr>& excludedbodies)
    {
        _ptrackingbody.reset();
        _setExcludeBodyIds.clear();
        pmanager->clear();
        mapCachedBodies.clear();
        FOREACH(itbody, excludedbodies) {
            _setExcludeBodyIds.insert((*itbody)->GetEnvironmentId());
        }
        pmanager->setup();
    }

    /// \brief makes sure that all the bodies are currently in the scene (if they are not explicitly excluded)
    void EnsureBodies(const std::set<KinBodyConstPtr>& vbodies)
    {
        _tmpbuffer.resize(0);
        std::vector<CollisionObjectPtr> vcolobjs;
        FOREACH(itbody, vbodies) {
            int bodyid = (*itbody)->GetEnvironmentId();
            if( _setExcludeBodyIds.count(bodyid) == 0 ) {
                std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(bodyid);
                if( it == mapCachedBodies.end() ) {
                    FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(*itbody);
                    uint64_t linkmask=0;
                    if( _AddBody(*itbody, pinfo, vcolobjs, linkmask, false) ) {
                        mapCachedBodies[bodyid] = KinBodyCache(*itbody, pinfo);
                        mapCachedBodies[bodyid].vcolobjs.swap(vcolobjs);
                        mapCachedBodies[bodyid].linkmask = linkmask;
                    }
                }
            }
        }
        if( _tmpbuffer.size() > 0 ) {
            pmanager->registerObjects(_tmpbuffer); // bulk update
        }
    }

    /// \brief ensures that pbody is being tracked inside the manager
    void EnsureBody(KinBodyConstPtr pbody)
    {
        _tmpbuffer.resize(0);
        std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(pbody->GetEnvironmentId());
        if( it == mapCachedBodies.end() ) {
            std::vector<CollisionObjectPtr> vcolobjs;
            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(pbody);
            uint64_t linkmask=0;
            if( _AddBody(pbody, pinfo, vcolobjs, linkmask, false) ) {
                mapCachedBodies[(pbody)->GetEnvironmentId()] = KinBodyCache(pbody, pinfo);
                mapCachedBodies[(pbody)->GetEnvironmentId()].vcolobjs.swap(vcolobjs);
                mapCachedBodies[(pbody)->GetEnvironmentId()].linkmask = linkmask;
            }
        }
        if( _tmpbuffer.size() > 0 ) {
            pmanager->registerObjects(_tmpbuffer); // bulk update
        }

    }

    /// \brief removes tracking of the body
    void RemoveBody(KinBodyConstPtr pbody)
    {
        if( _RemoveBody(pbody) ) {
            mapCachedBodies.erase(pbody->GetEnvironmentId());
        }
    }

    /// \brief Synchronizes the element of the manager instance whose update stamps are outdated
    void Synchronize()
    {
        _tmpbuffer.resize(0);
        _lastSyncTimeStamp = OpenRAVE::utils::GetMilliTime();
        bool bcallsetup = false;
        bool bAttachedBodiesChanged = false;
        std::map<int, KinBodyCache>::iterator itcache = mapCachedBodies.begin();
        KinBodyConstPtr ptrackingbody = _ptrackingbody.lock();
        if( !!ptrackingbody && _bTrackActiveDOF ) {
            std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(ptrackingbody->GetEnvironmentId());
            if( it == mapCachedBodies.end() ) {
                RAVELOG_WARN_FORMAT("%u tracking body not in current cached bodies", _lastSyncTimeStamp);
            }
            else {
                FCLSpace::KinBodyInfoPtr pnewinfo = _fclspace.GetInfo(ptrackingbody); // necessary in case pinfos were swapped!
                if( it->second.nActiveDOFUpdateStamp != pnewinfo->nActiveDOFUpdateStamp ) {
                    if( ptrackingbody->IsRobot() ) {
                        RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(ptrackingbody);
                        if( !!probot ) {
                            _UpdateActiveLinks(probot);
                        }
                    }
                }
            }
        }

        while(itcache != mapCachedBodies.end()) {
            KinBodyConstPtr pbody = itcache->second.pwbody.lock();
            FCLSpace::KinBodyInfoPtr pinfo = itcache->second.pwinfo.lock();

            if( !pbody || pbody->GetEnvironmentId() == 0 ) {
                // should happen when parts are removed
                RAVELOG_VERBOSE_FORMAT("%u manager contains invalid body %s, removing for now", _lastSyncTimeStamp%(!pbody ? std::string() : pbody->GetName()));
                FOREACH(itcolobj, itcache->second.vcolobjs) {
                    if( !!itcolobj->get() ) {
                        pmanager->unregisterObject(itcolobj->get());
                    }
                }
                itcache->second.vcolobjs.resize(0);
                mapCachedBodies.erase(itcache++);
                continue;
            }

            FCLSpace::KinBodyInfoPtr pnewinfo = _fclspace.GetInfo(pbody); // necessary in case pinfos were swapped!
            if( pinfo != pnewinfo ) {
                // everything changed!
                RAVELOG_VERBOSE_FORMAT("%u body %s entire KinBodyInfo changed", _lastSyncTimeStamp%pbody->GetName());
                FOREACH(itcolobj, itcache->second.vcolobjs) {
                    if( !!itcolobj->get() ) {
                        pmanager->unregisterObject(itcolobj->get());
                    }
                }
                itcache->second.vcolobjs.resize(0);
                _AddBody(pbody, pnewinfo, itcache->second.vcolobjs, itcache->second.linkmask, _bTrackActiveDOF&&pbody==ptrackingbody);
                itcache->second.pwinfo = pnewinfo;
                //itcache->second.ResetStamps();
                // need to update the stamps here so that we do not try to unregisterObject below and get into an error
                itcache->second.nLastStamp = -1;
                itcache->second.nLinkUpdateStamp = -1;
                itcache->second.nGeometryUpdateStamp = -1;
                itcache->second.nAttachedBodiesUpdateStamp = -1;
                itcache->second.nActiveDOFUpdateStamp = -1;
                itcache->second.geometrygroup.resize(0);

                itcache->second.nLastStamp = pnewinfo->nLastStamp;
                itcache->second.nLinkUpdateStamp = pnewinfo->nLinkUpdateStamp;
                itcache->second.nGeometryUpdateStamp = pnewinfo->nGeometryUpdateStamp;
                itcache->second.nAttachedBodiesUpdateStamp = -1;
                itcache->second.nActiveDOFUpdateStamp = pnewinfo->nActiveDOFUpdateStamp;
                itcache->second.geometrygroup = pnewinfo->_geometrygroup;
                pinfo = pnewinfo;
                if( _tmpbuffer.size() > 0 ) {
                    pmanager->registerObjects(_tmpbuffer); // bulk update
                    _tmpbuffer.resize(0);
                }
            }

            if( pinfo->nLinkUpdateStamp != itcache->second.nLinkUpdateStamp ) {
                RAVELOG_VERBOSE_FORMAT("%u body %s for cache changed link %d != %d", _lastSyncTimeStamp%pbody->GetName()%pinfo->nLinkUpdateStamp%itcache->second.nLinkUpdateStamp);
                // links changed
                uint64_t newlinkmask = pbody->GetLinkEnableStatesMask();
                if( _bTrackActiveDOF && ptrackingbody == pbody ) {
                    for(size_t itestlink = 0; itestlink < _vTrackingActiveLinks.size(); ++itestlink) {
                        if( !_vTrackingActiveLinks[itestlink] ) {
                            newlinkmask &= ~((uint64_t)1 << (uint64_t)itestlink);
                        }
                    }
                }

                uint64_t changed = itcache->second.linkmask ^ newlinkmask;
                if( changed ) {
                    for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                        if( changed & ((uint64_t)1<<ilink) ) {
                            if( newlinkmask & ((uint64_t)1<<ilink) ) {
                                CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(pinfo, ilink);
                                if( !!pcolobj ) {
                                    pmanager->registerObject(pcolobj.get());
                                }
                                itcache->second.vcolobjs.at(ilink) = pcolobj;
                            }
                            else {
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                                    itcache->second.vcolobjs.at(ilink).reset();
                                }
                            }
                        }
                    }
                }

                itcache->second.linkmask = newlinkmask;
                itcache->second.nLinkUpdateStamp = pinfo->nLinkUpdateStamp;
            }
            if( pinfo->nGeometryUpdateStamp != itcache->second.nGeometryUpdateStamp ) {

                if( itcache->second.geometrygroup.size() == 0 || itcache->second.geometrygroup != pinfo->_geometrygroup ) {
                    RAVELOG_VERBOSE_FORMAT("%u body %s for cache changed geometry %d != %d", _lastSyncTimeStamp%pbody->GetName()%pinfo->nGeometryUpdateStamp%itcache->second.nGeometryUpdateStamp);
                    // vcolobjs most likely changed
                    for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                        if( !!itcache->second.vcolobjs.at(ilink) ) {
                            CollisionObjectPtr pcol = _fclspace.GetLinkBV(pinfo, ilink);
                            if( !!pcol ) {
#ifdef FCLRAVE_USE_REPLACEOBJECT
                                pmanager->replaceObject(itcache->second.vcolobjs.at(ilink).get(), pcol.get(), false);
#else
                                pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                                pmanager->registerObject(pcol.get());
#endif
                                bcallsetup = true;
                            } else {
                                pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                            }
                            itcache->second.vcolobjs.at(ilink) = pcol;
                        }
                    }
                    itcache->second.geometrygroup = pinfo->_geometrygroup;
                }
                itcache->second.nGeometryUpdateStamp = pinfo->nGeometryUpdateStamp;
            }
            if( pinfo->nLastStamp != itcache->second.nLastStamp ) {
                //RAVELOG_VERBOSE_FORMAT("%u body %s for cache changed transform %d != %d", _lastSyncTimeStamp%pbody->GetName()%pinfo->nLastStamp%itcache->second.nLastStamp);
                // transform changed
                for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                    if( itcache->second.linkmask & ((uint64_t)1<<ilink) ) {
                        if( !!itcache->second.vcolobjs.at(ilink) ) {
#ifdef FCLRAVE_USE_BULK_UPDATE
                            pmanager->update(itcache->second.vcolobjs.at(ilink).get(), false);
#else
                            // Performance issue !!
                            pmanager->update(itcache->second.vcolobjs.at(ilink).get());
#endif
                            bcallsetup = true;
                        }
                    }
                }

                itcache->second.nLastStamp = pinfo->nLastStamp;
            }
            if( pinfo->nAttachedBodiesUpdateStamp != itcache->second.nAttachedBodiesUpdateStamp ) {
                // bodies changed!
                if( !!ptrackingbody ) {
                    bAttachedBodiesChanged = true;
                }

                itcache->second.nAttachedBodiesUpdateStamp = pinfo->nAttachedBodiesUpdateStamp;
            }

            ++itcache;
        }

        if( bAttachedBodiesChanged && !!ptrackingbody ) {
            // since tracking have to update all the bodies
            std::set<KinBodyConstPtr> attachedBodies;
            ptrackingbody->GetAttached(attachedBodies);

            // add any new bodies
            FOREACH(itbody, attachedBodies) {
                if( mapCachedBodies.find((*itbody)->GetEnvironmentId()) == mapCachedBodies.end() ) {
                    FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(*itbody);
                    std::vector<CollisionObjectPtr> vcolobjs;
                    uint64_t linkmask = 0;
                    if( _AddBody(*itbody, pinfo, vcolobjs, linkmask, _bTrackActiveDOF&&(*itbody == ptrackingbody)) ) {
                        mapCachedBodies[(*itbody)->GetEnvironmentId()] = KinBodyCache(*itbody, pinfo);
                        mapCachedBodies[(*itbody)->GetEnvironmentId()].vcolobjs.swap(vcolobjs);
                        mapCachedBodies[(*itbody)->GetEnvironmentId()].linkmask = linkmask;
                        bcallsetup = true;
                    }
                }
            }

            // remove bodies not attached anymore
            itcache = mapCachedBodies.begin();
            while(itcache != mapCachedBodies.end()) {
                KinBodyConstPtr pbody = itcache->second.pwbody.lock();
                if( !pbody ) {
                    FOREACH(itcol, itcache->second.vcolobjs) {
                        if( !!itcol->get() ) {
                            pmanager->unregisterObject(itcol->get());
                        }
                    }
                    itcache->second.vcolobjs.resize(0);
                    itcache = mapCachedBodies.erase(itcache++);
                }
                else if( attachedBodies.count(pbody) == 0 ) {
                    // not in attached bodies so should remove
                    _RemoveBody(pbody);
                    itcache = mapCachedBodies.erase(itcache++);
                }
                else {
                    ++itcache;
                }
            }
        }
        if( _tmpbuffer.size() > 0 ) {
            pmanager->registerObjects(_tmpbuffer); // bulk update
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

    inline const std::set<int>& GetExcludeBodyIds() const {
        return _setExcludeBodyIds;
    }

    void PrintStatus(uint32_t debuglevel)
    {
        if( IS_DEBUGLEVEL(debuglevel) ) {
            std::stringstream ss;
            ss << "bodies=[";
            FOREACH(it, mapCachedBodies) {
                KinBodyConstPtr pbody = it->second.pwbody.lock();
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
    /// should not add anything to mapCachedBodies! append to _tmpbuffer
    bool _AddBody(KinBodyConstPtr pbody, FCLSpace::KinBodyInfoPtr pinfo, std::vector<CollisionObjectPtr>& vcolobjs, uint64_t& linkmask, bool bTrackActiveDOF)
    {
        vcolobjs.resize(0);
        vcolobjs.resize(pbody->GetLinks().size());
        bool bsetUpdateStamp = false;
        linkmask = 0;
        FOREACH(itlink, (pbody)->GetLinks()) {
            if( (*itlink)->IsEnabled() && (!bTrackActiveDOF || _vTrackingActiveLinks.at((*itlink)->GetIndex())) ) {
                //pinfo->vlinks.at((*itlink)->GetIndex()).listRegisteredManagers.push_back(shared_from_this());
                CollisionObjectPtr pcol = _fclspace.GetLinkBV(pinfo, (*itlink)->GetIndex());
                if( !!pcol ) {
                    _tmpbuffer.push_back(pcol.get());
                    vcolobjs[(*itlink)->GetIndex()] = pcol;
                    linkmask |= ((uint64_t)1 << (uint64_t)(*itlink)->GetIndex());
                    bsetUpdateStamp = true;
                }
            }
        }
        return bsetUpdateStamp;
    }

    /// \brief return true if body was removed
    bool _RemoveBody(KinBodyConstPtr pbody)
    {
        std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(pbody->GetEnvironmentId());
        if( it != mapCachedBodies.end() ) {
            FOREACH(itcol, it->second.vcolobjs) {
                if( !!itcol->get() ) {
                    pmanager->unregisterObject(itcol->get());
                }
            }
            it->second.vcolobjs.resize(0);
            return true;
        }

        return false;
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

    FCLSpace& _fclspace; ///< reference for speed
    BroadPhaseCollisionManagerPtr pmanager;
    std::map<int, KinBodyCache> mapCachedBodies; ///< pair of (body id, (weak body, updatestamp)) where the key is KinBody::GetEnvironmentId
    uint32_t _lastSyncTimeStamp; ///< timestamp when last synchronized

    std::set<int> _setExcludeBodyIds; ///< any bodies that should not be considered inside the manager, used with environment mode
    CollisionGroup _tmpbuffer; ///< cache

    KinBodyConstWeakPtr _ptrackingbody; ///< if set, then only tracking the attached bodies if this body
    std::vector<int> _vTrackingActiveLinks; ///< indices of which links are active for tracking body

    bool _bTrackActiveDOF; ///< if true and _ptrackingbody is valid, then should be tracking the active dof of the _ptrackingbody

};

typedef boost::shared_ptr<FCLCollisionManagerInstance> FCLCollisionManagerInstancePtr;
typedef boost::weak_ptr<FCLCollisionManagerInstance> FCLCollisionManagerInstanceWeakPtr;

} // end namespace fclrave

#endif
