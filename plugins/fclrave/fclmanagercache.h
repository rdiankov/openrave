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
            nAttachedBodiesUpdateStamp = pinfo->nAttachedBodiesUpdateStamp;
            nActiveDOFUpdateStamp = pinfo->nActiveDOFUpdateStamp;
            linkmask = pbody->GetLinkEnableStatesMask();
        }

        void ResetStamps() {
            nLastStamp = -1;
            nLinkUpdateStamp = -1;
            nGeometryUpdateStamp = -1;
            nAttachedBodiesUpdateStamp = -1;
            nActiveDOFUpdateStamp = -1;
        }

        KinBodyConstWeakPtr pwbody; ///< weak pointer to body
        FCLSpace::KinBodyInfoWeakPtr pwinfo; ///< weak pointer to info
        int nLastStamp; ///< copyied from FCLSpace::KinBodyInfo when body was last updated
        int nLinkUpdateStamp; ///< copied from FCLSpace::KinBodyInfo when body was last updated
        int nGeometryUpdateStamp; ///< copied from FCLSpace::KinBodyInfo when geometry was last updated
        int nAttachedBodiesUpdateStamp; /// copied from FCLSpace::KinBodyInfo when attached bodies was last updated
        int nActiveDOFUpdateStamp; ///< update stamp when the active dof changed
        uint64_t linkmask; ///< links that are currently inside the manager
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
        _bTrackActiveDOF = bTrackActiveDOF;
        if( bTrackActiveDOF && pbody->IsRobot() ) {
            RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(pbody);
            if( !!probot ) {
                _UpdateActiveLinks(probot);
            }
        }
        pmanager->clear();
        _tmpbuffer.resize(0);
        mapCachedBodies.clear();
        FOREACH(itbody, attachedBodies) {
            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(*itbody);
            bool bsetUpdateStamp = false;
            uint64_t linkmask = 0;
            FOREACH(itlink, (*itbody)->GetLinks()) {
                if( (*itlink)->IsEnabled() && (*itbody != pbody || !_bTrackActiveDOF || _vTrackingActiveLinks.at((*itlink)->GetIndex())) ) {
                    //pinfo->vlinks.at((*itlink)->GetIndex()).listRegisteredManagers.push_back(shared_from_this());
                    _tmpbuffer.push_back(_fclspace.GetLinkBV(pinfo, (*itlink)->GetIndex()).get());
                    bsetUpdateStamp = true;
                    linkmask |= (1 << (uint64_t)(*itlink)->GetIndex());
                }
            }
            if( bsetUpdateStamp ) {
                mapCachedBodies[(*itbody)->GetEnvironmentId()] = KinBodyCache(*itbody, pinfo);
                mapCachedBodies[(*itbody)->GetEnvironmentId()].linkmask = linkmask;
            }
        }
        pmanager->registerObjects(_tmpbuffer); // bulk update
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
        FOREACH(itbody, vbodies) {
            int bodyid = (*itbody)->GetEnvironmentId();
            if( _setExcludeBodyIds.count(bodyid) == 0 ) {
                std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(bodyid);
                if( it == mapCachedBodies.end() ) {
                    FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(*itbody);
                    if( _AddBody(*itbody, pinfo, false) ) {
                        mapCachedBodies[bodyid] = KinBodyCache(*itbody, pinfo);
                    }
                }
            }
        }
    }

    /// \brief ensures that pbody is being tracked inside the manager
    void EnsureBody(KinBodyConstPtr pbody)
    {
        std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(pbody->GetEnvironmentId());
        if( it == mapCachedBodies.end() ) {
            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(pbody);
            if( _AddBody(pbody, pinfo, false) ) {
                mapCachedBodies[(pbody)->GetEnvironmentId()] = KinBodyCache(pbody, pinfo);
            }

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
        _lastSyncTimeStamp = OpenRAVE::utils::GetMilliTime();
        bool bcallsetup = false;
        bool bAttachedBodiesChanged = false;
        std::map<int, KinBodyCache>::iterator itcache = mapCachedBodies.begin();
        KinBodyConstPtr ptrackingbody = _ptrackingbody.lock();
        if( !!ptrackingbody && _bTrackActiveDOF ) {
            std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(ptrackingbody->GetEnvironmentId());
            if( it == mapCachedBodies.end() ) {
                RAVELOG_WARN("tracking body not in current cached bodies\n");
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
                // should not happen
                RAVELOG_WARN("manager contains invalid body, removing for now\n");
                _RemoveBody(pbody);
                mapCachedBodies.erase(itcache++);
                continue;
            }

            FCLSpace::KinBodyInfoPtr pnewinfo = _fclspace.GetInfo(pbody); // necessary in case pinfos were swapped!
            if( pinfo != pnewinfo ) {
                // everything changed!
                for(size_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                    bool bIsRegistered = !!(itcache->second.linkmask & (1<<(uint64_t)ilink));
                    if( bIsRegistered ) {
                        pmanager->unregisterObject(_fclspace.GetLinkBV(pinfo, ilink).get());
                    }
                }

                _AddBody(pbody, pnewinfo, _bTrackActiveDOF&&pbody==ptrackingbody);
                itcache->second.pwinfo = pnewinfo;
                itcache->second.ResetStamps();
                pinfo = pnewinfo;
            }

            if( pinfo->nLinkUpdateStamp != itcache->second.nLinkUpdateStamp ) {
                // links changed
                uint64_t newlinkmask = pbody->GetLinkEnableStatesMask();
                if( _bTrackActiveDOF && ptrackingbody == pbody ) {
                    for(size_t itestlink = 0; itestlink < _vTrackingActiveLinks.size(); ++itestlink) {
                        if( !_vTrackingActiveLinks[itestlink] ) {
                            newlinkmask &= ~(1 << (uint64_t)itestlink);
                        }
                    }
                }

                uint64_t changed = itcache->second.linkmask ^ newlinkmask;
                if( changed ) {
                    for(uint64_t ilink = 0; ilink < 64; ++ilink) {
                        if( changed & (1<<ilink) ) {
                            if( newlinkmask & (1<<ilink) ) {
                                pmanager->registerObject(_fclspace.GetLinkBV(pinfo, ilink).get());
                            }
                            else {
                                pmanager->unregisterObject(_fclspace.GetLinkBV(pinfo, ilink).get());
                            }
                        }
                    }
                }

                itcache->second.linkmask = newlinkmask;
                itcache->second.nLinkUpdateStamp = pinfo->nLinkUpdateStamp;
            }
            if( pinfo->nLastStamp != itcache->second.nLastStamp || pinfo->nGeometryUpdateStamp != itcache->second.nGeometryUpdateStamp ) {
                // transform changed
                for(uint64_t ilink = 0; ilink < 64; ++ilink) {
                    if( itcache->second.linkmask & (1<<ilink) ) {
                        pmanager->update(_fclspace.GetLinkBV(pinfo, ilink).get(), false);
                        bcallsetup = true;
                    }
                }

                itcache->second.nLastStamp = pinfo->nLastStamp;
                itcache->second.nGeometryUpdateStamp = pinfo->nGeometryUpdateStamp;
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
            _tmpbuffer.resize(0);

            // add any new bodies
            FOREACH(itbody, attachedBodies) {
                if( mapCachedBodies.find((*itbody)->GetEnvironmentId()) == mapCachedBodies.end() ) {
                    FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(*itbody);
                    if( _AddBody(*itbody, pinfo, _bTrackActiveDOF&&(*itbody == ptrackingbody)) ) {
                        mapCachedBodies[(*itbody)->GetEnvironmentId()] = KinBodyCache(*itbody, pinfo);
                        bcallsetup = true;
                    }
                }
            }

            // remove bodies not attached anymore
            itcache = mapCachedBodies.begin();
            while(itcache != mapCachedBodies.end()) {
                KinBodyConstPtr pbody = itcache->second.pwbody.lock();
                if( !pbody ) {
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

    void PrintStatus(int debuglevel)
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
    /// should not add anything to mapCachedBodies!
    bool _AddBody(KinBodyConstPtr pbody, FCLSpace::KinBodyInfoPtr pinfo, bool bTrackActiveDOF)
    {
        _tmpbuffer.resize(0);
        bool bsetUpdateStamp = false;
        FOREACH(itlink, (pbody)->GetLinks()) {
            if( (*itlink)->IsEnabled() && (!bTrackActiveDOF || _vTrackingActiveLinks.at((*itlink)->GetIndex())) ) {
                //pinfo->vlinks.at((*itlink)->GetIndex()).listRegisteredManagers.push_back(shared_from_this());
                _tmpbuffer.push_back(_fclspace.GetLinkBV(pinfo, (*itlink)->GetIndex()).get());
                bsetUpdateStamp = true;
            }
        }
        if( bsetUpdateStamp ) {
            pmanager->registerObjects(_tmpbuffer); // bulk update
        }
        return bsetUpdateStamp;
    }

    /// \brief return true if body was removed
    bool _RemoveBody(KinBodyConstPtr pbody)
    {
        std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(pbody->GetEnvironmentId());
        if( it != mapCachedBodies.end() ) {
            // have to unregister
            //KinBodyConstPtr pbody = it->second.pwbody.lock();
            FCLSpace::KinBodyInfoPtr pinfo = it->second.pwinfo.lock();
            if( !!pinfo ) {
                for(size_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                    bool bIsRegistered = !!(it->second.linkmask & (1<<(uint64_t)ilink));
                    if( bIsRegistered ) {
                        pmanager->unregisterObject(_fclspace.GetLinkBV(pinfo, ilink).get());
                    }
                }
            }
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
