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
        KinBodyCache() : nLastStamp(0), nLinkUpdateStamp(0), nGeometryUpdateStamp(0), nAttachedBodiesUpdateStamp(0), linkmask(0) {
        }
        KinBodyCache(KinBodyConstPtr pbody, FCLSpace::KinBodyInfoPtr pinfo) {
            pwbody = pbody;
            pwinfo = pinfo;
            nLastStamp = pinfo->nLastStamp;
            nLinkUpdateStamp = pinfo->nLinkUpdateStamp;
            nGeometryUpdateStamp = pinfo->nGeometryUpdateStamp;
            nAttachedBodiesUpdateStamp = pinfo->nAttachedBodiesUpdateStamp;
            linkmask = pbody->GetLinkEnableStatesMask();
        }

        void ResetStamps() {
            nLastStamp = -1;
            nLinkUpdateStamp = -1;
            nGeometryUpdateStamp = -1;
            nAttachedBodiesUpdateStamp = -1;
        }

        KinBodyConstWeakPtr pwbody; ///< weak pointer to body
        FCLSpace::KinBodyInfoWeakPtr pwinfo; ///< weak pointer to info
        int nLastStamp; ///< copyied from FCLSpace::KinBodyInfo when body was last updated
        int nLinkUpdateStamp; ///< copied from FCLSpace::KinBodyInfo when body was last updated
        int nGeometryUpdateStamp; ///< copied from FCLSpace::KinBodyInfo when geometry was last updated
        int nAttachedBodiesUpdateStamp; /// copied from FCLSpace::KinBodyInfo when attached bodies was last updated
        uint64_t linkmask; ///< links that are currently inside the manager
    };

public:
    FCLCollisionManagerInstance(FCLSpace& fclspace, BroadPhaseCollisionManagerPtr pmanager) : _fclspace(fclspace), pmanager(pmanager) {
    }

    void InitBodyManager(KinBodyConstPtr pbody)
    {
        _ptrackingbody = pbody;
        std::set<KinBodyConstPtr> attachedBodies;
        pbody->GetAttached(attachedBodies);
        pmanager->clear();
        _tmpbuffer.resize(0);
        mapCachedBodies.clear();
        FOREACH(itbody, attachedBodies) {
            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(*itbody);
            bool bsetUpdateStamp = false;
            FOREACH(itlink, (*itbody)->GetLinks()) {
                if( (*itlink)->IsEnabled() ) {
                    //pinfo->vlinks.at((*itlink)->GetIndex()).listRegisteredManagers.push_back(shared_from_this());
                    _tmpbuffer.push_back(_fclspace.GetLinkBV(pinfo, (*itlink)->GetIndex()).get());
                    bsetUpdateStamp = true;
                }
            }
            if( bsetUpdateStamp ) {
                mapCachedBodies[(*itbody)->GetEnvironmentId()] = KinBodyCache(*itbody, pinfo);
            }
        }
        pmanager->registerObjects(_tmpbuffer); // bulk update
        pmanager->setup();
    }

    void InitEnvironment()
    {
        _ptrackingbody.reset();
        BOOST_ASSERT(0);
    }
    
    /// \brief ensures that pbody is being tracked inside the manager
    void EnsureBody(KinBodyConstPtr pbody)
    {
        std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(pbody->GetEnvironmentId());
        if( it == mapCachedBodies.end() ) {
            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(pbody);
            if( _AddBody(pbody, pinfo) ) {
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
        bool bcallsetup = false;
        bool bAttachedBodiesChanged = false;
        std::map<int, KinBodyCache>::iterator itcache = mapCachedBodies.begin();
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

                _AddBody(pbody, pnewinfo);
                itcache->second.pwinfo = pnewinfo;
                itcache->second.ResetStamps();
                pinfo = pnewinfo;
            }

            if( pinfo->nLinkUpdateStamp != itcache->second.nLinkUpdateStamp ) {
                // links changed
                uint64_t newlinkmask = pbody->GetLinkEnableStatesMask();
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
                if( !!_ptrackingbody ) {
                    bAttachedBodiesChanged = true;
                }

                itcache->second.nAttachedBodiesUpdateStamp = pinfo->nAttachedBodiesUpdateStamp;
            }

            ++itcache;
        }


        if( bAttachedBodiesChanged ) {
            // since tracking have to update all the bodies
            std::set<KinBodyConstPtr> attachedBodies;
            _ptrackingbody->GetAttached(attachedBodies);
            _tmpbuffer.resize(0);

            // add any new bodies
            FOREACH(itbody, attachedBodies) {
                if( mapCachedBodies.find((*itbody)->GetEnvironmentId()) == mapCachedBodies.end() ) {
                    FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(*itbody);
                    if( _AddBody(*itbody, pinfo) ) {
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
    
private:
    /// \brief adds a body to the manager, returns true if something was added
    ///
    /// should not add anything to mapCachedBodies!
    bool _AddBody(KinBodyConstPtr pbody, FCLSpace::KinBodyInfoPtr pinfo)
    {
        _tmpbuffer.resize(0);
        bool bsetUpdateStamp = false;
        FOREACH(itlink, (pbody)->GetLinks()) {
            if( (*itlink)->IsEnabled() ) {
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


    FCLSpace& _fclspace; ///< reference for speed
    BroadPhaseCollisionManagerPtr pmanager;
    std::map<int, KinBodyCache> mapCachedBodies; ///< pair of (body id, (weak body, updatestamp)) where the key is KinBody::GetEnvironmentId, weak pody is KinBodyWeakPtr necessary since sometimes access to the body is required, and updatestamp is KinBody::GetUpdateStamp()

    KinBodyConstPtr _ptrackingbody; ///< if set, then only tracking the attached bodies if this body
    CollisionGroup _tmpbuffer; ///< cache
};

typedef boost::shared_ptr<FCLCollisionManagerInstance> FCLCollisionManagerInstancePtr;
typedef boost::weak_ptr<FCLCollisionManagerInstance> FCLCollisionManagerInstanceWeakPtr;

} // end namespace fclrave

#endif
