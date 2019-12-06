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
        KinBodyCache() : nLastStamp(0), nLinkUpdateStamp(0), nGeometryUpdateStamp(0), nAttachedBodiesUpdateStamp(0), nActiveDOFUpdateStamp(0) {
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
            pbody->GetLinkEnableStates(linkEnableStates);
        }
        ~KinBodyCache() {
            if( vcolobjs.size() > 0 ) { // should never happen
                KinBodyConstPtr pbody = pwbody.lock();
                std::string name;
                if( !!pbody ) {
                    name = pbody->GetName();
                }
                RAVELOG_WARN_FORMAT("there are %d fcl collision objects left for body %s", vcolobjs.size()%name);
            }
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
        if( _tmpbuffer.size() > 0 ) {
            RAVELOG_WARN_FORMAT("_tmpbuffer has left over objects %d", _tmpbuffer.size());
        }
        _tmpbuffer.resize(0);

        pmanager->clear();
        // should clear all vcolobjs notifying the destructor that manager has the objects unregistered
        FOREACH(it, mapCachedBodies) {
            it->second.vcolobjs.clear();
        }
        mapCachedBodies.clear();
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
        _tmpbuffer.resize(0);
        // should clear all vcolobjs notifying the destructor that manager has the objects unregistered
        FOREACH(it, mapCachedBodies) {
            it->second.vcolobjs.resize(0);
        }
        std::vector<CollisionObjectPtr> vcolobjs;
        mapCachedBodies.clear();
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
                        if( find(_tmpbuffer.begin(), _tmpbuffer.end(), pcol.get()) == _tmpbuffer.end() ) {
                            _tmpbuffer.push_back(pcol.get());
                        }
                        else {
                            RAVELOG_WARN_FORMAT("env=%d body %s link %s is added multiple times", pbody->GetEnv()->GetId()%pbody->GetName()%(*itlink)->GetName());
                        }
                    }
                    bsetUpdateStamp = true;
                    _linkEnableStates.at((*itlink)->GetIndex()) = 1;
                }
            }

            // regardless if the linkmask, have to always add to cache in order to track!
            if( 1 ) {//bsetUpdateStamp ) {
                //RAVELOG_VERBOSE_FORMAT("env=%d, %x adding body %s (%d) linkmask=0x%x, _tmpbuffer.size()=%d", (*itbody)->GetEnv()->GetId()%this%(*itbody)->GetName()%pbody->GetEnvironmentId()%_GetLinkMask(_linkEnableStates)%_tmpbuffer.size());
                mapCachedBodies[(*itbody)->GetEnvironmentId()] = KinBodyCache(*itbody, pinfo);
                mapCachedBodies[(*itbody)->GetEnvironmentId()].linkEnableStates = _linkEnableStates;
                mapCachedBodies[(*itbody)->GetEnvironmentId()].vcolobjs.swap(vcolobjs);
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
        if( _tmpbuffer.size() > 0 ) {
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
            SaveCollisionObjectDebugInfos();
#endif
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
                    FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(**itbody);
                    _linkEnableStates.resize((*itbody)->GetLinks().size()); ///< links that are currently inside the manager
                    std::fill(_linkEnableStates.begin(), _linkEnableStates.end(), 0);
                    if( _AddBody(*itbody, pinfo, vcolobjs, _linkEnableStates, false) ) { // new collision objects are already added to _tmpbuffer
                        mapCachedBodies[bodyid] = KinBodyCache(*itbody, pinfo);
                        mapCachedBodies[bodyid].vcolobjs.swap(vcolobjs);
                        mapCachedBodies[bodyid].linkEnableStates = _linkEnableStates;
                    }
                }
            }
        }
        if( _tmpbuffer.size() > 0 ) {
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
            SaveCollisionObjectDebugInfos();
#endif
            pmanager->registerObjects(_tmpbuffer); // bulk update
        }
    }

    /// \brief ensures that pbody is being tracked inside the manager
//    void EnsureBody(KinBodyConstPtr pbody)
//    {
//        _tmpbuffer.resize(0);
//        std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(pbody->GetEnvironmentId());
//        if( it == mapCachedBodies.end() ) {
//            std::vector<CollisionObjectPtr> vcolobjs;
//            FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(pbody);
//            uint64_t linkmask=0;
//            if( _AddBody(pbody, pinfo, vcolobjs, linkmask, false) ) { // new collision objects are already added to _tmpbuffer
//                mapCachedBodies[(pbody)->GetEnvironmentId()] = KinBodyCache(pbody, pinfo);
//                mapCachedBodies[(pbody)->GetEnvironmentId()].vcolobjs.swap(vcolobjs);
//                mapCachedBodies[(pbody)->GetEnvironmentId()].linkmask = linkmask;
//            }
//        }
//        if( _tmpbuffer.size() > 0 ) {
//#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
//            SaveCollisionObjectDebugInfos();
//#endif
//            pmanager->registerObjects(_tmpbuffer); // bulk update
//        }
//
//    }

    /// \brief remove tracking of the body, return true if body was removed
    bool RemoveBody(const KinBody &body)
    {
        RAVELOG_VERBOSE_FORMAT("%u removing body %s", _lastSyncTimeStamp% body.GetName());

        std::map<int, KinBodyCache>::iterator it = mapCachedBodies.find(body.GetEnvironmentId());
        if( it != mapCachedBodies.end() ) {
            FOREACH(itcol, it->second.vcolobjs) {
                if( !!itcol->get() ) {
                    pmanager->unregisterObject(itcol->get());
                }
            }
            it->second.vcolobjs.resize(0);
            mapCachedBodies.erase(it);
            return true;
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
        _tmpbuffer.resize(0);
        _lastSyncTimeStamp = OpenRAVE::utils::GetMilliTime();
        bool bcallsetup = false;
        bool bAttachedBodiesChanged = false;
        KinBodyConstPtr ptrackingbody = _ptrackingbody.lock();
        if( !!ptrackingbody && _bTrackActiveDOF ) {
            std::map<int, KinBodyCache>::iterator ittracking = mapCachedBodies.find(ptrackingbody->GetEnvironmentId());
            if( ittracking == mapCachedBodies.end() ) {
                std::string ssinfo;
                FOREACH(itcache, mapCachedBodies) {
                    KinBodyConstPtr pbody = itcache->second.pwbody.lock(); ///< weak pointer to body
                    if( !!pbody ) {
                        ssinfo += str(boost::format("(id=%d, linkmask=0x%x, numcols=%d, name=%s), ")%itcache->first%_GetLinkMask(itcache->second.linkEnableStates)%itcache->second.vcolobjs.size()%pbody->GetName());
                    }
                    else {
                        ssinfo += str(boost::format("id=%d, linkmask=0x%x, numcols=%d")%itcache->first%_GetLinkMask(itcache->second.linkEnableStates)%itcache->second.vcolobjs.size());
                    }
                }
                RAVELOG_WARN_FORMAT("%x tracking body not in current cached bodies (tracking body %s (id=%d)) (env %d). Current cache is: %s", this%ptrackingbody->GetName()%ptrackingbody->GetEnvironmentId()%ptrackingbody->GetEnv()->GetId()%ssinfo);
            }
            else {
                FCLSpace::KinBodyInfoPtr pinfo = ittracking->second.pwinfo.lock();
                FCLSpace::KinBodyInfoPtr pnewinfo = _fclspace.GetInfo(*ptrackingbody); // necessary in case pinfos were swapped!
                if( ittracking->second.nActiveDOFUpdateStamp != pnewinfo->nActiveDOFUpdateStamp ) {
                    if( ptrackingbody->IsRobot() ) {
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
                                            if( !!ittracking->second.vcolobjs.at(ilink) ) {
                                                pmanager->replaceObject(ittracking->second.vcolobjs.at(ilink).get(), pcolobj.get(), false);
                                            }
                                            else {
                                                pmanager->registerObject(pcolobj.get());
                                            }
    #else
                                            // no replace
                                            if( !!ittracking->second.vcolobjs.at(ilink) ) {
                                                pmanager->unregisterObject(ittracking->second.vcolobjs.at(ilink).get());
                                            }
    #ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                            SaveCollisionObjectDebugInfos(pcolobj.get());
    #endif
                                            pmanager->registerObject(pcolobj.get());
    #endif
                                            bcallsetup = true;
                                        }
                                        else {
                                            if( !!ittracking->second.vcolobjs.at(ilink) ) {
                                                pmanager->unregisterObject(ittracking->second.vcolobjs.at(ilink).get());
                                            }
                                        }
                                        ittracking->second.vcolobjs.at(ilink) = pcolobj;
                                    }
                                    else {
                                        if( !bIsActiveLinkEnabled && !!ittracking->second.vcolobjs.at(ilink) ) {
                                            //RAVELOG_VERBOSE_FORMAT("env=%d %x resetting cached colobj %s %d", probot->GetEnv()->GetId()%this%probot->GetName()%ilink);
                                            pmanager->unregisterObject(ittracking->second.vcolobjs.at(ilink).get());
                                            ittracking->second.vcolobjs.at(ilink).reset();
                                        }
                                    }
                                }

                                ittracking->second.nActiveDOFUpdateStamp = pnewinfo->nActiveDOFUpdateStamp;
                                ittracking->second.linkEnableStates = _linkEnableStates;
                            }
                        }
                    }
                }
            }
        }

        std::map<int, KinBodyCache>::iterator itcache = mapCachedBodies.begin();
        while(itcache != mapCachedBodies.end()) {
            KinBodyConstPtr pbody = itcache->second.pwbody.lock();
            FCLSpace::KinBodyInfoPtr pinfo = itcache->second.pwinfo.lock();

            if( !pbody || pbody->GetEnvironmentId() == 0 ) {
                // should happen when parts are removed
                RAVELOG_VERBOSE_FORMAT("%u manager contains invalid body %s, removing for now (env %d)", _lastSyncTimeStamp%(!pbody ? std::string() : pbody->GetName())%(!pbody ? -1 : pbody->GetEnv()->GetId()));
                FOREACH(itcolobj, itcache->second.vcolobjs) {
                    if( !!itcolobj->get() ) {
                        pmanager->unregisterObject(itcolobj->get());
                    }
                }
                itcache->second.vcolobjs.resize(0);
                mapCachedBodies.erase(itcache++);
                continue;
            }

            FCLSpace::KinBodyInfoPtr pnewinfo = _fclspace.GetInfo(*pbody); // necessary in case pinfos were swapped!
            if( pinfo != pnewinfo ) {
                // everything changed!
                RAVELOG_VERBOSE_FORMAT("%u body %s entire KinBodyInfo changed", _lastSyncTimeStamp%pbody->GetName());
                FOREACH(itcolobj, itcache->second.vcolobjs) {
                    if( !!itcolobj->get() ) {
                        pmanager->unregisterObject(itcolobj->get());
                    }
                }
                itcache->second.vcolobjs.resize(0);
                _AddBody(pbody, pnewinfo, itcache->second.vcolobjs, itcache->second.linkEnableStates, _bTrackActiveDOF&&pbody==ptrackingbody);
                itcache->second.pwinfo = pnewinfo;
                //itcache->second.ResetStamps();
                // need to update the stamps here so that we do not try to unregisterObject below and get into an error
                itcache->second.nLastStamp = pnewinfo->nLastStamp;
                itcache->second.nLinkUpdateStamp = pnewinfo->nLinkUpdateStamp;
                itcache->second.nGeometryUpdateStamp = pnewinfo->nGeometryUpdateStamp;
                itcache->second.nAttachedBodiesUpdateStamp = -1;
                itcache->second.nActiveDOFUpdateStamp = pnewinfo->nActiveDOFUpdateStamp;
                itcache->second.geometrygroup = pnewinfo->_geometrygroup;
                pinfo = pnewinfo;
                if( _tmpbuffer.size() > 0 ) {
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                    SaveCollisionObjectDebugInfos();
#endif
                    pmanager->registerObjects(_tmpbuffer); // bulk update
                    _tmpbuffer.resize(0);
                }
            }

            if( pinfo->nLinkUpdateStamp != itcache->second.nLinkUpdateStamp ) {
                // links changed
                std::vector<uint8_t> newLinkEnableStates;
                pbody->GetLinkEnableStates(newLinkEnableStates);
                if( _bTrackActiveDOF && ptrackingbody == pbody ) {
                    for(size_t itestlink = 0; itestlink < _vTrackingActiveLinks.size(); ++itestlink) {
                        if( !_vTrackingActiveLinks[itestlink] ) {
                            newLinkEnableStates.at(itestlink) = 0;
                        }
                    }
                }

                //uint64_t changed = itcache->second.linkmask ^ newlinkmask;
                //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), lastsync=%u body %s (%d) for cache changed link %d != %d, linkmask=0x%x", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%pbody->GetName()%pbody->GetEnvironmentId()%pinfo->nLinkUpdateStamp%itcache->second.nLinkUpdateStamp%_GetLinkMask(newLinkEnableStates));
                for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                    uint8_t changed = itcache->second.linkEnableStates.at(ilink) != newLinkEnableStates.at(ilink);
                    if( changed ) {
                        if( newLinkEnableStates.at(ilink) ) {
                            CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pinfo, ilink);
                            if( !!pcolobj ) {
#ifdef FCLRAVE_USE_REPLACEOBJECT
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    pmanager->replaceObject(itcache->second.vcolobjs.at(ilink).get(), pcolobj.get(), false);
                                }
                                else {
                                    pmanager->registerObject(pcolobj.get());
                                }
#else

                                // no replace
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                                }
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                pmanager->registerObject(pcolobj.get());
#endif
                                bcallsetup = true;
                            }
                            else {
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                                }
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

                itcache->second.linkEnableStates = newLinkEnableStates;
                itcache->second.nLinkUpdateStamp = pinfo->nLinkUpdateStamp;
            }
            if( pinfo->nGeometryUpdateStamp != itcache->second.nGeometryUpdateStamp ) {

                if( itcache->second.geometrygroup.size() == 0 || itcache->second.geometrygroup != pinfo->_geometrygroup ) {
                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), lastsync=%u body %s (%d) for cache changed geometry %d != %d, linkmask=0x%x", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%pbody->GetName()%pbody->GetEnvironmentId()%pinfo->nGeometryUpdateStamp%itcache->second.nGeometryUpdateStamp%_GetLinkMask(itcache->second.linkEnableStates));
                    // vcolobjs most likely changed
                    for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                        if( itcache->second.linkEnableStates.at(ilink) ) {
                            CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pinfo, ilink);
                            if( !!pcolobj ) {
                                //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s adding obj %x from link %d", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%pbody->GetName()%pcolobj.get()%ilink);
#ifdef FCLRAVE_USE_REPLACEOBJECT
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s replacing cached obj %x with %x ", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%pbody->GetName()%itcache->second.vcolobjs.at(ilink).get()%pcolobj.get());
                                    pmanager->replaceObject(itcache->second.vcolobjs.at(ilink).get(), pcolobj.get(), false);
                                }
                                else {
                                    pmanager->registerObject(pcolobj.get());
                                }
#else

                                // no replace
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s unregister cached obj %x ", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%pbody->GetName()%itcache->second.vcolobjs.at(ilink).get());
                                    fcl::CollisionObject* ptestobj = itcache->second.vcolobjs.at(ilink).get();
                                    pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                                }
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                pmanager->registerObject(pcolobj.get());
#endif
                                bcallsetup = true;
                                itcache->second.vcolobjs.at(ilink) = pcolobj;
                            }
                            else {
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x, body %s removing old obj from link %d", pbody->GetEnv()->GetId()%this%pbody->GetName()%ilink);
                                    pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                                    itcache->second.vcolobjs.at(ilink).reset();
                                }
                            }
                        }
                    }
                    itcache->second.geometrygroup = pinfo->_geometrygroup;
                }
                else {
                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x, lastsync=%u body %s (%d) for cache changed geometry but no update %d != %d, geometrygroup=%s", pbody->GetEnv()->GetId()%this%_lastSyncTimeStamp%pbody->GetName()%pbody->GetEnvironmentId()%pinfo->nGeometryUpdateStamp%itcache->second.nGeometryUpdateStamp%itcache->second.geometrygroup);
                }
                itcache->second.nGeometryUpdateStamp = pinfo->nGeometryUpdateStamp;
            }
            if( pinfo->nLastStamp != itcache->second.nLastStamp ) {
                if( IS_DEBUGLEVEL(OpenRAVE::Level_Verbose) ) {
                    //Transform tpose = pbody->GetTransform();
                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d) %u body %s (%d) for cache changed transform %d != %d, num=%d, mask=0x%x, trans=(%.3f, %.3f, %.3f)", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%pbody->GetName()%pbody->GetEnvironmentId()%pinfo->nLastStamp%itcache->second.nLastStamp%pinfo->vlinks.size()%_GetLinkMask(itcache->second.linkEnableStates)%tpose.trans.x%tpose.trans.y%tpose.trans.z);
                }
                // transform changed
                for(uint64_t ilink = 0; ilink < pinfo->vlinks.size(); ++ilink) {
                    if( itcache->second.linkEnableStates.at(ilink) ) {
                        CollisionObjectPtr pcolobj = _fclspace.GetLinkBV(*pinfo, ilink);
                        if( !!pcolobj ) {
                            //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s adding obj %x from link %d", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%pbody->GetName()%pcolobj.get()%ilink);
                            if( itcache->second.vcolobjs.at(ilink) == pcolobj ) {
#ifdef FCLRAVE_USE_BULK_UPDATE
                                // same object, so just update
                                pmanager->update(itcache->second.vcolobjs.at(ilink).get(), false);
#else
                                // Performance issue !!
                                pmanager->update(itcache->second.vcolobjs.at(ilink).get());
#endif
                            }
                            else {
#ifdef FCLRAVE_USE_REPLACEOBJECT
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif

                                // different object, have to replace
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s replacing cached obj %x with %x ", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%pbody->GetName()%itcache->second.vcolobjs.at(ilink).get()%pcolobj.get());
                                    pmanager->replaceObject(itcache->second.vcolobjs.at(ilink).get(), pcolobj.get(), false);
                                }
                                else {
                                    pmanager->registerObject(pcolobj.get());
                                }
#else // FCLRAVE_USE_REPLACEOBJECT

                                // no replace
                                if( !!itcache->second.vcolobjs.at(ilink) ) {
                                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d), body %s unregister cached obj %x ", pbody->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecke()%pbody->GetName()%itcache->second.vcolobjs.at(ilink).get());
                                    fcl::CollisionObject* ptestobj = itcache->second.vcolobjs.at(ilink).get();
                                    pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                                }
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
                                SaveCollisionObjectDebugInfos(pcolobj.get());
#endif
                                pmanager->registerObject(pcolobj.get());
#endif // FCLRAVE_USE_REPLACEOBJECT
                            }

                            bcallsetup = true;
                            itcache->second.vcolobjs.at(ilink) = pcolobj;
                        }
                        else {
                            if( !!itcache->second.vcolobjs.at(ilink) ) {
                                //RAVELOG_VERBOSE_FORMAT("env=%d, %x, body %s removing old obj from link %d", pbody->GetEnv()->GetId()%this%pbody->GetName()%ilink);
                                pmanager->unregisterObject(itcache->second.vcolobjs.at(ilink).get());
                                itcache->second.vcolobjs.at(ilink).reset();
                            }
                        }
                    }
                }

                itcache->second.nLastStamp = pinfo->nLastStamp;
            }
            if( pinfo->nAttachedBodiesUpdateStamp != itcache->second.nAttachedBodiesUpdateStamp ) {
                //RAVELOG_VERBOSE_FORMAT("env=%d, %u the nAttachedBodiesUpdateStamp changed %d != %d, trackingbody is %s", pbody->GetEnv()->GetId()%_lastSyncTimeStamp%pinfo->nAttachedBodiesUpdateStamp%itcache->second.nAttachedBodiesUpdateStamp%(!!ptrackingbody ? ptrackingbody->GetName() : std::string()));
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
            std::vector<CollisionObjectPtr> vcolobjs;
            //RAVELOG_VERBOSE_FORMAT("env=%d, %x %u setting %d attached bodies for body %s (%d)", ptrackingbody->GetEnv()->GetId()%this%_lastSyncTimeStamp%attachedBodies.size()%ptrackingbody->GetName()%ptrackingbody->GetEnvironmentId());
            // add any new bodies
            FOREACH(itbody, attachedBodies) {
                if( mapCachedBodies.find((*itbody)->GetEnvironmentId()) == mapCachedBodies.end() ) {
                    FCLSpace::KinBodyInfoPtr pinfo = _fclspace.GetInfo(**itbody);
                    _linkEnableStates.resize((*itbody)->GetLinks().size()); ///< links that are currently inside the manager
                    std::fill(_linkEnableStates.begin(), _linkEnableStates.end(), 0);
                    vcolobjs.resize(0);
                    if( _AddBody(*itbody, pinfo, vcolobjs, _linkEnableStates, _bTrackActiveDOF&&(*itbody == ptrackingbody)) ) {
                        bcallsetup = true;
                    }
                    //RAVELOG_VERBOSE_FORMAT("env=%d, %x (self=%d) %u adding body %s (%d)", (*itbody)->GetEnv()->GetId()%this%_fclspace.IsSelfCollisionChecker()%_lastSyncTimeStamp%(*itbody)->GetName()%(*itbody)->GetEnvironmentId());
                    mapCachedBodies[(*itbody)->GetEnvironmentId()] = KinBodyCache(*itbody, pinfo);
                    mapCachedBodies[(*itbody)->GetEnvironmentId()].vcolobjs.swap(vcolobjs);
                    mapCachedBodies[(*itbody)->GetEnvironmentId()].linkEnableStates = _linkEnableStates;
//                    else {
//                        RAVELOG_VERBOSE_FORMAT("env=%d %x could not add attached body %s for tracking body %s", ptrackingbody->GetEnv()->GetId()%this%(*itbody)->GetName()%ptrackingbody->GetName());
//                    }
                }
            }

            // remove bodies not attached anymore
            itcache = mapCachedBodies.begin();
            while(itcache != mapCachedBodies.end()) {
                KinBodyConstPtr pbody = itcache->second.pwbody.lock();
                // could be the case that the same pointer was re-added to the environment so have to check the environment id
                if( !pbody || attachedBodies.count(pbody) == 0 || pbody->GetEnvironmentId() != itcache->first ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, %x, %u removing old cache %d", pbody->GetEnv()->GetId()%this%_lastSyncTimeStamp%itcache->first);
                    // not in attached bodies so should remove
                    FOREACH(itcol, itcache->second.vcolobjs) {
                        if( !!itcol->get() ) {
                            pmanager->unregisterObject(itcol->get());
                        }
                    }
                    itcache->second.vcolobjs.resize(0);
                    mapCachedBodies.erase(itcache++);
                }
                else {
                    ++itcache;
                }
            }
        }
        if( _tmpbuffer.size() > 0 ) {
#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
            SaveCollisionObjectDebugInfos();
#endif
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
    bool _AddBody(KinBodyConstPtr pbody, FCLSpace::KinBodyInfoPtr pinfo, std::vector<CollisionObjectPtr>& vcolobjs, std::vector<uint8_t>& linkEnableStates, bool bTrackActiveDOF)
    {
        vcolobjs.resize(0); // reset so that existing collision objects can go away
        vcolobjs.resize(pbody->GetLinks().size(), CollisionObjectPtr());
        bool bsetUpdateStamp = false;
        linkEnableStates.resize(pbody->GetLinks().size()); ///< links that are currently inside the manager
        std::fill(linkEnableStates.begin(), linkEnableStates.end(), 0);
        FOREACH(itlink, (pbody)->GetLinks()) {
            if( (*itlink)->IsEnabled() && (!bTrackActiveDOF || _vTrackingActiveLinks.at((*itlink)->GetIndex())) ) {
                //pinfo->vlinks.at((*itlink)->GetIndex()).listRegisteredManagers.push_back(shared_from_this());
                CollisionObjectPtr pcol = _fclspace.GetLinkBV(*pinfo, (*itlink)->GetIndex());
                if( !!pcol ) {
                    if( find(_tmpbuffer.begin(), _tmpbuffer.end(), pcol.get()) == _tmpbuffer.end() ) {
                        _tmpbuffer.push_back(pcol.get());
                    }
                    else {
                        RAVELOG_WARN_FORMAT("env=%d body %s link %s is added multiple times", pbody->GetEnv()->GetId()%pbody->GetName()%(*itlink)->GetName());
                    }
                    vcolobjs[(*itlink)->GetIndex()] = pcol;
                    linkEnableStates.at((*itlink)->GetIndex()) = 1;
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
    std::map<int, KinBodyCache> mapCachedBodies; ///< pair of (body id, (weak body, updatestamp)) where the key is KinBody::GetEnvironmentId
    uint32_t _lastSyncTimeStamp; ///< timestamp when last synchronized

    std::set<int> _setExcludeBodyIds; ///< any bodies that should not be considered inside the manager, used with environment mode
    CollisionGroup _tmpbuffer; ///< cache

    KinBodyConstWeakPtr _ptrackingbody; ///< if set, then only tracking the attached bodies if this body
    std::vector<int> _vTrackingActiveLinks; ///< indices of which links are active for tracking body
    std::vector<uint8_t> _linkEnableStates; ///< links that are currently inside the manager

    bool _bTrackActiveDOF; ///< if true and _ptrackingbody is valid, then should be tracking the active dof of the _ptrackingbody

#ifdef FCLRAVE_DEBUG_COLLISION_OBJECTS
    void SaveCollisionObjectDebugInfos() {
        FOREACH(itpcollobj, _tmpbuffer) {
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
