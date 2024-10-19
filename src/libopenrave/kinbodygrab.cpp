// -*- coding: utf-8 -*-
// Copyright (C) 2006-2017 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "libopenrave.h"

namespace OpenRAVE {

/// \brief Push link to listNonCollidingLinksWhenGrabbed, only if it has no collision with the whole grabbedBody.
static void _PushLinkIfNonColliding(std::list<KinBody::LinkConstPtr>& listNonCollidingLinksWhenGrabbed,
                                    CollisionCheckerBasePtr& pchecker,
                                    const KinBody::LinkPtr& pLinkToCheck, const KinBody& grabbedBody)
{
    KinBody::LinkConstPtr pLinkToCheckConst(pLinkToCheck);
    for (const KinBody::LinkPtr& pGrabbedBodylink : grabbedBody.GetLinks()) {
        if( pchecker->CheckCollision(pLinkToCheckConst, KinBody::LinkConstPtr(pGrabbedBodylink)) ) {
            return; // if colliding, do not push.
        }
    }
    // if not colliding with any of links in grabbedBody, push it.
    listNonCollidingLinksWhenGrabbed.push_back(pLinkToCheck);
}

/// \brief remove link from listNonCollidingLinksWhenGrabbed if its parent is same as the given body.
template<typename KinBodyPtrT>
static void _RemoveLinkFromListNonCollidingLinksWhenGrabbed(std::list<KinBody::LinkConstPtr>& listNonCollidingLinksWhenGrabbed,
                                                            const KinBodyPtrT& pGrabbedBody)
{
    for (std::list<KinBody::LinkConstPtr>::iterator itlink = listNonCollidingLinksWhenGrabbed.begin(); itlink != listNonCollidingLinksWhenGrabbed.end();) {
        if ((*itlink)->GetParent() == pGrabbedBody) {
            itlink = listNonCollidingLinksWhenGrabbed.erase(itlink);
        }
        else {
            ++itlink;
        }
    }
}

/// \brief get one grabbed info from pgrabbed and pgrabbedbody.
/// \param[out] outputinfo : result
static void _GetOneGrabbedInfo(KinBody::GrabbedInfo& outputinfo,
                               const GrabbedPtr& pgrabbed, const KinBodyPtr& pgrabbedbody, const std::vector<KinBody::LinkPtr>& veclinks,
                               const std::string& bodyName, const EnvironmentBasePtr& pEnv)
{
    outputinfo._grabbedname = pgrabbedbody->GetName();
    outputinfo._robotlinkname = pgrabbed->_pGrabbingLink->GetName();
    outputinfo._trelative = pgrabbed->_tRelative;
    outputinfo._setIgnoreRobotLinkNames.clear();
    CopyRapidJsonDoc(pgrabbed->_rGrabbedUserData, outputinfo._rGrabbedUserData);

    for( int linkIndex : pgrabbed->_setGrabberLinkIndicesToIgnore ) {
        if (0 <= linkIndex && linkIndex < (int)veclinks.size()) {
            outputinfo._setIgnoreRobotLinkNames.insert(veclinks[linkIndex]->GetName());
        }
        else {
            RAVELOG_WARN_FORMAT("env=%s, grabbed body '%s' of body '%s' has an invalid grabber link index %d to ignore. number of links is %d.", pEnv->GetNameId()%pgrabbedbody->GetName()%bodyName%linkIndex%veclinks.size());
        }
    }
}

/// \brief create saver for grabbed/grabber.
static void _CreateSaverForGrabbedAndGrabber(KinBody::KinBodyStateSaverPtr& pSaver,
                                             const KinBodyPtr& pBody,
                                             const int defaultSaveOptions,
                                             const bool bDisableRestoreOnDestructor)
{
    if( pBody->IsRobot() ) {
        RobotBasePtr pRobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pBody);
        pSaver.reset(new RobotBase::RobotStateSaver(pRobot, defaultSaveOptions|KinBody::Save_ConnectedBodies));
    }
    else {
        pSaver.reset(new KinBody::KinBodyStateSaver(pBody, defaultSaveOptions));
    }
    if( bDisableRestoreOnDestructor ) {
        pSaver->SetRestoreOnDestructor(false);
    }
}

Grabbed::Grabbed(KinBodyPtr pGrabbedBody, KinBody::LinkPtr pGrabbingLink)
{
    _pGrabbedBody = pGrabbedBody;
    _pGrabbingLink = pGrabbingLink;
    _pGrabbingLink->GetRigidlyAttachedLinks(_vAttachedToGrabbingLink);
    _listNonCollidingIsValid = false;
    const bool bDisableRestoreOnDestructor = true; // This is very important! These saver are used only in ComputeListNonCollidingLinks and we don't want to restore on destructor.
    _CreateSaverForGrabbedAndGrabber(_pGrabbedSaver,
                                     pGrabbedBody,
                                     KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_JointLimits,
                                     bDisableRestoreOnDestructor);

    KinBodyPtr pGrabber = RaveInterfaceCast<KinBody>(_pGrabbingLink->GetParent());
    _CreateSaverForGrabbedAndGrabber(_pGrabberSaver,
                                     pGrabber,
                                     KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_JointLimits|KinBody::Save_LinkVelocities, // Need to save link velocities of the grabber since will be used for computing link velocities of the grabbed bodies.
                                     bDisableRestoreOnDestructor);
} // end Grabbed

void Grabbed::AddMoreIgnoreLinks(const std::set<int>& setAdditionalGrabberLinksToIgnore)
{
    KinBodyPtr pGrabber = RaveInterfaceCast<KinBody>(_pGrabbingLink->GetParent());
    FOREACHC(itLinkIndexToIgnore, setAdditionalGrabberLinksToIgnore) {
        _setGrabberLinkIndicesToIgnore.insert(*itLinkIndexToIgnore);

        if( _listNonCollidingIsValid ) {
            KinBody::LinkPtr pGrabberLink = pGrabber->GetLinks().at(*itLinkIndexToIgnore);
            _listNonCollidingLinksWhenGrabbed.remove(pGrabberLink);
        }
    }
}

void Grabbed::ComputeListNonCollidingLinks()
{
    if( _listNonCollidingIsValid ) {
        return;
    }

    // Save the current state before proceeding with the computation
    KinBodyPtr pGrabbedBody(_pGrabbedBody);
    KinBodyPtr pGrabber = RaveInterfaceCast<KinBody>(_pGrabbingLink->GetParent());
    KinBody::KinBodyStateSaverPtr pCurrentGrabbedSaver, pCurrentGrabberSaver;
    const int defaultSaveOptions = KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_LinkVelocities|KinBody::Save_JointLimits;
    const bool bDisableRestoreOnDestructor = false;
    _CreateSaverForGrabbedAndGrabber(pCurrentGrabbedSaver, pGrabbedBody, defaultSaveOptions, bDisableRestoreOnDestructor);
    _CreateSaverForGrabbedAndGrabber(pCurrentGrabberSaver, pGrabber, defaultSaveOptions, bDisableRestoreOnDestructor);
    // RAVELOG_INFO_FORMAT("env=%s, computing _listNonCollidingLinksWhenGrabbed for body %s", pGrabbedBody->GetEnv()->GetNameId()%pGrabbedBody->GetName());

    // Now that the current state is saved, set the state to the original (when Grab was called)
    // Note that the computation here is based on what is currently grabbed by the grabber, this is unavoidable since the grabbed bodies by the grabber could be different from when the Grabbed instance was created.
    _pGrabbedSaver->Restore();
    _pGrabberSaver->Restore(); // note that this Restore also updates other grabbed bodies.

    // Actual computation of _listNonCollidingLinksWhenGrabbed
    _listNonCollidingLinksWhenGrabbed.clear();
    EnvironmentBasePtr penv = pGrabber->GetEnv();
    // if( 1 ) {
    //     std::stringstream ssdebug;
    //     std::vector<dReal> vGrabberValues;
    //     pGrabber->GetDOFValues(vGrabberValues);
    //     Transform tGrabbedBody = pGrabbedBody->GetTransform();
    //     ssdebug << "vGrabberValues=[";
    //     FOREACHC(itval, vGrabberValues) {
    //         ssdebug << (*itval) << ",";
    //     }
    //     ssdebug << "]; grabbedBodyPose=[";
    //     ssdebug << tGrabbedBody.rot.x << ",";
    //     ssdebug << tGrabbedBody.rot.y << ",";
    //     ssdebug << tGrabbedBody.rot.z << ",";
    //     ssdebug << tGrabbedBody.rot.w << ",";
    //     ssdebug << tGrabbedBody.trans.x << ",";
    //     ssdebug << tGrabbedBody.trans.y << ",";
    //     ssdebug << tGrabbedBody.trans.z << "];";
    //     RAVELOG_INFO_FORMAT("env=%s, restored state before for computing _listNonCollidingLinksWhenGrabbed: %s", penv->GetNameId()%ssdebug.str());
    // }

    CollisionCheckerBasePtr pchecker = pGrabber->GetSelfCollisionChecker();
    if( !pchecker ) {
        pchecker = penv->GetCollisionChecker();
    }
    CollisionOptionsStateSaver colOptionsSaver(pchecker, /*newcollisionoptions*/ CO_IgnoreCallbacks); // reset collision options before proceeding
    {
        // Flatten the grabbed bodies so that we can zip our iteration with the cache of locked pointers
        // Use raw pointers to save overhead here since lifetime is guaranteed by _grabbedBodiesByEnvironmentIndex
        std::vector<Grabbed*> vGrabbedBodies;
        vGrabbedBodies.reserve(pGrabber->_grabbedBodiesByEnvironmentIndex.size());
        // locking weak pointer is expensive, so do it N times and cache, where N is the number of grabbedBody instead of N^2
        std::vector<KinBodyPtr> vLockedGrabbedBodiesCache;
        vLockedGrabbedBodiesCache.reserve(pGrabber->_grabbedBodiesByEnvironmentIndex.size());
        for (const KinBody::MapGrabbedByEnvironmentIndex::value_type& otherGrabbedPair : pGrabber->_grabbedBodiesByEnvironmentIndex) {
            const GrabbedPtr& pOtherGrabbed = otherGrabbedPair.second;

            // Remove this pGrabbedBody from _listNonCollidingLinksWhenGrabbed in other grabbed.
            // The condition when pOtherGrabbedBody is checked with pGrabbedBody might be different from the condition when this pGrabbedBody is checked with pOtherGrabbedBody now.
            // In such case, it's reasonable to respect the latest condition.
            // To do so, remove pOtherGrabbed->_listNonCollidingLinksWhenGrabbed first and this function will add it at the end of this function if necessary to this Grabbed's _listNonCollidingLinksWhenGrabbed.
            // Note that the _listNonCollidingLinksWhenGrabbed result might not be symmetric between pOtherGrabbed and this Grabbed, e.g. this Grabbed's _listNonCollidingLinksWhenGrabbed might contain pOtherGrabbedBody, but pOtherGrabbed->_listNonCollidingLinksWhenGrabbed does not contain pGrabbedBody.
            // Even if there is such asymmetricity, KinBody::CheckSelfCollision will consider the collision checking pair correctly.
            _RemoveLinkFromListNonCollidingLinksWhenGrabbed<KinBodyPtr>(pOtherGrabbed->_listNonCollidingLinksWhenGrabbed, pGrabbedBody);

            // extract valid pointers
            KinBodyPtr pOtherGrabbedBody = pOtherGrabbed->_pGrabbedBody.lock();
            if( !pOtherGrabbedBody ) {
                RAVELOG_WARN_FORMAT("env=%s, other grabbed body on %s has already been released. So ignoring it.", penv->GetNameId()%pGrabber->GetName());
                continue;
            }
            if( pOtherGrabbedBody->GetEnvironmentBodyIndex() == 0 ) {
                RAVELOG_WARN_FORMAT("env=%s, other grabbed body on %s has invalid environment body index. Perhaps already removed from the environment. So ignoring it.", penv->GetNameId()%pGrabber->GetName());
                continue;
            }
            if( pOtherGrabbedBody->GetLinks().empty() ) {
                RAVELOG_WARN_FORMAT("env=%s, other grabbed body %s on %s has no links. Perhaps not initialized. So ignoring it.", penv->GetNameId()%pOtherGrabbedBody->GetName()%pGrabber->GetName());
                continue;
            }

            if( pOtherGrabbedBody != pGrabbedBody ) {
                vGrabbedBodies.emplace_back(pOtherGrabbed.get());
                vLockedGrabbedBodiesCache.push_back(pOtherGrabbedBody);
            }
        }

        KinBody::KinBodyStateSaver grabbedEnableSaver(pGrabbedBody, KinBody::Save_LinkEnable);
        pGrabbedBody->Enable(true);
        KinBody::KinBodyStateSaver grabberEnableSaver(pGrabber, KinBody::Save_LinkEnable);
        pGrabber->Enable(true);

        // Check grabbed body vs grabber links
        const KinBody& grabbedBody = *pGrabbedBody;
        FOREACHC(itGrabberLink, pGrabber->GetLinks()) {
            if( std::find(_vAttachedToGrabbingLink.begin(), _vAttachedToGrabbingLink.end(), *itGrabberLink) != _vAttachedToGrabbingLink.end() ) {
                // This link (*itGrabberLink) is rigidly attached to _pGrabbingLink. Therefore, isNonColliding = false, meaning that we will not check collision between this link and the grabbed body later on.
            }
            else {
                // This link (*itGrabberLink) is *not* rigidly attached to _pGrabbingLink.
                if( _setGrabberLinkIndicesToIgnore.find((*itGrabberLink)->GetIndex()) == _setGrabberLinkIndicesToIgnore.end() ) {
                    // Not ignoring collisions between this link and the grabber body
                    _PushLinkIfNonColliding(_listNonCollidingLinksWhenGrabbed, pchecker, *itGrabberLink, grabbedBody);
                }
            }
        }

        // Check grabbed body vs other existing grabbed bodies
        // int iOtherGrabbed = -1;
        for(int iOtherGrabbed = 0; iOtherGrabbed < (int)vGrabbedBodies.size(); ++iOtherGrabbed) {
            {
                const KinBodyPtr& pOtherGrabbedBody = vLockedGrabbedBodiesCache[iOtherGrabbed];
                // sufficient to check one direction, whether grabbing link of body 2 is in links attached rigidly to grabbing link of body 1. Result is same for both directions.
                const bool bTwoGrabbedBodiesHaveStaticRelativePose = std::find(_vAttachedToGrabbingLink.begin(), _vAttachedToGrabbingLink.end(), vGrabbedBodies[iOtherGrabbed]->_pGrabbingLink) != _vAttachedToGrabbingLink.end();
                // if two grabbed bodies have static (constant) relative pose with respect to each other, do not need to check collision between them for the rest of time.
                if (!bTwoGrabbedBodiesHaveStaticRelativePose) {
                    KinBody::KinBodyStateSaver otherGrabbedEnableSaver(pOtherGrabbedBody, KinBody::Save_LinkEnable);
                    pOtherGrabbedBody->Enable(true);
                    for (const KinBody::LinkPtr& pOtherGrabbedLink : pOtherGrabbedBody->GetLinks()) {
                        _PushLinkIfNonColliding(_listNonCollidingLinksWhenGrabbed, pchecker, pOtherGrabbedLink, grabbedBody);
                    }
                }
            }
        }
    }

    _listNonCollidingIsValid = true;
    // if( 1 ) {
    //     std::stringstream ssdebug;
    //     ssdebug << "grabbedBody='" << pGrabbedBody->GetName() << "'; listNonCollidingLinks=[";
    //     for( std::list<KinBody::LinkConstPtr>::const_iterator itLink = _listNonCollidingLinksWhenGrabbed.begin(); itLink != _listNonCollidingLinksWhenGrabbed.end(); ++itLink ) {
    //         ssdebug << "'" << (*itLink)->GetName() << "',";
    //     }
    //     ssdebug << "];";
    //     RAVELOG_INFO_FORMAT("env=%s, %s", penv->GetNameId()%ssdebug.str());
    // }
}

bool KinBody::Grab(KinBodyPtr pGrabbedBody, LinkPtr pGrabbingLink, const rapidjson::Value& rGrabbedUserData)
{
    // always ignore links that are statically attached to plink (ie assume they are always colliding with the body)
    std::set<int> setGrabberLinksToIgnore;
    std::vector<KinBody::LinkPtr> vAttachedToGrabbingLink;
    pGrabbingLink->GetRigidlyAttachedLinks(vAttachedToGrabbingLink);
    FOREACHC(itAttachedLink, vAttachedToGrabbingLink) {
        setGrabberLinksToIgnore.insert((*itAttachedLink)->GetIndex());
    }
    return Grab(pGrabbedBody, pGrabbingLink, setGrabberLinksToIgnore, rGrabbedUserData);
}

bool KinBody::Grab(KinBodyPtr pGrabbedBody, LinkPtr pGrabbingLink, const std::set<std::string>& setIgnoreGrabberLinkNames, const rapidjson::Value& rGrabbedUserData)
{
    std::set<int> setGrabberLinksToIgnore;
    FOREACHC(itLinkName, setIgnoreGrabberLinkNames) {
        setGrabberLinksToIgnore.insert(GetLink(*itLinkName)->GetIndex());
    }
    return Grab(pGrabbedBody, pGrabbingLink, setGrabberLinksToIgnore, rGrabbedUserData);
}

bool KinBody::Grab(KinBodyPtr pGrabbedBody, LinkPtr pGrabbingLink, const std::set<int>& setGrabberLinksToIgnore, const rapidjson::Value& rGrabbedUserData)
{
    OPENRAVE_ASSERT_FORMAT(!!pGrabbedBody, "env=%s, body to be grabbed by body '%s' is invalid", GetEnv()->GetNameId()%GetName(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!pGrabbingLink, "env=%s, pGrabbingLink of body '%s' for grabbing body '%s' is invalid", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pGrabbingLink->GetParent().get() == this, "env=%s, pGrabbingLink name='%s' for grabbing '%s' is not part of body '%s'", GetEnv()->GetNameId()%pGrabbingLink->GetName()%pGrabbedBody->GetName()%GetName(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pGrabbedBody.get() != this, "env=%s, body '%s' cannot grab itself", GetEnv()->GetNameId()%pGrabbedBody->GetName(), ORE_InvalidArguments);

    // If pGrabbedBody has previously been grabbed, check if the grabbing condition is the same
    GrabbedPtr pPreviouslyGrabbed;
    MapGrabbedByEnvironmentIndex::iterator itPreviouslyGrabbed;
    FOREACHC(itGrabbed, _grabbedBodiesByEnvironmentIndex) {
        GrabbedPtr& pGrabbed = itGrabbed->second;
        if( pGrabbed->_pGrabbedBody.lock() == pGrabbedBody ) {
            pPreviouslyGrabbed = pGrabbed;
            itPreviouslyGrabbed = itGrabbed;
            break;
        }
    }

    // Double check if the grabbed body has anything attached to it. Collision checkers might not support this case.
    if( pGrabbedBody->HasAttached() ) {
        if( !!pPreviouslyGrabbed ) {
            RAVELOG_INFO_FORMAT("env=%s, body '%s' grabs body '%s' that has previously been grabbed", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName());
        }
        else if( IS_DEBUGLEVEL(Level_Verbose) ) {
            std::set<KinBodyPtr> setAttached;
            pGrabbedBody->GetAttached(setAttached);
            std::stringstream ss;
            if( setAttached.size() > 1 ) {
                FOREACH(itbody, setAttached) {
                    ss << (*itbody)->GetName() << ",";
                }
            }
            RAVELOG_VERBOSE_FORMAT("env=%s, body '%s' trying to grab body '%s' with %d attached bodies [%s]", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName()%setAttached.size()%ss.str());
        }
    }

    // The body has previously been grabbed. Check if anything changes.
    Transform tGrabbingLink = pGrabbingLink->GetTransform();
    Transform tGrabbedBody = pGrabbedBody->GetTransform();
    if( !!pPreviouslyGrabbed ) {
        if( pPreviouslyGrabbed->_pGrabbingLink == pGrabbingLink ) {
            dReal distError2 = TransformDistance2(tGrabbingLink*pPreviouslyGrabbed->_tRelative, tGrabbedBody);
            if( distError2 <= g_fEpsilonLinear ) {
                if (pPreviouslyGrabbed->_rGrabbedUserData == rGrabbedUserData ) {
                    // Grabbing the same object at the same relative transform with the same grabbing link with the same userdata.
                    // So just modify setGrabberLinksToIgnore and then return.
                    pPreviouslyGrabbed->AddMoreIgnoreLinks(setGrabberLinksToIgnore);
                    return true;
                }
                else {
                    RAVELOG_DEBUG_FORMAT("env=%s, body '%s' is already grabbing body '%s' but userdata differs.", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName());
                }
            }
            else {
                RAVELOG_DEBUG_FORMAT("env=%s, body '%s' is already grabbing body '%s' but grabbed body transform differs. distError2=%.15e", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName()%distError2);
            }

        }
        else {
            RAVELOG_DEBUG_FORMAT("env=%s, body '%s' is already grabbing body '%s' with link '%s' but the current desired grabbing link is '%s'", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName()%pPreviouslyGrabbed->_pGrabbingLink->GetName()%pGrabbingLink->GetName());
        }
        // Detach pGrabbedBody first before re-adding it.
        _RemoveAttachedBody(*pGrabbedBody);
        _RemoveGrabbedBody(itPreviouslyGrabbed);
    }

    GrabbedPtr pGrabbed(new Grabbed(pGrabbedBody, pGrabbingLink));
    pGrabbed->_tRelative = tGrabbingLink.inverse() * tGrabbedBody;
    pGrabbed->_setGrabberLinkIndicesToIgnore = setGrabberLinksToIgnore;

    if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
        // collision checking will not be automatically updated with environment calls, so need to do this manually
        _selfcollisionchecker->InitKinBody(pGrabbedBody);
    }

    std::pair<Vector, Vector> velocity = pGrabbingLink->GetVelocity();
    velocity.first += velocity.second.cross(tGrabbedBody.trans - tGrabbingLink.trans);
    pGrabbedBody->SetVelocity(velocity.first, velocity.second);
    CopyRapidJsonDoc(rGrabbedUserData, pGrabbed->_rGrabbedUserData);

    try {
        // if an exception happens in _AttachBody, have to remove from _vGrabbedBodies
        _AttachBody(pGrabbedBody);
    }
    catch(...) {
        RAVELOG_ERROR_FORMAT("env=%s, failed to attach %s to %s when grabbing", GetEnv()->GetNameId()%pGrabbedBody->GetName()%GetName());
        // do not call _selfcollisionchecker->RemoveKinBody since the same object might be re-attached later on and we should preserve the structures.
        throw;
    }

    BOOST_ASSERT(pGrabbedBody->GetEnvironmentBodyIndex() > 0);
    _grabbedBodiesByEnvironmentIndex[pGrabbedBody->GetEnvironmentBodyIndex()] = std::move(pGrabbed);

    try {
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
    catch( const std::exception& ex ) {
        RAVELOG_ERROR_FORMAT("env=%s, failed to post-process changed parameters: %s", GetEnv()->GetNameId()%ex.what());
        throw;
    }

    return true;
}

void KinBody::Release(KinBody &body)
{
    FOREACH(itgrabbed, _grabbedBodiesByEnvironmentIndex) {
        GrabbedPtr pgrabbed = itgrabbed->second;
        KinBodyConstPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        if( !!pgrabbedbody ) {
            bool bpointermatch = pgrabbedbody.get() == &body;
            bool bnamematch = pgrabbedbody->GetName() == body.GetName();
            if( bpointermatch != bnamematch ) {
                RAVELOG_WARN_FORMAT("env=%d, body %s has grabbed body %s (%d), but it does not match with %s (%d) ", GetEnv()->GetId()%GetName()%pgrabbedbody->GetName()%pgrabbedbody->GetEnvironmentBodyIndex()%body.GetName()%body.GetEnvironmentBodyIndex());
            }
            if( bpointermatch ) {
                _RemoveGrabbedBody(itgrabbed);
                _RemoveAttachedBody(body);
                _PostprocessChangedParameters(Prop_RobotGrabbed);
                return;
            }
        }
    }

    if( IS_DEBUGLEVEL(Level_Debug) ) {
        std::stringstream ss;
        for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
            KinBodyConstPtr pgrabbedbody = grabPair.second->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                ss << pgrabbedbody->GetName() << ", ";
            }
        }

        RAVELOG_DEBUG_FORMAT("env=%d, body %s is not grabbing body %s (%d), but grabbing bodies [%s]", GetEnv()->GetId()%GetName()%body.GetName()%body.GetEnvironmentBodyIndex()%ss.str());
    }
}

void KinBody::ReleaseAllGrabbed()
{
    // If we have no grabbed bodies, do nothing
    if (_grabbedBodiesByEnvironmentIndex.empty()) {
        return;
    }

    // Detach all previously grabbed bodies
    for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        KinBodyPtr pbody = grabPair.second->_pGrabbedBody.lock();
        if (!!pbody) {
            _RemoveAttachedBody(*pbody);
        }
    }

    // Clear our set of grabs
    _grabbedBodiesByEnvironmentIndex.clear();

    // Execute any hooks registered for grabs
    _PostprocessChangedParameters(Prop_RobotGrabbed);
}

void KinBody::ReleaseAllGrabbedWithLink(const KinBody::Link& bodyLinkToReleaseWith)
{
    OPENRAVE_ASSERT_FORMAT(bodyLinkToReleaseWith.GetParent().get() == this, "body %s invalid grab arguments", GetName(), ORE_InvalidArguments);

    // If we have no grabbed bodies, do nothing
    if (_grabbedBodiesByEnvironmentIndex.empty()) {
        return;
    }

    // Scan our grabs for any entries that use the grabbing link, and if found, remove
    bool didUngrabBody = false;
    for (MapGrabbedByEnvironmentIndex::iterator grabIt = _grabbedBodiesByEnvironmentIndex.begin(); grabIt != _grabbedBodiesByEnvironmentIndex.end(); /* nop */) {
        // If this isn't the right link, ignore
        if (grabIt->second->_pGrabbingLink.get() != &bodyLinkToReleaseWith) {
            grabIt++;
            continue;
        }

        // If it is, detach this body (if live)
        KinBodyPtr pbody = grabIt->second->_pGrabbedBody.lock();
        if (!!pbody) {
            _RemoveAttachedBody(*pbody);
        }

        // Erase this grab and flag that we need to run post-processing hooks when finished
        grabIt = _RemoveGrabbedBody(grabIt);

        didUngrabBody = true;
    }

    if (didUngrabBody) {
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
}

void KinBody::RegrabAll()
{
    CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
    CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options

    // Remove bodies from _listAttachedBodies first and then will add them back later. Maybe this is for triggering
    // postprocessing with Prop_BodyAttached?
    for (MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        KinBodyPtr pBody = grabPair.second->_pGrabbedBody.lock();
        if (!!pBody) {
            _RemoveAttachedBody(*pBody);
        }
    }

    // Swap out the previous set of grabbed bodies so that we can iterate and re-create
    MapGrabbedByEnvironmentIndex originalGrabbedBodiesByBodyName;
    originalGrabbedBodiesByBodyName.swap(_grabbedBodiesByEnvironmentIndex);

    // Regrab all the bodies again
    for (MapGrabbedByEnvironmentIndex::value_type& grabPair : originalGrabbedBodiesByBodyName) {
        const int grabbedBodyEnvIndex = grabPair.first;
        GrabbedPtr& pGrabbed = grabPair.second;

        // If this body has ceased to exist, don't re-add it
        KinBodyPtr pBody = pGrabbed->_pGrabbedBody.lock();
        if (!pBody) {
            RAVELOG_WARN_FORMAT("env=%s, grabbed body with index %d does not exist any more", GetEnv()->GetNameId()%grabbedBodyEnvIndex);
            continue;
        }

        GrabbedPtr pNewGrabbed(new Grabbed(pBody, pGrabbed->_pGrabbingLink));
        pNewGrabbed->_tRelative = pGrabbed->_tRelative;
        pNewGrabbed->_setGrabberLinkIndicesToIgnore = pGrabbed->_setGrabberLinkIndicesToIgnore;
        CopyRapidJsonDoc(pGrabbed->_rGrabbedUserData, pNewGrabbed->_rGrabbedUserData);

        std::pair<Vector, Vector> velocity = pNewGrabbed->_pGrabbingLink->GetVelocity();
        velocity.first += velocity.second.cross(pBody->GetTransform().trans - pNewGrabbed->_pGrabbingLink->GetTransform().trans);
        pBody->SetVelocity(velocity.first, velocity.second);

        try {
            _AttachBody(pBody);
        }
        catch(...) {
            RAVELOG_ERROR_FORMAT("env=%s, failed to attach body '%s' to body '%s' when grabbing", GetEnv()->GetNameId()%pBody->GetName()%GetName());
            throw;
        }

        BOOST_ASSERT(pBody->GetEnvironmentBodyIndex() > 0);
        _grabbedBodiesByEnvironmentIndex[pBody->GetEnvironmentBodyIndex()] = std::move(pNewGrabbed);
    }
}

KinBody::LinkPtr KinBody::IsGrabbing(const KinBody &body) const
{
    // If we have no grab for a body with a matching env index, we are not grabbing this body
    MapGrabbedByEnvironmentIndex::const_iterator grabIt = _grabbedBodiesByEnvironmentIndex.find(body.GetEnvironmentBodyIndex());
    if (grabIt == _grabbedBodiesByEnvironmentIndex.end()) {
        return nullptr;
    }

    // If we do have a grab with that index, check that the grabbed info we have still points to the same valid kinbody
    const KinBodyConstPtr grabbedBody = grabIt->second->_pGrabbedBody.lock();
    if (!grabbedBody || grabbedBody.get() != &body) {
        return nullptr;
    }

    // If this is definitely a match, return the grabbing link
    return grabIt->second->_pGrabbingLink;
}

int KinBody::CheckGrabbedInfo(const KinBody& body, const KinBody::Link& bodyLinkToGrabWith, const std::set<std::string>& setGrabberLinksToIgnore, const rapidjson::Value& rGrabbedUserData) const
{
    GrabbedInfoCheckResult defaultErrorCode = GICR_BodyNotGrabbed;
    for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        const GrabbedPtr& pgrabbed = grabPair.second;
        KinBodyConstPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();

        // compare grabbing body
        if( !pgrabbedbody || pgrabbedbody.get() != &body ) {
            continue;
        }
        defaultErrorCode = std::max(defaultErrorCode, GICR_GrabbingLinkNotMatch);

        // compare grabbing robot link
        if( pgrabbed->_pGrabbingLink.get() != &bodyLinkToGrabWith ) {
            continue;
        }
        defaultErrorCode = std::max(defaultErrorCode, GICR_IgnoredLinksNotMatch);

        // compare ignored robot links
        bool ignoringLinksMatch = true;
        size_t numIgnoredLinks = 0;  // needed to detect non-existing links in setBodyLinksToIgnore
        for( int linkIndex : pgrabbed->_setGrabberLinkIndicesToIgnore ) {
            const bool isLinkIndexValid = (0 <= linkIndex && linkIndex < (int)_veclinks.size());
            if( isLinkIndexValid ) {
                ++numIgnoredLinks;
                if( setGrabberLinksToIgnore.count(_veclinks[linkIndex]->GetName()) == 0 ) {
                    ignoringLinksMatch = false;
                    break;
                }
            }
            else {
                RAVELOG_WARN_FORMAT("env=%s, grabbed body '%s' of body '%s' has an invalid grabber link index %d to ignore. number of links is %d.", GetEnv()->GetNameId()%GetName()%body.GetName()%linkIndex%_veclinks.size());
            }
        }
        if( !ignoringLinksMatch || numIgnoredLinks != setGrabberLinksToIgnore.size() ) {
            continue;
        }
        defaultErrorCode = std::max(defaultErrorCode, GICR_UserDataNotMatch);

        if( pgrabbed->_rGrabbedUserData == rGrabbedUserData ) {
            return GICR_Identical;
        }
    }
    return defaultErrorCode;
}

void KinBody::GetGrabbed(std::vector<KinBodyPtr>& vbodies) const
{
    vbodies.clear();
    vbodies.reserve(_grabbedBodiesByEnvironmentIndex.size());
    for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        KinBodyPtr pbody = grabPair.second->_pGrabbedBody.lock();
        if (!!pbody && pbody->GetEnvironmentBodyIndex()) {
            vbodies.push_back(pbody);
        }
    }
}

void KinBody::GetGrabbedBodyNames(std::unordered_set<std::string>& bodyNames) const
{
    bodyNames.clear();
    for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        std::string bodyName;
        if (GetEnv()->GetBodyNameFromEnvironmentBodyIndex(grabPair.first, bodyName)) {
            bodyNames.emplace(std::move(bodyName));
        }
    }
}

void KinBody::GetGrabbedInfo(std::vector<KinBody::GrabbedInfoPtr>& vGrabbedInfos) const
{
    vGrabbedInfos.clear();
    vGrabbedInfos.reserve(_grabbedBodiesByEnvironmentIndex.size());
    for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        const GrabbedPtr& pgrabbed = grabPair.second;
        KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        // sometimes bodies can be removed before they are Released, this is ok and can happen during exceptions and stack unwinding
        if( !!pgrabbedbody ) {
            KinBody::GrabbedInfoPtr poutputinfo(new GrabbedInfo());
            vGrabbedInfos.emplace_back(std::move(poutputinfo));
            _GetOneGrabbedInfo(*(vGrabbedInfos.back()), pgrabbed, pgrabbedbody, _veclinks, _name, GetEnv());
        }
    }
}

void KinBody::GetGrabbedInfo(std::vector<GrabbedInfo>& vGrabbedInfos) const
{
    vGrabbedInfos.clear();
    vGrabbedInfos.reserve(_grabbedBodiesByEnvironmentIndex.size());
    for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        const GrabbedPtr& pgrabbed = grabPair.second;
        KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        // sometimes bodies can be removed before they are Released, this is ok and can happen during exceptions and stack unwinding
        if (!!pgrabbedbody) {
            vGrabbedInfos.emplace_back();
            _GetOneGrabbedInfo(vGrabbedInfos.back(), pgrabbed, pgrabbedbody, _veclinks, _name, GetEnv());
        }
    }
}

bool KinBody::GetGrabbedInfo(const std::string& grabbedName, GrabbedInfo& grabbedInfo) const
{
    grabbedInfo.Reset();

    // Look up the target body by name to get its index
    KinBodyPtr pSearchBody = GetEnv()->GetKinBody(grabbedName);
    if (!pSearchBody) {
        // If the body doesn't exist in the env, we can't be grabbing it
        return false;
    }

    // Check to see if we have a grab for that index. If not, we defintely aren't grabbing.
    MapGrabbedByEnvironmentIndex::const_iterator grabIt = _grabbedBodiesByEnvironmentIndex.find(pSearchBody->GetEnvironmentBodyIndex());
    if (grabIt == _grabbedBodiesByEnvironmentIndex.end()) {
        return false;
    }

    // If the body is dead, this grab doesn't count
    const GrabbedPtr& pgrabbed = grabIt->second;
    KinBodyPtr pGrabbedBody = pgrabbed->_pGrabbedBody.lock();
    if (!pGrabbedBody) {
        return false;
    }

    // Sanity check
    OPENRAVE_ASSERT_FORMAT(pGrabbedBody->GetName() == grabbedName, "env=%s, body '%s' thought it grabbed '%s' but actual body name is '%s'", GetEnv()->GetNameId()%GetName()%grabbedName%pGrabbedBody->GetName(), ORE_InvalidArguments);

    // Fill in the output data and return success
    _GetOneGrabbedInfo(grabbedInfo, pgrabbed, pGrabbedBody, _veclinks, _name, GetEnv());
    return true;
}

KinBody::GrabbedInfo::GrabbedInfo(const GrabbedInfo& other)
{
    *this = other;
}

bool KinBody::GrabbedInfo::operator==(const GrabbedInfo& other) const
{
    return _id == other._id
       && _grabbedname == other._grabbedname
       && _robotlinkname == other._robotlinkname
       && _trelative == other._trelative
       && _setIgnoreRobotLinkNames == other._setIgnoreRobotLinkNames
       && _rGrabbedUserData == other._rGrabbedUserData;
}

KinBody::GrabbedInfo& KinBody::GrabbedInfo::operator=(const GrabbedInfo& other)
{
    _id = other._id;
    _grabbedname = other._grabbedname;
    _robotlinkname = other._robotlinkname;
    _trelative = other._trelative;
    _setIgnoreRobotLinkNames = other._setIgnoreRobotLinkNames;
    _rGrabbedUserData = rapidjson::Document(); // reset allocator
    _rGrabbedUserData.CopyFrom(other._rGrabbedUserData, _rGrabbedUserData.GetAllocator());
    return *this;
}

void KinBody::GrabbedInfo::Reset()
{
    _id.clear();
    _grabbedname.clear();
    _robotlinkname.clear();
    _trelative = Transform();
    _setIgnoreRobotLinkNames.clear();
    _rGrabbedUserData.SetNull();
}

void KinBody::GrabbedInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    if( !_id.empty() ) {
        orjson::SetJsonValueByKey(value, "id", _id, allocator);
    }
    orjson::SetJsonValueByKey(value, "grabbedName", _grabbedname, allocator);
    orjson::SetJsonValueByKey(value, "robotLinkName", _robotlinkname, allocator);
    Transform transform = _trelative;
    transform.trans *= fUnitScale;
    orjson::SetJsonValueByKey(value, "transform", transform, allocator);
    if( !_setIgnoreRobotLinkNames.empty() ) {
        orjson::SetJsonValueByKey(value, "ignoreRobotLinkNames", _setIgnoreRobotLinkNames, allocator);
    }
    if( !_rGrabbedUserData.IsNull() ) {
        orjson::SetJsonValueByKey(value, "grabbedUserData", _rGrabbedUserData, allocator);
    }
}

void KinBody::GrabbedInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "id", _id);
    orjson::LoadJsonValueByKey(value, "grabbedName", _grabbedname);
    orjson::LoadJsonValueByKey(value, "robotLinkName", _robotlinkname);
    if (value.HasMember("transform")) {
        orjson::LoadJsonValueByKey(value, "transform", _trelative);
        _trelative.trans *= fUnitScale;
    }
    orjson::LoadJsonValueByKey(value, "ignoreRobotLinkNames", _setIgnoreRobotLinkNames);
    if( value.HasMember("grabbedUserData") ) {
        _rGrabbedUserData = rapidjson::Document(); // to remove the allocator
        orjson::SaveJsonValue(_rGrabbedUserData, value["grabbedUserData"], _rGrabbedUserData.GetAllocator());
    }
    else {
        _rGrabbedUserData.SetNull();
    }
}

void KinBody::GrabbedInfo::serialize(std::ostream& os) const
{
    os << _grabbedname << " ";
    os << _robotlinkname << " ";
    SerializeRound(os, _trelative);
    for( std::set<std::string>::const_iterator it = _setIgnoreRobotLinkNames.begin(); it != _setIgnoreRobotLinkNames.end(); ++it ) {
        os << (*it) << " ";
    }
    if( _rGrabbedUserData.IsNull() ) {
        // using 'void DumpJson(Value, ostream, unsigned int)' to let rapidjson::OStreamWrapper to handle std::ostream
        OpenRAVE::orjson::DumpJson(_rGrabbedUserData, os);
        os << " ";
    }
}

std::string KinBody::GrabbedInfo::GetGrabbedInfoHash() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
    serialize(ss);
    return utils::GetMD5HashString(ss.str());
}

void KinBody::ResetGrabbed(const std::vector<KinBody::GrabbedInfoConstPtr>& vGrabbedInfos)
{
    // First pass: remove all existing grabs that are no longer valid per the new list of grab infos
    // Keep track of which bodies are still attached, so that we can skip re-attaching them later
    std::unordered_set<int> existingAttachedGrabBodyIndices;
    {
        // Scan through the list of new grab infos and flag the indices of all the bodies that are staying attached
        for (const KinBody::GrabbedInfoConstPtr& grabInfo : vGrabbedInfos) {
            // Check to see if the grabbed body exists
            const int grabbedBodyIndex = GetEnv()->GetEnvironmentBodyIndexByName(grabInfo->_grabbedname);
            if (grabbedBodyIndex == 0) {
                RAVELOG_WARN_FORMAT("unable to determine environment index for body '%s', does it exist in the environment?", grabInfo->_grabbedname);
                continue;
            }

            // Check to see if this grab exists in the existing grab list
            MapGrabbedByEnvironmentIndex::iterator existingGrabIt = _grabbedBodiesByEnvironmentIndex.find(grabbedBodyIndex);
            if (existingGrabIt == _grabbedBodiesByEnvironmentIndex.end()) {
                // If it doesn't, this is a new grab - nothing to move.
                continue;
            }

            // Mark that this grabbed body is already attached
            existingAttachedGrabBodyIndices.emplace(grabbedBodyIndex);

            // Release the pointer from our list to indicate that we have processed it
            existingGrabIt->second.reset();
        }

        // At this point, any non-null grab pointers in the old _grabbedBodiesByEnvironmentIndex map are for bodies that are no longer grabbed.
        // Scan through and release these grabs now.
        for (MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
            // If this grab already got migrated to the list of grabs to keep, skip it
            if (!grabPair.second) {
                continue;
            }

            // If the body this grab info referenced is already deleted, nothing to be done
            // Don't count this as mutating the grabbed body state, as the body had already vanished
            KinBodyPtr pGrabbedBody = grabPair.second->_pGrabbedBody.lock();
            if (!pGrabbedBody) {
                continue;
            }

            // If the body is valid, detach it
            _RemoveAttachedBody(*pGrabbedBody);
        }

        // Now that we are done processing our old grabs, reset the set of grabbed bodies.
        // Any bodies that are still grabbed will be re-added in the next pass.
        _grabbedBodiesByEnvironmentIndex.clear();
    }

    // Ensure that we reset the collision checker options when done
    CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
    CollisionOptionsStateSaver colsaver(collisionchecker, 0);

    // Next, for each incoming grab info, either update the existing grab or create a new one if it doesn't exist
    for (const KinBody::GrabbedInfoConstPtr& pGrabbedInfo : vGrabbedInfos) {
        // Assert that this grab info maps to a real body in the environment
        KinBodyPtr pBody = GetEnv()->GetKinBody(pGrabbedInfo->_grabbedname);
        OPENRAVE_ASSERT_FORMAT(!!pBody, "env=%s, body '%s' grabs invalid grab body '%s'", GetEnv()->GetNameId()%GetName()%pGrabbedInfo->_grabbedname, ORE_InvalidArguments);

        // Check that the specified grab link is also real
        KinBody::LinkPtr pGrabbingLink = GetLink(pGrabbedInfo->_robotlinkname);
        if (!pGrabbingLink) {
            std::stringstream ss;
            for (const LinkPtr& pLink : _veclinks) {
                ss << pLink->GetName() << ",";
            }
            throw OPENRAVE_EXCEPTION_FORMAT("env=%s, body '%s' grabs body '%s' with an invalid grabbing link '%s'. Available links are [%s]", GetEnv()->GetNameId()%GetName()%pGrabbedInfo->_grabbedname%pGrabbedInfo->_robotlinkname%ss.str(), ORE_InvalidArguments);
        }

        // Assert that we are not trying to self-grab
        OPENRAVE_ASSERT_FORMAT(pBody.get() != this, "env=%s, body '%s' cannot grab itself", GetEnv()->GetNameId()%pBody->GetName(), ORE_InvalidArguments);

        // If we have a collision checker that is _not_ the default environment collision checker, we need to update it manually
        if (!!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker()) {
            _selfcollisionchecker->InitKinBody(pBody);
        }

        // Update the body's transform in the environment before grabbing it.
        // This has to be done before we create the new Grabbed container, as the Grabbed object will persist the current transform of the object when constructed.
        if (pBody->GetLinks().size() == 0) {
            RAVELOG_WARN_FORMAT("env=%s, cannot set transform of grabbed body '%s' with no links when grabbing by body '%s'", GetEnv()->GetNameId()%pBody->GetName()%GetName());
        }
        Transform tGrabbingLink = pGrabbingLink->GetTransform();
        Transform tBody = tGrabbingLink * pGrabbedInfo->_trelative;
        pBody->SetTransform(tBody);

        // Get the index of the body we're tring to grab
        const int grabbedBodyIndex = GetEnv()->GetEnvironmentBodyIndexByName(pGrabbedInfo->_grabbedname);
        if (grabbedBodyIndex == 0) {
            RAVELOG_WARN_FORMAT("unable to determine environment index for body '%s', does it exist in the environment?", pGrabbedInfo->_grabbedname);
            continue;
        }

        // Check if this maps to an existing grab, in which case we don't need to re-attach the body
        bool isNewlyGrabbedBody = existingAttachedGrabBodyIndices.find(grabbedBodyIndex) == existingAttachedGrabBodyIndices.end();

        // Even if this is an existing grabbed body, re-allocate our Grabbed record for two reasons:
        // - We need to re-save the current state of the grabbed object (e.g. the relative transform)
        // - References to the old Grabbed infos for this body may be held by state savers somewhere outside the body, so we can't mutate them without invalidating those checkpoints
        MapGrabbedByEnvironmentIndex::iterator existingGrabIt = _grabbedBodiesByEnvironmentIndex.emplace(pBody->GetEnvironmentBodyIndex(), new Grabbed(pBody, pGrabbingLink)).first;

        // Update the grab object with the ancillary grab info
        GrabbedPtr& pGrabbed = existingGrabIt->second;
        pGrabbed->_tRelative = pGrabbedInfo->_trelative;
        FOREACHC(itLinkName, pGrabbedInfo->_setIgnoreRobotLinkNames) {
            pGrabbed->_setGrabberLinkIndicesToIgnore.insert(GetLink(*itLinkName)->GetIndex());
        }
        CopyRapidJsonDoc(pGrabbedInfo->_rGrabbedUserData, pGrabbed->_rGrabbedUserData);

        // Multiply the velocity of the grabbing link by the additional transform to get the extended velocity of this new body
        std::pair<Vector, Vector> velocity = pGrabbingLink->GetVelocity();
        velocity.first += velocity.second.cross(tBody.trans - tGrabbingLink.trans);
        pBody->SetVelocity(velocity.first, velocity.second);

        // Only attach the body again if it's actually a new body
        if (isNewlyGrabbedBody) {
            _AttachBody(pBody);
        }
    }

    // Now that we have finished processing all of the grab changes, invoke any register grab callbacks
    _PostprocessChangedParameters(Prop_RobotGrabbed);
}

void KinBody::GetIgnoredLinksOfGrabbed(KinBodyConstPtr body, std::list<KinBody::LinkConstPtr>& ignorelinks) const
{
    ignorelinks.clear();
    for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        const GrabbedPtr& pGrabbed = grabPair.second;
        KinBodyPtr pGrabbedBody = pGrabbed->_pGrabbedBody.lock();
        if( pGrabbedBody == body ) {
            for( int linkIndex : pGrabbed->_setGrabberLinkIndicesToIgnore ) {
                if (0 <= linkIndex && linkIndex < (int)_veclinks.size()) {
                    ignorelinks.push_back(_veclinks[linkIndex]);
                }
                else {
                    RAVELOG_WARN_FORMAT("env=%s, grabbed body '%s' of body '%s' has an invalid grabber link index %d to ignore. number of links is %d.", GetEnv()->GetNameId()%pGrabbedBody->GetName()%GetName()%linkIndex%_veclinks.size());
                }
                ignorelinks.push_back(_veclinks.at(linkIndex));
            }
            return;
        }
    }
    RAVELOG_WARN_FORMAT("env=%s, body '%s' is not currently grabbing '%s' so cannot get ignoreLinks", GetEnv()->GetNameId()%GetName()%body->GetName());
}

void KinBody::_UpdateGrabbedBodies()
{
    std::pair<Vector, Vector> velocity;
    Transform tGrabbedBody; // cache
    for (MapGrabbedByEnvironmentIndex::iterator grabIt = _grabbedBodiesByEnvironmentIndex.begin(); grabIt != _grabbedBodiesByEnvironmentIndex.end(); /* nop */) {
        GrabbedPtr pgrabbed = grabIt->second;
        KinBodyPtr pGrabbedBody = pgrabbed->_pGrabbedBody.lock();
        if( !!pGrabbedBody ) {
            const Transform& tGrabbingLink = pgrabbed->_pGrabbingLink->GetTransform();
            tGrabbedBody = tGrabbingLink * pgrabbed->_tRelative;
            pGrabbedBody->SetTransform(tGrabbedBody);
            // set the correct velocity
            pgrabbed->_pGrabbingLink->GetVelocity(velocity.first, velocity.second);
            velocity.first += velocity.second.cross(tGrabbedBody.trans - tGrabbingLink.trans);
            pGrabbedBody->SetVelocity(velocity.first, velocity.second);
            ++grabIt;
        }
        else {
            RAVELOG_DEBUG_FORMAT("env=%s, erasing invalid grabbed body from grabbing body '%s'", GetEnv()->GetNameId()%GetName());
            grabIt = _RemoveGrabbedBody(grabIt);
        }
    }
}

KinBody::MapGrabbedByEnvironmentIndex::iterator KinBody::_RemoveGrabbedBody(MapGrabbedByEnvironmentIndex::iterator itGrabbed)
{
    // Cache the actual grab body here so that we can post-process
    KinBodyConstPtr pGrabbedBody = itGrabbed->second->_pGrabbedBody.lock();

    // Remove the body from our set of grabs
    itGrabbed = _grabbedBodiesByEnvironmentIndex.erase(itGrabbed);

    // If the grabbed body wasn't real, skip updating the link collision states
    if (!pGrabbedBody) {
        return itGrabbed;
    }

    // Scan through the other grabs we have and update the set of non-colliding links in those bodies to not include the links of the body we just removed
    for (const MapGrabbedByEnvironmentIndex::value_type& otherGrabPair : _grabbedBodiesByEnvironmentIndex) {
        const GrabbedPtr& pOtherGrabbed = otherGrabPair.second;
        _RemoveLinkFromListNonCollidingLinksWhenGrabbed<KinBodyConstPtr>(pOtherGrabbed->_listNonCollidingLinksWhenGrabbed, pGrabbedBody);
    }

    return itGrabbed;
}

} // end namespace OpenRAVE
