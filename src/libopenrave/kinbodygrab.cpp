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

Grabbed::Grabbed(KinBodyPtr pGrabbedBody, KinBody::LinkPtr pGrabbingLink)
{
    _pGrabbedBody = pGrabbedBody;
    _pGrabbingLink = pGrabbingLink;
    _pGrabbingLink->GetRigidlyAttachedLinks(_vAttachedToGrabbingLink);
    _listNonCollidingIsValid = false;
    // Need to save link velocities of the grabber since will be used for computing link velocities of the grabbed bodies.
    int defaultGrabbedSaveOptions = KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_JointLimits|KinBody::Save_GrabbedBodies;
    int defaultGrabberSaveOptions = KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_JointLimits|KinBody::Save_GrabbedBodies|KinBody::Save_LinkVelocities;
    if( pGrabbedBody->IsRobot() ) {
        RobotBasePtr pGrabbedRobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pGrabbedBody);
        _pGrabbedSaver.reset(new RobotBase::RobotStateSaver(pGrabbedRobot, defaultGrabbedSaveOptions|KinBody::Save_ConnectedBodies));
    }
    else {
        _pGrabbedSaver.reset(new KinBody::KinBodyStateSaver(pGrabbedBody, defaultGrabbedSaveOptions));
    }
    _pGrabbedSaver->SetRestoreOnDestructor(false); // This is very important!

    KinBodyPtr pGrabber = RaveInterfaceCast<KinBody>(_pGrabbingLink->GetParent());
    if( pGrabber->IsRobot() ) {
        RobotBasePtr pRobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pGrabber);
        _pGrabberSaver.reset(new RobotBase::RobotStateSaver(pRobot, defaultGrabberSaveOptions|KinBody::Save_ConnectedBodies));
    }
    else {
        _pGrabberSaver.reset(new KinBody::KinBodyStateSaver(pGrabber, defaultGrabberSaveOptions));
    }
    _pGrabberSaver->SetRestoreOnDestructor(false); // This is very important!
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
    int defaultSaveOptions = KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_LinkVelocities|KinBody::Save_JointLimits|KinBody::Save_GrabbedBodies;
    if( pGrabbedBody->IsRobot() ) {
        RobotBasePtr pGrabbedRobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pGrabbedBody);
        pCurrentGrabbedSaver.reset(new RobotBase::RobotStateSaver(pGrabbedRobot, defaultSaveOptions|KinBody::Save_ConnectedBodies));
    }
    else {
        pCurrentGrabbedSaver.reset(new KinBody::KinBodyStateSaver(pGrabbedBody, defaultSaveOptions));
    }
    if( pGrabber->IsRobot() ) {
        RobotBasePtr pRobot = OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pGrabber);
        pCurrentGrabberSaver.reset(new RobotBase::RobotStateSaver(pRobot, defaultSaveOptions|KinBody::Save_ConnectedBodies));
    }
    else {
        pCurrentGrabberSaver.reset(new KinBody::KinBodyStateSaver(pGrabber, defaultSaveOptions));
    }
    // RAVELOG_INFO_FORMAT("env=%s, computing _listNonCollidingLinksWhenGrabbed for body %s", pGrabbedBody->GetEnv()->GetNameId()%pGrabbedBody->GetName());

    // Now that the current state is saved, set the state to the original (when Grab was called)
    pGrabber->ReleaseAllGrabbed();
    _pGrabbedSaver->Restore();
    _pGrabberSaver->Restore();

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
    CollisionOptionsStateSaver colOptionsSaver(pchecker, /*newcollisionoptions*/ 0); // reset collision options before proceeding
    {
        KinBody::KinBodyStateSaver grabbedEnableSaver(pGrabbedBody, KinBody::Save_LinkEnable);
        pGrabbedBody->Enable(true);
        KinBody::KinBodyStateSaver grabberEnableSaver(pGrabber, KinBody::Save_LinkEnable);
        pGrabber->Enable(true);

        // Check grabbed body vs grabber links
        FOREACHC(itGrabberLink, pGrabber->GetLinks()) {
            bool isNonColliding = false;
            if( std::find(_vAttachedToGrabbingLink.begin(), _vAttachedToGrabbingLink.end(), *itGrabberLink) != _vAttachedToGrabbingLink.end() ) {
                // This link (*itGrabberLink) is rigidly attached to _pGrabbingLink. Therefore, isNonColliding = false, meaning that we will not check collision between this link and the grabbed body later on.
            }
            else {
                // This link (*itGrabberLink) is *not* rigidly attached to _pGrabbingLink.
                if( _setGrabberLinkIndicesToIgnore.find((*itGrabberLink)->GetIndex()) == _setGrabberLinkIndicesToIgnore.end() ) {
                    // Not ignoring collisions between this link and the grabber body
                    if( !pchecker->CheckCollision(KinBody::LinkConstPtr(*itGrabberLink), pGrabbedBody) ) {
                        isNonColliding = true;
                    }
                }
            }
            if( isNonColliding ) {
                _listNonCollidingLinksWhenGrabbed.push_back(*itGrabberLink);
            }
        }

        // Check grabbed body vs other existing grabbed bodies
        // int iOtherGrabbed = -1;
        for( const GrabbedPtr& pOtherGrabbed : pGrabber->_vGrabbedBodies ) {
            // ++iOtherGrabbed;
            KinBodyPtr pOtherGrabbedBody = pOtherGrabbed->_pGrabbedBody.lock();
            if( !pOtherGrabbedBody ) {
                RAVELOG_WARN_FORMAT("env=%s, other grabbed body on %s has already been released. So ignoring it.", penv->GetNameId()%pGrabber->GetName());
                continue;
            }
            if( pOtherGrabbedBody->GetLinks().empty() ) {
                RAVELOG_WARN_FORMAT("env=%s, other grabbed body %s on %s has no links. Perhaps not initialized. So ignoring it.", penv->GetNameId()%pOtherGrabbedBody->GetName()%pGrabber->GetName());
                continue;
            }
            // RAVELOG_INFO_FORMAT("env=%s, current grabbed='%s'; other grabbed(%d/%d)='%s'", penv->GetNameId()%pGrabbedBody->GetName()%iOtherGrabbed%numOtherGrabbed%pOtherGrabbedBody->GetName());


            if( pOtherGrabbedBody != pGrabbedBody ) {
                // sufficient to check one direction, whether grabbing link of body 2 is in links attached rigidly to grabbing link of body 1. Result is same for both directions.
                const bool bTwoGrabbedBodiesHaveStaticRelativePose = std::find(_vAttachedToGrabbingLink.begin(), _vAttachedToGrabbingLink.end(), pOtherGrabbed->_pGrabbingLink) != _vAttachedToGrabbingLink.end();
                // if two grabbed bodies have static (constant) relative pose with respect to each other, do not need to check collision between them for the rest of time.
                if (!bTwoGrabbedBodiesHaveStaticRelativePose) {
                    KinBody::KinBodyStateSaver otherGrabbedEnableSaver(pOtherGrabbedBody, KinBody::Save_LinkEnable);
                    pOtherGrabbedBody->Enable(true);
                    for (const KinBody::LinkPtr& pOtherGrabbedLink : pOtherGrabbedBody->GetLinks()) {
                        const bool isNonColliding = !pchecker->CheckCollision(KinBody::LinkConstPtr(pOtherGrabbedLink), pGrabbedBody);
                        if( isNonColliding ) {
                            _listNonCollidingLinksWhenGrabbed.push_back(pOtherGrabbedLink);
                        }
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
    std::vector<GrabbedPtr>::iterator itPreviouslyGrabbed;
    FOREACHC(itGrabbed, _vGrabbedBodies) {
        GrabbedPtr pGrabbed = *itGrabbed;
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
        else {
            std::set<KinBodyPtr> setAttached;
            pGrabbedBody->GetAttached(setAttached);
            std::stringstream ss;
            if( setAttached.size() > 1 ) {
                FOREACH(itbody, setAttached) {
                    ss << (*itbody)->GetName() << ",";
                }
            }
            RAVELOG_WARN_FORMAT("env=%s, body '%s' trying to grab body '%s' with %d attached bodies [%s]", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName()%setAttached.size()%ss.str());
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
        _vGrabbedBodies.erase(itPreviouslyGrabbed);
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
    _vGrabbedBodies.push_back(pGrabbed);

    try {
        // if an exception happens in _AttachBody, have to remove from _vGrabbedBodies
        _AttachBody(pGrabbedBody);
    }
    catch(...) {
        RAVELOG_ERROR_FORMAT("env=%s, failed to attach %s to %s when grabbing", GetEnv()->GetNameId()%pGrabbedBody->GetName()%GetName());
        BOOST_ASSERT(_vGrabbedBodies.back() == pGrabbed);
        // do not call _selfcollisionchecker->RemoveKinBody since the same object might be re-attached later on and we should preserve the structures.
        _vGrabbedBodies.pop_back();
        throw;
    }

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
    FOREACH(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = *itgrabbed;
        KinBodyConstPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        if( !!pgrabbedbody ) {
            bool bpointermatch = pgrabbedbody.get() == &body;
            bool bnamematch = pgrabbedbody->GetName() == body.GetName();
            if( bpointermatch != bnamematch ) {
                RAVELOG_WARN_FORMAT("env=%d, body %s has grabbed body %s (%d), but it does not match with %s (%d) ", GetEnv()->GetId()%GetName()%pgrabbedbody->GetName()%pgrabbedbody->GetEnvironmentBodyIndex()%body.GetName()%body.GetEnvironmentBodyIndex());
            }
            if( bpointermatch ) {
                _vGrabbedBodies.erase(itgrabbed);
                _RemoveAttachedBody(body);
                _PostprocessChangedParameters(Prop_RobotGrabbed);
                return;
            }
        }
    }

    if( IS_DEBUGLEVEL(Level_Debug) ) {
        std::stringstream ss;
        for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
            KinBodyConstPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                ss << pgrabbedbody->GetName() << ", ";
            }
        }

        RAVELOG_DEBUG_FORMAT("env=%d, body %s is not grabbing body %s (%d), but grabbing bodies [%s]", GetEnv()->GetId()%GetName()%body.GetName()%body.GetEnvironmentBodyIndex()%ss.str());
    }
}

void KinBody::ReleaseAllGrabbed()
{
    if( _vGrabbedBodies.size() > 0 ) {
        for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
            KinBodyPtr pbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pbody ) {
                _RemoveAttachedBody(*pbody);
            }
        }
        _vGrabbedBodies.clear();
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
}

void KinBody::ReleaseAllGrabbedWithLink(const KinBody::Link& bodyLinkToReleaseWith)
{
    OPENRAVE_ASSERT_FORMAT(bodyLinkToReleaseWith.GetParent().get() == this, "body %s invalid grab arguments",GetName(), ORE_InvalidArguments);

    if( _vGrabbedBodies.size() > 0 ) {
        bool bReleased = false;
        int nCheckIndex = (int)_vGrabbedBodies.size()-1;
        while(nCheckIndex >= 0) {
            GrabbedPtr pgrabbed = _vGrabbedBodies.at(nCheckIndex);
            if( pgrabbed->_pGrabbingLink.get() == &bodyLinkToReleaseWith ) {
                KinBodyPtr pbody = pgrabbed->_pGrabbedBody.lock();
                if( !!pbody ) {
                    _RemoveAttachedBody(*pbody);
                }
                _vGrabbedBodies.erase(_vGrabbedBodies.begin()+nCheckIndex);
                bReleased = true;
            }
            --nCheckIndex;
        }
        if( bReleased ) {
            _PostprocessChangedParameters(Prop_RobotGrabbed);
        }
    }
}

void KinBody::RegrabAll()
{
    CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
    CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options

    size_t numGrabbed = _vGrabbedBodies.size();
    // Remove bodies from _listAttachedBodies first and then will add them back later. Maybe this is for triggering
    // postprocessing with Prop_BodyAttached?
    FOREACH(itGrabbed, _vGrabbedBodies) {
        GrabbedPtr pGrabbed = boost::dynamic_pointer_cast<Grabbed>(*itGrabbed);
        KinBodyPtr pBody = pGrabbed->_pGrabbedBody.lock();
        if( !!pBody ) {
            _RemoveAttachedBody(*pBody);
        }
    }

    std::vector<GrabbedPtr> vOriginalGrabbed;
    vOriginalGrabbed.swap(_vGrabbedBodies);

    _vGrabbedBodies.reserve(numGrabbed);
    // Regrab all the objects in the same order.
    for( size_t iGrabbed = 0; iGrabbed < numGrabbed; ++iGrabbed ) {
        GrabbedPtr pGrabbed = vOriginalGrabbed[iGrabbed];
        KinBodyPtr pBody = pGrabbed->_pGrabbedBody.lock();
        if( !pBody ) {
            RAVELOG_WARN_FORMAT("env=%s, grabbed body %d/%d does not exist any more", GetEnv()->GetNameId()%iGrabbed%numGrabbed);
            continue;
        }

        GrabbedPtr pNewGrabbed(new Grabbed(pBody, pGrabbed->_pGrabbingLink));
        pNewGrabbed->_tRelative = pGrabbed->_tRelative;
        pNewGrabbed->_setGrabberLinkIndicesToIgnore.swap(pGrabbed->_setGrabberLinkIndicesToIgnore);
        CopyRapidJsonDoc(pGrabbed->_rGrabbedUserData, pNewGrabbed->_rGrabbedUserData);

        std::pair<Vector, Vector> velocity = pNewGrabbed->_pGrabbingLink->GetVelocity();
        velocity.first += velocity.second.cross(pBody->GetTransform().trans - pNewGrabbed->_pGrabbingLink->GetTransform().trans);
        pBody->SetVelocity(velocity.first, velocity.second);

        _vGrabbedBodies.push_back(pNewGrabbed);
        try {
            _AttachBody(pBody);
        }
        catch(...) {
            RAVELOG_ERROR_FORMAT("env=%s, failed to attach body '%s' to body '%s' when grabbing", GetEnv()->GetNameId()%pBody->GetName()%GetName());
            BOOST_ASSERT(_vGrabbedBodies.back() == pGrabbed);
            _vGrabbedBodies.pop_back();
            throw;
        }
    }
}

KinBody::LinkPtr KinBody::IsGrabbing(const KinBody &body) const
{
    for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
        KinBodyConstPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        if( !!pgrabbedbody && pgrabbedbody.get() == &body ) {
            return pgrabbed->_pGrabbingLink;
        }
    }
    return LinkPtr();
}

int KinBody::CheckGrabbedInfo(const KinBody& body, const KinBody::Link& bodyLinkToGrabWith) const
{
    GrabbedInfoCheckResult defaultErrorCode = GICR_BodyNotGrabbed;
    for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
        KinBodyConstPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();

        // compare grabbing body
        if( !pgrabbedbody || pgrabbedbody.get() != &body ) {
            continue;
        }
        defaultErrorCode = GICR_GrabbingLinkNotMatch;

        // compare grabbing robot link
        if( pgrabbed->_pGrabbingLink.get() != &bodyLinkToGrabWith ) {
            continue;
        }
        return GICR_Identical;
    }
    return defaultErrorCode;
}

int KinBody::CheckGrabbedInfo(const KinBody& body, const KinBody::Link& bodyLinkToGrabWith, const std::set<int>& setGrabberLinksToIgnore, const rapidjson::Value& rGrabbedUserData) const
{
    GrabbedInfoCheckResult defaultErrorCode = GICR_BodyNotGrabbed;
    for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
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
        for( const LinkPtr& link : _veclinks ) {
            const bool isLinkIgnored = std::find(pgrabbed->_setGrabberLinkIndicesToIgnore.begin(), pgrabbed->_setGrabberLinkIndicesToIgnore.end(), link->GetIndex()) != pgrabbed->_setGrabberLinkIndicesToIgnore.end();
            if( isLinkIgnored ) {
                ++numIgnoredLinks;
            }
            if( isLinkIgnored != (setGrabberLinksToIgnore.count(link->GetIndex()) > 0) ) {
                ignoringLinksMatch = false;
                break;
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

int KinBody::CheckGrabbedInfo(const KinBody& body, const KinBody::Link& bodyLinkToGrabWith, const std::set<std::string>& setGrabberLinksToIgnore, const rapidjson::Value& rGrabbedUserData) const
{
    GrabbedInfoCheckResult defaultErrorCode = GICR_BodyNotGrabbed;
    for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
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
        for( const LinkPtr& link : _veclinks ) {
            const bool isLinkIgnored = std::find(pgrabbed->_setGrabberLinkIndicesToIgnore.begin(), pgrabbed->_setGrabberLinkIndicesToIgnore.end(), link->GetIndex()) != pgrabbed->_setGrabberLinkIndicesToIgnore.end();
            if( isLinkIgnored ) {
                ++numIgnoredLinks;
            }
            if( isLinkIgnored != (setGrabberLinksToIgnore.count(link->GetName()) > 0) ) {
                ignoringLinksMatch = false;
                break;
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
    vbodies.reserve(_vGrabbedBodies.size());
    for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
        KinBodyPtr pbody = pgrabbed->_pGrabbedBody.lock();
        if( !!pbody && pbody->GetEnvironmentBodyIndex() ) {
            vbodies.push_back(pbody);
        }
    }
}

KinBodyPtr KinBody::GetGrabbedBody(int iGrabbed) const
{
    GrabbedConstPtr pgrabbed = _vGrabbedBodies.at(iGrabbed);
    KinBodyPtr pbody = pgrabbed->_pGrabbedBody.lock();
    if( !!pbody && pbody->GetEnvironmentBodyIndex() ) {
        return pbody;
    }

    return KinBodyPtr(); // whatever is grabbed is not valid.
}

void KinBody::GetGrabbedInfo(std::vector<KinBody::GrabbedInfoPtr>& vGrabbedInfos) const
{
    vGrabbedInfos.reserve(_vGrabbedBodies.size());
    vGrabbedInfos.clear();
    for(size_t i = 0; i < _vGrabbedBodies.size(); ++i) {
        GrabbedConstPtr pgrabbed = _vGrabbedBodies[i];
        KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        // sometimes bodies can be removed before they are Released, this is ok and can happen during exceptions and stack unwinding
        if( !!pgrabbedbody ) {
            KinBody::GrabbedInfoPtr poutputinfo(new GrabbedInfo());
            poutputinfo->_grabbedname = pgrabbedbody->GetName();
            poutputinfo->_robotlinkname = pgrabbed->_pGrabbingLink->GetName();
            poutputinfo->_trelative = pgrabbed->_tRelative;
            poutputinfo->_setIgnoreRobotLinkNames.clear();
            CopyRapidJsonDoc(pgrabbed->_rGrabbedUserData, poutputinfo->_rGrabbedUserData);

            FOREACHC(itlink, _veclinks) {
                if( find(pgrabbed->_setGrabberLinkIndicesToIgnore.begin(), pgrabbed->_setGrabberLinkIndicesToIgnore.end(), (*itlink)->GetIndex()) != pgrabbed->_setGrabberLinkIndicesToIgnore.end() ) {
                    poutputinfo->_setIgnoreRobotLinkNames.insert((*itlink)->GetName());
                }
            }
            vGrabbedInfos.push_back(poutputinfo);
        }
    }
}

void KinBody::GetGrabbedInfo(std::vector<GrabbedInfo>& vGrabbedInfos) const
{
    vGrabbedInfos.resize(_vGrabbedBodies.size());
    for(size_t igrabbed = 0; igrabbed < _vGrabbedBodies.size(); ++igrabbed) {
        vGrabbedInfos[igrabbed].Reset(); /// have to reset everything

        GrabbedConstPtr pgrabbed = _vGrabbedBodies[igrabbed];
        KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        // sometimes bodies can be removed before they are Released, this is ok and can happen during exceptions and stack unwinding
        if( !!pgrabbedbody ) {
            KinBody::GrabbedInfo& outputinfo = vGrabbedInfos[igrabbed];
            outputinfo._grabbedname = pgrabbedbody->GetName();
            outputinfo._robotlinkname = pgrabbed->_pGrabbingLink->GetName();
            outputinfo._trelative = pgrabbed->_tRelative;
            outputinfo._setIgnoreRobotLinkNames.clear();
            CopyRapidJsonDoc(pgrabbed->_rGrabbedUserData, outputinfo._rGrabbedUserData);

            FOREACHC(itlink, _veclinks) {
                if( find(pgrabbed->_setGrabberLinkIndicesToIgnore.begin(), pgrabbed->_setGrabberLinkIndicesToIgnore.end(), (*itlink)->GetIndex()) != pgrabbed->_setGrabberLinkIndicesToIgnore.end() ) {
                    outputinfo._setIgnoreRobotLinkNames.insert((*itlink)->GetName());
                }
            }
        }
    }
}

bool KinBody::GetGrabbedInfo(const std::string& grabbedname, GrabbedInfo& grabbedInfo) const
{
    grabbedInfo.Reset();
    for(size_t igrabbed = 0; igrabbed < _vGrabbedBodies.size(); ++igrabbed) {
        GrabbedConstPtr pgrabbed = _vGrabbedBodies[igrabbed];
        if( !!pgrabbed ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody && pgrabbedbody->GetName() == grabbedname ) {
                grabbedInfo._grabbedname = pgrabbedbody->GetName();
                grabbedInfo._robotlinkname = pgrabbed->_pGrabbingLink->GetName();
                grabbedInfo._trelative = pgrabbed->_tRelative;
                grabbedInfo._setIgnoreRobotLinkNames.clear();
                CopyRapidJsonDoc(pgrabbed->_rGrabbedUserData, grabbedInfo._rGrabbedUserData);

                FOREACHC(itlink, _veclinks) {
                    if( find(pgrabbed->_setGrabberLinkIndicesToIgnore.begin(), pgrabbed->_setGrabberLinkIndicesToIgnore.end(), (*itlink)->GetIndex()) != pgrabbed->_setGrabberLinkIndicesToIgnore.end() ) {
                        grabbedInfo._setIgnoreRobotLinkNames.insert((*itlink)->GetName());
                    }
                }
                return true;
            }
        }
    }
    return false;
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
    ReleaseAllGrabbed();
    if( vGrabbedInfos.size() > 0 ) {
        CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        FOREACHC(itGrabbedInfo, vGrabbedInfos) {
            GrabbedInfoConstPtr pGrabbedInfo = *itGrabbedInfo;
            KinBodyPtr pBody = GetEnv()->GetKinBody(pGrabbedInfo->_grabbedname);
            OPENRAVE_ASSERT_FORMAT(!!pBody, "env=%s, body '%s' grabs invalid grab body '%s'",GetEnv()->GetNameId()%GetName()%pGrabbedInfo->_grabbedname, ORE_InvalidArguments);
            KinBody::LinkPtr pGrabbingLink = GetLink(pGrabbedInfo->_robotlinkname);
            if( !pGrabbingLink ) {
                std::stringstream ss;
                for( const LinkPtr& pLink : _veclinks ) {
                    ss << pLink->GetName() << ",";
                }
                throw OPENRAVE_EXCEPTION_FORMAT("env=%s, body '%s' grabs body '%s' with an invalid grabbing link '%s'. Available links are [%s]",GetEnv()->GetNameId()%GetName()%pGrabbedInfo->_grabbedname%pGrabbedInfo->_robotlinkname%ss.str(), ORE_InvalidArguments);
            }
            OPENRAVE_ASSERT_FORMAT(pBody.get() != this, "env=%s, body '%s' cannot grab itself", GetEnv()->GetNameId()%pBody->GetName(), ORE_InvalidArguments);
            if( IsGrabbing(*pBody) ) {
                RAVELOG_VERBOSE_FORMAT("env=%s, body '%s' already grabs body '%s'", GetEnv()->GetNameId()%GetName()%pGrabbedInfo->_grabbedname);
                continue;
            }

            if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
                // collision checking will not be automatically updated with environment calls, so need to do this manually
                _selfcollisionchecker->InitKinBody(pBody);
            }

            if( pBody->GetLinks().size() == 0 ) {
                RAVELOG_WARN_FORMAT("env=%s, cannot set transform of grabbed body '%s' with no links when grabbing by body '%s'", GetEnv()->GetNameId()%pBody->GetName()%GetName());
            }
            Transform tGrabbingLink = pGrabbingLink->GetTransform();
            Transform tBody = tGrabbingLink * pGrabbedInfo->_trelative;
            pBody->SetTransform(tBody); // need to set the correct transform to pBody before creation of pGrabbed

            GrabbedPtr pGrabbed(new Grabbed(pBody, pGrabbingLink));
            pGrabbed->_tRelative = pGrabbedInfo->_trelative;
            FOREACHC(itLinkName, pGrabbedInfo->_setIgnoreRobotLinkNames) {
                pGrabbed->_setGrabberLinkIndicesToIgnore.insert(GetLink(*itLinkName)->GetIndex());
            }
            CopyRapidJsonDoc(pGrabbedInfo->_rGrabbedUserData, pGrabbed->_rGrabbedUserData);

            std::pair<Vector, Vector> velocity = pGrabbingLink->GetVelocity();
            velocity.first += velocity.second.cross(tBody.trans - tGrabbingLink.trans);
            pBody->SetVelocity(velocity.first, velocity.second);

            _vGrabbedBodies.push_back(pGrabbed);
            _AttachBody(pBody);
        } // end FOREACHC

        _PostprocessChangedParameters(Prop_RobotGrabbed);
    } // end if vGrabbedInfos.size() > 0
}

void KinBody::GetIgnoredLinksOfGrabbed(KinBodyConstPtr body, std::list<KinBody::LinkConstPtr>& ignorelinks) const
{
    ignorelinks.clear();
    for (const GrabbedPtr& pGrabbed : _vGrabbedBodies) {
        KinBodyPtr pGrabbedBody = pGrabbed->_pGrabbedBody.lock();
        if( pGrabbedBody == body ) {
            FOREACHC(itGrabberLink, _veclinks) {
                if( pGrabbed->_setGrabberLinkIndicesToIgnore.find((*itGrabberLink)->GetIndex()) != pGrabbed->_setGrabberLinkIndicesToIgnore.end() ) {
                    ignorelinks.push_back(*itGrabberLink);
                }
            }
            return;
        }
    }
    RAVELOG_WARN_FORMAT("env=%s, body '%s' is not currently grabbing '%s' so cannot get ignoreLinks", GetEnv()->GetNameId()%GetName()%body->GetName());
}

void KinBody::_UpdateGrabbedBodies()
{
    std::vector<GrabbedPtr>::iterator itgrabbed = _vGrabbedBodies.begin();
    std::pair<Vector, Vector> velocity;
    Transform tGrabbedBody; // cache
    while(itgrabbed != _vGrabbedBodies.end() ) {
        GrabbedPtr pgrabbed = *itgrabbed;
        KinBodyPtr pGrabbedBody = pgrabbed->_pGrabbedBody.lock();
        if( !!pGrabbedBody ) {
            const Transform& tGrabbingLink = pgrabbed->_pGrabbingLink->GetTransform();
            tGrabbedBody = tGrabbingLink * pgrabbed->_tRelative;
            pGrabbedBody->SetTransform(tGrabbedBody);
            // set the correct velocity
            pgrabbed->_pGrabbingLink->GetVelocity(velocity.first, velocity.second);
            velocity.first += velocity.second.cross(tGrabbedBody.trans - tGrabbingLink.trans);
            pGrabbedBody->SetVelocity(velocity.first, velocity.second);
            ++itgrabbed;
        }
        else {
            RAVELOG_DEBUG_FORMAT("env=%s, erasing invalid grabbed body from grabbing body '%s'", GetEnv()->GetNameId()%GetName());
            itgrabbed = _vGrabbedBodies.erase(itgrabbed);
        }
    }
}

} // end namespace OpenRAVE
