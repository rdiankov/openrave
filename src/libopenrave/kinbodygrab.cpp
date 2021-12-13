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

bool KinBody::Grab(KinBodyPtr pGrabbedBody, LinkPtr pGrabbingLink)
{
    OPENRAVE_ASSERT_FORMAT(!!pGrabbedBody, "invalid body to grab by %s",GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!pGrabbingLink && pGrabbingLink->GetParent().get() == this, "body %s grabbing link needs to be part of body",GetName(),ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pGrabbedBody.get() != this,"body %s cannot grab itself",GetName(), ORE_InvalidArguments);
    //uint64_t starttime0 = utils::GetMicroTime();

    // if grabbing, check if the transforms are different. If they are, then update the transform
    GrabbedPtr pPreviousGrabbed;
    std::vector<GrabbedPtr>::iterator itPreviousGrabbed;
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        if( pgrabbed->_pGrabbedBody.lock() == pGrabbedBody ) {
            pPreviousGrabbed = pgrabbed;
            itPreviousGrabbed = itgrabbed;
            break;
        }
    }

    // double check since collision checkers might not support this case
    if( pGrabbedBody->HasAttached() ) {
        if( !!pPreviousGrabbed ) {
            RAVELOG_INFO_FORMAT("env=%s, body '%s' is already grabbing body '%s'", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName());
        }
        else {
            std::set<KinBodyPtr> setAttached;
            pGrabbedBody->GetAttached(setAttached);
            std::stringstream ss;
            if( setAttached.size() > 1 ) {
                FOREACH(itbody, setAttached) {
                    ss << (*itbody)->GetName() << ", ";
                }
            }
            RAVELOG_WARN_FORMAT("env=%s, body '%s' trying to grab body '%s' with %d attached bodies [%s]", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName()%setAttached.size()%ss.str());
        }
    }

    Transform tGrabbingLink = pGrabbingLink->GetTransform();
    Transform tGrabbedBody = pGrabbedBody->GetTransform();
    // new body velocity is measured from body link
    std::pair<Vector, Vector> velocity = pGrabbingLink->GetVelocity();
    velocity.first += velocity.second.cross(tGrabbedBody.trans - tGrabbingLink.trans);
    if( !!pPreviousGrabbed ) {
        dReal disterror = TransformDistance2(tGrabbingLink*pPreviousGrabbed->_tRelative, tGrabbedBody);
        if( pPreviousGrabbed->_pGrabbingLink == pGrabbingLink && disterror <= g_fEpsilonLinear ) {
            // links and transforms are the same, so no worries
            return true;
        }
        RAVELOG_DEBUG_FORMAT("env=%s, body '%s' is already grabbing body '%s' but grabbed body transform differs. disterror=%.15e", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName()%disterror);
        _RemoveAttachedBody(*pGrabbedBody);
        _vGrabbedBodies.erase(itPreviousGrabbed); // need to remove it from _vGrabbedBodies since we'll be adding a new GrabbedPtr to _vGrabbedBodies
    }

    GrabbedPtr pGrabbed(new Grabbed(pGrabbedBody, pGrabbingLink));
    pGrabbed->_tRelative = tGrabbingLink.inverse() * tGrabbedBody;

    std::set<int> setGrabberLinksToIgnore;
    std::vector<KinBody::LinkPtr> vAttachedToGrabbingLink;
    pGrabbingLink->GetRigidlyAttachedLinks(vAttachedToGrabbingLink);
    FOREACHC(itAttachedLink, vAttachedToGrabbingLink) {
        setGrabberLinksToIgnore.insert((*itAttachedLink)->GetIndex());
    }
    pGrabbed->_setGrabberLinksToIgnore = setGrabberLinksToIgnore;

    if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
        _selfcollisionchecker->InitKinBody(pGrabbedBody);
    }

    pGrabbedBody->SetVelocity(velocity.first, velocity.second);
    _vGrabbedBodies.push_back(pGrabbed);

    try {
        _AttachBody(pGrabbedBody);
    }
    catch(...) {
        RAVELOG_ERROR_FORMAT("env=%s, failed to attach body '%s' to body '%s' when grabbing", GetEnv()->GetNameId()%pGrabbedBody->GetName()%GetName());
        BOOST_ASSERT(_vGrabbedBodies.back() == pGrabbed);
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

#if 0
    GrabbedPtr pgrabbed(new Grabbed(pbody,plink));
    pgrabbed->_troot = t.inverse() * tbody;
    //uint64_t starttime1 = utils::GetMicroTime();
    // always ignore links that are statically attached to plink (ie assume they are always colliding with the body)

    std::vector<boost::shared_ptr<Link> > vattachedlinks;
    plink->GetRigidlyAttachedLinks(vattachedlinks);
    std::set<int> setBodyLinksToIgnore;
    FOREACHC(itlink, vattachedlinks) {
        setBodyLinksToIgnore.insert((*itlink)->GetIndex());
    }
    if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
        // collision checking will not be automatically updated with environment calls, so need to do this manually
        //try {
        _selfcollisionchecker->InitKinBody(pbody);
//        }
//        catch (const std::exception& ex) {
//            RAVELOG_ERROR_FORMAT("env=%d, failed in _selfcollisionchecker->InitKinBody for body %s: %s", GetEnv()->GetId()%pbody->GetName()%ex.what());
//            throw;
//        }
    }
    //    try {
    pgrabbed->ProcessCollidingLinks(setBodyLinksToIgnore);
//    }
//    catch(const std::exception& ex) {
//        RAVELOG_ERROR_FORMAT("env=%d, failed in ProcessCollidingLinks for body %s: %s", GetEnv()->GetId()%pbody->GetName()%ex.what());
//        throw;
//    }

    pbody->SetVelocity(velocity.first, velocity.second);
    _vGrabbedBodies.push_back(pgrabbed);
    //uint64_t starttime2 = utils::GetMicroTime();
    try {
        // if an exception happens in _AttachBody, have to remove from _vGrabbedBodies
        _AttachBody(pbody);
    }
    catch(...) {
        RAVELOG_ERROR_FORMAT("env=%d, failed in attach body", GetEnv()->GetId());
        BOOST_ASSERT(_vGrabbedBodies.back()==pgrabbed);
        // do not call _selfcollisionchecker->RemoveKinBody since the same object might be re-attached later on and we should preserve the structures.
        _vGrabbedBodies.pop_back();
        throw;
    }
    //uint64_t starttime3 = utils::GetMicroTime();
    try {
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
    catch (const std::exception& ex) {
        RAVELOG_ERROR_FORMAT("env=%d, failed in _PostprocessChangedParameters: %s", GetEnv()->GetId()%ex.what());
        throw;
    }
    //RAVELOG_DEBUG_FORMAT("env=%d, post process elapsed (%d) %fs, %fs, %fs, %fs", GetEnv()->GetId()%vattachedlinks.size()%(1e-6*(starttime1-starttime0))%(1e-6*(starttime2-starttime0))%(1e-6*(starttime3-starttime0))%(1e-6*(utils::GetMicroTime()-starttime0)));
#endif
    return true;
}

bool KinBody::Grab(KinBodyPtr pGrabbedBody, LinkPtr pGrabbingLink, const std::set<std::string>& setIgnoreGrabberLinkNames)
{
    std::set<int> setGrabberLinksToIgnore;
    FOREACHC(itLinkName, setIgnoreGrabberLinkNames) {
        setGrabberLinksToIgnore.insert(GetLink(*itLinkName)->GetIndex());
    }
    return Grab(pGrabbedBody, pGrabbingLink, setGrabberLinksToIgnore);
}

bool KinBody::Grab(KinBodyPtr pGrabbedBody, LinkPtr pGrabbingLink, const std::set<int>& setGrabberLinksToIgnore)
{
    /*
       pbody --> pGrabbedBody
       pBodyLinkToGrabWith --> pGrabbingLink
       setBodyLinksToIgnore -> setGrabberLinksToIgnore
     */
    OPENRAVE_ASSERT_FORMAT(!!pGrabbedBody, "env=%s, body to be grabbed by body '%s' is invalid", GetEnv()->GetNameId()%GetName(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(!!pGrabbingLink, "env=%s, pGrabbingLink of body '%s' for grabbing body '%s' is invalid", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pGrabbingLink->GetParent().get() == this, "env=%s, pGrabbingLink name='%s' for grabbing '%s' is not part of body '%s'", GetEnv()->GetNameId()%pGrabbingLink->GetName()%pGrabbedBody->GetName()%GetName(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(pGrabbedBody.get() != this, "env=%s, body '%s' cannot grab itself", GetEnv()->GetNameId()%pGrabbedBody->GetName(), ORE_InvalidArguments);

    // If pGrabbedBody has previously been grabbed, update its set of ignore links and return.
    if( IsGrabbing(*pGrabbedBody) ) {
        if( setGrabberLinksToIgnore.size() > 0 ) {
            // Update Grabbed with additional links to ignore
            FOREACHC(itGrabbed, _vGrabbedBodies) {
                GrabbedPtr pGrabbed = boost::dynamic_pointer_cast<Grabbed>(*itGrabbed);
                if( pGrabbed->_pGrabbedBody.lock() == pGrabbedBody ) {
                    pGrabbed->AddMoreIgnoreLinks(setGrabberLinksToIgnore);
                    break;
                }
            }
        }
        RAVELOG_VERBOSE_FORMAT("env=%s, body '%s' grabs body '%s' that has previously been grabbed", GetEnv()->GetNameId()%GetName()%pGrabbedBody->GetName());
        return true;
    }

    GrabbedPtr pGrabbed(new Grabbed(pGrabbedBody, pGrabbingLink));
    Transform tGrabbingLink = pGrabbingLink->GetTransform();
    Transform tGrabbedBody = pGrabbedBody->GetTransform();
    pGrabbed->_tRelative = tGrabbingLink.inverse() * tGrabbedBody;

    pGrabbed->_setGrabberLinksToIgnore = setGrabberLinksToIgnore;

    if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
        _selfcollisionchecker->InitKinBody(pGrabbedBody);
    }

    std::pair<Vector, Vector> velocity = pGrabbingLink->GetVelocity();
    velocity.first += velocity.second.cross(tGrabbedBody.trans - tGrabbingLink.trans);
    pGrabbedBody->SetVelocity(velocity.first, velocity.second);
    _vGrabbedBodies.push_back(pGrabbed);

    try {
        _AttachBody(pGrabbedBody);
    }
    catch(...) {
        RAVELOG_ERROR_FORMAT("env=%s, failed to attach %s to %s when grabbing", GetEnv()->GetNameId()%pGrabbedBody->GetName()%GetName());
        BOOST_ASSERT(_vGrabbedBodies.back() == pGrabbed);
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

#if 0
    if( IsGrabbing(*pbody) ) {
        if( setBodyLinksToIgnore.size() > 0 ) {
            // update the current grabbed info with setBodyLinksToIgnore
            FOREACHC(itgrabbed, _vGrabbedBodies) {
                GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                if( pgrabbed->_pgrabbedbody.lock() == pbody ) {
                    pgrabbed->AddMoreIgnoreLinks(setBodyLinksToIgnore);
                    break;
                }
            }
        }
        RAVELOG_VERBOSE(str(boost::format("Body %s: body %s already grabbed\n")%GetName()%pbody->GetName()));
        return true;
    }

    GrabbedPtr pgrabbed(new Grabbed(pbody,pBodyLinkToGrabWith));
    Transform t = pBodyLinkToGrabWith->GetTransform();
    Transform tbody = pbody->GetTransform();
    pgrabbed->_troot = t.inverse() * tbody;

    if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
        // collision checking will not be automatically updated with environment calls, so need to do this manually
        _selfcollisionchecker->InitKinBody(pbody);
    }
    pgrabbed->ProcessCollidingLinks(setBodyLinksToIgnore);

    // set velocity
    std::pair<Vector, Vector> velocity = pBodyLinkToGrabWith->GetVelocity();
    velocity.first += velocity.second.cross(tbody.trans - t.trans);
    pbody->SetVelocity(velocity.first, velocity.second);
    _vGrabbedBodies.push_back(pgrabbed);
    try {
        // if an exception happens in _AttachBody, have to remove from _vGrabbedBodies
        _AttachBody(pbody);
    }
    catch(...) {
        BOOST_ASSERT(_vGrabbedBodies.back()==pgrabbed);
        // do not call _selfcollisionchecker->RemoveKinBody since the same object might be re-attached later on and we should preserve the structures.
        _vGrabbedBodies.pop_back();
        throw;
    }
    _PostprocessChangedParameters(Prop_RobotGrabbed);
#endif
    return true;
}

void KinBody::Release(KinBody &body)
{
    FOREACH(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
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
        FOREACH(itgrabbed, _vGrabbedBodies) {
            GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
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
        FOREACH(itgrabbed, _vGrabbedBodies) {
            GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
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
            GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(_vGrabbedBodies.at(nCheckIndex));
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
    std::vector<CallOnDestruction> vHooks;
    vHooks.reserve(numGrabbed);
    FOREACH(itGrabbed, _vGrabbedBodies) {
        GrabbedPtr pGrabbed = boost::dynamic_pointer_cast<Grabbed>(*itGrabbed);
        KinBodyPtr pBody = pGrabbed->_pGrabbedBody.lock();
        if( !!pBody ) {
            _RemoveAttachedBody(*pBody);
            vHooks.push_back(CallOnDestruction(boost::bind(&RobotBase::_AttachBody, this, pBody)));
        }
    }

    std::vector<GrabbedPtr> vOriginalGrabbed;
    vOriginalGrabbed.swap(_vGrabbedBodies);

    _vGrabbedBodies.reserve(numGrabbed);
    for( size_t iGrabbed = 0; iGrabbed < numGrabbed; ++iGrabbed ) {
        GrabbedPtr pGrabbed = vOriginalGrabbed[iGrabbed];
        KinBodyPtr pBody = pGrabbed->_pGrabbedBody.lock();
        if( !pBody ) {
            RAVELOG_WARN_FORMAT("env=%s, grabbed body %d/%d does not exist any more", GetEnv()->GetNameId()%iGrabbed%numGrabbed);
            continue;
        }

        GrabbedPtr pNewGrabbed(new Grabbed(pBody, pGrabbed->_pGrabbingLink));
        pNewGrabbed->_tRelative = pGrabbed->_tRelative;
        pNewGrabbed->_setGrabberLinksToIgnore.swap(pGrabbed->_setGrabberLinksToIgnore);
        _vGrabbedBodies.push_back(pNewGrabbed);
    }

#if 0
    FOREACH(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        KinBodyPtr pbody = pgrabbed->_pgrabbedbody.lock();
        if( !!pbody ) {
            _RemoveAttachedBody(*pbody);
            CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pbody));
            pgrabbed->ProcessCollidingLinks(pgrabbed->_setRobotLinksToIgnore);
        }
    }
#endif
}

void KinBody::_Regrab(UserDataPtr _pgrabbed)
{
    GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(_pgrabbed);
    KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
    RAVELOG_WARN("Not implemented yet.");
#if 0
    if( !!pgrabbedbody ) {
        // have to re-grab the body, which means temporarily resetting the collision checker and attachment
        CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        _RemoveAttachedBody(*pgrabbedbody);
        CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pgrabbedbody));
        pgrabbed->ProcessCollidingLinks(pgrabbed->_setRobotLinksToIgnore);
    }
#endif
}

KinBody::LinkPtr KinBody::IsGrabbing(const KinBody &body) const
{
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
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
    for( const UserDataPtr& grabbedDataPtr : _vGrabbedBodies ) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(grabbedDataPtr);
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

int KinBody::CheckGrabbedInfo(const KinBody& body, const KinBody::Link& bodyLinkToGrabWith, const std::set<int>& setGrabberLinksToIgnore) const
{
    /*
       setBodyLinksToIgnore --> setGrabberLinksToIgnore
     */
    GrabbedInfoCheckResult defaultErrorCode = GICR_BodyNotGrabbed;
    for( const UserDataPtr& grabbedDataPtr : _vGrabbedBodies ) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(grabbedDataPtr);
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

        // pgrabbed->ComputeListNonCollidingLinks();

        // compare ignored robot links
        bool ignoringLinksMatch = true;
        size_t numIgnoredLinks = 0;  // needed to detect non-existing links in setBodyLinksToIgnore
        for( const LinkPtr& link : _veclinks ) {
            const bool isLinkIgnored = std::find(pgrabbed->_setGrabberLinksToIgnore.begin(), pgrabbed->_setGrabberLinksToIgnore.end(), link->GetIndex()) != pgrabbed->_setGrabberLinksToIgnore.end(); // ||
            // find(pgrabbed->_listNonCollidingLinksWhenGrabbed.begin(), pgrabbed->_listNonCollidingLinksWhenGrabbed.end(), link) == pgrabbed->_listNonCollidingLinksWhenGrabbed.end();
            if( isLinkIgnored ) {
                ++numIgnoredLinks;
            }
            if( isLinkIgnored != (setGrabberLinksToIgnore.count(link->GetIndex()) > 0) ) {
                ignoringLinksMatch = false;
                break;
            }
        }
        if( ignoringLinksMatch && numIgnoredLinks == setGrabberLinksToIgnore.size() ) {
            return GICR_Identical;
        }
    }
    return defaultErrorCode;
}

int KinBody::CheckGrabbedInfo(const KinBody& body, const KinBody::Link& bodyLinkToGrabWith, const std::set<std::string>& setGrabberLinksToIgnore) const
{
    GrabbedInfoCheckResult defaultErrorCode = GICR_BodyNotGrabbed;
    for( const UserDataPtr& grabbedDataPtr : _vGrabbedBodies ) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(grabbedDataPtr);
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

        // pgrabbed->ComputeListNonCollidingLinks();

        // compare ignored robot links
        bool ignoringLinksMatch = true;
        size_t numIgnoredLinks = 0;  // needed to detect non-existing links in setBodyLinksToIgnore
        for( const LinkPtr& link : _veclinks ) {
            const bool isLinkIgnored = std::find(pgrabbed->_setGrabberLinksToIgnore.begin(), pgrabbed->_setGrabberLinksToIgnore.end(), link->GetIndex()) != pgrabbed->_setGrabberLinksToIgnore.end(); // ||
            // find(pgrabbed->_listNonCollidingLinksWhenGrabbed.begin(), pgrabbed->_listNonCollidingLinksWhenGrabbed.end(), link) == pgrabbed->_listNonCollidingLinksWhenGrabbed.end();
            if( isLinkIgnored ) {
                ++numIgnoredLinks;
            }
            if( isLinkIgnored != (setGrabberLinksToIgnore.count(link->GetName()) > 0) ) {
                ignoringLinksMatch = false;
                break;
            }
        }
        if( ignoringLinksMatch && numIgnoredLinks == setGrabberLinksToIgnore.size() ) {
            return GICR_Identical;
        }
    }
    return defaultErrorCode;
}

void KinBody::GetGrabbed(std::vector<KinBodyPtr>& vbodies) const
{
    vbodies.clear();
    vbodies.reserve(_vGrabbedBodies.size());
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        KinBodyPtr pbody = pgrabbed->_pGrabbedBody.lock();
        if( !!pbody && pbody->GetEnvironmentBodyIndex() ) {
            vbodies.push_back(pbody);
        }
    }
}

KinBodyPtr KinBody::GetGrabbedBody(int iGrabbed) const
{
    GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(_vGrabbedBodies.at(iGrabbed));
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
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(_vGrabbedBodies[i]);
        KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        // sometimes bodies can be removed before they are Released, this is ok and can happen during exceptions and stack unwinding
        if( !!pgrabbedbody ) {
            KinBody::GrabbedInfoPtr poutputinfo(new GrabbedInfo());
            poutputinfo->_grabbedname = pgrabbedbody->GetName();
            poutputinfo->_robotlinkname = pgrabbed->_pGrabbingLink->GetName();
            poutputinfo->_trelative = pgrabbed->_tRelative;
            poutputinfo->_setIgnoreRobotLinkNames.clear();

            FOREACHC(itlink, _veclinks) {
                if( find(pgrabbed->_setGrabberLinksToIgnore.begin(), pgrabbed->_setGrabberLinksToIgnore.end(), (*itlink)->GetIndex()) != pgrabbed->_setGrabberLinksToIgnore.end() ) {
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

        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(_vGrabbedBodies[igrabbed]);
        KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
        // sometimes bodies can be removed before they are Released, this is ok and can happen during exceptions and stack unwinding
        if( !!pgrabbedbody ) {
            KinBody::GrabbedInfo& outputinfo = vGrabbedInfos[igrabbed];
            outputinfo._grabbedname = pgrabbedbody->GetName();
            outputinfo._robotlinkname = pgrabbed->_pGrabbingLink->GetName();
            outputinfo._trelative = pgrabbed->_tRelative;
            outputinfo._setIgnoreRobotLinkNames.clear();

            FOREACHC(itlink, _veclinks) {
                if( find(pgrabbed->_setGrabberLinksToIgnore.begin(), pgrabbed->_setGrabberLinksToIgnore.end(), (*itlink)->GetIndex()) != pgrabbed->_setGrabberLinksToIgnore.end() ) {
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
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(_vGrabbedBodies[igrabbed]);
        if( !!pgrabbed ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody && pgrabbedbody->GetName() == grabbedname ) {
                grabbedInfo._grabbedname = pgrabbedbody->GetName();
                grabbedInfo._robotlinkname = pgrabbed->_pGrabbingLink->GetName();
                grabbedInfo._trelative = pgrabbed->_tRelative;
                grabbedInfo._setIgnoreRobotLinkNames.clear();

                FOREACHC(itlink, _veclinks) {
                    if( find(pgrabbed->_setGrabberLinksToIgnore.begin(), pgrabbed->_setGrabberLinksToIgnore.end(), (*itlink)->GetIndex()) != pgrabbed->_setGrabberLinksToIgnore.end() ) {
                        grabbedInfo._setIgnoreRobotLinkNames.insert((*itlink)->GetName());
                    }
                }
                return true;
            }
        }
    }
    return false;
}

void KinBody::GrabbedInfo::Reset()
{
    _id.clear();
    _grabbedname.clear();
    _robotlinkname.clear();
    _trelative = Transform();
    _setIgnoreRobotLinkNames.clear();
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
}

void KinBody::GrabbedInfo::serialize(std::ostream& o) const
{
    o << _grabbedname << " ";
    o << _robotlinkname << " ";
    SerializeRound(o, _trelative);
    for( std::set<std::string>::const_iterator it = _setIgnoreRobotLinkNames.begin(); it != _setIgnoreRobotLinkNames.end(); ++it ) {
        o << (*it) << " ";
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
                pGrabbed->_setGrabberLinksToIgnore.insert(GetLink(*itLinkName)->GetIndex());
            }

            // TODO: do we need to set velocities?

            _vGrabbedBodies.push_back(pGrabbed);
            _AttachBody(pBody);
        } // end FOREACHC

        _PostprocessChangedParameters(Prop_RobotGrabbed);
    } // end if vGrabbedInfos.size() > 0
#if 0
    if( vgrabbedinfo.size() > 0 ) {
        CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        FOREACHC(itgrabbedinfo, vgrabbedinfo) {
            GrabbedInfoConstPtr pgrabbedinfo = *itgrabbedinfo;
            KinBodyPtr pbody = GetEnv()->GetKinBody(pgrabbedinfo->_grabbedname);
            OPENRAVE_ASSERT_FORMAT(!!pbody, "env=%d, when grabbing with body '%s' invalid grab body '%s'",GetEnv()->GetId()%GetName()%pgrabbedinfo->_grabbedname, ORE_InvalidArguments);
            KinBody::LinkPtr pBodyLinkToGrabWith = GetLink(pgrabbedinfo->_robotlinkname);
            if( !pBodyLinkToGrabWith ) {
                std::stringstream ss;
                for(const LinkPtr& plink : _veclinks) {
                    ss << plink->GetName() << ",";
                }
                throw OPENRAVE_EXCEPTION_FORMAT("env=%d, when grabbing with body '%s' invalid grab link '%s'. Available links are [%s]",GetEnv()->GetId()%GetName()%pgrabbedinfo->_robotlinkname%ss.str(), ORE_InvalidArguments);
            }
            OPENRAVE_ASSERT_FORMAT(pbody.get() != this, "body %s cannot grab itself",pbody->GetName(), ORE_InvalidArguments);
            if( IsGrabbing(*pbody) ) {
                RAVELOG_VERBOSE(str(boost::format("Body %s: body %s already grabbed\n")%GetName()%pbody->GetName()));
                continue;
            }

            GrabbedPtr pgrabbed(new Grabbed(pbody,pBodyLinkToGrabWith));
            pgrabbed->_troot = pgrabbedinfo->_trelative;
            if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
                // collision checking will not be automatically updated with environment calls, so need to do this manually
                _selfcollisionchecker->InitKinBody(pbody);
            }
            std::set<int> setRobotLinksToIgnore;
            FOREACHC(itLinkName, pgrabbedinfo->_setIgnoreRobotLinkNames) {
                setRobotLinksToIgnore.insert(GetLink(*itLinkName)->GetIndex());
            }
            pgrabbed->ProcessCollidingLinks(setRobotLinksToIgnore);
            Transform tlink = pBodyLinkToGrabWith->GetTransform();
            Transform tbody = tlink * pgrabbed->_troot;
            if( pbody->GetLinks().size() == 0 ) {
                RAVELOG_WARN_FORMAT("env=%d, cannot set transform of body '%s' with no links when grabbing by '%s'", GetEnv()->GetId()%pbody->GetName()%GetName());
            }
            pbody->SetTransform(tbody);
            // set velocity
            std::pair<Vector, Vector> velocity = pBodyLinkToGrabWith->GetVelocity();
            velocity.first += velocity.second.cross(tbody.trans - tlink.trans);
            pbody->SetVelocity(velocity.first, velocity.second);
            _vGrabbedBodies.push_back(pgrabbed);
            _AttachBody(pbody);
        }
        _PostprocessChangedParameters(Prop_RobotGrabbed);
    }
#endif
}

void KinBody::GetIgnoredLinksOfGrabbed(KinBodyConstPtr body, std::list<KinBody::LinkConstPtr>& ignorelinks) const
{
    ignorelinks.clear();
    FOREACHC(itGrabbed, _vGrabbedBodies) {
        GrabbedConstPtr pGrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itGrabbed);
        KinBodyPtr pGrabbedBody = pGrabbed->_pGrabbedBody.lock();
        if( pGrabbedBody == body ) {
            FOREACHC(itGrabberLink, _veclinks) {
                if( pGrabbed->_setGrabberLinksToIgnore.find((*itGrabberLink)->GetIndex()) != pGrabbed->_setGrabberLinksToIgnore.end() ) {
                    ignorelinks.push_back(*itGrabberLink);
                }
            }
            return;
        }
    }
#if 0
    FOREACHC(itgrabbed, _vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbed);
        KinBodyPtr grabbedbody = pgrabbed->_pgrabbedbody.lock();
        if( grabbedbody == body ) {
            FOREACHC(itbodylink, _veclinks) {
                if( find(pgrabbed->_listNonCollidingLinks.begin(), pgrabbed->_listNonCollidingLinks.end(), *itbodylink) == pgrabbed->_listNonCollidingLinks.end() ) {
                    ignorelinks.push_back(*itbodylink);
                }
            }
            return;
        }
    }
#endif
    RAVELOG_WARN(str(boost::format("body %s is not currently grabbed")%body->GetName()));
}

void KinBody::_UpdateGrabbedBodies()
{
    std::vector<GrabbedPtr>::iterator itgrabbed = _vGrabbedBodies.begin();
    std::pair<Vector, Vector> velocity;
    while(itgrabbed != _vGrabbedBodies.end() ) {
        GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
        KinBodyPtr pGrabbedBody = pgrabbed->_pGrabbedBody.lock();
        if( !!pGrabbedBody ) {
            Transform t = pgrabbed->_pGrabbingLink->GetTransform();
            pGrabbedBody->SetTransform(t * pgrabbed->_tRelative);
            // set the correct velocity
            pgrabbed->_pGrabbingLink->GetVelocity(velocity.first, velocity.second);
            velocity.first += velocity.second.cross(t.rotate(pgrabbed->_tRelative.trans));
            pGrabbedBody->SetVelocity(velocity.first, velocity.second);
            ++itgrabbed;
        }
        else {
            RAVELOG_DEBUG_FORMAT("env=%d, erasing invaliding grabbed body from %s", GetEnv()->GetId()%GetName());
            itgrabbed = _vGrabbedBodies.erase(itgrabbed);
        }
    }
}

} // end namespace OpenRAVE
