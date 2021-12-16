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
    // always ignore links that are statically attached to plink (ie assume they are always colliding with the body)
    std::set<int> setGrabberLinksToIgnore;
    std::vector<KinBody::LinkPtr> vAttachedToGrabbingLink;
    pGrabbingLink->GetRigidlyAttachedLinks(vAttachedToGrabbingLink);
    FOREACHC(itAttachedLink, vAttachedToGrabbingLink) {
        setGrabberLinksToIgnore.insert((*itAttachedLink)->GetIndex());
    }
    return Grab(pGrabbedBody, pGrabbingLink, setGrabberLinksToIgnore);
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
                // Grabbing the same object at the same relative transform with the same grabbing link. So just modify
                // setGrabberLinksToIgnore and then return.
                pPreviouslyGrabbed->AddMoreIgnoreLinks(setGrabberLinksToIgnore);
                return true;
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
    pGrabbed->_setGrabberLinksToIgnore = setGrabberLinksToIgnore;

    if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
        // collision checking will not be automatically updated with environment calls, so need to do this manually
        _selfcollisionchecker->InitKinBody(pGrabbedBody);
    }

    std::pair<Vector, Vector> velocity = pGrabbingLink->GetVelocity();
    velocity.first += velocity.second.cross(tGrabbedBody.trans - tGrabbingLink.trans);
    pGrabbedBody->SetVelocity(velocity.first, velocity.second);
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
        pNewGrabbed->_setGrabberLinksToIgnore.swap(pGrabbed->_setGrabberLinksToIgnore);

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

void KinBody::_Regrab(GrabbedPtr pgrabbed)
{
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

int KinBody::CheckGrabbedInfo(const KinBody& body, const KinBody::Link& bodyLinkToGrabWith, const std::set<int>& setGrabberLinksToIgnore) const
{
    /*
       setBodyLinksToIgnore --> setGrabberLinksToIgnore
     */
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

        GrabbedConstPtr pgrabbed = _vGrabbedBodies[igrabbed];
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
        GrabbedConstPtr pgrabbed = _vGrabbedBodies[igrabbed];
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
                if( pGrabbed->_setGrabberLinksToIgnore.find((*itGrabberLink)->GetIndex()) != pGrabbed->_setGrabberLinksToIgnore.end() ) {
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
