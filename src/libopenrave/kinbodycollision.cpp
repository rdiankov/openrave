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

/// \brief print the status on check self collision.
static void _PrintStatusOnCheckSelfCollisoin(CollisionReportPtr& report, const KinBody& body)
{
    if( IS_DEBUGLEVEL(Level_Verbose) ) {
        std::vector<OpenRAVE::dReal> colvalues;
        body.GetDOFValues(colvalues);
        std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
        FOREACHC(itval, colvalues) {
            ss << *itval << ",";
        }
        RAVELOG_VERBOSE_FORMAT("env=%s, self collision report=%s; colvalues=[%s]", body.GetEnv()->GetNameId()%report->__str__()%ss.str());
    }
}

/// \brief post process on the check self collision, mostly for grabbed bodies.
static void _PostProcessOnCheckSelfCollision(CollisionReportPtr& report, CollisionReportPtr& pusereport, const KinBody& body)
{
    if( !!report ) {
        if( report != pusereport ) {
            *report = *pusereport;
        }
    }
}

/// \brief check if the pair of grabbed-grabber links should be skipped. This function is only called when pGrabberLinkToCheck is specified.
/// \param bIsGrabbedBodyAttachedToGrabbingLinkToCheck : true if the given grabbed body's pGrabbingLink is same as pGrabberLinkToCheck
/// \param pGrabberLink : grabber link
/// \param pGrabberLinkToCheck : grabber link to check.
/// \param vIncludedLinks : vector of included links.
/// \param pGrabbed : ptr of Grabbed instance.
/// \return true if the collision checking should be skipped.
static bool _ShouldSkipGrabbedGrabberCollisionCheck(const bool bIsGrabbedBodyAttachedToGrabbingLinkToCheck,
                                                    const KinBody::LinkConstPtr& pGrabberLink,
                                                    const KinBody::LinkPtr& pGrabberLinkToCheck,
                                                    const std::vector<KinBody::LinkConstPtr>& vIncludedLinks,
                                                    const Grabbed* pGrabbed)
{
    // if this grabbed body is not attached to pGrabberLinkToCheck, and if the target link in the pair is not same pGrabberLinkToCheck, we should skip collision checking.
    if( !bIsGrabbedBodyAttachedToGrabbingLinkToCheck && pGrabberLink != pGrabberLinkToCheck ) {
        return true;
    }
    // if vIncludedLinks is not empty, and taret link in the pair is not in the vIncludedLinks, we should skip collision checking.
    if( vIncludedLinks.size() > 0 ) {
        const KinBody::LinkConstPtr& pLinkToCheck = bIsGrabbedBodyAttachedToGrabbingLinkToCheck ? pGrabberLink : pGrabbed->_pGrabbingLink;
        if( std::find(vIncludedLinks.begin(), vIncludedLinks.end(), pLinkToCheck) == vIncludedLinks.end() ) {
            return true;
        }
    }
    return false;
}

/// \brief check if the pair of inter-grabber links should be skipped. This function is only called when pGrabberLinkToCheck is specified.
/// \param vGrabbedBodiesWithGrabberLinkToCheck : vector of grabbed bodies which pGrabbingLink is same as pGrabberLinkToCheck
/// \param vGrabbedBodiesInInclusiveLinks : vector of grabbed bodies which pGrabbingLink is included in vIncludedLinks
/// \param vIncludedLinks : vector of included links.
/// \param pGrabbedBody1, pGrabbedBody2 : two grabbed bodies.
/// \return true if the collision checking should be skipped.
static bool _ShouldSkipInterGrabbedCollisionCheck(const std::vector<KinBody*>& vGrabbedBodiesWithGrabberLinkToCheck,
                                                  const std::vector<KinBody*>& vGrabbedBodiesInInclusiveLinks,
                                                  const std::vector<KinBody::LinkConstPtr>& vIncludedLinks,
                                                  const KinBodyPtr& pGrabbedBody1,
                                                  const KinBodyPtr& pGrabbedBody2)
{
    if( std::find(vGrabbedBodiesWithGrabberLinkToCheck.begin(), vGrabbedBodiesWithGrabberLinkToCheck.end(), pGrabbedBody1.get()) != vGrabbedBodiesWithGrabberLinkToCheck.end() ) {
        if ( vIncludedLinks.empty() || std::find(vGrabbedBodiesInInclusiveLinks.begin(), vGrabbedBodiesInInclusiveLinks.end(), pGrabbedBody2.get()) != vGrabbedBodiesInInclusiveLinks.end() ) {
            return false;
        }
    }
    else if (std::find(vGrabbedBodiesWithGrabberLinkToCheck.begin(), vGrabbedBodiesWithGrabberLinkToCheck.end(), pGrabbedBody2.get()) != vGrabbedBodiesWithGrabberLinkToCheck.end() ) {
        if ( vIncludedLinks.empty() || std::find(vGrabbedBodiesInInclusiveLinks.begin(), vGrabbedBodiesInInclusiveLinks.end(), pGrabbedBody1.get()) != vGrabbedBodiesInInclusiveLinks.end() ) {
            return false;
        }
    }
    return true;
}

bool KinBody::CheckSelfCollision(CollisionReportPtr report, CollisionCheckerBasePtr collisionchecker) const
{
    if( !collisionchecker ) {
        collisionchecker = _selfcollisionchecker;
        if( !collisionchecker ) {
            collisionchecker = GetEnv()->GetCollisionChecker();
            if( !collisionchecker ) {
                // no checker set
                return false;
            }
        }
        else {
            // have to set the same options as GetEnv()->GetCollisionChecker() since stuff like CO_ActiveDOFs is only set on the global checker
            collisionchecker->SetCollisionOptions(GetEnv()->GetCollisionChecker()->GetCollisionOptions());
        }
    }

    bool bAllLinkCollisions = !!(collisionchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bCollision = false;
    if( collisionchecker->CheckStandaloneSelfCollision(shared_kinbody_const(), report) ) {
        if( !!report ) {
            _PrintStatusOnCheckSelfCollisoin(report, *this);
        }
        if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
            return true;
        }

        bCollision = true;
    }

    if( _CheckGrabbedBodiesSelfCollision(collisionchecker, report, bAllLinkCollisions, LinkPtr(), std::vector<KinBody::LinkConstPtr>(), nullptr) ) {
        if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
            return true;
        }
        bCollision = true;
    }

    if( bCollision && !!report ) {
        _PrintStatusOnCheckSelfCollisoin(report, *this);
    }
    return bCollision;
}

bool KinBody::_CheckGrabbedBodiesSelfCollision(CollisionCheckerBasePtr& collisionchecker,
                                               CollisionReportPtr& report,
                                               const bool bAllLinkCollisions,
                                               const KinBody::LinkPtr& pGrabberLinkToCheck,
                                               const std::vector<KinBody::LinkConstPtr>& vIncludedLinks,
                                               const TransformConstPtr& pLinkTransformForGrabberLinkToCheck) const
{
    const bool bCheckSpecificGrabbingLinkOnly = !!pGrabberLinkToCheck;
    if( vIncludedLinks.size() > 0 ) {
        OPENRAVE_ASSERT_FORMAT(bCheckSpecificGrabbingLinkOnly, "env=%s, vIncludedLinks is specified, but pGrabberLinkToCheck is not specified. for body '%s'", GetEnv()->GetNameId()%GetName(), ORE_InvalidArguments);
    }
    if( !!pLinkTransformForGrabberLinkToCheck ) {
        OPENRAVE_ASSERT_FORMAT(bCheckSpecificGrabbingLinkOnly, "env=%s, pLinkTransformForGrabberLinkToCheck is specified, but pGrabberLinkToCheck is not specified. for body '%s'", GetEnv()->GetNameId()%GetName(), ORE_InvalidArguments);
    }

    bool bCollision = false;
    // if collision checker is set to distance checking, have to compare reports for the minimum distance
    const int coloptions = collisionchecker->GetCollisionOptions();
    CollisionReport tempreport;
    CollisionReportPtr pusereport = report;
    if( !!report && (coloptions & CO_Distance) ) {
        pusereport = boost::shared_ptr<CollisionReport>(&tempreport,utils::null_deleter());
    }

    // Flatten the grabbed bodies so that we can zip our iteration with the cache of locked pointers
    // Use raw pointers to save overhead here since lifetime is guaranteed by _grabbedBodiesByEnvironmentIndex
    // locking weak pointer is expensive, so do it N times and cache, where N is the number of grabbedBody instead of N^2
    std::vector<Grabbed*> vGrabbedBodies;
    vGrabbedBodies.reserve(_grabbedBodiesByEnvironmentIndex.size());
    std::vector<KinBodyPtr> vLockedGrabbedBodiesCache;
    vLockedGrabbedBodiesCache.reserve(_grabbedBodiesByEnvironmentIndex.size());
    for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        KinBodyPtr pGrabbedBody = grabPair.second.get()->_pGrabbedBody.lock();
        if( !pGrabbedBody ) {
            RAVELOG_WARN_FORMAT("env=%s, grabbed body on %s has already been destroyed, ignoring.", GetEnv()->GetNameId()%GetName());
            continue;
        }
        if( !pGrabbedBody->IsEnabled() ) {
            continue;
        }
        vGrabbedBodies.emplace_back(grabPair.second.get());
        vLockedGrabbedBodiesCache.push_back(pGrabbedBody);
    }

    // check all grabbed bodies with (TODO: support CO_ActiveDOFs option)
    const size_t numGrabbed = vGrabbedBodies.size();
    // RAVELOG_INFO_FORMAT("env=%s, checking self collision for %s with grabbed bodies: numgrabbed=%d", GetEnv()->GetNameId()%GetName()%numGrabbed);
    std::vector<KinBody*> vGrabbedBodiesWithGrabberLinkToCheck; // vector of grabbed bodies which pGrabbingLink is same as pGrabberLinkToCheck
    std::vector<KinBody*> vGrabbedBodiesInInclusiveLinks; // vector of grabbed bodies which pGrabbingLink is included in vIncludedLinks
    std::vector<KinBody::KinBodyStateSaverPtr> vGrabbedBodyStateSaversWithGivenGrabbingLink;
    if( bCheckSpecificGrabbingLinkOnly ) {
        vGrabbedBodiesWithGrabberLinkToCheck.reserve(numGrabbed);
        vGrabbedBodiesInInclusiveLinks.reserve(numGrabbed);
        vGrabbedBodyStateSaversWithGivenGrabbingLink.reserve(numGrabbed);
    }
    for (size_t indexGrabbed1 = 0; indexGrabbed1 < numGrabbed; indexGrabbed1++) {
        Grabbed* pGrabbed = vGrabbedBodies[indexGrabbed1];
        pGrabbed->ComputeListNonCollidingLinks();
        const ListNonCollidingLinkPairs& nonCollidingLinkPairs = pGrabbed->_listNonCollidingGrabbedGrabberLinkPairsWhenGrabbed;
        const bool bIsGrabbedBodyAttachedToGrabbingLinkToCheck = pGrabbed->_pGrabbingLink == pGrabberLinkToCheck; // true if this grabbedbody's grabbinglink should be checked.
        if( bCheckSpecificGrabbingLinkOnly ) {
            if( bIsGrabbedBodyAttachedToGrabbingLinkToCheck ) {
                KinBodyPtr& pGrabbedBody = vLockedGrabbedBodiesCache[indexGrabbed1];
                vGrabbedBodiesWithGrabberLinkToCheck.emplace_back(pGrabbedBody.get());
                if( !!pLinkTransformForGrabberLinkToCheck ) {
                    vGrabbedBodyStateSaversWithGivenGrabbingLink.push_back(KinBodyStateSaverPtr(new KinBodyStateSaver(pGrabbedBody, KinBody::Save_LinkTransformation)));
                    pGrabbedBody->SetTransform((*pLinkTransformForGrabberLinkToCheck) * pGrabbed->_tRelative);
                }
            }
            if( vIncludedLinks.size() > 0 && std::find(vIncludedLinks.begin(), vIncludedLinks.end(), pGrabbed->_pGrabbingLink) != vIncludedLinks.end() ) {
                vGrabbedBodiesInInclusiveLinks.emplace_back(vLockedGrabbedBodiesCache[indexGrabbed1].get());
            }
        }

        KinBodyPtr pLinkParent;
        FOREACHC(itNonCollidingLinkPairs, nonCollidingLinkPairs) {
            const KinBody::LinkConstPtr& probotlinkFromNonColliding = (*itNonCollidingLinkPairs).second;
            const KinBody::Link& robotlinkFromNonColliding = *probotlinkFromNonColliding;
            pLinkParent = robotlinkFromNonColliding.GetParent(true);
            if( !pLinkParent ) {
                RAVELOG_WARN_FORMAT("env=%s, _listNonCollidingLinks has invalid link %s:%d", GetEnv()->GetNameId()%robotlinkFromNonColliding.GetName()%robotlinkFromNonColliding.GetIndex());
            }
            const KinBody::LinkConstPtr& probotlink = (!!pLinkParent) ? probotlinkFromNonColliding : _veclinks.at(robotlinkFromNonColliding.GetIndex());
            if( bCheckSpecificGrabbingLinkOnly ) {
                if( _ShouldSkipGrabbedGrabberCollisionCheck(bIsGrabbedBodyAttachedToGrabbingLinkToCheck, probotlink, pGrabberLinkToCheck, vIncludedLinks, pGrabbed) ) {
                    continue;
                }
            }

            // have to use link/link collision since link/body checks attached bodies
            const KinBody::LinkConstPtr& pGrabbedBodylink = (*itNonCollidingLinkPairs).first;
            if( collisionchecker->CheckCollision(probotlink, pGrabbedBodylink, pusereport) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    _PostProcessOnCheckSelfCollision(report, pusereport, *this);
                    return true;
                }
                bCollision = true;
            }
            if( !!pusereport && pusereport->minDistance < report->minDistance ) {
                *report = *pusereport;
            }
        }

        const KinBody& grabbedBody1 = *vLockedGrabbedBodiesCache[indexGrabbed1];
        if( grabbedBody1.CheckSelfCollision(pusereport, collisionchecker) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                _PostProcessOnCheckSelfCollision(report, pusereport, *this);
                return true;
            }
            bCollision = true;
        }
        if( !!pusereport && pusereport->minDistance < report->minDistance ) {
            *report = *pusereport;
        }
    } // end for indexGrabbed1

    // Check grabbed vs grabbed collision
    FOREACHC(itInfo, _mapListNonCollidingInterGrabbedLinkPairsWhenGrabbed) {
        const ListNonCollidingLinkPairs& pairs = (*itInfo).second;
        if( pairs.empty() ) {
            continue;
        }
        const KinBodyPtr pLink1 = pairs.front().first->GetParent(true);
        const KinBodyPtr pLink2 = pairs.front().second->GetParent(true);
        if( !pLink1 || !pLink2 ) {
            RAVELOG_WARN_FORMAT("env=%s, _listNonCollidingLinks has invalid link %s, %s", GetEnv()->GetNameId() % pairs.front().first->GetName() % pairs.front().second->GetName());
            continue;
        }
        if( bCheckSpecificGrabbingLinkOnly ) {
            if( _ShouldSkipInterGrabbedCollisionCheck(vGrabbedBodiesWithGrabberLinkToCheck, vGrabbedBodiesInInclusiveLinks, vIncludedLinks, pLink1, pLink2) ) {
                continue;
            }
        }
        FOREACHC(itLinks, pairs) {
            if( collisionchecker->CheckCollision((*itLinks).first, (*itLinks).second, pusereport) ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    _PostProcessOnCheckSelfCollision(report, pusereport, *this);
                    return true;
                }
                bCollision = true;
            }
            if( !!pusereport && pusereport->minDistance < report->minDistance ) {
                *report = *pusereport;
            }
        }
    }

    if( bCollision ) {
        _PostProcessOnCheckSelfCollision(report, pusereport, *this);
    }
    return bCollision;
}

bool KinBody::CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, KinBodyConstPtr pbody, CollisionReportPtr report)
{
    LinkPtr plink = _veclinks.at(ilinkindex);
    CollisionCheckerBasePtr pchecker = GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;
    if( plink->IsEnabled() ) {
        boost::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        plink->SetTransform(tlinktrans);
        if( pchecker->CheckCollision(LinkConstPtr(plink),pbody,report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the specified body
    for (MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        GrabbedPtr& pgrabbed = grabPair.second;
        if( pgrabbed->_pGrabbingLink == plink ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                KinBodyStateSaver bodysaver(pgrabbedbody,Save_LinkTransformation);
                pgrabbedbody->SetTransform(tlinktrans * pgrabbed->_tRelative);
                if( pchecker->CheckCollision(KinBodyConstPtr(pgrabbedbody),pbody, report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool KinBody::CheckLinkCollision(int ilinkindex, KinBodyConstPtr pbody, CollisionReportPtr report)
{
    LinkPtr plink = _veclinks.at(ilinkindex);
    CollisionCheckerBasePtr pchecker = GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
    bool bincollision = false;
    if( plink->IsEnabled() ) {
        if( pchecker->CheckCollision(LinkConstPtr(plink),pbody,report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the specified body
    for (MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
        GrabbedPtr& pgrabbed = grabPair.second;
        if( pgrabbed->_pGrabbingLink == plink ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                if( pchecker->CheckCollision(KinBodyConstPtr(pgrabbedbody),pbody, report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool KinBody::CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report)
{
    LinkPtr plink = _veclinks.at(ilinkindex);
    CollisionCheckerBasePtr pchecker = GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }

    bool bincollision = false;
    if( plink->IsEnabled() ) {
        boost::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        plink->SetTransform(tlinktrans);
        if( pchecker->CheckCollision(LinkConstPtr(plink),report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed,_grabbedBodiesByEnvironmentIndex) {
        GrabbedConstPtr pgrabbed = itgrabbed->second;
        if( pgrabbed->_pGrabbingLink == plink ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2,_grabbedBodiesByEnvironmentIndex) {
                    if( itgrabbed2 != itgrabbed ) {
                        GrabbedConstPtr pgrabbed2 = itgrabbed2->second;
                        KinBodyPtr pgrabbedbody2 = pgrabbed2->_pGrabbedBody.lock();
                        if( !!pgrabbedbody2 ) {
                            vbodyexcluded.push_back(pgrabbedbody2);
                        }
                    }
                }
                KinBodyStateSaver bodysaver(pgrabbedbody,Save_LinkTransformation);
                pgrabbedbody->SetTransform(tlinktrans * pgrabbed->_tRelative);
                if( pchecker->CheckCollision(KinBodyConstPtr(pgrabbedbody),vbodyexcluded, vlinkexcluded, report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool KinBody::CheckLinkCollision(int ilinkindex, CollisionReportPtr report)
{
    LinkPtr plink = _veclinks.at(ilinkindex);
    CollisionCheckerBasePtr pchecker = GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
    bool bincollision = false;
    if( plink->IsEnabled() ) {
        if( pchecker->CheckCollision(LinkConstPtr(plink),report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    FOREACHC(itgrabbed,_grabbedBodiesByEnvironmentIndex) {
        GrabbedConstPtr pgrabbed = itgrabbed->second;
        if( pgrabbed->_pGrabbingLink == plink ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2,_grabbedBodiesByEnvironmentIndex) {
                    if( itgrabbed2 != itgrabbed ) {
                        GrabbedConstPtr pgrabbed2 = itgrabbed2->second;
                        KinBodyPtr pgrabbedbody2 = pgrabbed2->_pGrabbedBody.lock();
                        if( !!pgrabbedbody2 ) {
                            vbodyexcluded.push_back(pgrabbedbody2);
                        }
                    }
                }
                if( pchecker->CheckCollision(KinBodyConstPtr(pgrabbedbody),vbodyexcluded, vlinkexcluded, report) ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        return true;
                    }
                    bincollision = true;
                }
            }
        }
    }
    return bincollision;
}

bool KinBody::CheckLinkSelfCollision(int ilinkindex, const std::vector<KinBody::LinkConstPtr>& vIncludedLinks, const TransformConstPtr& pLinkTransform, CollisionReportPtr report)
{
    CollisionCheckerBasePtr pchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
    bool bAllLinkCollisions = !!(pchecker->GetCollisionOptions()&CO_AllLinkCollisions);
    CollisionReportKeepSaver reportsaver(report);
    if( !!report && bAllLinkCollisions && report->nKeepPrevious == 0 ) {
        report->Reset();
        report->nKeepPrevious = 1; // have to keep the previous since aggregating results
    }
    bool bincollision = false;
    LinkPtr plink = _veclinks.at(ilinkindex);
    boost::shared_ptr<TransformSaver<LinkPtr> > linksaver;
    if( plink->IsEnabled() ) {
        if( !!pLinkTransform ) {
            linksaver = boost::shared_ptr<TransformSaver<LinkPtr> >(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
            plink->SetTransform(*pLinkTransform);
        }
        if( pchecker->CheckStandaloneSelfCollision(LinkConstPtr(plink), vIncludedLinks, report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    if( _CheckGrabbedBodiesSelfCollision(pchecker, report, bAllLinkCollisions, plink, vIncludedLinks, pLinkTransform) ) {
        bincollision = true;
    }
    return bincollision;
}

} // end namespace OpenRAVE
