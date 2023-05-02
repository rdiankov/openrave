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
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                std::vector<OpenRAVE::dReal> colvalues;
                GetDOFValues(colvalues);
                std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                FOREACHC(itval, colvalues) {
                    ss << *itval << ",";
                }
                RAVELOG_VERBOSE_FORMAT("env=%s, self collision report=%s; colvalues=[%s]", GetEnv()->GetNameId()%report->__str__()%ss.str());
            }
        }
        if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
            return true;
        }

        bCollision = true;
    }

    // if collision checker is set to distance checking, have to compare reports for the minimum distance
    int coloptions = collisionchecker->GetCollisionOptions();
    CollisionReport tempreport;
    CollisionReportPtr pusereport = report;
    if( !!report && (coloptions & CO_Distance) ) {
        pusereport = boost::shared_ptr<CollisionReport>(&tempreport,utils::null_deleter());
    }

    // locking weak pointer is expensive, so do it N times and cache, where N is the number of grabbedBody instead of N^2
    std::vector<KinBodyPtr> vLockedGrabbedBodiesCache;
    vLockedGrabbedBodiesCache.reserve(_vGrabbedBodies.size());
    for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
        vLockedGrabbedBodiesCache.push_back(pgrabbed->_pGrabbedBody.lock());
    }

    // check all grabbed bodies with (TODO: support CO_ActiveDOFs option)
    const size_t numGrabbed = _vGrabbedBodies.size();
    // RAVELOG_INFO_FORMAT("env=%s, checking self collision for %s with grabbed bodies: numgrabbed=%d", GetEnv()->GetNameId()%GetName()%numGrabbed);
    for (size_t indexGrabbed1 = 0; indexGrabbed1 < numGrabbed; indexGrabbed1++) {
        const KinBodyPtr& pGrabbedBody1 = vLockedGrabbedBodiesCache[indexGrabbed1];
        if( !pGrabbedBody1 ) {
            RAVELOG_WARN_FORMAT("env=%s, grabbed body on %s has already been destroyed, ignoring.", GetEnv()->GetNameId()%GetName());
            continue;
        }
        const KinBody& grabbedBody1 = *pGrabbedBody1;
        if( !grabbedBody1.IsEnabled() ) {
            continue;
        }

        _vGrabbedBodies[indexGrabbed1]->ComputeListNonCollidingLinks();

        const std::list<KinBody::LinkConstPtr>& nonCollidingLinks1 = _vGrabbedBodies[indexGrabbed1]->_listNonCollidingLinksWhenGrabbed;

        KinBodyPtr pLinkParent;

        for (const KinBody::LinkConstPtr& probotlinkFromNonColliding : nonCollidingLinks1) {
            const KinBody::Link& robotlinkFromNonColliding = *probotlinkFromNonColliding;
            pLinkParent = robotlinkFromNonColliding.GetParent(true);
            if( !pLinkParent ) {
                RAVELOG_WARN_FORMAT("env=%s, _listNonCollidingLinks has invalid link %s:%d", GetEnv()->GetNameId()%robotlinkFromNonColliding.GetName()%robotlinkFromNonColliding.GetIndex());
            }
            const KinBody::LinkConstPtr& probotlink = (!!pLinkParent) ? probotlinkFromNonColliding : _veclinks.at(robotlinkFromNonColliding.GetIndex());

            // have to use link/link collision since link/body checks attached bodies
            for (const KinBody::LinkPtr& pGrabbedBodylink : grabbedBody1.GetLinks()) {
                if( collisionchecker->CheckCollision(probotlink, pGrabbedBodylink, pusereport) ) {
                    bCollision = true;
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        break;
                    }
                }
                if( !!pusereport && pusereport->minDistance < report->minDistance ) {
                    *report = *pusereport;
                }
            }
            if( bCollision ) {
                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                    break;
                }
            }
        }
        if( bCollision ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                break;
            }
        }

        if( grabbedBody1.CheckSelfCollision(pusereport, collisionchecker) ) {
            bCollision = true;
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                break;
            }
        }
        if( !!pusereport && pusereport->minDistance < report->minDistance ) {
            *report = *pusereport;
        }

        if( numGrabbed > 1 ) {
            // Since collision checking is commutative (i.e. CheckCollision(link1, link2) == CheckCollision(link2, link1)), checking it once per pair is sufficient.
            for( size_t indexGrabbed2 = indexGrabbed1 + 1; indexGrabbed2 < numGrabbed; ++indexGrabbed2 ) {
                const KinBodyPtr& pGrabbedBody2 = vLockedGrabbedBodiesCache[indexGrabbed2];
                if( !pGrabbedBody2 ) {
                    RAVELOG_WARN_FORMAT("env=%s, grabbed body on %s has already been destroyed, so ignoring.", GetEnv()->GetNameId()%GetName());
                    continue;
                }
                const KinBody& grabbedBody2 = *pGrabbedBody2;
                if( !grabbedBody2.IsEnabled() ) {
                    continue;
                }

                _vGrabbedBodies[indexGrabbed2]->ComputeListNonCollidingLinks();

                const std::list<KinBody::LinkConstPtr>& nonCollidingLinks2 = _vGrabbedBodies[indexGrabbed2]->_listNonCollidingLinksWhenGrabbed;

                for( const KinBody::LinkPtr& pGrabbedBody2Link : grabbedBody2.GetLinks() ) {
                    if( !pGrabbedBody2Link->IsEnabled() ) {
                        continue;
                    }
                    // See if these two links were initially colliding. If they are, then no further
                    // check is need (as this link pair should be skipped).
                    if( std::find(nonCollidingLinks1.begin(), nonCollidingLinks1.end(), pGrabbedBody2Link) != nonCollidingLinks1.end() ) {
                        for( const KinBody::LinkPtr& pGrabbedBody1Link : grabbedBody1.GetLinks() ) {
                            if( !pGrabbedBody1Link->IsEnabled() ) {
                                continue;
                            }
                            if( std::find(nonCollidingLinks2.begin(), nonCollidingLinks2.end(), pGrabbedBody1Link) != nonCollidingLinks2.end() ) {
                                if( collisionchecker->CheckCollision(KinBody::LinkConstPtr(pGrabbedBody1Link),
                                                                     KinBody::LinkConstPtr(pGrabbedBody2Link),
                                                                     pusereport) ) {
                                    bCollision = true;
                                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                        break;
                                    }
                                } // end if CheckCollision
                                if( !!pusereport && pusereport->minDistance < report->minDistance ) {
                                    *report = *pusereport;
                                }
                            } // end if pGrabbedBody1Link in nonCollidingLinks2
                            if( bCollision ) {
                                if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                                    break;
                                }
                            }
                        } // end for pGrabbedBody1Link in nonCollidingLinks2
                    } // end if pGrabbedBody2Link in nonCollidingLinks1
                    if( bCollision ) {
                        if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                            break;
                        }
                    }
                } // end for pGrabbedBody2Link
                if( bCollision ) {
                    if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                        break;
                    }
                }
            } // end for indexGrabbed2
        } // end if numGrabbed > 1
        if( bCollision ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                break;
            }
        }
    } // end for indexGrabbed1

    if( bCollision && !!report ) {
        if( report != pusereport ) {
            *report = *pusereport;
        }
        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            std::vector<OpenRAVE::dReal> colvalues;
            GetDOFValues(colvalues);
            std::stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
            FOREACHC(itval, colvalues) {
                ss << *itval << ",";
            }
            RAVELOG_VERBOSE_FORMAT("env=%s, self collision report=%s; colvalues=[%s]", GetEnv()->GetNameId()%report->__str__()%ss.str());
        }
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
    for (const GrabbedPtr& pgrabbed : _vGrabbedBodies) {
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
    for (const GrabbedConstPtr& pgrabbed : _vGrabbedBodies) {
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
    FOREACHC(itgrabbed,_vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = *itgrabbed;
        if( pgrabbed->_pGrabbingLink == plink ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2,_vGrabbedBodies) {
                    if( itgrabbed2 != itgrabbed ) {
                        GrabbedConstPtr pgrabbed2 = *itgrabbed2;
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
    FOREACHC(itgrabbed,_vGrabbedBodies) {
        GrabbedConstPtr pgrabbed = *itgrabbed;
        if( pgrabbed->_pGrabbingLink == plink ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                vbodyexcluded.resize(0);
                vbodyexcluded.push_back(shared_kinbody_const());
                FOREACHC(itgrabbed2,_vGrabbedBodies) {
                    if( itgrabbed2 != itgrabbed ) {
                        GrabbedConstPtr pgrabbed2 = *itgrabbed2;
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

bool KinBody::CheckLinkSelfCollision(int ilinkindex, CollisionReportPtr report)
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
    if( plink->IsEnabled() ) {
        if( pchecker->CheckStandaloneSelfCollision(LinkConstPtr(plink),report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    KinBodyStateSaverPtr linksaver;
    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for (const GrabbedConstPtr& pgrabbed : _vGrabbedBodies) {
        if( pgrabbed->_pGrabbingLink == plink ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                if( !linksaver ) {
                    linksaver.reset(new KinBodyStateSaver(shared_kinbody()));
                    plink->Enable(false);
                    // also disable rigidly attached links?
                }
                KinBodyStateSaver bodysaver(pgrabbedbody,Save_LinkTransformation);
                if( pchecker->CheckCollision(shared_kinbody_const(), KinBodyConstPtr(pgrabbedbody),report) ) {
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

bool KinBody::CheckLinkSelfCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report)
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
    if( plink->IsEnabled() ) {
        boost::shared_ptr<TransformSaver<LinkPtr> > linksaver(new TransformSaver<LinkPtr>(plink)); // gcc optimization bug when linksaver is on stack?
        plink->SetTransform(tlinktrans);
        if( pchecker->CheckStandaloneSelfCollision(LinkConstPtr(plink),report) ) {
            if( !bAllLinkCollisions ) { // if checking all collisions, have to continue
                return true;
            }
            bincollision = true;
        }
    }

    KinBodyStateSaverPtr linksaver;
    // check if any grabbed bodies are attached to this link, and if so check their collisions with the environment
    // it is important to make sure to add all other attached bodies in the ignored list!
    std::vector<KinBodyConstPtr> vbodyexcluded;
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for (const GrabbedConstPtr& pgrabbed : _vGrabbedBodies) {
        if( pgrabbed->_pGrabbingLink == plink ) {
            KinBodyPtr pgrabbedbody = pgrabbed->_pGrabbedBody.lock();
            if( !!pgrabbedbody ) {
                if( !linksaver ) {
                    linksaver.reset(new KinBodyStateSaver(shared_kinbody()));
                    plink->Enable(false);
                    // also disable rigidly attached links?
                }
                KinBodyStateSaver bodysaver(pgrabbedbody,Save_LinkTransformation);
                pgrabbedbody->SetTransform(tlinktrans * pgrabbed->_tRelative);
                if( pchecker->CheckCollision(shared_kinbody_const(), KinBodyConstPtr(pgrabbedbody),report) ) {
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

} // end namespace OpenRAVE
