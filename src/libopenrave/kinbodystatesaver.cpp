// -*- coding: utf-8 -*-
// Copyright (C) 2006-2019 Rosen Diankov (rosen.diankov@gmail.com)
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

/// \brief push link to _listNonCollidingLinksWhenGrabbed of grabbed.
static void _PushLinkToListNonCollidingLinksWhenGrabbed(Grabbed& grabbed,
                                                        const int linkindex,
                                                        const KinBody::LinkConstPtr& pLinkRef,
                                                        const std::vector<KinBody::LinkPtr>& vLinks,
                                                        const EnvironmentBasePtr& pEnv)
{
    if( linkindex < 0 || linkindex >= (int)vLinks.size() ) {
        RAVELOG_WARN_FORMAT("env=%s, could not restore link '%s' since its index %d is out of range (body num links is %d)", pEnv->GetNameId()%pLinkRef->GetName()%pLinkRef->GetIndex()%vLinks.size());
    }
    else {
        grabbed._listNonCollidingLinksWhenGrabbed.push_back(vLinks.at(linkindex));
    }
}

void KinBody::_RestoreGrabbedBodiesFromSavedData(const KinBody& savedBody,
                                                 const int options,
                                                 const std::unordered_map<int, GrabbedPtr>& savedGrabbedBodiesByEnvironmentIndex)
{
    {
        // have to release all grabbed first
        ReleaseAllGrabbed();
        OPENRAVE_ASSERT_OP(_grabbedBodiesByEnvironmentIndex.size(),==,0);
        for (const std::unordered_map<int, GrabbedPtr>::value_type& grabPair : savedGrabbedBodiesByEnvironmentIndex) {
            const GrabbedPtr& pGrabbed = grabPair.second;
            KinBodyPtr pGrabbedBody = pGrabbed->_pGrabbedBody.lock();
            if( !!pGrabbedBody ) {
                KinBody::LinkPtr pGrabbingLink(pGrabbed->_pGrabbingLink);
                if( !pGrabbingLink ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, could not find grabbing link for grabbed body '%s'"),
                                                    GetEnv()->GetNameId()%pGrabbedBody->GetName(),
                                                    ORE_Failed);
                }
                else if( !GetLink(pGrabbingLink->GetName()) ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, could not find grabbing link '%s' for grabbed body '%s' since %s does not have the link (body num links is %d)"),
                                                    GetEnv()->GetNameId()%pGrabbingLink->GetName()%pGrabbedBody->GetName()%GetName()%GetLinks().size(),
                                                    ORE_Failed);
                }
                if( GetEnv() == savedBody.GetEnv() ) {
                    // attach and add
                    _AttachBody(pGrabbedBody);
                    BOOST_ASSERT(pGrabbedBody->GetEnvironmentBodyIndex() > 0);
                    _grabbedBodiesByEnvironmentIndex[pGrabbedBody->GetEnvironmentBodyIndex()] = pGrabbed;

                    // grabbed bodies could have been removed from env and self collision checker.
                    CollisionCheckerBasePtr collisionchecker = GetSelfCollisionChecker();
                    if (!!collisionchecker) {
                        collisionchecker->InitKinBody(pGrabbedBody);
                    }
                }
                else {
                    // The body that the state was saved from is from a different environment from savedBody. This case can
                    // happen when cloning an environment (see Environment::_Clone). Since cloning is supposed to
                    // preserve environmentBodyIndex of bodies, it is ok do the following.

                    KinBodyPtr pNewGrabbedBody = GetEnv()->GetBodyFromEnvironmentBodyIndex(pGrabbedBody->GetEnvironmentBodyIndex());
                    if( !!pNewGrabbedBody ) {
                        if( pGrabbedBody->GetKinematicsGeometryHash() != pNewGrabbedBody->GetKinematicsGeometryHash() ) {
                            RAVELOG_WARN_FORMAT("env=%s, new grabbed body '%s' kinematics-geometry hash is different from original grabbed body '%s' from env=%s", GetEnv()->GetNameId()%pNewGrabbedBody->GetName()%pGrabbedBody->GetName()%savedBody.GetEnv()->GetNameId());
                        }
                        else {
                            // body is supposed to already be set to some "proper" configuration as the newly
                            // initialized Grabbed objects will save the current state of pbody for later computation of
                            // _listNonCollidingLinksWhenGrabbed (in case it is not yet computed).
                            KinBody::LinkPtr pNewGrabbingLink = GetLinks().at(KinBody::LinkPtr(pGrabbingLink)->GetIndex());
                            GrabbedPtr pNewGrabbed(new Grabbed(pNewGrabbedBody, pNewGrabbingLink));
                            pNewGrabbed->_tRelative = pGrabbed->_tRelative;
                            pNewGrabbed->_setGrabberLinkIndicesToIgnore = pGrabbed->_setGrabberLinkIndicesToIgnore;
                            if( pGrabbed->IsListNonCollidingLinksValid() ) {
                                FOREACHC(itLinkRef, pGrabbed->_listNonCollidingLinksWhenGrabbed) {
                                    const KinBodyPtr pParent = (*itLinkRef)->GetParent();
                                    if( !pParent ) {
                                        RAVELOG_WARN_FORMAT("env=%s, could not restore link '%s' since parent is not found.", GetEnv()->GetNameId()%(*itLinkRef)->GetName());
                                        continue;
                                    }
                                    const int linkindex = (*itLinkRef)->GetIndex();
                                    if( pParent.get() == this ) {
                                        _PushLinkToListNonCollidingLinksWhenGrabbed(*pNewGrabbed, linkindex, *itLinkRef, GetLinks(), GetEnv());
                                    }
                                    else {
                                        const KinBodyPtr pNewNonCollidingBody = GetEnv()->GetBodyFromEnvironmentBodyIndex(pParent->GetEnvironmentBodyIndex());
                                        if( !pNewNonCollidingBody) {
                                            RAVELOG_WARN_FORMAT("env=%s, could not restore link '%s' since could not not find body with id %d.", GetEnv()->GetNameId()%(*itLinkRef)->GetName() % pParent->GetEnvironmentBodyIndex());
                                            continue;
                                        }
                                        _PushLinkToListNonCollidingLinksWhenGrabbed(*pNewGrabbed, linkindex, *itLinkRef, pNewNonCollidingBody->GetLinks(), GetEnv());
                                    }
                                }
                                pNewGrabbed->_SetLinkNonCollidingIsValid(true);
                            }
                            CopyRapidJsonDoc(pGrabbed->_rGrabbedUserData, pNewGrabbed->_rGrabbedUserData);

                            _AttachBody(pNewGrabbedBody);
                            BOOST_ASSERT(pNewGrabbedBody->GetEnvironmentBodyIndex() > 0);
                            _grabbedBodiesByEnvironmentIndex[pNewGrabbedBody->GetEnvironmentBodyIndex()] = pNewGrabbed;
                            CollisionCheckerBasePtr collisionchecker = GetSelfCollisionChecker();
                            if (!!collisionchecker) {
                                collisionchecker->InitKinBody(pNewGrabbedBody);
                            }
                        }
                    }
                    else {
                        RAVELOG_WARN_FORMAT("env=%s, could not find body with id %d (body '%s' from env=%s)", GetEnv()->GetNameId()%pGrabbedBody->GetEnvironmentBodyIndex()%pGrabbedBody->GetName()%savedBody.GetEnv()->GetNameId());
                    } // end if !!pNewGrabbedBody
                }
            }
        }

        // if not calling SetLinkTransformations, then manually call _UpdateGrabbedBodies
        if( !(options & KinBody::Save_LinkTransformation ) ) {
            _UpdateGrabbedBodies();
        }
    }
}

KinBody::KinBodyStateSaver::KinBodyStateSaver(KinBodyPtr pbody, int options) : _pbody(pbody), _options(options), _bRestoreOnDestructor(true)
{
    if( _options & Save_LinkTransformation ) {
        _pbody->GetLinkTransformations(_vLinkTransforms, _vdoflastsetvalues);
    }
    if( _options & Save_LinkEnable ) {
        _vEnabledLinks.resize(_pbody->GetLinks().size());
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            _vEnabledLinks[i] = _pbody->GetLinks().at(i)->IsEnabled();
        }
    }
    if( _options & Save_LinkVelocities ) {
        _pbody->GetLinkVelocities(_vLinkVelocities);
    }
    if( _options & Save_JointMaxVelocityAndAcceleration ) {
        _pbody->GetDOFVelocityLimits(_vMaxVelocities);
        _pbody->GetDOFAccelerationLimits(_vMaxAccelerations);
        _pbody->GetDOFJerkLimits(_vMaxJerks);
    }
    if( _options & Save_JointWeights ) {
        _pbody->GetDOFWeights(_vDOFWeights);
    }
    if( _options & Save_JointLimits ) {
        _pbody->GetDOFLimits(_vDOFLimits[0], _vDOFLimits[1]);
    }
    if( _options & Save_JointResolutions ) {
        _pbody->GetDOFResolutions(_vDOFResolutions);
    }
    if( _options & Save_GrabbedBodies ) {
        _grabbedBodiesByEnvironmentIndex = _pbody->_grabbedBodiesByEnvironmentIndex;
    }
}

KinBody::KinBodyStateSaver::~KinBodyStateSaver()
{
    if( _bRestoreOnDestructor && !!_pbody && _pbody->GetEnvironmentBodyIndex() != 0 ) {
        _RestoreKinBody(_pbody);
    }
}

void KinBody::KinBodyStateSaver::Restore(boost::shared_ptr<KinBody> body)
{
    _RestoreKinBody(!body ? _pbody : body);
}

void KinBody::KinBodyStateSaver::Release()
{
    _pbody.reset();
}

void KinBody::KinBodyStateSaver::SetRestoreOnDestructor(bool restore)
{
    _bRestoreOnDestructor = restore;
}

void KinBody::KinBodyStateSaver::_RestoreKinBody(boost::shared_ptr<KinBody> pbody)
{
    if( !pbody ) {
        return;
    }
    if( pbody->GetEnvironmentBodyIndex() == 0 ) {
        RAVELOG_WARN_FORMAT("env=%d, body %s not added to environment, skipping restore", pbody->GetEnv()->GetId()%pbody->GetName());
        return;
    }
    if( _options & Save_JointLimits ) {
        pbody->SetDOFLimits(_vDOFLimits[0], _vDOFLimits[1]);
    }
    // restoring grabbed bodies has to happen first before link transforms can be restored since _UpdateGrabbedBodies can be called with the old grabbed bodies.
    if( _options & Save_GrabbedBodies ) {
        pbody->_RestoreGrabbedBodiesFromSavedData(*_pbody, _options, _grabbedBodiesByEnvironmentIndex);
    }
    if( _options & Save_LinkTransformation ) {
        pbody->SetLinkTransformations(_vLinkTransforms, _vdoflastsetvalues);
//        if( IS_DEBUGLEVEL(Level_Warn) ) {
//            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
//            ss << "restoring kinbody " << pbody->GetName() << " to values=[";
//            std::vector<dReal> values;
//            pbody->GetDOFValues(values);
//            FOREACH(it,values) {
//                ss << *it << ", ";
//            }
//            ss << "]";
//            RAVELOG_WARN(ss.str());
//        }
    }
    if( _options & Save_LinkEnable ) {
        // should first enable before calling the parameter callbacks
        bool bchanged = false;
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            if( pbody->GetLinks().at(i)->IsEnabled() != !!_vEnabledLinks[i] ) {
                pbody->GetLinks().at(i)->_Enable(!!_vEnabledLinks[i]);
                bchanged = true;
            }
        }
        if( bchanged ) {
            pbody->_nNonAdjacentLinkCache &= ~AO_Enabled;
            pbody->_PostprocessChangedParameters(Prop_LinkEnable);
        }
    }
    if( _options & Save_JointMaxVelocityAndAcceleration ) {
        pbody->SetDOFVelocityLimits(_vMaxVelocities);
        pbody->SetDOFAccelerationLimits(_vMaxAccelerations);
        pbody->SetDOFJerkLimits(_vMaxJerks);
    }
    if( _options & Save_LinkVelocities ) {
        pbody->SetLinkVelocities(_vLinkVelocities);
    }
    if( _options & Save_JointWeights ) {
        pbody->SetDOFWeights(_vDOFWeights);
    }
    if( _options & Save_JointResolutions ) {
        pbody->SetDOFResolutions(_vDOFResolutions);
    }
}


KinBody::KinBodyStateSaverRef::KinBodyStateSaverRef(KinBody& body, int options) : _body(body), _options(options), _bRestoreOnDestructor(true), _bReleased(false)
{
    if( _options & Save_LinkTransformation ) {
        body.GetLinkTransformations(_vLinkTransforms, _vdoflastsetvalues);
    }
    if( _options & Save_LinkEnable ) {
        _vEnabledLinks.resize(body.GetLinks().size());
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            _vEnabledLinks[i] = body.GetLinks().at(i)->IsEnabled();
        }
    }
    if( _options & Save_LinkVelocities ) {
        body.GetLinkVelocities(_vLinkVelocities);
    }
    if( _options & Save_JointMaxVelocityAndAcceleration ) {
        body.GetDOFVelocityLimits(_vMaxVelocities);
        body.GetDOFAccelerationLimits(_vMaxAccelerations);
        body.GetDOFJerkLimits(_vMaxJerks);
    }
    if( _options & Save_JointWeights ) {
        body.GetDOFWeights(_vDOFWeights);
    }
    if( _options & Save_JointLimits ) {
        body.GetDOFLimits(_vDOFLimits[0], _vDOFLimits[1]);
    }
    if( _options & Save_GrabbedBodies ) {
        _grabbedBodiesByEnvironmentIndex = body._grabbedBodiesByEnvironmentIndex;
    }
    if( _options & Save_JointResolutions ) {
        body.GetDOFResolutions(_vDOFResolutions);
    }
}

KinBody::KinBodyStateSaverRef::~KinBodyStateSaverRef()
{
    if( _bRestoreOnDestructor && !_bReleased && _body.GetEnvironmentBodyIndex() != 0 ) {
        _RestoreKinBody(_body);
    }
}

void KinBody::KinBodyStateSaverRef::Restore()
{
    if( !_bReleased ) {
        _RestoreKinBody(_body);
    }
}

void KinBody::KinBodyStateSaverRef::Restore(KinBody& body)
{
    _RestoreKinBody(body);
}

void KinBody::KinBodyStateSaverRef::Release()
{
    _bReleased = true;
}

void KinBody::KinBodyStateSaverRef::SetRestoreOnDestructor(bool restore)
{
    _bRestoreOnDestructor = restore;
}

void KinBody::KinBodyStateSaverRef::_RestoreKinBody(KinBody& body)
{
    if( body.GetEnvironmentBodyIndex() == 0 ) {
        RAVELOG_WARN(str(boost::format("body %s not added to environment, skipping restore")%body.GetName()));
        return;
    }
    if( _options & Save_JointLimits ) {
        body.SetDOFLimits(_vDOFLimits[0], _vDOFLimits[1]);
    }
    // restoring grabbed bodies has to happen first before link transforms can be restored since _UpdateGrabbedBodies can be called with the old grabbed bodies.
    if( _options & Save_GrabbedBodies ) {
        body._RestoreGrabbedBodiesFromSavedData(_body, _options, _grabbedBodiesByEnvironmentIndex);
    }
    if( _options & Save_LinkTransformation ) {
        body.SetLinkTransformations(_vLinkTransforms, _vdoflastsetvalues);
//        if( IS_DEBUGLEVEL(Level_Warn) ) {
//            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
//            ss << "restoring kinbody " << body.GetName() << " to values=[";
//            std::vector<dReal> values;
//            body.GetDOFValues(values);
//            FOREACH(it,values) {
//                ss << *it << ", ";
//            }
//            ss << "]";
//            RAVELOG_WARN(ss.str());
//        }
    }
    if( _options & Save_LinkEnable ) {
        // should first enable before calling the parameter callbacks
        bool bchanged = false;
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            if( body.GetLinks().at(i)->IsEnabled() != !!_vEnabledLinks[i] ) {
                body.GetLinks().at(i)->_Enable(!!_vEnabledLinks[i]);
                bchanged = true;
            }
        }
        if( bchanged ) {
            body._nNonAdjacentLinkCache &= ~AO_Enabled;
            body._PostprocessChangedParameters(Prop_LinkEnable);
        }
    }
    if( _options & Save_JointMaxVelocityAndAcceleration ) {
        body.SetDOFVelocityLimits(_vMaxVelocities);
        body.SetDOFAccelerationLimits(_vMaxAccelerations);
        body.SetDOFJerkLimits(_vMaxJerks);
    }
    if( _options & Save_LinkVelocities ) {
        body.SetLinkVelocities(_vLinkVelocities);
    }
    if( _options & Save_JointWeights ) {
        body.SetDOFWeights(_vDOFWeights);
    }
    if( _options & Save_JointResolutions ) {
        body.SetDOFResolutions(_vDOFResolutions);
    }
}

} // end namespace OpenRAVE
