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
        _vGrabbedBodies = _pbody->_vGrabbedBodies;
        if( _pbody->IsRobot() ) {
            // For now, do not allow a robot's connected body active states to change while the robot is grabbing
            // something. The change in connected body active states essentially affect the robot's link indices, which
            // will then mess up Grabbed's setGrabberLinksToIgnore.
            RobotBasePtr pRobot = RaveInterfaceCast<RobotBase>(_pbody);
            pRobot->GetConnectedBodyActiveStates(_vConnectedBodyActiveStates);
        }
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
        if( pbody->IsRobot() ) {
            RobotBasePtr pRobot = RaveInterfaceCast<RobotBase>(pbody);
            std::vector<int8_t> vCurrentConnectedBodyActiveStates;
            pRobot->GetConnectedBodyActiveStates(vCurrentConnectedBodyActiveStates);

            size_t numExpectedConnectedBodies = _vConnectedBodyActiveStates.size();
            OPENRAVE_ASSERT_FORMAT(numExpectedConnectedBodies == vCurrentConnectedBodyActiveStates.size(), "env=%s, body '%s' now has %d connected bodies. expected=%d (env=%s)", pbody->GetEnv()->GetNameId()%pbody->GetName()%vCurrentConnectedBodyActiveStates.size()%numExpectedConnectedBodies%_pbody->GetEnv()->GetNameId(), ORE_InvalidState);

            for( size_t iConnectedBody = 0; iConnectedBody < numExpectedConnectedBodies; ++iConnectedBody ) {
                OPENRAVE_ASSERT_FORMAT(_vConnectedBodyActiveStates[iConnectedBody] == vCurrentConnectedBodyActiveStates[iConnectedBody], "env=%s, body '%s' connected body %d/%d active state=%d not expected", pbody->GetEnv()->GetNameId()%pbody->GetName()%iConnectedBody%numExpectedConnectedBodies%int(vCurrentConnectedBodyActiveStates[iConnectedBody]), ORE_InvalidState);
            }
        }

        // have to release all grabbed first
        pbody->ReleaseAllGrabbed();
        OPENRAVE_ASSERT_OP(pbody->_vGrabbedBodies.size(),==,0);
        FOREACH(itgrabbed, _vGrabbedBodies) {
            GrabbedPtr pgrabbed = *itgrabbed;
            KinBodyPtr pbodygrab = pgrabbed->_pGrabbedBody.lock();
            if( !!pbodygrab ) {
                if( pbody->GetEnv() == _pbody->GetEnv() ) {
                    pbody->_AttachBody(pbodygrab);
                    pbody->_vGrabbedBodies.push_back(*itgrabbed);
                }
                else {
                    RAVELOG_WARN("Not implemented yet");
#if 0
                    // pgrabbed points to a different environment, so have to re-initialize
                    KinBodyPtr pnewbody = pbody->GetEnv()->GetBodyFromEnvironmentBodyIndex(pbodygrab->GetEnvironmentBodyIndex());
                    if( !!pnewbody ) {
                        if( pbodygrab->GetKinematicsGeometryHash() != pnewbody->GetKinematicsGeometryHash() ) {
                            RAVELOG_WARN_FORMAT("env=%d, body %s is not similar across environments", pbody->GetEnv()->GetId()%pbodygrab->GetName());
                        }
                        else {
                            GrabbedPtr pnewgrabbed(new Grabbed(pnewbody,pbody->GetLinks().at(KinBody::LinkPtr(pgrabbed->_plinkrobot)->GetIndex())));
                            pnewgrabbed->_troot = pgrabbed->_troot;
                            pnewgrabbed->_listNonCollidingLinks.clear();
                            FOREACHC(itlinkref, pgrabbed->_listNonCollidingLinks) {
                                pnewgrabbed->_listNonCollidingLinks.push_back(pbody->GetLinks().at((*itlinkref)->GetIndex()));
                            }
                            pbody->_AttachBody(pnewbody);
                            pbody->_vGrabbedBodies.push_back(pnewgrabbed);
                        }
                    }
                    else {
                        RAVELOG_WARN_FORMAT("env=%d, could not find body %s with id %d", pbody->GetEnv()->GetId()%pbodygrab->GetName()%pbodygrab->GetEnvironmentBodyIndex());
                    }
#endif
                }
            }
        }

        // if not calling SetLinkTransformations, then manually call _UpdateGrabbedBodies
        if( !(_options & Save_LinkTransformation ) ) {
            pbody->_UpdateGrabbedBodies();
        }
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
        _vGrabbedBodies = body._vGrabbedBodies;
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
        // have to release all grabbed first
        body.ReleaseAllGrabbed();
        OPENRAVE_ASSERT_OP(body._vGrabbedBodies.size(),==,0);
        FOREACH(itgrabbed, _vGrabbedBodies) {
            GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
            KinBodyPtr pbodygrab = pgrabbed->_pGrabbedBody.lock();
            if( !!pbodygrab ) {
                if( body.GetEnv() == body.GetEnv() ) {
                    body._AttachBody(pbodygrab);
                    body._vGrabbedBodies.push_back(*itgrabbed);
                }
                else {
                    RAVELOG_WARN("Not implemented yet");
#if 0
                    // pgrabbed points to a different environment, so have to re-initialize
                    KinBodyPtr pnewbody = body.GetEnv()->GetBodyFromEnvironmentBodyIndex(pbodygrab->GetEnvironmentBodyIndex());
                    if( pbodygrab->GetKinematicsGeometryHash() != pnewbody->GetKinematicsGeometryHash() ) {
                        RAVELOG_WARN(str(boost::format("body %s is not similar across environments")%pbodygrab->GetName()));
                    }
                    else {
                        GrabbedPtr pnewgrabbed(new Grabbed(pnewbody,body.GetLinks().at(KinBody::LinkPtr(pgrabbed->_plinkrobot)->GetIndex())));
                        pnewgrabbed->_troot = pgrabbed->_troot;
                        pnewgrabbed->_listNonCollidingLinks.clear();
                        FOREACHC(itlinkref, pgrabbed->_listNonCollidingLinks) {
                            pnewgrabbed->_listNonCollidingLinks.push_back(body.GetLinks().at((*itlinkref)->GetIndex()));
                        }
                        body._AttachBody(pnewbody);
                        body._vGrabbedBodies.push_back(pnewgrabbed);
                    }
#endif
                }
            }
        }

        // if not calling SetLinkTransformations, then manually call _UpdateGrabbedBodies
        if( !(_options & Save_LinkTransformation ) ) {
            body._UpdateGrabbedBodies();
        }
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
