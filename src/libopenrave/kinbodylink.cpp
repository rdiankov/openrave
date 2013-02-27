// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov (rosen.diankov@gmail.com)
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
#include <algorithm>

namespace OpenRAVE {

KinBody::LinkInfo::LinkInfo() : XMLReadable("link"), _mass(0), _bStatic(false), _bIsEnabled(true) {
}

KinBody::Link::Link(KinBodyPtr parent)
{
    _parent = parent;
    _index = -1;
}

KinBody::Link::~Link()
{
}


void KinBody::Link::Enable(bool bEnable)
{
    if( _info._bIsEnabled != bEnable ) {
        KinBodyPtr parent = GetParent();
        parent->_nNonAdjacentLinkCache &= ~AO_Enabled;
        _info._bIsEnabled = bEnable;
        GetParent()->_ParametersChanged(Prop_LinkEnable);
    }
}

bool KinBody::Link::IsEnabled() const
{
    return _info._bIsEnabled;
}

bool KinBody::Link::SetVisible(bool visible)
{
    bool bchanged = false;
    FOREACH(itgeom,_vGeometries) {
        if( (*itgeom)->_info._bVisible != visible ) {
            (*itgeom)->_info._bVisible = visible;
            bchanged = true;
        }
    }
    if( bchanged ) {
        GetParent()->_ParametersChanged(Prop_LinkDraw);
        return true;
    }
    return false;
}

bool KinBody::Link::IsVisible() const
{
    FOREACHC(itgeom,_vGeometries) {
        if( (*itgeom)->IsVisible() ) {
            return true;
        }
    }
    return false;
}

void KinBody::Link::GetParentLinks(std::vector< boost::shared_ptr<Link> >& vParentLinks) const
{
    KinBodyConstPtr parent(_parent);
    vParentLinks.resize(_vParentLinks.size());
    for(size_t i = 0; i < _vParentLinks.size(); ++i) {
        vParentLinks[i] = parent->GetLinks().at(_vParentLinks[i]);
    }
}

bool KinBody::Link::IsParentLink(boost::shared_ptr<Link const> plink) const
{
    return find(_vParentLinks.begin(),_vParentLinks.end(),plink->GetIndex()) != _vParentLinks.end();
}

/** _tMassFrame * PrincipalInertia * _tMassFrame.inverse()

    from openravepy.ikfast import *
    quat = [Symbol('q0'),Symbol('q1'),Symbol('q2'),Symbol('q3')]
    IKFastSolver.matrixFromQuat(quat)
    Inertia = eye(3)
    Inertia[0,0] = Symbol('i0'); Inertia[1,1] = Symbol('i1'); Inertia[2,2] = Symbol('i2')
    MM = M * Inertia * M.transpose()
 */
static TransformMatrix ComputeInertia(const Transform& tMassFrame, const Vector& vinertiamoments)
{
    TransformMatrix minertia;
    dReal i0 = vinertiamoments[0], i1 = vinertiamoments[1], i2 = vinertiamoments[2];
    dReal q0=tMassFrame.rot[0], q1=tMassFrame.rot[1], q2=tMassFrame.rot[2], q3=tMassFrame.rot[3];
    dReal q1_2 = q1*q1, q2_2 = q2*q2, q3_2 = q3*q3;
    minertia.m[0] = i0*utils::Sqr(1 - 2*q2_2 - 2*q3_2) + i1*utils::Sqr(-2*q0*q3 + 2*q1*q2) + i2*utils::Sqr(2*q0*q2 + 2*q1*q3);
    minertia.m[1] = i0*(2*q0*q3 + 2*q1*q2)*(1 - 2*q2_2 - 2*q3_2) + i1*(-2*q0*q3 + 2*q1*q2)*(1 - 2*q1_2 - 2*q3_2) + i2*(-2*q0*q1 + 2*q2*q3)*(2*q0*q2 + 2*q1*q3);
    minertia.m[2] = i0*(-2*q0*q2 + 2*q1*q3)*(1 - 2*q2_2 - 2*q3_2) + i1*(-2*q0*q3 + 2*q1*q2)*(2*q0*q1 + 2*q2*q3) + i2*(2*q0*q2 + 2*q1*q3)*(1 - 2*q1_2 - 2*q2_2);
    minertia.m[3] = 0;
    minertia.m[4] = minertia.m[1];
    minertia.m[5] = i0*utils::Sqr(2*q0*q3 + 2*q1*q2) + i1*utils::Sqr(1 - 2*q1_2 - 2*q3_2) + i2*utils::Sqr(-2*q0*q1 + 2*q2*q3);
    minertia.m[6] = i0*(-2*q0*q2 + 2*q1*q3)*(2*q0*q3 + 2*q1*q2) + i1*(2*q0*q1 + 2*q2*q3)*(1 - 2*q1_2 - 2*q3_2) + i2*(-2*q0*q1 + 2*q2*q3)*(1 - 2*q1_2 - 2*q2_2);
    minertia.m[7] = 0;
    minertia.m[8] = minertia.m[2];
    minertia.m[9] = minertia.m[6];
    minertia.m[10] = i0*utils::Sqr(-2*q0*q2 + 2*q1*q3) + i1*utils::Sqr(2*q0*q1 + 2*q2*q3) + i2*utils::Sqr(1 - 2*q1_2 - 2*q2_2);
    minertia.m[11] = 0;
    return minertia;
}
TransformMatrix KinBody::Link::GetLocalInertia() const
{
    return ComputeInertia(_info._tMassFrame, _info._vinertiamoments);
}

TransformMatrix KinBody::Link::GetGlobalInertia() const
{
    return ComputeInertia(_info._t*_info._tMassFrame, _info._vinertiamoments);
}

void KinBody::Link::SetLocalMassFrame(const Transform& massframe)
{
    _info._tMassFrame=massframe;
    GetParent()->_ParametersChanged(Prop_LinkDynamics);
}

void KinBody::Link::SetPrincipalMomentsOfInertia(const Vector& inertiamoments)
{
    _info._vinertiamoments = inertiamoments;
    GetParent()->_ParametersChanged(Prop_LinkDynamics);
}

void KinBody::Link::SetMass(dReal mass)
{
    _info._mass=mass;
    GetParent()->_ParametersChanged(Prop_LinkDynamics);
}

AABB KinBody::Link::ComputeLocalAABB() const
{
    if( _vGeometries.size() == 1) {
        return _vGeometries.front()->ComputeAABB(Transform());
    }
    else if( _vGeometries.size() > 1 ) {
        Vector vmin, vmax;
        bool binitialized=false;
        AABB ab;
        FOREACHC(itgeom,_vGeometries) {
            ab = (*itgeom)->ComputeAABB(Transform());
            if( ab.extents.x <= 0 || ab.extents.y <= 0 || ab.extents.z <= 0 ) {
                continue;
            }
            Vector vnmin = ab.pos - ab.extents;
            Vector vnmax = ab.pos + ab.extents;
            if( !binitialized ) {
                vmin = vnmin;
                vmax = vnmax;
                binitialized = true;
            }
            else {
                if( vmin.x > vnmin.x ) {
                    vmin.x = vnmin.x;
                }
                if( vmin.y > vnmin.y ) {
                    vmin.y = vnmin.y;
                }
                if( vmin.z > vnmin.z ) {
                    vmin.z = vnmin.z;
                }
                if( vmax.x < vnmax.x ) {
                    vmax.x = vnmax.x;
                }
                if( vmax.y < vnmax.y ) {
                    vmax.y = vnmax.y;
                }
                if( vmax.z < vnmax.z ) {
                    vmax.z = vnmax.z;
                }
            }
        }
        if( !binitialized ) {
            ab.pos = _info._t.trans;
            ab.extents = Vector(0,0,0);
        }
        else {
            ab.pos = (dReal)0.5 * (vmin + vmax);
            ab.extents = vmax - ab.pos;
        }
        return ab;
    }
    return AABB();
}

AABB KinBody::Link::ComputeAABB() const
{
    if( _vGeometries.size() == 1) {
        return _vGeometries.front()->ComputeAABB(_info._t);
    }
    else if( _vGeometries.size() > 1 ) {
        Vector vmin, vmax;
        bool binitialized=false;
        AABB ab;
        FOREACHC(itgeom,_vGeometries) {
            ab = (*itgeom)->ComputeAABB(_info._t);
            if( ab.extents.x <= 0 || ab.extents.y <= 0 || ab.extents.z <= 0 ) {
                continue;
            }
            Vector vnmin = ab.pos - ab.extents;
            Vector vnmax = ab.pos + ab.extents;
            if( !binitialized ) {
                vmin = vnmin;
                vmax = vnmax;
                binitialized = true;
            }
            else {
                if( vmin.x > vnmin.x ) {
                    vmin.x = vnmin.x;
                }
                if( vmin.y > vnmin.y ) {
                    vmin.y = vnmin.y;
                }
                if( vmin.z > vnmin.z ) {
                    vmin.z = vnmin.z;
                }
                if( vmax.x < vnmax.x ) {
                    vmax.x = vnmax.x;
                }
                if( vmax.y < vnmax.y ) {
                    vmax.y = vnmax.y;
                }
                if( vmax.z < vnmax.z ) {
                    vmax.z = vnmax.z;
                }
            }
        }
        if( !binitialized ) {
            ab.pos = _info._t.trans;
            ab.extents = Vector(0,0,0);
        }
        else {
            ab.pos = (dReal)0.5 * (vmin + vmax);
            ab.extents = vmax - ab.pos;
        }
        return ab;
    }
    // have to at least return the correct position!
    return AABB(_info._t.trans,Vector(0,0,0));
}

void KinBody::Link::serialize(std::ostream& o, int options) const
{
    o << _index << " ";
    if( options & SO_Geometry ) {
        o << _vGeometries.size() << " ";
        FOREACHC(it,_vGeometries) {
            (*it)->serialize(o,options);
        }
    }
    if( options & SO_Dynamics ) {
        SerializeRound(o,_info._tMassFrame);
        SerializeRound(o,_info._mass);
        SerializeRound3(o,_info._vinertiamoments);
    }
}

void KinBody::Link::SetStatic(bool bStatic)
{
    if( _info._bStatic != bStatic ) {
        _info._bStatic = bStatic;
        GetParent()->_ParametersChanged(Prop_LinkStatic);
    }
}

void KinBody::Link::SetTransform(const Transform& t)
{
    _info._t = t;
    GetParent()->_nUpdateStampId++;
}

void KinBody::Link::SetForce(const Vector& force, const Vector& pos, bool bAdd)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetBodyForce(shared_from_this(), force, pos, bAdd);
}

void KinBody::Link::SetTorque(const Vector& torque, bool bAdd)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetBodyTorque(shared_from_this(), torque, bAdd);
}

void KinBody::Link::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetLinkVelocity(shared_from_this(), linearvel, angularvel);
}

void KinBody::Link::GetVelocity(Vector& linearvel, Vector& angularvel) const
{
    GetParent()->GetEnv()->GetPhysicsEngine()->GetLinkVelocity(shared_from_this(), linearvel, angularvel);
}

/// \brief return the linear/angular velocity of the link
std::pair<Vector,Vector> KinBody::Link::GetVelocity() const
{
    std::pair<Vector,Vector> velocities;
    GetParent()->GetEnv()->GetPhysicsEngine()->GetLinkVelocity(shared_from_this(), velocities.first, velocities.second);
    return velocities;
}

KinBody::Link::GeometryPtr KinBody::Link::GetGeometry(int index)
{
    return _vGeometries.at(index);
}

void KinBody::Link::InitGeometries(std::vector<KinBody::GeometryInfoConstPtr>& geometries)
{
    _vGeometries.resize(geometries.size());
    for(size_t i = 0; i < geometries.size(); ++i) {
        _vGeometries[i].reset(new Geometry(shared_from_this(),*geometries[i]));
    }
    _Update();
    GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

void KinBody::Link::InitGeometries(std::list<KinBody::GeometryInfo>& geometries)
{
    _vGeometries.resize(geometries.size());
    size_t i = 0;
    FOREACH(itinfo,geometries) {
        _vGeometries[i].reset(new Geometry(shared_from_this(),*itinfo));
        ++i;
    }
    _Update();
    GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

void KinBody::Link::SwapGeometries(boost::shared_ptr<Link>& link)
{
    _vGeometries.swap(link->_vGeometries);
    FOREACH(itgeom,_vGeometries) {
        (*itgeom)->_parent = shared_from_this();
    }
    FOREACH(itgeom,link->_vGeometries) {
        (*itgeom)->_parent = link;
    }
    _Update();
    link->_Update();
    GetParent()->_ParametersChanged(Prop_LinkGeometry);
    link->GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

bool KinBody::Link::ValidateContactNormal(const Vector& position, Vector& normal) const
{
    if( _vGeometries.size() == 1) {
        return _vGeometries.front()->ValidateContactNormal(position,normal);
    }
    else if( _vGeometries.size() > 1 ) {
        RAVELOG_VERBOSE(str(boost::format("cannot validate normal when there is more than one geometry in link '%s(%d)' (do not know colliding geometry)")%_info._name%GetIndex()));
    }
    return false;
}

void KinBody::Link::GetRigidlyAttachedLinks(std::vector<boost::shared_ptr<Link> >& vattachedlinks) const
{
    KinBodyPtr parent(_parent);
    vattachedlinks.resize(0);
    FOREACHC(it, _vRigidlyAttachedLinks) {
        vattachedlinks.push_back(parent->GetLinks().at(*it));
    }
}

void KinBody::Link::SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters)
{
    if( parameters.size() > 0 ) {
        _info._mapFloatParameters[key] = parameters;
    }
    else {
        _info._mapFloatParameters.erase(key);
    }
    GetParent()->_ParametersChanged(Prop_LinkCustomParameters);
}

void KinBody::Link::SetIntParameters(const std::string& key, const std::vector<int>& parameters)
{
    if( parameters.size() > 0 ) {
        _info._mapIntParameters[key] = parameters;
    }
    else {
        _info._mapIntParameters.erase(key);
    }
    GetParent()->_ParametersChanged(Prop_LinkCustomParameters);
}

bool KinBody::Link::IsRigidlyAttached(boost::shared_ptr<Link const> plink) const
{
    return find(_vRigidlyAttachedLinks.begin(),_vRigidlyAttachedLinks.end(),plink->GetIndex()) != _vRigidlyAttachedLinks.end();
}

void KinBody::Link::UpdateInfo()
{
    if( _info._vgeometryinfos.size() != _vGeometries.size() ) {
        // have to recompute the geometries
        _info._vgeometryinfos.resize(_vGeometries.size());
        for(size_t i = 0; i < _info._vgeometryinfos.size(); ++i) {
            if( !_info._vgeometryinfos[i] ) {
                _info._vgeometryinfos[i].reset(new KinBody::GeometryInfo());
            }
            *_info._vgeometryinfos[i] = _vGeometries[i]->GetInfo();
        }
    }
}

void KinBody::Link::_Update()
{
    _collision.vertices.resize(0);
    _collision.indices.resize(0);
    FOREACH(itgeom,_vGeometries) {
        _collision.Append((*itgeom)->GetCollisionMesh(),(*itgeom)->GetTransform());
    }
    GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

}
