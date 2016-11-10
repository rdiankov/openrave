// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov (rosen.diankov@gmail.com)
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

// the following constructor handles mapping from deprecated reference to the actual
// member, so need to disable deprecation warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
KinBody::LinkInfo::LinkInfo() :
    XMLReadable("link"),
    _vgeometryinfos(geometries),
    _mapExtraGeometries(extraGeometries),
    _name(name),
    _t(transform),
    _tMassFrame(massTransform),
    _mass(mass),
    _vinertiamoments(inertiaMoments),
    _mapFloatParameters(floatParameters),
    _mapIntParameters(intParameters),
    _mapStringParameters(stringParameters),
    _vForcedAdjacentLinks(forcedAdjacentLinks),
    _bStatic(isStatic),
    _bIsEnabled(isEnabled)
{
    isStatic = false;
    isEnabled = true;
}
#pragma GCC diagnostic pop

KinBody::LinkInfo::LinkInfo(const KinBody::LinkInfo& other) : LinkInfo()
{
    *this = other;
}

KinBody::LinkInfo::~LinkInfo()
{
}

KinBody::LinkInfo& KinBody::LinkInfo::operator=(const KinBody::LinkInfo& other)
{
    sid = other.sid;
    geometries = other.geometries;
    extraGeometries = other.extraGeometries;
    name = other.name;
    transform = other.transform;
    massTransform = other.massTransform;
    mass = other.mass;
    inertiaMoments = other.inertiaMoments;
    floatParameters = other.floatParameters;
    intParameters = other.intParameters;
    stringParameters = other.stringParameters;
    forcedAdjacentLinks = other.forcedAdjacentLinks;
    isStatic = other.isStatic;
    isEnabled = other.isEnabled;
    return *this;
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
    if( _info.isEnabled != bEnable ) {
        KinBodyPtr parent = GetParent();
        parent->_nNonAdjacentLinkCache &= ~AO_Enabled;
        _info.isEnabled = bEnable;
        GetParent()->_PostprocessChangedParameters(Prop_LinkEnable);
    }
}

bool KinBody::Link::IsEnabled() const
{
    return _info.isEnabled;
}

bool KinBody::Link::SetVisible(bool visible)
{
    bool bchanged = false;
    FOREACH(itgeom,_vGeometries) {
        if( (*itgeom)->_info.visible != visible ) {
            (*itgeom)->_info.visible = visible;
            bchanged = true;
        }
    }
    if( bchanged ) {
        GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
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
    return ComputeInertia(_info.massTransform, _info.inertiaMoments);
}

TransformMatrix KinBody::Link::GetGlobalInertia() const
{
    return ComputeInertia(_info.transform*_info.massTransform, _info.inertiaMoments);
}

void KinBody::Link::SetLocalMassFrame(const Transform& massframe)
{
    _info.massTransform=massframe;
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
}

void KinBody::Link::SetPrincipalMomentsOfInertia(const Vector& inertiamoments)
{
    _info.inertiaMoments = inertiamoments;
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
}

void KinBody::Link::SetMass(dReal mass)
{
    _info.mass=mass;
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
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
            ab.pos = _info.transform.trans;
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
        return _vGeometries.front()->ComputeAABB(_info.transform);
    }
    else if( _vGeometries.size() > 1 ) {
        Vector vmin, vmax;
        bool binitialized=false;
        AABB ab;
        FOREACHC(itgeom,_vGeometries) {
            ab = (*itgeom)->ComputeAABB(_info.transform);
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
            ab.pos = _info.transform.trans;
            ab.extents = Vector(0,0,0);
        }
        else {
            ab.pos = (dReal)0.5 * (vmin + vmax);
            ab.extents = vmax - ab.pos;
        }
        return ab;
    }
    // have to at least return the correct position!
    return AABB(_info.transform.trans,Vector(0,0,0));
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
        SerializeRound(o,_info.massTransform);
        SerializeRound(o,_info.mass);
        SerializeRound3(o,_info.inertiaMoments);
    }
}

void KinBody::LinkInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, int options)
{
    RAVE_SERIALIZEJSON_ENSURE_OBJECT(value);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "sid", sid);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "name", name);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "transform", transform);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "massTransform", massTransform);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "mass", mass);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "inertiaMoments", inertiaMoments);

    if (floatParameters.size() > 0) {
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "floatParameters", floatParameters);
    }

    if (intParameters.size() > 0) {
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "intParameters", intParameters);
    }

    if (stringParameters.size() > 0) {
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "stringParameters", stringParameters);
    }

    if (forcedAdjacentLinks.size() > 0) {
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "forcedAdjacentLinks", forcedAdjacentLinks);
    }

    if (geometries.size() > 0) {
        rapidjson::Value geometriesValue;
        RAVE_SERIALIZEJSON_CLEAR_ARRAY(geometriesValue);
        FOREACHC(it, geometries) {
            rapidjson::Value geometryValue;
            (*it)->SerializeJSON(geometryValue, allocator, options);
            geometriesValue.PushBack(geometryValue, allocator);
        }
        value.AddMember("geometries", geometriesValue, allocator);
    }

    // TODO(jsonserialization)
    // extraGeometries

    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "isStatic", isStatic);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "isEnabled", isEnabled);
}

void KinBody::LinkInfo::DeserializeJSON(const rapidjson::Value &value, const dReal fUnitScale)
{
    RAVE_DESERIALIZEJSON_ENSURE_OBJECT(value);

    RAVE_DESERIALIZEJSON_REQUIRED(value, "sid", sid);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "name", name);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "transform", transform);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "massTransform", massTransform);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "mass", mass);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "inertiaMoments", inertiaMoments);
    RAVE_DESERIALIZEJSON_OPTIONAL(value, "floatParameters", floatParameters);
    RAVE_DESERIALIZEJSON_OPTIONAL(value, "intParameters", intParameters);
    RAVE_DESERIALIZEJSON_OPTIONAL(value, "stringParameters", stringParameters);
    RAVE_DESERIALIZEJSON_OPTIONAL(value, "forcedAdjacentLinks", forcedAdjacentLinks);


    transform.trans *= fUnitScale;
    massTransform.trans *= fUnitScale;

    if (value.HasMember("geometries")) {
        RAVE_DESERIALIZEJSON_ENSURE_ARRAY(value["geometries"]);

        geometries.resize(0);
        geometries.reserve(value["geometries"].Size());
        for (size_t i = 0; i < value["geometries"].Size(); ++i) {
            GeometryInfoPtr geometry(new GeometryInfo());
            geometry->DeserializeJSON(value["geometries"][i], fUnitScale);
            geometries.push_back(geometry);
        }
    }

    // TODO(jsonserialization)
    // extraGeometries

    RAVE_DESERIALIZEJSON_REQUIRED(value, "isStatic", isStatic);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "isEnabled", isEnabled);
}

void KinBody::Link::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, int options)
{
    UpdateInfo();
    _info.SerializeJSON(value, allocator, options);
}

void KinBody::Link::DeserializeJSON(const rapidjson::Value &value, const dReal fUnitScale)
{
    _info.DeserializeJSON(value, fUnitScale);
}

void KinBody::Link::SetStatic(bool bStatic)
{
    if( _info.isStatic != bStatic ) {
        _info.isStatic = bStatic;
        GetParent()->_PostprocessChangedParameters(Prop_LinkStatic);
    }
}

void KinBody::Link::SetTransform(const Transform& t)
{
    _info.transform = t;
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
        if( _vGeometries[i]->GetCollisionMesh().vertices.size() == 0 ) {
            RAVELOG_VERBOSE("geometry has empty collision mesh\n");
            _vGeometries[i]->InitCollisionMesh(); // have to initialize the mesh since some plugins might not understand all geometry types
        }
    }
    _info.extraGeometries.clear();
    // have to reset the self group! cannot use geometries directly since we require exclusive access to the GeometryInfo objects
    std::vector<KinBody::GeometryInfoPtr> vgeometryinfos;
    vgeometryinfos.resize(_vGeometries.size());
    for(size_t i = 0; i < vgeometryinfos.size(); ++i) {
        vgeometryinfos[i].reset(new KinBody::GeometryInfo());
        *vgeometryinfos[i] = _vGeometries[i]->_info;
    }
    SetGroupGeometries("self", vgeometryinfos);
    _Update();
}

void KinBody::Link::InitGeometries(std::list<KinBody::GeometryInfo>& geometries)
{
    _vGeometries.resize(geometries.size());
    size_t i = 0;
    FOREACH(itinfo,geometries) {
        _vGeometries[i].reset(new Geometry(shared_from_this(),*itinfo));
        if( _vGeometries[i]->GetCollisionMesh().vertices.size() == 0 ) {
            RAVELOG_VERBOSE("geometry has empty collision mesh\n");
            _vGeometries[i]->InitCollisionMesh(); // have to initialize the mesh since some plugins might not understand all geometry types
        }
        ++i;
    }
    _info.extraGeometries.clear();
    // have to reset the self group!
    std::vector<KinBody::GeometryInfoPtr> vgeometryinfos;
    vgeometryinfos.resize(_vGeometries.size());
    for(size_t i = 0; i < vgeometryinfos.size(); ++i) {
        vgeometryinfos[i].reset(new KinBody::GeometryInfo());
        *vgeometryinfos[i] = _vGeometries[i]->_info;
    }
    SetGroupGeometries("self", vgeometryinfos);
    _Update();
}

void KinBody::Link::SetGeometriesFromGroup(const std::string& groupname)
{
    std::vector<KinBody::GeometryInfoPtr>* pvinfos = NULL;
    if( groupname.size() == 0 ) {
        pvinfos = &_info.geometries;
    }
    else {
        std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = _info.extraGeometries.find(groupname);
        if( it == _info.extraGeometries.end() ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("could not find geometries %s for link %s"),groupname%GetName(),ORE_InvalidArguments);
        }
        pvinfos = &it->second;
    }
    _vGeometries.resize(pvinfos->size());
    for(size_t i = 0; i < pvinfos->size(); ++i) {
        _vGeometries[i].reset(new Geometry(shared_from_this(),*pvinfos->at(i)));
        if( _vGeometries[i]->GetCollisionMesh().vertices.size() == 0 ) {
            RAVELOG_VERBOSE("geometry has empty collision mesh\n");
            _vGeometries[i]->InitCollisionMesh();
        }
    }
    _Update();
}

const std::vector<KinBody::GeometryInfoPtr>& KinBody::Link::GetGeometriesFromGroup(const std::string& groupname) const
{
    std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::const_iterator it = _info.extraGeometries.find(groupname);
    if( it == _info.extraGeometries.end() ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("geometry group %s does not exist for link %s"), groupname%GetName(), ORE_InvalidArguments);
    }
    return it->second;
}

void KinBody::Link::SetGroupGeometries(const std::string& groupname, const std::vector<KinBody::GeometryInfoPtr>& geometries)
{
    std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = _info.extraGeometries.insert(make_pair(groupname,std::vector<KinBody::GeometryInfoPtr>())).first;
    it->second.resize(geometries.size());
    std::copy(geometries.begin(),geometries.end(),it->second.begin());
    GetParent()->_PostprocessChangedParameters(Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
}

int KinBody::Link::GetGroupNumGeometries(const std::string& groupname) const
{
    std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::const_iterator it = _info.extraGeometries.find(groupname);
    if( it == _info.extraGeometries.end() ) {
        return -1;
    }
    return it->second.size();
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
}

bool KinBody::Link::ValidateContactNormal(const Vector& position, Vector& normal) const
{
    if( _vGeometries.size() == 1) {
        return _vGeometries.front()->ValidateContactNormal(position,normal);
    }
    else if( _vGeometries.size() > 1 ) {
        RAVELOG_VERBOSE(str(boost::format("cannot validate normal when there is more than one geometry in link '%s(%d)' (do not know colliding geometry)")%_info.name%GetIndex()));
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
        _info.floatParameters[key] = parameters;
    }
    else {
        _info.floatParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

void KinBody::Link::SetIntParameters(const std::string& key, const std::vector<int>& parameters)
{
    if( parameters.size() > 0 ) {
        _info.intParameters[key] = parameters;
    }
    else {
        _info.intParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

void KinBody::Link::SetStringParameters(const std::string& key, const std::string& value)
{
    if( value.size() > 0 ) {
        _info.stringParameters[key] = value;
    }
    else {
        _info.stringParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

bool KinBody::Link::IsRigidlyAttached(boost::shared_ptr<Link const> plink) const
{
    return find(_vRigidlyAttachedLinks.begin(),_vRigidlyAttachedLinks.end(),plink->GetIndex()) != _vRigidlyAttachedLinks.end();
}

void KinBody::Link::UpdateInfo()
{
    if( _info.geometries.size() != _vGeometries.size() ) {
        // have to recompute the geometries
        _info.geometries.resize(_vGeometries.size());
        for(size_t i = 0; i < _info.geometries.size(); ++i) {
            if( !_info.geometries[i] ) {
                _info.geometries[i].reset(new KinBody::GeometryInfo());
            }
            *_info.geometries[i] = _vGeometries[i]->GetInfo();
        }
    }
}

void KinBody::Link::_Update(bool parameterschanged)
{
    // if there's only one trimesh geometry and it has identity offset, then copy it directly
    if( _vGeometries.size() == 1 && _vGeometries.at(0)->GetType() == GT_TriMesh && TransformDistanceFast(Transform(), _vGeometries.at(0)->GetTransform()) <= g_fEpsilonLinear ) {
        _collision = _vGeometries.at(0)->GetCollisionMesh();
    }
    else {
        _collision.vertices.resize(0);
        _collision.indices.resize(0);
        FOREACH(itgeom,_vGeometries) {
            _collision.Append((*itgeom)->GetCollisionMesh(),(*itgeom)->GetTransform());
        }
    }
    if( parameterschanged ) {
        GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);
    }
}

}
