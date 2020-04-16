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
#include <algorithm>

namespace OpenRAVE {

KinBody::LinkInfo::LinkInfo() : XMLReadable("link"), _mass(0), _bStatic(false), _bIsEnabled(true) {
}

KinBody::LinkInfo::LinkInfo(const LinkInfo& other) : XMLReadable("link")
{
    *this = other;
}

void KinBody::LinkInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    openravejson::SetJsonValueByKey(value, "name", _name, allocator);

    Transform tmpTransform {_t};
    Transform tmpMassTransform {_tMassFrame};
    tmpTransform.trans *= fUnitScale;
    tmpMassTransform.trans *= fUnitScale;

    openravejson::SetJsonValueByKey(value, "transform", tmpTransform, allocator);
    openravejson::SetJsonValueByKey(value, "massTransform", tmpMassTransform, allocator);
    openravejson::SetJsonValueByKey(value, "mass", _mass, allocator);
    openravejson::SetJsonValueByKey(value, "intertialMoments", _vinertiamoments, allocator);

    if (_mapFloatParameters.size() > 0) {
        openravejson::SetJsonValueByKey(value, "floatParameters", _mapFloatParameters, allocator);
    }

    if (_mapIntParameters.size() > 0) {
        openravejson::SetJsonValueByKey(value, "intParameters", _mapIntParameters, allocator);
    }

    if (_mapStringParameters.size() > 0) {
        openravejson::SetJsonValueByKey(value, "stringParameters", _mapStringParameters, allocator);
    }

    if (_vForcedAdjacentLinks.size() > 0) {
        openravejson::SetJsonValueByKey(value, "forcedAdjacentLinks", _vForcedAdjacentLinks, allocator);
    }

    if (_vgeometryinfos.size() > 0) {
        rapidjson::Value geometriesValue;
        geometriesValue.SetArray();
        geometriesValue.Reserve(_vgeometryinfos.size(), allocator);
        FOREACHC(it, _vgeometryinfos) {
            rapidjson::Value geometryValue;
            (*it)->SerializeJSON(geometryValue, allocator, options);
            geometriesValue.PushBack(geometryValue, allocator);
        }
        value.AddMember("geometries", geometriesValue, allocator);
    }

    if(_mapExtraGeometries.size() > 0 ) {
        rapidjson::Value extraGeometriesValue;
        extraGeometriesValue.SetObject();
        FOREACHC(im, _mapExtraGeometries) {
            rapidjson::Value geometriesValue;
            geometriesValue.SetArray();
            FOREACHC(iv, im->second){
                if(!!(*iv))
                {
                    rapidjson::Value geometryValue;
                    (*iv)->SerializeJSON(geometryValue, allocator);
                    geometriesValue.PushBack(geometryValue, allocator);
                }
            }
            extraGeometriesValue.AddMember(rapidjson::Value(im->first.c_str(), allocator).Move(), geometriesValue, allocator);
        }
        value.AddMember("extraGeometries", extraGeometriesValue, allocator);
    }

    openravejson::SetJsonValueByKey(value, "isStatic", _bStatic, allocator);
    openravejson::SetJsonValueByKey(value, "isEnabled", _bIsEnabled, allocator);
}

void KinBody::LinkInfo::DeserializeJSON(const rapidjson::Value &value, dReal fUnitScale)
{
    openravejson::LoadJsonValueByKey(value, "name", _name);
    openravejson::LoadJsonValueByKey(value, "transform", _t);
    openravejson::LoadJsonValueByKey(value, "massTransform", _tMassFrame);
    openravejson::LoadJsonValueByKey(value, "mass", _mass);
    openravejson::LoadJsonValueByKey(value, "intertialMoments", _vinertiamoments);
    openravejson::LoadJsonValueByKey(value, "floatParameters", _mapFloatParameters);
    openravejson::LoadJsonValueByKey(value, "intParameters", _mapIntParameters);
    openravejson::LoadJsonValueByKey(value, "stringParameters", _mapStringParameters);
    openravejson::LoadJsonValueByKey(value, "forcedAdjacentLinks", _vForcedAdjacentLinks);

    _t.trans *= fUnitScale;
    _tMassFrame.trans *= fUnitScale;

    if (value.HasMember("geometries")) {
        _vgeometryinfos.clear();
        _vgeometryinfos.reserve(value["geometries"].Size());
        for (size_t i = 0; i < value["geometries"].Size(); ++i) {
            GeometryInfoPtr pGeometryInfo(new GeometryInfo());
            pGeometryInfo->DeserializeJSON(value["geometries"][i], fUnitScale);
            _vgeometryinfos.push_back(pGeometryInfo);
        }
    }
    if (value.HasMember("extraGeometries")) {
        _mapExtraGeometries.clear();
        for (rapidjson::Value::ConstMemberIterator it = value["extraGeometries"].MemberBegin(); it != value["extraGeometries"].MemberEnd(); ++it) {
            _mapExtraGeometries[it->name.GetString()] = std::vector<GeometryInfoPtr>();
            std::vector<GeometryInfoPtr>& vgeometries = _mapExtraGeometries[it->name.GetString()];
            vgeometries.reserve(it->value.Size());

            for(rapidjson::Value::ConstValueIterator im = it->value.Begin(); im != it->value.End(); ++im) {
                GeometryInfoPtr pInfo (new GeometryInfo());
                pInfo->DeserializeJSON(*im, fUnitScale);
                vgeometries.push_back(pInfo);
            }
        }
    }

    openravejson::LoadJsonValueByKey(value, "isStatic", _bStatic);
    openravejson::LoadJsonValueByKey(value, "isEnabled", _bIsEnabled);
}

KinBody::LinkInfo& KinBody::LinkInfo::operator=(const KinBody::LinkInfo& other)
{
    _vgeometryinfos.resize(other._vgeometryinfos.size());
    for( size_t i = 0; i < _vgeometryinfos.size(); ++i ) {
        if( !other._vgeometryinfos[i] ) {
            _vgeometryinfos[i].reset();
        }
        else {
            _vgeometryinfos[i].reset(new GeometryInfo(*(other._vgeometryinfos[i])));
        }
    }

    _mapExtraGeometries.clear();
    for( std::map< std::string, std::vector<GeometryInfoPtr> >::const_iterator it = other._mapExtraGeometries.begin(); it != other._mapExtraGeometries.end(); ++it ) {
        _mapExtraGeometries[it->first] = std::vector<GeometryInfoPtr>(it->second.size());
        std::vector<GeometryInfoPtr>& extraGeometries = _mapExtraGeometries[it->first];
        for( size_t i = 0; i < extraGeometries.size(); ++i ) {
            if( !!(it->second[i]) ) {
                extraGeometries[i].reset(new GeometryInfo(*(it->second[i])));
            }
        }
    }

    _name = other._name;
    _t = other._t;
    _tMassFrame = other._tMassFrame;
    _mass = other._mass;
    _vinertiamoments = other._vinertiamoments;
    _mapFloatParameters = other._mapFloatParameters;
    _mapIntParameters = other._mapIntParameters;
    _mapStringParameters = other._mapStringParameters;
    _vForcedAdjacentLinks = other._vForcedAdjacentLinks;
    _bStatic = other._bStatic;
    _bIsEnabled = other._bIsEnabled;

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
    if( _info._bIsEnabled != bEnable ) {
        KinBodyPtr parent = GetParent();
        parent->_nNonAdjacentLinkCache &= ~AO_Enabled;
        _info._bIsEnabled = bEnable;
        GetParent()->_PostprocessChangedParameters(Prop_LinkEnable);
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

bool KinBody::Link::IsParentLink(const Link &link) const
{
    return find(_vParentLinks.begin(),_vParentLinks.end(), link.GetIndex()) != _vParentLinks.end();
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
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
}

void KinBody::Link::SetPrincipalMomentsOfInertia(const Vector& inertiamoments)
{
    _info._vinertiamoments = inertiamoments;
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
}

void KinBody::Link::SetMass(dReal mass)
{
    _info._mass=mass;
    GetParent()->_PostprocessChangedParameters(Prop_LinkDynamics);
}

AABB KinBody::Link::ComputeLocalAABB() const
{
    return ComputeAABBFromTransform(Transform());
}

AABB KinBody::Link::ComputeAABB() const
{
    return ComputeAABBFromTransform(_info._t);
}

AABB KinBody::Link::ComputeAABBFromTransform(const Transform& tLink) const
{
    if( _vGeometries.size() == 1) {
        return _vGeometries.front()->ComputeAABB(tLink);
    }
    else if( _vGeometries.size() > 1 ) {
        Vector vmin, vmax;
        bool binitialized=false;
        AABB ab;
        FOREACHC(itgeom,_vGeometries) {
            ab = (*itgeom)->ComputeAABB(tLink);
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
            ab.pos = tLink.trans;
            ab.extents = Vector(0,0,0);
        }
        else {
            ab.pos = (dReal)0.5 * (vmin + vmax);
            ab.extents = vmax - ab.pos;
        }
        return ab;
    }
    // have to at least return the correct position!
    return AABB(tLink.trans,Vector(0,0,0));
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
        GetParent()->_PostprocessChangedParameters(Prop_LinkStatic);
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

void KinBody::Link::InitGeometries(std::vector<KinBody::GeometryInfoConstPtr>& geometries, bool bForceRecomputeMeshCollision)
{
    _vGeometries.resize(geometries.size());
    for(size_t i = 0; i < geometries.size(); ++i) {
        _vGeometries[i].reset(new Geometry(shared_from_this(),*geometries[i]));
        if( bForceRecomputeMeshCollision || _vGeometries[i]->GetCollisionMesh().vertices.size() == 0 ) {
            if( !bForceRecomputeMeshCollision ) {
                RAVELOG_VERBOSE("geometry has empty collision mesh\n");
            }
            _vGeometries[i]->InitCollisionMesh(); // have to initialize the mesh since some plugins might not understand all geometry types
        }
    }
    _info._mapExtraGeometries.clear();
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

void KinBody::Link::InitGeometries(std::list<KinBody::GeometryInfo>& geometries, bool bForceRecomputeMeshCollision)
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
    _info._mapExtraGeometries.clear();
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
        pvinfos = &_info._vgeometryinfos;
    }
    else {
        std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = _info._mapExtraGeometries.find(groupname);
        if( it == _info._mapExtraGeometries.end() ) {
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
    std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::const_iterator it = _info._mapExtraGeometries.find(groupname);
    if( it == _info._mapExtraGeometries.end() ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("geometry group %s does not exist for link %s"), groupname%GetName(), ORE_InvalidArguments);
    }
    return it->second;
}

void KinBody::Link::SetGroupGeometries(const std::string& groupname, const std::vector<KinBody::GeometryInfoPtr>& geometries)
{
    FOREACH(itgeominfo, geometries) {
        if( !(*itgeominfo) ) {
            int igeominfo = itgeominfo - geometries.begin();
            throw OPENRAVE_EXCEPTION_FORMAT("GeometryInfo index %d is invalid for body %s", igeominfo%GetParent()->GetName(), ORE_InvalidArguments);
        }
    }
    std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = _info._mapExtraGeometries.insert(make_pair(groupname,std::vector<KinBody::GeometryInfoPtr>())).first;
    it->second.resize(geometries.size());
    std::copy(geometries.begin(),geometries.end(),it->second.begin());
    GetParent()->_PostprocessChangedParameters(Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
}

int KinBody::Link::GetGroupNumGeometries(const std::string& groupname) const
{
    std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::const_iterator it = _info._mapExtraGeometries.find(groupname);
    if( it == _info._mapExtraGeometries.end() ) {
        return -1;
    }
    return it->second.size();
}

void KinBody::Link::AddGeometry(KinBody::GeometryInfoPtr pginfo, bool addToGroups)
{
    if( !pginfo ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("tried to add improper geometry to link %s"), GetName(), ORE_InvalidArguments);
    }

    const KinBody::GeometryInfo& ginfo = *pginfo;
    if( ginfo._name.size() > 0 ) {
        // check if similar name exists and throw if it does
        FOREACH(itgeometry, _vGeometries) {
            if( (*itgeometry)->GetName() == ginfo._name ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("new added geometry %s has conflicting name for link %s"), ginfo._name%GetName(), ORE_InvalidArguments);
            }
        }

        FOREACH(itgeometryinfo, _info._vgeometryinfos) {
            if( (*itgeometryinfo)->_name == ginfo._name ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("new added geometry %s has conflicting name for link %s"), ginfo._name%GetName(), ORE_InvalidArguments);
            }
        }
        if( addToGroups ) {
            FOREACH(itgeometrygroup, _info._mapExtraGeometries) {
                FOREACH(itgeometryinfo, itgeometrygroup->second) {
                    if( (*itgeometryinfo)->_name == ginfo._name ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("new added geometry %s for group %s has conflicting name for link %s"), ginfo._name%itgeometrygroup->first%GetName(), ORE_InvalidArguments);
                    }
                }
            }
        }
    }

    _vGeometries.push_back(GeometryPtr(new Geometry(shared_from_this(),*pginfo)));
    _vGeometries.back()->InitCollisionMesh();
    _info._vgeometryinfos.push_back(pginfo);
    if( addToGroups ) {
        FOREACH(itgeometrygroup, _info._mapExtraGeometries) {
            itgeometrygroup->second.push_back(pginfo);
        }
    }
    _Update(true, Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
}

void KinBody::Link::RemoveGeometryByName(const std::string& geometryname, bool removeFromAllGroups)
{
    OPENRAVE_ASSERT_OP(geometryname.size(),>,0);
    bool bChanged = false;

    std::vector<GeometryPtr>::iterator itgeometry = _vGeometries.begin();
    while(itgeometry != _vGeometries.end()) {
        if( (*itgeometry)->GetName() == geometryname ) {
            itgeometry = _vGeometries.erase(itgeometry);
            bChanged = true;
        }
        else {
            ++itgeometry;
        }
    }
    std::vector<KinBody::GeometryInfoPtr>::iterator itgeometryinfo = _info._vgeometryinfos.begin();
    while(itgeometryinfo != _info._vgeometryinfos.end()) {
        if( (*itgeometryinfo)->_name == geometryname ) {
            itgeometryinfo = _info._vgeometryinfos.erase(itgeometryinfo);
            bChanged = true;
        }
        else {
            ++itgeometryinfo;
        }
    }

    if( removeFromAllGroups ) {
        FOREACH(itgeometrygroup, _info._mapExtraGeometries) {
            std::vector<KinBody::GeometryInfoPtr>::iterator itgeometryinfo2 = itgeometrygroup->second.begin();
            while(itgeometryinfo2 != itgeometrygroup->second.end()) {
                if( (*itgeometryinfo2)->_name == geometryname ) {
                    itgeometryinfo2 = itgeometrygroup->second.erase(itgeometryinfo2);
                    bChanged = true;
                }
                else {
                    ++itgeometryinfo2;
                }
            }
        }
    }

    if( bChanged ) {
        _Update(true, Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
    }
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
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

void KinBody::Link::SetIntParameters(const std::string& key, const std::vector<int>& parameters)
{
    if( parameters.size() > 0 ) {
        _info._mapIntParameters[key] = parameters;
    }
    else {
        _info._mapIntParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

void KinBody::Link::SetStringParameters(const std::string& key, const std::string& value)
{
    if( value.size() > 0 ) {
        _info._mapStringParameters[key] = value;
    }
    else {
        _info._mapStringParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_LinkCustomParameters);
}

bool KinBody::Link::IsRigidlyAttached(const Link &link) const
{
    return find(_vRigidlyAttachedLinks.begin(),_vRigidlyAttachedLinks.end(),link.GetIndex()) != _vRigidlyAttachedLinks.end();
}

void KinBody::Link::UpdateInfo()
{
    // always have to recompute the geometries
    _info._vgeometryinfos.resize(_vGeometries.size());
    for(size_t i = 0; i < _info._vgeometryinfos.size(); ++i) {
        if( !_info._vgeometryinfos[i] ) {
            _info._vgeometryinfos[i].reset(new KinBody::GeometryInfo());
        }
        *_info._vgeometryinfos[i] = _vGeometries[i]->GetInfo();
    }
}

void KinBody::Link::_Update(bool parameterschanged, uint32_t extraParametersChanged)
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
    if( parameterschanged || extraParametersChanged ) {
        GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry|extraParametersChanged);
    }
}

}
