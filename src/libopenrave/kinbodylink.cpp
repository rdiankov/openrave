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

void KinBody::LinkInfo::Reset()
{
    _vgeometryinfos.clear();
    _mapExtraGeometries.clear();
    _id.clear();
    _name.clear();
    _t = Transform();
    _tMassFrame = Transform();
    _mass = 1e-10; // to avoid divide by 0 for inverse dynamics/physics computations
    _vinertiamoments = Vector();
    _mapFloatParameters.clear();
    _mapIntParameters.clear();
    _mapStringParameters.clear();
    _vForcedAdjacentLinks.clear();
    _mReadableInterfaces.clear();
    _bStatic = false;
    _bIsEnabled = false;
    _bIgnoreSelfCollision = false;
}

int KinBody::LinkInfo::Compare(const LinkInfo& rhs, int linkCompareOptions, dReal fUnitScale, dReal fEpsilon) const
{
    if( _vgeometryinfos.size() != rhs._vgeometryinfos.size() ) {
        return 1;
    }

    if( _id != rhs._id ) {
        return 2;
    }
    if( _name != rhs._name ) {
        return 3;
    }
    if( _mapFloatParameters != rhs._mapFloatParameters ) {
        return 4;
    }
    if( _mapIntParameters != rhs._mapIntParameters ) {
        return 5;
    }
    if( _mapStringParameters != rhs._mapStringParameters ) {
        return 6;
    }

    if( _vForcedAdjacentLinks != rhs._vForcedAdjacentLinks ) {
        return 7;
    }

    if( _bStatic != rhs._bStatic ) {
        return 8;
    }

    if( _bIsEnabled != rhs._bIsEnabled ) {
        return 9;
    }

    if( !(linkCompareOptions & 1) ) {
        if( !IsZeroWithEpsilon4(_t.rot - rhs._t.rot, fEpsilon) ) {
            return 10;
        }
        if( !IsZeroWithEpsilon3(_t.trans - rhs._t.trans*fUnitScale, fEpsilon) ) {
            return 11;
        }
    }

    if( !IsZeroWithEpsilon4(_tMassFrame.rot - rhs._tMassFrame.rot, fEpsilon) ) {
        return 12;
    }
    if( !IsZeroWithEpsilon3(_tMassFrame.trans - rhs._tMassFrame.trans*fUnitScale, fEpsilon) ) {
        return 13;
    }

    if( RaveFabs(_mass - rhs._mass) > fEpsilon ) {
        return 14;
    }
    if( RaveFabs(_vinertiamoments[0]- rhs._vinertiamoments[0]*fUnitScale*fUnitScale) > fEpsilon ) {
        return 15;
    }
    if( RaveFabs(_vinertiamoments[1]- rhs._vinertiamoments[1]*fUnitScale*fUnitScale) > fEpsilon ) {
        return 16;
    }
    if( RaveFabs(_vinertiamoments[2]- rhs._vinertiamoments[2]*fUnitScale*fUnitScale) > fEpsilon ) {
        return 17;
    }

    if( _bIgnoreSelfCollision != rhs._bIgnoreSelfCollision ) {
        return 18;
    }

    for(int igeom = 0; igeom < (int)_vgeometryinfos.size(); ++igeom) {
        int geomcompare = _vgeometryinfos[igeom]->Compare(*rhs._vgeometryinfos[igeom], fUnitScale, fEpsilon);
        if (geomcompare != 0) {
            return 19|(igeom<<16)|(geomcompare<<24);
        }
    }

    return 0;
}

void KinBody::LinkInfo::ConvertUnitScale(dReal fUnitScale)
{
    FOREACH(itgeometry, _vgeometryinfos) {
        (*itgeometry)->ConvertUnitScale(fUnitScale);
    }
    FOREACH(itextra, _mapExtraGeometries) {
        FOREACH(itgeometry, itextra->second) {
            (*itgeometry)->ConvertUnitScale(fUnitScale);
        }
    }

    _tMassFrame.trans *= fUnitScale;
    _vinertiamoments *= fUnitScale*fUnitScale;
}

void KinBody::LinkInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    value.SetObject();
    orjson::SetJsonValueByKey(value, "id", _id, allocator);
    orjson::SetJsonValueByKey(value, "name", _name, allocator);

    Transform tmpTransform {_t};
    Transform tmpMassTransform {_tMassFrame};
    tmpTransform.trans *= fUnitScale;
    tmpMassTransform.trans *= fUnitScale;

    orjson::SetJsonValueByKey(value, "transform", tmpTransform, allocator);
    orjson::SetJsonValueByKey(value, "massTransform", tmpMassTransform, allocator);
    orjson::SetJsonValueByKey(value, "mass", _mass, allocator);
    orjson::SetJsonValueByKey(value, "inertiaMoments", _vinertiamoments*fUnitScale*fUnitScale, allocator);

    if(_mapFloatParameters.size() > 0) {
        rapidjson::Value parameters;
        parameters.SetArray();
        FOREACHC(it, _mapFloatParameters) {
            rapidjson::Value parameter;
            parameter.SetObject();
            orjson::SetJsonValueByKey(parameter, "id", it->first, allocator);
            orjson::SetJsonValueByKey(parameter, "values", it->second, allocator);
            parameters.PushBack(parameter, allocator);
        }
        value.AddMember("floatParameters", parameters, allocator);
    }
    if(_mapIntParameters.size() > 0) {
        rapidjson::Value parameters;
        parameters.SetArray();
        FOREACHC(it, _mapIntParameters) {
            rapidjson::Value parameter;
            parameter.SetObject();
            orjson::SetJsonValueByKey(parameter, "id", it->first, allocator);
            orjson::SetJsonValueByKey(parameter, "values", it->second, allocator);
            parameters.PushBack(parameter, allocator);
        }
        value.AddMember("intParameters", parameters, allocator);
    }
    if(_mapStringParameters.size() > 0) {
        rapidjson::Value parameters;
        parameters.SetArray();
        FOREACHC(it, _mapStringParameters) {
            rapidjson::Value parameter;
            parameter.SetObject();
            orjson::SetJsonValueByKey(parameter, "id", it->first, allocator);
            orjson::SetJsonValueByKey(parameter, "value", it->second, allocator);
            parameters.PushBack(parameter, allocator);
        }
        value.AddMember("stringParameters", parameters, allocator);
    }

    if (_vForcedAdjacentLinks.size() > 0) {
        orjson::SetJsonValueByKey(value, "forcedAdjacentLinks", _vForcedAdjacentLinks, allocator);
    }

    if (_vgeometryinfos.size() > 0) {
        rapidjson::Value geometriesValue;
        geometriesValue.SetArray();
        geometriesValue.Reserve(_vgeometryinfos.size(), allocator);
        FOREACHC(it, _vgeometryinfos) {
            rapidjson::Value geometryValue;
            (*it)->SerializeJSON(geometryValue, allocator, fUnitScale, options);
            geometriesValue.PushBack(geometryValue, allocator);
        }
        value.AddMember("geometries", geometriesValue, allocator);
    }

    if (!_mReadableInterfaces.empty()) {
        rapidjson::Value rReadableInterfaces;
        rReadableInterfaces.SetObject();
        for (std::map<std::string, ReadablePtr>::const_iterator it = _mReadableInterfaces.begin(); it != _mReadableInterfaces.end(); it++) {
            rapidjson::Value rReadable;
            it->second->SerializeJSON(rReadable, allocator, fUnitScale, options);
            orjson::SetJsonValueByKey(rReadableInterfaces, it->first.c_str(), rReadable, allocator);
        }
        value.AddMember("readableInterfaces", std::move(rReadableInterfaces), allocator);
    }

    // if(_mapExtraGeometries.size() > 0 ) {
    //     rapidjson::Value extraGeometriesValue;
    //     extraGeometriesValue.SetObject();
    //     FOREACHC(im, _mapExtraGeometries) {
    //         rapidjson::Value geometriesValue;
    //         geometriesValue.SetArray();
    //         FOREACHC(iv, im->second){
    //             if(!!(*iv))
    //             {
    //                 rapidjson::Value geometryValue;
    //                 (*iv)->SerializeJSON(geometryValue, allocator);
    //                 geometriesValue.PushBack(geometryValue, allocator);
    //             }
    //         }
    //         extraGeometriesValue.AddMember(rapidjson::Value(im->first.c_str(), allocator).Move(), geometriesValue, allocator);
    //     }
    //     value.AddMember("extraGeometries", extraGeometriesValue, allocator);
    // }

    orjson::SetJsonValueByKey(value, "isStatic", _bStatic, allocator);
    orjson::SetJsonValueByKey(value, "isEnabled", _bIsEnabled, allocator);
    orjson::SetJsonValueByKey(value, "isSelfCollisionIgnored", _bIgnoreSelfCollision, allocator);
}

void KinBody::LinkInfo::DeserializeJSON(const rapidjson::Value &value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "id", _id);
    orjson::LoadJsonValueByKey(value, "name", _name);

    if (value.HasMember("transform")) {
        orjson::LoadJsonValueByKey(value, "transform", _t);
        _t.trans *= fUnitScale;  // partial update should only mutliply fUnitScale once if the key is in value
        _modifiedFields |= KinBody::LinkInfo::LIF_Transform;
    }
    if (value.HasMember("massTransform")) {
        orjson::LoadJsonValueByKey(value, "massTransform", _tMassFrame);
        _tMassFrame.trans *= fUnitScale;
    }

    orjson::LoadJsonValueByKey(value, "mass", _mass);
    orjson::LoadJsonValueByKey(value, "inertiaMoments", _vinertiamoments);
    _vinertiamoments *= fUnitScale*fUnitScale;

    if (value.HasMember("floatParameters") && value["floatParameters"].IsArray()) {
        for (rapidjson::Value::ConstValueIterator it = value["floatParameters"].Begin(); it != value["floatParameters"].End(); ++it) {
            std::string key;
            if( it->HasMember("id") ) {
                orjson::LoadJsonValueByKey(*it, "id", key);
            }
            else if( it->HasMember("key") ) {
                // backward compatibility
                orjson::LoadJsonValueByKey(*it, "key", key);
            }
            if (key.empty()) {
                RAVELOG_WARN_FORMAT("ignored an entry in floatParameters in link %s due to missing or empty id", _id);
                continue;
            }
            // delete
            if (OpenRAVE::orjson::GetJsonValueByKey<bool>(*it, "__deleted__", false)) {
                _mapFloatParameters.erase(key);
                continue;
            }
            orjson::LoadJsonValueByKey(*it, "values", _mapFloatParameters[key]);
        }
    }
    if (value.HasMember("intParameters") && value["intParameters"].IsArray()) {
        for (rapidjson::Value::ConstValueIterator it = value["intParameters"].Begin(); it != value["intParameters"].End(); ++it) {
            std::string key;
            if( it->HasMember("id") ) {
                orjson::LoadJsonValueByKey(*it, "id", key);
            }
            else if( it->HasMember("key") ) {
                // backward compatibility
                orjson::LoadJsonValueByKey(*it, "key", key);
            }
            if (key.empty()) {
                RAVELOG_WARN_FORMAT("ignored an entry in intParameters in link %s due to missing or empty id", _id);
                continue;
            }
            // delete
            if (OpenRAVE::orjson::GetJsonValueByKey<bool>(*it, "__deleted__", false)) {
                _mapIntParameters.erase(key);
                continue;
            }
            orjson::LoadJsonValueByKey(*it, "values", _mapIntParameters[key]);
        }
    }
    if (value.HasMember("stringParameters") && value["stringParameters"].IsArray()) {
        for (rapidjson::Value::ConstValueIterator it = value["stringParameters"].Begin(); it != value["stringParameters"].End(); ++it) {
            std::string key;
            if( it->HasMember("id") ) {
                orjson::LoadJsonValueByKey(*it, "id", key);
            }
            else if( it->HasMember("key") ) {
                // backward compatibility
                orjson::LoadJsonValueByKey(*it, "key", key);
            }
            if (key.empty()) {
                RAVELOG_WARN_FORMAT("ignored an entry in stringParameters in link %s due to missing or empty id", _id);
                continue;
            }
            // delete
            if (OpenRAVE::orjson::GetJsonValueByKey<bool>(*it, "__deleted__", false)) {
                _mapStringParameters.erase(key);
                continue;
            }
            orjson::LoadJsonValueByKey(*it, "value", _mapStringParameters[key]);
        }
    }

    orjson::LoadJsonValueByKey(value, "forcedAdjacentLinks", _vForcedAdjacentLinks);

    if (value.HasMember("geometries")) {
        _vgeometryinfos.reserve(value["geometries"].Size() + _vgeometryinfos.size());
        for (rapidjson::Value::ConstValueIterator it = value["geometries"].Begin(); it != value["geometries"].End(); ++it) {
            UpdateOrCreateInfoWithNameCheck(*it, _vgeometryinfos, "name", fUnitScale, options);
        }
    }

    if (value.HasMember("readableInterfaces") && value["readableInterfaces"].IsObject()) {
        for (rapidjson::Value::ConstMemberIterator it = value["readableInterfaces"].MemberBegin(); it != value["readableInterfaces"].MemberEnd(); ++it) {
            _DeserializeReadableInterface(it->name.GetString(), it->value, fUnitScale);
        }
    }

    _mapExtraGeometries.clear();
    // if (value.HasMember("extraGeometries")) {
    //     for (rapidjson::Value::ConstMemberIterator it = value["extraGeometries"].MemberBegin(); it != value["extraGeometries"].MemberEnd(); ++it) {
    //         if (_mapExtraGeometries.find(it->name.GetString()) == _mapExtraGeometries.end()) {
    //             _mapExtraGeometries[it->name.GetString()] = std::vector<GeometryInfoPtr>();
    //         }
    //         std::vector<GeometryInfoPtr>& vgeometries = _mapExtraGeometries[it->name.GetString()];
    //         vgeometries.reserve(it->value.Size() + vgeometries.size());
    //         size_t iGeometry = 0;
    //         for(rapidjson::Value::ConstValueIterator im = it->value.Begin(); im != it->value.End(); ++im, ++iGeometry) {
    //             std::string id = orjson::GetStringJsonValueByKey(*im, "id");
    //             UpdateOrCreateInfoWithNameCheck(*im, id, vgeometries, "name", fUnitScale);
    //         }
    //     }
    // }

    orjson::LoadJsonValueByKey(value, "isStatic", _bStatic);
    orjson::LoadJsonValueByKey(value, "isEnabled", _bIsEnabled);
    orjson::LoadJsonValueByKey(value, "isSelfCollisionIgnored", _bIgnoreSelfCollision);
}

void KinBody::LinkInfo::_DeserializeReadableInterface(const std::string& id, const rapidjson::Value& rReadable, dReal fUnitScale) {
    std::map<std::string, ReadablePtr>::iterator itReadable = _mReadableInterfaces.find(id);
    ReadablePtr pReadable;
    if(itReadable != _mReadableInterfaces.end()) {
        pReadable = itReadable->second;
    }
    // NOTE: we use PT_KinBody (for now) for the following reasons:
    // 1. Link shares the same set of readable plugins as KinBody
    // 2. It might be confusing to add PT_Link since it is not an interface type
    BaseJSONReaderPtr pReader = RaveCallJSONReader(PT_KinBody, id, pReadable, AttributesList());
    if (!!pReader) {
        pReader->DeserializeJSON(rReadable, fUnitScale);
        _mReadableInterfaces[id] = pReader->GetReadable();
        return;
    }
    if (rReadable.IsString()) {
        StringReadablePtr pStringReadable(new StringReadable(id, rReadable.GetString()));
        _mReadableInterfaces[id] = pStringReadable;
        return;
    }
    JSONReadablePtr pReadableJSON(new JSONReadable(id, rReadable));
    _mReadableInterfaces[id] = pReadableJSON;
    // RAVELOG_WARN_FORMAT("deserialize readable interface '%s' failed, perhaps need to call 'RaveRegisterJSONReader' with the appropriate reader.", id);
}

bool KinBody::LinkInfo::operator==(const KinBody::LinkInfo& other) const {
    return _id == other._id
           && _name == other._name
           && _t == other._t
           && _tMassFrame == other._tMassFrame
           && _mass == other._mass
           && _vinertiamoments == other._vinertiamoments
           && _mapFloatParameters == other._mapFloatParameters
           && _mapIntParameters == other._mapIntParameters
           && _mapStringParameters == other._mapStringParameters
           && _vForcedAdjacentLinks == other._vForcedAdjacentLinks
           && _bStatic == other._bStatic
           && _bIsEnabled == other._bIsEnabled
           && _bIgnoreSelfCollision == other._bIgnoreSelfCollision
           && _mReadableInterfaces == other._mReadableInterfaces
           && AreVectorsDeepEqual(_vgeometryinfos, other._vgeometryinfos);
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
        _Enable(bEnable);
        GetParent()->_PostprocessChangedParameters(Prop_LinkEnable);
    }
}

void KinBody::Link::_Enable(bool bEnable)
{
    _info._bIsEnabled = bEnable;
    GetParent()->NotifyLinkEnabled(GetIndex(), bEnable);
}

bool KinBody::Link::IsEnabled() const
{
    return _info._bIsEnabled;
}

void KinBody::Link::SetIgnoreSelfCollision(bool bIgnore)
{
    _info._bIgnoreSelfCollision = bIgnore;
}

bool KinBody::Link::IsSelfCollisionIgnored() const
{
    return _info._bIgnoreSelfCollision;
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

AABB KinBody::Link::ComputeLocalAABBForGeometryGroup(const std::string& geomgroupname) const
{
    return ComputeAABBForGeometryGroupFromTransform(geomgroupname, Transform());
}

AABB KinBody::Link::ComputeAABBForGeometryGroup(const std::string& geomgroupname) const
{
    return ComputeAABBForGeometryGroupFromTransform(geomgroupname, _info._t);
}

AABB KinBody::Link::ComputeAABBForGeometryGroupFromTransform(const std::string& geomgroupname, const Transform& tLink) const
{
    const std::vector<KinBody::GeometryInfoPtr>& vgeoms = GetGeometriesFromGroup(geomgroupname);
    if( vgeoms.size() == 1 ) {
        return vgeoms.front()->ComputeAABB(tLink);
    }
    else if( vgeoms.size() > 1 ) {
        Vector vmin, vmax;
        bool binitialized = false;
        AABB ab;
        FOREACHC(itgeom, vgeoms) {
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
        } // end FOREACHC
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
    // have to at least return the correct position
    return AABB(tLink.trans, Vector(0, 0, 0));
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

KinBody::GeometryPtr KinBody::Link::GetGeometry(int index) const
{
    return _vGeometries.at(index);
}

KinBody::GeometryPtr KinBody::Link::GetGeometry(const string_view geomname) const
{
    if( !geomname.empty() ) {
        for(const KinBody::GeometryPtr& pgeom : _vGeometries) {
            if( pgeom->GetName() == geomname ) {
                return pgeom;
            }
        }
    }
    return KinBody::GeometryPtr();
}

void KinBody::Link::_InitGeometriesInternal(bool bForceRecomputeMeshCollision) {
    for(GeometryPtr& pgeom : _vGeometries) {
        if( bForceRecomputeMeshCollision || pgeom->GetCollisionMesh().vertices.size() == 0 ) {
            if( !bForceRecomputeMeshCollision ) {
                RAVELOG_VERBOSE("geometry has empty collision mesh\n");
            }
            pgeom->InitCollisionMesh(); // have to initialize the mesh since some plugins might not understand all geometry types
        }
    }
    _info._mapExtraGeometries.clear();
    // have to reset the self group! cannot use geometries directly since we require exclusive access to the GeometryInfo objects
    std::vector<KinBody::GeometryInfoPtr> vgeometryinfos;
    vgeometryinfos.resize(_vGeometries.size());
    for(int index = 0; index < (int)vgeometryinfos.size(); ++index) {
        vgeometryinfos[index].reset(new KinBody::GeometryInfo());
        *vgeometryinfos[index] = _vGeometries[index]->_info;
    }

    // SetGroupGeometries calls PostprocessChangedParameters on the parent, which would increment the body update stamp, and then we would invoke PostprocessChangedParameters _again_ in _Update here, resulting in duplicated work in callbacks.
    // Coalesce these update calls into one by calling the NoPostprocess version and adding the Prop_LinkGeometryGroup flag to our main _Update call instead.
    _SetGroupGeometriesNoPostprocess("self", vgeometryinfos);
    _Update(/* parametersChanged */ true, /* extraParametersChanged */ Prop_LinkGeometryGroup);
}

void KinBody::Link::InitGeometries(std::vector<KinBody::GeometryInfoConstPtr>& geometries, bool bForceRecomputeMeshCollision)
{
    int index = 0;
    _vGeometries.resize(geometries.size());
    for(KinBody::GeometryInfoConstPtr& pgeominfo : geometries) {
        _vGeometries[index++].reset(new KinBody::Link::Geometry(shared_from_this(), *pgeominfo));
    }
    _InitGeometriesInternal(bForceRecomputeMeshCollision);
}

void KinBody::Link::InitGeometries(std::list<KinBody::GeometryInfo>& geometries, bool bForceRecomputeMeshCollision)
{
    int index = 0;
    _vGeometries.resize(geometries.size());
    for(KinBody::GeometryInfo& geominfo : geometries) {
        _vGeometries[index++].reset(new KinBody::Link::Geometry(shared_from_this(), geominfo));
    }
    _InitGeometriesInternal(bForceRecomputeMeshCollision);
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
        std::stringstream ssGroupNames;
        for(const std::pair< const std::string, std::vector<KinBody::GeometryInfoPtr> >& grouppair : _info._mapExtraGeometries) {
            ssGroupNames << grouppair.first << ", ";
        }
        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, geometry group '%s' does not exist for body '%s' link '%s', current number of geometries=%d, extra groups=[%s], isEnabled=%d."), GetParent()->GetEnv()->GetNameId()%groupname%GetParent()->GetName()%GetName()%_vGeometries.size()%ssGroupNames.str()%_info._bIsEnabled, ORE_InvalidArguments);
    }
    return it->second;
}

void KinBody::Link::_SetGroupGeometriesNoPostprocess(const std::string& groupname, const std::vector<KinBody::GeometryInfoPtr>& geometries)
{
    FOREACH(itgeominfo, geometries) {
        if (!(*itgeominfo)) {
            int igeominfo = itgeominfo - geometries.begin();
            throw OPENRAVE_EXCEPTION_FORMAT("GeometryInfo index %d is invalid for body %s", igeominfo % GetParent()->GetName(), ORE_InvalidArguments);
        }
    }
    std::map<std::string, std::vector<KinBody::GeometryInfoPtr>>::iterator it = _info._mapExtraGeometries.insert(make_pair(groupname, std::vector<KinBody::GeometryInfoPtr>())).first;
    it->second.resize(geometries.size());
    std::copy(geometries.begin(), geometries.end(), it->second.begin());
}

void KinBody::Link::SetGroupGeometries(const std::string& groupname, const std::vector<KinBody::GeometryInfoPtr>& geometries)
{
    _SetGroupGeometriesNoPostprocess(groupname, geometries);
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
                throw OPENRAVE_EXCEPTION_FORMAT(_("newly added geometry %s has conflicting name for link %s"), ginfo._name%GetName(), ORE_InvalidArguments);
            }
        }

        FOREACH(itgeometryinfo, _info._vgeometryinfos) {
            if( (*itgeometryinfo)->_name == ginfo._name ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("newly added geometry %s has conflicting name for link %s"), ginfo._name%GetName(), ORE_InvalidArguments);
            }
        }
        if( addToGroups ) {
            FOREACH(itgeometrygroup, _info._mapExtraGeometries) {
                FOREACH(itgeometryinfo, itgeometrygroup->second) {
                    if( (*itgeometryinfo)->_name == ginfo._name ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("newly added geometry %s for group %s has conflicting name for link %s"), ginfo._name%itgeometrygroup->first%GetName(), ORE_InvalidArguments);
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

void KinBody::Link::AddGeometryToGroup(KinBody::GeometryInfoPtr pginfo, const std::string& groupname)
{
    if( !pginfo ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("tried to add improper geometry to link %s"), GetName(), ORE_InvalidArguments);
    }

    const KinBody::GeometryInfo& ginfo = *pginfo;

    std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = _info._mapExtraGeometries.find(groupname);
    if( it == _info._mapExtraGeometries.end() ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("geometry group %s does not exist for link %s"), groupname%GetName(), ORE_InvalidArguments);
    }
    if( ginfo._name.size() > 0 ) {
        FOREACHC(itgeometryinfo, it->second) {
            if( (*itgeometryinfo)->_name == ginfo._name ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("newly added geometry %s for group %s has conflicting name for link %s"), ginfo._name%groupname%GetName(), ORE_InvalidArguments);
            }
        }
    }

    it->second.push_back(pginfo);
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

bool KinBody::Link::IsRigidlyAttached(const int linkIndex) const
{
    return find(_vRigidlyAttachedLinks.begin(),_vRigidlyAttachedLinks.end(),linkIndex) != _vRigidlyAttachedLinks.end();
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

void KinBody::Link::ExtractInfo(KinBody::LinkInfo& info) const
{
    info = _info;
    info._modifiedFields = 0;
    info._vgeometryinfos.resize(_vGeometries.size());
    for (size_t i = 0; i < info._vgeometryinfos.size(); ++i) {
        info._vgeometryinfos[i].reset(new KinBody::GeometryInfo());
        _vGeometries[i]->ExtractInfo(*info._vgeometryinfos[i]);
    }
    {
        boost::shared_lock< boost::shared_mutex > lock(GetReadableInterfaceMutex());
        FOREACHC(it, GetReadableInterfaces()) {
            if (!!it->second) {
                // make a copy of the readable interface
                // caller may modify and call UpdateFromInfo with modified readable interfaces
                info._mReadableInterfaces[it->first] = it->second->CloneSelf();
            }
        }
    }
}

UpdateFromInfoResult KinBody::Link::UpdateFromInfo(const KinBody::LinkInfo& info)
{
    if(!info._id.empty() && _info._id != info._id) {
        throw OPENRAVE_EXCEPTION_FORMAT("Do not allow updating body '%s' link '%s' (id='%s') with a different info id='%s'", GetParent()->GetName()%GetName()%_info._id%info._id, ORE_Assert);
    }

    UpdateFromInfoResult updateFromInfoResult = UFIR_NoChange;

    std::vector<KinBody::Link::GeometryPtr> vGeometries = _vGeometries;
    if (!UpdateChildrenFromInfo(info._vgeometryinfos, vGeometries, updateFromInfoResult)) {
        return updateFromInfoResult;
    }

    // transform
    if( info.IsModifiedField(KinBody::LinkInfo::LIF_Transform) ) {
        if (TransformDistanceFast(GetTransform(), info._t) > g_fEpsilonLinear) {
            RAVELOG_VERBOSE_FORMAT("link %s transform changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }

    // name
    if (GetName() != info._name) {
        RAVELOG_VERBOSE_FORMAT("link %s name changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    // _bIsEnable
    if (IsEnabled() != info._bIsEnabled) {
        Enable(info._bIsEnabled);
        RAVELOG_VERBOSE_FORMAT("link %s enabled changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // _bIgnoreSelfCollision
    if (IsSelfCollisionIgnored() != info._bIgnoreSelfCollision) {
        SetIgnoreSelfCollision(info._bIgnoreSelfCollision);
        RAVELOG_VERBOSE_FORMAT("link %s ignoreSelfCollision changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // _bStatic
    if (IsStatic() != info._bStatic) {
        SetStatic(info._bStatic);
        RAVELOG_VERBOSE_FORMAT("link %s static changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // _mass
    if (GetMass() != info._mass) {
        SetMass(info._mass);
        RAVELOG_VERBOSE_FORMAT("link %s mass changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // inertiamoments
    if (_info._vinertiamoments != info._vinertiamoments) {
        SetPrincipalMomentsOfInertia(info._vinertiamoments);
        RAVELOG_VERBOSE_FORMAT("link %s inertia moments changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // mass frame
    if (TransformDistanceFast(GetLocalMassFrame(), info._tMassFrame) > g_fEpsilonLinear) {
        SetLocalMassFrame(info._tMassFrame);
        RAVELOG_VERBOSE_FORMAT("link %s mass frame changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // _mapFloatParameters
    const std::map<std::string, std::vector<dReal> > floatParameters = GetFloatParameters();
    if (floatParameters != info._mapFloatParameters) {
        FOREACH(itParam, floatParameters) {
            SetFloatParameters(itParam->first, {}); // erase current parameters
        }
        FOREACH(itParam, info._mapFloatParameters) {
            SetFloatParameters(itParam->first, itParam->second);  // update with new info
        }
        RAVELOG_VERBOSE_FORMAT("link %s float parameters changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // _mapIntParameters
    const std::map<std::string, std::vector<int> > intParameters = GetIntParameters();
    if (intParameters != info._mapIntParameters) {
        FOREACH(itParam, intParameters) {
            SetIntParameters(itParam->first, {});
        }
        FOREACH(itParam, info._mapIntParameters) {
            SetIntParameters(itParam->first, itParam->second);
        }
        RAVELOG_VERBOSE_FORMAT("link %s int parameters changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // _mapStringParameters
    const std::map<std::string, std::string> stringParameters = GetStringParameters();
    if (stringParameters != info._mapStringParameters) {
        FOREACH(itParam, stringParameters) {
            SetStringParameters(itParam->first, {});
        }
        FOREACH(itParam, info._mapStringParameters) {
            SetStringParameters(itParam->first, itParam->second);
        }
        RAVELOG_VERBOSE_FORMAT("link %s string parameters changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }


    // forced adjacent links
    if (_info._vForcedAdjacentLinks != info._vForcedAdjacentLinks) {
        _info._vForcedAdjacentLinks = info._vForcedAdjacentLinks;
        RAVELOG_VERBOSE_FORMAT("link %s forced adjacent links changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    if ( UpdateReadableInterfaces(info._mReadableInterfaces) ) {
        RAVELOG_VERBOSE_FORMAT("link %s updated due to readable interface change", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    return updateFromInfoResult;
}

void KinBody::Link::_Update(bool parameterschanged, uint32_t extraParametersChanged)
{
    // if there's only one trimesh geometry and it has identity offset, then copy it directly
    if( _vGeometries.size() == 1 && _vGeometries.at(0)->GetType() == GT_TriMesh && TransformDistanceFast(Transform(), _vGeometries.at(0)->GetTransform()) <= g_fEpsilonLinear ) {
        _collision = _vGeometries.at(0)->GetCollisionMesh();
    }
    else {
        // Clear the existing collision volume data
        _collision.vertices.resize(0);
        _collision.indices.resize(0);

        // Do a quick precalculation of the new collision volume total size so we can reduce allocs in Append
        unsigned totalVertices = 0, totalIndices = 0;
        FOREACH(itgeom,_vGeometries) {
            totalVertices += (*itgeom)->GetCollisionMesh().vertices.size();
            totalIndices += (*itgeom)->GetCollisionMesh().indices.size();
        }

        // Pre-size the vertex/index vectors for the final volume size
        _collision.vertices.reserve(totalVertices);
        _collision.indices.reserve(totalIndices);

        // Actually go through and transform/insert our collision geometries
        FOREACH(itgeom,_vGeometries) {
            _collision.Append((*itgeom)->GetCollisionMesh(),(*itgeom)->GetTransform());
        }
    }
    if( parameterschanged || extraParametersChanged ) {
        GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry|extraParametersChanged);
    }
}

}
