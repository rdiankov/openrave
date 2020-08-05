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

// used for functions that are also used internally
#define CHECK_NO_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed == 0, "env=%d, body %s cannot be added to environment when doing this operation, current value is %d", GetEnv()->GetId()%GetName()%_nHierarchyComputed, ORE_InvalidState);
#define CHECK_INTERNAL_COMPUTATION0 OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed != 0, "env=%d, body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetEnv()->GetId()%GetName()%_nHierarchyComputed, ORE_NotInitialized);
#define CHECK_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed == 2, "env=%d, body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetEnv()->GetId()%GetName()%_nHierarchyComputed, ORE_NotInitialized);

namespace OpenRAVE {

const char* GetDynamicsConstraintsTypeString(DynamicsConstraintsType type)
{
    switch(type) {
    case DC_Unknown: return "Unknown";
    case DC_IgnoreTorque: return "IgnoreTorque";
    case DC_NominalTorque: return "NominalTorque";
    case DC_InstantaneousTorque: return "InstantaneousTorque";
    }
    return "";
}

class ChangeCallbackData : public UserData
{
public:
    ChangeCallbackData(int properties, const boost::function<void()>& callback, KinBodyConstPtr pbody) : _properties(properties), _callback(callback), _pweakbody(pbody) {
    }
    virtual ~ChangeCallbackData() {
        KinBodyConstPtr pbody = _pweakbody.lock();
        if( !!pbody ) {
            boost::unique_lock< boost::shared_mutex > lock(pbody->GetInterfaceMutex());
            FOREACH(itinfo, _iterators) {
                pbody->_vlistRegisteredCallbacks.at(itinfo->first).erase(itinfo->second);
            }
        }
    }

    list< std::pair<uint32_t, list<UserDataWeakPtr>::iterator> > _iterators;
    int _properties;
    boost::function<void()> _callback;
protected:
    boost::weak_ptr<KinBody const> _pweakbody;
};

class CallFunctionAtDestructor
{
public:
    CallFunctionAtDestructor(const boost::function<void()>& fn) : _fn(fn) {
    }
    ~CallFunctionAtDestructor() {
        _fn();
    }

protected:
    boost::function<void()> _fn;
};

typedef boost::shared_ptr<ChangeCallbackData> ChangeCallbackDataPtr;

void ElectricMotorActuatorInfo::Reset()
{
    model_type.clear();
    assigned_power_rating = 0;
    max_speed = 0;
    no_load_speed = 0;
    stall_torque = 0;
    max_instantaneous_torque = 0;
    nominal_speed_torque_points.clear();
    max_speed_torque_points.clear();
    nominal_torque = 0;
    rotor_inertia = 0;
    torque_constant = 0;
    nominal_voltage = 0;
    speed_constant = 0;
    starting_current = 0;
    terminal_resistance = 0;
    gear_ratio = 0;
    coloumb_friction = 0;
    viscous_friction = 0;
}

void ElectricMotorActuatorInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    orjson::SetJsonValueByKey(value, "modelType", model_type, allocator);
    orjson::SetJsonValueByKey(value, "assignedPowerRating", assigned_power_rating, allocator);
    orjson::SetJsonValueByKey(value, "maxSpeed", max_speed, allocator);
    orjson::SetJsonValueByKey(value, "noLoadSpeed", no_load_speed, allocator);
    orjson::SetJsonValueByKey(value, "stallTorque", stall_torque, allocator);
    orjson::SetJsonValueByKey(value, "maxInstantaneousTorque", max_instantaneous_torque, allocator);
    orjson::SetJsonValueByKey(value, "nominalSpeedTorquePoints", nominal_speed_torque_points, allocator);
    orjson::SetJsonValueByKey(value, "maxSpeedTorquePoints", max_speed_torque_points, allocator);
    orjson::SetJsonValueByKey(value, "nominalTorque", nominal_torque, allocator);
    orjson::SetJsonValueByKey(value, "rotorInertia", rotor_inertia, allocator);
    orjson::SetJsonValueByKey(value, "torqueConstant", torque_constant, allocator);
    orjson::SetJsonValueByKey(value, "nominalVoltage", nominal_voltage, allocator);
    orjson::SetJsonValueByKey(value, "speedConstant", speed_constant, allocator);
    orjson::SetJsonValueByKey(value, "startingCurrent", starting_current, allocator);
    orjson::SetJsonValueByKey(value, "terminalResistance", terminal_resistance, allocator);
    orjson::SetJsonValueByKey(value, "gearRatio", gear_ratio, allocator);
    orjson::SetJsonValueByKey(value, "coloumbFriction", coloumb_friction, allocator);
    orjson::SetJsonValueByKey(value, "viscousFriction", viscous_friction, allocator);
}

void ElectricMotorActuatorInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "modelType", model_type);
    orjson::LoadJsonValueByKey(value, "assignedPowerRating", assigned_power_rating);
    orjson::LoadJsonValueByKey(value, "maxSpeed", max_speed);
    orjson::LoadJsonValueByKey(value, "noLoadSpeed", no_load_speed);
    orjson::LoadJsonValueByKey(value, "stallTorque", stall_torque);
    orjson::LoadJsonValueByKey(value, "maxInstantaneousTorque", max_instantaneous_torque);
    orjson::LoadJsonValueByKey(value, "nominalSpeedTorquePoints", nominal_speed_torque_points);
    orjson::LoadJsonValueByKey(value, "maxSpeedTorquePoints", max_speed_torque_points);
    orjson::LoadJsonValueByKey(value, "nominalTorque", nominal_torque);
    orjson::LoadJsonValueByKey(value, "rotorInertia", rotor_inertia);
    orjson::LoadJsonValueByKey(value, "torqueConstant", torque_constant);
    orjson::LoadJsonValueByKey(value, "nominalVoltage", nominal_voltage);
    orjson::LoadJsonValueByKey(value, "speedConstant", speed_constant);
    orjson::LoadJsonValueByKey(value, "startingCurrent", starting_current);
    orjson::LoadJsonValueByKey(value, "terminalResistance", terminal_resistance);
    orjson::LoadJsonValueByKey(value, "gearRatio", gear_ratio);
    orjson::LoadJsonValueByKey(value, "coloumbFriction", coloumb_friction);
    orjson::LoadJsonValueByKey(value, "viscousFriction", viscous_friction);
}

void KinBody::KinBodyInfo::Reset()
{
    _id.clear();
    _uri.clear();
    _name.clear();
    _referenceUri.clear();
    _transform = Transform();
    _dofValues.clear();
    _vGrabbedInfos.clear();
    _vLinkInfos.clear();
    _vJointInfos.clear();
    _mReadableInterfaces.clear();
}

void KinBody::KinBodyInfo::SerializeJSON(rapidjson::Value& rKinBodyInfo, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    rKinBodyInfo.SetObject();
    orjson::SetJsonValueByKey(rKinBodyInfo, "id", _id, allocator);
    orjson::SetJsonValueByKey(rKinBodyInfo, "name", _name, allocator);
    if (!_referenceUri.empty()) {
        if( options & ISO_ReferenceUriHint ) {
            orjson::SetJsonValueByKey(rKinBodyInfo, "referenceUriHint", _referenceUri, allocator);
        }
        else {
            orjson::SetJsonValueByKey(rKinBodyInfo, "referenceUri", _referenceUri, allocator);
        }
    }
    orjson::SetJsonValueByKey(rKinBodyInfo, "interfaceType", _interfaceType, allocator);
    orjson::SetJsonValueByKey(rKinBodyInfo, "transform", _transform, allocator);
    orjson::SetJsonValueByKey(rKinBodyInfo, "isRobot", _isRobot, allocator);

    if (_dofValues.size() > 0) {
        rapidjson::Value dofValues;
        dofValues.SetArray();
        dofValues.Reserve(_dofValues.size(), allocator);
        FOREACHC(itDofValue, _dofValues) {
            rapidjson::Value dofValue;
            orjson::SetJsonValueByKey(dofValue, "jointName", itDofValue->first.first, allocator);
            orjson::SetJsonValueByKey(dofValue, "jointAxis", itDofValue->first.second, allocator);
            orjson::SetJsonValueByKey(dofValue, "value", itDofValue->second, allocator);
            dofValues.PushBack(dofValue, allocator);
        }
        rKinBodyInfo.AddMember("dofValues", dofValues, allocator);
    }

    if (_vGrabbedInfos.size() > 0) {
        rapidjson::Value rGrabbedInfoValues;
        rGrabbedInfoValues.SetArray();
        rGrabbedInfoValues.Reserve(_vGrabbedInfos.size(), allocator);
        FOREACHC(it, _vGrabbedInfos) {
            rapidjson::Value grabbedInfoValue;
            (*it)->SerializeJSON(grabbedInfoValue, allocator, fUnitScale, options);
            rGrabbedInfoValues.PushBack(grabbedInfoValue, allocator);
        }
        rKinBodyInfo.AddMember("grabbed", rGrabbedInfoValues, allocator);
    }

    if (_vLinkInfos.size() > 0) {
        rapidjson::Value rLinkInfoValues;
        rLinkInfoValues.SetArray();
        rLinkInfoValues.Reserve(_vLinkInfos.size(), allocator);
        FOREACHC(it, _vLinkInfos) {
            rapidjson::Value linkInfoValue;
            (*it)->SerializeJSON(linkInfoValue, allocator, fUnitScale, options);
            rLinkInfoValues.PushBack(linkInfoValue, allocator);
        }
        rKinBodyInfo.AddMember("links", rLinkInfoValues, allocator);
    }

    if (_vJointInfos.size() > 0) {
        rapidjson::Value rJointInfoValues;
        rJointInfoValues.SetArray();
        rJointInfoValues.Reserve(_vJointInfos.size(), allocator);
        FOREACHC(it, _vJointInfos) {
            rapidjson::Value jointInfoValue;
            (*it)->SerializeJSON(jointInfoValue, allocator, fUnitScale, options);
            rJointInfoValues.PushBack(jointInfoValue, allocator);
        }
        rKinBodyInfo.AddMember("joints", rJointInfoValues, allocator);
    }

    if (_mReadableInterfaces.size() > 0) {
        rapidjson::Value rReadableInterfaces;
        rReadableInterfaces.SetObject();
        for(std::map<std::string, ReadablePtr>::const_iterator it = _mReadableInterfaces.begin(); it != _mReadableInterfaces.end(); it++) {
            rapidjson::Value rReadable;
            it->second->SerializeJSON(rReadable, allocator, fUnitScale, options);
            orjson::SetJsonValueByKey(rReadableInterfaces, it->first.c_str(), rReadable, allocator);
        }
        rKinBodyInfo.AddMember("readableInterfaces", rReadableInterfaces, allocator);
    }
}

void KinBody::KinBodyInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "name", _name);
    orjson::LoadJsonValueByKey(value, "id", _id);
    if( _id.empty() ) {
        _id = _name;
    }

    if( !(options & IDO_IgnoreReferenceUri) ) {
        orjson::LoadJsonValueByKey(value, "referenceUri", _referenceUri);
    }

    orjson::LoadJsonValueByKey(value, "interfaceType", _interfaceType);
    orjson::LoadJsonValueByKey(value, "isRobot", _isRobot);

    if (value.HasMember("grabbed")) {
        _vGrabbedInfos.reserve(value["grabbed"].Size() + _vGrabbedInfos.size());
        size_t iGrabbed = 0;
        for (rapidjson::Value::ConstValueIterator it = value["grabbed"].Begin(); it != value["grabbed"].End(); ++it, ++iGrabbed) {
            const rapidjson::Value& grabbedValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(grabbedValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(grabbedValue, "grabbedName");
            }
            UpdateOrCreateInfo(grabbedValue, id, _vGrabbedInfos, fUnitScale, options);
        }
    }

    if (value.HasMember("links")) {
        _vLinkInfos.reserve(value["links"].Size() + _vLinkInfos.size());
        for (rapidjson::Value::ConstValueIterator it = value["links"].Begin(); it != value["links"].End(); ++it) {
            const rapidjson::Value& linkValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(linkValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(linkValue, "name");
            }
            UpdateOrCreateInfo(linkValue, id, _vLinkInfos, fUnitScale, options);
        }
    }

    if (value.HasMember("joints")) {
        _vJointInfos.reserve(value["joints"].Size() + _vJointInfos.size());
        for (rapidjson::Value::ConstValueIterator it = value["joints"].Begin(); it != value["joints"].End(); ++it) {
            const rapidjson::Value& jointValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(jointValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(jointValue, "name");
            }
            UpdateOrCreateInfo(jointValue, id, _vJointInfos, fUnitScale, options);
        }
    }

    if (value.HasMember("dofValues")) {
        _dofValues.resize(0);
        for(rapidjson::Value::ConstValueIterator itr = value["dofValues"].Begin(); itr != value["dofValues"].End(); ++itr) {
            if (itr->IsObject() && itr->HasMember("jointName") && itr->HasMember("value")) {
                std::string jointName;
                int jointAxis = 0;
                dReal dofValue;
                orjson::LoadJsonValueByKey(*itr, "jointName", jointName);
                orjson::LoadJsonValueByKey(*itr, "jointAxis", jointAxis);
                orjson::LoadJsonValueByKey(*itr, "value", dofValue);
                _dofValues.emplace_back(std::make_pair(jointName, jointAxis), dofValue);
            }
        }
    }

    if (value.HasMember("readableInterfaces") && value["readableInterfaces"].IsObject()) {
        for (rapidjson::Value::ConstMemberIterator it = value["readableInterfaces"].MemberBegin(); it != value["readableInterfaces"].MemberEnd(); ++it) {
            _DeserializeReadableInterface(it->name.GetString(), it->value);
        }
    }

    if (value.HasMember("transform")) {
        orjson::LoadJsonValueByKey(value, "transform", _transform);
    }
}

void KinBody::KinBodyInfo::_DeserializeReadableInterface(const std::string& id, const rapidjson::Value& value) {
    BaseJSONReaderPtr pReader = RaveCallJSONReader(PT_KinBody, id, KinBodyPtr(), AttributesList());
    if (!!pReader) {
        pReader->DeserializeJSON(value);
        ReadablePtr pReadable = pReader->GetReadable();
        if (!!pReadable) {
            _mReadableInterfaces[id] = pReadable;
        }
    }
    else if (value.IsString()) {
        StringReadablePtr pReadable(new StringReadable(id, value.GetString()));
        _mReadableInterfaces[id] = pReadable;
    }
}

KinBody::KinBody(InterfaceType type, EnvironmentBasePtr penv) : InterfaceBase(type, penv)
{
    _nHierarchyComputed = 0;
    _nParametersChanged = 0;
    _bMakeJoinedLinksAdjacent = true;
    _environmentid = 0;
    _nNonAdjacentLinkCache = 0x80000000;
    _nUpdateStampId = 0;
    _bAreAllJoints1DOFAndNonCircular = false;
}

KinBody::~KinBody()
{
    RAVELOG_VERBOSE_FORMAT("env=%d, destructing kinbody '%s'", GetEnv()->GetId()%GetName());
    Destroy();
}

void KinBody::Destroy()
{
    ReleaseAllGrabbed();
    if( _listAttachedBodies.size() > 0 ) {
        // could be in the environment destructor?
        stringstream ss; ss << GetName() << " still has attached bodies: ";
        FOREACHC(it,_listAttachedBodies) {
            KinBodyPtr pattached = it->lock();
            if( !!pattached ) {
                ss << pattached->GetName();
            }
        }
        RAVELOG_VERBOSE(ss.str());
    }
    _listAttachedBodies.clear();

    _veclinks.clear();
    _vecjoints.clear();
    _vTopologicallySortedJoints.clear();
    _vTopologicallySortedJointsAll.clear();
    _vDOFOrderedJoints.clear();
    _vPassiveJoints.clear();
    _vJointsAffectingLinks.clear();
    _vDOFIndices.clear();

    _setAdjacentLinks.clear();
    _vInitialLinkTransformations.clear();
    _vAllPairsShortestPaths.clear();
    _vClosedLoops.clear();
    _vClosedLoopIndices.clear();
    _vForcedAdjacentLinks.clear();
    _nHierarchyComputed = 0;
    _nParametersChanged = 0;
    _pManageData.reset();

    _ResetInternalCollisionCache();
    _selfcollisionchecker.reset();
}

bool KinBody::InitFromBoxes(const std::vector<AABB>& vaabbs, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_info._name = "base";
    plink->_info._bStatic = true;
    size_t numvertices=0, numindices=0;
    FOREACHC(itab, vaabbs) {
        GeometryInfo info;
        info._type = GT_Box;
        info._t.trans = itab->pos;
        info._bVisible = visible;
        info._vGeomData = itab->extents;
        info._vDiffuseColor=Vector(1,0.5f,0.5f,1);
        info._vAmbientColor=Vector(0.1,0.0f,0.0f,0);
        Link::GeometryPtr geom(new Link::Geometry(plink,info));
        geom->_info.InitCollisionMesh();
        numvertices += geom->GetCollisionMesh().vertices.size();
        numindices += geom->GetCollisionMesh().indices.size();
        plink->_vGeometries.push_back(geom);
    }

    plink->_collision.vertices.reserve(numvertices);
    plink->_collision.indices.reserve(numindices);
    TriMesh trimesh;
    FOREACH(itgeom,plink->_vGeometries) {
        trimesh = (*itgeom)->GetCollisionMesh();
        trimesh.ApplyTransform((*itgeom)->GetTransform());
        plink->_collision.Append(trimesh);
    }
    _veclinks.push_back(plink);
    __struri = uri;
    return true;
}

bool KinBody::InitFromBoxes(const std::vector<OBB>& vobbs, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_info._name = "base";
    plink->_info._bStatic = true;
    size_t numvertices=0, numindices=0;
    FOREACHC(itobb, vobbs) {
        TransformMatrix tm;
        tm.trans = itobb->pos;
        tm.m[0] = itobb->right.x; tm.m[1] = itobb->up.x; tm.m[2] = itobb->dir.x;
        tm.m[4] = itobb->right.y; tm.m[5] = itobb->up.y; tm.m[6] = itobb->dir.y;
        tm.m[8] = itobb->right.z; tm.m[9] = itobb->up.z; tm.m[10] = itobb->dir.z;
        GeometryInfo info;
        info._type = GT_Box;
        info._t = tm;
        info._bVisible = visible;
        info._vGeomData = itobb->extents;
        info._vDiffuseColor=Vector(1,0.5f,0.5f,1);
        info._vAmbientColor=Vector(0.1,0.0f,0.0f,0);
        Link::GeometryPtr geom(new Link::Geometry(plink,info));
        geom->_info.InitCollisionMesh();
        numvertices += geom->GetCollisionMesh().vertices.size();
        numindices += geom->GetCollisionMesh().indices.size();
        plink->_vGeometries.push_back(geom);
    }

    plink->_collision.vertices.reserve(numvertices);
    plink->_collision.indices.reserve(numindices);
    TriMesh trimesh;
    FOREACH(itgeom,plink->_vGeometries) {
        trimesh = (*itgeom)->GetCollisionMesh();
        trimesh.ApplyTransform((*itgeom)->GetTransform());
        plink->_collision.Append(trimesh);
    }
    _veclinks.push_back(plink);
    __struri = uri;
    return true;
}

bool KinBody::InitFromSpheres(const std::vector<Vector>& vspheres, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_info._name = "base";
    plink->_info._bStatic = true;
    TriMesh trimesh;
    FOREACHC(itv, vspheres) {
        GeometryInfo info;
        info._type = GT_Sphere;
        info._t.trans.x = itv->x; info._t.trans.y = itv->y; info._t.trans.z = itv->z;
        info._bVisible = visible;
        info._vGeomData.x = itv->w;
        info._vDiffuseColor=Vector(1,0.5f,0.5f,1);
        info._vAmbientColor=Vector(0.1,0.0f,0.0f,0);
        Link::GeometryPtr geom(new Link::Geometry(plink,info));
        geom->_info.InitCollisionMesh();
        plink->_vGeometries.push_back(geom);
        trimesh = geom->GetCollisionMesh();
        trimesh.ApplyTransform(geom->GetTransform());
        plink->_collision.Append(trimesh);
    }
    _veclinks.push_back(plink);
    __struri = uri;
    return true;
}

bool KinBody::InitFromTrimesh(const TriMesh& trimesh, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_info._name = "base";
    plink->_info._bStatic = true;
    plink->_collision = trimesh;
    GeometryInfo info;
    info._type = GT_TriMesh;
    info._bVisible = visible;
    info._vDiffuseColor=Vector(1,0.5f,0.5f,1);
    info._vAmbientColor=Vector(0.1,0.0f,0.0f,0);
    info._meshcollision = trimesh;
    Link::GeometryPtr geom(new Link::Geometry(plink,info));
    plink->_vGeometries.push_back(geom);
    _veclinks.push_back(plink);
    __struri = uri;
    return true;
}

bool KinBody::InitFromGeometries(const std::list<KinBody::GeometryInfo>& geometries, const std::string& uri)
{
    std::vector<GeometryInfoConstPtr> newgeometries; newgeometries.reserve(geometries.size());
    FOREACHC(it, geometries) {
        newgeometries.push_back(GeometryInfoConstPtr(&(*it), utils::null_deleter()));
    }
    return InitFromGeometries(newgeometries, uri);
}

bool KinBody::InitFromGeometries(const std::vector<KinBody::GeometryInfoConstPtr>& geometries, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    OPENRAVE_ASSERT_OP(geometries.size(),>,0);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_info._name = "base";
    plink->_info._bStatic = true;
    FOREACHC(itinfo,geometries) {
        Link::GeometryPtr geom(new Link::Geometry(plink,**itinfo));
        geom->_info.InitCollisionMesh();
        plink->_vGeometries.push_back(geom);
        plink->_collision.Append(geom->GetCollisionMesh(),geom->GetTransform());
    }
    _veclinks.push_back(plink);
    __struri = uri;
    return true;
}

void KinBody::SetLinkGeometriesFromGroup(const std::string& geomname)
{
    // need to call _PostprocessChangedParameters at the very end, even if exception occurs
    CallFunctionAtDestructor callfn(boost::bind(&KinBody::_PostprocessChangedParameters, this, Prop_LinkGeometry));
    FOREACHC(itlink, _veclinks) {
        std::vector<KinBody::GeometryInfoPtr>* pvinfos = NULL;
        if( geomname.size() == 0 ) {
            pvinfos = &(*itlink)->_info._vgeometryinfos;
        }
        else {
            std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = (*itlink)->_info._mapExtraGeometries.find(geomname);
            if( it == (*itlink)->_info._mapExtraGeometries.end() ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("could not find geometries %s for link %s"),geomname%GetName(),ORE_InvalidArguments);
            }
            pvinfos = &it->second;
        }
        (*itlink)->_vGeometries.resize(pvinfos->size());
        for(size_t i = 0; i < pvinfos->size(); ++i) {
            (*itlink)->_vGeometries[i].reset(new Link::Geometry(*itlink,*pvinfos->at(i)));
            if( (*itlink)->_vGeometries[i]->GetCollisionMesh().vertices.size() == 0 ) { // try to avoid recomputing
                (*itlink)->_vGeometries[i]->InitCollisionMesh();
            }
        }
        (*itlink)->_Update(false);
    }
    // have to reset the adjacency cache
    _ResetInternalCollisionCache();
}

void KinBody::SetLinkGroupGeometries(const std::string& geomname, const std::vector< std::vector<KinBody::GeometryInfoPtr> >& linkgeometries)
{
    OPENRAVE_ASSERT_OP( linkgeometries.size(), ==, _veclinks.size() );
    FOREACH(itlink, _veclinks) {
        Link& link = **itlink;
        std::map< std::string, std::vector<KinBody::GeometryInfoPtr> >::iterator it = link._info._mapExtraGeometries.insert(make_pair(geomname,std::vector<KinBody::GeometryInfoPtr>())).first;
        const std::vector<KinBody::GeometryInfoPtr>& geometries = linkgeometries.at(link.GetIndex());
        it->second.resize(geometries.size());
        std::copy(geometries.begin(),geometries.end(),it->second.begin());
    }
    _PostprocessChangedParameters(Prop_LinkGeometryGroup); // have to notify collision checkers that the geometry info they are caching could have changed.
}

bool KinBody::Init(const std::vector<KinBody::LinkInfoConstPtr>& linkinfos, const std::vector<KinBody::JointInfoConstPtr>& jointinfos, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    _veclinks.reserve(linkinfos.size());
    FOREACHC(itlinkinfo, linkinfos) {
        LinkPtr plink(new Link(shared_kinbody()));
        plink->_info = **itlinkinfo;
        _InitAndAddLink(plink);
    }
    _vecjoints.reserve(jointinfos.size());
    FOREACHC(itjointinfo, jointinfos) {
        JointInfoConstPtr rawinfo = *itjointinfo;
        JointPtr pjoint(new Joint(shared_kinbody()));
        pjoint->_info = *rawinfo;
        _InitAndAddJoint(pjoint);
    }
    __struri = uri;
    return true;
}

void KinBody::InitFromLinkInfos(const std::vector<LinkInfo>& linkinfos, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    OPENRAVE_ASSERT_OP(linkinfos.size(),>,0);
    Destroy();
    _veclinks.reserve(linkinfos.size());
    FOREACHC(itlinkinfo, linkinfos) {
        LinkPtr plink(new Link(shared_kinbody()));
        plink->_info = *itlinkinfo;
        _InitAndAddLink(plink);
    }
    if( linkinfos.size() > 1 ) {
        // create static joints
        _vecjoints.clear();
        _vecjoints.reserve(linkinfos.size()-1);
        for(int ilinkinfo = 0; ilinkinfo+1 < (int)linkinfos.size(); ++ilinkinfo) {
            JointPtr pjoint(new Joint(shared_kinbody()));
            pjoint->_info._type = JointRevolute;
            pjoint->_info._name = "dummy";
            pjoint->_info._name += boost::lexical_cast<std::string>(ilinkinfo);
            pjoint->_info._linkname0 = linkinfos[ilinkinfo]._name;
            pjoint->_info._linkname1 = linkinfos[ilinkinfo+1]._name;
            pjoint->_info._bIsActive = false;
            _InitAndAddJoint(pjoint);
        }
    }
    __struri = uri;
}

bool KinBody::InitFromKinBodyInfo(const KinBodyInfo& info)
{
    std::vector<KinBody::LinkInfoConstPtr> vLinkInfosConst(info._vLinkInfos.begin(), info._vLinkInfos.end());
    std::vector<KinBody::JointInfoConstPtr> vJointInfosConst(info._vJointInfos.begin(), info._vJointInfos.end());
    if( !KinBody::Init(vLinkInfosConst, vJointInfosConst, info._uri) ) {
        return false;
    }

    _id = info._id;
    _name = info._name;
    _referenceUri = info._referenceUri;

    FOREACH(it, info._mReadableInterfaces) {
        SetReadableInterface(it->first, it->second);
    }

    if( GetXMLId() != info._interfaceType ) {
        RAVELOG_WARN_FORMAT("body '%s' interfaceType does not match %s != %s", GetName()%GetXMLId()%info._interfaceType);
    }

    return true;
}

void KinBody::SetName(const std::string& newname)
{
    OPENRAVE_ASSERT_OP(newname.size(), >, 0);
    if( _name != newname ) {
        // have to replace the 2nd word of all the groups with the robot name
        FOREACH(itgroup, _spec._vgroups) {
            stringstream ss(itgroup->name);
            string grouptype, oldname;
            ss >> grouptype >> oldname;
            stringbuf buf;
            ss.get(buf,0);
            itgroup->name = str(boost::format("%s %s %s")%grouptype%newname%buf.str());
        }
        _name = newname;
        _PostprocessChangedParameters(Prop_Name);
    }
}

void KinBody::SetDOFTorques(const std::vector<dReal>& torques, bool bAdd)
{
    OPENRAVE_ASSERT_OP_FORMAT((int)torques.size(), >=, GetDOF(), "not enough values %d<%d", torques.size()%GetDOF(),ORE_InvalidArguments);
    if( !bAdd ) {
        FOREACH(itlink, _veclinks) {
            (*itlink)->SetForce(Vector(),Vector(),false);
            (*itlink)->SetTorque(Vector(),false);
        }
    }
    std::vector<dReal> jointtorques;
    FOREACH(it, _vecjoints) {
        jointtorques.resize((*it)->GetDOF());
        std::copy(torques.begin()+(*it)->GetDOFIndex(),torques.begin()+(*it)->GetDOFIndex()+(*it)->GetDOF(),jointtorques.begin());
        (*it)->AddTorque(jointtorques);
    }
}

int KinBody::GetDOF() const
{
    return _vecjoints.size() > 0 ? _vecjoints.back()->GetDOFIndex()+_vecjoints.back()->GetDOF() : 0;
}

void KinBody::GetDOFValues(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    if( dofindices.size() == 0 ) {
        v.clear();
        v.reserve(GetDOF());
        FOREACHC(it, _vDOFOrderedJoints) {
            int toadd = (*it)->GetDOFIndex()-(int)v.size();
            if( toadd > 0 ) {
                v.insert(v.end(),toadd,0);
            }
            else if( toadd < 0 ) {
                std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                ss << "values=[";
                FOREACH(itvalue, v) {
                    ss << *itvalue << ", ";
                }
                ss << "]; jointorder=[";
                FOREACH(itj, _vDOFOrderedJoints) {
                    ss << (*itj)->GetName() << ", ";
                }
                ss << "];";
                throw OPENRAVE_EXCEPTION_FORMAT(_("dof indices mismatch joint %s (dofindex=%d), toadd=%d, v.size()=%d in call GetDOFValues with %s"), (*it)->GetName()%(*it)->GetDOFIndex()%toadd%v.size()%ss.str(), ORE_InvalidState);
            }
            (*it)->GetValues(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetValue(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFVelocities(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, _vDOFOrderedJoints) {
            (*it)->GetVelocities(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetVelocity(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        vLowerLimit.resize(0);
        if( (int)vLowerLimit.capacity() < GetDOF() ) {
            vLowerLimit.reserve(GetDOF());
        }
        vUpperLimit.resize(0);
        if( (int)vUpperLimit.capacity() < GetDOF() ) {
            vUpperLimit.reserve(GetDOF());
        }
        FOREACHC(it,_vDOFOrderedJoints) {
            (*it)->GetLimits(vLowerLimit,vUpperLimit,true);
        }
    }
    else {
        vLowerLimit.resize(dofindices.size());
        vUpperLimit.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            std::pair<dReal, dReal> res = pjoint->GetLimit(dofindices[i]-pjoint->GetDOFIndex());
            vLowerLimit[i] = res.first;
            vUpperLimit[i] = res.second;
        }
    }
}

void KinBody::GetDOFVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        vlower.resize(0);
        vupper.resize(0);
        if( (int)vlower.capacity() < GetDOF() ) {
            vlower.reserve(GetDOF());
        }
        if( (int)vupper.capacity() < GetDOF() ) {
            vupper.reserve(GetDOF());
        }
        FOREACHC(it,_vDOFOrderedJoints) {
            (*it)->GetVelocityLimits(vlower,vupper,true);
        }
    }
    else {
        vlower.resize(dofindices.size());
        vupper.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            std::pair<dReal, dReal> res = pjoint->GetVelocityLimit(dofindices[i]-pjoint->GetDOFIndex());
            vlower[i] = res.first;
            vupper[i] = res.second;
        }
    }
}

void KinBody::GetDOFVelocityLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, _vDOFOrderedJoints) {
            (*it)->GetVelocityLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetMaxVel(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFAccelerationLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, _vDOFOrderedJoints) {
            (*it)->GetAccelerationLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetAccelerationLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFJerkLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, _vDOFOrderedJoints) {
            (*it)->GetJerkLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetJerkLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFHardVelocityLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, _vDOFOrderedJoints) {
            (*it)->GetHardVelocityLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetHardVelocityLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFHardAccelerationLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, _vDOFOrderedJoints) {
            (*it)->GetHardAccelerationLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetHardAccelerationLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFHardJerkLimits(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, _vDOFOrderedJoints) {
            (*it)->GetHardJerkLimits(v,true);
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetHardJerkLimit(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFTorqueLimits(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() ) {
        v.reserve(GetDOF());
    }
    FOREACHC(it, _vDOFOrderedJoints) {
        (*it)->GetTorqueLimits(v,true);
    }
}

void KinBody::GetDOFMaxTorque(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() ) {
        v.reserve(GetDOF());
    }
    FOREACHC(it, _vDOFOrderedJoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxTorque());
    }
}

void KinBody::GetDOFResolutions(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(0);
        if( (int)v.capacity() < GetDOF() )
            v.reserve(GetDOF());
        FOREACHC(it, _vDOFOrderedJoints) {
            v.insert(v.end(),(*it)->GetDOF(),(*it)->GetResolution());
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetResolution(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::GetDOFWeights(std::vector<dReal>& v, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        v.resize(GetDOF());
        std::vector<dReal>::iterator itv = v.begin();
        FOREACHC(it, _vDOFOrderedJoints) {
            for(int i = 0; i < (*it)->GetDOF(); ++i) {
                *itv++ = (*it)->GetWeight(i);
            }
        }
    }
    else {
        v.resize(dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            v[i] = pjoint->GetWeight(dofindices[i]-pjoint->GetDOFIndex());
        }
    }
}

void KinBody::SetDOFWeights(const std::vector<dReal>& v, const std::vector<int>& dofindices)
{
    if( dofindices.size() == 0 ) {
        OPENRAVE_ASSERT_OP((int)v.size(),>=,GetDOF());
        for(int i = 0; i < GetDOF(); ++i) {
            OPENRAVE_ASSERT_OP_FORMAT(v[i], >, 0, "dof %d weight %f has to be >= 0", i%v[i], ORE_InvalidArguments);
        }
        std::vector<dReal>::const_iterator itv = v.begin();
        FOREACHC(it, _vDOFOrderedJoints) {
            std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vweights.begin());
            itv += (*it)->GetDOF();
        }
    }
    else {
        OPENRAVE_ASSERT_OP(v.size(),==,dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            pjoint->_info._vweights.at(dofindices[i]-pjoint->GetDOFIndex()) = v[i];
        }
    }
    _PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::SetDOFResolutions(const std::vector<dReal>& v, const std::vector<int>& dofindices)
{
    if( dofindices.size() == 0 ) {
        OPENRAVE_ASSERT_OP((int)v.size(),>=,GetDOF());
        for(int i = 0; i < GetDOF(); ++i) {
            OPENRAVE_ASSERT_OP_FORMAT(v[i], >, 0, "dof %d resolution %f has to be >= 0", i%v[i], ORE_InvalidArguments);
        }
        std::vector<dReal>::const_iterator itv = v.begin();
        FOREACHC(it, _vDOFOrderedJoints) {
            std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vresolution.begin());
            itv += (*it)->GetDOF();
        }
    }
    else {
        OPENRAVE_ASSERT_OP(v.size(),==,dofindices.size());
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            pjoint->_info._vresolution.at(dofindices[i]-pjoint->GetDOFIndex()) = v[i];
        }
    }
    _PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::SetDOFLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper, const std::vector<int>& dofindices)
{
    bool bChanged = false;
    if( dofindices.size() == 0 ) {
        OPENRAVE_ASSERT_OP((int)lower.size(),==,GetDOF());
        OPENRAVE_ASSERT_OP((int)upper.size(),==,GetDOF());
        std::vector<dReal>::const_iterator itlower = lower.begin(), itupper = upper.begin();
        FOREACHC(it, _vDOFOrderedJoints) {
            for(int i = 0; i < (*it)->GetDOF(); ++i) {
                if( (*it)->_info._vlowerlimit.at(i) != *(itlower+i) || (*it)->_info._vupperlimit.at(i) != *(itupper+i) ) {
                    bChanged = true;
                    std::copy(itlower,itlower+(*it)->GetDOF(), (*it)->_info._vlowerlimit.begin());
                    std::copy(itupper,itupper+(*it)->GetDOF(), (*it)->_info._vupperlimit.begin());
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        if( (*it)->IsRevolute(i) && !(*it)->IsCircular(i) ) {
                            // TODO, necessary to set wrap?
                            if( (*it)->_info._vlowerlimit.at(i) < -PI || (*it)->_info._vupperlimit.at(i) > PI) {
                                (*it)->SetWrapOffset(0.5f * ((*it)->_info._vlowerlimit.at(i) + (*it)->_info._vupperlimit.at(i)),i);
                            }
                            else {
                                (*it)->SetWrapOffset(0,i);
                            }
                        }
                    }
                    break;
                }
            }
            itlower += (*it)->GetDOF();
            itupper += (*it)->GetDOF();
        }
    }
    else {
        OPENRAVE_ASSERT_OP(lower.size(),==,dofindices.size());
        OPENRAVE_ASSERT_OP(upper.size(),==,dofindices.size());
        for(size_t index = 0; index < dofindices.size(); ++index) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[index]);
            int iaxis = dofindices[index]-pjoint->GetDOFIndex();
            if( pjoint->_info._vlowerlimit.at(iaxis) != lower[index] || pjoint->_info._vupperlimit.at(iaxis) != upper[index] ) {
                bChanged = true;
                pjoint->_info._vlowerlimit.at(iaxis) = lower[index];
                pjoint->_info._vupperlimit.at(iaxis) = upper[index];
                if( pjoint->IsRevolute(iaxis) && !pjoint->IsCircular(iaxis) ) {
                    // TODO, necessary to set wrap?
                    if( pjoint->_info._vlowerlimit.at(iaxis) < -PI || pjoint->_info._vupperlimit.at(iaxis) > PI) {
                        pjoint->SetWrapOffset(0.5f * (pjoint->_info._vlowerlimit.at(iaxis) + pjoint->_info._vupperlimit.at(iaxis)),iaxis);
                    }
                    else {
                        pjoint->SetWrapOffset(0,iaxis);
                    }
                }
            }
        }
    }
    if( bChanged ) {
        _PostprocessChangedParameters(Prop_JointLimits);
    }
}

void KinBody::SetDOFVelocityLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vmaxvel.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFAccelerationLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vmaxaccel.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFJerkLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vmaxjerk.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFHardVelocityLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vhardmaxvel.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFHardAccelerationLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vhardmaxaccel.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFHardJerkLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vhardmaxjerk.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFTorqueLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vmaxtorque.begin());
        itv += (*it)->GetDOF();
    }
    _PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SimulationStep(dReal fElapsedTime)
{
    _UpdateGrabbedBodies();
}

void KinBody::SubtractDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2, const std::vector<int>& dofindices) const
{
    OPENRAVE_ASSERT_OP(q1.size(), ==, q2.size() );
    if (_bAreAllJoints1DOFAndNonCircular) {
        for(size_t i = 0; i < q1.size(); ++i) {
            q1[i] -= q2[i];
        }
        return;
    }

    if( dofindices.size() == 0 ) {
        OPENRAVE_ASSERT_OP((int)q1.size(), ==, GetDOF() );
        FOREACHC(itjoint,_vecjoints) {
            int dof = (*itjoint)->GetDOFIndex();
            for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                if( (*itjoint)->IsCircular(i) ) {
                    q1[dof+i] = utils::NormalizeCircularAngle(q1[dof+i]-q2[dof+i],(*itjoint)->_vcircularlowerlimit.at(i), (*itjoint)->_vcircularupperlimit.at(i));
                }
                else {
                    q1[dof+i] -= q2[dof+i];
                }
            }
        }
    }
    else {
        OPENRAVE_ASSERT_OP(q1.size(), ==, dofindices.size() );
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            if( pjoint->IsCircular(dofindices[i]-pjoint->GetDOFIndex()) ) {
                int iaxis = dofindices[i]-pjoint->GetDOFIndex();
                q1[i] = utils::NormalizeCircularAngle(q1[i]-q2[i], pjoint->_vcircularlowerlimit.at(iaxis), pjoint->_vcircularupperlimit.at(iaxis));
            }
            else {
                q1[i] -= q2[i];
            }
        }
    }
}

// like apply transform except everything is relative to the first frame
void KinBody::SetTransform(const Transform& trans)
{
    if( _veclinks.size() == 0 ) {
        return;
    }
    Transform tbaseinv = _veclinks.front()->GetTransform().inverse();
    Transform tapply = trans * tbaseinv;
    FOREACH(itlink, _veclinks) {
        (*itlink)->SetTransform(tapply * (*itlink)->GetTransform());
    }
    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

Transform KinBody::GetTransform() const
{
    return _veclinks.size() > 0 ? _veclinks.front()->GetTransform() : Transform();
}

bool KinBody::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    if( _veclinks.size() > 0 ) {
        std::vector<std::pair<Vector,Vector> > velocities(_veclinks.size());
        velocities.at(0).first = linearvel;
        velocities.at(0).second = angularvel;
        Vector vlinktrans = _veclinks.at(0)->GetTransform().trans;
        for(size_t i = 1; i < _veclinks.size(); ++i) {
            velocities[i].first = linearvel + angularvel.cross(_veclinks[i]->GetTransform().trans-vlinktrans);
            velocities[i].second = angularvel;
        }

        bool bSuccess = GetEnv()->GetPhysicsEngine()->SetLinkVelocities(shared_kinbody(),velocities);
        _UpdateGrabbedBodies();
        return bSuccess;
    }
    return false;
}

void KinBody::SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, const Vector& linearvel, const Vector& angularvel, uint32_t checklimits)
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_OP_FORMAT((int)vDOFVelocities.size(), >=, GetDOF(), "env=%d, not enough values %d!=%d", GetEnv()->GetId()%vDOFVelocities.size()%GetDOF(),ORE_InvalidArguments);
    std::vector<std::pair<Vector,Vector> > velocities(_veclinks.size());
    velocities.at(0).first = linearvel;
    velocities.at(0).second = angularvel;

    vector<dReal> vlower,vupper,vtempvalues, veval;
    if( checklimits != CLA_Nothing ) {
        GetDOFVelocityLimits(vlower,vupper);
    }

    // have to compute the velocities ahead of time since they are dependent on the link transformations
    std::vector< std::vector<dReal> > vPassiveJointVelocities(_vPassiveJoints.size());
    for(size_t i = 0; i < vPassiveJointVelocities.size(); ++i) {
        if( !_vPassiveJoints[i]->IsMimic() ) {
            _vPassiveJoints[i]->GetVelocities(vPassiveJointVelocities[i]);
        }
        else {
            vPassiveJointVelocities[i].resize(_vPassiveJoints[i]->GetDOF(),0);
        }
    }

    std::vector<uint8_t> vlinkscomputed(_veclinks.size(),0);
    vlinkscomputed[0] = 1;
    boost::array<dReal,3> dummyvalues; // dummy values for a joint

    for(size_t ijoint = 0; ijoint < _vTopologicallySortedJointsAll.size(); ++ijoint) {
        JointPtr pjoint = _vTopologicallySortedJointsAll[ijoint];
        int jointindex = _vTopologicallySortedJointIndicesAll[ijoint];
        int dofindex = pjoint->GetDOFIndex();
        const dReal* pvalues=dofindex >= 0 ? &vDOFVelocities.at(dofindex) : NULL;
        if( pjoint->IsMimic() ) {
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pjoint->IsMimic(i) ) {
                    vtempvalues.resize(0);
                    const std::vector<Mimic::DOFFormat>& vdofformat = pjoint->_vmimic[i]->_vdofformat;
                    FOREACHC(itdof,vdofformat) {
                        JointPtr pj = itdof->jointindex < (int)_vecjoints.size() ? _vecjoints[itdof->jointindex] : _vPassiveJoints.at(itdof->jointindex-_vecjoints.size());
                        vtempvalues.push_back(pj->GetValue(itdof->axis));
                    }
                    dummyvalues[i] = 0;
                    int err = pjoint->_Eval(i,1,vtempvalues,veval);
                    if( err ) {
                        RAVELOG_WARN(str(boost::format("env=%d, failed to evaluate joint %s, fparser error %d")%GetEnv()->GetId()%pjoint->GetName()%err));
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            err = pjoint->_Eval(i,1,vtempvalues,veval);
                        }
                    }
                    else {
                        for(size_t ipartial = 0; ipartial < vdofformat.size(); ++ipartial) {
                            dReal partialvelocity;
                            if( vdofformat[ipartial].dofindex >= 0 ) {
                                partialvelocity = vDOFVelocities.at(vdofformat[ipartial].dofindex);
                            }
                            else {
                                partialvelocity = vPassiveJointVelocities.at(vdofformat[ipartial].jointindex-_vecjoints.size()).at(vdofformat[ipartial].axis);
                            }
                            if( ipartial < veval.size() ) {
                                dummyvalues[i] += veval.at(ipartial) * partialvelocity;
                            }
                            else {
                                RAVELOG_DEBUG_FORMAT("env=%d, cannot evaluate partial velocity for mimic joint %s, perhaps equations don't exist", GetEnv()->GetId()%pjoint->GetName());
                            }
                        }
                    }

                    // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                    if( dofindex < 0 ) {
                        vPassiveJointVelocities.at(jointindex-(int)_vecjoints.size()).at(i) = dummyvalues[i];
                    }
                }
                else if( dofindex >= 0 ) {
                    dummyvalues[i] = vDOFVelocities.at(dofindex+i); // is this correct? what is a joint has a mimic and non-mimic axis?
                }
                else {
                    // preserve passive joint values
                    dummyvalues[i] = vPassiveJointVelocities.at(jointindex-(int)_vecjoints.size()).at(i);
                }
            }
            pvalues = &dummyvalues[0];
        }
        // do the test after mimic computation!
        if( vlinkscomputed[pjoint->GetHierarchyChildLink()->GetIndex()] ) {
            continue;
        }
        if( !pvalues ) {
            // has to be a passive joint
            pvalues = &vPassiveJointVelocities.at(jointindex-(int)_vecjoints.size()).at(0);
        }

        if( checklimits != CLA_Nothing && dofindex >= 0 ) {
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pvalues[i] < vlower.at(dofindex+i)-g_fEpsilonJointLimit ) {
                    if( checklimits == CLA_CheckLimits ) {
                        RAVELOG_WARN(str(boost::format("env=%d, dof %d velocity is not in limits %.15e<%.15e")%GetEnv()->GetId()%(dofindex+i)%pvalues[i]%vlower.at(dofindex+i)));
                    }
                    else if( checklimits == CLA_CheckLimitsThrow ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, dof %d velocity is not in limits %.15e<%.15e"), GetEnv()->GetId()%(dofindex+i)%pvalues[i]%vlower.at(dofindex+i), ORE_InvalidArguments);
                    }
                    dummyvalues[i] = vlower[dofindex+i];
                }
                else if( pvalues[i] > vupper.at(dofindex+i)+g_fEpsilonJointLimit ) {
                    if( checklimits == CLA_CheckLimits ) {
                        RAVELOG_WARN(str(boost::format("env=%d, dof %d velocity is not in limits %.15e>%.15e")%GetEnv()->GetId()%(dofindex+i)%pvalues[i]%vupper.at(dofindex+i)));
                    }
                    else if( checklimits == CLA_CheckLimitsThrow ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, dof %d velocity is not in limits %.15e>%.15e"), GetEnv()->GetId()%(dofindex+i)%pvalues[i]%vupper.at(dofindex+i), ORE_InvalidArguments);
                    }
                    dummyvalues[i] = vupper[dofindex+i];
                }
                else {
                    dummyvalues[i] = pvalues[i];
                }
            }
            pvalues = &dummyvalues[0];
        }

        // compute for global coordinate system
        Vector vparent, wparent;
        Transform tparent;
        if( !pjoint->GetHierarchyParentLink() ) {
            tparent = _veclinks.at(0)->GetTransform();
            vparent = velocities.at(0).first;
            wparent = velocities.at(0).second;
        }
        else {
            tparent = pjoint->GetHierarchyParentLink()->GetTransform();
            vparent = velocities[pjoint->GetHierarchyParentLink()->GetIndex()].first;
            wparent = velocities[pjoint->GetHierarchyParentLink()->GetIndex()].second;
        }

        int childindex = pjoint->GetHierarchyChildLink()->GetIndex();
        Transform tchild = pjoint->GetHierarchyChildLink()->GetTransform();
        Vector xyzdelta = tchild.trans - tparent.trans;
        Transform tdelta = tparent * pjoint->GetInternalHierarchyLeftTransform();
//        if( pjoint->GetType() & JointSpecialBit ) {
//            switch(pjoint->GetType()) {
//            case JointHinge2: {
//                Transform tfirst;
//                tfirst.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pjoint->GetValue(0));
//                w = pvalues[0]*pjoint->GetInternalHierarchyAxis(0) + tfirst.rotate(pvalues[1]*pjoint->GetInternalHierarchyAxis(1));
//                break;
//            }
//            case JointSpherical:
//                w.x = pvalues[0]; w.y = pvalues[1]; w.z = pvalues[2];
//                break;
//            default:
//                RAVELOG_WARN(str(boost::format("env=%d, forward kinematic type %d not supported")%GetEnv()->GetId()%pjoint->GetType()));
//                break;
//            }
//        }
//        else {
        if( pjoint->GetType() == JointRevolute ) {
            Vector gw = tdelta.rotate(pvalues[0]*pjoint->GetInternalHierarchyAxis(0));
            velocities.at(childindex) = make_pair(vparent + wparent.cross(xyzdelta) + gw.cross(tchild.trans-tdelta.trans), wparent + gw);
        }
        else if( pjoint->GetType() == JointPrismatic ) {
            velocities.at(childindex) = make_pair(vparent + wparent.cross(xyzdelta) + tdelta.rotate(pvalues[0]*pjoint->GetInternalHierarchyAxis(0)), wparent);
        }
        else if( pjoint->GetType() == JointTrajectory ) {
            Transform tlocalvelocity, tlocal;
            if( pjoint->IsMimic(0) ) {
                // vtempvalues should already be init from previous _Eval call
                int err = pjoint->_Eval(0,0,vtempvalues,veval);
                if( err != 0 ) {
                    RAVELOG_WARN(str(boost::format("env=%d, error with evaluation of joint %s")%GetEnv()->GetId()%pjoint->GetName()));
                }
                dReal fvalue = veval[0];
                if( pjoint->IsCircular(0) ) {
                    fvalue = utils::NormalizeCircularAngle(fvalue,pjoint->_vcircularlowerlimit.at(0), pjoint->_vcircularupperlimit.at(0));
                }
                pjoint->_info._trajfollow->Sample(vtempvalues,fvalue);
            }
            else {
                // calling GetValue() could be extremely slow
                pjoint->_info._trajfollow->Sample(vtempvalues,pjoint->GetValue(0));
            }
            pjoint->_info._trajfollow->GetConfigurationSpecification().ExtractTransform(tlocal, vtempvalues.begin(), KinBodyConstPtr(),0);
            pjoint->_info._trajfollow->GetConfigurationSpecification().ExtractTransform(tlocalvelocity, vtempvalues.begin(), KinBodyConstPtr(),1);
            Vector gw = tdelta.rotate(quatMultiply(tlocalvelocity.rot, quatInverse(tlocal.rot))*2*pvalues[0]); // qvel = [0,axisangle] * qrot * 0.5 * vel
            gw = Vector(gw.y,gw.z,gw.w);
            Vector gv = tdelta.rotate(tlocalvelocity.trans*pvalues[0]);
            velocities.at(childindex) = make_pair(vparent + wparent.cross(xyzdelta) + gw.cross(tchild.trans-tdelta.trans) + gv, wparent + gw);
        }
        else if( pjoint->GetType() == JointSpherical ) {
            Vector gw = tdelta.rotate(Vector(pvalues[0],pvalues[1],pvalues[2]));
            velocities.at(childindex) = make_pair(vparent + wparent.cross(xyzdelta) + gw.cross(tchild.trans-tdelta.trans), wparent + gw);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, joint 0x%x not supported for querying velocities"),GetEnv()->GetId()%pjoint->GetType(),ORE_Assert);
//                //todo
//                Transform tjoint;
//                for(int iaxis = 0; iaxis < pjoint->GetDOF(); ++iaxis) {
//                    Transform tdelta;
//                    if( pjoint->IsRevolute(iaxis) ) {
//                        w += tjoint.rotate(pvalues[iaxis]*pjoint->GetInternalHierarchyAxis(iaxis));
//                        tdelta.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(iaxis), pvalues[iaxis]);
//                    }
//                    else {
//                        tdelta.trans = pjoint->GetInternalHierarchyAxis(iaxis) * pvalues[iaxis];
//                        v += tjoint.rotate(pvalues[iaxis]*pjoint->GetInternalHierarchyAxis(iaxis)) + w.cross(tdelta.trans);
//                    }
//                    tjoint = tjoint * tdelta;
//                }
        }
//        }


        vlinkscomputed[childindex] = 1;
    }
    SetLinkVelocities(velocities);
}

void KinBody::SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, uint32_t checklimits, const std::vector<int>& dofindices)
{
    Vector linearvel,angularvel;
    _veclinks.at(0)->GetVelocity(linearvel,angularvel);
    if( dofindices.size() == 0 ) {
        return SetDOFVelocities(vDOFVelocities,linearvel,angularvel,checklimits);
    }

    // check if all dofindices are supplied
    if( (int)dofindices.size() == GetDOF() ) {
        bool bordereddof = true;
        for(size_t i = 0; i < dofindices.size(); ++i) {
            if( dofindices[i] != (int)i ) {
                bordereddof = false;
                break;
            }
        }
        if( bordereddof ) {
            return SetDOFVelocities(vDOFVelocities,linearvel,angularvel,checklimits);
        }
    }
    OPENRAVE_ASSERT_OP_FORMAT(vDOFVelocities.size(),==,dofindices.size(),"env=%d, index sizes do not match %d!=%d", GetEnv()->GetId()%vDOFVelocities.size()%dofindices.size(), ORE_InvalidArguments);
    // have to recreate the correct vector
    std::vector<dReal> vfulldof(GetDOF());
    std::vector<int>::const_iterator it;
    for(size_t i = 0; i < dofindices.size(); ++i) {
        it = find(dofindices.begin(), dofindices.end(), i);
        if( it != dofindices.end() ) {
            vfulldof[i] = vDOFVelocities.at(static_cast<size_t>(it-dofindices.begin()));
        }
        else {
            JointPtr pjoint = GetJointFromDOFIndex(i);
            if( !!pjoint ) {
                vfulldof[i] = _vecjoints.at(_vDOFIndices.at(i))->GetVelocity(i-_vDOFIndices.at(i));
            }
        }
    }
    return SetDOFVelocities(vfulldof,linearvel,angularvel,checklimits);
}

void KinBody::GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const
{
    GetEnv()->GetPhysicsEngine()->GetLinkVelocities(shared_kinbody_const(),velocities);
}

void KinBody::GetLinkTransformations(vector<Transform>& vtrans) const
{
    if( RaveGetDebugLevel() & Level_VerifyPlans ) {
        RAVELOG_VERBOSE("GetLinkTransformations should be called with doflastsetvalues\n");
    }
    vtrans.resize(_veclinks.size());
    vector<Transform>::iterator it;
    vector<LinkPtr>::const_iterator itlink;
    for(it = vtrans.begin(), itlink = _veclinks.begin(); it != vtrans.end(); ++it, ++itlink) {
        *it = (*itlink)->GetTransform();
    }
}

void KinBody::GetLinkTransformations(std::vector<Transform>& transforms, std::vector<dReal>& doflastsetvalues) const
{
    transforms.resize(_veclinks.size());
    vector<Transform>::iterator it;
    vector<LinkPtr>::const_iterator itlink;
    for(it = transforms.begin(), itlink = _veclinks.begin(); it != transforms.end(); ++it, ++itlink) {
        *it = (*itlink)->GetTransform();
    }

    doflastsetvalues.resize(0);
    if( (int)doflastsetvalues.capacity() < GetDOF() ) {
        doflastsetvalues.reserve(GetDOF());
    }
    FOREACHC(it, _vDOFOrderedJoints) {
        int toadd = (*it)->GetDOFIndex()-(int)doflastsetvalues.size();
        if( toadd > 0 ) {
            doflastsetvalues.insert(doflastsetvalues.end(),toadd,0);
        }
        else if( toadd < 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("dof indices mismatch joint %s, toadd=%d"), (*it)->GetName()%toadd, ORE_InvalidState);
        }
        for(int i = 0; i < (*it)->GetDOF(); ++i) {
            doflastsetvalues.push_back((*it)->_doflastsetvalues[i]);
        }
    }
}

void KinBody::GetLinkEnableStates(std::vector<uint8_t>& enablestates) const
{
    enablestates.resize(_veclinks.size());
    for(size_t ilink = 0; ilink < _veclinks.size(); ++ilink) {
        enablestates[ilink] = _veclinks[ilink]->IsEnabled();
    }
}

uint64_t KinBody::GetLinkEnableStatesMask() const
{
    if( _veclinks.size() > 64 ) {
        RAVELOG_WARN_FORMAT("%s has too many links and will only return enable mask for first 64", _name);
    }
    uint64_t linkstate = 0;
    for(size_t ilink = 0; ilink < _veclinks.size(); ++ilink) {
        linkstate |= ((uint64_t)_veclinks[ilink]->_info._bIsEnabled<<ilink);
    }
    return linkstate;
}

KinBody::JointPtr KinBody::GetJointFromDOFIndex(int dofindex) const
{
    return _vecjoints.at(_vDOFIndices.at(dofindex));
}

AABB KinBody::ComputeAABB(bool bEnabledOnlyLinks) const
{
    Vector vmin, vmax;
    bool binitialized=false;
    AABB ab;
    FOREACHC(itlink,_veclinks) {
        if( bEnabledOnlyLinks && !(*itlink)->IsEnabled() ) {
            continue;
        }
        ab = (*itlink)->ComputeAABB();
        if((ab.extents.x == 0)&&(ab.extents.y == 0)&&(ab.extents.z == 0)) {
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
        ab.pos = GetTransform().trans;
        ab.extents = Vector(0,0,0);
    }
    else {
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
    }
    return ab;
}

AABB KinBody::ComputeAABBFromTransform(const Transform& tBody, bool bEnabledOnlyLinks) const
{
    Vector vmin, vmax;
    bool binitialized=false;
    AABB ablocal;
    Transform tConvertToNewFrame = tBody*GetTransform().inverse();
    FOREACHC(itlink,_veclinks) {
        if( bEnabledOnlyLinks && !(*itlink)->IsEnabled() ) {
            continue;
        }
        ablocal = (*itlink)->ComputeLocalAABB();
        if( ablocal.extents.x == 0 && ablocal.extents.y == 0 && ablocal.extents.z == 0 ) {
            continue;
        }

        Transform tlink = tConvertToNewFrame*(*itlink)->GetTransform();
        TransformMatrix mlink(tlink);
        Vector projectedExtents(RaveFabs(mlink.m[0]*ablocal.extents[0]) + RaveFabs(mlink.m[1]*ablocal.extents[1]) + RaveFabs(mlink.m[2]*ablocal.extents[2]),
                                RaveFabs(mlink.m[4]*ablocal.extents[0]) + RaveFabs(mlink.m[5]*ablocal.extents[1]) + RaveFabs(mlink.m[6]*ablocal.extents[2]),
                                RaveFabs(mlink.m[8]*ablocal.extents[0]) + RaveFabs(mlink.m[9]*ablocal.extents[1]) + RaveFabs(mlink.m[10]*ablocal.extents[2]));
        Vector vWorldPos = tlink * ablocal.pos;

        Vector vnmin = vWorldPos - projectedExtents;
        Vector vnmax = vWorldPos + projectedExtents;
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

    AABB ab;
    if( !binitialized ) {
        ab.pos = GetTransform().trans;
        ab.extents = Vector(0,0,0);
    }
    else {
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
    }
    return ab;
}

AABB KinBody::ComputeLocalAABB(bool bEnabledOnlyLinks) const
{
    return ComputeAABBFromTransform(Transform(), bEnabledOnlyLinks);
}

Vector KinBody::GetCenterOfMass() const
{
    // find center of mass and set the outer transform to it
    Vector center;
    dReal fTotalMass = 0;

    FOREACHC(itlink, _veclinks) {
        center += ((*itlink)->GetTransform() * (*itlink)->GetCOMOffset() * (*itlink)->GetMass());
        fTotalMass += (*itlink)->GetMass();
    }

    if( fTotalMass > 0 ) {
        center /= fTotalMass;
    }
    return center;
}

void KinBody::SetLinkTransformations(const std::vector<Transform>& vbodies)
{
    if( RaveGetDebugLevel() & Level_VerifyPlans ) {
        RAVELOG_WARN("SetLinkTransformations should be called with doflastsetvalues, re-setting all values\n");
    }
    else {
        RAVELOG_DEBUG("SetLinkTransformations should be called with doflastsetvalues, re-setting all values\n");
    }
    OPENRAVE_ASSERT_OP_FORMAT(vbodies.size(), >=, _veclinks.size(), "env=%d, not enough links %d<%d", GetEnv()->GetId()%vbodies.size()%_veclinks.size(),ORE_InvalidArguments);
    vector<Transform>::const_iterator it;
    vector<LinkPtr>::iterator itlink;
    for(it = vbodies.begin(), itlink = _veclinks.begin(); it != vbodies.end(); ++it, ++itlink) {
        (*itlink)->SetTransform(*it);
    }
    FOREACH(itjoint,_vecjoints) {
        for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
            (*itjoint)->_doflastsetvalues[i] = (*itjoint)->GetValue(i);
        }
    }
    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

void KinBody::SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<dReal>& doflastsetvalues)
{
    OPENRAVE_ASSERT_OP_FORMAT(transforms.size(), >=, _veclinks.size(), "env=%d, not enough links %d<%d", GetEnv()->GetId()%transforms.size()%_veclinks.size(),ORE_InvalidArguments);
    vector<Transform>::const_iterator it;
    vector<LinkPtr>::iterator itlink;
    for(it = transforms.begin(), itlink = _veclinks.begin(); it != transforms.end(); ++it, ++itlink) {
        (*itlink)->SetTransform(*it);
    }
    FOREACH(itjoint,_vecjoints) {
        for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
            (*itjoint)->_doflastsetvalues[i] = doflastsetvalues.at((*itjoint)->GetDOFIndex()+i);
        }
    }
    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

void KinBody::SetLinkVelocities(const std::vector<std::pair<Vector,Vector> >& velocities)
{
    GetEnv()->GetPhysicsEngine()->SetLinkVelocities(shared_kinbody(),velocities);
    _UpdateGrabbedBodies();
}

void KinBody::SetLinkEnableStates(const std::vector<uint8_t>& enablestates)
{
    OPENRAVE_ASSERT_OP(enablestates.size(),==,_veclinks.size());
    bool bchanged = false;
    for(size_t ilink = 0; ilink < enablestates.size(); ++ilink) {
        bool bEnable = enablestates[ilink]!=0;
        if( _veclinks[ilink]->_info._bIsEnabled != bEnable ) {
            _veclinks[ilink]->_info._bIsEnabled = bEnable;
            _nNonAdjacentLinkCache &= ~AO_Enabled;
            bchanged = true;
        }
    }
    if( bchanged ) {
        _PostprocessChangedParameters(Prop_LinkEnable);
    }
}

void KinBody::SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transBase, uint32_t checklimits)
{
    if( _veclinks.size() == 0 ) {
        return;
    }
    Transform tbase = transBase*_veclinks.at(0)->GetTransform().inverse();
    _veclinks.at(0)->SetTransform(transBase);

    // apply the relative transformation to all links!! (needed for passive joints)
    for(size_t i = 1; i < _veclinks.size(); ++i) {
        _veclinks[i]->SetTransform(tbase*_veclinks[i]->GetTransform());
    }
    SetDOFValues(vJointValues,checklimits);
}

void KinBody::SetDOFValues(const std::vector<dReal>& vJointValues, uint32_t checklimits, const std::vector<int>& dofindices)
{
    CHECK_INTERNAL_COMPUTATION;
    if( vJointValues.size() == 0 || _veclinks.size() == 0) {
        return;
    }
    int expecteddof = dofindices.size() > 0 ? (int)dofindices.size() : GetDOF();
    OPENRAVE_ASSERT_OP_FORMAT((int)vJointValues.size(),>=,expecteddof, "env=%d, not enough values %d<%d", GetEnv()->GetId()%vJointValues.size()%GetDOF(),ORE_InvalidArguments);

    const dReal* pJointValues = &vJointValues[0];
    if( checklimits != CLA_Nothing || dofindices.size() > 0 ) {
        _vTempJoints.resize(GetDOF());
        if( dofindices.size() > 0 ) {
            // user only set a certain number of indices, so have to fill the temporary array with the full set of values first
            // and then overwrite with the user set values
            GetDOFValues(_vTempJoints);
            for(size_t i = 0; i < dofindices.size(); ++i) {
                _vTempJoints.at(dofindices[i]) = pJointValues[i];
            }
            pJointValues = &_vTempJoints[0];
        }
        dReal* ptempjoints = &_vTempJoints[0];

        // check the limits
        vector<dReal> upperlim, lowerlim;
        FOREACHC(it, _vecjoints) {
            const dReal* p = pJointValues+(*it)->GetDOFIndex();
            if( checklimits == CLA_Nothing ) {
                // limits should not be checked, so just copy
                for(int i = 0; i < (*it)->GetDOF(); ++i) {
                    *ptempjoints++ = p[i];
                }
                continue;
            }
            OPENRAVE_ASSERT_OP( (*it)->GetDOF(), <=, 3 );
            (*it)->GetLimits(lowerlim, upperlim);
            if( (*it)->GetType() == JointSpherical ) {
                dReal fcurang = fmod(RaveSqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]),2*PI);
                if( fcurang < lowerlim[0] ) {
                    if( fcurang < 1e-10 ) {
                        *ptempjoints++ = lowerlim[0]; *ptempjoints++ = 0; *ptempjoints++ = 0;
                    }
                    else {
                        dReal fmult = lowerlim[0]/fcurang;
                        *ptempjoints++ = p[0]*fmult; *ptempjoints++ = p[1]*fmult; *ptempjoints++ = p[2]*fmult;
                    }
                }
                else if( fcurang > upperlim[0] ) {
                    if( fcurang < 1e-10 ) {
                        *ptempjoints++ = upperlim[0]; *ptempjoints++ = 0; *ptempjoints++ = 0;
                    }
                    else {
                        dReal fmult = upperlim[0]/fcurang;
                        *ptempjoints++ = p[0]*fmult; *ptempjoints++ = p[1]*fmult; *ptempjoints++ = p[2]*fmult;
                    }
                }
                else {
                    *ptempjoints++ = p[0]; *ptempjoints++ = p[1]; *ptempjoints++ = p[2];
                }
            }
            else {
                for(int i = 0; i < (*it)->GetDOF(); ++i) {
                    if( (*it)->IsCircular(i) ) {
                        // don't normalize since user is expecting the values he sets are exactly returned via GetDOFValues
                        *ptempjoints++ = p[i]; //utils::NormalizeCircularAngle(p[i],(*it)->_vcircularlowerlimit[i],(*it)->_vcircularupperlimit[i]);
                    }
                    else {
                        if( p[i] < lowerlim[i] ) {
                            if( p[i] < lowerlim[i]-g_fEpsilonEvalJointLimit ) {
                                if( checklimits == CLA_CheckLimits ) {
                                    RAVELOG_WARN(str(boost::format("env=%d, dof %d value %e is smaller than the lower limit %e")%GetEnv()->GetId()%((*it)->GetDOFIndex()+i)%p[i]%lowerlim[i]));
                                }
                                else if( checklimits == CLA_CheckLimitsThrow ) {
                                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, dof %d value %e is smaller than the lower limit %e"), GetEnv()->GetId()%((*it)->GetDOFIndex()+i)%p[i]%lowerlim[i], ORE_InvalidArguments);
                                }
                            }
                            *ptempjoints++ = lowerlim[i];
                        }
                        else if( p[i] > upperlim[i] ) {
                            if( p[i] > upperlim[i]+g_fEpsilonEvalJointLimit ) {
                                if( checklimits == CLA_CheckLimits ) {
                                    RAVELOG_WARN(str(boost::format("env=%d, dof %d value %e is greater than the upper limit %e")%GetEnv()->GetId()%((*it)->GetDOFIndex()+i)%p[i]%upperlim[i]));
                                }
                                else if( checklimits == CLA_CheckLimitsThrow ) {
                                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, dof %d value %e is greater than the upper limit %e"), GetEnv()->GetId()%((*it)->GetDOFIndex()+i)%p[i]%upperlim[i], ORE_InvalidArguments);
                                }
                            }
                            *ptempjoints++ = upperlim[i];
                        }
                        else {
                            *ptempjoints++ = p[i];
                        }
                    }
                }
            }
        }
        pJointValues = &_vTempJoints[0];
    }

    // have to compute the angles ahead of time since they are dependent on the link
    const int nActiveJoints = _vecjoints.size();
    const int nPassiveJoints = _vPassiveJoints.size();
    std::vector< boost::array<dReal, 3> > vPassiveJointValues(nPassiveJoints);
    for(int i = 0; i < nPassiveJoints; ++i) {
        const KinBody::JointPtr& pjoint = _vPassiveJoints[i];
        const KinBody::Joint& joint = *pjoint;
        if(joint.IsStatic()) {
            continue;
        }
        const boost::array<dReal, 3>& vlowerlimit = joint._info._vlowerlimit;
        const boost::array<dReal, 3>& vupperlimit = joint._info._vupperlimit;
        boost::array<dReal, 3>& jvals = vPassiveJointValues[i];
        if( !joint.IsMimic() ) {
            joint.GetValues(jvals);
            // check if out of limits!
            for(size_t j = 0; j < 3; ++j) {
                if( !joint.IsCircular(j) ) {
                    if( jvals[j] < vlowerlimit.at(j) ) {
                        if( jvals[j] < vlowerlimit.at(j) - 5e-4f ) {
                            RAVELOG_WARN_FORMAT("env=%d, dummy joint out of lower limit! %e < %e", GetEnv()->GetId() % vlowerlimit.at(j) % jvals[j]);
                        }
                        jvals[j] = vlowerlimit.at(j);
                    }
                    else if( jvals[j] > vupperlimit.at(j) ) {
                        if( jvals[j] > vupperlimit.at(j) + 5e-4f ) {
                            RAVELOG_WARN_FORMAT("env=%d, dummy joint out of upper limit! %e > %e", GetEnv()->GetId() % vupperlimit.at(j) % jvals[j]);
                        }
                        jvals[j] = vupperlimit.at(j);
                    }
                }
            }
        }
    }

    std::vector<uint8_t> vlinkscomputed(_veclinks.size(),0);
    vlinkscomputed[0] = 1;
    boost::array<dReal,3> dummyvalues; // dummy values for a joint
    std::vector<dReal> vtempvalues, veval;

    for(size_t ijoint = 0; ijoint < _vTopologicallySortedJointsAll.size(); ++ijoint) {
        const JointPtr& pjoint = _vTopologicallySortedJointsAll[ijoint];
        KinBody::Joint& joint = *pjoint;
        const LinkPtr& parentlink = joint._attachedbodies[0];
        const LinkPtr& childlink = joint._attachedbodies[1];

        if( joint.IsStatic() ) {
            // if joint.IsStatic(), then joint._info._tRightNoOffset and tjoint are assigned identities
            const Transform t = (!!parentlink ? parentlink->GetTransform() : _veclinks.at(0)->GetTransform()) * joint.GetInternalHierarchyLeftTransform();
            childlink->SetTransform(t);
            vlinkscomputed[childlink->GetIndex()] = 1;
            continue;
        }

        const int jointindex = _vTopologicallySortedJointIndicesAll[ijoint];
        const int dofindex = joint.GetDOFIndex(); // active joint has dofindex>=0; passive has dofindex=-1 but jointindex>=0
        const int jointdof = joint.GetDOF();
        const KinBody::JointType jointtype = joint.GetType();
        const dReal* pvalues = dofindex >= 0 ? pJointValues + dofindex : NULL;
        const boost::array<dReal, 3>& vlowerlimit = joint._info._vlowerlimit;
        const boost::array<dReal, 3>& vupperlimit = joint._info._vupperlimit;

        if( joint.IsMimic() ) {
            for(int i = 0; i < jointdof; ++i) {
                if( joint.IsMimic(i) ) {
                    vtempvalues.clear();
                    const std::vector<Mimic::DOFFormat>& vdofformat = joint._vmimic[i]->_vdofformat;
                    for(const Mimic::DOFFormat& dofformat : vdofformat) {
                        vtempvalues.push_back(dofformat.dofindex >= 0 ? pJointValues[dofformat.dofindex]
                                              : vPassiveJointValues.at(dofformat.jointindex-nActiveJoints).at(dofformat.axis));
                    }
                    const int err = joint._Eval(i, 0, vtempvalues, veval);
                    if( err ) {
                        RAVELOG_WARN(str(boost::format("env=%d, failed to evaluate joint %s, fparser error %d")%GetEnv()->GetId()%joint.GetName()%err));
                    }
                    else {
                        const std::vector<dReal> vevalcopy = veval;
                        for(std::vector<dReal>::iterator iteval = veval.begin(); iteval != veval.end(); ) {
                            if( jointtype == JointSpherical || joint.IsCircular(i) ) {
                            }
                            else if( *iteval < vlowerlimit[i] ) {
                                if(*iteval >= vlowerlimit[i]-g_fEpsilonJointLimit ) {
                                    *iteval = vlowerlimit[i];
                                }
                                else {
                                    iteval = veval.erase(iteval); // invalid value so remove from candidates
                                    continue;
                                }
                            }
                            else if( *iteval > vupperlimit[i] ) {
                                if(*iteval <= vupperlimit[i]+g_fEpsilonJointLimit ) {
                                    *iteval = vupperlimit[i];
                                }
                                else {
                                    iteval = veval.erase(iteval); // invalid value so remove from candidates
                                    continue;
                                }
                            }
                            ++iteval;
                        }

                        if( veval.empty() ) {
                            for(dReal eval : vevalcopy) {
                                if( checklimits == CLA_Nothing || jointtype == JointSpherical || joint.IsCircular(i) ) {
                                    veval.push_back(eval);
                                }
                                else if( eval < vlowerlimit[i]-g_fEpsilonEvalJointLimit ) {
                                    veval.push_back(vlowerlimit[i]);
                                    if( checklimits == CLA_CheckLimits ) {
                                        RAVELOG_WARN(str(boost::format("env=%d, joint %s: lower limit (%e) is not followed: %e")%GetEnv()->GetId()%joint.GetName()%vlowerlimit[i]%eval));
                                    }
                                    else if( checklimits == CLA_CheckLimitsThrow ) {
                                        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, joint %s: lower limit (%e) is not followed: %e"), GetEnv()->GetId()%joint.GetName()%joint._info._vlowerlimit[i]%eval, ORE_InvalidArguments);
                                    }
                                }
                                else if( eval > vupperlimit[i]+g_fEpsilonEvalJointLimit ) {
                                    veval.push_back(vupperlimit[i]);
                                    if( checklimits == CLA_CheckLimits ) {
                                        RAVELOG_WARN(str(boost::format("env=%d, joint %s: upper limit (%e) is not followed: %e")%GetEnv()->GetId()%joint.GetName()%vupperlimit[i]%eval));
                                    }
                                    else if( checklimits == CLA_CheckLimitsThrow ) {
                                        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, joint %s: upper limit (%e) is not followed: %e"), GetEnv()->GetId()%joint.GetName()%vupperlimit[i]%eval, ORE_InvalidArguments);
                                    }
                                }
                                else {
                                    veval.push_back(eval);
                                }
                            }
                            OPENRAVE_ASSERT_FORMAT(!veval.empty(), "env=%d, no valid values for joint %s", GetEnv()->GetId()%joint.GetName(),ORE_Assert);
                        }
                        if( veval.size() > 1 ) {
                            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                            ss << "env=" << GetEnv()->GetId() << ", multiplie values for joint " << joint.GetName() << ": ";
                            for(dReal eval : veval) {
                                ss << eval << " ";
                            }
                            RAVELOG_WARN(ss.str());
                        }
                        dummyvalues[i] = veval.at(0);
                    }

                    // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                    if( dofindex < 0 ) {
                        vPassiveJointValues.at(jointindex-nActiveJoints).at(i) = dummyvalues[i];
                    }
                }
                else if( dofindex >= 0 ) {
                    dummyvalues[i] = pvalues[dofindex+i]; // is this correct? what is a joint has a mimic and non-mimic axis?
                }
                else {
                    // preserve passive joint values
                    dummyvalues[i] = vPassiveJointValues.at(jointindex-nActiveJoints).at(i);
                }
            }
            pvalues = &dummyvalues[0];
        }
        // do the test after mimic computation!
        if( vlinkscomputed[childlink->GetIndex()] ) {
            continue;
        }
        if( !pvalues ) {
            // has to be a passive joint
            pvalues = vPassiveJointValues.at(jointindex-nActiveJoints).data();
        }

        Transform tjoint;
        if( jointtype & JointSpecialBit ) {
            switch(jointtype) {
            case JointHinge2: {
                Transform tfirst;
                tfirst.rot = quatFromAxisAngle(joint.GetInternalHierarchyAxis(0), pvalues[0]);
                Transform tsecond;
                tsecond.rot = quatFromAxisAngle(tfirst.rotate(joint.GetInternalHierarchyAxis(1)), pvalues[1]);
                tjoint = tsecond * tfirst;
                joint._doflastsetvalues[0] = pvalues[0];
                joint._doflastsetvalues[1] = pvalues[1];
                break;
            }
            case JointSpherical: {
                dReal fang = pvalues[0]*pvalues[0]+pvalues[1]*pvalues[1]+pvalues[2]*pvalues[2];
                if( fang > 0 ) {
                    fang = RaveSqrt(fang);
                    dReal fiang = 1/fang;
                    tjoint.rot = quatFromAxisAngle(Vector(pvalues[0]*fiang,pvalues[1]*fiang,pvalues[2]*fiang),fang);
                }
                break;
            }
            case JointTrajectory: {
                vector<dReal> vdata;
                tjoint = Transform();
                dReal fvalue = pvalues[0];
                if( joint.IsCircular(0) ) {
                    // need to normalize the value
                    fvalue = utils::NormalizeCircularAngle(fvalue,joint._vcircularlowerlimit.at(0), joint._vcircularupperlimit.at(0));
                }
                joint._info._trajfollow->Sample(vdata,fvalue);
                if( !joint._info._trajfollow->GetConfigurationSpecification().ExtractTransform(tjoint,vdata.begin(),KinBodyConstPtr()) ) {
                    RAVELOG_WARN(str(boost::format("env=%d, trajectory sampling for joint %s failed")%GetEnv()->GetId()%joint.GetName()));
                }
                joint._doflastsetvalues[0] = 0;
                break;
            }
            default:
                RAVELOG_WARN(str(boost::format("env=%d, forward kinematic type 0x%x not supported")%GetEnv()->GetId()%jointtype));
                break;
            }
        }
        else {
            if( jointtype == JointRevolute ) {
                tjoint.rot = quatFromAxisAngle(joint.GetInternalHierarchyAxis(0), pvalues[0]);
                joint._doflastsetvalues[0] = pvalues[0];
            }
            else if( jointtype == JointPrismatic ) {
                tjoint.trans = joint.GetInternalHierarchyAxis(0) * pvalues[0];
            }
            else {
                for(int iaxis = 0; iaxis < jointdof; ++iaxis) {
                    Transform tdelta;
                    if( joint.IsRevolute(iaxis) ) {
                        tdelta.rot = quatFromAxisAngle(joint.GetInternalHierarchyAxis(iaxis), pvalues[iaxis]);
                        joint._doflastsetvalues[iaxis] = pvalues[iaxis];
                    }
                    else {
                        tdelta.trans = joint.GetInternalHierarchyAxis(iaxis) * pvalues[iaxis];
                    }
                    tjoint = tjoint * tdelta;
                }
            }
        }

        const Transform t = (!!parentlink ? parentlink->GetTransform() : _veclinks.at(0)->GetTransform()) * (joint.GetInternalHierarchyLeftTransform() * tjoint * joint.GetInternalHierarchyRightTransform());
        childlink->SetTransform(t);
        vlinkscomputed[childlink->GetIndex()] = 1;
    }

    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

bool KinBody::IsDOFRevolute(int dofindex) const
{
    int jointindex = _vDOFIndices.at(dofindex);
    return _vecjoints.at(jointindex)->IsRevolute(dofindex-_vecjoints.at(jointindex)->GetDOFIndex());
}

bool KinBody::IsDOFPrismatic(int dofindex) const
{
    int jointindex = _vDOFIndices.at(dofindex);
    return _vecjoints.at(jointindex)->IsPrismatic(dofindex-_vecjoints.at(jointindex)->GetDOFIndex());
}

KinBody::LinkPtr KinBody::GetLink(const std::string& linkname) const
{
    for(std::vector<LinkPtr>::const_iterator it = _veclinks.begin(); it != _veclinks.end(); ++it) {
        if ( (*it)->GetName() == linkname ) {
            return *it;
        }
    }
    return LinkPtr();
}

const std::vector<KinBody::JointPtr>& KinBody::GetDependencyOrderedJoints() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _vTopologicallySortedJoints;
}

const std::vector< std::vector< std::pair<KinBody::LinkPtr, KinBody::JointPtr> > >& KinBody::GetClosedLoops() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _vClosedLoops;
}

bool KinBody::GetChain(int linkindex1, int linkindex2, std::vector<JointPtr>& vjoints) const
{
    CHECK_INTERNAL_COMPUTATION0;
    OPENRAVE_ASSERT_FORMAT(linkindex1>=0 && linkindex1<(int)_veclinks.size(), "body %s linkindex1 %d invalid (num links %d)", GetName()%linkindex1%_veclinks.size(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex2>=0 && linkindex2<(int)_veclinks.size(), "body %s linkindex2 %d invalid (num links %d)", GetName()%linkindex2%_veclinks.size(), ORE_InvalidArguments);
    vjoints.resize(0);
    if( linkindex1 == linkindex2 ) {
        return true;
    }
    int offset = linkindex2*_veclinks.size();
    int curlink = linkindex1;
    while(_vAllPairsShortestPaths[offset+curlink].first>=0) {
        int jointindex = _vAllPairsShortestPaths[offset+curlink].second;
        vjoints.push_back(jointindex < (int)_vecjoints.size() ? _vecjoints.at(jointindex) : _vPassiveJoints.at(jointindex-_vecjoints.size()));
        int prevlink = curlink;
        curlink = _vAllPairsShortestPaths[offset+curlink].first;
        OPENRAVE_ASSERT_OP(prevlink,!=,curlink); // avoid loops
    }
    return vjoints.size()>0; // otherwise disconnected
}

bool KinBody::GetChain(int linkindex1, int linkindex2, std::vector<LinkPtr>& vlinks) const
{
    CHECK_INTERNAL_COMPUTATION0;
    OPENRAVE_ASSERT_FORMAT(linkindex1>=0 && linkindex1<(int)_veclinks.size(), "body %s linkindex1 %d invalid (num links %d)", GetName()%linkindex1%_veclinks.size(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex2>=0 && linkindex2<(int)_veclinks.size(), "body %s linkindex2 %d invalid (num links %d)", GetName()%linkindex2%_veclinks.size(), ORE_InvalidArguments);
    vlinks.resize(0);
    int offset = linkindex2*_veclinks.size();
    int curlink = linkindex1;
    if( _vAllPairsShortestPaths[offset+curlink].first < 0 ) {
        return false;
    }
    vlinks.push_back(_veclinks.at(linkindex1));
    if( linkindex1 == linkindex2 ) {
        return true;
    }
    while(_vAllPairsShortestPaths[offset+curlink].first != linkindex2) {
        curlink = _vAllPairsShortestPaths[offset+curlink].first;
        if( curlink < 0 ) {
            vlinks.resize(0);
            return false;
        }
        vlinks.push_back(_veclinks.at(curlink));
    }
    vlinks.push_back(_veclinks.at(linkindex2));
    return true; // otherwise disconnected
}

bool KinBody::IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const
{
    CHECK_INTERNAL_COMPUTATION0;
    int jointindex = _vDOFIndices.at(dofindex);
    return (DoesAffect(jointindex,linkindex1)==0) != (DoesAffect(jointindex,linkindex2)==0);
}

int KinBody::GetJointIndex(const std::string& jointname) const
{
    int index = 0;
    FOREACHC(it,_vecjoints) {
        if ((*it)->GetName() == jointname ) {
            return index;
        }
        ++index;
    }
    return -1;
}

KinBody::JointPtr KinBody::GetJoint(const std::string& jointname) const
{
    FOREACHC(it,_vecjoints) {
        if ((*it)->GetName() == jointname ) {
            return *it;
        }
    }
    FOREACHC(it,_vPassiveJoints) {
        if ((*it)->GetName() == jointname ) {
            return *it;
        }
    }
    return JointPtr();
}

void KinBody::ComputeJacobianTranslation(int linkindex, const Vector& position, vector<dReal>& vjacobian,const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)_veclinks.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%_veclinks.size(),ORE_InvalidArguments);
    size_t dofstride=0;
    if( dofindices.size() > 0 ) {
        dofstride = dofindices.size();
    }
    else {
        dofstride = GetDOF();
    }
    vjacobian.resize(3*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(),vjacobian.end(),0);

    Vector v;
    int offset = linkindex*_veclinks.size();
    int curlink = 0;
    std::vector<std::pair<int,dReal> > vpartials;
    std::vector<int> vpartialindices;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    while(_vAllPairsShortestPaths[offset+curlink].first>=0) {
        int jointindex = _vAllPairsShortestPaths[offset+curlink].second;
        if( jointindex < (int)_vecjoints.size() ) {
            // active joint
            const JointPtr& pjoint = _vecjoints.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect != 0 ) {
                    if( pjoint->IsRevolute(dof) ) {
                        v = pjoint->GetAxis(dof).cross(position-pjoint->GetAnchor());
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        v = pjoint->GetAxis(dof);
                    }
                    else {
                        RAVELOG_WARN("ComputeJacobianTranslation joint %d not supported\n", pjoint->GetType());
                        continue;
                    }
                    if( dofindices.size() > 0 ) {
                        std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex != dofindices.end() ) {
                            size_t index = itindex-dofindices.begin();
                            vjacobian[index] += v.x; vjacobian[index+dofstride] += v.y; vjacobian[index+2*dofstride] += v.z;
                        }
                    }
                    else {
                        vjacobian[dofindex+dof] += v.x; vjacobian[dofstride+dofindex+dof] += v.y; vjacobian[2*dofstride+dofindex+dof] += v.z;
                    }
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            const JointPtr& pjoint = _vPassiveJoints.at(jointindex-_vecjoints.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    bool bhas = dofindices.size() == 0;
                    if( !bhas ) {
                        FOREACHC(itmimicdof, pjoint->_vmimic[idof]->_vmimicdofs) {
                            if( find(dofindices.begin(),dofindices.end(),itmimicdof->dofindex) != dofindices.end() ) {
                                bhas = true;
                                break;
                            }
                        }
                    }
                    if( bhas ) {
                        Vector vaxis;
                        if( pjoint->IsRevolute(idof) ) {
                            vaxis = pjoint->GetAxis(idof).cross(position-pjoint->GetAnchor());
                        }
                        else if( pjoint->IsPrismatic(idof) ) {
                            vaxis = pjoint->GetAxis(idof);
                        }
                        else {
                            RAVELOG_WARN("ComputeJacobianTranslation joint %d not supported\n", pjoint->GetType());
                            continue;
                        }
                        pjoint->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                        FOREACH(itpartial,vpartials) {
                            Vector v = vaxis * itpartial->second;
                            int index = itpartial->first;
                            if( dofindices.size() > 0 ) {
                                std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                                if( itindex == dofindices.end() ) {
                                    continue;
                                }
                                index = itindex-dofindices.begin();
                            }
                            vjacobian[index] += v.x;
                            vjacobian[dofstride+index] += v.y;
                            vjacobian[2*dofstride+index] += v.z;
                        }
                    }
                }
            }
        }
        curlink = _vAllPairsShortestPaths[offset+curlink].first;
    }
}

void KinBody::CalculateJacobian(int linkindex, const Vector& trans, boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    ComputeJacobianTranslation(linkindex,trans,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetDOF());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetDOF(),itdst->begin());
        itsrc += GetDOF();
    }
}

void KinBody::CalculateRotationJacobian(int linkindex, const Vector& q, std::vector<dReal>& vjacobian) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)_veclinks.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%_veclinks.size(),ORE_InvalidArguments);
    int dofstride = GetDOF();
    vjacobian.resize(4*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(),vjacobian.end(),0);
    Vector v;
    int offset = linkindex*_veclinks.size();
    int curlink = 0;
    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    while(_vAllPairsShortestPaths[offset+curlink].first>=0) {
        int jointindex = _vAllPairsShortestPaths[offset+curlink].second;
        if( jointindex < (int)_vecjoints.size() ) {
            // active joint
            JointPtr pjoint = _vecjoints.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect == 0 ) {
                    RAVELOG_WARN(str(boost::format("link %s should be affected by joint %s")%_veclinks.at(linkindex)->GetName()%pjoint->GetName()));
                }
                else {
                    if( pjoint->IsRevolute(dof) ) {
                        v = pjoint->GetAxis(dof);
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        v = Vector(0,0,0);
                    }
                    else {
                        RAVELOG_WARN("CalculateRotationJacobian joint %d not supported\n", pjoint->GetType());
                        v = Vector(0,0,0);
                    }
                    vjacobian[dofindex+dof] += dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
                    vjacobian[dofstride+dofindex+dof] += dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
                    vjacobian[2*dofstride+dofindex+dof] += dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
                    vjacobian[3*dofstride+dofindex+dof] += dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = _vPassiveJoints.at(jointindex-_vecjoints.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    Vector vaxis;
                    if( pjoint->IsRevolute(idof) ) {
                        vaxis = pjoint->GetAxis(idof);
                    }
                    else if( pjoint->IsPrismatic(idof) ) {
                        vaxis = Vector(0,0,0);
                    }
                    else {
                        RAVELOG_WARN("CalculateRotationJacobian joint %d not supported\n", pjoint->GetType());
                        continue;
                    }
                    pjoint->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                    FOREACH(itpartial,vpartials) {
                        int dofindex = itpartial->first;
                        Vector v = vaxis * itpartial->second;
                        vjacobian[dofindex] += dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
                        vjacobian[dofstride+dofindex] += dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
                        vjacobian[2*dofstride+dofindex] += dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
                        vjacobian[3*dofstride+dofindex] += dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
                    }
                }
            }
        }
        curlink = _vAllPairsShortestPaths[offset+curlink].first;
    }
}

void KinBody::CalculateRotationJacobian(int linkindex, const Vector& q, boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[4][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    CalculateRotationJacobian(linkindex,q,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,4*GetDOF());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetDOF(),itdst->begin());
        itsrc += GetDOF();
    }
}

void KinBody::ComputeJacobianAxisAngle(int linkindex, std::vector<dReal>& vjacobian, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)_veclinks.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%_veclinks.size(),ORE_InvalidArguments);
    size_t dofstride=0;
    if( dofindices.size() > 0 ) {
        dofstride = dofindices.size();
    }
    else {
        dofstride = GetDOF();
    }
    vjacobian.resize(3*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(),vjacobian.end(),0);

    Vector v, anchor, axis;
    int offset = linkindex*_veclinks.size();
    int curlink = 0;
    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    while(_vAllPairsShortestPaths[offset+curlink].first>=0) {
        int jointindex = _vAllPairsShortestPaths[offset+curlink].second;
        if( jointindex < (int)_vecjoints.size() ) {
            // active joint
            JointPtr pjoint = _vecjoints.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect != 0 ) {
                    if( pjoint->IsRevolute(dof) ) {
                        v = pjoint->GetAxis(dof);
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        continue;
                    }
                    else {
                        RAVELOG_WARN("ComputeJacobianAxisAngle joint %d not supported\n", pjoint->GetType());
                        continue;
                    }
                    if( dofindices.size() > 0 ) {
                        std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex != dofindices.end() ) {
                            size_t index = itindex-dofindices.begin();
                            vjacobian[index] += v.x; vjacobian[index+dofstride] += v.y; vjacobian[index+2*dofstride] += v.z;
                        }
                    }
                    else {
                        vjacobian[dofindex+dof] += v.x; vjacobian[dofstride+dofindex+dof] += v.y; vjacobian[2*dofstride+dofindex+dof] += v.z;
                    }
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = _vPassiveJoints.at(jointindex-_vecjoints.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    bool bhas = dofindices.size() == 0;
                    if( !bhas ) {
                        FOREACHC(itmimicdof, pjoint->_vmimic[idof]->_vmimicdofs) {
                            if( find(dofindices.begin(),dofindices.end(),itmimicdof->dofindex) != dofindices.end() ) {
                                bhas = true;
                                break;
                            }
                        }
                    }
                    if( bhas ) {
                        Vector vaxis;
                        if( pjoint->IsRevolute(idof) ) {
                            vaxis = pjoint->GetAxis(idof);
                        }
                        else if( pjoint->IsPrismatic(idof) ) {
                            continue;
                        }
                        else {
                            RAVELOG_WARN("ComputeJacobianAxisAngle joint %d not supported\n", pjoint->GetType());
                            continue;
                        }
                        pjoint->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                        FOREACH(itpartial,vpartials) {
                            Vector v = vaxis * itpartial->second;
                            int index = itpartial->first;
                            if( dofindices.size() > 0 ) {
                                std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                                if( itindex == dofindices.end() ) {
                                    continue;
                                }
                                index = itindex-dofindices.begin();
                            }
                            vjacobian[index] += v.x;
                            vjacobian[dofstride+index] += v.y;
                            vjacobian[2*dofstride+index] += v.z;
                        }
                    }
                }
            }
        }
        curlink = _vAllPairsShortestPaths[offset+curlink].first;
    }
}

void KinBody::CalculateAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& mjacobian) const
{
    mjacobian.resize(boost::extents[3][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    ComputeJacobianAxisAngle(linkindex,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetDOF());
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetDOF(),itdst->begin());
        itsrc += GetDOF();
    }
}

void KinBody::ComputeHessianTranslation(int linkindex, const Vector& position, std::vector<dReal>& hessian, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)_veclinks.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%_veclinks.size(),ORE_InvalidArguments);
    size_t dofstride=0;
    if( dofindices.size() > 0 ) {
        dofstride = dofindices.size();
    }
    else {
        dofstride = GetDOF();
    }
    hessian.resize(dofstride*3*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(hessian.begin(),hessian.end(),0);

    int offset = linkindex*_veclinks.size();
    int curlink = 0;
    std::vector<Vector> vaxes, vjacobian; vaxes.reserve(dofstride); vjacobian.reserve(dofstride);
    std::vector<int> vpartialindices;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    std::vector<int> vinsertedindices; vinsertedindices.reserve(dofstride);
    typedef std::pair< std::vector<Vector>, std::vector<std::pair<int,dReal> > > PartialInfo;
    std::map<size_t, PartialInfo > mappartialsinserted; // if vinsertedindices has -1, that index will be here
    while(_vAllPairsShortestPaths[offset+curlink].first>=0) {
        int jointindex = _vAllPairsShortestPaths[offset+curlink].second;
        if( jointindex < (int)_vecjoints.size() ) {
            // active joint
            JointPtr pjoint = _vecjoints.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect == 0 ) {
                    RAVELOG_WARN(str(boost::format("link %s should be affected by joint %s")%_veclinks.at(linkindex)->GetName()%pjoint->GetName()));
                }
                else {
                    size_t index = dofindex+dof;
                    if( dofindices.size() > 0 ) {
                        std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex != dofindices.end() ) {
                            index = itindex-dofindices.begin();
                        }
                        else {
                            continue;
                        }
                    }

                    if( pjoint->IsRevolute(dof) ) {
                        vaxes.push_back(pjoint->GetAxis(dof));
                        vjacobian.push_back(pjoint->GetAxis(dof).cross(position-pjoint->GetAnchor()));
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        vaxes.push_back(Vector());
                        vjacobian.push_back(pjoint->GetAxis(dof));
                    }
                    else {
                        vaxes.push_back(Vector());
                        vjacobian.push_back(Vector());
                        RAVELOG_WARN("ComputeHessianTranslation joint %d not supported\n", pjoint->GetType());
                    }
                    vinsertedindices.push_back(index);
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = _vPassiveJoints.at(jointindex-_vecjoints.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    bool bhas = dofindices.size() == 0;
                    if( !bhas ) {
                        FOREACHC(itmimicdof, pjoint->_vmimic[idof]->_vmimicdofs) {
                            if( find(dofindices.begin(),dofindices.end(),itmimicdof->dofindex) != dofindices.end() ) {
                                bhas = true;
                                break;
                            }
                        }
                    }
                    if( bhas ) {
                        Vector vaxis;
                        if( pjoint->IsRevolute(idof) ) {
                            vaxes.push_back(pjoint->GetAxis(idof));
                            vjacobian.push_back(pjoint->GetAxis(idof).cross(position-pjoint->GetAnchor()));
                        }
                        else if( pjoint->IsPrismatic(idof) ) {
                            vjacobian.push_back(pjoint->GetAxis(idof));
                            vaxes.push_back(Vector());
                        }
                        else {
                            vaxes.push_back(Vector());
                            vjacobian.push_back(Vector());
                            RAVELOG_WARN("ComputeHessianTranslation joint %d not supported\n", pjoint->GetType());
                        }
                        PartialInfo& partialinfo = mappartialsinserted[vinsertedindices.size()];
                        partialinfo.first.resize(vinsertedindices.size());
                        pjoint->_ComputePartialVelocities(partialinfo.second,idof,mapcachedpartials);
                        vinsertedindices.push_back(-1);
                    }
                }
            }
        }
        curlink = _vAllPairsShortestPaths[offset+curlink].first;
    }

    for(size_t i = 0; i < vaxes.size(); ++i) {
        if( vinsertedindices[i] < 0 ) {
            PartialInfo& partialinfo = mappartialsinserted[i];
            FOREACH(itpartial,partialinfo.second) {
                int index = itpartial->first;
                if( dofindices.size() > 0 ) {
                    std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                    if( itindex == dofindices.end() ) {
                        continue;
                    }
                    index = itindex-dofindices.begin();
                }

                for(size_t j = 0; j < i; ++j) {
                    Vector v = partialinfo.first.at(j)*itpartial->second;
                    if( vinsertedindices[j] < 0 ) {
                        //RAVELOG_WARN("hessian unhandled condition with mimic\n");
                        PartialInfo& partialinfo2 = mappartialsinserted[j];
                        FOREACH(itpartial2,partialinfo2.second) {
                            int index2 = itpartial2->first;
                            if( dofindices.size() > 0 ) {
                                std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                                if( itindex == dofindices.end() ) {
                                    continue;
                                }
                                index2 = itindex-dofindices.begin();
                            }

                            Vector v2 = v*itpartial2->second;
                            size_t indexoffset = 3*dofstride*index2+index;
                            hessian[indexoffset+0] += v2.x;
                            hessian[indexoffset+dofstride] += v2.y;
                            hessian[indexoffset+2*dofstride] += v2.z;
                            if( j != i ) {
                                // symmetric
                                indexoffset = 3*dofstride*index+index2;
                                hessian[indexoffset+0] += v2.x;
                                hessian[indexoffset+dofstride] += v2.y;
                                hessian[indexoffset+2*dofstride] += v2.z;
                            }
                        }
                    }
                    else {
                        size_t indexoffset = 3*dofstride*index+vinsertedindices[j];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                        if( j != i ) {
                            // symmetric
                            indexoffset = 3*dofstride*vinsertedindices[j]+index;
                            hessian[indexoffset+0] += v.x;
                            hessian[indexoffset+dofstride] += v.y;
                            hessian[indexoffset+2*dofstride] += v.z;
                        }
                    }
                }

                for(size_t j = i; j < vaxes.size(); ++j) {
                    Vector v = vaxes[i].cross(vjacobian[j]);
                    if( j == i ) {
                        dReal f = itpartial->second*itpartial->second;
                        size_t indexoffset = 3*dofstride*index+index;
                        hessian[indexoffset+0] += v.x*f;
                        hessian[indexoffset+dofstride] += v.y*f;
                        hessian[indexoffset+2*dofstride] += v.z*f;
                        continue;
                    }

                    if( vinsertedindices[j] < 0 ) {
                        // only add the first time, do not multiply by itpartial->second yet?
                        if( itpartial == partialinfo.second.begin() ) {
                            mappartialsinserted[j].first.at(i) += v; // will get to it later
                        }
                    }
                    else {
                        v *= itpartial->second;
                        size_t indexoffset = 3*dofstride*index+vinsertedindices[j];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                        if( j != i ) {
                            // symmetric
                            indexoffset = 3*dofstride*vinsertedindices[j]+index;
                            hessian[indexoffset+0] += v.x;
                            hessian[indexoffset+dofstride] += v.y;
                            hessian[indexoffset+2*dofstride] += v.z;
                        }
                    }
                }
            }
        }
        else {
            size_t ioffset = 3*dofstride*vinsertedindices[i];
            for(size_t j = i; j < vaxes.size(); ++j) {
                Vector v = vaxes[i].cross(vjacobian[j]);
                if( vinsertedindices[j] < 0 ) {
                    mappartialsinserted[j].first.at(i) = v; // we'll get to it later
                }
                else {
                    size_t indexoffset = ioffset+vinsertedindices[j];
                    hessian[indexoffset+0] += v.x;
                    hessian[indexoffset+dofstride] += v.y;
                    hessian[indexoffset+2*dofstride] += v.z;
                    if( j != i ) {
                        // symmetric
                        indexoffset = 3*dofstride*vinsertedindices[j]+vinsertedindices[i];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                    }
                }
            }
        }
    }
}

void KinBody::ComputeHessianAxisAngle(int linkindex, std::vector<dReal>& hessian, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)_veclinks.size(), "body %s bad link index %d (num links %d)", GetName()%linkindex%_veclinks.size(),ORE_InvalidArguments);
    size_t dofstride=0;
    if( dofindices.size() > 0 ) {
        dofstride = dofindices.size();
    }
    else {
        dofstride = GetDOF();
    }
    hessian.resize(dofstride*3*dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(hessian.begin(),hessian.end(),0);

    int offset = linkindex*_veclinks.size();
    int curlink = 0;
    std::vector<Vector> vaxes; vaxes.reserve(dofstride);
    std::vector<int> vpartialindices;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;
    std::vector<int> vinsertedindices; vinsertedindices.reserve(dofstride);
    typedef std::pair< std::vector<Vector>, std::vector<std::pair<int,dReal> > > PartialInfo;
    std::map<size_t, PartialInfo > mappartialsinserted; // if vinsertedindices has -1, that index will be here
    while(_vAllPairsShortestPaths[offset+curlink].first>=0) {
        int jointindex = _vAllPairsShortestPaths[offset+curlink].second;
        if( jointindex < (int)_vecjoints.size() ) {
            // active joint
            JointPtr pjoint = _vecjoints.at(jointindex);
            int dofindex = pjoint->GetDOFIndex();
            int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int dof = 0; dof < pjoint->GetDOF(); ++dof) {
                if( affect == 0 ) {
                    RAVELOG_WARN(str(boost::format("link %s should be affected by joint %s")%_veclinks.at(linkindex)->GetName()%pjoint->GetName()));
                }
                else {
                    size_t index = dofindex+dof;
                    if( dofindices.size() > 0 ) {
                        std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex != dofindices.end() ) {
                            index = itindex-dofindices.begin();
                        }
                        else {
                            continue;
                        }
                    }

                    if( pjoint->IsRevolute(dof) ) {
                        vaxes.push_back(pjoint->GetAxis(dof));
                    }
                    else if( pjoint->IsPrismatic(dof) ) {
                        vaxes.push_back(Vector());
                    }
                    else {
                        vaxes.push_back(Vector());
                        RAVELOG_WARN("ComputeHessianTranslation joint %d not supported\n", pjoint->GetType());
                    }
                    vinsertedindices.push_back(index);
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            JointPtr pjoint = _vPassiveJoints.at(jointindex-_vecjoints.size());
            for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    bool bhas = dofindices.size() == 0;
                    if( !bhas ) {
                        FOREACHC(itmimicdof, pjoint->_vmimic[idof]->_vmimicdofs) {
                            if( find(dofindices.begin(),dofindices.end(),itmimicdof->dofindex) != dofindices.end() ) {
                                bhas = true;
                                break;
                            }
                        }
                    }
                    if( bhas ) {
                        Vector vaxis;
                        if( pjoint->IsRevolute(idof) ) {
                            vaxes.push_back(pjoint->GetAxis(idof));
                        }
                        else if( pjoint->IsPrismatic(idof) ) {
                            vaxes.push_back(Vector());
                        }
                        else {
                            vaxes.push_back(Vector());
                            RAVELOG_WARN("ComputeHessianTranslation joint %d not supported\n", pjoint->GetType());
                        }
                        PartialInfo& partialinfo = mappartialsinserted[vinsertedindices.size()];
                        partialinfo.first.resize(vinsertedindices.size());
                        pjoint->_ComputePartialVelocities(partialinfo.second,idof,mapcachedpartials);
                        vinsertedindices.push_back(-1);
                    }
                }
            }
        }
        curlink = _vAllPairsShortestPaths[offset+curlink].first;
    }

    for(size_t i = 0; i < vaxes.size(); ++i) {
        if( vinsertedindices[i] < 0 ) {
            PartialInfo& partialinfo = mappartialsinserted[i];
            FOREACH(itpartial,partialinfo.second) {
                int index = itpartial->first;
                if( dofindices.size() > 0 ) {
                    std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                    if( itindex == dofindices.end() ) {
                        continue;
                    }
                    index = itindex-dofindices.begin();
                }

                for(size_t j = 0; j < i; ++j) {
                    Vector v = partialinfo.first.at(j)*itpartial->second;
                    if( vinsertedindices[j] < 0 ) {
                        //RAVELOG_WARN("hessian unhandled condition with mimic\n");
                        PartialInfo& partialinfo2 = mappartialsinserted[j];
                        FOREACH(itpartial2,partialinfo2.second) {
                            int index2 = itpartial2->first;
                            if( dofindices.size() > 0 ) {
                                std::vector<int>::const_iterator itindex = find(dofindices.begin(),dofindices.end(),itpartial->first);
                                if( itindex == dofindices.end() ) {
                                    continue;
                                }
                                index2 = itindex-dofindices.begin();
                            }

                            Vector v2 = v*itpartial2->second;
                            size_t indexoffset = 3*dofstride*index2+index;
                            hessian[indexoffset+0] += v2.x;
                            hessian[indexoffset+dofstride] += v2.y;
                            hessian[indexoffset+2*dofstride] += v2.z;
                            if( j != i ) {
                                // symmetric
                                indexoffset = 3*dofstride*index+index2;
                                hessian[indexoffset+0] += v2.x;
                                hessian[indexoffset+dofstride] += v2.y;
                                hessian[indexoffset+2*dofstride] += v2.z;
                            }
                        }
                    }
                    else {
                        size_t indexoffset = 3*dofstride*index+vinsertedindices[j];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                        if( j != i ) {
                            // symmetric
                            indexoffset = 3*dofstride*vinsertedindices[j]+index;
                            hessian[indexoffset+0] += v.x;
                            hessian[indexoffset+dofstride] += v.y;
                            hessian[indexoffset+2*dofstride] += v.z;
                        }
                    }
                }

                for(size_t j = i+1; j < vaxes.size(); ++j) {
                    Vector v = vaxes[i].cross(vaxes[j]);
                    if( j == i ) {
                        dReal f = itpartial->second*itpartial->second;
                        size_t indexoffset = 3*dofstride*index+index;
                        hessian[indexoffset+0] += v.x*f;
                        hessian[indexoffset+dofstride] += v.y*f;
                        hessian[indexoffset+2*dofstride] += v.z*f;
                        continue;
                    }

                    if( vinsertedindices[j] < 0 ) {
                        // only add the first time, do not multiply by itpartial->second yet?
                        if( itpartial == partialinfo.second.begin() ) {
                            mappartialsinserted[j].first.at(i) += v; // will get to it later
                        }
                    }
                    else {
                        v *= itpartial->second;
                        size_t indexoffset = 3*dofstride*index+vinsertedindices[j];
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                        // symmetric
                        indexoffset = 3*dofstride*vinsertedindices[j]+index;
                        hessian[indexoffset+0] += v.x;
                        hessian[indexoffset+dofstride] += v.y;
                        hessian[indexoffset+2*dofstride] += v.z;
                    }
                }
            }
        }
        else {
            size_t ioffset = 3*dofstride*vinsertedindices[i];
            for(size_t j = i+1; j < vaxes.size(); ++j) {
                Vector v = vaxes[i].cross(vaxes[j]);
                if( vinsertedindices[j] < 0 ) {
                    mappartialsinserted[j].first.at(i) = v; // we'll get to it later
                }
                else {
                    size_t indexoffset = ioffset+vinsertedindices[j];
                    hessian[indexoffset+0] += v.x;
                    hessian[indexoffset+dofstride] += v.y;
                    hessian[indexoffset+2*dofstride] += v.z;
                    // symmetric
                    indexoffset = 3*dofstride*vinsertedindices[j]+vinsertedindices[i];
                    hessian[indexoffset+0] += v.x;
                    hessian[indexoffset+dofstride] += v.y;
                    hessian[indexoffset+2*dofstride] += v.z;
                }
            }
        }
    }
}

void KinBody::ComputeInverseDynamics(std::vector<dReal>& doftorques, const std::vector<dReal>& vDOFAccelerations, const KinBody::ForceTorqueMap& mapExternalForceTorque) const
{
    CHECK_INTERNAL_COMPUTATION;
    doftorques.resize(GetDOF());
    if( _vecjoints.size() == 0 ) {
        return;
    }

    Vector vgravity = GetEnv()->GetPhysicsEngine()->GetGravity();
    std::vector<dReal> vDOFVelocities;
    std::vector<pair<Vector, Vector> > vLinkVelocities, vLinkAccelerations; // linear, angular
    _ComputeDOFLinkVelocities(vDOFVelocities, vLinkVelocities);
    // check if all velocities are 0, if yes, then can simplify some computations since only have contributions from dofacell and external forces
    bool bHasVelocity = false;
    FOREACH(it,vDOFVelocities) {
        if( RaveFabs(*it) > g_fEpsilonLinear ) {
            bHasVelocity = true;
            break;
        }
    }
    if( !bHasVelocity ) {
        vDOFVelocities.resize(0);
    }
    AccelerationMap externalaccelerations;
    externalaccelerations[0] = make_pair(-vgravity, Vector());
    AccelerationMapPtr pexternalaccelerations(&externalaccelerations, utils::null_deleter());
    _ComputeLinkAccelerations(vDOFVelocities, vDOFAccelerations, vLinkVelocities, vLinkAccelerations, pexternalaccelerations);

    // all valuess are in the global coordinate system
    // Given the velocity/acceleration of the object is on point A, to change to B do:
    // v_B = v_A + angularvel x (B-A)
    // a_B = a_A + angularaccel x (B-A) + angularvel x (angularvel x (B-A))
    // forward recursion
    std::vector<Vector> vLinkCOMLinearAccelerations(_veclinks.size()), vLinkCOMMomentOfInertia(_veclinks.size());
    for(size_t i = 0; i < vLinkVelocities.size(); ++i) {
        Vector vglobalcomfromlink = _veclinks.at(i)->GetGlobalCOM() - _veclinks.at(i)->_info._t.trans;
        Vector vangularaccel = vLinkAccelerations.at(i).second;
        Vector vangularvelocity = vLinkVelocities.at(i).second;
        vLinkCOMLinearAccelerations[i] = vLinkAccelerations.at(i).first + vangularaccel.cross(vglobalcomfromlink) + vangularvelocity.cross(vangularvelocity.cross(vglobalcomfromlink));
        TransformMatrix tm = _veclinks.at(i)->GetGlobalInertia();
        vLinkCOMMomentOfInertia[i] = tm.rotate(vangularaccel) + vangularvelocity.cross(tm.rotate(vangularvelocity));
    }

    // backward recursion
    std::vector< std::pair<Vector, Vector> > vLinkForceTorques(_veclinks.size());
    FOREACHC(it,mapExternalForceTorque) {
        vLinkForceTorques.at(it->first) = it->second;
    }
    std::fill(doftorques.begin(),doftorques.end(),0);

    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;

    // go backwards
    for(size_t ijoint = 0; ijoint < _vTopologicallySortedJointsAll.size(); ++ijoint) {
        JointPtr pjoint = _vTopologicallySortedJointsAll.at(_vTopologicallySortedJointsAll.size()-1-ijoint);
        int childindex = pjoint->GetHierarchyChildLink()->GetIndex();
        Vector vcomforce = vLinkCOMLinearAccelerations[childindex]*pjoint->GetHierarchyChildLink()->GetMass() + vLinkForceTorques.at(childindex).first;
        Vector vjointtorque = vLinkForceTorques.at(childindex).second + vLinkCOMMomentOfInertia.at(childindex);

        if( !!pjoint->GetHierarchyParentLink() ) {
            Vector vchildcomtoparentcom = pjoint->GetHierarchyChildLink()->GetGlobalCOM() - pjoint->GetHierarchyParentLink()->GetGlobalCOM();
            int parentindex = pjoint->GetHierarchyParentLink()->GetIndex();
            vLinkForceTorques.at(parentindex).first += vcomforce;
            vLinkForceTorques.at(parentindex).second += vjointtorque + vchildcomtoparentcom.cross(vcomforce);
        }

        Vector vcomtoanchor = pjoint->GetHierarchyChildLink()->GetGlobalCOM() - pjoint->GetAnchor();
        if( pjoint->GetDOFIndex() >= 0 ) {
            if( pjoint->GetType() == JointHinge ) {
                doftorques.at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vjointtorque + vcomtoanchor.cross(vcomforce));
            }
            else if( pjoint->GetType() == JointSlider ) {
                doftorques.at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vcomforce)/(2*PI);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("joint 0x%x not supported"), pjoint->GetType(), ORE_Assert);
            }

            dReal fFriction = 0; // torque due to friction
            dReal fRotorAccelerationTorque = 0; // torque due to accelerating motor rotor (and gear)
            // see if any friction needs to be added. Only add if the velocity is non-zero since with zero velocity do not know the exact torque on the joint...
            if( !!pjoint->_info._infoElectricMotor ) {
                const ElectricMotorActuatorInfoPtr pActuatorInfo = pjoint->_info._infoElectricMotor;
                if( pjoint->GetDOFIndex() < (int)vDOFVelocities.size() ) {
                    if( vDOFVelocities.at(pjoint->GetDOFIndex()) > g_fEpsilonLinear ) {
                        fFriction += pActuatorInfo->coloumb_friction;
                    }
                    else if( vDOFVelocities.at(pjoint->GetDOFIndex()) < -g_fEpsilonLinear ) {
                        fFriction -= pActuatorInfo->coloumb_friction;
                    }
                    fFriction += vDOFVelocities.at(pjoint->GetDOFIndex())*pActuatorInfo->viscous_friction;

                    if (pActuatorInfo->rotor_inertia > 0.0) {
                        // converting inertia on motor side to load side requires multiplying by gear ratio squared because inertia unit is mass * distance^2
                        const dReal fInertiaOnLoadSide = pActuatorInfo->rotor_inertia * pActuatorInfo->gear_ratio * pActuatorInfo->gear_ratio;
                        fRotorAccelerationTorque += vDOFAccelerations.at(pjoint->GetDOFIndex()) * fInertiaOnLoadSide;
                    }
                }

                doftorques.at(pjoint->GetDOFIndex()) += fFriction + fRotorAccelerationTorque;
            }
        }
        else if( pjoint->IsMimic(0) ) {
            // passive joint, so have to transfer the torque to its dependent joints.
            // TODO if there's more than one dependent joint, how do we split?
            dReal faxistorque;
            if( pjoint->GetType() == JointHinge ) {
                faxistorque = pjoint->GetAxis(0).dot3(vjointtorque + vcomtoanchor.cross(vcomforce));
            }
            else if( pjoint->GetType() == JointSlider ) {
                faxistorque = pjoint->GetAxis(0).dot3(vcomforce)/(2*PI);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("joint 0x%x not supported"), pjoint->GetType(), ORE_Assert);
            }

            if( !!pjoint->_info._infoElectricMotor ) {
                // TODO how to process this correctly? what is velocity of this joint? pjoint->GetVelocity(0)?
            }

            pjoint->_ComputePartialVelocities(vpartials,0,mapcachedpartials);
            FOREACH(itpartial,vpartials) {
                int dofindex = itpartial->first;
                doftorques.at(dofindex) += itpartial->second*faxistorque;
            }
        }
        else {
            // joint should be static
            OPENRAVE_ASSERT_FORMAT(pjoint->IsStatic(), "joint %s (%d) is expected to be static", pjoint->GetName()%ijoint, ORE_Assert);
        }
    }
}

void KinBody::ComputeInverseDynamics(boost::array< std::vector<dReal>, 3>& vDOFTorqueComponents, const std::vector<dReal>& vDOFAccelerations, const KinBody::ForceTorqueMap& mapExternalForceTorque) const
{
    CHECK_INTERNAL_COMPUTATION;
    FOREACH(itdoftorques,vDOFTorqueComponents) {
        itdoftorques->resize(GetDOF());
    }
    if( _vecjoints.size() == 0 ) {
        return;
    }

    Vector vgravity = GetEnv()->GetPhysicsEngine()->GetGravity();
    std::vector<dReal> vDOFVelocities;
    boost::array< std::vector<pair<Vector, Vector> >, 3> vLinkVelocities; // [0] = all zeros, [1] = dof velocities only, [2] = only velocities due to base link
    boost::array< std::vector<pair<Vector, Vector> >, 3> vLinkAccelerations; // [0] = dofaccel only, [1] = dofvel only, [2] - gravity + external only (dofaccel=0, dofvel=0)
    boost::array<int,3> linkaccelsimilar = {{-1,-1,-1}}; // used for tracking which vLinkAccelerations indices are similar to each other (to avoid computation)

    vLinkVelocities[0].resize(_veclinks.size());
    _ComputeDOFLinkVelocities(vDOFVelocities, vLinkVelocities[1], false);
    // check if all velocities are 0, if yes, then can simplify some computations since only have contributions from dofacell and external forces
    bool bHasVelocity = false;
    FOREACH(it,vDOFVelocities) {
        if( RaveFabs(*it) > g_fEpsilonLinear ) {
            bHasVelocity = true;
            break;
        }
    }
    if( !bHasVelocity ) {
        vDOFVelocities.resize(0);
    }

    AccelerationMap externalaccelerations;
    externalaccelerations[0] = make_pair(-vgravity, Vector());
    AccelerationMapPtr pexternalaccelerations(&externalaccelerations, utils::null_deleter());

    // all valuess are in the global coordinate system
    // try to compute as little as possible by checking what is non-zero
    Vector vbaselinear, vbaseangular;
    _veclinks.at(0)->GetVelocity(vbaselinear,vbaseangular);
    bool bHasGravity = vgravity.lengthsqr3() > g_fEpsilonLinear*g_fEpsilonLinear;
    bool bHasBaseLinkAccel = vbaseangular.lengthsqr3() > g_fEpsilonLinear*g_fEpsilonLinear;
    if( bHasBaseLinkAccel || bHasGravity ) {
        if( bHasBaseLinkAccel ) {
            // remove the base link velocity frame
            // v_B = v_A + angularvel x (B-A)
            vLinkVelocities[2].resize(_veclinks.size());
            Vector vbasepos = _veclinks.at(0)->_info._t.trans;
            for(size_t i = 1; i < vLinkVelocities[0].size(); ++i) {
                Vector voffset = _veclinks.at(i)->_info._t.trans - vbasepos;
                vLinkVelocities[2][i].first = vbaselinear + vbaseangular.cross(voffset);
                vLinkVelocities[2][i].second = vbaseangular;
            }
        }
        else {
            vLinkVelocities[2] = vLinkVelocities[0];
        }
        _ComputeLinkAccelerations(std::vector<dReal>(), std::vector<dReal>(), vLinkVelocities[2], vLinkAccelerations[2], pexternalaccelerations);
        if( bHasVelocity ) {
            _ComputeLinkAccelerations(vDOFVelocities, std::vector<dReal>(), vLinkVelocities[1], vLinkAccelerations[1]);
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0]);
            }
            else {
                linkaccelsimilar[0] = 1;
            }
        }
        else {
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0]);
            }
        }
    }
    else {
        // no external forces
        vLinkVelocities[2] = vLinkVelocities[0];
        if( bHasVelocity ) {
            _ComputeLinkAccelerations(vDOFVelocities, std::vector<dReal>(), vLinkVelocities[1], vLinkAccelerations[1]);
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0]);
            }
            else {
                linkaccelsimilar[0] = 1;
            }
        }
        else {
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0]);
            }
        }
    }

    boost::array< std::vector<Vector>, 3> vLinkCOMLinearAccelerations, vLinkCOMMomentOfInertia;
    boost::array< std::vector< std::pair<Vector, Vector> >, 3> vLinkForceTorques;
    for(size_t j = 0; j < 3; ++j) {
        if( vLinkAccelerations[j].size() > 0 ) {
            vLinkCOMLinearAccelerations[j].resize(_veclinks.size());
            vLinkCOMMomentOfInertia[j].resize(_veclinks.size());
            vLinkForceTorques[j].resize(_veclinks.size());
        }
    }

    for(size_t i = 0; i < _veclinks.size(); ++i) {
        Vector vglobalcomfromlink = _veclinks.at(i)->GetGlobalCOM() - _veclinks.at(i)->_info._t.trans;
        TransformMatrix tm = _veclinks.at(i)->GetGlobalInertia();
        for(size_t j = 0; j < 3; ++j) {
            if( vLinkAccelerations[j].size() > 0 ) {
                Vector vangularaccel = vLinkAccelerations[j].at(i).second;
                Vector vangularvelocity = vLinkVelocities[j].at(i).second;
                vLinkCOMLinearAccelerations[j][i] = vLinkAccelerations[j].at(i).first + vangularaccel.cross(vglobalcomfromlink) + vangularvelocity.cross(vangularvelocity.cross(vglobalcomfromlink));
                vLinkCOMMomentOfInertia[j][i] = tm.rotate(vangularaccel) + vangularvelocity.cross(tm.rotate(vangularvelocity));
            }
        }
    }

    FOREACH(itdoftorques,vDOFTorqueComponents) {
        std::fill(itdoftorques->begin(),itdoftorques->end(),0);
    }

    // backward recursion
    vLinkForceTorques[2].resize(_veclinks.size());
    FOREACHC(it,mapExternalForceTorque) {
        vLinkForceTorques[2].at(it->first) = it->second;
    }

    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mapcachedpartials;

    // go backwards
    for(size_t ijoint = 0; ijoint < _vTopologicallySortedJointsAll.size(); ++ijoint) {
        JointPtr pjoint = _vTopologicallySortedJointsAll.at(_vTopologicallySortedJointsAll.size()-1-ijoint);
        int childindex = pjoint->GetHierarchyChildLink()->GetIndex();
        Vector vchildcomtoparentcom;
        int parentindex = -1;
        if( !!pjoint->GetHierarchyParentLink() ) {
            vchildcomtoparentcom = pjoint->GetHierarchyChildLink()->GetGlobalCOM() - pjoint->GetHierarchyParentLink()->GetGlobalCOM();
            parentindex = pjoint->GetHierarchyParentLink()->GetIndex();
        }

        bool bIsMimic = pjoint->GetDOFIndex() < 0 && pjoint->IsMimic(0);
        if( bIsMimic ) {
            pjoint->_ComputePartialVelocities(vpartials,0,mapcachedpartials);
        }

        dReal mass = pjoint->GetHierarchyChildLink()->GetMass();
        Vector vcomtoanchor = pjoint->GetHierarchyChildLink()->GetGlobalCOM() - pjoint->GetAnchor();
        for(size_t j = 0; j < 3; ++j) {
            if( vLinkForceTorques[j].size() == 0 ) {
                continue;
            }
            Vector vcomforce = vLinkForceTorques[j].at(childindex).first;
            Vector vjointtorque = vLinkForceTorques[j].at(childindex).second;
            if( vLinkCOMLinearAccelerations[j].size() > 0 ) {
                vcomforce += vLinkCOMLinearAccelerations[j][childindex]*mass;
                vjointtorque += vLinkCOMMomentOfInertia[j].at(childindex);
            }

            if( parentindex >= 0 ) {
                vLinkForceTorques[j].at(parentindex).first += vcomforce;
                vLinkForceTorques[j].at(parentindex).second += vjointtorque + vchildcomtoparentcom.cross(vcomforce);
            }

            if( pjoint->GetDOFIndex() >= 0 ) {
                if( pjoint->GetType() == JointHinge ) {
                    vDOFTorqueComponents[j].at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vjointtorque + vcomtoanchor.cross(vcomforce));
                }
                else if( pjoint->GetType() == JointSlider ) {
                    vDOFTorqueComponents[j].at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vcomforce)/(2*PI);
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("joint 0x%x not supported"), pjoint->GetType(), ORE_Assert);
                }
            }
            else if( bIsMimic ) {
                // passive joint, so have to transfer the torque to its dependent joints.
                // TODO if there's more than one dependent joint, how do we split?
                dReal faxistorque;
                if( pjoint->GetType() == JointHinge ) {
                    faxistorque = pjoint->GetAxis(0).dot3(vjointtorque + vcomtoanchor.cross(vcomforce));
                }
                else if( pjoint->GetType() == JointSlider ) {
                    faxistorque = pjoint->GetAxis(0).dot3(vcomforce)/(2*PI);
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("joint 0x%x not supported"), pjoint->GetType(), ORE_Assert);
                }

                FOREACH(itpartial,vpartials) {
                    int dofindex = itpartial->first;
                    vDOFTorqueComponents[j].at(dofindex) += itpartial->second*faxistorque;
                }
            }
            else {
                // joint should be static
                BOOST_ASSERT(pjoint->IsStatic());
            }
        }
    }
}

void KinBody::GetLinkAccelerations(const std::vector<dReal>&vDOFAccelerations, std::vector<std::pair<Vector,Vector> >&vLinkAccelerations, AccelerationMapConstPtr externalaccelerations) const
{
    CHECK_INTERNAL_COMPUTATION;
    if( _veclinks.size() == 0 ) {
        vLinkAccelerations.resize(0);
    }
    else {
        std::vector<dReal> vDOFVelocities;
        std::vector<pair<Vector, Vector> > vLinkVelocities;
        _ComputeDOFLinkVelocities(vDOFVelocities,vLinkVelocities);
        _ComputeLinkAccelerations(vDOFVelocities, vDOFAccelerations, vLinkVelocities, vLinkAccelerations, externalaccelerations);
    }
}

void KinBody::_ComputeDOFLinkVelocities(std::vector<dReal>& dofvelocities, std::vector<std::pair<Vector,Vector> >& vLinkVelocities, bool usebaselinkvelocity) const
{
    GetEnv()->GetPhysicsEngine()->GetLinkVelocities(shared_kinbody_const(),vLinkVelocities);
    if( _veclinks.size() <= 1 ) {
        dofvelocities.resize(GetDOF());
        if( !usebaselinkvelocity && _veclinks.size() > 0 ) {
            vLinkVelocities[0].first = Vector();
            vLinkVelocities[0].second = Vector();
        }
        return;
    }
    if( !usebaselinkvelocity ) {
        Vector vbasepos = _veclinks.at(0)->_info._t.trans;
        // v_B = v_A + angularvel x (B-A)
        for(size_t i = 1; i < vLinkVelocities.size(); ++i) {
            Vector voffset = _veclinks.at(i)->_info._t.trans - vbasepos;
            vLinkVelocities[i].first -= vLinkVelocities[0].first + vLinkVelocities[0].second.cross(voffset);
            vLinkVelocities[i].second -= vLinkVelocities[0].second;
        }
        vLinkVelocities[0].first = Vector();
        vLinkVelocities[0].second = Vector();
    }
    dofvelocities.resize(0);
    if( (int)dofvelocities.capacity() < GetDOF() ) {
        dofvelocities.reserve(GetDOF());
    }
    FOREACHC(it, _vDOFOrderedJoints) {
        int parentindex = 0;
        if( !!(*it)->_attachedbodies[0] ) {
            parentindex = (*it)->_attachedbodies[0]->GetIndex();
        }
        int childindex = (*it)->_attachedbodies[1]->GetIndex();
        (*it)->_GetVelocities(dofvelocities,true,vLinkVelocities.at(parentindex),vLinkVelocities.at(childindex));
    }
}

void KinBody::_ComputeLinkAccelerations(const std::vector<dReal>& vDOFVelocities, const std::vector<dReal>& vDOFAccelerations, const std::vector< std::pair<Vector, Vector> >& vLinkVelocities, std::vector<std::pair<Vector,Vector> >& vLinkAccelerations, AccelerationMapConstPtr pexternalaccelerations) const
{
    vLinkAccelerations.resize(_veclinks.size());
    if( _veclinks.size() == 0 ) {
        return;
    }

    vector<dReal> vtempvalues, veval;
    boost::array<dReal,3> dummyvelocities = {{0,0,0}}, dummyaccelerations={{0,0,0}}; // dummy values for a joint

    // set accelerations of all links as if they were the base link
    for(size_t ilink = 0; ilink < vLinkAccelerations.size(); ++ilink) {
        vLinkAccelerations.at(ilink).first += vLinkVelocities.at(ilink).second.cross(vLinkVelocities.at(ilink).first);
        vLinkAccelerations.at(ilink).second = Vector();
    }

    if( !!pexternalaccelerations ) {
        FOREACHC(itaccel, *pexternalaccelerations) {
            vLinkAccelerations.at(itaccel->first).first += itaccel->second.first;
            vLinkAccelerations.at(itaccel->first).second += itaccel->second.second;
        }
    }

    // have to compute the velocities and accelerations ahead of time since they are dependent on the link transformations
    std::vector< std::vector<dReal> > vPassiveJointVelocities(_vPassiveJoints.size()), vPassiveJointAccelerations(_vPassiveJoints.size());
    for(size_t i = 0; i <_vPassiveJoints.size(); ++i) {
        if( vDOFAccelerations.size() > 0 ) {
            vPassiveJointAccelerations[i].resize(_vPassiveJoints[i]->GetDOF(),0);
        }
        if( vDOFVelocities.size() > 0 ) {
            if( !_vPassiveJoints[i]->IsMimic() ) {
                _vPassiveJoints[i]->GetVelocities(vPassiveJointVelocities[i]);
            }
            else {
                vPassiveJointVelocities[i].resize(_vPassiveJoints[i]->GetDOF(),0);
            }
        }
    }

    Transform tdelta;
    Vector vlocalaxis;
    std::vector<uint8_t> vlinkscomputed(_veclinks.size(),0);
    vlinkscomputed[0] = 1;

    // compute the link accelerations going through topological order
    for(size_t ijoint = 0; ijoint < _vTopologicallySortedJointsAll.size(); ++ijoint) {
        JointPtr pjoint = _vTopologicallySortedJointsAll[ijoint];
        int jointindex = _vTopologicallySortedJointIndicesAll[ijoint];
        int dofindex = pjoint->GetDOFIndex();

        // have to compute the partial accelerations for each mimic dof
        const dReal* pdofaccelerations=NULL, *pdofvelocities=NULL;
        if( dofindex >= 0 ) {
            if( vDOFAccelerations.size() ) {
                pdofaccelerations = &vDOFAccelerations.at(dofindex);
            }
            if( vDOFVelocities.size() > 0 ) {
                pdofvelocities=&vDOFVelocities.at(dofindex);
            }
        }
        if( pjoint->IsMimic() && (vDOFAccelerations.size() > 0 || vDOFVelocities.size() > 0) ) {
            // compute both partial velocity and acceleration information
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pjoint->IsMimic(i) ) {
                    vtempvalues.resize(0);
                    const std::vector<Mimic::DOFFormat>& vdofformat = pjoint->_vmimic[i]->_vdofformat;
                    FOREACHC(itdof,vdofformat) {
                        JointPtr pj = itdof->jointindex < (int)_vecjoints.size() ? _vecjoints[itdof->jointindex] : _vPassiveJoints.at(itdof->jointindex-_vecjoints.size());
                        vtempvalues.push_back(pj->GetValue(itdof->axis));
                    }
                    dummyvelocities[i] = 0;
                    dummyaccelerations[i] = 0;

                    // velocity
                    if( vDOFVelocities.size() > 0 ) {
                        int err = pjoint->_Eval(i,1,vtempvalues,veval);
                        if( err ) {
                            RAVELOG_WARN_FORMAT("failed to evaluate joint %s, fparser error %d", pjoint->GetName()%err);
                        }
                        else {
                            for(size_t ipartial = 0; ipartial < vdofformat.size(); ++ipartial) {
                                dReal partialvelocity;
                                if( vdofformat[ipartial].dofindex >= 0 ) {
                                    partialvelocity = vDOFVelocities.at(vdofformat[ipartial].dofindex);
                                }
                                else {
                                    partialvelocity = vPassiveJointVelocities.at(vdofformat[ipartial].jointindex-_vecjoints.size()).at(vdofformat[ipartial].axis);
                                }
                                if( ipartial < veval.size() ) {
                                    dummyvelocities[i] += veval.at(ipartial) * partialvelocity;
                                }
                                else {
                                    RAVELOG_DEBUG_FORMAT("cannot evaluate partial velocity for mimic joint %s, perhaps equations don't exist", pjoint->GetName());
                                }
                            }
                        }
                        // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                        if( dofindex < 0 ) {
                            vPassiveJointVelocities.at(jointindex-(int)_vecjoints.size()).at(i) = dummyvelocities[i];
                        }
                    }

                    // acceleration
                    if( vDOFAccelerations.size() > 0 ) {
                        int err = pjoint->_Eval(i,2,vtempvalues,veval);
                        if( err ) {
                            RAVELOG_WARN(str(boost::format("failed to evaluate joint %s, fparser error %d")%pjoint->GetName()%err));
                        }
                        else {
                            for(size_t ipartial = 0; ipartial < vdofformat.size(); ++ipartial) {
                                dReal partialacceleration;
                                if( vdofformat[ipartial].dofindex >= 0 ) {
                                    partialacceleration = vDOFAccelerations.at(vdofformat[ipartial].dofindex);
                                }
                                else {
                                    partialacceleration = vPassiveJointAccelerations.at(vdofformat[ipartial].jointindex-_vecjoints.size()).at(vdofformat[ipartial].axis);
                                }
                                if( ipartial < veval.size() ) {
                                    dummyaccelerations[i] += veval.at(ipartial) * partialacceleration;
                                }
                                else {
                                    RAVELOG_DEBUG_FORMAT("cannot evaluate partial acceleration for mimic joint %s, perhaps equations don't exist", pjoint->GetName());
                                }
                            }
                        }
                        // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                        if( dofindex < 0 ) {
                            vPassiveJointAccelerations.at(jointindex-(int)_vecjoints.size()).at(i) = dummyaccelerations[i];
                        }
                    }
                }
                else if( dofindex >= 0 ) {
                    // is this correct? what is a joint has a mimic and non-mimic axis?
                    dummyvelocities[i] = vDOFVelocities.at(dofindex+i);
                    dummyaccelerations[i] = vDOFAccelerations.at(dofindex+i);
                }
                else {
                    // preserve passive joint values
                    dummyvelocities[i] = vPassiveJointVelocities.at(jointindex-(int)_vecjoints.size()).at(i);
                    dummyaccelerations[i] = vPassiveJointAccelerations.at(jointindex-(int)_vecjoints.size()).at(i);
                }
            }
            pdofvelocities = &dummyvelocities[0];
            pdofaccelerations = &dummyaccelerations[0];
        }

        // do the test after mimic computation!?
        if( vlinkscomputed[pjoint->GetHierarchyChildLink()->GetIndex()] ) {
            continue;
        }

        if( vDOFVelocities.size() > 0 && !pdofvelocities ) {
            // has to be a passive joint
            pdofvelocities = &vPassiveJointVelocities.at(jointindex-(int)_vecjoints.size()).at(0);
        }
        if( vDOFAccelerations.size() > 0 && !pdofaccelerations ) {
            // has to be a passive joint
            pdofaccelerations = &vPassiveJointAccelerations.at(jointindex-(int)_vecjoints.size()).at(0);
        }

        int childindex = pjoint->GetHierarchyChildLink()->GetIndex();
        const Transform& tchild = pjoint->GetHierarchyChildLink()->GetTransform();
        const pair<Vector, Vector>& vChildVelocities = vLinkVelocities.at(childindex);
        pair<Vector, Vector>& vChildAccelerations = vLinkAccelerations.at(childindex);

        int parentindex = 0;
        if( !!pjoint->GetHierarchyParentLink() ) {
            parentindex = pjoint->GetHierarchyParentLink()->GetIndex();
        }

        const pair<Vector, Vector>& vParentVelocities = vLinkVelocities.at(parentindex);
        const pair<Vector, Vector>& vParentAccelerations = vLinkAccelerations.at(parentindex);
        Vector xyzdelta = tchild.trans - _veclinks.at(parentindex)->_info._t.trans;
        if( !!pdofaccelerations || !!pdofvelocities ) {
            tdelta = _veclinks.at(parentindex)->_info._t * pjoint->GetInternalHierarchyLeftTransform();
            vlocalaxis = pjoint->GetInternalHierarchyAxis(0);
        }

        // check out: http://en.wikipedia.org/wiki/Rotating_reference_frame
        // compute for global coordinate system
        // code for symbolic computation (python sympy)
        // t=Symbol('t'); q=Function('q')(t); dq=diff(q,t); axis=Matrix(3,1,symbols('ax,ay,az')); delta=Matrix(3,1,[Function('dx')(t), Function('dy')(t), Function('dz')(t)]); vparent=Matrix(3,1,[Function('vparentx')(t),Function('vparenty')(t),Function('vparentz')(t)]); wparent=Matrix(3,1,[Function('wparentx')(t),Function('wparenty')(t),Function('wparentz')(t)]); Mparent=Matrix(3,4,[Function('m%d%d'%(i,j))(t) for i in range(3) for j in range(4)]); c = Matrix(3,1,symbols('cx,cy,cz'))
        // hinge joints:
        // p = Mparent[0:3,3] + Mparent[0:3,0:3]*(Left[0:3,3] + Left[0:3,0:3]*Rot[0:3,0:3]*Right[0:3,3])
        // v = vparent + wparent.cross(p-Mparent[0:3,3]) + Mparent[0:3,0:3] * Left[0:3,0:3] * dq * axis.cross(Rot[0:3,0:3]*Right)
        // wparent.cross(v) = wparent.cross(vparent) + wparent.cross(wparent.cross(p-Mparent[0:3,3])) + wparent.cross(p-Mparent[0:3,3])
        // dv = vparent.diff(t) + wparent.diff(t).cross(p-Mparent[0:3,3]).transpose() + wparent.cross(v-vparent) + wparent.cross(v-vparent-wparent.cross(wparent.cross(p-Mparent[0:3,3]))) + Mparent[0:3,0:3] * Left[0:3,0:3] * (ddq * axis.cross(Rot[0:3,0:3]*Right) + dq * axis.cross(dq*axis.cross(Rot[0:3,0:3]*Right)))
        // w = wparent + Mparent[0:3,0:3]*Left[0:3,0:3]*axis*dq
        // dw = wparent.diff(t) + wparent.cross(Mparent[0:3,0:3]*Left[0:3,0:3]*axis*dq).transpose() + Mparent[0:3,0:3]*Left[0:3,0:3]*axis*ddq
        // slider:
        // v = vparent + wparent.cross(p-Mparent[0:3,3]) + Mparent[0:3,0:3]*Left[0:3,0:3]*dq*axis
        // dv = vparent.diff(t) + wparent.diff(t).cross(p-Mparent[0:3,3]).transpose() + wparent.cross(v-vparent) + wparent.cross(Mparent[0:3,0:3]*Left[0:3,0:3]*dq*axis) + Mparent[0:3,0:3]*Left[0:3,0:3]*ddq*axis
        // w = wparent
        // dw = wparent.diff(t)
        if( pjoint->GetType() == JointRevolute ) {
            vChildAccelerations.first = vParentAccelerations.first + vParentAccelerations.second.cross(xyzdelta) + vParentVelocities.second.cross((vChildVelocities.first-vParentVelocities.first)*2-vParentVelocities.second.cross(xyzdelta));
            vChildAccelerations.second = vParentAccelerations.second;
            if( !!pdofvelocities ) {
                Vector gw = tdelta.rotate(vlocalaxis*pdofvelocities[0]);
                vChildAccelerations.first += gw.cross(gw.cross(tchild.trans-tdelta.trans));
                vChildAccelerations.second += vParentVelocities.second.cross(gw);
            }
            if( !!pdofaccelerations ) {
                Vector gdw = tdelta.rotate(vlocalaxis*pdofaccelerations[0]);
                vChildAccelerations.first += gdw.cross(tchild.trans-tdelta.trans);
                vChildAccelerations.second += gdw;
            }
        }
        else if( pjoint->GetType() == JointPrismatic ) {
            Vector w = tdelta.rotate(vlocalaxis);
            vChildAccelerations.first = vParentAccelerations.first + vParentAccelerations.second.cross(xyzdelta);
            Vector angularveloctiycontrib = vChildVelocities.first-vParentVelocities.first;
            if( !!pdofvelocities ) {
                angularveloctiycontrib += w*pdofvelocities[0];
            }
            vChildAccelerations.first += vParentVelocities.second.cross(angularveloctiycontrib);
            if( !!pdofaccelerations ) {
                vChildAccelerations.first += w*pdofaccelerations[0];
            }
            vChildAccelerations.second = vParentAccelerations.second;
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_("joint type 0x%x not supported for getting link acceleration"),pjoint->GetType(),ORE_Assert);
        }
        vlinkscomputed[childindex] = 1;
    }
}

void KinBody::SetSelfCollisionChecker(CollisionCheckerBasePtr collisionchecker)
{
    if( _selfcollisionchecker != collisionchecker ) {
        _selfcollisionchecker = collisionchecker;
        // reset the internal cache
        _ResetInternalCollisionCache();
        if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() ) {
            // collision checking will not be automatically updated with environment calls, so need to do this manually
            _selfcollisionchecker->InitKinBody(shared_kinbody());
        }
    }
}

CollisionCheckerBasePtr KinBody::GetSelfCollisionChecker() const
{
    return _selfcollisionchecker;
}


void KinBody::_ComputeInternalInformation()
{
    uint64_t starttime = utils::GetMicroTime();
    _nHierarchyComputed = 1;

    int lindex=0;
    FOREACH(itlink,_veclinks) {
        (*itlink)->_index = lindex; // always reset, necessary since index cannot be initialized by custom links
        (*itlink)->_vParentLinks.clear();
        if((_veclinks.size() > 1)&&((*itlink)->GetName().size() == 0)) {
            RAVELOG_WARN(str(boost::format("%s link index %d has no name")%GetName()%lindex));
        }
        lindex++;
    }

    {
        // move any enabled passive joints to the regular joints list
        vector<JointPtr>::iterator itjoint = _vPassiveJoints.begin();
        while(itjoint != _vPassiveJoints.end()) {
            bool bmimic = false;
            for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                if( !!(*itjoint)->_vmimic[idof] ) {
                    bmimic = true;
                }
            }
            if( !bmimic && (*itjoint)->_info._bIsActive ) {
                _vecjoints.push_back(*itjoint);
                itjoint = _vPassiveJoints.erase(itjoint);
            }
            else {
                ++itjoint;
            }
        }
        // move any mimic joints to the passive joints
        itjoint = _vecjoints.begin();
        while(itjoint != _vecjoints.end()) {
            bool bmimic = false;
            for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                if( !!(*itjoint)->_vmimic[idof] ) {
                    bmimic = true;
                    break;
                }
            }
            if( bmimic || !(*itjoint)->_info._bIsActive ) {
                _vPassiveJoints.push_back(*itjoint);
                itjoint = _vecjoints.erase(itjoint);
            }
            else {
                ++itjoint;
            }
        }
        int jointindex=0;
        int dofindex=0;
        FOREACH(itjoint,_vecjoints) {
            (*itjoint)->jointindex = jointindex++;
            (*itjoint)->dofindex = dofindex;
            (*itjoint)->_info._bIsActive = true;
            dofindex += (*itjoint)->GetDOF();
        }
        FOREACH(itjoint,_vPassiveJoints) {
            (*itjoint)->jointindex = -1;
            (*itjoint)->dofindex = -1;
            (*itjoint)->_info._bIsActive = false;
        }
    }

    vector<size_t> vorder(_vecjoints.size());
    vector<int> vJointIndices(_vecjoints.size());
    _vDOFIndices.resize(GetDOF());
    for(size_t i = 0; i < _vecjoints.size(); ++i) {
        vJointIndices[i] = _vecjoints[i]->dofindex;
        for(int idof = 0; idof < _vecjoints[i]->GetDOF(); ++idof) {
            _vDOFIndices.at(vJointIndices[i]+idof) = i;
        }
        vorder[i] = i;
    }
    sort(vorder.begin(), vorder.end(), utils::index_cmp<vector<int>&>(vJointIndices));
    _vDOFOrderedJoints.resize(0);
    FOREACH(it,vorder) {
        _vDOFOrderedJoints.push_back(_vecjoints.at(*it));
    }

    try {
        // initialize all the mimic equations
        for(int ijoints = 0; ijoints < 2; ++ijoints) {
            vector<JointPtr>& vjoints = ijoints ? _vPassiveJoints : _vecjoints;
            FOREACH(itjoint,vjoints) {
                for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                    if( !!(*itjoint)->_vmimic[i] ) {
                        std::string poseq = (*itjoint)->_vmimic[i]->_equations[0], veleq = (*itjoint)->_vmimic[i]->_equations[1], acceleq = (*itjoint)->_vmimic[i]->_equations[2]; // have to copy since memory can become invalidated
                        (*itjoint)->SetMimicEquations(i,poseq,veleq,acceleq);
                    }
                }
            }
        }
        // fill Mimic::_vmimicdofs, check that there are no circular dependencies between the mimic joints
        std::map<Mimic::DOFFormat, boost::shared_ptr<Mimic> > mapmimic;
        for(int ijoints = 0; ijoints < 2; ++ijoints) {
            vector<JointPtr>& vjoints = ijoints ? _vPassiveJoints : _vecjoints;
            int jointindex=0;
            FOREACH(itjoint,vjoints) {
                Mimic::DOFFormat dofformat;
                if( ijoints ) {
                    dofformat.dofindex = -1;
                    dofformat.jointindex = jointindex+(int)_vecjoints.size();
                }
                else {
                    dofformat.dofindex = (*itjoint)->GetDOFIndex();
                    dofformat.jointindex = (*itjoint)->GetJointIndex();
                }
                for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                    dofformat.axis = idof;
                    if( !!(*itjoint)->_vmimic[idof] ) {
                        // only add if depends on mimic joints
                        FOREACH(itdofformat,(*itjoint)->_vmimic[idof]->_vdofformat) {
                            JointPtr pjoint = itdofformat->GetJoint(*this);
                            if( pjoint->IsMimic(itdofformat->axis) ) {
                                mapmimic[dofformat] = (*itjoint)->_vmimic[idof];
                                break;
                            }
                        }
                    }
                }
                ++jointindex;
            }
        }
        bool bchanged = true;
        while(bchanged) {
            bchanged = false;
            FOREACH(itmimic,mapmimic) {
                boost::shared_ptr<Mimic> mimic = itmimic->second;
                Mimic::DOFHierarchy h;
                h.dofformatindex = 0;
                FOREACH(itdofformat,mimic->_vdofformat) {
                    if( mapmimic.find(*itdofformat) == mapmimic.end() ) {
                        continue; // this is normal, just means that the parent is a regular dof
                    }
                    boost::shared_ptr<Mimic> mimicparent = mapmimic[*itdofformat];
                    FOREACH(itmimicdof, mimicparent->_vmimicdofs) {
                        if( mimicparent->_vdofformat[itmimicdof->dofformatindex] == itmimic->first ) {
                            JointPtr pjoint = itmimic->first.GetJoint(*this);
                            JointPtr pjointparent = itdofformat->GetJoint(*this);
                            throw OPENRAVE_EXCEPTION_FORMAT(_("joint index %s uses a mimic joint %s that also depends on %s! this is not allowed"), pjoint->GetName()%pjointparent->GetName()%pjoint->GetName(), ORE_Failed);
                        }
                        h.dofindex = itmimicdof->dofindex;
                        if( find(mimic->_vmimicdofs.begin(),mimic->_vmimicdofs.end(),h) == mimic->_vmimicdofs.end() ) {
                            mimic->_vmimicdofs.push_back(h);
                            bchanged = true;
                        }
                    }
                    ++h.dofformatindex;
                }
            }
        }
    }
    catch(const std::exception& ex) {
        RAVELOG_ERROR(str(boost::format("failed to set mimic equations on kinematics body %s: %s\n")%GetName()%ex.what()));
        for(int ijoints = 0; ijoints < 2; ++ijoints) {
            vector<JointPtr>& vjoints = ijoints ? _vPassiveJoints : _vecjoints;
            FOREACH(itjoint,vjoints) {
                for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                    (*itjoint)->_vmimic[i].reset();
                }
            }
        }
    }

    _vTopologicallySortedJoints.resize(0);
    _vTopologicallySortedJointsAll.resize(0);
    _vTopologicallySortedJointIndicesAll.resize(0);
    _vJointsAffectingLinks.resize(_vecjoints.size()*_veclinks.size());

    // compute the all-pairs shortest paths
    {
        _vAllPairsShortestPaths.resize(_veclinks.size()*_veclinks.size());
        FOREACH(it,_vAllPairsShortestPaths) {
            it->first = -1;
            it->second = -1;
        }
        vector<uint32_t> vcosts(_veclinks.size()*_veclinks.size(),0x3fffffff); // initialize to 2^30-1 since we'll be adding
        for(size_t i = 0; i < _veclinks.size(); ++i) {
            vcosts[i*_veclinks.size()+i] = 0;
        }
        FOREACHC(itjoint,_vecjoints) {
            if( !!(*itjoint)->GetFirstAttached() && !!(*itjoint)->GetSecondAttached() ) {
                int index = (*itjoint)->GetFirstAttached()->GetIndex()*_veclinks.size()+(*itjoint)->GetSecondAttached()->GetIndex();
                _vAllPairsShortestPaths[index] = std::pair<int16_t,int16_t>((*itjoint)->GetFirstAttached()->GetIndex(),(*itjoint)->GetJointIndex());
                vcosts[index] = 1;
                index = (*itjoint)->GetSecondAttached()->GetIndex()*_veclinks.size()+(*itjoint)->GetFirstAttached()->GetIndex();
                _vAllPairsShortestPaths[index] = std::pair<int16_t,int16_t>((*itjoint)->GetSecondAttached()->GetIndex(),(*itjoint)->GetJointIndex());
                vcosts[index] = 1;
            }
        }
        int jointindex = (int)_vecjoints.size();
        FOREACHC(itjoint,_vPassiveJoints) {
            if( !!(*itjoint)->GetFirstAttached() && !!(*itjoint)->GetSecondAttached() ) {
                int index = (*itjoint)->GetFirstAttached()->GetIndex()*_veclinks.size()+(*itjoint)->GetSecondAttached()->GetIndex();
                _vAllPairsShortestPaths[index] = std::pair<int16_t,int16_t>((*itjoint)->GetFirstAttached()->GetIndex(),jointindex);
                vcosts[index] = 1;
                index = (*itjoint)->GetSecondAttached()->GetIndex()*_veclinks.size()+(*itjoint)->GetFirstAttached()->GetIndex();
                _vAllPairsShortestPaths[index] = std::pair<int16_t,int16_t>((*itjoint)->GetSecondAttached()->GetIndex(),jointindex);
                vcosts[index] = 1;
            }
            ++jointindex;
        }
        for(size_t k = 0; k < _veclinks.size(); ++k) {
            for(size_t i = 0; i < _veclinks.size(); ++i) {
                if( i == k ) {
                    continue;
                }
                for(size_t j = 0; j < _veclinks.size(); ++j) {
                    if((j == i)||(j == k)) {
                        continue;
                    }
                    uint32_t kcost = vcosts[k*_veclinks.size()+i] + vcosts[j*_veclinks.size()+k];
                    if( vcosts[j*_veclinks.size()+i] > kcost ) {
                        vcosts[j*_veclinks.size()+i] = kcost;
                        _vAllPairsShortestPaths[j*_veclinks.size()+i] = _vAllPairsShortestPaths[k*_veclinks.size()+i];
                    }
                }
            }
        }
    }

    // Use the APAC algorithm to initialize the kinematics hierarchy: _vTopologicallySortedJoints, _vJointsAffectingLinks, Link::_vParentLinks.
    // SIMOES, Ricardo. APAC: An exact algorithm for retrieving cycles and paths in all kinds of graphs. Tékhne, Dec. 2009, no.12, p.39-55. ISSN 1654-9911.
    if((_veclinks.size() > 0)&&(_vecjoints.size() > 0)) {
        std::vector< std::vector<int> > vlinkadjacency(_veclinks.size());
        // joints with only one attachment are attached to a static link, which is attached to link 0
        FOREACHC(itjoint,_vecjoints) {
            vlinkadjacency.at((*itjoint)->GetFirstAttached()->GetIndex()).push_back((*itjoint)->GetSecondAttached()->GetIndex());
            vlinkadjacency.at((*itjoint)->GetSecondAttached()->GetIndex()).push_back((*itjoint)->GetFirstAttached()->GetIndex());
        }
        FOREACHC(itjoint,_vPassiveJoints) {
            vlinkadjacency.at((*itjoint)->GetFirstAttached()->GetIndex()).push_back((*itjoint)->GetSecondAttached()->GetIndex());
            vlinkadjacency.at((*itjoint)->GetSecondAttached()->GetIndex()).push_back((*itjoint)->GetFirstAttached()->GetIndex());
        }
        FOREACH(it,vlinkadjacency) {
            sort(it->begin(), it->end());
        }

        // all unique paths starting at the root link or static links
        std::vector< std::list< std::list<int> > > vuniquepaths(_veclinks.size());
        std::list< std::list<int> > closedloops;
        int s = 0;
        std::list< std::list<int> > S;
        FOREACH(itv,vlinkadjacency[s]) {
            std::list<int> P;
            P.push_back(s);
            P.push_back(*itv);
            S.push_back(P);
            vuniquepaths[*itv].push_back(P);
        }
        while(!S.empty()) {
            std::list<int>& P = S.front();
            int u = P.back();
            FOREACH(itv,vlinkadjacency[u]) {
                std::list<int>::iterator itfound = find(P.begin(),P.end(),*itv);
                if( itfound == P.end() ) {
                    S.push_back(P);
                    S.back().push_back(*itv);
                    vuniquepaths[*itv].push_back(S.back());
                }
                else {
                    // found a cycle
                    std::list<int> cycle;
                    while(itfound != P.end()) {
                        cycle.push_back(*itfound);
                        ++itfound;
                    }
                    if( cycle.size() > 2 ) {
                        // sort the cycle so that it starts with the lowest link index and the direction is the next lowest index
                        // this way the cycle becomes unique and can be compared for duplicates
                        itfound = cycle.begin();
                        std::list<int>::iterator itmin = itfound++;
                        while(itfound != cycle.end()) {
                            if( *itmin > *itfound ) {
                                itmin = itfound;
                            }
                            itfound++;
                        }
                        if( itmin != cycle.begin() ) {
                            cycle.splice(cycle.end(),cycle,cycle.begin(),itmin);
                        }
                        if( *++cycle.begin() > cycle.back() ) {
                            // reverse the cycle
                            cycle.reverse();
                            cycle.push_front(cycle.back());
                            cycle.pop_back();
                        }
                        if( find(closedloops.begin(),closedloops.end(),cycle) == closedloops.end() ) {
                            closedloops.push_back(cycle);
                        }
                    }
                }
            }
            S.pop_front();
        }
        // fill each link's parent links
        FOREACH(itlink,_veclinks) {
            if( (*itlink)->GetIndex() > 0 && vuniquepaths.at((*itlink)->GetIndex()).size() == 0 ) {
                RAVELOG_WARN(str(boost::format("_ComputeInternalInformation: %s has incomplete kinematics! link %s not connected to root %s")%GetName()%(*itlink)->GetName()%_veclinks.at(0)->GetName()));
            }
            FOREACH(itpath, vuniquepaths.at((*itlink)->GetIndex())) {
                OPENRAVE_ASSERT_OP(itpath->back(),==,(*itlink)->GetIndex());
                int parentindex = *---- itpath->end();
                if( find((*itlink)->_vParentLinks.begin(),(*itlink)->_vParentLinks.end(),parentindex) == (*itlink)->_vParentLinks.end() ) {
                    (*itlink)->_vParentLinks.push_back(parentindex);
                }
            }
        }
        // find the link depths (minimum path length to the root)
        vector<int> vlinkdepths(_veclinks.size(),-1);
        vlinkdepths.at(0) = 0;
        for(size_t i = 0; i < _veclinks.size(); ++i) {
            if( _veclinks[i]->IsStatic() ) {
                vlinkdepths[i] = 0;
            }
        }
        bool changed = true;
        while(changed) {
            changed = false;
            FOREACH(itlink,_veclinks) {
                if( vlinkdepths[(*itlink)->GetIndex()] == -1 ) {
                    int bestindex = -1;
                    FOREACH(itparent, (*itlink)->_vParentLinks) {
                        if( vlinkdepths[*itparent] >= 0 ) {
                            if( bestindex == -1 || (bestindex >= 0 && vlinkdepths[*itparent] < bestindex) ) {
                                bestindex = vlinkdepths[*itparent]+1;
                            }
                        }
                    }
                    if( bestindex >= 0 ) {
                        vlinkdepths[(*itlink)->GetIndex()] = bestindex;
                        changed = true;
                    }
                }
            }
        }


        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            FOREACH(itlink, _veclinks) {
                std::stringstream ss; ss << GetName() << ":" << (*itlink)->GetName() << " depth=" << vlinkdepths.at((*itlink)->GetIndex()) << ", parents=[";
                FOREACHC(itparentlink, (*itlink)->_vParentLinks) {
                    ss << _veclinks.at(*itparentlink)->GetName() << ", ";
                }
                ss << "]";
                RAVELOG_VERBOSE(ss.str());
            }
        }

        // build up a directed graph of joint dependencies
        int numjoints = (int)(_vecjoints.size()+_vPassiveJoints.size());
        // build the adjacency list
        vector<int> vjointadjacency(numjoints*numjoints,0);
        for(int ij0 = 0; ij0 < numjoints; ++ij0) {
            JointPtr j0 = ij0 < (int)_vecjoints.size() ? _vecjoints[ij0] : _vPassiveJoints[ij0-_vecjoints.size()];
            bool bj0hasstatic = (!j0->GetFirstAttached() || j0->GetFirstAttached()->IsStatic()) || (!j0->GetSecondAttached() || j0->GetSecondAttached()->IsStatic());
            // mimic joint sorting is the hardest limit
            if( j0->IsMimic() ) {
                for(int i = 0; i < j0->GetDOF(); ++i) {
                    if(j0->IsMimic(i)) {
                        FOREACH(itdofformat, j0->_vmimic[i]->_vdofformat) {
                            if( itdofformat->dofindex < 0 ) {
                                vjointadjacency[itdofformat->jointindex*numjoints+ij0] = 1;
                            }
                        }
                    }
                }
            }

            for(int ij1 = ij0+1; ij1 < numjoints; ++ij1) {
                JointPtr j1 = ij1 < (int)_vecjoints.size() ? _vecjoints[ij1] : _vPassiveJoints[ij1-_vecjoints.size()];
                bool bj1hasstatic = (!j1->GetFirstAttached() || j1->GetFirstAttached()->IsStatic()) || (!j1->GetSecondAttached() || j1->GetSecondAttached()->IsStatic());

                // test if connected to world, next in priority to mimic joints
                if( bj0hasstatic && bj1hasstatic ) {
                    continue;
                }
                if( vjointadjacency[ij1*numjoints+ij0] || vjointadjacency[ij0*numjoints+ij1] ) {
                    // already have an edge, so no reason to add any more
                    continue;
                }

                // sort by link depth
                int j0l0 = vlinkdepths[j0->GetFirstAttached()->GetIndex()];
                int j0l1 = vlinkdepths[j0->GetSecondAttached()->GetIndex()];
                int j1l0 = vlinkdepths[j1->GetFirstAttached()->GetIndex()];
                int j1l1 = vlinkdepths[j1->GetSecondAttached()->GetIndex()];
                int diff = min(j0l0,j0l1) - min(j1l0,j1l1);
                if( diff < 0 ) {
                    OPENRAVE_ASSERT_OP(min(j0l0,j0l1),<,100);
                    vjointadjacency[ij0*numjoints+ij1] = 100-min(j0l0,j0l1);
                    continue;
                }
                if( diff > 0 ) {
                    OPENRAVE_ASSERT_OP(min(j1l0,j1l1),<,100);
                    vjointadjacency[ij1*numjoints+ij0] = 100-min(j1l0,j1l1);
                    continue;
                }
                diff = max(j0l0,j0l1) - max(j1l0,j1l1);
                if( diff < 0 ) {
                    OPENRAVE_ASSERT_OP(max(j0l0,j0l1),<,100);
                    vjointadjacency[ij0*numjoints+ij1] = 100-max(j0l0,j0l1);
                    continue;
                }
                if( diff > 0 ) {
                    OPENRAVE_ASSERT_OP(max(j1l0,j1l1),<,100);
                    vjointadjacency[ij1*numjoints+ij0] = 100-max(j1l0,j1l1);
                    continue;
                }
            }
        }
        // topologically sort the joints
        _vTopologicallySortedJointIndicesAll.resize(0); _vTopologicallySortedJointIndicesAll.reserve(numjoints);
        std::list<int> noincomingedges;
        for(int i = 0; i < numjoints; ++i) {
            bool hasincoming = false;
            for(int j = 0; j < numjoints; ++j) {
                if( vjointadjacency[j*numjoints+i] ) {
                    hasincoming = true;
                    break;
                }
            }
            if( !hasincoming ) {
                noincomingedges.push_back(i);
            }
        }
        bool bcontinuesorting = true;
        while(bcontinuesorting) {
            bcontinuesorting = false;
            while(!noincomingedges.empty()) {
                int n = noincomingedges.front();
                noincomingedges.pop_front();
                _vTopologicallySortedJointIndicesAll.push_back(n);
                for(int i = 0; i < numjoints; ++i) {
                    if( vjointadjacency[n*numjoints+i] ) {
                        vjointadjacency[n*numjoints+i] = 0;
                        bool hasincoming = false;
                        for(int j = 0; j < numjoints; ++j) {
                            if( vjointadjacency[j*numjoints+i] ) {
                                hasincoming = true;
                                break;
                            }
                        }
                        if( !hasincoming ) {
                            noincomingedges.push_back(i);
                        }
                    }
                }
            }

            // go backwards so we prioritize moving joints towards the end rather than the beginning (not a formal heurstic)
            int imaxadjind = vjointadjacency[numjoints*numjoints-1];
            for(int ijoint = numjoints*numjoints-1; ijoint >= 0; --ijoint) {
                if( vjointadjacency[ijoint] > vjointadjacency[imaxadjind] ) {
                    imaxadjind = ijoint;
                }
            }
            if( vjointadjacency[imaxadjind] != 0 ) {
                bcontinuesorting = true;
                int ifirst = imaxadjind/numjoints;
                int isecond = imaxadjind%numjoints;
                if( vjointadjacency[imaxadjind] <= 2 ) { // level 1 - static constraint violated, level 2 - mimic constraint
                    JointPtr pji = ifirst < (int)_vecjoints.size() ? _vecjoints[ifirst] : _vPassiveJoints.at(ifirst-_vecjoints.size());
                    JointPtr pjj = isecond < (int)_vecjoints.size() ? _vecjoints[isecond] : _vPassiveJoints.at(isecond-_vecjoints.size());
                    RAVELOG_WARN(str(boost::format("cannot sort joints topologically %d for robot %s joints %s:%s!! forward kinematics might be buggy\n")%vjointadjacency[imaxadjind]%GetName()%pji->GetName()%pjj->GetName()));
                }
                // remove this edge
                vjointadjacency[imaxadjind] = 0;
                bool hasincoming = false;
                for(int j = 0; j < numjoints; ++j) {
                    if( vjointadjacency[j*numjoints+isecond] ) {
                        hasincoming = true;
                        break;
                    }
                }
                if( !hasincoming ) {
                    noincomingedges.push_back(isecond);
                }
            }
        }
        OPENRAVE_ASSERT_OP((int)_vTopologicallySortedJointIndicesAll.size(),==,numjoints);
        FOREACH(itindex,_vTopologicallySortedJointIndicesAll) {
            JointPtr pj = *itindex < (int)_vecjoints.size() ? _vecjoints[*itindex] : _vPassiveJoints.at(*itindex-_vecjoints.size());
            if( *itindex < (int)_vecjoints.size() ) {
                _vTopologicallySortedJoints.push_back(pj);
            }
            _vTopologicallySortedJointsAll.push_back(pj);
            //RAVELOG_INFO(str(boost::format("top: %s")%pj->GetName()));
        }

        // based on this topological sorting, find the parent link for each joint
        FOREACH(itjoint,_vTopologicallySortedJointsAll) {
            Joint& joint = **itjoint;
            int parentlinkindex = -1;
            if( !joint.GetFirstAttached() || joint.GetFirstAttached()->IsStatic() ) {
                if( !!joint.GetSecondAttached() && !joint.GetSecondAttached()->IsStatic() ) {
                    parentlinkindex = joint.GetSecondAttached()->GetIndex();
                }
            }
            else if( !joint.GetSecondAttached() || joint.GetSecondAttached()->IsStatic() ) {
                parentlinkindex = joint.GetFirstAttached()->GetIndex();
            }
            else {
                // NOTE: possibly try to choose roots that do not involve mimic joints. ikfast might have problems
                // dealing with very complex formulas
                LinkPtr plink0 = joint.GetFirstAttached(), plink1 = joint.GetSecondAttached();
                if( vlinkdepths[plink0->GetIndex()] < vlinkdepths[plink1->GetIndex()] ) {
                    parentlinkindex = plink0->GetIndex();
                }
                else if( vlinkdepths[plink0->GetIndex()] > vlinkdepths[plink1->GetIndex()] ) {
                    parentlinkindex = plink1->GetIndex();
                }
                else {
                    // depths are the same, so check the adjacent joints of each link
                    size_t link0pos=_vTopologicallySortedJointIndicesAll.size(), link1pos=_vTopologicallySortedJointIndicesAll.size();
                    FOREACHC(itparentlink,plink0->_vParentLinks) {
                        int jointindex = _vAllPairsShortestPaths[plink0->GetIndex()*_veclinks.size()+*itparentlink].second;
                        size_t pos = find(_vTopologicallySortedJointIndicesAll.begin(),_vTopologicallySortedJointIndicesAll.end(),jointindex) - _vTopologicallySortedJointIndicesAll.begin();
                        link0pos = min(link0pos,pos);
                    }
                    FOREACHC(itparentlink,plink1->_vParentLinks) {
                        int jointindex = _vAllPairsShortestPaths[plink1->GetIndex()*_veclinks.size()+*itparentlink].second;
                        size_t pos = find(_vTopologicallySortedJointIndicesAll.begin(),_vTopologicallySortedJointIndicesAll.end(),jointindex) - _vTopologicallySortedJointIndicesAll.end();
                        link1pos = min(link1pos,pos);
                    }
                    if( link0pos < link1pos ) {
                        parentlinkindex = plink0->GetIndex();
                    }
                    else if( link0pos > link1pos ) {
                        parentlinkindex = plink1->GetIndex();
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("links %s and %s have joints on the same depth %d and %d?")%plink0->GetName()%plink1->GetName()%link0pos%link1pos));
                    }
                }
            }
            if( parentlinkindex == -1 ) {
                RAVELOG_WARN(str(boost::format("could not compute parent link for joint %s")%joint.GetName()));
            }
            else if( parentlinkindex != joint.GetFirstAttached()->GetIndex() ) {
                RAVELOG_VERBOSE(str(boost::format("swapping link order of joint %s(%d)")%joint.GetName()%joint.GetJointIndex()));
                // have to swap order
                Transform tswap = joint.GetInternalHierarchyRightTransform().inverse();
                std::vector<Vector> vaxes(joint.GetDOF());
                for(size_t i = 0; i < vaxes.size(); ++i) {
                    vaxes[i] = -tswap.rotate(joint.GetInternalHierarchyAxis(i));
                }
                std::vector<dReal> vcurrentvalues;
                joint.GetValues(vcurrentvalues);
                // have to reset the link transformations temporarily in order to avoid setting a joint offset
                TransformSaver<LinkPtr> linksaver0(joint.GetFirstAttached());
                TransformSaver<LinkPtr> linksaver1(joint.GetSecondAttached());
                // assume joint values are set to 0
                joint.GetFirstAttached()->SetTransform(Transform());
                joint.GetSecondAttached()->SetTransform(joint.GetInternalHierarchyLeftTransform()*joint.GetInternalHierarchyRightTransform());
                // pass in empty joint values
                std::vector<dReal> vdummyzerovalues;
                joint._ComputeJointInternalInformation(joint.GetSecondAttached(),joint.GetFirstAttached(),tswap.trans,vaxes,vdummyzerovalues);
                // initialize joint values to the correct value
                joint._info._vcurrentvalues = vcurrentvalues;
            }

            joint._ComputeInternalStaticInformation(); // IsStatic should be computable here
        }
        // find out what links are affected by what joints.
        FOREACH(it,_vJointsAffectingLinks) {
            *it = 0;
        }
        vector<int8_t> vusedlinks;
        for(int i = 0; i < (int)_veclinks.size(); ++i) {
            vusedlinks.resize(0); vusedlinks.resize(_veclinks.size());
            FOREACH(itpath,vuniquepaths[i]) {
                FOREACH(itlink,*itpath) {
                    vusedlinks[*itlink] = 1;
                }
            }
            for(int j = 0; j < (int)_veclinks.size(); ++j) {
                if( vusedlinks[j] &&(i != j)) {
                    int jointindex = _vAllPairsShortestPaths[i*_veclinks.size()+j].second;
                    OPENRAVE_ASSERT_OP( jointindex, >=, 0 );
                    JointPtr pjoint = jointindex < (int)_vecjoints.size() ? _vecjoints[jointindex] : _vPassiveJoints.at(jointindex-_vecjoints.size());
                    if( jointindex < (int)_vecjoints.size() ) {
                        _vJointsAffectingLinks[jointindex*_veclinks.size()+i] = pjoint->GetHierarchyParentLink()->GetIndex() == i ? -1 : 1;
                    }
                    if( pjoint->IsMimic() ) {
                        for(int idof = 0; idof < pjoint->GetDOF(); ++idof) {
                            if( pjoint->IsMimic(idof) ) {
                                FOREACHC(itmimicdof,pjoint->_vmimic[idof]->_vmimicdofs) {
                                    JointPtr pjoint2 = GetJointFromDOFIndex(itmimicdof->dofindex);
                                    _vJointsAffectingLinks[pjoint2->GetJointIndex()*_veclinks.size()+i] = pjoint2->GetHierarchyParentLink()->GetIndex() == i ? -1 : 1;
                                }
                            }
                        }
                    }
                }
            }
        }

        // process the closed loops, note that determining 'degrees of freedom' of the loop is very difficult and should be left to the 'fkfast' tool
        _vClosedLoopIndices.resize(0); _vClosedLoopIndices.reserve(closedloops.size());
        _vClosedLoops.resize(0); _vClosedLoops.reserve(closedloops.size());
        FOREACH(itclosedloop,closedloops) {
            _vClosedLoopIndices.push_back(vector< std::pair<int16_t, int16_t> >());
            _vClosedLoopIndices.back().reserve(itclosedloop->size());
            _vClosedLoops.push_back(vector< std::pair<LinkPtr, JointPtr> >());
            _vClosedLoops.back().reserve(itclosedloop->size());
            // fill the links
            FOREACH(itlinkindex,*itclosedloop) {
                _vClosedLoopIndices.back().emplace_back(*itlinkindex, 0);
                _vClosedLoops.back().emplace_back(_veclinks.at(*itlinkindex), JointPtr());
            }
            // fill the joints
            for(size_t i = 0; i < _vClosedLoopIndices.back().size(); ++i) {
                int nextlink = i+1 < _vClosedLoopIndices.back().size() ? _vClosedLoopIndices.back()[i+1].first : _vClosedLoopIndices.back()[0].first;
                int jointindex = _vAllPairsShortestPaths[nextlink*_veclinks.size()+_vClosedLoopIndices.back()[i].first].second;
                _vClosedLoopIndices.back()[i].second = jointindex;
                if( jointindex < (int)_vecjoints.size() ) {
                    _vClosedLoops.back().at(i).second = _vecjoints.at(jointindex);
                }
                else {
                    _vClosedLoops.back().at(i).second = _vPassiveJoints.at(jointindex-_vecjoints.size());
                }
            }

            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                stringstream ss;
                ss << GetName() << " closedloop found: ";
                FOREACH(itlinkindex,*itclosedloop) {
                    LinkPtr plink = _veclinks.at(*itlinkindex);
                    ss << plink->GetName() << "(" << plink->GetIndex() << ") ";
                }
                RAVELOG_VERBOSE(ss.str());
            }
        }
    }

    // compute the rigidly attached links
    for(size_t ilink = 0; ilink < _veclinks.size(); ++ilink) {
        vector<int>& vattachedlinks = _veclinks[ilink]->_vRigidlyAttachedLinks;
        vattachedlinks.resize(0);
        vattachedlinks.push_back(ilink);
        if((ilink == 0)|| _veclinks[ilink]->IsStatic() ) {
            FOREACHC(itlink,_veclinks) {
                if( (*itlink)->IsStatic() ) {
                    if( (*itlink)->GetIndex() != (int)ilink ) {
                        vattachedlinks.push_back((*itlink)->GetIndex());
                    }
                }
            }
            FOREACHC(itjoint, GetJoints()) {
                if( (*itjoint)->IsStatic() ) {
                    if( !(*itjoint)->GetFirstAttached() && !!(*itjoint)->GetSecondAttached() && !(*itjoint)->GetSecondAttached()->IsStatic() ) {
                        vattachedlinks.push_back((*itjoint)->GetSecondAttached()->GetIndex());
                    }
                    if( !(*itjoint)->GetSecondAttached() && !!(*itjoint)->GetFirstAttached() && !(*itjoint)->GetFirstAttached()->IsStatic() ) {
                        vattachedlinks.push_back((*itjoint)->GetFirstAttached()->GetIndex());
                    }
                }
            }
            FOREACHC(itpassive, GetPassiveJoints()) {
                if( (*itpassive)->IsStatic() ) {
                    if( !(*itpassive)->GetFirstAttached() && !!(*itpassive)->GetSecondAttached() && !(*itpassive)->GetSecondAttached()->IsStatic() ) {
                        vattachedlinks.push_back((*itpassive)->GetSecondAttached()->GetIndex());
                    }
                    if( !(*itpassive)->GetSecondAttached() && !!(*itpassive)->GetFirstAttached() && !(*itpassive)->GetFirstAttached()->IsStatic() ) {
                        vattachedlinks.push_back((*itpassive)->GetFirstAttached()->GetIndex());
                    }
                }
            }
        }

        // breadth first search for rigid links
        for(size_t icurlink = 0; icurlink<vattachedlinks.size(); ++icurlink) {
            LinkPtr plink=_veclinks.at(vattachedlinks[icurlink]);
            FOREACHC(itjoint, _vecjoints) {
                if( (*itjoint)->IsStatic() ) {
                    if(((*itjoint)->GetFirstAttached() == plink)&& !!(*itjoint)->GetSecondAttached() &&(find(vattachedlinks.begin(),vattachedlinks.end(),(*itjoint)->GetSecondAttached()->GetIndex()) == vattachedlinks.end())) {
                        vattachedlinks.push_back((*itjoint)->GetSecondAttached()->GetIndex());
                    }
                    if(((*itjoint)->GetSecondAttached() == plink)&& !!(*itjoint)->GetFirstAttached() &&(find(vattachedlinks.begin(),vattachedlinks.end(),(*itjoint)->GetFirstAttached()->GetIndex()) == vattachedlinks.end())) {
                        vattachedlinks.push_back((*itjoint)->GetFirstAttached()->GetIndex());
                    }
                }
            }
            FOREACHC(itpassive, _vPassiveJoints) {
                if( (*itpassive)->IsStatic() ) {
                    if(((*itpassive)->GetFirstAttached() == plink)&& !!(*itpassive)->GetSecondAttached() &&(find(vattachedlinks.begin(),vattachedlinks.end(),(*itpassive)->GetSecondAttached()->GetIndex()) == vattachedlinks.end())) {
                        vattachedlinks.push_back((*itpassive)->GetSecondAttached()->GetIndex());
                    }
                    if(((*itpassive)->GetSecondAttached() == plink)&& !!(*itpassive)->GetFirstAttached() &&(find(vattachedlinks.begin(),vattachedlinks.end(),(*itpassive)->GetFirstAttached()->GetIndex()) == vattachedlinks.end())) {
                        vattachedlinks.push_back((*itpassive)->GetFirstAttached()->GetIndex());
                    }
                }
            }
        }
    }

    for(size_t ijoint = 0; ijoint < _vecjoints.size(); ++ijoint ) {
        if( _vecjoints[ijoint]->GetName().size() == 0 ) {
            RAVELOG_WARN(str(boost::format("%s joint index %d has no name")%GetName()%ijoint));
        }
    }
    for(size_t ijoint = 0; ijoint < _vPassiveJoints.size(); ++ijoint ) {
        if( _vPassiveJoints[ijoint]->GetName().size() == 0 ) {
            RAVELOG_WARN(str(boost::format("%s passive joint index %d has no name")%GetName()%ijoint));
        }
    }
    for(size_t ijoint0 = 0; ijoint0 < _vTopologicallySortedJointsAll.size(); ++ijoint0 ) {
        JointPtr pjoint0 = _vTopologicallySortedJointsAll[ijoint0];
        for(size_t ijoint1 = ijoint0+1; ijoint1 < _vTopologicallySortedJointsAll.size(); ++ijoint1 ) {
            JointPtr pjoint1 = _vTopologicallySortedJointsAll[ijoint1];
            if( pjoint0->GetName() == pjoint1->GetName() && (pjoint0->GetJointIndex() >= 0 || pjoint1->GetJointIndex() >= 0) ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("joint indices %d and %d share the same name '%s'"), pjoint0->GetJointIndex()%pjoint1->GetJointIndex()%pjoint0->GetName(), ORE_InvalidState);
            }
        }
    }

    __hashkinematics.resize(0);

    // create the adjacency list
    {
        _setAdjacentLinks.clear();
        FOREACH(itadj, _vForcedAdjacentLinks) {
            LinkPtr pl0 = GetLink(itadj->first);
            LinkPtr pl1 = GetLink(itadj->second);
            if( !!pl0 && !!pl1 ) {
                int ind0 = pl0->GetIndex();
                int ind1 = pl1->GetIndex();
                if( ind1 < ind0 ) {
                    _setAdjacentLinks.insert(ind1|(ind0<<16));
                }
                else {
                    _setAdjacentLinks.insert(ind0|(ind1<<16));
                }
            }
        }

        // make no-geometry links adjacent to all other links
        FOREACH(itlink0, _veclinks) {
            if( (*itlink0)->GetGeometries().size() == 0 ) {
                int ind0 = (*itlink0)->GetIndex();
                FOREACH(itlink1,_veclinks) {
                    if( *itlink0 != *itlink1 ) {
                        int ind1 = (*itlink1)->GetIndex();
                        if( ind1 < ind0 ) {
                            _setAdjacentLinks.insert(ind1|(ind0<<16));
                        }
                        else {
                            _setAdjacentLinks.insert(ind0|(ind1<<16));
                        }
                    }
                }
            }
        }

        if( _bMakeJoinedLinksAdjacent ) {
            FOREACH(itj, _vecjoints) {
                int ind0 = (*itj)->_attachedbodies[0]->GetIndex();
                int ind1 = (*itj)->_attachedbodies[1]->GetIndex();
                if( ind1 < ind0 ) {
                    _setAdjacentLinks.insert(ind1|(ind0<<16));
                }
                else {
                    _setAdjacentLinks.insert(ind0|(ind1<<16));
                }
            }

            FOREACH(itj, _vPassiveJoints) {
                int ind0 = (*itj)->_attachedbodies[0]->GetIndex();
                int ind1 = (*itj)->_attachedbodies[1]->GetIndex();
                if( ind1 < ind0 ) {
                    _setAdjacentLinks.insert(ind1|(ind0<<16));
                }
                else {
                    _setAdjacentLinks.insert(ind0|(ind1<<16));
                }
            }

            // if a pair links has exactly one non-static joint in the middle, then make the pair adjacent
            vector<JointPtr> vjoints;
            for(int i = 0; i < (int)_veclinks.size()-1; ++i) {
                for(int j = i+1; j < (int)_veclinks.size(); ++j) {
                    GetChain(i,j,vjoints);
                    size_t numstatic = 0;
                    FOREACH(it,vjoints) {
                        numstatic += (*it)->IsStatic();
                    }
                    if( numstatic+1 >= vjoints.size() ) {
                        if( i < j ) {
                            _setAdjacentLinks.insert(i|(j<<16));
                        }
                        else {
                            _setAdjacentLinks.insert(j|(i<<16));
                        }
                    }
                }
            }
        }
        _ResetInternalCollisionCache();
    }
    _ResolveInfoIds();
    _nHierarchyComputed = 2;
    // because of mimic joints, need to call SetDOFValues at least once, also use this to check for links that are off
    {
        vector<Transform> vprevtrans, vnewtrans;
        vector<dReal> vprevdoflastsetvalues, vnewdoflastsetvalues;
        GetLinkTransformations(vprevtrans, vprevdoflastsetvalues);
        vector<dReal> vcurrentvalues;
        // unfortunately if SetDOFValues is overloaded by the robot, it could call the robot's _UpdateGrabbedBodies, which is a problem during environment cloning since the grabbed bodies might not be initialized. Therefore, call KinBody::SetDOFValues
        GetDOFValues(vcurrentvalues);
        std::vector<UserDataPtr> vGrabbedBodies; vGrabbedBodies.swap(_vGrabbedBodies); // swap to get rid of _vGrabbedBodies
        KinBody::SetDOFValues(vcurrentvalues,CLA_CheckLimits, std::vector<int>());
        vGrabbedBodies.swap(_vGrabbedBodies); // swap back
        GetLinkTransformations(vnewtrans, vnewdoflastsetvalues);
        for(size_t i = 0; i < vprevtrans.size(); ++i) {
            if( TransformDistanceFast(vprevtrans[i],vnewtrans[i]) > 1e-5 ) {
                RAVELOG_VERBOSE(str(boost::format("link %d has different transformation after SetDOFValues (error=%f), this could be due to mimic joint equations kicking into effect.")%_veclinks.at(i)->GetName()%TransformDistanceFast(vprevtrans[i],vnewtrans[i])));
            }
        }
        for(int i = 0; i < GetDOF(); ++i) {
            if( vprevdoflastsetvalues.at(i) != vnewdoflastsetvalues.at(i) ) {
                RAVELOG_VERBOSE(str(boost::format("dof %d has different values after SetDOFValues %d!=%d, this could be due to mimic joint equations kicking into effect.")%i%vprevdoflastsetvalues.at(i)%vnewdoflastsetvalues.at(i)));
            }
        }
        _vInitialLinkTransformations = vnewtrans;
    }

    {
        // do not initialize interpolation, since it implies a motion sampling strategy
        int offset = 0;
        _spec._vgroups.resize(0);
        if( GetDOF() > 0 ) {
            ConfigurationSpecification::Group group;
            stringstream ss;
            ss << "joint_values " << GetName();
            for(int i = 0; i < GetDOF(); ++i) {
                ss << " " << i;
            }
            group.name = ss.str();
            group.dof = GetDOF();
            group.offset = offset;
            offset += group.dof;
            _spec._vgroups.push_back(group);
        }

        ConfigurationSpecification::Group group;
        group.name = str(boost::format("affine_transform %s %d")%GetName()%DOF_Transform);
        group.offset = offset;
        group.dof = RaveGetAffineDOF(DOF_Transform);
        _spec._vgroups.push_back(group);
    }

    // set the "self" extra geometry group
    std::string selfgroup("self");
    FOREACH(itlink, _veclinks) {
        if( (*itlink)->_info._mapExtraGeometries.find(selfgroup) == (*itlink)->_info._mapExtraGeometries.end() ) {
            std::vector<GeometryInfoPtr> vgeoms;
            FOREACH(itgeom, (*itlink)->_vGeometries) {
                vgeoms.push_back(GeometryInfoPtr(new GeometryInfo((*itgeom)->GetInfo())));
            }
            (*itlink)->_info._mapExtraGeometries.insert(make_pair(selfgroup, vgeoms));
        }
    }

    _bAreAllJoints1DOFAndNonCircular = true;
    for (size_t ijoint = 0; ijoint < _vecjoints.size(); ++ijoint) {
        if (_vecjoints[ijoint]->GetDOF() != 1 || _vecjoints[ijoint]->IsCircular()) {
            _bAreAllJoints1DOFAndNonCircular = false;
            break;
        }
    }

    // notify any callbacks of the changes
    std::list<UserDataWeakPtr> listRegisteredCallbacks;
    uint32_t index = 0;
    uint32_t parameters = _nParametersChanged;
    while(parameters && index < _vlistRegisteredCallbacks.size()) {
        if( (parameters & 1) &&  _vlistRegisteredCallbacks.at(index).size() > 0 ) {
            {
                boost::shared_lock< boost::shared_mutex > lock(GetInterfaceMutex());
                listRegisteredCallbacks = _vlistRegisteredCallbacks.at(index); // copy since it can be changed
            }
            FOREACH(it,listRegisteredCallbacks) {
                ChangeCallbackDataPtr pdata = boost::dynamic_pointer_cast<ChangeCallbackData>(it->lock());
                if( !!pdata ) {
                    pdata->_callback();
                }
            }
        }
        parameters >>= 1;
        index += 1;
    }
    _nParametersChanged = 0;
    RAVELOG_VERBOSE_FORMAT("initialized %s in %fs", GetName()%(1e-6*(utils::GetMicroTime()-starttime)));
}

void KinBody::_DeinitializeInternalInformation()
{
    _nHierarchyComputed = 0; // should reset to inform other elements that kinematics information might not be accurate
}

bool KinBody::IsAttached(const KinBody &body) const
{
    if(this == &body ) {
        return true;
    }
    std::set<KinBodyConstPtr> dummy;
    return _IsAttached(body, dummy);
}

void KinBody::GetAttached(std::set<KinBodyPtr>&setAttached) const
{
    setAttached.insert(boost::const_pointer_cast<KinBody>(shared_kinbody_const()));
    FOREACHC(itbody,_listAttachedBodies) {
        KinBodyPtr pattached = itbody->lock();
        if( !!pattached && setAttached.insert(pattached).second ) {
            pattached->GetAttached(setAttached);
        }
    }
}

void KinBody::GetAttached(std::set<KinBodyConstPtr>&setAttached) const
{
    setAttached.insert(shared_kinbody_const());
    FOREACHC(itbody,_listAttachedBodies) {
        KinBodyConstPtr pattached = itbody->lock();
        if( !!pattached && setAttached.insert(pattached).second ) {
            pattached->GetAttached(setAttached);
        }
    }
}

bool KinBody::HasAttached() const
{
    return _listAttachedBodies.size() > 0;
}

bool KinBody::_IsAttached(const KinBody &body, std::set<KinBodyConstPtr>&setChecked) const
{
    if( !setChecked.insert(shared_kinbody_const()).second ) {
        return false;
    }
    FOREACHC(itbody,_listAttachedBodies) {
        KinBodyConstPtr pattached = itbody->lock();
        if( !!pattached && ((pattached.get() == &body)|| pattached->_IsAttached(body,setChecked)) ) {
            return true;
        }
    }
    return false;
}

void KinBody::_AttachBody(KinBodyPtr pbody)
{
    _listAttachedBodies.push_back(pbody);
    pbody->_listAttachedBodies.push_back(shared_kinbody());
    _PostprocessChangedParameters(Prop_BodyAttached);
}

bool KinBody::_RemoveAttachedBody(KinBody &body)
{
    int numremoved = 0;
    FOREACH(it,_listAttachedBodies) {
        if( it->lock().get() == &body ) {
            _listAttachedBodies.erase(it);
            numremoved++;
            break;
        }
    }

    FOREACH(it, body._listAttachedBodies) {
        if( it->lock().get() == this ) { // need to compare lock pointer since cannot rely on shared_kinbody() since in a destructor this will crash
            body._listAttachedBodies.erase(it);
            numremoved++;
            break;
        }
    }

    if( numremoved > 0 ) {
        _PostprocessChangedParameters(Prop_BodyAttached);
    }

    return numremoved == 2;
}

void KinBody::Enable(bool bEnable)
{
    bool bchanged = false;
    FOREACH(it, _veclinks) {
        if( (*it)->_info._bIsEnabled != bEnable ) {
            (*it)->_info._bIsEnabled = bEnable;
            _nNonAdjacentLinkCache &= ~AO_Enabled;
            bchanged = true;
        }
    }
    if( bchanged ) {
        _PostprocessChangedParameters(Prop_LinkEnable);
    }
}

bool KinBody::IsEnabled() const
{
    FOREACHC(it, _veclinks) {
        if((*it)->IsEnabled()) {
            return true;
        }
    }
    return false;
}

bool KinBody::SetVisible(bool visible)
{
    bool bchanged = false;
    FOREACH(it, _veclinks) {
        FOREACH(itgeom,(*it)->_vGeometries) {
            if( (*itgeom)->IsVisible() != visible ) {
                (*itgeom)->_info._bVisible = visible;
                bchanged = true;
            }
        }
    }
    if( bchanged ) {
        _PostprocessChangedParameters(Prop_LinkDraw);
        return true;
    }
    return false;
}

bool KinBody::IsVisible() const
{
    FOREACHC(it, _veclinks) {
        if((*it)->IsVisible()) {
            return true;
        }
    }
    return false;
}

int KinBody::GetEnvironmentId() const
{
    return _environmentid;
}

int8_t KinBody::DoesAffect(int jointindex, int linkindex ) const
{
    CHECK_INTERNAL_COMPUTATION0;
    OPENRAVE_ASSERT_FORMAT(jointindex >= 0 && jointindex < (int)_vecjoints.size(), "body %s jointindex %d invalid (num joints %d)", GetName()%jointindex%_vecjoints.size(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)_veclinks.size(), "body %s linkindex %d invalid (num links %d)", GetName()%linkindex%_veclinks.size(), ORE_InvalidArguments);
    return _vJointsAffectingLinks.at(jointindex*_veclinks.size()+linkindex);
}

int8_t KinBody::DoesDOFAffectLink(int dofindex, int linkindex ) const
{
    CHECK_INTERNAL_COMPUTATION0;
    OPENRAVE_ASSERT_FORMAT(dofindex >= 0 && dofindex < GetDOF(), "body %s dofindex %d invalid (num dofs %d)", GetName()%GetDOF(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)_veclinks.size(), "body %s linkindex %d invalid (num links %d)", GetName()%linkindex%_veclinks.size(), ORE_InvalidArguments);
    int jointindex = _vDOFIndices.at(dofindex);
    return _vJointsAffectingLinks.at(jointindex*_veclinks.size()+linkindex);
}

void KinBody::SetNonCollidingConfiguration()
{
    _ResetInternalCollisionCache();
    vector<dReal> vdoflastsetvalues;
    GetLinkTransformations(_vInitialLinkTransformations, vdoflastsetvalues);
}

void KinBody::_ResetInternalCollisionCache()
{
    _nNonAdjacentLinkCache = 0x80000000;
    FOREACH(it,_vNonAdjacentLinks) {
        it->resize(0);
    }
}

bool CompareNonAdjacentFarthest(int pair0, int pair1)
{
    // order so that farthest links are first. if equal, then prioritize links that are furthest down the chain.
    int pair0link0 = (pair0&0xffff);
    int pair0link1 = ((pair0>>16)&0xffff);
    int dist0 = pair0link1 - pair0link0; // link1 > link0
    int pair1link0 = (pair1&0xffff);
    int pair1link1 = ((pair1>>16)&0xffff);
    int dist1 = pair1link1 - pair1link0; // link1 > link0
    if( dist0 == dist1 ) {
        if( pair0link1 == pair1link1 ) {
            return pair0link0 > pair1link0;
        }
        else {
            return pair0link1 > pair1link1;
        }
    }
    return dist0 > dist1;
}

const std::vector<int>& KinBody::GetNonAdjacentLinks(int adjacentoptions) const
{
    class TransformsSaver
    {
public:
        TransformsSaver(KinBodyConstPtr pbody) : _pbody(pbody) {
            _pbody->GetLinkTransformations(vcurtrans, _vdoflastsetvalues);
        }
        ~TransformsSaver() {
            for(size_t i = 0; i < _pbody->_veclinks.size(); ++i) {
                boost::static_pointer_cast<Link>(_pbody->_veclinks[i])->_info._t = vcurtrans.at(i);
            }
            for(size_t i = 0; i < _pbody->_vecjoints.size(); ++i) {
                for(int j = 0; j < _pbody->_vecjoints[i]->GetDOF(); ++j) {
                    _pbody->_vecjoints[i]->_doflastsetvalues[j] = _vdoflastsetvalues.at(_pbody->_vecjoints[i]->GetDOFIndex()+j);
                }
            }
        }
private:
        KinBodyConstPtr _pbody;
        std::vector<Transform> vcurtrans;
        std::vector<dReal> _vdoflastsetvalues;
    };

    CHECK_INTERNAL_COMPUTATION;
    if( _nNonAdjacentLinkCache & 0x80000000 ) {
        // Check for colliding link pairs given the initial pose _vInitialLinkTransformations
        // this is actually weird, we need to call the individual link collisions on a const body. in order to pull this off, we need to be very careful with the body state.
        TransformsSaver saver(shared_kinbody_const());
        CollisionCheckerBasePtr collisionchecker = !!_selfcollisionchecker ? _selfcollisionchecker : GetEnv()->GetCollisionChecker();
        CollisionOptionsStateSaver colsaver(collisionchecker,0); // have to reset the collision options
        for(size_t i = 0; i < _veclinks.size(); ++i) {
            boost::static_pointer_cast<Link>(_veclinks[i])->_info._t = _vInitialLinkTransformations.at(i);
        }
        _nUpdateStampId++; // because transforms were modified
        _vNonAdjacentLinks[0].resize(0);
        for(size_t i = 0; i < _veclinks.size(); ++i) {
            for(size_t j = i+1; j < _veclinks.size(); ++j) {
                if((_setAdjacentLinks.find(i|(j<<16)) == _setAdjacentLinks.end())&& !collisionchecker->CheckCollision(LinkConstPtr(_veclinks[i]), LinkConstPtr(_veclinks[j])) ) {
                    _vNonAdjacentLinks[0].push_back(i|(j<<16));
                }
            }
        }
        std::sort(_vNonAdjacentLinks[0].begin(), _vNonAdjacentLinks[0].end(), CompareNonAdjacentFarthest);
        _nUpdateStampId++; // because transforms were modified
        _nNonAdjacentLinkCache = 0;
    }
    if( (_nNonAdjacentLinkCache&adjacentoptions) != adjacentoptions ) {
        int requestedoptions = (~_nNonAdjacentLinkCache)&adjacentoptions;
        // find out what needs to computed
        if( requestedoptions & AO_Enabled ) {
            _vNonAdjacentLinks.at(AO_Enabled).resize(0);
            FOREACHC(itset, _vNonAdjacentLinks[0]) {
                KinBody::LinkConstPtr plink1(_veclinks.at(*itset&0xffff)), plink2(_veclinks.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    _vNonAdjacentLinks[AO_Enabled].push_back(*itset);
                }
            }
            _nNonAdjacentLinkCache |= AO_Enabled;
            std::sort(_vNonAdjacentLinks[AO_Enabled].begin(), _vNonAdjacentLinks[AO_Enabled].end(), CompareNonAdjacentFarthest);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT(_("no support for adjacentoptions %d"), adjacentoptions,ORE_InvalidArguments);
        }
    }
    return _vNonAdjacentLinks.at(adjacentoptions);
}

const std::set<int>& KinBody::GetAdjacentLinks() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _setAdjacentLinks;
}

void KinBody::SetAdjacentLinks(int linkindex0, int linkindex1)
{
    OPENRAVE_ASSERT_OP(linkindex0,!=,linkindex1);
    if( linkindex0 > linkindex1 ) {
        std::swap(linkindex0, linkindex1);
    }

    _setAdjacentLinks.insert(linkindex0|(linkindex1<<16));
    std::string linkname0 = _veclinks.at(linkindex0)->GetName();
    std::string linkname1 = _veclinks.at(linkindex1)->GetName();
    std::pair<std::string, std::string> adjpair = std::make_pair(linkname0, linkname1);
    if( find(_vForcedAdjacentLinks.begin(), _vForcedAdjacentLinks.end(), adjpair) == _vForcedAdjacentLinks.end() ) {
        _vForcedAdjacentLinks.push_back(adjpair);
    }
    _ResetInternalCollisionCache();
}

void KinBody::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    InterfaceBase::Clone(preference,cloningoptions);
    KinBodyConstPtr r = RaveInterfaceConstCast<KinBody>(preference);

    _name = r->_name;
    _nHierarchyComputed = r->_nHierarchyComputed;
    _bMakeJoinedLinksAdjacent = r->_bMakeJoinedLinksAdjacent;
    __hashkinematics = r->__hashkinematics;
    _vTempJoints = r->_vTempJoints;

    _veclinks.resize(0); _veclinks.reserve(r->_veclinks.size());
    FOREACHC(itlink, r->_veclinks) {
        LinkPtr pnewlink(new Link(shared_kinbody()));
        // TODO should create a Link::Clone method
        *pnewlink = **itlink; // be careful of copying pointers
        pnewlink->_parent = shared_kinbody();
        // have to copy all the geometries too!
        std::vector<Link::GeometryPtr> vnewgeometries(pnewlink->_vGeometries.size());
        for(size_t igeom = 0; igeom < vnewgeometries.size(); ++igeom) {
            vnewgeometries[igeom].reset(new Link::Geometry(pnewlink, pnewlink->_vGeometries[igeom]->_info));
        }
        pnewlink->_vGeometries = vnewgeometries;
        _veclinks.push_back(pnewlink);
    }

    _vecjoints.resize(0); _vecjoints.reserve(r->_vecjoints.size());
    FOREACHC(itjoint, r->_vecjoints) {
        JointPtr pnewjoint(new Joint(shared_kinbody()));
        *pnewjoint = **itjoint; // be careful of copying pointers!
        pnewjoint->_parent = shared_kinbody();
        pnewjoint->_attachedbodies[0] = _veclinks.at((*itjoint)->_attachedbodies[0]->GetIndex());
        pnewjoint->_attachedbodies[1] = _veclinks.at((*itjoint)->_attachedbodies[1]->GetIndex());
        _vecjoints.push_back(pnewjoint);
    }

    _vPassiveJoints.resize(0); _vPassiveJoints.reserve(r->_vPassiveJoints.size());
    FOREACHC(itjoint, r->_vPassiveJoints) {
        JointPtr pnewjoint(new Joint(shared_kinbody()));
        *pnewjoint = **itjoint; // be careful of copying pointers!
        pnewjoint->_parent = shared_kinbody();
        pnewjoint->_attachedbodies[0] = _veclinks.at((*itjoint)->_attachedbodies[0]->GetIndex());
        pnewjoint->_attachedbodies[1] = _veclinks.at((*itjoint)->_attachedbodies[1]->GetIndex());
        _vPassiveJoints.push_back(pnewjoint);
    }

    _vTopologicallySortedJoints.resize(0); _vTopologicallySortedJoints.resize(r->_vTopologicallySortedJoints.size());
    FOREACHC(itjoint, r->_vTopologicallySortedJoints) {
        _vTopologicallySortedJoints.push_back(_vecjoints.at((*itjoint)->GetJointIndex()));
    }
    _vTopologicallySortedJointsAll.resize(0); _vTopologicallySortedJointsAll.resize(r->_vTopologicallySortedJointsAll.size());
    FOREACHC(itjoint, r->_vTopologicallySortedJointsAll) {
        std::vector<JointPtr>::const_iterator it = find(r->_vecjoints.begin(),r->_vecjoints.end(),*itjoint);
        if( it != r->_vecjoints.end() ) {
            _vTopologicallySortedJointsAll.push_back(_vecjoints.at(it-r->_vecjoints.begin()));
        }
        else {
            it = find(r->_vPassiveJoints.begin(), r->_vPassiveJoints.end(),*itjoint);
            if( it != r->_vPassiveJoints.end() ) {
                _vTopologicallySortedJointsAll.push_back(_vPassiveJoints.at(it-r->_vPassiveJoints.begin()));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("joint %s doesn't belong to anythong?"),(*itjoint)->GetName(), ORE_Assert);
            }
        }
    }
    _vDOFOrderedJoints = r->_vDOFOrderedJoints;
    _vJointsAffectingLinks = r->_vJointsAffectingLinks;
    _vDOFIndices = r->_vDOFIndices;

    _setAdjacentLinks = r->_setAdjacentLinks;
    _vInitialLinkTransformations = r->_vInitialLinkTransformations;
    _vForcedAdjacentLinks = r->_vForcedAdjacentLinks;
    _vAllPairsShortestPaths = r->_vAllPairsShortestPaths;
    _vClosedLoopIndices = r->_vClosedLoopIndices;
    _vClosedLoops.resize(0); _vClosedLoops.reserve(r->_vClosedLoops.size());
    FOREACHC(itloop,_vClosedLoops) {
        _vClosedLoops.push_back(std::vector< std::pair<LinkPtr,JointPtr> >());
        FOREACHC(it,*itloop) {
            _vClosedLoops.back().emplace_back(_veclinks.at(it->first->GetIndex()), JointPtr());
            // the joint might be in _vPassiveJoints
            std::vector<JointPtr>::const_iterator itjoint = find(r->_vecjoints.begin(),r->_vecjoints.end(),it->second);
            if( itjoint != r->_vecjoints.end() ) {
                _vClosedLoops.back().back().second = _vecjoints.at(itjoint-r->_vecjoints.begin());
            }
            else {
                itjoint = find(r->_vPassiveJoints.begin(), r->_vPassiveJoints.end(),it->second);
                if( itjoint != r->_vPassiveJoints.end() ) {
                    _vClosedLoops.back().back().second = _vPassiveJoints.at(itjoint-r->_vPassiveJoints.begin());
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("joint %s in closed loop doesn't belong to anything?"),(*itjoint)->GetName(), ORE_Assert);
                }
            }
        }
    }

    _listAttachedBodies.clear(); // will be set in the environment
    if( !(cloningoptions & Clone_IgnoreAttachedBodies) ) {
        FOREACHC(itatt, r->_listAttachedBodies) {
            KinBodyConstPtr pattref = itatt->lock();
            if( !!pattref ) {
                _listAttachedBodies.push_back(GetEnv()->GetBodyFromEnvironmentId(pattref->GetEnvironmentId()));
            }
        }
    }

    // cannot copy the velocities since it requires the physics engine to be initialized with this kinbody, which might not happen before the clone..?
//    std::vector<std::pair<Vector,Vector> > velocities;
//    r->GetLinkVelocities(velocities);
//    SetLinkVelocities(velocities);

    // do not force-reset the callbacks!! since the ChangeCallbackData destructors will crash
    //_listRegisteredCallbacks.clear();

    // cache
    _ResetInternalCollisionCache();

    // clone the grabbed bodies, note that this can fail if the new cloned environment hasn't added the bodies yet (check out Environment::Clone)
    _vGrabbedBodies.resize(0);
    FOREACHC(itgrabbedref, r->_vGrabbedBodies) {
        GrabbedConstPtr pgrabbedref = boost::dynamic_pointer_cast<Grabbed const>(*itgrabbedref);
        if( !pgrabbedref ) {
            RAVELOG_WARN_FORMAT("env=%d, have uninitialized GrabbedConstPtr in _vGrabbedBodies", GetEnv()->GetId());
            continue;
        }

        KinBodyPtr pbodyref = pgrabbedref->_pgrabbedbody.lock();
        KinBodyPtr pgrabbedbody;
        if( !!pbodyref ) {
            //pgrabbedbody = GetEnv()->GetBodyFromEnvironmentId(pbodyref->GetEnvironmentId());
            pgrabbedbody = GetEnv()->GetKinBody(pbodyref->GetName());
            if( !pgrabbedbody ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("When cloning body '%s', could not find grabbed object '%s' in environmentid=%d"), GetName()%pbodyref->GetName()%pbodyref->GetEnv()->GetId(), ORE_InvalidState);
            }
            //BOOST_ASSERT(pgrabbedbody->GetName() == pbodyref->GetName());

            GrabbedPtr pgrabbed(new Grabbed(pgrabbedbody,_veclinks.at(KinBody::LinkPtr(pgrabbedref->_plinkrobot)->GetIndex())));
            pgrabbed->_troot = pgrabbedref->_troot;
            pgrabbed->_listNonCollidingLinks.clear();
            FOREACHC(itlinkref, pgrabbedref->_listNonCollidingLinks) {
                pgrabbed->_listNonCollidingLinks.push_back(_veclinks.at((*itlinkref)->GetIndex()));
            }
            _vGrabbedBodies.push_back(pgrabbed);
        }
    }

    // Clone self-collision checker
    _selfcollisionchecker.reset();
    if( !!r->_selfcollisionchecker ) {
        _selfcollisionchecker = RaveCreateCollisionChecker(GetEnv(), r->_selfcollisionchecker->GetXMLId());
        _selfcollisionchecker->SetCollisionOptions(r->_selfcollisionchecker->GetCollisionOptions());
        _selfcollisionchecker->SetGeometryGroup(r->_selfcollisionchecker->GetGeometryGroup());
        if( GetEnvironmentId() != 0 ) {
            // This body has been added to the environment already so can call InitKinBody.
            _selfcollisionchecker->InitKinBody(shared_kinbody());
        }
        else {
            // InitKinBody will be called when the body is added to the environment.
        }
    }

    _nUpdateStampId++; // update the stamp instead of copying
}

void KinBody::_PostprocessChangedParameters(uint32_t parameters)
{
    _nUpdateStampId++;
    if( _nHierarchyComputed == 1 ) {
        _nParametersChanged |= parameters;
        return;
    }

    if( (parameters & Prop_JointMimic) == Prop_JointMimic || (parameters & Prop_LinkStatic) == Prop_LinkStatic) {
        KinBodyStateSaver saver(shared_kinbody(),Save_LinkTransformation);
        vector<dReal> vzeros(GetDOF(),0);
        SetDOFValues(vzeros,Transform(),true);
        _ComputeInternalInformation();
    }
    // do not change hash if geometry changed!
    if( !!(parameters & (Prop_LinkDynamics|Prop_LinkGeometry|Prop_JointMimic)) ) {
        __hashkinematics.resize(0);
    }

    if( (parameters&Prop_LinkEnable) == Prop_LinkEnable ) {
        // check if any regrabbed bodies have the link in _listNonCollidingLinks and the link is enabled, or are missing the link in _listNonCollidingLinks and the link is disabled
        std::map<GrabbedPtr, list<KinBody::LinkConstPtr> > mapcheckcollisions;
        FOREACH(itlink,_veclinks) {
            if( (*itlink)->IsEnabled() ) {
                FOREACH(itgrabbed,_vGrabbedBodies) {
                    GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                    if( find(pgrabbed->GetRigidlyAttachedLinks().begin(),pgrabbed->GetRigidlyAttachedLinks().end(), *itlink) == pgrabbed->GetRigidlyAttachedLinks().end() ) {
                        std::list<KinBody::LinkConstPtr>::iterator itnoncolliding = find(pgrabbed->_listNonCollidingLinks.begin(),pgrabbed->_listNonCollidingLinks.end(),*itlink);
                        if( itnoncolliding != pgrabbed->_listNonCollidingLinks.end() ) {
                            if( pgrabbed->WasLinkNonColliding(*itlink) == 0 ) {
                                pgrabbed->_listNonCollidingLinks.erase(itnoncolliding);
                            }
                            mapcheckcollisions[pgrabbed].push_back(*itlink);
                        }
                        else {
                            // try to restore
                            if( pgrabbed->WasLinkNonColliding(*itlink) == 1 ) {
                                pgrabbed->_listNonCollidingLinks.push_back(*itlink);
                            }
                        }
                    }
                }
            }
            else {
                // add since it is disabled?
                FOREACH(itgrabbed,_vGrabbedBodies) {
                    GrabbedPtr pgrabbed = boost::dynamic_pointer_cast<Grabbed>(*itgrabbed);
                    if( find(pgrabbed->GetRigidlyAttachedLinks().begin(),pgrabbed->GetRigidlyAttachedLinks().end(), *itlink) == pgrabbed->GetRigidlyAttachedLinks().end() ) {
                        if( find(pgrabbed->_listNonCollidingLinks.begin(),pgrabbed->_listNonCollidingLinks.end(),*itlink) == pgrabbed->_listNonCollidingLinks.end() ) {
                            if( pgrabbed->WasLinkNonColliding(*itlink) != 0 ) {
                                pgrabbed->_listNonCollidingLinks.push_back(*itlink);
                            }
                        }
                    }
                }
            }
        }

//        if( mapcheckcollisions.size() > 0 ) {
//            CollisionOptionsStateSaver colsaver(GetEnv()->GetCollisionChecker(),0); // have to reset the collision options
//            FOREACH(itgrabbed, mapcheckcollisions) {
//                KinBodyPtr pgrabbedbody(itgrabbed->first->_pgrabbedbody);
//                _RemoveAttachedBody(pgrabbedbody);
//                CallOnDestruction destructionhook(boost::bind(&RobotBase::_AttachBody,this,pgrabbedbody));
//                FOREACH(itlink, itgrabbed->second) {
//                    if( pchecker->CheckCollision(*itlink, KinBodyConstPtr(pgrabbedbody)) ) {
//                        itgrabbed->first->_listNonCollidingLinks.remove(*itlink);
//                    }
//                }
//            }
//        }
    }

    std::list<UserDataWeakPtr> listRegisteredCallbacks;
    uint32_t index = 0;
    while(parameters && index < _vlistRegisteredCallbacks.size()) {
        if( (parameters & 1) &&  _vlistRegisteredCallbacks.at(index).size() > 0 ) {
            {
                boost::shared_lock< boost::shared_mutex > lock(GetInterfaceMutex());
                listRegisteredCallbacks = _vlistRegisteredCallbacks.at(index); // copy since it can be changed
            }
            FOREACH(it,listRegisteredCallbacks) {
                ChangeCallbackDataPtr pdata = boost::dynamic_pointer_cast<ChangeCallbackData>(it->lock());
                if( !!pdata ) {
                    pdata->_callback();
                }
            }
        }
        parameters >>= 1;
        index += 1;
    }
}

void KinBody::Serialize(BaseXMLWriterPtr writer, int options) const
{
    InterfaceBase::Serialize(writer,options);
}

void KinBody::serialize(std::ostream& o, int options) const
{
    o << _veclinks.size() << " ";
    FOREACHC(it,_veclinks) {
        (*it)->serialize(o,options);
    }
    o << _vecjoints.size() << " ";
    FOREACHC(it,_vecjoints) {
        (*it)->serialize(o,options);
    }
    o << _vPassiveJoints.size() << " ";
    FOREACHC(it,_vPassiveJoints) {
        (*it)->serialize(o,options);
    }
}

void KinBody::SetZeroConfiguration()
{
    std::vector<Vector> vaxes;
    FOREACH(itjoint,_vecjoints) {
        vaxes.resize((*itjoint)->GetDOF());
        for(size_t i = 0; i < vaxes.size(); ++i) {
            vaxes[i] = (*itjoint)->GetInternalHierarchyLeftTransform().rotate((*itjoint)->GetInternalHierarchyAxis(i));
        }
        (*itjoint)->_ComputeJointInternalInformation((*itjoint)->GetFirstAttached(), (*itjoint)->GetSecondAttached(),(*itjoint)->GetInternalHierarchyLeftTransform().trans,vaxes,std::vector<dReal>());
    }
}

const std::string& KinBody::GetKinematicsGeometryHash() const
{
    CHECK_INTERNAL_COMPUTATION;
    if( __hashkinematics.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        // should add dynamics since that affects a lot how part is treated.
        serialize(ss,SO_Kinematics|SO_Geometry|SO_Dynamics);
        __hashkinematics = utils::GetMD5HashString(ss.str());
    }
    return __hashkinematics;
}

void KinBody::SetConfigurationValues(std::vector<dReal>::const_iterator itvalues, uint32_t checklimits)
{
    vector<dReal> vdofvalues(GetDOF());
    if( GetDOF() > 0 ) {
        std::copy(itvalues,itvalues+GetDOF(),vdofvalues.begin());
    }
    Transform t;
    RaveGetTransformFromAffineDOFValues(t,itvalues+GetDOF(),DOF_Transform);
    SetDOFValues(vdofvalues,t,checklimits);
}

void KinBody::GetConfigurationValues(std::vector<dReal>&v) const
{
    GetDOFValues(v);
    v.resize(GetDOF()+RaveGetAffineDOF(DOF_Transform));
    RaveGetAffineDOFValuesFromTransform(v.begin()+GetDOF(),GetTransform(),DOF_Transform);
}

ConfigurationSpecification KinBody::GetConfigurationSpecification(const std::string& interpolation) const
{
    CHECK_INTERNAL_COMPUTATION;
    if( interpolation.size() == 0 ) {
        return _spec;
    }
    ConfigurationSpecification spec=_spec;
    FOREACH(itgroup,spec._vgroups) {
        itgroup->interpolation=interpolation;
    }
    return spec;
}

ConfigurationSpecification KinBody::GetConfigurationSpecificationIndices(const std::vector<int>&indices, const std::string& interpolation) const
{
    CHECK_INTERNAL_COMPUTATION;
    ConfigurationSpecification spec;
    if( indices.size() > 0 ) {
        spec._vgroups.resize(1);
        stringstream ss;
        ss << "joint_values " << GetName();
        FOREACHC(it,indices) {
            ss << " " << *it;
        }
        spec._vgroups[0].name = ss.str();
        spec._vgroups[0].dof = indices.size();
        spec._vgroups[0].offset = 0;
        spec._vgroups[0].interpolation=interpolation;
    }
    return spec;
}

UserDataPtr KinBody::RegisterChangeCallback(uint32_t properties, const boost::function<void()>&callback) const
{
    ChangeCallbackDataPtr pdata(new ChangeCallbackData(properties,callback,shared_kinbody_const()));
    boost::unique_lock< boost::shared_mutex > lock(GetInterfaceMutex());

    uint32_t index = 0;
    while(properties) {
        if( properties & 1 ) {
            if( index >= _vlistRegisteredCallbacks.size() ) {
                // have to resize _vlistRegisteredCallbacks, but have to preserve the internal lists since ChangeCallbackData keep track of the list iterators
                std::vector<std::list<UserDataWeakPtr> > vlistRegisteredCallbacks(index+1);
                for(size_t i = 0; i < _vlistRegisteredCallbacks.size(); ++i) {
                    vlistRegisteredCallbacks[i].swap(_vlistRegisteredCallbacks[i]);
                }
                _vlistRegisteredCallbacks.swap(vlistRegisteredCallbacks);
            }
            pdata->_iterators.emplace_back(index, _vlistRegisteredCallbacks.at(index).insert(_vlistRegisteredCallbacks.at(index).end(), pdata));
        }
        properties >>= 1;
        index += 1;
    }
    return pdata;
}

void KinBody::_InitAndAddLink(LinkPtr plink)
{
    CHECK_NO_INTERNAL_COMPUTATION;
    LinkInfo& info = plink->_info;

    // check to make sure there are no repeating names in already added links
    FOREACH(itlink, _veclinks) {
        if( (*itlink)->GetName() == info._name ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("link %s is declared more than once in body %s"), info._name%GetName(), ORE_InvalidArguments);
        }
    }

    plink->_index = static_cast<int>(_veclinks.size());
    plink->_vGeometries.clear();
    plink->_collision.vertices.clear();
    plink->_collision.indices.clear();
    FOREACHC(itgeominfo,info._vgeometryinfos) {
        Link::GeometryPtr geom(new Link::Geometry(plink,**itgeominfo));
        if( geom->_info._meshcollision.vertices.size() == 0 ) { // try to avoid recomputing
            geom->_info.InitCollisionMesh();
        }
        plink->_vGeometries.push_back(geom);
        plink->_collision.Append(geom->GetCollisionMesh(),geom->GetTransform());
    }
    FOREACHC(itadjacentname, info._vForcedAdjacentLinks) {
        // make sure the same pair isn't added more than once
        std::pair<std::string, std::string> adjpair = std::make_pair(info._name, *itadjacentname);
        if( find(_vForcedAdjacentLinks.begin(), _vForcedAdjacentLinks.end(), adjpair) == _vForcedAdjacentLinks.end() ) {
            _vForcedAdjacentLinks.push_back(adjpair);
        }
    }
    _veclinks.push_back(plink);
}

void KinBody::_InitAndAddJoint(JointPtr pjoint)
{
    CHECK_NO_INTERNAL_COMPUTATION;
    // check to make sure there are no repeating names in already added links
    JointInfo& info = pjoint->_info;
    FOREACH(itjoint, _vecjoints) {
        if( (*itjoint)->GetName() == info._name ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("joint %s is declared more than once in body %s"), info._name%GetName(), ORE_InvalidArguments);
        }
    }

    for(size_t i = 0; i < info._vmimic.size(); ++i) {
        if( !!info._vmimic[i] ) {
            pjoint->_vmimic[i].reset(new Mimic());
            pjoint->_vmimic[i]->_equations = info._vmimic[i]->_equations;
//
//            if( !pjoint->_vmimic[i]->_equations.at(0).empty() ) {
//                std::string poseq = pjoint->_vmimic[i]->_equations[0], veleq = pjoint->_vmimic[i]->_equations[1], acceleq = pjoint->_vmimic[i]->_equations[2]; // have to copy since memory can become invalidated
//                pjoint->SetMimicEquations(i,poseq,veleq,acceleq);
//            }
        }
    }
    LinkPtr plink0, plink1;
    FOREACHC(itlink, _veclinks) {
        if( (*itlink)->_info._name == info._linkname0 ) {
            plink0 = *itlink;
            if( !!plink1 ) {
                break;
            }
        }
        if( (*itlink)->_info._name == info._linkname1 ) {
            plink1 = *itlink;
            if( !!plink0 ) {
                break;
            }
        }
    }
    OPENRAVE_ASSERT_FORMAT(!!plink0&&!!plink1, "cannot find links '%s' and '%s' of body '%s' joint %s ", info._linkname0%info._linkname1%GetName()%info._name, ORE_Failed);
    std::vector<Vector> vaxes(pjoint->GetDOF());
    std::copy(info._vaxes.begin(),info._vaxes.begin()+vaxes.size(), vaxes.begin());
    pjoint->_ComputeJointInternalInformation(plink0, plink1, info._vanchor, vaxes, info._vcurrentvalues);
    if( info._bIsActive ) {
        _vecjoints.push_back(pjoint);
    }
    else {
        _vPassiveJoints.push_back(pjoint);
    }
}

void KinBody::_ResolveInfoIds()
{
    char sTempIndexConversion[9]; // temp memory space for converting indices to hex strings, enough space to convert uint32_t
    uint32_t nTempIndexConversion = 0; // length of sTempIndexConversion

    // go through all link infos and make sure _id is unique
    static const char pLinkIdPrefix[] = "link";
    static const char pGeometryIdPrefix[] = "geom";
    int nLinkId = 0;
    int numlinks = (int)_veclinks.size();
    for(int ilink = 0; ilink < numlinks; ++ilink) {
        KinBody::LinkInfo& linkinfo = _veclinks[ilink]->_info;
        bool bGenerateNewId = linkinfo._id.empty();
        if( !bGenerateNewId ) {
            for(int itestlink = 0; itestlink < ilink; ++itestlink) {
                if( _veclinks[itestlink]->_info._id == linkinfo._id ) {
                    bGenerateNewId = true;
                    break;
                }
            }
        }

        if( bGenerateNewId ) {
            while(1) {
                nTempIndexConversion = ConvertUIntToHex(nLinkId, sTempIndexConversion);
                bool bHasSame = false;
                for(int itestlink = 0; itestlink < numlinks; ++itestlink) {
                    const std::string& testid = _veclinks[itestlink]->_info._id;
                    if( testid.size() == sizeof(pLinkIdPrefix)-1+nTempIndexConversion ) {
                        if( strncmp(testid.c_str() + (sizeof(pLinkIdPrefix)-1), sTempIndexConversion, nTempIndexConversion) == 0 ) {
                            // matches
                            bHasSame = true;
                            break;
                        }
                    }
                }

                if( bHasSame ) {
                    nLinkId++;
                    continue;
                }

                break;
            }

            linkinfo._id = pLinkIdPrefix;
            linkinfo._id += sTempIndexConversion;
            nLinkId++;
        }

        // geometries
        {
            int nGeometryId = 0;
            const std::vector<Link::GeometryPtr>& vgeometries = _veclinks[ilink]->GetGeometries();
            int numgeometries = (int)vgeometries.size();
            for(int igeometry = 0; igeometry < numgeometries; ++igeometry) {
                KinBody::GeometryInfo& geometryinfo = vgeometries[igeometry]->_info;
                bool bGenerateNewId = geometryinfo._id.empty();
                if( !bGenerateNewId ) {
                    for(int itestgeometry = 0; itestgeometry < igeometry; ++itestgeometry) {
                        if( vgeometries[itestgeometry]->_info._id == geometryinfo._id ) {
                            bGenerateNewId = true;
                            break;
                        }
                    }
                }

                if( bGenerateNewId ) {
                    while(1) {
                        nTempIndexConversion = ConvertUIntToHex(nGeometryId, sTempIndexConversion);
                        bool bHasSame = false;
                        for(int itestgeometry = 0; itestgeometry < numgeometries; ++itestgeometry) {
                            const std::string& testid = vgeometries[itestgeometry]->_info._id;
                            if( testid.size() == sizeof(pGeometryIdPrefix)-1+nTempIndexConversion ) {
                                if( strncmp(testid.c_str() + (sizeof(pGeometryIdPrefix)-1), sTempIndexConversion, nTempIndexConversion) == 0 ) {
                                    // matches
                                    bHasSame = true;
                                    break;
                                }
                            }
                        }

                        if( bHasSame ) {
                            nGeometryId++;
                            continue;
                        }

                        break;
                    }

                    geometryinfo._id = pGeometryIdPrefix;
                    geometryinfo._id += sTempIndexConversion;
                    nGeometryId++;
                }
            }
        }
    }

    static const char pJointIdPrefix[] = "joint";
    int nJointId = 0;
    int numjoints = (int)_vecjoints.size();
    for(int ijoint = 0; ijoint < numjoints; ++ijoint) {
        KinBody::JointInfo& jointinfo = _vecjoints[ijoint]->_info;
        bool bGenerateNewId = jointinfo._id.empty();
        if( !bGenerateNewId ) {
            for(int itestjoint = 0; itestjoint < ijoint; ++itestjoint) {
                if( _vecjoints[itestjoint]->_info._id == jointinfo._id ) {
                    bGenerateNewId = true;
                    break;
                }
            }
        }

        if( bGenerateNewId ) {
            while(1) {
                nTempIndexConversion = ConvertUIntToHex(nJointId, sTempIndexConversion);
                bool bHasSame = false;
                for(int itestjoint = 0; itestjoint < numjoints; ++itestjoint) {
                    const std::string& testid = _vecjoints[itestjoint]->_info._id;
                    if( testid.size() == sizeof(pJointIdPrefix)-1+nTempIndexConversion ) {
                        if( strncmp(testid.c_str() + (sizeof(pJointIdPrefix)-1), sTempIndexConversion, nTempIndexConversion) == 0 ) {
                            // matches
                            bHasSame = true;
                            break;
                        }
                    }
                }

                if( bHasSame ) {
                    nJointId++;
                    continue;
                }

                break;
            }

            jointinfo._id = pJointIdPrefix;
            jointinfo._id += sTempIndexConversion;
            nJointId++;
        }
    }

    int numPassiveJoints = (int)_vPassiveJoints.size();
    for(int ijoint = 0; ijoint < numPassiveJoints; ++ijoint) {
        KinBody::JointInfo& jointinfo = _vPassiveJoints[ijoint]->_info;
        bool bGenerateNewId = jointinfo._id.empty();
        if( !bGenerateNewId ) {
            for(int itestjoint = 0; itestjoint < ijoint; ++itestjoint) {
                if( _vPassiveJoints[itestjoint]->_info._id == jointinfo._id ) {
                    bGenerateNewId = true;
                    break;
                }
            }
        }

        if( bGenerateNewId ) {
            while(1) {
                nTempIndexConversion = ConvertUIntToHex(nJointId, sTempIndexConversion);
                bool bHasSame = false;
                for(int itestjoint = 0; itestjoint < numPassiveJoints; ++itestjoint) {
                    const std::string& testid = _vPassiveJoints[itestjoint]->_info._id;
                    if( testid.size() == sizeof(pJointIdPrefix)-1+nTempIndexConversion ) {
                        if( strncmp(testid.c_str() + (sizeof(pJointIdPrefix)-1), sTempIndexConversion, nTempIndexConversion) == 0 ) {
                            // matches
                            bHasSame = true;
                            break;
                        }
                    }
                }

                if( bHasSame ) {
                    nJointId++;
                    continue;
                }

                break;
            }

            jointinfo._id = pJointIdPrefix;
            jointinfo._id += sTempIndexConversion;
            nJointId++;
        }
    }
}

void KinBody::ExtractInfo(KinBodyInfo& info)
{
    _ResolveInfoIds();

    info._id = _id;
    info._uri = __struri;
    info._name = _name;
    info._referenceUri = _referenceUri;
    info._interfaceType = GetXMLId();

    info._dofValues.resize(0);
    std::vector<dReal> vDOFValues;
    GetDOFValues(vDOFValues);
    for (size_t idof = 0; idof < vDOFValues.size(); ++idof) {
        JointPtr pJoint = GetJointFromDOFIndex(idof);
        int jointAxis = idof - pJoint->GetDOFIndex();
        info._dofValues.emplace_back(std::make_pair(pJoint->GetName(), jointAxis), vDOFValues[idof]);
    }

    info._transform = GetTransform();
    info._vGrabbedInfos.resize(0);
    GetGrabbedInfo(info._vGrabbedInfos);

    KinBody::KinBodyStateSaver saver(shared_kinbody());
    vector<dReal> vZeros(GetDOF(), 0);
    SetDOFValues(vZeros, KinBody::CLA_Nothing);
    SetTransform(Transform());

    // need to avoid extracting info for links and joints belonging to connected bodies
    std::vector<bool> isConnectedLink(_veclinks.size(), false);  // indicate which link comes from connectedbody
    std::vector<bool> isConnectedJoint(_vecjoints.size(), false); // indicate which joint comes from connectedbody
    std::vector<bool> isConnectedPassiveJoint(_vPassiveJoints.size(), false); // indicate which passive joint comes from connectedbody

    if (IsRobot()) {
        RobotBasePtr pRobot = RaveInterfaceCast<RobotBase>(shared_from_this());
        std::vector<KinBody::LinkPtr> resolvedLinks;
        std::vector<KinBody::JointPtr> resolvedJoints;
        FOREACHC(itConnectedBody, pRobot->GetConnectedBodies()) {
            (*itConnectedBody)->GetResolvedLinks(resolvedLinks);
            (*itConnectedBody)->GetResolvedJoints(resolvedJoints);
            KinBody::JointPtr resolvedDummyJoint = (*itConnectedBody)->GetResolvedDummyPassiveJoint();

            FOREACHC(itLink, _veclinks) {
                if (std::find(resolvedLinks.begin(), resolvedLinks.end(), *itLink) != resolvedLinks.end()) {
                    isConnectedLink[itLink-_veclinks.begin()] = true;
                }
            }
            FOREACHC(itJoint, _vecjoints) {
                if (std::find(resolvedJoints.begin(), resolvedJoints.end(), *itJoint) != resolvedJoints.end()) {
                    isConnectedJoint[itJoint-_vecjoints.begin()] = true;
                }
            }
            FOREACHC(itPassiveJoint, _vPassiveJoints) {
                if (std::find(resolvedJoints.begin(), resolvedJoints.end(), *itPassiveJoint) != resolvedJoints.end()) {
                    isConnectedPassiveJoint[itPassiveJoint-_vPassiveJoints.begin()] = true;
                } else if (resolvedDummyJoint == *itPassiveJoint) {
                    isConnectedPassiveJoint[itPassiveJoint-_vPassiveJoints.begin()] = true;
                }
            }
        }
    }

    info._vLinkInfos.reserve(_veclinks.size());
    for(size_t iLink = 0; iLink < _veclinks.size(); ++iLink) {
        if (isConnectedLink[iLink]) {
            continue;
        }
        KinBody::LinkInfoPtr pLinkInfo(new KinBody::LinkInfo());
        info._vLinkInfos.push_back(pLinkInfo);
        _veclinks[iLink]->ExtractInfo(*(info._vLinkInfos.back()));
    }

    info._vJointInfos.reserve(_vecjoints.size() + _vPassiveJoints.size());
    for(size_t iJoint = 0; iJoint < _vecjoints.size(); iJoint++) {
        if (isConnectedJoint[iJoint]) {
            continue;
        }
        KinBody::JointInfoPtr pJointInfo(new KinBody::JointInfo());
        info._vJointInfos.push_back(pJointInfo);
        _vecjoints[iJoint]->ExtractInfo(*(info._vJointInfos.back()));
    }

    for(size_t iJoint = 0; iJoint < _vPassiveJoints.size(); iJoint++) {
        if (isConnectedPassiveJoint[iJoint]) {
            continue;
        }
        KinBody::JointInfoPtr pJointInfo(new KinBody::JointInfo());
        info._vJointInfos.push_back(pJointInfo);
        _vPassiveJoints[iJoint]->ExtractInfo(*(info._vJointInfos.back()));
    }


    FOREACHC(it, GetReadableInterfaces()) {
        ReadablePtr pReadable = boost::dynamic_pointer_cast<Readable>(it->second);
        if (!!pReadable) {
            info._mReadableInterfaces[it->first] = pReadable;
        }
    }
}

UpdateFromInfoResult KinBody::UpdateFromKinBodyInfo(const KinBodyInfo& info)
{
    if(info._id != _id) {
        RAVELOG_WARN_FORMAT("body '%s' update info ids do not match %s != %s", GetName()%_id%info._id);
    }

    // links
    FOREACHC(itLinkInfo, info._vLinkInfos) {
        // find existing link in body
        std::vector<KinBody::LinkPtr>::iterator itExistingLink = _veclinks.end();
        FOREACH(itLink, _veclinks) {
            if ((*itLink)->_info._id == (*itLinkInfo)->_id) {
                itExistingLink = itLink;
                break;
            }
        }

        KinBody::LinkInfoPtr pLinkInfo = *itLinkInfo;
        if (itExistingLink != _veclinks.end()) {
            // update existing link
            UpdateFromInfoResult updateFromLinkInfoResult = UFIR_Success;
            KinBody::LinkPtr pLink = *itExistingLink;
            updateFromLinkInfoResult = pLink->UpdateFromInfo(*pLinkInfo);
            if (updateFromLinkInfoResult == UFIR_Success) {
                continue;
            }
            // link update failed.
            return updateFromLinkInfoResult;
        }

        // new links is added
        return UFIR_RequireReinitialize;
    }

    // delete links
    FOREACH(itLink, _veclinks) {
        bool stillExists = false;
        FOREACHC(itLinkInfo, info._vLinkInfos) {
            if ((*itLink)->_info._id == (*itLinkInfo)->_id) {
                stillExists = true;
                break;
            }
        }
        if (!stillExists) {
            return UFIR_RequireReinitialize;
        }
    }

    // joints
    FOREACHC(itJointInfo, info._vJointInfos) {
        // find exsiting joint in body
        std::vector<KinBody::JointPtr>::iterator itExistingJoint = _vecjoints.end();
        FOREACH(itJoint, _vecjoints) {
            if ((*itJoint)->_info._id == (*itJointInfo)->_id) {
                itExistingJoint = itJoint;
                break;
            }
        }

        if (itExistingJoint == _vecjoints.end()) {
            FOREACH(itJoint, _vPassiveJoints) {
                if ((*itJoint)->_info._id == (*itJointInfo)->_id) {
                    itExistingJoint = itJoint;
                    break;
                }
            }
        }

        KinBody::JointInfoPtr pJointInfo = *itJointInfo;
        if (itExistingJoint != _vecjoints.end() || itExistingJoint != _vPassiveJoints.end()) {
            // update current joint
            UpdateFromInfoResult updateFromJointInfoResult = UFIR_Success;
            KinBody::JointPtr pJoint = *itExistingJoint;
            updateFromJointInfoResult = pJoint->UpdateFromInfo(*pJointInfo);
            if (updateFromJointInfoResult == UFIR_Success) {
                continue;
            }
            // joint update failed;
            return updateFromJointInfoResult;
        }
        // new joints is added or deleted
        return UFIR_RequireReinitialize;
    }

    // delete joints
    FOREACH(itJoint, _vecjoints) {
        bool stillExists = false;
        FOREACHC(itJointInfo, info._vJointInfos) {
            if ((*itJoint)->_info._id == (*itJointInfo)->_id) {
                stillExists = true;
                break;
            }
        }
        if (!stillExists) {
            return UFIR_RequireReinitialize;
        }
    }

    // grabbedinfos
    bool resetGrabbed = false;
    std::vector<KinBody::GrabbedInfoPtr> vGrabbedInfos;
    GetGrabbedInfo(vGrabbedInfos);
    if (vGrabbedInfos.size() != info._vGrabbedInfos.size()) {
        resetGrabbed = true;
    }
    else {
        FOREACHC(itExistingGrabbedInfo, vGrabbedInfos) {
            bool foundGrabbedInfo = false;
            FOREACHC(itGrabbedInfo, info._vGrabbedInfos) {
                // find existing grabbedinfo
                if ((*itGrabbedInfo)->_id == (*itExistingGrabbedInfo)->_id) {
                    foundGrabbedInfo = true;
                    if ((**itGrabbedInfo) != (**itExistingGrabbedInfo)) {
                        resetGrabbed = true;
                        break;
                    }
                }
            }
            if (!foundGrabbedInfo) {
                resetGrabbed = true;
            }
            if (resetGrabbed) {
                break;
            }
        }
    }

    if (resetGrabbed) {
        std::vector<KinBody::GrabbedInfoConstPtr> grabbedInfos(info._vGrabbedInfos.begin(), info._vGrabbedInfos.end());
        ResetGrabbed(grabbedInfos);
    }

    // name
    if (GetName() != info._name) {
        SetName(info._name);
    }

    // transform
    if (GetTransform() != info._transform) {
        SetTransform(info._transform);
    }

    // dof values
    std::vector<dReal> dofValues;
    GetDOFValues(dofValues);
    bool bDOFChanged = false;
    for(std::vector<std::pair<std::pair<std::string, int>, dReal> >::const_iterator it = info._dofValues.begin(); it != info._dofValues.end(); it++) {
        // find the joint in the active chain
        JointPtr joint;
        FOREACHC(itJoint,_vecjoints) {
            if ((*itJoint)->GetName() == it->first.first) {
                joint = *itJoint;
                break;
            }
        }
        if (!joint) {
            continue;
        }
        if (it->first.second >= joint->GetDOF()) {
            continue;
        }
        if (dofValues[joint->GetDOFIndex()+it->first.second] != it->second) {
            dofValues[joint->GetDOFIndex()+it->first.second] = it->second;
            bDOFChanged = true;
        }
    }
    if (bDOFChanged) {
        SetDOFValues(dofValues);
    }

    FOREACH(it, info._mReadableInterfaces) {
        ReadablePtr pReadable = boost::dynamic_pointer_cast<Readable>(GetReadableInterface(it->first));
        if (!!pReadable) {
            if ( (*(it->second)) != (*pReadable)) {
                rapidjson::Document docReadable;
                dReal fUnitScale = 1.0;
                int options = 0;
                it->second->SerializeJSON(docReadable, docReadable.GetAllocator(), fUnitScale, options);
                pReadable->DeserializeJSON(docReadable, fUnitScale);
            }
        }
        else {
            // TODO: create a new Readable?
            SetReadableInterface(it->first, it->second);
        }
    }

    // delete readableInterface
    FOREACH(itExisting, GetReadableInterfaces()) {
        bool bFound = false;
        FOREACHC(it, info._mReadableInterfaces) {
            if (itExisting->first == it->first) {
                bFound = true;
                break;
            }
        }
        if (!bFound) {
            ClearReadableInterface(itExisting->first);
        }
    }
    return UFIR_Success;
}

} // end namespace OpenRAVE
