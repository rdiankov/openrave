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
#include <unordered_set>

// used for functions that are also used internally
#define CHECK_NO_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed == 0, "env=%s, body %s cannot be added to environment when doing this operation, current value is %d", GetEnv()->GetNameId()%GetName()%_nHierarchyComputed, ORE_InvalidState);
#define CHECK_INTERNAL_COMPUTATION0 OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed != 0, "env=%s, body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetEnv()->GetNameId()%GetName()%_nHierarchyComputed, ORE_NotInitialized);
#define CHECK_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed == 2, "env=%s, body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetEnv()->GetNameId()%GetName()%_nHierarchyComputed, ORE_NotInitialized);

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

/* given index pair i and j (i < j), convert to a scalar index as in the following table. this way, existing table entries stay valid when table is extended.
 | i\j          | 0   | 1   | 2   | 3   |
 | ------------ | --- | --- | --- | --- |
 | 0            | -   | 0   | 1   | 3   |
 | 1            | -   | -   | 2   | 4   |
 | 2            | -   | -   | -   | 5   |
 | 3            | -   | -   | -   | -   |
   this indexing is used for data structure holding symmetric 2d table information as 1d vector such as _vForcedAdjacentLinks.
   This way, when number of links increases, we do not need to restructure the existing entry.
 */
inline int _GetIndex1d(int index0, int index1)
{
    BOOST_ASSERT(index0 > 0 || index1 > 0);
    if (index0 < index1) {
        return index0 + index1 * (index1 - 1) /2;
    }
    else {
        return index1 + index0 * (index0 - 1) /2;
    }
}

inline void _ResizeVectorFor2DTable(std::vector<int8_t>& vec, size_t vectorSize)
{
    const size_t tableSize = vectorSize * (vectorSize - 1) / 2;
    if (vec.size() < tableSize) {
        vec.resize(tableSize, 0);
    }
}

static void _PrintDOFValuesForInitialLinkTransformations(const KinBody& body, const std::vector<dReal>& vdoflastsetvalues, const char* context)
{
    if( body.GetDOF() == 0 ) {
        return;
    }
    std::stringstream ssJoints;
    for(size_t iDOF = 0; iDOF < vdoflastsetvalues.size(); ++iDOF) {
        if( iDOF > 0 ) {
            ssJoints << ",";
        }
        ssJoints << vdoflastsetvalues[iDOF];
    }
    RAVELOG_INFO_FORMAT("env=%s, body '%s' _vInitialLinkTransformations is updated in %s by dofValues=[%s]", body.GetEnv()->GetNameId()%body.GetName()%context%ssJoints.str());
}

class ChangeCallbackData : public UserData
{
public:
    ChangeCallbackData(int properties, const boost::function<void()>& callback, KinBodyConstPtr pbody) : _properties(properties), _callback(callback), _pweakbody(pbody) {
    }
    virtual ~ChangeCallbackData() {
        KinBodyConstPtr pbody = _pweakbody.lock();
        if( !!pbody ) {
            std::unique_lock<boost::shared_mutex> lock(pbody->GetInterfaceMutex());
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

bool KinBody::KinBodyInfo::operator==(const KinBodyInfo& other) const {
    return _id == other._id
           && _uri == other._uri
           && _name == other._name
           && _referenceUri == other._referenceUri
           && _interfaceType == other._interfaceType
           && _dofValues == other._dofValues
           && _transform == other._transform
           && _isRobot == other._isRobot
           && AreVectorsDeepEqual(_vLinkInfos, other._vLinkInfos)
           && AreVectorsDeepEqual(_vJointInfos, other._vJointInfos)
           && AreVectorsDeepEqual(_vGrabbedInfos, other._vGrabbedInfos)
           && _mReadableInterfaces == other._mReadableInterfaces
           && (!_prAssociatedFileEntries ? !other._prAssociatedFileEntries : (!!other._prAssociatedFileEntries && *_prAssociatedFileEntries == *other._prAssociatedFileEntries));
}

void KinBody::KinBodyInfo::Reset()
{
    _id.clear();
    _name.clear();
    _uri.clear();
    _referenceUri.clear();
    _interfaceType.clear();
    _transform = Transform();
    _dofValues.clear();
    _vGrabbedInfos.clear();
    _vLinkInfos.clear();
    _vJointInfos.clear();
    _mReadableInterfaces.clear();
    _prAssociatedFileEntries.reset();
    _isRobot = false;
    _isPartial = true;
}

void KinBody::KinBodyInfo::SerializeJSON(rapidjson::Value& rKinBodyInfo, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    rKinBodyInfo.SetObject();

    if( !_id.empty() ) {
        orjson::SetJsonValueByKey(rKinBodyInfo, "id", _id, allocator);
    }
    if( !_name.empty() ) {
        orjson::SetJsonValueByKey(rKinBodyInfo, "name", _name, allocator);
    }
    if (!_referenceUri.empty()) {
        if( options & ISO_ReferenceUriHint ) {
            orjson::SetJsonValueByKey(rKinBodyInfo, "referenceUriHint", _referenceUri, allocator);
        }
        else {
            orjson::SetJsonValueByKey(rKinBodyInfo, "referenceUri", _referenceUri, allocator);
        }
    }

    // perhaps should not save "uri" since that could affect how the body is loaded later

    if( !_interfaceType.empty() ) {
        orjson::SetJsonValueByKey(rKinBodyInfo, "interfaceType", _interfaceType, allocator);
    }

    {
        Transform transform = _transform;
        transform.trans *= fUnitScale;
        orjson::SetJsonValueByKey(rKinBodyInfo, "transform", transform, allocator);
    }
    orjson::SetJsonValueByKey(rKinBodyInfo, "isRobot", _isRobot, allocator);

    if (_dofValues.size() > 0) {
        rapidjson::Value dofValues;
        dofValues.SetArray();
        dofValues.Reserve(_dofValues.size(), allocator);
        FOREACHC(itDofValue, _dofValues) {
            rapidjson::Value dofValue;
            orjson::SetJsonValueByKey(dofValue, "jointName", itDofValue->first.first, allocator);
            // don't save jointAxis unless not 0
            if( itDofValue->first.second != 0 ) {
                orjson::SetJsonValueByKey(dofValue, "jointAxis", itDofValue->first.second, allocator);
            }
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
            // skip serializing __collada__ since we won't need it in json
            if (it->first == "__collada__") {
                continue;
            }
            rapidjson::Value rReadable;
            it->second->SerializeJSON(rReadable, allocator, fUnitScale, options);
            orjson::SetJsonValueByKey(rReadableInterfaces, it->first.c_str(), rReadable, allocator);
        }
        rKinBodyInfo.AddMember("readableInterfaces", rReadableInterfaces, allocator);
    }

    rKinBodyInfo.AddMember("__isPartial__", _isPartial, allocator);

    if( !!_prAssociatedFileEntries ) {
        rapidjson::Value rAssociatedFileEntries;
        rAssociatedFileEntries.CopyFrom(*_prAssociatedFileEntries, allocator);
        rKinBodyInfo.AddMember("files", rAssociatedFileEntries, allocator);
    }
}

void KinBody::KinBodyInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    if (value.HasMember("__isPartial__") ) {
        bool isPartial = true;
        orjson::LoadJsonValue(value["__isPartial__"], isPartial);
        // even if value["__isPartial__"] is False, do not call Reset() since it could be on top of the already loaded correct struct.
        if( !isPartial ) {
            // data will be filled so apply to the struct
            _isPartial = false;
        }
    }
    orjson::LoadJsonValueByKey(value, "name", _name);
    orjson::LoadJsonValueByKey(value, "id", _id);

    if( !(options & IDO_IgnoreReferenceUri) ) {
        if (value.HasMember("referenceUri")) {
            orjson::LoadJsonValueByKey(value, "referenceUri", _referenceUri);
            AddModifiedField(KinBodyInfo::KBIF_ReferenceURI);
        }
        if (value.HasMember("uri")) {
            orjson::LoadJsonValueByKey(value, "uri", _uri); // user specifies this in case they want to control how the uri is
            AddModifiedField(KinBodyInfo::KBIF_URI);
        }
    }

    orjson::LoadJsonValueByKey(value, "interfaceType", _interfaceType);
    orjson::LoadJsonValueByKey(value, "isRobot", _isRobot);

    if (value.HasMember("grabbed")) {
        _vGrabbedInfos.reserve(value["grabbed"].Size() + _vGrabbedInfos.size());
        size_t iGrabbed = 0;
        for (rapidjson::Value::ConstValueIterator it = value["grabbed"].Begin(); it != value["grabbed"].End(); ++it, ++iGrabbed) {
            //UpdateOrCreateInfo(*it, _vGrabbedInfos, fUnitScale, options);
            const rapidjson::Value& rGrabbed = *it;
            std::string id = OpenRAVE::orjson::GetStringJsonValueByKey(rGrabbed, "id");
            bool isDeleted = OpenRAVE::orjson::GetJsonValueByKey<bool>(rGrabbed, "__deleted__", false);
            std::vector<GrabbedInfoPtr>::iterator itMatchingId = _vGrabbedInfos.end();
            std::vector<GrabbedInfoPtr>::iterator itMatchingName = _vGrabbedInfos.end();
            if (!id.empty()) {
                // only try to find old info if id is not empty
                FOREACH(itInfo, _vGrabbedInfos) {
                    if ((*itInfo)->_id == id) {
                        itMatchingId = itInfo;
                        break;
                    }
                }
            }

            std::string grabbedName = OpenRAVE::orjson::GetStringJsonValueByKey(rGrabbed, "grabbedName");
            // only try to find old info if id is not empty
            FOREACH(itInfo, _vGrabbedInfos) {
                if ((*itInfo)->_grabbedname == grabbedName) {
                    itMatchingName = itInfo;
                    if( id.empty() ) {
                        id = (*itInfo)->_id;
                    }
                    break;
                }
            }

            // here we allow items with empty id to be created because
            // when we load things from json, some id could be missing on file
            // and for the partial update case, the id should be non-empty
            if (itMatchingId != _vGrabbedInfos.end()) {
                if (isDeleted) {
                    _vGrabbedInfos.erase(itMatchingId);
                    continue;
                }
                (*itMatchingId)->DeserializeJSON(rGrabbed, fUnitScale, options);

                if( itMatchingId != itMatchingName && itMatchingName != _vGrabbedInfos.end() ) {
                    // there is another entry with matching name, so remove it
                    _vGrabbedInfos.erase(itMatchingName);
                }
                continue;
            }

            if( itMatchingName != _vGrabbedInfos.end() ) {
                if (isDeleted) {
                    _vGrabbedInfos.erase(itMatchingName);
                    continue;
                }
                (*itMatchingName)->DeserializeJSON(rGrabbed, fUnitScale, options);
                (*itMatchingName)->_id = id;
                continue;
            }

            if (isDeleted) {
                // ignore
                continue;
            }

            GrabbedInfoPtr pNewInfo(new GrabbedInfo());
            pNewInfo->DeserializeJSON(rGrabbed, fUnitScale, options);
            pNewInfo->_id = id;
            _vGrabbedInfos.push_back(pNewInfo);
        }
    }

    if (value.HasMember("links")) {
        _vLinkInfos.reserve(value["links"].Size() + _vLinkInfos.size());
        UpdateOrCreateInfosWithNameCheck(value["links"].Begin(), value["links"].End(), _vLinkInfos, "name", fUnitScale, options);

        // Assert that we don't have any duplicate link names
        std::unordered_set<OpenRAVE::string_view> usedLinkNames;
        for (const LinkInfoPtr& linkInfo : _vLinkInfos) {
            const bool didEmplace = usedLinkNames.emplace(linkInfo->_name).second;
            if (!didEmplace) {
                throw OPENRAVE_EXCEPTION_FORMAT("Body '%s' has info with multiple links sharing the same linkname '%s', which is not allowed.", _name%linkInfo->_name, ORE_Assert);
            }
        }
    }

    if (value.HasMember("joints")) {
        _vJointInfos.reserve(value["joints"].Size() + _vJointInfos.size());
        UpdateOrCreateInfosWithNameCheck(value["joints"].Begin(), value["joints"].End(), _vJointInfos, "name", fUnitScale, options);
    }

    if (value.HasMember("dofValues")) {
        _dofValues.resize(0);
        for(rapidjson::Value::ConstValueIterator itr = value["dofValues"].Begin(); itr != value["dofValues"].End(); ++itr) {
            if (itr->IsObject() && itr->HasMember("jointName") && itr->HasMember("value")) {
                std::string jointName;
                int jointAxis = 0; // default
                dReal dofValue=0;
                orjson::LoadJsonValueByKey(*itr, "jointName", jointName);
                orjson::LoadJsonValueByKey(*itr, "jointAxis", jointAxis);
                orjson::LoadJsonValueByKey(*itr, "value", dofValue);
                _dofValues.emplace_back(std::make_pair(jointName, jointAxis), dofValue);
            }
        }
        AddModifiedField(KinBodyInfo::KBIF_DOFValues);
    }

    if (value.HasMember("readableInterfaces") && value["readableInterfaces"].IsObject()) {
        for (rapidjson::Value::ConstMemberIterator it = value["readableInterfaces"].MemberBegin(); it != value["readableInterfaces"].MemberEnd(); ++it) {
            // skip over __collada__ since it will most likely fail to deserialize
            if (strcmp(it->name.GetString(), "__collada__") == 0 ) {
                continue;
            }
            _DeserializeReadableInterface(it->name.GetString(), it->value, fUnitScale);
        }
    }

    if (value.HasMember("transform")) {
        orjson::LoadJsonValueByKey(value, "transform", _transform);
        _transform.trans *= fUnitScale;  // partial update should only mutliply fUnitScale once if the key is in value
        AddModifiedField(KinBodyInfo::KBIF_Transform);
    }

    rapidjson::Value::ConstMemberIterator itFiles = value.FindMember("files");
    if( itFiles != value.MemberEnd() ) {
        if( !_prAssociatedFileEntries ) {
            _prAssociatedFileEntries.reset(new rapidjson::Document());
        }
        else {
            *_prAssociatedFileEntries = rapidjson::Document();
        }
        _prAssociatedFileEntries->CopyFrom(itFiles->value, _prAssociatedFileEntries->GetAllocator());
    }
}

void KinBody::KinBodyInfo::_DeserializeReadableInterface(const std::string& id, const rapidjson::Value& rReadable, dReal fUnitScale)
{
    std::map<std::string, ReadablePtr>::iterator itReadable = _mReadableInterfaces.find(id);
    ReadablePtr pReadable;
    if(itReadable != _mReadableInterfaces.end()) {
        pReadable = itReadable->second;
    }
    BaseJSONReaderPtr pReader = RaveCallJSONReader(PT_KinBody, id, pReadable, AttributesList());
    if (!!pReader) {
        pReader->DeserializeJSON(rReadable, fUnitScale);
        _mReadableInterfaces[id] = pReader->GetReadable();
        return;
    }
    if (rReadable.IsString()) {
        // When parsing c-strings as readables need to make sure we use the pointer + length constructor to handle strings with null bytes
        StringReadablePtr pStringReadable(new StringReadable(id, rReadable.GetString(), rReadable.GetStringLength()));
        _mReadableInterfaces[id] = pStringReadable;
        return;
    }
    JSONReadablePtr pReadableJSON(new JSONReadable(id, rReadable));
    _mReadableInterfaces[id] = pReadableJSON;
    // RAVELOG_WARN_FORMAT("deserialize readable interface '%s' failed for body '%s' (uri '%s'), perhaps need to call 'RaveRegisterJSONReader' with the appropriate reader.", id%_name%(_uri.empty() ? _referenceUri : _uri));
}

KinBody::KinBody(InterfaceType type, EnvironmentBasePtr penv) : InterfaceBase(type, penv)
{
    _nHierarchyComputed = 0;
    _nParametersChanged = 0;
    _bMakeJoinedLinksAdjacent = true;
    _environmentBodyIndex = 0;
    _nNonAdjacentLinkCache = 0x80000000;
    _nUpdateStampId = 0;
    _bAreAllJoints1DOFAndNonCircular = false;
    _lastModifiedAtUS = 0;
    _revisionId = 0;
}

KinBody::~KinBody()
{
    RAVELOG_VERBOSE_FORMAT("env=%s, destructing kinbody '%s'", GetEnv()->GetNameId()%GetName());
    Destroy();
}

void KinBody::Destroy()
{
    //OPENRAVE_ASSERT_OP_FORMAT(GetEnvironmentBodyIndex(),==,0, "env=%s, destroying body '%s' while it is still in the environment!", GetEnv()->GetNameId()%GetName(), ORE_Assert);
    if( GetEnvironmentBodyIndex() != 0 ) {
        RAVELOG_DEBUG_FORMAT("env=%s, destroying body '%s' with bodyIndex=%d while it is still in the environment.", GetEnv()->GetNameId()%GetName()%GetEnvironmentBodyIndex());
    }

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
    _vLinkTransformPointers.clear();
    _vecjoints.clear();
    _vTopologicallySortedJoints.clear();
    _vTopologicallySortedJointsAll.clear();
    _vDOFOrderedJoints.clear();
    _vPassiveJoints.clear();
    _vJointsAffectingLinks.clear();
    _vDOFIndices.clear();

    _vAdjacentLinks.clear();
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

    __hashKinematicsGeometryDynamics.resize(0);
    ClearReadableInterfaces();
}

bool KinBody::InitFromBoxes(const std::vector<AABB>& vaabbs, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentBodyIndex()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
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
    _vLinkTransformPointers.clear();
    __struri = uri;
    _referenceUri.clear(); // because completely removing the previous body, should reset
    _prAssociatedFileEntries.reset();
    return true;
}

bool KinBody::InitFromBoxes(const std::vector<OBB>& vobbs, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentBodyIndex()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
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
    _vLinkTransformPointers.clear();
    __struri = uri;
    _referenceUri.clear(); // because completely removing the previous body, should reset
    _prAssociatedFileEntries.reset();
    return true;
}

bool KinBody::InitFromSpheres(const std::vector<Vector>& vspheres, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentBodyIndex()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
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
    _vLinkTransformPointers.clear();
    __struri = uri;
    _referenceUri.clear(); // because completely removing the previous body, should reset
    _prAssociatedFileEntries.reset();
    return true;
}

bool KinBody::InitFromTrimesh(const TriMesh& trimesh, bool visible, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentBodyIndex()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
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
    _vLinkTransformPointers.clear();
    __struri = uri;
    _referenceUri.clear(); // because completely removing the previous body, should reset
    _prAssociatedFileEntries.reset();
    return true;
}

bool KinBody::InitFromGeometries(const std::vector<KinBody::GeometryInfo>& geometries, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentBodyIndex() == 0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    OPENRAVE_ASSERT_OP_FORMAT(geometries.size(), >, 0, "Cannot initializing body '%s' with no geometries.", GetName(), ORE_Failed);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_info._name = "base";
    plink->_vGeometries.reserve(geometries.size());
    plink->_info._bStatic = true;

    // Initialize each of our geometries and track the total vertex/index count
    unsigned totalVertices = 0, totalIndices = 0;
    FOREACHC(geomIt, geometries) {
        Link::GeometryPtr geom {new KinBody::Link::Geometry(plink, *geomIt)};
        geom->_info.InitCollisionMesh();
        const TriMesh& mesh = geom->GetCollisionMesh();
        totalVertices += mesh.vertices.size();
        totalIndices += mesh.indices.size();
        plink->_vGeometries.push_back(geom);
    }

    // Once we have all of the geometries initialized and know their total size, reserve space in our unified collision mesh and append them all at once to reduce reallocs
    plink->_collision.vertices.reserve(totalVertices);
    plink->_collision.indices.reserve(totalIndices);
    for (const KinBody::GeometryPtr& geom : plink->_vGeometries) {
        plink->_collision.Append(geom->GetCollisionMesh(), geom->GetTransform());
    }

    _veclinks.push_back(plink);
    _vLinkTransformPointers.clear();
    __struri = uri;
    _referenceUri.clear(); // because completely removing the previous body, should reset
    _prAssociatedFileEntries.reset();
    return true;
}

void KinBody::SetLinkGeometriesFromGroup(const std::string& geomname, const bool propagateGroupNameToSelfCollisionChecker)
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

    GetEnv()->GetCollisionChecker()->SetBodyGeometryGroup(shared_kinbody_const(), geomname);
    if( !!_selfcollisionchecker && _selfcollisionchecker != GetEnv()->GetCollisionChecker() && propagateGroupNameToSelfCollisionChecker ) {
        _selfcollisionchecker->SetBodyGeometryGroup(shared_kinbody_const(), geomname);
    }
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

void KinBody::_InitLinkFromInfo(KinBody::LinkPtr& linkPtr, const KinBody::LinkInfo& linkInfo)
{
    linkPtr->_vGeometries.clear();
    linkPtr->_collision.vertices.clear();
    linkPtr->_collision.indices.clear();

    linkPtr->_vGeometries.reserve(linkInfo._vgeometryinfos.size());

    for (const KinBody::GeometryInfoPtr& geomInfo : linkInfo._vgeometryinfos) {
        KinBody::Link::GeometryPtr geom(new KinBody::Link::Geometry(linkPtr, *geomInfo));
        if (geom->_info._meshcollision.vertices.size() == 0) { // try to avoid recomputing
            geom->_info.InitCollisionMesh();
        }
        linkPtr->_vGeometries.push_back(geom);
        linkPtr->_collision.Append(geom->GetCollisionMesh(), geom->GetTransform());
    }

    FOREACH(it, linkInfo._mReadableInterfaces) {
        linkPtr->SetReadableInterface(it->first, it->second);
    }
}

static const KinBody::LinkInfo& _ResolveLinkInfo(const KinBody::LinkInfoConstPtr& linkInfo) { return *linkInfo; }
static const KinBody::LinkInfo& _ResolveLinkInfo(const KinBody::LinkInfo& linkInfo) { return linkInfo; }
template <typename LinkInfoT>
void KinBody::_InitWithInitialLinks(const std::vector<LinkInfoT>& linkInfos)
{
    CHECK_NO_INTERNAL_COMPUTATION;
    BOOST_ASSERT(_veclinks.empty()); // We assume we are the first call to initialize links

    // Keep track of link names as we go to deduplicate
    std::unordered_set<std::string> existingLinkNames;
    existingLinkNames.reserve(linkInfos.size());

    _veclinks.reserve(linkInfos.size());
    for (const LinkInfoT& linkInfo : linkInfos) {
        LinkPtr plink(new Link(shared_kinbody()));
        plink->_info = _ResolveLinkInfo(linkInfo);
        LinkInfo& info = plink->_info;

        // check to make sure there are no repeating names in already added links
        const bool isLinkNameUnique = existingLinkNames.emplace(info._name).second;
        if (!isLinkNameUnique) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("link '%s' is declared more than once in body '%s', uri is '%s'"), info._name%GetName()%GetURI(), ORE_InvalidArguments);
        }

        // Initialize the link from the associated info and add to our list of links
        plink->_index = static_cast<int>(_veclinks.size());
        _InitLinkFromInfo(plink, info);
        _veclinks.push_back(std::move(plink));
    }

    _vLinkTransformPointers.clear();
    __hashKinematicsGeometryDynamics.resize(0);
}

bool KinBody::Init(const std::vector<KinBody::LinkInfoConstPtr>& linkinfos, const std::vector<KinBody::JointInfoConstPtr>& jointinfos, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentBodyIndex()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    Destroy();
    _InitWithInitialLinks(linkinfos);
    _vecjoints.reserve(jointinfos.size());
    FOREACHC(itjointinfo, jointinfos) {
        JointInfoConstPtr rawinfo = *itjointinfo;
        JointPtr pjoint(new Joint(shared_kinbody()));
        pjoint->_info = *rawinfo;
        _InitAndAddJoint(pjoint);
    }
    _vLinkTransformPointers.clear();
    __struri = uri;
    _referenceUri.clear(); // because completely removing the previous body, should reset
    _prAssociatedFileEntries.reset();
    return true;
}

void KinBody::InitFromLinkInfos(const std::vector<LinkInfo>& linkinfos, const std::string& uri)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentBodyIndex()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    OPENRAVE_ASSERT_OP(linkinfos.size(),>,0);
    Destroy();
    _InitWithInitialLinks(linkinfos);
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
    _vLinkTransformPointers.clear();
    __struri = uri;
    _referenceUri.clear(); // because completely removing the previous body, should reset
    _prAssociatedFileEntries.reset();
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
    if( info._vLinkInfos.size() > 0 ) {
        _baseLinkInBodyTransform = info._vLinkInfos[0]->GetTransform();
        _invBaseLinkInBodyTransform = _baseLinkInBodyTransform.inverse();
    }
    else {
        _baseLinkInBodyTransform = _invBaseLinkInBodyTransform = Transform();
    }

    FOREACH(it, info._mReadableInterfaces) {
        SetReadableInterface(it->first, it->second);
    }

    if( GetXMLId() != info._interfaceType ) {
        RAVELOG_WARN_FORMAT("body '%s' interfaceType does not match %s != %s", GetName()%GetXMLId()%info._interfaceType);
    }

    if( !!info._prAssociatedFileEntries ) {
        if( !_prAssociatedFileEntries ) {
            _prAssociatedFileEntries.reset(new rapidjson::Document());
        }
        else {
            *_prAssociatedFileEntries = rapidjson::Document();
        }
        _prAssociatedFileEntries->CopyFrom(*info._prAssociatedFileEntries, _prAssociatedFileEntries->GetAllocator());
    }

    return true;
}

void KinBody::SetName(const std::string& newname)
{
    OPENRAVE_ASSERT_OP(newname.size(), >, 0);
    if( _name != newname ) {
        if (GetEnvironmentBodyIndex() > 0) {
            // need to update some cache stored in env, but only if this body is added to env.
            // otherwise, we may modify the cache for origbody unexpectedly in the following scenario
            // 1. clonebody = origbody.Clone()
            // 2. clonebody.SetName('cloned') // this shouldn't cause cache in env to be modified for origbody
            if( !GetEnv()->NotifyKinBodyNameChanged(_name, newname) ) {
                throw OPENRAVE_EXCEPTION_FORMAT("env=%d, cannot change body '%s' name to '%s' since it conflicts with another body", GetEnv()->GetId()%_name%newname, ORE_BodyNameConflict);
            }
        }
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

void KinBody::SetId(const std::string& newid)
{
    // allow empty id to be set
    if( _id != newid ) {
        if (GetEnvironmentBodyIndex() > 0) {
            if( !GetEnv()->NotifyKinBodyIdChanged(_id, newid) ) {
                throw OPENRAVE_EXCEPTION_FORMAT("env=%d, cannot change body '%s' id from '%s' -> '%s' since it conflicts with another body", GetEnv()->GetId()%_name%_id%newid, ORE_BodyIdConflict);
            }
        }
        _id = newid;
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetValue(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetVelocity(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            std::pair<dReal, dReal> res = joint.GetLimit(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            std::pair<dReal, dReal> res = joint.GetVelocityLimit(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetMaxVel(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetAccelerationLimit(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetJerkLimit(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetHardVelocityLimit(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetHardAccelerationLimit(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetHardJerkLimit(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetResolution(dofindices[i]-joint.GetDOFIndex());
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            v[i] = joint.GetWeight(dofindices[i]-joint.GetDOFIndex());
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
            Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            joint._info._vweights.at(dofindices[i]-joint.GetDOFIndex()) = v[i];
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
            Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            joint._info._vresolution.at(dofindices[i]-joint.GetDOFIndex()) = v[i];
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
                    for(int j = 0; j < (*it)->GetDOF(); ++j) {
                        if( (*it)->IsRevolute(j) && !(*it)->IsCircular(j) ) {
                            // TODO, necessary to set wrap?
                            if( (*it)->_info._vlowerlimit.at(j) < -PI || (*it)->_info._vupperlimit.at(j) > PI) {
                                (*it)->SetWrapOffset(0.5f * ((*it)->_info._vlowerlimit.at(j) + (*it)->_info._vupperlimit.at(j)),j);
                            }
                            else {
                                (*it)->SetWrapOffset(0,j);
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
            Joint& joint = _GetJointFromDOFIndex(dofindices[index]);
            int iaxis = dofindices[index]-joint.GetDOFIndex();
            if( joint._info._vlowerlimit.at(iaxis) != lower[index] || joint._info._vupperlimit.at(iaxis) != upper[index] ) {
                bChanged = true;
                joint._info._vlowerlimit.at(iaxis) = lower[index];
                joint._info._vupperlimit.at(iaxis) = upper[index];
                if( joint.IsRevolute(iaxis) && !joint.IsCircular(iaxis) ) {
                    // TODO, necessary to set wrap?
                    if( joint._info._vlowerlimit.at(iaxis) < -PI || joint._info._vupperlimit.at(iaxis) > PI) {
                        joint.SetWrapOffset(0.5f * (joint._info._vlowerlimit.at(iaxis) + joint._info._vupperlimit.at(iaxis)),iaxis);
                    }
                    else {
                        joint.SetWrapOffset(0,iaxis);
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
            const Joint& joint = _GetJointFromDOFIndex(dofindices[i]);
            if( joint.IsCircular(dofindices[i]-joint.GetDOFIndex()) ) {
                int iaxis = dofindices[i]-joint.GetDOFIndex();
                q1[i] = utils::NormalizeCircularAngle(q1[i]-q2[i], joint._vcircularlowerlimit.at(iaxis), joint._vcircularupperlimit.at(iaxis));
            }
            else {
                q1[i] -= q2[i];
            }
        }
    }
}

// like apply transform except everything is relative to the first frame
void KinBody::SetTransform(const Transform& bodyTransform)
{
    if( _veclinks.size() == 0 ) {
        return;
    }
    Transform baseLinkTransform = bodyTransform * _baseLinkInBodyTransform;
    Transform tbaseinv = _veclinks.front()->GetTransform().inverse();
    Transform tapply = baseLinkTransform * tbaseinv;
    FOREACH(itlink, _veclinks) {
        (*itlink)->SetTransform(tapply * (*itlink)->GetTransform());
    }
    _UpdateGrabbedBodies();
    _PostprocessChangedParameters(Prop_LinkTransforms);
}

bool KinBody::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    if (_veclinks.size() == 0) {
        return false;
    }

    bool bSuccess = false;
    if( _veclinks.size() == 1 ) {
        bSuccess = GetEnv()->GetPhysicsEngine()->SetLinkVelocity(_veclinks[0], linearvel, angularvel);
    }
    else {
        std::vector<std::pair<Vector,Vector> >& velocities = _vVelocitiesCache;
        velocities.resize(_veclinks.size());
        velocities.at(0).first = linearvel;
        velocities.at(0).second = angularvel;
        Vector vlinktrans = _veclinks.at(0)->GetTransform().trans;
        for(size_t ilink = 1; ilink < _veclinks.size(); ++ilink) {
            velocities[ilink].first = linearvel + angularvel.cross(_veclinks[ilink]->GetTransform().trans-vlinktrans);
            velocities[ilink].second = angularvel;
        }

        bSuccess = GetEnv()->GetPhysicsEngine()->SetLinkVelocities(shared_kinbody(),velocities);
    }

    _UpdateGrabbedBodies();
    return bSuccess;
}

void KinBody::SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, const Vector& linearvel, const Vector& angularvel, uint32_t checklimits)
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_OP_FORMAT((int)vDOFVelocities.size(), >=, GetDOF(), "env=%s, not enough values %d!=%d", GetEnv()->GetNameId()%vDOFVelocities.size()%GetDOF(),ORE_InvalidArguments);
    std::vector<std::pair<Vector,Vector> >& velocities = _vVelocitiesCache;
    velocities.resize(_veclinks.size());
    velocities.at(0).first = linearvel;
    velocities.at(0).second = angularvel;

    vector<dReal> vlower,vupper,vtempvalues, veval;
    if( checklimits != CLA_Nothing ) {
        GetDOFVelocityLimits(vlower,vupper);
    }

    // have to compute the velocities ahead of time since they are dependent on the link transformations
    std::vector< boost::array<dReal, 3> >& vPassiveJointVelocities = _vPassiveJointValuesCache;
    vPassiveJointVelocities.resize(_vPassiveJoints.size());
    for(size_t ijoint = 0; ijoint < vPassiveJointVelocities.size(); ++ijoint) {
        if( !_vPassiveJoints[ijoint]->IsMimic() ) {
            _vPassiveJoints[ijoint]->GetVelocities(vPassiveJointVelocities[ijoint]);
        }
    }

    std::vector<uint8_t>& vlinkscomputed = _vLinksVisitedCache;
    vlinkscomputed.resize(_veclinks.size());
    std::fill(vlinkscomputed.begin(), vlinkscomputed.end(), 0);
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
                                RAVELOG_DEBUG_FORMAT("env=%s, cannot evaluate partial velocity for mimic joint %s, perhaps equations don't exist", GetEnv()->GetNameId()%pjoint->GetName());
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
                        RAVELOG_WARN(str(boost::format("env=%s, dof %d velocity is not in limits %.15e<%.15e")%GetEnv()->GetNameId()%(dofindex+i)%pvalues[i]%vlower.at(dofindex+i)));
                    }
                    else if( checklimits == CLA_CheckLimitsThrow ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, dof %d velocity is not in limits %.15e<%.15e"), GetEnv()->GetId()%(dofindex+i)%pvalues[i]%vlower.at(dofindex+i), ORE_InvalidArguments);
                    }
                    dummyvalues[i] = vlower[dofindex+i];
                }
                else if( pvalues[i] > vupper.at(dofindex+i)+g_fEpsilonJointLimit ) {
                    if( checklimits == CLA_CheckLimits ) {
                        RAVELOG_WARN(str(boost::format("env=%s, dof %d velocity is not in limits %.15e>%.15e")%GetEnv()->GetNameId()%(dofindex+i)%pvalues[i]%vupper.at(dofindex+i)));
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
    OPENRAVE_ASSERT_OP_FORMAT(vDOFVelocities.size(),==,dofindices.size(),"env=%s, index sizes do not match %d!=%d", GetEnv()->GetNameId()%vDOFVelocities.size()%dofindices.size(), ORE_InvalidArguments);
    // have to recreate the correct vector
    std::vector<dReal> vfulldof(GetDOF());
    std::vector<int>::const_iterator it;
    for(size_t i = 0; i < dofindices.size(); ++i) {
        it = find(dofindices.begin(), dofindices.end(), i);
        if( it != dofindices.end() ) {
            vfulldof[i] = vDOFVelocities.at(static_cast<size_t>(it-dofindices.begin()));
        }
        else {
            const Joint& joint = _GetJointFromDOFIndex(i);
            vfulldof[i] = joint.GetVelocity(i-_vDOFIndices.at(i));
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
    FOREACHC(joint, _vDOFOrderedJoints) {
        int toadd = (*joint)->GetDOFIndex()-(int)doflastsetvalues.size();
        if( toadd > 0 ) {
            doflastsetvalues.insert(doflastsetvalues.end(),toadd,0);
        }
        else if( toadd < 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("dof indices mismatch joint %s, toadd=%d"), (*joint)->GetName()%toadd, ORE_InvalidState);
        }
        for(int i = 0; i < (*joint)->GetDOF(); ++i) {
            doflastsetvalues.push_back((*joint)->_doflastsetvalues[i]);
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

void KinBody::NotifyLinkEnabled(size_t linkIndex, bool bEnable)
{
    if (_vLinkEnableStatesMask.empty()) {
        RAVELOG_VERBOSE_FORMAT("env=%s, body=%s is not initialized probably this is still in initiailization phase. So skip notifying link enabled (index=%d, value=%d) and let _ComputeInternalInformation() take care of this later", GetEnv()->GetNameId()%GetName()%linkIndex%bEnable);
        return;
    }
    if (bEnable) {
        EnableLinkStateBit(_vLinkEnableStatesMask, linkIndex);
    }
    else {
        DisableLinkStateBit(_vLinkEnableStatesMask, linkIndex);
    }
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
    Transform tConvertToNewFrame = tBody*GetTransform().inverse();
    FOREACHC(itlink,_veclinks) {
        if( bEnabledOnlyLinks && !(*itlink)->IsEnabled() ) {
            continue;
        }

        Transform tlink = tConvertToNewFrame*(*itlink)->GetTransform();
        AABB ablink = (*itlink)->ComputeAABBFromTransform(tlink);
        if( ablink.extents.x == 0 && ablink.extents.y == 0 && ablink.extents.z == 0 ) {
            continue;
        }

        Vector vnmin = ablink.pos - ablink.extents;
        Vector vnmax = ablink.pos + ablink.extents;
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
        ab.pos = tBody.trans;
        ab.extents = Vector(0,0,0);
    }
    else {
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
    }
    return ab;
}

OrientedBox KinBody::ComputeOBBOnAxes(const Vector& quatWorld, bool bEnabledOnlyLinks) const
{
    // need to get the body's transform with respect to the new world
    Transform tinv; tinv.rot = quatInverse(quatWorld);
    Transform tBodyWithRespectToQuatWorld = tinv * GetTransform();
    AABB abInQuatWorld = ComputeAABBFromTransform(tBodyWithRespectToQuatWorld, bEnabledOnlyLinks);
    OrientedBox obb;
    obb.extents = abInQuatWorld.extents;
    obb.transform.rot = quatWorld;
    obb.transform.trans = quatRotate(quatWorld, abInQuatWorld.pos);
    return obb;
}

AABB KinBody::ComputeLocalAABB(bool bEnabledOnlyLinks) const
{
    return ComputeAABBFromTransform(Transform(), bEnabledOnlyLinks);
}

AABB KinBody::ComputeAABBForGeometryGroup(const std::string& geomgroupname, bool bEnabledOnlyLinks) const
{
    Vector vmin, vmax;
    bool binitialized=false;
    AABB ab;
    FOREACHC(itlink,_veclinks) {
        if( bEnabledOnlyLinks && !(*itlink)->IsEnabled() ) {
            continue;
        }
        ab = (*itlink)->ComputeAABBForGeometryGroup(geomgroupname);
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

AABB KinBody::ComputeAABBForGeometryGroupFromTransform(const std::string& geomgroupname, const Transform& tBody, bool bEnabledOnlyLinks) const
{
    Vector vmin, vmax;
    bool binitialized=false;
    AABB ablocal;
    Transform tConvertToNewFrame = tBody*GetTransform().inverse();
    FOREACHC(itlink,_veclinks) {
        if( bEnabledOnlyLinks && !(*itlink)->IsEnabled() ) {
            continue;
        }
        ablocal = (*itlink)->ComputeLocalAABBForGeometryGroup(geomgroupname);
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

AABB KinBody::ComputeLocalAABBForGeometryGroup(const std::string& geomgroupname, bool bEnabledOnlyLinks) const
{
    return ComputeAABBForGeometryGroupFromTransform(geomgroupname, Transform(), bEnabledOnlyLinks);
}

dReal KinBody::GetMass() const
{
    dReal fTotalMass = 0;
    for(const LinkPtr& plink : _veclinks) {
        fTotalMass += plink->GetMass();
    }
    return fTotalMass;
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
    OPENRAVE_ASSERT_OP_FORMAT(vbodies.size(), >=, _veclinks.size(), "env=%s, not enough links %d<%d", GetEnv()->GetNameId()%vbodies.size()%_veclinks.size(),ORE_InvalidArguments);
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
    OPENRAVE_ASSERT_OP_FORMAT(transforms.size(), >=, _veclinks.size(), "env=%s, not enough links %d<%d", GetEnv()->GetNameId()%transforms.size()%_veclinks.size(),ORE_InvalidArguments);
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
            _veclinks[ilink]->_Enable(bEnable);
            _nNonAdjacentLinkCache &= ~AO_Enabled;
            bchanged = true;
        }
    }
    if( bchanged ) {
        _PostprocessChangedParameters(Prop_LinkEnable);
    }
}

void KinBody::SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& bodyTransform, uint32_t checklimits)
{
    if( _veclinks.size() == 0 ) {
        return;
    }
    Transform baseLinkTransform = bodyTransform * _baseLinkInBodyTransform;
    Transform tbase = baseLinkTransform*_veclinks.at(0)->GetTransform().inverse();
    _veclinks.at(0)->SetTransform(baseLinkTransform);

    // apply the relative transformation to all links!! (needed for passive joints)
    for(size_t i = 1; i < _veclinks.size(); ++i) {
        _veclinks[i]->SetTransform(tbase*_veclinks[i]->GetTransform());
    }
    SetDOFValues(vJointValues,checklimits);
}

void KinBody::SetDOFValues(const std::vector<dReal>& vJointValues, uint32_t checklimits, const std::vector<int>& dofindices)
{
    if( vJointValues.size() == 0 ) {
        return;
    }
    SetDOFValues(&vJointValues[0], vJointValues.size(), checklimits, dofindices);
}

void KinBody::SetDOFValues(const dReal* pJointValues, int dof, uint32_t checklimits, const std::vector<int>& dofindices)
{
    CHECK_INTERNAL_COMPUTATION;
    if( dof == 0 || _veclinks.size() == 0) {
        return;
    }
    int expecteddof = dofindices.size() > 0 ? (int)dofindices.size() : GetDOF();
    OPENRAVE_ASSERT_OP_FORMAT((int)dof,>=,expecteddof, "env=%s, not enough values %d<%d", GetEnv()->GetNameId()%dof%GetDOF(),ORE_InvalidArguments);

    GetDOFValues(_vTempJoints);
    if( dofindices.size() > 0 ) {
        // user only set a certain number of indices, so have to fill the temporary array with the full set of values first
        // and then overwrite with the user set values
        for(size_t i = 0; i < dofindices.size(); ++i) {
            if( !std::isnan(pJointValues[i]) ) {
                _vTempJoints.at(dofindices[i]) = pJointValues[i];
            }
        }
    }
    else {
        for(size_t i = 0; i < _vTempJoints.size(); ++i) {
            if( !std::isnan(pJointValues[i]) ) {
                _vTempJoints[i] = pJointValues[i];
            }
        }
    }
    pJointValues = &_vTempJoints[0];

    if( checklimits != CLA_Nothing ) {
        dReal* ptempjoints = &_vTempJoints[0];

        // check the limits
        for(const JointPtr& pjoint : _vecjoints) {
            const Joint& joint = *pjoint;

            const dReal* p = pJointValues+joint.GetDOFIndex();
            if( joint.GetType() == JointSpherical ) {
                dReal fcurang = fmod(RaveSqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]),2*PI);
                dReal lowerlimit = joint.GetLowerLimit(0);
                dReal upperlimit = joint.GetUpperLimit(0);

                if( fcurang < lowerlimit ) {
                    if( fcurang < 1e-10 ) {
                        *ptempjoints++ = lowerlimit; *ptempjoints++ = 0; *ptempjoints++ = 0;
                    }
                    else {
                        dReal fmult = lowerlimit/fcurang;
                        *ptempjoints++ = p[0]*fmult; *ptempjoints++ = p[1]*fmult; *ptempjoints++ = p[2]*fmult;
                    }
                }
                else if( fcurang > upperlimit ) {
                    if( fcurang < 1e-10 ) {
                        *ptempjoints++ = upperlimit; *ptempjoints++ = 0; *ptempjoints++ = 0;
                    }
                    else {
                        dReal fmult = upperlimit/fcurang;
                        *ptempjoints++ = p[0]*fmult; *ptempjoints++ = p[1]*fmult; *ptempjoints++ = p[2]*fmult;
                    }
                }
                else {
                    *ptempjoints++ = p[0]; *ptempjoints++ = p[1]; *ptempjoints++ = p[2];
                }
            }
            else {
                for(int i = 0; i < joint.GetDOF(); ++i) {
                    if( joint.IsCircular(i) ) {
                        // don't normalize since user is expecting the values he sets are exactly returned via GetDOFValues
                        *ptempjoints++ = p[i]; //utils::NormalizeCircularAngle(p[i],(*it)->_vcircularlowerlimit[i],(*it)->_vcircularupperlimit[i]);
                    }
                    else {
                        dReal lowerlimit = joint.GetLowerLimit(0);
                        dReal upperlimit = joint.GetUpperLimit(0);

                        if( p[i] < lowerlimit ) {
                            if( p[i] < lowerlimit-g_fEpsilonEvalJointLimit ) {
                                if( checklimits == CLA_CheckLimits ) {
                                    RAVELOG_WARN(str(boost::format("env=%s, body '%s' joint '%s' dofindex %d value %.16e is smaller than the lower limit %.16e")%GetEnv()->GetNameId()%GetName()%joint.GetName()%(joint.GetDOFIndex()+i)%p[i]%lowerlimit));
                                }
                                else if( checklimits == CLA_CheckLimitsThrow ) {
                                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, body '%s' joint '%s' dofindex %d value %.16e is smaller than the lower limit %.16e"), GetEnv()->GetNameId()%GetName()%joint.GetName()%(joint.GetDOFIndex()+i)%p[i]%lowerlimit, ORE_InvalidArguments);
                                }
                            }
                            *ptempjoints++ = lowerlimit;
                        }
                        else if( p[i] > upperlimit ) {
                            if( p[i] > upperlimit+g_fEpsilonEvalJointLimit ) {
                                if( checklimits == CLA_CheckLimits ) {
                                    RAVELOG_WARN_FORMAT("env=%s, body '%s' joint '%s' dofindex %d value %.16e is greater than the upper limit %.16e", GetEnv()->GetNameId()%GetName()%joint.GetName()%(joint.GetDOFIndex()+i)%p[i]%upperlimit);
                                }
                                else if( checklimits == CLA_CheckLimitsThrow ) {
                                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, body '%s' joint '%s' dofindex %d value %.16e is greater than the upper limit %.16e"), GetEnv()->GetNameId()%GetName()%joint.GetName()%(joint.GetDOFIndex()+i)%p[i]%upperlimit, ORE_InvalidArguments);
                                }
                            }
                            *ptempjoints++ = upperlimit;
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

    if( !!_pCurrentKinematicsFunctions ) {
        if(_pCurrentKinematicsFunctions->SetLinkTransforms(pJointValues, _vLinkTransformPointers)) {
            // have to set _doflastsetvalues!
            for(const JointPtr& pjoint : _vecjoints) {
                Joint& joint = *pjoint;
                int dofindex = joint.GetDOFIndex();
                for(int iaxis = 0; iaxis < joint.GetDOF(); ++iaxis) {
                    joint._doflastsetvalues[iaxis] = pJointValues[dofindex+iaxis];
                }
            }
            _UpdateGrabbedBodies();
            _PostprocessChangedParameters(Prop_LinkTransforms);
            return;
        }
    }

    // have to compute the angles ahead of time since they are dependent on the link
    const int nActiveJoints = _vecjoints.size();
    const int nPassiveJoints = _vPassiveJoints.size();
    std::vector< boost::array<dReal, 3> >& vPassiveJointValues = _vPassiveJointValuesCache;
    vPassiveJointValues.resize(nPassiveJoints);
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
                            RAVELOG_WARN_FORMAT("env=%s, dummy joint out of lower limit! %e < %e", GetEnv()->GetNameId() % vlowerlimit.at(j) % jvals[j]);
                        }
                        jvals[j] = vlowerlimit.at(j);
                    }
                    else if( jvals[j] > vupperlimit.at(j) ) {
                        if( jvals[j] > vupperlimit.at(j) + 5e-4f ) {
                            RAVELOG_WARN_FORMAT("env=%s, dummy joint out of upper limit! %e > %e", GetEnv()->GetNameId() % vupperlimit.at(j) % jvals[j]);
                        }
                        jvals[j] = vupperlimit.at(j);
                    }
                }
            }
        }
    }

    std::vector<uint8_t>& vlinkscomputed = _vLinksVisitedCache;
    vlinkscomputed.resize(_veclinks.size());
    std::fill(vlinkscomputed.begin(), vlinkscomputed.end(), 0);
    vlinkscomputed[0] = 1;
    boost::array<dReal,3> dummyvalues; // dummy values for a joint
    std::vector<dReal>& vtempvalues = _vTempMimicValues;
    std::vector<dReal>& veval = _vTempMimicValues2;

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
                        RAVELOG_WARN_FORMAT("env=%d, failed to evaluate joint %s, fparser error %d", GetEnv()->GetId()%joint.GetName()%err);
                    }
                    else {
                        std::vector<dReal>& vevalcopy = _vTempMimicValues3;
                        vevalcopy = veval;
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
                            OPENRAVE_ASSERT_FORMAT(!veval.empty(), "env=%s, no valid values for joint %s", GetEnv()->GetNameId()%joint.GetName(),ORE_Assert);
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
            //RAVELOG_DEBUG_FORMAT("Joint %s's jointtype = %d has special bit", GetName() % jointtype);
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
    for(const LinkPtr& plink : _veclinks) {
        if ( plink->GetName() == linkname ) {
            return plink;
        }
    }
    return LinkPtr();
}

KinBody::LinkPtr KinBody::GetLink(const string_view linkname) const
{
    for(const LinkPtr& plink : _veclinks) {
        if ( plink->GetName() == linkname ) {
            return plink;
        }
    }
    return LinkPtr();
}

KinBody::LinkPtr KinBody::GetLink(const char* plinkname) const
{
    if( !!plinkname ) {
        for(const LinkPtr& plink : _veclinks) {
            if ( plink->GetName().compare(plinkname) == 0 ) {
                return plink;
            }
        }
    }
    return LinkPtr();
}

int KinBody::GetLinkIndex(const string_view linkname) const
{
    for(int ilink = 0; ilink < (int)_veclinks.size(); ++ilink) {
        if ( _veclinks[ilink]->GetName() == linkname ) {
            return ilink;
        }
    }
    return -1;
}

const std::vector<KinBody::JointPtr>& KinBody::GetDependencyOrderedJoints() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _vTopologicallySortedJoints;
}

const std::vector<KinBody::JointPtr>& KinBody::GetDependencyOrderedJointsAll() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _vTopologicallySortedJointsAll;
}

const std::vector< std::vector< std::pair<KinBody::LinkPtr, KinBody::JointPtr> > >& KinBody::GetClosedLoops() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _vClosedLoops;
}

bool KinBody::GetChain(int linkindex1, int linkindex2, std::vector<JointPtr>& vjoints) const
{
    CHECK_INTERNAL_COMPUTATION0;
    _EnsureAllPairsShortestPaths();
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
    _EnsureAllPairsShortestPaths();
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

void KinBody::ComputeJacobianTranslation(const int linkindex,
                                         const Vector& position,
                                         std::vector<dReal>& vjacobian,
                                         const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    _EnsureAllPairsShortestPaths();
    const int nlinks = _veclinks.size();
    const int nActiveJoints = _vecjoints.size();
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < nlinks, "body %s bad link index %d (num links %d)",
                           this->GetName() % linkindex % nlinks, ORE_InvalidArguments
                           );
    const size_t dofstride = dofindices.empty() ? this->GetDOF() : dofindices.size();
    vjacobian.resize(3 * dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(), vjacobian.end(), 0.0);

    std::vector<std::pair<int, dReal> > vDofindexDerivativePairs; ///< vector of (dof index, total derivative) pairs
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mTotalderivativepairValue; ///< map a joint pair (z, x) to the total derivative dz/dx

    Vector vColumn; ///< cache for a column of the linear velocity Jacobian
    const int offset = linkindex * nlinks;
    for(int curlink = 0;
        _vAllPairsShortestPaths[offset + curlink].first >= 0;     // parent link is still available
        curlink = _vAllPairsShortestPaths[offset + curlink].first // get index of parent link
        ) {
        const int jointindex = _vAllPairsShortestPaths[offset + curlink].second; ///< generalized joint index, which counts in [_vecjoints, _vPassiveJoints]
        if( jointindex < nActiveJoints ) {
            // active joint
            const JointPtr& pjoint = _vecjoints.at(jointindex);
            const int dofindex = pjoint->GetDOFIndex();
            const int ndof = pjoint->GetDOF();
            const int8_t affect = this->DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int idof = 0; idof < ndof; ++idof) {
                if( !affect ) {
                    continue; ///< not affect
                }

                const bool bPrismatic = pjoint->IsPrismatic(idof);
                const bool bRevolute = pjoint->IsRevolute(idof);
                if( !bPrismatic && !bRevolute ) {
                    RAVELOG_WARN("ComputeJacobianTranslation only supports revolute and prismatic joints, but not this joint type %d", pjoint->GetType());
                    continue;
                }

                // formula for active joint's linear velocity Jacobian
                if(bPrismatic) {
                    vColumn = pjoint->GetAxis(idof);
                }
                else {
                    vColumn =  pjoint->GetAxis(idof).cross(position - pjoint->GetAnchor());
                }

                int index = -1;
                if( !dofindices.empty() ) {
                    const std::vector<int>::const_iterator itindex = std::find(dofindices.begin(), dofindices.end(), dofindex + idof);
                    if( itindex == dofindices.end() ) {
                        continue;
                    }
                    index = itindex - dofindices.begin();
                }
                else {
                    index = dofindex + idof;
                }
                vjacobian[index                ] += vColumn.x;
                vjacobian[index + dofstride    ] += vColumn.y;
                vjacobian[index + dofstride * 2] += vColumn.z;
            }
        }
        else {
            // add in the contributions from the passive joint
            const JointPtr& pjoint = _vPassiveJoints.at(jointindex - nActiveJoints);
            const int ndof = pjoint->GetDOF();
            for(int idof = 0; idof < ndof; ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    const bool bPrismatic = pjoint->IsPrismatic(idof);
                    const bool bRevolute = pjoint->IsRevolute(idof);
                    if( !bPrismatic && !bRevolute ) {
                        RAVELOG_WARN("ComputeJacobianTranslation only supports revolute and prismatic joints, but not this joint type %d", pjoint->GetType());
                        continue;
                    }

                    // if this joint were active, then this is its column in the linear velocity Jacobian
                    if(bPrismatic) {
                        vColumn = pjoint->GetAxis(idof);
                    }
                    else {
                        vColumn =  pjoint->GetAxis(idof).cross(position - pjoint->GetAnchor());
                    }

                    // compute the partial derivatives of this mimic joint w.r.t all joints on which it directly/undirectly depends, by chain rule
                    // vDofindexDerivativePairs is a vector of (dof index, total derivative) pairs
                    pjoint->_ComputePartialVelocities(vDofindexDerivativePairs, idof, mTotalderivativepairValue);

                    for(const std::pair<int, dReal>& dofindexDerivativePair : vDofindexDerivativePairs) {
                        int index = -1;
                        const int dofindex = dofindexDerivativePair.first;
                        if( !dofindices.empty() ) {
                            const std::vector<int>::const_iterator itindex = std::find(dofindices.begin(), dofindices.end(), dofindex);
                            if( itindex == dofindices.end() ) {
                                continue;
                            }
                            index = itindex - dofindices.begin(); ///< index of an active joint
                        }
                        else {
                            index = dofindex;
                        }
                        OPENRAVE_ASSERT_OP_FORMAT(index, >=, 0, "index should be >= 0; now %d", index, ORE_InvalidArguments);
                        const dReal partialderiv = dofindexDerivativePair.second;
                        vjacobian[index                ] += vColumn.x * partialderiv;
                        vjacobian[index + dofstride    ] += vColumn.y * partialderiv;
                        vjacobian[index + dofstride * 2] += vColumn.z * partialderiv;
                    }
                }
            }
        }

    }
}

void KinBody::CalculateJacobian(const int linkindex,
                                const Vector& position,
                                std::vector<dReal>& jacobian) const {
    this->ComputeJacobianTranslation(linkindex, position, jacobian);
}

void KinBody::CalculateJacobian(const int linkindex,
                                const Vector& position,
                                boost::multi_array<dReal, 2>& mjacobian) const
{
    const size_t ndof = this->GetDOF();
    mjacobian.resize(boost::extents[3][ndof]);
    if( ndof == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    ComputeJacobianTranslation(linkindex, position, vjacobian);
    OPENRAVE_ASSERT_OP(vjacobian.size(), ==, 3*ndof);
    std::vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst, mjacobian) {
        std::copy(itsrc, itsrc + ndof, itdst->begin());
        itsrc += ndof;
    }
}

void KinBody::CalculateRotationJacobian(const int linkindex,
                                        const Vector& quat,
                                        std::vector<dReal>& vjacobian) const
{
    CHECK_INTERNAL_COMPUTATION;
    _EnsureAllPairsShortestPaths();
    const int nlinks = _veclinks.size();
    const int nActiveJoints = _vecjoints.size();
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < nlinks, "body %s bad link index %d (num links %d)",
                           this->GetName() % linkindex % nlinks, ORE_InvalidArguments
                           );
    const int dofstride = GetDOF();
    vjacobian.resize(4 * dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(), vjacobian.end(), 0.0);

    std::vector<std::pair<int, dReal> > vDofindexDerivativePairs; ///< vector of (dof index, total derivative) pairs
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mTotalderivativepairValue; ///< map a joint pair (z, x) to the total derivative dz/dx

    Vector vColumn; ///< cache for a column of the quaternion velocity Jacobian
    const int offset = linkindex * nlinks;

    for(int curlink = 0;
        _vAllPairsShortestPaths[offset+curlink].first >= 0;     // parent link is still available
        curlink = _vAllPairsShortestPaths[offset+curlink].first // get index of parent link
        ) {
        const int jointindex = _vAllPairsShortestPaths[offset+curlink].second;
        if( jointindex < nActiveJoints ) {
            // active joint
            const JointPtr& pjoint = _vecjoints.at(jointindex);
            const int dofindex = pjoint->GetDOFIndex();
            const int ndof = pjoint->GetDOF();
            const int8_t affect = DoesAffect(pjoint->GetJointIndex(), linkindex);
            for(int idof = 0; idof < ndof; ++idof) {
                if( !affect ) {
                    RAVELOG_WARN(str(boost::format("link %s should be affected by joint %s")% _veclinks.at(linkindex)->GetName() % pjoint->GetName()));
                    continue;
                }
                if( pjoint->IsPrismatic(idof) ) {
                    continue;
                }
                else if (!pjoint->IsRevolute(idof)) {
                    RAVELOG_WARN("CalculateRotationJacobian only supports revolute and prismatic joints, but not this joint type %d", pjoint->GetType());
                    continue;
                }
                vColumn = pjoint->GetAxis(idof); ///< joint axis of a revolute joint
                const size_t index = dofindex + idof;
                vjacobian[index                ] += 0.5 * (-quat.y * vColumn.x - quat.z * vColumn.y - quat.w * vColumn.z);
                vjacobian[index + dofstride    ] += 0.5 * ( quat.x * vColumn.x - quat.z * vColumn.z + quat.w * vColumn.y);
                vjacobian[index + dofstride * 2] += 0.5 * ( quat.x * vColumn.y + quat.y * vColumn.z - quat.w * vColumn.x);
                vjacobian[index + dofstride * 3] += 0.5 * ( quat.x * vColumn.z - quat.y * vColumn.y + quat.z * vColumn.x);
            }
        }
        else {
            // add in the contributions from the passive joint
            const JointPtr& pjoint = _vPassiveJoints.at(jointindex - nActiveJoints);
            const int ndof = pjoint->GetDOF();
            for(int idof = 0; idof < ndof; ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    if( pjoint->IsPrismatic(idof) ) {
                        continue;
                    }
                    else if (!pjoint->IsRevolute(idof)) {
                        RAVELOG_WARN("CalculateRotationJacobian only supports revolute and prismatic joints, but not this joint type %d", pjoint->GetType());
                        continue;
                    }

                    // if this revolute joint were active, then this is its column in the quaternion velocity Jacobian
                    vColumn = pjoint->GetAxis(idof);

                    // compute the partial derivatives of this mimic joint w.r.t all joints on which it directly/undirectly depends, by chain rule
                    // vDofindexDerivativePairs is a vector of (dof index, total derivative) pairs
                    pjoint->_ComputePartialVelocities(vDofindexDerivativePairs, idof, mTotalderivativepairValue);

                    for(const std::pair<int, dReal>& dofindexDerivativePair : vDofindexDerivativePairs) {
                        const int dofindex = dofindexDerivativePair.first;
                        if(dofindex + idof >= dofstride) {
                            RAVELOG_WARN_FORMAT("dofindex + idof = %d + %d >= %d = dofstride",
                                                dofindex % idof % dofstride
                                                );
                            continue;
                        }
                        OPENRAVE_ASSERT_OP_FORMAT(dofindex, >=, 0, "dofindex should be >= 0; now %d", dofindex, ORE_InvalidArguments);
                        const size_t index = dofindex + idof;
                        const dReal partialderiv = dofindexDerivativePair.second;
                        vjacobian[index                ] += 0.5 * partialderiv * (-quat.y * vColumn.x - quat.z * vColumn.y - quat.w * vColumn.z);
                        vjacobian[index + dofstride    ] += 0.5 * partialderiv * ( quat.x * vColumn.x - quat.z * vColumn.z + quat.w * vColumn.y);
                        vjacobian[index + dofstride * 2] += 0.5 * partialderiv * ( quat.x * vColumn.y + quat.y * vColumn.z - quat.w * vColumn.x);
                        vjacobian[index + dofstride * 3] += 0.5 * partialderiv * ( quat.x * vColumn.z - quat.y * vColumn.y + quat.z * vColumn.x);
                    }
                }
            }
        }
    }
}

void KinBody::CalculateRotationJacobian(const int linkindex,
                                        const Vector& quat,
                                        boost::multi_array<dReal, 2>& mjacobian) const
{
    const size_t ndof = this->GetDOF();
    mjacobian.resize(boost::extents[4][ndof]);
    if( ndof == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    CalculateRotationJacobian(linkindex, quat, vjacobian);
    OPENRAVE_ASSERT_OP(vjacobian.size(), ==, 4 * ndof);
    std::vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst, mjacobian) {
        std::copy(itsrc, itsrc+ndof, itdst->begin());
        itsrc += ndof;
    }
}

void KinBody::ComputeJacobianAxisAngle(const int linkindex,
                                       std::vector<dReal>& vjacobian,
                                       const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    _EnsureAllPairsShortestPaths();
    const int nlinks = _veclinks.size();
    const int nActiveJoints = _vecjoints.size();
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < nlinks, "body %s bad link index %d (num links %d)",
                           this->GetName() % linkindex % nlinks, ORE_InvalidArguments
                           );
    const size_t dofstride = dofindices.empty() ? this->GetDOF() : dofindices.size();
    vjacobian.resize(3 * dofstride);
    if( dofstride == 0 ) {
        return;
    }
    std::fill(vjacobian.begin(), vjacobian.end(), 0.0);

    std::vector<std::pair<int, dReal> > vDofindexDerivativePairs; ///< vector of (dof index, total derivative) pairs
    std::map< std::pair<Mimic::DOFFormat, int>, dReal > mTotalderivativepairValue; ///< map a joint pair (z, x) to the total derivative dz/dx

    Vector vColumn; ///< cache for a column of the angular velocity Jacobian
    const int offset = linkindex * nlinks;

    for(int curlink = 0;
        _vAllPairsShortestPaths[offset + curlink].first >= 0;     // parent link is still available
        curlink = _vAllPairsShortestPaths[offset + curlink].first // get index of parent link
        ) {
        const int jointindex = _vAllPairsShortestPaths[offset + curlink].second;
        if( jointindex < nActiveJoints ) {
            // active joint
            const JointPtr& pjoint = _vecjoints.at(jointindex);
            const int dofindex = pjoint->GetDOFIndex();
            const int ndof = pjoint->GetDOF();
            const int8_t affect = this->DoesAffect(pjoint->GetJointIndex(), linkindex);

            for(int dof = 0; dof < ndof; ++dof) {
                if( affect != 0 ) {
                    if( pjoint->IsPrismatic(dof) ) {
                        continue;
                    }
                    else if( !pjoint->IsRevolute(dof) ) {
                        RAVELOG_WARN("ComputeJacobianAxisAngle only supports revolute and prismatic joints, but not this joint type %d", pjoint->GetType());
                        continue;
                    }

                    // axis of an (active) revolute joint is its corresponding column in the angular velocity Jacobian
                    vColumn = pjoint->GetAxis(dof);
                    int index = -1;
                    if( !dofindices.empty() ) {
                        const std::vector<int>::const_iterator itindex = std::find(dofindices.begin(),dofindices.end(),dofindex+dof);
                        if( itindex == dofindices.end() ) {
                            continue;
                        }
                        index = itindex - dofindices.begin();
                    }
                    else {
                        index = dofindex + dof;
                    }
                    vjacobian[index                ] += vColumn.x;
                    vjacobian[index + dofstride    ] += vColumn.y;
                    vjacobian[index + dofstride * 2] += vColumn.z;
                }
            }
        }
        else {
            // add in the contributions from the passive joint
            const JointPtr& pjoint = _vPassiveJoints.at(jointindex - nActiveJoints);
            const int ndof = pjoint->GetDOF();
            for(int idof = 0; idof < ndof; ++idof) {
                if( pjoint->IsMimic(idof) ) {
                    if(pjoint->IsPrismatic(idof)) {
                        continue; // prismatic joint does not affect orientation of manip
                    }
                    else if( !pjoint->IsRevolute(idof) ) {
                        RAVELOG_WARN("ComputeJacobianAxisAngle only supports revolute and prismatic joints, but not this joint type %d", pjoint->GetType());
                        continue;
                    }

                    // if this revolute joint were active, then this is its column in the angular velocity Jacobian
                    vColumn = pjoint->GetAxis(idof);

                    // compute the partial derivatives of this mimic joint w.r.t all joints on which it directly/undirectly depends, by chain rule
                    // vDofindexDerivativePairs is a vector of (dof index, total derivative) pairs
                    pjoint->_ComputePartialVelocities(vDofindexDerivativePairs, idof, mTotalderivativepairValue);

                    for(const std::pair<int, dReal>& dofindexDerivativePair : vDofindexDerivativePairs) {
                        int index = -1;
                        const int dofindex = dofindexDerivativePair.first;
                        if( !dofindices.empty() ) {
                            const std::vector<int>::const_iterator itindex = std::find(dofindices.begin(), dofindices.end(), dofindex);
                            if( itindex == dofindices.end() ) {
                                continue;
                            }
                            index = itindex - dofindices.begin(); ///< index of an active joint
                        }
                        else {
                            index = dofindex;
                        }
                        OPENRAVE_ASSERT_OP_FORMAT(index, >=, 0, "index should be >= 0; now %d", index, ORE_InvalidArguments);
                        const dReal partialderiv = dofindexDerivativePair.second;
                        vjacobian[index                ] += vColumn.x * partialderiv;
                        vjacobian[index + dofstride    ] += vColumn.y * partialderiv;
                        vjacobian[index + dofstride * 2] += vColumn.z * partialderiv;
                    }
                }
            }
        }
    }
}

void KinBody::CalculateAngularVelocityJacobian(const int linkindex, std::vector<dReal>& jacobian) const {
    this->ComputeJacobianAxisAngle(linkindex, jacobian);
}

void KinBody::CalculateAngularVelocityJacobian(const int linkindex,
                                               boost::multi_array<dReal, 2>& mjacobian) const
{
    const size_t ndof = this->GetDOF();
    mjacobian.resize(boost::extents[3][ndof]);
    if( ndof == 0 ) {
        return;
    }
    std::vector<dReal> vjacobian;
    ComputeJacobianAxisAngle(linkindex, vjacobian);
    OPENRAVE_ASSERT_OP(vjacobian.size(), ==, 3 * ndof);
    std::vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst, mjacobian) {
        std::copy(itsrc, itsrc + ndof, itdst->begin());
        itsrc += ndof;
    }
}

void KinBody::ComputeHessianTranslation(int linkindex, const Vector& position, std::vector<dReal>& hessian, const std::vector<int>& dofindices) const
{
    CHECK_INTERNAL_COMPUTATION;
    _EnsureAllPairsShortestPaths();
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
                        pjoint->_ComputePartialVelocities(partialinfo.second, idof, mapcachedpartials);
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
    _EnsureAllPairsShortestPaths();
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
                        pjoint->_ComputePartialVelocities(partialinfo.second, idof, mapcachedpartials);
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

    std::vector<std::pair<int,dReal> > vDofindexDerivativePairs;
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

                }
                if (pActuatorInfo->rotor_inertia > 0.0) {
                    // converting inertia on motor side to load side requires multiplying by gear ratio squared because inertia unit is mass * distance^2
                    const dReal fInertiaOnLoadSide = pActuatorInfo->rotor_inertia * pActuatorInfo->gear_ratio * pActuatorInfo->gear_ratio;
                    fRotorAccelerationTorque += vDOFAccelerations.at(pjoint->GetDOFIndex()) * fInertiaOnLoadSide;
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

            pjoint->_ComputePartialVelocities(vDofindexDerivativePairs, 0, mapcachedpartials);
            for(const std::pair<int, dReal>& dofindexDerivativePair : vDofindexDerivativePairs) {
                doftorques.at(dofindexDerivativePair.first) += dofindexDerivativePair.second * faxistorque;
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
            pjoint->_ComputePartialVelocities(vpartials, 0, mapcachedpartials);
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

bool KinBody::GetDOFDynamicAccelerationJerkLimits(std::vector<dReal>& vDynamicAccelerationLimits, std::vector<dReal>& vDynamicJerkLimits,
                                                  const std::vector<dReal>& vDOFPositions, const std::vector<dReal>& vDOFVelocities) const
{
    return false;
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
    dofvelocities.resize(GetDOF(),0);
    FOREACHC(it, _vDOFOrderedJoints) {
        const Joint& joint = **it;
        int parentindex = 0;
        if( !!joint._attachedbodies[0] ) {
            parentindex = joint._attachedbodies[0]->GetIndex();
        }
        int childindex = joint._attachedbodies[1]->GetIndex();
        joint._GetVelocities(&dofvelocities[joint.GetDOFIndex()],vLinkVelocities.at(parentindex),vLinkVelocities.at(childindex));
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
    std::vector< boost::array<dReal,3> >& vPassiveJointVelocities = _vPassiveJointValuesCache;
    std::vector< boost::array<dReal,3> >& vPassiveJointAccelerations = _vPassiveJointAccelerationsCache;
    vPassiveJointVelocities.resize(_vPassiveJoints.size());
    vPassiveJointAccelerations.resize(_vPassiveJoints.size());
    for(size_t i = 0; i <_vPassiveJoints.size(); ++i) {
        if( vDOFAccelerations.size() > 0 ) {
            vPassiveJointAccelerations[i][0] = 0;
            vPassiveJointAccelerations[i][1] = 0;
            vPassiveJointAccelerations[i][2] = 0;
        }
        if( vDOFVelocities.size() > 0 ) {
            if( !_vPassiveJoints[i]->IsMimic() ) {
                _vPassiveJoints[i]->GetVelocities(vPassiveJointVelocities[i]);
            }
            else {
                vPassiveJointVelocities[i][0] = 0;
                vPassiveJointVelocities[i][1] = 0;
                vPassiveJointVelocities[i][2] = 0;
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

            // self collision checker initializes internal data structure at the time of grab, so need to do it here for newly set self collision checker.
            std::vector<KinBodyPtr> vGrabbed;
            GetGrabbed(vGrabbed);
            for (const KinBodyPtr& pgrabbed : vGrabbed) {
                _selfcollisionchecker->InitKinBody(pgrabbed);
            }
        }
    }
}

CollisionCheckerBasePtr KinBody::GetSelfCollisionChecker() const
{
    return _selfcollisionchecker;
}

void KinBody::_EnsureAllPairsShortestPaths() const
{
    // If the all pairs shortest paths have already been calculated, no need to do anything
    if (!_vAllPairsShortestPaths.empty()) {
        return;
    }

    // Preallocate to fit our NxN joint map
    _vAllPairsShortestPaths.resize(_veclinks.size() * _veclinks.size());

    // Default each entry to a pair of invalid joint indices
    for (std::pair<int16_t, int16_t>& pair : _vAllPairsShortestPaths) {
        pair.first = -1;
        pair.second = -1;
    }

    // All of our arrays are essentially 2d look up tables, so create a wrapper to generate an array index from a 2d point
    const size_t linksSize = _veclinks.size();
#define MAKE_INDEX(X, Y) ((X)*linksSize + (Y))

    // Create an NxN array of costs, where vcosts[MAKE_INDEX(i, j)] is the cost of joint i -> joint j
    // Initialize the costs to 2^30-1 rather than uint32_t max as a default 'infinite' value because we will later be adding costs together and need to make sure that value doesn't overflow
    vector<uint32_t> vcosts(_veclinks.size() * _veclinks.size(), 0x3fffffff);

    // Set the diagonal values (all paths from a link to itself) to zero
    for (size_t i = 0; i < _veclinks.size(); ++i) {
        vcosts[MAKE_INDEX(i, i)] = 0;
    }

    // Since not all links will be part of valid joints, we should only consider those valid links when building our cost map.
    // Otherwise, scenes that contain a large number of non-jointed links will incur significant overhead.
    // Note that we use an ordered set here - iterating the links in-order ensures that we mimic the actual connection order of the links.
    // Iterating in non-deterministic order may produce unexpected paths where a 'future' link traversal shares the same cost as a traversal that is closer to the robot base.
    std::set<int> usedLinkIndices;

    for (const JointPtr& joint : _vecjoints) {
        // If this joint doesn't have two links to calculate a cost between, skip it
        if (!joint->GetFirstAttached() || !joint->GetSecondAttached()) {
            continue;
        }

        // The links are directly connected to this joint, so we know they're the shortest path and can assign them a cost of 1 hop
        const int jointIndex = joint->GetJointIndex();
        const int firstLinkIndex = joint->GetFirstAttached()->GetIndex();
        const int secondLinkIndex = joint->GetSecondAttached()->GetIndex();

        // Mark these links as used
        usedLinkIndices.emplace(firstLinkIndex);
        usedLinkIndices.emplace(secondLinkIndex);

        // First link
        {
            int index = MAKE_INDEX(firstLinkIndex, secondLinkIndex);
            _vAllPairsShortestPaths[index] = std::pair<int16_t, int16_t>(firstLinkIndex, jointIndex);
            vcosts[index] = 1;
        }

        // Second link
        {
            int index = MAKE_INDEX(secondLinkIndex, firstLinkIndex);
            _vAllPairsShortestPaths[index] = std::pair<int16_t, int16_t>(secondLinkIndex, jointIndex);
            vcosts[index] = 1;
        }
    }

    // Since we are splaying across two different vectors here, we need to add the size of the base joint vector to our joint index for the passive joints
    int jointindex = (int)_vecjoints.size();
    for (const JointPtr& passiveJoint : _vPassiveJoints) {
        // If this joint doesn't have two links, ignore it
        if (!passiveJoint->GetFirstAttached() || !passiveJoint->GetSecondAttached()) {
            continue;
        }

        // The links are directly connected to this joint, so we know they're the shortest path and can assign them a cost of 1 hop
        const int firstLinkIndex = passiveJoint->GetFirstAttached()->GetIndex();
        const int secondLinkIndex = passiveJoint->GetSecondAttached()->GetIndex();

        // Mark these links as used
        usedLinkIndices.emplace(firstLinkIndex);
        usedLinkIndices.emplace(secondLinkIndex);

        // First link
        {
            int index = MAKE_INDEX(firstLinkIndex, secondLinkIndex);
            _vAllPairsShortestPaths[index] = std::pair<int16_t, int16_t>(firstLinkIndex, jointindex);
            vcosts[index] = 1;
        }

        // Second link
        {
            int index = MAKE_INDEX(secondLinkIndex, firstLinkIndex);
            _vAllPairsShortestPaths[index] = std::pair<int16_t, int16_t>(secondLinkIndex, jointindex);
            vcosts[index] = 1;
        }

        // Manually track joint index
        ++jointindex;
    }

    // Now that we have the base costs set for all joints, iterate the links we know to be jointed and calculate the total cost between each pair
    for (size_t k : usedLinkIndices) {
        for (size_t i : usedLinkIndices) {
            // Skip comparisons of a link with itself
            if (i == k) {
                continue;
            }

            for (size_t j : usedLinkIndices) {
                // Skip comparisons of a link with itself
                if ((j == i) || (j == k)) {
                    continue;
                }

                // Calculate the total cost of going from j -> i via k (aka the cost of j -> k + cost k -> i)
                uint32_t kcost = vcosts[MAKE_INDEX(j, k)] + vcosts[MAKE_INDEX(k, i)];

                // If that's cheaper than the current cost of going from j -> i, pick it as the new shortest path
                if (vcosts[MAKE_INDEX(j, i)] > kcost) {
                    // Floor the cost for this movement
                    vcosts[MAKE_INDEX(j, i)] = kcost;

                    // Update the path with the new route
                    _vAllPairsShortestPaths[MAKE_INDEX(j, i)] = _vAllPairsShortestPaths[MAKE_INDEX(k, i)];
                }
            }
        }
    }
#undef MAKE_INDEX
}

void KinBody::_ComputeInternalInformation()
{
    uint64_t starttime = utils::GetMicroTime();
    _nHierarchyComputed = 1;

    _vLinkTransformPointers.clear();
    if( !!_pCurrentKinematicsFunctions ) {
        RAVELOG_DEBUG_FORMAT("env=%d, resetting custom kinematics functions for body %s", GetEnv()->GetId()%GetName());
        _pCurrentKinematicsFunctions.reset();
    }

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
        FOREACH(itvjoint,_vecjoints) {
            (*itvjoint)->jointindex = jointindex++;
            (*itvjoint)->dofindex = dofindex;
            (*itvjoint)->_info._bIsActive = true;
            dofindex += (*itvjoint)->GetDOF();
        }
        FOREACH(passive,_vPassiveJoints) {
            (*passive)->jointindex = -1;
            (*passive)->dofindex = -1;
            (*passive)->_info._bIsActive = false;
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
    FOREACH(index,vorder) {
        _vDOFOrderedJoints.push_back(_vecjoints.at(*index));
    }

    try {
        // initialize all the mimic equations
        for(int bPassiveJoints = 0; bPassiveJoints < 2; ++bPassiveJoints) { // simulate false/true
            const std::vector<JointPtr>& vjoints = bPassiveJoints ? _vPassiveJoints : _vecjoints;
            for(const JointPtr& pjoint : vjoints) {
                const int ndof = pjoint->GetDOF();
                const boost::array<MimicPtr, 3>& vmimic = pjoint->_vmimic;
                for(int idof = 0; idof < ndof; ++idof) {
                    const MimicPtr& pmimic = vmimic[idof];
                    if( !!pmimic ) {
                        const std::string poseq = pmimic->_equations[0];
                        const std::string veleq = pmimic->_equations[1];
                        const std::string acceleq = pmimic->_equations[2]; // have to copy since memory can become invalidated
                        pjoint->SetMimicEquations(idof, poseq, veleq, acceleq);
                    }
                }
            }
        }

        // fill Mimic::_vmimicdofs, check that there are no circular dependencies between the mimic joints
        const int nActiveJoints = _vecjoints.size();
        std::map<Mimic::DOFFormat, MimicPtr> mapmimic; ///< collects if thisdofformat.jointaxis depends on a mimic joint
        for(int bPassiveJoints = 0; bPassiveJoints < 2; ++bPassiveJoints) { // simulate false/true
            const std::vector<JointPtr>& vjoints = bPassiveJoints ? _vPassiveJoints : _vecjoints;
            const int njoints = vjoints.size();
            for(int ijoint = 0; ijoint < njoints; ++ijoint) {
                const JointPtr& pjoint = vjoints[ijoint];

                Mimic::DOFFormat thisdofformat; ///< construct for pjoint
                if( bPassiveJoints ) {
                    thisdofformat.dofindex   = -1; ///< mimic dofindex = -1, ...
                    thisdofformat.jointindex = ijoint + nActiveJoints; ///< but has a generalized joint index
                }
                else {
                    thisdofformat.dofindex   = pjoint->GetDOFIndex();  ///< >= 0
                    thisdofformat.jointindex = pjoint->GetJointIndex(); ///< in [0, nActiveJoints)
                }

                const int ndof = pjoint->GetDOF();
                const boost::array<MimicPtr, 3>& vmimic = pjoint->_vmimic;
                for(int idof = 0; idof < ndof; ++idof) {
                    const MimicPtr& pmimic = vmimic[idof]; // enumerate
                    thisdofformat.axis = idof;
                    if( !!pmimic ) {
                        // only add if pjoint depends on mimic joints
                        // TGN: Can an active joint depend on mimic joints??? If not, why need vjoints = _vecjoints?
                        for(const Mimic::DOFFormat& dofformat : pmimic->_vdofformat) {
                            const JointPtr pjointDepended = dofformat.GetJoint(*this);
                            if( pjointDepended->IsMimic(dofformat.axis) ) {
                                mapmimic[thisdofformat] = pmimic; ///< pjoint depends on pjointDepended
                                RAVELOG_VERBOSE_FORMAT("mimic joint %s depends on mimic joint %s", pjoint->GetName() % pjointDepended->GetName());
                                break;
                            }
                        }
                    }
                }
            }
        }

        bool bchanged = true;
        while(bchanged) {
            bchanged = false;
            for(const std::pair<const Mimic::DOFFormat, MimicPtr>& keyvalue : mapmimic) {
                const Mimic::DOFFormat& thisdofformat = keyvalue.first;
                const MimicPtr& pmimic = keyvalue.second;
                std::vector<Mimic::DOFHierarchy>& vmimicdofs = pmimic->_vmimicdofs; ///< to collect information of active joints on which pmimic depends on
                const std::vector<Mimic::DOFFormat>& vdofformat = pmimic->_vdofformat; ///<  collected information of all joints on which pmimic depends on

                const JointPtr pjoint = thisdofformat.GetJoint(*this); ///< pjoint depends on all [dofformat.GetJoint(*this) for dofformat in vdofformat]
                const int ndofformat = vdofformat.size();
                for(int idofformat = 0; idofformat < ndofformat; ++idofformat) {
                    const Mimic::DOFFormat& dofformat = vdofformat[idofformat];
                    const JointPtr pjointDepended = dofformat.GetJoint(*this);
                    if( !mapmimic.count(dofformat) ) {
                        continue; // this means pjointDepended depends on active joints only
                    }

                    const MimicPtr& pmimicDepended = mapmimic.at(dofformat); // dofformat.jointindex depends on pmimicDepended
                    const std::vector<Mimic::DOFHierarchy>&   vmimicdofsDepended = pmimicDepended->_vmimicdofs;
                    const std::vector<Mimic::DOFFormat>& vmimicdofformatDepended = pmimicDepended->_vdofformat;

                    for(const Mimic::DOFHierarchy& mimicdofDepended : vmimicdofsDepended) {
                        if( vmimicdofformatDepended[mimicdofDepended.dofformatindex] == thisdofformat ) {
                            throw OPENRAVE_EXCEPTION_FORMAT(_("joint %s depends on a mimic joint %s that also depends on %s; circular dependency!!!"),
                                                            pjoint->GetName() % pjointDepended->GetName() % pjoint->GetName(), ORE_Failed);
                        }

                        // TGN: Since Mimic::_vmimicdofs only contains active joints (c.f. KinBody::Joint::SetMimicEquations),
                        // when computing partial/total derivatives by chain rule, we shall use Mimic::_vdofformat
                        Mimic::DOFHierarchy h;
                        h.dofformatindex = idofformat; ///< index in vdofformat
                        h.dofindex = mimicdofDepended.dofindex; // >= 0, dofindex of active joint
                        if( find(vmimicdofs.begin(), vmimicdofs.end(), h) == vmimicdofs.end() ) {
                            vmimicdofs.push_back(h);
                            bchanged = true;
                        }
                    }
                }
            }
        }
    }
    catch(const std::exception& ex) {
        RAVELOG_ERROR(str(boost::format("failed to set mimic equations on kinematics body %s: %s\n")%GetName()%ex.what()));
        for(int bPassiveJoints = 0; bPassiveJoints < 2; ++bPassiveJoints) { // simulate false/true
            const std::vector<JointPtr>& vjoints = bPassiveJoints ? _vPassiveJoints : _vecjoints;
            for(const JointPtr& pjoint : vjoints) {
                const int ndof = pjoint->GetDOF();
                for(int idof = 0; idof < ndof; ++idof) {
                    pjoint->_vmimic[idof].reset();
                }
            }
        }
    }

    _vTopologicallySortedJoints.resize(0);
    _vTopologicallySortedJointsAll.resize(0);
    _vTopologicallySortedJointIndicesAll.resize(0);
    _vJointsAffectingLinks.resize(_vecjoints.size()*_veclinks.size());

    // Invalidate our all pairs shortest path table by default
    _vAllPairsShortestPaths.clear();

    // Use the APAC algorithm to initialize the kinematics hierarchy: _vTopologicallySortedJoints, _vJointsAffectingLinks, Link::_vParentLinks.
    // SIMOES, Ricardo. APAC: An exact algorithm for retrieving cycles and paths in all kinds of graphs. Tékhne, Dec. 2009, no.12, p.39-55. ISSN 1654-9911.
    if ((_veclinks.size() > 0) && (_vecjoints.size() > 0)) {
        // Some of our calculations later depend on the link:link shortest path table, so ensure it's precalculated once here
        _EnsureAllPairsShortestPaths();

        std::vector< std::vector<int> > vlinkadjacency(_veclinks.size());
        // joints with only one attachment are attached to a static link, which is attached to link 0
        for( const JointPtr& joint :_vecjoints) {
            vlinkadjacency.at(joint->GetFirstAttached()->GetIndex()).push_back(joint->GetSecondAttached()->GetIndex());
            vlinkadjacency.at(joint->GetSecondAttached()->GetIndex()).push_back(joint->GetFirstAttached()->GetIndex());
        }
        for( const JointPtr& passive : _vPassiveJoints) {
            vlinkadjacency.at(passive->GetFirstAttached()->GetIndex()).push_back(passive->GetSecondAttached()->GetIndex());
            vlinkadjacency.at(passive->GetSecondAttached()->GetIndex()).push_back(passive->GetFirstAttached()->GetIndex());
        }
        for( std::vector<int> &adj : vlinkadjacency) {
            sort(adj.begin(), adj.end());
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
                    FOREACHC(itparentlink0,plink0->_vParentLinks) {
                        int jointindex = _vAllPairsShortestPaths[plink0->GetIndex()*_veclinks.size()+*itparentlink0].second;
                        size_t pos = find(_vTopologicallySortedJointIndicesAll.begin(),_vTopologicallySortedJointIndicesAll.end(),jointindex) - _vTopologicallySortedJointIndicesAll.begin();
                        link0pos = min(link0pos,pos);
                    }
                    FOREACHC(itparentlink1,plink1->_vParentLinks) {
                        int jointindex = _vAllPairsShortestPaths[plink1->GetIndex()*_veclinks.size()+*itparentlink1].second;
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
        _vJointsAffectingLinks.assign( _vJointsAffectingLinks.size(), 0);

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
                                    const Joint& joint2 = _GetJointFromDOFIndex(itmimicdof->dofindex);
                                    _vJointsAffectingLinks[joint2.GetJointIndex()*_veclinks.size()+i] = joint2.GetHierarchyParentLink()->GetIndex() == i ? -1 : 1;
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

    const size_t numLinks = GetLinks().size();

    // create the adjacency list
    {
        _ResizeVectorFor2DTable(_vForcedAdjacentLinks, numLinks);
        //std::fill(_vForcedAdjacentLinks.begin(), _vForcedAdjacentLinks.end(), 0);

        for (const LinkPtr& plink : _veclinks) {
            for (const std::string& forceAdjacentLinkFromInfo : plink->_info._vForcedAdjacentLinks) {
                LinkPtr pLinkForceAdjacentLinkFromInfo = GetLink(forceAdjacentLinkFromInfo);
                if (!pLinkForceAdjacentLinkFromInfo) {
                    RAVELOG_WARN_FORMAT("env=%d, could not find link \"%s\" in _vForcedAdjacentLinks of body \"%s:%s\"", GetEnv()->GetId()%forceAdjacentLinkFromInfo%GetName()%plink->GetName());
                    continue;
                }

                const size_t forcedAdjacentLinkIndex = pLinkForceAdjacentLinkFromInfo->GetIndex();
                _vForcedAdjacentLinks.at(_GetIndex1d(plink->_index, forcedAdjacentLinkIndex)) = 1;
            }
        }

        _vAdjacentLinks = _vForcedAdjacentLinks;
        // probably unnecessary, but just in case
        _ResizeVectorFor2DTable(_vAdjacentLinks, numLinks);

        // make no-geometry links adjacent to all other links
        for (const LinkPtr& plink0 : _veclinks) {
            const Link& link0 = *plink0;
            if (link0.GetGeometries().size() == 0) {
                const int ind0 = link0.GetIndex();
                for (const LinkPtr& link1 : _veclinks) {
                    const int ind1 = link1->GetIndex();
                    if (ind0 != ind1) {
                        _vAdjacentLinks.at(_GetIndex1d(ind0, ind1)) = 1;
                    }
                }
            }
        }

        if( _bMakeJoinedLinksAdjacent ) {
            for (const JointPtr& pjoint : _vecjoints) {
                const Joint& joint = *pjoint;
                int ind0 = joint._attachedbodies[0]->GetIndex();
                int ind1 = joint._attachedbodies[1]->GetIndex();
                _vAdjacentLinks.at(_GetIndex1d(ind0, ind1)) = 1;
            }

            for (const JointPtr& pjoint : _vPassiveJoints) {
                const Joint& joint = *pjoint;
                int ind0 = joint._attachedbodies[0]->GetIndex();
                int ind1 = joint._attachedbodies[1]->GetIndex();
                _vAdjacentLinks.at(_GetIndex1d(ind0, ind1)) = 1;
            }

            // If a pair of links has exactly one non-static joint in the middle, then make the pair adjacent
            vector<JointPtr> vjoints;
            if (!_vecjoints.empty()) {
                for (int ind0 = 0; ind0 < (int)_veclinks.size() - 1; ++ind0) {
                    for (int ind1 = ind0 + 1; ind1 < (int)_veclinks.size(); ++ind1) {
                        // If these links are already adjacent, skip computation of their connecting chain
                        const size_t adjacencyIndex = _GetIndex1d(ind0, ind1);
                        if (_vAdjacentLinks[adjacencyIndex]) {
                            continue;
                        }

                        GetChain(ind0, ind1, vjoints);
                        size_t numstatic = 0;
                        FOREACH(it, vjoints) {
                            numstatic += (*it)->IsStatic();
                        }
                        if (numstatic + 1 >= vjoints.size()) {
                            _vAdjacentLinks[adjacencyIndex] = 1;
                        }
                    }
                }
            }
        }
        _ResetInternalCollisionCache();
    }

    _vLinkTransformPointers.resize(_veclinks.size());
    for(int ilink = 0; ilink < (int)_veclinks.size(); ++ilink) {
        _vLinkTransformPointers[ilink] = &_veclinks[ilink]->_info._t;
    }

    InitializeLinkStateBitMasks(_vLinkEnableStatesMask, _veclinks.size());
    for (const LinkPtr& plink : _veclinks) {
        const Link& link = *plink;
        if (link.IsEnabled()) {
            EnableLinkStateBit(_vLinkEnableStatesMask, link.GetIndex());
        }
    }

    _nHierarchyComputed = 2;
    // because of mimic joints, need to call SetDOFValues at least once, also use this to check for links that are off
    {
        vector<Transform> vprevtrans, vnewtrans;
        vector<dReal> vprevdoflastsetvalues, vnewdoflastsetvalues;
        GetLinkTransformations(vprevtrans, vprevdoflastsetvalues);
        vector<dReal> vcurrentvalues;
        // unfortunately if SetDOFValues is overloaded by the robot, it could call the robot's _UpdateGrabbedBodies, which is a problem during environment cloning since the grabbed bodies might not be initialized. Therefore, call KinBody::SetDOFValues
        GetDOFValues(vcurrentvalues);
        MapGrabbedByEnvironmentIndex grabbedBodiesByBodyName; grabbedBodiesByBodyName.swap(_grabbedBodiesByEnvironmentIndex); // swap to get rid of grabbed bodies
        KinBody::SetDOFValues(vcurrentvalues,CLA_CheckLimits, std::vector<int>());
        grabbedBodiesByBodyName.swap(_grabbedBodiesByEnvironmentIndex); // swap back
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
        _PrintDOFValuesForInitialLinkTransformations(*this, vnewdoflastsetvalues, __FUNCTION__);
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

    if( !!_pKinematicsGenerator ) {
        _pCurrentKinematicsFunctions = _pKinematicsGenerator->GenerateKinematicsFunctions(*this);
        if( !!_pCurrentKinematicsFunctions ) {
            RAVELOG_DEBUG_FORMAT("env=%s, setting custom kinematics functions for body %s", GetEnv()->GetNameId()%GetName());
        }
        else {
            RAVELOG_DEBUG_FORMAT("env=%s, failed to set custom kinematics functions for body %s", GetEnv()->GetNameId()%GetName());
        }
    }
    else {
        _pCurrentKinematicsFunctions.reset();
    }

    RAVELOG_VERBOSE_FORMAT("env=%d, initialized %s in %f[s]", GetEnv()->GetId()%GetName()%(1e-6*(utils::GetMicroTime()-starttime)));
}

void KinBody::_DeinitializeInternalInformation()
{
    _nHierarchyComputed = 0; // should reset to inform other elements that kinematics information might not be accurate

    // Clear all-pairs shortest path table to force recomputation
    _vAllPairsShortestPaths.clear();
}

void KinBody::GetDirectlyAttachedBodies(std::vector<KinBodyPtr>& vBodies) const
{
    // Clear the output and reserve enough space for all potentially attached bodies
    vBodies.clear();
    vBodies.reserve(_listAttachedBodies.size());

    // Filter down the attached bodies to only the set of bodies that are still live
    for (const KinBodyWeakPtr& pbody : _listAttachedBodies) {
        KinBodyPtr attachedBody = pbody.lock();
        if (!!attachedBody) {
            vBodies.emplace_back(std::move(attachedBody));
        }
    }
}

bool KinBody::IsAttached(const KinBody &body) const
{
    // handle obvious cases without doing expensive operations
    if(this == &body ) {
        return true;
    }
    else if (_listAttachedBodies.empty()) {
        return false;
    }

    std::vector<int8_t>& vAttachedVisited = _vAttachedVisitedCache;
    vAttachedVisited.resize(GetEnv()->GetMaxEnvironmentBodyIndex() + 1);
    std::fill(vAttachedVisited.begin(), vAttachedVisited.end(), 0);
    return _IsAttached(body.GetEnvironmentBodyIndex(), vAttachedVisited);
}

void KinBody::GetAttached(std::set<KinBodyPtr>& setAttached) const
{
    setAttached.clear();
    // by spec "including this body.", include this body
    setAttached.insert(boost::const_pointer_cast<KinBody>(shared_kinbody_const()));

    // early exist, probably this is the case most of the time
    if (_listAttachedBodies.empty()) {
        return;
    }

    const EnvironmentBase& env = *GetEnv();
    std::vector<int8_t>& vAttachedVisited = _vAttachedVisitedCache;
    vAttachedVisited.resize(env.GetMaxEnvironmentBodyIndex() + 1);
    std::fill(vAttachedVisited.begin(), vAttachedVisited.end(), 0);

    vAttachedVisited.at(GetEnvironmentBodyIndex()) = -1;

    int numAttached = _GetNumAttached(vAttachedVisited);
    if( numAttached ) {
        for (int bodyIndex = 1; bodyIndex < (int)vAttachedVisited.size(); ++bodyIndex) {
            if (vAttachedVisited[bodyIndex] == 1) {
                KinBodyPtr pbody = env.GetBodyFromEnvironmentBodyIndex(bodyIndex);
                if( !!pbody ) {
                    setAttached.insert(pbody);
                    if (--numAttached == 0) {
                        // already filled vAttached with contents of vAttachedVisited
                        return;
                    }
                }
            }
        }
    }
}

void KinBody::GetAttached(std::set<KinBodyConstPtr>& setAttached) const
{
    setAttached.clear();
    // by spec "including this body.", include this body
    setAttached.insert(boost::const_pointer_cast<KinBody>(shared_kinbody_const()));

    // early exist, probably this is the case most of the time
    if (_listAttachedBodies.empty()) {
        return;
    }

    const EnvironmentBase& env = *GetEnv();
    std::vector<int8_t>& vAttachedVisited = _vAttachedVisitedCache;
    vAttachedVisited.resize(env.GetMaxEnvironmentBodyIndex() + 1);
    std::fill(vAttachedVisited.begin(), vAttachedVisited.end(), 0);

    vAttachedVisited.at(GetEnvironmentBodyIndex()) = -1;

    int numAttached = _GetNumAttached(vAttachedVisited);
    if( numAttached ) {
        for (int bodyIndex = 1; bodyIndex < (int)vAttachedVisited.size(); ++bodyIndex) {
            if (vAttachedVisited[bodyIndex] == 1) {
                KinBodyPtr pbody = env.GetBodyFromEnvironmentBodyIndex(bodyIndex);
                if( !!pbody ) {
                    setAttached.insert(pbody);
                    if (--numAttached == 0) {
                        // already filled vAttached with contents of vAttachedVisited
                        return;
                    }
                }
            }
        }
    }
}

void KinBody::GetAttached(std::vector<KinBodyPtr>& vAttached) const
{
    vAttached.clear();

    // early exist, probably this is the case most of the time
    if (_listAttachedBodies.empty()) {
        vAttached.push_back(boost::const_pointer_cast<KinBody>(shared_kinbody_const()));
        return;
    }

    const EnvironmentBase& env = *GetEnv();
    std::vector<int8_t>& vAttachedVisited = _vAttachedVisitedCache;
    vAttachedVisited.resize(env.GetMaxEnvironmentBodyIndex() + 1);
    std::fill(vAttachedVisited.begin(), vAttachedVisited.end(), 0);

    vAttachedVisited.at(GetEnvironmentBodyIndex()) = 1;
    int numAttached = 1 + _GetNumAttached(vAttachedVisited); // +1 for self

    vAttached.reserve(numAttached);
    for (int bodyIndex = 1; bodyIndex < (int)vAttachedVisited.size(); ++bodyIndex) {
        // 0 means not attached, -1 means it was already in original vAttached so skip
        if (vAttachedVisited[bodyIndex] == 1) {
            vAttached.push_back(env.GetBodyFromEnvironmentBodyIndex(bodyIndex));
            if (--numAttached == 0) {
                // already filled vAttached with contents of vAttachedVisited
                return;
            }
        }
    }
}

void KinBody::GetAttached(std::vector<KinBodyConstPtr>& vAttached) const
{
    vAttached.clear();

    // early exist, probably this is the case most of the time
    if (_listAttachedBodies.empty()) {
        vAttached.push_back(boost::const_pointer_cast<KinBody>(shared_kinbody_const()));
        return;
    }

    const EnvironmentBase& env = *GetEnv();
    std::vector<int8_t>& vAttachedVisited = _vAttachedVisitedCache;
    vAttachedVisited.resize(env.GetMaxEnvironmentBodyIndex() + 1);
    std::fill(vAttachedVisited.begin(), vAttachedVisited.end(), 0);

    vAttachedVisited.at(GetEnvironmentBodyIndex()) = 1;
    int numAttached = 1 + _GetNumAttached(vAttachedVisited); // +1 for self

    vAttached.reserve(numAttached);
    for (int bodyIndex = 1; bodyIndex < (int)vAttachedVisited.size(); ++bodyIndex) {
        // 0 means not attached, -1 means it was already in original vAttached so skip
        if (vAttachedVisited[bodyIndex] == 1) {
            vAttached.push_back(env.GetBodyFromEnvironmentBodyIndex(bodyIndex));
            if (--numAttached == 0) {
                // already filled vAttached with contents of vAttachedVisited
                return;
            }
        }
    }
}

void KinBody::GetAttachedEnvironmentBodyIndices(std::vector<int>& vAttached) const
{
    vAttached.clear();

    // early exist, probably this is the case most of the time
    if (_listAttachedBodies.empty()) {
        vAttached.push_back(GetEnvironmentBodyIndex());
        return;
    }

    const EnvironmentBase& env = *GetEnv();
    std::vector<int8_t>& vAttachedVisited = _vAttachedVisitedCache;
    vAttachedVisited.resize(env.GetMaxEnvironmentBodyIndex() + 1);
    std::fill(vAttachedVisited.begin(), vAttachedVisited.end(), 0);

    vAttachedVisited.at(GetEnvironmentBodyIndex()) = 1;
    int numAttached = 1 + _GetNumAttached(vAttachedVisited); // +1 for self

    vAttached.reserve(numAttached);
    for (int bodyIndex = 1; bodyIndex < (int)vAttachedVisited.size(); ++bodyIndex) {
        if (vAttachedVisited[bodyIndex] == 1) {
            vAttached.push_back(bodyIndex);
            if (--numAttached == 0) {
                // already filled vAttached with contents of vAttachedVisited
                return;
            }
        }
    }
}

bool KinBody::_IsAttached(int otherBodyid, std::vector<int8_t>& vAttachedVisited) const
{
    for (const KinBodyWeakPtr& pbody : _listAttachedBodies) {
        KinBodyConstPtr pattached = pbody.lock();
        if (!pattached) {
            continue;
        }
        const KinBody& attached = *pattached;
        if (otherBodyid == attached._environmentBodyIndex) {
            vAttachedVisited.at(attached._environmentBodyIndex) = 1; // attached, and visited
            return true;
        }
        else if (vAttachedVisited.at(attached._environmentBodyIndex) != 0) {
            // already checked, but need to continue to check other entries in _listAttachedBodies
            continue;
        }
        else {
            vAttachedVisited.at(attached._environmentBodyIndex) = -1; // not attached, but visitied
        }

        // if attached._listAttachedBodies has only one element, that element is same as attached as attachement relationship is by-directional, so not worth checking
        if (attached._listAttachedBodies.size() > 1 && attached._IsAttached(otherBodyid, vAttachedVisited)) {
            return true;
        }
    }

    return false;
}


int KinBody::_GetNumAttached(std::vector<int8_t>& vAttachedVisited) const
{
    int numFound = 0;
    for (const KinBodyWeakPtr& pbody : _listAttachedBodies) {
        KinBodyConstPtr pattachedBody = pbody.lock();
        if (!pattachedBody) {
            continue;
        }
        const KinBody& attachedBody = *pattachedBody;
        const int bodyIndex = attachedBody.GetEnvironmentBodyIndex();
        if (vAttachedVisited.at(bodyIndex) != 0) {
            // already checked, so skip
            continue;
        }
        vAttachedVisited.at(bodyIndex) = 1;
        numFound++;
        // if attached._listAttachedBodies has only one element, that element is same as attached as attachement relationship is by-directional, so not worth checking
        if (attachedBody._listAttachedBodies.size() > 1) {
            numFound += attachedBody._GetNumAttached(vAttachedVisited);
        }
    }
    return numFound;
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
    for (const LinkPtr& plink : _veclinks) {
        Link& link = *plink;
        if( link._info._bIsEnabled != bEnable ) {
            link._info._bIsEnabled = bEnable;
            _nNonAdjacentLinkCache &= ~AO_Enabled;
            bchanged = true;
        }
    }

    if (bEnable) {
        EnableAllLinkStateBitMasks(_vLinkEnableStatesMask, _veclinks.size());
    }
    else {
        DisableAllLinkStateBitMasks(_vLinkEnableStatesMask);
    }

    if( bchanged ) {
        _PostprocessChangedParameters(Prop_LinkEnable);
    }
}

bool KinBody::IsEnabled() const
{
    for (uint64_t mask : _vLinkEnableStatesMask) {
        if (mask > 0) {
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
    OPENRAVE_ASSERT_FORMAT(dofindex >= 0 && dofindex < GetDOF(), "body %s dofindex %d invalid (num dofs %d)", GetName()%dofindex%GetDOF(), ORE_InvalidArguments);
    OPENRAVE_ASSERT_FORMAT(linkindex >= 0 && linkindex < (int)_veclinks.size(), "body %s linkindex %d invalid (num links %d)", GetName()%linkindex%_veclinks.size(), ORE_InvalidArguments);
    int jointindex = _vDOFIndices.at(dofindex);
    return _vJointsAffectingLinks.at(jointindex*_veclinks.size()+linkindex);
}

void KinBody::SetNonCollidingConfiguration()
{
    _ResetInternalCollisionCache();
    vector<dReal> vdoflastsetvalues;
    GetLinkTransformations(_vInitialLinkTransformations, vdoflastsetvalues);
    _PrintDOFValuesForInitialLinkTransformations(*this, vdoflastsetvalues, __FUNCTION__);
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

void KinBody::_PrintNonAdjacentLinks(const boost::array<std::vector<int>, 4>& vNonAdjacentLinks, const size_t nonAdjacentMask, const std::string& envNameId, const std::string& bodyName)
{
    std::stringstream ssLinks;
    const std::vector<int>& vSelectedNonAdjacentLinks = vNonAdjacentLinks[nonAdjacentMask];
    for(size_t iLinks = 0; iLinks < vSelectedNonAdjacentLinks.size(); ++iLinks) {
        const int value = vSelectedNonAdjacentLinks[iLinks];
        if( iLinks > 0 ) {
            ssLinks << ",";
        }
        ssLinks << "(" << (value & 0xffff) << "," << (value>>16) << ")";
    }
    RAVELOG_INFO_FORMAT("env=%s, body '%s' computes the cache for GetNonAdjacentLinks(%d). linkPairs=[%s]", envNameId%bodyName%nonAdjacentMask%ssLinks.str());
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
        CollisionOptionsStateSaver colsaver(collisionchecker, CO_IgnoreCallbacks); // have to reset the collision options
        for(size_t i = 0; i < _veclinks.size(); ++i) {
            boost::static_pointer_cast<Link>(_veclinks[i])->_info._t = _vInitialLinkTransformations.at(i);
        }
        _nUpdateStampId++; // because transforms were modified
        _vNonAdjacentLinks[0].resize(0);

        for(size_t ind0 = 0; ind0 < _veclinks.size(); ++ind0) {
            for(size_t ind1 = ind0+1; ind1 < _veclinks.size(); ++ind1) {
                const bool bAdjacent = AreAdjacentLinks(ind0, ind1);
                if(!bAdjacent && !collisionchecker->CheckCollision(LinkConstPtr(_veclinks[ind0]), LinkConstPtr(_veclinks[ind1])) ) {
                    _vNonAdjacentLinks[0].push_back(ind0|(ind1<<16));
                }
            }
        }
        std::sort(_vNonAdjacentLinks[0].begin(), _vNonAdjacentLinks[0].end(), CompareNonAdjacentFarthest);
        _nUpdateStampId++; // because transforms were modified
        _nNonAdjacentLinkCache = 0;
        KinBody::_PrintNonAdjacentLinks(_vNonAdjacentLinks, 0, GetEnv()->GetNameId(), GetName());
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

bool KinBody::AreAdjacentLinks(int linkindex0, int linkindex1) const
{
    CHECK_INTERNAL_COMPUTATION;
    const size_t index = _GetIndex1d(linkindex0, linkindex1);
    if (index < _vAdjacentLinks.size()) {
        return _vAdjacentLinks.at(index);
    }
    RAVELOG_WARN_FORMAT("env=%d, body %s has %d links (size of _vAdjacentLinks is %d). Cannot compute adjacent for index %d and %d (1d index is %d)",
                        GetEnv()->GetId()%GetName()%GetLinks().size()%_vAdjacentLinks.size()%linkindex0%linkindex1%index);
    return false;
}

void KinBody::SetAdjacentLinksCombinations(const std::vector<int>& linkIndices)
{
    for (size_t idx0 = 0; idx0 < linkIndices.size(); idx0++) {
        const int linkIndex0 = linkIndices[idx0];
        for (size_t idx1 = idx0 + 1; idx1 < linkIndices.size(); idx1++) {
            const int linkIndex1 = linkIndices[idx1];
            OPENRAVE_ASSERT_OP(linkIndex0,!=,linkIndex1);
            _SetAdjacentLinksInternal(linkIndex0, linkIndex1);
        }
    }
    _ResetInternalCollisionCache();
}

void KinBody::SetAdjacentLinksPairs(const std::vector<std::pair<int, int> >& linkIndices)
{
    for ( const std::pair<int, int>& link01 : linkIndices) {
        OPENRAVE_ASSERT_OP(link01.first, !=, link01.second);
        _SetAdjacentLinksInternal(link01.first, link01.second);
    }
    _ResetInternalCollisionCache();
}

void KinBody::SetAdjacentLinks(int linkindex0, int linkindex1)
{
    OPENRAVE_ASSERT_OP(linkindex0,!=,linkindex1);
    _SetAdjacentLinksInternal(linkindex0, linkindex1);

    _ResetInternalCollisionCache();
}

void KinBody::_SetAdjacentLinksInternal(int linkindex0, int linkindex1)
{
    const int numLinks = GetLinks().size();
    BOOST_ASSERT(linkindex0 < numLinks);
    BOOST_ASSERT(linkindex1 < numLinks);

    const size_t index = _GetIndex1d(linkindex0, linkindex1);

    _ResizeVectorFor2DTable(_vAdjacentLinks, numLinks);
    _vAdjacentLinks.at(index) = 1;

    _ResizeVectorFor2DTable(_vForcedAdjacentLinks, numLinks);
    _vForcedAdjacentLinks.at(index) = 1;
}

void KinBody::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    InterfaceBase::Clone(preference,cloningoptions);
    KinBodyConstPtr r = RaveInterfaceConstCast<KinBody>(preference);

    _pKinematicsGenerator.reset();
    _pCurrentKinematicsFunctions.reset();
    _name = r->_name;
    _referenceUri = r->_referenceUri;
    _nHierarchyComputed = r->_nHierarchyComputed;
    _bMakeJoinedLinksAdjacent = r->_bMakeJoinedLinksAdjacent;
    __hashKinematicsGeometryDynamics = r->__hashKinematicsGeometryDynamics;
    _vTempJoints = r->_vTempJoints;

    _vLinkTransformPointers.clear(); _vLinkTransformPointers.reserve(r->_veclinks.size());
    _veclinks.clear(); _veclinks.reserve(r->_veclinks.size());
    for (const LinkPtr& origLinkPtr : r->_veclinks) {
        LinkPtr pnewlink(new Link(shared_kinbody()));
        Link& newlink = *pnewlink;
        // TODO should create a Link::Clone method
        newlink = *origLinkPtr; // be careful of copying pointers
        newlink._parent = shared_kinbody();

        {
            // have to copy all the geometries too!
            std::vector<Link::GeometryPtr> vnewgeometries(newlink._vGeometries.size());
            for(size_t igeom = 0; igeom < vnewgeometries.size(); ++igeom) {
                vnewgeometries[igeom].reset(new Link::Geometry(pnewlink, newlink._vGeometries[igeom]->_info));
            }
            newlink._vGeometries = vnewgeometries;
        }
        {
            // deep copy extra geometries as well, otherwise changing value of map in original map affects value of cloned map
            std::map< std::string, std::vector<GeometryInfoPtr> > newMapExtraGeometries;
            for (const std::pair<const std::string, std::vector<GeometryInfoPtr> >& keyValue : newlink._info._mapExtraGeometries) {
                std::vector<GeometryInfoPtr> newvalues;
                newvalues.reserve(keyValue.second.size());
                for (const GeometryInfoPtr& geomInfoPtr : keyValue.second) {
                    newvalues.push_back(GeometryInfoPtr(new GeometryInfo(*geomInfoPtr)));
                }
                newMapExtraGeometries[keyValue.first] = newvalues;
            }
            newlink._info._mapExtraGeometries = newMapExtraGeometries;
        }

        _veclinks.push_back(pnewlink);
        _vLinkTransformPointers.push_back(&newlink._info._t);
    }
    _vLinkEnableStatesMask = r->_vLinkEnableStatesMask;

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

    _vAdjacentLinks = r->_vAdjacentLinks;
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

    // cannot copy the velocities since it requires the physics engine to be initialized with this kinbody, which might not happen before the clone..?
//    std::vector<std::pair<Vector,Vector> > velocities;
//    r->GetLinkVelocities(velocities);
//    SetLinkVelocities(velocities);

    // do not force-reset the callbacks!! since the ChangeCallbackData destructors will crash
    //_listRegisteredCallbacks.clear();

    // cache
    _ResetInternalCollisionCache();

    // clone the grabbed bodies, note that this can fail if the new cloned environment hasn't added the bodies yet (check out Environment::Clone)
    _listAttachedBodies.clear(); // will be set in the environment
    _grabbedBodiesByEnvironmentIndex.clear();
    if ((cloningoptions & Clone_IgnoreGrabbedBodies) != Clone_IgnoreGrabbedBodies) {
        for (const MapGrabbedByEnvironmentIndex::value_type& otherGrabPair : r->_grabbedBodiesByEnvironmentIndex) {
            const GrabbedPtr& pgrabbedref = otherGrabPair.second;
            if( !pgrabbedref ) {
                RAVELOG_WARN_FORMAT("env=%s, have uninitialized GrabbedConstPtr in _vGrabbedBodies", GetEnv()->GetNameId());
                continue;
            }

            KinBodyPtr pbodyref = pgrabbedref->_pGrabbedBody.lock();
            KinBodyPtr pgrabbedbody;
            if( !!pbodyref ) {
                //pgrabbedbody = GetEnv()->GetBodyFromEnvironmentBodyIndex(pbodyref->GetEnvironmentBodyIndex());
                pgrabbedbody = GetEnv()->GetKinBody(pbodyref->GetName());
                if( !pgrabbedbody ) {
                    if( cloningoptions & Clone_PassOnMissingBodyReferences ) {
                        continue;
                    }
                    else {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("When cloning body '%s' from env=%s, could not find grabbed object '%s' in env=%s"), GetName()%r->GetEnv()->GetNameId()%pbodyref->GetName()%GetEnv()->GetNameId(), ORE_InvalidState);
                    }
                }
                //BOOST_ASSERT(pgrabbedbody->GetName() == pbodyref->GetName());

                GrabbedPtr pgrabbed(new Grabbed(pgrabbedbody,_veclinks.at(KinBody::LinkPtr(pgrabbedref->_pGrabbingLink)->GetIndex())));
                pgrabbed->_tRelative = pgrabbedref->_tRelative;
                pgrabbed->_setGrabberLinkIndicesToIgnore = pgrabbedref->_setGrabberLinkIndicesToIgnore; // can do this since link indices are the same
                CopyRapidJsonDoc(pgrabbedref->_rGrabbedUserData, pgrabbed->_rGrabbedUserData);
                if( pgrabbedref->IsListNonCollidingLinksValid() ) {
                    FOREACHC(itLinkRef, pgrabbedref->_listNonCollidingLinksWhenGrabbed) {
                        if( (*itLinkRef)->GetParent() == r ) {
                            pgrabbed->_listNonCollidingLinksWhenGrabbed.push_back(_veclinks.at((*itLinkRef)->GetIndex()));
                        }
                        else {
                            KinBodyPtr pOtherGrabbedBody = GetEnv()->GetKinBody((*itLinkRef)->GetParent()->GetName());
                            if( !!pOtherGrabbedBody ) {
                                KinBody::LinkPtr plink = pOtherGrabbedBody->GetLink((*itLinkRef)->GetName());
                                if( !!plink ) {
                                    pgrabbed->_listNonCollidingLinksWhenGrabbed.push_back(plink);
                                }
                                else {
                                    RAVELOG_WARN_FORMAT("env=%s, When cloning body '%s' from env=%s, could not find non-colliding link %s in body %s.", GetEnv()->GetNameId()%GetName()%r->GetEnv()->GetNameId()%(*itLinkRef)->GetName()%(*itLinkRef)->GetParent()->GetName());
                                }
                            }
                            else {
                                RAVELOG_WARN_FORMAT("env=%s, When cloning body '%s' from env=%s, could not find body %s for non-colliding link %s.", GetEnv()->GetNameId()%GetName()%r->GetEnv()->GetNameId()%(*itLinkRef)->GetParent()->GetName()%(*itLinkRef)->GetName());
                            }
                        }
                    }
                    pgrabbed->SetLinkNonCollidingIsValid(true);
                }

                try {
                    // if an exception happens in _AttachBody, have to remove from _vGrabbedBodies
                    _AttachBody(pgrabbedbody);
                }
                catch (...) {
                    RAVELOG_ERROR_FORMAT("env=%s, failed in attach body", GetEnv()->GetNameId());
                    throw;
                }

                BOOST_ASSERT(pgrabbedbody->GetEnvironmentBodyIndex() > 0);
                _grabbedBodiesByEnvironmentIndex[pgrabbedbody->GetEnvironmentBodyIndex()] = std::move(pgrabbed);
            }
        } // end for pgrabbedref
    } // end if not Clone_IgnoreGrabbedBodies

    // Clone self-collision checker
    _selfcollisionchecker.reset();
    if( !!r->_selfcollisionchecker ) {
        _selfcollisionchecker = RaveCreateCollisionChecker(GetEnv(), r->_selfcollisionchecker->GetXMLId());
        _selfcollisionchecker->SetCollisionOptions(r->_selfcollisionchecker->GetCollisionOptions());
        _selfcollisionchecker->SetGeometryGroup(r->_selfcollisionchecker->GetGeometryGroup());
        if( GetEnvironmentBodyIndex() != 0 ) {
            // This body has been added to the environment already so can call InitKinBody.
            _selfcollisionchecker->InitKinBody(shared_kinbody());
        }
        else {
            // InitKinBody will be called when the body is added to the environment.
        }
        // have to also call InitKinBody to grabbed bodies.
        for (const MapGrabbedByEnvironmentIndex::value_type& grabPair : _grabbedBodiesByEnvironmentIndex) {
            KinBodyPtr pGrabbedBody = grabPair.second->_pGrabbedBody.lock();
            _selfcollisionchecker->InitKinBody(pGrabbedBody);
        }
    }

    // can copy the generator, but not the functions! use SetKinematicsGenerator
    SetKinematicsGenerator(r->_pKinematicsGenerator);

    _lastModifiedAtUS = r->_lastModifiedAtUS;
    _revisionId = r->_revisionId;

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
        __hashKinematicsGeometryDynamics.resize(0);
    }

    if( (parameters&Prop_LinkEnable) == Prop_LinkEnable ) {
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
    if( __hashKinematicsGeometryDynamics.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        // should add dynamics since that affects a lot how part is treated.
        serialize(ss,SO_Kinematics|SO_Geometry|SO_Dynamics);
        __hashKinematicsGeometryDynamics = utils::GetMD5HashString(ss.str());
    }
    return __hashKinematicsGeometryDynamics;
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
    std::unique_lock<boost::shared_mutex> lock(GetInterfaceMutex());

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

void KinBody::_SetForcedAdjacentLinks(int linkindex0, int linkindex1)
{
    const int numLinks = GetLinks().size();
    BOOST_ASSERT(linkindex0 < numLinks);
    BOOST_ASSERT(linkindex1 < numLinks);
    const size_t index = _GetIndex1d(linkindex0, linkindex1);

    _ResizeVectorFor2DTable(_vForcedAdjacentLinks, numLinks);
    _vForcedAdjacentLinks.at(index) = 1;
}

void KinBody::_InitAndAddLink(LinkPtr plink)
{
    CHECK_NO_INTERNAL_COMPUTATION;
    LinkInfo& info = plink->_info;

    // check to make sure there are no repeating names in already added links
    FOREACH(itlink, _veclinks) {
        if( (*itlink)->GetName() == info._name ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("link '%s' is declared more than once in body '%s', uri is '%s'"), info._name%GetName()%GetURI(), ORE_InvalidArguments);
        }
    }

    plink->_index = static_cast<int>(_veclinks.size());
    _InitLinkFromInfo(plink, info);

    _veclinks.push_back(plink);
    _vLinkTransformPointers.clear();
    __hashKinematicsGeometryDynamics.resize(0);
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
    FOREACH(it, info._mReadableInterfaces) {
        pjoint->SetReadableInterface(it->first, it->second);
    }
    std::vector<Vector> vaxes(pjoint->GetDOF());
    std::copy(info._vaxes.begin(),info._vaxes.begin()+vaxes.size(), vaxes.begin());
    pjoint->_ComputeJointInternalInformation(plink0, plink1, info._vanchor, vaxes, info._vcurrentvalues);
    if( info._bIsActive ) {
        _vecjoints.push_back(pjoint);
    }
    else {
        _vPassiveJoints.push_back(pjoint);
    }
    __hashKinematicsGeometryDynamics.resize(0);
}

void KinBody::ExtractInfo(KinBodyInfo& info, ExtractInfoOptions options)
{
    info._modifiedFields = 0;
    info._id = _id;
    info._uri = GetURI();
    info._name = _name;
    info._referenceUri = _referenceUri;
    info._interfaceType = GetXMLId();
    info._isPartial = false; // extracting everything

    info._dofValues.clear();

    if( !(options & EIO_SkipDOFValues) ) {
        CHECK_INTERNAL_COMPUTATION; // the GetDOFValues requires that internal information is initialized
        std::vector<dReal> vDOFValues;
        GetDOFValues(vDOFValues);
        for (size_t idof = 0; idof < vDOFValues.size(); ++idof) {
            const Joint& joint = _GetJointFromDOFIndex(idof);
            int jointAxis = idof - joint.GetDOFIndex();
            info._dofValues.emplace_back(std::make_pair(joint.GetName(), jointAxis), vDOFValues[idof]);
        }
    }

    info._vGrabbedInfos.clear();
    GetGrabbedInfo(info._vGrabbedInfos);

    info._transform = GetTransform();

    // in order for link transform comparision to make sense
    KinBody::KinBodyStateSaver stateSaver(shared_kinbody(), Save_LinkTransformation);
    boost::shared_ptr<void> restoreTransformEvenWhenBodyIsNotAddedToEnv = nullptr;
    if( (options & EIO_SkipDOFValues) && GetEnvironmentBodyIndex() == 0 ) {
        RAVELOG_WARN("resetting DOF to zeros");
        restoreTransformEvenWhenBodyIsNotAddedToEnv = boost::shared_ptr<void>((void*) 0, std::bind(&KinBody::SetTransform, this, std::cref(info._transform)));
    }

    SetTransform(Transform()); // so that base link is at exactly _baseLinkInBodyTransform
    vector<dReal> vZeros(GetDOF(), 0);
    SetDOFValues(vZeros, KinBody::CLA_Nothing);

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

    if( !!_prAssociatedFileEntries ) {
        if( !info._prAssociatedFileEntries ) {
            info._prAssociatedFileEntries.reset(new rapidjson::Document());
        }
        else {
            *info._prAssociatedFileEntries = rapidjson::Document();
        }
        info._prAssociatedFileEntries->CopyFrom(*_prAssociatedFileEntries, info._prAssociatedFileEntries->GetAllocator());
    }
}

UpdateFromInfoResult KinBody::UpdateFromKinBodyInfo(const KinBodyInfo& info)
{
    UpdateFromInfoResult updateFromInfoResult = UFIR_NoChange;
    if(_id != info._id) {
        if( _id.empty() ) {
            RAVELOG_DEBUG_FORMAT("env=%s, body '%s' assigning empty id to '%s'", GetEnv()->GetNameId()%GetName()%info._id);
            SetId(info._id);
        }
        else if( info._id.empty() ) {
            RAVELOG_INFO_FORMAT("env=%d, body '%s' do not update id '%s' since update has empty id", GetEnv()->GetId()%GetName()%GetId());
        }
        else {
            RAVELOG_INFO_FORMAT("env=%d, body %s update info ids do not match this '%s' != update '%s'. current links=%d, new links=%d", GetEnv()->GetId()%GetName()%_id%info._id%_veclinks.size()%info._vLinkInfos.size());
            SetId(info._id);
        }
        updateFromInfoResult = UFIR_Success;
    }

    // need to avoid checking links and joints belonging to connected bodies
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

    // build vectors of links and joints that we will deal with
    std::vector<KinBody::LinkPtr> vLinks; vLinks.reserve(_veclinks.size());
    std::vector<KinBody::JointPtr> vJoints; vJoints.reserve(_vecjoints.size() + _vPassiveJoints.size());
    for (size_t iLink = 0; iLink < _veclinks.size(); ++iLink) {
        if (!isConnectedLink[iLink]) {
            vLinks.push_back(_veclinks[iLink]);
        }
    }
    for(size_t iJoint = 0; iJoint < _vecjoints.size(); iJoint++) {
        if (!isConnectedJoint[iJoint]) {
            vJoints.push_back(_vecjoints[iJoint]);
        }
    }
    for(size_t iPassiveJoint = 0; iPassiveJoint < _vPassiveJoints.size(); iPassiveJoint++) {
        if (!isConnectedPassiveJoint[iPassiveJoint]) {
            vJoints.push_back(_vPassiveJoints[iPassiveJoint]);
        }
    }

    {
        // in order for link transform comparision to make sense, have to change the kinbody to the identify.
        // First check if any of the link infos have modified transforms
        KinBody::KinBodyStateSaverPtr stateSaver;
        FOREACHC(itLinkInfo, info._vLinkInfos) {
            // if any link has its transform field set, we need to set zero configuration before comparison
            if( (*itLinkInfo)->IsModifiedField(KinBody::LinkInfo::LIF_Transform) ) {
                stateSaver.reset(new KinBody::KinBodyStateSaver(shared_kinbody(), Save_LinkTransformation));
                SetTransform(Transform());
                vector<dReal> vZeros(GetDOF(), 0);
                SetDOFValues(vZeros, KinBody::CLA_Nothing);
                break;
            }
        }

        // links
        if (!UpdateChildrenFromInfo(info._vLinkInfos, vLinks, updateFromInfoResult)) {
            return updateFromInfoResult;
        }
    }

    // update the base link transform after all links have been updated
    if( info._vLinkInfos.empty() ) {
        _baseLinkInBodyTransform = _invBaseLinkInBodyTransform = Transform();
    }
    else {
        _baseLinkInBodyTransform = info._vLinkInfos.front()->GetTransform();
        _invBaseLinkInBodyTransform = _baseLinkInBodyTransform.inverse();
    }

    // joints
    if (!UpdateChildrenFromInfo(info._vJointInfos, vJoints, updateFromInfoResult)) {
        return updateFromInfoResult;
    }

    // name
    if (GetName() != info._name) {
        OPENRAVE_ASSERT_OP(info._name.size(), >, 0);
        SetName(info._name);
        updateFromInfoResult = UFIR_Success;
        RAVELOG_VERBOSE_FORMAT("env=%s, body '%s' updated due to name change", GetEnv()->GetNameId()%info._name);
    }

    if( info.IsModifiedField(KinBodyInfo::KBIF_URI) && GetURI() != info._uri ) {
        __struri = info._uri;
        updateFromInfoResult = UFIR_Success;
        RAVELOG_VERBOSE_FORMAT("env=%s, body '%s' updated uri to '%s'", GetEnv()->GetNameId()%info._name%info._uri);
    }

    if( info.IsModifiedField(KinBodyInfo::KBIF_ReferenceURI) &&_referenceUri != info._referenceUri ) {
        _referenceUri = info._referenceUri;
        updateFromInfoResult = UFIR_Success;
        RAVELOG_VERBOSE_FORMAT("env=%s, body '%s' updated referenceUri to '%s'", GetEnv()->GetNameId()%info._name%info._referenceUri);
    }

    // transform
    if( info.IsModifiedField(KinBodyInfo::KBIF_Transform) ) {
        if( GetTransform().CompareTransform(info._transform, g_fEpsilon) ) {
            SetTransform(info._transform);
            updateFromInfoResult = UFIR_Success;
            RAVELOG_VERBOSE_FORMAT("body %s updated due to transform change", _id);
        }
    }

    // don't change the dof values here since body might not be added!
    if( info.IsModifiedField(KinBodyInfo::KBIF_DOFValues) && _nHierarchyComputed == 2 ) {
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
            int dofIndex = joint->GetDOFIndex()+it->first.second;
            if (RaveFabs(dofValues.at(dofIndex) - it->second) > g_fEpsilon) {
                dofValues[dofIndex] = it->second;
                bDOFChanged = true;
                //RAVELOG_VERBOSE_FORMAT("body %s dof %d value changed", _id%dofIndex);
            }
        }
        if (bDOFChanged) {
            SetDOFValues(dofValues);
            updateFromInfoResult = UFIR_Success;
            RAVELOG_VERBOSE_FORMAT("body %s updated due to dof values change", _id);
        }
    }

    if( UpdateReadableInterfaces(info._mReadableInterfaces) ) {
        updateFromInfoResult = UFIR_Success;
        RAVELOG_VERBOSE_FORMAT("body %s updated due to readable interface change", _id);
    }

    if( !!info._prAssociatedFileEntries ) {
        if( !_prAssociatedFileEntries ) {
            _prAssociatedFileEntries.reset(new rapidjson::Document());
            _prAssociatedFileEntries->CopyFrom(*info._prAssociatedFileEntries, _prAssociatedFileEntries->GetAllocator());
        }
        else if( *_prAssociatedFileEntries != *info._prAssociatedFileEntries ) {
            *_prAssociatedFileEntries = rapidjson::Document();
            _prAssociatedFileEntries->CopyFrom(*info._prAssociatedFileEntries, _prAssociatedFileEntries->GetAllocator());
        }
    }

    return updateFromInfoResult;
}

void KinBody::SetKinematicsGenerator(KinematicsGeneratorPtr pGenerator)
{
    if( _pKinematicsGenerator == pGenerator ) {
        // same pointer, so do nothing
        return;
    }
    _pKinematicsGenerator = pGenerator;
    if( !!_pKinematicsGenerator ) {
        try {
            _pCurrentKinematicsFunctions = _pKinematicsGenerator->GenerateKinematicsFunctions(*this);
        }
        catch(std::exception& ex) {
            RAVELOG_WARN_FORMAT("env=%s, failed to generate the kinematics functions: %s", GetEnv()->GetNameId()%ex.what());
            throw;
        }
        if( !!_pCurrentKinematicsFunctions ) {
            RAVELOG_DEBUG_FORMAT("env=%s, setting custom kinematics functions for body %s", GetEnv()->GetNameId()%GetName());
        }
        else {
            RAVELOG_DEBUG_FORMAT("env=%s, failed to set custom kinematics functions for body %s", GetEnv()->GetNameId()%GetName());
        }
    }
    else {
        if( !!_pCurrentKinematicsFunctions ) {
            RAVELOG_DEBUG_FORMAT("env=%d, resetting custom kinematics functions for body %s", GetEnv()->GetId()%GetName());
            _pCurrentKinematicsFunctions.reset();
        }
    }
}

} // end namespace OpenRAVE
