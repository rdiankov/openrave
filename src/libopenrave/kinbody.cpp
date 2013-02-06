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
#define CHECK_INTERNAL_COMPUTATION0 OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed != 0, "body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%_nHierarchyComputed, ORE_NotInitialized);
#define CHECK_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed == 2, "body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%_nHierarchyComputed, ORE_NotInitialized);

namespace OpenRAVE {

class ChangeCallbackData : public UserData
{
public:
    ChangeCallbackData(int properties, const boost::function<void()>& callback, KinBodyConstPtr pbody) : _properties(properties), _callback(callback), _pweakbody(pbody) {
    }
    virtual ~ChangeCallbackData() {
        KinBodyConstPtr pbody = _pweakbody.lock();
        if( !!pbody ) {
            boost::unique_lock< boost::shared_mutex > lock(pbody->GetInterfaceMutex());
            pbody->_listRegisteredCallbacks.erase(_iterator);
        }
    }

    list<UserDataWeakPtr>::iterator _iterator;
    int _properties;
    boost::function<void()> _callback;
protected:
    boost::weak_ptr<KinBody const> _pweakbody;
};

typedef boost::shared_ptr<ChangeCallbackData> ChangeCallbackDataPtr;

KinBody::KinBodyStateSaver::KinBodyStateSaver(KinBodyPtr pbody, int options) : _options(options), _pbody(pbody)
{
    if( _options & Save_LinkTransformation ) {
        _pbody->GetLinkTransformations(_vLinkTransforms, _vdofbranches);
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
    }
}

KinBody::KinBodyStateSaver::~KinBodyStateSaver()
{
    _RestoreKinBody(_pbody);
}

void KinBody::KinBodyStateSaver::Restore(boost::shared_ptr<KinBody> body)
{
    _RestoreKinBody(!body ? _pbody : body);
}

void KinBody::KinBodyStateSaver::Release()
{
    _pbody.reset();
}

void KinBody::KinBodyStateSaver::_RestoreKinBody(boost::shared_ptr<KinBody> pbody)
{
    if( !pbody ) {
        return;
    }
    if( pbody->GetEnvironmentId() == 0 ) {
        RAVELOG_WARN(str(boost::format("body %s not added to environment, skipping restore")%pbody->GetName()));
        return;
    }
    if( _options & Save_LinkTransformation ) {
        pbody->SetLinkTransformations(_vLinkTransforms, _vdofbranches);
    }
    if( _options & Save_LinkEnable ) {
        // should first enable before calling the parameter callbacks
        bool bchanged = false;
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            if( pbody->GetLinks().at(i)->IsEnabled() != !!_vEnabledLinks[i] ) {
                pbody->GetLinks().at(i)->_info._bIsEnabled = !!_vEnabledLinks[i];
                bchanged = true;
            }
        }
        if( bchanged ) {
            pbody->_nNonAdjacentLinkCache &= ~AO_Enabled;
            pbody->_ParametersChanged(Prop_LinkEnable);
        }
    }
    if( _options & Save_JointMaxVelocityAndAcceleration ) {
        pbody->SetDOFVelocityLimits(_vMaxVelocities);
        pbody->SetDOFAccelerationLimits(_vMaxAccelerations);
    }
    if( _options & Save_LinkVelocities ) {
        pbody->SetLinkVelocities(_vLinkVelocities);
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
}

KinBody::~KinBody()
{
    RAVELOG_VERBOSE(str(boost::format("destroying kinbody: %s\n")%GetName()));
    Destroy();
}

void KinBody::Destroy()
{
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
}

bool KinBody::InitFromBoxes(const std::vector<AABB>& vaabbs, bool visible)
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
    return true;
}

bool KinBody::InitFromBoxes(const std::vector<OBB>& vobbs, bool visible)
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
    return true;
}

bool KinBody::InitFromSpheres(const std::vector<Vector>& vspheres, bool visible)
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
    return true;
}

bool KinBody::InitFromTrimesh(const TriMesh& trimesh, bool visible)
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
    return true;
}

bool KinBody::InitFromGeometries(const std::list<KinBody::GeometryInfo>& geometries)
{
    std::vector<GeometryInfoConstPtr> newgeometries; newgeometries.reserve(geometries.size());
    FOREACHC(it, geometries) {
        newgeometries.push_back(GeometryInfoConstPtr(&(*it), utils::null_deleter()));
    }
    return InitFromGeometries(newgeometries);
}

bool KinBody::InitFromGeometries(const std::vector<KinBody::GeometryInfoConstPtr>& geometries)
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
    return true;
}

bool KinBody::Init(const std::vector<KinBody::LinkInfoConstPtr>& linkinfos, const std::vector<KinBody::JointInfoConstPtr>& jointinfos)
{
    OPENRAVE_ASSERT_FORMAT(GetEnvironmentId()==0, "%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    OPENRAVE_ASSERT_OP(linkinfos.size(),>,0);
    Destroy();
    _veclinks.reserve(linkinfos.size());
    FOREACHC(itlinkinfo, linkinfos) {
        LinkInfoConstPtr rawinfo = *itlinkinfo;
        LinkPtr plink(new Link(shared_kinbody()));
        plink->_info = *rawinfo;
        LinkInfo& info = plink->_info;
        plink->_index = static_cast<int>(_veclinks.size());
        FOREACHC(itgeominfo,info._vgeometryinfos) {
            Link::GeometryPtr geom(new Link::Geometry(plink,**itgeominfo));
            geom->_info.InitCollisionMesh();
            plink->_vGeometries.push_back(geom);
            plink->_collision.Append(geom->GetCollisionMesh(),geom->GetTransform());
        }
        FOREACHC(itadjacentname, info._vForcedAdjacentLinks) {
            _vForcedAdjacentLinks.push_back(std::make_pair(info._name, *itadjacentname));
        }
        _veclinks.push_back(plink);
    }
    _vecjoints.reserve(jointinfos.size());
    FOREACHC(itjointinfo, jointinfos) {
        JointInfoConstPtr rawinfo = *itjointinfo;
        JointPtr pjoint(new Joint(shared_kinbody(), rawinfo->_type));
        pjoint->_info = *rawinfo;
        JointInfo& info = pjoint->_info;
        for(size_t i = 0; i < info._vmimic.size(); ++i) {
            if( !!info._vmimic[i] ) {
                pjoint->_vmimic[i].reset(new Mimic());
                pjoint->_vmimic[i]->_equations = info._vmimic[i]->_equations;
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
        pjoint->_ComputeInternalInformation(plink0, plink1, info._vanchor, vaxes, info._vcurrentvalues);
        if( info._bIsActive ) {
            _vecjoints.push_back(pjoint);
        }
        else {
            _vPassiveJoints.push_back(pjoint);
        }
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
        _ParametersChanged(Prop_Name);
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
        v.resize(0);
        if( (int)v.capacity() < GetDOF() ) {
            v.reserve(GetDOF());
        }
        FOREACHC(it, _vDOFOrderedJoints) {
            int toadd = (*it)->GetDOFIndex()-(int)v.size();
            if( toadd > 0 ) {
                v.insert(v.end(),toadd,0);
            }
            else if( toadd < 0 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("dof indices mismatch joint %s, toadd=%d", (*it)->GetName()%toadd, ORE_InvalidState);
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
    _ParametersChanged(Prop_JointProperties);
}

void KinBody::SetDOFLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper)
{
    OPENRAVE_ASSERT_OP((int)lower.size(),==,GetDOF());
    OPENRAVE_ASSERT_OP((int)upper.size(),==,GetDOF());
    bool bChanged = false;
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
    if( bChanged ) {
        _ParametersChanged(Prop_JointLimits);
    }
}

void KinBody::SetDOFVelocityLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vmaxvel.begin());
        itv += (*it)->GetDOF();
    }
    _ParametersChanged(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFAccelerationLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vmaxaccel.begin());
        itv += (*it)->GetDOF();
    }
    _ParametersChanged(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SetDOFTorqueLimits(const std::vector<dReal>& v)
{
    std::vector<dReal>::const_iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        std::copy(itv,itv+(*it)->GetDOF(), (*it)->_info._vmaxtorque.begin());
        itv += (*it)->GetDOF();
    }
    _ParametersChanged(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::SimulationStep(dReal fElapsedTime)
{
}

void KinBody::SubtractDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2, const std::vector<int>& dofindices) const
{
    if( dofindices.size() == 0 ) {
        FOREACHC(itjoint,_vecjoints) {
            int dof = (*itjoint)->GetDOFIndex();
            for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                if( (*itjoint)->IsCircular(i) ) {
                    q1.at(dof+i) = utils::NormalizeCircularAngle(q1.at(dof+i)-q2.at(dof+i),(*itjoint)->_vcircularlowerlimit.at(i), (*itjoint)->_vcircularupperlimit.at(i));
                }
                else {
                    q1.at(dof+i) -= q2.at(dof+i);
                }
            }
        }
    }
    else {
        for(size_t i = 0; i < dofindices.size(); ++i) {
            JointPtr pjoint = GetJointFromDOFIndex(dofindices[i]);
            if( pjoint->IsCircular(dofindices[i]-pjoint->GetDOFIndex()) ) {
                int iaxis = dofindices[i]-pjoint->GetDOFIndex();
                q1.at(i) = utils::NormalizeCircularAngle(q1.at(i)-q2.at(i), pjoint->_vcircularlowerlimit.at(iaxis), pjoint->_vcircularupperlimit.at(iaxis));
            }
            else {
                q1.at(i) -= q2.at(i);
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
}

Transform KinBody::GetTransform() const
{
    return _veclinks.size() > 0 ? _veclinks.front()->GetTransform() : Transform();
}

bool KinBody::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    std::vector<std::pair<Vector,Vector> > velocities(_veclinks.size());
    velocities.at(0).first = linearvel;
    velocities.at(0).second = angularvel;
    Vector vlinktrans = _veclinks.at(0)->GetTransform().trans;
    for(size_t i = 1; i < _veclinks.size(); ++i) {
        velocities[i].first = linearvel + angularvel.cross(_veclinks[i]->GetTransform().trans-vlinktrans);
        velocities[i].second = angularvel;
    }
    return GetEnv()->GetPhysicsEngine()->SetLinkVelocities(shared_kinbody(),velocities);
}

void KinBody::SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, const Vector& linearvel, const Vector& angularvel, uint32_t checklimits)
{
    CHECK_INTERNAL_COMPUTATION;
    OPENRAVE_ASSERT_OP_FORMAT((int)vDOFVelocities.size(), >=, GetDOF(), "not enough values %d!=%d", vDOFVelocities.size()%GetDOF(),ORE_InvalidArguments);
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
                        RAVELOG_WARN(str(boost::format("failed to evaluate joint %s, fparser error %d")%pjoint->GetName()%err));
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
                            dummyvalues[i] += veval.at(ipartial) * partialvelocity;
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

        if( checklimits &&(dofindex >= 0)) {
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pvalues[i] < vlower.at(dofindex+i) ) {
                    if( checklimits == CLA_CheckLimits ) {
                        RAVELOG_WARN(str(boost::format("dof %d velocity is not in limits %.15e<%.15e")%i%pvalues[i]%vlower.at(dofindex+i)));
                    }
                    else if( checklimits == CLA_CheckLimitsThrow ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("dof %d velocity is not in limits %.15e<%.15e", i%pvalues[i]%vlower.at(dofindex+i), ORE_InvalidArguments);
                    }
                    dummyvalues[i] = vlower[i];
                }
                else if( pvalues[i] > vupper.at(dofindex+i) ) {
                    if( checklimits == CLA_CheckLimits ) {
                        RAVELOG_WARN(str(boost::format("dof %d velocity is not in limits %.15e>%.15e")%i%pvalues[i]%vupper.at(dofindex+i)));
                    }
                    else if( checklimits == CLA_CheckLimitsThrow ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("dof %d velocity is not in limits %.15e>%.15e", i%pvalues[i]%vupper.at(dofindex+i), ORE_InvalidArguments);
                    }
                    dummyvalues[i] = vupper[i];
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
//                RAVELOG_WARN(str(boost::format("forward kinematic type %d not supported")%pjoint->GetType()));
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
                    RAVELOG_WARN(str(boost::format("error with evaluation of joint %s")%pjoint->GetName()));
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
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("joint %s not supported for querying velocities",pjoint->GetType(),ORE_Assert);
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
    OPENRAVE_ASSERT_OP_FORMAT0(vDOFVelocities.size(),==,dofindices.size(),"index sizes do not match", ORE_InvalidArguments);
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
        RAVELOG_VERBOSE("GetLinkTransformations should be called with dofbranches\n");
    }
    vtrans.resize(_veclinks.size());
    vector<Transform>::iterator it;
    vector<LinkPtr>::const_iterator itlink;
    for(it = vtrans.begin(), itlink = _veclinks.begin(); it != vtrans.end(); ++it, ++itlink) {
        *it = (*itlink)->GetTransform();
    }
}

void KinBody::GetLinkTransformations(vector<Transform>& vtrans, std::vector<int>& dofbranches) const
{
    vtrans.resize(_veclinks.size());
    vector<Transform>::iterator it;
    vector<LinkPtr>::const_iterator itlink;
    for(it = vtrans.begin(), itlink = _veclinks.begin(); it != vtrans.end(); ++it, ++itlink) {
        *it = (*itlink)->GetTransform();
    }

    dofbranches.resize(0);
    if( (int)dofbranches.capacity() < GetDOF() ) {
        dofbranches.reserve(GetDOF());
    }
    FOREACHC(it, _vDOFOrderedJoints) {
        int toadd = (*it)->GetDOFIndex()-(int)dofbranches.size();
        if( toadd > 0 ) {
            dofbranches.insert(dofbranches.end(),toadd,0);
        }
        else if( toadd < 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT("dof indices mismatch joint %s, toadd=%d", (*it)->GetName()%toadd, ORE_InvalidState);
        }
        for(int i = 0; i < (*it)->GetDOF(); ++i) {
            dofbranches.push_back((*it)->_dofbranches[i]);
        }
    }
}

KinBody::JointPtr KinBody::GetJointFromDOFIndex(int dofindex) const
{
    return _vecjoints.at(_vDOFIndices.at(dofindex));
}

AABB KinBody::ComputeAABB() const
{
    Vector vmin, vmax;
    bool binitialized=false;
    AABB ab;
    FOREACHC(itlink,_veclinks) {
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
        RAVELOG_WARN("SetLinkTransformations should be called with dofbranches, setting all branches to 0\n");
    }
    else {
        RAVELOG_DEBUG("SetLinkTransformations should be called with dofbranches, setting all branches to 0\n");
    }
    OPENRAVE_ASSERT_OP_FORMAT(vbodies.size(), >=, _veclinks.size(), "not enough links %d<%d", vbodies.size()%_veclinks.size(),ORE_InvalidArguments);
    vector<Transform>::const_iterator it;
    vector<LinkPtr>::iterator itlink;
    for(it = vbodies.begin(), itlink = _veclinks.begin(); it != vbodies.end(); ++it, ++itlink) {
        (*itlink)->SetTransform(*it);
    }
    FOREACH(itjoint,_vecjoints) {
        for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
            (*itjoint)->_dofbranches[i] = 0;
        }
    }
    _nUpdateStampId++;
}

void KinBody::SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<int>& dofbranches)
{
    OPENRAVE_ASSERT_OP_FORMAT(transforms.size(), >=, _veclinks.size(), "not enough links %d<%d", transforms.size()%_veclinks.size(),ORE_InvalidArguments);
    vector<Transform>::const_iterator it;
    vector<LinkPtr>::iterator itlink;
    for(it = transforms.begin(), itlink = _veclinks.begin(); it != transforms.end(); ++it, ++itlink) {
        (*itlink)->SetTransform(*it);
    }
    FOREACH(itjoint,_vecjoints) {
        for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
            (*itjoint)->_dofbranches[i] = dofbranches.at((*itjoint)->GetDOFIndex()+i);
        }
    }
    _nUpdateStampId++;
}

void KinBody::SetLinkVelocities(const std::vector<std::pair<Vector,Vector> >& velocities)
{
    GetEnv()->GetPhysicsEngine()->SetLinkVelocities(shared_kinbody(),velocities);
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
    _nUpdateStampId++;
    if( vJointValues.size() == 0 || _veclinks.size() == 0) {
        return;
    }
    int expecteddof = dofindices.size() > 0 ? (int)dofindices.size() : GetDOF();
    OPENRAVE_ASSERT_OP_FORMAT((int)vJointValues.size(),>=,expecteddof, "not enough values %d<%d", vJointValues.size()%GetDOF(),ORE_InvalidArguments);

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
                                    RAVELOG_WARN(str(boost::format("dof %d value is not in limits %e<%e")%((*it)->GetDOFIndex()+i)%p[i]%lowerlim[i]));
                                }
                                else if( checklimits == CLA_CheckLimitsThrow ) {
                                    throw OPENRAVE_EXCEPTION_FORMAT("dof %d value is not in limits %e<%e", ((*it)->GetDOFIndex()+i)%p[i]%lowerlim[i], ORE_InvalidArguments);
                                }
                            }
                            *ptempjoints++ = lowerlim[i];
                        }
                        else if( p[i] > upperlim[i] ) {
                            if( p[i] > upperlim[i]+g_fEpsilonEvalJointLimit ) {
                                if( checklimits == CLA_CheckLimits ) {
                                    RAVELOG_WARN(str(boost::format("dof %d value is not in limits %e<%e")%((*it)->GetDOFIndex()+i)%p[i]%upperlim[i]));
                                }
                                else if( checklimits == CLA_CheckLimitsThrow ) {
                                    throw OPENRAVE_EXCEPTION_FORMAT("dof %d value is not in limits %e<%e",((*it)->GetDOFIndex()+i)%p[i]%upperlim[i], ORE_InvalidArguments);
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

    boost::array<dReal,3> dummyvalues; // dummy values for a joint
    std::vector<dReal> vtempvalues, veval;

    // have to compute the angles ahead of time since they are dependent on the link transformations
    std::vector< std::vector<dReal> > vPassiveJointValues(_vPassiveJoints.size());
    for(size_t i = 0; i < vPassiveJointValues.size(); ++i) {
        if( !_vPassiveJoints[i]->IsMimic() ) {
            _vPassiveJoints[i]->GetValues(vPassiveJointValues[i]);
            // check if out of limits!
            for(size_t j = 0; j < vPassiveJointValues[i].size(); ++j) {
                if( !_vPassiveJoints[i]->IsCircular(j) ) {
                    if( vPassiveJointValues[i][j] < _vPassiveJoints[i]->_info._vlowerlimit.at(j) ) {
                        if( vPassiveJointValues[i][j] < _vPassiveJoints[i]->_info._vlowerlimit.at(j)-5e-4f ) {
                            RAVELOG_WARN(str(boost::format("dummy joint out of lower limit! %e < %e\n")%_vPassiveJoints[i]->_info._vlowerlimit.at(j)%vPassiveJointValues[i][j]));
                        }
                        vPassiveJointValues[i][j] = _vPassiveJoints[i]->_info._vlowerlimit.at(j);
                    }
                    else if( vPassiveJointValues[i][j] > _vPassiveJoints[i]->_info._vupperlimit.at(j) ) {
                        if( vPassiveJointValues[i][j] > _vPassiveJoints[i]->_info._vupperlimit.at(j)+5e-4f ) {
                            RAVELOG_WARN(str(boost::format("dummy joint out of upper limit! %e > %e\n")%_vPassiveJoints[i]->_info._vupperlimit.at(j)%vPassiveJointValues[i][j]));
                        }
                        vPassiveJointValues[i][j] = _vPassiveJoints[i]->_info._vupperlimit.at(j);
                    }
                }
            }
        }
        else {
            vPassiveJointValues[i].reserve(_vPassiveJoints[i]->GetDOF()); // do not resize so that we can catch hierarchy errors
        }
    }

    std::vector<uint8_t> vlinkscomputed(_veclinks.size(),0);
    vlinkscomputed[0] = 1;

    for(size_t ijoint = 0; ijoint < _vTopologicallySortedJointsAll.size(); ++ijoint) {
        JointPtr pjoint = _vTopologicallySortedJointsAll[ijoint];
        int jointindex = _vTopologicallySortedJointIndicesAll[ijoint];
        int dofindex = pjoint->GetDOFIndex();
        const dReal* pvalues=dofindex >= 0 ? pJointValues + dofindex : NULL;
        if( pjoint->IsMimic() ) {
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pjoint->IsMimic(i) ) {
                    vtempvalues.resize(0);
                    const std::vector<Mimic::DOFFormat>& vdofformat = pjoint->_vmimic[i]->_vdofformat;
                    FOREACHC(itdof,vdofformat) {
                        if( itdof->dofindex >= 0 ) {
                            vtempvalues.push_back(pJointValues[itdof->dofindex]);
                        }
                        else {
                            vtempvalues.push_back(vPassiveJointValues.at(itdof->jointindex-_vecjoints.size()).at(itdof->axis));
                        }
                    }
                    int err = pjoint->_Eval(i, 0, vtempvalues, veval);
                    if( err ) {
                        RAVELOG_WARN(str(boost::format("failed to evaluate joint %s, fparser error %d")%pjoint->GetName()%err));
                    }
                    else {
                        vector<dReal> vevalcopy = veval;
                        vector<dReal>::iterator iteval = veval.begin();
                        while(iteval != veval.end()) {
                            bool removevalue = false;
                            if( pjoint->GetType() == JointSpherical || pjoint->IsCircular(i) ) {
                            }
                            else if( *iteval < pjoint->_info._vlowerlimit[i] ) {
                                if(*iteval >= pjoint->_info._vlowerlimit[i]-g_fEpsilonJointLimit ) {
                                    *iteval = pjoint->_info._vlowerlimit[i];
                                }
                                else {
                                    removevalue=true;
                                }
                            }
                            else if( *iteval > pjoint->_info._vupperlimit[i] ) {
                                if(*iteval <= pjoint->_info._vupperlimit[i]+g_fEpsilonJointLimit ) {
                                    *iteval = pjoint->_info._vupperlimit[i];
                                }
                                else {
                                    removevalue=true;
                                }
                            }

                            if( removevalue ) {
                                iteval = veval.erase(iteval); // invalid value so remove from candidates
                            }
                            else {
                                ++iteval;
                            }
                        }

                        if( veval.empty() ) {
                            FORIT(iteval,vevalcopy) {
                                if( checklimits == CLA_Nothing || pjoint->GetType() == JointSpherical || pjoint->IsCircular(i) ) {
                                    veval.push_back(*iteval);
                                }
                                else if( *iteval < pjoint->_info._vlowerlimit[i]-g_fEpsilonEvalJointLimit ) {
                                    veval.push_back(pjoint->_info._vlowerlimit[i]);
                                    if( checklimits == CLA_CheckLimits ) {
                                        RAVELOG_WARN(str(boost::format("joint %s: lower limit (%e) is not followed: %e")%pjoint->GetName()%pjoint->_info._vlowerlimit[i]%*iteval));
                                    }
                                    else if( checklimits == CLA_CheckLimitsThrow ) {
                                        throw OPENRAVE_EXCEPTION_FORMAT("joint %s: lower limit (%e) is not followed: %e", pjoint->GetName()%pjoint->_info._vlowerlimit[i]%*iteval, ORE_InvalidArguments);
                                    }
                                }
                                else if( *iteval > pjoint->_info._vupperlimit[i]+g_fEpsilonEvalJointLimit ) {
                                    veval.push_back(pjoint->_info._vupperlimit[i]);
                                    if( checklimits == CLA_CheckLimits ) {
                                        RAVELOG_WARN(str(boost::format("joint %s: upper limit (%e) is not followed: %e")%pjoint->GetName()%pjoint->_info._vupperlimit[i]%*iteval));
                                    }
                                    else if( checklimits == CLA_CheckLimitsThrow ) {
                                        throw OPENRAVE_EXCEPTION_FORMAT("joint %s: upper limit (%e) is not followed: %e", pjoint->GetName()%pjoint->_info._vupperlimit[i]%*iteval, ORE_InvalidArguments);
                                    }
                                }
                                else {
                                    veval.push_back(*iteval);
                                }
                            }
                            OPENRAVE_ASSERT_FORMAT(!veval.empty(), "no valid values for joint %s", pjoint->GetName(),ORE_Assert);
                        }
                        if( veval.size() > 1 ) {
                            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                            ss << "multiplie values for joint " << pjoint->GetName() << ": ";
                            FORIT(iteval,veval) {
                                ss << *iteval << " ";
                            }
                            RAVELOG_WARN(ss.str());
                        }
                        dummyvalues[i] = veval.at(0);
                    }

                    // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                    if( dofindex < 0 ) {
                        vPassiveJointValues.at(jointindex-(int)_vecjoints.size()).resize(pjoint->GetDOF());
                        vPassiveJointValues.at(jointindex-(int)_vecjoints.size()).at(i) = dummyvalues[i];
                    }
                }
                else if( dofindex >= 0 ) {
                    dummyvalues[i] = pvalues[dofindex+i]; // is this correct? what is a joint has a mimic and non-mimic axis?
                }
                else {
                    // preserve passive joint values
                    dummyvalues[i] = vPassiveJointValues.at(jointindex-(int)_vecjoints.size()).at(i);
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
            pvalues = &vPassiveJointValues.at(jointindex-(int)_vecjoints.size()).at(0);
        }

        Transform tjoint;
        if( pjoint->GetType() & JointSpecialBit ) {
            switch(pjoint->GetType()) {
            case JointHinge2: {
                Transform tfirst;
                tfirst.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pvalues[0]);
                Transform tsecond;
                tsecond.rot = quatFromAxisAngle(tfirst.rotate(pjoint->GetInternalHierarchyAxis(1)), pvalues[1]);
                tjoint = tsecond * tfirst;
                pjoint->_dofbranches[0] = CountCircularBranches(pvalues[0]-pjoint->GetWrapOffset(0));
                pjoint->_dofbranches[1] = CountCircularBranches(pvalues[1]-pjoint->GetWrapOffset(1));
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
                if( pjoint->IsCircular(0) ) {
                    // need to normalize the value
                    fvalue = utils::NormalizeCircularAngle(fvalue,pjoint->_vcircularlowerlimit.at(0), pjoint->_vcircularupperlimit.at(0));
                }
                pjoint->_info._trajfollow->Sample(vdata,fvalue);
                if( !pjoint->_info._trajfollow->GetConfigurationSpecification().ExtractTransform(tjoint,vdata.begin(),KinBodyConstPtr()) ) {
                    RAVELOG_WARN(str(boost::format("trajectory sampling for joint %s failed")%pjoint->GetName()));
                }
                pjoint->_dofbranches[0] = 0;
                break;
            }
            default:
                RAVELOG_WARN(str(boost::format("forward kinematic type 0x%x not supported")%pjoint->GetType()));
                break;
            }
        }
        else {
            if( pjoint->GetType() == JointRevolute ) {
                tjoint.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pvalues[0]);
                pjoint->_dofbranches[0] = CountCircularBranches(pvalues[0]-pjoint->GetWrapOffset(0));
            }
            else if( pjoint->GetType() == JointPrismatic ) {
                tjoint.trans = pjoint->GetInternalHierarchyAxis(0) * pvalues[0];
            }
            else {
                for(int iaxis = 0; iaxis < pjoint->GetDOF(); ++iaxis) {
                    Transform tdelta;
                    if( pjoint->IsRevolute(iaxis) ) {
                        tdelta.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(iaxis), pvalues[iaxis]);
                        pjoint->_dofbranches[iaxis] = CountCircularBranches(pvalues[iaxis]-pjoint->GetWrapOffset(iaxis));
                    }
                    else {
                        tdelta.trans = pjoint->GetInternalHierarchyAxis(iaxis) * pvalues[iaxis];
                    }
                    tjoint = tjoint * tdelta;
                }
            }
        }

        Transform t = pjoint->GetInternalHierarchyLeftTransform() * tjoint * pjoint->GetInternalHierarchyRightTransform();
        if( !pjoint->GetHierarchyParentLink() ) {
            t = _veclinks.at(0)->GetTransform() * t;
        }
        else {
            t = pjoint->GetHierarchyParentLink()->GetTransform() * t;
        }
        pjoint->GetHierarchyChildLink()->SetTransform(t);
        vlinkscomputed[pjoint->GetHierarchyChildLink()->GetIndex()] = 1;
    }
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
            JointPtr pjoint = _vecjoints.at(jointindex);
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
    _ComputeLinkAccelerations(vDOFVelocities, vDOFAccelerations, vLinkVelocities, vLinkAccelerations, vgravity);

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
                doftorques.at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vcomforce);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT("joint 0x%x not supported", pjoint->GetType(), ORE_Assert);
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
                faxistorque = pjoint->GetAxis(0).dot3(vcomforce);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT("joint 0x%x not supported", pjoint->GetType(), ORE_Assert);
            }

            pjoint->_ComputePartialVelocities(vpartials,0,mapcachedpartials);
            FOREACH(itpartial,vpartials) {
                int dofindex = itpartial->first;
                doftorques.at(dofindex) += itpartial->second*faxistorque;
            }
        }
        else {
            // joint should be static
            BOOST_ASSERT(pjoint->IsStatic());
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
        _ComputeLinkAccelerations(std::vector<dReal>(), std::vector<dReal>(), vLinkVelocities[2], vLinkAccelerations[2], vgravity);
        if( bHasVelocity ) {
            _ComputeLinkAccelerations(vDOFVelocities, std::vector<dReal>(), vLinkVelocities[1], vLinkAccelerations[1], Vector(0,0,0));
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0], Vector(0,0,0));
            }
            else {
                linkaccelsimilar[0] = 1;
            }
        }
        else {
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0], Vector(0,0,0));
            }
        }
    }
    else {
        // no external forces
        vLinkVelocities[2] = vLinkVelocities[0];
        if( bHasVelocity ) {
            _ComputeLinkAccelerations(vDOFVelocities, std::vector<dReal>(), vLinkVelocities[1], vLinkAccelerations[1], Vector(0,0,0));
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0], Vector(0,0,0));
            }
            else {
                linkaccelsimilar[0] = 1;
            }
        }
        else {
            if( vDOFAccelerations.size() > 0 ) {
                _ComputeLinkAccelerations(std::vector<dReal>(), vDOFAccelerations, vLinkVelocities[0], vLinkAccelerations[0], Vector(0,0,0));
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
                    vDOFTorqueComponents[j].at(pjoint->GetDOFIndex()) += pjoint->GetAxis(0).dot3(vcomforce);
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT("joint 0x%x not supported", pjoint->GetType(), ORE_Assert);
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
                    faxistorque = pjoint->GetAxis(0).dot3(vcomforce);
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT("joint 0x%x not supported", pjoint->GetType(), ORE_Assert);
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

void KinBody::GetLinkAccelerations(const std::vector<dReal>&vDOFAccelerations, std::vector<std::pair<Vector,Vector> >&vLinkAccelerations) const
{
    CHECK_INTERNAL_COMPUTATION;
    if( _veclinks.size() == 0 ) {
        vLinkAccelerations.resize(0);
    }
    else {
        std::vector<dReal> vDOFVelocities;
        std::vector<pair<Vector, Vector> > vLinkVelocities;
        _ComputeDOFLinkVelocities(vDOFVelocities,vLinkVelocities);
        _ComputeLinkAccelerations(vDOFVelocities, vDOFAccelerations, vLinkVelocities, vLinkAccelerations, GetEnv()->GetPhysicsEngine()->GetGravity());
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
        int childindex = (*it)->_attachedbodies[0]->GetIndex();
        (*it)->_GetVelocities(dofvelocities,true,vLinkVelocities.at(parentindex),vLinkVelocities.at(childindex));
    }
}

void KinBody::_ComputeLinkAccelerations(const std::vector<dReal>& vDOFVelocities, const std::vector<dReal>& vDOFAccelerations, const std::vector< std::pair<Vector, Vector> >& vLinkVelocities, std::vector<std::pair<Vector,Vector> >& vLinkAccelerations, const Vector& vGravity) const
{
    vLinkAccelerations.resize(_veclinks.size());
    if( _veclinks.size() == 0 ) {
        return;
    }

    vector<dReal> vtempvalues, veval;
    boost::array<dReal,3> dummyvelocities = {{0,0,0}}, dummyaccelerations={{0,0,0}}; // dummy values for a joint

    // set accelerations of first link to 0 since it isn't actuated
    vLinkAccelerations.at(0).first = -vGravity + vLinkVelocities.at(0).second.cross(vLinkVelocities.at(0).first);
    vLinkAccelerations.at(0).second = Vector();

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
                            RAVELOG_WARN(str(boost::format("failed to evaluate joint %s, fparser error %d")%pjoint->GetName()%err));
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
                                dummyvelocities[i] += veval.at(ipartial) * partialvelocity;
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
                                dummyaccelerations[i] += veval.at(ipartial) * partialacceleration;
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
            throw OPENRAVE_EXCEPTION_FORMAT("joint type 0x%x not supported for getting link acceleration",pjoint->GetType(),ORE_Assert);
        }
        vlinkscomputed[childindex] = 1;
    }
}

bool KinBody::CheckSelfCollision(CollisionReportPtr report) const
{
    if( GetEnv()->CheckSelfCollision(shared_kinbody_const(), report) ) {
        if( !!report ) {
            RAVELOG_VERBOSE(str(boost::format("Self collision: %s\n")%report->__str__()));
        }
        return true;
    }
    return false;
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
                            JointPtr pjoint = itdofformat->GetJoint(shared_kinbody());
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
                            JointPtr pjoint = itmimic->first.GetJoint(shared_kinbody());
                            JointPtr pjointparent = itdofformat->GetJoint(shared_kinbody());
                            throw OPENRAVE_EXCEPTION_FORMAT("joint index %s uses a mimic joint %s that also depends on %s! this is not allowed", pjoint->GetName()%pjointparent->GetName()%pjoint->GetName(), ORE_Failed);
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
            int parentlinkindex = -1;
            if( !(*itjoint)->GetFirstAttached() || (*itjoint)->GetFirstAttached()->IsStatic() ) {
                if( !!(*itjoint)->GetSecondAttached() && !(*itjoint)->GetSecondAttached()->IsStatic() ) {
                    parentlinkindex = (*itjoint)->GetSecondAttached()->GetIndex();
                }
            }
            else if( !(*itjoint)->GetSecondAttached() || (*itjoint)->GetSecondAttached()->IsStatic() ) {
                parentlinkindex = (*itjoint)->GetFirstAttached()->GetIndex();
            }
            else {
                // NOTE: possibly try to choose roots that do not involve mimic joints. ikfast might have problems
                // dealing with very complex formulas
                LinkPtr plink0 = (*itjoint)->GetFirstAttached(), plink1 = (*itjoint)->GetSecondAttached();
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
                RAVELOG_WARN(str(boost::format("could not compute parent link for joint %s")%(*itjoint)->GetName()));
            }
            else if( parentlinkindex != (*itjoint)->GetFirstAttached()->GetIndex() ) {
                RAVELOG_VERBOSE(str(boost::format("swapping link order of joint %s(%d)")%(*itjoint)->GetName()%(*itjoint)->GetJointIndex()));
                // have to swap order
                Transform tswap = (*itjoint)->GetInternalHierarchyRightTransform().inverse();
                std::vector<Vector> vaxes((*itjoint)->GetDOF());
                std::vector<dReal> vcurrentvalues;
                (*itjoint)->GetValues(vcurrentvalues);
                for(size_t i = 0; i < vaxes.size(); ++i) {
                    vaxes[i] = -tswap.rotate((*itjoint)->GetInternalHierarchyAxis(i));
                }
                // have to reset the link transformations temporarily in order to avoid setting a joint offset
                TransformSaver<LinkPtr> linksaver0((*itjoint)->GetFirstAttached());
                TransformSaver<LinkPtr> linksaver1((*itjoint)->GetSecondAttached());
                (*itjoint)->GetFirstAttached()->SetTransform(Transform());
                (*itjoint)->GetSecondAttached()->SetTransform((*itjoint)->GetInternalHierarchyLeftTransform()*(*itjoint)->GetInternalHierarchyRightTransform());
                (*itjoint)->_ComputeInternalInformation((*itjoint)->GetSecondAttached(),(*itjoint)->GetFirstAttached(),tswap.trans,vaxes,vcurrentvalues);
            }
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
                _vClosedLoopIndices.back().push_back(make_pair<int16_t,int16_t>(*itlinkindex,0));
                _vClosedLoops.back().push_back(make_pair<LinkPtr,JointPtr>(_veclinks.at(*itlinkindex),JointPtr()));
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
                throw OPENRAVE_EXCEPTION_FORMAT("joint indices %d and %d share the same name '%s'", pjoint0->GetJointIndex()%pjoint1->GetJointIndex()%pjoint0->GetName(), ORE_InvalidState);
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
    _nHierarchyComputed = 2;
    // because of mimic joints, need to call SetDOFValues at least once, also use this to check for links that are off
    {
        vector<Transform> vprevtrans, vnewtrans;
        vector<int> vprevdofbranches, vnewdofbranches;
        GetLinkTransformations(vprevtrans, vprevdofbranches);
        vector<dReal> vcurrentvalues;
        // unfortunately if SetDOFValues is overloaded by the robot, it could call the robot's _UpdateGrabbedBodies, which is a problem during environment cloning since the grabbed bodies might not be initialized. Therefore, call KinBody::SetDOFValues
        GetDOFValues(vcurrentvalues);
        KinBody::SetDOFValues(vcurrentvalues,true, std::vector<int>());
        GetLinkTransformations(vnewtrans, vnewdofbranches);
        for(size_t i = 0; i < vprevtrans.size(); ++i) {
            if( TransformDistanceFast(vprevtrans[i],vnewtrans[i]) > 1e-5 ) {
                RAVELOG_VERBOSE(str(boost::format("link %d has different transformation after SetDOFValues (error=%f), this could be due to mimic joint equations kicking into effect.")%_veclinks.at(i)->GetName()%TransformDistanceFast(vprevtrans[i],vnewtrans[i])));
            }
        }
        for(int i = 0; i < GetDOF(); ++i) {
            if( vprevdofbranches.at(i) != vnewdofbranches.at(i) ) {
                RAVELOG_VERBOSE(str(boost::format("dof %d has different branches after SetDOFValues %d!=%d, this could be due to mimic joint equations kicking into effect.")%i%vprevdofbranches.at(i)%vnewdofbranches.at(i)));
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

    // notify any callbacks of the changes
    if( _nParametersChanged ) {
        std::list<UserDataWeakPtr> listRegisteredCallbacks;
        {
            boost::shared_lock< boost::shared_mutex > lock(GetInterfaceMutex());
            listRegisteredCallbacks = _listRegisteredCallbacks; // copy since it can be changed
        }
        FOREACH(it,listRegisteredCallbacks) {
            ChangeCallbackDataPtr pdata = boost::dynamic_pointer_cast<ChangeCallbackData>(it->lock());
            if( !!pdata && (pdata->_properties & _nParametersChanged) ) {
                pdata->_callback();
            }
        }
        _nParametersChanged = 0;
    }
    RAVELOG_DEBUG(str(boost::format("_ComputeInternalInformation %s in %fs")%GetName()%(1e-6*(utils::GetMicroTime()-starttime))));
}

bool KinBody::IsAttached(KinBodyConstPtr pbody) const
{
    if( shared_kinbody_const() == pbody ) {
        return true;
    }
    std::set<KinBodyConstPtr> dummy;
    return _IsAttached(pbody,dummy);
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

bool KinBody::_IsAttached(KinBodyConstPtr pbody, std::set<KinBodyConstPtr>&setChecked) const
{
    if( !setChecked.insert(shared_kinbody_const()).second ) {
        return false;
    }
    FOREACHC(itbody,_listAttachedBodies) {
        KinBodyConstPtr pattached = itbody->lock();
        if( !!pattached && ((pattached == pbody)|| pattached->_IsAttached(pbody,setChecked)) ) {
            return true;
        }
    }
    return false;
}

void KinBody::_AttachBody(KinBodyPtr pbody)
{
    _listAttachedBodies.push_back(pbody);
    pbody->_listAttachedBodies.push_back(shared_kinbody());
}

bool KinBody::_RemoveAttachedBody(KinBodyPtr pbody)
{
    int numremoved = 0;
    FOREACH(it,_listAttachedBodies) {
        if( it->lock() == pbody ) {
            _listAttachedBodies.erase(it);
            numremoved++;
            break;
        }
    }

    KinBodyPtr pthisbody = shared_kinbody();
    FOREACH(it,pbody->_listAttachedBodies) {
        if( it->lock() == pthisbody ) {
            pbody->_listAttachedBodies.erase(it);
            numremoved++;
            break;
        }
    }

    return numremoved==2;
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
        _ParametersChanged(Prop_LinkEnable);
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
        _ParametersChanged(Prop_LinkDraw);
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

void KinBody::SetNonCollidingConfiguration()
{
    _ResetInternalCollisionCache();
    vector<int> vdofbranches;
    GetLinkTransformations(_vInitialLinkTransformations, vdofbranches);
}

void KinBody::_ResetInternalCollisionCache()
{
    _nNonAdjacentLinkCache = 0x80000000;
    FOREACH(it,_setNonAdjacentLinks) {
        it->clear();
    }
}

const std::set<int>& KinBody::GetNonAdjacentLinks(int adjacentoptions) const
{
    class TransformsSaver
    {
public:
        TransformsSaver(KinBodyConstPtr pbody) : _pbody(pbody) {
            _pbody->GetLinkTransformations(vcurtrans, _vdofbranches);
        }
        ~TransformsSaver() {
            for(size_t i = 0; i < _pbody->_veclinks.size(); ++i) {
                boost::static_pointer_cast<Link>(_pbody->_veclinks[i])->_info._t = vcurtrans.at(i);
            }
            for(size_t i = 0; i < _pbody->_vecjoints.size(); ++i) {
                for(int j = 0; j < _pbody->_vecjoints[i]->GetDOF(); ++j) {
                    _pbody->_vecjoints[i]->_dofbranches[j] = _vdofbranches.at(_pbody->_vecjoints[i]->GetDOFIndex()+j);
                }
            }
        }
private:
        KinBodyConstPtr _pbody;
        std::vector<Transform> vcurtrans;
        std::vector<int> _vdofbranches;
    };

    CHECK_INTERNAL_COMPUTATION;
    if( _nNonAdjacentLinkCache & 0x80000000 ) {
        // Check for colliding link pairs given the initial pose _vInitialLinkTransformations
        // this is actually weird, we need to call the individual link collisions on a const body. in order to pull this off, we need to be very careful with the body state.
        TransformsSaver saver(shared_kinbody_const());
        CollisionOptionsStateSaver colsaver(GetEnv()->GetCollisionChecker(),0); // have to reset the collision options
        for(size_t i = 0; i < _veclinks.size(); ++i) {
            boost::static_pointer_cast<Link>(_veclinks[i])->_info._t = _vInitialLinkTransformations.at(i);
        }
        _nUpdateStampId++; // because transforms were modified
        for(size_t i = 0; i < _veclinks.size(); ++i) {
            for(size_t j = i+1; j < _veclinks.size(); ++j) {
                if((_setAdjacentLinks.find(i|(j<<16)) == _setAdjacentLinks.end())&& !GetEnv()->CheckCollision(LinkConstPtr(_veclinks[i]), LinkConstPtr(_veclinks[j])) ) {
                    _setNonAdjacentLinks[0].insert(i|(j<<16));
                }
            }
        }
        _nUpdateStampId++; // because transforms were modified
        _nNonAdjacentLinkCache = 0;
    }
    if( (_nNonAdjacentLinkCache&adjacentoptions) != adjacentoptions ) {
        int requestedoptions = (~_nNonAdjacentLinkCache)&adjacentoptions;
        // find out what needs to computed
        if( requestedoptions & AO_Enabled ) {
            _setNonAdjacentLinks.at(AO_Enabled).clear();
            FOREACHC(itset, _setNonAdjacentLinks[0]) {
                KinBody::LinkConstPtr plink1(_veclinks.at(*itset&0xffff)), plink2(_veclinks.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    _setNonAdjacentLinks[AO_Enabled].insert(*itset);
                }
            }
            _nNonAdjacentLinkCache |= AO_Enabled;
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("no support for adjacentoptions %d", adjacentoptions,ORE_InvalidArguments);
        }
    }
    return _setNonAdjacentLinks.at(adjacentoptions);
}

const std::set<int>& KinBody::GetAdjacentLinks() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _setAdjacentLinks;
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
        *pnewlink = **itlink; // be careful of copying pointers
        pnewlink->_parent = shared_kinbody();
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
                throw OPENRAVE_EXCEPTION_FORMAT("joint %s doesn't belong to anythong?",(*itjoint)->GetName(), ORE_Assert);
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
            _vClosedLoops.back().push_back(make_pair(_veclinks.at(it->first->GetIndex()),JointPtr()));
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
                    throw OPENRAVE_EXCEPTION_FORMAT("joint %s in closed loop doesn't belong to anythong?",(*itjoint)->GetName(), ORE_Assert);
                }
            }
        }
    }

    _listAttachedBodies.clear(); // will be set in the environment
    FOREACHC(itatt, r->_listAttachedBodies) {
        KinBodyConstPtr pattref = itatt->lock();
        if( !!pattref ) {
            _listAttachedBodies.push_back(GetEnv()->GetBodyFromEnvironmentId(pattref->GetEnvironmentId()));
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
    _nUpdateStampId++; // update the stamp instead of copying
}

void KinBody::_ParametersChanged(int parameters)
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
    boost::array<int,3> hashproperties = {{Prop_LinkDynamics, Prop_LinkGeometry, Prop_JointMimic }};
    FOREACH(it, hashproperties) {
        if( (parameters & *it) == *it ) {
            __hashkinematics.resize(0);
            break;
        }
    }

    std::list<UserDataWeakPtr> listRegisteredCallbacks;
    {
        boost::shared_lock< boost::shared_mutex > lock(GetInterfaceMutex());
        listRegisteredCallbacks = _listRegisteredCallbacks; // copy since it can be changed
    }
    FOREACH(it,listRegisteredCallbacks) {
        ChangeCallbackDataPtr pdata = boost::dynamic_pointer_cast<ChangeCallbackData>(it->lock());
        if( !!pdata && (pdata->_properties & parameters) ) {
            pdata->_callback();
        }
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
        (*itjoint)->_ComputeInternalInformation((*itjoint)->GetFirstAttached(), (*itjoint)->GetSecondAttached(),(*itjoint)->GetInternalHierarchyLeftTransform().trans,vaxes,std::vector<dReal>());
    }
}

const std::string& KinBody::GetKinematicsGeometryHash() const
{
    CHECK_INTERNAL_COMPUTATION;
    if( __hashkinematics.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics|SO_Geometry);
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

UserDataPtr KinBody::RegisterChangeCallback(int properties, const boost::function<void()>&callback) const
{
    ChangeCallbackDataPtr pdata(new ChangeCallbackData(properties,callback,shared_kinbody_const()));
    boost::unique_lock< boost::shared_mutex > lock(GetInterfaceMutex());
    pdata->_iterator = _listRegisteredCallbacks.insert(_listRegisteredCallbacks.end(),pdata);
    return pdata;
}

} // end namespace OpenRAVE
