// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
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
/*! --------------------------------------------------------------------
  \file   KinBody.Cpp
  \brief  Encapsulate a kinematic chain of links
 -------------------------------------------------------------------- */

#include "libopenrave.h"

#include <algorithm>

namespace OpenRAVE {

void KinBody::Link::TRIMESH::ApplyTransform(const Transform& t)
{
    FOREACH(it, vertices)
        *it = t * *it;
}

void KinBody::Link::TRIMESH::ApplyTransform(const TransformMatrix& t)
{
    FOREACH(it, vertices)
        *it = t * *it;
}

void KinBody::Link::TRIMESH::Append(const TRIMESH& mesh)
{
    int offset = (int)vertices.size();
    vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
    if( indices.capacity() < indices.size()+mesh.indices.size() )
        indices.reserve(indices.size()+mesh.indices.size());

    FOREACHC(it, mesh.indices)
        indices.push_back(*it+offset);
}

void KinBody::Link::TRIMESH::Append(const TRIMESH& mesh, const Transform& trans)
{
    int offset = (int)vertices.size();
    vertices.resize(vertices.size() + mesh.vertices.size());
    for(size_t i = 0; i < mesh.vertices.size(); ++i)
        vertices[i+offset] = trans * mesh.vertices[i];
    
    if( indices.capacity() < indices.size()+mesh.indices.size() )
        indices.reserve(indices.size()+mesh.indices.size());

    FOREACHC(it, mesh.indices)
        indices.push_back(*it+offset);
}

AABB KinBody::Link::TRIMESH::ComputeAABB() const
{
    AABB ab;
    Vector vmin, vmax;
    vmin = vmax = vertices.front();
    FOREACHC(itv, vertices) {
        Vector v = *itv;
        if( vmin.x > v.x ) vmin.x = v.x;
        if( vmin.y > v.y ) vmin.y = v.y;
        if( vmin.z > v.z ) vmin.z = v.z;
        if( vmax.x < v.x ) vmax.x = v.x;
        if( vmax.y < v.y ) vmax.y = v.y;
        if( vmax.z < v.z ) vmax.z = v.z;
    }

    ab.extents = (dReal)0.5*(vmax-vmin);
    ab.pos = (dReal)0.5*(vmax+vmin);
    return ab;
}

KinBody::Link::GEOMPROPERTIES::GEOMPROPERTIES()
{
    diffuseColor = Vector(1,1,1);
    type = GeomNone;
    ftransparency = 0;
    vRenderScale = Vector(1,1,1);
    bDraw = true;
}

AABB KinBody::Link::GEOMPROPERTIES::ComputeAABB(const Transform& t) const
{
    AABB ab;

    TransformMatrix tglobal = t * _t;

    switch(type) {
    case GeomBox:
        ab.extents.x = RaveFabs(tglobal.m[0])*vGeomData.x + RaveFabs(tglobal.m[1])*vGeomData.y + RaveFabs(tglobal.m[2])*vGeomData.z;
        ab.extents.y = RaveFabs(tglobal.m[4])*vGeomData.x + RaveFabs(tglobal.m[5])*vGeomData.y + RaveFabs(tglobal.m[6])*vGeomData.z;
        ab.extents.z = RaveFabs(tglobal.m[8])*vGeomData.x + RaveFabs(tglobal.m[9])*vGeomData.y + RaveFabs(tglobal.m[10])*vGeomData.z;
        ab.pos = tglobal.trans;
        break;
    case GeomSphere:
        ab.extents.x = ab.extents.y = ab.extents.z = vGeomData[0];
        ab.pos = tglobal.trans;
        break;
    case GeomCylinder:
        ab.extents.x = (dReal)0.5*tglobal.m[2]*vGeomData.y + RaveSqrt(1-tglobal.m[2]*tglobal.m[2])*vGeomData.x;
        ab.extents.y = (dReal)0.5*tglobal.m[6]*vGeomData.y + RaveSqrt(1-tglobal.m[6]*tglobal.m[6])*vGeomData.x;
        ab.extents.z = (dReal)0.5*tglobal.m[10]*vGeomData.y + RaveSqrt(1-tglobal.m[10]*tglobal.m[10])*vGeomData.x;
        ab.pos = tglobal.trans+(dReal)0.5*vGeomData.y*Vector(tglobal.m[2],tglobal.m[6],tglobal.m[10]);
        break;
    case GeomTrimesh:
        // just use collisionmesh
        if( collisionmesh.vertices.size() > 0) {
            Vector vmin, vmax; vmin = vmax = tglobal*collisionmesh.vertices.front();
            FOREACHC(itv, collisionmesh.vertices) {
                Vector v = tglobal * *itv;
                if( vmin.x > v.x ) vmin.x = v.x;
                if( vmin.y > v.y ) vmin.y = v.y;
                if( vmin.z > v.z ) vmin.z = v.z;
                if( vmax.x < v.x ) vmax.x = v.x;
                if( vmax.y < v.y ) vmax.y = v.y;
                if( vmax.z < v.z ) vmax.z = v.z;
            }

            ab.extents = (dReal)0.5*(vmax-vmin);
            ab.pos = (dReal)0.5*(vmax+vmin);
        }
        break;
    default:
        assert(0);
    }

    return ab;
}

#define GTS_M_ICOSAHEDRON_X /* sqrt(sqrt(5)+1)/sqrt(2*sqrt(5)) */ \
  (dReal)0.850650808352039932181540497063011072240401406
#define GTS_M_ICOSAHEDRON_Y /* sqrt(2)/sqrt(5+sqrt(5))         */ \
  (dReal)0.525731112119133606025669084847876607285497935
#define GTS_M_ICOSAHEDRON_Z (dReal)0.0

// generate a sphere triangulation starting with an icosahedron
// all triangles are oriented counter clockwise
void GenerateSphereTriangulation(KinBody::Link::TRIMESH& tri, int levels)
{
    KinBody::Link::TRIMESH temp, temp2;

    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));
    temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
    temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
    temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
    temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
    temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));

    const int nindices=60;
    int indices[nindices] = {
        0, 1, 2,
        1, 3, 4,
        3, 5, 6,
        2, 4, 7,
        5, 6, 8,
        2, 7, 9,
        0, 5, 8,
        7, 9, 10,
        0, 1, 5,
        7, 10, 11,
        1, 3, 5,
        6, 10, 11,
        3, 6, 11,
        9, 10, 8,
        3, 4, 11,
        6, 8, 10,
        4, 7, 11,
        1, 2, 4,
        0, 8, 9,
        0, 2, 9
    };

    Vector v[3];
    
    // make sure oriented CCW 
    for(int i = 0; i < nindices; i += 3 ) {
        v[0] = temp.vertices[indices[i]];
        v[1] = temp.vertices[indices[i+1]];
        v[2] = temp.vertices[indices[i+2]];
        if( dot3(v[0], (v[1]-v[0]).Cross(v[2]-v[0])) < 0 )
            swap(indices[i], indices[i+1]);
    }

    temp.indices.resize(nindices);
    std::copy(&indices[0],&indices[nindices],temp.indices.begin());

    KinBody::Link::TRIMESH* pcur = &temp;
    KinBody::Link::TRIMESH* pnew = &temp2;
    while(levels-- > 0) {

        pnew->vertices.resize(0);
        pnew->vertices.reserve(2*pcur->vertices.size());
        pnew->vertices.insert(pnew->vertices.end(), pcur->vertices.begin(), pcur->vertices.end());
        pnew->indices.resize(0);
        pnew->indices.reserve(4*pcur->indices.size());

        map< uint64_t, int > mapnewinds;
        map< uint64_t, int >::iterator it;

        for(size_t i = 0; i < pcur->indices.size(); i += 3) {
            // for ever tri, create 3 new vertices and 4 new triangles.
            v[0] = pcur->vertices[pcur->indices[i]];
            v[1] = pcur->vertices[pcur->indices[i+1]];
            v[2] = pcur->vertices[pcur->indices[i+2]];

            int inds[3];
            for(int j = 0; j < 3; ++j) {
                uint64_t key = ((uint64_t)pcur->indices[i+j]<<32)|(uint64_t)pcur->indices[i + ((j+1)%3) ];
                it = mapnewinds.find(key);

                if( it == mapnewinds.end() ) {
                    inds[j] = mapnewinds[key] = mapnewinds[(key<<32)|(key>>32)] = (int)pnew->vertices.size();
                    pnew->vertices.push_back((v[j]+v[(j+1)%3 ]).normalize3());
                }
                else {
                    inds[j] = it->second;
                }
            }

            pnew->indices.push_back(pcur->indices[i]);    pnew->indices.push_back(inds[0]);    pnew->indices.push_back(inds[2]);
            pnew->indices.push_back(inds[0]);    pnew->indices.push_back(pcur->indices[i+1]);    pnew->indices.push_back(inds[1]);
            pnew->indices.push_back(inds[2]);    pnew->indices.push_back(inds[0]);    pnew->indices.push_back(inds[1]);
            pnew->indices.push_back(inds[2]);    pnew->indices.push_back(inds[1]);    pnew->indices.push_back(pcur->indices[i+2]);
        }

        swap(pnew,pcur);
    }

    tri = *pcur;
}

bool KinBody::Link::GEOMPROPERTIES::InitCollisionMesh(float fTessellation)
{
    if( GetType() == KinBody::Link::GEOMPROPERTIES::GeomTrimesh )
        return true;

    collisionmesh.indices.clear();
    collisionmesh.vertices.clear();

    if( fTessellation < (dReal)0.01 )
        fTessellation = (dReal)0.01;

    // start tesselating
    switch(GetType()) {
    case KinBody::Link::GEOMPROPERTIES::GeomSphere: {
        // log_2 (1+ tess)
        GenerateSphereTriangulation(collisionmesh, 3 + (int)(logf(fTessellation) / logf(2.0f)) );
        dReal fRadius = GetSphereRadius();
        FOREACH(it, collisionmesh.vertices)
            *it *= fRadius;
        break;
    }
    case KinBody::Link::GEOMPROPERTIES::GeomBox: {
        // trivial
        Vector ex = GetBoxExtents();
        Vector v[8] = { Vector(ex.x, ex.y, ex.z),
                        Vector(ex.x, ex.y, -ex.z),
                        Vector(ex.x, -ex.y, ex.z),
                        Vector(ex.x, -ex.y, -ex.z),
                        Vector(-ex.x, ex.y, ex.z),
                        Vector(-ex.x, ex.y, -ex.z),
                        Vector(-ex.x, -ex.y, ex.z),
                        Vector(-ex.x, -ex.y, -ex.z) };
        const int nindices = 36;
        int indices[] = {
            0, 1, 2,
            1, 2, 3,
            4, 5, 6,
            5, 6, 7,
            0, 1, 4,
            1, 4, 5,
            2, 3, 6,
            3, 6, 7,
            0, 2, 4,
            2, 4, 6,
            1, 3, 5,
            3, 5, 7
        };

        for(int i = 0; i < nindices; i += 3 ) {
            Vector v1 = v[indices[i]];
            Vector v2 = v[indices[i+1]];
            Vector v3 = v[indices[i+2]];
            Vector vtemp;
            if( dot3(v1, cross3(vtemp, v2-v1, v3-v1)) < 0 )
                swap(indices[i], indices[i+1]);
        }

        collisionmesh.vertices.resize(8);
        std::copy(&v[0],&v[8],collisionmesh.vertices.begin());
        collisionmesh.indices.resize(nindices);
        std::copy(&indices[0],&indices[nindices],collisionmesh.indices.begin());
        break;
    }
    case KinBody::Link::GEOMPROPERTIES::GeomCylinder: {
        // cylinder is on y axis
        dReal rad = GetCylinderRadius(), len = GetCylinderHeight()*0.5f;

        int numverts = (int)(fTessellation*24.0f) + 3;
        dReal dtheta = 2 * PI / (dReal)numverts;
        collisionmesh.vertices.push_back(Vector(0,0,len));
        collisionmesh.vertices.push_back(Vector(0,0,-len));
        collisionmesh.vertices.push_back(Vector(rad,0,len));
        collisionmesh.vertices.push_back(Vector(rad,0,-len));

        for(int i = 0; i < numverts+1; ++i) {
            dReal s = rad * sinf(dtheta * (dReal)i);
            dReal c = rad * cosf(dtheta * (dReal)i);

            int off = (int)collisionmesh.vertices.size();
            collisionmesh.vertices.push_back(Vector(c, s, len));
            collisionmesh.vertices.push_back(Vector(c, s, -len));

            collisionmesh.indices.push_back(0);       collisionmesh.indices.push_back(off);       collisionmesh.indices.push_back(off-2);
            collisionmesh.indices.push_back(1);       collisionmesh.indices.push_back(off-1);       collisionmesh.indices.push_back(off+1);
            collisionmesh.indices.push_back(off-2);   collisionmesh.indices.push_back(off);         collisionmesh.indices.push_back(off-1);
            collisionmesh.indices.push_back(off);   collisionmesh.indices.push_back(off-1);         collisionmesh.indices.push_back(off+1);
        }
        break;
    }
    default:
        throw openrave_exception(str(boost::format("unrecognized geom type %d!\n")%GetType()));
    }

    return true;
}

KinBody::Link::Link(KinBodyPtr parent)
{
    _parent = parent;
    bStatic = false;
    userdata = 0;
    index = -1;
    _bIsEnabled = true;
}

KinBody::Link::~Link()
{
    //RAVELOG_VERBOSEA(str(boost::format("destroy link %s\n")%GetName()));
}

bool KinBody::Link::IsEnabled() const
{
    return _bIsEnabled;
}

void KinBody::Link::Enable(bool bEnable)
{
    GetParent()->GetEnv()->GetCollisionChecker()->EnableLink(shared_from_this(),bEnable);
    _bIsEnabled = bEnable;
}

AABB KinBody::Link::ComputeAABB() const
{
    // enable/disable everything
    if( _listGeomProperties.size() == 1)
        return _listGeomProperties.front().ComputeAABB(_t);
    else if( _listGeomProperties.size() > 1 ) {
        AABB ab = _listGeomProperties.front().ComputeAABB(_t);
        Vector vmin = ab.pos-ab.extents, vmax = ab.pos+ab.extents;
        list<GEOMPROPERTIES>::const_iterator it = _listGeomProperties.begin();
        while(++it != _listGeomProperties.end()) {
            ab = it->ComputeAABB(_t);
            Vector vmin2 = ab.pos-ab.extents, vmax2 = ab.pos+ab.extents;
            if( vmin.x > vmin2.x ) vmin.x = vmin2.x;
            if( vmin.y > vmin2.y ) vmin.y = vmin2.y;
            if( vmin.z > vmin2.z ) vmin.z = vmin2.z;
            if( vmax.x < vmax2.x ) vmax.x = vmax2.x;
            if( vmax.y < vmax2.y ) vmax.y = vmax2.y;
            if( vmax.z < vmax2.z ) vmax.z = vmax2.z;
        }

        return AABB( 0.5f * (vmin+vmax), 0.5f * (vmax-vmin) );
    }

    return AABB();
}

void KinBody::Link::SetTransform(const Transform& t)
{
    _t = t;
    GetParent()->_nUpdateStampId++;
}

Transform KinBody::Link::GetTransform() const
{
    return _t;
}

void KinBody::Link::SetForce(const Vector& force, const Vector& pos, bool bAdd)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetBodyForce(shared_from_this(), force, pos, bAdd);
}

void KinBody::Link::SetTorque(const Vector& torque, bool bAdd)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetBodyTorque(shared_from_this(), torque, bAdd);
}

KinBody::Joint::Joint(KinBodyPtr parent)
{
    _parent = parent;
    vMimicCoeffs.resize(2); vMimicCoeffs[0] = 1; vMimicCoeffs[1] = 0;
    nMimicJointIndex = -1;
    fResolution = dReal(0.02);
    fMaxVel = 1e5f;
    fMaxAccel = 1e5f;
    fMaxTorque = 1e5f;
    offset = 0;
    dofindex = -1; // invalid index
}

KinBody::Joint::~Joint()
{
}

int KinBody::Joint::GetDOF() const
{
    switch(type) {
    case JointHinge:
    case JointSlider: return 1;
    case JointHinge2:
    case JointUniversal: return 2;
    case JointSpherical: return 3;
    default: return 0;
    }
}

void KinBody::Joint::GetValues(vector<dReal>& pValues, bool bAppend) const
{
    Transform tjoint;
    if ( !!bodies[1] && !bodies[1]->IsStatic() ) 
        tjoint = tinvLeft * bodies[0]->GetTransform().inverse() * bodies[1]->GetTransform() * tinvRight;
    else
        tjoint = tinvLeft * GetParent()->GetTransform().inverse() * bodies[0]->GetTransform() * tinvRight;
        
    if( !bAppend )
        pValues.resize(0);

    switch(type) {
    case JointHinge:
        pValues.push_back(offset-2.0f*RaveAtan2(tjoint.rot.y*vAxes[0].x+tjoint.rot.z*vAxes[0].y+tjoint.rot.w*vAxes[0].z, tjoint.rot.x));
        break;
    case JointHinge2:
        {
            Vector axis1cur = tjoint.rotate(vAxes[0]);
            Vector axis2cur = tjoint.rotate(vAxes[1]);
            Vector vec1;
            Vector vec2;
            Vector vec3;
            vec1 = (vAxes[1] - vAxes[0].dot(vAxes[1])*vAxes[0]).normalize();
            vec2 = (axis2cur - vAxes[0].dot(axis2cur)*vAxes[0]).normalize();
            vec3 = vAxes[0]; vec3.Cross(vec1);
            pValues.push_back(-RaveAtan2(vec3.dot(vec2), vec1.dot(vec2)));
            vec1 = (vAxes[0] - axis2cur.dot(vAxes[0])*axis2cur).normalize();
            vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
            vec3 = axis2cur; vec3.Cross(vec1);
            pValues.push_back(-RaveAtan2(vec3.dot(vec2), vec1.dot(vec2)));
        }
        break;
    case JointSlider:
        pValues.push_back(offset-(tjoint.trans.x*vAxes[0].x+tjoint.trans.y*vAxes[0].y+tjoint.trans.z*vAxes[0].z));
        break;
    case JointSpherical: {
        dReal fsinang2 = tjoint.rot.y*tjoint.rot.y+tjoint.rot.z*tjoint.rot.z+tjoint.rot.w*tjoint.rot.w;
        if( fsinang2 > 1e-10f ) {
            dReal fsinang = RaveSqrt(fsinang2);
            dReal fmult = 2*RaveAtan2(fsinang,tjoint.rot.x)/fsinang;
            pValues.push_back(tjoint.rot.y*fmult);
            pValues.push_back(tjoint.rot.z*fmult);
            pValues.push_back(tjoint.rot.w*fmult);
        }
        else {
            pValues.push_back(0);
            pValues.push_back(0);
            pValues.push_back(0);
        }
        break;
    }
    default:
        throw openrave_exception(str(boost::format("unknown joint type %d\n")%type));
    }
}

Vector KinBody::Joint::GetAnchor() const
{
    if( !bodies[0] )
        return vanchor;
    else if( !!bodies[1] && bodies[1]->IsStatic() )
        return bodies[1]->GetTransform() * vanchor;
    else
        return bodies[0]->GetTransform() * vanchor;
}

Vector KinBody::Joint::GetAxis(int iaxis ) const
{
    assert(iaxis >= 0 && iaxis < 3 );
    if( !bodies[0] )
        return vAxes[iaxis];
    else if( !!bodies[1] && bodies[1]->IsStatic() )
        return bodies[1]->GetTransform().rotate(vAxes[iaxis]);
    else
        return bodies[0]->GetTransform().rotate(vAxes[iaxis]);
}

void KinBody::Joint::GetVelocities(std::vector<dReal>& pVelocities, bool bAppend) const
{
    vector<dReal> dummy;
    if( !GetParent()->GetEnv()->GetPhysicsEngine()->GetJointVelocity(shared_from_this(), bAppend ? dummy : pVelocities) )
        throw openrave_exception("Failed to get velocities");
    if( bAppend )
        pVelocities.insert(pVelocities.end(),dummy.begin(),dummy.end());
}

void KinBody::Joint::GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, bool bAppend) const
{
    if( bAppend ) {
        vLowerLimit.insert(vLowerLimit.end(),_vlowerlimit.begin(),_vlowerlimit.end());
        vUpperLimit.insert(vUpperLimit.end(),_vupperlimit.begin(),_vupperlimit.end());
    }
    else {
        vLowerLimit = _vlowerlimit;
        vUpperLimit = _vupperlimit;
    }
}

void KinBody::Joint::SetJointOffset(dReal newoffset)
{
    offset = newoffset;
    GetParent()->ParametersChanged(Prop_JointOffset);
}

void KinBody::Joint::SetJointLimits(const std::vector<dReal>& vLowerLimit, const std::vector<dReal>& vUpperLimit)
{
    _vlowerlimit = vLowerLimit;
    _vupperlimit = vUpperLimit;
    GetParent()->ParametersChanged(Prop_JointLimits);
}

void KinBody::Joint::SetResolution(dReal resolution)
{
    fResolution = resolution;
    GetParent()->ParametersChanged(Prop_JointProperties);
}

void KinBody::Joint::AddTorque(const std::vector<dReal>& pTorques)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->AddJointTorque(shared_from_this(), pTorques);
}

KinBody::KinBodyStateSaver::KinBodyStateSaver(KinBodyPtr pbody) : _pbody(pbody)
{
    _pbody->GetBodyTransformations(_vtransPrev);
}

KinBody::KinBodyStateSaver::~KinBodyStateSaver()
{
    _pbody->SetBodyTransformations(_vtransPrev);
}

KinBody::KinBody(PluginType type, EnvironmentBasePtr penv) : InterfaceBase(type, penv)
{
    _bHierarchyComputed = false;
    networkid = 0;
    
    _nUpdateStampId = 0;
}

KinBody::~KinBody()
{
    RAVELOG_VERBOSEA(str(boost::format("destroying kinbody: %s\n")%GetName()));
    Destroy();
}

void KinBody::Destroy()
{
    _veclinks.clear();
    _vecjoints.clear();
    _vecPassiveJoints.clear();
    _vecJointIndices.clear();
    _vecJointWeights.clear();
    _vecJointHierarchy.clear();

    _setAttachedBodies.clear();
    _setAdjacentLinks.clear();
    _setNonAdjacentLinks.clear();
    _vForcedAdjacentLinks.clear();
    _bHierarchyComputed = false;
    _pGuiData.reset();
    _pPhysicsData.reset();
    _pCollisionData.reset();
    _pManageData.reset();
}

bool KinBody::InitFromFile(const std::string& filename, const std::list<std::pair<std::string,std::string> >& atts)
{
    bool bSuccess = GetEnv()->ReadKinBodyXMLFile(shared_kinbody(), filename, atts)==shared_kinbody();
    if( !bSuccess ) {
        Destroy();
        return false;
    }

    return true;
}

bool KinBody::InitFromData(const std::string& data, const std::list<std::pair<std::string,std::string> >& atts)
{
    bool bSuccess = GetEnv()->ReadKinBodyXMLData(shared_kinbody(), data, atts)==shared_kinbody();
    if( !bSuccess ) {
        Destroy();
        return false;
    }

    return true;
}

bool KinBody::InitFromBoxes(const std::vector<AABB>& vaabbs, bool bDraw)
{
    bool bAddedToEnv = GetEnv()->RemoveKinBody(shared_kinbody());
    if( bAddedToEnv )
        throw openrave_exception(str(boost::format("KinBody::Init for %s, cannot Init a body while it is added to the environment\n")%GetName()));

    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->index = 0;
    plink->name = "base";
    Link::TRIMESH trimesh;
    FOREACHC(itab, vaabbs) {
        plink->_listGeomProperties.push_back(Link::GEOMPROPERTIES());
        Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        geom.type = Link::GEOMPROPERTIES::GeomBox;
        geom._t.trans = itab->pos;
        geom.bDraw = bDraw;
        geom.vGeomData = itab->extents;
        geom.InitCollisionMesh();
        trimesh = geom.GetCollisionMesh();
        trimesh.ApplyTransform(geom._t);
        plink->collision.Append(trimesh);
    }
    
    _veclinks.push_back(plink);
    return true;
}

void KinBody::SetName(const std::string& newname)
{
    name = newname;
}

void KinBody::SetJointTorques(const std::vector<dReal>& torques, bool bAdd)
{
    if( (int)torques.size() != GetDOF() )
        throw openrave_exception(str(boost::format("dof not equal %d!=%d")%torques.size()%GetDOF()),ORE_InvalidArguments);

    if( !bAdd ) {
        FOREACH(itlink, _veclinks) {
            (*itlink)->SetForce(Vector(),Vector(),false);
            (*itlink)->SetTorque(Vector(),false);
        }
    }

    vector<int>::iterator itindex = _vecJointIndices.begin();
    std::vector<dReal> jointtorques;
    FOREACH(it, _vecjoints) {
        jointtorques.resize((*it)->GetDOF());
        std::copy(torques.begin()+*itindex,torques.begin()+*itindex+(*it)->GetDOF(),jointtorques.begin());
        (*it)->AddTorque(jointtorques);
    }
}

void KinBody::GetJointValues(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints)
        (*it)->GetValues(v,true);
}

void KinBody::GetJointVelocities(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints)
        (*it)->GetVelocities(v,true);
}

void KinBody::GetJointLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const
{
    vLowerLimit.resize(0);
    if( (int)vLowerLimit.capacity() < GetDOF() )
        vLowerLimit.reserve(GetDOF());
    vUpperLimit.resize(0);
    if( (int)vUpperLimit.capacity() < GetDOF() )
        vUpperLimit.reserve(GetDOF());
    FOREACHC(it,_vecjoints)
        (*it)->GetLimits(vLowerLimit,vUpperLimit,true);
}

void KinBody::GetJointMaxVel(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints)
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxVel());
}

void KinBody::GetJointMaxAccel(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints)
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxAccel());
}

void KinBody::GetJointMaxTorque(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints)
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxTorque());
}

void KinBody::GetJointResolutions(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints)
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetResolution());
}

void KinBody::SimulationStep(dReal fElapsedTime)
{
}

// like apply transform except everything is relative to the first frame
void KinBody::SetTransform(const Transform& trans)
{
    if( _veclinks.size() == 0 )
        return;

    Transform tbaseinv = _veclinks.front()->GetTransform().inverse();
    ApplyTransform(trans * tbaseinv);
}

Transform KinBody::GetTransform() const
{
    return _veclinks.size() > 0 ? _veclinks.front()->GetTransform() : Transform();
}

void KinBody::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    GetEnv()->GetPhysicsEngine()->SetBodyVelocity(shared_kinbody(), linearvel, angularvel);
}

void KinBody::GetVelocity(Vector& linearvel, Vector& angularvel) const
{
    GetEnv()->GetPhysicsEngine()->GetBodyVelocity(shared_kinbody_const(), linearvel, angularvel);
}

void KinBody::GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const
{
    velocities.resize(_veclinks.size());
    if( velocities.size() == 0 )
        return;
    GetVelocity(velocities[0].first,velocities[0].second);

    // set the first body and all static bodies to computed
    _veclinks[0]->userdata = 1;
    int numleft = (int)_veclinks.size()-1;
    for(size_t i = 1; i < _veclinks.size(); ++i) {
        if( _veclinks[i]->IsStatic() ) {
            numleft--;
            _veclinks[i]->userdata = 1;
            velocities[i] = velocities[0];
        }
        else _veclinks[i]->userdata = 0;
    }

    const vector<JointPtr>* alljoints[2] = {&_vecjoints, &_vecPassiveJoints};
    vector<dReal> vjointang,vjointvel;

    while(numleft > 0) {
        int org = numleft;

        // iterate through the two sets of joints: active and passive
        // for active joints, read the angle from pJointValues, for passive joints
        // check if they have corresponding mimic joints, if they do, take those angles
        // otherwise set to 0
        for(size_t j = 0; j < 2; ++j) {
            FOREACHC(itjoint, *alljoints[j]) {
                (*itjoint)->GetValues(vjointang);
                if( (*itjoint)->GetMimicJointIndex() >= 0 ) {
                    _vecjoints[(*itjoint)->GetMimicJointIndex()]->GetVelocities(vjointvel);
                    FOREACH(itv,vjointvel)
                        *itv *= (*itjoint)->GetMimicCoeffs()[0];
                }
                else
                    (*itjoint)->GetVelocities(vjointvel);

                LinkPtr* bodies = (*itjoint)->bodies;
                if( !!bodies[0] && !!bodies[1] && !bodies[1]->IsStatic()) {
                    if( bodies[0]->userdata ) {
                        if( !bodies[1]->userdata ) {
                            Vector w,v;
                            // init 1 from 0
                            switch((*itjoint)->GetType()) {
                            case Joint::JointHinge:
                                w = -vjointvel[0]*(*itjoint)->vAxes[0];
                                break;
                            case Joint::JointHinge2: {
                                Transform tfirst;
                                tfirst.rotfromaxisangle((*itjoint)->vAxes[0], -vjointang[0]);
                                w = -vjointvel[0]*(*itjoint)->vAxes[0] + tfirst.rotate(-vjointvel[1]*(*itjoint)->vAxes[1]);
                                break;
                            }
                            case Joint::JointSlider:
                                v = -vjointvel[0]*(*itjoint)->vAxes[0];
                                break;
                            case Joint::JointSpherical:
                                w.x = vjointvel[0]; w.y = vjointvel[1]; w.z = vjointvel[2];
                                break;
                            default:
                                RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                                break;
                            }
                            
                            Transform tbody0 = bodies[0]->GetTransform();
                            Transform tdelta = tbody0 * (*itjoint)->tLeft;
                            Vector vparent = velocities[bodies[0]->GetIndex()].first;
                            Vector wparent = velocities[bodies[0]->GetIndex()].second;
                            velocities[bodies[1]->GetIndex()].first = vparent + tdelta.rotate(v) + Vector().Cross(wparent,bodies[1]->GetTransform().trans-tbody0.trans);
                            velocities[bodies[1]->GetIndex()].second = wparent + tdelta.rotate(w);

                            bodies[1]->userdata = 1;
                            numleft--;
                        }
                    }
                    else if( bodies[1]->userdata ) {
                        Vector w,v;
                        // init 1 from 0
                        switch((*itjoint)->GetType()) {
                        case Joint::JointHinge:
                            w = vjointvel[0]*(*itjoint)->vAxes[0];
                            break;
                        case Joint::JointHinge2: {
                            Transform tfirst;
                            tfirst.rotfromaxisangle((*itjoint)->vAxes[0], vjointang[0]);
                            w = vjointvel[0]*(*itjoint)->vAxes[0] + tfirst.rotate(vjointvel[1]*(*itjoint)->vAxes[1]);
                            break;
                        }
                        case Joint::JointSlider:
                            v = vjointvel[0]*(*itjoint)->vAxes[0];
                            break;
                        case Joint::JointSpherical:
                            w.x = -vjointvel[0]; w.y = -vjointvel[1]; w.z = -vjointvel[2];
                            break;
                        default:
                            RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                            break;
                        }

                        Transform tbody1 = bodies[1]->GetTransform();
                        Transform tdelta = tbody1 * (*itjoint)->tinvRight;
                        Vector vparent = velocities[bodies[1]->GetIndex()].first;
                        Vector wparent = velocities[bodies[1]->GetIndex()].second;
                        velocities[bodies[0]->GetIndex()].first = vparent + tdelta.rotate(v) + Vector().Cross(wparent,bodies[0]->GetTransform().trans-tbody1.trans);
                        velocities[bodies[0]->GetIndex()].second = wparent + tdelta.rotate(w);

                        bodies[0]->userdata = 1;
                        numleft--;
                    }
                }
                else if( !!bodies[0] && !bodies[0]->userdata ) {
                    Vector w,v;
                    // joint attached to static environment (it will never be [1])
                    switch((*itjoint)->GetType()) {
                    case Joint::JointHinge:
                        w = -vjointvel[0]*(*itjoint)->vAxes[0];
                        break;
                    case Joint::JointHinge2: {
                        Transform tfirst;
                        tfirst.rotfromaxisangle((*itjoint)->vAxes[0], -vjointang[0]);
                        w = -vjointvel[0]*(*itjoint)->vAxes[0] + tfirst.rotate(-vjointvel[1]*(*itjoint)->vAxes[1]);
                        break;
                    }
                    case Joint::JointSlider:
                        v = -vjointvel[0]*(*itjoint)->vAxes[0];
                        break;
                    case Joint::JointSpherical:
                        w.x = vjointvel[0]; w.y = vjointvel[1]; w.z = vjointvel[2];
                        break;
                    default:
                        RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                        break;
                    }
                    
                    Transform tbody = GetTransform();
                    Transform tdelta = tbody * (*itjoint)->tLeft;
                    Vector vparent = velocities[0].first;
                    Vector wparent = velocities[0].second;
                    velocities[bodies[0]->GetIndex()].first = vparent + tdelta.rotate(v) + Vector().Cross(wparent,bodies[0]->GetTransform().trans-tbody.trans);
                    velocities[bodies[0]->GetIndex()].second = wparent + tdelta.rotate(w);

                    bodies[0]->userdata = 1;
                    numleft--;
                }
            }
        }

        // nothing changed so exit
        if( org == numleft )
            break;
    }

    // some links might not be connected to any joints. In this case, transform them by tbase
    if( numleft > 0 ) {
        for(size_t i = 1; i < _veclinks.size(); ++i) {
            if( _veclinks[i]->userdata == 0 ) {
                velocities[i] = velocities[0];
            }
        }
    }
}

void KinBody::ApplyTransform(const Transform& trans)
{
    FOREACH(itlink, _veclinks) {
        Transform tlocal = (*itlink)->GetTransform();
        Transform tfinal = trans * (*itlink)->GetTransform();
        (*itlink)->SetTransform(trans * (*itlink)->GetTransform());
    }
}

void KinBody::GetBodyTransformations(vector<Transform>& vtrans) const
{
    vtrans.resize(_veclinks.size());

    vector<Transform>::iterator it;
    vector<LinkPtr>::const_iterator itlink;
    
    for(it = vtrans.begin(), itlink = _veclinks.begin(); it != vtrans.end(); ++it, ++itlink)
        *it = (*itlink)->GetTransform();
}

KinBody::JointPtr KinBody::GetJointFromDOFIndex(int dofindex) const
{
    int jointindex = 0;
    FOREACHC(it, _vecJointIndices) {
        if( dofindex <= *it )
            break;
        ++jointindex;
    }
    if( jointindex < (int)_vecjoints.size() )
        return _vecjoints[jointindex];
    return JointPtr();
}

AABB KinBody::ComputeAABB() const
{
    if( _veclinks.size() == 0 )
        return AABB();
    
    Vector vmin, vmax;
    std::vector<LinkPtr>::const_iterator itlink = _veclinks.begin();

    AABB ab = (*itlink++)->ComputeAABB();
    vmin = ab.pos - ab.extents;
    vmax = ab.pos + ab.extents;

    while(itlink != _veclinks.end()) {
        ab = (*itlink++)->ComputeAABB();
        Vector vnmin = ab.pos - ab.extents;
        Vector vnmax = ab.pos + ab.extents;

        if( vmin.x > vnmin.x ) vmin.x = vnmin.x;
        if( vmin.y > vnmin.y ) vmin.y = vnmin.y;
        if( vmin.z > vnmin.z ) vmin.z = vnmin.z;
        if( vmax.x < vnmax.x ) vmax.x = vnmax.x;
        if( vmax.y < vnmax.y ) vmax.y = vnmax.y;
        if( vmax.z < vnmax.z ) vmax.z = vnmax.z;
    }

    ab.pos = (dReal)0.5 * (vmin + vmax);
    ab.extents = vmax - ab.pos;
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

    if( fTotalMass > 0 )
        center /= fTotalMass;
    return center;
}

dReal KinBody::ConfigDist(const std::vector<dReal>& q1) const
{
    if( (int)q1.size() != GetDOF() )
        throw openrave_exception(str(boost::format("configs %"PRIdS" now equal to body dof %d")%q1.size()%GetDOF()));

    dReal dist = 0;
    vector<JointPtr>::const_iterator it;
    vector<dReal>::const_iterator itweight = _vecJointWeights.begin();

    std::vector<dReal> vTempJoints;
    GetJointValues(vTempJoints);

    for(int i = 0; i < GetDOF(); ++i) {
        dReal s = vTempJoints[i]-q1[i];
        dist += s * s * *itweight++;
    }

    return RaveSqrt(dist);
}

dReal KinBody::ConfigDist(const std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    if( (int)q1.size() != GetDOF() || (int)q2.size() != GetDOF() )
        throw openrave_exception(str(boost::format("configs %"PRIdS" %"PRIdS" now equal to body dof %d")%q1.size()%q2.size()%GetDOF()));

    dReal dist = 0.0;
    for (size_t i = 0; i < _vecJointWeights.size(); i++) {
        dist += _vecJointWeights[i] * (q2[i] - q1[i]) * (q2[i] - q1[i]);
    }
    return RaveSqrt(dist);
}

void KinBody::SetBodyTransformations(const std::vector<Transform>& vbodies)
{
    if( vbodies.size() != _veclinks.size() )
        throw openrave_exception(str(boost::format("links not equal %d!=%d")%vbodies.size()%_veclinks.size()),ORE_InvalidArguments);
    
    vector<Transform>::const_iterator it;
    vector<LinkPtr>::iterator itlink;
    
    for(it = vbodies.begin(), itlink = _veclinks.begin(); it != vbodies.end(); ++it, ++itlink)
        (*itlink)->SetTransform(*it);

    _nUpdateStampId++;
}

void KinBody::SetJointValues(const std::vector<dReal>& vJointValues, const Transform& transBase, bool bCheckLimits)
{
    Transform tbase = _veclinks.front()->GetTransform().inverse() * transBase;
    _veclinks.front()->SetTransform(transBase);

    for(size_t i = 1; i < _veclinks.size(); ++i) {
        if( _veclinks[i]->IsStatic() ) {
            // if static and trans is valid, then apply the relative transformation to the link
            _veclinks[i]->SetTransform(_veclinks[i]->GetTransform()*tbase);
        }
        else
            _veclinks[i]->userdata = 0;
    }

    SetJointValues(vJointValues,bCheckLimits);

    // some links might not be connected to any joints. In this case, transform them by tbase
    for(size_t i = 1; i < _veclinks.size(); ++i) {
        if( _veclinks[i]->userdata == 0 ) {
            _veclinks[i]->SetTransform(_veclinks[i]->GetTransform()*tbase);
        }
    }
}

void KinBody::SetJointValues(const std::vector<dReal>& vJointValues, bool bCheckLimits)
{
    if( (int)vJointValues.size() != GetDOF() )
        throw openrave_exception(str(boost::format("dof not equal %d!=%d")%vJointValues.size()%GetDOF()),ORE_InvalidArguments);

    const dReal* pJointValues = &vJointValues[0];
    if( bCheckLimits ) {
        _vTempJoints.resize(GetDOF());

        dReal* ptempjoints = &_vTempJoints[0];

        // check the limits
        vector<int>::const_iterator itindex = _vecJointIndices.begin();

        vector<dReal> upperlim, lowerlim;
        FOREACHC(it, _vecjoints) {
            const dReal* p = pJointValues+*itindex++;
            assert( (*it)->GetDOF() <= 3 );
            (*it)->GetLimits(lowerlim, upperlim);
            if( (*it)->GetType() == Joint::JointSpherical ) {
                dReal fcurang = fmodf(RaveSqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]),2*PI);
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
                    if( p[i] < lowerlim[i] ) *ptempjoints++ = lowerlim[i];
                    else if( p[i] > upperlim[i] ) *ptempjoints++ = upperlim[i];
                    else *ptempjoints++ = p[i];
                }
            }
        }

        pJointValues = &_vTempJoints[0];
    }

    // set the first body and all static bodies to computed
    _veclinks[0]->userdata = 1;
    int numleft = (int)_veclinks.size()-1;

    for(size_t i = 1; i < _veclinks.size(); ++i) {
        if( _veclinks[i]->IsStatic() ) {
            numleft--;
            _veclinks[i]->userdata = 1;
        }
        else _veclinks[i]->userdata = 0;
    }

    dReal dummyvalues[3]; // dummy values for a joint
    Vector axis, anchor;
    Transform trans;
    Transform identity;
    vector<JointPtr>* alljoints[2] = {&_vecjoints, &_vecPassiveJoints};
    vector<dReal> vjointang;

    vector< vector<dReal> > vPassiveJointValues(_vecPassiveJoints.size());
    for(size_t i = 0; i < vPassiveJointValues.size(); ++i)
        _vecPassiveJoints[i]->GetValues(vPassiveJointValues[i]);

    while(numleft > 0) {
        int org = numleft;

        // iterate through the two sets of joints: active and passive
        // for active joints, read the angle from pJointValues, for passive joints
        // check if they have corresponding mimic joints, if they do, take those angles
        // otherwise set to 0
        vector<int>::const_iterator itindex = _vecJointIndices.begin();
        for(size_t j = 0; j < 2; ++j) {
            
            FOREACH(itjoint, *alljoints[j]) {
                const dReal* pvalues=NULL;
                
                if( itindex != _vecJointIndices.end() ) pvalues = pJointValues+*itindex++;
                else {
                    assert(j == 1); // passive
                    if( (*itjoint)->GetMimicJointIndex() >= 0 ) {
                        int mimicindex = (*itjoint)->GetMimicJointIndex();
                        int jointsize = mimicindex < (int)_vecJointIndices.size()-1 ? _vecJointIndices[mimicindex+1] : GetDOF();
                        pvalues = pJointValues+_vecJointIndices[mimicindex];
                        jointsize -= _vecJointIndices[mimicindex];

                        for(int i = 0; i < jointsize; ++i)
                            dummyvalues[i] = pvalues[i] * (*itjoint)->GetMimicCoeffs()[0] + (*itjoint)->GetMimicCoeffs()[1];
                        pvalues = dummyvalues;
                    }
                    else {
                        if( alljoints[j] == &_vecPassiveJoints ) {
                            pvalues = &vPassiveJointValues[itjoint-_vecPassiveJoints.begin()][0];
                        }
                        else {
                            dummyvalues[0] = dummyvalues[1] = dummyvalues[2] = 0;
                            pvalues = dummyvalues;
                            (*itjoint)->bodies[0]->userdata = 1;
                            numleft--;
                            continue;
                        }
                    }
                }

                LinkPtr* bodies = (*itjoint)->bodies;

                // make sure there is no wrap around for limits close to pi
                vjointang.resize((*itjoint)->GetDOF());
                for(int iang = 0; iang < (*itjoint)->GetDOF(); ++iang) { 
                    vjointang[iang] = pvalues[iang]-(*itjoint)->GetOffset();
                    if( (*itjoint)->GetType() != Joint::JointSlider && (*itjoint)->GetType() != Joint::JointSpherical ) {
                        if( vjointang[iang] < -PI+0.001f ) vjointang[iang] = -PI+0.001f;
                        else if( vjointang[iang] > PI-0.001f ) vjointang[iang] = PI-0.001f;
                    }
                }

                if( !!bodies[0] && !!bodies[1] && !bodies[1]->IsStatic()) {
                    if( bodies[0]->userdata ) {
                        if( !bodies[1]->userdata ) {
                            Transform tjoint;
                            axis = (*itjoint)->vAxes[0];

                            // init 1 from 0
                            switch((*itjoint)->GetType()) {
                            case Joint::JointHinge:
                                tjoint.rotfromaxisangle(axis, -vjointang[0]);
                                break;
                            case Joint::JointHinge2: {
                                Transform tfirst;
                                tfirst.rotfromaxisangle((*itjoint)->vAxes[0], -vjointang[0]);
                                Transform tsecond;
                                tsecond.rotfromaxisangle(tfirst.rotate((*itjoint)->vAxes[1]), -vjointang[1]);
                                tjoint = tsecond * tfirst;
                                break;
                            }
                            case Joint::JointSlider:
                                tjoint.trans = -axis * vjointang[0];
                                break;
                            case Joint::JointSpherical: {
                                dReal fang = vjointang[0]*vjointang[0]+vjointang[1]*vjointang[1]+vjointang[2]*vjointang[2];
                                if( fang > 1e-10 ) {
                                    fang = RaveSqrt(fang);
                                    dReal fiang = 1/fang;
                                    tjoint.rotfromaxisangle(Vector(vjointang[0]*fiang,vjointang[1]*fiang,vjointang[2]*fiang),fang);
                                }
                                break;
                            }
                            default:
                                RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                                break;
                            }
                            
                            trans = bodies[0]->GetTransform() * (*itjoint)->tLeft * tjoint * (*itjoint)->tRight;
                            bodies[1]->SetTransform(trans);

                            bodies[1]->userdata = 1;
                            numleft--;
                        }
                    }
                    else if( bodies[1]->userdata ) {
                        Transform tjoint;
                        axis = (*itjoint)->vAxes[0];

                        // init 1 from 0
                        switch((*itjoint)->GetType()) {
                        case Joint::JointHinge:
                            tjoint.rotfromaxisangle(axis, vjointang[0]);
                            break;
                        case Joint::JointHinge2: {
                            Transform tfirst;
                            tfirst.rotfromaxisangle((*itjoint)->vAxes[0], vjointang[0]);
                            Transform tsecond;
                            tsecond.rotfromaxisangle(tfirst.rotate((*itjoint)->vAxes[1]), vjointang[1]);
                            tjoint = tsecond * tfirst;
                            break;
                        }
                        case Joint::JointSlider:
                            tjoint.trans = axis * vjointang[0];
                            break;
                        case Joint::JointSpherical: {
                            dReal fang = vjointang[0]*vjointang[0]+vjointang[1]*vjointang[1]+vjointang[2]*vjointang[2];
                            if( fang > 1e-10 ) {
                                fang = RaveSqrt(fang);
                                dReal fiang = 1/fang;
                                tjoint.rotfromaxisangle(Vector(vjointang[0]*fiang,vjointang[1]*fiang,vjointang[2]*fiang),-fang);
                            }
                            break;
                        }
                        default:
                            RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                            break;
                        }

                        trans = bodies[1]->GetTransform() * (*itjoint)->tinvRight * tjoint * (*itjoint)->tinvLeft;
                        bodies[0]->SetTransform(trans);
                        bodies[0]->userdata = 1;
                        numleft--;
                    }
                }
                else if( !!bodies[0] && !bodies[0]->userdata ) {

                    Transform tjoint;
                    axis = (*itjoint)->vAxes[0];
                    
                    // joint attached to static environment (it will never be [1])
                    switch((*itjoint)->GetType()) {
                    case Joint::JointHinge:
                        tjoint.rotfromaxisangle(axis, -vjointang[0]);
                        break;
                    case Joint::JointHinge2: {
                        Transform tfirst;
                        tfirst.rotfromaxisangle((*itjoint)->vAxes[0], -vjointang[0]);
                        Transform tsecond;
                        tsecond.rotfromaxisangle(tfirst.rotate((*itjoint)->vAxes[1]), -vjointang[1]);
                        tjoint = tsecond * tfirst;
                        break;
                    }
                    case Joint::JointSlider:
                        tjoint.trans = -axis * vjointang[0];
                        break;
                    case Joint::JointSpherical: {
                        dReal fang = vjointang[0]*vjointang[0]+vjointang[1]*vjointang[1]+vjointang[2]*vjointang[2];
                        if( fang > 1e-10 ) {
                            fang = RaveSqrt(fang);
                            dReal fiang = 1/fang;
                            tjoint.rotfromaxisangle(Vector(vjointang[0]*fiang,vjointang[1]*fiang,vjointang[2]*fiang),fang);
                        }
                        break;
                    }
                    default:
                        RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                        break;
                    }
                    
                    trans = GetTransform() * (*itjoint)->tLeft * tjoint * (*itjoint)->tRight;
                    bodies[0]->SetTransform(trans);
                    bodies[0]->userdata = 1;
                    numleft--;
                }
            }
        }

        // nothing changed so exit
        if( org == numleft )
            break;
    }
}

void KinBody::SetJointVelocities(const std::vector<dReal>& pJointVelocities)
{
    Vector linearvel, angularvel;
    GetVelocity(linearvel, angularvel);
    GetEnv()->GetPhysicsEngine()->SetBodyVelocity(shared_kinbody(), linearvel, angularvel, pJointVelocities);
}

KinBody::LinkPtr KinBody::GetLink(const std::string& linkname) const
{
    for(std::vector<LinkPtr>::const_iterator it = _veclinks.begin(); it != _veclinks.end(); ++it) {
        if ( (*it)->GetName() == linkname )
            return *it;
    }
    RAVELOG_VERBOSEA("Link::GetLink - Error Unknown Link %s\n",linkname.c_str());
    return LinkPtr();
}

//! return a pointer to the joint with the given name, else -1
int KinBody::GetJointIndex(const std::string& jointname) const
{
    int index = 0;
    FOREACHC(it,_vecjoints) {
        if ((*it)->GetName() == jointname )
            return index;
        ++index;
    }
    return -1;
}

KinBody::JointPtr KinBody::GetJoint(const std::string& jointname) const
{
    FOREACHC(it,_vecjoints) {
        if ((*it)->GetName() == jointname )
            return *it;
    }
    return JointPtr();
}

void KinBody::CalculateJacobian(int index, const Vector& trans, vector<dReal>& vJacobian) const
{
    if( index < 0 || index >= (int)_veclinks.size() )
        throw openrave_exception(str(boost::format("bad index %d")%index),ORE_InvalidArguments);

    vJacobian.resize(GetDOF()*3);
    if( GetDOF() == 0 )
        return;
    dReal* pfJacobian = &vJacobian[0];

    //Vector trans = _veclinks[index]->GetTransform() * offset;
    Vector v, anchor, axis;

    int jointindex = 0;
    FOREACHC(itjoint, _vecjoints) {
        char affect = DoesAffect(jointindex, index);
        if( affect == 0 ) {
            pfJacobian[0] = pfJacobian[GetDOF()] = pfJacobian[2*GetDOF()] = 0;
            ++jointindex;
            ++pfJacobian;
            continue;
        }

        switch((*itjoint)->GetType()) {
        case Joint::JointHinge:
            cross3(v, (*itjoint)->GetAxis(0), (*itjoint)->GetAnchor()-trans);
            break;
        case Joint::JointSlider:
            v = -(*itjoint)->GetAxis(0);
            break;
        default:
            RAVELOG_WARNA("CalculateJacobian joint %d not supported\n", (*itjoint)->GetType());
            v = Vector(0,0,0);
            break;
        }

        pfJacobian[0] = v.x;
        pfJacobian[GetDOF()] = v.y;
        pfJacobian[GetDOF()*2] = v.z;

        ++jointindex;
        pfJacobian += (*itjoint)->GetDOF();
    }
}

void KinBody::CalculateRotationJacobian(int index, const Vector& q, vector<dReal>& vJacobian) const
{
    if( index < 0 || index >= (int)_veclinks.size() )
        throw openrave_exception(str(boost::format("bad index %d")%index),ORE_InvalidArguments);

    vJacobian.resize(GetDOF()*4);
    if( GetDOF() == 0 )
        return;
    dReal* pfJacobian = &vJacobian[0];

    //Vector trans = _veclinks[index]->GetTransform() * offset;
    Vector v, anchor, axis;

    int jointindex = 0;
    FOREACHC(itjoint, _vecjoints) {
        char affect = DoesAffect(jointindex, index);
        if( affect == 0 ) {
            pfJacobian[0] = pfJacobian[GetDOF()] = pfJacobian[2*GetDOF()] = pfJacobian[3*GetDOF()] = 0;
            ++jointindex;
            ++pfJacobian;
            continue;
        }

        switch((*itjoint)->GetType()) {
        case Joint::JointHinge:
            v = (*itjoint)->GetAxis(0);
            break;
        case Joint::JointSlider:
            v = Vector(0,0,0);
            break;
        default:
            RAVELOG_WARNA("CalculateRotationJacobian joint %d not supported\n", (*itjoint)->GetType());
            v = Vector(0,0,0);
            break;
        }

        pfJacobian[0] =            -q.y*v.x - q.z*v.y - q.w*v.z;
        pfJacobian[GetDOF()] =      q.x*v.x - q.z*v.z + q.w*v.y;
        pfJacobian[GetDOF()*2] =    q.x*v.y + q.y*v.z - q.w*v.x;
        pfJacobian[GetDOF()*3] =    q.x*v.z - q.y*v.y + q.z*v.x;

        ++jointindex;
        pfJacobian += (*itjoint)->GetDOF();
    }
}

void KinBody::CalculateAngularVelocityJacobian(int index, std::vector<dReal>& vJacobian) const
{
    if( index < 0 || index >= (int)_veclinks.size() )
        throw openrave_exception(str(boost::format("bad index %d")%index),ORE_InvalidArguments);

    vJacobian.resize(GetDOF()*3);
    if( GetDOF() == 0 )
        return;
    dReal* pfJacobian = &vJacobian[0];

    //Vector trans = _veclinks[index]->GetTransform() * offset;
    Vector v, anchor, axis;

    int jointindex = 0;
    FOREACHC(itjoint, _vecjoints) {
        char affect = DoesAffect(jointindex, index);
        if( affect == 0 ) {
            pfJacobian[0] = pfJacobian[GetDOF()] = pfJacobian[2*GetDOF()] = 0;
            ++jointindex;
            ++pfJacobian;
            continue;
        }

        switch((*itjoint)->GetType()) {
        case Joint::JointHinge:
            v = (*itjoint)->GetAxis(0);
            break;
        case Joint::JointSlider:
            v = Vector(0,0,0);
            break;
        default:
            RAVELOG_WARNA("CalculateAngularVelocityJacobian joint %d not supported\n", (*itjoint)->GetType());
            v = Vector(0,0,0);
            break;
        }

        pfJacobian[0] =            v.x;
        pfJacobian[GetDOF()] =     v.y;
        pfJacobian[GetDOF()*2] =   v.z;

        ++jointindex;
        pfJacobian += (*itjoint)->GetDOF();
    }
}

bool KinBody::CheckSelfCollision(CollisionReportPtr report) const
{
    return GetEnv()->GetCollisionChecker()->CheckSelfCollision(shared_kinbody_const(), report);
}

void KinBody::ComputeJointHierarchy()
{
    _bHierarchyComputed = true;
    _vecJointHierarchy.resize(_vecjoints.size()*_veclinks.size());
    if( _vecJointHierarchy.size() == 0 )
        return;

    memset(&_vecJointHierarchy[0], 0, _vecJointHierarchy.size() * sizeof(_vecJointHierarchy[0]));
    
    for(size_t i = 0; i < _veclinks.size(); ++i) {
        _veclinks[i]->userdata = _veclinks[i]->IsStatic();
    }
    _veclinks[0]->userdata = 1; // always set the first to be a root

    typedef pair<JointPtr, int> JOINTPAIR;
    list<JOINTPAIR> ljoints;
    for(int i = 0; i < (int)_vecjoints.size(); ++i) {
        ljoints.push_back(JOINTPAIR(_vecjoints[i], i));
    }
    for(int i = 0; i < (int)_vecPassiveJoints.size(); ++i)
        ljoints.push_back(JOINTPAIR(_vecPassiveJoints[i], -1));

    list<JOINTPAIR>::iterator itjoint;

    // iterate through all the joints and keep track of which joint affects which link
    while(ljoints.size() > 0) {

        int cursize = ljoints.size();

        itjoint = ljoints.begin();
        while(itjoint != ljoints.end()) {
            
            bool bDelete = true;
            bool bIsPassive = itjoint->second < 0;
            LinkPtr* bodies = itjoint->first->bodies;
            if( bIsPassive && itjoint->first->GetMimicJointIndex() < 0 ) {
                // is not part of the hierarchy, but still used to join links
                if( !!bodies[0] ) {
                    if( !!bodies[1] ) {
                        if( bodies[0]->userdata ) {
                            bodies[1]->userdata = 1;
                            int srcindex = bodies[0]->GetIndex();
                            int dstindex = bodies[1]->GetIndex();
                            // copy the data from bodies[0]
                            for(int j = 0; j < (int)_vecjoints.size(); ++j) {
                                _vecJointHierarchy[j*_veclinks.size()+dstindex] = _vecJointHierarchy[j*_veclinks.size()+srcindex];
                            }
                        }
                        else if( bodies[1]->userdata ) {
                            bodies[0]->userdata = 1;
                            int srcindex = bodies[1]->GetIndex();
                            int dstindex = bodies[0]->GetIndex();
                            // copy the data from bodies[1]
                            for(int j = 0; j < (int)_vecjoints.size(); ++j) {
                                _vecJointHierarchy[j*_veclinks.size()+dstindex] = _vecJointHierarchy[j*_veclinks.size()+srcindex];
                            }
                        }
                        else
                            bDelete = false;
                    }
                    else
                        bodies[0]->userdata = 1;
                }

                if( bDelete ) itjoint = ljoints.erase(itjoint);
                else ++itjoint;
                continue;
            }

            char* pvalues = &_vecJointHierarchy[!bIsPassive ? itjoint->second*_veclinks.size() : itjoint->first->GetMimicJointIndex() * _veclinks.size()];

            if( !!bodies[0] ) {
                if( !!bodies[1] ) {

                    if( bodies[0]->userdata ) {
                        
                        int srcindex = bodies[0]->GetIndex();
                        int dstindex = bodies[1]->GetIndex();
                    
                        if( bodies[1]->userdata ) {
                            assert( pvalues[dstindex] >= 0 );
                        }
                        else {
                            // copy the data from bodies[0]
                            for(int j = 0; j < (int)_vecjoints.size(); ++j) {
                                _vecJointHierarchy[j*_veclinks.size()+dstindex] = _vecJointHierarchy[j*_veclinks.size()+srcindex];
                            }
                        }

                        bodies[1]->userdata = 1;
                        pvalues[dstindex] = 1;
                    }
                    else {

                        if( bodies[1]->userdata ) {
                            // copy the data from bodies[1]
                            int srcindex = bodies[1]->GetIndex();
                            int dstindex = bodies[0]->GetIndex();
                            
                            for(int j = 0; j < (int)_vecjoints.size(); ++j) {
                                _vecJointHierarchy[j*_veclinks.size()+dstindex] = _vecJointHierarchy[j*_veclinks.size()+srcindex];
                            }

                            bodies[0]->userdata = 1;
                            pvalues[dstindex] = -1;
                        }
                        else bDelete = false; // skip
                    }
                }
                else {
                    bodies[0]->userdata = 1;
                    pvalues[bodies[0]->GetIndex()] = 1;
                }
            }

            if( bDelete ) itjoint = ljoints.erase(itjoint);
            else ++itjoint;
        }

        if( cursize == (int)ljoints.size() ) {
            RAVELOG_ERRORA("Cannot compute joint hierarchy for number of joints %"PRIdS"! Part of robot might not be moveable\n", _vecjoints.size());
            break;
        }
    }

    if( ljoints.size() == 0 )
        RAVELOG_DEBUGA("Successfully computed joint hierarchy for number of joints %"PRIdS"!\n", _vecjoints.size());

    // create the adjacency list
    vector<dReal> prevvalues; GetJointValues(prevvalues);
    vector<dReal> vzero(GetDOF());
    SetJointValues(vzero);
    _setAdjacentLinks.clear();
    _setNonAdjacentLinks.clear();  

    FOREACH(itj, _vecjoints) {
        if( !!(*itj)->bodies[0] && !!(*itj)->bodies[1] ) {
            int ind0 = (*itj)->bodies[0]->GetIndex();
            int ind1 = (*itj)->bodies[1]->GetIndex();
            if( ind1 < ind0 ) _setAdjacentLinks.insert(ind1|(ind0<<16));
            else _setAdjacentLinks.insert(ind0|(ind1<<16));
        }
    }
    FOREACH(itj, _vecPassiveJoints) {
        if( !!(*itj)->bodies[0] && !!(*itj)->bodies[1] ) {
            int ind0 = (*itj)->bodies[0]->GetIndex();
            int ind1 = (*itj)->bodies[1]->GetIndex();
            if( ind1 < ind0 ) _setAdjacentLinks.insert(ind1|(ind0<<16));
            else _setAdjacentLinks.insert(ind0|(ind1<<16));
        }
    }

    FOREACH(itadj, _vForcedAdjacentLinks) {
        LinkPtr pl0 = GetLink(itadj->first.c_str());
        LinkPtr pl1 = GetLink(itadj->second.c_str());
        if( !!pl0 && !!pl1 ) {
            int ind0 = pl0->GetIndex();
            int ind1 = pl1->GetIndex();
            if( ind1 < ind0 ) _setAdjacentLinks.insert(ind1|(ind0<<16));
            else _setAdjacentLinks.insert(ind0|(ind1<<16));
        }
    }

    // set a default pose
    for(size_t i = 0; i < _veclinks.size(); ++i) {
        for(size_t j = i+1; j < _veclinks.size(); ++j) {
            if( _setAdjacentLinks.find(i|(j<<16)) == _setAdjacentLinks.end() &&
                !GetEnv()->CheckCollision(LinkConstPtr(_veclinks[i]), LinkConstPtr(_veclinks[j])) )
                _setNonAdjacentLinks.insert(i|(j<<16));
        }
    }

    //RAVEPRINT(L"nonadj: %"PRIdS"\n", _setNonAdjacentLinks.size());
    // save the forward kinematics
    string filename = GetEnv()->GetHomeDirectory() + string("/fk_") + GetName();
    ofstream f(filename.c_str());
    WriteForwardKinematics(f);
    SetJointValues(prevvalues, true);
}

struct LINKTRANSINFO
{
    LINKTRANSINFO() : bRecord(false) { vJointCoeffs.resize(2); vJointCoeffs[0] = 1; vJointCoeffs[1] = 0; }
    int linkcur;
    int linkbase;
    int jointindex;
    Vector vjointaxis;
    vector<dReal> vJointCoeffs; ///< use (jointangle*coeffs[0]+coeffs[1]) for the rotation around the axis
    Transform Tleft, Tright;
    string type; ///< joint type
    bool bRecord;
};

void KinBody::WriteForwardKinematics(std::ostream& f)
{
    // set the first body and all static bodies to computed
    _veclinks[0]->userdata = 1;
    int numleft = (int)_veclinks.size()-1;

    for(size_t i = 1; i < _veclinks.size(); ++i) {
        if( _veclinks[i]->IsStatic() ) {
            numleft--;
            _veclinks[i]->userdata = 1;
        }
        else _veclinks[i]->userdata = 0;
    }

    list<LINKTRANSINFO> listlinkinfo;
    LINKTRANSINFO info;

    // same logic loop as SetJointValues
    Vector anchor;
    Transform trans;
    Transform identity;
    vector<JointPtr>* alljoints[2] = {&_vecjoints, &_vecPassiveJoints};

    while(numleft > 0) {
        int org = numleft;

        // iterate through the two sets of joints: active and passive
        // for active joints, read the angle from pJointValues, for passive joints
        // check if they have corresponding mimic joints, if they do, take those angles
        // otherwise set to 0
        for(size_t j = 0; j < 2; ++j) {
            
            FOREACH(itjoint, *alljoints[j]) {

                int jointindex = 0;
                if( (*itjoint)->GetMimicJointIndex() >= 0 ) {
                    jointindex = (*itjoint)->GetMimicJointIndex();
                    info.vJointCoeffs = (*itjoint)->GetMimicCoeffs();
                }
                else if( j == 0 ) {
                    jointindex = (int)(itjoint-alljoints[j]->begin());
                    info.vJointCoeffs[0] = 1; info.vJointCoeffs[1] = 0;
                }

                LinkPtr* bodies = (*itjoint)->bodies;

                if( !!bodies[0] && !!bodies[1] && !bodies[1]->IsStatic()) {
                    if( bodies[0]->userdata ) {
                        if( !bodies[1]->userdata ) {
                            // init 1 from 0
                            switch( (*itjoint)->GetType() ) {
                            case Joint::JointHinge:
                            case Joint::JointSlider: {
                                info.type = (*itjoint)->GetType() == Joint::JointHinge ? "hinge" : "slider";
                                info.vjointaxis = -(*itjoint)->vAxes[0];
                                info.Tright = (*itjoint)->tRight;
                                Transform tjoint;
                                if( (*itjoint)->GetType() == Joint::JointHinge )
                                    tjoint.rotfromaxisangle(info.vjointaxis,-(*itjoint)->offset);
                                else
                                    tjoint.trans = -info.vjointaxis*(*itjoint)->offset;
                                info.Tleft = (*itjoint)->tLeft*tjoint;
                                info.linkcur = bodies[1]->GetIndex();
                                info.linkbase = bodies[0]->GetIndex();
                                info.jointindex = jointindex;
                                info.bRecord = true;
                                listlinkinfo.push_back(info);
                                break;
                            }
                            case Joint::JointSpherical: {
                                info.type = "spherical";
                                info.vjointaxis = Vector(1,1,1);
                                info.Tright = (*itjoint)->tRight;
                                info.Tleft = (*itjoint)->tLeft;
                                info.linkcur = bodies[1]->GetIndex();
                                info.linkbase = bodies[0]->GetIndex();
                                info.jointindex = jointindex;
                                info.bRecord = true;
                                listlinkinfo.push_back(info);
                                break;
                            }
                            default:
                                RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                                break;
                            }

                            bodies[1]->userdata = 1;
                            numleft--;
                        }
                    }
                    else if( bodies[1]->userdata ) {
                        // init 1 from 0
                        switch( (*itjoint)->GetType() ) {
                        case Joint::JointHinge:
                        case Joint::JointSlider: {
                            info.type = (*itjoint)->GetType() == Joint::JointHinge ? "hinge" : "slider";
                            info.vjointaxis = -(*itjoint)->vAxes[0];
                            info.Tright = (*itjoint)->tinvLeft;
                            Transform tjoint;
                            if( (*itjoint)->GetType() == Joint::JointHinge )
                                tjoint.rotfromaxisangle(info.vjointaxis,(*itjoint)->offset);
                            else
                                tjoint.trans = info.vjointaxis*(*itjoint)->offset;
                            info.Tleft = (*itjoint)->tinvRight*tjoint;
                            info.linkcur = bodies[0]->GetIndex();
                            info.linkbase = bodies[1]->GetIndex();
                            info.jointindex = jointindex;
                            info.bRecord = true;
                            listlinkinfo.push_back(info);
                            break;
                        }
                        case Joint::JointSpherical: {
                            info.type = "spherical";
                            info.vjointaxis = Vector(-1,-1,-1);
                            info.Tright = (*itjoint)->tinvLeft;
                            info.Tleft = (*itjoint)->tinvRight;
                            info.linkcur = bodies[0]->GetIndex();
                            info.linkbase = bodies[1]->GetIndex();
                            info.jointindex = jointindex;
                            info.bRecord = true;
                            listlinkinfo.push_back(info);
                            break;
                        }
                        default:
                            RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                            break;
                        }

                        bodies[0]->userdata = 1;
                        numleft--;
                    }
                }
                else if( !!bodies[0] && !bodies[0]->userdata ) {

                    // joint attached to static environment (it will never be [1])
                    switch( (*itjoint)->GetType() ) {
                    case Joint::JointHinge:
                    case Joint::JointSlider: {
                        // reset body
                        LINKTRANSINFO info;
                        info.type = (*itjoint)->GetType() == Joint::JointHinge ? "hinge" : "slider";
                        info.vjointaxis = -(*itjoint)->vAxes[0];
                        info.linkcur = bodies[0]->GetIndex();
                        info.linkbase = -1;
                        info.jointindex = jointindex;
                        info.bRecord =true;
                        
                        info.Tright = (*itjoint)->tRight;
                        info.Tleft = (*itjoint)->tLeft;
                        listlinkinfo.push_back(info);
                        
                        break;
                    }
                    case Joint::JointSpherical: {
                        // reset body
                        LINKTRANSINFO info;
                        info.type = "spherical";
                        info.vjointaxis = Vector(1,1,1);
                        info.linkcur = bodies[0]->GetIndex();
                        info.linkbase = -1;
                        info.jointindex = jointindex;
                        info.bRecord =true;
                        info.Tright = (*itjoint)->tRight;
                        info.Tleft = (*itjoint)->tLeft;
                        listlinkinfo.push_back(info);
                        
                        break;
                    }
                    default:
                        RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                        break;
                    }

                    bodies[0]->userdata = 1;
                    numleft--;
                }
            }
        }

        // nothing changed so exit
        if( org == numleft )
            break;
    }

    // write out each of the links
    f << listlinkinfo.size() << endl;
    list<LINKTRANSINFO>::iterator it;
    FORIT(it, listlinkinfo) {
        if( !it->bRecord )
            continue;

        f << it->type << " " << it->linkcur << " " << it->linkbase << " " << it->jointindex << " "
          << it->vjointaxis.x << " " << it->vjointaxis.y << " " << it->vjointaxis.z << " "
          << it->vJointCoeffs[0] << " " << it->vJointCoeffs[1] <<  endl << endl;
        TransformMatrix m(it->Tleft);
        for(int j = 0; j < 3; ++j) {
            for(int k = 0; k < 3; ++k)
                f << m.m[4*j+k] << " ";
            f << m.trans[j];
            f << endl;
        }
        f << endl;
        m = TransformMatrix(it->Tright);
        for(int j = 0; j < 3; ++j) {
            for(int k = 0; k < 3; ++k)
                f << m.m[4*j+k] << " ";
            f << m.trans[j];
            f << endl;
        }
        f << endl << endl;
    }
}

void KinBody::AttachBody(KinBodyPtr pbody)
{
    _setAttachedBodies.insert(pbody);
    pbody->_setAttachedBodies.insert(shared_kinbody());
}

void KinBody::RemoveBody(KinBodyPtr pbody)
{
    if( !pbody ) {
        FOREACH(it, _setAttachedBodies) {
            KinBodyPtr p(*it);
            if( !!p )
                p->_setAttachedBodies.erase(KinBodyWeakPtr(shared_kinbody()));
        }
        _setAttachedBodies.clear();
    }
    else {
        _setAttachedBodies.erase(pbody);
        pbody->_setAttachedBodies.erase(KinBodyWeakPtr(shared_kinbody()));
    }
}

bool KinBody::IsAttached(KinBodyConstPtr pbody) const
{
    return _setAttachedBodies.find(boost::const_pointer_cast<KinBody>(pbody)) != _setAttachedBodies.end();
}

void KinBody::GetAttached(std::vector<KinBodyPtr>& attached) const
{
    attached.resize(0);
    FOREACHC(it,_setAttachedBodies) {
        KinBodyPtr p(*it);
        if( !!p )
            attached.push_back(p);
        else
            RAVELOG_DEBUGA("attached body is invalid\n");
    }
}

void KinBody::Enable(bool bEnable)
{
    GetEnv()->GetCollisionChecker()->Enable(shared_kinbody(),bEnable);
    FOREACH(it, _veclinks)
        (*it)->_bIsEnabled = bEnable;
}

bool KinBody::IsEnabled() const
{
    // enable/disable everything
    FOREACHC(it, _veclinks) {
        if((*it)->IsEnabled())
            return true;
    }

    return false;
}

int KinBody::GetNetworkId() const
{
    assert( networkid != 0 );
    return networkid;
}

char KinBody::DoesAffect(int jointindex, int linkindex ) const
{
    if( !_bHierarchyComputed ) {
        RAVELOG_WARNA("DoesAffect: joint hierarchy needs to be computed\n");
        return 0;
    }
    return _vecJointHierarchy[jointindex*_veclinks.size()+linkindex];
}

const std::set<int>& KinBody::GetNonAdjacentLinks() const
{
    if( !_bHierarchyComputed ) {
        RAVELOG_WARNA("GetNonAdjacentLinks: joint hierarchy needs to be computed\n");
    }

    return _setNonAdjacentLinks;
}

const std::set<int>& KinBody::GetAdjacentLinks() const
{
    if( !_bHierarchyComputed ) {
        RAVELOG_WARNA("GetAdjacentLinks: joint hierarchy needs to be computed\n");
    }

    return _setAdjacentLinks;
}

bool KinBody::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    if( !InterfaceBase::Clone(preference,cloningoptions) )
        return false;

    KinBodyConstPtr r = boost::static_pointer_cast<KinBody const>(preference);

    name = r->name;
    
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
        if( !!(*itjoint)->bodies[0] )
            pnewjoint->bodies[0] = _veclinks[(*itjoint)->bodies[0]->GetIndex()];
        if( !!(*itjoint)->bodies[1] )
            pnewjoint->bodies[1] = _veclinks[(*itjoint)->bodies[1]->GetIndex()];
        _vecjoints.push_back(pnewjoint);
    }

    _vecPassiveJoints.reserve(r->_vecPassiveJoints.size());
    FOREACHC(itjoint, r->_vecPassiveJoints) {
        JointPtr pnewjoint(new Joint(shared_kinbody()));
        *pnewjoint = **itjoint; // be careful of copying pointers!
        pnewjoint->_parent = shared_kinbody();
        if( !!(*itjoint)->bodies[0] )
            pnewjoint->bodies[0] = _veclinks[(*itjoint)->bodies[0]->GetIndex()];
        if( !!(*itjoint)->bodies[1] )
            pnewjoint->bodies[1] = _veclinks[(*itjoint)->bodies[1]->GetIndex()];
        _vecPassiveJoints.push_back(pnewjoint);
    }

    _vecJointIndices = r->_vecJointIndices;
    _vecJointWeights = r->_vecJointWeights;
    _vecJointHierarchy = r->_vecJointHierarchy;
    
    _setAdjacentLinks = r->_setAdjacentLinks;
    _setNonAdjacentLinks = r->_setNonAdjacentLinks;
    _vForcedAdjacentLinks = r->_vForcedAdjacentLinks;

    strXMLFilename = r->strXMLFilename;
    _vTempJoints = r->_vTempJoints;

    _nUpdateStampId++;
    return true;
}

void KinBody::ParametersChanged(int parameters)
{
    FOREACH(itfns,_listRegisteredCallbacks) {
        if( itfns->first & parameters )
            itfns->second();
    }
}

void KinBody::__erase_iterator(KinBodyWeakPtr pweakbody, std::list<std::pair<int,boost::function<void()> > >::iterator* pit)
{
    if( !!pit ) {
        KinBodyPtr pbody = pweakbody.lock();
        if( !!pbody )
            pbody->_listRegisteredCallbacks.erase(*pit);
        delete pit;
    }
}

boost::shared_ptr<void> KinBody::RegisterChangeCallback(int properties, const boost::function<void()>& callback)
{
    return boost::shared_ptr<void>(new std::list<std::pair<int,boost::function<void()> > >::iterator(_listRegisteredCallbacks.insert(_listRegisteredCallbacks.end(),make_pair(properties,callback))), boost::bind(KinBody::__erase_iterator,KinBodyWeakPtr(shared_kinbody()), _1));
}

} // end namespace OpenRAVE
