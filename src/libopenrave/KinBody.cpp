// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

    int indices[] = {
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
    for(size_t i = 0; i < ARRAYSIZE(indices); i += 3 ) {
        v[0] = temp.vertices[indices[i]];
        v[1] = temp.vertices[indices[i+1]];
        v[2] = temp.vertices[indices[i+2]];
        if( dot3(v[0], (v[1]-v[0]).Cross(v[2]-v[0])) < 0 )
            swap(indices[i], indices[i+1]);
    }

    temp.indices.resize(ARRAYSIZE(indices));
    memcpy(&temp.indices[0], indices, sizeof(indices));

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

        for(size_t i = 0; i < ARRAYSIZE(indices); i += 3 ) {
            Vector v1 = v[indices[i]];
            Vector v2 = v[indices[i+1]];
            Vector v3 = v[indices[i+2]];
            Vector vtemp;
            if( dot3(v1, cross3(vtemp, v2-v1, v3-v1)) < 0 )
                swap(indices[i], indices[i+1]);
        }

        collisionmesh.vertices.resize(8);
        memcpy(&collisionmesh.vertices[0], v, sizeof(v));
        collisionmesh.indices.resize(ARRAYSIZE(indices));
        memcpy(&collisionmesh.indices[0], indices, sizeof(indices));
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
        RAVELOG(L"unrecognized geom type %d!\n", GetType());
        return false;
    }

    return true;
}

KinBody::Link::Link(KinBody* parent)
{
    _parent = parent;
    bStatic = false;
    userdata = 0;
    index = -1;
    _bIsEnabled = true;
}

KinBody::Link::~Link()
{
    RAVELOG_VERBOSEA("destroy body %S\n", GetName());
}

bool KinBody::Link::IsEnabled() const
{
    return _bIsEnabled;
}

void KinBody::Link::Enable(bool bEnable)
{
    GetParent()->GetEnv()->GetCollisionChecker()->EnableLink(this,bEnable);
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
    _parent->_nUpdateStampId++;
}

Transform KinBody::Link::GetTransform() const
{
    return _t;
}

void KinBody::Link::SetForce(const Vector& force, const Vector& pos, bool bAdd)
{
    assert( GetParent()->GetEnv() != NULL );
    GetParent()->GetEnv()->GetPhysicsEngine()->SetBodyForce(this, force, pos, bAdd);
}

void KinBody::Link::SetTorque(const Vector& torque, bool bAdd)
{
    assert( GetParent()->GetEnv() != NULL );
    GetParent()->GetEnv()->GetPhysicsEngine()->SetBodyTorque(this, torque, bAdd);
}

KinBody::Joint::Joint(KinBody* parent)
{
    assert( parent != NULL );
    _parent = parent;
    nMimicJointIndex = -1;
    fResolution = 0.02;
    fMaxVel = 1e5f;
    fMaxAccel = 1e5f;
    fMaxTorque = 1e5f;
    bodies[0] = bodies[1] = NULL;
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
    case JointSlider:
        return 1;
    case JointHinge2:
    case JointUniversal:
        return 2;
    default: 
        return 0;
    }
}

int KinBody::Joint::GetValues(dReal *pValues) const
{
    if( pValues != NULL ) {
        Transform tjoint;
        if ( bodies[1] != NULL && !bodies[1]->IsStatic() )
            tjoint = tinvLeft * bodies[0]->GetTransform().inverse() * bodies[1]->GetTransform() * tinvRight;
        else
            tjoint = tinvLeft * _parent->GetTransform().inverse() * bodies[0]->GetTransform() * tinvRight;
        
        switch(type) {
        case JointHinge:
            pValues[0] = offset-2.0f*RaveAtan2(tjoint.rot.y*vAxes[0].x+tjoint.rot.z*vAxes[0].y+tjoint.rot.w*vAxes[0].z, tjoint.rot.x);
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
                pValues[0] = -RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
                vec1 = (vAxes[0] - axis2cur.dot(vAxes[0])*axis2cur).normalize();
                vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
                vec3 = axis2cur; vec3.Cross(vec1);
                pValues[1] = -RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
			}
            break;
        case JointSlider:
            pValues[0] = offset-(tjoint.trans.x*vAxes[0].x+tjoint.trans.y*vAxes[0].y+tjoint.trans.z*vAxes[0].z);
            break;
        default:
            RAVELOG_WARNA("unknown joint type %d\n", type);
            if( GetDOF() > 0 )
                memset(pValues,0,sizeof(dReal)*GetDOF());
        }
    }

    return GetDOF();
}

Vector KinBody::Joint::GetAnchor() const
{
    if( bodies[0] == NULL )
        return vanchor;
    else if( bodies[1] != NULL && bodies[1]->IsStatic() )
        return bodies[1]->GetTransform() * vanchor;
    else
        return bodies[0]->GetTransform() * vanchor;
}

Vector KinBody::Joint::GetAxis(int iaxis ) const
{
    assert(iaxis >= 0 && iaxis < 3 );
    if( bodies[0] == NULL )
        return vAxes[iaxis];
    else if( bodies[1] != NULL && bodies[1]->IsStatic() )
        return bodies[1]->GetTransform().rotate(vAxes[iaxis]);
    else
        return bodies[0]->GetTransform().rotate(vAxes[iaxis]);
}

int KinBody::Joint::GetVelocities(dReal* pVelocities) const
{
    if( pVelocities != NULL ) {
        assert( _parent->GetEnv() != NULL );
        _parent->GetEnv()->GetPhysicsEngine()->GetJointVelocity(this, pVelocities);
    }
    
    return GetDOF();
}

int KinBody::Joint::GetLimits(dReal* pLowerLimit, dReal* pUpperLimit) const
{
    assert( (int)_vlowerlimit.size() == GetDOF() && (int)_vupperlimit.size() == GetDOF() );
    if( pLowerLimit != NULL )
        memcpy(pLowerLimit,&_vlowerlimit[0],GetDOF()*sizeof(dReal));
    if( pUpperLimit != NULL )
        memcpy(pUpperLimit,&_vupperlimit[0],GetDOF()*sizeof(dReal));
    return GetDOF();
}

void KinBody::Joint::AddTorque(const dReal* pTorques)
{
    assert( _parent->GetEnv() != NULL );
    _parent->GetEnv()->GetPhysicsEngine()->AddJointTorque(this, pTorques);
}

KinBody::KinBodyStateSaver::KinBodyStateSaver(KinBody* pbody) : _pbody(pbody)
{
    _pbody->GetBodyTransformations(_vtransPrev);
}

KinBody::KinBodyStateSaver::~KinBodyStateSaver()
{
    _pbody->SetBodyTransformations(_vtransPrev);
}

KinBody::KinBody(PluginType type, EnvironmentBase* penv) : InterfaceBase(type, penv)
{
    DestroyCallback = NULL;
    networkid = 0;
    
    _nUpdateStampId = 0;
    _pGuiData = _pCollisionData = _pPhysicsData = NULL;
}

KinBody::~KinBody()
{
    RAVELOG_VERBOSEA("destroying kinbody: %S\n", GetName());
    
    Destroy();

    if( DestroyCallback != NULL )
        DestroyCallback(GetEnv(), this);
}

void KinBody::Destroy()
{
    name.clear();
    FOREACH(itlink, _veclinks) delete *itlink;
    FOREACH(itjoint, _vecjoints) delete *itjoint;
    FOREACH(itjoint, _vecPassiveJoints) delete *itjoint;

    _vecjoints.clear();
    _vecPassiveJoints.clear();
    _veclinks.clear();
    _vecJointIndices.clear();
    _vecJointWeights.clear();

    _setAttachedBodies.clear();
}

bool KinBody::Init(const char* filename, const char**atts)
{
    // check if added
    FOREACHC(itbody, GetEnv()->GetBodies()) {
        if( *itbody == this ) {
            RAVELOG_ERRORA("KinBody::Init for %S, cannot Init a body while it is added to the environment, please remove it first\n", GetName());
            return false;
        }
    }

    SetGuiData(NULL);

    //Destroy(); // don't destroy! operation should append to current bodies

    bool bSuccess = GetEnv()->ReadKinBodyXML(this, filename, atts)==this;

    if( !bSuccess ) {
        Destroy();
        return false;
    }

    strXMLFilename = filename;
    return true;
}

bool KinBody::InitFromBoxes(const std::vector<AABB>& vaabbs, bool bDraw)
{
    // check if added
    FOREACHC(itbody, GetEnv()->GetBodies()) {
        if( *itbody == this ) {
            RAVELOG_ERRORA("KinBody::Init for %S, cannot Init a body while it is added to the environment, please remove it first\n", GetName());
            return false;
        }
    }

    Destroy();
    Link* plink = new Link(this);
    plink->index = 0;
    plink->name = L"base";
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

void KinBody::SetName(const wchar_t* pNewName)
{
    name = pNewName;
}

void KinBody::SetName(const char* pNewName)
{
    name.clear();
    if( pNewName == NULL )
        return;

    name = _ravembstowcs(pNewName);
}

void KinBody::SetJointTorques(const dReal* pValues, bool bAdd)
{
    if( !bAdd ) {
        FOREACH(itlink, _veclinks) {
            (*itlink)->SetForce(Vector(),Vector(),false);
            (*itlink)->SetTorque(Vector(),false);
        }
    }

    vector<int>::iterator itindex = _vecJointIndices.begin();
    FOREACH(it, _vecjoints) {
        /// check limits
        assert( pValues[0] >= -(*it)->fMaxTorque && pValues[0] <= (*it)->fMaxTorque );
        (*it)->AddTorque(pValues+*itindex++);
    }
}

void KinBody::GetJointValues(dReal *jointAngles) const
{
    assert( jointAngles != NULL );
    FOREACHC(it, _vecjoints)
        jointAngles += (*it)->GetValues(jointAngles);
}

void KinBody::GetJointValues(std::vector<dReal>& v) const
{
    v.resize(GetDOF());
    if( GetDOF() > 0 )
        GetJointValues(&v[0]);
}


void KinBody::GetJointVelocities(dReal* pRates) const
{
    assert( pRates != NULL );
    FOREACHC(it, _vecjoints)
        pRates += (*it)->GetVelocities(pRates);
}

void KinBody::GetJointLimits(dReal* pLowerLimit, dReal* pUpperLimit) const
{
    FOREACHC(it, _vecjoints) {
        int dof = (*it)->GetLimits(pLowerLimit, pUpperLimit);
        if( pLowerLimit != NULL )
            pLowerLimit += dof;
        if( pUpperLimit != NULL )
            pUpperLimit += dof;
    }
}

void KinBody::GetJointLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const
{
    vLowerLimit.resize(GetDOF());
    vUpperLimit.resize(GetDOF());
    GetJointLimits(&vLowerLimit[0], &vUpperLimit[0]);
}

void KinBody::GetJointMaxVel(dReal* pValues) const
{
    assert( pValues != NULL );

    FOREACHC(it, _vecjoints) {
        int num = (*it)->GetDOF();
        while(num-- > 0 )
            *pValues++ = (*it)->fMaxVel;
    }
}

void KinBody::GetJointMaxAccel(dReal* pValues) const
{
    assert( pValues != NULL );

    FOREACHC(it, _vecjoints) {
        int num = (*it)->GetDOF();
        while(num-- > 0 )
            *pValues++ = (*it)->fMaxAccel;
    }
}

void KinBody::GetJointMaxTorque(dReal* pValues) const
{
    assert( pValues != NULL );

    FOREACHC(it, _vecjoints) {
        int num = (*it)->GetDOF();
        while(num-- > 0 )
            *pValues++ = (*it)->fMaxTorque;
    }
}

void KinBody::GetJointResolutions(dReal* pValues) const
{
    assert( pValues != NULL );

    FOREACHC(it, _vecjoints) {
        int num = (*it)->GetDOF();
        while(num-- > 0 )
            *pValues++ = (*it)->fResolution;
    }
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
    GetEnv()->GetPhysicsEngine()->SetBodyVelocity(this, linearvel, angularvel, NULL);
}

void KinBody::GetVelocity(Vector& linearvel, Vector& angularvel) const
{
    GetEnv()->GetPhysicsEngine()->GetBodyVelocity(this, linearvel, angularvel, NULL);
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
    vector<Link*>::const_iterator itlink;
    
    for(it = vtrans.begin(), itlink = _veclinks.begin(); it != vtrans.end(); ++it, ++itlink)
        *it = (*itlink)->GetTransform();
}

KinBody::Joint* KinBody::GetJointFromDOFIndex(int dofindex) const
{
    int jointindex = 0;
    FOREACHC(it, _vecJointIndices) {
        if( dofindex <= *it )
            break;
        ++jointindex;
    }
    if( jointindex >= (int)_vecjoints.size() )
        return NULL;
    return _vecjoints[jointindex];
}

AABB KinBody::ComputeAABB() const
{
    if( _veclinks.size() == 0 )
        return AABB();
    
    Vector vmin, vmax;
    std::vector<Link*>::const_iterator itlink = _veclinks.begin();

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

dReal KinBody::ConfigDist(const dReal *q1) const
{
    dReal dist = 0;
    vector<Joint*>::const_iterator it;
    vector<dReal>::const_iterator itweight = _vecJointWeights.begin();

    GetJointValues(_vTempJoints);

    for(int i = 0; i < GetDOF(); ++i) {
        dReal s = _vTempJoints[i]-*q1++;
        dist += s * s * *itweight++;
    }

    return RaveSqrt(dist);
}

dReal KinBody::ConfigDist(const dReal *q1, const dReal *q2, int dof) const
{
    dReal dist = 0.0;
    for (size_t i = 0; i < min((size_t)dof, _vecJointWeights.size()); i++) {
        dist += _vecJointWeights[i] * (q2[i] - q1[i]) * (q2[i] - q1[i]);
    }
    return sqrtf(dist);
}

void KinBody::SetBodyTransformations(const std::vector<Transform>& vbodies)
{
    if( vbodies.size() != _veclinks.size() ) {
        RAVELOG_ERRORA("wrong number of transforms passed! %"PRIdS" != %"PRIdS"\n", vbodies.size(), _veclinks.size());
        return;
    }
    
    vector<Transform>::const_iterator it;
    vector<Link*>::iterator itlink;
    
    for(it = vbodies.begin(), itlink = _veclinks.begin(); it != vbodies.end(); ++it, ++itlink)
        (*itlink)->SetTransform(*it);

    _nUpdateStampId++;
}

void KinBody::SetJointValues(vector<Transform>* pvbodies, const Transform* ptrans, const dReal* _pJointValues, bool bCheckLimits)
{
    if( pvbodies != NULL ) pvbodies->resize(_veclinks.size());
    
    const dReal* pJointValues = _pJointValues;

    if( bCheckLimits ) {

        _vTempJoints.resize(GetDOF());

        dReal* ptempjoints = &_vTempJoints[0];

        // check the limits
        vector<int>::const_iterator itindex = _vecJointIndices.begin();

        dReal upperlim[3], lowerlim[3];
        FOREACHC(it, _vecjoints) {
            const dReal* p = pJointValues+*itindex++;
            assert( (*it)->GetDOF() <= 3 );
            (*it)->GetLimits(lowerlim, upperlim);
            for(int i = 0; i < (*it)->GetDOF(); ++i) {
                if( p[i] < lowerlim[i] ) *ptempjoints++ = lowerlim[i];
                else if( p[i] > upperlim[i] ) *ptempjoints++ = upperlim[i];
                else *ptempjoints++ = p[i];
            }
        }

        pJointValues = &_vTempJoints[0];
    }

    Transform tbase; // base transform of first link

    if( ptrans != NULL ) {
        tbase = _veclinks.front()->GetTransform().inverse() * *ptrans;
        if( pvbodies != NULL ) (*pvbodies)[0] = *ptrans;
        _veclinks.front()->SetTransform(*ptrans);
    }
    else {
        if( pvbodies != NULL )
            pvbodies->front() = _veclinks.front()->GetTransform();
    }

    // set the first body and all static bodies to computed
    _veclinks[0]->userdata = 1;
    int numleft = (int)_veclinks.size()-1;

    for(size_t i = 1; i < _veclinks.size(); ++i) {
        if( _veclinks[i]->IsStatic() ) {
            numleft--;
            _veclinks[i]->userdata = 1;

            if( ptrans != NULL ) {
                // if static and trans is valid, then apply the relative transformation to the link
                Transform tcur = _veclinks[i]->GetTransform();
                // might be wrong
                _veclinks[i]->SetTransform(tcur*tbase);
            }
            
            if( pvbodies != NULL ) {
                (*pvbodies)[i] = _veclinks[i]->GetTransform();
            }
        }
        else _veclinks[i]->userdata = 0;
    }

    dReal dummyvalues[3]; // dummy values for a joint
    Vector axis, anchor;
    Transform trans;
    Transform identity;
    vector<Joint*>* alljoints[2] = {&_vecjoints, &_vecPassiveJoints};

    while(numleft > 0) {
        int org = numleft;

        // iterate through the two sets of joints: active and passive
        // for active joints, read the angle from pJointValues, for passive joints
        // check if they have corresponding mimic joints, if they do, take those angles
        // otherwise set to 0
        vector<int>::const_iterator itindex = _vecJointIndices.begin();
        for(size_t j = 0; j < ARRAYSIZE(alljoints); ++j) {
            
            FOREACH(itjoint, *alljoints[j]) {
                const dReal* pvalues;
                
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
                        dummyvalues[0] = dummyvalues[1] = dummyvalues[2] = 0;
                        pvalues = dummyvalues;
                        (*itjoint)->bodies[0]->userdata = 1;
                        numleft--;
                        continue;
                    }
                }

                Link** bodies = (*itjoint)->bodies;

                // make sure there is no wrap around for limits close to pi
                dReal fjointang = pvalues[0]-(*itjoint)->GetOffset();
                if( (*itjoint)->GetType() != Joint::JointSlider ) {
                    if( fjointang < -PI+0.001f ) fjointang = -PI+0.001f;
                    else if( fjointang > PI-0.001f ) fjointang = PI-0.001f;
                }

                if( bodies[0] != NULL && bodies[1] != NULL && !bodies[1]->IsStatic()) {
                    if( bodies[0]->userdata ) {
                        if( !bodies[1]->userdata ) {
                            //RAVELOG_INFOA("reg %S:%S!\n",bodies[0]->GetName(),bodies[1]->GetName());
                            Transform tjoint;
                            axis = (*itjoint)->vAxes[0];

                            // init 1 from 0
                            switch((*itjoint)->GetType()) {
                            case Joint::JointHinge:
                                tjoint.rotfromaxisangle(axis, -fjointang);
                                break;
                            case Joint::JointHinge2:
                                {
                                    Transform tfirst;
                                    tfirst.rotfromaxisangle((*itjoint)->vAxes[0], -pvalues[0]);
                                    Transform tsecond;
                                    tsecond.rotfromaxisangle(tfirst.rotate((*itjoint)->vAxes[1]), -pvalues[1]);
                                    tjoint = tsecond * tfirst;
                                }
                                break;
                            case Joint::JointSlider:
                                tjoint.trans = -axis * fjointang;
                                break;
                            default:
                                RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                                break;
                            }
                            
                            trans = bodies[0]->GetTransform() * (*itjoint)->tLeft * tjoint * (*itjoint)->tRight;
                            bodies[1]->SetTransform(trans);

                            if( pvbodies != NULL ) {
                                for(size_t i = 0; i < _veclinks.size(); ++i) {
                                    if( _veclinks[i] == bodies[1] ) {
                                        (*pvbodies)[i] = trans;
                                        break;
                                    }
                                }
                            }

                            bodies[1]->userdata = 1;
                            numleft--;

#ifdef _DEBUG
                            {
                                dReal angs[3];
                                (*itjoint)->GetValues(angs);
                                assert( fabsf(fjointang+(*itjoint)->GetOffset()-angs[0]) < 0.03f );
                            }
#endif
                        }
                    }
                    else if( bodies[1]->userdata ) {
                        Transform tjoint;
                        axis = (*itjoint)->vAxes[0];

                        // init 1 from 0
                        switch((*itjoint)->GetType()) {
                        case Joint::JointHinge:
                            tjoint.rotfromaxisangle(axis, fjointang);
                            break;
                        case Joint::JointSlider:
                            tjoint.trans = axis * fjointang;
                            break;
                        default:
                            RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                            break;
                        }

                        trans = bodies[1]->GetTransform() * (*itjoint)->tinvRight * tjoint * (*itjoint)->tinvLeft;
                        bodies[0]->SetTransform(trans);

                        if( pvbodies != NULL ) {
                            for(size_t i = 0; i < _veclinks.size(); ++i) {
                                if( _veclinks[i] == bodies[0] ) {
                                    (*pvbodies)[i] = trans;
                                    break;
                                }
                            }
                        }

                        bodies[0]->userdata = 1;
                        numleft--;

#ifdef _DEBUG
                        {
                            dReal angs[3];
                            (*itjoint)->GetValues(angs);
                            assert( fabsf(fjointang+(*itjoint)->GetOffset()-angs[0]) < 0.03f );
                        }
#endif
                    }
                }
                else if( bodies[0] != NULL && !bodies[0]->userdata ) {

                    Transform tjoint;
                    axis = (*itjoint)->vAxes[0];
                    
                    // joint attached to static environment (it will never be [1])
                    switch((*itjoint)->GetType()) {
                    case Joint::JointHinge:
                        tjoint.rotfromaxisangle(axis, -fjointang);
                        break;
                    case Joint::JointHinge2:
                        {
                            Transform tfirst;
                            tfirst.rotfromaxisangle((*itjoint)->vAxes[0], -pvalues[0]);
                            Transform tsecond;
                            tsecond.rotfromaxisangle(tfirst.rotate((*itjoint)->vAxes[1]), -pvalues[1]);
                            tjoint = tsecond * tfirst;
                        }
                        break;
                    case Joint::JointSlider:
                        tjoint.trans = -axis * fjointang;
                        break;
                    default:
                        RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                        break;
                    }
                    
                    trans = GetTransform() * (*itjoint)->tLeft * tjoint * (*itjoint)->tRight;
                    bodies[0]->SetTransform(trans);

                    if( pvbodies != NULL ) {
                        for(size_t i = 0; i < _veclinks.size(); ++i) {
                            if( _veclinks[i] == bodies[0] ) {
                                (*pvbodies)[i] = trans;
                                break;
                            }
                        }
                    }

#ifdef _DEBUG
                    {
                        dReal angs[3];
                        (*itjoint)->GetValues(angs);
                        assert( fabsf(fjointang+(*itjoint)->GetOffset()-angs[0]) < 0.03f );
                    }
#endif
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
                _veclinks[i]->SetTransform(_veclinks[i]->GetTransform()*tbase);
                
                if( pvbodies != NULL ) {
                    (*pvbodies)[i] = _veclinks[i]->GetTransform();
                }
            }
        }
    }
}

void KinBody::SetJointVelocities(const dReal* pJointVelocities)
{
	if( pJointVelocities != NULL ) {
		assert( GetEnv() != NULL );
		Vector linearvel, angularvel;
		GetVelocity(linearvel, angularvel);
	    GetEnv()->GetPhysicsEngine()->SetBodyVelocity(this, linearvel, angularvel, pJointVelocities);
	}
}

void KinBody::CalculateJacobian(int index, const Vector& trans, dReal* pfJacobian) const
{
    assert( index >= 0 && index < (int)_veclinks.size() && pfJacobian != NULL );

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
        ++pfJacobian;
    }
}

void KinBody::CalculateRotationJacobian(int index, const Vector& q, dReal* pfJacobian) const
{
    assert( index >= 0 && index < (int)_veclinks.size() && pfJacobian != NULL );

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
        ++pfJacobian;
    }
}

void KinBody::CalculateAngularVelocityJacobian(int index, dReal* pfJacobian) const
{
    assert( index >= 0 && index < (int)_veclinks.size() && pfJacobian != NULL );

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
        ++pfJacobian;
    }
}

bool KinBody::CheckSelfCollision(COLLISIONREPORT* pReport) const
{
    return GetEnv()->GetCollisionChecker()->CheckSelfCollision(this, pReport);
}

void KinBody::ComputeJointHierarchy()
{
    _vecJointHierarchy.resize(_vecjoints.size()*_veclinks.size());
    if( _vecJointHierarchy.size() == 0 )
        return;

    memset(&_vecJointHierarchy[0], 0, _vecJointHierarchy.size() * sizeof(_vecJointHierarchy[0]));
    
    for(size_t i = 0; i < _veclinks.size(); ++i) {
        _veclinks[i]->userdata = _veclinks[i]->IsStatic();
    }
    _veclinks[0]->userdata = 1; // always set the first to be a root

    typedef pair<Joint*, int> JOINTPAIR;
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
            if( bIsPassive && itjoint->first->GetMimicJointIndex() < 0 ) {
                // passive and doesn't affect links in any way, so skip
                itjoint = ljoints.erase(itjoint);
                continue;
            }

            char* pvalues = &_vecJointHierarchy[!bIsPassive ? itjoint->second*_veclinks.size() : itjoint->first->GetMimicJointIndex() * _veclinks.size()];

            Link** bodies = itjoint->first->bodies;
            if( bodies[0] != NULL ) {
                
                if( bodies[1] != NULL ) {

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
    SetJointValues(NULL, NULL, &vzero[0]);
    _setAdjacentLinks.clear();
    _setNonAdjacentLinks.clear();  

    FOREACH(itj, _vecjoints) {
        if( (*itj)->bodies[0] != NULL && (*itj)->bodies[1] != NULL ) {
            int ind0 = (*itj)->bodies[0]->GetIndex();
            int ind1 = (*itj)->bodies[1]->GetIndex();
            if( ind1 < ind0 ) _setAdjacentLinks.insert(ind1|(ind0<<16));
            else _setAdjacentLinks.insert(ind0|(ind1<<16));
        }
    }

    FOREACH(itadj, _vForcedAdjacentLinks) {
        Link* pl0 = GetLink(itadj->first.c_str());
        Link* pl1 = GetLink(itadj->second.c_str());
        if( pl0 != NULL && pl1 != NULL ) {
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
                !GetEnv()->CheckCollision(_veclinks[i], _veclinks[j]) )
                _setNonAdjacentLinks.insert(i|(j<<16));
        }
    }

    //RAVEPRINT(L"nonadj: %"PRIdS"\n", _setNonAdjacentLinks.size());
    // save the forward kinematics
    char filename[100];
    sprintf(filename, "fk_%S.txt", GetName());
    ofstream f(filename);
    WriteForwardKinematics(f);
    SetJointValues(NULL, NULL, &prevvalues[0], true);
}

struct LINKTRANSINFO
{
    LINKTRANSINFO() : bRecord(false) { fJointCoeffs[0] = 1; fJointCoeffs[1] = 0; }
    int linkcur;
    int linkbase;
    int jointindex;
    Vector vjointaxis;
    dReal fJointCoeffs[2]; ///< use (jointangle*coeffs[0]+coeffs[1]) for the rotation around the axis
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
    vector<Joint*>* alljoints[2] = {&_vecjoints, &_vecPassiveJoints};

    while(numleft > 0) {
        int org = numleft;

        // iterate through the two sets of joints: active and passive
        // for active joints, read the angle from pJointValues, for passive joints
        // check if they have corresponding mimic joints, if they do, take those angles
        // otherwise set to 0
        for(size_t j = 0; j < ARRAYSIZE(alljoints); ++j) {
            
            FOREACH(itjoint, *alljoints[j]) {

                int jointindex = 0;
                if( (*itjoint)->GetMimicJointIndex() >= 0 ) {
                    jointindex = (*itjoint)->GetMimicJointIndex();
                    memcpy(info.fJointCoeffs, (*itjoint)->GetMimicCoeffs(), sizeof(info.fJointCoeffs));
                }
                else if( j == 0 ) {
                    jointindex = (int)(itjoint-alljoints[j]->begin());
                    info.fJointCoeffs[0] = 1; info.fJointCoeffs[1] = 0;
                }

                Link** bodies = (*itjoint)->bodies;

                if( bodies[0] != NULL && bodies[1] != NULL && !bodies[1]->IsStatic()) {
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
                        default:
                            RAVELOG_WARNA("forward kinematic type %d not supported\n", (*itjoint)->GetType());
                            break;
                        }

                        bodies[0]->userdata = 1;
                        numleft--;
                    }
                }
                else if( bodies[0] != NULL && !bodies[0]->userdata ) {

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
          << it->fJointCoeffs[0] << " " << it->fJointCoeffs[1] <<  endl << endl;
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

void KinBody::AttachBody(KinBody* pbody)
{
    assert( pbody != NULL );
    _setAttachedBodies.insert(pbody);
    pbody->_setAttachedBodies.insert(this);
}

void KinBody::RemoveBody(KinBody* pbody)
{
    if( pbody != NULL ) {
        _setAttachedBodies.erase(pbody);
        pbody->_setAttachedBodies.erase(this);
    }
    else {
        FOREACH(it, _setAttachedBodies)
            (*it)->_setAttachedBodies.erase(this);
        _setAttachedBodies.clear();
    }
}

bool KinBody::IsAttached(const KinBody* pbody) const
{
    return pbody != NULL && _setAttachedBodies.find((KinBody*)pbody) != _setAttachedBodies.end();
}

void KinBody::Enable(bool bEnable)
{
    GetEnv()->GetCollisionChecker()->Enable(this,bEnable);
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

bool KinBody::Clone(const InterfaceBase* preference, int cloningoptions)
{
    if( preference == NULL )
        return false;
    const KinBody& r = *(const KinBody*)preference;

    name = r.name;
    
    _veclinks.resize(0); _veclinks.reserve(r._veclinks.size());
    FOREACHC(itlink, r._veclinks) {
        Link* pnewlink = new Link(this);
        *pnewlink = **itlink; // be careful of copying pointers
        pnewlink->_parent = this;
        _veclinks.push_back(pnewlink);
    }

    _vecjoints.resize(0); _vecjoints.reserve(r._vecjoints.size());
    FOREACHC(itjoint, r._vecjoints) {
        Joint* pnewjoint = new Joint(this);
        *pnewjoint = **itjoint; // be careful of copying pointers!
        pnewjoint->_parent = this;
        if( (*itjoint)->bodies[0] != NULL )
            pnewjoint->bodies[0] = _veclinks[(*itjoint)->bodies[0]->GetIndex()];
        if( (*itjoint)->bodies[1] != NULL )
            pnewjoint->bodies[1] = _veclinks[(*itjoint)->bodies[1]->GetIndex()];
        _vecjoints.push_back(pnewjoint);
    }

    _vecPassiveJoints.reserve(r._vecPassiveJoints.size());
    FOREACHC(itjoint, r._vecPassiveJoints) {
        Joint* pnewjoint = new Joint(this);
        *pnewjoint = **itjoint; // be careful of copying pointers!
        pnewjoint->_parent = this;
        if( (*itjoint)->bodies[0] != NULL )
            pnewjoint->bodies[0] = _veclinks[(*itjoint)->bodies[0]->GetIndex()];
        if( (*itjoint)->bodies[1] != NULL )
            pnewjoint->bodies[1] = _veclinks[(*itjoint)->bodies[1]->GetIndex()];
        _vecPassiveJoints.push_back(pnewjoint);
    }

    _vecJointIndices = r._vecJointIndices;
    _vecJointWeights = r._vecJointWeights;
    _vecJointHierarchy = r._vecJointHierarchy;
    
    _setAdjacentLinks = r._setAdjacentLinks;
    _setNonAdjacentLinks = r._setNonAdjacentLinks;
    _vForcedAdjacentLinks = r._vForcedAdjacentLinks;

    strXMLFilename = r.strXMLFilename;
    _vTempJoints = r._vTempJoints;

    _nUpdateStampId++;
    return true;
}

} // end namespace OpenRAVE
