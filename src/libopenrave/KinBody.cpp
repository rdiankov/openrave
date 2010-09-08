// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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
/** \file   KinBody.cpp
    \brief  Definition of OpenRAVE::KinBody
 */
#include "libopenrave.h"

#include <algorithm>

#define CHECK_INTERNAL_COMPUTATION { \
    if( !_bHierarchyComputed ) { \
        throw openrave_exception(str(boost::format("%s: joint hierarchy needs to be computed (is body added to environment?)\n")%__PRETTY_FUNCTION__)); \
    } \
} \

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
    vmin = vmax = vertices.at(0);
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

void KinBody::Link::TRIMESH::serialize(std::ostream& o, int options) const
{
    o << vertices.size() << " ";
    FOREACHC(it,vertices)
        o << it->x << " " << it->y << " " << it->z << " ";
    o << indices.size() << " ";
    FOREACHC(it,indices)
        o << *it << " ";
}

KinBody::Link::GEOMPROPERTIES::GEOMPROPERTIES(KinBody::LinkPtr parent) : _parent(parent)
{
    diffuseColor = Vector(1,1,1);
    type = GeomNone;
    ftransparency = 0;
    vRenderScale = Vector(1,1,1);
    _bDraw = true;
    _bModifiable = true;
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
        ab.extents.x = (dReal)0.5*RaveFabs(tglobal.m[2])*vGeomData.y + RaveSqrt(1-tglobal.m[2]*tglobal.m[2])*vGeomData.x;
        ab.extents.y = (dReal)0.5*RaveFabs(tglobal.m[6])*vGeomData.y + RaveSqrt(1-tglobal.m[6]*tglobal.m[6])*vGeomData.x;
        ab.extents.z = (dReal)0.5*RaveFabs(tglobal.m[10])*vGeomData.y + RaveSqrt(1-tglobal.m[10]*tglobal.m[10])*vGeomData.x;
        ab.pos = tglobal.trans;//+(dReal)0.5*vGeomData.y*Vector(tglobal.m[2],tglobal.m[6],tglobal.m[10]);
        break;
    case GeomTrimesh:
        // just use collisionmesh
        if( collisionmesh.vertices.size() > 0) {
            Vector vmin, vmax; vmin = vmax = tglobal*collisionmesh.vertices.at(0);
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
        BOOST_ASSERT(0);
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
        if( v[0].dot3((v[1]-v[0]).cross(v[2]-v[0])) < 0 ) {
            swap(indices[i], indices[i+1]);
        }
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

    if( fTessellation < 0.01f )
        fTessellation = 0.01f;

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
            if( v1.dot3(v2-v1.cross(v3-v1)) < 0 ) {
                swap(indices[i], indices[i+1]);
            }
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
            dReal s = rad * RaveSin(dtheta * (dReal)i);
            dReal c = rad * RaveCos(dtheta * (dReal)i);

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

void KinBody::Link::GEOMPROPERTIES::serialize(std::ostream& o, int options) const
{
    SerializeRound(o,_t);
    o << type << " ";
    SerializeRound3(o,vRenderScale);
    if( type == GeomTrimesh )
        collisionmesh.serialize(o,options);
    else
        SerializeRound3(o,vGeomData);
}

void KinBody::Link::GEOMPROPERTIES::SetCollisionMesh(const TRIMESH& mesh)
{
    if( !_bModifiable )
        throw openrave_exception("geometry cannot be modified");
    LinkPtr parent(_parent);
    collisionmesh = mesh;
    parent->UpdateCollisionMesh();
    parent->GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

void KinBody::Link::GEOMPROPERTIES::SetDraw(bool bDraw)
{
    if( _bDraw != bDraw ) {
        LinkPtr parent(_parent);
        _bDraw = bDraw;
        parent->GetParent()->_ParametersChanged(Prop_LinkDraw);
    }
}

void KinBody::Link::GEOMPROPERTIES::SetTransparency(float f)
{
    LinkPtr parent(_parent);
    ftransparency = f;
    parent->GetParent()->_ParametersChanged(Prop_LinkDraw);
}

/*
 * Ray-box intersection using IEEE numerical properties to ensure that the
 * test is both robust and efficient, as described in:
 *
 *      Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley
 *      "An Efficient and Robust Ray-Box Intersection Algorithm"
 *      Journal of graphics tools, 10(1):49-54, 2005
 *
 */
//static bool RayAABBIntersect(const Ray &r, float t0, float t1) const
//{
//    dReal tmin, tmax, tymin, tymax, tzmin, tzmax;
//    tmin = (parameters[r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
//    tmax = (parameters[1-r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
//    tymin = (parameters[r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
//    tymax = (parameters[1-r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
//    if ( (tmin > tymax) || (tymin > tmax) ) 
//        return false;
//    if (tymin > tmin)
//        tmin = tymin;
//    if (tymax < tmax)
//        tmax = tymax;
//    tzmin = (parameters[r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
//    tzmax = (parameters[1-r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
//    if ( (tmin > tzmax) || (tzmin > tmax) ) 
//        return false;
//    if (tzmin > tmin)
//        tmin = tzmin;
//    if (tzmax < tmax)
//        tmax = tzmax;
//    return ( (tmin < t1) && (tmax > t0) );
//}

bool KinBody::Link::GEOMPROPERTIES::ValidateContactNormal(const Vector& _position, Vector& _normal) const
{
    Transform tinv = _t.inverse();
    Vector position = tinv*_position;
    Vector normal = tinv.rotate(_normal);
    const dReal feps=0.00005f;
    switch(GetType()) {
    case KinBody::Link::GEOMPROPERTIES::GeomBox: {
        // transform position in +x+y+z octant
        Vector tposition=position, tnormal=normal;
        if( tposition.x < 0) {
            tposition.x = -tposition.x;
            tnormal.x = -tnormal.x;
        }
        if( tposition.y < 0) {
            tposition.y = -tposition.y;
            tnormal.y = -tnormal.y;
        }
        if( tposition.z < 0) {
            tposition.z = -tposition.z;
            tnormal.z = -tnormal.z;
        }
        // find the normal to the surface depending on the region the position is in
        dReal xaxis = -vGeomData.z*tposition.y+vGeomData.y*tposition.z;
        dReal yaxis = -vGeomData.x*tposition.z+vGeomData.z*tposition.x;
        dReal zaxis = -vGeomData.y*tposition.x+vGeomData.x*tposition.y;
        dReal penetration=0;
        if( zaxis < feps && yaxis > -feps ) { // x-plane
            if( RaveFabs(tnormal.x) > RaveFabs(penetration) )
                penetration = tnormal.x;
        }
        if( zaxis > -feps && xaxis < feps ) { // y-plane
            if( RaveFabs(tnormal.y) > RaveFabs(penetration) )
                penetration = tnormal.y;
        }
        if( yaxis < feps && xaxis > -feps ) { // z-plane
            if( RaveFabs(tnormal.z) > RaveFabs(penetration) )
                penetration = tnormal.z;
        }
        if( penetration < -feps ) {
            _normal = -_normal;
            return true;
        }
        break;
    }
    case KinBody::Link::GEOMPROPERTIES::GeomCylinder: { // z-axis
        dReal fInsideCircle = position.x*position.x+position.y*position.y-vGeomData.x*vGeomData.x;
        dReal fInsideHeight = 2.0f*RaveFabs(position.z)-vGeomData.y;
        if( fInsideCircle < -feps && fInsideHeight > -feps && normal.z*position.z<0 ) {
            _normal = -_normal;
            return true;
        }
        if( fInsideCircle > -feps && fInsideHeight < -feps && normal.x*position.x+normal.y*position.y < 0 ) {
            _normal = -_normal;
            return true;
        }
        break;
    }
    case KinBody::Link::GEOMPROPERTIES::GeomSphere:
        if( normal.dot3(position) < 0 ) {
            _normal = -_normal;
            return true;
        }
        break;
    default:
        break;
    }
    return false;
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
    KinBodyPtr parent = GetParent();
    if( !parent->GetEnv()->GetCollisionChecker()->EnableLink(LinkConstPtr(shared_from_this()),bEnable) ) {
        throw openrave_exception(str(boost::format("failed to enable link %s:%s")%parent->GetName()%GetName()));
    }
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

void KinBody::Link::serialize(std::ostream& o, int options) const
{
    o << index << " ";
    if( options & SO_Geometry ) {
        o << _listGeomProperties.size() << " ";
        FOREACHC(it,_listGeomProperties)
            it->serialize(o,options);
    }
    if( options & SO_Dynamics ) {
        SerializeRound(o,_transMass);
        SerializeRound(o,_mass);
    }
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

void KinBody::Link::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->SetLinkVelocity(shared_from_this(), linearvel, angularvel);
}

void KinBody::Link::GetVelocity(Vector& linearvel, Vector& angularvel) const
{
    GetParent()->GetEnv()->GetPhysicsEngine()->GetLinkVelocity(shared_from_this(), linearvel, angularvel);
}

KinBody::Link::GEOMPROPERTIES& KinBody::Link::GetGeometry(int index)
{
    std::list<GEOMPROPERTIES>::iterator it = _listGeomProperties.begin();
    advance(it,index);
    return *it;
}

void KinBody::Link::SwapGeometries(std::list<KinBody::Link::GEOMPROPERTIES>& listNewGeometries)
{
    LinkWeakPtr pnewlink;
    if( listNewGeometries.size() > 0 )
        pnewlink=listNewGeometries.front()._parent;
    _listGeomProperties.swap(listNewGeometries);
    FOREACH(itgeom,_listGeomProperties)
        itgeom->_parent = LinkWeakPtr(shared_from_this());
    FOREACH(itgeom,listNewGeometries)
        itgeom->_parent=pnewlink;
    UpdateCollisionMesh();
    GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

bool KinBody::Link::ValidateContactNormal(const Vector& position, Vector& normal) const
{
    if( _listGeomProperties.size() == 1) {
        return _listGeomProperties.front().ValidateContactNormal(position,normal);
    }
    return false;
}

void KinBody::Link::UpdateCollisionMesh()
{
    collision.vertices.resize(0);
    collision.indices.resize(0);
    FOREACH(itgeom,_listGeomProperties)
        collision.Append(itgeom->GetCollisionMesh(),itgeom->GetTransform());
}

KinBody::Joint::Joint(KinBodyPtr parent)
{
    _parent = parent;
    vMimicCoeffs.resize(2); vMimicCoeffs[0] = 1; vMimicCoeffs[1] = 0;
    // fill vAxes with valid values
    FOREACH(it,vAxes)
        *it = Vector(0,0,1);
    nMimicJointIndex = -1;
    fResolution = dReal(0.02);
    fMaxVel = 1e5f;
    fMaxAccel = 1e5f;
    fMaxTorque = 1e5f;
    offset = 0;
    jointindex=-1;
    dofindex = -1; // invalid index
    _bIsCircular = false;
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

bool KinBody::Joint::IsStatic() const
{
    if( IsCircular() || GetMimicJointIndex() >= 0 )
        return false;
    for(size_t i = 0; i < _vlowerlimit.size(); ++i) {
        if( _vlowerlimit[i] < _vupperlimit[i] )
            return false;
    }
    return true;
}

void KinBody::Joint::GetValues(vector<dReal>& pValues, bool bAppend) const
{
    Transform tjoint;
    dReal f;
    if ( !!bodies[1] && !bodies[1]->IsStatic() ) 
        tjoint = tinvLeft * bodies[0]->GetTransform().inverse() * bodies[1]->GetTransform() * tinvRight;
    else
        tjoint = tinvLeft * GetParent()->GetTransform().inverse() * bodies[0]->GetTransform() * tinvRight;
        
    if( !bAppend )
        pValues.resize(0);

    switch(type) {
    case JointHinge:
        f = 2.0f*RaveAtan2(tjoint.rot.y*vAxes[0].x+tjoint.rot.z*vAxes[0].y+tjoint.rot.w*vAxes[0].z, tjoint.rot.x);
        // expect values to be within -PI to PI range
        if( f < -PI )
            f += 2*PI;
        else if( f > PI )
            f -= 2*PI;
        pValues.push_back(offset+f);
        break;
    case JointHinge2:
        {
            Vector axis1cur = tjoint.rotate(vAxes[0]);
            Vector axis2cur = tjoint.rotate(vAxes[1]);
            Vector vec1;
            Vector vec2;
            Vector vec3;
            vec1 = (vAxes[1] - vAxes[0].dot3(vAxes[1])*vAxes[0]).normalize();
            vec2 = (axis2cur - vAxes[0].dot3(axis2cur)*vAxes[0]).normalize();
            vec3 = vAxes[0].cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
            if( f < -PI )
                f += 2*PI;
            else if( f > PI )
                f -= 2*PI;
            pValues.push_back(offset+f);
            vec1 = (vAxes[0] - axis2cur.dot(vAxes[0])*axis2cur).normalize();
            vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
            vec3 = axis2cur.cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
            if( f < -PI )
                f += 2*PI;
            else if( f > PI )
                f -= 2*PI;
            pValues.push_back(f);
        }
        break;
    case JointSlider:
        pValues.push_back(offset+(tjoint.trans.x*vAxes[0].x+tjoint.trans.y*vAxes[0].y+tjoint.trans.z*vAxes[0].z));
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

Vector KinBody::Joint::GetAxis(int iaxis) const
{
    if( !bodies[0] ) {
        return vAxes.at(iaxis);
    }
    else if( !!bodies[1] && bodies[1]->IsStatic() ) {
        return bodies[1]->GetTransform().rotate(vAxes.at(iaxis));
    }
    else {
        return bodies[0]->GetTransform().rotate(vAxes.at(iaxis));
    }
}

Vector KinBody::Joint::GetInternalHierarchyAxis(int iaxis) const
{
    return vAxes.at(iaxis);
}

Transform KinBody::Joint::GetInternalHierarchyLeftTransform() const
{
    if( !!bodies[1] && bodies[1] == bodies[0]->GetParentLink() ) {
        // bodies[0] is a child
        Transform tjoint;
        if( GetType() == Joint::JointHinge ) {
            tjoint.rotfromaxisangle(vAxes.at(0),GetOffset());
        }
        else if( GetType() == Joint::JointSlider ) {
            tjoint.trans = vAxes.at(0)*(GetOffset());
        }
        return tinvRight*tjoint;
    }
    Transform tjoint;
    if( GetType() == Joint::JointHinge ) {
        tjoint.rotfromaxisangle(vAxes.at(0),-GetOffset());
    }
    else if( GetType() == Joint::JointSlider ) {
        tjoint.trans = vAxes.at(0)*(-GetOffset());
    }
    return tLeft*tjoint;
}

Transform KinBody::Joint::GetInternalHierarchyRightTransform() const
{
    return ( !!bodies[1] && bodies[1] == bodies[0]->GetParentLink() ) ? tinvLeft : tRight;
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

dReal KinBody::Joint::GetWeight(int iaxis) const
{
    return _vweights.at(iaxis);
}

void KinBody::Joint::SetJointOffset(dReal newoffset)
{
    offset = newoffset;
    GetParent()->_ParametersChanged(Prop_JointOffset);
}

void KinBody::Joint::SetJointLimits(const std::vector<dReal>& vLowerLimit, const std::vector<dReal>& vUpperLimit)
{
    _vlowerlimit = vLowerLimit;
    _vupperlimit = vUpperLimit;
    GetParent()->_ParametersChanged(Prop_JointLimits);
}

void KinBody::Joint::SetResolution(dReal resolution)
{
    fResolution = resolution;
    GetParent()->_ParametersChanged(Prop_JointProperties);
}

void KinBody::Joint::SetWeights(const std::vector<dReal>& vweights)
{
    _vweights = vweights;
    GetParent()->_ParametersChanged(Prop_JointProperties);
}

void KinBody::Joint::AddTorque(const std::vector<dReal>& pTorques)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->AddJointTorque(shared_from_this(), pTorques);
}

void KinBody::Joint::serialize(std::ostream& o, int options) const
{
    if( options & SO_Kinematics ) {
        o << dofindex << " " << jointindex << " " << type << " ";
        SerializeRound(o,tRight);
        SerializeRound(o,tLeft);
        SerializeRound(o,offset);
        o << nMimicJointIndex << " ";
        SerializeRound3(o,vanchor);
        for(int i = 0; i < GetDOF(); ++i)
            SerializeRound3(o,vAxes[i]);
        FOREACHC(it,vMimicCoeffs)
            SerializeRound(o,*it);
        o << (!bodies[0]?-1:bodies[0]->GetIndex()) << " " << (!bodies[1]?-1:bodies[1]->GetIndex()) << " ";
    }
    if( options & SO_Dynamics ) {
        SerializeRound(o,fMaxVel);
        SerializeRound(o,fMaxAccel);
        SerializeRound(o,fMaxTorque);
        FOREACHC(it,_vlowerlimit)
            SerializeRound(o,*it);
        FOREACHC(it,_vupperlimit)
            SerializeRound(o,*it);
    }
}

KinBody::KinBodyStateSaver::KinBodyStateSaver(KinBodyPtr pbody, int options) : _options(options), _pbody(pbody)
{
    if( _options & Save_LinkTransformation ) {
        _pbody->GetBodyTransformations(_vLinkTransforms);
    }
    if( _options & Save_LinkEnable ) {
        _vEnabledLinks.resize(_vLinkTransforms.size());
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i)
            _vEnabledLinks[i] = _pbody->GetLinks().at(i)->IsEnabled();
    }
}

KinBody::KinBodyStateSaver::~KinBodyStateSaver()
{
    if( _options & Save_LinkTransformation ) {
        _pbody->SetBodyTransformations(_vLinkTransforms);
    }
    if( _options & Save_LinkEnable ) {
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            if( _pbody->GetLinks().at(i)->IsEnabled() != !!_vEnabledLinks[i] )
                _pbody->GetLinks().at(i)->Enable(!!_vEnabledLinks[i]);
        }
    }
}

KinBody::KinBody(InterfaceType type, EnvironmentBasePtr penv) : InterfaceBase(type, penv)
{
    _bHierarchyComputed = false;
    _bMakeJoinedLinksAdjacent = true;
    _environmentid = 0;
    
    _nUpdateStampId = 0;
}

KinBody::~KinBody()
{
    RAVELOG_VERBOSEA(str(boost::format("destroying kinbody: %s\n")%GetName()));
    Destroy();
}

void KinBody::Destroy()
{
    if( _listAttachedBodies.size() > 0 ) {
        stringstream ss; ss << GetName() << " still has attached bodies: ";
        FOREACHC(it,_listAttachedBodies) {
            KinBodyPtr pattached = it->lock();
            if( !!pattached )
                ss << pattached->GetName();
        }
        RAVELOG_WARNA(ss.str());
    }
    _listAttachedBodies.clear();

    _veclinks.clear();
    _vecjoints.clear();
    _vDependencyOrderedJoints.clear();
    _vDOFOrderedJoints.clear();
    _vecPassiveJoints.clear();
    _vecJointIndices.clear();
    _vecJointHierarchy.clear();

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
    if( GetEnvironmentId() )
        throw openrave_exception(str(boost::format("KinBody::Init for %s, cannot Init a body while it is added to the environment\n")%GetName()));

    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->index = 0;
    plink->name = "base";
    plink->bStatic = true;
    Link::TRIMESH trimesh;
    FOREACHC(itab, vaabbs) {
        plink->_listGeomProperties.push_back(Link::GEOMPROPERTIES(plink));
        Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        geom.type = Link::GEOMPROPERTIES::GeomBox;
        geom._t.trans = itab->pos;
        geom._bDraw = bDraw;
        geom.vGeomData = itab->extents;
        geom.InitCollisionMesh();
        geom.diffuseColor=Vector(1,0.5f,0.5f,1);
        geom.ambientColor=Vector(0.1,0.0f,0.0f,0);
        trimesh = geom.GetCollisionMesh();
        trimesh.ApplyTransform(geom._t);
        plink->collision.Append(trimesh);
    }
    
    _veclinks.push_back(plink);
    return true;
}

bool KinBody::InitFromBoxes(const std::vector<OBB>& vobbs, bool bDraw)
{
    if( GetEnvironmentId() )
        throw openrave_exception(str(boost::format("KinBody::Init for %s, cannot Init a body while it is added to the environment\n")%GetName()));

    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->index = 0;
    plink->name = "base";
    plink->bStatic = true;
    Link::TRIMESH trimesh;
    FOREACHC(itobb, vobbs) {
        plink->_listGeomProperties.push_back(Link::GEOMPROPERTIES(plink));
        Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        geom.type = Link::GEOMPROPERTIES::GeomBox;
        TransformMatrix tm;
        tm.trans = itobb->pos;
        tm.m[0] = itobb->right.x; tm.m[1] = itobb->up.x; tm.m[2] = itobb->dir.x;
        tm.m[4] = itobb->right.y; tm.m[5] = itobb->up.y; tm.m[6] = itobb->dir.y;
        tm.m[8] = itobb->right.z; tm.m[9] = itobb->up.z; tm.m[10] = itobb->dir.z;
        geom._t = tm;
        geom._bDraw = bDraw;
        geom.vGeomData = itobb->extents;
        geom.InitCollisionMesh();
        geom.diffuseColor=Vector(1,0.5f,0.5f,1);
        geom.ambientColor=Vector(0.1,0.0f,0.0f,0);
        trimesh = geom.GetCollisionMesh();
        trimesh.ApplyTransform(geom._t);
        plink->collision.Append(trimesh);
    }
    
    _veclinks.push_back(plink);
    return true;
}

void KinBody::SetName(const std::string& newname)
{
    BOOST_ASSERT(newname.size() > 0);
    if( name != newname ) {
        name = newname;
        _ParametersChanged(Prop_Name);
    }
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
		++itindex;
    }
}

int KinBody::GetDOF() const
{
    return _vecjoints.size() > 0 ? _vecjoints.back()->GetDOFIndex()+_vecjoints.back()->GetDOF() : 0;
}

void KinBody::GetDOFValues(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vDOFOrderedJoints) {
        int toadd = (*it)->GetDOFIndex()-(int)v.size();
        if( toadd > 0 ) {
            v.insert(v.end(),toadd,0);
        }
        else if( toadd < 0 ) {
            BOOST_ASSERT(0);
        }
        (*it)->GetValues(v,true);
    }
}

void KinBody::GetDOFVelocities(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vDOFOrderedJoints) {
        (*it)->GetVelocities(v,true);
    }
}

void KinBody::GetDOFLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const
{
    vLowerLimit.resize(0);
    if( (int)vLowerLimit.capacity() < GetDOF() )
        vLowerLimit.reserve(GetDOF());
    vUpperLimit.resize(0);
    if( (int)vUpperLimit.capacity() < GetDOF() )
        vUpperLimit.reserve(GetDOF());
    FOREACHC(it,_vDOFOrderedJoints) {
        (*it)->GetLimits(vLowerLimit,vUpperLimit,true);
    }
}

void KinBody::GetDOFMaxVel(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vDOFOrderedJoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxVel());
    }
}

void KinBody::GetDOFMaxAccel(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vDOFOrderedJoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxAccel());
    }
}

void KinBody::GetDOFMaxTorque(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vDOFOrderedJoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxTorque());
    }
}

void KinBody::GetDOFResolutions(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vDOFOrderedJoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetResolution());
    }
}

void KinBody::GetDOFWeights(std::vector<dReal>& v) const
{
    v.resize(GetDOF());
    std::vector<dReal>::iterator itv = v.begin();
    FOREACHC(it, _vDOFOrderedJoints) {
        for(int i = 0; i < (*it)->GetDOF(); ++i) {
            *itv++ = (*it)->GetWeight(i);
        }
    }
}

void KinBody::GetJointValues(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints) {
        (*it)->GetValues(v,true);
    }
}

void KinBody::GetJointVelocities(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints) {
        (*it)->GetVelocities(v,true);
    }
}

void KinBody::GetJointLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const
{
    vLowerLimit.resize(0);
    if( (int)vLowerLimit.capacity() < GetDOF() )
        vLowerLimit.reserve(GetDOF());
    vUpperLimit.resize(0);
    if( (int)vUpperLimit.capacity() < GetDOF() )
        vUpperLimit.reserve(GetDOF());
    FOREACHC(it,_vecjoints) {
        (*it)->GetLimits(vLowerLimit,vUpperLimit,true);
    }
}

void KinBody::GetJointMaxVel(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxVel());
    }
}

void KinBody::GetJointMaxAccel(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxAccel());
    }
}

void KinBody::GetJointMaxTorque(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetMaxTorque());
    }
}

void KinBody::GetJointResolutions(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() )
        v.reserve(GetDOF());
    FOREACHC(it, _vecjoints) {
        v.insert(v.end(),(*it)->GetDOF(),(*it)->GetResolution());
    }
}

void KinBody::GetJointWeights(std::vector<dReal>& v) const
{
    v.resize(GetDOF());
    std::vector<dReal>::iterator itv = v.begin();
    FOREACHC(it, _vecjoints) {
        for(int i = 0; i < (*it)->GetDOF(); ++i)
            *itv++ = (*it)->GetWeight(i);
    }
}

void KinBody::SimulationStep(dReal fElapsedTime)
{
}

void KinBody::SubtractJointValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    FOREACHC(itjoint,_vecjoints) {
        int dof = (*itjoint)->GetDOFIndex();
        if( (*itjoint)->IsCircular() ) {
            for(int i = 0; i < (*itjoint)->GetDOF(); ++i)
                q1.at(dof+i) = ANGLE_DIFF(q1.at(dof+i), q2.at(dof+i));
        }
        else {
            for(int i = 0; i < (*itjoint)->GetDOF(); ++i)
                q1.at(dof+i) -= q2.at(dof+i);
        }
    }
}

// like apply transform except everything is relative to the first frame
void KinBody::SetTransform(const Transform& trans)
{
    if( _veclinks.size() == 0 )
        return;

    Transform tbaseinv = _veclinks.front()->GetTransform().inverse();
    Transform tapply = trans * tbaseinv;
    FOREACH(itlink, _veclinks)
        (*itlink)->SetTransform(tapply * (*itlink)->GetTransform());
}

Transform KinBody::GetTransform() const
{
    return _veclinks.size() > 0 ? _veclinks.front()->GetTransform() : Transform();
}

void KinBody::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    PhysicsEngineBasePtr p = GetEnv()->GetPhysicsEngine();
    FOREACH(itlink,_veclinks) {
        p->SetLinkVelocity(*itlink,linearvel,angularvel);
    }
}

void KinBody::GetVelocity(Vector& linearvel, Vector& angularvel) const
{
    GetEnv()->GetPhysicsEngine()->GetLinkVelocity(_veclinks.at(0), linearvel, angularvel);
}

void KinBody::GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const
{
    velocities.resize(_veclinks.size());
    if( velocities.size() == 0 )
        return;
    GetVelocity(velocities[0].first,velocities[0].second);

    // set the first body and all static bodies to computed
    _veclinks.front()->userdata = 1;
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

                boost::array<LinkPtr,2>& bodies = (*itjoint)->bodies;
                if( !!bodies[0] && !!bodies[1] && !bodies[1]->IsStatic()) {
                    if( bodies[0]->userdata ) {
                        if( !bodies[1]->userdata ) {
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
                            velocities[bodies[1]->GetIndex()].first = vparent + tdelta.rotate(v) + wparent.cross(bodies[1]->GetTransform().trans-tbody0.trans);
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
                            w = -vjointvel[0]*(*itjoint)->vAxes[0];
                            break;
                        case Joint::JointHinge2: {
                            Transform tfirst;
                            tfirst.rotfromaxisangle((*itjoint)->vAxes[0], -vjointang[0]);
                            w = -(vjointvel[0]*(*itjoint)->vAxes[0] + tfirst.rotate(vjointvel[1]*(*itjoint)->vAxes[1]));
                            break;
                        }
                        case Joint::JointSlider:
                            v = -vjointvel[0]*(*itjoint)->vAxes[0];
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
                        velocities[bodies[0]->GetIndex()].first = vparent + tdelta.rotate(v) + wparent.cross(bodies[0]->GetTransform().trans-tbody1.trans);
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
                    velocities[bodies[0]->GetIndex()].first = vparent + tdelta.rotate(v) + wparent.cross(bodies[0]->GetTransform().trans-tbody.trans);
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
    return _vecjoints.at(jointindex);
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
    if( _veclinks.size() == 0 )
        return;
    Transform tbase = transBase*_veclinks.at(0)->GetTransform().inverse();
    _veclinks.front()->SetTransform(transBase);

    // apply the relative transformation to all links!! (needed for passive joints)
    for(size_t i = 1; i < _veclinks.size(); ++i) {
        LinkPtr plink = _veclinks[i];
        plink->SetTransform(tbase*plink->GetTransform());
        plink->userdata = (int)plink->IsStatic();
    }

    SetJointValues(vJointValues,bCheckLimits);

    // some links might not be connected to any joints. In this case, transform them by tbase
    for(size_t i = 1; i < _veclinks.size(); ++i) {
        if( _veclinks[i]->userdata == 0 ) {
            _veclinks[i]->SetTransform(tbase*_veclinks[i]->GetTransform());
        }
    }
}

void KinBody::SetJointValues(const std::vector<dReal>& vJointValues, bool bCheckLimits)
{
    if( (int)vJointValues.size() != GetDOF() )
        throw openrave_exception(str(boost::format("dof not equal %d!=%d")%vJointValues.size()%GetDOF()),ORE_InvalidArguments);
    if( vJointValues.size() == 0 )
        return;

    const dReal* pJointValues = &vJointValues[0];
    if( bCheckLimits ) {
        _vTempJoints.resize(GetDOF());

        dReal* ptempjoints = &_vTempJoints[0];

        // check the limits
        vector<int>::const_iterator itindex = _vecJointIndices.begin();

        vector<dReal> upperlim, lowerlim;
        FOREACHC(it, _vecjoints) {
            const dReal* p = pJointValues+*itindex++;
            BOOST_ASSERT( (*it)->GetDOF() <= 3 );
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
                if( (*it)->IsCircular() ) {
                    for(int i = 0; i < (*it)->GetDOF(); ++i)
                        *ptempjoints++ = NORMALIZE_ANGLE(p[i],lowerlim[i],upperlim[i]);
                }
                else {
                    for(int i = 0; i < (*it)->GetDOF(); ++i) {
                        if( p[i] < lowerlim[i] ) *ptempjoints++ = lowerlim[i];
                        else if( p[i] > upperlim[i] ) *ptempjoints++ = upperlim[i];
                        else *ptempjoints++ = p[i];
                    }
                }
            }
        }

        pJointValues = &_vTempJoints[0];
    }

    // set the first body and all static bodies to computed
    _veclinks.at(0)->userdata = 1;
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
                
                if( itindex != _vecJointIndices.end() )
                    pvalues = pJointValues+*itindex++;

                if( (*itjoint)->GetMimicJointIndex() >= 0 ) {
                    int mimicindex = (*itjoint)->GetMimicJointIndex();
                    int jointsize = mimicindex < (int)_vecJointIndices.size()-1 ? _vecJointIndices[mimicindex+1] : GetDOF();
                    pvalues = pJointValues+_vecJointIndices[mimicindex];
                    jointsize -= _vecJointIndices[mimicindex];
                    
                    for(int i = 0; i < jointsize; ++i)
                        dummyvalues[i] = pvalues[i] * (*itjoint)->GetMimicCoeffs()[0] + (*itjoint)->GetMimicCoeffs()[1];
                    pvalues = dummyvalues;
                }
                
                if( pvalues == NULL ) {
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

                boost::array<LinkPtr,2>& bodies = (*itjoint)->bodies;

                // make sure there is no wrap around for limits close to pi
                vjointang.resize((*itjoint)->GetDOF());
                for(int iang = 0; iang < (*itjoint)->GetDOF(); ++iang) { 
                    vjointang[iang] = pvalues[iang]-(*itjoint)->GetOffset();
                }

                if( !!bodies[0] && !!bodies[1] && !bodies[1]->IsStatic()) {
                    if( bodies[0]->userdata ) {
                        if( !bodies[1]->userdata ) {
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

void KinBody::GetRigidlyAttachedLinks(int linkindex, std::vector<KinBody::LinkPtr>& vattachedlinks) const
{
    vattachedlinks.resize(0);
    if( linkindex < 0 ) {
        FOREACHC(itjoint, GetJoints()) {
            if( (*itjoint)->IsStatic() ) {
                if( !(*itjoint)->GetFirstAttached() && !!(*itjoint)->GetSecondAttached() )
                    vattachedlinks.push_back((*itjoint)->GetSecondAttached());
                if( !(*itjoint)->GetSecondAttached() && !!(*itjoint)->GetFirstAttached() )
                    vattachedlinks.push_back((*itjoint)->GetFirstAttached());
            }
        }
        FOREACHC(itpassive, GetPassiveJoints()) {
            if( (*itpassive)->IsStatic() ) {
                if( !(*itpassive)->GetFirstAttached() && !!(*itpassive)->GetSecondAttached() )
                    vattachedlinks.push_back((*itpassive)->GetSecondAttached());
                if( !(*itpassive)->GetSecondAttached() && !!(*itpassive)->GetFirstAttached() )
                    vattachedlinks.push_back((*itpassive)->GetFirstAttached());
            }
        }
    }
    else {
        vattachedlinks.push_back(_veclinks.at(linkindex));
        for(size_t icurlink = 0; icurlink<vattachedlinks.size(); ++icurlink) {
            LinkPtr plink=vattachedlinks[icurlink];
            FOREACHC(itjoint, GetJoints()) {
                if( (*itjoint)->IsStatic() ) {
                    if( (*itjoint)->GetFirstAttached() == plink && !!(*itjoint)->GetSecondAttached() && find(vattachedlinks.begin(),vattachedlinks.end(),(*itjoint)->GetSecondAttached()) == vattachedlinks.end())
                        vattachedlinks.push_back((*itjoint)->GetSecondAttached());
                    if( (*itjoint)->GetSecondAttached() == plink && !!(*itjoint)->GetFirstAttached() && find(vattachedlinks.begin(),vattachedlinks.end(),(*itjoint)->GetFirstAttached()) == vattachedlinks.end())
                        vattachedlinks.push_back((*itjoint)->GetFirstAttached());
                }
            }
            FOREACHC(itpassive, GetPassiveJoints()) {
                if( (*itpassive)->IsStatic() ) {
                    if( (*itpassive)->GetFirstAttached() == plink && !!(*itpassive)->GetSecondAttached() && find(vattachedlinks.begin(),vattachedlinks.end(),(*itpassive)->GetSecondAttached()) == vattachedlinks.end())
                        vattachedlinks.push_back((*itpassive)->GetSecondAttached());
                    if( (*itpassive)->GetSecondAttached() == plink && !!(*itpassive)->GetFirstAttached() && find(vattachedlinks.begin(),vattachedlinks.end(),(*itpassive)->GetFirstAttached()) == vattachedlinks.end())
                        vattachedlinks.push_back((*itpassive)->GetFirstAttached());
                }
            }
        }
    }
}

bool KinBody::GetChain(int linkbaseindex, int linkendindex, std::vector<JointPtr>& vjoints) const
{
    vjoints.resize(0);
    int numjoints = (int)_vecjoints.size();
    vector<int> vjointparents(numjoints+_vecPassiveJoints.size(),numjoints);
    list<pair<LinkConstPtr, int > > listlinks;
    vector<LinkPtr> vattachedlinks;
    GetRigidlyAttachedLinks(linkbaseindex,vattachedlinks);
    FOREACHC(itlink,vattachedlinks)
        listlinks.push_back(make_pair(LinkConstPtr(*itlink),-1));
    while(listlinks.size()>0) {
        LinkConstPtr plink = listlinks.front().first;
        int parentjoint = listlinks.front().second;
        if( plink->GetIndex() == linkendindex ) {
            vjoints.resize(0);
            list<int> listpath;
            while(parentjoint >= 0) {
                listpath.push_front(parentjoint);
                parentjoint = vjointparents.at(parentjoint);
            }
            vjoints.resize(0); vjoints.reserve(listpath.size());
            FOREACH(it,listpath)
                vjoints.push_back(_vecjoints.at(*it));
            return true;
        }

        listlinks.pop_front();
        FOREACHC(itjoint, _vecjoints) {
            int jointindex = (*itjoint)->GetJointIndex();
            // use the source mimic joint
            if( vjointparents.at(jointindex) == numjoints ) {
                LinkConstPtr pother;
                if( (*itjoint)->GetFirstAttached() == plink && !!(*itjoint)->GetSecondAttached() )
                    pother = (*itjoint)->GetSecondAttached();
                if( (*itjoint)->GetSecondAttached() == plink && !!(*itjoint)->GetFirstAttached() )
                    pother = (*itjoint)->GetFirstAttached();
                if( !!pother ) {
                    vjointparents[jointindex] = parentjoint;
                    int addjointindex = (*itjoint)->GetMimicJointIndex() >= 0 ? (*itjoint)->GetMimicJointIndex() : jointindex;
                    vattachedlinks.resize(0);
                    GetRigidlyAttachedLinks(pother->GetIndex(),vattachedlinks);
                    FOREACHC(itlink,vattachedlinks)
                        listlinks.push_back(make_pair(LinkConstPtr(*itlink),addjointindex));
                }
            }
        }
        // also check passive mimic joints!
        FOREACHC(itjoint, _vecPassiveJoints) {
            if( (*itjoint)->GetMimicJointIndex() >= 0 ) {
                int jointindex = (*itjoint)->GetJointIndex();
                if( vjointparents.at(numjoints+jointindex) == numjoints ) {
                    LinkConstPtr pother;
                    if( (*itjoint)->GetFirstAttached() == plink && !!(*itjoint)->GetSecondAttached() )
                        pother = (*itjoint)->GetSecondAttached();
                    if( (*itjoint)->GetSecondAttached() == plink && !!(*itjoint)->GetFirstAttached() )
                        pother = (*itjoint)->GetFirstAttached();
                    if( !!pother ) {
                        vjointparents[numjoints+jointindex] = parentjoint;
                        vattachedlinks.resize(0);
                        GetRigidlyAttachedLinks(pother->GetIndex(),vattachedlinks);
                        FOREACHC(itlink,vattachedlinks)
                            listlinks.push_back(make_pair(LinkConstPtr(*itlink),(*itjoint)->GetMimicJointIndex()));
                    }
                }
            }
        }
    }

    return false;
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

void KinBody::CalculateJacobian(int index, const Vector& trans, boost::multi_array<dReal,2>& mjacobian) const
{
    if( index < 0 || index >= (int)_veclinks.size() )
        throw openrave_exception(str(boost::format("bad index %d")%index),ORE_InvalidArguments);

    mjacobian.resize(boost::extents[3][GetDOF()]);
    if( GetDOF() == 0 )
        return;

    //Vector trans = _veclinks[index]->GetTransform() * offset;
    Vector v;
    FOREACHC(itjoint, _vecjoints) {
        int dofindex = (*itjoint)->GetDOFIndex();
        char affect = DoesAffect((*itjoint)->GetJointIndex(), index);
        for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
            if( affect == 0 )
                mjacobian[0][dofindex+dof] = mjacobian[1][dofindex+dof] = mjacobian[2][dofindex+dof] = 0;
            else {
                switch((*itjoint)->GetType()) {
                case Joint::JointHinge:
                    v = (*itjoint)->GetAxis(dof).cross(trans-(*itjoint)->GetAnchor());
                    break;
                case Joint::JointSlider:
                    v = (*itjoint)->GetAxis(dof);
                    break;
                default:
                    RAVELOG_WARNA("CalculateJacobian joint %d not supported\n", (*itjoint)->GetType());
                    v = Vector(0,0,0);
                    break;
                }

                mjacobian[0][dofindex+dof] = v.x; mjacobian[1][dofindex+dof] = v.y; mjacobian[2][dofindex+dof] = v.z;
            }
        }
    }

    // add in the contributions for each of the passive links
    FOREACHC(itjoint, _vecPassiveJoints) {
        if( (*itjoint)->GetMimicJointIndex() >= 0 && DoesAffect((*itjoint)->GetMimicJointIndex(), index) ) {
            int dofindex = _vecjoints[(*itjoint)->GetMimicJointIndex()]->GetDOFIndex();
            for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
                switch((*itjoint)->GetType()) {
                case Joint::JointHinge:
                    v = (*itjoint)->GetAxis(dof).cross(trans-(*itjoint)->GetAnchor());
                    break;
                case Joint::JointSlider:
                    v = (*itjoint)->GetAxis(dof);
                    break;
                default:
                    RAVELOG_WARNA("CalculateJacobian joint %d not supported\n", (*itjoint)->GetType());
                    v = Vector(0,0,0);
                    break;
                }

                v *= (*itjoint)->GetMimicCoeffs()[0];
                mjacobian[0][dofindex+dof] += v.x; mjacobian[1][dofindex+dof] += v.y; mjacobian[2][dofindex+dof] += v.z;
            }
        }
    }
}

void KinBody::CalculateJacobian(int index, const Vector& trans, vector<dReal>& vjacobian) const
{
    boost::multi_array<dReal,2> mjacobian;
    KinBody::CalculateJacobian(index,trans,mjacobian);
    vjacobian.resize(3*GetDOF());
    vector<dReal>::iterator itdst = vjacobian.begin();
    FOREACH(it,mjacobian) {
        std::copy(it->begin(),it->end(),itdst);
        itdst += GetDOF();
    }
}

void KinBody::CalculateRotationJacobian(int index, const Vector& q, boost::multi_array<dReal,2>& vjacobian) const
{
    if( index < 0 || index >= (int)_veclinks.size() )
        throw openrave_exception(str(boost::format("bad index %d")%index),ORE_InvalidArguments);

    vjacobian.resize(boost::extents[4][GetDOF()]);
    if( GetDOF() == 0 )
        return;

    Vector v;
    FOREACHC(itjoint, _vecjoints) {
        int dofindex = (*itjoint)->GetDOFIndex();
        char affect = DoesAffect((*itjoint)->GetJointIndex(), index);
        for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
            if( affect == 0 ) {
                vjacobian[0][dofindex+dof] = vjacobian[1][dofindex+dof] = vjacobian[2][dofindex+dof] = vjacobian[3][dofindex+dof] = 0;
            }
            else {
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

                vjacobian[0][dofindex+dof] = -q.y*v.x - q.z*v.y - q.w*v.z;
                vjacobian[1][dofindex+dof] = q.x*v.x - q.z*v.z + q.w*v.y;
                vjacobian[2][dofindex+dof] = q.x*v.y + q.y*v.z - q.w*v.x;
                vjacobian[3][dofindex+dof] = q.x*v.z - q.y*v.y + q.z*v.x;
            }
        }
    }

    // add in the contributions for each of the passive links
    FOREACHC(itjoint, _vecPassiveJoints) {
        if( (*itjoint)->GetMimicJointIndex() >= 0 && DoesAffect((*itjoint)->GetMimicJointIndex(), index) ) {
            int dofindex = _vecjoints[(*itjoint)->GetMimicJointIndex()]->GetDOFIndex();
            for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
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

                v *= (*itjoint)->GetMimicCoeffs()[0];
                vjacobian[0][dofindex+dof] += -q.y*v.x - q.z*v.y - q.w*v.z;
                vjacobian[1][dofindex+dof] += q.x*v.x - q.z*v.z + q.w*v.y;
                vjacobian[2][dofindex+dof] += q.x*v.y + q.y*v.z - q.w*v.x;
                vjacobian[3][dofindex+dof] += q.x*v.z - q.y*v.y + q.z*v.x;
            }
        }
    }
}

void KinBody::CalculateRotationJacobian(int index, const Vector& q, vector<dReal>& vjacobian) const
{
    boost::multi_array<dReal,2> mjacobian;
    KinBody::CalculateRotationJacobian(index,q,mjacobian);
    vjacobian.resize(4*GetDOF());
    vector<dReal>::iterator itdst = vjacobian.begin();
    FOREACH(it,mjacobian) {
        std::copy(it->begin(),it->end(),itdst);
        itdst += GetDOF();
    }
}

void KinBody::CalculateAngularVelocityJacobian(int index, boost::multi_array<dReal,2>& mjacobian) const
{
    if( index < 0 || index >= (int)_veclinks.size() )
        throw openrave_exception(str(boost::format("bad index %d")%index),ORE_InvalidArguments);

    mjacobian.resize(boost::extents[3][GetDOF()]);
    if( GetDOF() == 0 )
        return;

    Vector v, anchor, axis;
    FOREACHC(itjoint, _vecjoints) {
        int dofindex = (*itjoint)->GetDOFIndex();
        char affect = DoesAffect((*itjoint)->GetJointIndex(), index);
        for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
            if( affect == 0 ) {
                mjacobian[0][dofindex+dof] = mjacobian[1][dofindex+dof] = mjacobian[2][dofindex+dof] = 0;
            }
            else {
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

                mjacobian[0][dofindex+dof] = v.x; mjacobian[1][dofindex+dof] = v.y; mjacobian[2][dofindex+dof] = v.z;
            }
        }
    }

    // add in the contributions for each of the passive links
    FOREACHC(itjoint, _vecPassiveJoints) {
        if( (*itjoint)->GetMimicJointIndex() >= 0 && DoesAffect((*itjoint)->GetMimicJointIndex(), index) ) {
            int dofindex = _vecjoints[(*itjoint)->GetMimicJointIndex()]->GetDOFIndex();
            for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
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

                v *= (*itjoint)->GetMimicCoeffs()[0];
                mjacobian[0][dofindex+dof] += v.x; mjacobian[1][dofindex+dof] += v.y; mjacobian[2][dofindex+dof] += v.z;
            }
        }
    }
}

void KinBody::CalculateAngularVelocityJacobian(int index, std::vector<dReal>& vjacobian) const
{
    boost::multi_array<dReal,2> mjacobian;
    KinBody::CalculateAngularVelocityJacobian(index,mjacobian);
    vjacobian.resize(3*GetDOF());
    vector<dReal>::iterator itdst = vjacobian.begin();
    FOREACH(it,mjacobian) {
        std::copy(it->begin(),it->end(),itdst);
        itdst += GetDOF();
    }
}

bool KinBody::CheckSelfCollision(CollisionReportPtr report) const
{
    if( GetEnv()->CheckSelfCollision(shared_kinbody_const(), report) ) {
        if( !!report )
            RAVELOG_VERBOSEA(str(boost::format("Self collision: %s\n")%report->__str__()));
        return true;
    }
    return false;
}

void KinBody::_ComputeInternalInformation()
{
    BOOST_ASSERT(_vecJointIndices.size()==_vecjoints.size());
    _bHierarchyComputed = true;
    _vecJointHierarchy.resize(_vecjoints.size()*_veclinks.size());
    _vecJointIndices.resize(0);

    int jindex=0;
    FOREACH(itjoint,_vecjoints) {
        BOOST_ASSERT(jindex++==(*itjoint)->GetJointIndex());
    }
    int lindex=0;
    FOREACH(itlink,_veclinks) {
        BOOST_ASSERT( lindex++ == (*itlink)->GetIndex() );
        (*itlink)->_parentlink.reset();
    }

    if( _vecJointHierarchy.size() > 0 ) {
        vector<size_t> vorder(_vecjoints.size());
        _vecJointIndices.resize(_vecjoints.size());
        for(size_t i = 0; i < _vecjoints.size(); ++i) {
            _vecJointIndices[i] = _vecjoints[i]->dofindex;
            vorder[i] = i;
        }
        sort(vorder.begin(), vorder.end(), index_cmp<vector<int>&>(_vecJointIndices));
        _vDOFOrderedJoints.resize(0);
        FOREACH(it,vorder) {
            _vDOFOrderedJoints.push_back(_vecjoints.at(*it));
        }
        FOREACH(it,_vecJointHierarchy) {
            *it = 0;
        }
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
        _vDependencyOrderedJoints.resize(0);

        // iterate through all the joints and keep track of which joint affects which link
        while(ljoints.size() > 0) {
            size_t cursize = ljoints.size();

            itjoint = ljoints.begin();
            while(itjoint != ljoints.end()) {
            
                bool bDelete = true;
                bool bIsPassive = itjoint->second < 0;
                boost::array<LinkPtr,2>& bodies = itjoint->first->bodies;
                if( bIsPassive && itjoint->first->GetMimicJointIndex() < 0 ) {
                    // is not part of the hierarchy, but still used to join links
                    if( !!bodies[0] ) {
                        if( !!bodies[1] ) {
                            if( bodies[0]->userdata ) {
                                bodies[1]->_parentlink = bodies[0];
                                bodies[1]->userdata = 1;
                                int srcindex = bodies[0]->GetIndex();
                                int dstindex = bodies[1]->GetIndex();
                                // copy the data from bodies[0]
                                for(int j = 0; j < (int)_vecjoints.size(); ++j) {
                                    _vecJointHierarchy[j*_veclinks.size()+dstindex] = _vecJointHierarchy[j*_veclinks.size()+srcindex];
                                }
                            }
                            else if( bodies[1]->userdata ) {
                                bodies[0]->_parentlink = bodies[1];
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

                    if( bDelete )
                        itjoint = ljoints.erase(itjoint);
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
                                BOOST_ASSERT( pvalues[dstindex] >= 0 );
                            }
                            else {
                                // copy the data from bodies[0]
                                for(int j = 0; j < (int)_vecjoints.size(); ++j) {
                                    _vecJointHierarchy[j*_veclinks.size()+dstindex] = _vecJointHierarchy[j*_veclinks.size()+srcindex];
                                }
                            }

                            bodies[1]->_parentlink = bodies[0];
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

                                bodies[0]->_parentlink = bodies[1];
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

                if( bDelete ) {
                    JointPtr pjoint = bIsPassive ? _vecjoints[itjoint->first->GetMimicJointIndex()] : itjoint->first;
                    if( find(_vDependencyOrderedJoints.begin(),_vDependencyOrderedJoints.end(),pjoint) == _vDependencyOrderedJoints.end() )
                        _vDependencyOrderedJoints.push_back(pjoint);
                    itjoint = ljoints.erase(itjoint);
                }
                else ++itjoint;
            }

            if( cursize == ljoints.size() ) {
                RAVELOG_ERRORA(str(boost::format("Cannot compute joint hierarchy for number of joints %d! Part of robot might not be moveable\n")%_vecjoints.size()));
                break;
            }
        }

        if( ljoints.size() == 0 )
            RAVELOG_DEBUGA(str(boost::format("Successfully computed joint hierarchy for number of joints %d!\n")%_vecjoints.size()));
    }

    // create the adjacency list
    vector<dReal> prevvalues; GetDOFValues(prevvalues);
    vector<dReal> vzero(GetDOF());
    SetJointValues(vzero);
    _setAdjacentLinks.clear();
    _setNonAdjacentLinks.clear();  

    if( _bMakeJoinedLinksAdjacent ) {
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

    SetJointValues(prevvalues, true);

    {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics|SO_Geometry);
        __hashkinematics = GetMD5HashString(ss.str());
    }
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
    if( _veclinks.size() == 0 )
        return;

    // set the first body and all static bodies to computed
    _veclinks.at(0)->userdata = 1;
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

                boost::array<LinkPtr,2>& bodies = (*itjoint)->bodies;

                if( !!bodies[0] && !!bodies[1] && !bodies[1]->IsStatic()) {
                    if( bodies[0]->userdata ) {
                        if( !bodies[1]->userdata ) {
                            // init 1 from 0
                            switch( (*itjoint)->GetType() ) {
                            case Joint::JointHinge:
                            case Joint::JointSlider: {
                                info.type = (*itjoint)->GetType() == Joint::JointHinge ? "hinge" : "slider";
                                info.vjointaxis = (*itjoint)->vAxes[0];
                                info.Tright = (*itjoint)->tRight;
                                Transform tjoint;
                                if( (*itjoint)->GetType() == Joint::JointHinge ) {
                                    tjoint.rotfromaxisangle(info.vjointaxis,(*itjoint)->offset);
                                }
                                else if( (*itjoint)->GetType() == Joint::JointSlider ) {
                                    tjoint.trans = info.vjointaxis*(*itjoint)->offset;
                                }
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
                            info.vjointaxis = (*itjoint)->vAxes[0];
                            info.Tright = (*itjoint)->tinvLeft;
                            Transform tjoint;
                            if( (*itjoint)->GetType() == Joint::JointHinge ) {
                                tjoint.rotfromaxisangle(info.vjointaxis,-(*itjoint)->offset);
                            }
                            else if( (*itjoint)->GetType() == Joint::JointSlider ) {
                                tjoint.trans = -info.vjointaxis*(*itjoint)->offset;
                            }
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
                        info.vjointaxis = (*itjoint)->vAxes[0];
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

        if( _vecjoints.at(it->jointindex)->GetMimicJointIndex() >= 0 ) {
            BOOST_ASSERT( TransformDistanceFast(_vecjoints.at(it->jointindex)->GetInternalHierarchyLeftTransform(),it->Tleft) < 1e-5f );
            BOOST_ASSERT( TransformDistanceFast(_vecjoints.at(it->jointindex)->GetInternalHierarchyRightTransform(),it->Tright) < 1e-5f );
        }

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

bool KinBody::IsAttached(KinBodyConstPtr pbody) const
{
    if( shared_kinbody_const() == pbody )
        return true;
    std::set<KinBodyConstPtr> dummy;
    return _IsAttached(pbody,dummy);
}

void KinBody::GetAttached(std::set<KinBodyPtr>& setAttached) const
{
    setAttached.insert(boost::const_pointer_cast<KinBody>(shared_kinbody_const()));
    FOREACHC(itbody,_listAttachedBodies) {
        KinBodyPtr pattached = itbody->lock();
        if( !!pattached && setAttached.insert(pattached).second )
            pattached->GetAttached(setAttached);
    }
}

bool KinBody::_IsAttached(KinBodyConstPtr pbody, std::set<KinBodyConstPtr>& setChecked) const
{
    if( !setChecked.insert(shared_kinbody_const()).second )
        return false;
    FOREACHC(itbody,_listAttachedBodies) {
        KinBodyConstPtr pattached = itbody->lock();
        if( !!pattached && (pattached == pbody || pattached->_IsAttached(pbody,setChecked)) )
                return true;
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

int KinBody::GetEnvironmentId() const
{
    return _environmentid;
}

char KinBody::DoesAffect(int jointindex, int linkindex ) const
{
    CHECK_INTERNAL_COMPUTATION;
    BOOST_ASSERT(jointindex >= 0 && jointindex < (int)_vecjoints.size());
    return _vecJointHierarchy.at(jointindex*_veclinks.size()+linkindex);
}

const std::set<int>& KinBody::GetNonAdjacentLinks() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _setNonAdjacentLinks;
}

const std::set<int>& KinBody::GetAdjacentLinks() const
{
    CHECK_INTERNAL_COMPUTATION;
    return _setAdjacentLinks;
}

bool KinBody::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    if( !InterfaceBase::Clone(preference,cloningoptions) ) {
        return false;
    }
    KinBodyConstPtr r = RaveInterfaceConstCast<KinBody>(preference);

    name = r->name;
    _bHierarchyComputed = r->_bHierarchyComputed;
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
        if( !!(*itjoint)->bodies[0] )
            pnewjoint->bodies[0] = _veclinks.at((*itjoint)->bodies[0]->GetIndex());
        if( !!(*itjoint)->bodies[1] )
            pnewjoint->bodies[1] = _veclinks.at((*itjoint)->bodies[1]->GetIndex());
        _vecjoints.push_back(pnewjoint);
    }

    _vecPassiveJoints.resize(0); _vecPassiveJoints.reserve(r->_vecPassiveJoints.size());
    FOREACHC(itjoint, r->_vecPassiveJoints) {
        JointPtr pnewjoint(new Joint(shared_kinbody()));
        *pnewjoint = **itjoint; // be careful of copying pointers!
        pnewjoint->_parent = shared_kinbody();
        if( !!(*itjoint)->bodies[0] )
            pnewjoint->bodies[0] = _veclinks.at((*itjoint)->bodies[0]->GetIndex());
        if( !!(*itjoint)->bodies[1] )
            pnewjoint->bodies[1] = _veclinks.at((*itjoint)->bodies[1]->GetIndex());
        _vecPassiveJoints.push_back(pnewjoint);
    }

    _vDependencyOrderedJoints.resize(0); _vDependencyOrderedJoints.resize(r->_vDependencyOrderedJoints.size());
    FOREACHC(itjoint, r->_vDependencyOrderedJoints)
        _vDependencyOrderedJoints.push_back(_vecjoints.at((*itjoint)->GetJointIndex()));

    _vDOFOrderedJoints = r->_vDOFOrderedJoints;
    _vecJointIndices = r->_vecJointIndices;
    _vecJointHierarchy = r->_vecJointHierarchy;
    
    _setAdjacentLinks = r->_setAdjacentLinks;
    _setNonAdjacentLinks = r->_setNonAdjacentLinks;
    _vForcedAdjacentLinks = r->_vForcedAdjacentLinks;

    _listAttachedBodies.clear(); // will be set in the environment
    _listRegisteredCallbacks.clear(); // reset the callbacks

    _nUpdateStampId++; // update the stamp instead of copying
    return true;
}

void KinBody::_ParametersChanged(int parameters)
{
    _nUpdateStampId++;
    std::list<std::pair<int,boost::function<void()> > > listRegisteredCallbacks = _listRegisteredCallbacks; // copy since it can be changed
    FOREACH(itfns,listRegisteredCallbacks) {
        if( itfns->first & parameters )
            itfns->second();
    }
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
    o << _vecPassiveJoints.size() << " ";
    FOREACHC(it,_vecPassiveJoints) {
        (*it)->serialize(o,options);
    }
}

const std::string& KinBody::GetKinematicsGeometryHash() const
{
    CHECK_INTERNAL_COMPUTATION;
    return __hashkinematics;
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
