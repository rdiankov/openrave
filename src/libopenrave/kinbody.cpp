// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
#include <fparser.hh>
#include <algorithm>
#include <boost/algorithm/string.hpp> // boost::trim
#include <boost/lexical_cast.hpp>

// used for functions that are also used internally
#define CHECK_INTERNAL_COMPUTATION0 { \
        if( _nHierarchyComputed == 0 ) { \
            throw OPENRAVE_EXCEPTION_FORMAT("body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%_nHierarchyComputed, ORE_Failed); \
        } \
} \

#define CHECK_INTERNAL_COMPUTATION { \
        if( _nHierarchyComputed != 2 ) { \
            throw OPENRAVE_EXCEPTION_FORMAT("body %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%_nHierarchyComputed, ORE_Failed); \
        } \
} \

namespace OpenRAVE {

class ChangeCallbackData : public UserData
{
public:
    ChangeCallbackData(int properties, const boost::function<void()>& callback, KinBodyPtr pbody) : _properties(properties), _callback(callback), _pweakbody(pbody) {
    }
    virtual ~ChangeCallbackData() {
        KinBodyPtr pbody = _pweakbody.lock();
        if( !!pbody ) {
            pbody->_listRegisteredCallbacks.erase(_iterator);
        }
    }

    list<UserDataWeakPtr>::iterator _iterator;
    int _properties;
    boost::function<void()> _callback;
protected:
    KinBodyWeakPtr _pweakbody;
};

typedef boost::shared_ptr<ChangeCallbackData> ChangeCallbackDataPtr;

void KinBody::Link::TRIMESH::ApplyTransform(const Transform& t)
{
    FOREACH(it, vertices) {
        *it = t * *it;
    }
}

void KinBody::Link::TRIMESH::ApplyTransform(const TransformMatrix& t)
{
    FOREACH(it, vertices) {
        *it = t * *it;
    }
}

void KinBody::Link::TRIMESH::Append(const TRIMESH& mesh)
{
    int offset = (int)vertices.size();
    vertices.insert(vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
    if( indices.capacity() < indices.size()+mesh.indices.size() ) {
        indices.reserve(indices.size()+mesh.indices.size());
    }
    FOREACHC(it, mesh.indices) {
        indices.push_back(*it+offset);
    }
}

void KinBody::Link::TRIMESH::Append(const TRIMESH& mesh, const Transform& trans)
{
    int offset = (int)vertices.size();
    vertices.resize(vertices.size() + mesh.vertices.size());
    for(size_t i = 0; i < mesh.vertices.size(); ++i) {
        vertices[i+offset] = trans * mesh.vertices[i];
    }
    if( indices.capacity() < indices.size()+mesh.indices.size() ) {
        indices.reserve(indices.size()+mesh.indices.size());
    }
    FOREACHC(it, mesh.indices) {
        indices.push_back(*it+offset);
    }
}

AABB KinBody::Link::TRIMESH::ComputeAABB() const
{
    AABB ab;
    if( vertices.size() == 0 ) {
        return ab;
    }
    Vector vmin, vmax;
    vmin = vmax = vertices.at(0);
    FOREACHC(itv, vertices) {
        Vector v = *itv;
        if( vmin.x > v.x ) {
            vmin.x = v.x;
        }
        if( vmin.y > v.y ) {
            vmin.y = v.y;
        }
        if( vmin.z > v.z ) {
            vmin.z = v.z;
        }
        if( vmax.x < v.x ) {
            vmax.x = v.x;
        }
        if( vmax.y < v.y ) {
            vmax.y = v.y;
        }
        if( vmax.z < v.z ) {
            vmax.z = v.z;
        }
    }

    ab.extents = (dReal)0.5*(vmax-vmin);
    ab.pos = (dReal)0.5*(vmax+vmin);
    return ab;
}

void KinBody::Link::TRIMESH::serialize(std::ostream& o, int options) const
{
    o << vertices.size() << " ";
    FOREACHC(it,vertices) {
        o << it->x << " " << it->y << " " << it->z << " ";
    }
    o << indices.size() << " ";
    FOREACHC(it,indices) {
        o << *it << " ";
    }
}

std::ostream& operator<<(std::ostream& O, const KinBody::Link::TRIMESH& trimesh)
{
    trimesh.serialize(O,0);
    return O;
}

std::istream& operator>>(std::istream& I, KinBody::Link::TRIMESH& trimesh)
{
    trimesh.vertices.resize(0);
    trimesh.indices.resize(0);
    size_t s=0;
    I >> s;
    if( !I ) {
        return I;
    }
    trimesh.vertices.resize(s);
    FOREACH(it,trimesh.vertices) {
        I >> it->x >> it->y >> it->z;
    }
    I >> s;
    if( !I ) {
        return I;
    }
    trimesh.indices.resize(s);
    FOREACH(it,trimesh.indices) {
        I >> *it;
    }
    return I;
}

KinBody::Link::GEOMPROPERTIES::GEOMPROPERTIES(KinBody::LinkPtr parent) : _parent(parent)
{
    diffuseColor = Vector(1,1,1);
    _type = GeomNone;
    ftransparency = 0;
    vRenderScale = Vector(1,1,1);
    _bDraw = true;
    _bModifiable = true;
}

AABB KinBody::Link::GEOMPROPERTIES::ComputeAABB(const Transform& t) const
{
    AABB ab;
    TransformMatrix tglobal = t * _t;

    switch(_type) {
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
        ab.pos = tglobal.trans; //+(dReal)0.5*vGeomData.y*Vector(tglobal.m[2],tglobal.m[6],tglobal.m[10]);
        break;
    case GeomTrimesh:
        // just use collisionmesh
        if( collisionmesh.vertices.size() > 0) {
            Vector vmin, vmax; vmin = vmax = tglobal*collisionmesh.vertices.at(0);
            FOREACHC(itv, collisionmesh.vertices) {
                Vector v = tglobal * *itv;
                if( vmin.x > v.x ) {
                    vmin.x = v.x;
                }
                if( vmin.y > v.y ) {
                    vmin.y = v.y;
                }
                if( vmin.z > v.z ) {
                    vmin.z = v.z;
                }
                if( vmax.x < v.x ) {
                    vmax.x = v.x;
                }
                if( vmax.y < v.y ) {
                    vmax.y = v.y;
                }
                if( vmax.z < v.z ) {
                    vmax.z = v.z;
                }
            }
            ab.extents = (dReal)0.5*(vmax-vmin);
            ab.pos = (dReal)0.5*(vmax+vmin);
        }
        else {
            ab.pos = tglobal.trans;
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
    if( _type == KinBody::Link::GEOMPROPERTIES::GeomTrimesh ) {
        return true;
    }
    collisionmesh.indices.clear();
    collisionmesh.vertices.clear();

    if( fTessellation < 0.01f ) {
        fTessellation = 0.01f;
    }
    // start tesselating
    switch(_type) {
    case KinBody::Link::GEOMPROPERTIES::GeomSphere: {
        // log_2 (1+ tess)
        GenerateSphereTriangulation(collisionmesh, 3 + (int)(logf(fTessellation) / logf(2.0f)) );
        dReal fRadius = GetSphereRadius();
        FOREACH(it, collisionmesh.vertices) {
            *it *= fRadius;
        }
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
            0, 2, 1,
            1, 2, 3,
            4, 5, 6,
            5, 7, 6,
            0, 1, 4,
            1, 5, 4,
            2, 6, 3,
            3, 6, 7,
            0, 4, 2,
            2, 4, 6,
            1, 3, 5,
            3, 7, 5
        };
        collisionmesh.vertices.resize(8);
        std::copy(&v[0],&v[8],collisionmesh.vertices.begin());
        collisionmesh.indices.resize(nindices);
        std::copy(&indices[0],&indices[nindices],collisionmesh.indices.begin());
        break;
    }
    case KinBody::Link::GEOMPROPERTIES::GeomCylinder: {
        // cylinder is on z axis
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

            collisionmesh.indices.push_back(0);       collisionmesh.indices.push_back(off-2);       collisionmesh.indices.push_back(off);
            collisionmesh.indices.push_back(1);       collisionmesh.indices.push_back(off+1);       collisionmesh.indices.push_back(off-1);
            collisionmesh.indices.push_back(off-2);   collisionmesh.indices.push_back(off-1);         collisionmesh.indices.push_back(off);
            collisionmesh.indices.push_back(off);   collisionmesh.indices.push_back(off-1);         collisionmesh.indices.push_back(off+1);
        }
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("unrecognized geom type %d!", _type, ORE_InvalidArguments);
    }

    return true;
}

void KinBody::Link::GEOMPROPERTIES::serialize(std::ostream& o, int options) const
{
    SerializeRound(o,_t);
    o << _type << " ";
    SerializeRound3(o,vRenderScale);
    if( _type == GeomTrimesh ) {
        collisionmesh.serialize(o,options);
    }
    else {
        SerializeRound3(o,vGeomData);
    }
}

void KinBody::Link::GEOMPROPERTIES::SetCollisionMesh(const TRIMESH& mesh)
{
    if( !_bModifiable ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("geometry cannot be modified", ORE_Failed);
    }
    LinkPtr parent(_parent);
    collisionmesh = mesh;
    parent->_Update();
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

void KinBody::Link::GEOMPROPERTIES::SetDiffuseColor(const RaveVector<float>& color)
{
    LinkPtr parent(_parent);
    diffuseColor = color;
    parent->GetParent()->_ParametersChanged(Prop_LinkDraw);
}

void KinBody::Link::GEOMPROPERTIES::SetAmbientColor(const RaveVector<float>& color)
{
    LinkPtr parent(_parent);
    ambientColor = color;
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
    switch(_type) {
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
        if((zaxis < feps)&&(yaxis > -feps)) { // x-plane
            if( RaveFabs(tnormal.x) > RaveFabs(penetration) ) {
                penetration = tnormal.x;
            }
        }
        if((zaxis > -feps)&&(xaxis < feps)) { // y-plane
            if( RaveFabs(tnormal.y) > RaveFabs(penetration) ) {
                penetration = tnormal.y;
            }
        }
        if((yaxis < feps)&&(xaxis > -feps)) { // z-plane
            if( RaveFabs(tnormal.z) > RaveFabs(penetration) ) {
                penetration = tnormal.z;
            }
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
        if((fInsideCircle < -feps)&&(fInsideHeight > -feps)&&(normal.z*position.z<0)) {
            _normal = -_normal;
            return true;
        }
        if((fInsideCircle > -feps)&&(fInsideHeight < -feps)&&(normal.x*position.x+normal.y*position.y < 0)) {
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

void KinBody::Link::GEOMPROPERTIES::SetRenderFilename(const std::string& renderfilename)
{
    LinkPtr parent(_parent);
    _renderfilename = renderfilename;
    parent->GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

KinBody::Link::Link(KinBodyPtr parent)
{
    _parent = parent;
    _bStatic = false;
    _index = -1;
    _bIsEnabled = true;
}

KinBody::Link::~Link()
{
}

bool KinBody::Link::IsEnabled() const
{
    return _bIsEnabled;
}

void KinBody::Link::Enable(bool bEnable)
{
    if( _bIsEnabled != bEnable ) {
        KinBodyPtr parent = GetParent();
        if( !parent->GetEnv()->GetCollisionChecker()->EnableLink(LinkConstPtr(shared_from_this()),bEnable) ) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to enable link %s:%s", parent->GetName()%GetName(), ORE_Failed);
        }
        parent->_nNonAdjacentLinkCache &= ~AO_Enabled;
        _bIsEnabled = bEnable;
    }
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

AABB KinBody::Link::ComputeAABB() const
{
    if( _listGeomProperties.size() == 1) {
        return _listGeomProperties.front().ComputeAABB(_t);
    }
    else if( _listGeomProperties.size() > 1 ) {
        Vector vmin, vmax;
        bool binitialized=false;
        AABB ab;
        FOREACHC(itgeom,_listGeomProperties) {
            ab = itgeom->ComputeAABB(_t);
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
            ab.pos = _t.trans;
            ab.extents = Vector(0,0,0);
        }
        else {
            ab.pos = (dReal)0.5 * (vmin + vmax);
            ab.extents = vmax - ab.pos;
        }
        return ab;
    }
    // have to at least return the correct position!
    return AABB(_t.trans,Vector(0,0,0));
}

void KinBody::Link::serialize(std::ostream& o, int options) const
{
    o << _index << " ";
    if( options & SO_Geometry ) {
        o << _listGeomProperties.size() << " ";
        FOREACHC(it,_listGeomProperties) {
            it->serialize(o,options);
        }
    }
    if( options & SO_Dynamics ) {
        SerializeRound(o,_transMass);
        SerializeRound(o,_mass);
    }
}

void KinBody::Link::SetStatic(bool bStatic)
{
    if( _bStatic != bStatic ) {
        _bStatic = bStatic;
        GetParent()->_ParametersChanged(Prop_LinkStatic);
    }
}

void KinBody::Link::SetTransform(const Transform& t)
{
    _t = t;
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

KinBody::Link::GEOMPROPERTIES& KinBody::Link::GetGeometry(int index)
{
    BOOST_ASSERT(index >= 0 && index < (int)_listGeomProperties.size());
    std::list<GEOMPROPERTIES>::iterator it = _listGeomProperties.begin();
    advance(it,index);
    return *it;
}

void KinBody::Link::SwapGeometries(std::list<KinBody::Link::GEOMPROPERTIES>& listNewGeometries)
{
    LinkWeakPtr pnewlink;
    if( listNewGeometries.size() > 0 ) {
        pnewlink=listNewGeometries.front()._parent;
    }
    _listGeomProperties.swap(listNewGeometries);
    FOREACH(itgeom,_listGeomProperties) {
        itgeom->_parent = LinkWeakPtr(shared_from_this());
    }
    FOREACH(itgeom,listNewGeometries) {
        itgeom->_parent=pnewlink;
    }
    _Update();
    GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

bool KinBody::Link::ValidateContactNormal(const Vector& position, Vector& normal) const
{
    if( _listGeomProperties.size() == 1) {
        return _listGeomProperties.front().ValidateContactNormal(position,normal);
    }
    else if( _listGeomProperties.size() > 1 ) {
        RAVELOG_VERBOSE(str(boost::format("cannot validate normal when there is more then one geometry in link '%s(%d)' (do not know colliding geometry)")%_name%GetIndex()));
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

bool KinBody::Link::IsRigidlyAttached(boost::shared_ptr<Link const> plink) const
{
    return find(_vRigidlyAttachedLinks.begin(),_vRigidlyAttachedLinks.end(),plink->GetIndex()) != _vRigidlyAttachedLinks.end();
}

void KinBody::Link::_Update()
{
    collision.vertices.resize(0);
    collision.indices.resize(0);
    FOREACH(itgeom,_listGeomProperties) {
        collision.Append(itgeom->GetCollisionMesh(),itgeom->GetTransform());
    }
    GetParent()->_ParametersChanged(Prop_LinkGeometry);
}

KinBody::Joint::Joint(KinBodyPtr parent)
{
    _parent = parent;
    // fill _vaxes with valid values
    FOREACH(it,_vaxes) {
        *it = Vector(0,0,1);
    }
    FOREACH(it,_voffsets) {
        *it = 0;
    }
    fResolution = dReal(0.02);
    FOREACH(it,_vmaxvel) {
        *it = 10;
    }
    FOREACH(it,_vmaxaccel) {
        *it = 50;
    }
    FOREACH(it,_vmaxtorque) {
        *it = 1e5f;
    }
    jointindex=-1;
    dofindex = -1; // invalid index
    FOREACH(it,_bIsCircular) {
        *it = false;
    }
    _bActive = true;
    _bInitialized = false;
    _type = JointNone;
}

KinBody::Joint::~Joint()
{
}

int KinBody::Joint::GetDOF() const
{
    if( _type & JointSpecialBit ) {
        switch(_type) {
        case JointHinge2:
        case JointUniversal: return 2;
        case JointSpherical: return 3;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("invalid joint type 0x%x", _type, ORE_Failed);
        }
    }
    return int(_type & 0xf);
}

bool KinBody::Joint::IsRevolute(int iaxis) const
{
    if( _type & JointSpecialBit ) {
        return _type == JointHinge2 || _type == JointUniversal;
    }
    return !(_type&(1<<(4+iaxis)));
}

bool KinBody::Joint::IsPrismatic(int iaxis) const
{
    if( _type & JointSpecialBit ) {
        return false;
    }
    return !!(_type&(1<<(4+iaxis)));
}

bool KinBody::Joint::IsStatic() const
{
    if( IsMimic() ) {
        bool bstatic = true;
        KinBodyConstPtr parent(_parent);
        for(int i = 0; i < GetDOF(); ++i) {
            if( !!_vmimic.at(i) ) {
                FOREACHC(it, _vmimic.at(i)->_vmimicdofs) {
                    if( !parent->GetJointFromDOFIndex(it->dofindex)->IsStatic() ) {
                        bstatic = false;
                        break;
                    }
                }
                if( !bstatic ) {
                    break;
                }
            }
        }
        if( bstatic ) {
            return true;
        }
    }
    for(int i = 0; i < GetDOF(); ++i) {
        if( IsCircular(i) ) {
            return false;
        }
        if( _vlowerlimit.at(i) < _vupperlimit.at(i) ) {
            return false;
        }
    }
    return true;
}

void KinBody::Joint::GetValues(vector<dReal>& pValues, bool bAppend) const
{
    BOOST_ASSERT(_bInitialized);
    if( !bAppend ) {
        pValues.resize(0);
    }
    if( GetDOF() == 1 ) {
        pValues.push_back(GetValue(0));
        return;
    }
    dReal f;
    Transform tjoint = _tinvLeft * _attachedbodies[0]->GetTransform().inverse() * _attachedbodies[1]->GetTransform() * _tinvRight;
    if( _type & JointSpecialBit ) {
        switch(_type) {
        case JointHinge2: {
            Vector axis1cur = tjoint.rotate(_vaxes[0]), axis2cur = tjoint.rotate(_vaxes[1]);
            Vector vec1, vec2, vec3;
            vec1 = (_vaxes[1] - _vaxes[0].dot3(_vaxes[1])*_vaxes[0]).normalize();
            vec2 = (axis2cur - _vaxes[0].dot3(axis2cur)*_vaxes[0]).normalize();
            vec3 = _vaxes[0].cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            pValues.push_back(_voffsets[0]+f);
            vec1 = (_vaxes[0] - axis2cur.dot(_vaxes[0])*axis2cur).normalize();
            vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
            vec3 = axis2cur.cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            pValues.push_back(_voffsets[1]+f);
            break;
        }
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
            throw OPENRAVE_EXCEPTION_FORMAT("unknown joint type 0x%x", _type, ORE_Failed);
        }
    }
    else {
        // chain of revolute and prismatic joints
        for(int i = 0; i < GetDOF(); ++i) {
            Vector vaxis = _vaxes.at(i);
            if( IsRevolute(i) ) {
                if( i+1 < GetDOF() ) {
                    std::pair<dReal, Vector > res = normalizeAxisRotation(vaxis,tjoint.rot);
                    tjoint.rot = res.second;
                    if( res.first != 0 ) {
                        // could speed up by checking if trans is ever needed after this
                        tjoint.trans = quatRotate(quatFromAxisAngle(vaxis,res.first),tjoint.trans);
                    }
                    f = -res.first;
                }
                else {
                    f = 2.0f*RaveAtan2(tjoint.rot.y*vaxis.x+tjoint.rot.z*vaxis.y+tjoint.rot.w*vaxis.z, tjoint.rot.x);
                }
                // expect values to be within -PI to PI range
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                pValues.push_back(_voffsets[i]+f);
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                pValues.push_back(_voffsets[i]+f);
                if( i+1 < GetDOF() ) {
                    tjoint.trans -= vaxis*f;
                }
            }
        }
    }
}

dReal KinBody::Joint::GetValue(int iaxis) const
{
    BOOST_ASSERT(_bInitialized);
    dReal f;
    Transform tjoint = _tinvLeft * _attachedbodies[0]->GetTransform().inverse() * _attachedbodies[1]->GetTransform() * _tinvRight;
    if( _type & JointSpecialBit ) {
        switch(_type) {
        case JointHinge2: {
            Vector axis1cur = tjoint.rotate(_vaxes[0]), axis2cur = tjoint.rotate(_vaxes[1]);
            Vector vec1, vec2, vec3;
            if( iaxis == 0 ) {
                vec1 = (_vaxes[1] - _vaxes[0].dot3(_vaxes[1])*_vaxes[0]).normalize();
                vec2 = (axis2cur - _vaxes[0].dot3(axis2cur)*_vaxes[0]).normalize();
                vec3 = _vaxes[0].cross(vec1);
                f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                return _voffsets[0]+f;
            }
            else if( iaxis == 1 ) {
                vec1 = (_vaxes[0] - axis2cur.dot(_vaxes[0])*axis2cur).normalize();
                vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
                vec3 = axis2cur.cross(vec1);
                f = 2.0*RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                return _voffsets[1]+f;
            }
            break;
        }
        case JointSpherical: {
            dReal fsinang2 = tjoint.rot.y*tjoint.rot.y+tjoint.rot.z*tjoint.rot.z+tjoint.rot.w*tjoint.rot.w;
            if( fsinang2 > 1e-10f ) {
                dReal fsinang = RaveSqrt(fsinang2);
                dReal fmult = 2*RaveAtan2(fsinang,tjoint.rot.x)/fsinang;
                if( iaxis == 0 ) {
                    return tjoint.rot.y*fmult;
                }
                else if( iaxis == 1 ) {
                    return tjoint.rot.z*fmult;
                }
                else if( iaxis == 2 ) {
                    return tjoint.rot.w*fmult;
                }
            }
            else {
                if((iaxis >= 0)&&(iaxis < 3)) {
                    return 0;
                }
            }
            break;
        }
        default:
            break;
        }
    }
    else {
        if( _type == JointPrismatic ) {
            return _voffsets[0]+(tjoint.trans.x*_vaxes[0].x+tjoint.trans.y*_vaxes[0].y+tjoint.trans.z*_vaxes[0].z);
        }
        else if( _type == JointRevolute ) {
            f = 2.0f*RaveAtan2(tjoint.rot.y*_vaxes[0].x+tjoint.rot.z*_vaxes[0].y+tjoint.rot.w*_vaxes[0].z, tjoint.rot.x);
            // expect values to be within -PI to PI range
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            return _voffsets[0]+f;
        }

        // chain of revolute and prismatic joints
        for(int i = 0; i < GetDOF(); ++i) {
            Vector vaxis = _vaxes.at(i);
            if( IsRevolute(i) ) {
                if( i+1 < GetDOF() ) {
                    std::pair<dReal, Vector > res = normalizeAxisRotation(vaxis,tjoint.rot);
                    tjoint.rot = res.second;
                    if( res.first != 0 ) {
                        // could speed up by checking if trans is ever needed after this
                        tjoint.trans = quatRotate(quatFromAxisAngle(vaxis,res.first),tjoint.trans);
                    }
                    f = -res.first;
                }
                else {
                    f = 2.0f*RaveAtan2(tjoint.rot.y*vaxis.x+tjoint.rot.z*vaxis.y+tjoint.rot.w*vaxis.z, tjoint.rot.x);
                }
                // expect values to be within -PI to PI range
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                if( i == iaxis ) {
                    return _voffsets[i]+f;
                }
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                if( i == iaxis ) {
                    return _voffsets[i]+f;
                }
                if( i+1 < GetDOF() ) {
                    tjoint.trans -= vaxis*f;
                }
            }
        }
    }
    throw OPENRAVE_EXCEPTION_FORMAT("unknown joint type 0x%x axis %d\n", _type%iaxis, ORE_Failed);
}

void KinBody::Joint::GetVelocities(std::vector<dReal>& pVelocities, bool bAppend) const
{
    BOOST_ASSERT(_bInitialized);
    boost::array<Transform,2> transforms;
    boost::array<Vector,2> linearvel, angularvel;
    _attachedbodies[0]->GetVelocity(linearvel[0],angularvel[0]);
    _attachedbodies[1]->GetVelocity(linearvel[1],angularvel[1]);
    transforms[0] = _attachedbodies[0]->GetTransform();
    transforms[1] = _attachedbodies[1]->GetTransform();
    if( !bAppend ) {
        pVelocities.resize(0);
    }
    Vector quatdelta = quatMultiply(transforms[0].rot,_tLeft.rot);
    Vector quatdeltainv = quatInverse(quatdelta);
    if( _type & JointSpecialBit ) {
        switch(_type) {
        case JointSpherical: {
            Vector v = quatRotate(quatdeltainv,angularvel[1]-angularvel[0]);
            pVelocities.push_back(v.x);
            pVelocities.push_back(v.y);
            pVelocities.push_back(v.z);
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("unknown joint type 0x%x", _type, ORE_InvalidArguments);
        }
    }
    else {
        if( _type == JointPrismatic ) {
            pVelocities.push_back(_vaxes[0].dot3(quatRotate(quatdeltainv,linearvel[1]-linearvel[0]-angularvel[0].cross(transforms[1].trans-transforms[0].trans))));
        }
        else if( _type == JointRevolute ) {
            pVelocities.push_back(_vaxes[0].dot3(quatRotate(quatdeltainv,angularvel[1]-angularvel[0])));
        }
        else {
            // chain of revolute and prismatic joints
            Vector angvelocitycovered, linvelocitycovered;
            for(int i = 0; i < GetDOF(); ++i) {
                if( IsRevolute(i) ) {
                    pVelocities.push_back(_vaxes[i].dot3(quatRotate(quatdeltainv,angularvel[1]-angularvel[0]-angvelocitycovered)));
                    angvelocitycovered += quatRotate(quatdelta,_vaxes[i]*pVelocities.back());
                }
                else { // prismatic
                    pVelocities.push_back(_vaxes[i].dot3(quatRotate(quatdeltainv,linearvel[1]-linearvel[0]-(angularvel[0]-angvelocitycovered).cross(transforms[1].trans-transforms[0].trans)-linvelocitycovered)));
                    linvelocitycovered += quatRotate(quatdelta,_vaxes[i]*pVelocities.back());
                }
            }
        }
    }
}

Vector KinBody::Joint::GetAnchor() const
{
    return _attachedbodies[0]->GetTransform() * _tLeft.trans;
}

Vector KinBody::Joint::GetAxis(int iaxis) const
{
    return _attachedbodies[0]->GetTransform().rotate(_tLeft.rotate(_vaxes.at(iaxis)));
}

void KinBody::Joint::_ComputeInternalInformation(LinkPtr plink0, LinkPtr plink1, const Vector& vanchorraw, const std::vector<Vector>& vaxes, const std::vector<dReal>& vcurrentvalues)
{
    if( !plink0 || !plink1 ) {
        throw OPENRAVE_EXCEPTION_FORMAT("one or more attached _attachedbodies are invalid for joint %s", GetName(),ORE_InvalidArguments);
    }
    BOOST_ASSERT((int)vaxes.size()==GetDOF());

    KinBodyPtr parent(_parent);
    _bInitialized = false;
    _attachedbodies[0] = plink0;
    _attachedbodies[1] = plink1;
    Transform trel, tbody0, tbody1;
    Vector vanchor=vanchorraw;
    for(int i = 0; i < GetDOF(); ++i) {
        _vaxes[i] = vaxes[i];
    }
    // make sure first body is always closer to the root, unless the second body is static
    if( _attachedbodies[1]->IsStatic() ) {
        if( !_attachedbodies[0]->IsStatic() ) {
            Transform tswap = plink1->GetTransform().inverse() * plink0->GetTransform();
            for(int i = 0; i < GetDOF(); ++i) {
                _vaxes[i] = -tswap.rotate(_vaxes[i]);
            }
            vanchor = tswap*vanchor;
            swap(_attachedbodies[0],_attachedbodies[1]);
        }
    }

    if( _attachedbodies[1]->IsStatic() ) {
        RAVELOG_WARN(str(boost::format("joint %s: all attached links are static!\n")%GetName()));
    }

    tbody0 = _attachedbodies[0]->GetTransform();
    tbody1 = _attachedbodies[1]->GetTransform();
    trel = tbody0.inverse() * tbody1;
    _tLeft = Transform();
    _tLeftNoOffset = Transform();
    _tRight = Transform();
    _tRightNoOffset = Transform();

    if( _type & JointSpecialBit ) {
        switch(_type) {
        case JointUniversal:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            break;
        case JointHinge2:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            break;
        case JointSpherical:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("unrecognized joint type 0x%x", _type, ORE_InvalidArguments);
        }
    }
    else {
        _tLeftNoOffset.trans = vanchor;
        _tRightNoOffset.trans = -vanchor;
        _tRightNoOffset = _tRightNoOffset * trel;
        if( GetDOF() == 1 ) {
            // in the case of one axis, create a new coordinate system such that the axis rotates about (0,0,1)
            // this is necessary in order to simplify the rotation matrices (for future symbolic computation)
            // and suppress any floating-point error. The data structures are only setup for this to work in 1 DOF.
            Transform trot; trot.rot = quatRotateDirection(_vaxes[0],Vector(0,0,1));
            _tLeftNoOffset = _tLeftNoOffset * trot.inverse();
            _tRightNoOffset = trot*_tRightNoOffset;
            _vaxes[0] = Vector(0,0,1);
        }

        Transform toffset;
        if( IsRevolute(0) ) {
            toffset.rot = quatFromAxisAngle(_vaxes[0], _voffsets[0]);
        }
        else {
            toffset.trans = _vaxes[0]*_voffsets[0];
        }
        _tLeft = _tLeftNoOffset * toffset;
        _tRight = _tRightNoOffset;
        if( GetDOF() > 1 ) {
            // right multiply by the offset of the last axis, might be buggy?
            if( IsRevolute(GetDOF()-1) ) {
                _tRight = matrixFromAxisAngle(_vaxes[GetDOF()-1], _voffsets[GetDOF()-1]) * _tRight;
            }
            else {
                _tRight.trans += _vaxes[GetDOF()-1]*_voffsets[GetDOF()-1];
            }
        }
    }

    if( vcurrentvalues.size() > 0 ) {
        // see if any joints have offsets
        if( !(_type&JointSpecialBit) ||(_type != JointSpherical)) {
            Transform toffset;
            if( IsRevolute(0) ) {
                toffset.rot = quatFromAxisAngle(_vaxes[0], -vcurrentvalues[0]);
            }
            else {
                toffset.trans = -_vaxes[0]*vcurrentvalues[0];
            }
            _tLeftNoOffset *= toffset;
            _tLeft *= toffset;
            if( vcurrentvalues.size() > 1 ) {
                if( IsRevolute(GetDOF()-1) ) {
                    toffset.rot = quatFromAxisAngle(_vaxes[GetDOF()-1], -vcurrentvalues.at(GetDOF()-1));
                }
                else {
                    toffset.trans = -_vaxes[GetDOF()-1]*vcurrentvalues.at(GetDOF()-1);
                }
                _tRightNoOffset = toffset * _tRightNoOffset;
                _tRight = toffset * _tRight;
            }
        }
    }
    _tinvRight = _tRight.inverse();
    _tinvLeft = _tLeft.inverse();
    _bInitialized = true;
}

KinBody::LinkPtr KinBody::Joint::GetHierarchyParentLink() const
{
    return _attachedbodies[0];
}

KinBody::LinkPtr KinBody::Joint::GetHierarchyChildLink() const
{
    return _attachedbodies[1];
}

Vector KinBody::Joint::GetInternalHierarchyAnchor() const
{
    return _tLeftNoOffset.trans;
}

Vector KinBody::Joint::GetInternalHierarchyAxis(int iaxis) const
{
    return _vaxes.at(iaxis);
}

Transform KinBody::Joint::GetInternalHierarchyLeftTransform() const
{
    BOOST_ASSERT(_bInitialized);
    return _tLeftNoOffset;
}

Transform KinBody::Joint::GetInternalHierarchyRightTransform() const
{
    BOOST_ASSERT(_bInitialized);
    return _tRightNoOffset;
}

void KinBody::Joint::GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, bool bAppend) const
{
    if( !bAppend ) {
        vLowerLimit.resize(0);
        vUpperLimit.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vLowerLimit.push_back(_vlowerlimit[i]);
        vUpperLimit.push_back(_vupperlimit[i]);
    }
}

void KinBody::Joint::SetLimits(const std::vector<dReal>& vLowerLimit, const std::vector<dReal>& vUpperLimit)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _vlowerlimit[i] = vLowerLimit.at(i);
        _vupperlimit[i] = vUpperLimit.at(i);
    }
    GetParent()->_ParametersChanged(Prop_JointLimits);
}

void KinBody::Joint::GetVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper, bool bAppend) const
{
    if( !bAppend ) {
        vlower.resize(0);
        vupper.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vlower.push_back(-_vmaxvel[i]);
        vupper.push_back(_vmaxvel[i]);
    }
}

void KinBody::Joint::GetVelocityLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_vmaxvel[i]);
    }
}

void KinBody::Joint::SetVelocityLimits(const std::vector<dReal>& vmaxvel)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _vmaxvel[i] = vmaxvel.at(i);
    }
    GetParent()->_ParametersChanged(Prop_JointAccelerationVelocityLimits);
}

void KinBody::Joint::GetAccelerationLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_vmaxaccel[i]);
    }
}

void KinBody::Joint::SetAccelerationLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _vmaxaccel[i] = vmax.at(i);
    }
    GetParent()->_ParametersChanged(Prop_JointAccelerationVelocityLimits);
}

dReal KinBody::Joint::GetWeight(int iaxis) const
{
    return _vweights.at(iaxis);
}

void KinBody::Joint::SetWrapOffset(dReal newoffset, int iaxis)
{
    _voffsets.at(iaxis) = newoffset;
    if( iaxis == 0 ) {
        Transform toffset;
        if( IsRevolute(0) ) {
            toffset.rot = quatFromAxisAngle(_vaxes[0], newoffset);
        }
        else {
            toffset.trans = _vaxes[0]*newoffset;
        }
        _tLeft = _tLeftNoOffset * toffset;
        _tinvLeft = _tLeft.inverse();
    }
    if((GetDOF() > 1)&&(iaxis==GetDOF()-1)) {
        _tRight = _tRightNoOffset;
        // right multiply by the offset of the last axis, might be buggy?
        if( IsRevolute(GetDOF()-1) ) {
            _tRight = matrixFromAxisAngle(_vaxes[GetDOF()-1], newoffset) * _tRight;
        }
        else {
            _tRight.trans += _vaxes[GetDOF()-1]*newoffset;
        }
        _tinvRight = _tRight.inverse();
    }
    GetParent()->_ParametersChanged(Prop_JointOffset);
}

void KinBody::Joint::SetResolution(dReal resolution)
{
    fResolution = resolution;
    GetParent()->_ParametersChanged(Prop_JointProperties);
}

void KinBody::Joint::SetWeights(const std::vector<dReal>& vweights)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _vweights[i] = vweights.at(i);
    }
    GetParent()->_ParametersChanged(Prop_JointProperties);
}

void KinBody::Joint::AddTorque(const std::vector<dReal>& pTorques)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->AddJointTorque(shared_from_this(), pTorques);
}

int KinBody::Joint::GetMimicJointIndex() const
{
    for(int i = 0; i < GetDOF(); ++i) {
        if( !!_vmimic.at(i) &&(_vmimic.at(i)->_vmimicdofs.size() > 0)) {
            return GetParent()->GetJointFromDOFIndex(_vmimic.at(i)->_vmimicdofs.front().dofindex)->GetJointIndex();
        }
    }
    return -1;
}

const std::vector<dReal> KinBody::Joint::GetMimicCoeffs() const
{
    RAVELOG_WARN("deprecated KinBody::Joint::GetMimicCoeffs(): could not deduce coeffs\n");
    std::vector<dReal> coeffs(2); coeffs[0] = 1; coeffs[1] = 0;
    return coeffs;
}

bool KinBody::Joint::IsMimic(int iaxis) const
{
    if( iaxis >= 0 ) {
        return !!_vmimic.at(iaxis);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        if( !!_vmimic.at(i) ) {
            return true;
        }
    }
    return false;
}

std::string KinBody::Joint::GetMimicEquation(int iaxis, int itype, const std::string& format) const
{
    if( !_vmimic.at(iaxis) ) {
        return "";
    }
    if((format.size() == 0)||(format == "fparser")) {
        return _vmimic.at(iaxis)->_equations.at(itype);
    }
    else if( format == "mathml" ) {
        boost::format mathfmt("<math xmlns=\"http://www.w3.org/1998/Math/MathML\">\n%s</math>\n");
        std::vector<std::string> Vars;
        std::string sout;
        KinBodyConstPtr parent(_parent);
        FOREACHC(itdofformat, _vmimic.at(iaxis)->_vdofformat) {
            JointConstPtr pjoint = itdofformat->GetJoint(parent);
            if( pjoint->GetDOF() > 1 ) {
                Vars.push_back(str(boost::format("<csymbol>%s_%d</csymbol>")%pjoint->GetName()%(int)itdofformat->axis));
            }
            else {
                Vars.push_back(str(boost::format("<csymbol>%s</csymbol>")%pjoint->GetName()));
            }
        }
        if( itype == 0 ) {
            _vmimic.at(iaxis)->_posfn->toMathML(sout,Vars);
            if((sout.size() > 9)&&(sout.substr(0,9) == "<csymbol>")) {
                // due to a bug in ROS robot_model, have to return with <apply> (remove this in 2012).
                sout = boost::str(boost::format("<apply>\n  <plus/><cn type=\"real\">0</cn>\n  %s\n  </apply>")%sout);
            }
            sout = str(mathfmt%sout);
        }
        else if( itype == 1 ) {
            std::string stemp;
            FOREACHC(itfn, _vmimic.at(iaxis)->_velfns) {
                (*itfn)->toMathML(stemp,Vars);
                sout += str(mathfmt%stemp);
            }
        }
        else if( itype == 2 ) {
            std::string stemp;
            FOREACHC(itfn, _vmimic.at(iaxis)->_accelfns) {
                (*itfn)->toMathML(stemp,Vars);
                sout += str(mathfmt%stemp);
            }
        }
        return sout;
    }
    throw OPENRAVE_EXCEPTION_FORMAT("unsupported math format %s", format, ORE_InvalidArguments);
}

void KinBody::Joint::GetMimicDOFIndices(std::vector<int>& vmimicdofs, int iaxis) const
{
    if( !_vmimic.at(iaxis) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("joint %s axis %d is not mimic", GetName()%iaxis,ORE_InvalidArguments);
    }
    vmimicdofs.resize(0);
    FOREACHC(it, _vmimic.at(iaxis)->_vmimicdofs) {
        std::vector<int>::iterator itinsert = std::lower_bound(vmimicdofs.begin(),vmimicdofs.end(), it->dofindex);
        if((itinsert == vmimicdofs.end())||(*itinsert != it->dofindex)) {
            vmimicdofs.insert(itinsert,it->dofindex);
        }
    }
}

void KinBody::Joint::SetMimicEquations(int iaxis, const std::string& poseq, const std::string& veleq, const std::string& acceleq)
{
    _vmimic.at(iaxis).reset();
    if( poseq.size() == 0 ) {
        return;
    }
    KinBodyPtr parent(_parent);
    std::vector<std::string> resultVars;
    boost::shared_ptr<MIMIC> mimic(new MIMIC());
    mimic->_equations.at(0) = poseq;
    mimic->_equations.at(1) = veleq;
    mimic->_equations.at(2) = acceleq;

    mimic->_posfn = parent->_CreateFunctionParser();
    // because openrave joint names can hold symbols like '-' and '.' can affect the equation, so first do a search and replace
    std::vector< std::pair<std::string, std::string> > jointnamepairs; jointnamepairs.reserve(parent->GetJoints().size());
    FOREACHC(itjoint,parent->GetJoints()) {
        if( (*itjoint)->GetName().size() > 0 ) {
            jointnamepairs.push_back(make_pair((*itjoint)->GetName(),str(boost::format("joint%d")%(*itjoint)->GetJointIndex())));
        }
    }
    size_t index = parent->GetJoints().size();
    FOREACHC(itjoint,parent->GetPassiveJoints()) {
        if( (*itjoint)->GetName().size() > 0 ) {
            jointnamepairs.push_back(make_pair((*itjoint)->GetName(),str(boost::format("joint%d")%index)));
        }
        ++index;
    }

    std::map<std::string,std::string> mapinvnames;
    FOREACH(itpair,jointnamepairs) {
        mapinvnames[itpair->second] = itpair->first;
    }

    std::string eq;
    int ret = mimic->_posfn->ParseAndDeduceVariables(SearchAndReplace(eq,mimic->_equations[0],jointnamepairs),resultVars);
    if( ret >= 0 ) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to set equation '%s' on %s:%s, at %d. Error is %s\n", mimic->_equations[0]%parent->GetName()%GetName()%ret%mimic->_posfn->ErrorMsg(),ORE_InvalidArguments);
    }
    // process the variables
    FOREACH(itvar,resultVars) {
        if( itvar->find("joint") != 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT("equation '%s' uses unknown variable", mimic->_equations[0], ORE_InvalidArguments);
        }
        MIMIC::DOFFormat dofformat;
        size_t axisindex = itvar->find('_');
        if( axisindex != std::string::npos ) {
            dofformat.jointindex = boost::lexical_cast<uint16_t>(itvar->substr(5,axisindex-5));
            dofformat.axis = boost::lexical_cast<uint8_t>(itvar->substr(axisindex+1));
        }
        else {
            dofformat.jointindex = boost::lexical_cast<uint16_t>(itvar->substr(5));
            dofformat.axis = 0;
        }
        dofformat.dofindex = -1;
        JointPtr pjoint = dofformat.GetJoint(parent);
        if((pjoint->GetDOFIndex() >= 0)&& !pjoint->IsMimic(dofformat.axis) ) {
            dofformat.dofindex = pjoint->GetDOFIndex()+dofformat.axis;
            MIMIC::DOFHierarchy h;
            h.dofindex = dofformat.dofindex;
            h.dofformatindex = mimic->_vdofformat.size();
            mimic->_vmimicdofs.push_back(h);
        }
        mimic->_vdofformat.push_back(dofformat);
    }

    stringstream sVars;
    if( !resultVars.empty() ) {
        sVars << resultVars.at(0);
        for(size_t i = 1; i < resultVars.size(); ++i) {
            sVars << "," << resultVars[i];
        }
    }

    for(int itype = 1; itype < 3; ++itype) {
        if((itype == 2)&&(mimic->_equations[itype].size() == 0)) {
            continue;
        }

        std::vector<boost::shared_ptr<FunctionParserBase<dReal> > > vfns(resultVars.size());
        // extract the equations
        SearchAndReplace(eq,mimic->_equations[itype],jointnamepairs);
        size_t index = eq.find('|');
        while(index != std::string::npos) {
            size_t startindex = index+1;
            index = eq.find('|',startindex);
            string sequation;
            if( index != std::string::npos) {
                sequation = eq.substr(startindex,index-startindex);
            }
            else {
                sequation = eq.substr(startindex);
            }
            boost::trim(sequation);
            size_t nameendindex = sequation.find(' ');
            string varname;
            if( nameendindex == std::string::npos ) {
                RAVELOG_WARN(str(boost::format("invalid equation syntax '%s' for joint %s")%sequation%_name));
                varname = sequation;
                sequation = "0";
            }
            else {
                varname = sequation.substr(0,nameendindex);
                sequation = sequation.substr(nameendindex);
            }
            vector<string>::iterator itnameindex = find(resultVars.begin(),resultVars.end(),varname);
            if( itnameindex == resultVars.end() ) {
                throw OPENRAVE_EXCEPTION_FORMAT("variable %s from velocity equation is not referenced in the position, skipping...", mapinvnames[varname],ORE_InvalidArguments);
            }

            boost::shared_ptr<FunctionParserBase<dReal> > fn(parent->_CreateFunctionParser());
            ret = fn->Parse(sequation,sVars.str());
            if( ret >= 0 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("failed to set equation '%s' on %s:%s, at %d. Error is %s", sequation%parent->GetName()%GetName()%ret%fn->ErrorMsg(),ORE_InvalidArguments);
            }
            vfns.at(itnameindex-resultVars.begin()) = fn;
        }
        // check if anything is missing
        for(size_t j = 0; j < resultVars.size(); ++j) {
            if( !vfns[j] ) {
                // print a message instead of throwing an exception since it might be common for only position equations to be specified
                RAVELOG_WARN(str(boost::format("SetMimicEquations: missing variable %s from partial derivatives of joint %s!")%mapinvnames[resultVars[j]]%_name));
                vfns[j] = parent->_CreateFunctionParser();
                vfns[j]->Parse("0","");
            }
        }

        if( itype == 1 ) {
            mimic->_velfns.swap(vfns);
        }
        else {
            mimic->_accelfns.swap(vfns);
        }
    }
    _vmimic.at(iaxis) = mimic;
    parent->_ParametersChanged(Prop_JointMimic);
}

void KinBody::Joint::_ComputePartialVelocities(std::vector<std::pair<int,dReal> >& vpartials, int iaxis, std::map< std::pair<MIMIC::DOFFormat, int>, dReal >& mapcachedpartials) const
{
    vpartials.resize(0);
    if( dofindex >= 0 ) {
        vpartials.push_back(make_pair(dofindex+iaxis,1.0));
        return;
    }
    if( !_vmimic.at(iaxis) ) {
        throw OPENRAVE_EXCEPTION_FORMAT("cannot compute partial velocities of joint %s", _name, ORE_Failed);
    }
    KinBodyConstPtr parent(_parent);
    MIMIC::DOFFormat thisdofformat;
    thisdofformat.dofindex = -1; // always -1 since it is mimiced
    thisdofformat.axis = iaxis;
    thisdofformat.jointindex = jointindex;
    if( jointindex < 0 ) {
        // this is a weird computation... have to figure out the passive joint index given where it is in parent->GetPassiveJoints()
        thisdofformat.jointindex = parent->GetJoints().size() + (find(parent->GetPassiveJoints().begin(),parent->GetPassiveJoints().end(),shared_from_this()) - parent->GetPassiveJoints().begin());
    }
    std::vector<std::pair<int,dReal> > vtemppartials;
    vector<dReal> vtempvalues;
    FOREACHC(itmimicdof, _vmimic[iaxis]->_vmimicdofs) {
        std::pair<MIMIC::DOFFormat, int> key = make_pair(thisdofformat,itmimicdof->dofindex);
        std::map< std::pair<MIMIC::DOFFormat, int>, dReal >::iterator it = mapcachedpartials.find(key);
        if( it == mapcachedpartials.end() ) {
            // not in the cache so compute using the chain rule
            if( vtempvalues.empty() ) {
                FOREACHC(itdofformat, _vmimic[iaxis]->_vdofformat) {
                    vtempvalues.push_back(itdofformat->GetJoint(parent)->GetValue(itdofformat->axis));
                }
            }
            dReal fvel = _vmimic[iaxis]->_velfns.at(itmimicdof->dofformatindex)->Eval(vtempvalues.empty() ? NULL : &vtempvalues[0]);
            const MIMIC::DOFFormat& dofformat = _vmimic[iaxis]->_vdofformat.at(itmimicdof->dofformatindex);
            if( dofformat.GetJoint(parent)->IsMimic(dofformat.axis) ) {
                dofformat.GetJoint(parent)->_ComputePartialVelocities(vtemppartials,dofformat.axis,mapcachedpartials);
                dReal fpartial = 0;
                FOREACHC(itpartial,vtemppartials) {
                    if( itpartial->first == itmimicdof->dofindex ) {
                        fpartial += itpartial->second;
                    }
                }
                fvel *= fpartial;
            }
            // before pushing back, check for repetition
            bool badd = true;
            FOREACH(itpartial,vpartials) {
                if( itpartial->first == itmimicdof->dofindex ) {
                    itpartial->second += fvel;
                    badd = false;
                    break;
                }
            }
            if( badd ) {
                vpartials.push_back(make_pair(itmimicdof->dofindex, fvel));
            }
        }
        else {
            bool badd = true;
            FOREACH(itpartial,vpartials) {
                if( itpartial->first == itmimicdof->dofindex ) {
                    badd = false;
                    break;
                }
            }
            if( badd ) {
                vpartials.push_back(make_pair(itmimicdof->dofindex, it->second));
            }
        }
    }
}

bool KinBody::Joint::MIMIC::DOFFormat::operator <(const KinBody::Joint::MIMIC::DOFFormat& r) const
{
    return jointindex < r.jointindex || (jointindex == r.jointindex && (dofindex < r.dofindex || (dofindex == r.dofindex && axis < r.axis)));
}

bool KinBody::Joint::MIMIC::DOFFormat::operator ==(const KinBody::Joint::MIMIC::DOFFormat& r) const
{
    return jointindex == r.jointindex && dofindex == r.dofindex && axis == r.axis;
}

KinBody::JointPtr KinBody::Joint::MIMIC::DOFFormat::GetJoint(KinBodyPtr parent) const
{
    int numjoints = (int)parent->GetJoints().size();
    return jointindex < numjoints ? parent->GetJoints().at(jointindex) : parent->GetPassiveJoints().at(jointindex-numjoints);
}

KinBody::JointConstPtr KinBody::Joint::MIMIC::DOFFormat::GetJoint(KinBodyConstPtr parent) const
{
    int numjoints = (int)parent->GetJoints().size();
    return jointindex < numjoints ? parent->GetJoints().at(jointindex) : parent->GetPassiveJoints().at(jointindex-numjoints);
}

void KinBody::Joint::serialize(std::ostream& o, int options) const
{
    if( options & SO_Kinematics ) {
        o << dofindex << " " << jointindex << " " << _type << " ";
        SerializeRound(o,_tRightNoOffset);
        SerializeRound(o,_tLeftNoOffset);
        for(int i = 0; i < GetDOF(); ++i) {
            SerializeRound3(o,_vaxes[i]);
            if( !!_vmimic.at(i) ) {
                FOREACHC(iteq,_vmimic.at(i)->_equations) {
                    o << *iteq << " ";
                }
            }
        }
        o << (!_attachedbodies[0] ? -1 : _attachedbodies[0]->GetIndex()) << " " << (_attachedbodies[1]->GetIndex()) << " ";
    }
    if( options & SO_Dynamics ) {
        for(int i = 0; i < GetDOF(); ++i) {
            SerializeRound(o,_vmaxvel[i]);
            SerializeRound(o,_vmaxaccel[i]);
            SerializeRound(o,_vmaxtorque[i]);
            SerializeRound(o,_vlowerlimit[i]);
            SerializeRound(o,_vupperlimit[i]);
        }
    }
}

KinBody::KinBodyStateSaver::KinBodyStateSaver(KinBodyPtr pbody, int options) : _options(options), _pbody(pbody)
{
    if( _options & Save_LinkTransformation ) {
        _pbody->GetLinkTransformations(_vLinkTransforms);
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
}

KinBody::KinBodyStateSaver::~KinBodyStateSaver()
{
    _RestoreKinBody();
}

void KinBody::KinBodyStateSaver::Restore()
{
    _RestoreKinBody();
}

void KinBody::KinBodyStateSaver::_RestoreKinBody()
{
    if( _options & Save_LinkTransformation ) {
        _pbody->SetLinkTransformations(_vLinkTransforms);
    }
    if( _options & Save_LinkEnable ) {
        for(size_t i = 0; i < _vEnabledLinks.size(); ++i) {
            if( _pbody->GetLinks().at(i)->IsEnabled() != !!_vEnabledLinks[i] ) {
                _pbody->GetLinks().at(i)->Enable(!!_vEnabledLinks[i]);
            }
        }
    }
    if( _options & Save_LinkVelocities ) {
        _pbody->SetLinkVelocities(_vLinkVelocities);
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
        stringstream ss; ss << GetName() << " still has attached bodies: ";
        FOREACHC(it,_listAttachedBodies) {
            KinBodyPtr pattached = it->lock();
            if( !!pattached ) {
                ss << pattached->GetName();
            }
        }
        RAVELOG_WARN(ss.str());
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
    _pViewerData.reset();
    _pPhysicsData.reset();
    _pCollisionData.reset();
    _pManageData.reset();

    _ResetInternalCollisionCache();
}

bool KinBody::InitFromFile(const std::string& filename, const AttributesList& atts)
{
    bool bSuccess = GetEnv()->ReadKinBodyURI(shared_kinbody(), filename, atts)==shared_kinbody();
    if( !bSuccess ) {
        Destroy();
        return false;
    }
    return true;
}

bool KinBody::InitFromData(const std::string& data, const AttributesList& atts)
{
    bool bSuccess = GetEnv()->ReadKinBodyData(shared_kinbody(), data, atts)==shared_kinbody();
    if( !bSuccess ) {
        Destroy();
        return false;
    }
    return true;
}

bool KinBody::InitFromBoxes(const std::vector<AABB>& vaabbs, bool bDraw)
{
    if( GetEnvironmentId() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    }
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_name = "base";
    plink->_bStatic = true;
    size_t numvertices=0, numindices=0;
    FOREACHC(itab, vaabbs) {
        plink->_listGeomProperties.push_back(Link::GEOMPROPERTIES(plink));
        Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        geom._type = Link::GEOMPROPERTIES::GeomBox;
        geom._t.trans = itab->pos;
        geom._bDraw = bDraw;
        geom.vGeomData = itab->extents;
        geom.InitCollisionMesh();
        geom.diffuseColor=Vector(1,0.5f,0.5f,1);
        geom.ambientColor=Vector(0.1,0.0f,0.0f,0);
        numvertices += geom.GetCollisionMesh().vertices.size();
        numindices += geom.GetCollisionMesh().indices.size();
    }

    plink->collision.vertices.reserve(numvertices);
    plink->collision.indices.reserve(numindices);
    Link::TRIMESH trimesh;
    FOREACH(itgeom,plink->_listGeomProperties) {
        trimesh = itgeom->GetCollisionMesh();
        trimesh.ApplyTransform(itgeom->_t);
        plink->collision.Append(trimesh);
    }
    _veclinks.push_back(plink);
    return true;
}

bool KinBody::InitFromBoxes(const std::vector<OBB>& vobbs, bool bDraw)
{
    if( GetEnvironmentId() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    }
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_name = "base";
    plink->_bStatic = true;
    size_t numvertices=0, numindices=0;
    FOREACHC(itobb, vobbs) {
        plink->_listGeomProperties.push_back(Link::GEOMPROPERTIES(plink));
        Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        geom._type = Link::GEOMPROPERTIES::GeomBox;
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
        numvertices += geom.GetCollisionMesh().vertices.size();
        numindices += geom.GetCollisionMesh().indices.size();
    }

    plink->collision.vertices.reserve(numvertices);
    plink->collision.indices.reserve(numindices);
    Link::TRIMESH trimesh;
    FOREACH(itgeom,plink->_listGeomProperties) {
        trimesh = itgeom->GetCollisionMesh();
        trimesh.ApplyTransform(itgeom->_t);
        plink->collision.Append(trimesh);
    }

    _veclinks.push_back(plink);
    return true;
}

bool KinBody::InitFromSpheres(const std::vector<Vector>& vspheres, bool bDraw)
{
    if( GetEnvironmentId() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("%s: cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    }
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_name = "base";
    plink->_bStatic = true;
    Link::TRIMESH trimesh;
    FOREACHC(itv, vspheres) {
        plink->_listGeomProperties.push_back(Link::GEOMPROPERTIES(plink));
        Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        geom._type = Link::GEOMPROPERTIES::GeomSphere;
        geom._t.trans.x = itv->x; geom._t.trans.y = itv->y; geom._t.trans.z = itv->z;
        geom._bDraw = bDraw;
        geom.vGeomData.x = itv->w;
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

bool KinBody::InitFromTrimesh(const KinBody::Link::TRIMESH& trimesh, bool draw)
{
    if( GetEnvironmentId() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("%s, cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    }
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_name = "base";
    plink->_bStatic = true;
    plink->collision = trimesh;

    plink->_listGeomProperties.push_back(Link::GEOMPROPERTIES(plink));
    Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
    geom._type = Link::GEOMPROPERTIES::GeomTrimesh;
    geom._bDraw = draw;
    geom.collisionmesh = trimesh;
    geom.diffuseColor=Vector(1,0.5f,0.5f,1);
    geom.ambientColor=Vector(0.1,0.0f,0.0f,0);
    _veclinks.push_back(plink);
    return true;
}

bool KinBody::InitFromGeometries(std::list<KinBody::Link::GEOMPROPERTIES>& listGeometries, bool draw)
{
    if( GetEnvironmentId() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("%s, cannot Init a body while it is added to the environment", GetName(), ORE_Failed);
    }
    BOOST_ASSERT(listGeometries.size()>0);
    Destroy();
    LinkPtr plink(new Link(shared_kinbody()));
    plink->_index = 0;
    plink->_name = "base";
    plink->_bStatic = true;
    plink->_listGeomProperties.splice(plink->_listGeomProperties.end(),listGeometries);
    FOREACH(itgeom,plink->_listGeomProperties) {
        itgeom->_bDraw = draw;
        itgeom->_parent = plink;
        plink->collision.Append(itgeom->GetCollisionMesh(),itgeom->_t);
    }
    _veclinks.push_back(plink);
    return true;
}

void KinBody::SetName(const std::string& newname)
{
    BOOST_ASSERT(newname.size() > 0);
    if( _name != newname ) {
        _name = newname;
        _ParametersChanged(Prop_Name);
    }
}

void KinBody::SetDOFTorques(const std::vector<dReal>& torques, bool bAdd)
{
    if( (int)torques.size() < GetDOF() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("not enough values %d<%d", torques.size()%GetDOF(),ORE_InvalidArguments);
    }
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

void KinBody::GetDOFValues(std::vector<dReal>& v) const
{
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
            BOOST_ASSERT(0);
        }
        (*it)->GetValues(v,true);
    }
}

void KinBody::GetDOFVelocities(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() ) {
        v.reserve(GetDOF());
    }
    FOREACHC(it, _vDOFOrderedJoints) {
        (*it)->GetVelocities(v,true);
    }
}

void KinBody::GetDOFLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const
{
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

void KinBody::GetDOFVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper) const
{
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

void KinBody::GetDOFVelocityLimits(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() ) {
        v.reserve(GetDOF());
    }
    FOREACHC(it, _vDOFOrderedJoints) {
        (*it)->GetVelocityLimits(v,true);
    }
}

void KinBody::GetDOFAccelerationLimits(std::vector<dReal>& v) const
{
    v.resize(0);
    if( (int)v.capacity() < GetDOF() ) {
        v.reserve(GetDOF());
    }
    FOREACHC(it, _vDOFOrderedJoints) {
        (*it)->GetAccelerationLimits(v,true);
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

void KinBody::SimulationStep(dReal fElapsedTime)
{
}

void KinBody::SubtractDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    FOREACHC(itjoint,_vecjoints) {
        int dof = (*itjoint)->GetDOFIndex();
        for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
            if( (*itjoint)->IsCircular(i) ) {
                q1.at(dof+i) = ANGLE_DIFF(q1.at(dof+i), q2.at(dof+i));
            }
            else {
                q1.at(dof+i) -= q2.at(dof+i);
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

void KinBody::SetDOFVelocities(const std::vector<dReal>& vDOFVelocity, const Vector& linearvel, const Vector& angularvel, bool checklimits)
{
    CHECK_INTERNAL_COMPUTATION;
    if( (int)vDOFVelocity.size() < GetDOF() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("not enough values %d!=%d", vDOFVelocity.size()%GetDOF(),ORE_InvalidArguments);
    }
    std::vector<std::pair<Vector,Vector> > velocities(_veclinks.size());
    velocities.at(0).first = linearvel;
    velocities.at(0).second = angularvel;

    vector<dReal> vlower,vupper,vtempvalues;
    if( checklimits ) {
        GetDOFVelocityLimits(vlower,vupper);
    }

    // have to compute the velocities ahead of time since they are dependent on the link transformations
    std::vector< std::vector<dReal> > vPassiveJointVelocities(_vPassiveJoints.size());
    for(size_t i = 0; i < vPassiveJointVelocities.size(); ++i) {
        if( !_vPassiveJoints[i]->IsMimic() ) {
            _vPassiveJoints[i]->GetValues(vPassiveJointVelocities[i]);
        }
        else {
            vPassiveJointVelocities[i].resize(_vPassiveJoints[i]->GetDOF());
        }
    }

    std::vector<uint8_t> vlinkscomputed(_veclinks.size(),0);
    vlinkscomputed[0] = 1;
    boost::array<dReal,3> dummyvalues; // dummy values for a joint

    for(size_t i = 0; i < _vTopologicallySortedJointsAll.size(); ++i) {
        JointPtr pjoint = _vTopologicallySortedJointsAll[i];
        int jointindex = _vTopologicallySortedJointIndicesAll[i];
        int dofindex = pjoint->GetDOFIndex();
        const dReal* pvalues=dofindex >= 0 ? &vDOFVelocity.at(dofindex) : NULL;
        if( pjoint->IsMimic() ) {
            for(int i = 0; i < pjoint->GetDOF(); ++i) {
                if( pjoint->IsMimic(i) ) {
                    vtempvalues.resize(0);
                    const std::vector<Joint::MIMIC::DOFFormat>& vdofformat = pjoint->_vmimic[i]->_vdofformat;
                    FOREACHC(itdof,vdofformat) {
                        JointPtr pj = itdof->jointindex < (int)_vecjoints.size() ? _vecjoints[itdof->jointindex] : _vPassiveJoints.at(itdof->jointindex-_vecjoints.size());
                        vtempvalues.push_back(pj->GetValue(itdof->axis));
                    }
                    dummyvalues[i] = 0;
                    int varindex = 0;
                    FOREACH(itfn, pjoint->_vmimic.at(i)->_velfns) {
                        dReal partialvelocity;
                        if( vdofformat[varindex].dofindex >= 0 ) {
                            partialvelocity = vDOFVelocity.at(vdofformat[varindex].dofindex);
                        }
                        else {
                            partialvelocity = vPassiveJointVelocities.at(vdofformat[varindex].jointindex-_vecjoints.size()).at(vdofformat[varindex].axis);
                        }
                        dummyvalues[i] += (*itfn)->Eval(vtempvalues.empty() ? NULL : &vtempvalues[0]) * partialvelocity;
                        ++varindex;
                    }

                    // if joint is passive, update the stored joint values! This is necessary because joint value might be referenced in the future.
                    if( dofindex < 0 ) {
                        vPassiveJointVelocities.at(jointindex-(int)_vecjoints.size()).at(i) = dummyvalues[i];
                    }
                }
                else if( dofindex >= 0 ) {
                    dummyvalues[i] = vDOFVelocity.at(dofindex+i); // is this correct? what is a joint has a mimic and non-mimic axis?
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
                    dummyvalues[i] = vlower[i];
                }
                else if( pvalues[i] > vupper.at(dofindex+i) ) {
                    dummyvalues[i] = vupper[i];
                }
                else {
                    dummyvalues[i] = pvalues[i];
                }
            }
            pvalues = &dummyvalues[0];
        }

        Vector w,v;
        if( pjoint->GetType() & Joint::JointSpecialBit ) {
            switch(pjoint->GetType()) {
            case Joint::JointHinge2: {
                Transform tfirst;
                tfirst.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pjoint->GetValue(0));
                w = pvalues[0]*pjoint->GetInternalHierarchyAxis(0) + tfirst.rotate(pvalues[1]*pjoint->GetInternalHierarchyAxis(1));
                break;
            }
            case Joint::JointSpherical:
                w.x = pvalues[0]; w.y = pvalues[1]; w.z = pvalues[2];
                break;
            default:
                RAVELOG_WARN(str(boost::format("forward kinematic type %d not supported")%pjoint->GetType()));
                break;
            }
        }
        else {
            if( pjoint->GetType() == Joint::JointRevolute ) {
                w = pvalues[0]*pjoint->GetInternalHierarchyAxis(0);
            }
            else if( pjoint->GetType() == Joint::JointPrismatic ) {
                v = pvalues[0]*pjoint->GetInternalHierarchyAxis(0);
            }
            else {
                //todo
                Transform tjoint;
                for(int iaxis = 0; iaxis < pjoint->GetDOF(); ++iaxis) {
                    Transform tdelta;
                    if( pjoint->IsRevolute(iaxis) ) {
                        w += tjoint.rotate(pvalues[iaxis]*pjoint->GetInternalHierarchyAxis(iaxis));
                        tdelta.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(iaxis), pvalues[iaxis]);
                    }
                    else {
                        tdelta.trans = pjoint->GetInternalHierarchyAxis(iaxis) * pvalues[iaxis];
                        v += tjoint.rotate(pvalues[iaxis]*pjoint->GetInternalHierarchyAxis(iaxis)) + w.cross(tdelta.trans);
                    }
                    tjoint = tjoint * tdelta;
                }
            }
        }

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
        Transform tdelta = tparent * pjoint->GetInternalHierarchyLeftTransform();
        velocities.at(pjoint->GetHierarchyChildLink()->GetIndex()) = make_pair(vparent + tdelta.rotate(v) + wparent.cross(pjoint->GetHierarchyChildLink()->GetTransform().trans-tparent.trans), wparent + tdelta.rotate(w));
        vlinkscomputed[pjoint->GetHierarchyChildLink()->GetIndex()] = 1;
    }
    SetLinkVelocities(velocities);
}

void KinBody::SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, bool checklimits)
{
    Vector linearvel,angularvel;
    _veclinks.at(0)->GetVelocity(linearvel,angularvel);
    return SetDOFVelocities(vDOFVelocities,linearvel,angularvel,checklimits);
}

void KinBody::GetVelocity(Vector& linearvel, Vector& angularvel) const
{
    GetEnv()->GetPhysicsEngine()->GetLinkVelocity(_veclinks.at(0), linearvel, angularvel);
}

void KinBody::GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const
{
    GetEnv()->GetPhysicsEngine()->GetLinkVelocities(shared_kinbody_const(),velocities);
}

void KinBody::GetLinkTransformations(vector<Transform>& vtrans) const
{
    vtrans.resize(_veclinks.size());
    vector<Transform>::iterator it;
    vector<LinkPtr>::const_iterator itlink;
    for(it = vtrans.begin(), itlink = _veclinks.begin(); it != vtrans.end(); ++it, ++itlink) {
        *it = (*itlink)->GetTransform();
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
    if( vbodies.size() < _veclinks.size() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("not enough links %d<%d", vbodies.size()%_veclinks.size(),ORE_InvalidArguments);
    }
    vector<Transform>::const_iterator it;
    vector<LinkPtr>::iterator itlink;
    for(it = vbodies.begin(), itlink = _veclinks.begin(); it != vbodies.end(); ++it, ++itlink) {
        (*itlink)->SetTransform(*it);
    }
    _nUpdateStampId++;
}

void KinBody::SetLinkVelocities(const std::vector<std::pair<Vector,Vector> >& velocities)
{
    GetEnv()->GetPhysicsEngine()->SetLinkVelocities(shared_kinbody(),velocities);
}

void KinBody::SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transBase, bool bCheckLimits)
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

    SetDOFValues(vJointValues,bCheckLimits);
}

void KinBody::SetDOFValues(const std::vector<dReal>& vJointValues, bool bCheckLimits)
{
    CHECK_INTERNAL_COMPUTATION;
    _nUpdateStampId++;
    if( (int)vJointValues.size() < GetDOF() ) {
        throw OPENRAVE_EXCEPTION_FORMAT("not enough values %d<%d", vJointValues.size()%GetDOF(),ORE_InvalidArguments);
    }
    if((vJointValues.size() == 0)||(_veclinks.size() == 0)) {
        return;
    }
    const dReal* pJointValues = &vJointValues[0];
    if( bCheckLimits ) {
        _vTempJoints.resize(GetDOF());
        dReal* ptempjoints = &_vTempJoints[0];

        // check the limits
        vector<dReal> upperlim, lowerlim;
        FOREACHC(it, _vecjoints) {
            const dReal* p = pJointValues+(*it)->GetDOFIndex();
            BOOST_ASSERT( (*it)->GetDOF() <= 3 );
            (*it)->GetLimits(lowerlim, upperlim);
            if( (*it)->GetType() == Joint::JointSpherical ) {
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
                        *ptempjoints++ = NORMALIZE_ANGLE(p[i],lowerlim[i],upperlim[i]);
                    }
                    else {
                        if( p[i] < lowerlim[i] ) {
                            *ptempjoints++ = lowerlim[i];
                        }
                        else if( p[i] > upperlim[i] ) {
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
                    if( vPassiveJointValues[i][j] < _vPassiveJoints[i]->_vlowerlimit.at(j) ) {
                        if( vPassiveJointValues[i][j] < _vPassiveJoints[i]->_vlowerlimit.at(j)-5e-4f ) {
                            RAVELOG_WARN(str(boost::format("dummy joint out of lower limit! %e < %e\n")%_vPassiveJoints[i]->_vlowerlimit.at(j)%vPassiveJointValues[i][j]));
                        }
                        vPassiveJointValues[i][j] = _vPassiveJoints[i]->_vlowerlimit.at(j);
                    }
                    else if( vPassiveJointValues[i][j] > _vPassiveJoints[i]->_vupperlimit.at(j) ) {
                        if( vPassiveJointValues[i][j] > _vPassiveJoints[i]->_vupperlimit.at(j)+5e-4f ) {
                            RAVELOG_WARN(str(boost::format("dummy joint out of upper limit! %e > %e\n")%_vPassiveJoints[i]->_vupperlimit.at(j)%vPassiveJointValues[i][j]));
                        }
                        vPassiveJointValues[i][j] = _vPassiveJoints[i]->_vupperlimit.at(j);
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
                    const std::vector<Joint::MIMIC::DOFFormat>& vdofformat = pjoint->_vmimic[i]->_vdofformat;
                    FOREACHC(itdof,vdofformat) {
                        if( itdof->dofindex >= 0 ) {
                            vtempvalues.push_back(pJointValues[itdof->dofindex]);
                        }
                        else {
                            vtempvalues.push_back(vPassiveJointValues.at(itdof->jointindex-_vecjoints.size()).at(itdof->axis));
                        }
                    }
                    //dummyvalues[i] = pjoint->_vmimic.at(i)->_posfn->Eval(vtempvalues.size() > 0 ? &vtempvalues[0] : NULL);
                    pjoint->_vmimic.at(i)->_posfn->EvalMulti(veval, vtempvalues.empty() ? NULL : &vtempvalues[0]);
                    if( pjoint->_vmimic.at(i)->_posfn->EvalError() ) {
                        RAVELOG_WARN(str(boost::format("failed to evaluate joint %s, fparser error %d")%pjoint->GetName()%pjoint->_vmimic.at(i)->_posfn->EvalError()));
                    }
                    vector<dReal> vevalcopy = veval;
                    vector<dReal>::iterator iteval = veval.begin();
                    while(iteval != veval.end()) {
                        bool removevalue = false;
                        if((pjoint->GetType() == Joint::JointSpherical)|| pjoint->IsCircular(i) ) {
                        }
                        else if( *iteval < pjoint->_vlowerlimit[i] ) {
                            if(*iteval >= pjoint->_vlowerlimit[i]-g_fEpsilon*1000 ) {
                                *iteval = pjoint->_vlowerlimit[i];
                            }
                            else {
                                removevalue=true;
                            }
                        }
                        else if( *iteval > pjoint->_vupperlimit[i] ) {
                            if(*iteval <= pjoint->_vupperlimit[i]+g_fEpsilon*1000 ) {
                                *iteval = pjoint->_vupperlimit[i];
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
                            if((pjoint->GetType() == Joint::JointSpherical)|| pjoint->IsCircular(i) ) {
                                veval.push_back(*iteval);
                            }
                            else if( *iteval < pjoint->_vlowerlimit[i]-g_fEpsilon*1000 ) {
                                veval.push_back(pjoint->_vlowerlimit[i]);
                                RAVELOG_WARN(str(boost::format("joint %s: lower limit (%e) is not followed: %e")%pjoint->GetName()%pjoint->_vlowerlimit[i]%*iteval));
                            }
                            else if( *iteval > pjoint->_vupperlimit[i]+g_fEpsilon*1000 ) {
                                veval.push_back(pjoint->_vupperlimit[i]);
                                RAVELOG_WARN(str(boost::format("joint %s: upper limit (%e) is not followed: %e")%pjoint->GetName()%pjoint->_vupperlimit[i]%*iteval));
                            }
                            else {
                                veval.push_back(*iteval);
                            }
                        }
                        if( veval.empty() ) {
                            throw OPENRAVE_EXCEPTION_FORMAT("no valid values for joint %s", pjoint->GetName(),ORE_Assert);
                        }
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
        if( pjoint->GetType() & Joint::JointSpecialBit ) {
            switch(pjoint->GetType()) {
            case Joint::JointHinge2: {
                Transform tfirst;
                tfirst.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pvalues[0]);
                Transform tsecond;
                tsecond.rot = quatFromAxisAngle(tfirst.rotate(pjoint->GetInternalHierarchyAxis(1)), pvalues[1]);
                tjoint = tsecond * tfirst;
                break;
            }
            case Joint::JointSpherical: {
                dReal fang = pvalues[0]*pvalues[0]+pvalues[1]*pvalues[1]+pvalues[2]*pvalues[2];
                if( fang > 0 ) {
                    fang = RaveSqrt(fang);
                    dReal fiang = 1/fang;
                    tjoint.rot = quatFromAxisAngle(Vector(pvalues[0]*fiang,pvalues[1]*fiang,pvalues[2]*fiang),fang);
                }
                break;
            }
            default:
                RAVELOG_WARN(str(boost::format("forward kinematic type 0x%x not supported")%pjoint->GetType()));
                break;
            }
        }
        else {
            if( pjoint->GetType() == Joint::JointRevolute ) {
                tjoint.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(0), pvalues[0]);
            }
            else if( pjoint->GetType() == Joint::JointPrismatic ) {
                tjoint.trans = pjoint->GetInternalHierarchyAxis(0) * pvalues[0];
            }
            else {
                for(int iaxis = 0; iaxis < pjoint->GetDOF(); ++iaxis) {
                    Transform tdelta;
                    if( pjoint->IsRevolute(iaxis) ) {
                        tdelta.rot = quatFromAxisAngle(pjoint->GetInternalHierarchyAxis(iaxis), pvalues[iaxis]);
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

void KinBody::GetRigidlyAttachedLinks(int linkindex, std::vector<KinBody::LinkPtr>& vattachedlinks) const
{
    _veclinks.at(linkindex)->GetRigidlyAttachedLinks(vattachedlinks);
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
    BOOST_ASSERT(linkindex1>=0&&linkindex1<(int)_veclinks.size());
    BOOST_ASSERT(linkindex2>=0&&linkindex2<(int)_veclinks.size());
    vjoints.resize(0);
    if( linkindex1 == linkindex2 ) {
        return true;
    }
    int offset = linkindex2*_veclinks.size();
    int curlink = linkindex1;
    while(_vAllPairsShortestPaths[offset+curlink].first>=0) {
        int jointindex = _vAllPairsShortestPaths[offset+curlink].second;
        vjoints.push_back(jointindex < (int)_vecjoints.size() ? _vecjoints.at(jointindex) : _vPassiveJoints.at(jointindex-_vecjoints.size()));
        curlink = _vAllPairsShortestPaths[offset+curlink].first;
    }
    return vjoints.size()>0; // otherwise disconnected
}

bool KinBody::GetChain(int linkindex1, int linkindex2, std::vector<LinkPtr>& vlinks) const
{
    CHECK_INTERNAL_COMPUTATION0;
    BOOST_ASSERT(linkindex1>=0&&linkindex1<(int)_veclinks.size());
    BOOST_ASSERT(linkindex2>=0&&linkindex2<(int)_veclinks.size());
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

void KinBody::CalculateJacobian(int linkindex, const Vector& trans, boost::multi_array<dReal,2>& mjacobian) const
{
    CHECK_INTERNAL_COMPUTATION;
    if((linkindex < 0)||(linkindex >= (int)_veclinks.size())) {
        throw OPENRAVE_EXCEPTION_FORMAT("bad link index %d", linkindex,ORE_InvalidArguments);
    }
    mjacobian.resize(boost::extents[3][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }

    //Vector trans = _veclinks[linkindex]->GetTransform() * offset;
    Vector v;
    FOREACHC(itjoint, _vecjoints) {
        int dofindex = (*itjoint)->GetDOFIndex();
        int8_t affect = DoesAffect((*itjoint)->GetJointIndex(), linkindex);
        for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
            if( affect == 0 ) {
                mjacobian[0][dofindex+dof] = mjacobian[1][dofindex+dof] = mjacobian[2][dofindex+dof] = 0;
            }
            else {
                if( (*itjoint)->IsRevolute(dof) ) {
                    v = (*itjoint)->GetAxis(dof).cross(trans-(*itjoint)->GetAnchor());
                }
                else if( (*itjoint)->IsPrismatic(dof) ) {
                    v = (*itjoint)->GetAxis(dof);
                }
                else {
                    RAVELOG_WARN("CalculateJacobian joint %d not supported\n", (*itjoint)->GetType());
                    v = Vector(0,0,0);
                }
                mjacobian[0][dofindex+dof] = v.x; mjacobian[1][dofindex+dof] = v.y; mjacobian[2][dofindex+dof] = v.z;
            }
        }
    }

    // add in the contributions for each of the passive joints
    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Joint::MIMIC::DOFFormat, int>, dReal > mapcachedpartials;
    FOREACHC(itjoint, _vPassiveJoints) {
        for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
            if( (*itjoint)->IsMimic(idof) ) {
                bool usejoint = false;
                FOREACHC(itmimicdof,(*itjoint)->_vmimic[idof]->_vmimicdofs) {
                    if( DoesAffect(itmimicdof->dofindex,linkindex) ) {
                        usejoint = true;
                    }
                }
                if( !usejoint ) {
                    continue;
                }
                Vector vaxis;
                if( (*itjoint)->IsRevolute(idof) ) {
                    vaxis = (*itjoint)->GetAxis(idof).cross(trans-(*itjoint)->GetAnchor());
                }
                else if( (*itjoint)->IsPrismatic(idof) ) {
                    vaxis = (*itjoint)->GetAxis(idof);
                }
                else {
                    RAVELOG_WARN("CalculateJacobian joint %d not supported\n", (*itjoint)->GetType());
                    vaxis = Vector(0,0,0);
                }
                (*itjoint)->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                FOREACH(itpartial,vpartials) {
                    int dofindex = itpartial->first;
                    if( DoesAffect(_vDOFIndices.at(dofindex),linkindex) ) {
                        Vector v = vaxis * itpartial->second;
                        mjacobian[0][dofindex] += v.x;
                        mjacobian[1][dofindex] += v.y;
                        mjacobian[2][dofindex] += v.z;
                    }
                }
            }
        }
    }
}

void KinBody::CalculateJacobian(int linkindex, const Vector& trans, vector<dReal>& vjacobian) const
{
    boost::multi_array<dReal,2> mjacobian;
    KinBody::CalculateJacobian(linkindex,trans,mjacobian);
    vjacobian.resize(3*GetDOF());
    vector<dReal>::iterator itdst = vjacobian.begin();
    FOREACH(it,mjacobian) {
        std::copy(it->begin(),it->end(),itdst);
        itdst += GetDOF();
    }
}

void KinBody::CalculateRotationJacobian(int linkindex, const Vector& q, boost::multi_array<dReal,2>& mjacobian) const
{
    CHECK_INTERNAL_COMPUTATION;
    if((linkindex < 0)||(linkindex >= (int)_veclinks.size())) {
        throw OPENRAVE_EXCEPTION_FORMAT("bad link index %d", linkindex,ORE_InvalidArguments);
    }
    mjacobian.resize(boost::extents[4][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }
    Vector v;
    FOREACHC(itjoint, _vecjoints) {
        int dofindex = (*itjoint)->GetDOFIndex();
        int8_t affect = DoesAffect((*itjoint)->GetJointIndex(), linkindex);
        for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
            if( affect == 0 ) {
                mjacobian[0][dofindex+dof] = mjacobian[1][dofindex+dof] = mjacobian[2][dofindex+dof] = mjacobian[3][dofindex+dof] = 0;
            }
            else {
                if( (*itjoint)->IsRevolute(dof) ) {
                    v = (*itjoint)->GetAxis(dof);
                }
                else if( (*itjoint)->IsPrismatic(dof) ) {
                    v = Vector(0,0,0);
                }
                else {
                    RAVELOG_WARN("CalculateRotationJacobian joint %d not supported\n", (*itjoint)->GetType());
                    v = Vector(0,0,0);
                }
                mjacobian[0][dofindex+dof] = dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
                mjacobian[1][dofindex+dof] = dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
                mjacobian[2][dofindex+dof] = dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
                mjacobian[3][dofindex+dof] = dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
            }
        }
    }

    // add in the contributions for each of the passive links
    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Joint::MIMIC::DOFFormat, int>, dReal > mapcachedpartials;
    FOREACHC(itjoint, _vPassiveJoints) {
        for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
            if( (*itjoint)->IsMimic(idof) ) {
                bool usejoint = false;
                FOREACHC(itmimicdof,(*itjoint)->_vmimic[idof]->_vmimicdofs) {
                    if( DoesAffect(itmimicdof->dofindex,linkindex) ) {
                        usejoint = true;
                    }
                }
                if( !usejoint ) {
                    continue;
                }
                Vector vaxis;
                if( (*itjoint)->IsRevolute(idof) ) {
                    vaxis = (*itjoint)->GetAxis(idof);
                }
                else if( (*itjoint)->IsPrismatic(idof) ) {
                    vaxis = Vector(0,0,0);
                }
                else {
                    RAVELOG_WARN("CalculateRotationJacobian joint %d not supported\n", (*itjoint)->GetType());
                    vaxis = Vector(0,0,0);
                }
                (*itjoint)->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                FOREACH(itpartial,vpartials) {
                    int dofindex = itpartial->first;
                    if( DoesAffect(_vDOFIndices.at(dofindex),linkindex) ) {
                        Vector v = vaxis * itpartial->second;
                        mjacobian[0][dofindex] += dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
                        mjacobian[1][dofindex] += dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
                        mjacobian[2][dofindex] += dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
                        mjacobian[3][dofindex] += dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
                    }
                }
            }
        }
    }
}

void KinBody::CalculateRotationJacobian(int linkindex, const Vector& q, vector<dReal>& vjacobian) const
{
    boost::multi_array<dReal,2> mjacobian;
    KinBody::CalculateRotationJacobian(linkindex,q,mjacobian);
    vjacobian.resize(4*GetDOF());
    vector<dReal>::iterator itdst = vjacobian.begin();
    FOREACH(it,mjacobian) {
        std::copy(it->begin(),it->end(),itdst);
        itdst += GetDOF();
    }
}

void KinBody::CalculateAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& mjacobian) const
{
    CHECK_INTERNAL_COMPUTATION;
    if((linkindex < 0)||(linkindex >= (int)_veclinks.size())) {
        throw OPENRAVE_EXCEPTION_FORMAT("bad link index %d", linkindex,ORE_InvalidArguments);
    }
    mjacobian.resize(boost::extents[3][GetDOF()]);
    if( GetDOF() == 0 ) {
        return;
    }
    Vector v, anchor, axis;
    FOREACHC(itjoint, _vecjoints) {
        int dofindex = (*itjoint)->GetDOFIndex();
        int8_t affect = DoesAffect((*itjoint)->GetJointIndex(), linkindex);
        for(int dof = 0; dof < (*itjoint)->GetDOF(); ++dof) {
            if( affect == 0 ) {
                mjacobian[0][dofindex+dof] = mjacobian[1][dofindex+dof] = mjacobian[2][dofindex+dof] = 0;
            }
            else {
                if( (*itjoint)->IsRevolute(dof) ) {
                    v = (*itjoint)->GetAxis(dof);
                }
                else if( (*itjoint)->IsPrismatic(dof) ) {
                    v = Vector(0,0,0);
                }
                else {
                    RAVELOG_WARN("CalculateAngularVelocityJacobian joint %d not supported\n", (*itjoint)->GetType());
                    v = Vector(0,0,0);
                }
                mjacobian[0][dofindex+dof] = v.x; mjacobian[1][dofindex+dof] = v.y; mjacobian[2][dofindex+dof] = v.z;
            }
        }
    }

    // add in the contributions for each of the passive links
    std::vector<std::pair<int,dReal> > vpartials;
    std::map< std::pair<Joint::MIMIC::DOFFormat, int>, dReal > mapcachedpartials;
    FOREACHC(itjoint, _vPassiveJoints) {
        for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
            if( (*itjoint)->IsMimic(idof) ) {
                bool usejoint = false;
                FOREACHC(itmimicdof,(*itjoint)->_vmimic[idof]->_vmimicdofs) {
                    if( DoesAffect(itmimicdof->dofindex,linkindex) ) {
                        usejoint = true;
                    }
                }
                if( !usejoint ) {
                    continue;
                }
                Vector vaxis;
                if( (*itjoint)->IsRevolute(idof) ) {
                    vaxis = (*itjoint)->GetAxis(idof);
                }
                else if( (*itjoint)->IsPrismatic(idof) ) {
                    vaxis = Vector(0,0,0);
                }
                else {
                    RAVELOG_WARN("CalculateAngularVelocityJacobian joint %d not supported\n", (*itjoint)->GetType());
                    vaxis = Vector(0,0,0);
                }
                (*itjoint)->_ComputePartialVelocities(vpartials,idof,mapcachedpartials);
                FOREACH(itpartial,vpartials) {
                    int dofindex = itpartial->first;
                    if( DoesAffect(_vDOFIndices.at(dofindex),linkindex) ) {
                        Vector v = vaxis * itpartial->second;
                        mjacobian[0][dofindex] += v.x; mjacobian[1][dofindex] += v.y; mjacobian[2][dofindex] += v.z;
                    }
                }
            }
        }
    }
}

void KinBody::CalculateAngularVelocityJacobian(int linkindex, std::vector<dReal>& vjacobian) const
{
    boost::multi_array<dReal,2> mjacobian;
    KinBody::CalculateAngularVelocityJacobian(linkindex,mjacobian);
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
        if( !!report ) {
            RAVELOG_VERBOSE(str(boost::format("Self collision: %s\n")%report->__str__()));
        }
        return true;
    }
    return false;
}

void KinBody::_ComputeInternalInformation()
{
    uint64_t starttime = GetMicroTime();
    _nHierarchyComputed = 1;

    int lindex=0;
    FOREACH(itlink,_veclinks) {
        BOOST_ASSERT( lindex == (*itlink)->GetIndex() );
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
            if( !bmimic && (*itjoint)->_bActive ) {
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
            if( bmimic || !(*itjoint)->_bActive ) {
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
            dofindex += (*itjoint)->GetDOF();
        }
        FOREACH(itjoint,_vPassiveJoints) {
            (*itjoint)->jointindex = -1;
            (*itjoint)->dofindex = -1;
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
    sort(vorder.begin(), vorder.end(), index_cmp<vector<int>&>(vJointIndices));
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
        // fill Joint::MIMIC::_vmimicdofs, check that there are no circular dependencies between the mimic joints
        std::map<Joint::MIMIC::DOFFormat, boost::shared_ptr<Joint::MIMIC> > mapmimic;
        for(int ijoints = 0; ijoints < 2; ++ijoints) {
            vector<JointPtr>& vjoints = ijoints ? _vPassiveJoints : _vecjoints;
            int jointindex=0;
            FOREACH(itjoint,vjoints) {
                Joint::MIMIC::DOFFormat dofformat;
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
                boost::shared_ptr<Joint::MIMIC> mimic = itmimic->second;
                Joint::MIMIC::DOFHierarchy h;
                h.dofformatindex = 0;
                FOREACH(itdofformat,mimic->_vdofformat) {
                    if( mapmimic.find(*itdofformat) == mapmimic.end() ) {
                        continue; // this is normal, just means that the parent is a regular dof
                    }
                    boost::shared_ptr<Joint::MIMIC> mimicparent = mapmimic[*itdofformat];
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
    catch(const openrave_exception& ex) {
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
            if(((*itlink)->GetIndex() > 0)&&(vuniquepaths[(*itlink)->GetIndex()].size() == 0)) {
                RAVELOG_WARN(str(boost::format("_ComputeInternalInformation: %s has incomplete kinematics! link %s not connected to root %s")%GetName()%(*itlink)->GetName()%_veclinks.at(0)->GetName()));
            }
            FOREACH(itpath, vuniquepaths[(*itlink)->GetIndex()]) {
                BOOST_ASSERT(itpath->back()==(*itlink)->GetIndex());
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
                            if((bestindex == -1)|| ((bestindex >= 0)&&(vlinkdepths[*itparent] < bestindex)) ) {
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
        // build up a directed graph of joint dependencies
        int numjoints = (int)(_vecjoints.size()+_vPassiveJoints.size());
        // build the adjacency list
        vector<int> vjointadjacency(numjoints*numjoints,0);
        for(int ij0 = 0; ij0 < numjoints; ++ij0) {
            JointPtr j0 = ij0 < (int)_vecjoints.size() ? _vecjoints[ij0] : _vPassiveJoints[ij0-_vecjoints.size()];
            bool bj0hasstatic = (!j0->GetFirstAttached() || j0->GetFirstAttached()->IsStatic()) || (!j0->GetSecondAttached() || j0->GetSecondAttached()->IsStatic());
            // mimic joint sorting is a hard limit
            if( j0->IsMimic() ) {
                for(int i = 0; i < j0->GetDOF(); ++i) {
                    if(j0->IsMimic(i)) {
                        FOREACH(itdofformat, j0->_vmimic[i]->_vdofformat) {
                            if( itdofformat->dofindex < 0 ) {
                                vjointadjacency[itdofformat->jointindex*numjoints+ij0] = 2;
                            }
                        }
                    }
                }
            }

            for(int ij1 = ij0+1; ij1 < numjoints; ++ij1) {
                JointPtr j1 = ij1 < (int)_vecjoints.size() ? _vecjoints[ij1] : _vPassiveJoints[ij1-_vecjoints.size()];
                bool bj1hasstatic = (!j1->GetFirstAttached() || j1->GetFirstAttached()->IsStatic()) || (!j1->GetSecondAttached() || j1->GetSecondAttached()->IsStatic());

                // test if connected to world
                if( bj0hasstatic ) {
                    if( bj1hasstatic ) {
                        continue;
                    }
                    else {
                        vjointadjacency[ij0*numjoints+ij1] = 1;
                        continue;
                    }
                }
                else if( bj1hasstatic ) {
                    vjointadjacency[ij1*numjoints+ij0] = 1;
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
                    BOOST_ASSERT(min(j0l0,j0l1)<100);
                    vjointadjacency[ij0*numjoints+ij1] = 100-min(j0l0,j0l1);
                    continue;
                }
                if( diff > 0 ) {
                    BOOST_ASSERT(min(j1l0,j1l1)<100);
                    vjointadjacency[ij1*numjoints+ij0] = 100-min(j1l0,j1l1);
                    continue;
                }
                diff = max(j0l0,j0l1) - max(j1l0,j1l1);
                if( diff < 0 ) {
                    BOOST_ASSERT(max(j0l0,j0l1)<100);
                    vjointadjacency[ij0*numjoints+ij1] = 100-max(j0l0,j0l1);
                    continue;
                }
                if( diff > 0 ) {
                    BOOST_ASSERT(max(j1l0,j1l1)<100);
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
        BOOST_ASSERT((int)_vTopologicallySortedJointIndicesAll.size()==numjoints);
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
                RAVELOG_DEBUG(str(boost::format("swapping link order of joint %s(%d)")%(*itjoint)->GetName()%(*itjoint)->GetJointIndex()));
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
                    BOOST_ASSERT( jointindex >= 0 );
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

            if( IS_DEBUGLEVEL(Level_Debug) ) {
                stringstream ss;
                ss << GetName() << " closedloop found: ";
                FOREACH(itlinkindex,*itclosedloop) {
                    LinkPtr plink = _veclinks.at(*itlinkindex);
                    ss << plink->GetName() << "(" << plink->GetIndex() << ") ";
                }
                RAVELOG_DEBUG(ss.str());
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
            if((pjoint0->GetName() == pjoint1->GetName())&& ((pjoint0->GetJointIndex() >= 0)||(pjoint1->GetJointIndex() >= 0)) ) {
                RAVELOG_WARN(str(boost::format("joint indices %d and %d share the same name %s")%pjoint0->GetJointIndex()%pjoint1->GetJointIndex()%pjoint0->GetName()));
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
        GetLinkTransformations(vprevtrans);
        vector<dReal> vcurrentvalues;
        GetDOFValues(vcurrentvalues);
        SetDOFValues(vcurrentvalues,true);
        GetLinkTransformations(vnewtrans);
        for(size_t i = 0; i < vprevtrans.size(); ++i) {
            if( TransformDistanceFast(vprevtrans[i],vnewtrans[i]) > 1e-5 ) {
                RAVELOG_VERBOSE(str(boost::format("link %d has different transformation after SetDOFValues (error=%f), this could be due to mimic joint equations kicking into effect.")%_veclinks.at(i)->GetName()%TransformDistanceFast(vprevtrans[i],vnewtrans[i])));
            }
        }
        _vInitialLinkTransformations = vnewtrans;
    }

    {
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
            group.interpolation = "linear";
            offset += group.dof;
            _spec._vgroups.push_back(group);
        }

        ConfigurationSpecification::Group group;
        group.name = str(boost::format("affine_transform %s %d")%GetName()%DOF_Transform);
        group.offset = offset;
        group.dof = RaveGetAffineDOF(DOF_Transform);
        group.interpolation = "linear";
        _spec._vgroups.push_back(group);
    }

    // notify any callbacks of the changes
    if( _nParametersChanged ) {
        std::list<UserDataWeakPtr> listRegisteredCallbacks = _listRegisteredCallbacks; // copy since it can be changed
        FOREACH(it,listRegisteredCallbacks) {
            ChangeCallbackDataPtr pdata = boost::dynamic_pointer_cast<ChangeCallbackData>(it->lock());
            if( !!pdata && (pdata->_properties & _nParametersChanged) ) {
                pdata->_callback();
            }
        }
        _nParametersChanged = 0;
    }
    RAVELOG_DEBUG(str(boost::format("_ComputeInternalInformation %s in %fs")%GetName()%(1e-6*(GetMicroTime()-starttime))));
}

bool KinBody::IsAttached(KinBodyConstPtr pbody) const
{
    if( shared_kinbody_const() == pbody ) {
        return true;
    }
    std::set<KinBodyConstPtr> dummy;
    return _IsAttached(pbody,dummy);
}

void KinBody::GetAttached(std::set<KinBodyPtr>& setAttached) const
{
    setAttached.insert(boost::const_pointer_cast<KinBody>(shared_kinbody_const()));
    FOREACHC(itbody,_listAttachedBodies) {
        KinBodyPtr pattached = itbody->lock();
        if( !!pattached && setAttached.insert(pattached).second ) {
            pattached->GetAttached(setAttached);
        }
    }
}

bool KinBody::_IsAttached(KinBodyConstPtr pbody, std::set<KinBodyConstPtr>& setChecked) const
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
    CollisionCheckerBasePtr p = GetEnv()->GetCollisionChecker();
    FOREACH(it, _veclinks) {
        if( (*it)->_bIsEnabled != bEnable ) {
            if( !p->EnableLink(LinkConstPtr(*it),bEnable) ) {
                throw OPENRAVE_EXCEPTION_FORMAT("failed to enable link %s:%s", GetName()%(*it)->GetName(), ORE_Failed);
            }
            (*it)->_bIsEnabled = bEnable;
            _nNonAdjacentLinkCache &= ~AO_Enabled;
        }
    }
}

bool KinBody::IsEnabled() const
{
    // enable/disable everything
    FOREACHC(it, _veclinks) {
        if((*it)->IsEnabled()) {
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
    BOOST_ASSERT(jointindex >= 0 && jointindex < (int)_vecjoints.size());
    BOOST_ASSERT(linkindex >= 0 && linkindex < (int)_veclinks.size());
    return _vJointsAffectingLinks.at(jointindex*_veclinks.size()+linkindex);
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
            _pbody->GetLinkTransformations(vcurtrans);
        }
        ~TransformsSaver() {
            for(size_t i = 0; i < _pbody->_veclinks.size(); ++i) {
                boost::static_pointer_cast<Link>(_pbody->_veclinks[i])->_t = vcurtrans.at(i);
            }
        }
private:
        KinBodyConstPtr _pbody;
        std::vector<Transform> vcurtrans;
    };

    CHECK_INTERNAL_COMPUTATION;
    if( _nNonAdjacentLinkCache & 0x80000000 ) {
        // Check for colliding link pairs given the initial pose _vInitialLinkTransformations
        // this is actually weird, we need to call the individual link collisions on a const body. in order to pull this off, we need to be very careful with the body state.
        TransformsSaver saver(shared_kinbody_const());
        CollisionOptionsStateSaver colsaver(GetEnv()->GetCollisionChecker(),0); // have to reset the collision options
        for(size_t i = 0; i < _veclinks.size(); ++i) {
            boost::static_pointer_cast<Link>(_veclinks[i])->_t = _vInitialLinkTransformations.at(i);
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
    _listRegisteredCallbacks.clear(); // reset the callbacks

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

    if(((parameters & Prop_JointMimic) == Prop_JointMimic)||((parameters & Prop_LinkStatic) == Prop_LinkStatic)) {
        KinBodyStateSaver saver(shared_kinbody(),Save_LinkTransformation);
        vector<dReal> vzeros(GetDOF(),0);
        SetDOFValues(vzeros,Transform(),true);
        _ComputeInternalInformation();
    }
    // do not change hash if geometry changed!
    //if( parameters&Prop_LinkGeometry )
    if( parameters & (Prop_Joints|Prop_Links) ) {
        __hashkinematics.resize(0);
    }

    std::list<UserDataWeakPtr> listRegisteredCallbacks = _listRegisteredCallbacks; // copy since it can be changed
    FOREACH(it,listRegisteredCallbacks) {
        ChangeCallbackDataPtr pdata = boost::dynamic_pointer_cast<ChangeCallbackData>(it->lock());
        if( !!pdata && (pdata->_properties & parameters) ) {
            pdata->_callback();
        }
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
        __hashkinematics = GetMD5HashString(ss.str());
    }
    return __hashkinematics;
}

void KinBody::SetConfigurationValues(std::vector<dReal>::const_iterator itvalues, bool checklimits)
{
    vector<dReal> vdofvalues(GetDOF());
    if( GetDOF() > 0 ) {
        std::copy(itvalues,itvalues+GetDOF(),vdofvalues.begin());
    }
    Transform t;
    RaveGetTransformFromAffineDOFValues(t,itvalues+GetDOF(),DOF_Transform);
    SetDOFValues(vdofvalues,t,checklimits);
}

void KinBody::GetConfigurationValues(std::vector<dReal>& v) const
{
    GetDOFValues(v);
    v.resize(GetDOF()+RaveGetAffineDOF(DOF_Transform));
    RaveGetAffineDOFValuesFromTransform(v.begin()+GetDOF(),GetTransform(),DOF_Transform);
}

const ConfigurationSpecification& KinBody::GetConfigurationSpecification() const
{
    return _spec;
}

ConfigurationSpecification KinBody::GetConfigurationSpecificationIndices(const std::vector<int>& indices) const
{
    ConfigurationSpecification spec;
    spec._vgroups.resize(1);
    stringstream ss;
    ss << "joint_values " << GetName();
    FOREACHC(it,indices) {
        ss << " " << *it;
    }
    spec._vgroups[0].name = ss.str();
    spec._vgroups[0].dof = indices.size();
    spec._vgroups[0].offset = 0;
    return spec;
}

UserDataPtr KinBody::RegisterChangeCallback(int properties, const boost::function<void()>& callback)
{
    ChangeCallbackDataPtr pdata(new ChangeCallbackData(properties,callback,shared_kinbody()));
    pdata->_iterator = _listRegisteredCallbacks.insert(_listRegisteredCallbacks.end(),pdata);
    return pdata;
}

static std::vector<dReal> fparser_polyroots2(const dReal* rawcoeffs)
{
    int numroots=0;
    std::vector<dReal> rawroots(2);
    polyroots2<dReal>(rawcoeffs,&rawroots[0],numroots);
    rawroots.resize(numroots);
    return rawroots;
}

template <int D>
static std::vector<dReal> fparser_polyroots(const dReal* rawcoeffs)
{
    int numroots=0;
    std::vector<dReal> rawroots(D);
    polyroots<dReal,D>(rawcoeffs,&rawroots[0],numroots);
    rawroots.resize(numroots);
    return rawroots;
}

// take triangle 3 sides and compute the angle opposite the first side
static std::vector<dReal> fparser_sssa(const dReal* coeffs)
{
    std::vector<dReal> res;
    dReal a = coeffs[0], b = coeffs[1], c = coeffs[2];
    dReal f = (a*a+b*b-c*c)/(2*b);
    res.push_back(RaveAtan2(RaveSqrt(a*a-f*f),b-f));
    return res;
}

/// take triangle 2 sides and an angle and compute the missing angle
static std::vector<dReal> fparser_sasa(const dReal* coeffs)
{
    std::vector<dReal> res;
    dReal a = coeffs[0], gamma = coeffs[1], b = coeffs[2];
    res.push_back(RaveAtan2(a*RaveSin(gamma),b-a*RaveCos(gamma)));
    return res;
}

/// take triangle 2 sides and an angle and compute the missing side
static std::vector<dReal> fparser_sass(const dReal* coeffs)
{
    std::vector<dReal> res;
    dReal a = coeffs[0], gamma = coeffs[1], b = coeffs[2];
    res.push_back(RaveSqrt(a*a+b*b-2*a*b*RaveCos(gamma)));
    return res;
}

boost::shared_ptr<FunctionParserBase<dReal> > KinBody::_CreateFunctionParser()
{
    boost::shared_ptr<FunctionParserBase<dReal> > parser(new FunctionParserBase<dReal>());
    // register commonly used functions
    parser->AddFunction("polyroots2",fparser_polyroots2,3);
    parser->AddFunction("polyroots3",fparser_polyroots<3>,4);
    parser->AddFunction("polyroots4",fparser_polyroots<4>,5);
    parser->AddFunction("polyroots5",fparser_polyroots<5>,6);
    parser->AddFunction("polyroots6",fparser_polyroots<6>,7);
    parser->AddFunction("polyroots7",fparser_polyroots<7>,8);
    parser->AddFunction("polyroots8",fparser_polyroots<8>,9);
    parser->AddFunction("SSSA",fparser_sssa,3);
    parser->AddFunction("SASA",fparser_sasa,3);
    parser->AddFunction("SASS",fparser_sass,3);
    return parser;
}

} // end namespace OpenRAVE
