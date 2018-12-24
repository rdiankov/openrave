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

#define GTS_M_ICOSAHEDRON_X /* sqrt(sqrt(5)+1)/sqrt(2*sqrt(5)) */ \
    (dReal)0.850650808352039932181540497063011072240401406
#define GTS_M_ICOSAHEDRON_Y /* sqrt(2)/sqrt(5+sqrt(5))         */ \
    (dReal)0.525731112119133606025669084847876607285497935
#define GTS_M_ICOSAHEDRON_Z (dReal)0.0

// generate a sphere triangulation starting with an icosahedron
// all triangles are oriented counter clockwise
void GenerateSphereTriangulation(TriMesh& tri, int levels)
{
    TriMesh temp, temp2;

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

    TriMesh* pcur = &temp;
    TriMesh* pnew = &temp2;
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

/// \param ex half extents
void AppendBoxTriangulation(const Vector& pos, const Vector& ex, TriMesh& tri)
{
    // trivial
    Vector v[8] = { Vector(ex.x, ex.y, ex.z)+pos,
                    Vector(ex.x, ex.y, -ex.z)+pos,
                    Vector(ex.x, -ex.y, ex.z)+pos,
                    Vector(ex.x, -ex.y, -ex.z)+pos,
                    Vector(-ex.x, ex.y, ex.z)+pos,
                    Vector(-ex.x, ex.y, -ex.z)+pos,
                    Vector(-ex.x, -ex.y, ex.z)+pos,
                    Vector(-ex.x, -ex.y, -ex.z)+pos};
    int vertexoffset = (int)tri.vertices.size();
    const int nindices = 36;
    int indices[nindices] = {
        0+vertexoffset, 2+vertexoffset, 1+vertexoffset,
        1+vertexoffset, 2+vertexoffset, 3+vertexoffset,
        4+vertexoffset, 5+vertexoffset, 6+vertexoffset,
        5+vertexoffset, 7+vertexoffset, 6+vertexoffset,
        0+vertexoffset, 1+vertexoffset, 4+vertexoffset,
        1+vertexoffset, 5+vertexoffset, 4+vertexoffset,
        2+vertexoffset, 6+vertexoffset, 3+vertexoffset,
        3+vertexoffset, 6+vertexoffset, 7+vertexoffset,
        0+vertexoffset, 4+vertexoffset, 2+vertexoffset,
        2+vertexoffset, 4+vertexoffset, 6+vertexoffset,
        1+vertexoffset, 3+vertexoffset, 5+vertexoffset,
        3+vertexoffset, 7+vertexoffset, 5+vertexoffset
    };
    tri.vertices.insert(tri.vertices.end(), &v[0], &v[8]);
    tri.indices.insert(tri.indices.end(), &indices[0], &indices[nindices]);
}

KinBody::GeometryInfo::GeometryInfo() : XMLReadable("geometry")
{
    _vDiffuseColor = Vector(1,1,1);
    _type = GT_None;
    _fTransparency = 0;
    _vRenderScale = _vCollisionScale = Vector(1,1,1);
    _bVisible = true;
    _bModifiable = true;
}

bool KinBody::GeometryInfo::InitCollisionMesh(float fTessellation)
{
    if( _type == GT_TriMesh || _type == GT_None ) {
        return true;
    }

    // is clear() better since it releases the memory?
    _meshcollision.indices.resize(0);
    _meshcollision.vertices.resize(0);

    if( fTessellation < 0.01f ) {
        fTessellation = 0.01f;
    }
    // start tesselating
    switch(_type) {
    case GT_Sphere: {
        // log_2 (1+ tess)
        GenerateSphereTriangulation(_meshcollision, 3 + (int)(logf(fTessellation) / logf(2.0f)) );
        dReal fRadius = GetSphereRadius();
        FOREACH(it, _meshcollision.vertices) {
            *it *= fRadius;
        }
        break;
    }
    case GT_Box: {
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
        _meshcollision.vertices.resize(8);
        std::copy(&v[0],&v[8],_meshcollision.vertices.begin());
        _meshcollision.indices.resize(nindices);
        std::copy(&indices[0],&indices[nindices],_meshcollision.indices.begin());
        break;
    }
    case GT_Cylinder: {
        // cylinder is on z axis
        dReal rad = GetCylinderRadius(), len = GetCylinderHeight()*0.5f;
        int numverts = (int)(fTessellation*48.0f) + 3;
        dReal dtheta = 2 * PI / (dReal)numverts;
        _meshcollision.vertices.push_back(Vector(0,0,len));
        _meshcollision.vertices.push_back(Vector(0,0,-len));
        _meshcollision.vertices.push_back(Vector(rad,0,len));
        _meshcollision.vertices.push_back(Vector(rad,0,-len));
        for(int i = 0; i < numverts+1; ++i) {
            dReal s = rad * RaveSin(dtheta * (dReal)i);
            dReal c = rad * RaveCos(dtheta * (dReal)i);
            int off = (int)_meshcollision.vertices.size();
            _meshcollision.vertices.push_back(Vector(c, s, len));
            _meshcollision.vertices.push_back(Vector(c, s, -len));

            _meshcollision.indices.push_back(0);       _meshcollision.indices.push_back(off-2);       _meshcollision.indices.push_back(off);
            _meshcollision.indices.push_back(1);       _meshcollision.indices.push_back(off+1);       _meshcollision.indices.push_back(off-1);
            _meshcollision.indices.push_back(off-2);   _meshcollision.indices.push_back(off-1);         _meshcollision.indices.push_back(off);
            _meshcollision.indices.push_back(off);   _meshcollision.indices.push_back(off-1);         _meshcollision.indices.push_back(off+1);
        }
        break;
    }
    case GT_Container: {
        const Vector& outerextents = _vGeomData;
        const Vector& innerextents = _vGeomData2;
        const Vector& bottomcross = _vGeomData3;
        // +x wall
        AppendBoxTriangulation(Vector((outerextents[0]+innerextents[0])/4.,0,outerextents[2]/2.), Vector((outerextents[0]-innerextents[0])/4., outerextents[1]/2., outerextents[2]/2.), _meshcollision);
        // -x wall
        AppendBoxTriangulation(Vector(-(outerextents[0]+innerextents[0])/4.,0,outerextents[2]/2.), Vector((outerextents[0]-innerextents[0])/4., outerextents[1]/2., outerextents[2]/2.), _meshcollision);
        // +y wall
        AppendBoxTriangulation(Vector(0,(outerextents[1]+innerextents[1])/4.,outerextents[2]/2.), Vector(outerextents[0]/2., (outerextents[1]-innerextents[1])/4., outerextents[2]/2.), _meshcollision);
        // -y wall
        AppendBoxTriangulation(Vector(0,-(outerextents[1]+innerextents[1])/4.,outerextents[2]/2.), Vector(outerextents[0]/2., (outerextents[1]-innerextents[1])/4., outerextents[2]/2.), _meshcollision);
        // bottom
        if( outerextents[2] - innerextents[2] >= 1e-6 ) { // small epsilon error can make thin triangles appear, so test with a reasonable threshold
            AppendBoxTriangulation(Vector(0,0,(outerextents[2]-innerextents[2])/2.), Vector(outerextents[0]/2., outerextents[1]/2., (outerextents[2]-innerextents[2])/2), _meshcollision);
        }
        // cross
        if( bottomcross[2] > 0 ) {
            if( bottomcross[0] > 0 ) {
                AppendBoxTriangulation(Vector(0, 0, bottomcross[2]/2+outerextents[2]-innerextents[2]), Vector(bottomcross[0]/2, innerextents[1]/2, bottomcross[2]/2), _meshcollision);
            }
            if( bottomcross[1] > 0 ) {
                AppendBoxTriangulation(Vector(0, 0, bottomcross[2]/2+outerextents[2]-innerextents[2]), Vector(innerextents[0]/2, bottomcross[1]/2, bottomcross[2]/2), _meshcollision);
            }
        }
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_("unrecognized geom type %d!"), _type, ORE_InvalidArguments);
    }

    return true;
}

KinBody::Link::Geometry::Geometry(KinBody::LinkPtr parent, const KinBody::GeometryInfo& info) : _parent(parent), _info(info)
{
}

bool KinBody::Link::Geometry::InitCollisionMesh(float fTessellation)
{
    return _info.InitCollisionMesh(fTessellation);
}

AABB KinBody::Link::Geometry::ComputeAABB(const Transform& t) const
{
    AABB ab;
    TransformMatrix tglobal = t * _info._t;

    switch(_info._type) {
    case GT_None:
        ab.extents.x = 0;
        ab.extents.y = 0;
        ab.extents.z = 0;
        break;
    case GT_Box: // origin of box is at the center
        ab.extents.x = RaveFabs(tglobal.m[0])*_info._vGeomData.x + RaveFabs(tglobal.m[1])*_info._vGeomData.y + RaveFabs(tglobal.m[2])*_info._vGeomData.z;
        ab.extents.y = RaveFabs(tglobal.m[4])*_info._vGeomData.x + RaveFabs(tglobal.m[5])*_info._vGeomData.y + RaveFabs(tglobal.m[6])*_info._vGeomData.z;
        ab.extents.z = RaveFabs(tglobal.m[8])*_info._vGeomData.x + RaveFabs(tglobal.m[9])*_info._vGeomData.y + RaveFabs(tglobal.m[10])*_info._vGeomData.z;
        ab.pos = tglobal.trans;
        break;
    case GT_Container: // origin of container is at the bottom
        ab.extents.x = 0.5*(RaveFabs(tglobal.m[0])*_info._vGeomData.x + RaveFabs(tglobal.m[1])*_info._vGeomData.y + RaveFabs(tglobal.m[2])*_info._vGeomData.z);
        ab.extents.y = 0.5*(RaveFabs(tglobal.m[4])*_info._vGeomData.x + RaveFabs(tglobal.m[5])*_info._vGeomData.y + RaveFabs(tglobal.m[6])*_info._vGeomData.z);
        ab.extents.z = 0.5*(RaveFabs(tglobal.m[8])*_info._vGeomData.x + RaveFabs(tglobal.m[9])*_info._vGeomData.y + RaveFabs(tglobal.m[10])*_info._vGeomData.z);
        ab.pos = tglobal.trans + Vector(tglobal.m[2], tglobal.m[6], tglobal.m[10])*(0.5*_info._vGeomData.z);
        break;
    case GT_Sphere:
        ab.extents.x = ab.extents.y = ab.extents.z = _info._vGeomData[0];
        ab.pos = tglobal.trans;
        break;
    case GT_Cylinder:
        ab.extents.x = (dReal)0.5*RaveFabs(tglobal.m[2])*_info._vGeomData.y + RaveSqrt(max(dReal(0),1-tglobal.m[2]*tglobal.m[2]))*_info._vGeomData.x;
        ab.extents.y = (dReal)0.5*RaveFabs(tglobal.m[6])*_info._vGeomData.y + RaveSqrt(max(dReal(0),1-tglobal.m[6]*tglobal.m[6]))*_info._vGeomData.x;
        ab.extents.z = (dReal)0.5*RaveFabs(tglobal.m[10])*_info._vGeomData.y + RaveSqrt(max(dReal(0),1-tglobal.m[10]*tglobal.m[10]))*_info._vGeomData.x;
        ab.pos = tglobal.trans; //+(dReal)0.5*_info._vGeomData.y*Vector(tglobal.m[2],tglobal.m[6],tglobal.m[10]);
        break;
    case GT_TriMesh:
        // just use _info._meshcollision
        if( _info._meshcollision.vertices.size() > 0) {
            Vector vmin, vmax; vmin = vmax = tglobal*_info._meshcollision.vertices.at(0);
            FOREACHC(itv, _info._meshcollision.vertices) {
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
        throw OPENRAVE_EXCEPTION_FORMAT(_("unknown geometry type %d"), _info._type, ORE_InvalidArguments);
    }

    return ab;
}

void KinBody::Link::Geometry::serialize(std::ostream& o, int options) const
{
    SerializeRound(o,_info._t);
    o << _info._type << " ";
    SerializeRound3(o,_info._vRenderScale);
    if( _info._type == GT_TriMesh ) {
        _info._meshcollision.serialize(o,options);
    }
    else {
        SerializeRound3(o,_info._vGeomData);
    }
}

void KinBody::Link::Geometry::SetCollisionMesh(const TriMesh& mesh)
{
    OPENRAVE_ASSERT_FORMAT0(_info._bModifiable, "geometry cannot be modified", ORE_Failed);
    LinkPtr parent(_parent);
    _info._meshcollision = mesh;
    parent->_Update();
}

bool KinBody::Link::Geometry::SetVisible(bool visible)
{
    if( _info._bVisible != visible ) {
        _info._bVisible = visible;
        LinkPtr parent(_parent);
        parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
        return true;
    }
    return false;
}

void KinBody::Link::Geometry::SetTransparency(float f)
{
    LinkPtr parent(_parent);
    _info._fTransparency = f;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
}

void KinBody::Link::Geometry::SetDiffuseColor(const RaveVector<float>& color)
{
    LinkPtr parent(_parent);
    _info._vDiffuseColor = color;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
}

void KinBody::Link::Geometry::SetAmbientColor(const RaveVector<float>& color)
{
    LinkPtr parent(_parent);
    _info._vAmbientColor = color;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
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

bool KinBody::Link::Geometry::ValidateContactNormal(const Vector& _position, Vector& _normal) const
{
    Transform tinv = _info._t.inverse();
    Vector position = tinv*_position;
    Vector normal = tinv.rotate(_normal);
    const dReal feps=0.00005f;
    switch(_info._type) {
    case GT_Box: {
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
        dReal xaxis = -_info._vGeomData.z*tposition.y+_info._vGeomData.y*tposition.z;
        dReal yaxis = -_info._vGeomData.x*tposition.z+_info._vGeomData.z*tposition.x;
        dReal zaxis = -_info._vGeomData.y*tposition.x+_info._vGeomData.x*tposition.y;
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
    case GT_Cylinder: { // z-axis
        dReal fInsideCircle = position.x*position.x+position.y*position.y-_info._vGeomData.x*_info._vGeomData.x;
        dReal fInsideHeight = 2.0f*RaveFabs(position.z)-_info._vGeomData.y;
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
    case GT_Sphere:
        if( normal.dot3(position) < 0 ) {
            _normal = -_normal;
            return true;
        }
        break;
    case GT_None:
    default:
        break;
    }
    return false;
}

void KinBody::Link::Geometry::SetRenderFilename(const std::string& renderfilename)
{
    LinkPtr parent(_parent);
    _info._filenamerender = renderfilename;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);
}

void KinBody::Link::Geometry::SetName(const std::string& name)
{
    LinkPtr parent(_parent);
    _info._name = name;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);

}
    
}
