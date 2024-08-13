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

static void AppendConeTriangulation(const Vector& pos, const dReal radius, dReal halfHeight, const uint numFaces, TriMesh& tri, const bool upsideDown = false)
{
    const dReal dTheta = 2 * PI / (dReal)numFaces; // degrees to rotate every time
    const dReal rotateOffset = upsideDown ? M_PI_2 : 0;
    if (upsideDown) {
        halfHeight = -halfHeight;
    }

    const int32_t base = tri.vertices.size();
    const size_t indexBase = tri.indices.size();
    tri.vertices.resize(base + numFaces + 2); // top and bottom center vertices
    tri.indices.resize(indexBase + 3 * 2 * numFaces); // two triangles per face (one on the side, one on bottom)

    std::vector<Vector>::iterator vertexIt = tri.vertices.begin() + base;
    std::vector<int32_t>::iterator indexIt = tri.indices.begin() + indexBase;

    *(vertexIt++) = Vector(pos.x, pos.y, pos.z + halfHeight); // top center
    *(vertexIt++) = Vector(pos.x, pos.y, pos.z - halfHeight); // bottom center

    // first line
    if (upsideDown) {
        *(vertexIt++) = Vector(pos.x, pos.y + radius, pos.z - halfHeight);
    } else {
        *(vertexIt++) = Vector(pos.x + radius, pos.y, pos.z - halfHeight);
    }

    int32_t off = base + 3;
    for (uint i = 1; i < numFaces; ++i) {
        // line on the side
        *(vertexIt++) = Vector(
            pos.x + RaveCos(dTheta * i - rotateOffset) * radius,
            pos.y + RaveSin(dTheta * i + rotateOffset) * radius,
            pos.z - halfHeight
            );

        // two triangles
        // 1. bottom face triangle, bottom center, this line bottom, last line bottom
        *(indexIt++) = base + 1; *(indexIt++) = off;     *(indexIt++) = off - 1;
        // 2. side face triangle, top center, last line bottom, this line bottom
        *(indexIt++) = base;     *(indexIt++) = off - 1; *(indexIt++) = off;

        off += 1;
    }

    // close the loop
    *(indexIt++) = base + 1; *(indexIt++) = base + 2; *(indexIt++) = off  - 1;
    *(indexIt++) = base;     *(indexIt++) = off  - 1; *(indexIt++) = base + 2;
}

static void AppendConicalFrustumTriangulation(const Vector& pos, const dReal topRadius, const dReal bottomRadius, const dReal halfHeight, const uint numFaces, TriMesh& tri)
{
    if (topRadius == 0) {
        return AppendConeTriangulation(pos, bottomRadius, halfHeight, numFaces, tri);
    }
    if (bottomRadius == 0) {
        return AppendConeTriangulation(pos, topRadius, halfHeight, numFaces, tri, true);
    }
    // once again, cylinder is on z axis
    const dReal dTheta = 2 * PI / (dReal)numFaces; // degrees to rotate every time

    const int32_t base = tri.vertices.size();
    const size_t indexBase = tri.indices.size();
    tri.vertices.resize(base + 2 * numFaces + 2); // top and bottom center vertices
    tri.indices.resize(indexBase + 3 * 4 * numFaces); // four triangles per face (two on the side, one on top, one on bottom)

    std::vector<Vector>::iterator vertexIt = tri.vertices.begin() + base;
    std::vector<int32_t>::iterator indexIt = tri.indices.begin() + indexBase;

    *(vertexIt++) = Vector(pos.x, pos.y, pos.z + halfHeight); // top center
    *(vertexIt++) = Vector(pos.x, pos.y, pos.z - halfHeight); // bottom center

    // first line
    *(vertexIt++) = Vector(pos.x + topRadius, pos.y, pos.z + halfHeight); // line top
    *(vertexIt++) = Vector(pos.x + bottomRadius, pos.y, pos.z - halfHeight); // line bottom

    int32_t off = base + 4;
    for (uint i = 1; i < numFaces; ++i) {
        const dReal unitX = RaveCos(dTheta * i);
        const dReal unitY = RaveSin(dTheta * i);

        // line on the side
        *(vertexIt++) = Vector(pos.x + unitX * topRadius, pos.y + unitY * topRadius, pos.z + halfHeight); // line top
        *(vertexIt++) = Vector(pos.x + unitX * bottomRadius, pos.y + unitY * bottomRadius, pos.z - halfHeight); // line bottom

        // four triangles
        // 1. top face triangle, top center, last line top, this line top
        *(indexIt++) = base;     *(indexIt++) = off - 2; *(indexIt++) = off;
        // 2. bottom face triangle, bottom center, this line bottom, last line bottom
        *(indexIt++) = base + 1; *(indexIt++) = off + 1; *(indexIt++) = off - 1;
        // 3. side face triangle 1, last line top, last line bottom, this line top
        *(indexIt++) = off  - 2; *(indexIt++) = off - 1; *(indexIt++) = off;
        // 4. side face triangle 2, this line top, last line bottom, this line bottom
        *(indexIt++) = off;      *(indexIt++) = off - 1; *(indexIt++) = off + 1;

        off += 2;
    }

    // close the loop
    *(indexIt++) = base;     *(indexIt++) = off  - 2; *(indexIt++) = base + 2;
    *(indexIt++) = base + 1; *(indexIt++) = base + 3; *(indexIt++) = off  - 1;
    *(indexIt++) = off  - 2; *(indexIt++) = off  - 1; *(indexIt++) = base + 2;
    *(indexIt++) = base + 2; *(indexIt++) = off  - 1; *(indexIt++) = base + 3;
}

static void AppendCylinderTriangulation(const Vector& pos, const dReal radius, const dReal halfHeight, const uint numverts, TriMesh& tri)
{
    return AppendConicalFrustumTriangulation(pos, radius, radius, halfHeight, numverts, tri);
}

void KinBody::GeometryInfo::GenerateCalibrationBoardDotMesh(TriMesh& tri, float fTessellation) const
{
    // reset dots mesh
    tri.indices.clear();
    tri.vertices.clear();
    if (_type != GT_CalibrationBoard) {
        RAVELOG_WARN_FORMAT("Cannot generate calibration board dot grid for geometry of type %d.", _type);
        return;
    }
    if (_calibrationBoardParameters.size() == 0) {
        RAVELOG_WARN("Cannot generate calibration board dot grid since _calibrationBoardParameters are empty.\n");
        return;
    }
    Vector boardEx = _vGeomData;
    const CalibrationBoardParameters& parameters = _calibrationBoardParameters[0];

    // create mesh for dot grid
    dReal nDotsX = parameters.numDotsX;
    dReal nDotsY = parameters.numDotsY;
    dReal dotDx = parameters.dotsDistanceX;
    dReal dotDy = parameters.dotsDistanceY;
    dReal dotRadius = parameters.dotDiameterDistanceRatio * std::min(dotDx, dotDy) / 2;
    dReal bigDotRadius = parameters.bigDotDiameterDistanceRatio * std::min(dotDx, dotDy) / 2;
    dReal selectedRadius = dotRadius;
    dReal dotLength = 0.01f * boardEx[2];
    dReal dotZOffset = dotLength/2;
    int numverts = (int)(fTessellation*48.0f) + 3;

    if (nDotsX >= 3 && nDotsY >= 3) {
        for (dReal rowPos = -(nDotsX-1)/2; rowPos <= (nDotsX-1)/2; rowPos++ ) {
            for (dReal colPos = -(nDotsY-1)/2; colPos <= (nDotsY-1)/2; colPos++ ) {
                Vector dotPos = Vector(rowPos * dotDx, colPos * dotDy, dotZOffset);
                // calibration board pattern types
                if (parameters.patternName == "threeBigDotsDotGrid") {
                    dReal cRowPos = std::ceil(rowPos);
                    dReal cColPos = std::ceil(colPos);
                    // use big dot radius if dot pos coords is at (0, 0), (0, 1), or (1, 0) when ceiling'd
                    // otherwise, use normal dot radius
                    if ((cRowPos == 0 && (cColPos == 0 || cColPos == 1)) || (cRowPos == 1 && cColPos == 0)) {
                        selectedRadius = bigDotRadius;
                    }
                    else {
                        selectedRadius = dotRadius;
                    }
                }
                AppendCylinderTriangulation(dotPos, selectedRadius, dotLength, numverts, tri);
            }
        }
    }
    else {
        RAVELOG_WARN_FORMAT("Cannot generate calibration board dot grid of size %sx%s - minimum size is 3 x 3.", parameters.numDotsX%parameters.numDotsY);
    }
}

int KinBody::GeometryInfo::SideWall::Compare(const SideWall& rhs, dReal fUnitScale, dReal fEpsilon) const
{
    if(!IsZeroWithEpsilon3(transf.trans - rhs.transf.trans*fUnitScale, fEpsilon)) {
        return 1;
    }
    if(!IsZeroWithEpsilon4(transf.rot - rhs.transf.rot, fEpsilon)) {
        return 2;
    }
    if(!IsZeroWithEpsilon3(vExtents - rhs.vExtents*fUnitScale, fEpsilon)) {
        return 3;
    }
    return 0;
}

int KinBody::GeometryInfo::CalibrationBoardParameters::Compare(const CalibrationBoardParameters& other, dReal fEpsilon) const {
    if (numDotsX != other.numDotsX) {
        return 1;
    }
    if (numDotsY != other.numDotsY) {
        return 2;
    }
    if ( RaveFabs(dotsDistanceX - other.dotsDistanceX) > fEpsilon) {
        return 3;
    }
    if ( RaveFabs(dotsDistanceY - other.dotsDistanceY) > fEpsilon) {
        return 4;
    }
    if (!IsZeroWithEpsilon3(dotColor - other.dotColor, fEpsilon)) {
        return 5;
    }
    if (patternName != other.patternName) {
        return 6;
    }
    if (RaveFabs(dotDiameterDistanceRatio - other.dotDiameterDistanceRatio) > fEpsilon) {
        return 7;
    }
    if (RaveFabs(bigDotDiameterDistanceRatio - other.bigDotDiameterDistanceRatio) > fEpsilon) {
        return 8;
    }
    return 0;
}

int KinBody::GeometryInfo::Compare(const GeometryInfo& rhs, dReal fUnitScale, dReal fEpsilon) const
{
    if( _type != rhs._type ) {
        return 1;
    }
    if( _id != rhs._id ) {
        return 2;
    }
    if( _name != rhs._name ) {
        return 3;
    }
    if( !IsZeroWithEpsilon4(_t.rot - rhs._t.rot, fEpsilon) ) {
        return 4;
    }

    if( !IsZeroWithEpsilon3(_t.trans - rhs._t.trans*fUnitScale, fEpsilon) ) {
        return 5;
    }

    switch(_type) {
    case GT_Box:
        if( !IsZeroWithEpsilon3(_vGeomData - rhs._vGeomData*fUnitScale, fEpsilon) ) {
            return 6;
        }
        break;

    case GT_Sphere:
        if( RaveFabs(_vGeomData.x - rhs._vGeomData.x*fUnitScale) > fEpsilon ) {
            return 7;
        }
        break;

    case GT_Capsule:
    case GT_Cylinder:
        if( RaveFabs(_vGeomData.x - rhs._vGeomData.x*fUnitScale) > fEpsilon || RaveFabs(_vGeomData.y - rhs._vGeomData.y*fUnitScale) > fEpsilon ) {
            return 8;
        }
        break;

    case GT_Container:
        if( !IsZeroWithEpsilon3(_vGeomData - rhs._vGeomData*fUnitScale, fEpsilon) ) {
            return 9;
        }
        if( !IsZeroWithEpsilon3(_vGeomData2 - rhs._vGeomData2*fUnitScale, fEpsilon) ) {
            return 10;
        }
        if( !IsZeroWithEpsilon3(_vGeomData3 - rhs._vGeomData3*fUnitScale, fEpsilon) ) {
            return 11;
        }
        if( !IsZeroWithEpsilon3(_vGeomData4 - rhs._vGeomData4*fUnitScale, fEpsilon) ) {
            return 12;
        }
        if( !IsZeroWithEpsilon3(_vNegativeCropContainerMargins - rhs._vNegativeCropContainerMargins*fUnitScale, fEpsilon) ) {
            return 22;
        }
        if( !IsZeroWithEpsilon3(_vPositiveCropContainerMargins - rhs._vPositiveCropContainerMargins*fUnitScale, fEpsilon) ) {
            return 23;
        }
        if( !IsZeroWithEpsilon3(_vNegativeCropContainerEmptyMargins - rhs._vNegativeCropContainerEmptyMargins*fUnitScale, fEpsilon) ) {
            return 26;
        }
        if( !IsZeroWithEpsilon3(_vPositiveCropContainerEmptyMargins - rhs._vPositiveCropContainerEmptyMargins*fUnitScale, fEpsilon) ) {
            return 27;
        }
        break;

    case GT_Cage: {
        if( !IsZeroWithEpsilon3(_vGeomData - rhs._vGeomData*fUnitScale, fEpsilon) ) {
            return 13;
        }
        if( !IsZeroWithEpsilon3(_vGeomData2 - rhs._vGeomData2*fUnitScale, fEpsilon) ) {
            return 14;
        }
        if( _vSideWalls.size() != rhs._vSideWalls.size() ) {
            return 15;
        }

        for(int iwall = 0; iwall < (int)_vSideWalls.size(); ++iwall) {
            if(_vSideWalls[iwall].Compare(rhs._vSideWalls[iwall], fUnitScale, fEpsilon) > 0) {
                return 16;
            }
        }
        if( !IsZeroWithEpsilon3(_vNegativeCropContainerMargins - rhs._vNegativeCropContainerMargins*fUnitScale, fEpsilon) ) {
            return 24;
        }
        if( !IsZeroWithEpsilon3(_vPositiveCropContainerMargins - rhs._vPositiveCropContainerMargins*fUnitScale, fEpsilon) ) {
            return 25;
        }
        if( !IsZeroWithEpsilon3(_vNegativeCropContainerEmptyMargins - rhs._vNegativeCropContainerEmptyMargins*fUnitScale, fEpsilon) ) {
            return 28;
        }
        if( !IsZeroWithEpsilon3(_vPositiveCropContainerEmptyMargins - rhs._vPositiveCropContainerEmptyMargins*fUnitScale, fEpsilon) ) {
            return 29;
        }
        break;
    }
    case GT_ConicalFrustum:
        if( RaveFabs(_vGeomData.x - rhs._vGeomData.x*fUnitScale) > fEpsilon ||
            RaveFabs(_vGeomData.y - rhs._vGeomData.y*fUnitScale) > fEpsilon ||
            RaveFabs(_vGeomData.z - rhs._vGeomData.z*fUnitScale) > fEpsilon) {
            return 31;
        }
        break;

    case GT_Axial:
        if( _vAxialSlices.size() != rhs._vAxialSlices.size() ) {
            return 30;
        }
    // use collision mesh to make the rest of the comparison
    case GT_Prism:
    case GT_TriMesh:
        if( _meshcollision.vertices.size() != rhs._meshcollision.vertices.size() ) {
            return 17;
        }
        for(int ivertex = 0; ivertex < (int)_meshcollision.vertices.size(); ++ivertex) {
            if( !IsZeroWithEpsilon3(_meshcollision.vertices[ivertex]-rhs._meshcollision.vertices[ivertex]*fUnitScale, fEpsilon) ) {
                return 18;
            }
        }
        if( _meshcollision.indices != rhs._meshcollision.indices ) {
            return 19;
        }

        break;

    case GT_CalibrationBoard:
        if( !IsZeroWithEpsilon3(_vGeomData - rhs._vGeomData*fUnitScale, fEpsilon) ) {
            return 20;
        }
        for(int iparams = 0; iparams < (int)_calibrationBoardParameters.size(); ++iparams) {
            if(_calibrationBoardParameters[iparams].Compare(rhs._calibrationBoardParameters[iparams], fEpsilon) > 0) {
                return 21;
            }
        }

        break;

    case GT_None:
        break;
    }

    return 0;
}

bool KinBody::GeometryInfo::InitCollisionMesh(float fTessellation)
{
    if( _type == GT_None || _type == GT_TriMesh || _type == GT_Prism ) {
        return true;
    }

    // is clear() better since it releases the memory?
    _modifiedFields |= GIF_Mesh;
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
        int numverts = (int)(fTessellation*48.0f) + 3;
        AppendCylinderTriangulation(Vector(0, 0, 0), GetCylinderRadius(), GetCylinderHeight()*0.5, numverts, _meshcollision);
        break;
    }
    case GT_Capsule: {
        // capsule is on z axis
        int numverts = (int)(fTessellation*48.0f) + 3;
        AppendCylinderTriangulation(Vector(0, 0, 0), GetCapsuleRadius(), GetCapsuleHeight()*0.5, numverts, _meshcollision);
        TriMesh tri;
        GenerateSphereTriangulation(tri, 3 + (int)(logf(fTessellation) / logf(2.0f)));
        dReal fRadius = GetCapsuleRadius();
        FOREACH(it, tri.vertices) {
            *it *= fRadius;
        }
        _meshcollision.Append(tri, Transform(Vector(), Vector(0, 0, GetCapsuleHeight() * 0.5)));
        _meshcollision.Append(tri, Transform(Vector(), Vector(0, 0, -GetCapsuleHeight() * 0.5)));
        break;
    }
    case GT_ConicalFrustum:
        AppendConicalFrustumTriangulation(
            Vector(0, 0, 0),
            GetConicalFrustumTopRadius(),
            GetConicalFrustumBottomRadius(),
            GetConicalFrustumHeight() / 2,
            (int)(fTessellation*48.0f) + 3,
            _meshcollision
            );
        break;
    case GT_Axial: {
        if (_vAxialSlices.size() > 1) {
            // there has to be at least two slices: top and bottom
            // sort the axial slices by the Z value
            std::sort(_vAxialSlices.begin(), _vAxialSlices.end());

            int numberOfSections = (int)(fTessellation*48.0f) + 3;
            int numberOfAxialSlices = _vAxialSlices.size();
            _meshcollision.vertices.reserve(2+numberOfAxialSlices*(numberOfSections+1));
            _meshcollision.indices.reserve((numberOfAxialSlices*2)*(numberOfSections+1));

            // add top center point
            _meshcollision.vertices.push_back(Vector(0, 0, _vAxialSlices.front().zOffset));

            // add bottom center point
            _meshcollision.vertices.push_back(Vector(0, 0, _vAxialSlices.back().zOffset));

            // tessellate the surfaces
            dReal dAngle = 2 * PI / (dReal)numberOfSections;
            for(int sectionIndex = 0; sectionIndex <= numberOfSections; sectionIndex++) {
                dReal dTheta = (dReal)sectionIndex * dAngle;
                dReal sinTheta = RaveSin(dTheta);
                dReal cosTheta = RaveCos(dTheta);

                // add every slice's outer edge vertices
                FOREACH(axialSlice, _vAxialSlices) {
                    _meshcollision.vertices.push_back(Vector(axialSlice->radius*cosTheta, axialSlice->radius*sinTheta, axialSlice->zOffset));
                }

                // we can start adding vertices after the first section only
                if (sectionIndex < 1) {
                    continue;
                }

                int numberOfVertices = (int)(_meshcollision.vertices.size());

                // add top circle surface
                _meshcollision.indices.push_back(0);
                _meshcollision.indices.push_back(numberOfVertices-numberOfAxialSlices);
                _meshcollision.indices.push_back(numberOfVertices-2*numberOfAxialSlices);

                // add bottom circle surface
                _meshcollision.indices.push_back(1);
                _meshcollision.indices.push_back(numberOfVertices-numberOfAxialSlices-1);
                _meshcollision.indices.push_back(numberOfVertices-1);

                // add the upper side triangles
                for (int index = 0; index < numberOfAxialSlices-1; index++) {
                    _meshcollision.indices.push_back(index+numberOfVertices-numberOfAxialSlices);
                    _meshcollision.indices.push_back(index+numberOfVertices-numberOfAxialSlices+1);
                    _meshcollision.indices.push_back(index+numberOfVertices-2*numberOfAxialSlices);
                }

                // add the lower side triangles
                for (int index = 1; index < numberOfAxialSlices; index++) {
                    _meshcollision.indices.push_back(index+(numberOfVertices-numberOfAxialSlices));
                    _meshcollision.indices.push_back(index+(numberOfVertices-2*numberOfAxialSlices));
                    _meshcollision.indices.push_back(index+(numberOfVertices-2*numberOfAxialSlices)-1);
                }
            }
        }
        break;
    }
    case GT_Cage: {
        const Vector& vCageBaseExtents = _vGeomData;
        for (size_t i = 0; i < _vSideWalls.size(); ++i) {
            const SideWall &s = _vSideWalls[i];
            const size_t vBase = _meshcollision.vertices.size();
            AppendBoxTriangulation(Vector(0, 0, s.vExtents[2]), s.vExtents, _meshcollision);

            for (size_t j = 0; j < 8; ++j) {
                _meshcollision.vertices[vBase + j] = s.transf * _meshcollision.vertices[vBase + j];
            }
        }
        // finally add the base
        AppendBoxTriangulation(Vector(0, 0, vCageBaseExtents.z),vCageBaseExtents, _meshcollision);
        break;
    }
    case GT_Container: {
        const Vector& outerextents = _vGeomData;
        const Vector& innerextents = _vGeomData2;
        const Vector& bottomcross = _vGeomData3;
        const Vector& bottom = _vGeomData4;
        dReal zoffset = 0;
        if( bottom[2] > 0 ) {
            if( bottom[0] > 0 && bottom[1] > 0 ) {
                zoffset = bottom[2];
            }
        }
        // +x wall
        AppendBoxTriangulation(Vector((outerextents[0]+innerextents[0])/4.,0,outerextents[2]/2.+zoffset), Vector((outerextents[0]-innerextents[0])/4., outerextents[1]/2., outerextents[2]/2.), _meshcollision);
        // -x wall
        AppendBoxTriangulation(Vector(-(outerextents[0]+innerextents[0])/4.,0,outerextents[2]/2.+zoffset), Vector((outerextents[0]-innerextents[0])/4., outerextents[1]/2., outerextents[2]/2.), _meshcollision);
        // +y wall
        AppendBoxTriangulation(Vector(0,(outerextents[1]+innerextents[1])/4.,outerextents[2]/2.+zoffset), Vector(outerextents[0]/2., (outerextents[1]-innerextents[1])/4., outerextents[2]/2.), _meshcollision);
        // -y wall
        AppendBoxTriangulation(Vector(0,-(outerextents[1]+innerextents[1])/4.,outerextents[2]/2.+zoffset), Vector(outerextents[0]/2., (outerextents[1]-innerextents[1])/4., outerextents[2]/2.), _meshcollision);
        // bottom
        if( outerextents[2] - innerextents[2] >= 1e-6 ) { // small epsilon error can make thin triangles appear, so test with a reasonable threshold
            AppendBoxTriangulation(Vector(0,0,(outerextents[2]-innerextents[2])/2.+zoffset), Vector(outerextents[0]/2., outerextents[1]/2., (outerextents[2]-innerextents[2])/2), _meshcollision);
        }
        // cross
        if( bottomcross[2] > 0 ) {
            if( bottomcross[0] > 0 ) {
                AppendBoxTriangulation(Vector(0, 0, bottomcross[2]/2+outerextents[2]-innerextents[2]+zoffset), Vector(bottomcross[0]/2, innerextents[1]/2, bottomcross[2]/2), _meshcollision);
            }
            if( bottomcross[1] > 0 ) {
                AppendBoxTriangulation(Vector(0, 0, bottomcross[2]/2+outerextents[2]-innerextents[2]+zoffset), Vector(innerextents[0]/2, bottomcross[1]/2, bottomcross[2]/2), _meshcollision);
            }
        }
        // bottom
        if( bottom[2] > 0 ) {
            if( bottom[0] > 0 && bottom[1] > 0 ) {
                AppendBoxTriangulation(Vector(0, 0, bottom[2]/2), Vector(bottom[0]/2., bottom[1]/2., bottom[2]/2.), _meshcollision);
            }
        }
        break;
    }
    case GT_CalibrationBoard: {
        // create board mesh
        Vector boardEx = GetBoxExtents();
        AppendBoxTriangulation(Vector(0, 0, -boardEx[2]), boardEx, _meshcollision);
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_("unrecognized geom type %d!"), _type, ORE_InvalidArguments);
    }

    return true;
}

bool KinBody::GeometryInfo::ComputeInnerEmptyVolume(Transform& tInnerEmptyVolume, Vector& abInnerEmptyExtents) const
{
    switch(_type) {
    case GT_Cage: {
        Vector vwallmin, vwallmax;
        vwallmax.z = vwallmin.z = _vGeomData.z*2;

        // initialize to the base extents if there is no wall
        vwallmin.x = -_vGeomData.x;
        vwallmin.y = -_vGeomData.y;
        vwallmax.x = _vGeomData.x;
        vwallmax.y = _vGeomData.y;
        int sideWallExtents = 0;

        FOREACH(itwall, _vSideWalls) {
            // compute the XYZ extents of the wall
            Vector vxaxis = geometry::ExtractAxisFromQuat(itwall->transf.rot, 0);
            Vector vyaxis = geometry::ExtractAxisFromQuat(itwall->transf.rot, 1);
            Vector vzaxis = geometry::ExtractAxisFromQuat(itwall->transf.rot, 2);

            Vector vprojectedextents;
            for(int idim = 0; idim < 3; ++idim) {
                vprojectedextents[idim] = RaveFabs(vxaxis[idim])*itwall->vExtents.x + RaveFabs(vyaxis[idim])*itwall->vExtents.y + RaveFabs(vzaxis[idim])*itwall->vExtents.z;
            }

            // the origin of the side wall is the bottom center
            if( vwallmax.z < itwall->transf.trans.z + 2*vprojectedextents.z ) {
                vwallmax.z = itwall->transf.trans.z + 2*vprojectedextents.z;
            }

            sideWallExtents |= (int)(1<<itwall->type);

            switch(itwall->type) {
            case GeometryInfo::SWT_NX:
                vwallmin.x = itwall->transf.trans.x + vprojectedextents.x;
                break;
            case GeometryInfo::SWT_NY:
                vwallmin.y = itwall->transf.trans.y + vprojectedextents.y;
                break;
            case GeometryInfo::SWT_PX:
                vwallmax.x = itwall->transf.trans.x - vprojectedextents.x;
                break;
            case GeometryInfo::SWT_PY:
                vwallmax.y = itwall->transf.trans.y - vprojectedextents.y;
                break;
            }
        }

        // if _vGeomData2 is greater than 0, force inner region wherever possible.
        // The only thing that will prevent _vGeomData2's inner region is a wall present.
        // Should not use base to restrict _vGeomData2
        if( _vGeomData2.x > 0 ) {
            if( sideWallExtents & (1<<GeometryInfo::SWT_NX) ) {
                if( vwallmin.x < -0.5*_vGeomData2.x ) {
                    vwallmin.x = -0.5*_vGeomData2.x;
                }
            }
            else {
                // no wall defined on NX
                vwallmin.x = -0.5*_vGeomData2.x;
            }

            if( sideWallExtents & (1<<GeometryInfo::SWT_PX) ) {
                if( vwallmax.x > 0.5*_vGeomData2.x ) {
                    vwallmax.x = 0.5*_vGeomData2.x;
                }
            }
            else {
                // no wall defined on NX
                vwallmax.x = 0.5*_vGeomData2.x;
            }
        }

        if( _vGeomData2.y > 0 ) {
            if( sideWallExtents & (1<<GeometryInfo::SWT_NY) ) {
                if( vwallmin.y < -0.5*_vGeomData2.y ) {
                    vwallmin.y = -0.5*_vGeomData2.y;
                }
            }
            else {
                vwallmin.y = -0.5*_vGeomData2.y;
            }

            if( sideWallExtents & (1<<GeometryInfo::SWT_PY) ) {
                if( vwallmax.y > 0.5*_vGeomData2.y ) {
                    vwallmax.y = 0.5*_vGeomData2.y;
                }
            }
            else {
                vwallmax.y = 0.5*_vGeomData2.y;
            }
        }

        // the top has no constraints, so use the max of walls and force inner region
        if( vwallmax.z < _vGeomData.z*2 + _vGeomData2.z ) {
            vwallmax.z = _vGeomData.z*2 + _vGeomData2.z;
        }

        abInnerEmptyExtents = 0.5*(vwallmax - vwallmin);
        tInnerEmptyVolume = _t;
        tInnerEmptyVolume.trans += tInnerEmptyVolume.rotate(0.5*(vwallmax + vwallmin));
        return true;
    }
    case GT_Container: {
        Transform tempty;
        // full outer extents - full inner extents + inner extents = _vGeomData.z - 0.5*_vGeomData2.z
        tempty.trans.z = _vGeomData.z - 0.5 * _vGeomData2.z;
        if( _vGeomData4.x > 0 && _vGeomData4.y > 0 && _vGeomData4.z > 0 ) {
            // if _vGeomData4 is valid, need to shift the empty region up.
            tempty.trans.z += _vGeomData4.z;
        }
        tInnerEmptyVolume = _t*tempty;
        abInnerEmptyExtents = 0.5*_vGeomData2;
        return true;
    }
    default:
        return false;
    }
}

inline void SaveJsonValue(rapidjson::Value& v, const KinBody::GeometryInfo::SideWall& t, rapidjson::Document::AllocatorType& alloc)
{
    v.SetObject();
    orjson::SetJsonValueByKey(v, "transform", t.transf, alloc);
    orjson::SetJsonValueByKey(v, "halfExtents", t.vExtents, alloc);
    switch (t.type) {
    case KinBody::GeometryInfo::SideWallType::SWT_NX:
        orjson::SetJsonValueByKey(v, "type", "nx", alloc);
        break;
    case KinBody::GeometryInfo::SideWallType::SWT_PX:
        orjson::SetJsonValueByKey(v, "type", "px", alloc);
        break;
    case KinBody::GeometryInfo::SideWallType::SWT_NY:
        orjson::SetJsonValueByKey(v, "type", "ny", alloc);
        break;
    case KinBody::GeometryInfo::SideWallType::SWT_PY:
        orjson::SetJsonValueByKey(v, "type", "py", alloc);
        break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_("unrecognized sidewall type %d for saving to json"), t.type, ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& rValue, KinBody::GeometryInfo::SideWall& sideWall)
{
    if(rValue.IsObject()) {
        orjson::LoadJsonValueByKey(rValue, "transform", sideWall.transf);
        orjson::LoadJsonValueByKey(rValue, "halfExtents", sideWall.vExtents);

        if (!rValue.HasMember("type")) {
            // no type provided, default to nx
            sideWall.type = KinBody::GeometryInfo::SideWallType::SWT_NX;
        }
        else if(rValue["type"].IsInt()) {
            // backward compatibility, support enum sideWall type
            int sideWallType = (int)KinBody::GeometryInfo::SideWallType::SWT_NX;
            orjson::LoadJsonValueByKey(rValue, "type", sideWallType);
            if (!(sideWallType >= KinBody::GeometryInfo::SideWallType::SWT_First
                  && sideWallType <= KinBody::GeometryInfo::SideWallType::SWT_Last)) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("unrecognized sidewall type enum range %d for loading from json"), sideWallType, ORE_InvalidArguments);
            }
            sideWall.type = (KinBody::GeometryInfo::SideWallType)sideWallType;
        }
        else {
            std::string type = "nx";
            orjson::LoadJsonValueByKey(rValue, "type", type);
            if(type == "nx") {
                sideWall.type = KinBody::GeometryInfo::SideWallType::SWT_NX;
            }
            else if(type == "px") {
                sideWall.type = KinBody::GeometryInfo::SideWallType::SWT_PX;
            }
            else if(type == "ny") {
                sideWall.type = KinBody::GeometryInfo::SideWallType::SWT_NY;
            }
            else if(type == "py") {
                sideWall.type = KinBody::GeometryInfo::SideWallType::SWT_PY;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("unrecognized sidewall type string %s for loading from json"), type, ORE_InvalidArguments);
            }
        }
    } else {
        throw OPENRAVE_EXCEPTION_FORMAT(_("Cannot convert JSON type %s to Geometry::SideWall"), orjson::GetJsonTypeName(rValue), ORE_InvalidArguments);
    }
}

inline void LoadJsonValue(const rapidjson::Value& rValue, KinBody::GeometryInfo::CalibrationBoardParameters& p) {
    if (!rValue.IsObject()) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Cannot load value of non-object.", OpenRAVE::ORE_InvalidArguments);
    }
    const char *calibrationBoardParamNames[] = {
        "numDotsX", "numDotsY", "dotsDistanceX",
        "dotsDistanceY", "dotColor", "patternName",
        "dotDiameterDistanceRatio", "bigDotDiameterDistanceRatio"
    };
    for (int idx = 0; idx < 8; idx++) {
        if (!rValue.HasMember(calibrationBoardParamNames[idx])) {
            RAVELOG_ERROR_FORMAT("Missing calibration board parameter \"%s\".", calibrationBoardParamNames[idx]);
        }
    }
    orjson::LoadJsonValue(rValue["numDotsX"], p.numDotsX);
    orjson::LoadJsonValue(rValue["numDotsY"], p.numDotsY);
    orjson::LoadJsonValue(rValue["dotsDistanceX"], p.dotsDistanceX);
    orjson::LoadJsonValue(rValue["dotsDistanceY"], p.dotsDistanceY);
    orjson::LoadJsonValue(rValue["dotColor"], p.dotColor);
    orjson::LoadJsonValue(rValue["patternName"], p.patternName);
    orjson::LoadJsonValue(rValue["dotDiameterDistanceRatio"], p.dotDiameterDistanceRatio);
    orjson::LoadJsonValue(rValue["bigDotDiameterDistanceRatio"], p.bigDotDiameterDistanceRatio);
}

void KinBody::GeometryInfo::ConvertUnitScale(dReal fUnitScale)
{
    _t.trans *= fUnitScale;
    _modifiedFields |= GIF_Transform;

    switch(_type) {
    case GT_Box:
        _vGeomData *= fUnitScale;
        break;

    case GT_Container:
        _vGeomData *= fUnitScale;
        _vGeomData2 *= fUnitScale;
        _vGeomData3 *= fUnitScale;
        _vGeomData4 *= fUnitScale;
        _vNegativeCropContainerMargins *= fUnitScale;
        _vPositiveCropContainerMargins *= fUnitScale;
        _vNegativeCropContainerEmptyMargins *= fUnitScale;
        _vPositiveCropContainerEmptyMargins *= fUnitScale;
        break;

    case GT_Cage: {
        _vGeomData *= fUnitScale;

        FOREACH(itwall, _vSideWalls) {
            itwall->transf.trans *= fUnitScale;
            itwall->vExtents *= fUnitScale;
        }
        _vGeomData2 *= fUnitScale;
        _vNegativeCropContainerMargins *= fUnitScale;
        _vPositiveCropContainerMargins *= fUnitScale;
        _vNegativeCropContainerEmptyMargins *= fUnitScale;
        _vPositiveCropContainerEmptyMargins *= fUnitScale;
        break;
    }
    case GT_Capsule:
    case GT_ConicalFrustum:
    case GT_Sphere:
    case GT_Cylinder:
        _vGeomData *= fUnitScale;
        break;

    case GT_Axial:
        FOREACH(itAxialSlice, _vAxialSlices) {
            itAxialSlice->zOffset *= fUnitScale;
            itAxialSlice->radius *= fUnitScale;
        }
        break;

    case GT_Prism:
    case GT_TriMesh:
        FOREACH(itvertex, _meshcollision.vertices) {
            *itvertex *= fUnitScale;
        }
        _modifiedFields |= GIF_Mesh;
        break;

    case GT_CalibrationBoard:
        _vGeomData *= fUnitScale;
        FOREACH(itparams, _calibrationBoardParameters) {
            itparams->dotsDistanceX *= fUnitScale;
            itparams->dotsDistanceY *= fUnitScale;
        }
        break;

    case GT_None:
        break;
    }
}

void KinBody::GeometryInfo::Reset()
{
    _t = Transform();
    _vGeomData = Vector();
    _vGeomData2 = Vector();
    _vGeomData3 = Vector();
    _vGeomData4 = Vector();
    _vSideWalls.clear();
    _vAxialSlices.clear();
    _vDiffuseColor = Vector(1,1,1);
    _vAmbientColor = Vector(0,0,0);
    _meshcollision.vertices.clear();
    _meshcollision.indices.clear();
    _type = GT_None;
    _id.clear();
    _name.clear();
    _filenamerender.clear();
    _filenamecollision.clear();
    _vRenderScale = Vector(1,1,1);
    _vCollisionScale = Vector(1,1,1);
    _fTransparency = 0;
    _bVisible = true;
    _bModifiable = true;
    _calibrationBoardParameters.clear();
    _modifiedFields = 0xffffffff;
    _vNegativeCropContainerMargins = Vector(0,0,0);
    _vPositiveCropContainerMargins = Vector(0,0,0);
    _vNegativeCropContainerEmptyMargins = Vector(0,0,0);
    _vPositiveCropContainerEmptyMargins = Vector(0,0,0);
}

const char* GetGeometryTypeString(GeometryType geometryType)
{
    switch(geometryType) {
    case GT_Box:
        return "box";
    case GT_Container:
        return "container";
    case GT_Cage:
        return "cage";
    case GT_Sphere:
        return "sphere";
    case GT_Cylinder:
        return "cylinder";
    case GT_Axial:
        return "axial";
    case GT_TriMesh:
        return "trimesh";
    case GT_CalibrationBoard:
        return "calibrationboard";
    case GT_ConicalFrustum:
        return "conicalfrustum";
    case GT_Prism:
        return "prism";
    case GT_Capsule:
        return "capsule";
    case GT_None:
        return "";
    }
    return "(unknown)";
}

void KinBody::GeometryInfo::SerializeJSON(rapidjson::Value& rGeometryInfo, rapidjson::Document::AllocatorType& allocator, const dReal fUnitScale, int options) const
{
    rGeometryInfo.SetObject();
    orjson::SetJsonValueByKey(rGeometryInfo, "id", _id, allocator);
    orjson::SetJsonValueByKey(rGeometryInfo, "name", _name, allocator);

    // unfortunately too much code relies on "transform" being present
    //if( _t.rot.x != 1 || _t.rot.y != 0 || _t.rot.z != 0 || _t.rot.w != 0 || _t.trans.x != 0 || _t.trans.y != 0 || _t.trans.z != 0 )
    {
        Transform tscaled = _t;
        tscaled.trans *= fUnitScale;
        orjson::SetJsonValueByKey(rGeometryInfo, "transform", tscaled, allocator);
    }

    orjson::SetJsonValueByKey(rGeometryInfo, "type", GetGeometryTypeString(_type), allocator);

    switch(_type) {
    case GT_Box:
        orjson::SetJsonValueByKey(rGeometryInfo, "halfExtents", _vGeomData*fUnitScale, allocator);
        break;

    case GT_Container:
        //if( _vGeomData[0] != 0 || _vGeomData[1] != 0 || _vGeomData[2] != 0 )
        {
            orjson::SetJsonValueByKey(rGeometryInfo, "outerExtents", _vGeomData*fUnitScale, allocator);
        }
        //if( _vGeomData2[0] != 0 || _vGeomData2[1] != 0 || _vGeomData2[2] != 0 )
        {
            orjson::SetJsonValueByKey(rGeometryInfo, "innerExtents", _vGeomData2*fUnitScale, allocator);
        }
        if( _vGeomData3[0] != 0 || _vGeomData3[1] != 0 || _vGeomData3[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "bottomCross", _vGeomData3*fUnitScale, allocator);
        }
        if( _vGeomData4[0] != 0 || _vGeomData4[1] != 0 || _vGeomData4[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "bottom", _vGeomData4*fUnitScale, allocator);
        }
        if( _vNegativeCropContainerMargins[0] != 0 || _vNegativeCropContainerMargins[1] != 0 || _vNegativeCropContainerMargins[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "negativeCropContainerMargins", _vNegativeCropContainerMargins*fUnitScale, allocator);
        }
        if( _vPositiveCropContainerMargins[0] != 0 || _vPositiveCropContainerMargins[1] != 0 || _vPositiveCropContainerMargins[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "positiveCropContainerMargins", _vPositiveCropContainerMargins*fUnitScale, allocator);
        }
        if( _vNegativeCropContainerEmptyMargins[0] != 0 || _vNegativeCropContainerEmptyMargins[1] != 0 || _vNegativeCropContainerEmptyMargins[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "negativeCropContainerEmptyMargins", _vNegativeCropContainerEmptyMargins*fUnitScale, allocator);
        }
        if( _vPositiveCropContainerEmptyMargins[0] != 0 || _vPositiveCropContainerEmptyMargins[1] != 0 || _vPositiveCropContainerEmptyMargins[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "positiveCropContainerEmptyMargins", _vPositiveCropContainerEmptyMargins*fUnitScale, allocator);
        }
        break;

    case GT_Cage: {
        //if( _vGeomData[0] != 0 || _vGeomData[1] != 0 || _vGeomData[2] != 0 )
        {
            orjson::SetJsonValueByKey(rGeometryInfo, "baseExtents", _vGeomData*fUnitScale, allocator);
        }

        if( _vGeomData2.x > g_fEpsilon ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "innerSizeX", _vGeomData2.x*fUnitScale, allocator);
        }
        if( _vGeomData2.y > g_fEpsilon ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "innerSizeY", _vGeomData2.y*fUnitScale, allocator);
        }
        if( _vGeomData2.z > g_fEpsilon ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "innerSizeZ", _vGeomData2.z*fUnitScale, allocator);
        }
        if( !_vSideWalls.empty() ) {
            std::vector<SideWall> vScaledSideWalls = _vSideWalls;
            FOREACH(itwall, vScaledSideWalls) {
                itwall->transf.trans *= fUnitScale;
                itwall->vExtents *= fUnitScale;
            }
            orjson::SetJsonValueByKey(rGeometryInfo, "sideWalls", vScaledSideWalls, allocator);
        }
        if( _vNegativeCropContainerMargins[0] != 0 || _vNegativeCropContainerMargins[1] != 0 || _vNegativeCropContainerMargins[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "negativeCropContainerMargins", _vNegativeCropContainerMargins*fUnitScale, allocator);
        }
        if( _vPositiveCropContainerMargins[0] != 0 || _vPositiveCropContainerMargins[1] != 0 || _vPositiveCropContainerMargins[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "positiveCropContainerMargins", _vPositiveCropContainerMargins*fUnitScale, allocator);
        }
        if( _vNegativeCropContainerEmptyMargins[0] != 0 || _vNegativeCropContainerEmptyMargins[1] != 0 || _vNegativeCropContainerEmptyMargins[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "negativeCropContainerEmptyMargins", _vNegativeCropContainerEmptyMargins*fUnitScale, allocator);
        }
        if( _vPositiveCropContainerEmptyMargins[0] != 0 || _vPositiveCropContainerEmptyMargins[1] != 0 || _vPositiveCropContainerEmptyMargins[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "positiveCropContainerEmptyMargins", _vPositiveCropContainerEmptyMargins*fUnitScale, allocator);
        }
        break;
    }
    case GT_Sphere:
        orjson::SetJsonValueByKey(rGeometryInfo, "radius", _vGeomData.x*fUnitScale, allocator);
        break;

    case GT_Cylinder:
        orjson::SetJsonValueByKey(rGeometryInfo, "radius", _vGeomData.x*fUnitScale, allocator);
        orjson::SetJsonValueByKey(rGeometryInfo, "height", _vGeomData.y*fUnitScale, allocator);
        break;

    case GT_ConicalFrustum:
        orjson::SetJsonValueByKey(rGeometryInfo, "topRadius", GetConicalFrustumTopRadius()*fUnitScale, allocator);
        orjson::SetJsonValueByKey(rGeometryInfo, "bottomRadius", GetConicalFrustumBottomRadius()*fUnitScale, allocator);
        orjson::SetJsonValueByKey(rGeometryInfo, "height", GetConicalFrustumHeight()*fUnitScale, allocator);
        break;

    case GT_Axial: {
        rapidjson::Value rAxial;
        rAxial.SetArray();
        rAxial.Reserve(_vAxialSlices.size()*2, allocator);
        FOREACH(itAxialSlice, _vAxialSlices) {
            rAxial.PushBack(itAxialSlice->zOffset*fUnitScale, allocator);
            rAxial.PushBack(itAxialSlice->radius*fUnitScale, allocator);
        }
        orjson::SetJsonValueByKey(rGeometryInfo, "axial", rAxial, allocator);
        break;
    }
    case GT_Prism: {
        rapidjson::Value rPoints;
        rPoints.SetArray();
        rPoints.Reserve(_meshcollision.vertices.size(), allocator);
        for( size_t ipoint = 0; ipoint < _meshcollision.vertices.size(); ipoint += 2 ) {
            rPoints.PushBack(_meshcollision.vertices[ipoint].x * fUnitScale, allocator);
            rPoints.PushBack(_meshcollision.vertices[ipoint].y * fUnitScale, allocator);
        }
        rGeometryInfo.AddMember("crossSection", rPoints, allocator);
        rGeometryInfo.AddMember("height", _vGeomData.y, allocator);
        break;
    }
    case GT_Capsule: {
        orjson::SetJsonValueByKey(rGeometryInfo, "radius", _vGeomData.x * fUnitScale, allocator);
        orjson::SetJsonValueByKey(rGeometryInfo, "height", _vGeomData.y * fUnitScale, allocator);
        break;
    }
    case GT_TriMesh: {
        // has to be scaled correctly
        rapidjson::Value rTriMesh;
        rTriMesh.SetObject();
        rapidjson::Value rVertices;
        rVertices.SetArray();
        rVertices.Reserve(_meshcollision.vertices.size()*3, allocator);
        for(size_t ivertex = 0; ivertex < _meshcollision.vertices.size(); ++ivertex) {
            rVertices.PushBack(_meshcollision.vertices[ivertex][0]*fUnitScale, allocator);
            rVertices.PushBack(_meshcollision.vertices[ivertex][1]*fUnitScale, allocator);
            rVertices.PushBack(_meshcollision.vertices[ivertex][2]*fUnitScale, allocator);
        }
        rTriMesh.AddMember("vertices", rVertices, allocator);
        orjson::SetJsonValueByKey(rTriMesh, "indices", _meshcollision.indices, allocator);
        rGeometryInfo.AddMember(rapidjson::Document::StringRefType("mesh"), rTriMesh, allocator);
        break;
    }
    case GT_CalibrationBoard: {
        if( _vGeomData[0] != 0 || _vGeomData[1] != 0 || _vGeomData[2] != 0 ) {
            orjson::SetJsonValueByKey(rGeometryInfo, "halfExtents", _vGeomData*fUnitScale, allocator);
        }
        rapidjson::Value rCalibrationBoardParameters;
        rCalibrationBoardParameters.SetObject();
        CalibrationBoardParameters params = _calibrationBoardParameters.size() > 0 ? _calibrationBoardParameters[0] : CalibrationBoardParameters();
        orjson::SetJsonValueByKey(rCalibrationBoardParameters, "numDotsX", params.numDotsX, allocator);
        orjson::SetJsonValueByKey(rCalibrationBoardParameters, "numDotsY", params.numDotsY, allocator);
        orjson::SetJsonValueByKey(rCalibrationBoardParameters, "dotsDistanceX", params.dotsDistanceX*fUnitScale, allocator);
        orjson::SetJsonValueByKey(rCalibrationBoardParameters, "dotsDistanceY", params.dotsDistanceY*fUnitScale, allocator);
        orjson::SetJsonValueByKey(rCalibrationBoardParameters, "dotColor", params.dotColor, allocator);
        orjson::SetJsonValueByKey(rCalibrationBoardParameters, "patternName", params.patternName, allocator);
        orjson::SetJsonValueByKey(rCalibrationBoardParameters, "dotDiameterDistanceRatio", params.dotDiameterDistanceRatio, allocator);
        orjson::SetJsonValueByKey(rCalibrationBoardParameters, "bigDotDiameterDistanceRatio", params.bigDotDiameterDistanceRatio, allocator);
        rGeometryInfo.AddMember(rapidjson::Document::StringRefType("calibrationBoardParameters"), rCalibrationBoardParameters, allocator);
        break;
    }
    default:
        break;
    }

    if( _fTransparency != 0 ) {
        orjson::SetJsonValueByKey(rGeometryInfo, "transparency", _fTransparency, allocator);
    }
    if( !_bVisible ) { // default is true
        orjson::SetJsonValueByKey(rGeometryInfo, "visible", _bVisible, allocator);
    }
    if( _vDiffuseColor[0] != 1 || _vDiffuseColor[1] != 1 || _vDiffuseColor[2] != 1 ) {
        orjson::SetJsonValueByKey(rGeometryInfo, "diffuseColor", _vDiffuseColor, allocator);
    }
    if( _vAmbientColor[0] != 0 || _vAmbientColor[1] != 0 || _vAmbientColor[2] != 0 ) {
        orjson::SetJsonValueByKey(rGeometryInfo, "ambientColor", _vAmbientColor, allocator);
    }
    if( !_bModifiable ) { // default is true
        orjson::SetJsonValueByKey(rGeometryInfo, "modifiable", _bModifiable, allocator);
    }
}

void KinBody::GeometryInfo::DeserializeJSON(const rapidjson::Value &value, const dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "id", _id);
    orjson::LoadJsonValueByKey(value, "name", _name);

    if (value.HasMember("transform")) {
        Transform tnew;
        orjson::LoadJsonValueByKey(value, "transform", tnew);
        tnew.trans *= fUnitScale;
        if( _t.CompareTransform(tnew, g_fEpsilon) ) {
            _modifiedFields |= KinBody::GeometryInfo::GIF_Transform;
        }
        _t = tnew; // should always set in case of error less than epsilon
    }

    if (value.HasMember("type")) {
        std::string typestr;
        GeometryType type;
        orjson::LoadJsonValueByKey(value, "type", typestr);
        if (typestr == "box") {
            type = GT_Box;
        }
        else if (typestr == "container") {
            type = GT_Container;
        }
        else if (typestr == "cage") {
            type = GT_Cage;
        }
        else if (typestr == "sphere") {
            type = GT_Sphere;
        }
        else if (typestr == "cylinder") {
            type = GT_Cylinder;
        }
        else if (typestr == "axial") {
            type = GT_Axial;
        }
        else if (typestr == "trimesh" || typestr == "mesh") {
            type = GT_TriMesh;
        }
        else if (typestr == "calibrationboard") {
            type = GT_CalibrationBoard;
        }
        else if (typestr == "conicalfrustum") {
            type = GT_ConicalFrustum;
        }
        else if (typestr == "prism") {
            type = GT_Prism;
        }
        else if (typestr == "capsule") {
            type = GT_Capsule;
        }
        else if (typestr.empty()) {
            type = GT_None;
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to deserialize json, unsupported geometry type \"%s\"", typestr, ORE_InvalidArguments);
        }
        if (_type != type) {
            _meshcollision.Clear();
            _type = type;
        }
    }
    Vector vGeomDataTemp;
    std::vector<CalibrationBoardParameters> calibrationBoardParametersTemp;
    switch (_type) {
    case GT_Box:
        if (value.HasMember("halfExtents")) {
            orjson::LoadJsonValueByKey(value, "halfExtents", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vGeomData) {
                _vGeomData = vGeomDataTemp;
                _meshcollision.Clear();
            }
        }
        break;
    case GT_Container:
        if (value.HasMember("outerExtents")) {
            orjson::LoadJsonValueByKey(value, "outerExtents", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vGeomData) {
                _vGeomData = vGeomDataTemp;
                _meshcollision.Clear();
            }
        }
        if (value.HasMember("innerExtents")) {
            orjson::LoadJsonValueByKey(value, "innerExtents", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vGeomData2) {
                _vGeomData2 = vGeomDataTemp;
                _meshcollision.Clear();
            }
        }
        if (value.HasMember("bottomCross")) {
            orjson::LoadJsonValueByKey(value, "bottomCross", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vGeomData3) {
                _vGeomData3 = vGeomDataTemp;
                _meshcollision.Clear();
            }
        }
        if (value.HasMember("bottom")) {
            orjson::LoadJsonValueByKey(value, "bottom", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vGeomData4) {
                _vGeomData4 = vGeomDataTemp;
                _meshcollision.Clear();
            }
        }
        if (value.HasMember("negativeCropContainerMargins")) {
            orjson::LoadJsonValueByKey(value, "negativeCropContainerMargins", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vNegativeCropContainerMargins) {
                _vNegativeCropContainerMargins = vGeomDataTemp;
            }
        }
        if (value.HasMember("positiveCropContainerMargins")) {
            orjson::LoadJsonValueByKey(value, "positiveCropContainerMargins", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vPositiveCropContainerMargins) {
                _vPositiveCropContainerMargins = vGeomDataTemp;
            }
        }
        if (value.HasMember("negativeCropContainerEmptyMargins")) {
            orjson::LoadJsonValueByKey(value, "negativeCropContainerEmptyMargins", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vNegativeCropContainerEmptyMargins) {
                _vNegativeCropContainerEmptyMargins = vGeomDataTemp;
            }
        }
        if (value.HasMember("positiveCropContainerEmptyMargins")) {
            orjson::LoadJsonValueByKey(value, "positiveCropContainerEmptyMargins", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vPositiveCropContainerEmptyMargins) {
                _vPositiveCropContainerEmptyMargins = vGeomDataTemp;
            }
        }
        break;
    case GT_Cage:
        if (value.HasMember("baseExtents")) {
            orjson::LoadJsonValueByKey(value, "baseExtents", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vGeomData) {
                _vGeomData = vGeomDataTemp;
                _meshcollision.Clear();
            }
        }
        vGeomDataTemp = _vGeomData2;
        if (value.HasMember("innerSizeX")) {
            orjson::LoadJsonValueByKey(value, "innerSizeX", vGeomDataTemp.x);
            vGeomDataTemp.x *= fUnitScale;
        }
        if (value.HasMember("innerSizeY")) {
            orjson::LoadJsonValueByKey(value, "innerSizeY", vGeomDataTemp.y);
            vGeomDataTemp.y *= fUnitScale;
        }
        if (value.HasMember("innerSizeZ")) {
            orjson::LoadJsonValueByKey(value, "innerSizeZ", vGeomDataTemp.z);
            vGeomDataTemp.z *= fUnitScale;
        }
        if (vGeomDataTemp != _vGeomData2) {
            _vGeomData2 = vGeomDataTemp;
            _meshcollision.Clear();
        }

        if (value.HasMember("sideWalls")) {
            std::vector<SideWall> vSideWalls;
            orjson::LoadJsonValueByKey(value, "sideWalls", vSideWalls);
            FOREACH(itsidewall, vSideWalls) {
                itsidewall->transf.trans *= fUnitScale;
                itsidewall->vExtents *= fUnitScale;
            }
            if (vSideWalls.size() != _vSideWalls.size()) {
                _vSideWalls = std::move(vSideWalls);
                _meshcollision.Clear();
            }
            else {
                bool bSideWallChanged = false;
                for(unsigned iSideWall=0; iSideWall < vSideWalls.size(); iSideWall++) {
                    if (vSideWalls[iSideWall].Compare(_vSideWalls[iSideWall], fUnitScale) > 0) {
                        _vSideWalls[iSideWall] = std::move(vSideWalls[iSideWall]);
                        bSideWallChanged = true;
                    }
                }
                if (bSideWallChanged) {
                    _meshcollision.Clear();
                }
            }
        }
        if (value.HasMember("negativeCropContainerMargins")) {
            orjson::LoadJsonValueByKey(value, "negativeCropContainerMargins", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vNegativeCropContainerMargins) {
                _vNegativeCropContainerMargins = vGeomDataTemp;
            }
        }
        if (value.HasMember("positiveCropContainerMargins")) {
            orjson::LoadJsonValueByKey(value, "positiveCropContainerMargins", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vPositiveCropContainerMargins) {
                _vPositiveCropContainerMargins = vGeomDataTemp;
            }
        }
        if (value.HasMember("negativeCropContainerEmptyMargins")) {
            orjson::LoadJsonValueByKey(value, "negativeCropContainerEmptyMargins", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vNegativeCropContainerEmptyMargins) {
                _vNegativeCropContainerEmptyMargins = vGeomDataTemp;
            }
        }
        if (value.HasMember("positiveCropContainerEmptyMargins")) {
            orjson::LoadJsonValueByKey(value, "positiveCropContainerEmptyMargins", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vPositiveCropContainerEmptyMargins) {
                _vPositiveCropContainerEmptyMargins = vGeomDataTemp;
            }
        }
        break;
    case GT_Sphere:
        vGeomDataTemp = _vGeomData;
        if (value.HasMember("radius")) {
            orjson::LoadJsonValueByKey(value, "radius", vGeomDataTemp.x);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vGeomData) {
                _vGeomData = vGeomDataTemp;
                _meshcollision.Clear();
            }
        }
        break;
    case GT_Cylinder:
        vGeomDataTemp = _vGeomData;
        if (value.HasMember("radius")) {
            orjson::LoadJsonValueByKey(value, "radius", vGeomDataTemp.x);
            vGeomDataTemp.x *= fUnitScale;
        }
        else if( value.HasMember("topRadius") ) { // due to a previous version of openrave
            RAVELOG_WARN("cylinder geometry uses 'topRadius', should change this.");
            orjson::LoadJsonValueByKey(value, "topRadius", vGeomDataTemp.x);
            vGeomDataTemp.x *= fUnitScale;
        }

        if (value.HasMember("height")) {
            orjson::LoadJsonValueByKey(value, "height", vGeomDataTemp.y);
            vGeomDataTemp.y *= fUnitScale;
        }
        if (vGeomDataTemp != _vGeomData) {
            _vGeomData = vGeomDataTemp;
            _meshcollision.Clear();
        }
        break;
    case GT_ConicalFrustum:
        vGeomDataTemp = _vGeomData;
        if (orjson::LoadJsonValueByKey(value, "topRadius", vGeomDataTemp.x)) {
            vGeomDataTemp.x *= fUnitScale;
        }
        if (orjson::LoadJsonValueByKey(value, "bottomRadius", vGeomDataTemp.y)) {
            vGeomDataTemp.y *= fUnitScale;
        }
        if (orjson::LoadJsonValueByKey(value, "height", vGeomDataTemp.z)) {
            vGeomDataTemp.z *= fUnitScale;
        }
        if (vGeomDataTemp != _vGeomData) {
            _vGeomData = vGeomDataTemp;
            _meshcollision.Clear();
        }
        break;
    case GT_Axial:
        if (value.HasMember("axial") && value["axial"].IsArray()) {
            const rapidjson::Value::ConstArray& rAxial = value["axial"].GetArray();
            if (rAxial.Size() < 4 || rAxial.Size() % 2 != 0) {
                // invalid axial array
                RAVELOG_DEBUG("there has to be at least 2 axial slices (4 elements) and even number of elements");
                break;
            }
            std::vector<AxialSlice> vAxialSlices;
            for (rapidjson::SizeType i = 0; i < rAxial.Size(); i += 2) {
                AxialSlice axialSlice;
                orjson::LoadJsonValue(rAxial[i], axialSlice.zOffset);
                orjson::LoadJsonValue(rAxial[i+1], axialSlice.radius);
                vAxialSlices.push_back(axialSlice);
            }
            if (vAxialSlices != _vAxialSlices) {
                _vAxialSlices = vAxialSlices;
                _meshcollision.Clear();
            }
        }
        break;

    case GT_Prism: {
        vGeomDataTemp = _vGeomData;
        if( value.HasMember("height") ) {
            orjson::LoadJsonValueByKey(value, "height", vGeomDataTemp.y);
            vGeomDataTemp.y *= fUnitScale;
        }
        std::vector<dReal> vCrossSection; // vertices of the cross-section, [x0, y0, x1, y1, ...]
        if( value.HasMember("crossSection") && value["crossSection"].IsArray() && value["crossSection"].Size() >= 2 && value["crossSection"].Size() % 2 == 0 ) {
            orjson::LoadJsonValue(value["crossSection"], vCrossSection);
        }
        else if ( vGeomDataTemp != _vGeomData && !_meshcollision.vertices.empty() ) {
            for( size_t ipoint = 0; ipoint < _meshcollision.vertices.size(); ipoint += 2 ) {
                vCrossSection.emplace_back(_meshcollision.vertices[ipoint].x);
                vCrossSection.emplace_back(_meshcollision.vertices[ipoint].y);
            }
        }
        if( !vCrossSection.empty() ) {
            const size_t nPoints = vCrossSection.size();
            _meshcollision.vertices.clear();
            _meshcollision.indices.clear();
            _meshcollision.vertices.reserve(nPoints);
            _meshcollision.indices.reserve(nPoints * 3);
            OpenRAVE::Vector vertex;
            for( size_t ipoint = 0; ipoint < nPoints; ipoint += 2 ) {
                vertex.x = vCrossSection[ipoint] * fUnitScale;
                vertex.y = vCrossSection[ipoint + 1] * fUnitScale;
                for( dReal z : { -vGeomDataTemp.y * 0.5, vGeomDataTemp.y * 0.5 } ) { // in meter
                    vertex.z = z * fUnitScale;
                    _meshcollision.vertices.push_back(vertex);
                }
                _meshcollision.indices.push_back(ipoint + 0);
                _meshcollision.indices.push_back(ipoint + 1);
                _meshcollision.indices.push_back((ipoint + 2) % nPoints);
                _meshcollision.indices.push_back(ipoint + 1);
                _meshcollision.indices.push_back((ipoint + 3) % nPoints);
                _meshcollision.indices.push_back((ipoint + 2) % nPoints);
            }
            _vGeomData = vGeomDataTemp;
            _modifiedFields |= KinBody::GeometryInfo::GIF_Mesh; // hard to check if mesh changed, need to do manual rapidjson operations for that
        }
        break;
    }
    case GT_Capsule:
        vGeomDataTemp = _vGeomData;
        if (value.HasMember("radius")) {
            orjson::LoadJsonValueByKey(value, "radius", vGeomDataTemp.x);
            vGeomDataTemp.x *= fUnitScale;
        }
        if (value.HasMember("height")) {
            orjson::LoadJsonValueByKey(value, "height", vGeomDataTemp.y);
            vGeomDataTemp.y *= fUnitScale;
        }
        if (vGeomDataTemp != _vGeomData) {
            _vGeomData = vGeomDataTemp;
            _meshcollision.Clear();
        }
        break;

    case GT_TriMesh:
        if (value.HasMember("mesh")) {
            orjson::LoadJsonValueByKey(value, "mesh", _meshcollision);
            FOREACH(itvertex, _meshcollision.vertices) {
                *itvertex *= fUnitScale;
            }
            _modifiedFields |= KinBody::GeometryInfo::GIF_Mesh; // hard to check if mesh changed, need to do manual rapidjson operations for that
        }
        break;
    case GT_CalibrationBoard:
        if (value.HasMember("halfExtents")) {
            orjson::LoadJsonValueByKey(value, "halfExtents", vGeomDataTemp);
            vGeomDataTemp *= fUnitScale;
            if (vGeomDataTemp != _vGeomData) {
                _vGeomData = vGeomDataTemp;
                _meshcollision.Clear();
            }
        }
        if (_calibrationBoardParameters.size() == 0) {
            _calibrationBoardParameters.push_back(CalibrationBoardParameters());
        }
        if (value.HasMember("calibrationBoardParameters")) {
            calibrationBoardParametersTemp.push_back(CalibrationBoardParameters(_calibrationBoardParameters[0]));
            orjson::LoadJsonValueByKey(value, "calibrationBoardParameters", calibrationBoardParametersTemp[0]);
            calibrationBoardParametersTemp[0].dotsDistanceX *= fUnitScale;
            calibrationBoardParametersTemp[0].dotsDistanceY *= fUnitScale;
            if (calibrationBoardParametersTemp[0] != _calibrationBoardParameters[0]) {
                _calibrationBoardParameters.clear();
                _calibrationBoardParameters.push_back(calibrationBoardParametersTemp[0]);
                _meshcollision.Clear();
            }
        }
        break;
    default:
        break;
    }

    orjson::LoadJsonValueByKey(value, "transparency", _fTransparency);
    orjson::LoadJsonValueByKey(value, "visible", _bVisible);
    orjson::LoadJsonValueByKey(value, "diffuseColor", _vDiffuseColor);
    orjson::LoadJsonValueByKey(value, "ambientColor", _vAmbientColor);
    orjson::LoadJsonValueByKey(value, "modifiable", _bModifiable);
}

inline void _UpdateExtrema(const Vector& v, Vector& vmin, Vector& vmax)
{
    if( vmin.x > v.x ) {
        vmin.x = v.x;
    }
    else if( vmax.x < v.x ) {
        vmax.x = v.x;
    }
    if( vmin.y > v.y ) {
        vmin.y = v.y;
    }
    else if( vmax.y < v.y ) {
        vmax.y = v.y;
    }
    if( vmin.z > v.z ) {
        vmin.z = v.z;
    }
    else if( vmax.z < v.z ) {
        vmax.z = v.z;
    }
}

AABB KinBody::GeometryInfo::ComputeAABB(const Transform& tGeometryWorld) const
{
    AABB ab;
    TransformMatrix tglobal = tGeometryWorld * _t;

    switch(_type) {
    case GT_None:
        ab.extents.x = 0;
        ab.extents.y = 0;
        ab.extents.z = 0;
        break;
    case GT_CalibrationBoard: // the tangible part of the board is basically the box
    case GT_Box: // origin of box is at the center
        ab.extents.x = RaveFabs(tglobal.m[0])*_vGeomData.x + RaveFabs(tglobal.m[1])*_vGeomData.y + RaveFabs(tglobal.m[2])*_vGeomData.z;
        ab.extents.y = RaveFabs(tglobal.m[4])*_vGeomData.x + RaveFabs(tglobal.m[5])*_vGeomData.y + RaveFabs(tglobal.m[6])*_vGeomData.z;
        ab.extents.z = RaveFabs(tglobal.m[8])*_vGeomData.x + RaveFabs(tglobal.m[9])*_vGeomData.y + RaveFabs(tglobal.m[10])*_vGeomData.z;
        ab.pos = tglobal.trans;
        break;
    case GT_Container: // origin of container is at the bottom
        ab.extents.x = 0.5*(RaveFabs(tglobal.m[0])*_vGeomData.x + RaveFabs(tglobal.m[1])*_vGeomData.y + RaveFabs(tglobal.m[2])*_vGeomData.z);
        ab.extents.y = 0.5*(RaveFabs(tglobal.m[4])*_vGeomData.x + RaveFabs(tglobal.m[5])*_vGeomData.y + RaveFabs(tglobal.m[6])*_vGeomData.z);
        ab.extents.z = 0.5*(RaveFabs(tglobal.m[8])*_vGeomData.x + RaveFabs(tglobal.m[9])*_vGeomData.y + RaveFabs(tglobal.m[10])*_vGeomData.z);
        ab.pos = tglobal.trans + Vector(tglobal.m[2], tglobal.m[6], tglobal.m[10])*(0.5*_vGeomData.z);

        if( _vGeomData4.x > 0 && _vGeomData4.y > 0 && _vGeomData4.z > 0 ) {
            // Container with bottom
            Vector vcontainerdir = Vector(tglobal.m[2], tglobal.m[6], tglobal.m[10]);
            ab.pos += vcontainerdir*_vGeomData4.z; // take into account the bottom of the container

            Vector vbottompos = tglobal.trans + vcontainerdir*(0.5*_vGeomData4.z);
            Vector vbottomextents;
            vbottomextents.x = 0.5*(RaveFabs(tglobal.m[0])*_vGeomData4.x + RaveFabs(tglobal.m[1])*_vGeomData4.y + RaveFabs(tglobal.m[2])*_vGeomData4.z);
            vbottomextents.y = 0.5*(RaveFabs(tglobal.m[4])*_vGeomData4.x + RaveFabs(tglobal.m[5])*_vGeomData4.y + RaveFabs(tglobal.m[6])*_vGeomData4.z);
            vbottomextents.z = 0.5*(RaveFabs(tglobal.m[8])*_vGeomData4.x + RaveFabs(tglobal.m[9])*_vGeomData4.y + RaveFabs(tglobal.m[10])*_vGeomData4.z);
            Vector vmin = ab.pos - ab.extents;
            Vector vmax = ab.pos + ab.extents;
            Vector vbottommin = vbottompos - vbottomextents;
            Vector vbottommax = vbottompos + vbottomextents;
            if( vmin.x > vbottommin.x ) {
                vmin.x = vbottommin.x;
            }
            if( vmin.y > vbottommin.y ) {
                vmin.y = vbottommin.y;
            }
            if( vmin.z > vbottommin.z ) {
                vmin.z = vbottommin.z;
            }
            if( vmax.x < vbottommax.x ) {
                vmax.x = vbottommax.x;
            }
            if( vmax.y < vbottommax.y ) {
                vmax.y = vbottommax.y;
            }
            if( vmax.z < vbottommax.z ) {
                vmax.z = vbottommax.z;
            }
            ab.pos = 0.5 * (vmin + vmax);
            ab.extents = vmax - ab.pos;
        }
        break;
    case GT_Sphere:
        ab.extents.x = ab.extents.y = ab.extents.z = _vGeomData[0];
        ab.pos = tglobal.trans;
        break;
    case GT_Cylinder:
        ab.extents.x = (dReal)0.5*RaveFabs(tglobal.m[2])*_vGeomData.y + RaveSqrt(max(dReal(0),1-tglobal.m[2]*tglobal.m[2]))*_vGeomData.x;
        ab.extents.y = (dReal)0.5*RaveFabs(tglobal.m[6])*_vGeomData.y + RaveSqrt(max(dReal(0),1-tglobal.m[6]*tglobal.m[6]))*_vGeomData.x;
        ab.extents.z = (dReal)0.5*RaveFabs(tglobal.m[10])*_vGeomData.y + RaveSqrt(max(dReal(0),1-tglobal.m[10]*tglobal.m[10]))*_vGeomData.x;
        ab.pos = tglobal.trans; //+(dReal)0.5*_vGeomData.y*Vector(tglobal.m[2],tglobal.m[6],tglobal.m[10]);
        break;
    case GT_Capsule:
        ab.extents.x = (dReal)0.5*RaveFabs(tglobal.m[2])*_vGeomData.y + _vGeomData.x;
        ab.extents.y = (dReal)0.5*RaveFabs(tglobal.m[6])*_vGeomData.y + _vGeomData.x;
        ab.extents.z = (dReal)0.5*RaveFabs(tglobal.m[10])*_vGeomData.y + _vGeomData.x;
        ab.pos = tglobal.trans;
        break;
    case GT_Cage: {
        // have to return the entire volume, even the inner region since a lot of code use the bounding box to compute cropping and other functions
        const Vector& vCageBaseExtents = _vGeomData;
        const Vector& vCageForceInnerFull = _vGeomData2;

        Vector vmin, vmax;
        vmin.x = -vCageBaseExtents.x;
        vmin.y = -vCageBaseExtents.y;
        vmax.x = vCageBaseExtents.x;
        vmax.y = vCageBaseExtents.y;
        vmax.z = vCageBaseExtents.z*2;
        for (size_t i = 0; i < _vSideWalls.size(); ++i) {
            const GeometryInfo::SideWall &s = _vSideWalls[i];
            TransformMatrix sidewallmat = s.transf;
            Vector vselocal = s.vExtents;
            Vector vsegeom;
            vsegeom.x = RaveFabs(sidewallmat.m[0])*vselocal.x + RaveFabs(sidewallmat.m[1])*vselocal.y + RaveFabs(sidewallmat.m[2])*vselocal.z;
            vsegeom.y = RaveFabs(sidewallmat.m[4])*vselocal.x + RaveFabs(sidewallmat.m[5])*vselocal.y + RaveFabs(sidewallmat.m[6])*vselocal.z;
            vsegeom.z = RaveFabs(sidewallmat.m[8])*vselocal.x + RaveFabs(sidewallmat.m[9])*vselocal.y + RaveFabs(sidewallmat.m[10])*vselocal.z;

            Vector vcenterpos = s.transf.trans + Vector(sidewallmat.m[2], sidewallmat.m[6], sidewallmat.m[10])*(vselocal.z);
            Vector vsidemin = vcenterpos - vsegeom;
            Vector vsidemax = vcenterpos + vsegeom;

            if( vmin.x > vsidemin.x ) {
                vmin.x = vsidemin.x;
            }
            if( vmin.y > vsidemin.y ) {
                vmin.y = vsidemin.y;
            }
            if( vmin.z > vsidemin.z ) {
                vmin.z = vsidemin.z;
            }
            if( vmax.x < vsidemax.x ) {
                vmax.x = vsidemax.x;
            }
            if( vmax.y < vsidemax.y ) {
                vmax.y = vsidemax.y;
            }
            if( vmax.z < vsidemax.z ) {
                vmax.z = vsidemax.z;
            }
        }

        if( vCageForceInnerFull.x > 0 ) {
            if( vmin.x > -0.5*vCageForceInnerFull.x ) {
                vmin.x = -0.5*vCageForceInnerFull.x;
            }
            if( vmax.x < 0.5*vCageForceInnerFull.x ) {
                vmax.x = 0.5*vCageForceInnerFull.x;
            }
        }
        if( vCageForceInnerFull.y > 0 ) {
            if( vmin.y > -0.5*vCageForceInnerFull.y ) {
                vmin.y = -0.5*vCageForceInnerFull.y;
            }
            if( vmax.y < 0.5*vCageForceInnerFull.y ) {
                vmax.y = 0.5*vCageForceInnerFull.y;
            }
        }
        if( vCageForceInnerFull.z > 0 ) {
            if( vmax.z < vCageBaseExtents.z*2+vCageForceInnerFull.z ) {
                vmax.z = vCageBaseExtents.z*2+vCageForceInnerFull.z;
            }
        }

        // now that vmin and vmax are in geom space, transform them
        Vector vgeomextents = 0.5*(vmax-vmin);

        ab.extents.x = RaveFabs(tglobal.m[0])*vgeomextents.x + RaveFabs(tglobal.m[1])*vgeomextents.y + RaveFabs(tglobal.m[2])*vgeomextents.z;
        ab.extents.y = RaveFabs(tglobal.m[4])*vgeomextents.x + RaveFabs(tglobal.m[5])*vgeomextents.y + RaveFabs(tglobal.m[6])*vgeomextents.z;
        ab.extents.z = RaveFabs(tglobal.m[8])*vgeomextents.x + RaveFabs(tglobal.m[9])*vgeomextents.y + RaveFabs(tglobal.m[10])*vgeomextents.z;
        ab.pos = tglobal*(0.5*(vmin+vmax));
        break;

    }
    case GT_Prism:
    case GT_ConicalFrustum:
    case GT_Axial:
    case GT_TriMesh: {
        // Cage: init collision mesh?
        // just use _meshcollision
        if( _meshcollision.vertices.size() > 0) {
            // no need to check rot(2,2), guaranteed to be 1 if rot(0,0) and rot(1,1) are both 1
            const bool bRotationIsIdentity = RaveFabs(tglobal.rot(0,0) - 1.0) <= g_fEpsilon && RaveFabs(tglobal.rot(1,1) - 1.0) <= g_fEpsilon;
            Vector vmin, vmax;
            // if no rotation (identity), skip rotation of vertices
            if (bRotationIsIdentity) {
                vmin = vmax = _meshcollision.vertices.at(0);
                for (const Vector& vertex : _meshcollision.vertices) {
                    _UpdateExtrema(vertex, vmin, vmax);
                }
                ab.pos = (dReal)0.5*(vmax+vmin) + tglobal.trans;
            }
            else {
                vmin = vmax = tglobal*_meshcollision.vertices.at(0);
                for (const Vector& vertex : _meshcollision.vertices) {
                    _UpdateExtrema(tglobal * vertex, vmin, vmax);
                }
                ab.pos = (dReal)0.5*(vmax+vmin);
            }
            ab.extents = (dReal)0.5*(vmax-vmin);
        }
        else {
            ab.pos = tglobal.trans;
        }
        break;
    }
    default:
        throw OPENRAVE_EXCEPTION_FORMAT(_("unknown geometry type %d"), _type, ORE_InvalidArguments);
    }

    return ab;
}

uint8_t KinBody::GeometryInfo::GetSideWallExists() const
{
    uint8_t mask = 0;
    for (size_t i = 0; i < _vSideWalls.size(); ++i) {
        mask |= 1 << _vSideWalls[i].type;
    }
    return mask;
}

KinBody::Geometry::Geometry(KinBody::LinkPtr parent, const KinBody::GeometryInfo& info) : _parent(parent), _info(info)
{
}

bool KinBody::Geometry::InitCollisionMesh(float fTessellation)
{
    return _info.InitCollisionMesh(fTessellation);
}

bool KinBody::Geometry::ComputeInnerEmptyVolume(Transform& tInnerEmptyVolume, Vector& abInnerEmptyExtents) const
{
    return _info.ComputeInnerEmptyVolume(tInnerEmptyVolume, abInnerEmptyExtents);
}

AABB KinBody::Geometry::ComputeAABB(const Transform& t) const
{
    return _info.ComputeAABB(t);
}

void KinBody::Geometry::serialize(std::ostream& o, int options) const
{
    SerializeRound(o,_info._t);
    o << (int)_info._type << " ";
    SerializeRound3(o,_info._vRenderScale);
    if( _info._type == GT_TriMesh ) {
        _info._meshcollision.serialize(o,options);
    }
    else {
        SerializeRound3(o,_info._vGeomData);
        if( _info._type == GT_Cage ) {
            SerializeRound3(o,_info._vGeomData2);
            for (size_t iwall = 0; iwall < _info._vSideWalls.size(); ++iwall) {
                const GeometryInfo::SideWall &s = _info._vSideWalls[iwall];
                SerializeRound(o,s.transf);
                SerializeRound3(o,s.vExtents);
                o << (uint32_t)s.type;
            }
        }
        else if( _info._type == GT_Container ) {
            SerializeRound3(o,_info._vGeomData2);
            SerializeRound3(o,_info._vGeomData3);
            SerializeRound3(o,_info._vGeomData4);
        }
    }
}

void KinBody::Geometry::SetCollisionMesh(const TriMesh& mesh)
{
    OPENRAVE_ASSERT_FORMAT0(_info._bModifiable, "geometry cannot be modified", ORE_Failed);
    LinkPtr parent(_parent);
    _info._meshcollision = mesh;
    // _info._modifiedFields; change??
    parent->_Update();
}

bool KinBody::Geometry::SetVisible(bool visible)
{
    if( _info._bVisible != visible ) {
        _info._bVisible = visible;
        LinkPtr parent(_parent);
        parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
        return true;
    }
    return false;
}

void KinBody::Geometry::SetTransparency(float f)
{
    LinkPtr parent(_parent);
    _info._fTransparency = f;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
}

void KinBody::Geometry::SetDiffuseColor(const RaveVector<float>& color)
{
    LinkPtr parent(_parent);
    _info._vDiffuseColor = color;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
}

void KinBody::Geometry::SetAmbientColor(const RaveVector<float>& color)
{
    LinkPtr parent(_parent);
    _info._vAmbientColor = color;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
}

void KinBody::Geometry::SetNegativeCropContainerMargins(const Vector& negativeCropContainerMargins)
{
    LinkPtr parent(_parent);
    _info._vNegativeCropContainerMargins = negativeCropContainerMargins;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
}

void KinBody::Geometry::SetPositiveCropContainerMargins(const Vector& positiveCropContainerMargins)
{
    LinkPtr parent(_parent);
    _info._vPositiveCropContainerMargins = positiveCropContainerMargins;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
}

void KinBody::Geometry::SetNegativeCropContainerEmptyMargins(const Vector& negativeCropContainerEmptyMargins)
{
    LinkPtr parent(_parent);
    _info._vNegativeCropContainerEmptyMargins = negativeCropContainerEmptyMargins;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkDraw);
}

void KinBody::Geometry::SetPositiveCropContainerEmptyMargins(const Vector& positiveCropContainerEmptyMargins)
{
    LinkPtr parent(_parent);
    _info._vPositiveCropContainerEmptyMargins = positiveCropContainerEmptyMargins;
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

bool KinBody::Geometry::ValidateContactNormal(const Vector& _position, Vector& _normal) const
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

void KinBody::Geometry::SetRenderFilename(const std::string& renderfilename)
{
    LinkPtr parent(_parent);
    _info._filenamerender = renderfilename;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);
}

void KinBody::Geometry::SetName(const std::string& name)
{
    LinkPtr parent(_parent);
    _info._name = name;
    parent->GetParent()->_PostprocessChangedParameters(Prop_LinkGeometry);
}

void KinBody::Geometry::UpdateInfo()
{
}

void KinBody::Geometry::ExtractInfo(KinBody::GeometryInfo& info) const
{
    info = _info;
    info._modifiedFields = 0;
}

UpdateFromInfoResult KinBody::Geometry::UpdateFromInfo(const KinBody::GeometryInfo& info)
{
    if(!info._id.empty() && _info._id != info._id) {
        throw OPENRAVE_EXCEPTION_FORMAT("Do not allow updating link '%s' geometry '%s' (id='%s') with a different info id='%s'", _parent.lock()->GetName()%GetName()%_info._id%info._id, ORE_Assert);
    }
    UpdateFromInfoResult updateFromInfoResult = UFIR_NoChange;

    if (GetName() != info._name) {
        SetName(info._name);
        RAVELOG_VERBOSE_FORMAT("geometry %s name changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    if (GetType() != info._type) {
        RAVELOG_VERBOSE_FORMAT("geometry %s type changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    if( info.IsModifiedField(KinBody::GeometryInfo::GIF_Transform) && GetTransform().CompareTransform(info._t, g_fEpsilon) ) {
        RAVELOG_VERBOSE_FORMAT("geometry %s transform changed", _info._id);
        return UFIR_RequireReinitialize;
    }

    if (GetType() == GT_Box) {
        if (GetBoxExtents() != info._vGeomData) {
            RAVELOG_VERBOSE_FORMAT("geometry %s box extents changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_Container) {
        if (GetContainerOuterExtents() != info._vGeomData || GetContainerInnerExtents() != info._vGeomData2 || GetContainerBottomCross() != info._vGeomData3 || GetContainerBottom() != info._vGeomData4) {
            RAVELOG_VERBOSE_FORMAT("geometry %s container extents changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_Cage) {
        if (GetCageBaseExtents() != info._vGeomData || _info._vGeomData2 != info._vGeomData2 || _info._vSideWalls != info._vSideWalls) {
            RAVELOG_VERBOSE_FORMAT("geometry %s cage changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_Sphere) {
        if (GetSphereRadius() != info._vGeomData.x) {
            RAVELOG_VERBOSE_FORMAT("geometry %s sphere changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_Cylinder) {
        if (GetCylinderRadius() != info._vGeomData.x || GetCylinderHeight() != info._vGeomData.y) {
            RAVELOG_VERBOSE_FORMAT("geometry %s cylinder changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_ConicalFrustum) {
        if (GetConicalFrustumTopRadius() != info.GetConicalFrustumTopRadius() ||
            GetConicalFrustumBottomRadius() != info.GetConicalFrustumBottomRadius() ||
            GetConicalFrustumHeight() != info.GetConicalFrustumHeight()) {
            RAVELOG_VERBOSE_FORMAT("geometry %s conical frustum changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_Axial) {
        if (_info._vAxialSlices != info._vAxialSlices) {
            RAVELOG_VERBOSE_FORMAT("geometry %s axial changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_Prism) {
        if( info.IsModifiedField(KinBody::GeometryInfo::GIF_Mesh) && info._meshcollision != _info._meshcollision ) {
            RAVELOG_VERBOSE_FORMAT("geometry %s prism changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_Capsule) {
        if( GetCapsuleRadius() != info._vGeomData.x || GetCapsuleHeight() != info._vGeomData.y ) {
            RAVELOG_VERBOSE_FORMAT("geometry %s capsule changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }
    else if (GetType() == GT_TriMesh) {
        if( info.IsModifiedField(KinBody::GeometryInfo::GIF_Mesh) && info._meshcollision != _info._meshcollision ) {
            RAVELOG_VERBOSE_FORMAT("geometry %s trimesh changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    } else if (GetType() == GT_CalibrationBoard) {
        if (GetBoxExtents() != info._vGeomData || info._calibrationBoardParameters != _info._calibrationBoardParameters) {
            RAVELOG_VERBOSE_FORMAT("geometry %s calibrationboard changed", _info._id);
            return UFIR_RequireReinitialize;
        }
    }

    // transparency
    if (GetTransparency() != info._fTransparency) {
        SetTransparency(info._fTransparency);
        RAVELOG_VERBOSE_FORMAT("geometry %s transparency changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // visible
    if (IsVisible() != info._bVisible) {
        SetVisible(info._bVisible);
        RAVELOG_VERBOSE_FORMAT("geometry %s visible changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // diffuseColor
    if (GetDiffuseColor() != info._vDiffuseColor) {
        SetDiffuseColor(info._vDiffuseColor);
        RAVELOG_VERBOSE_FORMAT("geometry %s diffuse color changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // ambientColor
    if (GetAmbientColor() != info._vAmbientColor) {
        SetAmbientColor(info._vAmbientColor);
        RAVELOG_VERBOSE_FORMAT("geometry %s ambient color changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // modifiable
    if (IsModifiable() != info._bModifiable) {
        _info._bModifiable = info._bModifiable;
        RAVELOG_VERBOSE_FORMAT("geometry %s modifiable changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // negativeCropContainerMargins
    if(GetNegativeCropContainerMargins() != info._vNegativeCropContainerMargins) {
        SetNegativeCropContainerMargins(info._vNegativeCropContainerMargins);
        RAVELOG_VERBOSE_FORMAT("geometry %s negativeCropContainerMargins changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // positiveCropContainerMargins
    if(GetPositiveCropContainerMargins() != info._vPositiveCropContainerMargins) {
        SetPositiveCropContainerMargins(info._vPositiveCropContainerMargins);
        RAVELOG_VERBOSE_FORMAT("geometry %s positiveCropContainerMargins changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // negativeCropContainerEmptyMargins
    if(GetNegativeCropContainerEmptyMargins() != info._vNegativeCropContainerEmptyMargins) {
        SetNegativeCropContainerEmptyMargins(info._vNegativeCropContainerEmptyMargins);
        RAVELOG_VERBOSE_FORMAT("geometry %s negativeCropContainerEmptyMargins changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    // positiveCropContainerEmptyMargins
    if(GetPositiveCropContainerEmptyMargins() != info._vPositiveCropContainerEmptyMargins) {
        SetPositiveCropContainerEmptyMargins(info._vPositiveCropContainerEmptyMargins);
        RAVELOG_VERBOSE_FORMAT("geometry %s positiveCropContainerEmptyMargins changed", _info._id);
        updateFromInfoResult = UFIR_Success;
    }

    return updateFromInfoResult;
}

}
