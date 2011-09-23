// -*- coding: utf-8 -*-
// Copyright (C) 2011 Rosen Diankov (rosen.diankov@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
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
#include "qtcoin.h"
#include <boost/algorithm/string.hpp>

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/SbMatrix.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoShape.h>

class IvModelLoader : public ModuleBase
{
public:
    IvModelLoader(EnvironmentBasePtr penv) : ModuleBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nProvides a simple model loader functionality returning a triangle mesh using the Coin3D library. Depending on the version, Coin3D might be licensed under GPL.";
        RegisterCommand("LoadModel",boost::bind(&IvModelLoader::_LoadModel,this,_1,_2),"Returns the triangle mesh given a model filename");
    }

    int main(const string& cmd)
    {
        return 0;
    }

    bool _LoadModel(ostream& sout, istream& sinput)
    {
        string filename;
        if (!getline(sinput, filename) ) {
            RAVELOG_WARN("failed to get filename\n");
            return false;
        }
        boost::trim(filename);

        boost::mutex::scoped_lock lock(g_mutexsoqt);
        SoDBWriteLock dblock;
        if(!SoDB::isInitialized()) {
            SoDB::init();
        }
        // have to lock coin3d, or otherwise state gets corrupted
        SoInput mySceneInput;
        if (!mySceneInput.openFile(filename.c_str())) {
            RAVELOG_WARN(str(boost::format("Failed to open '%s' for KinBody:TriMesh\n")%filename));
            return false;
        }

        Vector diffuseColor(1,1,1,1), ambientColor;
        dReal ftransparency=0;
        KinBody::Link::TRIMESH trimesh;
        bool bSuccess = false;
        // SoDB::readAll memory leaks!
        SoSeparator* psep = SoDB::readAll(&mySceneInput);
        if( !!psep ) {
            // try to extract a material
            SoSearchAction search;
            search.setInterest(SoSearchAction::ALL);
            search.setType(SoMaterial::getClassTypeId());
            psep->ref();
            search.apply(psep);
            for(int i = 0; i < search.getPaths().getLength(); ++i) {
                SoPath* path = search.getPaths()[i];
                SoMaterial* pmtrl = (SoMaterial*)path->getTail();
                if( !pmtrl ) {
                    continue;
                }
                if( !!pmtrl->diffuseColor.getValues(0) ) {
                    diffuseColor.x = pmtrl->diffuseColor.getValues(0)->getValue()[0];
                    diffuseColor.y = pmtrl->diffuseColor.getValues(0)->getValue()[1];
                    diffuseColor.z = pmtrl->diffuseColor.getValues(0)->getValue()[2];
                }
                if( !!pmtrl->ambientColor.getValues(0) ) {
                    ambientColor.x = pmtrl->ambientColor.getValues(0)->getValue()[0];
                    ambientColor.y = pmtrl->ambientColor.getValues(0)->getValue()[1];
                    ambientColor.z = pmtrl->ambientColor.getValues(0)->getValue()[2];

                }
                if( !!pmtrl->transparency.getValues(0) ) {
                    ftransparency = pmtrl->transparency.getValues(0)[0];
                }
            }
            _Coin3dCreateTriMeshData(psep, trimesh);
            psep->unref();
            bSuccess = true;
        }

        mySceneInput.closeFile();
        sout << trimesh << diffuseColor << ambientColor << ftransparency;
        return bSuccess;
    }

    // Coin specific routines
    static SbMatrix& GetModelMatrix() {
        static SbMatrix m;
        return m;
    }
    static void _Coin3dTriangulateCB(void *data, SoCallbackAction *action, const SoPrimitiveVertex *vertex1, const SoPrimitiveVertex *vertex2, const SoPrimitiveVertex *vertex3)
    {
        KinBody::Link::TRIMESH* ptri = (KinBody::Link::TRIMESH*)data;
        GetModelMatrix() = action->getModelMatrix();

        // set the vertices (SCALED)
        //    ptri->vertices.push_back(Vector(&vertex1->getPoint()[0]));
        //    ptri->vertices.push_back(Vector(&vertex2->getPoint()[0]));
        //    ptri->vertices.push_back(Vector(&vertex3->getPoint()[0]));
        SbVec3f v;
        GetModelMatrix().multVecMatrix(vertex1->getPoint(), v);
        ptri->vertices.push_back(Vector(&v[0]));

        GetModelMatrix().multVecMatrix(vertex2->getPoint(), v);
        ptri->vertices.push_back(Vector(&v[0]));

        GetModelMatrix().multVecMatrix(vertex3->getPoint(), v);
        ptri->vertices.push_back(Vector(&v[0]));
    }

    static void _Coin3dCreateTriMeshData(SoNode* pnode, KinBody::Link::TRIMESH& tri)
    {
        tri.vertices.resize(0);
        tri.vertices.reserve(256);

        // create the collision model and triangulate
        SoCallbackAction triAction;

        // add the callbacks for all nodes
        triAction.addTriangleCallback(SoShape::getClassTypeId(), _Coin3dTriangulateCB, &tri);
        pnode->ref();
        triAction.apply(pnode);
        //pnode->unref();
        tri.indices.resize(tri.vertices.size());
        for(size_t i = 0; i < tri.vertices.size(); ++i) {
            tri.indices[i] = i;
        }
    }
};

ModuleBasePtr CreateIvModelLoader(EnvironmentBasePtr penv) {
    return ModuleBasePtr(new IvModelLoader(penv));
}
