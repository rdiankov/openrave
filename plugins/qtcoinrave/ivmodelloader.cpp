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
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLGeometry.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLVertexShape.h>
#include <Inventor/VRMLnodes/SoVRMLVertexLine.h>
#include <Inventor/VRMLnodes/SoVRMLVertexPoint.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLine.h>
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
        TriMesh trimesh;
        bool bSuccess = false;
        // SoDB::readAll memory leaks!
        SoSeparator* psep = SoDB::readAll(&mySceneInput);
        if( !!psep ) {
            // try to extract a material
            SoSearchAction search;
            search.setFind(SoSearchAction::TYPE);
            search.setInterest(SoSearchAction::ALL);
            search.setType(SoMaterial::getClassTypeId());
            psep->ref();
            search.apply(psep);
            for(int i = 0; i < search.getPaths().getLength(); ++i) {
                SoPath* path = search.getPaths()[i];
                if( !path || path->getTail()->getTypeId() != SoMaterial::getClassTypeId() ) {
                    continue;
                }
                SoMaterial* pmtrl = (SoMaterial*)path->getTail();
                if( !pmtrl ) {
                    continue;
                }
                if( !!pmtrl->diffuseColor.getValues(0) ) {
                    diffuseColor.x = pmtrl->diffuseColor.getValues(0)->getValue()[0];
                    diffuseColor.y = pmtrl->diffuseColor.getValues(0)->getValue()[1];
                    diffuseColor.z = pmtrl->diffuseColor.getValues(0)->getValue()[2];
                }
                else if( !!pmtrl->emissiveColor.getValues(0) ) {
                    diffuseColor.x = pmtrl->emissiveColor.getValues(0)->getValue()[0];
                    diffuseColor.y = pmtrl->emissiveColor.getValues(0)->getValue()[1];
                    diffuseColor.z = pmtrl->emissiveColor.getValues(0)->getValue()[2];
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
            {

//                SoCallbackAction materialaction;
//                materialaction.apply(psep);
//                SbColor ambient, diffuse, specular, emission;
//                float shininess, transparency;
//                materialaction.getMaterial(ambient,diffuse,specular,emission,shininess,transparency);


//                // add the callbacks for all nodes
//                triAction.addTriangleCallback(SoVRMLMaterial::getClassTypeId(), _Coin3dDiffuseColor, &diffuseColor);
//                pnode->ref();
//                triAction.apply(pnode);
//
//
                // have to look for vrml materials, which is a different type
                SoSearchAction search;
                //search.setFind(SoSearchAction::TYPE);
                search.setInterest(SoSearchAction::ALL);
                search.setType(SoVRMLAppearance::getClassTypeId());
                search.apply(psep);
                for(int i = 0; i < search.getPaths().getLength(); ++i) {
                    SoPath* path = search.getPaths()[i];
                    if( !path || !path->getTail() ) {
                        RAVELOG_INFO("no path");
                        continue;
                    }
                    SoVRMLAppearance* pappearance = NULL;
                    SoVRMLMaterial* pmtrl = NULL;
                    SoVRMLShape* pshape = NULL;
                    if( path->getTail()->getTypeId() == SoVRMLShape::getClassTypeId() ) {
                        pshape = (SoVRMLShape*)path->getTail();
                    }
                    else if( path->getTail()->getTypeId() == SoVRMLAppearance::getClassTypeId() ) {
                        pappearance = (SoVRMLAppearance*)path->getTail();
                    }
                    else if( path->getTail()->getTypeId() == SoVRMLMaterial::getClassTypeId() ) {
                        pmtrl = (SoVRMLMaterial*)path->getTail();
                    }
                    else {
                        RAVELOG_INFO("unknown vrml type: %s\n",path->getTail()->getTypeId().getName().getString());
                        continue;
                    }

                    if( !!pshape ) {
                        // check if the shape geometry has faces or not.
                        if( !pshape->geometry.getValue() || pshape->geometry.getValue()->isOfType(SoVRMLVertexLine::getClassTypeId()) || pshape->geometry.getValue()->isOfType(SoVRMLVertexPoint::getClassTypeId()) || pshape->geometry.getValue()->isOfType(SoVRMLIndexedLineSet::getClassTypeId()) || pshape->geometry.getValue()->isOfType(SoVRMLIndexedLine::getClassTypeId())) {
                            continue;
                        }
                        pappearance = (SoVRMLAppearance*)pshape->appearance.getValue();
                    }
                    if( !!pappearance ) {
                        pmtrl = (SoVRMLMaterial*)pappearance->material.getValue();
                    }
                    if( !pmtrl ) {
                        continue;
                    }

                    diffuseColor.x = pmtrl->diffuseColor.getValue()[0];
                    diffuseColor.y = pmtrl->diffuseColor.getValue()[1];
                    diffuseColor.z = pmtrl->diffuseColor.getValue()[2];
                    ambientColor.x = pmtrl->ambientIntensity.getValue();
                    ambientColor.y = pmtrl->ambientIntensity.getValue();
                    ambientColor.z = pmtrl->ambientIntensity.getValue();
                    ftransparency = pmtrl->transparency.getValue();
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
        TriMesh* ptri = (TriMesh*)data;
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

    static void _Coin3dCreateTriMeshData(SoNode* pnode, TriMesh& tri)
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
