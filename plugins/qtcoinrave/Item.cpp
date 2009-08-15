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
  \file   Item.cpp
  \brief  Abstract base class for an Item
 -------------------------------------------------------------------- */
#include "qtcoin.h"

#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoMaterialBinding.h>

Item::Item(QtCoinViewer* viewer) : _viewer(viewer)
{
    _pname = NULL;

    // set up the Inventor nodes
    _ivXform = new SoTransform;
    _ivRoot = new SoSeparator;
    _ivGeom = new SoSwitch(2);
    _ivGeom->whichChild.setValue(SO_SWITCH_ALL); 

    _ivRoot->ref();
    _ivRoot->addChild(_ivXform);
    _ivRoot->addChild(_ivGeom);

    _ivTransparency = new SoTransparencyType();
    _ivTransparency->value = SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND;
    _ivGeom->insertChild(_ivTransparency, 0);

    _viewer->GetRoot()->addChild(_ivRoot);
}

Item::~Item()
{
    delete[] _pname;
    if( _ivRoot != NULL ) {
        _viewer->GetRoot()->removeChild(_ivRoot);
        _ivRoot->unref();
    }
    
}

//! set the item name
void Item::SetName(const wchar_t* pNewName)
{
    delete[] _pname; _pname = _ravewcsdup(pNewName);
}

/// returns true if the given node is in the inventor hierarchy
bool Item::ContainsIvNode(SoNode *pNode)
{
  SoSearchAction search;
  search.setNode(pNode);
  search.apply(_ivGeom);

  if (search.getPath())
    return true;

  return false;
}

///! returns true if the path the given node passes through
///          the geometry root of this item (i.e. the node is part
///          of the geometry of this item).
bool Item::ContainsIvNode(SoPath *pNodePath)
{
  //return (pNodePath->containsNode(_ivGeom));    
  return (ContainsIvNode(pNodePath->getTail()));
}

/// Set the visibility of the geometry (ON = true).
void Item::SetGeomVisibility(bool bFlag)
{
  _ivGeom->whichChild.setValue(bFlag ? SO_SWITCH_ALL : SO_SWITCH_NONE); 
}

/// Set the pick style of the node to be unpickable
void Item::SetUnpickable()
{
  SoPickStyle* pickStyle = new SoPickStyle();
  pickStyle->style = SoPickStyle::UNPICKABLE;

  _ivGeom->insertChild(pickStyle, 0);
}

/// KinBodyItem class
KinBodyItem::KinBodyItem(QtCoinViewer* viewer, KinBody* pchain, ViewGeometry viewmode) : Item(viewer), _viewmode(viewmode)
{
    assert( pchain != NULL );
    _pchain = pchain;
    bGrabbed = false;
    bEnabled = pchain->IsEnabled();
    _userdata = 0;

    networkid = pchain->GetNetworkId();

    vector<KinBody::Link*>::const_iterator it;

    FORIT(it, _pchain->GetLinks()) {
        LINK lnk;
        lnk.first = new SoSeparator();
        lnk.second = new SoTransform();

        RaveTransform<float> tbody = (*it)->GetTransform();
        lnk.second->rotation.setValue(tbody.rot.y, tbody.rot.z, tbody.rot.w, tbody.rot.x);
        lnk.second->translation.setValue(tbody.trans.x, tbody.trans.y, tbody.trans.z);
                    
        lnk.first->addChild(lnk.second);
        _ivGeom->addChild(lnk.first);
        
        _veclinks.push_back(lnk);

        FOREACHC(itgeom, (*it)->GetGeometries()) {
            if( !itgeom->IsDraw() && _viewmode == VG_RenderOnly )
                continue;
            
            SoSeparator* psep = NULL;
            SoTransform* ptrans = new SoTransform();
            Transform tgeom = itgeom->GetTransform();
            ptrans->rotation.setValue(tgeom.rot.y, tgeom.rot.z, tgeom.rot.w, tgeom.rot.x);
            ptrans->translation.setValue(tgeom.trans.x, tgeom.trans.y, tgeom.trans.z);
            
            // open
            bool bSucceeded = false;
            if( _viewmode == VG_RenderOnly || _viewmode == VG_RenderCollision ) {
                SoInput mySceneInput;
                if( itgeom->GetRenderFilename().size() > 0 && mySceneInput.openFile(itgeom->GetRenderFilename().c_str())) {
                    psep = SoDB::readAll(&mySceneInput);

                    SoScale* s = new SoScale();
                    s->scaleFactor.setValue(itgeom->GetRenderScale().x, itgeom->GetRenderScale().y, itgeom->GetRenderScale().z);
                    psep->insertChild(s, 0);

                    if( itgeom->GetTransparency() > 0 ) {
                        // set a diffuse color
                        SoSearchAction search;
                        search.setType(SoMaterial::getClassTypeId());
                        search.setInterest(SoSearchAction::ALL);
                        psep->ref();
                        search.apply(psep);
                        for(int i = 0; i < search.getPaths().getLength(); ++i) {
                            SoPath* path = search.getPaths()[i];
                            SoMaterial* pmtrl = (SoMaterial*)path->getTail();
                            pmtrl->transparency = itgeom->GetTransparency();
                        }
                    }

                    mySceneInput.closeFile();
                    bSucceeded = true;
                }
            }

            if( !bSucceeded || _viewmode == VG_RenderCollision ) {

                // create custom
                if( psep == NULL )
                    psep = new SoSeparator();
                else {
                    SoSeparator* pparentsep = new SoSeparator();
                    pparentsep->addChild(psep);
                    psep = pparentsep;
                }

                // set a diffuse color
                SoMaterial* mtrl = new SoMaterial;
                mtrl->diffuseColor = SbColor(itgeom->GetDiffuseColor());
                mtrl->ambientColor = SbColor(itgeom->GetAmbientColor());
                mtrl->setOverride(true);
                mtrl->transparency = itgeom->GetTransparency();
                if( _viewmode == VG_RenderCollision && bSucceeded ) {
                    mtrl->transparency = 0.5f;
                    mtrl->diffuseColor = SbColor(0.6f,0.6f,1.0f);
                    mtrl->ambientColor = SbColor(0.4f,0.4f,1.0f);
                }
                psep->addChild(mtrl);

                // by default render only one side of objects 
                // don't change since sensors like cameras are usually inside objects, but they
                // need to see outside of the world
                SoShapeHints* phints = new SoShapeHints();
                phints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
                phints->shapeType = SoShapeHints::SOLID;
                phints->faceType = SoShapeHints::CONVEX;
                phints->creaseAngle = 0;
                psep->addChild(phints);

                switch(itgeom->GetType()) {
                case KinBody::Link::GEOMPROPERTIES::GeomSphere: {
                    SoSphere* s = new SoSphere();
                    s->radius = itgeom->GetSphereRadius();
                    psep->addChild(s);
                    break;
                }
                case KinBody::Link::GEOMPROPERTIES::GeomBox: {
                    Vector v;
                    SoCube* c = new SoCube();
                    c->width = itgeom->GetBoxExtents().x*2.0f;
                    c->height = itgeom->GetBoxExtents().y*2.0f;
                    c->depth = itgeom->GetBoxExtents().z*2.0f;
                    psep->addChild(c);
                    break;
                }
                case KinBody::Link::GEOMPROPERTIES::GeomCylinder: {
                    // make SoCylinder point towards z, not y
                    SbMatrix m;
                    SbRotation(SbVec3f(1,0,0),M_PI/2).getValue(m);
                    ptrans->multLeft(m);
                        
                    SoCylinder* cy = new SoCylinder();
                    cy->radius = itgeom->GetCylinderRadius();
                    cy->height = itgeom->GetCylinderHeight();
                    cy->parts = SoCylinder::ALL;

                    psep->addChild(cy);
                    break;
                }
                case KinBody::Link::GEOMPROPERTIES::GeomTrimesh: {

                    // set to render for both faces
                    phints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;
                    
                    SoMaterialBinding* pbinding = new SoMaterialBinding();
                    pbinding->value = SoMaterialBinding::OVERALL;
                    psep->addChild(pbinding);

                    if( itgeom->GetTransparency() > 0 ) {
                        SoTransparencyType* ptype = new SoTransparencyType();
                        ptype->value = SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND;
                        psep->addChild(ptype);
                    }

                    const KinBody::Link::TRIMESH& mesh = itgeom->GetCollisionMesh();
                    
                    SoCoordinate3* vprop = new SoCoordinate3();
                    // this makes it crash!
                    //vprop->point.set1Value(mesh.indices.size()-1,SbVec3f(0,0,0)); // resize
                    int i = 0;
                    FOREACHC(itind, mesh.indices) {
                        RaveVector<float> v = mesh.vertices[*itind];
                        vprop->point.set1Value(i++, SbVec3f(v.x,v.y,v.z));
                    }

                    psep->addChild(vprop);

                    SoFaceSet* faceset = new SoFaceSet();
                    // this makes it crash!
                    //faceset->numVertices.set1Value(mesh.indices.size()/3-1,3);
                    for(size_t i = 0; i < mesh.indices.size()/3; ++i)
                        faceset->numVertices.set1Value(i,3);
                    psep->addChild(faceset);
                    
                    break;
                }
                default:
                    RAVEPRINT(L"No render data for link %S:%S, geom type = %d\n", _pchain->GetName(), (*it)->GetName(), itgeom->GetType());
                    break;
                }
            }

            if( psep != NULL ) {
                psep->insertChild(ptrans, 0);
                lnk.first->addChild(psep);
            }
        }
    }
}

KinBodyItem::~KinBodyItem()
{
    //delete _pchain; // pointer doesn't belong to gui
    _veclinks.clear();
}

bool KinBodyItem::UpdateFromIv()
{
    if( _pchain == NULL )
        return false;

    vector<Transform> vtrans(_veclinks.size());
    Transform tglob = GetRaveTransform(_ivXform);
    
    vector<Transform>::iterator ittrans = vtrans.begin();
    FOREACH(it, _veclinks) {
        // propagate the rotations down and reset the centroid
        *ittrans = GetRaveTransform(it->second);
        *ittrans = tglob * *ittrans;
        ++ittrans;
    }

    _pchain->GetEnv()->LockPhysics(true);
    _pchain->SetBodyTransformations(vtrans);
    _pchain->GetEnv()->LockPhysics(false);
    return true;
}

bool KinBodyItem::UpdateFromModel()
{
    if( _pchain == NULL )
        return false;

    vector<Transform> vtrans;
    vector<RaveTransform<dReal> > vguitrans;

    _pchain->GetEnv()->LockPhysics(true);
   
    // make sure the body is still present!
    if( _pchain->GetEnv()->GetBodyFromNetworkId(networkid) == _pchain ) {
        _pchain->GetBodyTransformations(vtrans);
    }
    else {
        _pchain = NULL;
    }
    _pchain->GetEnv()->LockPhysics(false);

    vguitrans.resize(vtrans.size());
    for(size_t i = 0; i < vtrans.size(); ++i)
        vguitrans[i] = vtrans[i];
    return UpdateFromModel(vguitrans);    
}

bool KinBodyItem::UpdateFromModel(const vector<RaveTransform<dReal> >& vtrans)
{
    if(_pchain == NULL ) {
        // don't update, physics is disabled anyway
        return false;
    }

    if( vtrans.size() == 0 || _veclinks.size() != vtrans.size() )
        // something's wrong, so just return
        return false;

    Transform tglob = vtrans.front();//_pchain->GetCenterOfMass();
    SbMatrix m; m.makeIdentity();
    _ivXform->setMatrix(m);
    _ivXform->translation.setValue(tglob.trans.x, tglob.trans.y, tglob.trans.z);
    _ivXform->rotation.setValue(tglob.rot.y, tglob.rot.z, tglob.rot.w, tglob.rot.x);

    vector<LINK>::iterator it = _veclinks.begin();
    FOREACHC(ittrans, vtrans) {

        Transform tlocal = tglob.inverse() * *ittrans;

        it->second->rotation.setValue(tlocal.rot.y, tlocal.rot.z, tlocal.rot.w, tlocal.rot.x);
        it->second->translation.setValue(tlocal.trans.x, tlocal.trans.y, tlocal.trans.z);
        ++it;
    }

    return true;
}

void KinBodyItem::SetGrab(bool bGrab, bool bUpdate)
{
    if(_pchain == NULL )
        return;

    // need to preserve enabled state
    if( bGrab && !bGrabbed )
        bEnabled = _pchain->IsEnabled();

    bGrabbed = bGrab;

    if( bUpdate ) {
        if( bGrab ) UpdateFromModel();
        else UpdateFromIv();
    }

    // need to preserve enabled state
    if( bEnabled )
        _pchain->Enable(!bGrab);
}

KinBody::Link* KinBodyItem::GetLinkFromIv(SoNode* plinknode) const
{
    vector<LINK>::const_iterator it;
    vector<KinBody::Link*>::const_iterator itlink = _pchain->GetLinks().begin();
    SoSearchAction search;
    
    FORIT(it, _veclinks) {
        search.setNode(plinknode);   
        search.apply(it->first);
        
        if (search.getPath())
            return *itlink;
    
        ++itlink;
    }
    
    return NULL;
}

RobotItem::RobotItem(QtCoinViewer* viewer, RobotBase* robot, ViewGeometry viewgeom) : KinBodyItem(viewer, robot, viewgeom)
{
    int index = 0;
    FOREACHC(itmanip, robot->GetManipulators()) {

        if(itmanip->pEndEffector != NULL) {
            
            SoSwitch* peeswitch = new SoSwitch();
            SoSeparator* peesep = new SoSeparator();
            SoTransform* ptrans = new SoTransform();
            _vEndEffectors.push_back(EE(index, ptrans, peeswitch));

            _ivGeom->addChild(peeswitch);
            peeswitch->addChild(peesep);
            peeswitch->whichChild = SO_SWITCH_NONE;
            peesep->addChild(ptrans);

            // set a diffuse color
            {
                SoMaterial* mtrl = new SoMaterial;
                mtrl->diffuseColor = SbColor(1,0.5,0.5);
                mtrl->ambientColor = SbColor(1,0.5,0.5);
                peesep->addChild(mtrl);

                SoSphere* c = new SoSphere();
                c->radius = 0.004f;
                peesep->addChild(c);
            }
            
            // add some axes
            SoSeparator* paxes = new SoSeparator();

            Vector colors[] = {Vector(0,0,1),Vector(0,1,0),Vector(1,0,0)};
            Vector rotations[] = {Vector(1,0,0,PI/2), Vector(1,0,0,0), Vector(0,0,1,-PI/2)};

            // add 3 cylinder+cone axes
            for(int i = 0; i < 3; ++i) {
                // set a diffuse color
                SoSeparator* psep = new SoSeparator();

                SoMaterial* mtrl = new SoMaterial;
                mtrl->diffuseColor = SbColor(colors[i].x, colors[i].y, colors[i].z);
                mtrl->ambientColor = SbColor(colors[i].x, colors[i].y, colors[i].z);
                mtrl->setOverride(true);
            
                SoTransform* protation = new SoTransform();
                protation->rotation.setValue(SbVec3f(rotations[i].x, rotations[i].y, rotations[i].z), rotations[i].w);

                SoTransform* pcyltrans = new SoTransform();
                pcyltrans->translation.setValue(0,0.02f,0);

                SoCylinder* c = new SoCylinder();
                c->radius = 0.002f;
                c->height = 0.04f;
            
                SoCone* cn = new SoCone();
                cn->bottomRadius = 0.004f;
                cn->height = 0.02f;
            
                SoTransform* pconetrans = new SoTransform();
                pconetrans->translation.setValue(0,0.02f,0);

                psep->addChild(mtrl);
                psep->addChild(protation);
                psep->addChild(pcyltrans);
                psep->addChild(c);
                psep->addChild(pconetrans);
                psep->addChild(cn);
                paxes->addChild(psep);
            }

            peesep->addChild(paxes);

            // add text
            {
                SoSeparator* ptextsep = new SoSeparator();
                peesep->addChild(ptextsep);
        
                //Transform t = GetRaveTransform(_selectedItem->GetIvTransform());
                
                SoTranslation* ptrans = new SoTranslation();
                ptrans->translation.setValue(SbVec3f(0.02f,0.02f,0.02f));
                ptextsep->addChild(ptrans);
        
                SoTransparencyType* ptype = new SoTransparencyType();
                ptype->value = SoGLRenderAction::NONE;
                ptextsep->addChild(ptype);

                SoBaseColor* pcolor = new SoBaseColor();
                pcolor->rgb.setValue(0,0,0);
                ptextsep->addChild(pcolor);

                SoFont* pfont = new SoFont();
                pfont->name = "Courier:Bold";
                pfont->size = 18;
                ptextsep->addChild(pfont);

                SoText2 * ptext = new SoText2();
                char str[256];
                sprintf(str,"EE%d", index);
                ptext->string.setValue(str);
                ptextsep->addChild(ptext);
            }
        }

        ++index;
    }
}

void RobotItem::SetGrab(bool bGrab, bool bUpdate)
{
    if( GetRobot() == NULL )
        return;

    if( bGrab ) {
        // turn off any controller commands if a robot
        if( GetRobot()->GetController() != NULL )
            GetRobot()->GetController()->SetPath(NULL);
    }

    FOREACH(itee, _vEndEffectors)
        itee->_pswitch->whichChild = bGrab ? SO_SWITCH_ALL : SO_SWITCH_NONE;

    KinBodyItem::SetGrab(bGrab, bUpdate);
}

bool RobotItem::UpdateFromIv()
{
    if( !KinBodyItem::UpdateFromIv() )
        return false;

    return true;
}

bool RobotItem::UpdateFromModel(const vector<RaveTransform<dReal> >& vtrans)
{
    if( !KinBodyItem::UpdateFromModel(vtrans) )
        return false;

    if( bGrabbed ) {
        // only updated when grabbing!
        RaveTransform<float> transInvRoot = GetRaveTransform(_ivXform).inverse();
        
        FOREACH(itee, _vEndEffectors) {
            if( itee->_index >= 0 && itee->_index < GetRobot()->GetManipulators().size()) {
            
                // not thread safe!
                const RobotBase::Manipulator& manip = GetRobot()->GetManipulators()[itee->_index];
                if( manip.pEndEffector != NULL ) {
                    RaveTransform<float> tgrasp = vtrans[manip.pEndEffector->GetIndex()]*manip.tGrasp;
                    SetSoTransform(itee->_ptrans, transInvRoot * tgrasp);
                }
            }
        }
    }

    return true;
}


//                    SoGetBoundingBoxAction bbact(g_pEnviron->GetViewer()->GetViewer()->getViewportRegion());
//                    psep->ref();
//                    bbact.apply(psep);
//                    //psep->unref();
//                    SbXfBox3f box = bbact.getXfBoundingBox();
//                    Vector size;
//                    box.getSize(size.x, size.y, size.z);
//                    SbVec3f center = box.getCenter();
//                    SbMatrix mat = box.getTransform();
//                    const SbMat& m = mat.getValue();
//                    printf("%S:\n\t<Translation>%f %f %f</Translation>\n\t<Extents>%f %f %f</Extents>\n", (*it)->GetName(), center.getValue()[0], center.getValue()[1], center.getValue()[2], size.x/2, size.y/2, size.z/2);
//                    for(int k = 0; k < 3; ++k) {
//                        printf("%\tf %f %f %f\n", m[k][0], m[k][1], m[k][2], m[k][3]);
//                    }
//                    printf("\t<RotationMat>");
//                    for(int k = 0; k < 3; ++k) {
//                        printf("%f %f %f ", m[k][0], m[k][1], m[k][2]);
//                    }
//                    printf("</RotationMat>\n");
