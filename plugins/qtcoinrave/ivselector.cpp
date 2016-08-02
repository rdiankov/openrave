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
/**
   \file   IvSelector.cpp
   \brief  OpenInventor selection class
 */
#include "qtcoin.h"

const SbColor IvDragger::CHECK_COLOR(0.2f, 0.8f, 0.3f);
const SbColor IvDragger::COLLISION_COLOR(1.0f, 0.4f, 0.0f);

IvDragger::IvDragger(QtCoinViewerPtr viewer, ItemPtr pItem, float draggerScale)
{
    _selectedItem = pItem;
    _viewer = viewer;
    _scale = draggerScale;
    _penv = viewer->GetEnv();
    //_ptext = NULL;

    // set some default behavioral options
    _checkCollision = false;
    _prevtransparency = pItem->GetIvTransparency()->value;
    pItem->GetIvTransparency()->value = SoGLRenderAction::SCREEN_DOOR;

    if( !!pItem &&(pItem->GetIvRoot() != NULL)) {
        _GetBounds(pItem->GetIvRoot(), _ab);

        // make the item transparent
        SoSearchAction search;
        search.setType(SoMaterial::getClassTypeId());
        search.setInterest(SoSearchAction::ALL);
        search.apply(pItem->GetIvRoot());
        for(int i = 0; i < search.getPaths().getLength(); ++i) {
            SoPath* path = search.getPaths()[i];
            SoMaterial* pmtrl = (SoMaterial*)path->getTail();
            vtransparency.push_back(pmtrl->transparency[0]);
            pmtrl->transparency = 0.25f;
        }

        _vlinkaxes.resize(pItem->GetNumIvLinks());
        for(size_t i = 0; i < _vlinkaxes.size(); ++i) {
            _vlinkaxes[i] = _CreateAxes(i == 0 ? 1.0f : 0.25f,0.5f);
            pItem->GetIvLink(i)->addChild(_vlinkaxes[i]);
        }
    }
}

SoSeparator* IvDragger::_CreateAxes(float fSize,float fColor)
{
    SoSeparator* axes = new SoSeparator();
    Vector colors[] = { Vector(0,0,fColor),Vector(0,fColor,0),Vector(fColor,0,0)};
    Vector rotations[] = { Vector(1,0,0,PI/2), Vector(1,0,0,0), Vector(0,0,1,-PI/2)};

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
        pcyltrans->translation.setValue(0,0.02f*fSize,0);

        SoCylinder* c = new SoCylinder();
        c->radius = 0.002f*fSize;
        c->height = 0.04f*fSize;

        SoCone* cn = new SoCone();
        cn->bottomRadius = 0.004f*fSize;
        cn->height = 0.02f*fSize;

        SoTransform* pconetrans = new SoTransform();
        pconetrans->translation.setValue(0,0.02f*fSize,0);

        psep->addChild(mtrl);
        psep->addChild(protation);
        psep->addChild(pcyltrans);
        psep->addChild(c);
        psep->addChild(pconetrans);
        psep->addChild(cn);
        axes->addChild(psep);
    }
    return axes;
}

IvDragger::~IvDragger()
{
    ItemPtr selectedItem = GetSelectedItem();
    if( !!selectedItem &&(selectedItem->GetIvRoot() != NULL)) {
        for(size_t i = 0; i < _vlinkaxes.size(); ++i) {
            selectedItem->GetIvLink(i)->removeChild(_vlinkaxes[i]);
        }
        _vlinkaxes.clear();

        // revert transparency
        SoSearchAction search;
        search.setType(SoMaterial::getClassTypeId());
        search.setInterest(SoSearchAction::ALL);
        search.apply(selectedItem->GetIvRoot());
        for(int i = 0; i < search.getPaths().getLength(); ++i) {
            SoPath* path = search.getPaths()[i];
            SoMaterial* pmtrl = (SoMaterial*)path->getTail();
            if( i < (int)vtransparency.size() )
                pmtrl->transparency = vtransparency[i];
        }

        selectedItem->GetIvTransparency()->value = _prevtransparency;
    }
}

void IvDragger::_GetBounds(SoSeparator *subtree, AABB& ab)
{
    SoGetBoundingBoxAction bbox(_viewer.lock()->GetViewer()->getViewportRegion());
    bbox.apply(subtree);
    SbBox3f box = bbox.getBoundingBox();
    RaveVector<float> vmin, vmax;
    box.getBounds(vmin.x,vmin.y,vmin.z,vmax.x,vmax.y,vmax.z);
    ab.pos = 0.5*(vmin+vmax);
    ab.extents = 0.5*(vmax-vmin);
}

void IvDragger::_GetMatrix(SbMatrix &matrix, SoNode *root, SoNode *node)
{
    SoGetMatrixAction getXform(_viewer.lock()->GetViewer()->getViewportRegion());

    // get a path from the root to the node

    SoSearchAction mySearchAction;
    mySearchAction.setNode(node);
    mySearchAction.setInterest(SoSearchAction::FIRST);
    mySearchAction.apply(root);

    // get the transformation matrix

    getXform.apply(mySearchAction.getPath());
    matrix = getXform.getMatrix();
}

void IvDragger::_MotionHandler(void *userData, SoDragger *)
{
    try {
        ((IvDragger *) userData)->UpdateSkeleton();
    }
    catch(const openrave_exception& ex) {
        RAVELOG_ERROR_FORMAT("unexpected openrave error in gui handler: %s", ex.what());
    }
}

IvObjectDragger::IvObjectDragger(QtCoinViewerPtr viewer, ItemPtr pItem, float draggerScale, bool bAllowRotation)
    : IvDragger(viewer, pItem, draggerScale)
{
    // create a root node for the dragger nodes
    _draggerRoot = new SoSeparator;
    ItemPtr selectedItem = GetSelectedItem();
    selectedItem->GetIvRoot()->insertChild(_draggerRoot, 0);

    // create and size a transform box dragger and then add it to the scene graph
    _transformBox = new SoTransformBoxDragger;

    _transformBox->scaleFactor.setValue(_ab.extents.x * _scale, _ab.extents.y * _scale, _ab.extents.z * _scale);
    _transformBox->translation.setValue(_ab.pos.x, _ab.pos.y, _ab.pos.z);
    _toffset = selectedItem->GetTransform();

    _draggerRoot->addChild(_transformBox);

    // disable the scaling part of the transform box
    _transformBox->setPart("scaler", NULL);

    // disable the rotation around the X and Z axes
    if (!bAllowRotation) {
        const char *rotators[2] = { "rotator1", "rotator3" };
        for (int i = 0; i < 2; i++)
            _transformBox->setPart(rotators[i], NULL);
    }

    // get the material node that governs the color of the dragger and
    // note the dragger's normal color
    if (bAllowRotation) {
        SoRotateCylindricalDragger *rp = (SoRotateCylindricalDragger *)_transformBox->getPart("rotator1", false);
        SoSeparator *s = (SoSeparator *) rp->getPart("rotator", false);
        _draggerMaterial = (SoMaterial *) s->getChild(0);
    }
    else
        _draggerMaterial = new SoMaterial();

    _normalColor = _draggerMaterial->diffuseColor[0];

    // add a motion callback handler for the dragger
    _transformBox->addMotionCallback(_MotionHandler, this);

    UpdateDragger();
}

IvObjectDragger::~IvObjectDragger()
{
    _SetColor(_normalColor);

    _transformBox->removeMotionCallback(_MotionHandler, this);
    if( _draggerRoot != NULL ) {
        ItemPtr selectedItem = GetSelectedItem();
        if( !!selectedItem ) {
            selectedItem->GetIvRoot()->removeChild(_draggerRoot);
        }
    }
}

void IvObjectDragger::_SetColor(const SbColor &color)
{
    _draggerMaterial->diffuseColor.setValue(color);
    _draggerMaterial->emissiveColor.setValue(color);
}

// If the dragger is to detect for collision between the moving object
// and the rest of the world, set up the appropriate collision models.
void IvObjectDragger::CheckCollision(bool flag)
{
    _checkCollision = flag;
    ItemPtr selectedItem = GetSelectedItem();
    if (_checkCollision && !!selectedItem) {
        // synchronize the collision model transform
        KinBodyItemPtr pbody = std::dynamic_pointer_cast<KinBodyItem>(selectedItem);
        if( !!pbody ) {
            EnvironmentMutex::scoped_try_lock lock(_penv->GetMutex());
            if( !!lock ) {
                int prevoptions = _penv->GetCollisionChecker()->GetCollisionOptions();
                _penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
                CollisionReportPtr preport(new CollisionReport());
                if( pbody->GetBody()->CheckSelfCollision(preport) ) {
                    RAVELOG_VERBOSE(str(boost::format("self-collision %s\n")%preport->__str__()));
                    _SetColor(COLLISION_COLOR);
                }
                else if( _penv->CheckCollision(KinBodyConstPtr(pbody->GetBody()), preport)) {
                    // if there is a collision, revert to the original transform
                    RAVELOG_VERBOSE(str(boost::format("collision %s\n")%preport->__str__()));
                    _SetColor(COLLISION_COLOR);
                }
                else {
                    _SetColor(CHECK_COLOR);
                }
                _penv->GetCollisionChecker()->SetCollisionOptions(prevoptions);
            }
        }
    }
}

// Update the skeleton transforms based on the dragger.
void IvObjectDragger::UpdateSkeleton()
{
    ItemPtr selectedItem = GetSelectedItem();
    if( !selectedItem ) {
        return;
    }
    RaveTransform<float> tbox;
    const float* q = _transformBox->rotation.getValue().getValue();
    tbox.rot = Vector(q[3], q[0], q[1], q[2]);
    SbVec3f v = _transformBox->translation.getValue();
    tbox.trans = Vector(v[0], v[1], v[2]);

    Transform told; told.trans = -_ab.pos;

    RaveTransform<float> tnew = tbox*told*_toffset;
    SetSoTransform(selectedItem->GetIvTransform(), tnew);

    KinBodyItemPtr pbody = std::dynamic_pointer_cast<KinBodyItem>(selectedItem);
    if( !!pbody ) {
        pbody->UpdateFromIv();
        CheckCollision(_checkCollision);
    }
    // other motion handler calls
    _viewer.lock()->_UpdateCameraTransform(0);
}

void IvObjectDragger::GetMessage(ostream& sout)
{
    ItemPtr selectedItem = GetSelectedItem();
    if( !selectedItem ) {
        return;
    }
    KinBodyItemPtr pbody = std::dynamic_pointer_cast<KinBodyItem>(selectedItem);
    if( !pbody ) {
        return;
    }

    Transform t = pbody->GetTransform();
    sout << "Selected " << selectedItem->GetName() << " (id=" << pbody->GetNetworkId() << ")" << endl
         << "  translation = ("
         << std::fixed << std::setprecision(5)
         << std::setw(8) << std::left << t.trans.x << ", "
         << std::setw(8) << std::left << t.trans.y << ", "
         << std::setw(8) << std::left << t.trans.z << ")" << endl
         << "  quaternion = ("
         << std::fixed << std::setprecision(5)
         << std::setw(8) << std::left << t.rot.x << ", "
         << std::setw(8) << std::left << t.rot.y << ", "
         << std::setw(8) << std::left << t.rot.z << ", "
         << std::setw(8) << std::left << t.rot.w << ")" << endl;
}

IvJointDragger::IvJointDragger(QtCoinViewerPtr viewer, ItemPtr pItem, int iSelectedLink, float draggerScale, int iJointIndex, bool bHilitJoint) : IvDragger(viewer, pItem, draggerScale)
{
    KinBodyItemPtr pbody = std::dynamic_pointer_cast<KinBodyItem>(pItem);
    BOOST_ASSERT( !!pItem );

    _trackball = NULL;
    _draggerRoot = NULL;

    if( !pbody || !pbody->GetBody() ) {
        return;
    }
    if((iSelectedLink < 0)||(iSelectedLink >= (int)pbody->GetBody()->GetLinks().size())) {
        return;
    }
    if((iJointIndex < 0)||(iJointIndex >= (int)pbody->GetBody()->GetJoints().size())) {
        return;
    }

    _iSelectedLink = iSelectedLink;
    _iJointIndex = iJointIndex;
    KinBody::JointConstPtr pjoint = pbody->GetBody()->GetJoints().at(iJointIndex);

    _jointtype = pjoint->GetType();
    _dofindex = pjoint->GetDOFIndex();
    _jointname = pjoint->GetName();
    _jointoffset = 0; //pjoint->GetOffset();
    pjoint->GetLimits(_vlower,_vupper);

    _pLinkNode = pbody->GetIvLink(iSelectedLink);
    if( _pLinkNode == NULL ) {
        RAVELOG_WARN("no link is selected\n");
        return;
    }

    Transform tlink = pbody->GetBody()->GetLinks().at(iSelectedLink)->GetTransform();

    // create a root node for the dragger nodes
    _draggerRoot = new SoSeparator;
    SoTransform* draggertrans = new SoTransform();
    _pLinkNode->insertChild(_draggerRoot, 1); // insert right after transform

    // add a new material to change the color of the nodes being dragged
    _bHilitJoint = bHilitJoint;
    if (_bHilitJoint) {
        _material = new SoMaterial;
        _material->set("diffuseColor 0.8 0.6 0.2");
        _material->setOverride(true);
        _pLinkNode->insertChild(_material, 1);
    }

    Vector vaxes[3];
    for(int i = 0; i < pjoint->GetDOF(); ++i) {
        vaxes[i] = tlink.inverse().rotate(pjoint->GetAxis(i));
    }

    // need to make sure the rotation is pointed towards the joint axis
    Vector vnorm = Vector(1,0,0).cross(vaxes[0]);
    dReal fsinang = RaveSqrt(vnorm.lengthsqr3());
    if( fsinang > 0.0001f ) {
        vnorm /= fsinang;
    }
    else vnorm = Vector(1,0,0);

    Vector vtrans = tlink.inverse()*pjoint->GetAnchor();
    draggertrans->translation.setValue(vtrans.x, vtrans.y, vtrans.z);
    draggertrans->rotation = SbRotation(SbVec3f(vnorm.x, vnorm.y, vnorm.z), atan2f(fsinang,vaxes[0].x));
    _draggerRoot->addChild(draggertrans);

    // construct an Inventor trackball dragger
    float scale = _scale;
    _trackball = new SoTrackballDragger;
    AABB ab;
    _GetBounds(_pLinkNode, ab);
    _trackball->scaleFactor.setValue(ab.extents.x * scale, ab.extents.y * scale, ab.extents.z * scale);
    _trackball->setAnimationEnabled(false);
    _draggerRoot->addChild(_trackball);

    // get the material nodes that govern the color of the dragger and
    // note the dragger's normal color
    const char* rotators[3] = { "XRotator", "YRotator", "ZRotator" };
    const char* rotatorsActive[3] = { "XRotatorActive", "YRotatorActive", "ZRotatorActive" };

    // enable or disable each axis
    for (int i = 0; i < 3; i++) {
        if (i < pjoint->GetDOF()) {
            SoSeparator *s = (SoSeparator *)_trackball->getPart(rotators[i], false);
            _draggerMaterial[i] = (SoMaterial *) s->getChild(0);
            _normalColor = _draggerMaterial[i]->diffuseColor[0];
        }
        else {
            // disable the rotator on this axis
            _trackball->setPart(rotators[i], NULL);
            _trackball->setPart(rotatorsActive[i], NULL);
            _draggerMaterial[i] = NULL;
        }
    }

    // add a motion callback handler for the dragger
    _trackball->addMotionCallback(_MotionHandler, this);

    UpdateDragger();
}

IvJointDragger::~IvJointDragger()
{
    _SetColor(_normalColor);
    if( _trackball != NULL )
        _trackball->removeMotionCallback(_MotionHandler, this);
    if( _pLinkNode != NULL ) {
        if(_draggerRoot != NULL )
            _pLinkNode->removeChild(_draggerRoot);
        if (_bHilitJoint)
            _pLinkNode->removeChild(_material);
    }
}

void IvJointDragger::_SetColor(const SbColor &color)
{
    for (int i = 0; i < 3; i++)
        if (_draggerMaterial[i]) {
            _draggerMaterial[i]->diffuseColor.setValue(color);
            _draggerMaterial[i]->emissiveColor.setValue(color);
        }
}

// Set up collision models for the moving subtree of nodes and the rest of the world.
void IvJointDragger::CheckCollision(bool flag)
{
    _checkCollision = flag;
    ItemPtr selectedItem = GetSelectedItem();
    if (_checkCollision && !!selectedItem) {
        KinBodyItemPtr pbody = std::dynamic_pointer_cast<KinBodyItem>(selectedItem);

        if( !!pbody ) {
            EnvironmentMutex::scoped_try_lock lock(_penv->GetMutex());
            if( !!lock ) {
                int prevoptions = _penv->GetCollisionChecker()->GetCollisionOptions();
                _penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
                if (_penv->CheckCollision(KinBodyConstPtr(pbody->GetBody())) || pbody->GetBody()->CheckSelfCollision()) {
                    _SetColor(COLLISION_COLOR);
                }
                else {
                    _SetColor(CHECK_COLOR);
                }
                _penv->GetCollisionChecker()->SetCollisionOptions(prevoptions);
            }
        }
    }
}

// Update the skeleton transforms based on the dragger.
void IvJointDragger::UpdateSkeleton()
{
    ItemPtr selectedItem = GetSelectedItem();
    if( !selectedItem ) {
        return;
    }
    KinBodyItemPtr pbody = std::dynamic_pointer_cast<KinBodyItem>(selectedItem);
    if( !pbody ) {
        return;
    }

    // save the transform of the object before dragging
    SbMatrix mrot;
    _trackball->rotation.getValue().getValue(mrot);

    float fang = -atan2f(mrot[2][1], mrot[1][1]);

    // if a robot, reset the controller
    RobotItemPtr probotitem = std::dynamic_pointer_cast<RobotItem>(pbody);

    {
        EnvironmentMutex::scoped_try_lock lock(_penv->GetMutex());

        if( !!lock ) {
            KinBody::JointPtr pjoint = pbody->GetBody()->GetJoints()[_iJointIndex];
            int d = pjoint->GetDOFIndex();
            vector<dReal> vlower,vupper;
            pbody->GetBody()->GetDOFLimits(vlower, vupper);

            if( pjoint->GetType() == KinBody::JointSlider ) {
                fang = fang*(vupper.at(d)-vlower.at(d))+vlower.at(d);
            }

            // update the joint's transform
            vector<dReal> vjoints;
            pbody->GetBody()->GetDOFValues(vjoints);
            // double check all the limits
            for(size_t i = 0; i < vjoints.size(); ++i) {
                if( vjoints[i] < vlower[i] ) {
                    vjoints[i] = vlower[i];
                }
                else if( vjoints[i] > vupper[i] ) {
                    vjoints[i] = vupper[i];
                }
            }
            if( pjoint->GetType() == KinBody::JointSpherical ) {
                SbVec3f axis; float angle;
                _trackball->rotation.getValue(axis,angle);
                vjoints.at(d+0) = axis[0]*angle;
                vjoints.at(d+1) = axis[1]*angle;
                vjoints.at(d+2) = axis[2]*angle;
            }
            else {
                vjoints.at(d+0) = fang+_jointoffset;
                if( pjoint->IsRevolute(0) ) {
                    if( vjoints.at(d) < vlower.at(d) ) {
                        if( vlower.at(d)-vjoints.at(d) < vjoints.at(d)+2*PI-vupper.at(d) ) {
                            vjoints[d] = vlower[d];
                        }
                        else {
                            vjoints[d] = vupper[d];
                        }
                    }
                    else if( vjoints.at(d) > vupper.at(d) ) {
                        if( vlower.at(d)-vjoints.at(d)+2*PI < vjoints.at(d)-vupper.at(d) ) {
                            vjoints[d] = vlower[d];
                        }
                        else {
                            vjoints[d] = vupper[d];
                        }
                    }
                }
                else {
                    if( vjoints.at(d) < vlower.at(d) ) {
                        vjoints[d] = vlower[d];
                    }
                    else if( vjoints.at(d) > vupper.at(d) ) {
                        vjoints[d] = vupper[d];
                    }
                }
            }

            if( !!probotitem && !!probotitem->GetRobot()->GetController() ) {
                probotitem->GetRobot()->GetController()->SetDesired(vjoints);
            }
            else {
                pbody->GetBody()->SetDOFValues(vjoints, KinBody::CLA_CheckLimits);
            }
        }
    }

    _viewer.lock()->_UpdateCameraTransform(0);

    UpdateDragger();

    // do collision checking if required
    CheckCollision(_checkCollision);

    selectedItem->SetGrab(false, false); // will be updating manually
    pbody->UpdateFromModel();
}

// Update the dragger based on the skeleton.
void IvJointDragger::UpdateDragger()
{
    ItemPtr selectedItem = GetSelectedItem();
    if( !selectedItem ) {
        return;
    }
    KinBodyItemPtr pbody = std::dynamic_pointer_cast<KinBodyItem>(selectedItem);
    if( !pbody ) {
        return;
    }

    vector<dReal> vjoints;
    pbody->GetDOFValues(vjoints);

    if( _jointtype == KinBody::JointSpherical ) {
        Vector vaxis(vjoints[_dofindex+0],vjoints[_dofindex+1],vjoints[_dofindex+2]);
        dReal fang = RaveSqrt(vaxis.lengthsqr3())-_jointoffset;
        _trackball->rotation = SbRotation(fang > 0 ? SbVec3f(vaxis.x/fang,vaxis.y/fang,vaxis.z/fang) : SbVec3f(1,0,0), fang);
    }
    else {
        float fang = vjoints[_dofindex]-_jointoffset;
        if( _jointtype == KinBody::JointSlider ) {
            if( _vupper[0] > _vlower[0] ) {
                fang = (fang-_vlower[0])/(_vupper[0]-_vlower[0]);
            }
            else {
                fang = 0;
            }
        }
        _trackball->rotation = SbRotation(SbVec3f(1,0,0), fang);
    }
}

void IvJointDragger::GetMessage(ostream& sout)
{
    ItemPtr selectedItem = GetSelectedItem();
    if( !selectedItem ) {
        return;
    }
    KinBodyItemPtr pbody = std::dynamic_pointer_cast<KinBodyItem>(selectedItem);
    if( !pbody ) {
        return;
    }

    vector<dReal> vjoints;
    pbody->GetDOFValues(vjoints);

    sout << "Selected " << selectedItem->GetName() << " (id=" << pbody->GetNetworkId() << ")" << endl
         << std::fixed << std::setprecision(4)
         << "  joint " << _jointname << " (" << _iJointIndex << ") "  << " = " << vjoints[_iJointIndex];

    if( pbody->GetBody()->GetJoints()[_iJointIndex]->GetType() != KinBody::JointSlider ) {
        sout << " rad (" << (vjoints[_iJointIndex]/PI*180.0f) << " deg)" << endl;
    }
}
