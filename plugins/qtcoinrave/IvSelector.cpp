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
\file   IvSelector.cpp
\brief  OpenInventor selection class
-------------------------------------------------------------------- */

#include "qtcoin.h"

const SbColor IvDragger::CHECK_COLOR(0.2f, 0.8f, 0.3f);
const SbColor IvDragger::COLLISION_COLOR(1.0f, 0.4f, 0.0f);

///////////////////
// IvDragger class
// Construct a new dragger given the root of the scene graph and
// the selected node.
IvDragger::IvDragger(QtCoinViewer* viewer, Item *pItem, float draggerScale)
{
    _selectedItem = pItem;
    _viewer = viewer;
    _scale = draggerScale;
    _axes = NULL;
    //_ptext = NULL;

    // set some default behavioral options
    _checkCollision = false;
    _prevtransparency = pItem->GetIvTransparency()->value;
    pItem->GetIvTransparency()->value = SoGLRenderAction::SCREEN_DOOR;

    if( _selectedItem != NULL && _selectedItem->GetIvRoot() != NULL ) {

        _GetBounds(_selectedItem->GetIvRoot(), _ab);

        // make the item transparent
        SoSearchAction search;
        search.setType(SoMaterial::getClassTypeId());
        search.setInterest(SoSearchAction::ALL);
        search.apply(_selectedItem->GetIvRoot());
        for(int i = 0; i < search.getPaths().getLength(); ++i) {
            SoPath* path = search.getPaths()[i];
            SoMaterial* pmtrl = (SoMaterial*)path->getTail();
            vtransparency.push_back(pmtrl->transparency[0]);
            pmtrl->transparency = 0.25f;
        }
        
        _axes = new SoSeparator();

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
            _axes->addChild(psep);
        }

        _selectedItem->GetIvRoot()->addChild(_axes);
    }
    else _axes = NULL;
}

// Destructor
IvDragger::~IvDragger()
{
    if( _selectedItem != NULL && _selectedItem->GetIvRoot() != NULL ) {
        _selectedItem->GetIvRoot()->removeChild(_axes);

        // revert transparency
        SoSearchAction search;
        search.setType(SoMaterial::getClassTypeId());
        search.setInterest(SoSearchAction::ALL);
        search.apply(_selectedItem->GetIvRoot());
        for(int i = 0; i < search.getPaths().getLength(); ++i) {
            SoPath* path = search.getPaths()[i];
            SoMaterial* pmtrl = (SoMaterial*)path->getTail();
            if( i < (int)vtransparency.size() )
                pmtrl->transparency = vtransparency[i];
        }

        _selectedItem->GetIvTransparency()->value = _prevtransparency;
    }
}

// Get the bounding box of the given subtree.
void IvDragger::_GetBounds(SoSeparator *subtree, AABB& ab)
{
    SoGetBoundingBoxAction bbox(_viewer->GetViewer()->getViewportRegion());
    bbox.apply(subtree);
    SbBox3f box = bbox.getBoundingBox();
    RaveVector<float> vmin, vmax;
    box.getBounds(vmin.x,vmin.y,vmin.z,vmax.x,vmax.y,vmax.z);
    ab.pos = 0.5*(vmin+vmax);
    ab.extents = 0.5*(vmax-vmin);
}

// Get the Inventor transformation matrix that describes the given
// node relative to the given root.
void IvDragger::_GetMatrix(SbMatrix &matrix, SoNode *root, SoNode *node)
{
    SoGetMatrixAction getXform(_viewer->GetViewer()->getViewportRegion());

    // get a path from the root to the node

    SoSearchAction mySearchAction;
    mySearchAction.setNode(node);
    mySearchAction.setInterest(SoSearchAction::FIRST);
    mySearchAction.apply(root);

    // get the transformation matrix

    getXform.apply(mySearchAction.getPath());
    matrix = getXform.getMatrix();
}

// Handler for Inventor motion callbacks.
void IvDragger::_MotionHandler(void *userData, SoDragger *)
{
    ((IvDragger *) userData)->UpdateSkeleton();
}

// Class to represent an object dragger. This allows general
// translation and rotation, and checks for collision against
// the rest of the world if requested.
IvObjectDragger::IvObjectDragger(QtCoinViewer* viewer, Item *pItem, float draggerScale, bool bAllowRotation)
: IvDragger(viewer, pItem, draggerScale)
{
    //if( _ptext != NULL ) {
//        char str[256];
//        sprintf(str,"%S", _selectedItem->GetName());
//        _ptext->string.setValue(str);
//    }

    // create a root node for the dragger nodes
    _draggerRoot = new SoSeparator;
    _selectedItem->GetIvRoot()->insertChild(_draggerRoot, 0);

    // create and size a transform box dragger and then add it to the scene graph
    _transformBox = new SoTransformBoxDragger;

    _transformBox->scaleFactor.setValue(_ab.extents.x * _scale, _ab.extents.y * _scale, _ab.extents.z * _scale);
    _transformBox->translation.setValue(_ab.pos.x, _ab.pos.y, _ab.pos.z);
    _toffset = _selectedItem->GetTransform();

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

// Destructor
IvObjectDragger::~IvObjectDragger()
{
    _SetColor(_normalColor);

    _transformBox->removeMotionCallback(_MotionHandler, this);
    _selectedItem->GetIvRoot()->removeChild(_draggerRoot);
}

// Set the color of the dragger.
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

    if (_checkCollision) {
        // synchronize the collision model transform
        KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(_selectedItem);
        if( pbody != NULL ) {
            _viewer->GetEnv()->LockPhysics(true);
            bool bPrevEnable = pbody->GetBody()->IsEnabled();
            if( !bPrevEnable )
                pbody->GetBody()->Enable(true);

            COLLISIONREPORT report;

            if( pbody->GetBody()->CheckSelfCollision(&report) ) {
                RAVELOG_VERBOSEA("self collisionp  %S, links %S:%S\n", pbody->GetBody()->GetName(),
                                 report.plink1 != NULL ? report.plink1->GetName() : L"(NULL)",
                                 report.plink2 != NULL ? report.plink2->GetName() : L"(NULL)");
                _SetColor(COLLISION_COLOR);
            }
            else if( _viewer->GetEnv()->CheckCollision(pbody->GetBody(), &report)) {
                // if there is a collision, revert to the original transform
                RAVELOG_VERBOSEA("collision %S:%S with %S:%S\n",
                                 report.plink1?report.plink1->GetParent()->GetName():L"(NULL",
                                 report.plink1?report.plink1->GetName():L"(NULL)",
                                 report.plink2?report.plink2->GetParent()->GetName():L"(NULL)",
                                 report.plink2?report.plink2->GetName():L"(NULL)");
                _SetColor(COLLISION_COLOR);
            }
            else
                _SetColor(CHECK_COLOR);

            if( !bPrevEnable )
                pbody->GetBody()->Enable(false);
            
            _viewer->GetEnv()->LockPhysics(false);
        }
    }
}

// Update the skeleton transforms based on the dragger.
void IvObjectDragger::UpdateSkeleton()
{
    RaveTransform<float> tbox;
    const float* q = _transformBox->rotation.getValue().getValue();
    tbox.rot = Vector(q[3], q[0], q[1], q[2]);
    SbVec3f v = _transformBox->translation.getValue();
    tbox.trans = Vector(v[0], v[1], v[2]);

    Transform told; told.trans = -_ab.pos;
    
    RaveTransform<float> tnew = tbox*told*_toffset;
    SetSoTransform(_selectedItem->GetIvTransform(), tnew);

    KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(_selectedItem);

    if( pbody != NULL ) {
        pbody->UpdateFromIv();
        CheckCollision(_checkCollision);
    }
    // other motion handler calls
    _viewer->UpdateCameraTransform();
}

void IvObjectDragger::GetMessage(ostream& sout)
{
    KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(_selectedItem);
    if( pbody == NULL )
        return;

    Transform t = pbody->GetTransform();

    string name; name.resize(wcslen(_selectedItem->GetName()));
    sprintf(&name[0], "%S", _selectedItem->GetName());
                       
    sout << "Selected " << name << " (id=" << pbody->GetNetworkId() << ")" << endl
         << "  translation = ("
         << std::fixed << std::setprecision(3) 
         << std::setw(8) << std::left << t.trans.x << ", "
         << std::setw(8) << std::left << t.trans.y << ", "
         << std::setw(8) << std::left << t.trans.z << ")" << endl
         << "  quaternion = ("
         << std::fixed << std::setprecision(3) 
         << std::setw(8) << std::left << t.rot.x << ", "
         << std::setw(8) << std::left << t.rot.y << ", "
         << std::setw(8) << std::left << t.rot.z << ", "
         << std::setw(8) << std::left << t.rot.w << ")" << endl;
} 

// Class to represent an joint rotation dragger. This allows
// rotation relative to the parent joint. It honors joint limits
// and checks for collision between the world and the joint's subtree.
IvJointDragger::IvJointDragger(QtCoinViewer *viewer, Item *pItem, int iSelectedLink, float draggerScale, int iJointIndex, bool bHilitJoint)
                               : IvDragger(viewer, pItem, draggerScale)
{
    KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(pItem);
    assert( pItem != NULL );

    _trackball = NULL;
    _draggerRoot = NULL;

    if( pbody == NULL || pbody->GetBody() == NULL )
        return;
    if( iSelectedLink < 0 || iSelectedLink >= (int)pbody->GetBody()->GetLinks().size() )
        return;
    if( iJointIndex < 0 || iJointIndex >= (int)pbody->GetBody()->GetJoints().size() )
        return;

    _iSelectedLink = iSelectedLink;
    _iJointIndex = iJointIndex;
    const KinBody::Joint* pjoint = pbody->GetBody()->GetJoints()[iJointIndex];

    _pLinkNode = pbody->GetIvLink(iSelectedLink);
    if( _pLinkNode == NULL ) {
        RAVELOG(L"no link is selected\n");
        return;
    }

    //if( _ptext != NULL ) {
//        char str[256];
//        sprintf(str,"%S:joint %d", _selectedItem->GetName(), _iJointIndex);
//        _ptext->string.setValue(str);
//    }

    // create a root node for the dragger nodes
    _draggerRoot = new SoSeparator;
    SoTransform* draggertrans = new SoTransform();
    _pLinkNode->insertChild(_draggerRoot, 0);

    // add a new material to change the color of the nodes being dragged
    _bHilitJoint = bHilitJoint;
    if (_bHilitJoint) {
        _material = new SoMaterial;
        _material->set("diffuseColor 0.8 0.6 0.2");
        _material->setOverride(true);
        _pLinkNode->insertChild(_material, 1);
    }
    
    for(int i = 0; i < pjoint->GetDOF(); ++i)
        vaxes[i] = pjoint->GetAxis(i);

    // need to make sure the rotation is pointed towards the joint axis
    Vector vnorm; cross3(vnorm, Vector(1,0,0), vaxes[0]);
    float fsinang = sqrtf(lengthsqr3(vnorm));
    if( fsinang > 0.01f )
        vnorm /= fsinang;   
    else vnorm = Vector(1,0,0);
    
    Vector vtrans = GetJointOffset();
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

RaveVector<float> IvJointDragger::GetJointOffset()
{
    KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(_selectedItem);
    if( pbody == NULL )
        return Vector();

    // offset the dragger based on the joint offset so that it appears
    // at the joint
    Vector vtrans, vlinkoffset;
    vlinkoffset = Vector(_selectedItem->GetIvTransform()->translation.getValue().getValue());
    
    const KinBody::Joint* pjoint = pbody->GetBody()->GetJoints()[_iJointIndex];
    switch(pjoint->GetType()) {
    case KinBody::Joint::JointHinge:
    case KinBody::Joint::JointUniversal:
    case KinBody::Joint::JointHinge2:
        vtrans = pjoint->GetAnchor() - vlinkoffset;
        break;
    default:            
        vtrans = pbody->GetBody()->GetLinks()[_iSelectedLink]->GetTransform().trans - vlinkoffset;
    }

    return vtrans;
}

// Destructor
IvJointDragger::~IvJointDragger()
{
    _SetColor(_normalColor);
    if( _trackball != NULL )
        _trackball->removeMotionCallback(_MotionHandler, this);
    if( _pLinkNode != NULL ) {
        _pLinkNode->removeChild(_draggerRoot);
        if (_bHilitJoint)
            _pLinkNode->removeChild(_material);
    }
}

// Set the color of the dragger.
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

    if (_checkCollision) {
        KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(_selectedItem);
        
        if( pbody != NULL ) {
            _viewer->GetEnv()->LockPhysics(true);

            bool bPrevEnable = pbody->GetBody()->IsEnabled();
            if( !bPrevEnable )
                pbody->GetBody()->Enable(true);

            if (_viewer->GetEnv()->CheckCollision(pbody->GetBody()) || pbody->GetBody()->CheckSelfCollision())
                _SetColor(COLLISION_COLOR);
            else
                _SetColor(CHECK_COLOR);

            if( !bPrevEnable )
                pbody->GetBody()->Enable(false);

            _viewer->GetEnv()->LockPhysics(false);
        }
    }
}

// Update the skeleton transforms based on the dragger.
void IvJointDragger::UpdateSkeleton()
{
    KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(_selectedItem);
    if( pbody == NULL )
        return;

    // save the transform of the object before dragging
    SbMatrix mrot;
    _trackball->rotation.getValue().getValue(mrot);

    float fang = atan2f(mrot[2][1], mrot[1][1]);

    // if a robot, reset the controller
    RobotItem* probotitem = dynamic_cast<RobotItem*>(pbody);

    _viewer->GetEnv()->LockPhysics(true);

    KinBody::Joint* pjoint = pbody->GetBody()->GetJoints()[_iJointIndex];
    vector<dReal> vlower(pjoint->GetDOF()), vupper(pjoint->GetDOF());
    pjoint->GetLimits(&vlower[0], &vupper[0]);

    if( pjoint->GetType() == KinBody::Joint::JointSlider )
        fang = fang*(vupper[0]-vlower[0])+vlower[0];

    // update the joint's transform
    vector<dReal> vjoints;
    pbody->GetBody()->GetJointValues(vjoints);
    vjoints[_iJointIndex] = fang;

    if( probotitem != NULL && probotitem->GetRobot()->GetController() != NULL ) {
        probotitem->GetRobot()->GetController()->SetDesired(&vjoints[0]);
    }
    else {
        pbody->GetBody()->SetJointValues(NULL, NULL,&vjoints[0],true);
    }
    _viewer->GetEnv()->LockPhysics(false);

    _viewer->UpdateCameraTransform();

    UpdateDragger();

    // do collision checking if required
    CheckCollision(_checkCollision);

    _selectedItem->SetGrab(false, false); // will be updating manually
    pbody->UpdateFromModel();
}

// Update the dragger based on the skeleton.
void IvJointDragger::UpdateDragger()
{
    KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(_selectedItem);
    if( pbody == NULL )
        return;
    
    _viewer->GetEnv()->LockPhysics(true);
    vector<dReal> vjoints;
    pbody->GetBody()->GetJointValues(vjoints);
    float fang = vjoints[_iJointIndex];

    KinBody::Joint* pjoint = pbody->GetBody()->GetJoints()[_iJointIndex];
    vector<dReal> vlower(pjoint->GetDOF()), vupper(pjoint->GetDOF());
    pjoint->GetLimits(&vlower[0], &vupper[0]);
    if( pjoint->GetType() == KinBody::Joint::JointSlider ) {
        if( vupper[0] > vlower[0] )
            fang = (fang-vlower[0])/(vupper[0]-vlower[0]);
        else
            fang = 0;
    }

    _viewer->GetEnv()->LockPhysics(false);

    _trackball->rotation = SbRotation(SbVec3f(1,0,0), -fang);
    ((SoTransform*)_draggerRoot->getChild(0))->translation.setValue(GetJointOffset());
}

void IvJointDragger::GetMessage(ostream& sout)
{
    KinBodyItem* pbody = dynamic_cast<KinBodyItem*>(_selectedItem);
    if( pbody == NULL )
        return;

    vector<dReal> vjoints;
    pbody->GetBody()->GetJointValues(vjoints);
    
    string name; name.resize(wcslen(_selectedItem->GetName()));
    sprintf(&name[0], "%S", _selectedItem->GetName());

    sout << "Selected " << name << " (id=" << pbody->GetNetworkId() << ")" << endl
         << std::fixed << std::setprecision(3)
         << "  joint " << _iJointIndex << " = " << vjoints[_iJointIndex];

    if( pbody->GetBody()->GetJoints()[_iJointIndex]->GetType() != KinBody::Joint::JointSlider )
        sout << " rad (" << (vjoints[_iJointIndex]/PI*180.0f) << " deg)" << endl;
}
