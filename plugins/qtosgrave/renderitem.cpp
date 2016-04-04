// -*- coding: utf-8 -*-
// Copyright (C) 2012-2014 Gustavo Puche, Rosen Diankov, OpenGrasp Team
//
// OpenRAVE Qt/OpenSceneGraph Viewer is licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*! --------------------------------------------------------------------
   \file   Item.cpp
   \brief  Abstract base class for an Item
   -------------------------------------------------------------------- */
#include "qtosg.h"
#include "qtosgviewer.h"
#include "renderitem.h"

namespace qtosgrave {

Item::Item(QtOSGViewerPtr viewer) : _viewer(viewer)
{
    // set up the Inventor nodes
    _ivXform = new osg::MatrixTransform;
    _ivRoot = new osg::Group;
    _ivGeom = new osg::Switch; // TODO : Put 2 childrens...
    _ivGeom->setAllChildrenOn();

    _ivRoot->ref();
    _ivRoot->addChild(_ivXform);
    _ivXform->addChild(_ivGeom);

    _viewer->GetRoot()->addChild(_ivRoot);
}

Item::~Item()
{
    if( _ivRoot != NULL ) {
        _viewer->GetRoot()->removeChild(_ivRoot);
        _ivRoot->unref();
    }

}

bool Item::ContainsOSGNode(osg::Node *pNode)
{
    FindNode *search = new FindNode(pNode);
    search->apply(_ivGeom);

    if (search->getNode()) {
        delete search;
        return true;
    }

    delete search;
    return false;
}

void Item::SetGeomVisibility(bool bFlag)
{
    if (bFlag) {
        _ivGeom->setAllChildrenOn();
    }
    else {
        _ivGeom->setAllChildrenOff();
    }
}

/// KinBodyItem class
KinBodyItem::KinBodyItem(QtOSGViewerPtr viewer, KinBodyPtr pchain, ViewGeometry viewmode) : Item(viewer), _viewmode(viewmode)
{
    BOOST_ASSERT( !!pchain );
    _pchain = pchain;
    bGrabbed = false;
    _userdata = 0;
    _bReload = false;
    _bDrawStateChanged = false;
    networkid = pchain->GetEnvironmentId();
    _geometrycallback = pchain->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&KinBodyItem::GeometryChangedCallback,this));
    _drawcallback = pchain->RegisterChangeCallback(KinBody::Prop_LinkDraw, boost::bind(&KinBodyItem::DrawChangedCallback,this));
}


KinBodyItem::~KinBodyItem()
{
    //delete _pchain; // pointer doesn't belong to gui
    _veclinks.clear();
}

void KinBodyItem::setNamedNode(const std::string&  name, osg::Node* currNode)
{
    osg::Group* currGroup;

    // check to see if we have a valid (non-NULL) node.
    // if we do have a null node, return NULL.
    if ( !!currNode) {
        // Sets the name of the node
        currNode->setName(name);

        // We have a valid node, but not the one we are looking for.
        // Check to see if it has children (non-leaf node). If the node
        // has children, check each of the child nodes by recursive call.
        // If one of the recursive calls returns a non-null value we have
        // found the correct node, so return this node.
        // If we check all of the children and have not found the node,
        // return NULL
        currGroup = currNode->asGroup(); // returns NULL if not a group.
        if ( currGroup ) {
            for (unsigned int i = 0; i < currGroup->getNumChildren(); i++) {
                setNamedNode(name, currGroup->getChild(i));
            }
        }
    }
}

void KinBodyItem::Load()
{
    osg::Matrix mT;
    osg::Matrix mR;
    osg::Matrix mS;
    osg::Matrix mIdentity;

    osg::Group* parent;
    osg::Group* child;

    vector<KinBody::LinkPtr>::const_iterator it;

    mIdentity.makeIdentity();

    //  Sets name of Robot or Kinbody
    _ivGeom->setName(_pchain->GetName());

//  RAVELOG_DEBUG("Kinbody name=%s\n",_ivGeom->getName().c_str());
//  RAVELOG_VERBOSE("Number of links = %d\n",_pchain->GetLinks().size());

    //  Extract geometry
    FORIT(it, _pchain->GetLinks()) {
        LINK lnk;
        lnk.first = new osg::Group();
        lnk.second = new osg::MatrixTransform();

        RaveTransform<float> tbody = (*it)->GetTransform();

        RAVELOG_DEBUG_FORMAT("Link name = %s",(*it)->GetName());

//    RAVELOG_INFO("Rt  Ext: %f,%f,%f,%f\n",tbody.rot.y, tbody.rot.z,tbody.rot.w,tbody.rot.x);

        mR.makeRotate(osg::Quat(tbody.rot.y, tbody.rot.z,tbody.rot.w,tbody.rot.x));
        mT.makeTranslate(tbody.trans.x, tbody.trans.y, tbody.trans.z);

//    mR.makeRotate(osg::Quat(0.0, 0.0,0.0,1.0));
//    mT.makeTranslate(0.0, 0.0, 0.0);

        lnk.second->preMult(mT);
        lnk.second->preMult(mR);

        lnk.first->addChild(lnk.second);
        _veclinks.push_back(lnk);

        FOREACHC(itgeom, (*it)->GetGeometries()) {
            KinBody::Link::GeometryPtr orgeom = *itgeom;
            if( !orgeom->IsVisible() && _viewmode == VG_RenderOnly ) {
                continue;
            }

            osg::Group*           psep = NULL;
            osg::MatrixTransform* ptrans = new osg::MatrixTransform();
            Transform tgeom = orgeom->GetTransform();

//      RAVELOG_VERBOSE("Trn Int: %f,%f,%f\n",tgeom.trans.x, tgeom.trans.y, tgeom.trans.z);
//      RAVELOG_INFO("Rt  Int: %f,%f,%f,%f\n",tgeom.rot.y, tgeom.rot.z,tgeom.rot.w,tgeom.rot.x);

            mR.makeRotate(osg::Quat(tgeom.rot.y, tgeom.rot.z,tgeom.rot.w,tgeom.rot.x));
            mT.makeTranslate(tgeom.trans.x, tgeom.trans.y, tgeom.trans.z);

            ptrans->preMult(mT);
            ptrans->preMult(mR);

            // open
            bool bSucceeded = false;
            if( _viewmode == VG_RenderOnly || _viewmode == VG_RenderCollision ) {

                //  OpenRAVE 0.5 version
                string extension;
                if( orgeom->GetRenderFilename().find("__norenderif__:") == 0 ) {
                    string ignoreextension = orgeom->GetRenderFilename().substr(15);
                    if( ignoreextension == "wrl" || extension == "iv" || extension == "vrml" ) {
                        continue;
                    }
                }
                if( orgeom->GetRenderFilename().find_last_of('.') != string::npos ) {
                    extension = orgeom->GetRenderFilename().substr(orgeom->GetRenderFilename().find_last_of('.')+1);
                    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
                }
                if( extension == "wrl" || extension == "iv" || extension == "vrml" ) {
                    osg::Node* loadedModel;
                    osg::Matrix mRotate;

                    mRotate.makeRotate(-osg::PI/2,osg::Vec3f(1.0f,0.0f,0.0f));

                    mS.makeScale(orgeom->GetRenderScale().x, orgeom->GetRenderScale().y, orgeom->GetRenderScale().z);

                    ptrans->preMult(mS);
                    ptrans->preMult(mRotate);

                    loadedModel   = osgDB::readNodeFile(orgeom->GetRenderFilename());

                    psep          = loadedModel->asGroup();
                    osg::StateSet* state = psep->getOrCreateStateSet();
                    state->setMode(GL_RESCALE_NORMAL,osg::StateAttribute::ON);

                    bSucceeded    = true;
                    //RaveVector<float> color = orgeom->GetDiffuseColor();
                }
            }

            if( !bSucceeded || _viewmode == VG_RenderCollision ) {
                float x,y,z,w;

                osg::ref_ptr<osg::Material> mat = new osg::Material;
                float transparency = orgeom->GetTransparency();
                if( _viewmode == VG_RenderCollision && bSucceeded ) {
                    mat->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4f(0.6f,0.6f,1.0f,1.0f));
                    mat->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4f(0.4f,0.4f,1.0f,1.0f));
                    transparency = 0.5f;
                }

                // create custom
                if( psep == NULL ) {
                    psep = new osg::Group();
                }

                // set a diffuse color
                osg::StateSet* state = psep->getOrCreateStateSet();
                
                x = orgeom->GetDiffuseColor().x;
                y = orgeom->GetDiffuseColor().y;
                z = orgeom->GetDiffuseColor().z;
                w = 1.0f;

                mat->setDiffuse( osg::Material::FRONT_AND_BACK, osg::Vec4f(x,y,z,w) );

                RAVELOG_VERBOSE_FORMAT("Diffuse color= %f %f %f\n",x%y%z);

                x = orgeom->GetAmbientColor().x;
                y = orgeom->GetAmbientColor().y;
                z = orgeom->GetAmbientColor().z;
                w = 1.0f;

                mat->setAmbient( osg::Material::FRONT_AND_BACK, osg::Vec4f(x,y,z,w) );

                mat->setShininess( osg::Material::FRONT_AND_BACK, 25.0);
                mat->setEmission(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 1.0));

                // getting the object to be displayed with transparency
                if (transparency > 0) {
                    mat->setTransparency(osg::Material::FRONT_AND_BACK, transparency);

                    state->setRenderBinDetails(0, "transparent"); 
                    state->setMode(GL_BLEND, osg::StateAttribute::ON); 
                    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 
                }
                state->setAttributeAndModes(mat);

                switch(orgeom->GetType()) {
                //  Geometry is defined like a Sphere
                case GT_Sphere: {
                    osg::Sphere* s = new osg::Sphere();
                    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                    s->setRadius(orgeom->GetSphereRadius());
                    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(s);
                    geode->addDrawable(sd.get());

                    psep->addChild(geode.get());
                    break;
                }
                //  Geometry is defined like a Box
                case GT_Box: {
                    Vector v;
                    osg::Box* box = new osg::Box();
                    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

                    box->setHalfLengths(osg::Vec3f(orgeom->GetBoxExtents().x,orgeom->GetBoxExtents().y,orgeom->GetBoxExtents().z));
                    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(box);
                    geode->addDrawable(sd.get());

                    psep->addChild(geode.get());
                    break;
                }
                //  Geometry is defined like a Cylinder
                case GT_Cylinder: {
                    // make SoCylinder point towards z, not y
                    osg::Cylinder* cy = new osg::Cylinder();
                    cy->setRadius(orgeom->GetCylinderRadius());
                    cy->setHeight(orgeom->GetCylinderHeight());
                    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(cy);
                    geode->addDrawable(sd.get());
                    psep->addChild(geode.get());
                    break;
                }
                //  Extract geometry from collision Mesh
                case GT_TriMesh: {
                    // make triangleMesh
                    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

                    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

                    const TriMesh& mesh = orgeom->GetCollisionMesh();
                    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
                    geom->setVertexArray(vertices.get());


                    //RAVELOG_VERBOSE_FORMAT("Indices=%d", mesh.indices.size());

                    FOREACHC(itind, mesh.indices) {
                        RaveVector<float> v = mesh.vertices[*itind];
                        vertices->push_back(osg::Vec3(v.x,v.y,v.z));
                    }

                    //RAVELOG_VERBOSE_FORMAT("Vertices=%d",mesh.indices.size());

                    osg::ref_ptr<osg::DrawElementsUInt> geom_prim = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,0);
                    for(size_t i = 0; i < mesh.indices.size()/3; ++i) {
                        geom_prim->push_back(i);
                    }

                    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,vertices->size()));

                    //RAVELOG_VERBOSE("Calculate Normals\n");

                    //  Calculate normals and set binding
                    osg::Vec3Array  *normals;
                    normals = generateNormals((osg::Vec3Array*)geom->getVertexArray());
                    
                    //RAVELOG_VERBOSE("Normals Calculated!!!\n");

                    geom->setNormalArray(normals);
                    geom->setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

                    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                    geode->addDrawable(geom);
                    psep->addChild(geode);
                    break;
                }
                default:
                    break;
                }
            }

            if( psep != NULL ) {
                ptrans->addChild(psep);

                //  Apply external transform to local transform
                lnk.second->addChild(ptrans);

                string name;

                if ((*it)->GetName() == "") {
                    name = (*it)->GetParent()->GetName();
                }
                else {
                    name = (*it)->GetName();
                }

                //  Sets name of Link Group
                setNamedNode(name,lnk.first);

                //  Global transform
                lnk.second->setName(std::string(QTOSG_GLOBALTRANSFORM_PREFIX)+name);

                //  Local transform
                ptrans->setName(std::string(QTOSG_LOCALTRANSFORM_PREFIX)+name);
            }
        }
    }

    RAVELOG_DEBUG_FORMAT("Number of links added = %d", _veclinks.size());

    //  Is an object without joints
    if (_pchain->GetJoints().size() < 1) {
        _ivGeom->addChild(_veclinks[0].first);
    }
    //  Object with joints
    else {
        //  Assemble link hierarchy
        FOREACH(itjoint, _pchain->GetJoints()) {
            parent = _FindNodeName((*itjoint)->GetHierarchyParentLink()->GetName());
            child = _FindNodeName((*itjoint)->GetHierarchyChildLink()->GetName());
            if( !parent ) {
                RAVELOG_WARN(str(boost::format("cannot find node link %s")%(*itjoint)->GetHierarchyParentLink()->GetName()));
            }
            else if( !child ) {
                RAVELOG_WARN(str(boost::format("cannot find node link %s")%(*itjoint)->GetHierarchyChildLink()->GetName()));
            }
            else {
                parent->addChild(child);
            }
        }

        //  Assemble passive joints
        FOREACH(itjoint, _pchain->GetPassiveJoints()) {
            parent = _FindNodeName((*itjoint)->GetHierarchyParentLink()->GetName());
            child = _FindNodeName((*itjoint)->GetHierarchyChildLink()->GetName());
            if( !parent ) {
                RAVELOG_WARN(str(boost::format("cannot find node link %s")%(*itjoint)->GetHierarchyParentLink()->GetName()));
            }
            else if( !child ) {
                RAVELOG_WARN(str(boost::format("cannot find node link %s")%(*itjoint)->GetHierarchyChildLink()->GetName()));
            }
            else {
                parent->addChild(child);
            }
        }

        //  Gets the parent
        while (parent->getParents().size() > 0) {
            parent = parent->getParent(0);
        }

        _ivGeom->addChild(parent);
    }

    RAVELOG_DEBUG("Model added successfully!!!!!!\n");
    //  Print Scene Graph after creation
    //  _PrintSceneGraph("",_ivGeom);

    _bReload = false;
    _bDrawStateChanged = false;
}

osg::Group* KinBodyItem::_FindNodeName(const string& name)
{
    osg::Group* node;
    for (size_t i = 0; i < _veclinks.size(); i++) {
        node = _veclinks[i].first->asGroup();

        if (node->getName() == name) {
//      RAVELOG_DEBUG("Node '%s' found\n",name.c_str());
            return node;
        }
    }

    return NULL;
}

void KinBodyItem::_PrintMatrix(osg::Matrix& m)
{
    for (size_t i = 0; i < 4; i++) {
        RAVELOG_WARN("Line '%d'= %f %f %f %f\n",i,m(i,0),m(i,1),m(i,2),m(i,3));
    }

    //  Void line
    RAVELOG_DEBUG("\n");
}

void KinBodyItem::_PrintSceneGraph(const std::string& currLevel,osg::Node* currNode)
{
    std::string level;
    level = currLevel;

    // check to see if we have a valid (non-NULL) node.
    // if we do have a null node, return NULL.
    if ( !!currNode) {
        level = level + "-";
        RAVELOG_VERBOSE_FORMAT("|%sNode class:%s (%s)\n",currLevel%currNode->className()%currNode->getName());
        osg::ref_ptr<osg::Group> currGroup = currNode->asGroup(); // returns NULL if not a group.
        if ( !!currGroup ) {
            for (unsigned int i = 0; i < currGroup->getNumChildren(); i++) {
                _PrintSceneGraph(level,currGroup->getChild(i));
            }
        }
    }
}

void KinBodyItem::_PrintNodeFeatures(osg::Node *node)
{
//  RAVELOG_VERBOSE("----->>>> printNodeFeatures(node)\n");
//  osg::StateSet* state;
//  osg::Material* mat;
//  osg::Light  *light;
//  osg::Geode  *geode;


//  for (size_t i = 0; i < node->asGroup()->getNumChildren(); i++)
//  {
//    geode = node->asGroup()->getChild(i)->asGroup()->getChild(0)->asGeode();
//    state = geode->getDrawable(0)->getOrCreateStateSet();
//    RAVELOG_VERBOSE("Number of Attributes = %d\n",state->getAttributeList().size());
//    RAVELOG_VERBOSE("Number of Modes = %d\n",state->getModeList().size());

//    mat = (osg::Material*)state->getAttribute(osg::StateAttribute::MATERIAL);
//    if (!!mat)
//    {
//      RAVELOG_INFO("SHININESS= %f\n",mat->getShininess(osg::Material::FRONT));
//      RAVELOG_INFO("Color Mode = %d\n",mat->getColorMode());
//    }
//    if (!!state)
//    {
//      RAVELOG_DEBUG("GEODE has StateSet\n");
//      for (osg::StateSet::AttributeList::iterator it = state->getAttributeList().begin();
//          it != state->getAttributeList().end();
//          it++)
//      {
//        RAVELOG_VERBOSE("Attribute Type: %d\n",(*it).first.first);
//      }
//      for (osg::StateSet::ModeList::iterator it = state->getModeList().begin();
//          it != state->getModeList().end();
//          it++)
//      {
//
//      }
//    }
//    else
//    {
//      RAVELOG_DEBUG("GEODE does NOT have StateSet\n");
//    }
//  }
}

////////////////////////////////////////////////////////////////////////////////
/// Generate normals
////////////////////////////////////////////////////////////////////////////////
osg::Vec3Array* KinBodyItem::generateNormals(osg::Vec3Array *vertices)
{
    osg::Vec3Array *normals = new osg::Vec3Array;

    if (!vertices) {
        return NULL;
    }
    dReal f;
    /*
     * Calculate per-face normals from face vertices.
     */
    unsigned int fi = 0;
    while (fi < vertices->size()) {
        // Edge vectors
        Vector e0;
        e0.x = (*vertices)[fi+1].x() - (*vertices)[fi].x();
        e0.y = (*vertices)[fi+1].y() - (*vertices)[fi].y();
        e0.z = (*vertices)[fi+1].z() - (*vertices)[fi].z();

        Vector e1;
        e1.x = (*vertices)[fi+2].x() - (*vertices)[fi].x();
        e1.y = (*vertices)[fi+2].y() - (*vertices)[fi].y();
        e1.z = (*vertices)[fi+2].z() - (*vertices)[fi].z();

        // Cross-product of e0,e1
        Vector normal;
        normal.x = e0.y * e1.z - e0.z * e1.y;
        normal.y = e0.z * e1.x - e0.x * e1.z;
        normal.z = e0.x * e1.y - e0.y * e1.x;

        f = normal.x*normal.x+normal.y*normal.y+normal.z*normal.z;
        f = RaveSqrt(f);

        normal.x /= f;
        normal.y /= f;
        normal.z /= f;

//    normal.normalize3();

        // Add to per-face normals
        normals->push_back(osg::Vec3(normal.x,normal.y,normal.z));

        fi = fi + 3;
    }

    return normals;
}

void KinBodyItem::GeometryChangedCallback()
{
    _bReload = true;
}

void KinBodyItem::DrawChangedCallback()
{
    _bDrawStateChanged = true;
}

bool KinBodyItem::UpdateFromIv()
{
    //osg::Matrix m;
    if( _pchain == NULL ) {
        return false;
    }
    vector<Transform> vtrans(_veclinks.size());
    Transform tglob = GetRaveTransform(_ivXform);

    vector<Transform>::iterator ittrans = vtrans.begin();
    FOREACH(it, _veclinks) {
        *ittrans = GetRaveTransform(it->second);
        //*ittrans = tglob * *ittrans;
        //m = it->second->getMatrix();
        ++ittrans;
    }

    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv = _viewer->LockEnvironment(50000,false);
    if( !!lockenv ) {
        _pchain->SetLinkTransformations(vtrans,_vdofbranches);
    }
    else {
        RAVELOG_WARN("failed to acquire environment lock for updating body (viewer updates might be choppy, otherwise this does not affect internal openrave state)\n");
    }
    return true;
}

void KinBodyItem::GetDOFValues(vector<dReal>& vjoints) const
{
    boost::mutex::scoped_lock lock(_mutexjoints);
    vjoints = _vjointvalues;
}

void KinBodyItem::GetLinkTransformations(vector<Transform>& vtrans, std::vector<dReal>& vdofbranches) const
{
    boost::mutex::scoped_lock lock(_mutexjoints);
    vtrans = _vtrans;
    vdofbranches = _vdofbranches;
}

bool KinBodyItem::UpdateFromModel()
{
    if( !_pchain ) {
        return false;
    }
    vector<Transform> vtrans;
    vector<dReal> vjointvalues;

    {
        boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv = _viewer->LockEnvironment(50000,false);
        if( !lockenv ) {
            return false;
        }
        if( _bReload || _bDrawStateChanged ) {
            Load();
        }
        // make sure the body is still present!
        if( _pchain->GetEnv()->GetBodyFromEnvironmentId(networkid) == _pchain ) {
            _pchain->GetLinkTransformations(_vtrans, _vdofbranches);
            _pchain->GetDOFValues(vjointvalues);
        }
        else {
            _pchain.reset();
        }
    }

    return UpdateFromModel(vjointvalues,vtrans);
}

bool KinBodyItem::UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans)
{
    osg::MatrixTransform* mtransform;

    if(_pchain == NULL ) {
        // don't update, physics is disabled anyway
        return false;
    }

    _vjointvalues = vjointvalues;
    _vtrans = vtrans;

    if( _vtrans.size() == 0 || _veclinks.size() != _vtrans.size() )
        // something's wrong, so just return
        return false;

    //  Global transform
    Transform tglob = _vtrans.at(0); //_pchain->GetCenterOfMass();

    //  Matrices for intermediate calculations
    osg::Matrix m;
    osg::Matrix mT,mR,mS;

    //  Link iterator
    vector<LINK>::iterator it = _veclinks.begin();

    //  Vector of transformations
    FOREACHC(ittrans, _vtrans) {

        osg::ref_ptr<osg::MatrixTransform> aux = new osg::MatrixTransform();
        SetMatrixTransform(aux,*ittrans);

        //  Local transform
        Transform tlocal = *ittrans;

        //  New matrix transform for intermediate calculations
        osg::ref_ptr<osg::MatrixTransform> ptrans = new osg::MatrixTransform();
        SetMatrixTransform(ptrans,tlocal);

        //  Error control
        if (it->second->getNumChildren() == 0) {
            return false;
        }

        //  Gets matrix transform of child transform
        mtransform = it->second->asTransform()->asMatrixTransform();

        m = ptrans->getMatrix();

        // TODO :  Modifies matrix of link
        mtransform->setMatrix(m);

        ++it;
    }

    return true;
}

void KinBodyItem::SetGrab(bool bGrab, bool bUpdate)
{
    if(!_pchain ) {
        return;
    }
    
//    // need to preserve enabled state
//    if( bGrab && !bGrabbed ) {
//        bEnabled = _pchain->IsEnabled();
//    }
    
    bGrabbed = bGrab;

    if( bUpdate ) {
        if( bGrab ) {
            UpdateFromModel();
        }
        else {
            UpdateFromIv();
        }
    }

//    // need to preserve enabled state
//    if( bEnabled ) {
//        _pchain->Enable(!bGrab);
//    }
}

KinBody::LinkPtr KinBodyItem::GetLinkFromIv(osg::Node* plinknode) const
{
    vector<LINK>::const_iterator it;
    vector<KinBody::LinkPtr>::const_iterator itlink = _pchain->GetLinks().begin();
    FindNode search(plinknode);
    FORIT(it, _veclinks) {
        search.apply(it->first);
        if (search.getNode()) {
            return *itlink;
        }

        itlink++;
    }
    
    return KinBody::LinkPtr();
}

RobotItem::RobotItem(QtOSGViewerPtr viewer, RobotBasePtr robot, ViewGeometry viewgeom) : KinBodyItem(viewer, robot, viewgeom)
{
    int index = 0;
    FOREACHC(itmanip, robot->GetManipulators()) {

        if((*itmanip)->GetEndEffector()) {
            osg::Switch* peeswitch = new osg::Switch();
            osg::Group* peesep = new osg::Group();
            osg::MatrixTransform* ptrans = new osg::MatrixTransform();
            _vEndEffectors.push_back(EE(index, ptrans, peeswitch));

            _ivGeom->addChild(peeswitch);
            peeswitch->addChild(ptrans);
            peeswitch->setAllChildrenOff();
            ptrans->addChild(peesep);

            // set a diffuse color
            {
                osg::StateSet* state = peesep->getOrCreateStateSet();
                osg::ref_ptr<osg::Material> mat = new osg::Material;

                mat->setDiffuse( osg::Material::FRONT, osg::Vec4f(1,0.5,0.5,1) );
                mat->setAmbient( osg::Material::FRONT, osg::Vec4f(1,0.5,0.5,1));

                state->setAttribute(mat.get());

                osg::Sphere* sphere = new osg::Sphere();
                osg::Geode* geode = new osg::Geode;
                sphere->setRadius(0.004f);
                osg::ShapeDrawable* sd = new osg::ShapeDrawable(sphere);
                geode->addDrawable(sd);
                peesep->addChild(geode);
            }

            // add some axes
            osg::Group* paxes = new osg::Group();

            Vector colors[] = {Vector(0,0,1),Vector(0,1,0),Vector(1,0,0)};
            Vector rotations[] = {Vector(1,0,0,M_PI/2), Vector(1,0,0,0), Vector(0,0,1,-M_PI/2)};

            // add 3 cylinder+cone axes
            for(int i = 0; i < 3; ++i) {
                // set a diffuse color
                osg::Group* psep = new osg::Group();

                // set a diffuse color
                osg::StateSet* state = psep->getOrCreateStateSet();
                osg::Material* mat = new osg::Material;
                mat->setDiffuse(osg::Material::FRONT,
                                osg::Vec4f(colors[i].x, colors[i].y, colors[i].z,1.0));
                mat->setAmbient(osg::Material::FRONT,
                                osg::Vec4f(colors[i].x, colors[i].y, colors[i].z,1.0));

                state->setAttribute( mat );

                osg::Matrix matrix;
                osg::MatrixTransform* protation = new osg::MatrixTransform();
                matrix.makeRotate(osg::Quat(rotations[i].x,rotations[i].y,rotations[i].z,rotations[i].w));
                protation->setMatrix(matrix);

                matrix.makeIdentity();
                osg::MatrixTransform* pcyltrans = new osg::MatrixTransform();
                matrix.makeTranslate(0.0f,0.02f,0.0f);
                pcyltrans->setMatrix(matrix);

                // make SoCylinder point towards z, not y
                osg::Cylinder* cy = new osg::Cylinder();
                cy->setRadius(0.002f);
                cy->setHeight(0.04f);
                osg::ref_ptr<osg::Geode> gcyl = new osg::Geode;
                osg::ref_ptr<osg::ShapeDrawable> sdcyl = new osg::ShapeDrawable(cy);
                gcyl->addDrawable(sdcyl.get());

                osg::Cone* cone = new osg::Cone();
                cone->setRadius(0.004f);
                cone->setHeight(0.02f);

                osg::ref_ptr<osg::Geode> gcone = new osg::Geode;
                osg::ref_ptr<osg::ShapeDrawable> sdcone = new osg::ShapeDrawable(cone);

                matrix.makeIdentity();
                osg::MatrixTransform* pconetrans = new osg::MatrixTransform();
                matrix.setTrans(osg::Vec3f(0,0.02f,0));
                pconetrans->setMatrix(matrix);

                psep->addChild(protation);
                psep->addChild(pcyltrans);
                psep->addChild(gcyl.get());
                psep->addChild(pconetrans);
                psep->addChild(gcone.get());
                paxes->addChild(psep);
            }

            peesep->addChild(paxes);

            // add text
            {
                osg::Group* ptextsep = new osg::Group();
                osg::Geode* textGeode = new osg::Geode;
                peesep->addChild(ptextsep);

                osg::Matrix matrix;
                osg::MatrixTransform* ptrans = new osg::MatrixTransform();
                ptrans->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
                matrix.setTrans(osg::Vec3f(0.02f,0.02f,0.02f));
                ptextsep->addChild(ptrans);

                osgText::Text* text = new osgText::Text();

                //Set the screen alignment - always face the screen
                text->setAxisAlignment(osgText::Text::SCREEN);

                text->setColor(osg::Vec4(0,0,0,1));
                text->setFontResolution(18,18);

                char str[256];
                sprintf(str,"EE%d", index);
                text->setText(str);
                textGeode->addDrawable(text);
                ptextsep->addChild(textGeode);
            }
        }

        ++index;
    }
}

void RobotItem::SetGrab(bool bGrab, bool bUpdate)
{
    if( !_probot ) {
        return;
    }
    if( bGrab ) {
        // turn off any controller commands if a robot
        if( !!_probot->GetController() ) {
            _probot->GetController()->SetPath(TrajectoryBaseConstPtr());
        }
    }

    FOREACH(itee, _vEndEffectors) {
        if( !!itee->_pswitch ) {
            itee->_pswitch->setAllChildrenOn();
        }
        else {
            itee->_pswitch->setAllChildrenOff();
        }
    }
    FOREACH(itee, _vAttachedSensors) {
        if( !!itee->_pswitch ) {
            itee->_pswitch->setAllChildrenOn();
        }
        else {
            itee->_pswitch->setAllChildrenOff();
        }
    }

    KinBodyItem::SetGrab(bGrab, bUpdate);
}

bool RobotItem::UpdateFromIv()
{
    if( !KinBodyItem::UpdateFromIv() ) {
        return false;
    }
    return true;
}

bool RobotItem::UpdateFromModel(const vector<dReal>& vjointvalues, const vector<Transform>& vtrans)
{
    if( !KinBodyItem::UpdateFromModel(vjointvalues,vtrans) ) {
        return false;
    }
    if( bGrabbed ) {
        // only updated when grabbing!
        RaveTransform<float> transInvRoot = GetRaveTransform(_ivXform).inverse();

        FOREACH(itee, _vEndEffectors) {
            if( itee->_index >= 0 && itee->_index < (int)_probot->GetManipulators().size()) {
                RobotBase::ManipulatorConstPtr manip = _probot->GetManipulators().at(itee->_index);
                if( !!manip->GetEndEffector() ) {
                    RaveTransform<float> tgrasp = vtrans.at(manip->GetEndEffector()->GetIndex())*manip->GetLocalToolTransform();
                    SetMatrixTransform(itee->_ptrans, transInvRoot * tgrasp);
                }
            }
        }

        FOREACH(itee, _vAttachedSensors) {
            if( itee->_index >= 0 && itee->_index < (int)_probot->GetAttachedSensors().size()) {
                RobotBase::AttachedSensorConstPtr sensor = _probot->GetAttachedSensors().at(itee->_index);
                if( !!sensor->GetAttachingLink() ) {
                    RaveTransform<float> tgrasp = vtrans.at(sensor->GetAttachingLink()->GetIndex())*sensor->GetRelativeTransform();
                    SetMatrixTransform(itee->_ptrans, transInvRoot * tgrasp);
                }
            }
        }
    }

    return true;
}

}
