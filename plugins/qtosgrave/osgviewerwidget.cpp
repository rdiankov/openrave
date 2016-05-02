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
#include "osgviewerwidget.h"

namespace qtosgrave {

// \ brief rigid transformation dragger (does not allow scale)
class DualDraggerTransformCallback : public osgManipulator::DraggerCallback
{
public:
    DualDraggerTransformCallback(osg::MatrixTransform* sourcetransform, osg::MatrixTransform* updatetransform) : osgManipulator::DraggerCallback(), _sourcetransform(sourcetransform), _updatetransform(updatetransform) {
    }

    virtual bool receive(const osgManipulator::MotionCommand& command) {
        if( !!_sourcetransform && !!_updatetransform ) {
            if( command.getStage() == osgManipulator::MotionCommand::FINISH || command.getStage() == osgManipulator::MotionCommand::MOVE) {
                _updatetransform->setMatrix(_sourcetransform->getMatrix());
                return true;
            }
        }

        return false;
    }

protected:
    osg::observer_ptr<osg::MatrixTransform> _sourcetransform, _updatetransform;

};

class OpenRAVEKeyboardEventHandler : public osgGA::GUIEventHandler
{
public:
    OpenRAVEKeyboardEventHandler(const boost::function<bool(int)>& onKeyDown) : _onKeyDown(onKeyDown) {
    }

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
    {
        switch(ea.getEventType())
        {
        case (osgGA::GUIEventAdapter::KEYDOWN): {
            return _onKeyDown(ea.getKey());
        }
        default:
            return false;
        }
    }

    // only for osg3.0?
//    virtual void accept(osgGA::GUIEventHandlerVisitor& v)   {
//        v.visit(*this);
//    }

private:
    boost::function<bool(int)> _onKeyDown; ///< called when key is pressed
};

ViewerWidget::ViewerWidget(EnvironmentBasePtr penv, const std::string& userdatakey, const boost::function<bool(int)>& onKeyDown) : QWidget(), _onKeyDown(onKeyDown)
{
    setKeyEventSetsDone(0); // disable Escape key from killing the viewer!

    _userdatakey = userdatakey;
    _penv = penv;
    _bLightOn = true;
    _actualKinbody = "";
    _osgview = new osgViewer::View();
    _osghudview = new osgViewer::View();

    //  Improve FPS to 60 per viewer
    setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);

    QWidget* widgetview = _AddViewWidget(_CreateCamera(0,0,100,100), _osgview, _CreateHUDCamera(0,0,100,100), _osghudview);
    QGridLayout* grid = new QGridLayout;
    grid->addWidget(widgetview, 0, 0);
    grid->setContentsMargins(1, 1, 1, 1);
    setLayout(grid);

    //  Sets pickhandler
    _picker = new OSGPickHandler(boost::bind(&ViewerWidget::SelectLink, this, _1, _2));
    _osgview->addEventHandler(_picker);

    _keyhandler = new OpenRAVEKeyboardEventHandler(boost::bind(&ViewerWidget::HandleOSGKeyDown, this, _1));
    _osgview->addEventHandler(_keyhandler);

    // initialize the environment
    _osgSceneRoot = new osg::Group();    
    _osgFigureRoot = new osg::Group();
  
    // create world axis
    _osgWorldAxis = new osg::MatrixTransform();
    //_osgWorldAxis->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE );
    
    {
        osg::Vec4f colors[] = {
            osg::Vec4f(0,0,1,1),
            osg::Vec4f(0,1,0,1),
            osg::Vec4f(1,0,0,1)
        };
        osg::Quat rotations[] = {
            osg::Quat(0, osg::Vec3f(0,0,1)),
            osg::Quat(-M_PI/2.0, osg::Vec3f(1,0,0)),
            osg::Quat(M_PI/2.0, osg::Vec3f(0,1,0))
        };

        // add 3 cylinder+cone axes
        for(int i = 0; i < 3; ++i) {
            osg::MatrixTransform* psep = new osg::MatrixTransform();
            psep->setMatrix(osg::Matrix::translate(-16.0f,-16.0f,-16.0f));

            // set a diffuse color
            osg::StateSet* state = psep->getOrCreateStateSet();
            osg::Material* mat = new osg::Material;
            mat->setDiffuse(osg::Material::FRONT, colors[i]);
            mat->setAmbient(osg::Material::FRONT, colors[i]);
            state->setAttribute( mat );

            osg::Matrix matrix;
            osg::MatrixTransform* protation = new osg::MatrixTransform();
            matrix.makeRotate(rotations[i]);
            protation->setMatrix(matrix);

            matrix.makeIdentity();
            osg::MatrixTransform* pcyltrans = new osg::MatrixTransform();
            matrix.setTrans(osg::Vec3f(0,0,16.0f));
            pcyltrans->setMatrix(matrix);

            // make SoCylinder point towards z, not y
            osg::Cylinder* cy = new osg::Cylinder();
            cy->setRadius(2.0f);
            cy->setHeight(32.0f);
            osg::ref_ptr<osg::Geode> gcyl = new osg::Geode;
            osg::ref_ptr<osg::ShapeDrawable> sdcyl = new osg::ShapeDrawable(cy);
            gcyl->addDrawable(sdcyl.get());

            osg::Cone* cone = new osg::Cone();
            cone->setRadius(4.0f);
            cone->setHeight(16.0f);

            osg::ref_ptr<osg::Geode> gcone = new osg::Geode;
            osg::ref_ptr<osg::ShapeDrawable> sdcone = new osg::ShapeDrawable(cone);
            gcone->addDrawable(sdcone.get());

            matrix.makeIdentity();
            osg::MatrixTransform* pconetrans = new osg::MatrixTransform();
            matrix.setTrans(osg::Vec3f(0,0,32.0f));
            pconetrans->setMatrix(matrix);

            psep->addChild(protation);
            protation->addChild(pcyltrans);
            pcyltrans->addChild(gcyl.get());
            protation->addChild(pconetrans);
            pconetrans->addChild(gcone.get());
            _osgWorldAxis->addChild(psep);
        }
    }

    if( !!_osgCameraHUD ) {
        // in order to get the axes to render without lighting:
        
        osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();        
        osg::ref_ptr<osg::Light> light(new osg::Light());
        // each light must have a unique number
        light->setLightNum(0);
        // we set the light's position via a PositionAttitudeTransform object
        light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
        light->setDiffuse(osg::Vec4(0, 0, 0, 1.0));
        light->setSpecular(osg::Vec4(0, 0, 0, 1.0));
        light->setAmbient( osg::Vec4(1, 1, 1, 1.0));
        lightSource->setLight(light.get());
        
        _osgCameraHUD->addChild(lightSource.get());
        lightSource->addChild(_osgWorldAxis.get());
    }

    _InitializeLights(2);

    _osgLightsGroup->addChild(_osgSceneRoot);
    //osg::ref_ptr<osgFX::Cartoon> toon = new osgFX::Cartoon();
    //_osgLightsGroup->addChild(toon);
    //toon->addChild(_osgSceneRoot);
        
    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()) );
    _timer.start( 10 );
}

bool ViewerWidget::HandleOSGKeyDown(int key)
{
    if( !!_onKeyDown ) {
        if( _onKeyDown(key) ) {
            return true;
        }
    }
//    switch(key) {
//    case osgGA::GUIEventAdapter::KEY_Escape:
//
//        _picker->ActivateSelection(!_picker->IsSelectionActive());
//        if( !_picker->IsSelectionActive() ) {
//            // have to clear any draggers if selection is not active
//            _ClearDragger();
//            _draggerName.clear();
//        }
//        return true;
//    }
    return false;
}

void ViewerWidget::SelectActive(bool active)
{
    _picker->ActivateSelection(active);
}

void ViewerWidget::SetDraggerMode(const std::string& draggerName)
{
    if( draggerName.size() > 0 ) {
        _draggerName = draggerName;
        SelectRobot(_actualKinbody);
    }
    else {
        _ClearDragger();
        _draggerName.clear();
    }
}

//void ViewerWidget::DrawTrackball(bool pressed)
//{
//    //  If boundingbox button is pressed
//    if (pressed) {
//        _draggerName = "TrackballDragger";
//        SelectRobot(_actualKinbody);
//    }
//    else {
//        // remove any old dragger
//        _ClearDragger();
//        _draggerName.clear();
//    }
//}
//
//void ViewerWidget::DrawAxes(bool pressed)
//{
//    if (pressed) {
//        _draggerName = "TranslateAxisDragger";
//        SelectRobot(_actualKinbody);
//    }
//    else {
//        // remove any old dragger
//        _ClearDragger();
//        _draggerName.clear();
//    }
//}

void ViewerWidget::SelectRobot(std::string name)
{
    //  Gets camera transform
    OSGNodePtr node = _FindNamedNode(name,OSGNodePtr(_osgview->getSceneData()));

    if (!!node) {
        _PropagateTransforms();

        //  Sets robot selected
        _actualKinbody = name;

        RAVELOG_DEBUG_FORMAT("Node name %s", node->getName());
        _AddDraggerToObject(node,_draggerName);
    }
}

void ViewerWidget::SetSceneData()
{
    OSGGroupPtr rootscene(new osg::Group());
    //  Normalize object normals
    rootscene->getOrCreateStateSet()->setMode(GL_NORMALIZE,osg::StateAttribute::ON);
    rootscene->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
 
    if (_bLightOn) {
        rootscene->addChild(_osgLightsGroup);
    }
    else {
        rootscene->addChild(_osgSceneRoot);
    }
    rootscene->addChild(_osgFigureRoot);
    _osgview->setSceneData(rootscene.get());
}

void ViewerWidget::ResetViewToHome()
{
    _osgview->home();
}

void ViewerWidget::SetHome()
{
    if (!!_osgLightsGroup) {
        const osg::BoundingSphere& bs = _osgLightsGroup->getBound();
        _osgview->getCameraManipulator()->setHomePosition(osg::Vec3d(-4.0*bs.radius(),4.0*bs.radius(),0.0),bs.center(),osg::Vec3d(0.0,0.0,1.0));
        _osgview->home();
    }
}

void ViewerWidget::SetLight(bool enabled)
{
    _bLightOn = enabled;
    SetSceneData();
}

void ViewerWidget::SetFacesMode(bool enabled)
{
    if( !_osgview->getSceneData() ) {
        return;
    }
    osg::ref_ptr<osg::StateSet> stateset = _osgview->getSceneData()->getOrCreateStateSet();
    if (enabled)
    {
        stateset->setAttribute(new osg::CullFace(osg::CullFace::FRONT));
        stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
        stateset->setAttributeAndModes(new osg::CullFace, osg::StateAttribute::OFF);
    }
    else
    {
        stateset->setAttribute(new osg::CullFace(osg::CullFace::FRONT));
        stateset->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
        stateset->setAttributeAndModes(new osg::CullFace, osg::StateAttribute::ON);
    }

    _osgview->getSceneData()->setStateSet(stateset);
}

void ViewerWidget::SetPolygonMode(int mode)
{
    osg::ref_ptr<osg::PolygonMode> poly(new osg::PolygonMode());
    osg::ref_ptr<osg::ShadeModel> sm(new osg::ShadeModel());
    switch (mode)
    {
    case 0:
        poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::FILL);
        sm->setMode(osg::ShadeModel::SMOOTH);
        _osgview->getSceneData()->getOrCreateStateSet()->setAttribute(poly);
        _osgview->getSceneData()->getOrCreateStateSet()->setAttribute(sm);
        break;
    case 1:
        poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::FILL);
        sm->setMode(osg::ShadeModel::FLAT);
        _osgview->getSceneData()->getOrCreateStateSet()->setAttributeAndModes(poly,osg::StateAttribute::ON);
        _osgview->getSceneData()->getOrCreateStateSet()->setAttribute(sm);
        break;
    case 2:
        poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
        sm->setMode(osg::ShadeModel::SMOOTH);
        _osgview->getSceneData()->getOrCreateStateSet()->setAttribute(poly);
        _osgview->getSceneData()->getOrCreateStateSet()->setAttribute(sm);
        break;
    }
}

void ViewerWidget::SetWire(OSGNodePtr node)
{
    osg::ref_ptr<osg::PolygonMode> poly(new osg::PolygonMode());
    osg::ref_ptr<osg::ShadeModel> sm(new osg::ShadeModel());

    poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
    sm->setMode(osg::ShadeModel::SMOOTH);

    node->getOrCreateStateSet()->setAttribute(poly.get());
    node->getOrCreateStateSet()->setAttribute(sm.get());
}

OSGMatrixTransformPtr ViewerWidget::GetLinkTransform(std::string& robotName, KinBody::LinkPtr link)
{
    OSGNodePtr robot = _FindNamedNode(robotName,_osgview->getSceneData());
    OSGNodePtr node = _FindNamedNode(QTOSG_GLOBALTRANSFORM_PREFIX+link->GetName(),robot);

    OSGMatrixTransformPtr transform;
    if (!!node) {
        transform = OSGMatrixTransformPtr(node->asTransform()->asMatrixTransform());
    }

    return transform;
}

void ViewerWidget::SelectLink(OSGNodePtr node, int modkeymask)
{
    if (!node) {
        if( !(modkeymask & osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) ) {
            // user clicked on empty region, so remove selection
            _ClearDragger();
        }
        return;
    }

    string linkName;
    string robotName;
    OSGNodePtr scene;
    OSGNodePtr node_found;
    OSGNodePtr selected;
    KinBody::JointPtr joint;

    linkName = node->getName();

    OSGNodePtr robot = _FindRobot(node);
    if (!!robot) {
        robotName = robot->getName();
        RAVELOG_VERBOSE_FORMAT("found %s", robotName);

        if( (modkeymask & osgGA::GUIEventAdapter::MODKEY_ALT) ) {
            //RAVELOG_INFO("setting camera manipulator tracking node\n");
            //_osgCameraManipulator->setTrackNode(robot);
            _osgCameraManipulator->setNode(robot.get());
            _osgCameraManipulator->computeHomePosition();
            _osgCameraManipulator->home(2);
        }

        //  Gets camera transform
        _StoreMatrixTransform(); // TODO store matrix for later canceling of the move

        //  Copy scene node for modify it
        scene = _osgview->getSceneData();

        // Find joint of a link name given
        joint = _FindJoint(robotName,linkName);

        node_found = _FindNamedNode(QTOSG_GLOBALTRANSFORM_PREFIX+linkName,robot);
        if( !!node_found ) {
            node_found = node_found->getParents().at(0);
            if (!!node_found ) {
                if( (modkeymask & osgGA::GUIEventAdapter::MODKEY_CTRL) ) {
//                    if( _draggerName == "RotateCylinderDragger" ) {
//                        if( !!joint) {
//                            selected = _AddDraggerToObject(robotName, node_found, "RotateCylinderDragger", joint);
//                        }
//                    }
                }
                else {
                    if( _draggerName == "RotateCylinderDragger" ) {
                        if( !!joint) {
                            selected = _AddDraggerToObject(robotName, node_found, "RotateCylinderDragger", joint);
                        }
                    }
                    else {
                        // select new body
                        SelectRobot(robotName);
                    }
                }
            }
        }
    }

    if( !robot || !node_found ) {
        if( !(modkeymask & osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) ) {
            // user clicked on empty region, so remove selection
            _ClearDragger();
        }
    }
}

void ViewerWidget::_ClearDragger()
{
    FOREACH(itdragger, _draggers) {
        if (!!*itdragger) {
            (*itdragger)->getParents().at(0)->removeChild(*itdragger);
        }
    }
    _draggers.clear();
}

void ViewerWidget::SetViewport(int width, int height)
{
    _osgview->getCamera()->setViewport(0,0,width,height);
    _osghudview->getCamera()->setViewport(0,0,width,height);
    _osghudview->getCamera()->setProjectionMatrix(osg::Matrix::ortho(-width/2, width/2, -height/2, height/2, 0, 1000));

    osg::Matrix m = GetCameraManipulator()->getMatrix();
    m.setTrans(width/2 - 40, -height/2 + 40, -50);
    _osgWorldAxis->setMatrix(m);
}

QWidget* ViewerWidget::_AddViewWidget( osg::ref_ptr<osg::Camera> camera, osg::ref_ptr<osgViewer::View> view, osg::ref_ptr<osg::Camera> hudcamera, osg::ref_ptr<osgViewer::View> hudview )
{
    view->setCamera( camera.get() );
    hudview->setCamera( hudcamera.get() );
    addView( view.get() );
    addView( hudview.get() );

    //view->addEventHandler( new osgViewer::StatsHandler );

    _osgCameraManipulator = new osgGA::TrackballManipulator();//NodeTrackerManipulator();//TrackballManipulator();
    view->setCameraManipulator( _osgCameraManipulator.get() );

    _osgCameraHUD = new osg::MatrixTransform();
    hudcamera->addChild( _osgCameraHUD.get() );
    _osgCameraHUD->setMatrix(osg::Matrix::identity());

    GraphicsWindowQt* gw = dynamic_cast<GraphicsWindowQt*>( camera->getGraphicsContext() );
    hudcamera->setGraphicsContext(gw);
    hudcamera->setViewport(0,0,gw->getTraits()->width, gw->getTraits()->height);
    return gw ? gw->getGraphWidget() : NULL;
}

osg::ref_ptr<osg::Camera> ViewerWidget::_CreateCamera( int x, int y, int w, int h)
{
    osg::ref_ptr<osg::DisplaySettings> ds = osg::DisplaySettings::instance();

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = "";
    traits->windowDecoration = false;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();

    osg::ref_ptr<osg::Camera> camera(new osg::Camera());
    camera->setGraphicsContext(new GraphicsWindowQt(traits.get()));

    camera->setClearColor(osg::Vec4(0.95, 0.95, 0.95, 1.0));
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    camera->setProjectionMatrixAsPerspective(30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );

    return camera;
}

osg::ref_ptr<osg::Camera> ViewerWidget::_CreateHUDCamera( int x, int y, int w, int h)
{
    osg::ref_ptr<osg::Camera> camera(new osg::Camera());
    camera->setProjectionMatrix(osg::Matrix::ortho(-1,1,-1,1,1,10));

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::POST_RENDER);

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // we don't want the camera to grab event focus from the viewers main camera(s).
    camera->setAllowEventFocus(false);

    // set the view matrix
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    return camera;
}

OSGNodePtr ViewerWidget::_FindNamedNode(const std::string& searchName, OSGNodePtr currNode)
{
    OSGGroupPtr currGroup;
    OSGNodePtr foundNode;

    // check to see if we have a valid (non-NULL) node.
    // if we do have a null node, return NULL.
    if ( !currNode) {
        return OSGNodePtr();
    }

    // We have a valid node, check to see if this is the node we
    // are looking for. If so, return the current node.
    if (currNode->getName() == searchName) {
        return currNode;
    }

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
            foundNode = _FindNamedNode(searchName, OSGNodePtr(currGroup->getChild(i)));
            if (!!foundNode) {
                return foundNode; // found a match!
            }
        }
        // We have checked each child node - no match found.
    }
    else {
        // leaf node, no match
    }
    return OSGNodePtr();
}

//void ViewerWidget::_ShowSceneGraph(const std::string& currLevel,OSGNodePtr currNode)
//{
//    std::string level;
//    OSGGroupPtr currGroup;
//
//    level = currLevel;
//
//    // check to see if we have a valid (non-NULL) node.
//    // if we do have a null node, return NULL.
//    if ( !!currNode) {
//        RAVELOG_WARN_FORMAT("|%sNode class:%s (%s)",currLevel%currNode->className()%currNode->getName());
//        level = level + "-";
//        currGroup = currNode->asGroup(); // returns NULL if not a group.
//        if ( currGroup ) {
//            for (unsigned int i = 0; i < currGroup->getNumChildren(); i++) {
//                _ShowSceneGraph(level,currGroup->getChild(i));
//            }
//        }
//    }
//}

void ViewerWidget::_GetLinkChildren( std::string & robotName, KinBody::LinkPtr link, std::vector<KinBody::LinkPtr> vlinks)
{
    OSGNodePtr robot = _FindNamedNode(robotName,_osgview->getSceneData());
    OSGNodePtr transform = _FindNamedNode(QTOSG_GLOBALTRANSFORM_PREFIX+link->GetName(),robot);
    _linkChildren.push_back(transform->asTransform()->asMatrixTransform());
    FOREACH(itlink,vlinks) {
        if ((*itlink)->IsParentLink(link)) {
            _GetLinkChildren(robotName,(*itlink),vlinks);
        }
    }
}

OSGNodePtr ViewerWidget::_FindRobot(OSGNodePtr node)
{
    if (!node) {
        //  Error robot not found
        RAVELOG_WARN("robot not found!\n");
        return OSGNodePtr();
    }

    if (string(node->className()) == "Switch" && node->getName().size() > 0) {
        return node; // found
    }
    else {
        if (string(node->className()) == "Geode") {
            //  Search robot in parent node
            if( node->asGeode()->getParents().size() > 0 ) {
                return _FindRobot(OSGNodePtr(node->asGeode()->getParents().at(0)));
            }
        }
        else {
            //  Search robot in parent node
            if( node->asGroup()->getParents().size() > 0 ) {
                return _FindRobot(OSGNodePtr(node->asGroup()->getParents().at(0)));
            }
        }
    }

    return OSGNodePtr();
}

OSGNodePtr ViewerWidget::_FindLinkParent(OSGNodePtr node)
{
    //  There is an error?
    if (!node) {
        return OSGNodePtr();
    }
    if( node->getParents().size() == 0 ) {
        return OSGNodePtr();
    }

    if (string(node->className()) == string(node->getParents().at(0)->className()) &&  string(node->className()) == string("Group")) {
        //  Node found
        return node;
    }
    else {
        //  Continue searching for parent
        return _FindLinkParent(OSGNodePtr(node->getParents().at(0)));
    }
}

void ViewerWidget::_PropagateTransforms()
{
    OSGMatrixTransformPtr tglobal;
    osg::Matrix mR,mL;

    if (_linkChildren.size() == 0) {
        return;
    }
    if (!_osgDraggerRoot) {
        //  Clears childrens of link
        _linkChildren.clear();
        return;
    }

    // Get robot
    OSGNodePtr robot = _FindRobot(_selected);

    //  Gets parent of _osgDraggerRoot
    OSGGroupPtr parent(_osgDraggerRoot->getParents().at(0));

    //  Restore parent of selected link
    parent->addChild(_selected.get());

    //  Clears object selection
    _draggerMatrix->removeChild(_selected.get());

    //  Remove _osgDraggerRoot from scene graph
    parent->removeChild(_osgDraggerRoot);

    //  Clears memory of _osgDraggerRoot selection object
    _osgDraggerRoot.release();

    //  For each children of dragger recalculate his global transform
    for (size_t i = 0; i < _linkChildren.size(); i++) {
        tglobal = _linkChildren[i];

        //  Debug
//        qWarning("link: %s",tglobal->getName().c_str());

//        //  If link  does Not need anchor to place correctly dragger
//        if (_needAnchor.find(robot->getName() + tglobal->getName()) == _needAnchor.end())
//        {
//          //  If link transform demands anchor to place dragger
//          if (tglobal->getMatrix().getTrans().x() == 0.0 && tglobal->getMatrix().getTrans().y() == 0.0
//              && tglobal->getMatrix().getTrans().z() == 0.0)
//          {
//            _needAnchor[robot->getName() + tglobal->getName()] = true;
//          }
//        }

        mL = _draggerMatrix->getMatrix();

        //  Modify transform
        tglobal->setMatrix(tglobal->getMatrix() * _draggerMatrix->getMatrix());
    }

    // Clears list of link children
    _linkChildren.resize(0);
    _UpdateFromOSG();
}

KinBody::JointPtr ViewerWidget::_FindJoint(std::string & robotName,std::string &linkName)
{
    KinBody::JointPtr joint;
    KinBody::LinkPtr link;

    std::vector<RobotBasePtr> robots;

    //  Gets robots
    _penv->GetRobots(robots);

    for (size_t i = 0; i < robots.size(); i++) {
        if (robots[i]->GetName() == robotName) {
            link = robots[i]->GetLink(linkName);

            if (!!link) {
                //  Propagate transformations to child nodes
                _PropagateTransforms();

                //  Gets all childs of the link
                _GetLinkChildren(robotName,link,robots[i]->GetLinks());
                FOREACH(itjoint,robots[i]->GetJoints()) {
                    if ((*itjoint)->GetSecondAttached()==link) {
                        return *itjoint;
                    }
                }
            }
        }
    }

    return joint;
}

//  Lighting Stuff //

osg::ref_ptr<osg::Material> ViewerWidget::_CreateSimpleMaterial(osg::Vec4 color)
{
    osg::ref_ptr<osg::Material> material(new osg::Material());
    material->setDiffuse(osg::Material::FRONT,  osg::Vec4(0.0, 0.0, 0.0, 1.0));
    material->setEmission(osg::Material::FRONT, color);
    return material;
}

osg::ref_ptr<osg::Light> ViewerWidget::_CreateLight(osg::Vec4 color, int lightid)
{
    osg::ref_ptr<osg::Light> light(new osg::Light());
    // each light must have a unique number
    light->setLightNum(lightid);
    // we set the light's position via a PositionAttitudeTransform object
    light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
    light->setDiffuse(color);
    light->setSpecular(osg::Vec4(0.8, 0.8, 0.8, 1.0));
    light->setAmbient( osg::Vec4(0.2, 0.2, 0.2, 1.0));
    //light->setConstantAttenuation(1);
    //light->setQuadraticAttenuation(0.1);
    //light->setSpotCutoff(70.0);
    return light;
}

osg::ref_ptr<osg::Light> ViewerWidget::_CreateAmbientLight(osg::Vec4 color, int lightid)
{
    osg::ref_ptr<osg::Light> light(new osg::Light());
    // each light must have a unique number
    light->setLightNum(lightid);
    // we set the light's position via a PositionAttitudeTransform object
    light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
    light->setDiffuse(color);
    light->setSpecular(osg::Vec4(0, 0, 0, 1.0));
    light->setAmbient( osg::Vec4(0.5, 0.5, 0.5, 1.0));

    //  Attenuation
    light->setConstantAttenuation(1);
    light->setQuadraticAttenuation(0.2);
    return light;
}

void ViewerWidget::_InitializeLights(int nlights)
{
    _vLightTransform.resize(nlights);

    // we need the scene's state set to enable the light for the entire scene
    _osgLightsGroup = new osg::Group();
    _lightStateSet = _osgLightsGroup->getOrCreateStateSet();

    // Create 3 Lights
    osg::Vec4 lightColors[] = { osg::Vec4(1.0, 1.0, 1.0, 1.0),
                                osg::Vec4(1.0, 1.0, 1.0, 1.0), osg::Vec4(1.0, 1.0, 1.0, 1.0),
                                osg::Vec4(1.0, 1.0, 1.0, 1.0), osg::Vec4(1.0, 1.0, 1.0, 1.0) };

    osg::Vec3 lightPosition[] = { osg::Vec3(0, 0, 3.5),
                                  osg::Vec3(0, 0, 2.5), osg::Vec3(-2, -2.5, 2.5),
                                  osg::Vec3(2, 2.5, 2.5), osg::Vec3(-2, 2.5, 2.5) };

    osg::Vec3 lightDirection[] = {osg::Vec3(0.0, 0.0, -1.0),
                                  osg::Vec3(0.0, 0.0, -1.0), osg::Vec3(0.0, 0.0, -1.0),
                                  osg::Vec3(0.0, 0.0, -1.0), osg::Vec3(0.0, 0.0, -1.0) };

    int lightid = 0;
    for (int i = 0; i < nlights; i++)
    {
        // osg::ref_ptr<osg::Geode> lightMarker = new osg::Geode();
        // lightMarker->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 1)));
        // lightMarker->getOrCreateStateSet()->setAttribute(_CreateSimpleMaterial(lightColors[i]));

        osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource();

        if (i == 0) {
            osg::ref_ptr<osg::Light> light = _CreateAmbientLight(lightColors[i], lightid++);
            lightSource->setLight(light.get());
        }
        else {
            osg::ref_ptr<osg::Light> light = _CreateLight(lightColors[i], lightid++);
            lightSource->setLight(light.get());
        }

        lightSource->getLight()->setDirection(lightDirection[i]);
        lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
        lightSource->setStateSetModes(*_lightStateSet, osg::StateAttribute::ON);

        _vLightTransform[i] = new osg::PositionAttitudeTransform();
        _vLightTransform[i]->addChild(lightSource.get());
        // _vLightTransform[i]->addChild(lightMarker);
        _vLightTransform[i]->setPosition(lightPosition[i]);
        _vLightTransform[i]->setScale(osg::Vec3(0.1,0.1,0.1));
        _osgLightsGroup->addChild(_vLightTransform[i].get());
    }
}

void ViewerWidget::_UpdateFromOSG()
{
    std::vector<KinBody::BodyState> vecbodies;
    _penv->GetPublishedBodies(vecbodies);
    FOREACH(itbody,vecbodies) {
        BOOST_ASSERT( !!itbody->pbody );
        KinBodyPtr pbody = itbody->pbody; // try to use only as an id, don't call any methods!
        KinBodyItemPtr pitem = boost::dynamic_pointer_cast<KinBodyItem>(pbody->GetUserData(_userdatakey));
        if (!!pitem) {
            pitem->UpdateFromOSG();
        }
    }
}

osg::Camera *ViewerWidget::GetCamera()
{
    return _osgview->getCamera();
}

osg::ref_ptr<osgGA::CameraManipulator> ViewerWidget::GetCameraManipulator()
{
    return _osgCameraManipulator;
}

OSGMatrixTransformPtr ViewerWidget::GetCameraHUD()
{
    return _osgCameraHUD;
}

void ViewerWidget::_StoreMatrixTransform()
{
    _viewCameraMatrix = _osgview->getCamera()->getViewMatrix();
}

void ViewerWidget::_LoadMatrixTransform()
{
    _osgview->getCamera()->setViewMatrix(_viewCameraMatrix);
}

std::vector<osg::ref_ptr<osgManipulator::Dragger> > ViewerWidget::_CreateDragger(const std::string& draggerName)
{
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > draggers;
    if ("TabPlaneDragger" == draggerName)
    {
        osgManipulator::TabPlaneDragger* d = new osgManipulator::TabPlaneDragger();
        d->setupDefaultGeometry();
        draggers.push_back(d);
    }
    else if ("TabPlaneTrackballDragger" == draggerName)
    {
        osgManipulator::TabPlaneTrackballDragger* d = new osgManipulator::TabPlaneTrackballDragger();
        d->setupDefaultGeometry();
        draggers.push_back(d);
    }
    else if ("TrackballDragger" == draggerName)
    {
        osgManipulator::TrackballDragger* d = new osgManipulator::TrackballDragger();
        d->setupDefaultGeometry();
        draggers.push_back(d);
    }
    else if ("TranslateTrackballDragger" == draggerName)
    {
        osgManipulator::TrackballDragger* d = new osgManipulator::TrackballDragger();
        d->setupDefaultGeometry();
        draggers.push_back(d);
        osgManipulator::TranslateAxisDragger* d2 = new osgManipulator::TranslateAxisDragger();
        d2->setupDefaultGeometry();
        draggers.push_back(d2);
    }
    else if ("Translate1DDragger" == draggerName)
    {
        osgManipulator::Translate1DDragger* d = new osgManipulator::Translate1DDragger();
        d->setupDefaultGeometry();
        draggers.push_back(d);
    }
    else if ("Translate2DDragger" == draggerName)
    {
        osgManipulator::Translate2DDragger* d = new osgManipulator::Translate2DDragger();
        d->setupDefaultGeometry();
        draggers.push_back(d);
    }
    else if ("TranslateAxisDragger" == draggerName)
    {
        osgManipulator::TranslateAxisDragger* d = new osgManipulator::TranslateAxisDragger();
        d->setupDefaultGeometry();
        draggers.push_back(d);
    }
    else if ("RotateCylinderDragger" == draggerName)
    {
        osgManipulator::RotateCylinderDragger* d = new osgManipulator::RotateCylinderDragger();
        d->setupDefaultGeometry();
        draggers.push_back(d);
    }
    return draggers;
}

OSGNodePtr ViewerWidget::_AddDraggerToObject(OSGNodePtr object, const std::string& name)
{
    return _AddDraggerToObject(std::string(),object,name,KinBody::JointPtr());
}

OSGNodePtr ViewerWidget::_AddDraggerToObject(const std::string& robotName, OSGNodePtr object, const std::string& draggerName, KinBody::JointPtr joint)
{
    osg::MatrixList matrices = object->getWorldMatrices();
    if( matrices.size() > 0 ) {
        RAVELOG_DEBUG_FORMAT("%s %d: %f %f %f", object->getName()%matrices.size()%matrices[0].getTrans()[0]%matrices[0].getTrans()[1]%matrices[0].getTrans()[2]);
    }

//      object->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    // Clears dragger
    _ClearDragger();

    //  New selection
    _draggerMatrix = new osg::MatrixTransform;

    //  Create a new dragger
    _draggers = _CreateDragger(draggerName);
    _osgDraggerRoot = new osg::Group;
    for(size_t idragger = 0; idragger < _draggers.size(); ++idragger) {
        //if( idragger > 0 ) {
        // add a progressively bigger scale
        osg::ref_ptr<osg::MatrixTransform> pscaleparent(new osg::MatrixTransform);
        pscaleparent->setMatrix(osg::Matrix::scale(1+0.1*idragger, 1+0.1*idragger, 1+0.1*idragger));
        pscaleparent->addChild(_draggers[idragger].get());
        _osgDraggerRoot->addChild(pscaleparent.get());
    }
    _osgDraggerRoot->addChild(_draggerMatrix);


    //  Store object selected in global variable _selected
    _selected = object;

    if (draggerName == "RotateCylinderDragger" && !!joint) {
        //  Change view of dragger since a joint is selected
        SetWire(_draggers.at(0));

        for (size_t i = 0; i < object->getParents().size(); i++) {
            OSGGroupPtr parent;
            parent = object->getParents().at(i);
            parent->removeChild(object);
            parent->addChild(_osgDraggerRoot);
        }
    }
    else if (draggerName != "RotateCylinderDragger") {
        for (size_t i = 0; i < object->getParents().size(); i++) {
            OSGGroupPtr parent;
            parent = object->getParents().at(i);
            parent->replaceChild(object,_osgDraggerRoot);
        }
    }

    //  Adds object to selection
    _draggerMatrix->addChild(object);

    float scale = object->getBound().radius() * 1.2;

    if (draggerName == "RotateCylinderDragger" && !!joint) {
        Vector axis;
        Vector anchor;
        Vector dragger_direction;
        Vector dragger_rotation;
        osg::Matrix matrix;
        axis = joint->GetAxis();
        anchor = joint->GetAnchor();
        dragger_direction = Vector(0,0,1);
        dragger_rotation = quatRotateDirection(dragger_direction,axis);

        matrix.makeRotate(osg::Quat(dragger_rotation.y,dragger_rotation.z,dragger_rotation.w,dragger_rotation.x));

        _draggers.at(0)->setMatrix(matrix * osg::Matrix::translate(anchor.x,anchor.y,anchor.z)*osg::Matrix::scale(scale, scale, scale));
    }
    else {
        FOREACH(itdragger, _draggers) {
            (*itdragger)->setMatrix(osg::Matrix::translate(object->getBound().center())*osg::Matrix::scale(scale, scale, scale));
        }
    }

    FOREACH(itdragger, _draggers) {
        (*itdragger)->addTransformUpdating(_draggerMatrix.get()); // in version 3.2 can specify what to transform
        // we want the dragger to handle it's own events automatically
        (*itdragger)->setHandleEvents(true);

        // if we don't set an activation key or mod mask then any mouse click on
        // the dragger will activate it, however if do define either of ActivationModKeyMask or
        // and ActivationKeyEvent then you'll have to press either than mod key or the specified key to
        // be able to activate the dragger when you mouse click on it.  Please note the follow allows
        // activation if either the ctrl key or the 'a' key is pressed and held down.
        (*itdragger)->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
        //(*itdragger)->setActivationKeyEvent('a');
    }

    for(size_t idragger0 = 0; idragger0 < _draggers.size(); ++idragger0) {
        for(size_t idragger1 = 0; idragger1 < _draggers.size(); ++idragger1) {
            if( idragger0 != idragger1 ) {
                _draggers[idragger0]->addDraggerCallback(new DualDraggerTransformCallback(_draggers[idragger0].get(), _draggers[idragger1].get()));
            }
        }
    }

    return _osgDraggerRoot;
}

void ViewerWidget::paintEvent( QPaintEvent* event )
{
    try {
        frame(); // osgViewer::CompositeViewer
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("got exception in paint event: %s", ex.what());
    }
}

//    void mouseReleaseEvent(QMouseEvent *e)
//    {
//      if (doubleClickPressed)
//      {
//        doubleClickPressed = false;
//
//        if (isSimpleView)
//        {
//          setMultipleView();
//        }
//        else
//        {
//          setSimpleView();
//        }
//      }
//    }
//    ////////////////////////////////////////////////////////////////////////////
//    /// Mouse double click event handler
//    ////////////////////////////////////////////////////////////////////////////
//    void mouseDoubleClickEvent(QMouseEvent *e)
//    {
//      doubleClickPressed = true;
//    }

} // end namespace qtosgrave
