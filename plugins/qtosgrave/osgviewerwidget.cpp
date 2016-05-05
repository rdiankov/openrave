// -*- coding: utf-8 -*-
// Copyright (C) 2012-2016 Rosen Diankov, Gustavo Puche, OpenGrasp Team
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
#include "osgcartoon.h"

#include <osg/ShadeModel>
#include <osgDB/ReadFile>
#include <osg/FrontFace>
#include <osg/CullFace>

#include <osgManipulator/CommandManager>
#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>


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
    OpenRAVEKeyboardEventHandler(const boost::function<bool(int, int)>& onKeyDown) : _onKeyDown(onKeyDown) {
    }

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
    {
        switch(ea.getEventType())
        {
        case (osgGA::GUIEventAdapter::KEYDOWN): {
            return _onKeyDown(ea.getKey(), ea.getModKeyMask());
        }
        default:
            return false;
        }
    }

private:
    boost::function<bool(int, int)> _onKeyDown; ///< called when key is pressed
};

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

ViewerWidget::ViewerWidget(EnvironmentBasePtr penv, const std::string& userdatakey, const boost::function<bool(int)>& onKeyDown) : QWidget(), _onKeyDown(onKeyDown)
{
    setKeyEventSetsDone(0); // disable Escape key from killing the viewer!

    _userdatakey = userdatakey;
    _penv = penv;
    _bLightOn = true;
    _bIsSelectiveActive = false;
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
    _picker = new OSGPickHandler(boost::bind(&ViewerWidget::HandleRayPick, this, _1, _2, _3), boost::bind(&ViewerWidget::UpdateFromOSG,this));
    _osgview->addEventHandler(_picker);

    _keyhandler = new OpenRAVEKeyboardEventHandler(boost::bind(&ViewerWidget::HandleOSGKeyDown, this, _1, _2));
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

        _osgHudText = new osgText::Text();

        //Set the screen alignment - always face the screen
        _osgHudText->setAxisAlignment(osgText::Text::SCREEN);
        _osgHudText->setColor(osg::Vec4(0,0,0,1));
        //text->setFontResolution(32,32);

        _osgHudText->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE ); // need to do this, otherwise will be using the light sources

        osg::ref_ptr<osg::Geode> geodetext = new osg::Geode;
        geodetext->addDrawable(_osgHudText);
        _osgCameraHUD->addChild(geodetext);
    }

    _InitializeLights(2);

    {
        osg::ref_ptr<qtosgrave::OpenRAVECartoon> toon = new qtosgrave::OpenRAVECartoon();
        //toon->setOutlineColor(osg::Vec4(0,1,0,1));
        _osgLightsGroup->addChild(toon);
        toon->addChild(_osgSceneRoot);
    }

    //_osgLightsGroup->addChild(_osgSceneRoot);
    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()) );
    _timer.start( 10 );
}

bool ViewerWidget::HandleOSGKeyDown(int key, int modkeymask)
{
    if( !!_onKeyDown ) {
        if( _onKeyDown(key) ) {
            return true;
        }
    }

    switch(key) {
    case osgGA::GUIEventAdapter::KEY_Left:
    case osgGA::GUIEventAdapter::KEY_Right:
    case osgGA::GUIEventAdapter::KEY_Up:
    case osgGA::GUIEventAdapter::KEY_Down:{
        osg::Matrixd m = _osgCameraManipulator->getMatrix();
        osg::Vec3d center = _osgCameraManipulator->getCenter();
        osg::Vec3d dir;
        if( (modkeymask & osgGA::GUIEventAdapter::MODKEY_SHIFT) ) {
            if( key == osgGA::GUIEventAdapter::KEY_Up ) {
                dir = osg::Vec3d(-m(2,0), -m(2,1), -m(2,2));
            }
            else if( key == osgGA::GUIEventAdapter::KEY_Down ) {
                dir = osg::Vec3d(m(2,0), m(2,1), m(2,2));
            }

        }
        else {
            if( key == osgGA::GUIEventAdapter::KEY_Left ) {
                dir = osg::Vec3d(-m(0,0), -m(0,1), -m(0,2));
            }
            else if( key == osgGA::GUIEventAdapter::KEY_Right ) {
                dir = osg::Vec3d(m(0,0), m(0,1), m(0,2));
            }
            else if( key == osgGA::GUIEventAdapter::KEY_Down ) {
                dir = osg::Vec3d(-m(1,0), -m(1,1), -m(1,2));
            }
            else if( key == osgGA::GUIEventAdapter::KEY_Up ) {
                dir = osg::Vec3d(m(1,0), m(1,1), m(1,2));
            }
        }
        _osgCameraManipulator->setCenter(center + dir*_osgCameraManipulator->getDistance()*0.05);
                    
        return true;
    }        
//        _picker->ActivateSelection(!_picker->IsSelectionActive());
//        if( !_picker->IsSelectionActive() ) {
//            // have to clear any draggers if selection is not active
//            _ClearDragger();
//            _draggerName.clear();
//        }
//        return true;
    }
    return false;
}

void ViewerWidget::ActivateSelection(bool active)
{
    osgViewer::Viewer::Windows windows;
    getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end(); ++itr) {
        (*itr)->setCursor(active ? osgViewer::GraphicsWindow::HandCursor : osgViewer::GraphicsWindow::LeftArrowCursor);
    }
    _bIsSelectiveActive = active;
}

void ViewerWidget::SetDraggerMode(const std::string& draggerName)
{
    if( draggerName.size() > 0 ) {
        _draggerName = draggerName;
        SelectItem(_selectedItem);
    }
    else {
        _ClearDragger();
        _draggerName.clear();
    }
}

void ViewerWidget::SelectItem(KinBodyItemPtr item)
{
    _selectedItem = item;
    if (!!_selectedItem) {
        _AddDraggerToObject(_draggerName, _selectedItem, KinBody::JointPtr());
    }
    else {
        // no dragger so clear?
        _ClearDragger();
    }
}

void ViewerWidget::SelectItemFromName(const std::string& name)
{
    SelectItem(_GetItemFromName(name));
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
        osg::ref_ptr<qtosgrave::OpenRAVECartoon> toon = new qtosgrave::OpenRAVECartoon();
        //toon->setOutlineColor(osg::Vec4(0,1,0,1));
        rootscene->addChild(toon);
        toon->addChild(_osgSceneRoot);
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
        const osg::BoundingSphere& bs = _osgSceneRoot->getBound();
        _osgview->getCameraManipulator()->setHomePosition(osg::Vec3d(1.5*bs.radius(),0,1.5*bs.radius()),bs.center(),osg::Vec3d(0.0,0.0,1.0));
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
    if (enabled) {
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

void ViewerWidget::HandleRayPick(const osgUtil::LineSegmentIntersector::Intersection& intersection, int buttonPressed, int modkeymask)
{
    if (intersection.nodePath.empty()) {
        _strRayInfoText.clear();
        _UpdateHUDText();
        if( _bIsSelectiveActive && buttonPressed ) {
            SelectOSGLink(OSGNodePtr(), modkeymask);
        }
    }
    else {
        if (!intersection.nodePath.empty() ) {
            OSGNodePtr node = intersection.nodePath.back();//intersection.drawable->getParent(0);

            // something hit
            if( buttonPressed ) {
                if( _bIsSelectiveActive ) {
                    if( !!node ) {
                        SelectOSGLink(node, modkeymask);
                    }
                }
                else {
                    // ignore...
                }
            }
            else {
                // draw the intersection point in the HUD
                KinBodyItemPtr item = FindKinBodyItemFromOSGNode(node);

                if( !!item ) {
                    osg::Vec3d pos = intersection.getWorldIntersectPoint();
                    osg::Vec3d normal = intersection.getWorldIntersectNormal();
                    KinBody::LinkPtr link = item->GetLinkFromOSG(node);
                    std::string linkname;
                    if( !!link ) {
                        linkname = link->GetName();
                    }
                    _strRayInfoText = str(boost::format("mouse on %s:%s: (%.5f, %.5f, %.5f), n=(%.5f, %.5f, %.5f)")%item->GetName()%linkname%pos.x()%pos.y()%pos.z()%normal.x()%normal.y()%normal.z());
                }
                else {
                    _strRayInfoText.clear();
                }
                _UpdateHUDText();
            }
        }
    }
}

void ViewerWidget::UpdateFromOSG()
{
    if( !!_selectedItem ) {
        // have to update the underlying openrave model since dragger is most likely changing the positions
        _selectedItem->UpdateFromOSG();
        Transform t = _selectedItem->GetTransform();
        _strSelectedItemText = str(boost::format("Selected %s. trans=(%.5f, %.5f, %.5f)")%_selectedItem->GetName()%t.trans.x%t.trans.y%t.trans.z);
    }
    else {
        _strSelectedItemText.clear();
    }
}

void ViewerWidget::SelectOSGLink(OSGNodePtr node, int modkeymask)
{
    if (!node) {
        if( !(modkeymask & osgGA::GUIEventAdapter::MODKEY_CTRL) ) {
            // user clicked on empty region, so remove selection
            _ClearDragger();
        }
        return;
    }

    KinBody::JointPtr joint;
    KinBodyItemPtr item = FindKinBodyItemFromOSGNode(node);
    if (!!item) {
        KinBody::LinkPtr link = item->GetLinkFromOSG(node);

//        if( (modkeymask & osgGA::GUIEventAdapter::MODKEY_ALT) ) {
//            //RAVELOG_INFO("setting camera manipulator tracking node\n");
//            //_osgCameraManipulator->setTrackNode(robot);
//            _osgCameraManipulator->setNode(node.get());//robot.get());
//            _osgCameraManipulator->computeHomePosition();
//            _osgCameraManipulator->home(2);
//        }

        //  Gets camera transform
        //_StoreMatrixTransform(); // TODO store matrix for later canceling of the move

        // Find joint of a link name given
        joint = _FindJoint(item, link);
        if( (modkeymask & osgGA::GUIEventAdapter::MODKEY_CTRL) ) {
        }
        else {
            if( _draggerName == "RotateCylinderDragger" ) {
                if( !!joint) {
                    _selectedItem = item;
                    _AddDraggerToObject("RotateCylinderDragger", item, joint);
                }
            }
            else {
                // select new body
                SelectItem(item);
            }
        }
    }

    if( !item ) {
        if( !(modkeymask & osgGA::GUIEventAdapter::MODKEY_CTRL) ) {
            // user clicked on empty region, so remove selection
            _ClearDragger();
        }
    }
}

void ViewerWidget::_ClearDragger()
{
    if( !!_osgSelectedNodeByDragger && !!_osgDraggerRoot ) {
        OSGGroupPtr parent;
        if( _osgDraggerRoot->getNumParents() > 0 ) {
            parent = _osgDraggerRoot->getParent(0);
        }
        else {
            parent = _osgSceneRoot; // fallback
        }
        if( !parent->replaceChild(_osgDraggerRoot, _osgSelectedNodeByDragger) ) {
            RAVELOG_WARN("failed to replace child when restoring dragger\n");
        }
        if( !!_draggerMatrix ) {
            _draggerMatrix->setMatrix(osg::Matrix::identity());
        }
        if( !!_selectedItem ) {
            _selectedItem->UpdateFromModel(); // since the scene graph changed ,have to update the OSG nodes!
        }
    }

    FOREACH(itdragger, _draggers) {
        if (!!*itdragger && (*itdragger)->getNumParents() > 0) {
            (*itdragger)->getParents().at(0)->removeChild(*itdragger);
        }
    }
    _draggers.resize(0);
    _osgSelectedNodeByDragger.release();
    _osgDraggerRoot.release();
}

void ViewerWidget::SetUserHUDText(const std::string& text)
{
    _strUserText = text;
    _UpdateHUDText();
}

void ViewerWidget::_UpdateHUDText()
{
    std::string s;
    if( _strRayInfoText.size() > 0 ) {
        s += _strRayInfoText;
    }
    if( _strUserText.size() > 0 ) {
        if( s.size() > 0 ) {
            s += "\n";
        }
        s += _strRayInfoText;
    }
    if( _strSelectedItemText.size() > 0 ) {
        if( s.size() > 0 ) {
            s += "\n";
        }
        s += _strSelectedItemText;
    }
    _osgHudText->setText(s);
}

void ViewerWidget::SetViewType(int isorthogonal, double metersinunit)
{
    int width = _osgview->getCamera()->getViewport()->width();
    int height = _osgview->getCamera()->getViewport()->height();
    double aspect = static_cast<double>(width)/static_cast<double>(height);
    if( isorthogonal ) {
        double distance = 0.5*_osgCameraManipulator->getDistance();
        _osgview->getCamera()->setProjectionMatrixAsOrtho(-distance/metersinunit, distance/metersinunit, -distance/(metersinunit*aspect), distance/(metersinunit*aspect), 0, 100.0/metersinunit);
    }
    else {
        _osgview->getCamera()->setProjectionMatrixAsPerspective(30.0f, aspect, 1.0f, 100.0/metersinunit );
    }
}

void ViewerWidget::SetViewport(int width, int height, double metersinunit)
{
    _osgview->getCamera()->setViewport(0,0,width,height);
    if( _osgview->getCamera()->getProjectionMatrix()(2,3) == 0 ) {
        // orthogonal
        double aspect = static_cast<double>(width)/static_cast<double>(height);
        double distance = 0.5*_osgCameraManipulator->getDistance();
        _osgview->getCamera()->setProjectionMatrixAsOrtho(-distance/metersinunit, distance/metersinunit, -distance/(metersinunit*aspect), distance/(metersinunit*aspect), 0, 100.0/metersinunit);
    }

    _osghudview->getCamera()->setViewport(0,0,width,height);
    _osghudview->getCamera()->setProjectionMatrix(osg::Matrix::ortho(-width/2, width/2, -height/2, height/2, 0, 1000));

    osg::Matrix m = _osgCameraManipulator->getInverseMatrix();
    m.setTrans(width/2 - 40, -height/2 + 40, -50);
    _osgWorldAxis->setMatrix(m);

    double textheight = (10.0/480.0)*height;
    _osgHudText->setPosition(osg::Vec3(-width/2+10, height/2-textheight, -50));
    _osgHudText->setCharacterSize(textheight);
}

QWidget* ViewerWidget::_AddViewWidget( osg::ref_ptr<osg::Camera> camera, osg::ref_ptr<osgViewer::View> view, osg::ref_ptr<osg::Camera> hudcamera, osg::ref_ptr<osgViewer::View> hudview )
{
    view->setCamera( camera.get() );
    hudview->setCamera( hudcamera.get() );
    addView( view.get() );
    addView( hudview.get() );

    //view->addEventHandler( new osgViewer::StatsHandler );

    _osgCameraManipulator = new osgGA::TrackballManipulator();//NodeTrackerManipulator();
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

KinBodyItemPtr ViewerWidget::_GetItemFromName(const std::string& name)
{
    KinBodyPtr pbody = _penv->GetKinBody(name);
    KinBodyItemPtr pitem = boost::dynamic_pointer_cast<KinBodyItem>(pbody->GetUserData(_userdatakey));
    return pitem;
}

KinBodyItemPtr ViewerWidget::FindKinBodyItemFromOSGNode(OSGNodePtr node)
{
    if (!node) {
        return KinBodyItemPtr();
    }

    if( !!node->getUserData() ) {
        OSGItemUserData* pdata = dynamic_cast<OSGItemUserData*>(node->getUserData());
        if( !!pdata ) {
            ItemPtr pitem = pdata->GetItem();
            if( !pitem ) {
                RAVELOG_WARN("trying to use a deleted item\n");
            }
            return boost::dynamic_pointer_cast<KinBodyItem>(pitem);
        }
    }

    // go up the parent chain until can find OSGItemUserData
    for(size_t iparent = 0; iparent < node->getParents().size(); ++iparent) {
        osg::Group* parent = node->getParents().at(iparent);
        KinBodyItemPtr pitem = FindKinBodyItemFromOSGNode(parent);
        if( !!pitem ) {
            return pitem;
        };
    }

    return KinBodyItemPtr();
}


KinBody::JointPtr ViewerWidget::_FindJoint(KinBodyItemPtr pitem, KinBody::LinkPtr link)
{
    if( !!pitem && !!pitem->GetBody() && !!link ) {
        // search for the joint whose child link is this link
        FOREACHC(itjoint, pitem->GetBody()->GetJoints() ) {
            if( (*itjoint)->GetHierarchyChildLink() == link ) {
                return *itjoint;
            }
        }
    }
    return KinBody::JointPtr();
}

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
        //lightSource->setReferenceFrame(osg::LightSource::ABSOLUTE_RF);

        _vLightTransform[i] = new osg::PositionAttitudeTransform();
        _vLightTransform[i]->addChild(lightSource.get());
        // _vLightTransform[i]->addChild(lightMarker);
        _vLightTransform[i]->setPosition(lightPosition[i]);
        _vLightTransform[i]->setScale(osg::Vec3(0.1,0.1,0.1));
        _osgLightsGroup->addChild(_vLightTransform[i].get());
    }
}

//void ViewerWidget::_UpdateFromOSG()
//{
//    std::vector<KinBody::BodyState> vecbodies;
//    _penv->GetPublishedBodies(vecbodies);
//    FOREACH(itbody,vecbodies) {
//        BOOST_ASSERT( !!itbody->pbody );
//        KinBodyPtr pbody = itbody->pbody; // try to use only as an id, don't call any methods!
//        KinBodyItemPtr pitem = boost::dynamic_pointer_cast<KinBodyItem>(pbody->GetUserData(_userdatakey));
//        if (!!pitem) {
//            pitem->UpdateFromOSG();
//        }
//    }
//}

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
        
        // scale the axes so that they are bigger. since d and d2 need to share the same transform, have to add a scale node in between
        osg::ref_ptr<osg::MatrixTransform> pscaleparent(new osg::MatrixTransform);
        pscaleparent->setMatrix(osg::Matrix::scale(1.3, 1.3, 1.3));
        for(size_t ichild = 0; ichild < d2->getNumChildren(); ++ichild) {
            pscaleparent->addChild(d2->getChild(ichild));
        }
        d2->removeChildren(0, d2->getNumChildren());
        d2->addChild(pscaleparent);
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

OSGNodePtr ViewerWidget::_AddDraggerToObject(const std::string& draggerName, KinBodyItemPtr item, KinBody::JointPtr pjoint)
{
    // Clears dragger
    _ClearDragger();

    //  New selection
    _draggerMatrix = new osg::MatrixTransform;

    //  Create a new dragger
    _draggers = _CreateDragger(draggerName);
    _osgDraggerRoot = new osg::Group;
    for(size_t idragger = 0; idragger < _draggers.size(); ++idragger) {
        _osgDraggerRoot->addChild(_draggers[idragger]);
    }
    _osgDraggerRoot->addChild(_draggerMatrix);

    //  Store object selected in global variable _osgSelectedNodeByDragger
    osg::Matrixd selectedmatrix;
    
    if( !pjoint ) {
        _osgSelectedNodeByDragger = item->GetOSGRoot();
        selectedmatrix = item->GetOSGRoot()->getMatrix();
    }
    else {
        OSGMatrixTransformPtr osglinktrans = item->GetOSGLink(pjoint->GetHierarchyChildLink()->GetIndex());
        selectedmatrix = osglinktrans->getMatrix();
        _osgSelectedNodeByDragger = osglinktrans->getParent(0);
    }

    if (draggerName == "RotateCylinderDragger" && !!pjoint) {
        //  Change view of dragger since a joint is selected
        SetWire(_draggers.at(0));
        OSGGroupPtr(_osgSelectedNodeByDragger->getParent(0))->replaceChild(_osgSelectedNodeByDragger, _osgDraggerRoot);
    }
    else if (draggerName != "RotateCylinderDragger") {
        _osgSceneRoot->replaceChild(_osgSelectedNodeByDragger, _osgDraggerRoot);
    }

    //  Adds object to selection
    _draggerMatrix->addChild(_osgSelectedNodeByDragger);
    float scale = _osgSelectedNodeByDragger->getBound().radius() * 1.2;
    
    if (draggerName == "RotateCylinderDragger" && !!pjoint) {
        Vector axis;
        Vector anchor;
        Vector dragger_direction;
        Vector dragger_rotation;
        osg::Matrix matrix;

        Transform tbodyinv = item->GetTransform().inverse();

        // need to be in the body coord system
        axis = tbodyinv.rotate(pjoint->GetAxis());
        anchor = item->GetTransform().inverse() * pjoint->GetAnchor();
        dragger_direction = Vector(0,0,1);
        dragger_rotation = quatRotateDirection(dragger_direction,axis);
        matrix.makeRotate(osg::Quat(dragger_rotation.y,dragger_rotation.z,dragger_rotation.w,dragger_rotation.x));
        matrix.setTrans(osg::Vec3(anchor.x, anchor.y, anchor.z));
        matrix.preMult(osg::Matrix::scale(scale, scale, scale));
        _draggers.at(0)->setMatrix(matrix);
    }
    else {
        selectedmatrix.preMult(osg::Matrix::scale(scale, scale, scale));
        for(size_t idragger = 0; idragger < _draggers.size(); ++idragger) {
            _draggers[idragger]->setMatrix(selectedmatrix);
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

} // end namespace qtosgrave
