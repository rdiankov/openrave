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
#include "osgskybox.h"

#include <osg/ShadeModel>
#include <osgDB/ReadFile>
#include <osg/FrontFace>
#include <osg/CullFace>
#include <osg/CoordinateSystemNode>
#include <osg/BlendFunc>
#include <osgGA/NodeTrackerManipulator>
#include <osgManipulator/CommandManager>
#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>

namespace qtosgrave {

class OpenRAVETracker : public osgGA::NodeTrackerManipulator {
public:
    OpenRAVETracker()
    {
        _currentTransitionAnimationTime = 0;
        _transitionAnimationPath = new osg::AnimationPath();
        _transitionAnimationPath->setLoopMode(osg::AnimationPath::NO_LOOPING);
    }

    void StartTrackingNode(osg::Node* node, const osg::Vec3d& offset, double trackDistance, osg::Camera* currentCamera, const osg::Vec3d& worldUpVector)
    {
        _offset = offset;
        _distance = trackDistance;
        osg::NodePathList nodeParents = node->getParentalNodePaths();
        if(nodeParents.empty()) {
            RAVELOG_WARN("Could not track node, node has no transform chain");
            return;
        }
        NodeTrackerManipulator::setTrackNodePath(nodeParents[0]);
        _transitionAnimationDuration = 1.0; //< transition animation time
        _currentTransitionAnimationTime = 0;
        _CreateTransitionAnimationPath(node, currentCamera, worldUpVector);
        _time.restart();
    }

    // OSG overloaded methods
public:

    osg::Matrixd getMatrix() const {
        osg::Vec3d nodeCenter;
        osg::Quat nodeRotation;
        computeNodeCenterAndRotation(nodeCenter, nodeRotation);

        osg::Matrixd localToWorld;
        computeNodeLocalToWorld(localToWorld);

        nodeCenter = osg::Vec3d(_offset) * localToWorld;

        return osg::Matrixd::translate(0.0,0.0,_distance)*osg::Matrixd::rotate(_rotation)*osg::Matrix::translate(nodeCenter);
    }

    // need to reimplement this method so we can track based on nodes origin instead of center of bounding sphere
    osg::Matrixd getInverseMatrix() const
    {
        if(!_IsTransitionAnimationFinished()) {
            // a very small const encapsulation break, since this allow all animation logic to be totally contained in the manipulator
            return const_cast<OpenRAVETracker*>(this)->_GetTransitionAnimationMatrix();
        }

        osg::Vec3d nodeCenter;
        osg::Quat nodeRotation;
        computeNodeCenterAndRotation(nodeCenter,nodeRotation);

        osg::Matrixd localToWorld, worldToLocal;
        computeNodeLocalToWorld(localToWorld);

        nodeCenter = osg::Vec3d(_offset) * localToWorld;
        return osg::Matrixd::translate(-nodeCenter)*osg::Matrixd::rotate(_rotation.inverse())*osg::Matrixd::translate(0.0,0.0,-_distance);
    }

private:
    void _CreateTransitionAnimationPath(osg::Node* node, osg::Camera* currentCamera, const osg::Vec3d& worldUpVector)
    {
        _transitionAnimationPath->clear();

        // create animation frames in world space
        osg::Matrixd viewMatrix = osg::Matrixd::inverse(currentCamera->getViewMatrix());

        osg::Vec3d cameraWorldPos(viewMatrix(3,0), viewMatrix(3,1), viewMatrix(3,2));

         // now calculate a pose that will look to the node using current position
        osg::Matrixd localToWorld;
        computeNodeLocalToWorld(localToWorld);

        // openscenegraph has transposed matrix form to match opengl spec
        osg::Vec3d nodeCenterWorld = osg::Vec3d(_offset) * localToWorld;
        osg::Vec3d towardsNodeVector = nodeCenterWorld - cameraWorldPos;
        if(towardsNodeVector.length() < 1e-3) {
            // already very close to desired position, no need to animate
            _transitionAnimationDuration = 0;
            return;
        }


        osg::Quat cameraRotation = viewMatrix.getRotate();
        _transitionAnimationPath->insert(0,osg::AnimationPath::ControlPoint(cameraWorldPos, cameraRotation));

        // last frame pose is looking at the node from a distance _distance
        osg::Matrixd lookAtNodeMatrix;
        // // check if we are already at a proper distance from tracked node - if so, no need to navigate to there, just skip this keyframe
        // if( abs(towardsNodeVector.length() -_distance ) < 1e-3 ) {
        //     return;
        // }
        towardsNodeVector.normalize();
        lookAtNodeMatrix.makeLookAt(nodeCenterWorld - towardsNodeVector * _distance, nodeCenterWorld, worldUpVector);
        _transitionAnimationPath->insert(_transitionAnimationDuration,
            osg::AnimationPath::ControlPoint(nodeCenterWorld - (towardsNodeVector * _distance), lookAtNodeMatrix.getRotate().inverse()));
    }

    bool _IsTransitionAnimationFinished() const
    {
        return _transitionAnimationDuration == 0 || _currentTransitionAnimationTime >= _transitionAnimationDuration;
    }

    osg::Matrixd _GetTransitionAnimationMatrix()
    {
        if(_IsTransitionAnimationFinished()) {
            return osg::Matrixd();
        }
        double dt = _time.restart() / 1000.0;
        osg::AnimationPath::ControlPoint controlPoint;
        _transitionAnimationPath->getInterpolatedControlPoint(_currentTransitionAnimationTime, controlPoint);
        _currentTransitionAnimationTime =_currentTransitionAnimationTime+dt;
        osg::Quat lookAtRotation = controlPoint.getRotation().inverse();
        osg::Vec3d lookAtTranslation = lookAtRotation * -controlPoint.getPosition();

        // build a lookat matrix from position and camera axis
        osg::Matrixd result;
        result.makeRotate(lookAtRotation);
        result(3,0) = lookAtTranslation.x();
        result(3,1) = lookAtTranslation.y();
        result(3,2) = lookAtTranslation.z();

        // set final manipulator rotation so it matches the last animation rotation, so when it finishes (or get stopped),
        // we start manipulating from there
        _rotation = controlPoint.getRotation();
        return result;
    }

private:
    QTime _time;
    osg::Vec3d _offset;
    double _transitionAnimationDuration; //< specifies how long the transition will take
    double _currentTransitionAnimationTime;
    osg::ref_ptr<osg::AnimationPath> _transitionAnimationPath;
};

class OpenRAVETrackball : public osgGA::TrackballManipulator
{
public:
    OpenRAVETrackball(QOSGViewerWidget* osgviewerwidget) {
        _bInSeekMode = false;
        _posgviewerwidget = osgviewerwidget;
        _pviewer = osgviewerwidget->GetViewer();
        setAnimationTime(0.25);
    }

    void SetSeekMode(bool bInSeekMode) {
        _bInSeekMode = bInSeekMode;
        osgViewer::Viewer::Windows windows;
        _pviewer->getWindows(windows);
        if( _bInSeekMode ) {
            QCursor cursor = _bInSeekMode ? Qt::CrossCursor : Qt::ArrowCursor;
            _posgviewerwidget->setCursor(cursor);
        }
        else {
            _posgviewerwidget->RestoreCursor();
        }
    }

    bool InSeekMode() const {
        return _bInSeekMode;
    }

protected:
    class OpenRAVEAnimationData : public OrbitAnimationData {
public:
        osg::Vec3d _eyemovement;
    };

    virtual void allocAnimationData() {
        _animationData = new OpenRAVEAnimationData();
    }

    virtual bool performMovement() {
        // return if less then two events have been added
        if( _ga_t0.get() == NULL || _ga_t1.get() == NULL ) {
            return false;
        }
        // get delta time
        double eventTimeDelta = _ga_t0->getTime() - _ga_t1->getTime();
        if( eventTimeDelta < 0. ) {
            OSG_WARN << "Manipulator warning: eventTimeDelta = " << eventTimeDelta << std::endl;
            eventTimeDelta = 0.;
        }

        // get deltaX and deltaY
        float dx = _ga_t0->getXnormalized() - _ga_t1->getXnormalized();
        float dy = _ga_t0->getYnormalized() - _ga_t1->getYnormalized();

        // return if there is no movement.
        if( dx == 0. && dy == 0. ) {
            return false;
        }

        unsigned int buttonMask = _ga_t1->getButtonMask();
        if( (buttonMask & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) && (_ga_t1->getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_SHIFT) ) {
            return performMovementMiddleMouseButton( eventTimeDelta, dx, dy );
        }

        return osgGA::TrackballManipulator::performMovement();
    }

    // make zooming faster
    bool performMovementRightMouseButton( const double eventTimeDelta, const double dx, const double dy )
    {
        return osgGA::TrackballManipulator::performMovementRightMouseButton(eventTimeDelta, dx, dy*2);
    }
    void applyAnimationStep( const double currentProgress, const double prevProgress )
    {
        OpenRAVEAnimationData *ad = dynamic_cast< OpenRAVEAnimationData* >( _animationData.get() );
        assert( ad );

        // compute new center
        osg::Vec3d prevCenter, prevEye, prevUp;
        getTransformation( prevEye, prevCenter, prevUp );
        osg::Vec3d newCenter = osg::Vec3d(prevCenter) + (ad->_movement * (currentProgress - prevProgress));
        osg::Vec3d newEye = osg::Vec3d(prevEye) + (ad->_eyemovement * (currentProgress - prevProgress));

        // fix vertical axis
        if( getVerticalAxisFixed() )
        {
            osg::CoordinateFrame coordinateFrame = getCoordinateFrame( newCenter );
            osg::Vec3d localUp = getUpVector( coordinateFrame );

            fixVerticalAxis( newCenter - newEye, prevUp, prevUp, localUp, false );
        }

        // apply new transformation
        setTransformation( newEye, newCenter, prevUp );
    }

    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        if (osgGA::TrackballManipulator::handle(ea, us)) {
            return true;
        }
        switch( ea.getEventType() )
        {
        case osgGA::GUIEventAdapter::DOUBLECLICK:
            return handleMouseDoubleClick( ea, us );

        default:
            return false;
        }
    }

    bool setCenterByMousePointerIntersection( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        using namespace osg;
        using namespace osgUtil; // to preserve struct with osg library
        osg::View* view = us.asView();
        if( !view )
            return false;

        Camera *camera = view->getCamera();
        if( !camera )
            return false;

        // prepare variables
        float x = ( ea.getX() - ea.getXmin() ) / ( ea.getXmax() - ea.getXmin() );
        float y = ( ea.getY() - ea.getYmin() ) / ( ea.getYmax() - ea.getYmin() );
        LineSegmentIntersector::CoordinateFrame cf;
        Viewport *vp = camera->getViewport();
        if( vp ) {
            cf = Intersector::WINDOW;
            x *= vp->width();
            y *= vp->height();
        } else
            cf = Intersector::PROJECTION;

        // perform intersection computation
        ref_ptr< LineSegmentIntersector > picker = new LineSegmentIntersector( cf, x, y );
        IntersectionVisitor iv( picker.get() );
        iv.setTraversalMask(OSG_IS_PICKABLE_MASK); // different from OSG implementation!
        camera->accept( iv );

        // return on no intersections
        if( !picker->containsIntersections() )
            return false;

        // get all intersections
        LineSegmentIntersector::Intersections& intersections = picker->getIntersections();

        // get current transformation
        osg::Vec3d eye, oldCenter, up;
        getTransformation( eye, oldCenter, up );

        // new center
        osg::Vec3d newCenter = (*intersections.begin()).getWorldIntersectPoint();

        // make vertical axis correction
        if( getVerticalAxisFixed() )
        {

            CoordinateFrame coordinateFrame = getCoordinateFrame( newCenter );
            Vec3d localUp = getUpVector( coordinateFrame );

            fixVerticalAxis( newCenter - eye, up, up, localUp, true );

        }

        // set the new center
        setTransformation( eye, newCenter, up );


        // warp pointer
        // note: this works for me on standard camera on GraphicsWindowEmbedded and Qt,
        //       while it was necessary to implement requestWarpPointer like follows:
        //
        // void QOSGWidget::requestWarpPointer( float x, float y )
        // {
        //    osgViewer::Viewer::requestWarpPointer( x, y );
        //    QCursor::setPos( this->mapToGlobal( QPoint( int( x+.5f ), int( y+.5f ) ) ) );
        // }
        //
        // Additions of .5f are just for the purpose of rounding.
        centerMousePointer( ea, us );

        return true;
    }


    virtual bool seekToMousePointer( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        SetSeekMode(false);
        if( !isAnimating() ) {
            // get current transformation
            osg::Vec3d prevCenter, prevEye, prevUp;
            getTransformation( prevEye, prevCenter, prevUp );

            // center by mouse intersection
            if( !setCenterByMousePointerIntersection( ea, us ) ) {
                return false;
            }

            OpenRAVEAnimationData *ad = dynamic_cast< OpenRAVEAnimationData*>( _animationData.get() );
            BOOST_ASSERT( !!ad );

            // setup animation data and restore original transformation
            ad->_eyemovement = (osg::Vec3d(_center) - prevEye) * 0.5;
            ad->start( osg::Vec3d(_center) - prevCenter, ea.getTime() );
            setTransformation( prevEye, prevCenter, prevUp );
            return true;
        }
        return false;
    }

    virtual bool handleMouseDoubleClick( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        if (seekToMousePointer(ea, us)) {
            return true;
        }
        return false;
    }

    virtual bool handleMousePush( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        if( _bInSeekMode ) {
            if (seekToMousePointer(ea, us)) {
                return true;
            }
        }
        return osgGA::TrackballManipulator::handleMousePush(ea, us);
    }

    virtual bool handleKeyDown( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us )
    {
        int key = ea.getKey();
        int modkeymask = ea.getModKeyMask();
        switch(key) {
        case osgGA::GUIEventAdapter::KEY_Left:
        case osgGA::GUIEventAdapter::KEY_Right:
        case osgGA::GUIEventAdapter::KEY_Up:
        case osgGA::GUIEventAdapter::KEY_Down: {
            osg::Matrixd m = getMatrix();
            osg::Vec3d center = getCenter();
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
            setCenter(center + dir*getDistance()*0.05);
            return true;
        }
        }

        return osgGA::TrackballManipulator::handleKeyDown(ea, us);
    }

private:
    QOSGViewerWidget* _posgviewerwidget;
    osg::ref_ptr<osgViewer::CompositeViewer> _pviewer;
    bool _bInSeekMode; ///< if true, in seek mode
};

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
    OpenRAVEKeyboardEventHandler(const boost::function<bool(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)>& onKeyDown) : _onKeyDown(onKeyDown) {
    }

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
    {
        switch(ea.getEventType())
        {
        case (osgGA::GUIEventAdapter::KEYDOWN): {
            return _onKeyDown(ea, aa);
        }
        default:
            return false;
        }
    }

private:
    boost::function<bool(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)> _onKeyDown; ///< called when key is pressed
};

//void QOSGViewerWidget::_ShowSceneGraph(const std::string& currLevel,OSGNodePtr currNode)
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

QOSGViewerWidget::QOSGViewerWidget(EnvironmentBasePtr penv, const std::string& userdatakey,
                                   const boost::function<bool(int)>& onKeyDown, double metersinunit,
                                   QWidget* parent) : QOpenGLWidget(parent), _onKeyDown(onKeyDown)
{

    setFocus( Qt::ActiveWindowFocusReason );
    setMouseTracking(true);
    _userdatakey = userdatakey;
    _penv = penv;
    _bSwitchMouseLeftMiddleButton = false;
    _bLightOn = true;
    _bIsSelectiveActive = false;
    _metersinunit = metersinunit;
    _osgview = new osgViewer::View();
    _osghudview = new osgViewer::View();
    _osgviewer = new osgViewer::CompositeViewer();
    _osgviewer->setKeyEventSetsDone(0); // disable Escape key from killing the viewer!
    // Qt5 specific thread model
    _osgviewer->setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);

    _SetupCamera(_CreateCamera(0, 0, 100, 100, metersinunit), _osgview,
                 _CreateHUDCamera(0, 0, 100, 100, metersinunit), _osghudview);

    //  Sets pickhandler
    _picker = new OSGPickHandler(boost::bind(&QOSGViewerWidget::HandleRayPick, this, _1, _2, _3), boost::bind(&QOSGViewerWidget::UpdateFromOSG,this));
    _osgview->addEventHandler(_picker);

    _keyhandler = new OpenRAVEKeyboardEventHandler(boost::bind(&QOSGViewerWidget::HandleOSGKeyDown, this, _1, _2));
    _osgview->addEventHandler(_keyhandler);

    _osgview->addEventHandler( new osgViewer::StatsHandler );

    // initialize the environment
    _osgSceneRoot = new osg::Group();
    {
        _osgFigureRoot = new osg::Group();
        osg::ref_ptr<osg::StateSet> stateset = _osgFigureRoot->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE ); // need to do this, otherwise will be using the light sources
        stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
        stateset->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA ));

        _osgFigureRoot->setStateSet(stateset);

    }

    {
        _osgSkybox = new Skybox;
        _osgFigureRoot->addChild(_osgSkybox);
        _osgFigureRoot->setNodeMask(~OSG_IS_PICKABLE_MASK);
    }

    // create world axis
    _osgWorldAxis = new osg::MatrixTransform();

    _osgWorldAxis->addChild(CreateOSGXYZAxes(32.0, 2.0));

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
        _osgHudText->setBackdropType(osgText::Text::DROP_SHADOW_BOTTOM_RIGHT);
        _osgHudText->setBackdropColor(osg::Vec4(1,1,1,1));
        //setBackdropOffset
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

    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()) );
    _timer.start( 10 );

    RestoreCursor();
}

QOSGViewerWidget::~QOSGViewerWidget()
{
    if( !!_selectedItem ) {
        _selectedItem->SetGrab(false);
    }
    _selectedItem.reset();
}

osg::ref_ptr<osgViewer::CompositeViewer> QOSGViewerWidget::GetViewer()
{
    return _osgviewer;
}

bool QOSGViewerWidget::HandleOSGKeyDown(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    int key = ea.getKey();
    if( !!_onKeyDown ) {
        if( _onKeyDown(key) ) {
            return true;
        }
    }

    if( key == 'f' ) {
        _osgDefaultManipulator->SetSeekMode(!_osgDefaultManipulator->InSeekMode());
    }
    return false;
}

void QOSGViewerWidget::RestoreCursor()
{
    osgViewer::Viewer::Windows windows;
    _osgviewer->getWindows(windows);
    for(osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end(); ++itr) {
        // can do (*itr)->setCursor(osgViewer::GraphicsWindow::HandCursor), but cursors are limited so have to use Qt
        QCursor _currentCursor;
        if( _bIsSelectiveActive ) {
            if( _draggerName == "TranslateTrackballDragger" ) {
                _currentCursor = QCursor(Qt::ArrowCursor);
            }
            else {
                QPixmap pixmap(":/images/no_edit.png");
                _currentCursor = QCursor(pixmap.scaled(QSize(32,32)), Qt::KeepAspectRatio);
            }
        }
        else {
            // need a custom cursor
            _currentCursor = QCursor(QPixmap(":/images/rotation-icon.png"));
        }
        this->setCursor(_currentCursor);
    }
}

void QOSGViewerWidget::ActivateSelection(bool active)
{
    _bIsSelectiveActive = active;
    RestoreCursor();
}

void QOSGViewerWidget::SetDraggerMode(const std::string& draggerName)
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

void QOSGViewerWidget::SelectItem(KinBodyItemPtr item, KinBody::JointPtr joint)
{
    if( _selectedItem != item ) {
        if( !!_selectedItem ) {
            _selectedItem->SetGrab(false);
        }
        _selectedItem = item;
        if( !!_selectedItem ) {
            _selectedItem->SetGrab(true);
        }

        if (!!_selectedItem) {
            _AddDraggerToObject(_draggerName, _selectedItem, joint);
        }
        else {
            // no dragger so clear?
            _ClearDragger();
        }
    }
}

void QOSGViewerWidget::SelectItemFromName(const std::string& name)
{
    SelectItem(GetItemFromName(name));
}

void QOSGViewerWidget::SetSceneData()
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

void QOSGViewerWidget::ResetViewToHome()
{
    SetHome();
    _osgview->home();
}

void QOSGViewerWidget::SetHome()
{
    if (!!_osgLightsGroup) {
        const osg::BoundingSphere& bs = _osgSceneRoot->getBound();
        _osgview->getCameraManipulator()->setHomePosition(osg::Vec3d(1.5*bs.radius(),0,1.5*bs.radius()),bs.center(),osg::Vec3d(0.0,0.0,1.0));
        _osgview->home();
    }
}

void QOSGViewerWidget::SetLight(bool enabled)
{
    _bLightOn = enabled;
    SetSceneData();
}

void QOSGViewerWidget::SetFacesMode(bool enabled)
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

void QOSGViewerWidget::SetPolygonMode(int mode)
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

void QOSGViewerWidget::SetWire(OSGNodePtr node)
{
    osg::ref_ptr<osg::PolygonMode> poly(new osg::PolygonMode());
    osg::ref_ptr<osg::ShadeModel> sm(new osg::ShadeModel());

    poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
    sm->setMode(osg::ShadeModel::SMOOTH);

    node->getOrCreateStateSet()->setAttribute(poly.get());
    node->getOrCreateStateSet()->setAttribute(sm.get());
}

void QOSGViewerWidget::HandleRayPick(const osgUtil::LineSegmentIntersector::Intersection& intersection, int buttonPressed, int modkeymask)
{
    if (intersection.nodePath.empty()) {
        _strRayInfoText.clear();
        _UpdateHUDText();
        if( buttonPressed ) {
            _osgDefaultManipulator->SetSeekMode(false);
        }
        if( _bIsSelectiveActive && buttonPressed ) {
            SelectOSGLink(OSGNodePtr(), modkeymask);
        }
    }
    else {
        OSGNodePtr node = intersection.nodePath.back();

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
                KinBody::Link::GeometryPtr geom = item->GetGeomFromOSG(node);
                std::string geomname;
                if( !!geom ) {
                    geomname = geom->GetName();
                }
                _strRayInfoText = str(boost::format("mouse on %s:%s:%s: (%.5f, %.5f, %.5f), n=(%.5f, %.5f, %.5f)")%item->GetName()%linkname%geomname%pos.x()%pos.y()%pos.z()%normal.x()%normal.y()%normal.z());
            }
            else {
                _strRayInfoText.clear();
            }
            _UpdateHUDText();
        }
    }
}

void QOSGViewerWidget::UpdateFromOSG()
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

void QOSGViewerWidget::SelectOSGLink(OSGNodePtr node, int modkeymask)
{
    if (!node) {
        if( !(modkeymask & osgGA::GUIEventAdapter::MODKEY_CTRL) ) {
            // user clicked on empty region, so remove selection
            SelectItem(KinBodyItemPtr());
        }
        return;
    }

    KinBody::JointPtr joint;
    KinBodyItemPtr item = FindKinBodyItemFromOSGNode(node);
    if (!!item) {
        KinBody::LinkPtr link = item->GetLinkFromOSG(node);

        // Find joint of a link name given
        joint = _FindJoint(item, link);
        if( (modkeymask & osgGA::GUIEventAdapter::MODKEY_CTRL) ) {
        }
        else {
            // select new body
            SelectItem(item, joint);
        }
    }

    if( !item ) {
        if( !(modkeymask & osgGA::GUIEventAdapter::MODKEY_CTRL) ) {
            // user clicked on empty region, so remove selection
            _ClearDragger();
        }
    }
}

void QOSGViewerWidget::StartTrackingNode(OSGNodePtr node, const std::string& trackInfoText, const osg::Vec3d& offset, double trackDistance, const osg::Vec3d& worldUpVector)
{
    _strTrackInfoText = str(boost::format("Tracking %s")%trackInfoText);
    SetCurrentCameraManipulator(_osgTrackModeManipulator.get());
    _osgTrackModeManipulator->StartTrackingNode(node.get(), offset, trackDistance, GetCamera(), osg::Vec3d(0,0,1));
}

void QOSGViewerWidget::StopTrackingNode()
{
    _strTrackInfoText = "";
    RestoreDefaultManipulator();
}

void QOSGViewerWidget::_ClearDragger()
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

void QOSGViewerWidget::SetUserHUDText(const std::string& text)
{
    _strUserText = text;
    _UpdateHUDText();
}

void QOSGViewerWidget::_UpdateHUDText()
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
    if( _strTrackInfoText.size() > 0 ) {
        if( s.size() > 0 ) {
            s += "\n";
        }
        s += _strTrackInfoText;
    }
    _osgHudText->setText(s);
}

void QOSGViewerWidget::SetNearPlane(double nearplane)
{
    _zNear = nearplane;
    if( _osgview->getCamera()->getProjectionMatrix()(2,3) == 0 ) {
        // orthogonal
        double left, right, bottom, top, zNear, zFar;
        _osgview->getCamera()->getProjectionMatrixAsOrtho(left, right, bottom, top, zNear, zFar);
        _osgview->getCamera()->setProjectionMatrixAsOrtho(left, right, bottom, top, nearplane, 10000.0 * nearplane);
    }
    else {
        double fovy, aspectRatio, zNear, zFar;
        _osgview->getCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
        _osgview->getCamera()->setProjectionMatrixAsPerspective(fovy, aspectRatio, nearplane, 10000.0 * nearplane);
    }
}

double QOSGViewerWidget::GetCameraNearPlane()
{
    return _zNear;
}

void QOSGViewerWidget::SetViewType(int isorthogonal)
{
    int width = _osgview->getCamera()->getViewport()->width();
    int height = _osgview->getCamera()->getViewport()->height();
    double aspect = static_cast<double>(width)/static_cast<double>(height);
    if( isorthogonal ) {
        double distance = 0.5*_osgDefaultManipulator->getDistance();
        _osgview->getCamera()->setProjectionMatrixAsOrtho(-distance, distance, -distance/aspect, distance/aspect, _zNear, _zNear * 10000.0);
    }
    else {
        _osgview->getCamera()->setProjectionMatrixAsPerspective(45.0f, aspect, _zNear, _zNear * 10000.0);
    }
}

void QOSGViewerWidget::SetViewport(int width, int height)
{
    float scale = this->devicePixelRatio();
    _osgview->getCamera()->setViewport(0,0,width*scale,height*scale);
    _osghudview->getCamera()->setViewport(0,0,width*scale,height*scale);
    _osghudview->getCamera()->setProjectionMatrix(osg::Matrix::ortho(-width*scale/2, width*scale/2, -height*scale/2, height*scale/2, 0.01/_metersinunit, 100.0/_metersinunit));

    osgViewer::Viewer::Windows windows;
    _osgviewer->getWindows(windows);
    for (osgViewer::Viewer::Windows::iterator itr = windows.begin(); itr != windows.end(); ++itr) {
        osgViewer::GraphicsWindowEmbedded *gw = dynamic_cast<osgViewer::GraphicsWindowEmbedded *>(*itr);
        gw->getEventQueue()->windowResize(this->x() * scale, this->y() * scale, width * scale, height * scale);
        gw->resized(this->x() * scale, this->y() * scale, width * scale, height * scale);
    }

    osg::Camera *camera = _osgview->getCamera();
    camera->setViewport(0, 0, width * scale, height * scale);
    osg::Camera *hudcamera = _osghudview->getCamera();
    hudcamera->setViewport(0, 0, width * scale, height * scale);

    double textheight = 12*scale;
    _osgHudText->setPosition(osg::Vec3(-width*scale/2+10, height*scale/2-textheight, -50));
    _osgHudText->setCharacterSize(textheight);
    UpdateHUDAxisTransform(width, height);
}

void QOSGViewerWidget::UpdateHUDAxisTransform(int width, int height)
{
    float scale = this->devicePixelRatio();
    osg::Matrix m = GetCurrentCameraManipulator()->getInverseMatrix();
    m.setTrans(width*scale/2 - 40, -height*scale/2 + 40, -50);
    _osgWorldAxis->setMatrix(m);
}

void QOSGViewerWidget::Zoom(float factor)
{
    // Ortho
    if ( _osgview->getCamera()->getProjectionMatrix()(2,3) == 0 ) {
        const int width = _osgview->getCamera()->getViewport()->width();
        const int height = _osgview->getCamera()->getViewport()->height();
        const double aspect = static_cast<double>(width)/static_cast<double>(height);
        const double nearplane = GetCameraNearPlane();
        const double distance = 0.5 * _osgDefaultManipulator->getDistance() / factor;

        _osgview->getCamera()->setProjectionMatrixAsOrtho(-distance, distance, -distance/aspect, distance/aspect, nearplane, 10000*nearplane);
    } else {
        _osgDefaultManipulator->setDistance(_osgDefaultManipulator->getDistance() / factor);
        _osgTrackModeManipulator->setDistance(_osgTrackModeManipulator->getDistance() / factor);
    }
}

void QOSGViewerWidget::SetTextureCubeMap(const std::string& posx, const std::string& negx, const std::string& posy,
                                         const std::string& negy, const std::string& posz, const std::string& negz)
{
    _osgSkybox->setTextureCubeMap(posx, negx, posy, negy, posz, negz);
}

void QOSGViewerWidget::_SetupCamera(osg::ref_ptr<osg::Camera> camera, osg::ref_ptr<osgViewer::View> view,
                                    osg::ref_ptr<osg::Camera> hudcamera, osg::ref_ptr<osgViewer::View> hudview)
{
    view->setCamera( camera.get() );
    hudview->setCamera( hudcamera.get() );
    _osgviewer->addView( view.get() );
    _osgviewer->addView( hudview.get() );

    _osgDefaultManipulator = new OpenRAVETrackball(this);
    _osgDefaultManipulator->setWheelZoomFactor(0.2);
    _osgDefaultManipulator->setAllowThrow(false);
    view->setCameraManipulator( _osgDefaultManipulator.get() );

    _osgTrackModeManipulator = new OpenRAVETracker();
    _osgTrackModeManipulator->setWheelZoomFactor(0.2);
    _osgTrackModeManipulator->setAllowThrow(false);
    _osgTrackModeManipulator->setRotationMode(osgGA::NodeTrackerManipulator::TRACKBALL);

    _osgCameraHUD = new osg::MatrixTransform();
    hudcamera->addChild( _osgCameraHUD.get() );
    _osgCameraHUD->setMatrix(osg::Matrix::identity());

    _osgGraphicWindow = dynamic_cast<osgViewer::GraphicsWindowEmbedded*>( camera->getGraphicsContext() );
    hudcamera->setGraphicsContext(_osgGraphicWindow);
    hudcamera->setViewport(0,0,_osgGraphicWindow->getTraits()->width, _osgGraphicWindow->getTraits()->height);
}

osg::ref_ptr<osg::Camera> QOSGViewerWidget::_CreateCamera( int x, int y, int w, int h, double metersinunit)
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
    camera->setGraphicsContext(new osgViewer::GraphicsWindowEmbedded(traits.get()));

    camera->setClearColor(osg::Vec4(0.95, 0.95, 0.95, 1.0));
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    _zNear = 0.01/metersinunit;
    camera->setProjectionMatrixAsPerspective(45.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), _zNear, 100.0/metersinunit);
    camera->setCullingMode(camera->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING); // need this for allowing small points with zero bunding voluem to be displayed correctly
    return camera;
}

osg::ref_ptr<osg::Camera> QOSGViewerWidget::_CreateHUDCamera( int x, int y, int w, int h, double metersinunit)
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
    camera->setCullingMode(camera->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING); // need this for allowing small points with zero bunding voluem to be displayed correctly
    return camera;
}

KinBodyItemPtr QOSGViewerWidget::GetItemFromName(const std::string& name)
{
    KinBodyPtr pbody = _penv->GetKinBody(name);
    return GetItemFromKinBody(pbody);
}

KinBodyItemPtr QOSGViewerWidget::GetItemFromKinBody(KinBodyPtr kinBody)
{
    return boost::dynamic_pointer_cast<KinBodyItem>(kinBody->GetUserData(_userdatakey));
}

KinBodyItemPtr QOSGViewerWidget::FindKinBodyItemFromOSGNode(OSGNodePtr node)
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


KinBody::JointPtr QOSGViewerWidget::_FindJoint(KinBodyItemPtr pitem, KinBody::LinkPtr link)
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

osg::ref_ptr<osg::Material> QOSGViewerWidget::_CreateSimpleMaterial(osg::Vec4 color)
{
    osg::ref_ptr<osg::Material> material(new osg::Material());
    material->setDiffuse(osg::Material::FRONT,  osg::Vec4(0.0, 0.0, 0.0, 1.0));
    material->setEmission(osg::Material::FRONT, color);
    return material;
}

osg::ref_ptr<osg::Light> QOSGViewerWidget::_CreateLight(osg::Vec4 color, int lightid)
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

osg::ref_ptr<osg::Light> QOSGViewerWidget::_CreateAmbientLight(osg::Vec4 color, int lightid)
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

void QOSGViewerWidget::_InitializeLights(int nlights)
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

//void QOSGViewerWidget::_UpdateFromOSG()
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

osg::Camera *QOSGViewerWidget::GetCamera()
{
    return _osgview->getCamera();
}

osg::ref_ptr<osgGA::CameraManipulator> QOSGViewerWidget::GetCurrentCameraManipulator()
{
    return _osgview->getCameraManipulator();
}

void QOSGViewerWidget::SetCurrentCameraManipulator(osgGA::CameraManipulator* manipulator)
{
    _osgview->setCameraManipulator(manipulator);
}

void QOSGViewerWidget::RestoreDefaultManipulator()
{
    if(_osgview->getCameraManipulator() == _osgDefaultManipulator.get()) {
        return;
    }
    // copy matrix from trackManipulator to default one so change of manipulators occurs seamless
    // SetCurrentCameraManipulator must be called before setByMatrix in order to take effect
    SetCurrentCameraManipulator(_osgDefaultManipulator.get());
    _osgDefaultManipulator->setByMatrix(_osgTrackModeManipulator->getMatrix());
}

osg::ref_ptr<osgGA::TrackballManipulator> QOSGViewerWidget::GetDefaultCameraManipulator() {
    return osg::dynamic_pointer_cast<osgGA::TrackballManipulator>(_osgDefaultManipulator);
}

osg::ref_ptr<osgGA::NodeTrackerManipulator> QOSGViewerWidget::GetTrackModeManipulator() {
    return osg::dynamic_pointer_cast<osgGA::NodeTrackerManipulator>(_osgTrackModeManipulator);
}

OSGMatrixTransformPtr QOSGViewerWidget::GetCameraHUD()
{
    return _osgCameraHUD;
}

void QOSGViewerWidget::_StoreMatrixTransform()
{
    _viewCameraMatrix = _osgview->getCamera()->getViewMatrix();
}

void QOSGViewerWidget::_LoadMatrixTransform()
{
    _osgview->getCamera()->setViewMatrix(_viewCameraMatrix);
}

std::vector<osg::ref_ptr<osgManipulator::Dragger> > QOSGViewerWidget::_CreateDragger(const std::string& draggerName)
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

OSGNodePtr QOSGViewerWidget::_AddDraggerToObject(const std::string& draggerName, KinBodyItemPtr item, KinBody::JointPtr pjoint)
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

    if( draggerName == "RotateCylinderDragger" && !!pjoint ) {
        OSGMatrixTransformPtr osglinktrans = item->GetOSGLink(pjoint->GetHierarchyChildLink()->GetIndex());
        selectedmatrix = osglinktrans->getMatrix();
        _osgSelectedNodeByDragger = osglinktrans->getParent(0);
    }
    else {
        _osgSelectedNodeByDragger = item->GetOSGRoot();
        selectedmatrix = item->GetOSGRoot()->getMatrix();
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

void QOSGViewerWidget::initializeGL() {
    resizeGL(width(), height());
}

void QOSGViewerWidget::paintGL()
{
    try {
        _osgviewer->frame(); // osgViewer::CompositeViewer
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("got exception in paint event: %s", ex.what());
    }
}

void QOSGViewerWidget::resizeGL(int width, int height)
{
    SetViewport(width, height);
}

void QOSGViewerWidget::mouseMoveEvent(QMouseEvent *event)
{
    float scale = this->devicePixelRatio();
    SetKeyboardModifiers(event);
    _osgGraphicWindow->getEventQueue()->mouseMotion(event->x() * scale, event->y() * scale);
}

void QOSGViewerWidget::GetSwitchedButtonValue(unsigned int &button) {
    if (this->_bSwitchMouseLeftMiddleButton && button < 3) {
        // left button = 1
        // middle button = 2
        button = (button << 1) % 3;
    }
}

void QOSGViewerWidget::mousePressEvent(QMouseEvent *event)
{
    float scale = this->devicePixelRatio();
    unsigned int button = qtOSGKeyEventTranslator.GetOSGButtonValue(event);
    SetKeyboardModifiers(event);
    GetSwitchedButtonValue(button);
    _osgGraphicWindow->getEventQueue()->mouseButtonPress(event->x() * scale, event->y() * scale, button);
}

void QOSGViewerWidget::mouseReleaseEvent(QMouseEvent *event)
{
    float scale = this->devicePixelRatio();
    unsigned int button = qtOSGKeyEventTranslator.GetOSGButtonValue(event);
    SetKeyboardModifiers(event);
    GetSwitchedButtonValue(button);
    _osgGraphicWindow->getEventQueue()->mouseButtonRelease(event->x() * scale, event->y() * scale, button);
}

void QOSGViewerWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    float scale = this->devicePixelRatio();
    unsigned int button = qtOSGKeyEventTranslator.GetOSGButtonValue(event);
    SetKeyboardModifiers(event);
    GetSwitchedButtonValue(button);
    // OSG's event loop has 2 kinds of timestamp, one is frame reference time and the other is up time
    // in order to properly handle event related to frame e.g. animation. need to create event using frame referece time
    _osgGraphicWindow->getEventQueue()->mouseDoubleButtonPress(event->x() * scale, event->y() * scale, button,
                                                               _osgviewer->getFrameStamp()->getReferenceTime());
}

void QOSGViewerWidget::wheelEvent(QWheelEvent *event)
{
    int delta = event->delta();
    osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?
                                                     osgGA::GUIEventAdapter::SCROLL_UP
                                                     : osgGA::GUIEventAdapter::SCROLL_DOWN;
    SetKeyboardModifiers(event);
    _osgGraphicWindow->getEventQueue()->mouseScroll(motion);
}

void QOSGViewerWidget::keyPressEvent(QKeyEvent *event)
{
    SetKeyboardModifiers(event);
    _osgGraphicWindow->getEventQueue()->keyPress(qtOSGKeyEventTranslator.GetOSGKeyValue(event));

}

void QOSGViewerWidget::keyReleaseEvent(QKeyEvent *event)
{
    SetKeyboardModifiers(event);
    if (event->isAutoRepeat()) {
        event->ignore();
    } else {
        SetKeyboardModifiers(event);
        _osgGraphicWindow->getEventQueue()->keyRelease(qtOSGKeyEventTranslator.GetOSGKeyValue(event));
    }
}

bool QOSGViewerWidget::event(QEvent *event)
{
    bool handled = QOpenGLWidget::event(event);
    this->update();
    return handled;
}

void QOSGViewerWidget::SetKeyboardModifiers(QInputEvent *event)
{
    int modifierKeys = event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier);
    unsigned int mask = 0;
    if ( modifierKeys & Qt::ShiftModifier ) mask |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
    if ( modifierKeys & Qt::ControlModifier ) mask |= osgGA::GUIEventAdapter::MODKEY_CTRL;
    if ( modifierKeys & Qt::AltModifier ) mask |= osgGA::GUIEventAdapter::MODKEY_ALT;
    _osgGraphicWindow->getEventQueue()->getCurrentEventState()->setModKeyMask(mask);
}


} // end namespace qtosgrave
