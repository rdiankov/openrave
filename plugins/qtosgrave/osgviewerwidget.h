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
#ifndef OPENRAVE_QTOSG_VIEWERCONTEXT_H
#define OPENRAVE_QTOSG_VIEWERCONTEXT_H

#include "qtosg.h"

#include "osgpick.h"

#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/ShadeModel>
#include <osgDB/ReadFile>
#include <osg/FrontFace>
#include <osg/CullFace>
#include <iostream>

#include <osgGA/NodeTrackerManipulator>

#include <osgQt/GraphicsWindowQt>
#include <osgManipulator/CommandManager>
#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>

#include <osg/PositionAttitudeTransform>

namespace qtosgrave {

using namespace OpenRAVE;
using namespace osgQt;

/// \brief  Class of the openscene graph 3d viewer
class ViewerWidget : public QWidget, public osgViewer::CompositeViewer
{
public:
    ViewerWidget(EnvironmentBasePtr penv, const std::string& userdatakey, const boost::function<bool(int)>& onKeyDown=boost::function<bool(int)>());

    /// \brief Draws bounding box around actual kinbody
    void DrawBoundingBox(bool pressed);

    /// \brief Active selection
    void SelectActive(bool active);

    /// \brief possible names include TrackballDragger, TranslateAxisDragger
    void SetDraggerMode(const std::string& draggerName);
    
    /// \brief Select robot or kinbody from screen
    void SelectRobot(std::string name);

    /// \brief  Sets scene data node in all viewers
    void SetSceneData();

    /// \brief  Reset viewer to original position
    void ResetViewToHome();

    /// \brief Reset viewer to original position
    void SetHome();

    /// \brief Light button
    void SetLight(bool enabled);
    //  Cull face
    void SetFacesMode(bool enabled);

    /// \brief Sets poligon mode (SMOOTH, FLAT or WIRED)
    void SetPolygonMode(int mode);

    /// \brief Set wire view to a node
    void SetWire(OSGNodePtr node);

    OSGGroupPtr GetSceneRoot() const {
        return _osgSceneRoot;
    }

    OSGGroupPtr GetFigureRoot() const {
        return _osgFigureRoot;
    }

    /// \brief Gets transform matrix of a given link
    OSGMatrixTransformPtr GetLinkTransform(std::string& robotName, KinBody::LinkPtr link);

    /// \brief Select the link picked
    void SelectLink(OSGNodePtr node, int modkeymask=0);

    void SetViewport(int width, int height);
    osg::Camera *GetCamera();
    osg::ref_ptr<osgGA::CameraManipulator> GetCameraManipulator();
    OSGMatrixTransformPtr GetCameraHUD();
    
protected:
    bool HandleOSGKeyDown(int);

    /// \brief Clear dragger from the viewer
    void _ClearDragger();

    /// \brief Create a viewer widget
    QWidget* _AddViewWidget( osg::ref_ptr<osg::Camera> camera, osg::ref_ptr<osgViewer::View> view, osg::ref_ptr<osg::Camera> hudcamera, osg::ref_ptr<osgViewer::View> hudview );

    /// \brief Create Open GL Context
    osg::ref_ptr<osg::Camera> _CreateCamera( int x, int y, int w, int h );
    osg::ref_ptr<osg::Camera> _CreateHUDCamera(int x, int y, int w, int h );

    /// \brief Find an OSG Node with the name given
    OSGNodePtr _FindNamedNode(const std::string& searchName, OSGNodePtr currNode);

    /// \brief Print nodes of scenegraph
    //void _ShowSceneGraph(const std::string& currLevel,osg::Node* currNode);

    /// \brief Get link children and store list in global variable _linkChildren
    void _GetLinkChildren( std::string & robotName, KinBody::LinkPtr link, std::vector<KinBody::LinkPtr> vlinks);
    
    /// \brief Find node of Robot for the link picked
    OSGNodePtr _FindRobot(OSGNodePtr node);

    /// \brief Find link initial node. Group node
    OSGNodePtr _FindLinkParent(OSGNodePtr node);

    /// \brief Propagate transform to link children
    void _PropagateTransforms();

    /// \brief Find joint into OpenRAVE core
    KinBody::JointPtr _FindJoint(std::string & robotName,std::string &linkName);

    //  Lighting Stuff //

    osg::ref_ptr<osg::Material> _CreateSimpleMaterial(osg::Vec4 color);
    osg::ref_ptr<osg::Light> _CreateLight(osg::Vec4 color, int lightid);
    osg::ref_ptr<osg::Light> _CreateAmbientLight(osg::Vec4 color, int lightid);
    
    /// \brief Initialize lighting
    void _InitializeLights(int nlights);

    /// \brief Updates joint values from viewer to OpenRAVE core. uses GetPublishedBodies, so doesn't need environment lock.
    virtual void _UpdateFromOSG();

    /// \brief Stores matrix transform
    void _StoreMatrixTransform();

    /// \brief Loads the stored matrix transform to the camera
    void _LoadMatrixTransform();

    /// \brief Create a dragger with a name given
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > _CreateDragger(const std::string& name);
    
    /// \brief Create a manipulator over an object pased
    OSGNodePtr _AddDraggerToObject(OSGNodePtr object, const std::string& name);

    OSGNodePtr _AddDraggerToObject(const std::string& robotName, OSGNodePtr object, const std::string& name, KinBody::JointPtr joint);
    virtual void paintEvent( QPaintEvent* event );

//    //  Mouse release event
//    virtual void mouseReleaseEvent(QMouseEvent *e)
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
//    /// Mouse double click event handler
//    virtual void mouseDoubleClickEvent(QMouseEvent *e)
//    {
//      doubleClickPressed = true;
//    }


//    //  Flags to apply anchor (if true) to the dragger position
//    std::map<std::string,bool> _needAnchor;

    OSGGroupPtr _osgSceneRoot; ///< root scene node
    OSGGroupPtr _osgFigureRoot; ///< the node that all the figures are drawn into
    OSGMatrixTransformPtr _osgWorldAxis; ///< the node that draws the rgb axes on the lower right corner
    
    std::string _userdatakey; ///< the key to use for KinBody::GetUserData and KinBody::SetUserData
    OSGGroupPtr _osgLightsGroup; ///< Scene Node with lights
    OSGGroupPtr _osgLightsGroupData; ///< Scene Data to romove after each repaint
    OSGGroupPtr _osgDraggerRoot; ///< Parent of dragger and selection
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > _draggers; ///< There is only one dragger at the same time
    OSGMatrixTransformPtr _draggerMatrix; ///< Transform applied by dragger
    OSGNodePtr _selected; ///< Object selected by dragger
    OSGMatrixTransformPtr _osgCameraHUD; ///< MatrixTransform node that gets displayed in the heads up display
    
    std::string _actualKinbody; ///< Kinematic body selected or robot
    std::string _draggerName; ///< Actual dragger selected
    
    osg::ref_ptr<OSGPickHandler> _picker; ///<  Pick handler for joint selection
    osg::ref_ptr<osgGA::GUIEventHandler> _keyhandler; ///<  Pick handler for joint selection
    osg::Matrixf _viewCameraMatrix; ///< stored matrix transform

    std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > _vLightTransform;
    osg::ref_ptr<osg::StateSet> _lightStateSet;
    osg::ref_ptr<osgViewer::View> _osgview;
    osg::ref_ptr<osgViewer::View> _osghudview;
    osg::ref_ptr<osgGA::CameraManipulator> _osgCameraManipulator;
    
    QTimer _timer; ///< Timer for repaint
    EnvironmentBasePtr _penv;
    std::vector<OSGMatrixTransformPtr> _linkChildren; ///< List of link children

    boost::function<bool(int)> _onKeyDown; ///< call whenever key press is detected
    
    bool _bLightOn; ///< whether lights are on or not
};

}

#endif
