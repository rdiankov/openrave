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
#ifndef OPENRAVE_QTOSG_VIEWERCONTEXT_H
#define OPENRAVE_QTOSG_VIEWERCONTEXT_H

#include "qtosg.h"
#include "osgrenderitem.h"
#include "osgpick.h"

#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/PositionAttitudeTransform>
#include <osgManipulator/Dragger>
#include <iostream>

#include <osgQt/GraphicsWindowQt>

namespace qtosgrave {

using namespace OpenRAVE;
using namespace osgQt;

class OpenRAVETrackball;
    
/// \brief  Class of the openscene graph 3d viewer
class ViewerWidget : public QWidget, public osgViewer::CompositeViewer
{
public:
    ViewerWidget(EnvironmentBasePtr penv, const std::string& userdatakey, const boost::function<bool(int)>& onKeyDown=boost::function<bool(int)>(), double metersinunit=1);
    virtual ~ViewerWidget();
    
    /// \brief Draws bounding box around actual kinbody
    void DrawBoundingBox(bool pressed);

    /// \brief Active selection
    void ActivateSelection(bool active);

    /// \brief possible names include TrackballDragger, TranslateAxisDragger
    void SetDraggerMode(const std::string& draggerName);

    /// \brief sets up a dragger selection for a robot or kinbody item
    void SelectItem(KinBodyItemPtr item, KinBody::JointPtr joint=KinBody::JointPtr());

    void SelectItemFromName(const std::string& name);

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

    /// \brief set the viewport to perspective or orthogonal
    void SetViewType(int isorthogonal);

    /// \brief sets the near plane for the camera
    void SetNearPlane(double nearplane);

    /// \brief returns the near plane set on the camera
    double GetCameraNearPlane();
    
    /// \brief called when the qt window size changes
    void SetViewport(int width, int height, double metersinunit);

    /// \brief sets user-controlled hud text
    void SetUserHUDText(const std::string& text);

    /// \brief Set wire view to a node
    void SetWire(OSGNodePtr node);

    OSGGroupPtr GetSceneRoot() const {
        return _osgSceneRoot;
    }

    OSGGroupPtr GetFigureRoot() const {
        return _osgFigureRoot;
    }

    /// \brief called when the mouse is over a specified point
    ///
    void HandleRayPick(const osgUtil::LineSegmentIntersector::Intersection& intersection, int buttonPressed, int modkeymask=0);

    /// \brief handle case when link is selected
    void SelectOSGLink(OSGNodePtr node, int modkeymask);
    
    osg::Camera *GetCamera();
    osg::ref_ptr<osgGA::CameraManipulator> GetCameraManipulator();
    OSGMatrixTransformPtr GetCameraHUD();

    /// \brief Updates any changes in OSG to to OpenRAVE core.
    void UpdateFromOSG();

    /// \brief Find node of Robot for the link picked
    KinBodyItemPtr FindKinBodyItemFromOSGNode(OSGNodePtr node);

    /// \brief restores cursor to what it was originally set to
    void RestoreCursor();
    
protected:
    /// \brief handles a key press and looks at the modifier keys
    bool HandleOSGKeyDown(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    /// \brief Clear dragger from the viewer
    void _ClearDragger();

    /// \brief gather all the necessary text and updates it on the HUD control
    void _UpdateHUDText();
    
    /// \brief Create a viewer widget
    QWidget* _AddViewWidget( osg::ref_ptr<osg::Camera> camera, osg::ref_ptr<osgViewer::View> view, osg::ref_ptr<osg::Camera> hudcamera, osg::ref_ptr<osgViewer::View> hudview );

    /// \brief Create Open GL Context
    osg::ref_ptr<osg::Camera> _CreateCamera( int x, int y, int w, int h, double metersinunit );
    osg::ref_ptr<osg::Camera> _CreateHUDCamera(int x, int y, int w, int h, double metersinunit );

    KinBodyItemPtr _GetItemFromName(const std::string& name);

    /// \brief Find joint into OpenRAVE core
    KinBody::JointPtr _FindJoint(KinBodyItemPtr pitem, KinBody::LinkPtr link);

    //  Lighting Stuff //
    osg::ref_ptr<osg::Material> _CreateSimpleMaterial(osg::Vec4 color);
    osg::ref_ptr<osg::Light> _CreateLight(osg::Vec4 color, int lightid);
    osg::ref_ptr<osg::Light> _CreateAmbientLight(osg::Vec4 color, int lightid);

    /// \brief Initialize lighting
    void _InitializeLights(int nlights);

    /// \brief Stores matrix transform
    void _StoreMatrixTransform();

    /// \brief Loads the stored matrix transform to the camera
    void _LoadMatrixTransform();

    /// \brief Create a dragger with a name given
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > _CreateDragger(const std::string& name);

    /// \brief Create a manipulator over a render item
    ///
    /// \param draggerName the type of dragger to create
    /// \param joint if not empty, the joint to create the dragger over (ie for moving the joint value)
    OSGNodePtr _AddDraggerToObject(const std::string& draggerName, KinBodyItemPtr item, KinBody::JointPtr joint);
    virtual void paintEvent( QPaintEvent* event );

    OSGGroupPtr _osgSceneRoot; ///< root scene node
    OSGGroupPtr _osgFigureRoot; ///< the node that all the figures are drawn into
    OSGMatrixTransformPtr _osgWorldAxis; ///< the node that draws the rgb axes on the lower right corner

    std::string _userdatakey; ///< the key to use for KinBody::GetUserData and KinBody::SetUserData
    OSGGroupPtr _osgLightsGroup; ///< Scene Node with lights
    OSGGroupPtr _osgLightsGroupData; ///< Scene Data to romove after each repaint
    OSGGroupPtr _osgDraggerRoot; ///< Parent of dragger and selection
    std::vector<osg::ref_ptr<osgManipulator::Dragger> > _draggers; ///< There is only one dragger at the same time
    OSGMatrixTransformPtr _draggerMatrix; ///< Transform applied by dragger
    OSGGroupPtr _osgSelectedNodeByDragger; ///< Object selected by dragger
    OSGMatrixTransformPtr _osgCameraHUD; ///< MatrixTransform node that gets displayed in the heads up display

    KinBodyItemPtr _selectedItem; ///< render item selected
    std::string _draggerName; ///< Actual dragger selected

    osg::ref_ptr<OSGPickHandler> _picker; ///<  Pick handler for joint selection
    osg::ref_ptr<osgGA::GUIEventHandler> _keyhandler; ///<  Pick handler for joint selection
    osg::Matrixf _viewCameraMatrix; ///< stored matrix transform

    std::vector<osg::ref_ptr<osg::PositionAttitudeTransform> > _vLightTransform;
    osg::ref_ptr<osg::StateSet> _lightStateSet;
    osg::ref_ptr<osgViewer::View> _osgview;
    osg::ref_ptr<osgViewer::View> _osghudview;
    osg::ref_ptr<OpenRAVETrackball> _osgCameraManipulator;

    osg::ref_ptr<osgText::Text> _osgHudText; ///< the HUD text in the upper left corner
    std::string _strUserText, _strSelectedItemText, _strRayInfoText; ///< the user hud text

    QTimer _timer; ///< Timer for repaint
    EnvironmentBasePtr _penv;

    boost::function<bool(int)> _onKeyDown; ///< call whenever key press is detected

    bool _bLightOn; ///< whether lights are on or not
    bool _bIsSelectiveActive; ///< if true, then can select a new
};

}

#endif
