// -*- coding: utf-8 -*-
// Copyright (C) 2012 Gustavo Puche, Rosen Diankov, OpenGrasp Team
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
#ifndef OPENRAVE_QTOSG_VIEWER_H
#define OPENRAVE_QTOSG_VIEWER_H
//
#include "qtosg.h"
#include <QMainWindow>

#include "treemodel.h"
#include <iostream>

#include <osg/ArgumentParser>
#include "osgviewerQtContext.h"

//class QAction;
//class QMenu;
//class QOsgWidget;
//class QScrollArea;
//class QGridLayout;
//class QTreeView;
//class QMouseEvent;
//class QComboBox;
//class QButtonGroup;
//class TreeModel;

namespace qtosgrave {

/// \brief class of the entire viewer that periodically syncs with the openrave environment.
class QtOSGViewer : public QMainWindow, public ViewerBase
{
    Q_OBJECT;
public:
    QtOSGViewer(EnvironmentBasePtr penv, std::istream& sinput);
    bool isSimpleView();
    void setSimpleView(bool state);

    /// reset the camera depending on its mode
    virtual void UpdateCameraTransform();

    /// goes into the main loop
    virtual int main(bool bShow);
    /// destroys the main loop
    virtual void quitmainloop();

    virtual bool GetFractionOccluded(KinBodyPtr pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded);

    /// Retries a 24bit RGB image of dimensions width and height from the current scene
    /// extrinsic is the rotation and translation of the camera
    /// pKK is 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK);
    virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension);
    virtual void SetCamera(const RaveTransform<float>& trans, float focalDistance=0);
//  virtual void SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup);
    virtual RaveTransform<float> GetCameraTransform();

    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0);
    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0, bool bhasalpha=false);

    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color);

    virtual GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture);
    virtual GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents);

    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color);
    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors);

    virtual void closegraph(void* handle);
    virtual void _Deselect();

    //	TODO : Specific QtOsgRave functions
    //virtual osg::Camera* GetCamera() { return _ivCamera; }
    virtual osg::ref_ptr<osg::Group> GetRoot() {
        return _ivRoot;
    }

    //    virtual void UpdateCameraTransform();
    //    static void _PlayCB(void *userData, SoSensor *);
    //
    //    virtual void resize ( int w, int h);
    //    virtual void resize ( const QSize & qs);
    //

    virtual void SetSize(int w, int h);
    virtual void Move(int x, int y);
    virtual void SetName(const string& name);

    /// \brief notified when a body has been removed from the environment
    virtual void RemoveKinBody(KinBodyPtr pbody) {
        if( !!pbody ) {
            pbody->RemoveUserData("qtosg");
        }
    }

    //
    //
    //    /// updates all render objects from the internal openrave classes
    //    virtual void UpdateFromModel();

    virtual void Reset();
    virtual void SetBkgndColor(const RaveVector<float>& color);

    virtual void StartPlaybackTimer();
    virtual void StopPlaybackTimer();

    virtual bool LoadModel(const string& filename);

    /// updates all render objects from the internal openrave classes
    void UpdateFromModel();

    virtual void SetEnvironmentSync(bool bUpdate);
    virtual void EnvironmentSync();

    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> LockEnvironment(uint64_t timeout=50000,bool bUpdateEnvironment = true);

public slots:

    void LoadEnvironment();
    void ImportEnvironment();
    void SaveEnvironment();
    void multiWidget();
    void simpleWidget();

    /// \brief Refresh the screen with a new frame. Reads the scene from OpenRAVE Core. on timer?
    void Refresh();
    
    void home();
    void light();
    void polygonMode();
    void facesMode();
    void boundingBox();
    void axes();
    void pointerGroupClicked(int button);
    void draggerGroupClicked(int button);

    void sceneListClicked(QTreeWidgetItem* item,int num);

protected:

    inline QtOSGViewerPtr shared_viewer() {
        return boost::static_pointer_cast<QtOSGViewer>(shared_from_this());
    }
    inline QtOSGViewerConstPtr shared_viewer_const() const {
        return boost::static_pointer_cast<QtOSGViewer const>(shared_from_this());
    }

    virtual void _InitGUI();
    virtual void _UpdateEnvironment();
    virtual bool _ForceUpdatePublishedBodies();
    virtual void _Reset();

    /// \brief Actions that achieve buttons and menus
    void CreateActions();
    void createMenus();
    void createScrollArea();
    void createToolsBar();
    void _RepaintWidgets(boost::shared_ptr<osg::Group>);

    /// \brief Create layouts
    void CreateLayouts();
    void CreateStatusBar();
    void mouseDoubleClickEvent(QMouseEvent *e);
    void createDockWidgets();

    //  Create and set main options of object tree
    QTreeWidget* createObjectTree();

    //  Fills object tree with robot info
    void fillObjectTree(QTreeWidget *tw);
    
    virtual void _DeleteItemCallback(Item* pItem)
    {
        boost::mutex::scoped_lock lock(_mutexItems);
        pItem->PrepForDeletion();
        _listRemoveItems.push_back(pItem);
    }
    
    QApplication  *application; ///< root qt application?

    // Message Queue
    //list<EnvMessagePtr> _listMessages;
    list<Item*> _listRemoveItems; ///< raw points of items to be deleted, triggered from _DeleteItemCallback
    boost::mutex _mutexItems, _mutexMessages, _mutexUpdating, _mutexMouseMove; ///< mutex protected messages
    
    // Rendering
    osg::ref_ptr<osg::Group> _ivRoot;        ///< root node
    osg::ref_ptr<osg::Node> _selectedNode;
    osg::ref_ptr<osgManipulator::Dragger>    _pdragger;
    
    std::map<KinBodyPtr, KinBodyItemPtr> _mapbodies;    ///< all the bodies created
    ItemPtr _pSelectedItem;     ///< the currently selected item
    
    RaveTransform<float>     _initSelectionTrans;       ///< initial tarnsformation of selected item
    RaveTransform<float> Tcam;
    std::string _name;

    unsigned int _fb;
    unsigned int _rb;
    unsigned int _outTex;
    unsigned int _queryBodyAlone;
    unsigned int _queryWithEnv;
    unsigned int _sampleCountAlone;
    unsigned int _sampleCountWithEnv;
    int _available;
    

    boost::mutex _mutexUpdateModels, _mutexCallbacks;
    boost::condition _condUpdateModels; ///< signaled everytime environment models are updated
    boost::mutex _mutexGUI;
    bool _bInIdleThread;


    timeval timestruct;

    PhysicsEngineBasePtr pphysics;

    // toggle switches
    bool _bModelsUpdated;
    bool _bDisplayGrid;
    bool _bDisplayIK;
    bool _bDisplayFPS;
    bool _bDisplayFeedBack;
    bool _bJointHilit;
    bool _bDynamicReplan;
    bool _bVelPredict;
    bool _bDynSim;
    bool _bGravity;
    bool _bControl;

    bool _bSensing;
    bool _bMemory;
    bool _bHardwarePlan;
    bool _bShareBitmap;
    bool _bManipTracking;
    bool _bAntialiasing;

    // data relating to playback
    bool _bStopped;
    bool _bTimeInitialized;
    int _fileFrame;
    int _recordInterval;
    time_t _prevSec;
    long _prevMicSec;

    bool _bTimeElapsed;
    //bool _bRecordMotion;
    bool _bSaveVideo;
    bool _bRealtimeVideo;
    bool _bAutoSetCamera;

    // ode thread related
    bool _bQuitMainLoop;
    bool _bUpdateEnvironment;

    ViewGeometry _viewGeometryMode;

    std::list<std::pair<int,ViewerCallbackFn > > _listRegisteredCallbacks; ///< callbacks to call when particular properties of the body change.

    //QSize getSize();
    QGridLayout* _qCentralLayout;
    QWidget* _qCentralWidget;
    boost::shared_ptr<ViewerWidget> _osgWidget;
    boost::shared_ptr<TreeModel> treeModel;

    QMenu* fileMenu;
    QMenu* viewMenu;
    QMenu* helpMenu;
    QMenu* animation;
    QMenu* options;
    QMenu* dynamics;

    QAction* exitAct;
    QAction* loadAct;
    QAction* multiAct;
    QAction* simpleAct;

    QAction* importAct;
    QAction* saveAct;
    QAction* viewCamAct;
    QAction* viewColAct;
    QAction* pubilshAct;
    QAction* printAct;
    QAction* showAct;
    QAction* playAct;
    QAction* stopAct;
    QAction* recordAct;
    QAction* odeAct;
    QAction* selfAct;
    QAction* applyAct;
    QAction* aboutAct;
    QAction* pauseAct;
    QAction* puntAct;
    QAction* AxesAct;
    QAction* houseAct;
    QAction* smoothAct;
    QAction* flatAct;
    QAction* lightAct;
    QAction* wireAct;
    QAction* facesAct;
    QAction* bboxAct;

    QToolBar* fileToolBar;
    QToolBar* actionToolBar;
    QToolBar* physicsToolBar;
    QToolBar* toolsBar;

    QComboBox* physicsComboBox;
    
    QString fileName;
    QTreeView* _qtree;

    QActionGroup* shapeGroup;
    QButtonGroup* pointerTypeGroup;
    QButtonGroup* buttonGroup;
    QButtonGroup* draggerTypeGroup;
    
    QTreeWidget* objectTree; ///< Tree of robots, joints and links
    QTreeWidget* detailsTree; ///< Details panel object
    
    QTimer _timer;
    
    bool _bLockEnvironment; ///< if true, should lock the environment.
};

}

#endif
