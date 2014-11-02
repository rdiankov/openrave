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
#ifndef OPENRAVE_QTOSG_VIEWER_H
#define OPENRAVE_QTOSG_VIEWER_H
//
#include "qtosg.h"
#include <QMainWindow>

#include "treemodel.h"

namespace qtosgrave {

/// \brief class of the entire viewer that periodically syncs with the openrave environment.
class QtOSGViewer : public QMainWindow, public ViewerBase
{
    Q_OBJECT;
public:
    QtOSGViewer(EnvironmentBasePtr penv, std::istream& sinput);
    bool isSimpleView();
    void setSimpleView(bool state);

    /// \brief goes into the main loop
    virtual int main(bool bShow);

    /// \brief destroys the main loop
    virtual void quitmainloop();

    virtual void Show(int showtype);

    virtual bool GetFractionOccluded(KinBodyPtr pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded);

    /// Retries a 24bit RGB image of dimensions width and height from the current scene
    /// extrinsic is the rotation and translation of the camera
    /// pKK is 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK);
    virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension);
    virtual void SetCamera(const RaveTransform<float>& trans, float focalDistance=0);
//  virtual void SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup);
    virtual RaveTransform<float> GetCameraTransform() const;
    virtual geometry::RaveCameraIntrinsics<float> GetCameraIntrinsics() const;
    virtual SensorBase::CameraIntrinsics GetCameraIntrinsics2() const;

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

    /// \brief override base class
    virtual void customEvent(QEvent * e);

    //    virtual void UpdateCameraTransform();
    //    static void _PlayCB(void *userData, SoSensor *);
    //
    //    virtual void resize ( int w, int h);
    //    virtual void resize ( const QSize & qs);
    //

    virtual void SetSize(int w, int h);
    virtual void Move(int x, int y);

    /// \brief Set title of the viewer window
    virtual void SetName(const string& name);

    /// \brief notified when a body has been removed from the environment
    virtual void RemoveKinBody(KinBodyPtr pbody) {
        if( !!pbody ) {
            pbody->RemoveUserData("qtosg");
        }
    }

    virtual void Reset();
    virtual void SetBkgndColor(const RaveVector<float>& color);

    virtual void StartPlaybackTimer();
    virtual void StopPlaybackTimer();

    /// \brief Reads model from file and loads it in viewer
    virtual bool LoadModel(const string& filename);

    /// \brief updates all render objects from the internal openrave classes
    void UpdateFromModel();

    /// \brief Set Sync environment
    virtual void SetEnvironmentSync(bool bUpdate);

    /// \brief Synchronize environment
    virtual void EnvironmentSync();

    virtual UserDataPtr RegisterItemSelectionCallback(const ItemSelectionCallbackFn& fncallback);
    virtual UserDataPtr RegisterViewerThreadCallback(const ViewerThreadCallbackFn& fncallback);

    /// \brief Locks environment
    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> LockEnvironment(uint64_t timeout=50000,bool bUpdateEnvironment = true);

//    class EnvMessage : public boost::enable_shared_from_this<EnvMessage>
//    {
//public:
//        EnvMessage(QtOSGViewerPtr pviewer, void** ppreturn, bool bWaitForMutex);
//        virtual ~EnvMessage();
//
//        /// execute the command in the caller
//        /// \param bGuiThread if true, then calling from gui thread, so execute directly
//        virtual void callerexecute(bool bGuiThread);
//        /// execute the command in the viewer
//        virtual void viewerexecute();
//
//        virtual void releasemutex()
//        {
//            _plock.reset();
//        }
//
//protected:
//        boost::weak_ptr<QtOSGViewer> _pviewer;
//        void** _ppreturn;
//        boost::mutex _mutex;
//        boost::shared_ptr<boost::mutex::scoped_lock> _plock;
//    };
//    typedef boost::shared_ptr<EnvMessage> EnvMessagePtr;
//    typedef boost::shared_ptr<EnvMessage const> EnvMessageConstPtr;

public slots:

    void LoadEnvironment();

    /// \brief Adds models to current scene
    void ImportEnvironment();

    /// \brief Saves the environment into a file
    void SaveEnvironment();
    void multiWidget();
    void simpleWidget();

    /// \brief Refresh the screen with a new frame. Reads the scene from OpenRAVE Core. on timer?
    void _Refresh();
    
    /// Set model home position in viewer
    void ResetViewToHome();

    /// \brief executed when menu item light is clicked to change the lights in the scene.
    void _ProcessLightChange();
    void polygonMode();

    //  \brief Sets COUNTER CLOCKWISE and CLOCKWISE polygons
    void _ProcessFacesModeChange();

    /// Sets or reset bounding box
    void _ProcessBoundingBox();

    /// \brief Sets or reset axes
    void axes();

    /// \brief Event clicked of pointer, hand, bound and axes buttons
    void _ProcessPointerGroupClicked(int button);

    /// \brief Event clicked of bound and axes button
    void _ProcessDraggerGroupClicked(int button);

    /// \brief Slot to listen scene list events
    void _OnObjectTreeClick(QTreeWidgetItem* item,int num);

protected:

    inline QtOSGViewerPtr shared_viewer() {
        return boost::static_pointer_cast<QtOSGViewer>(shared_from_this());
    }
    inline QtOSGViewerConstPtr shared_viewer_const() const {
        return boost::static_pointer_cast<QtOSGViewer const>(shared_from_this());
    }

    virtual void _InitGUI(bool bCreateStatusBar);
    
    /// \brief Update model and camera transform
    virtual void _UpdateEnvironment(float fTimeElapsed);
    virtual bool _ForceUpdatePublishedBodies();

    /// \brief Reset update from model
    virtual void _Reset();

    /// \brief reset the camera depending on its mode
    virtual void _UpdateCameraTransform(float fTimeElapsed);

    /// \brief Actions that achieve buttons and menus
    void _CreateActions();

    /// \brief Create menus
    void _CreateMenus();

    /// \brief Set Buttons and Icons in the ToolBar
    void _CreateToolsBar();

    /// \brief Repaint widgets
    void _RepaintWidgets(osg::ref_ptr<osg::Group>);

    /// \brief Create StatusBar and Set a Message
    void _CreateStatusBar();
    void mouseDoubleClickEvent(QMouseEvent *e);

    /// \brief Create info panels
    void _CreateDockWidgets();

    /// \brief Create and set main options of object tree
    QTreeWidget* _CreateObjectTree();

    /// \brief Fills object tree with robot info
    void _FillObjectTree(QTreeWidget *tw);
    
    void _DeleteItemCallback(Item* pItem);

    // Message Queue
    //list<EnvMessagePtr> _listMessages;
    list<Item*> _listRemoveItems; ///< raw points of items to be deleted, triggered from _DeleteItemCallback
    boost::mutex _mutexItems, _mutexUpdating, _mutexMouseMove; ///< mutex protected messages
    mutable boost::mutex _mutexMessages;
    
    // Rendering
    osg::ref_ptr<osg::Group> _ivRoot;        ///< root node
    osg::ref_ptr<osg::Node> _selectedNode;
    osg::ref_ptr<osgManipulator::Dragger>    _pdragger;
    
    std::string _userdatakey; ///< the key to use for KinBody::GetUserData and KinBody::SetUserData
    std::map<KinBodyPtr, KinBodyItemPtr> _mapbodies;    ///< all the bodies created
    ItemPtr _pSelectedItem;     ///< the currently selected item
    
    RaveTransform<float>     _initSelectionTrans;       ///< initial tarnsformation of selected item
    RaveTransform<float> _Tcamera;
    geometry::RaveCameraIntrinsics<float> _camintrinsics;

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


    ViewGeometry _viewGeometryMode;

    std::list<UserDataWeakPtr> _listRegisteredItemSelectionCallbacks;
    std::list<UserDataWeakPtr> _listRegisteredViewerThreadCallbacks;

    //QSize getSize();
    QGridLayout* _qCentralLayout;
    QWidget* _qCentralWidget;
    ViewerWidget* _posgWidget; // cannot be shared_ptr since QMainWindow takes ownership of it internally
    //boost::shared_ptr<TreeModel> treeModel;

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
    bool _bLockEnvironment; ///< if true, should lock the environment.

    friend class ItemSelectionCallbackData;
    friend class ViewerThreadCallbackData;
    friend void DeleteItemCallbackSafe(QtOSGViewerWeakPtr, Item*);
};

}

#endif
