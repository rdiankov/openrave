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
#ifndef OPENRAVE_QTOSG_VIEWER_H
#define OPENRAVE_QTOSG_VIEWER_H

#include "qtosg.h"
#include "osgrenderitem.h"
#include <QLayout>
#include <QComboBox>
#include <QPushButton>
#include <QAbstractButton>
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QToolButton>
#include <QDockWidget>
#include <QToolBar>
#include "qtreemodel.h"

namespace qtosgrave {

class QOSGViewerWidget;
class QtOSGViewer;
typedef boost::shared_ptr<QtOSGViewer> QtOSGViewerPtr;
typedef boost::weak_ptr<QtOSGViewer> QtOSGViewerWeakPtr;
typedef boost::shared_ptr<QtOSGViewer const> QtOSGViewerConstPtr;

/// \brief class of the entire viewer that periodically syncs with the openrave environment.
class QtOSGViewer : public QMainWindow, public ViewerBase
{
    Q_OBJECT;
public:
    QtOSGViewer(EnvironmentBasePtr penv, std::istream& sinput);
    virtual ~QtOSGViewer();
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
    virtual float GetCameraDistanceToFocus() const;
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

    virtual void _Deselect();

    //	TODO : Specific QtOsgRave functions
    //virtual osg::Camera* GetCamera() { return _ivCamera; }

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

    /// \brief sets the zoom factor. only affects orthogonal view
    /// \param factor > 1.0 = Zoom in. < 1.0 = Zoom out
    virtual void Zoom(float factor);

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

public slots:

    void LoadEnvironment();

    /// \brief Adds models to current scene
    void ImportEnvironment();

    /// \brief Saves the environment into a file
    void SaveEnvironment();
    //void multiWidget();
    //void simpleWidget();

    /// \brief updates the screen with a new frame and runs viewer update logic. Also tries to update with the openrave environment
    void _UpdateViewerCallback();

    /// \brief called when application quits
    void _ProcessApplicationQuit();

    /// Set model home position in viewer
    void ResetViewToHome();

    /// \brief executed when menu item light is clicked to change the lights in the scene.
    //void _ProcessLightChange();

    /// \brief changes the perspective view
    void _ProcessPerspectiveViewChange();

    /// \brief show an about dialog
    void _ProcessAboutDialog();

    void _SetDebugLevelInfo();

    void _SetDebugLevelDebug();

    void _SetDebugLevelVerbose();

    /// \brief change camera to see xy plane
    void _ChangeViewToXY();

    /// \brief change camera to see xz plane
    void _ChangeViewToXZ();

    /// \brief change camera to see yz plane
    void _ChangeViewToYZ();

    void polygonMode();

    //  \brief Sets COUNTER CLOCKWISE and CLOCKWISE polygons
    //void _ProcessFacesModeChange();

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
//    void keyPressEvent(QKeyEvent* event)
//    {
//        RAVELOG_INFO("key pressed event\n");
//        QWidget::keyPressEvent(event);
//    }
//
//    void keyReleaseEvent(QKeyEvent* event)
//    {
//        RAVELOG_INFO("key released event\n");
//        QWidget::keyReleaseEvent(event);
//    }

    bool _HandleOSGKeyDown(int);

    /// \brief contains a function to be called inside the GUI thread.
    class GUIThreadFunction
    {
public:
        GUIThreadFunction(const boost::function<void()>& fn, bool isblocking=false) : _fn(fn), _bcalled(false), _bfinished(false), _bisblocking(isblocking) {
        }

        inline void Call() {
            // have to set finished at the end
            boost::shared_ptr<void> finishfn((void*) 0, boost::bind(&GUIThreadFunction::SetFinished, this));
            BOOST_ASSERT(!_bcalled);
            _bcalled = true;
            _fn();
        }

        /// \brief true if function has already been called
        inline bool IsCalled() const {
            return _bcalled;
        }

        inline bool IsFinished() const {
            return _bfinished;
        }

        inline bool IsBlocking() const {
            return _bisblocking;
        }

        void SetFinished() {
            _bfinished = true;
        }

private:

        boost::function<void()> _fn;
        bool _bcalled; ///< true if function already called
        bool _bfinished; ///< true if function processing is finished
        bool _bisblocking; ///< if true, then the caller is blocking until the function completes
    };
    typedef boost::shared_ptr<GUIThreadFunction> GUIThreadFunctionPtr;

    /// \brief implementation of graph handles for qtosg
    class PrivateGraphHandle : public GraphHandle
    {
public:
        PrivateGraphHandle(QtOSGViewerWeakPtr wviewer, OSGSwitchPtr handle) : _handle(handle), _wviewer(wviewer) {
            BOOST_ASSERT(_handle != NULL);
        }
        virtual ~PrivateGraphHandle() {
            boost::shared_ptr<QtOSGViewer> viewer = _wviewer.lock();
            if(!!viewer) {
                viewer->_PostToGUIThread(boost::bind(&QtOSGViewer::_CloseGraphHandle, viewer, _handle)); // _handle is copied, so it will maintain the reference
            }
        }

        virtual void SetTransform(const RaveTransform<float>& t)
        {
            boost::shared_ptr<QtOSGViewer> viewer = _wviewer.lock();
            if(!!viewer) {
                viewer->_PostToGUIThread(boost::bind(&QtOSGViewer::_SetGraphTransform, viewer, _handle, t)); // _handle is copied, so it will maintain the reference
            }
        }

        virtual void SetShow(bool bShow)
        {
            boost::shared_ptr<QtOSGViewer> viewer = _wviewer.lock();
            if(!!viewer) {
                viewer->_PostToGUIThread(boost::bind(&QtOSGViewer::_SetGraphShow, viewer, _handle, bShow)); // _handle is copied, so it will maintain the reference
            }
        }

        OSGSwitchPtr _handle;
        QtOSGViewerWeakPtr _wviewer;
    };

    inline QtOSGViewerPtr shared_viewer() {
        return boost::static_pointer_cast<QtOSGViewer>(shared_from_this());
    }
    inline QtOSGViewerWeakPtr weak_viewer() {
        return QtOSGViewerWeakPtr(shared_viewer());
    }
    inline QtOSGViewerConstPtr shared_viewer_const() const {
        return boost::static_pointer_cast<QtOSGViewer const>(shared_from_this());
    }

    /// \brief initializes osg view and the qt menus/dialog boxes
    virtual void _InitGUI(bool bCreateStatusBar, bool bCreateMenu);

    /// \brief Update model and camera transform
    virtual void _UpdateEnvironment(float fTimeElapsed);
    virtual bool _ForceUpdatePublishedBodies();

    /// \brief Reset update from model
    virtual void _Reset();

    /// \brief reset the camera depending on its mode
    virtual void _UpdateCameraTransform(float fTimeElapsed);
    virtual void _SetCameraTransform();

    virtual OSGSwitchPtr _CreateGraphHandle();
    virtual void _CloseGraphHandle(OSGSwitchPtr handle);
    virtual void _SetGraphTransform(OSGSwitchPtr handle, const RaveTransform<float> t);
    virtual void _SetGraphShow(OSGSwitchPtr handle, bool bShow);

    virtual void _Draw(OSGSwitchPtr handle, osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::Vec4Array> colors, osg::PrimitiveSet::Mode mode, osg::ref_ptr<osg::StateAttribute> attribute, bool bUsingTransparency=false);
    virtual void _DrawTriMesh(OSGSwitchPtr handle, osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::Vec4Array> colors, osg::ref_ptr<osg::DrawElementsUInt> osgindices, bool bUsingTransparency);
    virtual void _SetTriangleMesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, osg::ref_ptr<osg::Vec3Array> osgvertices, osg::ref_ptr<osg::DrawElementsUInt> osgindices);
    virtual void _DrawBox(OSGSwitchPtr handle, const RaveVector<float>& vpos, const RaveVector<float>& vextents, bool bUsingTransparency);
    virtual void _DrawPlane(OSGSwitchPtr handle, const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture);

    virtual void _SetCamera(RaveTransform<float> trans, float focalDistance);
    virtual void _SetCameraDistanceToFocus(float focalDistance);
    virtual void _UpdateDistanceToFocusFromCurrentManipulator();
    virtual void _SetTrackManipulatorToStopTracking();
    virtual bool _SetTrackManipulatorToTrackLink(KinBody::LinkPtr link, KinBodyItemPtr linksKinBodyItem, const RaveTransform<float>& linkRelativeTranslation);
    virtual void _SetItemVisualization(std::string& itemname, std::string& visualizationmode);
    virtual void _SetProjectionMode(const std::string& projectionMode);
    virtual void _SetBkgndColor(const RaveVector<float>& color);

    virtual void _SetName(const std::string& name);
    virtual void _Zoom(float factor);

    /// \brief posts a function to be executed in the GUI thread
    ///
    /// \param fn the function to execute
    /// \param block if true, will return once the function has been executed.
    void _PostToGUIThread(const boost::function<void()>& fn, bool block=false);

    /// \brief Actions that achieve buttons and menus
    void _CreateActions();

    /// \brief Create menus
    void _CreateMenus();

    /// \brief Set Buttons and Icons in the ToolBar
    void _CreateToolsBar();

    /// \brief Repaint widgets
    void _RepaintWidgets();

    /// \brief Create StatusBar and Set a Message
    void _CreateStatusBar();

    /// \brief Create info panels
    void _CreateDockWidgets();

    /// \brief Create and set main options of object tree
    QTreeWidget* _CreateObjectTree();

    /// \brief Create viewer control button
    void _CreateControlButtons();

    /// \brief Fills object tree with robot info
    void _FillObjectTree(QTreeWidget *tw);

    void _DeleteItemCallback(Item* pItem);

    bool _SetFiguresInCamera(ostream& sout, istream& sinput);
    bool _ShowWorldAxesCommand(ostream& sout, istream& sinput);
    bool _SetItemVisualizationCommand(ostream& sout, istream& sinput);
    bool _SetNearPlaneCommand(ostream& sout, istream& sinput);
    bool _SetTextureCubeMap(ostream& out, istream& sinput);
    bool _TrackLinkCommand(ostream& sout, istream& sinput);
    bool _TrackManipulatorCommand(ostream& sout, istream& sinput);
    bool _SetTrackingAngleToUpCommand(ostream& sout, istream& sinput);
    bool _StartViewerLoopCommand(ostream& sout, istream& sinput);
    bool _SetProjectionModeCommand(ostream& sout, istream& sinput);
    bool _ZoomCommand(ostream& sout, istream& sinput);

    //@{ Message Queue
    list<GUIThreadFunctionPtr> _listGUIFunctions; ///< list of GUI functions that should be called in the viewer update thread. protected by _mutexGUIFunctions
    mutable list<Item*> _listRemoveItems; ///< raw points of items to be deleted in the viewer update thread, triggered from _DeleteItemCallback. proteced by _mutexItems
    boost::mutex _mutexItems; ///< protects _listRemoveItems
    mutable boost::mutex _mutexGUIFunctions;
    mutable boost::condition _notifyGUIFunctionComplete; ///< signaled when a blocking _listGUIFunctions has been completed
    //@}

    //@{ osg rendering primitives
    OSGNodePtr _selectedNode;
    //@}

    std::string _userdatakey; ///< the key to use for KinBody::GetUserData and KinBody::SetUserData
    std::map<KinBodyPtr, KinBodyItemPtr> _mapbodies;    ///< mapping of all the bodies created
    ItemPtr _pSelectedItem;     ///< the currently selected item

    //@{ camera
    RaveTransform<float> _Tcamera; ///< current position of the camera representing the current view, updated periodically, read only.
    float _focalDistance;  ///< current focal distance of the camera, read-only
    geometry::RaveCameraIntrinsics<float> _camintrinsics; ///< intrinsics of the camera representing the current view, updated periodically, read only.
    //@}

    boost::mutex _mutexUpdating; ///< when inside an update function, even if just checking if the viewer should be updated, this will be locked.
    boost::mutex _mutexUpdateModels; ///< locked when osg environment is being updated from the underlying openrave environment
    boost::condition _condUpdateModels; ///< signaled everytime environment models are updated

    ViewGeometry _viewGeometryMode; ///< the visualization mode of the geometries

    //@{ callbacks
    boost::mutex _mutexCallbacks; ///< maintains lock on list of callsbacks viewer has to make like _listRegisteredViewerThreadCallbacks, _listRegisteredItemSelectionCallbacks
    std::list<UserDataWeakPtr> _listRegisteredItemSelectionCallbacks;
    std::list<UserDataWeakPtr> _listRegisteredViewerThreadCallbacks;
    //@}

    //@{ qt menus and dialog boxes
    QTimer _updateViewerTimer;

    QGridLayout* _qCentralLayout;
    QWidget* _qCentralWidget;
    QOSGViewerWidget* _posgWidget; // cannot be shared_ptr since QMainWindow takes ownership of it internally

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
    QAction* debugLevelInfoAct;
    QAction* debugLevelDebugAct;
    QAction* debugLevelVerboseAct;
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
    QAction* _qactChangeViewtoXY, *_qactChangeViewtoYZ, *_qactChangeViewtoXZ;
    //QAction* smoothAct;
    //QAction* flatAct;
    //QAction* lightAct;
    QAction* _qactPerspectiveView;
    QAction* wireAct;
    //QAction* facesAct;
    QAction* bboxAct;

    QToolBar* fileToolBar;
    QToolBar* actionToolBar;
    QToolBar* physicsToolBar;
    QToolBar* toolsBar;

    QComboBox* physicsComboBox;

    QTreeView* _qtree;

    //QActionGroup* shapeGroup;
    QButtonGroup* _pointerTypeGroup;
    QButtonGroup* buttonGroup;
    QButtonGroup* draggerTypeGroup;

    QTreeWidget* _qobjectTree; ///< Tree of robots, joints and links
    QTreeWidget* _qdetailsTree; ///< Details panel object
    //@}

    /// tracking parameters
    //@{
    KinBody::LinkPtr _ptrackinglink; ///< current link tracking
    Transform _tTrackingLinkRelative; ///< relative transform in the _ptrackinglink coord system  that should be tracking.
    RobotBase::ManipulatorPtr _ptrackingmanip; ///< current manipulator tracking
    Transform _tTrackingCameraVelocity; ///< camera velocity
    float _fTrackAngleToUp; ///< tilt a little when looking at the point
    //@}

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

    // control related
    bool _bUpdateEnvironment; ///< if true, should update the viewer to the openrave environment periodically
    bool _bLockEnvironment; ///< if true, should lock the environment when updating from it. Otherwise, the environment can assumed to be already locked in another thread that the viewer controls

    bool _bExternalLoop; ///< If true, the Qt loop is not started by qtosgviewer, which means qtosgviewer should not terminate the Qt loop during deallocation.
    int _nQuitMainLoop; ///< controls if the main loop's state. If 0, then nothing is initialized. If -1, then currently initializing/running. If 1, then currently quitting from the main loop. If 2, then successfully quit from the main loop.

    bool _bRenderFiguresInCamera;

    friend class ItemSelectionCallbackData;
    friend class ViewerThreadCallbackData;
    friend void DeleteItemCallbackSafe(QtOSGViewerWeakPtr, Item*);
};

}

#endif
