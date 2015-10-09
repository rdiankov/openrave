// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
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
#ifndef RAVE_QTCOINGUI_H
#define RAVE_QTCOINGUI_H

#ifndef _WIN32 // not a necessary define

#if QT_VERSION >= 0x040000 // check for qt4
#include <QtCore/QObject>
#else
#include <qobject.h>
#include <qaction.h>
#include <qmainwindow.h>
#include <qgroupbox.h>
#include <qlayout.h>
#include <qfiledialog.h>
#endif

#include <sys/time.h>
#include <time.h>

#endif

#include "qtcoin.h"

//#include <QtDeclarative/QDeclarativeExtensionPlugin>
//#include <QtDeclarative/qdeclarative.h>
//#include <QtGui/QGraphicsProxyWidget>

/// Render and GUI engine. Can be used to simulate a camera sensor
///
/// ViewerBase holds __plugin, which is a reference to the shared object that loads the qt4 library. Ideally we
/// would have ViewerBase as the first class so we can assure destruction of qt before __plugin goes out of scope; unfortunately qt moc requires QMainWindows to be declared first (http://doc.qt.digia.com/qt/moc.html)
class QtCoinViewer : public QMainWindow, public ViewerBase
{
    Q_OBJECT

public:
    QtCoinViewer(EnvironmentBasePtr penv, std::istream& sinput);
    virtual ~QtCoinViewer();

    //! the kinds of toggle switches
    enum ToggleEnum { TOGGLE_GRID = 0,   TOGGLE_PATH,     TOGGLE_COLLISION,
                      TOGGLE_RECORD,     TOGGLE_REPLAN,   TOGGLE_DYNAMICS,
                      TOGGLE_TIME_ELPSD, TOGGLE_SENSING,  TOGGLE_MEMORY,
                      TOGGLE_HW_PLAN,    TOGGLE_BM_SHARE, TOGGLE_MANIP_TRACK,
                      TOGGLE_TRACKING,   TOGGLE_FRUSTUM,  TOGGLE_IK_HANDLE,
                      TOGGLE_GRAVITY,    TOGGLE_CONTROL,  TOGGLE_CENTROID,
                      TOGGLE_AA,         TOGGLE_GOAL,     TOGGLE_VEL_PRED,
                      TOGGLE_VISION,     TOGGLE_HILIT,    TOGGLE_REC_MOT,
                      TOGGLE_VISDATA,    TOGGLE_SKELETON, TOGGLE_PSERVER,
                      NUM_TOGGLES };

    //! the different views
    enum ViewEnum { PLAN_VIEW = 0, MAIN_VIEW,   CAMERA_VIEW,  VIS_VIEW,
                    FOLLOW_VIEW,   ORTHO_VIEW,  STEREO_VIEW, NUM_VIEWS };

    // other methods relating to the user-interface
    virtual SoQtExaminerViewer* GetViewer() const {
        return _pviewer;
    }

    virtual int main(bool bShow);
    virtual void quitmainloop();

    virtual void DumpIvRoot(const char* filename, bool bBinaryFile);

    // methods relating to playback
    static void GlobAdvanceFrame(void*, SoSensor*);
    virtual void AdvanceFrame(bool bForward);

    static void GlobVideoFrame(void*, SoSensor*);
    virtual void _VideoFrame();

    virtual void Select(SoNode *pNode) {
        _ivRoot->select(pNode);
    }

    virtual bool AltDown()     {
        return _altDown[0] || _altDown[1];
    }
    virtual bool ControlDown() {
        return _ctrlDown[0] || _ctrlDown[1];
    }

    virtual void deselect();

    virtual SoPerspectiveCamera* GetCamera() {
        return _ivCamera;
    }
    virtual SoSelection* GetRoot() {
        return _ivRoot;
    }
    virtual SoSeparator* GetBodiesRoot() {
        return _ivBodies;
    }

    virtual void _UpdateCameraTransform(float fTimeElapsed);
    static void _PlayCB(void *userData, SoSensor *);

    virtual void resize ( int w, int h);
    virtual void resize ( const QSize & qs);

    virtual void SetSize(int w, int h);
    virtual void Move(int x, int y);
    virtual void Show(int showtype);
    virtual void SetName(const string& name);
    virtual const std::string& GetName() const {
        return _name;
    }

    virtual bool LoadModel(const string& filename);

    virtual bool ForceUpdatePublishedBodies();

    /// updates all render objects from the internal openrave classes
    virtual void UpdateFromModel();

    virtual void Reset();
    virtual boost::shared_ptr<void> LockGUI();

    /// \brief notified when a body has been removed from the environment
    virtual void RemoveKinBody(KinBodyPtr pbody) {
        if( !!pbody ) {
            pbody->RemoveUserData("qtcoinviewer");
        }
    }

    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK);

    virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension);
    virtual void SetCamera(const RaveTransform<float>& trans, float focalDistance=0);
    virtual float GetCameraDistanceToFocus() const;
    virtual void SetBkgndColor(const RaveVector<float>& color);

    virtual void PrintCamera();
    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0);
    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0, bool bhasalpha=false);

    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color);
    virtual GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents);
    virtual GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture);
    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color);
    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors);

    virtual void SetEnvironmentSync(bool bUpdate);
    virtual void EnvironmentSync();
    virtual uint8_t* _GetVideoFrame();

    virtual RaveTransform<float> GetCameraTransform() const;
    virtual geometry::RaveCameraIntrinsics<float> GetCameraIntrinsics() const;
    virtual SensorBase::CameraIntrinsics GetCameraIntrinsics2() const;

    virtual void customEvent(QEvent * e);

    virtual UserDataPtr RegisterItemSelectionCallback(const ItemSelectionCallbackFn& fncallback);
    virtual UserDataPtr RegisterViewerImageCallback(const ViewerImageCallbackFn& fncallback);
    virtual UserDataPtr RegisterViewerThreadCallback(const ViewerThreadCallbackFn& fncallback);
    virtual void _DeleteItemCallback(Item* pItem)
    {
        boost::mutex::scoped_lock lock(_mutexItems);
        pItem->PrepForDeletion();
        _listRemoveItems.push_back(pItem);
    }

    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> LockEnvironment(uint64_t timeout=50000,bool bUpdateEnvironment = true);

public slots:

    // menu items
    void LoadEnvironment();
    void ImportEnvironment();
    void SaveEnvironment();
    void Quit();

    void ViewCameraParams();
    void ViewGeometryChanged(QAction*);
    void ViewDebugLevelChanged(QAction*);
    void ViewToggleFPS(bool on);
    void ViewToggleFeedBack(bool on);

    void RecordRealtimeVideo(bool on);
    void RecordSimtimeVideo(bool on);
    void VideoCodecChanged(QAction*);
    void ToggleSimulation(bool on);
    void CollisionCheckerChanged(QAction*);
    void PhysicsEngineChanged(QAction*);

    void DynamicSelfCollision(bool on);
    void DynamicGravity();

    void UpdateInterfaces();
    void InterfaceSendCommand(QAction*);
    void About();

public:
    class EnvMessage : public boost::enable_shared_from_this<EnvMessage>
    {
public:
        EnvMessage(QtCoinViewerPtr pviewer, void** ppreturn, bool bWaitForMutex);
        virtual ~EnvMessage();

        /// execute the command in the caller
        /// \param bGuiThread if true, then calling from gui thread, so execute directly
        virtual void callerexecute(bool bGuiThread);
        /// execute the command in the viewer
        virtual void viewerexecute();

        virtual void releasemutex()
        {
            _plock.reset();
        }

protected:
        boost::weak_ptr<QtCoinViewer> _pviewer;
        void** _ppreturn;
        boost::mutex _mutex;
        boost::shared_ptr<boost::mutex::scoped_lock> _plock;
    };
    typedef boost::shared_ptr<EnvMessage> EnvMessagePtr;
    typedef boost::shared_ptr<EnvMessage const> EnvMessageConstPtr;

protected:
    void _InitConstructor(std::istream& sinput);

    class PrivateGraphHandle : public GraphHandle
    {
public:
        PrivateGraphHandle(boost::weak_ptr<QtCoinViewer> wviewer, SoSwitch* handle) : _handle(handle), _wviewer(wviewer) {
            BOOST_ASSERT(_handle!=NULL);
        }
        virtual ~PrivateGraphHandle() {
            boost::shared_ptr<QtCoinViewer> viewer = _wviewer.lock();
            if( !!viewer ) {
                viewer->closegraph(_handle);
            }
        }

        virtual void SetTransform(const RaveTransform<float>& t)
        {
            boost::shared_ptr<QtCoinViewer> viewer = _wviewer.lock();
            if( !!viewer ) {
                viewer->SetGraphTransform(_handle,t);
            }
        }

        virtual void SetShow(bool bshow)
        {
            boost::shared_ptr<QtCoinViewer> viewer = _wviewer.lock();
            if( !!viewer ) {
                viewer->SetGraphShow(_handle,bshow);
            }
        }

        SoSwitch* _handle;
        boost::weak_ptr<QtCoinViewer> _wviewer;
    };

    inline QtCoinViewerPtr shared_viewer() {
        return boost::dynamic_pointer_cast<QtCoinViewer>(shared_from_this());
    }
    inline QtCoinViewerWeakPtr weak_viewer() {
        return QtCoinViewerWeakPtr(shared_viewer());
    }
    inline QtCoinViewerConstPtr shared_viewer_const() const {
        return boost::dynamic_pointer_cast<QtCoinViewer const>(shared_from_this());
    }

    static void mousemove_cb(void * userdata, SoEventCallback * node);
    void _mousemove_cb(SoEventCallback * node);

    virtual void _SetSize(int w, int h);
    virtual void _Move(int x, int y);
    virtual void _Show(int showtype);
    virtual void _SetName(const string& ptitle);

    virtual bool _GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK);
    virtual bool _WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension);
    virtual void _SetCamera(const RaveTransform<float>& trans, float focalDistance);

    virtual void closegraph(SoSwitch* handle);
    virtual void SetGraphTransform(SoSwitch* handle, const RaveTransform<float>& t);
    virtual void SetGraphShow(SoSwitch* handle, bool bshow);

    virtual SoSwitch* _createhandle();
    virtual void* _plot3(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color);
    virtual void* _plot3(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, bool bhasalpha);

    virtual void* _drawspheres(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color);
    virtual void* _drawspheres(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, bool bhasalpha);

    virtual void* _drawlinestrip(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual void* _drawlinestrip(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual void* _drawlinelist(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual void* _drawlinelist(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual void* _drawarrow(SoSwitch* handle, const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color);

    virtual void* _drawbox(SoSwitch* handle, const RaveVector<float>& vpos, const RaveVector<float>& vextents);
    virtual void* _drawplane(SoSwitch* handle, const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture);
    virtual void* _drawtrimesh(SoSwitch* handle, const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color);
    virtual void* _drawtrimesh(SoSwitch* handle, const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors);

    virtual void _deselect();

    virtual void _SetMaterial(SoGroup* pparent, const RaveVector<float>& color);
    virtual void _SetMaterial(SoGroup* pparent, const boost::multi_array<float,2>& colors);
    virtual void _SetTriangleMesh(SoSeparator* pparent, const float* ppoints, int stride, const int* pIndices, int numTriangles);
    virtual void _Reset();
    virtual void _SetBkgndColor(const RaveVector<float>& color);
    virtual void _closegraph(SoSwitch* handle);
    virtual void _SetGraphTransform(SoSwitch* handle, const RaveTransform<float>& t);
    virtual void _SetGraphShow(SoSwitch* handle, bool bshow);

    virtual void _StartPlaybackTimer();
    virtual void _StopPlaybackTimer();
    void _RecordSetup(bool bOn, bool bRealtimeVideo);

    /// find the appropriate collision checker to select, otherwise select None
    void _UpdateCollisionChecker();

    /// find the appropriate physics engine to select, otherwise select None
    void _UpdatePhysicsEngine();
    void _UpdateToggleSimulation();

    virtual void InitOffscreenRenderer();
    virtual void SetupMenus();

    virtual void _UpdateEnvironment(float fTimeElapsed);

    bool _SetFiguresInCamera(ostream& sout, istream& sinput);
    bool _SetFeedbackVisibility(ostream& sout, istream& sinput);
    bool _CommandResize(ostream& sout, istream& sinput);
    bool _SaveBodyLinkToVRMLCommand(ostream& sout, istream& sinput);
    bool _SetNearPlaneCommand(ostream& sout, istream& sinput);
    bool _StartViewerLoopCommand(ostream& sout, istream& sinput);
    bool _ShowCommand(ostream& sout, istream& sinput);
    bool _TrackLinkCommand(ostream& sout, istream& sinput);
    bool _TrackManipulatorCommand(ostream& sout, istream& sinput);
    bool _SetTrackingAngleToUpCommand(ostream& sout, istream& sinput);

    void _SetNearPlane(dReal nearplane);

    // selection and deselection handling
    static void _SelectHandler(void *, class SoPath *);
    static void _DeselectHandler(void *, class SoPath *);
    virtual bool _HandleSelection(SoPath *path);
    virtual bool _HandleDeselection(SoNode *node);

    static void _KeyHandler(void *, class SoEventCallback *);

    int _nFrameNum;     ///< frame number for recording
    string _strMouseMove;     ///< mouse move message
    // Message Queue
    list<EnvMessagePtr> _listMessages;
    list<Item*> _listRemoveItems;
    boost::mutex _mutexItems, _mutexUpdating, _mutexMouseMove;     ///< mutex protected messages
    mutable boost::mutex _mutexMessages;

    QVBoxLayout * vlayout;
    QGroupBox * view1;

    // Rendering
    SoSelection*        _ivRoot;            ///< root node of the inventor scene
    SoSeparator*        _ivBodies;          ///< all the environment bodies are stored in this node
    SoPerspectiveCamera* _ivCamera;           ///< the camera itself
    SoDrawStyle*         _ivStyle;
    SoTimerSensor*      _timerSensor;       ///< used for animation callback

    SoTimerSensor*      _timerVideo;        ///< used for video recording

    // the GUI
    SoQtExaminerViewer* _pviewer;
    QActionGroup* _pToggleDebug, *_pSelectedCollisionChecker, *_pSelectedPhysicsEngine, *_pActionSendCommand;
    QAction* _pToggleSimulation, *_pToggleSelfCollision;
    QMenu* _pMenuSendCommand;

    SoNode*       _selectedNode;
    boost::shared_ptr<IvDragger>    _pdragger;
    std::list< boost::shared_ptr<IvDragger> > _plistdraggers;     /// draggers drawn
    SoEventCallback* _eventKeyboardCB;

    boost::array<SoText2*,2> _messageNodes;
    SoTranslation* _messageShadowTranslation;

    bool _altDown[2];
    bool _ctrlDown[2];
    int _VideoFrameRate;

    std::string _name;
    std::map<KinBodyPtr, KinBodyItemPtr> _mapbodies;        ///< all the bodies created

    ItemPtr _pSelectedItem;                   ///< the currently selected item
    KinBody::LinkWeakPtr _pMouseOverLink;
    RaveVector<float> _vMouseSurfacePosition,_vMouseRayDirection, _vMouseSurfaceNormal;

    /// for movie recording
    SoOffscreenRenderer _ivOffscreen;
    SoSeparator *_pFigureRoot;
    bool _bCanRenderOffscreen;
    int _videocodec;

    RaveTransform<float>     _initSelectionTrans;           ///< initial tarnsformation of selected item
    RaveTransform<float> _Tcamera; ///< current camera transform, read-only
    float _focalDistance;  ///< current focal distance of the camera, read-only
    geometry::RaveCameraIntrinsics<float> _camintrinsics;

    unsigned int _fb;
    unsigned int _rb;
    unsigned int _outTex;
    unsigned int _queryBodyAlone;
    unsigned int _queryWithEnv;
    unsigned int _sampleCountAlone;
    unsigned int _sampleCountWithEnv;
    int _available;

    bool _bLockEnvironment;
    boost::mutex _mutexUpdateModels, _mutexCallbacks;
    boost::condition _condUpdateModels;     ///< signaled everytime environment models are updated
    boost::mutex _mutexGUI;
    bool _bInIdleThread;

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

    /// tracking parameters
    //@{
    KinBody::LinkPtr _ptrackinglink; ///< current link tracking
    Transform _tTrackingLinkRelative; ///< relative transform in the _ptrackinglink coord system  that should be tracking.
    RobotBase::ManipulatorPtr _ptrackingmanip; ///< current manipulator tracking
    Transform _tTrackingCameraVelocity; ///< camera velocity

    //@}

    // data relating to playback
    bool _bStopped;
    bool _bTimeInitialized;
    int _fileFrame;
    int _recordInterval;
    time_t _prevSec;
    long _prevMicSec;

    bool _bTimeElapsed;
    //bool _bRecordMotion;
    ModuleBasePtr _pvideorecorder;
    bool _bAutoSetCamera;
    bool _bRenderFiguresInCamera;
    // ode thread related
    int _nQuitMainLoop;
    bool _bUpdateEnvironment;
    ViewGeometry _viewGeometryMode;

    std::list<UserDataWeakPtr> _listRegisteredItemSelectionCallbacks;
    std::list<UserDataWeakPtr> _listRegisteredViewerImageCallbacks;
    std::list<UserDataWeakPtr> _listRegisteredViewerThreadCallbacks;

    /// width and height of offscreen-rendered image
    unsigned int _nRenderWidth;
    unsigned int _nRenderHeight;
    float _fTrackAngleToUp; ///< tilt a little when looking at the point

    friend class EnvMessage;
    friend class ViewerSetSizeMessage;
    friend class ViewerMoveMessage;
    friend class ViewerSetNameMessage;
    friend class GetCameraImageMessage;
    friend class WriteCameraImageMessage;
    friend class SetCameraMessage;
    friend class DrawMessage;
    friend class DrawArrowMessage;
    friend class DrawBoxMessage;
    friend class DrawPlaneMessage;
    friend class DrawTriMeshMessage;
    friend class DrawTriMeshColorMessage;
    friend class CloseGraphMessage;
    friend class DeselectMessage;
    friend class ResetMessage;
    friend class SetBkgndColorMessage;
    friend class StartPlaybackTimerMessage;
    friend class StopPlaybackTimerMessage;
    friend class SetGraphTransformMessage;
    friend class SetGraphShowMessage;
    friend class SetNearPlaneMessage;
    friend class ViewerShowMessage;

    friend class ItemSelectionCallbackData;
    friend class ViewerImageCallbackData;
    friend class ViewerThreadCallbackData;

    // qml
    friend class ScreenRendererWidget;
};

/*class ScreenRendererWidget : public QWidget
   {
    Q_OBJECT

   public:
    ScreenRendererWidget();

   public slots:
    void Animate();

   protected:
     void paintEvent(QPaintEvent *event);

   private:
    EnvironmentBasePtr _penv;
    boost::shared_ptr<QtCoinViewer> _openraveviewer;
    std::vector<uint8_t> _memory;
    //QBasicTimer _timer;
   };

   class QtCoinViewerProxy : public QGraphicsProxyWidget
   {
    Q_OBJECT
    //Q_PROPERTY(QString text READ text WRITE setText NOTIFY textChanged)

   public:
    QtCoinViewerProxy(QGraphicsItem* parent = 0);
    virtual ~QtCoinViewerProxy() {
    }


   //    QString text() const
   //    {
   //        return widget->text();
   //    }
   //
   //    void setText(const QString& text)
   //    {
   //        if (text != widget->text()) {
   //            widget->setText(text);
   //            emit textChanged();
   //        }
   //    }
   //
   //Q_SIGNALS:
   //    void clicked(bool);
   //    void textChanged();

   };

   class QOpenRAVEWidgetsPlugin : public QDeclarativeExtensionPlugin
   {
    Q_OBJECT
   public:
    void registerTypes(const char *uri)
    {
        //RAVELOG_INFO("registering %s to OpenRAVECoinViewer\n", uri);
        qmlRegisterType<QtCoinViewerProxy>(uri, 1, 0, "OpenRAVECoinViewer");
    }
   };*/

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(SoSeparator)
BOOST_TYPEOF_REGISTER_TYPE(SoTransform)
BOOST_TYPEOF_REGISTER_TYPE(QtCoinViewer::EnvMessage)
#endif

#endif
