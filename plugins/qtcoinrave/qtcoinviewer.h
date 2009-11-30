// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
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

/// Render and GUI engine. Can be used to simulate a camera sensor
class QtCoinViewer : public QMainWindow, public RaveViewerBase
{
    Q_OBJECT

public:
    QtCoinViewer(EnvironmentBasePtr penv);
    ~QtCoinViewer();

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
    enum ViewEnum   { PLAN_VIEW = 0, MAIN_VIEW,   CAMERA_VIEW,  VIS_VIEW,
	        FOLLOW_VIEW,   ORTHO_VIEW,  STEREO_VIEW, NUM_VIEWS };

    // other methods relating to the user-interface
    virtual SoQtExaminerViewer* GetViewer() const { return _pviewer; }

    virtual int main(bool bShow);
    virtual void quitmainloop();

    virtual void DumpIvRoot(const char* filename, bool bBinaryFile);
    
    // methods relating to playback
    static void GlobAdvanceFrame(void*, SoSensor*);
    virtual void AdvanceFrame(bool bForward);

    static void GlobVideoFrame(void*, SoSensor*);
    virtual void VideoFrame();

    virtual void Select(SoNode *pNode) { _ivRoot->select(pNode); }
    
    virtual bool AltDown()     { return _altDown[0] || _altDown[1]; }
    virtual bool ControlDown() { return _ctrlDown[0] || _ctrlDown[1]; }

    virtual void deselect();

    virtual SoPerspectiveCamera* GetCamera() { return _ivCamera; }
    virtual SoSelection* GetRoot() { return _ivRoot; }

    virtual void UpdateCameraTransform();
    static void _PlayCB(void *userData, SoSensor *);
    
    virtual void resize ( int w, int h);
    virtual void resize ( const QSize & qs);

    virtual void ViewerSetSize(int w, int h);
    virtual void ViewerMove(int x, int y);
    virtual void ViewerSetTitle(const string& ptitle);
    
    virtual bool LoadModel(const string& filename);

    virtual bool ForceUpdatePublishedBodies();

    /// updates all render objects from the internal openrave classes
    virtual void UpdateFromModel();

    virtual void Reset();
    virtual boost::shared_ptr<void> LockGUI();

    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK);
    virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension);
    virtual void SetCamera(const RaveVector<float>& pos, const RaveVector<float>& quat);
    virtual void SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup);
    virtual void SetBkgndColor(const RaveVector<float>& color);

    virtual void PrintCamera();
    virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0);
    virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0);

    virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual void* drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color);
    virtual void* drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents);
    virtual void* drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color);
    virtual void closegraph(void* handle);

    virtual void SetEnvironmentSync(bool bUpdate);
    virtual void EnvironmentSync();

    virtual bool _RecordVideo();

    virtual RaveTransform<float> GetCameraTransform();

    virtual void UnregisterCallback(std::list<std::pair<int,ViewerCallbackFn > >::iterator it)
    {
        boost::mutex::scoped_lock lock(_mutexCallbacks);
        _listRegisteredCallbacks.erase(it);
        
    }
    virtual boost::shared_ptr<void> RegisterCallback(int properties, const ViewerCallbackFn& fncallback) {
        boost::mutex::scoped_lock lock(_mutexCallbacks);
        return boost::shared_ptr<void>((void*)1,boost::bind(&QtCoinViewer::UnregisterCallback,this,_listRegisteredCallbacks.insert(_listRegisteredCallbacks.end(),make_pair(properties,fncallback))));
    }

    virtual void _DeleteItemCallback(Item* pItem)
    {
        boost::mutex::scoped_lock lock(_mutexItems);
        _listRemoveItems.push_back(pItem);
    }

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
;
    void StartPlayback();
    void StopPlayback();

    void RecordRealtimeVideo(bool on);
    void RecordSimtimeVideo(bool on);
    void RecordSetup();

    void DynamicSimulation(bool on);
    void DynamicSelfCollision(bool on);
    void DynamicGravity(bool on);

    void About();

    void actionTriggered(QAction *action);

public:
    class EnvMessage : public boost::enable_shared_from_this<EnvMessage>
    {
    public:
        EnvMessage(QtCoinViewerPtr pviewer, void** ppreturn, bool bWaitForMutex);
        virtual ~EnvMessage();

        /// execute the command in the caller
        virtual void callerexecute();
        /// execute the command in the viewer
        virtual void viewerexecute();

        virtual void releasemutex()
        {
            _plock.reset();
        }

    protected:
        QtCoinViewerPtr _pviewer;
        void** _ppreturn;
        boost::mutex _mutex;
        boost::shared_ptr<boost::mutex::scoped_lock> _plock;
    };
    typedef boost::shared_ptr<EnvMessage> EnvMessagePtr;
    typedef boost::shared_ptr<EnvMessage const> EnvMessageConstPtr;

    static int s_InitRefCount;

protected:
    inline QtCoinViewerPtr shared_viewer() { return boost::static_pointer_cast<QtCoinViewer>(shared_from_this()); }
    inline QtCoinViewerConstPtr shared_viewer_const() const { return boost::static_pointer_cast<QtCoinViewer const>(shared_from_this()); }

    static void mousemove_cb(void * userdata, SoEventCallback * node);
    void _mousemove_cb(SoEventCallback * node);

    virtual void _ViewerSetSize(int w, int h);
    virtual void _ViewerMove(int x, int y);
    virtual void _ViewerSetTitle(const string& ptitle);

    virtual bool _GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK);
    virtual bool _WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension);
    virtual void _SetCamera(const RaveVector<float>& pos, const RaveVector<float>& quat);
    virtual void _SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup);

    virtual void* _plot3(SoSeparator* pparent, const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color);
    virtual void* _plot3(SoSeparator* pparent, const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors);
    
    virtual void* _drawspheres(SoSeparator* pparent, const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color);
    virtual void* _drawspheres(SoSeparator* pparent, const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors);

    virtual void* _drawlinestrip(SoSeparator* pparent, const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual void* _drawlinestrip(SoSeparator* pparent, const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual void* _drawlinelist(SoSeparator* pparent, const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color);
    virtual void* _drawlinelist(SoSeparator* pparent, const float* ppoints, int numPoints, int stride, float fwidth, const float* colors);

    virtual void* _drawarrow(SoSeparator* pparent, const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color);

    virtual void* _drawbox(SoSeparator* pparent, const RaveVector<float>& vpos, const RaveVector<float>& vextents);
    virtual void* _drawtrimesh(SoSeparator* pparent, const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color);

    virtual void _closegraph(void* handle);
    virtual void _deselect();

    virtual void _Reset();
    virtual void _SetBkgndColor(const RaveVector<float>& color);

    virtual void _StartPlaybackTimer();
    virtual void _StopPlaybackTimer();

    virtual void InitOffscreenRenderer();
    virtual void SetupMenus();

    // selection and deselection handling
    static void _SelectHandler(void *, class SoPath *); 
    static void _DeselectHandler(void *, class SoPath *);
    virtual bool _HandleSelection(SoPath *path);
    virtual bool _HandleDeselection(SoNode *node);

    static void _KeyHandler(void *, class SoEventCallback *);

    int _nFrameNum; ///< frame number for recording
    string _strMouseMove; ///< mouse move message
    // Message Queue
    list<EnvMessagePtr> _listMessages;
    list<Item*> _listRemoveItems;
    boost::mutex _mutexItems, _mutexMessages, _mutexUpdating, _mutexMouseMove; ///< mutex protected messages

    QVBoxLayout * vlayout;
    QGroupBox * view1;

    // Rendering
    SoSelection*        _ivRoot;        ///< root node of the inventor scene
    SoSeparator*        _ivBodies;      ///< all the environment bodies are stored in this node
    SoPerspectiveCamera* _ivCamera;       ///< the camera itself
    SoDrawStyle*         _ivStyle;
    SoTimerSensor*      _timerSensor;   ///< used for animation callback

    SoTimerSensor*      _timerVideo;    ///< used for video recording

    // the GUI
    QWidget       *_pQtWidget;
    SoQtExaminerViewer* _pviewer;
    QAction *pToggleDynamicSimulation;
    QActionGroup* _pToggleDebug;

    SoNode*       _selectedNode;
    boost::shared_ptr<IvDragger>    _pdragger;
    SoEventCallback* _eventKeyboardCB;

    SoText2* _messageNode;

    bool _altDown[2];
    bool _ctrlDown[2];
    bool _bAVIInit;
    int  _VideoFrameRate;

    std::map<KinBodyPtr, KinBodyItemPtr> _mapbodies;    ///< all the bodies created
    
    ItemPtr          _pSelectedItem;      ///< the currently selected item
    KinBody::LinkWeakPtr _pMouseOverLink;
    RaveVector<float> _vMouseSurfacePosition,_vMouseRayDirection;

    /// for movie recording
    SoOffscreenRenderer _ivOffscreen;
    SoSeparator* _pOffscreenVideo, *_pFigureRoot;
    uint64_t _nLastVideoFrame, _nVideoTimeOffset;
    bool _bCanRenderOffscreen;

    RaveTransform<float>     _initSelectionTrans;       ///< initial tarnsformation of selected item    
    RaveTransform<float> Tcam;

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
    boost::condition _condUpdateModels; ///< signaled everytime environment models are updated
    boost::mutex _mutexGUI;
    bool _bInIdleThread;

    // toggle switches
    bool _bModelsUpdated;
    bool _bSelfCollision;
    bool         _bDisplayGrid;
    bool         _bDisplayIK;
    bool         _bDisplayFPS;
    bool 	 _bDisplayFeedBack;
    bool         _bJointHilit;
    bool         _bDynamicReplan;
    bool         _bVelPredict;
    bool         _bDynSim;
    bool         _bGravity;
    bool         _bControl;

    bool         _bSensing;
    bool         _bMemory;
    bool         _bHardwarePlan;
    bool         _bShareBitmap;
    bool         _bManipTracking;
    bool         _bAntialiasing;

    // data relating to playback
    bool         _bStopped;
    bool         _bTimeInitialized;
    int          _fileFrame;
    int          _recordInterval;
    time_t       _prevSec;
    long         _prevMicSec;

    bool _bTimeElapsed;
    //bool _bRecordMotion;
    bool _bSaveVideo;
    bool _bRealtimeVideo;
    
    // ode thread related
    bool _bQuitMainLoop;
    bool _bUpdateEnvironment;
    ViewGeometry _viewGeometryMode;

    std::list<std::pair<int,ViewerCallbackFn > > _listRegisteredCallbacks; ///< callbacks to call when particular properties of the body change.

    friend class EnvMessage;
    friend class ViewerSetSizeMessage;
    friend class ViewerMoveMessage;
    friend class ViewerSetTitleMessage;
    friend class GetCameraImageMessage;
    friend class WriteCameraImageMessage;
    friend class SetCameraMessage;
    friend class SetCameraLookAtMessage;
    friend class DrawMessage;
    friend class DrawArrowMessage;
    friend class DrawBoxMessage;
    friend class DrawTriMeshMessage;
    friend class CloseGraphMessage;
    friend class DeselectMessage;
    friend class ResetMessage;
    friend class SetBkgndColorMessage;
    friend class StartPlaybackTimerMessage;
    friend class StopPlaybackTimerMessage;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(SoSeparator)
BOOST_TYPEOF_REGISTER_TYPE(SoTransform)
BOOST_TYPEOF_REGISTER_TYPE(QtCoinViewer::EnvMessage)
#endif

#endif
