// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "qtcoin.h"

#include <Inventor/elements/SoGLCacheContextElement.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoMaterialBinding.h>
#include <Inventor/nodes/SoTextureCombine.h>
#include <Inventor/nodes/SoTextureCoordinate2.h>
#include <Inventor/nodes/SoTextureScalePolicy.h>
#include <Inventor/nodes/SoTransparencyType.h>
#include <Inventor/misc/SoGLImage.h>
#include <Inventor/events/SoLocation2Event.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>

#if QT_VERSION >= 0x040000 // check for qt4
#include <QtOpenGL/QGLWidget>
#else
#include <qgl.h>
#endif

#include <locale>

const float TIMER_SENSOR_INTERVAL = (1.0f/60.0f);

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
#define VIDEO_FRAMERATE (30000.0/1001.0) // 29.97 //60

void DeleteItemCallbackSafe(QtCoinViewerWeakPtr wpt, Item* pItem)
{
    QtCoinViewerPtr pviewer = wpt.lock();
    if( !!pviewer ) {
        pviewer->_DeleteItemCallback(pItem);
    }
}

#define ITEM_DELETER boost::bind(DeleteItemCallbackSafe,weak_viewer(),_1)

class ItemSelectionCallbackData : public UserData
{
public:
    ItemSelectionCallbackData(const ViewerBase::ItemSelectionCallbackFn& callback, boost::shared_ptr<QtCoinViewer> pviewer) : _callback(callback), _pweakviewer(pviewer) {
    }
    virtual ~ItemSelectionCallbackData() {
        boost::shared_ptr<QtCoinViewer> pviewer = _pweakviewer.lock();
        if( !!pviewer ) {
            boost::mutex::scoped_lock lock(pviewer->_mutexCallbacks);
            pviewer->_listRegisteredItemSelectionCallbacks.erase(_iterator);
        }
    }

    list<UserDataWeakPtr>::iterator _iterator;
    ViewerBase::ItemSelectionCallbackFn _callback;
protected:
    boost::weak_ptr<QtCoinViewer> _pweakviewer;
};
typedef boost::shared_ptr<ItemSelectionCallbackData> ItemSelectionCallbackDataPtr;

class ViewerImageCallbackData : public UserData
{
public:
    ViewerImageCallbackData(const ViewerBase::ViewerImageCallbackFn& callback, boost::shared_ptr<QtCoinViewer> pviewer) : _callback(callback), _pweakviewer(pviewer) {
    }
    virtual ~ViewerImageCallbackData() {
        boost::shared_ptr<QtCoinViewer> pviewer = _pweakviewer.lock();
        if( !!pviewer ) {
            boost::mutex::scoped_lock lock(pviewer->_mutexCallbacks);
            pviewer->_listRegisteredViewerImageCallbacks.erase(_iterator);
        }
    }

    list<UserDataWeakPtr>::iterator _iterator;
    ViewerBase::ViewerImageCallbackFn _callback;
protected:
    boost::weak_ptr<QtCoinViewer> _pweakviewer;
};
typedef boost::shared_ptr<ViewerImageCallbackData> ViewerImageCallbackDataPtr;

class ViewerThreadCallbackData : public UserData
{
public:
    ViewerThreadCallbackData(const ViewerBase::ViewerThreadCallbackFn& callback, boost::shared_ptr<QtCoinViewer> pviewer) : _callback(callback), _pweakviewer(pviewer) {
    }
    virtual ~ViewerThreadCallbackData() {
        boost::shared_ptr<QtCoinViewer> pviewer = _pweakviewer.lock();
        if( !!pviewer ) {
            boost::mutex::scoped_lock lock(pviewer->_mutexCallbacks);
            pviewer->_listRegisteredViewerThreadCallbacks.erase(_iterator);
        }
    }

    list<UserDataWeakPtr>::iterator _iterator;
    ViewerBase::ViewerThreadCallbackFn _callback;
protected:
    boost::weak_ptr<QtCoinViewer> _pweakviewer;
};
typedef boost::shared_ptr<ViewerThreadCallbackData> ViewerThreadCallbackDataPtr;

static SoErrorCB* s_DefaultHandlerCB=NULL;
void CustomCoinHandlerCB(const class SoError * error, void * data)
{
    if( error != NULL ) {
        // extremely annoying errors
        if((strstr(error->getDebugString().getString(),"Coin warning in SbLine::setValue()") != NULL)||
           ( strstr(error->getDebugString().getString(),"Coin warning in SbDPLine::setValue()") != NULL) ||
           ( strstr(error->getDebugString().getString(),"Coin warning in SbVec3f::setValue()") != NULL) ||
           ( strstr(error->getDebugString().getString(),"Coin warning in SoNormalGenerator::calcFaceNormal()") != NULL) ||
           ( strstr(error->getDebugString().getString(),"Coin error in SoGroup::removeChild(): tried to remove non-existent child") != NULL) ||
           ( strstr(error->getDebugString().getString(),"Coin error in SoSwitch::doAction(): whichChild 0 out of range -- switch node has no children!") != NULL) ||
           ( strstr(error->getDebugString().getString(),"Coin warning in SbPlane::SbPlane(): The three points defining the plane cannot be on line.") != NULL ) ) {
            return;
        }
    }

    if( s_DefaultHandlerCB != NULL ) {
        s_DefaultHandlerCB(error,data);
    }
}

static QtCoinViewer* s_pviewer = NULL;
QtCoinViewer::QtCoinViewer(EnvironmentBasePtr penv, std::istream& sinput)
    : QMainWindow(NULL, Qt::Window),
    ViewerBase(penv), _ivOffscreen(SbViewportRegion(_nRenderWidth, _nRenderHeight))
{
    s_pviewer = this;
    _InitConstructor(sinput);
}

void QtCoinViewer::_InitConstructor(std::istream& sinput)
{
    int qtcoinbuild = SoQtExaminerViewer::BUILD_ALL;
    bool bCreateStatusBar = true, bCreateMenu = true;
    int nAlwaysOnTopFlag = 0; // 1 - add on top flag (keep others), 2 - add on top flag (remove others)
    sinput >> qtcoinbuild >> bCreateStatusBar >> bCreateMenu >> nAlwaysOnTopFlag;

    _nQuitMainLoop = 0;
    _name = str(boost::format("OpenRAVE %s")%OPENRAVE_VERSION_STRING);
    if( (OPENRAVE_VERSION_MINOR%2) || (OPENRAVE_VERSION_PATCH%2) ) {
        _name += " (Development Version)";
    }
    else {
        _name += " (Stable Release)";
    }
#if QT_VERSION >= 0x040000 // check for qt4
    setWindowTitle(_name.c_str());
    if(  bCreateStatusBar ) {
        statusBar()->showMessage(tr("Status Bar"));
    }
#endif
    __description = ":Interface Author: Rosen Diankov\n\nProvides a GUI using the Qt4, Coin3D, and SoQt libraries. Depending on the version, Coin3D and SoQt might be licensed under GPL.\n\nIf the current directory contains a filename **environment.iv** when the qtcoin viewer is loaded, then this file will define the scene file all other elements are loaded under. This allows users to define their own lighting model. For example, the following **environment.iv** file will force every object to be draw as wirefire:\n\n\
.. code-block:: c\n\
\n\
  #Inventor V2.1 ascii\n\
\n\
  Separator {\n\
  DrawStyle { style LINES  lineWidth 2 }\n\
  }\n\n";
    RegisterCommand("SetFiguresInCamera",boost::bind(&QtCoinViewer::_SetFiguresInCamera,this,_1,_2),
                    "Accepts 0/1 value that decides whether to render the figure plots in the camera image through GetCameraImage");
    RegisterCommand("SetFeedbackVisibility",boost::bind(&QtCoinViewer::_SetFeedbackVisibility,this,_1,_2),
                    "Accepts 0/1 value that decides whether to render the cross hairs");
    RegisterCommand("ShowWorldAxes",boost::bind(&QtCoinViewer::_SetFeedbackVisibility,this,_1,_2),
                    "Accepts 0/1 value that decides whether to render the cross hairs");
    RegisterCommand("Resize",boost::bind(&QtCoinViewer::_CommandResize,this,_1,_2),
                    "Accepts width x height to resize internal video frame");
    RegisterCommand("SaveBodyLinkToVRML",boost::bind(&QtCoinViewer::_SaveBodyLinkToVRMLCommand,this,_1,_2),
                    "Saves a body and/or a link to VRML. Format is::\n\n  bodyname linkindex filename\\n\n\nwhere linkindex >= 0 to save for a specific link, or < 0 to save all links");
    RegisterCommand("SetNearPlane", boost::bind(&QtCoinViewer::_SetNearPlaneCommand, this, _1, _2),
                    "Sets the near plane for rendering of the image. Useful when tweaking rendering units");
    RegisterCommand("StartViewerLoop", boost::bind(&QtCoinViewer::_StartViewerLoopCommand, this, _1, _2),
                    "starts the viewer sync loop and shows the viewer. expects someone else will call the qapplication exec fn");
    RegisterCommand("Show", boost::bind(&QtCoinViewer::_ShowCommand, this, _1, _2),
                    "executs the show directly");
    RegisterCommand("TrackLink", boost::bind(&QtCoinViewer::_TrackLinkCommand, this, _1, _2),
                    "camera tracks the link maintaining a specific relative transform: robotname, manipname, focalDistance");
    RegisterCommand("TrackManipulator", boost::bind(&QtCoinViewer::_TrackManipulatorCommand, this, _1, _2),
                    "camera tracks the manipulator maintaining a specific relative transform: robotname, manipname, focalDistance");
    RegisterCommand("SetTrackingAngleToUp", boost::bind(&QtCoinViewer::_SetTrackingAngleToUpCommand, this, _1, _2),
                    "sets a new angle to up");

    _fTrackAngleToUp = 0.3;
    _bLockEnvironment = true;
    _pToggleDebug = NULL;
    _pSelectedCollisionChecker = NULL;
    _pSelectedPhysicsEngine = NULL;
    _pToggleSimulation = NULL;
    _bInIdleThread = false;
    _bAutoSetCamera = true;
    _videocodec = -1;
    _bRenderFiguresInCamera = false;
    _focalDistance = 0.0;

    //vlayout = new QVBoxLayout(this);
    view1 = new QGroupBox(this);
    //vlayout->addWidget(view1, 1);
    setCentralWidget (view1);

    _nRenderWidth=VIDEO_WIDTH;
    _nRenderHeight=VIDEO_HEIGHT;

    resize(_nRenderWidth, _nRenderHeight);

    _pviewer = new SoQtExaminerViewer(view1, "qtcoinopenrave", 1, (SoQtExaminerViewer::BuildFlag)qtcoinbuild, SoQtViewer::BROWSER);

    _selectedNode = NULL;
    s_DefaultHandlerCB = SoDebugError::getHandlerCallback();
    SoDebugError::setHandlerCallback(CustomCoinHandlerCB,NULL);

    // initialize the environment
    _ivRoot = new SoSelection();
    _ivRoot->ref();
    _ivRoot->policy.setValue(SoSelection::SHIFT);

    _ivCamera = new SoPerspectiveCamera();
    _ivStyle = new SoDrawStyle();
    _ivCamera->position.setValue(-0.5f, 1.5f, 0.8f);
    _ivCamera->orientation.setValue(SbVec3f(1,0,0), -0.5f);
    _ivCamera->aspectRatio = (float)view1->size().width() / (float)view1->size().height();

    _ivBodies = NULL;
    if( !!ifstream("environment.iv") ) {
        SoInput mySceneInput;
        if( mySceneInput.openFile("environment.iv") ) {
            _ivBodies = SoDB::readAll(&mySceneInput);
            if( !!_ivBodies ) {
                // environment should take care of this
                _pviewer->setHeadlight(false);
            }
        }
    }
    if( _ivBodies == NULL ) {
        _ivBodies = new SoSeparator();
    }

    // add the message texts
    SoSeparator* pmsgsep = new SoSeparator();
    SoTranslation* pmsgtrans0 = new SoTranslation();
    pmsgtrans0->translation.setValue(SbVec3f(-0.978f,0.93f,0));
    pmsgsep->addChild(pmsgtrans0);
    SoBaseColor* pcolor0 = new SoBaseColor();
    pcolor0->rgb.setValue(0.0f,0.0f,0.0f);
    pmsgsep->addChild(pcolor0);
    _messageNodes[0] = new SoText2();
    pmsgsep->addChild(_messageNodes[0]);

    _messageShadowTranslation = new SoTranslation();
    _messageShadowTranslation->translation.setValue(SbVec3f(-0.002f,0.032f,0));
    pmsgsep->addChild(_messageShadowTranslation);
    SoBaseColor* pcolor1 = new SoBaseColor();
    pcolor1->rgb.setValue(0.99f,0.99f,0.99f);
    pmsgsep->addChild(pcolor1);
    _messageNodes[1] = new SoText2();
    pmsgsep->addChild(_messageNodes[1]);

    _ivRoot->addChild(pmsgsep);
    _ivRoot->addChild(_ivCamera);

    SoEventCallback * ecb = new SoEventCallback;
    ecb->addEventCallback(SoLocation2Event::getClassTypeId(), mousemove_cb, this);
    _ivRoot->addChild(ecb);

    _ivRoot->addChild(_ivStyle);
    _ivRoot->addChild(_ivBodies);

    // add Inventor selection callbacks
    _ivRoot->addSelectionCallback(_SelectHandler, this);
    _ivRoot->addDeselectionCallback(_DeselectHandler, this);

    SoComplexity* pcomplexity = new SoComplexity();
    pcomplexity->value = 0.1f; // default =0.5, lower is faster
    pcomplexity->type = SoComplexity::SCREEN_SPACE;
    pcomplexity->textureQuality = 1.0; // good texture quality
    _ivRoot->addChild(pcomplexity);
    SoTextureScalePolicy* ppolicy = new SoTextureScalePolicy();
    ppolicy->policy = SoTextureScalePolicy::FRACTURE; // requires in order to support non-power of 2 textures
    _ivRoot->addChild(ppolicy);

    _pFigureRoot = new SoSeparator();
    {
        SoLightModel* plightmodel = new SoLightModel();
        plightmodel->model = SoLightModel::BASE_COLOR; // disable lighting
        _pFigureRoot->addChild(plightmodel);
    }
    _ivRoot->addChild(_pFigureRoot);

    _pviewer->setSceneGraph(_ivRoot);
    _pviewer->setAutoClippingStrategy(SoQtViewer::CONSTANT_NEAR_PLANE, 0.01f);
    _pviewer->setSeekTime(1.0f);

    _SetBkgndColor(Vector(1,1,1));

    // setup a callback handler for keyboard events
    _eventKeyboardCB = new SoEventCallback;
    _ivRoot->addChild(_eventKeyboardCB);
    _eventKeyboardCB->addEventCallback(SoKeyboardEvent::getClassTypeId(), _KeyHandler, this);

    _altDown[0] = _altDown[1]  = false;
    _ctrlDown[0] = _ctrlDown[1] = false;
    _bUpdateEnvironment = true;

    // toggle switches
    _nFrameNum = 0;
    _bDisplayGrid = false;
    _bDisplayIK = false;
    _bDisplayFPS = false;
    _bJointHilit = true;
    _bDynamicReplan = false;
    _bVelPredict = true;
    _bDynSim = false;
    _bControl = true;
    _bGravity = true;
    _bTimeElapsed = false;
    _bSensing = false;
    _bMemory = true;
    _bHardwarePlan = false;
    _bShareBitmap = true;
    _bManipTracking = false;
    _bAntialiasing = false;
    _viewGeometryMode = VG_RenderOnly;

    if( bCreateMenu ) {
        SetupMenus();
    }

    InitOffscreenRenderer();

    _timerSensor = new SoTimerSensor(GlobAdvanceFrame, this);
    _timerSensor->setInterval(SbTime(TIMER_SENSOR_INTERVAL));

    _timerVideo = new SoTimerSensor(GlobVideoFrame, this);
    _timerVideo->setInterval(SbTime(0.7/VIDEO_FRAMERATE)); // better to produce more frames than get slow video
    if (!_timerVideo->isScheduled()) {
        _timerVideo->schedule();
    }

    SoDB::setRealTimeInterval(SbTime(0.7/VIDEO_FRAMERATE));  // better to produce more frames than get slow video

    // set to the classic locale so that number serialization/hashing works correctly
    // for some reason qt4 resets the locale to the default locale at some point, and openrave stops working
    // std::locale::global(std::locale::classic());

    if( nAlwaysOnTopFlag != 0 ) {
        Qt::WindowFlags flags = Qt::CustomizeWindowHint | Qt::WindowStaysOnTopHint;
        if( nAlwaysOnTopFlag == 1 ) {
            flags |= this->windowFlags();
        }
        this->setWindowFlags(flags);
    }
}

QtCoinViewer::~QtCoinViewer()
{
    RAVELOG_DEBUG("destroying qtcoinviewer\n");

    {
        boost::mutex::scoped_lock lock(_mutexMessages);

        list<EnvMessagePtr>::iterator itmsg;
        FORIT(itmsg, _listMessages) {
            try {
                (*itmsg)->viewerexecute(); // have to execute instead of deleteing since there can be threads waiting
            }
            catch(const boost::bad_weak_ptr& ex) {
                // most likely viewer
            }
        }
        _listMessages.clear();
    }

    _ivRoot->deselectAll();

    if (_timerSensor->isScheduled()) {
        _timerSensor->unschedule();
    }
    if (_timerVideo->isScheduled()) {
        _timerVideo->unschedule();
    }

    _eventKeyboardCB->removeEventCallback(SoKeyboardEvent::getClassTypeId(), _KeyHandler, this);
    _ivRoot->removeSelectionCallback(_SelectHandler, this);
    _ivRoot->removeDeselectionCallback(_DeselectHandler, this);
    _eventKeyboardCB->unref();

    _condUpdateModels.notify_all();

    _pvideorecorder.reset();
    // don't dereference
    //    if( --s_InitRefCount <= 0 )
    //        SoQt::done();
}

void QtCoinViewer::resize ( int w, int h)
{
    QMainWindow::resize(w,h);
}

void QtCoinViewer::resize ( const QSize & qs)
{
    resize(qs.width(), qs.height());
}

void QtCoinViewer::mousemove_cb(void * userdata, SoEventCallback * node)
{
    ((QtCoinViewer*)userdata)->_mousemove_cb(node);
}

void QtCoinViewer::_mousemove_cb(SoEventCallback * node)
{
    SoRayPickAction rp( _pviewer->getViewportRegion() );
    rp.setPoint(node->getEvent()->getPosition());
    rp.apply(_ivRoot);
    SoPickedPoint * pt = rp.getPickedPoint(0);
    if( pt != NULL ) {
        SoPath* path = pt->getPath();
        ItemPtr pItem;
        SoNode* node = NULL;
        for(int i = path->getLength()-1; i >= 0; --i) {
            node = path->getNode(i);

            // search the environment
            FOREACH(it, _mapbodies) {
                BOOST_ASSERT( !!it->second );
                if (it->second->ContainsIvNode(node)) {
                    pItem = it->second;
                    break;
                }
            }

            if( !!pItem ) {
                break;
            }
        }

        if (!!pItem) {
            boost::mutex::scoped_lock lock(_mutexMessages);

            KinBodyItemPtr pKinBody = boost::dynamic_pointer_cast<KinBodyItem>(pItem);
            KinBody::LinkPtr pSelectedLink;
            if( !!pKinBody ) {
                pSelectedLink = pKinBody->GetLinkFromIv(node);
            }
            _pMouseOverLink = pSelectedLink;
            _vMouseSurfacePosition.x = pt->getPoint()[0];
            _vMouseSurfacePosition.y = pt->getPoint()[1];
            _vMouseSurfacePosition.z = pt->getPoint()[2];
            _vMouseSurfaceNormal.x = pt->getNormal()[0];
            _vMouseSurfaceNormal.y = pt->getNormal()[1];
            _vMouseSurfaceNormal.z = pt->getNormal()[2];
            SbVec3f cp = GetCamera()->position.getValue();
            RaveVector<float> camerapos (cp[0],cp[1],cp[2]);
            _vMouseRayDirection = _vMouseSurfacePosition-camerapos;
            if( _vMouseRayDirection.lengthsqr3() > 0 ) {
                _vMouseRayDirection.normalize3();
            }
            else {
                _vMouseRayDirection = Vector(0,0,0);
            }

            stringstream ss;
            ss << "mouse on " << pKinBody->GetBody()->GetName() << ":";
            if( !!pSelectedLink ) {
                ss << pSelectedLink->GetName() << "(" << pSelectedLink->GetIndex() << ")";
            }
            else {
                ss << "(NULL)";
            }

            ss << " (" << std::fixed << std::setprecision(5)
               << std::setw(8) << std::left << pt->getPoint()[0] << ", "
               << std::setw(8) << std::left << pt->getPoint()[1] << ", "
               << std::setw(8) << std::left << pt->getPoint()[2] << ")";
            ss << ", n=(" << std::setw(8) << std::left << _vMouseSurfaceNormal.x << ", "
               << std::setw(8) << std::left << _vMouseSurfaceNormal.y << ", "
               << std::setw(8) << std::left << _vMouseSurfaceNormal.z << ")";
            ss << endl;
            _strMouseMove = ss.str();
        }
        else {
            boost::mutex::scoped_lock lock(_mutexMessages);
            _strMouseMove.resize(0);
        }
    }
    else {
        boost::mutex::scoped_lock lock(_mutexMessages);
        _strMouseMove.resize(0);
    }
}

class ViewerSetSizeMessage : public QtCoinViewer::EnvMessage
{
public:
    ViewerSetSizeMessage(QtCoinViewerPtr pviewer, void** ppreturn, int width, int height)
        : EnvMessage(pviewer, ppreturn, false), _width(width), _height(height) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !!pviewer ) {
            pviewer->_SetSize(_width, _height);
        }
        EnvMessage::viewerexecute();
    }

private:
    int _width, _height;
};

void QtCoinViewer::SetSize(int w, int h)
{
    EnvMessagePtr pmsg(new ViewerSetSizeMessage(shared_viewer(), (void**)NULL, w, h));
    pmsg->callerexecute(false);
}

void QtCoinViewer::_SetSize(int w, int h)
{
    resize(w,h);
}

class ViewerMoveMessage : public QtCoinViewer::EnvMessage
{
public:
    ViewerMoveMessage(QtCoinViewerPtr pviewer, void** ppreturn, int x, int y)
        : EnvMessage(pviewer, ppreturn, false), _x(x), _y(y) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !!pviewer ) {
            pviewer->_Move(_x, _y);
        }
        EnvMessage::viewerexecute();
    }

private:
    int _x, _y;
};

void QtCoinViewer::Move(int x, int y)
{
    EnvMessagePtr pmsg(new ViewerMoveMessage(shared_viewer(), (void**)NULL, x, y));
    pmsg->callerexecute(false);
}

void QtCoinViewer::_Move(int x, int y)
{
    move(x,y);
}

class ViewerShowMessage : public QtCoinViewer::EnvMessage
{
public:
    ViewerShowMessage(QtCoinViewerPtr pviewer, void** ppreturn, int showtype)
        : EnvMessage(pviewer, ppreturn, false), _showtype(showtype) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !!pviewer ) {
            pviewer->_Show(_showtype);
        }
        EnvMessage::viewerexecute();
    }

private:
    int _showtype;
};

void QtCoinViewer::Show(int showtype)
{
    EnvMessagePtr pmsg(new ViewerShowMessage(shared_viewer(), (void**)NULL, showtype));
    pmsg->callerexecute(false);
}

void QtCoinViewer::_Show(int showtype)
{
    if( showtype ) {
        _pviewer->show();
    }
    else {
        _pviewer->hide();
    }
}

class ViewerSetNameMessage : public QtCoinViewer::EnvMessage
{
public:
    ViewerSetNameMessage(QtCoinViewerPtr pviewer, void** ppreturn, const string& ptitle)
        : EnvMessage(pviewer, ppreturn, false), _title(ptitle) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !!pviewer ) {
            pviewer->_SetName(_title.c_str());
        }
        EnvMessage::viewerexecute();
    }

private:
    string _title;
};

void QtCoinViewer::SetName(const string& ptitle)
{
    _name = ptitle;
    EnvMessagePtr pmsg(new ViewerSetNameMessage(shared_viewer(), (void**)NULL, ptitle));
    pmsg->callerexecute(false);
}

void QtCoinViewer::_SetName(const string& ptitle)
{
    setWindowTitle(ptitle.c_str());
}

bool QtCoinViewer::LoadModel(const string& pfilename)
{
    SoInput mySceneInput;
    if (mySceneInput.openFile(pfilename.c_str())) {
        GetBodiesRoot()->addChild(SoDB::readAll(&mySceneInput));
        return true;
    }

    return false;
}

void QtCoinViewer::_StartPlaybackTimer()
{
    if (!_timerSensor->isScheduled()) {
        _timerSensor->schedule();
    }
}

void QtCoinViewer::_StopPlaybackTimer()
{
    if (_timerSensor->isScheduled()) {
        _timerSensor->unschedule();
    }
    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    _condUpdateModels.notify_all();
}

class GetCameraImageMessage : public QtCoinViewer::EnvMessage
{
public:
    GetCameraImageMessage(QtCoinViewerPtr pviewer, void** ppreturn,
                          std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& extrinsic, const SensorBase::CameraIntrinsics& KK)
        : EnvMessage(pviewer, ppreturn, true), _memory(memory), _width(width), _height(height), _extrinsic(extrinsic), _KK(KK) {
    }

    virtual void viewerexecute() {
        void* ret = (void*)QtCoinViewerPtr(_pviewer)->_GetCameraImage(_memory, _width, _height, _extrinsic, _KK);
        if( _ppreturn != NULL )
            *_ppreturn = ret;
        EnvMessage::viewerexecute();
    }

private:
    vector<uint8_t>& _memory;
    int _width, _height;
    const RaveTransform<float>& _extrinsic;
    const SensorBase::CameraIntrinsics& _KK;
};

bool QtCoinViewer::GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK)
{
    void* ret = NULL;
    if (_timerSensor->isScheduled() && _bUpdateEnvironment) {
        if( !ForceUpdatePublishedBodies() ) {
            RAVELOG_WARN("failed to GetCameraImage: force update failed\n");
            return false;
        }
        EnvMessagePtr pmsg(new GetCameraImageMessage(shared_viewer(), &ret, memory, width, height, t, KK));
        pmsg->callerexecute(false);
    }
    else {
        RAVELOG_VERBOSE("failed to GetCameraImage: viewer is not updating\n");
    }
    return *(bool*)&ret;
}

class WriteCameraImageMessage : public QtCoinViewer::EnvMessage
{
public:
    WriteCameraImageMessage(QtCoinViewerPtr pviewer, void** ppreturn,
                            int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& fileName, const std::string& extension)
        : EnvMessage(pviewer, ppreturn, true), _width(width), _height(height), _t(t),
        _KK(KK), _fileName(fileName), _extension(extension) {
    }

    virtual void viewerexecute() {
        void* ret = (void*)QtCoinViewerPtr(_pviewer)->_WriteCameraImage(_width, _height, _t, _KK, _fileName, _extension);
        if( _ppreturn != NULL )
            *_ppreturn = ret;
        EnvMessage::viewerexecute();
    }

private:
    int _width, _height;
    const RaveTransform<float>& _t;
    const SensorBase::CameraIntrinsics& _KK;
    const string& _fileName, &_extension;
};

bool QtCoinViewer::WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& fileName, const std::string& extension)
{
    void* ret;
    if (_timerSensor->isScheduled() && _bUpdateEnvironment) {
        if( !ForceUpdatePublishedBodies() ) {
            RAVELOG_WARN("failed to WriteCameraImage\n");
            return false;
        }
        EnvMessagePtr pmsg(new WriteCameraImageMessage(shared_viewer(), &ret, width, height, t, KK, fileName, extension));
        pmsg->callerexecute(false);
    }
    else {
        RAVELOG_WARN("failed to WriteCameraImage: viewer is not updating\n");
    }
    return *(bool*)&ret;
}

class SetCameraMessage : public QtCoinViewer::EnvMessage
{
public:
    SetCameraMessage(QtCoinViewerPtr pviewer, void** ppreturn, const RaveTransform<float>& trans, float focalDistance)
        : EnvMessage(pviewer, ppreturn, false), _trans(trans),_focalDistance(focalDistance) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr(_pviewer)->_SetCamera(_trans,_focalDistance);
        EnvMessage::viewerexecute();
    }

private:
    const RaveTransform<float> _trans;
    float _focalDistance;
};

void QtCoinViewer::SetCamera(const RaveTransform<float>& trans,float focalDistance)
{
    EnvMessagePtr pmsg(new SetCameraMessage(shared_viewer(), (void**)NULL, trans,focalDistance));
    pmsg->callerexecute(false);
}

class DrawMessage : public QtCoinViewer::EnvMessage
{
public:
    enum DrawType
    {
        DT_Point = 0,
        DT_Sphere,
        DT_LineStrip,
        DT_LineList,
    };

    DrawMessage(QtCoinViewerPtr pviewer, SoSwitch* handle, const float* ppoints, int numPoints,
                int stride, float fwidth, const float* colors, DrawType type, bool bhasalpha)
        : EnvMessage(pviewer, NULL, false), _numPoints(numPoints),
        _fwidth(fwidth), _handle(handle), _type(type), _bhasalpha(bhasalpha)
    {
        _vpoints.resize(3*numPoints);
        for(int i = 0; i < numPoints; ++i) {
            _vpoints[3*i+0] = ppoints[0];
            _vpoints[3*i+1] = ppoints[1];
            _vpoints[3*i+2] = ppoints[2];
            ppoints = (float*)((char*)ppoints + stride);
        }
        _stride = 3*sizeof(float);

        _vcolors.resize((_bhasalpha ? 4 : 3)*numPoints);
        if( colors != NULL )
            memcpy(&_vcolors[0], colors, sizeof(float)*_vcolors.size());

        _bManyColors = true;
    }
    DrawMessage(QtCoinViewerPtr pviewer, SoSwitch* handle, const float* ppoints, int numPoints,
                int stride, float fwidth, const RaveVector<float>& color, DrawType type)
        : EnvMessage(pviewer, NULL, false), _numPoints(numPoints),
        _fwidth(fwidth), _color(color), _handle(handle), _type(type)
    {
        _vpoints.resize(3*numPoints);
        for(int i = 0; i < numPoints; ++i) {
            _vpoints[3*i+0] = ppoints[0];
            _vpoints[3*i+1] = ppoints[1];
            _vpoints[3*i+2] = ppoints[2];
            ppoints = (float*)((char*)ppoints + stride);
        }
        _stride = 3*sizeof(float);

        _bManyColors = false;
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        void* ret=NULL;
        switch(_type) {
        case DT_Point:
            if( _bManyColors ) {
                ret = pviewer->_plot3(_handle, &_vpoints[0], _numPoints, _stride, _fwidth, &_vcolors[0],_bhasalpha);
            }
            else  {
                ret = pviewer->_plot3(_handle, &_vpoints[0], _numPoints, _stride, _fwidth, _color);
            }
            break;
        case DT_Sphere:
            if( _bManyColors ) {
                ret = pviewer->_drawspheres(_handle, &_vpoints[0], _numPoints, _stride, _fwidth, &_vcolors[0],_bhasalpha);
            }
            else {
                ret = pviewer->_drawspheres(_handle, &_vpoints[0], _numPoints, _stride, _fwidth, _color);
            }
            break;
        case DT_LineStrip:
            if( _bManyColors ) {
                ret = pviewer->_drawlinestrip(_handle, &_vpoints[0], _numPoints, _stride, _fwidth, &_vcolors[0]);
            }
            else {
                ret = pviewer->_drawlinestrip(_handle, &_vpoints[0], _numPoints, _stride, _fwidth, _color);
            }
            break;
        case DT_LineList:
            if( _bManyColors ) {
                ret = pviewer->_drawlinelist(_handle, &_vpoints[0], _numPoints, _stride, _fwidth, &_vcolors[0]);
            }
            else {
                ret = pviewer->_drawlinelist(_handle, &_vpoints[0], _numPoints, _stride, _fwidth, _color);
            }
            break;
        }

        BOOST_ASSERT( _handle == ret);
        EnvMessage::viewerexecute();
    }

private:
    vector<float> _vpoints;
    int _numPoints, _stride;
    float _fwidth;
    const RaveVector<float> _color;
    vector<float> _vcolors;
    SoSwitch* _handle;
    bool _bManyColors;
    DrawType _type;
    bool _bhasalpha;
};

SoSwitch* QtCoinViewer::_createhandle()
{
    SoSwitch* handle = new SoSwitch();
    handle->whichChild = SO_SWITCH_ALL;
    return handle;
}

GraphHandlePtr QtCoinViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawMessage(shared_viewer(), handle, ppoints, numPoints, stride, fPointSize, color, drawstyle ? DrawMessage::DT_Sphere : DrawMessage::DT_Point));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtCoinViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle, bool bhasalpha)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawMessage(shared_viewer(), handle, ppoints, numPoints, stride, fPointSize, colors, drawstyle ? DrawMessage::DT_Sphere : DrawMessage::DT_Point, bhasalpha));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtCoinViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawMessage(shared_viewer(), handle, ppoints, numPoints, stride, fwidth, color,DrawMessage::DT_LineStrip));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtCoinViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawMessage(shared_viewer(), handle, ppoints, numPoints, stride, fwidth, colors, DrawMessage::DT_LineStrip,false));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtCoinViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawMessage(shared_viewer(), handle, ppoints, numPoints, stride, fwidth, color, DrawMessage::DT_LineList));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtCoinViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawMessage(shared_viewer(), handle, ppoints, numPoints, stride, fwidth, colors, DrawMessage::DT_LineList,false));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

class DrawArrowMessage : public QtCoinViewer::EnvMessage
{
public:
    DrawArrowMessage(QtCoinViewerPtr pviewer, SoSwitch* handle, const RaveVector<float>& p1,
                     const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
        : EnvMessage(pviewer, NULL, false), _p1(p1), _p2(p2), _color(color), _handle(handle), _fwidth(fwidth) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        void* ret = pviewer->_drawarrow(_handle, _p1, _p2, _fwidth, _color);
        BOOST_ASSERT( _handle == ret );
        EnvMessage::viewerexecute();
    }

private:
    RaveVector<float> _p1, _p2, _color;
    SoSwitch* _handle;
    float _fwidth;
};

GraphHandlePtr QtCoinViewer::drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawArrowMessage(shared_viewer(), handle, p1, p2, fwidth, color));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

class DrawBoxMessage : public QtCoinViewer::EnvMessage
{
public:
    DrawBoxMessage(QtCoinViewerPtr pviewer, SoSwitch* handle,
                   const RaveVector<float>& vpos, const RaveVector<float>& vextents)
        : EnvMessage(pviewer, NULL, false), _vpos(vpos), _vextents(vextents), _handle(handle) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        void* ret = pviewer->_drawbox(_handle, _vpos, _vextents);
        BOOST_ASSERT( _handle == ret);
        EnvMessage::viewerexecute();
    }

private:
    RaveVector<float> _vpos, _vextents;
    SoSwitch* _handle;
};

GraphHandlePtr QtCoinViewer::drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawBoxMessage(shared_viewer(), handle, vpos, vextents));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

class DrawPlaneMessage : public QtCoinViewer::EnvMessage
{
public:
    DrawPlaneMessage(QtCoinViewerPtr pviewer, SoSwitch* handle,
                     const Transform& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
        : EnvMessage(pviewer, NULL, false), _tplane(tplane), _vextents(vextents),_vtexture(vtexture), _handle(handle) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        void* ret = pviewer->_drawplane(_handle, _tplane,_vextents,_vtexture);
        BOOST_ASSERT( _handle == ret);
        EnvMessage::viewerexecute();
    }

private:
    RaveTransform<float> _tplane;
    RaveVector<float> _vextents;
    boost::multi_array<float,3> _vtexture;
    SoSwitch* _handle;
};

GraphHandlePtr QtCoinViewer::drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawPlaneMessage(shared_viewer(), handle, tplane,vextents,vtexture));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

class DrawTriMeshMessage : public QtCoinViewer::EnvMessage
{
public:
    DrawTriMeshMessage(QtCoinViewerPtr pviewer, SoSwitch* handle, const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
        : EnvMessage(pviewer, NULL, false), _color(color), _handle(handle)
    {
        _vpoints.resize(3*3*numTriangles);
        if( pIndices == NULL ) {
            for(int i = 0; i < 3*numTriangles; ++i) {
                _vpoints[3*i+0] = ppoints[0];
                _vpoints[3*i+1] = ppoints[1];
                _vpoints[3*i+2] = ppoints[2];
                ppoints = (float*)((char*)ppoints + stride);
            }
        }
        else {
            for(int i = 0; i < numTriangles*3; ++i) {
                float* p = (float*)((char*)ppoints +  stride * pIndices[i]);
                _vpoints[3*i+0] = p[0];
                _vpoints[3*i+1] = p[1];
                _vpoints[3*i+2] = p[2];
            }
        }
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        void* ret = pviewer->_drawtrimesh(_handle, &_vpoints[0], 3*sizeof(float), NULL, _vpoints.size()/9,_color);
        BOOST_ASSERT( _handle == ret);
        EnvMessage::viewerexecute();
    }

private:
    vector<float> _vpoints;
    RaveVector<float> _color;
    SoSwitch* _handle;
};

class DrawTriMeshColorMessage : public QtCoinViewer::EnvMessage
{
public:
    DrawTriMeshColorMessage(QtCoinViewerPtr pviewer, SoSwitch* handle, const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
        : EnvMessage(pviewer, NULL, false), _colors(colors), _handle(handle)
    {
        _vpoints.resize(3*3*numTriangles);
        if( pIndices == NULL ) {
            for(int i = 0; i < 3*numTriangles; ++i) {
                _vpoints[3*i+0] = ppoints[0];
                _vpoints[3*i+1] = ppoints[1];
                _vpoints[3*i+2] = ppoints[2];
                ppoints = (float*)((char*)ppoints + stride);
            }
        }
        else {
            for(int i = 0; i < numTriangles*3; ++i) {
                float* p = (float*)((char*)ppoints +  stride * pIndices[i]);
                _vpoints[3*i+0] = p[0];
                _vpoints[3*i+1] = p[1];
                _vpoints[3*i+2] = p[2];
            }
        }
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        void* ret = pviewer->_drawtrimesh(_handle, &_vpoints[0], 3*sizeof(float), NULL, _vpoints.size()/9,_colors);
        BOOST_ASSERT( _handle == ret);
        EnvMessage::viewerexecute();
    }

private:
    vector<float> _vpoints;
    boost::multi_array<float,2> _colors;
    SoSwitch* _handle;
};


GraphHandlePtr QtCoinViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawTriMeshMessage(shared_viewer(), handle, ppoints, stride, pIndices, numTriangles, color));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtCoinViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
{
    SoSwitch* handle = _createhandle();
    EnvMessagePtr pmsg(new DrawTriMeshColorMessage(shared_viewer(), handle, ppoints, stride, pIndices, numTriangles, colors));
    pmsg->callerexecute(false);
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

class CloseGraphMessage : public QtCoinViewer::EnvMessage
{
public:
    CloseGraphMessage(QtCoinViewerPtr pviewer, void** ppreturn, SoSwitch* handle)
        : EnvMessage(pviewer, ppreturn, false), _handle(handle) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        pviewer->_closegraph(_handle);
        EnvMessage::viewerexecute();
    }

private:
    SoSwitch* _handle;
};

void QtCoinViewer::closegraph(SoSwitch* handle)
{
    EnvMessagePtr pmsg(new CloseGraphMessage(shared_viewer(), (void**)NULL, handle));
    pmsg->callerexecute(false);
}

class SetGraphTransformMessage : public QtCoinViewer::EnvMessage
{
public:
    SetGraphTransformMessage(QtCoinViewerPtr pviewer, void** ppreturn, SoSwitch* handle, const RaveTransform<float>& t)
        : EnvMessage(pviewer, ppreturn, false), _handle(handle), _t(t) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        pviewer->_SetGraphTransform(_handle,_t);
        EnvMessage::viewerexecute();
    }

private:
    SoSwitch* _handle;
    RaveTransform<float> _t;
};

void QtCoinViewer::SetGraphTransform(SoSwitch* handle, const RaveTransform<float>& t)
{
    EnvMessagePtr pmsg(new SetGraphTransformMessage(shared_viewer(), (void**)NULL, handle, t));
    pmsg->callerexecute(false);
}

class SetGraphShowMessage : public QtCoinViewer::EnvMessage
{
public:
    SetGraphShowMessage(QtCoinViewerPtr pviewer, void** ppreturn, SoSwitch* handle, bool bshow)
        : EnvMessage(pviewer, ppreturn, false), _handle(handle), _bshow(bshow) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        pviewer->_SetGraphShow(_handle,_bshow);
        EnvMessage::viewerexecute();
    }

private:
    SoSwitch* _handle;
    bool _bshow;
};

void QtCoinViewer::SetGraphShow(SoSwitch* handle, bool bshow)
{
    EnvMessagePtr pmsg(new SetGraphShowMessage(shared_viewer(), (void**)NULL, handle, bshow));
    pmsg->callerexecute(false);
}

class DeselectMessage : public QtCoinViewer::EnvMessage
{
public:
    DeselectMessage(QtCoinViewerPtr pviewer, void** ppreturn)
        : EnvMessage(pviewer, ppreturn, false) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        pviewer->_deselect();
        EnvMessage::viewerexecute();
    }
};

void QtCoinViewer::deselect()
{
    EnvMessagePtr pmsg(new DeselectMessage(shared_viewer(), (void**)NULL));
    pmsg->callerexecute(false);
}

class ResetMessage : public QtCoinViewer::EnvMessage
{
public:
    ResetMessage(QtCoinViewerPtr pviewer, void** ppreturn)
        : EnvMessage(pviewer, ppreturn, true) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        pviewer->_Reset();
        EnvMessage::viewerexecute();
    }
};

void QtCoinViewer::Reset()
{
    if (_timerSensor->isScheduled() && _bUpdateEnvironment) {
        EnvMessagePtr pmsg(new ResetMessage(shared_viewer(), (void**)NULL));
        pmsg->callerexecute(false);
    }
}

boost::shared_ptr<void> QtCoinViewer::LockGUI()
{
    boost::shared_ptr<boost::mutex::scoped_lock> lock(new boost::mutex::scoped_lock(_mutexGUI));
    while(!_bInIdleThread) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
    return lock;
}

class SetBkgndColorMessage : public QtCoinViewer::EnvMessage
{
public:
    SetBkgndColorMessage(QtCoinViewerPtr pviewer, void** ppreturn, const RaveVector<float>& color)
        : EnvMessage(pviewer, ppreturn, false), _color(color) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        pviewer->_SetBkgndColor(_color);
        EnvMessage::viewerexecute();
    }

private:
    RaveVector<float> _color;
};

void QtCoinViewer::SetBkgndColor(const RaveVector<float>& color)
{
    if (_timerSensor->isScheduled() && _bUpdateEnvironment) {
        EnvMessagePtr pmsg(new SetBkgndColorMessage(shared_viewer(), (void**)NULL, color));
        pmsg->callerexecute(false);
    }
}

void QtCoinViewer::SetEnvironmentSync(bool bUpdate)
{
    boost::mutex::scoped_lock lockupdating(_mutexUpdating);
    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    _bUpdateEnvironment = bUpdate;
    _condUpdateModels.notify_all();

    if( !bUpdate ) {
        // remove all messages in order to release the locks
        boost::mutex::scoped_lock lockmsg(_mutexMessages);
        FOREACH(it,_listMessages) {
            (*it)->releasemutex();
        }
        _listMessages.clear();
    }
}

void QtCoinViewer::EnvironmentSync()
{
    {
        boost::mutex::scoped_lock lockupdating(_mutexUpdating);
        if( !_bUpdateEnvironment ) {
            RAVELOG_WARN("cannot update models from environment sync\n");
            return;
        }
    }

    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    _bModelsUpdated = false;
    _condUpdateModels.wait(lock);
    if( !_bModelsUpdated ) {
        RAVELOG_WARN("failed to update models from environment sync\n");
    }
}

void QtCoinViewer::_SetCamera(const RaveTransform<float>& _t, float focalDistance)
{
    _bAutoSetCamera = false;
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    RaveTransform<float> t = _t*trot;
    GetCamera()->position.setValue(t.trans.x, t.trans.y, t.trans.z);
    GetCamera()->orientation.setValue(t.rot.y, t.rot.z, t.rot.w, t.rot.x);
    if( focalDistance > 0 ) {
        GetCamera()->focalDistance = focalDistance;
    }
    _UpdateCameraTransform(0);
}

void QtCoinViewer::_SetBkgndColor(const RaveVector<float>& color)
{
    _pviewer->setBackgroundColor(SbColor(color.x, color.y, color.z));
    _ivOffscreen.setBackgroundColor(SbColor(color.x, color.y, color.z));
}

void QtCoinViewer::_closegraph(SoSwitch* handle)
{
    if( handle != NULL ) {
        _pFigureRoot->removeChild(handle);
    }
}

void QtCoinViewer::_SetGraphTransform(SoSwitch* handle, const RaveTransform<float>& t)
{
    if( handle != NULL ) {
        SoNode* pparent = handle->getChild(0);
        if((pparent != NULL)&&(pparent->getTypeId() == SoSeparator::getClassTypeId())) {
            SoNode* ptrans = ((SoSeparator*)pparent)->getChild(0);
            if((ptrans != NULL)&&(ptrans->getTypeId() == SoTransform::getClassTypeId())) {
                SetSoTransform((SoTransform*)ptrans, t);
            }
        }
    }
}

void QtCoinViewer::_SetGraphShow(SoSwitch* handle, bool bshow)
{
    if( handle != NULL ) {
        handle->whichChild = bshow ? SO_SWITCH_ALL : SO_SWITCH_NONE;
    }
}

void QtCoinViewer::PrintCamera()
{
    _UpdateCameraTransform(0);
    // have to flip Z axis
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    RaveTransform<float> T = _Tcamera*trot;
    Vector vaxis = axisAngleFromQuat(T.rot);
    dReal fangle = RaveSqrt(vaxis.lengthsqr3());
    vaxis *= (1/fangle);
    RAVELOG_INFO(str(boost::format("Camera Transformation:\n"
                                   "<camtrans>%f %f %f</camtrans>\n"
                                   "<camrotationaxis>%f %f %f %f</camrotationaxis>\n"
                                   "<camfocal>%f</camfocal>\n"
                                   "height angle: %f\n")%T.trans[0]%T.trans[1]%T.trans[2]%vaxis[0]%vaxis[1]%vaxis[2]%(fangle*180.0f/PI)%GetCamera()->focalDistance.getValue()%GetCamera()->heightAngle.getValue()));
}

RaveTransform<float> QtCoinViewer::GetCameraTransform() const
{
    boost::mutex::scoped_lock lock(_mutexMessages);
    // have to flip Z axis
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    return _Tcamera*trot;
}

float QtCoinViewer::GetCameraDistanceToFocus() const
{
    boost::mutex::scoped_lock lock(_mutexMessages);
    return _focalDistance;
}

geometry::RaveCameraIntrinsics<float> QtCoinViewer::GetCameraIntrinsics() const
{
    boost::mutex::scoped_lock lock(_mutexMessages);
    return _camintrinsics;
}

SensorBase::CameraIntrinsics QtCoinViewer::GetCameraIntrinsics2() const
{
    boost::mutex::scoped_lock lock(_mutexMessages);
    SensorBase::CameraIntrinsics intr;
    intr.fx = _camintrinsics.fx;
    intr.fy = _camintrinsics.fy;
    intr.cx = _camintrinsics.cx;
    intr.cy = _camintrinsics.cy;
    intr.distortion_model = _camintrinsics.distortion_model;
    intr.distortion_coeffs.resize(_camintrinsics.distortion_coeffs.size());
    std::copy(_camintrinsics.distortion_coeffs.begin(), _camintrinsics.distortion_coeffs.end(), intr.distortion_coeffs.begin());
    intr.focal_length = _camintrinsics.focal_length;
    return intr;
}

void* QtCoinViewer::_plot3(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color)
{
    if((handle == NULL)||(numPoints <= 0)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    SoMaterial* mtrl = new SoMaterial;
    mtrl->diffuseColor = SbColor(color.x, color.y, color.z);
    mtrl->ambientColor = SbColor(0,0,0);
    mtrl->transparency = max(0.0f,1.0f-color.w);
    mtrl->setOverride(true);
    pparent->addChild(mtrl);

    if( color.w < 1.0f ) {
        SoTransparencyType* ptype = new SoTransparencyType();
        ptype->value = SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND;
        pparent->addChild(ptype);
    }

    SoCoordinate3* vprop = new SoCoordinate3();

    if( stride != sizeof(float)*3 ) {
        vector<float> mypoints(numPoints*3);
        for(int i = 0; i < numPoints; ++i) {
            mypoints[3*i+0] = ppoints[0];
            mypoints[3*i+1] = ppoints[1];
            mypoints[3*i+2] = ppoints[2];
            ppoints = (float*)((char*)ppoints + stride);
        }

        vprop->point.setValues(0,numPoints,(float(*)[3])&mypoints[0]);
    }
    else {
        vprop->point.setValues(0,numPoints,(float(*)[3])ppoints);
    }
    pparent->addChild(vprop);

    SoDrawStyle* style = new SoDrawStyle();
    style->style = SoDrawStyle::POINTS;
    style->pointSize = fPointSize;
    pparent->addChild(style);

    SoPointSet* pointset = new SoPointSet();
    pointset->numPoints.setValue(-1);
    pparent->addChild(pointset);

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_plot3(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, bool bhasalpha)
{
    if((handle == NULL)||(numPoints <= 0)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    SoMaterial* mtrl = new SoMaterial;
    if( bhasalpha ) {
        vector<float> colorsonly(numPoints*3),alphaonly(numPoints);
        for(int i = 0; i < numPoints; ++i) {
            colorsonly[3*i+0] = colors[4*i+0];
            colorsonly[3*i+1] = colors[4*i+1];
            colorsonly[3*i+2] = colors[4*i+2];
            alphaonly[i] = 1-colors[4*i+3];
        }
        mtrl->diffuseColor.setValues(0, numPoints, (float(*)[3])&colorsonly[0]);
        mtrl->transparency.setValues(0,numPoints,(float*)&alphaonly[0]);
    }
    else
        mtrl->diffuseColor.setValues(0, numPoints, (float(*)[3])colors);
    mtrl->setOverride(true);
    pparent->addChild(mtrl);

    if( bhasalpha ) {
        SoTransparencyType* ptype = new SoTransparencyType();
        // SORTED_OBJECT_SORTED_TRIANGLE_BLEND fails to render points correctly if each have a different transparency value
        ptype->value = SoGLRenderAction::SORTED_OBJECT_BLEND;
        pparent->addChild(ptype);
    }

    SoMaterialBinding* pbinding = new SoMaterialBinding();
    pbinding->value = SoMaterialBinding::PER_VERTEX;
    pparent->addChild(pbinding);

    SoCoordinate3* vprop = new SoCoordinate3();

    if( stride != sizeof(float)*3 ) {
        vector<float> mypoints(numPoints*3);
        for(int i = 0; i < numPoints; ++i) {
            mypoints[3*i+0] = ppoints[0];
            mypoints[3*i+1] = ppoints[1];
            mypoints[3*i+2] = ppoints[2];
            ppoints = (float*)((char*)ppoints + stride);
        }

        vprop->point.setValues(0,numPoints,(float(*)[3])&mypoints[0]);
    }
    else
        vprop->point.setValues(0,numPoints,(float(*)[3])ppoints);

    pparent->addChild(vprop);

    SoDrawStyle* style = new SoDrawStyle();
    style->style = SoDrawStyle::POINTS;
    style->pointSize = fPointSize;
    pparent->addChild(style);

    SoPointSet* pointset = new SoPointSet();
    pointset->numPoints.setValue(-1);

    pparent->addChild(pointset);

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawspheres(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color)
{
    if((handle == NULL)||(ppoints == NULL)||(numPoints <= 0)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    for(int i = 0; i < numPoints; ++i) {
        SoSeparator* psep = new SoSeparator();
        SoTransform* ptrans = new SoTransform();

        ptrans->translation.setValue(ppoints[0], ppoints[1], ppoints[2]);

        psep->addChild(ptrans);
        pparent->addChild(psep);
        _SetMaterial(psep,color);

        SoSphere* c = new SoSphere();
        c->radius = fPointSize;
        psep->addChild(c);

        ppoints = (float*)((char*)ppoints + stride);
    }

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawspheres(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, bool bhasalpha)
{
    if((handle == NULL)||(ppoints == NULL)||(numPoints <= 0)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    int colorstride = bhasalpha ? 4 : 3;
    for(int i = 0; i < numPoints; ++i) {
        SoSeparator* psep = new SoSeparator();
        SoTransform* ptrans = new SoTransform();

        ptrans->translation.setValue(ppoints[0], ppoints[1], ppoints[2]);

        psep->addChild(ptrans);
        pparent->addChild(psep);

        // set a diffuse color
        SoMaterial* mtrl = new SoMaterial;
        mtrl->diffuseColor = SbColor(colors[colorstride*i +0], colors[colorstride*i +1], colors[colorstride*i +2]);
        mtrl->ambientColor = SbColor(0,0,0);
        if( bhasalpha )
            mtrl->transparency = max(0.0f,1-colors[colorstride*i+3]);
        mtrl->setOverride(true);
        psep->addChild(mtrl);

        if( bhasalpha &&(colors[colorstride*i+3] < 1)) {
            SoTransparencyType* ptype = new SoTransparencyType();
            ptype->value = SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND;
            pparent->addChild(ptype);
        }

        SoSphere* c = new SoSphere();
        c->radius = fPointSize;
        psep->addChild(c);

        ppoints = (float*)((char*)ppoints + stride);
    }

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawlinestrip(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    if((handle == NULL)||(numPoints < 2)||(ppoints == NULL)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    _SetMaterial(pparent,color);

    vector<float> mypoints((numPoints-1)*6);
    float* next;
    for(int i = 0; i < numPoints-1; ++i) {
        next = (float*)((char*)ppoints + stride);

        mypoints[6*i+0] = ppoints[0];
        mypoints[6*i+1] = ppoints[1];
        mypoints[6*i+2] = ppoints[2];
        mypoints[6*i+3] = next[0];
        mypoints[6*i+4] = next[1];
        mypoints[6*i+5] = next[2];

        ppoints = next;
    }

    SoCoordinate3* vprop = new SoCoordinate3();
    vprop->point.setValues(0,2*(numPoints-1),(float(*)[3])&mypoints[0]);
    pparent->addChild(vprop);

    SoDrawStyle* style = new SoDrawStyle();
    style->style = SoDrawStyle::LINES;
    style->lineWidth = fwidth;
    pparent->addChild(style);

    SoLineSet* pointset = new SoLineSet();
    vector<int> vinds(numPoints-1,2);
    pointset->numVertices.setValues(0,vinds.size(), &vinds[0]);

    pparent->addChild(pointset);

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawlinestrip(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    if((handle == NULL)||(numPoints < 2)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    SoMaterial* mtrl = new SoMaterial;
    mtrl->setOverride(true);
    pparent->addChild(mtrl);

    SoMaterialBinding* pbinding = new SoMaterialBinding();
    pbinding->value = SoMaterialBinding::PER_VERTEX;
    pparent->addChild(pbinding);

    vector<float> mypoints((numPoints-1)*6), mycolors((numPoints-1)*6);
    float* next;
    for(int i = 0; i < numPoints-1; ++i) {
        next = (float*)((char*)ppoints + stride);

        mypoints[6*i+0] = ppoints[0];
        mypoints[6*i+1] = ppoints[1];
        mypoints[6*i+2] = ppoints[2];
        mypoints[6*i+3] = next[0];
        mypoints[6*i+4] = next[1];
        mypoints[6*i+5] = next[2];

        mycolors[6*i+0] = colors[3*i+0];
        mycolors[6*i+1] = colors[3*i+1];
        mycolors[6*i+2] = colors[3*i+2];
        mycolors[6*i+3] = colors[3*i+3];
        mycolors[6*i+4] = colors[3*i+4];
        mycolors[6*i+5] = colors[3*i+5];

        ppoints = next;
    }

    mtrl->diffuseColor.setValues(0, 2*(numPoints-1), (float(*)[3])&mycolors[0]);

    SoCoordinate3* vprop = new SoCoordinate3();
    vprop->point.setValues(0,2*(numPoints-1),(float(*)[3])&mypoints[0]);
    pparent->addChild(vprop);

    SoDrawStyle* style = new SoDrawStyle();
    style->style = SoDrawStyle::LINES;
    style->lineWidth = fwidth;
    pparent->addChild(style);

    SoLineSet* pointset = new SoLineSet();
    vector<int> vinds(numPoints-1,2);
    pointset->numVertices.setValues(0,vinds.size(), &vinds[0]);

    pparent->addChild(pointset);

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawlinelist(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    if((handle == NULL)||(numPoints < 2)||(ppoints == NULL)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    _SetMaterial(pparent,color);

    vector<float> mypoints(numPoints*3);
    for(int i = 0; i < numPoints; ++i) {
        mypoints[3*i+0] = ppoints[0];
        mypoints[3*i+1] = ppoints[1];
        mypoints[3*i+2] = ppoints[2];
        ppoints = (float*)((char*)ppoints + stride);
    }

    SoCoordinate3* vprop = new SoCoordinate3();
    vprop->point.setValues(0,numPoints,(float(*)[3])&mypoints[0]);
    pparent->addChild(vprop);

    SoDrawStyle* style = new SoDrawStyle();
    style->style = SoDrawStyle::LINES;
    style->lineWidth = fwidth;
    pparent->addChild(style);

    SoLineSet* pointset = new SoLineSet();
    vector<int> vinds(numPoints/2,2);
    pointset->numVertices.setValues(0,vinds.size(), &vinds[0]);

    pparent->addChild(pointset);

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawlinelist(SoSwitch* handle, const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    if((handle == NULL)||(numPoints < 2)||(ppoints == NULL)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());

    boost::multi_array<float,2> vcolors; vcolors.resize(boost::extents[numPoints][3]);
    for(int i = 0; i < numPoints; ++i) {
        vcolors[i][0] = colors[3*i+0];
        vcolors[i][1] = colors[3*i+1];
        vcolors[i][2] = colors[3*i+2];
    }
    _SetMaterial(pparent,vcolors);

    vector<float> mypoints(numPoints*3);
    for(int i = 0; i < numPoints; ++i) {
        mypoints[3*i+0] = ppoints[0];
        mypoints[3*i+1] = ppoints[1];
        mypoints[3*i+2] = ppoints[2];
        ppoints = (float*)((char*)ppoints + stride);
    }

    SoCoordinate3* vprop = new SoCoordinate3();
    vprop->point.setValues(0,numPoints,(float(*)[3])&mypoints[0]);
    pparent->addChild(vprop);

    SoDrawStyle* style = new SoDrawStyle();
    style->style = SoDrawStyle::LINES;
    style->lineWidth = fwidth;
    pparent->addChild(style);

    SoLineSet* pointset = new SoLineSet();
    vector<int> vinds(numPoints/2,2);
    pointset->numVertices.setValues(0,vinds.size(), &vinds[0]);

    pparent->addChild(pointset);

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawarrow(SoSwitch* handle, const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
{
    if( handle == NULL ) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    SoSeparator* psep = new SoSeparator();
    SoTransform* ptrans = new SoTransform();

    SoDrawStyle* _style = new SoDrawStyle();
    _style->style = SoDrawStyle::FILLED;
    pparent->addChild(_style);

    RaveVector<float> direction = p2-p1;
    float fheight = RaveSqrt(direction.lengthsqr3());

    float coneheight = fheight/10.0f;

    direction.normalize3();
    //check to make sure points aren't the same
    if(RaveSqrt(direction.lengthsqr3()) < 0.9f)
    {
        RAVELOG_WARN("QtCoinViewer::drawarrow - Error: End points are the same.\n");
        return handle;
    }

    //rotate to face point
    RaveVector<float> qrot = quatRotateDirection(RaveVector<dReal>(0,1,0),RaveVector<dReal>(direction));
    RaveVector<float> vaxis = axisAngleFromQuat(qrot);
    dReal angle = RaveSqrt(vaxis.lengthsqr3());
    if( angle > 0 ) {
        vaxis *= 1/angle;
    }
    else {
        vaxis = RaveVector<float>(1,0,0);
    }
    ptrans->rotation.setValue(SbVec3f(vaxis.x, vaxis.y, vaxis.z), angle);

    //reusing direction vector for efficieny
    RaveVector<float> linetranslation = p1 + (fheight/2.0f-coneheight/2.0f)*direction;
    ptrans->translation.setValue(linetranslation.x, linetranslation.y, linetranslation.z);

    psep->addChild(ptrans);
    pparent->addChild(psep);

    // set a diffuse color
    _SetMaterial(pparent, color);

    SoCylinder* c = new SoCylinder();
    c->radius = fwidth;
    c->height = fheight-coneheight;
    psep->addChild(c);

    //place a cone for the arrow tip

    SoCone* cn = new SoCone();
    cn->bottomRadius = fwidth;
    cn->height = coneheight;

    ptrans = new SoTransform();
    ptrans->rotation.setValue(SbVec3f(vaxis.x, vaxis.y, vaxis.z), angle);
    linetranslation = p2 - coneheight/2.0f*direction;
    ptrans->translation.setValue(linetranslation.x, linetranslation.y, linetranslation.z);

    psep = new SoSeparator();

    psep->addChild(ptrans);
    psep->addChild(cn);

    pparent->addChild(psep);

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawbox(SoSwitch* handle, const RaveVector<float>& vpos, const RaveVector<float>& vextents)
{
    if( handle == NULL ) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    RAVELOG_ERROR("drawbox not implemented\n");

    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawplane(SoSwitch* handle, const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
{
    if( handle == NULL ) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    RaveTransformMatrix<float> m(tplane);
    Vector vright(m.m[0],m.m[4],m.m[8]),vup(m.m[1],m.m[5],m.m[9]),vdir(m.m[2],m.m[6],m.m[10]);

    SoTextureCombine* pcombine = new SoTextureCombine();
    pcombine->rgbSource = SoTextureCombine::TEXTURE;
    pcombine->rgbOperation = SoTextureCombine::REPLACE;

    if( vtexture.shape()[2] == 4 ) {
        SoTransparencyType* ptype = new SoTransparencyType();
        ptype->value = SoGLRenderAction::SORTED_OBJECT_BLEND;
        pparent->addChild(ptype);
        pcombine->alphaSource = SoTextureCombine::TEXTURE;
        pcombine->alphaOperation = SoTextureCombine::REPLACE;
    }

    pparent->addChild(pcombine);

    // the texture image
    SoTexture2 *tex = new SoTexture2;
    vector<unsigned char> vimagedata(vtexture.shape()[0]*vtexture.shape()[1]*vtexture.shape()[2]);
    if( vimagedata.size() > 0 ) {
        vector<unsigned char>::iterator itdst = vimagedata.begin();
        FOREACHC(ith,vtexture) {
            FOREACHC(itw,*ith) {
                FOREACHC(itp,*itw) {
                    *itdst++ = (unsigned char)(255.0f*CLAMP_ON_RANGE(*itp,0.0f,1.0f));
                }
            }
        }
        tex->image.setValue(SbVec2s(vtexture.shape()[1],vtexture.shape()[0]),vtexture.shape()[2],&vimagedata[0]);
    }
    tex->model = SoTexture2::REPLACE; // new features, but supports grayscale images
    tex->wrapS = SoTexture2::CLAMP;
    tex->wrapT = SoTexture2::CLAMP;
    pparent->addChild(tex);

    boost::array<RaveVector<float>, 4> vplanepoints = { { m.trans-vextents[0]*vright-vextents[1]*vup,
                                                          m.trans-vextents[0]*vright+vextents[1]*vup,
                                                          m.trans+vextents[0]*vright-vextents[1]*vup,
                                                          m.trans+vextents[0]*vright+vextents[1]*vup}};
    boost::array<float,8> texpoints = { { 0,0,0,1,1,0,1,1}};
    boost::array<int,6> indices = { { 0,1,2,1,2,3}};
    boost::array<float,18> vtripoints;
    boost::array<float,12> vtexpoints; /// points of plane
    for(int i = 0; i < 6; ++i) {
        RaveVector<float> v = vplanepoints[indices[i]];
        vtripoints[3*i+0] = v[0]; vtripoints[3*i+1] = v[1]; vtripoints[3*i+2] = v[2];
        vtexpoints[2*i+0] = texpoints[2*indices[i]+0]; vtexpoints[2*i+1] = texpoints[2*indices[i]+1];
    }
    SoCoordinate3* vprop = new SoCoordinate3();
    vprop->point.setValues(0,6,(float(*)[3])&vtripoints[0]);
    pparent->addChild(vprop);
    SoTextureCoordinate2* tprop = new SoTextureCoordinate2();
    tprop->point.setValues(0,6,(float(*)[2])&vtexpoints[0]);
    pparent->addChild(tprop);

    SoFaceSet* faceset = new SoFaceSet();
    faceset->numVertices.set1Value(0,3);
    faceset->numVertices.set1Value(1,3);
    //faceset->generateDefaultNormals(SoShape, SoNormalCache);
    pparent->addChild(faceset);

    _pFigureRoot->addChild(handle);
    return handle;
}

void QtCoinViewer::_SetMaterial(SoGroup* pparent, const RaveVector<float>& color)
{
    SoMaterial* mtrl = new SoMaterial;
    mtrl->diffuseColor = SbColor(color.x, color.y, color.z);
    mtrl->ambientColor = SbColor(0,0,0);
    mtrl->transparency = max(0.0f,1-color.w);
    mtrl->setOverride(true);
    pparent->addChild(mtrl);

    SoMaterialBinding* pbinding = new SoMaterialBinding();
    pbinding->value = SoMaterialBinding::OVERALL;
    pparent->addChild(pbinding);

    if( color.w < 1.0f ) {
        SoTransparencyType* ptype = new SoTransparencyType();
        ptype->value = SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND;
        pparent->addChild(ptype);
    }
}

void QtCoinViewer::_SetMaterial(SoGroup* pparent, const boost::multi_array<float,2>& colors)
{
    if( colors.size() == 0 )
        return;
    SoMaterial* mtrl = new SoMaterial;
    mtrl->ambientColor = SbColor(0,0,0);
    vector<float> vcolors(colors.shape()[0]*3);
    switch(colors.shape()[1]) {
    case 1:
        for(size_t i = 0; i < colors.shape()[0]; ++i) {
            vcolors[3*i+0] = colors[i][0];
            vcolors[3*i+1] = colors[i][0];
            vcolors[3*i+2] = colors[i][0];
        }
        break;
    case 4:
        for(size_t i = 0; i < colors.shape()[0]; ++i)
            vcolors[i] = 1.0f-colors[i][3];
        mtrl->transparency.setValues(0,colors.shape()[0],&vcolors[0]);
    case 3:
        for(size_t i = 0; i < colors.shape()[0]; ++i) {
            vcolors[3*i+0] = colors[i][0];
            vcolors[3*i+1] = colors[i][1];
            vcolors[3*i+2] = colors[i][2];
        }
        break;
    default:
        RAVELOG_WARN(str(boost::format("unsupported color dimension %d\n")%colors.shape()[1]));
        return;
    }

    mtrl->diffuseColor.setValues(0, colors.shape()[0], (float(*)[3])&vcolors[0]);
    mtrl->setOverride(true);
    pparent->addChild(mtrl);

    SoMaterialBinding* pbinding = new SoMaterialBinding();
    pbinding->value = SoMaterialBinding::PER_VERTEX;
    pparent->addChild(pbinding);

    if( colors.shape()[1] == 4 ) {
        SoTransparencyType* ptype = new SoTransparencyType();
        ptype->value = SoGLRenderAction::SORTED_OBJECT_SORTED_TRIANGLE_BLEND;
        pparent->addChild(ptype);
    }
}

void QtCoinViewer::_SetTriangleMesh(SoSeparator* pparent, const float* ppoints, int stride, const int* pIndices, int numTriangles)
{
    SoCoordinate3* vprop = new SoCoordinate3();

    if( pIndices != NULL ) {
        // this makes it crash!
        //vprop->point.set1Value(3*numTriangles-1,SbVec3f(0,0,0)); // resize
        for(int i = 0; i < 3*numTriangles; ++i) {
            float* p = (float*)((char*)ppoints + stride * pIndices[i]);
            vprop->point.set1Value(i, SbVec3f(p[0], p[1], p[2]));
        }
    }
    else {
        if( stride != sizeof(float)*3 ) {
            // this makes it crash!
            //vprop->point.set1Value(3*numTriangles-1,SbVec3f(0,0,0)); // resize
            for(int i = 0; i < 3*numTriangles; ++i) {
                vprop->point.set1Value(i, SbVec3f(ppoints[0], ppoints[1], ppoints[2]));
                ppoints = (float*)((char*)ppoints + stride);
            }
        }
        else {
            vprop->point.setValues(0,numTriangles*3,(float(*)[3])ppoints);
        }
    }

    pparent->addChild(vprop);

    SoFaceSet* faceset = new SoFaceSet();
    // this makes it crash!
    //faceset->numVertices.set1Value(numTriangles-1,3);
    for(int i = 0; i < numTriangles; ++i) {
        faceset->numVertices.set1Value(i,3);
    }
    //faceset->generateDefaultNormals(SoShape, SoNormalCache);

    pparent->addChild(faceset);
}

void* QtCoinViewer::_drawtrimesh(SoSwitch* handle, const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
{
    if((handle == NULL)||(ppoints == NULL)||(numTriangles <= 0)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    _SetMaterial(pparent,color);
    _SetTriangleMesh(pparent, ppoints,stride,pIndices,numTriangles);
    _pFigureRoot->addChild(handle);
    return handle;
}

void* QtCoinViewer::_drawtrimesh(SoSwitch* handle, const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
{
    if((handle == NULL)||(ppoints == NULL)||(numTriangles <= 0)) {
        return handle;
    }
    SoSeparator* pparent = new SoSeparator(); handle->addChild(pparent);
    pparent->addChild(new SoTransform());
    _SetMaterial(pparent, colors);
    _SetTriangleMesh(pparent, ppoints,stride,pIndices,numTriangles);
    _pFigureRoot->addChild(handle);
    return handle;
}

#define ADD_MENU(name, checkable, shortcut, tip, fn) { \
        pact = new QAction(tr(name), this); \
        if( checkable ) pact->setCheckable(checkable); \
        if( shortcut != NULL ) pact->setShortcut(tr(shortcut)); \
        if( tip != NULL ) pact->setStatusTip(tr(tip)); \
        if( checkable ) \
            connect(pact, SIGNAL(triggered(bool)), this, SLOT(fn(bool))); \
        else \
            connect(pact, SIGNAL(triggered()), this, SLOT(fn())); \
        pcurmenu->addAction(pact); \
        if( pgroup != NULL ) pgroup->addAction(pact); \
}

void QtCoinViewer::SetupMenus()
{
#if QT_VERSION >= 0x040000 // check for qt4
    QMenu* pcurmenu;
    QAction* pact;
    QActionGroup* pgroup = NULL;

    pcurmenu = menuBar()->addMenu(tr("&File"));
    ADD_MENU("Load Environment...", false, NULL, NULL, LoadEnvironment);
    ADD_MENU("Import Environment...", false, NULL, NULL, ImportEnvironment);
    ADD_MENU("Save Environment...", false, NULL, NULL, SaveEnvironment);
    pcurmenu->addSeparator();
    ADD_MENU("&Quit", false, NULL, NULL, Quit);

    pcurmenu = menuBar()->addMenu(tr("&View"));
    ADD_MENU("View Camera Params", false, NULL, NULL, ViewCameraParams);

    QMenu* psubmenu = pcurmenu->addMenu(tr("&Geometry"));
    pgroup = new QActionGroup(this);

    {
        pact = new QAction(tr("Render Only"), this);
        pact->setCheckable(true);
        pact->setChecked(_viewGeometryMode==VG_RenderOnly);
        pact->setData(VG_RenderOnly);
        psubmenu->addAction(pact);
        pgroup->addAction(pact);
    }
    {
        pact = new QAction(tr("Collision Only"), this);
        pact->setCheckable(true);
        pact->setChecked(_viewGeometryMode==VG_CollisionOnly);
        pact->setData(VG_CollisionOnly);
        psubmenu->addAction(pact);
        pgroup->addAction(pact);
    }
    {
        pact = new QAction(tr("Render/Collision"), this);
        pact->setCheckable(true);
        pact->setChecked(_viewGeometryMode==VG_RenderCollision);
        pact->setData(VG_RenderCollision);
        psubmenu->addAction(pact);
        pgroup->addAction(pact);
    }
    connect( pgroup, SIGNAL(triggered(QAction*)), this, SLOT(ViewGeometryChanged(QAction*)) );
    pgroup = NULL;

    // add debug levels
    psubmenu = pcurmenu->addMenu(tr("&Debug Levels"));
    _pToggleDebug = new QActionGroup(this);

    {
        pact = new QAction(tr("Fatal"), this);
        pact->setCheckable(true);
        pact->setChecked((RaveGetDebugLevel()&Level_OutputMask)==Level_Fatal);
        pact->setData(Level_Fatal);
        psubmenu->addAction(pact);
        _pToggleDebug->addAction(pact);
    }
    {
        pact = new QAction(tr("Error"), this);
        pact->setCheckable(true);
        pact->setChecked((RaveGetDebugLevel()&Level_OutputMask)==Level_Error);
        pact->setData(Level_Error);
        psubmenu->addAction(pact);
        _pToggleDebug->addAction(pact);
    }
    {
        pact = new QAction(tr("Warn"), this);
        pact->setCheckable(true);
        pact->setChecked((RaveGetDebugLevel()&Level_OutputMask)==Level_Warn);
        pact->setData(Level_Warn);
        psubmenu->addAction(pact);
        _pToggleDebug->addAction(pact);
    }
    {
        pact = new QAction(tr("Info"), this);
        pact->setCheckable(true);
        pact->setChecked((RaveGetDebugLevel()&Level_OutputMask)==Level_Info);
        pact->setData(Level_Info);
        psubmenu->addAction(pact);
        _pToggleDebug->addAction(pact);
    }
    {
        pact = new QAction(tr("Debug"), this);
        pact->setCheckable(true);
        pact->setChecked((RaveGetDebugLevel()&Level_OutputMask)==Level_Debug);
        pact->setData(Level_Debug);
        psubmenu->addAction(pact);
        _pToggleDebug->addAction(pact);
    }
    {
        pact = new QAction(tr("Verbose"), this);
        pact->setCheckable(true);
        pact->setChecked((RaveGetDebugLevel()&Level_OutputMask)==Level_Verbose);
        pact->setData(Level_Verbose);
        psubmenu->addAction(pact);
        _pToggleDebug->addAction(pact);
    }

    connect( _pToggleDebug, SIGNAL(triggered(QAction*)), this, SLOT(ViewDebugLevelChanged(QAction*)) );

    ADD_MENU("Show Framerate", true, NULL, "Toggle showing the framerate", ViewToggleFPS);
    ADD_MENU("Show World Axes", true, NULL, "Toggle showing the axis cross", ViewToggleFeedBack);

    pcurmenu = menuBar()->addMenu(tr("&Options"));
    ADD_MENU("&Record Real-time Video", true, NULL, "Start recording an AVI in real clock time. Clicking this menu item again will pause the recording", RecordRealtimeVideo);
    ADD_MENU("R&ecord Sim-time Video", true, NULL, "Start recording an AVI in simulation time. Clicking this menu item again will pause the recording", RecordSimtimeVideo);

    {
        psubmenu = pcurmenu->addMenu(tr("Video &Codecs"));
        QActionGroup* pVideoCodecs = new QActionGroup(this);
        ModuleBasePtr pvideocodecs = RaveCreateModule(GetEnv(),"viewerrecorder");
        stringstream scodecs, sin;
        if( !!pvideocodecs ) {
            sin << "GetCodecs";
            if (pvideocodecs->SendCommand(scodecs,sin) ) {
                pact = new QAction(tr("Default (mpeg4)"), this);
                pact->setCheckable(true);
                pact->setChecked(true);
                pact->setData((int)-1);
                psubmenu->addAction(pact);
                pVideoCodecs->addAction(pact);

                string mime_type, name;
                while(!scodecs.eof()) {
                    int index = -1;
                    scodecs >> index >> mime_type;
                    if( !scodecs ) {
                        break;
                    }
                    getline(scodecs,name);
                    boost::trim(name);
                    pact = new QAction(tr(name.c_str()), this);
                    pact->setCheckable(true);
                    pact->setChecked(false);
                    pact->setData(index);
                    psubmenu->addAction(pact);
                    pVideoCodecs->addAction(pact);
                }
                connect(pVideoCodecs, SIGNAL(triggered(QAction*)), this, SLOT(VideoCodecChanged(QAction*)) );
            }
        }
    }

    ADD_MENU("&Simulation", true, NULL, "Control environment simulation loop", ToggleSimulation);
    _pToggleSimulation = pact;

    std::map<InterfaceType, std::vector<std::string> > interfacenames;
    RaveGetLoadedInterfaces(interfacenames);

    psubmenu = pcurmenu->addMenu(tr("&Collision Checkers"));
    _pSelectedCollisionChecker = new QActionGroup(this);

    {
        pact = new QAction(tr("[None]"), this);
        pact->setCheckable(true);
        pact->setChecked(false);
        psubmenu->addAction(pact);
        _pSelectedCollisionChecker->addAction(pact);
    }

    FOREACH(itname,interfacenames[PT_CollisionChecker]) {
        pact = new QAction(tr(itname->c_str()), this);
        pact->setCheckable(true);
        pact->setChecked(false);
        psubmenu->addAction(pact);
        _pSelectedCollisionChecker->addAction(pact);
    }

    connect( _pSelectedCollisionChecker, SIGNAL(triggered(QAction*)), this, SLOT(CollisionCheckerChanged(QAction*)) );

    psubmenu = pcurmenu->addMenu(tr("&Physics Engines"));
    _pSelectedPhysicsEngine = new QActionGroup(this);

    {
        pact = new QAction(tr("[None]"), this);
        pact->setCheckable(true);
        pact->setChecked(false);
        psubmenu->addAction(pact);
        _pSelectedPhysicsEngine->addAction(pact);
    }

    FOREACH(itname,interfacenames[PT_PhysicsEngine]) {
        pact = new QAction(tr(itname->c_str()), this);
        pact->setCheckable(true);
        pact->setChecked(false);
        psubmenu->addAction(pact);
        _pSelectedPhysicsEngine->addAction(pact);
    }

    connect( _pSelectedPhysicsEngine, SIGNAL(triggered(QAction*)), this, SLOT(PhysicsEngineChanged(QAction*)) );

    pcurmenu = menuBar()->addMenu(tr("&Interfaces"));
    connect(pcurmenu, SIGNAL(aboutToShow()), this, SLOT(UpdateInterfaces()));
    _pMenuSendCommand = pcurmenu->addMenu(tr("&Send Command"));
    _pActionSendCommand = new QActionGroup(this);
    connect( _pActionSendCommand, SIGNAL(triggered(QAction*)), this, SLOT(InterfaceSendCommand(QAction*)) );

    psubmenu = pcurmenu->addMenu(tr("&Physics"));
    {
        pact = new QAction(tr("Self Collision"), this);
        pact->setCheckable(true);
        connect(pact, SIGNAL(triggered(bool)), this, SLOT(DynamicSelfCollision(bool)));
        psubmenu->addAction(pact);
        _pToggleSelfCollision = pact;
    }
    {
        pact = new QAction(tr("Set Gravity to -Z"), this);
        connect(pact, SIGNAL(triggered(bool)), this, SLOT(DynamicGravity()));
        psubmenu->addAction(pact);
    }

    pcurmenu = menuBar()->addMenu(tr("&Help"));
    ADD_MENU("About", false, NULL, NULL, About);

#endif
}

void QtCoinViewer::customEvent(QEvent * e)
{
    if (e->type() == CALLBACK_EVENT) {
        MyCallbackEvent* pe = dynamic_cast<MyCallbackEvent*>(e);
        if( !pe ) {
            RAVELOG_WARN("got a qt message that isn't of MyCallbackEvent, converting statically (dangerous)\n");
            pe = static_cast<MyCallbackEvent*>(e);
        }
        pe->_fn();
        e->setAccepted(true);
    }
}

bool QtCoinViewer::_StartViewerLoopCommand(ostream& sout, istream& sinput)
{
    bool bcallmain = false;
    sinput >> bcallmain;
    _nQuitMainLoop = -1;
    _StartPlaybackTimer();
    _pviewer->show();
    if( bcallmain ) {
        // calls the main GUI loop
        SoQt::mainLoop();
    }
    return true;
}

bool QtCoinViewer::_ShowCommand(ostream& sout, istream& sinput)
{
    int showtype=1;
    sinput >> showtype;
    if( showtype ) {
        _pviewer->show();
        // just in case?
        SoDB::enableRealTimeSensor(true);
        SoSceneManager::enableRealTimeUpdate(true);
    }
    else {
        _pviewer->hide();
    }
    return true;
}

bool QtCoinViewer::_TrackLinkCommand(ostream& sout, istream& sinput)
{
    bool bresetvelocity = true;
    std::string bodyname, linkname;
    float focalDistance = 0.0;
    Transform tTrackingLinkRelative;
    sinput >> bodyname >> linkname >> focalDistance >> bresetvelocity;
    if( focalDistance > 0 ) {
        GetCamera()->focalDistance = focalDistance; // TODO is this thread safe?
    }
    _ptrackinglink.reset();
    _ptrackingmanip.reset();
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    KinBodyPtr pbody = GetEnv()->GetKinBody(bodyname);
    if( !pbody ) {
        return false;
    }
    _ptrackinglink = pbody->GetLink(linkname);
    if( !!_ptrackinglink ) {
        sinput >> tTrackingLinkRelative;
        if( !!sinput ) {
            _tTrackingLinkRelative = tTrackingLinkRelative;
        }
        else {
            RAVELOG_WARN("failed to get tracking link relative trans\n");
            _tTrackingLinkRelative = Transform(); // use the identity
        }
    }
    if( bresetvelocity ) {
        _tTrackingCameraVelocity.trans = _tTrackingCameraVelocity.rot = Vector(); // reset velocity?
    }
    return !!_ptrackinglink;
}

bool QtCoinViewer::_TrackManipulatorCommand(ostream& sout, istream& sinput)
{
    bool bresetvelocity = true;
    std::string robotname, manipname;
    float focalDistance = 0.0;
    sinput >> robotname >> manipname >> focalDistance >> bresetvelocity;
    if( focalDistance > 0 ) {
        GetCamera()->focalDistance = focalDistance; // TODO is this thread safe?
    }
    _ptrackinglink.reset();
    _ptrackingmanip.reset();
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    RobotBasePtr probot = GetEnv()->GetRobot(robotname);
    if( !probot ) {
        return false;
    }
    _ptrackingmanip = probot->GetManipulator(manipname);
    if( bresetvelocity ) {
        _tTrackingCameraVelocity.trans = _tTrackingCameraVelocity.rot = Vector(); // reset velocity?
    }
    return !!_ptrackingmanip;
}

bool QtCoinViewer::_SetTrackingAngleToUpCommand(ostream& sout, istream& sinput)
{
    sinput >> _fTrackAngleToUp;
    return true;
}

int QtCoinViewer::main(bool bShow)
{
    _nQuitMainLoop = -1;
    _StartPlaybackTimer();

    // need the _nQuitMainLoop in case _pviewer->show() exits after a quitmainloop is called
    if( bShow ) {
        if( _nQuitMainLoop < 0 ) {
            _pviewer->show();
        }
        //SoQt::show(this);
    }
    if( _nQuitMainLoop < 0 ) {
        SoQt::mainLoop();
    }
    SetEnvironmentSync(false);
    return 0;
}

void QtCoinViewer::quitmainloop()
{
    _nQuitMainLoop = 1;
    bool bGuiThread = QThread::currentThread() == QCoreApplication::instance()->thread();
    if( !bGuiThread ) {
        SetEnvironmentSync(false);
    }
    SoQt::exitMainLoop();
    _nQuitMainLoop = 2;
}

void QtCoinViewer::InitOffscreenRenderer()
{
    _ivOffscreen.setComponents(SoOffscreenRenderer::RGB);
    _bCanRenderOffscreen = true;
}

void QtCoinViewer::DumpIvRoot(const char *filename, bool bBinaryFile )
{
    SoOutput outfile;

    if (!outfile.openFile(filename)) {
        std::cerr << "could not open the file: " << filename << endl;
        return;
    }

    if (bBinaryFile)
        outfile.setBinary(true);

    // useful for debugging hierarchy
    SoWriteAction writeAction(&outfile);
    writeAction.apply(_ivRoot);
    outfile.closeFile();
}

void QtCoinViewer::_SelectHandler(void * userData, class SoPath * path)
{
    ((QtCoinViewer*)userData)->_HandleSelection(path);
}

void QtCoinViewer::_DeselectHandler(void * userData, class SoPath * path)
{
    ((QtCoinViewer*)userData)->_HandleDeselection(path->getTail());
}

bool QtCoinViewer::_HandleSelection(SoPath *path)
{
    ItemPtr pItem;
    float scale = 1.0;
    bool bAllowRotation = true;

    // search the robots
    KinBody::JointPtr pjoint;
    bool bIK = false;

    // for loop necessary for 3D models that include files
    SoNode* node = NULL;
    for(int i = path->getLength()-1; i >= 0; --i) {
        node = path->getNode(i);

        // search the environment
        FOREACH(it, _mapbodies) {
            BOOST_ASSERT( !!it->second );
            if (it->second->ContainsIvNode(node)) {
                pItem = it->second;
                break;
            }
        }

        if( !!pItem )
            break;
    }

    if (!pItem) {
        _ivRoot->deselectAll();
        return false;
    }

    KinBodyItemPtr pKinBody = boost::dynamic_pointer_cast<KinBodyItem>(pItem);
    KinBody::LinkPtr pSelectedLink;
    if( !!pKinBody ) {
        pSelectedLink = pKinBody->GetLinkFromIv(node);
    }

    bool bProceedSelection = true;

    // check the callbacks
    if( !!pSelectedLink ) {
        boost::mutex::scoped_lock lock(_mutexCallbacks);
        FOREACH(it,_listRegisteredItemSelectionCallbacks) {
            bool bSame;
            {
                boost::mutex::scoped_lock lock(_mutexMessages);
                bSame = !_pMouseOverLink.expired() && KinBody::LinkPtr(_pMouseOverLink) == pSelectedLink;
            }
            if( bSame ) {
                ItemSelectionCallbackDataPtr pdata = boost::dynamic_pointer_cast<ItemSelectionCallbackData>(it->lock());
                if( !!pdata ) {
                    if( pdata->_callback(pSelectedLink,_vMouseSurfacePosition,_vMouseRayDirection) ) {
                        bProceedSelection = false;
                    }
                }
            }
        }
    }

    if( !bProceedSelection ) {
        _ivRoot->deselectAll();
        return false;
    }

    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv = LockEnvironment(100000);
    if( !lockenv ) {
        _ivRoot->deselectAll();
        RAVELOG_WARN("failed to grab environment lock\n");
        return false;
    }

    if( ControlDown() ) {
        if( !!pSelectedLink ) {
            // search the joint nodes
            FOREACHC(itjoint, pKinBody->GetBody()->GetJoints()) {
                if( (((*itjoint)->GetFirstAttached()==pSelectedLink)||((*itjoint)->GetSecondAttached()==pSelectedLink)) ) {
                    if( !(*itjoint)->IsStatic() ) {
                        // joint has a range, so consider it for selection
                        if( pKinBody->GetBody()->DoesAffect((*itjoint)->GetJointIndex(), pSelectedLink->GetIndex()) )
                            pjoint = *itjoint;
                    }
                    else {
                        KinBody::LinkPtr pother;
                        if( (*itjoint)->GetFirstAttached()==pSelectedLink )
                            pother = (*itjoint)->GetSecondAttached();
                        else if( (*itjoint)->GetSecondAttached()==pSelectedLink )
                            pother = (*itjoint)->GetFirstAttached();
                        if( !!pother ) {
                            FOREACHC(itjoint2, pKinBody->GetBody()->GetJoints()) {
                                if( !(*itjoint2)->IsStatic() && pKinBody->GetBody()->DoesAffect((*itjoint2)->GetJointIndex(), pother->GetIndex()) && (((*itjoint2)->GetFirstAttached()==pother)||((*itjoint2)->GetSecondAttached()==pother)) ) {
                                    pjoint = *itjoint2;
                                    break;
                                }
                            }
                        }
                    }

                    if( !!pjoint )
                        break;
                }
            }

            if( !pjoint ) {
                std::vector<int> vmimicdofs;
                FOREACHC(itjoint, pKinBody->GetBody()->GetPassiveJoints()) {
                    if( (*itjoint)->IsMimic() ) {
                        for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                            if( (*itjoint)->IsMimic(idof) ) {
                                (*itjoint)->GetMimicDOFIndices(vmimicdofs,idof);
                                FOREACHC(itmimicdof, vmimicdofs) {
                                    KinBody::JointPtr ptempjoint = pKinBody->GetBody()->GetJointFromDOFIndex(*itmimicdof);
                                    if( !ptempjoint->IsStatic() && pKinBody->GetBody()->DoesAffect(ptempjoint->GetJointIndex(), pSelectedLink->GetIndex()) &&
                                        (((*itjoint)->GetFirstAttached()==pSelectedLink)||((*itjoint)->GetSecondAttached()==pSelectedLink)) ) {
                                        pjoint = ptempjoint;
                                        break;
                                    }
                                }
                                if( !!pjoint ) {
                                    break;
                                }
                            }
                        }
                    }
                    else {
                        KinBody::LinkPtr pother;
                        if( (*itjoint)->GetFirstAttached()==pSelectedLink ) {
                            pother = (*itjoint)->GetSecondAttached();
                        }
                        else if( (*itjoint)->GetSecondAttached()==pSelectedLink ) {
                            pother = (*itjoint)->GetFirstAttached();
                        }
                        if( !!pother ) {
                            FOREACHC(itjoint2, pKinBody->GetBody()->GetJoints()) {
                                if( !(*itjoint2)->IsStatic() && pKinBody->GetBody()->DoesAffect((*itjoint2)->GetJointIndex(), pother->GetIndex()) && (((*itjoint2)->GetFirstAttached()==pother)||((*itjoint2)->GetSecondAttached()==pother)) ) {
                                    pjoint = *itjoint2;
                                    break;
                                }
                            }
                        }

                        if( !!pjoint ) {
                            break;
                        }
                    }
                }
            }

            if( !pjoint ) {
                return false;
            }
        }
    }

    // construct an appropriate _pdragger
    if (!!pjoint) {
        _pdragger.reset(new IvJointDragger(shared_viewer(), pItem, pSelectedLink->GetIndex(), scale, pjoint->GetJointIndex(), _bJointHilit));
    }
    else if (!bIK) {
        _pdragger.reset(new IvObjectDragger(shared_viewer(), pItem, scale, bAllowRotation));
    }
    else {
        //_pdragger = new IvIKDragger(this, pItem, scale, bAxis);
    }

    _pdragger->CheckCollision(true);
    pItem->SetGrab(true);

    BOOST_ASSERT(!_pSelectedItem);
    _pSelectedItem = pItem;

    // record the initially selected transform
    _initSelectionTrans = GetRaveTransform(pItem->GetIvTransform());

    return true;
}

void QtCoinViewer::_deselect()
{
    _pdragger.reset();
    _plistdraggers.clear();
    if( !!_pSelectedItem ) {
        _pSelectedItem->SetGrab(false);
        _pSelectedItem.reset();
        _ivRoot->deselectAll();
    }
}

boost::shared_ptr<EnvironmentMutex::scoped_try_lock> QtCoinViewer::LockEnvironment(uint64_t timeout,bool bUpdateEnvironment)
{
    // try to acquire the lock
#if BOOST_VERSION >= 103500
    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),boost::defer_lock_t()));
#else
    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),false));
#endif
    uint64_t basetime = utils::GetMicroTime();
    while(utils::GetMicroTime()-basetime<timeout ) {
        if( lockenv->try_lock() ) {
            break;
        }
        if( bUpdateEnvironment ) {
            _UpdateEnvironment(0);
        }
    }

    if( !*lockenv ) {
        lockenv.reset();
    }
    return lockenv;
}

bool QtCoinViewer::_HandleDeselection(SoNode *node)
{
    _pdragger.reset();
    _plistdraggers.clear();
    if( !!_pSelectedItem ) {
        _pSelectedItem->SetGrab(false);
        _pSelectedItem.reset();
    }
    return true;
}

// Keyboard callback handler
void QtCoinViewer::_KeyHandler(void * userData, class SoEventCallback * eventCB)
{
    QtCoinViewer* viewer = (QtCoinViewer*) userData;
    const SoEvent *event = eventCB->getEvent();

    if (SO_KEY_PRESS_EVENT(event, LEFT_ALT) )
        viewer->_altDown[0] = true;
    else if (SO_KEY_RELEASE_EVENT(event, LEFT_ALT) )
        viewer->_altDown[0] = false;
    if(SO_KEY_PRESS_EVENT(event, RIGHT_ALT))
        viewer->_altDown[1] = true;
    else if(SO_KEY_RELEASE_EVENT(event, RIGHT_ALT))
        viewer->_altDown[1] = false;

    if (SO_KEY_PRESS_EVENT(event, LEFT_CONTROL) ) {
        viewer->_ctrlDown[0] = true;
    }
    else if (SO_KEY_RELEASE_EVENT(event, LEFT_CONTROL) ) {
        viewer->_ctrlDown[0] = false;
    }

    if( SO_KEY_PRESS_EVENT(event, RIGHT_CONTROL)) {
        viewer->_ctrlDown[1] = true;
    }
    else if( SO_KEY_RELEASE_EVENT(event, RIGHT_CONTROL)) {
        viewer->_ctrlDown[1] = false;
    }
}

void QtCoinViewer::GlobAdvanceFrame(void* p, SoSensor*)
{
    BOOST_ASSERT( p != NULL );
    ((QtCoinViewer*)p)->AdvanceFrame(true);
}

void QtCoinViewer::AdvanceFrame(bool bForward)
{
    // frame counting
    static int nToNextFPSUpdate = 1;
    static int UPDATE_FRAMES = 16;
    static uint32_t basetime = utils::GetMilliTime();
    static uint32_t nFrame = 0;
    static float fFPS = 0;

    //    {
    //        _bInIdleThread = true;
    //        boost::mutex::scoped_lock lock(_mutexGUI);
    //        _bInIdleThread = false;
    //    }

    if( --nToNextFPSUpdate <= 0 ) {
        uint32_t newtime = utils::GetMilliTime();
        fFPS = UPDATE_FRAMES * 1000.0f / (float)max((int)(newtime-basetime),1);
        basetime = newtime;
        if( fFPS < 16 ) {
            UPDATE_FRAMES = 4;
        }
        else if( fFPS < 32 ) {
            UPDATE_FRAMES = 8;
        }
        else {
            UPDATE_FRAMES = 16;
        }
        nToNextFPSUpdate = UPDATE_FRAMES;
    }

    if( (nFrame++%16) == 0 ) {
        stringstream ss;

        if( _bDisplayFPS ) {
            ss << "fps: " << fixed << setprecision(2) << fFPS << ", simulation time: " << setprecision(4) << GetEnv()->GetSimulationTime()*1e-6 << "s" << endl;
        }

        if( !_pviewer->isViewing() ) {
            boost::mutex::scoped_lock lock(_mutexMessages);
            ss << _strMouseMove;
        }

        if( !!_pdragger ) {
            _pdragger->GetMessage(ss);
        }

        // adjust the shadow text
        SbViewportRegion v = _pviewer->getViewportRegion();
        float fwratio = 964.0f/v.getWindowSize()[0], fhratio = 688.0f/v.getWindowSize()[1];
        _messageShadowTranslation->translation.setValue(SbVec3f(-0.002f*fwratio,0.032f*fhratio,0));

        // search for all new lines
        string msg = ss.str();
        for(size_t i = 0; i < _messageNodes.size(); ++i) {
            _messageNodes[i]->string.setValue("");
        }
        int index = 0;
        std::string::size_type pos = 0, newpos=0;
        while( pos < msg.size() ) {
            newpos = msg.find('\n', pos);
            std::string::size_type n = newpos == std::string::npos ? msg.size()-pos : (newpos-pos);

            for(size_t i = 0; i < _messageNodes.size(); ++i) {
                _messageNodes[i]->string.set1Value(index++, msg.substr(pos, n).c_str());
            }

            if( newpos == std::string::npos ) {
                break;
            }
            pos = newpos+1;
        }
    }

    if( !!_pToggleDebug ) {
        _pToggleDebug->actions().at(RaveGetDebugLevel()&Level_OutputMask)->setChecked(true);
    }
    _UpdateToggleSimulation();
    _UpdateCollisionChecker();
    _UpdatePhysicsEngine();
    _UpdateEnvironment(TIMER_SENSOR_INTERVAL);

    {
        std::list<UserDataWeakPtr> listRegisteredViewerThreadCallbacks;
        {
            boost::mutex::scoped_lock lock(_mutexCallbacks);
            listRegisteredViewerThreadCallbacks = _listRegisteredViewerThreadCallbacks;
        }
        FOREACH(it,listRegisteredViewerThreadCallbacks) {
            ViewerThreadCallbackDataPtr pdata = boost::dynamic_pointer_cast<ViewerThreadCallbackData>(it->lock());
            if( !!pdata ) {
                try {
                    pdata->_callback();
                }
                catch(const std::exception& e) {
                    RAVELOG_ERROR(str(boost::format("Viewer Thread Callback Failed with error %s")%e.what()));
                }
            }
        }
    }

    if( ControlDown() ) {

    }
}

void QtCoinViewer::_UpdateEnvironment(float fTimeElapsed)
{
    boost::mutex::scoped_lock lockupd(_mutexUpdating);

    if( _bUpdateEnvironment ) {
        // process all messages
        list<EnvMessagePtr> listmessages;
        {
            boost::mutex::scoped_lock lockmsg(_mutexMessages);
            listmessages.swap(_listMessages);
            BOOST_ASSERT( _listMessages.size() == 0 );
        }

        FOREACH(itmsg, listmessages) {
            (*itmsg)->viewerexecute();
        }

        // have to update model after messages since it can lock the environment
        UpdateFromModel();
        _UpdateCameraTransform(fTimeElapsed);

        // this causes the GUI links to jitter when moving the joints, perhaps there is fighting somewhere?
        //        if( !!_pdragger ) {
        //            _pdragger->UpdateSkeleton();
        //        }
    }
}

bool QtCoinViewer::ForceUpdatePublishedBodies()
{
    {
        boost::mutex::scoped_lock lockupdating(_mutexUpdating);
        if( !_bUpdateEnvironment )
            return false;
    }

    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    GetEnv()->UpdatePublishedBodies();

    _bModelsUpdated = false;
    _bLockEnvironment = false; // notify UpdateFromModel to update without acquiring the lock
    _condUpdateModels.wait(lock);
    _bLockEnvironment = true; // reste
    return _bModelsUpdated;
}

void QtCoinViewer::GlobVideoFrame(void* p, SoSensor*)
{
    BOOST_ASSERT( p != NULL );
    ((QtCoinViewer*)p)->_VideoFrame();
}

void QtCoinViewer::_VideoFrame()
{
    std::list<UserDataWeakPtr> listRegisteredViewerImageCallbacks;
    {
        boost::mutex::scoped_lock lock(_mutexCallbacks);
        if( _listRegisteredViewerImageCallbacks.size() == 0 ) {
            return;
        }
        listRegisteredViewerImageCallbacks = _listRegisteredViewerImageCallbacks;
    }

    const uint8_t* memory;
    memory = _GetVideoFrame();
    if( !memory ) {
        RAVELOG_WARN("cannot record video\n");
        return;
    }
    FOREACH(it,listRegisteredViewerImageCallbacks) {
        ViewerImageCallbackDataPtr pdata = boost::dynamic_pointer_cast<ViewerImageCallbackData>(it->lock());
        if( !!pdata ) {
            try {
                pdata->_callback(memory,_nRenderWidth,_nRenderHeight,3);
            }
            catch(const std::exception& e) {
                RAVELOG_ERROR(str(boost::format("Viewer Image Callback Failed with error %s")%e.what()));
            }
        }
    }
}

void QtCoinViewer::UpdateFromModel()
{
    {
        boost::mutex::scoped_lock lock(_mutexItems);
        FOREACH(it,_listRemoveItems) {
            delete *it;
        }
        _listRemoveItems.clear();
    }

    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    vector<KinBody::BodyState> vecbodies;


#if BOOST_VERSION >= 103500
    EnvironmentMutex::scoped_try_lock lockenv(GetEnv()->GetMutex(),boost::defer_lock_t());
#else
    EnvironmentMutex::scoped_try_lock lockenv(GetEnv()->GetMutex(),false);
#endif

    if( _bLockEnvironment && !lockenv ) {
        uint64_t basetime = utils::GetMicroTime();
        while(utils::GetMicroTime()-basetime<1000 ) {
            if( lockenv.try_lock() ) {
                // acquired the lock, so update the bodies!
                GetEnv()->UpdatePublishedBodies();
                break;
            }
        }
    }

    try {
        GetEnv()->GetPublishedBodies(vecbodies,100000); // 0.1s
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN("timeout of GetPublishedBodies\n");
        return;
    }
    FOREACH(it, _mapbodies) {
        it->second->SetUserData(0);
    }

    FOREACH(itbody, vecbodies) {
        BOOST_ASSERT( !!itbody->pbody );
        KinBodyPtr pbody = itbody->pbody; // try to use only as an id, don't call any methods!
        KinBodyItemPtr pitem = boost::dynamic_pointer_cast<KinBodyItem>(pbody->GetUserData("qtcoinviewer"));

        if( !!pitem ) {
            if( !pitem->GetBody() ) {
                // looks like item has been destroyed, so remove from data
                pbody->RemoveUserData("qtcoinviewer");
                pitem.reset();
            }
        }
        if( !pitem ) {
            // make sure pbody is actually present
            if( GetEnv()->GetBodyFromEnvironmentId(itbody->environmentid) == pbody ) {

                // check to make sure the real GUI data is also NULL
                if( !pbody->GetUserData("qtcoinviewer") ) {
                    if( _mapbodies.find(pbody) != _mapbodies.end() ) {
                        RAVELOG_WARN_FORMAT("body %s already registered!", pbody->GetName());
                        continue;
                    }

                    if( _bLockEnvironment && !lockenv ) {
                        uint64_t basetime = utils::GetMicroTime();
                        while(utils::GetMicroTime()-basetime<1000 ) {
                            if( lockenv.try_lock() )  {
                                break;
                            }
                        }
                        if( !lockenv ) {
                            return; // couldn't acquire the lock, try next time. This prevents deadlock situations
                        }
                    }

                    if( pbody->IsRobot() ) {
                        pitem = boost::shared_ptr<RobotItem>(new RobotItem(shared_viewer(), RaveInterfaceCast<RobotBase>(pbody), _viewGeometryMode),ITEM_DELETER);
                    }
                    else {
                        pitem = boost::shared_ptr<KinBodyItem>(new KinBodyItem(shared_viewer(), pbody, _viewGeometryMode),ITEM_DELETER);
                    }

                    if( !!_pdragger && _pdragger->GetSelectedItem() == pitem ) {
                        _deselect();
                    }
                    pitem->Load();
                    pbody->SetUserData("qtcoinviewer",pitem);
                    _mapbodies[pbody] = pitem;
                }
                else {
                    pitem = boost::dynamic_pointer_cast<KinBodyItem>(pbody->GetUserData("qtcoinviewer"));
                    BOOST_ASSERT( _mapbodies.find(pbody) != _mapbodies.end() && _mapbodies[pbody] == pitem );
                }
            }
            else {
                // body is gone
                continue;
            }
        }

        map<KinBodyPtr, KinBodyItemPtr>::iterator itmap = _mapbodies.find(pbody);

        if( itmap == _mapbodies.end() ) {
            //RAVELOG_VERBOSE("body %s doesn't have a map associated with it!\n", itbody->strname.c_str());
            continue;
        }

        BOOST_ASSERT( pitem->GetBody() == pbody);
        BOOST_ASSERT( itmap->second == pitem );

        pitem->SetUserData(1);
        pitem->UpdateFromModel(itbody->jointvalues,itbody->vectrans);
    }

    FOREACH_NOINC(it, _mapbodies) {
        if( !it->second->GetUserData() ) {
            // item doesn't exist anymore, remove it
            it->first->RemoveUserData("qtcoinviewer");

            if( _pSelectedItem == it->second ) {
                _pdragger.reset();
                _pSelectedItem.reset();
                _ivRoot->deselectAll();
            }

            _mapbodies.erase(it++);
        }
        else {
            ++it;
        }
    }

    _bModelsUpdated = true;
    _condUpdateModels.notify_all();

    if( _bAutoSetCamera &&(_mapbodies.size() > 0)) {
        RAVELOG_DEBUG("auto-setting camera location\n");
        _bAutoSetCamera = false;
        _pviewer->viewAll();
    }
}

void QtCoinViewer::_Reset()
{
    _deselect();
    UpdateFromModel();
    _condUpdateModels.notify_all();

    FOREACH(itbody, _mapbodies) {
        BOOST_ASSERT( itbody->first->GetUserData("qtcoinviewer") == itbody->second );
        itbody->first->RemoveUserData("qtcoinviewer");
    }
    _mapbodies.clear();

    if( !!_pvideorecorder ) {
        SoDB::enableRealTimeSensor(true);
        SoSceneManager::enableRealTimeUpdate(true);
        _pvideorecorder.reset();
    }

    {
        boost::mutex::scoped_lock lock(_mutexItems);
        FOREACH(it,_listRemoveItems) {
            delete *it;
        }
        _listRemoveItems.clear();
    }
}

void QtCoinViewer::_UpdateCameraTransform(float fTimeElapsed)
{
    boost::mutex::scoped_lock lock(_mutexMessages);

    SbVec3f pos = GetCamera()->position.getValue();
    _Tcamera.trans = RaveVector<float>(pos[0], pos[1], pos[2]);

    SbVec3f axis;
    float fangle;
    GetCamera()->orientation.getValue(axis, fangle);
    _Tcamera.rot = quatFromAxisAngle(RaveVector<float>(axis[0],axis[1],axis[2]),fangle);

    if( fTimeElapsed > 0 ) {
        // animate the camera if necessary
        bool bTracking=false;
        Transform tTrack;
        KinBody::LinkPtr ptrackinglink = _ptrackinglink;
        if( !!ptrackinglink ) {
            bTracking = true;
            tTrack = ptrackinglink->GetTransform()*_tTrackingLinkRelative;
            //tTrack.trans = ptrackinglink->ComputeAABB().pos;
        }
        RobotBase::ManipulatorPtr ptrackingmanip=_ptrackingmanip;
        if( !!ptrackingmanip ) {
            bTracking = true;
            tTrack = ptrackingmanip->GetTransform();
        }

        if( bTracking ) {
            RaveVector<float> vup(0,0,1); // up vector that camera should always be oriented to
            RaveVector<float> vlookatdir = _Tcamera.trans - tTrack.trans;
            vlookatdir -= vup*vup.dot3(vlookatdir);
            float flookatlen = sqrtf(vlookatdir.lengthsqr3());
            vlookatdir = vlookatdir*cosf(_fTrackAngleToUp) + flookatlen*sinf(_fTrackAngleToUp)*vup; // flookatlen shouldn't change
            if( flookatlen > g_fEpsilon ) {
                vlookatdir *= 1/flookatlen;
            }
            else {
                vlookatdir = Vector(1,0,0);
            }
            vup -= vlookatdir*vlookatdir.dot3(vup);
            vup.normalize3();

            //RaveVector<float> vcameradir = ExtractAxisFromQuat(_Tcamera.rot, 2);
            //RaveVector<float> vToDesiredQuat = quatRotateDirection(vcameradir, vlookatdir);
            //RaveVector<float> vDestQuat = quatMultiply(vToDesiredQuat, _Tcamera.rot);
            //vDestQuat = quatMultiply(quatRotateDirection(ExtractAxisFromQuat(vDestQuat,1), vup), vDestQuat);
            float angle = normalizeAxisRotation(vup, _Tcamera.rot).first;
            RaveVector<float> vDestQuat = quatMultiply(quatFromAxisAngle(vup, -angle), quatRotateDirection(RaveVector<float>(0,1,0), vup));
            //transformLookat(tTrack.trans, _Tcamera.trans, vup);

            // focal distance is the tracking radius. ie how far from the coord system camera shoud be
            RaveVector<float> vDestPos = tTrack.trans + ExtractAxisFromQuat(vDestQuat,2)*GetCamera()->focalDistance.getValue();

            if(1) {
                // PID animation
                float pconst = 0.02;
                float dconst = 0.2;
                RaveVector<float> newtrans = _Tcamera.trans + fTimeElapsed*_tTrackingCameraVelocity.trans;
                newtrans += pconst*(vDestPos - _Tcamera.trans); // proportional term
                newtrans -= dconst*_tTrackingCameraVelocity.trans*fTimeElapsed; // derivative term (target is zero velocity)

                pconst = 0.001;
                dconst = 0.04;
                RaveVector<float> newquat = _Tcamera.rot + fTimeElapsed*_tTrackingCameraVelocity.rot;
                newquat += pconst*(vDestQuat - _Tcamera.rot);
                newquat -= dconst*_tTrackingCameraVelocity.rot*fTimeElapsed;
                newquat.normalize4();
                // have to make sure newquat's y vector aligns with vup

                _tTrackingCameraVelocity.trans = (newtrans-_Tcamera.trans)*(2/fTimeElapsed) - _tTrackingCameraVelocity.trans;
                _tTrackingCameraVelocity.rot = (newquat-_Tcamera.rot)*(2/fTimeElapsed) - _tTrackingCameraVelocity.rot;
                _Tcamera.trans = newtrans;
                _Tcamera.rot = newquat;
            }
            else {
                _Tcamera.trans = vDestPos;
                _Tcamera.rot = vDestQuat;
            }
            GetCamera()->position.setValue(_Tcamera.trans.x, _Tcamera.trans.y, _Tcamera.trans.z);
            GetCamera()->orientation.setValue(_Tcamera.rot.y, _Tcamera.rot.z, _Tcamera.rot.w, _Tcamera.rot.x);
        }
    }

    int width = centralWidget()->size().width();
    int height = centralWidget()->size().height();
    _ivCamera->aspectRatio = (float)view1->size().width() / (float)view1->size().height();

    _focalDistance = GetCamera()->focalDistance.getValue();
    _camintrinsics.fy = 0.5*height/RaveTan(0.5f*GetCamera()->heightAngle.getValue());
    _camintrinsics.fx = (float)width*_camintrinsics.fy/((float)height*GetCamera()->aspectRatio.getValue());
    _camintrinsics.cx = (float)width/2;
    _camintrinsics.cy = (float)height/2;
    _camintrinsics.focal_length = GetCamera()->nearDistance.getValue();
    _camintrinsics.distortion_model = "";
}

// menu items
void QtCoinViewer::LoadEnvironment()
{
#if QT_VERSION >= 0x040000 // check for qt4
    QString s = QFileDialog::getOpenFileName( this, "Load Environment", NULL, "Environment Files (*.xml *.dae *.zae)");
    if( s.length() == 0 )
        return;

    bool bReschedule = false;
    if (_timerSensor->isScheduled()) {
        _timerSensor->unschedule();
        bReschedule = true;
    }

    _Reset();
    GetEnv()->Reset();

    _bAutoSetCamera = true;
    GetEnv()->Load(s.toAscii().data());
    if( bReschedule ) {
        _timerSensor->schedule();
    }
#endif
}

void QtCoinViewer::ImportEnvironment()
{
#if QT_VERSION >= 0x040000 // check for qt4
    QString s = QFileDialog::getOpenFileName( this, "Load Environment", NULL, "Environment Files (*.xml *.dae *.zae)");
    if( s.length() == 0 )
        return;

    GetEnv()->Load(s.toAscii().data());
#endif
}

void QtCoinViewer::SaveEnvironment()
{
#if QT_VERSION >= 0x040000 // check for qt4
    QString s = QFileDialog::getSaveFileName( this, "Save Environment", NULL, "COLLADA Files (*.dae)");
    if( s.length() == 0 )
        return;

    GetEnv()->Save(s.toAscii().data());
#endif
}

void QtCoinViewer::Quit()
{
    close();
}

void QtCoinViewer::ViewCameraParams()
{
    stringstream ss; ss << "BarrettWAM_camera";
    PrintCamera();
}

void QtCoinViewer::ViewGeometryChanged(QAction* pact)
{
    _viewGeometryMode = (ViewGeometry)pact->data().toInt();

    // destroy all bodies
    _deselect();

    UpdateFromModel();
    FOREACH(itbody, _mapbodies) {
        BOOST_ASSERT( itbody->first->GetUserData("qtcoinviewer") == itbody->second );
        itbody->first->RemoveUserData("qtcoinviewer");
    }
    _mapbodies.clear();

    {
        boost::mutex::scoped_lock lock(_mutexItems);
        FOREACH(it,_listRemoveItems) {
            delete *it;
        }
        _listRemoveItems.clear();
    }
}

void QtCoinViewer::ViewDebugLevelChanged(QAction* pact)
{
    GetEnv()->SetDebugLevel((DebugLevel)pact->data().toInt());
}

void QtCoinViewer::VideoCodecChanged(QAction* pact)
{
    _videocodec = pact->data().toInt();
}

void QtCoinViewer::ViewToggleFPS(bool on)
{
    _bDisplayFPS = on;
    if( !_bDisplayFPS ) {
        for(size_t i = 0; i < _messageNodes.size(); ++i) {
            _messageNodes[i]->string.setValue("");
        }
    }
}

void QtCoinViewer::ViewToggleFeedBack(bool on)
{
    _bDisplayFeedBack = on;
    _pviewer->setFeedbackVisibility(on);
    if( !_bDisplayFeedBack ) {
        for(size_t i = 0; i < _messageNodes.size(); ++i) {
            _messageNodes[i]->string.setValue("Feedback Visibility OFF");
        }
    }
}

void QtCoinViewer::RecordSimtimeVideo(bool on)
{
    _RecordSetup(on,false);
}

void QtCoinViewer::RecordRealtimeVideo(bool on)
{
    _RecordSetup(on,true);
}

void QtCoinViewer::ToggleSimulation(bool on)
{
    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv = LockEnvironment(200000);
    if( !!lockenv ) {
        if( on ) {
            GetEnv()->StartSimulation(0.01f);
        }
        else {
            GetEnv()->StopSimulation();
        }
        lockenv.reset();
    }
    else {
        RAVELOG_WARN("failed to lock environment\n");
    }
}

void QtCoinViewer::_UpdateToggleSimulation()
{
    if( !!_pToggleSimulation ) {
        _pToggleSimulation->setChecked(GetEnv()->IsSimulationRunning());
    }
    if( !!_pToggleSelfCollision ) {
        PhysicsEngineBasePtr p = GetEnv()->GetPhysicsEngine();
        if( !!p ) {
            _pToggleSelfCollision->setChecked(!!(p->GetPhysicsOptions()&PEO_SelfCollisions));
        }
    }
}

void QtCoinViewer::_RecordSetup(bool bOn, bool bRealtimeVideo)
{
    if( !bOn ) {
        if( !!_pvideorecorder ) {
            SoDB::enableRealTimeSensor(true);
            SoSceneManager::enableRealTimeUpdate(true);
            _pvideorecorder.reset();
        }
        return;
    }

#if QT_VERSION >= 0x040000 // check for qt4
    if( bOn ) {
        _pvideorecorder = RaveCreateModule(GetEnv(),"viewerrecorder");
        if( !!_pvideorecorder ) {
#ifdef _WIN32
            QString s = QFileDialog::getSaveFileName( this, "Choose video filename", NULL, "AVI Files (*.avi)");
#else
            QString s = QFileDialog::getSaveFileName( this, "Choose video filename", NULL, "Video Files (*.*)");
#endif
            if( s.length() == 0 ) {
                return;
            }

            stringstream sout, sin;
            sin << "Start " << _nRenderWidth << " " << _nRenderHeight << " " << VIDEO_FRAMERATE << " codec " << _videocodec << " ";
            if( bRealtimeVideo ) {
                sin << "timing realtime ";
            }
            else {
                sin << "timing simtime ";
            }
            sin << " viewer " << GetName() << endl << " filename " << s.toAscii().data() << endl;
            if( !_pvideorecorder->SendCommand(sout,sin) ) {
                _pvideorecorder.reset();
                RAVELOG_DEBUG("video recording failed");
                return;
            }

            SoDB::enableRealTimeSensor(false);
            SoSceneManager::enableRealTimeUpdate(false);
        }
    }
#endif
}

bool QtCoinViewer::_GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& _t, const SensorBase::CameraIntrinsics& intrinsics)
{
    if( !_bCanRenderOffscreen ) {
        RAVELOG_WARN("cannot render offscreen\n");
        return false;
    }

    // have to flip Z axis
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    RaveTransform<float> t = _t * trot;

    SoSFVec3f position = GetCamera()->position;
    SoSFRotation orientation = GetCamera()->orientation;
    SoSFFloat aspectRatio = GetCamera()->aspectRatio;
    SoSFFloat heightAngle = GetCamera()->heightAngle;
    SoSFFloat nearDistance = GetCamera()->nearDistance;
    SoSFFloat farDistance = GetCamera()->farDistance;

    SoOffscreenRenderer* ivOffscreen = &_ivOffscreen;
    SbViewportRegion vpr(width, height);
    vpr.setViewport(SbVec2f(intrinsics.cx/(float)(width)-0.5f, 0.5f-intrinsics.cy/(float)(height)), SbVec2f(1,1));
    ivOffscreen->setViewportRegion(vpr);

    GetCamera()->position.setValue(t.trans.x, t.trans.y, t.trans.z);
    GetCamera()->orientation.setValue(t.rot.y, t.rot.z, t.rot.w, t.rot.x);
    GetCamera()->aspectRatio = (intrinsics.fy/(float)height) / (intrinsics.fx/(float)width);
    GetCamera()->heightAngle = 2.0f*atanf(0.5f*height/intrinsics.fy);
    GetCamera()->nearDistance = intrinsics.focal_length;
    GetCamera()->farDistance = intrinsics.focal_length*50000; // control the precision
    GetCamera()->viewportMapping = SoCamera::LEAVE_ALONE;

    _pFigureRoot->ref();
    bool bRenderFiguresInCamera = *(bool*)&_bRenderFiguresInCamera; // will this be optimized by compiler?
    if( !bRenderFiguresInCamera ) {
        _ivRoot->removeChild(_pFigureRoot);
    }
    bool bSuccess = ivOffscreen->render(_pviewer->getSceneManager()->getSceneGraph());
    if( !bRenderFiguresInCamera ) {
        _ivRoot->addChild(_pFigureRoot);
    }
    _pFigureRoot->unref();

    if( bSuccess ) {
        // vertically flip since we want upper left corner to correspond to (0,0)
        memory.resize(width*height*3);
        for(int i = 0; i < height; ++i) {
            memcpy(&memory[i*width*3], ivOffscreen->getBuffer()+(height-i-1)*width*3, width*3);
        }
    }
    else {
        RAVELOG_WARN("offscreen renderer failed (check video driver), disabling\n");
        _bCanRenderOffscreen = false; // need this or ivOffscreen.render will freeze next time
    }

    GetCamera()->position = position;
    GetCamera()->orientation = orientation;
    GetCamera()->aspectRatio = aspectRatio;
    GetCamera()->heightAngle = heightAngle;
    GetCamera()->nearDistance = nearDistance;
    GetCamera()->farDistance = farDistance;
    GetCamera()->viewportMapping = SoCamera::ADJUST_CAMERA;
    return bSuccess;
}

bool QtCoinViewer::_WriteCameraImage(int width, int height, const RaveTransform<float>& _t, const SensorBase::CameraIntrinsics& intrinsics, const std::string& filename, const std::string& extension)
{
    if( !_bCanRenderOffscreen ) {
        return false;
    }
    // have to flip Z axis
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    RaveTransform<float> t = _t * trot;

    SoSFVec3f position = GetCamera()->position;
    SoSFRotation orientation = GetCamera()->orientation;
    SoSFFloat aspectRatio = GetCamera()->aspectRatio;
    SoSFFloat heightAngle = GetCamera()->heightAngle;
    SoSFFloat nearDistance = GetCamera()->nearDistance;
    SoSFFloat farDistance = GetCamera()->farDistance;

    SbViewportRegion vpr(width, height);
    vpr.setViewport(SbVec2f(intrinsics.cx/(float)(width)-0.5f, 0.5f-intrinsics.cy/(float)(height)), SbVec2f(1,1));
    _ivOffscreen.setViewportRegion(vpr);

    GetCamera()->position.setValue(t.trans.x, t.trans.y, t.trans.z);
    GetCamera()->orientation.setValue(t.rot.y, t.rot.z, t.rot.w, t.rot.x);
    GetCamera()->aspectRatio = (intrinsics.fy/(float)height) / (intrinsics.fx/(float)width);
    GetCamera()->heightAngle = 2.0f*atanf(0.5f*height/intrinsics.fy);
    GetCamera()->nearDistance = intrinsics.focal_length;
    GetCamera()->farDistance = intrinsics.focal_length*10000; // control the precision
    GetCamera()->viewportMapping = SoCamera::LEAVE_ALONE;

    _pFigureRoot->ref();
    _ivRoot->removeChild(_pFigureRoot);

    bool bSuccess = true;
    if( !_ivOffscreen.render(_pviewer->getSceneManager()->getSceneGraph()) ) {
        RAVELOG_WARN("offscreen renderer failed (check video driver), disabling\n");
        _bCanRenderOffscreen = false;
        bSuccess = false;
    }
    else {
        if( !_ivOffscreen.isWriteSupported(extension.c_str()) ) {
            RAVELOG_WARN("file type %s not supported, supported filetypes are\n", extension.c_str());
            stringstream ss;

            for(int i = 0; i < _ivOffscreen.getNumWriteFiletypes(); ++i) {
                SbPList extlist;
                SbString fullname, description;
                _ivOffscreen.getWriteFiletypeInfo(i, extlist, fullname, description);
                ss << fullname.getString() << ": " << description.getString() << " (extensions: ";
                for (int j=0; j < extlist.getLength(); j++) {
                    ss << (const char*) extlist[j] << ", ";
                }
                ss << ")" << endl;
            }

            RAVELOG_INFO(ss.str().c_str());
            bSuccess = false;
        }
        else {
            bSuccess = _ivOffscreen.writeToFile(SbString(filename.c_str()), SbString(extension.c_str()));
        }
    }

    _ivRoot->addChild(_pFigureRoot);
    _pFigureRoot->unref();

    GetCamera()->position = position;
    GetCamera()->orientation = orientation;
    GetCamera()->aspectRatio = aspectRatio;
    GetCamera()->heightAngle = heightAngle;
    GetCamera()->nearDistance = nearDistance;
    GetCamera()->farDistance = farDistance;
    GetCamera()->viewportMapping = SoCamera::ADJUST_CAMERA;
    return bSuccess;
}

uint8_t* QtCoinViewer::_GetVideoFrame()
{
    if( !_bCanRenderOffscreen ) {
        return NULL;
    }
    _ivOffscreen.setViewportRegion(SbViewportRegion(_nRenderWidth, _nRenderHeight));
    _ivOffscreen.render(_pviewer->getSceneManager()->getSceneGraph());

    if( _ivOffscreen.getBuffer() == NULL ) {
        RAVELOG_WARN("offset buffer null, disabling\n");
        _bCanRenderOffscreen = false;
        return NULL;
    }

    // flip R and B
    for(unsigned int i = 0; i < _nRenderHeight; ++i) {
        for(unsigned int j = 0; j < _nRenderWidth; ++j) {
            unsigned char* ptr = _ivOffscreen.getBuffer() + 3 * (i * _nRenderWidth + j);
            swap(ptr[0], ptr[2]);
        }
    }

    return (uint8_t*)_ivOffscreen.getBuffer();
}

void QtCoinViewer::CollisionCheckerChanged(QAction* pact)
{
    if( pact->text() == "[None]" ) {
        GetEnv()->SetCollisionChecker(CollisionCheckerBasePtr());
    }
    else {
        CollisionCheckerBasePtr p = RaveCreateCollisionChecker(GetEnv(),pact->text().toUtf8().constData());
        if( !!p ) {
            GetEnv()->SetCollisionChecker(p);
        }
        else {
            _UpdateCollisionChecker();
        }
    }
}

void QtCoinViewer::_UpdateCollisionChecker()
{
    if( !!_pSelectedCollisionChecker ) {
        CollisionCheckerBasePtr p = GetEnv()->GetCollisionChecker();
        if( !!p ) {
            for(int i = 0; i < _pSelectedCollisionChecker->actions().size(); ++i) {
                QAction* pact = _pSelectedCollisionChecker->actions().at(i);
                std::string selectedname = pact->text().toUtf8().constData();
                if( selectedname == p->GetXMLId()) {
                    pact->setChecked(true);
                    return;
                }
            }
            //RAVELOG_VERBOSE(str(boost::format("cannot find collision checker menu item %s\n")%p->GetXMLId()));
        }

        // set to default
        _pSelectedCollisionChecker->actions().at(0)->setChecked(true);
    }
}

void QtCoinViewer::PhysicsEngineChanged(QAction* pact)
{

    if( pact->text() == "[None]" ) {
        GetEnv()->SetPhysicsEngine(PhysicsEngineBasePtr());
    }
    else {
        PhysicsEngineBasePtr p = RaveCreatePhysicsEngine(GetEnv(),pact->text().toUtf8().constData());
        if( !!p ) {
            GetEnv()->SetPhysicsEngine(p);
        }
        else {
            _UpdatePhysicsEngine();
        }
    }
}

void QtCoinViewer::_UpdatePhysicsEngine()
{
    PhysicsEngineBasePtr p;
    if( 0&&!!_pSelectedPhysicsEngine ) {
        p = GetEnv()->GetPhysicsEngine();
        if( !!p ) {
            for(int i = 0; i < _pSelectedPhysicsEngine->actions().size(); ++i) {
                QAction* pact = _pSelectedPhysicsEngine->actions().at(i);
                std::string selectedname = pact->text().toUtf8().constData();
                if( selectedname == p->GetXMLId() ) {
                    pact->setChecked(true);
                    return;
                }
            }
            RAVELOG_VERBOSE(str(boost::format("cannot find physics engine menu item %s\n")%p->GetXMLId()));
        }

        // set to default
        _pSelectedPhysicsEngine->actions().at(0)->setChecked(true);
    }
}

void QtCoinViewer::UpdateInterfaces()
{
    list<ModuleBasePtr> listProblems;
    vector<KinBodyPtr> vbodies;
    GetEnv()->GetLoadedProblems(listProblems);
    GetEnv()->GetBodies(vbodies);

    //    FOREACH(it,listInterfaces) {
    //        pact = new QAction(tr((*it)->GetXMLId().c_str()), this);
    //        pact->setCheckable(true);
    //        pact->setChecked(_bSelfCollision);
    //        connect(pact, SIGNAL(triggered(bool)), this, SLOT(DynamicSelfCollision(bool)));
    //        psubmenu->addAction(pact);
    //    }
}

void QtCoinViewer::InterfaceSendCommand(QAction* pact)
{
}

void QtCoinViewer::DynamicSelfCollision(bool on)
{
    PhysicsEngineBasePtr p = GetEnv()->GetPhysicsEngine();
    if( !!p ) {
        int opts = p->GetPhysicsOptions();
        if( on ) {
            opts |= PEO_SelfCollisions;
        }
        else {
            opts &= ~PEO_SelfCollisions;
        }
        p->SetPhysicsOptions(opts);
    }
}

void QtCoinViewer::DynamicGravity()
{
    GetEnv()->GetPhysicsEngine()->SetGravity(Vector(0, 0, -9.8f));
}

void QtCoinViewer::About()
{
    QMessageBox::information(this,"About OpenRAVE...",
                             "OpenRAVE is a open-source robotics planning and simulation environment\n"
                             "Lead Developer: Rosen Diankov\n"
                             "License: Lesser General Public License v3.0 (LGPLv3)\n");

}

QtCoinViewer::EnvMessage::EnvMessage(QtCoinViewerPtr pviewer, void** ppreturn, bool bWaitForMutex)
    : _pviewer(pviewer), _ppreturn(ppreturn)
{
    // get a mutex
    if( bWaitForMutex ) {
        _plock.reset(new boost::mutex::scoped_lock(_mutex));
    }
}

QtCoinViewer::EnvMessage::~EnvMessage()
{
    _plock.reset();
}

/// execute the command in the caller
void QtCoinViewer::EnvMessage::callerexecute(bool bGuiThread)
{
    bool bWaitForMutex = !!_plock;

    // QThread::currentThread is buggy and gets into an infinite loop if the Gui thread is calling QEventLoop::processEvents
    //bool bGuiThread = QThread::currentThread() == QCoreApplication::instance()->thread();
    if( bGuiThread ) {
        viewerexecute();
    }
    else {
        {
            QtCoinViewerPtr pviewer = _pviewer.lock();
            if( !!pviewer ) {
                boost::mutex::scoped_lock lock(pviewer->_mutexMessages);
                pviewer->_listMessages.push_back(shared_from_this());
            }
        }

        if( bWaitForMutex ) {
            boost::mutex::scoped_lock lock(_mutex);
        }
    }
}

/// execute the command in the viewer
void QtCoinViewer::EnvMessage::viewerexecute()
{
    _plock.reset();
}

bool QtCoinViewer::_SetFiguresInCamera(ostream& sout, istream& sinput)
{
    sinput >> _bRenderFiguresInCamera;
    return !!sinput;
}

bool QtCoinViewer::_CommandResize(ostream& sout, istream& sinput)
{
    sinput >> _nRenderWidth;
    sinput >> _nRenderHeight;
    if( !!sinput ) {
        return true;
    }
    return false;
}

bool QtCoinViewer::_SetFeedbackVisibility(ostream& sout, istream& sinput)
{
    sinput >> _bDisplayFeedBack;
    if( !!sinput ) {
        ViewToggleFeedBack(_bDisplayFeedBack);
        return true;
    }
    return false;
}

UserDataPtr QtCoinViewer::RegisterItemSelectionCallback(const ItemSelectionCallbackFn& fncallback)
{
    ItemSelectionCallbackDataPtr pdata(new ItemSelectionCallbackData(fncallback,shared_viewer()));
    pdata->_iterator = _listRegisteredItemSelectionCallbacks.insert(_listRegisteredItemSelectionCallbacks.end(),pdata);
    return pdata;
}

UserDataPtr QtCoinViewer::RegisterViewerImageCallback(const ViewerImageCallbackFn& fncallback)
{
    ViewerImageCallbackDataPtr pdata(new ViewerImageCallbackData(fncallback,shared_viewer()));
    pdata->_iterator = _listRegisteredViewerImageCallbacks.insert(_listRegisteredViewerImageCallbacks.end(),pdata);
    return pdata;
}

UserDataPtr QtCoinViewer::RegisterViewerThreadCallback(const ViewerThreadCallbackFn& fncallback)
{
    ViewerThreadCallbackDataPtr pdata(new ViewerThreadCallbackData(fncallback,shared_viewer()));
    pdata->_iterator = _listRegisteredViewerThreadCallbacks.insert(_listRegisteredViewerThreadCallbacks.end(),pdata);
    return pdata;
}


bool QtCoinViewer::_SaveBodyLinkToVRMLCommand(ostream& sout, istream& sinput)
{
    string bodyname, filename;
    int linkindex=-1;
    sinput >> bodyname >> linkindex;
    if (!getline(sinput, filename) ) {
        RAVELOG_WARN("failed to get filename\n");
        return false;
    }
    boost::trim(filename);

    KinBodyPtr pbody = GetEnv()->GetKinBody(bodyname);
    if( !pbody ) {
        RAVELOG_WARN_FORMAT("could not find body %s", bodyname);
        return false;
    }

    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    boost::mutex::scoped_lock lock2(g_mutexsoqt);

    if( _mapbodies.find(pbody) == _mapbodies.end() ) {
        RAVELOG_WARN_FORMAT("couldn't find body %s in viewer list", bodyname);
        return false;
    }

    KinBodyItemPtr pitem = _mapbodies[pbody];
    SoNode* psep;
    if( linkindex >= 0 && linkindex < (int)pbody->GetLinks().size() ) {
        psep = pitem->GetIvLink(linkindex);
    }
    else {
        psep = pitem->GetIvGeom();
    }

    psep->ref();
    SoToVRML2Action tovrml2;
    tovrml2.apply(psep);
    SoVRMLGroup *newroot = tovrml2.getVRML2SceneGraph();
    newroot->ref();
    psep->unref();

    SoOutput out;
    out.openFile(filename.c_str());
    out.setHeaderString("#VRML V2.0 utf8");
    SoWriteAction wa(&out);
    wa.apply(newroot);
    out.closeFile();
    newroot->unref();
    RAVELOG_VERBOSE_FORMAT("saved body %s to file %s", pbody->GetName()%filename);
    return true;
}

class SetNearPlaneMessage : public QtCoinViewer::EnvMessage
{
public:
    SetNearPlaneMessage(QtCoinViewerPtr pviewer, void** ppreturn, bool fnearplane)
        : EnvMessage(pviewer, ppreturn, false), _fnearplane(fnearplane) {
    }

    virtual void viewerexecute() {
        QtCoinViewerPtr pviewer = _pviewer.lock();
        if( !pviewer ) {
            return;
        }
        pviewer->_SetNearPlane(_fnearplane);
        EnvMessage::viewerexecute();
    }

private:
    dReal _fnearplane;
};

bool QtCoinViewer::_SetNearPlaneCommand(ostream& sout, istream& sinput)
{
    dReal nearplane=0.01f;
    sinput >> nearplane;
    EnvMessagePtr pmsg(new SetNearPlaneMessage(shared_viewer(), (void**)NULL, nearplane));
    pmsg->callerexecute(false);
    return true;
}

void QtCoinViewer::_SetNearPlane(dReal nearplane)
{
    _pviewer->setAutoClippingStrategy(SoQtViewer::CONSTANT_NEAR_PLANE, nearplane);
}

/*ScreenRendererWidget::ScreenRendererWidget()
   {

    std::list<EnvironmentBasePtr> listenvironments;
    RaveGetEnvironments(listenvironments);
    if( listenvironments.size() > 0 ) {
        _penv = listenvironments.front();
        _openraveviewer = boost::dynamic_pointer_cast<QtCoinViewer>(_penv->GetViewer());
        if( !!_openraveviewer ) {
            QTimer *timer = new QTimer(this);
            connect(timer, SIGNAL(timeout()), this, SLOT(Animate()));
            timer->start(50); // ms
        }
    }
   }

   void ScreenRendererWidget::Animate()
   {
    update();
   }

   void ScreenRendererWidget::paintEvent(QPaintEvent *event)
   {
    if( !!_openraveviewer ) {
        QPainter painter(this);
        int width=320, height=240;
        SensorBase::CameraIntrinsics intr;
        intr.fx = 320;
        intr.fy = 320;
        intr.cx = 160;
        intr.cy = 120;
        intr.focal_length = _openraveviewer->GetCameraIntrinsics().focal_length;
        bool bsuccess = _openraveviewer->_GetCameraImage(_memory, width, height, _openraveviewer->GetCameraTransform(), intr);
        if( bsuccess ) {
            //emit image_ready(QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888));
            QImage image = QImage( width, height, QImage::Format_RGB888 );
            for (int i = 0; i < height; ++i) {
                memcpy(image.scanLine(i), &_memory[0] + i * width * 3, width * 3);
            }
            painter.drawImage(QRectF(0,0,width,height), image);
        }
        else {
            RAVELOG_WARN("openrave viewer failed to _GetCameraImage\n");
        }
    }
   }

   QtCoinViewerProxy::QtCoinViewerProxy(QGraphicsItem* parent) : QGraphicsProxyWidget(parent)
   {
    setWidget(new ScreenRendererWidget());
   }

   Q_EXPORT_PLUGIN2(qtcoinrave, QOpenRAVEWidgetsPlugin);
 */
