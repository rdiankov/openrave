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
#include "qtosgviewer.h"

#include <boost/lexical_cast.hpp>
#include <iostream>

#include <QSurfaceFormat>
#include <osg/ArgumentParser>
#include "osgviewerwidget.h"

namespace qtosgrave {

#define ITEM_DELETER boost::bind(DeleteItemCallbackSafe,weak_viewer(),_1)

void DeleteItemCallbackSafe(QtOSGViewerWeakPtr wpt, Item* pItem)
{
    QtOSGViewerPtr pviewer = wpt.lock();
    if( !!pviewer ) {
        pviewer->_DeleteItemCallback(pItem);
    }
}

class ItemSelectionCallbackData : public UserData
{
public:
    ItemSelectionCallbackData(const ViewerBase::ItemSelectionCallbackFn& callback, boost::shared_ptr<QtOSGViewer> pviewer) : _callback(callback), _pweakviewer(pviewer) {
    }
    virtual ~ItemSelectionCallbackData() {
        boost::shared_ptr<QtOSGViewer> pviewer = _pweakviewer.lock();
        if( !!pviewer ) {
            boost::mutex::scoped_lock lock(pviewer->_mutexCallbacks);
            pviewer->_listRegisteredItemSelectionCallbacks.erase(_iterator);
        }
    }

    list<UserDataWeakPtr>::iterator _iterator;
    ViewerBase::ItemSelectionCallbackFn _callback;
protected:
    boost::weak_ptr<QtOSGViewer> _pweakviewer;
};
typedef boost::shared_ptr<ItemSelectionCallbackData> ItemSelectionCallbackDataPtr;

class ViewerThreadCallbackData : public UserData
{
public:
    ViewerThreadCallbackData(const ViewerBase::ViewerThreadCallbackFn& callback, boost::shared_ptr<QtOSGViewer> pviewer) : _callback(callback), _pweakviewer(pviewer) {
    }
    virtual ~ViewerThreadCallbackData() {
        boost::shared_ptr<QtOSGViewer> pviewer = _pweakviewer.lock();
        if( !!pviewer ) {
            boost::mutex::scoped_lock lock(pviewer->_mutexCallbacks);
            pviewer->_listRegisteredViewerThreadCallbacks.erase(_iterator);
        }
    }

    list<UserDataWeakPtr>::iterator _iterator;
    ViewerBase::ViewerThreadCallbackFn _callback;
protected:
    boost::weak_ptr<QtOSGViewer> _pweakviewer;
};
typedef boost::shared_ptr<ViewerThreadCallbackData> ViewerThreadCallbackDataPtr;

QtOSGViewer::QtOSGViewer(EnvironmentBasePtr penv, std::istream& sinput) : QMainWindow(NULL, Qt::Window), ViewerBase(penv)
{
    //
    // initialize member variables
    //

    _qobjectTree = NULL;
    _qdetailsTree = NULL;
    _focalDistance = 0.0;
    _fTrackAngleToUp = 0.3;
    _pointerTypeGroup = NULL;
    _posgWidget = NULL;
    _nQuitMainLoop = 0;
    _qactPerspectiveView = NULL;
    _qactChangeViewtoXY = _qactChangeViewtoYZ = _qactChangeViewtoXZ = NULL;
    //osg::setNotifyLevel(osg::DEBUG_FP);
    _userdatakey = std::string("qtosg") + boost::lexical_cast<std::string>(this);
    debugLevelInfoAct = NULL;
    debugLevelDebugAct = NULL;
    debugLevelVerboseAct = NULL;

    //
    // read viewer parameters
    //

    int qtosgbuild = 0; // not used
    bool bCreateStatusBar = true, bCreateMenu = true;
    int nAlwaysOnTopFlag = 0; // 1 - add on top flag (keep others), 2 - add on top flag (remove others)
    sinput >> qtosgbuild >> bCreateStatusBar >> bCreateMenu >> nAlwaysOnTopFlag;

    //
    // figure out window flags
    //

    if (nAlwaysOnTopFlag != 0) {
        Qt::WindowFlags flags = Qt::Window | Qt::FramelessWindowHint | Qt::CustomizeWindowHint | Qt::WindowStaysOnTopHint;
        if (nAlwaysOnTopFlag == 1) {
            flags |= windowFlags();
        }
        setWindowFlags(flags);
    }

    //
    // figure out window title
    //

    std::string name = str(boost::format("OpenRAVE %s")%OPENRAVE_VERSION_STRING);
    if( (OPENRAVE_VERSION_MINOR%2) || (OPENRAVE_VERSION_PATCH%2) ) {
        name += " (Development Version)";
    }
    else {
        name += " (Stable Release)";
    }
    setWindowTitle(name.c_str());

    if(bCreateStatusBar) {
        statusBar()->showMessage(tr("Status Bar"));
    }

    __description = ":Interface Author: Rosen Diankov, Gustavo Puche\n\nProvides a viewer based on OpenSceneGraph library. Currently tested with v3.4. Usage:\n\n\
  - ESC to toggle between camera, selection, and joint modes.\n\
  - Left Click to rotate (or select) object.\n\
  - Middle (or Shift+Left, Left+Right) Click to pan.\n\
  - Right Click to zoom.\n\
  - (Shift+)ArrowKey for moving via keyboard.\n\
  - In selection mode, Ctrl+Left Click to move object.\n\
  - Press 's' and Left click on screen to center camera.\n\
";

    RegisterCommand("SetFiguresInCamera",boost::bind(&QtOSGViewer::_SetFiguresInCamera, this, _1, _2),
                    "Accepts 0/1 value that decides whether to render the figure plots in the camera image through GetCameraImage");
    RegisterCommand("SetItemVisualization",boost::bind(&QtOSGViewer::_SetItemVisualizationCommand, this, _1, _2),
                    "sets the visualization mode of a kinbody/render item in the viewer");
    RegisterCommand("ShowWorldAxes",boost::bind(&QtOSGViewer::_ShowWorldAxesCommand, this, _1, _2),
                    "Accepts 0/1 value that decides whether to render the cross hairs");
    RegisterCommand("SetNearPlane", boost::bind(&QtOSGViewer::_SetNearPlaneCommand, this, _1, _2),
                    "Sets the near plane for rendering of the image. Useful when tweaking rendering units");
    RegisterCommand("SetTextureCubeMap", boost::bind(&QtOSGViewer::_SetTextureCubeMap, this, _1, _2),
                    "Sets the skybox with cubemap");
    RegisterCommand("TrackLink", boost::bind(&QtOSGViewer::_TrackLinkCommand, this, _1, _2),
                    "camera tracks the link maintaining a specific relative transform: robotname, manipname, focalDistance");
    RegisterCommand("TrackManipulator", boost::bind(&QtOSGViewer::_TrackManipulatorCommand, this, _1, _2),
                    "camera tracks the manipulator maintaining a specific relative transform: robotname, manipname, focalDistance");
    RegisterCommand("SetTrackingAngleToUp", boost::bind(&QtOSGViewer::_SetTrackingAngleToUpCommand, this, _1, _2),
                    "sets a new angle to up");
    RegisterCommand("StartViewerLoop", boost::bind(&QtOSGViewer::_StartViewerLoopCommand, this, _1, _2),
                    "starts the viewer sync loop and shows the viewer. expects someone else will call the qapplication exec fn");
    RegisterCommand("SetProjectionMode", boost::bind(&QtOSGViewer::_SetProjectionModeCommand, this, _1, _2),
                    "sets the viewer projection mode, perspective or orthogonal");
    RegisterCommand("Zoom", boost::bind(&QtOSGViewer::_ZoomCommand, this, _1, _2),
                    "Set the zooming factor of the view");
    _bLockEnvironment = true;
    _InitGUI(bCreateStatusBar, bCreateMenu);
    _bUpdateEnvironment = true;
    _bExternalLoop = false;
}

QtOSGViewer::~QtOSGViewer()
{
    RAVELOG_DEBUG("destroying qtosg viewer\n");
    // _notifyGUIFunctionComplete can still be waiting. Code will crash when
    // the mutex is destroyed in that state. SetEnvironmentSync will release
    // _notifyGUIFunctionComplete
    SetEnvironmentSync(false);
    {
        boost::mutex::scoped_lock lock(_mutexGUIFunctions);

        list<GUIThreadFunctionPtr>::iterator itmsg;
        FORIT(itmsg, _listGUIFunctions) {
            try {
                (*itmsg)->Call();
            }
            catch(const boost::bad_weak_ptr& ex) {
                // most likely viewer
            }
            catch(const std::exception& ex2) {
                RAVELOG_WARN_FORMAT("unexpected exception in gui function: %s", ex2.what());
            }
        }
        _listGUIFunctions.clear();
    }

    _condUpdateModels.notify_all();
}

void QtOSGViewer::_InitGUI(bool bCreateStatusBar, bool bCreateMenu)
{
    osg::ArgumentParser arguments(0, NULL);

    if( !QApplication::instance() ) {
        RAVELOG_WARN("no app instance to attach close\n");
    }
    else {
        connect(QApplication::instance(), SIGNAL(aboutToQuit()), this, SLOT(_ProcessApplicationQuit()));
    }

    QSurfaceFormat sf;
    sf.setSamples(8);
    QSurfaceFormat::setDefaultFormat(sf);

    _posgWidget = new QOSGViewerWidget(GetEnv(), _userdatakey, boost::bind(&QtOSGViewer::_HandleOSGKeyDown, this, _1), GetEnv()->GetUnit().second, this);

    setCentralWidget(_posgWidget);

    _RepaintWidgets();
    _posgWidget->SetFacesMode(false);
    _posgWidget->SetLight(false);

    _qtree = new QTreeView;

    _CreateActions();

    if( bCreateStatusBar ) {
        _CreateStatusBar();
    }

    if ( bCreateMenu ) {
        _CreateMenus();
        _CreateToolsBar();
        _CreateDockWidgets();
    }

    if (!bCreateMenu && !bCreateMenu) {
        _CreateControlButtons();
    }

    resize(1024, 768);

    // toggle switches
    _bDisplayGrid = false;
    _bDisplayIK = false;
    _bDisplayFPS = false;
    _bJointHilit = true;
    _bDynamicReplan = false;
    _bVelPredict = true;
    _bDynSim = false;
    _bControl = true;
    _bGravity = true;
    _bSensing = false;
    _bMemory = true;
    _bHardwarePlan = false;
    _bShareBitmap = true;
    _bManipTracking = false;
    _bAntialiasing = false;
    _viewGeometryMode = VG_RenderOnly;
    _bRenderFiguresInCamera = true;
    _bDisplayFeedBack = true;
}

bool QtOSGViewer::_HandleOSGKeyDown(int key)
{
    switch(key) {
    case osgGA::GUIEventAdapter::KEY_Escape:
        if( !!_pointerTypeGroup ) {
            // toggle
            int newcheckedid = _pointerTypeGroup->checkedId()-1;
            int minchecked = -4, maxchecked = -2;
            if( newcheckedid < minchecked || newcheckedid > maxchecked ) {
                newcheckedid = maxchecked;
            }
            _pointerTypeGroup->button(newcheckedid)->setChecked(true);
            _ProcessPointerGroupClicked(newcheckedid);
            return true;
//            //RAVELOG_INFO_FORMAT("checked id %d", _pointerTypeGroup->checkedId());
//            //if( !!_pointerTypeGroup->checkedButton() ) {
//                //_pointerTypeGroup->checkedButton()->setChecked(false);
//                _PostToGUIThread(boost::bind(&QAbstractButton::setChecked, _pointerTypeGroup->checkedButton(), false));
//            }
//            else {
//                // check one?
//            }
        }
        break;
    }

    return false;
}

void QtOSGViewer::customEvent(QEvent * e)
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

bool QtOSGViewer::_ForceUpdatePublishedBodies()
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

void QtOSGViewer::_CreateActions()
{
    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcut(tr("Ctrl+Q"));
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    loadAct = new QAction(QIcon(":/images/open.png"),tr("L&oad..."), this);
    loadAct->setShortcut(tr("Ctrl+L"));
    connect(loadAct, SIGNAL(triggered()), this, SLOT(LoadEnvironment()));

    importAct = new QAction(QIcon(":/images/Import.png"),tr("I&mport..."), this);
    importAct->setShortcut(tr("Ctrl+I"));
    connect(importAct, SIGNAL(triggered()), this, SLOT(ImportEnvironment()));

    saveAct = new QAction(QIcon(":/images/save.png"),tr("S&ave..."), this);
    saveAct->setShortcut(tr("Ctrl+S"));
    connect(saveAct, SIGNAL(triggered()), this, SLOT(SaveEnvironment()));

    viewCamAct = new QAction(tr("View Camera Params"), this);

    viewColAct = new QAction(tr("View Collision Word"), this);

    pubilshAct = new QAction(tr("Pubilsh Bodies Anytimes"), this);

    debugLevelInfoAct = new QAction(tr("Debug Level (Info)"), this);
    connect(debugLevelInfoAct, SIGNAL(triggered()), this, SLOT(_SetDebugLevelInfo()));

    debugLevelDebugAct = new QAction(tr("Debug Level (Debug)"), this);
    connect(debugLevelDebugAct, SIGNAL(triggered()), this, SLOT(_SetDebugLevelDebug()));

    debugLevelVerboseAct = new QAction(tr("Debug Level (Verbose)"), this);
    connect(debugLevelVerboseAct, SIGNAL(triggered()), this, SLOT(_SetDebugLevelVerbose()));

    showAct = new QAction(tr("Show Framerate"), this);

    playAct = new QAction(QIcon(":/images/play.png"),tr("Play"), this);

    stopAct = new QAction(QIcon(":/images/stop.png"),tr("Stop"), this);

    recordAct = new QAction(tr("Record V&ideo"), this);

    odeAct = new QAction(tr("ODE Dynamic Simulation"), this);
    odeAct->setCheckable(true);

    selfAct = new QAction(tr("Self Collision"), this);
    selfAct->setCheckable(true);

    applyAct = new QAction(tr("Apply Gravity"), this);
    applyAct->setCheckable(true);
    applyAct->setChecked(true);

    aboutAct = new QAction(tr("About"), this);
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(_ProcessAboutDialog()));

    pauseAct = new QAction(QIcon(":/images/pause.png"),tr("Pause"), this);

    puntAct = new QAction(QIcon(":/images/no_edit.png"),tr("Pointer"), this);

//    AxesAct = new QAction(QIcon(":/images/axes.png"),tr("Axes"), this);
//    AxesAct->setCheckable(true);
//    connect(AxesAct, SIGNAL(triggered()), this, SLOT(axes()));

    houseAct = new QAction(QIcon(":/images/house.png"),tr("Home"), this);
    connect(houseAct, SIGNAL(triggered()),this,SLOT(ResetViewToHome()));

    _qactChangeViewtoXY = new QAction(QIcon(":/images/xyplane.png"),tr("Show XY"), this);
    connect(_qactChangeViewtoXY, SIGNAL(triggered()),this,SLOT(_ChangeViewToXY()));

    _qactChangeViewtoXZ = new QAction(QIcon(":/images/xzplane.png"),tr("Show XZ"), this);
    connect(_qactChangeViewtoXZ, SIGNAL(triggered()),this,SLOT(_ChangeViewToXZ()));

    _qactChangeViewtoYZ = new QAction(QIcon(":/images/yzplane.png"),tr("Show YZ"), this);
    connect(_qactChangeViewtoYZ, SIGNAL(triggered()),this,SLOT(_ChangeViewToYZ()));

//    smoothAct = new QAction(QIcon(":/images/smooth.png"),tr("Smooth"), this);
//    connect(smoothAct, SIGNAL(triggered()), this, SLOT(polygonMode()));
//
//    flatAct = new QAction(QIcon(":/images/flat.png"),tr("Flat"), this);
//    connect(flatAct, SIGNAL(triggered()), this, SLOT(polygonMode()));

    _qactPerspectiveView = new QAction(QIcon(":/images/perspective.png"),tr("View Model"), this);
    _qactPerspectiveView->setCheckable(true);
    _qactPerspectiveView->setChecked(false);
    connect(_qactPerspectiveView, SIGNAL(triggered()), this, SLOT(_ProcessPerspectiveViewChange()));

//    lightAct = new QAction(QIcon(":/images/lightoff.png"),tr("Light"), this);
//    lightAct->setCheckable(true);
//    lightAct->setChecked(true);
//    connect(lightAct, SIGNAL(triggered()), this, SLOT(_ProcessLightChange()));

    wireAct = new QAction(QIcon(":/images/wire.png"),tr("Wire"), this);
    connect(wireAct, SIGNAL(triggered()), this, SLOT(polygonMode()));

//    facesAct = new QAction(QIcon(":/images/faces.png"),tr("Cull face"), this);
//    facesAct->setCheckable(true);
//    facesAct->setChecked(true);
//    connect(facesAct, SIGNAL(triggered()), this, SLOT(_ProcessFacesModeChange()));

    bboxAct = new QAction(QIcon(":/images/bbox.png"),tr("Polygon"), this);
    bboxAct->setCheckable(true);
    connect(bboxAct, SIGNAL(triggered()), this, SLOT(_ProcessBoundingBox()));

    connect(&_updateViewerTimer, SIGNAL(timeout()), this, SLOT(_UpdateViewerCallback()));
    _updateViewerTimer.start(1000/60); // ms
}

void QtOSGViewer::_UpdateViewerCallback()
{
    try {
        _UpdateEnvironment(1.0/60.0);
        //UpdateFromModel();

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
    }
    catch(const std::exception& ex) {
        RAVELOG_ERROR_FORMAT("got an exception in the viewer thread: %s", ex.what());
    }
}

void QtOSGViewer::_Reset()
{
    _Deselect();
    UpdateFromModel();
    _condUpdateModels.notify_all();

    FOREACH(itbody, _mapbodies) {
        BOOST_ASSERT( itbody->first->GetUserData(_userdatakey) == itbody->second );
        itbody->first->RemoveUserData(_userdatakey);
    }
    _mapbodies.clear();

    if (!!_qobjectTree) {
        _qobjectTree->clear();
    }


    {
        boost::mutex::scoped_lock lock(_mutexItems);
        FOREACH(it,_listRemoveItems) {
            delete *it;
        }
        _listRemoveItems.clear();
    }
}

void QtOSGViewer::_CreateMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(loadAct);
    fileMenu->addAction(importAct);
    fileMenu->addAction(saveAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    viewMenu = menuBar()->addMenu(tr("&View"));

    viewMenu->addSeparator();
    viewMenu->addAction(viewCamAct);

    viewMenu->addAction(viewColAct);
    viewMenu->addAction(pubilshAct);
    viewMenu->addAction(debugLevelInfoAct);
    viewMenu->addAction(debugLevelDebugAct);
    viewMenu->addAction(debugLevelVerboseAct);
    viewMenu->addAction(showAct);

//	animation = menuBar()->addMenu(tr("&Animation"));
//	animation->addAction(playAct);
//	animation->addAction(stopAct);
//
//	options = menuBar()->addMenu(tr("&Options"));
//	options->addAction(recordAct);
//
//	dynamics = menuBar()->addMenu(tr("D&ynamics"));
//	dynamics->addAction(odeAct);
//	dynamics->addAction(selfAct);
//	dynamics->addAction(applyAct);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
}

void QtOSGViewer::LoadEnvironment()
{
    QString s = QFileDialog::getOpenFileName( _posgWidget, "Load Environment", QString(), "Env Files (*.xml);;COLLADA Files (*.dae|*.zae)");
    if( s.length() == 0 ) {
        return;
    }

    _Reset();
    try {
        EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        GetEnv()->Reset();

        GetEnv()->Load(s.toLatin1().data());

        RAVELOG_INFO("\n---------Refresh--------\n");

        //  Refresh the screen.
        UpdateFromModel();

        RAVELOG_INFO("----> set home <----\n");

        //  Center object in window viewer
        _posgWidget->SetHome();
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("failed to load environment %s: %s", s.toLatin1().data()%ex.what());
    }

}

void QtOSGViewer::ImportEnvironment()
{
    QString s = QFileDialog::getOpenFileName( this, "Import Environment", NULL,
                                              "Env Files (*.xml);;COLLADA Files (*.dae)");

    if( s.length() == 0 ) {
        return;
    }
    try {
        EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        GetEnv()->Load(s.toLatin1().data());

        //  Refresh the screen.
        UpdateFromModel();
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("failed to import model %s: %s", s.toLatin1().data()%ex.what());
    }
}

void QtOSGViewer::SaveEnvironment()
{
    QString s = QFileDialog::getSaveFileName( this, "Save Environment", NULL, "Env Files (*.xml);;COLLADA Files (*.dae)");
    if( s.length() == 0 ) {
        return;
    }
    try {
        EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        GetEnv()->Save(s.toLatin1().data());
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("failed to save to file %s: %s", s.toLatin1().data()%ex.what());
    }
}

void QtOSGViewer::ResetViewToHome()
{
    _posgWidget->ResetViewToHome();
}

void QtOSGViewer::_ProcessPerspectiveViewChange()
{
    if( _qactPerspectiveView->isChecked() ) {
        _qactPerspectiveView->setIcon(QIcon(":/images/bbox.png"));
    }
    else {
        _qactPerspectiveView->setIcon(QIcon(":/images/perspective.png"));
    }

    _posgWidget->SetViewType(_qactPerspectiveView->isChecked());
}

void QtOSGViewer::_ProcessAboutDialog()
{
    QMessageBox msgBox;
    msgBox.setText("OpenRAVE qtosg plugin");
    msgBox.setInformativeText(__description.c_str());
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);
    msgBox.exec();
}

void QtOSGViewer::_SetDebugLevelInfo()
{
    RaveSetDebugLevel(Level_Info);
}

void QtOSGViewer::_SetDebugLevelDebug()
{
    RaveSetDebugLevel(Level_Debug);
}

void QtOSGViewer::_SetDebugLevelVerbose()
{
    RaveSetDebugLevel(Level_Verbose);
}

void QtOSGViewer::_ChangeViewToXY()
{
    _UpdateCameraTransform(0);
    osg::Vec3d center = _posgWidget->GetSceneRoot()->getBound().center();
    _Tcamera.rot = quatFromAxisAngle(RaveVector<float>(1,0,0), float(0));
    _Tcamera.trans.x = center.x();
    _Tcamera.trans.y = center.y();
    _Tcamera.trans.z = center.z()+_focalDistance;
    _SetCameraTransform();
}

void QtOSGViewer::_ChangeViewToXZ()
{
    _UpdateCameraTransform(0);
    osg::Vec3d center = _posgWidget->GetSceneRoot()->getBound().center();
    _Tcamera.rot = quatFromAxisAngle(RaveVector<float>(1,0,0), float(M_PI/2));
    _Tcamera.trans.x = center.x();
    _Tcamera.trans.y = center.y()-_focalDistance;
    _Tcamera.trans.z = center.z();
    _SetCameraTransform();

}

void QtOSGViewer::_ChangeViewToYZ()
{
    _UpdateCameraTransform(0);
    osg::Vec3d center = _posgWidget->GetSceneRoot()->getBound().center();
    _Tcamera.rot = quatMultiply(quatFromAxisAngle(RaveVector<float>(0,0,1), float(M_PI/2)), quatFromAxisAngle(RaveVector<float>(1,0,0), float(M_PI/2)));
    _Tcamera.trans.x = center.x()+_focalDistance;
    _Tcamera.trans.y = center.y();
    _Tcamera.trans.z = center.z();
    _SetCameraTransform();

}

//void QtOSGViewer::_ProcessLightChange()
//{
//    if (lightAct->isChecked()) {
//        lightAct->setIcon(QIcon(":/images/lightoff.png"));
//    }
//    else {
//        lightAct->setIcon(QIcon(":/images/lighton.png"));
//    }
//    _posgWidget->SetLight(!lightAct->isChecked());
//}

//void QtOSGViewer::_ProcessFacesModeChange()
//{
//    _posgWidget->SetFacesMode(!facesAct->isChecked());
//}

void QtOSGViewer::polygonMode()
{
    _posgWidget->SetPolygonMode(wireAct->isChecked() ? 2 : 0);
}

void QtOSGViewer::_ProcessBoundingBox()
{
    _posgWidget->DrawBoundingBox(bboxAct->isChecked());
}

void QtOSGViewer::axes()
{
    //?
    //_posgWidget->DrawAxes(AxesAct->isChecked());
}

void QtOSGViewer::_ProcessPointerGroupClicked(int button)
{
    switch (button) {
    case -2:
        _posgWidget->ActivateSelection(false);
        break;
    case -3:
        _posgWidget->SetDraggerMode("TranslateTrackballDragger");
        _posgWidget->ActivateSelection(true);
        break;
    case -4:
        _posgWidget->SetDraggerMode("RotateCylinderDragger");
        _posgWidget->ActivateSelection(true);
        break;
    default:
        RAVELOG_ERROR_FORMAT("pointerGroupClicked failure. Button %d pushed", button);
        _posgWidget->ActivateSelection(false);
        break;
    }
}

void QtOSGViewer::_ProcessDraggerGroupClicked(int button)
{
    RAVELOG_INFO_FORMAT("Dragger button clicked %d", button);
}

void QtOSGViewer::_RepaintWidgets()
{
    _posgWidget->SetSceneData();
}

void QtOSGViewer::_CreateToolsBar()
{
    fileToolBar = addToolBar(tr("File Bar"));
    fileToolBar->addAction(loadAct);
    fileToolBar->addAction(importAct);
    fileToolBar->addAction(saveAct);

//  actionToolBar = addToolBar(tr("Action Bar"));
//  actionToolBar->addAction(playAct);
//  actionToolBar->addAction(stopAct);
//  actionToolBar->addAction(pauseAct);

//  physicsToolBar = addToolBar(tr("Physics Engine Bar"));
//  physicsComboBox = new QComboBox;
//
//  physicsComboBox->addItem(tr("Physics Engine"));
//  physicsComboBox->addItem(tr("Bullet"));
//  physicsComboBox->addItem(tr("ODE"));
//  physicsComboBox->addItem(tr("PAL"));
//  physicsComboBox->addItem(tr("PhysX"));
//
//
//  physicsToolBar->addWidget(physicsComboBox);

    toolsBar = addToolBar(tr("Tools Bar"));
    QToolButton *pointerButton = new QToolButton;
    pointerButton->setCheckable(true);
    pointerButton->setChecked(true);
    pointerButton->setIcon(QIcon(":/images/rotation-icon.png"));

    QToolButton *axesButton = new QToolButton;
    axesButton->setCheckable(true);
    axesButton->setIcon(QIcon(":/images/axes.png"));

    QToolButton *rotationButton = new QToolButton;
    rotationButton->setCheckable(true);
    rotationButton->setIcon(QIcon(":/images/no_edit.png"));
//
//    QToolButton *handButton = new QToolButton;
//    handButton->setCheckable(true);
//    handButton->setIcon(QIcon(":/images/hand.png"));


    _pointerTypeGroup = new QButtonGroup();
    _pointerTypeGroup->addButton(pointerButton);
    _pointerTypeGroup->addButton(axesButton);
    _pointerTypeGroup->addButton(rotationButton);
    //_pointerTypeGroup->addButton(handButton);

    connect(_pointerTypeGroup, SIGNAL(buttonClicked(int)), this, SLOT(_ProcessPointerGroupClicked(int)));

    //shapeGroup = new QActionGroup(this);
    //smoothAct->setCheckable(true);
    //flatAct->setCheckable(true);
    wireAct->setCheckable(true);
    //shapeGroup->addAction(smoothAct);
    //shapeGroup->addAction(flatAct);
    //shapeGroup->addAction(wireAct);
    //smoothAct->setChecked(true);

    toolsBar->addWidget(pointerButton);
    toolsBar->addWidget(axesButton);
    toolsBar->addWidget(rotationButton);
    //toolsBar->addWidget(handButton);
    toolsBar->addAction(houseAct);
    toolsBar->addAction(_qactChangeViewtoXY);
    toolsBar->addAction(_qactChangeViewtoXZ);
    toolsBar->addAction(_qactChangeViewtoYZ);
    toolsBar->addAction(_qactPerspectiveView);
    //toolsBar->addAction(lightAct);
    //toolsBar->addAction(smoothAct);
    //toolsBar->addAction(flatAct);
    toolsBar->addAction(wireAct);
    //toolsBar->addAction(facesAct);
}

void QtOSGViewer::_CreateStatusBar()
{
    statusBar()->showMessage(tr("Ready"));
}

void QtOSGViewer::_CreateDockWidgets()
{
    QDockWidget *dock = new QDockWidget(tr("Objects Tree"), this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    _qobjectTree = _CreateObjectTree();
    dock->setWidget(_qobjectTree);
    dock->hide();

    addDockWidget(Qt::RightDockWidgetArea, dock);
    viewMenu->addAction(dock->toggleViewAction());


    dock = new QDockWidget(tr("Details"), this);

    //  QListWidget *sensorList = new QListWidget(dock);

    _qdetailsTree = new QTreeWidget();
    _qdetailsTree->setHeaderLabel(QString("Properties"));
    dock->setWidget(_qdetailsTree);
    dock->hide();
    addDockWidget(Qt::RightDockWidgetArea, dock);
    viewMenu->addAction(dock->toggleViewAction());
}

void QtOSGViewer::_CreateControlButtons()
{

    QWidget *controlWidget = new QWidget(_posgWidget);
    controlWidget->setGeometry(10, 10, 50, 150);

    QVBoxLayout *qvBoxLayout = new QVBoxLayout(controlWidget);
    qvBoxLayout->setSpacing(5);
    qvBoxLayout->setAlignment(Qt::AlignTop);
    qvBoxLayout->heightForWidth(40);

    QPushButton *zoomInButton = new QPushButton("+");
    connect(zoomInButton, &QPushButton::pressed, [=](){
            this->_posgWidget->Zoom(1.1);
        });

    QPushButton *zoomOutButton = new QPushButton("-");
    connect(zoomOutButton, &QPushButton::pressed, [=](){
            this->_posgWidget->Zoom(0.9);
        });

    QPushButton *cameraMoveModeButton = new QPushButton("Rot");
    cameraMoveModeButton->setText(this->_posgWidget->GetCameraMoveMode());
    connect(cameraMoveModeButton, &QPushButton::pressed, [=]() {
            _posgWidget->ToggleCameraMoveMode();
            cameraMoveModeButton->setText(this->_posgWidget->GetCameraMoveMode());
        });

    qvBoxLayout->addWidget(zoomInButton);
    qvBoxLayout->addWidget(zoomOutButton);
    qvBoxLayout->addWidget(cameraMoveModeButton);

    controlWidget->setLayout(qvBoxLayout);
}

void QtOSGViewer::_OnObjectTreeClick(QTreeWidgetItem* item,int num)
{
    RobotBasePtr robot;
    KinBodyPtr kinbody;
    KinBody::LinkPtr link;

    std::string mass;

    //  Select robot in Viewers
    _posgWidget->SelectItemFromName(item->text(0).toLatin1().data());

    //  Clears details
    if (!!_qdetailsTree) {
        _qdetailsTree->clear();
    }

    QList<QTreeWidgetItem*> items;

    if (!!item->parent()) {
        if (item->parent()->text(0) == "Links") {
            std::ostringstream strs;

            //  Set Title
            if (!!_qdetailsTree) {
                _qdetailsTree->setHeaderLabel(item->text(0).toLatin1().data());
            }

            robot = GetEnv()->GetRobot(item->parent()->parent()->text(0).toLatin1().data());
            link  = robot->GetLink(item->text(0).toLatin1().data());

            //  Clears output string
            strs.clear();

            strs << link->GetMass();

            mass = string(" Mass= ") + strs.str();

            items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(mass.c_str()))));
        }
        else {
            //  Set Title
            if (!!_qdetailsTree) {
                _qdetailsTree->setHeaderLabel(item->text(0).toLatin1().data());
            }
        }
    }
    else {
        //  Set Title
        if (!!_qdetailsTree) {
            _qdetailsTree->setHeaderLabel(item->text(0).toLatin1().data());
        }
        kinbody = GetEnv()->GetKinBody(item->text(0).toLatin1().data());
        for (size_t i=0; i<kinbody->GetLinks().size(); i++) {
            std::ostringstream strs;
            link = kinbody->GetLinks()[i];
            strs << link->GetMass();
            mass = link->GetName() + string(" Mass= ") + strs.str();
            items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(mass.c_str()))));
        }
    }

    //  Add Items to details panel
    if (!!_qdetailsTree) {
        _qdetailsTree->insertTopLevelItems(0, items);
    }
}

QTreeWidget* QtOSGViewer::_CreateObjectTree()
{
    QTreeWidget *treeWidget = new QTreeWidget();
    treeWidget->setColumnCount(1);

    treeWidget->setHeaderLabel(QString("Scene"));

    //  Connect scene list clicked
    connect(treeWidget,SIGNAL(itemClicked(QTreeWidgetItem*,int)), this, SLOT(_OnObjectTreeClick(QTreeWidgetItem*,int)));

    return treeWidget;
}

void QtOSGViewer::_FillObjectTree(QTreeWidget *treeWidget)
{
    RAVELOG_VERBOSE("Begin _FillObjectTree....\n");
    vector<KinBodyPtr> kinbodies;
    QList<QTreeWidgetItem*> items;

    //  Clears tree
    treeWidget->clear();
    GetEnv()->GetBodies(kinbodies);
    for (size_t i = 0; i < kinbodies.size(); i++) {
        items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(kinbodies[i]->GetName().c_str()))));
        //  Number of child to add
        int nchild = -1;

        if( kinbodies[i]->GetLinks().size() > 0 ) {
            //  Header 'Links'
            items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Links"))));
            nchild++;
            FOREACHC(itlink, kinbodies[i]->GetLinks()) {
                items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString((*itlink)->GetName().c_str()))));
            }
        }

        if (kinbodies[i]->GetJoints().size() > 0) {
            //  Header 'Joints'
            items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Joints"))));
            nchild++;

            FOREACHC(itjoint, kinbodies[i]->GetJoints()) {
                KinBody::JointConstPtr pjoint = *itjoint;
                QTreeWidgetItem* pqjoint = new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pjoint->GetName().c_str())));

                //  Adds links of joints
                pqjoint->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pjoint->GetFirstAttached()->GetName().c_str()))));
                pqjoint->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pjoint->GetSecondAttached()->GetName().c_str()))));
                items[i]->child(nchild)->addChild(pqjoint);
            }
        }

        if (kinbodies[i]->IsRobot()) {
            RobotBasePtr robot = RaveInterfaceCast<RobotBase>(kinbodies[i]);

            if (robot->GetAttachedSensors().size() > 0) {
                //  Header 'Sensors'
                items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Sensors"))));
                nchild++;

                FOREACHC(itattsensor, robot->GetAttachedSensors()) {
                    RobotBase::AttachedSensorPtr pattsensor = *itattsensor;
                    QTreeWidgetItem* pqattsensor = new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pattsensor->GetName().c_str())));
                    RAVELOG_VERBOSE_FORMAT("Attach sensor %s robotlink=%s", pattsensor->GetName()%pattsensor->GetAttachingLink()->GetName());
                    pqattsensor->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pattsensor->GetAttachingLink()->GetName().c_str()))));
                    items[i]->child(nchild)->addChild(pqattsensor);
                }
            }

            if (robot->GetManipulators().size() > 0) {
                //  Header 'Manipulators'
                items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Manipulators"))));
                nchild++;

                FOREACHC(itmanip, robot->GetManipulators()) {
                    RobotBase::ManipulatorPtr pmanip = *itmanip;
                    items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pmanip->GetName().c_str()))));
                }
            }

            ControllerBasePtr controller = robot->GetController();
            if (!!controller) {
                //  Header 'Controller'
                items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Controller"))));
                nchild++;

                items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(controller->GetXMLFilename().c_str()))));
            }
        }
    }

    treeWidget->insertTopLevelItems(0, items);
    RAVELOG_VERBOSE("End _FillObjectTree....\n");
}

void QtOSGViewer::_UpdateCameraTransform(float fTimeElapsed)
{
    boost::mutex::scoped_lock lock(_mutexGUIFunctions);

    int width = centralWidget()->size().width();
    int height = centralWidget()->size().height();
    _posgWidget->UpdateHUDAxisTransform(width, height);

    double fovy, aspectRatio, zNear, zFar;
    _posgWidget->GetCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
    int camwidth = _posgWidget->GetCamera()->getViewport()->width();
    int camheight = _posgWidget->GetCamera()->getViewport()->height();

    _camintrinsics.fy = 0.5*camheight/RaveTan(0.5f*fovy*M_PI/180.0);
    _camintrinsics.fx = _camintrinsics.fy*float(camwidth)/(float)camheight/aspectRatio;
    _camintrinsics.cx = (float)camwidth/2;
    _camintrinsics.cy = (float)camheight/2;
    _camintrinsics.focal_length = zNear;
    _camintrinsics.distortion_model = "";
}

bool QtOSGViewer::_SetFiguresInCamera(ostream& sout, istream& sinput)
{
    sinput >> _bRenderFiguresInCamera;
    return !!sinput;
}

bool QtOSGViewer::_SetItemVisualizationCommand(ostream& sout, istream& sinput)
{
    std::string itemname, visualizationmode;
    sinput >> itemname >> visualizationmode;

    _PostToGUIThread(boost::bind(&QtOSGViewer::_SetItemVisualization, this, itemname, visualizationmode));
    return !!sinput;
}

void QtOSGViewer::_SetItemVisualization(std::string& itemname, std::string& visualizationmode)
{
    FOREACH(it, _mapbodies) {
        if( it->second->GetName() == itemname ) {
            it->second->SetVisualizationMode(visualizationmode);
        }
    }
}

bool QtOSGViewer::_ShowWorldAxesCommand(ostream& sout, istream& sinput)
{
    // TODO
    return true;
}

bool QtOSGViewer::_SetNearPlaneCommand(ostream& sout, istream& sinput)
{
    dReal nearplane=0.01;
    sinput >> nearplane;
    if( !!sinput ) {
        _posgWidget->SetNearPlane(nearplane);
    }
    return !!sinput;
}

bool QtOSGViewer::_SetTextureCubeMap(ostream& out, istream& sinput)
{
    std::string posx, negx, posy, negy, posz, negz;
    sinput >> posx >> negx >> posy >> negy >> posz >> negz;

    if(!!sinput) {
        _posgWidget->SetTextureCubeMap(posx, negx, posy, negy, posz, negz);
    }

    return !!sinput;
}

bool QtOSGViewer::_TrackLinkCommand(ostream& sout, istream& sinput)
{
    bool bresetvelocity = true;
    std::string bodyname, linkname;
    float focalDistance = 0.0;
    Transform tTrackingLinkRelative;
    sinput >> bodyname >> linkname >> focalDistance >> bresetvelocity;
    if( focalDistance > 0 ) {
        _SetCameraDistanceToFocus(focalDistance);
    }
    _ptrackinglink.reset();
    _ptrackingmanip.reset();
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    KinBodyPtr pbody = GetEnv()->GetKinBody(bodyname);
    if( !pbody ) {
        // restore navigation manipulator
        _SetTrackManipulatorToStopTracking();
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
    KinBodyItemPtr kinBodyItem = _posgWidget->GetItemFromName(bodyname);
    if(!_SetTrackManipulatorToTrackLink(_ptrackinglink, kinBodyItem, _tTrackingLinkRelative)) {
        return false;
    }
    if( bresetvelocity ) {
        _tTrackingCameraVelocity.trans = _tTrackingCameraVelocity.rot = Vector(); // reset velocity?
    }
    return !!_ptrackinglink;
}

bool QtOSGViewer::_TrackManipulatorCommand(ostream& sout, istream& sinput)
{
    bool bresetvelocity = true;
    std::string robotname, manipname;
    float focalDistance = 0.0;
    sinput >> robotname >> manipname >> focalDistance >> bresetvelocity;
    if( focalDistance > 0 ) {
        // not sure if this is thread safe...
        _SetCameraDistanceToFocus(focalDistance);
    }
    _ptrackinglink.reset();
    _ptrackingmanip.reset();
    EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    RobotBasePtr probot = GetEnv()->GetRobot(robotname);
    _ptrackingmanip = probot->GetManipulator(manipname);
    if( !probot ) {
        return false;
    }
    if( bresetvelocity ) {
        _tTrackingCameraVelocity.trans = _tTrackingCameraVelocity.rot = Vector(); // reset velocity?
    }
    return !!_ptrackingmanip;
}

bool QtOSGViewer::_SetTrackingAngleToUpCommand(ostream& sout, istream& sinput)
{
    sinput >> _fTrackAngleToUp;
    return true;
}

void QtOSGViewer::_ProcessApplicationQuit()
{
    RAVELOG_VERBOSE("processing viewer application quit\n");
    // remove all messages in order to release the locks
    boost::mutex::scoped_lock lockmsg(_mutexGUIFunctions);
    if( _listGUIFunctions.size() > 0 ) {
        bool bnotify = false;
        FOREACH(it,_listGUIFunctions) {
            (*it)->SetFinished();
            if( (*it)->IsBlocking() ) {
                bnotify = true;
            }
        }
        if( bnotify ) {
            _notifyGUIFunctionComplete.notify_all();
        }
    }
    _listGUIFunctions.clear();

}

bool QtOSGViewer::_StartViewerLoopCommand(ostream& sout, istream& sinput)
{
    if( !QApplication::instance() ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("need a valid QApplication before viewer loop is run", ORE_InvalidState);
    }

    bool bcallmain = false;
    sinput >> bcallmain;
    _nQuitMainLoop = -1;
    //_StartPlaybackTimer();
    Show(1);
    if( bcallmain ) {
        _bExternalLoop = false;
        _posgWidget->SetHome();
        QApplication::instance()->exec();
        _nQuitMainLoop = 2; // have to specify that quit!
    } else {
        _bExternalLoop = true;
    }
    return true;
}

bool QtOSGViewer::_SetProjectionModeCommand(ostream& sout, istream& sinput)
{
    std::string projectionMode = "";
    sinput >> projectionMode;
    _PostToGUIThread(boost::bind(&QtOSGViewer::_SetProjectionMode, this, projectionMode));
    return true;
}

bool QtOSGViewer::_ZoomCommand(ostream& sout, istream& sinput)
{
    float factor = 1.0f;
    sinput >> factor;
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Zoom, this, factor));
    return true;
}

void QtOSGViewer::_SetProjectionMode(const std::string& projectionMode)
{
    if (projectionMode == "orthogonal")
    {
        _qactPerspectiveView->setChecked(true);
        _posgWidget->SetViewType(true);
    }
    else
    {
        _qactPerspectiveView->setChecked(false);
        _posgWidget->SetViewType(false);
    }
}

int QtOSGViewer::main(bool bShow)
{
    if( !QApplication::instance() ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("need a valid QApplication before viewer loop is run", ORE_InvalidState);
    }
    _nQuitMainLoop = -1;
    //_StartPlaybackTimer();
    if (bShow) {
        if( _nQuitMainLoop < 0 ) {
            Show(1);
        }
    }

    UpdateFromModel();
    _posgWidget->SetHome();
    if( _nQuitMainLoop < 0 ) {
        _bExternalLoop = false;
        QApplication::instance()->exec();
        _nQuitMainLoop = 2; // have to specify that quit!
    }
    SetEnvironmentSync(false);
    return 0;
}

void QtOSGViewer::quitmainloop()
{
    _nQuitMainLoop = 1;
    bool bGuiThread = QThread::currentThread() == QCoreApplication::instance()->thread();
    if( !bGuiThread ) {
        SetEnvironmentSync(false);
    }
    if (!_bExternalLoop) {
        QApplication::instance()->exit(0);
    }
    _nQuitMainLoop = 2;
}

void QtOSGViewer::Show(int showtype)
{
    if( showtype ) {
        _PostToGUIThread(boost::bind(&QtOSGViewer::show, this));
    }
    else {
        _PostToGUIThread(boost::bind(&QtOSGViewer::hide, this));
    }
//    // have to put this in the message queue
//    if (showtype ) {
//        show();
//    }
//    else {
//        hide();
//    }
}

bool QtOSGViewer::GetFractionOccluded(KinBodyPtr pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded)
{
    return false;
}

bool QtOSGViewer::GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK)
{
    return false;
}

bool QtOSGViewer::WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension)
{
    return false;
}

void QtOSGViewer::_SetCameraTransform()
{
    osg::ref_ptr<osgGA::TrackballManipulator> ptrackball = osg::dynamic_pointer_cast<osgGA::TrackballManipulator>(_posgWidget->GetCurrentCameraManipulator());
    if( !!ptrackball ) {
        ptrackball->setDistance(_focalDistance);
    }

    // has to come after setting distance because internally orbit manipulator uses the distance to deduct view center
    _posgWidget->GetCurrentCameraManipulator()->setByMatrix(GetMatrixFromRaveTransform(_Tcamera));

    //osg::Vec3d eye, center, up;
    //osg::Matrix::inverse(GetMatrixFromRaveTransform(_Tcamera)).getLookAt(eye, center, up, _focalDistance);
    //_posgWidget->GetCurrentCameraManipulator()->setTransformation(eye, center, up);
}

void QtOSGViewer::_SetCamera(RaveTransform<float> trans, float focalDistance)
{
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    _Tcamera = trans*trot;

    if (focalDistance > 0) {
        _focalDistance = focalDistance;
    }

    _SetCameraTransform();
    _UpdateCameraTransform(0);
}

void QtOSGViewer::SetCamera(const RaveTransform<float>& trans, float focalDistance)
{
    _PostToGUIThread(boost::bind(&QtOSGViewer::_SetCamera, this, trans, focalDistance));
}

void QtOSGViewer::_SetTrackManipulatorToStopTracking()
{
    _posgWidget->RestoreDefaultManipulator();
}

bool QtOSGViewer::_SetTrackManipulatorToTrackLink(KinBody::LinkPtr link, KinBodyItemPtr linkParentKinBodyItem, const RaveTransform<float>& linkRelativeTranslation)
{
    if(!linkParentKinBodyItem || !link) {
        RAVELOG_ERROR_FORMAT("Could not set track manipulator to track link %s of object %s", link->GetName()%linkParentKinBodyItem->GetName());
        return false;
    }
    auto osgNode = linkParentKinBodyItem->GetOSGLink(link->GetIndex());
    assert(osgNode);

    OpenRAVETracker* trackManipulator = _posgWidget->GetTrackModeManipulator();
    _posgWidget->SetCurrentCameraManipulator(trackManipulator);
    trackManipulator->startTrackingNode(osgNode.get(), _posgWidget->GetCamera(), osg::Vec3d(0,0,1));
    trackManipulator->setDistance(_focalDistance);
    trackManipulator->setOffset(osg::Vec3d(linkRelativeTranslation.trans[0], linkRelativeTranslation.trans[1], linkRelativeTranslation.trans[2]));
    return true;
}

void QtOSGViewer::_SetCameraDistanceToFocus(float focalDistance)
{
    if (focalDistance > 0) {
        _focalDistance = focalDistance;
    }

    _SetCameraTransform();
    _UpdateCameraTransform(0);
}

RaveTransform<float> QtOSGViewer::GetCameraTransform() const
{
    boost::mutex::scoped_lock lock(_mutexGUIFunctions);
    // have to flip Z axis
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    return _Tcamera*trot;
}

float QtOSGViewer::GetCameraDistanceToFocus() const
{
    boost::mutex::scoped_lock lock(_mutexGUIFunctions);
    return _focalDistance;
}

geometry::RaveCameraIntrinsics<float> QtOSGViewer::GetCameraIntrinsics() const
{
    boost::mutex::scoped_lock lock(_mutexGUIFunctions);
    return _camintrinsics;
}

SensorBase::CameraIntrinsics QtOSGViewer::GetCameraIntrinsics2() const
{
    boost::mutex::scoped_lock lock(_mutexGUIFunctions);
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

void QtOSGViewer::_Draw(OSGSwitchPtr handle, osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::Vec4Array> colors, osg::PrimitiveSet::Mode mode, osg::ref_ptr<osg::StateAttribute> attribute, bool bUsingTransparency)
{
    OSGMatrixTransformPtr trans(new osg::MatrixTransform());
    osg::ref_ptr<osg::Geode> geode(new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry(new osg::Geometry());

    geometry->setVertexArray(vertices.get());
    geometry->setColorArray(colors.get());
    geometry->setColorBinding(colors->size() == vertices->size() ? osg::Geometry::BIND_PER_VERTEX : osg::Geometry::BIND_OVERALL);

    geometry->addPrimitiveSet(new osg::DrawArrays(mode, 0, vertices->size()));
    geometry->getOrCreateStateSet()->setAttribute(attribute, osg::StateAttribute::ON);

    // don't do transparent bin since that is too slow for big point clouds...
    //geometry->getOrCreateStateSet()->setRenderBinDetails(0, "transparent");
    geometry->getOrCreateStateSet()->setRenderingHint(bUsingTransparency ? osg::StateSet::TRANSPARENT_BIN : osg::StateSet::OPAQUE_BIN);

    geode->addDrawable(geometry.get());

    trans->addChild(geode);
    handle->addChild(trans);
    _posgWidget->GetFigureRoot()->insertChild(0, handle);
}

GraphHandlePtr QtOSGViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
{
    OSGSwitchPtr handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
        ppoints = (float*)((char*)ppoints + stride);
    }
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(1);
    (*vcolors)[0] = osg::Vec4f(color.x, color.y, color.z, color.w);
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::POINTS, new osg::Point(fPointSize),color.w<1)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle, bool bhasalpha)
{
    OSGSwitchPtr handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
        ppoints = (float*)((char*)ppoints + stride);
        if (bhasalpha) {
            (*vcolors)[i] = osg::Vec4f(colors[i * 4 + 0], colors[i * 4 + 1], colors[i * 4 + 2], colors[i * 4 + 3]);
        }
        else {
            (*vcolors)[i] = osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f);
        }
    }

    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::POINTS, osg::ref_ptr<osg::Point>(new osg::Point(fPointSize)), bhasalpha)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    OSGSwitchPtr handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
        ppoints = (float*)((char*)ppoints + stride);
    }
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(1);
    (*vcolors)[0] = osg::Vec4f(color.x, color.y, color.z, color.w);
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINE_STRIP, osg::ref_ptr<osg::LineWidth>(new osg::LineWidth(fwidth)), color.w<1)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}
GraphHandlePtr QtOSGViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    OSGSwitchPtr handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
        ppoints = (float*)((char*)ppoints + stride);
        (*vcolors)[i] = osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f);
    }

    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINE_STRIP, osg::ref_ptr<osg::LineWidth>(new osg::LineWidth(fwidth)), false)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    OSGSwitchPtr handle = _CreateGraphHandle();
    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
        ppoints = (float*)((char*)ppoints + stride);
    }
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(1);
    (*vcolors)[0] = osg::Vec4f(color.x, color.y, color.z, color.w);
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINES, osg::ref_ptr<osg::LineWidth>(new osg::LineWidth(fwidth)), color.w<1)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}
GraphHandlePtr QtOSGViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    OSGSwitchPtr handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
        ppoints = (float*)((char*)ppoints + stride);
        (*vcolors)[i] = osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f);
    }

    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINES, osg::ref_ptr<osg::LineWidth>(new osg::LineWidth(fwidth)), false)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
{
    RAVELOG_WARN("drawarrow not implemented\n");
    return GraphHandlePtr();
}

void QtOSGViewer::_DrawBox(OSGSwitchPtr handle, const RaveVector<float>& vpos, const RaveVector<float>& vextents, bool bUsingTransparency)
{
    OSGMatrixTransformPtr trans(new osg::MatrixTransform());
    osg::ref_ptr<osg::Geode> geode(new osg::Geode());

    osg::ref_ptr<osg::Box> box = new osg::Box();
    box->setHalfLengths(osg::Vec3(vextents.x, vextents.y, vextents.z));
    box->setCenter(osg::Vec3(vpos.x, vpos.y, vpos.z));

    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(box.get());
    geode->addDrawable(sd);

    // don't do transparent bin since that is too slow for big point clouds...
    //geometry->getOrCreateStateSet()->setRenderBinDetails(0, "transparent");
    handle->getOrCreateStateSet()->setRenderingHint(bUsingTransparency ? osg::StateSet::TRANSPARENT_BIN : osg::StateSet::OPAQUE_BIN);

    trans->addChild(geode);
    handle->addChild(trans);
    _posgWidget->GetFigureRoot()->insertChild(0, handle);
}

GraphHandlePtr QtOSGViewer::drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
{
    OSGSwitchPtr handle = _CreateGraphHandle();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_DrawBox, this, handle, vpos, vextents, false)); // copies ref counts
    return GraphHandlePtr();
}

void QtOSGViewer::_DrawPlane(OSGSwitchPtr handle, const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
{
    OSGMatrixTransformPtr trans(new osg::MatrixTransform());
    osg::ref_ptr<osg::Geode> geode(new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry(new osg::Geometry());

    // make triangleMesh
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4);
    (*vertices)[0] = GetOSGVec3(tplane*RaveVector<float>(-vextents.x, vextents.y, 0));
    (*vertices)[1] = GetOSGVec3(tplane*RaveVector<float>(-vextents.x, -vextents.y, 0));
    (*vertices)[2] = GetOSGVec3(tplane*RaveVector<float>(vextents.x, -vextents.y, 0));
    (*vertices)[3] = GetOSGVec3(tplane*RaveVector<float>(vextents.x, vextents.y, 0));
    geometry->setVertexArray(vertices.get());

    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0,0);
    (*texcoords)[1].set(0,1);
    (*texcoords)[2].set(1,1);
    (*texcoords)[3].set(1,0);
    geometry->setTexCoordArray(0,texcoords);

    geometry->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    osg::ref_ptr<osg::Vec4Array> osgcolors = new osg::Vec4Array(1);
    (*osgcolors)[0] = osg::Vec4(1,1,1,1);
    geometry->setColorArray(osgcolors.get());
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    geode->addDrawable(geometry.get());

    bool bhasalpha = false;
    // create the osg texture
    size_t imagesize = vtexture.shape()[0]*vtexture.shape()[1]*vtexture.shape()[2];
    if( imagesize > 0 ) {
        uint8_t* vimagedata = new uint8_t[imagesize]; // Image class will take care of deleting
        bhasalpha = vtexture.shape()[2] == 4;
        GLint internalTextureFormat = vtexture.shape()[2]; // number of components in texture?
        GLenum format = GL_RGBA;
        switch( vtexture.shape()[2] ) {
        case 1: format = GL_R; break;
        case 2: format = GL_RG; break;
        case 3: format = GL_RGB; break;
        }

        osg::ref_ptr<osg::Image> image;
        uint8_t* pdst = vimagedata;
        FOREACHC(ith,vtexture) {
            FOREACHC(itw,*ith) {
                FOREACHC(itp,*itw) {
                    *pdst++ = (unsigned char)(255.0f*ClampOnRange(*itp,0.0f,1.0f));
                }
            }
        }
        image = new osg::Image();
        image->setImage(vtexture.shape()[0], vtexture.shape()[1], 1, internalTextureFormat, format, GL_UNSIGNED_BYTE, vimagedata, osg::Image::USE_NEW_DELETE);

        osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
        tex->setImage(image);
        osg::ref_ptr<osg::StateSet> stateOne = new osg::StateSet();

        // Assign texture unit 0 of our new StateSet to the texture
        // we just created and enable the texture.
        stateOne->setTextureAttributeAndModes(0,tex,osg::StateAttribute::ON);
        geode->setStateSet(stateOne);
    }

    // don't do transparent bin since that is too slow for big point clouds...
    //geometry->getOrCreateStateSet()->setRenderBinDetails(0, "transparent");
    geometry->getOrCreateStateSet()->setRenderingHint(bhasalpha ? osg::StateSet::TRANSPARENT_BIN : osg::StateSet::OPAQUE_BIN);

    trans->addChild(geode);
    handle->addChild(trans);
    _posgWidget->GetFigureRoot()->insertChild(0, handle);
}

GraphHandlePtr QtOSGViewer::drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
{
    OSGSwitchPtr handle = _CreateGraphHandle();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_DrawPlane, this, handle, tplane, vextents, vtexture)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

void QtOSGViewer::_DrawTriMesh(OSGSwitchPtr handle, osg::ref_ptr<osg::Vec3Array> vertices, osg::ref_ptr<osg::Vec4Array> colors, osg::ref_ptr<osg::DrawElementsUInt> osgindices, bool bUsingTransparency)
{
    OSGMatrixTransformPtr trans(new osg::MatrixTransform());
    osg::ref_ptr<osg::Geode> geode(new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry(new osg::Geometry());

    // make triangleMesh
    geometry->setVertexArray(vertices.get());
    geometry->addPrimitiveSet(osgindices.get());

    geometry->setColorArray(colors.get());
    geometry->setColorBinding(colors->size() == vertices->size() ? osg::Geometry::BIND_PER_VERTEX : osg::Geometry::BIND_OVERALL);

    // don't do transparent bin since that is too slow for big point clouds...
    //geometry->getOrCreateStateSet()->setRenderBinDetails(0, "transparent");
    geometry->getOrCreateStateSet()->setRenderingHint(bUsingTransparency ? osg::StateSet::TRANSPARENT_BIN : osg::StateSet::OPAQUE_BIN);

    geode->addDrawable(geometry.get());

    trans->addChild(geode);
    handle->addChild(trans);
    _posgWidget->GetFigureRoot()->insertChild(0, handle);
}

void QtOSGViewer::_SetTriangleMesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, osg::ref_ptr<osg::Vec3Array> osgvertices, osg::ref_ptr<osg::DrawElementsUInt> osgindices)
{
    BOOST_ASSERT(ppoints!=NULL);
    int maxindex = -1;
    osgindices->resizeElements(numTriangles*3);
    if( pIndices != NULL ) {
        for(int i = 0; i < 3*numTriangles; ++i) {
            (*osgindices)[i] = pIndices[i];
            if( maxindex < pIndices[i] ) {
                maxindex = pIndices[i];
            }
        }
    }
    else {
        maxindex = 3*numTriangles-1;
        for(int i = 0; i < 3*numTriangles; ++i) {
            (*osgindices)[i] = i;
        }
    }
    osgvertices->resizeArray(maxindex+1);
    for(int i = 0; i <= maxindex; ++i) {
        (*osgvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
        ppoints = (float*)((char*)ppoints + stride);
    }
}

GraphHandlePtr QtOSGViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
{
    osg::ref_ptr<osg::Vec3Array> osgvertices = new osg::Vec3Array();
    osg::ref_ptr<osg::DrawElementsUInt> osgindices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);
    _SetTriangleMesh(ppoints, stride, pIndices, numTriangles, osgvertices, osgindices);

    OSGSwitchPtr handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec4Array> osgcolors = new osg::Vec4Array(1);
    (*osgcolors)[0] = osg::Vec4f(color.x, color.y, color.z, color.w);

    _PostToGUIThread(boost::bind(&QtOSGViewer::_DrawTriMesh, this, handle, osgvertices, osgcolors, osgindices, color.w<1)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
{
    osg::ref_ptr<osg::Vec3Array> osgvertices = new osg::Vec3Array();
    osg::ref_ptr<osg::DrawElementsUInt> osgindices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);
    _SetTriangleMesh(ppoints, stride, pIndices, numTriangles, osgvertices, osgindices);

    bool bhasalpha = false;
    osg::ref_ptr<osg::Vec4Array> osgcolors = new osg::Vec4Array(colors.size());
    switch(colors.shape()[1]) {
    case 1:
        for(size_t i = 0; i < colors.shape()[0]; ++i) {
            (*osgcolors)[i] = osg::Vec4f(colors[i][0], colors[i][0], colors[i][0], 1);
        }
        break;
    case 4:
        for(size_t i = 0; i < colors.shape()[0]; ++i) {
            (*osgcolors)[i] = osg::Vec4(colors[i][0], colors[i][1], colors[i][2], colors[i][3]);
        }
        bhasalpha = true;
        break;
    case 3:
        for(size_t i = 0; i < colors.shape()[0]; ++i) {
            (*osgcolors)[i] = osg::Vec4(colors[i][0], colors[i][1], colors[i][2], 1);
        }
        break;
    default:
        RAVELOG_WARN_FORMAT("unsupported color dimension %d", colors.shape()[1]);
        return GraphHandlePtr();
    }

    OSGSwitchPtr handle = _CreateGraphHandle();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_DrawTriMesh, this, handle, osgvertices, osgcolors, osgindices, bhasalpha)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

void QtOSGViewer::_Deselect()
{

}

void QtOSGViewer::Reset()
{
    // TODO
}

void QtOSGViewer::SetBkgndColor(const RaveVector<float>& color)
{
    _PostToGUIThread(boost::bind(&QtOSGViewer::_SetBkgndColor, this, color));
}

void QtOSGViewer::_SetBkgndColor(const RaveVector<float>& color)
{
    _posgWidget->GetCamera()->setClearColor(osg::Vec4f(color.x, color.y, color.z, 1.0));
}

void QtOSGViewer::StartPlaybackTimer()
{

}
void QtOSGViewer::StopPlaybackTimer()
{

}

void QtOSGViewer::SetSize(int w, int h)
{
    _PostToGUIThread(boost::bind(&QtOSGViewer::resize, this, w, h));
}
void QtOSGViewer::Move(int x, int y)
{
    _PostToGUIThread(boost::bind(&QtOSGViewer::move, this, x, y));
}

void QtOSGViewer::Zoom(float factor)
{
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Zoom, this, factor));
}

void QtOSGViewer::_Zoom(float factor)
{
    _posgWidget->Zoom(factor);
}

void QtOSGViewer::SetName(const string& name)
{
    _PostToGUIThread(boost::bind(&QtOSGViewer::_SetName, this, name));
}

void QtOSGViewer::_SetName(const string& name)
{
    setWindowTitle(name.c_str());
}

bool QtOSGViewer::LoadModel(const string& filename)
{
    if( filename == "") {
        return false;
    }
    OSGNodePtr loadedModel = osgDB::readNodeFile(filename);
    if (!!loadedModel) {
        _posgWidget->GetSceneRoot()->addChild(loadedModel.get());
        return true;
    }

    return false;
}

void QtOSGViewer::UpdateFromModel()
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

    bool newdata = false; // set to true if new object was created
    FOREACH(itbody, vecbodies) {
        BOOST_ASSERT( !!itbody->pbody );
        KinBodyPtr pbody = itbody->pbody; // try to use only as an id, don't call any methods!
        KinBodyItemPtr pitem = boost::dynamic_pointer_cast<KinBodyItem>(pbody->GetUserData(_userdatakey));

        if( !!pitem ) {
            if( !pitem->GetBody() ) {
                // looks like item has been destroyed, so remove from data
                pbody->RemoveUserData(_userdatakey);
                pitem.reset();
            }
        }

        if( !pitem ) {
            // create a new body
            // make sure pbody is actually present
            if( GetEnv()->GetBodyFromEnvironmentId(itbody->environmentid) == pbody ) {

                // check to make sure the real GUI data is also NULL
                if( !pbody->GetUserData(_userdatakey) ) {
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
                        pitem = boost::shared_ptr<RobotItem>(new RobotItem(_posgWidget->GetSceneRoot(), _posgWidget->GetFigureRoot(), boost::static_pointer_cast<RobotBase>(pbody), _viewGeometryMode), ITEM_DELETER);
                    }
                    else {
                        pitem = boost::shared_ptr<KinBodyItem>(new KinBodyItem(_posgWidget->GetSceneRoot(), _posgWidget->GetFigureRoot(), pbody, _viewGeometryMode), ITEM_DELETER);
                    }
                    newdata = true;

                    // TODO
//                    if( !!_pdragger && _pdragger->GetSelectedItem() == pitem ) {
//                        _Deselect();
//                    }
                    pitem->Load();

                    pbody->SetUserData(_userdatakey, pitem);
                    _mapbodies[pbody] = pitem;
                    newdata = true;
                }
                else {
                    pitem = boost::static_pointer_cast<KinBodyItem>(pbody->GetUserData(_userdatakey));
                    BOOST_ASSERT( _mapbodies.find(pbody) != _mapbodies.end() && _mapbodies[pbody] == pitem );
                }
            }
            else {
                // body is gone
                continue;
            }
        }

        map<KinBodyPtr, KinBodyItemPtr>::iterator itmap = _mapbodies.find(pbody);

        //  There are NO mapbodies
        if( itmap == _mapbodies.end() ) {
            continue;
        }

        BOOST_ASSERT( pitem->GetBody() == pbody);
        BOOST_ASSERT( itmap->second == pitem );

        pitem->SetUserData(1);

        //  Update viewer with core transforms
        pitem->UpdateFromModel(itbody->jointvalues, itbody->vectrans);
    }

    FOREACH_NOINC(it, _mapbodies) {
        if( !it->second->GetUserData() ) {
            // item doesn't exist anymore, remove it
            it->first->RemoveUserData(_userdatakey);

            if( _pSelectedItem == it->second ) {
                // TODO
                //_pdragger.reset();
                _pSelectedItem.reset();
                //_osgViewerRoot->deselectAll();
            }

            _mapbodies.erase(it++);
        }
        else {
            ++it;
        }
    }

    _bModelsUpdated = true;
    _condUpdateModels.notify_all();

//    if( _bAutoSetCamera &&(_mapbodies.size() > 0)) {
//        RAVELOG_DEBUG("auto-setting camera location\n");
//        _bAutoSetCamera = false;
//        _pviewer->viewAll();
//    }

    //  Repaint the scene created
    //_RepaintWidgets();

    if (newdata) {
        if (!!_qobjectTree) {
            //  Fill tree widget with robot and joints
            _FillObjectTree(_qobjectTree);
        }
    }
}

boost::shared_ptr<EnvironmentMutex::scoped_try_lock> QtOSGViewer::LockEnvironment(uint64_t timeout,bool bUpdateEnvironment)
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

void QtOSGViewer::_UpdateEnvironment(float fTimeElapsed)
{
    boost::mutex::scoped_lock lockupd(_mutexUpdating);

    if( _bUpdateEnvironment ) {
        // process all messages
        list<GUIThreadFunctionPtr> listGUIFunctions;
        {
            boost::mutex::scoped_lock lockmsg(_mutexGUIFunctions);
            listGUIFunctions.swap(_listGUIFunctions);
        }

        FOREACH(itmsg, listGUIFunctions) {
            try {
                (*itmsg)->Call();
            }
            catch(const std::exception& ex) {
                RAVELOG_WARN_FORMAT("gui thread function failed: %s", ex.what());
            }
        }

        // have to update model after messages since it can lock the environment
        UpdateFromModel();
        _UpdateCameraTransform(fTimeElapsed);
    }
}

void QtOSGViewer::_PostToGUIThread(const boost::function<void()>& fn, bool block)
{
    if( _nQuitMainLoop != -1 ) {
        // viewer quit, so anything posted won't get processed
        return;
    }

    boost::mutex::scoped_lock lockmsg(_mutexGUIFunctions);
    if( _listGUIFunctions.size() > 1000 ) {
        // can happen if system is especially slow
        //RAVELOG_WARN_FORMAT("too many gui post commands, ignoring %d", _listGUIFunctions.size());
        return;
    }
    GUIThreadFunctionPtr pfn(new GUIThreadFunction(fn, block));
    _listGUIFunctions.push_back(pfn);
    if( block ) {
        while(!pfn->IsFinished()) {
            _notifyGUIFunctionComplete.wait(_mutexGUIFunctions);
        }
    }
}

void QtOSGViewer::SetEnvironmentSync(bool bUpdate)
{
    boost::mutex::scoped_lock lockupdating(_mutexUpdating);
    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    _bUpdateEnvironment = bUpdate;
    _condUpdateModels.notify_all();

    if( !bUpdate ) {
        // remove all messages in order to release the locks
        boost::mutex::scoped_lock lockmsg(_mutexGUIFunctions);
        if( _listGUIFunctions.size() > 0 ) {
            bool bnotify = false;
            FOREACH(it,_listGUIFunctions) {
                (*it)->SetFinished();
                if( (*it)->IsBlocking() ) {
                    bnotify = true;
                }
            }
            if( bnotify ) {
                _notifyGUIFunctionComplete.notify_all();
            }
        }
        _listGUIFunctions.clear();
    }
}

void QtOSGViewer::_DeleteItemCallback(Item* pItem)
{
    boost::mutex::scoped_lock lock(_mutexItems);
    pItem->PrepForDeletion();
    _listRemoveItems.push_back(pItem);
}

void QtOSGViewer::EnvironmentSync()
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
        RAVELOG_WARNA("failed to update models from environment sync\n");
    }
}

OSGSwitchPtr QtOSGViewer::_CreateGraphHandle()
{
    OSGSwitchPtr handle(new osg::Switch());
    handle->setAllChildrenOn();
    return handle;
}

void QtOSGViewer::_CloseGraphHandle(OSGSwitchPtr handle)
{
    _posgWidget->GetFigureRoot()->removeChild(handle);
}

void QtOSGViewer::_SetGraphTransform(OSGSwitchPtr handle, const RaveTransform<float> t)
{
    // have to convert to smart pointers so that we can get exceptions thrown rather than dereferencing null pointers
    SetMatrixTransform(*OSGMatrixTransformPtr(OSGTransformPtr(OSGNodePtr(handle->getChild(0))->asTransform())->asMatrixTransform()), t);
}

void QtOSGViewer::_SetGraphShow(OSGSwitchPtr handle, bool bShow)
{
    if (bShow) {
        handle->setAllChildrenOn();
    }
    else {
        handle->setAllChildrenOff();
    }
}

UserDataPtr QtOSGViewer::RegisterItemSelectionCallback(const ItemSelectionCallbackFn& fncallback)
{
    ItemSelectionCallbackDataPtr pdata(new ItemSelectionCallbackData(fncallback,shared_viewer()));
    pdata->_iterator = _listRegisteredItemSelectionCallbacks.insert(_listRegisteredItemSelectionCallbacks.end(),pdata);
    return pdata;
}

UserDataPtr QtOSGViewer::RegisterViewerThreadCallback(const ViewerThreadCallbackFn& fncallback)
{
    ViewerThreadCallbackDataPtr pdata(new ViewerThreadCallbackData(fncallback,shared_viewer()));
    pdata->_iterator = _listRegisteredViewerThreadCallbacks.insert(_listRegisteredViewerThreadCallbacks.end(),pdata);
    return pdata;
}

ViewerBasePtr CreateQtOSGViewer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ViewerBasePtr(new QtOSGViewer(penv, sinput));
}

} // end namespace qtosgviewer
