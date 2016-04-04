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
#include "qtosgviewer.h"

#include <boost/lexical_cast.hpp>
#include <iostream>

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

    objectTree = NULL;
    detailsTree = NULL;
    _focalDistance = 0.0;
    _fTrackAngleToUp = 0.3;

    _userdatakey = std::string("qtosg") + boost::lexical_cast<std::string>(this);

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

    _name = str(boost::format("OpenRAVE %s")%OPENRAVE_VERSION_STRING);
    if( (OPENRAVE_VERSION_MINOR%2) || (OPENRAVE_VERSION_PATCH%2) ) {
        _name += " (Development Version)";
    }
    else {
        _name += " (Stable Release)";
    }
    setWindowTitle(_name.c_str());

    if(bCreateStatusBar) {
        statusBar()->showMessage(tr("Status Bar"));
    }

    __description = ":Interface Author: Gustavo Puche, Rosen Diankov\n\nProvides a viewer based on Open Scene Graph.";

    RegisterCommand("SetFiguresInCamera",boost::bind(&QtOSGViewer::_SetFiguresInCamera, this, _1, _2),
                    "Accepts 0/1 value that decides whether to render the figure plots in the camera image through GetCameraImage");
    RegisterCommand("SetFeedbackVisibility",boost::bind(&QtOSGViewer::_SetFeedbackVisibility, this, _1, _2),
                    "Accepts 0/1 value that decides whether to render the cross hairs");
    RegisterCommand("ShowWorldAxes",boost::bind(&QtOSGViewer::_SetFeedbackVisibility, this, _1, _2),
                    "Accepts 0/1 value that decides whether to render the cross hairs");
    RegisterCommand("SetNearPlane", boost::bind(&QtOSGViewer::_SetNearPlaneCommand, this, _1, _2),
                    "Sets the near plane for rendering of the image. Useful when tweaking rendering units");
    RegisterCommand("TrackLink", boost::bind(&QtOSGViewer::_TrackLinkCommand, this, _1, _2),
                    "camera tracks the link maintaining a specific relative transform: robotname, manipname, focalDistance");
    RegisterCommand("TrackManipulator", boost::bind(&QtOSGViewer::_TrackManipulatorCommand, this, _1, _2),
                    "camera tracks the manipulator maintaining a specific relative transform: robotname, manipname, focalDistance");
    RegisterCommand("SetTrackingAngleToUp", boost::bind(&QtOSGViewer::_SetTrackingAngleToUpCommand, this, _1, _2),
                    "sets a new angle to up");

    _bLockEnvironment = true;
    _InitGUI(bCreateStatusBar, bCreateMenu);
    _bUpdateEnvironment = true;
}

QtOSGViewer::~QtOSGViewer()
{
    RAVELOG_DEBUG("destroying qtosg viewer\n");
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

    _posgWidget = new ViewerWidget(GetEnv());
    setCentralWidget(_posgWidget);

    // initialize the environment
    _ivRoot = new osg::Group();
    _ivRoot->ref();

    _ivFigureRoot = new osg::Group();   
    _ivRoot->addChild(_ivFigureRoot);

    // create world axis

    _ivWorldAxis = new osg::MatrixTransform();
    _posgWidget->GetCameraHUD()->addChild(_ivWorldAxis);

    {
        osg::Vec4f colors[] = {
            osg::Vec4f(0,0,1,1),
            osg::Vec4f(0,1,0,1),
            osg::Vec4f(1,0,0,1)
        };
        osg::Quat rotations[] = {
            osg::Quat(0, osg::Vec3f(0,0,1)),
            osg::Quat(-M_PI/2.0, osg::Vec3f(1,0,0)),
            osg::Quat(M_PI/2.0, osg::Vec3f(0,1,0))
        };

        // add 3 cylinder+cone axes
        for(int i = 0; i < 3; ++i) {
            osg::MatrixTransform* psep = new osg::MatrixTransform();
            psep->setMatrix(osg::Matrix::translate(-16.0f,-16.0f,-16.0f));

            // set a diffuse color
            osg::StateSet* state = psep->getOrCreateStateSet();
            osg::Material* mat = new osg::Material;
            mat->setDiffuse(osg::Material::FRONT, colors[i]);
            mat->setAmbient(osg::Material::FRONT, colors[i]);
            state->setAttribute( mat );

            osg::Matrix matrix;
            osg::MatrixTransform* protation = new osg::MatrixTransform();
            matrix.makeRotate(rotations[i]);
            protation->setMatrix(matrix);

            matrix.makeIdentity();
            osg::MatrixTransform* pcyltrans = new osg::MatrixTransform();
            matrix.setTrans(osg::Vec3f(0,0,16.0f));
            pcyltrans->setMatrix(matrix);

            // make SoCylinder point towards z, not y
            osg::Cylinder* cy = new osg::Cylinder();
            cy->setRadius(2.0f);
            cy->setHeight(32.0f);
            osg::ref_ptr<osg::Geode> gcyl = new osg::Geode;
            osg::ref_ptr<osg::ShapeDrawable> sdcyl = new osg::ShapeDrawable(cy);
            gcyl->addDrawable(sdcyl.get());

            osg::Cone* cone = new osg::Cone();
            cone->setRadius(4.0f);
            cone->setHeight(16.0f);

            osg::ref_ptr<osg::Geode> gcone = new osg::Geode;
            osg::ref_ptr<osg::ShapeDrawable> sdcone = new osg::ShapeDrawable(cone);
            gcone->addDrawable(sdcone.get());

            matrix.makeIdentity();
            osg::MatrixTransform* pconetrans = new osg::MatrixTransform();
            matrix.setTrans(osg::Vec3f(0,0,32.0f));
            pconetrans->setMatrix(matrix);

            psep->addChild(protation);
            protation->addChild(pcyltrans);
            pcyltrans->addChild(gcyl.get());
            protation->addChild(pconetrans);
            pconetrans->addChild(gcone.get());
            _ivWorldAxis->addChild(psep);
        }
    }

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
    _bTimeElapsed = false;
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

    printAct = new QAction(tr("Print Debug Output"), this);

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

    pauseAct = new QAction(QIcon(":/images/pause.png"),tr("Pause"), this);

    puntAct = new QAction(QIcon(":/images/no_edit.png"),tr("Pointer"), this);

//    AxesAct = new QAction(QIcon(":/images/axes.png"),tr("Axes"), this);
//    AxesAct->setCheckable(true);
//    connect(AxesAct, SIGNAL(triggered()), this, SLOT(axes()));

    houseAct = new QAction(QIcon(":/images/house.png"),tr("Home"), this);
    connect(houseAct, SIGNAL(triggered()),this,SLOT(ResetViewToHome()));

    smoothAct = new QAction(QIcon(":/images/smooth.png"),tr("Smooth"), this);
    connect(smoothAct, SIGNAL(triggered()), this, SLOT(polygonMode()));

    flatAct = new QAction(QIcon(":/images/flat.png"),tr("Flat"), this);
    connect(flatAct, SIGNAL(triggered()), this, SLOT(polygonMode()));

    lightAct = new QAction(QIcon(":/images/lighton.png"),tr("Light"), this);
    lightAct->setCheckable(true);
    connect(lightAct, SIGNAL(triggered()), this, SLOT(_ProcessLightChange()));

    wireAct = new QAction(QIcon(":/images/wire.png"),tr("Wire"), this);
    connect(wireAct, SIGNAL(triggered()), this, SLOT(polygonMode()));

    facesAct = new QAction(QIcon(":/images/faces.png"),tr("Cull face"), this);
    facesAct->setCheckable(true);
    connect(facesAct, SIGNAL(triggered()), this, SLOT(_ProcessFacesModeChange()));

    bboxAct = new QAction(QIcon(":/images/bbox.png"),tr("Poligon"), this);
    bboxAct->setCheckable(true);
    connect(bboxAct, SIGNAL(triggered()), this, SLOT(_ProcessBoundingBox()));

    connect(&_timer, SIGNAL(timeout()), this, SLOT(_Refresh()));
    _timer.start(1000/60); // ms
}

void QtOSGViewer::_Refresh()
{
    _UpdateEnvironment(1.0/60.0);
    //UpdateFromModel();
    //  _posgWidget->update();

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

    if (objectTree != NULL) {
        objectTree->clear();
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
    viewMenu->addAction(printAct);
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
    QString s = QFileDialog::getOpenFileName( this, "Load Environment", NULL,
                                              "Env Files (*.xml);;COLLADA Files (*.dae)");

    if( s.length() == 0 )
        return;

    _Reset();
    GetEnv()->Reset();

    GetEnv()->Load(s.toAscii().data());

    RAVELOG_INFO("\n---------Refresh--------\n");

    //  Refresh the screen.
    UpdateFromModel();

    RAVELOG_INFO("----> set home <----\n");

    //  Center object in window viewer
    _posgWidget->SetHome();
}

void QtOSGViewer::ImportEnvironment()
{
    QString s = QFileDialog::getOpenFileName( this, "Import Environment", NULL,
                                              "Env Files (*.xml);;COLLADA Files (*.dae)");

    if( s.length() == 0 )
        return;

    GetEnv()->Load(s.toAscii().data());

    //  Refresh the screen.
    UpdateFromModel();
}

void QtOSGViewer::SaveEnvironment()
{
    QString s = QFileDialog::getSaveFileName( this, "Save Environment", NULL,
                                              "Env Files (*.xml);;COLLADA Files (*.dae)");

    if( s.length() == 0 )
        return;

    GetEnv()->Save(s.toAscii().data());
}

void QtOSGViewer::ResetViewToHome()
{
    _posgWidget->ResetViewToHome();
}

void QtOSGViewer::_ProcessLightChange()
{
    if (lightAct->isChecked()) {
        lightAct->setIcon(QIcon(":/images/lightoff.png"));
    }
    else {
        lightAct->setIcon(QIcon(":/images/lighton.png"));
    }
    _posgWidget->SetLight(!lightAct->isChecked());
}

void QtOSGViewer::_ProcessFacesModeChange()
{
    _posgWidget->SetFacesMode(!facesAct->isChecked());
}

void QtOSGViewer::keyPressEvent(QKeyEvent* event)
{
    RAVELOG_INFO("key pressed event\n");
    QMainWindow::keyPressEvent(event);
}

void QtOSGViewer::keyReleaseEvent(QKeyEvent* event)
{
    RAVELOG_INFO("key released event\n");
    QMainWindow::keyReleaseEvent(event);
}

void QtOSGViewer::polygonMode()
{
    if (smoothAct->isChecked())
    {
        _posgWidget->setPolygonMode(0);
    }
    else if (flatAct->isChecked())
    {
        _posgWidget->setPolygonMode(1);
    }
    else
    {
        _posgWidget->setPolygonMode(2);
    }
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
        _posgWidget->SetDraggerMode("RotateCylinderDragger");
        _posgWidget->SelectActive(true);
        break;
    case -3:
        _posgWidget->SetDraggerMode("TrackballDragger");
        _posgWidget->SelectActive(true);
        break;
    case -4:
        _posgWidget->SetDraggerMode("TranslateAxisDragger");
        _posgWidget->SelectActive(true);
        break;
    default:
        RAVELOG_ERROR_FORMAT("pointerGroupClicked failure. Button %d pushed", button);
        _posgWidget->SelectActive(false);
        break;
    }
}

void QtOSGViewer::_ProcessDraggerGroupClicked(int button)
{
    RAVELOG_INFO_FORMAT("Dragger button clicked %d", button);
}

void QtOSGViewer::_RepaintWidgets(osg::ref_ptr<osg::Group> group)
{
    _posgWidget->SetSceneData(group);
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
    pointerButton->setChecked(false);
    pointerButton->setIcon(QIcon(":/images/pointer.png"));

    QToolButton *handButton = new QToolButton;
    handButton->setCheckable(true);
    handButton->setIcon(QIcon(":/images/hand.png"));
    
    QToolButton *axesButton = new QToolButton;
    axesButton->setCheckable(true);
    axesButton->setIcon(QIcon(":/images/axes.png"));

    pointerTypeGroup = new QButtonGroup;
    pointerTypeGroup->addButton(pointerButton);
    pointerTypeGroup->addButton(handButton);
    pointerTypeGroup->addButton(axesButton);

    connect(pointerTypeGroup, SIGNAL(buttonClicked(int)), this, SLOT(_ProcessPointerGroupClicked(int)));
    
    shapeGroup = new QActionGroup(this);
    smoothAct->setCheckable(true);
    flatAct->setCheckable(true);
    wireAct->setCheckable(true);
    shapeGroup->addAction(smoothAct);
    shapeGroup->addAction(flatAct);
    shapeGroup->addAction(wireAct);
    smoothAct->setChecked(true);


    toolsBar->addWidget(pointerButton);
    toolsBar->addWidget(handButton);
    toolsBar->addWidget(axesButton);
    toolsBar->addAction(houseAct);
    toolsBar->addAction(lightAct);
    toolsBar->addAction(smoothAct);
    toolsBar->addAction(flatAct);
    toolsBar->addAction(wireAct);
    toolsBar->addAction(facesAct);
}

void QtOSGViewer::_CreateStatusBar()
{
    statusBar()->showMessage(tr("Ready"));
}

void QtOSGViewer::_CreateDockWidgets()
{
    QDockWidget *dock = new QDockWidget(tr("Objects Tree"), this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    objectTree = _CreateObjectTree();

    dock->setWidget(objectTree);

    addDockWidget(Qt::RightDockWidgetArea, dock);
    viewMenu->addAction(dock->toggleViewAction());


    dock = new QDockWidget(tr("Details"), this);

    //  QListWidget *sensorList = new QListWidget(dock);

    detailsTree = new QTreeWidget();
    detailsTree->setHeaderLabel(QString("Properties"));
    dock->setWidget(detailsTree);

    addDockWidget(Qt::RightDockWidgetArea, dock);
    viewMenu->addAction(dock->toggleViewAction());
}

void QtOSGViewer::_OnObjectTreeClick(QTreeWidgetItem* item,int num)
{
    RobotBasePtr robot;
    KinBodyPtr kinbody;
    KinBody::LinkPtr link;

    std::string mass;

    //  Select robot in Viewers
    _posgWidget->SelectRobot(item->text(0).toAscii().data());

    //  Clears details
    if (detailsTree != NULL) {
        detailsTree->clear();
    }

    QList<QTreeWidgetItem*> items;

    if (!!item->parent()) {
        if (item->parent()->text(0) == "Links") {
            std::ostringstream strs;

            //  Set Title
            if (detailsTree != NULL) {
                detailsTree->setHeaderLabel(item->text(0).toAscii().data());
            }

            robot = GetEnv()->GetRobot(item->parent()->parent()->text(0).toAscii().data());
            link  = robot->GetLink(item->text(0).toAscii().data());

            //  Clears output string
            strs.clear();

            strs << link->GetMass();

            mass = string(" Mass= ") + strs.str();

            items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(mass.c_str()))));
        }
        else {
            //  Set Title
            if (detailsTree != NULL) {
                detailsTree->setHeaderLabel(item->text(0).toAscii().data());
            }
        }
    }
    else {
        //  Set Title
        if (detailsTree != NULL) {
            detailsTree->setHeaderLabel(item->text(0).toAscii().data());
        }
        kinbody = GetEnv()->GetKinBody(item->text(0).toAscii().data());
        for (size_t i=0; i<kinbody->GetLinks().size(); i++) {
            std::ostringstream strs;
            link = kinbody->GetLinks()[i];
            strs << link->GetMass();
            mass = link->GetName() + string(" Mass= ") + strs.str();
            items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(mass.c_str()))));
        }
    }

    //  Add Items to details panel
    if (detailsTree != NULL) {
        detailsTree->insertTopLevelItems(0, items);
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

    // set the viewport size correctly so we can draw stuff on the hud using window coordinates
    int width = centralWidget()->size().width();
    int height = centralWidget()->size().height();
    _posgWidget->SetViewport(width, height);

    // read the current camera transform
    osg::Vec3d eye, center, up;
    _posgWidget->GetCameraManipulator()->getTransformation(eye, center, up);
    osg::Vec3d lv(center - eye);

    // setup the world axis correctly
    osg::Matrix m = osg::Matrix::lookAt(eye, center, up);
    m.setTrans(width/2 - 40, -height/2 + 40, -50);
    _ivWorldAxis->setMatrix(m);

    _Tcamera = GetRaveTransformFromMatrix(osg::Matrix::inverse(osg::Matrix::lookAt(eye, center, up)));
    _focalDistance = lv.length();

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
            RaveVector<float> vDestPos = tTrack.trans + ExtractAxisFromQuat(vDestQuat,2)*_focalDistance;

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

            _SetCameraTransform();
        }
    }

    double fovy, aspectRatio, zNear, zFar;
    _posgWidget->GetCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

    _camintrinsics.fy = fovy;
    _camintrinsics.fx = aspectRatio * fovy;
    _camintrinsics.cx = _posgWidget->GetCamera()->getViewport()->width() / 2;
    _camintrinsics.cy = _posgWidget->GetCamera()->getViewport()->height() / 2;
    _camintrinsics.focal_length = zNear;
    _camintrinsics.distortion_model = "";

    /*
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
    */
}

bool QtOSGViewer::_SetFiguresInCamera(ostream& sout, istream& sinput)
{
    sinput >> _bRenderFiguresInCamera;
    return !!sinput;
}

bool QtOSGViewer::_SetFeedbackVisibility(ostream& sout, istream& sinput)
{
    sinput >> _bDisplayFeedBack;
    // ViewToggleFeedBack(_bDisplayFeedBack);
    return !!sinput;
}

bool QtOSGViewer::_SetNearPlaneCommand(ostream& sout, istream& sinput)
{
    dReal nearplane=0.01f;
    sinput >> nearplane;
    // EnvMessagePtr pmsg(new SetNearPlaneMessage(shared_viewer(), (void**)NULL, nearplane));
    // pmsg->callerexecute(false);
    return true;
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

bool QtOSGViewer::_TrackManipulatorCommand(ostream& sout, istream& sinput)
{
    bool bresetvelocity = true;
    std::string robotname, manipname;
    float focalDistance = 0.0;
    sinput >> robotname >> manipname >> focalDistance >> bresetvelocity;
    if( focalDistance > 0 ) {
        _SetCameraDistanceToFocus(focalDistance);
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

bool QtOSGViewer::_SetTrackingAngleToUpCommand(ostream& sout, istream& sinput)
{
    sinput >> _fTrackAngleToUp;
    return true;
}

int QtOSGViewer::main(bool bShow)
{
    if( !QApplication::instance() ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("need a valid QApplication before viewer loop is run", ORE_InvalidState);
    }
    if (bShow) {
        this->show();
    }

    UpdateFromModel();
    _posgWidget->SetHome();

    QApplication::instance()->exec();
    return 0;
}

void QtOSGViewer::quitmainloop()
{

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
    osg::Vec3d eye, center, up;
    osg::Matrix::inverse(GetMatrixFromRaveTransform(_Tcamera)).getLookAt(eye, center, up, _focalDistance);
    _posgWidget->GetCameraManipulator()->setTransformation(eye, center, up);
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

void QtOSGViewer::_Draw(osg::Switch *handle, osg::Vec3Array *vertices, osg::Vec4Array *colors, osg::PrimitiveSet::Mode mode, osg::StateAttribute *attribute)
{
    osg::ref_ptr<osg::Group> parent = new osg::Group;
    osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform;
    osg::ref_ptr<osg::Group> child = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;

    geometry->setVertexArray(vertices);
    vertices->unref();

    geometry->setColorArray(colors);
    colors->unref();

    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    geometry->addPrimitiveSet(new osg::DrawArrays(mode, 0, vertices->size()));
    geometry->getOrCreateStateSet()->setAttribute(attribute, osg::StateAttribute::ON);

    geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geometry->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);     
    geometry->getOrCreateStateSet()->setRenderBinDetails(0, "transparent"); 
    geometry->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN); 

    geode->addDrawable(geometry);

    child->addChild(geode);
    trans->addChild(child);
    parent->addChild(trans);
    handle->addChild(parent);
    _ivFigureRoot->addChild(handle);
    handle->unref();
}

GraphHandlePtr QtOSGViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
{
    osg::Switch* handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        vvertices->push_back(osg::Vec3(ppoints[0], ppoints[1], ppoints[2]));
        ppoints = (float*)((char*)ppoints + stride);

        vcolors->push_back(osg::Vec4f(color.x, color.y, color.z, color.w));        
    }

    handle->ref();
    vvertices->ref();
    vcolors->ref();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::POINTS, new osg::Point(fPointSize)));

    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle, bool bhasalpha)
{
    osg::Switch* handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        vvertices->push_back(osg::Vec3(ppoints[0], ppoints[1], ppoints[2]));
        ppoints = (float*)((char*)ppoints + stride);

        if (bhasalpha) {
            vcolors->push_back(osg::Vec4f(colors[i * 4 + 0], colors[i * 4 + 1], colors[i * 4 + 2], colors[i * 4 + 3]));
        } else {
            vcolors->push_back(osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f));
        }
    }
    
    handle->ref();
    vvertices->ref();
    vcolors->ref();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::POINTS, new osg::Point(fPointSize)));

    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    osg::Switch* handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        vvertices->push_back(osg::Vec3(ppoints[0], ppoints[1], ppoints[2]));
        ppoints = (float*)((char*)ppoints + stride);

        vcolors->push_back(osg::Vec4f(color.x, color.y, color.z, color.w));
    }

    handle->ref();
    vvertices->ref();
    vcolors->ref();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINE_STRIP, new osg::LineWidth(fwidth)));

    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}
GraphHandlePtr QtOSGViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    osg::Switch* handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        vvertices->push_back(osg::Vec3(ppoints[0], ppoints[1], ppoints[2]));
        ppoints = (float*)((char*)ppoints + stride);
        vcolors->push_back(osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f));
    }
    
    handle->ref();
    vvertices->ref();
    vcolors->ref();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINE_STRIP, new osg::LineWidth(fwidth)));

    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    osg::Switch* handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        vvertices->push_back(osg::Vec3(ppoints[0], ppoints[1], ppoints[2]));
        ppoints = (float*)((char*)ppoints + stride);

        vcolors->push_back(osg::Vec4f(color.x, color.y, color.z, color.w));
    }

    handle->ref();
    vvertices->ref();
    vcolors->ref();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINES, new osg::LineWidth(fwidth)));

    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}
GraphHandlePtr QtOSGViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    osg::Switch* handle = _CreateGraphHandle();

    osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    for(int i = 0; i < numPoints; ++i) {
        vvertices->push_back(osg::Vec3(ppoints[0], ppoints[1], ppoints[2]));
        ppoints = (float*)((char*)ppoints + stride);

        vcolors->push_back(osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f));
    }
    
    handle->ref();
    vvertices->ref();
    vcolors->ref();
    _PostToGUIThread(boost::bind(&QtOSGViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINES, new osg::LineWidth(fwidth)));

    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOSGViewer::drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
{
    return GraphHandlePtr();
}

GraphHandlePtr QtOSGViewer::drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
{
    return GraphHandlePtr();
}

GraphHandlePtr QtOSGViewer::drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
{
    return GraphHandlePtr();
}

GraphHandlePtr QtOSGViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
{
    return GraphHandlePtr();
}

GraphHandlePtr QtOSGViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
{
    return GraphHandlePtr();
}

void QtOSGViewer::_Deselect()
{

}

void QtOSGViewer::Reset()
{

}

void QtOSGViewer::SetBkgndColor(const RaveVector<float>& color)
{

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

void QtOSGViewer::SetName(const string& ptitle)
{
    setWindowTitle(ptitle.c_str());
}

bool QtOSGViewer::LoadModel(const string& filename)
{
    //    Debug
//  RAVELOG_WARN("QtOSGViewer::LoadModel(pfilename)\n");

    if( filename == "")
        return false;

    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile(filename);
    osg::Node *node = loadedModel.get();

    if (node != NULL) {
        GetRoot()->addChild(node);
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
                        pitem = boost::shared_ptr<RobotItem>(new RobotItem(shared_viewer(), boost::static_pointer_cast<RobotBase>(pbody), _viewGeometryMode), ITEM_DELETER);
                    }
                    else {
                        pitem = boost::shared_ptr<KinBodyItem>(new KinBodyItem(shared_viewer(), pbody, _viewGeometryMode), ITEM_DELETER);
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
                //_ivRoot->deselectAll();
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
    _RepaintWidgets(GetRoot());

    if (newdata) {
        if (objectTree != NULL) {
            //  Fill tree widget with robot and joints
            _FillObjectTree(objectTree);
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
    GUIThreadFunctionPtr pfn(new GUIThreadFunction(fn));
    boost::mutex::scoped_lock lockmsg(_mutexGUIFunctions);
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
        FOREACH(it,_listGUIFunctions) {
            (*it)->SetFinished();
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

osg::Switch* QtOSGViewer::_CreateGraphHandle()
{
    osg::Switch* handle = new osg::Switch;
    handle->setAllChildrenOn();
    return handle;
}

void QtOSGViewer::_CloseGraphHandle(osg::Switch* handle)
{
    _ivFigureRoot->removeChild(handle);
    handle->unref();
}

void QtOSGViewer::_SetGraphTransform(osg::Switch* handle, const RaveTransform<float> t)
{
    SetMatrixTransform(handle->getChild(0)->asGroup()->getChild(0)->asTransform()->asMatrixTransform(), t);
    handle->unref();
}

void QtOSGViewer::_SetGraphShow(osg::Switch* handle, bool bShow)
{
    if (bShow) {
        handle->setAllChildrenOn();
    } else {
        handle->setAllChildrenOff();
    }
    handle->unref();
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
