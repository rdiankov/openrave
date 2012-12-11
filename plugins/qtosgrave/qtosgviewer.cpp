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
#include "qtosgviewer.h"

namespace qtosgrave {

////////////////////////////////////////////////////////////////////////////////
/// Constructor
////////////////////////////////////////////////////////////////////////////////
QtOSGViewer::QtOSGViewer(EnvironmentBasePtr penv,QApplication *app) : application(app),
#if QT_VERSION >= 0x040000 // check for qt4
    QMainWindow(NULL, Qt::Window),
#else
    QMainWindow(NULL, "OpenRAVE", Qt::WType_TopLevel),
#endif
    ViewerBase(penv)
{
    _name = str(boost::format("OpenRAVE %s")%OPENRAVE_VERSION_STRING);
    if( (OPENRAVE_VERSION_MINOR%2) || (OPENRAVE_VERSION_PATCH%2) ) {
        _name += " (Development Version)";
    }
    else {
        _name += " (Stable Release)";
    }
#if QT_VERSION >= 0x040000 // check for qt4
    setWindowTitle(_name.c_str());
    statusBar()->showMessage(tr("Status Bar"));
#endif

    __description = ":Interface Author: Gustavo Puche\n\nProvides a viewer based on Open Scene Graph.";

    osg::ArgumentParser arguments(0, NULL);

    centralWidget = new QWidget;

    centralWidget->adjustSize();

    setCentralWidget(centralWidget);


    osgWidget = new ViewerWidget;

    // Sends environment to widget
    osgWidget->setEnv(penv);

    // initialize the environment
    _ivRoot = new osg::Group();
    _ivRoot->ref();

    tree = new QTreeView;

    createActions();
    createMenus();
    createToolsBar();
    createLayouts();
    createStatusBar();
    createDockWidgets();

    centralWidget->setLayout(centralLayout);

    resize(1024, 750);

    simpleView = true;

    // toggle switches
//     _nFrameNum = 0;
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
}

////////////////////////////////////////////////////////////////////////////////
/// Create layouts
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::createLayouts()
{
//  RAVELOG_VERBOSE("----->>>> createLayouts()\n");

    centralLayout = new QGridLayout;
    centralLayout->addWidget(osgWidget,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Actions that achieve buttons and menus
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::createActions()
{
//  RAVELOG_VERBOSE("----->>>> createActions()\n");

    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcut(tr("Ctrl+Q"));
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    loadAct = new QAction(QIcon(":/images/open.png"),tr("L&oad..."), this);
    loadAct->setShortcut(tr("Ctrl+L"));
    connect(loadAct, SIGNAL(triggered()), this, SLOT(LoadEnvironment()));

    multiAct = new QAction(tr("M&ultiView"), this);
    multiAct->setShortcut(tr("Ctrl+M"));
    connect(multiAct, SIGNAL(triggered()), this, SLOT(multiWidget()));

    simpleAct = new QAction(tr("S&impleView"), this);
    simpleAct->setShortcut(tr("Ctrl+1"));
    connect(simpleAct, SIGNAL(triggered()), this, SLOT(simpleWidget()));

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

    AxesAct = new QAction(QIcon(":/images/axes.png"),tr("Axes"), this);
    AxesAct->setCheckable(true);
    connect(AxesAct, SIGNAL(triggered()), this, SLOT(axes()));

    houseAct = new QAction(QIcon(":/images/house.png"),tr("Home"), this);
    connect(houseAct, SIGNAL(triggered()),this,SLOT(home()));

    smoothAct = new QAction(QIcon(":/images/smooth.png"),tr("Smooth"), this);
    connect(smoothAct, SIGNAL(triggered()), this, SLOT(polygonMode()));

    flatAct = new QAction(QIcon(":/images/flat.png"),tr("Flat"), this);
    connect(flatAct, SIGNAL(triggered()), this, SLOT(polygonMode()));

    lightAct = new QAction(QIcon(":/images/lighton.png"),tr("Light"), this);
    lightAct->setCheckable(true);
    connect(lightAct, SIGNAL(triggered()), this, SLOT(light()));

    wireAct = new QAction(QIcon(":/images/wire.png"),tr("Wire"), this);
    connect(wireAct, SIGNAL(triggered()), this, SLOT(polygonMode()));

    facesAct = new QAction(QIcon(":/images/faces.png"),tr("Cull face"), this);
    facesAct->setCheckable(true);
    connect(facesAct, SIGNAL(triggered()), this, SLOT(facesMode()));

    bboxAct = new QAction(QIcon(":/images/bbox.png"),tr("Poligon"), this);
    bboxAct->setCheckable(true);
    connect(bboxAct, SIGNAL(triggered()), this, SLOT(boundingBox()));

    connect(&_timer, SIGNAL(timeout()), this, SLOT(refresh()));
    _timer.start(10);
}

////////////////////////////////////////////////////////////////////////////////
//  Refresh the screen with a new frame. Reads the scene from OpenRAVE Core.
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::refresh(){

//  RAVELOG_WARN("\t\t QtOSGViewer::refresh()\n\n");

//  _Reset();


    UpdateFromModel();


//  osgWidget->update();
}

////////////////////////////////////////////////////////////////////////////////
/// Reset update from model
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::_Reset()
{
    //    Debug
//  RAVELOG_INFO("\t\t QtOSGViewer::_Reset()\n");

//  deselect();

//  RAVELOG_INFO("Number of bodies mapped = %d\n",_mapbodies.size());

    FOREACH(itbody, _mapbodies)
    {
        BOOST_ASSERT( itbody->first->GetUserData("qtosg") == itbody->second );

        //  Clear Gui Data

        //  Modified for OpenRAVE 0.5v
        itbody->first->RemoveUserData("qtosg");
    }

    GetEnv()->UpdatePublishedBodies();

    _mapbodies.clear();

    objectTree->clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Create menus
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::createMenus()
{
//  RAVELOG_VERBOSE("----->>>> createMenus()\n");

    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(loadAct);
    fileMenu->addAction(importAct);
    fileMenu->addAction(saveAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    viewMenu = menuBar()->addMenu(tr("&View"));
    viewMenu->addAction(multiAct);
    viewMenu->addAction(simpleAct);

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

////////////////////////////////////////////////////////////////////////////////
/// Future use
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::createScrollArea()
{

}

////////////////////////////////////////////////////////////////////////////////
/// Menu items
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::LoadEnvironment()
{
    //    Debug
//  RAVELOG_WARN("----->>>> LoadEnvironment()\n");

#if QT_VERSION >= 0x040000 // check for qt4
    QString s = QFileDialog::getOpenFileName( this, "Load Environment", NULL,
                                              "Env Files (*.xml);;COLLADA Files (*.dae)");

    if( s.length() == 0 )
        return;

    _Reset();
    GetEnv()->Reset();

    GetEnv()->Load(s.toAscii().data());


    //  Debug
    RAVELOG_INFO("\n---------Refresh--------\n");

    //


    //  Refresh the screen.
    UpdateFromModel();

    RAVELOG_INFO("----> set home <----\n");

    //  Center object in window viewer
    osgWidget->setHome();

#endif
}

////////////////////////////////////////////////////////////////////////////////
/// Adds models to current scene
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// Saves the environment into a file
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::SaveEnvironment()
{
    QString s = QFileDialog::getSaveFileName( this, "Save Environment", NULL,
                                              "Env Files (*.xml);;COLLADA Files (*.dae)");

    if( s.length() == 0 )
        return;

    GetEnv()->Save(s.toAscii().data());
}

////////////////////////////////////////////////////////////////////////////////
/// Set model home position in viewer
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::home()
{
    osgWidget->home();
}

////////////////////////////////////////////////////////////////////////////////
/// Sets On/Off light
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::light()
{
//  RAVELOG_VERBOSE("----->>>> SLOT light()\n");

    if (lightAct->isChecked())
    {
//    RAVELOG_DEBUG("Switch OFF light \n");
        lightAct->setIcon(QIcon(":/images/lightoff.png"));
        osgWidget->setLight(false);
    }
    else
    {
//    RAVELOG_DEBUG("Switch ON light \n");
        lightAct->setIcon(QIcon(":/images/lighton.png"));
        osgWidget->setLight(true);
    }
}

////////////////////////////////////////////////////////////////////////////////
//  Sets COUNTER CLOCKWISE and CLOCKWISE polygons
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::facesMode()
{
//  RAVELOG_VERBOSE("----->>>> SLOT facesMode()\n");

    if (facesAct->isChecked())
    {
        osgWidget->setFacesMode(false);
    }
    else
    {
        osgWidget->setFacesMode(true);
    }
}

void QtOSGViewer::polygonMode()
{
    if (smoothAct->isChecked())
    {
        osgWidget->setPolygonMode(0);
    }
    else if (flatAct->isChecked())
    {
        osgWidget->setPolygonMode(1);
    }
    else
    {
        osgWidget->setPolygonMode(2);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Sets or reset bounding box
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::boundingBox()
{
    osgWidget->drawBoundingBox(bboxAct->isChecked());
}

////////////////////////////////////////////////////////////////////////////////
/// Sets or reset axes
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::axes()
{
    osgWidget->drawAxes(AxesAct->isChecked());
}

////////////////////////////////////////////////////////////////////////////////
/// Event clicked of pointer, hand, bound and axes buttons
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::pointerGroupClicked(int button)
{
    switch (button) {
    case -2:
        osgWidget->drawTrackball(false);
        osgWidget->select(true);
        break;
    case -3:
        osgWidget->drawTrackball(true);
        osgWidget->select(false);
        break;
    case -4:
        osgWidget->drawBoundingBox(true);
        osgWidget->select(false);
        break;
    case -5:
        osgWidget->drawAxes(true);
        osgWidget->select(false);
        break;
    default:
        RAVELOG_ERROR("pointerGroupClicked failure. Button %d pushed\n",button);
        osgWidget->select(false);
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Event clicked of bound and axes button
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::draggerGroupClicked(int button)
{
    RAVELOG_INFO("Dragger button clicked %d\n",button);
}

////////////////////////////////////////////////////////////////////////////////
/// Multi Widget selection
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::multiWidget()
{
    osgWidget->setMultipleView();

    repaintWidgets(GetRoot());
}

void QtOSGViewer::simpleWidget()
{
    osgWidget->setSimpleView();

    repaintWidgets(GetRoot());
}

////////////////////////////////////////////////////////////////////////////////
/// Repaint widgets
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::repaintWidgets(osg::Group* group)
{
    osgWidget->setSceneData(group);
}

////////////////////////////////////////////////////////////////////////////////
/// Set Buttons and Icons in the ToolBar
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::createToolsBar()
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
    pointerButton->setIcon(QIcon(":/images/pointer.png"));

    QToolButton *handButton = new QToolButton;
    handButton->setCheckable(true);
    handButton->setIcon(QIcon(":/images/hand.png"));

    QToolButton *boundButton = new QToolButton;
    boundButton->setCheckable(true);
    boundButton->setIcon(QIcon(":/images/bbox.png"));

    QToolButton *axesButton = new QToolButton;
    axesButton->setCheckable(true);
    axesButton->setIcon(QIcon(":/images/axes.png"));

    pointerTypeGroup = new QButtonGroup;
    pointerTypeGroup->addButton(pointerButton);
    pointerTypeGroup->addButton(handButton);
    pointerTypeGroup->addButton(boundButton);
    pointerTypeGroup->addButton(axesButton);

    connect(pointerTypeGroup, SIGNAL(buttonClicked(int)), this, SLOT(pointerGroupClicked(int)));

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
    toolsBar->addWidget(boundButton);
    toolsBar->addWidget(axesButton);
    toolsBar->addAction(houseAct);
    toolsBar->addAction(lightAct);
    toolsBar->addAction(smoothAct);
    toolsBar->addAction(flatAct);
    toolsBar->addAction(wireAct);
    toolsBar->addAction(facesAct);
}

////////////////////////////////////////////////////////////////////////////////
/// Create StatusBar and Set a Message
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::createStatusBar()
{
    statusBar()->showMessage(tr("Ready"));
}

////////////////////////////////////////////////////////////////////////////////
/// Create info panels
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::createDockWidgets()
{
    //    Debug
//  RAVELOG_VERBOSE("----->>>> createDockWidgets()\n");

    QDockWidget *dock = new QDockWidget(tr("Objects Tree"), this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    objectTree = createObjectTree();

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

////////////////////////////////////////////////////////////////////////////////
///  TODO : Slot to listen scene list events
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::sceneListClicked(QTreeWidgetItem* item,int num)
{
    RobotBasePtr robot;
    KinBodyPtr kinbody;
    KinBody::LinkPtr link;

    std::string mass;

    //  Select robot in Viewers
    osgWidget->selectRobot(item->text(0).toAscii().data());

    //  Clears details
    detailsTree->clear();

    QList<QTreeWidgetItem*> items;

    if (!!item->parent())
    {
        if (item->parent()->text(0) == "Links")
        {
            std::ostringstream strs;

            //  Set Title
            detailsTree->setHeaderLabel(item->text(0).toAscii().data());

            robot = GetEnv()->GetRobot(item->parent()->parent()->text(0).toAscii().data());
            link  = robot->GetLink(item->text(0).toAscii().data());

            //  Clears output string
            strs.clear();

            strs << link->GetMass();

            mass = string(" Mass= ") + strs.str();

            items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(mass.c_str()))));
        }
        else
        {
            //  Set Title
            detailsTree->setHeaderLabel(item->text(0).toAscii().data());
        }
    }
    else
    {
        //  Set Title
        detailsTree->setHeaderLabel(item->text(0).toAscii().data());

        kinbody = GetEnv()->GetKinBody(item->text(0).toAscii().data());

        for (size_t i=0; i<kinbody->GetLinks().size(); i++)
        {
            std::ostringstream strs;

            link = kinbody->GetLinks()[i];

            strs << link->GetMass();

            mass = link->GetName() + string(" Mass= ") + strs.str();

            items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(mass.c_str()))));
        }
    }

    //  Add Items to details panel
    detailsTree->insertTopLevelItems(0, items);
}

////////////////////////////////////////////////////////////////////////////////
///  TODO : Create the Object tree widget
////////////////////////////////////////////////////////////////////////////////
QTreeWidget* QtOSGViewer::createObjectTree()
{
    QTreeWidget *treeWidget = new QTreeWidget();
    treeWidget->setColumnCount(1);

    treeWidget->setHeaderLabel(QString("Scene"));

    //  Connect scene list clicked
    connect(treeWidget,SIGNAL(itemClicked(QTreeWidgetItem*,int)), this, SLOT(sceneListClicked(QTreeWidgetItem*,int)));

    return treeWidget;
}

////////////////////////////////////////////////////////////////////////////////
///  Fills object tree with robot info
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::fillObjectTree(QTreeWidget *treeWidget)
{
    //  Debug
    RAVELOG_DEBUG("Begin fillObjectTree....\n");

    RobotBase* robot;

    vector<KinBodyPtr> kinbodies;

    QList<QTreeWidgetItem*> items;

    //  Clears tree
    treeWidget->clear();

    GetEnv()->GetBodies(kinbodies);

    for (size_t i = 0; i < kinbodies.size(); i++)
    {
        items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(kinbodies[i]->GetName().c_str()))));

        //  Number of child to add
        size_t nchild = 0;

        vector<KinBody::LinkPtr> links = kinbodies[i]->GetLinks();
        vector<KinBody::JointPtr>  joints = kinbodies[i]->GetJoints();

        //  Header 'Links'
        items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Links"))));

        for (size_t j = 0; j < links.size(); j++)
        {
            items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(links[j]->GetName().c_str()))));
        }

        if (joints.size() > 0)
        {
            //  Header 'Joints'
            items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Joints"))));

            nchild++;
        }

        for (size_t j = 0; j < joints.size(); j++)
        {
            items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(joints[j]->GetName().c_str()))));

            //  Adds links of joints
            items[i]->child(nchild)->child(j)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(joints[j]->GetFirstAttached()->GetName().c_str()))));
            items[i]->child(nchild)->child(j)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(joints[j]->GetSecondAttached()->GetName().c_str()))));
        }

        if (kinbodies[i]->IsRobot())
        {
            robot = (RobotBase*)kinbodies[i].get();

            vector<RobotBase::ManipulatorPtr>     manipulators;
            vector<RobotBase::AttachedSensorPtr>  sensors;
            ControllerBasePtr controller;

            sensors = robot->GetAttachedSensors();
            manipulators  = robot->GetManipulators();
            controller  = robot->GetController();

            //  Debug
            RAVELOG_DEBUG("Sensors....\n");

            if (sensors.size() > 0)
            {
                //  Header 'Sensors'
                items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Sensors"))));

                nchild++;
            }

            //  Debug
            RAVELOG_DEBUG("Sensors number=%d\n",sensors.size());

            for (size_t j = 0; j < sensors.size(); j++)
            {
                RAVELOG_INFO("Sensor name=%s\n",sensors[j]->GetName().c_str());

                items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(sensors[j]->GetName().c_str()))));

                RAVELOG_WARN("Sensor link=%s\n",sensors[j]->GetAttachingLink()->GetName().c_str());

                items[i]->child(nchild)->child(j)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(sensors[j]->GetAttachingLink()->GetName().c_str()))));
            }

            //  Debug
            RAVELOG_DEBUG("Manipulators....\n");

            if (manipulators.size() > 0)
            {
                //  Header 'Manipulators'
                items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Manipulators"))));

                nchild++;
            }

            for (size_t j = 0; j < manipulators.size(); j++)
            {
                items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(manipulators[j]->GetName().c_str()))));
            }

            //  Debug
            RAVELOG_DEBUG("Controller....\n");

            if (!!controller)
            {
                //  Header 'Controller'
                items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Controller"))));

                nchild++;

                items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(controller->GetXMLFilename().c_str()))));
            }
        }
    }

    treeWidget->insertTopLevelItems(0, items);

    //  Debug
    RAVELOG_DEBUG("End fillObjectTree....\n");
}

void QtOSGViewer::mouseDoubleClickEvent(QMouseEvent *e)
{
    std::cout << "Press mouse: doubleClick" << std::endl;
    std::cout << e->button() << std::endl;
}

void QtOSGViewer::setSimpleView(bool state)
{
    simpleView = state;
}
bool QtOSGViewer::isSimpleView()
{
    return simpleView;
}

///////////////////////////////////////
//	 RaveViewerBase interface	       //
///////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// Reset the camera depending on its mode
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::UpdateCameraTransform()
{
//  // set the camera depending on its mode
//
//  // get the camera
//  MutexLock m(&_mutexMessages);
//  SbVec3f pos = GetCamera()->position.getValue();
//
//  Tcam.trans = RaveVector<float>(pos[0], pos[1], pos[2]);
//
//  SbVec3f axis;
//  float fangle;
//  GetCamera()->orientation.getValue(axis, fangle);
//  Tcam.rotfromaxisangle(RaveVector<float>(axis[0],axis[1],axis[2]),fangle);
}

////////////////////////////////////////////////////////////////////////////////
/// Goes into the main loop
////////////////////////////////////////////////////////////////////////////////
int QtOSGViewer::main(bool bShow)
{
//  RAVELOG_VERBOSE("----->>>> main(bool bShow)\n");

    if (bShow)
    {
        this->show();
    }

    UpdateFromModel();

    //  Center model in viewer windows
    osgWidget->setHome();

    application->exec();
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Destroys the main loop
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::quitmainloop()
{

}

bool QtOSGViewer::GetFractionOccluded(KinBodyPtr pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded)
{
    return false;
}

////////////////////////////////////////////////////////////////////////////////
/// Retries a 24bit RGB image of dimensions width and height from the current scene
/// extrinsic is the rotation and translation of the camera
/// pKK is 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
////////////////////////////////////////////////////////////////////////////////
bool QtOSGViewer::GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK)
{
    return false;
}
bool QtOSGViewer::WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension)
{
    return false;
}
void QtOSGViewer::SetCamera(const RaveTransform<float>& trans, float focalDistance)
{
//  RAVELOG_INFO("[SetCamera]\n");
}
//void QtOSGViewer::SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup)
//{
//  RAVELOG_INFO("[SetCameraLookAt]\n");
//}

RaveTransform<float> QtOSGViewer::GetCameraTransform()
{
    RaveTransform<float> t;
    return t;
}

GraphHandlePtr QtOSGViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
{
    return GraphHandlePtr();
}

GraphHandlePtr QtOSGViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle, bool bhasalpha)
{
    return GraphHandlePtr();
}

GraphHandlePtr QtOSGViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    return GraphHandlePtr();
}
GraphHandlePtr QtOSGViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    return GraphHandlePtr();
}

GraphHandlePtr QtOSGViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    return GraphHandlePtr();
}
GraphHandlePtr QtOSGViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    return GraphHandlePtr();
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
//    void* pret = new SoSeparator();
//    EnvMessagePtr pmsg(new DrawPlaneMessage(shared_viewer(), (SoSeparator*)pret, tplane,vextents,vtexture));
//    pmsg->callerexecute();

    return GraphHandlePtr();
}


GraphHandlePtr QtOSGViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
{
    return GraphHandlePtr();
}

GraphHandlePtr QtOSGViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
{


    //    void* pret = new SoSeparator();
    //    EnvMessagePtr pmsg(new DrawTriMeshColorMessage(shared_viewer(), (SoSeparator*)pret, ppoints, stride, pIndices, numTriangles, colors));
    //    pmsg->callerexecute();

    return GraphHandlePtr();
}

void QtOSGViewer::closegraph(void* handle)
{

}

void QtOSGViewer::deselect()
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
    RAVELOG_VERBOSE("----->>>> ViewerSetSize(int w, int h)\n");

}
void QtOSGViewer::Move(int x, int y)
{
    RAVELOG_VERBOSE("----->>>> ViewerMove(int x, int y)\n");
}

////////////////////////////////////////////////////////////////////////////////
/// Set title of the viewer window
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::SetName(const string& ptitle)
{
    setWindowTitle(ptitle.c_str());
}

////////////////////////////////////////////////////////////////////////////////
/// Reads model from file and loads it in viewer
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// Updates all render objects from the internal openrave classes
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::UpdateFromModel()
{
    bool newdata;

    _listRemoveItems.clear();

    vector<KinBody::BodyState> vecbodies;

    GetEnv()->GetPublishedBodies(vecbodies);

    newdata = false;

    FOREACH(it, _mapbodies)
    {
        it->second->SetUserData(0);
    }

    FOREACH(itbody, vecbodies)
    {
        BOOST_ASSERT( !!itbody->pbody );
        KinBodyPtr pbody = itbody->pbody; // try to use only as an id, don't call any methods!
        KinBodyItemPtr pitem = boost::dynamic_pointer_cast<KinBodyItem>(pbody->GetUserData("qtosg"));

        //  if pitem is FALSE
        if( !pitem )
        {
            // create a new body
            // make sure pbody is actually present
            if( GetEnv()->GetBodyFromEnvironmentId(itbody->environmentid) == pbody ) {

                // check to make sure the real GUI data is also NULL
                if( !pbody->GetUserData("qtosg") )
                {
                    if( _mapbodies.find(pbody) != _mapbodies.end() )
                    {
                        continue;
                    }

                    if( pbody->IsRobot() )
                        pitem = boost::shared_ptr<RobotItem>(new RobotItem(shared_viewer(), boost::static_pointer_cast<RobotBase>(pbody), _viewGeometryMode));
                    else
                        pitem = boost::shared_ptr<KinBodyItem>(new KinBodyItem(shared_viewer(), pbody, _viewGeometryMode));

                    newdata = true;

                    //  Adds pitem to list of items for remove
                    _listRemoveItems.push_back(pitem.get());

                    pitem->Load();

                    //  Modified for OpenRAVE 0.5v
                    pbody->SetUserData("qtosg",pitem);

                    _mapbodies[pbody] = pitem;
                }
                else {
                    pitem = boost::static_pointer_cast<KinBodyItem>(pbody->GetUserData("qtosg"));
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

        //  TODO : Revise
        BOOST_ASSERT( pitem->GetBody() == pbody);
        BOOST_ASSERT( itmap->second == pitem );

        pitem->SetUserData(1);

        //  Update viewer with core transforms
        pitem->UpdateFromModel(itbody->jointvalues,itbody->vectrans);
    }

    //  Repaint the scene created
    repaintWidgets(GetRoot());

    if (newdata)
    {
        //  Fill tree widget with robot and joints
        fillObjectTree(objectTree);
    }

    FOREACH_NOINC(it, _mapbodies)
    {
        if(!it->second->GetUserData())
        {
            //  Modified for OpenRAVE 0.5v
            it->first->RemoveUserData("qtosg");
            _mapbodies.erase(it++);
        }
        else
            ++it;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Locks environment
////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<EnvironmentMutex::scoped_try_lock> QtOSGViewer::LockEnvironment(uint64_t timeout,bool bUpdateEnvironment)
{
    // try to acquire the lock
//#if BOOST_VERSION >= 103500
    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),boost::defer_lock_t()));
//#else
//    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),false));
//#endif
    uint64_t basetime = utils::GetMicroTime();
    while(utils::GetMicroTime()-basetime<timeout ) {
        lockenv->try_lock();
        if( !!*lockenv )
            break;
        if( bUpdateEnvironment )
            _UpdateEnvironment();
    }

    if( !*lockenv )
        lockenv.reset();
    return lockenv;
}

////////////////////////////////////////////////////////////////////////////////
/// Update model and camera transform
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::_UpdateEnvironment()
{
//    RAVELOG_INFO("\t_UpdateEnvironment\n\n");
    boost::mutex::scoped_lock lockupd(_mutexUpdating);

    if( _bUpdateEnvironment ) {
        // process all messages
        list<EnvMessagePtr> listmessages;
        {
            boost::mutex::scoped_lock lockmsg(_mutexMessages);
            listmessages.swap(_listMessages);
            BOOST_ASSERT( _listMessages.size() == 0 );
        }

        FOREACH(itmsg, listmessages)
            (*itmsg)->viewerexecute();

        // have to update model after messages since it can lock the environment
        UpdateFromModel();
        UpdateCameraTransform();
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Environment message
////////////////////////////////////////////////////////////////////////////////
QtOSGViewer::EnvMessage::EnvMessage(QtOSGViewerPtr pviewer, void** ppreturn, bool bWaitForMutex)
    : _pviewer(pviewer), _ppreturn(ppreturn)
{
    // get a mutex
    if( bWaitForMutex ) {
        _plock.reset(new boost::mutex::scoped_lock(_mutex));
    }
}

QtOSGViewer::EnvMessage::~EnvMessage()
{
    _plock.reset();
}

/// execute the command in the caller
void QtOSGViewer::EnvMessage::callerexecute()
{
    bool bWaitForMutex = !!_plock;

    {
        boost::mutex::scoped_lock lock(_pviewer->_mutexMessages);
        _pviewer->_listMessages.push_back(shared_from_this());
    }

    if( bWaitForMutex )
        boost::mutex::scoped_lock lock(_mutex);
}

/// execute the command in the viewer
void QtOSGViewer::EnvMessage::viewerexecute()
{
    _plock.reset();
}

////////////////////////////////////////////////////////////////////////////////
/// Set Sync environment
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::SetEnvironmentSync(bool bUpdate)
{
    boost::mutex::scoped_lock lockupdating(_mutexUpdating);
    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    _bUpdateEnvironment = bUpdate;
    _condUpdateModels.notify_all();

    if( !bUpdate ) {
        // remove all messages in order to release the locks
        boost::mutex::scoped_lock lockmsg(_mutexMessages);
        FOREACH(it,_listMessages)
            (*it)->releasemutex();
        _listMessages.clear();
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Synchronize environment
////////////////////////////////////////////////////////////////////////////////
void QtOSGViewer::EnvironmentSync()
{
    {
        boost::mutex::scoped_lock lockupdating(_mutexUpdating);
        if( !_bUpdateEnvironment ) {
//            RAVELOG_WARNA("cannot update models from environment sync\n");
            return;
        }
    }

    boost::mutex::scoped_lock lock(_mutexUpdateModels);
    _bModelsUpdated = false;
    _condUpdateModels.wait(lock);
    if( !_bModelsUpdated )
        RAVELOG_WARNA("failed to update models from environment sync\n");
}

}
