// This class is based on http://wiki.ogre3d.org/Integrating+Ogre+into+QT5

#include "qtogreviewer.h"

#include <OGRE/Compositor/OgreCompositorManager2.h>

namespace qtogrerave {

QtOgreViewer::QtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput) :
    QMainWindow(NULL, Qt::Window), ViewerBase(penv),
    // Ogre vars
    m_update_pending(false),
    m_animating(false),
    m_ogreRoot(NULL),
    m_ogreWindow(NULL),
    m_ogreCamera(NULL),
    m_cameraMan(NULL)
{
    //
    // initialize member variables
    //
    m_ogreBackground = Ogre::ColourValue(1.0f, 1.0f, 1.0f);

    // _qobjectTree = NULL;
    // _qdetailsTree = NULL;
    _focalDistance = 0.0;
    _fTrackAngleToUp = 0.3;
    // _pointerTypeGroup = NULL;
    // _posgWidget = NULL;
    _nQuitMainLoop = 0;
    // _qactPerspectiveView = NULL;
    // _qactChangeViewtoXY = _qactChangeViewtoYZ = _qactChangeViewtoXZ = NULL;
    //osg::setNotifyLevel(osg::DEBUG_FP);
    // _userdatakey = std::string("qtosg") + boost::lexical_cast<std::string>(this);
    // debugLevelDebugAct = NULL;
    // debugLevelVerboseAct = NULL;

    //
    // read viewer parameters
    //

    bool bCreateStatusBar = true, bCreateMenu = true;
    int nAlwaysOnTopFlag = 0; // 1 - add on top flag (keep others), 2 - add on top flag (remove others)
    sinput >> bCreateStatusBar >> bCreateMenu >> nAlwaysOnTopFlag;

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

    RegisterCommand("SetFiguresInCamera",boost::bind(&QtOgreViewer::_SetFiguresInCamera, this, _1, _2),
                    "Accepts 0/1 value that decides whether to render the figure plots in the camera image through GetCameraImage");
    RegisterCommand("SetItemVisualization",boost::bind(&QtOgreViewer::_SetItemVisualizationCommand, this, _1, _2),
                    "sets the visualization mode of a kinbody/render item in the viewer");
    RegisterCommand("ShowWorldAxes",boost::bind(&QtOgreViewer::_ShowWorldAxesCommand, this, _1, _2),
                    "Accepts 0/1 value that decides whether to render the cross hairs");
    RegisterCommand("SetNearPlane", boost::bind(&QtOgreViewer::_SetNearPlaneCommand, this, _1, _2),
                    "Sets the near plane for rendering of the image. Useful when tweaking rendering units");
    RegisterCommand("SetTextureCubeMap", boost::bind(&QtOgreViewer::_SetTextureCubeMap, this, _1, _2),
                    "Sets the skybox with cubemap");
    RegisterCommand("TrackLink", boost::bind(&QtOgreViewer::_TrackLinkCommand, this, _1, _2),
                    "camera tracks the link maintaining a specific relative transform: robotname, manipname, focalDistance");
    RegisterCommand("TrackManipulator", boost::bind(&QtOgreViewer::_TrackManipulatorCommand, this, _1, _2),
                    "camera tracks the manipulator maintaining a specific relative transform: robotname, manipname, focalDistance");
    RegisterCommand("SetTrackingAngleToUp", boost::bind(&QtOgreViewer::_SetTrackingAngleToUpCommand, this, _1, _2),
                    "sets a new angle to up");
    RegisterCommand("StartViewerLoop", boost::bind(&QtOgreViewer::_StartViewerLoopCommand, this, _1, _2),
                    "starts the viewer sync loop and shows the viewer. expects someone else will call the qapplication exec fn");
    RegisterCommand("SetProjectionMode", boost::bind(&QtOgreViewer::_SetProjectionModeCommand, this, _1, _2),
                    "sets the viewer projection mode, perspective or orthogonal");
    RegisterCommand("Zoom", boost::bind(&QtOgreViewer::_ZoomCommand, this, _1, _2),
                    "Set the zooming factor of the view");
    _bLockEnvironment = true;
    _InitGUI(bCreateStatusBar, bCreateMenu);
    _bUpdateEnvironment = true;
    _bExternalLoop = false;
}

QtOgreViewer::~QtOgreViewer()
{
    RAVELOG_DEBUG("destroying qtogre viewer\n");
    // _notifyGUIFunctionComplete can still be waiting. Code will crash when
    // the mutex is destroyed in that state. SetEnvironmentSync will release
    // _notifyGUIFunctionComplete
    SetEnvironmentSync(false);
    {
        boost::mutex::scoped_lock lock(_mutexGUIFunctions);

        for (GUIThreadFunctionPtr &itmsg: _listGUIFunctions) {
            try {
                itmsg->Call();
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

    if (m_cameraMan) delete m_cameraMan;

    // _condUpdateModels.notify_all();
}

void QtOgreViewer::_InitGUI(bool bCreateStatusBar, bool bCreateMenu)
{
    if( !QApplication::instance() ) {
        RAVELOG_WARN("no app instance to attach close\n");
    }
    else {
        connect(QApplication::instance(), SIGNAL(aboutToQuit()), this, SLOT(_ProcessApplicationQuit()));
    }

    // setCentralWidget(_posgWidget);

    // _RepaintWidgets();

    // _qtree = new QTreeView;

    // _CreateActions();

    // if( bCreateStatusBar ) {
    //     _CreateStatusBar();
    // }

    // if ( bCreateMenu ) {
    //     _CreateMenus();
    //     _CreateToolsBar();
    //     _CreateDockWidgets();
    // }

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
    // _viewGeometryMode = VG_RenderOnly;
    _bRenderFiguresInCamera = true;
    _bDisplayFeedBack = true;

    // Initialize OGRE
    m_ogreRoot = boost::make_shared<Ogre::Root>(Ogre::String("plugins.cfg"));

    const Ogre::RenderSystemList& rsList = m_ogreRoot->getAvailableRenderers();
    Ogre::RenderSystem* rs = rsList[0];

    Ogre::StringVector renderOrder;
    renderOrder.push_back("OpenGL 3+");
    renderOrder.push_back("OpenGL");
    for (Ogre::StringVector::iterator iter = renderOrder.begin(); iter != renderOrder.end(); iter++)
    {
        for (Ogre::RenderSystemList::const_iterator it = rsList.begin(); it != rsList.end(); it++)
        {
            if ((*it)->getName().find(*iter) != Ogre::String::npos)
            {
                rs = *it;
                break;
            }
        }
        if (rs != NULL) break;
    }
    if (rs == NULL)
    {
        if (!m_ogreRoot->restoreConfig())
        {
            if (!m_ogreRoot->showConfigDialog())
                OGRE_EXCEPT(Ogre::Exception::ERR_INVALIDPARAMS,
                    "Abort render system configuration",
                    "QtOgreViewer::initialize");
        }
    }

    QString dimensions = QString("%1 x %2").arg(1024).arg(768);
    rs->setConfigOption("Video Mode", dimensions.toStdString());
    rs->setConfigOption("Full Screen", "No");
    rs->setConfigOption("VSync", "Yes");
    m_ogreRoot->setRenderSystem(rs);
    m_ogreRoot->initialise(false);

    Ogre::NameValuePairList parameters;
    /*
    Flag within the parameters set so that Ogre3D initializes an OpenGL context on it's own.
    */
    if (rs->getName().find("GL") <= rs->getName().size()) {
        parameters["currentGLContext"] = Ogre::String("false");
    }

    /*
    We need to supply the low level OS window handle to this QWindow so that Ogre3D knows where to draw
    the scene. Below is a cross-platform method on how to do this.
    If you set both options (externalWindowHandle and parentWindowHandle) this code will work with OpenGL
    and DirectX.
    */
// #if defined(Q_OS_MAC) || defined(Q_OS_WIN)
//     parameters["externalWindowHandle"] = Ogre::StringConverter::toString((size_t)(this->winId()));
//     parameters["parentWindowHandle"] = Ogre::StringConverter::toString((size_t)(this->winId()));
// #else
//     parameters["externalWindowHandle"] = Ogre::StringConverter::toString((unsigned long)(this->winId()));
//     parameters["parentWindowHandle"] = Ogre::StringConverter::toString((unsigned long)(this->winId()));
// #endif

#if defined(Q_OS_MAC)
    parameters["macAPI"] = "cocoa";
    parameters["macAPICocoaUseNSView"] = "true";
#endif


    /*
    Note below that we supply the creation function for the Ogre3D window the width and height
    from the current QWindow object using the "this" pointer.
    */
    m_ogreWindow = m_ogreRoot->createRenderWindow("QT Window",
        this->width(),
        this->height(),
        false,
        &parameters);
    m_ogreWindow->setVisible(true);

    const size_t numThreads = std::max<int>(1, Ogre::PlatformInformation::getNumLogicalCores());
    Ogre::InstancingThreadedCullingMethod threadedCullingMethod = Ogre::INSTANCING_CULLING_SINGLETHREAD;
    if (numThreads > 1)threadedCullingMethod = Ogre::INSTANCING_CULLING_THREADED;
    m_ogreSceneMgr = m_ogreRoot->createSceneManager(Ogre::ST_GENERIC, numThreads, threadedCullingMethod);

        m_ogreCamera = m_ogreSceneMgr->createCamera("MainCamera");
    m_ogreCamera->setPosition(Ogre::Vector3(0.0f, 0.0f, 10.0f));
    m_ogreCamera->lookAt(Ogre::Vector3(0.0f, 0.0f, -300.0f));
    m_ogreCamera->setNearClipDistance(0.1f);
    m_ogreCamera->setFarClipDistance(200.0f);
    m_cameraMan = new OgreQtBites::SdkQtCameraMan(m_ogreCamera);   // create a default camera controller

#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
    createCompositor();
#else
    Ogre::Viewport* pViewPort = m_ogreWindow->addViewport(m_ogreCamera);
    pViewPort->setBackgroundColour(m_ogreBackground);
#endif

    m_ogreCamera->setAspectRatio(
        Ogre::Real(m_ogreWindow->getWidth()) / Ogre::Real(m_ogreWindow->getHeight()));
    m_ogreCamera->setAutoAspectRatio(true);

    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    // TODO: take care of this later
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups(true);

    createScene();

    m_ogreRoot->addFrameListener(this);
}

void QtOgreViewer::createScene()
{
    /*
    Example scene
    Derive this class for your own purpose and overwite this function to have a working Ogre widget with
    your own content.
    */
    m_ogreSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f), Ogre::ColourValue(0.5f, 0.5f, 0.5f), Ogre::Vector3(0.0f, 0.0f, 1.0f));

#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
    Ogre::v1::Entity* sphereMesh = m_ogreSceneMgr->createEntity(Ogre::SceneManager::PT_SPHERE);
#else
    Ogre::v1::Entity* sphereMesh = m_ogreSceneMgr->createEntity("mySphere", Ogre::SceneManager::PT_SPHERE);
#endif

    Ogre::SceneNode* childSceneNode = m_ogreSceneMgr->getRootSceneNode()->createChildSceneNode();

    childSceneNode->attachObject(sphereMesh);

    Ogre::MaterialPtr sphereMaterial = Ogre::MaterialManager::getSingleton().create("SphereMaterial",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

    sphereMaterial->getTechnique(0)->getPass(0)->setAmbient(0.1f, 0.1f, 0.1f);
    sphereMaterial->getTechnique(0)->getPass(0)->setDiffuse(0.2f, 0.2f, 0.2f, 1.0f);
    sphereMaterial->getTechnique(0)->getPass(0)->setSpecular(0.9f, 0.9f, 0.9f, 1.0f);
    //sphereMaterial->setAmbient(0.2f, 0.2f, 0.5f);
    //sphereMaterial->setSelfIllumination(0.2f, 0.2f, 0.1f);

    sphereMesh->setMaterialName("SphereMaterial");
    childSceneNode->setPosition(Ogre::Vector3(0.0f, 0.0f, 0.0f));
    childSceneNode->setScale(Ogre::Vector3(0.01f, 0.01f, 0.01f)); // Radius, in theory.

#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
    Ogre::SceneNode* pLightNode = m_ogreSceneMgr->getRootSceneNode()->createChildSceneNode();
    Ogre::Light* light = m_ogreSceneMgr->createLight();
    pLightNode->attachObject(light);
    pLightNode->setPosition(20.0f, 80.0f, 50.0f);
#else
    Ogre::Light* light = m_ogreSceneMgr->createLight("MainLight");
    light->setPosition(20.0f, 80.0f, 50.0f);
#endif
}

#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
void QtOgreViewer::createCompositor()
{
    /*
    Example compositor
    Derive this class for your own purpose and overwite this function to have a working Ogre
    widget with your own compositor.
    */
    Ogre::CompositorManager2* compMan = m_ogreRoot->getCompositorManager2();
    const Ogre::String workspaceName = "default scene workspace";
    const Ogre::IdString workspaceNameHash = workspaceName;
    compMan->createBasicWorkspaceDef(workspaceName, m_ogreBackground);
    compMan->addWorkspace(m_ogreSceneMgr, m_ogreWindow, m_ogreCamera, workspaceNameHash, true);
}
#endif

void QtOgreViewer::render()
{
    /*
    How we tied in the render function for OGre3D with QWindow's render function. This is what gets call
    repeatedly. Note that we don't call this function directly; rather we use the renderNow() function
    to call this method as we don't want to render the Ogre3D scene unless everything is set up first.
    That is what renderNow() does.

    Theoretically you can have one function that does this check but from my experience it seems better
    to keep things separate and keep the render function as simple as possible.
    */
    Ogre::WindowEventUtilities::messagePump();
    m_ogreRoot->renderOneFrame();
}

void QtOgreViewer::renderLater()
{
    /*
    This function forces QWindow to keep rendering. Omitting this causes the renderNow() function to
    only get called when the window is resized, moved, etc. as opposed to all of the time; which is
    generally what we need.
    */
    if (!m_update_pending)
    {
        m_update_pending = true;
        QApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
    }
}

bool QtOgreViewer::event(QEvent *event)
{
    /*
    QWindow's "message pump". The base method that handles all QWindow events. As you will see there
    are other methods that actually process the keyboard/other events of Qt and the underlying OS.

    Note that we call the renderNow() function which checks to see if everything is initialized, etc.
    before calling the render() function.
    */

    switch (event->type())
    {
    case QEvent::UpdateRequest:
        m_update_pending = false;
        renderNow();
        return true;

    default:
        return QWindow::event(event);
    }
}

/*
Called after the QWindow is reopened or when the QWindow is first opened.
*/
// void QtOgreViewer::exposeEvent(QExposeEvent *event)
// {
//     Q_UNUSED(event);

//     if (isExposed())
//         renderNow();
// }

/*
The renderNow() function calls the initialize() function when needed and if the QWindow is already
initialized and prepped calls the render() method.
*/
void QtOgreViewer::renderNow()
{
    if (!isExposed())
        return;

    if (m_ogreRoot == NULL)
    {
        initialize();
    }

    render();

    if (m_animating)
        renderLater();
}

/*
Our event filter; handles the resizing of the QWindow. When the size of the QWindow changes note the
call to the Ogre3D window and camera. This keeps the Ogre3D scene looking correct.
*/
bool QtOgreViewer::eventFilter(QObject *target, QEvent *event)
{
    if (target == this)
    {
        if (event->type() == QEvent::Resize)
        {
            if (isExposed() && m_ogreWindow != NULL)
            {
                m_ogreWindow->resize(this->width(), this->height());
            }
        }
    }

    return false;
}

/*
How we handle keyboard and mouse events.
*/
// void QtOgreViewer::keyPressEvent(QKeyEvent * ev)
// {
//     if(m_cameraMan)
//         m_cameraMan->injectKeyDown(*ev);
// }

// void QtOgreViewer::keyReleaseEvent(QKeyEvent * ev)
// {
//     if(m_cameraMan)
//         m_cameraMan->injectKeyUp(*ev);
// }

// void QtOgreViewer::mouseMoveEvent( QMouseEvent* e )
// {
//     static int lastX = e->x();
//     static int lastY = e->y();
//     int relX = e->x() - lastX;
//     int relY = e->y() - lastY;
//     lastX = e->x();
//     lastY = e->y();

//     if(m_cameraMan && (e->buttons() & Qt::LeftButton))
//         m_cameraMan->injectMouseMove(relX, relY);
// }

// void QtOgreViewer::wheelEvent(QWheelEvent *e)
// {
//     if(m_cameraMan)
//         m_cameraMan->injectWheelMove(*e);
// }

// void QtOgreViewer::mousePressEvent( QMouseEvent* e )
// {
//     if(m_cameraMan)
//         m_cameraMan->injectMouseDown(*e);
// }

// void QtOgreViewer::mouseReleaseEvent( QMouseEvent* e )
// {
//     if(m_cameraMan)
//         m_cameraMan->injectMouseUp(*e);

//     QPoint pos = e->pos();
//     Ogre::Ray mouseRay = m_ogreCamera->getCameraToViewportRay(
//         (Ogre::Real)pos.x() / m_ogreWindow->getWidth(),
//         (Ogre::Real)pos.y() / m_ogreWindow->getHeight());
//     Ogre::RaySceneQuery* pSceneQuery = m_ogreSceneMgr->createRayQuery(mouseRay);
//     pSceneQuery->setSortByDistance(true);
//     Ogre::RaySceneQueryResult vResult = pSceneQuery->execute();
//     for (size_t ui = 0; ui < vResult.size(); ui++)
//     {
//         if (vResult[ui].movable)
//         {
//             if (vResult[ui].movable->getMovableType().compare("Entity") == 0)
//             {
//                 emit entitySelected((Ogre::v1::Entity*)vResult[ui].movable);
//             }
//         }
//     }
//     m_ogreSceneMgr->destroyQuery(pSceneQuery);
// }

/*
Function to keep track of when we should and shouldn't redraw the window; we wouldn't want to do
rendering when the QWindow is minimized. This takes care of those scenarios.
*/
void QtOgreViewer::setAnimating(bool animating)
{
    m_animating = animating;

    if (animating)
        renderLater();
}

bool QtOgreViewer::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    m_cameraMan->frameRenderingQueued(evt);
    return true;
}

bool QtOgreViewer::_HandleOgreKeyDown(int key)
{
//     switch(key) {
//     case osgGA::GUIEventAdapter::KEY_Escape:
//         if( !!_pointerTypeGroup ) {
//             // toggle
//             int newcheckedid = _pointerTypeGroup->checkedId()-1;
//             int minchecked = -4, maxchecked = -2;
//             if( newcheckedid < minchecked || newcheckedid > maxchecked ) {
//                 newcheckedid = maxchecked;
//             }
//             _pointerTypeGroup->button(newcheckedid)->setChecked(true);
//             _ProcessPointerGroupClicked(newcheckedid);
//             return true;
// //            //RAVELOG_INFO_FORMAT("checked id %d", _pointerTypeGroup->checkedId());
// //            //if( !!_pointerTypeGroup->checkedButton() ) {
// //                //_pointerTypeGroup->checkedButton()->setChecked(false);
// //                _PostToGUIThread(boost::bind(&QAbstractButton::setChecked, _pointerTypeGroup->checkedButton(), false));
// //            }
// //            else {
// //                // check one?
// //            }
//         }
//         break;
//     }

    return false;
}

void QtOgreViewer::customEvent(QEvent * e)
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

bool QtOgreViewer::_ForceUpdatePublishedBodies()
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

void QtOgreViewer::_CreateActions()
{
#if 0
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
#endif
}

void QtOgreViewer::_UpdateViewerCallback()
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

void QtOgreViewer::_Reset()
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


    // {
    //     boost::mutex::scoped_lock lock(_mutexItems);
    //     FOREACH(it,_listRemoveItems) {
    //         delete *it;
    //     }
    //     _listRemoveItems.clear();
    // }
}

void QtOgreViewer::_CreateMenus()
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
    viewMenu->addAction(debugLevelDebugAct);
    viewMenu->addAction(debugLevelVerboseAct);
    viewMenu->addAction(showAct);

//  animation = menuBar()->addMenu(tr("&Animation"));
//  animation->addAction(playAct);
//  animation->addAction(stopAct);
//
//  options = menuBar()->addMenu(tr("&Options"));
//  options->addAction(recordAct);
//
//  dynamics = menuBar()->addMenu(tr("D&ynamics"));
//  dynamics->addAction(odeAct);
//  dynamics->addAction(selfAct);
//  dynamics->addAction(applyAct);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
}

void QtOgreViewer::LoadEnvironment()
{
    QString s = QFileDialog::getOpenFileName( _posgWidget, "Load Environment", QString(), "Env Files (*.xml);;COLLADA Files (*.dae|*.zae)");
    if( s.length() == 0 ) {
        return;
    }

    _Reset();
    try {
        EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        GetEnv()->Reset();

        GetEnv()->Load(s.toAscii().data());

        RAVELOG_INFO("\n---------Refresh--------\n");

        //  Refresh the screen.
        UpdateFromModel();

        RAVELOG_INFO("----> set home <----\n");

        //  Center object in window viewer
        _posgWidget->SetHome();
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("failed to load environment %s: %s", s.toAscii().data()%ex.what());
    }

}

void QtOgreViewer::ImportEnvironment()
{
    QString s = QFileDialog::getOpenFileName( this, "Import Environment", NULL,
                                              "Env Files (*.xml);;COLLADA Files (*.dae)");

    if( s.length() == 0 ) {
        return;
    }
    try {
        EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        GetEnv()->Load(s.toAscii().data());

        //  Refresh the screen.
        UpdateFromModel();
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("failed to import model %s: %s", s.toAscii().data()%ex.what());
    }
}

void QtOgreViewer::SaveEnvironment()
{
    QString s = QFileDialog::getSaveFileName( this, "Save Environment", NULL, "Env Files (*.xml);;COLLADA Files (*.dae)");
    if( s.length() == 0 ) {
        return;
    }
    try {
        EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        GetEnv()->Save(s.toAscii().data());
    }
    catch(const std::exception& ex) {
        RAVELOG_WARN_FORMAT("failed to save to file %s: %s", s.toAscii().data()%ex.what());
    }
}

void QtOgreViewer::ResetViewToHome()
{
    _posgWidget->ResetViewToHome();
}

void QtOgreViewer::_ProcessPerspectiveViewChange()
{
    if( _qactPerspectiveView->isChecked() ) {
        _qactPerspectiveView->setIcon(QIcon(":/images/bbox.png"));
    }
    else {
        _qactPerspectiveView->setIcon(QIcon(":/images/perspective.png"));
    }

    _posgWidget->SetViewType(_qactPerspectiveView->isChecked());
}

void QtOgreViewer::_ProcessAboutDialog()
{
    QMessageBox msgBox;
    msgBox.setText("OpenRAVE qtosg plugin");
    msgBox.setInformativeText(__description.c_str());
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);
    msgBox.exec();
}

void QtOgreViewer::_SetDebugLevelDebug()
{
    RaveSetDebugLevel(Level_Debug);
}

void QtOgreViewer::_SetDebugLevelVerbose()
{
    RaveSetDebugLevel(Level_Verbose);
}

void QtOgreViewer::_ChangeViewToXY()
{
    // _UpdateCameraTransform(0);
    // osg::Vec3d center = _posgWidget->GetSceneRoot()->getBound().center();
    // _Tcamera.rot = quatFromAxisAngle(RaveVector<float>(1,0,0), float(0));
    // _Tcamera.trans.x = center.x();
    // _Tcamera.trans.y = center.y();
    // _Tcamera.trans.z = center.z()+_focalDistance;
    // _SetCameraTransform();
}

void QtOgreViewer::_ChangeViewToXZ()
{
    // _UpdateCameraTransform(0);
    // osg::Vec3d center = _posgWidget->GetSceneRoot()->getBound().center();
    // _Tcamera.rot = quatFromAxisAngle(RaveVector<float>(1,0,0), float(M_PI/2));
    // _Tcamera.trans.x = center.x();
    // _Tcamera.trans.y = center.y()-_focalDistance;
    // _Tcamera.trans.z = center.z();
    // _SetCameraTransform();

}

void QtOgreViewer::_ChangeViewToYZ()
{
    // _UpdateCameraTransform(0);
    // osg::Vec3d center = _posgWidget->GetSceneRoot()->getBound().center();
    // _Tcamera.rot = quatMultiply(quatFromAxisAngle(RaveVector<float>(0,0,1), float(M_PI/2)), quatFromAxisAngle(RaveVector<float>(1,0,0), float(M_PI/2)));
    // _Tcamera.trans.x = center.x()+_focalDistance;
    // _Tcamera.trans.y = center.y();
    // _Tcamera.trans.z = center.z();
    // _SetCameraTransform();

}

//void QtOgreViewer::_ProcessLightChange()
//{
//    if (lightAct->isChecked()) {
//        lightAct->setIcon(QIcon(":/images/lightoff.png"));
//    }
//    else {
//        lightAct->setIcon(QIcon(":/images/lighton.png"));
//    }
//    _posgWidget->SetLight(!lightAct->isChecked());
//}

//void QtOgreViewer::_ProcessFacesModeChange()
//{
//    _posgWidget->SetFacesMode(!facesAct->isChecked());
//}

void QtOgreViewer::polygonMode()
{
    _posgWidget->SetPolygonMode(wireAct->isChecked() ? 2 : 0);
}

void QtOgreViewer::_ProcessBoundingBox()
{
    _posgWidget->DrawBoundingBox(bboxAct->isChecked());
}

void QtOgreViewer::axes()
{
    //?
    //_posgWidget->DrawAxes(AxesAct->isChecked());
}

void QtOgreViewer::_ProcessPointerGroupClicked(int button)
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

void QtOgreViewer::_ProcessDraggerGroupClicked(int button)
{
    RAVELOG_INFO_FORMAT("Dragger button clicked %d", button);
}

void QtOgreViewer::_RepaintWidgets()
{
    _posgWidget->SetSceneData();
}

void QtOgreViewer::_CreateToolsBar()
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


    // _pointerTypeGroup = new QButtonGroup();
    // _pointerTypeGroup->addButton(pointerButton);
    // _pointerTypeGroup->addButton(axesButton);
    // _pointerTypeGroup->addButton(rotationButton);
    // //_pointerTypeGroup->addButton(handButton);

    // connect(_pointerTypeGroup, SIGNAL(buttonClicked(int)), this, SLOT(_ProcessPointerGroupClicked(int)));

    // //shapeGroup = new QActionGroup(this);
    // //smoothAct->setCheckable(true);
    // //flatAct->setCheckable(true);
    // wireAct->setCheckable(true);
    // //shapeGroup->addAction(smoothAct);
    // //shapeGroup->addAction(flatAct);
    // //shapeGroup->addAction(wireAct);
    // //smoothAct->setChecked(true);

    // toolsBar->addWidget(pointerButton);
    // toolsBar->addWidget(axesButton);
    // toolsBar->addWidget(rotationButton);
    // //toolsBar->addWidget(handButton);
    // toolsBar->addAction(houseAct);
    // toolsBar->addAction(_qactChangeViewtoXY);
    // toolsBar->addAction(_qactChangeViewtoXZ);
    // toolsBar->addAction(_qactChangeViewtoYZ);
    // toolsBar->addAction(_qactPerspectiveView);
    // //toolsBar->addAction(lightAct);
    // //toolsBar->addAction(smoothAct);
    // //toolsBar->addAction(flatAct);
    // toolsBar->addAction(wireAct);
    //toolsBar->addAction(facesAct);
}

void QtOgreViewer::_CreateStatusBar()
{
    statusBar()->showMessage(tr("Ready"));
}

void QtOgreViewer::_CreateDockWidgets()
{
    // QDockWidget *dock = new QDockWidget(tr("Objects Tree"), this);
    // dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    // _qobjectTree = _CreateObjectTree();
    // dock->setWidget(_qobjectTree);
    // dock->hide();

    // addDockWidget(Qt::RightDockWidgetArea, dock);
    // viewMenu->addAction(dock->toggleViewAction());


    // dock = new QDockWidget(tr("Details"), this);

    // //  QListWidget *sensorList = new QListWidget(dock);

    // _qdetailsTree = new QTreeWidget();
    // _qdetailsTree->setHeaderLabel(QString("Properties"));
    // dock->setWidget(_qdetailsTree);
    // dock->hide();
    // addDockWidget(Qt::RightDockWidgetArea, dock);
    // viewMenu->addAction(dock->toggleViewAction());
}

void QtOgreViewer::_OnObjectTreeClick(QTreeWidgetItem* item,int num)
{
    // RobotBasePtr robot;
    // KinBodyPtr kinbody;
    // KinBody::LinkPtr link;

    // std::string mass;

    // //  Select robot in Viewers
    // _posgWidget->SelectItemFromName(item->text(0).toAscii().data());

    // //  Clears details
    // if (!!_qdetailsTree) {
    //     _qdetailsTree->clear();
    // }

    // QList<QTreeWidgetItem*> items;

    // if (!!item->parent()) {
    //     if (item->parent()->text(0) == "Links") {
    //         std::ostringstream strs;

    //         //  Set Title
    //         if (!!_qdetailsTree) {
    //             _qdetailsTree->setHeaderLabel(item->text(0).toAscii().data());
    //         }

    //         robot = GetEnv()->GetRobot(item->parent()->parent()->text(0).toAscii().data());
    //         link  = robot->GetLink(item->text(0).toAscii().data());

    //         //  Clears output string
    //         strs.clear();

    //         strs << link->GetMass();

    //         mass = std::string(" Mass= ") + strs.str();

    //         items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(mass.c_str()))));
    //     }
    //     else {
    //         //  Set Title
    //         if (!!_qdetailsTree) {
    //             _qdetailsTree->setHeaderLabel(item->text(0).toAscii().data());
    //         }
    //     }
    // }
    // else {
    //     //  Set Title
    //     if (!!_qdetailsTree) {
    //         _qdetailsTree->setHeaderLabel(item->text(0).toAscii().data());
    //     }
    //     kinbody = GetEnv()->GetKinBody(item->text(0).toAscii().data());
    //     for (size_t i=0; i<kinbody->GetLinks().size(); i++) {
    //         std::ostringstream strs;
    //         link = kinbody->GetLinks()[i];
    //         strs << link->GetMass();
    //         mass = link->GetName() + std::string(" Mass= ") + strs.str();
    //         items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(mass.c_str()))));
    //     }
    // }

    // //  Add Items to details panel
    // if (!!_qdetailsTree) {
    //     _qdetailsTree->insertTopLevelItems(0, items);
    // }
}

QTreeWidget* QtOgreViewer::_CreateObjectTree()
{
    // QTreeWidget *treeWidget = new QTreeWidget();
    // treeWidget->setColumnCount(1);

    // treeWidget->setHeaderLabel(QString("Scene"));

    // //  Connect scene list clicked
    // connect(treeWidget,SIGNAL(itemClicked(QTreeWidgetItem*,int)), this, SLOT(_OnObjectTreeClick(QTreeWidgetItem*,int)));

    // return treeWidget;
}

void QtOgreViewer::_FillObjectTree(QTreeWidget *treeWidget)
{
    // RAVELOG_VERBOSE("Begin _FillObjectTree....\n");
    // vector<KinBodyPtr> kinbodies;
    // QList<QTreeWidgetItem*> items;

    // //  Clears tree
    // treeWidget->clear();
    // GetEnv()->GetBodies(kinbodies);
    // for (size_t i = 0; i < kinbodies.size(); i++) {
    //     items.append(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(kinbodies[i]->GetName().c_str()))));
    //     //  Number of child to add
    //     int nchild = -1;

    //     if( kinbodies[i]->GetLinks().size() > 0 ) {
    //         //  Header 'Links'
    //         items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Links"))));
    //         nchild++;
    //         FOREACHC(itlink, kinbodies[i]->GetLinks()) {
    //             items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString((*itlink)->GetName().c_str()))));
    //         }
    //     }

    //     if (kinbodies[i]->GetJoints().size() > 0) {
    //         //  Header 'Joints'
    //         items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Joints"))));
    //         nchild++;

    //         FOREACHC(itjoint, kinbodies[i]->GetJoints()) {
    //             KinBody::JointConstPtr pjoint = *itjoint;
    //             QTreeWidgetItem* pqjoint = new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pjoint->GetName().c_str())));

    //             //  Adds links of joints
    //             pqjoint->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pjoint->GetFirstAttached()->GetName().c_str()))));
    //             pqjoint->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pjoint->GetSecondAttached()->GetName().c_str()))));
    //             items[i]->child(nchild)->addChild(pqjoint);
    //         }
    //     }

    //     if (kinbodies[i]->IsRobot()) {
    //         RobotBasePtr robot = RaveInterfaceCast<RobotBase>(kinbodies[i]);

    //         if (robot->GetAttachedSensors().size() > 0) {
    //             //  Header 'Sensors'
    //             items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Sensors"))));
    //             nchild++;

    //             FOREACHC(itattsensor, robot->GetAttachedSensors()) {
    //                 RobotBase::AttachedSensorPtr pattsensor = *itattsensor;
    //                 QTreeWidgetItem* pqattsensor = new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pattsensor->GetName().c_str())));
    //                 RAVELOG_VERBOSE_FORMAT("Attach sensor %s robotlink=%s", pattsensor->GetName()%pattsensor->GetAttachingLink()->GetName());
    //                 pqattsensor->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pattsensor->GetAttachingLink()->GetName().c_str()))));
    //                 items[i]->child(nchild)->addChild(pqattsensor);
    //             }
    //         }

    //         if (robot->GetManipulators().size() > 0) {
    //             //  Header 'Manipulators'
    //             items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Manipulators"))));
    //             nchild++;

    //             FOREACHC(itmanip, robot->GetManipulators()) {
    //                 RobotBase::ManipulatorPtr pmanip = *itmanip;
    //                 items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(pmanip->GetName().c_str()))));
    //             }
    //         }

    //         ControllerBasePtr controller = robot->GetController();
    //         if (!!controller) {
    //             //  Header 'Controller'
    //             items[i]->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString("Controller"))));
    //             nchild++;

    //             items[i]->child(nchild)->addChild(new QTreeWidgetItem((QTreeWidget*)0, QStringList(QString(controller->GetXMLFilename().c_str()))));
    //         }
    //     }
    // }

    // treeWidget->insertTopLevelItems(0, items);
    // RAVELOG_VERBOSE("End _FillObjectTree....\n");
}

void QtOgreViewer::_UpdateCameraTransform(float fTimeElapsed)
{
}

bool QtOgreViewer::_SetFiguresInCamera(std::ostream& sout, std::istream& sinput)
{
    sinput >> _bRenderFiguresInCamera;
    return !!sinput;
}

bool QtOgreViewer::_SetItemVisualizationCommand(std::ostream& sout, std::istream& sinput)
{
    std::string itemname, visualizationmode;
    sinput >> itemname >> visualizationmode;

    FOREACH(it, _mapbodies) {
        if( it->second->GetName() == itemname ) {
            it->second->SetVisualizationMode(visualizationmode);
        }
    }
    return !!sinput;
}

bool QtOgreViewer::_ShowWorldAxesCommand(std::ostream& sout, std::istream& sinput)
{
    // TODO
    return true;
}

bool QtOgreViewer::_SetNearPlaneCommand(std::ostream& sout, std::istream& sinput)
{
    dReal nearplane=0.01;
    sinput >> nearplane;
    if( !!sinput ) {
        _posgWidget->SetNearPlane(nearplane);
    }
    return !!sinput;
}

bool QtOgreViewer::_SetTextureCubeMap(std::ostream& out, std::istream& sinput)
{
    std::string posx, negx, posy, negy, posz, negz;
    sinput >> posx >> negx >> posy >> negy >> posz >> negz;

    if(!!sinput) {
        _posgWidget->SetTextureCubeMap(posx, negx, posy, negy, posz, negz);
    }

    return !!sinput;
}

bool QtOgreViewer::_TrackLinkCommand(std::ostream& sout, std::istream& sinput)
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

bool QtOgreViewer::_TrackManipulatorCommand(std::ostream& sout, std::istream& sinput)
{
    // bool bresetvelocity = true;
    // std::string robotname, manipname;
    // float focalDistance = 0.0;
    // sinput >> robotname >> manipname >> focalDistance >> bresetvelocity;
    // if( focalDistance > 0 ) {
    //     // not sure if this is thread safe...
    //     _SetCameraDistanceToFocus(focalDistance);
    // }
    // _ptrackinglink.reset();
    // _ptrackingmanip.reset();
    // EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    // RobotBasePtr probot = GetEnv()->GetRobot(robotname);
    // if( !probot ) {
    //     return false;
    // }
    // _ptrackingmanip = probot->GetManipulator(manipname);
    // if( bresetvelocity ) {
    //     _tTrackingCameraVelocity.trans = _tTrackingCameraVelocity.rot = Vector(); // reset velocity?
    // }
    // return !!_ptrackingmanip;
}

bool QtOgreViewer::_SetTrackingAngleToUpCommand(std::ostream& sout, std::istream& sinput)
{
    sinput >> _fTrackAngleToUp;
    return true;
}

void QtOgreViewer::_ProcessApplicationQuit()
{
    // RAVELOG_VERBOSE("processing viewer application quit\n");
    // // remove all messages in order to release the locks
    // boost::mutex::scoped_lock lockmsg(_mutexGUIFunctions);
    // if( _listGUIFunctions.size() > 0 ) {
    //     bool bnotify = false;
    //     FOREACH(it,_listGUIFunctions) {
    //         (*it)->SetFinished();
    //         if( (*it)->IsBlocking() ) {
    //             bnotify = true;
    //         }
    //     }
    //     if( bnotify ) {
    //         _notifyGUIFunctionComplete.notify_all();
    //     }
    // }
    // _listGUIFunctions.clear();

}

bool QtOgreViewer::_StartViewerLoopCommand(std::ostream& sout, std::istream& sinput)
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

bool QtOgreViewer::_SetProjectionModeCommand(std::ostream& sout, std::istream& sinput)
{
    std::string projectionMode = "";
    sinput >> projectionMode;
    _PostToGUIThread(boost::bind(&QtOgreViewer::_SetProjectionMode, this, projectionMode));
    return true;
}

bool QtOgreViewer::_ZoomCommand(std::ostream& sout, std::istream& sinput)
{
    float factor = 1.0f;
    sinput >> factor;
    _PostToGUIThread(boost::bind(&QtOgreViewer::_Zoom, this, factor));
    return true;
}

void QtOgreViewer::_SetProjectionMode(const std::string& projectionMode)
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

int QtOgreViewer::main(bool bShow)
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

void QtOgreViewer::quitmainloop()
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

void QtOgreViewer::Show(int showtype)
{
    if( showtype ) {
        _PostToGUIThread(boost::bind(&QtOgreViewer::show, this));
    }
    else {
        _PostToGUIThread(boost::bind(&QtOgreViewer::hide, this));
    }
//    // have to put this in the message queue
//    if (showtype ) {
//        show();
//    }
//    else {
//        hide();
//    }
}

bool QtOgreViewer::GetFractionOccluded(KinBodyPtr pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded)
{
    return false;
}

bool QtOgreViewer::GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK)
{
    return false;
}

bool QtOgreViewer::WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension)
{
    return false;
}

void QtOgreViewer::_SetCameraTransform()
{
}

void QtOgreViewer::_SetCamera(RaveTransform<float> trans, float focalDistance)
{
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    _Tcamera = trans*trot;

    if (focalDistance > 0) {
        _focalDistance = focalDistance;
    }

    _SetCameraTransform();
    _UpdateCameraTransform(0);
}

void QtOgreViewer::SetCamera(const RaveTransform<float>& trans, float focalDistance)
{
    _PostToGUIThread(boost::bind(&QtOgreViewer::_SetCamera, this, trans, focalDistance));
}

void QtOgreViewer::_SetCameraDistanceToFocus(float focalDistance)
{
    if (focalDistance > 0) {
        _focalDistance = focalDistance;
    }

    _SetCameraTransform();
    _UpdateCameraTransform(0);
}

RaveTransform<float> QtOgreViewer::GetCameraTransform() const
{
    boost::mutex::scoped_lock lock(_mutexGUIFunctions);
    // have to flip Z axis
    RaveTransform<float> trot; trot.rot = quatFromAxisAngle(RaveVector<float>(1,0,0),(float)PI);
    return _Tcamera*trot;
}

float QtOgreViewer::GetCameraDistanceToFocus() const
{
    boost::mutex::scoped_lock lock(_mutexGUIFunctions);
    return _focalDistance;
}

geometry::RaveCameraIntrinsics<float> QtOgreViewer::GetCameraIntrinsics() const
{
    boost::mutex::scoped_lock lock(_mutexGUIFunctions);
    return _camintrinsics;
}

SensorBase::CameraIntrinsics QtOgreViewer::GetCameraIntrinsics2() const
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

GraphHandlePtr QtOgreViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
{
    // OgreHandlePtr handle = _CreateGraphHandle();

    // osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    // for(int i = 0; i < numPoints; ++i) {
    //     (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
    //     ppoints = (float*)((char*)ppoints + stride);
    // }
    // osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(1);
    // (*vcolors)[0] = osg::Vec4f(color.x, color.y, color.z, color.w);
    // _PostToGUIThread(boost::bind(&QtOgreViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::POINTS, new osg::Point(fPointSize),color.w<1)); // copies ref counts
    // return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOgreViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle, bool bhasalpha)
{
    // OgreHandlePtr handle = _CreateGraphHandle();

    // osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    // osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    // for(int i = 0; i < numPoints; ++i) {
    //     (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
    //     ppoints = (float*)((char*)ppoints + stride);
    //     if (bhasalpha) {
    //         (*vcolors)[i] = osg::Vec4f(colors[i * 4 + 0], colors[i * 4 + 1], colors[i * 4 + 2], colors[i * 4 + 3]);
    //     }
    //     else {
    //         (*vcolors)[i] = osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f);
    //     }
    // }

    // _PostToGUIThread(boost::bind(&QtOgreViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::POINTS, osg::ref_ptr<osg::Point>(new osg::Point(fPointSize)), bhasalpha)); // copies ref counts
    // return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOgreViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    // OgreHandlePtr handle = _CreateGraphHandle();

    // osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    // for(int i = 0; i < numPoints; ++i) {
    //     (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
    //     ppoints = (float*)((char*)ppoints + stride);
    // }
    // osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(1);
    // (*vcolors)[0] = osg::Vec4f(color.x, color.y, color.z, color.w);
    // _PostToGUIThread(boost::bind(&QtOgreViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINE_STRIP, osg::ref_ptr<osg::LineWidth>(new osg::LineWidth(fwidth)), color.w<1)); // copies ref counts
    // return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}
GraphHandlePtr QtOgreViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    // OgreHandlePtr handle = _CreateGraphHandle();

    // osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    // osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    // for(int i = 0; i < numPoints; ++i) {
    //     (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
    //     ppoints = (float*)((char*)ppoints + stride);
    //     (*vcolors)[i] = osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f);
    // }

    // _PostToGUIThread(boost::bind(&QtOgreViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINE_STRIP, osg::ref_ptr<osg::LineWidth>(new osg::LineWidth(fwidth)), false)); // copies ref counts
    // return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOgreViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    // OgreHandlePtr handle = _CreateGraphHandle();
    // osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    // for(int i = 0; i < numPoints; ++i) {
    //     (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
    //     ppoints = (float*)((char*)ppoints + stride);
    // }
    // osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(1);
    // (*vcolors)[0] = osg::Vec4f(color.x, color.y, color.z, color.w);
    // _PostToGUIThread(boost::bind(&QtOgreViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINES, osg::ref_ptr<osg::LineWidth>(new osg::LineWidth(fwidth)), color.w<1)); // copies ref counts
    // return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}
GraphHandlePtr QtOgreViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    // OgreHandlePtr handle = _CreateGraphHandle();

    // osg::ref_ptr<osg::Vec3Array> vvertices = new osg::Vec3Array(numPoints);
    // osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(numPoints);
    // for(int i = 0; i < numPoints; ++i) {
    //     (*vvertices)[i] = osg::Vec3(ppoints[0], ppoints[1], ppoints[2]);
    //     ppoints = (float*)((char*)ppoints + stride);
    //     (*vcolors)[i] = osg::Vec4f(colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2], 1.0f);
    // }

    // _PostToGUIThread(boost::bind(&QtOgreViewer::_Draw, this, handle, vvertices, vcolors, osg::PrimitiveSet::LINES, osg::ref_ptr<osg::LineWidth>(new osg::LineWidth(fwidth)), false)); // copies ref counts
    // return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOgreViewer::drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
{
    // RAVELOG_WARN("drawarrow not implemented\n");
    // return GraphHandlePtr();
}

GraphHandlePtr QtOgreViewer::drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
{
    // OgreHandlePtr handle = _CreateGraphHandle();
    // _PostToGUIThread(boost::bind(&QtOgreViewer::_DrawBox, this, handle, vpos, vextents, false)); // copies ref counts
    // return GraphHandlePtr();
}

GraphHandlePtr QtOgreViewer::drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
{
    // OgreHandlePtr handle = _CreateGraphHandle();
    // _PostToGUIThread(boost::bind(&QtOgreViewer::_DrawPlane, this, handle, tplane, vextents, vtexture)); // copies ref counts
    // return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

void QtOgreViewer::_SetTriangleMesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, osg::ref_ptr<osg::Vec3Array> osgvertices, osg::ref_ptr<osg::DrawElementsUInt> osgindices)
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

GraphHandlePtr QtOgreViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
{
    _PostToGUIThread(boost::bind(&QtOgreViewer::_DrawTriMesh, this, handle, osgvertices, osgcolors, osgindices, color.w<1)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

GraphHandlePtr QtOgreViewer::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
{
    OgreHandlePtr handle = _CreateGraphHandle();
    _PostToGUIThread(boost::bind(&QtOgreViewer::_DrawTriMesh, this, handle, osgvertices, osgcolors, osgindices, bhasalpha)); // copies ref counts
    return GraphHandlePtr(new PrivateGraphHandle(shared_viewer(), handle));
}

void QtOgreViewer::_Deselect()
{

}

void QtOgreViewer::Reset()
{
    // TODO
}

void QtOgreViewer::SetBkgndColor(const RaveVector<float>& color)
{
    _PostToGUIThread(boost::bind(&QtOgreViewer::_SetBkgndColor, this, color));
}

void QtOgreViewer::_SetBkgndColor(const RaveVector<float>& color)
{
    m_ogreBackground = Ogre::ColourValue(color.x, color.y, color.z);
    // TODO: Set???
}

void QtOgreViewer::StartPlaybackTimer()
{

}
void QtOgreViewer::StopPlaybackTimer()
{

}

void QtOgreViewer::SetSize(int w, int h)
{
    _PostToGUIThread(boost::bind(&QtOgreViewer::resize, this, w, h));
}
void QtOgreViewer::Move(int x, int y)
{
    _PostToGUIThread(boost::bind(&QtOgreViewer::move, this, x, y));
}

void QtOgreViewer::Zoom(float factor)
{
    _PostToGUIThread(boost::bind(&QtOgreViewer::_Zoom, this, factor));
}

void QtOgreViewer::_Zoom(float factor)
{
    _posgWidget->Zoom(factor);
}

void QtOgreViewer::SetName(const std::string& name)
{
    _PostToGUIThread(boost::bind(&QtOgreViewer::_SetName, this, name));
}

void QtOgreViewer::_SetName(const std::string& name)
{
    setWindowTitle(name.c_str());
}

bool QtOgreViewer::LoadModel(const std::string& filename)
{
    if( filename == "") {
        return false;
    }
    OgreNodePtr loadedModel = osgDB::readNodeFile(filename);
    if (!!loadedModel) {
        _posgWidget->GetSceneRoot()->addChild(loadedModel.get());
        return true;
    }

    return false;
}

void QtOgreViewer::UpdateFromModel()
{
#if 0
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
#endif
}

boost::shared_ptr<EnvironmentMutex::scoped_try_lock> QtOgreViewer::LockEnvironment(uint64_t timeout,bool bUpdateEnvironment)
{
#if 0
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
#endif
}

void QtOgreViewer::_UpdateEnvironment(float fTimeElapsed)
{
#if 0
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
#endif
}

void QtOgreViewer::_PostToGUIThread(const boost::function<void()>& fn, bool block)
{
    // if( _nQuitMainLoop != -1 ) {
    //     // viewer quit, so anything posted won't get processed
    //     return;
    // }

    // boost::mutex::scoped_lock lockmsg(_mutexGUIFunctions);
    // if( _listGUIFunctions.size() > 1000 ) {
    //     RAVELOG_WARN_FORMAT("too many gui post commands, ignoring %d", _listGUIFunctions.size());
    //     return;
    // }
    // GUIThreadFunctionPtr pfn(new GUIThreadFunction(fn, block));
    // _listGUIFunctions.push_back(pfn);
    // if( block ) {
    //     while(!pfn->IsFinished()) {
    //         _notifyGUIFunctionComplete.wait(_mutexGUIFunctions);
    //     }
    // }
}

void QtOgreViewer::SetEnvironmentSync(bool bUpdate)
{
    // boost::mutex::scoped_lock lockupdating(_mutexUpdating);
    // boost::mutex::scoped_lock lock(_mutexUpdateModels);
    // _bUpdateEnvironment = bUpdate;
    // _condUpdateModels.notify_all();

    // if( !bUpdate ) {
    //     // remove all messages in order to release the locks
    //     boost::mutex::scoped_lock lockmsg(_mutexGUIFunctions);
    //     if( _listGUIFunctions.size() > 0 ) {
    //         bool bnotify = false;
    //         FOREACH(it,_listGUIFunctions) {
    //             (*it)->SetFinished();
    //             if( (*it)->IsBlocking() ) {
    //                 bnotify = true;
    //             }
    //         }
    //         if( bnotify ) {
    //             _notifyGUIFunctionComplete.notify_all();
    //         }
    //     }
    //     _listGUIFunctions.clear();
    // }
}

void QtOgreViewer::_DeleteItemCallback(Item* pItem)
{
    boost::mutex::scoped_lock lock(_mutexItems);
    pItem->PrepForDeletion();
    _listRemoveItems.push_back(pItem);
}

void QtOgreViewer::EnvironmentSync()
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

UserDataPtr QtOgreViewer::RegisterItemSelectionCallback(const ItemSelectionCallbackFn& fncallback)
{
    ItemSelectionCallbackDataPtr pdata(new ItemSelectionCallbackData(fncallback,shared_viewer()));
    pdata->_iterator = _listRegisteredItemSelectionCallbacks.insert(_listRegisteredItemSelectionCallbacks.end(),pdata);
    return pdata;
}

UserDataPtr QtOgreViewer::RegisterViewerThreadCallback(const ViewerThreadCallbackFn& fncallback)
{
    ViewerThreadCallbackDataPtr pdata(new ViewerThreadCallbackData(fncallback,shared_viewer()));
    pdata->_iterator = _listRegisteredViewerThreadCallbacks.insert(_listRegisteredViewerThreadCallbacks.end(),pdata);
    return pdata;
}

ViewerBasePtr CreateQtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ViewerBasePtr(new QtOgreViewer(penv, sinput));
}

} // end namespace qtogrerave