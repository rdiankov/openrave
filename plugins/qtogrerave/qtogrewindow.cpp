#include <openrave/openrave.h>

#include <OGRE/Hlms/Pbs/OgreHlmsPbs.h>
#include "qtogrewindow.h"
#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
#include <OGRE/Compositor/OgreCompositorManager2.h>
#endif

/*
Note that we pass any supplied QWindow parent to the base QWindow class. This is necessary should we
need to use our class within a container.
*/
QtOgreWindow::QtOgreWindow(const std::function<void()> &environmentUpdateFunc, QWindow *parent)
    : QWindow(parent)
    , _environmentUpdateFunc(environmentUpdateFunc)
    , m_update_pending(false)
    , m_animating(false)
    , m_ogreRoot(NULL)
    , m_ogreWindow(NULL)
    , m_ogreCamera(NULL)
    , m_cameraMan(NULL)
{
    setAnimating(true);
    installEventFilter(this);
    m_ogreBackground = Ogre::ColourValue(1.0f, 1.0f, 1.0f);
}

/*
Upon destruction of the QWindow object we destroy the Ogre3D scene.
*/
QtOgreWindow::~QtOgreWindow()
{
    _frameRenderingUpdateQueue.clear();
    if (m_cameraMan) delete m_cameraMan;
    delete m_ogreRoot;
}

/*
In case any drawing surface backing stores (QRasterWindow or QOpenGLWindow) of Qt are supplied to this
class in any way we inform Qt that they will be unused.
*/
void QtOgreWindow::render(QPainter *painter)
{
    Q_UNUSED(painter);
}

/*
Our initialization function. Called by our renderNow() function once when the window is first exposed.
*/
void QtOgreWindow::initialize()
{
    /*
    As shown Ogre3D is initialized normally; just like in other documentation.
    */
    m_ogreRoot = new Ogre::Root();
    m_ogreRoot->loadPlugin("RenderSystem_GL3Plus");
    /*

    Commended out for simplicity but should you need to initialize resources you can do so normally.
    Ogre::ConfigFile ogreConfig;
    ogreConfig.load("resources/resource_configs/resources.cfg");

    Ogre::ConfigFile::SectionIterator seci = ogreConfig.getSectionIterator();
    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            archName, typeName, secName);
        }
    }

    */

    const Ogre::RenderSystemList& rsList = m_ogreRoot->getAvailableRenderers();
    if (rsList.empty()){
        throw OPENRAVE_EXCEPTION_FORMAT0("Cannot get any renderers. Do you have plugins.cfg in your directory? TODO: Fix this", OpenRAVE::ORE_Failed);
    }
    Ogre::RenderSystem* rs = rsList[0];

    /*
    This list setup the search order for used render system.
    */
    Ogre::StringVector renderOrder;
#if defined(Q_OS_WIN)
    renderOrder.push_back("Direct3D11");
    renderOrder.push_back("Direct3D9");
#endif
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
                    "QtOgreWindow::initialize");
        }
    }

    /*
    Setting size and VSync on windows will solve a lot of problems
    */
    QString dimensions = QString("%1 x %2").arg(this->width()).arg(this->height());
    rs->setConfigOption("Video Mode", dimensions.toStdString());
    rs->setConfigOption("Full Screen", "No");
    rs->setConfigOption("VSync", "Yes");
    m_ogreRoot->setRenderSystem(rs);
    m_ogreRoot->initialise(false);

    Ogre::NameValuePairList parameters;
    /*
    Flag within the parameters set so that Ogre3D initializes an OpenGL context on it's own.
    */
    if (rs->getName().find("GL") <= rs->getName().size())
        parameters["currentGLContext"] = Ogre::String("false");

    /*
    We need to supply the low level OS window handle to this QWindow so that Ogre3D knows where to draw
    the scene. Below is a cross-platform method on how to do this.
    If you set both options (externalWindowHandle and parentWindowHandle) this code will work with OpenGL
    and DirectX.
    */
#if defined(Q_OS_MAC) || defined(Q_OS_WIN)
    parameters["externalWindowHandle"] = Ogre::StringConverter::toString((size_t)(this->winId()));
    parameters["parentWindowHandle"] = Ogre::StringConverter::toString((size_t)(this->winId()));
#else
    parameters["externalWindowHandle"] = Ogre::StringConverter::toString((unsigned long)(this->winId()));
    parameters["parentWindowHandle"] = Ogre::StringConverter::toString((unsigned long)(this->winId()));
#endif

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

    /*
    The rest of the code in the initialization function is standard Ogre3D scene code. Consult other
    tutorials for specifics.
    */
#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
    const size_t numThreads = std::max<int>(1, Ogre::PlatformInformation::getNumLogicalCores());
    Ogre::InstancingThreadedCullingMethod threadedCullingMethod = Ogre::INSTANCING_CULLING_SINGLETHREAD;
    if (numThreads > 1)threadedCullingMethod = Ogre::INSTANCING_CULLING_THREADED;
    m_ogreSceneMgr = m_ogreRoot->createSceneManager(Ogre::ST_GENERIC, numThreads, threadedCullingMethod);
#else
    m_ogreSceneMgr = m_ogreRoot->createSceneManager(Ogre::ST_GENERIC);
#endif
    
    m_ogreCamera = m_ogreSceneMgr->createCamera("MainCamera");
    m_ogreCamera->setPosition(Ogre::Vector3(0.0f, 0.0f, 5.0f));
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
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups(true);

    createScene();

    m_ogreRoot->addFrameListener(this);
    m_ogreSceneMgr->getRootSceneNode()->setName("root");
    m_miscDrawNode = m_ogreSceneMgr->getRootSceneNode()->createChildSceneNode();
    m_envNode = m_ogreSceneMgr->getRootSceneNode()->createChildSceneNode();
}

void QtOgreWindow::createScene()
{
    /*
    Example scene
    Derive this class for your own purpose and overwite this function to have a working Ogre widget with
    your own content.
    */
    m_ogreSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f), Ogre::ColourValue(0.5f, 0.5f, 0.5f), Ogre::Vector3(0.0f, 0.0f, 1.0f));

// >>>>>>>>>>>>>>>> hack
    Ogre::Archive *archiveLibrary = Ogre::ArchiveManager::getSingletonPtr()->load(
                "/home/mujin/mujin/checkoutroot/ogre/Samples/Media/Hlms/Common/GLSL",
                "FileSystem", true );
    Ogre::ArchiveVec library;
    library.push_back( archiveLibrary );
    Ogre::Archive *archivePbs = Ogre::ArchiveManager::getSingletonPtr()->load(
                    "/home/mujin/mujin/checkoutroot/ogre/Samples/Media/Hlms/Pbs/GLSL",
                    "FileSystem", true );
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< delete?????????
    Ogre::HlmsPbs *hlmsPbs = OGRE_NEW Ogre::HlmsPbs( archivePbs, &library );
    m_ogreRoot->getHlmsManager()->registerHlms( hlmsPbs );

    Ogre::Light *light = m_ogreSceneMgr->createLight();
    Ogre::SceneNode *lightNode = m_ogreSceneMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    light->setPowerScale( Ogre::Math::PI ); //Since we don't do HDR, counter the PBS' division by PI
    light->setType( Ogre::Light::LT_DIRECTIONAL );
    light->setDirection( Ogre::Vector3( -1, -1, -1 ).normalisedCopy() );
}

void QtOgreWindow::createCompositor()
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

void QtOgreWindow::render()
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

void QtOgreWindow::renderLater()
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

bool QtOgreWindow::event(QEvent *event)
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
void QtOgreWindow::exposeEvent(QExposeEvent *event)
{
    Q_UNUSED(event);

    if (isExposed())
        renderNow();
}

/*
The renderNow() function calls the initialize() function when needed and if the QWindow is already
initialized and prepped calls the render() method.
*/
void QtOgreWindow::renderNow()
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
bool QtOgreWindow::eventFilter(QObject *target, QEvent *event)
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
void QtOgreWindow::keyPressEvent(QKeyEvent * ev)
{
    if(m_cameraMan)
        m_cameraMan->injectKeyDown(*ev);
}

void QtOgreWindow::keyReleaseEvent(QKeyEvent * ev)
{
    if(m_cameraMan)
        m_cameraMan->injectKeyUp(*ev);
}

void QtOgreWindow::mouseMoveEvent( QMouseEvent* e )
{
    static int lastX = e->x();
    static int lastY = e->y();
    int relX = e->x() - lastX;
    int relY = e->y() - lastY;
    lastX = e->x();
    lastY = e->y();

    if(m_cameraMan && (e->buttons() & Qt::LeftButton))
        m_cameraMan->injectMouseMove(relX, relY);

    // Experimental ray intersection code
    QPoint pos = e->pos();
    Ogre::Ray mouseRay = m_ogreCamera->getCameraToViewportRay(
        (Ogre::Real)pos.x() / m_ogreWindow->getWidth(),
        (Ogre::Real)pos.y() / m_ogreWindow->getHeight());
    Ogre::RaySceneQuery* pSceneQuery = m_ogreSceneMgr->createRayQuery(mouseRay);
    pSceneQuery->setSortByDistance(true);
    Ogre::RaySceneQueryResult vResult = pSceneQuery->execute();
    for (size_t ui = 0; ui < vResult.size(); ui++) {
        const Ogre::RaySceneQueryResultEntry &result = vResult[ui];
        if (result.movable) {
            // Transform intersection from camera space to world space
            Ogre::Matrix4 transf;
            m_ogreCamera->getWorldTransforms(&transf);
            //const Ogre::Vector3 intersection = transf.inverseAffine() * mouseRay.getPoint(result.distance);
            const Ogre::Vector3 intersection = mouseRay.getPoint(result.distance);

            printf("%s (%f %f %f)\n", vResult[ui].movable->getParentSceneNode()->getName().c_str(),
                   intersection[0], intersection[1], intersection[2]);
        }
    }
    // for (size_t ui = 0; ui < vResult.size(); ui++)
    // {
    //     if (vResult[ui].movable)
    //     {
    //         if (vResult[ui].movable->getMovableType().compare("Entity") == 0)
    //         {
    //             emit entitySelected((Ogre::v1::Entity*)vResult[ui].movable);
    //         }
    //     }
    // }
    m_ogreSceneMgr->destroyQuery(pSceneQuery);
}

void QtOgreWindow::wheelEvent(QWheelEvent *e)
{
    if(m_cameraMan)
        m_cameraMan->injectWheelMove(*e);
}

void QtOgreWindow::mousePressEvent( QMouseEvent* e )
{
    if(m_cameraMan)
        m_cameraMan->injectMouseDown(*e);
}

void QtOgreWindow::mouseReleaseEvent( QMouseEvent* e )
{
    if(m_cameraMan)
        m_cameraMan->injectMouseUp(*e);

    QPoint pos = e->pos();
    Ogre::Ray mouseRay = m_ogreCamera->getCameraToViewportRay(
        (Ogre::Real)pos.x() / m_ogreWindow->getWidth(),
        (Ogre::Real)pos.y() / m_ogreWindow->getHeight());
    Ogre::RaySceneQuery* pSceneQuery = m_ogreSceneMgr->createRayQuery(mouseRay);
    pSceneQuery->setSortByDistance(true);
    Ogre::RaySceneQueryResult vResult = pSceneQuery->execute();
    for (size_t ui = 0; ui < vResult.size(); ui++)
    {
        if (vResult[ui].movable)
        {
            if (vResult[ui].movable->getMovableType().compare("Entity") == 0)
            {
                emit entitySelected((Ogre::v1::Entity*)vResult[ui].movable);
            }
        }
    }
    m_ogreSceneMgr->destroyQuery(pSceneQuery);
}

/*
Function to keep track of when we should and shouldn't redraw the window; we wouldn't want to do
rendering when the QWindow is minimized. This takes care of those scenarios.
*/
void QtOgreWindow::setAnimating(bool animating)
{
    m_animating = animating;

    if (animating)
        renderLater();
}

bool QtOgreWindow::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    m_cameraMan->frameRenderingQueued(evt);
    std::list<std::function<void()>> localQueue;
    {
        boost::mutex::scoped_lock lock(_mutexFrameRenderingUpdate);
        localQueue = std::move(_frameRenderingUpdateQueue);
    }
    for (std::function<void()>& func: localQueue) {
        func();
    }
    _environmentUpdateFunc();

    return true;
}
