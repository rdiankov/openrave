#include "qtogreviewer.h"
#include "ogrehandle.h"

#include <condition_variable>
#include <limits>
#include <mutex>
#include <ctime>
#include <string>

#include <OGRE/OgreMeshManager2.h>
#include <OGRE/OgreMesh2.h>
#include <OGRE/OgreItem.h>
#include <OGRE/OgreSubMesh2.h>
#include <OGRE/Hlms/Pbs/OgreHlmsPbs.h>
#include <OGRE/Hlms/Pbs/OgreHlmsPbsDatablock.h>

namespace qtogrerave {

QtOgreViewer::QtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput) : ViewerBase(penv) {
    __description = ":Interface Author: Woody Chow\n\nQt/Ogre Viewer";

    RegisterCommand("StartViewerLoop", boost::bind(&QtOgreViewer::startmainloop, this, _1, _2),
                    "starts the viewer sync loop and shows the viewer. expects someone else will call the qapplication exec fn");

    _userdatakey = std::string("qtogre") + std::to_string(reinterpret_cast<uintptr_t>(this));
}

QtOgreViewer::~QtOgreViewer() {
    quitmainloop();
}

int QtOgreViewer::main(bool bShow)
{
    if( !QApplication::instance() ) {
        throw OPENRAVE_EXCEPTION_FORMAT0("need a valid QApplication before viewer loop is run", ORE_InvalidState);
    }

    // TODO: Take care of bshow
    _ogreWindow = boost::make_shared<QtOgreWindow>([this](){_EnvironmentUpdate();});
    // _ogreWindow->show();
    _ogreWindow->showMaximized();
    QGuiApplication::instance()->exec();
}

bool QtOgreViewer::startmainloop(std::ostream& sout, std::istream& sinput)
{
    QGuiApplication::instance()->exec();
}

void QtOgreViewer::quitmainloop()
{
    QApplication::quit();
}

void QtOgreViewer::_EnvironmentUpdate()
{
    // boost::mutex::scoped_lock lock(_mutexUpdateModels);
    std::vector<KinBody::BodyState> vecbodies;

#if BOOST_VERSION >= 103500
    EnvironmentMutex::scoped_try_lock lockenv(GetEnv()->GetMutex(),boost::defer_lock_t());
#else
    EnvironmentMutex::scoped_try_lock lockenv(GetEnv()->GetMutex(),false);
#endif

    if( /*_bLockEnvironment && */!lockenv ) {
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

    Ogre::SceneNode* envNode = _ogreWindow->GetEnvNode();

    // Don't need to lock anymore?
    for (const KinBody::BodyState &bs: vecbodies) {
        KinBodyPtr pbody = bs.pbody; // try to use only as an id, don't call any methods!
        OgreNodeHandlePtr pNodeHandle = boost::dynamic_pointer_cast<OgreNodeHandle>(pbody->GetUserData(_userdatakey));

        if (pNodeHandle.get() != nullptr) {
            pNodeHandle->Update(*pbody);
        } else {
            pNodeHandle = boost::make_shared<OgreNodeHandle>(_ogreWindow->GetRoot(), envNode, *pbody);
            pbody->SetUserData(_userdatakey, pNodeHandle);

            _mKinbodyNode[pbody.get()] = pNodeHandle; // memory leak?????
        }
    }

    // TODO~~~~~~~~``
    // FOREACH_NOINC(it, _mapbodies) {
    //     if( !it->second->GetUserData() ) {
    //         // item doesn't exist anymore, remove it
    //         it->first->RemoveUserData(_userdatakey);

    //         if( _pSelectedItem == it->second ) {
    //             // TODO
    //             //_pdragger.reset();
    //             _pSelectedItem.reset();
    //             //_osgViewerRoot->deselectAll();
    //         }

    //         _mapbodies.erase(it++);
    //     }
    //     else {
    //         ++it;
    //     }
    // }
}

GraphHandlePtr QtOgreViewer::plot3(const float* ppoints, int nPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
{
    Ogre::Vector3 min, max;
    float* vpoints = FormatPoints(ppoints, nPoints, stride, min, max);

    std::mutex cv_m;
    std::condition_variable cv;
    OgreGraphHandlePtr handle = boost::make_shared<OgreGraphHandle>();

    _ogreWindow->QueueRenderingUpdate([this, &cv_m, &cv, &handle, vpoints, nPoints, stride, fPointSize, color, drawstyle, &min, &max]() {
        std::lock_guard<std::mutex> lk(cv_m);
        Ogre::RenderSystem *renderSystem = _ogreWindow->GetRoot()->getRenderSystem();
        Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

        Ogre::VertexBufferPacked* vertexBuffer = CreatePointsBuffer(vaoManager, nPoints, vpoints, Ogre::VES_POSITION);

        Ogre::VertexArrayObject* vao = vaoManager->createVertexArrayObject(
            {vertexBuffer},
            nullptr, // Points do not need index buffer
            Ogre::OT_POINT_LIST);

        Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode(); // _ogreWindow->GetRootSceneNode();//
        Ogre::SceneNode* node = parentNode->createChildSceneNode();
        static uint32_t count = 0;
        Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
            "plot3" + std::to_string(count++),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        Ogre::SubMesh* submesh = mesh->createSubMesh();
        submesh->mVao[Ogre::VpNormal].push_back(vao);
        submesh->mVao[Ogre::VpShadow].push_back(vao);
        //Set the bounds to get frustum culling and LOD to work correctly.
        Ogre::Aabb aabb = Ogre::Aabb::newFromExtents(min, max);
        mesh->_setBounds(aabb, false);
        mesh->_setBoundingSphereRadius(aabb.getRadius());
        Ogre::SceneManager *sceneManager = node->getCreator();
        Ogre::Item *item = sceneManager->createItem(mesh);
        item->setDatablock(_ogreWindow->datablockhack);
        node->attachObject(item);
        node->setPosition(0.0f, 0.0f, 0.0f);
        handle->_node = node;

        cv.notify_all();
    });

    {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk);
    }

    return handle;
}

GraphHandlePtr QtOgreViewer::drawlinestrip(const float* ppoints, int nPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    Ogre::Vector3 min, max;
    float* vpoints = FormatPoints(ppoints, nPoints, stride, min, max);

    std::mutex cv_m;
    std::condition_variable cv;
    OgreGraphHandlePtr handle = boost::make_shared<OgreGraphHandle>();

    _ogreWindow->QueueRenderingUpdate([this, &cv_m, &cv, &handle, vpoints, nPoints, stride, fwidth, color]() {
        Ogre::RenderSystem *renderSystem = _ogreWindow->GetRoot()->getRenderSystem();
        Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

        Ogre::VertexBufferPacked* vertexBuffer = CreatePointsBuffer(vaoManager, nPoints, vpoints, Ogre::VES_POSITION);

        Ogre::VertexArrayObject* vao = vaoManager->createVertexArrayObject(
            {vertexBuffer},
            nullptr, // Do not need index buffer
            Ogre::OT_LINE_STRIP);

        Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode();
        Ogre::SceneNode* node = parentNode->createChildSceneNode();
        static uint32_t count = 0;
        Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
            "drawlinestrip" + std::to_string(count++),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        Ogre::SubMesh* submesh = mesh->createSubMesh();
        submesh->mVao[Ogre::VpNormal].push_back(vao);
        submesh->mVao[Ogre::VpShadow].push_back(vao);
        //Set the bounds to get frustum culling and LOD to work correctly.
        // mesh->_setBounds( Ogre::Aabb( Ogre::Vector3::ZERO, Ogre::Vector3(256, 128, 256)), false );
        // mesh->_setBoundingSphereRadius( 128.0f );
        Ogre::SceneManager *sceneManager = node->getCreator();
        Ogre::Item *item = sceneManager->createItem(mesh);
        node->attachObject(item);
        handle->_node = node;
        cv.notify_all();
    });

    {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk);
    }

    return handle;
}

GraphHandlePtr QtOgreViewer::drawlinelist(const float* ppoints, int nPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    // From my experience, graphics driver will convert vec3 to vec4 if vec3 is provided
    // TODO: Benchmark?
    std::vector<float> vpoints(4 * nPoints);
    for (int64_t i = 0; i < 4 * nPoints; i += 4) {
        vpoints[i] = ppoints[0];
        vpoints[i + 1] = ppoints[1];
        vpoints[i + 2] = ppoints[2];
        vpoints[i + 3] = 1.0f;
        ppoints = (float*)((char*)ppoints + stride);
    }

    _ogreWindow->QueueRenderingUpdate([this, vpoints, nPoints, stride, fwidth, color]() {
        Ogre::RenderSystem *renderSystem = _ogreWindow->GetRoot()->getRenderSystem();
        Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();

        Ogre::VertexElement2Vec vertexElements;
        vertexElements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT4, Ogre::VES_POSITION));

        Ogre::VertexBufferPacked* vertexBuffer = nullptr;
        try
        {
            //Create the actual vertex buffer.
            vertexBuffer = vaoManager->createVertexBuffer(
                vertexElements, nPoints,
                Ogre::BT_IMMUTABLE,
                const_cast<float*>(vpoints.data()), // const when keepAsShadow is false
                false                               // keepAsShadow
            );
        }
        catch(Ogre::Exception &e)
        {
            OGRE_FREE_SIMD(vertexBuffer, Ogre::MEMCATEGORY_GEOMETRY);
            vertexBuffer = nullptr;
            throw e; // TODO: throw openrave exception
        }

        Ogre::VertexArrayObject* vao = vaoManager->createVertexArrayObject(
            {vertexBuffer},
            nullptr, // Do not need index buffer
            Ogre::OT_LINE_LIST);

        Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode();
        Ogre::SceneNode* node = parentNode->createChildSceneNode();
        static uint32_t count = 0;
        Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(
            "linelist" + std::to_string(count++),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        Ogre::SubMesh* submesh = mesh->createSubMesh();
        submesh->mVao[Ogre::VpNormal].push_back(vao);
        submesh->mVao[Ogre::VpShadow].push_back(vao);
        //Set the bounds to get frustum culling and LOD to work correctly.
        // mesh->_setBounds( Ogre::Aabb( Ogre::Vector3::ZERO, Ogre::Vector3(256, 128, 256)), false );
        // mesh->_setBoundingSphereRadius( 128.0f );
        Ogre::SceneManager *sceneManager = node->getCreator();
        Ogre::Item *item = sceneManager->createItem(mesh);
        node->attachObject(item);
        // *handle = OgreGraphHandle(node); // fix later
    });
}



GraphHandlePtr QtOgreViewer::drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
{
    std::mutex cv_m;
    std::condition_variable cv;
    OgreGraphHandlePtr handle = boost::make_shared<OgreGraphHandle>();

    _ogreWindow->QueueRenderingUpdate([this, &cv_m, &cv, &handle, &vpos, &vextents]() {
        Ogre::HlmsManager *hlmsManager = _ogreWindow->GetRoot()->getHlmsManager();
        // assert(dynamic_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms( Ogre::HLMS_PBS ) ) );
        Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms(Ogre::HLMS_PBS) );
        static uint32_t count = 0;
        const std::string datablockName = "box" + std::to_string(count++);
        Ogre::HlmsPbsDatablock *datablock = static_cast<Ogre::HlmsPbsDatablock*>(
            hlmsPbs->createDatablock(datablockName, datablockName,
                                     Ogre::HlmsMacroblock(),
                                     Ogre::HlmsBlendblock(),
                                     Ogre::HlmsParamVec()));
        datablock->setDiffuse(Ogre::Vector3(1.0f, 0.0f, 0.0f));

        Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode();
        Ogre::SceneNode* node = parentNode->createChildSceneNode();
        Ogre::v1::Entity* cube = node->getCreator()->createEntity(Ogre::SceneManager::PT_CUBE);
        // cube->setDatablock(_ogreWindow->datablockhack);
        cube->setDatablock(datablock);
        node->attachObject(cube);
        node->setPosition(Ogre::Vector3(vpos.x, vpos.y, vpos.z));
        node->setScale(Ogre::Vector3(vextents.x, vextents.y, vextents.z)); // <--------- is this extents?
        handle->_node = node;

        cv.notify_all();
    });

    {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait(lk);
    }

    return handle;
}

ViewerBasePtr CreateQtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return boost::make_shared<QtOgreViewer>(penv, sinput);
}

}; // namespace qtogrerave