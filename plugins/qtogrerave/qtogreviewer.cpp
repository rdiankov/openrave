#include "qtogreviewer.h"

namespace qtogrerave {

QtOgreViewer::QtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput) : ViewerBase(penv) {
    __description = ":Interface Author: Woody Chow\n\nQt/Ogre Viewer";

    RegisterCommand("StartViewerLoop", boost::bind(&QtOgreViewer::startmainloop, this, _1, _2),
                    "starts the viewer sync loop and shows the viewer. expects someone else will call the qapplication exec fn");
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
    _ogreWindow = boost::make_shared<QtOgreWindow>();
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

GraphHandlePtr QtOgreViewer::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
{
    Ogre::RenderSystem *renderSystem = _ogreWindow->GetRoot()->getRenderSystem();
    Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();
    

    // --------------------------------------------
    Ogre::VertexData* data = new Ogre::VertexData();
    data->vertexCount = numPoints;
    Ogre::VertexDeclaration* decl = data->vertexDeclaration;
    decl->addElement(0, stride - 3 * sizeof(float), Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    // Thread safe?
    Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        decl->getVertexSize(0),                     // This value is the size of a vertex in memory
        numPoints,                                  // The number of vertices you'll put into this buffer
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
    );
    // float *array
    vbuf->writeData(0, vbuf->getSizeInBytes(), ppoints, true);
    Ogre::VertexBufferBinding* bind = data->vertexBufferBinding;
    bind->setBinding(0, vbuf);

    _ogreWindow->QueueRenderingUpdate(boost::make_shared<QtOgreWindow::GUIThreadFunction>([this, ppoints, numPoints, stride, fPointSize, color, drawstyle]() {
        Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode();
        Ogre::SceneNode* node = parentNode->createChildSceneNode();
        // Do the mesh and mesh group name have to be unique?
        Ogre::Mesh* mesh = Ogre::MeshManager::getSingleton().createManual("plot3", "main");
        Ogre::SubMesh* submesh = mMesh->createSubMesh();
        submesh->useSharedVertices = false;
        submesh->vertexData = data;
        submesh->indexData->indexCount = 0; // Points do not need index buffer
        submesh->operationType = OT_POINT_LIST;
        mesh->load();
        node->attachObject(mesh);
        // *handle = OgreHandle(node); // fix later
    }));    
}

GraphHandlePtr QtOgreViewer::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    Ogre::VertexData* data = new Ogre::VertexData();
    data->vertexCount = numPoints;
    Ogre::VertexDeclaration* decl = data->vertexDeclaration;
    decl->addElement(0, stride - 3 * sizeof(float), Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    // Thread safe?
    HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        decl->getVertexSize(0),                     // This value is the size of a vertex in memory
        numPoints,                                  // The number of vertices you'll put into this buffer
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
    );
    // float *array
    vbuf->writeData(0, vbuf->getSizeInBytes(), ppoints, true);
    Ogre::VertexBufferBinding* bind = data->vertexBufferBinding;
    bind->setBinding(0, vbuf);

    _ogreWindow->QueueRenderingUpdate(boost::make_shared<QtOgreWindow::GUIThreadFunction>([this, points, numPoints, stride, fPointSize, color, drawstyle]() {
        Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode();
        Ogre::SceneNode* node = parentNode->createChildSceneNode();
        // Do the mesh and mesh group name have to be unique?
        Ogre::Mesh* mesh = Ogre::MeshManager::getSingleton().createManual("linelist", "main");
        Ogre::SubMesh* submesh = mMesh->createSubMesh();
        submesh->useSharedVertices = false;
        submesh->vertexData = data;
        submesh->indexData->indexCount = 0; // Points do not need index buffer
        submesh->operationType = OT_LINE_STRIP;
        mesh->load();
        node->attachObject(mesh);

        // VET_COLOUR, VES_DIFFUSE
        // *handle = OgreHandle(node); // fix later
    }));
}

GraphHandlePtr QtOgreViewer::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    Ogre::VertexData* data = new Ogre::VertexData();
    data->vertexCount = numPoints;
    Ogre::VertexDeclaration* decl = data->vertexDeclaration;
    decl->addElement(0, stride - 3 * sizeof(float), Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    // Thread safe?
    HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        decl->getVertexSize(0),                     // This value is the size of a vertex in memory
        numPoints,                                  // The number of vertices you'll put into this buffer
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
    );
    // float *array
    vbuf->writeData(0, vbuf->getSizeInBytes(), ppoints, true);
    Ogre::VertexBufferBinding* bind = data->vertexBufferBinding;
    bind->setBinding(0, vbuf);

    _ogreWindow->QueueRenderingUpdate(boost::make_shared<QtOgreWindow::GUIThreadFunction>([this, points, numPoints, stride, fPointSize, color, drawstyle]() {
        Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode();
        Ogre::SceneNode* node = parentNode->createChildSceneNode();
        // Do the mesh and mesh group name have to be unique?
        Ogre::Mesh* mesh = Ogre::MeshManager::getSingleton().createManual("linelist", "main");
        Ogre::SubMesh* submesh = mMesh->createSubMesh();
        submesh->useSharedVertices = false;
        submesh->vertexData = data;
        submesh->indexData->indexCount = 0; // Points do not need index buffer
        submesh->operationType = OT_LINE_LIST;
        mesh->load();
        node->attachObject(mesh);

        // VET_COLOUR, VES_DIFFUSE
        // *handle = OgreHandle(node); // fix later
    }));
}

GraphHandlePtr QtOgreViewer::drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
{
    _ogreWindow->QueueRenderingUpdate(boost::make_shared<QtOgreWindow::GUIThreadFunction>([this, &vpos, &vextents]() {
        // TODO: Use vertex buffer?
        Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode();
        Ogre::SceneNode* node = parentNode->createChildSceneNode();
        Ogre::v1::Entity* cube = node->getCreator()->createEntity(Ogre::SceneManager::PT_CUBE);
        cube->setDatablock(_ogreWindow->datablockhack);
        node->attachObject(cube);
        node->setPosition(Ogre::Vector3(vpos.x, vpos.y, vpos.z));
        node->setScale(Ogre::Vector3(vextents.x, vextents.y, vextents.z)); // <--------- is this extents?
        // *handle = OgreHandle(node); // fix later
    }));
    // Ogre::SceneNode* parentNode = _ogreWindow->GetMiscDrawNode();
    //     Ogre::SceneNode* node = parentNode->createChildSceneNode();
    //     Ogre::v1::Entity* cube = node->getCreator()->createEntity(Ogre::SceneManager::PT_CUBE);
    //     cube->setDatablock(_ogreWindow->datablockhack);
    //     node->attachObject(cube);
    //     node->setPosition(Ogre::Vector3(vpos.x, vpos.y, vpos.z));
    //     node->setScale(Ogre::Vector3(vextents.x, vextents.y, vextents.z)); // <--------- is this extents?

    //return boost::make_shared<OgreHandle>(node);
    //return boost::make_shared<OgreHandle>(node);
}

ViewerBasePtr CreateQtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return boost::make_shared<QtOgreViewer>(penv, sinput);
}

}; // namespace qtogrerave