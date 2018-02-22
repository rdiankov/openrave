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