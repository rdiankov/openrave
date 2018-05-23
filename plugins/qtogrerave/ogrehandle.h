#ifndef OPENRAVE_OGRENODEHANDLE_H_
#define OPENRAVE_OGRENODEHANDLE_H_

#include "qtogre.h"
#include <vector>

namespace qtogrerave {

class OgreNodeHandle : public boost::enable_shared_from_this<OgreNodeHandle>, public OpenRAVE::UserData
{
public:
    OgreNodeHandle(Ogre::Root *root, Ogre::SceneNode *parentNode, OpenRAVE::KinBody &body);
    virtual ~OgreNodeHandle();

    void Update(const OpenRAVE::KinBody &body);
private:
    Ogre::Root *_root;
    Ogre::SceneNode *_node; // Node that contains the KinBody
    std::vector<Ogre::String> _materialNames;
};

class OgreGraphHandle : public OpenRAVE::GraphHandle {
public:
    OgreGraphHandle() : _node(nullptr) {}
    OgreGraphHandle(Ogre::SceneNode *node) : _node(node) {}
    virtual ~OgreGraphHandle() {
        if (_node) {
            _node->getParentSceneNode()->removeAndDestroyChild(_node);
        }
    }

    // PrivateGraphHandle(QtOgreViewerWeakPtr wviewer, OgreSwitchPtr handle) : _handle(handle), _wviewer(wviewer) {
    //         BOOST_ASSERT(_handle != NULL);
    //     }
    //     virtual ~PrivateGraphHandle() {
    //         boost::shared_ptr<QtOgreViewer> viewer = _wviewer.lock();
    //         if(!!viewer) {
    //             viewer->_PostToGUIThread(boost::bind(&QtOgreViewer::_CloseGraphHandle, viewer, _handle)); // _handle is copied, so it will maintain the reference
    //         }
    //     }

    virtual void SetTransform(const OpenRAVE::RaveTransform<float>& t)
    {
        // boost::shared_ptr<QtOSGViewer> viewer = _wviewer.lock();
        // if(!!viewer) {
        //     viewer->_PostToGUIThread(boost::bind(&QtOSGViewer::_SetGraphTransform, viewer, _handle, t)); // _handle is copied, so it will maintain the reference
        // }
        SetOgreNodeTransform(_node, t);
    }

    virtual void SetShow(bool bShow)
    {
        // boost::shared_ptr<QtOSGViewer> viewer = _wviewer.lock();
        // if(!!viewer) {
        //     viewer->_PostToGUIThread(boost::bind(&QtOSGViewer::_SetGraphShow, viewer, _handle, bShow)); // _handle is copied, so it will maintain the reference
        // }
        _node->setVisible(bShow);
    }
// private:
    Ogre::SceneNode *_node;
    // QtOgreViewerWeakPtr _wviewer;
};

typedef boost::shared_ptr<OgreNodeHandle> OgreNodeHandlePtr;
typedef boost::shared_ptr<OgreGraphHandle> OgreGraphHandlePtr;

};

#endif // OPENRAVE_OGRERANDOMOgreNodeHandle_H_