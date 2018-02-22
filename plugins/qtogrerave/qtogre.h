#ifndef OPENRAVE_QTOGRE_H_
#define OPENRAVE_QTOGRE_H_

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions
#include <openrave/utils.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Ogre.h>

namespace qtogrerave {

class OgreHandle : public OpenRAVE::GraphHandle {
public:
    OgreHandle() : _node(nullptr) {}
    OgreHandle(Ogre::SceneNode *node) : _node(node) {}
    ~OgreHandle() {
        _node->getParentSceneNode()->removeAndDestroyChild(_node);
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
        _node->setOrientation(t.rot.x, t.rot.y, t.rot.z, t.rot.w);
        _node->setPosition(t.trans.x, t.trans.y, t.trans.z);
    }

    virtual void SetShow(bool bShow)
    {
        // boost::shared_ptr<QtOSGViewer> viewer = _wviewer.lock();
        // if(!!viewer) {
        //     viewer->_PostToGUIThread(boost::bind(&QtOSGViewer::_SetGraphShow, viewer, _handle, bShow)); // _handle is copied, so it will maintain the reference
        // }
        _node->setVisible(bShow);
    }
private:
    Ogre::SceneNode *_node;
    // QtOgreViewerWeakPtr _wviewer;
};

typedef boost::shared_ptr<OgreHandle> OgreHandlePtr;

}; // namespace qtogrerave

#endif