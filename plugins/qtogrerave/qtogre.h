#ifndef OPENRAVE_QTOGRE_H_
#define OPENRAVE_QTOGRE_H_

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions
#include <openrave/utils.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Ogre.h>

namespace qtogrerave {

template <typename T>
static void SetOgreNodeTransform(Ogre::SceneNode *node, const OpenRAVE::RaveTransform<T>& t)
{
    node->setOrientation(t.rot.x, t.rot.y, t.rot.z, t.rot.w);
    node->setPosition(t.trans.x, t.trans.y, t.trans.z);
}

}; // namespace qtogrerave

#endif