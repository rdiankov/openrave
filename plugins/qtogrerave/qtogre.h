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

static float* FormatPoints(const float* ppoints, int numPoints, int stride, Ogre::Vector3 &min, Ogre::Vector3 &max) {
    // From my experience, graphics driver will convert vec3 to vec4 if vec3 is provided
    // TODO: Benchmark?
    // float max[3]; max[0] = max[1] = max[2] = std::numeric_limits<float>::min();
    // float min[3]; min[0] = min[1] = min[2] = std::numeric_limits<float>::min();
    float *vpoints = reinterpret_cast<float*>(OGRE_MALLOC_SIMD(
        4 * numPoints *  sizeof(float), Ogre::MEMCATEGORY_GEOMETRY));
    max[0] = max[1] = max[2] = std::numeric_limits<float>::min();
    min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
    for (int64_t i = 0; i < 4 * numPoints; i += 4) {
        vpoints[i] = ppoints[0];
        vpoints[i + 1] = ppoints[1];
        vpoints[i + 2] = ppoints[2];
        vpoints[i + 3] = 1.0f;
        // TODO: vector?
        max[0] = std::max(max[0], ppoints[0]);
        max[1] = std::max(max[1], ppoints[1]);
        max[2] = std::max(max[2], ppoints[2]);
        min[0] = std::min(min[0], ppoints[0]);
        min[1] = std::min(min[1], ppoints[1]);
        min[2] = std::min(min[2], ppoints[2]);
        ppoints = (float*)((char*)ppoints + stride);
    }

    return vpoints;
}

}; // namespace qtogrerave

#endif