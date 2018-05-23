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
    float *vpoints = reinterpret_cast<float*>(OGRE_MALLOC_SIMD(
        4 * numPoints *  sizeof(float), Ogre::MEMCATEGORY_GEOMETRY));
    max[0] = max[1] = max[2] = std::numeric_limits<float>::min();
    min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
    for (size_t i = 0; i < 4 * numPoints; i += 4) {
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

static float* FormatPoints(const std::vector<OpenRAVE::Vector> &vertices, Ogre::Vector3 &min, Ogre::Vector3 &max) {
    // From my experience, graphics driver will convert vec3 to vec4 if vec3 is provided
    // TODO: Benchmark?
    float *vpoints = reinterpret_cast<float*>(OGRE_MALLOC_SIMD(
        4 * vertices.size() *  sizeof(float), Ogre::MEMCATEGORY_GEOMETRY));
    max[0] = max[1] = max[2] = std::numeric_limits<float>::min();
    min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
    for (size_t i = 0, j = 0; i < vertices.size(); ++i, j += 4) {
        const OpenRAVE::Vector &v = vertices[i];
        vpoints[j] = v[0];
        vpoints[j + 1] = v[1];
        vpoints[j + 2] = v[2];
        vpoints[j + 3] = 1.0f;
        max[0] = std::max(max[0], vpoints[j]);
        max[1] = std::max(max[1], vpoints[j + 1]);
        max[2] = std::max(max[2], vpoints[j + 2]);
        min[0] = std::min(min[0], vpoints[j]);
        min[1] = std::min(min[1], vpoints[j + 1]);
        min[2] = std::min(min[2], vpoints[j + 2]);
    }
    return vpoints;
}

static Ogre::VertexBufferPacked* CreateBufferPacked(
    Ogre::VaoManager* vaoManager,
    const std::vector<OpenRAVE::Vector> &vertices,
    const std::vector<int32_t> &indices,
    Ogre::Vector3 &min, Ogre::Vector3 &max)
{
    float *data = reinterpret_cast<float*>(OGRE_MALLOC_SIMD(
        8 * vertices.size() *  sizeof(float), Ogre::MEMCATEGORY_GEOMETRY));
    std::memset(data, 0, 8 * vertices.size() *  sizeof(float));

    for (size_t i = 0, j = 0; i < vertices.size(); ++i, j += 8) {
        const OpenRAVE::Vector &v = vertices[i];
        data[j] = v[0];
        data[j + 1] = v[1];
        data[j + 2] = v[2];
        data[j + 3] = 1.0f;
        max[0] = std::max(max[0], data[j]);
        max[1] = std::max(max[1], data[j + 1]);
        max[2] = std::max(max[2], data[j + 2]);
        min[0] = std::min(min[0], data[j]);
        min[1] = std::min(min[1], data[j + 1]);
        min[2] = std::min(min[2], data[j + 2]);
    }

    for (size_t i = 0; i < indices.size(); i += 3) {
        uint32_t i0 = indices[i];
        uint32_t i1 = indices[i + 1];
        uint32_t i2 = indices[i + 2];
        const OpenRAVE::Vector &v0 = vertices[i0];
        const OpenRAVE::Vector normal = (vertices[i1] - v0).cross(vertices[i2] - v0).normalize3();

        data[i0 * 8 + 4] += normal[0];
        data[i0 * 8 + 5] += normal[1];
        data[i0 * 8 + 6] += normal[2];

        data[i1 * 8 + 4] += normal[0];
        data[i1 * 8 + 5] += normal[1];
        data[i1 * 8 + 6] += normal[2];

        data[i2 * 8 + 4] += normal[0];
        data[i2 * 8 + 5] += normal[1];
        data[i2 * 8 + 6] += normal[2];
    }

    Ogre::VertexElement2Vec vertexElements;
    vertexElements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT4, Ogre::VES_POSITION));
    vertexElements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT4, Ogre::VES_NORMAL));

    Ogre::VertexBufferPacked* packedBuffer = nullptr;
    try
    {
        packedBuffer = vaoManager->createVertexBuffer(
            vertexElements, vertices.size(),
            Ogre::BT_IMMUTABLE,
            data,
            true // True - Ogre is responsible for deallocating "ogrePoints"
        );
    }
    catch(Ogre::Exception &e)
    {
        OGRE_FREE_SIMD(packedBuffer, Ogre::MEMCATEGORY_GEOMETRY);
        throw e; // TODO: throw openrave exception
    }
    return packedBuffer;
}

/**
 * @param  ogrePoints Nx4 points allocated by OGRE_MALLOC_SIMD. It will be
 *                    deallocated by Ogre's internal mechanism
 */
static Ogre::VertexBufferPacked* CreatePointsBuffer(
    Ogre::VaoManager* vaoManager, size_t nPoints, float* ogrePoints, Ogre::VertexElementSemantic semantic) {

    Ogre::VertexElement2Vec vertexElements;
    vertexElements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT4, semantic));

    Ogre::VertexBufferPacked* vertexBuffer = nullptr;
    try
    {
        vertexBuffer = vaoManager->createVertexBuffer(
            vertexElements, nPoints,
            Ogre::BT_IMMUTABLE,
            ogrePoints,
            true // True - Ogre is responsible for deallocating "ogrePoints"
        );
    }
    catch(Ogre::Exception &e)
    {
        OGRE_FREE_SIMD(vertexBuffer, Ogre::MEMCATEGORY_GEOMETRY);
        throw e; // TODO: throw openrave exception
    }
    return vertexBuffer;
}


}; // namespace qtogrerave

#endif