
#include <osg/Node>

#ifndef OPENRAVE_QTOSG_RENDERUTILS_H
#define OPENRAVE_QTOSG_RENDERUTILS_H

class OutlineShaderPipeline {
public:
    static osg::Camera* createOutlineRenderingCamera(osg::Node* sceneRoot);
}

#endif