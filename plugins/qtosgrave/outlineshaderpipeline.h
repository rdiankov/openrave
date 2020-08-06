
#include <osg/Node>
#include <osg/Group>

#ifndef OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H
#define OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H

class OutlineShaderPipeline {
public:
    virtual ~OutlineShaderPipeline() {;}
    /// \brief This function creates a outline scene pipeline with two passes to render a regular scene with outline edges
    static OutlineShaderPipeline CreateOutlineRenderingScene(osg::ref_ptr<osg::Camera> originalSceneCamera, osg::ref_ptr<osg::Node> originalSceneRoot, int viewportWidth, int viewportHeight);

    void Resize(int width, int height);

public:
    osg::ref_ptr<osg::Group> firstPassGroup;
    osg::ref_ptr<osg::Camera> firstPassCamera;
};

#endif