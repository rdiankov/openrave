
#include <osg/Node>
#include <osg/Group>
#include <osg/Uniform>

#ifndef OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H
#define OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H

struct OutlineFirstPassState {
    osg::ref_ptr<osg::Group> firstPassGroup;
    osg::ref_ptr<osg::Camera> firstPassCamera;
};

class OutlineShaderPipeline {
public:
    OutlineShaderPipeline();
    virtual ~OutlineShaderPipeline() {;}

    /// \brief This function creates a outline scene pipeline with two passes to render a regular scene with outline edges
    void InitializeOutlinePipelineState(osg::ref_ptr<osg::Camera> originalSceneCamera, osg::ref_ptr<osg::Node> originalSceneRoot, int viewportWidth, int viewportHeight);
    void UpdateOutlineCameraFromOriginal(osg::ref_ptr<osg::Camera> originalSceneCamera, int width, int height);

public:
    OutlineFirstPassState _firstPassState;
};

#endif